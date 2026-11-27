"""Runs YOUR controller (bash run.sh) in the graded episode, over the grading wire.

    /opt/venv/bin/python3 /home/user/graded/run_graded.py [--start] [--steps N] [--finish]
        [--submission /home/user/submission] [--record FILE]

With --start it first starts the graded episode (once per session: that freezes the submission folder as it is);
without it, it continues the episode where it stands, from its latest observation. The controller is started from a
fresh copy of --submission (with only AF_UNIX sockets, as at grading time) and gets `reset`, then `act` with the
episode's own step index (not 0 when it continues an episode). The wire's deadlines apply (READY within 120 s, each
reply within 30 s, a reply line at most 4 MiB); a missed one stops the run with nothing applied for that step, and the
episode stays where it is. There is no time budget, no restart and no repeated-step query. It stops when the episode
is over, after --steps N steps, or on a wire failure; with --finish it then ends the graded episode. It prints the
steps it ran and the episode's status (no score). --record FILE appends every observation's readings (no images) and
every applied action as JSON lines.
"""

import argparse
import json
import os
import queue
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from client import GradedError, connect_graded  # noqa: E402

READY_TIMEOUT = 120.0
STEP_TIMEOUT = 30.0
MAX_LINE = 1 << 22
CAMS = ("overhead", "front", "tray", "wrist_l", "wrist_r")

_FILTER_SRC = r'''
import ctypes, os, platform, struct, sys
ARCH = {"x86_64": (0xC000003E, 41, 53, 425, 0x40000000), "aarch64": (0xC00000B7, 198, 199, 425, 0)}
def ins(c, jt, jf, k): return struct.pack("HBBI", c, jt, jf, k)
audit, ns, nsp, nu, x32 = ARCH[platform.machine()]
deny = 0x00050000 | 1; allow = 0x7FFF0000
p = [ins(0x20,0,0,4), ins(0x15,1,0,audit), ins(0x06,0,0,deny), ins(0x20,0,0,0)]
if x32: p += [ins(0x35,0,1,x32), ins(0x06,0,0,deny)]
p += [ins(0x15,0,1,nu), ins(0x06,0,0,deny), ins(0x15,2,0,ns), ins(0x15,1,0,nsp), ins(0x06,0,0,allow),
      ins(0x20,0,0,16), ins(0x15,0,1,1), ins(0x06,0,0,allow), ins(0x06,0,0,deny)]
prog = b"".join(p); buf = ctypes.create_string_buffer(prog, len(prog))
class F(ctypes.Structure): _fields_ = [("len", ctypes.c_ushort), ("filter", ctypes.c_void_p)]
f = F(len(prog) // 8, ctypes.cast(buf, ctypes.c_void_p)); libc = ctypes.CDLL(None, use_errno=True)
if libc.prctl(38, 1, 0, 0, 0) != 0 or libc.prctl(22, 2, ctypes.byref(f), 0, 0) != 0:
    sys.stderr.write("[graded] could not install the network filter\n")
env = {"PATH": "/opt/venv/bin:/usr/local/bin:/usr/bin:/bin", "HOME": "/home/user", "LANG": "C.UTF-8"}
os.execvpe("bash", ["bash", "run.sh"], env)
'''


class WireError(Exception):
    pass


class Wire(object):
    """The controller's side of the grading wire (READY / reset / act / close), with the grader's deadlines."""

    def __init__(self, submission):
        self.proc = subprocess.Popen([sys.executable, "-I", "-c", _FILTER_SRC], cwd=submission,
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, universal_newlines=True,
                                     bufsize=1, start_new_session=True)
        self.lines = queue.Queue()
        threading.Thread(target=self._pump, daemon=True).start()
        deadline = time.monotonic() + READY_TIMEOUT
        for _ in range(200):
            if self._read(deadline, "READY").strip() == "READY":
                return
        raise WireError("solver never printed READY")

    def _pump(self):
        while True:
            try:
                line = self.proc.stdout.readline(MAX_LINE + 1)
            except (ValueError, OSError):
                break
            if not line or (len(line) > MAX_LINE and not line.endswith("\n")):
                break
            self.lines.put(line)
        self.lines.put(None)

    def _read(self, deadline, what):
        try:
            line = self.lines.get(timeout=max(0.0, deadline - time.monotonic()))
        except queue.Empty:
            raise WireError("solver missed its {} deadline".format(what))
        if line is None:
            raise WireError("solver closed its stdout (or wrote a line over 4 MiB)")
        return line

    def request(self, msg):
        try:
            self.proc.stdin.write(json.dumps(msg) + "\n")
            self.proc.stdin.flush()
        except (OSError, ValueError) as exc:
            raise WireError(str(exc))
        deadline = time.monotonic() + STEP_TIMEOUT
        while True:
            line = self._read(deadline, "step").strip()
            if not line:
                continue
            try:
                reply = json.loads(line)
            except ValueError:
                raise WireError("non-JSON output on the protocol stdout: {!r}".format(line[:80]))
            if not isinstance(reply, dict):
                raise WireError("reply is not a JSON object")
            return reply

    def act(self, t, obs):
        reply = self.request({"type": "act", "t": t, "obs": obs})
        if "action" not in reply:
            raise WireError("solver error: {}".format(reply.get("error", "no action")))
        return reply["action"]

    def stop(self, polite=True):
        if polite:
            try:
                self.request({"type": "close"})
            except Exception:  # noqa: BLE001
                pass
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except OSError:
            pass
        try:
            self.proc.wait(timeout=10)
        except Exception:  # noqa: BLE001
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", action="store_true", help="start the graded episode first (once per session)")
    ap.add_argument("--steps", type=int, default=None, help="stop after this many steps")
    ap.add_argument("--finish", action="store_true", help="end the graded episode when the run stops")
    ap.add_argument("--submission", default="/home/user/submission")
    ap.add_argument("--record", default=None, help="append the observations' readings and the actions here")
    a = ap.parse_args()
    if not os.path.isfile(os.path.join(a.submission, "run.sh")):
        print(json.dumps({"error": "no run.sh in {}".format(a.submission)}))
        return 2
    tmp = tempfile.mkdtemp(prefix="graded-")
    work = os.path.join(tmp, "submission")
    shutil.copytree(a.submission, work, symlinks=True)
    wire = g = rec = None
    ran, stopped = 0, None
    t0 = time.monotonic()
    try:
        g = connect_graded()
        if a.start:
            g.start()
        else:
            g.observe()                                      # where the episode stands (GradedError if not started)
        obs = g.obs
        wire = Wire(work)
        wire.request({"type": "reset"})
        rec = open(a.record, "a") if a.record else None
        while not g.done and obs is not None and (a.steps is None or ran < a.steps):
            t = int(obs["t"])
            act = wire.act(t, obs)
            if rec is not None:
                rec.write(json.dumps({"t": t, "obs": {k: v for k, v in obs.items() if k not in CAMS},
                                      "action": act}) + "\n")
                rec.flush()
            obs = g.step(act)
            ran += 1
    except WireError as exc:
        stopped = "the controller's wire failed: {}".format(str(exc)[:200])
    except GradedError as exc:
        stopped = "the graded episode's service refused: {}".format(exc)
    finally:
        if rec is not None:
            rec.close()
        if wire is not None:
            wire.stop(polite=stopped is None)
        st = None
        if g is not None:
            try:
                if a.finish and not g.done:
                    g.finish()
                st = g.status()
            except (GradedError, OSError):
                pass
            g.close()
        shutil.rmtree(tmp, ignore_errors=True)
    print(json.dumps({"steps_run": ran, "stopped": stopped, "status": st, "wall_s": round(time.monotonic() - t0, 1),
                      "note": "the graded episode: this reports where it stands, never its credit"}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
