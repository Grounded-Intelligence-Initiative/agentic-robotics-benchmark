"""Practice bench in YOUR DEV SIM: runs your controller over the grading wire on practice kits.

    /opt/venv/bin/python3 /home/user/practice_bench/run_bench.py [--episodes N] [--seed S ...]
        [--kit /home/user/practice_kit.json | --sample K] [--submission /home/user/submission]
        [--final-images DIR]

Runs your controller (bash run.sh in --submission) against /home/user/keyed_env.py on practice kits: the
practice kit (or any kit file you pass with --kit), or with --sample K one new kit per episode drawn
with keyed_env.sample_kit(numpy.random.default_rng(K + episode index)), from the same family as the
graded kits (never one of them). It drives your controller the way the grader does: READY within 120 s,
each reply within 30 s, repeated step indices with perturbed observations (only the reply to the real
observation is applied), every episode a NEW controller process from a fresh copy of the submission
(nothing carries over between episodes), replies longer than 64 KiB abort the episode, only AF_UNIX
sockets (the same seccomp filter as at grading time), and the per-kit controller budget (600 s: start-up
and reply time plus 0.45 s per left or right image and 0.25 s per wrist image asked for).

Like a commissioning bench, it reports what the cell itself would tell you, never a verdict: for every
episode the steps run and how the episode ended (all steps run, a safety stop: a ruined nut or an unstable
simulation, an abort of the wire with its reason, or the controller budget used up), your controller's
budget use and image count, and the wall time. It does not judge the assembly (no completion verdict, no
count of assembled nuts, no ground truth); how a kit went, you judge from what your controller saw and
recorded, or from the cameras' images of the final state (--final-images DIR saves the left, right and
wrist images at the end of every episode as PNG files). Since it does not judge completion, its episodes run
all 2400 steps unless a safety stop, an abort or the budget ends them (the grader ends an episode as soon as
its kit is complete).
"""

import argparse
import json
import os
import queue
import random
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # the agent home
import keyed_env as E  # noqa: E402

PRACTICE_SEEDS = [0x9E3779B97F4A7C15, 0xBF58476D1CE4E5B9, 0x94D049BB133111EB, 0x2545F4914F6CDD1D,
                  0xD1B54A32D192ED03, 0x8CB92BA72F3D8DD7, 0xA0761D6478BD642F, 0xE7037ED1A0B428DB]
READY_TIMEOUT = 120.0
STEP_TIMEOUT = 30.0
MAX_RESTARTS = 3
MAX_LINE = 65536
CONTROLLER_BUDGET = 600.0                                   # s per kit, as when graded
IMAGE_CHARGE = {"left": 0.45, "right": 0.45, "wrist": 0.25}  # s per image asked for, as when graded
PROBE_RATE = 0.01

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
    sys.stderr.write("[practice] could not install the network filter\n")
env = {"PATH": "/opt/venv/bin:/usr/local/bin:/usr/bin:/bin", "HOME": "/home/user", "LANG": "C.UTF-8"}
os.execvpe("bash", ["bash", "run.sh"], env)
'''


class WireError(Exception):
    pass


class Wire(object):
    def __init__(self, submission):
        self.submission = submission
        self.restarts = 0
        self.dead = False
        self.broken = False
        self._start()

    def _start(self):
        self.proc = subprocess.Popen([sys.executable, "-I", "-c", _FILTER_SRC], cwd=self.submission,
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, universal_newlines=True,
                                     bufsize=1, start_new_session=True)
        self.lines = queue.Queue()
        threading.Thread(target=self._pump, args=(self.proc, self.lines), daemon=True).start()
        deadline = time.monotonic() + READY_TIMEOUT          # one deadline for the whole handshake
        for _ in range(200):
            if self._read(READY_TIMEOUT, "READY", deadline).strip() == "READY":
                return
        raise WireError("solver never printed READY")

    @staticmethod
    def _pump(proc, lines):
        while True:
            try:
                line = proc.stdout.readline(MAX_LINE + 1)
            except (ValueError, OSError):
                break
            if not line or (len(line) > MAX_LINE and not line.endswith("\n")):
                break
            lines.put(line)
        lines.put(None)

    def _read(self, timeout, what, deadline=None):
        try:
            line = self.lines.get(timeout=timeout if deadline is None else max(0.0, deadline - time.monotonic()))
        except queue.Empty:
            raise WireError("solver exceeded its {} s {} deadline".format(timeout, what))
        if line is None:
            raise WireError("solver closed its stdout (or wrote a line over 64 KiB)")
        return line

    def stop(self):
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except OSError:
            pass
        try:
            self.proc.wait(timeout=10)
        except Exception:  # noqa: BLE001
            pass

    def request(self, msg):
        if self.dead:
            raise WireError("solver could not be restarted after an earlier abort")
        try:
            self.proc.stdin.write(json.dumps(msg) + "\n")
            self.proc.stdin.flush()
            deadline = time.monotonic() + STEP_TIMEOUT       # one deadline per request: blank lines do not extend it
            while True:
                line = self._read(STEP_TIMEOUT, "step", deadline).strip()
                if not line:
                    continue
                try:
                    reply = json.loads(line)
                except ValueError:
                    raise WireError("non-JSON output on the protocol stdout: {!r}".format(line[:80]))
                if not isinstance(reply, dict):
                    raise WireError("reply is not a JSON object")
                return reply
        except (OSError, ValueError) as exc:
            self.broken = True
            raise WireError(str(exc))
        except WireError:
            self.broken = True
            raise

    def reset(self):
        if self.broken:
            if self.restarts >= MAX_RESTARTS:
                self.dead = True
                raise WireError("restart limit reached; remaining episodes abort")
            self.restarts += 1
            self.stop()
            self.broken = False
            self._start()
        self.request({"type": "reset"})

    def act(self, t, obs):
        reply = self.request({"type": "act", "t": t, "obs": obs})
        if "action" not in reply:
            self.broken = True
            raise WireError("solver error: {}".format(reply.get("error", "no action")))
        return reply["action"]

    def close(self):
        try:
            self.request({"type": "close"})
        except Exception:  # noqa: BLE001
            pass
        self.stop()


def differ(a, b):
    try:
        aa, bb = [float(v) for v in a], [float(v) for v in b]
    except (TypeError, ValueError):
        return False
    return len(aa) != len(bb) or max([abs(x - y) for x, y in zip(aa, bb)] or [0.0]) > 1e-4


def image_charge(action):
    try:
        _mv, _g, render = E.parse_action(action)
    except (E.ActionError, TypeError, ValueError, OverflowError):
        return 0.0, 0
    return sum(IMAGE_CHARGE[c] for c in render), len(render)


def save_final_images(env, out_dir, tag):
    from PIL import Image
    os.makedirs(out_dir, exist_ok=True)
    for cam in E.CAMERA_NAMES:
        Image.fromarray(env.render(cam)).save(os.path.join(out_dir, "{}_{}.png".format(tag, cam)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--submission", default="/home/user/submission")
    ap.add_argument("--kit", default=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                                  "practice_kit.json"))
    ap.add_argument("--sample", type=int, default=None, help="draw a new kit per episode with sample_kit")
    ap.add_argument("--episodes", type=int, default=2)
    ap.add_argument("--seed", type=int, action="append", help="run these layout seeds instead of the practice list")
    ap.add_argument("--final-images", default=None, help="save the three cameras' images of every final state here")
    a = ap.parse_args()
    if not os.path.isfile(os.path.join(a.submission, "run.sh")):
        print(json.dumps({"error": "no run.sh in {}".format(a.submission)}))
        return 2
    fixed_kit = None if a.sample is not None else E.load_kit(a.kit)
    seeds = a.seed if a.seed else PRACTICE_SEEDS[:max(1, min(a.episodes, len(PRACTICE_SEEDS)))]
    rng = random.Random(2026)
    pairs = differed = 0
    results = []
    t0 = time.monotonic()
    for i, seed in enumerate(seeds):
        if fixed_kit is None:
            import numpy as np
            kit = E.sample_kit(np.random.default_rng(a.sample + i), "S%d" % (a.sample + i))
            kit_name = "sample_kit seed {}".format(a.sample + i)
        else:
            kit = fixed_kit
            kit_name = os.path.basename(a.kit)
        env = E.NutEnv(kit["nuts"], early_end=True)
        # like the grader: every episode runs a new controller process from a fresh copy of the submission
        tmp = tempfile.mkdtemp(prefix="practice-")
        work = os.path.join(tmp, "submission")
        shutil.copytree(a.submission, work, symlinks=True)
        wire = None
        spent, images, ended = 0.0, 0, "all steps run"
        ep_t0 = time.monotonic()
        try:
            obs = env.reset(seed, kit["order"])
            started = time.monotonic()
            wire = Wire(work)
            wire.reset()
            spent += time.monotonic() - started
            for t in range(env.max_steps):
                wobs = E.to_wire(obs)
                other = E.to_wire(env.perturbed_observation(obs, rng)) if rng.random() < PROBE_RATE else None
                asked = time.monotonic()
                if other is not None:
                    if rng.random() < 0.5:
                        b = wire.act(t, other)
                        act = wire.act(t, wobs)
                    else:
                        act = wire.act(t, wobs)
                        b = wire.act(t, other)
                    pairs += 1
                    differed += int(differ(act, b))
                else:
                    act = wire.act(t, wobs)
                spent += time.monotonic() - asked
                charge, n_img = image_charge(act)
                spent += charge
                if spent > CONTROLLER_BUDGET:
                    ended = "controller budget ({:.0f} s) used up at step {}".format(CONTROLLER_BUDGET, t)
                    break
                images += n_img
                obs, done, info = env.step(act)
                if done:
                    if env.fault is not None:
                        ended = "safety stop at step {}: unstable simulation ({})".format(env.t, env.fault)
                    elif info["ruined"]:
                        ended = "safety stop at step {}: nut {} ruined (pin forced through the groove wall)".format(
                            env.t, ", ".join(env.work_order()[k]["colour"] for k in info["ruined"]))
                    break
        except (WireError, E.ActionError, ValueError, TypeError) as exc:
            ended = "aborted at step {}: {}".format(env.t if env.model is not None else 0, str(exc)[:200])
        finally:
            if a.final_images and env.model is not None:
                try:
                    save_final_images(env, a.final_images, "ep{:02d}_seed{}".format(i, seed))
                except Exception as exc:  # noqa: BLE001 - the images are a convenience
                    sys.stderr.write("[practice] final images not saved: {}\n".format(exc))
            env.close()
            if wire is not None:
                wire.close()
            shutil.rmtree(tmp, ignore_errors=True)
        results.append({"seed": seed, "kit": kit_name, "order": env.work_order() if env.model is not None else None,
                        "steps": env.t if env.model is not None else 0, "ended": ended,
                        "controller_s": round(spent, 1), "controller_budget_s": CONTROLLER_BUDGET,
                        "images_requested": images, "wall_s": round(time.monotonic() - ep_t0, 1)})
    print(json.dumps({
        "bench": "dev sim, {}".format(
            "sample_kit from seed {}".format(a.sample) if a.sample is not None else os.path.basename(a.kit)),
        "episodes": results,
        "repeated_t_pairs": pairs,
        "repeated_t_pairs_that_differed": differed,
        "wall_seconds": round(time.monotonic() - t0, 1),
        "note": "a practice bench: it reports how each episode ended, never whether the kit was assembled",
    }, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
