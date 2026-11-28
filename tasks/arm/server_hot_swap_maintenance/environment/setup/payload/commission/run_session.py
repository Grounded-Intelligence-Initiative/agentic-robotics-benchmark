"""Run your controller for one commissioning session on a development unit (each session counts when it opens).

    /opt/venv/bin/python3 ~/commission/run_session.py --status
    /opt/venv/bin/python3 ~/commission/run_session.py --unit d1 [--submission ~/submission] [--log FILE]

It starts `bash run.sh` in the submission folder the way the grader does (with only PATH, HOME, LANG and the numerics
variables set) and speaks the same wire to it: READY within 120 s, then {"type": "reset"}, then one
{"type": "act", "t": t, "obs": {...}} per step (an answer within 30 s), then {"type": "close"}. The observations come
from the commissioning cell (the graded cell's build on a development unit); your controller's actions go back to it.
No step is asked twice here (the grader's probe pairs are not part of a session). At the end it prints the cell's
message (why the episode ended), the steps and your controller's time. The log (--log, default
~/commission/logs/session_<time>.jsonl) has one line per step: t, both arms' joint angles, tool poses, wrenches and
gripper openings, and the action; no frames (save the ones you want from inside your controller, to stderr or files).
"""

import argparse
import json
import os
import select
import socket
import subprocess
import sys
import time

SOCKET = "/run/srvcell/commission.sock"
NUMERICS = {"OPENBLAS_CORETYPE": "Haswell", "OPENBLAS_L2_SIZE": "256", "OPENBLAS_NUM_THREADS": "1",
            "OMP_NUM_THREADS": "1", "NPY_DISABLE_CPU_FEATURES": "X86_V4 AVX512_ICL AVX512_SPR", "OPENCV_IPP": "avx2"}


class Line(object):
    def __init__(self, sock):
        self.sock, self.buf = sock, b""

    def read(self, timeout=600.0):
        deadline = time.time() + timeout
        while b"\n" not in self.buf:
            left = deadline - time.time()
            if left <= 0:
                raise RuntimeError("the commissioning service did not answer")
            r, _, _ = select.select([self.sock], [], [], left)
            if not r:
                continue
            chunk = self.sock.recv(1 << 20)
            if not chunk:
                raise RuntimeError("the commissioning service closed the connection")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line.decode("utf-8"))

    def send(self, obj):
        self.sock.sendall((json.dumps(obj) + "\n").encode("utf-8"))


def connect():
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(SOCKET)
    return s, Line(s)


def ctl_read(proc, timeout):
    r, _, _ = select.select([proc.stdout], [], [], timeout)
    if not r:
        raise RuntimeError("the controller did not answer within %.0f s" % timeout)
    line = proc.stdout.readline()
    if not line:
        raise RuntimeError("the controller exited")
    return line


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--unit")
    ap.add_argument("--status", action="store_true")
    ap.add_argument("--submission", default=os.path.expanduser("~/submission"))
    ap.add_argument("--log")
    a = ap.parse_args()
    if a.status:
        s, c = connect()
        c.send({"type": "open", "status": True})
        print(json.dumps(c.read(30.0)))
        return
    if not a.unit:
        sys.exit("--unit is required (see --status for the development units)")
    entry = os.path.join(a.submission, "run.sh")
    if not os.path.isfile(entry):
        sys.exit("no run.sh in %s" % a.submission)
    logdir = os.path.expanduser("~/commission/logs")
    os.makedirs(logdir, exist_ok=True)
    logpath = a.log or os.path.join(logdir, "session_%s.jsonl" % time.strftime("%Y%m%d_%H%M%S"))
    env = dict(NUMERICS, PATH="/opt/venv/bin:/usr/local/bin:/usr/bin:/bin", HOME=os.path.expanduser("~"),
               LANG="C.UTF-8")
    proc = subprocess.Popen(["bash", entry], cwd=a.submission, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            universal_newlines=True, bufsize=1, env=env, start_new_session=True)
    ctl_s = 0.0
    t0 = time.time()
    try:
        while True:
            line = ctl_read(proc, 120.0 - (time.time() - t0))
            if line.strip() == "READY":
                break
        ctl_s += time.time() - t0
        s, c = connect()
        c.send({"type": "open", "unit": a.unit})
        hello = c.read(120.0)
        if not hello.get("ok"):
            sys.exit("not opened: %s" % hello.get("error"))
        print("session %d on unit %s (%d left after this one)" % (hello["session"], a.unit, hello["sessions_left"]),
              file=sys.stderr)
        t1 = time.time()
        proc.stdin.write(json.dumps({"type": "reset"}) + "\n")
        proc.stdin.flush()
        ctl_read(proc, 30.0)
        ctl_s += time.time() - t1
        msg = c.read()
        t = 0
        with open(logpath, "w") as lg:
            while True:
                obs = msg["obs"]
                t1 = time.time()
                proc.stdin.write(json.dumps({"type": "act", "t": t, "obs": obs}) + "\n")
                proc.stdin.flush()
                reply = json.loads(ctl_read(proc, 30.0))
                ctl_s += time.time() - t1
                action = reply.get("action") if isinstance(reply, dict) else None
                rec = {"t": t, "action": action}
                for s_ in ("l", "r"):
                    rec[s_] = {k: obs[s_][k] for k in ("q", "ee_pos", "ee_quat", "wrench", "gripper")}
                lg.write(json.dumps(rec) + "\n")
                c.send({"action": action})
                msg = c.read()
                t += 1
                if "over" in msg:
                    print(json.dumps({"over": msg["over"], "steps": t, "controller_seconds": round(ctl_s, 1),
                                      "log": logpath}))
                    break
    finally:
        try:
            proc.stdin.write(json.dumps({"type": "close"}) + "\n")
            proc.stdin.flush()
            proc.wait(timeout=10)
        except Exception:  # noqa: BLE001
            try:
                os.killpg(proc.pid, 9)
            except OSError:
                pass


if __name__ == "__main__":
    main()
