"""Run your submission on one practice run of the real cell (4 throws), the way the grader runs it.

    /opt/venv/bin/python3 ~/practice/run_practice.py [--submission ~/submission] [--video out.mp4] [--log out.jsonl]

Starts `bash run.sh` in the submission folder, waits for READY, sends {"type": "reset"}, then for every control step
{"type": "act", "t": t, "obs": ...} and applies the reply's action to the practice cell, until the run is over, then
{"type": "close"}. Prints, per throw, the cell's own judgement of the throw (caught and held, credit and its parts).

The world keeps moving, as in grading: each reply is timed from the moment the observation is written until the reply
has been read. A reply within one control period (20 ms) acts at this step; a slower one acts ceil(elapsed / 20 ms) - 1
steps later (those steps keep the previous action, and their observations are never shown to your controller); a
reply later than 5 periods is discarded and 5 steps run with the previous action. The practice machine is not the
grading machine: compare your reply times (printed at the end) with the budget, with margin.

EACH RUN USES ONE OF THE SESSION'S 4 PRACTICE RUNS (see /run/dexcell/practice_status.json). The run is spent as soon
as it starts, also if your controller crashes. Practice runs are fixed throws, never a graded throw. Unlike the
grader, this tool never repeats a step (the grader's repeated-`t` queries are explained in the task instruction).
"""

import argparse
import base64
import json
import math
import os
import socket
import subprocess
import time

import numpy as np

SOCKET = "/run/dexcell/practice.sock"
PERIOD = 0.02
LATE_LIMIT = 5


class PracticeCell(object):
    def __init__(self, path=SOCKET):
        self.s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.s.connect(path)
        self.rf = self.s.makefile("rb")
        self.wf = self.s.makefile("wb")

    def call(self, method, **params):
        self.wf.write((json.dumps({"method": method, "params": params}) + "\n").encode())
        self.wf.flush()
        line = self.rf.readline()
        if not line:
            raise RuntimeError("the practice cell closed the connection")
        rep = json.loads(line)
        if "error" in rep:
            raise RuntimeError(rep["error"])
        return rep["result"]


def decode(d):
    return np.frombuffer(base64.b64decode(d["b64"]), np.uint8).reshape(d["shape"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--submission", default=os.path.expanduser("~/submission"))
    ap.add_argument("--video", default="", help="write both cameras of the run to this .mp4 (50 fps)")
    ap.add_argument("--log", default="", help="write every step's proprioception and your action to this .jsonl")
    a = ap.parse_args()
    proc = subprocess.Popen(["bash", "run.sh"], cwd=a.submission, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            text=True, bufsize=1)

    def ask(msg):
        proc.stdin.write(json.dumps(msg) + "\n")
        proc.stdin.flush()
        while True:
            line = proc.stdout.readline()
            if not line:
                raise RuntimeError("your controller exited")
            if line.strip():
                return json.loads(line)

    t0 = time.time()
    while True:
        line = proc.stdout.readline()
        if not line:
            raise SystemExit("your controller exited before READY")
        if line.strip() == "READY":
            break
    print("READY after %.1fs" % (time.time() - t0))
    cell = PracticeCell()
    r = cell.call("reset")
    print("practice runs left after this one:", r["runs_left"])
    obs, done, frames, times, late, missed = r["obs"], False, [], [], 0, 0
    logf = open(a.log, "w") if a.log else None
    ask({"type": "reset"})
    prev = None
    k = 0
    while not done:
        if a.video:
            frames.append(np.concatenate([decode(obs["img_side"]), decode(obs["img_head"])], 1))
        s = time.monotonic()
        rep = ask({"type": "act", "t": k, "obs": obs})
        el = time.monotonic() - s
        times.append(el)
        action = rep["action"]
        if prev is None:
            prev = action  # the first step: nothing earlier to hold
        n_late = max(0, int(math.ceil(el / PERIOD - 1e-9)) - 1)
        if n_late >= LATE_LIMIT:
            missed += 1
            steps = [prev] * LATE_LIMIT
        else:
            late += n_late > 0
            steps = [prev] * n_late + [action]
            prev = action
        if logf:
            logf.write(json.dumps({"t": k, "obs": {kk: v for kk, v in obs.items() if not kk.startswith("img_")},
                                   "action": action, "reply_s": el}) + "\n")
        for act in steps:
            r = cell.call("step", action=act)
            k += 1
            if r["throw"] is not None:
                print("throw:", json.dumps(r["throw"]), flush=True)
            done = r["done"]
            if done:
                break
            obs = r["obs"]
    res = cell.call("close")
    try:
        ask({"type": "close"})
    except Exception:  # noqa: BLE001
        pass
    proc.terminate()
    credits = [t["credit"] for t in res["throws"]]
    print("credits:", credits, "-> mean %.3f" % (sum(credits) / max(len(credits), 1)))
    tt = np.array(times) * 1e3
    print("reply time: p50 %.1f ms, p99 %.1f ms, max %.1f ms; late replies %d, missed %d"
          % (np.percentile(tt, 50), np.percentile(tt, 99), tt.max(), late, missed))
    if a.video and frames:
        import cv2
        h, w = frames[0].shape[:2]
        vw = cv2.VideoWriter(a.video, cv2.VideoWriter_fourcc(*"mp4v"), 50, (w * 2, h * 2))
        for f in frames:
            vw.write(cv2.cvtColor(cv2.resize(f, (w * 2, h * 2), interpolation=cv2.INTER_NEAREST), cv2.COLOR_RGB2BGR))
        vw.release()
        print("video:", a.video)


if __name__ == "__main__":
    main()
