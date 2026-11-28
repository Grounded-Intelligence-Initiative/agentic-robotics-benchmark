"""Run your submission on one practice shift of the real cell, the way the grader runs it.

    /opt/venv/bin/python3 ~/practice/run_practice.py [--submission ~/submission] [--video out.mp4] [--log out.jsonl]

Starts `bash run.sh` in the submission folder, waits for READY, sends {"type": "reset"}, then for every control step
{"type": "act", "t": t, "obs": ...} and applies the reply's action to the practice cell, until the shift is over,
then {"type": "close"}. Prints what the kit inspection reports per pocket and how many parts were lost.

Timing as in grading: the practice cell itself does not wait (see client.py). Each control step has 0.1 s of
wall-clock time from when the cell sends an observation until the next step request reaches it; for every whole
0.1 s a reply took, the cell first runs one hold step (no motion, the gripper as last commanded, no images, no
turnover request), and your controller never sees those steps' observations. This tool adds only the relay to your
controller (a few milliseconds) and reports the reply times and hold steps. Like the grader, it aborts the shift on a
reply later than 5 s. (The practice cell runs on this machine, the agent sandbox; the grader measures on its own
4-CPU verifier.)

EACH RUN USES ONE OF THE SESSION'S PRACTICE SHIFTS (see /run/kitcell/practice_status.json). The shift is spent as
soon as it starts, also if your controller crashes. Practice shifts are fixed cell layouts, never a graded shift.
Unlike the grader, this tool does not repeat any step (the grader's repeated-`t` queries are explained in the task
instruction).
"""

import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from client import PracticeCell, decode  # noqa: E402

REPLY_TIMEOUT_S = 5.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--submission", default=os.path.expanduser("~/submission"))
    ap.add_argument("--video", default="", help="write the images your controller received to this .mp4")
    ap.add_argument("--log", default="", help="write every observation (without images) and action to this .jsonl")
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
    print("practice shifts left after this one:", r["shifts_left"])
    obs, done, frames = r["obs"], False, []
    logf = open(a.log, "w") if a.log else None
    ask({"type": "reset"})
    late, late_steps, slowest, n = 0, 0, 0.0, 0
    try:
        while not done:
            if a.video:
                ims = [decode(obs[c]) for c in ("overview", "wrist", "inspect") if obs[c]["shape"][0] > 0]
                if ims:
                    frames.append(ims)
            t1 = time.monotonic()
            rep = ask({"type": "act", "t": obs["t"], "obs": obs})
            spent = time.monotonic() - t1
            n += 1
            slowest = max(slowest, spent)
            if spent > REPLY_TIMEOUT_S:
                print("reply later than %.0f s at t=%d: the shift is aborted" % (REPLY_TIMEOUT_S, obs["t"]))
                break
            action = rep["action"]
            rr = cell.call("step", action=action)
            k = int(rr["info"].get("hold_steps") or 0)
            if k:
                late += 1
                late_steps += k
            if logf:
                logf.write(json.dumps({"t": obs["t"], "action": action, "reply_s": round(spent, 4),
                                       "obs": {k2: v for k2, v in obs.items() if k2 not in ("overview", "wrist", "inspect")}}) + "\n")
            obs, done, info = rr["obs"], rr["done"], rr["info"]
            if done and info.get("ended") and info["ended"] != "the shift is over":
                print("the cell ended the shift:", info["ended"])
            if obs["t"] % 200 == 0:
                print("t=%d time=%.1fs" % (obs["t"], obs["time"]), flush=True)
    finally:
        res = cell.call("close")
        try:
            ask({"type": "close"})
        except Exception:  # noqa: BLE001
            pass
        proc.terminate()
    print("kit inspection:", res["kit"], "-> %d of %d pockets seated; %d parts lost" % (
        sum(1 for s in res["kit"] if s == "seated"), len(res["kit"]), res["lost"]))
    print("replies %d, late %d (%d hold steps), slowest %.3f s" % (n, late, late_steps, slowest))
    if a.video and frames:
        import cv2
        H = 360
        vw = None
        for ims in frames:
            row = [cv2.resize(im, (int(im.shape[1] * H / im.shape[0]), H), interpolation=cv2.INTER_NEAREST) for im in ims]
            f = np.concatenate(row, 1)
            if vw is None:
                W = 3 * 480
                vw = cv2.VideoWriter(a.video, cv2.VideoWriter_fourcc(*"mp4v"), 10, (W, H))
            canvas = np.zeros((H, 3 * 480, 3), np.uint8)
            canvas[:, :min(f.shape[1], 3 * 480)] = f[:, :3 * 480]
            vw.write(cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR))
        if vw is not None:
            vw.release()
            print("video:", a.video)


if __name__ == "__main__":
    main()
