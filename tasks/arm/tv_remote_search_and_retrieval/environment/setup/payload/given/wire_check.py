"""Checks a controller against the grading wire, without a room.

    /opt/venv/bin/python3 ~/given/wire_check.py [--submission ~/submission] [--steps 40]

It starts `bash run.sh` in the submission folder the way the grader does, waits for READY, sends
`reset` and then `act` requests, and checks every reply: one JSON object with an `action` of 10
finite numbers, the last an integer from 0 to 3, inside the deadlines. Some steps are asked twice
with the same `t`, as the grader does. It then restarts the controller once.

There is no simulation behind it. Every observation is the recorded first observation of a
survey site (survey/site_1/observation.json) with only `t` changed: the robot never moves, and
images are attached whenever the previous action asked for them. It tells you that the protocol
works, never how a controller would do in the room; it prints no score.
"""

import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
READY_S, STEP_S = 120.0, 30.0


class Wire(object):
    def __init__(self, folder):
        self.proc = subprocess.Popen(["bash", "run.sh"], cwd=folder, stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, text=True, bufsize=1)

    def line(self, limit):
        box = []
        reader = threading.Thread(target=lambda: box.append(self.proc.stdout.readline()), daemon=True)
        reader.start()
        reader.join(limit)
        if reader.is_alive() or not box or not box[0]:
            raise RuntimeError("no reply within {:.0f} s (or the controller exited)".format(limit))
        return box[0].strip()

    def ask(self, message, limit=STEP_S):
        started = time.monotonic()
        self.proc.stdin.write(json.dumps(message) + "\n")
        self.proc.stdin.flush()
        while True:
            text = self.line(max(0.01, limit - (time.monotonic() - started)))
            if text:
                return json.loads(text), time.monotonic() - started

    def stop(self):
        try:
            self.ask({"type": "close"}, 10.0)
        except Exception:
            pass
        try:
            self.proc.wait(5)
        except subprocess.TimeoutExpired:
            self.proc.kill()


def check_action(reply):
    action = reply.get("action") if isinstance(reply, dict) else None
    if not isinstance(action, list) or len(action) != 10:
        raise RuntimeError("the reply needs an `action` of 10 numbers: {}".format(str(reply)[:120]))
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in action):
        raise RuntimeError("an action holds a value that is not a finite number")
    if action[9] not in (0, 1, 2, 3):
        raise RuntimeError("the camera mask (last number) must be 0, 1, 2 or 3")
    return action


def one_run(folder, base, steps):
    started = time.monotonic()
    wire = Wire(folder)
    try:
        if wire.line(READY_S) != "READY":
            raise RuntimeError("the first line must be READY")
        print("READY after {:.1f} s".format(time.monotonic() - started))
        reply, _ = wire.ask({"type": "reset"})
        if reply != {"ok": True}:
            raise RuntimeError("reset must be answered with {\"ok\": true}")
        mask, slowest, asked = 3, 0.0, 0
        for t in range(steps):
            obs = dict(base, t=t)
            obs["images"] = {name: base["images"][name] for name, bit in (("overview", 1), ("wrist", 2)) if mask & bit}
            asked += len(obs["images"])
            for _ in range(2 if t % 13 == 5 else 1):          # a step asked twice now and then
                reply, took = wire.ask({"type": "act", "t": t, "obs": obs})
                action = check_action(reply)
                slowest = max(slowest, took)
            mask = int(action[9])
        print("{} steps answered, slowest reply {:.2f} s, {} images delivered".format(steps, slowest, asked))
    finally:
        wire.stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--submission", default=os.path.expanduser("~/submission"))
    parser.add_argument("--steps", type=int, default=40)
    args = parser.parse_args()
    with open(os.path.join(HERE, "survey", "site_1", "observation.json")) as handle:
        base = json.load(handle)
    try:
        one_run(args.submission, base, args.steps)
        one_run(args.submission, base, 3)                       # run.sh has to work when started again
    except Exception as exc:
        print("WIRE CHECK FAILED: {}".format(exc))
        sys.exit(1)
    print("wire check passed (protocol only: nothing was simulated)")


if __name__ == "__main__":
    main()
