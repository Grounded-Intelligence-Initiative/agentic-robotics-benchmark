"""Run your submission on one practice episode of the real cell, the way the grader drives it.

    /opt/venv/bin/python3 ~/practice/run_practice.py [--submission ~/submission] [--frames DIR] [--every 10]

Starts `bash run.sh` in the submission folder, waits for READY (120 s, as the grader does), sends {"type": "reset"},
then for every control step {"type": "act", "t": t, "obs": ...} and applies the reply's action to the practice cell,
until the episode ends; then {"type": "close"}. Each reply must come within 30 s, as in the graded run. At the end it
prints what the cell reports: the steps taken, how many of the 8 mates hold a part of the kit, and whether the car
passed the roll test.

Each practice episode builds its own car of the family on a fixed practice layout, with the sibling sets mixed in,
like a graded episode. Its manual arrives with the observation at t = 0 (and only then), as in a graded episode; the
car of practice episode k (k = 1, 2, 3) is ~/examples/car_k (numbers in kit.json, manual/ pictures).

EACH RUN SPENDS ONE OF THE SESSION'S 3 PRACTICE EPISODES (see /run/toycell/practice_status.json). The episode is
spent as soon as it starts, also if your controller crashes.

Repeated t: a few steps are asked twice with the same t and the SAME observation (the grader's second copy is
perturbed; see the instruction). This tool checks that the two replies are identical, i.e. that your controller does
not advance its state twice for one t, and applies the first.

--frames DIR saves the overview and wrist images every --every steps as PNG files.
"""

import argparse
import base64
import json
import os
import queue
import random
import socket
import subprocess
import threading
import time

import numpy as np

SOCKET = "/run/toycell/practice.sock"
STEP_TIMEOUT_S = 30.0
READY_TIMEOUT_S = 120.0
REPEAT_RATE = 0.01


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


class Controller(object):
    """Your run.sh over stdio, with the grader's deadlines."""

    def __init__(self, submission):
        self.proc = subprocess.Popen(["bash", "run.sh"], cwd=submission, stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, text=True, bufsize=1)
        self.lines = queue.Queue()
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        for line in self.proc.stdout:
            self.lines.put(line)
        self.lines.put(None)

    def readline(self, timeout):
        try:
            line = self.lines.get(timeout=timeout)
        except queue.Empty:
            raise SystemExit("no reply within %.0f s: the graded run aborts the episode here" % timeout)
        if line is None:
            raise SystemExit("your controller closed its stdout (did it crash? see its stderr)")
        return line

    def wait_ready(self):
        t0 = time.time()
        while True:
            left = READY_TIMEOUT_S - (time.time() - t0)
            if left <= 0:
                raise SystemExit("no READY within %.0f s" % READY_TIMEOUT_S)
            if self.readline(left).strip() == "READY":
                return time.time() - t0

    def ask(self, msg):
        self.proc.stdin.write(json.dumps(msg) + "\n")
        self.proc.stdin.flush()
        t0 = time.time()
        while True:
            left = STEP_TIMEOUT_S - (time.time() - t0)
            if left <= 0:
                raise SystemExit("no reply within %.0f s: the graded run aborts the episode here" % STEP_TIMEOUT_S)
            line = self.readline(left).strip()
            if not line:
                continue
            try:
                return json.loads(line)
            except ValueError:
                raise SystemExit("non-JSON on stdout: %r (stdout is the wire; log to stderr)" % line[:120])

    def close(self):
        try:
            self.ask({"type": "close"})
        except BaseException:  # noqa: BLE001
            pass
        try:
            self.proc.wait(timeout=10)
        except Exception:  # noqa: BLE001
            self.proc.kill()


def decode(d):
    return np.frombuffer(base64.b64decode(d["b64"]), np.uint8).reshape(d["shape"])


def save_frames(obs, t, frames):
    from PIL import Image
    for cam in ("overview", "wrist"):
        Image.fromarray(decode(obs[cam])).save(os.path.join(frames, "%s_%04d.png" % (cam, t)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--submission", default=os.path.expanduser("~/submission"))
    ap.add_argument("--frames", default="", help="save the camera images here")
    ap.add_argument("--every", type=int, default=10)
    a = ap.parse_args()
    if not os.path.isfile(os.path.join(a.submission, "run.sh")):
        raise SystemExit("no run.sh in %s" % a.submission)
    if a.frames:
        os.makedirs(a.frames, exist_ok=True)
    ctl = Controller(a.submission)
    print("READY after %.1f s" % ctl.wait_ready(), flush=True)
    cell = PracticeCell()
    r = cell.call("reset")
    print("practice episodes left after this one:", r["episodes_left"], flush=True)
    obs, done = r["obs"], False
    rng = random.Random(2026)
    repeats = same = 0
    ctl.ask({"type": "reset"})
    t = 0
    try:
        while not done:
            if a.frames and t % a.every == 0:
                save_frames(obs, t, a.frames)
            rep = ctl.ask({"type": "act", "t": t, "obs": obs})
            if "action" not in rep:
                raise SystemExit("reply without an action at t=%d: %s" % (t, rep))
            if rng.random() < REPEAT_RATE:
                rep2 = ctl.ask({"type": "act", "t": t, "obs": obs})
                repeats += 1
                same += int(rep2.get("action") == rep["action"])
            r = cell.call("step", action=rep["action"])
            obs, done = r["obs"], r["done"]
            t += 1
            if t % 200 == 0:
                print("t=%d" % t, flush=True)
    finally:
        res = cell.call("close")
        ctl.close()
    print(json.dumps({"steps": res["steps"], "mates_with_kit_parts": res["mates_with_kit_parts"],
                      "rolls": res["rolls"], "repeated_t_with_the_same_obs": repeats,
                      "identical_replies_to_those": same, "note": "practice car and layout only"}))


if __name__ == "__main__":
    main()
