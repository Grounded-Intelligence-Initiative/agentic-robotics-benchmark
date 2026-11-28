"""Run your submission on one practice episode of the real cell, the way the grader runs it.

    /opt/venv/bin/python3 ~/practice/run_practice.py [--submission ~/submission] [--video out.mp4] [--record rec.jsonl]

Starts `bash run.sh` in the submission folder, waits for READY (40 s), sends {"type": "reset"}, then for every control
step {"type": "act", "t": t, "obs": ...} and applies the reply's action to the practice cell, until the episode is
over, then {"type": "close"}. Like the grader, it asks about 1 % of the steps a second time with the same `t` (once
with the real observation, once with a slightly perturbed copy, in a random order; only the reply to the real one
is applied) and gives every reply 10 s. At the end it prints what the cell's end-of-line inspection reports: rings
sorted, spindles complete, rings damaged (the raw numbers of the task's metric).

--video writes the images your controller asked for (overview and wrist, side by side, the last one of each kept
until a new one arrives). --record writes one JSON line per step: t, ee_pos, ee_yaw, gripper, wrench, the action.

THE SESSION HAS ONE PRACTICE EPISODE (see /run/ringcell/practice_status.json). The episode is
spent as soon as it starts, also if your controller crashes. Practice episodes are fixed cells of their own, never a
graded episode.
"""

import argparse
import base64
import json
import os
import random
import select
import socket
import subprocess
import time

import numpy as np

SOCKET = "/run/ringcell/practice.sock"
STEP_TIMEOUT = 10.0
READY_TIMEOUT = 40.0


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


class Controller(object):
    def __init__(self, folder):
        self.proc = subprocess.Popen(["bash", "run.sh"], cwd=folder, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     text=True, bufsize=1)

    def readline(self, timeout):
        end = time.monotonic() + timeout
        while True:
            left = end - time.monotonic()
            if left <= 0:
                raise RuntimeError("your controller did not answer within %.0f s" % timeout)
            r, _, _ = select.select([self.proc.stdout], [], [], left)
            if r:
                line = self.proc.stdout.readline()
                if not line:
                    raise RuntimeError("your controller exited")
                if line.strip():
                    return line

    def ask(self, msg, timeout=STEP_TIMEOUT):
        self.proc.stdin.write(json.dumps(msg) + "\n")
        self.proc.stdin.flush()
        return json.loads(self.readline(timeout))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--submission", default=os.path.expanduser("~/submission"))
    ap.add_argument("--video", default="", help="write the images of the episode to this .mp4")
    ap.add_argument("--record", default="", help="write one JSON line per step (readings and action) to this file")
    a = ap.parse_args()
    ctl = Controller(a.submission)
    t0 = time.time()
    while ctl.readline(READY_TIMEOUT).strip() != "READY":
        pass
    print("READY after %.1fs" % (time.time() - t0))
    cell = PracticeCell()
    r = cell.call("reset")
    print("practice episodes left after this one:", r["episodes_left"])
    obs, done, frames, rec = r["obs"], False, [], None
    last_img = {}
    if a.record:
        rec = open(a.record, "w")
    ctl.ask({"type": "reset"})
    rnd = random.Random()
    probes = differ = 0
    try:
        while not done:
            if a.video:
                for cam in ("overview", "wrist"):
                    if obs[cam]["shape"][0] > 0:
                        last_img[cam] = decode(obs[cam])
                if last_img:
                    frames.append(dict(last_img))
            if rnd.random() < 0.01:
                other = cell.call("probe")["obs"]
                pair = [obs, other]
                rnd.shuffle(pair)
                reps = [ctl.ask({"type": "act", "t": obs["t"], "obs": o}) for o in pair]
                rep = reps[0] if pair[0] is obs else reps[1]
                probes += 1
                differ += int(reps[0].get("action") != reps[1].get("action"))
            else:
                rep = ctl.ask({"type": "act", "t": obs["t"], "obs": obs})
            if rec is not None:
                rec.write(json.dumps({"t": obs["t"], "ee_pos": obs["ee_pos"], "ee_yaw": obs["ee_yaw"],
                                      "gripper": obs["gripper"], "wrench": obs["wrench"],
                                      "action": rep.get("action")}) + "\n")
            r = cell.call("step", action=rep.get("action"))
            obs, done, info = r["obs"], r["done"], r["info"]
            if obs["t"] % 200 == 0:
                print("t=%d" % obs["t"], flush=True)
            if info.get("fault"):
                print("simulation fault:", info["fault"])
    except RuntimeError as exc:
        print("episode aborted:", exc)
    res = cell.call("close")
    try:
        ctl.ask({"type": "close"})
    except Exception:  # noqa: BLE001
        pass
    ctl.proc.terminate()
    print("repeated-t queries: %d, replies differed in %d" % (probes, differ))
    print("end of episode at t=%d: %d of 24 rings sorted, %d of 6 spindles complete, %d rings damaged" % (
        res["t"], res["rings_sorted"], res["spindles_complete"], res["rings_damaged"]))
    if a.video and frames:
        import cv2
        h = 180
        vw = None
        for fr in frames:
            tiles = []
            for cam, size in (("overview", (240, 180)), ("wrist", (240, 180))):
                img = fr.get(cam)
                img = np.zeros((size[1], size[0], 3), np.uint8) if img is None else cv2.resize(img, size)
                tiles.append(img)
            im = np.concatenate(tiles, 1)
            if vw is None:
                vw = cv2.VideoWriter(a.video, cv2.VideoWriter_fourcc(*"mp4v"), 10, (im.shape[1] * 2, h * 2))
            vw.write(cv2.cvtColor(cv2.resize(im, (im.shape[1] * 2, h * 2), interpolation=cv2.INTER_NEAREST),
                                  cv2.COLOR_RGB2BGR))
        vw.release()
        print("video:", a.video)


if __name__ == "__main__":
    main()
