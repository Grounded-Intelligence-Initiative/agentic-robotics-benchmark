#!/opt/venv/bin/python3
"""Play practice runs to your /home/user/submission/run.sh over the same wire as the real grading run, and print what
happened in each run and the raw P, F and per-type fractions. It reports no score.

    /opt/venv/bin/python3 /home/user/practice_grader/grade.py [--limit N] [--kind failure|success] [--practice DIR]

Same playback as grading: chunks of 10 frames from frame 0; a run stops at key.json `stream_frames` (a failed run
3.0-5.0 s after its onset, a successful run 3.0-5.0 s before the task was complete, a fixed draw per run), or at the
first alarm. Same limits: 30 s per reply,
READY within 120 s, your program's own time over a run's chunks at most 20 s more than the time shown (the reset and any
restart are not counted), restart after an aborted run (three at most), reply lines of at most 65,536 characters, and
run.sh started with only PATH, HOME and LANG set, to the same values as in grading. Your program never sees key.json.
"""
import argparse
import fcntl
import json
import os
import queue
import select
import shutil
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import cv2
import numpy as np

CLASSES = ("dropped_after_grasp", "grasp_missed", "grasped_wrong_object")
FPS = 20
CHUNK = 10
FULL_S, ZERO_S = 1.0, 3.0
EARLY_S = 0.5   # alarms up to 0.5 s before the onset count as at the onset
STEP_TIMEOUT = 30.0
READY_TIMEOUT = 120.0
ALLOWANCE_S = 20.0
MAX_RESTARTS = 3
MAX_LINE = 1 << 16   # characters per reply line, as in grading
# Grading starts run.sh with exactly these variables and no others; so does this script.
PROGRAM_ENV = {"PATH": "/opt/venv/bin:/usr/local/bin:/usr/bin:/bin", "HOME": "/home/user", "LANG": "C.UTF-8"}
LOG_KEYS = ("gripper_command", "ee_pos_rel", "ee_rot_rel_rotvec", "base_pos", "base_rotvec")
HEADER_KEYS = ("instruction", "candidates", "camera_model", "fps")


class Aborted(Exception):
    pass


class Wire:
    def __init__(self, submission):
        self.proc = subprocess.Popen(["bash", "run.sh"], cwd=submission, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=None, text=True, bufsize=1, start_new_session=True, env=dict(PROGRAM_ENV))
        self.lines = queue.Queue()
        threading.Thread(target=self._pump, daemon=True).start()
        deadline = time.monotonic() + READY_TIMEOUT
        while self._get(deadline).strip() != "READY":
            pass

    def _pump(self):
        while True:
            line = self.proc.stdout.readline(MAX_LINE + 1)
            if not line or (len(line) > MAX_LINE and not line.endswith("\n")):
                break   # end of stream, or a reply line over the cap: the run aborts as a closed stream
            self.lines.put(line)
        self.lines.put(None)

    def _get(self, deadline):
        try:
            line = self.lines.get(timeout=max(0.0, deadline - time.monotonic()))
        except queue.Empty:
            raise Aborted("deadline missed")
        if line is None:
            raise Aborted("program closed its stdout")
        return line

    def request(self, msg):
        # As in grading: the deadline starts when the write starts and bounds the write too.
        deadline = time.monotonic() + STEP_TIMEOUT
        self._write(json.dumps(msg) + "\n", deadline)
        while True:
            line = self._get(deadline).strip()
            if not line:
                continue
            try:
                reply = json.loads(line)
            except ValueError:
                raise Aborted("non-JSON reply")
            if not isinstance(reply, dict):
                raise Aborted("reply is not an object")
            return reply

    def _write(self, text, deadline):
        data = memoryview(text.encode("utf-8"))
        try:
            self.proc.stdin.flush()
            fd = self.proc.stdin.fileno()
        except (OSError, ValueError):
            raise Aborted("stdin closed")
        flags = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        try:
            while data:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise Aborted("program did not read the request within its deadline")
                if not select.select([], [fd], [], remaining)[1]:
                    continue
                try:
                    data = data[os.write(fd, data):]
                except BlockingIOError:
                    continue
                except OSError:
                    raise Aborted("stdin closed")
        finally:
            fcntl.fcntl(fd, fcntl.F_SETFL, flags)

    def stop(self):
        try:
            self.proc.stdin.write(json.dumps({"type": "close"}) + "\n")
            self.proc.stdin.flush()
            self.proc.wait(timeout=10)
        except Exception:  # noqa: BLE001
            pass
        try:
            os.killpg(self.proc.pid, 9)
        except OSError:
            pass


def parse_action(action):
    if isinstance(action, dict) and set(action) == {"alarm"} and action["alarm"] is False:
        return "none", None
    if isinstance(action, dict) and set(action) == {"alarm", "failure_type", "object"} and action["alarm"] is True \
            and action["failure_type"] in CLASSES and isinstance(action["object"], str):
        return "alarm", action
    return "malformed", None


def timeliness(d):
    if d < -EARLY_S:
        return 0.0
    d = max(0.0, d)
    if d <= FULL_S:
        return 1.0
    if d >= ZERO_S:
        return 0.0
    return (ZERO_S - d) / (ZERO_S - FULL_S)


def play(wire, ep_dir, key, work):
    ep = json.load(open(ep_dir / "episode.json"))
    n = int(key["stream_frames"])
    z = np.load(ep_dir / "logs.npz")
    log = {k: np.round(np.asarray(z[k], dtype=np.float64)[:n], 6).tolist() for k in LOG_KEYS}
    header = {k: ep[k] for k in HEADER_KEYS}
    cap = cv2.VideoCapture(str(ep_dir / "video.mp4"))
    own = 0.0
    out = {"alarm_frame": None, "alarm": None, "void": False, "why": ""}
    wire.request({"type": "reset"})   # not counted, as in grading
    for t, start in enumerate(range(0, n, CHUNK)):
        stop = min(n, start + CHUNK)
        frames = []
        for _ in range(stop - start):
            ok, f = cap.read()
            if not ok:
                raise RuntimeError("practice video shorter than its key says")
            frames.append(cv2.cvtColor(f, cv2.COLOR_BGR2RGB))
        path = os.path.join(work, "chunk_{:05d}.npy".format(t))
        np.save(path, np.stack(frames).astype(np.uint8))
        obs = dict(header, frames=[start, stop], images=path, log={k: v[start:stop] for k, v in log.items()})
        t0 = time.monotonic()
        reply = wire.request({"type": "act", "t": t, "obs": obs})
        own += time.monotonic() - t0
        os.unlink(path)
        if "action" not in reply:
            raise Aborted("reply without an action")
        kind, alarm = parse_action(reply["action"])
        if own > stop / FPS + ALLOWANCE_S:
            out.update(void=True, why="fell behind real time")
            break
        if kind != "none":
            out.update(alarm_frame=stop - 1, alarm=alarm, why=kind)
            break
    cap.release()
    return out, own


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--kind", choices=("failure", "success"), default=None)
    ap.add_argument("--practice", default="/home/user/practice")
    ap.add_argument("--submission", default="/home/user/submission")
    args = ap.parse_args()
    root = Path(args.practice)
    ids = []
    for p in sorted(root.iterdir()):
        if (p / "key.json").is_file():
            k = json.load(open(p / "key.json"))
            if args.kind is None or k["kind"] == args.kind:
                ids.append((p.name, k))
    if args.limit:
        ids = ids[: args.limit]
    wire, restarts, broken = Wire(args.submission), 0, False
    work = tempfile.mkdtemp(prefix="practice_")
    rows = []
    try:
        for i, (eid, key) in enumerate(ids):
            if broken:
                wire.stop()
                if restarts >= MAX_RESTARTS:
                    print("three restarts used: the remaining runs would all be aborted")
                    break
                restarts += 1
                wire, broken = Wire(args.submission), False
            try:
                out, own = play(wire, root / eid, key, work)
            except Aborted as exc:
                out, own, broken = {"alarm_frame": None, "alarm": None, "void": True, "why": "aborted: " + str(exc)}, 0.0, True
            if key["kind"] == "success":
                alarmed = out["void"] or out["alarm_frame"] is not None
                rows.append({"kind": "success", "alarmed": alarmed})
                verdict = "false alarm" if alarmed else "no alarm (right)"
            else:
                credit, verdict = 0.0, "no alarm"
                if out["void"]:
                    verdict = out["why"]
                elif out["alarm_frame"] is not None:
                    d = (out["alarm_frame"] - key["onset_frame"]) / FPS
                    a = out["alarm"]
                    right = a is not None and a["failure_type"] == key["failure_type"] and a["object"] == key["subject"]
                    credit = timeliness(d) if right else 0.0
                    what = "false alarm (more than 0.5 s before the onset)" if d < -EARLY_S else \
                        "type and object right" if right else "type or object wrong, or malformed"
                    verdict = "alarm {:+.2f} s from onset, {} -> {:.2f}".format(d, what, credit)
                rows.append({"kind": "failure", "type": key["failure_type"], "credit": credit})
            print("{}/{} {} {} {} own {:.1f}s".format(i + 1, len(ids), eid, key.get("failure_type", "success"),
                                                     verdict, own), flush=True)
    finally:
        wire.stop()
        shutil.rmtree(work, ignore_errors=True)
    per = {c: [r["credit"] for r in rows if r["kind"] == "failure" and r["type"] == c] for c in CLASSES}
    present = [v for v in per.values() if v]
    p = sum(sum(v) / len(v) for v in present) / len(present) if present else 0.0
    s = [r["alarmed"] for r in rows if r["kind"] == "success"]
    f = sum(s) / len(s) if s else 0.0
    print("per-type mean alarm score:", {c: round(sum(v) / len(v), 3) for c, v in per.items() if v})
    print("P = {:.3f} over {} failed runs; F = {:.3f} over {} successful runs".format(
        p, sum(len(v) for v in per.values()), f, len(s)))


if __name__ == "__main__":
    main()
