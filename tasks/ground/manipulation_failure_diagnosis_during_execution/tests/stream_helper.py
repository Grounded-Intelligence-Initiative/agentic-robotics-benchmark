"""Read one hidden recording for the grader (run as root on /opt/venv/bin/python3, which has NumPy and OpenCV).

    python3 -I stream_helper.py      (then one JSON line on stdin: {"dir": <episode_dir>, "n": <n_frames>})

The recording and the playback length come over stdin, never on the command line or in the environment: a process's
command line is readable by other accounts through /proc, and the program being graded must not learn either.

Writes to stdout: one JSON line {"log": {name: rows}} with the first n_frames rows of logs.npz, then n_frames raw
RGB frames (256 x 768 x 3 uint8 each), decoded one at a time. The pipe's buffer is small, so decoding stays at most a
frame ahead of what the grader has taken: the helper does not compete with the program for the CPU while it works.
Any failure exits non-zero; the grader treats that as a task error.
"""
import json
import os
import sys

import cv2
import numpy as np

LOG_KEYS = ("gripper_command", "ee_pos_rel", "ee_rot_rel_rotvec", "base_pos", "base_rotvec")
H, W = 256, 768


def main():
    req = json.loads(sys.stdin.readline())
    ep_dir, n = req["dir"], int(req["n"])
    sys.stdin.close()
    cv2.setNumThreads(1)
    z = np.load(os.path.join(ep_dir, "logs.npz"))
    log = {}
    for k in LOG_KEYS:
        a = np.asarray(z[k], dtype=np.float64)
        if len(a) < n:
            raise SystemExit("logs shorter than the playback")
        log[k] = np.round(a[:n], 6).tolist()
    out = sys.stdout.buffer
    out.write((json.dumps({"log": log}, separators=(",", ":")) + "\n").encode())
    cap = cv2.VideoCapture(os.path.join(ep_dir, "video.mp4"))
    for _ in range(n):
        ok, frame = cap.read()
        if not ok or frame.shape != (H, W, 3):
            raise SystemExit("video shorter than the playback")
        out.write(np.ascontiguousarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).tobytes())
    out.flush()
    cap.release()


if __name__ == "__main__":
    main()
