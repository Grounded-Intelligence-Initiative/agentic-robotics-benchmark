"""Scoring of per-frame estimates of the arm's state against a log.

One module, used unchanged by the grader and by the practice tool (tools/score.py).

A recording is a video of n frames at 24 frames per second and a log of the arm. An estimate
is an array (n, 5), one row per video frame: the four arm joint angles in radians (the log's
pos1 ... pos4) and the gripper's aperture in millimetres (the log's aperture). Video frame k
is compared with the log at time k/24 + d, linearly interpolated, for a time offset d that is
tried from 0.5 s to 1.5 s in steps of 1/240 s. Frames outside the time span of the log are
left out. A frame is correct when each of its four angles is within 5.0 degrees of the log
and its aperture within 3.0 mm of the log; a row with a non-finite number is a wrong frame.
The score of the recording is the fraction of correct frames at the offset that makes it
largest (ties: the offset with the smaller mean error, each column's error divided by its
tolerance).
"""
import math

import numpy as np

FPS = 24.0
JOINTS = 4
COLUMNS = JOINTS + 1   # q1..q4 in radians, then the aperture in millimetres
TOL_DEG = 5.0
TOL_MM = 3.0
OFFSET_LO = 0.5        # s
OFFSET_HI = 1.5        # s
OFFSETS_PER_S = 240
MIN_OVERLAP = 0.5      # an offset counts only if at least this fraction of the frames lies inside the log
HEADER = "q1,q2,q3,q4,aperture"
LOG_COLUMNS = ["pos1", "pos2", "pos3", "pos4", "aperture"]
MAX_BYTES = 4 * 1024 * 1024


class Malformed(Exception):
    """An estimate file that does not follow the format."""


def parse_estimate(data, frames):
    """Bytes of an estimate file -> array (frames, 5): q1..q4 in radians, aperture in mm. Raises Malformed."""
    if len(data) > MAX_BYTES:
        raise Malformed("larger than %d bytes" % MAX_BYTES)
    try:
        text = data.decode("ascii")
    except UnicodeDecodeError:
        raise Malformed("not ASCII text")
    lines = text.replace("\r\n", "\n").split("\n")
    while lines and lines[-1].strip() == "":
        lines.pop()
    if not lines or lines[0].strip() != HEADER:
        raise Malformed("the first line is not '%s'" % HEADER)
    rows = lines[1:]
    if len(rows) != frames:
        raise Malformed("%d rows, but the video has %d frames" % (len(rows), frames))
    out = np.empty((frames, COLUMNS))
    for i, line in enumerate(rows):
        parts = line.split(",")
        if len(parts) != COLUMNS:
            raise Malformed("row %d has %d fields, not %d" % (i + 1, len(parts), COLUMNS))
        for j, p in enumerate(parts):
            try:
                out[i, j] = float(p)
            except ValueError:
                raise Malformed("row %d, field %d is not a number" % (i + 1, j + 1))
    return out


def read_log(path):
    """(t, s) of a log file: time in seconds (m,), and (m, 5): pos1..pos4 in radians and the aperture in mm."""
    with open(path) as f:
        header = f.readline().strip().split(",")
    col = {c: i for i, c in enumerate(header)}
    need = ["timestamp"] + LOG_COLUMNS
    for c in need:
        if c not in col:
            raise ValueError("the log %s has no column %s" % (path, c))
    a = np.loadtxt(path, delimiter=",", skiprows=1, usecols=[col[c] for c in need], ndmin=2)
    t, s = a[:, 0], a[:, 1:]
    if len(t) < 2 or not np.all(np.diff(t) > 0):
        raise ValueError("the time of the log %s does not increase" % path)
    return t, s


def score_recording(pred, t_log, s_log):
    """Score an estimate (n, 5) against a log (t_log, s_log from read_log). Returns a dict; see the module text."""
    pred = np.asarray(pred, dtype=np.float64)
    if pred.ndim != 2 or pred.shape[1] != COLUMNS:
        raise ValueError("an estimate must have shape (frames, %d)" % COLUMNS)
    n = pred.shape[0]
    k = np.arange(n) / FPS
    tol = np.array([math.radians(TOL_DEG)] * JOINTS + [TOL_MM])
    finite = np.isfinite(pred).all(axis=1)
    safe = np.where(finite[:, None], pred, 0.0)
    best_key, best = None, None
    steps = int(round((OFFSET_HI - OFFSET_LO) * OFFSETS_PER_S))
    for i in range(steps + 1):
        d = OFFSET_LO + i / OFFSETS_PER_S
        tq = k + d
        inside = (tq >= t_log[0]) & (tq <= t_log[-1])
        m = int(inside.sum())
        if m == 0 or m < MIN_OVERLAP * n:
            continue
        ref = np.stack([np.interp(tq[inside], t_log, s_log[:, j]) for j in range(COLUMNS)], axis=1)
        diff = np.abs(safe[inside] - ref)
        diff[:, :JOINTS] = np.abs((safe[inside, :JOINTS] - ref[:, :JOINTS] + math.pi) % (2.0 * math.pi) - math.pi)
        fin = finite[inside]
        ok = (diff <= tol) & fin[:, None]
        correct = int(ok.all(axis=1).sum())
        mean_err = float((diff[fin] / tol).mean()) if fin.any() else float("inf")
        key = (correct / m, -mean_err)
        if best_key is None or key > best_key:
            best_key = key
            best = {
                "score": correct / m,
                "offset_s": d,
                "frames": n,
                "frames_scored": m,
                "frames_correct": correct,
                "mean_abs_error_deg": [float(v) for v in np.degrees(diff[fin, :JOINTS].mean(axis=0))] if fin.any() else None,
                "mean_abs_error_aperture_mm": float(diff[fin, JOINTS].mean()) if fin.any() else None,
                "fraction_within_tol": [float(v) for v in ok.mean(axis=0)],
            }
    if best is None:
        raise ValueError("the log does not cover the video for any time offset")
    return best
