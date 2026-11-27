"""Desired wrist trajectories: the motions a surgeon or an autonomy stack asks of the wrist.

One generator serves the hidden scored episodes (fresh seeds from OS entropy, in grader
memory only), the practice request the agent gets and the bench sessions in its logs, so the
three come from one distribution. A trajectory is N x 3 (pitch, yaw, grip) in radians at the
200 Hz command rate.

Motion vocabulary, mixed at random:
* reach: a minimum-jerk move to a new wrist pose over 0.25 to 1.6 s;
* hold: stay put for 0.1 to 0.8 s;
* tremor-scale corrections: a 2 to 6 Hz burst of 0.5 to 3 degree reversals on one or two axes;
* grasp: close the jaws to 3 to 8 degrees of opening and reopen, as when taking a needle;
* suture arc: a pitch-and-yaw arc with the jaws closed, as when driving a needle.
"""

import math

import numpy as np

DT = 0.005
PITCH_LIM = 1.15        # rad
YAW_LIM = 0.95          # rad
GRIP_MIN, GRIP_MAX = 0.05, 1.00
JAW_LIM = 1.35          # |yaw +- grip/2| stays inside this


def _minjerk(a, b, n):
    s = np.linspace(0.0, 1.0, n + 1)[1:]
    h = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    return a[None, :] + (b - a)[None, :] * h[:, None]


def _clip_pose(p):
    p = np.array(p, dtype=float)
    p[0] = np.clip(p[0], -PITCH_LIM, PITCH_LIM)
    p[2] = np.clip(p[2], GRIP_MIN, GRIP_MAX)
    room = JAW_LIM - 0.5 * p[2]
    p[1] = np.clip(p[1], -min(YAW_LIM, room), min(YAW_LIM, room))
    return p


def _random_pose(rng):
    return _clip_pose([rng.uniform(-PITCH_LIM, PITCH_LIM), rng.uniform(-YAW_LIM, YAW_LIM),
                       rng.uniform(GRIP_MIN, GRIP_MAX)])


def generate(rng, seconds=20.0, start=None):
    """One trajectory of `seconds` length from numpy Generator `rng`."""
    n_total = int(round(seconds / DT))
    pose = _clip_pose(start) if start is not None else _random_pose(rng)
    rows = [pose[None, :].copy()]
    count = 1
    while count < n_total:
        kind = rng.choice(["reach", "reach", "hold", "tremor", "grasp", "arc"],
                          p=[0.28, 0.12, 0.12, 0.2, 0.14, 0.14])
        if kind == "reach":
            target = _random_pose(rng)
            if rng.random() < 0.5:   # many moves are local
                target = _clip_pose(pose + rng.normal(0, [0.25, 0.22, 0.15]))
            n = max(2, int(rng.uniform(0.25, 1.6) / DT))
            seg = _minjerk(pose, target, n)
        elif kind == "hold":
            n = max(2, int(rng.uniform(0.1, 0.8) / DT))
            seg = np.repeat(pose[None, :], n, axis=0)
        elif kind == "tremor":
            n = max(4, int(rng.uniform(0.4, 1.5) / DT))
            f = rng.uniform(2.0, 6.0)
            amp = np.zeros(3)
            axes = rng.choice(3, size=rng.integers(1, 3), replace=False)
            amp[axes] = np.radians(rng.uniform(0.5, 3.0, size=len(axes)))
            t = np.arange(1, n + 1) * DT
            env = np.sin(np.pi * t / t[-1]) ** 2
            phase = rng.uniform(0, 2 * np.pi, 3)
            seg = pose[None, :] + (amp[None, :] * env[:, None]
                                   * np.sin(2 * np.pi * f * t[:, None] + phase[None, :]))
            seg = np.array([_clip_pose(p) for p in seg])
        elif kind == "grasp":
            closed = pose.copy()
            closed[2] = np.radians(rng.uniform(3.0, 8.0))
            closed = _clip_pose(closed)
            n1 = max(2, int(rng.uniform(0.2, 0.6) / DT))
            n2 = max(2, int(rng.uniform(0.1, 0.5) / DT))
            reopen = closed.copy()
            reopen[2] = rng.uniform(0.3, GRIP_MAX)
            reopen = _clip_pose(reopen)
            n3 = max(2, int(rng.uniform(0.2, 0.6) / DT))
            seg = np.concatenate([_minjerk(pose, closed, n1),
                                  np.repeat(closed[None, :], n2, axis=0),
                                  _minjerk(closed, reopen, n3)])
        else:  # arc
            n = max(4, int(rng.uniform(0.6, 1.8) / DT))
            t = np.linspace(0, 1, n + 1)[1:]
            ang = rng.uniform(0.6, 1.6) * np.sign(rng.normal())
            rp, ry = rng.uniform(0.15, 0.45), rng.uniform(0.15, 0.45)
            ph = rng.uniform(0, 2 * np.pi)
            s = 10 * t ** 3 - 15 * t ** 4 + 6 * t ** 5
            centre = pose - np.array([rp * math.cos(ph), ry * math.sin(ph), 0.0])
            seg = np.stack([centre[0] + rp * np.cos(ph + ang * s),
                            centre[1] + ry * np.sin(ph + ang * s),
                            np.full(n, np.radians(rng.uniform(3.0, 6.0)))], axis=-1)
            # blend the grip change in over the first fifth
            k = max(1, n // 5)
            seg[:k, 2] = np.linspace(pose[2], seg[k, 2], k + 1)[1:]
            seg = np.array([_clip_pose(p) for p in seg])
        rows.append(seg)
        count += len(seg)
        pose = seg[-1].copy()
    out = np.concatenate(rows)[:n_total]
    return out
