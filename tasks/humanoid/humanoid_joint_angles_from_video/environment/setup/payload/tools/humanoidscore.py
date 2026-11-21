"""Scoring of per-frame joint-angle estimates of the Unitree G1 against the true joint angles.

One module, used unchanged by the grader and by the practice tool (tools/score.py). It needs MuJoCo and the file
g1_keypoints.xml next to it: the kinematic tree of the G1 (the MuJoCo Menagerie model unitree_g1 without its
geometry, its pelvis fixed as on the stand) with 21 sites, the body points that the score looks at: knee, ankle,
toe and heel of each leg; the head; shoulder, elbow, wrist, hand centre, hand tip and the thumb side of the hand of
each arm.

An estimate is an array (n, 29): one row per video frame, the 29 joint angles in radians in the order of JOINTS.
For every frame the 21 points are placed by forward kinematics, once from the estimate and once from the true
angles. A point is correct when it lies within 5.0 cm of its true place; a row with a number that is not finite has
all its points wrong. The score of a recording is the fraction of correct points over all its frames (21 per frame).
"""
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
KEYPOINT_XML = os.path.join(HERE, "g1_keypoints.xml")
JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
KEYPOINTS = [
    "left_knee", "left_ankle", "left_toe", "left_heel", "right_knee", "right_ankle", "right_toe", "right_heel",
    "head",
    "left_shoulder", "left_elbow", "left_wrist", "left_hand", "left_hand_tip", "left_hand_side",
    "right_shoulder", "right_elbow", "right_wrist", "right_hand", "right_hand_tip", "right_hand_side",
]
TOL_M = 0.05
HEADER = ",".join(JOINTS)
MAX_BYTES = 8 * 1024 * 1024


class Malformed(Exception):
    """An estimate file that does not follow the format."""


def parse_estimate(data, frames):
    """Bytes of an estimate file -> array (frames, 29) of joint angles in radians. Raises Malformed."""
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
        raise Malformed("the first line is not the 29 joint names of JOINTS, comma-separated")
    rows = lines[1:]
    if len(rows) != frames:
        raise Malformed("%d rows, but the recording has %d frames" % (len(rows), frames))
    out = np.empty((frames, len(JOINTS)))
    for i, line in enumerate(rows):
        parts = line.split(",")
        if len(parts) != len(JOINTS):
            raise Malformed("row %d has %d fields, not %d" % (i + 1, len(parts), len(JOINTS)))
        for j, p in enumerate(parts):
            try:
                out[i, j] = float(p)
            except ValueError:
                raise Malformed("row %d, field %d is not a number" % (i + 1, j + 1))
    return out


def read_angles(path, frames):
    """A file in the estimate format with true angles (a labelled recording's joints.csv)."""
    with open(path, "rb") as f:
        return parse_estimate(f.read(), frames)


class Kinematics:
    """Forward kinematics of the 21 body points from the 29 joint angles."""

    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(KEYPOINT_XML)
        self.data = mujoco.MjData(self.model)
        names = [self.model.joint(j).name for j in range(self.model.njnt)]
        if sorted(names) != sorted(JOINTS) or self.model.nq != len(JOINTS):
            raise ValueError("g1_keypoints.xml does not have exactly the 29 joints of JOINTS")
        self.qadr = np.array([self.model.jnt_qposadr[self.model.joint(n).id] for n in JOINTS])
        self.sites = np.array([self.model.site(n).id for n in KEYPOINTS])

    def points(self, angles):
        """(n, 29) radians -> (n, 21, 3) metres; a row with a non-finite angle gives NaN points."""
        angles = np.asarray(angles, dtype=np.float64)
        out = np.full((angles.shape[0], len(KEYPOINTS), 3), np.nan)
        for i, q in enumerate(angles):
            if not np.all(np.isfinite(q)):
                continue
            self.data.qpos[self.qadr] = q
            mujoco.mj_kinematics(self.model, self.data)
            out[i] = self.data.site_xpos[self.sites]
        return out


def score_recording(pred, truth, kin):
    """Score an estimate (n, 29) against the true angles (n, 29). Returns a dict; see the module text."""
    pred = np.asarray(pred, dtype=np.float64)
    truth = np.asarray(truth, dtype=np.float64)
    if pred.shape != truth.shape or pred.ndim != 2 or pred.shape[1] != len(JOINTS):
        raise ValueError("estimate and truth must both have shape (frames, %d)" % len(JOINTS))
    p, t = kin.points(pred), kin.points(truth)
    if not np.all(np.isfinite(t)):
        raise ValueError("the true angles are not all finite")
    dist = np.linalg.norm(p - t, axis=2)                       # NaN where the estimate row is not finite
    ok = np.where(np.isfinite(dist), dist <= TOL_M, False)
    fin = np.isfinite(dist).all(axis=1)
    return {
        "score": float(ok.mean()),
        "frames": int(pred.shape[0]),
        "frames_finite": int(fin.sum()),
        "points_correct": int(ok.sum()),
        "mean_point_error_m": float(dist[fin].mean()) if fin.any() else None,
        "fraction_correct_per_point": {k: float(v) for k, v in zip(KEYPOINTS, ok.mean(axis=0))},
    }
