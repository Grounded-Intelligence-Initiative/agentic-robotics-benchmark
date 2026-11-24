"""Client side of the disassembly cell: the commissioning connection, frame decoding and the pose arithmetic of the
wire. Standard library, NumPy and OpenCV only. Nothing of the cell itself is in here: the cell runs in the
commissioning service (and in the grader), not in your account.

    from packcell_client import Session, decode_rgb, decode_depth, pose_T, T_pose
    s = Session(pack=1)                       # opens a commissioning session on development pack 1 (it is counted)
    obs = s.obs                               # the first observation
    obs = s.step({"move": {"type": "tcp", "delta": [0, 0, -0.05, 0, 0, 0], "ticks": 10}, "cams": ["wrist"]})
    rgb = decode_rgb(obs["images"]["wrist"])  # H x W x 3 uint8 (RGB)
    depth = decode_depth(obs["images"]["wrist"])   # H x W float32, metres along the optical axis, 0 = no reading
    s.close()
"""

import base64
import json
import math
import os
import socket

import numpy as np

SOCKET = os.environ.get("PACKCELL_SOCKET", "/run/packcell/commission.sock")
STATUS = "/run/packcell/status.json"


class CommissioningError(Exception):
    pass


class Session(object):
    """One commissioning session: a development pack in the cell, from its arrival to its end."""

    def __init__(self, pack, cams=(), path=SOCKET, timeout=1800.0):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect(path)
        self.f = self.sock.makefile("rwb")
        reply = self._ask({"op": "open", "pack": int(pack), "cams": list(cams)})
        self.obs = reply["obs"]
        self.number = reply.get("session")
        self.sessions_left = reply.get("sessions_left")

    def _ask(self, msg):
        self.f.write((json.dumps(msg, separators=(",", ":")) + "\n").encode("utf-8"))
        self.f.flush()
        line = self.f.readline()
        if not line:
            raise CommissioningError("the commissioning service closed the connection")
        reply = json.loads(line.decode("utf-8"))
        if "error" in reply:
            raise CommissioningError(reply["error"])
        return reply

    def step(self, action):
        """Execute one action; returns the observation. A refused action raises CommissioningError and changes
        nothing in the cell."""
        self.obs = self._ask({"op": "step", "action": action})["obs"]
        return self.obs

    def close(self):
        try:
            self._ask({"op": "close"})
        except (CommissioningError, OSError, ValueError):
            pass
        try:
            self.sock.close()
        except OSError:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
        return False


def status(path=SOCKET):
    """{"sessions_left": n, "sessions_used": k, "packs": [...], "busy": bool} without opening a session."""
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(30.0)
    s.connect(path)
    f = s.makefile("rwb")
    f.write(b'{"op":"status"}\n')
    f.flush()
    out = json.loads(f.readline().decode("utf-8"))
    s.close()
    return out


# ------------------------------------------------------------------------------------------------- frames
def _png(b64, flags):
    import cv2
    return cv2.imdecode(np.frombuffer(base64.b64decode(b64), np.uint8), flags)


def decode_rgb(image):
    """`image`: one entry of an observation's `images`. Returns H x W x 3 uint8, RGB."""
    import cv2
    return cv2.cvtColor(_png(image["rgb"], cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)


def decode_depth(image):
    """Depth along the camera's optical axis in metres (float32), 0 where there is no reading."""
    import cv2
    return _png(image["depth"], cv2.IMREAD_UNCHANGED).astype(np.float32) * float(image.get("depth_unit_m", 1e-4))


# ------------------------------------------------------------------------------------------------- poses
def rotvec_R(v):
    """Rotation matrix of a rotation vector (axis times angle, rad)."""
    v = np.asarray(v, dtype=float)
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


def R_rotvec(R):
    """Rotation vector of a rotation matrix (angle in [0, pi])."""
    c = max(-1.0, min(1.0, (R[0, 0] + R[1, 1] + R[2, 2] - 1.0) / 2.0))
    a = math.acos(c)
    if a < 1e-9:
        return np.zeros(3)
    if math.pi - a < 1e-6:
        M = (R + np.eye(3)) / 2.0
        i = int(np.argmax(np.diag(M)))
        return M[:, i] / math.sqrt(max(M[i, i], 1e-12)) * a
    return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(a)) * a


def pose_T(p):
    """A pose of the wire [x, y, z, rx, ry, rz] as a 4x4 matrix."""
    T = np.eye(4)
    T[:3, :3] = rotvec_R(p[3:6])
    T[:3, 3] = p[:3]
    return T


def T_pose(T):
    return [float(v) for v in T[:3, 3]] + [float(v) for v in R_rotvec(T[:3, :3])]


def tcp_delta(goal_T, current_T):
    """The `delta` of a "tcp" move that takes the commanded tool pose `current_T` to `goal_T`: the translation
    difference and the rotation vector of R_goal R_current^T, both in the base frame."""
    return [float(v) for v in goal_T[:3, 3] - current_T[:3, 3]] + [float(v) for v in R_rotvec(goal_T[:3, :3] @ current_T[:3, :3].T)]


def perturbed(obs, rng):
    """The copy of an observation a repeated step may carry (see the cell's interface document, section 2): the
    robot's own readings shifted by a hidden amount, the same for the measured and the commanded values. `rng` has
    uniform(lo, hi). The commissioning launcher uses this to repeat steps the way grading does."""
    out = dict(obs)
    dq = [rng.uniform(-0.02, 0.02) for _ in range(6)]
    dp = [rng.uniform(-0.012, 0.012) for _ in range(3)]
    dr = np.array([rng.uniform(-0.03, 0.03) for _ in range(3)])
    dw = [rng.uniform(-3.0, 3.0) for _ in range(3)] + [rng.uniform(-0.3, 0.3) for _ in range(3)]
    for k in ("q", "q_target"):
        out[k] = [round(v + d, 6) for v, d in zip(obs[k], dq)]
    for k in ("tcp", "tcp_target"):
        T = pose_T(obs[k])
        T[:3, :3] = rotvec_R(dr) @ T[:3, :3]
        T[:3, 3] += dp
        out[k] = [round(v, 6) for v in T_pose(T)]
    out["wrench"] = [round(v + d, 3) for v, d in zip(obs["wrench"], dw)]
    return out
