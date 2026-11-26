"""Client side of the disassembly cell: the commissioning connection, frame decoding, the pose arithmetic of the
wire and the grader's rule for a repeated step's two replies (`replies_differ`). Standard library, NumPy and OpenCV
only. Nothing of the cell itself is in here: the cell runs in the
commissioning service (and in the grader), not in your account.

    from packcell_client import Session, decode_rgb, decode_depth, pose_T, T_pose, flange_T
    s = Session(pack=1)                       # opens a commissioning session on development pack 1 (it is counted)
    obs = s.obs                               # the first observation
    obs = s.step({"move": {"type": "tcp", "delta": [0, 0, -0.05, 0, 0, 0], "ticks": 10}, "cams": ["wrist"]})
    rgb = decode_rgb(obs["images"]["wrist"])  # H x W x 3 uint8 (RGB)
    depth = decode_depth(obs["images"]["wrist"])   # H x W float32, metres along the optical axis, 0 = no reading
                                              # (an active-stereo sensor's: noisy, quantized, with holes)
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
    """Rotation vector (axis times angle, angle in [0, pi]) of a rotation matrix, accurate at every angle: the angle
    from atan2 of the antisymmetric part's norm and the trace; within 1e-3 rad of a half turn the axis from the
    symmetric part, signed by the antisymmetric one (acos and a division by sin lose up to 2e-3 there)."""
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = 0.5 * float(np.linalg.norm(w))
    c = 0.5 * (R[0, 0] + R[1, 1] + R[2, 2] - 1.0)
    a = math.atan2(s, c)
    if a < 1e-9:
        return 0.5 * w
    if a < math.pi - 1e-3:
        return w / (2.0 * s) * a
    B = 0.5 * (R + R.T) - c * np.eye(3)              # (1 - cos a) k k^T
    i = int(np.argmax(np.diag(B)))
    k = B[:, i] / math.sqrt(max(B[i, i], 1e-300))
    if float(np.dot(k, w)) < 0.0:
        k = -k
    return k / np.linalg.norm(k) * a


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


# ------------------------------------------------------------------------------------------------- the robot's model
_FILES = {}


def _given(*parts):
    """A JSON file of `~/given`, read once (this library lies in `~/given/lib`)."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", *parts)
    if path not in _FILES:
        with open(path) as f:
            _FILES[path] = json.load(f)
    return _FILES[path]


def _quat_R(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def flange_T(q):
    """The tool flange's pose in the base frame (4x4) for joint angles `q` (rad), from `~/given/robot/ur20.json`:
    each joint frame in its parent link, then a turn about that frame's z axis. The cell reports `tcp` as this
    times the mounted tool's centre point (`tool_centre_point_z` along the flange's z axis), `tcp_target` as the
    same of `q_target`, and a wrist frame's `pose` as this times the camera's nominal mounting (`cell.json`; the
    cell's own mounting offset is not in it, see the interface document, section 5)."""
    T = np.eye(4)
    for joint, angle in zip(_given("robot", "ur20.json")["joints"], q):
        J = np.eye(4)
        J[:3, :3] = _quat_R(joint["quaternion_wxyz"])
        J[:3, 3] = joint["translation"]
        c, s = math.cos(float(angle)), math.sin(float(angle))
        Z = np.eye(4)
        Z[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
        T = T @ J @ Z
    return T


PROBE_SHIFT_RAD = 0.01


def perturbed(obs, rng):
    """The copy of an observation a repeated step may carry (see the cell's interface document, section 2): the
    observation the cell would give with the robot's joints shifted by a hidden amount, up to `PROBE_SHIFT_RAD` a
    joint, the same for the measured and the commanded angles (with the other sign where it would pass a joint's
    limit). `q` and `q_target` are the shifted angles; `tcp`, `tcp_target` and a wrist frame's `pose` are the poses
    of the shifted angles (`flange_T`), so the copy's readings agree with each other as a true observation's do.
    Everything else is unchanged. `rng` has uniform(lo, hi). The commissioning launcher uses this to repeat steps
    the way grading does."""
    robot = _given("robot", "ur20.json")
    out = dict(obs)
    q, qt = [float(v) for v in obs["q"]], [float(v) for v in obs["q_target"]]
    dq = []
    for i in range(6):
        d = round(rng.uniform(-PROBE_SHIFT_RAD, PROBE_SHIFT_RAD), 6)      # whole digits of a reading
        lo, hi = robot["limits_rad"][i]
        if not (lo <= q[i] + d <= hi and lo <= qt[i] + d <= hi):
            d = -d
        dq.append(d)
    # a reading is rounded to 1e-6: the angle behind it is anywhere within half a digit
    q = [v + rng.uniform(-0.5e-6, 0.5e-6) for v in q]
    qt = [v + rng.uniform(-0.5e-6, 0.5e-6) for v in qt]
    q1, qt1 = [a + d for a, d in zip(q, dq)], [a + d for a, d in zip(qt, dq)]
    tool = np.eye(4)
    tool[2, 3] = robot["tool_centre_point_z"].get(obs.get("tool") or "none", 0.0)
    out["q"] = [round(v, 6) for v in q1]
    out["q_target"] = [round(v, 6) for v in qt1]
    out["tcp"] = [round(v, 6) for v in T_pose(flange_T(q1) @ tool)]
    out["tcp_target"] = [round(v, 6) for v in T_pose(flange_T(qt1) @ tool)]
    wrist = (obs.get("images") or {}).get("wrist")
    if isinstance(wrist, dict) and "pose" in wrist:
        cam = _given("cell", "cell.json")["cameras"]["wrist"]
        mount = np.eye(4)
        mount[:3, :3], mount[:3, 3] = cam["R"], cam["pos"]
        out["images"] = dict(obs["images"], wrist=dict(wrist, pose=[round(v, 6) for v in T_pose(flange_T(q1) @ mount)]))
    return out


# The grader's comparison of the two replies to a repeated step (the cell's interface document, section 2), as the
# grader computes it: an action as a fixed list of 15 numbers, and a pair counted only if one of them moves the robot.
MAX_TICKS = 40
CAMS = ("overhead", "wrist")
CMDS = ("eject", "tool_release", "tool_lock", "done", "bit:S10", "bit:S13", "bit:T30", "bit:T40", "bit:EX")
MOVES = ("hold", "tcp", "joint")
PAIR_TOLERANCE = 1e-4
PAIR_PASS_FRACTION = 0.1


def _num(x, what, lo=-math.inf, hi=math.inf):
    if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) or not lo <= x <= hi:
        raise ValueError("%s must be a number in [%g, %g]" % (what, lo, hi))
    return float(x)


def check_action(a, max_ticks=MAX_TICKS):
    """The action in full form (every key present), or ValueError saying what is wrong with it (the cell's rule)."""
    if not isinstance(a, dict):
        raise ValueError("an action is a JSON object")
    extra = set(a) - {"move", "spindle", "gripper", "cmd", "cams"}
    if extra:
        raise ValueError("unknown action key(s): %s" % ", ".join(sorted(str(k) for k in extra)))
    m = a.get("move") or {"type": "hold"}
    if not isinstance(m, dict) or m.get("type", "hold") not in MOVES:
        raise ValueError("move.type must be one of %s" % ", ".join(MOVES))
    if set(m) - {"type", "delta", "ticks", "guard_force", "guard_stroke_mm"}:
        raise ValueError("unknown key in move")
    kind = m.get("type", "hold")
    ticks = m.get("ticks", 1)
    if isinstance(ticks, bool) or not isinstance(ticks, int) or not 1 <= ticks <= max_ticks:
        raise ValueError("move.ticks must be an integer from 1 to %d" % max_ticks)
    delta = m.get("delta", [0.0] * 6)
    if kind == "hold":
        delta = [0.0] * 6
    if not isinstance(delta, list) or len(delta) != 6:
        raise ValueError("move.delta must be a list of 6 numbers")
    lim = [2.0] * 3 + [math.pi] * 3 if kind == "tcp" else [2 * math.pi] * 6
    delta = [_num(v, "move.delta[%d]" % i, -lim[i], lim[i]) for i, v in enumerate(delta)]
    out = {"move": {"type": kind, "delta": delta, "ticks": ticks,
                    "guard_force": None if m.get("guard_force") is None else _num(m["guard_force"], "move.guard_force", 0.5, 1000.0),
                    "guard_stroke_mm": None if m.get("guard_stroke_mm") is None else _num(m["guard_stroke_mm"], "move.guard_stroke_mm", 0.1, 25.0)}}
    sp = a.get("spindle")
    if sp is not None:
        if not isinstance(sp, dict) or set(sp) - {"speed_rpm", "torque_limit"}:
            raise ValueError("spindle is {speed_rpm, torque_limit}")
        sp = {"speed_rpm": _num(sp.get("speed_rpm"), "spindle.speed_rpm", -1000.0, 1000.0),
              "torque_limit": _num(sp.get("torque_limit"), "spindle.torque_limit", 0.0, 1000.0)}
    out["spindle"] = sp
    g = a.get("gripper")
    out["gripper"] = None if g is None else _num(g, "gripper", 0.0, 1.0)
    cmd = a.get("cmd")
    if cmd is not None and cmd not in CMDS:
        raise ValueError("cmd must be one of %s" % ", ".join(CMDS))
    out["cmd"] = cmd
    cams = a.get("cams") or []
    if not isinstance(cams, list) or any(c not in CAMS for c in cams) or len(set(cams)) != len(cams):
        raise ValueError("cams is a list of distinct camera names from %s" % ", ".join(CAMS))
    out["cams"] = list(cams)
    return out


def canonical(a):
    """An action as the fixed list of 15 numbers the grader compares. Raises ValueError on a bad action."""
    a = check_action(a)
    m, sp = a["move"], a["spindle"]
    return ([float(MOVES.index(m["type"]))] + m["delta"] + [float(m["ticks"]),
            -1.0 if m["guard_force"] is None else m["guard_force"], -1.0 if m["guard_stroke_mm"] is None else m["guard_stroke_mm"],
            sp["speed_rpm"] if sp else 0.0, sp["torque_limit"] if sp else 0.0,
            -1.0 if a["gripper"] is None else a["gripper"], float(CMDS.index(a["cmd"]) + 1 if a["cmd"] else 0),
            float(sum(1 << CAMS.index(c) for c in a["cams"]))])


def is_idle(a):
    """True when the action commands no motion: a pair of two such replies is not compared."""
    try:
        m = check_action(a)["move"]
    except ValueError:
        return False
    return m["type"] == "hold" or all(abs(v) < 1e-9 for v in m["delta"])


def replies_differ(first, second, tolerance=PAIR_TOLERANCE):
    """The grader's rule for the two replies to one repeated step: True when one of their 15 numbers differs by more
    than `tolerance`. Raises ValueError for a reply that is not a valid action. A program passes when at least
    `PAIR_PASS_FRACTION` of its compared pairs (pairs in which both replies are idle are not compared) differ."""
    a, b = canonical(first), canonical(second)
    return len(a) == len(b) and max([abs(x - y) for x, y in zip(a, b)] or [0.0]) > tolerance
