"""The cell's wire: what an action may be, how an observation's poses are written, and the two helpers the grader's
probe lock needs (an action as a fixed list of numbers, a perturbed copy of an observation). Standard library and
NumPy only.

An action (one JSON object; every key optional):

    {"move": {"type": "tcp" | "joint" | "hold", "delta": [6 numbers], "ticks": 1..40,
              "guard_force": N, "guard_stroke_mm": mm},
     "spindle": {"speed_rpm": signed (negative loosens), "torque_limit": N m},
     "gripper": opening in metres,
     "cmd": "eject" | "tool_release" | "tool_lock" | "bit:<name>" | "done",
     "cams": ["overhead", "wrist"]}

`move` displaces the commanded target by `delta` over `ticks` control ticks (0.05 s each): for "tcp" the mounted
tool's centre point by [dx, dy, dz] (m, base frame) and by the rotation vector [rx, ry, rz] (rad, base frame, applied
on the left of the current orientation); for "joint" the six joint angles (rad). A pose on the wire is
[x, y, z, rx, ry, rz]: position (m) and rotation vector (rad) in the base frame.
"""

import math

import numpy as np

from . import kin
from . import series as S

DEFAULTS = {
    "sim_cap_s": 2700.0, "frame_budget": 400, "max_ticks": 40,
    "tcp_speed_max": 0.6, "tcp_rot_speed_max": 2.0, "joint_speed_max": [1.5, 1.5, 2.0, 2.5, 2.5, 2.5],
    "overload_force": 220.0, "overload_torque": 80.0, "overload_joint_error": 0.12, "stop_lock_s": 3.0,
    "tool_change_s": 2.0, "bit_change_s": 2.5, "settle_s": 1.5,
    "noise_force": 0.4, "noise_torque": 0.02,
    "render_subframes": 4,
    "render_settings": {
        "omni:rtx:rendermode": "RealTimePathTracing", "omni:rtx:post:aa:op": "dlss",
        "omni:rtx:post:dlss:execMode": "quality", "omni:rtx:post:motionblur:enabled": False,
        "omni:rtx:post:dof:enabled": False, "omni:rtx:post:tonemap:operator": "acesApproximation",
        "omni:rtx:post:tonemap:exposureKey": 0.25, "omni:rtx:rt:ecoMode:enabled": False,
    },
}
CAMS = ("overhead", "wrist")
CMDS = ("eject", "tool_release", "tool_lock", "done") + tuple("bit:" + b for b in S.BITS)
MOVES = ("hold", "tcp", "joint")


def pose6(T):
    return [round(float(v), 6) for v in T[:3, 3]] + [round(float(v), 6) for v in kin.R_rotvec(T[:3, :3])]


def pose_T(p):
    return kin.pose(kin.rotvec_R(p[3:6]), np.array(p[:3], dtype=float))


def _num(x, what, lo=-math.inf, hi=math.inf):
    if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) or not lo <= x <= hi:
        raise ValueError("%s must be a number in [%g, %g]" % (what, lo, hi))
    return float(x)


def check_action(a, max_ticks=DEFAULTS["max_ticks"]):
    """The action in full form (every key present), or ValueError saying what is wrong with it."""
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
    """An action as the fixed list of 15 numbers the probe lock compares. Raises ValueError on a bad action."""
    a = check_action(a)
    m, sp = a["move"], a["spindle"]
    return ([float(MOVES.index(m["type"]))] + m["delta"] + [float(m["ticks"]),
            -1.0 if m["guard_force"] is None else m["guard_force"], -1.0 if m["guard_stroke_mm"] is None else m["guard_stroke_mm"],
            sp["speed_rpm"] if sp else 0.0, sp["torque_limit"] if sp else 0.0,
            -1.0 if a["gripper"] is None else a["gripper"], float(CMDS.index(a["cmd"]) + 1 if a["cmd"] else 0),
            float(sum(1 << CAMS.index(c) for c in a["cams"]))])


def is_idle(a):
    """True when the action commands no motion (the probe lock does not count a pair of such actions)."""
    try:
        m = check_action(a)["move"]
    except ValueError:
        return False
    return m["type"] == "hold" or all(abs(v) < 1e-9 for v in m["delta"])


PROBE_SHIFT_RAD = 0.01


def _tool_T(name):
    """The mounted tool's centre point in the flange frame, as `cell.tcp_offset` has it."""
    return kin.pose(np.eye(3), np.array([0.0, 0.0, S.TOOLS[name]["tcp_z"] if name in S.TOOLS else 0.0]))


def _wrist_T():
    """The wrist camera in the flange frame, as `cell.frames` has it."""
    return kin.pose(S.camera_R("wrist"), np.array(S.CAMERAS["wrist"]["pos"]))


def perturbed(obs, rng, exact=None):
    """A copy of an observation as the cell would report it with the robot's joints shifted by a hidden amount: up
    to `PROBE_SHIFT_RAD` a joint, the same for the measured and the commanded angles (a shift that would take an
    angle past its limit is applied with the other sign).

    The copy is built the way the cell builds an observation (`cell.observe`, `cell.frames`): from joint angles that
    are not rounded, the angles are written rounded, `tcp` is `kin.flange(q)` times the mounted tool, `tcp_target`
    the same of `q_target`, and a wrist frame's `pose` is `kin.flange(q)` times the camera's mounting. So a copy's
    poses agree with its joint readings exactly as a true observation's do, down to the rounding, and the robot's
    model, which the agent holds, cannot tell the two apart (`residual`; a copy whose poses were shifted
    independently of its joints was recognised by that). The wrench, the joint speeds (differences of readings),
    the pictures, the spindle's and the gripper's readings are left as they are.

    `exact` are the cell's own angles before rounding (`cell.joints()`: the grader asks for them); the shifted
    angles are those plus the shift, and the shift is a whole number of the readings' last digit (1e-6 rad). So what
    a reading hides behind its last digit is the same in the copy as in the true observation, whether that is
    nothing (a commanded angle with round digits) or anything: measured on 4000 observations of each kind, the true
    one had the larger `residual` in 50 % of the pairs. Without `exact` (the agent's library has only the
    observation) each reading stands for an angle within half a digit of it, drawn afresh. `rng` gives
    uniform(lo, hi)."""
    out = dict(obs)
    q, qt = [float(v) for v in obs["q"]], [float(v) for v in obs["q_target"]]
    dq = []
    for i in range(6):
        d = round(rng.uniform(-PROBE_SHIFT_RAD, PROBE_SHIFT_RAD), 6)      # whole digits of a reading: see below
        lo, hi = float(kin.Q_LIMITS[i][0]), float(kin.Q_LIMITS[i][1])
        if not (lo <= q[i] + d <= hi and lo <= qt[i] + d <= hi):
            d = -d
        dq.append(d)
    if exact is not None:
        q, qt = [float(v) for v in exact["q"]], [float(v) for v in exact["q_target"]]
    else:
        q = [v + rng.uniform(-0.5e-6, 0.5e-6) for v in q]
        qt = [v + rng.uniform(-0.5e-6, 0.5e-6) for v in qt]
    q1, qt1 = [a + d for a, d in zip(q, dq)], [a + d for a, d in zip(qt, dq)]
    tool = _tool_T(obs.get("tool"))
    out["q"] = [round(v, 6) for v in q1]
    out["q_target"] = [round(v, 6) for v in qt1]
    out["tcp"] = pose6(kin.flange(q1) @ tool)
    out["tcp_target"] = pose6(kin.flange(qt1) @ tool)
    wrist = (obs.get("images") or {}).get("wrist")
    if isinstance(wrist, dict) and "pose" in wrist:
        out["images"] = dict(obs["images"], wrist=dict(wrist, pose=pose6(kin.flange(q1) @ _wrist_T())))
    return out


def residual(obs):
    """How far an observation's poses are from the ones the cell's rule gives for its own joint readings: the
    largest difference of a matrix entry (metres, for a position). Rounding on the wire leaves about 1e-6. The
    grader records it for the true and for the copied observation of every probed step (`probe_residual_true`,
    `probe_residual_copy` in the verdict's metrics): the two have to look alike, or the copy can be recognised."""
    tool = _tool_T(obs.get("tool"))
    pairs = [(obs["q"], obs["tcp"], tool), (obs["q_target"], obs["tcp_target"], tool)]
    wrist = (obs.get("images") or {}).get("wrist")
    if isinstance(wrist, dict) and "pose" in wrist:
        pairs.append((obs["q"], wrist["pose"], _wrist_T()))
    return max(float(np.abs(kin.flange([float(v) for v in q]) @ T - pose_T(p)).max()) for q, p, T in pairs)
