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


def perturbed(obs, rng):
    """A copy of an observation in which the robot's own readings are shifted by a hidden amount, the same for the
    measured and the commanded values: joints by up to 0.02 rad, the tool centre point by up to 12 mm and 0.03 rad,
    the wrench by up to 3 N and 0.3 N m. Images, the spindle's and the gripper's readings are left as they are.
    `rng` gives uniform(lo, hi)."""
    out = dict(obs)
    dq = [rng.uniform(-0.02, 0.02) for _ in range(6)]
    dp = [rng.uniform(-0.012, 0.012) for _ in range(3)]
    dr = np.array([rng.uniform(-0.03, 0.03) for _ in range(3)])
    dw = [rng.uniform(-3.0, 3.0) for _ in range(3)] + [rng.uniform(-0.3, 0.3) for _ in range(3)]
    for k in ("q", "q_target"):
        out[k] = [round(v + d, 6) for v, d in zip(obs[k], dq)]
    for k in ("tcp", "tcp_target"):
        T = pose_T(obs[k])
        T[:3, :3] = kin.rotvec_R(dr) @ T[:3, :3]
        T[:3, 3] += dp
        out[k] = pose6(T)
    out["wrench"] = [round(v + d, 3) for v, d in zip(obs["wrench"], dw)]
    return out
