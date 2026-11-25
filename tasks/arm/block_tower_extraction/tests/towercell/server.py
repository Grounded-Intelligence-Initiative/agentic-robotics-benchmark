"""The cell behind the wire: one episode (one tower), the program's calls (`handle(method, params)`), the budgets.

The grader (tests/verify.py through cellkit.wire) and the commissioning service run the same class. `work_wall` is the
wall time the cell itself spends (simulation, rendering): the wire charges the program everything else.
"""

import math
import os
import time

import mujoco
import numpy as np

from .cameras import NAMES, Cameras
from .cell import Cell, quat_to_mat
from .config import CONFIG, n_cap, public_config

PUBLIC = ("describe", "get_state", "get_camera", "step", "ik", "finish")
PRIVILEGED = ("get_state_privileged", "contacts_above")
WORK_CALLS = ("step", "get_camera", "ik")


def default_panda_dir():
    env = os.environ.get("BT_PANDA_DIR")
    if env:
        return env
    for cand in ("/opt/task-assets/panda",):
        if os.path.isfile(os.path.join(cand, "panda.xml")):
            return cand
    raise RuntimeError("the Panda model is not found (BT_PANDA_DIR)")


def _num(v, name, lo=None, hi=None):
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError("%s must be a number" % name)
    try:
        x = float(v)
    except (OverflowError, ValueError):
        raise ValueError("%s is not a finite number" % name)
    if not math.isfinite(x):
        raise ValueError("%s must be finite" % name)
    if (lo is not None and x < lo) or (hi is not None and x > hi):
        raise ValueError("%s must be within %s .. %s" % (name, lo, hi))
    return x


def _int(v, name, lo, hi):
    x = _num(v, name, lo, hi)
    if x != int(x):
        raise ValueError("%s must be an integer" % name)
    return int(x)


def _f(v, n, name, lo=-10.0, hi=10.0):
    if v is None:
        return None
    if not isinstance(v, (list, tuple)) or len(v) != n:
        raise ValueError("%s must be %d numbers" % (name, n))
    return [_num(x, name, lo, hi) for x in v]


def _r(x, k=6):
    return [round(float(v), k) for v in x]


class CellServer(object):
    def __init__(self, tower, duration_s=None, privileged=False, panda_dir=None, mode="hidden"):
        self.cell = Cell(panda_dir or default_panda_dir(), tower, duration_s=duration_s)
        self.cams = Cameras(self.cell)
        self.mode = mode
        self.privileged = privileged
        self.work_wall = 0.0
        self.wall_start = None
        self.wall_used = 0.0
        self.program_used = 0.0
        self.calls = 0
        self.step_calls = 0
        self.illegal_calls = 0
        self.bad_calls = 0
        self.internal_errors = 0
        self.physics_faults = 0
        self.steps_after_done = 0
        b = CONFIG["budgets"]
        dur = self.cell.duration
        factor = b["privileged_program_time_factor"] if privileged else b["program_time_factor"]
        self.program_budget_s = factor * dur + b["program_time_extra_s"]
        self.silence_s = b["silence_s"]

    @property
    def done(self):
        return self.cell.done

    @property
    def done_reason(self):
        return self.cell.done_reason

    def _end(self, reason):
        self.cell.end(reason)

    def close(self):
        self.cams.close()

    def handle(self, method, params):
        self.calls += 1
        if not isinstance(params, dict):
            self.bad_calls += 1
            return {"error": "bad_params", "detail": "params must be an object"}
        allowed = PUBLIC + (PRIVILEGED if self.privileged else ())
        if method not in allowed:
            self.illegal_calls += 1
            return {"error": "forbidden"}
        t0 = time.time()
        try:
            out = getattr(self, "m_" + method)(**params)
        except (TypeError, ValueError, OverflowError) as e:
            self.bad_calls += 1
            out = {"error": "bad_params", "detail": str(e)[:200]}
        except mujoco.FatalError as e:
            self.physics_faults += 1
            self.cell.end("physics_fault")
            out = {"error": "internal_error", "detail": "the simulation failed: %s" % str(e)[:200]}
        except Exception as e:  # noqa: BLE001 - the cell's own fault: counted, the episode ends
            self.internal_errors += 1
            self.cell.end("internal_error")
            out = {"error": "internal_error", "detail": "%s: %s" % (type(e).__name__, str(e)[:200])}
            import traceback
            traceback.print_exc()
        if method in WORK_CALLS:
            self.work_wall += time.time() - t0
            if method == "step" and not self.cell.done and not np.all(np.isfinite(self.cell.d.qpos)):
                self.physics_faults += 1
                self.cell.end("physics_fault")
        return out

    def frames_delivered(self):
        return self.cell.frames_used

    # ----------------------------------------------------------------------------------------- calls
    def m_describe(self):
        c = self.cell
        cams = {}
        for n in NAMES:
            cams[n] = {"intrinsics": self.cams.intrinsics(n), "depth_range_m": CONFIG["cameras"][n]["depth_range_m"],
                       "mount": "hand" if n == "wrist" else "fixed"}
            if n != "wrist":
                cams[n]["T_world_cam"] = [[round(float(x), 6) for x in r] for r in self.cams.pose(n)]
        return {"config": public_config(), "cameras": cams, "duration_s": c.duration, "n_cap": n_cap(),
                "frame_budget": c.frame_budget, "program_budget_s": self.program_budget_s,
                "silence_s": self.silence_s, "max_ticks_per_step": CONFIG["budgets"]["max_ticks_per_step"],
                "arm_joint_limits": [[float(a), float(b)] for a, b in zip(c.lo - 0.02, c.hi + 0.02)]}

    def m_get_state(self):
        c = self.cell
        d = c.d
        tcp, tR = c.site_pose("tcp")
        rt, _ = c.site_pose("rod_tip")
        f, tq = c.wrench()
        prog_left = None
        ptf = getattr(self, "program_time", None)
        if callable(ptf):
            prog_left = round(self.program_budget_s - ptf(), 2)
        from .cell import mat_to_quat
        return {
            "t": round(c.t, 4),
            "q": _r(d.qpos[c.qadr]),
            "dq": _r(d.qvel[c.dadr], 5),
            "q_cmd": _r(c.q_cmd),
            "tcp": {"pos": _r(tcp), "quat": _r(mat_to_quat(tR))},
            "rod_tip": {"pos": _r(rt)},
            "motion": {"goal_reached": bool(c.reached), "out_of_reach": bool(c.out_of_reach),
                       "ik_error_m": round(float(c.ik_err[0]), 6), "ik_error_rad": round(float(c.ik_err[1]), 5)},
            "gripper": {"mode": c.grip_mode[0], "setpoint": c.grip_mode[1], "opening_m": round(c.opening(), 5)},
            "wrench": {"force_N": _r(f, 3), "torque_Nm": _r(tq, 4)},
            "pusher_deployed": bool(c.pusher_on),
            "turntable": {"angle_rad": round(c.table_angle(), 6), "target_rad": round(c.table_goal, 6),
                          "moving": bool(c.table_moving())},
            "frames_left": c.frame_budget - c.frames_used,
            "program_time_left_s": prog_left,
            "episode_over": c.done,
            "over_reason": c.done_reason,
        }

    def m_get_camera(self, name, depth=False):
        if not isinstance(name, str) or name not in NAMES:
            raise ValueError("camera must be one of %s" % (NAMES,))
        return self.cams.frame(name, bool(depth))

    def m_step(self, n=1, target=None, frame="tcp", speed=0.1, rot_speed=1.0, q_target=None, joint_speed=0.8,
               posture=None, grip_open=None, grip_close=None, pusher=None, turntable=None):
        c = self.cell
        self.step_calls += 1
        n = _int(n, "n", 0, CONFIG["budgets"]["max_ticks_per_step"])
        speed = _num(speed, "speed", 0.0, 10.0)
        rot_speed = _num(rot_speed, "rot_speed", 0.0, 10.0)
        joint_speed = _num(joint_speed, "joint_speed", 0.0, 10.0)
        if target is not None and q_target is not None:
            raise ValueError("give target or q_target, not both")
        if not isinstance(frame, str) or frame not in ("tcp", "rod_tip"):
            raise ValueError("frame must be tcp or rod_tip")
        if c.done:
            return dict(self.m_get_state(), ticks=0)
        if posture is not None:
            c.q_nom = np.clip(np.array(_f(posture, 7, "posture", -7.0, 7.0)), c.lo, c.hi)
        if target is not None:
            if not isinstance(target, dict):
                raise ValueError("target must be {pos, quat}")
            pos = _f(target.get("pos"), 3, "target.pos", -2.0, 2.0)
            quat = _f(target.get("quat"), 4, "target.quat", -2.0, 2.0)
            if np.linalg.norm(quat) < 1e-3:
                raise ValueError("quat must be non-zero")
            c.set_cart_goal(frame, pos, quat, speed, rot_speed)
        if q_target is not None:
            c.set_joint_goal(_f(q_target, 7, "q_target", -7.0, 7.0), joint_speed)
        if grip_open is not None and grip_close is not None:
            raise ValueError("give grip_open or grip_close, not both")
        if grip_open is not None:
            c.grip_open(_num(grip_open, "grip_open", 0.0, 0.08))
        if grip_close is not None:
            c.grip_close(_num(grip_close, "grip_close", 0.0, 1000.0))
        if pusher is not None:
            c.pusher(bool(pusher))
        if turntable is not None:
            c.table_goal = _num(turntable, "turntable", -100.0, 100.0)
        t0 = c.ticks
        c.run_ticks(n)
        out = self.m_get_state()
        out["ticks"] = c.ticks - t0
        return out

    def m_ik(self, pos, quat, frame="tcp", seed=None):
        c = self.cell
        if not isinstance(frame, str) or frame not in ("tcp", "rod_tip"):
            raise ValueError("frame must be tcp or rod_tip")
        p = _f(pos, 3, "pos", -2.0, 2.0)
        q = _f(quat, 4, "quat", -2.0, 2.0)
        if np.linalg.norm(q) < 1e-3:
            raise ValueError("quat must be non-zero")
        s = c.q_home if seed is None else np.clip(np.array(_f(seed, 7, "seed", -7.0, 7.0)), c.lo, c.hi)
        qs, ep, er = c.solve_ik(frame, np.array(p), quat_to_mat(q), s)
        return {"q": _r(qs), "pos_error_m": round(ep, 7), "rot_error_rad": round(er, 6)}

    def m_finish(self):
        self.cell.end("finished")
        return {"ok": True}

    # privileged (the reference program's token only)
    def m_get_state_privileged(self):
        c = self.cell
        loads, gaps = c.loads_and_gaps()
        blocks = []
        for i in range(c.n):
            b = c.ids[i]
            blocks.append({"id": i, "pos": _r(c.d.xpos[b], 7), "xmat": _r(c.d.xmat[b], 7),
                           "h": c.tower["blocks"][i]["h"], "state": c.state[i], "level": c.level[i],
                           "slot": c.slot[i], "load_above_N": round(float(loads[i]), 5),
                           "gap_above_m": round(float(gaps[i]), 7)})
        return {"blocks": blocks, "legal_below": c.legal_below(), "collapsed": c.collapsed}

    def m_contacts_above(self, i):
        c = self.cell
        i = _int(i, "i", 0, c.n - 1)
        f, p = c.contacts_above(i)
        b = c.ids[i]
        return {"force_N": round(float(f), 6), "pos": None if p is None else _r(p, 7), "block_pos": _r(c.d.xpos[b], 7)}

    # summary
    def summary(self):
        c = self.cell
        s = dict(c.tally())
        s.update({"t_end": c.t, "end_reason": c.done_reason, "calls": self.calls, "step_calls": self.step_calls,
                  "illegal_calls": self.illegal_calls, "bad_calls": self.bad_calls,
                  "internal_errors": self.internal_errors, "frames_used": c.frames_used,
                  "work_wall_s": round(self.work_wall, 2), "physics_faults": self.physics_faults,
                  "program_time_s": round(float(self.program_used), 2), "program_budget_s": self.program_budget_s})
        return s
