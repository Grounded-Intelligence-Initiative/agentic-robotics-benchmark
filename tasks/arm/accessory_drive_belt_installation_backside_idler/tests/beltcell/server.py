"""BeltCellServer: the grader-owned physics, rules and API of the belt station (cell internals; the hidden grader runs
this class on the pinned hidden engines).

API (metres / rad / wxyz quaternions, world frame unless said; `cell_api.md` is the agent-facing contract):
  describe()                              static facts: robots, hands, control, sensors, cameras, the drive drawing,
                                          the belt and tensioner data, the grip rule, safety, budgets
  get_work_order()                        engine id, drive variant, belt part number, routing decal id
  get_state()                             sim_time, done, stopped; per arm: joints, tcp pose, mode, stiffness, gripper
                                          width, the flange F/T (tare-relative, mean over the tick)
  set_control(arm, mode, stiffness_scale) joint | cartesian; stiffness_scale in [0.2, 1.0]
  command(left={...}, right={...})        per arm: q (joint mode) | tcp_pos + tcp_quat (cartesian mode), width
  tare(arm)                               zero that arm's F/T at its current reading
  step(n_ticks)                           advance n x 10 ms (n <= 1000); {sim_time, done, stopped}
  get_camera(name, depth)                 front | left_wrist | right_wrist: RGB png + depth png (+ the pose at capture)
  finish()                                ends the episode; the grader then runs the drive in and inspects it
Every other call answers forbidden and is counted.

Nothing on the wire says whether a span is seated, the belt slipped in a hand, the tensioner caught the belt or a pulley
is wrapped: the program has the robots' own sensors and the cameras. A protective stop ends the episode and sets
`stopped` (no diagnosis); damage is silent.
"""

import base64
import io
import json
import math
import time
import traceback

import numpy as np

from . import CELL_VERSION
from . import drive as dr
from .cameras import CAMERA_SPECS, SENSOR, CameraRig, apply_sensor_model, draw_colour
from .control import IMP
from .episode import RUN_IN, run_in, score_engine
from .layout import HIDDEN_RANGES, draw_layout
from .physics import GRIPPER, HOME_Q, LAYOUT, PHYS, Scene
from .rules import DAMAGE, GRIP, SAFETY, Rules

__all__ = ["BeltCellServer", "DEFAULT_CONFIG", "PUBLIC_METHODS"]

PUBLIC_METHODS = ("describe", "get_work_order", "get_state", "set_control", "command", "tare", "step", "get_camera", "finish")
STATE_METHODS = ("set_control", "command", "tare", "step")
ARMS = {"left": "L", "right": "R"}

DEFAULT_CONFIG = {
    "sim_cap_s": 1500.0,
    "program_time_cap_s": 2400.0,
    "silence_s": 120.0,
    "max_ticks_per_step": 1000,
    "camera_budget_frames": 600,
    "ft_noise": {"force_N": 0.3, "torque_Nm": 0.02},
    "settle_s": 1.0,
}


def _deep_update(base, upd):
    for k, v in (upd or {}).items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_update(base[k], v)
        else:
            base[k] = v
    return base


def _jsonable(x):
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return [_jsonable(v) for v in x.tolist()]
    if isinstance(x, np.floating):
        return float(x)
    if isinstance(x, np.integer):
        return int(x)
    if isinstance(x, np.bool_):
        return bool(x)
    return x


def _finite(v, n=None):
    a = np.asarray(v, float).reshape(-1)
    if n is not None and a.size != n:
        raise ValueError("expected %d numbers" % n)
    if not np.all(np.isfinite(a)):
        raise ValueError("non-finite number")
    return a


class BeltCellServer(object):
    def __init__(self, panda_dir, config=None, log=None, render=True):
        import mujoco
        self.mujoco = mujoco
        self.panda_dir = panda_dir
        self.cfg = _deep_update(json.loads(json.dumps(DEFAULT_CONFIG)), config)
        self.log = log or (lambda s: None)
        self.render_enabled = bool(render)
        self.scene = None
        self.done = False
        self.done_reason = None
        self.wall_used = 0.0
        self.program_used = 0.0
        self.work_wall = 0.0
        self.internal_errors = 0

    # ------------------------------------------------------------------ episode setup
    def reset(self, seed):
        self.seed = int(seed)
        self.layout = draw_layout(self.seed)
        t0 = time.time()
        self.scene = Scene(self.layout["prm"], self.panda_dir)
        self.rules = Rules(self.scene)
        self.scene.reset_arms_home()
        self.scene.pose_on_peg(self.layout["drape"]["bottom_r"], self.layout["drape"]["start"])
        self.tick_s = self.scene.tick_s
        self.done = False
        self.done_reason = None
        self.tick_count = 0
        self.illegal_calls = 0
        self.internal_errors = 0
        self.calls = {}
        self.step_calls = 0
        self.frames_used = 0
        self.frames_by_camera = {c: 0 for c in CAMERA_SPECS}
        self.render_wall_s = 0.0
        self.render_failures = 0
        self.frames_lost = 0
        self._noise_rng = np.random.default_rng([self.layout["noise_seed"], 0x5E])
        self.ft_bias = {s: np.asarray(self.layout["ft_bias"][s], float) for s in ("L", "R")}
        self.ft_noise = {s: np.zeros(6) for s in ("L", "R")}
        self.ft_tare = {s: np.zeros(6) for s in ("L", "R")}
        self.cameras = CameraRig(self.scene.model) if self.render_enabled else None
        self.colour = draw_colour(np.random.default_rng([self.layout["noise_seed"], 0xC0]), list(CAMERA_SPECS))
        self.max_lever_load = 0.0
        # the belt settles on its peg with the arms holding their home pose
        for _ in range(int(round(self.cfg["settle_s"] / self.tick_s))):
            self.physics_tick()
        self.t_start = float(self.scene.data.time)
        self.build_wall_s = time.time() - t0
        return {"ok": True}

    # ------------------------------------------------------------------ helpers
    def _arm(self, arm):
        a = ARMS.get(str(arm))
        if a is None:
            raise ValueError("arm must be 'left' or 'right'")
        return a

    def _ft(self, s):
        raw = self.scene.ft_tick[s]
        return raw + self.ft_bias[s] + self.ft_noise[s] - self.ft_tare[s]

    def _arm_state(self, s):
        sc = self.scene
        arm = sc.arms[s]
        p, R = arm.tcp_actual()
        base = np.asarray(LAYOUT["bases"][s], float)
        ft = self._ft(s)
        return {"q": [float(v) for v in arm.q()], "dq": [float(v) for v in arm.dq()], "cmd_q": [float(v) for v in arm.q_cmd],
                "tcp_pos": [float(v) for v in p], "tcp_quat": [float(v) for v in arm.mat_to_quat(R)],
                "tcp_pos_base": [float(v) for v in (p - base)], "mode": arm.mode, "stiffness_scale": float(arm.stiffness_scale),
                "gripper_width": float(arm.width()),
                "ft": {"force_N": [float(v) for v in ft[:3]], "torque_Nm": [float(v) for v in ft[3:]]}}

    # ------------------------------------------------------------------ API
    def describe(self):
        return describe_cell(self)

    def get_work_order(self):
        L = self.layout
        n = dr.VARIANTS[dr.CELL_VARIANT]["belt_links"]
        return {"engine_id": L["engine_id"], "variant": dr.CELL_VARIANT, "belt_part_number": "6PK%d" % int(round(1000 * n * dr.BELT["link"])),
                "decal_id": "ROUTE-B1", "routing": "ribs on C, ALT and WP; back on I and T (cell_manual.md, catalog.md)"}

    def get_state(self):
        sc = self.scene
        return {"sim_time": float(sc.data.time), "done": bool(self.done), "stopped": bool(self.rules.stopped),
                "left": self._arm_state("L"), "right": self._arm_state("R")}

    def set_control(self, arm, mode="joint", stiffness_scale=1.0):
        if self.done:
            return {"error": "episode_over"}
        s = self._arm(arm)
        if str(mode) not in ("joint", "cartesian"):
            raise ValueError("mode must be 'joint' or 'cartesian'")
        k = float(_finite([stiffness_scale], 1)[0])
        if not 0.2 - 1e-9 <= k <= 1.0 + 1e-9:
            raise ValueError("stiffness_scale must be within [0.2, 1.0]")
        a = self.scene.arms[s]
        a.set_mode(str(mode))
        a.set_stiffness(k)
        return {"ok": True, "arm": arm, "mode": a.mode, "stiffness_scale": a.stiffness_scale}

    def _command_one(self, s, spec):
        a = self.scene.arms[s]
        if not isinstance(spec, dict):
            raise ValueError("an arm's command is an object")
        extra = set(spec) - {"q", "tcp_pos", "tcp_quat", "width"}
        if extra:
            raise ValueError("unknown command keys: %s" % ", ".join(sorted(extra)))
        if spec.get("q") is not None:
            if a.mode != "joint":
                raise ValueError("a q target needs joint mode")
            a.command_joint(_finite(spec["q"], 7))
        tp, tq = spec.get("tcp_pos"), spec.get("tcp_quat")
        if tp is not None or tq is not None:
            if a.mode != "cartesian":
                raise ValueError("a tcp target needs cartesian mode")
            if tp is None or tq is None:
                raise ValueError("tcp_pos and tcp_quat go together")
            q = _finite(tq, 4)
            if np.linalg.norm(q) < 1e-6:
                raise ValueError("zero quaternion")
            a.command_cartesian(_finite(tp, 3), q / np.linalg.norm(q))
        if spec.get("width") is not None:
            w = float(_finite([spec["width"]], 1)[0])
            if not -1e-9 <= w <= 0.08 + 1e-9:
                raise ValueError("width must be within [0, 0.08] m")
            self.rules.command_width(s, w, a.width())
            a.command_width(w)

    def command(self, left=None, right=None):
        if self.done:
            return {"error": "episode_over"}
        if left is None and right is None:
            raise ValueError("command needs left and/or right")
        for arm, spec in (("left", left), ("right", right)):
            if spec is not None:
                self._command_one(ARMS[arm], spec)
        return {"ok": True}

    def tare(self, arm):
        s = self._arm(arm)
        self.ft_tare[s] = self.scene.ft_tick[s] + self.ft_bias[s] + self.ft_noise[s]
        return {"ok": True, "arm": arm}

    def step(self, n_ticks=1):
        n = int(n_ticks)
        if n < 1 or n > int(self.cfg["max_ticks_per_step"]):
            raise ValueError("n_ticks must be within [1, %d]" % int(self.cfg["max_ticks_per_step"]))
        self.step_calls += 1
        t = time.time()
        try:
            for _ in range(n):
                if self.done:
                    break
                self.physics_tick()
        finally:
            self.work_wall += time.time() - t
        return {"sim_time": float(self.scene.data.time), "done": bool(self.done), "stopped": bool(self.rules.stopped)}

    def physics_tick(self):
        sc = self.scene
        for a in sc.arms.values():
            a.tick()
        ok = sc.step_physics()
        self.tick_count += 1
        nr = self._noise_rng
        n = self.cfg["ft_noise"]
        for s in ("L", "R"):
            self.ft_noise[s] = np.concatenate([nr.normal(0.0, n["force_N"], 3), nr.normal(0.0, n["torque_Nm"], 3)])
        if not ok:
            if not self.done:
                self._end("physics_fault")
            return
        self.rules.after_physics()
        if self.rules.held.get("R") == ("lever",) or self.rules.held.get("L") == ("lever",):
            for s in ("L", "R"):
                if self.rules.held.get(s) == ("lever",):
                    self.max_lever_load = max(self.max_lever_load, self.rules.grip_load(s)[0])
        if self.rules.stopped and not self.done:
            self._end("protective_stop")
        if sc.data.time - getattr(self, "t_start", 0.0) >= self.cfg["sim_cap_s"] - 1e-9 and not self.done:
            self._end("sim_cap")

    def get_camera(self, name="front", depth=True):
        name = str(name)
        if name not in CAMERA_SPECS:
            raise ValueError("unknown camera (cameras: %s)" % ", ".join(CAMERA_SPECS))
        if not self.render_enabled:
            return {"error": "camera_unavailable"}
        if self.frames_used >= int(self.cfg["camera_budget_frames"]):
            return {"error": "camera_budget"}
        k = self.frames_used
        self.frames_used += 1
        self.frames_by_camera[name] += 1
        t_frame = time.time()
        try:
            out = self._frame(name, k, bool(depth))
            if isinstance(out, dict) and out.get("error") == "camera_unavailable":
                self.frames_used -= 1
                self.frames_by_camera[name] -= 1
            return out
        finally:
            self.work_wall += time.time() - t_frame

    def _frame(self, name, k, depth):
        t = time.time()
        sc = self.scene
        rgb = dep = None
        for attempt in range(2):
            try:
                rgb, dep = self.cameras.render(sc.data, name)
                break
            except Exception as e:  # noqa: BLE001
                self.log("[cell] %s frame %d not rendered (attempt %d): %s" % (name, k, attempt + 1, type(e).__name__))
                self.render_failures += 1
        if rgb is None:
            self.frames_lost += 1
            self.render_wall_s += time.time() - t
            return {"error": "camera_unavailable"}
        from PIL import Image
        kind = CAMERA_SPECS[name]["kind"]
        rgb2, q = apply_sensor_model(kind, rgb, dep, self._noise_rng, self.colour[name])
        buf = io.BytesIO()
        Image.fromarray(rgb2).save(buf, format="PNG", compress_level=1)
        out = {"ok": True, "camera": name, "frame_index": int(k), "sim_time": float(sc.data.time), "width": int(rgb2.shape[1]),
               "height": int(rgb2.shape[0]), "format": "png", "rgb_png_b64": base64.b64encode(buf.getvalue()).decode("ascii"),
               "T_world_cam": self.cameras.extrinsics(sc.data, name).tolist()}
        if depth:
            b2 = io.BytesIO()
            Image.fromarray(q).save(b2, format="PNG", compress_level=1)
            out["depth_u16_b64"] = base64.b64encode(b2.getvalue()).decode("ascii")
            out["depth_unit_m"] = SENSOR[kind]["quant_m"]
            out["depth_invalid_value"] = 0
        self.render_wall_s += time.time() - t
        return out

    def finish(self):
        self._end("finish")
        return {"ok": True, "sim_time": float(self.scene.data.time)}

    # ------------------------------------------------------------------ dispatch
    def handle(self, method, params):
        self.calls[method] = self.calls.get(method, 0) + 1
        try:
            return self._dispatch(method, params)
        except Exception:  # noqa: BLE001
            self.internal_errors += 1
            self.log("[cell] internal error in %s\n%s" % (method, traceback.format_exc()))
            if method in STATE_METHODS:
                self._end("internal_error")
            return {"error": "internal_error"}

    def _dispatch(self, method, params):
        if method not in PUBLIC_METHODS:
            self.illegal_calls += 1
            return {"error": "forbidden"}
        if params is not None and not isinstance(params, dict):
            return {"error": "bad_params: params must be an object"}
        try:
            return getattr(self, method)(**(params or {}))
        except (TypeError, ValueError, KeyError, IndexError, ArithmeticError, OverflowError) as e:
            return {"error": "bad_params: %s" % str(e)[:200]}

    def _end(self, reason):
        if not self.done:
            self.done = True
            self.done_reason = reason

    # ------------------------------------------------------------------ end of episode
    def finalize(self):
        if not self.done:
            self._end("solve_returned")
        sc = self.scene
        stopped = bool(self.rules.stopped)
        damage = self.rules.damage
        fault = bool(sc.physics_fault)
        result = None
        score, items = 0.0, {}
        t_run = time.time()
        if not fault and not stopped:
            result = run_in(sc, self.rules)
            fault = not result["ok"]
            damage = self.rules.damage              # read after the run-in: the damage rules hold through it (amendment 1)
            if not fault:
                score, items = score_engine(result, stopped, damage, dr.VARIANTS[dr.CELL_VARIANT]["tensioner"]["window"])
        summary = {
            "cell_version": CELL_VERSION, "seed": self.seed, "engine_id": self.layout["engine_id"], "done_reason": self.done_reason,
            "sim_time_s": float(sc.data.time), "wall_time_s": float(self.wall_used), "ticks": int(self.tick_count),
            "episode_score": float(score), "items": items, "physics_fault": fault, "stopped": stopped, "stop_what": self.rules.stop_what,
            "damage": damage, "run_in": None if result is None else {"slip": result["slip"], "inspect": result["inspect"]},
            "run_in_wall_s": time.time() - t_run,
            "illegal_call_count": int(self.illegal_calls), "internal_errors": int(self.internal_errors), "calls": dict(self.calls),
            "step_calls": int(self.step_calls), "program_time_s": float(self.program_used), "cell_work_s": float(self.work_wall),
            "camera_frames_total": int(self.frames_used), "camera_frames": dict(self.frames_by_camera),
            "render_wall_s": float(self.render_wall_s), "render_failures": int(self.render_failures), "frames_lost": int(self.frames_lost),
            "build_wall_s": float(self.build_wall_s), "peak_lever_N": float(self.max_lever_load), "peaks": self.rules.peak,
            "events_log": [[round(t, 3), e] for t, e in self.rules.events][:4000],
        }
        return _jsonable(summary)


def describe_cell(srv):
    """The static facts of the station (the agent's describe()): everything here is in cell_manual.md / catalog.md."""
    sc = srv.scene
    arms = {}
    for side, name in (("L", "left"), ("R", "right")):
        a = sc.arms[side]
        arms[name] = {"base_pos": list(LAYOUT["bases"][side]), "base_yaw_rad": 0.0, "home_q": list(HOME_Q[side]),
                      "joints": ["%s/joint%d" % (side, i + 1) for i in range(7)], "joint_range": a.ranges(),
                      "torque_limit_Nm": list(IMP["tau_max"])}
    return _jsonable({
        "cell_version": CELL_VERSION,
        "units": {"length": "m", "angle": "rad", "quaternion": "wxyz", "time": "s", "force": "N", "torque": "N m"},
        "frames": {"world": "x forward (towards the engine), y left, z up; the base plate top is z = 0",
                   "engine": "E: origin on the crank axis in the plane of the accessory face; x to the right and z up as seen from the robots, y into the engine; depth d = -y in front of the face",
                   "engine_nominal_pose": {"origin_world": list(LAYOUT["E_pos"]), "x_E_world": [0.0, -1.0, 0.0], "y_E_world": [1.0, 0.0, 0.0], "z_E_world": [0.0, 0.0, 1.0],
                                           "tolerance": "the engine stands on its pallet within +-4 mm, +-1 deg yaw and +-0.5 deg roll of nominal"}},
        "robots": {"model": "two franka_emika_panda with the Franka hand (MuJoCo Menagerie, Apache-2.0), torque-driven joints; links gravity-compensated",
                   "arms": arms,
                   "tcp": "between the finger pads, 0.1034 m from the hand frame along its +z (the approach axis); +y = the fingers' closing axis",
                   "gripper": {"max_width_m": 0.08, "rate_m_s": 0.05, "max_force_N": GRIPPER["max_force"]},
                   "hand_data_sheet": HAND_DATA},
        "control": CONTROL_DATA,
        "grip_rule": GRIP_RULE_TEXT,
        "safety": SAFETY_TEXT,
        "damage": DAMAGE_TEXT,
        "ft_sensor": {"frame": "the hand frame at the flange (each arm)",
                      "reports": "force (N) and torque (N m) the arm exerts on the hand and everything it holds (the hand's own weight included), mean over the 10 ms tick, minus the reading at the last tare()",
                      "noise": {"force_N_rms": srv.cfg["ft_noise"]["force_N"], "torque_Nm_rms": srv.cfg["ft_noise"]["torque_Nm"]},
                      "bias": "an unknown constant offset per arm and episode (within +-1.0 N, +-0.05 N m), removed by tare()"},
        "cameras": srv.cameras.describe() if srv.cameras is not None else {},
        "drive": drive_drawing(),
        "belt": BELT_DATA,
        "tensioner": TENSIONER_DATA,
        "acceptance": ACCEPTANCE_TEXT,
        "budgets": {"sim_time_cap_s": srv.cfg["sim_cap_s"], "program_time_cap_s": srv.cfg["program_time_cap_s"],
                    "program_time": "the episode's wall clock minus the cell's own time inside step (simulation) and get_camera (rendering)",
                    "silence_abort_s": srv.cfg["silence_s"], "max_ticks_per_step": srv.cfg["max_ticks_per_step"],
                    "camera_frames": int(srv.cfg["camera_budget_frames"]), "tick_s": PHYS["tick_s"]},
    })


# ---------------------------------------------------------------------------------------------------- the data sheets
HAND_DATA = {
    "mass_kg": 0.76, "com_in_hand_frame_m": [-0.0096, 0.0, 0.0311],
    "note": "hand + two fingers; the flange F/T reads their weight (about 7.5 N) after a tare at another orientation: compensate it from this mass and centre of mass, or tare again after turning the hand",
    "finger": {"pad_end_beyond_tcp_m": 0.0080, "fingertip_beyond_tcp_m": 0.0089, "pad_half_width_x_m": 0.0085,
               "pad_z_range_in_hand_frame_m": [0.0944, 0.1114], "pad_face_to_finger_back_m": 0.0155,
               "only_pads": "only the pads' faces are finger pads; every other surface of a finger counts as a link for the safety rule"},
}
CONTROL_DATA = {
    "tick_s": 0.01, "physics_step_s": 0.002, "modes": ["joint", "cartesian"], "stiffness_scale_range": [0.2, 1.0],
    "joint": "each joint follows a trapezoidal path to its target (<= 1.5 rad/s, <= 3 rad/s^2), tracked by a PD on the joint torques: tau = s kp (q_cmd - q) - kd dq, kp = (4500, 4500, 3500, 3500, 2000, 2000, 2000) N m/rad, kd = min(0.1 s kp, (88, 1183, 74, 293, 56, 61, 43)) N m s/rad",
    "cartesian": "an impedance at the tcp: F = s 3000 N/m (x_ref - x) - critical damping, M = s 150 N m/rad orientation error - critical damping, mapped by J^T; a null-space term keeps the arm near the IK posture of the reference; x_ref moves towards the commanded pose at <= 0.5 m/s and <= 1.5 rad/s (a straight line); the reference starts from the actual tcp when the mode changes to cartesian",
    "holds_loads": "links are gravity-compensated; held loads and contact forces are not",
    "torque_limit_Nm": list(IMP["tau_max"]),
}
GRIP_RULE_TEXT = (
    "A closing gripper (commanded width more than 2 mm below its current width, or 1 mm or less, while its fingers close towards it: until they reach "
    "the commanded width or stop on what they hold) holds what lies between its finger pads at the pose of "
    "closing: a belt link whose centre is within the pads (hand-frame |x| <= 8.5 mm (+3 mm), z 94.4-111.4 mm (+-3 mm), "
    "|y| within the opening) and whose thickness lies across the closing axis within 30 deg (the link is welded to the "
    "hand), or the round service lever whose axis passes between the pads within 30 deg of square to the closing axis "
    "(two point constraints on the lever's axis 5 mm either side of the pads' middle, so the lever may turn between "
    "the pads); the lever can be held by one hand at a time: a second hand closing on it while the other holds it holds nothing. Opening the fingers (commanded width above the current width by more than 2 mm) lets go. The hold slips "
    "(lets go) when the held thing pulls with more than 30 N, or when the moment about the grip on a held belt exceeds 1.0 N m about any axis (bending it flatwise or edgewise, or twisting it), for 50 ms. "
    "Open fingers and the backs of closed fingers otherwise touch the belt without friction, so a span can be pushed. The belt meets a finger pad on its flat face (the pads' small ridges do not touch the belt).")
SAFETY_TEXT = ("Any Panda link or hand surface pressing on anything (the engine, the drive and its lever, the belt, the peg, the base "
               "plate and the pallet, the other arm) with more than 30 N for 30 ms, and any part of one arm, its finger pads included, "
               "pressing on the other arm with more than 30 N for 30 ms, stops both arms: the episode ends (stopped) and the engine "
               "scores 0. The finger pads are excepted against everything except the other arm. The force is judged "
               "per pair of bodies (the largest contact force between them, not a sum).")
DAMAGE_TEXT = ("The tensioner's damper is rated for 60 deg/s of arm speed: faster (a snap-back, a violent swing) damages it and "
               "the engine is rejected. A belt that touches the base plate (an element joint lower than 4 mm above it) is scrap. "
               "Damage is not reported during the episode; it is judged through the episode, the switch-off, the settle and the run-in.")
ACCEPTANCE_TEXT = (
    "After finish the robots are switched off and out of the drive; the drive settles for 1 s; a run-in motor turns the "
    "crank clockwise at 1.5 rev/s for 4 s against the accessories' drag. Then: every grooved pulley (C, ALT, WP) wrapped on its rib "
    "side with the belt in the groove zone on the groove floor (not on a shoulder or edge), the tensioner pulley and the idler I "
    "each wrapped on the belt's back with the belt on its face between the flanges, each accessory (ALT, WP) turned within 3 % "
    "of the crank's angle times its ratio (slip), the tensioner's pointer inside its window marks, all judged the moment the run-in motor stops. Score = g x q per engine: "
    "g = 1 if no damage, no protective stop, the pointer inside the window and the idler I passing, else 0; "
    "q = (C, ALT, WP, T passing + 1 if the slip is within spec) / 5.")


def drive_drawing():
    V = dr.VARIANTS[dr.CELL_VARIANT]
    T = V["tensioner"]
    g = dr.GROOVE
    tp = dr.TPULLEY
    return {
        "variant": dr.CELL_VARIANT, "frame": "engine frame E (x, z in the face plane; depth d in front of the face)",
        "pulleys": {k: ({"centre_xz": list(P["c"]), "groove_floor_radius": P["r"], "kind": P["kind"]} if P["kind"] != "idler" else
                        {"centre_xz": list(P["c"]), "face_radius": P["r"], "kind": "idler", "face_d": [tp["d_face0"], tp["d_face1"]],
                         "flanges_d": [[tp["d_back"], tp["d_face0"]], [tp["d_face1"], tp["d_front1"]]], "flange_height": tp["flange_h"],
                         "surface": "smooth, flanged (the belt's back runs on it)"})
                    for k, P in V["pulleys"].items()},
        "route": "C, ALT, I, WP, T (counter-clockwise as drawn): ribs on C, ALT, WP; back on I and T",
        "crank_rotation": "clockwise as drawn (the belt runs C, T, WP, I, ALT)",
        "groove": {"back_shoulder_d": [g["d_back"], g["d_zone0"]], "groove_zone_d": [g["d_zone0"], g["d_zone1"]],
                   "front_shoulder_d": [g["d_zone1"], g["d_front1"]], "front_chamfer": "45 deg, %.1f mm, from the shoulder top down to the floor radius in front" % (1000 * g["chamfer"]),
                   "shoulder_height_above_floor": g["shoulder_h"]},
        "tensioner_pulley": {"radius": T["r"], "face_d": [tp["d_face0"], tp["d_face1"]], "flanges_d": [[tp["d_back"], tp["d_face0"]], [tp["d_face1"], tp["d_front1"]]],
                             "flange_height": tp["flange_h"], "surface": "smooth (the belt's back runs on it)"},
        "tensioner": {"pivot_xz": list(T["pivot"]), "arm_length": T["arm"],
                      "angle": "theta = the pulley arm's direction from the pivot, counter-clockwise from E +x in the drawing; the tensioner opens clockwise (theta decreasing)",
                      "closed_stop_deg": math.degrees(T["theta_closed"]), "install_stop_deg": math.degrees(T["theta_open"]),
                      "install_stop_clearance": "at the install stop the pulley face stands 13.5 mm (its flanges 9.5 mm) clear of the straight belt line from WP to C",
                      "window_marks_deg": [math.degrees(v) for v in T["window"]],
                      "pointer": "a white blade on the arm's boss pointing away from the pulley (theta + 180 deg), 30-46 mm from the pivot, 8.5 mm in front of the face; the window marks are white lines on the face at theta + 180 deg, 40-52 mm from the pivot",
                      "service_lever": {"radius_from_pivot": T["lever_r"], "angle_from_pulley_arm_deg": math.degrees(T["lever_off"]),
                                        "diameter": 2 * T["lever_radius"], "d_range": [T["lever_d0"], T["lever_d0"] + T["lever_len"]],
                                        "note": "a round bar parallel to the pulley axes on a branch of the arm; it swings with the arm on a circle about the pivot"}},
        "peg": {"axis_xz": list(LAYOUT["peg"]), "radius": LAYOUT["peg_r"], "d_range": [0.0, LAYOUT["peg_len"]]},
    }


BELT_DATA = {
    "section": "poly-V, 20 mm wide x 4.5 mm thick (modelled flat, no rib/groove interlock, no twist)",
    "nominal_length_m": dr.VARIANTS[dr.CELL_VARIANT]["belt_links"] * dr.BELT["link"], "length_tolerance": "+-0.5 % of nominal",
    "pitch_line": "10 mm rigid elements (links), each with flatwise and edgewise bending joints",
    "edgewise_to_flatwise_stiffness": 8.0,
    "mass_per_m_kg": [0.216, 0.264], "droop_class": "a 200 mm free end held level droops 32-71 mm",
    "install_slack": "with the tensioner at its install stop the belt is longer than the four-pulley path (C, ALT, I, WP with the back on I; 1.1665 m at the joint line) by about 37-50 mm",
    "print": "white part-number print every 50 mm on the back; the back is black",
    "friction": "belt on the pulleys' grooves and faces: mu 0.6-0.9; on everything else: frictionless",
}
TENSIONER_DATA = {
    "spring": "torsion spring towards closing: torque = preload at the closed stop + rate x (theta - closed stop)",
    "rate_Nm_per_rad": dr.VARIANTS[dr.CELL_VARIANT]["tensioner"]["rate"], "preload_at_closed_stop_Nm": dr.VARIANTS[dr.CELL_VARIANT]["tensioner"]["tau_closed"],
    "spread": "rate and preload each +-15 % per engine", "damper_Nm_s_per_rad": [0.7, 1.3], "pivot_friction_Nm": [0.04, 0.12],
    "rated_arm_speed_dps": DAMAGE["arm_speed_dps"],
}
