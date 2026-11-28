"""The packing cell: one shift at one feed point (chute, slat conveyor or overhead drop feeder) with an R1Pro, in OmniGibson.

Runs in the simulator interpreter. Scene: OmniGibson (MIT) + R1Pro (MIT assets) + NVIDIA public Isaac assets
(Simple_Room backdrop, YCB items converted by build_items.py). No BEHAVIOR-1K data.

Frames: world = robot start frame (robot at the origin facing +x, z up). 30 Hz control, CPU physics.
Time: one control step per action row; every camera frame asked for costs FRAME_COST_STEPS further steps with the robot
holding its targets and the base stopped (a fixed price, the same on every host).
"""
from __future__ import annotations
import json, math, os, tempfile
from pathlib import Path
import numpy as np

import props as PR
import shift as SH

HZ = 30
FRAME_COST_STEPS = 3
FAST = os.environ.get("CELL_FAST", "1") != "0"     # fast stepping path (validated by the ramp session, job 62700367: 51 against 18 steps/s, identical proprioception)
ITEMS_JSON = Path(os.environ.get("CELL_ITEMS", "/opt/cell_items/items.json"))
ROOM_USD = os.environ.get("CELL_ROOM", "/opt/isaac_assets/Isaac/Environments/Simple_Room/simple_room.usd")
FINGER_FRICTION = 2.0     # rubber pads on the gripper fingers


class R:      # R1Pro action row (30 Hz): joint position targets except the base (velocity) and grippers (-1 close .. 1 open)
    ACTION_DIM = 23
    ACTION_SLICES = {"base": slice(0, 3), "trunk": slice(3, 7), "arm_left": slice(7, 14), "gripper_left": slice(14, 15),
                     "arm_right": slice(15, 22), "gripper_right": slice(22, 23)}
CAMERAS = {"head": "zed_link", "left_wrist": "left_realsense_link", "right_wrist": "right_realsense_link"}
LOW_END = np.array([0.38, -0.30, 0.85])        # chute's low end / belt's open end (surface height), nominal: in the right arm's reach
TOTE_AT = np.array([0.10, -0.70, 0.55])        # centre of the tote's inner floor, nominal
FEED_AT = np.array([0.50, -0.30])               # overhead feeder: centre of its drop window, nominal
MIN_FALL = 0.25                                # a dropped part may be touched only after it has fallen this far (metres)
PARK = np.array([-8.0, 6.0, 0.5])
HOLD_STEPS, REST_SPEED, BYSTANDER_MOVE = 30, 0.05, 0.05
RACK = {"rows": 2, "cols": 5, "pitch": 0.10}     # overhead feeder: ten rods hang at once, in two rows of five
BYSTANDER_SETTLE = 45     # steps after which a bystander's resting place is recorded
CREDIT = {"placed": 1.0, "held": 0.5, "lost": 0.0}


def quat_xyzw(roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr, cp, sp, cy, sy = (math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2), math.cos(yaw / 2), math.sin(yaw / 2))
    return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]


def rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Cell:
    def __init__(self, station: str, seed: int, n_parts: int = 20, cam_res: int = 320, overrides: dict | None = None, room: bool = True, assisted: bool = False):
        import sys
        import omnigibson as og, torch as th
        from omegaconf import OmegaConf
        from omnigibson.macros import gm
        # The R1Pro configuration OmniGibson's own evaluator uses, read directly (the evaluator module drags in
        # tele-operation and data-set packages this cell has no use for).
        DEFAULT_ROBOT_CONFIG_PATH = os.path.join(os.path.dirname(og.__file__), "eval", "r1pro.yaml"); EVAL_BASE_LINK_MASS = 250.0
        _to_plain_dict = lambda cfg: dict(OmegaConf.to_container(cfg, resolve=True))
        assert station in ("track", "belt", "drop")
        self.og, self.th, self.R, self.station = og, th, R, station
        gm.HEADLESS = True; gm.RENDER_VIEWER_CAMERA = False
        gm.USE_GPU_DYNAMICS = False; gm.ENABLE_TRANSITION_RULES = True      # as OmniGibson's evaluator sets them: PhysX on the CPU
        # One Kit app at a time through start-up and scene loading: two starting together on one card segfault.
        self._start_lock = None
        if os.environ.get("CELL_START_LOCK"):
            import fcntl
            self._start_lock = open(os.environ["CELL_START_LOCK"], "a"); fcntl.flock(self._start_lock, fcntl.LOCK_EX)
        og.launch()                                   # Kit first: the prop files below are written with Kit's own pxr
        # The finger pads' material (finger_static_friction below) is created by Isaac's PhysicsMaterial while the robot
        # loads, outside OmniGibson's USD edit context, which this OmniGibson refuses ("USD edit detected outside of
        # og.sim.editing_usd()", job 62739780). Give that constructor the context when it has none.
        from isaacsim.core.api.materials.physics_material import PhysicsMaterial
        if not getattr(PhysicsMaterial, "_cell_edit_context", False):
            _init = PhysicsMaterial.__init__
            def _init_in_context(mat, *args, **kwargs):
                if og.sim._editing_usd: return _init(mat, *args, **kwargs)
                with og.sim.editing_usd(): return _init(mat, *args, **kwargs)
            PhysicsMaterial.__init__ = _init_in_context; PhysicsMaterial._cell_edit_context = True
        self.plan = SH.make_shift("A", seed, n_parts, station=station)
        c = self.plan.constants
        c.update(overrides or {})
        g = np.random.default_rng([21, seed])
        # Installed layout: nominal plus this shift's hidden offsets.
        self.low_end = LOW_END + np.array([g.uniform(-0.05, 0.05), g.uniform(-0.08, 0.08), g.uniform(-0.03, 0.03)])
        self.yaw = float(g.uniform(-0.15, 0.15))
        self.tote_at = TOTE_AT + np.array([g.uniform(-0.05, 0.05), g.uniform(-0.05, 0.05), 0.0])
        self.slope = math.radians(c.get("track_slope_deg", 7.0)); self.friction = float(c.get("track_friction", 0.05))
        self.belt_speed = float(c.get("belt_speed", 0.3))
        if station == "drop":                         # low_end: centre of the feeder's underside (parts hang just below it)
            self.low_end = np.array([FEED_AT[0] + (self.low_end[0] - LOW_END[0]), FEED_AT[1] + (self.low_end[1] - LOW_END[1]), float(c.get("feeder_height", 1.5))])
        tmp = Path(tempfile.mkdtemp(prefix="cell_props_"))
        catalog = json.loads(ITEMS_JSON.read_text())
        # Overhead feeder: the parts are rods in sponge sleeves (high friction), hanging upright, every one of a shift with its
        # own radius, length, mass and colour. CELL_DROP_PARTS=ycb puts the 20 household items there instead.
        self.rods = station == "drop" and os.environ.get("CELL_DROP_PARTS", "rods") == "rods"
        if self.rods:
            import colorsys
            for k, e in enumerate(self.plan.events):
                r = e.target["rod"]; name = f"rod_{k:02d}"
                usd = PR.write_rod(tmp / f"{name}.usd", r["radius"], r["length"], colorsys.hsv_to_rgb(r["hue"], 0.75, 0.9), friction=PR.ROD_FRICTION)
                catalog[name] = {"usd": usd, "size_m": [2 * r["radius"], 2 * r["radius"], r["length"]], "mass_kg": r["mass"]}; e.target["item"] = name
        self.item_names = [e.target["item"] for e in self.plan.events]
        self.rack = None
        if self.rods:
            # The rack: ten rods hang under the feeder at once and nothing shows which one goes next (the author's decision
            # of 2026-11-26: with one rod hanging, the hand could wait under it and the catch was a matter of timing only).
            # Each event lets go the rod at one place of the rack; when the event is over a new rod is hung there. Which
            # place goes is drawn anew for every event, every place equally likely: there is no favourite place to wait under
            # (2026-11-28; with per-shift weights a frontier model kept its hand under the likeliest places and scored 0.74).
            gr = np.random.default_rng([51, seed]); S = RACK["rows"] * RACK["cols"]
            w = np.full(S, 1.0 / S); seq = [int(gr.choice(S, p=w)) for _ in range(n_parts)]      # every place equally likely (2026-11-28)
            slot_of, hang_at, extra = {}, {}, []          # object index -> place; object index -> event at whose start it is hung (-1: from the start)
            for sl in range(S):
                evs = [i for i, x in enumerate(seq) if x == sl]; prev = -1
                for i in evs: slot_of[i] = sl; hang_at[i] = prev; prev = i + 1
                if prev < n_parts: k = n_parts + len(extra); extra.append(k); slot_of[k] = sl; hang_at[k] = prev      # hangs there to the end of the shift
            for k in extra:
                r = {"radius": float(gr.uniform(0.0125, 0.030)), "length": float(gr.uniform(0.28, 0.42)), "mass": float(gr.uniform(0.05, 0.40)), "hue": float(gr.uniform(0.0, 1.0))}; name = f"rod_{k:02d}"
                usd = PR.write_rod(tmp / f"{name}.usd", r["radius"], r["length"], colorsys.hsv_to_rgb(r["hue"], 0.75, 0.9), friction=PR.ROD_FRICTION)
                catalog[name] = {"usd": usd, "size_m": [2 * r["radius"], 2 * r["radius"], r["length"]], "mass_kg": r["mass"]}; self.item_names.append(name)
            self.rack = {"slot_of": slot_of, "hang_at": hang_at, "weights": w.tolist(), "sequence": seq}; self.hung = {}
        spare = [n for n in SH.CATALOG if n not in self.item_names]
        self.bystander_names = (spare + [self.item_names[int(k)] for k in g.permutation(n_parts)[:2]])[:3]
        n_parts_all = len(self.item_names)

        robot_cfg = _to_plain_dict(OmegaConf.load(DEFAULT_ROBOT_CONFIG_PATH)); robot_cfg.pop("eval", None)
        robot_cfg["model"] = robot_cfg["model"].lower()
        robot_cfg.update(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0], obs_modalities=["proprio", "rgb", "depth_linear"])
        # Grasping is physical: a part stays in the hand by finger friction alone. OmniGibson's "assisted" mode (the simulator
        # attaches the part to the hand once both fingers have touched it for 0.3 s) is a privilege of the reference only.
        robot_cfg["grasping_mode"] = "assisted" if assisted else "physical"; self.assisted = bool(assisted)
        # Rubber finger pads: with no assisted grasp a part is held by squeezing it, so the pads grip well (the author's
        # decision, 2026-11-26). OmniGibson puts this material on the finger links only.
        robot_cfg["finger_static_friction"] = FINGER_FRICTION; robot_cfg["finger_dynamic_friction"] = FINGER_FRICTION
        robot_cfg["sensor_config"] = {"VisionSensor": {"sensor_kwargs": {"image_height": cam_res, "image_width": cam_res}}}
        objs = []
        fwd = np.array([math.cos(self.yaw), math.sin(self.yaw), 0.0]); left = np.array([-math.sin(self.yaw), math.cos(self.yaw), 0.0])
        if station == "track":
            usd = PR.write_prop(tmp / "chute.usd", PR.chute_boxes(), rgb=(0.62, 0.64, 0.68), friction=self.friction, roughness=0.3)
            objs.append({"type": "USDObject", "name": "chute", "usd_path": usd, "fixed_base": True, "position": self.low_end.tolist(),
                         "orientation": quat_xyzw(pitch=-self.slope, yaw=self.yaw)})
            mid = self.low_end + fwd * 0.5 * PR.CHUTE["length"] * math.cos(self.slope)
            h = float(mid[2] + 0.5 * PR.CHUTE["length"] * math.sin(self.slope) - 0.12)
            objs.append({"type": "PrimitiveObject", "name": "chute_stand", "primitive_type": "Cube", "fixed_base": True, "scale": [0.12, 0.3, h],
                         "position": [float(mid[0] + 0.25 * fwd[0]), float(mid[1] + 0.25 * fwd[1]), h / 2], "orientation": quat_xyzw(yaw=self.yaw), "rgba": [0.25, 0.25, 0.28, 1.0]})
        elif station == "drop":
            F = PR.FEEDER; top = float(self.low_end[2]); back = self.low_end + fwd * (F["depth"] / 2 + F["arm"])
            objs.append({"type": "PrimitiveObject", "name": "feeder", "primitive_type": "Cube", "fixed_base": True, "scale": [F["depth"], F["window"], F["thick"]],
                         "position": (self.low_end + np.array([0, 0, F["thick"] / 2])).tolist(), "orientation": quat_xyzw(yaw=self.yaw), "rgba": [0.85, 0.55, 0.10, 1.0]})
            objs.append({"type": "PrimitiveObject", "name": "feeder_arm", "primitive_type": "Cube", "fixed_base": True, "scale": [F["arm"] + 0.06, 0.06, 0.04],
                         "position": (self.low_end + fwd * (F["depth"] / 2 + F["arm"] / 2) + np.array([0, 0, F["thick"] / 2])).tolist(), "orientation": quat_xyzw(yaw=self.yaw), "rgba": [0.25, 0.25, 0.28, 1.0]})
            objs.append({"type": "PrimitiveObject", "name": "feeder_post", "primitive_type": "Cube", "fixed_base": True, "scale": [0.06, 0.06, top + F["thick"]],
                         "position": [float(back[0]), float(back[1]), (top + F["thick"]) / 2], "orientation": quat_xyzw(yaw=self.yaw), "rgba": [0.25, 0.25, 0.28, 1.0]})
        else:
            B = PR.BELT; self.n_slats = int(round(B["length"] / B["pitch"]))
            for k in range(self.n_slats):
                p = self.low_end + fwd * (B["pitch"] * (k + 0.5)) - np.array([0, 0, B["slat_thick"] / 2])
                objs.append({"type": "PrimitiveObject", "name": f"slat{k}", "primitive_type": "Cube", "scale": [B["pitch"] - 0.004, B["width"], B["slat_thick"]],
                             "position": p.tolist(), "orientation": quat_xyzw(yaw=self.yaw), "rgba": [0.12, 0.12, 0.13, 1.0], "mass": 200.0})
            mid = self.low_end + fwd * B["length"] / 2
            objs.append({"type": "PrimitiveObject", "name": "belt_bed", "primitive_type": "Cube", "fixed_base": True, "scale": [B["length"] - 0.1, B["width"] + 0.08, float(self.low_end[2]) - 0.08],
                         "position": [float(mid[0]), float(mid[1]), (float(self.low_end[2]) - 0.08) / 2], "orientation": quat_xyzw(yaw=self.yaw), "rgba": [0.3, 0.32, 0.36, 1.0]})
        tote = PR.write_prop(tmp / "tote.usd", PR.tote_boxes(), rgb=(0.10, 0.28, 0.62), friction=0.8)
        objs.append({"type": "USDObject", "name": "tote", "usd_path": tote, "fixed_base": True, "position": self.tote_at.tolist()})
        objs.append({"type": "PrimitiveObject", "name": "tote_stand", "primitive_type": "Cube", "fixed_base": True, "scale": [0.5, 0.4, float(self.tote_at[2]) - 0.014],
                     "position": [float(self.tote_at[0]), float(self.tote_at[1]), (float(self.tote_at[2]) - 0.014) / 2], "rgba": [0.25, 0.25, 0.28, 1.0]})
        side = np.array([0.55, 0.95, 0.5])
        objs.append({"type": "PrimitiveObject", "name": "side_table", "primitive_type": "Cube", "fixed_base": True, "scale": [0.5, 0.4, float(side[2])],
                     "position": [float(side[0]), float(side[1]), float(side[2]) / 2], "rgba": [0.45, 0.36, 0.26, 1.0]})
        for k, n in enumerate(self.item_names):
            objs.append({"type": "USDObject", "name": f"part{k}", "usd_path": catalog[n]["usd"], "position": (PARK + [0.45 * k, 0, 0]).tolist()})
        self.bystander_home = []
        for k, n in enumerate(self.bystander_names):
            p = side + np.array([-0.15 + 0.15 * k, g.uniform(-0.1, 0.1), catalog[n]["size_m"][2] / 2 + 0.01])
            self.bystander_home.append(p); objs.append({"type": "USDObject", "name": f"bystander{k}", "usd_path": catalog[n]["usd"], "position": p.tolist()})
        cfg = {"env": {"action_frequency": HZ, "rendering_frequency": HZ, "physics_frequency": 120},
               "scene": {"type": "Scene", "use_floor_plane": True, "floor_plane_visible": not room, "floor_plane_color": (0.2, 0.22, 0.25)},
               "robots": [robot_cfg], "objects": objs, "task": {"type": "DummyTask"}}
        self.env = og.Environment(configs=cfg); self.scene = self.env.scenes[0]; self.robot = self.scene.robots[0]
        og.sim.stop(); self.robot.base_footprint_link.mass = EVAL_BASE_LINK_MASS
        if room:
            from pxr import Gf, UsdGeom
            with og.sim.editing_usd():
                rp = og.sim.stage.DefinePrim("/World/backdrop_room", "Xform"); rp.GetReferences().AddReference(ROOM_USD)
                UsdGeom.Xformable(rp).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
        og.sim.play(); self.env.reset()
        if self._start_lock is not None: self._start_lock.close(); self._start_lock = None
        reg = lambda n: self.scene.object_registry("name", n)
        self.parts = [reg(f"part{k}") for k in range(n_parts_all)]; self.bystanders = [reg(f"bystander{k}") for k in range(len(self.bystander_names))]
        self.tote = reg("tote"); self.catalog = catalog
        self.slats = [reg(f"slat{k}") for k in range(getattr(self, "n_slats", 0))]
        for s in self.slats: s.root_link.disable_gravity()
        self.mass_errors = 0
        for part, n, ev in zip(self.parts, self.item_names, self.plan.events):
            try: part.root_link.mass = float(catalog[n]["mass_kg"] * c.get("mass_scale", 1.0) * ev.target.get("mass_scale", 1.0))
            except Exception: self.mass_errors += 1
        self.fwd, self.left = fwd, left
        self.up_chute = np.array([math.cos(self.slope) * fwd[0], math.cos(self.slope) * fwd[1], math.sin(self.slope)])
        self.fingers = {s: list(self.robot.finger_links[s]) for s in ("left", "right")}
        self.eefs = {s: self.robot.eef_links[s] for s in ("left", "right")}
        self.sensors = {cam: next(v for k, v in self.robot.sensors.items() if link in k) for cam, link in CAMERAS.items()}
        q0 = self.robot.get_joint_positions().clone(); self.hold = np.zeros(R.ACTION_DIM)
        for n in ("trunk", "arm_left", "arm_right"): self.hold[R.ACTION_SLICES[n]] = q0[th.as_tensor(self._dof(n))].cpu().numpy()
        self.hold[R.ACTION_SLICES["gripper_left"]] = 1.0; self.hold[R.ACTION_SLICES["gripper_right"]] = 1.0
        self.last_row = self.hold.copy()
        self.step_id, self.frames_charged, self.done = 0, 0, False; self.late_steps = 0
        self.event_i, self.event_t, self.gap, self.phase = -1, 0, 0, "gap"
        self.records, self.bystander_hit = [], [False] * len(self.bystanders)
        # The feed runs on its own timetable, fixed before the shift starts: part k is let go at release_at[k] (7 to 11 s
        # after the one before) and appears at the feed point release_step[k] steps earlier. Nothing the robot does moves
        # it (the author's decision of 2026-11-26: the controller does not decide when a part comes).
        gt = np.random.default_rng([41, seed]); rel = []; t_next = 2 * HZ + int(self.plan.events[0].release_step)
        for k, e in enumerate(self.plan.events):
            rel.append(int(t_next)); nxt = self.plan.events[k + 1].release_step if k + 1 < len(self.plan.events) else 0
            t_next = max(t_next + int(gt.uniform(7.0, 11.0) * HZ), t_next + HZ + int(nxt))
        self.release_at = rel; self.staged_at = [r - int(e.release_step) for r, e in zip(rel, self.plan.events)]
        self.score = SH.ShiftScore(self.plan); self.log = []
        if self.rack:
            for k, at in self.rack["hang_at"].items():
                if at <= 0: self._hang(k)

    # ------------------------------------------------------------------ helpers
    def _dof(self, name):
        from omnigibson.controllers.controller_view import ControllerView
        return ControllerView.get_dof_idx(self.robot.controllers[name][0]).cpu().numpy()

    def _pos(self, o): return o.get_position_orientation()[0].cpu().numpy().astype(float)
    def _quat(self, o): return o.get_position_orientation()[1].cpu().numpy().astype(float)
    def _set(self, o, p, q=None):
        th = self.th
        o.set_position_orientation(position=th.tensor(np.asarray(p, float), dtype=th.float32), orientation=th.tensor(q or [0, 0, 0, 1.0], dtype=th.float32))

    def _touch(self, query, with_set):
        from omnigibson.utils.usd_utils import RigidContactAPI
        return bool(RigidContactAPI.is_in_contact(scene_idx=self.scene.idx, query_set=set(query), with_set=list(with_set), ignore_set=None, current_only=True))

    def _in_tote(self, pos):
        local = rot(self._quat(self.tote)).T @ (np.asarray(pos) - self._pos(self.tote)); (a, b, h) = PR.TOTE["inner"]
        return bool(abs(local[0]) < a / 2 and abs(local[1]) < b / 2 and -0.01 < local[2] < h + 0.30)     # parts pile up over a shift

    # -------------------------------------------------------------------- events
    def _hang(self, k):
        """Hang rod k at its place of the rack (it stays there, held, until its own event lets it go)."""
        sl = self.rack["slot_of"][k]; row, col = divmod(sl, RACK["cols"]); L = float(self.catalog[self.item_names[k]]["size_m"][2])
        pos = (self.low_end + self.fwd * ((row - (RACK["rows"] - 1) / 2) * RACK["pitch"]) + self.left * ((col - (RACK["cols"] - 1) / 2) * RACK["pitch"])
               - np.array([0, 0, L / 2 + 0.005]))
        q = quat_xyzw(yaw=self.yaw); self.hung[k] = (pos, q); self._set(self.parts[k], pos, q); self.parts[k].keep_still()

    def _next_event(self):
        self.event_i += 1
        if self.event_i >= len(self.plan.events):
            self.done, self.phase = True, "done"; return
        ev = self.plan.events[self.event_i]; part = self.parts[self.event_i]
        size = np.array(self.catalog[self.item_names[self.event_i]]["size_m"])
        lane = float(ev.target.get("position", [0, 0, 0])[1] if self.station == "belt" else self.plan.constants.get("belt_lane", 0.0))
        lane = float(np.clip(lane, -0.15, 0.15))
        if self.station == "track":
            along = PR.CHUTE["length"] - 0.20
            self.spawn = self.low_end + self.up_chute * along + self.left * lane + np.array([0, 0, 1.0]) * (max(size) / 2 + 0.02)
            self.spawn_vel = -self.up_chute * float(ev.target.get("speed", 0.2))
        elif self.station == "drop":
            off = ev.target.get("offset", [0.0, 0.0])
            self.spawn = self.low_end + self.fwd * float(off[0]) + self.left * float(off[1]) - np.array([0, 0, size[2] / 2 + 0.005])
            self.spawn_vel = np.zeros(3)
            if self.rack:
                for k, at in self.rack["hang_at"].items():
                    if at == self.event_i and at > 0: self._hang(k)       # the place emptied by the last event gets a new rod
                self.spawn = self.hung[self.event_i][0]
        else:
            self.spawn = self.low_end + self.fwd * (PR.BELT["length"] - 0.2) + self.left * lane + np.array([0, 0, size[2] / 2 + 0.03])
            self.spawn_vel = -self.fwd * self.belt_speed
        item_yaw = float(ev.target.get("yaw", 0.0))
        if self.station == "track":   # long side down the chute, so the part passes a gripper; short cans go across and roll
            roller = self.item_names[self.event_i] in ("005_tomato_soup_can", "007_tuna_fish_can")
            item_yaw = (0.0 if roller or size[0] >= size[1] else math.pi / 2) + float(ev.target.get("mass_scale", 1.0) - 1.0) * 1.5
        self.spawn_q = quat_xyzw(yaw=self.yaw + item_yaw)
        if self.rack: self.spawn_q = self.hung[self.event_i][1]
        self.phase, self.event_t, self.held_run, self.was_held, self.touched, self.still_run = "staged", 0, 0, False, False, 0
        self._prev_rel = None
        self._set(part, self.spawn, self.spawn_q); part.keep_still()
        self.rec = {"index": self.event_i, "item": self.item_names[self.event_i], "kind": ev.kind, "start_step": self.step_id}

    def _end_event(self, outcome, reason):
        part = self.parts[self.event_i]; ev = self.plan.events[self.event_i]
        if self.rack: self.hung.pop(self.event_i, None)
        credit = CREDIT["placed"] if outcome == "placed" else (CREDIT["held"] if self.was_held else 0.0)
        self.rec.update(outcome=outcome, reason=reason, held=bool(self.was_held), touched=bool(self.touched), credit=credit, end_step=self.step_id)
        self.records.append(self.rec); self.score.record(ev, credit >= 1.0, reason, self.event_t)
        if outcome != "placed":                       # clear a lost part away; a placed one stays in the tote
            self._set(part, PARK + [0.45 * self.event_i, 0, 0]); part.keep_still()
        self.phase, self.gap = "gap", 0

    def _tick(self):
        """Once per control step, after physics: drive the feed point, judge the part."""
        th = self.th
        # Slat conveyor: every slat is put on its ideal place each control step (constant speed along the belt, recycled past
        # the open end) and given the belt's velocity, which is what carries the parts. Velocity alone was not enough: a
        # hand or a part pressing on a slat tipped it and the belt sagged into a staircase (job 62700202, head camera).
        if self.slats:
            self.belt_travel = getattr(self, "belt_travel", 0.0) + self.belt_speed / HZ; L = PR.BELT["pitch"] * self.n_slats
            v = th.tensor(-self.fwd * self.belt_speed, dtype=th.float32); q = quat_xyzw(yaw=self.yaw)
            for k, s in enumerate(self.slats):
                along = (PR.BELT["pitch"] * (k + 0.5) - self.belt_travel) % L
                self._set(s, self.low_end + self.fwd * along - np.array([0, 0, PR.BELT["slat_thick"] / 2]), q)
                s.set_linear_velocity(v); s.set_angular_velocity(th.zeros(3))
        # A bystander's home is where it has come to rest, not where it was spawned: items are set down 1 cm above the side
        # table and some settle or roll more than BYSTANDER_MOVE on their own, which charged a penalty to a robot that
        # had not moved (seen 2026-11-26, seed 2100, a controller holding still for 60 steps). The first part is staged
        # no earlier than step 60.
        if self.step_id == BYSTANDER_SETTLE: self.bystander_home = [self._pos(b).copy() for b in self.bystanders]
        for k, b in enumerate(self.bystanders):
            if self.step_id > BYSTANDER_SETTLE and not self.bystander_hit[k] and np.linalg.norm(self._pos(b) - self.bystander_home[k]) > BYSTANDER_MOVE: self.bystander_hit[k] = True
        if self.phase == "done": return
        if self.rack:
            for k, (hp, hq) in self.hung.items(): self._set(self.parts[k], hp, hq); self.parts[k].keep_still()
            # Hands off the rack: a finger on any rod that still hangs ends the current rod's event.
            if self.phase in ("staged", "live") and any(self._touch([l], [self.parts[k] for k in self.hung if not (k == self.event_i and self.phase == "live")]) for links in self.fingers.values() for l in links if self.hung):
                if self.event_i in self.hung: self.hung.pop(self.event_i)
                self.touched = True; return self._end_event("lost", "early_touch")
        if self.phase == "gap":
            self.gap += 1
            nxt = self.event_i + 1
            if nxt >= len(self.plan.events): self.done, self.phase = True, "done"
            elif self.step_id >= self.staged_at[nxt]: self._next_event()
            return
        ev = self.plan.events[self.event_i]; part = self.parts[self.event_i]
        self.event_t += 1
        # The next part is about to appear: this one's time is up, wherever it is (in the hand, in the air, on the feed).
        if self.event_i + 1 < len(self.plan.events) and self.step_id >= self.staged_at[self.event_i + 1] - 1: return self._end_event("lost", "next_part")
        if self.phase == "staged":
            if self.event_t <= ev.release_step:
                self._set(part, self.spawn, self.spawn_q); part.keep_still()
                if self.station == "drop" and any(self._touch([l], [part]) for links in self.fingers.values() for l in links):
                    self.touched = True; return self._end_event("lost", "early_touch")     # hands off a part the feeder still holds
                return
            part.set_linear_velocity(th.tensor(self.spawn_vel, dtype=th.float32)); self.phase = "live"
            if self.rack: self.hung.pop(self.event_i, None)
        pos = self._pos(part)
        fing = {s: [self._touch([l], [part]) for l in links] for s, links in self.fingers.items()}
        first_touch = any(any(f) for f in fing.values()) and not self.touched
        if any(any(f) for f in fing.values()): self.touched = True
        if self.station == "drop" and first_touch and float(self.spawn[2] - pos[2]) < MIN_FALL: return self._end_event("lost", "early_touch")
        rel = {s: pos - self._pos(self.eefs[s]) for s in ("left", "right")}; prev = self._prev_rel or rel; self._prev_rel = rel
        on_prop = self._touch(part.links.values(), [o for o in ([self.scene.object_registry("name", "chute")] if self.station == "track" else [self.scene.object_registry("name", n) for n in ("feeder", "feeder_arm", "feeder_post")] if self.station == "drop" else self.slats) if o is not None] + [self.tote])
        try: low = float(part.aabb[0][2])
        except Exception: low = float(pos[2]) - 0.02
        # Lost to the floor: on it, or well below the feed surface with no finger on it (thin parts can end on a base).
        floor = low < 0.015 or (pos[2] < self.low_end[2] - 0.30 and not any(any(f) for f in fing.values()) and float(np.linalg.norm(part.get_linear_velocity().cpu().numpy())) < 0.05 and not self._in_tote(pos))
        holding = any(all(fing[s]) and np.linalg.norm(rel[s] - prev[s]) * HZ < REST_SPEED for s in fing) and not on_prop and not floor
        self.held_run = self.held_run + 1 if holding else 0
        if self.held_run >= HOLD_STEPS: self.was_held = True
        in_tote = self._in_tote(pos)
        speed = float(np.linalg.norm(part.get_linear_velocity().cpu().numpy()))
        if in_tote and not any(any(f) for f in fing.values()) and speed < REST_SPEED: return self._end_event("placed", "in_tote")
        if floor: return self._end_event("lost", "hit_floor")
        # Jam: a part that stands still on the feed surface for a second without being held blocks the line. This is
        # what rules out parking a hand in the part's way and picking it up at leisure: it must be taken while it moves.
        self.still_run = self.still_run + 1 if (on_prop and not in_tote and not holding and speed < 0.03) else 0
        if self.still_run >= HZ: return self._end_event("lost", "jam")
        if self.station == "track" and not any(any(f) for f in fing.values()) and float((pos - self.low_end) @ self.up_chute) < 0.5 * max(self.catalog[self.item_names[self.event_i]]["size_m"]) + 0.05 and speed < 0.15 and abs(float((pos - self.low_end) @ self.left)) < 0.3 and on_prop:
            return self._end_event("lost", "end_stop")
        if self.event_t - ev.release_step >= ev.limit_steps: return self._end_event("lost", "time_limit")

    # ---------------------------------------------------------------------- step
    def _advance(self, row, render=False):
        """One 30 Hz control step. Fast path (default): apply the action and step physics without rendering or reading any
        sensor: Environment.step() renders and reads back every camera on every step, which nobody asked for. Only the
        steps that lead to a requested frame are rendered (see step()). CELL_FAST=0 restores Environment.step()."""
        a = self.th.as_tensor(row, dtype=self.th.float32)
        if FAST:
            self.robot.apply_action(a)
            with self.og.sim.render_on_step(bool(render)): self.og.sim.step()
        else:
            self.env.step(a.reshape(1, -1))
        self.step_id += 1; self._tick()

    def step(self, row=None, cameras=(), late=0):
        """One action row, then the fixed price of the frames asked for. Returns the observation.
        late: control steps that went by while the controller was still working out this row. The cell does not wait: it
        runs them first, with the arms and grippers on their previous targets and the base stopped, and only then applies
        the row."""
        if not self.done and late > 0:
            hold = self.last_row.copy(); hold[self.R.ACTION_SLICES["base"]] = 0.0
            for _ in range(int(min(late, 900))):
                if self.done: break
                self._advance(hold); self.late_steps += 1
        if not self.done:
            row = self.hold if row is None else np.asarray(row, float)
            frames = [c for c in cameras if c in self.sensors]
            # Steps that lead to a requested frame are rendered, every one of them (the action's own step and the three
            # steps each frame costs), as Environment.step() renders every step: a frame read after unrendered steps and
            # a bare render() showed the scene of the PREVIOUS request (found 2026-11-25 by comparing with the slow path).
            self.last_row = row.copy(); self._advance(row, render=bool(frames))
            if frames:
                hold = row.copy(); hold[self.R.ACTION_SLICES["base"]] = 0.0
                for _ in range(FRAME_COST_STEPS * len(frames)):
                    if self.done: break
                    self._advance(hold, render=True)
                self.frames_charged += len(frames)
        return self.observe(cameras)

    def observe(self, cameras=()):
        def flatten_obs_dict(obs, parent=""):
            out = {}
            for k, v in obs.items():
                key = f"{parent}::{k}" if parent else k
                out.update(flatten_obs_dict(v, key) if isinstance(v, dict) else {key: v})
            return out
        if FAST:
            p = self.robot.get_proprioception()[0]
            if cameras and self.step_id == 0:          # the first observation, before any step: nothing has been rendered yet
                for _ in range(4): self.og.sim.render()
        else:
            obs, _ = self.env.get_obs(); obs = obs[0] if isinstance(obs, (list, tuple)) else obs
            p = flatten_obs_dict(obs)[f"{self.robot.name}::proprio"]
        out = {"t": self.step_id, "done": self.done, "proprio": np.asarray(p.detach().cpu().numpy() if hasattr(p, "detach") else p, float).reshape(-1)}
        bp, bq = self.robot.get_position_orientation(); bp, Rb = bp.cpu().numpy().astype(float), rot(bq.cpu().numpy().astype(float))
        out["odometry"] = [float(bp[0]), float(bp[1]), float(math.atan2(Rb[1, 0], Rb[0, 0]))]
        for cam in cameras:
            s = self.sensors.get(cam)
            if s is None: continue
            o = s.get_obs()[0]; f = lambda x: x.detach().cpu().numpy() if hasattr(x, "detach") else np.asarray(x)
            cp, cq = s.get_position_orientation(); cp = cp.cpu().numpy().astype(float); Rc = rot(cq.cpu().numpy().astype(float))
            # From the camera's own USD attributes (as RAX does): VisionSensor.intrinsic_matrix needs a finished render
            # and asserted "degenerate" under load.
            w, hgt = int(s.image_width), int(s.image_height); fpx = float(s.focal_length) * w / float(s.horizontal_aperture)
            K = np.array([[fpx, 0.0, w / 2.0], [0.0, fpx, hgt / 2.0], [0.0, 0.0, 1.0]])
            out[cam] = {"rgb": np.ascontiguousarray(f(o["rgb"])[..., :3]).astype(np.uint8), "depth": f(o["depth_linear"]).astype(np.float32),
                        "K": np.asarray(K, float), "pos_base": Rb.T @ (cp - bp), "rot_base": Rb.T @ Rc}
        return out

    # --------------------------------------------------------------------- truth
    def truth(self):
        """Ground truth for the privileged reference and for tests. Never part of an observation."""
        t = {"phase": self.phase, "event": self.event_i, "station": self.station, "low_end": self.low_end.tolist(), "yaw": self.yaw,
             "slope": self.slope, "belt_speed": self.belt_speed, "tote": self.tote_at.tolist(), "step": self.step_id,
             "next_staged_in": (int(self.staged_at[self.event_i + 1] - self.step_id) if self.event_i + 1 < len(self.plan.events) else None),
             "next_release_in": (int(self.release_at[self.event_i + 1] - self.step_id) if self.event_i + 1 < len(self.plan.events) else None)}
        t["assisted"] = self.assisted
        bp, bq = self.robot.get_position_orientation(); t["base_pos"] = bp.cpu().numpy().astype(float).tolist(); t["base_quat"] = bq.cpu().numpy().astype(float).tolist()
        for s in ("left", "right"):
            t[f"eef_{s}_pos"] = self._pos(self.eefs[s]).tolist(); t[f"eef_{s}_quat"] = self._quat(self.eefs[s]).tolist()
            t[f"fingers_{s}"] = np.mean([self._pos(l) for l in self.fingers[s]], axis=0).tolist()
            try: t[f"finger_low_{s}"] = float(min(float(l.aabb[0][2]) for l in self.fingers[s]))
            except Exception: t[f"finger_low_{s}"] = None
            t[f"finger_gap_{s}"] = float(np.linalg.norm(self._pos(self.fingers[s][0]) - self._pos(self.fingers[s][1])))
        try:   # assisted grasp: the simulator attaches a part to the hand once both fingers have held it for 0.3 s
            held = self.robot._ag_obj_in_hand.get("right"); t["attached_right"] = bool(held is not None and self.phase == "live" and held is self.parts[self.event_i])
        except Exception: t["attached_right"] = None
        t["finger_contact_right"] = [bool(self._touch([l], [self.parts[self.event_i]])) for l in self.fingers["right"]] if self.phase == "live" else [False, False]
        try: t["is_grasping_right"] = str(self.robot.is_grasping(arm="right", candidate_obj=self.parts[self.event_i] if self.phase == "live" else None))
        except Exception as exc: t["is_grasping_right"] = "err:" + type(exc).__name__
        for s in ("left", "right"):
            pass
        t["friction"] = self.friction; t["fwd"] = self.fwd.tolist(); t["left"] = self.left.tolist(); t["up_chute"] = self.up_chute.tolist()
        if self.phase in ("staged", "live"):
            part = self.parts[self.event_i]; ev = self.plan.events[self.event_i]
            t.update(item=self.item_names[self.event_i], pos=self._pos(part).tolist(), quat=self._quat(part).tolist(),
                     vel=part.get_linear_velocity().cpu().numpy().astype(float).tolist(), size=self.catalog[self.item_names[self.event_i]]["size_m"], aabb=[[float(x) for x in part.aabb[0]], [float(x) for x in part.aabb[1]]],
                     release_in=int(ev.release_step - self.event_t), event_t=self.event_t, was_held=self.was_held, rod=bool(getattr(self, "rods", False)), spawn=np.asarray(self.spawn, float).tolist(), min_fall=MIN_FALL)
        return t

    def summary(self):
        n = len(self.plan.events); credit = [0.0] * n
        for r in self.records: credit[r["index"]] = r["credit"]
        pen = 0.5 * sum(self.bystander_hit)
        third = max(1, n // 3)
        return {"station": self.station, "seed": self.plan.seed, "parts": n, "events_done": len(self.records), "steps": self.step_id,
                "frames_charged": self.frames_charged, "late_steps": self.late_steps, "score": max(0.0, (sum(credit) - pen) / n), "placed": sum(c >= 1.0 for c in credit),
                "held_only": sum(c == 0.5 for c in credit), "bystanders_moved": int(sum(self.bystander_hit)),
                "score_first_third": sum(credit[:third]) / third, "score_last_third": sum(credit[-third:]) / third,
                "mass_errors": self.mass_errors, "records": self.records,
                "constants": {k: v for k, v in self.plan.constants.items() if not isinstance(v, (list, dict))}}
