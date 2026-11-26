"""The packing cell: one shift at one feed point (chute or slat conveyor) with an R1Pro, in OmniGibson.

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
from outcomes import inside_tote, layout_offsets, released_and_resting, summarize_credits

HZ = 30
FRAME_COST_STEPS = 3
FEED_INTERVAL_S = (9.0, 13.0)   # seconds between two releases: the line feeds at random intervals of its own
LATENCY_STEPS = (3, 6)          # 0.10 to 0.20 s between a frame being taken and the controller receiving it
FAST = os.environ.get("CELL_FAST", "1") != "0"
ITEMS_JSON = Path(os.environ.get("CELL_ITEMS", "/opt/cell_items/items.json"))
ROOM_USD = os.environ.get("CELL_ROOM", "/opt/isaac_assets/Isaac/Environments/Simple_Room/simple_room.usd")
class R:      # R1Pro action row (30 Hz): joint position targets except the base (velocity) and grippers (-1 close .. 1 open)
    ACTION_DIM = 23
    ACTION_SLICES = {"base": slice(0, 3), "trunk": slice(3, 7), "arm_left": slice(7, 14), "gripper_left": slice(14, 15),
                     "arm_right": slice(15, 22), "gripper_right": slice(22, 23)}
CAMERAS = {"head": "zed_link", "left_wrist": "left_realsense_link", "right_wrist": "right_realsense_link"}
LOW_END = np.array([0.38, -0.30, 0.85])        # chute's low end / belt's open end (surface height), nominal: in the right arm's reach
TOTE_AT = np.array([0.10, -0.70, 0.55])        # centre of the tote's inner floor, nominal
PARK = np.array([-8.0, 6.0, 0.5])
HOLD_STEPS, REST_SPEED, BYSTANDER_MOVE = 30, 0.05, 0.05
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
    def __init__(self, station: str, seed: int, n_parts: int = 20, cam_res: int = 320, overrides: dict | None = None, room: bool = True, salt: int = 0, assisted: bool = False):
        import sys
        import omnigibson as og, torch as th
        from omegaconf import OmegaConf
        from omnigibson.macros import gm
        # The R1Pro configuration OmniGibson's own evaluator uses, read directly (the evaluator module drags in
        # tele-operation and data-set packages this cell has no use for).
        DEFAULT_ROBOT_CONFIG_PATH = os.path.join(os.path.dirname(og.__file__), "eval", "r1pro.yaml"); EVAL_BASE_LINK_MASS = 250.0
        _to_plain_dict = lambda cfg: dict(OmegaConf.to_container(cfg, resolve=True))
        assert station in ("track", "belt")
        self.og, self.th, self.R, self.station = og, th, R, station
        gm.HEADLESS = True; gm.RENDER_VIEWER_CAMERA = False
        gm.USE_GPU_DYNAMICS = False; gm.ENABLE_TRANSITION_RULES = True      # as OmniGibson's evaluator sets them: PhysX on the CPU
        # One Kit app at a time through start-up and scene loading: two starting together on one card segfault.
        self._start_lock = None
        if os.environ.get("CELL_START_LOCK"):
            import fcntl
            self._start_lock = open(os.environ["CELL_START_LOCK"], "a"); fcntl.flock(self._start_lock, fcntl.LOCK_EX)
        og.launch()                                   # Kit first: the prop files below are written with Kit's own pxr
        self.plan = SH.make_shift("A", seed, n_parts, station=station, salt=salt)
        c = self.plan.constants
        c.update(overrides or {})
        g, self.low_end, self.yaw, self.tote_at = layout_offsets(seed, salt, LOW_END, TOTE_AT)
        # Camera latency: a frame reaches the controller this many 30 Hz steps after it was taken. Fixed for the shift, hidden,
        # drawn from a stream of its own (the shift's parts and layout are what they were before this constant existed).
        self.latency = int(np.random.default_rng([31, seed] + ([int(salt)] if salt else [])).integers(LATENCY_STEPS[0], LATENCY_STEPS[1] + 1))
        self.latency = int((overrides or {}).get("camera_latency_steps", self.latency)); self.pending = []
        # The line's schedule, fixed before the shift starts and independent of anything the robot does: part k is let go
        # at release_at[k], at random intervals; it is put on the chute staged_at[k], 1 to 3 s earlier. Its event is over at
        # the latest when the next part is put on. Drawn from a stream of its own, hidden.
        gs = np.random.default_rng([41, seed] + ([int(salt)] if salt else [])); self.release_at, self.staged_at = [], []
        t_rel = int(gs.uniform(4.0, 6.0) * HZ)
        for k in range(n_parts):
            self.release_at.append(t_rel); self.staged_at.append(t_rel - int(gs.uniform(1.0, 3.0) * HZ))
            t_rel += int(gs.uniform(FEED_INTERVAL_S[0], FEED_INTERVAL_S[1]) * HZ)
        self.shift_end = self.release_at[-1] + int(FEED_INTERVAL_S[1] * HZ)
        # Installed layout: nominal plus this shift's hidden offsets.
        self.slope = math.radians(c.get("track_slope_deg", 7.0)); self.friction = float(c.get("track_friction", 0.05))
        self.belt_speed = float(c.get("belt_speed", 0.3))
        tmp = Path(tempfile.mkdtemp(prefix="cell_props_"))
        catalog = json.loads(ITEMS_JSON.read_text())
        self.item_names = [e.target["item"] for e in self.plan.events]
        spare = [n for n in SH.CATALOG if n not in self.item_names]
        self.bystander_names = (spare + [self.item_names[int(k)] for k in g.permutation(n_parts)[:2]])[:3]

        robot_cfg = _to_plain_dict(OmegaConf.load(DEFAULT_ROBOT_CONFIG_PATH)); robot_cfg.pop("eval", None)
        robot_cfg["model"] = robot_cfg["model"].lower()
        robot_cfg.update(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0], obs_modalities=["proprio", "rgb", "depth_linear"])
        # Grasping is physical: a part stays in the hand by finger friction alone. OmniGibson's "assisted" mode (the simulator
        # attaches the part to the hand once both fingers have touched it for 0.3 s) is a privilege of the reference only.
        robot_cfg["grasping_mode"] = "assisted" if assisted else "physical"; self.assisted = bool(assisted)
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
        self.parts = [reg(f"part{k}") for k in range(n_parts)]; self.bystanders = [reg(f"bystander{k}") for k in range(len(self.bystander_names))]
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
        self.step_id, self.frames_charged, self.done = 0, 0, False
        self.event_i, self.event_t, self.gap, self.phase = -1, 0, 0, "gap"
        self.records, self.bystander_hit = [], [False] * len(self.bystanders)
        self.score = SH.ShiftScore(self.plan); self.log = []
        self._next_event()

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
        return inside_tote(pos, self._pos(self.tote), rot(self._quat(self.tote)), PR.TOTE["inner"])

    # -------------------------------------------------------------------- events
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
        else:
            self.spawn = self.low_end + self.fwd * (PR.BELT["length"] - 0.2) + self.left * lane + np.array([0, 0, size[2] / 2 + 0.03])
            self.spawn_vel = -self.fwd * self.belt_speed
        item_yaw = float(ev.target.get("yaw", 0.0))
        if self.station == "track":   # long side down the chute, so the part passes a gripper; short cans go across and roll
            roller = self.item_names[self.event_i] in ("005_tomato_soup_can", "007_tuna_fish_can")
            item_yaw = (0.0 if roller or size[0] >= size[1] else math.pi / 2) + float(ev.target.get("mass_scale", 1.0) - 1.0) * 1.5
        self.spawn_q = quat_xyzw(yaw=self.yaw + item_yaw)
        self.phase, self.event_t, self.held_run, self.was_held, self.touched, self.still_run = "staged", 0, 0, False, False, 0
        self._prev_rel = None
        self._set(part, self.spawn, self.spawn_q); part.keep_still()
        self.rec = {"index": self.event_i, "item": self.item_names[self.event_i], "kind": ev.kind, "start_step": self.step_id}

    def _deadline(self):
        """The step at which the current part's event is over whatever its state: one step before the next part goes on."""
        k = self.event_i
        return (self.staged_at[k + 1] - 1) if k + 1 < len(self.staged_at) else self.shift_end

    def _end_event(self, outcome, reason):
        part = self.parts[self.event_i]; ev = self.plan.events[self.event_i]
        credit = CREDIT["placed"] if outcome == "placed" else (CREDIT["held"] if self.was_held else 0.0)
        self.rec.update(outcome=outcome, reason=reason, held=bool(self.was_held), touched=bool(self.touched), credit=credit, end_step=self.step_id)
        self.records.append(self.rec); self.score.record(ev, credit >= 1.0, reason, self.event_t)
        if outcome != "placed":                       # clear a lost part away; a placed one stays in the tote
            self._set(part, PARK + [0.45 * self.event_i, 0, 0]); part.keep_still()
        self.phase, self.gap = "gap", 0

    def _tick(self):
        """Once per control step, after physics: drive the feed point, judge the part."""
        th = self.th
        for k, s in enumerate(self.slats):            # slat conveyor: constant speed, recycle past the open end
            p = self._pos(s); along = float((p - self.low_end) @ self.fwd)
            if along < 0.0:
                self._set(s, p + self.fwd * PR.BELT["pitch"] * self.n_slats, quat_xyzw(yaw=self.yaw)); p = self._pos(s)
            dz = (self.low_end[2] - PR.BELT["slat_thick"] / 2) - p[2]; dy = -float((p - self.low_end) @ self.left)
            v = -self.fwd * self.belt_speed + self.left * (5.0 * dy) + np.array([0, 0, 5.0 * dz])
            s.set_linear_velocity(th.tensor(v, dtype=th.float32)); s.set_angular_velocity(th.zeros(3))
        for k, b in enumerate(self.bystanders):
            if not self.bystander_hit[k] and np.linalg.norm(self._pos(b) - self.bystander_home[k]) > BYSTANDER_MOVE: self.bystander_hit[k] = True
        if self.phase == "done": return
        if self.phase == "gap":
            self.gap += 1
            # The line sets its own pace: part k goes on the chute at staged_at[k], whatever the robot is doing and however
            # the last part ended. Nothing a controller does moves that moment.
            if self.event_i + 1 >= len(self.plan.events): self.done, self.phase = True, "done"
            elif self.step_id >= self.staged_at[self.event_i + 1]: self._next_event()
            return
        ev = self.plan.events[self.event_i]; part = self.parts[self.event_i]
        self.event_t += 1
        if self.phase == "staged":
            if self.step_id < self.release_at[self.event_i]:
                self._set(part, self.spawn, self.spawn_q); part.keep_still(); return
            part.set_linear_velocity(th.tensor(self.spawn_vel, dtype=th.float32)); self.phase = "live"
        pos = self._pos(part)
        fing = {s: [self._touch([l], [part]) for l in links] for s, links in self.fingers.items()}
        if any(any(f) for f in fing.values()): self.touched = True
        rel = {s: pos - self._pos(self.eefs[s]) for s in ("left", "right")}; prev = self._prev_rel or rel; self._prev_rel = rel
        on_prop = self._touch(part.links.values(), [o for o in ([self.scene.object_registry("name", "chute")] if self.station == "track" else self.slats) if o is not None] + [self.tote])
        try: low = float(part.aabb[0][2])
        except Exception: low = float(pos[2]) - 0.02
        # Lost to the floor: on it, or well below the feed surface with no finger on it (thin parts can end on a base).
        floor = low < 0.015 or (pos[2] < self.low_end[2] - 0.30 and not any(any(f) for f in fing.values()) and float(np.linalg.norm(part.get_linear_velocity().cpu().numpy())) < 0.05 and not self._in_tote(pos))
        holding = any(all(fing[s]) and np.linalg.norm(rel[s] - prev[s]) * HZ < REST_SPEED for s in fing) and not on_prop and not floor
        self.held_run = self.held_run + 1 if holding else 0
        if self.held_run >= HOLD_STEPS: self.was_held = True
        in_tote = self._in_tote(pos)
        speed = float(np.linalg.norm(part.get_linear_velocity().cpu().numpy()))
        if released_and_resting(in_tote, any(any(f) for f in fing.values()), speed, REST_SPEED): return self._end_event("placed", "in_tote")
        if floor: return self._end_event("lost", "hit_floor")
        # Jam: a part that stands still on the feed surface for a second without being held blocks the line. This is
        # what rules out parking a hand in the part's way and picking it up at leisure: it must be taken while it moves.
        self.still_run = self.still_run + 1 if (on_prop and not in_tote and not holding and speed < 0.03) else 0
        if self.still_run >= HZ: return self._end_event("lost", "jam")
        if self.station == "track" and not any(any(f) for f in fing.values()) and float((pos - self.low_end) @ self.up_chute) < 0.5 * max(self.catalog[self.item_names[self.event_i]]["size_m"]) + 0.05 and speed < 0.15 and abs(float((pos - self.low_end) @ self.left)) < 0.3 and on_prop \
                and not in_tote and float((pos - self.low_end) @ self.up_chute) > -0.12 and not self._touch(part.links.values(), [self.tote]):
            # (at the chute's low end, not in or on the tote: a part just let go in the near half of the tote, still settling,
            # used to be taken for one lying at the end stop and was lost: found 2026-11-26 from parts that ended in the tote unplaced)
            return self._end_event("lost", "end_stop")
        if self.step_id >= self._deadline(): return self._end_event("lost", "time_limit")      # the next part is due

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

    def step(self, row=None, cameras=()):
        """One action row, then the fixed price of the frames asked for. Returns the observation."""
        if not self.done:
            row = self.hold if row is None else np.asarray(row, float)
            # Commands are bounded here, whatever a controller sends: base velocity and grippers to [-1, 1], joint targets to
            # a sane range. An unbounded command once drove the base pose to NaN and took the simulator down with it, which
            # would have been a task error instead of that controller's score.
            row = np.nan_to_num(row, nan=0.0, posinf=0.0, neginf=0.0).copy(); S = self.R.ACTION_SLICES
            for key in ("base", "gripper_left", "gripper_right"): row[S[key]] = np.clip(row[S[key]], -1.0, 1.0)
            for key in ("trunk", "arm_left", "arm_right"): row[S[key]] = np.clip(row[S[key]], -6.3, 6.3)
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
                taken = self._capture(frames)                      # taken now, delivered self.latency steps from now
                self.pending.append((self.step_id + self.latency, taken))
        return self.observe(())

    def _capture(self, cameras, bp=None, Rb=None):
        """The frames of the named cameras as they are now, each stamped with the step it was taken at."""
        if bp is None:
            bp, bq = self.robot.get_position_orientation(); bp, Rb = bp.cpu().numpy().astype(float), rot(bq.cpu().numpy().astype(float))
        out = {}
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
                        "K": np.asarray(K, float), "pos_base": Rb.T @ (cp - bp), "rot_base": Rb.T @ Rc, "t_capture": int(self.step_id)}
        return out

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
        if cameras: out.update(self._capture(cameras, bp, Rb))          # asked for directly (the first observation): no latency
        due = [f for f in self.pending if f[0] <= self.step_id]; self.pending = [f for f in self.pending if f[0] > self.step_id]
        for _, taken in due: out.update(taken)                         # a later frame of the same camera replaces an earlier one
        return out

    # --------------------------------------------------------------------- truth
    def truth(self):
        """Ground truth for the privileged reference and for tests. Never part of an observation."""
        t = {"phase": self.phase, "event": self.event_i, "station": self.station, "low_end": self.low_end.tolist(), "yaw": self.yaw,
             "slope": self.slope, "belt_speed": self.belt_speed, "tote": self.tote_at.tolist(), "step": self.step_id}
        bp, bq = self.robot.get_position_orientation(); t["base_pos"] = bp.cpu().numpy().astype(float).tolist(); t["base_quat"] = bq.cpu().numpy().astype(float).tolist()
        for s in ("left", "right"):
            t[f"eef_{s}_pos"] = self._pos(self.eefs[s]).tolist(); t[f"eef_{s}_quat"] = self._quat(self.eefs[s]).tolist()
            t[f"fingers_{s}"] = np.mean([self._pos(l) for l in self.fingers[s]], axis=0).tolist()
            try: t[f"finger_low_{s}"] = float(min(float(l.aabb[0][2]) for l in self.fingers[s]))
            except Exception: t[f"finger_low_{s}"] = None
            t[f"finger_gap_{s}"] = float(np.linalg.norm(self._pos(self.fingers[s][0]) - self._pos(self.fingers[s][1])))
        t["assisted"] = self.assisted
        t["proprio"] = [float(v) for v in np.asarray(self.observe(())["proprio"], float)]      # the true readings (a grader may hand the controller shifted ones)
        try:   # assisted grasp: the simulator attaches a part to the hand once both fingers have held it for 0.3 s
            held = self.robot._ag_obj_in_hand.get("right"); t["attached_right"] = bool(held is not None and self.phase == "live" and held is self.parts[self.event_i])
        except Exception: t["attached_right"] = None
        t["finger_contact_right"] = [bool(self._touch([l], [self.parts[self.event_i]])) for l in self.fingers["right"]] if self.phase == "live" else [False, False]
        try: t["is_grasping_right"] = str(self.robot.is_grasping(arm="right", candidate_obj=self.parts[self.event_i] if self.phase == "live" else None))
        except Exception as exc: t["is_grasping_right"] = "err:" + type(exc).__name__
        # What is coming, seconds ahead: the reference is told the future a camera cannot show. None of this ever reaches
        # an ordinary controller.
        try:
            hl = self.robot._ag_obj_in_hand.get("left"); t["attached_left"] = bool(hl is not None and self.phase == "live" and hl is self.parts[self.event_i])
        except Exception: t["attached_left"] = None
        t["finger_contact_left"] = [bool(self._touch([l], [self.parts[self.event_i]])) for l in self.fingers["left"]] if self.phase == "live" else [False, False]
        t["accel_slide"] = float(9.81 * (math.sin(self.slope) - self.friction * math.cos(self.slope)))      # along the chute, a part that slides
        t["accel_roll"] = float(9.81 * math.sin(self.slope) * 2.0 / 3.0)                                    # a can rolling without slipping
        t["chute"] = dict(PR.CHUTE); t["lane"] = float(np.clip(self.plan.constants.get("belt_lane", 0.0), -0.15, 0.15)); t["hz"] = HZ
        nxt = self.event_i + (0 if self.phase in ("staged", "live") else 1)
        t["upcoming"] = [{"event": k, "item": self.item_names[k], "size": self.catalog[self.item_names[k]]["size_m"], "mass": float(self.catalog[self.item_names[k]]["mass_kg"]),
                          "staged_in": int(self.staged_at[k] - self.step_id), "release_in": int(self.release_at[k] - self.step_id), "release_speed": float(self.plan.events[k].target.get("speed", 0.2)),
                          "rolls": self.item_names[k] in ("005_tomato_soup_can", "007_tuna_fish_can")} for k in range(max(nxt, 0), min(nxt + 4, len(self.plan.events)))]
        if self.phase == "gap" and self.event_i + 1 < len(self.staged_at): t["staged_in"] = int(max(0, self.staged_at[self.event_i + 1] - self.step_id))
        t["friction"] = self.friction; t["fwd"] = self.fwd.tolist(); t["left"] = self.left.tolist(); t["up_chute"] = self.up_chute.tolist()
        if self.phase in ("staged", "live"):
            part = self.parts[self.event_i]; ev = self.plan.events[self.event_i]
            t.update(item=self.item_names[self.event_i], pos=self._pos(part).tolist(), quat=self._quat(part).tolist(),
                     vel=part.get_linear_velocity().cpu().numpy().astype(float).tolist(), size=self.catalog[self.item_names[self.event_i]]["size_m"], aabb=[[float(x) for x in part.aabb[0]], [float(x) for x in part.aabb[1]]],
                     release_in=int(self.release_at[self.event_i] - self.step_id), event_t=self.event_t, was_held=self.was_held,
                     ang_vel=part.get_angular_velocity().cpu().numpy().astype(float).tolist(), spawn=np.asarray(self.spawn, float).tolist(),
                     spawn_vel=np.asarray(self.spawn_vel, float).tolist(), limit_in=int(self._deadline() - self.step_id))
        return t

    def summary(self):
        n = len(self.plan.events); credit = [0.0] * n
        for r in self.records: credit[r["index"]] = r["credit"]
        return {"station": self.station, "seed": self.plan.seed, "parts": n, "events_done": len(self.records), "steps": self.step_id,
                "frames_charged": self.frames_charged, **summarize_credits(credit, sum(self.bystander_hit)),
                "mass_errors": self.mass_errors, "records": self.records, "camera_latency_steps": self.latency,
                "constants": {k: v for k, v in self.plan.constants.items() if not isinstance(v, (list, dict))}}
