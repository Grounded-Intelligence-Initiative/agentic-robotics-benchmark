"""The harvesting cell in Isaac Sim: one row section, the cart with the robot and its cutter-gripper, the two cameras.

`Cell(app, row, config)` builds the stage (`scene.py`), runs PhysX on the CPU and renders on the GPU only when an
action asks for a camera frame. `step(action)` executes one action of the wire (a motion of 1 to 40 control ticks of
the arm and the cart with the gripper command that goes with it, see `wire.py`) and returns the observation: the
robot's own readings, the external wrench at the flange, the cart's odometer, the gripper's opening and force, and the
frames that were asked for. Where a plant's member is and what state a truss is in never leave this module except
through `truth()` (the reference's privileged view, grader only) and `score()`.

Physics state decides everything that is scored: where each truss lies at the end and what happened to it on the
way (`rules.Rules.credit`).
"""

import base64
import math
import time

import numpy as np

from . import farm as F
from . import kin, rules, scene, sensors, wire

G = 9.81
ARM_SENSORS = ("ee_link", "wrist_3_link", "wrist_2_link", "wrist_1_link", "forearm_link")


def tf_T(tf):
    """PhysX transform row [x y z qx qy qz qw] -> 4x4."""
    return kin.pose(kin.quat_R((tf[6], tf[3], tf[4], tf[5])), np.array(tf[:3], dtype=float))


class Poses(object):
    """Body poses of one tick (W), converted on demand."""

    def __init__(self, tfs, order):
        self.tfs, self.order, self.cache = tfs, order, {}

    def __contains__(self, bid):
        return bid in self.order

    def __getitem__(self, bid):
        if bid not in self.cache:
            self.cache[bid] = tf_T(self.tfs[self.order[bid]])
        return self.cache[bid]

    def get(self, bid, default=None):
        return self[bid] if bid in self.order else default


class Cell(object):
    def __init__(self, app, row, config=None, robot_usd=None, render=True, log=None):
        import carb
        import omni.kit.app
        import omni.physics.tensors as pt
        import omni.usd
        from isaacsim.core.api import SimulationContext
        from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics

        self.app, self.row, self.render_on = app, row, bool(render)
        self.cfg = dict(wire.DEFAULTS, **(config or {}))
        self.log = log or (lambda *a: None)
        self.Usd, self.UsdGeom, self.UsdPhysics = Usd, UsdGeom, UsdPhysics
        self.settings = carb.settings.get_settings()
        t0 = time.time()
        ctx = omni.usd.get_context()
        ctx.new_stage()
        self.stage = ctx.get_stage()
        self.index = scene.build(self.stage, row, robot_usd, render=self.render_on)
        R = self.index["robot"]
        for name in ARM_SENSORS[1:]:
            PhysxSchema.PhysxContactReportAPI.Apply(self.stage.OverridePrim("%s/%s" % (R, name))).CreateThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(self.stage.GetPrimAtPath(self.index["ee"])).CreateThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(self.stage.GetPrimAtPath(self.index["cart"])).CreateThresholdAttr(0.0)
        for _ in range(3):
            app.update()
        self.rules = rules.Rules(row)
        # the physics engine's poses are read through the tensor views; writing every body's pose back to the stage
        # at every physics step would cost four times the simulation itself. They are written before a render only
        # (`_render_once`).
        self.settings.set("/physics/updateToUsd", False)
        self.settings.set("/physics/updateVelocitiesToUsd", False)
        self.sim = SimulationContext(physics_dt=F.PHYS_DT, rendering_dt=F.PHYS_DT, stage_units_in_meters=1.0)
        self.sim.reset()
        self.view = pt.create_simulation_view("numpy", ctx.get_stage_id())
        self.art = self.view.create_articulation_view(self.index["rail"])
        mt = self.art.shared_metatype
        names, links = list(mt.dof_names), list(mt.link_names)
        self.dof = [names.index(n) for n in kin.JOINT_NAMES]
        self.dof_cart = names.index("cart_joint")
        self.link_ee, self.link_base = links.index("ee_link"), links.index("base_link")
        self.one = np.array([0], dtype=np.int32)
        self.body_ids = list(self.index["bodies"])
        self.order = {bid: i for i, bid in enumerate(self.body_ids)}
        self.bv = self.view.create_rigid_body_view([self.index["bodies"][bid] for bid in self.body_ids])
        self.rachis = list(self.index["rachis"])
        self.rachis_idx = np.array([self.order[bid] for bid in self.rachis], dtype=int)
        # what presses on the fruits: the tool, the arm's forward links and the cart (its crates), each against
        # every rachis body
        self.contact = []
        filters = [self.index["bodies"][bid] for bid in self.rachis]
        for path in ["%s/%s" % (R, n) for n in ARM_SENSORS] + [self.index["cart"]]:
            try:
                self.contact.append(self.view.create_rigid_contact_view(path, filter_patterns=filters, max_contact_data_count=256))
            except Exception as exc:  # noqa: BLE001
                raise RuntimeError("no contact view on %s: %s" % (path, str(exc)[:200]))
        # the robot at home, the jaws open, the cart at the start of the section
        self.q_cmd = np.array(F.CART["home_q"], dtype=float)
        self.y_cmd, self.odometer = 0.0, 0.0
        q = np.zeros((1, self.art.max_dofs), dtype=np.float32)
        q[0, self.dof] = self.q_cmd
        self.art.set_dof_positions(q, self.one)
        self.art.set_dof_velocities(np.zeros_like(q), self.one)
        self.targets = q.copy()
        self.art.set_dof_position_targets(self.targets, self.one)
        self.tool_T = kin.pose(np.eye(3), np.array([0.0, 0.0, F.TOOL["tcp_z"]]))
        self.tool_mass, self.tool_com = F.TOOL["mass"], np.array(F.TOOL["com"])
        seed = int(row["seed"])
        self.rng = np.random.default_rng([seed & 0xFFFFFFFF, seed >> 32, 0x5EED])
        self.cam_rng = np.random.default_rng([seed & 0xFFFFFFFF, seed >> 32, 0xDE97])
        self.tick_n, self.frames_used, self.stops, self.lock_ticks = 0, 0, 0, 0
        self.ended = None
        self.grip_target, self.grip_limit = float(F.TOOL["open_max"]), 0.0
        self.f_ext, self.t_ext = np.zeros(3), np.zeros(3)
        self.wall = {"physics": 0.0, "render": 0.0, "rules": 0.0}
        self.jaws_dirty = True
        self.q_hist = []
        self.holds = {}                  # member id -> the joint that keeps its body on the tool
        self.hold_n = 0
        self.v_prev = None
        self.slip = float(row["mechanics"]["slip"])
        # check that the view follows the order it was asked in
        tfs = np.array(self.bv.get_transforms())
        want = np.array([self._rest_origin(bid) for bid in self.body_ids])
        d = np.linalg.norm(tfs[:, :3] - want, axis=1)
        if float(d.max()) > 0.03:
            worst = np.argsort(-d)[:5]
            raise RuntimeError("body view order does not match the stage (max error %.4f m: %s)" % (
                float(d.max()), ", ".join("%s %.4f" % (self.body_ids[i], d[i]) for i in worst)))
        self.start_error = {self.body_ids[i]: round(float(d[i]), 4) for i in np.argsort(-d)[:5]}
        if self.render_on:
            self._render_setup()
        self._read()
        for _ in range(int(round(self.cfg["start_settle_s"] / F.TICK))):       # the plants come to rest under their own weight
            for _k in range(F.SUBSTEPS):
                self.sim.step(render=False)
        self._read()
        self.v_prev = np.array(self.bv.get_velocities())[self.rachis_idx, :3]
        self.log("cell ready in %.1f s: %s, %d bodies, %d trusses (%d ready)" % (
            time.time() - t0, row["cultivar"], len(self.body_ids), len(self.rules.truss), len(row["ready"])))

    def _rest_origin(self, bid):
        m = self.rules.member[bid]
        for p in self.row["plants"]:
            if p["id"] != m["plant"]:
                continue
            if m["kind"] == "stem":
                return p["points"][m["index"]]
            if m["kind"] == "petiole":
                return [lf for lf in p["leaves"] if lf["id"] == bid][0]["petiole"]["a"]
            t = [t for t in p["trusses"] if t["id"] == m["truss"]][0]
            return t["peduncle"]["a"] if m["kind"] == "peduncle" else t["rachis"][m["index"]]["a"]
        raise KeyError(bid)

    # ------------------------------------------------------------------ rendering
    def _prim(self, path):
        return self.stage.GetPrimAtPath(path)

    def _render_setup(self):
        import omni.kit.app
        t0 = time.time()
        self.settings.set("/exts/omni.replicator.core/maxAssetLoadingTime", 1.0)
        omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("omni.replicator.core", True)
        for _ in range(2):
            self.app.update()
        self.settings.set("/exts/omni.replicator.core/maxAssetLoadingTime", 1.0)
        import omni.replicator.core as rep
        self.rep = rep
        self.cams = {}
        pinned = {}
        for name, c in F.CAMERAS.items():
            rp = rep.create.render_product(self.index["cam_" + name], (c["width"], c["height"]), name="rowcell_" + name)
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb.attach([rp])
            dep = None
            if c["depth"]:
                dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
                dep.attach([rp])
            prim = self._prim(rp.path if hasattr(rp, "path") else str(rp))
            done = {}
            for key, val in self.cfg["render_settings"].items():
                a = prim.GetAttribute(key) if prim and prim.IsValid() else None
                if a is not None and a.IsValid():
                    a.Set(val)
                    done[key] = val
            pinned[name] = done
            self.cams[name] = (rgb, dep)
        self.render_pinned = pinned
        for _ in range(3):
            self._render_once()
        self.log("renderer ready in %.1f s; pinned %s" % (time.time() - t0, {k: len(v) for k, v in pinned.items()}))

    def _render_once(self):
        import omni.physx
        omni.physx.get_physx_interface().update_transformations(False, True, False, False)   # the bodies' poses onto the stage
        self.settings.set_bool("/app/player/playSimulations", False)      # a render must not advance the physics
        try:
            self.rep.orchestrator.step(rt_subframes=int(self.cfg["render_subframes"]), delta_time=0.0, pause_timeline=False)
        finally:
            self.settings.set_bool("/app/player/playSimulations", True)

    def _sync_looks(self):
        if self.jaws_dirty:
            from pxr import Gf
            w = self.rules.opening
            for side, sgn in (("l", -1), ("r", 1)):
                op = self.UsdGeom.Xformable(self._prim(self.index["jaw_" + side])).GetOrderedXformOps()[0]
                op.Set(Gf.Vec3d(0.0, sgn * w / 2, 0.0))
            self.jaws_dirty = False

    def frames(self, names):
        """Render and encode the cameras in `names`: {name: {"rgb": png base64[, "depth": 16-bit png base64]}}."""
        import cv2
        t0 = time.time()
        self._sync_looks()
        self._render_once()
        out = {}
        light = self.row.get("light") or {}
        gains = [light.get("gain", 1.0) * c for c in light.get("rgb", [1.0, 1.0, 1.0])]
        for name in names:
            rgb, dep = self.cams[name]
            c = F.CAMERAS[name]
            img = np.asarray(rgb.get_data())
            if img.ndim != 3 or img.shape[0] != c["height"] or img.shape[1] != c["width"]:
                raise RuntimeError("camera %s returned no frame (rgb %s)" % (name, img.shape))
            ok1, png = cv2.imencode(".png", cv2.cvtColor(img[..., :3], cv2.COLOR_RGB2BGR), [cv2.IMWRITE_PNG_COMPRESSION, 1])
            if not ok1:
                raise RuntimeError("camera %s: frame could not be encoded" % name)
            out[name] = {"rgb": base64.b64encode(png.tobytes()).decode("ascii"), "t": round(self.rules.t, 3)}
            if dep is not None:
                d = np.asarray(dep.get_data(), dtype=np.float32)
                if d.shape[:2] != img.shape[:2]:
                    raise RuntimeError("camera %s returned no depth (%s)" % (name, d.shape))
                d = np.where(np.isfinite(d) & (d < 11.5), d, 0.0)
                # what the sensor makes of the true depth: noise, quantization, lost readings (sensors.py, farm.SENSORS)
                d = sensors.measure(d, img[..., :3], F.camera_K(name), F.SENSORS[name], self.cam_rng, light_gain=gains)
                d16 = np.clip(np.round(d * 1e4), 0, 65535).astype(np.uint16)          # 0.1 mm per count, 0 = no reading
                ok2, dpng = cv2.imencode(".png", d16, [cv2.IMWRITE_PNG_COMPRESSION, 1])
                if not ok2:
                    raise RuntimeError("camera %s: depth could not be encoded" % name)
                out[name].update(depth=base64.b64encode(dpng.tobytes()).decode("ascii"), depth_unit_m=1e-4)
            if name == "wrist":
                # the nominal mounting (the robot's own offset is not on the wire): wire.perturbed builds it so too
                out[name]["pose"] = wire.pose6(self.flange_T @ F.camera_mount("wrist"))
            self.frames_used += 1
        self.wall["render"] += time.time() - t0
        return out

    # ------------------------------------------------------------------ state
    def _read(self):
        """Read the robot's state and the external wrench at the flange after a tick."""
        q = np.array(self.art.get_dof_positions()[0], dtype=float)
        q_new = q[self.dof]
        # joint speeds by differences over the tick, as a controller computes them from its encoders
        self.qd = (q_new - self.q) / F.TICK if getattr(self, "q", None) is not None else np.zeros(6)
        self.q = q_new
        self.y_cart = float(q[self.dof_cart])
        links = np.array(self.art.get_link_transforms()[0])
        self.ee_T = tf_T(links[self.link_ee])              # W
        self.flange_T = kin.flange(self.q)                 # C
        inc = np.array(self.art.get_link_incoming_joint_force()[0][self.link_ee], dtype=float)
        R = self.ee_T[:3, :3]
        gvec = np.array([0.0, 0.0, -G])
        # incoming joint force: what the flange exerts on the tool carrier, in the carrier's frame; what is left
        # after the tool's own weight is what the world exerts on the tool (a held truss's weight included)
        self.f_ext = -(R @ inc[:3]) - self.tool_mass * gvec
        self.t_ext = -(R @ inc[3:]) - np.cross(R @ self.tool_com, self.tool_mass * gvec)

    def poses(self):
        return Poses(np.array(self.bv.get_transforms()), self.order)

    # ------------------------------------------------------------------ one control tick
    def _tick(self, q_next, y_next, trace):
        r, cfg = self.rules, self.cfg
        t0 = time.time()
        q_prev, y_prev = self.q_cmd, self.y_cmd
        for k in range(F.SUBSTEPS):
            s = (k + 1) / float(F.SUBSTEPS)
            self.targets[0, self.dof] = q_prev + (q_next - q_prev) * s
            self.targets[0, self.dof_cart] = y_prev + (y_next - y_prev) * s
            self.art.set_dof_position_targets(self.targets, self.one)
            self.sim.step(render=False)
        self.q_cmd, self.y_cmd = np.array(q_next, dtype=float), float(y_next)
        self.tick_n += 1
        r.t = self.tick_n * F.TICK
        self._read()
        self.q_hist = (self.q_hist + [self.q.copy()])[-6:]
        t1 = time.time()
        self.wall["physics"] += t1 - t0
        poses = self.poses()
        # the gripper: what the blade cuts and the pad holds
        w0 = r.opening
        for what, mid in r.gripper(self.ee_T @ self.tool_T, poses, self.grip_target, self.grip_limit, F.TICK):
            if what == "cut":
                self._cut(mid)
            elif what == "hold":
                self._hold(mid, poses[mid])
            elif what == "release":
                self._release(mid)
        if r.opening != w0:
            self.jaws_dirty = True
        # what bruises a fruit: a change of speed within the tick, a press by the robot or against a crate
        v = np.array(self.bv.get_velocities())[self.rachis_idx, :3]
        if self.v_prev is not None:
            dv = np.linalg.norm(v - self.v_prev, axis=1)
            hit = np.nonzero(dv > 0.25 * F.DAMAGE["impact_dv"])[0]
            if len(hit):
                r.impacts({self.rachis[i]: float(dv[i]) for i in hit})
        self.v_prev = v
        press = np.zeros(len(self.rachis))
        for cv in self.contact:
            fm = np.array(cv.get_contact_force_matrix(F.PHYS_DT))
            press = np.maximum(press, np.linalg.norm(fm.reshape(-1, len(self.rachis), 3), axis=2).max(axis=0))
        hit = np.nonzero(press > 0.25 * F.DAMAGE["squeeze_force"])[0]
        if len(hit):
            r.presses({self.rachis[i]: float(press[i]) for i in hit})
        # overload: the robot stops where it is, as a collaborative robot does
        err = float(np.max(np.abs(self.q_cmd - self.q)))
        if (np.linalg.norm(self.f_ext) > cfg["overload_force"] or np.linalg.norm(self.t_ext) > cfg["overload_torque"]
                or err > cfg["overload_joint_error"]) and self.lock_ticks == 0:
            # the arm backs off to where it was a quarter of a second before and stays there for the lock time
            self.stops += 1
            self.lock_ticks = int(round(cfg["stop_lock_s"] / F.TICK))
            self.q_cmd = self.q_hist[0].copy() if self.q_hist else self.q.copy()
            r.log("protective_stop", force=[round(float(x), 1) for x in self.f_ext], torque=[round(float(x), 1) for x in self.t_ext],
                  joint_error=round(err, 4), tcp=[round(float(x), 3) for x in (self.flange_T @ self.tool_T)[:3, 3]])
            if self.stops >= F.CONSEQUENCE["latch"] and not r.ended:
                r.ended = "stops_latched"                  # the safety controller latches: a person has to reset it
                r.log("stops_latched", stops=self.stops)
        elif self.lock_ticks > 0:
            self.lock_ticks -= 1
        if trace is not None:
            trace["force"].append(round(float(np.linalg.norm(self.f_ext)), 2))
            trace["opening_mm"].append(round(self._opening_reading() * 1000.0, 3))
            trace["grip_force"].append(round(self._force_reading(), 2))
        if r.ended and not self.ended:
            self.ended = r.ended
        self.wall["rules"] += time.time() - t1

    def _opening_reading(self):
        return max(0.0, self.rules.opening + float(self.rng.normal(0.0, F.TOOL["opening_noise"])))

    def _force_reading(self):
        f = self.rules.force
        return max(0.0, f + float(self.rng.normal(0.0, F.TOOL["force_noise"]))) if f > 0 else 0.0

    # ------------------------------------------------------------------ what the rules hand over
    def _wake(self, bid):
        i = np.array([self.order[bid]], dtype=np.int32)
        self.bv.set_velocities(np.array(self.bv.get_velocities(), dtype=np.float32), i)

    def _cut(self, mid):
        """A member is cut through: the joint to what it grew on goes."""
        path = self.index["joints"].get(mid)
        if path and self._prim(path).IsValid():
            self.stage.RemovePrim(path)
            self._wake(mid)

    def _hold(self, mid, T_body):
        """The pad holds a member: its body rides on the tool, where it is."""
        from pxr import Gf
        rel = kin.inv(self.ee_T) @ T_body
        q = kin.R_quat(rel[:3, :3])
        self.hold_n += 1
        path = "/World/Hold%d" % self.hold_n
        j = self.UsdPhysics.FixedJoint.Define(self.stage, path)
        j.CreateBody0Rel().SetTargets([self.index["ee"]])
        j.CreateBody1Rel().SetTargets([self.index["bodies"][mid]])
        j.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in rel[:3, 3]]))
        j.CreateLocalRot0Attr(Gf.Quatf(float(q[0]), float(q[1]), float(q[2]), float(q[3])))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr(True)
        self.holds[mid] = path

    def _release(self, mid):
        path = self.holds.pop(mid, None)
        if path:
            self.stage.RemovePrim(path)
            self._wake(mid)        # a body that hung still may have gone to sleep and would not notice its joint is gone

    # ------------------------------------------------------------------ one action of the wire
    def step(self, action):
        """Execute a validated action (`wire.check_action`) and return the observation."""
        cfg, r = self.cfg, self.rules
        events = []
        if self.ended:
            return self.observe(events=["the row section has ended: " + self.ended])
        a = action
        move = a["move"]
        n = int(move["ticks"])
        if a["gripper"] is not None:
            self.grip_target = min(F.TOOL["open_max"], float(a["gripper"]["opening"]))
            self.grip_limit = min(F.TOOL["force_max"], float(a["gripper"]["force"]))
        trace = {"force": [], "opening_mm": [], "grip_force": []}
        flags = {"clipped": False, "guard": False, "stopped": False, "cart_limit": False}
        moving = move["type"] != "hold" and self.lock_ticks == 0
        if move["type"] != "hold" and not moving:
            flags["stopped"] = True
        q0 = self.q_cmd.copy()
        delta = np.array(move["delta"], dtype=float)
        peak = 1.5 / (n * F.TICK)
        if moving and move["type"] == "tcp":
            T0 = kin.flange(q0) @ self.tool_T
            k = max(1.0, np.linalg.norm(delta[:3]) * peak / cfg["tcp_speed_max"], np.linalg.norm(delta[3:]) * peak / cfg["tcp_rot_speed_max"])
            if k > 1.0:
                delta, flags["clipped"] = delta / k, True
        elif moving and move["type"] == "joint":
            k = max(1.0, float(np.max(np.abs(delta) * peak / np.array(cfg["joint_speed_max"]))))
            if k > 1.0:
                delta, flags["clipped"] = delta / k, True
        # the cart: `cart` metres by its odometer; its wheels slip, so it travels less than they turn
        d_odo = float(a["cart"])
        if abs(d_odo) * peak > F.CART["speed_max"]:
            d_odo, flags["clipped"] = d_odo * F.CART["speed_max"] / (abs(d_odo) * peak), True
        d_true = d_odo * (1.0 - self.slip) * (1.0 + float(self.rng.normal(0.0, F.ROW["slip_sigma"])) if d_odo else 0.0)
        y_goal = min(F.CART["travel"][1], max(F.CART["travel"][0], self.y_cmd + d_true))
        if abs(y_goal - (self.y_cmd + d_true)) > 1e-9:
            flags["cart_limit"] = True
            d_odo = d_odo * ((y_goal - self.y_cmd) / d_true) if abs(d_true) > 1e-12 else 0.0
            d_true = y_goal - self.y_cmd
        y0, odo0 = self.y_cmd, self.odometer
        f_start = self.f_ext.copy()
        stops0 = self.stops
        dq_max = np.array(cfg["joint_speed_max"]) * F.TICK
        for j in range(1, n + 1):
            s = j / float(n)
            s = s * s * (3 - 2 * s)                      # smooth start and stop
            q_next = self.q_cmd
            if moving and self.lock_ticks == 0:
                if move["type"] == "tcp":
                    Tj = kin.pose(kin.rotvec_R(s * delta[3:]) @ T0[:3, :3], T0[:3, 3] + s * delta[:3])
                    q_ik, _ = kin.ik_step(self.q_cmd, Tj, self.tool_T)
                else:
                    q_ik = np.clip(q0 + s * delta, kin.Q_LIMITS[:, 0], kin.Q_LIMITS[:, 1])
                dq = q_ik - self.q_cmd
                k = float(np.max(np.abs(dq) / dq_max))
                if k > 1.0:
                    dq, flags["clipped"] = dq / k, True
                q_next = self.q_cmd + dq
            self._tick(q_next, y0 + s * d_true, trace)
            self.odometer = odo0 + s * d_odo
            if self.stops != stops0:
                flags["stopped"], moving = True, False
            if moving and move["guard_force"] is not None and np.linalg.norm(self.f_ext - f_start) > move["guard_force"]:
                flags["guard"], moving = True, False
            if self.ended or r.t >= cfg["sim_cap_s"]:
                break
        if a["cmd"] == "done" and not self.ended:
            self.ended = "done"
        if not self.ended and r.t >= cfg["sim_cap_s"]:
            self.ended = "sim_cap"
        if self.ended:
            self._settle()
        cams = [c for c in a["cams"] if self.render_on]
        if cams and self.frames_used + len(cams) > cfg["frame_budget"]:
            events.append("cams: refused, the frame budget is used up")
            cams = []
        return self.observe(events, flags, trace, cams)

    def _settle(self):
        """After the end: the arm holds, the bodies come to rest (the rules still watch the fruits)."""
        if getattr(self, "_settled", False):
            return
        self._settled = True
        for _ in range(int(round(self.cfg["settle_s"] / F.TICK))):
            self._tick(self.q_cmd, self.y_cmd, None)

    def observe(self, events=None, flags=None, trace=None, cams=()):
        r, cfg = self.rules, self.cfg
        noise = self.rng.normal(0.0, 1.0, 6) * np.array([cfg["noise_force"]] * 3 + [cfg["noise_torque"]] * 3)
        obs = {
            "time": round(r.t, 3),
            "q": [round(float(v), 6) for v in self.q],
            "qd": [round(float(v), 5) for v in self.qd],
            "q_target": [round(float(v), 6) for v in self.q_cmd],
            "tcp": wire.pose6(self.flange_T @ self.tool_T),
            "tcp_target": wire.pose6(kin.flange(self.q_cmd) @ self.tool_T),
            "wrench": [round(float(v), 3) for v in np.concatenate([self.f_ext, self.t_ext]) + noise],
            "cart": {"odometer": round(self.odometer, 5)},
            "gripper": {"opening": round(self._opening_reading(), 5), "force": round(self._force_reading(), 2)},
            "status": dict(flags or {"clipped": False, "guard": False, "stopped": False, "cart_limit": False},
                           locked_s=round(self.lock_ticks * F.TICK, 2), protective_stops=self.stops),
            "trace": trace or {"force": [], "opening_mm": [], "grip_force": []},
            "events": list(events or []),
            "frames_left": int(cfg["frame_budget"] - self.frames_used),
            "time_left": round(max(0.0, cfg["sim_cap_s"] - r.t), 3),
            "ended": self.ended,
            "images": {},
        }
        if cams:
            obs["images"] = self.frames(cams)
            obs["frames_left"] = int(cfg["frame_budget"] - self.frames_used)
        return obs

    # ------------------------------------------------------------------ grader only
    def joints(self):
        """The measured and the commanded joint angles as the cell holds them: an observation gives them rounded to
        1e-6 and computes its poses from these. The grader builds a probe's copy from them (`wire.perturbed`)."""
        return {"q": [float(v) for v in self.q], "q_target": [float(v) for v in self.q_cmd]}

    def truth(self):
        t = self.rules.truth(self.poses(), self.y_cart)
        Tt = self.ee_T @ self.tool_T
        Tt[1, 3] -= self.y_cart
        t["tool"] = wire.pose6(Tt)                                  # the tool centre point as the physics has it (C)
        t["tcp_target"] = wire.pose6(kin.flange(self.q_cmd) @ self.tool_T)   # the commanded target exactly
        t["q_target"] = [round(float(v), 6) for v in self.q_cmd]
        t["odometer"] = round(self.odometer, 5)
        t["slip"] = self.slip
        t["time"] = round(self.rules.t, 3)
        t["grip_force"] = round(self.rules.force, 2)
        t["stalled"] = bool(self.rules.stalled)
        return t

    def score(self):
        """The row section's yield as things lie now. After the end (or when the grader has given the controller up:
        it says so with `final`) the bodies are first left to come to rest, the arm holding its pose."""
        if self.ended:
            self._settle()
        credit, detail = self.rules.credit(self.poses(), self.y_cart)
        r = self.rules
        return {"credit": credit, "detail": detail, "ended": self.ended, "events": r.events, "time": round(r.t, 2),
                "ticks": self.tick_n, "frames": self.frames_used, "protective_stops": self.stops,
                "ready": sum(1 for t in r.truss.values() if t["ready"]),
                "delivered": sum(1 for d in detail.values() if d["counts"]),
                "cut_unready": sum(1 for t in r.truss.values() if not t["ready"] and t["status"] != "on_plant"),
                "plants_lost": sum(1 for p in r.plant.values() if p["lost"]),
                "leaves_cut": sum(1 for lf in r.leaf.values() if lf["cut"]),
                "margins": dict(r.margins, dv_max=round(max([t["dv_max"] for t in r.truss.values()] or [0.0]), 3),
                                press_max=round(max([t["press_max"] for t in r.truss.values()] or [0.0]), 2),
                                pad_ratio_max=round(max([t["pad_ratio"] for t in r.truss.values()] or [0.0]), 3)),
                "cart_y": round(self.y_cart, 4), "odometer": round(self.odometer, 4),
                "wall": {k: round(v, 1) for k, v in self.wall.items()}}
