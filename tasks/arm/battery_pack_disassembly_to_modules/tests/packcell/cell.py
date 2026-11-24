"""The disassembly cell in Isaac Sim: one pack on the fixture, the robot with its tools, the two cameras.

`Cell(app, pack, config)` builds the stage (`scene.py`), runs PhysX on the CPU and renders on the GPU only when an
action asks for a camera frame. `step(action)` executes one action of the wire (a motion of 1 to 40 control ticks
with the spindle and the gripper commands that go with it, see `wire.py`) and returns the observation: the robot's
own readings, the external wrench at the flange, the nutrunner's torque, angle and stroke, the gripper's opening,
and the frames that were asked for. Part poses and fastener states never leave this module except through
`truth()` (the reference's privileged view, grader only) and `score()`.

Physics state decides everything that is scored: where each body lies at the end (`rules.Rules.credit`).
"""

import base64
import math
import time

import numpy as np

from . import kin, rules, scene, wire
from . import series as S

G = 9.81
PARK = np.array([30.0, 30.0, -5.0])      # where a fastener's body waits while its copy rides on the bit


def tf_T(tf):
    """PhysX transform row [x y z qx qy qz qw] -> 4x4."""
    return kin.pose(kin.quat_R((tf[6], tf[3], tf[4], tf[5])), np.array(tf[:3], dtype=float))


def T_tf(T):
    q = kin.R_quat(T[:3, :3])
    return np.array([T[0, 3], T[1, 3], T[2, 3], q[1], q[2], q[3], q[0]], dtype=np.float32)


class Poses(object):
    """Part poses of one tick, converted on demand."""

    def __init__(self, tfs, order):
        self.tfs, self.order, self.cache = tfs, order, {}

    def __contains__(self, pid):
        return pid in self.order

    def __getitem__(self, pid):
        if pid not in self.cache:
            self.cache[pid] = tf_T(self.tfs[self.order[pid]])
        return self.cache[pid]

    def get(self, pid, default=None):
        return self[pid] if pid in self.order else default


class Cell(object):
    def __init__(self, app, pack, config=None, robot_usd=None, render=True, log=None):
        import carb
        import omni.kit.app
        import omni.physics.tensors as pt
        import omni.usd
        from isaacsim.core.api import SimulationContext
        from pxr import Usd, UsdGeom, UsdPhysics

        self.app, self.pack, self.render_on = app, pack, bool(render)
        self.cfg = dict(wire.DEFAULTS, **(config or {}))
        self.log = log or (lambda *a: None)
        self.Usd, self.UsdGeom, self.UsdPhysics = Usd, UsdGeom, UsdPhysics
        self.settings = carb.settings.get_settings()
        t0 = time.time()
        ctx = omni.usd.get_context()
        ctx.new_stage()
        self.stage = ctx.get_stage()
        self.index = scene.build(self.stage, pack, robot_usd, render=self.render_on)
        for _ in range(3):
            app.update()
        self.rules = rules.Rules(pack)
        self.sim = SimulationContext(physics_dt=S.PHYS_DT, rendering_dt=S.PHYS_DT, stage_units_in_meters=1.0)
        self.sim.reset()
        self.view = pt.create_simulation_view("numpy", ctx.get_stage_id())
        self.art = self.view.create_articulation_view(self.index["base"])
        mt = self.art.shared_metatype
        names, links = list(mt.dof_names), list(mt.link_names)
        self.dof = [names.index(n) for n in kin.JOINT_NAMES]
        self.dof_spindle = names.index("spindle_joint")
        self.link_ee, self.link_w3 = links.index("ee_link"), links.index("wrist_3_link")
        self.link_sp = links.index("spindle_link")
        self.one = np.array([0], dtype=np.int32)
        self.part_ids = [p["id"] for p in pack["parts"]]
        self.part_order = {pid: i for i, pid in enumerate(self.part_ids)}
        self.pv = self.view.create_rigid_body_view([self.index["parts"][pid] for pid in self.part_ids])
        self.fast_ids = [f["id"] for f in pack["fasteners"] if not f["missing"]]
        self.fast_order = {fid: i for i, fid in enumerate(self.fast_ids)}
        self.fv = self.view.create_rigid_body_view([self.index["fasteners"][fid] for fid in self.fast_ids]) if self.fast_ids else None
        self.seat_pos = np.array([self.rules.part[pid]["seat"][:3, 3] for pid in self.part_ids])
        self.module_idx = [(pid, i) for i, pid in enumerate(self.part_ids) if self.rules.part[pid]["kind"] == "module"]
        self.harness_idx = self.part_order.get("harness")
        # the robot at home, the nutrunner mounted with the S10 socket
        self.q_cmd = np.array(S.CELL["home_q"], dtype=float)
        q = np.zeros((1, self.art.max_dofs), dtype=np.float32)
        q[0, self.dof] = self.q_cmd
        self.art.set_dof_positions(q, self.one)
        self.art.set_dof_velocities(np.zeros_like(q), self.one)
        self.targets = q.copy()
        self.art.set_dof_position_targets(self.targets, self.one)
        self.masses = np.array(self.art.get_masses(), dtype=np.float32)
        self.coms = np.array(self.art.get_coms(), dtype=np.float32)
        self.tool = "nutrunner"
        self._show_tool()
        self.rng = np.random.default_rng([int(pack["seed"]) & 0xFFFFFFFF, int(pack["seed"]) >> 32, 0x5EED])
        self.tick_n, self.frames_used, self.stops, self.lock_ticks, self.busy_ticks = 0, 0, 0, 0, 0
        self.ended, self.grasp_joint, self.grip_target = None, None, S.TOOLS["gripper"]["open_max"]
        self.spindle_torque, self.spindle_speed, self.stroke = 0.0, 0.0, 0.0
        self.f_ext, self.t_ext = np.zeros(3), np.zeros(3)
        self.wall = {"physics": 0.0, "render": 0.0, "rules": 0.0}
        self.fingers_dirty = True
        self.q_hist = []
        # check that the views follow the order they were asked in
        tfs = np.array(self.pv.get_transforms())
        d = float(np.max(np.linalg.norm(tfs[:, :3] - self.seat_pos, axis=1)))
        if d > 1e-3:
            raise RuntimeError("part view order does not match the stage (max seat error %.4f m)" % d)
        if self.render_on:
            self._render_setup()
        for _ in range(S.SUBSTEPS * 10):
            self.sim.step(render=False)
        self._read()
        self.log("cell ready in %.1f s: pack %s, %d parts, %d fasteners" % (time.time() - t0, pack["variant"], len(self.part_ids), len(self.fast_ids)))

    # ------------------------------------------------------------------ looks that follow the tool state
    def _prim(self, path):
        return self.stage.GetPrimAtPath(path)

    def _visible(self, path, on):
        im = self.UsdGeom.Imageable(self._prim(path))
        (im.MakeVisible if on else im.MakeInvisible)()

    def _collide(self, path, on):
        self.UsdPhysics.CollisionAPI(self._prim(path)).CreateCollisionEnabledAttr(bool(on))

    def _show_tool(self):
        """Looks, colliders and mass of the tool carrier for the mounted tool (None: the bare changer plate)."""
        ix, r = self.index, self.rules
        for name in ("nutrunner", "gripper"):
            on = self.tool == name
            for path in ix["tool_roots"][name]:
                self._visible(path, on)
            for path in ix["ee_colliders"][name]:
                self._collide(path, on)
            self._visible(ix["parked"][name], not on)
        for bit in S.BITS:
            mounted = self.tool == "nutrunner" and r.bit == bit
            self._visible(ix["bits"][bit], mounted)
            self._visible(ix["parked"]["bit_" + bit], not mounted)
        held = r.fast[r.captured]["head"] if r.captured else None
        for head, path in ix["held"].items():
            self._visible(path, self.tool == "nutrunner" and head == held)
        t = S.TOOLS.get(self.tool)
        mass, com = (t["mass"], t["com"]) if t else (0.6, [0.0, 0.0, 0.015])
        self.masses[0, self.link_ee] = mass
        self.coms[0, self.link_ee, :3] = com
        self.art.set_masses(self.masses, self.one)
        self.art.set_coms(self.coms, self.one)
        self.tool_mass = mass + S.TOOLS["nutrunner"]["spindle_mass"]
        sp_c = np.array([0.0, 0.0, S.TOOLS["nutrunner"]["spindle_origin_z"] + 0.06])
        self.tool_com = (mass * np.array(com) + S.TOOLS["nutrunner"]["spindle_mass"] * sp_c) / self.tool_mass
        self.fingers_dirty = True

    def tcp_offset(self):
        z = S.TOOLS[self.tool]["tcp_z"] if self.tool else 0.0
        return kin.pose(np.eye(3), np.array([0.0, 0.0, z]))

    # ------------------------------------------------------------------ rendering
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
        for name, c in S.CAMERAS.items():
            rp = rep.create.render_product(self.index["cam_" + name], (c["width"], c["height"]), name="packcell_" + name)
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
            rgb.attach([rp])
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
        self.settings.set_bool("/app/player/playSimulations", False)      # a render must not advance the physics
        try:
            self.rep.orchestrator.step(rt_subframes=int(self.cfg["render_subframes"]), delta_time=0.0, pause_timeline=False)
        finally:
            self.settings.set_bool("/app/player/playSimulations", True)

    def _sync_looks(self):
        if self.fingers_dirty:
            w = self.rules.width if self.tool == "gripper" else S.TOOLS["gripper"]["open_max"]
            from pxr import Gf
            for side, sgn in (("l", -1), ("r", 1)):
                op = self.UsdGeom.Xformable(self._prim(self.index["finger_" + side])).GetOrderedXformOps()[0]
                op.Set(Gf.Vec3d(0.0, sgn * w / 2, 0.0))
            self.fingers_dirty = False

    def frames(self, names):
        """Render and encode the cameras in `names`: {name: {"rgb": png base64, "depth": 16-bit png base64}}."""
        import cv2
        t0 = time.time()
        self._sync_looks()
        self._render_once()
        out = {}
        for name in names:
            rgb, dep = self.cams[name]
            c = S.CAMERAS[name]
            img = np.asarray(rgb.get_data())
            d = np.asarray(dep.get_data(), dtype=np.float32)
            if img.ndim != 3 or img.shape[0] != c["height"] or img.shape[1] != c["width"] or d.shape[:2] != img.shape[:2]:
                raise RuntimeError("camera %s returned no frame (rgb %s, depth %s)" % (name, img.shape, d.shape))
            d = np.where(np.isfinite(d) & (d < 6.5), d, 0.0)
            d16 = np.clip(np.round(d * 1e4), 0, 65535).astype(np.uint16)          # 0.1 mm per count, 0 = no reading
            ok1, png = cv2.imencode(".png", cv2.cvtColor(img[..., :3], cv2.COLOR_RGB2BGR), [cv2.IMWRITE_PNG_COMPRESSION, 1])
            ok2, dpng = cv2.imencode(".png", d16, [cv2.IMWRITE_PNG_COMPRESSION, 1])
            if not (ok1 and ok2):
                raise RuntimeError("camera %s: frame could not be encoded" % name)
            out[name] = {"rgb": base64.b64encode(png.tobytes()).decode("ascii"),
                         "depth": base64.b64encode(dpng.tobytes()).decode("ascii"),
                         "depth_unit_m": 1e-4, "t": round(self.rules.t, 3)}
            if name == "wrist":
                T = self.flange_T @ kin.pose(S.camera_R("wrist"), np.array(S.CAMERAS["wrist"]["pos"]))
                out[name]["pose"] = wire.pose6(T)
            self.frames_used += 1
        self.wall["render"] += time.time() - t0
        return out

    # ------------------------------------------------------------------ state
    def _read(self):
        """Read the robot's state and the external wrench at the flange after a tick."""
        q = np.array(self.art.get_dof_positions()[0], dtype=float)
        q_new = q[self.dof]
        # joint speeds by differences over the tick, as a controller computes them from its encoders
        self.qd = (q_new - self.q) / S.TICK if getattr(self, "q", None) is not None else np.zeros(6)
        self.q = q_new
        self.stroke = max(0.0, -float(q[self.dof_spindle])) if self.tool == "nutrunner" else 0.0
        links = np.array(self.art.get_link_transforms()[0])
        self.ee_T = tf_T(links[self.link_ee])
        self.sp_T = tf_T(links[self.link_sp])
        self.flange_T = kin.flange(self.q)
        inc = np.array(self.art.get_link_incoming_joint_force()[0][self.link_ee], dtype=float)
        R = self.ee_T[:3, :3]
        gvec = np.array([0.0, 0.0, -G])
        # incoming joint force: what the flange exerts on the tool carrier, in the carrier's frame; what is left
        # after the tool's own weight is what the world exerts on the tool
        self.f_ext = -(R @ inc[:3]) - self.tool_mass * gvec
        self.t_ext = -(R @ inc[3:]) - np.cross(R @ self.tool_com, self.tool_mass * gvec)

    def tcp_T(self, q=None):
        return (self.flange_T if q is None else kin.flange(q)) @ self.tcp_offset()

    def tip(self):
        """The bit's tip as the physics has it (spindle compressed)."""
        n = S.TOOLS["nutrunner"]
        return self.sp_T[:3, 3] + self.sp_T[:3, :3] @ np.array([0.0, 0.0, n["spindle_len"] + n["bit_len"]]), self.sp_T[:3, 2]

    def poses(self):
        return Poses(np.array(self.pv.get_transforms()), self.part_order)

    def fastener_poses(self):
        if self.fv is None:
            return {}
        tfs = np.array(self.fv.get_transforms())
        return {fid: tf_T(tfs[i]) for fid, i in self.fast_order.items()}

    # ------------------------------------------------------------------ one control tick
    def _tick(self, q_next, spindle, trace):
        r, cfg = self.rules, self.cfg
        t0 = time.time()
        q_prev = self.q_cmd
        for k in range(S.SUBSTEPS):
            self.targets[0, self.dof] = q_prev + (q_next - q_prev) * (k + 1) / S.SUBSTEPS
            self.art.set_dof_position_targets(self.targets, self.one)
            self.sim.step(render=False)
        self.q_cmd = np.array(q_next, dtype=float)
        self.tick_n += 1
        r.t = self.tick_n * S.TICK
        self._read()
        self.q_hist = (self.q_hist + [self.q.copy()])[-6:]
        t1 = time.time()
        self.wall["physics"] += t1 - t0
        tfs = np.array(self.pv.get_transforms())
        poses = Poses(tfs, self.part_order)
        # what holds the parts (before the spindle's rule: a fastener that comes out in this tick frees its part in
        # the next one, when its body has left the seat)
        moved = np.nonzero(np.linalg.norm(tfs[:, :3] - self.seat_pos, axis=1) > 0.02)[0]
        for pid in r.unlock([self.part_ids[i] for i in moved], self.f_ext):
            self._free(pid)
        # the nutrunner
        self.spindle_torque, self.spindle_speed = 0.0, 0.0
        if self.tool == "nutrunner":
            tip, axis = self.tip()
            force = S.TOOLS["nutrunner"]["spring_N_per_m"] * self.stroke
            if spindle is not None:
                h = None
                if self.harness_idx is not None and not r.part["harness"]["damaged"]:
                    hp, Th = r.part["harness"], poses["harness"]
                    h = [(Th[:3, :3] @ (np.array(p) - np.array(hp["pos"]))) + Th[:3, 3] for p in hp["path"]]
                had = r.captured
                tq, sp = r.spindle(tip, axis, force, spindle[0], spindle[1], S.TICK, h)
                self.spindle_torque = tq + (float(self.rng.normal(0.0, 0.03)) if tq > 0 else 0.0)
                self.spindle_speed = sp
                if r.captured is not None and r.captured != had:
                    self._capture(r.captured)
            r.hv_check([tip])
        elif self.tool == "gripper":
            tcp = self.ee_T @ self.tcp_offset()
            w0 = r.width
            ev = r.gripper(tcp, self.grip_target, S.TICK, poses)
            if r.width != w0:
                self.fingers_dirty = True
            if ev is not None and ev[0] == "attach":
                self._attach(ev[1], poses[ev[1]])
            elif ev is not None and ev[0] == "detach":
                self._detach()
            r.hv_check(r.grip_points(tcp))
        if self.module_idx:
            vel = np.array(self.pv.get_velocities())
            r.drop_check({pid: float(vel[i, 2]) for pid, i in self.module_idx})
        # overload: the robot stops where it is, as a collaborative robot does
        err = float(np.max(np.abs(self.q_cmd - self.q)))
        if (np.linalg.norm(self.f_ext) > cfg["overload_force"] or np.linalg.norm(self.t_ext) > cfg["overload_torque"]
                or err > cfg["overload_joint_error"]) and self.lock_ticks == 0:
            # the arm backs off to where it was a quarter of a second before and stays there for the lock time
            self.stops += 1
            self.lock_ticks = int(round(cfg["stop_lock_s"] / S.TICK))
            self.q_cmd = self.q_hist[0].copy() if self.q_hist else self.q.copy()
            r.log("protective_stop", force=[round(float(v), 1) for v in self.f_ext], torque=[round(float(v), 1) for v in self.t_ext],
                  joint_error=round(err, 4), tcp=[round(float(v), 3) for v in (self.flange_T @ self.tcp_offset())[:3, 3]],
                  holding=r.grasp)
        elif self.lock_ticks > 0:
            self.lock_ticks -= 1
        if self.busy_ticks > 0:
            self.busy_ticks -= 1
        if trace is not None:
            z = self.flange_T[:3, 2]
            trace["force_axial"].append(round(float(np.dot(self.f_ext, z)), 2))
            trace["torque"].append(round(self.spindle_torque, 3))
            trace["angle_deg"].append(round(math.degrees(r.angle), 2))
            trace["stroke_mm"].append(round(self.stroke * 1000.0, 3))
        if r.ended and not self.ended:
            self.ended = r.ended
        self.wall["rules"] += time.time() - t1

    # ------------------------------------------------------------------ bodies the rules hand over
    def _set_kinematic(self, path, on):
        self.UsdPhysics.RigidBodyAPI(self._prim(path)).GetKinematicEnabledAttr().Set(bool(on))

    def _free(self, pid):
        self._set_kinematic(self.index["parts"][pid], False)

    def _attach(self, pid, T_part):
        from pxr import Gf
        rel = kin.inv(self.ee_T) @ T_part
        q = kin.R_quat(rel[:3, :3])
        j = self.UsdPhysics.FixedJoint.Define(self.stage, "/World/GraspJoint")
        j.CreateBody0Rel().SetTargets([self.index["ee"]])
        j.CreateBody1Rel().SetTargets([self.index["parts"][pid]])
        j.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in rel[:3, 3]]))
        j.CreateLocalRot0Attr(Gf.Quatf(float(q[0]), float(q[1]), float(q[2]), float(q[3])))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr(True)
        self.grasp_joint = pid
        if not self.rules.part[pid]["locked"]:
            self._free(pid)

    def _detach(self):
        if self.grasp_joint is not None:
            self.stage.RemovePrim("/World/GraspJoint")
            # a part that hung still on the joint may have gone to sleep, and a sleeping body does not notice that
            # its joint is gone: writing its velocity wakes it
            i = np.array([self.part_order[self.grasp_joint]], dtype=np.int32)
            self.pv.set_velocities(np.array(self.pv.get_velocities(), dtype=np.float32), i)
            self.grasp_joint = None

    def _set_pose(self, path, T):
        """Move a kinematic body by authoring its pose on the stage (the physics engine reads a kinematic body's
        pose from there, also when the body is made dynamic afterwards)."""
        from pxr import Gf
        ops = self.UsdGeom.Xformable(self._prim(path)).GetOrderedXformOps()
        q = kin.R_quat(T[:3, :3])
        ops[0].Set(Gf.Vec3d(float(T[0, 3]), float(T[1, 3]), float(T[2, 3])))
        ops[1].Set(Gf.Quatd(float(q[0]), float(q[1]), float(q[2]), float(q[3])))

    def _capture(self, fid):
        """A freed fastener leaves its seat: its body waits out of the way, a copy rides on the bit."""
        i = self.fast_order[fid]
        path = self.index["fasteners"][fid]
        for name in ("col_head", "col_shank"):
            self._collide(path + "/" + name, False)
        self._set_pose(path, kin.pose(np.eye(3), PARK + [0.05 * i, 0.0, 0.0]))
        self._show_tool()

    def _eject(self):
        fid = self.rules.eject()
        if fid is None:
            return False
        head = S.HEADS[self.rules.fast[fid]["head"]]
        tip, axis = self.tip()
        # head toward the tool, clear of the bit, then it falls
        Rb = self.sp_T[:3, :3] @ np.diag([1.0, -1.0, -1.0])
        path = self.index["fasteners"][fid]
        self._set_pose(path, kin.pose(Rb, tip + axis * (head["height"] + 0.004)))
        self.sim.step(render=False)            # the body is at the bit before it is let go
        for name in ("col_head", "col_shank"):
            self._collide(path + "/" + name, True)
        self._set_kinematic(path, False)
        self._show_tool()
        return True

    # ------------------------------------------------------------------ commands
    def _command(self, cmd, events):
        r = self.rules
        speed = float(np.max(np.abs(self.qd)))
        still = speed <= 0.02
        if cmd == "eject":
            events.append("eject: ok" if self._eject() else "eject: nothing on the bit")
        elif cmd == "tool_release":
            if self.tool is None:
                events.append("tool_release: refused, no tool mounted")
            elif r.captured is not None or r.grasp is not None:
                events.append("tool_release: refused, the tool holds something")
            elif not (still and r.at_dock(self.flange_T, 0.0, self.tool)):
                events.append("tool_release: refused, the flange is not at rest in the %s dock" % self.tool)
            else:
                events.append("tool_release: ok")
                self.tool = None
                r.tool = None
                self.busy_ticks = int(round(self.cfg["tool_change_s"] / S.TICK))
                self._show_tool()
        elif cmd == "tool_lock":
            here = [n for n in S.CELL["docks"] if still and r.at_dock(self.flange_T, 0.0, n)]
            if self.tool is not None:
                events.append("tool_lock: refused, a tool is mounted")
            elif not here:
                events.append("tool_lock: refused, the flange is not at rest in a dock")
            else:
                self.tool = r.tool = here[0]
                r.width = self.grip_target = S.TOOLS["gripper"]["open_max"]
                events.append("tool_lock: ok, %s" % self.tool)
                self.busy_ticks = int(round(self.cfg["tool_change_s"] / S.TICK))
                self._show_tool()
        elif cmd.startswith("bit:"):
            bit = cmd[4:]
            tip, axis = self.tip()
            if self.tool != "nutrunner":
                events.append("%s: refused, the nutrunner is not mounted" % cmd)
            elif r.captured is not None:
                events.append("%s: refused, a fastener is on the bit" % cmd)
            elif not (still and r.at_bit_station(tip, axis, 0.0)):
                events.append("%s: refused, the bit is not at rest in the bit station" % cmd)
            else:
                r.bit = bit
                events.append("%s: ok" % cmd)
                self.busy_ticks = int(round(self.cfg["bit_change_s"] / S.TICK))
                self._show_tool()
        elif cmd == "done":
            self.ended = self.ended or "done"

    # ------------------------------------------------------------------ one action of the wire
    def step(self, action):
        """Execute a validated action (`wire.check_action`) and return the observation."""
        cfg, r = self.cfg, self.rules
        events = []
        if self.ended:
            return self.observe(events=["the pack has ended: " + self.ended])
        a = action
        move = a["move"]
        n = int(move["ticks"])
        if a["gripper"] is not None:
            self.grip_target = float(a["gripper"])
        spindle = None
        if a["spindle"] is not None and self.tool == "nutrunner":
            nr = S.TOOLS["nutrunner"]
            rpm = max(-nr["speed_max_rpm"], min(nr["speed_max_rpm"], a["spindle"]["speed_rpm"]))
            spindle = (rpm * 2 * math.pi / 60.0, max(0.0, min(nr["torque_max"], a["spindle"]["torque_limit"])))
        trace = {"force_axial": [], "torque": [], "angle_deg": [], "stroke_mm": []}
        flags = {"clipped": False, "guard": False, "stopped": False, "busy": False}
        moving = move["type"] != "hold" and self.lock_ticks == 0 and self.busy_ticks == 0
        if move["type"] != "hold" and not moving:
            flags["busy" if self.busy_ticks > 0 else "stopped"] = True
        q0 = self.q_cmd.copy()
        tool = self.tcp_offset()
        delta = np.array(move["delta"], dtype=float)
        if moving and move["type"] == "tcp":
            T0 = kin.flange(q0) @ tool
            peak = 1.5 / (n * S.TICK)
            k = max(1.0, np.linalg.norm(delta[:3]) * peak / cfg["tcp_speed_max"], np.linalg.norm(delta[3:]) * peak / cfg["tcp_rot_speed_max"])
            if k > 1.0:
                delta, flags["clipped"] = delta / k, True
        elif moving and move["type"] == "joint":
            peak = 1.5 / (n * S.TICK)
            k = max(1.0, float(np.max(np.abs(delta) * peak / np.array(cfg["joint_speed_max"]))))
            if k > 1.0:
                delta, flags["clipped"] = delta / k, True
        f_start = self.f_ext.copy()
        stops0 = self.stops
        dq_max = np.array(cfg["joint_speed_max"]) * S.TICK
        for j in range(1, n + 1):
            q_next = self.q_cmd
            if moving and self.lock_ticks == 0:
                s = j / float(n)
                s = s * s * (3 - 2 * s)                  # smooth start and stop
                if move["type"] == "tcp":
                    Tj = kin.pose(kin.rotvec_R(s * delta[3:]) @ T0[:3, :3], T0[:3, 3] + s * delta[:3])
                    q_ik, _ = kin.ik_step(self.q_cmd, Tj, tool)
                else:
                    q_ik = np.clip(q0 + s * delta, kin.Q_LIMITS[:, 0], kin.Q_LIMITS[:, 1])
                dq = q_ik - self.q_cmd
                k = float(np.max(np.abs(dq) / dq_max))
                if k > 1.0:
                    dq, flags["clipped"] = dq / k, True
                q_next = self.q_cmd + dq
            self._tick(q_next, spindle, trace)
            if self.stops != stops0:
                flags["stopped"], moving = True, False
            if moving and move["guard_force"] is not None and np.linalg.norm(self.f_ext - f_start) > move["guard_force"]:
                flags["guard"], moving = True, False
            if moving and move["guard_stroke_mm"] is not None and self.stroke * 1000.0 > move["guard_stroke_mm"]:
                flags["guard"], moving = True, False
            if self.ended or r.t >= cfg["sim_cap_s"]:
                break
        if a["cmd"] and not self.ended:
            self._command(a["cmd"], events)
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
        """After the end: the arm holds, the bodies come to rest."""
        if getattr(self, "_settled", False):
            return
        self._settled = True
        for _ in range(int(round(self.cfg["settle_s"] / S.TICK))):
            for _k in range(S.SUBSTEPS):
                self.sim.step(render=False)

    def observe(self, events=None, flags=None, trace=None, cams=()):
        r, cfg = self.rules, self.cfg
        noise = self.rng.normal(0.0, 1.0, 6) * np.array([cfg["noise_force"]] * 3 + [cfg["noise_torque"]] * 3)
        tcp_t = kin.flange(self.q_cmd) @ self.tcp_offset()
        obs = {
            "time": round(r.t, 3),
            "q": [round(float(v), 6) for v in self.q],
            "qd": [round(float(v), 5) for v in self.qd],
            "q_target": [round(float(v), 6) for v in self.q_cmd],
            "tcp": wire.pose6(self.flange_T @ self.tcp_offset()),
            "tcp_target": wire.pose6(tcp_t),
            "wrench": [round(float(v), 3) for v in np.concatenate([self.f_ext, self.t_ext]) + noise],
            "tool": self.tool or "none",
            "bit": r.bit if self.tool == "nutrunner" else None,
            "spindle": {"torque": round(self.spindle_torque, 3), "speed_rpm": round(self.spindle_speed * 60 / (2 * math.pi), 2),
                        "angle_deg": round(math.degrees(r.angle), 2), "stroke_mm": round(self.stroke * 1000.0, 3)},
            "gripper": {"width": round(r.width, 5) if self.tool == "gripper" else None,
                        "holding": bool(r.grasp is not None)},
            "status": dict(flags or {"clipped": False, "guard": False, "stopped": False, "busy": False},
                           locked_s=round(self.lock_ticks * S.TICK, 2), busy_s=round(self.busy_ticks * S.TICK, 2),
                           protective_stops=self.stops),
            "trace": trace or {"force_axial": [], "torque": [], "angle_deg": [], "stroke_mm": []},
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
    def truth(self):
        t = self.rules.truth(self.poses())
        t["tip"] = [round(float(v), 5) for v in self.tip()[0]] if self.tool == "nutrunner" else None
        # the commanded targets exactly (an observation of a repeated step may carry shifted ones)
        t["tcp_target"] = wire.pose6(kin.flange(self.q_cmd) @ self.tcp_offset())
        t["q_target"] = [round(float(v), 6) for v in self.q_cmd]
        t["time"] = round(self.rules.t, 3)
        return t

    def score(self):
        """The pack's credit as things lie now. After the end (or when the grader has given the controller up: it
        says so with `final`) the bodies are first left to come to rest, the arm holding its pose."""
        return self._score(final=bool(self.ended))

    def _score(self, final):
        if final:
            self._settle()
        poses, fposes = self.poses(), self.fastener_poses()
        credit, detail = self.rules.credit(poses, fposes)
        r = self.rules
        where = {pid: [round(float(v), 3) for v in poses[pid][:3, 3]] for pid in self.part_ids}
        where.update({fid: [round(float(v), 3) for v in T[:3, 3]] for fid, T in fposes.items()})
        return {"credit": credit, "detail": detail, "where": where, "ended": self.ended, "events": r.events, "time": round(r.t, 2),
                "ticks": self.tick_n, "frames": self.frames_used, "protective_stops": self.stops,
                "stripped": sorted(fid for fid, f in r.fast.items() if f["stripped"]),
                "wall": {k: round(v, 1) for k, v in self.wall.items()}}
