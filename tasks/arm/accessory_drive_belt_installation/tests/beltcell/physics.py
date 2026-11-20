"""The belt station (cell internals): the MuJoCo scene and its stepping.

Two Franka Pandas (MuJoCo Menagerie, Apache-2.0) stand on the base plate 0.70 m apart, facing the engine; each hand has the
standard two-finger gripper and a six-axis force/torque sensor at the flange. In front of them the engine on its pallet
shows its accessory face (the drive: crank C, water pump WP, alternator ALT, the automatic tensioner T); left of the
drive the new belt hangs on a padded peg.

World frame: x forward (towards the engine), y left, z up; the base plate top is z = 0. The engine frame E (all drive
geometry, see drive.py): x to the right and z up as seen from the robots, y into the engine.

Stepping: every 10 ms control tick runs 5 physics steps of 2 ms; before each step the arms' torque controllers and the
belt's junction springs are evaluated on the state of that step (mj_step1 / mj_step2). The flange F/T reading of a tick
is the mean over its 5 steps.

Runtime dependencies: mujoco + numpy.
"""

import math
import os

import mujoco
import numpy as np

from . import drive as dr
from .cameras import build_cameras
from .control import IMP, ImpedanceArm, mat_to_quat

__all__ = ["Scene", "Arm", "PHYS", "LAYOUT", "HOME_Q", "GRIPPER", "rot_axis", "q_from_R"]

PHYS = {"timestep": 0.002, "tick_s": 0.01, "iterations": 30, "ls_iterations": 30, "impratio": 10.0}
LAYOUT = {
    "bases": {"L": (0.0, 0.35, 0.0), "R": (0.0, -0.35, 0.0)},
    "E_pos": (0.55, 0.0, 0.30),                 # crank axis on the accessory face, world
    "peg": (-0.33, 0.30),                       # E (x, z) of the peg axis; the peg runs from the face to depth 0.10
    "peg_r": 0.012, "peg_len": 0.10,
}
# E axes in world: x_E = -y_w, y_E = +x_w, z_E = +z_w
R_E0 = np.column_stack([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
HOME_Q = {"L": [0.35, -0.55, 0.0, -2.35, 0.0, 1.85, 0.785], "R": [-0.35, -0.55, 0.0, -2.35, 0.0, 1.85, 0.785]}
GRIPPER = {"kp": 2500.0, "kv": 50.0, "max_force": 70.0}
PAD_FRICTION = (1.5, 0.005, 0.0001)
# geom classes for the safety watch: 1 arm link or hand (not a pad), 2 finger pad, 3 belt, 4 drive / engine / peg / floor
CLS_LINK, CLS_PAD, CLS_BELT, CLS_WORLD = 1, 2, 3, 4


def rot_axis(axis, ang):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def q_from_R(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).reshape(9))
    return q


class Arm(ImpedanceArm):
    """The cell's arm controller (torque-driven joints: joint PD or Cartesian impedance) with the hold-width rule."""

    def __init__(self, model, data, prefix, home_q, tick_s):
        ImpedanceArm.__init__(self, model, data, prefix, prefix + "tcp", gripper=True, tick_s=tick_s)
        self.home_q = np.asarray(home_q, float)
        self.hold_w = None
        self.side = prefix.rstrip("/")

    @staticmethod
    def mat_to_quat(R):
        return mat_to_quat(R)

    def ranges(self):
        return [[float(self.model.jnt_range[j][0]), float(self.model.jnt_range[j][1])] for j in self.joints]

    def rebase(self):
        self.q_cmd = self.q().copy()
        self.q_goal = self.q_cmd.copy()
        self.width_cmd = self.width()
        self.width_goal = self.width_cmd
        self.write_ctrl()

    def width(self):
        return max(0.0, ImpedanceArm.width(self))

    def hold_width(self, w):
        self.hold_w = None if w is None else float(w)
        if self.hold_w is not None and self.width_goal < self.hold_w:
            self.width_goal = self.hold_w

    def command_width(self, width):
        ImpedanceArm.command_width(self, width)
        if self.hold_w is not None and self.width_goal < self.hold_w:
            self.width_goal = self.hold_w


def add_panda(spec, prefix, base_pos, yaw, panda_dir):
    """Menagerie Panda with hand at base_pos/yaw: joint torque motors (the impedance controller drives them), the
    finger tendon actuator re-tuned, pad friction, a tcp site between the pads and a flange F/T sensor."""
    child = mujoco.MjSpec.from_file(os.path.join(panda_dir, "panda.xml"))
    for k in list(child.keys):
        child.delete(k)
    for light in list(child.lights):
        child.delete(light)
    frame = spec.worldbody.add_frame(name=prefix + "base_frame", pos=list(base_pos),
                                     quat=[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])
    spec.attach(child, prefix=prefix, frame=frame)
    for b in spec.bodies:
        if b.name.startswith(prefix):
            b.gravcomp = 1.0
    hand = spec.body(prefix + "hand")
    hand.add_site(name=prefix + "tcp", pos=[0.0, 0.0, 0.1034], size=[0.003, 0.003, 0.003], rgba=[0.2, 0.9, 0.2, 0.5], group=4)
    hand.add_site(name=prefix + "ft", pos=[0.0, 0.0, 0.0], size=[0.004, 0.004, 0.004], group=4)
    pads = []
    for g in spec.geoms:
        b = g.parent
        if b is not None and b.name in (prefix + "left_finger", prefix + "right_finger") and g.type == mujoco.mjtGeom.mjGEOM_BOX:
            g.friction = list(PAD_FRICTION)
            g.condim = 4
            pads.append(g)
    for a in spec.actuators:
        for i in range(7):
            if a.name == prefix + "actuator%d" % (i + 1):
                a.gaintype = mujoco.mjtGain.mjGAIN_FIXED
                a.gainprm = [1.0] + [0.0] * 9
                a.biastype = mujoco.mjtBias.mjBIAS_NONE
                a.biasprm = [0.0] * 10
                a.ctrlrange = [-IMP["tau_max"][i], IMP["tau_max"][i]]
                a.forcerange = [-IMP["tau_max"][i], IMP["tau_max"][i]]
        if a.name == prefix + "actuator8":
            scale = 0.04 / 255.0
            a.gainprm = [GRIPPER["kp"] * scale] + [0.0] * 9
            a.biasprm = [0.0, -GRIPPER["kp"], -GRIPPER["kv"]] + [0.0] * 7
            a.forcerange = [-GRIPPER["max_force"], GRIPPER["max_force"]]
    spec.add_sensor(name=prefix + "ft_force", type=mujoco.mjtSensor.mjSENS_FORCE, objtype=mujoco.mjtObj.mjOBJ_SITE, objname=prefix + "ft")
    spec.add_sensor(name=prefix + "ft_torque", type=mujoco.mjtSensor.mjSENS_TORQUE, objtype=mujoco.mjtObj.mjOBJ_SITE, objname=prefix + "ft")
    return {"tcp_local": (0.0, 0.0, 0.1034)}


class Scene(object):
    """The station for one engine: hidden physics `prm` (layout.py) and the variant."""

    def __init__(self, prm, panda_dir, variant="A", with_arms=True, offscreen=(1280, 960)):
        self.prm = dict(prm)
        self.variant = variant
        self.panda_dir = panda_dir
        self.with_arms = with_arms
        self.tick_s = PHYS["tick_s"]
        self.substeps = int(round(PHYS["tick_s"] / PHYS["timestep"]))
        self._build(offscreen)
        self._index()

    # ------------------------------------------------------------------ build
    def _build(self, offscreen):
        P = self.prm
        spec = mujoco.MjSpec()
        spec.modelname = "belt_station"
        spec.compiler.degree = False
        spec.compiler.autolimits = True
        o = spec.option
        o.timestep = PHYS["timestep"]
        o.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        o.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        o.jacobian = mujoco.mjtJacobian.mjJAC_SPARSE
        o.solver = mujoco.mjtSolver.mjSOL_NEWTON
        o.iterations = PHYS["iterations"]
        o.ls_iterations = PHYS["ls_iterations"]
        o.impratio = PHYS["impratio"]
        o.disableflags |= mujoco.mjtDisableBit.mjDSBL_MULTICCD
        spec.visual.global_.offwidth = int(offscreen[0])
        spec.visual.global_.offheight = int(offscreen[1])
        spec.visual.headlight.diffuse = [0.30, 0.30, 0.30]
        spec.visual.headlight.ambient = [0.34, 0.34, 0.34]
        spec.visual.headlight.specular = [0.0, 0.0, 0.0]
        spec.visual.map.znear = 0.005
        spec.visual.map.zfar = 10.0
        spec.visual.quality.shadowsize = 0
        mats = {}
        for name, rgba, spc in (("steel", (0.70, 0.72, 0.74, 1), 0.5), ("dark", (0.15, 0.16, 0.17, 1), 0.2),
                                ("arm", (0.25, 0.27, 0.30, 1), 0.3), ("lever", (0.85, 0.55, 0.10, 1), 0.2),
                                ("mark", (0.95, 0.95, 0.95, 1), 0.0), ("belt", (0.06, 0.06, 0.06, 1), 0.1),
                                ("engine", (0.45, 0.47, 0.50, 1), 0.2), ("plate", (0.55, 0.53, 0.50, 1), 0.05),
                                ("pallet", (0.62, 0.48, 0.30, 1), 0.05), ("peg", (0.20, 0.35, 0.60, 1), 0.1)):
            spec.add_material(name="mat/" + name, rgba=list(rgba), specular=spc, shininess=0.3)
            mats[name] = "mat/" + name
        self.mats = mats
        w = spec.worldbody
        w.add_geom(name="base_plate", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3.0, 3.0, 0.1], material=mats["plate"])
        for i, (pos, dd) in enumerate((((0.3, 0.0, 2.0), (0.2, 0.0, -1.0)), ((-0.4, 0.8, 1.4), (0.6, -0.6, -0.6)),
                                       ((-0.4, -0.8, 1.4), (0.6, 0.6, -0.6)))):
            w.add_light(name="light%d" % i, pos=list(pos), dir=list(dd), diffuse=[0.42] * 3, specular=[0.05] * 3, castshadow=False)
        # the engine on its pallet (hidden pose variation: dpos, yaw about z, roll about the face normal)
        R_E = rot_axis([0, 0, 1], P.get("yaw", 0.0)) @ R_E0 @ rot_axis([0, 1, 0], P.get("roll", 0.0))
        E_pos = np.asarray(LAYOUT["E_pos"], float) + np.asarray(P.get("dpos", [0, 0, 0]), float)
        self.R_E, self.E_pos = R_E, E_pos
        w.add_geom(name="pallet", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.25, 0.42, 0.05], pos=[E_pos[0] + 0.17, E_pos[1] + 0.05, 0.05],
                   material=mats["pallet"])
        eng = w.add_body(name="engine", pos=E_pos.tolist(), quat=q_from_R(R_E).tolist())
        eng.add_geom(name="eng/face", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.32, 0.01, 0.30], pos=[0.05, 0.01, 0.12],
                     material=mats["engine"])
        eng.add_geom(name="eng/block", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.30, 0.15, 0.10], pos=[0.05, 0.17, 0.0],
                     material=mats["engine"])
        self.desc = dr.add_drive(spec, eng, self.variant, P, mats)
        # the belt's padded peg on a post at the pallet's left end (E frame)
        px, pz = LAYOUT["peg"]
        qy = q_from_R(np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]))
        eng.add_geom(name="peg", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[LAYOUT["peg_r"], 0.5 * LAYOUT["peg_len"]],
                     pos=[px, -0.5 * LAYOUT["peg_len"], pz], quat=qy.tolist(), material=mats["peg"])
        eng.add_geom(name="peg_post", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.02, 0.01, 0.20], pos=[px, 0.0, pz - 0.18],
                     material=mats["dark"])
        bp = {"mass_per_m": P["mass_per_m"], "bend_EI": P["bend_EI"], "print_material": mats["mark"],
              "link": dr.BELT["link"] * float(P.get("len_scale", 1.0)), "n_sub": 8, "n_spheres": 4}
        self.bdesc = dr.add_belt(spec, w, "belt", dr.VARIANTS[self.variant]["belt_links"], bp, mats["belt"])
        # the crank's run-in motor (a velocity servo, off during the episode)
        spec.add_actuator(name="crank_motor", target="eng/C/spin", trntype=mujoco.mjtTrn.mjTRN_JOINT,
                          gaintype=mujoco.mjtGain.mjGAIN_FIXED, biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                          gainprm=[0.0] + [0] * 9, biasprm=[0.0, 0.0, 0.0] + [0] * 7, forcerange=[-30, 30], forcelimited=True)
        if self.with_arms:
            for side, base in LAYOUT["bases"].items():
                add_panda(spec, side + "/", base, 0.0, self.panda_dir)
            # the grip machinery: per hand one inactive weld (a belt link) and two inactive point constraints (the lever)
            for side in LAYOUT["bases"]:
                wd = [0.0] * 11
                wd[10] = 1.0
                spec.add_equality(name="grip/" + side, type=mujoco.mjtEq.mjEQ_WELD, objtype=mujoco.mjtObj.mjOBJ_BODY,
                                  name1=side + "/hand", name2="world", active=False, solref=[0.004, 1.0],
                                  solimp=[0.95, 0.99, 0.001, 0.5, 2.0], data=wd)
                for k in ("a", "b"):
                    spec.add_equality(name="pin/%s/%s" % (side, k), type=mujoco.mjtEq.mjEQ_CONNECT, objtype=mujoco.mjtObj.mjOBJ_BODY,
                                      name1=side + "/hand", name2="world", active=False, solref=[0.004, 1.0], data=[0.0] * 11)
        # the cameras: the fixed front camera over the drive, a wrist camera on each hand (cameras.py)
        self.cam_info = build_cameras(spec, (0.0, 0.0, 0.1034), with_arms=self.with_arms)
        self.spec = spec
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)

    # ------------------------------------------------------------------ index
    def _index(self):
        m, d = self.model, self.data
        self.drive = dr.Drive(m, self.desc, "engine")
        self.belt = dr.Belt(m, self.bdesc)
        dx = dr.BELT["close_dx"]
        for k, e in enumerate(self.belt.close_eq):
            m.eq_data[e, 0:3] = [(-dx, dx)[k % 2], 0.0, self.belt.l]
            m.eq_data[e, 3:6] = [(-dx, dx)[k % 2], 0.0, 0.0]
        for r in self.belt.roots:
            for k in (3, 4, 5):
                m.dof_armature[self.belt.free_dof[r] + k] = dr.BELT["armature"]
        self.belt.setup_junctions(self.bdesc["kf"], self.bdesc["cf"] * 0.3)
        self.crank_act = m.actuator("crank_motor").id
        self.arms = {}
        if self.with_arms:
            for side in LAYOUT["bases"]:
                self.arms[side] = Arm(m, d, side + "/", HOME_Q[side], self.tick_s)
            self.ft_adr = {s: (m.sensor_adr[m.sensor(s + "/ft_force").id], m.sensor_adr[m.sensor(s + "/ft_torque").id])
                           for s in self.arms}
            self.hand_body = {s: m.body(s + "/hand").id for s in self.arms}
            self.finger_bodies = {s: (m.body(s + "/left_finger").id, m.body(s + "/right_finger").id) for s in self.arms}
        # geom classes and arm membership (the safety watch)
        gc = np.zeros(m.ngeom, dtype=int)
        ga = np.zeros(m.ngeom, dtype=int)
        belt_geoms = set(int(g) for g in self.belt.colliders)
        for gi in range(m.ngeom):
            if int(m.geom_contype[gi]) == 0 and int(m.geom_conaffinity[gi]) == 0:
                continue
            bname = m.body(int(m.geom_bodyid[gi])).name
            if gi in belt_geoms:
                gc[gi] = CLS_BELT
            elif bname.startswith(("L/", "R/")):
                ga[gi] = 1 if bname.startswith("L/") else 2
                is_pad = bname.endswith(("left_finger", "right_finger")) and int(m.geom_type[gi]) == int(mujoco.mjtGeom.mjGEOM_BOX)
                gc[gi] = CLS_PAD if is_pad else CLS_LINK
            else:
                gc[gi] = CLS_WORLD
        self.geom_class = gc
        self.geom_arm = ga
        self.physics_fault = False
        self._v6 = np.zeros(6)
        self._ft_acc = {s: np.zeros(6) for s in self.arms}
        self.ft_tick = {s: np.zeros(6) for s in self.arms}

    # ------------------------------------------------------------------ geometry
    def E_to_world(self, x, z, d):
        return self.E_pos + self.R_E @ np.array([x, -d, z])

    def world_to_E(self, p):
        v = self.R_E.T @ (np.asarray(p, float) - self.E_pos)
        return np.array([v[0], v[2], -v[1]])

    def pose_on_peg(self, bottom_r, start):
        """The belt as a closed loop hanging on its peg: an arc over the peg, two spans, a rounded bottom of radius
        bottom_r; the bottom's depth is solved so that the chain closes; the chain starts at path sample `start`.
        The tensioner at its closed stop. (A settle follows, run by the caller.)"""
        n, l = self.belt.n, self.belt.l
        px, pz = LAYOUT["peg"]
        rp = LAYOUT["peg_r"] + 0.5 * dr.BELT["thick"] + 0.0005
        rb = float(bottom_r)

        def make(zb):
            path = []
            for k in range(60):
                a = math.pi - math.pi * k / 60
                path.append((px + rp * math.cos(a), pz + rp * math.sin(a)))
            for k in range(80):
                u = k / 80
                path.append((px + rp + (rb - rp) * u, pz - (pz - zb) * u))
            for k in range(60):
                a = -math.pi * k / 60
                path.append((px + rb * math.cos(a), zb + rb * math.sin(a)))
            for k in range(80):
                u = k / 80
                path.append((px - rb + (rb - rp) * u, zb + (pz - zb) * u))
            path = np.asarray(path)
            per = float(np.sum(np.linalg.norm(np.diff(np.vstack([path, path[:1]]), axis=0), axis=1)))
            return per - n * l, path

        lo, hi = pz - 0.10, pz - 0.70
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            if make(mid)[0] < 0.0008:
                lo = mid
            else:
                hi = mid
        _, path = make(0.5 * (lo + hi))
        pts, _ = dr.resample_closed(np.vstack([path, path]), n, l, start_index=int(start) % len(path))
        P3 = np.array([self.E_to_world(x, z, 0.06) for x, z in pts])
        nvec = np.tile(self.R_E[:, 1], (n, 1))
        self.belt.set_points(self.data, P3, nvec)
        self.data.qpos[self.drive.t_qadr] = 0.0
        mujoco.mj_forward(self.model, self.data)

    # ------------------------------------------------------------------ arms
    def reset_arms_home(self):
        m, d = self.model, self.data
        for side, arm in self.arms.items():
            d.qpos[arm.qadr] = arm.home_q
            d.qvel[arm.vadr] = 0.0
            for jn in ("finger_joint1", "finger_joint2"):
                d.qpos[m.jnt_qposadr[m.joint(side + "/" + jn).id]] = 0.04
        mujoco.mj_forward(m, d)
        for arm in self.arms.values():
            arm.rebase()
            arm.set_mode("joint")
            arm.q_goal = arm.home_q.copy()
            arm.command_width(0.08)

    def ft_raw(self, side):
        a, b = self.ft_adr[side]
        d = self.data
        return np.concatenate([d.sensordata[a:a + 3], d.sensordata[b:b + 3]])

    # ------------------------------------------------------------------ stepping
    def step_physics(self, before_substep=None, arms=True):
        """One control tick: 5 physics steps; arms' torques and the belt's junction springs evaluated before each.
        arms=False: the arms are switched off (their joint torques zero; the acceptance run-in). Returns False on a
        physics fault."""
        m, d = self.model, self.data
        v = self._v6
        for b in self.belt.body:
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, int(b), v, 0)
            d.xfrc_applied[b, 0:3] = -dr.BELT["drag"] * v[3:6]
        for s in self._ft_acc:
            self._ft_acc[s][:] = 0.0
        if not arms:
            for arm in self.arms.values():
                d.ctrl[arm.acts] = 0.0
        for _ in range(self.substeps):
            mujoco.mj_step1(m, d)
            self.belt.apply_junction_torques(d)
            if arms:
                for arm in self.arms.values():
                    arm.write_ctrl()
            if before_substep is not None:
                before_substep()
            mujoco.mj_step2(m, d)
            for s in self._ft_acc:
                self._ft_acc[s] += self.ft_raw(s)
        for s in self._ft_acc:
            self.ft_tick[s] = self._ft_acc[s] / float(self.substeps)
        if int(d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number) > 0 or not np.all(np.isfinite(d.qpos)):
            self.physics_fault = True
            return False
        return True
