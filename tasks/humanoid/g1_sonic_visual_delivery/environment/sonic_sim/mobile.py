"""Ordered walk, dock and contact evidence for the mobile G1 tabletop scene."""
import math

import mujoco
import numpy as np

from .physics import G1Task, PAD_HALF_WIDTH, PAD_TOP, PHYSICS_DT, SIDES, ContactSample, _uniform

HORIZON = 1100
START_MODES = ("near", "far")
SHAPES = ("box", "cylinder")
TARGET_SIDES = ("left", "right")


def _yaw(quat):
    w, x, y, z = quat
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


class DeliveryTask(G1Task):
    def __init__(self, seed, robot_root, start_mode=None, shape=None, target_side=None):
        area = "g1_sonic_visual_delivery"
        def factor(name, supplied, options):
            if supplied is None:
                return options[int(_uniform(seed, area, name, 0, 2))]
            if supplied not in options:
                raise ValueError(f"unknown {name}: {supplied!r}")
            return supplied
        self.start_mode = factor("start-mode", start_mode, START_MODES)
        self.shape = factor("shape", shape, SHAPES)
        self.target_side = factor("target-side", target_side, TARGET_SIDES)
        super().__init__(self.shape, self.target_side, seed, robot_root)
        m, d = self.model, self.data
        if self.start_mode == "far":
            x = _uniform(seed, area, "far-x", -1.005, -.965)
            y = _uniform(seed, area, "far-y", .078, .103)
            angle = _uniform(seed, area, "far-heading", -.038, -.026)
        else:
            x = _uniform(seed, area, "near-x", -.010, .010)
            y = _uniform(seed, area, "near-y", -.010, .010)
            angle = _uniform(seed, area, "near-heading", -.020, .020)
        # These initial conditions are written before the first physics step.
        d.qpos[:2] = [x, y]
        d.qpos[3:7] = [math.cos(angle/2), 0., 0., math.sin(angle/2)]
        mujoco.mj_forward(m, d)
        self.floor = m.geom("floor").id
        self.fixed = set(np.flatnonzero(m.geom_bodyid == 0))
        self.ankles = [m.body(side + "_ankle_roll_link").id for side in SIDES]
        self.sole_geoms = {g: self.ankles.index(int(m.geom_bodyid[g])) for g in range(m.ngeom)
                           if m.geom_bodyid[g] in self.ankles
                           and m.geom_group[g] == 3 and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE}
        self.force = np.zeros(6)
        self.initial_pelvis = d.xpos[self.pelvis].copy()
        self.initial_ankles = d.xpos[self.ankles].copy()
        self.peak_foot_rise = np.zeros(2)
        self.single_candidate = None
        self.single_duration = 0
        self.single_sides = []
        self.settle_substeps = 0
        self.brace_substeps = 0
        self.violation = None
        self.walked = False
        self.dock_tick = self.contact_tick = self.lift_tick = None
        self.substeps = 0
        self.last_substeps = []
        self._record(0., ContactSample(), d.qvel[self.object_vadr:self.object_vadr+6])

    def geometry(self):
        return dict(start_mode=self.start_mode, shape=self.shape, target_side=self.target_side,
                    initial_object_z_m=self.score.initial_z, object_mass_kg=self.config.mass,
                    object_size_m=list(self.config.size), object_shape=self.config.shape,
                    pad_half_width_m=PAD_HALF_WIDTH, pad_top_m=PAD_TOP, pad_xyz=self.goal_pos.tolist(),
                    start_pelvis_xyz=self.initial_pelvis.tolist(), start_ankle_xyz=self.initial_ankles.tolist())

    def _floor_forces(self):
        loads = [0., 0.]
        bracing = 0.
        for i, c in enumerate(self.data.contact):
            a, b = int(c.geom1), int(c.geom2)
            if a in self.fixed:
                static, other = a, b
            elif b in self.fixed:
                static, other = b, a
            else:
                continue
            if other in self.fixed or other == self.object_geom:
                continue
            mujoco.mj_contactForce(self.model, self.data, i, self.force)
            normal = max(0., float(self.force[0]))
            if static == self.floor and other in self.sole_geoms:
                loads[self.sole_geoms[other]] += normal
            else:
                bracing += normal
        return loads, bracing

    def _walk(self, feet, loads, forward):
        self.peak_foot_rise = np.maximum(self.peak_foot_rise, feet[:, 2] - self.initial_ankles[:, 2])
        candidate = None
        for i, load in enumerate(loads):
            if load >= 20. and loads[1-i] < 5.:
                candidate = i
        if candidate != self.single_candidate:
            self.single_candidate, self.single_duration = candidate, 0
        if candidate is not None:
            self.single_duration += 1
            if self.single_duration == 20 and (not self.single_sides or self.single_sides[-1] != candidate):
                self.single_sides.append(candidate)
        self.walked |= bool(forward >= .72 and np.all(feet[:, 0] - self.initial_ankles[:, 0] >= .60)
                            and np.all(self.peak_foot_rise >= .02) and len(self.single_sides) >= 3)

    def _record(self, dt, contacts, velocity):
        d = self.data
        pelvis = d.xpos[self.pelvis]
        feet = d.xpos[self.ankles]
        loads, brace = self._floor_forces()
        speed = float(np.linalg.norm(d.qvel[:2]))
        up = float(d.xmat[self.pelvis, 8])
        yaw = _yaw(d.xquat[self.pelvis])
        forward = float(pelvis[0] - self.initial_pelvis[0])
        if dt:
            self.substeps += 1
            tick = (self.substeps + 9) // 10
            if contacts.hand_contact and self.contact_tick is None:
                self.contact_tick = tick
            if self.score.lifted and self.lift_tick is None:
                self.lift_tick = tick
            if self.dock_tick is None and self.violation is None:
                self._walk(feet, loads, forward)
                self.brace_substeps = self.brace_substeps + 1 if brace >= 15. else 0
                if contacts.hand_contact:
                    self.violation = "early_object_contact"
                elif self.brace_substeps >= 10:
                    self.violation = "braced_before_dock"
                inside = ((self.start_mode == "near" or self.walked) and abs(pelvis[0]) <= .10 and abs(pelvis[1]) <= .11
                          and abs(yaw) <= .30 and up >= .94 and pelvis[2] >= .62 and speed <= .10)
                self.settle_substeps = self.settle_substeps + 1 if inside else 0
                if self.settle_substeps >= 250 and self.violation is None:
                    self.dock_tick = tick
                    self.model.geom_rgba[self.pad_geom] = (.16, .65, .47, 1.)
        self.last_substeps.append(dict(
            time_s=self.substeps*PHYSICS_DT, dt_s=dt, pelvis_xyz=pelvis.tolist(), pelvis_up=up,
            pelvis_yaw=yaw, pelvis_speed_xy=speed, ankle_xyz=feet.tolist(), sole_force_n=loads,
            nonsole_support_n=brace, object_xyz=d.xpos[self.object_body].tolist(),
            object_velocity=np.asarray(velocity).tolist(), object_rotation=d.xmat[self.object_body].reshape(3, 3).tolist(),
            pad_xyz=self.goal_pos.tolist(), hand_contact=bool(contacts.hand_contact), opposed=bool(contacts.opposed),
            hand_load_z_n=float(contacts.hand_force_z), support_load_z_n=float(contacts.support_force_z),
            pad_load_z_n=float(contacts.pad_force_z)))

    def _sample(self, dt, contacts, object_velocity):
        super()._sample(dt, contacts, object_velocity)
        if hasattr(self, "last_substeps"):
            self._record(dt, contacts, object_velocity)

    def step(self, joint_targets, hand=(0., 0.)):
        self.last_substeps = []
        super().step(joint_targets, hand)

    def metrics(self):
        out = super().metrics()
        docked = self.dock_tick is not None
        return dict(out, success=bool(out["placed"] and docked and self.violation is None),
                    walked=bool(self.walked), docked=docked, approach_valid=self.violation is None,
                    approach_violation=self.violation, dock_tick=self.dock_tick,
                    first_contact_tick=self.contact_tick, lift_tick=self.lift_tick,
                    forward_m=float(self.data.xpos[self.pelvis, 0]-self.initial_pelvis[0]),
                    foot_rise_m=self.peak_foot_rise.tolist(), single_support_changes=max(0, len(self.single_sides)-1),
                    single_support_phases=len(self.single_sides), settle_ticks=self.settle_substeps//10)
