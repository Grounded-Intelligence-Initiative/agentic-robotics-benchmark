"""Independent physical audit of a continuous walk, dock, lift and released placement."""
import math

import numpy as np

from sonic_sim.motion_cost import MeasuredSession


class MobileScore:
    def __init__(self, geometry, sample):
        self.geometry = geometry
        self.initial = np.asarray(geometry["start_pelvis_xyz"])
        self.feet = np.asarray(geometry["start_ankle_xyz"])
        self.foot_rise = np.zeros(2)
        self.minimum_height = float(self.initial[2])
        self.forward = 0.
        self.max_forward = 0.
        self.last_xy = self.initial[:2].copy()
        self.last_v = np.zeros(2)
        self.path = self.acc2_integral = 0.
        self.last_time = float(sample["time_s"])
        self.support_side = self.qualified_side = None
        self.support_duration = 0.
        self.support_phases = 0
        self.brace_hold = self.dock_hold = self.max_dock_hold = 0.
        self.lift_hold = self.release_hold = self.max_lift = 0.
        self.walk_s = self.dock_s = self.touch_s = self.lift_s = self.place_s = None
        self.invalid = None
        self.fallen = False

    def update(self, s):
        now = float(s["time_s"])
        dt = now - self.last_time
        if not math.isclose(dt, .002, rel_tol=0., abs_tol=1e-8):
            raise ValueError("mobile audit requires uninterrupted 2 ms physics samples")
        self.last_time = now
        pos = np.asarray(s["pelvis_xyz"])
        feet = np.asarray(s["ankle_xyz"])
        displ = pos[:2] - self.last_xy
        vxy = displ / dt
        delta_v = vxy - self.last_v
        self.acc2_integral += float(delta_v @ delta_v) / dt
        self.path += float(np.linalg.norm(displ))
        self.last_xy, self.last_v = pos[:2].copy(), vxy
        self.minimum_height = min(self.minimum_height, pos[2])
        self.forward = float(pos[0] - self.initial[0])
        self.max_forward = max(self.max_forward, self.forward)
        self.foot_rise = np.maximum(self.foot_rise, feet[:, 2] - self.feet[:, 2])
        self.fallen |= bool(pos[2] <= .45 or s["pelvis_up"] <= .55)

        if self.dock_s is None:
            left, right = s["sole_force_n"]
            support = 0 if left >= 20 and right < 5 else 1 if right >= 20 and left < 5 else None
            self.support_duration = self.support_duration + dt if support == self.support_side else dt
            self.support_side = support
            if support is not None and support != self.qualified_side and self.support_duration >= .040 - 1e-9:
                self.support_phases += 1
                self.qualified_side = support
            if (self.walk_s is None and self.forward >= .72 and min(feet[:, 0] - self.feet[:, 0]) >= .60
                    and min(self.foot_rise) >= .020 and self.support_phases >= 3):
                self.walk_s = now
            self.brace_hold = self.brace_hold + dt if s["nonsole_support_n"] >= 15 else 0.
            if self.invalid is None:
                if s["hand_contact"]:
                    self.invalid = "hand contact before docking"
                elif self.brace_hold >= .020 - 1e-9:
                    self.invalid = "non-sole support before docking"
            within = ((self.geometry["start_mode"] == "near" or self.walk_s is not None) and abs(pos[0]) <= .100 and abs(pos[1]) <= .110
                      and abs(s["pelvis_yaw"]) <= .30 and pos[2] >= .62 and s["pelvis_up"] >= .94
                      and s["pelvis_speed_xy"] <= .10)
            self.dock_hold = self.dock_hold + dt if within else 0.
            self.max_dock_hold = max(self.max_dock_hold, self.dock_hold)
            if self.dock_hold >= .50 - 1e-9 and self.invalid is None and not self.fallen:
                self.dock_s = now
        if s["hand_contact"] and self.touch_s is None:
            self.touch_s = now
        xyz = np.asarray(s["object_xyz"])
        lift = float(xyz[2] - self.geometry["initial_object_z_m"])
        self.max_lift = max(self.max_lift, lift)
        valid = self.dock_s is not None and self.invalid is None and not self.fallen
        if not valid:
            return
        weight = self.geometry["object_mass_kg"] * 9.81
        bearing = (s["opposed"] and s["hand_load_z_n"] >= .5 * weight
                   and s["support_load_z_n"] <= .05 * weight and lift >= .04)
        self.lift_hold = self.lift_hold + dt if bearing else 0.
        if self.lift_s is None and self.lift_hold >= .040 - 1e-9:
            self.lift_s = now
        if self.lift_s is None:
            return
        rot = np.asarray(s["object_rotation"]).reshape(3, 3)
        size = self.geometry["object_size_m"]
        if self.geometry["object_shape"] == "box":
            extent = abs(rot[:2]) @ size
        else:
            axis = abs(rot[:2, 2])
            extent = size[0] * np.sqrt(np.maximum(0., 1. - axis ** 2)) + size[-1] * axis
        vel = np.asarray(s["object_velocity"])
        delta = xyz - s["pad_xyz"]
        released = (s["pad_load_z_n"] >= .5 * weight and not s["hand_contact"]
                    and s["support_load_z_n"] - s["pad_load_z_n"] <= .05 * weight
                    and np.all(np.abs(delta[:2]) + extent <= self.geometry["pad_half_width_m"])
                    and abs(rot[2, 2]) >= .95 and abs(delta[2]) < .009
                    and np.linalg.norm(vel[:3]) < .08 and np.linalg.norm(vel[3:]) < 1.2)
        self.release_hold = self.release_hold + dt if released else 0.
        if self.place_s is None and self.release_hold >= .250 - 1e-9:
            self.place_s = now

    def result(self):
        return dict(walked=int(self.walk_s is not None), docked=int(self.dock_s is not None),
                    contacted=int(self.touch_s is not None), lifted=int(self.lift_s is not None),
                    placed=int(self.place_s is not None and not self.fallen and self.invalid is None),
                    disqualified=int(self.invalid is not None), fell=int(self.fallen),
                    walk_s=self.walk_s, dock_s=self.dock_s, first_contact_s=self.touch_s,
                    lift_s=self.lift_s, place_s=self.place_s, simulated_s=self.last_time,
                    forward_m=self.forward, max_forward_m=self.max_forward,
                    horizontal_path_m=self.path, minimum_pelvis_height_m=self.minimum_height,
                    left_foot_rise_m=self.foot_rise[0], right_foot_rise_m=self.foot_rise[1],
                    single_support_phases=self.support_phases, best_dock_hold_s=self.max_dock_hold,
                    max_lift_m=self.max_lift, release_hold_s=self.release_hold,
                    base_xy_acceleration_rms_m_s2=math.sqrt(self.acc2_integral / max(.002, self.last_time)))


class AuditedSession(MeasuredSession):
    """Check all native contact samples and retain diagnostics for each applied row."""

    def __init__(self, session):
        super().__init__(session)
        self.score = MobileScore(session.episode_geometry, session.last_substeps[0])

    def step(self, token, hand_l, hand_r):
        state = super().step(token, hand_l, hand_r)
        for sample in self.session.last_substeps:
            self.score.update(sample)
        return state

    def report(self):
        return {**self.score.result(), **self.action_metrics()}
