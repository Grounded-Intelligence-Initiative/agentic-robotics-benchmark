"""Steer through a measured outgoing rack and recover an upright green stop."""

import math

import numpy as np

from rack_vision import rack_from_rgbd
from route import rotation, wrap


class RackPolicy:
    def __init__(self):
        self.entrance = None
        self.heading = None
        self.length = None
        self.height = None
        self.phase = "survey"
        self.lower_tick = None
        self.rise_tick = None
        self.goal = None
        self.depth = 0.0
        self.debug = {}

    @staticmethod
    def posture_depth(underside_m):
        """Select hip flexion from measured opening height relative to the floor."""
        return 0.50 if underside_m < 1.31 else 0.0

    def command(self, obs, s, green, gait):
        local_tick = s.tick - s.entered
        nominal = (1 if s.turn_sign > 0 else -1) * math.pi / 2
        if self.phase in ("survey", "align") and (
            self.entrance is None or local_tick % 4 == 0
        ):
            m = rack_from_rgbd(
                obs["camera"], obs["observation"]["gravity_body"], s.xy, s.yaw, nominal
            )
            if m["visible"] and m["confidence"] > 0.54:
                center = np.asarray(m["center_xy"])
                if (
                    self.entrance is None
                    or np.linalg.norm(self.entrance - center) < 0.25
                ):
                    w = 1.0 if self.entrance is None else 0.12
                    self.entrance = (
                        center
                        if self.entrance is None
                        else (1 - w) * self.entrance + w * center
                    )
                    self.height = (
                        m["underside_m"]
                        if self.height is None
                        else (1 - w) * self.height + w * m["underside_m"]
                    )
                    h = m["heading"]
                    self.heading = (
                        h
                        if self.heading is None
                        else self.heading + w * wrap(h - self.heading)
                    )
                    d = m.get("depth_m")
                    if d is not None and 0.54 < d < 0.90:
                        self.length = (
                            d if self.length is None else (1 - w) * self.length + w * d
                        )
                    self.phase = "align"
        axis_heading = self.heading if self.heading is not None else nominal
        axes = rotation(axis_heading)
        if green is not None:
            observed = s.xy + rotation(s.yaw) @ green - 1.50 * axes[:, 0]
            self.goal = (
                observed if self.goal is None else 0.3 * observed + 0.7 * self.goal
            )

        heading_error = wrap(axis_heading - s.yaw)
        vel = axes.T @ s.velocity
        vx, vy = vel
        f = y = 0.0
        motion = "pivot" if abs(heading_error) > 0.048 else "stand"
        if self.entrance is not None:
            delta = axes.T @ (self.entrance - s.xy)
            f, y = delta
            if self.phase == "align":
                motion = "stage"
                f -= 0.93
                if (
                    abs(f) < 0.23
                    and abs(y) < 0.10
                    and abs(vy) < 0.23
                    and abs(heading_error) < 0.12
                ):
                    self.phase = "transit"
                    self.depth = self.posture_depth(self.height)
                    self.lower_tick = local_tick
            if self.phase == "transit":
                motion = "rack"
                f = delta[0] + 1.65
                # Keep the torso and trailing foot clear of the measured rear post.
                length = self.length if self.length is not None else 0.82
                if delta[0] < -length - 0.30:
                    self.phase = "recover"
                    self.rise_tick = local_tick
            if self.phase in ("recover", "park") and self.goal is not None:
                target = self.goal
                f, y = axes.T @ (target - s.xy)
                motion = "stage"
                distance = np.linalg.norm(target - s.xy)
                if (
                    distance < 0.166
                    and abs(heading_error) < 0.115
                    and np.linalg.norm(vel) < 0.145
                    and abs(s.yaw_rate) < 0.19
                ):
                    self.phase = "park"
                if self.phase == "park":
                    motion = "stand"
                    if distance > 0.24 or abs(heading_error) > 0.163:
                        self.phase = "recover"
        codes = gait.codes(
            local_tick,
            motion,
            f,
            y,
            heading_error,
            vx,
            s.yaw_rate,
            vy,
            posture_depth=self.depth,
            lowering_tick=self.lower_tick,
            rising_tick=self.rise_tick,
        )
        self.debug = dict(
            phase=self.phase,
            entrance=None if self.entrance is None else self.entrance.tolist(),
            heading=axis_heading,
            height=self.height,
            length=self.length,
            depth=self.depth,
            goal=None if self.goal is None else self.goal.tolist(),
            f=f,
            y=y,
            err=heading_error,
        )
        return {"rows": [{"codes": codes.tolist()}]}
