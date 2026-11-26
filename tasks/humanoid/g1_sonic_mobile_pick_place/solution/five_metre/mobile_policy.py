"""Five-metre practice controller using public odometry and the released SONIC encoder."""

import math
import os
from pathlib import Path

import numpy as np

from sonic_sim.decoder import DEFAULT_Q
from sonic_sim.encoder import ReferenceEncoder
from v4 import V4


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw(quat):
    w, x, y, z = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class MobilePolicy:
    def __init__(self):
        assets = Path(os.environ.get("SONIC_ASSETS", "/opt/sonic/assets"))
        self.encoder = ReferenceEncoder(assets / "policy/model_encoder.onnx")
        self.visual = V4()
        self.reset()

    def reset(self):
        self.previous = None
        self.quiet = 0
        self.restore_tick = None
        self.manipulating = False
        self.visual.reset()

    def snapshot(self):
        return self.previous, self.quiet, self.restore_tick, self.manipulating, self.visual.snapshot()

    def restore(self, state):
        self.previous, self.quiet, self.restore_tick, self.manipulating, memory = state
        self.visual.restore(memory)

    def _encode(self, joint_positions, joint_velocities, anchor, retraction):
        pose = joint_positions.copy()
        pose[:, [15, 22]] += 0.46 * retraction
        pose[:, 16] += 0.055 * retraction
        pose[:, 23] -= 0.055 * retraction
        return self.encoder.codes_g1(pose, joint_velocities, anchor)

    def _walk(self, tick, xy, heading, velocity, yaw_rate):
        # Fixed commanded dock waypoint. Object and pad positions come only from
        # the visual policy after the robot has physically settled.
        dx, dy = np.asarray((0.18, -0.012)) - xy
        cs, sn = np.cos(heading), np.sin(heading)
        forward, left = cs * dx + sn * dy, -sn * dx + cs * dy
        error = wrap(-0.055 - heading)
        cross_track = -np.sin(error) * forward + np.cos(error) * left
        remaining_route = np.cos(error) * forward + np.sin(error) * left
        steering = error + np.arctan(1.5 * cross_track)
        speed = cs * velocity[0] + sn * velocity[1]

        future = np.arange(10) * 5
        sample = future[:, None] + np.array([-0.25, 0.0, 0.25])[None, :]
        phase = 2 * np.pi * (tick + sample) / 45
        warmup = np.clip((tick + sample - 25) / 15, 0, 1)
        remaining = remaining_route - 0.075 - np.clip(speed, 0.0, 0.48) * sample / 50
        amplitude = np.clip(remaining / 0.24, 0.0, 1.0) * warmup
        q = np.broadcast_to(DEFAULT_Q, (*phase.shape, 29)).copy()
        for side, hip, shoulder in ((1, 0, 15), (-1, 6, 22)):
            foot = -side * 0.25 * np.cos(phase)
            knee = 0.40 * np.maximum(side * np.sin(phase), 0.0)
            roll = 0.065 * np.sin(phase)
            q[:, :, hip] += (-foot - knee / 2) * amplitude
            q[:, :, hip + 3] += knee * amplitude
            q[:, :, hip + 4] += (foot - knee / 2) * amplitude
            q[:, :, hip + 1] += roll * amplitude
            q[:, :, hip + 5] -= roll * amplitude
            q[:, :, shoulder] += 0.55 * foot * amplitude
        bias = np.clip(0.18 * steering, -0.19, 0.19)
        rate = np.clip(1.8 * steering - 0.08 * yaw_rate, -0.75, 0.75)
        rot = (bias + rate * future / 50) * amplitude[:, 1]
        c, s = np.cos(rot), np.sin(rot)
        zeros = np.zeros_like(c)
        anchor = np.stack((c, -s, s, c, zeros, zeros), axis=1)
        dq = (q[:, 2] - q[:, 0]) * 100
        return self._encode(q[:, 1], dq, anchor, float(np.clip((xy[0] + 1.30) / 0.55, 0, 1)))

    def act(self, public):
        if self.manipulating:
            return self.visual.act(public)
        tick = public.tick
        xy = np.asarray(public.proprio["base_pos"][:2])
        heading = yaw(public.proprio["base_quat"])
        old_tick, old_xy, old_yaw = (
            self.previous if self.previous is not None else (tick - 1, xy, heading)
        )
        dt = (tick - old_tick) / 50
        velocity = (xy - old_xy) / dt
        yaw_rate = wrap(heading - old_yaw) / dt
        self.previous = tick, xy.copy(), heading
        stable = (
            abs(xy[0]) < 0.099
            and abs(xy[1]) < 0.105
            and abs(heading) < 0.29
            and np.linalg.norm(velocity) < 0.1
        )
        self.quiet = self.quiet + 1 if stable else 0
        if self.restore_tick is None and self.quiet >= 45:
            self.restore_tick = tick
        if self.restore_tick is None:
            code = self._walk(tick, xy, heading, velocity, yaw_rate)
        else:
            age = tick - self.restore_tick
            if age >= 90 and self.quiet >= 20 and tick % 5 == 0:
                self.manipulating = True
                return self.visual.act(public)
            pose = np.broadcast_to(DEFAULT_Q, (10, 29))
            anchor = np.broadcast_to([1, 0, 0, 1, 0, 0], (10, 6))
            code = self._encode(pose, np.zeros_like(pose), anchor, float(np.clip(1 - age / 45, 0, 1)))
        return [{"codes": code.tolist(), "hand": [0.0, 0.0]}]
