"""Walk a procedural, discrete locomotion gait and start the RGB policy from a quiet dock."""
import hashlib
from pathlib import Path

import numpy as np

from stand import STAND_CODES
from v4 import V4


class MobilePolicy:
    def __init__(self):
        path = Path(__file__).with_name("approach_codes.npy")
        if hashlib.sha256(path.read_bytes()).hexdigest() != "fbd5985072b2a64e6ceee8c55d689b8a70a253297c3a130054b7c56502649d8b":
            raise ValueError("unexpected procedural approach codebook")
        self.gait = np.load(path, allow_pickle=False)
        self.actor = V4()
        self.reset()

    def reset(self):
        self.actor.reset()
        self.manipulating = False
        self.last_pose = None
        self.quiet_frames = 0

    def snapshot(self):
        return self.manipulating, self.last_pose, self.quiet_frames, self.actor.snapshot()

    def restore(self, state):
        self.manipulating, self.last_pose, self.quiet_frames, memory = state
        self.actor.restore(memory)

    def act(self, obs):
        if self.manipulating:
            return self.actor.act(obs)
        p = np.asarray(obs.proprio["base_pos"])
        q = np.asarray(obs.proprio["base_quat"])
        yaw = np.arctan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
        if self.last_pose is not None:
            previous_tick, previous_xy = self.last_pose
            speed = np.linalg.norm(p[:2]-previous_xy)*50/(obs.tick-previous_tick)
            quiet = abs(p[0]) <= .10 and abs(p[1]) <= .11 and abs(yaw) <= .30 and speed <= .10
            self.quiet_frames = self.quiet_frames + 1 if quiet else 0
        self.last_pose = obs.tick, p[:2].copy()
        if obs.tick >= 250 and self.quiet_frames >= 5:
            self.manipulating = True
            return self.actor.act(obs)
        rows = []
        for t in range(obs.tick, obs.tick + 5 - obs.tick % 5):
            index = t-30
            code = self.gait[index] if 0 <= index < len(self.gait) else STAND_CODES
            rows.append({"codes": code.tolist(), "hand": [0., 0.]})
        return rows
