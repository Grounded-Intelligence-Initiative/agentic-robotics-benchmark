"""Native action usage and motion smoothness for a single G1 episode."""
from collections import deque
import math

import numpy as np


class MeasuredSession:
    """Read actions and 29 measured joint velocities from each completed native step."""

    def __init__(self, session):
        self.session = session
        self.n = self.changed = self.jump = 0
        self.target_delta2 = 0.
        self.velocity_history = deque([np.asarray(session.state["observation"]["dq"], dtype=np.float64).copy()], maxlen=5)
        self.joint_accel_square_sum = 0.
        self.joint_accel_samples = 0
        self.codes_seen = set()
        self.last_codes = self.last_target = None

    def __getattr__(self, key):
        return getattr(self.session, key)

    def step(self, token, hand_l, hand_r):
        state = self.session.step(token, hand_l, hand_r)
        velocity = np.asarray(state["observation"]["dq"], dtype=np.float64)
        if len(self.velocity_history) == 5:
            acceleration = (velocity - self.velocity_history[0]) / .10
            self.joint_accel_square_sum += float(acceleration @ acceleration)
            self.joint_accel_samples += velocity.size
        self.velocity_history.append(velocity.copy())
        codes = np.rint(np.asarray(token) * 16 + 16).astype(np.int16)
        self.codes_seen.add(codes.astype(np.uint8).tobytes())
        target = np.asarray(self.session.last_target)
        if self.last_codes is not None:
            diff = np.abs(codes - self.last_codes)
            self.changed += int(np.count_nonzero(diff))
            self.jump += int(diff.sum())
            self.target_delta2 += float(np.mean((target - self.last_target)**2))
        self.last_codes, self.last_target = codes, target.copy()
        self.n += 1
        return state

    def action_metrics(self):
        transitions = max(self.n - 1, 0)
        return dict(applied_rows=self.n, applied_codes=64*self.n, simulated_s=self.n / 50,
                    unique_code_rows=len(self.codes_seen), changed_coordinates=self.changed,
                    code_transition_slots=64*transitions, absolute_bin_jump=self.jump,
                    coordinate_change_fraction=self.changed / (64*transitions) if transitions else 0.,
                    target_step_rms_rad=math.sqrt(self.target_delta2 / max(1, transitions)),
                    joint_acceleration_samples=self.joint_accel_samples,
                    joint_accel_square_sum=self.joint_accel_square_sum,
                    joint_accel_rms_rad_s2=math.sqrt(self.joint_accel_square_sum / max(1, self.joint_accel_samples)))
