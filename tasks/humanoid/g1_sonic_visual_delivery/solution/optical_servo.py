"""Closed-loop optical picking with a native-code Cartesian servo, at 50 Hz."""

from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation
from direct_token_control import DirectTokenController
from optical_scene import SceneTracker
from stand import STAND_CODES


def blend(age, period):
    x = np.clip(age / period, 0.0, 1.0)
    return x * x * (3 - 2 * x)


class OpticalServo:
    """Hold the item upright and move its observed center onto the lit pad."""

    def __init__(self, model):
        self.model = DirectTokenController(model)
        self.scene = SceneTracker()
        self.phase = "settle"
        self.entered_tick = 0
        self.tick = 0
        self.pick = None
        self.level = np.zeros(3)

    def change(self, phase):
        self.phase = phase
        self.entered_tick = self.tick + 1

    def actions(self, pose, views, body):
        obj, goal = self.scene.observe(views, body)
        hand = np.asarray(pose["right_hand_pos"])
        if obj is None or goal is None:
            return [
                {"codes": STAND_CODES.tolist(), "hand": [0.0, 0.0]} for _ in range(5)
            ]
        is_box = self.scene.half_height > 0.055
        bias = np.array([-0.005, 0.0, 0.025] if is_box else [-0.003, 0.0, 0.016])
        goal = goal.copy()
        # Compensate the measured forward landing shortfall after release.
        goal[0] += 0.018
        if is_box:
            goal[:2] += [0.010, -0.006]
        rows = []
        for _ in range(5):
            age = self.tick - self.entered_tick
            phase = self.phase
            grip = 0.0
            if phase == "settle":
                target = None
                if age >= 30:
                    self.change("retract")
            elif phase == "retract":
                target = np.array([0.072, -0.23, 0.76])
                if age >= 40:
                    self.change("raise")
            elif phase == "raise":
                target = np.array([0.09, -0.26, 0.875])
                if (age >= 50 and hand[2] >= 0.85) or age >= 105:
                    self.start = hand.copy()
                    self.change("approach")
            elif phase == "approach":
                self.pick = obj.copy()
                target = self.start + (obj + [0, -0.042, 0.095] - self.start) * blend(
                    age, 52
                )
                if (age >= 65 and np.linalg.norm(target - hand) < 0.036) or age >= 120:
                    self.start = hand.copy()
                    self.change("align")
            elif phase == "align":
                self.pick = obj.copy()
                target = self.start + (obj + bias - self.start) * blend(age, 32)
                if (age >= 46 and np.linalg.norm(target - hand) < 0.020) or age >= 83:
                    self.change("grasp")
            elif phase == "grasp":
                target = self.pick + bias
                grip = min(1.0, age / 20.0)
                if age >= 36:
                    self.change("lift")
            elif phase == "lift":
                grip = 1.0
                target = self.pick + bias + [0.0, 0.0, 0.107 * blend(age, 32)]
                if age >= 40 and obj[2] > self.pick[2] + 0.056:
                    self.start = hand.copy()
                    self.offset = obj - hand
                    self.change("transfer")
                elif age >= 86:
                    self.change("retry")
            elif phase == "retry":
                target = obj + [0.0, -0.047, 0.095]
                if age >= 42:
                    self.model.integral *= 0.3
                    self.start = hand.copy()
                    self.change("approach")
            elif phase in ("transfer", "lower"):
                self.level = np.clip(
                    self.level + 0.005 * np.cross(self.scene.up, [0.0, 0.0, 1.0]),
                    -0.24,
                    0.24,
                )
                grip = 1.0
                self.offset += np.clip(obj - hand - self.offset, -0.0015, 0.0015)
                target = goal - self.offset
                if phase == "transfer":
                    target[2] += 0.093
                    target = self.start + (target - self.start) * blend(age, 50)
                    if (
                        age >= 58 and np.linalg.norm(obj[:2] - goal[:2]) < 0.019
                    ) or age >= 112:
                        self.low = float(np.clip(obj[2] - goal[2], 0.070, 0.115))
                        self.change("lower")
                else:
                    target[:2] += 0.55 * np.clip(goal[:2] - obj[:2], -0.026, 0.026)
                    target[2] += max(-0.017, self.low - age * 0.0016)
                    if (
                        obj[2] < goal[2] + 0.012
                        and np.linalg.norm(obj[:2] - goal[:2]) < 0.025
                    ) or age >= 120:
                        self.open = 0.5 * (target + hand)
                        self.change("release")
            elif phase == "release":
                grip = max(0.0, 1 - age / 18.0)
                target = self.open + np.array([0, -0.012, 0.004]) * blend(age, 22)
                if age >= 28:
                    self.open = target.copy()
                    self.change("retreat")
            else:
                target = self.open + np.array([-0.052, -0.025, 0.048]) * blend(age, 44)
            cmd = self.model.command(
                pose,
                target,
                wrist_quat=Rotation.from_rotvec(self.level).as_quat()[[3, 0, 1, 2]],
            )
            rows.append(dict(codes=cmd["codes"], hand=[0.0, float(grip)]))
            self.tick += 1
        return rows

    def snapshot(self):
        planning = {
            k: v for k, v in self.__dict__.items() if k not in ("model", "scene")
        }
        return (
            deepcopy(planning),
            deepcopy(self.scene.__dict__),
            self.model.integral.copy(),
            self.model.smooth_codes.copy(),
        )

    def restore(self, state):
        planning, vision, integral, smooth = state
        for k in tuple(self.__dict__):
            if k not in ("model", "scene"):
                del self.__dict__[k]
        self.__dict__.update(deepcopy(planning))
        self.scene.__dict__ = deepcopy(vision)
        self.model.integral[:] = integral
        self.model.smooth_codes[:] = smooth
