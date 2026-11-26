"""Version 1. Visually gated release near the pad, then clear the object vertically."""

import numpy as np

from .kinematics import PickPlaceAgent


class LandingPlan(PickPlaceAgent):
    def __init__(self, robot):
        super().__init__(robot, approach_ticks=55)
        self.low_aligned_ticks = 0

    def waypoint(self, observation, *, hand_pos=None, apply_integral=False):
        phase = self.phase
        if phase not in {"lower", "release", "retreat", "done"}:
            return super().waypoint(observation, hand_pos=hand_pos, apply_integral=apply_integral)
        obj, goal = observation["object_pos"], observation["goal_pos"]
        hand = observation["right_hand_pos"] if hand_pos is None else hand_pos
        age = self.tick - self.entered
        closure = 0.
        if phase == "lower":
            closure = 1.
            self.carried_offset += np.clip(obj - hand - self.carried_offset, -.0006, .0006)
            target = goal - self.carried_offset
            target[2] += max(0., .090 - age * .0015)
            target[:2] += .65 * np.clip(goal[:2] - obj[:2], -.030, .030)
            # At a safe inset of the 138 mm pad, waiting for an exact center can
            # carry a supported object away before the fingers start opening.
            inset = np.max(np.abs(obj[:2] - goal[:2])) < .028
            low = -.008 <= obj[2] - goal[2] <= .012
            self.low_aligned_ticks = self.low_aligned_ticks + 1 if low and inset else 0
            if age >= 35 and self.low_aligned_ticks >= 6:
                self.release_at = hand.copy()
                self._change("release")
        elif phase == "release":
            closure = max(0., 1 - age / 8)
            target = self.release_at.copy()
            if age >= 16:
                self._change("retreat")
        else:
            target = self.release_at.copy()
            target[2] += np.clip(age / 25, 0., 1.) * .060 if phase == "retreat" else .060
            target[1] -= np.clip((age - 20) / 35, 0., 1.) * .070 if phase == "retreat" else .070
            if phase == "retreat" and age >= 60:
                self._change("done")
        if apply_integral:
            self.integral = np.clip(self.integral + .008 * np.clip(target - hand, -.05, .05), -.052, .052)
            target += self.integral
        self.tick += 1
        return target, (0., float(closure))
