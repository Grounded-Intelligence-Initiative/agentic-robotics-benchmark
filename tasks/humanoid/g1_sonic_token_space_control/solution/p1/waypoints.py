"""Task-space stages and feedback for a right-handed tabletop pick and place."""
import numpy as np


class PickPlaceWaypoints:
    """State-feedback planner emitting Cartesian targets at 50 Hz."""

    def __init__(self, *, settle_ticks=30, approach_ticks=0, placement_offset=None):
        self.settle_ticks = settle_ticks
        self.approach_ticks = approach_ticks
        self.placement_offset = placement_offset
        self.approach_start = None
        self.integral = np.zeros(3)
        self.phase = "settle"
        self.tick = 0
        self.entered = 0
        self.attempts = 1
        self.events = []
        self.pick_pos = None

    def _change(self, phase):
        self.events.append((self.tick, phase))
        self.phase = phase
        if phase == "approach":
            self.approach_start = None
        self.entered = self.tick

    def waypoint(self, observation, *, hand_pos=None, apply_integral=False):
        """Return the world grasp target (or None during settle) and Dex3 closures.

        Object, pad, and hand centers use meters. The observed hand point should
        correspond to the calibrated right grasp pocket on the public Dex3.
        """
        obj = np.array(observation["object_pos"], dtype=float)
        goal = np.array(observation["goal_pos"], dtype=float)
        if self.pick_pos is None:
            self.pick_pos = obj.copy()
        if hand_pos is None:
            hand_pos = np.asarray(observation["right_hand_pos"], dtype=float)
        age = self.tick - self.entered
        phase = self.phase
        closed = 0.0
        target = self.pick_pos.copy()
        if phase == "settle":
            if age >= self.settle_ticks:
                self._change("retract")
        elif phase == "retract":
            target = np.array([0.072, -0.23, 0.76])
            if age >= 40:
                self._change("raise")
        elif phase == "raise":
            target = np.array([0.090, -0.26, 0.875])
            if (age >= 50 and hand_pos[2] >= 0.85) or age >= 105:
                self._change("approach")
        elif phase == "approach":
            # Outside the object's fingers-first side, above the work surface.
            self.pick_pos = obj.copy()
            target = obj + np.array([0.0, -0.045, 0.085])
            if self.approach_ticks:
                if self.approach_start is None:
                    self.approach_start = hand_pos.copy()
                t = np.clip(age / self.approach_ticks, 0., 1.)
                blend = t * t * (3. - 2. * t)
                target = self.approach_start * (1. - blend) + target * blend
            if (age >= 65 and np.linalg.norm(target - hand_pos) < 0.035) or age >= 160:
                self._change("align")
        elif phase == "align":
            self.pick_pos = obj.copy()
            target = self.pick_pos.copy()
            if (age >= 40 and np.linalg.norm(target - hand_pos) < 0.016) or age >= 145:
                self._change("grasp")
        elif phase == "grasp":
            closed = min(1, age / 18)
            if age >= 36:
                self._change("lift")
        elif phase == "lift":
            closed = 1.0
            target[2] += 0.105
            if obj[2] - self.pick_pos[2] > 0.065 and age >= 40:
                self.carried_offset = obj - hand_pos
                self._change("transfer")
            elif age >= 135:
                self._change("retry" if self.attempts < 3 else "failed")
        elif phase in ("transfer", "lower"):
            closed = 1.0
            # Regulate the object's center even if it shifts within the hand.
            self.carried_offset += np.clip(obj - hand_pos - self.carried_offset, -0.0006, 0.0006)
            if self.placement_offset is not None:
                goal[:2] += self.placement_offset
            target = goal - self.carried_offset
            if phase == "transfer":
                target[2] += 0.090
                if (np.linalg.norm(obj[:2] - goal[:2]) < 0.021 and age >= 45) or age >= 175:
                    self._change("lower")
            else:
                target[2] += max(0.0, 0.090 - age * 0.0015)
                target[:2] += 0.65 * np.clip(goal[:2] - obj[:2], -0.03, 0.03)
                aligned = np.linalg.norm(obj[:2] - goal[:2]) < 0.015
                if (age >= 60 and aligned and obj[2] < goal[2] + 0.011) or age >= 145:
                    self.release_at = hand_pos.copy()
                    self._change("release")
        elif phase == "release":
            closed = max(0, 1 - age / 18)
            target = self.release_at.copy()
            if age >= 28:
                self._change("retreat")
        elif phase in ("retreat", "done"):
            target = self.release_at + np.array([0.0, -0.075, 0.065])
            if phase == "retreat" and age >= 60:
                self._change("done")
        elif phase in ("retry", "failed"):
            target = obj + np.array([0.0, -0.060, 0.095])
            if phase == "retry" and age >= 40:
                self.attempts += 1
                self.integral[:] = 0.
                self._change("approach")

        if phase != "settle" and apply_integral:
            # The encoder/decoder and PD lag vary with pose. Bound the Cartesian
            # integral and joint speed to keep the balance controller engaged.
            error = target - hand_pos
            self.integral += np.clip(error, -0.05, 0.05) * 0.008
            self.integral = np.clip(self.integral, -0.052, 0.052)
            target = target + self.integral
        if phase == "settle":
            target = None
        self.tick += 1
        return target, (0.0, float(closed))
