"""Bounded local inverse search on 64 discrete SONIC coordinates.

Subclasses supply a batch of decoder predictions at an unchanged control tick.
"""

import numpy as np

from sonic_sim.decoder import DEFAULT_Q, KP
from .stand import STAND_CODES


class InverseCodeSearch:
    def __init__(self, goal, arm_bias, *, refresh=4, ridge=0.00022, posture_drift=0.00001):
        if type(refresh) is not int or not 1 <= refresh <= 64:
            raise ValueError("refresh must be an integer between 1 and 64")
        if (
            not np.isfinite(ridge)
            or ridge <= 0
            or not np.isfinite(posture_drift)
            or posture_drift < 0
        ):
            raise ValueError("ridge must be positive and posture_drift nonnegative")
        self.goal = goal
        self.arm_bias = arm_bias
        self.smoothed_point = None
        self.code = STAND_CODES.copy()
        self.jac = None
        self.rotate = 0
        self.refresh = refresh
        self.ridge = ridge
        self.posture_drift = posture_drift
        self.weight = np.r_[np.full(22, 0.9), [1.6, 1.6, 1.4, 1.5, 0.9, 0.7, 0.6]]
        self.calls = 0
        self.last_diagnostics = {}

    def _probe(self, obs, codes):
        raise NotImplementedError

    def _predict(self, obs, codes):
        predictions = np.asarray(self._probe(obs, codes))
        if predictions.shape != (len(codes), 29) or not np.isfinite(predictions).all():
            raise ValueError("Invalid decoder predictions")
        self.calls += len(codes)
        return predictions

    def _goal(self, obs, point, standing):
        # Limit waypoint speed during contact acquisition and transport.
        aim = point
        if self.smoothed_point is None:
            self.smoothed_point = np.asarray(obs["right_hand_pos"]).copy()
        offset = aim - self.smoothed_point
        self.smoothed_point += offset * min(1.0, 0.004 / max(np.linalg.norm(offset), 1e-8))
        arm = self.goal._arm_ik(self.smoothed_point, self.goal.orientation, robot_observation=obs)
        self.goal.reference[22:] += np.clip(arm - self.goal.reference[22:], -0.020, 0.020)
        target = standing.copy()
        # The inverse target includes gravity and a damping term. This only
        # defines a counterfactual cost; only the selected code reaches physics.
        bias = self.arm_bias() / KP[22:]
        q = obs["q"][22:]
        target[22:] = q + (self.goal.reference[22:] - q) - 0.034 * obs["dq"][22:] + bias
        return target

    def act(self, obs):
        current_phase = self.goal.phase
        point, hand = self.goal.waypoint(obs, apply_integral=True)
        if point is not None:
            # Retain 30% of the phase planner's integrated position correction.
            point = point - 0.7 * self.goal.integral
        return self.command(obs, point, phase=current_phase), hand

    def command(self, obs, wrist_goal, *, phase=None):
        """Predict 64 integer codes for a world-frame right-grasp-point goal."""
        self.last_diagnostics = {"phase": phase, "wrist_goal": wrist_goal}
        if wrist_goal is None:
            self.code = STAND_CODES.copy()
            self.smoothed_point = None
            self.goal.reference[:] = DEFAULT_Q
            self.jac = None
            self.rotate = 0
            self._standing(obs)
            return {"mode": "codes", "codes": self.code.tolist()}
        point = np.asarray(wrist_goal, dtype=float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError("wrist_goal must contain three finite world coordinates")
        start_calls = self.calls
        if self.jac is None:
            self.jac = np.empty((29, 64))
            dims = np.arange(64)
        else:
            dims = np.arange(self.rotate, self.rotate + self.refresh) % 64
            self.rotate = (self.rotate + self.refresh) % 64
        separate_stand = not np.array_equal(self.code, STAND_CODES)
        batch = [self.code, STAND_CODES] if separate_stand else [self.code]
        steps = np.where(self.code[dims] >= 31, -1, 1)
        neighbors = np.tile(self.code, (len(dims), 1))
        neighbors[np.arange(len(dims)), dims] += steps
        values = self._predict(obs, batch + list(neighbors))
        base, stand = values[0], values[int(separate_stand)]
        self.jac[:, dims] = ((values[len(batch):] - base) / steps[:, None]).T
        target = self._goal(obs, point, stand)
        diff = self.weight * (base - target)
        jac = self.weight[:, None] * self.jac
        # Ridge step in 64 integer coordinates. Solve the smaller output-space system.
        alpha = self.ridge + self.posture_drift
        prior = (STAND_CODES - self.code) * (self.posture_drift / alpha)
        delta = prior - jac.T @ np.linalg.solve(
            jac @ jac.T + alpha * np.eye(29), diff + jac @ prior
        )
        lo = np.maximum(-2, -self.code)
        hi = np.minimum(2, 31 - self.code)

        def cost(pred, change):
            e = self.weight * (pred - target)
            return (
                e @ e
                + self.ridge * (change @ change)
                + self.posture_drift * np.sum((self.code + change - STAND_CODES) ** 2)
            )

        initial_cost = best_cost = cost(base, np.zeros(64))
        best = self.code
        # Quantized, coupled updates can overshoot the local model. The released
        # decoder ranks the unchanged token and two candidate steps on this exact history.
        candidates = []
        step = np.clip(np.rint(delta), lo, hi)
        candidates.append(step.copy())
        remainder = diff + jac @ step
        denom = np.einsum("ij,ij->j", jac, jac) + alpha
        for j in np.argsort(-np.abs(jac.T @ remainder)):
            grad = jac[:, j] @ remainder + alpha * (step[j] - prior[j])
            add = np.clip(np.rint(-grad / denom[j]), lo[j] - step[j], hi[j] - step[j])
            step[j] += add
            remainder += add * jac[:, j]
        candidates.append(step)
        changes = [change for change in candidates if np.any(change)]
        proposals = [(self.code + change).astype(int) for change in changes]
        predictions = self._predict(obs, proposals) if proposals else []
        for change, proposed, prediction in zip(changes, proposals, predictions):
            value = cost(prediction, change)
            if value < best_cost:
                best, best_cost = proposed, value
        self.last_diagnostics.update(
            proxy_before=float(initial_cost),
            proxy_after=float(best_cost),
            search_forwards=self.calls - start_calls,
            changed_coordinates=int(np.count_nonzero(best - self.code)),
        )
        self.code = best
        return {"mode": "codes", "codes": self.code.tolist()}

    def _standing(self, obs):
        """External clients may request a prediction for the selected standing code."""
