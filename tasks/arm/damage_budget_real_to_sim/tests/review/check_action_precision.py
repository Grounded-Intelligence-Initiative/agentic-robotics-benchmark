"""Focused arm wire-contract checks; run with the task image's Python."""
import math
from pathlib import Path
import sys
import unittest

TASK = Path(__file__).resolve().parents[2]
PAYLOAD = TASK/"environment/setup/payload"
sys.path[:0] = [str(TASK/"tests"), str(PAYLOAD)]

import numpy as np
from arm_dynamics import ACTION_EPS, bounded_action
from model_rollout import rollout
from world import LIMITS, World


class ActionPrecision(unittest.TestCase):
    def test_packaged_copies(self):
        self.assertEqual((TASK/"tests/world.py").read_bytes(),
                         (TASK/"environment/setup/runtime/world.py").read_bytes())
        for name in ("arm_dynamics.py", "model_rollout.py"):
            self.assertEqual((TASK/"tests"/name).read_bytes(), (PAYLOAD/name).read_bytes())

    def test_boundary(self):
        self.assertEqual(ACTION_EPS, 1e-12)
        last = 1 + ACTION_EPS
        if last - 1 > ACTION_EPS:
            last = math.nextafter(last, 1.)
        for sign in (-1, 1):
            for near in (1.0000000000000009, last):
                raw = [sign*near, -0., .73, -sign*near]
                self.assertEqual(bounded_action(raw), [sign, -0., .73, -sign])
                self.assertEqual(raw[0], sign*near)
            with self.assertRaises(ValueError):
                bounded_action([0, 0, 0, sign*math.nextafter(last, math.inf)])
        legal = [-1, math.nextafter(-1., 0.), math.nextafter(1., 0.), 1]
        self.assertEqual(bounded_action(legal), legal)

    def test_no_change_to_valid_physics_or_budget(self):
        actual, exact = [World("arm", 101, render=False) for _ in range(2)]
        self.addCleanup(actual.close)
        self.addCleanup(exact.close)
        raw, canonical = [1.0000000000000009, -1.0000000000000009, .5, -0.], [1., -1., .5, -0.]
        np.testing.assert_array_equal(rollout(actual.model, [raw]), rollout(exact.model, [canonical]))
        actual.steps = exact.steps = LIMITS["arm"] - 1
        actual.act(raw)
        exact.act(canonical)
        np.testing.assert_array_equal(actual.data.qpos, exact.data.qpos)
        np.testing.assert_array_equal(actual.data.qvel, exact.data.qvel)
        np.testing.assert_array_equal(actual.arm.command, exact.arm.command)
        self.assertEqual(actual.metrics(), exact.metrics())
        self.assertTrue(actual.done)
        self.assertEqual(actual.steps, LIMITS["arm"])
        with self.assertRaisesRegex(ValueError, "episode finished"):
            actual.act(raw)

    def test_rejected_before_physics(self):
        w = World("arm", 101, render=False)
        self.addCleanup(w.close)
        qpos, cmd = w.data.qpos.copy(), w.arm.command.copy()
        for raw in ([], [0]*3, [0]*5, (0,)*4, [[0], 0, 0, 0], {"action": [0]*4},
                    [0, 0, 0, True], [False, 0, 0, 0], ["0", 0, 0, 0], [0, None, 0, 0],
                    [float("nan"), 0, 0, 0], [0, float("inf"), 0, 0], [0, 0, -math.inf, 0],
                    [1.000001, 0, 0, 0], [0, 0, 0, -1.000001], [10**400, 0, 0, 0]):
            with self.subTest(action=raw):
                with self.assertRaises(ValueError):
                    w.act(raw)
                with self.assertRaises(ValueError):
                    rollout(w.model, [raw])
                self.assertEqual(w.steps, 0)
                self.assertEqual(w.data.time, 0.)
                np.testing.assert_array_equal(w.data.qpos, qpos)
                np.testing.assert_array_equal(w.arm.command, cmd)


if __name__ == "__main__":
    unittest.main()
