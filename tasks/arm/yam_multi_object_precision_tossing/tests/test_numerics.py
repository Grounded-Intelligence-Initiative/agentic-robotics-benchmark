"""Numerical probe lifecycle and stationary-world regression checks."""
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numerics_check


class NumericalProbeTests(unittest.TestCase):
    def setUp(self):
        self.arena = SimpleNamespace(
            arms={"left": {}, "right": {}},
            model=object(), data=SimpleNamespace(time=0., qpos=np.zeros(2)),
            drive=Mock(), close=Mock(),
        )

    def test_stationary_world_probe_finishes_and_closes(self):
        def step(model, data):
            data.time += .002
            data.qpos += .001

        with patch("workcell.ArenaRound", return_value=self.arena), \
             patch("workcell.check_physics") as check, \
             patch("mujoco.mj_step", side_effect=step) as physics:
            actual = numerics_check.fingerprint()
        self.assertEqual(set(actual), {"physics", "controller"})
        self.assertTrue(all(len(value) == 64 for value in actual.values()))
        self.assertEqual(physics.call_count, 160)
        self.assertEqual(check.call_count, 160)
        self.assertEqual(self.arena.drive.call_count, 320)
        self.arena.close.assert_called_once_with()

    def test_physics_failure_closes_arena(self):
        with patch("workcell.ArenaRound", return_value=self.arena), \
             patch("workcell.check_physics", side_effect=RuntimeError("invalid physics")), \
             patch("mujoco.mj_step"):
            with self.assertRaisesRegex(RuntimeError, "invalid physics"):
                numerics_check.fingerprint()
        self.arena.close.assert_called_once_with()

    def test_verifier_rejects_changed_fingerprint(self):
        changed = {"physics": "changed", "controller": "changed"}
        with patch("subprocess.check_output", return_value=json.dumps(changed).encode()):
            with self.assertRaisesRegex(RuntimeError, "YAM reproducibility check failed"):
                numerics_check.verify_numerics()


if __name__ == "__main__":
    unittest.main()
