"""Run with the task's dependencies/assets; no paid model is required."""
from pathlib import Path
import sys
import unittest

import mujoco
import numpy as np

TASK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TASK/"environment/setup/runtime"))
from long_bridge import ACTION_LIMIT, BridgeWorld
from service import Runtime


class Contract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = BridgeWorld(101, render=False)

    def setUp(self):
        w = self.world
        w.steps = w.frame_id = w.phase = 0
        w.reports, w.last_frames = [], {}
        w.done = w.success = False
        w.observed_components.clear()
        w.data.qpos[:2], w.data.qvel[:] = [-2, -5.5], 0
        mujoco.mj_forward(w.model, w.data)

    def photograph(self, defect):
        w = self.world
        w.data.qpos[:2] = w.component_point(*defect)
        mujoco.mj_forward(w.model, w.data)
        return w.observe()["frame_id"]

    def test_three_spread_defects_and_large_extent(self):
        w = self.world
        self.assertIn(w.length, (72, 96, 120))
        self.assertEqual(len(set(w.defects)), 3)
        self.assertGreater(w.defects[-1][0]-w.defects[0][0], w.spans/3)

    def test_observation_does_not_leak_state_or_answers(self):
        obs = self.world.observe()
        self.assertEqual(set(obs), {"images", "frame_id", "steps_remaining", "phase", "done", "reports_submitted"})

    def test_old_frame_evidence_and_finalization(self):
        w = self.world
        evidence = [(self.photograph(d), w.component_point(*d).tolist()) for d in w.defects]
        for frame, point in evidence:
            w.report(frame, point)
            self.assertEqual(w.metrics()["inspection_quality"], 0)
        w.advance()
        self.assertEqual(w.metrics()["inspection_quality"], 1)

    def test_blank_guesses_duplicates_and_wrong_coordinates(self):
        w = self.world
        frame = self.photograph(w.defects[0])
        point = w.component_point(*w.defects[0]).tolist()
        w.report(987654, point)
        w.report(frame, [point[0], -point[1]])
        w.report(frame, point)
        w.report(frame, point)
        w.advance()
        self.assertEqual(w.metrics()["true_reports"], 1)
        self.assertEqual(w.metrics()["false_reports"], 3)
        self.assertAlmostEqual(w.metrics()["inspection_quality"], 2/7)

    def test_remote_frame_cannot_prove_defect(self):
        w = self.world
        frame = w.observe()["frame_id"]
        w.report(frame, w.component_point(*w.defects[0]).tolist())
        w.advance()
        self.assertEqual(w.metrics()["inspection_quality"], 0)

    def test_visual_occlusion_invalidates_evidence(self):
        w = self.world
        defect = w.defects[0]
        self.photograph(defect)
        self.assertIn(defect, w._visible_defects())
        roof = w.model.geom("road_center_1").id
        pos = w.model.geom_pos[roof].copy()
        size = w.model.geom_size[roof].copy()
        w.model.geom_pos[roof] = [*w.component_point(*defect), 9.]
        w.model.geom_size[roof] = [1., 1., .1]
        try:
            mujoco.mj_forward(w.model, w.data)
            self.assertNotIn(defect, w._visible_defects())
        finally:
            w.model.geom_pos[roof] = pos
            w.model.geom_size[roof] = size
            mujoco.mj_forward(w.model, w.data)

    def test_budget_exhaustion_keeps_final_reporting_available(self):
        w = self.world
        frame = self.photograph(w.defects[0])
        w.steps = ACTION_LIMIT
        with self.assertRaises(ValueError):
            w.act([0, 0])
        w.report(frame, w.component_point(*w.defects[0]).tolist())
        w.advance()
        self.assertEqual(w.metrics()["inspection_quality"], .5)
        with self.assertRaises(ValueError):
            w.report(frame, [0, 0])

    def test_reset_is_not_an_operation(self):
        runtime = Runtime.__new__(Runtime)
        runtime.world = self.world
        with self.assertRaises(ValueError):
            runtime.dispatch("reset", {})

    def test_invalid_actions_do_not_consume_movement(self):
        for action in ([True, 0], [float("nan"), 0], [0], [2, 0]):
            with self.assertRaises(ValueError):
                self.world.act(action)
        self.assertEqual(self.world.steps, 0)

    def test_no_correctness_feedback(self):
        w = self.world
        frame = self.photograph(w.defects[0])
        right = w.report(frame, w.component_point(*w.defects[0]).tolist())
        wrong = w.report(frame, [-999, 999])
        self.assertEqual(set(right), set(wrong))
        self.assertEqual(right["phase"], wrong["phase"])
        self.assertFalse(right["done"] or wrong["done"])


if __name__ == "__main__":
    unittest.main()
