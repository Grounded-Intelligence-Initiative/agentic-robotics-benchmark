"""Deterministic dynamics/contracts tests. Run in the task image on remote CPU."""
from dataclasses import replace
import filecmp
import json
import math
from pathlib import Path
import sys
import unittest

TASK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TASK/"tests"))
import mujoco
import numpy as np
from service import dispatch
from world import (ANGLE_RELEASE, ANGLE_TOLERANCE, DOCK_FRAMES, GOAL_RELEASE, GOAL_TOLERANCE,
                   ACTION_LIMIT, FRAME_LIMIT, REQUEST_LIMIT, VERSION,
                   LOAD_AFTER, STOP_SPEED, STOP_YAW, Scene, World)


class DynamicsTests(unittest.TestCase):
    def setUp(self):
        self.w = World(3001, render=False)

    def tearDown(self):
        self.w.close()

    def act(self, name, frames):
        return self.w.act([self.w.scene.mapping.index(name)/3], frames=frames)

    def pose(self, xy=(0., 0.), yaw=0.):
        self.w.data.qpos[:] = [*xy, yaw]
        self.w.data.qvel[:] = 0
        self.w.response.drive = self.w.response.turn = 0.
        self.w.response.terrain = self.w.scene.terrain_at(xy)
        mujoco.mj_forward(self.w.model, self.w.data)

    def dock_pose(self):
        self.pose(self.w.goal(), self.w.scene.goal_headings[self.w.current])

    def test_fixed_mapping_and_copy_parity(self):
        self.assertEqual(self.w.scene.mapping, Scene(3001).mapping)
        for name in ("world.py", "service.py", "geometry.py", "scene_assets.py", "pbr_scene.py", "pbr_renderer.py"):
            self.assertTrue(filecmp.cmp(TASK/"tests"/name, TASK/"environment/setup/runtime"/name, shallow=False), name)
            self.assertTrue(filecmp.cmp(TASK/"tests"/name, TASK/"solution/private"/name, shallow=False), name)
        self.assertTrue(filecmp.cmp(TASK/"tests/hidden_scenes.json", TASK/"environment/setup/scenes.json", shallow=False))

    def test_invalid_requests_do_not_mutate_physics(self):
        fingerprint = self.w.state_fingerprint()
        for params in ({"action": [0], "frames": x} for x in (0, 121, -1, True, 1.5, "2", None)):
            with self.assertRaises(ValueError):
                dispatch(self.w, "act", params)
        for params in ({"action": [x]} for x in (True, .5, float("nan"), float("inf"))):
            with self.assertRaises(ValueError):
                dispatch(self.w, "act", params)
        for method, params in (("truth", {}), ("reset", {}), ("observe", {"camera": "top"}),
                               ("unlock", {"token": "anything"}), ("unlock", {"token_sha256": "anything"}),
                               ("act", {"action": [0], "seed": 1})):
            with self.assertRaises(ValueError):
                dispatch(self.w, method, params)
        self.assertEqual(fingerprint, self.w.state_fingerprint())

    def test_onboard_only_and_no_truth_response(self):
        self.assertEqual(self.w.model.ncam, 1)
        self.assertEqual(self.w.model.camera(0).name, "onboard")
        self.assertEqual(int(self.w.model.cam_bodyid[0]), self.w.agent_id)
        self.assertEqual(self.w.model.cam_projection[0], mujoco.mjtProjection.mjPROJ_PERSPECTIVE)
        self.assertEqual(float(self.w.model.cam_fovy[0]), 100.)
        self.assertAlmostEqual(float(self.w.model.vis.map.znear), .0006, places=8)
        keys = {"frame_id", "delivery", "deliveries", "delivered", "goal_reached", "loaded",
                "steps_remaining", "frames_remaining", "done"}
        self.assertEqual(set(self.w.observe()), keys)
        spec = mujoco.mjtState.mjSTATE_INTEGRATION
        state = np.empty(mujoco.mj_stateSize(self.w.model, spec))
        mujoco.mj_getState(self.w.model, self.w.data, state, spec)
        response = replace(self.w.response)
        docking = (self.w.dock_armed, self.w.dock_frames)
        before = self.w.state_fingerprint()
        for _ in range(10):
            self.w.observe()
        after = np.empty_like(state)
        mujoco.mj_getState(self.w.model, self.w.data, after, spec)
        np.testing.assert_array_equal(state, after)
        self.assertEqual(response, self.w.response)
        self.assertEqual(docking, (self.w.dock_armed, self.w.dock_frames))
        self.assertNotEqual(before, self.w.state_fingerprint(), "camera accounting must be attested")

    def test_separate_camera_budget_and_status_only_actions(self):
        self.assertEqual((VERSION, ACTION_LIMIT, FRAME_LIMIT, REQUEST_LIMIT), (4, 500, 120, 660))
        # A sentinel renderer makes an accidental automatic image fail even headlessly.
        self.w.render_enabled = True
        self.w.image = lambda: self.fail("act/advance must not render")
        self.assertNotIn("images", self.act("brake", 1))
        self.w.delivered = 1
        self.assertNotIn("images", self.w.advance())
        self.assertEqual(self.w.frame_id, 0)
        self.w.render_enabled = False
        for n in range(FRAME_LIMIT):
            self.assertEqual(self.w.observe()["frames_remaining"], FRAME_LIMIT-n-1)
        fingerprint = self.w.state_fingerprint()
        with self.assertRaisesRegex(ValueError, "camera frame budget exhausted"):
            self.w.observe()
        self.assertEqual(self.w.state_fingerprint(), fingerprint)
        self.assertEqual(self.act("brake", 1)["frames_remaining"], 0)
        self.assertEqual(self.w.metrics()["frames"], FRAME_LIMIT)

    def test_force_integration_and_history(self):
        self.pose()
        self.act("forward", 1)
        self.assertGreater(self.w.data.qvel[0], 0)
        self.assertLess(self.w.data.qvel[0], .01)
        self.act("forward", 40)
        speed = self.w.data.qvel[0]
        self.act("turn_left", 1)
        self.assertGreater(self.w.data.qvel[0], .9*speed)
        self.assertGreater(self.w.response.drive, 0)
        self.act("turn_left", 30)
        self.act("turn_right", 1)
        self.assertGreater(self.w.data.qvel[2], 0, "opposite steering cannot erase yaw momentum in one tick")

    def test_body_frame_nonholonomic_motion(self):
        self.pose(yaw=math.pi/2)
        self.act("forward", 40)
        self.assertGreater(self.w.data.qpos[1], .05)
        self.assertLess(abs(self.w.data.qpos[0]), 1e-9)

    def test_terrain_hysteresis_and_different_traction(self):
        centre, half, kind = self.w.scene.bands[0]
        edge = centre+half
        self.assertEqual(self.w.scene.terrain_at((0, edge), kind), kind)
        self.assertEqual(self.w.scene.terrain_at((0, edge), 0), 0)
        self.assertEqual(self.w.scene.terrain_at((0, edge+.05), kind), 0)
        data = mujoco.MjData(self.w.model)
        self.pose((0, centre))
        mujoco.mj_copyData(data, self.w.model, self.w.data)
        dry = replace(self.w.response, terrain=0)
        self.act("forward", 70)
        wet_distance = self.w.data.qpos[0]
        bands = self.w.scene.bands
        self.w.scene.bands = []
        try:
            for _ in range(70):
                self.w.integrate(data, dry, self.w.scene.mapping.index("forward"))
        finally:
            self.w.scene.bands = bands
        self.assertGreater(data.qpos[0], wet_distance*1.2)

    def test_precision_pose_speed_and_continuous_dwell(self):
        self.dock_pose()
        self.w.data.qpos[2] += ANGLE_RELEASE+.01
        self.act("brake", 120)
        self.assertEqual(self.w.delivered, 0)
        self.dock_pose()
        self.w.data.qvel[0] = STOP_SPEED+.01
        self.w.update_docking()
        self.assertEqual(self.w.dock_frames, 0)
        self.w.data.qvel[:] = 0
        self.w.data.qvel[2] = STOP_YAW+.01
        self.w.update_docking()
        self.assertEqual(self.w.dock_frames, 0)
        self.w.data.qvel[:] = 0
        self.act("brake", DOCK_FRAMES-1)
        self.assertEqual(self.w.delivered, 0)
        self.w.data.qvel[2] = STOP_YAW+.01
        self.w.update_docking()
        self.w.data.qvel[:] = 0
        self.assertEqual(self.w.dock_frames, 0)
        self.act("brake", DOCK_FRAMES)
        self.assertEqual(self.w.delivered, 1)
        self.act("brake", 120)
        self.assertEqual(self.w.delivered, 1, "cannot farm a latched delivery")

    def test_docking_schmitt_gate(self):
        self.dock_pose()
        self.w.data.qpos[0] += (GOAL_TOLERANCE+GOAL_RELEASE)/2
        self.w.update_docking()
        self.assertFalse(self.w.dock_armed)
        self.dock_pose()
        self.w.update_docking()
        self.assertTrue(self.w.dock_armed)
        self.w.data.qpos[0] += (GOAL_TOLERANCE+GOAL_RELEASE)/2
        self.w.data.qpos[2] += (ANGLE_TOLERANCE+ANGLE_RELEASE)/2
        self.w.update_docking()
        self.assertTrue(self.w.dock_armed)
        self.w.data.qpos[0] += .1
        self.w.update_docking()
        self.assertFalse(self.w.dock_armed)
        self.assertEqual(self.w.dock_frames, 0)

    def test_advance_does_not_teleport_or_reset_and_load_changes_inertia(self):
        with self.assertRaises(ValueError):
            self.w.advance()
        self.w.current, self.w.delivered = LOAD_AFTER-1, LOAD_AFTER
        self.w.data.qvel[:] = [.1, -.02, .03]
        before = self.w.data.qpos.copy(), self.w.data.qvel.copy()
        spec = mujoco.mjtState.mjSTATE_INTEGRATION
        state_before = np.empty(mujoco.mj_stateSize(self.w.model, spec))
        mujoco.mj_getState(self.w.model, self.w.data, state_before, spec)
        mass = self.w.model.body_mass[self.w.agent_id]
        inertia = self.w.model.body_inertia[self.w.agent_id].copy()
        mapping = list(self.w.scene.mapping)
        response = replace(self.w.response)
        self.w.advance()
        np.testing.assert_array_equal(self.w.data.qpos, before[0])
        np.testing.assert_array_equal(self.w.data.qvel, before[1])
        self.assertEqual(self.w.response, response)
        state_after = np.empty_like(state_before)
        mujoco.mj_getState(self.w.model, self.w.data, state_after, spec)
        np.testing.assert_array_equal(state_after, state_before)
        self.assertEqual(self.w.scene.mapping, mapping)
        self.assertGreater(self.w.model.body_mass[self.w.agent_id], mass)
        self.assertTrue(np.all(self.w.model.body_inertia[self.w.agent_id] > inertia))
        self.assertTrue(self.w.loaded)

    def test_fingerprint_covers_hidden_memory(self):
        for obj, field, value in ((self.w, "dock_armed", True), (self.w, "dock_frames", 1),
                                  (self.w.response, "drive", .1), (self.w.response, "turn", .1),
                                  (self.w.response, "terrain", 1), (self.w, "loaded", True),
                                  (self.w, "frame_id", 1)):
            old = getattr(obj, field)
            before = self.w.state_fingerprint()
            setattr(obj, field, value)
            self.assertNotEqual(before, self.w.state_fingerprint(), field)
            setattr(obj, field, old)
        before = self.w.state_fingerprint()
        self.w.model.body_mass[self.w.agent_id] += .1
        self.assertNotEqual(before, self.w.state_fingerprint())

    def test_load_is_not_a_constant_displacement_scale(self):
        self.pose()
        loaded = World(3001, render=False)
        try:
            loaded.current, loaded.delivered = LOAD_AFTER-1, LOAD_AFTER
            loaded.advance()
            loaded.data.qpos[:] = self.w.data.qpos
            loaded.data.qvel[:] = 0
            loaded.response.terrain = self.w.response.terrain
            mujoco.mj_forward(loaded.model, loaded.data)
            button = self.w.scene.mapping.index("forward")/3
            self.w.act([button], frames=20)
            loaded.act([button], frames=20)
            early = loaded.data.qvel[0]/self.w.data.qvel[0]
            self.w.act([button], frames=80)
            loaded.act([button], frames=80)
            late = loaded.data.qvel[0]/self.w.data.qvel[0]
            self.assertGreater(late-early, .02)
            self.assertLess(early, self.w.scene.load)
        finally:
            loaded.close()

    def test_partitioned_presses_same_dynamics(self):
        other = World(3001, render=False)
        try:
            b = self.w.scene.mapping.index("forward")/3
            self.w.act([b], frames=120)
            for _ in range(4):
                other.act([b], frames=30)
            np.testing.assert_array_equal(self.w.data.qpos, other.data.qpos)
            np.testing.assert_array_equal(self.w.data.qvel, other.data.qvel)
            self.assertEqual(self.w.response, other.response)
            self.assertEqual(self.w.physics_frames, other.physics_frames)
        finally:
            other.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
