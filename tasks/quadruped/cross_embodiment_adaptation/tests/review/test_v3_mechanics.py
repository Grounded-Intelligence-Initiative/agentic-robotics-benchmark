"""Fast source/mechanics tests, with optional native MuJoCo checks in the image."""
import importlib.util
import math
from pathlib import Path
import sys
import unittest

TASK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TASK/'tests'/'runtime'))
import mechanics as M


class GeometryTests(unittest.TestCase):
    def test_orientation_changes_fit(self):
        walls = [(0, .65, 3., .12), (0, -.65, 3., .12)]
        self.assertTrue(M.free((0, 0, 0), (.88, .34), walls))
        self.assertFalse(M.free((0, 0, math.pi/2), (.88, .34), walls))
        # A scalar circumradius rejects a genuinely usable straight passage.
        radius = math.hypot(.88, .34)
        self.assertFalse(M.free((0, 0, 0), (radius, radius), walls))

    def test_endpoints_do_not_certify_sweep(self):
        obstacle = [(.65, .65, .045, .045)]
        half = (.9, .30)
        self.assertTrue(M.free((0, 0, 0), half, obstacle))
        self.assertTrue(M.free((0, 0, math.pi/2), half, obstacle))
        self.assertFalse(M.free((0, 0, math.pi/4), half, obstacle))

    def test_dogleg_is_not_scout_route_replay(self):
        site = M.Site(1001)
        a, b = site.carrier_half
        self.assertLess(2*b, site.dogleg_width)
        self.assertGreater(2*a, site.dogleg_width)
        # The middle leg admits the aligned body but not its entry orientation:
        # being individually wide enough does not certify the junction's turn.
        w = site.dogleg_width
        walls = [(-w/2-.12, 0, .12, 2), (w/2+.12, 0, .12, 2)]
        self.assertTrue(M.free((0, 0, math.pi/2), (.155, .155), walls))
        self.assertTrue(M.free((0, 0, math.pi/2), (a, b), walls))
        self.assertFalse(M.free((0, 0, 0), (a, b), walls))

    def test_constructive_witness_is_collision_free(self):
        import json
        seeds = json.loads((TASK/'tests/hidden/sites.json').read_text())['seeds']
        for seed in seeds + list(range(1000, 1020)):
            site = M.Site(seed)
            self.assertTrue(all(M.free(p, site.carrier_half, site.obstacles(), .04)
                                for p in site.witness()), seed)

    def test_dock_needs_heading_rest_and_position(self):
        goal = (1, 2, .4)
        good = (1, 2, .4, 0, 0, 0, 0)
        self.assertTrue(M.docked(good, goal))
        for i, value in ((0, 1.2), (2, .7), (3, .1), (5, .2)):
            bad = list(good)
            bad[i] = value
            self.assertFalse(M.docked(bad, goal))

    def test_drive_is_nonholonomic_and_lagged(self):
        params = M.Site(1001).dynamics
        rest = (0, 0, 0, 0, 0, 0, 0)
        turning = M.pulse(rest, (0, 1), params)[-1]
        self.assertEqual(turning[:6], rest[:6])
        moving = M.pulse(rest, (1, 0), params)[-1]
        coast = M.pulse(moving, (0, 0), params)[-1]
        self.assertGreater(coast[0], moving[0])
        self.assertGreater(coast[3], 0)
        self.assertLess(coast[3], moving[3])
        acceleration, _ = M.accelerations(0, (1, .5, 1), .7, (1, 1), params)
        self.assertLessEqual(math.hypot(*acceleration[:2]), params['traction']*9.81+1e-10)

    def test_runtime_mirror(self):
        for source in (TASK/'environment/setup/runtime').glob('*.py'):
            self.assertEqual(source.read_bytes(), (TASK/'tests/runtime'/source.name).read_bytes(), source.name)


@unittest.skipUnless(importlib.util.find_spec('mujoco'), 'native checks require task image')
class NativeTests(unittest.TestCase):
    def carrier(self):
        from world import World
        w = World(1001, render=False)
        w.steps[0] = 12
        w.advance()
        self.addCleanup(w.close)
        return w

    def test_mj_step_matches_free_space_predictor(self):
        w = self.carrier()
        state = w.state()
        command = (.4, .6)
        action = [0., 0.]
        for i in range(2):
            action[w.axes[i]] = command[i]*w.signs[i]
        predicted = M.pulse(state, command, w.site.dynamics)[-1]
        w.act(action)
        for a, b in zip(predicted, w.state()):
            self.assertAlmostEqual(M.wrap(a-b), 0, places=7)
        self.assertFalse(w.contact_stop)

    def test_payload_descendant_contact_stops(self):
        import mujoco
        w = self.carrier()
        # Payload nose reaches the boundary while the small central body is clear.
        w.data.qpos[:3] = (M.FIELD-w.half[0]+.01, 0., 0.)
        w.data.qvel[:] = 0
        mujoco.mj_forward(w.model, w.data)
        self.assertTrue(w._touching())
        self.assertIn(w.model.body('payload').id, w.robot_bodies)
        w.act([0., 0.])
        self.assertTrue(w.contact_stop)
        self.assertFalse(w.success)

    def test_public_observation_has_no_pose(self):
        w = self.carrier()
        self.assertEqual(set(w.observe()), {'frame_id', 'phase', 'done', 'steps_remaining', 'bumper', 'contact_stop'})
        for name in ('ego_front', 'ego_rear'):
            self.assertEqual(w.model.cam_bodyid[w.model.camera(name).id], w.agent_id)


if __name__ == '__main__':
    unittest.main()
