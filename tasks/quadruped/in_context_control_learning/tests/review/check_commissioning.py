"""Physical, optical and blinded-operator checks for the exchangeable loom."""
from itertools import permutations
from pathlib import Path
import sys
import unittest

import mujoco
import numpy as np

TASK = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(TASK/"tests"),str(TASK/"solution")]
from lane import loom, plan
from reference import Controller, clear_pickups
from service import dispatch
from world import World


class RackTests(unittest.TestCase):
    def test_installation_lot_is_balanced_and_independent_of_dynamics(self):
        hardware = set(permutations(range(4)))
        self.assertEqual({loom(101,i) for i in range(24)}, hardware)
        neutral_stations, _, yaw, gains = plan(101,0)
        for i in range(24):
            stations, _, y, g = plan(101,i)
            self.assertEqual((stations,y), (neutral_stations,yaw))
            np.testing.assert_array_equal(g,gains)

    def test_flags_measure_the_pad_not_motor_commands(self):
        w = World(101, scene_index=0)
        self.addCleanup(w.close)
        self.assertEqual(w.pickup_covered().tolist(),[True]*4)
        original = w.observe()["images"][0]
        w.data.ctrl[:] = [0.,1.22,-2.44]*4  # no integration, feet did not move
        self.assertEqual(w.observe()["images"][0],original)
        # Compliant foot-floor contact slightly compresses the settled sphere into the plane.
        # Test the specified absolute clearance, not displacement from that settled pose.
        sole_z = w.data.geom_xpos[w.feet,2] - w.model.geom_size[w.feet,0]
        w.data.qpos[2] += .024 - max(sole_z)
        mujoco.mj_forward(w.model,w.data)
        self.assertEqual(w.pickup_covered().tolist(),[True]*4)
        w.data.qpos[2] += .006
        mujoco.mj_forward(w.model,w.data)
        self.assertEqual(w.pickup_covered().tolist(),[False]*4)
        self.assertEqual(clear_pickups(w.observe()["images"][0]),(True,)*4)
        w.recover()
        w.data.qpos[1] += .085  # same height, outside the four spatial footprints
        mujoco.mj_forward(w.model,w.data)
        self.assertEqual(w.pickup_covered().tolist(),[False]*4)

    def test_three_rgb_probes_identify_every_replug_of_the_same_robot(self):
        reference_image = None
        for i in range(24):
            with self.subTest(i=i):
                w = World(101, scene_index=i)
                try:
                    image = w.observe()["images"][0]
                    if reference_image is None:
                        reference_image = image
                    self.assertEqual(image,reference_image)  # no static clue, even at the pixel level
                    operator = Controller(lambda method,**params: dispatch(w,method,params))
                    guess = operator.identify()
                    # Only the test compares with ground truth, after the blind operator returns.
                    self.assertEqual(guess,w.legs_by_port)
                    self.assertEqual((w.steps,w.ticks,w.recoveries,w.falls,w.calibration),(3,165,3,0,0))
                    self.assertEqual(operator.probe_readings[0],(False,)*4)
                    self.assertEqual(len({tuple(m) for m in operator.probe_readings[1:]}),3)
                finally:
                    w.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
