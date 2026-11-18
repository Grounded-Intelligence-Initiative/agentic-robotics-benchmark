"""Author regressions, run inside the task Docker image."""
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent/"runtime"))

from articulated import MAX_SCOUT, layout
from service import Runtime


HOLD = {"target_rad": [0., .9, -1.8]*4, "ticks": 40}


def request(runtime, method, **params):
    return runtime.call({"method": method, "params": params})


class Contract(unittest.TestCase):
    def test_packaged_copies_agree(self):
        task = HERE.parents[1]
        for local, remote in (("runtime", "runtime"), ("payload", "setup_payload")):
            source, target = task/"environment/setup"/local, task/"tests"/remote
            for file in source.iterdir():
                if file.is_file():
                    self.assertEqual(file.read_bytes(), (target/file.name).read_bytes(), file.name)

    def test_commissioning_does_not_deploy_target(self):
        with TemporaryDirectory() as tmp:
            runtime = Runtime(tmp, "commissioning", 1)
            try:
                for _ in range(4):
                    request(runtime, "act", action=[.2, 0.])
                with self.assertRaisesRegex(ValueError, "scout-only"):
                    request(runtime, "advance")
                self.assertIsNone(runtime.world.go1)
                self.assertEqual(request(runtime, "new_site")["phase"], 0)
            finally:
                runtime.world.close()

    def test_ground_action_and_free_base(self):
        with TemporaryDirectory() as tmp:
            runtime = Runtime(tmp, "graded", 1)
            try:
                world = runtime.world
                self.assertEqual(world.observe()["phase"], 0)
                for method in ("advance", "new_site", "truth"):
                    with self.assertRaises(ValueError):
                        request(runtime, method)
                with self.assertRaises(ValueError):
                    request(runtime, "act", action=[.61, 0.])
                for _ in range(MAX_SCOUT):
                    request(runtime, "act", action=[0., 0.])
                with self.assertRaises(ValueError):
                    request(runtime, "act", action=[0., 0.])
                initial = request(runtime, "advance")
                self.assertEqual(set(initial), {"phase", "done", "tracking_xy_m", "heading_rad",
                                               "gravity_body_unit", "gyro_body_rad_s", "joint_rad",
                                               "joint_rad_s", "foot_contact", "ticks_remaining",
                                               "requests_remaining"})
                with self.assertRaises(ValueError):
                    request(runtime, "advance")
                dog = world.go1
                self.assertEqual((dog.model.nq, dog.model.nv, dog.model.nu), (19, 18, 12))
                self.assertEqual((dog.model.neq, dog.model.nmocap, dog.model.ntendon), (0, 0, 0))
                self.assertEqual(dog.model.opt.timestep, .002)
                q0 = dog.data.qpos.copy()
                for bad in ([0., 0.], {**HOLD, "ticks": 41}, {**HOLD, "ticks": True},
                            {**HOLD, "target_rad": [float("nan")]*12},
                            {**HOLD, "target_rad": [5.]*12}):
                    with self.assertRaises(ValueError):
                        request(runtime, "act", action=bad)
                np.testing.assert_array_equal(q0, dog.data.qpos)
                for _ in range(10):
                    request(runtime, "observe")
                np.testing.assert_array_equal(q0, dog.data.qpos)
                for _ in range(40):
                    request(runtime, "act", action=HOLD)
                np.testing.assert_array_equal(dog.data.ctrl, HOLD["target_rad"])
                self.assertFalse(np.any(dog.data.xfrc_applied) or np.any(dog.data.qfrc_applied))
                self.assertEqual(dog.ticks, 1600)
                self.assertLess(np.linalg.norm(dog.xy), .06)
                self.assertEqual(world.metrics()["success"], 0)
                self.assertEqual(world.metrics()["corridor_entered"], 0)
                self.assertEqual(world.metrics()["first_aisle_correct"], 0)
                dog.outcome = "fell"
                self.assertEqual(world.metrics()["fell_before_aisle"], 1)
                dog.corridor = -world.side
                self.assertEqual(world.metrics()["corridor_entered"], 1)
                self.assertEqual(world.metrics()["first_aisle_correct"], 0)
                self.assertEqual(world.metrics()["fell_before_aisle"], 0)
                dog.corridor = world.side
                self.assertEqual(world.metrics()["first_aisle_correct"], 1)
            finally:
                runtime.world.close()

    def test_alternative_routes_not_visible_at_launch(self):
        for seed in range(32):
            _, open_side, _, walls = layout(seed)
            x0, _, y0, y1 = walls["gate"]
            self.assertGreater(x0, 1.75)
            self.assertTrue(y1 < 0 if open_side == 1 else y0 > 0)


if __name__ == "__main__":
    unittest.main()
