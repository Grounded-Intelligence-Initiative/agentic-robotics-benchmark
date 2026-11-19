"""Author regressions, run inside the task Docker image."""
from collections import Counter
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent/"runtime"))

from articulated import CAMERA_Z, FOV, MAX_SCOUT, WALL_Z, Go1, layout
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
                dog.choices[:] = ["shortcut", "wide", "wide"]
                self.assertEqual(world.metrics()["corridor_entered"], 1)
                self.assertEqual(world.metrics()["first_aisle_correct"], 0)
                self.assertEqual(world.metrics()["fell_before_aisle"], 0)
                self.assertEqual(world.metrics()["second_aisle_correct"], 1)
                dog.choices[:] = ["wide", "wide", "shortcut"]
                self.assertEqual(world.metrics()["first_aisle_correct"], 1)
                self.assertEqual(world.metrics()["second_shortcut_selected"], 1)
                dog.contact_wall = "rack3_1"
                self.assertEqual(world.metrics()["contact_first"], 0)
                self.assertEqual(world.metrics()["contact_second"], 1)
            finally:
                runtime.world.close()

    def test_fixed_suite_has_all_route_pairs_and_metric_gaps(self):
        cases = Counter()
        seeds = json.loads((HERE.parent/"hidden/sites.json").read_text())["seeds"]
        launch_sight = (CAMERA_Z-WALL_Z)*np.tan(np.deg2rad(FOV/2))
        for seed in seeds:
            _, racks, goal, walls = layout(seed)
            cases[tuple(rack.wide[0] > 0 for rack in racks[::2])] += 1
            self.assertGreater(racks[0].near, launch_sight)
            self.assertGreater(goal[0], racks[2].far)
            for i, rack in enumerate(racks):
                self.assertGreaterEqual(rack.wide[1]-rack.wide[0], .94-1e-9)
                expected = [rack.wide]
                if i != 1:
                    self.assertLess(rack.shortcut[1]-rack.shortcut[0], .254)
                    expected.append(rack.shortcut)
                else:
                    self.assertIsNone(rack.shortcut)
                slabs = sorted((y0, y1) for n, (_, _, y0, y1) in walls.items()
                               if n.startswith(f"rack{i+1}_"))
                self.assertEqual([(a[1], b[0]) for a, b in zip(slabs, slabs[1:])], sorted(expected))
        self.assertEqual(sorted(cases.values()), [2]*4)

    def test_choices_latch_separately_from_full_crossings(self):
        _, racks, _, _ = layout(173)
        dog = SimpleNamespace(racks=racks, choices=[None]*3, crossed=[False]*3, lateral_margin=[None]*3)
        for i, rack in enumerate(racks):
            y = sum(rack.wide)/2
            dog.xy = (rack.near-.40, y)
            Go1.record_passages(dog)
            self.assertIsNone(dog.choices[i])
            dog.xy = (rack.near-.30, y)
            Go1.record_passages(dog)
            self.assertEqual(dog.choices[i], "wide")
            self.assertFalse(dog.crossed[i])
            dog.xy = (rack.far+.35, y)
            Go1.record_passages(dog)
            self.assertTrue(dog.crossed[i])


if __name__ == "__main__":
    unittest.main()
