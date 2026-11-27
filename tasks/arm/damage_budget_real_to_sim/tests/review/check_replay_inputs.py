"""Private contract tests: streams are reproducible, isolated and not public."""
import copy
from itertools import product
from pathlib import Path
import sys
import unittest

TASK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TASK/"tests"))
from arm_evaluation import transfer_plan
from arm_replay import prepare_arm_inputs, probe_seeds
from robotics_grader import seed_rng
from world import arm_rng, drive_cassette


class Inputs(unittest.TestCase):
    def test_per_scene_streams(self):
        plan = transfer_plan(seed_rng(71351), 35101)
        master = bytes(range(32)).hex()
        keys = probe_seeds(plan, master)
        self.assertEqual(len(set(keys)), 8)
        self.assertEqual(keys[::-1], probe_seeds(plan[::-1], master))
        self.assertNotEqual(keys, probe_seeds(plan, bytes(32).hex()))
        r = seed_rng(int(keys[0], 16))
        for _ in range(750):
            r.random()
        other = seed_rng(int(keys[1], 16))
        alone = seed_rng(int(probe_seeds(plan[1:2], master)[0], 16))
        self.assertEqual([other.random() for _ in range(30)], [alone.random() for _ in range(30)])
        mounts = [tuple(arm_rng(s,tag).choice((-1,1)) for tag in
                        (b"arm-sleeve-dir",b"arm-key-turn",b"arm-cradle-tare"))
                  for s, _ in plan]
        self.assertEqual(sorted(mounts), sorted(product((-1,1),repeat=3)))
        self.assertEqual(sorted(drive_cassette(s) for s, _ in plan),
                         sorted(product((-1, 1), (.8, 1.2), (.8, 1.2))))

    def test_input_round_trip(self):
        record, _ = prepare_arm_inputs(35101)
        recovered, cases = prepare_arm_inputs(35101, copy.deepcopy(record))
        self.assertEqual(recovered, record)
        self.assertEqual(len(cases), 4)
        bad = copy.deepcopy(record)
        bad["reconstruction"][0]["actions"][0][0] += .01
        with self.assertRaises(RuntimeError):
            prepare_arm_inputs(35101, bad)
        bad = copy.deepcopy(record)
        bad["probe_episode_hex"][0] = "00"*32
        with self.assertRaises(RuntimeError):
            prepare_arm_inputs(35101, bad)

    def test_not_in_participant_surface(self):
        for path in [TASK/"instruction.md", *(TASK/"environment/setup").rglob("*.py")]:
            with self.subTest(path=path):
                self.assertNotIn("probe_master_hex", path.read_text())
                self.assertNotIn("probe_episode_hex", path.read_text())
                self.assertNotIn("offline-record.json", path.read_text())


if __name__ == "__main__":
    unittest.main()
