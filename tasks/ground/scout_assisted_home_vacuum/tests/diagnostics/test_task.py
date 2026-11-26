import base64
from io import BytesIO
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "tests"), str(ROOT / "solution")]
from PIL import Image
from cameras import Cameras
from controller import deliver, read_view
from world import Home, VEC, connected, plus


class Contracts(unittest.TestCase):
    def test_shared_clock_and_battery(self):
        w = Home(10, ticks=2, flight=1)
        inward = next(d for d,v in VEC.items() if plus(w.dock,v) in w.mats.values())
        w.advance([inward, inward, 2, False])
        self.assertEqual((w.t, w.motion, w.fuel), (1, [1, 2], 0))
        p = w.air
        w.advance(["stay", inward, 2, False])
        self.assertEqual((w.t, w.air), (2, p))
        with self.assertRaises(ValueError):
            w.advance(["stay", "stay", 0, False])

    def test_push_then_clean_and_terminal_goal(self):
        w = Home(22)
        b = w.box
        d,v = next((d,v) for d,v in VEC.items() if d!="stay"
                   and all(not w.blocked((b[0]+j*v[0],b[1]+j*v[1])) and
                           (b[0]+j*v[0],b[1]+j*v[1]) not in w.furniture for j in (-1,1)))
        w.ground = (b[0]-v[0],b[1]-v[1])
        w.advance([d, "stay", 0, False])
        self.assertEqual(w.ground, b)
        self.assertNotEqual(w.box, b)
        self.assertIn(b, w.dust)
        w.advance(["stay", "stay", 0, True])
        self.assertNotIn(b, w.dust)
        w.dust.clear()
        w.ground = w.dock
        w.box = w.mats[w.target]
        self.assertTrue(w.result()["success"])
        w.target = "blue" if w.target == "red" else "red"
        self.assertFalse(w.result()["success"])

    def test_observation_and_no_free_progress(self):
        w = Home(17, flight=1)
        c = Cameras(w)
        try:
            obs = c.observation()
            self.assertEqual(
                set(obs), {"instruction", "ground", "air", "moved", "left", "flight"}
            )
            for cam in ("ground", "air"):
                im = Image.open(BytesIO(base64.b64decode(obs[cam])))
                self.assertEqual((im.size, im.mode), ((84, 84), "RGB"))
            ground = {p: k for p, k, _ in read_view(obs["ground"], (0, 0), 1)}
            self.assertEqual(ground[(0, 0)], "dock")
            w.advance(["stay", "stay", 0, True])
            self.assertIsNone(c.observation()["air"])
            self.assertEqual(w.result()["score"], 0)
        finally:
            c.close()

    def test_replayable_unseen_apartments_and_invalid_actions(self):
        self.assertEqual(Home(1).original, Home(1).original)
        self.assertGreater(len({frozenset(Home(s).wall) for s in range(12)}), 8)
        for action in (["jump", "stay", 0, True], ["stay", "N", 3, True], [0, 0], None):
            with self.assertRaises(ValueError):
                Home(1).advance(action)

    def test_runner_brush_is_a_real_constraint(self):
        w=Home(203)
        w.ground=next(iter(w.rugs))
        w.advance(["stay","stay",0,False])
        self.assertFalse(w.scuffs)
        w.advance(["stay","stay",0,True])
        w.ground,w.box=w.dock,w.mats[w.target]
        w.dust.clear()
        self.assertFalse(w.result()["success"])
        self.assertAlmostEqual(w.result()["score"],.70)

    def test_generated_passages_and_goal_access(self):
        orientations=set()
        for seed in range(100):
            w=Home(seed)
            free=set(w.area)-w.wall-set(w.furniture)
            self.assertTrue(connected(free),seed)
            self.assertFalse(w.rugs & (w.dust | set(w.mats.values()) | {w.dock}))
            self.assertGreaterEqual(len(w.pieces),12)
            self.assertTrue((w.dust | w.rugs | {w.box,w.dock} | set(w.mats.values())) <= free)
            self.assertTrue(deliver(w.dock,w.box,w.mats[w.target],free),seed)
            orientations.add(next(d for d,v in VEC.items() if plus(w.dock,v) in w.mats.values()))
        self.assertEqual(len(orientations),4)

    def test_scout_has_no_xray_or_cleaning_credit(self):
        w = Home(42)
        start = next(
            (x, y)
            for x, y in w.area
            if not w.blocked((x, y))
            and w.blocked((x + 1, y))
            and not w.blocked((x + 2, y))
        )
        w.air = start
        c = Cameras(w)
        try:
            obs = c.observation()
            visible = {p for p, _, _ in read_view(obs["air"], (0, 0), 3)}
            self.assertIn((1, 0), visible)
            self.assertNotIn((2, 0), visible)
            w.advance(["stay", "E", 2, False])
            self.assertEqual(w.motion, [0, 0])
            self.assertEqual(w.result()["score"], 0)
        finally:
            c.close()


if __name__ == "__main__":
    unittest.main()
