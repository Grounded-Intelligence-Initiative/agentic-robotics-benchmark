import base64
from io import BytesIO
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "tests"), str(ROOT / "solution")]
from PIL import Image
from cameras import Cameras
from controller import as_image, deliver, infer_mount, north_up, read_card, read_view
from world import Home, ROOMS, VEC, connected, plus


class Contracts(unittest.TestCase):
    def test_shared_clock_and_battery(self):
        w = Home(10, ticks=2, flight=1)
        w.mount = (0, 0)
        inward = next(
            d
            for d, v in VEC.items()
            if d != "stay"
            and all(not w.blocked(plus(w.dock, (n * v[0], n * v[1]))) for n in (1, 2))
        )
        w.advance([inward, inward, 2, False])
        self.assertEqual((w.t, w.motion, w.fuel), (1, [1, 2], 0))
        p = w.air
        w.advance(["stay", inward, 2, False])
        self.assertEqual((w.t, w.air), (2, p))
        with self.assertRaises(ValueError):
            w.advance(["stay", "stay", 0, False])

    def test_push_then_clean_and_terminal_goal(self):
        w = Home(22)
        w.mount = (0, 0)
        b = w.box
        d, v = next(
            (d, v)
            for d, v in VEC.items()
            if d != "stay"
            and all(
                not w.blocked((b[0] + j * v[0], b[1] + j * v[1]))
                and (b[0] + j * v[0], b[1] + j * v[1]) not in w.furniture
                for j in (-1, 1)
            )
        )
        w.ground = (b[0] - v[0], b[1] - v[1])
        w.advance([d, "stay", 0, False])
        self.assertEqual(w.ground, b)
        self.assertNotEqual(w.box, b)
        self.assertIn(b, w.dust)
        w.advance(["stay", "stay", 0, True])
        self.assertNotIn(b, w.dust)
        w.dust.clear()
        w.ground = w.dock
        w.box = w.mats[w.target]
        self.assertFalse(w.result()["success"])
        w.staged = True
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
        w = Home(203)
        w.ground = next(iter(w.rugs))
        w.advance(["stay", "stay", 0, False])
        self.assertFalse(w.scuffs)
        w.advance(["stay", "stay", 0, True])
        w.ground, w.box, w.staged = w.dock, w.mats[w.target], True
        w.dust.clear()
        self.assertFalse(w.result()["success"])
        self.assertAlmostEqual(w.result()["score"], 0.70)

    def test_generated_passages_and_goal_access(self):
        orientations = set()
        for seed in range(100):
            w = Home(seed)
            w.mount = (0, 0)
            free = set(w.area) - w.wall - set(w.furniture)
            self.assertTrue(connected(free), seed)
            self.assertFalse(w.rugs & (w.dust | set(w.mats.values()) | {w.dock}))
            self.assertGreaterEqual(len(w.pieces), 12)
            self.assertTrue(
                (w.dust | w.rugs | {w.box, w.dock} | set(w.mats.values())) <= free
            )
            self.assertIn(w.area[w.mats[w.stage]], ROOMS)
            self.assertNotEqual(w.area[w.mats["jade"]], w.area[w.mats["violet"]])
            self.assertEqual(w.area[w.mats[w.target]], "hall")
            orientations.add(
                (
                    w.dock[0] == 1,
                    w.dock[0] == w.width - 2,
                    w.dock[1] == 1,
                    w.dock[1] == w.height - 2,
                )
            )
            for pad in (w.stage, w.target):
                plan = deliver(w.ground, w.box, w.mats[pad], free)
                self.assertTrue(plan, seed)
                for d in plan:
                    w.advance([d, "stay", 0, False])
                self.assertEqual(w.box, w.mats[pad], seed)
            self.assertTrue(w.result()["parked"], seed)
        self.assertEqual(len(orientations), 4)

    def test_scout_has_no_xray_or_cleaning_credit(self):
        w = Home(42)
        w.mount = (0, 0)
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

    def test_visual_work_order_is_counterfactual_and_scout_only(self):
        images = []
        layouts = []
        for order in (1, 2):
            w = Home(211, order_key=order)
            layouts.append(
                (w.card, w.dock, w.mats, w.request, w.stage, w.target, w.prompt)
            )
            w.mount = (order, (order + 1) % 4)
            c = Cameras(w)
            try:
                obs = c.observation()
                self.assertEqual(
                    (
                        infer_mount(as_image(obs["ground"]), 1),
                        infer_mount(as_image(obs["air"]), 3),
                    ),
                    w.mount,
                )
                w.air = w.card
                w.ground = next(
                    plus(w.card, v)
                    for d, v in VEC.items()
                    if d != "stay"
                    and not w.blocked(plus(w.card, v))
                    and plus(w.card, v) not in w.furniture
                )
                obs = c.observation()
                answer = read_card(north_up(as_image(obs["air"]), w.mount[1]), (0, 0))
                self.assertEqual(answer, ((set(w.request), w.stage, w.target), (0, 0)))
                images.append(north_up(as_image(obs["ground"]), w.mount[0]).tobytes())
            finally:
                c.close()
        self.assertEqual(layouts[0][:3], layouts[1][:3])
        self.assertNotEqual(layouts[0][3:6], layouts[1][3:6])
        self.assertEqual(layouts[0][6], layouts[1][6])
        self.assertEqual(images[0], images[1])

    def test_protected_treatment_cannot_be_swept_back_into_place(self):
        w = Home(14)
        r = next(r for r in ROOMS if r not in w.request)
        w.ground = next(iter(w.original[r]))
        w.advance(["stay", "stay", 0, True])
        self.assertEqual(w.result()["treatment_preserved"], 0.75)
        w.staged, w.ground, w.box = True, w.dock, w.mats[w.target]
        w.dust.clear()
        self.assertFalse(w.result()["success"])


if __name__ == "__main__":
    unittest.main()
