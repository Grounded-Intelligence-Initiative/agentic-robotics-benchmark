"""Geometric guarantees, the physical departure sign, and rejection of a centred wrong-facing dock."""
import json
import math
from itertools import combinations
from pathlib import Path
import sys
import unittest

import mujoco
import numpy as np

TASK = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(TASK/"tests"), str(TASK/"solution")]
from lane import plan
from reference import departure_turn
from world import World

SEEDS = json.loads((TASK/"tests/hidden_scenes.json").read_text())["seeds"]


def direction(delta):
    return math.atan2(delta[1], delta[0])


def radians_diff(a, b):
    return math.remainder(a-b, 2*math.pi)


def clearance(a, b, c, d):
    def side(x, y, p):
        v, w = y-x, p-x
        return v[0]*w[1]-v[1]*w[0]
    if side(a,b,c)*side(a,b,d) < 0 and side(c,d,a)*side(c,d,b) < 0:
        return 0.
    def point(p, x, y):
        v = y-x
        return np.linalg.norm(p-x-np.clip(np.dot(p-x,v)/np.dot(v,v),0,1)*v)
    return min(point(a,c,d),point(b,c,d),point(c,a,b),point(d,a,b))


def geometry(seed):
    stations, _, _, _ = plan(seed)
    points = np.vstack(([0.,0.], np.array(stations)[:,:2]))
    legs = np.diff(points, axis=0)
    yaw = [a for _,_,a in stations]
    bearings = list(map(direction, legs))
    delta = lambda a,b: math.degrees(radians_diff(a,b))
    return {"seed": seed, "centres_headings": stations, "lengths_m": np.linalg.norm(legs, axis=1).tolist(),
            "turns_deg": [delta(a,b) for a,b in zip(bearings[1:],bearings)],
            "facing_turns_deg": [delta(a,b) for a,b in zip(yaw[1:],yaw)],
            "inspection_offsets_deg": [delta(a,b) for a,b in zip(yaw,bearings)],
            "nonadjacent_clearance_m": min(clearance(points[i],points[i+1],points[j],points[j+1])
                                           for i,j in combinations(range(6),2) if j>i+1)}


class RouteTests(unittest.TestCase):
    def test_all_fixed_routes_have_both_signed_turns_clearance_and_one_viewing_offset(self):
        cases = [geometry(s) for s in SEEDS]
        for case in cases:
            with self.subTest(seed=case["seed"]):
                turns, facing = case["turns_deg"], case["facing_turns_deg"]
                self.assertTrue(all(83.9 <= abs(a) <= 96.1 for a in turns))
                self.assertTrue(min(turns) < -80 and max(turns) > 80)
                # Even with both +/-18-degree acceptance windows, the early turns cannot be skipped.
                self.assertTrue(min(facing[:2]) < -80 and max(facing[:2]) > 80)
                lengths = case["lengths_m"]
                self.assertTrue(.489 <= lengths[0] <= .551 and all(.739 <= v <= .871 for v in lengths[1:]))
                self.assertGreater(case["nonadjacent_clearance_m"], .46)
                for x, y, _ in case["centres_headings"]:
                    self.assertTrue(-1.7 < x < 3.7 and -3.2 < y < 3.2)
                self.assertEqual(sum(abs(a)>50 for a in case["inspection_offsets_deg"]), 1)
        offsets = [max(map(abs,c["inspection_offsets_deg"])) for c in cases]
        self.assertTrue(any(d > 149 for d in offsets) and any(52 <= d <= 76 for d in offsets))

    def test_v6_yaw_and_native_gains_unchanged_despite_new_loom(self):
        for seed in SEEDS:
            _, _, yaw, gains = plan(seed)
            rng = np.random.default_rng(seed)
            rng.integers(2)
            self.assertEqual(yaw, rng.uniform(-.035,.035))
            rng.choice([-1,1])
            for i in range(6):
                rng.uniform(-.035,.035) if not i else rng.uniform(.12,.25)
                rng.uniform(.39,.45) if not i else rng.uniform(.45,.61)
            np.testing.assert_array_equal(gains, rng.uniform(.88,1.12,12))

    def test_signed_departure_is_visible_even_when_next_pad_is_off_camera(self):
        w = World(SEEDS[0], render=True)
        self.addCleanup(w.close)
        w.advance()  # stow the launch fixture
        signs = []
        for i in (1,2):
            x,y,yaw = w.stations[i-1]
            nx,ny,_ = w.stations[i]
            w._place_station(i)
            w.data.qpos[:2] = [x,y]
            w.data.qpos[3:7] = [math.cos(yaw/2),0,0,math.sin(yaw/2)]
            mujoco.mj_forward(w.model,w.data)
            turn = radians_diff(math.atan2(ny-y,nx-x),yaw)
            self.assertGreater(abs(turn),math.radians(80))
            sign = departure_turn(w.observe()["images"][0])
            self.assertGreater(sign*turn,0)
            signs.append(sign)
        self.assertLess(signs[0]*signs[1],0)

    def test_pass_through_or_reversed_heading_cannot_complete_an_inspection(self):
        w = World(SEEDS[1], render=False)
        self.addCleanup(w.close)
        i = int(np.argmax(np.abs(geometry(SEEDS[1])["inspection_offsets_deg"])))
        x,y,yaw = w.stations[i]
        w.current, w.approaches, w.delivered = i+1, i, i
        w.data.qpos[:2] = [x,y]
        w.data.qvel[:] = 0
        w.data.qpos[3:7] = [math.cos((yaw+math.pi)/2),0,0,math.sin((yaw+math.pi)/2)]
        mujoco.mj_forward(w.model,w.data)
        for _ in range(200):
            w._tick()
        self.assertEqual((w.approaches,w.delivered), (i+1,i))
        self.assertEqual(w.dwell,0)
        w.data.qpos[3:7] = [math.cos(yaw/2),0,0,math.sin(yaw/2)]
        mujoco.mj_forward(w.model,w.data)
        for _ in range(175):
            w._tick()
        self.assertEqual(w.delivered,i+1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
