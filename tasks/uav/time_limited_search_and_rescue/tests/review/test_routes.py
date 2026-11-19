"""Small camera-map fixtures; no MuJoCo, case seeds or world coordinates."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/"solution"))
from oracle_run import DECK, DIRECTIONS, TRAVEL, Discovery


class Hover:
    def __init__(self):
        self.command = [0., 0., TRAVEL, 0.]
        self.obs = {"frame_id": 0, "done": False}
        self.trace, self.path, self.calls = [], [], []
        self.doors = {}

    def scan(self):
        return None, {d: self.doors.get(d, "wall") for d in DIRECTIONS}

    def face(self, *heading):
        self.calls.append(("face", heading))

    def cell(self, cell, height, settle=0):
        self.calls.append(("cell", cell, height))

    def register(self, cell):
        self.calls.append(("fix", cell))


class Routes(unittest.TestCase):
    inlet, landing, bay, spur = (0, 1), (0, 2), (0, 3), (1, 3)

    def setUp(self):
        self.p = Hover()
        self.nav = Discovery(self.p, self.inlet)

    def fly_out(self):
        nav = self.nav
        nav.floors.update({self.landing: 0., self.bay: DECK, self.spur: DECK})
        nav.adjacency = {self.inlet: {self.landing: "open"},
                         self.landing: {self.bay: "landing"}, self.bay: {self.spur: "open"}}
        nav.seen.update((self.inlet, self.landing, self.bay))
        nav.move(self.landing)
        nav.move(self.bay)
        self.p.calls.clear()

    def test_at_goal_and_at_own_frontier(self):
        nav = self.nav
        self.assertEqual(nav.route(self.inlet), [])
        nav.return_to(self.inlet)
        self.assertEqual(nav.route(), [])
        nav.explore()
        self.assertFalse(self.p.calls)
        self.p.doors[(0, 1)] = "open"
        nav.scan(False)
        self.assertEqual(nav.route(), [self.landing])
        self.assertIsNone(nav.route(self.landing))  # visible, never crossed

    def test_known_return_not_nearer_frontier(self):
        self.fly_out()
        nav = self.nav
        self.assertEqual(nav.route(), [self.spur])
        self.assertEqual(nav.route(self.inlet), [self.landing, self.inlet])
        nav.return_to(self.inlet)
        self.assertEqual([c[1] for c in self.p.calls if c[0] == "fix"], [self.landing, self.inlet])
        self.assertIn(("cell", self.landing, TRAVEL+DECK), self.p.calls)
        self.assertIn(("cell", self.landing, TRAVEL), self.p.calls)
        nav.return_to(self.bay)
        self.assertEqual(nav.current, self.bay)
        self.assertNotIn(self.spur, self.p.path)

    def test_unreachable_goal_does_not_launch_exploration(self):
        self.fly_out()
        self.assertIsNone(self.nav.route(self.spur))
        with self.assertRaisesRegex(RuntimeError, "no crossed route"):
            self.nav.return_to(self.spur)
        self.assertFalse(self.p.calls)

    def test_unscanned_arrival_and_occluded_back_apron(self):
        self.fly_out()
        nav = self.nav
        nav.seen.remove(self.bay)
        self.assertEqual(nav.route(), [])
        self.assertEqual(nav.route(self.inlet), [self.landing, self.inlet])
        _, view = nav.scan(False)  # blank view with the sled over the incoming apron
        self.assertEqual(view[(0, -1)], "landing")
        self.assertFalse(self.p.calls)

    def test_confirmed_shutter_breaks_both_return_directions(self):
        self.fly_out()
        nav = self.nav
        nav.confirmed_shutters.add(tuple(sorted((self.bay, self.landing))))
        self.p.doors = {(0, -1): "closed", (1, 0): "open"}
        nav.scan(True)
        self.assertIsNone(nav.route(self.inlet))
        self.assertEqual(nav.route(), [self.spur])
        self.assertNotIn(self.landing, nav.adjacency[self.bay])
        self.assertNotIn(self.bay, nav.adjacency[self.landing])
        # A parallel observed corridor is viable for egress, not a proven return.
        bypass = (1, 2)
        nav.adjacency[self.spur] = {bypass: "landing"}
        nav.adjacency[bypass] = {self.landing: "open"}
        nav.floors[bypass] = 0.
        nav.seen.update((self.spur, bypass))
        self.assertIsNone(nav.route(self.inlet))
        self.assertEqual(nav.route(self.inlet, observed_detour=True),
                         [self.spur, bypass, self.landing, self.inlet])
        self.p.doors[(0, -1)] = "open"
        nav.scan(True)
        self.assertNotIn(self.landing, nav.adjacency[self.bay])

    def test_terminal_move_is_not_a_crossing(self):
        nav = self.nav
        self.p.doors[(0, 1)] = "open"
        nav.scan(False)
        self.p.register = lambda cell: self.p.obs.update(done=True)
        nav.move(self.landing)
        self.assertEqual(nav.current, self.inlet)
        self.assertFalse(nav.crossed)


if __name__ == "__main__":
    unittest.main()
