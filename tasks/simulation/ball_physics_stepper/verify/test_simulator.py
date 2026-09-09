"""Maintainer tests: run this file from the task checkout with Python 3.12."""
import importlib.util
import json
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from cases import edge_cases, suite
from reference import Reference
from robotics_grader import SolverAborted, seed_rng
from verify import agrees, evaluate, state_values

spec = importlib.util.spec_from_file_location("ball_oracle", HERE.parent / "oracle/solver.py")
oracle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(oracle)


class LocalSolver:
    def __init__(self, mutation=None):
        self.sim = oracle.BallSimulator()
        self.mutation = mutation

    def request(self, req):
        req = json.loads(json.dumps(req))
        if req["type"] == "reset":
            if self.mutation == "ignore_plate":
                req["config"]["plate_y"] = None
            if self.mutation == "fixed_gravity":
                req["config"]["gravity"] = 9.81
            if self.mutation == "fixed_mass":
                req["config"]["mass"] = 1
            if self.mutation == "elastic_only":
                req["config"]["restitution"] = 1
            if self.mutation == "bad_reset" and hasattr(self.sim, "time"):
                return {"state": self.sim.state()}
            state = self.sim.reset(req["config"], req["state"])
        else:
            if self.mutation == "ignore_impulse":
                req["impulse"] = [0, 0]
            state = self.sim.step(req["dt"], req["impulse"])
        if self.mutation == "constant":
            state = {"position": [0, 0], "velocity": [0, 0], "time": 0}
        return {"state": state}


class PhysicsTests(unittest.TestCase):
    def make(self, g=2, mass=1, e=1, plate=-0.125, p=(0, 1), v=(0, 0)):
        sim = oracle.BallSimulator()
        sim.reset({"gravity": g, "mass": mass, "restitution": e,
                   "radius": 0.125, "plate_y": plate},
                  {"position": list(p), "velocity": list(v)})
        return sim

    def assertState(self, actual, expected):
        self.assertTrue(agrees(state_values({"state": actual}), expected), actual)

    def test_ballistic_and_mass(self):
        sim = self.make(g=10, mass=2, plate=None, p=(0, 5), v=(1, 0))
        self.assertState(sim.step(0.2, [2, 4]), [0.4, 5.2, 2, 0, 0.2])
        self.assertState(sim.step(0.3, [0, 0]), [1, 4.75, 2, -3, 0.5])

    def test_collision_at_interval_end(self):
        self.assertState(self.make().step(1, [0, 0]), [0, 0, 0, 2, 1])

    def test_multiple_elastic_collisions(self):
        sim = self.make(g=20, p=(0, 0.025))
        # Drop for 0.05 s; bottom-to-bottom period is 0.1 s thereafter.
        self.assertState(sim.step(1.975, [0, 0]), [0, 0.01875, 0, 0.5, 1.975])

    def test_rest_then_lift(self):
        sim = self.make(e=0, v=(1, 0))
        self.assertState(sim.step(2, [0, 0]), [2, 0, 1, 0, 2])
        self.assertState(sim.step(0, [0, -2]), [2, 0, 1, 0, 2])
        self.assertState(sim.step(0.5, [0, 2]), [2.5, 0.75, 1, 1, 2.5])

    def test_zero_gravity_collision(self):
        self.assertState(self.make(g=0, v=(1, -1), e=0.5).step(2, [0, 0]),
                         [2, 0.5, 1, 0.5, 2])

    def test_inelastic_collapse(self):
        self.assertState(self.make(e=0.5, p=(0, 0.01)).step(2, [0, 0]),
                         [0, 0, 0, 0, 2])

    def test_subdivision_invariance(self):
        one, many = self.make(e=0.65), self.make(e=0.65)
        a = one.step(2, [1, 2])
        many.step(0, [1, 2])
        for _ in range(200):
            b = many.step(0.01, [0, 0])
        self.assertTrue(agrees(state_values({"state": a}), state_values({"state": b})))

    def test_independent_reference_random_suites(self):
        for seed in range(12):
            measured, details = evaluate(LocalSolver(), suite(seed_rng(seed+1), 3, 50))
            self.assertEqual(measured, 1, (seed, details))

    def test_tiny_height_many_bounces(self):
        for e in (1, 0.999999, 0.999999999, 0.99, 0.5):
            config = {"gravity": 20, "mass": 1, "restitution": e,
                      "radius": 0.125, "plate_y": -0.125}
            state = {"position": [0, 0], "velocity": [0, 0.0001]}
            a, b = oracle.BallSimulator(), Reference()
            a.reset(config, state)
            b.reset(config, state)
            x, y = a.step(1.9999993, [0, 0]), b.step(1.9999993, [0, 0])
            self.assertTrue(agrees(state_values({"state": x}), state_values({"state": y})), (e,x,y))

    def test_hand_cases_match_decimal(self):
        for requests in edge_cases():
            a, b = oracle.BallSimulator(), Reference()
            for req in requests:
                if req["type"] == "reset":
                    x = a.reset(req["config"], req["state"])
                    y = b.reset(req["config"], req["state"])
                else:
                    x, y = a.step(req["dt"], req["impulse"]), b.step(req["dt"], req["impulse"])
                self.assertTrue(agrees(state_values({"state": x}), state_values({"state": y})))

    def test_wrong_simulators_do_not_pass(self):
        cases = suite(seed_rng(778), 3, 40)
        for mutation in ("constant", "ignore_plate", "fixed_gravity", "fixed_mass",
                         "elastic_only", "bad_reset", "ignore_impulse"):
            rate, _ = evaluate(LocalSolver(mutation), cases)
            self.assertLess(rate, 0.95, (mutation, rate))

    def test_malformed_output(self):
        for bad in (None, {}, {"state": []},
                    {"state": {"position": [0], "velocity": [0, 0], "time": 0}},
                    {"state": {"position": [0, 0], "velocity": [0, 0], "time": "0"}},
                    {"state": {"position": [0, 0], "velocity": [0, 0], "time": True}},
                    {"state": {"position": [float("nan"), 0], "velocity": [0, 0], "time": 0}},
                    {"state": {"position": [float("inf"), 0], "velocity": [0, 0], "time": 0}},
                    {"state": {"position": [10**400, 0], "velocity": [0, 0], "time": 0}}):
            with self.assertRaises(SolverAborted):
                state_values(bad)


if __name__ == "__main__":
    unittest.main(verbosity=2)
