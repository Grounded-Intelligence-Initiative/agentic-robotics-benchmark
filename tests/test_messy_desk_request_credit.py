"""Request milestones must not reward repeated holds, historical contact, a mere touch or a guessed mark."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


TASK = Path(__file__).resolve().parents[1] / "tasks/arm/object_finding_and_organizing_on_messy_desk"


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, TASK / "verify" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


score = load("messy_desk_credit_score", "messy_desk_score.py")
env = load("messy_desk_credit_env", "env.py")


class RequestCreditTests(unittest.TestCase):
    def setUp(self):
        self.scorer = score.EpisodeScorer(horizon=1)

    def event(self, kind, rid=0):
        self.scorer.add_step({"objects": [], "events": [{"event": kind, "request": rid}]})

    def test_hold_is_immediate_once_and_delivery_tops_up(self):
        self.event("request_opened")
        self.assertEqual(self.scorer.score(), 0)
        self.event("request_first_hold")
        self.assertEqual(self.scorer.request_progress(), 0.5)
        self.assertEqual(self.scorer.request_points[0], 1)
        self.assertEqual(self.scorer.score(), 0.25)  # One of two request points; R has half the weight.
        for _ in range(10):
            self.event("request_first_hold")
        self.assertEqual(self.scorer.request_progress(), 0.5)
        self.assertEqual(self.scorer.counts["first_holds"], 1)
        self.event("request_delivered")
        self.assertEqual(self.scorer.request_progress(), 1)
        self.assertEqual(self.scorer.score(), 0.5)
        self.assertEqual(self.scorer.request_points[0], 2)
        self.event("request_first_hold")
        self.assertEqual(self.scorer.request_progress(), 1)

    def test_expiry_retains_only_earned_progress(self):
        for held in (False, True):
            with self.subTest(held=held):
                self.setUp()
                self.event("request_opened")
                if held:
                    self.event("request_first_hold")
                self.event("request_expired")
                self.assertEqual(self.scorer.request_points[0], 1 if held else 0)
                self.assertEqual(self.scorer.request_progress(), 0.5 if held else 0)
                self.assertEqual(self.scorer.request_rate(), 0)

    def test_wrong_mark_costs_a_point(self):
        for wrong in score.WRONG_MARK_EVENTS:
            for held in (False, True):
                with self.subTest(wrong=wrong, held=held):
                    self.setUp()
                    self.event("request_opened")
                    if held:
                        self.event("request_first_hold")
                    self.event(wrong)
                    self.assertEqual(self.scorer.request_points[0], (1 if held else 0) - score.WRONG_MARK_COST)
                    self.assertEqual(self.scorer.request_progress(), 0)   # floored at 0
                    self.assertEqual(self.scorer.request_rate(), 0)

    def test_guessing_absent_does_not_pay(self):
        # one correct and four wrong "no such object" marks: the one-in-five absent rate
        for rid, kind in enumerate(["request_absent_correct"] + ["request_absent_wrong"] * 4):
            self.event("request_opened", rid)
            self.event(kind, rid)
        self.assertEqual(self.scorer.request_progress(), 0)
        self.setUp()
        self.event("request_opened", 0)
        self.event("request_delivered", 0)
        self.event("request_opened", 1)
        self.event("request_absent_wrong", 1)
        self.assertEqual(self.scorer.request_progress(), 0.25)   # (2 - 1) / 4

    def test_absence_and_open_requests_use_one_bounded_average(self):
        self.event("request_opened", 0)
        self.event("request_absent_correct", 0)
        self.event("request_opened", 1)
        self.assertEqual(self.scorer.request_progress(), 0.5)
        self.event("request_first_hold", 1)
        self.assertEqual(self.scorer.request_progress(), 0.75)
        self.event("request_delivered", 1)
        self.assertEqual(self.scorer.request_progress(), 1)

    def test_empty_and_organization_only_remain_unchanged(self):
        self.assertEqual(self.scorer.request_progress(), 0)
        self.scorer.add_step({"objects": [{"id": 0, "present": True, "value": 1, "baseline": 0}]})
        self.assertEqual(self.scorer.organization(), 1)
        self.assertEqual(self.scorer.score(), 0.5)

    def world(self, target_state="held"):
        target = {"id": 0, "present": True, "state": target_state, "in_container": None}
        other = {"id": 1, "present": True, "state": "held", "in_container": None}
        return SimpleNamespace(
            t=10, rng=None, events=[], objects=[target, other], touched={0},
            request={"id": 0, "object": 0, "status": "open", "deadline_t": 100},
            requests_done=[], next_request=math.inf, next_arrival=math.inf,
            next_disturb=math.inf,
        )

    def test_current_target_hold_only_and_once_per_request(self):
        world = self.world("on_table")
        env.MessyDeskEnv._world_events(world, 0)
        self.assertEqual(world.events, [])  # Historical contact and another held object do not count.
        world.objects[0]["state"] = "held"
        env.MessyDeskEnv._world_events(world, 0)
        self.assertEqual([e["event"] for e in world.events], ["request_first_hold"])
        world.events.clear()
        world.objects[0]["state"] = "on_table"
        env.MessyDeskEnv._world_events(world, 0)
        world.objects[0]["state"] = "held"
        env.MessyDeskEnv._world_events(world, 0)
        self.assertEqual(world.events, [])
        world.request = {"id": 1, "object": 0, "status": "open", "deadline_t": 100}
        env.MessyDeskEnv._world_events(world, 0)
        self.assertEqual(world.events[0]["request"], 1)

    def test_first_hold_on_closing_step_is_not_dropped(self):
        world = self.world()
        env.MessyDeskEnv._world_events(world, 1)  # Still held: the delivery claim is wrong.
        self.assertEqual([e["event"] for e in world.events],
                         ["request_first_hold", "request_claim_wrong"])
        self.scorer.add_step({"objects": [], "events": world.events})
        self.assertEqual(self.scorer.request_points[0], 0)   # the hold's point, minus the wrong mark
        self.assertEqual(self.scorer.request_progress(), 0)

    def test_a_touch_is_not_a_hold(self):
        world = SimpleNamespace(t=10, request={"id": 0, "object": 0, "status": "open", "deadline_t": 100})
        world._is_target = lambda o: env.MessyDeskEnv._is_target(world, o)
        touched = {"id": 0, "cls": "A", "present": True, "removed": False, "contact": True, "held": False,
                   "resting": True, "in_container": None}
        self.assertEqual(env.MessyDeskEnv._value(world, touched), 0)
        lifted = dict(touched, held=True, resting=False)
        self.assertEqual(env.MessyDeskEnv._value(world, lifted), 1)
        in_pile = dict(touched, in_container="gray")             # touched where it lies: not placed, not held
        self.assertEqual(env.MessyDeskEnv._value(world, in_pile), 0)
        self.assertEqual(env.MessyDeskEnv._value(world, dict(in_pile, contact=False)), -1)

    def test_practice_and_hidden_copies_match(self):
        for hidden, practice in [("env.py", "messy_desk_env.py"),
                                 ("messy_desk_score.py", "messy_desk_score.py")]:
            self.assertEqual((TASK / "verify" / hidden).read_bytes(),
                             (TASK / "setup/payload" / practice).read_bytes())


if __name__ == "__main__":
    unittest.main()
