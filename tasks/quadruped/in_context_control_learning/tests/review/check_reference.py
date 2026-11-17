"""Stdlib-only author-runner regressions; no rendering or simulation rollouts."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

HERE = Path(__file__).resolve().parent


class FakeWorld:
    instances = []

    def __init__(self, seed, render=False):
        self.seed, self.closed = seed, False
        self.instances.append(self)

    def metrics(self):
        return {"delivered": 6, "deliveries": 6, "delivered_fraction": 1.0, "steps": 1}

    def state_fingerprint(self):
        return f"state-{self.seed}"

    def close(self):
        self.closed = True


class FakeSession:
    def __init__(self, world):
        self.events = []

    def call(self, method, params):
        self.events.append({"method": method, "params": params})


class FakeController:
    def __init__(self, world, call):
        pass

    def solve(self):
        pass


world = types.ModuleType("world")
world.KIND, world.VERSION, world.World = "icl", 3, FakeWorld
privileged = types.ModuleType("privileged")
privileged.Controller, privileged.Session = FakeController, FakeSession
grader = types.ModuleType("grader")
grader.replay = None
spec = importlib.util.spec_from_file_location("reference_under_test", HERE/"privileged_reference.py")
reference = importlib.util.module_from_spec(spec)
with mock.patch.dict(sys.modules, {"world": world, "privileged": privileged, "grader": grader}):
    spec.loader.exec_module(reference)


class ReferenceTests(unittest.TestCase):
    def setUp(self):
        FakeWorld.instances.clear()

    def test_replay_system_exit_is_persisted_as_failure(self):
        for code in (1, 0, "replay divergence"):
            with self.subTest(code=code), tempfile.TemporaryDirectory() as tmp:
                output = Path(tmp)
                with mock.patch.object(reference, "replay", side_effect=SystemExit(code)), \
                        contextlib.redirect_stdout(io.StringIO()):
                    row = reference.run(11, 0, output)
                self.assertFalse(row["replay_ok"])
                self.assertIn(f"SystemExit: {code}", row["error"])
                self.assertEqual(json.loads((output/"11.result.json").read_text()), row)
                self.assertTrue((output/"11.ledger.json").is_file())
                self.assertTrue(FakeWorld.instances[-1].closed)

    def test_failed_first_seed_does_not_abort_fixed_cohort(self):
        seeds = json.loads((reference.TASK/"tests/hidden_scenes.json").read_text())["seeds"]

        def replay(ledger):
            if ledger["seed"] == seeds[0]:
                raise SystemExit("injected replay divergence")
            return ledger["metrics"]

        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)/"cohort"
            argv = ["privileged_reference.py", "--all-hidden", "--output", str(output)]
            with mock.patch.object(sys, "argv", argv), mock.patch.object(reference, "replay", side_effect=replay) as replay_mock, \
                    contextlib.redirect_stdout(io.StringIO()), self.assertRaises(SystemExit) as exc:
                reference.main()
            self.assertEqual(exc.exception.code, 1)
            self.assertEqual(replay_mock.call_count, len(seeds))
            summary = json.loads((output/"summary.json").read_text())
            self.assertFalse(summary["passed"])
            self.assertEqual([row["seed"] for row in summary["runs"]], seeds)
            self.assertFalse(summary["runs"][0]["replay_ok"])
            self.assertIn("SystemExit: injected replay divergence", summary["runs"][0]["error"])
            self.assertTrue(all(row["replay_ok"] and row["error"] is None for row in summary["runs"][1:]))
            for seed in seeds:
                self.assertTrue((output/f"{seed}.ledger.json").is_file())
                self.assertTrue((output/f"{seed}.result.json").is_file())
            self.assertTrue((output/"receipt.json").is_file())
            self.assertTrue(all(w.closed for w in FakeWorld.instances))

    def test_keyboard_interrupt_is_not_swallowed(self):
        with tempfile.TemporaryDirectory() as tmp, \
                mock.patch.object(reference, "replay", side_effect=KeyboardInterrupt), \
                self.assertRaises(KeyboardInterrupt):
            reference.run(11, 0, Path(tmp))
        self.assertTrue(FakeWorld.instances[-1].closed)


if __name__ == "__main__":
    unittest.main(verbosity=2)
