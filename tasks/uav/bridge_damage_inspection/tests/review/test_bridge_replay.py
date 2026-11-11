"""Integrity regressions for the isolated verifier, using a recorded reference path."""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

from PIL import Image

TASK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TASK / "tests"))
from bridge_grader import replay
from world import World


class ReplayContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = tempfile.TemporaryDirectory()
        cls.evidence = Path(cls.fixture.name)
        world = World("bridge", 101, render=False)
        events = []
        arguments = {"observe": (), "act": ("action",),
                     "report": ("frame_id", "point"), "advance": ()}
        try:
            recorded = json.loads((TASK / "solution/success-interaction.json").read_text())
            for step in recorded:
                method = step["method"]
                params = dict(zip(arguments[method], step["arguments"]))
                result = getattr(world, method)(**params)
                if result["frame_id"] != step["frame_id"]:
                    raise AssertionError("reference path no longer replays")
                event = {"method": method, "params": params, "frame_id": result["frame_id"]}
                if method == "report":
                    path = cls.evidence / f'{params["frame_id"]:04d}-0.png'
                    # Synthetic unit fixtures test byte integrity only. Real rendered
                    # evidence is exercised separately by the ALE oracle validation.
                    Image.new("RGB", (384, 384), (params["frame_id"] % 256, 0, 0)).save(path)
                    event["evidence_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
                events.append(event)
            cls.state = {"kind": "bridge", "seed": 101, "events": events,
                         "metrics": world.metrics(), "requests": len(events)}
            if cls.state["metrics"]["inspection_quality"] != 1:
                raise AssertionError("reference path must solve the task")
        finally:
            world.close()

    @classmethod
    def tearDownClass(cls):
        cls.fixture.cleanup()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.files = Path(self.temp.name)
        shutil.copytree(self.evidence, self.files, dirs_exist_ok=True)
        self.ledger = copy.deepcopy(self.state)
        self.report = next(e for e in self.ledger["events"] if e["method"] == "report")
        self.frame = self.files / f'{self.report["params"]["frame_id"]:04d}-0.png'

    def assertRejected(self):
        with self.assertRaises(SystemExit):
            replay(self.ledger, self.files)

    def test_reference_actions_recompute_full_score(self):
        self.assertEqual(replay(self.ledger, self.files)["inspection_quality"], 1)

    def test_changed_metrics_rejected(self):
        self.ledger["metrics"]["steps"] += 1
        self.assertRejected()

    def test_deleted_action_rejected(self):
        del self.ledger["events"][1]
        self.assertRejected()

    def test_fake_error_cannot_hide_valid_action(self):
        self.ledger["events"][1]["error"] = "pretend the move never happened"
        self.assertRejected()

    def test_missing_evidence_rejected(self):
        self.frame.unlink()
        self.assertRejected()

    def test_changed_evidence_bytes_rejected(self):
        self.frame.write_bytes(self.frame.read_bytes() + b"modified")
        self.assertRejected()

    def test_missing_evidence_hash_rejected(self):
        del self.report["evidence_sha256"]
        self.assertRejected()

    def test_symlink_evidence_rejected(self):
        target = self.frame.with_suffix(".original")
        self.frame.rename(target)
        self.frame.symlink_to(target)
        self.assertRejected()

    def test_missing_finalization_rejected(self):
        self.ledger["events"].pop()
        self.assertRejected()


if __name__ == "__main__":
    unittest.main()
