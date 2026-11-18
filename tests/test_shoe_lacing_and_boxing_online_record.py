"""shoe_lacing_and_boxing, online variant: tests/verify.py scores the graded service's record and nothing else.

The verifier runs no program (the agent played the graded episode during its session; the service's record directory
is a declared artifact restored into the separate verifier), so the offline variant's verify-start lockdown (and its
survivor test) has no subject here. What it must do instead: score the record's episode_score against the anchor when
the record is the graded episode's (format, task, episode and seed digest as tests/online.json, started), and give the
honest zero for a missing, foreign, malformed or never-started record.

Runs tests/verify.py as a host-side subprocess on a copy of tests/ (the grading kit is stdlib-only), with the record
directory (ARB_ONLINE_RECORD_DIR) and the verdict (ROBOTICS_GRADER_VERDICT_PATH) in a temporary folder.
"""

import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

TASK = Path(__file__).resolve().parents[1] / "tasks/arm/shoe_lacing_and_boxing"
ONLINE = json.loads((TASK / "tests/online.json").read_text())
ANCHOR = json.loads((TASK / "tests/anchor.json").read_text())


def record(**over):
    rec = {"format": ONLINE["format"], "task": ONLINE["task"], "episode": ONLINE["episode"],
           "seed_sha256": ONLINE["seed_sha256"], "started": True, "ended": True, "end_reason": "finish",
           "requests": 120, "steps": 80, "sim_time_s": 300.5, "wall_s_since_start": 1800.0, "episode_score": 0.56,
           "metrics": {"milestone_lacing_a": 1.0, "milestone_lacing_b": 0.0, "milestone_erected": 1.0,
                       "safety_ok": 1.0, "work_score": 0.56}}
    rec.update(over)
    return rec


class RecordScoring(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="shoe-online-"))
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.stage = self.tmp / "tests"
        shutil.copytree(TASK / "tests", self.stage, ignore=shutil.ignore_patterns("__pycache__", "verdict.local.json"))
        self.records = self.tmp / "arb-online"
        self.records.mkdir()

    def anchor(self, value):
        a = json.loads((self.stage / "anchor.json").read_text())
        a["value"] = value
        (self.stage / "anchor.json").write_text(json.dumps(a))

    def verdict(self, rec=None, raw=None):
        if rec is not None:
            (self.records / "record.json").write_text(json.dumps(rec))
        if raw is not None:
            (self.records / "record.json").write_bytes(raw)
        out = self.tmp / "verdict.json"
        env = dict(os.environ, ALE_STAGE_DIR=str(self.stage), ARB_ONLINE_RECORD_DIR=str(self.records),
                   ROBOTICS_GRADER_VERDICT_PATH=str(out))
        env.pop("ALE_VERIFY_PYTHON", None)
        r = subprocess.run([sys.executable, "-B", str(self.stage / "verify.py")], cwd=self.stage, env=env,
                           capture_output=True, text=True, timeout=120)
        self.assertEqual(r.returncode, 0, r.stderr)
        return json.loads(out.read_text())

    def test_no_record_scores_zero(self):
        self.anchor(1.0)
        v = self.verdict()
        self.assertEqual(v["rewards"]["reward"], 0.0)
        self.assertIn("never started", v["diagnostic"])
        self.assertEqual(v["metrics"]["record_present"], 0.0)

    def test_the_graded_episode_s_record_is_scored_against_the_anchor(self):
        self.anchor(1.0)
        v = self.verdict(record())
        self.assertAlmostEqual(v["rewards"]["reward"], min(1.0, 0.56 / 1.0 / ANCHOR["full_at"]))
        self.assertEqual(v["metrics"]["work_score"], 0.56)
        self.assertEqual(v["metrics"]["end_reason_finish"], 1.0)
        self.assertEqual(v["metrics"]["steps"], 80.0)
        self.assertEqual(v["metrics"]["episode_milestone_lacing_a"], 1.0)
        self.assertIn("graded episode %s: work_score 0.5600" % ONLINE["episode"], v["diagnostic"])

    def test_a_record_left_by_an_ended_session_scores_as_it_stood(self):
        self.anchor(1.0)
        v = self.verdict(record(ended=False, end_reason=None, episode_score=0.3))
        self.assertAlmostEqual(v["rewards"]["reward"], 0.3 / ANCHOR["full_at"])
        self.assertEqual(v["metrics"]["end_reason_none"], 1.0)

    def test_a_full_score_reads_one(self):
        self.anchor(1.0)
        self.assertEqual(self.verdict(record(episode_score=1.0))["rewards"]["reward"], 1.0)

    def test_foreign_or_malformed_records_score_zero(self):
        self.anchor(1.0)
        cases = [record(episode="ep02"), record(seed_sha256="0" * 64), record(task="other_task"),
                 record(format="arb-online-record/0"), record(started=False), record(episode_score=float("nan")),
                 record(episode_score=-0.1), record(episode_score=True), record(episode_score="0.9"),
                 {k: v for k, v in record().items() if k != "episode_score"}]
        for rec in cases:
            v = self.verdict(rec)
            self.assertEqual(v["rewards"]["reward"], 0.0, rec)
            self.assertTrue(v["diagnostic"].startswith("scoring 0:"), v)
        for raw in (b"{not json", b"[1, 2]", b" " * (1 << 20) + b"{}"):
            self.assertEqual(self.verdict(raw=raw)["rewards"]["reward"], 0.0)

    def test_a_symlinked_record_scores_zero(self):
        self.anchor(1.0)
        target = self.tmp / "elsewhere.json"
        target.write_text(json.dumps(record(episode_score=1.0)))
        (self.records / "record.json").symlink_to(target)
        self.assertEqual(self.verdict()["rewards"]["reward"], 0.0)

    def test_the_bootstrap_run_reports_the_measured_value(self):
        self.anchor(None)
        v = self.verdict(record(episode_score=1.0))
        self.assertEqual(v["rewards"]["reward"], 0.0)
        self.assertEqual(v["metrics"]["work_score"], 1.0)
        self.assertEqual(v["metrics"]["anchor_recorded"], 0.0)

    def test_every_metric_is_a_finite_number(self):
        self.anchor(1.0)
        v = self.verdict(record(metrics={"a": float("inf"), "b": "text", "c": 2, "work_score": 9.0}))
        self.assertTrue(all(isinstance(x, float) and math.isfinite(x) for x in v["metrics"].values()))
        self.assertEqual(v["metrics"]["work_score"], 0.56)
        self.assertEqual(v["metrics"]["episode_c"], 2.0)


if __name__ == "__main__":
    unittest.main()
