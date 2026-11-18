"""shoe_lacing_and_boxing, dual mode: the graded episode's record, the freeze of the program at the start, and how the
verifier uses both.

tests/verify.py runs main's grading of the program (offline_verify.py, its own tests) and scores the graded episode's
record. What is tested here, host-side and stdlib only (the verifier's record and freeze functions import nothing else;
the service's freeze is environment/setup/online/server.py's snapshot_submission, run on temporary folders):
  * the record: the graded episode's and usable, or the honest zero with the reason (missing, foreign, malformed,
    never started, not a regular file, too large);
  * the online item: the record's episode_score against anchor_online.json;
  * the freeze: the service copies the submission folder the way main's grader snapshots it (regular files and folders
    only; links, special files and __pycache__ dropped; no link followed), refuses over the 60 MB cap with no copy, and
    its digest is the one the verifier recomputes;
  * the program graded: the frozen copy replaces the restored submission when the episode started (a copy that differs
    from the record is a task error), an empty folder when it was too large, the restored submission when the episode
    never started.
"""

import importlib.util
import json
import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

TASK = Path(__file__).resolve().parents[1] / "tasks/arm/shoe_lacing_and_boxing"
ONLINE = json.loads((TASK / "tests/online.json").read_text())
ANCHOR_ONLINE = json.loads((TASK / "tests/anchor_online.json").read_text())


def _load(name, path):
    saved = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    finally:
        sys.dont_write_bytecode = saved


SERVER = _load("shoe_online_server", TASK / "environment/setup/online/server.py")


def record(**over):
    rec = {"format": ONLINE["format"], "task": ONLINE["task"], "episode": ONLINE["episode"],
           "seed_sha256": ONLINE["seed_sha256"], "started": True, "ended": True, "end_reason": "finish",
           "requests": 120, "steps": 80, "sim_time_s": 300.5, "wall_s_since_start": 1800.0, "episode_score": 0.56,
           "metrics": {"milestone_lacing_a": 1.0, "milestone_erected": 1.0, "safety_ok": 1.0, "work_score": 0.56},
           "snapshot_status": "ok", "snapshot_files": 0, "snapshot_bytes": 0, "snapshot_sha256": None}
    rec.update(over)
    return rec


class _Case(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="shoe-dual-"))
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.stage = self.tmp / "tests"
        shutil.copytree(TASK / "tests", self.stage, ignore=shutil.ignore_patterns("__pycache__", "verdict.local.json"))
        self.records = self.tmp / "arb-online"
        self.records.mkdir()
        self.home = self.tmp / "home" / "user"
        (self.home / "submission").mkdir(parents=True)
        env = {"ALE_STAGE_DIR": str(self.stage), "ARB_ONLINE_RECORD_DIR": str(self.records), "ALE_HOME": str(self.home)}
        saved = {k: os.environ.get(k) for k in env}
        os.environ.update(env)
        self.addCleanup(lambda: [os.environ.pop(k, None) if v is None else os.environ.__setitem__(k, v)
                                 for k, v in saved.items()])
        saved_path = list(sys.path)
        self.addCleanup(lambda: sys.path.__setitem__(slice(None), saved_path))
        for m in [m for m in sys.modules if m == "robotics_grader" or m.startswith(("robotics_grader.", "cellkit"))]:
            del sys.modules[m]
        self.verify = _load("shoe_dual_verify", self.stage / "verify.py")

    def write_record(self, rec):
        (self.records / "record.json").write_text(json.dumps(rec))

    def freeze(self, src, max_bytes=60 << 20):
        return SERVER.snapshot_submission(str(src), str(self.records / "submission_snapshot"), max_bytes, 50000, 32)


class RecordTests(_Case):
    def test_no_record_is_the_honest_zero(self):
        rec, why = self.verify.load_record(ONLINE)
        self.assertIsNone(rec)
        self.assertIn("never started", why)
        reward, stats, _ = self.verify.online_item(rec, why)
        self.assertEqual(reward, 0.0)
        self.assertEqual(stats["online_record_present"], 0.0)

    def test_the_graded_episode_s_record_is_scored_against_its_anchor(self):
        self.write_record(record())
        rec, why = self.verify.load_record(ONLINE)
        self.assertIsNone(why)
        reward, stats, text = self.verify.online_item(rec, why)
        self.assertAlmostEqual(reward, min(1.0, 0.56 / ANCHOR_ONLINE["value"] / ANCHOR_ONLINE["full_at"]))
        self.assertEqual(stats["online_work_score"], 0.56)
        self.assertEqual(stats["online_end_reason_finish"], 1.0)
        self.assertIn("graded episode %s: work_score 0.5600" % ONLINE["episode"], text)

    def test_foreign_or_malformed_records_are_refused(self):
        for rec in (record(episode="ep02"), record(seed_sha256="0" * 64), record(task="other"),
                    record(format="arb-online-record/0"), record(started=False), record(episode_score=float("nan")),
                    record(episode_score=-0.1), record(episode_score=True), record(episode_score="0.9")):
            self.write_record(rec)
            self.assertIsNone(self.verify.load_record(ONLINE)[0], rec)
        for raw in (b"{not json", b"[1, 2]", b" " * (1 << 20) + b"{}"):
            (self.records / "record.json").write_bytes(raw)
            self.assertIsNone(self.verify.load_record(ONLINE)[0])
        (self.records / "record.json").unlink()
        target = self.tmp / "elsewhere.json"
        target.write_text(json.dumps(record()))
        (self.records / "record.json").symlink_to(target)
        self.assertIsNone(self.verify.load_record(ONLINE)[0])


class FreezeTests(_Case):
    def program(self, root, text="def solve(cell):\n    pass\n"):
        (root / "pkg").mkdir(parents=True, exist_ok=True)
        (root / "packer.py").write_text(text)
        (root / ".oracle_token").write_text("t")
        (root / "pkg" / "helper.py").write_text("X = 1\n")
        (root / "__pycache__").mkdir(exist_ok=True)
        (root / "__pycache__" / "packer.cpython-312.pyc").write_bytes(b"\0" * 10)
        (root / "link_to_secret").symlink_to(self.tmp / "secret.txt")
        (self.tmp / "secret.txt").write_text("hidden")
        (root / "linked_dir").symlink_to(self.tmp)
        os.mkfifo(str(root / "a_pipe"))

    def test_the_freeze_copies_as_main_s_snapshot_does_and_the_verifier_s_digest_agrees(self):
        sub = self.home / "submission"
        self.program(sub)
        res = self.freeze(sub)
        self.assertEqual(res["status"], "ok", res)
        frozen = self.records / "submission_snapshot"
        names = sorted(str(p.relative_to(frozen)) for p in frozen.rglob("*"))
        self.assertEqual(names, [".oracle_token", "packer.py", "pkg", "pkg/helper.py"])   # no link, pipe or __pycache__
        self.assertEqual((res["files"], res["skipped"]), (3, 4))
        self.assertEqual(oct(frozen.stat().st_mode & 0o777), "0o700")
        self.assertEqual(oct((frozen / "packer.py").stat().st_mode & 0o777), "0o600")
        self.assertEqual(self.verify.tree_digest(str(frozen)), (res["sha256"], res["files"], res["bytes"]))

    def test_over_the_cap_there_is_no_copy(self):
        sub = self.home / "submission"
        (sub / "big.bin").write_bytes(b"x" * 4096)
        res = self.freeze(sub, max_bytes=1000)
        self.assertEqual(res["status"], "too_large")
        self.assertFalse((self.records / "submission_snapshot").exists())

    def test_a_missing_folder_is_an_empty_program_and_a_planted_link_is_not_followed(self):
        res = self.freeze(self.home / "nothing_here")
        self.assertEqual((res["status"], res["files"]), ("ok", 0))
        dst = self.records / "submission_snapshot"
        shutil.rmtree(dst)
        dst.symlink_to(self.tmp)                   # a link where the copy goes: removed, never written through
        res = self.freeze(self.home / "submission")
        self.assertEqual(res["status"], "ok", res)
        self.assertFalse(dst.is_symlink())
        self.assertTrue((self.tmp / "tests").is_dir())

    def test_the_program_graded_is_the_frozen_copy_when_the_episode_started(self):
        sub = self.home / "submission"
        self.program(sub, "def solve(cell):\n    print('PROGRAM-A')\n")
        res = self.freeze(sub)
        (sub / "packer.py").write_text("def solve(cell):\n    print('PROGRAM-B')\n")   # changed after the start
        self.write_record(record(snapshot_files=res["files"], snapshot_bytes=res["bytes"], snapshot_sha256=res["sha256"]))
        rec, _ = self.verify.load_record(ONLINE)
        label, _ = self.verify.program_graded(rec)
        self.assertEqual(label, "frozen_at_start")
        self.assertIn("PROGRAM-A", (sub / "packer.py").read_text())
        self.assertFalse((sub / "link_to_secret").exists())

    def test_a_frozen_copy_that_differs_from_the_record_is_a_task_error(self):
        sub = self.home / "submission"
        self.program(sub)
        res = self.freeze(sub)
        (self.records / "submission_snapshot" / "packer.py").write_text("changed")
        self.write_record(record(snapshot_files=res["files"], snapshot_bytes=res["bytes"], snapshot_sha256=res["sha256"]))
        rec, _ = self.verify.load_record(ONLINE)
        with self.assertRaises(SystemExit):
            self.verify.program_graded(rec)

    def test_too_large_grades_nothing_and_never_started_grades_the_session_end(self):
        sub = self.home / "submission"
        (sub / "packer.py").write_text("def solve(cell):\n    pass\n")
        self.write_record(record(snapshot_status="too_large"))
        rec, _ = self.verify.load_record(ONLINE)
        self.assertEqual(self.verify.program_graded(rec)[0], "too_large")
        self.assertEqual(list(sub.iterdir()), [])
        (sub / "packer.py").write_text("def solve(cell):\n    pass\n")
        self.assertEqual(self.verify.program_graded(None)[0], "session_end")
        self.assertTrue((sub / "packer.py").exists())


if __name__ == "__main__":
    unittest.main()
