"""lab_sample_handling, dual mode (experiment, branch exp/dual-mode-scoring): the controller item and the graded episode.

The controller item is main's grading, unchanged (tests/offline_verify.py is main's tests/verify.py, its anchor
tests/anchor_offline.json main's tests/anchor.json); the online item is one episode of the hidden set, served live by
environment/setup/online/graded_service.py on the grader's own simulator, which also freezes the submission folder at
the graded start (or at the close of the start window); tests/verify.py grades both. These checks keep the pieces
consistent and run the freeze and the verifier's freeze decision host-side.

Standard library only (the host-side suite has no numpy): the service's freeze and the verifier's decision are
imported and run on temporary folders; nothing of the simulator is imported.
"""

import hashlib
import importlib.util
import json
import os
import re
import shutil
import stat
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TASK = ROOT / "tasks/arm/lab_sample_handling"
ONLINE_SETUP = TASK / "environment/setup/online"
SERVICE = json.loads((ONLINE_SETUP / "service.json").read_text())
EPISODE = json.loads((ONLINE_SETUP / "episode.json").read_text())
ONLINE = json.loads((TASK / "tests/online.json").read_text())
INSTRUCTION = (TASK / "instruction.md").read_text()


def flat(text):
    return " ".join(text.split())


def load(path, name):
    """Import a task file as a module without leaving bytecode in the task folder."""
    saved = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec = importlib.util.spec_from_file_location(name, str(path))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        sys.dont_write_bytecode = saved
    return mod


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Pieces(unittest.TestCase):
    def test_the_service_runs_the_graders_simulator(self):
        self.assertEqual((ONLINE_SETUP / "episode.py").read_bytes(), (TASK / "tests/episode.py").read_bytes())
        self.assertEqual(sorted(SERVICE["sim_sha256"]), ["box_transport_env.py", "episode.py", "lab_env.py",
                                                         "lab_look.json", "lab_render.py", "lab_sim.py"])
        for name, digest in SERVICE["sim_sha256"].items():
            self.assertEqual(digest, sha256(TASK / "tests" / name), name)
            if name != "episode.py":            # setup.sh copies these from the image's commissioning folder
                self.assertEqual(digest, sha256(TASK / "environment/commission_service" / name), name)
        setup = (TASK / "environment/setup/setup.sh").read_text()
        self.assertIn("for f in lab_sim.py lab_env.py box_transport_env.py lab_render.py lab_look.json; do", setup)
        self.assertIn('cp "$GRADED/episode.py" "$GRADED/sim/episode.py"', setup)

    def test_the_graded_episode_is_one_of_the_hidden_set(self):
        eps = json.loads((TASK / "tests/hidden/episodes.json").read_text())["episodes"]
        units = json.loads((TASK / "tests/units.json").read_text())["units"]
        entry = eps[EPISODE["index"]]
        self.assertEqual((entry["unit"], int(entry["seed"])), (EPISODE["unit"], EPISODE["seed"]))
        self.assertEqual(EPISODE["unit_props"], units[EPISODE["unit"]])
        self.assertEqual(EPISODE["seed_sha256"], hashlib.sha256(str(EPISODE["seed"]).encode()).hexdigest())
        self.assertEqual(EPISODE["episode"], "hidden-%02d" % EPISODE["index"])
        for key in ("episode", "unit", "seed_sha256"):
            self.assertEqual(ONLINE[key], EPISODE[key], key)
        self.assertEqual(ONLINE["task"], SERVICE["task"])
        self.assertEqual(EPISODE["oracle_token_sha256"], (TASK / "tests/hidden/oracle_token.sha256").read_text().strip())
        self.assertEqual(hashlib.sha256((TASK / "solution/oracle_token").read_text().strip().encode()).hexdigest(),
                         EPISODE["oracle_token_sha256"])
        draw = ONLINE["draw"]
        self.assertIn(sha256(TASK / "tests/hidden/episodes.json"), draw["from"])
        self.assertIn("secrets.randbelow(12)", draw["how"])
        # the seed is in no file the agent's image or home gets
        for p in list((TASK / "environment").rglob("*")):
            if p.is_file() and "online" not in p.relative_to(TASK / "environment").parts:
                self.assertNotIn(str(EPISODE["seed"]), p.read_text(errors="replace"), str(p))

    def test_anchors(self):
        main_anchor = json.loads((TASK / "tests/anchor_offline.json").read_text())
        online = json.loads((TASK / "tests/anchor_online.json").read_text())
        dual = json.loads((TASK / "tests/anchor.json").read_text())
        self.assertEqual((main_anchor["metric"], main_anchor["direction"], main_anchor["value"], main_anchor["full_at"]),
                         ("mean_episode_credit", "higher", 1.0, 0.98))
        self.assertEqual((online["metric"], online["direction"], online["full_at"]), ("mean_episode_credit", "higher",
                                                                                     main_anchor["full_at"]))
        self.assertEqual((dual["metric"], dual["direction"], dual["value"], dual["full_at"]),
                         ("dual_mode_score", "higher", 1.0, 1.0))
        src = (TASK / "tests/verify.py").read_text()
        self.assertIn('read_anchor("anchor_offline.json")', src)
        self.assertIn('read_anchor("anchor_online.json")', src)
        self.assertIn("import offline_verify as OV", src)
        self.assertIn("combined = (off_r + on_r) / 2.0", src)

    def test_task_toml(self):
        toml = (TASK / "task.toml").read_text()
        self.assertIn('artifacts = ["/home/user/submission", "/var/lib/arb-online"]', toml)
        agent = toml[toml.index("[agent]"):toml.index("[verifier]")]
        self.assertEqual(int(re.search(r"^timeout_sec\s*=\s*(\d+)", agent, re.M).group(1)),
                         SERVICE["prep_s"] + SERVICE["graded_s"])
        verifier = toml[toml.index("[verifier]"):]
        self.assertEqual(int(re.search(r"^timeout_sec\s*=\s*(\d+)", verifier, re.M).group(1)), 27000)   # main's
        self.assertEqual((SERVICE["prep_s"], SERVICE["graded_s"]), (ONLINE["prep_s"], ONLINE["graded_s"]))

    def test_the_commissioning_service_closes_at_the_graded_start(self):
        src = (TASK / "environment/commission_service/commissiond.py").read_text()
        self.assertIn('GRADED_FLAG = "%s"' % SERVICE["practice_flag"], src)
        self.assertEqual(os.path.dirname(os.path.dirname(SERVICE["practice_flag"])), os.path.dirname(SERVICE["socket"]))
        self.assertIn('self.send(rid, error="commissioning_closed", detail=CLOSED)', src)
        self.assertIn("self.end_bench()                # the graded episode started while this bench waited (O13)", src)
        client = (TASK / "environment/setup/payload/graded/client.py").read_text()
        self.assertIn('SOCK = "%s"' % SERVICE["socket"], client)
        self.assertNotIn("token", client)                            # the agent's client has no token option

    def test_the_instruction_states_both_items(self):
        text = flat(INSTRUCTION)
        for words in ("The score is the mean of two items, each relative to its own hidden reference value",
                      "The controller graded is the one in the submission folder at the earlier of two moments",
                      "over 5000 files or 200 MiB nothing is copied",
                      "The folder as it is when your session ends is graded only if the session ends before either "
                      "moment",
                      "Without a start the graded episode scores 0",
                      "**When the graded episode starts** the service closes for the rest of the session",
                      "`commissioning_closed`", "`prep_window_closed`",
                      "It is one of the 12 hidden episodes your controller is graded on",
                      "a malformed action is refused (`malformed_action`) and simulates nothing",
                      "a connection silent for 600 s is closed (the episode stays)",
                      "It is then credited as it stands: nothing is simulated after the end",
                      "If your session ends first, the episode counts as it stood after your last step",
                      "the graded episode has none of them"):
            self.assertIn(words, text)
        for false in ("is graded as it is", "not graded", "scores nothing", "late finish", "You have 2 hours of "
                      "wall-clock time"):
            self.assertNotIn(false, text)
        self.assertIn("idle_s", SERVICE)
        self.assertEqual(SERVICE["idle_s"], 600)
        self.assertEqual((SERVICE["snapshot_max_files"], SERVICE["snapshot_max_bytes"]), (5000, 200 * 1024 * 1024))
        main_grader = (TASK / "tests/offline_verify.py").read_text()
        self.assertIn("MAX_SUBMISSION_BYTES = 200 * 1024 * 1024", main_grader)
        self.assertIn("MAX_SUBMISSION_FILES = 5000", main_grader)


class Freeze(unittest.TestCase):
    """The service's freeze (take_snapshot, run by its root supervisor) and the verifier's use of it, host-side as the
    current user (who stands in for the agent's account)."""

    @classmethod
    def setUpClass(cls):
        cls.svc = load(ONLINE_SETUP / "graded_service.py", "lab_graded_service_under_test")
        sys.path.insert(0, str(TASK / "tests"))
        try:
            cls.ver = load(TASK / "tests/verify.py", "lab_dual_verify_under_test")
        finally:
            sys.path.remove(str(TASK / "tests"))

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.sub = os.path.join(self.tmp, "home", "submission")
        self.rec = os.path.join(self.tmp, "record")
        os.makedirs(self.sub)
        os.makedirs(self.rec)
        self.cfg = dict(self.svc.DEFAULTS, submission_dir=self.sub, record_dir=self.rec)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write(self, rel, data=b"x"):
        p = os.path.join(self.sub, rel)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "wb") as f:
            f.write(data)
        return p

    def test_copies_files_and_folders_skips_links_and_matches_the_verifier(self):
        self.write("run.sh", b"#!/usr/bin/env bash\necho READY\n")
        os.chmod(os.path.join(self.sub, "run.sh"), 0o755)
        self.write("pkg/a.py", b"print(1)\n")
        self.write(".hidden", b"h")
        outside = os.path.join(self.tmp, "secret")
        with open(outside, "w") as f:
            f.write("secret")
        os.symlink(outside, os.path.join(self.sub, "link"))
        os.symlink(self.tmp, os.path.join(self.sub, "dirlink"))
        os.mkfifo(os.path.join(self.sub, "fifo"))
        res = self.svc.take_snapshot(self.cfg, os.getuid())
        self.assertEqual(res["status"], "ok")
        self.assertEqual((res["files"], res["skipped"]), (3, 3))
        snap = os.path.join(self.rec, "submission_snapshot")
        self.assertEqual(sorted(os.listdir(snap)), [".hidden", "pkg", "run.sh"])
        self.assertTrue(os.stat(os.path.join(snap, "run.sh")).st_mode & stat.S_IXUSR)
        files, size, digest, odd = self.ver.snapshot_digest(snap)
        self.assertEqual((files, size, digest, odd), (res["files"], res["bytes"], res["sha256"], 0))
        fields = {"snapshot_status": "ok", "snapshot_files": files, "snapshot_bytes": size, "snapshot_sha256": digest}
        self.assertIsNone(self.ver.check_snapshot(fields, root=snap))
        # an edit after the freeze does not reach the frozen copy
        self.write("pkg/a.py", b"print(2)\n")
        self.assertEqual(Path(snap, "pkg/a.py").read_bytes(), b"print(1)\n")
        dest = os.path.join(self.tmp, "restored")
        os.makedirs(dest)
        Path(dest, "late.py").write_text("late")
        self.ver.install_snapshot(snap, dest)
        self.assertEqual(sorted(os.listdir(dest)), [".hidden", "pkg", "run.sh"])
        # a tampered copy does not match its record
        Path(snap, "pkg/a.py").write_bytes(b"print(3)\n")
        self.assertIsNotNone(self.ver.check_snapshot(fields, root=snap))

    def test_main_caps(self):
        for i in range(5):
            self.write("f%d" % i)
        self.assertEqual(self.svc.take_snapshot(dict(self.cfg, snapshot_max_files=4), os.getuid())["status"],
                         "too_large")
        self.assertFalse(os.path.exists(os.path.join(self.rec, "submission_snapshot")))
        self.assertEqual(self.svc.take_snapshot(dict(self.cfg, snapshot_max_files=5), os.getuid())["status"], "ok")
        os.symlink("/nonexistent", os.path.join(self.sub, "l"))     # a link counts as a file, as main counts it
        self.assertEqual(self.svc.take_snapshot(dict(self.cfg, snapshot_max_files=5), os.getuid())["status"],
                         "too_large")
        self.write("big", b"y" * 1000)
        self.assertEqual(self.svc.take_snapshot(dict(self.cfg, snapshot_max_bytes=999), os.getuid())["status"],
                         "too_large")

    def test_a_folder_of_another_account_or_a_missing_one_is_missing(self):
        shutil.rmtree(self.sub)
        self.assertEqual(self.svc.take_snapshot(self.cfg, os.getuid())["status"], "missing")
        os.makedirs(self.sub)
        self.assertEqual(self.svc.take_snapshot(self.cfg, os.getuid() + 1)["status"], "missing")

    def test_the_verifier_grades_what_was_frozen(self):
        base = {"format": ONLINE["format"], "task": ONLINE["task"], "episode": ONLINE["episode"],
                "seed_sha256": ONLINE["seed_sha256"]}
        snap = {"snapshot_status": "ok", "snapshot_files": 1, "snapshot_bytes": 2, "snapshot_sha256": "d"}
        decide = self.ver.freeze_decision
        self.assertEqual(decide(None, None, ONLINE), (None, {}))                      # untouched: session end
        state = {"service_started": 1.0, "started": False}
        self.assertEqual(decide(None, state, ONLINE), (None, {}))
        trig, fields = decide(dict(base, started=True, **snap), None, ONLINE)
        self.assertEqual((trig, fields["snapshot_status"]), ("start", "ok"))
        trig, fields = decide(dict(base, started=False, freeze="prep_window_closed", **snap), None, ONLINE)
        self.assertEqual(trig, "prep_window")
        trig, fields = decide(dict(base, started=False, freeze="prep_window_closed", snapshot_status="failed"), None,
                              ONLINE)
        self.assertEqual(trig, "prep_window")
        self.assertIsNotNone(self.ver.check_snapshot(fields))                   # a failed freeze: no fallback
        trig, fields = decide(None, {"started": True, "snapshot": {"status": "too_large"}}, ONLINE)
        self.assertEqual((trig, fields["snapshot_status"]), ("start", "too_large"))
        self.assertIn("caps", self.ver.check_snapshot(fields))
        other = dict(base, episode="hidden-00", started=True, **snap)                  # not this task's episode
        self.assertEqual(decide(other, None, ONLINE), (None, {}))
        self.assertIsNotNone(self.ver.check_record(other, ONLINE))
        good = dict(base, started=True, episode_score=0.67)
        self.assertIsNone(self.ver.check_record(good, ONLINE))
        for bad in (dict(good, started=False), dict(good, episode_score=1.5), dict(good, episode_score=True),
                    dict(good, task="other")):
            self.assertIsNotNone(self.ver.check_record(bad, ONLINE))


if __name__ == "__main__":
    unittest.main()
