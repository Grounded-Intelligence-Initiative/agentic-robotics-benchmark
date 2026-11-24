"""longtail_manipulation_recovery, dual mode (experiment, branch exp/dual-mode-scoring): the controller item and the
graded episode.

The controller item is main's grading, unchanged (tests/offline_verify.py is main's tests/verify.py, its anchor
tests/anchor_offline.json main's tests/anchor.json); the online item is one failure state of the hidden set, served
live by environment/setup/online/graded_service.py with the grader's own kitchen process (byte copies of
tests/sim_worker.py, kitchen_sim.py and wire_rules.py) on a second RoboCasa tree with the graded kitchens, which also
freezes the submission folder at the graded start (or at the close of the start window); tests/verify.py grades both.
These checks keep the pieces consistent and run the freeze and the verifier's freeze decision host-side.

Standard library only (the host-side suite has no numpy): the service's freeze and the verifier's decision are
imported and run on temporary folders; nothing of the simulator is imported.
"""

import hashlib
import importlib.util
import json
import os
import re
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TASK = ROOT / "tasks/arm/longtail_manipulation_recovery"
ONLINE_SETUP = TASK / "environment/setup/online"
SERVICE = json.loads((ONLINE_SETUP / "service.json").read_text())
EPISODE = json.loads((ONLINE_SETUP / "episode.json").read_text())
ONLINE = json.loads((TASK / "tests/online.json").read_text())
INSTRUCTION = (TASK / "instruction.md").read_text()


def flat(text):
    return " ".join(text.split())


def load(path, name, folder=None):
    """Import a task file as a module (its folder importable while it loads) without leaving bytecode behind."""
    saved, before = sys.dont_write_bytecode, list(sys.path)
    sys.dont_write_bytecode = True
    if folder is not None:
        sys.path.insert(0, str(folder))
    try:
        spec = importlib.util.spec_from_file_location(name, str(path))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        sys.dont_write_bytecode = saved
        sys.path[:] = before
    return mod


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Pieces(unittest.TestCase):
    def test_the_service_runs_the_graders_kitchen_on_the_graded_state(self):
        for name, digest in SERVICE["code_sha256"].items():
            self.assertEqual((ONLINE_SETUP / "code" / name).read_bytes(), (TASK / "tests" / name).read_bytes(), name)
            self.assertEqual(digest, sha256(TASK / "tests" / name), name)
        st = EPISODE["state"]
        self.assertEqual(sorted(SERVICE["state_sha256"]), sorted((st["prefix"], st["qpos"], st["card"])))
        for name, digest in SERVICE["state_sha256"].items():
            self.assertEqual((ONLINE_SETUP / "state" / name).read_bytes(), (TASK / "tests/hidden" / name).read_bytes())
            self.assertEqual(digest, sha256(TASK / "tests/hidden" / name), name)
        self.assertEqual(sorted(p.name for p in (ONLINE_SETUP / "state").iterdir()), sorted(SERVICE["state_sha256"]))
        verify = (TASK / "tests/offline_verify.py").read_text()
        env = {k: v for k, v in SERVICE["sim_env"].items()}
        for key, value in env.items():                       # the grader's own environment for its kitchen process
            self.assertIn('"%s": "%s"' % (key, value), verify.replace("\"\n            \"", ""), key)

    def test_the_graded_kitchens_tree_is_the_verifiers(self):
        self.assertEqual((TASK / "environment/graded_assets.sha256").read_bytes(),
                         (TASK / "tests/assets.sha256").read_bytes())
        self.assertEqual((TASK / "environment/verifier_assets.files.sha256").read_bytes(),
                         (TASK / "tests/verifier_assets.files.sha256").read_bytes())
        docker, verifier = (TASK / "environment/Dockerfile").read_text(), (TASK / "tests/Dockerfile").read_text()
        commit = "robocasa/robocasa/archive/456174f62b89b8fca99eaaf33949c29fec9cfc2a.tar.gz"
        self.assertIn(commit, verifier)
        self.assertEqual(docker.count(commit), 2)            # the practice tree's and the graded tree's
        for line in ("chown -R longgrade:longgrade /opt/longtail-graded && chmod -R u+rwX,go-rwx /opt/longtail-graded",
                     "! runuser -u user -- ls /opt/longtail-graded",
                     "/opt/task-assets/make_stubs.py /opt/task-assets/object_instances.txt "
                     "/opt/longtail-graded/robocasa/robocasa/models/assets/objects"):
            self.assertIn(line, docker)
        self.assertEqual(SERVICE["robocasa_dir"], "/opt/longtail-graded/robocasa")

    def test_the_graded_episode_is_one_of_the_hidden_set(self):
        spec = json.loads((TASK / "tests/hidden/states.json").read_text())
        self.assertEqual(spec["states"][EPISODE["index"]], EPISODE["state"])
        self.assertEqual(EPISODE["step_budget"], spec["step_budget"])
        self.assertEqual(EPISODE["episode"], EPISODE["state"]["id"])
        self.assertEqual(EPISODE["seed_sha256"], hashlib.sha256(str(EPISODE["state"]["seed"]).encode()).hexdigest())
        for key in ("episode", "seed_sha256"):
            self.assertEqual(ONLINE[key], EPISODE[key], key)
        self.assertEqual(ONLINE["job"], EPISODE["state"]["task"])
        self.assertEqual(ONLINE["task"], SERVICE["task"])
        self.assertEqual(EPISODE["oracle_token_sha256"], (TASK / "tests/oracle_token.sha256").read_text().strip())
        self.assertEqual(sha256(TASK / "solution/oracle_token"), EPISODE["oracle_token_sha256"])
        self.assertTrue((TASK / "solution/rescues" / (EPISODE["episode"] + ".npz")).is_file())
        self.assertIn(sha256(TASK / "tests/hidden/states.json"), ONLINE["draw"]["from"])
        self.assertIn("secrets.randbelow(27)", ONLINE["draw"]["how"])
        # the seed is in no file the agent's image or home gets
        for p in (TASK / "environment").rglob("*"):
            if p.is_file() and "online" not in p.relative_to(TASK / "environment").parts and p.suffix != ".npz":
                self.assertNotIn(str(EPISODE["state"]["seed"]), p.read_text(errors="replace"), str(p))

    def test_anchors_and_task_toml(self):
        main_anchor = json.loads((TASK / "tests/anchor_offline.json").read_text())
        online = json.loads((TASK / "tests/anchor_online.json").read_text())
        dual = json.loads((TASK / "tests/anchor.json").read_text())
        self.assertEqual((main_anchor["metric"], main_anchor["value"], main_anchor["full_at"]),
                         ("job_balanced_recovered_fraction", 1.0, 1.0))
        self.assertEqual((online["metric"], online["direction"], online["full_at"]),
                         (main_anchor["metric"], "higher", main_anchor["full_at"]))
        self.assertEqual((dual["metric"], dual["direction"], dual["value"], dual["full_at"]),
                         ("dual_mode_score", "higher", 1.0, 1.0))
        toml = (TASK / "task.toml").read_text()
        self.assertIn('artifacts = ["/home/user/submission", "/var/lib/arb-online"]', toml)
        agent = toml[toml.index("[agent]"):toml.index("[verifier]")]
        self.assertEqual(int(re.search(r"^timeout_sec\s*=\s*(\d+)", agent, re.M).group(1)),
                         SERVICE["prep_s"] + SERVICE["graded_s"])
        self.assertEqual(SERVICE["prep_s"], 28800)                    # main's agent budget
        verifier = toml[toml.index("[verifier]"):]
        self.assertEqual(int(re.search(r"^timeout_sec\s*=\s*(\d+)", verifier, re.M).group(1)), 28800)   # main's
        self.assertEqual((SERVICE["prep_s"], SERVICE["graded_s"]), (ONLINE["prep_s"], ONLINE["graded_s"]))
        client = (TASK / "environment/setup/payload/graded/client.py").read_text()
        self.assertIn('SOCK = "%s"' % SERVICE["socket"], client)
        self.assertNotIn("token", client)

    def test_the_instruction_states_both_items(self):
        text = flat(INSTRUCTION)
        for words in ("The score is the mean of two items, each relative to its own hidden reference value",
                      "The controller graded is the one in the submission folder at the earlier of two moments",
                      "over 5000 files or 200 MiB nothing is copied",
                      "The folder as it is when your session ends is graded only if the session ends before either "
                      "moment",
                      "Without a start the graded episode scores 0", "`prep_window_closed`", "`kitchen_failed`",
                      "It is one of the 27 hidden failure states your controller is graded on",
                      "a malformed action is refused (`malformed_action`) and simulates nothing",
                      "a connection silent for 600 s is closed (the episode stays)",
                      "needs about 6 GB of your 12 GB: do not run a kitchen simulator of your own while it is open",
                      "If your session ends first, the episode counts as it stood after your last step",
                      "The session lasts **11 hours** of wall time: **8 hours** for the controller and your preparation",
                      "the graded episode has none of them"):
            self.assertIn(words, text)
        for false in ("not graded", "scores nothing", "late finish", "You have **8 hours** of wall time",
                      "which are not installed here"):
            self.assertNotIn(false, text)


class Freeze(unittest.TestCase):
    """The service's freeze and the verifier's use of it, host-side as the current user (the agent's stand-in)."""

    @classmethod
    def setUpClass(cls):
        cls.svc = load(ONLINE_SETUP / "graded_service.py", "longtail_graded_service_under_test")
        cls.ver = load(TASK / "tests/verify.py", "longtail_dual_verify_under_test", TASK / "tests")

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.sub = os.path.join(self.tmp, "home", "submission")
        self.rec = os.path.join(self.tmp, "record")
        os.makedirs(self.sub)
        os.makedirs(self.rec)
        self.cfg = dict(self.svc.DEFAULTS, submission_dir=self.sub, record_dir=self.rec)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_the_freeze_and_the_verifiers_digest(self):
        Path(self.sub, "run.sh").write_text("#!/usr/bin/env bash\n")
        os.makedirs(os.path.join(self.sub, "rescues"))
        Path(self.sub, "rescues", "a.npz").write_bytes(b"\0" * 100)
        os.symlink("/etc/passwd", os.path.join(self.sub, "link"))
        res = self.svc.take_snapshot(self.cfg, os.getuid())
        self.assertEqual((res["status"], res["files"], res["skipped"]), ("ok", 2, 1))
        snap = os.path.join(self.rec, "submission_snapshot")
        files, size, digest, odd = self.ver.snapshot_digest(snap)
        self.assertEqual((files, size, digest, odd), (res["files"], res["bytes"], res["sha256"], 0))
        fields = {"snapshot_status": "ok", "snapshot_files": files, "snapshot_bytes": size, "snapshot_sha256": digest}
        self.assertIsNone(self.ver.check_snapshot(fields, root=snap))
        Path(self.sub, "run.sh").write_text("edited after the freeze\n")
        self.assertEqual(Path(snap, "run.sh").read_text(), "#!/usr/bin/env bash\n")
        self.assertEqual(self.svc.take_snapshot(dict(self.cfg, snapshot_max_files=2), os.getuid())["status"],
                         "too_large")                               # the link counts as a file, as main counts it

    def test_the_verifier_grades_what_was_frozen(self):
        base = {"format": ONLINE["format"], "task": ONLINE["task"], "episode": ONLINE["episode"],
                "seed_sha256": ONLINE["seed_sha256"]}
        snap = {"snapshot_status": "ok", "snapshot_files": 1, "snapshot_bytes": 2, "snapshot_sha256": "d"}
        decide = self.ver.freeze_decision
        self.assertEqual(decide(None, None, ONLINE), (None, {}))
        self.assertEqual(decide(None, {"service_started": 1.0, "started": False}, ONLINE), (None, {}))
        self.assertEqual(decide(dict(base, started=True, **snap), None, ONLINE)[0], "start")
        trig, fields = decide(dict(base, started=False, freeze="prep_window_closed", snapshot_status="failed"), None,
                              ONLINE)
        self.assertEqual(trig, "prep_window")
        self.assertIsNotNone(self.ver.check_snapshot(fields))       # a failed freeze: no fallback
        good = dict(base, started=True, episode_score=1.0)
        self.assertIsNone(self.ver.check_record(good, ONLINE))
        for bad in (dict(good, started=False), dict(good, episode_score=2.0), dict(good, episode="h04")):
            self.assertIsNotNone(self.ver.check_record(bad, ONLINE))
        self.assertEqual(self.ver.METRIC, json.loads((TASK / "tests/anchor_offline.json").read_text())["metric"])


if __name__ == "__main__":
    unittest.main()
