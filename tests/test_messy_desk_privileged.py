"""object_finding_and_organizing_on_messy_desk: the privileged reference channel, the fixed hidden episode set and,
in the dual mode (branch exp/dual-mode-scoring), the freeze of the program at the graded start.

Main's offline grader (tests/offline_verify.py, byte-identical to main's tests/verify.py) hands the simulator's
ground truth only to a submission whose .oracle_token (a regular file, never a link) hashes to
tests/oracle_token.sha256; the hidden episodes are a fixed set of distinct 64-bit seeds, and an episode's probe
stream depends only on the set's key and the episode's seed. The graded episode service
(environment/setup/online/server.py) hands the same truth only to a start request whose token hashes to its copy of
the digest; nothing agent-visible names the channel or holds a hidden seed (environment/setup/ outside payload/ is
root-only while the agent works and may). At the graded start the service's root side copies the submission folder,
following no link and copying nothing another account owns; the dual verifier grades that copy when its digest
matches the record's.
"""
import hashlib
import importlib.util
import json
import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

TASK = Path(__file__).resolve().parents[1] / "tasks/arm/object_finding_and_organizing_on_messy_desk"
MAIN_VERIFY_SHA256 = "eb08b182ff49e6b7eaa29e36bb7a41fd756ed1efe66aa52f4de02eb3d2eb56b1"   # main's tests/verify.py


def load(name, path):
    sys.path.insert(0, str(path.parent))
    try:
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(path.parent))


verify = load("messy_desk_verify", TASK / "tests" / "offline_verify.py")       # main's offline grader
dual = load("messy_desk_dual_verify", TASK / "tests" / "verify.py")
server = load("messy_desk_online_server", TASK / "environment/setup/online/server.py")


class TokenTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.sub = os.path.join(self.tmp.name, "submission")
        os.makedirs(self.sub)
        self.token = b"0123456789abcdef" * 4
        self.digest = os.path.join(self.tmp.name, "oracle_token.sha256")
        with open(self.digest, "w") as f:
            f.write(hashlib.sha256(self.token).hexdigest() + "\n")

    def tearDown(self):
        self.tmp.cleanup()

    def state(self):
        return verify.token_state(self.sub, digest_path=self.digest)

    def test_no_token_is_neither(self):
        self.assertEqual(self.state(), (False, False))

    def test_matching_token_is_privileged(self):
        with open(os.path.join(self.sub, ".oracle_token"), "wb") as f:
            f.write(self.token + b"\n")
        self.assertEqual(self.state(), (True, False))

    def test_wrong_token_is_a_mismatch(self):
        with open(os.path.join(self.sub, ".oracle_token"), "wb") as f:
            f.write(b"f" * 64)
        self.assertEqual(self.state(), (False, True))

    def test_a_link_is_never_followed(self):
        real = os.path.join(self.tmp.name, "real_token")
        with open(real, "wb") as f:
            f.write(self.token)
        os.symlink(real, os.path.join(self.sub, ".oracle_token"))
        self.assertEqual(self.state(), (False, True))

    def test_a_directory_or_a_huge_file_is_a_mismatch(self):
        os.makedirs(os.path.join(self.sub, ".oracle_token"))
        self.assertEqual(self.state(), (False, True))
        os.rmdir(os.path.join(self.sub, ".oracle_token"))
        with open(os.path.join(self.sub, ".oracle_token"), "wb") as f:
            f.write(self.token + b" " * (verify.TOKEN_MAX_BYTES + 10))
        self.assertEqual(self.state(), (False, True))

    def test_the_reference_token_matches_the_digest(self):
        token = (TASK / "solution/privileged/oracle_token").read_bytes().strip()
        self.assertEqual(len(token), 64)                      # 256 bits, hex
        want = (TASK / "tests/oracle_token.sha256").read_text().split()[0]
        self.assertEqual(hashlib.sha256(token).hexdigest(), want)
        self.assertEqual((TASK / "environment/setup/online/oracle_token.sha256").read_text(),
                         (TASK / "tests/oracle_token.sha256").read_text())

    def test_the_service_judges_a_start_token_alike(self):
        digest = hashlib.sha256(self.token).hexdigest()
        self.assertEqual(server.token_ok(None, digest), (False, False))
        self.assertEqual(server.token_ok(self.token.decode() + "\n", digest), (True, False))
        self.assertEqual(server.token_ok("f" * 64, digest), (False, True))
        for token in (7, ["x"], "x" * (server.TOKEN_MAX_BYTES + 1)):
            with self.assertRaises(server.ReplyError):
                server.parse_request(json.dumps({"op": "start", "token": token}).encode())

    def test_nothing_agent_visible_names_the_channel(self):
        """The agent sees instruction.md, the image (environment/ less setup/) and the payload staged into its
        home; environment/setup/ outside payload/ stays root-only (/opt/ale/setup) while the agent works."""
        token = (TASK / "solution/privileged/oracle_token").read_text().strip()
        digest = (TASK / "tests/oracle_token.sha256").read_text().split()[0]
        seeds = [str(s) for s in json.loads((TASK / "tests/episodes.json").read_text())["seeds"]]
        env = TASK / "environment"
        visible = [TASK / "instruction.md"] + [p for p in env.rglob("*") if p.is_file() and (
            "setup" not in p.relative_to(env).parts[:1] or p.relative_to(env).parts[:2] == ("setup", "payload"))]
        self.assertIn(env / "setup/payload/graded_episode.py", visible)
        for p in visible:
            text = p.read_text(errors="replace")
            for needle in [token, digest, ".oracle_token", "oracle_token", "privileged_mode", '"privileged"'] + seeds:
                self.assertNotIn(needle, text, "{} mentions {}".format(p.relative_to(TASK), needle[:16]))


class EpisodeSetTests(unittest.TestCase):
    def test_fixed_set_is_distinct_64_bit(self):
        seeds, key = verify.load_episodes(str(TASK / "tests/episodes.json"))
        self.assertGreaterEqual(len(seeds), 1)
        self.assertEqual(len(set(seeds)), len(seeds))
        self.assertTrue(all(2 ** 32 <= s < 2 ** 64 for s in seeds))     # none a seed anyone would type into the lab
        self.assertTrue(0 < key < 2 ** 64)

    def test_probe_stream_depends_on_key_and_seed_only(self):
        seeds, key = verify.load_episodes(str(TASK / "tests/episodes.json"))
        a = verify.seed_rng(entropy=(key << 64) | seeds[0])
        b = verify.seed_rng(entropy=(key << 64) | seeds[0])
        self.assertEqual([a.random() for _ in range(50)], [b.random() for _ in range(50)])
        if len(seeds) > 1:
            c = verify.seed_rng(entropy=(key << 64) | seeds[1])
            d = verify.seed_rng(entropy=(key << 64) | seeds[0])
            self.assertNotEqual([c.random() for _ in range(5)], [d.random() for _ in range(5)])

    def test_the_graded_episode_is_one_of_the_set_and_matches_its_digest(self):
        seeds = [int(s) for s in json.loads((TASK / "tests/episodes.json").read_text())["seeds"]]
        episode = json.loads((TASK / "environment/setup/online/episode.json").read_text())
        online = json.loads((TASK / "tests/online.json").read_text())
        self.assertIn(episode["seed"], seeds)
        self.assertEqual(online["episode"], episode["episode"])
        self.assertEqual(episode["episode"], "hidden-%d" % seeds.index(episode["seed"]))
        self.assertEqual(online["seed_sha256"], hashlib.sha256(str(episode["seed"]).encode()).hexdigest())

    def test_the_service_runs_the_verifiers_own_episode_code(self):
        for name in ("env.py", "sim_client.py", "messy_desk_score.py"):
            self.assertEqual((TASK / "environment/setup/online" / name).read_bytes(), (TASK / "tests" / name).read_bytes(),
                             name)

    def test_the_offline_item_is_mains_grader(self):
        data = (TASK / "tests/offline_verify.py").read_bytes()
        self.assertEqual(hashlib.sha256(data).hexdigest(), MAIN_VERIFY_SHA256)


class PrivilegedBlockTests(unittest.TestCase):
    def test_block_carries_the_truth_of_the_open_request(self):
        state = {"t": 7, "ik_ok": True,
                 "objects": [{"id": 3, "model": "025_mug", "cls": "C", "present": True, "removed": False,
                              "pos": [0.5, 0.1, 0.04], "quat": [1, 0, 0, 0], "scale": 1.0, "state": "on_table",
                              "in_container": None, "on_table": True, "held": False, "contact": False, "resting": True,
                              "value": 0, "baseline": 0},
                             {"id": 4, "model": "024_bowl", "cls": "C", "present": False, "removed": True}],
                 "request": {"id": 2, "model": "025_mug", "object": 3, "text": "the mug", "opened_t": 5,
                             "deadline_t": 1205, "status": "open"}}
        obs = {"ee_pose": [0.46, 0.0, 0.39, 0.0, 1.0, 0.0, 0.0], "joint_pos": [0.0] * 7, "gripper_width": 0.08}
        cont = {"red": {"xy": [0.25, 0.0], "yaw": 0.01}}
        for module in (verify, server):                 # the offline grader and the graded episode's service alike
            blk = module.privileged_block(state, cont, obs)
            self.assertEqual([o["id"] for o in blk["objects"]], [3])           # present objects only
            self.assertNotIn("value", blk["objects"][0])                        # the score's terms stay out
            self.assertEqual(blk["request"]["object"], 3)
            self.assertEqual(blk["containers"], cont)
            self.assertEqual(blk["ee_pose"], obs["ee_pose"])
            closed = dict(state, request=dict(state["request"], status="delivered"))
            self.assertIsNone(module.privileged_block(closed, cont, obs)["request"])

    def test_a_probe_keeps_the_true_block(self):
        obs = {"ee_pose": [0.46, 0.0, 0.39, 0.0, 1.0, 0.0, 0.0], "joint_pos": [0.0] * 7, "gripper_width": 0.08,
               "privileged": {"ee_pose": [0.46, 0.0, 0.39, 0.0, 1.0, 0.0, 0.0]}}
        p = verify._perturbed(obs, verify.seed_rng(entropy=1))
        self.assertNotEqual(p["ee_pose"][:3], obs["ee_pose"][:3])
        self.assertIs(p["privileged"], obs["privileged"])


class FreezeTests(unittest.TestCase):
    """The service's snapshot (root side, server.snapshot_submission) and the dual verifier's check of it
    (verify.snapshot_digest, verify.install_snapshot), run here as the current account in a temporary folder."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        self.src, self.records = root / "submission", root / "records"
        (self.src / "pkg" / "deep").mkdir(parents=True)
        self.records.mkdir()
        (self.src / "run.sh").write_text("#!/usr/bin/env bash\necho READY\n")
        os.chmod(self.src / "run.sh", 0o755)
        (self.src / ".oracle_token").write_text("x" * 64)
        (self.src / "pkg" / "weights.bin").write_bytes(os.urandom(70000))
        os.chmod(self.src / "pkg" / "weights.bin", 0o600)
        (self.src / "pkg" / "deep" / "notes.txt").write_text("v1")
        self.outside = root / "hidden.json"
        self.outside.write_text('{"seed": 1}')
        os.symlink(self.outside, self.src / "seed_link.json")              # a link: never followed
        os.symlink(root, self.src / "dir_link")
        os.mkfifo(self.src / "pipe")                                          # not a regular file
        self.cfg = {"submission": str(self.src), "snapshot_dir": str(self.records / "submission_snapshot"),
                    "snapshot_caps": {"max_entries": 100, "max_depth": 64, "max_bytes": 1 << 20}}

    def tearDown(self):
        self.tmp.cleanup()

    def test_copies_own_files_with_modes_and_skips_links_and_fifos(self):
        rep = server.snapshot_submission(self.cfg, os.getuid())
        snap = Path(self.cfg["snapshot_dir"])
        self.assertEqual(rep["status"], "ok")
        self.assertEqual(rep["files"], 4)
        self.assertEqual(rep["skipped"], 3)
        self.assertEqual(sorted(p.relative_to(snap).as_posix() for p in snap.rglob("*") if p.is_file()),
                         [".oracle_token", "pkg/deep/notes.txt", "pkg/weights.bin", "run.sh"])
        self.assertFalse(any(p.is_symlink() for p in snap.rglob("*")))
        self.assertEqual(os.stat(snap / "run.sh").st_mode & 0o777, 0o755)
        self.assertEqual(os.stat(snap / "pkg" / "weights.bin").st_mode & 0o777, 0o600)
        self.assertEqual((snap / "pkg" / "weights.bin").read_bytes(), (self.src / "pkg" / "weights.bin").read_bytes())
        self.assertEqual(dual.snapshot_digest(str(snap)), rep["sha256"])

    def test_a_change_after_the_freeze_does_not_reach_the_graded_program(self):
        rep = server.snapshot_submission(self.cfg, os.getuid())
        (self.src / "pkg" / "deep" / "notes.txt").write_text("v2, written after the start")
        (self.src / "late.py").write_text("print('late')")
        graded = Path(self.tmp.name) / "graded_submission"
        shutil.copytree(self.src, graded, symlinks=True,                     # the session-end folder, restored
                        ignore=lambda d, names: [n for n in names if n == "pipe"])   # (the engine copies no fifo)
        self.assertEqual(dual.snapshot_digest(self.cfg["snapshot_dir"]), rep["sha256"])
        dual.install_snapshot(self.cfg["snapshot_dir"], str(graded))
        self.assertEqual((graded / "pkg" / "deep" / "notes.txt").read_text(), "v1")
        self.assertFalse((graded / "late.py").exists())
        self.assertFalse((graded / "seed_link.json").exists())

    def test_a_changed_snapshot_no_longer_matches_its_digest(self):
        rep = server.snapshot_submission(self.cfg, os.getuid())
        (Path(self.cfg["snapshot_dir"]) / "run.sh").write_text("#!/usr/bin/env bash\necho forged\n")
        self.assertNotEqual(dual.snapshot_digest(self.cfg["snapshot_dir"]), rep["sha256"])

    def test_over_a_cap_nothing_is_kept(self):
        for caps in ({"max_entries": 3, "max_depth": 64, "max_bytes": 1 << 20},
                     {"max_entries": 100, "max_depth": 64, "max_bytes": 1000}):
            rep = server.snapshot_submission(dict(self.cfg, snapshot_caps=caps), os.getuid())
            self.assertEqual(rep["status"], "too_large")
            self.assertFalse(os.path.exists(self.cfg["snapshot_dir"]))
            self.assertFalse(os.path.exists(self.cfg["snapshot_dir"] + ".tmp"))

    def test_another_accounts_files_and_a_missing_folder(self):
        rep = server.snapshot_submission(self.cfg, os.getuid() + 1)          # nothing is the agent's
        self.assertEqual((rep["status"], rep["files"]), ("ok", 0))
        shutil.rmtree(self.src)
        rep = server.snapshot_submission(self.cfg, os.getuid())
        self.assertEqual((rep["status"], rep["files"], rep["note"]), ("ok", 0, "no submission folder"))
        self.assertEqual(dual.snapshot_digest(self.cfg["snapshot_dir"]), rep["sha256"])


if __name__ == "__main__":
    unittest.main()
