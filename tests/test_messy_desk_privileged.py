"""object_finding_and_organizing_on_messy_desk: the privileged reference channel and the fixed hidden episode set.

Online variant (branch exp/online-graded-episode): the graded episode service
(environment/setup/online/server.py) hands the simulator's ground truth only to a start request whose token hashes
to the digest (environment/setup/online/oracle_token.sha256, a copy of tests/oracle_token.sha256); nothing
agent-visible names the channel or holds the graded episode's seed (environment/setup/ outside payload/ is
root-only while the agent works and may); the hidden episodes are a fixed set of distinct 64-bit seeds, and the
graded episode is one of them, recorded by digest in tests/online.json.
"""
import hashlib
import importlib.util
import json
import unittest
from pathlib import Path

TASK = Path(__file__).resolve().parents[1] / "tasks/arm/object_finding_and_organizing_on_messy_desk"


def load_server():
    online = TASK / "environment/setup/online"
    spec = importlib.util.spec_from_file_location("messy_desk_online_server", online / "server.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


server = load_server()


class TokenTests(unittest.TestCase):
    def setUp(self):
        self.token = "0123456789abcdef" * 4
        self.digest = hashlib.sha256(self.token.encode()).hexdigest()

    def test_no_token_is_neither(self):
        self.assertEqual(server.token_ok(None, self.digest), (False, False))

    def test_matching_token_is_privileged(self):
        self.assertEqual(server.token_ok(self.token + "\n", self.digest), (True, False))

    def test_wrong_token_is_a_mismatch(self):
        self.assertEqual(server.token_ok("f" * 64, self.digest), (False, True))

    def test_a_token_that_is_not_a_short_string_is_refused(self):
        for token in (7, None, ["x"], "x" * (server.TOKEN_MAX_BYTES + 1)):
            line = json.dumps({"op": "start", "token": token}).encode()
            with self.assertRaises(server.ReplyError):
                server.parse_request(line)
        self.assertEqual(server.parse_request(json.dumps({"op": "start", "token": self.token}).encode())["token"],
                         self.token)

    def test_the_reference_token_matches_the_digest(self):
        token = (TASK / "solution/privileged/oracle_token").read_bytes().strip()
        self.assertEqual(len(token), 64)                      # 256 bits, hex
        want = (TASK / "tests/oracle_token.sha256").read_text().split()[0]
        self.assertEqual(hashlib.sha256(token).hexdigest(), want)
        self.assertEqual((TASK / "environment/setup/online/oracle_token.sha256").read_text(),
                         (TASK / "tests/oracle_token.sha256").read_text())

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
        spec = json.loads((TASK / "tests/episodes.json").read_text())
        seeds, key = [int(s) for s in spec["seeds"]], int(spec["probe_key"])
        self.assertGreaterEqual(len(seeds), 1)
        self.assertEqual(len(set(seeds)), len(seeds))
        self.assertTrue(all(2 ** 32 <= s < 2 ** 64 for s in seeds))     # none a seed anyone would type into the lab
        self.assertTrue(0 < key < 2 ** 64)

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
        blk = server.privileged_block(state, cont, obs)
        self.assertEqual([o["id"] for o in blk["objects"]], [3])           # present objects only
        self.assertNotIn("value", blk["objects"][0])                        # the score's terms stay out
        self.assertEqual(blk["request"]["object"], 3)
        self.assertEqual(blk["containers"], cont)
        self.assertEqual(blk["ee_pose"], obs["ee_pose"])
        state["request"]["status"] = "delivered"
        self.assertIsNone(server.privileged_block(state, cont, obs)["request"])


if __name__ == "__main__":
    unittest.main()
