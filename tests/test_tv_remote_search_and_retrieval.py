"""tv_remote_search_and_retrieval: the agent practises with exactly the simulator the grader runs, the hidden set is
the one its selection record made, nothing the grader logs names a place, and what the instruction says about the
session clock, the episodes and the numerics pin holds in the files.

Standard library only (the host-side suite has no numpy): nothing of the task is imported.
"""

import hashlib
import json
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TASK = ROOT / "tasks/arm/tv_remote_search_and_retrieval"
PIN = {"OPENBLAS_CORETYPE": "Haswell", "OPENBLAS_L2_SIZE": "256",
       "NPY_DISABLE_CPU_FEATURES": "X86_V4 AVX512_ICL AVX512_SPR",
       "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1", "OPENCV_IPP": "avx2"}
PLAN = {"left_cushion": 1, "right_cushion": 1, "under_sofa": 2, "under_chest": 2, "under_hat": 2, "in_box": 2,
        "in_drawer": 2}
PLACE_WORDS = ("cushion", "under_sofa", "under_chest", "under_hat", "in_box", "in_drawer", "spot", "place")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def toml_int(text, table, key):
    body = text[text.index("[{}]".format(table)):]
    return int(re.search(r"^{}\s*=\s*(\d+)".format(key), body, re.M).group(1))


class Copies(unittest.TestCase):

    def test_the_agent_has_the_grader_s_simulator(self):
        self.assertEqual(sha256(TASK / "environment/setup/payload/remote_env.py"), sha256(TASK / "tests/remote_env.py"))

    def test_the_hidden_rule_and_the_reference_are_not_in_the_agent_s_reach(self):
        staged = {p.name for p in (TASK / "environment").rglob("*") if p.is_file()}
        for name in ("episodes.py", "episodes.json", "oracle.py", "search_skills.py", "author_controller.py",
                     "oracle_token"):
            self.assertNotIn(name, staged, name)
        simulator = (TASK / "tests/remote_env.py").read_text()
        for name in PLAN:
            self.assertNotIn(name, simulator, name)


class SessionTime(unittest.TestCase):

    def test_script_instruction_and_task_agree_on_six_hours(self):
        script = (TASK / "environment/setup/payload/session_time.sh").read_text()
        self.assertEqual(int(re.search(r"^LIMIT_MIN=(\d+)", script, re.M).group(1)) * 60,
                         toml_int((TASK / "task.toml").read_text(), "agent", "timeout_sec"))
        text = (TASK / "instruction.md").read_text()
        self.assertIn("## Session time", text)
        self.assertIn("bash /home/user/session_time.sh", text)
        self.assertIn("/run/session/start", (TASK / "environment/setup/setup.sh").read_text())


class HiddenSet(unittest.TestCase):

    def setUp(self):
        self.eps = json.loads((TASK / "tests/hidden/episodes.json").read_text())["episodes"]
        self.rec = json.loads((TASK / "tests/hidden/selection.json").read_text())

    def test_twelve_episodes_in_the_registered_composition(self):
        count = {}
        for e in self.eps:
            count[e["spot"]] = count.get(e["spot"], 0) + 1
        self.assertEqual(count, PLAN)
        self.assertEqual(len({int(e["seed"]) for e in self.eps}), len(self.eps))
        self.assertIn("mean score over {} graded episodes".format(len(self.eps)), (TASK / "instruction.md").read_text())

    def test_the_set_is_the_latest_draw_s_first_kept_candidates(self):
        last = self.rec["draws"][-1]
        self.assertEqual(last["status"], "made")
        self.assertTrue(all(d["status"] == "void" and d.get("void_reason") for d in self.rec["draws"][:-1]))
        want = set()
        for place, need in PLAN.items():
            kept = [c for c in last["candidates"] if c["place"] == place and c["status"] == "kept"]
            kept.sort(key=lambda c: c["index"])
            want |= {(place, int(c["seed"])) for c in kept[:need]}
        self.assertEqual({(e["spot"], int(e["seed"])) for e in self.eps}, want)

    def test_every_candidate_s_seed_comes_from_the_key(self):
        last = self.rec["draws"][-1]
        key = bytes.fromhex(last["key"])
        for c in last["candidates"]:
            msg = "candidate|{}|{}".format(c["place"], c["index"]).encode()
            seed = int.from_bytes(hashlib.blake2b(msg, key=key, digest_size=8).digest(), "little")
            self.assertEqual(int(c["seed"]), seed)

    def test_a_kept_candidate_scored_one_in_three_wire_runs_within_the_step_margin(self):
        cap = self.rec["step_margin"] * 4800
        for c in self.rec["draws"][-1]["candidates"]:
            if c["status"] == "kept":
                self.assertEqual(len(c["wire"]), 3)
                self.assertTrue(all(w["score"] == 1.0 and w["steps"] <= cap for w in c["wire"]))

    def test_the_frozen_hashes_are_the_files(self):
        for rel, digest in self.rec["frozen"].items():
            self.assertEqual(sha256(TASK / rel), digest, rel)

    def test_the_oracle_token_is_the_recorded_one(self):
        token = (TASK / "solution/oracle_token").read_bytes().strip()
        self.assertEqual(hashlib.sha256(token).hexdigest(), (TASK / "tests/hidden/oracle_token.sha256").read_text().strip())


class Log(unittest.TestCase):

    def test_what_the_grader_logs_names_no_place(self):
        for name in ("remote_grade.py", "verify.py"):
            text = (TASK / "tests" / name).read_text()
            for call in re.findall(r"(?:log|write_verdict|zero_verdict)\((.*)", text):
                for word in PLACE_WORDS:
                    self.assertNotIn(word, call, "{}: {}".format(name, call[:80]))


class Assets(unittest.TestCase):

    def test_the_room_is_pinned_and_not_in_git(self):
        pin = (TASK / "environment/assets.sha256").read_text()
        self.assertRegex(pin, r"# revision: [0-9a-f]{40}")
        for name in ("sofa_frame.obj", "remote.obj", "remote.png", "remote_b.png", "SOURCES.md"):
            self.assertIn("room/" + name, pin)
        self.assertFalse((TASK / "environment/room").exists())
        self.assertIn("ln -s /opt/task-assets/payload/room /opt/room", (TASK / "environment/Dockerfile").read_text())


class Numerics(unittest.TestCase):

    def test_the_image_pins_what_the_launcher_passes(self):
        docker = (TASK / "environment/Dockerfile").read_text()
        launcher = (TASK / "tests/nonet_exec.py").read_text()
        for key, value in PIN.items():
            self.assertIn(key, docker)
            self.assertIn(value, docker)
            self.assertIn(key, launcher)
        self.assertTrue((TASK / "tests/numerics.json").is_file())


class NoAuthorPaths(unittest.TestCase):

    def test_no_home_directory_in_the_task_s_files(self):
        for path in TASK.rglob("*.py"):
            if "robotics_grader" in path.parts:
                continue                      # the kit's own files
            self.assertFalse(re.search(r"/home/(?!user\b)[a-z]", path.read_text()), str(path))


if __name__ == "__main__":
    unittest.main()
