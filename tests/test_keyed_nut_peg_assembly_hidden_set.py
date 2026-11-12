"""keyed_nut_peg_assembly: the fixed hidden set, its pre-registered selection, the anchor, and the dev sim's contract.

The hidden kits (tests/ref/episodes.json) are the first 10, in drawn order, of the pre-registered os.urandom
candidates (tests/hidden/selection.json) that the frozen reference solution completed when each ran once through the
grader (tests/review/select_set.py, tests/review/regrade.sh). The files the reference and the grader run are unchanged
since (their sha256 in the selection record). The anchor is the reference's recorded full verify of the set, which
repeated every kept selection run. The agent's dev sim (environment/setup/payload/keyed_env.py, byte-identical in
tests/) has no ground-truth accessor and no score, its practice bench prints no completion verdict, and the practice
kit is none of the graded kits.

Standard library only (the host-side suite has no numpy or mujoco): the task's Python files are read as text or syntax
trees, never imported.
"""

import ast
import hashlib
import json
import unittest
from pathlib import Path

TASK = Path(__file__).resolve().parents[1] / "tasks/arm/keyed_nut_peg_assembly"
SELECTION = json.loads((TASK / "tests/hidden/selection.json").read_text())
EPISODES = json.loads((TASK / "tests/ref/episodes.json").read_text())["episodes"]
ANCHOR = json.loads((TASK / "tests/anchor.json").read_text())
PAYLOAD = TASK / "environment/setup/payload"


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class HiddenSet(unittest.TestCase):
    def test_candidates_are_unique_63_bit_seeds(self):
        cands = SELECTION["candidates"]
        self.assertGreaterEqual(len(cands), 16)
        self.assertEqual([c["i"] for c in cands], list(range(len(cands))))
        seeds = [c["kit_seed"] for c in cands] + [c["layout_seed"] for c in cands]
        self.assertEqual(len(set(seeds)), len(seeds))
        self.assertTrue(all(0 < s < 2 ** 63 for s in seeds))

    def test_set_is_the_first_ten_kept_in_drawn_order(self):
        runs = SELECTION["selection_run"]["runs"]
        self.assertEqual([r["id"] for r in runs], [c["id"] for c in SELECTION["candidates"]])
        kept = [r["id"] for r in runs if r["kept"]]
        self.assertEqual(SELECTION["hidden_set"], kept[:SELECTION["set_size"]])
        self.assertEqual(len(SELECTION["hidden_set"]), 10)
        for r in runs:
            if r["kept"]:
                self.assertEqual((r["kit_complete"], r["nuts_assembled"], r["drop_reasons"]), (1, 3, []))
            else:
                self.assertEqual(r["kit_complete"], 0)
                self.assertTrue(r["drop_reasons"])

    def test_episodes_are_the_selected_candidates(self):
        cands = {c["id"]: c for c in SELECTION["candidates"]}
        self.assertEqual([e["id"] for e in EPISODES], SELECTION["hidden_set"])
        for e in EPISODES:
            self.assertEqual(e["seed"], cands[e["id"]]["layout_seed"])
            nuts = e["kit"]["nuts"]
            self.assertEqual(len(nuts), 3)
            self.assertEqual([n["id"] for n in nuts], ["%s-%d" % (e["id"], k) for k in range(3)])
            self.assertEqual(sorted(e["kit"]["order"]), [1, 2, 3])

    def test_frozen_files_unchanged(self):
        for rel, digest in SELECTION["frozen"].items():
            self.assertEqual(sha256(TASK / rel), digest, "%s changed since the selection: record an amendment" % rel)

    def test_anchor_is_the_recorded_full_verify(self):
        av = SELECTION["anchor_verify"]
        self.assertTrue(av["every_episode_repeats_its_selection_run"])
        self.assertEqual([e["id"] for e in av["episodes"]], SELECTION["hidden_set"])
        runs = {r["id"]: r for r in SELECTION["selection_run"]["runs"]}
        for e in av["episodes"]:
            self.assertEqual((e["kit_complete"], e["steps"]), (runs[e["id"]]["kit_complete"], runs[e["id"]]["steps"]))
        self.assertEqual(ANCHOR["metric"], "kit_success_rate")
        self.assertEqual(ANCHOR["value"], av["measured"])
        self.assertLessEqual(ANCHOR["full_at"], ANCHOR["value"])
        self.assertGreaterEqual(ANCHOR["full_at"], 0.95)

    def test_practice_kit_is_not_a_graded_kit(self):
        practice = json.loads((PAYLOAD / "practice_kit.json").read_text())
        grooves = {json.dumps(n["groove"], sort_keys=True) for n in practice["nuts"]}
        for e in EPISODES:
            graded = {json.dumps(n["groove"], sort_keys=True) for n in e["kit"]["nuts"]}
            self.assertFalse(grooves <= graded, "%s has the practice kit's grooves" % e["id"])


class DevSimContract(unittest.TestCase):
    def test_grader_and_agent_run_the_same_sim(self):
        self.assertEqual(sha256(PAYLOAD / "keyed_env.py"), sha256(TASK / "tests/keyed_env.py"))

    def test_dev_sim_has_no_truth_or_score(self):
        tree = ast.parse((PAYLOAD / "keyed_env.py").read_text())
        env = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "NutEnv")
        methods = {n.name for n in env.body if isinstance(n, ast.FunctionDef)}
        for name in ("gt", "episode_score", "nuts_assembled", "progress", "assembled", "stage_now"):
            self.assertNotIn(name, methods)

    def test_bench_prints_no_verdict(self):
        bench = (PAYLOAD / "practice_bench/run_bench.py").read_text()
        for word in ("episode_score", "nuts_assembled", "kit_complete", "progress(", ".gt(", "keyed_credit"):
            self.assertNotIn(word, bench)
        self.assertFalse((PAYLOAD / "practice_grader").exists())

    def test_grader_scores_with_the_credit_module(self):
        verify = (TASK / "tests/verify.py").read_text()
        self.assertIn("KC.GradedNutEnv(", verify)
        self.assertIn("KC.ground_truth(env, wire)", verify)
        self.assertNotIn("E.NutEnv(", verify)
        config = json.loads((TASK / "tests/grader_config.json").read_text())
        self.assertNotIn("grader_budget_s", config)
        self.assertEqual(config["controller_budget_s"], 600)


if __name__ == "__main__":
    unittest.main()
