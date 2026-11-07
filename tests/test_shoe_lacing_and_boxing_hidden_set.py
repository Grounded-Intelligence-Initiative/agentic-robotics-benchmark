"""shoe_lacing_and_boxing: the privileged reference's token, the fixed hidden set, its selection and the anchor.

The grader gives the privileged calls (the reference controller inside the cell) only to a submission whose
.oracle_token hashes to tests/hidden/oracle_token.sha256, and that digest lives only there. The hidden episodes are a
fixed set (tests/hidden/seeds.json): the first 6, in drawn order, of a pre-registered batch of os.urandom seeds on
which the frozen reference scores 1.0 (tests/hidden/selection.json; the rule in tests/review/select_set.py), and
verify.py runs that list as it stands. The files the set was selected with (the reference, the cell, the grader, the
image's inputs) are unchanged since: their sha256 in the selection record. The anchor is the lowest work_score of the
recorded full verifies of the privileged submission on the set, with a full_at under which those verifies still read
reward 1.0. Nothing the agent sees names the token, the set or its selection; the cell library's own mentions of its
privileged calls (the PR author's public cell code, byte-identical to the grader's) are pinned as they are.

Standard library only (the host-side suite has no numpy): verify.py is read as a syntax tree, not imported.
"""

import ast
import hashlib
import importlib.util
import json
import re
import sys
import unittest
from pathlib import Path

TASK = Path(__file__).resolve().parents[1] / "tasks/arm/shoe_lacing_and_boxing"
HIDDEN = TASK / "tests/hidden"


def _module(name, path):
    saved = sys.dont_write_bytecode
    sys.dont_write_bytecode = True  # (no __pycache__ in the task folder)
    try:
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    finally:
        sys.dont_write_bytecode = saved


seeds_mod = _module("shoe_lacing_cellkit_seeds", TASK / "tests/cellkit/seeds.py")
select_set = _module("shoe_lacing_select_set", TASK / "tests/review/select_set.py")
SELECTION = json.loads((HIDDEN / "selection.json").read_text())
SEEDS = json.loads((HIDDEN / "seeds.json").read_text())
ANCHOR = json.loads((TASK / "tests/anchor.json").read_text())
CONFIG = json.loads((TASK / "tests/grader_config.json").read_text())


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def task_files():
    return sorted(p for p in TASK.rglob("*") if p.is_file() and "__pycache__" not in p.parts)


def agent_visible():
    return [TASK / "instruction.md"] + sorted(p for p in (TASK / "environment").rglob("*") if p.is_file())


class TokenTests(unittest.TestCase):
    def test_the_reference_token_matches_the_digest(self):
        token = (TASK / "solution/reference/oracle_token").read_bytes().strip()
        self.assertRegex(token.decode(), r"^[0-9a-f]{64}$")  # 256 bits, hex
        digest = (HIDDEN / "oracle_token.sha256").read_text().split()[0]
        self.assertEqual(hashlib.sha256(token).hexdigest(), digest)

    def test_the_digest_lives_only_under_tests_hidden(self):
        digest = (HIDDEN / "oracle_token.sha256").read_text().split()[0]
        holders = [p.relative_to(TASK).as_posix() for p in task_files() if digest in p.read_text(errors="replace")]
        self.assertEqual(holders, ["tests/hidden/oracle_token.sha256"])

    def test_the_token_lives_only_in_the_solution(self):
        token = (TASK / "solution/reference/oracle_token").read_text().strip()
        holders = [p.relative_to(TASK).as_posix() for p in task_files() if token in p.read_text(errors="replace")]
        self.assertEqual(holders, ["solution/reference/oracle_token"])


class HiddenSetTests(unittest.TestCase):
    def kept(self):
        return [c for c in sorted(SELECTION["candidates"], key=lambda c: c["draw"]) if c["status"] == "kept"]

    def test_seeds_json_is_the_episode_count_of_distinct_63_bit_seeds(self):
        pinned = seeds_mod.load_pinned(str(HIDDEN / "seeds.json"))
        seeds = pinned["seeds"]
        self.assertEqual(len(seeds), CONFIG["episodes"])
        self.assertEqual(CONFIG["episodes"], 6)  # instruction.md: "6 episodes"
        self.assertIn("(6 episodes)", (TASK / "instruction.md").read_text())
        self.assertEqual(len(set(seeds)), len(seeds))
        self.assertTrue(all(isinstance(s, int) and 2 ** 32 <= s < 2 ** 63 for s in seeds))
        self.assertEqual(set(SEEDS) - {"seeds", "source"}, set())

    def test_the_set_is_the_selection_s_kept_seeds_in_drawn_order(self):
        kept = self.kept()
        self.assertEqual([c["seed"] for c in kept], SEEDS["seeds"])
        self.assertEqual(SELECTION["set"]["kept"], [c["draw"] for c in kept])
        self.assertEqual(SELECTION["set"]["seeds_sha256"], sha256(HIDDEN / "seeds.json"))
        k, d, a = select_set.classify(SELECTION, CONFIG["episodes"])  # the rule, applied again to the record
        self.assertEqual([c["draw"] for c in k], SELECTION["set"]["kept"])
        self.assertEqual([c["draw"] for c in d], SELECTION["set"]["dropped"])
        self.assertEqual([c["draw"] for c in a], SELECTION["set"]["after_cutoff"])
        for c in kept:
            r = c["result"]
            self.assertTrue(select_set.passes(r))
            self.assertEqual(r["work_score"], 1.0)
            self.assertTrue(r["safety_ok"])
        for c in d:
            self.assertFalse(select_set.passes(c["result"]))
            self.assertTrue(c.get("reason"))

    def test_every_candidate_was_drawn_before_any_ran_and_ran_once(self):
        cands = sorted(SELECTION["candidates"], key=lambda c: c["draw"])
        self.assertEqual([c["draw"] for c in cands], list(range(len(cands))))
        self.assertEqual(len({c["seed"] for c in cands}), len(cands))
        self.assertTrue(all(2 ** 32 <= c["seed"] < 2 ** 63 for c in cands))
        drawn = {b["batch"]: b["drawn_utc"] for b in SELECTION["batches"]}
        for c in cands:
            self.assertIn(c["status"], ("kept", "dropped", "after_cutoff"))
            r = c["result"]
            self.assertEqual(r["seed"], c["seed"])
            self.assertLess(drawn[c["batch"]], r["started_utc"])  # (ISO 8601 UTC strings order as times)
            self.assertTrue(r["frozen_ok"])
            self.assertEqual(r["image_id"], SELECTION["image"]["id"])
        self.assertEqual(sum(b["count"] for b in SELECTION["batches"]), len(cands))

    def test_the_pinned_list_is_run_as_it_stands(self):
        """verify.py reads the list itself (hidden_set); a pinned stream's derive() would hash it into other seeds."""
        tree = ast.parse((TASK / "tests/verify.py").read_text())
        funcs = {n.name: n for n in tree.body if isinstance(n, ast.FunctionDef)}
        calls = {n.func.id for n in ast.walk(funcs["_main_locked"]) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)}
        self.assertIn("hidden_set", calls)
        attrs = {n.func.attr for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
        self.assertNotIn("derive", attrs)
        self.assertIn('"seeds.json"', ast.get_source_segment((TASK / "tests/verify.py").read_text(), funcs["hidden_set"]))
        path = str(HIDDEN / "seeds.json")
        self.assertEqual(seeds_mod.seed_rng(pinned=path).seeds(6, bits=63), SEEDS["seeds"])
        self.assertNotEqual(seeds_mod.seed_rng(pinned=path).derive("episodes").seeds(6, bits=63), SEEDS["seeds"])

    def test_the_hollowing_check_is_recorded_as_the_rule_computes_it(self):
        k, d, a = select_set.classify(SELECTION, CONFIG["episodes"])
        self.assertEqual(SELECTION["set"]["hollowing"], select_set.hollowing(k, d, a))


class FrozenTests(unittest.TestCase):
    def test_the_files_the_set_was_selected_with_are_unchanged(self):
        frozen = SELECTION["frozen"]
        for rel, digest in sorted(frozen.items()):
            self.assertEqual(sha256(TASK / rel), digest, "{} changed since the selection: the set and the anchor "
                             "were measured with the frozen file".format(rel))
        self.assertEqual(sorted(frozen), select_set.frozen_paths(str(TASK)))

    def test_every_reference_and_cell_file_is_frozen(self):
        frozen = set(SELECTION["frozen"])
        for folder in ("tests/reference", "solution/reference", "tests/shoecell", "tests/cellkit"):
            for p in (TASK / folder).iterdir():
                if p.is_file():
                    self.assertIn(p.relative_to(TASK).as_posix(), frozen)
        for rel in ("solution/solve.sh", "tests/verify.py", "tests/test.sh", "tests/grader_config.json",
                    "tests/given_hashes.json", "tests/hidden/oracle_token.sha256", "environment/Dockerfile"):
            self.assertIn(rel, frozen)


class AnchorTests(unittest.TestCase):
    def test_the_anchor_is_the_lowest_recorded_verify_of_the_set(self):
        verifies = ANCHOR["verifies"]
        self.assertGreaterEqual(len(verifies), 1)
        self.assertEqual(ANCHOR["value"], min(v["work_score"] for v in verifies))
        for v in verifies:
            self.assertEqual([e["seed"] for e in v["episodes"]], SEEDS["seeds"])
            mean = sum(e["work_score"] for e in v["episodes"]) / len(v["episodes"])
            self.assertAlmostEqual(v["work_score"], mean, places=12)
            self.assertEqual(v["image_id"], SELECTION["image"]["id"])
            self.assertEqual(v["seeds_sha256"], SELECTION["set"]["seeds_sha256"])

    def test_full_at_follows_the_verifies_against_the_selection_runs(self):
        runs = {c["seed"]: c["result"] for c in SELECTION["candidates"] if c["status"] == "kept"}
        identical = all(e["work_score"] == runs[e["seed"]]["work_score"] for v in ANCHOR["verifies"] for e in v["episodes"])
        for v in ANCHOR["verifies"]:
            for e in v["episodes"]:
                self.assertEqual(e["equals_selection_run"], e["work_score"] == runs[e["seed"]]["work_score"])
        if identical:
            self.assertEqual(ANCHOR["full_at"], 0.98)
        else:
            self.assertLess(ANCHOR["full_at"], 0.98)
        for v in ANCHOR["verifies"]:  # the oracle's own verifies read reward 1.0 (half-up, two decimals)
            reward = min(1.0, max(0.0, v["work_score"] / ANCHOR["value"] / ANCHOR["full_at"]))
            self.assertGreaterEqual(reward, 0.995)


class AgentVisibleTests(unittest.TestCase):
    # the PR author's public cell code names its privileged calls (the grader runs this exact code; the calls answer
    # "forbidden" to a program without the token): pinned here as it stands, so no other agent-visible text starts to
    PRIVILEGED = re.compile(r"\bprivileged|reference_run|reference program's token", re.IGNORECASE)
    PINNED = {"environment/setup/payload/given/cell/cell_api.md": 1,
              "environment/setup/payload/given/lib/shoecell/__init__.py": 1,
              "environment/setup/payload/given/lib/shoecell/client.py": 3,
              "environment/setup/payload/given/lib/shoecell/local.py": 1,
              "environment/setup/payload/given/lib/shoecell/server.py": 12,
              "environment/setup/payload/practice/grade.py": 1}

    def test_nothing_agent_visible_names_the_token_the_set_or_its_selection(self):
        token = (TASK / "solution/reference/oracle_token").read_text().strip()
        digest = (HIDDEN / "oracle_token.sha256").read_text().split()[0]
        needles = [token, digest, "oracle_token", "privileged_mode", "seeds.json", "selection.json", "select_set",
                   "run_candidate", "regrade", "fixed set", "fixed hidden", "hidden set", "anchor", SELECTION["format"],
                   SEEDS["source"][:40]] + [str(c["seed"]) for c in SELECTION["candidates"]]
        for p in agent_visible():
            text = p.read_text(errors="replace")
            for needle in needles:
                self.assertNotIn(needle, text, "{} names {}".format(p.relative_to(TASK), needle[:24]))
        self.assertNotIn("fresh seeds", (TASK / "instruction.md").read_text())

    def test_the_cell_library_s_mentions_of_its_privileged_calls_are_the_pinned_ones(self):
        found = {}
        for p in agent_visible():
            n = sum(1 for line in p.read_text(errors="replace").splitlines() if self.PRIVILEGED.search(line))
            if n:
                found[p.relative_to(TASK).as_posix()] = n
        self.assertEqual(found, self.PINNED)


if __name__ == "__main__":
    unittest.main()
