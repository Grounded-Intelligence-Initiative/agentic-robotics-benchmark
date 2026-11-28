"""dexterous_hand_grip_controller_from_docs: the fixed hidden set, the givens and the grading code stay as frozen.

The acceptance runs a fixed hidden set (tests/hidden/scenarios: two hand units, each with two shifts), selected once
by a registered rule and frozen by sha256 with the digest of the code it belongs to. The grader refuses a scenario
whose bytes, code digest or ceilings changed (world.scenarios.load); this test says so at review time already:

  * every kept scenario matches its sha256 in the manifest, and carries the manifest's code digest;
  * the code digest (sha256 of the canonical JSON of the sorted [relpath, sha256] pairs of every file that
    tests/grip/SHIPPED.json lists) equals the manifest's, so no shipped module or data file changed after the freeze;
  * tests/given_hashes.json is exactly the sha256 of every file of tests/setup_payload, and tests/setup_payload is a
    byte copy of environment/setup/payload (the agent and the grader see the same givens);
  * the oracle token's sha256 is recorded and the anchor is the reference's 1.0 under the task's metric name
    (acceptance_score, the name task.toml's [metadata] metric carries).

Standard library only.
"""
import hashlib
import json
import os
import sys
import unittest
from pathlib import Path

sys.dont_write_bytecode = True

SLUG = "dexterous_hand_grip_controller_from_docs"


# a fixed path (the benchmark's tests/ beside tasks/), or ARB_TASK_DIR: a missing folder fails, it never skips
TASK = Path(os.environ.get("ARB_TASK_DIR") or Path(__file__).resolve().parents[1] / "tasks" / "hand" / SLUG)
TESTS = TASK / "tests"
GRIP = TESTS / "grip"
SCEN = TESTS / "hidden" / "scenarios"


def sha(b):
    return hashlib.sha256(b).hexdigest()


def tree(root):
    out = {}
    for dp, dn, fn in os.walk(root):
        dn[:] = [d for d in dn if d != "__pycache__"]
        for f in fn:
            p = Path(dp) / f
            out[p.relative_to(root).as_posix()] = p.read_bytes()
    return out


def code_digest():
    shipped = json.loads((GRIP / "SHIPPED.json").read_text(encoding="utf-8"))
    pairs = sorted([r, sha((GRIP / r).read_bytes())] for r in shipped["modules"] + shipped["data"])
    return sha(json.dumps(pairs, separators=(",", ":"), ensure_ascii=True).encode("ascii"))


class FrozenSet(unittest.TestCase):
    def test_the_kept_scenarios_match_the_manifest(self):
        man = json.loads((SCEN / "manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(len(man["kept"]), 2)
        for k in man["kept"]:
            raw = (SCEN / k["file"]).read_bytes()
            self.assertEqual(sha(raw), k["sha256"], k["file"])
            self.assertEqual(json.loads(raw.decode("ascii"))["code_digest"], man["code_digest"], k["file"])

    def test_the_code_digest_is_the_frozen_one(self):
        man = json.loads((SCEN / "manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(code_digest(), man["code_digest"])

    def test_the_givens_are_the_hashed_ones_and_mirrored(self):
        payload = tree(TASK / "environment" / "setup" / "payload")
        mirror = tree(TESTS / "setup_payload")
        self.assertEqual(sorted(payload), sorted(mirror))
        for rel in payload:
            self.assertEqual(payload[rel], mirror[rel], rel)
        hashes = json.loads((TESTS / "given_hashes.json").read_text(encoding="utf-8"))
        self.assertEqual(hashes, {rel: sha(b) for rel, b in mirror.items()})

    def test_the_token_and_the_anchor(self):
        tok = (TESTS / "hidden" / "oracle_token.sha256").read_text(encoding="ascii").strip()
        self.assertRegex(tok, r"^[0-9a-f]{64}$")
        anchor = json.loads((TESTS / "anchor.json").read_text(encoding="utf-8"))
        self.assertEqual((anchor["value"], anchor["full_at"], anchor["metric"]), (1.0, 1.0, "acceptance_score"))


if __name__ == "__main__":
    unittest.main()
