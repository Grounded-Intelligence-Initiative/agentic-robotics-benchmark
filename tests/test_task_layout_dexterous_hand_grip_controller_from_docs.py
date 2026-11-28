"""dexterous_hand_grip_controller_from_docs: the Harbor layout, and what may reach the agent's image.

The grader's files (tests/) and the reference submission (solution/) must not reach the image or the agent home: no
file of tests/ or solution/ is byte-identical to a file of the image's build context (environment/ less setup/) or of
the givens (environment/setup/payload/), except tests/setup_payload/ (the grader's byte copy of the givens, which is
agent-visible by design). tests/given_hashes.json is exactly the sha256 of every given; the oracle token's hash is
pinned and the token appears nowhere else in the task. Every agent-visible text is rendered: no template placeholder
({NAME[key]} or a doubled brace) is left in the instruction or in a given document. The metric named by
tests/anchor.json is the one task.toml names. The image's working directory is / (the last instruction of
environment/Dockerfile), not the agent's writable home, where the engine's root commands of the agent's sandbox would
otherwise start. Stdlib only; nothing is downloaded.
"""
import hashlib
import json
import os
import re
import sys
import tomllib
import unittest
from pathlib import Path

sys.dont_write_bytecode = True

SLUG = "dexterous_hand_grip_controller_from_docs"
TASK = Path(os.environ.get("ARB_TASK_DIR") or Path(__file__).resolve().parents[1] / "tasks" / "hand" / SLUG)
TEXT = (".md", ".txt", ".json", ".csv", ".py")
# a str.format-style placeholder with an index ({INTERFACE[size_cap_mb]}) or a dotted one ({SERVO.stall_nm}) that a
# renderer left behind, and the doubled opening brace of str.format's escapes (a closing pair is legal JSON)
LEFT_PLACEHOLDER = re.compile(r"\{[A-Z][A-Z0-9_]*(?:\[[^\]\n]*\]|\.[A-Za-z0-9_.]+)+\}|\{\{")


def _sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def _tree(root):
    out = {}
    for d, dirs, files in os.walk(root):
        dirs[:] = sorted(x for x in dirs if x != "__pycache__")
        for f in sorted(files):
            p = os.path.join(d, f)
            out[os.path.relpath(p, root).replace(os.sep, "/")] = _sha(p)
    return out


class TaskLayout(unittest.TestCase):
    def test_top_level_is_the_harbor_layout(self):
        names = {p.name for p in TASK.iterdir()} - {".ale-cache"}
        self.assertEqual(names, {"task.toml", "instruction.md", "environment", "tests", "solution"})

    def test_manifest(self):
        cfg = tomllib.loads((TASK / "task.toml").read_text(encoding="utf-8"))
        self.assertEqual(cfg["schema_version"], "1.4+ale.1")
        self.assertEqual(cfg["task"]["name"], "grounded-intelligence-initiative/" + SLUG)
        self.assertEqual(cfg["metadata"]["platform"], "hand")
        self.assertEqual(cfg["artifacts"], ["/home/user/submission"])
        self.assertEqual((cfg["agent"]["timeout_sec"], cfg["agent"]["network_mode"]), (21600, "no-network"))
        self.assertEqual(cfg["verifier"]["environment_mode"], "separate")
        ve = cfg["verifier"]["environment"]
        self.assertEqual(ve["gpus"], 0)
        self.assertNotIn("gpu_types", cfg["environment"])

    def test_the_anchor_names_the_manifests_metric(self):
        cfg = tomllib.loads((TASK / "task.toml").read_text(encoding="utf-8"))
        anchor = json.loads((TASK / "tests" / "anchor.json").read_text(encoding="utf-8"))
        self.assertEqual(anchor["metric"], cfg["metadata"]["metric"]["name"])

    def test_no_bytecode_in_the_task(self):
        found = [str(p.relative_to(TASK)) for p in TASK.rglob("*") if p.name == "__pycache__" or p.suffix == ".pyc"]
        self.assertEqual(found, [])

    def test_grader_and_reference_never_reach_the_image(self):
        hidden = {"tests/" + r: h for r, h in _tree(TASK / "tests").items()}
        hidden.update({"solution/" + r: h for r, h in _tree(TASK / "solution").items()})
        visible = {}
        for rel, digest in _tree(TASK / "environment").items():
            if not rel.startswith("setup/"):
                visible.setdefault(digest, []).append("environment/" + rel)
        for rel, digest in _tree(TASK / "environment" / "setup" / "payload").items():
            visible.setdefault(digest, []).append("payload/" + rel)
        empty = hashlib.sha256(b"").hexdigest()
        bad = [(rel, visible[d]) for rel, d in sorted(hidden.items())
               if d in visible and not rel.startswith("tests/setup_payload/") and d != empty]
        self.assertEqual(bad, [])

    def test_setup_payload_is_the_givens_byte_for_byte(self):
        self.assertEqual(_tree(TASK / "tests" / "setup_payload"), _tree(TASK / "environment" / "setup" / "payload"))

    def test_given_hashes_pin_the_givens(self):
        pin = json.loads((TASK / "tests" / "given_hashes.json").read_text(encoding="utf-8"))
        self.assertEqual(pin, _tree(TASK / "environment" / "setup" / "payload"))

    def test_the_oracle_token_lives_in_solution_only(self):
        tok_path = TASK / "solution" / "reference" / "oracle_token"
        pin_path = TASK / "tests" / "hidden" / "oracle_token.sha256"
        if not tok_path.is_file() or not pin_path.is_file():
            self.fail("no oracle token / no pinned hash yet (the hidden set is not frozen)")
        token = tok_path.read_bytes().strip()
        self.assertGreaterEqual(len(token), 32)
        self.assertEqual(hashlib.sha256(token).hexdigest(), pin_path.read_text(encoding="utf-8").split()[0])
        for p in TASK.rglob("*"):
            if p.is_file() and p != tok_path and p.stat().st_size < (8 << 20):
                self.assertNotIn(token, p.read_bytes(), str(p.relative_to(TASK)))

    def test_agent_visible_text_is_rendered(self):
        files = [TASK / "instruction.md"] + [p for p in (TASK / "environment" / "setup" / "payload").rglob("*")
                                              if p.is_file() and p.suffix in TEXT]
        left = []
        for p in files:
            text = p.read_text(encoding="utf-8", errors="replace")
            if p.suffix == ".py":          # code may hold braces; check only its docstrings and comments
                text = "\n".join(ln for ln in text.splitlines() if ln.lstrip().startswith("#"))
            for n, line in enumerate(text.splitlines(), 1):
                if "$$" in line or "\\" in line:          # LaTeX in the makers' notes
                    continue
                if LEFT_PLACEHOLDER.search(line):
                    left.append("%s:%d: %s" % (p.relative_to(TASK), n, line.strip()[:90]))
        self.assertEqual(left, [], "unrendered template text in agent-visible files:\n" + "\n".join(left[:20]))

    def test_the_images_working_directory_is_root(self):
        docker = (TASK / "environment" / "Dockerfile").read_text(encoding="utf-8")
        instructions = [ln.split()[0] for ln in docker.splitlines()
                        if re.match(r"^(FROM|RUN|COPY|ADD|ENV|WORKDIR|USER|ENTRYPOINT|CMD)\b", ln)]
        self.assertEqual(instructions[-1], "WORKDIR")
        self.assertEqual(docker.strip().splitlines()[-1], "WORKDIR /")
        self.assertEqual([ln for ln in docker.splitlines() if ln.startswith("WORKDIR")], ["WORKDIR /"])


if __name__ == "__main__":
    unittest.main()
