"""dexterous_hand_robot_description_from_photos: the Harbor layout, the frozen hidden set, and what may reach the agent.

The grader checks its own files when verify starts (tests/verify.py check_task_files): every file of tests/gt/ against
tests/gt/MANIFEST.sha256, the hidden set's code digests against the shipped grading modules and grader_config.json,
and the hidden set's counts against grader_config.json; a mismatch is a TASK ERROR on every run, whatever the agent
handed in. A maintenance edit of one of those files therefore breaks every future episode without failing anything
else in CI: this test runs the same comparisons (digests only; no hidden value is read or printed).

Also: the top level is the Harbor layout and task.toml holds the registered values; no bytecode; the agent's checker
(environment/setup/payload/checker/) is the grader's gate code byte for byte; no other file of tests/ or solution/ is
byte-identical to a file the agent's image or home receives (the build context, the inline payload and the files of
the asset list); the reference (solution/answer/) is the photographed unit's own description, byte-identical to
tests/gt/; tests/given_hashes.json pins exactly the givens: the small text files inline in environment/setup/payload/
and the binary givens (the part meshes, the photographs, the figures) named by environment/assets.sha256, which the
image build fetches from the asset dataset, pinned by revision and sha256. Where the Dockerfile fetches, the asset list
must exist, name the task's prefix, the repository of task-assets.toml and a dataset commit (the placeholder revision
of forty zeros is refused), the fetcher must be the shared one, and no binary or large (> 512 KiB) file may be in
environment/. Stdlib only; nothing is downloaded.
"""
import hashlib
import json
import os
import tomllib
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SLUG = "dexterous_hand_robot_description_from_photos"
TASK = REPO / "tasks" / "hand" / SLUG
TESTS = TASK / "tests"
PAYLOAD = TASK / "environment" / "setup" / "payload"
CHECKER = ("calib.py", "contract.py", "gate.py", "whitelist.py")      # the agent's checker = the grader's gate code
CODE_FILES = {"kin.py": "hand_desc/kin.py", "calib.py": "hand_desc/calib.py", "hidden_unit.py": "hand_desc/hidden_unit.py",
              "ik.py": "hand_desc/ik.py", "collide.py": "hand_desc/collide.py", "score.py": "hand_desc/score.py",
              "grader_config.json": "grader_config.json"}
PLACEHOLDER_REVISION = "0" * 40
FETCH_RUN = "RUN python3 /opt/task-assets/fetch_assets.py /opt/task-assets/assets.sha256 /opt/task-assets/payload"
BINARY_SUFFIXES = (".stl", ".jpg", ".jpeg", ".png", ".step", ".stp", ".obj", ".npz", ".npy", ".zip", ".gz", ".xz", ".mp4")
MAX_GIT_BYTES = 512 * 1024


def _sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def _tree(root):
    out = {}
    if not root.is_dir():
        return out
    for d, dirs, files in os.walk(root):
        dirs[:] = sorted(x for x in dirs if x != "__pycache__")
        for f in sorted(files):
            p = os.path.join(d, f)
            out[os.path.relpath(p, root).replace(os.sep, "/")] = _sha(p)
    return out


def _asset_list():
    path = TASK / "environment" / "assets.sha256"
    head, entries = {}, {}
    if not path.is_file():
        return None, {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        if line.startswith("#"):
            key, _, value = line[1:].partition(":")
            head[key.strip()] = value.strip()
            continue
        digest, rel = line.split(None, 1)
        entries[rel.strip().lstrip("*")] = digest.lower()
    return head, entries


class Layout(unittest.TestCase):
    def test_top_level_is_the_harbor_layout(self):
        names = {p.name for p in TASK.iterdir()} - {".ale-cache"}
        self.assertEqual(names, {"task.toml", "instruction.md", "environment", "tests", "solution"})

    def test_manifest(self):
        cfg = tomllib.loads((TASK / "task.toml").read_text(encoding="utf-8"))
        self.assertEqual(cfg["schema_version"], "1.4+ale.1")
        self.assertEqual(cfg["task"]["name"], "grounded-intelligence-initiative/" + SLUG)
        self.assertEqual(cfg["metadata"]["platform"], "hand")
        self.assertEqual(cfg["artifacts"], ["/home/user/submission"])
        self.assertEqual((cfg["environment"]["cpus"], cfg["environment"]["memory_mb"]), (8, 16384))
        self.assertEqual((cfg["agent"]["timeout_sec"], cfg["agent"]["network_mode"]), (21600, "no-network"))
        self.assertEqual((cfg["verifier"]["environment_mode"], cfg["verifier"]["timeout_sec"]), ("separate", 2400))
        ve = cfg["verifier"]["environment"]
        self.assertEqual((ve["cpus"], ve["memory_mb"], ve["gpus"]), (2, 8192, 0))

    def test_no_bytecode_in_the_task(self):
        found = [str(p.relative_to(TASK)) for p in TASK.rglob("*") if p.name == "__pycache__" or p.suffix == ".pyc"]
        self.assertEqual(found, [])


class FrozenHiddenSet(unittest.TestCase):
    """What check_task_files compares when verify starts, compared here first."""

    def test_gt_equals_its_manifest(self):
        listed = {}
        for line in (TESTS / "gt" / "MANIFEST.sha256").read_text(encoding="utf-8").splitlines():
            if line.strip():
                digest, rel = line.split("  ", 1)
                listed[rel] = digest
        present = {k: v for k, v in _tree(TESTS / "gt").items() if k != "MANIFEST.sha256"}
        self.assertEqual(sorted(present), sorted(listed))
        self.assertEqual([k for k in listed if listed[k] != present[k]], [])

    def test_the_hidden_set_was_drawn_with_the_shipped_grading_files(self):
        hs = json.loads((TESTS / "gt" / "hidden_set.json").read_text(encoding="utf-8"))
        self.assertEqual(hs.get("schema"), "hd04-set/1")
        code = hs.get("code_sha256") or {}
        self.assertEqual(sorted(code), sorted(CODE_FILES))
        self.assertEqual([n for n, rel in sorted(CODE_FILES.items()) if _sha(TESTS / rel) != code[n]], [])
        cfg = json.loads((TESTS / "grader_config.json").read_text(encoding="utf-8"))
        self.assertEqual([g for g, n in sorted(cfg["counts"].items()) if len(hs.get(g, [])) != int(n)], [])

    def test_the_test_mirrors_the_graders_list(self):
        import ast
        tree = ast.parse((TESTS / "verify.py").read_text(encoding="utf-8"))
        node = next(n for n in tree.body if isinstance(n, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == "CODE_FILES" for t in n.targets))
        self.assertEqual(ast.literal_eval(node.value), CODE_FILES)

    def test_the_reference_is_the_units_own_description(self):
        self.assertEqual(_sha(TASK / "solution" / "answer" / "robot.xml"), _sha(TESTS / "gt" / "robot.xml"))
        self.assertEqual(_tree(TASK / "solution" / "answer" / "assets"), _tree(TESTS / "gt" / "assets"))
        self.assertTrue((TASK / "solution" / "answer" / "config.yaml").is_file())


class WhatReachesTheAgent(unittest.TestCase):
    def test_the_checker_is_the_graders_gate_code(self):
        for name in CHECKER:
            self.assertEqual(_sha(PAYLOAD / "checker" / name), _sha(TESTS / "hand_desc" / name), name)

    def test_no_other_hidden_file_reaches_the_image_or_the_home(self):
        visible = {}
        for rel, digest in _tree(TASK / "environment").items():
            visible.setdefault(digest, []).append("environment/" + rel)
        for rel, digest in _asset_list()[1].items():
            visible.setdefault(digest, []).append("assets/" + rel)
        allowed = {_sha(TESTS / "hand_desc" / n) for n in CHECKER} | {hashlib.sha256(b"").hexdigest()}
        hidden = {"tests/" + r: h for r, h in _tree(TESTS).items()}
        hidden.update({"solution/" + r: h for r, h in _tree(TASK / "solution").items()})
        bad = sorted((rel, visible[d][:2]) for rel, d in hidden.items() if d in visible and d not in allowed)
        self.assertEqual(bad, [])

    def test_given_hashes_pin_the_givens(self):
        pin = json.loads((TESTS / "given_hashes.json").read_text(encoding="utf-8"))
        files = {k: (v["sha256"] if isinstance(v, dict) else v) for k, v in pin["files"].items()}
        expect = dict(_tree(PAYLOAD))
        assets = _asset_list()[1]
        self.assertFalse(set(expect) & set(assets), "a given both inline and in the asset list")
        expect.update(assets)
        self.assertEqual(sorted(files), sorted(expect))
        self.assertEqual([k for k in files if files[k] != expect[k]], [])

    def test_the_asset_list_once_there(self):
        head, entries = _asset_list()
        docker = (TASK / "environment" / "Dockerfile").read_text(encoding="utf-8")
        if head is None:
            self.assertNotIn(FETCH_RUN, docker, "the Dockerfile fetches task assets but there is no asset list")
            self.skipTest("the payload is still inline (no environment/assets.sha256)")
        conf = tomllib.loads((REPO / "task-assets.toml").read_text(encoding="utf-8"))["huggingface"]
        self.assertEqual(head.get("prefix"), "tasks/hand/%s/payload" % SLUG)
        self.assertEqual(head.get("repo"), conf["dataset"])
        self.assertEqual(head.get("endpoint"), conf["endpoint"].rstrip("/"))
        self.assertRegex(head.get("revision", ""), r"^[0-9a-f]{40}$")
        self.assertNotEqual(head.get("revision"), PLACEHOLDER_REVISION,
                            "the asset list still names the placeholder revision, not the dataset commit")
        self.assertTrue(entries, "the asset list names no file")
        for rel in entries:
            self.assertFalse(".." in rel.split("/") or rel.startswith("/"), rel)
        self.assertEqual(docker.count(FETCH_RUN), 1, "the Dockerfile fetches the asset list once")
        self.assertEqual(_sha(TASK / "environment" / "fetch_assets.py"),
                         _sha(REPO / "shared" / "task_assets" / "fetch_assets.py"))
        setup = (TASK / "environment" / "setup" / "setup.sh").read_text(encoding="utf-8")
        self.assertIn('cp -r /opt/task-assets/payload/. "$HOME_DIR/"', setup)
        in_git = []
        for rel in _tree(TASK / "environment"):
            p = TASK / "environment" / rel
            if rel.lower().endswith(BINARY_SUFFIXES) or p.stat().st_size > MAX_GIT_BYTES:
                in_git.append(rel)
        self.assertEqual(in_git, [], "a binary or large given is in git, not in the asset list")


if __name__ == "__main__":
    unittest.main()
