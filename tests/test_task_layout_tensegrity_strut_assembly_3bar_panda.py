"""tensegrity_strut_assembly_3bar_panda: the Harbor layout and what may reach the agent's image.

The grader's files (tests/), the reference submission (solution/) and the cell's hidden kit (the tarball pinned in
tests/grader_config.json hidden_gt) must not reach the image or the agent home: no file of tests/ or solution/ is
byte-identical to a file of the image's build context (environment/ less setup/), of the small givens
(environment/setup/payload/) or of the payload the image fetches (environment/assets.sha256), and the hidden kit's
sha256 is none of them. The only exceptions are by design: tests/setup_payload/ (the grader's byte copy of
environment/setup/payload/, which is agent-visible) and the precedent's library modules that ship byte-identical to the
agent (environment/setup/payload/given/lib/) and to the grader. The asset list holds the payload only (the modules'
STLs, the picture manual, the Panda model, the cameras' scenery) under this task's prefix; tests/given_hashes.json, the
grader's pin of the givens, equals the small givens and the asset list; the oracle token's hash is pinned and the token
appears nowhere else in the task. Stdlib only; nothing is downloaded (the payload and the hidden kit are not in git).
"""
import hashlib
import json
import os
import re
import tomllib
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SLUG = "tensegrity_strut_assembly_3bar_panda"
TASK = REPO / "tasks" / "soft" / SLUG
PAYLOAD_DIRS = ("given/parts/", "given/manual/", "given/panda/", "render/bench/", "render/room/")


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


def _asset_list():
    head, entries = {}, {}
    for line in (TASK / "environment" / "assets.sha256").read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        if line.startswith("#"):
            key, _, value = line[1:].partition(":")
            head[key.strip()] = value.strip()
            continue
        digest, rel = line.split(None, 1)
        entries[rel.strip().lstrip("*")] = digest.lower()
    return head, entries


class TaskLayout(unittest.TestCase):
    def test_top_level_is_the_harbor_layout(self):
        names = {p.name for p in TASK.iterdir()} - {".ale-cache"}
        self.assertEqual(names, {"task.toml", "instruction.md", "environment", "tests", "solution"})

    def test_manifest(self):
        cfg = tomllib.loads((TASK / "task.toml").read_text(encoding="utf-8"))
        self.assertEqual(cfg["schema_version"], "1.4+ale.1")
        self.assertEqual(cfg["task"]["name"], "grounded-intelligence-initiative/" + SLUG)
        self.assertEqual(cfg["metadata"]["platform"], "soft")
        # dual mode (experimental): the graded episode's record directory is the second artifact, and the session is
        # main's 6 h (the start window) plus the 3 h graded window
        self.assertEqual(cfg["artifacts"], ["/home/user/submission", "/var/lib/arb-online"])
        self.assertEqual((cfg["environment"]["cpus"], cfg["environment"]["memory_mb"]), (8, 16384))
        self.assertEqual((cfg["agent"]["timeout_sec"], cfg["agent"]["network_mode"]), (21600 + 10800, "no-network"))
        self.assertEqual((cfg["verifier"]["environment_mode"], cfg["verifier"]["timeout_sec"]), ("separate", 14400))
        ve = cfg["verifier"]["environment"]
        self.assertEqual((ve["cpus"], ve["memory_mb"], ve["gpus"]), (8, 24576, 0))

    def test_no_bytecode_in_the_task(self):
        found = [str(p.relative_to(TASK)) for p in TASK.rglob("*") if p.name == "__pycache__" or p.suffix == ".pyc"]
        self.assertEqual(found, [])

    def test_asset_list_is_the_payload_only(self):
        head, entries = _asset_list()
        conf = tomllib.loads((REPO / "task-assets.toml").read_text(encoding="utf-8"))["huggingface"]
        self.assertEqual(head.get("prefix"), "tasks/soft/%s/payload" % SLUG)
        self.assertEqual(head.get("repo"), conf["dataset"])
        self.assertEqual(head.get("endpoint"), conf["endpoint"].rstrip("/"))
        self.assertRegex(head.get("revision", ""), r"^[0-9a-f]{40}$")
        self.assertTrue(entries)
        for rel, digest in entries.items():
            self.assertTrue(rel.startswith(PAYLOAD_DIRS), "not a payload file: %s" % rel)
            self.assertFalse(".." in rel.split("/") or rel.startswith("/"), rel)
            self.assertRegex(digest, r"^[0-9a-f]{64}$")
        for want in ("given/parts/M0_strut_core.stl", "given/manual/photos.json", "given/panda/panda.xml", "render/room/room.json"):
            self.assertIn(want, entries)

    def test_grader_reference_and_kit_never_reach_the_image(self):
        hidden = {"tests/" + r: h for r, h in _tree(TASK / "tests").items()}
        hidden.update({"solution/" + r: h for r, h in _tree(TASK / "solution").items()})
        lib = set(_tree(TASK / "environment" / "setup" / "payload" / "given" / "lib").values())
        visible = {}
        for rel, digest in _tree(TASK / "environment").items():
            if not rel.startswith("setup/"):
                visible.setdefault(digest, []).append("environment/" + rel)
        for rel, digest in _tree(TASK / "environment" / "setup" / "payload").items():
            visible.setdefault(digest, []).append("payload/" + rel)
        for rel, digest in _asset_list()[1].items():
            visible.setdefault(digest, []).append("assets/" + rel)
        bad = []
        for rel, digest in sorted(hidden.items()):
            if digest not in visible:
                continue
            if rel.startswith("tests/setup_payload/") or digest in lib or digest == hashlib.sha256(b"").hexdigest():
                continue
            bad.append((rel, visible[digest]))
        self.assertEqual(bad, [])
        gt = json.loads((TASK / "tests" / "grader_config.json").read_text(encoding="utf-8"))["hidden_gt"]
        self.assertRegex(gt["sha256"], r"^[0-9a-f]{64}$")
        self.assertNotIn(gt["sha256"], visible)
        self.assertTrue(gt["url"].endswith("tasks/soft/%s/verifier/gt.tar.xz" % SLUG), gt["url"])
        self.assertIsInstance(gt["files"], int)
        local = TASK / "tests" / "assets" / "gt.tar.xz"
        if local.is_file():
            self.assertEqual(_sha(local), gt["sha256"])

    def test_setup_payload_is_the_givens_byte_for_byte(self):
        self.assertEqual(_tree(TASK / "tests" / "setup_payload"), _tree(TASK / "environment" / "setup" / "payload"))

    def test_given_hashes_pin_the_givens_and_the_payload(self):
        pin = json.loads((TASK / "tests" / "given_hashes.json").read_text(encoding="utf-8"))
        expect = dict(_tree(TASK / "environment" / "setup" / "payload"))
        assets = _asset_list()[1]
        self.assertFalse(set(expect) & set(assets))
        expect.update(assets)
        self.assertEqual(pin, expect)

    def test_the_oracle_token_lives_in_solution_only(self):
        token = (TASK / "solution" / "reference" / "oracle_token").read_bytes().strip()
        self.assertGreaterEqual(len(token), 32)
        pinned = (TASK / "tests" / "hidden" / "oracle_token.sha256").read_text(encoding="utf-8").split()[0]
        self.assertEqual(hashlib.sha256(token).hexdigest(), pinned)
        for p in TASK.rglob("*"):
            if p.is_file() and p != TASK / "solution" / "reference" / "oracle_token" and p.stat().st_size < (8 << 20):
                self.assertNotIn(token, p.read_bytes(), str(p.relative_to(TASK)))

    def test_online_cell_is_the_verifiers_byte_for_byte(self):
        """dual mode: the graded episode's service serves the verifier's own cell (environment/setup/online/cell is a
        byte copy of the cell's code and data in tests/), its episode is the one tests/online.json names, and the oracle
        token it accepts is the task's"""
        online = TASK / "environment" / "setup" / "online"
        cell = online / "cell"
        want = {}
        for pkg in ("arm", "common", "render", "render_filament", "room"):
            want.update({pkg + "/" + r: h for r, h in _tree(TASK / "tests" / pkg).items()})
        for src, dst in (("episode_trace.py", "episode_trace.py"), ("grader_config.json", "grader_config.json"),
                         ("kit_info.json", "kit_info.json"), ("hidden/bench_plan.json", "bench_plan.json")):
            want[dst] = _sha(TASK / "tests" / src)
        self.assertEqual(_tree(cell), want)
        ep = json.loads((online / "episode.json").read_text(encoding="utf-8"))
        rec = json.loads((TASK / "tests" / "online.json").read_text(encoding="utf-8"))
        self.assertEqual((ep["episode"], ep["seed_sha256"]), (rec["episode"], rec["seed_sha256"]))
        self.assertEqual(hashlib.sha256(str(int(ep["seed"])).encode("ascii")).hexdigest(), ep["seed_sha256"])
        self.assertLess(int(ep["seed"]), 1 << 63)
        pinned = (TASK / "tests" / "hidden" / "oracle_token.sha256").read_text(encoding="utf-8").split()[0]
        self.assertEqual(ep["oracle_token_sha256"], pinned)
        cfg = json.loads((online / "online.json").read_text(encoding="utf-8"))
        task = tomllib.loads((TASK / "task.toml").read_text(encoding="utf-8"))
        self.assertEqual(cfg["prep_s"] + cfg["graded_s"], task["agent"]["timeout_sec"])
        self.assertEqual(cfg["task"], SLUG)
        offline = json.loads((TASK / "tests" / "anchor_offline.json").read_text(encoding="utf-8"))
        self.assertEqual(offline["metric"], "assembly_score")

    def test_render_omit_list_is_pinned(self):
        cfg = json.loads((TASK / "tests" / "grader_config.json").read_text(encoding="utf-8"))
        omit = TASK / "tests" / "arm" / "assembly_strut" / "render_omit.json"
        self.assertEqual(_sha(omit), cfg["agent_render"]["omit_list_sha256"])
        self.assertEqual(cfg["scope"], "default")
        self.assertTrue(re.match(r"^[0-9a-f]{64}$", cfg["agent_render"]["omit_list_sha256"]))


if __name__ == "__main__":
    unittest.main()
