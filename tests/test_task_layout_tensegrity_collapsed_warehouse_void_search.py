"""tensegrity_collapsed_warehouse_void_search: the Harbor layout and what may reach the agent's image.

The grader's files (tests/), the reference submission (solution/), the hidden site packages and the oracle token must
not reach the image or the agent home: no file of tests/ or solution/ is byte-identical to a file of the image's build
context (environment/ less setup/) or of the givens (environment/setup/payload/), with exactly the declared
exceptions: the practice ground's copy of the cell (environment/practice_ground/voidcell/ = tests/voidcell/ plus
tests/voidscore/ as voidcell/voidscore/, installed for the practice ground's own account only, mode 700), the given
client library (environment/setup/payload/given/lib/voidcell/client.py = tests/voidcell/client.py) and
tests/setup_payload/ (the grader's byte copy of the givens). The practice ground holds no reference program, no token
and no hidden site package. Every agent-visible file is a small text file or picture in git (no asset dataset, no
fetched payload). tests/given_hashes.json, the grader's pin of the givens, equals the givens. The image installs only
requirements.txt and practice_ground/ and derives from the pinned base. Stdlib only; nothing is downloaded or run.
"""
import hashlib
import json
import os
import re
import tomllib
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SLUG = "tensegrity_collapsed_warehouse_void_search"
TASK = REPO / "tasks" / "soft" / SLUG
ENV = TASK / "environment"
PAYLOAD = ENV / "setup" / "payload"
PRACTICE = ENV / "practice_ground"
TESTS = TASK / "tests"
SOLUTION = TASK / "solution"
EMPTY = hashlib.sha256(b"").hexdigest()


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
        self.assertEqual(cfg["metadata"]["platform"], "soft")
        self.assertEqual(cfg["artifacts"], ["/home/user/submission"])
        self.assertEqual((cfg["environment"]["cpus"], cfg["environment"]["memory_mb"]), (8, 16384))
        self.assertEqual((cfg["agent"]["timeout_sec"], cfg["agent"]["network_mode"]), (14400, "no-network"))
        self.assertEqual(cfg["verifier"]["environment_mode"], "separate")
        # the floor for ten serial worst-case episodes plus one re-run: at least 4.5 h
        self.assertGreaterEqual(cfg["verifier"]["timeout_sec"], 16200)
        ve = cfg["verifier"]["environment"]
        self.assertEqual((ve["cpus"], ve["memory_mb"], ve["gpus"]), (8, 16384, 0))

    def test_no_bytecode_in_the_task(self):
        found = [str(p.relative_to(TASK)) for p in TASK.rglob("*")
                 if p.name in ("__pycache__", ".pytest_cache") or p.suffix == ".pyc"]
        self.assertEqual(found, [])

    def test_givens_are_small_files_in_git(self):
        self.assertFalse((ENV / "assets.sha256").exists(), "this task fetches no payload")
        self.assertFalse((ENV / "fetch_assets.py").exists())
        tree = _tree(PAYLOAD)
        self.assertTrue(tree)
        total = 0
        for rel in tree:
            size = (PAYLOAD / rel).stat().st_size
            total += size
            self.assertLess(size, 512 * 1024, rel)
            self.assertTrue(rel.endswith((".md", ".json", ".png", ".py")), rel)
        self.assertLess(total, 2 * 1024 * 1024)
        for want in ("given/robot_handbook.md", "given/station_api.md", "given/site_briefing.md",
                     "given/camera_datasheet.md", "given/calibration_site/calibration_site.md",
                     "given/calibration_site/site_plan.json", "given/calibration_site/site_plan.png",
                     "given/lib/voidcell/__init__.py", "given/lib/voidcell/client.py", "example/operate.py",
                     "training/run_mission.py"):
            self.assertIn(want, tree)

    def test_the_given_client_is_the_graders(self):
        self.assertEqual(_sha(PAYLOAD / "given" / "lib" / "voidcell" / "client.py"), _sha(TESTS / "voidcell" / "client.py"))

    def test_practice_ground_is_the_cell_copy_without_the_hidden_side(self):
        pg = _tree(PRACTICE / "voidcell")
        cell = {r: h for r, h in pg.items() if not r.startswith("voidscore/")}
        score = {r[len("voidscore/"):]: h for r, h in pg.items() if r.startswith("voidscore/")}
        self.assertEqual(cell, _tree(TESTS / "voidcell"))
        self.assertEqual(score, _tree(TESTS / "voidscore"))
        names = [r for r in _tree(PRACTICE) if re.search(r"(reference|voidref|oracle|token|hidden|selection|seeds|"
                                                         r"anchor|limits\.json|grader_config)", r)]
        self.assertEqual(names, [])

    def test_grader_reference_and_hidden_sites_never_reach_the_image(self):
        hidden = {"tests/" + r: h for r, h in _tree(TESTS).items()}
        hidden.update({"solution/" + r: h for r, h in _tree(SOLUTION).items()})
        visible = {}
        for rel, digest in _tree(ENV).items():
            if not rel.startswith("setup/"):
                visible.setdefault(digest, []).append("environment/" + rel)
        for rel, digest in _tree(PAYLOAD).items():
            visible.setdefault(digest, []).append("payload/" + rel)
        bad = []
        for rel, digest in sorted(hidden.items()):
            if digest not in visible or digest == EMPTY:
                continue
            where = visible[digest]
            if rel.startswith("tests/setup_payload/"):
                continue                                            # the grader's copy of the givens
            if rel.startswith(("tests/voidcell/", "tests/voidscore/")) and all(
                    w.startswith("environment/practice_ground/voidcell/") or w == "payload/given/lib/voidcell/client.py"
                    for w in where):
                continue                                            # the declared copies of the cell
            bad.append((rel, where))
        self.assertEqual(bad, [])
        for rel in _tree(TESTS / "hidden"):
            self.assertNotIn(_sha(TESTS / "hidden" / rel), visible, rel)
        for rel in _tree(TESTS / "reference"):
            self.assertNotIn(_sha(TESTS / "reference" / rel), visible, rel)

    def test_setup_payload_is_the_givens_byte_for_byte(self):
        self.assertEqual(_tree(TESTS / "setup_payload"), _tree(PAYLOAD))

    def test_given_hashes_pin_the_givens(self):
        pin = json.loads((TESTS / "given_hashes.json").read_text(encoding="utf-8"))
        self.assertEqual(pin, _tree(PAYLOAD))

    def test_the_oracle_token_lives_in_solution_only(self):
        token = (SOLUTION / "reference" / "oracle_token").read_bytes().strip()
        self.assertGreaterEqual(len(token), 32)
        pinned = (TESTS / "hidden" / "oracle_token.sha256").read_text(encoding="utf-8").split()[0]
        self.assertEqual(hashlib.sha256(token).hexdigest(), pinned)
        for p in TASK.rglob("*"):
            if p.is_file() and p != SOLUTION / "reference" / "oracle_token":
                data = p.read_bytes()
                self.assertNotIn(token, data, str(p.relative_to(TASK)))
                if p.is_relative_to(ENV):
                    self.assertNotIn(pinned.encode(), data, str(p.relative_to(TASK)))

    def test_hidden_set_is_pinned(self):
        seeds = json.loads((TESTS / "hidden" / "seeds.json").read_text(encoding="utf-8"))
        cfg = json.loads((TESTS / "grader_config.json").read_text(encoding="utf-8"))
        self.assertEqual(len(seeds["sites"]), cfg["episodes"])
        for row in seeds["sites"]:
            self.assertEqual(_sha(TESTS / "hidden" / row["archive"]), row["sha256"], row["tag"])
        practice = json.loads((PRACTICE / "sites.json").read_text(encoding="utf-8"))
        for row in practice["sites"]:
            self.assertEqual(_sha(PRACTICE / row["package"]), row["sha256"], row["name"])
        hidden_hashes = {row["sha256"] for row in seeds["sites"]}
        self.assertFalse(hidden_hashes & {row["sha256"] for row in practice["sites"]})

    def test_dockerfile_installs_only_the_environment(self):
        text = (ENV / "Dockerfile").read_text(encoding="utf-8")
        body = [ln for ln in text.splitlines() if ln.strip() and not ln.lstrip().startswith("#")]
        froms = [ln for ln in body if re.match(r"^FROM\b", ln)]
        self.assertEqual(froms, ["FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1"])
        copies = [ln.split()[1:] for ln in body if re.match(r"^(COPY|ADD)\b", ln)]
        self.assertEqual(copies, [["--from=ghcr.io/astral-sh/uv:0.11", "/uv", "/usr/local/bin/uv"],
                                  ["requirements.txt", "/tmp/requirements.txt"],
                                  ["practice_ground", "/opt/voidcell-practice"]])
        self.assertFalse([ln for ln in body if re.match(r"^\s*ENTRYPOINT\b", ln)])
        self.assertIn("chown -R voidcell:voidcell /opt/voidcell-practice", text)
        self.assertIn("chmod -R u+rwX,go-rwx /opt/voidcell-practice", text)
        self.assertIn("! runuser -u user -- ls /opt/voidcell-practice", text)
        setup = (ENV / "setup" / "setup.sh").read_text(encoding="utf-8")
        self.assertIn("runuser -u voidcell", setup)
        reqs = [ln for ln in (ENV / "requirements.txt").read_text(encoding="utf-8").splitlines()
                if ln.strip() and not ln.startswith("#")]
        self.assertTrue(reqs and all(re.match(r"^[A-Za-z0-9_.\-]+==[0-9][0-9A-Za-z.\-+]*$", ln) for ln in reqs), reqs)
        self.assertIn("mujoco==3.14.0", reqs)


if __name__ == "__main__":
    unittest.main()
