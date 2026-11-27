"""dexterous_hand_environment_twin_from_real_photo: the Harbor layout, what may reach the agent's image, the photo's
route from the asset dataset, and the truth the grader checks itself against.

The grader's files (tests/) and the reference twin (solution/) must not reach the image or the agent home: no file
of tests/ or solution/ is byte-identical to a file of the image's build context (environment/ less setup/), of the
givens (environment/setup/payload/) or of the payload the image fetches (environment/assets.sha256), with one
exception by design, tests/setup_payload/ (the grader's byte copy of the givens: the public gate, the standard
simulation and the pick instrument the grader runs are the agent's own). The image installs only requirements.txt
and, in its last step, the asset list with the vendored fetcher (byte-identical to shared/task_assets/fetch_assets.py),
and derives from the pinned base image.

The real photo is not in git: the image fetches it from the benchmark's asset dataset at the dataset commit that
environment/assets.sha256 pins (the placeholder revision of forty zeros is refused), and tests/given_hashes.json pins
every given, the photo included. Every other given is a small text file in git.

The truth: every file of tests/gt/ is listed in tests/gt/truth_manifest.json with its sha256 and size, nothing else is
there, and the manifest itself is pinned by its sha256, so that a change of the truth fails here before it changes a
score. The repository's .gitignore drops `*.npz`, and three of the truth's files are .npz: a commit that leaves them
out makes the grader's own self-test fail on every run (a task error), so this test fails first. The reference twin's
scene names only files that are in solution/answer/.

Stdlib only; nothing is downloaded (the photo is not in git).
"""
import hashlib
import json
import os
import re
import tomllib
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SLUG = "dexterous_hand_environment_twin_from_real_photo"
TASK = REPO / "tasks" / "hand" / SLUG
ENV = TASK / "environment"
PAYLOAD = ENV / "setup" / "payload"
TESTS = TASK / "tests"
SOLUTION = TASK / "solution"
EMPTY = hashlib.sha256(b"").hexdigest()
PLACEHOLDER_REVISION = "0" * 40
FETCH_COPY = "COPY fetch_assets.py assets.sha256 /opt/task-assets/"
FETCH_RUN = "RUN python3 /opt/task-assets/fetch_assets.py /opt/task-assets/assets.sha256 /opt/task-assets/payload"

# The values this version was validated and measured with.
ENVIRONMENT = (8, 32768)
AGENT = (21600, "no-network")
VERIFIER = ("separate", 1800)
VERIFIER_ENVIRONMENT = (2, 8192, 0)
CAPS = (5000, 314572800)
ANCHOR = ("twin_score", "higher", 1.0, 1.0)
ASSET_ROWS = ["photo.png"]
NPZ_FILES = ["tests/gt/reference_pieces.npz", "tests/gt/scans_occupancy.npz", "tests/gt/scans_surface.npz"]
TRUTH_MANIFEST_SHA256 = "00005169437bead61de1c03e430df433df17ccd4c320dedb745e698268213a15"
DOCKER_FROM = ["FROM ghcr.io/agentslastexam/container-ubuntu22-base:0.1.1"]
DOCKER_COPIES = [
    ["--from=ghcr.io/astral-sh/uv:0.11", "/uv", "/usr/local/bin/uv"],
    ["requirements.txt", "/tmp/"],
    ["fetch_assets.py", "assets.sha256", "/opt/task-assets/"],
]


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
            if f == ".gitignore":
                continue
            p = os.path.join(d, f)
            out[os.path.relpath(p, root).replace(os.sep, "/")] = _sha(p)
    return out


def _asset_list():
    path = ENV / "assets.sha256"
    head, entries = {}, {}
    if not path.is_file():
        return head, entries
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


def _instructions():
    """The Dockerfile's instructions, continuation lines joined, comments and blank lines dropped."""
    out, cur = [], ""
    for line in (ENV / "Dockerfile").read_text(encoding="utf-8").splitlines():
        if not cur and (not line.strip() or line.lstrip().startswith("#")):
            continue
        cur = (cur + " " + line.strip()) if cur else line.strip()
        if cur.endswith("\\"):
            cur = cur[:-1].rstrip()
            continue
        out.append(re.sub(r"\s+", " ", cur))
        cur = ""
    if cur:
        out.append(re.sub(r"\s+", " ", cur))
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
        self.assertEqual((cfg["environment"]["cpus"], cfg["environment"]["memory_mb"]), ENVIRONMENT)
        self.assertNotIn("gpus", cfg["environment"])
        self.assertEqual((cfg["agent"]["timeout_sec"], cfg["agent"]["network_mode"]), AGENT)
        self.assertEqual((cfg["verifier"]["environment_mode"], cfg["verifier"]["timeout_sec"]), VERIFIER)
        ve = cfg["verifier"]["environment"]
        self.assertEqual((ve["cpus"], ve["memory_mb"], ve["gpus"]), VERIFIER_ENVIRONMENT)

    def test_no_bytecode_and_no_link_in_the_task(self):
        found = [str(p.relative_to(TASK)) for p in TASK.rglob("*")
                 if p.name in ("__pycache__", ".pytest_cache") or p.suffix == ".pyc" or p.is_symlink()]
        self.assertEqual(found, [])

    def test_setup_payload_is_the_givens_byte_for_byte(self):
        self.assertEqual(_tree(TESTS / "setup_payload"), _tree(PAYLOAD))

    def test_given_hashes_pin_the_givens(self):
        pin = json.loads((TESTS / "given_hashes.json").read_text(encoding="utf-8"))
        self.assertEqual(pin["format"], "given-hashes/2")
        expect = dict(_tree(PAYLOAD))
        assets = _asset_list()[1]
        self.assertFalse(set(expect) & set(assets))
        expect.update(assets)
        self.assertEqual(pin["files"], expect)
        self.assertEqual(set(pin["sizes"]), set(expect))
        self.assertIn("photo.png", pin["files"])

    def test_the_photo_is_fetched_not_in_git(self):
        """The photo comes from the asset dataset at image build, pinned by revision and sha256; neither the givens
        nor the grader's copy of them carry it, and the task's own ignore files keep both copies out of git."""
        self.assertFalse((PAYLOAD / "photo.png").exists(), "the photo must come from the asset dataset")
        self.assertFalse((TESTS / "setup_payload" / "photo.png").exists())
        for gi, line in (("environment/setup/.gitignore", "payload/photo.png"),
                         ("tests/.gitignore", "setup_payload/photo.png")):
            self.assertIn(line, (TASK / gi).read_text(encoding="utf-8").splitlines(), gi)
        head, entries = _asset_list()
        conf = tomllib.loads((REPO / "task-assets.toml").read_text(encoding="utf-8"))["huggingface"]
        self.assertEqual(head.get("prefix"), "tasks/hand/%s/payload" % SLUG)
        self.assertEqual(head.get("repo"), conf["dataset"])
        self.assertEqual(head.get("endpoint"), conf["endpoint"].rstrip("/"))
        self.assertRegex(head.get("revision", ""), r"^[0-9a-f]{40}$")
        self.assertNotEqual(head.get("revision"), PLACEHOLDER_REVISION,
                            "the asset list still names the placeholder revision")
        self.assertEqual(sorted(entries), ASSET_ROWS)
        pin = json.loads((TESTS / "given_hashes.json").read_text(encoding="utf-8"))
        for rel, digest in entries.items():
            self.assertRegex(digest, r"^[0-9a-f]{64}$")
            self.assertEqual(digest, pin["files"][rel], rel)
        self.assertEqual(_sha(ENV / "fetch_assets.py"), _sha(REPO / "shared" / "task_assets" / "fetch_assets.py"))
        setup = (ENV / "setup" / "setup.sh").read_text(encoding="utf-8")
        self.assertIn('cp -r /opt/task-assets/payload/. "$HOME_DIR/"', setup)
        self.assertIn('if [ ! -f "$HOME_DIR/photo.png" ]; then', setup)

    def test_the_dockerfile_installs_only_the_environment(self):
        steps = _instructions()
        self.assertEqual([s for s in steps if s.startswith("FROM ")], DOCKER_FROM)
        self.assertEqual([s.split()[1:] for s in steps if s.startswith(("COPY ", "ADD "))], DOCKER_COPIES)
        self.assertFalse([s for s in steps if s.startswith(("ENTRYPOINT ", "CMD ", "ADD "))])
        # the fetch block is the last step, so every toolchain layer before it is kept when only the asset list moves
        self.assertEqual(steps[-2], FETCH_COPY)
        self.assertTrue(steps[-1].startswith(FETCH_RUN), steps[-1])
        self.assertEqual(sum(FETCH_RUN in s for s in steps), 1)
        self.assertFalse((ENV / "assets").exists())

    def test_givens_in_git_are_small_text_files(self):
        tree = _tree(PAYLOAD)
        self.assertTrue(tree)
        for rel in tree:
            data = (PAYLOAD / rel).read_bytes()
            self.assertLess(len(data), 256 * 1024, rel)
            self.assertNotIn(b"\0", data, rel)

    def test_grader_and_reference_never_reach_the_image(self):
        hidden = {"tests/" + r: h for r, h in _tree(TESTS).items()}
        hidden.update({"solution/" + r: h for r, h in _tree(SOLUTION).items()})
        visible = {}
        for rel, digest in _tree(ENV).items():
            if not rel.startswith("setup/"):
                visible.setdefault(digest, []).append("environment/" + rel)
        for rel, digest in _tree(PAYLOAD).items():
            visible.setdefault(digest, []).append("payload/" + rel)
        for rel, digest in _asset_list()[1].items():
            visible.setdefault(digest, []).append("assets/" + rel)
        bad = [(rel, visible[d]) for rel, d in sorted(hidden.items())
               if d in visible and not rel.startswith("tests/setup_payload/") and d != EMPTY]
        self.assertEqual(bad, [])

    def test_the_grader_reads_the_public_modules_from_its_copy_of_the_givens(self):
        """The gate, the standard simulation and the instrument the worker runs are tests/setup_payload/'s, which the
        mirror test above holds byte-identical to the agent's."""
        src = (TESTS / "verify.py").read_text(encoding="utf-8")
        self.assertIn('runner.copy_regular_files(os.path.join(STAGE_DIR, "setup_payload"), '
                      'os.path.join(code, "setup_payload"))', src)
        for name in ("twin_gate.py", "twin_sim.py", "twin_sim_config.json", "hand_instrument.py",
                     "instrument_config.json", "pick_sheet.json"):
            self.assertTrue((TESTS / "setup_payload" / name).is_file(), name)


class Truth(unittest.TestCase):
    def test_every_truth_file_is_in_the_manifest_with_its_bytes(self):
        gt = TESTS / "gt"
        manifest = json.loads((gt / "truth_manifest.json").read_text(encoding="utf-8"))
        rows = {r["name"]: r for r in manifest["files"]}
        on_disk = {k for k in _tree(gt) if k != "truth_manifest.json"}
        self.assertEqual(set(rows) - on_disk, set(), "listed in the manifest, missing from the tree")
        self.assertEqual(on_disk - set(rows), set(), "in the tree, not in the manifest")
        bad = [n for n, r in rows.items() if _sha(gt / n) != r["sha256"] or (gt / n).stat().st_size != r["size"]]
        self.assertEqual(bad, [])

    def test_the_truth_is_pinned(self):
        self.assertEqual(_sha(TESTS / "gt" / "truth_manifest.json"), TRUTH_MANIFEST_SHA256)

    def test_the_npz_files_the_gitignore_drops_are_present(self):
        """`*.npz` is in the repository's .gitignore: these must be added with `git add -f`."""
        found = sorted(str(p.relative_to(TASK)) for p in TASK.rglob("*.npz"))
        self.assertEqual(found, NPZ_FILES)

    def test_the_grader_config_points_at_the_truth(self):
        cfg = json.loads((TESTS / "grader_config.json").read_text(encoding="utf-8"))
        self.assertEqual(cfg["gt"]["dir"], "gt")
        self.assertEqual((cfg["caps"]["max_entries"], cfg["caps"]["max_bytes"]), CAPS)

    def test_the_reference_scene_names_only_its_own_files(self):
        answer = SOLUTION / "answer"
        scene = (answer / "scene.xml").read_text(encoding="utf-8")
        files = re.findall(r'\bfile="([^"]+)"', scene)
        meshdir = re.findall(r'\bmeshdir="([^"]+)"', scene)
        base = answer / (meshdir[0] if meshdir else "")
        self.assertTrue(files)
        missing = [f for f in files if not ((base / f).is_file() or (answer / f).is_file())]
        self.assertEqual(missing, [])
        for f in files:
            self.assertFalse(f.startswith("/") or ".." in Path(f).parts, f)
        self.assertTrue((answer / "objects.json").is_file())

    def test_anchor_is_the_reference_at_full(self):
        anchor = json.loads((TESTS / "anchor.json").read_text(encoding="utf-8"))
        self.assertEqual((anchor["metric"], anchor["direction"], anchor["value"], anchor["full_at"]), ANCHOR)


if __name__ == "__main__":
    unittest.main()
