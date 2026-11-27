"""pantry_jar_refilling, dual mode (experimental branch; never merged into main): the graded episode's service, the
freeze of the program, and how the verifier uses both.

tests/verify.py runs main's grading of the program (tests/offline_verify.py, main's tests/verify.py byte for byte; its
own checks are in test_pantry_jar_refilling_copies.py and test_numerics_pin_names.py) and scores the graded episode's
record. What is tested here, host-side and stdlib only (the service's freeze and the verifier's record and freeze
functions import nothing else):
  * the pins: the service runs the grader's simulator (the files setup.sh copies from the staged payload, byte copies of
    tests/'s), its episode is a member of the hidden set on its hidden unit, its windows and freeze caps are main's
    numbers, the anchors, the session clock, the agent's client;
  * the freeze: environment/setup/online/graded_service.py's snapshot_submission on temporary folders (regular files
    and folders only, no link followed, main's caps; its digest is the one the verifier recomputes);
  * the verifier: the record and the program it grades (the frozen copy, the session-end folder only when nothing was
    frozen, nothing for a copy over the caps; a copy that does not match its record is a task error).
"""

import hashlib
import importlib.util
import json
import os
import re
import tempfile
import unittest
from pathlib import Path

TASK = Path(__file__).resolve().parents[1] / "tasks/arm/pantry_jar_refilling"
ONLINE_DIR = TASK / "environment/setup/online"
SERVICE = json.loads((ONLINE_DIR / "service.json").read_text())
EPISODE = json.loads((ONLINE_DIR / "episode.json").read_text())
ONLINE = json.loads((TASK / "tests/online.json").read_text())
EPISODES = json.loads((TASK / "tests/hidden/episodes.json").read_text())["episodes"]
UNITS = json.loads((TASK / "tests/units.json").read_text())["units"]
CELL = ("episode.py", "jar_env.py", "jar_look.json", "jar_render.py", "jar_sim.py")


def _module(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


GS = _module("pantry_graded_service", ONLINE_DIR / "graded_service.py")
V = _module("pantry_dual_verify", TASK / "tests/verify.py")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Pins(unittest.TestCase):
    def test_the_service_runs_the_graders_simulator(self):
        self.assertEqual(sorted(SERVICE["cell_sha256"]), sorted(CELL))
        for name in CELL:
            self.assertEqual(SERVICE["cell_sha256"][name], sha256(TASK / "tests" / name), name)
            self.assertEqual(sha256(TASK / "environment/setup/payload" / name), sha256(TASK / "tests" / name), name)
        setup = (TASK / "environment/setup/setup.sh").read_text()
        self.assertIn('cp payload/jar_sim.py payload/jar_render.py payload/jar_look.json payload/jar_env.py '
                      'payload/episode.py "$GRADED/cell/"', setup)
        self.assertLess(setup.index("date -u +%s > /run/session/start"), setup.index("graded_service.py"))
        self.assertIn("useradd --system --user-group --no-create-home --home-dir /nonexistent", setup)
        self.assertIn(SERVICE["user"], setup)

    def test_the_graded_episode_is_a_member_of_the_hidden_set(self):
        entry = EPISODES[EPISODE["index"]]
        self.assertEqual((entry["unit"], entry["seed"]), (EPISODE["unit_id"], EPISODE["seed"]))
        self.assertEqual(EPISODE["unit"], UNITS[EPISODE["unit_id"]])
        digest = hashlib.sha256(str(int(EPISODE["seed"])).encode()).hexdigest()
        self.assertEqual(EPISODE["seed_sha256"], digest)
        self.assertEqual((ONLINE["episode"], ONLINE["seed_sha256"]), (EPISODE["episode"], digest))
        self.assertEqual(EPISODE["episode"], "ep%02d" % EPISODE["index"])
        self.assertEqual((ONLINE["format"], ONLINE["task"]), (GS.RECORD_FORMAT, SERVICE["task"]))
        self.assertEqual(EPISODE["oracle_token_sha256"], (TASK / "tests/hidden/oracle_token.sha256").read_text().strip())
        self.assertEqual(sha256(TASK / "tests/hidden/episodes.json"), ONLINE["draw"]["from"].split("sha256 ")[1][:64])

    def test_the_windows_and_caps_are_mains(self):
        self.assertEqual((SERVICE["prep_s"], SERVICE["graded_s"]), (7200, 10800))     # main's agent budget, then 3 h
        self.assertEqual((ONLINE["prep_s"], ONLINE["graded_s"]), (SERVICE["prep_s"], SERVICE["graded_s"]))
        toml = (TASK / "task.toml").read_text()
        agent = toml[toml.index("[agent]"):]
        self.assertEqual(int(re.search(r"^timeout_sec\s*=\s*(\d+)", agent, re.M).group(1)),
                         SERVICE["prep_s"] + SERVICE["graded_s"])
        verifier = toml[toml.index("[verifier]"):]
        self.assertEqual(int(re.search(r"^timeout_sec\s*=\s*(\d+)", verifier, re.M).group(1)), 21600)   # main's
        self.assertIn('artifacts = ["/home/user/submission", "/var/lib/arb-online"]', toml)
        offline = (TASK / "tests/offline_verify.py").read_text()
        self.assertIn("MAX_SUBMISSION_BYTES = 200 * 1024 * 1024", offline)
        self.assertIn("MAX_SUBMISSION_FILES = 5000", offline)
        self.assertEqual((SERVICE["snapshot_max_bytes"], SERVICE["snapshot_max_files"]), (200 * 1024 * 1024, 5000))
        self.assertEqual(os.path.join(SERVICE["record_dir"], SERVICE["snapshot_name"]), V.SNAPSHOT)
        docker = (TASK / "environment/Dockerfile").read_text()
        for line in docker.splitlines():
            if line.startswith("ENV ") and "OPENBLAS_CORETYPE" in line:
                for kv in line[4:].split():
                    k, v = kv.split("=")
                    self.assertEqual(SERVICE["env"][k], v, k)         # the grader's numerics pins

    def test_the_session_clock(self):
        script = (TASK / "environment/setup/payload/session_time.sh").read_text()
        self.assertEqual(int(re.search(r"^PREP_MIN=(\d+)$", script, re.M).group(1)) * 60, SERVICE["prep_s"])
        self.assertEqual(int(re.search(r"^LIMIT_MIN=(\d+)$", script, re.M).group(1)) * 60,
                         SERVICE["prep_s"] + SERVICE["graded_s"])

    def test_the_anchors(self):
        dual = json.loads((TASK / "tests/anchor.json").read_text())
        self.assertEqual((dual["metric"], dual["direction"], dual["value"], dual["full_at"]),
                         ("dual_mode_score", "higher", 1.0, 1.0))
        offline = json.loads((TASK / "tests/anchor_offline.json").read_text())
        self.assertEqual((offline["metric"], offline["value"], offline["full_at"]), ("mean_episode_credit", 1.0, 0.98))
        online = json.loads((TASK / "tests/anchor_online.json").read_text())
        self.assertEqual((online["metric"], online["direction"], online["value"], online["full_at"]),
                         ("mean_episode_credit", "higher", 1.0, 0.98))

    def test_the_agents_client(self):
        client = (TASK / "environment/setup/payload/graded/client.py").read_text()
        self.assertIn('GRADED_SOCKET = "%s"' % SERVICE["socket"], client)
        self.assertIn("    def start(self):\n", client)            # no token option (the oracle speaks the wire)
        self.assertNotIn("token", client)
        bridge = (TASK / "environment/setup/payload/graded/run_graded.py").read_text()
        self.assertNotIn("token", bridge)

    def test_the_instruction_names_both_items_and_the_freeze(self):
        text = " ".join((TASK / "instruction.md").read_text().split())
        for words in ("two items, equally weighted", "**The freeze.**", "prep_window_closed", "**2 hours** to prepare",
                      "up to **3 hours**", "at most 5 hours", "connect_graded", "run_graded.py --start",
                      "a connection that sends nothing for 900 s is closed", "bash /home/user/session_time.sh"):
            self.assertIn(words, text)
        for false in ("not graded", "scores nothing", "late finish", "when the time ends is graded",
                      "Your session is stopped 2 hours after it started"):
            self.assertNotIn(false, text)


def _cfg(root, **caps):
    cfg = {"record_dir": str(root / "record"), "snapshot_name": "submission_snapshot",
           "submission_dir": str(root / "submission"), "snapshot_max_files": 5000,
           "snapshot_max_bytes": 200 << 20, "snapshot_max_entries": 20000, "snapshot_max_depth": 32}
    cfg.update(caps)
    os.makedirs(cfg["record_dir"], exist_ok=True)
    return cfg


class FreezeAndVerifier(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        sub = self.root / "submission"
        (sub / "pkg").mkdir(parents=True)
        (sub / "run.sh").write_text("#!/usr/bin/env bash\nexec /opt/venv/bin/python3 -I main.py\n")
        (sub / "main.py").write_text("print('READY')\n")
        (sub / "pkg" / "model.npz").write_bytes(os.urandom(8192))
        os.symlink("/etc/hostname", str(sub / "host"))

    def tearDown(self):
        self.tmp.cleanup()

    def test_the_freeze_and_the_program_graded(self):
        cfg = _cfg(self.root)
        res = GS.snapshot_submission(cfg, os.getuid())
        snap = Path(cfg["record_dir"]) / "submission_snapshot"
        self.assertEqual((res["status"], res["files"], res["skipped"]), ("ok", 3, 1))
        entries, nfiles, nbytes, problems = V.snapshot_listing(str(snap))
        self.assertEqual((V.snapshot_digest(entries), nfiles, nbytes, problems),
                         (res["sha256"], res["files"], res["bytes"], []))
        fields = {"snapshot_status": "ok", "snapshot_files": res["files"], "snapshot_bytes": res["bytes"],
                  "snapshot_sha256": res["sha256"]}
        rec = {"format": ONLINE["format"], "task": ONLINE["task"], "episode": ONLINE["episode"],
               "seed_sha256": ONLINE["seed_sha256"], "started": True, "episode_score": 0.5, "freeze": "graded_start"}
        rec.update(fields)
        self.assertIsNone(V.check_record(rec, ONLINE))
        self.assertEqual(V.freeze_decision(rec, "started", None, root=str(snap))[0::2], ("graded_start", None))
        self.assertEqual(V.freeze_decision(None, "missing", None, root=str(snap)), (None, {}, None))
        too_large = GS.snapshot_submission(_cfg(self.root, snapshot_max_bytes=100), os.getuid())
        self.assertEqual(too_large["status"], "too_large")
        self.assertTrue(V.freeze_decision(dict(rec, snapshot_status="too_large"), "started", None, root=str(snap))[2])
        res = GS.snapshot_submission(cfg, os.getuid())        # a fresh copy (the cap test removed the one above)
        (snap / "main.py").write_text("edited\n")
        with self.assertRaises(SystemExit):
            V.freeze_decision(dict(rec, snapshot_sha256=res["sha256"]), "started", None, root=str(snap))



class _Fatal(Exception):
    pass


class _Mujoco:
    FatalError = _Fatal


class _E:
    """jar_env's check_action and to_wire, as far as the cell uses them."""

    @staticmethod
    def check_action(action):
        if not isinstance(action, dict) or not {"l", "r"} <= set(action):
            raise ValueError("malformed action")
        return {"cams": int(action.get("cams", 0))}

    @staticmethod
    def to_wire(obs):
        return dict(obs)


class _EP:
    EPISODE_STEPS = 12000


class _Env:
    def __init__(self):
        self.t, self.bad = 0, False

    def unstable(self):
        return self.bad

    def truth(self):
        return {"t": self.t}


class _Episode:
    def __init__(self, env, outcome):
        self.env, self.outcome = env, outcome

    def step(self, action):
        if self.outcome == "fatal":
            raise _Fatal("mj_step: bad state")
        if self.outcome == "render":
            raise RuntimeError("renderer: device lost")
        self.env.t += 1
        self.env.bad = self.outcome == "unstable"
        return {"t": self.env.t}, False, {}

    def score(self):
        if self.outcome == "fatal":
            raise AssertionError("the state after a fatal error is never read")
        return {"credit": 0.0 if self.env.bad else 0.5, "penalized": False,
                "jars": {"mint": {"done": True, "filled": True, "sealed": True},
                         "sugar": {"done": False, "filled": True, "sealed": False}}}


class CellEnds(unittest.TestCase):
    """The service's cell on stand-ins: how a step ends the episode and what the record then says."""

    def _cell(self, outcome):
        env = _Env()
        return GS.Cell(env, _Episode(env, outcome), _E, _EP, _Mujoco, "u1", {"t": 0})

    def test_a_malformed_action_is_refused_and_simulates_nothing(self):
        c = self._cell("ok")
        self.assertEqual(c.step([0] * 8)["error"], "bad_action")
        self.assertEqual((c.steps, c.refused, c.over), (0, 1, False))

    def test_a_step_as_it_stands(self):
        c = self._cell("ok")
        c.privileged = True
        res = c.step({"l": [0] * 7, "r": [0] * 7, "cams": 9})
        self.assertEqual((res["done"], res["obs"]["unit"], res["obs"]["truth"]), (False, "u1", {"t": 1}))
        self.assertEqual((c.score(), c.images, c.metrics()["part_done"]), (0.5, 2, 0.5))

    def test_a_fatal_error_scores_0_without_reading_the_state(self):
        c = self._cell("fatal")
        self.assertEqual(c.step({"l": [0] * 7, "r": [0] * 7}), {"obs": None, "done": True})
        self.assertTrue(c.over)
        self.assertEqual((c.score(), c.metrics()["sim_fault"], c.detail()["score"]), (0.0, 1.0, None))
        self.assertIn("fatal", c.end_text())

    def test_an_unstable_simulation_ends_the_episode_at_0(self):
        c = self._cell("unstable")
        self.assertTrue(c.step({"l": [0] * 7, "r": [0] * 7})["done"])
        self.assertEqual((c.over, c.score(), c.metrics()["sim_fault"]), (True, 0.0, 1.0))

    def test_a_renderer_failure_is_the_cells_error(self):
        c = self._cell("render")
        self.assertEqual(c.step({"l": [0] * 7, "r": [0] * 7, "cams": 1})["error"], "internal")
        self.assertTrue(c.over and c.error)
        self.assertEqual(c.metrics()["cell_error"], 1.0)


if __name__ == "__main__":
    unittest.main()
