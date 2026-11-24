"""keyed_nut_peg_assembly, dual mode (experimental branch; never merged into main): the graded episode's service, the
freeze of the program, and how the verifier uses both.

tests/verify.py runs main's grading of the program (tests/offline_verify.py, main's tests/verify.py byte for byte; its
own tests are test_keyed_nut_peg_assembly_*.py and test_verify_survivor_keyed_nut_peg_assembly.py) and scores the
graded episode's record. What is tested here, host-side and stdlib only (the service's freeze and the verifier's record
and freeze functions import nothing else):
  * the pins: the service runs byte copies of the grader's cell and credit rule; its episode is a member of the hidden
    set; its windows, image charges and freeze caps are main's numbers; the anchors; the agent's client;
  * the freeze: environment/setup/online/graded_service.py's snapshot_submission on temporary folders (regular files
    and folders only, no link followed, main's caps, an empty program for a missing folder; its digest is the one the
    verifier recomputes);
  * the verifier: the record (missing, foreign, malformed, never started), the program it grades (the copy frozen at the
    start or at the window's close, the session-end folder only when nothing was frozen, nothing for a copy over the
    caps; a copy that does not match its record is a task error) and the installed copy.
"""

import hashlib
import importlib.util
import json
import os
import re
import stat
import tempfile
import unittest
from pathlib import Path

TASK = Path(__file__).resolve().parents[1] / "tasks/arm/keyed_nut_peg_assembly"
ONLINE_DIR = TASK / "environment/setup/online"
SERVICE = json.loads((ONLINE_DIR / "service.json").read_text())
EPISODE = json.loads((ONLINE_DIR / "episode.json").read_text())
ONLINE = json.loads((TASK / "tests/online.json").read_text())
GRADER = json.loads((TASK / "tests/grader_config.json").read_text())
EPISODES = json.loads((TASK / "tests/ref/episodes.json").read_text())["episodes"]


def _module(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


GS = _module("keyed_nut_graded_service", ONLINE_DIR / "graded_service.py")
V = _module("keyed_nut_dual_verify", TASK / "tests/verify.py")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Pins(unittest.TestCase):
    def test_the_service_runs_the_graders_cell(self):
        names = sorted(p.name for p in (ONLINE_DIR / "cell").iterdir() if p.name != "__pycache__")
        self.assertEqual(names, ["keyed_credit.py", "keyed_env.py", "keyed_look.json", "keyed_render.py"])
        for name in names:
            self.assertEqual(sha256(ONLINE_DIR / "cell" / name), sha256(TASK / "tests" / name), name)
            self.assertEqual(SERVICE["cell_sha256"][name], sha256(TASK / "tests" / name), name)
        self.assertEqual(sorted(SERVICE["cell_sha256"]), names)

    def test_the_graded_episode_is_a_member_of_the_hidden_set(self):
        entry = next(e for e in EPISODES if e["id"] == EPISODE["episode"])
        self.assertEqual(EPISODES.index(entry), EPISODE["index"])
        self.assertEqual((entry["kit"], entry["seed"]), (EPISODE["kit"], EPISODE["seed"]))
        digest = hashlib.sha256(str(int(EPISODE["seed"])).encode()).hexdigest()
        self.assertEqual(EPISODE["seed_sha256"], digest)
        self.assertEqual((ONLINE["episode"], ONLINE["seed_sha256"]), (EPISODE["episode"], digest))
        self.assertEqual(ONLINE["format"], GS.RECORD_FORMAT)
        self.assertEqual(ONLINE["task"], SERVICE["task"])
        self.assertEqual(EPISODE["oracle_token_sha256"], (TASK / "tests/oracle_token.sha256").read_text().strip())
        self.assertEqual(sha256(TASK / "tests/ref/episodes.json"), ONLINE["draw"]["from"].split("sha256 ")[1][:64])

    def test_the_windows_charges_and_caps_are_mains(self):
        self.assertEqual(SERVICE["image_charge_s"], GRADER["image_charge_s"])
        self.assertEqual(SERVICE["image_budget_s"], GRADER["controller_budget_s"])
        self.assertEqual((SERVICE["prep_s"], SERVICE["graded_s"]), (28800, 10800))   # main's agent budget, then 3 h
        self.assertEqual((ONLINE["prep_s"], ONLINE["graded_s"]), (SERVICE["prep_s"], SERVICE["graded_s"]))
        toml = (TASK / "task.toml").read_text()
        agent = toml[toml.index("[agent]"):]
        self.assertEqual(int(re.search(r"^timeout_sec\s*=\s*(\d+)", agent, re.M).group(1)),
                         SERVICE["prep_s"] + SERVICE["graded_s"])
        verifier = toml[toml.index("[verifier]"):]
        self.assertEqual(int(re.search(r"^timeout_sec\s*=\s*(\d+)", verifier, re.M).group(1)), 12600)   # main's
        self.assertIn('artifacts = ["/home/user/submission", "/var/lib/arb-online"]', toml)
        offline = (TASK / "tests/offline_verify.py").read_text()
        self.assertIn("MAX_SUBMISSION_BYTES = 200 * 1024 * 1024", offline)
        self.assertIn("MAX_SUBMISSION_FILES = 5000", offline)
        self.assertEqual((SERVICE["snapshot_max_bytes"], SERVICE["snapshot_max_files"]), (200 * 1024 * 1024, 5000))
        self.assertEqual(SERVICE["record_dir"], V.ONLINE_DIR)
        self.assertEqual(os.path.join(SERVICE["record_dir"], SERVICE["snapshot_name"]), V.SNAPSHOT)

    def test_the_anchors(self):
        dual = json.loads((TASK / "tests/anchor.json").read_text())
        self.assertEqual((dual["metric"], dual["direction"], dual["value"], dual["full_at"]),
                         ("dual_mode_score", "higher", 1.0, 1.0))
        offline = json.loads((TASK / "tests/anchor_offline.json").read_text())
        self.assertEqual((offline["metric"], offline["value"], offline["full_at"]), ("kit_success_rate", 1.0, 0.98))
        online = json.loads((TASK / "tests/anchor_online.json").read_text())
        self.assertEqual((online["metric"], online["direction"], online["value"], online["full_at"]),
                         ("kit_success_rate", "higher", 1.0, 0.98))

    def test_practice_closes_at_the_start(self):
        commission = (TASK / "environment/commission_service/commissiond.py").read_text()
        self.assertIn('"KEYEDCELL_GRADED_FLAG", "%s"' % SERVICE["practice_flag"], commission)
        self.assertIn('"error": "practice_closed"', commission)
        setup = (TASK / "environment/setup/setup.sh").read_text()
        self.assertLess(setup.index("graded_service.py"), setup.index("commissiond.py"))
        self.assertIn("useradd --system --user-group --no-create-home --home-dir /nonexistent --shell "
                      "/usr/sbin/nologin " + SERVICE["user"], (TASK / "environment/Dockerfile").read_text())

    def test_the_agents_client(self):
        client = (TASK / "environment/setup/payload/commission/client.py").read_text()
        self.assertIn('GRADED_SOCKET = "%s"' % SERVICE["socket"], client)
        self.assertIn("    def start(self):\n", client)            # no token option (the oracle speaks the wire)
        self.assertNotIn("token", client)

    def test_the_instruction_names_both_items_and_the_freeze(self):
        text = " ".join((TASK / "instruction.md").read_text().split())
        for words in ("two items, equally weighted", "**The freeze.**", "prep_window_closed", "practice_closed",
                      "**8 hours** to prepare", "up to **3 hours**", "11 hours", "connect_graded",
                      "run_kit.py --graded --start", "a connection that sends nothing for 900 s is closed"):
            self.assertIn(words, text)
        for false in ("not graded", "scores nothing", "late finish", "Only the final controller is scored"):
            self.assertNotIn(false, text)


def _cfg(root, **caps):
    cfg = {"record_dir": str(root / "record"), "snapshot_name": "submission_snapshot",
           "submission_dir": str(root / "submission"), "snapshot_max_files": 5000,
           "snapshot_max_bytes": 200 << 20, "snapshot_max_entries": 20000, "snapshot_max_depth": 32}
    cfg.update(caps)
    os.makedirs(cfg["record_dir"], exist_ok=True)
    return cfg


def _program(root):
    sub = root / "submission"
    (sub / "lib").mkdir(parents=True)
    (sub / "run.sh").write_text("#!/usr/bin/env bash\nexec python3 main.py\n")
    os.chmod(sub / "run.sh", 0o775)
    (sub / "main.py").write_text("print('READY')\n")
    (sub / "lib" / "data.bin").write_bytes(os.urandom(4096))
    os.symlink("/etc/passwd", str(sub / "passwd"))
    os.symlink(str(sub / "lib"), str(sub / "lib_link"))
    os.mkfifo(str(sub / "fifo"))
    return sub


class Freeze(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_the_copy_holds_the_files_and_folders_and_follows_no_link(self):
        _program(self.root)
        cfg = _cfg(self.root)
        res = GS.snapshot_submission(cfg, os.getuid())
        snap = Path(cfg["record_dir"]) / "submission_snapshot"
        self.assertEqual(res["status"], "ok")
        self.assertEqual(sorted(p.relative_to(snap).as_posix() for p in snap.rglob("*")),
                         ["lib", "lib/data.bin", "main.py", "run.sh"])
        self.assertEqual((res["files"], res["skipped"]), (3, 3))
        self.assertEqual(stat.S_IMODE(os.stat(snap / "run.sh").st_mode), 0o755)    # group and other write dropped
        entries, nfiles, nbytes, problems = V.snapshot_listing(str(snap))
        self.assertEqual(problems, [])
        self.assertEqual((V.snapshot_digest(entries), nfiles, nbytes), (res["sha256"], res["files"], res["bytes"]))
        self.assertEqual(V.snapshot_digest(entries), GS.snapshot_digest(entries))

    def test_over_a_cap_nothing_is_copied(self):
        _program(self.root)
        for caps in ({"snapshot_max_files": 2}, {"snapshot_max_bytes": 1000}, {"snapshot_max_entries": 3},
                     {"snapshot_max_depth": 0}):
            cfg = _cfg(self.root, **caps)
            res = GS.snapshot_submission(cfg, os.getuid())
            self.assertEqual(res["status"], "too_large", caps)
            self.assertFalse((Path(cfg["record_dir"]) / "submission_snapshot").exists(), caps)
            self.assertIsNone(res["sha256"])

    def test_a_missing_folder_freezes_an_empty_program(self):
        cfg = _cfg(self.root)
        res = GS.snapshot_submission(cfg, os.getuid())
        self.assertEqual((res["status"], res["files"], res["bytes"]), ("ok", 0, 0))
        self.assertEqual(res["sha256"], V.snapshot_digest([]))

    def test_files_of_another_account_are_skipped(self):
        _program(self.root)
        cfg = _cfg(self.root)
        res = GS.snapshot_submission(cfg, os.getuid() + 1)     # as if the folder belonged to another account
        self.assertEqual((res["status"], res["files"]), ("ok", 0))


def _record(**kw):
    rec = {"format": ONLINE["format"], "task": ONLINE["task"], "episode": ONLINE["episode"],
           "seed_sha256": ONLINE["seed_sha256"], "started": True, "ended": True, "end_reason": "done",
           "episode_score": 1.0, "freeze": "graded_start"}
    rec.update(kw)
    return rec


class Verifier(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def _frozen(self):
        _program(self.root)
        cfg = _cfg(self.root)
        res = GS.snapshot_submission(cfg, os.getuid())
        fields = {"snapshot_status": res["status"], "snapshot_files": res["files"], "snapshot_bytes": res["bytes"],
                  "snapshot_sha256": res["sha256"]}
        return Path(cfg["record_dir"]) / "submission_snapshot", fields

    def test_the_record(self):
        p = self.root / "record.json"
        self.assertEqual(V.load_record(str(p)), (None, "missing"))
        p.write_text(json.dumps(_record()))
        rec, why = V.load_record(str(p))
        self.assertIsNone(why)
        self.assertIsNone(V.check_record(rec, ONLINE))
        self.assertEqual(V.record_state(rec, why, ONLINE), "started")
        for bad in ({"task": "other"}, {"episode": "F00"}, {"seed_sha256": "0" * 64}, {"format": "x"}):
            self.assertIsNotNone(V.check_record(_record(**bad), ONLINE), bad)
            self.assertEqual(V.record_state(_record(**bad), None, ONLINE), "invalid", bad)
        self.assertIn("never started", V.check_record(_record(started=False), ONLINE))
        for score in (-1.0, "1", True, None):
            self.assertIsNotNone(V.check_record(_record(episode_score=score), ONLINE), score)
        p.write_text('{"episode_score": NaN}')
        self.assertEqual(V.load_record(str(p)), (None, "not valid JSON"))
        p.write_text("[1]")
        self.assertEqual(V.load_record(str(p)), (None, "not a JSON object"))
        p.write_bytes(b" " * ((1 << 20) + 1))
        self.assertEqual(V.load_record(str(p)), (None, "larger than 1 MB"))
        q = self.root / "link.json"
        os.symlink(str(p), str(q))
        self.assertEqual(V.load_record(str(q)), (None, "not a regular file"))

    def test_the_program_graded(self):
        snap, fields = self._frozen()
        # started, frozen ok and matching: the frozen copy
        rec = _record(**fields)
        self.assertEqual(V.freeze_decision(rec, "started", None, root=str(snap)), ("graded_start", fields, None))
        # the start window closed without a start: the copy frozen then
        rec = _record(started=False, freeze="prep_window_closed", **fields)
        self.assertEqual(V.freeze_decision(rec, "not_started", None, root=str(snap))[0::2],
                         ("prep_window_closed", None))
        # no record of a freeze: the session-end submission
        self.assertEqual(V.freeze_decision(None, "missing", None, root=str(snap)), (None, {}, None))
        self.assertEqual(V.freeze_decision(_record(started=False, freeze=None), "not_started",
                                           {"started": False}, root=str(snap)), (None, {}, None))
        # no record, but the service's state.json says the episode started: the copy it froze
        state = {"started": True, "snapshot": {"status": "ok", "files": fields["snapshot_files"],
                                               "bytes": fields["snapshot_bytes"], "sha256": fields["snapshot_sha256"]}}
        self.assertEqual(V.freeze_decision(None, "missing", state, root=str(snap))[0::2], ("graded_start", None))
        # over a cap, failed or never finished: the program item scores 0, never the session-end folder
        for status in ("too_large", "failed", "pending"):
            frozen_by, _f, why = V.freeze_decision(_record(snapshot_status=status), "started", None, root=str(snap))
            self.assertEqual(frozen_by, "graded_start")
            self.assertTrue(why, status)
        # a copy that differs from its record, or is missing: a task error
        (snap / "main.py").write_text("changed after the freeze\n")
        with self.assertRaises(SystemExit):
            V.freeze_decision(_record(**fields), "started", None, root=str(snap))
        with self.assertRaises(SystemExit):
            V.freeze_decision(_record(**fields), "started", None, root=str(self.root / "nowhere"))

    def test_the_frozen_copy_replaces_the_restored_submission(self):
        snap, _fields = self._frozen()
        restored = self.root / "restored"
        restored.mkdir()
        (restored / "run.sh").write_text("#!/usr/bin/env bash\nexit 3\n")       # edited after the freeze
        (restored / "after.txt").write_text("B\n")
        V.install_snapshot(str(snap), str(restored))
        self.assertEqual(sorted(p.relative_to(restored).as_posix() for p in restored.rglob("*")),
                         ["lib", "lib/data.bin", "main.py", "run.sh"])
        self.assertEqual((restored / "run.sh").read_text(), (snap / "run.sh").read_text())
        self.assertEqual(stat.S_IMODE(os.stat(restored / "run.sh").st_mode), 0o755)



class _ActionError(ValueError):
    pass


class _E:
    """keyed_env's parse_action, to_wire and colour_name, as far as the cell uses them."""
    ActionError = _ActionError
    CONTROL_DT = 0.1

    @staticmethod
    def parse_action(a):
        if not isinstance(a, (list, tuple)) or len(a) != 8 or a[7] != int(a[7]) or not 0 <= a[7] <= 7:
            raise _ActionError("action must be a list of 8 numbers")
        return None, a[6], tuple(n for i, n in enumerate(("left", "right", "wrist")) if int(a[7]) & (1 << i))

    @staticmethod
    def to_wire(obs):
        return dict(obs)

    @staticmethod
    def colour_name(rgba):
        return "teal"


class _KC:
    @staticmethod
    def ground_truth(env, wire):
        return {"t": env.t}


class _Env:
    max_steps, n = 850, 3

    def __init__(self, render_fails=False):
        self.t, self.done, self.fault, self.render_fails = 0, False, None, render_fails
        self.broken, self.credit, self.order = [False] * 3, [0.0] * 3, [1, 2, 3]
        self.specs = [{"rgba": [0, 0, 0, 1]}] * 3

    def step(self, action):
        if self.render_fails:
            raise RuntimeError("renderer: device lost")
        self.t += 1
        return {"t": self.t}, False, {}

    def episode_score(self):
        return 0.0

    def nuts_assembled(self):
        return 0

    def progress(self):
        return 0.25

    def assembled(self, i):
        return False


class CellEnds(unittest.TestCase):
    """The service's cell on stand-ins: the image charges against the kit's budget, a malformed action, a renderer
    failure."""

    def _cell(self, **kw):
        return GS.Cell(_Env(**kw), _E, _KC, {"t": 0}, SERVICE["image_charge_s"], SERVICE["image_budget_s"])

    def test_images_are_charged_and_the_budget_ends_the_episode_as_in_grading(self):
        c = self._cell()
        both = [0, 0, 0, 0, 0, 0, -1, 3]                        # left + right: 0.9 s a step
        for _ in range(666):                                    # 666 x 0.9 = 599.4 s
            self.assertEqual(c.step(both)["done"], False)
        self.assertAlmostEqual(c.charged, 599.4)
        res = c.step(both)                                      # 600.3 s: past the budget, not applied
        self.assertEqual(res, {"obs": None, "done": True, "ended": "image_budget"})
        self.assertEqual((c.env.t, c.images, c.over, c.cut), (666, 1332, True, True))
        self.assertIn("image budget", c.end_text())
        self.assertEqual(c.metrics()["image_budget_cut"], 1.0)

    def test_steps_without_images_cost_nothing(self):
        c = self._cell()
        for _ in range(850):
            c.step([0, 0, 0, 0, 0, 0, -1, 0])
        self.assertEqual((c.charged, c.images, c.cut), (0.0, 0, False))

    def test_a_malformed_action_is_refused_and_simulates_nothing(self):
        c = self._cell()
        for bad in ([0] * 7, [0] * 7 + [8], [0] * 7 + [1.5], "x"):
            self.assertEqual(c.step(bad)["error"], "bad_action", bad)
        self.assertEqual((c.env.t, c.refused, c.over), (0, 4, False))

    def test_the_oracles_observations_carry_the_truth(self):
        c = self._cell()
        c.privileged = True
        self.assertEqual(c.first_observation()["_gt"], {"t": 0})
        self.assertEqual(c.step([0] * 7 + [0])["obs"]["_gt"], {"t": 1})

    def test_a_renderer_failure_is_the_cells_error(self):
        c = self._cell(render_fails=True)
        self.assertEqual(c.step([0, 0, 0, 0, 0, 0, -1, 4])["error"], "internal")
        self.assertTrue(c.over and c.error)
        self.assertEqual(c.metrics()["cell_error"], 1.0)


if __name__ == "__main__":
    unittest.main()
