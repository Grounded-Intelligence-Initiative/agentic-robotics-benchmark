"""dexterous_hand_robot_description_from_photos: a process of the agent account that outlives the verify-start kill (the
base image's supervisord respawns the desktop it runs as that account) is frozen, not held against the submission; one
that is still running once frozen still voids the run (no verdict); a process supervisord starts again after a clean
kill is stopped by the freezer that runs from the kill to the verdict.

Runs the grader's own `_early_lockdown` (and its helpers), taken from tests/verify.py by name, against stand-ins for
the root-only parts (the kill, the stage lock, the snapshot) and a real child process playing the survivor; runs the
grader's own freezer (tests/verify_freeze.py) on a real child playing the respawned process; and checks from the syntax
tree that tests/verify.py voids the run on the running count only, outside any handler that would write a verdict, and
stops the freezer only after the verdict. Stdlib only: the grading stack (numpy, mujoco, ...) that tests/verify.py
imports later is not needed. Nothing here signals a process it did not start.
"""
import ast
import importlib.util
import os
import signal
import stat
import subprocess
import tempfile
import time
import types
import unittest
from pathlib import Path

TESTS = Path(__file__).resolve().parents[1] / "tasks/hand/dexterous_hand_robot_description_from_photos/tests"
VERIFY = TESTS / "verify.py"


class _Os(types.ModuleType):
    """os, as root (getuid 0), with an optional stand-in for kill."""

    def __init__(self, kill=None):
        super().__init__("os")
        self._kill = kill

    def __getattr__(self, name):
        return getattr(os, name)

    @staticmethod
    def getuid():
        return 0

    def kill(self, pid, sig):
        return (self._kill or os.kill)(pid, sig)


class _Survivor:
    """A real process standing in for the respawned desktop process that outlived the kill."""

    def __init__(self):
        self.proc = subprocess.Popen(["sleep", "60"])

    def pids(self, uid=None):
        return [self.proc.pid] if self.proc.poll() is None else []

    def state(self):
        return Path("/proc/%d/status" % self.proc.pid).read_text().split("State:")[1].split()[0]

    def close(self):
        try:
            os.kill(self.proc.pid, signal.SIGCONT)
        except OSError:
            pass
        self.proc.kill()
        self.proc.wait()


def _functions(path, names):
    tree = ast.parse(path.read_text())
    body = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
    assert {n.name for n in body} == set(names), "%s lacks %s" % (path.name, set(names) - {n.name for n in body})
    return compile(ast.Module(body=body, type_ignores=[]), str(path), "exec")


def _module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _main():
    return next(n for n in ast.parse(VERIFY.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == "main")


class SurvivorAtVerifyStart(unittest.TestCase):
    def setUp(self):
        self.survivor = _Survivor()
        self.addCleanup(self.survivor.close)
        self.tmp = Path(self.enterContext(tempfile.TemporaryDirectory()))
        (self.tmp / "stage").mkdir()

    def _lockdown(self, left, freeze=True):
        """Run _early_lockdown with the kill reporting `left` survivors; `freeze` False: SIGSTOP has no effect."""
        logs = []
        survivor = self.survivor
        stop = (lambda pid, sig: None) if not freeze else None

        class Freezer:
            frozen = set()

            def start(self):
                return self

        lockdown = types.SimpleNamespace(
            stop_and_kill_uid=lambda uid, grace_s=5.0: (52, left),
            live_uid_processes=survivor.pids,
            stage_upload_time=lambda d: None, late_entries=lambda *a, **k: [])
        ns = {"os": _Os(kill=stop), "pwd": types.SimpleNamespace(getpwnam=lambda n: types.SimpleNamespace(pw_uid=4242)),
              "time": time, "signal": signal, "statmod": stat, "lockdown": lockdown, "FREEZER": Freezer(),
              "lock_stage": lambda d: {}, "log": logs.append, "_submission_root_ok": lambda: (True, None),
              "_submission_measure": lambda root: (0, 0, 0), "_snapshot_dir": lambda a, b: None,
              "STAGE_DIR": str(self.tmp / "stage"), "HOME": str(self.tmp / "home"), "AGENT_USER": "user",
              "SUBMISSION": str(self.tmp / "home" / "submission"), "LATE_GATE_MAX_GAP_S": 600.0,
              "SUBMISSION_ENTRIES_CAP": 20000, "SUBMISSION_BYTES_CAP": 200 * 1024 * 1024,
              "SNAP_ROOT": str(self.tmp / "snap_root"), "SNAPSHOT": str(self.tmp / "snap_root" / "submission")}
        exec(_functions(VERIFY, ["_proc_state", "_still_running", "_early_lockdown"]), ns)
        return ns["_early_lockdown"](), logs

    def test_frozen_survivor_does_not_void(self):
        out, logs = self._lockdown(left=1)
        self.assertEqual(out["agent_processes_left"], 1.0)
        self.assertEqual(out["agent_processes_running"], 0.0, out)
        self.assertTrue(any("outlived the kill" in m for m in logs), logs)
        self.assertEqual(self.survivor.state(), "T")

    def test_survivor_that_cannot_be_stopped_still_voids(self):
        out, _ = self._lockdown(left=1, freeze=False)
        self.assertGreater(out["agent_processes_running"], 0, out)

    def test_clean_kill_records_nothing(self):
        out, logs = self._lockdown(left=0)
        self.assertEqual(out["agent_processes_left"], 0.0)
        self.assertEqual(out["agent_processes_running"], 0.0)
        self.assertFalse(any("outlived" in m for m in logs), logs)


class RespawnAfterTheKill(unittest.TestCase):
    """The case of the image's own supervisor: after a clean kill it starts the account's desktop again; the grader's
    freezer (tests/verify_freeze.py, the real class) stops such a process within its period and keeps it stopped."""

    def test_the_freezer_stops_a_process_that_appears_after_the_kill(self):
        vf = _module(TESTS / "verify_freeze.py", "hd04_verify_freeze")
        survivor = _Survivor()
        self.addCleanup(survivor.close)
        appeared = []

        def lister(uid):                       # only the child this test started, and only once it "respawned"
            return list(appeared)

        freezer = vf.AgentFreezer(4242, period_s=0.02, lister=lister)
        freezer.start(require_root=False)
        self.addCleanup(freezer.stop)
        time.sleep(0.1)
        self.assertEqual(survivor.state() in ("S", "R"), True)
        appeared.append(survivor.proc.pid)     # the respawn
        deadline = time.time() + 5.0
        while survivor.state() != "T" and time.time() < deadline:
            time.sleep(0.02)
        self.assertEqual(survivor.state(), "T")
        self.assertIn(survivor.proc.pid, freezer.frozen)

    def test_zombies_are_not_counted_as_live(self):
        lk = _module(TESTS / "lockdown.py", "hd04_lockdown")
        child = subprocess.Popen(["true"])
        deadline = time.time() + 5.0
        while time.time() < deadline:
            st = Path("/proc/%d/status" % child.pid).read_text().split("State:")[1].split()[0]
            if st.startswith("Z"):
                break
            time.sleep(0.01)
        try:
            self.assertNotIn(child.pid, lk.live_uid_processes(os.getuid()))
        finally:
            child.wait()


class VoidIsNoVerdict(unittest.TestCase):
    def test_only_a_running_survivor_raises(self):
        """main() voids the run on agent_processes_running, never on the kill's raw survivor count."""
        tests = [ast.unparse(n.test) for n in ast.walk(_main())
                 if isinstance(n, ast.If) and any(isinstance(b, ast.Raise) for b in n.body)]
        self.assertFalse([t for t in tests if "agent_processes_left" in t], tests)
        self.assertTrue([t for t in tests if "agent_processes_running" in t], tests)

    def test_the_void_is_not_turned_into_a_zero_verdict(self):
        """The raise for a running survivor sits only inside try blocks that catch TaskError, so no handler writes a
        verdict for it (a void, not a score of 0)."""
        main = _main()
        target = next(n for n in ast.walk(main) if isinstance(n, ast.If) and "agent_processes_running" in ast.unparse(n.test))
        for node in ast.walk(main):
            if isinstance(node, ast.Try) and any(target is x for b in node.body for x in ast.walk(b)):
                names = [ast.unparse(h.type) if h.type is not None else "bare" for h in node.handlers]
                self.assertEqual(names, ["TaskError"], names)

    def test_the_freezer_runs_from_the_kill_to_after_the_verdict(self):
        tree = ast.parse(VERIFY.read_text())
        early = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "_early_lockdown")
        src = ast.unparse(early)
        self.assertLess(src.index("lockdown.stop_and_kill_uid"), src.index("FREEZER.start()"))
        self.assertLess(src.index("FREEZER.start()"), src.index("lock_stage(STAGE_DIR)"))
        main = _main()
        finals = [n for n in ast.walk(main) if isinstance(n, ast.Try) and n.finalbody]
        self.assertTrue(any("FREEZER.stop()" in ast.unparse(f) for t in finals for f in t.finalbody))
        body = ast.unparse(main)
        self.assertLess(body.index("write_verdict(GR, rewards, cfg, stats, details)"), body.rindex("FREEZER.stop()"))


if __name__ == "__main__":
    unittest.main()
