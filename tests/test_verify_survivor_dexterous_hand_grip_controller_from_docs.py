"""dexterous_hand_grip_controller_from_docs: a process of the agent account that outlives the verify-start kill (the base
image's supervisord respawns the desktop session it runs as that account, in the separate verifier too) is frozen and
not held against the submission; one that is still running once frozen still voids the run.

Runs the grader's own `_early_lockdown` (tests/grip/grader/verify_main.py, taken by name from the syntax tree) against
stand-ins for the root-only parts (the kill, the stage lock, the snapshot, the freezer) and a real child process playing
the survivor; and reads `_main_locked`'s gate expressions: a pass condition may depend on the count of processes still
RUNNING after the freeze, never on the kill's raw survivor count. Stdlib only: the grading stack (numpy, MuJoCo) that
verify_main imports later is not needed.
"""
import ast
import os
import signal
import stat
import subprocess
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path

sys.dont_write_bytecode = True

SLUG = "dexterous_hand_grip_controller_from_docs"
TASK = Path(os.environ.get("ARB_TASK_DIR") or Path(__file__).resolve().parents[1] / "tasks" / "hand" / SLUG)
VERIFY_MAIN = TASK / "tests" / "grip" / "grader" / "verify_main.py"


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

    def close(self):
        try:
            os.kill(self.proc.pid, signal.SIGCONT)
        except OSError:
            pass
        self.proc.kill()
        self.proc.wait()


def _functions(path, names):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    body = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
    missing = set(names) - {n.name for n in body}
    return compile(ast.Module(body=body, type_ignores=[]), str(path), "exec"), missing


def _state(pid):
    try:
        return Path("/proc/%d/status" % pid).read_text().split("State:")[1].split()[0]
    except (OSError, IndexError):
        return "X"


class SurvivorAtVerifyStart(unittest.TestCase):
    def setUp(self):
        self.survivor = _Survivor()
        self.addCleanup(self.survivor.close)
        self.tmp = Path(self.enterContext(tempfile.TemporaryDirectory()))
        for d in ("stage", "home", "home/submission", "opt"):
            (self.tmp / d).mkdir()

    def _lockdown(self, left, freeze=True):
        """Run _early_lockdown with the kill reporting `left` survivors; `freeze` False: SIGSTOP has no effect."""
        logs = []
        survivor = self.survivor
        stop = (lambda pid, sig: None) if not freeze else None

        class Freezer:
            frozen = set()

            def start(self):
                for pid in survivor.pids():
                    if freeze:
                        os.kill(pid, signal.SIGSTOP)
                        self.frozen.add(pid)
                return self

            def stop(self):
                pass

        lockdown = types.SimpleNamespace(
            stop_and_kill_uid=lambda uid, grace_s=5.0, settle_s=0.05: (52, left),
            live_uid_processes=survivor.pids, stage_upload_time=lambda d: None,
            late_entries=lambda *a, **k: [], snapshot_dir=lambda a, b: 0, quarantine=lambda *a: 0,
            process_age_ms=lambda: 1.0)
        runner = types.SimpleNamespace(lock_stage=lambda d: {}, unlock_stage=lambda s: None)
        verify_freeze = types.SimpleNamespace(AgentFreezer=types.SimpleNamespace(for_user=lambda u: Freezer()))
        P = types.SimpleNamespace(stage=str(self.tmp / "stage"), home=str(self.tmp / "home"), agent_user="user",
                                  submission=str(self.tmp / "home" / "submission"), runs=str(self.tmp / "opt" / "runs"),
                                  snapshot=str(self.tmp / "opt" / "runs" / "snap"), quarantine=str(self.tmp / "q"),
                                  sticky=[])
        ns = {"os": _Os(kill=stop), "pwd": types.SimpleNamespace(getpwnam=lambda n: types.SimpleNamespace(pw_uid=4242)),
              "time": time, "signal": signal, "statmod": stat, "lockdown": lockdown, "runner": runner,
              "verify_freeze": verify_freeze, "log": logs.append, "LATE_GATE_MAX_GAP_S": 600.0, "FREEZER": [None]}
        code, missing = _functions(VERIFY_MAIN, ["_early_lockdown"])
        self.assertEqual(missing, set())
        # helpers the fixed grader may define next to _early_lockdown (the CAD sibling's _proc_state / _still_running)
        helpers, _ = _functions(VERIFY_MAIN, ["_proc_state", "_still_running"])
        exec(helpers, ns)
        exec(code, ns)
        return ns["_early_lockdown"](P), logs

    def test_frozen_survivor_does_not_void(self):
        out, _ = self._lockdown(left=1)
        self.assertEqual(out["agent_processes_left"], 1.0)
        self.assertIn("agent_processes_running", out, "the grader records no count of survivors still running")
        self.assertEqual(out["agent_processes_running"], 0.0, out)
        self.assertEqual(_state(self.survivor.proc.pid), "T")

    def test_survivor_that_cannot_be_stopped_still_voids(self):
        out, _ = self._lockdown(left=1, freeze=False)
        self.assertGreater(out.get("agent_processes_running", 0.0), 0, out)

    def test_clean_kill_records_nothing(self):
        out, _ = self._lockdown(left=0)
        self.assertEqual(out["agent_processes_left"], 0.0)
        self.assertEqual(out.get("agent_processes_running", 0.0), 0.0)
        self.assertFalse([r for r in out["reasons"] if "surviv" in r or "outlived" in r], out["reasons"])


class GateExpressions(unittest.TestCase):
    """_main_locked's pass conditions: never the raw survivor count of the kill, always the count still running."""

    def _main_locked_source(self):
        src = VERIFY_MAIN.read_text(encoding="utf-8")
        fn = [n for n in ast.parse(src).body if isinstance(n, ast.FunctionDef) and n.name == "_main_locked"]
        self.assertEqual(len(fn), 1)
        return src, fn[0]

    def test_no_pass_condition_reads_the_raw_survivor_count(self):
        src, fn = self._main_locked_source()
        hits = [ast.get_source_segment(src, n) for n in ast.walk(fn)
                if isinstance(n, ast.Constant) and n.value == "agent_processes_left"
                for n2 in [n]]
        self.assertEqual(hits, [], "a pass condition of _main_locked reads agent_processes_left: a frozen respawn of "
                                   "the image's desktop then zeroes an honest run (benchmark #64)")

    def test_a_running_survivor_fails_the_run(self):
        src, fn = self._main_locked_source()
        whole = VERIFY_MAIN.read_text(encoding="utf-8")
        self.assertIn("agent_processes_running", whole,
                      "nothing in verify_main fails the run on a survivor still running after the freeze")

    def test_the_freezer_starts_after_the_kill_and_stops_after_the_verdict(self):
        src = VERIFY_MAIN.read_text(encoding="utf-8")
        tree = ast.parse(src)
        early = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "_early_lockdown")
        calls = [ast.unparse(c.func) for c in sorted((c for c in ast.walk(early) if isinstance(c, ast.Call)),
                                                     key=lambda c: (c.lineno, c.col_offset))]
        kill = calls.index("lockdown.stop_and_kill_uid")
        freeze = [i for i, c in enumerate(calls) if c.endswith(".start")]
        self.assertTrue(freeze and freeze[0] > kill, calls)
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
        fin = [t for t in ast.walk(main) if isinstance(t, ast.Try) and t.finalbody]
        self.assertTrue(fin and "FREEZER[0].stop" in ast.unparse(fin[0].finalbody[-1]))


if __name__ == "__main__":
    unittest.main()
