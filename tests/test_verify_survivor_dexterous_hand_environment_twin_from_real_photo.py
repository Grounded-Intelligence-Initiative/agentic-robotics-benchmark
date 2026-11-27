"""dexterous_hand_environment_twin_from_real_photo: a process of the agent account that outlives the verify-start kill
(the base image's supervisord respawns the desktop it runs as that account) is frozen, not held against the
submission; one that is still running once frozen still voids the run (task error). A write into the submission after
the stage upload began zeroes gate G1 (submission integrity), and the lockdown runs in its order: the stage closed
before the interpreter starts, the upload instant read, the kill, the freeze, the account freezer, the stage lock, the
late-write gate, the snapshot, all before the grader reads its configuration or starts the worker.

Runs the grader's own `_early_lockdown` (and its helpers), taken from tests/verify.py by name, against stand-ins for
the root-only parts (the kill, the account freezer, the stage lock, the snapshot) and a real child process playing the
survivor; the late-write gate also runs with the vendored lockdown's own `stage_upload_time` and `late_entries`
(tests/arm/assembly_six/lockdown.py) on real files. Stdlib only: tests/verify.py imports the vendored lockdown at
module level and the grading stack (numpy, mujoco, ...) in its worker, neither of which the host venv has.

Also: the grading worker runs on OpenBLAS's Haswell kernels with numpy's AVX-512 code paths off on every CPU
(FP_PINNING in its environment, whatever the verify stage's own environment says), with OPENBLAS_L2_SIZE=256 beside
the kernel, so that verifiers on different CPUs compute the same bits. Runs the grader's own `run_worker`, taken from
tests/verify.py by name, with the worker's start stubbed.

And: a worker that ends on a signal before the grade finished, the gate having passed, is reported as a crash in the
task error's text, whether it was started directly (a negative exit code) or, as root, through runuser (exit code
128 + the signal's number); a worker that ran out of time keeps the timeout wording. Runs main()'s own branch for it,
taken from tests/verify.py by its condition.

The lockdown is the one tests/test_verify_survivor_quadruped_environment_twin_from_real_photos.py holds for that task
(its functions have the same syntax tree); this grader keeps the lockdown's counts in its log and its grade dump, not
in the verdict, so the wiring test reads `_integrity(early)` instead of a statistics dictionary.
"""
import ast
import json
import os
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path
from unittest import mock

TASK = Path(__file__).resolve().parents[1] / "tasks/hand/dexterous_hand_environment_twin_from_real_photo"
TESTS = TASK / "tests"
VERIFY = TESTS / "verify.py"
TEST_SH = TESTS / "test.sh"
LOCKDOWN = TESTS / "arm/assembly_six/lockdown.py"
GRADER = ["_proc_state", "_still_running", "_submission_root_ok", "_submission_measure", "_early_lockdown", "_g1"]


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


def _vendored_lockdown():
    """The vendored lockdown's `stage_upload_time` and `late_entries` (stdlib only), loaded without importing the
    module, so nothing is written into the task folder."""
    ns = {"os": os, "stat": stat}
    exec(_functions(LOCKDOWN, ["stage_upload_time", "late_entries"]), ns)
    return ns["stage_upload_time"], ns["late_entries"]


def _main():
    return next(n for n in ast.parse(VERIFY.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == "main")


def _module_literal(name):
    node = next(n for n in ast.parse(VERIFY.read_text()).body
                if isinstance(n, ast.Assign) and [ast.unparse(t) for t in n.targets] == [name])
    return ast.literal_eval(node.value)


def _module_constant(name):
    """A module-level constant of tests/verify.py written as arithmetic on numbers (such as 300 * 1024 * 1024)."""
    node = next(n for n in ast.parse(VERIFY.read_text()).body
                if isinstance(n, ast.Assign) and [ast.unparse(t) for t in n.targets] == [name])
    assert all(isinstance(x, (ast.BinOp, ast.Constant, ast.operator)) for x in ast.walk(node.value)), name
    return eval(compile(ast.Expression(node.value), str(VERIFY), "eval"), {"__builtins__": {}})


class SurvivorAtVerifyStart(unittest.TestCase):
    def setUp(self):
        self.survivor = _Survivor()
        self.addCleanup(self.survivor.close)
        self.tmp = Path(self.enterContext(tempfile.TemporaryDirectory()))
        (self.tmp / "stage").mkdir()
        self.home = self.tmp / "home"
        self.submission = self.home / "submission"

    def _lockdown(self, left, freeze=True, upload_time=None, late_entries=None, events=None):
        """Run _early_lockdown with the kill reporting `left` survivors; `freeze` False: SIGSTOP has no effect.
        `upload_time` / `late_entries`: the stage-upload instant and the late-write scan (default: no upload instant,
        so the late-write gate is off, and no late entry); `events`: a list the stand-ins append their names to."""
        logs = []
        survivor = self.survivor
        stop = (lambda pid, sig: None) if not freeze else None
        started = []
        note = (events.append if events is not None else (lambda e: None))

        class Freezer:
            frozen = set()

            def start(self):
                note("freezer")
                started.append(survivor.state() if survivor.proc.poll() is None else "X")
                return self

        def stop_and_kill_uid(uid, grace_s=5.0):
            note("kill")
            return 54, left

        def stage_upload_time(d):
            note("upload_time")
            return upload_time(d) if callable(upload_time) else upload_time

        def scan_late(root, t_ref, **kw):
            note("late_entries")
            return late_entries(root, t_ref, **kw) if late_entries else []

        def lock_stage(d):
            note("lock_stage")
            return {}

        def snapshot_dir(a, b):
            note("snapshot")

        lockdown = types.SimpleNamespace(
            stop_and_kill_uid=stop_and_kill_uid, live_uid_processes=survivor.pids,
            stage_upload_time=stage_upload_time, late_entries=scan_late, snapshot_dir=snapshot_dir)
        runner = types.SimpleNamespace(lock_stage=lock_stage)
        ns = {"os": _Os(kill=stop), "pwd": types.SimpleNamespace(getpwnam=lambda n: types.SimpleNamespace(pw_uid=4242)),
              "time": time, "signal": signal, "statmod": stat, "lockdown": lockdown, "runner": runner,
              "FREEZER": Freezer(), "log": logs.append,
              "SUBMISSION_ENTRIES_CAP": _module_literal("SUBMISSION_ENTRIES_CAP"),
              "SUBMISSION_BYTES_CAP": _module_constant("SUBMISSION_BYTES_CAP"),
              "LATE_GATE_MAX_GAP_S": _module_literal("LATE_GATE_MAX_GAP_S"),
              "STAGE_DIR": str(self.tmp / "stage"), "HOME": str(self.home), "AGENT_USER": "user",
              "SUBMISSION": str(self.submission), "SNAP_ROOT": str(self.tmp / "snap"),
              "SNAPSHOT": str(self.tmp / "snap" / "submission")}
        exec(_functions(VERIFY, GRADER), ns)
        out = ns["_early_lockdown"]()
        self.assertTrue(started, "the account freezer is started after the kill")
        self.started = started
        return out, logs, ns

    # --- a process that survives into verify

    def test_frozen_survivor_does_not_void(self):
        out, logs, ns = self._lockdown(left=1)
        self.assertEqual(out["agent_processes_left"], 1.0)
        self.assertEqual(out["agent_processes_running"], 0.0, out)
        self.assertTrue(any("outlived the kill" in m for m in logs), logs)
        self.assertEqual(self.survivor.state(), "T")
        self.assertEqual(out["reasons"], [])
        self.assertEqual(ns["_g1"](out), 1.0)

    def test_survivor_that_cannot_be_stopped_still_voids(self):
        out, _, _ = self._lockdown(left=1, freeze=False)
        self.assertGreater(out["agent_processes_running"], 0.0, out)

    def test_clean_kill_records_nothing(self):
        out, logs, ns = self._lockdown(left=0)
        self.assertEqual(out["agent_processes_left"], 0.0)
        self.assertEqual(out["agent_processes_running"], 0.0)
        self.assertFalse(any("outlived" in m for m in logs), logs)
        self.assertEqual(ns["_g1"](out), 1.0)

    def test_a_process_respawned_after_the_kill_is_left_to_the_freezer(self):
        """The kill reports none left (the image's supervisord respawns about a second later): no SIGSTOP pass runs,
        the account freezer is started all the same, and nothing is held against the submission."""
        out, logs, ns = self._lockdown(left=0)
        self.assertEqual(self.started, ["S"], "no SIGSTOP pass before the freezer when the kill left none")
        self.assertEqual(out["agent_processes_running"], 0.0)
        self.assertFalse(any("outlived" in m for m in logs), logs)
        self.assertEqual(ns["_g1"](out), 1.0)

    def test_only_a_running_survivor_voids(self):
        """main() voids the run (fail(), a task error) on agent_processes_running, never on the kill's raw count."""
        def voids(node):
            return any(isinstance(b, ast.Raise) or (isinstance(b, ast.Expr) and isinstance(b.value, ast.Call)
                                                   and getattr(b.value.func, "id", None) == "fail")
                       for b in node.body)
        tests = [ast.unparse(n.test) for n in ast.walk(_main()) if isinstance(n, ast.If) and voids(n)]
        self.assertFalse([t for t in tests if "agent_processes_left" in t], tests)
        self.assertTrue([t for t in tests if "agent_processes_running" in t], tests)

    def test_g1_reads_the_four_reasons_only(self):
        """G1 is 0 on a late write, a rejected or replaced submission root, or an oversize submission, and on nothing
        the kill or the freeze records."""
        ns = {}
        exec(_functions(VERIFY, ["_g1"]), ns)
        base = {"submission_late_writes": 0.0, "submission_root_rejected": 0.0, "submission_root_violation": 0.0,
                "submission_oversize": 0.0, "agent_processes_left": 7.0, "agent_processes_killed": 54.0,
                "agent_processes_running": 0.0}
        self.assertEqual(ns["_g1"](base), 1.0)
        for key in ("submission_late_writes", "submission_root_rejected", "submission_root_violation",
                    "submission_oversize"):
            with self.subTest(key=key):
                self.assertEqual(ns["_g1"](dict(base, **{key: 1.0})), 0.0)

    # --- a write that survives into verify

    def _late_write(self, rewrite):
        """The submission written during the session, then the stage uploaded (its files' ctime is the upload instant),
        then, if `rewrite`, scene.xml rewritten in place (the folder itself unchanged), as a process left running by
        the agent would."""
        if os.geteuid() == 0:
            self.skipTest("late_entries skips root-owned entries: run as an unprivileged account")
        upload_time, late_entries = _vendored_lockdown()
        self.submission.mkdir(parents=True)
        (self.submission / "scene.xml").write_text("<mujoco/>\n")
        (self.submission / "objects.json").write_text("{}\n")
        time.sleep(0.1)
        for name in ("grader_config.json", "anchor.json"):
            (self.tmp / "stage" / name).write_text("{}\n")
        time.sleep(0.1)
        if rewrite:
            with open(self.submission / "scene.xml", "w") as fh:
                fh.write("<mujoco model='late'/>\n")
        return self._lockdown(left=0, upload_time=upload_time, late_entries=late_entries)

    def test_late_write_zeroes_g1(self):
        out, _, ns = self._late_write(rewrite=True)
        self.assertEqual(out["late_write_gate_active"], 1.0, out)
        self.assertEqual(out["submission_late_writes"], 1.0, out)
        self.assertEqual(out["submission_root_violation"], 0.0, out)
        self.assertTrue(any("written after the agent phase ended (scene.xml)" in r for r in out["reasons"]), out)
        self.assertEqual(ns["_g1"](out), 0.0)

    def test_submission_final_at_session_end_keeps_g1(self):
        """The engine's order in a separate verifier: the submission restored, then the stage uploaded. An honest
        submission keeps G1."""
        out, _, ns = self._late_write(rewrite=False)
        self.assertEqual(out["late_write_gate_active"], 1.0, out)
        self.assertEqual(out["submission_late_writes"], 0.0, out)
        self.assertEqual(out["submission_root_violation"], 0.0, out)
        self.assertEqual(out["submission_snapshot"], 1.0, out)
        self.assertEqual(out["reasons"], [])
        self.assertEqual(ns["_g1"](out), 1.0)

    # --- the lockdown order

    def test_lockdown_order(self):
        """In _early_lockdown: the upload instant is read before anything chmods the stage, the kill comes first, the
        survivor is already stopped when the account freezer starts, then the stage lock, the late-write gate and the
        snapshot every later read uses."""
        self.submission.mkdir(parents=True)
        (self.submission / "scene.xml").write_text("<mujoco/>\n")
        events = []
        out, _, _ = self._lockdown(left=1, upload_time=time.time() + 3600.0, events=events)
        self.assertEqual(events, ["upload_time", "kill", "freezer", "lock_stage", "late_entries", "snapshot"])
        self.assertEqual(self.started, ["T"], "the survivor is frozen before the account freezer starts")
        self.assertEqual(out["submission_snapshot"], 1.0, out)
        self.assertEqual(out["stage_locked"], 1.0, out)

    def test_main_and_test_sh_wiring(self):
        """test.sh closes the stage and its parent before the interpreter starts; main() runs the lockdown first,
        voids on a running survivor and writes G1 = 0 before it starts the worker, reads the worker's input from the
        root-owned snapshot, logs the lockdown's counts, and stops the freezer whatever happens, after the verdict."""
        sh = TEST_SH.read_text()
        launch = sh.index('exec "${ALE_VERIFY_PYTHON:-python3}" -B verify.py')
        self.assertLess(sh.index('chmod 700 "$STAGE"'), launch)
        self.assertLess(sh.index('chmod 700 "$PARENT"'), launch)
        main = _main()
        src = ast.unparse(main)
        first = src.index("early = _early_lockdown()")
        void = src.index("if early['agent_processes_running'] > 0:")
        config = src.index("open(os.path.join(STAGE_DIR, 'grader_config.json'))")
        g1 = src.index("if g1 < 1.0:")
        snap = src.index("sub_dir = SNAPSHOT if early['submission_snapshot']")
        worker = src.index("run_worker(cfg, sub_dir, budget, stats)")
        self.assertLess(first, void)
        self.assertLess(void, config)
        self.assertLess(config, g1)
        self.assertLess(g1, snap)
        self.assertLess(snap, worker)
        self.assertIn("log('integrity: ' + json.dumps(_integrity(early), sort_keys=True))", src)
        finals = [ast.unparse(s) for n in ast.walk(main) if isinstance(n, ast.Try) for s in n.finalbody]
        self.assertIn("_restore(early)", finals)
        self.assertIn("FREEZER.stop()", finals)
        # the lockdown's own exception: a zero verdict and the freezer stopped, never a run without a verdict
        handler = next(h for n in ast.walk(main) if isinstance(n, ast.Try) for h in n.handlers
                       if "_zero_verdict" in ast.unparse(h))
        self.assertIn("FREEZER.stop()", ast.unparse(handler))

    def test_the_freezer_is_the_vendored_one_for_the_agent_account(self):
        src = VERIFY.read_text()
        self.assertIn("FREEZER = verify_freeze.AgentFreezer.for_user(AGENT_USER)", src)
        self.assertIn("from arm import verify_freeze", src)


class WorkerKernels(unittest.TestCase):
    """The environment run_worker hands the grading worker: OpenBLAS pinned to its Haswell kernels with the L2 size
    beside the kernel, numpy's AVX-512 code paths off, one thread each, built from FP_PINNING."""

    PINNED = {
        "NPY_DISABLE_CPU_FEATURES": "X86_V4 AVX512_ICL AVX512_SPR",
        "OPENBLAS_CORETYPE": "Haswell",
        "OPENBLAS_L2_SIZE": "256",
    }

    def setUp(self):
        self.tmp = Path(self.enterContext(tempfile.TemporaryDirectory()))

    def _run_worker(self, parent_env, pinning=None):
        started = []
        work = self.tmp / "work"

        def prepare_work(cfg, sub_dir):
            for d in ("out", "home", "code"):
                (work / d).mkdir(parents=True)
            return str(work), {"progress": str(work / "out" / "progress.json"), "code": str(work / "code")}

        class Proc:
            pid, returncode = 0, None

            def __init__(self, argv, **kwargs):
                started.append((argv, kwargs))

            def wait(self, timeout=None):
                self.returncode = 0
                return 0

        ns = {"os": os, "sys": sys, "json": json, "time": time, "shutil": shutil, "signal": signal,
              "subprocess": types.SimpleNamespace(Popen=Proc, STDOUT=subprocess.STDOUT,
                                                  TimeoutExpired=subprocess.TimeoutExpired),
              "runner": types.SimpleNamespace(runner_account=lambda name, default: None),
              "_prepare_work": prepare_work, "_read_progress": lambda path: {"selftest": {"ok": True}},
              "log": lambda msg: None, "FP_PINNING": _module_literal("FP_PINNING") if pinning is None else pinning}
        exec(_functions(VERIFY, ["run_worker"]), ns)
        cfg = {"runtime": {"worker_margin_s": 30, "stage_caps_s": {}, "worker_address_space_mb": 6144}}
        stats = {}
        with mock.patch.dict(os.environ, {k: v for k, v in parent_env.items() if v is not None}):
            for key in [k for k, v in parent_env.items() if v is None]:
                os.environ.pop(key, None)
            doc, how, _ = ns["run_worker"](cfg, None, 600.0, stats)
        self.assertEqual((doc, how, stats["worker_exit"]), ({"selftest": {"ok": True}}, "finished", 0.0))
        self.assertEqual(len(started), 1)
        argv, kwargs = started[0]
        self.assertIn("grade.worker_main(", argv[-1])
        self.assertIn("-I", argv)
        return kwargs["env"]

    def test_fp_pinning_holds_the_three_variables(self):
        self.assertEqual(_module_literal("FP_PINNING"), self.PINNED)

    def test_the_worker_runs_on_the_haswell_kernels_with_their_block_sizes(self):
        for core in (None, "SkylakeX", ""):
            for l2 in (None, "1024", ""):
                with self.subTest(parent_core=core, parent_l2=l2):
                    env = self._run_worker({"OPENBLAS_CORETYPE": core, "OPENBLAS_L2_SIZE": l2, "STAGE_ONLY": "x"})
                    self.assertEqual({k: env.get(k) for k in self.PINNED}, self.PINNED)
                    self.assertEqual({k: env[k] for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS",
                                                          "MKL_NUM_THREADS")},
                                     {"OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"})
                    self.assertNotIn("STAGE_ONLY", env)
                    self.assertNotIn("MUJOCO_GL", env)  # the worker renders nothing

    def test_the_worker_environment_comes_from_fp_pinning(self):
        env = self._run_worker({}, pinning={"ARB_PINNING_PROBE": "1"})
        self.assertEqual(env.get("ARB_PINNING_PROBE"), "1")
        self.assertEqual({k: env.get(k) for k in self.PINNED}, {k: None for k in self.PINNED})

    def test_the_worker_is_started_in_one_place(self):
        main = _main()
        calls = [ast.unparse(n.func) for n in ast.walk(main) if isinstance(n, ast.Call)]
        self.assertEqual(calls.count("run_worker"), 1, calls)
        tree = ast.parse(VERIFY.read_text())
        holders = {f.name for f in ast.walk(tree) if isinstance(f, ast.FunctionDef) for c in ast.walk(f)
                   if isinstance(c, ast.Constant) and isinstance(c.value, str) and "worker_main(" in c.value}
        self.assertEqual(holders, {"run_worker"})
        popen = {f.name for f in ast.walk(tree) if isinstance(f, ast.FunctionDef) for c in ast.walk(f)
                 if isinstance(c, ast.Call) and ast.unparse(c.func) == "subprocess.Popen"}
        self.assertEqual(popen, {"run_worker"})


def _worker_ended_early():
    """The body of main()'s branch for a worker that ended before the grade finished with the gate passed, taken
    from tests/verify.py by its condition."""
    node = next(n for n in ast.walk(_main()) if isinstance(n, ast.If)
                and ast.unparse(n.test) == "not doc.get('done') and g0 == 1.0")
    return compile(ast.Module(body=node.body, type_ignores=[]), str(VERIFY), "exec")


class WorkerEndText(unittest.TestCase):
    """The task error's text for a worker that ended before the grade finished: a signal is read from a negative exit
    code, or from 128 + its number when runuser started the worker; everything else keeps the timeout wording. Both
    arms end in fail(), and the statistics are left as they were."""

    def _text(self, exit_code, deprivileged, timed_out):
        how = ("killed at the hard timeout" if timed_out else "exit code %d" % exit_code if exit_code else "finished")
        stats = {"worker_exit": float(exit_code), "worker_deprivileged": float(deprivileged),
                 "worker_timed_out": float(timed_out)}
        before = dict(stats)
        reasons = []

        def fail(reason):
            reasons.append(reason)
            raise SystemExit(1)

        with self.assertRaises(SystemExit):
            exec(_worker_ended_early(), {"stats": stats, "how": how, "fail": fail})
        self.assertEqual(stats, before)
        self.assertEqual(len(reasons), 1, reasons)
        return reasons[0], how

    def test_a_crash_is_named_and_a_timeout_keeps_its_wording(self):
        crash = "the worker ended on signal %d before the grade finished (a crash, not a timeout): the episode is void"
        timeout = "the worker was %s before the grade finished: the verifier ran out of time, the episode is void"
        for exit_code, deprivileged, timed_out, sig in (
                (-11, False, False, 11), (-11, True, False, 11), (139, True, False, 11), (134, True, False, 6),
                (129, True, False, 1), (159, True, False, 31), (139, False, False, None), (128, True, False, None),
                (160, True, False, None), (3, True, False, None), (3, False, False, None), (0, False, False, None),
                (-9, False, True, None), (137, True, True, None)):
            with self.subTest(exit_code=exit_code, deprivileged=deprivileged, timed_out=timed_out):
                text, how = self._text(exit_code, deprivileged, timed_out)
                self.assertEqual(text, timeout % how if sig is None else crash % sig)


if __name__ == "__main__":
    unittest.main()
