#!/usr/bin/env python3
"""The grader's own safeguards without the simulator (maintainers' tool; the grader never runs it): the real kit
Solver (the non-root path of verify.py's GuardedSolver, `bash run.sh` in a submission folder) against a stub cell that
applies joint moves kinematically and can be told to fail.

    python3 test_grader.py            (NumPy needed: the Isaac Sim venv or any Python 3.10+ with numpy)

- the deadline: verify.main with `verifier_deadline_s` cut to a few seconds writes a verdict from the packs graded by
  then, the rest at 0, `verifier_deadline_expired` 1, and the process ends with status 0;
- the process cap: a controller that starts more processes than `max_processes` has the pack ended and scored, and
  is started again for the next pack;
- the process cap: counted before every request (the pack in which a controller passes it ends at its next request);
- a malformed action with a JSON integer too large for a float (the re-review's N1), applied or in a probe pair;
- a cell failure: the controller (and what it started) is stopped before the fresh cell starts, its account's files
  outside the submission and its SysV IPC objects are removed and the submission is restored as staged, the restart
  is not counted against the three, a second failure after the controller was started again is scored (the pack
  counts 0, grading goes on), and a second failure with nothing of the submission left is a task error.
"""

import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
sys.path.insert(0, TESTS)
sys.dont_write_bytecode = True

import numpy as np  # noqa: E402

import pack_grade as G  # noqa: E402
import verify as V  # noqa: E402
from packcell import cellproc, kin, wire  # noqa: E402
from packcell import series as S  # noqa: E402

CONTROLLER = r'''
import json, os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
SC = open(os.path.join(HERE, "scenario")).read().strip()
with open(os.path.join(HERE, "launches.log"), "a") as f:
    f.write("%d\n" % os.getpid())
print("READY", flush=True)
kids = []
for line in sys.stdin:
    m = json.loads(line)
    if m["type"] == "reset":
        print(json.dumps({"ok": True}), flush=True); continue
    if m["type"] == "close":
        print(json.dumps({"ok": True}), flush=True); break
    t = m["t"]
    if SC == "forks" and t == 2 and not kids:
        kids = [subprocess.Popen(["sleep", "300"]) for _ in range(40)]
    if SC == "helper" and t == 1 and not kids:
        kids = [subprocess.Popen(["sleep", "300"])]
        with open(os.path.join(HERE, "helpers.log"), "a") as f:
            f.write("%d\n" % kids[0].pid)
    if SC == "slow":
        time.sleep(0.25)
    qt = m["obs"]["q_target"]
    act = {"move": {"type": "joint", "delta": [0.001 + 1e-3 * qt[0], 0, 0, 0, 0, 0], "ticks": 1}}
    if SC == "huge" and t == 3:
        act = {"move": {"type": "joint", "delta": [int("1" * 400), 0, 0, 0, 0, 0], "ticks": 1}}
    if t >= 12:
        act = {"cmd": "done"}
    print(json.dumps({"action": act}), flush=True)
'''


class StubCell(object):
    """Applies joint moves; credit 1.0 for a pack that ended with `done`, else 0. `fail_at`: (attempt, step) at which
    `step` raises a CellError; a start listed in `fail_start` raises at once."""

    starts = 0
    fail_at = set()
    fail_start = set()
    on_start = None

    def __init__(self):
        StubCell.starts += 1
        self.attempt = StubCell.starts
        if StubCell.on_start is not None:
            StubCell.on_start(self.attempt)
        if self.attempt in StubCell.fail_start:
            raise cellproc.CellError("stub: the cell did not start (attempt %d)" % self.attempt)
        self.q = np.array(S.CELL["home_q"], dtype=float)
        self.t, self.n, self.ended = 0.0, 0, None

    def obs(self):
        T = kin.flange(self.q) @ kin.pose(np.eye(3), np.array([0, 0, S.TOOLS["nutrunner"]["tcp_z"]]))
        q = [round(float(v), 6) for v in self.q]
        return {"time": round(self.t, 3), "q": q, "qd": [0.0] * 6, "q_target": q, "tcp": wire.pose6(T),
                "tcp_target": wire.pose6(T), "wrench": [0.0] * 6, "tool": "nutrunner", "bit": "S10", "spindle": {},
                "gripper": {}, "status": {}, "trace": {}, "events": [], "frames_left": 400, "time_left": 2700 - self.t,
                "ended": self.ended, "images": {}}

    def request(self, msg, timeout=0):
        cmd = msg["cmd"]
        if cmd == "reset":
            return {"obs": self.obs()}
        if cmd == "step":
            if (self.attempt, self.n) in StubCell.fail_at:
                raise cellproc.CellError("stub: the cell died (attempt %d, step %d)" % (self.attempt, self.n))
            a = wire.check_action(msg["action"])
            self.n += 1
            if a["move"]["type"] == "joint":
                self.q = self.q + np.array(a["move"]["delta"])
            self.t += 0.05 * a["move"]["ticks"]
            if a["cmd"] == "done":
                self.ended = "done"
            return {"obs": self.obs()}
        if cmd == "joints":
            return {"joints": {"q": [float(v) for v in self.q], "q_target": [float(v) for v in self.q]}}
        if cmd == "score":
            if not self.ended:
                self.ended = "given_up"
            return {"score": {"credit": 1.0 if self.ended == "done" else 0.0, "ended": self.ended, "time": self.t,
                              "frames": 0, "protective_stops": 0, "stripped": [], "detail": {"x": True},
                              "events": [], "wall": {}}}
        return {"error": "?"}

    def close(self):
        pass


def reset_stub():
    StubCell.starts, StubCell.fail_at, StubCell.fail_start, StubCell.on_start = 0, set(), set(), None


def submission(work, scenario):
    sub = os.path.join(work, "home", "submission")
    os.makedirs(sub, exist_ok=True)
    for name in ("launches.log", "helpers.log"):
        if os.path.exists(os.path.join(sub, name)):
            os.remove(os.path.join(sub, name))
    with open(os.path.join(sub, "controller.py"), "w") as f:
        f.write(CONTROLLER)
    with open(os.path.join(sub, "run.sh"), "w") as f:
        f.write("exec %s -I controller.py\n" % sys.executable)
    with open(os.path.join(sub, "scenario"), "w") as f:
        f.write(scenario)
    return sub


def alive(pid):
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    try:
        with open("/proc/%d/stat" % pid) as f:
            return f.read().split(") ")[-1].split()[0] not in ("Z", "X")
    except OSError:
        return False


def lines(path):
    try:
        return [int(x) for x in open(path).read().split()]
    except OSError:
        return []


def grade(sub, packs, config, **kw):
    clock = G.Clock()
    solver = clock.timed(V.GuardedSolver, step_timeout=config["step_timeout_s"], submission=sub)
    try:
        records, lock = G.run_packs(solver, packs, False, config, StubCell, log=lambda m: print("  [grader]", m),
                                    clock=clock, **kw)
    finally:
        solver.close()
    return solver, records


def check(name, ok, detail=""):
    print("%s  %s%s" % ("ok  " if ok else "FAIL", name, (": " + detail) if detail else ""))
    return bool(ok)


def test_process_cap(work, packs, config):
    reset_stub()
    sub = submission(work, "forks")
    base = len([p for p in os.listdir("/proc") if p.isdigit() and os.stat("/proc/" + p).st_uid == os.getuid()])
    V.GuardedSolver.max_processes = base + 15
    real_uid = V.GuardedSolver._uid
    V.GuardedSolver._uid = lambda self: os.getuid()        # outside the engine the agent account does not exist
    try:
        solver, records = grade(sub, packs[:3], config)
    finally:
        V.GuardedSolver._uid = real_uid
        V.GuardedSolver.max_processes = 0
    r = records
    res = [check("process cap: every pack in which the controller starts its forks (at t = 2) ends at its next request, scored",
                 all(x["aborted"] and "too many processes" in x["aborted"] and x["ended"] == "given_up" and x["credit"] == 0.0
                     and x["steps"] == 3 for x in r), str([(x["ended"], x["aborted"], x["steps"]) for x in r])),
           check("process cap: run.sh started again for the next pack, its children stopped",
                 len(lines(os.path.join(sub, "launches.log"))) >= 2 and solver.restarts >= 1,
                 "launches %d, restarts %d" % (len(lines(os.path.join(sub, "launches.log"))), solver.restarts))]
    return all(res)


def test_huge_numbers(work, packs, config):
    """N1 of the re-review: an action with a JSON integer too large for a float is a malformed action (the pack
    ends there, grading goes on), on the applied path and on the probe lock's path; never an OverflowError."""
    reset_stub()
    sub = submission(work, "huge")
    solver, r = grade(sub, packs[:2], config)
    res = [check("a 400-digit integer in an applied action (at t = 3, every start): each pack ends there as malformed",
                 len(r) == 2 and all("malformed action" in (x["aborted"] or "") and x["steps"] == 3 for x in r),
                 str([(x["ended"], x["aborted"]) for x in r]))]
    lock = G.IdleAwareProbeLock(None, rate=1.0, pass_fraction=0.1, tolerance=1e-4, canonical=G.canonical)
    huge = {"move": {"type": "joint", "delta": [int("9" * 400), 0, 0, 0, 0, 0]}}
    ok = False
    try:
        lock.record(huge, {"move": {"type": "joint", "delta": [0.01, 0, 0, 0, 0, 0]}})
    except V.SolverAborted as exc:
        ok = "malformed" in str(exc)
    res.append(check("a 400-digit integer in a probe pair's reply: a malformed action (SolverAborted)", ok))
    return all(res)


def test_cell_failure(work, packs, config):
    res = []
    # (a) the first attempt of pack 1 dies with a live controller (and its helper); the retry succeeds
    reset_stub()
    sub = submission(work, "helper")
    seen = {}

    def on_start(attempt):
        if attempt == 3:        # the retry of pack 1: nothing of the controller may be alive now
            pids = lines(os.path.join(sub, "launches.log")) + lines(os.path.join(sub, "helpers.log"))
            seen["alive"] = [p for p in pids if alive(p)]
    StubCell.fail_at, StubCell.on_start = {(2, 5)}, on_start
    solver, r = grade(sub, packs[:3], config)
    res.append(check("cell failure: controller and its helper stopped before the fresh cell starts",
                     seen.get("alive") == [], "alive at the retry: %s" % seen.get("alive")))
    res.append(check("cell failure: the retried pack is graded in full",
                     r[1]["credit"] == 1.0 and r[1].get("cell_retried") and len(r) == 3, str([x["credit"] for x in r])))
    res.append(check("cell failure: that restart is not one of the controller's three",
                     solver.restarts == 1 and solver.max_restarts == 4, "restarts %d of %d" % (solver.restarts, solver.max_restarts)))
    # (b) the retry dies again after the controller was started again: scored, grading goes on
    reset_stub()
    sub = submission(work, "plain")
    StubCell.fail_at = {(2, 5), (3, 4)}
    solver, r = grade(sub, packs[:3], config)
    res.append(check("second failure while the controller ran: the pack counts 0, grading goes on",
                     len(r) == 3 and r[1]["ended"] == "cell_failed" and r[1]["credit"] == 0.0 and r[2]["credit"] == 1.0,
                     str([(x["ended"], x["credit"]) for x in r])))
    res.append(check("second failure while the controller ran: neither restart counted",
                     solver.restarts == 2 and solver.max_restarts == 5, "restarts %d of %d" % (solver.restarts, solver.max_restarts)))
    # (c) the retry's cell does not start (nothing of the submission runs): a task error
    reset_stub()
    sub = submission(work, "plain")
    StubCell.fail_at, StubCell.fail_start = {(2, 5)}, {3}
    try:
        grade(sub, packs[:3], config)
        res.append(check("second failure with nothing of the submission running: a task error", False, "no exception"))
    except cellproc.CellError as exc:
        res.append(check("second failure with nothing of the submission running: a task error", True, str(exc)))
    # (d) the account is emptied before the retry: every file of it outside the submission (links not followed),
    #     and the submission is restored as it was staged (verify.scrub_files, restore_submission)
    top = tempfile.mkdtemp(dir=work)
    home, keep = os.path.join(top, "home"), os.path.join(top, "home", "submission")
    os.makedirs(os.path.join(keep, "lib"))
    os.makedirs(os.path.join(home, ".cache", "deep"))
    os.makedirs(os.path.join(top, "tmp", "notes"))
    for fp in (os.path.join(keep, "run.sh"), os.path.join(keep, "lib", "a.py"), os.path.join(home, ".cache", "deep", "x"),
               os.path.join(top, "tmp", "notes", "pack1.json"), os.path.join(top, "tmp", "fill")):
        with open(fp, "w") as f:
            f.write("x")
    outside = tempfile.mkdtemp(dir=work)
    with open(os.path.join(outside, "victim"), "w") as f:
        f.write("x")
    os.symlink(outside, os.path.join(top, "tmp", "link"))
    snap = os.path.join(tempfile.mkdtemp(dir=work), "s")
    shutil.copytree(keep, snap, symlinks=True)
    with open(os.path.join(keep, "notes_of_pack1"), "w") as f:
        f.write("x")
    os.unlink(os.path.join(keep, "lib", "a.py"))
    removed = V.scrub_files(os.getuid(), keep, tops=(top,))
    V.restore_submission(keep, snap, os.getuid(), os.getgid())
    left = sorted(os.path.relpath(os.path.join(dp, n), top) for dp, dns, fns in os.walk(top) for n in dns + fns)
    res.append(check("cell failure: the account's files outside the submission are removed, links not followed",
                     left == ["home", "home/submission", "home/submission/lib", "home/submission/lib/a.py", "home/submission/run.sh"]
                     and os.path.exists(os.path.join(outside, "victim")) and removed == 2, "%s, removed %d" % (left, removed)))
    # SysV IPC: the account's objects are found (one is made here and removed by id only: never every object of the
    # account running this test)
    import ctypes
    libc = ctypes.CDLL(None, use_errno=True)
    shm = libc.shmget(0, 4096, 0o1600)            # IPC_PRIVATE, IPC_CREAT | 0600
    found = ("shm", shm) in V.sysv_objects(os.getuid()) if shm >= 0 else False
    gone = V.remove_sysv([("shm", shm)]) == 1 if shm >= 0 else False
    res.append(check("cell failure: the account's SysV shared memory is found and removed",
                     shm >= 0 and found and gone and ("shm", shm) not in V.sysv_objects(os.getuid()), "id %d" % shm))
    return all(res)


DEADLINE_CHILD = r'''
import os, sys
sys.path.insert(0, os.environ["TG_TESTS"]); sys.path.insert(0, os.environ["TG_REVIEW"])
sys.dont_write_bytecode = True
import test_grader as T
import verify as V
V.canary = lambda host, config: (0.0, 0.0)
V.CellHost.start = lambda self, timeout: T.StubCell()
V.main()
print("main returned", flush=True)
'''


def stage_main(work, scenario, deadline_s):
    """verify.main in a child process with the stub cell; returns (exit status, stdout, stderr, wall, verdict)."""
    stage = os.path.join(work, "stage")
    os.makedirs(stage, exist_ok=True)
    cfg = json.load(open(os.path.join(TESTS, "grader_config.json")))
    cfg.update(verifier_deadline_s=deadline_s, step_timeout_s=5)
    json.dump(cfg, open(os.path.join(stage, "grader_config.json"), "w"))
    shutil.copy(os.path.join(TESTS, "anchor.json"), stage)
    sub = submission(work, scenario)
    verdict = os.path.join(work, "verdict-%s.json" % scenario)
    env = dict(os.environ, ALE_HOME=os.path.dirname(sub), ALE_STAGE_DIR=stage, ROBOTICS_GRADER_VERDICT_PATH=verdict,
               TG_TESTS=TESTS, TG_REVIEW=HERE)
    env.pop("ALE_VERIFY_PYTHON", None)
    t0 = time.monotonic()
    p = subprocess.run([sys.executable, "-c", DEADLINE_CHILD], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                       timeout=120, universal_newlines=True)
    wall = time.monotonic() - t0
    try:
        v = json.load(open(verdict))
    except (OSError, ValueError):
        v = {}
    return p, wall, v


def test_deadline(work):
    p, wall, v = stage_main(work, "slow", 7)            # 13 replies of 0.25 s a pack: about 3.3 s a pack
    m = v.get("metrics", {})
    graded = int(m.get("packs_graded", -1))
    res = [check("deadline: the stage ends with status 0 near the deadline, main did not return",
                 p.returncode == 0 and "main returned" not in p.stdout and wall < 30, "exit %s, %.1f s" % (p.returncode, wall)),
           check("deadline: a verdict with the cause", m.get("verifier_deadline_expired") == 1.0, str(sorted(m))[:300]),
           check("deadline: the packs graded so far count, the rest 0",
                 0 < graded < 6 and abs(m.get("mean_pack_credit", -1) - graded / 6.0) < 1e-9,
                 "graded %d, mean_pack_credit %s, reward %s" % (graded, m.get("mean_pack_credit"), v.get("rewards")))]
    p2, _wall, v2 = stage_main(work, "plain", 600)
    m2 = v2.get("metrics", {})
    res.append(check("no deadline reached: main writes the verdict of all six packs itself",
                     p2.returncode == 0 and "main returned" in p2.stdout and m2.get("packs_graded") == 6.0
                     and m2.get("mean_pack_credit") == 1.0 and "verifier_deadline_expired" not in m2,
                     "exit %s, graded %s, mean_pack_credit %s" % (p2.returncode, m2.get("packs_graded"), m2.get("mean_pack_credit"))))
    if not all(res):
        print(p.stderr[-2000:], p2.stderr[-2000:])
    return all(res)


def main():
    work = tempfile.mkdtemp(prefix="test-grader-")
    packs = V.load_set(V.PACKS)
    config = dict(G.CONFIG)
    config.update(step_timeout_s=5.0, probe_rate=0.2)
    try:
        ok = [test_cell_failure(work, packs, config), test_process_cap(work, packs, config), test_huge_numbers(work, packs, config),
              test_deadline(work)]
    finally:
        shutil.rmtree(work, ignore_errors=True)
    print("grader safeguards: %s" % ("all ok" if all(ok) else "FAILED"))
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
