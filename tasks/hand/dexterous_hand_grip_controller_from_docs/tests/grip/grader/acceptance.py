"""The acceptance of one hand unit: the bus cycle, the controller's budgets and the unit workers.

    run_unit(scenario, unit_index, endpoint, *, privileged, script=None, ...) -> (job_records, unit_stats)
        endpoint: a wire Runner (grader.runtime.wire) or LocalEndpoint(controller object): call(op, payload, cpu_s,
        wall_s) -> (status, reply, cpu_s, wall_s). The plant (plant.session.Session) runs one 1 ms tick at a time and
        the supervisor (grader.supervisor) reads it after every tick.
    run_workers(jobs, runs_root, isolation=UnitIsolation(...)) -> {unit: (job_records, unit_stats)}
                                                                         the units one after the other, each in
                                                                         its own worker process
    python3 -I -B acceptance.py --unit 1 | 2                              the worker (reads its job from stdin)

The cycle (CONTRACT.md section 4): effective period T = constants.period_ms(period_ms, reads). Cycle k starts at tick
t_k: the previous reply's writes are written then (in force at t_k + 1) and its flags reach the supervisor; the read
blocks are sampled at t_k + 2 and given to step(k); simulated time stands still while the controller computes; then
the plant runs on to t_k + T. A step that uses more than T of CPU is late by floor(cpu / T) cycles: those cycles get
no observation and no command, and its reply is written at the cycle after them. A step that raises sends no command.
A step over 1 s of CPU or 5 s of wall, a dead controller, a protocol error or more than 3,600 s of the controller's
own reply time over the unit ends the controller for the unit: the servos keep their last registers and the
supervisor runs on to the end. A privileged run (the reference script behind the matching token) is exempt from the
late rule and the wall caps; its would-be-late cycles and largest step CPU are recorded.

The worker: argv is [python3, -I, -B, <this file>, --unit N] and nothing else; its working folder is named unitN; its
environment is the fixed dict of worker_env(); its job (the scenario, the controller's folder, the privilege) arrives
on stdin, an inherited pipe; it writes result.json into its working folder.
"""
# --- bootstrap A: find the folders of the task's packages above this file and put them first on sys.path ---
import os as _os
import sys as _sys


def _grip_roots(_here):
    _d = _os.path.dirname(_os.path.abspath(_here))
    while True:
        if _os.path.isdir(_os.path.join(_d, "contract")):
            if _os.path.isdir(_os.path.join(_d, "hidden")):
                return [_os.path.join(_d, "hidden"), _d]
            if _os.path.isdir(_os.path.join(_d, "grader")):
                return [_d]
        _p = _os.path.dirname(_d)
        if _p == _d:
            raise SystemExit("bootstrap: no task root above " + str(_here))
        _d = _p


for _q in _grip_roots(__file__):
    while _q in _sys.path:
        _sys.path.remove(_q)
    _sys.path.insert(0, _q)
del _grip_roots, _q
# --- end of bootstrap A ---
import importlib
import json
import os
import shutil
import stat
import subprocess
import sys
import threading
import time

from contract import constants as C
from contract import names as N

from grader import cmdparse as CP
from grader import metrics as M
from grader import obsbuild as OB
from grader import supervisor as SV

IF = C.INTERFACE
# the numerics environment of every grader process and every controller process (the BLAS pair, one BLAS thread
# and numpy's loops on one code path: the same numerics on every CPU type)
BLAS_ENV = {"OPENBLAS_CORETYPE": "Haswell", "OPENBLAS_L2_SIZE": "256", "OPENBLAS_NUM_THREADS": "1",
            "NPY_DISABLE_CPU_FEATURES": "X86_V4 AVX512_ICL AVX512_SPR"}
PRIVILEGED_CAP_S = 3600.0           # a privileged step: no late rule, no wall cap (this bound only stops a hang)
MAX_LINE = 1 << 20
END_STATUSES = ("cpu", "timeout", "dead", "protocol")
# what parsing a reply may raise besides the parser's own error: a nested value past Python's limits
REPLY_ERRORS = (RecursionError, MemoryError)


# ------------------------------------------------------------------------------------------------- endpoints
class LocalEndpoint(object):
    """An in-process controller object (development and tests). Requests and replies pass through JSON exactly as on
    the wire; CPU is the calling thread's; the caps are checked after the call (nothing can be stopped in-process)."""

    def __init__(self, ctl):
        self.ctl = ctl
        self.dead = False
        self.stats = {"local": True, "runner_processes_killed": 0, "stray_processes": 0, "no_network": False}

    def call(self, op, payload=None, cpu_s=1.0, wall_s=10.0):
        if self.dead:
            return "dead", None, 0.0, 0.0
        req = json.loads(json.dumps(payload or {}))
        t0, c0 = time.time(), time.thread_time()
        status = "ok"
        try:
            if op == "setup":
                extra = {k: v for k, v in req.items() if k not in ("record", "cell")}
                rep = {"ok": True, "reply": self.ctl.setup(req["record"], req["cell"], **extra)}
            elif op == "step":
                rep = {"ok": True, "cmd": self.ctl.step(req["obs"])}
            else:
                rep = {"ok": False, "error": "unknown op"}
        except Exception as e:  # noqa: BLE001
            rep, status = {"ok": False, "error": "{}: {}".format(type(e).__name__, e)}, "error"
        cpu, wall = time.thread_time() - c0, time.time() - t0
        try:
            line = json.dumps(rep, separators=(",", ":"))
        except (TypeError, ValueError) as e:
            status = "error"
            line = json.dumps({"ok": False, "error": "reply: {}".format(e)})
        if len(line.encode("utf-8")) > MAX_LINE:
            self.dead = True
            return "protocol", None, cpu, wall
        rep = json.loads(line)
        if status == "ok" and not rep.get("ok"):
            status = "error"
        if cpu > float(cpu_s):
            status = "cpu"
        elif wall > float(wall_s):
            status = "timeout"
        if status in ("cpu", "timeout"):
            self.dead = True
        return status, rep, cpu, wall

    def close(self):
        return dict(self.stats)


class DeadEndpoint(object):
    """A controller that could not start: every call fails."""
    dead = True

    def __init__(self, why):
        self.stats = {"start_error": str(why)[:300], "runner_processes_killed": 0, "stray_processes": 0,
                      "no_network": False}

    def call(self, op, payload=None, cpu_s=1.0, wall_s=10.0):
        return "dead", None, 0.0, 0.0

    def close(self):
        return dict(self.stats)


# ------------------------------------------------------------------------------------------------- defaults
def default_session_class():
    from plant.session import Session
    return Session


def make_session(session_class, scenario):
    """The plant session of a unit (the scenario's ceilings go to a session class that takes them)."""
    import inspect
    try:
        params = inspect.signature(session_class).parameters
    except (TypeError, ValueError):
        params = {}
    if "ceilings" in params:
        return session_class(scenario["unit"], ceilings=scenario["ceilings"])
    return session_class(scenario["unit"])


def session_cell_parts(sess):
    """The nests (poses in H, rounded to 1e-6) and the taught grasp (deg) of the plant's own cell."""
    nests = {n: {"pos_m": [round(float(x), 6) for x in pq[0]], "quat_wxyz": [round(float(x), 6) for x in pq[1]]}
             for n, pq in sess.taught.nest_poses_h().items()}
    return {"nests": nests, "taught": sess.taught.preset()}


def default_make_script(scenario, beliefs=None):
    from world.truth_script import TruthScript
    return TruthScript(scenario, beliefs)


def dev_hooks(module_name):
    """A development module providing session_class(), cell_parts() and make_script(scenario, beliefs)."""
    mod = importlib.import_module(module_name)
    return {"session_class": mod.session_class(), "cell_parts": mod.cell_parts(), "make_script": mod.make_script}


def host_cpu_model():
    try:
        with open("/proc/cpuinfo") as fh:
            for line in fh:
                if line.startswith("model name"):
                    return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return "unknown"


# ------------------------------------------------------------------------------------------------- one unit
def plant_faults(sess):
    """The exception class(es) by which the session's plant reports a physics fault (its module's PlantError), or ()."""
    mod = sys.modules.get(type(sess).__module__)
    exc = getattr(mod, "PlantError", None)
    return (exc,) if isinstance(exc, type) and issubclass(exc, Exception) else ()


def run_unit(scenario, unit_index, endpoint, *, privileged, script=None, session_class=None, cell_parts=None,
             make_script=None, beliefs=None, log=None):
    """Run one unit's two shifts with the controller behind `endpoint`. Returns (job_records, unit_stats)."""
    log = log or (lambda m: None)
    session_class = session_class or default_session_class()
    sess = make_session(session_class, scenario)
    parts = cell_parts if cell_parts is not None else session_cell_parts(sess)
    cell = SV.cell_payload(scenario["ceilings"], parts)
    sup = SV.Supervisor(scenario, unit_index, sess, up_h=SV.up_of(parts["nests"]["pick"]), log=log)
    load0 = os.getloadavg()
    cpu0 = time.process_time()
    stats = {"unit": int(unit_index), "serial": scenario["record"]["serial"], "privileged": bool(privileged),
             "setup_ok": False, "setup_error": None, "period_ms": None, "reads": None, "cycles": 0, "late_cycles": 0,
             "on_time_fraction": 1.0, "step_errors": 0, "refused_writes": 0, "controller_ended": None,
             "controller_ended_ms": None, "reply_wall_s": 0.0, "step_cpu_max_s": 0.0, "would_be_late_cycles": 0,
             "truth_payloads": 0, "load_start": [round(x, 2) for x in load0]}

    def end(why):
        if stats["controller_ended"] is None:
            stats["controller_ended"] = why
            stats["controller_ended_ms"] = int(sess.t_ms)
            sup.mark_persistent("controller_ended")
            log("  unit %d: the controller ended (%s) at %d ms" % (unit_index, why, sess.t_ms))

    # ---- setup (before the hand-over)
    payload = {"record": scenario["record"], "cell": cell}
    if privileged:
        if script is None:
            script = (make_script or default_make_script)(scenario, beliefs)
        payload["truth"] = scenario["unit"]
        payload["truth_reply"] = script.setup()
        stats["truth_payloads"] += 1
    cap_cpu = PRIVILEGED_CAP_S if privileged else IF["setup_cpu_s"]
    cap_wall = PRIVILEGED_CAP_S if privileged else IF["setup_wall_s"]
    st, rep, _cpu, _wall = endpoint.call("setup", payload, cap_cpu, cap_wall)
    reads, T = [], IF["period_ms"][1]
    if st == "ok":
        try:
            sr = CP.parse_setup(rep.get("reply"))
            reads = sr["reads"]
            T = C.period_ms(sr["period_ms"], reads)
            stats.update(setup_ok=True, period_ms=T, reads=reads)
        except (CP.SetupError,) + REPLY_ERRORS as e:
            stats["setup_error"] = str(e)[:300] if isinstance(e, CP.SetupError) else "setup reply: " + type(e).__name__
    else:
        stats["setup_error"] = "setup %s%s" % (st, (": " + CP.shown(rep.get("error"))) if rep else "")
    sup.start()
    if not stats["setup_ok"]:
        end("setup")
    alive = stats["setup_ok"]

    def advance(n):
        for _ in range(int(n)):
            if sup.done:
                return
            sess.advance(1)
            sup.after_tick()

    # ---- the cycles
    Ts = T / 1000.0
    k = 0
    pending = None
    last_seq = -1
    late = 0
    step_cpu = PRIVILEGED_CAP_S if privileged else IF["step_cpu_end_s"]
    step_wall = PRIVILEGED_CAP_S if privileged else IF["step_wall_end_s"]
    acted = False
    faults = plant_faults(sess)
    try:
        while not sup.done:
            refused = []
            if pending is not None:
                acted = True                       # a reply of the controller reaches the plant
                if pending["write"]:
                    refused = sess.write(pending["write"]) or []
                    stats["refused_writes"] += len(refused)
                if pending["tactile_zero"]:
                    sess.tactile_zero(list(pending["tactile_zero"]))
                sup.on_reply(pending["grasped"], pending["decline"])
                pending = None
            if not alive:
                advance(T)
                continue
            advance(2)
            if sup.done:
                break
            regs = OB.regs_block(sess.read(reads), reads)
            fr = sess.tactile_newest(last_seq)
            if fr is not None:
                last_seq = int(fr[0])
            obs = OB.build(cycle=k, t_ms=sess.t_ms, period_ms=T, late=late, regs=regs, tactile=OB.tactile_block(fr),
                           cell=sup.cell_obs(), refused=refused, event=sup.obs_event())
            if privileged:
                obs["truth_cmd"] = script.command(json.loads(json.dumps(obs)), sess, sup.view())
            st, rep, cpu, wall = endpoint.call("step", {"obs": obs}, step_cpu, step_wall)
            stats["reply_wall_s"] += wall
            stats["step_cpu_max_s"] = max(stats["step_cpu_max_s"], cpu)
            L = int(cpu // Ts) if cpu > Ts else 0
            if privileged:
                stats["would_be_late_cycles"] += L
                L = 0
            stats["cycles"] += 1 + L
            if L:
                stats["late_cycles"] += 1 + L
                sup.mark_current("late_cycles")
            if st == "ok":
                try:
                    pending = CP.parse_step(rep.get("cmd"))
                except (CP.ProtocolError,) + REPLY_ERRORS as e:
                    stats["protocol_error"] = str(e)[:300] if isinstance(e, CP.ProtocolError) else type(e).__name__
                    end("protocol")
            elif st == "error":
                stats["step_errors"] += 1
            else:
                end(st if st in END_STATUSES else "dead")
            if not privileged and stats["reply_wall_s"] > IF["unit_reply_wall_s"]:
                end("reply_wall")
            if stats["controller_ended"] is not None:
                alive = False
                pending = None
            late = L
            k += 1 + L
            advance(T - 2 + L * T)
    except faults as e:
        # the physics went where it cannot come back from. Classified by a control replay: the control is
        # the privileged reference on this very unit, which every unit of the frozen set passed at selection under the
        # code digest that verify checks (a stale set is refused; the plant is deterministic). So a fault in the
        # privileged run, or before any reply of the controller reached the plant, is the control failing: the
        # task's failure (it propagates). After the controller's replies it is the controller's: the controller is
        # ended and the unit is over (the job in hand is lost, every job not yet run is voided); never a task error.
        if privileged or not acted:
            raise
        stats["plant_fault"] = "%s at %d ms" % (type(e).__name__, int(sess.t_ms))
        end("plant_fault")
        sup.abort()
    recs = sorted(sup.records, key=lambda r: (r["shift"], r["job"]))
    for r in recs:
        M.finalize_job(r)
    cyc = max(1, stats["cycles"])
    stats["on_time_fraction"] = 1.0 - stats["late_cycles"] / cyc
    stats["reply_wall_s"] = round(stats["reply_wall_s"], 6)
    stats["process_cpu_s"] = round(time.process_time() - cpu0, 3)
    stats["load_end"] = [round(x, 2) for x in os.getloadavg()]
    stats["sim_ms"] = int(sess.t_ms)
    stats["trace_sha256"] = sess.trace_digest()
    stats["events"] = [dict(e) for e in sup.unit_events]
    return recs, stats


# ------------------------------------------------------------------------------------------------- workers
def worker_env(home):
    """The fixed environment of a unit worker."""
    env = {"PATH": os.environ.get("PATH", "/opt/venv/bin:/usr/local/bin:/usr/bin:/bin"), "HOME": home,
           "LANG": "C.UTF-8", "MUJOCO_GL": os.environ.get("MUJOCO_GL", "osmesa")}
    env.update(BLAS_ENV)
    return env


def _endpoint_for(job):
    """(endpoint, runner): the controller on the wire as the runner account, from the read-only program copy."""
    from grader.runtime import wire as W
    r = W.Runner(job["program_dir"], job["controller_root"], job["client"], user=job.get("account", "nobody"),
                 stderr_path=job.get("stderr_log"), extra_env=dict(BLAS_ENV),
                 cpu_cap_s=C.INTERFACE["process_cpu_s"],
                 stderr_cap_bytes=C.INTERFACE["stderr_log_max_mb"] << 20, end_wait_s=C.INTERFACE["end_wait_s"])
    ok, info = r.start()            # a RuntimeError here is the task's failure: it propagates, the worker exits 1
    if not ok:
        r.close()
        return DeadEndpoint(info), r
    return r, r


def worker_main(argv):
    if argv[:1] != ["--unit"] or len(argv) != 2 or argv[1] not in ("1", "2"):
        print("usage: acceptance.py --unit 1|2 (the job on stdin)", file=sys.stderr)
        return 2
    unit = int(argv[1])
    job = json.loads(sys.stdin.read())
    hooks = dev_hooks(job["dev"]) if job.get("dev") else {}
    ep, runner = _endpoint_for(job)
    t0 = time.time()
    recs, stats = run_unit(job["scenario"], unit, ep, privileged=bool(job.get("privileged")), beliefs=job.get("beliefs"),
                           session_class=hooks.get("session_class"), cell_parts=hooks.get("cell_parts"),
                           make_script=hooks.get("make_script"),
                           log=lambda m: print(m, file=sys.stderr, flush=True))
    stats["runner"] = runner.close() if runner is not None else ep.close()
    if isinstance(ep, DeadEndpoint):
        stats["runner"].update(ep.close())
    stats["unit_wall_s"] = round(time.time() - t0, 3)
    stats["worker_pid"] = os.getpid()
    out = os.path.join(os.getcwd(), "result.json")
    fd = os.open(out, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as fh:
        json.dump({"records": recs, "stats": stats}, fh)
    return 0


# the places of the image the runner account can write besides its own run folder: the clean-up after every unit
# removes what the account owns there. The floor is /tmp, /var/tmp and /dev/shm; /run/lock and /dev/mqueue are the base
# system's other world-writable places; /home/user (the agent's home) as in the merged hand assembly task's swept set
# on the same base image. The image's own check (the Dockerfile's last RUN, its SWEPT = this list, bound by a test)
# fails the build when the account can write a directory anywhere else.
RUNNER_WRITABLE_DIRS = ("/tmp", "/var/tmp", "/dev/shm", "/run/lock", "/dev/mqueue", "/home/user")
SYSVIPC_ROOT = "/proc/sysvipc"
IPC_KINDS = (("shm", "shmid", "-m"), ("msg", "msqid", "-q"), ("sem", "semid", "-s"))


def _lstat(path):
    return os.lstat(path)


def remove_owned(root, uid):
    """Remove every entry under `root` owned by `uid` (a folder of that owner with all it holds; a link itself, never
    what it points to); folders of other owners are walked, not removed. Returns the number of entries removed."""
    removed = 0
    try:
        st = _lstat(root)
    except OSError:
        return 0
    if not stat.S_ISDIR(st.st_mode):
        return 0
    todo = [root]
    while todo:
        d = todo.pop()
        try:
            names = sorted(os.listdir(d))
        except OSError:
            continue
        for name in names:
            p = os.path.join(d, name)
            try:
                st = _lstat(p)
            except OSError:
                continue
            if st.st_uid == uid:
                try:
                    if stat.S_ISDIR(st.st_mode):
                        shutil.rmtree(p)
                    else:
                        os.unlink(p)
                    removed += 1
                except OSError:
                    pass
            elif stat.S_ISDIR(st.st_mode):
                todo.append(p)
    return removed


def ipc_objects(uid, root=SYSVIPC_ROOT):
    """[(kind, id)] of the System V IPC objects (shared memory, message queues, semaphores) owned by `uid`."""
    out = []
    for kind, id_col, _flag in IPC_KINDS:
        try:
            with open(os.path.join(root, kind)) as fh:
                rows = [ln.split() for ln in fh if ln.strip()]
        except OSError:
            continue
        if not rows:
            continue
        head = rows[0]
        if id_col not in head or "uid" not in head:
            continue
        i, j = head.index(id_col), head.index("uid")
        for r in rows[1:]:
            try:
                if int(r[j]) == int(uid):
                    out.append((kind, int(r[i])))
            except (IndexError, ValueError):
                continue
    return out


def _run_quiet(argv):
    try:
        return subprocess.run(argv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10).returncode
    except (OSError, subprocess.TimeoutExpired):
        return -1


def remove_ipc(uid, root=SYSVIPC_ROOT, run=_run_quiet, only=None):
    """Remove the System V IPC objects of `uid` (ipcrm); `only` limits it to the given [(kind, id)]. Returns the number
    removed."""
    ipcrm = shutil.which("ipcrm") or "/usr/bin/ipcrm"
    flag = {k: f for k, _c, f in IPC_KINDS}
    n = 0
    for kind, oid in ipc_objects(uid, root):
        if only is not None and (kind, oid) not in set(only):
            continue
        if run([ipcrm, flag[kind], str(oid)]) == 0:
            n += 1
    return n


class UnitIsolation(object):
    """What lies between two units, which run one after the other under the one runner account: before a unit no
    process of the account is alive (any found is killed); after it every process of the account is killed, the unit's
    run folder is removed, and everything the account owns under the writable places and its System V IPC objects are
    removed, so that nothing one unit's controller wrote can be read by the next one's (and no process it started
    lives on). The account-wide parts need root (the account's uid); off root (development runs) the controller runs as
    the caller, its session is ended by the runner, the run folder is still removed, and the sweep runs only with an
    explicit `sweep_uid` (tests, on folders of their own)."""

    def __init__(self, account="nobody", writable_dirs=RUNNER_WRITABLE_DIRS, sweep_uid=None):
        from grader.runtime import runner as RUN
        acct = RUN.runner_account(account) if os.getuid() == 0 else None
        self.uid = acct[1] if acct else None
        self.sweep_uid = self.uid if sweep_uid is None else int(sweep_uid)
        self.dirs = tuple(writable_dirs)

    def _kill(self):
        from grader.runtime import lockdown as LD
        if self.uid is None:
            return 0, 0, 0
        alive = len(LD.live_uid_processes(self.uid))
        if not alive:
            return 0, 0, 0
        killed, left = LD.stop_and_kill_uid(self.uid, grace_s=5.0)
        return alive, int(killed), int(left)

    def before(self, unit):
        alive, killed, left = self._kill()
        return {"alive": alive, "killed": killed, "left": left}

    def after(self, unit, job):
        _alive, killed, left = self._kill()
        root = job.get("controller_root")
        if root and os.path.lexists(root):
            if os.path.isdir(root) and not os.path.islink(root):
                shutil.rmtree(root, ignore_errors=True)
            else:
                try:
                    os.unlink(root)
                except OSError:
                    pass
        files = 0
        if self.sweep_uid is not None:
            for d in self.dirs:
                files += remove_owned(d, self.sweep_uid)
        ipc = remove_ipc(self.uid) if self.uid is not None else 0
        return {"killed": killed, "left": left, "run_folder_removed": not os.path.lexists(root or ""),
                "files_removed": files, "ipc_removed": ipc}


def run_workers(jobs, runs_root, python=None, log=None, on_start=None, isolation=None):
    """jobs: {unit: job dict}. The units run ONE AFTER THE OTHER, in unit order: a unit's worker (its plant and, through
    the runner, its controller) starts only after the previous unit's worker has ended and the clean-up after it
    (isolation.after) is done; before each unit isolation.before ends any process of the runner account still alive.
    Returns {unit: (records, stats)}; stats["isolation"] holds the counts of the clean-up before and after the unit, its
    wall span and whether it started after the previous unit had ended. A process of the runner account that cannot be
    ended before the first unit, or a worker that fails, is a task error (RuntimeError); processes that the clean-up
    after a unit finds or cannot end count as that unit's processes left behind (its controller's)."""
    log = log or (lambda m: None)
    python = python or sys.executable
    entry = os.path.abspath(__file__)
    out = {}
    prev_end = None
    for u, job in sorted(jobs.items()):
        pre = isolation.before(u) if isolation is not None else {}
        if pre.get("left") and prev_end is None:
            raise RuntimeError("%d process(es) of the runner account are alive before the first unit and cannot be "
                               "ended" % pre["left"])
        folder = os.path.join(runs_root, "unit%d" % int(u))
        os.makedirs(folder, exist_ok=True)
        os.chmod(folder, 0o700)
        t0 = time.time()
        err = open(os.path.join(folder, "worker.log"), "wb")
        p = subprocess.Popen([python, "-I", "-B", entry, "--unit", str(int(u))], cwd=folder, env=worker_env(folder),
                             stdin=subprocess.PIPE, stdout=err, stderr=err)
        err.close()
        data = json.dumps(job).encode("utf-8")

        def feed(p=p, data=data):
            try:
                p.stdin.write(data)
                p.stdin.close()
            except (OSError, ValueError):
                pass
        threading.Thread(target=feed, daemon=True).start()
        if on_start:
            on_start(u, p)
        rc = p.wait()
        post = isolation.after(u, job) if isolation is not None else {}
        t1 = time.time()
        if rc != 0:
            with open(os.path.join(folder, "worker.log"), "rb") as fh:
                tail = fh.read()[-2000:].decode("utf-8", "replace")
            raise RuntimeError("unit worker %d failed (exit %s): %s" % (u, rc, tail))
        with open(os.path.join(folder, "result.json")) as fh:
            res = json.load(fh)
        st = res["stats"]
        st["isolation"] = {"before": pre, "after": post, "wall_s": round(t1 - t0, 3),
                           "started_after_previous": prev_end is None or t0 >= prev_end}
        if post.get("killed") or post.get("left"):
            st["runner"]["stray_processes"] = int(st["runner"].get("stray_processes", 0)) + int(post.get("killed", 0)) \
                + int(post.get("left", 0))
        out[u] = (res["records"], st)
        log("[grader] unit %d: %d jobs, %s; clean-up %s" % (u, len(res["records"]), st.get("controller_ended"), post))
        prev_end = t1
    return out


if __name__ == "__main__":
    sys.exit(worker_main(sys.argv[1:]))
