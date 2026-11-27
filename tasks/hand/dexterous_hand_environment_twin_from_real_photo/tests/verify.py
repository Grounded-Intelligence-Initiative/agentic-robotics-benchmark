"""Grader of dexterous_hand_environment_twin_from_real_photo. Root, verify stage, in a SEPARATE verifier sandbox
(task.toml): a fresh sandbox from the task image into which the engine restored only /home/user/submission; the
agent's sandbox is gone and setup.sh has not run here.

  0. FIRST THING, stdlib only (`_early_lockdown`; test.sh has already chmod 0700 the stage and its parent): record when
     the engine began uploading this folder (earliest ctime of its files), kill every process of the agent account (a
     survivor that cannot be frozen voids the run: task error) and keep the account SIGSTOPped until the verdict is
     written (arm/verify_freeze.py: the image's supervisord respawns its desktop as that account), lock the stage (0700 /
     0600, restored at the end), and take a root-owned 0700 snapshot of the submission that every later read uses. Gate
     G1 (submission integrity) is 0 when an entry of the submission was written at or after the upload began, when the
     submission root is a symbolic link, not a directory, resolves outside the agent's home or was itself replaced
     after the session, or when the folder holds more than 5000 entries or 300 MB (then it is not copied at all).
  1. The worker: a separate process (hand_twin/grade.worker_main) as the unprivileged account `nobody` where it exists,
     under an address-space limit and a hard wall-clock timeout planned from ALE_VERIFY_DEADLINE_SECONDS, on private
     copies of the grading code, the public payload modules, the truth and the submission. It self-tests the truth
     (hand_twin/selftest.py: the files against their manifest, the constants against this configuration, the stored
     reference's outcome at the centre placement replayed for every scored request, one lift per lift unit), then
     grades the submission: the gate and the settle (G0), the registration onto the true support frame, every scored
     request planned on the registered twin and executed in the truth at the nine placements, the lifts, the use. Each
     stage runs under its own alarm (hand_twin/grade.py says what a failing or late stage costs).
  2. twin_score = G1 x G0 x use (use = 0 when the twin does not register); the verdict goes through the benchmark kit
     (robotics_grader.ratio_rewards against anchor.json: value 1.0 = the reference twin, full_at 1.0, so reward =
     twin_score). Metrics: only the names of grader_config.json's metrics_whitelist (and the kit's own), finite, and
     nothing that depends on the clock, so a rerun of the same submission writes a byte-identical verdict. With
     ARB_HAND_TWIN_GRADE_PATH set, the whole grade (hand_twin/dump.py) is written there as well (maintainers).
No agent code runs here (MJCF, meshes and JSON are data). The grader renders nothing (CPU only).
Knobs: grader_config.json; the public rules: setup_payload/ (the agent's copies of the payload modules and files).
"""
import json
import math
import os
import pwd
import shutil
import signal
import stat as statmod
import subprocess
import sys
import tempfile
import time

sys.dont_write_bytecode = True
STAGE_DIR = os.environ.get("ALE_STAGE_DIR", os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, STAGE_DIR)
HOME = os.environ.get("ALE_HOME", "/home/user")
AGENT_USER = os.path.basename(HOME.rstrip("/")) or "user"
SUBMISSION = os.path.join(HOME, "submission")
SNAP_ROOT = os.environ.get("ARB_SNAPSHOT_ROOT", "/opt/arb-verify")
SNAPSHOT = os.path.join(SNAP_ROOT, "submission")
LATE_GATE_MAX_GAP_S = 600.0
SUBMISSION_BYTES_CAP = 300 * 1024 * 1024        # = grader_config.json caps.max_bytes (checked)
SUBMISSION_ENTRIES_CAP = 5000                   # = grader_config.json caps.max_entries (checked)
PROGRESS_MAX_BYTES = 64 * 1024 * 1024
KIT_METRICS = ("ratio", "anchor_recorded")
# the worker's floating-point code paths: numpy's and OpenBLAS's CPU-dependent paths pinned to x86-64-v3 (AVX2 + FMA,
# no AVX-512), so that verifiers of that level compute the same bits; OPENBLAS_L2_SIZE keeps the kernel's own blocks
FP_PINNING = {"NPY_DISABLE_CPU_FEATURES": "X86_V4 AVX512_ICL AVX512_SPR", "OPENBLAS_CORETYPE": "Haswell",
              "OPENBLAS_L2_SIZE": "256"}

from arm.assembly_six import lockdown  # noqa: E402  (stdlib only; vendored)
from arm.assembly_cell import runner  # noqa: E402  (stdlib only; vendored)
from arm import verify_freeze  # noqa: E402  (stdlib only; vendored)

# started right after the verify-start kill, stopped after the verdict (a no-op unless root and the account exists)
FREEZER = verify_freeze.AgentFreezer.for_user(AGENT_USER)


def log(msg):
    print("[grader] " + str(msg), file=sys.stderr, flush=True)


def _submission_root_ok():
    """(ok, reason): the submission root must be a real directory inside the agent's home, never a symbolic link."""
    try:
        st = os.lstat(SUBMISSION)
    except FileNotFoundError:
        return True, None
    except OSError as exc:
        return False, "the submission root %s cannot be inspected (%s)" % (SUBMISSION, exc)
    if statmod.S_ISLNK(st.st_mode):
        return False, "the submission root %s is a symbolic link: not read" % SUBMISSION
    if not statmod.S_ISDIR(st.st_mode):
        return False, "the submission root %s is not a directory" % SUBMISSION
    home = os.path.realpath(HOME)
    real = os.path.realpath(SUBMISSION)
    if os.path.commonpath([home, real]) != home:
        return False, "the submission root %s resolves outside %s: not read" % (SUBMISSION, HOME)
    return True, None


def _submission_measure(root, max_entries=SUBMISSION_ENTRIES_CAP):
    """(entries, apparent bytes, allocated bytes) below `root` by lstat, never following a link; stops counting once
    max_entries is exceeded."""
    n, apparent, allocated = 0, 0, 0
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        for name in dirnames + filenames:
            n += 1
            if n > max_entries:
                return n, apparent, allocated
            try:
                st = os.lstat(os.path.join(dirpath, name))
            except OSError:
                continue
            if statmod.S_ISREG(st.st_mode):
                apparent += int(st.st_size)
                allocated += int(getattr(st, "st_blocks", 0)) * 512
    return n, apparent, allocated


def _proc_state(pid):
    """The /proc state letter of `pid` ("X" once it is gone)."""
    try:
        with open("/proc/%d/status" % pid) as fh:
            for line in fh:
                if line.startswith("State:"):
                    return line.split()[1]
    except (OSError, IndexError):
        pass
    return "X"


def _still_running(uid, bound_s=3.0):
    """SIGSTOP every live process of `uid` until a pass finds none still running (a respawn that appears meanwhile is
    stopped by the next pass) or `bound_s` elapsed; returns the PIDs still running (empty: every one is stopped)."""
    deadline = time.time() + float(bound_s)
    while True:
        for pid in lockdown.live_uid_processes(uid):
            try:
                os.kill(pid, signal.SIGSTOP)
            except OSError:
                pass
        time.sleep(0.05)
        running = [pid for pid in lockdown.live_uid_processes(uid) if _proc_state(pid) not in ("T", "t", "X")]
        if not running or time.time() > deadline:
            return running


def _early_lockdown():
    out = {"agent_processes_killed": 0.0, "agent_processes_left": 0.0, "agent_processes_running": 0.0,
           "stage_locked": 0.0, "stage_parent_locked": 0.0,
           "submission_late_writes": 0.0, "late_write_gate_active": 0.0, "submission_snapshot": 0.0,
           "submission_root_rejected": 0.0, "submission_root_violation": 0.0, "submission_oversize": 0.0,
           "saved": None, "parent": None, "reasons": [], "t_ref": None}
    root_ok, why = _submission_root_ok()
    if not root_ok:
        out["submission_root_rejected"] = 1.0
        out["reasons"].append(why)
    if os.getuid() != 0:
        return out
    t_ref = lockdown.stage_upload_time(STAGE_DIR)
    out["t_ref"] = t_ref
    try:
        uid = pwd.getpwnam(AGENT_USER).pw_uid
    except KeyError:
        uid = None
    if uid is not None and uid != 0:
        killed, left = lockdown.stop_and_kill_uid(uid, grace_s=8.0)
        out["agent_processes_killed"], out["agent_processes_left"] = float(killed), float(left)
        if left:
            # The image's supervisord respawns the account's desktop as fast as it is killed, so in the separate
            # verifier a process that outlives the kill is the image's own. It is frozen (a stopped process cannot
            # exit, so it is not respawned); only one still running once frozen voids the run.
            out["agent_processes_running"] = float(len(_still_running(uid)))
            log("%d process(es) of the agent account outlived the kill at verify start, %d still running after the "
                "freeze" % (left, out["agent_processes_running"]))
        try:
            FREEZER.start()               # whatever supervisord respawns from here on stays stopped until the verdict
        except Exception:  # noqa: BLE001 - the freezer never takes the verdict down
            pass
    out["saved"] = runner.lock_stage(STAGE_DIR)
    out["stage_locked"] = 1.0 if out["saved"] is not None else 0.0
    parent = os.path.dirname(os.path.abspath(STAGE_DIR))
    if parent and parent not in ("/", os.path.abspath(HOME)) and os.path.isdir(parent):
        try:
            mode = statmod.S_IMODE(os.lstat(parent).st_mode)
            os.chmod(parent, 0o700)
            saved_mode = os.environ.get("ARB_STAGE_PARENT_MODE")
            out["parent"] = (parent, int(saved_mode, 8) if saved_mode else mode)
            out["stage_parent_locked"] = 1.0
        except (OSError, ValueError):
            pass
    gate_active = t_ref is not None
    cfg_json = os.path.join(STAGE_DIR, "grader_config.json")
    if gate_active and os.path.isfile(cfg_json):
        try:
            gate_active = (os.lstat(cfg_json).st_ctime - t_ref) <= LATE_GATE_MAX_GAP_S
        except OSError:
            pass
    out["late_write_gate_active"] = 1.0 if gate_active else 0.0
    if root_ok and os.path.isdir(SUBMISSION):
        if gate_active:
            try:
                rst = os.lstat(SUBMISSION)
                if max(rst.st_ctime, rst.st_mtime) >= t_ref:
                    out["submission_root_violation"] = 1.0
                    out["reasons"].append("the submission directory itself was replaced or changed after the agent "
                                          "phase ended")
            except OSError:
                pass
        n, apparent, allocated = _submission_measure(SUBMISSION)
        if n > SUBMISSION_ENTRIES_CAP or max(apparent, allocated) > SUBMISSION_BYTES_CAP:
            out["submission_oversize"] = 1.0
            out["reasons"].append("the submission holds %s entries / %d bytes (caps %d entries, %d bytes): not "
                                  "copied" % (n if n <= SUBMISSION_ENTRIES_CAP else "> %d" % SUBMISSION_ENTRIES_CAP,
                                              max(apparent, allocated), SUBMISSION_ENTRIES_CAP, SUBMISSION_BYTES_CAP))
            return out
        if gate_active:
            late = lockdown.late_entries(SUBMISSION, t_ref)
            out["submission_late_writes"] = float(len(late))
            if late:
                out["reasons"].append("%d entr%s of the submission written after the agent phase ended (%s)" % (
                    len(late), "y" if len(late) == 1 else "ies", ", ".join(e[0] for e in late[:5])))
        os.makedirs(SNAP_ROOT, exist_ok=True)
        os.chmod(SNAP_ROOT, 0o700)
        lockdown.snapshot_dir(SUBMISSION, SNAPSHOT)
        out["submission_snapshot"] = 1.0
    return out


def _restore(early):
    try:
        if early.get("saved") is not None:
            runner.unlock_stage(early["saved"])
        if early.get("parent"):
            os.chmod(early["parent"][0], early["parent"][1])
        mode = os.environ.get("ARB_STAGE_MODE")
        if mode:
            os.chmod(STAGE_DIR, int(mode, 8))
    except OSError:
        pass


def _g1(early):
    return 0.0 if (early["submission_late_writes"] > 0 or early["submission_root_rejected"] > 0
                   or early["submission_root_violation"] > 0 or early["submission_oversize"] > 0) else 1.0


def _integrity(early):
    """The lockdown's flags and counts (no clock value): logged and kept in the grade dump, never in the metrics."""
    return {k: v for k, v in early.items() if isinstance(v, float) and k != "t_ref"}


# ---------------------------------------------------------------------------------------------------------------
# the worker

def _budget(cfg, t0):
    """(seconds the worker may run, where the number came from, the verify budget): the verify deadline the engine
    passes, less the time already spent and a reserve for the verdict; the configured default outside the engine."""
    rt = cfg["runtime"]
    raw = os.environ.get("ALE_VERIFY_DEADLINE_SECONDS")
    try:
        total, src = float(raw), "ALE_VERIFY_DEADLINE_SECONDS"
    except (TypeError, ValueError):
        total, src = float(rt["default_budget_s"]), "runtime.default_budget_s"
    left = total - (time.time() - t0) - float(rt["reserve_s"])
    return max(float(rt["min_worker_s"]), min(left, float(rt["max_worker_s"]))), src, total


def _copy_file(src, dst):
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copyfile(src, dst)


def _prepare_work(cfg, sub_dir):
    """A private work folder: code/ (hand_twin/, the payload's modules and files in setup_payload/, the grader's
    configuration), gt/ (the truth files), submission/ (regular files only), out/, home/; returns (work, job)."""
    work = tempfile.mkdtemp(prefix="arb_hand_twin_verify_")
    code = os.path.join(work, "code")
    runner.copy_regular_files(os.path.join(STAGE_DIR, "hand_twin"), os.path.join(code, "hand_twin"))
    runner.copy_regular_files(os.path.join(STAGE_DIR, "setup_payload"), os.path.join(code, "setup_payload"))
    _copy_file(os.path.join(STAGE_DIR, "grader_config.json"), os.path.join(code, "grader_config.json"))
    gt = os.path.join(work, "gt")
    runner.copy_regular_files(os.path.join(STAGE_DIR, cfg["gt"]["dir"]), gt)
    job = {"format": "hand-twin-worker-job/1", "gt": gt, "submission": os.path.join(work, "submission")}
    if sub_dir is not None and os.path.isdir(sub_dir):
        runner.copy_regular_files(sub_dir, job["submission"])
    else:
        os.makedirs(job["submission"])
    for d in ("out", "home"):
        os.makedirs(os.path.join(work, d))
    job.update({"progress": os.path.join(work, "out", "progress.json"), "code": code,
                "grader_config": os.path.join(code, "grader_config.json")})
    return work, job


def _read_progress(path):
    """The worker's progress document, read without following links and within a size cap (None if absent or bad)."""
    try:
        fd = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
    except OSError:
        return None
    try:
        if not statmod.S_ISREG(os.fstat(fd).st_mode):
            return None
        chunks, total = [], 0
        while total <= PROGRESS_MAX_BYTES:
            b = os.read(fd, 1 << 20)
            if not b:
                break
            chunks.append(b)
            total += len(b)
        data = b"".join(chunks)
    finally:
        os.close(fd)
    if len(data) > PROGRESS_MAX_BYTES:
        return None
    try:
        doc = json.loads(data.decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        return None
    return doc if isinstance(doc, dict) else None


def run_worker(cfg, sub_dir, budget_s, stats):
    """Run the worker; returns (progress document or None, how it ended, the tail of its log)."""
    rt = cfg["runtime"]
    work, job = _prepare_work(cfg, sub_dir)
    try:
        job["deadline_wall"] = time.time() + budget_s - float(rt["worker_margin_s"])
        job["stage_caps_s"] = dict(rt["stage_caps_s"])
        job["address_space_mb"] = int(rt["worker_address_space_mb"])
        job_path = os.path.join(work, "job.json")
        with open(job_path, "w") as f:
            json.dump(job, f)
        acct = runner.runner_account(rt.get("worker_account", "nobody"), None)
        # no MUJOCO_GL: the worker renders nothing, so no GL library is loaded
        env = {"HOME": os.path.join(work, "home"), "PATH": "/usr/bin:/bin", "LANG": "C.UTF-8",
               "PYTHONDONTWRITEBYTECODE": "1", "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1",
               "MKL_NUM_THREADS": "1", "TMPDIR": os.path.join(work, "home")}
        env.update(FP_PINNING)
        boot = ("import sys; sys.path.insert(0, %r); from hand_twin import grade; sys.exit(grade.worker_main(%r))"
                % (job["code"], job_path))
        argv = [sys.executable, "-I", "-B", "-c", boot]
        stats["worker_deprivileged"] = 0.0
        if acct is not None:
            for dirpath, dirnames, filenames in os.walk(work):
                for n in [dirpath] + [os.path.join(dirpath, x) for x in dirnames + filenames]:
                    os.lchown(n, acct[1], acct[2])
            os.chmod(work, 0o700)
            from robotics_grader.stage import secure_exec_path
            secure_exec_path()
            ru = shutil.which("runuser", path="/usr/sbin:/usr/bin:/sbin:/bin") or "/usr/sbin/runuser"
            argv = [ru, "-p", "-u", acct[0], "--"] + argv
            stats["worker_deprivileged"] = 1.0
        t = time.time()
        log("worker: %s, budget %.0f s, address space %d MB" % ("as " + acct[0] if acct else "as this account",
                                                                 budget_s, job["address_space_mb"]))
        with open(os.path.join(work, "out", "worker.log"), "wb") as logf:
            if acct is not None:
                os.lchown(os.path.join(work, "out", "worker.log"), acct[1], acct[2])
            proc = subprocess.Popen(argv, cwd=os.path.join(work, "home"), env=env, stdout=logf, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            how = "exit"
            try:
                proc.wait(timeout=budget_s)
            except subprocess.TimeoutExpired:
                how = "timeout"
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except OSError:
                    pass
                proc.wait()
        stats["worker_s"] = time.time() - t
        stats["worker_exit"] = float(proc.returncode if proc.returncode is not None else -999)
        stats["worker_timed_out"] = 1.0 if how == "timeout" else 0.0
        doc = _read_progress(job["progress"])
        try:
            with open(os.path.join(work, "out", "worker.log"), "rb") as f:
                f.seek(0, 2)
                f.seek(max(0, f.tell() - 3000))
                tail = f.read().decode("utf-8", "replace")
        except OSError:
            tail = ""
        if how == "timeout":
            log("worker: killed at the hard timeout (%.0f s)" % budget_s)
            how = "killed at the hard timeout"          # no clock value: this text may reach the verdict
        elif proc.returncode != 0:
            how = "exit code %d" % proc.returncode
        else:
            how = "finished"
        return doc, how, tail
    finally:
        shutil.rmtree(work, ignore_errors=True)


# ---------------------------------------------------------------------------------------------------------------
# the verdict

def _unit(x):
    try:
        x = float(x)
    except (TypeError, ValueError):
        return 0.0
    return min(1.0, max(0.0, x)) if math.isfinite(x) else 0.0


def _whitelisted(metrics, whitelist):
    """`metrics` in name order, finite numbers only, names in the whitelist or the kit's own."""
    allowed = set(whitelist) | set(KIT_METRICS)
    out = {}
    for k, v in sorted(metrics.items()):
        if k not in allowed:
            continue
        try:
            x = float(v)
        except (TypeError, ValueError):
            continue
        if math.isfinite(x):
            out[k] = x
    return out


def _metric_values(g1, g0, sub):
    reg = 1.0 if _unit(sub.get("registered")) == 1.0 else 0.0
    use = _unit(sub.get("use")) if (g1 and g0 and reg) else 0.0
    on = 1.0 if (g1 and g0 and reg) else 0.0
    return {"g1_integrity": float(g1), "g0_validity": float(g0), "registered": reg if g1 else 0.0, "use": use,
            "done_share": on * _unit(sub.get("done_share")), "stop_share": on * max(0.0, _num(sub.get("stop_share"))),
            "declined_share": on * _unit(sub.get("declined_share"))}


def _num(x):
    try:
        x = float(x)
    except (TypeError, ValueError):
        return 0.0
    return x if math.isfinite(x) else 0.0


def write_verdict(overall, values, diagnostic, whitelist):
    """One gated `reward` (the kit's envelope) = twin_score; the whitelisted values go to `metrics`, the diagnostics
    into the verdict's `diagnostic` text. In the engine (ALE_VERIFY_PYTHON set) the kit writes
    /logs/verifier/reward.json and verification.json through ale_verify; a host-side run writes the plain envelope to
    ROBOTICS_GRADER_VERDICT_PATH, else verdict.local.json in the stage folder."""
    from robotics_grader import ratio_rewards, read_anchor
    from robotics_grader import write_verdict as kit_write
    anchor, meta = read_anchor()
    extra = dict(values)
    extra["twin_score"] = float(overall)
    extra = {k: v for k, v in extra.items() if k in set(whitelist)}
    verdict = ratio_rewards(overall, anchor, higher_is_better=meta.get("direction", "higher") == "higher",
                            metric_name=meta.get("metric", "twin_score"), extra=extra, full_at=meta.get("full_at", 1.0))
    verdict["metrics"] = _whitelisted(verdict["metrics"], whitelist)
    if os.environ.get("ALE_VERIFY_PYTHON"):
        kit_write({"rewards": verdict["rewards"], "metrics": verdict["metrics"]}, diagnostic=diagnostic[:2000])
    else:
        kit_write(verdict, path=os.environ.get("ROBOTICS_GRADER_VERDICT_PATH")
                  or os.path.join(STAGE_DIR, "verdict.local.json"), diagnostic=diagnostic)
    verdict["twin_score"] = float(overall)
    return verdict


def _diagnostic(g1, g0, sub, diag):
    parts = ["G1 %.0f%s" % (g1, (": " + diag["G1"]) if diag.get("G1") else ""),
             "G0 %.0f%s" % (g0, (": " + diag["G0"]) if diag.get("G0") else "")]
    reg = sub.get("registration") if isinstance(sub.get("registration"), dict) else {}
    if g1 and g0:
        parts.append("registration %s%s" % ("held" if sub.get("registered") else "failed",
                                            (": " + str(reg.get("reason"))) if reg.get("reason") else ""))
    for k in ("registration", "requests", "lifts"):
        if diag.get(k):
            parts.append("%s: %s" % (k, diag[k]))
    if g1 and g0 and sub.get("registered"):
        parts.append("use %.4f (done %.4f, stop %.4f, declined %.4f)" % (
            _unit(sub.get("use")), _unit(sub.get("done_share")), _num(sub.get("stop_share")),
            _unit(sub.get("declined_share"))))
    return "; ".join(str(p)[:400] for p in parts)


def _write_dump(sub, st, verdict, early):
    path = os.environ.get("ARB_HAND_TWIN_GRADE_PATH")
    if not path:
        return
    try:
        from hand_twin import dump
        d = dump.build(sub, st, verdict)
        d["integrity"] = _integrity(early)
        dump.write(path, d)
    except Exception as exc:  # noqa: BLE001 - the dump never changes the verdict
        log("the grade dump was not written: %s" % exc)


def _zero_verdict(reason, whitelist):
    values = {"g1_integrity": 0.0, "g0_validity": 0.0, "registered": 0.0, "use": 0.0, "done_share": 0.0,
              "stop_share": 0.0, "declined_share": 0.0}
    return write_verdict(0.0, values, "G1 0: " + reason[:1500], whitelist)


def main():
    t0 = time.time()
    fallback_whitelist = ("twin_score", "g1_integrity", "g0_validity", "use", "registered", "done_share",
                          "stop_share", "declined_share")
    try:
        early = _early_lockdown()
    except Exception:  # noqa: BLE001 - never a task error without a verdict
        import traceback
        log("the verify-start lockdown raised:\n" + traceback.format_exc())
        _zero_verdict("the verify-start lockdown failed: " + traceback.format_exc()[-800:], fallback_whitelist)
        FREEZER.stop()
        return
    stats = {}
    try:
        from robotics_grader import fail
        if early["agent_processes_running"] > 0:
            fail("%d agent process(es) survived the verify-start kill and could not be stopped: run void"
                 % early["agent_processes_running"])
        with open(os.path.join(STAGE_DIR, "grader_config.json")) as fh:
            cfg = json.load(fh)
        whitelist = list(cfg["metrics_whitelist"])
        caps = cfg.get("caps") or {}
        if int(caps.get("max_entries", -1)) != SUBMISSION_ENTRIES_CAP or \
                int(caps.get("max_bytes", -1)) != SUBMISSION_BYTES_CAP:
            fail("grader_config.json's caps %s differ from the lockdown's (%d entries, %d bytes)"
                 % (caps, SUBMISSION_ENTRIES_CAP, SUBMISSION_BYTES_CAP))
        log("integrity: " + json.dumps(_integrity(early), sort_keys=True))
        g1 = _g1(early)
        diag = {}
        if g1 < 1.0:
            diag["G1"] = "; ".join(early["reasons"])
            verdict = write_verdict(0.0, _metric_values(0.0, 0.0, {}), _diagnostic(0.0, 0.0, {}, diag), whitelist)
            log("G1 = 0: %s" % diag["G1"])
            _write_dump({}, None, verdict, early)
            return
        sub_dir = SNAPSHOT if early["submission_snapshot"] else (SUBMISSION if os.path.isdir(SUBMISSION) else None)
        budget, src, total = _budget(cfg, t0)
        log("time plan: %.0f s from %s; worker %.0f s" % (total, src, budget))
        doc, how, tail = run_worker(cfg, sub_dir, budget, stats)
        if doc is None or not isinstance(doc.get("selftest"), dict):
            fail("the grader's worker produced no self-test result (%s): %s" % (how, tail[-1200:]))
        st = doc["selftest"]
        if not st.get("ok"):
            fail("the truth's self-test failed (%s): %s %s" % (how, st.get("reason"), str(st.get("trace", ""))[-800:]))
        if doc.get("task_error"):
            fail("task error while grading: %s" % doc["task_error"].get("reason"))
        sub = doc.get("submission") if isinstance(doc.get("submission"), dict) else {}
        g0 = 1.0 if _unit(sub.get("G0")) == 1.0 else 0.0
        if not doc.get("done") and g0 == 1.0:
            # an outcome of the registration, the requests or the lifts must never come from the wall clock or from
            # a lost worker (a signal: a negative exit code, or 128 + its number when runuser started the worker)
            code = int(stats["worker_exit"])
            sig = -code if code < 0 else code - 128 if stats["worker_deprivileged"] and 128 < code < 160 else 0
            if sig > 0 and not stats["worker_timed_out"]:
                fail("the worker ended on signal %d before the grade finished (a crash, not a timeout): the episode "
                     "is void" % sig)
            fail("the worker was %s before the grade finished: the verifier ran out of time, the episode is void"
                 % how)
        diag.update({k: str(v) for k, v in (sub.get("diagnostics") or {}).items()})
        if g0 < 1.0 and not diag.get("G0"):
            diag["G0"] = "the gate did not complete (the worker %s)" % how
        values = _metric_values(g1, g0, sub)
        overall = g1 * g0 * values["use"]
        verdict = write_verdict(overall, values, _diagnostic(g1, g0, sub, diag), whitelist)
        _write_dump(sub, st, verdict, early)
        timing = {"verify_s": time.time() - t0, "worker_s": stats.get("worker_s")}
        timing.update({"wall_s_" + k: v for k, v in ((sub.get("timing") or {}).get("wall_s") or {}).items()})
        log("timing: " + json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in sorted(timing.items())}))
        log("G1 %.0f G0 %.0f registered %.0f use %.4f twin_score %.4f (worker %s)" % (
            g1, g0, values["registered"], values["use"], overall, how))
    finally:
        _restore(early)
        FREEZER.stop()                    # after the verdict; the stopped processes are never resumed


if __name__ == "__main__":
    main()
