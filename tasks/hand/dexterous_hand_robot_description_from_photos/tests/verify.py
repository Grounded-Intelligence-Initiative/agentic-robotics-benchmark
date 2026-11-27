"""Grader of dexterous_hand_robot_description_from_photos (the robot-description stage of the dexterous-hand line).
Root, verify stage, in a SEPARATE verifier sandbox (task.toml): a fresh sandbox from the task image into which the
engine restored only /home/user/submission; the agent's sandbox is gone, setup.sh has not run here.

  0. FIRST THING, stdlib only (`_early_lockdown`; test.sh has already chmod 0700 the stage and its parent): record when
     the engine began uploading this folder (earliest ctime of its files), freeze and kill every process of the agent
     account (a survivor that cannot be frozen voids the run: task error), keep every process of the account SIGSTOPped
     until the verdict is written (`verify_freeze.AgentFreezer`: the image's supervisord respawns its desktop services
     as the agent account within seconds of the kill; a stopped process cannot exit, so nothing is respawned), lock the
     stage (0700 / 0600, restored at the end), take a root-owned 0700 snapshot of the submission that every later read
     uses, and zero the format check model_loads (stat submission_final = 0) when an entry of the submission was
     written at or after the stage upload began. A submission root that is a symbolic link, or that resolves outside
     the agent's home, is not read (stat submission_root_rejected = 1); a tree above 200 MB or 20 000 entries is not
     copied (stat submission_oversize = 1); both grade as an empty folder. Whatever the submission does, a verdict is
     written.
  1. The task's own files are checked before anything is graded: every file of gt/ against gt/MANIFEST.sha256, the
     hidden set's code digests against the shipped grading modules and grader_config.json, the hidden set's counts
     against grader_config.json. A mismatch is a TASK ERROR (stderr "TASK ERROR: ...", no verdict, exit 2), never the
     agent's score.
  2. The photographed unit from gt/ (robot.xml + assets, unit_state.json) and the frozen hidden set (hidden_set.json:
     readings and requests, drawn once when the task was frozen).
  3. hand_desc.grade.grade: the format check model_loads (hand_desc.gate) x the items pinch 0.55, single 0.15,
     pose 0.10, travrej 0.10, force_load 0.10 (grader_config.json); description_score = the weighted mean. Once the
     gate's rules 1 to 10 have passed, grade.grade runs in a CHILD process (`grade_in_budget`: os.fork, so the child
     holds the loaded models) under the budget of rule 11: RLIMIT_CPU of gate.GRADE_CPU_S seconds (no core file; the
     child dies with this process) and a wall-clock watchdog of gate.GRADE_WALL_FACTOR x that for a child that
     sleeps. The child's result comes back pickled through a pipe, so a grading inside the budget gives the same
     verdict bytes as one in this process; a child stopped at either limit is rule 11's failure (model_loads 0, a
     verdict without any measured time in it); a child that raises or crashes is a grader exception (a verdict, 0).
  4. The benchmark kit's envelope (robotics_grader.ratio_rewards against anchor.json: value 1.0 = the reference
     description, full_at 1.0): one gated `reward` = description_score; the format check, the items and the
     statistics named in grader_config.json "metric_keys" go to `metrics` (a statistic outside that list is a grader
     fault: task error). No timing is recorded in the verdict (timings go to stderr).
In the separate verifier the only processes of the agent account are the base image's desktop session and nothing is
written late (the engine restores the submission before it uploads tests/): the kill, the lock, the snapshot and the
late-write gate stay as defence in depth. The grader reads only tests/ and the restored submission; no code of the
submission runs here (MJCF, meshes and YAML are data). Knobs: grader_config.json.
"""
import hashlib
import json
import math
import os
import pwd
import shutil
import signal
import stat as statmod
import sys
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
SUBMISSION_BYTES_CAP = 200 * 1024 * 1024          # = hand_desc.gate.MAX_SUBMISSION_BYTES
SUBMISSION_ENTRIES_CAP = 20000                    # = hand_desc.gate.MAX_SUBMISSION_ENTRIES
GT_DIR = os.path.join(STAGE_DIR, "gt")
CODE_FILES = {"kin.py": "hand_desc/kin.py", "calib.py": "hand_desc/calib.py", "hidden_unit.py": "hand_desc/hidden_unit.py",
              "ik.py": "hand_desc/ik.py", "collide.py": "hand_desc/collide.py", "score.py": "hand_desc/score.py",
              "grader_config.json": "grader_config.json"}
KIT_ENVELOPE_KEYS = ("ratio", "anchor_recorded")  # the kit's own metrics beside the metric name

import lockdown  # noqa: E402  (stdlib only; vendored)
import verify_freeze  # noqa: E402  (stdlib only; vendored)

# started right after the verify-start kill, stopped after the verdict (a no-op unless root and the account exists)
FREEZER = verify_freeze.AgentFreezer.for_user(AGENT_USER)


class TaskError(Exception):
    """A fault of the task's own files or of the grader: no verdict, exit 2."""


def log(msg):
    print("[grader] " + str(msg), file=sys.stderr, flush=True)


def lock_stage(root):
    """chmod 0700 every directory and 0600 every file under `root` (root only); the modes to restore, or None."""
    if os.getuid() != 0:
        return None
    saved = []
    for dirpath, _dirnames, filenames in os.walk(root):
        for name in [""] + filenames:
            p = os.path.join(dirpath, name) if name else dirpath
            try:
                mode = statmod.S_IMODE(os.lstat(p).st_mode)
            except OSError:
                continue
            saved.append((p, mode))
            try:
                os.chmod(p, 0o700 if not name else 0o600)
            except OSError:
                pass
    return saved


def unlock_stage(saved):
    for p, mode in saved or []:
        try:
            os.chmod(p, mode)
        except OSError:
            pass


def _submission_root_ok():
    """(ok, reason): the submission root must be a real directory inside the agent's home, never a symbolic link."""
    try:
        st = os.lstat(SUBMISSION)
    except FileNotFoundError:
        return True, None
    except OSError as exc:
        return False, "the submission root %s cannot be inspected (%s)" % (SUBMISSION, exc)
    if statmod.S_ISLNK(st.st_mode):
        try:
            target = os.readlink(SUBMISSION)
        except OSError:
            target = "?"
        return False, "the submission root %s is a symbolic link (to %s): not read" % (SUBMISSION, target)
    home = os.path.realpath(HOME)
    real = os.path.realpath(SUBMISSION)
    if os.path.commonpath([home, real]) != home:
        return False, "the submission root %s resolves outside %s (%s): not read" % (SUBMISSION, HOME, real)
    return True, None


def _submission_measure(root, max_entries=SUBMISSION_ENTRIES_CAP):
    """(entries, apparent bytes, allocated bytes) of the tree below `root` by lstat, never following a link; stops
    counting once max_entries is exceeded."""
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


def _copy_regular_files(src, dst):
    """Copy the tree `src` to `dst` keeping regular files and directories only (symbolic links, special files
    and __pycache__ skipped). Returns the number of skipped entries."""
    skipped = 0
    os.makedirs(dst, exist_ok=True)
    for dirpath, dirnames, filenames in os.walk(src, followlinks=False):
        rel = os.path.relpath(dirpath, src)
        target = dst if rel == "." else os.path.join(dst, rel)
        keep = []
        for d in dirnames:
            p = os.path.join(dirpath, d)
            if d == "__pycache__" or os.path.islink(p):
                skipped += 1 if os.path.islink(p) else 0
                continue
            os.makedirs(os.path.join(target, d), exist_ok=True)
            keep.append(d)
        dirnames[:] = keep
        for f in filenames:
            p = os.path.join(dirpath, f)
            if os.path.islink(p) or not statmod.S_ISREG(os.lstat(p).st_mode):
                skipped += 1
                continue
            shutil.copyfile(p, os.path.join(target, f))
    return skipped


def _chmod_tree(root, dir_mode, file_mode):
    os.chmod(root, dir_mode)
    for dirpath, dirnames, filenames in os.walk(root):
        for d in dirnames:
            os.chmod(os.path.join(dirpath, d), dir_mode)
        for f in filenames:
            os.chmod(os.path.join(dirpath, f), file_mode)


def _snapshot_dir(src, dst):
    """Root-owned 0700 / 0600 copy of `src` (regular files and directories only). Returns the skipped entries."""
    if os.path.isdir(dst):
        shutil.rmtree(dst, ignore_errors=True)
    skipped = _copy_regular_files(src, dst)
    _chmod_tree(dst, 0o700, 0o600)
    return skipped


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
           "submission_root_rejected": 0.0,
           "submission_oversize": 0.0, "saved": None, "parent": None, "reasons": [], "t_ref": None}
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
        try:
            FREEZER.start()                   # whatever supervisord respawns from here on stays stopped until the verdict
        except Exception:  # noqa: BLE001 - the freezer never takes the verdict down (agent_processes_frozen then reads 0)
            pass
        if left:
            # The image's supervisord respawns the account's desktop as fast as it is killed, so in the separate
            # verifier a process that outlives the kill is the image's own. It is frozen (a stopped process cannot exit,
            # so it is not respawned); only one still running once frozen voids the run.
            out["agent_processes_running"] = float(len(_still_running(uid)))
            log("%d process(es) of the agent account outlived the kill at verify start, %d still running after the "
                "freeze" % (left, out["agent_processes_running"]))
    out["saved"] = lock_stage(STAGE_DIR)
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
        n, apparent, allocated = _submission_measure(SUBMISSION)
        if n > SUBMISSION_ENTRIES_CAP or max(apparent, allocated) > SUBMISSION_BYTES_CAP:
            out["submission_oversize"] = 1.0
            out["reasons"].append("the submission holds %s entries / %d apparent bytes (caps %d entries, %d bytes): not "
                                  "copied, graded as an empty folder" % (
                                      n if n <= SUBMISSION_ENTRIES_CAP else "> %d" % SUBMISSION_ENTRIES_CAP,
                                      apparent, SUBMISSION_ENTRIES_CAP, SUBMISSION_BYTES_CAP))
            return out
        if gate_active:
            late = lockdown.late_entries(SUBMISSION, t_ref)
            out["submission_late_writes"] = float(len(late))
            if late:
                out["reasons"].append("%d entr%s of the submission written after the agent phase ended (%s)" % (
                    len(late), "y" if len(late) == 1 else "ies", ", ".join(e[0] for e in late[:5])))
        os.makedirs(SNAP_ROOT, exist_ok=True)
        os.chmod(SNAP_ROOT, 0o700)
        _snapshot_dir(SUBMISSION, SNAPSHOT)
        out["submission_snapshot"] = 1.0
    return out


LOCKDOWN_STATS = ("agent_processes_killed", "agent_processes_left", "agent_processes_running", "stage_locked",
                  "stage_parent_locked", "submission_late_writes", "late_write_gate_active", "submission_snapshot",
                  "submission_root_rejected", "submission_oversize")


def lockdown_stats(early):
    """The lockdown's statistics for the verdict (never its reference time t_ref, a clock value)."""
    return {k: float(early[k]) for k in LOCKDOWN_STATS}


def _restore(early):
    try:
        if early.get("saved") is not None:
            unlock_stage(early["saved"])
        if early.get("parent"):
            os.chmod(early["parent"][0], early["parent"][1])
        mode = os.environ.get("ARB_STAGE_MODE")
        if mode:
            os.chmod(STAGE_DIR, int(mode, 8))
    except OSError:
        pass


# ------------------------------------------------------------------------------------------------ the task's own files
def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for b in iter(lambda: fh.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def check_task_files(stage_dir, cfg):
    """Raise TaskError unless gt/ equals gt/MANIFEST.sha256, the hidden set's code digests equal the shipped grading
    files and its counts equal grader_config.json. Returns the hidden set."""
    gt = os.path.join(stage_dir, "gt")
    man = os.path.join(gt, "MANIFEST.sha256")
    try:
        with open(man, encoding="utf-8") as fh:
            lines = [ln for ln in fh.read().splitlines() if ln.strip()]
    except OSError as exc:
        raise TaskError("the manifest cannot be read (%s): gt/MANIFEST.sha256" % exc)
    listed = {}
    for ln in lines:
        parts = ln.split("  ", 1)
        if len(parts) != 2 or len(parts[0]) != 64:
            raise TaskError("a malformed manifest line: gt/MANIFEST.sha256")
        rel = parts[1]
        if os.path.isabs(rel) or ".." in rel.replace("\\", "/").split("/"):
            raise TaskError("a manifest path outside gt/: %s" % rel)
        listed[rel] = parts[0]
    present = set()
    for dirpath, _dirs, files in os.walk(gt):
        for fn in files:
            rel = os.path.relpath(os.path.join(dirpath, fn), gt).replace(os.sep, "/")
            if rel != "MANIFEST.sha256":
                present.add(rel)
    for rel in sorted(set(listed) | present):
        if rel not in listed:
            raise TaskError("a file of gt/ is not in the manifest: gt/%s" % rel)
        if rel not in present:
            raise TaskError("a file of the manifest is missing: gt/%s" % rel)
        if _sha256(os.path.join(gt, rel)) != listed[rel]:
            raise TaskError("a file of gt/ differs from the manifest: gt/%s" % rel)
    try:
        with open(os.path.join(gt, "hidden_set.json"), encoding="utf-8") as fh:
            hidden_set = json.load(fh)
    except (OSError, ValueError) as exc:
        raise TaskError("the hidden set cannot be read (%s): gt/hidden_set.json" % exc)
    if hidden_set.get("schema") != "hd04-set/1":
        raise TaskError("the hidden set's schema is not hd04-set/1: gt/hidden_set.json")
    code = hidden_set.get("code_sha256") or {}
    for name, rel in sorted(CODE_FILES.items()):
        p = os.path.join(stage_dir, rel)
        if name not in code:
            raise TaskError("the hidden set holds no digest of %s: gt/hidden_set.json" % rel)
        if not os.path.isfile(p) or _sha256(p) != code[name]:
            raise TaskError("a grading file differs from the one the hidden set was drawn with: %s" % rel)
    for group, n in sorted(cfg["counts"].items()):
        if len(hidden_set.get(group, [])) != int(n):
            raise TaskError("the hidden set holds %d %s records, grader_config.json %d: gt/hidden_set.json"
                            % (len(hidden_set.get(group, [])), group, int(n)))
    return hidden_set


# ------------------------------------------------------------------------------------------------ the verdict
def _finite_metrics(metrics):
    """`metrics` in name order, finite numbers only: ale_verify.write refuses NaN and infinity."""
    out = {}
    for k, val in sorted(metrics.items()):
        try:
            x = float(val)
        except (TypeError, ValueError):
            continue
        if math.isfinite(x):
            out[k] = x
    return out


def write_verdict(GR, rewards, cfg, stats, details):
    """One gated `reward` (the kit's envelope) = description_score; the format check, the items and the listed
    statistics go to `metrics`, short per-item notes into the verdict's `diagnostic` text. In the engine
    (ALE_VERIFY_PYTHON set) the kit writes /logs/verifier/reward.json and verification.json through ale_verify; a
    host-side run writes the plain envelope to ROBOTICS_GRADER_VERDICT_PATH, else verdict.local.json in the stage
    folder."""
    from robotics_grader import ratio_rewards, read_anchor
    from robotics_grader import write_verdict as kit_write
    overall = GR.description_score(rewards, cfg)
    anchor, meta = read_anchor()
    allowed = set(cfg["metric_keys"])
    extra = {k: float(val) for k, val in stats.items() if isinstance(val, (int, float)) and not isinstance(val, bool)}
    extra.update({k: float(rewards[k]) for k in list(GR.GATES) + list(GR.ITEMS)})
    metric_name = meta.get("metric", "description_score")
    outside = sorted(set(extra) - allowed) + ([metric_name] if metric_name not in allowed else [])
    if outside:
        raise TaskError("statistics outside grader_config.json metric_keys: %s" % outside[:8])
    verdict = ratio_rewards(overall, anchor, higher_is_better=meta.get("direction", "higher") == "higher",
                            metric_name=metric_name, extra=extra, full_at=meta.get("full_at", 1.0))
    stray = sorted(set(verdict["metrics"]) - allowed - set(KIT_ENVELOPE_KEYS))
    if stray:
        raise TaskError("verdict metrics outside grader_config.json metric_keys: %s" % stray[:8])
    text = "; ".join("%s %.4f: %s" % (k, float(rewards[k]), (" | ".join(str(d) for d in details.get(k, [])))[:220])
                     for k in list(GR.GATES) + list(GR.ITEMS))
    if os.environ.get("ALE_VERIFY_PYTHON"):
        recorded = {"rewards": verdict["rewards"], "metrics": _finite_metrics(verdict["metrics"])}
        kit_write(recorded, diagnostic=text[:2000])
        return overall
    kit_write(verdict, path=os.environ.get("ROBOTICS_GRADER_VERDICT_PATH") or os.path.join(STAGE_DIR, "verdict.local.json"),
              diagnostic=text)
    return overall


# ------------------------------------------------------------------------------------------------ rule 11: the budget
def _limit_child(cpu_s, parent):
    """In the grading child: no core file, the CPU-time limit (soft = the budget in whole seconds, SIGXCPU's default
    action ends the child; hard = 5 s later, SIGKILL), and death with the parent (PR_SET_PDEATHSIG)."""
    import resource
    try:
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    except (ValueError, OSError):
        pass
    soft = max(1, int(math.ceil(float(cpu_s))))
    _cur, hard = resource.getrlimit(resource.RLIMIT_CPU)
    if hard != resource.RLIM_INFINITY:
        soft = min(soft, int(hard))
        hard = min(int(hard), soft + 5)
    else:
        hard = soft + 5
    resource.setrlimit(resource.RLIMIT_CPU, (soft, hard))
    signal.signal(signal.SIGXCPU, signal.SIG_DFL)
    try:
        import ctypes
        ctypes.CDLL(None, use_errno=True).prctl(1, int(signal.SIGKILL), 0, 0, 0)     # 1 = PR_SET_PDEATHSIG
    except Exception:  # noqa: BLE001 - without it a child left by a killed parent still ends at its CPU limit
        pass
    if os.getppid() != parent:                 # the parent died before the line above: end now
        os._exit(72)


def grade_in_budget(work, cpu_s, wall_s):
    """Run work() (the grading after the gate; it returns a picklable result) in a forked child under rule 11's
    limits. Returns (kind, payload, child CPU seconds): ("ok", work's result), ("exception", the traceback text),
    ("budget", None) when the child was stopped at its CPU limit, ("watchdog", None) when it ran wall_s seconds of
    wall-clock time, ("crash", a description) when it ended any other way. The child never returns into the caller
    (os._exit). The freezer's thread, running at the fork as root, holds no lock the child uses (it only reads /proc
    and sends SIGSTOP), so Python's warning about a fork in a threaded process is silenced here."""
    import pickle
    import select
    import warnings
    r, w = os.pipe()
    sys.stdout.flush()
    sys.stderr.flush()
    parent = os.getpid()
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        pid = os.fork()
    if pid == 0:                               # the child
        code = 71
        try:
            os.close(r)
            _limit_child(cpu_s, parent)
            try:
                msg = ("ok", work())
            except Exception:  # noqa: BLE001 - as in the parent before: a grading that raises is a grader exception
                import traceback
                msg = ("exception", traceback.format_exc())
            view = memoryview(pickle.dumps(msg, protocol=pickle.HIGHEST_PROTOCOL))
            while len(view):
                view = view[os.write(w, view):]
            code = 0
        except BaseException:  # noqa: BLE001 - never back into the caller's code
            code = 71
        finally:
            os._exit(code)
    os.close(w)
    chunks, watchdog = [], False
    deadline = time.monotonic() + float(wall_s)
    try:
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                watchdog = True
                try:
                    os.kill(pid, signal.SIGKILL)
                except OSError:
                    pass
                break
            ready, _w, _x = select.select([r], [], [], min(left, 5.0))
            if ready:
                b = os.read(r, 1 << 20)
                if not b:
                    break
                chunks.append(b)
    finally:
        os.close(r)
    _pid, status, ru = os.wait4(pid, 0)
    used = float(ru.ru_utime + ru.ru_stime)
    if watchdog:
        return "watchdog", None, used
    if os.WIFEXITED(status) and os.WEXITSTATUS(status) == 0:
        try:
            kind, payload = pickle.loads(b"".join(chunks))
        except Exception:  # noqa: BLE001
            return "crash", "the grading child's result cannot be read", used
        return kind, payload, used
    if os.WIFSIGNALED(status):
        sig = os.WTERMSIG(status)
        if sig == signal.SIGXCPU or (sig == signal.SIGKILL and used >= float(cpu_s)):
            return "budget", None, used
        return "crash", "the grading child ended by signal %d after %.1f s of CPU" % (sig, used), used
    return "crash", "the grading child exited with code %d after %.1f s of CPU" % (os.WEXITSTATUS(status), used), used


def budget_message(kind, cpu_s, wall_s):
    """Rule 11's diagnostic (no measured time in it: the verdict holds no timing)."""
    if kind == "watchdog":
        return ("rule 11: grading this description was stopped after %.0f s of wall-clock time (the grader's work "
                "after the gate may take at most %.0f s of CPU time and %.0f s of wall-clock time)" % (wall_s, cpu_s,
                                                                                                    wall_s))
    return ("rule 11: grading this description needs more than %.0f s of CPU time (the budget of the grader's work "
            "after the gate): scored 0" % cpu_s)


def _zero_verdict(reason, stats):
    """All rewards 0 (the lockdown itself failed): still a verdict, never a task error."""
    from hand_desc import grade as GR
    with open(os.path.join(STAGE_DIR, "grader_config.json")) as fh:
        cfg = json.load(fh)
    rewards = {k: 0.0 for k in list(GR.GATES) + list(GR.ITEMS)}
    details = {"model_loads": [reason[:1500]]}
    return write_verdict(GR, rewards, cfg, dict(stats, lockdown_exception=1.0), details)


def main():
    t0 = time.time()
    try:
        early = _early_lockdown()
    except Exception:  # noqa: BLE001 - never a task error without a verdict
        import traceback
        log("the verify-start lockdown raised:\n" + traceback.format_exc())
        _zero_verdict("the verify-start lockdown failed: " + traceback.format_exc()[-800:],
                      {"agent_processes_frozen": float(len(FREEZER.frozen))})
        FREEZER.stop()
        return 0
    stats = lockdown_stats(early)
    log("lockdown %.1f s" % (time.time() - t0))
    task_error = None
    try:
        if early["agent_processes_running"] > 0:
            raise RuntimeError("%d agent process(es) survived the verify-start kill and could not be stopped: run void"
                               % early["agent_processes_running"])
        with open(os.path.join(STAGE_DIR, "grader_config.json")) as fh:
            cfg = json.load(fh)
        check_task_files(STAGE_DIR, cfg)
        from hand_desc import gate as GT
        from hand_desc import grade as GR
        try:
            gt = GR.load_gt(GT_DIR)
        except Exception as exc:  # noqa: BLE001 - the task's own unit does not load: a task fault
            raise TaskError("the unit's files in gt/ do not load (%s): gt/" % str(exc)[:300])
        sub_dir = SNAPSHOT if early["submission_snapshot"] else SUBMISSION
        if early["submission_root_rejected"] > 0 or early["submission_oversize"] > 0:   # never read: an empty folder
            import tempfile
            sub_dir = tempfile.mkdtemp(prefix="arb_rejected_submission_")
        stats["grader_exception"] = 0.0
        t2 = time.time()
        child_cpu = 0.0
        try:
            gate = GT.gate_model_loads(sub_dir)
            if not gate[0]:                    # rules 1 to 10 failed: grade.grade only fills the zeros
                rewards, gstats, details = GR.grade(sub_dir, GT_DIR, cfg, gate=gate, gt=gt, log=log)
            else:                              # rule 11: the work after the gate in a child under the budget
                budget_s = float(GT.GRADE_CPU_S)
                wall_s = float(GT.GRADE_WALL_FACTOR) * budget_s
                kind, payload, child_cpu = grade_in_budget(
                    lambda: GR.grade(sub_dir, GT_DIR, cfg, gate=gate, gt=gt, log=log), budget_s, wall_s)
                if kind == "ok":
                    rewards, gstats, details = payload
                elif kind in ("budget", "watchdog"):
                    log("rule 11: the grading child was stopped (%s) after %.1f s of CPU" % (kind, child_cpu))
                    rewards = {k: 0.0 for k in list(GR.GATES) + list(GR.ITEMS)}
                    gstats = GR.empty_stats()
                    details = {"model_loads": [budget_message(kind, budget_s, wall_s)]}
                else:
                    log("exception while grading the submission (in the grading child):\n" + str(payload))
                    rewards = {k: 0.0 for k in list(GR.GATES) + list(GR.ITEMS)}
                    gstats, details = {}, {"model_loads": ["grader exception on this submission"]}
                    stats["grader_exception"] = 1.0
        except Exception:  # noqa: BLE001 - a submission that crashes the grader earns 0, not a task error
            import traceback
            log("exception while grading the submission:\n" + traceback.format_exc())
            rewards = {k: 0.0 for k in list(GR.GATES) + list(GR.ITEMS)}
            gstats, details = {}, {"model_loads": ["grader exception on this submission"]}
            stats["grader_exception"] = 1.0
        log("grade %.1f s (CPU %.1f s here, %.1f s in the grading child)" % (time.time() - t2, time.process_time(),
                                                                            child_cpu))
        for k, v in gstats.items():
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                stats[k] = float(v)
        final = 0.0 if (early["submission_late_writes"] > 0 or early["submission_root_rejected"] > 0
                        or early["submission_oversize"] > 0) else 1.0
        stats["submission_final"] = final
        if final < 1.0:                       # the deliverable must be final when the session ends: model_loads = 0
            rewards["model_loads"] = 0.0
            details["model_loads"] = ["submission_final: " + "; ".join(early["reasons"])] + list(details.get("model_loads", []))
        gate_v = 1.0
        for g in GR.GATES:
            gate_v *= float(rewards[g])
        for c in GR.ITEMS:
            rewards[c] = float(rewards[c]) * (1.0 if gate_v > 0 else 0.0)
        stats["agent_processes_frozen"] = float(len(FREEZER.frozen))
        overall = write_verdict(GR, rewards, cfg, stats, details)
        log("format check {} items {} overall {:.4f} ({:.0f} s)".format(
            {g: rewards[g] for g in GR.GATES}, {c: round(rewards[c], 4) for c in GR.ITEMS}, overall, time.time() - t0))
    except TaskError as exc:
        task_error = exc
    finally:
        _restore(early)
        FREEZER.stop()                        # after the verdict; the stopped processes are never resumed
    if task_error is not None:
        print("TASK ERROR: %s" % task_error, file=sys.stderr, flush=True)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
