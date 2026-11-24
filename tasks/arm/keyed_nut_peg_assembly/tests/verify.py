"""Grader for keyed_nut_peg_assembly, dual mode (experimental; never merged into main): one verify for the two items.

Runs as root in a SEPARATE verifier sandbox. The agent had one session for two scoring items, scored together:
  * the program (main's task): the controller it left in /home/user/submission/, graded by main's own grader,
    unchanged (tests/offline_verify.py, byte-identical to main's tests/verify.py: the 10 fixed hidden kits of
    ref/episodes.json, the kit wire with its probe lock, the seccomp launcher, the per-kit controller budget and image
    charges, the wall-clock safety net), against main's anchor (tests/anchor_offline.json, main's anchor.json);
  * the graded episode: one hidden kit of that set it assembled itself, live, through the graded episode's service
    (environment/setup/online/graded_service.py, the account `keyedgrade`), whose record the service wrote to
    /var/lib/arb-online/record.json, against the online anchor (tests/anchor_online.json).
The program is frozen at the earlier of the graded start and the close of the start window: the service's root
supervisor copied the submission folder into /var/lib/arb-online/submission_snapshot/ before the start was charged, or
when the start window closed without a start (the record holds the trigger, the status, the counts and the digest; the
service's state.json holds the same). The engine collected that directory and the submission as declared artifacts
after the agent phase and restored them here, root-owned.

In this order: stop every agent-UID process (as main's grader does); read and check the record (a regular file of at
most 1 MB, strict JSON, this task's format, task, episode and seed digest as tests/online.json); if the record (or,
failing it, state.json) records a freeze, the program graded is the snapshot: it must be status ok and match the
record's counts and digest (a missing or altered copy is a task error), and it replaces the restored submission; a
freeze that is too_large, failed or still pending scores the program item 0 without a run; when nothing was frozen
(the session ended before the start window closed, or there is no usable record of a freeze) the restored session-end
submission is graded, as on main. Main's grader runs on it, its verdict captured instead of written. The graded
episode's item is the record's episode_score (0 without a valid, started record; a record that says the cell itself
failed is a task error, as a failure of the offline grader's renderer is). Every agent-UID process is stopped again.
reward = the mean of the two items' rewards, each normalised by its own anchor, cap and full_at, written once through
the kit against tests/anchor.json (dual_mode_score, 1.0 when both items reach their anchors).
Untouched sandbox: no record, an empty submission -> 0 + 0. Oracle: the reference controller frozen at the start (main's
oracle) and the reference on the graded episode -> 1 + 1.
"""

import hashlib
import json
import math
import os
import pwd
import shutil
import signal
import stat
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robotics_grader import (  # noqa: E402
    agent_user,
    ratio_rewards,
    read_anchor,
    stage_config,
    write_verdict,
    zero_verdict,
)
from robotics_grader.stage import fail  # noqa: E402

STAGE = os.path.dirname(os.path.abspath(__file__))
ONLINE = os.path.join(STAGE, "online.json")
ONLINE_DIR = "/var/lib/arb-online"
RECORD = os.path.join(ONLINE_DIR, "record.json")
STATE = os.path.join(ONLINE_DIR, "state.json")
SNAPSHOT = os.path.join(ONLINE_DIR, "submission_snapshot")
RECORD_FORMAT = "arb-online-record/1"
MAX_RECORD_BYTES = 1 << 20
METRIC = "kit_success_rate"           # both items' metric (main's, and the graded episode's on the same scale)
END_REASONS = ("done", "finish", "graded_window", "worker_restart")


def log(msg):
    print("[grader] " + msg, file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------------------------- the agent's processes
# (main's own functions, as tests/offline_verify.py has them)
def _agent_ids():
    pw = pwd.getpwnam(agent_user())
    return pw.pw_uid, pw.pw_gid


def _agent_pids(uid):
    pids = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open("/proc/{}/status".format(name)) as f:
                zombie = False
                for line in f:
                    if line.startswith("State:"):
                        zombie = line.split()[1].startswith(("Z", "X"))
                    elif line.startswith("Uid:"):
                        if int(line.split()[1]) == uid and not zombie:
                            pids.append(int(name))
                        break
        except (OSError, ValueError, IndexError):
            continue
    return pids


def kill_agent_processes(uid, rounds=20, pause=0.25):
    """Kill every process of the agent account; True when none is left (bounded)."""
    for _ in range(rounds):
        pids = _agent_pids(uid)
        if not pids:
            return True
        for pid in pids:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        time.sleep(pause)
    return not _agent_pids(uid)


def _proc_state(pid):
    """The /proc state letter of `pid` ("X" once it is gone)."""
    try:
        with open("/proc/{}/status".format(pid)) as f:
            for line in f:
                if line.startswith("State:"):
                    return line.split()[1]
    except (OSError, IndexError):
        pass
    return "X"


def _still_running(uid, bound_s=3.0):
    """SIGSTOP every live process of `uid` until a pass finds none still running or `bound_s` elapsed; returns the
    PIDs still running (empty: every one is stopped)."""
    deadline = time.monotonic() + float(bound_s)
    while True:
        for pid in _agent_pids(uid):
            try:
                os.kill(pid, signal.SIGSTOP)
            except OSError:
                pass
        time.sleep(0.05)
        running = [pid for pid in _agent_pids(uid) if _proc_state(pid) not in ("T", "t", "X")]
        if not running or time.monotonic() > deadline:
            return running


def stop_agent_processes(uid):
    """kill_agent_processes, then: True when none is left or every one left is stopped (the base image's supervisord
    respawns the agent account's desktop as fast as it is killed; a stopped process is not respawned)."""
    if kill_agent_processes(uid):
        return True
    running = _still_running(uid)
    log("{} process(es) of the agent account outlived the kill, {} still running after the freeze".format(
        len(_agent_pids(uid)), len(running)))
    return not running


# ---------------------------------------------------------------------------------------------- the record
def _no_constants(name):
    raise ValueError("non-finite constant")


def load_record(path=RECORD):
    """(object, None) for a regular file of at most 1 MB holding a JSON object (never read through a link), else
    (None, why); why is "missing" when there is no file."""
    try:
        st = os.lstat(path)
    except FileNotFoundError:
        return None, "missing"
    except OSError as e:
        return None, "unreadable ({})".format(type(e).__name__)
    if not stat.S_ISREG(st.st_mode):
        return None, "not a regular file"
    if st.st_size > MAX_RECORD_BYTES:
        return None, "larger than 1 MB"
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except OSError:
        return None, "unreadable"
    try:
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            return None, "not a regular file"
        raw = os.read(fd, MAX_RECORD_BYTES + 1)
    finally:
        os.close(fd)
    if len(raw) > MAX_RECORD_BYTES:
        return None, "larger than 1 MB"
    try:
        obj = json.loads(raw.decode("utf-8"), parse_constant=_no_constants)
    except (ValueError, UnicodeDecodeError, RecursionError):
        return None, "not valid JSON"
    if not isinstance(obj, dict):
        return None, "not a JSON object"
    return obj, None


def record_state(rec, why, online):
    """missing | invalid (unreadable, or not this task's graded episode) | not_started | started."""
    if rec is None:
        return "missing" if why == "missing" else "invalid"
    if rec.get("format") != RECORD_FORMAT or any(rec.get(k) != online[k] for k in ("task", "episode", "seed_sha256")):
        return "invalid"
    return "started" if rec.get("started") is True else "not_started"


def check_record(rec, online):
    """None for a record of this task's graded episode that was started with a finite episode_score >= 0, else why."""
    if rec.get("format") != RECORD_FORMAT:
        return "the graded-episode record has another format"
    for key in ("task", "episode", "seed_sha256"):
        if rec.get(key) != online[key]:
            return "the graded-episode record's {} differs from tests/online.json".format(key)
    if rec.get("started") is not True:
        return "the graded episode was never started"
    score = rec.get("episode_score")
    if isinstance(score, bool) or not isinstance(score, (int, float)) or not math.isfinite(score) or score < 0:
        return "the graded-episode record has no finite episode_score >= 0"
    return None


def _number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def record_metrics(rec):
    """The record's counters, how the episode ended and its per-episode sub-metrics, as finite numbers."""
    out = {}
    for key in ("steps", "sim_time_s", "requests", "wall_s_since_start", "started_after_s"):
        if _number(rec.get(key)):
            out[key] = float(rec[key])
    out["episode_ended"] = 1.0 if rec.get("ended") is True else 0.0
    for reason in END_REASONS:
        out["end_" + reason] = 1.0 if rec.get("end_reason") == reason else 0.0
    out["privileged"] = 1.0 if rec.get("privileged") is True else 0.0
    metrics = rec.get("metrics")
    if isinstance(metrics, dict):
        for key, v in sorted(metrics.items()):
            if isinstance(key, str) and _number(v) and key not in out:
                out[key] = float(v)
    return out


# ---------------------------------------------------------------------------------------------- the freeze
def snapshot_digest(entries):
    """The graded episode's service's digest (graded_service.snapshot_digest): the sorted (path, size, sha256) list."""
    h = hashlib.sha256()
    for rel, size, sha in sorted(entries):
        h.update(rel.encode("utf-8", "surrogateescape") + b"\0" + str(int(size)).encode() + b"\0" + sha.encode() + b"\n")
    return h.hexdigest()


def snapshot_listing(root):
    """(entries, files, bytes, problems) of a restored snapshot: its regular files; a link or a special file is a
    problem (the service copies neither)."""
    entries, nbytes, problems = [], 0, []
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        for name in list(dirnames):
            if os.path.islink(os.path.join(dirpath, name)):
                problems.append("link " + name)
                dirnames.remove(name)
        for name in filenames:
            p = os.path.join(dirpath, name)
            st = os.lstat(p)
            if not stat.S_ISREG(st.st_mode):
                problems.append("not a regular file: " + os.path.relpath(p, root))
                continue
            h = hashlib.sha256()
            with open(p, "rb") as f:
                for b in iter(lambda: f.read(1 << 20), b""):
                    h.update(b)
            entries.append((os.path.relpath(p, root).replace(os.sep, "/"), st.st_size, h.hexdigest()))
            nbytes += st.st_size
    return entries, len(entries), nbytes, problems


def freeze_decision(rec, rstate, state, root=SNAPSHOT):
    """(frozen_by, snapshot fields, why the program item scores 0 or None). frozen_by is graded_start,
    prep_window_closed or None (nothing frozen: the session-end submission is graded). The freeze comes from a usable
    record (this task's graded episode: started, or not started with freeze prep_window_closed), else from the
    service's state.json; with neither, nothing counts as frozen. Neither file is writable from the agent's account.
    A copy recorded ok that is missing or does not match its record is a task error (fail)."""
    frozen_by, fields = None, {}
    if rstate == "started" or (rstate == "not_started" and rec.get("freeze") == "prep_window_closed"):
        frozen_by = "graded_start" if rstate == "started" else "prep_window_closed"
        fields = {k: rec.get(k) for k in ("snapshot_status", "snapshot_files", "snapshot_bytes", "snapshot_sha256")}
    elif isinstance(state, dict) and (state.get("started") is True or state.get("frozen") == "prep_window_closed"):
        frozen_by = "graded_start" if state.get("started") is True else "prep_window_closed"
        sn = state.get("snapshot") if isinstance(state.get("snapshot"), dict) else {}
        fields = {"snapshot_status": sn.get("status"), "snapshot_files": sn.get("files"),
                  "snapshot_bytes": sn.get("bytes"), "snapshot_sha256": sn.get("sha256")}
    if frozen_by is None:
        return None, {}, None
    status = fields.get("snapshot_status")
    when = "the graded episode started" if frozen_by == "graded_start" else "the start window closed"
    if status == "too_large":
        return frozen_by, fields, ("the submission folder was larger than the program's caps (5000 files, 200 MiB, or too "
                                   "many entries or folders) when {}: nothing was frozen".format(when))
    if status != "ok":
        return frozen_by, fields, "the submission folder was not frozen when {} (status {!r})".format(when, status)
    if os.path.islink(root) or not os.path.isdir(root):
        fail("the program frozen when {} is missing from the restored artifacts".format(when))
    entries, nfiles, nbytes, problems = snapshot_listing(root)
    if problems or snapshot_digest(entries) != fields.get("snapshot_sha256") or nfiles != fields.get("snapshot_files") \
            or nbytes != fields.get("snapshot_bytes"):
        fail("the program frozen when {} does not match its record ({} files, {} bytes; {})".format(
            when, nfiles, nbytes, "; ".join(problems[:3]) or "digest"))
    return frozen_by, fields, None


def install_snapshot(src, dst):
    """Replace the restored submission folder `dst` with the frozen program (as root): `dst` is removed without
    following links, then the snapshot's folders and regular files are copied, each file with its mode. Main's grader
    then hands the folder to the agent account, as it does with a restored submission."""
    if os.path.islink(dst) or (os.path.lexists(dst) and not os.path.isdir(dst)):
        os.unlink(dst)
    elif os.path.isdir(dst):
        shutil.rmtree(dst)
    os.mkdir(dst, 0o755)
    for dirpath, dirnames, filenames in os.walk(src, followlinks=False):
        rel = os.path.relpath(dirpath, src)
        target = dst if rel == "." else os.path.join(dst, rel)
        for name in dirnames:
            if not os.path.islink(os.path.join(dirpath, name)):
                os.mkdir(os.path.join(target, name), 0o755)
        for name in filenames:
            sp, tp = os.path.join(dirpath, name), os.path.join(target, name)
            st = os.lstat(sp)
            if stat.S_ISREG(st.st_mode):
                shutil.copyfile(sp, tp, follow_symlinks=False)
                os.chmod(tp, stat.S_IMODE(st.st_mode) & 0o755)
    os.chmod(dst, 0o755)


# ---------------------------------------------------------------------------------------------- the two items
def run_offline():
    """Main's grader (tests/offline_verify.py, unchanged) on /home/user/submission: its verdict and diagnostic,
    captured instead of written; it reads main's anchor (tests/anchor_offline.json)."""
    import offline_verify as OV
    captured = []

    def capture(verdict, path=None, diagnostic=None):
        captured.append((verdict, diagnostic))

    OV.write_verdict = capture
    OV.read_anchor = lambda name="anchor.json": read_anchor("anchor_offline.json")
    OV.main()
    if not captured:
        fail("main's grader wrote no verdict")
    verdict, diagnostic = captured[-1]
    return verdict, diagnostic if diagnostic is not None else verdict.get("diagnostic")


def online_component(rec, reason, config):
    """The graded episode's item: its record scored against the online anchor, or 0 with the reason."""
    anchor, meta = read_anchor("anchor_online.json")
    if reason is not None:
        return zero_verdict(reason=reason, anchored=anchor is not None)
    return ratio_rewards(float(rec["episode_score"]), anchor, cap=config.get("scoring_cap", 1.5),
                         higher_is_better=True, metric_name=METRIC, locks_ok=True, extra=record_metrics(rec),
                         full_at=meta.get("full_at", 1.0))


def main():
    config = stage_config()
    anchor, meta = read_anchor()
    with open(ONLINE) as fh:
        online = json.load(fh)
    try:
        uid, _gid = _agent_ids()
    except KeyError:
        write_verdict(zero_verdict(reason="no agent account", anchored=anchor is not None))
        return
    if not stop_agent_processes(uid):                                            # 1
        write_verdict(zero_verdict(reason="agent processes could not be stopped (cleanup_failed)",
                                   anchored=anchor is not None))
        return
    rec, why = load_record()                                                     # 2
    rstate = record_state(rec, why, online)
    if rec is None:
        online_reason = ("no graded-episode record: the graded episode was never started" if why == "missing"
                         else "the graded-episode record is {}".format(why))
    else:
        online_reason = check_record(rec, online)
    if rstate == "started" and rec.get("cell_error") is True:
        fail("the graded episode's cell failed during the episode ({}): an error of the task, as a failure of the "
             "offline grader's renderer is".format(((rec.get("detail") or {}).get("cell_error") or "?")[:200]))
    state, _ = load_record(STATE)
    frozen_by, snap, freeze_why = freeze_decision(rec, rstate, state)            # 3
    submission = os.path.join(os.path.expanduser("~" + agent_user()), "submission")
    if frozen_by and freeze_why is None:
        install_snapshot(SNAPSHOT, submission)
        program = "snapshot"
        log("the program item grades the copy frozen when {} ({} files, {} bytes)".format(
            "the graded episode started" if frozen_by == "graded_start" else "the start window closed",
            snap.get("snapshot_files"), snap.get("snapshot_bytes")))
    elif frozen_by:
        program = "none"
        log("program item 0: " + freeze_why)
    else:
        program = "session_end"
        log("nothing was frozen (the session ended before the start window closed, or no usable record of a freeze): "
            "the program item grades the session-end submission")
    if program == "none":                                                        # 4
        off, off_diag = zero_verdict(reason=freeze_why, anchored=True), None
    else:
        off, off_diag = run_offline()
    on = online_component(rec, online_reason, config)                            # 5
    cleanup_ok = stop_agent_processes(uid)
    off_r, on_r = float(off["rewards"]["reward"]), float(on["rewards"]["reward"])
    combined = (off_r + on_r) / 2.0                                              # 6
    if not cleanup_ok:
        combined = 0.0
    r = rec or {}
    extra = {"offline_reward": off_r, "online_reward": on_r, "cleanup_failed": 0.0 if cleanup_ok else 1.0,
             "program_snapshot": 1.0 if program == "snapshot" else 0.0,
             "program_session_end": 1.0 if program == "session_end" else 0.0,
             "program_none": 1.0 if program == "none" else 0.0,
             "frozen_at_graded_start": 1.0 if frozen_by == "graded_start" else 0.0,
             "frozen_at_window_close": 1.0 if frozen_by == "prep_window_closed" else 0.0,
             "episode_started": 1.0 if rstate == "started" else 0.0}
    for status in ("ok", "too_large", "failed", "pending"):
        extra["snapshot_" + status] = 1.0 if snap.get("snapshot_status") == status else 0.0
    for key in ("snapshot_files", "snapshot_bytes"):
        if _number(snap.get(key)):
            extra[key] = float(snap[key])
    for reason in END_REASONS:
        extra["online_end_" + reason] = 1.0 if r.get("end_reason") == reason else 0.0
    for key, v in sorted((off.get("metrics") or {}).items()):
        extra["offline_" + key] = v
    for key, v in sorted((on.get("metrics") or {}).items()):
        extra["online_" + key] = v
    verdict = ratio_rewards(combined, anchor, cap=1.0, higher_is_better=True, metric_name="dual_mode_score",
                            locks_ok=True, extra=extra, full_at=meta.get("full_at", 1.0))
    diagnostic = ("dual mode: reward = mean(program item {:.4f}, graded episode item {:.4f}) = {:.4f}. The program ({}): "
                  "{} The graded episode ({}; ended {}, {}): {}").format(
        off_r, on_r, combined,
        {"snapshot": "frozen at " + str(frozen_by), "session_end": "the session-end submission",
         "none": "not graded"}[program],
        off_diag or off.get("diagnostic") or "graded.", r.get("episode"), r.get("ended"), r.get("end_reason"),
        on.get("diagnostic") or "scored")
    write_verdict(verdict, diagnostic=diagnostic)
    log("{} anchor={} reward={:.3f}".format(diagnostic, anchor, verdict["rewards"]["reward"]))


if __name__ == "__main__":
    main()
