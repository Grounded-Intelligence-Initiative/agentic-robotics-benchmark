"""Grader for arm_hidden_mechanism_box, dual mode (experimental): one verify for the two scoring items.

Runs as root in a SEPARATE verifier sandbox. The agent had one session for two items, scored together:
  * the program (the offline item, main's task): the controller it left in /home/user/submission/, graded by main's
    own grader, unchanged (tests/offline_verify.py, byte-identical to main's tests/verify.py: 8 episodes on the four
    hidden boxes with fresh seeds, the kit's wire with the probe lock, the seccomp launcher, the budgets);
  * the graded episode (the online item): one hidden episode it played itself through the lab
    (environment/setup/lab/alelab/server.py, the account `alelab`), whose record the lab wrote to
    /var/lib/arb-online/record.json.
The program is frozen at the earlier of the graded start and the close of the start window: the lab's root supervisor
copied the submission folder into /var/lib/arb-online/submission_snapshot/ before the start was charged, or when the
start window closed without a start (the record holds the trigger, the status, the counts and the digest);
the engine collected that directory and the submission as declared artifacts after the agent phase and restored them
here, root-owned.

In this order: stop every agent-UID process (as main's grader does); read the record; if it records a freeze (at
the start or at the close of the start window), the program graded is the snapshot (its digest checked against the
record; a snapshot that is not "ok" or does not match scores the program 0), else (the session ended before the
window closed, or there is no usable record) the restored session-end submission, as on main; run main's grader on
it, its verdict
captured instead of written (its anchor: tests/anchor_offline.json, main's anchor.json); score the record as the
online variant does (tests/anchor_online.json; 0 without a record or a start); stop the agent account's processes
again; reward = the mean of the two items' rewards, written once through the kit against tests/anchor.json
(dual_mode_score, 1.0 when both items reach their anchors).
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

STAGE = os.path.dirname(os.path.abspath(__file__))
ONLINE = os.path.join(STAGE, "online.json")
RECORD = "/var/lib/arb-online/record.json"
SNAPSHOT = "/var/lib/arb-online/submission_snapshot"
RECORD_FORMAT = "arb-online-record/1"
MAX_RECORD_BYTES = 1 << 20
METRIC = "mean_opening_progress"  # both items' metric (main's, and the graded episode's on the same scale)
END_REASONS = ("done", "finish", "graded_window", "worker_restart")


def log(msg):
    print("[grader] " + msg, file=sys.stderr, flush=True)


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
                        # a zombie runs no code; it only waits to be reaped (r5: a zombie made the
                        # bounded kill report cleanup_failed in 1 of 24 oracle runs under load)
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
    """SIGSTOP every live process of `uid` until a pass finds none still running (a respawn that appears meanwhile is
    stopped by the next pass) or `bound_s` elapsed; returns the PIDs still running (empty: every one is stopped)."""
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
    """kill_agent_processes, then: True when none is left or every one left is stopped. The base image's supervisord
    respawns the agent account's desktop as fast as it is killed, so in the separate verifier a process that outlives
    the bounded kill is the image's own; it is stopped (a stopped process does not exit, so it is not respawned) and
    only one still running once stopped fails the run (cleanup_failed), as before."""
    if kill_agent_processes(uid):
        return True
    running = _still_running(uid)
    log("{} process(es) of the agent account outlived the kill, {} still running after the freeze".format(
        len(_agent_pids(uid)), len(running)))
    return not running


def _no_constants(name):
    raise ValueError("non-finite constant")


def load_record(path):
    """The record as a JSON object (a regular file of at most 1 MB, never through a link), or (None, reason)."""
    try:
        st = os.lstat(path)
    except OSError:
        return None, "no graded-episode record (the lab wrote none)"
    if not stat.S_ISREG(st.st_mode):
        return None, "the graded-episode record is not a regular file"
    if st.st_size > MAX_RECORD_BYTES:
        return None, "the graded-episode record exceeds 1 MB"
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except OSError:
        return None, "the graded-episode record cannot be opened"
    try:
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            return None, "the graded-episode record is not a regular file"
        raw = os.read(fd, MAX_RECORD_BYTES + 1)
    finally:
        os.close(fd)
    if len(raw) > MAX_RECORD_BYTES:
        return None, "the graded-episode record exceeds 1 MB"
    try:
        rec = json.loads(raw.decode("utf-8"), parse_constant=_no_constants)
    except (ValueError, UnicodeDecodeError):
        return None, "the graded-episode record is not valid JSON"
    if not isinstance(rec, dict):
        return None, "the graded-episode record is not a JSON object"
    return rec, None


def check_record(rec, online):
    """None for a record of this task's graded episode that was started with a finite score >= 0, else the reason."""
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
    """The record's counters, how the episode ended and the per-episode sub-metrics, as finite numbers."""
    out = {}
    for key in ("steps", "steps_rejected", "sim_time_s", "requests", "wall_s_since_start", "started_after_s"):
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


def snapshot_digest(root):
    """(files, bytes, sha256) of a snapshot tree, computed as the lab computes it: sha256 over the lines
    "<path>\\0<size>\\0<sha256 of content>\\n" of its regular files in sorted path order (no link is followed)."""
    listing = []
    for dp, dns, fns in os.walk(root, followlinks=False):
        for fn in fns:
            fp = os.path.join(dp, fn)
            st = os.lstat(fp)
            if not stat.S_ISREG(st.st_mode):
                continue
            h, size = hashlib.sha256(), 0
            with open(fp, "rb") as fh:
                for block in iter(lambda: fh.read(1 << 20), b""):
                    h.update(block)
                    size += len(block)
            listing.append((os.path.relpath(fp, root), size, h.hexdigest()))
    digest = hashlib.sha256()
    for rel, size, content_sha in sorted(listing):
        digest.update(("%s\0%d\0%s\n" % (rel, size, content_sha)).encode("utf-8", "surrogateescape"))
    return len(listing), sum(x[1] for x in listing), digest.hexdigest()


def check_snapshot(rec, root=SNAPSHOT):
    """None when the record says the program was frozen ("ok") and the restored snapshot matches its counts and
    digest, else the reason (the program item then scores 0)."""
    status = rec.get("snapshot_status")
    if status != "ok":
        return "the program was not frozen at the graded start (snapshot {})".format(status)
    if not os.path.isdir(root) or os.path.islink(root):
        return "the frozen program is missing from the artifacts"
    files, size, digest = snapshot_digest(root)
    if (files, size, digest) != (rec.get("snapshot_files"), rec.get("snapshot_bytes"), rec.get("snapshot_sha256")):
        return "the frozen program does not match the record's digest"
    return None


def install_snapshot(root, dest):
    """Replace the restored submission folder `dest` with the frozen program (as root): `dest` is removed without
    following links, then the snapshot's directories and regular files are copied with their modes. Main's grader
    then hands the folder to the agent account, as it does with a restored submission."""
    if os.path.islink(dest) or (os.path.lexists(dest) and not os.path.isdir(dest)):
        os.unlink(dest)
    elif os.path.isdir(dest):
        shutil.rmtree(dest)
    os.mkdir(dest, 0o700)
    for dp, dns, fns in os.walk(root, followlinks=False):
        rel = os.path.relpath(dp, root)
        target = dest if rel == "." else os.path.join(dest, rel)
        for dn in dns:
            if not os.path.islink(os.path.join(dp, dn)):
                os.mkdir(os.path.join(target, dn), 0o700)
        for fn in fns:
            sp, tp = os.path.join(dp, fn), os.path.join(target, fn)
            st = os.lstat(sp)
            if stat.S_ISREG(st.st_mode):
                shutil.copyfile(sp, tp, follow_symlinks=False)
                os.chmod(tp, stat.S_IMODE(st.st_mode))


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
        raise RuntimeError("main's grader wrote no verdict")
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
        uid, gid = _agent_ids()
    except KeyError:
        write_verdict(zero_verdict(reason="no agent account", anchored=anchor is not None))
        return
    if not stop_agent_processes(uid):
        write_verdict(zero_verdict(reason="agent processes could not be stopped (cleanup_failed)",
                                   anchored=anchor is not None))
        return
    rec, load_reason = load_record(RECORD)
    started = rec is not None and rec.get("started") is True
    online_reason = load_reason if rec is None else check_record(rec, online)
    # the program graded: the one frozen at the earlier of the start and the close of the start window; the
    # session-end submission only when the session ended before that (a usable record with no freeze) or when there
    # is no usable record
    usable = rec is not None and rec.get("format") == RECORD_FORMAT and rec.get("task") == online["task"]
    frozen = usable and (started or rec.get("snapshot_status") is not None)
    program = "session_end"
    offline_reason = None
    if frozen:
        offline_reason = check_snapshot(rec)
        if offline_reason is None:
            install_snapshot(SNAPSHOT, os.path.join(os.path.expanduser("~" + agent_user()), "submission"))
            program = "snapshot"
    if offline_reason is None:
        off, off_diag = run_offline()
    else:
        log("program item: " + offline_reason)
        off, off_diag = zero_verdict(reason=offline_reason, anchored=True), None
        program = "none"
    on = online_component(rec, online_reason, config)
    cleanup_ok = stop_agent_processes(uid)
    off_r, on_r = float(off["rewards"]["reward"]), float(on["rewards"]["reward"])
    combined = (off_r + on_r) / 2.0
    if not cleanup_ok:
        combined = 0.0
    extra = {"offline_reward": off_r, "online_reward": on_r, "cleanup_failed": 0.0 if cleanup_ok else 1.0,
             "program_snapshot": 1.0 if program == "snapshot" else 0.0,
             "program_session_end": 1.0 if program == "session_end" else 0.0,
             "snapshot_at_start": 1.0 if (rec or {}).get("snapshot_trigger") == "start" else 0.0,
             "snapshot_at_window_close": 1.0 if (rec or {}).get("snapshot_trigger") == "prep_window"
             else 0.0,
             "episode_started": 1.0 if started else 0.0}
    for status in ("ok", "too_large", "missing"):
        extra["snapshot_" + status] = 1.0 if (rec or {}).get("snapshot_status") == status else 0.0
    for key in ("snapshot_files", "snapshot_bytes"):
        if _number((rec or {}).get(key)):
            extra[key] = float(rec[key])
    for reason in END_REASONS:
        extra["online_end_" + reason] = 1.0 if (rec or {}).get("end_reason") == reason else 0.0
    for key, v in sorted((off.get("metrics") or {}).items()):
        extra["offline_" + key] = v
    for key, v in sorted((on.get("metrics") or {}).items()):
        extra["online_" + key] = v
    verdict = ratio_rewards(combined, anchor, cap=1.0, higher_is_better=True, metric_name="dual_mode_score",
                            locks_ok=True, extra=extra, full_at=meta.get("full_at", 1.0))
    diagnostic = ("dual mode: program {} ({} at {}; reward {:.4f}), graded episode {} (reward {:.4f}; {}), "
                  "mean {:.4f}; program item: {}; graded episode item: {}").format(
        program, (rec or {}).get("snapshot_status"), (rec or {}).get("snapshot_trigger"), off_r,
        (rec or {}).get("episode"), on_r,
        (rec or {}).get("end_reason"), combined, off_diag or off.get("diagnostic") or "graded",
        on.get("diagnostic") or "scored")
    write_verdict(verdict, diagnostic=diagnostic)
    log("{} anchor={} reward={:.3f}".format(diagnostic, anchor, verdict["rewards"]["reward"]))


if __name__ == "__main__":
    main()
