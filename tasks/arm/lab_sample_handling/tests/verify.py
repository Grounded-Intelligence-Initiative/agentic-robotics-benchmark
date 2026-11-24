"""Grader for lab_sample_handling, dual mode (experimental): one verify for the two scoring items.

Runs as root in a SEPARATE verifier sandbox (tests/Dockerfile's image). The agent had one session for two items,
scored together:
  * the controller (the offline item, main's task): the controller it left in /home/user/submission/, graded by main's
    own grader, unchanged (tests/offline_verify.py, byte-identical to main's tests/verify.py: the fixed hidden set of
    12 episodes, the kit's wire with the probe lock, the seccomp launcher, the controller-time budget, the numerics
    check, the render self-test, the oracle token's truth for the reference), against main's anchor
    (tests/anchor_offline.json, byte-identical to main's tests/anchor.json);
  * the graded episode (the online item): one episode of that hidden set (tests/online.json: hidden-02, unit u1) that
    it played itself through the graded episode's service (environment/setup/online/graded_service.py, the account
    `labgrade`), whose record the service wrote to /var/lib/arb-online/record.json, against the online anchor
    (tests/anchor_online.json: the reference's measured credit on that episode).
The controller is frozen at the earlier of the graded start and the close of the start window: the service's root
supervisor copied the submission folder into /var/lib/arb-online/submission_snapshot/ before the start was charged,
or when the start window closed without a start (the record holds the trigger, the status, the counts and the
digest); the engine collected that directory and the submission as declared artifacts after the agent phase and
restored them here, root-owned.

In this order: reclaim root's PATH directories (robotics_grader.stage.secure_exec_path) and stop every agent-UID
process (as main's grader does, which does it again); read the record; if it (or, failing a usable record, the
service's state.json) records a freeze, at the start or at the close of the start window, the controller graded is the
snapshot (its digest checked against the record; a snapshot that is not "ok", is missing or does not match scores the
controller item 0 without a run), else (the session ended before the window closed, or there is no usable record of a
freeze) the restored session-end submission, as on main; run main's grader on it in this process, its verdict
captured instead of written and its anchor read from anchor_offline.json (a task error there, such as the numerics
check or the renderer's self-test failing, is a task error here); score the record (0 without a valid, started
record; a record that says the renderer failed during the graded episode is a task error, as a renderer failure is
in main's grading); stop the agent account's processes again; reward = the mean of the two items' rewards, written
once through the kit against tests/anchor.json (dual_mode_score, 1.0 when both items reach their anchors).
Untouched sandbox: no record and an empty submission -> 0 + 0. Oracle: the reference frozen at the start (main's
oracle) and the privileged reference on the graded episode -> 1 + 1.
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
from robotics_grader.stage import fail, secure_exec_path  # noqa: E402

STAGE = os.path.dirname(os.path.abspath(__file__))
ONLINE = os.path.join(STAGE, "online.json")
ONLINE_DIR = "/var/lib/arb-online"
RECORD = os.path.join(ONLINE_DIR, "record.json")
STATE = os.path.join(ONLINE_DIR, "state.json")
SNAPSHOT = os.path.join(ONLINE_DIR, "submission_snapshot")
RECORD_FORMAT = "arb-online-record/1"
MAX_RECORD_BYTES = 1 << 20
METRIC = "mean_episode_credit"  # both items' metric (main's, and the graded episode's credit on the same scale)
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


def _no_constants(name):
    raise ValueError("non-finite constant")


def load_json(path, what):
    """A JSON object from a regular file of at most 1 MB, never through a link, or (None, reason)."""
    try:
        st = os.lstat(path)
    except OSError:
        return None, "no {} (the graded episode's service wrote none)".format(what)
    if not stat.S_ISREG(st.st_mode):
        return None, "the {} is not a regular file".format(what)
    if st.st_size > MAX_RECORD_BYTES:
        return None, "the {} exceeds 1 MB".format(what)
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except OSError:
        return None, "the {} cannot be opened".format(what)
    try:
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            return None, "the {} is not a regular file".format(what)
        raw = os.read(fd, MAX_RECORD_BYTES + 1)
    finally:
        os.close(fd)
    if len(raw) > MAX_RECORD_BYTES:
        return None, "the {} exceeds 1 MB".format(what)
    try:
        obj = json.loads(raw.decode("utf-8"), parse_constant=_no_constants)
    except (ValueError, UnicodeDecodeError):
        return None, "the {} is not valid JSON".format(what)
    if not isinstance(obj, dict):
        return None, "the {} is not a JSON object".format(what)
    return obj, None


def ours(rec, online):
    """True for a record of this task's graded episode (format, task, episode and seed digest as tests/online.json)."""
    return all(rec.get(k) == online[k] for k in ("format", "task", "episode", "seed_sha256"))


def check_record(rec, online):
    """None for a record of this task's graded episode that was started, with a finite credit in [0, 1], else the
    reason the graded episode's item scores 0."""
    if not ours(rec, online):
        return "the graded-episode record is not this task's graded episode's"
    if rec.get("started") is not True:
        return "the graded episode was never started"
    score = rec.get("episode_score")
    if isinstance(score, bool) or not isinstance(score, (int, float)) or not math.isfinite(score) or not \
            0.0 <= score <= 1.0:
        return "the graded-episode record has no finite episode_score in [0, 1]"
    return None


def _number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def record_metrics(rec):
    """The record's counters, how the episode ended and the per-episode sub-metrics, as finite numbers."""
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


def snapshot_digest(root):
    """(files, bytes, sha256) of a snapshot tree, computed as the service computes it: sha256 over the lines
    "<path>\\0<size>\\0<sha256 of content>\\n" of its regular files in sorted path order (no link is followed); also
    the number of entries that are neither regular files nor folders (a restored snapshot has none)."""
    listing, odd = [], 0
    for dp, dns, fns in os.walk(root, followlinks=False):
        for dn in list(dns):
            if os.path.islink(os.path.join(dp, dn)):
                odd += 1
                dns.remove(dn)
        for fn in fns:
            fp = os.path.join(dp, fn)
            st = os.lstat(fp)
            if not stat.S_ISREG(st.st_mode):
                odd += 1
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
    return len(listing), sum(x[1] for x in listing), digest.hexdigest(), odd


def freeze_decision(rec, state, online):
    """(trigger, snapshot fields) of the freeze the service recorded, or (None, {}) when nothing was frozen. The freeze
    comes from a usable record (this task's graded episode: started, or not started with freeze prep_window_closed),
    else from the service's state.json (started, or frozen at the window's close); neither file is writable from the
    agent's account."""
    if rec is not None and ours(rec, online) and (rec.get("started") is True or
                                                  rec.get("freeze") == "prep_window_closed"):
        return ("start" if rec.get("started") is True else "prep_window"), {
            k: rec.get(k) for k in ("snapshot_status", "snapshot_files", "snapshot_bytes", "snapshot_sha256",
                                    "snapshot_trigger")}
    if isinstance(state, dict) and (state.get("started") is True or state.get("frozen") == "prep_window_closed"):
        sn = state.get("snapshot") if isinstance(state.get("snapshot"), dict) else {}
        return ("start" if state.get("started") is True else "prep_window"), {
            "snapshot_status": sn.get("status"), "snapshot_files": sn.get("files"), "snapshot_bytes": sn.get("bytes"),
            "snapshot_sha256": sn.get("sha256"), "snapshot_trigger": sn.get("trigger")}
    return None, {}


def check_snapshot(fields, root=SNAPSHOT):
    """None when the freeze is "ok" and the restored snapshot matches its counts and digest, else the reason (the
    controller item then scores 0 without a run)."""
    status = fields.get("snapshot_status")
    if status == "too_large":
        return ("the submission folder was over the controller's caps (5000 files or 200 MiB) when it was frozen: "
                "nothing was copied, as a larger folder scores 0 at grading")
    if status != "ok":
        return "the controller was not frozen (snapshot {})".format(status)
    if not os.path.isdir(root) or os.path.islink(root):
        return "the frozen controller is missing from the artifacts"
    files, size, digest, odd = snapshot_digest(root)
    if odd or (files, size, digest) != (fields.get("snapshot_files"), fields.get("snapshot_bytes"),
                                        fields.get("snapshot_sha256")):
        return "the frozen controller does not match the record's digest"
    return None


def install_snapshot(root, dest):
    """Replace the restored submission folder `dest` with the frozen controller (as root): `dest` is removed without
    following links, then the snapshot's folders and regular files are copied with their modes. Main's grader then
    hands the folder to the agent account, as it does with a restored submission."""
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
    """Main's grader (tests/offline_verify.py, unchanged) on /home/user/submission, in this process: its verdict and
    diagnostic, captured instead of written; it reads main's anchor (tests/anchor_offline.json)."""
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
                         higher_is_better=meta.get("direction", "higher") == "higher", metric_name=METRIC,
                         locks_ok=True, extra=record_metrics(rec), full_at=meta.get("full_at", 1.0))


def main():
    secured = {"root_path_dirs_secured": float(len(secure_exec_path()))}   # before anything else runs
    config = stage_config()
    anchor, meta = read_anchor()
    with open(ONLINE) as fh:
        online = json.load(fh)
    try:
        uid, _gid = _agent_ids()
    except KeyError:
        write_verdict(zero_verdict(reason="no agent account", anchored=anchor is not None, extra=secured))
        return
    kill_agent_processes(uid)
    rec, load_reason = load_json(RECORD, "graded-episode record")
    state, _ = load_json(STATE, "graded episode's state")
    online_reason = load_reason if rec is None else check_record(rec, online)
    if rec is not None and online_reason is None and rec.get("render_error"):
        fail("the cameras' renderer failed during the graded episode ({}): an error of the environment, as in main's "
             "grading".format(str((rec.get("detail") or {}).get("render_error"))[:200]))
    # the controller graded: the one frozen at the earlier of the start and the close of the start window; the
    # session-end submission only when nothing was frozen (the session ended before that) or no usable record of a
    # freeze exists
    trigger, fields = freeze_decision(rec, state, online)
    program, offline_reason = "session_end", None
    if trigger is not None:
        offline_reason = check_snapshot(fields)
        if offline_reason is None:
            install_snapshot(SNAPSHOT, os.path.join(os.path.expanduser("~" + agent_user()), "submission"))
            program = "snapshot"
            log("the controller item grades the copy frozen at {} ({} files, {} bytes)".format(
                "the graded start" if trigger == "start" else "the close of the start window",
                fields.get("snapshot_files"), fields.get("snapshot_bytes")))
    else:
        log("nothing was frozen (no usable record of a freeze): the controller item grades the session-end submission")
    if offline_reason is None:
        off, off_diag = run_offline()
    else:
        log("controller item 0: " + offline_reason)
        off, off_diag = zero_verdict(reason=offline_reason, anchored=True), None
        program = "none"
    on = online_component(rec, online_reason, config)
    cleanup_ok = kill_agent_processes(uid)
    off_r, on_r = float(off["rewards"]["reward"]), float(on["rewards"]["reward"])
    combined = (off_r + on_r) / 2.0
    if not cleanup_ok:
        combined = 0.0
    snap_status = fields.get("snapshot_status")
    extra = {"offline_reward": off_r, "online_reward": on_r, "cleanup_failed": 0.0 if cleanup_ok else 1.0,
             "program_snapshot": 1.0 if program == "snapshot" else 0.0,
             "program_session_end": 1.0 if program == "session_end" else 0.0,
             "snapshot_at_start": 1.0 if trigger == "start" else 0.0,
             "snapshot_at_window_close": 1.0 if trigger == "prep_window" else 0.0,
             "episode_started": 1.0 if (rec or {}).get("started") is True and online_reason is None else 0.0,
             **secured}
    for status in ("ok", "too_large", "missing", "failed"):
        extra["snapshot_" + status] = 1.0 if snap_status == status else 0.0
    for key in ("snapshot_files", "snapshot_bytes"):
        if _number(fields.get(key)):
            extra[key] = float(fields[key])
    for reason in END_REASONS:
        extra["online_end_" + reason] = 1.0 if (rec or {}).get("end_reason") == reason else 0.0
    for key, v in sorted((off.get("metrics") or {}).items()):
        extra["offline_" + key] = v
    for key, v in sorted((on.get("metrics") or {}).items()):
        extra["online_" + key] = v
    verdict = ratio_rewards(combined, anchor, cap=1.0, higher_is_better=True, metric_name="dual_mode_score",
                            locks_ok=True, extra=extra, full_at=meta.get("full_at", 1.0))
    diagnostic = ("dual mode: controller {} (frozen at {}: {}; reward {:.4f}), graded episode {} (reward {:.4f}; ended "
                  "{}), mean {:.4f}; controller item: {}; graded episode item: {}").format(
        program, trigger, snap_status, off_r, (rec or {}).get("episode"), on_r, (rec or {}).get("end_reason"),
        combined, off_diag or off.get("diagnostic") or "graded", on.get("diagnostic") or "scored")
    write_verdict(verdict, diagnostic=diagnostic)
    log("{} anchor={} reward={:.3f}".format(diagnostic, anchor, verdict["rewards"]["reward"]))


if __name__ == "__main__":
    main()
