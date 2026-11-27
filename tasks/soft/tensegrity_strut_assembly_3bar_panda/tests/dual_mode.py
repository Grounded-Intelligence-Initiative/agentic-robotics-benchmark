"""The dual mode of tensegrity_strut_assembly_3bar_panda (experimental; never merged into main): one verify for the
two scoring items, reward = their mean. stdlib only at import (verify.py calls prepare() before numpy is imported).

verify.py is main's tests/verify.py with four documented hooks into this module (its docstring lists them); main's
grading itself is unchanged. In this order:
  1. prepare(), at verify start, before main's lockdown (verify.py: `DUAL = dual_mode.prepare(...)`, the line before
     `EARLY = _early_lockdown()`), as root:
     * stops the agent account's processes as main's grader does at verify start (lockdown.stop_and_kill_uid; main's
       lockdown repeats it right after, with its freezer that keeps the account stopped until the verdict);
     * reads the graded episode's record (online_record.read_record, against tests/online.json);
     * chooses the program the offline item grades (the freeze, DM3): the copy frozen at the earlier of the graded start
       and the start window's close (/var/lib/arb-online/submission_snapshot; its file count and digest checked against
       the record, a mismatch is a task error) takes the place of the restored /home/user/submission, root-owned; over
       main's caps when it froze (too_large), or not frozen when the start window closed (the freeze failed), nothing
       of it is graded: an empty submission folder, so main's grading gives contract_ok 0 and the offline item 0 (never
       the session-end submission); the submission the engine restored, the session's end, is graded as on main only
       when the session ended before the start window closed, without a start, or when there is no usable record. The
       restored submission that is replaced is moved aside into the maintainers' record (/opt/arb-verify/dual/);
     * scores the online item (online_record.online_verdict, against tests/anchor_online.json).
  2. main's lockdown and grading, unchanged (verify.py's main()): its verdict (robotics_grader.ratio_rewards of
     assembly_score against main's anchor, now tests/anchor_offline.json, byte-identical to main's tests/anchor.json) is
     handed to keep_offline() instead of being written (verify.py write_verdict's two hooks).
  3. run() writes the task's one verdict: reward = (offline reward + online reward) / 2, each the item's unrounded
     reward against its own anchor, cap and full_at; written once through robotics_grader.ratio_rewards against
     tests/anchor.json (dual_mode_score, 1.0 by construction), both items' metrics prefixed offline_ / online_.
Outside a dual run (main's functions imported by the repository's tests) keep_offline() returns False and main's
write_verdict writes as on main.
"""

import json
import math
import os
import pwd
import shutil
import stat as statmod
import sys
import time

import online_record

STAGE_DIR = os.environ.get("ALE_STAGE_DIR", os.path.dirname(os.path.abspath(__file__)))
ONLINE = os.path.join(STAGE_DIR, "online.json")
OFFLINE_ANCHOR = "anchor_offline.json"      # main's tests/anchor.json, byte-identical
RECORD_ROOT = os.environ.get("ARB_RECORD_ROOT", "/opt/arb-verify")
ASIDE = os.path.join(RECORD_ROOT, "dual", "session_end_submission")

STATE = {"active": False, "mode": None, "frozen": {}, "online": None, "offline": None, "record": None,
         "agent_processes_killed": 0.0, "agent_processes_left": 0.0}


def log(msg):
    print("[grader] dual mode: " + msg, file=sys.stderr, flush=True)


def fail(reason):
    log("TASK ERROR: " + reason)
    raise SystemExit(1)


def _choose(rec):
    """(mode, info): the program the offline item grades. `rec` is the record of the graded episode, started or its
    start window closed (read_record), or None.
      "snapshot": the record says the program was frozen (at the start or at the window's close) and the copy is whole:
          the frozen copy (its file count and digest checked against the record; a mismatch is a task error);
      "zero": the program was frozen over main's caps (too_large: nothing was copied), or the window closed and the
          program could not be frozen: nothing of it is graded, the offline item scores 0;
      "session_end": the session ended before the start window closed, without a start, or there is no usable record:
          the submission the engine restored, as on main."""
    frozen = rec is not None and (rec.get("started") is True or rec.get("prep_window_closed") is True)
    if not frozen:
        return "session_end", {"status": "not_frozen", "reason": (
            "no usable record of the graded episode" if rec is None else
            "the session ended before the start window closed, without a start") + (
            ": the program in the submission folder at the session's end is graded, as on main")}
    at = rec.get("snapshot_at") or ("graded_start" if rec.get("started") is True else "prep_window")
    status = rec.get("snapshot_status")
    if status == "too_large":
        return "zero", {"status": "too_large", "at": at, "reason": "the program in /home/user/submission was beyond "
                        "main's submission caps when it froze (at %s): nothing of it is graded" % at}
    if status != "ok":
        if at == "prep_window" and rec.get("started") is not True:
            return "zero", {"status": str(status), "at": at, "reason": "the program could not be frozen when the start "
                            "window closed (%s): nothing of it is graded" % (rec.get("snapshot_detail") or status)}
        fail("the record says the graded episode started but holds no frozen program (snapshot_status %r)" % (status,))
    files, size, digest = online_record.snapshot_digest()
    if digest != rec.get("snapshot_sha256") or files != rec.get("snapshot_files"):
        fail("the frozen program (%d files, sha256 %s) does not match its record (%r files, sha256 %r)" % (
            files, digest, rec.get("snapshot_files"), rec.get("snapshot_sha256")))
    return "snapshot", {"status": "ok", "at": at, "files": files, "bytes": size, "sha256": digest,
                        "reason": "the program frozen at %s (%d files, %d bytes) is graded" % (at, files, size)}


def _replace_submission(submission, src):
    """The restored submission folder is moved aside (the maintainers' record) and a root-owned folder takes its place:
    a copy of the frozen program `src` (its directories 0700 and regular files 0600; nothing else is in it), or empty
    (`src` None)."""
    os.makedirs(os.path.dirname(ASIDE), mode=0o700, exist_ok=True)
    os.chmod(RECORD_ROOT, 0o700)
    if os.path.lexists(ASIDE):
        shutil.rmtree(ASIDE, ignore_errors=True)
    if os.path.lexists(submission):
        if os.path.isdir(submission) and not os.path.islink(submission):
            try:
                os.rename(submission, ASIDE)
            except OSError:
                shutil.rmtree(submission)
        else:
            os.unlink(submission)
    os.mkdir(submission, 0o700)
    if src is None:
        return
    for dirpath, dirnames, filenames in os.walk(src, followlinks=False):
        rel = os.path.relpath(dirpath, src)
        target = submission if rel == "." else os.path.join(submission, rel)
        keep = []
        for d in sorted(dirnames):
            p = os.path.join(dirpath, d)
            if os.path.islink(p) or not os.path.isdir(p):
                continue
            os.mkdir(os.path.join(target, d), 0o700)
            keep.append(d)
        dirnames[:] = keep
        for f in sorted(filenames):
            p = os.path.join(dirpath, f)
            if not statmod.S_ISREG(os.lstat(p).st_mode):
                continue
            shutil.copyfile(p, os.path.join(target, f), follow_symlinks=False)
            os.chmod(os.path.join(target, f), 0o600)


def prepare(submission, agent_user):
    """Step 1 (see the module's text). Returns the module's state (verify.py keeps it as DUAL)."""
    STATE["active"] = True
    if os.getuid() == 0:
        from arm.assembly_six import lockdown
        try:
            uid = pwd.getpwnam(agent_user).pw_uid
        except KeyError:
            uid = None
        if uid is not None and uid != 0:
            killed, left = lockdown.stop_and_kill_uid(uid, grace_s=8.0)
            STATE["agent_processes_killed"], STATE["agent_processes_left"] = float(killed), float(left)
    with open(ONLINE) as fh:
        online = json.load(fh)
    rec_any, why_any = online_record.read_record(online)
    if rec_any is None:
        log("the record: " + why_any)
    mode, frozen = _choose(rec_any)
    STATE.update(mode=mode, frozen=frozen, record=rec_any)
    if os.getuid() == 0 and mode != "session_end":
        _replace_submission(submission, online_record.SNAPSHOT_DIR if mode == "snapshot" else None)
    log("%s (%s)" % (frozen["reason"], mode))
    rec, why = online_record.load_record(online)
    STATE["online"] = online_record.online_verdict(rec, why)
    STATE["online_started"] = rec is not None
    return STATE


def keep_offline(verdict, diagnostic):
    """verify.py write_verdict's hook: in a dual run main's verdict (its envelope as main builds it) is kept for run()
    instead of written; True when kept."""
    if not STATE["active"]:
        return False
    STATE["offline"] = {"verdict": verdict, "diagnostic": diagnostic}
    return True


def _finite_metrics(metrics):
    """main's filter of the engine's record: finite numbers only, '.' and '/' in a name replaced by '_'."""
    out = {}
    for k, v in sorted((metrics or {}).items()):
        try:
            v = float(v)
        except (TypeError, ValueError):
            continue
        if math.isfinite(v):
            out[str(k).replace(".", "_").replace("/", "_")] = v
    return out


def run(main):
    """Steps 2 and 3: main's grading (its verdict kept), then the task's one verdict, the mean of both items."""
    t0 = time.time()
    if not STATE["active"]:
        prepare(os.path.join(os.environ.get("ALE_HOME", "/home/user"), "submission"),
                os.path.basename(os.environ.get("ALE_HOME", "/home/user").rstrip("/")) or "user")
    main()
    if STATE["offline"] is None:
        fail("main's grading wrote no verdict")
    from robotics_grader import ratio_rewards, read_anchor
    from robotics_grader import write_verdict as kit_write_verdict
    off_v, on_v = STATE["offline"]["verdict"], STATE["online"]
    off_r, on_r = float(off_v["rewards"]["reward"]), float(on_v["rewards"]["reward"])
    combined = (off_r + on_r) / 2.0
    anchor, meta = read_anchor()
    frozen, mode = STATE["frozen"], STATE["mode"]
    extra = {"offline_reward": off_r, "online_reward": on_r,
             "offline_assembly_score": float((off_v.get("metrics") or {}).get("assembly_score", 0.0)),
             "online_assembly_score": float(on_v.get("assembly_score", 0.0)),
             "snapshot_ok": 1.0 if frozen.get("status") == "ok" else 0.0,
             "snapshot_too_large": 1.0 if frozen.get("status") == "too_large" else 0.0,
             "snapshot_files": float(frozen.get("files", 0) or 0), "snapshot_bytes": float(frozen.get("bytes", 0) or 0),
             "offline_graded_frozen_program": 1.0 if mode == "snapshot" else 0.0,
             "offline_graded_nothing": 1.0 if mode == "zero" else 0.0,
             "offline_graded_session_end": 1.0 if mode == "session_end" else 0.0,
             "snapshot_at_graded_start": 1.0 if frozen.get("at") == "graded_start" else 0.0,
             "snapshot_at_prep_window": 1.0 if frozen.get("at") == "prep_window" else 0.0,
             "online_started": 1.0 if STATE.get("online_started") else 0.0,
             "dual_agent_processes_killed": STATE["agent_processes_killed"],
             "dual_agent_processes_left": STATE["agent_processes_left"]}
    for prefix, metrics in (("offline_", off_v.get("metrics")), ("online_", on_v.get("metrics"))):
        for k, v in _finite_metrics(metrics).items():
            if prefix + k not in extra:
                extra[prefix + k] = v
    verdict = ratio_rewards(combined, anchor, cap=1.0, higher_is_better=True,
                            metric_name=meta.get("metric", "dual_mode_score"), locks_ok=True, extra=extra,
                            full_at=meta.get("full_at", 1.0))
    text = ("dual_mode_score %.4f = (offline item %.4f + online item %.4f) / 2. Offline item (%s): %s. Online item: %s"
            % (combined, off_r, on_r, frozen.get("reason"), STATE["offline"].get("diagnostic"),
               on_v.get("diagnostic") or "scored"))
    log(text)
    if os.environ.get("ALE_VERIFY_PYTHON"):
        kit_write_verdict({"rewards": verdict["rewards"], "metrics": _finite_metrics(verdict["metrics"])},
                          diagnostic=text)
    else:
        kit_write_verdict(verdict, path=os.environ.get("ROBOTICS_GRADER_VERDICT_PATH")
                          or os.path.join(STAGE_DIR, "verdict.local.json"), diagnostic=text)
    log("verdict written (%.0f s after main's grading began)" % (time.time() - t0))
    return combined
