"""Grader of shoe_lacing_and_boxing, DUAL MODE (experimental): one verify stage for the task's two scoring items. Runs
as ROOT in a separate verifier sandbox (task.toml), with the declared artifacts restored: the agent's submission folder
and the graded service's record directory (/var/lib/arb-online: the record, and the copy of the submission folder the
service froze when the graded episode started).

The two items, each normalised by its own reference value:
  * the program item (offline): main's grading, unchanged. offline_verify.py is main's tests/verify.py byte for byte;
    it runs as its own program in a copy of this folder laid out as main's (offline_verify.py as verify.py,
    anchor_offline.json as anchor.json), so main's verify-start lockdown, gates, network audit, hidden set
    (hidden/seeds.json), budgets, oracle token and reference value apply as they are. Its verdict (verdict.local.json in
    that copy) gives offline_reward.
  * the graded-episode item (online): the service's record, checked against online.json, its episode_score related to
    anchor_online.json (0 when the episode was never started or the record is unusable).
reward = (offline_reward + online_reward) / 2, written once through ratio_rewards against anchor.json (dual_mode_score,
direction higher, value 1.0, cap 1.0, full_at 1.0).

Order:
  1. every process of the agent account stopped as main's verifier starts (cellkit.lockdown.stop_and_kill_uid, then the
     account freezer), stdlib only;
  2. the record read and checked (format, task, episode and seed digest as online.json, started, a finite score);
  3. the program graded: if the graded episode started, the copy frozen at the start replaces the restored
     /home/user/submission (status ok: checked file by file against the record's snapshot_sha256; too_large: an empty
     folder, so the program item scores 0, as main's 60 MB route gate would); if it never started (no usable record),
     the restored submission stays: the folder as the session left it, as on main;
  4. main's grading of that submission (the offline item); a task error there is the verify's task error;
  5. the online item; 6. the mean, one verdict.
"""

import hashlib
import json
import math
import os
import pwd
import shutil
import stat
import subprocess
import sys

sys.dont_write_bytecode = True

STAGE_DIR = os.environ.get("ALE_STAGE_DIR", os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, STAGE_DIR)

from robotics_grader import ratio_rewards, read_anchor, write_verdict  # noqa: E402  (stdlib only)

HOME = os.environ.get("ALE_HOME", "/home/user")
AGENT_USER = os.path.basename(HOME.rstrip("/")) or "user"
SUBMISSION = os.path.join(HOME, "submission")
RECORD_DIR = os.environ.get("ARB_ONLINE_RECORD_DIR", "/var/lib/arb-online")
RECORD = os.path.join(RECORD_DIR, "record.json")
FROZEN = os.path.join(RECORD_DIR, "submission_snapshot")
OFFLINE_ROOT = os.environ.get("ARB_OFFLINE_STAGE_ROOT", "/opt/arb-offline")   # main's stage, rebuilt here (root 0700)
MAX_RECORD_BYTES = 1 << 20
END_REASONS = ("done", "finish", "graded_window", "worker_restart")
SNAPSHOT_STATUSES = ("ok", "too_large")
FREEZER = [None]


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def fail(reason):
    """A task error (the grading broke), never the agent's zero: no verdict, exit 1 (as main's verify.py does)."""
    log("[grader] TASK ERROR: " + reason)
    raise SystemExit(1)


def _number(v):
    """A finite float from a JSON number (not a bool), else None."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    v = float(v)
    return v if math.isfinite(v) else None


def _metrics(prefix, mapping):
    """`mapping`'s finite numbers under `prefix`, '.' and '/' in a name replaced by '_' (main's filter)."""
    out = {}
    for k, v in sorted((mapping or {}).items()):
        v = _number(v)
        if v is not None and isinstance(k, str) and len(k) <= 80:
            out[prefix + k.replace(".", "_").replace("/", "_")] = v
    return out


# ------------------------------------------------------------------------------------------------ 1. the agent stopped

def stop_agent():
    """Main's verify-start kill and freeze of the agent account (in the separate verifier: the image's own session of
    that account); stopped again, with main's own checks, by main's grader in step 4."""
    if os.getuid() != 0:
        return {}
    from cellkit import lockdown, verify_freeze
    try:
        uid = pwd.getpwnam(AGENT_USER).pw_uid
    except KeyError:
        return {}
    if uid == 0:
        return {}
    killed, left = lockdown.stop_and_kill_uid(uid, grace_s=8.0)
    FREEZER[0] = verify_freeze.AgentFreezer.for_user(AGENT_USER).start()
    return {"agent_processes_killed": float(killed), "agent_processes_left": float(left)}


# ------------------------------------------------------------------------------------------------ 2. the record

def load_record(online):
    """(record, None) when the record is the graded episode's and usable, else (None, why)."""
    try:
        st = os.lstat(RECORD)
    except FileNotFoundError:
        return None, "no record: the graded episode was never started"
    except OSError as e:
        return None, "the record cannot be read: %s" % e
    if not stat.S_ISREG(st.st_mode):
        return None, "the record is not a regular file"
    if st.st_size > MAX_RECORD_BYTES:
        return None, "the record is larger than 1 MB"
    try:
        fd = os.open(RECORD, os.O_RDONLY | os.O_NOFOLLOW)
        with os.fdopen(fd, "rb") as fh:
            blob = fh.read(MAX_RECORD_BYTES + 1)
        rec = json.loads(blob)
    except (OSError, ValueError, RecursionError) as e:
        return None, "the record is not valid JSON: %s" % type(e).__name__
    if not isinstance(rec, dict):
        return None, "the record is not a JSON object"
    for key in ("format", "task", "episode", "seed_sha256"):
        if rec.get(key) != online[key]:
            return None, "the record's %s is %r, not %r" % (key, rec.get(key), online[key])
    if rec.get("started") is not True:
        return None, "the record says the graded episode never started"
    score = _number(rec.get("episode_score"))
    if score is None or score < 0.0:
        return None, "the record's episode_score is not a finite number >= 0"
    return rec, None


def record_stats(rec):
    """The record's statistics as finite floats for the verdict's metrics (nothing here gates)."""
    out = {"record_present": 1.0, "record_ended": 1.0 if rec.get("ended") is True else 0.0}
    reason = rec.get("end_reason")
    for r in END_REASONS:
        out["end_reason_" + r] = 1.0 if reason == r else 0.0
    out["end_reason_none"] = 1.0 if reason is None else 0.0
    for key in ("requests", "steps", "sim_time_s", "wall_s_since_start"):
        v = _number(rec.get(key))
        if v is not None:
            out[key] = v
    v = _number(rec.get("seq"))
    if v is not None:
        out["record_seq"] = v
    metrics = rec.get("metrics") if isinstance(rec.get("metrics"), dict) else {}
    out.update(_metrics("episode_", {k: v for k, v in metrics.items() if k != "work_score"}))
    return out


# ------------------------------------------------------------------------------------------------ 3. the program graded

def tree_digest(root):
    """(digest, files, bytes) of the regular files under `root`: the service's snapshot_sha256 (the sha256 of the sorted
    "path NUL size NUL sha256(content)" lines, environment/setup/online/server.py snapshot_digest)."""
    entries, total = [], 0
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        dirnames.sort()
        for fn in filenames:
            p = os.path.join(dirpath, fn)
            st = os.lstat(p)
            if not stat.S_ISREG(st.st_mode):
                continue
            h = hashlib.sha256()
            with open(p, "rb") as fh:
                for block in iter(lambda: fh.read(1 << 20), b""):
                    h.update(block)
            entries.append((os.path.relpath(p, root).replace(os.sep, "/"), st.st_size, h.hexdigest()))
            total += st.st_size
    h = hashlib.sha256()
    for rel, size, digest in sorted(entries):
        h.update(("%s\0%d\0%s\n" % (rel, size, digest)).encode("utf-8"))
    return h.hexdigest(), len(entries), total


def _replace_submission(src):
    """The restored /home/user/submission replaced by a root-owned copy of `src` (main's copy: regular files and folders
    only), or by an empty folder when `src` is None."""
    from cellkit import runner
    try:
        st = os.lstat(SUBMISSION)
        if stat.S_ISDIR(st.st_mode):
            shutil.rmtree(SUBMISSION)
        else:
            os.unlink(SUBMISSION)
    except FileNotFoundError:
        pass
    if src is None:
        os.makedirs(SUBMISSION)
    else:
        runner.copy_regular_files(src, SUBMISSION)


def program_graded(rec):
    """Which submission the program item grades (step 3); returns (label, stats). A started episode always had its
    folder frozen (the service refuses a start whose copy failed): a frozen copy that is missing or differs from the
    record's digest is a task error."""
    if rec is None:
        return "session_end", {"program_session_end": 1.0}
    status = rec.get("snapshot_status")
    if status not in SNAPSHOT_STATUSES:
        fail("the record of a started episode has no freeze status (%r)" % (status,))
    if status == "too_large":
        _replace_submission(None)
        return "too_large", {"program_too_large": 1.0}
    if not os.path.isdir(FROZEN) or os.path.islink(FROZEN):
        fail("the frozen copy of the submission (%s) did not arrive" % FROZEN)
    digest, files, nbytes = tree_digest(FROZEN)
    if (digest, files, nbytes) != (rec.get("snapshot_sha256"), rec.get("snapshot_files"), rec.get("snapshot_bytes")):
        fail("the frozen copy of the submission differs from the record (%d files, %d bytes, %s; the record: %r files, "
             "%r bytes, %s)" % (files, nbytes, digest[:16], rec.get("snapshot_files"), rec.get("snapshot_bytes"),
                                str(rec.get("snapshot_sha256"))[:16]))
    _replace_submission(FROZEN)
    return "frozen_at_start", {"program_frozen_at_start": 1.0, "program_files": float(files), "program_bytes": float(nbytes)}


# ------------------------------------------------------------------------------------------------ 4. main's grading

def offline_stage():
    """A root-only copy of this folder laid out as main's tests/: offline_verify.py as verify.py, anchor_offline.json as
    anchor.json, the dual mode's own files left out. Outside /opt/ale (the engine clears its stage folders)."""
    if os.path.lexists(OFFLINE_ROOT):
        shutil.rmtree(OFFLINE_ROOT)
    os.makedirs(OFFLINE_ROOT, mode=0o700)
    os.chmod(OFFLINE_ROOT, 0o700)
    stage = os.path.join(OFFLINE_ROOT, "tests")
    shutil.copytree(STAGE_DIR, stage, symlinks=False,
                    ignore=shutil.ignore_patterns("__pycache__", "verdict.local.json", "verdict.json"))
    os.replace(os.path.join(stage, "offline_verify.py"), os.path.join(stage, "verify.py"))
    os.replace(os.path.join(stage, "anchor_offline.json"), os.path.join(stage, "anchor.json"))
    for name in ("anchor_online.json", "online.json"):
        os.remove(os.path.join(stage, name))
    return stage


def offline_item():
    """Main's verify.py run as its own program on the submission in place; (offline_reward, its metrics, its text)."""
    stage = offline_stage()
    env = dict(os.environ, ALE_STAGE_DIR=stage)
    for k in ("ALE_VERIFY_PYTHON", "ARB_STAGE_MODE", "ARB_STAGE_PARENT_MODE", "ROBOTICS_GRADER_VERDICT_PATH"):
        env.pop(k, None)               # main's grader writes its envelope to verdict.local.json in its own stage
    log("[grader] the program item: main's grading (%s)" % os.path.join(stage, "verify.py"))
    r = subprocess.run([sys.executable, "-s", os.path.join(stage, "verify.py")], cwd=stage, env=env)
    if r.returncode != 0:
        fail("main's grading of the program ended with exit status %d (its own TASK ERROR above)" % r.returncode)
    try:
        with open(os.path.join(stage, "verdict.local.json")) as fh:
            verdict = json.load(fh)
        reward = float(verdict["rewards"]["reward"])
        metrics = verdict.get("metrics") or {}
    except (OSError, ValueError, KeyError, TypeError) as e:
        fail("main's grading wrote no usable verdict: %s" % e)
    if not (0.0 <= reward <= 1.0):
        fail("main's grading returned reward %r" % reward)
    return reward, metrics


# ------------------------------------------------------------------------------------------------ 5.-6. the score

def online_item(rec, why):
    """(online_reward, its stats, text): the record's episode_score against anchor_online.json (0 without a record)."""
    anchor, meta = read_anchor("anchor_online.json")
    if rec is None:
        return 0.0, {"online_record_present": 0.0 if why and why.startswith("no record") else 1.0}, "online: " + why
    score = float(rec["episode_score"])
    v = ratio_rewards(score, anchor, higher_is_better=meta.get("direction", "higher") == "higher",
                      metric_name=meta.get("metric", "work_score"), locks_ok=True, full_at=meta.get("full_at", 1.0))
    m = rec.get("metrics") if isinstance(rec.get("metrics"), dict) else {}
    ms = ", ".join("%s %.2f" % (k, _number(m.get("milestone_" + k)) or 0.0)
                   for k in ("lacing_a", "lacing_b", "erected", "packed", "closed", "shipped"))
    text = ("online: graded episode %s: work_score %.4f (%s; safety_ok %s); ended %s (%s) at %.1f s simulated after %s "
            "requests (%s steps)") % (rec["episode"], score, ms, bool(_number(m.get("safety_ok"))), rec.get("ended") is True,
                                      rec.get("end_reason"), _number(rec.get("sim_time_s")) or 0.0, rec.get("requests"),
                                      rec.get("steps"))
    stats = dict(record_stats(rec), online_work_score=score, online_ratio=float(v["metrics"]["ratio"]))
    return float(v["rewards"]["reward"]), {"online_" + k if not k.startswith("online_") else k: x for k, x in stats.items()}, text


def _restore_modes():
    """The stage folder and its parent back to the modes tests/test.sh recorded (main's verify.py does the same)."""
    for path, var in ((STAGE_DIR, "ARB_STAGE_MODE"), (os.path.dirname(STAGE_DIR), "ARB_STAGE_PARENT_MODE")):
        mode = os.environ.get(var)
        if mode and path not in ("/", os.path.abspath(HOME)):
            try:
                os.chmod(path, int(mode, 8))
            except (OSError, ValueError):
                pass


def _write(verdict, text):
    if os.environ.get("ALE_VERIFY_PYTHON"):
        write_verdict(verdict, diagnostic=text)
    else:
        write_verdict(verdict, path=os.environ.get("ROBOTICS_GRADER_VERDICT_PATH") or os.path.join(STAGE_DIR, "verdict.local.json"),
                      diagnostic=text)


def main():
    try:
        early = stop_agent()                                    # 1
        with open(os.path.join(STAGE_DIR, "online.json")) as fh:
            online = json.load(fh)
        rec, why = load_record(online)                          # 2
        if why:
            log("[grader] online: " + why)
        label, program_stats = program_graded(rec)              # 3
        log("[grader] the program item grades the submission %s" % {
            "session_end": "as the session left it (the graded episode was never started)",
            "too_large": "frozen at the start: over the 60 MB cap, so none (the program item scores 0)",
            "frozen_at_start": "frozen when the graded episode started"}[label])
        if FREEZER[0] is not None:
            FREEZER[0].stop()                                   # main's grader runs its own lockdown and freezer now
        offline_reward, offline_metrics = offline_item()        # 4
        online_reward, online_stats, online_text = online_item(rec, why)   # 5
        combined = (offline_reward + online_reward) / 2.0       # 6
        anchor, meta = read_anchor()
        if meta.get("direction", "higher") != "higher" or meta.get("metric") != "dual_mode_score":
            fail("tests/anchor.json is not the dual mode's (dual_mode_score, higher)")
        extra = {"offline_reward": offline_reward, "online_reward": online_reward}
        if _number(offline_metrics.get("work_score")) is not None:
            extra["offline_work_score"] = float(offline_metrics["work_score"])
        extra.update(_metrics("offline_", {k: v for k, v in offline_metrics.items() if k != "work_score"}))
        extra.update(online_stats)
        extra.update(program_stats)
        extra.update({k: float(v) for k, v in early.items()})
        verdict = ratio_rewards(combined, anchor, cap=1.0, higher_is_better=True, metric_name="dual_mode_score",
                                locks_ok=True, extra=extra, full_at=meta.get("full_at", 1.0))
        text = ("dual_mode_score %.4f = (offline %.4f + online %.4f) / 2; program item: work_score %s (contract_ok %s) "
                "on the submission %s; %s") % (
            combined, offline_reward, online_reward, offline_metrics.get("work_score"), offline_metrics.get("contract_ok"),
            label.replace("_", " "), online_text)
        log("[grader] " + text)
        _write(verdict, text)
    finally:
        if FREEZER[0] is not None:
            FREEZER[0].stop()
        _restore_modes()


if __name__ == "__main__":
    main()
