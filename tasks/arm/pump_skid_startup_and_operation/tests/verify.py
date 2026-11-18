"""Grader of pump_skid_startup_and_operation, dual mode (experiment): runs as ROOT in the verify stage, in a separate
verifier sandbox (task.toml). One task, two items, one verdict:

  the program item    main's offline grading, unchanged (offline_verify.py: main's tests/verify.py with two hooks): the
                      agent's program on the 8 hidden sites of hidden/seeds.json, with main's budgets, gates, probe of
                      the network and rerun of an episode in which the cell failed, against main's anchor
                      (anchor_offline.json);
  the online item     the graded episode the agent played itself, live, through the graded episode's service
                      (environment/setup/online/graded_service.py), scored from its record
                      (/var/lib/arb-online/record.json) against the online anchor (anchor_online.json);
  the reward          the mean of the two items' rewards, each normalised by its own anchor, cap and full_at, written
                      once through ratio_rewards against anchor.json (dual_mode_score, 1.0 by construction).

The freeze: the program graded is the copy of /home/user/submission that the graded episode's root supervisor made at
the graded start (/var/lib/arb-online/submission_snapshot/, out of the agent's reach), checked against the record's
snapshot_sha256; if the episode never started, the restored session-end submission, as on main. In this order:
  1. stop the agent account's processes (cellkit.lockdown, as main's grader does; main's grader does it once more, and
     freezes them until its verdict, when it starts);
  2. read and check the record (as the online variant's grader: a regular file of at most 1 MB, strict JSON, format,
     task, episode and seed digest equal to online.json, started true, a finite episode_score >= 0);
  3. when the episode started (the record, or the service's state.json, says so): the snapshot must be status ok and
     match its digest, and it then replaces the restored /home/user/submission (root-owned, as every restored entry
     is, so main's late-write gate passes it); a snapshot that is too_large, missing or altered scores the program item
     0 without a run. When it never started, the restored submission stays;
  4. main's offline grading on /home/user/submission, as a process of its own (its verdict goes to a root-only file; a
     task error there is a task error here);
  5. the online item from the record (0 without a valid, started record);
  6. reward = (offline_reward + online_reward) / 2, through ratio_rewards once; the metrics carry both items (prefixes
     offline_ and online_), the snapshot's status and the online end reason.
Untouched sandbox: no record, an empty submission -> 0 + 0. Oracle: the reference program frozen at the start (main's
oracle) and the privileged reference on the graded episode -> 1 + 1.
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
import time

sys.dont_write_bytecode = True

STAGE_DIR = os.environ.get("ALE_STAGE_DIR", os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, STAGE_DIR)

from cellkit import lockdown  # noqa: E402  (stdlib only: main's)
from robotics_grader import ratio_rewards, read_anchor  # noqa: E402
from robotics_grader import write_verdict as kit_write_verdict  # noqa: E402

HOME = os.environ.get("ALE_HOME", "/home/user")
AGENT_USER = os.path.basename(HOME.rstrip("/")) or "user"
SUBMISSION = os.path.join(HOME, "submission")
ONLINE_DIR = os.environ.get("ARB_ONLINE_DIR", "/var/lib/arb-online")
SNAPSHOT = os.path.join(ONLINE_DIR, "submission_snapshot")
WORK = os.environ.get("ARB_DUAL_WORK") or ("/opt/arb-verify/dual" if os.getuid() == 0 else os.path.join(STAGE_DIR, "dual"))
RECORD_MAX_BYTES = 1 << 20
END_REASONS = ("done", "finish", "graded_window", "worker_restart")


def log(msg):
    print(msg, file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------------------------- 1. the agent's processes
def stop_agent():
    if os.getuid() != 0:
        return 0, 0
    try:
        uid = pwd.getpwnam(AGENT_USER).pw_uid
    except KeyError:
        return 0, 0
    if uid == 0:
        return 0, 0
    return lockdown.stop_and_kill_uid(uid, grace_s=8.0)


# ---------------------------------------------------------------------------------------------- 2. the record
def _reject_constant(name):
    raise ValueError("non-finite number %s" % name)


def read_json_file(path, cap=RECORD_MAX_BYTES):
    """(object, None) or (None, why)."""
    try:
        st = os.lstat(path)
    except FileNotFoundError:
        return None, "missing"
    except OSError as e:
        return None, "unreadable (%s)" % e
    if not stat.S_ISREG(st.st_mode):
        return None, "not a regular file"
    if st.st_size > cap:
        return None, "larger than 1 MB"
    try:
        with open(path, "rb") as f:
            obj = json.loads(f.read(cap + 1).decode("utf-8"), parse_constant=_reject_constant)
    except (OSError, ValueError, UnicodeDecodeError) as e:
        return None, "not valid JSON (%s)" % str(e)[:160]
    if not isinstance(obj, dict):
        return None, "not a JSON object"
    return obj, None


def check_record(rec, online):
    for k in ("format", "task", "episode", "seed_sha256"):
        if rec.get(k) != online[k]:
            return "the record's %s (%r) is not this task's graded episode's (%r)" % (k, rec.get(k), online[k])
    if rec.get("started") is not True:
        return "the record says the graded episode was never started"
    v = rec.get("episode_score")
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0:
        return "the record's episode_score (%r) is not a finite number >= 0" % (v,)
    return None


# ---------------------------------------------------------------------------------------------- 3. the freeze
def snapshot_digest(entries):
    """The graded episode's service's digest (graded_service.snapshot_digest): the sorted (path, size, sha256) list."""
    h = hashlib.sha256()
    for rel, size, sha in sorted(entries):
        h.update(rel.encode("utf-8", "surrogateescape") + b"\0" + str(int(size)).encode() + b"\0" + sha.encode() + b"\n")
    return h.hexdigest()


def snapshot_listing(root):
    """(entries, n_files, n_bytes, problems) of the restored snapshot: regular files and folders only."""
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


def place_snapshot(src, dst):
    """Replace the restored submission folder with a root-owned copy of the snapshot (regular files and folders)."""
    if os.path.islink(dst) or (os.path.lexists(dst) and not os.path.isdir(dst)):
        os.unlink(dst)
    elif os.path.isdir(dst):
        shutil.rmtree(dst)
    os.makedirs(dst, mode=0o755)
    for dirpath, dirnames, filenames in os.walk(src, followlinks=False):
        rel = os.path.relpath(dirpath, src)
        target = dst if rel == "." else os.path.join(dst, rel)
        for name in dirnames:
            os.makedirs(os.path.join(target, name), mode=0o755, exist_ok=True)
        for name in filenames:
            p = os.path.join(dirpath, name)
            if os.path.isfile(p) and not os.path.islink(p):
                shutil.copyfile(p, os.path.join(target, name))
                os.chmod(os.path.join(target, name), 0o644)
    os.chmod(dst, 0o755)


def record_state(rec, why, online):
    """missing | invalid (unreadable, or not this task's graded episode) | not_started | started."""
    if rec is None:
        return "missing" if why == "missing" else "invalid"
    if any(rec.get(k) != online[k] for k in ("format", "task", "episode", "seed_sha256")):
        return "invalid"
    return "started" if rec.get("started") is True else "not_started"


def freeze_decision(rec, rstate, state):
    """(started, snapshot fields, None) or (started, fields, why the program item scores 0 without a run). The episode
    started when its record or the service's state.json says so; a record that is not this task's graded episode's
    leaves the freeze unknown, and the program item then scores 0 (it never comes from the agent: neither file is
    writable from the agent's account)."""
    if rstate == "invalid":
        return True, {}, "the graded-episode record is invalid, so the freeze cannot be checked"
    started = rstate == "started" or bool(isinstance(state, dict) and state.get("started"))
    if not started:
        return False, {}, None
    if rstate == "started":
        fields = {k: rec.get(k) for k in ("snapshot_status", "snapshot_files", "snapshot_bytes", "snapshot_sha256")}
    else:
        sn = (state or {}).get("snapshot") or {}
        fields = {"snapshot_status": sn.get("status"), "snapshot_files": sn.get("files"), "snapshot_bytes": sn.get("bytes"),
                  "snapshot_sha256": sn.get("sha256")}
    status = fields.get("snapshot_status")
    if status == "too_large":
        return True, fields, ("the submission folder was larger than the program's cap (60 MB, or too many files or folders) "
                              "when the graded episode started: nothing was frozen, the program item scores 0")
    if status != "ok":
        return True, fields, "the submission folder was not frozen at the graded start (status %r)" % (status,)
    if os.path.islink(SNAPSHOT) or not os.path.isdir(SNAPSHOT):
        return True, fields, "the frozen copy of the submission folder is missing"
    entries, nfiles, nbytes, problems = snapshot_listing(SNAPSHOT)
    if problems or snapshot_digest(entries) != fields.get("snapshot_sha256") or nfiles != fields.get("snapshot_files") \
            or nbytes != fields.get("snapshot_bytes"):
        return True, fields, "the frozen copy of the submission folder does not match its record"
    return True, fields, None


# ---------------------------------------------------------------------------------------------- 4. the program item
def program_item():
    """Main's offline grading on /home/user/submission, as a process of its own. Its verdict (the kit's envelope)."""
    if os.getuid() == 0:
        os.makedirs(WORK, exist_ok=True)
        os.chmod(os.path.dirname(WORK), 0o700)
        os.chmod(WORK, 0o700)
    else:
        os.makedirs(WORK, exist_ok=True)
    out = os.path.join(WORK, "offline_verdict.json")
    if os.path.lexists(out):
        os.unlink(out)
    env = dict(os.environ)
    env.pop("ALE_VERIFY_PYTHON", None)            # main's grader writes its verdict to `out`, never the engine's result
    env.update({"ARB_OFFLINE_ANCHOR": "anchor_offline.json", "ARB_OFFLINE_VERDICT": out})
    t0 = time.time()
    r = subprocess.run([sys.executable, "-s", os.path.join(STAGE_DIR, "offline_verify.py")], cwd=STAGE_DIR, env=env)
    if r.returncode != 0 or not os.path.isfile(out):
        log("[grader] TASK ERROR: the program item's grading (offline_verify.py) failed (exit %s)" % r.returncode)
        raise SystemExit(1)
    with open(out) as f:
        verdict = json.load(f)
    verdict["wall_s"] = time.time() - t0
    return verdict


# ---------------------------------------------------------------------------------------------- 5.-6. scoring
def _number(v):
    if isinstance(v, bool):
        return 1.0 if v else 0.0
    if isinstance(v, (int, float)) and math.isfinite(float(v)):
        return float(v)
    return None


def write_verdict(verdict, diagnostic=None):
    """In the engine (ALE_VERIFY_PYTHON set) the kit writes reward.json and verification.json once; elsewhere the
    envelope goes to verdict.local.json in the stage folder (or ROBOTICS_GRADER_VERDICT_PATH)."""
    if os.environ.get("ALE_VERIFY_PYTHON"):
        kit_write_verdict(verdict, diagnostic=diagnostic)
        return
    path = os.environ.get("ROBOTICS_GRADER_VERDICT_PATH") or os.path.join(STAGE_DIR, "verdict.local.json")
    if diagnostic:
        verdict = dict(verdict, diagnostic=diagnostic)
    kit_write_verdict(verdict, path=path)


def main():
    killed, left = stop_agent()                                                  # 1
    with open(os.path.join(STAGE_DIR, "online.json")) as f:
        online = json.load(f)
    record_path = os.environ.get("ARB_ONLINE_RECORD") or online.get("record") or os.path.join(ONLINE_DIR, "record.json")
    rec, why = read_json_file(record_path)                                       # 2
    if rec is None:
        rec_why = ("no graded-episode record (%s): the graded episode was never started" % record_path if why == "missing"
                   else "the graded-episode record is %s" % why)
    else:
        rec_why = check_record(rec, online)
    rec_ok = rec is not None and rec_why is None
    rstate = record_state(rec, why, online)
    state, _ = read_json_file(os.path.join(ONLINE_DIR, "state.json"))
    started, snap, freeze_why = freeze_decision(rec, rstate, state)              # 3
    extra = {"agent_processes_killed_dual": float(killed), "online_record_valid": 1.0 if rec_ok else 0.0,
             "online_started": 1.0 if started else 0.0}
    for k, v in snap.items():
        if k != "snapshot_status" and _number(v) is not None:
            extra[k] = _number(v)
    extra["snapshot_ok"] = 1.0 if snap.get("snapshot_status") == "ok" else 0.0
    extra["snapshot_too_large"] = 1.0 if snap.get("snapshot_status") == "too_large" else 0.0
    if started and freeze_why is None:
        place_snapshot(SNAPSHOT, SUBMISSION)
        log("[grader] the program item grades the copy frozen at the graded start (%s files, %s bytes)" % (
            snap.get("snapshot_files"), snap.get("snapshot_bytes")))
    elif not started:
        log("[grader] the graded episode never started: the program item grades the session-end submission")

    if started and freeze_why is not None:                                       # 4
        log("[grader] program item 0: " + freeze_why)
        off_reward, off_score, off_text, off_metrics = 0.0, 0.0, "program item 0: " + freeze_why, {}
        extra["offline_ran"] = 0.0
    else:
        v = program_item()
        off_reward = float(v["rewards"]["reward"])
        off_metrics = v.get("metrics") or {}
        off_score = float(off_metrics.get("operation_score", 0.0))
        off_text = v.get("diagnostic") or ""
        extra["offline_ran"] = 1.0
        extra["offline_wall_s"] = float(v.get("wall_s", 0.0))

    anchor_on, meta_on = read_anchor("anchor_online.json")                       # 5
    if rec_ok:
        on_score = max(0.0, min(1.0, float(rec["episode_score"])))
        von = ratio_rewards(on_score, anchor_on, higher_is_better=meta_on.get("direction", "higher") == "higher",
                            metric_name="operation_score", locks_ok=True, full_at=meta_on.get("full_at", 1.0))
        on_reward = float(von["rewards"]["reward"])
        end = rec.get("end_reason")
        on_text = ("graded episode %s: ended %s (%s) at sim %.1f s after %s requests and %s step calls; operation_score %.4f"
                   % (rec.get("episode"), rec.get("ended"), end, float(rec.get("sim_time_s") or 0.0), rec.get("requests"),
                      rec.get("steps"), on_score))
        for r in END_REASONS + (None,):
            extra["online_end_reason_" + (r or "running")] = 1.0 if end == r else 0.0
        for k in ("steps", "sim_time_s", "requests", "wall_s_since_start"):
            if _number(rec.get(k)) is not None:
                extra["online_" + k] = _number(rec.get(k))
        for k, val in sorted((rec.get("metrics") or {}).items()):
            if _number(val) is not None:
                extra["online_" + str(k).replace(".", "_").replace("/", "_")] = _number(val)
    else:
        on_score, on_reward = 0.0, 0.0
        on_text = "graded episode item 0: " + str(rec_why)
        log("[grader] " + on_text)
    for k, val in off_metrics.items():
        name = "offline_" + str(k).replace(".", "_").replace("/", "_")
        if _number(val) is not None and name not in extra:
            extra[name] = _number(val)

    combined = (off_reward + on_reward) / 2.0                                    # 6
    extra.update({"offline_reward": off_reward, "online_reward": on_reward, "offline_operation_score": off_score,
                  "online_operation_score": on_score})
    anchor, meta = read_anchor()
    verdict = ratio_rewards(combined, anchor, cap=1.0, higher_is_better=meta.get("direction", "higher") == "higher",
                            metric_name=meta.get("metric", "dual_mode_score"), locks_ok=True, extra=extra,
                            full_at=meta.get("full_at", 1.0))
    text = ("dual mode: reward = mean(program item %.4f, graded episode item %.4f) = %.4f. The program (%s): %s The graded "
            "episode: %s." % (off_reward, on_reward, combined,
                              "frozen at the graded start" if started else "the session-end submission", off_text, on_text))
    log("[grader] " + text)
    write_verdict(verdict, diagnostic=text)


if __name__ == "__main__":
    main()
