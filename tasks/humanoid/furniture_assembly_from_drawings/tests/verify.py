"""Grader of furniture_assembly_from_drawings, dual mode: two scoring items of one task, one session, one verify. Runs
as ROOT in the verify stage (separate verification: a fresh verifier sandbox holding only the declared artifacts, restored
root-owned: the submission and the graded episode's record directory).

  * The program item (offline): main's grader, unchanged. tests/offline_verify.py is main's tests/verify.py with two
    hooks (the verdict is handed back instead of written; main(finish=False) leaves the stage modes and the freezer to
    this file); it runs the agent's assemble.py solve(cell) on 3 hidden episodes with fresh seeds, as on main, with
    main's gates, budgets, lockdown and record. The program it grades is the one frozen when the graded episode
    started (/var/lib/arb-online/submission_snapshot: the graded episode's root supervisor copied the submission folder
    there before the start was charged), or when the preparation window closed if the episode was never started; with
    no record of a freeze, the submission as the session left it (as on main).
  * The live item (online): the record of the ONE graded episode the agent played itself during its session, scored as
    the online variant scores it (the cell's stage score of the graded episode when it ended, 0 after a safety
    violation; 0 when the episode never started).
  * reward = the mean of the two components, each normalised by its own anchor, cap and full_at (anchor_offline.json is
    main's anchor.json; anchor_online.json the online variant's), written once through ratio_rewards against
    tests/anchor.json (dual_mode_score; 1.0 by construction).

Order:
  1. stop the agent account's processes as main's verifier does (kill, then keep them frozen until the verdict;
     stdlib only, before anything else is imported);
  2. read and check the record (tests/online.json: format, task, episode, seed digest);
  3. pick the program: the snapshot when the record says the program was frozen (its digest checked against the
     record: a mismatch is a task error), none when the frozen submission was over main's size cap (the program item
     then scores 0), else the restored submission;
  4. main's offline grading of that program (main's own lockdown runs on it: kill, freezer, stage lock, late-write gate,
     its root-owned snapshot, then the gates and the 3 episodes): the offline component;
  5. the live component from the record;
  6. reward = (offline_reward_exact + online_reward_exact) / 2;
  7. the verdict, written once; its metrics carry both components, both raw metrics, the freeze and the episode's
     end, and every metric of each component under "offline." / "online.".
Untouched sandbox: no program, no record -> 0. Oracle: the reference program frozen at the start (privileged offline
grading with its token, 1.0) and the privileged reference assembler on the graded episode (1.0) -> 1.
"""

import hashlib
import json
import math
import os
import pwd
import stat as statmod
import sys
import time

sys.dont_write_bytecode = True

STAGE_DIR = os.environ.get("ALE_STAGE_DIR", os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, STAGE_DIR)

HOME = os.environ.get("ALE_HOME", "/home/user")
AGENT_USER = os.path.basename(HOME.rstrip("/")) or "user"
ONLINE = os.path.join(STAGE_DIR, "online.json")
RECORD_FORMAT = "arb-online-record/1"
END_REASONS = ("done", "finish", "graded_window", "worker_restart")
SNAPSHOT_STATUSES = ("ok", "too_large", "failed")

from cellkit import lockdown, verify_freeze  # noqa: E402  (stdlib only)

FREEZER = [None]        # the agent account's verify_freeze.AgentFreezer: from _early_lockdown until the verdict


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def _uid_of(name):
    try:
        return pwd.getpwnam(name).pw_uid
    except (KeyError, TypeError):
        return None


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
    """PIDs of the account's live processes that are not stopped, after the freezer has had up to `bound_s` to stop
    them (a respawn that appears meanwhile is frozen by the next pass); empty when every one is stopped."""
    deadline = time.time() + float(bound_s)
    while True:
        FREEZER[0].freeze_now()
        time.sleep(0.05)
        running = [pid for pid in lockdown.live_uid_processes(uid) if _proc_state(pid) not in ("T", "t", "X")]
        if not running or time.time() > deadline:
            return running


def _early_lockdown():
    """Step 1: the agent account stopped as main's verifier stops it (killed, then kept frozen until the verdict). The
    rest of main's lockdown (stage lock, late-write gate, its snapshot of the program) runs with main's grading in
    step 4, on the program step 3 picks."""
    out = {"agent_processes_killed": 0.0, "agent_processes_left": 0.0, "agent_processes_running": 0.0, "reasons": []}
    if os.getuid() != 0:
        return out
    uid = _uid_of(AGENT_USER)
    if uid is not None and uid != 0:
        killed, left = lockdown.stop_and_kill_uid(uid, grace_s=8.0)
        out["agent_processes_killed"], out["agent_processes_left"] = float(killed), float(left)
    # the image's init respawns the agent account's desktop within seconds of the kill: every process of the account
    # stays SIGSTOPped (polled every 0.1 s) until the verdict is written
    FREEZER[0] = verify_freeze.AgentFreezer.for_user(AGENT_USER).start()
    if out["agent_processes_left"]:
        running = _still_running(uid)
        if running:
            out["agent_processes_running"] = float(len(running))
            out["reasons"].append("%d process(es) of the agent account survived the kill at verify start and could "
                                  "not be stopped" % len(running))
        else:
            log("[grader] %d process(es) of the agent account (the image's respawned desktop) outlived the kill at "
                "verify start: frozen until the verdict" % int(out["agent_processes_left"]))
    return out


EARLY = _early_lockdown() if __name__ == "__main__" else {"reasons": []}

import offline_verify as OV  # noqa: E402  (main's grader; its own lockdown runs only when called, in step 4)
from robotics_grader import fail, ratio_rewards, read_anchor  # noqa: E402
from robotics_grader import write_verdict as kit_write_verdict  # noqa: E402


def load_json(p):
    with open(p) as f:
        return json.load(f)


def _reject_constant(name):
    raise ValueError("non-finite number %s" % name)


def _finite(v):
    if isinstance(v, bool):
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


# ----------------------------------------------------------------------------------------------- step 2: the record
def read_record(path, expected, max_bytes):
    """(record, None) for a record of this task's graded episode (a regular file within the size limit, valid JSON,
    format, task, episode and seed digest equal to online.json), or (None, reason)."""
    try:
        st = os.lstat(path)
    except FileNotFoundError:
        return None, "no record of the graded episode at %s" % path
    except OSError as e:
        return None, "the record cannot be examined: %s" % e
    if not statmod.S_ISREG(st.st_mode):
        return None, "the record is not a regular file"
    if st.st_size > max_bytes:
        return None, "the record is larger than %d bytes" % max_bytes
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
        with os.fdopen(fd, "rb") as fh:
            blob = fh.read(max_bytes + 1)
        if len(blob) > max_bytes:
            return None, "the record is larger than %d bytes" % max_bytes
        rec = json.loads(blob.decode("utf-8"), parse_constant=_reject_constant)
    except (OSError, ValueError, UnicodeDecodeError) as e:
        return None, "the record is not valid JSON: %s" % (str(e)[:200],)
    if not isinstance(rec, dict):
        return None, "the record is not a JSON object"
    for key in ("format", "task", "episode", "seed_sha256"):
        if rec.get(key) != expected.get(key):
            return None, "the record's %s is %r, not %r" % (key, rec.get(key), expected.get(key))
    return rec, None


def live_score(rec):
    """(episode_score, None) for a started episode with a usable score, or (None, reason)."""
    if rec is None:
        return None, "no record"
    if rec.get("started") is not True:
        return None, "the graded episode was never started"
    score = _finite(rec.get("episode_score"))
    if score is None or score < 0.0:
        return None, "the record's episode_score is not a finite number >= 0: %r" % (rec.get("episode_score"),)
    return score, None


# ----------------------------------------------------------------------------------------------- step 3: the program
def tree_digest(files):
    """sha256 of the sorted file list with sizes and contents (environment/setup/online/graded_server.py computes the
    same over the files it copied): one line "<path>\\0<size>\\0<sha256 of the content>\\n" per regular file."""
    h = hashlib.sha256()
    for rel, size, digest in sorted(files):
        h.update(rel.encode("utf-8", "surrogateescape") + b"\0" + str(int(size)).encode() + b"\0" +
                 digest.encode() + b"\n")
    return h.hexdigest()


def snapshot_digest(root):
    """(digest, files, bytes) of the restored snapshot; a symlink or a special file in it is a task error (the service
    copies regular files and directories only)."""
    files, total = [], 0
    if not os.path.isdir(root) or os.path.islink(root):
        return tree_digest([]), 0, 0
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        for name in sorted(dirnames):
            if os.path.islink(os.path.join(dirpath, name)):
                fail("the frozen program holds a symlink (%s): it was not written by the graded episode's service"
                     % os.path.relpath(os.path.join(dirpath, name), root))
        for name in sorted(filenames):
            p = os.path.join(dirpath, name)
            st = os.lstat(p)
            if not statmod.S_ISREG(st.st_mode):
                fail("the frozen program holds a non-regular file (%s)" % os.path.relpath(p, root))
            h = hashlib.sha256()
            with open(p, "rb") as fh:
                for b in iter(lambda: fh.read(1 << 20), b""):
                    h.update(b)
            files.append((os.path.relpath(p, root), st.st_size, h.hexdigest()))
            total += st.st_size
    return tree_digest(files), len(files), total


def choose_program(rec, snapshot_root):
    """(program folder or None, facts): None means the program item scores 0 (the frozen submission was over main's
    cap). The snapshot is used when the record says the program was frozen, at the start or at the close of the
    preparation window; otherwise the submission as the session left it."""
    restored = os.path.join(HOME, "submission")
    facts = {"program_snapshot": 0.0, "program_session_end": 0.0, "program_too_large": 0.0}
    status = rec.get("snapshot_status") if rec is not None else None
    frozen = rec is not None and (rec.get("started") is True or rec.get("prep_window_closed") is True)
    if rec is not None and rec.get("started") is True and status not in ("ok", "too_large"):
        fail("the record of a started graded episode holds no usable freeze of the program (snapshot_status %r)"
             % (status,))
    if frozen and status == "ok":
        digest, n, size = snapshot_digest(snapshot_root)
        if digest != rec.get("snapshot_sha256") or n != int(rec.get("snapshot_files") or 0):
            fail("the frozen program (%d files, %d bytes) does not match its record (%s files, digest %s...)" % (
                n, size, rec.get("snapshot_files"), str(rec.get("snapshot_sha256"))[:12]))
        facts["program_snapshot"] = 1.0
        log("[grader] the program frozen at %s: %d files, %d bytes" % (rec.get("snapshot_at"), n, size))
        return snapshot_root, facts
    if frozen and status == "too_large":
        facts["program_too_large"] = 1.0
        log("[grader] the program frozen at %s was over main's submission cap (%s): the program item scores 0" % (
            rec.get("snapshot_at"), rec.get("snapshot_reason")))
        return None, facts
    facts["program_session_end"] = 1.0
    log("[grader] no freeze of the program on record (%s): the submission as the session left it" % (
        "no usable record" if rec is None else "snapshot_status %r" % (status,)))
    return restored, facts


# ----------------------------------------------------------------------------------------------- step 4: offline
def offline_component(program_dir):
    """(reward_exact, raw assembly_score, metrics) of main's grading of the program at `program_dir` (None: 0, no
    grading). Main's write_verdict would score main's verdict against tests/anchor.json; here the same formula runs
    against anchor_offline.json (main's anchor.json, byte-identical)."""
    anchor, meta = read_anchor("anchor_offline.json")
    if program_dir is None:
        return 0.0, 0.0, {"contract_ok": 0.0, "assembly_score": 0.0, "graded": 0.0}
    captured = {}

    def sink(rewards, metrics):
        captured["rewards"], captured["metrics"] = dict(rewards), dict(metrics)
        return float(rewards["assembly_score"])

    OV.SUBMISSION = program_dir                   # the program main's grading copies, gates and runs
    OV.EARLY = OV._early_lockdown()               # main's lockdown, on that program
    OV.VERDICT_SINK[0] = sink
    OV.main(finish=False)                         # main's grading; a grader-side failure ends the run as a task error
    if "rewards" not in captured:
        fail("main's grading ended without a verdict")
    rewards, metrics = captured["rewards"], captured["metrics"]
    overall = float(rewards["assembly_score"])
    extra = {}
    for k, v in metrics.items():                  # main's write_verdict: finite numbers, plus the gates and criteria
        f = _finite(v)
        if f is not None:
            extra[k] = f
    extra.update({k: float(rewards[k]) for k in OV.GATES + OV.CRITERIA})
    verdict = ratio_rewards(overall, anchor, higher_is_better=meta.get("direction", "higher") == "higher",
                            metric_name=meta.get("metric", "assembly_score"), extra=extra,
                            full_at=meta.get("full_at", 1.0))
    out = dict(verdict["metrics"], graded=1.0)
    return float(verdict["rewards"]["reward"]), overall, out


# ----------------------------------------------------------------------------------------------- step 5: online
def record_extra(rec):
    """The record's facts (finite numbers only): its per-episode statistics and its counters."""
    extra = {}
    for k, v in sorted((rec.get("metrics") or {}).items()):
        f = _finite(v)
        if f is not None:
            extra[str(k)] = f
    for k in ("steps", "ticks", "sim_time_s", "requests", "status_requests", "wall_s_since_start", "record_writes",
              "requests_log_dropped"):
        f = _finite(rec.get(k))
        if f is not None:
            extra[k] = f
    extra["episode_ended"] = 1.0 if rec.get("ended") is True else 0.0
    extra["cell_done_sim_cap"] = 1.0 if rec.get("cell_done_reason") == "sim_cap" else 0.0
    extra["cell_done_internal_error"] = 1.0 if rec.get("cell_done_reason") == "internal_error" else 0.0
    extra["privileged_mode"] = 1.0 if rec.get("privileged") is True else 0.0
    extra["oracle_token_mismatch"] = 1.0 if rec.get("oracle_token_mismatch") is True else 0.0
    return extra


def online_component(rec, why):
    """(reward_exact, raw episode_score, metrics, reason) of the live item; 0 without a started episode."""
    anchor, meta = read_anchor("anchor_online.json")
    score, why_live = live_score(rec) if rec is not None else (None, why)
    if score is None:
        return 0.0, 0.0, {"record_ok": 1.0 if rec is not None else 0.0, "started": 0.0}, why_live
    if rec.get("score_error"):
        fail("the graded episode's record reports that the cell could not score it: %s" % (rec["score_error"],))
    extra = dict(record_extra(rec), record_ok=1.0, started=1.0)
    verdict = ratio_rewards(score, anchor, cap=1.5, higher_is_better=meta.get("direction", "higher") == "higher",
                            metric_name=meta.get("metric", "assembly_score"), locks_ok=True, extra=extra,
                            full_at=meta.get("full_at", 1.0))
    return float(verdict["rewards"]["reward"]), score, dict(verdict["metrics"]), None


# ----------------------------------------------------------------------------------------------- steps 6-7
def emit(verdict, diagnostic):
    """The verdict through the kit, once: in the engine (`ALE_VERIFY_PYTHON` set) reward.json and verification.json,
    the metric names with '.' and '/' replaced by '_' (finite numbers only); elsewhere verdict.local.json in the
    stage folder."""
    if os.environ.get("ALE_VERIFY_PYTHON"):
        recorded = {}
        for k, v in sorted(verdict["metrics"].items()):
            f = _finite(v)
            if f is not None:
                recorded[k.replace(".", "_").replace("/", "_")] = f
        kit_write_verdict({"rewards": verdict["rewards"], "metrics": recorded}, diagnostic=diagnostic)
        return
    kit_write_verdict(dict(verdict, diagnostic=diagnostic), path=os.path.join(STAGE_DIR, "verdict.local.json"))


def _main_dual():
    t0 = time.time()
    expected = load_json(ONLINE)
    for reason in EARLY.get("reasons") or []:
        log("[grader] note: %s (main's lockdown checks again before the program runs)" % reason)
    # 2. the record
    rec, why = read_record(expected.get("record", "/var/lib/arb-online/record.json"), expected,
                           int(expected.get("record_max_bytes", 1 << 20)))
    if rec is None:
        log("[grader] record: %s" % why)
    # 3. the program
    program_dir, program_facts = choose_program(rec, expected.get("snapshot_dir",
                                                                  "/var/lib/arb-online/submission_snapshot"))
    # 4. the program item (main's grading)
    off_reward, off_raw, off_metrics = offline_component(program_dir)
    # 5. the live item
    on_reward, on_raw, on_metrics, on_why = online_component(rec, why)
    # 6. the mean
    combined = (off_reward + on_reward) / 2.0
    anchor, meta = read_anchor()
    extra = {"offline_reward": off_reward, "online_reward": on_reward, "offline_assembly_score": off_raw,
             "online_assembly_score": on_raw, "record_ok": 1.0 if rec is not None else 0.0,
             "graded_episode_started": 1.0 if (rec is not None and rec.get("started") is True) else 0.0,
             "snapshot_files": float((rec or {}).get("snapshot_files") or 0),
             "snapshot_bytes": float((rec or {}).get("snapshot_bytes") or 0),
             "agent_processes_running_at_start": float(EARLY.get("agent_processes_running", 0.0)),
             "dual_verify_wall_s": time.time() - t0}
    extra.update(program_facts)
    status = (rec or {}).get("snapshot_status")
    for s in SNAPSHOT_STATUSES:
        extra["snapshot_status." + s] = 1.0 if status == s else 0.0
    extra["snapshot_status.none"] = 1.0 if status is None else 0.0
    extra["snapshot_at.start"] = 1.0 if (rec or {}).get("snapshot_at") == "start" else 0.0
    extra["snapshot_at.prep_window"] = 1.0 if (rec or {}).get("snapshot_at") == "prep_window" else 0.0
    end = (rec or {}).get("end_reason")
    for r in END_REASONS:
        extra["online_end_reason." + r] = 1.0 if end == r else 0.0
    extra["online_end_reason.session_end"] = 1.0 if (rec is not None and rec.get("started") is True and
                                                     rec.get("ended") is not True) else 0.0
    for k, v in off_metrics.items():
        f = _finite(v)
        if f is not None:
            extra["offline." + k] = f
    for k, v in on_metrics.items():
        f = _finite(v)
        if f is not None:
            extra["online." + k] = f
    # 7. the verdict, once
    verdict = ratio_rewards(combined, anchor, cap=1.0, higher_is_better=True,
                            metric_name=meta.get("metric", "dual_mode_score"), extra=extra,
                            full_at=meta.get("full_at", 1.0))
    text = ("dual_mode_score %.4f = (offline %.4f + online %.4f) / 2. Program item: assembly_score %.4f of main's "
            "grading on the hidden episodes (%s). Live item: %s." % (
                combined, off_reward, on_reward, off_raw,
                "the program frozen at the graded start" if program_facts["program_snapshot"] and
                (rec or {}).get("snapshot_at") == "start" else
                "the program frozen when the preparation window closed" if program_facts["program_snapshot"] else
                "the frozen submission was over main's size cap, not graded" if program_facts["program_too_large"]
                else "the submission as the session left it",
                ("assembly_score %.4f on the graded episode %s, ended %s" % (
                    on_raw, rec.get("episode"),
                    rec.get("end_reason") if rec.get("ended") else "with the session (as of the last request)"))
                if on_why is None else "0 (%s)" % on_why))
    log("[grader] " + text)
    emit(verdict, text)


def main():
    try:
        _main_dual()
    finally:
        OV._restore_modes()                       # main's: the stage folder, its parent and its files (step 4's lock)
        if OV.FREEZER[0] is not None:
            OV.FREEZER[0].stop()
        if FREEZER[0] is not None:
            FREEZER[0].stop()                     # after the verdict; the frozen stay stopped


if __name__ == "__main__":
    main()
