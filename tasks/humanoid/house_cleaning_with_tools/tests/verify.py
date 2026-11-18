"""Grader of house_cleaning_with_tools, online graded episode (experimental variant; one Unitree G1 humanoid cleans a
studio apartment after a dinner, with tools, camera-only): runs as ROOT in the verify stage.

The agent played ONE hidden graded episode itself, during its session, through the graded episode's cell service
(environment/setup/online/service.py, started by environment/setup/setup.sh: the offline verifier's own house cell,
online/house_cell = a byte copy of tests/house_cell, with its scene, episode logic, per-episode limits, safety watch and
score function, served as a dedicated account that is not the agent's). The service left the episode's record in
/var/lib/arb-online/record.json, a declared artifact (task.toml `artifacts`) the engine collected as root after the
agent phase and restored here, root-owned, in a separate verifier sandbox that holds nothing else of the agent's. This
grader runs no program:
  0. FIRST, stdlib only (`_early_lockdown`; tests/test.sh closed the stage folder already): freezes and kills every
     process of the agent account (in the separate verifier: the image's own desktop session, started with the
     sandbox) and keeps it stopped until the verdict is written (cellkit.verify_freeze), and locks the stage; a
     process of that account that cannot be stopped ends the run as a TASK ERROR (the sandbox is not the grader's);
  1. reads the record: a regular file of at most 1 MB, valid JSON, its `format`, `task`, `episode` and `seed_sha256`
     equal to tests/online.json (the graded episode: its id and the sha256 of its seed), `started` true and its
     `episode_score` a finite number >= 0. A missing, invalid or never-started record scores 0 (zero_verdict, the
     reason as the diagnostic);
  2. a failure on the grading side that the service recorded (a camera the renderer could not draw, a state that could
     not be scored) ends the run as a TASK ERROR, as offline: a zero says the agent failed, a task error says the
     grading broke;
  3. criterion: cleaning_score = the record's episode_score, the offline verifier's per-episode score of the graded
     episode as it ended (the cell's stage score: the customer's checklist credit over its total weight of 10, trash
     sorted 2, dishes cleared 1, books shelved 1, table degreased 2.5, floor swept 2.5, finishing 1, less the damage
     charges; 0 with a safety violation); reward = robotics_grader.ratio_rewards against tests/anchor.json (the oracle's
     measured episode_score on the same graded episode). The record's per-episode statistics ride along in the metrics.
Untouched sandbox: the graded episode was never started, there is no record -> 0.
Oracle: solution/solve.sh starts the graded episode with the oracle token and runs the privileged reference cleaner
through the agent-side client, then finishes it -> 1.
"""

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
RECORD = os.environ.get("ARB_ONLINE_RECORD", "/var/lib/arb-online/record.json")
RECORD_FORMAT = "arb-online-record/1"
MAX_RECORD_BYTES = 1 << 20
END_REASONS = ("done", "finish", "graded_window", "worker_restart")

from cellkit import lockdown, runner, verify_freeze  # noqa: E402  (stdlib only)

FREEZER = [None]        # the agent account's verify_freeze.AgentFreezer: started by _early_lockdown, stopped after the verdict


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
    out = {"agent_processes_killed": 0.0, "agent_processes_left": 0.0, "stage_locked": 0.0, "stage_parent_locked": 0.0,
           "stage_lock_latency_ms": -1.0, "saved": None, "parent": None, "reasons": []}
    if os.getuid() != 0:
        return out
    uid = _uid_of(AGENT_USER)
    if uid is not None and uid != 0:
        killed, left = lockdown.stop_and_kill_uid(uid, grace_s=8.0)
        out["agent_processes_killed"], out["agent_processes_left"] = float(killed), float(left)
    # the image's init respawns the agent account's desktop within seconds of the kill: every process of the account
    # stays SIGSTOPped (polled every 0.1 s) until the verdict is written.
    FREEZER[0] = verify_freeze.AgentFreezer.for_user(AGENT_USER).start()
    if out["agent_processes_left"]:
        # in the separate verifier a process that outlives the kill is the image's own (its supervisord respawns the
        # account's desktop as fast as it is killed); it is frozen with the rest of the account, and only a process
        # that is still running once the freezer had its turn is held against the run.
        running = _still_running(uid)
        if running:
            out["reasons"].append("%d process(es) of the agent account survived the kill at verify start and could not be "
                                  "stopped" % len(running))
        else:
            log("[grader] %d process(es) of the agent account (the image's respawned desktop) outlived the kill at verify "
                "start: frozen until the verdict" % int(out["agent_processes_left"]))
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
    out["stage_lock_latency_ms"] = lockdown.process_age_ms()
    return out


EARLY = _early_lockdown() if __name__ == "__main__" else {"saved": None, "parent": None, "reasons": []}

from robotics_grader import fail, ratio_rewards, read_anchor, write_verdict, zero_verdict  # noqa: E402

ONLINE = os.path.join(STAGE_DIR, "online.json")


def load_json(p):
    with open(p) as f:
        return json.load(f)


def _number(v):
    """float(v) when v is a finite JSON number (not a bool), else None."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    v = float(v)
    return v if math.isfinite(v) else None


def load_record(online):
    """(record, None) for a record of the graded episode that was started, else (None, why it scores 0)."""
    try:
        st = os.lstat(RECORD)
    except FileNotFoundError:
        return None, "no record of the graded episode (%s): it was never started" % RECORD
    except OSError as e:
        return None, "the record %s cannot be read (%s)" % (RECORD, e.__class__.__name__)
    if not statmod.S_ISREG(st.st_mode):
        return None, "the record %s is not a regular file" % RECORD
    if st.st_size > MAX_RECORD_BYTES:
        return None, "the record %s is larger than %d bytes" % (RECORD, MAX_RECORD_BYTES)
    try:
        with open(RECORD, "rb") as fh:
            rec = json.loads(fh.read(MAX_RECORD_BYTES + 1).decode("utf-8"))
    except (OSError, ValueError, UnicodeDecodeError) as e:
        return None, "the record %s is not valid JSON (%s)" % (RECORD, e.__class__.__name__)
    if not isinstance(rec, dict):
        return None, "the record %s is not a JSON object" % RECORD
    for key, want in (("format", RECORD_FORMAT), ("task", online.get("task")), ("episode", online.get("episode")),
                      ("seed_sha256", online.get("seed_sha256"))):
        if rec.get(key) != want:
            return None, "the record's %s is %r, not the graded episode's %r" % (key, rec.get(key), want)
    if rec.get("started") is not True:
        return None, "the graded episode was never started"
    score = _number(rec.get("episode_score"))
    if score is None or score < 0.0:
        return None, "the record's episode_score %r is not a finite number >= 0" % (rec.get("episode_score"),)
    return rec, None


def _restore_modes():
    runner.unlock_stage(EARLY.get("saved"))
    mode = os.environ.get("ARB_STAGE_MODE")
    if mode:
        try:
            os.chmod(STAGE_DIR, int(mode, 8))
        except (OSError, ValueError):
            pass
    parent = EARLY.get("parent")
    if parent:
        try:
            os.chmod(parent[0], parent[1])
        except OSError:
            pass


def _write(verdict, diagnostic=None):
    """The kit's writer: in the engine (ALE_VERIFY_PYTHON set) reward.json and verification.json; elsewhere the
    envelope at ROBOTICS_GRADER_VERDICT_PATH or verdict.local.json in the stage folder."""
    if os.environ.get("ALE_VERIFY_PYTHON") or os.environ.get("ROBOTICS_GRADER_VERDICT_PATH"):
        write_verdict(verdict, diagnostic=diagnostic)
    else:
        write_verdict(verdict, path=os.path.join(STAGE_DIR, "verdict.local.json"), diagnostic=diagnostic)


def main():
    try:
        _main_locked()
    finally:
        _restore_modes()
        if FREEZER[0] is not None:
            FREEZER[0].stop()                   # after the verdict (written by _main_locked); the frozen stay stopped


def _main_locked():
    if EARLY.get("reasons"):
        fail("; ".join(EARLY["reasons"]))
    stats = {k: float(EARLY.get(k, -1.0 if k == "stage_lock_latency_ms" else 0.0))
             for k in ("stage_locked", "stage_parent_locked", "stage_lock_latency_ms", "agent_processes_killed",
                       "agent_processes_left")}
    stats["agent_processes_frozen"] = float(len(FREEZER[0].frozen)) if FREEZER[0] is not None else 0.0
    online = load_json(ONLINE)
    anchor, meta = read_anchor()
    metric = meta.get("metric", "cleaning_score")
    rec, why = load_record(online)
    if rec is None:
        stats["record_ok"] = 0.0
        _write(zero_verdict(reason=why, anchored=anchor is not None, extra=stats))
        return
    if int(_number(rec.get("render_errors")) or 0) > 0:
        fail("the renderer failed on %d camera request(s) of the graded episode: an infrastructure failure, not scored"
             % int(_number(rec.get("render_errors")) or 0))
    if rec.get("score_error"):
        fail("the graded episode could not be scored: %s" % (rec.get("score_error"),))
    score = float(rec["episode_score"])
    extra = dict(stats)
    reserved = {metric, "ratio", "anchor_recorded", "reward_exact"}
    for k, v in sorted((rec.get("metrics") or {}).items()):
        name = str(k).replace(".", "_").replace("/", "_")
        v = _number(v)
        if v is not None and name not in reserved:
            extra[name] = v
    extra.update({
        "record_ok": 1.0,
        "episode_ended": 1.0 if rec.get("ended") else 0.0,
        "steps": float(_number(rec.get("steps")) or 0.0),
        "sim_time_s": float(_number(rec.get("sim_time_s")) or 0.0),
        "requests": float(_number(rec.get("requests")) or 0.0),
        "wall_s_since_start": float(_number(rec.get("wall_s_since_start")) or 0.0),
        "privileged_mode": 1.0 if rec.get("privileged") else 0.0,
    })
    for r in END_REASONS:
        extra["end_reason_" + r] = 1.0 if (rec.get("ended") and rec.get("end_reason") == r) else 0.0
    verdict = ratio_rewards(score, anchor, higher_is_better=meta.get("direction", "higher") == "higher",
                            metric_name=metric, locks_ok=True, extra=extra, full_at=meta.get("full_at", 1.0))
    text = ("cleaning_score %.4f: the graded episode %s (%s at %.1f s of simulated time, %d steps, %d requests): the "
            "cell's stage score as the episode ended (the customer's checklist credit over its total weight of 10: "
            "trash sorted into the right bins 2, dishes on the dish tray 1, books on the shelf 1, grease off the dining "
            "table 2.5, crumbs in the general-waste bin 2.5, finishing 1: the sponge rinsed in its tray, the broom and "
            "the dustpan in their clips, the faucet off, the lids closed; less the damage charges), 0 with a safety "
            "violation") % (score, rec.get("episode"), ("ended: " + str(rec.get("end_reason"))) if rec.get("ended")
                             else "running when the session ended", float(_number(rec.get("sim_time_s")) or 0.0),
                             int(_number(rec.get("steps")) or 0), int(_number(rec.get("requests")) or 0))
    _write(verdict, diagnostic=text)
    log("[grader] %s; reward %.4f" % (text, verdict["rewards"]["reward"]))


if __name__ == "__main__":
    main()
