"""Grader of furniture_assembly_from_drawings, online variant (two Unitree G1 humanoids assemble a flat-pack bookcase
from its drawings, camera-only): scores the record of the ONE graded episode that the agent played itself during its
session. Runs as ROOT in the verify stage; runs no program.

During the agent phase the graded episode's service (environment/setup/online/graded_server.py: the offline grader's
own furniture cell on the hidden graded episode, under the offline per-episode rules) served the episode to the agent
as the lab account and was the only writer of its record directory, /var/lib/arb-online (the lab account's, 0700: the
agent could neither read nor write it). Verification is separate (task.toml): this runs in a fresh verifier sandbox
from the task image, into which the engine restored only the declared artifacts, root-owned: the submission (not read)
and the record directory. This grader:
  0. FIRST, stdlib only (`_early_lockdown`; tests/test.sh closed the stage folder already): freezes and kills every
     process of the agent account and keeps it stopped until the verdict is written (cellkit.verify_freeze; in the
     separate verifier these are the image's own desktop session, started with the sandbox) and locks the stage;
  1. reads the record (grader_config.json `record`): a regular file (no link) of at most `record_max_bytes`, valid
     JSON, format "arb-online-record/1", and task, episode and seed_sha256 equal to tests/online.json, started true,
     episode_score finite and >= 0. A missing, invalid or never-started record scores 0 (zero_verdict: the untouched
     sandbox never starts the episode); a record whose cell reported a scoring failure is a TASK ERROR
     (robotics_grader.fail: the grading broke, the agent did not fail);
  2. criterion: assembly_score = the record's episode_score, the offline grader's per-episode value for the graded
     episode: the cell's stage score (the credited dowel joints, the back panel's seat and the carcass's placement
     over the bookcase's total weight, furniture_cell.scoring), 0 after a safety violation, read when the episode
     ended (the agent's finish, its simulated-time cap, the graded window, a restart of the service's worker) or,
     when the session ended first, as it stood after the agent's last request (the service rewrites the record after
     the requests that change the episode, at most once a second while requests arrive);
  3. reward = robotics_grader.ratio_rewards(assembly_score, anchor) with tests/anchor.json (the oracle's measured
     episode_score on the graded episode); the record's end reason, steps, simulated time, requests and the
     per-episode statistics (the offline grader's names) ride in the metrics.
Untouched sandbox: the episode was never started -> 0. Oracle: solution/solve.sh started the graded episode with the
oracle token and ran the privileged reference assembler on it through the agent-side client -> 1.
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
CONFIG = os.path.join(STAGE_DIR, "grader_config.json")
ONLINE = os.path.join(STAGE_DIR, "online.json")
RECORD_FORMAT = "arb-online-record/1"
END_REASONS = ("done", "finish", "graded_window", "worker_restart")

from cellkit import lockdown, runner, verify_freeze  # noqa: E402  (stdlib only)

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
    """The agent account stopped (killed, and kept frozen until the verdict), the stage locked. Nothing of the agent's
    is read here or later except the record, which the engine restored root-owned."""
    out = {"agent_processes_killed": 0.0, "agent_processes_left": 0.0, "agent_processes_running": 0.0,
           "stage_locked": 0.0, "stage_parent_locked": 0.0, "stage_lock_latency_ms": -1.0, "saved": None,
           "parent": None, "reasons": []}
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

from robotics_grader import fail, ratio_rewards, read_anchor, zero_verdict  # noqa: E402
from robotics_grader import write_verdict as kit_write_verdict  # noqa: E402


def load_json(p):
    with open(p) as f:
        return json.load(f)


def _reject_constant(name):
    raise ValueError("non-finite number %s" % name)


def _finite(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def read_record(path, expected, max_bytes):
    """(record, None) for a usable record of the graded episode, or (None, reason) for a missing, invalid or
    never-started one (scored 0)."""
    try:
        st = os.lstat(path)
    except FileNotFoundError:
        return None, "no record of the graded episode at %s (the episode was never started)" % path
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
    if rec.get("started") is not True:
        return None, "the graded episode was never started"
    score = rec.get("episode_score")
    if isinstance(score, bool) or _finite(score) is None or float(score) < 0.0:
        return None, "the record's episode_score is not a finite number >= 0: %r" % (score,)
    return rec, None


def record_extra(rec):
    """The record's facts for the verdict's metrics (finite numbers only)."""
    extra = {}
    for k, v in sorted((rec.get("metrics") or {}).items()):
        f = _finite(v)
        if f is not None and not isinstance(v, bool):
            extra[str(k)] = f
    for k in ("steps", "ticks", "sim_time_s", "requests", "status_requests", "wall_s_since_start", "record_writes",
              "requests_log_dropped"):
        f = _finite(rec.get(k))
        if f is not None:
            extra[k] = f
    reason = rec.get("end_reason")
    for r in END_REASONS:
        extra["end_reason." + r] = 1.0 if reason == r else 0.0
    extra["episode_ended"] = 1.0 if rec.get("ended") is True else 0.0
    extra["end_reason.session_end"] = 0.0 if rec.get("ended") is True else 1.0    # scored as of the last request
    extra["cell_done_sim_cap"] = 1.0 if rec.get("cell_done_reason") == "sim_cap" else 0.0
    extra["cell_done_internal_error"] = 1.0 if rec.get("cell_done_reason") == "internal_error" else 0.0
    extra["privileged_mode"] = 1.0 if rec.get("privileged") is True else 0.0
    extra["oracle_token_mismatch"] = 1.0 if rec.get("oracle_token_mismatch") is True else 0.0
    return extra


def emit(verdict, diagnostic):
    """The verdict through the kit: in the engine (`ALE_VERIFY_PYTHON` set) reward.json and verification.json, the
    metric names with '.' and '/' replaced by '_' (finite numbers only); elsewhere the envelope goes to
    verdict.local.json in the stage folder."""
    if os.environ.get("ALE_VERIFY_PYTHON"):
        recorded = {}
        for k, v in sorted(verdict["metrics"].items()):
            f = _finite(v)
            if f is not None:
                recorded[k.replace(".", "_").replace("/", "_")] = f
        kit_write_verdict({"rewards": verdict["rewards"], "metrics": recorded}, diagnostic=diagnostic)
        return
    verdict = dict(verdict, diagnostic=diagnostic)
    kit_write_verdict(verdict, path=os.path.join(STAGE_DIR, "verdict.local.json"))


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


def _main_locked():
    t0 = time.time()
    cfg = load_json(CONFIG)
    expected = load_json(ONLINE)
    anchor, meta = read_anchor()
    metric = meta.get("metric", "assembly_score")
    base = {k: float(EARLY.get(k, 0.0)) for k in ("agent_processes_killed", "agent_processes_left",
                                                  "agent_processes_running", "stage_locked", "stage_parent_locked")}
    for reason in EARLY.get("reasons") or []:
        log("[grader] note: %s (the record is root-owned in this sandbox; not a scoring input)" % reason)
    rec, why = read_record(cfg.get("record", "/var/lib/arb-online/record.json"), expected,
                           int(cfg.get("record_max_bytes", 1 << 20)))
    if rec is None:
        base["record_ok"] = 0.0
        base["verify_wall_s"] = time.time() - t0
        verdict = zero_verdict(reason=why, anchored=anchor is not None, extra=base)
        emit(verdict, "scoring 0: " + why)
        return
    if rec.get("score_error"):
        fail("the graded episode's record reports that the cell could not score it: %s" % (rec["score_error"],))
    score = float(rec["episode_score"])
    extra = dict(base, record_ok=1.0, verify_wall_s=time.time() - t0)
    extra.update(record_extra(rec))
    verdict = ratio_rewards(score, anchor, cap=1.5, higher_is_better=meta.get("direction", "higher") == "higher",
                            metric_name=metric, locks_ok=True, extra=extra, full_at=meta.get("full_at", 1.0))
    m = rec.get("metrics") or {}
    text = ("assembly_score %.4f on the graded episode %s (the cell's stage score: credit %s of 9, %s joint(s) locked, "
            "back panel %s, placement %s; safety %s), ended %s (%s) after %s steps, %.1f s simulated, %s requests" % (
                score, rec.get("episode"), m.get("credit_mean"), m.get("locked_joints_mean"),
                "credited" if m.get("back_panel_rate") else "not credited",
                "ok" if m.get("placement_rate") else "not credited",
                "violated" if m.get("safety_violation_rate") else "ok",
                "yes" if rec.get("ended") else "no: the session ended first, scored as of the last request",
                rec.get("end_reason"), rec.get("steps"), float(rec.get("sim_time_s") or 0.0), rec.get("requests")))
    log("[grader] " + text)
    emit(verdict, text)


def main():
    try:
        _main_locked()
    finally:
        _restore_modes()
        if FREEZER[0] is not None:
            FREEZER[0].stop()                   # after the verdict; the frozen stay stopped


if __name__ == "__main__":
    main()
