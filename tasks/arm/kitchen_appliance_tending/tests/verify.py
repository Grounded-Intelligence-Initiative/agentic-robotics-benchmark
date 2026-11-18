"""Grader for kitchen_appliance_tending, online graded episode (experimental variant).

Runs as root in a SEPARATE verifier sandbox and runs no program. During the agent phase the agent played the task's
one graded episode itself, through the lab service (environment/setup/lab/alelab/server.py, serving as the account
`alelab`): one entry of the offline grader's fixed hidden set (tests/hidden/episodes.json), chosen at the start from a
random order of all 18 (environment/setup/online/order.json, the seeds root-only; tests/online.json holds the same
order as index, unit, episode id and seed digest): the first entry whose unit is not the unit of a lab trial charged
before the start, so the graded unit is one the agent never practised on. It is simulated exactly as the offline
grader simulates an episode of that set
(appliance_plugin.py: Episode(ApplianceEnv(spec, None, render=True), seed); the score is the episode's mean request
credit (episode.py), 0 after an unstable simulation; an episode ended before its last request closed, by a finish,
its window or the end of the session, is scored as if it ended at that moment: the open request closes then and a
request not yet opened earns 0). The lab wrote the episode's record to /var/lib/arb-online/record.json, a
directory of the lab account that the agent can neither read nor write; the engine collected that directory as a
declared artifact after the agent phase and restored it here, root-owned. Nothing else of the agent's is read: the
submission folder is not.

This grader reclaims root's PATH directories first (robotics_grader.stage.secure_exec_path), stops every agent-UID
process (in the separate verifier only the base image's desktop runs as that account: a process that outlives the
bounded kill is stopped, and only one still running once stopped fails the run, cleanup_failed), reads the record
(a regular file of at most 1 MB, never through a link), checks it against tests/online.json (format, task, episode
id, the seed's sha256), that the episode was started, that its episode is the one the order gives for the record's
trial units (order_index, episode and seed digest recomputed from tests/online.json) and that its score is a finite
number >= 0, stops the agent account's processes again, and scores the episode's score against the anchor with the
kit. A missing, invalid or never-started record, or one whose episode the order does not give, scores 0. The rest of tests/ (the environment, the episode logic, the hidden set, the renderer)
is the offline grader's and is not used here.
"""

import json
import math
import os
import pwd
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
from robotics_grader.stage import secure_exec_path  # noqa: E402

STAGE = os.path.dirname(os.path.abspath(__file__))
ONLINE = os.path.join(STAGE, "online.json")
RECORD = "/var/lib/arb-online/record.json"
RECORD_FORMAT = "arb-online-record/1"
MAX_RECORD_BYTES = 1 << 20
METRIC = "mean_request_credit"
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


def read_record(path, online):
    """(record, None) for a valid record of this task's graded episode that was started, else (None, reason)."""
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
    if rec.get("format") != RECORD_FORMAT:
        return None, "the graded-episode record has another format"
    if rec.get("task") != online["task"]:
        return None, "the graded-episode record's task differs from tests/online.json"
    if rec.get("started") is not True:
        return None, "the graded episode was never started"
    units = rec.get("trial_units")
    if not isinstance(units, list) or not all(isinstance(u, str) for u in units):
        return None, "the graded-episode record has no valid trial_units"
    pos, want = expected_choice(online, units)
    if want is None:
        return None, "no entry of the order is left for the record's trial units"
    got = rec.get("order_index")
    if isinstance(got, bool) or got != pos or rec.get("episode") != want["episode"] \
            or rec.get("seed_sha256") != want["seed_sha256"]:
        return None, "the record's graded episode is not the one the order gives for its trial units"
    score = rec.get("episode_score")
    if isinstance(score, bool) or not isinstance(score, (int, float)) or not math.isfinite(score) or score < 0:
        return None, "the graded-episode record has no finite episode_score >= 0"
    return rec, None


def expected_choice(online, trial_units):
    """(position, entry) of the first entry of tests/online.json's order whose unit is not one of `trial_units`, or
    (None, None) when none is left."""
    for pos, entry in enumerate(online["order"]):
        if entry["unit"] not in trial_units:
            return pos, entry
    return None, None


def _number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def record_metrics(rec):
    """The record's counters, how the episode ended and the per-episode sub-metrics, as finite numbers."""
    out = {}
    for key in ("steps", "steps_rejected", "sim_time_s", "requests", "wall_s_since_start", "started_after_s",
                "order_index"):
        if _number(rec.get(key)):
            out[key] = float(rec[key])
    out["episode_ended"] = 1.0 if rec.get("ended") is True else 0.0
    for reason in END_REASONS:
        out["end_" + reason] = 1.0 if rec.get("end_reason") == reason else 0.0
    out["privileged"] = 1.0 if rec.get("privileged") is True else 0.0
    units = rec.get("trial_units")
    out["trial_units_count"] = float(len(units)) if isinstance(units, list) else 0.0
    metrics = rec.get("metrics")
    if isinstance(metrics, dict):
        for key, v in sorted(metrics.items()):
            if isinstance(key, str) and _number(v) and key not in out:
                out[key] = float(v)
    return out


def main():
    secured = {"root_path_dirs_secured": float(len(secure_exec_path()))}   # before anything else runs
    config = stage_config()
    anchor, meta = read_anchor()
    with open(ONLINE) as fh:
        online = json.load(fh)
    try:
        uid, gid = _agent_ids()
    except KeyError:
        write_verdict(zero_verdict(reason="no agent account", anchored=anchor is not None, extra=secured))
        return
    if not stop_agent_processes(uid):
        write_verdict(zero_verdict(reason="agent processes could not be stopped (cleanup_failed)",
                                   anchored=anchor is not None, extra=secured))
        return
    rec, reason = read_record(RECORD, online)
    cleanup_ok = stop_agent_processes(uid)
    if rec is None:
        write_verdict(zero_verdict(reason=reason, anchored=anchor is not None, extra=secured))
        return
    measured = float(rec["episode_score"])
    if not cleanup_ok:
        measured = 0.0
    extra = dict(record_metrics(rec), cleanup_failed=0.0 if cleanup_ok else 1.0, **secured)
    verdict = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5), higher_is_better=True,
                            metric_name=METRIC, locks_ok=True, extra=extra, full_at=meta.get("full_at", 1.0))
    diagnostic = "graded episode {}: end_reason={} steps={} sim_time_s={} requests={} episode_score={:.4f}".format(
        rec["episode"], rec.get("end_reason"), rec.get("steps"), rec.get("sim_time_s"), rec.get("requests"),
        measured)
    write_verdict(verdict, diagnostic=diagnostic)
    log("{} anchor={} ratio={:.3f} reward={:.3f}".format(diagnostic, anchor, verdict["metrics"]["ratio"],
                                                         verdict["rewards"]["reward"]))


if __name__ == "__main__":
    main()
