"""The online item of house_cleaning_with_tools's dual mode (grader side, stdlib only): the graded episode's record and the
program frozen at its start.

The agent plays ONE hidden graded episode during its session through the graded episode's cell service
(environment/setup/online/service.py: the house cell, a byte copy of tests/house_cell, served as the dedicated `arbcell`
account). The service writes the episode's record to /var/lib/arb-online/record.json; at the graded start, before the
start is charged, its root supervisor copies /home/user/submission into /var/lib/arb-online/submission_snapshot (the
freeze). The engine collects /var/lib/arb-online as root after the agent phase and restores it here root-owned (a
declared artifact, task.toml). verify.py uses this module:
  * load_record(): the record's checks (a regular file of at most 1 MB, valid JSON, format, task, episode and seed
    digest equal to tests/online.json, started, episode_score finite and >= 0);
  * snapshot_digest(): the digest the service recorded for the frozen program (snapshot_sha256), recomputed;
  * online_verdict(): the online item, robotics_grader.ratio_rewards of the episode's score against
    tests/anchor_online.json (a missing, invalid or never-started record: robotics_grader.zero_verdict).
"""

import hashlib
import json
import math
import os
import stat as statmod

RECORD = os.environ.get("ARB_ONLINE_RECORD", "/var/lib/arb-online/record.json")
STATE = os.path.join(os.path.dirname(RECORD), "state.json")
SNAPSHOT_DIR = os.path.join(os.path.dirname(RECORD), "submission_snapshot")
RECORD_FORMAT = "arb-online-record/1"
MAX_RECORD_BYTES = 1 << 20
END_REASONS = ("done", "finish", "graded_window", "worker_restart")
ONLINE_ANCHOR = "anchor_online.json"


def _number(v):
    """float(v) when v is a finite JSON number (not a bool), else None."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    v = float(v)
    return v if math.isfinite(v) else None


def record_exists():
    return os.path.lexists(RECORD)


def state_started():
    """True when the service's own state file says the graded episode was started (its charge is written before the
    first record): evidence of a start when the record itself is missing or unusable."""
    try:
        st = os.lstat(STATE)
        if not statmod.S_ISREG(st.st_mode) or st.st_size > 65536:
            return False
        with open(STATE, "rb") as fh:
            state = json.loads(fh.read(65537).decode("utf-8"))
        return isinstance(state, dict) and state.get("started") is True
    except (OSError, ValueError, UnicodeDecodeError):
        return False


def load_record(online):
    """(record, None) for a record of the graded episode that was started, else (None, why the online item scores 0)."""
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


def snapshot_digest(root=SNAPSHOT_DIR):
    """(files, bytes, sha256) of the frozen program under `root`, as the service computes it: for every regular file in
    the sorted order of its relative path (POSIX separators), the line `<path>\\0<size>\\0<sha256 of its content>\\n`,
    and the sha256 of all the lines; symbolic links and special files are not part of it (the service copies none)."""
    entries = []
    if os.path.isdir(root) and not os.path.islink(root):
        for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
            dirnames.sort()
            for fn in filenames:
                p = os.path.join(dirpath, fn)
                st = os.lstat(p)
                if not statmod.S_ISREG(st.st_mode):
                    continue
                h = hashlib.sha256()
                with open(p, "rb") as fh:
                    for chunk in iter(lambda: fh.read(1 << 20), b""):
                        h.update(chunk)
                rel = os.path.relpath(p, root).replace(os.sep, "/")
                entries.append((rel, st.st_size, h.hexdigest()))
    entries.sort(key=lambda e: e[0].encode("utf-8", "surrogateescape"))
    total = hashlib.sha256()
    for rel, size, digest in entries:
        total.update(rel.encode("utf-8", "surrogateescape") + b"\0" + str(size).encode() + b"\0" + digest.encode()
                     + b"\n")
    return len(entries), sum(e[1] for e in entries), total.hexdigest()


def online_verdict(rec, why):
    """The online item's verdict: robotics_grader.ratio_rewards of the record's episode_score (the cell's stage score
    of the graded episode as it ended, 0 with a safety violation: the offline grader's per-episode score) against
    tests/anchor_online.json, or zero_verdict(why) without a usable record. A failure on the grading side that the
    service recorded (a camera the renderer could not draw, a state that could not be scored) is a task error, as
    offline."""
    from robotics_grader import fail, ratio_rewards, read_anchor, zero_verdict
    anchor, meta = read_anchor(ONLINE_ANCHOR)
    metric = meta.get("metric", "cleaning_score")
    if rec is None:
        v = zero_verdict(reason="online item: " + why, anchored=anchor is not None, extra={"record_ok": 0.0})
        v["cleaning_score"] = 0.0
        v["diagnostic"] = "online item 0: " + why
        return v
    if int(_number(rec.get("render_errors")) or 0) > 0:
        fail("the renderer failed on %d camera request(s) of the graded episode: an infrastructure failure, not scored"
             % int(_number(rec.get("render_errors")) or 0))
    if rec.get("score_error"):
        fail("the graded episode could not be scored: %s" % (rec.get("score_error"),))
    score = float(rec["episode_score"])
    extra = {}
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
    v = ratio_rewards(score, anchor, higher_is_better=meta.get("direction", "higher") == "higher", metric_name=metric,
                      locks_ok=True, extra=extra, full_at=meta.get("full_at", 1.0))
    v["cleaning_score"] = score
    v["diagnostic"] = ("online item: cleaning_score %.4f, the graded episode %s (%s at %.1f s of simulated time, %d "
                       "requests), 0 with a safety violation") % (
        score, rec.get("episode"), ("ended: " + str(rec.get("end_reason"))) if rec.get("ended")
        else "running when the session ended", float(_number(rec.get("sim_time_s")) or 0.0),
        int(_number(rec.get("requests")) or 0))
    return v
