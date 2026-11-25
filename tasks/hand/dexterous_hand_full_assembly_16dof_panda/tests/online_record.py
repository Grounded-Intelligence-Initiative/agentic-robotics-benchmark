"""The online item of dexterous_hand_full_assembly_16dof_panda's dual mode (grader side, stdlib only): the graded
episode's record and the program frozen at its start.

The agent plays ONE hidden graded episode during its session through the graded episode's cell service
(environment/setup/online/service.py: the hand cell, a byte copy of this folder's cell code, served as the dedicated
`arbcell` account). The service writes the episode's record to /var/lib/arb-online/record.json; at the earlier of the
graded start (before the start is charged) and the start window's close, its root supervisor copies
/home/user/submission into /var/lib/arb-online/submission_snapshot (the freeze). The engine collects /var/lib/arb-online
as root after the agent phase and restores it here root-owned (a declared artifact, task.toml). dual_mode.py uses
this module:
  * read_record(): the record's checks (a regular file of at most 1 MB, valid JSON, format, task, episode and seed
    digest equal to tests/online.json), for a started episode or a start window that closed without a start (the
    record then says prep_window_closed: the program was frozen at that moment);
  * load_record(): read_record() plus started and episode_score finite and >= 0 (the online item);
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


def _no_constants(name):
    raise ValueError("non-finite constant")


def read_record(online):
    """(record, None) for a usable record of the graded episode (started, or its start window closed without a start),
    else (None, why it is unusable or missing)."""
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
        fd = os.open(RECORD, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        try:
            raw = os.read(fd, MAX_RECORD_BYTES + 1)
        finally:
            os.close(fd)
        rec = json.loads(raw.decode("utf-8"), parse_constant=_no_constants)
    except (OSError, ValueError, UnicodeDecodeError) as e:
        return None, "the record %s is not valid JSON (%s)" % (RECORD, e.__class__.__name__)
    if not isinstance(rec, dict):
        return None, "the record %s is not a JSON object" % RECORD
    for key, want in (("format", RECORD_FORMAT), ("task", online.get("task")), ("episode", online.get("episode")),
                      ("seed_sha256", online.get("seed_sha256"))):
        if rec.get(key) != want:
            return None, "the record's %s is %r, not the graded episode's %r" % (key, rec.get(key), want)
    return rec, None


def load_record(online):
    """(record, None) for a record of the graded episode that was started, else (None, why the online item scores 0)."""
    rec, why = read_record(online)
    if rec is None:
        return None, why
    if rec.get("started") is not True:
        if rec.get("prep_window_closed") is True:
            return None, "the graded episode was never started (its start window closed first)"
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
    """The online item's verdict: robotics_grader.ratio_rewards of the record's episode_score (the episode's
    assembly_score as tests/verify.py scores one episode: the scope's fastened mates' weight over the scope's weight,
    times the safety factor after a violation) against tests/anchor_online.json, or zero_verdict(why) without a usable
    record. A record whose score the service could not compute (score_error) scores the item 0 with grader_exception 1,
    as main's grader scores a run whose scoring raised."""
    from robotics_grader import ratio_rewards, read_anchor, zero_verdict
    anchor, meta = read_anchor(ONLINE_ANCHOR)
    metric = meta.get("metric", "assembly_score")
    if rec is None:
        v = zero_verdict(reason="online item: " + why, anchored=anchor is not None, extra={"record_ok": 0.0})
        v["assembly_score"] = 0.0
        v["diagnostic"] = "online item 0: " + why
        return v
    if rec.get("score_error"):
        why = "the service could not score the graded episode (%s)" % (str(rec.get("score_error"))[:200],)
        v = zero_verdict(reason="online item: " + why, anchored=anchor is not None,
                         extra={"record_ok": 1.0, "grader_exception": 1.0})
        v["assembly_score"] = 0.0
        v["diagnostic"] = "online item 0: " + why
        return v
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
        "started_after_s": float(_number(rec.get("started_after_s")) or 0.0),
        "privileged_mode": 1.0 if rec.get("privileged") else 0.0,
        "cell_error": 1.0 if rec.get("cell_error") else 0.0,
    })
    for r in END_REASONS:
        extra["end_reason_" + r] = 1.0 if (rec.get("ended") and rec.get("end_reason") == r) else 0.0
    v = ratio_rewards(score, anchor, higher_is_better=meta.get("direction", "higher") == "higher", metric_name=metric,
                      locks_ok=True, extra=extra, full_at=meta.get("full_at", 1.0))
    v["assembly_score"] = score
    v["diagnostic"] = ("online item: assembly_score %.4f, the graded episode %s (%s at %.1f s of simulated time, %d "
                       "requests), an episode with a safety violation keeping %g of it") % (
        score, rec.get("episode"), ("ended: " + str(rec.get("end_reason"))) if rec.get("ended")
        else "running when the session ended", float(_number(rec.get("sim_time_s")) or 0.0),
        int(_number(rec.get("requests")) or 0), float(_number((rec.get("metrics") or {}).get("safety_violation_factor"))
                                                      or 0.0))
    return v
