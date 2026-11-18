"""The grader of the online variant. Runs as ROOT in the verify stage, in a fresh verifier sandbox (separate
verification); it runs no program and needs no simulator.

The agent played the graded episode itself, during its session, through the graded episode service
(environment/setup/online/server.py): a service of another account that held the hidden episode root-only,
ran the offline verifier's own simulator (env.py, byte-identical to the copy in this folder) and per-episode
scorer (messy_desk_score.EpisodeScorer), and kept the episode's record in /var/lib/arb-online, a directory
the agent could neither read nor write. The engine collected that directory as a declared artifact after
the agent phase and restored it, root-owned, into this sandbox. This grader reads the record and nothing
else of the agent's (the submission folder is not read).

Checks: a regular file of at most 1 MB (not a link), valid JSON, `format`, `task`, `episode` and
`seed_sha256` equal to online.json in this folder, `started` true, `episode_score` a finite number in
[0, 1]. A missing, invalid or never-started record scores 0 (an untouched session never starts the
episode, so it leaves none). Otherwise the record's `episode_score` (S = O * R of that one episode, scaled
by steps / horizon when it ended before its horizon, as the offline grader scores an episode it cuts) is
the measured `mean_episode_score` (the mean over one episode), scored against tests/anchor.json.
"""

import json
import math
import os
import stat
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robotics_grader import (  # noqa: E402
    fail,
    ratio_rewards,
    read_anchor,
    stage_config,
    stage_dir,
    write_verdict,
    zero_verdict,
)

FORMAT = "arb-online-record/1"
END_REASONS = ("done", "finish", "graded_window", "worker_restart")
# the record's per-episode sub-metrics that ride in the verdict (numbers only)
METRIC_KEYS = ("organization", "request_progress", "request_rate", "score_unscaled", "requests_opened",
               "requests_success", "requests_failed", "requests_delivered", "requests_expired", "absent_correct",
               "absent_wrong", "claims_wrong", "first_holds", "objects_lost", "arrivals", "containers_cleared",
               "disturbances", "org_num", "org_den", "episode_cut", "hires_frames", "bad_actions",
               "privileged_mode", "oracle_token_mismatch")


def _reject_constant(name):
    raise ValueError("non-finite constant {}".format(name))


def load_online(path=None):
    path = path or os.path.join(stage_dir(), "online.json")
    try:
        with open(path) as f:
            spec = json.load(f)
        for key in ("task", "episode", "seed_sha256", "record", "max_record_bytes"):
            spec[key]
    except (IOError, OSError, ValueError, KeyError, TypeError) as exc:
        fail("cannot read the graded episode's identity {}: {}".format(path, exc))
    return spec


def load_record(path, max_bytes):
    """(record, None) or (None, why): a regular file, not a link, at most max_bytes, one JSON object."""
    try:
        fd = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0))
    except OSError as exc:
        return None, "no record of the graded episode at {} ({}): it was never started".format(
            path, exc.strerror or exc)
    try:
        st = os.fstat(fd)
        if not stat.S_ISREG(st.st_mode):
            return None, "the record is not a regular file"
        if st.st_size > max_bytes:
            return None, "the record is larger than {} bytes".format(max_bytes)
        data = os.read(fd, max_bytes + 1)
    finally:
        os.close(fd)
    try:
        rec = json.loads(data.decode("utf-8"), parse_constant=_reject_constant)
    except (UnicodeDecodeError, ValueError) as exc:
        return None, "the record is not valid JSON: {}".format(exc)
    if not isinstance(rec, dict):
        return None, "the record is not a JSON object"
    return rec, None


def check_record(rec, online):
    """None when the record belongs to this task's graded episode and was started, else why not."""
    for key, want in (("format", FORMAT), ("task", online["task"]), ("episode", online["episode"]),
                      ("seed_sha256", online["seed_sha256"])):
        if rec.get(key) != want:
            return "the record's {} is {!r}, not {!r}".format(key, rec.get(key), want)
    if rec.get("started") is not True:
        return "the record says the graded episode was never started"
    score = rec.get("episode_score")
    if isinstance(score, bool) or not isinstance(score, (int, float)) or not math.isfinite(float(score)):
        return "the record's episode_score is not a finite number"
    if not 0.0 <= float(score) <= 1.0:
        return "the record's episode_score {} lies outside [0, 1]".format(score)
    return None


def _number(v):
    return not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(float(v))


def verdict_extra(rec):
    reason = rec.get("end_reason")
    extra = {"end_" + r: 1.0 if reason == r else 0.0 for r in END_REASONS}
    extra["end_running"] = 1.0 if reason is None else 0.0
    extra["record_ended"] = 1.0 if rec.get("ended") is True else 0.0
    for key in ("steps", "sim_time_s", "requests", "wall_s_since_start"):
        if _number(rec.get(key)):
            extra[key] = float(rec[key])
    metrics = rec.get("metrics") if isinstance(rec.get("metrics"), dict) else {}
    for key in METRIC_KEYS:
        if _number(metrics.get(key)):
            extra[key] = float(metrics[key])
    return extra


def main():
    config = stage_config()
    anchor, anchor_meta = read_anchor()
    online = load_online()
    rec, why = load_record(online["record"], int(online["max_record_bytes"]))
    if why is None:
        why = check_record(rec, online)
    if why is not None:
        write_verdict(zero_verdict(reason=why, anchored=anchor is not None))
        return
    measured = float(rec["episode_score"])
    extra = verdict_extra(rec)
    verdict = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5),
                            higher_is_better=anchor_meta.get("direction", "higher") == "higher",
                            metric_name="mean_episode_score", locks_ok=True, extra=extra,
                            full_at=anchor_meta.get("full_at", 1.0))
    diagnostic = ("graded episode {}: episode_score {:.6f} after {} of {} steps, end_reason {}, {} requests, "
                  "{:.0f} s of wall time since its start; organization {}, request_progress {}".format(
                      rec.get("episode"), measured, rec.get("steps"), rec.get("horizon"), rec.get("end_reason"),
                      rec.get("requests"), float(rec.get("wall_s_since_start") or 0.0),
                      (rec.get("metrics") or {}).get("organization"),
                      (rec.get("metrics") or {}).get("request_progress")))
    write_verdict(verdict, diagnostic=diagnostic)
    print("[grader] {} measured={:.4f} anchor={} ratio={:.3f} reward={:.3f}".format(
        diagnostic, measured, anchor, verdict["metrics"]["ratio"], verdict["rewards"]["reward"]),
        file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
