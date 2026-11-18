"""Grader of shoe_lacing_and_boxing, online variant (the graded episode): runs as ROOT in the verify stage.

Verification is separate (task.toml): this runs in a fresh verifier sandbox from the task image, with only the declared
artifacts restored into it. No program runs here. During the agent's session the agent played ONE graded episode
itself through the graded service (environment/setup/online/server.py: the offline grader's own cell,
shoecell.server.ShoeCellServer in mode "hidden" with the grader's configuration, on the episode drawn from the offline
verifier's hidden set, tests/online.json), which ran as its own account and kept the episode's record in
/var/lib/arb-online, a directory the agent could neither read nor write; the engine collected that directory as root
after the agent phase and restored it here.

This grader reads /var/lib/arb-online/record.json and checks it: a regular file of at most 1 MB, valid JSON, `format`,
`task`, `episode` and `seed_sha256` equal to tests/online.json, `started` true, `episode_score` a finite number >= 0.
A missing record (the graded episode was never started: the untouched sandbox), an invalid one or one that never
started scores 0 (zero_verdict, with the reason). Otherwise the score is the record's `episode_score`: the cell's
work score of the graded episode (the accepted work of its pair over the work order, voided by a safety violation),
computed by the cell's own code when the episode ended (its own end, the agent's finish, the graded window), or as it
stood after the agent's last request when the session ended first, or after the last record when the service's
worker restarted. It is related to the reference value exactly as the offline grader relates the mean over its
episodes: robotics_grader.ratio_rewards against tests/anchor.json (metric work_score, higher, full_at).
"""

import json
import math
import os
import stat
import sys

sys.dont_write_bytecode = True

STAGE_DIR = os.environ.get("ALE_STAGE_DIR", os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, STAGE_DIR)

from robotics_grader import ratio_rewards, read_anchor, write_verdict, zero_verdict  # noqa: E402

RECORD_DIR = os.environ.get("ARB_ONLINE_RECORD_DIR", "/var/lib/arb-online")
RECORD = os.path.join(RECORD_DIR, "record.json")
MAX_RECORD_BYTES = 1 << 20
END_REASONS = ("done", "finish", "graded_window", "worker_restart")


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def _number(v):
    """A finite float from a JSON number (not a bool), else None."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    v = float(v)
    return v if math.isfinite(v) else None


def load_record(online):
    """(record, None) when the record is usable, else (None, why)."""
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
    for key in ("requests", "steps", "sim_time_s", "wall_s_since_start", "seq"):
        v = _number(rec.get(key))
        if v is not None:
            out["record_" + key if key == "seq" else key] = v
    metrics = rec.get("metrics") if isinstance(rec.get("metrics"), dict) else {}
    for k, v in sorted(metrics.items()):
        v = _number(v)
        if v is not None and isinstance(k, str) and k != "work_score" and len(k) <= 64:
            out["episode_" + k.replace(".", "_").replace("/", "_")] = v
    return out


def _write(verdict, text=None):
    """In the engine (ALE_VERIFY_PYTHON set) through the kit's ale_verify path; elsewhere (a host-side run) the kit's
    envelope to ROBOTICS_GRADER_VERDICT_PATH or verdict.local.json in the stage folder."""
    if os.environ.get("ALE_VERIFY_PYTHON"):
        write_verdict(verdict, diagnostic=text)
    else:
        write_verdict(verdict, path=os.environ.get("ROBOTICS_GRADER_VERDICT_PATH") or os.path.join(STAGE_DIR, "verdict.local.json"),
                      diagnostic=text)


def main():
    with open(os.path.join(STAGE_DIR, "online.json")) as fh:
        online = json.load(fh)
    anchor, meta = read_anchor()
    rec, why = load_record(online)
    if rec is None:
        log("[grader] " + why)
        _write(zero_verdict(reason=why, anchored=anchor is not None, extra={"record_present": 0.0 if "no record" in why else 1.0}))
        return
    score = float(rec["episode_score"])
    extra = record_stats(rec)
    verdict = ratio_rewards(score, anchor, higher_is_better=meta.get("direction", "higher") == "higher",
                            metric_name=meta.get("metric", "work_score"), locks_ok=True, extra=extra,
                            full_at=meta.get("full_at", 1.0))
    m = rec.get("metrics") if isinstance(rec.get("metrics"), dict) else {}
    milestones = ", ".join("%s %.2f" % (k, _number(m.get("milestone_" + k)) or 0.0)
                           for k in ("lacing_a", "lacing_b", "erected", "packed", "closed", "shipped"))
    text = ("graded episode %s: work_score %.4f (%s; safety_ok %s); ended %s (%s) at %.1f s simulated after %s requests "
            "(%s steps)") % (rec["episode"], score, milestones, bool(_number(m.get("safety_ok"))), rec.get("ended") is True,
                             rec.get("end_reason"), _number(rec.get("sim_time_s")) or 0.0, rec.get("requests"), rec.get("steps"))
    log("[grader] " + text)
    _write(verdict, text)


if __name__ == "__main__":
    main()
