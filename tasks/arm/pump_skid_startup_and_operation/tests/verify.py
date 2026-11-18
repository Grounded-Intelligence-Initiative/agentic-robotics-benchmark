"""Grader of pump_skid_startup_and_operation, online variant (experimental): runs as ROOT in the verify stage, in a
separate verifier sandbox (task.toml), and runs no program.

The agent played ONE graded episode itself during its session, through the graded episode's service
(environment/setup/online/graded_service.py: a hidden episode of this grader's fixed set, hidden/seeds.json, on the
grader's own cell, under its own account; the agent can neither read nor write its record). The service wrote the
episode's record to /var/lib/arb-online/record.json; the engine collected that directory as a declared artifact
(task.toml `artifacts`) as root after the agent phase and restored it, root-owned, into this sandbox. This grader:
  1. reads the record: a regular file of at most 1 MB, strict JSON (no NaN / Infinity), an object;
  2. cross-checks it with online.json: `format`, `task`, `episode` and `seed_sha256` equal, `started` true, and
     `episode_score` a finite number >= 0;
  3. scores: operation_score = the record's `episode_score`, the shift's score of skidcell.episode for that one episode
     (start-up 0.20, operation in band 0.25, the window's events 0.25, shutdown 0.15, the operator log 0.15; a void
     scores it 0), which the service computed as the episode stood when it ended (the agent's finish, the cell's own
     end, the graded window) or, should the session have ended first, after the agent's last request; then
     ratio_rewards against the anchor, as the offline grader scored the mean over its episodes.
A missing record (the episode was never started: the untouched sandbox), an invalid one, or one that is not this
task's graded episode scores 0 (zero_verdict, with the reason). Nothing else of the agent is read: the submission
folder is not graded.
"""

import json
import math
import os
import stat
import sys

sys.dont_write_bytecode = True

STAGE_DIR = os.environ.get("ALE_STAGE_DIR", os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, STAGE_DIR)

from robotics_grader import ratio_rewards, read_anchor, zero_verdict  # noqa: E402
from robotics_grader import write_verdict as kit_write_verdict  # noqa: E402

RECORD_MAX_BYTES = 1 << 20
END_REASONS = ("done", "finish", "graded_window", "worker_restart")


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def _reject_constant(name):
    raise ValueError("non-finite number %s" % name)


def read_record(path):
    """(record, None) or (None, why it cannot be scored)."""
    try:
        st = os.lstat(path)
    except FileNotFoundError:
        return None, "no graded-episode record at %s: the graded episode was never started" % path
    except OSError as e:
        return None, "the record at %s is unreadable: %s" % (path, e)
    if not stat.S_ISREG(st.st_mode):
        return None, "the record at %s is not a regular file" % path
    if st.st_size > RECORD_MAX_BYTES:
        return None, "the record at %s is larger than 1 MB" % path
    try:
        with open(path, "rb") as f:
            raw = f.read(RECORD_MAX_BYTES + 1)
        rec = json.loads(raw.decode("utf-8"), parse_constant=_reject_constant)
    except (OSError, ValueError, UnicodeDecodeError) as e:
        return None, "the record at %s is not valid JSON: %s" % (path, str(e)[:200])
    if not isinstance(rec, dict):
        return None, "the record at %s is not a JSON object" % path
    return rec, None


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


def _number(v):
    if isinstance(v, bool):
        return 1.0 if v else 0.0
    if isinstance(v, (int, float)) and math.isfinite(float(v)):
        return float(v)
    return None


def write_verdict(verdict, diagnostic=None):
    """In the engine (ALE_VERIFY_PYTHON set) the kit writes reward.json and verification.json; elsewhere the envelope
    goes to verdict.local.json in the stage folder (or ROBOTICS_GRADER_VERDICT_PATH)."""
    if os.environ.get("ALE_VERIFY_PYTHON"):
        kit_write_verdict(verdict, diagnostic=diagnostic)
        return
    path = os.environ.get("ROBOTICS_GRADER_VERDICT_PATH") or os.path.join(STAGE_DIR, "verdict.local.json")
    if diagnostic:
        verdict = dict(verdict, diagnostic=diagnostic)
    kit_write_verdict(verdict, path=path)


def main():
    with open(os.path.join(STAGE_DIR, "online.json")) as f:
        online = json.load(f)
    path = os.environ.get("ARB_ONLINE_RECORD") or online.get("record") or "/var/lib/arb-online/record.json"
    anchor, meta = read_anchor()
    rec, why = read_record(path)
    if rec is not None:
        why = check_record(rec, online)
    if why:
        log("[grader] " + why)
        write_verdict(zero_verdict(reason=why, anchored=anchor is not None, extra={"record_valid": 0.0}))
        return
    score = max(0.0, min(1.0, float(rec["episode_score"])))
    end = rec.get("end_reason")
    extra = {"record_valid": 1.0, "record_ended": 1.0 if rec.get("ended") is True else 0.0}
    for r in END_REASONS + (None,):
        extra["end_reason_" + (r or "running")] = 1.0 if end == r else 0.0
    for k in ("steps", "sim_time_s", "requests", "wall_s_since_start"):
        v = _number(rec.get(k))
        if v is not None:
            extra[k] = v
    for k, v in sorted((rec.get("metrics") or {}).items()):
        v = _number(v)
        name = str(k).replace(".", "_").replace("/", "_")
        if v is not None and name not in ("ratio", "anchor_recorded", "reward_exact", meta.get("metric", "operation_score")):
            extra[name] = v
    verdict = ratio_rewards(score, anchor, higher_is_better=meta.get("direction", "higher") == "higher",
                            metric_name=meta.get("metric", "operation_score"), locks_ok=True, extra=extra,
                            full_at=meta.get("full_at", 1.0))
    text = ("graded episode %s: ended %s (%s) at sim %.1f s after %s requests and %s step calls; operation_score %.4f "
            "(the shift's score: start-up, operation in band, the window's events, shutdown, the operator log; a void "
            "scores 0)") % (rec.get("episode"), rec.get("ended"), end, float(extra.get("sim_time_s", 0.0)),
                            rec.get("requests"), rec.get("steps"), score)
    log("[grader] " + text)
    write_verdict(verdict, diagnostic=text)


if __name__ == "__main__":
    main()
