"""The grader of the dual mode. Runs as ROOT in the verify stage, in a fresh verifier sandbox (separate
verification) that holds the two declared artifacts: the agent's submission folder (/home/user/submission, as the
session left it) and the graded episode service's record directory (/var/lib/arb-online: the record, and the
snapshot of the submission the service froze at the graded start).

Two scoring items, one reward, their mean:

- the offline item: main's grading of the program, unchanged. offline_verify.py in this folder is byte-identical
  to main's tests/verify.py (the run.sh controller on the hidden episodes of episodes.json, each in an Isaac Sim
  process of its own, the controller budget, the probe lock, S = O * R); offline_item() below is main's main()
  with the verdict returned instead of written and main's anchor read from anchor_offline.json. The program it
  grades is the one frozen at the graded start (the record's snapshot, checked against the record's digest), or,
  when the episode was never started, the submission folder as the session left it, as on main.
- the online item: the graded episode's record, scored as in the online variant (anchor_online.json).

reward = (offline reward + online reward) / 2, each item's exact kit reward with its own anchor, cap and full_at;
the verdict goes out once, through ratio_rewards against anchor.json (dual_mode_score, 1.0 by construction).
"""

import hashlib
import json
import math
import os
import shutil
import stat
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from offline_verify import (  # noqa: E402  (main's offline grader, byte-identical to main's tests/verify.py)
    Solver,
    SolverAborted,
    SolverMissing,
    load_episodes,
    run_episodes,
    token_state,
)
from robotics_grader import (  # noqa: E402
    fail,
    ratio_rewards,
    read_anchor,
    stage_config,
    stage_dir,
    write_verdict,
    zero_verdict,
)
from robotics_grader.stage import agent_home  # noqa: E402

OFFLINE_ANCHOR = "anchor_offline.json"
ONLINE_ANCHOR = "anchor_online.json"
FORMAT = "arb-online-record/1"
END_REASONS = ("done", "finish", "graded_window", "worker_restart")
# the record's per-episode sub-metrics that ride in the verdict (numbers only)
METRIC_KEYS = ("organization", "request_progress", "request_rate", "score_unscaled", "requests_opened",
               "requests_success", "requests_failed", "requests_delivered", "requests_expired", "absent_correct",
               "absent_wrong", "claims_wrong", "first_holds", "objects_lost", "arrivals", "containers_cleared",
               "disturbances", "org_num", "org_den", "episode_cut", "hires_frames", "bad_actions",
               "privileged_mode", "oracle_token_mismatch")
PROGRAMS = ("session_end", "snapshot", "snapshot_too_large", "snapshot_mismatch", "record_invalid",
            "record_dir_missing")


# ----------------------------------------------------------------------------------------------------------
# the online record (the online variant's checks, O9)

def _reject_constant(name):
    raise ValueError("non-finite constant {}".format(name))


def load_online(path=None):
    path = path or os.path.join(stage_dir(), "online.json")
    try:
        with open(path) as f:
            spec = json.load(f)
        for key in ("task", "episode", "seed_sha256", "record", "max_record_bytes", "snapshot"):
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


def online_extra(rec):
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


def online_item(config, rec):
    """The online item's verdict: the record's episode_score against the online anchor."""
    anchor, meta = read_anchor(ONLINE_ANCHOR)
    return ratio_rewards(float(rec["episode_score"]), anchor, cap=config.get("scoring_cap", 1.5),
                         higher_is_better=meta.get("direction", "higher") == "higher",
                         metric_name="mean_episode_score", locks_ok=True, extra=online_extra(rec),
                         full_at=meta.get("full_at", 1.0))


# ----------------------------------------------------------------------------------------------------------
# the frozen program

def snapshot_digest(root):
    """The service's digest of a snapshot folder (environment/setup/online/server.py snapshot_submission): the
    sha256 of the sorted lines "<path>\\0<size>\\0<sha256 of the content>" of its regular files. None when the
    folder is missing or holds anything but regular files and directories (a snapshot never does)."""
    if not os.path.isdir(root) or os.path.islink(root):
        return None
    lines = []
    for dirpath, dirnames, filenames in os.walk(root):
        for name in dirnames:
            if os.path.islink(os.path.join(dirpath, name)):
                return None
        for name in filenames:
            path = os.path.join(dirpath, name)
            st = os.lstat(path)
            if not stat.S_ISREG(st.st_mode):
                return None
            h = hashlib.sha256()
            with open(path, "rb") as f:
                for chunk in iter(lambda: f.read(1 << 20), b""):
                    h.update(chunk)
            rel = os.path.relpath(path, root).replace(os.sep, "/")
            lines.append("%s\0%d\0%s\n" % (rel, st.st_size, h.hexdigest()))
    return hashlib.sha256("".join(sorted(lines)).encode("utf-8", "surrogateescape")).hexdigest()


def install_snapshot(src, dst):
    """Put the frozen program where main's grading looks for it: the restored session-end folder is replaced by
    a copy of the snapshot (modes kept, root-owned, as the engine restores an artifact)."""
    if os.path.lexists(dst):
        if os.path.isdir(dst) and not os.path.islink(dst):
            shutil.rmtree(dst)
        else:
            os.unlink(dst)
    shutil.copytree(src, dst, symlinks=True)


# ----------------------------------------------------------------------------------------------------------
# the offline item: main's main(), returning the verdict, with main's anchor from anchor_offline.json

def offline_item(submission):
    config = stage_config()
    anchor, anchor_meta = read_anchor(OFFLINE_ANCHOR)
    seeds, probe_key = load_episodes()
    # read before any of the submission's code runs
    privileged, mismatch = token_state(submission)
    if privileged or mismatch:
        print("[grader] {}".format("privileged mode (oracle token)" if privileged else "oracle token mismatch: ignored"),
              file=sys.stderr, flush=True)
    flags = {"privileged_mode": float(privileged), "oracle_token_mismatch": float(mismatch)}

    try:
        solver = Solver(step_timeout=config.get("step_timeout_s", 30), submission=submission)
    except (SolverMissing, SolverAborted) as exc:
        # An untouched sandbox, or a submission that never comes up, is a real all-zero
        # verdict, not a task error; and no simulator is launched for it.
        return zero_verdict(reason=str(exc), anchored=anchor is not None, extra=flags)

    try:
        scores, summaries, aborted, cut, lock = run_episodes(config, seeds, probe_key, solver, privileged=privileged)
    finally:
        solver.close()

    measured = sum(scores) / len(scores) if scores else 0.0
    extra = dict(lock.rewards(), episodes_aborted=float(aborted), episodes_cut=float(cut), **flags)
    for i, s in enumerate(scores):
        extra["episode_{}_score".format(i)] = float(s)
    for i, s in enumerate(summaries):
        extra["episode_{}_controller_s".format(i)] = float(s.get("controller_s", 0.0))
    for key in ("requests_success", "requests_failed", "requests_delivered", "requests_expired", "absent_correct",
                "absent_wrong", "claims_wrong", "first_holds", "objects_lost"):
        extra["total_" + key] = float(sum(s.get(key, 0) for s in summaries))
    for key in ("organization", "request_rate", "request_progress"):
        vals = [s[key] for s in summaries if key in s]
        extra["mean_" + key] = float(sum(vals) / len(vals)) if vals else 0.0
    verdict = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5),
                            higher_is_better=anchor_meta.get("direction", "higher") == "higher",
                            metric_name="mean_episode_score", locks_ok=lock.ok, extra=extra,
                            full_at=anchor_meta.get("full_at", 1.0))
    print("[grader] offline item: measured={:.4f} anchor={} ratio={:.3f} reward={:.3f} lock={} aborted={} cut={} "
          "privileged={}".format(measured, anchor, verdict["metrics"]["ratio"], verdict["rewards"]["reward"], lock.ok,
                                 aborted, cut, privileged), file=sys.stderr, flush=True)
    return verdict


# ----------------------------------------------------------------------------------------------------------

def main():
    config = stage_config()
    online = load_online()
    # 1. Agent processes: none run here (a fresh verifier sandbox, separate verification); main's grading
    #    reclaims root's PATH before any agent code runs (the kit's deprivileged runner), as on main.
    # 2. The online record (O9).
    record_dir = os.path.dirname(online["record"])
    record_dir_here = os.path.isdir(record_dir) and os.path.lexists(os.path.join(record_dir, "service.log"))
    record_present = record_dir_here and os.path.lexists(online["record"])
    rec, why = (load_record(online["record"], int(online["max_record_bytes"])) if record_present
                else (None, "no record of the graded episode: it was never started"))
    if rec is not None:
        why = check_record(rec, online)
    started = rec is not None and why is None
    # 3. The program: frozen at the start, or the session-end folder when the episode never started.
    submission = os.path.join(agent_home(), "submission")
    snapshot = online["snapshot"]
    if not record_dir_here:
        program, note = "record_dir_missing", "the graded episode's record directory did not reach the verifier"
    elif not record_present:
        program, note = "session_end", "the episode was never started: the submission as the session left it"
    elif not started:
        program, note = "record_invalid", "the record is invalid ({}): no program is graded".format(why)
    elif rec.get("snapshot_status") == "too_large":
        program, note = "snapshot_too_large", "the submission frozen at the start was too large ({})".format(
            rec.get("snapshot_note"))
    elif rec.get("snapshot_status") == "ok" and snapshot_digest(snapshot) == rec.get("snapshot_sha256"):
        install_snapshot(snapshot, submission)
        program, note = "snapshot", "the submission frozen at the start ({} files, {} bytes)".format(
            rec.get("snapshot_files"), rec.get("snapshot_bytes"))
    else:
        program, note = "snapshot_mismatch", "the frozen submission does not match the record's digest"
    print("[grader] program graded: {}".format(note), file=sys.stderr, flush=True)
    # 4. The offline item: main's grading.
    if program in ("session_end", "snapshot"):
        offline = offline_item(submission)
    else:
        offline = zero_verdict(reason=note, anchored=True)
    # 5. The online item: the record.
    online_v = online_item(config, rec) if started else zero_verdict(reason=why, anchored=True)
    # 6. The mean of the two exact rewards.
    off_r, on_r = float(offline["rewards"]["reward"]), float(online_v["rewards"]["reward"])
    combined = (off_r + on_r) / 2.0
    # 7. One verdict, through the kit, against the combined anchor (1.0).
    anchor, meta = read_anchor()
    extra = {"offline_reward": off_r, "online_reward": on_r}
    extra.update({"program_" + p: 1.0 if program == p else 0.0 for p in PROGRAMS})
    for key in ("snapshot_files", "snapshot_bytes", "snapshot_skipped"):
        if rec is not None and _number(rec.get(key)):
            extra[key] = float(rec[key])
    for key, value in offline["metrics"].items():
        extra["offline_" + key] = float(value)
    for key, value in online_v["metrics"].items():
        extra["online_" + key] = float(value)
    verdict = ratio_rewards(combined, anchor, cap=1.0, higher_is_better=meta.get("direction", "higher") == "higher",
                            metric_name="dual_mode_score", locks_ok=True, extra=extra,
                            full_at=meta.get("full_at", 1.0))
    diagnostic = ("dual mode: reward {:.6f} = (offline {:.6f} + online {:.6f}) / 2. Offline item: {}{}. "
                  "Online item: {}.".format(
                      verdict["rewards"]["reward"], off_r, on_r, note,
                      "" if "mean_episode_score" not in offline["metrics"] else
                      ", mean_episode_score {:.6f}".format(offline["metrics"]["mean_episode_score"]),
                      "graded episode {} episode_score {:.6f} after {} of {} steps, end_reason {}".format(
                          rec.get("episode"), float(rec["episode_score"]), rec.get("steps"), rec.get("horizon"),
                          rec.get("end_reason")) if started else why))
    write_verdict(verdict, diagnostic=diagnostic)
    print("[grader] " + diagnostic, file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
