"""Grader of freeflyer_uncrewed_station_caretaking, dual mode: two scoring items of one task, one session, one verify.
Runs as ROOT in the verify stage (separate verification: a fresh verifier sandbox holding only the declared artifacts,
restored root-owned: the submission and the graded episode's record directory; tests/test.sh built the root-owned
copy of the givens and named the asset root before this starts).

  * The program item (offline): main's grader, unchanged. tests/offline_verify.py is main's tests/verify.py with two
    hooks (the verdict is handed back instead of written; main(finish=False) leaves the stage modes and the freezer to
    this file); it runs the agent's caretake.py solve(robot) on the 6 hidden sites, as on main, with main's gates,
    budgets, lockdown, rerun rule and record. The program it grades is the one frozen when the graded episode started
    (/var/lib/arb-online/submission_snapshot: the graded episode's root supervisor copied the submission folder there
    before the start was charged), or when the preparation window closed if the episode was never started (a freeze at
    the window's close that failed, after the service's retries, scores the program item 0); the submission as the
    session left it (as on main) only when the session ended before the window closed or there is no usable record.
  * The live item (online): the ONE graded episode the agent flew itself during its session: a site of the hidden set
    (tests/online.json names it; tests/hidden/selection.json holds its package, seed and the reference's energy on
    it), judged by main's site grader (cabincell.grade) on the cell's record of the episode, which ends as a scored
    run ends when its program stops (the firmware's stop and hold, the station on for 60 s). When the service ended
    the episode (graded_finish, the site's simulated time, the graded window), the record names the cell's record
    (cell_record.json, sha256 checked) and that record is graded. When the session (or a failure of the service's
    worker) ended it first, the service's call log stands for it: this file re-simulates the episode in a fresh cell
    of its own (tests/cabincell, the site package, the grader's seed rule), every call of the log in order, checks
    each reply's code and mission time and the physics state's digest after it against the log (a difference is a task
    error), then ends it there and grades that record. 0 when the episode never started.
  * reward = the mean of the two components, each normalised by its own anchor, cap and full_at (anchor_offline.json
    is main's anchor.json; anchor_online.json the live item's), written once through ratio_rewards against
    tests/anchor.json (dual_mode_score; 1.0 by construction).

Order:
  1. stop the agent account's processes as main's verifier does (kill, then keep them frozen until the verdict;
     stdlib only, before anything else is imported);
  2. read and check the record (tests/online.json: format, task, episode, seed digest);
  3. pick the program: the snapshot when the record says the program was frozen (its digest checked against the
     record: a mismatch is a task error); none when the frozen submission was over main's size cap or the freeze at
     the window's close failed (the program item then scores 0); the restored submission only when the session ended
     before the window closed or the record is unusable;
  4. main's offline grading of that program (main's own lockdown runs on it: kill, freezer, stage lock, late-write
     gate, its root-owned snapshot, then the gates and the 6 sites): the offline component;
  5. the live component from the record (and the cell's record or the call log);
  6. reward = (offline_reward_exact + online_reward_exact) / 2;
  7. the verdict, written once; its metrics carry both components, both raw metrics, the freeze and the episode's
     end, and every metric of each component under "offline." / "online.".
Untouched sandbox: no program, no record -> 0. Oracle: the reference program frozen at the start (privileged offline
grading with its token, 1.0) and the privileged reference program on the graded site (1.0) -> 1.
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
END_REASONS = ("done", "finish", "graded_window", "worker_restart", "call_log_full")
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
    FREEZER[0] = verify_freeze.AgentFreezer.for_user(AGENT_USER).start()
    if out["agent_processes_left"]:
        running = _still_running(uid)
        if running:
            out["agent_processes_running"] = float(len(running))
            out["reasons"].append("%d process(es) of the agent account survived the kill at verify start and could "
                                  "not be stopped" % len(running))
        else:
            log("[grader] %d process(es) of the agent account outlived the kill at verify start: frozen until the "
                "verdict" % int(out["agent_processes_left"]))
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


def _read_regular(path, max_bytes, what):
    """The bytes of a regular file (no symlink) of at most `max_bytes`, or (None, reason)."""
    try:
        st = os.lstat(path)
    except FileNotFoundError:
        return None, "no %s at %s" % (what, path)
    except OSError as e:
        return None, "the %s cannot be examined: %s" % (what, e)
    if not statmod.S_ISREG(st.st_mode):
        return None, "the %s is not a regular file" % what
    if st.st_size > max_bytes:
        return None, "the %s is larger than %d bytes" % (what, max_bytes)
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
        with os.fdopen(fd, "rb") as fh:
            blob = fh.read(max_bytes + 1)
    except OSError as e:
        return None, "the %s cannot be read: %s" % (what, e)
    if len(blob) > max_bytes:
        return None, "the %s is larger than %d bytes" % (what, max_bytes)
    return blob, None


# ----------------------------------------------------------------------------------------------- step 2: the record
def read_record(path, expected, max_bytes):
    """(record, None) for a record of this task's graded episode (a regular file within the size limit, valid JSON,
    format, task, episode and seed digest equal to online.json), or (None, reason)."""
    blob, why = _read_regular(path, max_bytes, "record of the graded episode")
    if blob is None:
        return None, why
    try:
        rec = json.loads(blob.decode("utf-8"), parse_constant=_reject_constant)
    except (ValueError, UnicodeDecodeError) as e:
        return None, "the record is not valid JSON: %s" % (str(e)[:200],)
    if not isinstance(rec, dict):
        return None, "the record is not a JSON object"
    for key in ("format", "task", "episode", "seed_sha256"):
        if rec.get(key) != expected.get(key):
            return None, "the record's %s is %r, not %r" % (key, rec.get(key), expected.get(key))
    return rec, None


# ----------------------------------------------------------------------------------------------- step 3: the program
def tree_digest(files):
    """sha256 of the sorted file list with sizes and contents (environment/setup/online/graded_service.py computes the
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
    cap, or the freeze at the window's close failed). The snapshot is used when the record says the program was
    frozen, at the start or at the close of the preparation window; the submission as the session left it only when
    the session ended before the window closed (nothing frozen, nothing to freeze yet) or there is no usable record.
    A failed freeze never falls back to the session-end submission: that would give the program prep_s + graded_s."""
    restored = os.path.join(HOME, "submission")
    facts = {"program_snapshot": 0.0, "program_session_end": 0.0, "program_too_large": 0.0,
             "program_freeze_failed": 0.0}
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
    if frozen:                                    # the window closed and its freeze failed (or never ran)
        facts["program_freeze_failed"] = 1.0
        log("[grader] the freeze when the preparation window closed failed (snapshot_status %r after %s attempt(s)): "
            "the program item scores 0" % (status, rec.get("snapshot_attempts")))
        return None, facts
    facts["program_session_end"] = 1.0
    log("[grader] no freeze of the program on record (%s): the submission as the session left it" % (
        "no usable record" if rec is None else "snapshot_status %r" % (status,)))
    return restored, facts


# ----------------------------------------------------------------------------------------------- step 4: offline
def offline_component(program_dir):
    """(reward_exact, raw caretaking_score, metrics) of main's grading of the program at `program_dir` (None: 0, no
    grading). Main's write_verdict would score main's verdict against tests/anchor.json; here the same formula runs
    against anchor_offline.json (main's anchor.json, byte-identical), and main's honest zero stays a zero."""
    anchor, meta = read_anchor("anchor_offline.json")
    if program_dir is None:
        return 0.0, 0.0, {"contract_ok": 0.0, "caretaking_score": 0.0, "graded": 0.0}
    captured = {}

    def sink(rewards, metrics, contract_why, zero_reason):
        captured.update(rewards=dict(rewards), metrics=dict(metrics), contract_why=contract_why,
                        zero_reason=zero_reason)
        return float(rewards["caretaking_score"])

    OV.SUBMISSION = program_dir                   # the program main's grading copies, gates and runs
    OV.EARLY = OV._early_lockdown()               # main's lockdown, on that program
    OV.VERDICT_SINK[0] = sink
    OV.main(finish=False)                         # main's grading; a grader-side failure ends the run as a task error
    if "rewards" not in captured:
        fail("main's grading ended without a verdict")
    rewards, metrics = captured["rewards"], captured["metrics"]
    overall = float(rewards["caretaking_score"])
    extra = OV.finite_metrics(metrics)            # main's write_verdict: finite numbers, plus the gate and criterion
    extra.update({k: float(rewards[k]) for k in OV.GATES + OV.CRITERIA})
    if captured["zero_reason"]:
        log("[grader] program item: %s" % captured["zero_reason"])
        return 0.0, 0.0, dict(extra, graded=1.0, zero_verdict=1.0)
    verdict = ratio_rewards(overall, anchor, higher_is_better=meta.get("direction", "higher") == "higher",
                            metric_name=meta.get("metric", "caretaking_score"), extra=extra,
                            full_at=meta.get("full_at", 1.0))
    if captured["contract_why"] and not rewards["contract_ok"]:
        log("[grader] program item: the contract failed: %s" % str(captured["contract_why"])[:300])
    return float(verdict["rewards"]["reward"]), overall, dict(verdict["metrics"], graded=1.0)


# ----------------------------------------------------------------------------------------------- step 5: online
def state_digest(data):
    """sha256 of the physics state (qpos, qvel and the simulation time, little-endian float64), as the graded
    episode's service writes it into its call log after every call (environment/setup/online/graded_service.py)."""
    import numpy as np
    h = hashlib.sha256()
    h.update(np.ascontiguousarray(data.qpos, dtype="<f8").tobytes())
    h.update(np.ascontiguousarray(data.qvel, dtype="<f8").tobytes())
    h.update(np.asarray([data.time], dtype="<f8").tobytes())
    return h.hexdigest()


def graded_site(expected):
    """The graded episode's site of the hidden set: {tag, label, path, sha256, seed, ref_energy_wh} from
    hidden/selection.json (OV.hidden_set: the package checked against its recorded sha256), with its noise seed's
    digest checked against online.json."""
    sites = OV.hidden_set(int(OV.GR.load_runtime()["values"]["layout.sites"]))
    hits = [s for s in sites if s["label"] == expected["episode"]]
    if len(hits) != 1:
        fail("the graded episode %r is not one site of the hidden set" % (expected["episode"],))
    site = dict(hits[0])
    from cabincell import server as SV
    seed = site["seed"] if site["seed"] is not None else SV.noise_seed_for(site["sha256"])
    if hashlib.sha256(str(int(seed)).encode("ascii")).hexdigest() != expected["seed_sha256"]:
        fail("the graded site's noise seed does not match online.json's seed_sha256")
    site["noise_seed"] = int(seed)
    return site


def load_cell_record(rec, expected):
    """The cell's record of the episode that the service ended (cell_record.json, its sha256 as the record names it)."""
    meta = rec.get("cell_record")
    if not isinstance(meta, dict) or meta.get("file") != "cell_record.json":
        fail("the record of an ended graded episode names no cell record")
    path = os.path.join(expected.get("record_dir", "/var/lib/arb-online"), "cell_record.json")
    blob, why = _read_regular(path, int(expected.get("cell_record_max_bytes", 1 << 28)), "cell record")
    if blob is None:
        fail("the graded episode's cell record is unusable: %s" % why)
    if hashlib.sha256(blob).hexdigest() != meta.get("sha256"):
        fail("the graded episode's cell record does not match the sha256 its record names")
    try:
        return json.loads(blob.decode("ascii"))
    except (ValueError, UnicodeDecodeError) as e:
        fail("the graded episode's cell record is not valid JSON: %s" % str(e)[:200])


def read_calls(expected):
    """The service's call log: [(n, method, params, code, t, digest)] in order. A last line that is not complete
    (the session ended while it was being written: its call was never answered) is left out; any other bad line is a
    task error."""
    path = os.path.join(expected.get("record_dir", "/var/lib/arb-online"), "calls.jsonl")
    if not os.path.lexists(path):
        return []
    blob, why = _read_regular(path, int(expected.get("calls_max_bytes", 1 << 28)), "call log")
    if blob is None:
        fail("the graded episode's call log is unusable: %s" % why)
    lines = blob.split(b"\n")
    tail_partial = bool(lines and lines[-1])
    out = []
    for i, raw in enumerate(lines):
        if not raw:
            continue
        last = i == len(lines) - 1
        try:
            e = json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            if last and tail_partial:
                log("[grader] the call log's last line is incomplete (a call that was never answered): left out")
                break
            fail("the graded episode's call log has a bad line (%d)" % (i + 1))
        if not isinstance(e, dict) or e.get("n") != len(out) + 1 or not isinstance(e.get("m"), str) or "p" not in e \
                or "p_repr" in e:
            fail("the graded episode's call log line %d is not a replayable call" % (i + 1))
        out.append((e["n"], e["m"], e["p"], e.get("code"), e.get("t"), e.get("s")))
    return out


def replay_episode(rec, site, expected):
    """Re-simulate the graded episode from the service's call log in a fresh cell (tests/cabincell, the site's package
    and noise seed, the run's label, the cell limits of online.json, no frame renderer: frames do not touch the cell's
    random streams or its physics), checking every call's reply and the physics digest after it; then end it as the
    service ends an episode (the program stopped: the firmware's stop and hold, the settle) and return the cell's
    record and the replay's statistics."""
    from cabincell import server as SV
    calls = read_calls(expected)
    t0 = time.time()
    privileged = rec.get("privileged") is True
    srv = SV.CabinCellServer(privileged=privileged, limits=dict(expected["cell_limits"]),
                             reference_dir=OV.REFERENCE_DIR if privileged else None, log=log)
    srv.reset(site["path"], seed=site["seed"], label=site["tag"])
    if int(srv.cell.seed) != site["noise_seed"]:
        fail("the re-simulation's noise seed is not the graded episode's")
    for n, method, params, code, t, digest in calls:
        res = srv.handle(method, params)
        if isinstance(res, dict):
            got_code = res.get("code") if "code" in res else res.get("error")
            got_t = res.get("t")
        else:
            got_code, got_t = None, None
        if got_code != code or (t is not None and (not isinstance(got_t, (int, float)) or abs(float(got_t) - t) > 1e-9)):
            fail("the re-simulation of the graded episode diverged at call %d (%s): %r at t %r, the log %r at t %r" % (
                n, method, got_code, got_t, code, t))
        if digest is not None and state_digest(srv.cell.data) != digest:
            fail("the re-simulation of the graded episode diverged at call %d (%s): the physics state differs" % (
                n, method))
        if srv.done:
            if n != len(calls):
                fail("the call log goes on after the cell ended the episode (call %d of %d)" % (n, len(calls)))
            break
    summary = rec.get("calls_log") if isinstance(rec.get("calls_log"), dict) else {}
    if int(summary.get("lines") or 0) > len(calls):            # (the record can be collected after the log)
        log("[grader] note: the record counts %s calls, the call log holds %d: the episode ends after the log's last"
            % (summary.get("lines"), len(calls)))
    if calls and int(summary.get("lines") or 0) == len(calls) and \
            summary.get("last_state_sha256") not in (None, calls[-1][5]):
        fail("the call log's last state does not match the record's")
    if not srv.done:
        srv._end("returned")                      # the program stopped after its last call
    cell_rec = srv.finalize(exit_code=0)
    stats = {"replayed": 1.0, "replay_calls": float(len(calls)), "replay_wall_s": time.time() - t0}
    try:
        srv.close()
    except Exception:  # noqa: BLE001
        pass
    return cell_rec, stats


def record_extra(rec):
    """The record's facts (finite numbers only): its counters and its end."""
    extra = {}
    for k in ("steps", "sim_time_s", "requests", "queries", "status_requests", "wall_s_since_start", "started_after_s",
              "record_writes", "requests_log_dropped"):
        f = _finite(rec.get(k))
        if f is not None:
            extra[k] = f
    extra["episode_ended"] = 1.0 if rec.get("ended") is True else 0.0
    extra["privileged_mode"] = 1.0 if rec.get("privileged") is True else 0.0
    extra["oracle_token_mismatch"] = 1.0 if rec.get("oracle_token_mismatch") is True else 0.0
    extra["calls_logged"] = float((rec.get("calls_log") or {}).get("lines") or 0)
    return extra


def online_component(rec, why, expected):
    """(reward_exact, raw caretaking score of the episode, metrics, reason) of the live item; 0 without a started
    episode. A cell that failed in the episode (its record's exit cell_error, or an internal error) is a task error:
    offline such a site is run again, and a live episode cannot be."""
    anchor, meta = read_anchor("anchor_online.json")
    if rec is None:
        return 0.0, 0.0, {"record_ok": 0.0, "started": 0.0}, why
    if rec.get("started") is not True:
        return 0.0, 0.0, {"record_ok": 1.0, "started": 0.0}, "the graded episode was never started"
    site = graded_site(expected)
    stats = {"replayed": 0.0}
    if rec.get("ended") is True and isinstance(rec.get("cell_record"), dict) and rec.get("end_reason") != "worker_restart":
        cell_rec = load_cell_record(rec, expected)
        how = "the service's end (%s)" % rec.get("end_reason")
    else:
        log("[grader] the graded episode did not end in the service (%s): re-simulated from its call log" % (
            rec.get("end_reason") if rec.get("ended") else "the session ended first"))
        cell_rec, stats = replay_episode(rec, site, expected)
        how = "its re-simulation from the call log (%s)" % (
            rec.get("end_reason") if rec.get("ended") else "ended with the session")
    if (cell_rec.get("times") or {}).get("exit") == "cell_error" or \
            int(_finite((cell_rec.get("metrics") or {}).get("internal_errors")) or 0) > 0:
        fail("the cell failed in the graded episode (exit %r, %s internal error(s))" % (
            (cell_rec.get("times") or {}).get("exit"), (cell_rec.get("metrics") or {}).get("internal_errors")))
    from cabincell import server as SV
    site_d, _members, sha = SV.open_site(site["path"])
    if sha != site["sha256"]:
        fail("the graded site's package changed during the verify")
    try:
        g = OV.GR.grade(cell_rec, site_d, site["ref_energy_wh"], OV.GR.load_config(), OV.GR.load_runtime())
    except Exception as e:  # noqa: BLE001 - a grader defect, never the agent's zero
        fail("the graded episode could not be graded: %s" % e)
    score = _finite(g.get("score"))
    if score is None or not 0.0 <= score <= 1.0:
        fail("the graded episode's score %r is not a finite number in [0, 1]" % (g.get("score"),))
    svc_score = _finite(rec.get("episode_score"))
    if stats["replayed"] == 0.0 and (svc_score is None or abs(svc_score - score) > 1e-9):
        fail("the graded episode's score %r differs from the service's %r" % (score, rec.get("episode_score")))
    extra = dict(record_extra(rec), record_ok=1.0, started=1.0, **stats)
    for k, v in (g.get("metrics") or {}).items():
        f = _finite(v)
        if f is not None and not str(k).startswith("cell_"):
            extra["grade." + str(k)] = f
    for k, v in (g.get("items") or {}).items():
        extra["item." + str(k)] = float(v)
    extra["R"] = float(g.get("R", 0.0))
    verdict = ratio_rewards(score, anchor, higher_is_better=meta.get("direction", "higher") == "higher",
                            metric_name=meta.get("metric", "caretaking_score"), locks_ok=True, extra=extra,
                            full_at=meta.get("full_at", 1.0))
    log("[grader] live item: site score %.4f on %s by %s (items %s, R %s)" % (
        score, expected["episode"], how, {k: round(float(v), 3) for k, v in (g.get("items") or {}).items()}, g.get("R")))
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
    # 5. the live item (the asset root is checked and named for the cell first, as main's grading does)
    if rec is not None and rec.get("started") is True and not off_metrics.get("assets_ok"):
        OV.assets_ok()
    on_reward, on_raw, on_metrics, on_why = online_component(rec, why, expected)
    # 6. the mean
    combined = (off_reward + on_reward) / 2.0
    anchor, meta = read_anchor()
    extra = {"offline_reward": off_reward, "online_reward": on_reward, "offline_caretaking_score": off_raw,
             "online_caretaking_score": on_raw, "record_ok": 1.0 if rec is not None else 0.0,
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
    text = ("dual_mode_score %.4f = (offline %.4f + online %.4f) / 2. Program item: caretaking_score %.4f of main's "
            "grading on the 6 hidden sites (%s). Live item: %s." % (
                combined, off_reward, on_reward, off_raw,
                "the program frozen at the graded start" if program_facts["program_snapshot"] and
                (rec or {}).get("snapshot_at") == "start" else
                "the program frozen when the preparation window closed" if program_facts["program_snapshot"] else
                "the frozen submission was over main's size cap, not graded" if program_facts["program_too_large"]
                else "the freeze at the window's close failed, not graded" if program_facts["program_freeze_failed"]
                else "the submission as the session left it",
                ("site score %.4f on the graded site %s, ended %s" % (
                    on_raw, rec.get("episode"),
                    rec.get("end_reason") if rec.get("ended") else "with the session (re-simulated to its last call)"))
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
