"""Grader of humanoid_robot_description_from_calibration_records (the robot-description stage of the humanoid
workflow). Root, verify stage, in a SEPARATE verifier sandbox (task.toml): a fresh sandbox from the task image into
which the engine restored only /home/user/submission; the agent's sandbox is gone, setup.sh has not run here.

  0. FIRST THING, stdlib only (`_early_lockdown`; test.sh has already chmod 0700 the stage and its parent): record when
     the engine began uploading this folder (earliest ctime of its files), freeze and kill every process of the agent
     account (a survivor voids the run: task error), keep every process of the account SIGSTOPped until the verdict is
     written (`verify_freeze.AgentFreezer`), lock the stage (0700 / 0600, restored at the end), take a root-owned 0700
     snapshot of the submission that every later read uses, and list every entry of the submission written at or
     after the stage upload began (the gate's late-write rule refuses the submission when there is one). A submission
     root that is a symbolic link, or that resolves outside the agent's home, is not read; a tree above 200 MB or
     20 000 entries is not copied; both grade as an empty folder.
  1. The frozen inputs: every file named in hidden/FREEZE.json must carry its recorded sha256 (else no score: a task
     error). The given model's file and its 26 link meshes come from the image's copy of the givens
     (/opt/task-assets/payload), each checked against given_hashes.json, copied into a root-owned folder.
  2. rd04 (tests/rd04): the gate (rd04/gate.py, byte for byte the agent's given/contract/gate.py), then M (the files
     at the hidden bus samples) and D (the postures planned on the files, executed on the true unit); score =
     gate x M x D (grader_config.json: weights and bands).
  3. The benchmark kit's envelope (robotics_grader.ratio_rewards against anchor.json: value 1.0 = the reference,
     full_at 1.0): one gated `reward` = the score; the published metrics are rd04's whitelist (3 decimals, band words)
     and the kit's own keys; the lockdown's counts and the gate's reasons go into the verdict's diagnostic text.
Nothing submitted is executed (the model file and the calibration file are data). The grader reads only tests/ and
the restored submission, and from the image the given model.
"""
import hashlib
import json
import math
import os
import pwd
import shutil
import stat as statmod
import sys
import tempfile
import time

sys.dont_write_bytecode = True
STAGE_DIR = os.environ.get("ALE_STAGE_DIR", os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, STAGE_DIR)
HOME = os.environ.get("ALE_HOME", "/home/user")
AGENT_USER = os.path.basename(HOME.rstrip("/")) or "user"
SUBMISSION = os.path.join(HOME, "submission")
SNAP_ROOT = os.environ.get("ARB_SNAPSHOT_ROOT", "/opt/arb-verify")
SNAPSHOT = os.path.join(SNAP_ROOT, "submission")
GIVEN_ROOT = os.environ.get("ARB_GIVEN_ROOT", "/opt/task-assets/payload")
MODEL_DIR = "given/repo/sim-model/robot_25dof"
MODEL_FILE = MODEL_DIR + "/xml/robot_25dof.xml"
LATE_GATE_MAX_GAP_S = 600.0
SUBMISSION_BYTES_CAP = 200 * 1024 * 1024
SUBMISSION_ENTRIES_CAP = 20000
METRIC = "description_score"
KIT_KEYS = ("ratio", "anchor_recorded", METRIC, "reward_exact")

import lockdown  # noqa: E402  (stdlib only; vendored)
import verify_freeze  # noqa: E402  (stdlib only; vendored)

FREEZER = verify_freeze.AgentFreezer.for_user(AGENT_USER)


class TaskError(RuntimeError):
    """a fault of the task's own files or image (never of a submission): no score is written"""


def log(msg):
    print("[grader] " + str(msg), file=sys.stderr, flush=True)


def sha256_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def lock_stage(root):
    """chmod 0700 every directory and 0600 every file under `root` (root only); the modes to restore, or None."""
    if os.getuid() != 0:
        return None
    saved = []
    for dirpath, _dirnames, filenames in os.walk(root):
        for name in [""] + filenames:
            p = os.path.join(dirpath, name) if name else dirpath
            try:
                mode = statmod.S_IMODE(os.lstat(p).st_mode)
            except OSError:
                continue
            saved.append((p, mode))
            try:
                os.chmod(p, 0o700 if not name else 0o600)
            except OSError:
                pass
    return saved


def unlock_stage(saved):
    for p, mode in saved or []:
        try:
            os.chmod(p, mode)
        except OSError:
            pass


def _submission_root_ok():
    """(ok, reason): the submission root must be a real directory inside the agent's home, never a symbolic link."""
    try:
        st = os.lstat(SUBMISSION)
    except FileNotFoundError:
        return True, None
    except OSError as exc:
        return False, "the submission root %s cannot be inspected (%s)" % (SUBMISSION, exc)
    if statmod.S_ISLNK(st.st_mode):
        return False, "the submission root %s is a symbolic link: not read" % SUBMISSION
    home = os.path.realpath(HOME)
    real = os.path.realpath(SUBMISSION)
    if os.path.commonpath([home, real]) != home:
        return False, "the submission root %s resolves outside %s: not read" % (SUBMISSION, HOME)
    return True, None


def _submission_measure(root, max_entries=SUBMISSION_ENTRIES_CAP):
    """(entries, apparent bytes, allocated bytes) of the tree below `root` by lstat, never following a link."""
    n, apparent, allocated = 0, 0, 0
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        for name in dirnames + filenames:
            n += 1
            if n > max_entries:
                return n, apparent, allocated
            try:
                st = os.lstat(os.path.join(dirpath, name))
            except OSError:
                continue
            if statmod.S_ISREG(st.st_mode):
                apparent += int(st.st_size)
                allocated += int(getattr(st, "st_blocks", 0)) * 512
    return n, apparent, allocated


def _early_lockdown():
    out = {"agent_processes_killed": 0.0, "agent_processes_left": 0.0, "stage_locked": 0.0, "stage_parent_locked": 0.0,
           "submission_late_writes": 0.0, "late_write_gate_active": 0.0, "submission_snapshot": 0.0,
           "submission_root_rejected": 0.0, "submission_oversize": 0.0, "saved": None, "parent": None, "reasons": [],
           "t_ref": None, "late": []}
    root_ok, why = _submission_root_ok()
    if not root_ok:
        out["submission_root_rejected"] = 1.0
        out["reasons"].append(why)
    if os.getuid() != 0:
        return out
    t_ref = lockdown.stage_upload_time(STAGE_DIR)
    out["t_ref"] = t_ref
    try:
        uid = pwd.getpwnam(AGENT_USER).pw_uid
    except KeyError:
        uid = None
    if uid is not None and uid != 0:
        killed, left = lockdown.stop_and_kill_uid(uid, grace_s=8.0)
        out["agent_processes_killed"], out["agent_processes_left"] = float(killed), float(left)
        try:
            FREEZER.start()
        except Exception:  # noqa: BLE001 - the freezer never takes the verdict down
            pass
    out["saved"] = lock_stage(STAGE_DIR)
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
    gate_active = t_ref is not None
    cfg_json = os.path.join(STAGE_DIR, "grader_config.json")
    if gate_active and os.path.isfile(cfg_json):
        try:
            gate_active = (os.lstat(cfg_json).st_ctime - t_ref) <= LATE_GATE_MAX_GAP_S
        except OSError:
            pass
    out["late_write_gate_active"] = 1.0 if gate_active else 0.0
    if root_ok and os.path.isdir(SUBMISSION):
        n, apparent, allocated = _submission_measure(SUBMISSION)
        if n > SUBMISSION_ENTRIES_CAP or max(apparent, allocated) > SUBMISSION_BYTES_CAP:
            out["submission_oversize"] = 1.0
            out["reasons"].append("the submission holds more than the caps (%d entries, %d bytes): not copied, graded "
                                  "as an empty folder" % (SUBMISSION_ENTRIES_CAP, SUBMISSION_BYTES_CAP))
            return out
        if gate_active:
            late = lockdown.late_entries(SUBMISSION, t_ref)
            out["late"] = late
            out["submission_late_writes"] = float(len(late))
            if late:
                out["reasons"].append("%d entr%s of the submission written after the agent phase ended (%s)" % (
                    len(late), "y" if len(late) == 1 else "ies", ", ".join(e[0] for e in late[:5])))
        os.makedirs(SNAP_ROOT, exist_ok=True)
        os.chmod(SNAP_ROOT, 0o700)
        lockdown.snapshot_dir(SUBMISSION, SNAPSHOT)
        out["submission_snapshot"] = 1.0
    return out


def _restore(early):
    try:
        if early.get("saved") is not None:
            unlock_stage(early["saved"])
        if early.get("parent"):
            os.chmod(early["parent"][0], early["parent"][1])
        mode = os.environ.get("ARB_STAGE_MODE")
        if mode:
            os.chmod(STAGE_DIR, int(mode, 8))
    except OSError:
        pass


def check_freeze():
    """every file hidden/FREEZE.json names carries its recorded sha256 (paths relative to this folder)"""
    p = os.path.join(STAGE_DIR, "hidden", "FREEZE.json")
    try:
        with open(p, encoding="utf-8") as f:
            fr = json.load(f)
        files = fr["files_sha256"]
    except (OSError, ValueError, KeyError) as exc:
        raise TaskError("the freeze record hidden/FREEZE.json is unreadable: %s" % exc)
    bad = []
    for rel, want in sorted(files.items()):
        q = os.path.join(STAGE_DIR, rel)
        try:
            got = sha256_file(q)
        except OSError:
            got = None
        if got != want:
            bad.append(rel)
    if bad:
        raise TaskError("files differ from the freeze record (no score): %s" % ", ".join(bad[:8]))
    return fr


def given_model_copy():
    """the given model's file and 26 link meshes from the image's copy of the givens, each checked against
    given_hashes.json, copied into a root-owned folder: rd04.given.GivenModel"""
    with open(os.path.join(STAGE_DIR, "given_hashes.json"), encoding="utf-8") as f:
        pins = json.load(f)["files_sha256"]
    want = {rel: h for rel, h in pins.items()
            if rel == MODEL_FILE or (rel.startswith(MODEL_DIR + "/meshes/") and rel.count("/") == MODEL_DIR.count("/") + 2)}
    if MODEL_FILE not in want or len(want) != 27:
        raise TaskError("given_hashes.json does not pin the given model's file and its 26 meshes (%d)" % len(want))
    dst = os.path.join(SNAP_ROOT, "given_model")
    if os.path.isdir(dst):
        shutil.rmtree(dst, ignore_errors=True)
    os.makedirs(os.path.join(dst, "meshes"), exist_ok=True)
    for rel, h in sorted(want.items()):
        src = os.path.join(GIVEN_ROOT, rel)
        if os.path.islink(src) or not os.path.isfile(src) or sha256_file(src) != h:
            raise TaskError("the image's given file %s is missing or differs from given_hashes.json" % rel)
        out = os.path.join(dst, os.path.basename(rel)) if rel == MODEL_FILE else os.path.join(dst, "meshes",
                                                                                               os.path.basename(rel))
        shutil.copyfile(src, out)
    if os.getuid() == 0:
        for dp, dns, fns in os.walk(dst):
            os.chmod(dp, 0o700)
            for fn in fns:
                os.chmod(os.path.join(dp, fn), 0o600)
    from rd04.given import GivenModel
    return GivenModel.from_dir(dst)


def publish(metrics_rd04, postures, kit_env, reward):
    """the published metrics: rd04's whitelist (another key is a task error) and the kit's own keys"""
    from rd04.metrics import whitelist
    allowed = whitelist(postures)
    extra = sorted(set(metrics_rd04) - allowed)
    if extra:
        raise TaskError("rd04 returned metric keys outside its whitelist: %s" % extra)
    out = {}
    for k, v in metrics_rd04.items():
        if isinstance(v, float) and not math.isfinite(v):
            v = None
        out[k] = v
    for k, v in kit_env["metrics"].items():
        if k in out:
            raise TaskError("a kit metric key collides with rd04's: %s" % k)
        out[k] = float(v)
    out["reward_exact"] = float(reward)
    assert set(out) <= allowed | set(KIT_KEYS)
    return dict(sorted(out.items()))


def write_result(reward, metrics, diagnostic):
    """In the engine (ALE_VERIFY_PYTHON set) through ale_verify.write (reward.json and verification.json); a
    host-side run writes the same result to ROBOTICS_GRADER_VERDICT_PATH, else verdict.local.json in this folder."""
    result = {"reward": float(reward), "metrics": metrics, "diagnostic": str(diagnostic)[:4000]}
    if os.environ.get("ALE_VERIFY_PYTHON"):
        from ale_verify import write
        write(result)
        return
    path = os.environ.get("ROBOTICS_GRADER_VERDICT_PATH") or os.path.join(STAGE_DIR, "verdict.local.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1, sort_keys=True, allow_nan=False)
        f.write("\n")


def main():
    t0 = time.time()
    try:
        early = _early_lockdown()
    except Exception:  # noqa: BLE001 - never a task error without a verdict
        import traceback
        log("the verify-start lockdown raised:\n" + traceback.format_exc())
        write_result(0.0, {"ratio": 0.0, "anchor_recorded": 1.0, "reward_exact": 0.0},
                     "scoring 0: the verify-start lockdown failed: " + traceback.format_exc()[-800:])
        FREEZER.stop()
        return
    stats = {k: v for k, v in early.items() if isinstance(v, float)}
    stats["lockdown_s"] = round(time.time() - t0, 3)
    try:
        if early["agent_processes_left"] > 0:
            raise TaskError("%d agent process(es) survived the verify-start kill: run void" % early["agent_processes_left"])
        fr = check_freeze()
        given = given_model_copy()
        import rd04
        from robotics_grader import ratio_rewards, read_anchor
        with open(os.path.join(STAGE_DIR, "grader_config.json"), encoding="utf-8") as fh:
            cfg = json.load(fh)
        t1 = time.time()
        H = rd04.load_hidden(os.path.join(STAGE_DIR, "hidden"), os.path.join(STAGE_DIR, "gt"))
        stats["hidden_s"] = round(time.time() - t1, 3)
        sub_dir = SNAPSHOT if early["submission_snapshot"] else SUBMISSION
        if early["submission_root_rejected"] > 0 or early["submission_oversize"] > 0:
            sub_dir = tempfile.mkdtemp(prefix="arb_rejected_submission_")
        late = early.get("late") or None
        t2 = time.time()
        grader_exception = 0.0
        try:
            F = rd04.load(sub_dir, given=given, late_entries=late)
            out = rd04.score(F, H, config=cfg)
        except Exception as exc:  # noqa: BLE001 - a submission that crashes the grader earns 0, not a task error
            if type(exc).__name__ in ("GateMissing", "GateSetupError"):
                raise
            import traceback
            log("exception while grading the submission:\n" + traceback.format_exc())
            grader_exception = 1.0
            out = rd04.score(rd04.GateFail([("grader_exception", "the grader raised on this submission")]), H,
                             config=cfg)
        stats["grade_s"] = round(time.time() - t2, 3)
        stats["grader_exception"] = grader_exception
        score = float(out["score"])
        if not (math.isfinite(score) and 0.0 <= score <= 1.0):
            raise TaskError("rd04 returned a score outside [0, 1]: %r" % score)
        anchor, meta = read_anchor()
        env = ratio_rewards(score, anchor, higher_is_better=meta.get("direction", "higher") == "higher",
                            metric_name=meta.get("metric", METRIC), full_at=meta.get("full_at", 1.0))
        reward = float(env["rewards"]["reward"])
        metrics = publish(out["metrics"], H.postures, env, reward)
        stats["verify_s"] = round(time.time() - t0, 3)
        stats["agent_processes_frozen"] = float(len(FREEZER.frozen))
        gate = out.get("gate", {})
        diag = ["score %.6f (M %.6f, D %.6f)" % (score, float(out.get("M", 0.0)), float(out.get("D", 0.0))),
                "gate %s%s" % ("pass" if gate.get("pass") else "fail",
                               (" " + ",".join(gate.get("codes", []))) if gate.get("codes") else ""),
                "freeze %s" % fr.get("status", "?"),
                "lockdown " + json.dumps({k: stats[k] for k in sorted(stats)}, sort_keys=True)]
        if early["reasons"]:
            diag.append("submission: " + "; ".join(early["reasons"]))
        write_result(reward, metrics, " | ".join(diag))
        log("score %.4f M %.4f D %.4f gate %s (%.1f s)" % (score, float(out.get("M", 0.0)), float(out.get("D", 0.0)),
                                                         gate, stats["verify_s"]))
    finally:
        _restore(early)
        FREEZER.stop()


if __name__ == "__main__":
    main()
