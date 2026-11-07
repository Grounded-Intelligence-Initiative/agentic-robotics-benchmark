"""Verify stage: re-run the submitted grasp detector on the hidden frames and score it.

Pattern: artifact_rollout. The agent leaves a Python module, /home/user/submission/solver.py,
whose predict(rgb, depth, intrinsics, depth_scale) returns grasps. This stage:

  1. scores an honest 0 when there is no solver.py (the untouched pass of `ale validate`);
  2. stops every process the agent's account left running;
  3. fetches the hidden frames (28 GraspNet-1Billion novel-object scenes x 16 views, meshes and
     poses for their 26 objects; pinned and hash-checked) into this root-only stage folder;
  4. copies only each frame's inputs (colour, depth, intrinsics) to a world-readable folder and
     runs the detector over them in 8 processes as the agent's account, offline (seccomp, see
     predict_child.py), with a per-frame deadline enforced from here;
  5. stops the agent's account again and scores the returned arrays as root with graspnetAPI's
     own force-closure evaluation (score_frames.py), which never imports submitted code;
  6. reports the AP the submission adds over the provided starting detector, RE-Bench style:
     ap_gain = max(0, mean AP over all 448 frames - the starting detector's own mean AP on them)
     (a frame without a valid array counts 0), as a ratio to the reference implementation's own
     measured gain. Copying the starting detector therefore scores 0 and the reference 1.

Everything here except the two helper scripts runs on the 3.12 /opt/venv interpreter with the
standard library and the vendored kit; graspnetAPI lives on /opt/grasp (Python 3.10).
"""
import hashlib
import json
import os
import pwd
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import time

from robotics_grader import (agent_user, deprivileged, fail, ratio_rewards, read_anchor,
                             require, stage_config, stage_dir, write_verdict, zero_verdict)

GRASP_PY = "/opt/grasp/bin/python"
HOME = os.environ.get("ALE_HOME", "/home/user")
SUBMISSION = os.path.join(HOME, "submission")
# Set by make_workdir(): a fresh mkdtemp folder (an agent cannot pre-plant it), root-owned and
# world-traversable, holding FRAMES (inputs only: rgb.png, depth.png, camK.npy) and PRED (owned
# by the agent's account, where the solver processes write).
WORK = FRAMES = PRED = None
CHILD_ENV = {
    # subprocess resolves `runuser` (in /usr/sbin) through this PATH, not the grader's own
    "PATH": "/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin",
    "HOME": HOME,
    "LANG": "C.UTF-8",
    "MPLBACKEND": "Agg",
    "GRASPNET_DATA": "/data/graspnet",
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
}


def log(msg):
    print("[grader] " + msg, file=sys.stderr, flush=True)


# --------------------------------------------------------------------------- processes
def _live_uid_pids(uid):
    """PIDs owned by `uid` that can still run. A zombie is already dead: SIGKILL cannot remove it, only its
    parent's wait() can, and in a fresh verifier sandbox nothing may reap it."""
    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open("/proc/%s/status" % entry) as handle:
                lines = handle.read().splitlines()
        except (OSError, ValueError):
            continue
        state = next((l.split()[1] for l in lines if l.startswith("State:")), "")
        owner = next((int(l.split()[1]) for l in lines if l.startswith("Uid:")), -1)
        if owner == uid and state not in ("Z", "X"):
            pids.append(int(entry))
    return pids


def stop_account(uid):
    """SIGKILL every live process of `uid`, repeating until none is left (they may fork)."""
    for _ in range(20):
        pids = _live_uid_pids(uid)
        if not pids:
            return
        for pid in pids:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        time.sleep(0.2)
    left = []
    for entry in os.listdir("/proc"):
        if entry.isdigit():
            try:
                with open("/proc/%s/status" % entry) as handle:
                    if any(l.startswith("Uid:") and int(l.split()[1]) == uid for l in handle):
                        with open("/proc/%s/cmdline" % entry, "rb") as c:
                            left.append("%s %s" % (entry, c.read().replace(b"\0", b" ")[:120].decode(errors="replace")))
            except (OSError, ValueError):
                continue
    log("still running as uid %d: %s" % (uid, "; ".join(left) or "none visible"))
    fail("processes of uid %d keep reappearing after SIGKILL" % uid)


# --------------------------------------------------------------------------- hidden frames
def fetch_hidden(cfg):
    sealed = os.path.join(stage_dir(), "sealed")
    marker = os.path.join(sealed, ".complete")
    if os.path.isfile(marker):
        return sealed
    shutil.rmtree(sealed, ignore_errors=True)
    os.makedirs(sealed, mode=0o700)
    os.chmod(sealed, 0o700)
    tarball = os.path.join(stage_dir(), "hidden.tar.gz")
    t0 = time.time()
    for url, dest, digest in ((cfg["hidden_bundle_url"], tarball, cfg["hidden_bundle_sha256"]),
                              (cfg["hidden_manifest_url"], os.path.join(sealed, "manifest.json"),
                               cfg["hidden_manifest_sha256"])):
        rc = subprocess.call(["curl", "-fsSL", "--retry", "10", "--retry-delay", "5",
                              "--retry-all-errors", "-o", dest, url])
        require(rc == 0, "could not download %s (curl exit %d)" % (url, rc))
        got = subprocess.check_output(["sha256sum", dest]).split()[0].decode()
        require(got == digest, "hash mismatch for %s: %s" % (url, got))
    rc = subprocess.call(["tar", "-xzf", tarball, "-C", sealed, "--strip-components=1",
                          "--no-same-owner"])
    require(rc == 0, "could not unpack the hidden frames (tar exit %d)" % rc)
    os.remove(tarball)
    open(marker, "w").close()
    log("hidden frames ready in %.0fs" % (time.time() - t0))
    return sealed


def hidden_frames(sealed):
    manifest = json.load(open(os.path.join(sealed, "manifest.json")))
    return [(r["scene"], v) for r in manifest["roles"]["hidden"] for v in r["views"]]


def privileged(cfg):
    """True only for a submission that carries the oracle token (solution/oracle_token).

    The token's SHA-256 is in grader_config.json; the agent never sees solution/ or the token, so only the
    reference detector run by `ale validate` is handed the ground truth of the frames. Its only job is to
    show that the task can be done."""
    path = os.path.join(SUBMISSION, "oracle_token")
    try:
        st = os.lstat(path)
        if not stat.S_ISREG(st.st_mode) or st.st_size > 256:
            return False
        with open(path, "rb") as handle:
            token = handle.read().strip()
    except OSError:
        return False
    return hashlib.sha256(token).hexdigest() == cfg.get("oracle_token_sha256")


def export_ground_truth(sealed, frames):
    """Copy what graspnetAPI needs to label grasps (poses and meshes of the scenes' objects) into WORK/gt."""
    gt = os.path.join(WORK, "gt")
    objects = set()
    for sid in sorted({sid for sid, _ in frames}):
        src = os.path.join(sealed, "scenes", "scene_%04d" % sid)
        dst = os.path.join(gt, "scenes", "scene_%04d" % sid)
        os.makedirs(os.path.join(dst, "realsense", "meta"))
        os.makedirs(os.path.join(dst, "realsense", "annotations"))
        for name in ("camK.npy", "camera_poses.npy", "cam0_wrt_table.npy"):
            shutil.copyfile(os.path.join(src, "realsense", name), os.path.join(dst, "realsense", name))
        shutil.copyfile(os.path.join(src, "object_id_list.txt"), os.path.join(dst, "object_id_list.txt"))
        for s2, view in frames:
            if s2 == sid:
                for sub, ext in (("meta", "mat"), ("annotations", "xml")):
                    f = "%04d.%s" % (view, ext)
                    shutil.copyfile(os.path.join(src, "realsense", sub, f),
                                    os.path.join(dst, "realsense", sub, f))
        with open(os.path.join(src, "object_id_list.txt")) as handle:
            objects.update(int(v) for v in handle.read().split())
    for obj in sorted(objects):
        shutil.copytree(os.path.join(sealed, "models", "%03d" % obj), os.path.join(gt, "models", "%03d" % obj))
    CHILD_ENV["ORACLE_GT_ROOT"] = gt
    log("oracle token accepted: ground truth of %d scenes and %d objects exported" %
        (len({sid for sid, _ in frames}), len(objects)))


def make_workdir():
    global WORK, FRAMES, PRED
    WORK = tempfile.mkdtemp(prefix="grasp_grading_", dir="/tmp")
    FRAMES = os.path.join(WORK, "frames")
    PRED = os.path.join(WORK, "pred")


def export_inputs(sealed, frames, uid, gid, with_ground_truth=False):
    os.makedirs(FRAMES)
    for sid, view in frames:
        src = os.path.join(sealed, "scenes", "scene_%04d" % sid, "realsense")
        dst = os.path.join(FRAMES, "%04d_%04d" % (sid, view))
        os.makedirs(dst)
        shutil.copyfile(os.path.join(src, "rgb", "%04d.png" % view), os.path.join(dst, "rgb.png"))
        shutil.copyfile(os.path.join(src, "depth", "%04d.png" % view), os.path.join(dst, "depth.png"))
        shutil.copyfile(os.path.join(src, "camK.npy"), os.path.join(dst, "camK.npy"))
    shutil.copyfile(os.path.join(stage_dir(), "predict_child.py"),
                    os.path.join(WORK, "predict_child.py"))
    if with_ground_truth:
        export_ground_truth(sealed, frames)
    for root, dirs, files in os.walk(WORK):
        os.chmod(root, 0o755)
        for name in files:
            os.chmod(os.path.join(root, name), 0o644)
    os.makedirs(PRED)
    os.chown(PRED, uid, gid)
    os.chmod(PRED, 0o755)


# --------------------------------------------------------------------------- solver runs
def _read_small(path, limit=2000):
    """Read a status file the agent's account wrote, refusing anything but a regular file."""
    try:
        st = os.lstat(path)
        if not stat.S_ISREG(st.st_mode):
            return "not a regular file"
        with open(path, errors="replace") as handle:
            return handle.read(limit)
    except OSError as exc:
        return "unreadable: %s" % exc


class Slot(object):
    """One solver process working through its share of the frames, restarted after a kill."""

    def __init__(self, index, names, cfg, logs):
        self.index, self.remaining, self.cfg, self.logs = index, list(names), cfg, logs
        self.attempt, self.proc, self.done = 0, None, not names
        self.seen_start = {}
        self.guard_failed = False

    def tag(self):
        return "%d_%d" % (self.index, self.attempt)

    def start(self):
        self.attempt += 1
        listing = os.path.join(WORK, "list_%s.txt" % self.tag())
        with open(listing, "w") as handle:
            handle.write("\n".join(self.remaining) + "\n")
        os.chmod(listing, 0o644)
        out = open(os.path.join(self.logs, "solver_%s.log" % self.tag()), "w")
        self.proc = deprivileged(
            [GRASP_PY, "-I", os.path.join(WORK, "predict_child.py"), FRAMES, PRED, listing,
             self.tag(), str(self.cfg["frame_timeout_s"])],
            cwd=SUBMISSION, env=CHILD_ENV, stdout=out, stderr=subprocess.STDOUT,
            start_new_session=True)
        self.started_at = time.time()

    def kill(self):
        if self.proc is not None and self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, signal.SIGKILL)
            except OSError:
                pass
            self.proc.wait()

    def poll(self, status, now):
        """Advance this slot; returns False once it has nothing left to do."""
        if self.done:
            return False
        # frames finished since the last poll
        while self.remaining:
            name = self.remaining[0]
            if os.path.lexists(os.path.join(PRED, name + ".npy")):
                status[name] = "returned"
            elif os.path.lexists(os.path.join(PRED, name + ".err")):
                status[name] = _read_small(os.path.join(PRED, name + ".err"), 300)
            else:
                break
            self.remaining.pop(0)
        if not self.remaining:
            self.kill()
            self.done = True
            return False
        grace = self.cfg["kill_grace_s"]
        ready = os.path.lexists(os.path.join(PRED, "_ready_" + self.tag()))
        alive = self.proc.poll() is None
        if not ready:
            err = os.path.join(PRED, "_import_error_" + self.tag())
            guarded = os.path.lexists(os.path.join(PRED, "_guard_ok_" + self.tag()))
            if not alive and not guarded:
                # The child exits before importing anything when the seccomp filter cannot be
                # installed, so the submission never ran: this is the grader's failure.
                self.guard_failed = True
                self._give_up(status, "network guard could not be installed")
            elif os.path.lexists(err) or not alive:
                why = _read_small(err) if os.path.lexists(err) else \
                    "solver process exited with %s before importing" % self.proc.poll()
                self._give_up(status, "import failed: " + why[:200])
            elif now - self.started_at > self.cfg["import_timeout_s"] + grace:
                self.kill()
                self._give_up(status, "import did not finish in %ds" % self.cfg["import_timeout_s"])
            return not self.done
        name = self.remaining[0]
        if os.path.lexists(os.path.join(PRED, name + ".started")):
            self.seen_start.setdefault(name, now)
        overdue = name in self.seen_start and \
            now - self.seen_start[name] > self.cfg["frame_timeout_s"] + grace
        if overdue or not alive:
            self.kill()
            status[name] = "timeout (process killed)" if overdue else \
                "solver process died (exit %s)" % self.proc.returncode
            self.remaining.pop(0)
            if self.remaining:
                self.start()
            else:
                self.done = True
        return not self.done

    def _give_up(self, status, why):
        for name in self.remaining:
            status[name] = why
        self.remaining = []
        self.done = True


def run_solvers(names, cfg, logs):
    n = int(cfg["solver_processes"])
    per = -(-len(names) // n)             # contiguous blocks: a solver that caches per scene reloads it rarely
    slots = [Slot(i, names[i * per:(i + 1) * per], cfg, logs) for i in range(n)]
    for slot in slots:
        if not slot.done:
            slot.start()
    status = {}
    per_slot = -(-len(names) // n)
    budget = per_slot * cfg["frame_timeout_s"] + 3 * cfg["import_timeout_s"] + 120
    t0 = time.time()
    while True:
        now = time.time()
        active = [s.poll(status, now) for s in slots]
        if not any(active):
            break
        if now - t0 > budget:
            for slot in slots:
                slot.kill()
                if not slot.done:
                    slot._give_up(status, "prediction time budget (%ds) exhausted" % budget)
            break
        time.sleep(0.5)
    return status, time.time() - t0, any(s.guard_failed for s in slots)


# --------------------------------------------------------------------------- main
def main():
    cfg = stage_config()
    anchor, meta = read_anchor()
    cap = cfg.get("scoring_cap", 1.5)
    if not os.path.isfile(os.path.join(SUBMISSION, "solver.py")):
        write_verdict(zero_verdict(reason="no /home/user/submission/solver.py",
                                   anchored=anchor is not None))
        return

    account = pwd.getpwnam(agent_user())
    stop_account(account.pw_uid)
    sealed = fetch_hidden(cfg)
    frames = hidden_frames(sealed)
    require(len(frames) == cfg["expected_frames"],
            "hidden manifest lists %d frames, expected %d" % (len(frames), cfg["expected_frames"]))
    make_workdir()
    export_inputs(sealed, frames, account.pw_uid, account.pw_gid, with_ground_truth=privileged(cfg))
    logs = os.path.join(stage_dir(), "logs")
    os.makedirs(logs, exist_ok=True)

    names = ["%04d_%04d" % f for f in frames]
    status, predict_wall, guard_failed = run_solvers(names, cfg, logs)
    stop_account(account.pw_uid)
    require(not guard_failed, "the network guard could not be installed; see %s" % logs)
    log("prediction finished in %.0fs" % predict_wall)

    out_json = os.path.join(stage_dir(), "scores.json")
    t0 = time.time()
    rc = subprocess.call([GRASP_PY, "-I", os.path.join(stage_dir(), "score_frames.py"),
                          sealed, PRED, out_json, str(cfg["score_workers"])],
                         cwd=stage_dir())
    require(rc == 0, "score_frames.py exited %d" % rc)
    scores = json.load(open(out_json))
    score_wall = time.time() - t0
    require(scores["frames"] == len(frames), "scored %d of %d frames" % (scores["frames"], len(frames)))

    rows = scores["per_frame"]
    for row in rows:
        if row["status"] != "ok" or status.get(row["frame"]) != "returned":
            row["detail"] = status.get(row["frame"], "not run")
    ok = sum(1 for r in rows if r["status"] == "ok")
    timeouts = sum(1 for r in rows if "timeout" in str(r.get("detail", "")))
    print(json.dumps({"mean_ap": scores["mean_ap"], "frames": len(rows), "frames_valid": ok,
                      "per_frame": rows}), flush=True)
    log("mean AP %.3f over %d frames (%d with a valid array, %d timeouts); starting detector %.3f"
        % (scores["mean_ap"], len(rows), ok, timeouts, float(meta["baseline_mean_ap"])))

    baseline = float(meta["baseline_mean_ap"])
    gain = max(0.0, scores["mean_ap"] - baseline)
    write_verdict(ratio_rewards(
        gain, anchor, cap=cap, higher_is_better=True, metric_name="ap_gain",
        full_at=meta.get("full_at", 1.0),
        extra={"mean_ap": scores["mean_ap"], "frames": len(rows), "frames_valid": ok,
               "frames_timeout": timeouts, "predict_wall_s": round(predict_wall, 1),
               "score_wall_s": round(score_wall, 1)}))


if __name__ == "__main__":
    main()
