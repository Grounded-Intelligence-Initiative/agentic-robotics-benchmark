"""Hidden grader for multisensor_rig_calibration_from_logs (artifact rollout on hidden rigs).

Runs as root in a SEPARATE verifier sandbox: no agent process exists there, the setup stage has not
run there, and the engine restores only the declared artifact /home/user/submission.

1. Draws `rigs` fresh rig seeds (four 31-bit words each) from the kit's seed stream (OS entropy,
   never on disk while the submission runs; the verdict's diagnostic names them afterwards, so a
   maintainer can regenerate a rig with rigsim.make_rig) and generates the rigs with rigsim.py, in parallel worker processes: each rig's
   agent-visible folder (same schema as the practice rigs) goes into a fresh world-readable work
   folder; its true parameters, injected faults and held-out sequence stay in this process's memory.
2. Clears the agent's account (robotics_grader.account.clear_agent_account) and then runs the
   submission's `bash run.sh --input <rig> --output <out>` once per rig, all rigs at once, each at the
   agent's UID through nonet_exec.py (a seccomp filter: AF_UNIX sockets only, so no network), with
   cwd /home/user/submission, a wall-clock limit of `run_timeout_s` and an address-space limit of
   `run_memory_mb`. A run that is still going at its limit is killed with its process group and
   scores on whatever files it has left (normally none).
3. Reads each rig's calibration.yaml and fault_report.json (regular files only, at most 1 MiB, as
   data: yaml.safe_load / json) and measures them with rigmetrics.py (byte-identical to the agent's
   practice copy). Each rig earns a credit in [0, 1]:

       0.35 * alignment  (projection_px)
     + 0.30 * downstream (mean of rpe_rot_deg and rpe_trans_m credits)
     + 0.15 * time_sync  (time_rms_s)
     + 0.10 * imu        (mean of imu_rot_deg, gyro_bias_err_rps, accel_bias_err_mps2 credits)
     + 0.10 * faults     (fault_f1)

   each error credit being clamp(log(zero/e) / log(zero/full), 0, 1) with the `full` and `zero`
   points of grader_config.json `credit`. A calibration.yaml that is missing or does not parse
   earns 0 on the four calibration terms of that rig; a missing or malformed fault_report.json
   earns 0 on the fault term.
4. The metric `mean_rig_credit` (higher is better) is the mean over the rigs; the kit turns it into
   the reward against the anchor.

Nothing the submission prints is read. The grader's own failures (a rig that cannot be generated,
an account that cannot be cleared) are task errors (`fail`), never a zero.
"""

import json
import math
import multiprocessing
import os
import pwd
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import time

# One BLAS thread everywhere: the generator's arithmetic must not depend on the host's core count.
for _k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_k] = "1"

STAGE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, STAGE)

from robotics_grader import (agent_user, fail, ratio_rewards, read_anchor, seed_rng,  # noqa: E402
                             stage_config, write_verdict, zero_verdict)
from robotics_grader.account import clear_agent_account  # noqa: E402
from robotics_grader.stage import secure_exec_path  # noqa: E402

HOME = os.environ.get("ALE_HOME", "/home/user")
SUBMISSION = os.path.join(HOME, "submission")
NONET = os.path.join(STAGE, "nonet_exec.py")
MAX_READ = 1 << 20


def log(msg):
    print("[grader] " + msg, file=sys.stderr, flush=True)


# ----------------------------------------------------------------------------- agent processes

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
                        zombie = line.split()[1].startswith(("Z", "X"))
                    elif line.startswith("Uid:"):
                        if int(line.split()[1]) == uid and not zombie:
                            pids.append(int(name))
                        break
        except (OSError, ValueError, IndexError):
            continue
    return pids


def kill_agent_processes(uid, rounds=20, pause=0.25):
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


# ----------------------------------------------------------------------------- generation

def _generate(args):
    """Worker: write one hidden rig's agent-visible folder; return its truth and held-out data."""
    seed_words, rig_id, out_dir = args
    import numpy as np  # noqa: F401
    import rigsim
    import rigmetrics
    rig = rigsim.make_rig(seed_words)
    records = rigsim.write_rig(rig, rig_id, out_dir)
    ho = rigsim.heldout(rig)
    tr = ho["traj"]
    truth = {
        "K": rig.K, "T_body_lidar": rig.T_body_lidar,
        "T_lidar_camera": rig.T_lidar_camera, "T_lidar_imu": rig.T_lidar_imu,
        "gyro_bias": rig.gyro_bias, "accel_bias": rig.accel_bias, "faults": records,
        "clock": {"camera_offset_s": rig.cam_offset, "camera_drift_ppm": rig.cam_drift * 1e6,
                  "lidar_offset_s": rig.lidar_offset, "lidar_drift_ppm": rig.lidar_drift * 1e6},
    }
    held = {
        "seq_start": ho["seq_start"],
        "track": rigmetrics.BodyTrack(tr.tau, tr.pos, tr.ypr),
        "imu_tau": ho["imu_tau"], "gyro": ho["gyro"], "accel": ho["accel"],
        "cam_tau": ho["cam_tau"], "cam_stamp": ho["cam_stamp"],
        "lidar_tau": ho["lidar_tau"], "lidar_stamp": ho["lidar_stamp"],
        "pairs": ho["pairs"],
    }
    return truth, held


def _open_tree(root):
    for dirpath, dirnames, filenames in os.walk(root):
        os.chmod(dirpath, 0o755)
        for f in filenames:
            os.chmod(os.path.join(dirpath, f), 0o644)


# ----------------------------------------------------------------------------- running

def _read_small(path):
    """Contents of a regular file of at most MAX_READ bytes, never following a symlink."""
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except OSError as exc:
        return None, "cannot open ({})".format(type(exc).__name__)
    try:
        st = os.fstat(fd)
        if not stat.S_ISREG(st.st_mode):
            return None, "not a regular file"
        if st.st_size > MAX_READ:
            return None, "larger than 1 MiB"
        data = b""
        while len(data) <= MAX_READ:
            chunk = os.read(fd, 65536)
            if not chunk:
                break
            data += chunk
        if len(data) > MAX_READ:
            return None, "larger than 1 MiB"
        return data.decode("utf-8", "replace"), None
    finally:
        os.close(fd)


def run_submission(rig_dirs, out_dirs, uid, gid, config):
    timeout = float(config.get("run_timeout_s", 900))
    mem_mb = int(config.get("run_memory_mb", 6144))
    procs = []
    logs = []
    for i, (rd, od) in enumerate(zip(rig_dirs, out_dirs)):
        lp = os.path.join(os.path.dirname(od), "run_%d.log" % i)
        fh = open(lp, "wb")
        cmd = [sys.executable, "-I", NONET, "--uid", str(uid), "--gid", str(gid), "--cwd", SUBMISSION,
               "--rlimit-as-mb", str(mem_mb), "--", "bash", "run.sh", "--input", rd, "--output", od]
        p = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=fh, stderr=subprocess.STDOUT,
                             start_new_session=True, env={"PATH": "/usr/bin:/bin"})
        procs.append(p)
        logs.append((fh, lp))
    t0 = time.monotonic()
    status = [None] * len(procs)
    while any(s is None for s in status):
        for i, p in enumerate(procs):
            if status[i] is None:
                rc = p.poll()
                if rc is not None:
                    status[i] = ("exit", rc, time.monotonic() - t0)
                elif time.monotonic() - t0 > timeout:
                    try:
                        os.killpg(p.pid, signal.SIGKILL)
                    except OSError:
                        pass
                    p.wait()
                    status[i] = ("timeout", None, time.monotonic() - t0)
        time.sleep(0.5)
    kill_agent_processes(uid)
    tails = []
    for fh, lp in logs:
        fh.close()
        try:
            with open(lp, "rb") as f:
                f.seek(max(0, os.path.getsize(lp) - 600))
                tails.append(f.read().decode("utf-8", "replace"))
        except OSError:
            tails.append("")
    return status, tails


# ----------------------------------------------------------------------------- scoring

def credit(err, full, zero):
    if err is None or not math.isfinite(err):
        return 0.0
    if err <= full:
        return 1.0
    if err >= zero:
        return 0.0
    return max(0.0, min(1.0, math.log(zero / err) / math.log(zero / full)))


def rig_credit(m, cfg):
    c = cfg["credit"]
    w = cfg["weights"]
    terms = {
        "alignment": credit(m.get("projection_px"), *c["projection_px"]),
        "downstream": 0.5 * (credit(m.get("rpe_rot_deg"), *c["rpe_rot_deg"])
                             + credit(m.get("rpe_trans_m"), *c["rpe_trans_m"])),
        "time_sync": credit(m.get("time_rms_s"), *c["time_rms_s"]),
        "imu": (credit(m.get("imu_rot_deg"), *c["imu_rot_deg"])
                + credit(m.get("gyro_bias_err_rps"), *c["gyro_bias_err_rps"])
                + credit(m.get("accel_bias_err_mps2"), *c["accel_bias_err_mps2"])) / 3.0,
        "faults": float(m.get("fault_f1", 0.0)),
    }
    total = sum(w[k] * terms[k] for k in terms)
    return total, terms


def score_rig(truth, held, out_dir, cfg):
    import rigmetrics as M
    m = {}
    notes = []
    text, err = _read_small(os.path.join(out_dir, "calibration.yaml"))
    if text is None:
        notes.append("calibration.yaml: " + err)
    else:
        try:
            cal = M.parse_calibration(text)
            m.update(M.calibration_metrics(truth, held, cal))
        except M.CalibrationError as exc:
            notes.append(str(exc))
    text, err = _read_small(os.path.join(out_dir, "fault_report.json"))
    if text is None:
        notes.append("fault_report.json: " + err)
    else:
        try:
            rep = M.parse_fault_report(text)
            m["fault_f1"] = M.fault_f1(truth["faults"], rep)
        except M.CalibrationError as exc:
            notes.append(str(exc))
    total, terms = rig_credit(m, cfg)
    return total, terms, m, notes


# ----------------------------------------------------------------------------- main

def main():
    secure_exec_path()
    cfg = stage_config()
    anchor, meta = read_anchor()
    anchored = anchor is not None
    n_rigs = int(cfg.get("rigs", 6))
    run_sh = os.path.join(SUBMISSION, "run.sh")
    if not os.path.isfile(run_sh):
        write_verdict(zero_verdict(reason="no /home/user/submission/run.sh", anchored=anchored),
                      diagnostic="scoring 0: no /home/user/submission/run.sh")
        return
    try:
        pw = pwd.getpwnam(agent_user())
    except KeyError:
        fail("the agent account does not exist in the verifier sandbox")
    uid, gid = pw.pw_uid, pw.pw_gid

    cleared = clear_agent_account()
    if not cleared.get("cleared") and not cleared.get("skipped"):
        fail("the agent's account could not be cleared before grading: {}".format(
            "; ".join(cleared.get("left", []))[:300]))
    if not kill_agent_processes(uid):
        fail("processes of the agent account could not be stopped before grading")

    rng = seed_rng()
    seeds = [[rng.seed(bits=31) for _ in range(4)] for _ in range(n_rigs)]
    work = tempfile.mkdtemp(prefix="rigcal_", dir="/tmp")
    os.chmod(work, 0o755)
    rig_dirs = [os.path.join(work, "rigs", "rig_%02d" % i) for i in range(n_rigs)]
    out_dirs = [os.path.join(work, "out", "rig_%02d" % i) for i in range(n_rigs)]
    t0 = time.monotonic()
    procs = min(n_rigs, max(1, int(cfg.get("gen_workers", 6))))
    ctx = multiprocessing.get_context("fork")
    try:
        with ctx.Pool(procs) as pool:
            results = pool.map(_generate, [(seeds[i], "rig_%02d" % i, rig_dirs[i]) for i in range(n_rigs)],
                               chunksize=1)
    except Exception as exc:  # the grader's own generator failing is a task error
        fail("generating the hidden rigs failed: {}: {}".format(type(exc).__name__, str(exc)[:300]))
    log("generated {} hidden rigs in {:.0f} s".format(n_rigs, time.monotonic() - t0))
    _open_tree(os.path.join(work, "rigs"))
    os.makedirs(os.path.join(work, "out"), exist_ok=True)
    os.chmod(os.path.join(work, "out"), 0o755)
    for od in out_dirs:
        os.makedirs(od)
        os.chown(od, uid, gid)
        os.chmod(od, 0o755)

    t1 = time.monotonic()
    status, tails = run_submission(rig_dirs, out_dirs, uid, gid, cfg)
    log("submission runs finished in {:.0f} s: {}".format(time.monotonic() - t1,
                                                           [(s[0], s[1], round(s[2])) for s in status]))

    credits, per_terms, per_metrics, notes_all = [], [], [], []
    for i in range(n_rigs):
        truth, held = results[i]
        total, terms, m, notes = score_rig(truth, held, out_dirs[i], cfg)
        if status[i][0] == "timeout":
            notes.append("run exceeded %.0f s and was killed" % float(cfg.get("run_timeout_s", 900)))
        elif status[i][1] != 0:
            notes.append("run.sh exited with status %s" % status[i][1])
        credits.append(total)
        per_terms.append(terms)
        per_metrics.append(m)
        notes_all.append(notes)
        log("rig_%02d credit %.4f terms %s metrics %s notes %s" % (
            i, total, {k: round(v, 3) for k, v in terms.items()},
            {k: round(v, 6) for k, v in m.items()}, notes))
    measured = sum(credits) / len(credits)

    extra = {"rigs": float(n_rigs)}
    for k in ("alignment", "downstream", "time_sync", "imu", "faults"):
        extra["term_" + k] = sum(t[k] for t in per_terms) / n_rigs
    for i, c in enumerate(credits):
        extra["rig_%02d_credit" % i] = c
    for key in ("projection_px", "rpe_rot_deg", "rpe_trans_m", "time_rms_s", "imu_rot_deg",
                "gyro_bias_err_rps", "accel_bias_err_mps2", "fault_f1"):
        vals = [m[key] for m in per_metrics if key in m]
        if vals:
            vals.sort()
            extra["median_" + key] = vals[len(vals) // 2]
        extra["rigs_with_" + key] = float(len(vals))
    extra["runs_timed_out"] = float(sum(1 for s in status if s[0] == "timeout"))
    extra["runs_failed"] = float(sum(1 for s in status if s[0] == "exit" and s[1] != 0))

    verdict = ratio_rewards(measured, anchor, cap=cfg.get("scoring_cap", 1.5), higher_is_better=True,
                            metric_name=meta.get("metric", "mean_rig_credit"), extra=extra,
                            full_at=meta.get("full_at", 1.0))
    lines = ["mean_rig_credit %.4f over %d hidden rigs" % (measured, n_rigs)]
    for i in range(n_rigs):
        lines.append("rig_%02d (seed %s): credit %.3f %s %s%s" % (
            i, ",".join(str(w) for w in seeds[i]), credits[i], {k: round(v, 3) for k, v in per_terms[i].items()},
            {k: (round(v, 5) if isinstance(v, float) else v) for k, v in per_metrics[i].items()},
            (" | " + "; ".join(notes_all[i])) if notes_all[i] else ""))
        if not per_metrics[i] and tails[i].strip():
            lines.append("  run output tail: " + tails[i].strip().replace("\n", " | ")[-300:])
    shutil.rmtree(work, ignore_errors=True)
    write_verdict(verdict, diagnostic="\n".join(lines))


if __name__ == "__main__":
    main()
