"""The grader. Runs as ROOT in a fresh verifier sandbox, after the agent is gone.

Pattern: artifact_rollout. The submission is a localization program. For each hidden flight
the grader simulates the swarm in the warehouse (world.py: the same hardware units as in the
agent's three flights, a fresh flight plan, fresh per-flight gyro biases, headings and radio
warm-up amounts), writes the flight's logs in the agent's format, runs the program once at the
agent's UID (`robotics_grader.deprivileged`) and compares the trajectories it writes with the
truth. The metric is the RMS 3-D position error (m) over all drones, all 10 Hz output instants
and all flights: a cost, direction lower.

Per instant the error is capped at 10 m, and a missing or non-finite estimate counts as 10 m:
an estimate cannot do worse than not answering, and not answering is never free.
"""

import csv
import math
import os
import pwd
import shutil
import signal
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import flightio  # noqa: E402
import world as W  # noqa: E402
from robotics_grader import (  # noqa: E402
    deprivileged,
    ratio_rewards,
    read_anchor,
    seed_rng,
    stage_config,
    write_verdict,
    zero_verdict,
)
from robotics_grader.stage import agent_home, agent_user  # noqa: E402

NUMERICS_ENV = {"OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1",
                "OPENBLAS_CORETYPE": "Haswell"}
CAP_M = 10.0
MAX_OUTPUT_BYTES = 64 * 1024 * 1024


def read_estimates(path, n_steps, n_drones):
    """(D, n_steps, 3) estimates; NaN where missing or unusable."""
    est = np.full((n_drones, n_steps, 3), np.nan)
    if not os.path.isfile(path) or os.path.islink(path) or os.path.getsize(path) > MAX_OUTPUT_BYTES:
        return est, "no usable trajectories.csv"
    try:
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    t = float(row["t_s"])
                    i = int(float(row["drone"]))
                    p = [float(row["x_m"]), float(row["y_m"]), float(row["z_m"])]
                except (KeyError, TypeError, ValueError):
                    continue
                k = int(round(t * 10.0))
                if 0 <= i < n_drones and 0 <= k < n_steps and abs(t * 10.0 - k) < 0.05 \
                        and all(math.isfinite(v) for v in p):
                    est[i, k] = p
    except (OSError, csv.Error, UnicodeDecodeError) as exc:
        return est, "trajectories.csv unreadable ({})".format(str(exc)[:100])
    return est, "ok"


def run_flight(sensors, duration, config):
    home = agent_home()
    submission = os.path.join(home, "submission")
    entry = os.path.join(submission, "run.sh")
    pw = pwd.getpwnam(agent_user())
    work = tempfile.mkdtemp(prefix="swarm-grade-", dir="/tmp")
    flight = os.path.join(work, "flight")
    out = os.path.join(work, "out")
    flightio.write_flight(flight, sensors, duration)
    os.makedirs(out)
    for root, dirs, files in os.walk(work):
        os.chown(root, pw.pw_uid, pw.pw_gid)
        for name in files:
            os.chown(os.path.join(root, name), pw.pw_uid, pw.pw_gid)
    os.chmod(work, 0o700)
    env = dict(os.environ)
    env.update(NUMERICS_ENV)
    env["HOME"] = home
    for key in list(env):
        if key.startswith("ALE_VERIFY") or key == "ROBOTICS_GRADER_VERDICT_PATH":
            env.pop(key, None)
    budget = float(config.get("program_timeout_s", 600))
    started = time.time()
    proc = deprivileged(["bash", entry, flight, out], cwd=submission, env=env,
                        stdin=open(os.devnull, "rb"), stdout=sys.stderr, stderr=sys.stderr,
                        start_new_session=True)
    try:
        proc.wait(timeout=budget)
        note = "exit {} after {:.0f} s".format(proc.returncode, time.time() - started)
    except Exception:
        note = "exceeded its {:.0f} s budget".format(budget)
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except OSError:
        pass
    proc.wait()
    n_steps = int(round(duration * 10.0))
    est, status = read_estimates(os.path.join(out, "trajectories.csv"), n_steps, W.N_DRONES)
    shutil.rmtree(work, ignore_errors=True)
    return est, "{}; {}".format(note, status)


def main():
    config = stage_config()
    rng = seed_rng()
    anchor, anchor_meta = read_anchor()
    entry = os.path.join(agent_home(), "submission", "run.sh")
    if not os.path.isfile(entry):
        write_verdict(zero_verdict(reason="no /home/user/submission/run.sh",
                                   anchored=anchor is not None))
        return
    duration = float(config.get("flight_seconds", 150.0))
    errors, notes, answered = [], [], 0
    per_flight = []
    for f in range(int(config.get("flights", 3))):
        g = np.random.default_rng(rng.seed(bits=64))
        truth = W.generate_flight(g, duration)
        sensors = W.simulate_sensors(g, truth)
        est, note = run_flight(sensors, duration, config)
        _, P = flightio.truth_samples(truth)
        err = np.linalg.norm(est - P, axis=2)
        ok = np.isfinite(err)
        answered += int(ok.sum())
        err = np.where(ok, np.minimum(err, CAP_M), CAP_M)
        errors.append(err.ravel())
        per_flight.append(float(np.sqrt(np.mean(err ** 2))))
        notes.append("flight {}: {}; RMSE {:.3f} m, {:.1%} answered".format(
            f, note, per_flight[-1], ok.mean()))
        print("[grader] " + notes[-1], file=sys.stderr, flush=True)
    diagnostic = " | ".join(notes)
    if answered == 0:
        write_verdict(zero_verdict(reason="no usable estimate in any flight",
                                   anchored=anchor is not None), diagnostic=diagnostic)
        return
    measured = float(np.sqrt(np.mean(np.concatenate(errors) ** 2)))
    total = sum(len(e) for e in errors)
    verdict = ratio_rewards(
        measured, anchor,
        cap=config.get("scoring_cap", 1.5),
        higher_is_better=anchor_meta.get("direction", "lower") == "higher",
        metric_name="position_rmse_m",
        extra=dict(answered_fraction=answered / float(total),
                   worst_flight_rmse_m=max(per_flight)),
        full_at=anchor_meta.get("full_at", 1.0),
    )
    write_verdict(verdict, diagnostic=diagnostic)
    print("[grader] measured={:.4f} anchor={} ratio={:.3f} reward={:.3f}".format(
        measured, anchor, verdict["metrics"]["ratio"], verdict["rewards"]["reward"]),
        file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
