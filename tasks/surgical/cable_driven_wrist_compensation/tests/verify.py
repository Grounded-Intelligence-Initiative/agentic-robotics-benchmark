"""The grader. Runs as ROOT in a fresh verifier sandbox, after the agent is gone.

Pattern: artifact_rollout. The submission is a program. The grader mounts several hidden
instruments, each new and different (plant.random_instrument: cable ratios, coupling, motor
offsets, servo lags and lost motion within the datasheet's tolerances). For each
it records the calibration recording the robot makes after a mount (case.py: two minutes of
stock-firmware motion, the stereo endoscope's video rendered here, the endoscope pose the robot
reports), hands it with a request of hidden trajectories to the program, run once at the
agent's UID (`robotics_grader.deprivileged`), and plays the returned motor-command sequences
through that instrument back to back, in the request's order, from the state the recording left
(nothing re-settles the wrist), each with fresh run-to-run friction and servo variation. The metric is
the RMS jaw-tip position error (mm) over all instruments, both jaws and every scored
instant: a cost, direction lower.

What keeps the number honest:
* tests/ (this file, the plant, the anchor) is absent while the agent works;
* instruments, trajectories and per-episode variation are drawn from OS entropy in this
  process; only the recording and the request reach the disk, in a folder the program owns;
* the program runs at the agent's UID and cannot read this process or /opt/ale; only the
  motor commands it writes are used, and the error is computed here from the plant's state.

An episode whose commands are missing, malformed or non-finite is charged the error of the
drive's stock firmware (the datasheet's nominal mapping) on that trajectory: never dropped,
so failing an episode never beats trying it.
"""

import json
import os
import shutil
import signal
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import case as C  # noqa: E402
import plant as P  # noqa: E402
import render_frames as RF  # noqa: E402
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

NUMERICS_ENV = {
    "OPENBLAS_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "OPENBLAS_CORETYPE": "Haswell",
}
MAX_RESPONSE_BYTES = 200 * 1024 * 1024


def tip_sq_errors(inst, desired_joints, actual_joints, skip):
    d1, d2 = P.jaw_tips(*desired_joints[skip:].T, inst.l1, inst.l2)
    a1, a2 = P.jaw_tips(*actual_joints[skip:].T, inst.l1, inst.l2)
    return np.concatenate([np.sum((d1 - a1) ** 2, axis=1), np.sum((d2 - a2) ** 2, axis=1)])


def make_cases(config, rng):
    """Hidden instruments, their recordings and requests, from 64-bit OS-entropy seeds."""
    return [C.make_case(np.random.default_rng(rng.seed(bits=64)),
                        int(config.get("episodes_per_instrument", 6)),
                        float(config.get("episode_seconds", 20.0)),
                        float(config.get("friction_spread", 0.03)),
                        float(config.get("lag_spread", 0.03)))
            for _ in range(int(config.get("instruments", 4)))]


def _render(args):
    spec, out = args
    RF.render_spec(spec, out)


def stage_cases(cases):
    """Write every case's folder, render the videos two at a time, hand it all to the agent."""
    import pwd
    from multiprocessing import Pool
    pw = pwd.getpwnam(agent_user())
    work = tempfile.mkdtemp(prefix="wrist-grade-", dir="/tmp")
    folders = []
    for i, cs in enumerate(cases):
        folder = os.path.join(work, "instrument_{:d}".format(i))
        C.write_case(folder, cs)
        folders.append(folder)
    with Pool(2) as pool:
        pool.map(_render, [(cs["rec"]["spec"], os.path.join(f, "calibration", "endoscope"))
                           for cs, f in zip(cases, folders)])
    for root, dirs, files in os.walk(work):
        for name in dirs + files:
            os.chown(os.path.join(root, name), pw.pw_uid, pw.pw_gid)
    os.chown(work, pw.pw_uid, pw.pw_gid)
    os.chmod(work, 0o700)
    return work, folders


def run_program(folder, config):
    """Run the submission on one instrument. Returns ({id: commands} or None, note)."""
    home = agent_home()
    submission = os.path.join(home, "submission")
    entry = os.path.join(submission, "run.sh")
    if not os.path.isfile(entry):
        return None, "no /home/user/submission/run.sh"
    resp = os.path.join(os.path.dirname(folder), os.path.basename(folder) + "_response.json")
    env = dict(os.environ)
    env.update(NUMERICS_ENV)
    env["HOME"] = home
    for key in list(env):
        if key.startswith("ALE_VERIFY") or key in ("ROBOTICS_GRADER_VERDICT_PATH",):
            env.pop(key, None)
    budget = float(config.get("program_timeout_s", 900))
    started = time.time()
    proc = deprivileged(["bash", entry, folder, resp], cwd=submission, env=env,
                        stdin=subprocess_devnull(), stdout=sys.stderr, stderr=sys.stderr,
                        start_new_session=True)
    try:
        proc.wait(timeout=budget)
    except Exception:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except OSError:
            pass
        proc.wait()
        note = "exceeded its {:.0f} s budget".format(budget)
    else:
        note = "exited {} after {:.1f} s".format(proc.returncode, time.time() - started)
    try:
        os.killpg(proc.pid, signal.SIGKILL)   # nothing it started outlives it
    except OSError:
        pass
    commands = {}
    try:
        if os.path.isfile(resp) and not os.path.islink(resp) \
                and os.path.getsize(resp) <= MAX_RESPONSE_BYTES:
            with open(resp) as f:
                data = json.load(f)
            for item in data.get("episodes", []):
                if isinstance(item, dict) and isinstance(item.get("id"), str):
                    commands[item["id"]] = item.get("motor_cmd")
        else:
            note += "; no usable response"
    except (OSError, ValueError, AttributeError, TypeError) as exc:
        note += "; response unreadable ({})".format(str(exc)[:120])
    return commands, note


def subprocess_devnull():
    return open(os.devnull, "rb")


def as_commands(raw, n):
    """An N x 3 finite float array, or None."""
    try:
        arr = np.asarray(raw, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None
    if arr.shape != (n, 3) or not np.all(np.isfinite(arr)):
        return None
    return arr


def main():
    config = stage_config()
    rng = seed_rng()
    anchor, anchor_meta = read_anchor()
    skip = int(round(float(config.get("skip_seconds", 0.25)) / P.DT))
    entry = os.path.join(agent_home(), "submission", "run.sh")
    if not os.path.isfile(entry):
        note = "no /home/user/submission/run.sh"
        write_verdict(zero_verdict(reason=note, anchored=anchor is not None), diagnostic=note)
        return
    cases = make_cases(config, rng)
    work, folders = stage_cases(cases)

    sq, bad, clipped, answered, notes, per_inst = [], 0, 0, 0, [], []
    for i, (cs, folder) in enumerate(zip(cases, folders)):
        commands, note = run_program(folder, config)
        commands = commands or {}
        inst_sq = []
        # the episodes run back to back, in the request's order, from where the calibration
        # recording left the wrist: nothing re-settles it in between
        state = cs["rec"]["end"]
        for e in cs["episodes"]:
            joints = P.pyg_to_joints(e["pyg"])
            cmd = as_commands(commands.get(e["id"]), len(joints))
            if cmd is None:
                bad += 1
                cmd = P.nominal_commands(joints)
            else:
                answered += 1
            clipped += int(np.sum(np.abs(cmd) > P.MOTOR_LIMIT))
            actual, _, state = P.simulate(cs["inst"], cmd, e["scale_r"], e["scale_tau"],
                                          start=state, return_state=True)
            inst_sq.append(tip_sq_errors(cs["inst"], joints, actual, skip))
        inst_sq = np.concatenate(inst_sq)
        sq.append(inst_sq)
        per_inst.append(float(np.sqrt(inst_sq.mean())))
        notes.append("instrument {}: program {}, tip RMSE {:.4f} mm".format(i, note, per_inst[-1]))
        print("[grader] " + notes[-1], file=sys.stderr, flush=True)
    shutil.rmtree(work, ignore_errors=True)
    measured = float(np.sqrt(np.concatenate(sq).mean()))
    n_eps = sum(len(cs["episodes"]) for cs in cases)
    if answered == 0:
        reason = "no episode had usable commands ({})".format("; ".join(notes))
        write_verdict(zero_verdict(reason=reason, anchored=anchor is not None), diagnostic=reason)
        return

    verdict = ratio_rewards(
        measured, anchor,
        cap=config.get("scoring_cap", 1.5),
        higher_is_better=anchor_meta.get("direction", "lower") == "higher",
        metric_name="tip_rmse_mm",
        extra=dict(episodes_invalid=bad, commands_clipped=clipped,
                   worst_instrument_rmse_mm=max(per_inst)),
        full_at=anchor_meta.get("full_at", 1.0),
    )
    diagnostic = "tip RMSE {:.4f} mm over {} instruments, {} episodes ({} invalid); {}".format(
        measured, len(cases), n_eps, bad, "; ".join(notes))
    write_verdict(verdict, diagnostic=diagnostic)
    print("[grader] measured={:.4f} anchor={} ratio={:.3f} reward={:.3f}".format(
        measured, anchor, verdict["metrics"]["ratio"], verdict["rewards"]["reward"]),
        file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
