"""Hidden grader for cable_routing_through_clips (closed loop).

Runs as root in a SEPARATE verifier sandbox, after the agent is gone. The engine restores only /home/user/submission.
The grader owns the cell (cable_env.py, byte-identical to the practice cell's environment/setup/runtime/cable_env.py)
and drives the agent's controller through the kit's Solver: `bash run.sh` in /home/user/submission at the agent's
UID, one JSON object per line, READY first.

* A fixed set of hidden episodes (grader_config.json `episodes`: 64-bit seeds drawn once from OS entropy and
  pre-registered; never visible to the agent): a random route of 3 to 5 clips and posts ending at a socket, a random
  cable (length, radius, stiffness, friction, density, colour), lighting and starting layout.
* Every 0.1 s the controller gets {"type": "act", "t": t, "obs": ...}: the gripper's state, the fixtures, the
  socket, the connector's load cell, the cameras' calibration and two RGB-D frames, saved as .npy files under
  FRAME_DIR whose paths the observation names (written just before the request, removed after the reply). It answers
  {"action": {"pos": [x, y, z], "yaw": a, "grip": g, "done": bool}}. An episode ends when the controller says done or
  after MAX_STEPS steps; the grader then opens the gripper, lifts it clear, lets the cable settle for 2 s and checks
  the route and the plug (cable_env.evaluate): credit = route share * (0.4 + 0.6 * plug mated), 0 if the connector
  was overloaded (more than 40 N for 0.2 s).
* A step that misses its deadline, a crash or a malformed action aborts the episode: the cable is checked as it lies
  at that moment, and the controller is restarted for the next episode (the kit's rule, at most 3 restarts).
* Probe lock: some steps are asked twice with the same t, the real observation and a copy of the scene shifted on the
  board by a hidden 1-3 cm (every coordinate, and the overhead frame drawn shifted), in a hidden order; only the real
  reply is applied. A controller whose replies ignore the observation fails the lock and scores 0.
* Budget: the controller's replies may take `controller_budget_s` in total (wall time from a request to its reply);
  the episodes left when it runs out are not run and score 0.
* The reference solution is privileged: a submission whose `.oracle_token` hashes to hidden/oracle_token.sha256 also
  gets the cell's true state in each observation (`privileged`: the cable's centre line, the plug's pose, the cable's
  radius and length). No other submission does.

The metric `mean_episode_credit` (higher is better) is the mean episode credit over the hidden set.
"""

import hashlib
import math
import os
import shutil
import stat
import sys
import time

STAGE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, STAGE)
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MUJOCO_GL", "osmesa")

from robotics_grader import (ProbeLock, Solver, SolverAborted, SolverMissing,  # noqa: E402
                             ratio_rewards, read_anchor, seed_rng, stage_config, write_verdict,
                             zero_verdict)

import cable_env as E  # noqa: E402

SUBMISSION = "/home/user/submission"
FRAME_DIR = "/tmp/cable_frames"


def log(msg):
    print("[grader] " + msg, file=sys.stderr, flush=True)


def parse_action(reply):
    """The controller's action as a checked dict, or SolverAborted."""
    if not isinstance(reply, dict):
        raise SolverAborted("action is not an object")
    try:
        pos = [float(v) for v in reply["pos"]]
        yaw = float(reply["yaw"])
        grip = float(reply["grip"])
    except (KeyError, TypeError, ValueError, OverflowError):
        raise SolverAborted("malformed action: {!r}".format(reply)[:160])
    if len(pos) != 3 or not all(math.isfinite(v) for v in pos + [yaw, grip]):
        raise SolverAborted("action needs pos [x, y, z], yaw and grip as finite numbers")
    done = reply.get("done", False)
    return {"pos": pos, "yaw": yaw, "grip": grip, "done": bool(done) if isinstance(done, bool) else False}


def canonical(reply):
    a = parse_action(reply)
    return a["pos"] + [a["yaw"], a["grip"], 1.0 if a["done"] else 0.0]


def privileged_submission():
    """True when the submission carries the reference's token (a regular file, read without following links)."""
    try:
        with open(os.path.join(STAGE, "hidden", "oracle_token.sha256")) as f:
            want = f.read().strip()
        fd = os.open(os.path.join(SUBMISSION, ".oracle_token"), os.O_RDONLY | os.O_NOFOLLOW)
        try:
            if not stat.S_ISREG(os.fstat(fd).st_mode):
                return False
            data = os.read(fd, 4096)
        finally:
            os.close(fd)
    except OSError:
        return False
    return hashlib.sha256(data.strip()).hexdigest() == want


def frame_dir():
    if os.path.isdir(FRAME_DIR):
        shutil.rmtree(FRAME_DIR)
    os.makedirs(FRAME_DIR, mode=0o755)
    os.chmod(FRAME_DIR, 0o755)
    return FRAME_DIR


def run_episodes(config, rng, privileged):
    seeds = [int(s) for s in config["episodes"]]
    budget = float(config.get("controller_budget_s", 3600))
    lock = ProbeLock(rng, rate=config.get("probe_rate", 0.02),
                     pass_fraction=config.get("probe_pass_fraction", 0.5), canonical=canonical)
    env = E.CableRouting()
    fdir = frame_dir()
    credits, details, aborted, used = [], [], 0, 0.0
    solver = Solver(step_timeout=config.get("step_timeout_s", 10))
    try:
        for ep, seed in enumerate(seeds):
            if used >= budget:
                credits.append(0.0)
                details.append((None, "not run: the controller's time budget is used up", 0))
                log("episode %d: not run (budget used up)" % ep)
                continue
            obs = env.reset(seed)
            note = ""
            try:
                t0 = time.monotonic()
                solver.reset()
                used += time.monotonic() - t0
                for t in range(E.MAX_STEPS):
                    real = dict(obs)
                    if privileged:
                        real["privileged"] = env.privileged()
                    paths, arrays = E.write_frames(env, fdir)
                    real["frames"] = paths
                    extra = None
                    try:
                        if lock.due():
                            ang = rng.uniform(0.0, 2 * math.pi)
                            mag = rng.uniform(0.01, 0.03)
                            dx, dy = mag * math.cos(ang), mag * math.sin(ang)
                            other = E.shift_obs(real, dx, dy)
                            extra, _ = E.write_frames(env, fdir, shift=(dx, dy), wrist=arrays["wrist"])
                            other["frames"] = extra
                            t0 = time.monotonic()
                            reply = lock.probe(solver, t, real, perturbed=other)
                        else:
                            t0 = time.monotonic()
                            reply = solver.act(t, real)
                        used += time.monotonic() - t0
                    finally:
                        E.remove_frames(paths)
                        if extra is not None:
                            E.remove_frames(extra)
                    obs, done = env.step(parse_action(reply))
                    if done or obs is None:
                        break
                    if used >= budget:
                        note = "stopped: the controller's time budget ran out"
                        break
            except SolverAborted as exc:
                aborted += 1
                note = "aborted: %s" % str(exc)[:120]
            res = env.finish()
            credits.append(res["credit"])
            details.append((res, note, env.steps))
            log("episode %d: credit %.3f routed %d/%d mated %s damaged %s peak connector %.1f N steps %d %s" % (
                ep, res["credit"], res["routed"], res["fixtures"], res["mated"], res["damaged"],
                res["peak_connector_force"], env.steps, note))
    finally:
        solver.close()
        shutil.rmtree(fdir, ignore_errors=True)
    return credits, details, aborted, lock, used


def main():
    config = stage_config()
    rng = seed_rng()
    anchor, meta = read_anchor()
    privileged = privileged_submission()
    try:
        credits, details, aborted, lock, used = run_episodes(config, rng, privileged)
    except (SolverMissing, SolverAborted) as exc:
        write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None),
                      diagnostic="scoring 0: %s" % exc)
        return
    measured = sum(credits) / len(credits)
    ran = [r for r, _, _ in details if r is not None]
    extra = dict(lock.rewards(), episodes_aborted=aborted, episodes=len(credits))
    extra["episodes_not_run"] = float(len(credits) - len(ran))
    extra["episodes_damaged"] = float(sum(1 for r in ran if r["damaged"]))
    extra["episodes_full_route"] = float(sum(1 for r in ran if r["routed"] == r["fixtures"]))
    extra["episodes_mated"] = float(sum(1 for r in ran if r["mated"]))
    extra["controller_time_s"] = round(used, 1)
    extra["privileged_reference"] = 1.0 if privileged else 0.0
    for i, c in enumerate(credits):
        extra["episode_%02d_credit" % i] = c
    verdict = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5), higher_is_better=True,
                            metric_name=meta.get("metric", "mean_episode_credit"), locks_ok=lock.ok,
                            extra=extra, full_at=meta.get("full_at", 1.0))
    lines = ["mean_episode_credit %.4f over %d hidden episodes; probe lock %s (%d/%d); controller time %.0f s%s" % (
        measured, len(credits), "ok" if lock.ok else "FAILED", lock.passed, lock.total, used,
        "; privileged reference" if privileged else "")]
    for i, (r, note, steps) in enumerate(details):
        if r is None:
            lines.append("episode %d: credit 0.000 %s" % (i, note))
            continue
        lines.append("episode %d: credit %.3f routed %d/%d %s plug %s damaged %s peak %.1f N steps %d %s" % (
            i, r["credit"], r["routed"], r["fixtures"],
            "".join(("c" if c["type"] == "clip" else "p") + ("+" if c["ok"] else "-") for c in r["checks"]),
            "mated" if r["mated"] else "not mated", r["damaged"], r["peak_connector_force"], steps, note))
    write_verdict(verdict, diagnostic="\n".join(lines))


if __name__ == "__main__":
    main()
