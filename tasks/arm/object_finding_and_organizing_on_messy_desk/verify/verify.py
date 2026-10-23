"""The grader. Runs as ROOT in the verify stage, after the agent is gone (closed_loop).

The grader owns the simulator (an Isaac Sim process it launches from this folder's env.py)
and drives the agent's run.sh step by step over the ALE wire. The score is computed from
the simulator's privileged state only; nothing the solver prints is a scoring input.

Hidden seeds come from OS entropy inside this process. A probed step is asked twice with
the same `t` (real and perturbed proprioception/pixels, hidden order); only the real reply
reaches the simulator. An aborted episode (deadline, crash, malformed action) scores 0 and
is never dropped. An episode that exhausts its wall-clock budget is cut short: the steps
it never ran earn nothing.
"""

import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from messy_desk_score import EpisodeScorer  # noqa: E402
from robotics_grader import (  # noqa: E402
    ProbeLock,
    Solver,
    SolverAborted,
    SolverMissing,
    fail,
    ratio_rewards,
    read_anchor,
    seed_rng,
    stage_config,
    stage_dir,
    write_verdict,
    zero_verdict,
)
from sim_client import SimClient, SimError  # noqa: E402

ACTION_DIM = 8


def _action(reply):
    try:
        a = [float(v) for v in reply]
    except (TypeError, ValueError, OverflowError):
        raise SolverAborted("malformed action: {!r}".format(reply)[:160])
    if len(a) != ACTION_DIM or not all(math.isfinite(v) for v in a):
        raise SolverAborted("action must be {} finite numbers".format(ACTION_DIM))
    return a


def _perturbed(obs, rng):
    """A copy of the observation that an input-reading policy must answer differently:
    proprioception shifted, pixels untouched (re-encoding every frame would cost more
    than it proves). The step index stays the same."""
    p = dict(obs)
    p["joint_pos"] = [v + rng.uniform(-0.15, 0.15) for v in obs["joint_pos"]]
    ee = list(obs["ee_pose"])
    for i in range(3):
        ee[i] += rng.uniform(-0.04, 0.04)
    p["ee_pose"] = ee
    p["gripper_width"] = max(0.0, obs["gripper_width"] + rng.uniform(-0.02, 0.02))
    return p


def run_episodes(config, rng, solver):
    episodes = int(config.get("episodes", 3))
    horizon = int(config.get("horizon", 20000))
    wall_budget = float(config.get("episode_wall_s", 2700))
    lock = ProbeLock(rng, rate=config.get("probe_rate", 0.01),
                     pass_fraction=config.get("probe_pass_fraction", 0.2))
    env_script = os.path.join(stage_dir(), "env.py")
    try:
        sim = SimClient(env_script, ready_timeout=float(config.get("sim_ready_timeout_s", 900)),
                        log_path=os.path.join(stage_dir(), "sim.log"))
    except SimError as exc:
        fail("simulator failed to start: {}".format(exc))
    print("[grader] simulator up in {:.0f}s".format(sim.startup_s), file=sys.stderr, flush=True)

    scores, summaries, aborted, cut = [], [], 0, 0
    try:
        for episode in range(episodes):
            seed = rng.seed(bits=64)
            try:
                obs, state = sim.reset(seed, horizon)
            except SimError as exc:
                fail("simulator reset failed: {}".format(exc))
            scorer = EpisodeScorer(horizon)
            started = time.time()
            try:
                solver.reset()
                for t in range(horizon):
                    if lock.due():
                        action = lock.probe(solver, t, obs, perturbed=_perturbed(obs, rng))
                    else:
                        action = solver.act(t, obs)
                    action = _action(action)
                    try:
                        obs, state = sim.step(action)
                    except SimError as exc:
                        fail("simulator step failed: {}".format(exc))
                    scorer.add_step(state)
                    if time.time() - started > wall_budget:
                        cut += 1
                        print("[grader] episode {} cut at t={} (wall budget {:.0f}s)".format(
                            episode, t + 1, wall_budget), file=sys.stderr, flush=True)
                        break
            except SolverAborted as exc:
                aborted += 1
                scores.append(0.0)
                summaries.append({"aborted": str(exc)[:160], "steps": scorer.steps})
                print("[grader] episode {} aborted at t={} ({}); scored 0".format(
                    episode, scorer.steps, exc), file=sys.stderr, flush=True)
                continue
            scores.append(scorer.score())
            summaries.append(scorer.summary())
            print("[grader] episode {}: score={:.4f} {}".format(episode, scorer.score(),
                  json.dumps(scorer.summary(), sort_keys=True)), file=sys.stderr, flush=True)
    finally:
        sim.close()
    return scores, summaries, aborted, cut, lock


def main():
    config = stage_config()
    rng = seed_rng()
    anchor, anchor_meta = read_anchor()

    try:
        solver = Solver(step_timeout=config.get("step_timeout_s", 30))
    except (SolverMissing, SolverAborted) as exc:
        # An untouched sandbox, or a submission that never comes up, is a real all-zero
        # verdict, not a task error; and no simulator is launched for it.
        write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None))
        return

    try:
        scores, summaries, aborted, cut, lock = run_episodes(config, rng, solver)
    finally:
        solver.close()

    measured = sum(scores) / len(scores) if scores else 0.0
    extra = dict(lock.rewards(), episodes_aborted=float(aborted), episodes_cut=float(cut))
    for i, s in enumerate(scores):
        extra["episode_{}_score".format(i)] = float(s)
    for key in ("requests_success", "requests_failed", "requests_delivered", "requests_expired", "absent_correct",
                "absent_wrong", "first_contacts", "objects_lost"):
        extra["total_" + key] = float(sum(s.get(key, 0) for s in summaries))
    for key in ("organization", "request_rate"):
        vals = [s[key] for s in summaries if key in s]
        extra["mean_" + key] = float(sum(vals) / len(vals)) if vals else 0.0
    verdict = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5),
                            higher_is_better=anchor_meta.get("direction", "higher") == "higher",
                            metric_name="mean_episode_score", locks_ok=lock.ok, extra=extra,
                            full_at=anchor_meta.get("full_at", 1.0))
    write_verdict(verdict)
    print("[grader] measured={:.4f} anchor={} ratio={:.3f} reward={:.3f} lock={} aborted={} cut={}".format(
        measured, anchor, verdict["metrics"]["ratio"], verdict["rewards"]["reward"], lock.ok, aborted, cut),
        file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
