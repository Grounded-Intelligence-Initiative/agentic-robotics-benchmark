"""Fresh-seed, simulated-deadline score for native robosuite PickPlace."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rapid_sort_env import RapidSortEpisode
from robotics_grader import (
    ProbeLock, Solver, SolverAborted, SolverMissing, ratio_rewards, read_anchor,
    seed_rng, stage_config, write_verdict, zero_verdict,
)


def parse_action(value):
    if not isinstance(value, list) or len(value) != 7:
        raise SolverAborted("expected seven action numbers")
    try:
        action = [float(x) for x in value]
    except (ValueError, TypeError, OverflowError) as exc:
        raise SolverAborted("invalid action: {}".format(exc))
    if not all(math.isfinite(x) for x in action):
        raise SolverAborted("non-finite action")
    return action


def run_episodes(config, rng):
    count = int(config.get("episodes", 12))
    lock = ProbeLock(rng, rate=config.get("probe_rate", 0.01),
                     pass_fraction=config.get("probe_pass_fraction", 0.2))
    solver = Solver(step_timeout=float(config.get("step_timeout_s", 10)))
    completed = aborted = total_steps = 0
    try:
        for episode in range(count):
            world = RapidSortEpisode(rng.seed(bits=64))
            try:
                solver.reset()
                obs = world.observe()
                for t in range(world.env.horizon):
                    if lock.due():
                        shifted = dict(obs)
                        shifted["eef_pos"] = [obs["eef_pos"][0] + .02,
                                              obs["eef_pos"][1] - .02,
                                              obs["eef_pos"][2] + .01]
                        action = parse_action(lock.probe(solver, t, obs, perturbed=shifted))
                    else:
                        action = parse_action(solver.act(t, obs))
                    obs, done = world.step(action)
                    if done:
                        break
                completed += int(world.success)
                total_steps += world.t
            except SolverAborted as exc:
                aborted += 1
                print("[grader] episode {} aborted: {}".format(episode, exc),
                      file=sys.stderr, flush=True)
            finally:
                world.close()
    finally:
        solver.close()
    result = {"success_rate": completed / count,
              "complete_successes": completed,
              "episodes_aborted": aborted,
              "mean_steps": total_steps / count}
    result.update(lock.rewards())
    return result, lock.ok


def main():
    config = stage_config()
    rng = seed_rng()
    anchor, meta = read_anchor()
    try:
        result, locks_ok = run_episodes(config, rng)
    except (SolverMissing, SolverAborted) as exc:
        write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None))
        return
    measured = result.pop("success_rate")
    write_verdict(ratio_rewards(
        measured, anchor, cap=config.get("scoring_cap", 1.5),
        higher_is_better=True, metric_name="success_rate",
        locks_ok=locks_ok, extra=result, full_at=meta.get("full_at", 1.0),
    ))


if __name__ == "__main__":
    main()
