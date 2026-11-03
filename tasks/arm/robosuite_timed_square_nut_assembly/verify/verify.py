"""Private hidden-seed score for the stock robosuite square nut task."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from square_nut_env import SquareNutEpisode
from robotics_grader import (
    Solver, SolverAborted, SolverMissing, ratio_rewards, read_anchor,
    seed_rng, stage_config, write_verdict, zero_verdict,
)


def parse_action(value):
    if not isinstance(value, list) or len(value) != 7:
        raise SolverAborted("expected seven action numbers")
    try:
        output = [float(item) for item in value]
    except (ValueError, TypeError, OverflowError) as exc:
        raise SolverAborted("invalid action: {}".format(exc))
    if not all(math.isfinite(item) for item in output):
        raise SolverAborted("non-finite action")
    return output


def run_episodes(config):
    count = int(config.get("episodes", 40))
    rng = seed_rng()
    solver = Solver(step_timeout=float(config.get("step_timeout_s", 10)))
    completed = aborted = steps = 0
    try:
        for episode in range(count):
            world = SquareNutEpisode(rng.seed(bits=64))
            try:
                solver.reset()
                obs = world.observe()
                for t in range(world.env.horizon):
                    action = parse_action(solver.act(t, obs))
                    obs, done = world.step(action)
                    if done:
                        break
                completed += int(world.success)
                steps += world.t
            except SolverAborted as exc:
                aborted += 1
                print("[grader] episode {} aborted: {}".format(episode, exc),
                      file=sys.stderr, flush=True)
            finally:
                world.close()
    finally:
        solver.close()
    return {
        "success_rate": completed / count,
        "complete_successes": completed,
        "episodes_aborted": aborted,
        "mean_steps": steps / count,
    }


def main():
    anchor, meta = read_anchor()
    try:
        result = run_episodes(stage_config())
    except (SolverMissing, SolverAborted) as exc:
        write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None))
        return
    measured = result.pop("success_rate")
    write_verdict(ratio_rewards(
        measured, anchor, cap=stage_config().get("scoring_cap", 1.5),
        higher_is_better=True, metric_name="success_rate",
        extra=result, full_at=meta.get("full_at", 1.0),
    ))


if __name__ == "__main__":
    main()
