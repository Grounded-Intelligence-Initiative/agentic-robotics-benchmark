"""Authoritative closed-loop evaluation of an unprivileged submission."""

from pathlib import Path
import traceback

from robotics_grader import ProbeLock, Solver, SolverAborted, SolverMissing
from robotics_grader import ratio_rewards, read_anchor, seed_rng, stage_config, write_verdict, zero_verdict


def checked_action(value):
    from curtain_env import parse_action

    try:
        return parse_action(value).tolist()
    except (TypeError, ValueError, OverflowError) as error:
        raise SolverAborted(str(error)) from error


def probe_observation(obs, rng):
    state = dict(obs["robot_state"])
    offsets = [rng.uniform(-.025, .025) for _ in state["arm_q_rad"]]
    for key in ("arm_q_rad", "arm_target_rad"):
        state[key] = [q + offset for q, offset in zip(state[key], offsets, strict=True)]
    return {**obs, "robot_state": state}


def run_episode(env, solver, *, steps, lock, rng):
    solver.reset()
    for t in range(steps):
        obs = env.observation()
        if lock.due():
            action = lock.probe(solver, t, obs, perturbed=probe_observation(obs, rng))
        else:
            action = solver.act(t, obs)
        env.step(checked_action(action))
    return env.measure()


def main():
    config = stage_config()
    rng = seed_rng()
    anchor, meta = read_anchor()
    lock = ProbeLock(rng, rate=config["probe_rate"], pass_fraction=config["probe_pass_fraction"],
                     canonical=checked_action)
    try:
        solver = Solver(step_timeout=config["step_timeout_s"])
    except (SolverMissing, SolverAborted) as error:
        write_verdict(zero_verdict(reason=str(error), anchored=anchor is not None))
        return

    app = None
    try:
        from isaaclab.app import AppLauncher

        log_dir = Path("/logs/verifier/isaac")
        log_dir.mkdir(parents=True, exist_ok=True)
        app = AppLauncher(headless=True, enable_cameras=True,
                          kit_args=f"--/log/file={log_dir}/kit.log --/plugins/carb.tasking.plugin/threadCount=8").app
        from curtain_env import CurtainEnv

        env = CurtainEnv(log_dir, seed=rng.seed(bits=64))
        try:
            score = run_episode(env, solver, steps=config["max_steps"], lock=lock, rng=rng)
        except SolverAborted as error:
            verdict = zero_verdict(reason=str(error), anchored=anchor is not None)
        else:
            directions = score["threading_counts_by_direction"]
            verdict = ratio_rewards(
                score["score"], anchor, higher_is_better=True, metric_name=meta["metric"],
                locks_ok=lock.ok, full_at=meta["full_at"],
                extra={
                    **lock.rewards(),
                    "threading_score": score["threading_score"],
                    "installation_score": score["installation_score"],
                    "correct_eyelets": len(score["correct_eyelets"]),
                    "threading_from_negative_end": directions[0],
                    "threading_from_positive_end": directions[1],
                    "rod_on_both_brackets": score["rod_on_both_brackets"],
                },
            )
        write_verdict(verdict)
    except BaseException:
        traceback.print_exc()
        solver.close()
        if app is not None:
            app.close(exit_code=1)
        raise
    solver.close()
    app.close()


if __name__ == "__main__":
    main()
