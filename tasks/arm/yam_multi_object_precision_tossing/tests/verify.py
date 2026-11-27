"""Verify the submitted controller or the authenticated historical reference."""
import argparse
import json
from pathlib import Path

from arena_episode import run_episode
from workcell import MAX_ACTIONS
from robotics_grader import (ProbeLock, SolverAborted, SolverMissing, read_anchor,
                             seed_rng, stage_config, write_verdict, zero_verdict)
from isolated_solver import IsolatedSolver
from scoring import score_verdict
from numerics_check import verify_numerics
from reference import is_reference_episode, TrustedReferenceSolver, reference_observation


def main(single_set=False):
    verify_numerics()
    config = stage_config()
    rng = seed_rng()
    privileged = is_reference_episode()
    solver_type = TrustedReferenceSolver if privileged else IsolatedSolver
    observe = reference_observation if privileged else None
    lock = ProbeLock(rng, rate=0 if single_set else config["probe_rate"],
                     pass_fraction=config["probe_pass_fraction"])
    count = 1 if single_set else config["episodes"]
    action_limit = 14 if single_set else MAX_ACTIONS
    totals = {"left": 0, "right": 0}
    episodes = []
    aborted = 0
    metadata = {"single_set_diagnostic": int(single_set),
                "action_limit": action_limit, "privileged_reference": int(privileged)}
    try:
        with solver_type(step_timeout=config["step_timeout_s"]) as solver:
            record_root = Path("/logs/verifier/rounds")
            record_root.mkdir(mode=0o700, parents=True, exist_ok=True)
            for index in range(count):
                try:
                    solver.reset()
                    seed = 17 if single_set else rng.seed(bits=64)
                    other_seed = None if single_set else rng.seed(bits=64)
                    result = run_episode(seed, other_seed, solver, lock,
                                         record_path=record_root / str(index),
                                         max_actions=action_limit, observe=observe)
                except (SolverAborted, ValueError) as exc:
                    episodes.append({"aborted": type(exc).__name__, "message": str(exc)})
                    aborted += 1
                    continue
                for side in totals:
                    totals[side] += result[side]
                episodes.append({key: result[key] for key in ("left", "right", "actions")})
    except (SolverMissing, SolverAborted) as exc:
        verdict = zero_verdict(reason=str(exc), anchored=read_anchor()[0] is not None)
        verdict["metrics"].update(metadata)
        write_verdict(verdict)
        return
    verdict = score_verdict(totals["left"], totals["right"], count, aborted, lock.rewards())
    verdict["metrics"].update(metadata)
    write_verdict(verdict, diagnostic=json.dumps(episodes))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--single-set", action="store_true",
                        help="seed 17, one layout, at most 14 actions; diagnostic only")
    main(parser.parse_args().single_set)
