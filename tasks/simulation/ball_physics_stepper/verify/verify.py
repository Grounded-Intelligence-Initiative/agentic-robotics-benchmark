"""Verify a stateful simulator through an unprivileged process, one request at a time.

Unlike a control task, answers are predicted states, checked against an independent
reference. The submitted process never owns the expected trajectory or its score.
The canonical Solver transport is reused via request(); act/ProbeLock are not used
because duplicate step requests intentionally advance this task's state.
"""

import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cases import suite
from reference import Reference
from robotics_grader import (Solver, SolverAborted, SolverMissing, ratio_rewards,
                             read_anchor, seed_rng, stage_config, write_verdict,
                             zero_verdict)


def state_values(reply):
    if not isinstance(reply, dict) or not isinstance(reply.get("state"), dict):
        raise SolverAborted("missing state object")
    state = reply["state"]
    values = []
    for name in ("position", "velocity"):
        pair = state.get(name)
        if not isinstance(pair, list) or len(pair) != 2:
            raise SolverAborted("state requires two components in " + name)
        values.extend(pair)
    values.append(state.get("time"))
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in values):
        raise SolverAborted("state fields must be JSON numbers")
    try:
        result = list(map(float, values))
    except (OverflowError, ValueError) as exc:
        raise SolverAborted("invalid numeric state") from exc
    if not all(math.isfinite(v) for v in result):
        raise SolverAborted("non-finite state")
    return result


def agrees(actual, expected):
    return all(abs(a-b) <= 2e-6 + 2e-7*abs(b) for a, b in zip(actual, expected))


def evaluate(solver, cases):
    reference = Reference()
    successes = responses = correct = 0
    family_counts = {}
    max_error = 0.0
    for family, requests in cases:
        counts = family_counts.setdefault(family, [0, 0])
        counts[1] += 1
        passed = True
        for req in requests:
            expected = (reference.reset(req["config"], req["state"])
                        if req["type"] == "reset"
                        else reference.step(req["dt"], req["impulse"]))
            actual = state_values(solver.request(req))
            target = state_values({"state": expected})
            matched = agrees(actual, target)
            passed = passed and matched
            responses += 1
            correct += int(matched)
            # Cap diagnostics only; huge finite wrong outputs still fail the comparison.
            max_error = max(max_error, min(1e12, max(abs(a-b) for a, b in zip(actual, target))))
        successes += int(passed)
        counts[0] += int(passed)
    details = {"trajectories": len(cases), "trajectories_passed": successes,
               "responses": responses, "responses_passed": correct,
               "response_pass_rate": correct / responses if responses else 0,
               "max_absolute_error": max_error, "protocol_ok": 1}
    details.update({"pass_rate_"+key: a/b for key, (a, b) in family_counts.items()})
    return successes / len(cases), details


def main():
    config = stage_config()
    value, meta = read_anchor()
    cases = suite(seed_rng(), config.get("cases_per_family", 8),
                  config.get("steps_per_case", 40))
    solver = None
    try:
        # Allocate first so a failed handshake can still be cleaned up in finally.
        solver = Solver.__new__(Solver)
        solver.__init__(step_timeout=config.get("step_timeout_s", 1))
        measured, details = evaluate(solver, cases)
        if solver.request({"type": "close"}).get("ok") is not True:
            raise SolverAborted("missing close acknowledgement")
        try:
            exit_code = solver.proc.wait(timeout=1)
        except subprocess.TimeoutExpired as exc:
            solver.proc.kill()
            raise SolverAborted("solver did not exit after close") from exc
        if exit_code != 0:
            raise SolverAborted("solver exited unsuccessfully")
    except (SolverMissing, SolverAborted) as exc:
        write_verdict(zero_verdict(reason=str(exc), anchored=value is not None,
                                  extra={"protocol_ok": 0, "trajectories": len(cases)}))
        return
    finally:
        if solver is not None and hasattr(solver, "proc"):
            solver.close()
    result = ratio_rewards(measured, value, metric_name="trajectory_pass_rate",
                           higher_is_better=True, full_at=meta.get("full_at", 1),
                           cap=config.get("scoring_cap", 1.5), extra=details)
    write_verdict(result)
    print("[grader] trajectories={}/{} responses={}/{}".format(
        details["trajectories_passed"], details["trajectories"],
        details["responses_passed"], details["responses"]), file=sys.stderr)


if __name__ == "__main__":
    main()
