"""The grader of conveyor_precision_kitting. Runs as ROOT in a fresh verifier sandbox, after the agent is gone.

closed_loop: this process owns the kitting cell (cpk_env.py beside this file, a byte copy of the practice cell's)
and drives the agent's controller (`/home/user/submission/run.sh`, at the agent's UID, through the kit's `Solver`)
one control step at a time on hidden shifts. Seeds come from OS entropy in this process and never touch disk; the
credit is computed from this process's simulator state; nothing the controller prints is a scoring input.

The cell does not wait for the controller ("the world keeps moving"): each control step the controller has
REPLY_BUDGET_S of the grader's clock (time.monotonic, from the moment the observation is sent until its reply is
read) to answer. A reply that takes longer is applied at the step it arrives in: the cell runs the steps in between
on a hold command (no motion, the gripper as last commanded, no images, no turnover request), as many as the reply
spent whole budgets, and the controller never sees their observations (its next observation's `t` skips them).
A reply later than `step_timeout_s` (or none at all) aborts the shift. Only the controller's own reply time counts:
the grader's simulation, rendering and wire encoding are not charged, and the probe lock's duplicate query of a step
(its order hidden) is not timed.

Metric: `parts_seated_fraction`, the pockets holding their own part fully seated, key aligned and undamaged at the
end of each shift, over all pockets of all graded shifts (6 per shift). Higher is better. A shift whose controller
aborts (crash, a missed deadline, a malformed action) or whose simulation faults keeps what is seated in its kit at
that moment: standing still earns nothing, so aborting can never beat trying.
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cpk_env as K  # noqa: E402
from robotics_grader import (  # noqa: E402
    ProbeLock,
    Solver,
    SolverAborted,
    SolverMissing,
    ratio_rewards,
    read_anchor,
    seed_rng,
    stage_config,
    write_verdict,
    zero_verdict,
)


class Deadline(object):
    """The verify phase's own deadline (ALE_VERIFY_DEADLINE_SECONDS, when the engine sets one) less a margin, so that
    a run that would outlive the verifier ends its current shift as aborted and still writes a verdict."""

    def __init__(self, margin_s):
        secs = os.environ.get("ALE_VERIFY_DEADLINE_SECONDS")
        self.end = time.monotonic() + float(secs) - margin_s if secs else None

    def expired(self):
        return self.end is not None and time.monotonic() > self.end


class TimedSolver(Solver):
    """The controller's replies are timed against the cell's clock, so it runs at normal priority (the kit's default,
    nice 19, is for graders that do not measure the solver's speed); the grader itself waits while it thinks."""
    lowest_priority = False


def seven_numbers(action):
    """The controller's action as seven finite floats, or a protocol error (the shift aborts)."""
    try:
        vals = [float(v) for v in action]
    except (TypeError, ValueError, OverflowError):
        raise SolverAborted("malformed action: {!r}".format(action)[:160])
    if len(vals) != 7 or not all(math.isfinite(v) for v in vals):
        raise SolverAborted("action must be 7 finite numbers")
    cams = int(round(vals[5]))
    if cams < 0 or cams > 7 or abs(vals[5] - cams) > 1e-6:
        raise SolverAborted("the camera mask must be an integer from 0 to 7")
    return vals


def timed_act(solver, t, wire):
    t0 = time.monotonic()
    a = solver.act(t, wire)
    return a, time.monotonic() - t0


def run_shifts(config, rng, deadline):
    shifts = int(config.get("shifts", 12))
    budget = float(config.get("reply_budget_s", 0.1))
    lock = ProbeLock(rng, rate=config.get("probe_rate", 0.01), pass_fraction=config.get("probe_pass_fraction", 0.1),
                     tolerance=1e-4)
    seated, offered, aborted, faults, per_shift = 0, 0, 0, 0, []
    late_replies, late_steps, replies, slowest = 0, 0, 0, 0.0
    solver = TimedSolver(step_timeout=config.get("step_timeout_s", 5))
    try:
        for ep in range(shifts):
            env = K.CellEnv(render=True)
            seed = rng.seed(bits=64)
            reason = "ok"
            try:
                obs = env.reset(seed)
                if deadline.expired():
                    raise SolverAborted("verifier deadline reached")
                solver.reset()
                done = False
                grip = -1.0
                while not done:
                    if deadline.expired():
                        raise SolverAborted("verifier deadline reached")
                    wire = K.to_wire(obs)
                    t = int(obs["t"])
                    if lock.due():
                        other = K.to_wire(env.perturbed_observation(obs, rng))
                        if rng.random() < 0.5:
                            probe = solver.act(t, other)
                            action, spent = timed_act(solver, t, wire)
                        else:
                            action, spent = timed_act(solver, t, wire)
                            probe = solver.act(t, other)
                        lock.record(probe, action)
                    else:
                        action, spent = timed_act(solver, t, wire)
                    action = seven_numbers(action)
                    replies += 1
                    slowest = max(slowest, spent)
                    # the world kept moving while the controller thought: whole budgets spent run on a hold command
                    n_late = int(spent // budget)
                    if n_late > 0:
                        late_replies += 1
                        late_steps += n_late
                        for _ in range(n_late):
                            obs, done, info = env.step([0.0, 0.0, 0.0, 0.0, grip, 0, 0])
                            if done:
                                break
                        if done:
                            break
                    grip = float(min(1.0, max(-1.0, action[4])))
                    obs, done, info = env.step(action)
                if env.fault is not None:
                    faults += 1
                    reason = "sim fault: " + env.fault
            except SolverAborted as exc:
                aborted += 1
                reason = "aborted: {}".format(exc)
            except K.ActionError as exc:
                aborted += 1
                reason = "aborted: {}".format(exc)
            kit = env.inspect_kit() if env.model is not None else ["empty"] * len(K.POCKETS)
            credit = sum(1 for s in kit if s == "seated")
            lost = env.status.count("lost") if env.model is not None else 0
            env.close()
            seated += credit
            offered += len(K.POCKETS)
            per_shift.append(credit)
            print("[grader] shift {}: kit {} -> {} of {} seated, {} parts lost ({}; t={})".format(
                ep, kit, credit, len(K.POCKETS), lost, reason, env.t if env.model is not None else 0),
                file=sys.stderr, flush=True)
    finally:
        solver.close()
    timing = {"replies": replies, "late_replies": late_replies, "late_steps": late_steps, "slowest_s": slowest}
    return seated, offered, aborted, faults, per_shift, lock, timing


def main():
    t0 = time.monotonic()
    config = stage_config()
    rng = seed_rng()
    anchor, meta = read_anchor()
    deadline = Deadline(float(config.get("deadline_margin_s", 120)))
    try:
        seated, offered, aborted, faults, per_shift, lock, timing = run_shifts(config, rng, deadline)
    except (SolverMissing, SolverAborted) as exc:
        write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None))
        return
    measured = seated / offered if offered else 0.0
    verdict = ratio_rewards(
        measured, anchor, cap=config.get("scoring_cap", 1.5), higher_is_better=meta.get("direction", "higher") == "higher",
        metric_name="parts_seated_fraction", locks_ok=lock.ok,
        extra=dict(lock.rewards(), parts_seated=float(seated), pockets_offered=float(offered),
                   shifts_aborted=float(aborted), shifts_faulted=float(faults),
                   late_replies=float(timing["late_replies"]), late_steps=float(timing["late_steps"]),
                   slowest_reply_s=float(timing["slowest_s"])),
        full_at=meta.get("full_at", 1.0))
    text = ("parts seated {}/{} ({:.3f}) per shift {}; aborted {}, faults {}, lock {}; replies {}, late {} ({} steps), "
            "slowest {:.3f}s; {:.0f}s").format(
        seated, offered, measured, per_shift, aborted, faults, lock.ok, timing["replies"], timing["late_replies"],
        timing["late_steps"], timing["slowest_s"], time.monotonic() - t0)
    write_verdict(verdict, diagnostic=text)
    print("[grader] " + text, file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
