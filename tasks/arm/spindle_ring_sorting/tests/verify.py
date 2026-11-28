"""The grader of spindle_ring_sorting. Runs as ROOT in a fresh verifier sandbox, after the agent is gone.

closed_loop: this process owns the ring cell (ring_env.py beside this file) and drives the agent's controller
(`/home/user/submission/run.sh`, at the agent's UID, through the kit's `Solver`) one control step at a time, on hidden
episodes. Seeds come from OS entropy in this process and never touch disk; the credit is computed from this process's
simulator state; nothing the controller prints is a scoring input.

Metric: `ring_sort_score`, the mean over the graded episodes of
    0.8 * (rings sorted / 24) + 0.2 * (complete spindles / 6)
where a ring is sorted when, at the end of its episode, it rests threaded on a spindle whose rings are all of its
variant, none of them damaged, held or moving, and a spindle is complete when it holds all four rings of a variant
that way. Higher is better. An episode whose controller aborts (crash, a missed reply deadline, a malformed action)
or whose simulation faults is scored as the fixture stands at that moment: every start has no sorted ring at all,
so aborting can never beat trying.
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ring_env as K  # noqa: E402
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
    a run that would outlive the verifier ends its current episode as aborted and still writes a verdict."""

    def __init__(self, margin_s):
        secs = os.environ.get("ALE_VERIFY_DEADLINE_SECONDS")
        self.end = time.monotonic() + float(secs) - margin_s if secs else None

    def expired(self):
        return self.end is not None and time.monotonic() > self.end


def six_numbers(action):
    """The controller's action as six finite floats, the last a camera mask 0..3, or a protocol error (the episode
    aborts)."""
    if not isinstance(action, (list, tuple)):
        raise SolverAborted("malformed action: {!r}".format(action)[:160])
    try:
        vals = [float(v) for v in action]
    except (TypeError, ValueError, OverflowError):
        raise SolverAborted("malformed action: {!r}".format(action)[:160])
    if len(vals) != 6 or not all(math.isfinite(v) for v in vals):
        raise SolverAborted("action must be 6 finite numbers")
    if vals[5] not in (0.0, 1.0, 2.0, 3.0):
        raise SolverAborted("the camera mask must be 0, 1, 2 or 3")
    return vals


def run_episodes(config, rng, deadline):
    episodes = int(config.get("episodes", 6))
    lock = ProbeLock(rng, rate=config.get("probe_rate", 0.01), pass_fraction=config.get("probe_pass_fraction", 0.1),
                     tolerance=1e-4)
    scores, rings, complete, damaged, steps, aborted, faults, lines = [], 0, 0, 0, 0, 0, 0, []
    solver = Solver(step_timeout=config.get("step_timeout_s", 10))
    try:
        for ep in range(episodes):
            env = K.RingEnv(render=True)
            seed = rng.seed(bits=64)
            reason = "ok"
            try:
                obs = env.reset(seed)
                if deadline.expired():
                    raise SolverAborted("verifier deadline reached")
                solver.reset()
                done = False
                while not done:
                    if deadline.expired():
                        raise SolverAborted("verifier deadline reached")
                    wire = K.to_wire(obs)
                    t = int(obs["t"])
                    if lock.due():
                        other = K.to_wire(env.perturbed_observation(obs, rng))
                        action = lock.probe(solver, t, wire, perturbed=other)
                    else:
                        action = solver.act(t, wire)
                    obs, done, info = env.step(six_numbers(action))
                if env.fault is not None:
                    faults += 1
                    reason = "sim fault: " + env.fault
            except SolverAborted as exc:
                aborted += 1
                reason = "aborted: {}".format(exc)
            except K.ActionError as exc:
                aborted += 1
                reason = "aborted: {}".format(exc)
            r, c = env.score()
            s = env.episode_score()
            dmg = int(sum(env.damaged))
            env.close()
            scores.append(s)
            rings += r
            complete += c
            damaged += dmg
            steps += env.t
            line = "episode {}: {} rings sorted, {} spindles complete, {} damaged, {} steps -> {:.3f} ({})".format(
                ep, r, c, dmg, env.t, s, reason)
            lines.append(line)
            print("[grader] " + line, file=sys.stderr, flush=True)
    finally:
        solver.close()
    return scores, rings, complete, damaged, steps, aborted, faults, lock, lines


def main():
    t0 = time.monotonic()
    config = stage_config()
    rng = seed_rng()
    anchor, meta = read_anchor()
    deadline = Deadline(float(config.get("deadline_margin_s", 150)))
    try:
        scores, rings, complete, damaged, steps, aborted, faults, lock, lines = run_episodes(config, rng, deadline)
    except (SolverMissing, SolverAborted) as exc:
        write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None))
        return
    measured = sum(scores) / len(scores) if scores else 0.0
    verdict = ratio_rewards(
        measured, anchor, cap=config.get("scoring_cap", 1.5), higher_is_better=meta.get("direction", "higher") == "higher",
        metric_name="ring_sort_score", locks_ok=lock.ok,
        extra=dict(lock.rewards(), rings_sorted=float(rings), spindles_complete=float(complete),
                   rings_damaged=float(damaged), episodes=float(len(scores)), episodes_aborted=float(aborted),
                   episodes_faulted=float(faults), steps=float(steps)),
        full_at=meta.get("full_at", 1.0))
    text = "ring_sort_score {:.4f} over {} episodes {}; rings sorted {}, complete spindles {}, damaged {}, aborted {}, " \
           "faults {}, lock {}; {:.0f}s\n{}".format(measured, len(scores), [round(s, 3) for s in scores], rings,
                                                   complete, damaged, aborted, faults, lock.ok,
                                                   time.monotonic() - t0, "\n".join(lines))
    write_verdict(verdict, diagnostic=text)
    print("[grader] " + text, file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
