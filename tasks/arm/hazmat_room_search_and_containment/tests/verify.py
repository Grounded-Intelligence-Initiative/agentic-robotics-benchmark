"""The grader. Runs as ROOT in the verify stage, after the agent is gone.

Pattern: closed_loop. The grader owns the live range (lab_sim.py: the lab, its furniture
and drawers, the packages, the latch, the vessels and what a package is lost by, which the
agent never had; the cart in it is cart_model.py on arm_model.py, byte-identical to the
agent's copies) and drives the agent's controller step by step over the kit's wire
(`robotics_grader.Solver`, at the agent's UID). Every scored episode is a new lab, set of packages
and set of limits, drawn from a 64-bit seed that exists only in this process.

An episode is all or nothing: it is won when every package ends secured in a shut vessel
and the lab is left as it was (drawers shut, stools and chairs in place); a package that was
spilled (tilt, shock or countdown) cannot be undone. The metric is the fraction of episodes won,
`containment_rate`: an outcome on a positive scale, direction higher.

What keeps the number honest:
* tests/ is absent from the sandbox while the agent works; the seeds never touch disk;
* the controller cannot read this process, and nothing it prints is an input to the score:
  an episode's outcome is read from this process's own simulator state;
* a probed step is asked twice with the same `t` (the real observation and a copy whose
  joint and cart readings are shifted, in a hidden order) and only the real reply is applied;
  a step whose observation carries a picture is never probed;
* an episode that aborts (crash, missed deadline, malformed action) counts as not contained:
  never dropped, so giving up on a package never beats trying. The same holds for episodes
  that the grader's own time budget did not reach.
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lab_sim import LabEnv  # noqa: E402
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

N_JOINTS = 7


def canonical(action):
    """An action as the fixed-length list the lock compares and the simulator applies: seven
    finite joint targets, the cart's three velocities (zero when left out), the latch and
    the look commands. Anything else is a protocol error."""
    try:
        q = [float(v) for v in action["q"]]
        base = [float(v) for v in action.get("base", [0.0, 0.0, 0.0])]
        latch = int(action.get("latch", 0))
        look = int(action.get("look", 0))
    except (TypeError, ValueError, KeyError, OverflowError, AttributeError):
        raise SolverAborted("malformed action: {!r}".format(action)[:160])
    if len(q) != N_JOINTS or len(base) != 3 or not all(math.isfinite(v) for v in q + base) \
            or latch not in (0, 1) or look not in (0, 1):
        raise SolverAborted("malformed action: {!r}".format(action)[:160])
    return q + base + [float(latch), float(look)]


def run_episodes(config, rng):
    episodes = int(config.get("episodes", 24))
    shift_q = float(config.get("probe_shift_rad", 0.02))
    shift_b = float(config.get("probe_shift_base", 0.01))
    budget = float(config.get("time_budget_s", 12000))
    lock = ProbeLock(rng, rate=config.get("probe_rate", 0.03),
                     pass_fraction=config.get("probe_pass_fraction", 0.5), canonical=canonical)
    env = LabEnv()
    outcomes, aborted = [], 0
    started = time.time()
    # Raises SolverMissing (no run.sh) or SolverAborted (it never comes up): main() scores both 0.
    solver = Solver(step_timeout=config.get("step_timeout_s", 5))
    try:
        for episode in range(episodes):
            if time.time() - started > budget:
                outcomes.append("not run")
                continue
            obs = env.reset(rng.seed(bits=64))
            try:
                solver.reset()
                done, t = False, 0
                while not done:
                    if obs["camera"] is None and lock.due():
                        copy = env.observation(
                            q_offset=[rng.uniform(-shift_q, shift_q) for _ in range(N_JOINTS)],
                            base_offset=[rng.uniform(-shift_b, shift_b) for _ in range(3)])
                        action = lock.probe(solver, t, obs, perturbed=copy)
                    else:
                        action = solver.act(t, obs)
                    a = canonical(action)
                    obs, done = env.step({"q": a[:N_JOINTS], "base": a[N_JOINTS:N_JOINTS + 3],
                                          "latch": int(a[N_JOINTS + 3]), "look": int(a[N_JOINTS + 4])})
                    t += 1
            except SolverAborted as exc:
                aborted += 1
                outcomes.append("aborted")
                print("[grader] episode {} aborted ({})".format(episode, exc), file=sys.stderr,
                      flush=True)
                continue
            outcomes.append(env.outcome or "spilled:countdown")
            print("[grader] episode {}: {} at {:.1f} s of {:.0f} (peak tilt {:.1f} of {:.0f} deg, "
                  "peak shock {:.1f} of {:.1f}); {:.0f} s on the clock".format(
                      episode, outcomes[-1], env.t * 0.02, env.ep["countdown_s"], env.peak_tilt,
                      env.ep["tilt_limit_deg"], env.peak_shock, env.ep["shock_limit"],
                      time.time() - started), file=sys.stderr, flush=True)
    finally:
        solver.close()
    return outcomes, aborted, lock


def main():
    config = stage_config()
    rng = seed_rng()
    anchor, anchor_meta = read_anchor()
    try:
        outcomes, aborted, lock = run_episodes(config, rng)
    except (SolverMissing, SolverAborted) as exc:
        # An untouched sandbox, or a controller that never comes up: a real outcome, scored 0.
        write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None),
                      diagnostic=str(exc))
        return
    contained = sum(1 for o in outcomes if o == "contained")
    measured = contained / float(len(outcomes)) if outcomes else 0.0
    counts = {}
    for o in outcomes:
        counts[o] = counts.get(o, 0) + 1
    verdict = ratio_rewards(
        measured,
        anchor,
        cap=config.get("scoring_cap", 1.5),
        higher_is_better=anchor_meta.get("direction", "higher") == "higher",
        metric_name="containment_rate",
        locks_ok=lock.ok,
        extra=dict(lock.rewards(), episodes_aborted=aborted, episodes=len(outcomes),
                   episodes_contained=contained),
        full_at=anchor_meta.get("full_at", 1.0),
    )
    diagnostic = "contained {} of {}; outcomes: {}; probes {}/{} differed".format(
        contained, len(outcomes), ", ".join("{} x{}".format(k, v) for k, v in sorted(counts.items())),
        lock.passed, lock.total)
    write_verdict(verdict, diagnostic=diagnostic)
    print("[grader] {}; measured={:.3f} anchor={} ratio={:.3f} reward={:.3f} lock={}".format(
        diagnostic, measured, anchor, verdict["metrics"]["ratio"], verdict["rewards"]["reward"],
        lock.ok), file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
