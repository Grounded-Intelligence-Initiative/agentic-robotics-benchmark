"""The grader of toy_car_assembly_from_manual. Runs as root in a separate verifier sandbox, after the agent is gone.

closed_loop: this process owns the cell (toycar_env.py and toycar_manual.py beside this file). It drives the agent's
controller (`/home/user/submission/run.sh`, run at the agent's UID by robotics_grader.Solver) step by step on hidden
seeds.

Per episode: the seed draws a car of the toy family (toycar_env.sample_kit) and renders its picture manual
(toycar_manual.render_manual), which the controller receives with the observation at t = 0 and never again; the
car's parts and the sibling sets lie at fresh random poses, the stand and the overview camera are moved; the
controller builds. The car's numbers exist only in this process. The episode ends when every mate is filled, after MAX_STEPS steps, when the controller's thinking time
for the episode runs out, or when the controller aborts (crash, missed deadline, malformed action). Then:

    episode score = 0.8 * (mates filled by the kit's own part) / 8 + 0.2 * (all 8 mates hold kit parts and the car
                    passes the roll test)

The roll test sets whatever latched on the stand onto a clean table and pushes it (toycar_env.roll_test). The
metric `assembly_score` is the mean over the episodes; the reward is the kit's ratio against the reference
controller's own measured value (tests/anchor.json).
"""

import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402

from robotics_grader import (  # noqa: E402
    ProbeLock,
    Solver,
    SolverAborted,
    SolverMissing,
    VerifierDeadline,
    ratio_rewards,
    read_anchor,
    seed_rng,
    stage_config,
    write_verdict,
    zero_verdict,
)

import toycar_env as E  # noqa: E402


def canonical(action):
    """The solver's action as six finite floats, or a protocol error (the episode aborts)."""
    if not isinstance(action, (list, tuple)) or len(action) != 6:
        raise SolverAborted("malformed action: {!r}".format(action)[:160])
    out = []
    for v in action:
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise SolverAborted("malformed action: {!r}".format(action)[:160])
        v = float(v)
        if not math.isfinite(v) or abs(v) > 1e3:
            raise SolverAborted("non-finite or huge action value")
        out.append(v)
    return out


def episode_score(credit, rolls):
    """0.8 for the mates (by kit parts), 0.2 for the roll test of the manual's car: every mate holds a kit part and
    the car rolls."""
    return 0.8 * credit / 8.0 + 0.2 * (1.0 if (rolls and credit == 8) else 0.0)


class Grader(object):
    def __init__(self, config, rng):
        self.config = config
        self.rng = rng
        self.lock = ProbeLock(rng, rate=config.get("probe_rate", 0.01),
                              pass_fraction=config.get("probe_pass_fraction", 0.3), canonical=canonical)
        self.results = []
        self.aborted = 0
        self.timed_out = 0

    def measured(self, episodes):
        """Mean episode score, an unplayed episode counting 0."""
        scores = [r["score"] for r in self.results]
        scores += [0.0] * (episodes - len(scores))
        return sum(scores) / float(episodes)

    def run(self, solver, episodes):
        env = E.ToyCarEnv()                                # a car of the family per episode, drawn from its seed
        budget = float(self.config.get("controller_budget_s", 600))
        for ep in range(episodes):
            prng = np.random.default_rng(self.rng.seed(bits=63))
            for _draw in range(20):
                seed = self.rng.seed(bits=64)
                try:
                    obs = env.reset(seed)
                    break
                except E.EnvError as exc:              # a layout that does not fit: draw another seed
                    print("[grader] redrawing a seed: {}".format(exc), file=sys.stderr, flush=True)
            else:
                raise RuntimeError("no episode could be laid out")
            used = 0.0
            how = "built"
            try:
                solver.reset()
                t = 0
                while True:
                    wire = E.to_wire(obs)
                    t0 = time.monotonic()
                    if self.lock.due():
                        pert = E.to_wire(env.perturbed_observation(obs, prng))
                        action = self.lock.probe(solver, t, wire, perturbed=pert)
                    else:
                        action = solver.act(t, wire)
                    used += time.monotonic() - t0
                    obs, done, _info = env.step(canonical(action))
                    t += 1
                    if done:
                        break
                    if used > budget:
                        how = "thinking time used up"
                        self.timed_out += 1
                        break
            except SolverAborted as exc:
                self.aborted += 1
                how = "aborted: {}".format(exc)[:200]
            credit = env.credit()
            roll = env.roll_test()
            score = episode_score(credit, roll["rolls"])
            self.results.append({"score": score, "credit": credit, "rolls": bool(roll["rolls"] and credit == 8),
                                 "car_rolls": bool(roll["rolls"]), "steps": env.t,
                                 "how": how, "fault": env.fault, "damaged": len(getattr(env, "damaged", ()))})
            print("[grader] episode {}: {} mates by kit parts, rolls={} ({}), steps={}, {}, score={:.3f}".format(
                ep, credit, roll["rolls"], roll.get("reason", ""), env.t, how, score), file=sys.stderr, flush=True)
        env.close()


def main():
    config = stage_config()
    rng = seed_rng()
    anchor, meta = read_anchor()
    episodes = int(config.get("episodes", 6))
    cap = config.get("scoring_cap", 1.5)
    full_at = meta.get("full_at", 1.0)
    grader = Grader(config, rng)

    def verdict(note):
        measured = grader.measured(episodes)
        extra = dict(grader.lock.rewards(), episodes_aborted=grader.aborted, episodes_timed_out=grader.timed_out,
                     episodes_played=len(grader.results),
                     mates_by_kit_parts=sum(r["credit"] for r in grader.results),
                     cars_that_roll=sum(1 for r in grader.results if r["rolls"]),
                     parts_damaged=sum(r.get("damaged", 0) for r in grader.results))
        v = ratio_rewards(measured, anchor, cap=cap, higher_is_better=True, metric_name="assembly_score",
                          locks_ok=grader.lock.ok, extra=extra, full_at=full_at)
        lines = ["episode {}: {}".format(i, r) for i, r in enumerate(grader.results)]
        v["diagnostic"] = "{}; assembly_score={:.4f}\n{}".format(note, measured, "\n".join(lines))
        return v

    def expire(elapsed):
        write_verdict(verdict("verifier wall-clock cap reached after {:.0f} s".format(elapsed)))

    with VerifierDeadline(float(config.get("wall_cap_s", 6600)), expire):
        try:
            solver = Solver(step_timeout=config.get("step_timeout_s", 30))
        except (SolverMissing, SolverAborted) as exc:
            write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None))
            return
        try:
            grader.run(solver, episodes)
        finally:
            solver.close()
        v = verdict("all episodes played")
    write_verdict(v)
    print("[grader] assembly_score={:.4f} anchor={} reward={:.3f} lock={}".format(
        grader.measured(episodes), anchor, v["rewards"]["reward"], grader.lock.ok), file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
