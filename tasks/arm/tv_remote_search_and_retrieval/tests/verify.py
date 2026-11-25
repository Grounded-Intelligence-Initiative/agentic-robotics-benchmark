"""Hidden grader for tv_remote_search_and_retrieval (closed loop, fixed hidden episodes).

Runs as root in a separate verifier sandbox, after the agent is gone. It owns the living room
(remote_env.py and room/: the agent never had a simulator of the room, nor its meshes, nor the
list of hiding spots), plays the fixed hidden episodes of hidden/episodes.json in order (each a
seed and a hiding spot: two per spot, twelve in all), drives the agent's controller over the
stdio wire at the agent's UID (`robotics_grader.Solver`) and scores every episode from its own
simulator state (`Env.score`): 1 when the remote was retrieved, 0.4 for laying it open, 0.2 for
getting it into an image the controller asked for, else 0. The metric is the mean over the episodes.

Time: every episode has a controller budget (grader_config.json `controller_budget_s`): the time
the grader waits for the controller's replies plus a fixed charge for every image the controller
asks for. The simulator's own time is not charged, so the grading machine's speed does not change
a score; an episode whose budget runs out stops at that step and is scored as it stands. A
wall-clock cap on the whole run (`wall_cap_s`, inside the stage's timeout) is a safety net only:
episodes not finished by then score 0.

A probed step is asked twice with the same `t`, the real observation and a copy with slightly
shifted joint readings in a hidden order; only the reply to the real one is applied. The record of
whether the replies differ rides in the metrics and does not change the score: a controller that
follows a plan between looks answers both alike, honestly.
"""

import copy
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

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
from robotics_grader.deadline import VerifierDeadline  # noqa: E402

METRIC = "mean_retrieval_score"


def log(text):
    print("[grader] " + text, file=sys.stderr, flush=True)


def load_episodes():
    with open(os.path.join(HERE, "hidden", "episodes.json")) as handle:
        return json.load(handle)["episodes"]


def perturbed_copy(obs, rng):
    """The observation with its joint readings shifted a little (images and the rest unchanged)."""
    other = copy.copy(obs)
    other["q"] = [round(v + rng.uniform(-0.004, 0.004), 6) for v in obs["q"]]
    other["gripper"] = round(obs["gripper"] + rng.uniform(-0.0005, 0.0005), 5)
    return other


def canonical(action):
    values = [float(v) for v in action]
    if len(values) != 10:
        raise ValueError("an action is 10 numbers")
    return values


def run_episode(env, E, solver, lock, spec, config, rng, state):
    """One episode; returns (score, steps, how it ended)."""
    budget = float(config.get("controller_budget_s", 900.0))
    charges = config.get("image_charge_s", {})
    obs = env.reset(spec)
    used = 0.0
    started = time.monotonic()
    solver.reset()
    used += time.monotonic() - started
    done = False
    ended = "time"
    while not done:
        t = env.steps
        started = time.monotonic()
        if lock.due():
            action = lock.probe(solver, t, obs, perturbed=perturbed_copy(obs, rng))
        else:
            action = solver.act(t, obs)
        used += time.monotonic() - started
        try:
            _, mask = env.clip_action(action)
        except (ValueError, TypeError, OverflowError) as exc:
            raise SolverAborted("malformed action: {}".format(exc)[:160])
        used += sum(float(charges.get(name, 0.0)) for name in E.CAMERAS if mask & E.CAMERAS[name][0])
        obs, done = env.step(action)
        state["steps"] += 1
        if not done and used > budget:
            ended = "controller budget"
            break
    if done:
        ended = env.stopped
    return env.score(), env.steps, ended


def main():
    config = stage_config()
    rng = seed_rng()
    anchor, meta = read_anchor()
    episodes = load_episodes()
    state = {"scores": [], "steps": 0, "aborted": 0, "ended": [], "written": False}

    def verdict(lock=None, note=""):
        scores = state["scores"] + [0.0] * (len(episodes) - len(state["scores"]))
        measured = sum(scores) / len(scores)
        extra = {"episodes": len(episodes), "episodes_scored": len(state["scores"]),
                 "episodes_aborted": state["aborted"], "episodes_retrieved": sum(1 for s in scores if s >= 1.0),
                 "control_steps": state["steps"]}
        if lock is not None:
            extra.update(lock.rewards())
        result = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5),
                               higher_is_better=meta.get("direction", "higher") == "higher",
                               metric_name=METRIC, extra=extra, full_at=meta.get("full_at", 1.0))
        text = "{} episodes: {}{}".format(len(episodes), "; ".join(state["ended"]), note)
        write_verdict(result, diagnostic=text[:4000])
        state["written"] = True
        log("measured={:.3f} anchor={} reward={:.3f} {}".format(
            measured, anchor, result["rewards"]["reward"], note))

    try:
        solver = Solver(step_timeout=config.get("step_timeout_s", 30))
    except (SolverMissing, SolverAborted) as exc:
        # An untouched sandbox, or a submission that never comes up: a real zero, not a crash.
        write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None))
        return

    # The simulator is imported only now: an untouched sandbox is judged without it.
    import remote_env as E

    lock = ProbeLock(rng, rate=config.get("probe_rate", 0.01),
                     pass_fraction=config.get("probe_pass_fraction", 0.1), canonical=canonical)
    env = E.Env()

    def out_of_time(elapsed):
        verdict(lock, note=" [wall cap reached after {:.0f} s: the rest score 0]".format(elapsed))

    with VerifierDeadline(float(config.get("wall_cap_s", 6000)), out_of_time):
        try:
            for index, item in enumerate(episodes):
                spec = E.make_episode(int(item["seed"]), item["spot"])
                try:
                    score, steps, ended = run_episode(env, E, solver, lock, spec, config, rng, state)
                except SolverAborted as exc:
                    # A crash, a missed deadline or a malformed reply: the episode scores 0.
                    state["aborted"] += 1
                    score, steps, ended = 0.0, env.steps, "aborted ({})".format(str(exc)[:80])
                state["scores"].append(float(score))
                state["ended"].append("{}:{} {:.1f} after {} steps ({})".format(index, item["spot"], score, steps, ended))
                log(state["ended"][-1])
        finally:
            solver.close()
            env.close()
    verdict(lock)


if __name__ == "__main__":
    main()
