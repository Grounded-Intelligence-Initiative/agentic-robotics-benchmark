"""The grader of dexterous_tool_catch_and_regrasp. Runs as ROOT in a fresh verifier sandbox, after the agent is gone.

closed_loop: this process owns the catching cell (cell.py beside this file, the same cell as the practice cell's) and
drives the agent's controller (`/home/user/submission/run.sh`, at the agent's UID, through the kit's `Solver`) one
control step at a time, over hidden runs of throws. Seeds come from OS entropy in this process and never touch disk;
the credit is computed from this process's simulator state; nothing the controller prints is a scoring input.

The world keeps moving: the cell advances at its control rate (50 Hz) whether or not the controller has answered.
Each step the controller's reply is timed on this process's monotonic clock, from the moment the observation is
handed to the wire until the reply has been read (only that time: the cell's own physics and rendering are not
charged). A reply within one control period acts at this step. A reply that takes longer acts at the step it would
have arrived at: ceil(elapsed / period) - 1 steps run first with the previous action held, and the controller then
sees the observation of the step after its action (the steps in between are never shown). A reply later than
`late_limit_periods` periods is discarded as missed: that many steps run with the previous action held. On top of
that the cell's fixed sensing/actuation latency applies (cell.LATENCY steps between an observation and the step its
reply acts at). Probe-lock queries (the same `t` asked twice, real and perturbed, in a hidden order) charge only the
real query's time.

Metric: `mean_throw_credit`, the mean over all graded throws of each throw's credit (cell.ThrowJudge: 0 for a
dropped or not-held tool, otherwise catch credit plus the functional-grasp credit). Higher is better. A run whose
controller aborts (crash, a missed hard step deadline, a malformed action) keeps the throws it finished and earns
nothing for the rest of that run.
"""

import collections
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("ALLEGRO_XML", os.path.join(HERE, "wonik_allegro", "right_hand.xml"))

import numpy as np  # noqa: E402

import cell as C  # noqa: E402
import wire as W  # noqa: E402
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
    a run that would outlive the verifier ends its current run of throws as aborted and still writes a verdict."""

    def __init__(self, margin_s):
        secs = os.environ.get("ALE_VERIFY_DEADLINE_SECONDS")
        self.end = time.monotonic() + float(secs) - margin_s if secs else None

    def expired(self):
        return self.end is not None and time.monotonic() > self.end


def checked(action):
    """The controller's action, or a protocol error (the run aborts)."""
    try:
        C.parse_action(action)
    except (C.ActionError, TypeError, KeyError) as exc:
        raise SolverAborted("malformed action: {}".format(exc)[:200])
    return action


def canonical(action):
    """The numbers the probe lock compares: wrist position, wrist rotation vector, finger targets."""
    if not isinstance(action, dict):
        raise SolverAborted("action is not an object")
    out = []
    for key, n in (("wrist_pos", 3), ("wrist_rotvec", 3), ("fingers", 16)):
        vals = action.get(key)
        if not isinstance(vals, list) or len(vals) != n:
            raise SolverAborted("action[{!r}] must be a list of {} numbers".format(key, n))
        out.extend(vals)
    return out


def perturbed_copy(wire_obs, rng):
    """The probe's copy of an observation: the hand's reported pose and finger angles moved a little, the touch and
    wrist force/torque readings perturbed by sensor-noise-sized amounts and both camera images shifted by a pixel or
    two, as if the step had been sensed a moment apart."""
    other = dict(wire_obs)
    other["wrist_pos"] = [v + rng.uniform(-0.03, 0.03) for v in wire_obs["wrist_pos"]]
    other["finger_q"] = [v + rng.uniform(-0.05, 0.05) for v in wire_obs["finger_q"]]
    q = np.array(wire_obs["wrist_quat"], float) + np.array([0.0] + [rng.uniform(-0.03, 0.03) for _ in range(3)])
    other["wrist_quat"] = (q / np.linalg.norm(q)).tolist()
    # the channels an honest controller reads after the catch: link touch forces and the wrist force/torque sensor
    other["touch"] = [max(0.0, v * rng.uniform(0.9, 1.1) + rng.uniform(-0.05, 0.05)) for v in wire_obs["touch"]]
    other["ft_force"] = [v + rng.uniform(-0.2, 0.2) for v in wire_obs["ft_force"]]
    other["ft_torque"] = [v + rng.uniform(-0.01, 0.01) for v in wire_obs["ft_torque"]]
    du, dv = rng.choice([-2, -1, 1, 2]), rng.choice([-2, -1, 1, 2])
    for k in wire_obs:
        if k.startswith("img_"):
            img = W.decode_image(wire_obs[k])
            other[k] = W.encode_image(np.roll(np.roll(img, dv, axis=0), du, axis=1))
    return other


def timed_act(solver, t, wire_obs):
    t0 = time.monotonic()
    action = solver.act(t, wire_obs)
    return action, time.monotonic() - t0


def run_all(config, rng, deadline):
    episodes = int(config.get("episodes", 4))
    throws = int(config.get("throws_per_episode", 12))
    period = float(config.get("control_period_s", C.CONTROL_DT))
    late_limit = int(config.get("late_limit_periods", 5))
    lock = ProbeLock(rng, rate=config.get("probe_rate", 0.01), pass_fraction=config.get("probe_pass_fraction", 0.1),
                     tolerance=1e-4, canonical=canonical)
    stats = collections.Counter()
    reply_times = []
    results = []          # one dict per graded throw (aborted throws: credit 0)
    solver = Solver(step_timeout=config.get("step_timeout_s", 10))
    try:
        for ep in range(episodes):
            seed = rng.seed(bits=63)
            env = C.Episode(seed, n_throws=throws, render=True)
            reason = "ok"
            done_throws = []
            try:
                obs = env.reset()
                if deadline.expired():
                    raise SolverAborted("verifier deadline reached")
                solver.reset()
                prev = None       # the action in force; before the first reply, the first reply itself
                k = 0
                done = False
                while not done:
                    if deadline.expired():
                        raise SolverAborted("verifier deadline reached")
                    wire_obs = W.to_wire(obs)
                    if lock.due():
                        other = perturbed_copy(wire_obs, rng)
                        if rng.random() < 0.5:
                            a_probe = solver.act(k, other)
                            action, el = timed_act(solver, k, wire_obs)
                        else:
                            action, el = timed_act(solver, k, wire_obs)
                            a_probe = solver.act(k, other)
                        lock.record(a_probe, action)
                    else:
                        action, el = timed_act(solver, k, wire_obs)
                    action = checked(action)
                    reply_times.append(el)
                    if prev is None:
                        prev = action
                    late = max(0, int(math.ceil(el / period - 1e-9)) - 1)
                    if late >= late_limit:
                        stats["missed"] += 1
                        steps = [prev] * late_limit
                    else:
                        if late:
                            stats["late"] += 1
                            stats["late_steps"] += late
                        steps = [prev] * late + [action]
                        prev = action
                    for a in steps:
                        obs, done, info = env.step(a)
                        k += 1
                        if "throw_result" in info:
                            done_throws.append(info["throw_result"])
                        if done:
                            break
            except SolverAborted as exc:
                stats["aborted_runs"] += 1
                reason = "aborted: {}".format(exc)
            except C.ActionError as exc:
                stats["aborted_runs"] += 1
                reason = "aborted: {}".format(exc)
            env.close()
            results.extend(done_throws)
            for _ in range(throws - len(done_throws)):
                results.append({"score": 0.0, "caught": False, "tool": "unthrown", "zone": None})
            print("[grader] run {}: {} throws, credit {:.3f} ({})".format(
                ep, len(done_throws), sum(r["score"] for r in done_throws), reason), file=sys.stderr, flush=True)
    finally:
        solver.close()
    return results, stats, reply_times, lock


def main():
    t0 = time.monotonic()
    config = stage_config()
    rng = seed_rng()
    anchor, meta = read_anchor()
    deadline = Deadline(float(config.get("deadline_margin_s", 180)))
    try:
        results, stats, reply_times, lock = run_all(config, rng, deadline)
    except (SolverMissing, SolverAborted) as exc:
        write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None))
        return
    credits = [float(r["score"]) for r in results]
    measured = float(np.mean(credits)) if credits else 0.0
    caught = sum(bool(r.get("caught")) for r in results)
    per_tool = collections.defaultdict(list)
    for r in results:
        per_tool[r["tool"]].append(float(r["score"]))
    rt = np.array(reply_times) if reply_times else np.zeros(1)
    extra = dict(lock.rewards(), throws=float(len(results)), throws_caught=float(caught),
                 functional_throws=float(sum(c > 0.9 for c in credits)),
                 replies_late=float(stats["late"]), replies_missed=float(stats["missed"]),
                 runs_aborted=float(stats["aborted_runs"]),
                 reply_p50_s=float(np.percentile(rt, 50)), reply_p99_s=float(np.percentile(rt, 99)))
    verdict = ratio_rewards(
        measured, anchor, cap=config.get("scoring_cap", 1.5),
        higher_is_better=meta.get("direction", "higher") == "higher",
        metric_name="mean_throw_credit", locks_ok=lock.ok, extra=extra, full_at=meta.get("full_at", 1.0))
    text = ("mean throw credit {:.3f} over {} throws; caught {}; functional {}; per tool {}; late replies {} "
            "(+{} steps), missed {}, aborted runs {}; reply time p50 {:.1f} ms p99 {:.1f} ms max {:.1f} ms; "
            "lock {} ({}/{}); {:.0f}s").format(
        measured, len(results), caught, sum(c > 0.9 for c in credits),
        {t: "{:.2f} (n={})".format(np.mean(v), len(v)) for t, v in sorted(per_tool.items())},
        stats["late"], stats["late_steps"], stats["missed"], stats["aborted_runs"],
        1e3 * np.percentile(rt, 50), 1e3 * np.percentile(rt, 99), 1e3 * rt.max(), lock.ok, lock.passed, lock.total,
        time.monotonic() - t0)
    write_verdict(verdict, diagnostic=text)
    print("[grader] " + text, file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
