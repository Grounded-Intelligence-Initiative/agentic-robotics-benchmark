"""Hidden grader for cable_routing_through_clips (closed loop).

Runs as root in a SEPARATE verifier sandbox, after the agent is gone. The engine restores only
/home/user/submission. The grader owns the cell simulator (cable_env.py, byte-identical to the
agent's ~/cell/cable_env.py) and drives the agent's controller through the kit's Solver: `bash
run.sh` in /home/user/submission at the agent's UID, one JSON object per line, READY first.

* `episodes` hidden episodes, each from a 64-bit seed drawn from the kit's seed stream (OS
  entropy, never on disk): a random fixture route (3 to 5 clips and posts), cable length, radius,
  stiffness, friction and density, and a random starting layout.
* Every 0.1 s the controller gets {"type": "act", "t": t, "obs": ...} and answers {"action": {"pos":
  [x, y, z], "yaw": a, "grip": g, "done": bool}}. An episode ends when the controller says done, or
  after MAX_STEPS steps. The grader then opens the gripper, lifts it clear, lets the cable settle
  for 2 s and evaluates the route (cable_env.evaluate): the episode's credit is the fraction of
  the route satisfied in order from the connector, and 0 if the connector was overloaded (more than
  25 N for 0.1 s).
* A step that misses its deadline, a crash or a malformed action aborts the episode: the cable is
  evaluated as it lies at that moment (as if the controller had said done), and the controller is
  restarted for the next episode (the kit's rule, at most 3 restarts).
* Probe lock: some steps are asked twice with the same t, the real observation and a copy whose
  horizontal coordinates are all shifted by the same hidden offset, in a hidden order; only the real
  reply is applied. A controller whose replies ignore the observation fails the lock and scores 0.

The metric `mean_route_credit` (higher is better) is the mean episode credit.
"""

import math
import os
import sys

STAGE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, STAGE)
os.environ.setdefault("OMP_NUM_THREADS", "1")

from robotics_grader import (ProbeLock, Solver, SolverAborted, SolverMissing,  # noqa: E402
                             ratio_rewards, read_anchor, seed_rng, stage_config, write_verdict,
                             zero_verdict)

import cable_env as E  # noqa: E402


def log(msg):
    print("[grader] " + msg, file=sys.stderr, flush=True)


def parse_action(reply):
    """The controller's action as a checked dict, or SolverAborted."""
    if not isinstance(reply, dict):
        raise SolverAborted("action is not an object")
    try:
        pos = [float(v) for v in reply["pos"]]
        yaw = float(reply["yaw"])
        grip = float(reply["grip"])
    except (KeyError, TypeError, ValueError, OverflowError):
        raise SolverAborted("malformed action: {!r}".format(reply)[:160])
    if len(pos) != 3 or not all(math.isfinite(v) for v in pos + [yaw, grip]):
        raise SolverAborted("action needs pos [x, y, z], yaw and grip as finite numbers")
    done = reply.get("done", False)
    return {"pos": pos, "yaw": yaw, "grip": grip, "done": bool(done) if isinstance(done, bool) else False}


def canonical(reply):
    a = parse_action(reply)
    return a["pos"] + [a["yaw"], a["grip"], 1.0 if a["done"] else 0.0]


def shifted(obs, dx, dy):
    """The observation with every horizontal coordinate moved by (dx, dy)."""
    out = dict(obs)
    out["cable"] = [[p[0] + dx, p[1] + dy, p[2]] for p in obs["cable"]]
    g = dict(obs["gripper"])
    g["pos"] = [g["pos"][0] + dx, g["pos"][1] + dy, g["pos"][2]]
    out["gripper"] = g
    c = dict(obs["connector"])
    c["pos"] = [c["pos"][0] + dx, c["pos"][1] + dy]
    out["connector"] = c
    out["fixtures"] = []
    for f in obs["fixtures"]:
        f2 = dict(f)
        f2["pos"] = [f["pos"][0] + dx, f["pos"][1] + dy]
        out["fixtures"].append(f2)
    return out


def run_episodes(config, rng):
    episodes = int(config.get("episodes", 12))
    lock = ProbeLock(rng, rate=config.get("probe_rate", 0.02),
                     pass_fraction=config.get("probe_pass_fraction", 0.5), canonical=canonical)
    env = E.CableRouting()
    credits, details, aborted = [], [], 0
    solver = Solver(step_timeout=config.get("step_timeout_s", 10))
    try:
        for ep in range(episodes):
            seed = rng.seed(bits=64)
            obs = env.reset(seed)
            note = ""
            try:
                solver.reset()
                for t in range(E.MAX_STEPS):
                    if lock.due():
                        ang = rng.uniform(0.0, 2 * math.pi)
                        mag = rng.uniform(0.01, 0.03)
                        reply = lock.probe(solver, t, obs, perturbed=shifted(obs, mag * math.cos(ang),
                                                                              mag * math.sin(ang)))
                    else:
                        reply = solver.act(t, obs)
                    obs, done = env.step(parse_action(reply))
                    if done:
                        break
            except SolverAborted as exc:
                aborted += 1
                note = "aborted: %s" % str(exc)[:120]
            res = env.finish()
            credits.append(res["credit"])
            details.append((res, note, env.steps))
            log("episode %d: credit %.3f routed %d/%d damaged %s peak connector %.1f N steps %d %s" % (
                ep, res["credit"], res["routed"], res["fixtures"], res["damaged"],
                res["peak_connector_force"], env.steps, note))
    finally:
        solver.close()
    return credits, details, aborted, lock


def main():
    config = stage_config()
    rng = seed_rng()
    anchor, meta = read_anchor()
    try:
        credits, details, aborted, lock = run_episodes(config, rng)
    except (SolverMissing, SolverAborted) as exc:
        write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None),
                      diagnostic="scoring 0: %s" % exc)
        return
    measured = sum(credits) / len(credits)
    extra = dict(lock.rewards(), episodes_aborted=aborted, episodes=len(credits))
    extra["episodes_damaged"] = float(sum(1 for r, _, _ in details if r["damaged"]))
    extra["episodes_full_route"] = float(sum(1 for r, _, _ in details if r["routed"] == r["fixtures"]))
    for i, c in enumerate(credits):
        extra["episode_%02d_credit" % i] = c
    verdict = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5), higher_is_better=True,
                            metric_name=meta.get("metric", "mean_route_credit"), locks_ok=lock.ok,
                            extra=extra, full_at=meta.get("full_at", 1.0))
    lines = ["mean_route_credit %.4f over %d hidden episodes; probe lock %s (%d/%d)" % (
        measured, len(credits), "ok" if lock.ok else "FAILED", lock.passed, lock.total)]
    for i, (r, note, steps) in enumerate(details):
        lines.append("episode %d: credit %.3f routed %d/%d %s damaged %s peak %.1f N steps %d %s" % (
            i, r["credit"], r["routed"], r["fixtures"],
            "".join(("c" if c["type"] == "clip" else "p") + ("+" if c["ok"] else "-") for c in r["checks"]),
            r["damaged"], r["peak_connector_force"], steps, note))
    write_verdict(verdict, diagnostic="\n".join(lines))


if __name__ == "__main__":
    main()
