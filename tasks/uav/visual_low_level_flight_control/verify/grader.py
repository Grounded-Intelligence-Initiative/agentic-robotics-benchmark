"""Evaluate an immutable image controller at the agent UID on fresh private scenes."""
import json
import os
from pathlib import Path
import secrets
import signal

import mujoco
import numpy as np

from submission_snapshot import snapshot_submission
from robotics_grader import (
    Solver, SolverAborted, SolverMissing, ratio_rewards, read_anchor, require,
    write_verdict, zero_verdict, ProbeLock, seed_rng,
)
from world import World, png

SUBMISSION = "/opt/visual_robotics/evaluation/submission"


def counterfactual_observation(world, observation, rng):
    original = world.data
    alternate = mujoco.MjData(world.model)
    alternate.qpos[:] = original.qpos
    alternate.qvel[:] = original.qvel
    alternate.qpos[:2] += [rng.uniform(.5, .8)*rng.choice((-1, 1)) for _ in range(2)]
    alternate.qpos[:2] = np.clip(alternate.qpos[:2], -3., 3.)
    alternate.qpos[2] = np.clip(alternate.qpos[2]+rng.uniform(.3, .6), .4, 2.)
    try:
        mujoco.mj_forward(world.model, alternate)
        world.data = alternate
        return {**observation, "images": [png(im) for im in world.images()]}
    finally:
        world.data = original


def evaluate_controller(seeds):
    results = []
    with Solver(step_timeout=10, max_restarts=3, submission=SUBMISSION) as solver:
        for seed in seeds:
            world = World("flight", seed)
            lock = ProbeLock(seed_rng(), rate=.1, pass_fraction=.5)
            try:
                solver.reset()
                obs = world.observe()
                while not obs["done"]:
                    if lock.due():
                        alternate = counterfactual_observation(world, obs, lock.rng)
                        action = lock.probe(solver, world.steps, obs, perturbed=alternate)
                    else:
                        action = solver.act(world.steps, obs)
                    obs = world.act(action)
                results.append({**world.metrics(), **lock.rewards(), "aborted": 0})
            except (SolverAborted, ValueError, TypeError, OverflowError) as exc:
                results.append({**world.metrics(), **lock.rewards(), "success": 0., "aborted": 1})
                print(f"[controller-episode-aborted] {exc}", flush=True)
            finally:
                world.close()
    return results


def main():
    private = Path("/var/lib/visual_robotics")
    require((private/"state.json").is_file(), "missing protected environment ledger")
    pid = int((private/"pid").read_text())
    try:
        os.kill(pid, 0)
        require(Path(f"/proc/{pid}/stat").read_text().split()[2] != "Z",
                "world service crashed; environment failure, not agent failure")
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        require(False, "world service exited unexpectedly")
    state = json.loads((private/"state.json").read_text())
    require(state["kind"] == "flight", "wrong service episode")
    anchor, calibration = read_anchor()
    print("[authoritative-journal] " + json.dumps(state, separators=(",", ":")), flush=True)
    extra = {"trusted_requests": state["requests"]}
    try:
        snapshot = snapshot_submission("/home/user/submission", SUBMISSION)
        print("[frozen-submission] " + json.dumps(snapshot), flush=True)
    except (ValueError, OSError) as exc:
        write_verdict(zero_verdict(f"invalid submission: {exc}", anchored=anchor is not None))
        return
    if not (Path(SUBMISSION)/"run.sh").is_file():
        write_verdict(zero_verdict("missing deliverables", anchored=anchor is not None))
        return
    seeds = [secrets.randbits(64) for _ in range(4)]
    try:
        results = evaluate_controller(seeds)
    except (SolverMissing, SolverAborted, ValueError, TypeError) as exc:
        write_verdict(zero_verdict(f"controller protocol failure: {exc}", anchored=anchor is not None))
        return
    print("[evaluation-replay-seeds-post-run] " + json.dumps(seeds), flush=True)
    print("[evaluation-episodes] " + json.dumps(results), flush=True)
    success = float(np.mean([r["success"] for r in results]))
    probes = sum(r["probes_total"] for r in results)
    passed = sum(r["probes_passed"] for r in results)
    extra.update({"success_rate": success, "probes_total": probes, "probes_passed": passed,
                  "match_lock_ok": float(probes == 0 or passed/probes >= .5),
                  "evaluation_episodes": len(results),
                  "episodes_aborted": sum(r["aborted"] for r in results),
                  "mean_collisions": np.mean([r["collisions"] for r in results]),
                  "mean_actions": np.mean([r["steps"] for r in results]),
                  "mean_gates_passed": np.mean([r["gates_passed"] for r in results])})
    write_verdict(ratio_rewards(success, anchor, metric_name=calibration["metric"], extra=extra,
                               locks_ok=bool(extra["match_lock_ok"]), full_at=calibration["full_at"]))


if __name__ == "__main__":
    main()
