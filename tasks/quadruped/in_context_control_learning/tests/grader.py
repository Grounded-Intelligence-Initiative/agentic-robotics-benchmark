"""Trusted scoring in the separate verifier sandbox. No agent process or agent code runs here.

What the grader reads:
- the robot service's ledger, /var/lib/visual_robotics/state.json: the task declares it as an artifact, so the engine
  copies it out of the agent's sandbox (as root; the agent cannot write it) and restores it here, root-owned;
- the engine's trajectory of the agent session (ALE_TRAJECTORY_PATH), for the direct-control audit;
- tests/: the hidden scene set, the world model, the anchor.

It replays the ledger's requests against the scene the ledger names (a scene of the hidden set) and scores the
replayed episode: reward = (deliveries reached / 6) / anchor / full_at, clamped to [0, 1]. A replay that disagrees with
the service's own record is an environment failure (task error), never the agent's low score.
"""
import json
import os
from pathlib import Path

from audit import audit_trajectory
from robotics_grader import ratio_rewards, read_anchor, require, stage_dir, write_verdict, zero_verdict
from service import dispatch
from world import DELIVERIES, KIND, VERSION, World

LEDGER = Path("/var/lib/visual_robotics/state.json")


def replay(state):
    """Re-run the ledger's requests on the same scene, without rendering; the episode's metrics."""
    world = World(state["seed"], render=False)
    for n, event in enumerate(state["events"]):
        try:
            dispatch(world, event.get("method"), event.get("params"))
            error = None
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            error = str(exc)
        require((error is None) == ("error" not in event),
                f"replay diverged from the service at request {n+1}: {error!r} vs {event.get('error')!r}; "
                "an environment failure, not an agent result")
        if error is None:
            require((event.get("steps"), event.get("delivered")) == (world.steps, world.delivered),
                    f"replay diverged from the service at request {n+1} (presses, deliveries); "
                    "an environment failure, not an agent result")
    metrics = world.metrics()
    world.close()
    return metrics


def main():
    root = Path(stage_dir())
    anchor, calibration = read_anchor()
    require(LEDGER.is_file(), "the robot service's ledger was not restored into the verifier "
                              "(an environment failure, not an agent result)")
    state = json.loads(LEDGER.read_text())
    hidden = json.loads((root/"hidden_scenes.json").read_text())
    require(state.get("kind") == KIND and state.get("version") == VERSION, "ledger of another task version")
    index = state.get("scene_index")
    require(isinstance(index, int) and 0 <= index < len(hidden["seeds"]) and hidden["seeds"][index] == state["seed"],
            "the ledger's scene is not in the hidden set")
    events = state["events"]
    extra = {"scene_index": index, "requests": state["requests"], "ledger_events": len(events)}
    if not events:
        write_verdict(zero_verdict("no interaction", anchored=anchor is not None, extra=extra))
        return
    metrics = replay(state)
    for key in ("delivered", "steps", "collisions"):
        require(metrics[key] == state["metrics"][key],
                f"replayed {key} {metrics[key]} differs from the service's {state['metrics'][key]}; "
                "an environment failure, not an agent result")
    extra.update({k: v for k, v in metrics.items() if isinstance(v, (int, float)) and not isinstance(v, bool)})
    path = os.environ.get("ALE_TRAJECTORY_PATH")
    require(path is not None and Path(path).is_file(), "the engine's trajectory is missing; cannot audit the session")
    trajectory = json.loads(Path(path).read_text())
    # The oracle is the engine's own scripted harness (the trajectory's agent block is written by the engine, not
    # by the agent): it drives the robot service through the socket, so the direct-control audit does not apply.
    oracle = (trajectory.get("agent") or {}).get("name") == "oracle"
    extra["scripted_oracle"] = float(oracle)
    if not oracle:
        require(str(trajectory.get("schema_version", "")).startswith("ATIF"),
                "unsupported trajectory format; cannot audit the session")
        passed, reason, stats = audit_trajectory(trajectory, events)
        extra.update(stats)
        extra["direct_control_audit_passed"] = float(passed)
        if not passed:
            ale = (trajectory.get("extra") or {}).get("ale") or {}
            require(not (ale.get("incomplete") and ale.get("incomplete_reason") == "conversion_failed"),
                    "the session's trajectory could not be converted; the audit cannot be run fairly")
            write_verdict(zero_verdict("direct-control audit failed: " + reason,
                                       anchored=anchor is not None, extra=extra))
            return
    measured = metrics["delivered"]/DELIVERIES
    write_verdict(ratio_rewards(measured, anchor, metric_name=calibration["metric"], extra=extra,
                                full_at=calibration["full_at"]))


if __name__ == "__main__":
    main()
