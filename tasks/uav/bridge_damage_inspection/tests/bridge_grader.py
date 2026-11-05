"""Re-score framework-collected private evidence in a fresh verifier sandbox."""
import hashlib
import json
from pathlib import Path

from PIL import Image

from robotics_grader import read_anchor, ratio_rewards, require, write_verdict
from world import World


def replay(state, evidence):
    require(state.get("kind") == "bridge", "wrong task ledger")
    events = state.get("events")
    require(isinstance(events, list) and len(events) <= 2048, "invalid event count")
    require(isinstance(state.get("seed"), int) and 0 <= state["seed"] < 2**64, "invalid replay seed")
    allowed = {"observe": set(), "act": {"action"}, "advance": set(), "report": {"frame_id", "point"}}
    world = World("bridge", state["seed"], render=False)
    try:
        for event in events:
            method, params = event.get("method"), event.get("params")
            try:
                if not isinstance(method, str) or method not in allowed or not isinstance(params, dict) or set(params) != allowed[method]:
                    raise ValueError("invalid request")
                result = getattr(world, method)(**params)
            except (ValueError, TypeError, KeyError, OverflowError):
                require("error" in event, "unrecorded invalid action")
                continue
            require("error" not in event and event.get("frame_id") == result["frame_id"], "event/frame mismatch")
            if method == "report" and "evidence_sha256" in event:
                path = evidence/f'{params["frame_id"]:04d}-0.png'
                require(path.is_file() and not path.is_symlink(), "missing collected evidence")
                require(hashlib.sha256(path.read_bytes()).hexdigest() == event["evidence_sha256"], "changed evidence image")
                with Image.open(path) as image:
                    require(image.size == (384, 384) and image.format == "PNG", "invalid evidence image")
                    image.verify()
            elif method == "report":
                # Invalid frame IDs may be reported, but a valid proof must have
                # been preserved by the trusted service before agent teardown.
                proof = world.last_frames.get(params["frame_id"], {})
                require(not proof.get("visible_defects"), "valid proof missing from private evidence export")
        measured = world.metrics()
        require(measured == state.get("metrics"), "ledger metrics disagree with independent replay")
        return measured
    finally:
        world.close()


def main():
    private = Path("/var/lib/visual_robotics")
    require((private/"state.json").is_file(), "missing framework-collected private ledger")
    state = json.loads((private/"state.json").read_text())
    metrics = replay(state, private/"evidence")
    anchor, calibration = read_anchor()
    print("[independent-replay] "+json.dumps({"metrics": metrics, "requests": state["requests"],
                                            "seed_post_run": state["seed"]}), flush=True)
    write_verdict(ratio_rewards(metrics["inspection_quality"], anchor,
                               metric_name=calibration["metric"], full_at=calibration["full_at"],
                               extra={**metrics, "trusted_requests": state["requests"], "replay_verified": 1.}))


if __name__ == "__main__":
    main()
