"""Score only the completed root-owned live interaction; never replay submissions."""
import json
import os
from pathlib import Path
import signal

from robotics_grader import ratio_rewards, read_anchor, require, stage_dir, write_verdict


def main():
    private = Path("/var/lib/visual_robotics")
    require((private/"state.json").is_file(), "missing protected rescue ledger")
    pid = int((private/"pid").read_text())
    try:
        os.kill(pid, 0)
        require(Path(f"/proc/{pid}/stat").read_text().split()[2] != "Z",
                "rescue service crashed")
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        require(False, "rescue service exited unexpectedly")
    state = json.loads((private/"state.json").read_text())
    config = json.loads((Path(stage_dir())/"grader_config.json").read_text())
    require(state["kind"] == config["kind"] == "rescue", "wrong service episode")
    anchor, calibration = read_anchor()
    metrics = state["metrics"]
    measured = metrics["rescue_progress"]
    # Framework-owned verifier output, unavailable to the live agent.
    print("[authoritative-journal] " + json.dumps(state, separators=(",", ":")), flush=True)
    extra = {k: v for k, v in metrics.items() if k != "rescue_progress"}
    extra["trusted_requests"] = state["requests"]
    write_verdict(ratio_rewards(measured, anchor, metric_name=calibration["metric"], extra=extra,
                                full_at=calibration["full_at"]))


if __name__ == "__main__":
    main()
