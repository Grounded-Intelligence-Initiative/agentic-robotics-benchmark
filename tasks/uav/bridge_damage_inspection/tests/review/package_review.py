"""Package measured review evidence; never synthesize missing run results."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil

TASK = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=Path, required=True)
    args = parser.parse_args()
    root = args.runs.resolve()
    rows = []
    for name in ("reference-v2", "reference-more"):
        rows += json.loads((root/name/"summary.json").read_text())
    assert len(rows) == 6 and all(row["success"] == 1 for row in rows)
    gate_file, = (root/"ale-isolated-confirmation").glob("*/validation.json")
    gate = json.loads(gate_file.read_text())
    assert all(task["passed"] for task in gate["tasks"])
    gate_metrics = {}
    for role in ("oracle", "untouched"):
        episode = gate["tasks"][0][role]["episode_id"]
        checked = json.loads((gate_file.parent/episode/"verification.json").read_text())
        assert checked["metrics"]["replay_verified"] == 1
        gate_metrics[role] = checked["metrics"]
    hashes = {str(path.relative_to(TASK)): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in sorted((TASK/"environment/setup").rglob("*.py"))}
    for relative in ("instruction.md", "task.toml", "environment/Dockerfile", "tests/bridge_grader.py"):
        hashes[relative] = hashlib.sha256((TASK/relative).read_bytes()).hexdigest()
    evidence = {"revision": "long-bridge-2026-11-04", "reference_advantage": "exact actuator calibration",
                "reference_scenes": rows, "ale_gate": gate, "ale_gate_metrics": gate_metrics,
                "verifier_mode": "separate",
                "source_sha256": hashes,
                "claims": "Development feasibility, not a sealed frontier benchmark or causal memory study."}
    finals = list((root/"astra-pilot/engine").glob("*/*/result.json"))
    if finals:
        assert len(finals) == 1, "select and label additional model runs explicitly"
        result = json.loads(finals[0].read_text())
        verification = json.loads((finals[0].parent/"verification.json").read_text())
        lock = json.loads((finals[0].parent/"lock.json").read_text())
        assert result["status"] == "completed"
        assert lock["agent"]["model"] == "gpt-6-astra"
        metrics = verification["metrics"]
        keys = ("inspection_f1", "true_reports", "false_reports", "missed_defects", "steps",
                "simulation_seconds", "bridge_length_m", "scenario_id", "coverage_fraction")
        started = datetime.fromisoformat(result["started_at"].replace("Z", "+00:00"))
        finished = datetime.fromisoformat(result["finished_at"].replace("Z", "+00:00"))
        evidence["astra_pilot"] = {
            "status": result["status"], "episode_id": result["episode_id"],
            "model": lock["agent"]["model"], "harness": lock["agent"]["harness"],
            "reasoning_effort": lock["agent"]["settings"]["reasoning_effort"],
            "task_commit": lock["agent"]["resources"][0]["source_version"],
            "engine_commit": lock["framework"]["commit"],
            "verifier_mode_at_run": lock["verification"]["mode"],
            "reward": verification["reward"], "metrics": {key: metrics[key] for key in keys},
            "wall_seconds": round((finished-started).total_seconds(), 3),
            "completed_at": result["finished_at"],
            "scope": "One development pilot, before separate-verifier transport hardening. "
                     "Scene, actions, instruction and F1 definition are unchanged. "
                     "Astra solved this scene; frontier difficulty is not established.",
        }
    else:
        evidence["astra_pilot"] = {"status": "pending"}
    solution = TASK/"solution"
    (solution/"validation-evidence.json").write_text(json.dumps(evidence, indent=2)+"\n")
    source = root/"reference-v2/101"
    for src, dst in (("reviewer-overview.png", "preview.png"), ("reviewer-detail.png", "bridge-detail.png"),
                     ("damage-agent-view.png", "agent-view.png"), ("repair-agent-view.png", "repair-view.png"),
                     ("interaction.json", "success-interaction.json"), ("result.json", "success-example.json")):
        shutil.copyfile(source/src, solution/dst)
    for seed, folder, label in ((102, "reference-v2", "steel"), (104, "reference-more", "maintenance")):
        shutil.copyfile(root/folder/str(seed)/"reviewer-overview.png", solution/f"{label}-overview.png")
        shutil.copyfile(root/folder/str(seed)/"reviewer-detail.png", solution/f"{label}-detail.png")
    print(json.dumps({"reference_successes": len(rows), "gate": "passed",
                      "astra_status": "recorded" if finals else "pending"}))


if __name__ == "__main__":
    main()
