"""Package measured review evidence; never synthesize missing run results."""
import argparse
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
    gate_file, = (root/"ale-confirmation").glob("*/validation.json")
    gate = json.loads(gate_file.read_text())
    assert all(task["passed"] for task in gate["tasks"])
    hashes = {str(path.relative_to(TASK)): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in sorted((TASK/"environment/setup").rglob("*.py"))}
    hashes["instruction.md"] = hashlib.sha256((TASK/"instruction.md").read_bytes()).hexdigest()
    evidence = {"revision": "long-bridge-2026-11-04", "reference_advantage": "exact actuator calibration",
                "reference_scenes": rows, "ale_gate": gate, "source_sha256": hashes,
                "claims": "Development feasibility, not a sealed frontier benchmark or causal memory study."}
    finals = list((root/"astra-pilot/engine").glob("*/*/result.json"))
    if finals:
        result = json.loads(finals[0].read_text())
        verification = json.loads((finals[0].parent/"verification.json").read_text())
        evidence["astra_pilot"] = {"result": result, "verification": verification}
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
