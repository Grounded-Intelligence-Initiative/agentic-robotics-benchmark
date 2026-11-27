"""Package exact GT buffers alongside a genuinely trained baseline's provenance.

No training records are generated here. The original checkpoint, complete logs,
configuration and source are preserved under baseline/. The oracle's config
explicitly distinguishes privileged GT predictions from learned-model accuracy.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import torch


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def package(baseline, output, fixture_path):
    fixture = json.loads(fixture_path.read_text())
    if fixture["role"] != "author_only_ground_truth_oracle" or fixture["schema"] != 1:
        raise ValueError("Unexpected oracle fixture schema")
    state = torch.load(baseline / "model.pt", map_location="cpu", weights_only=True)
    if not state or any(not isinstance(v, torch.Tensor) for v in state.values()):
        raise ValueError("A real trained baseline state dictionary is required")
    config = json.loads((baseline / "config.json").read_text())
    if config["checkpoint_sha256"] != sha(baseline / "model.pt"):
        raise ValueError("Baseline differs from its recorded selected checkpoint")
    if not (baseline / "training_log.jsonl").stat().st_size:
        raise ValueError("Actual baseline training records are required")
    rows = sorted(fixture["predictions"].items())
    keys, offsets, boxes = [], [0], []
    for key, prediction in rows:
        keys.append(list(bytes.fromhex(key)))
        if len(keys[-1]) != 32:
            raise ValueError("Invalid pixel digest")
        boxes.extend(prediction)
        offsets.append(len(boxes))
    combined = {"baseline." + k: v for k, v in state.items()}
    combined.update({"oracle.keys": torch.tensor(keys, dtype=torch.uint8),
                     "oracle.offsets": torch.tensor(offsets, dtype=torch.int64),
                     "oracle.boxes": torch.tensor(boxes, dtype=torch.float64).reshape(-1, 5)})
    output.mkdir(parents=True, exist_ok=True)
    torch.save(combined, output / "model.pt")
    shutil.copyfile(Path(__file__).with_name("gt_model.py"), output / "model.py")
    shutil.copyfile(baseline / "training_log.jsonl", output / "training_log.jsonl")
    payload = {"architecture": "author_gt_oracle_with_trained_baseline_provenance",
               "prediction_source": "Exact privileged ground truth, NOT learned baseline accuracy",
               "baseline_config": config,
               "baseline_checkpoint_sha256": sha(baseline / "model.pt"),
               "training_log_sha256": sha(baseline / "training_log.jsonl"),
               "oracle_fixture_sha256": sha(fixture_path),
               "data_annotations_sha256": fixture["annotations_sha256"],
               "frames": fixture["frames"], "unique_pixel_keys": len(rows)}
    (output / "config.json").write_text(json.dumps(payload, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fixture", type=Path, required=True)
    args = parser.parse_args()
    package(args.baseline, args.output, args.fixture)
