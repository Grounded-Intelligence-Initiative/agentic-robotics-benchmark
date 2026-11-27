"""Export a measured checkpoint with its exact log prefix and reproducible recipe.

Author-only tooling. The full schedule is retained: a 6k snapshot of a 24k cosine
schedule is not the same model as a run configured to finish at 6k.
"""

import argparse
import hashlib
import json
import math
import shutil
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def select(run, step):
    config = json.loads((run / "config.json").read_text())
    total = config["steps"]
    if type(step) is not int or not 0 < step <= total:
        raise ValueError(
            "checkpoint step must be inside the original training schedule"
        )
    rows = []
    previous = 0
    with (run / "training_log.jsonl").open() as source:
        for line in source:
            row = json.loads(line)
            if (
                type(row["step"]) is not int
                or row["step"] <= previous
                or not math.isfinite(row["loss"])
            ):
                raise ValueError("invalid source training log")
            previous = row["step"]
            if row["step"] <= step:
                rows.append(line)
    if not rows or json.loads(rows[-1])["step"] != step:
        raise ValueError("checkpoint step has no corresponding training record")
    checkpoint = run / f"model-{step}.pt"
    checkpoint_hash = sha256(checkpoint)
    config.update(
        checkpoint_step=step,
        training_schedule_steps=total,
        checkpoint_sha256=checkpoint_hash,
        checkpoint_training_elapsed_s=json.loads(rows[-1])["elapsed_s"],
        provenance_note="Author reference run; not an autonomous agent trajectory or ALE gate.",
    )
    # Preserve the complete run log as author evidence before truncating the
    # submission log to the selected checkpoint. This is safe to run again.
    full_log = run / "training_log.full.jsonl"
    if previous >= total or not full_log.exists():
        shutil.copyfile(run / "training_log.jsonl", full_log)
    shutil.copyfile(checkpoint, run / "model.pt")
    (run / "config.json").write_text(json.dumps(config, indent=2) + "\n")
    (run / "training_log.jsonl").write_text("".join(rows))
    manifest = {
        p.name: sha256(p)
        for p in sorted(run.iterdir())
        if p.is_file() and p.name != "artifact-sha256.json"
    }
    (run / "artifact-sha256.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return config


def export(source, destination, step):
    if destination.exists():
        raise FileExistsError("refusing to overwrite an existing checkpoint export")
    config = json.loads((source / "config.json").read_text())
    destination.mkdir(parents=True)
    for name in (
        "model.py",
        "train.py",
        "config.json",
        "training_log.jsonl",
        f"model-{step}.pt",
    ):
        shutil.copy2(source / name, destination / name)
    shutil.copy2(__file__, destination / "export_checkpoint.py")
    selected = select(destination, step)
    script = (
        '#!/usr/bin/env bash\nset -euo pipefail\ncd "$(dirname "$0")"\n'
        "python train.py --data /opt/uav-data/train "
        "--initialization /opt/uav-data/resnet18-f37072fd.pth --output . "
        f"--steps {config['steps']} --seed {config['seed']} "
        f"--batch-size {config['batch_size']} --workers {config['workers']}\n"
        f"python export_checkpoint.py --select-in-place . --step {step}\n"
    )
    (destination / "train.sh").write_text(script)
    (destination / "train.sh").chmod(0o755)
    manifest = {
        p.name: sha256(p)
        for p in sorted(destination.iterdir())
        if p.is_file() and p.name != "artifact-sha256.json"
    }
    (destination / "artifact-sha256.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    return selected


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--source", type=Path)
    mode.add_argument("--select-in-place", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--step", type=int, required=True)
    args = parser.parse_args()
    if args.source:
        if args.output is None:
            parser.error("--source requires --output")
        result = export(args.source, args.output, args.step)
    else:
        result = select(args.select_in_place, args.step)
    print(json.dumps(result, indent=2))
