"""Generate the author-only GT fixture from the pinned hidden export."""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image

EXPECTED_ANNOTATIONS = "d86c75cc95679a2c843e50fea6c4adc9d44d7589bf02c5ad6f3ad0bf1f8186ed"


def pixel_key(frame):
    height, width = frame.shape[:2]
    return hashlib.sha256(f"{height}x{width}:".encode() + frame.tobytes()).hexdigest()


def export(root, output):
    raw = (root / "annotations.json").read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    if digest != EXPECTED_ANNOTATIONS:
        raise ValueError("Oracle export must use the pinned held-out annotation source")
    data = json.loads(raw)
    boxes = defaultdict(list)
    for row in data["annotations"]:
        if row["category_id"] != 1 or row.get("iscrowd", 0):
            raise ValueError("Unexpected category/crowd in GT source")
        x, y, w, h = row["bbox"]
        boxes[row["image_id"]].append([x, y, x + w, y + h, 1.0])
    frames = {}
    for row in data["images"]:
        path = (root / row["file_name"]).resolve()
        if not path.is_relative_to(root.resolve()):
            raise ValueError("Frame escapes data root")
        with Image.open(path) as image:
            frame = np.array(image.convert("RGB"))
        if frame.shape != (row["height"], row["width"], 3):
            raise ValueError("Unexpected decoded dimensions")
        target = sorted(boxes[row["id"]])
        if len(target) > 300:
            raise ValueError("GT exceeds detector output limit")
        key = pixel_key(frame)
        if key in frames and frames[key] != target:
            raise ValueError("Identical RGB frames have conflicting annotations")
        frames[key] = target
    result = {"schema": 1, "role": "author_only_ground_truth_oracle",
              "annotations_sha256": digest, "frames": len(data["images"]),
              "unique_pixel_keys": len(frames), "predictions": frames}
    output.write_text(json.dumps(result, sort_keys=True, separators=(",", ":")) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "predictions"}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    export(args.data, args.output)
