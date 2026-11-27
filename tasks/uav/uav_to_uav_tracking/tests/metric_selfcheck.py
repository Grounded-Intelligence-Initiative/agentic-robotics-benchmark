"""Author-only GT/empty metric checks; never a model or an admission oracle.

Run against a complete frozen annotation export. Only aggregate metrics and source
hashes leave this process; no predictions or labels are written to a submission.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from uav_eval import metrics, validate_boxes


def check(path):
    raw = path.read_bytes()
    data = json.loads(raw)
    images = {image["id"]: image for image in data["images"]}
    assert len(images) == len(data["images"]), "duplicate image IDs"
    assert images, "empty image set"
    annotations = data["annotations"]
    assert annotations, "empty annotation set"
    assert len({a["id"] for a in annotations}) == len(annotations), "duplicate annotation IDs"
    counts = Counter()
    predictions = []
    for a in annotations:
        assert a["category_id"] == 1 and not a.get("iscrowd", 0)
        image = images[a["image_id"]]
        x, y, w, h = a["bbox"]
        validate_boxes([[x, y, x+w, y+h, 1.0]], image["width"], image["height"])
        assert math.isclose(a["area"], w*h, rel_tol=1e-8, abs_tol=1e-8)
        counts[a["image_id"]] += 1
        predictions.append({"image_id": a["image_id"], "category_id": 1,
                            "bbox": list(a["bbox"]), "score": 1.0})
    assert max(counts.values()) <= 300, "GT exceeds the prediction limit"
    perfect = metrics(data, predictions)
    empty = metrics(data, [])
    for name in ("uav_ap50", "uav_ap50_95", "uav_ar300"):
        assert math.isclose(perfect[name], 1.0, abs_tol=1e-9), (name, perfect[name])
    assert all(value == 0.0 for value in empty.values()), empty
    return {"annotations_sha256": hashlib.sha256(raw).hexdigest(),
            "frames": len(images), "annotations": len(annotations),
            "negative_frames": len(images)-len(counts),
            "sequences": len({i["sequence_id"] for i in images.values()}),
            "ground_truth_as_predictions": perfect, "empty_predictions": empty,
            "status": "passed"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("annotations", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = {"scope": "Metric correctness only; not model accuracy, training, or ALE admission",
              "metric_source_sha256": hashlib.sha256(
                  Path(__file__).with_name("uav_eval.py").read_bytes()).hexdigest(),
              "splits": {str(p): check(p) for p in args.annotations}}
    args.output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps({"status": "passed", "splits": len(result["splits"])}))


if __name__ == "__main__":
    main()
