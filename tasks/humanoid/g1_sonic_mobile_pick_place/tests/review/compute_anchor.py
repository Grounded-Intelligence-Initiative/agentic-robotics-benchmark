"""Compute the registered anchor from official N0–N3 oracle verification receipts."""
import argparse
import hashlib
import json
from pathlib import Path


def compute(paths):
    if len(paths) != 4 or len({p.resolve() for p in paths}) != 4:
        raise ValueError("provide four different N0 through N3 oracle verification receipts")
    panels = []
    counts = []
    hashes = set()
    for i, path in enumerate(paths):
        blob = path.read_bytes()
        receipt = json.loads(blob)
        metrics = receipt["metrics"]
        digest = hashlib.sha256(blob).hexdigest()
        if digest in hashes:
            raise ValueError("calibration requires four independent receipts")
        hashes.add(digest)
        box, cylinder = metrics["box_placements"], metrics["cylinder_placements"]
        count = metrics["strict_mobile_placements"]
        if (receipt["reward"] != 0 or metrics["anchor_recorded"] != 0 or metrics["match_lock_ok"] != 1
                or (metrics["episodes"], metrics["box_episodes"], metrics["cylinder_episodes"]) != (16, 8, 8)
                or box != int(box) or cylinder != int(cylinder) or not (0 <= box <= 8 and 0 <= cylinder <= 8)
                or count != box+cylinder or metrics["strict_mobile_placement_fraction"] != count/16):
            raise ValueError(f"N{i} is not a valid complete null-anchor measurement")
        counts.append(int(count))
        panels.append(dict(panel=f"N{i}", episode=path.parent.name, verification_sha256=digest,
                           strict_placements=int(count), raw=count/16, box=int(box), cylinder=int(cylinder)))
    if not counts[0]:
        raise ValueError("the N0 reference achieved zero strict mobile placements")
    required = max(1, min(counts)-1)
    return dict(value=counts[0]/16, full_at=required/counts[0], full_credit_placements=required,
                episodes=16, panels=panels)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("receipts", type=Path, nargs=4)
    print(json.dumps(compute(parser.parse_args().receipts), indent=2, allow_nan=False))
