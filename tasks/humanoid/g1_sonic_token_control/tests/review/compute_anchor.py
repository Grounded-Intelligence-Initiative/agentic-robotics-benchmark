"""Reproduce the registered calibration from four original ALE oracle verification.json files.

    python3 compute_anchor.py N0/verification.json N1/verification.json N2/verification.json N3/verification.json

N0 is the oracle arm of the initial null-anchor ALE validation. The remaining three paths
are the independently scheduled oracle runs described in anchor_protocol.json.
"""
import argparse
import hashlib
import json
from pathlib import Path


def integer(value, name):
    if type(value) not in (int, float) or (isinstance(value, float) and not value.is_integer()):
        raise ValueError(f"{name} must be an integer measurement")
    return int(value)


def compute(paths):
    if len(paths) != 4 or len({p.resolve() for p in paths}) != 4:
        raise ValueError("supply the four distinct N0 through N3 oracle receipts in registered order")
    panels = []
    for i, path in enumerate(paths):
        raw = path.read_bytes()
        verdict = json.loads(raw)
        m = verdict["metrics"]
        episodes, boxes, cylinders = (integer(m[k], k) for k in ("episodes", "box_episodes", "cylinder_episodes"))
        placed, box, cylinder = (integer(m[k], k) for k in ("placed", "box_placed", "cylinder_placed"))
        if (episodes, boxes, cylinders) != (16, 8, 8) or not (0 <= box <= 8 and 0 <= cylinder <= 8
                and placed == box + cylinder and m["strict_placement_fraction"] == placed / episodes):
            raise ValueError(f"N{i} has an inconsistent native placement numerator or denominator")
        if m["anchor_recorded"] != 0 or verdict["reward"] != 0:
            raise ValueError(f"N{i} must be a null-anchor bootstrap measurement")
        panels.append(dict(id=f"N{i}", episode_id=path.parent.name,
                           verification_sha256=hashlib.sha256(raw).hexdigest(), metrics=m))
    nominal = int(panels[0]["metrics"]["placed"])
    if nominal == 0:
        raise ValueError("N0 recorded zero strict placements; this reference cannot provide a positive anchor")
    required = max(1, min(int(p["metrics"]["placed"]) for p in panels) - 1)
    return dict(value=nominal / 16, full_at=required / nominal,
                strict_placements_for_full_credit=required, episodes_per_panel=16, panels=panels)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("receipts", nargs=4, type=Path, metavar="VERIFICATION")
    args = parser.parse_args()
    print(json.dumps(compute(args.receipts), indent=2, allow_nan=False))
