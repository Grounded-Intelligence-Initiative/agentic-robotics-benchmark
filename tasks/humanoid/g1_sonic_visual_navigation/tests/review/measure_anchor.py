"""Calculate the registered obstacle anchor from two actual null-anchor oracle verifications."""

import argparse
import hashlib
import json
import math
from pathlib import Path


def panel(path):
    source = path.read_bytes()
    verdict = json.loads(source)
    m = verdict["metrics"]
    n, count = int(m["episodes"]), int(m["successes"])
    if (
        n != 16
        or count not in (15, 16)
        or m["episodes_aborted"]
        or m["anchor_recorded"]
        or verdict["reward"]
    ):
        raise ValueError(
            "null calibration requires 15 or 16 real completions with all courses accounted for"
        )
    if not math.isclose(m["strict_course_fraction"], count / n, abs_tol=1e-12):
        raise ValueError("the raw fraction disagrees with its denominator")
    for field in (
        "left_successes",
        "right_successes",
        "both_service_successes",
        "left_service_successes",
        "right_service_successes",
    ):
        if m[field] < 1:
            raise ValueError(f"missing strict course completion in {field}")
    rows = [f"trial_{i:02d}_" for i in range(1, n + 1)]
    if sum(m[p + "success"] for p in rows) != count:
        raise ValueError("the number of successful physical trials disagrees with the aggregate")
    for p in rows:
        if m[p + "success"] and (
            not m[p + "native_success"]
            or m[p + "stages_completed"] != 3
            or m[p + "obstacle_contact"]
            or m[p + "obstacle_contact_substeps"]
            or m[p + "course_boundary_violation"]
            or m[p + "missed_passage"]
            or not m.get(p + "shutter_clear_width_m", 0) > 0
            or not m.get(p + "shutter_gap_edge_clearance_m", 0) > 0
            or m[p + "passed_gates"] != m[p + "required_gates"]
        ):
            raise ValueError(f"{p}reported success inconsistent with its physics")
    return dict(
        episodes=n,
        successes=count,
        raw_fraction=count / n,
        verification_sha256=hashlib.sha256(source).hexdigest(),
    )


def measure(bootstrap, spread):
    first, second = panel(bootstrap), panel(spread)
    if first["verification_sha256"] == second["verification_sha256"]:
        raise ValueError("two independently initialized official measurements are required")
    required = min(first["successes"], second["successes"])
    return dict(
        bootstrap=first,
        independent_spread=second,
        full_credit_count=required,
        anchor_value=first["raw_fraction"],
        full_at=required / first["successes"],
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bootstrap", type=Path)
    parser.add_argument("spread", type=Path)
    args = parser.parse_args()
    print(json.dumps(measure(args.bootstrap, args.spread), indent=2))
