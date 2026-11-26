"""Recalculate the predeclared course normalizer from four null-anchor ALE receipts."""
import argparse
import hashlib
import json
from pathlib import Path


def compute(paths):
    if len(paths) != 4 or len({p.resolve() for p in paths}) != 4:
        raise ValueError("supply four independent verification.json files in N0–N3 order")
    panels = []
    hashes = set()
    for i, path in enumerate(paths):
        raw = path.read_bytes()
        sha = hashlib.sha256(raw).hexdigest()
        if sha in hashes:
            raise ValueError("duplicated verification receipt")
        hashes.add(sha)
        proof = json.loads(raw)
        m = proof["metrics"]
        counts = [int(m[sign + "_successes"]) for sign in ("left", "right")]
        n = int(m["successes"])
        if (proof["reward"] != 0 or m["anchor_recorded"] != 0 or m["episodes"] != 16 or
                any(m[sign + "_episodes"] != 8 for sign in ("left", "right")) or
                n != sum(counts) or n != m["departure_stops"] or m["strict_course_fraction"] != n / 16 or
                not (m["match_lock_ok"] and m["branch_feedback_ok"] and m["state_feedback_ok"])):
            raise ValueError(f"N{i} is not an eligible balanced null-anchor panel")
        panels.append({"id": f"N{i}", "episode_id": path.parent.name, "verification_sha256": sha,
                       "strict_courses": n, "left": counts[0], "right": counts[1], "metrics": m})
    first = panels[0]
    if not (first["left"] and first["right"]):
        raise ValueError("N0 must complete a full course in both directions")
    counts = [p["strict_courses"] for p in panels]
    required = max(12, min(counts) - 1)
    if max(counts) < required or required / counts[0] > 1.5:
        raise ValueError("the registered threshold is inadmissible for this reference")
    return dict(value=counts[0] / 16, full_at=required / counts[0], full_credit_courses=required,
                episodes_per_panel=16, panels=panels)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("receipts", nargs=4, type=Path)
    print(json.dumps(compute(ap.parse_args().receipts), indent=2, allow_nan=False))
