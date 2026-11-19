"""Offline author acceptance check. Not part of the verifier or reward."""
from collections import Counter
import json
from pathlib import Path
import sys


def read(path, required):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    key = {(r["site"], r["arm"]): r for r in rows}
    if set(key) != required or len(rows) != len(required):
        raise ValueError(f"incomplete or duplicate cases: {path}")
    return key


def check(root):
    sites = range(8)
    arms = ("adaptive", "ordinary", "crouch", "intermediate")
    terrain = read(root/"final/results.jsonl", {(i, m) for i in sites for m in arms})
    flat = read(root/"flat-final/results.jsonl", {(i, m) for i in sites for m in arms[1:3]})
    swapped = read(root/"swapped/results.jsonl", {(i, m) for i in (0, 2) for m in arms[:3]})
    for batch in (terrain, swapped):
        for (i, arm), r in batch.items():
            positive = batch[i, "adaptive"]
            for k in ("applied_pose", "rgb_passages", "observed_fittings", "launch_pose_world"):
                assert r[k] == positive[k], (i, arm, k)
            heights = [f["top_m"] for f in r["observed_fittings"]]
            expected = [.4954, .1484] if r["duct_first"] else [.1484, .4954]
            assert len(heights) == 2 and max(abs(a-b) for a, b in zip(heights, expected)) < .016
            assert not (r["target_fell"] or r["target_budget_stop"] or r["contact_load_floor"])
            if arm == "adaptive":
                assert r["success"] and r["duct_cleared"] and r["deck_cleared"] and r["second_partition_crossed"]
                assert r["deck_support_feet"] == 4 and r["min_duct_gap_m"] > 0 and r["dock_zone_entered"]
            else:
                assert not r["success"] and r["target_contact"]
                assert (r["contact_duct"] and r["contact_part"] == "crown_guard") or (
                    r["contact_deck"] and r["contact_part"] == "scanner_head")
                if arm != "intermediate":
                    assert r["contact_duct" if arm == "ordinary" else "contact_deck"]
    for (i, arm), r in flat.items():
        assert r["success"] and r["flat"] and not r["observed_fittings"]
        assert r["launch_pose_world"] == terrain[i, arm]["launch_pose_world"]
    for (i, arm), r in swapped.items():
        assert r["swapped"] and r["alternate_survey"] and r["duct_first"] != terrain[i, arm]["duct_first"]
        assert r["launch_pose_world"] == terrain[i, arm]["launch_pose_world"]
    for name, batch in (("main", terrain), ("flat", flat), ("swapped", swapped)):
        print(name, dict(Counter((r["arm"], r["target_outcome"]) for r in batch.values())))


if __name__ == "__main__":
    check(Path(sys.argv[1]))
