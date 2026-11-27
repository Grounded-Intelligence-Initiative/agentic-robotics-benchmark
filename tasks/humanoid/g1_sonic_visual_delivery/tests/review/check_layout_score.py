"""Exercise the production scheduler and the opposite-destination and full-layout metrics."""
from collections import Counter, defaultdict
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(1, str(Path(__file__).resolve().parents[2] / "environment"))
from verify import panel_jobs, seed_rng, summarize


def main():
    layout = {"start":"far", "shape":"cylinder", "seed":19}
    seen = Counter()
    for seed in range(300):
        jobs = panel_jobs({"layouts":[layout]}, seed_rng(seed))
        sides = tuple(job[4] for job in jobs)
        assert len(sides) == 3 and set(sides) == {"left", "right"}
        assert all(job[:4] == (0, "far", "cylinder", 19) for job in jobs)
        seen[sides] += 1
    assert len(seen) == 6
    candidate = {"layouts": [{"start": "far", "shape": "cylinder", "eligible_seeds": [41, 51]}]}
    sampled = set()
    for stream in range(40):
        chosen = {job[3] for job in panel_jobs(candidate, seed_rng(stream))}
        assert len(chosen) == 1
        sampled.update(chosen)
    assert sampled == {41, 51}
    def row(side, ok):
        return defaultdict(int, pair=0, start="far", shape="cylinder", side=side,
                           outcome="success" if ok else "time_limit", complete=int(ok))
    rows=[row("left", True), row("right", True), row("left", False)]
    partial = summarize(rows)
    assert (partial["strict_layout_fraction"], partial["both_targets_fraction"],
            partial["strict_delivery_fraction"]) == (0., 1., 2/3)
    rows[-1] = row("left", True)
    assert summarize(rows)["strict_layout_fraction"] == 1.
    try:
        summarize(rows[:2])
    except ValueError:
        pass
    else:
        raise AssertionError("missing third member earned credit")
    print("all six signal orders exercised; failed repeat and missing member receive no full-layout credit")


if __name__ == "__main__":
    main()
