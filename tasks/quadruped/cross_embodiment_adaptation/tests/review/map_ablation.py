"""How much of the scouted map the carrier needs (reviewer check on development seeds, no rendering).

Runs the oracle's carrier planner (exact state and dynamics through `truth`, so control is perfect)
with a partial map and counts arrivals. `full`: every obstacle (the oracle itself). `wall_only`:
the partition with its openings and the hall's boundary, no crates (a scout that swept only the
partition). `blind`: the boundary only (no scouting at all). A miss is a contact stop, a carrier
that finds no route, or the budget. Run inside the task image:

    /opt/venv/bin/python3 -B tests/review/map_ablation.py --seeds 7000-7199 > map_ablation.json
"""
import argparse
import json
import sys
from pathlib import Path

TASK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TASK/"tests"/"runtime"))
sys.path.insert(0, str(TASK/"solution"/"oracle"))
import numpy as np  # noqa: E402
import world as W  # noqa: E402
import oracle as O  # noqa: E402


def mission(seed, keep):
    w = W.World(seed, render=False)
    plus = next(b for b, d in enumerate(w.truth()["buttons"]) if d == [1, 0])
    minus = next(b for b, d in enumerate(w.truth()["buttons"]) if d == [-1, 0])
    for k in range(12):
        w.act([(plus if k % 2 == 0 else minus)/3])
    obs = w.advance()
    t = w.truth()
    rects = t["obstacles"]
    n_wall = len(w.site.partition)
    n_crate = len(w.site.crates)
    if keep == "wall_only":
        t = dict(t, obstacles=rects[:n_wall]+rects[n_wall+n_crate:])
    elif keep == "blind":
        t = dict(t, obstacles=rects[n_wall+n_crate:])
    planner = O.Planner(t)
    reason = "budget"
    while not obs["done"]:
        p, v = np.array(t["position"]), np.array(t["velocity"])
        buttons = planner.plan(p, v)
        if not buttons:
            reason = "no route"
            break
        for button in buttons:
            obs = w.act([button/3])
            t2 = w.truth()
            t = dict(t, position=t2["position"], velocity=t2["velocity"])
            if obs["done"]:
                break
    m = w.metrics()
    if m["success"]:
        reason = "arrived"
    elif m["contact_stop"]:
        reason = "contact stop"
    return {"seed": seed, "map": keep, "success": m["success"], "reason": reason,
            "carrier_actions": m["carrier_actions"], "crates": n_crate}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", default="7000-7099")
    args = parser.parse_args()
    lo, hi = (int(v) for v in args.seeds.split("-"))
    rows = [mission(seed, keep) for seed in range(lo, hi+1) for keep in ("full", "wall_only", "blind")]
    summary = {}
    for keep in ("full", "wall_only", "blind"):
        sel = [r for r in rows if r["map"] == keep]
        summary[keep] = {"missions": len(sel), "arrived": sum(r["success"] for r in sel),
                         "reasons": {k: sum(r["reason"] == k for r in sel) for k in sorted({r["reason"] for r in sel})}}
    print(json.dumps({"seeds": args.seeds, "summary": summary, "rows": rows}, indent=1))


if __name__ == "__main__":
    main()
