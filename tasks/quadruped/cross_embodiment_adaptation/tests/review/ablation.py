"""What the carrier needs to know (reviewer check on development seeds, no rendering).

The oracle's carrier controller (exact dynamics, so control is perfect) is given less than the truth and its
arrivals are counted. A miss is a contact stop or the press budget.

  full            the truth: every obstacle, and its own pose after every press (the oracle itself);
  no_scouting     its own true pose and the pad's place, but no partition and no crates (nothing was scouted);
  partition_only  the partition and its openings, but no crates (a scout that looked at the partition only);
  dock_point      the whole map, but it takes its start to be the dock point (where it is set down "give or
                  take 0.6 m") and reckons from there with a perfect measure of its own motion;
  scout_heading   the whole map and its true start, but it takes its heading to be the scout's, the map's own
                  (no alignment of the two bodies' frames), with a perfect measure of its own motion;
  odometer_only   the whole map and its true start pose, then dead reckoning on the leg odometer (which
                  over-reads when a foot slips and does not sense the turn);
  no_turns        the whole map, its true start pose and a perfect measure of its displacement in its own
                  frame, but a heading taken as constant (the slips' turns are not noticed).

The last four show that the carrier must find itself in the scout's map, in position and in heading, and
keep doing so while it drives. Run inside the task image:

    /opt/venv/bin/python3 -B tests/review/ablation.py --seeds 7000-7199 > ablation.json
"""
import argparse
import json
import math
import sys
from pathlib import Path

TASK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TASK/"tests"/"runtime"))
sys.path.insert(0, str(TASK/"solution"/"oracle"))
import numpy as np  # noqa: E402
import world as W  # noqa: E402
import oracle as O  # noqa: E402

MODES = ("full", "no_scouting", "partition_only", "dock_point", "scout_heading", "odometer_only", "no_turns")


def rot(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


def mission(seed, mode):
    w = W.World(seed, render=False)
    t = w.truth()
    back = next(i for i in range(4) if t["buttons"][i] == [-t["buttons"][0][0], -t["buttons"][0][1]])
    for k in range(12):
        w.act([(0, back)[k % 2]/3])
    obs = w.advance()
    t = w.truth()
    rects, n_wall, n_crate = t["obstacles"], len(w.site.partition), len(w.site.crates)
    known = dict(t)
    if mode == "no_scouting":
        known["obstacles"] = rects[n_wall+n_crate:]
    elif mode == "partition_only":
        known["obstacles"] = rects[:n_wall]+rects[n_wall+n_crate:]
    controller = O.Controller(known)
    start, heading0 = np.array(t["position"]), t["yaw"]
    twist = 0.
    if mode == "dock_point":
        believed = np.array(t["dock"])
    elif mode == "scout_heading":
        believed, twist = start.copy(), t["scout_yaw"]-heading0
    else:
        believed = start.copy()
    position, odometer = start.copy(), np.array(t["odometer"])
    while not obs["done"]:
        heading = t["yaw"]
        if mode in ("full", "no_scouting", "partition_only"):
            p, v, yaw = t["position"], t["velocity"], heading
        elif mode in ("dock_point", "scout_heading"):
            p, v, yaw = believed, rot(twist)@np.array(t["velocity"]), heading+twist
        else:
            # everything the carrier believes is its true motion as seen in its own frame, laid out along
            # the heading it started with
            p, v, yaw = believed, rot(heading0-heading)@np.array(t["velocity"]), heading0
        obs = w.act([controller.choose(p, v, yaw)/3])
        t = w.truth()
        now = np.array(t["position"])
        if mode in ("dock_point", "scout_heading"):
            believed = believed+rot(twist)@(now-position)
        elif mode == "odometer_only":
            believed = believed+rot(heading0)@(np.array(t["odometer"])-odometer)
        elif mode == "no_turns":
            believed = believed+rot(heading0-heading)@(now-position)
        position, odometer = now, np.array(t["odometer"])
    m = w.metrics()
    reason = "arrived" if m["success"] else "contact stop" if m["contact_stop"] else "presses spent"
    return {"seed": seed, "mode": mode, "success": m["success"], "reason": reason,
            "carrier_actions": m["carrier_actions"], "crates": n_crate,
            "start_offset": round(float(np.hypot(*(start-np.array(t["dock"])))), 3)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", default="7000-7099")
    parser.add_argument("--modes", default=",".join(MODES))
    args = parser.parse_args()
    lo, hi = (int(v) for v in args.seeds.split("-"))
    modes = args.modes.split(",")
    rows = [mission(seed, mode) for seed in range(lo, hi+1) for mode in modes]
    summary = {}
    for mode in modes:
        sel = [r for r in rows if r["mode"] == mode]
        summary[mode] = {"missions": len(sel), "arrived": sum(r["success"] for r in sel),
                         "reasons": {k: sum(r["reason"] == k for r in sel) for k in sorted({r["reason"] for r in sel})}}
    print(json.dumps({"seeds": args.seeds, "summary": summary, "rows": rows}, indent=1))


if __name__ == "__main__":
    main()
