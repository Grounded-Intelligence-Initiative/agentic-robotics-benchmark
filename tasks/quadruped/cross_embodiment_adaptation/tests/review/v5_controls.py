"""Author-only matched controls and raw privileged feasibility check."""
import argparse
import base64
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent/"runtime"), str(HERE.parent/"setup_payload"),
                str(HERE.parents[1]/"solution"/"oracle")]
from mission import World
from survey import scan, waypoints
from walk import Walk


def trial(seed, arm, out, site):
    world = World(seed)
    try:
        if arm == "privileged":
            world.act([.20, 0.])
            world.act([.20, 0.])
            world.act([.20, 0.])
            world.act([.20, 0.])
            side, goal = world.side, world.goal.copy()
        else:
            if arm == "scout" and site in (0, 1):
                (out/f"site{site}-launch.png").write_bytes(base64.b64decode(world.observe()["images"][0]))
            mapping = scan(world, keep=arm == "scout")
            if mapping is not None:
                side, goal = mapping.route()
                if site in (0, 1):
                    (out/f"site{site}-survey-end.png").write_bytes(
                        base64.b64decode(world.observe()["images"][0]))
            else:
                side, goal = int(arm), np.array([3.16, 0.])
        obs = world.advance()
        walk = Walk(waypoints(side, goal))
        samples = []
        while not obs["done"]:
            xy = world.go1.xy.copy() if arm == "privileged" else np.asarray(obs["tracking_xy_m"])
            yaw = world.go1.heading if arm == "privileged" else obs["heading_rad"]
            if world.go1.actions % 20 == 0:
                samples.append([world.go1.ticks, *xy.tolist(), yaw])
            obs = world.act(walk.action(xy, yaw, obs["ticks_remaining"]))
        if site in (0, 1):
            samples.append([world.go1.ticks, *world.go1.xy.tolist(), world.go1.heading])
            (out/f"site{site}-{arm}-path.json").write_text(json.dumps(samples))
        return {"site": site, "arm": arm, "planned_side": side, "open_side": world.side,
                "physical_corridor": world.go1.corridor,
                "outcome": world.go1.outcome, "end_xy_m": world.go1.xy.tolist(),
                "end_heading_rad": world.go1.heading,
                "lift_and_contact_feet": sum(len(v) == 2 for v in world.go1.foot_modes),
                **world.metrics()}
    finally:
        world.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("out", type=Path)
    parser.add_argument("--sites", type=int, nargs="+", default=list(range(8)))
    parser.add_argument("--arms", nargs="+", choices=("privileged", "scout", "-1", "1"),
                        default=("privileged", "scout", "-1", "1"))
    args = parser.parse_args()
    args.out.mkdir()
    seeds = json.loads((HERE.parent/"hidden/sites.json").read_text())["seeds"]
    with (args.out/"results.jsonl").open("w") as stream:
        for site in args.sites:
            for arm in args.arms:
                receipt = trial(seeds[site], arm, args.out, site)
                print(json.dumps(receipt), flush=True)
                print(json.dumps(receipt), file=stream, flush=True)


if __name__ == "__main__":
    main()
