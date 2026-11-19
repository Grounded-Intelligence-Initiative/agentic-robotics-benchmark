"""Short author-only straight-lane physics check; not evidence for survey transfer."""
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent/"runtime"), str(HERE.parents[1]/"solution/oracle")]
from articulated import Go1
from mobility import Fitting
from walk import Walk
from walk import STANCE_M


def trial(kind, mode):
    fixtures = [] if kind == "flat" else [Fitting(kind, 1.3, -.56, .56, .44 if kind == "duct" else .62)]
    goal = np.array([2.5, 0.])
    dog = Go1(np.random.default_rng(1701), {}, goal, [], [], [0, 0, 0], fixtures)
    walk = Walk([goal], posture=mode)
    heights = []
    try:
        while dog.outcome == "running":
            obs = dog.observe()
            heights.append(dog.data.xpos[dog.trunk, 2])
            dog.act(walk.action(np.asarray(obs["odom_xy_m"]), obs["odom_yaw_rad"], obs["ticks_remaining"]))
        return dict(fitting=kind, mode=mode, outcome=dog.outcome, obstacle=dog.contact_wall,
                    part=dog.contact_part, max_x=dog.max_x, ticks=dog.ticks, deck_feet=len(dog.deck_feet),
                    gap=dog.min_duct_gap, zmin=min(heights), zmax=max(heights))
    finally:
        dog.close()


if __name__ == "__main__":
    if sys.argv[1:] == ["--compromise"]:
        for height in (.211, .217, .223, .229, .235):
            STANCE_M["intermediate"] = height
            for kind in ("duct", "deck"):
                print(json.dumps({"fixed_height_m": height, **trial(kind, "intermediate")}), flush=True)
    else:
        for kind in ("flat", "duct", "deck"):
            for mode in ("ordinary", "crouch"):
                print(json.dumps(trial(kind, mode)), flush=True)
