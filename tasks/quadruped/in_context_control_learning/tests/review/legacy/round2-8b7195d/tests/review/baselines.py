"""Luck floor and a camera-less baseline on development seeds (not the hidden set), without rendering.

    /opt/venv/bin/python3 tests/review/baselines.py <random|blind> <out.jsonl> <seed> [<seed> ...]

random: a uniformly random button each press; advances as soon as a point is reached.
blind:  PRIVILEGED calibration (it is told which button is which gait and the exact step sizes, before and after
        the loading stop), then only what an agent has without the camera: the start pose, the compass and the
        delivery coordinates. It turns to the bearing of the active point and walks there by dead reckoning; when
        its reckoning says it has arrived and the point is not reached, it walks a widening square around the spot.
        It never sees a wall, so a wall stops it without its knowing. It shows what the camera is needed for.

Recorded in baselines.jsonl on development seeds 3001-3100.
"""
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np  # noqa: E402
from world import World  # noqa: E402


def run_random(w, rng):
    while not w.done:
        if w.delivered > w.current:
            w.advance()
            continue
        w.act([int(rng.integers(4))/3])


def run_blind(w):
    s = w.scene
    button = {gait: index for index, gait in enumerate(s.mapping)}
    x, y = s.start                                   # its own reckoning, never corrected

    def steps():
        load = s.load if w.loaded else 1.
        return {k: s.speeds[k]*.175*load for k in s.speeds}

    def walk_to(tx, ty):
        nonlocal x, y
        for _ in range(400):
            if w.done or w.delivered > w.current:
                return
            size = steps()
            distance = math.hypot(tx-x, ty-y)
            if distance < size["forward"]/2:
                return
            heading = math.radians(w.status()["heading_deg"])
            error = (math.atan2(ty-y, tx-x)-heading+math.pi) % (2*math.pi)-math.pi
            gait = "left" if error > 0 else "right"
            if abs(error) > size[gait]/2:
                w.act([button[gait]/3])
            else:
                w.act([button["forward"]/3])
                x, y = x+size["forward"]*math.cos(heading), y+size["forward"]*math.sin(heading)

    while not w.done:
        if w.delivered > w.current:
            w.advance()
            continue
        goal = w.status()["goal"]
        walk_to(goal["x"], goal["y"])
        ring = 1
        while not w.done and w.delivered <= w.current:
            for dx, dy in ((1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)):
                walk_to(goal["x"]+.4*ring*dx, goal["y"]+.4*ring*dy)
                if w.done or w.delivered > w.current:
                    break
            ring += 1


def main():
    mode, out = sys.argv[1], open(sys.argv[2], "a")
    for seed in map(int, sys.argv[3:]):
        w = World(seed, render=False)
        run_random(w, np.random.default_rng(seed)) if mode == "random" else run_blind(w)
        out.write(json.dumps({"seed": seed, "mode": mode, **w.metrics()})+"\n")
        out.flush()


if __name__ == "__main__":
    main()
