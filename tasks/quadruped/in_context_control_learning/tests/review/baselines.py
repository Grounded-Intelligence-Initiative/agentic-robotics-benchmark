"""Luck floor and a privileged baseline on development seeds (not the hidden set), without rendering.

    /opt/venv/bin/python3 tests/review/baselines.py <random|greedy> <out.jsonl> <seed> [<seed> ...]

random: a uniformly random button each press; advances as soon as a point is reached.
greedy: PRIVILEGED (reads the robot's and the goal's true positions): learns the four button displacements by pressing
        each once (again after the loading stop), then always presses the button whose displacement lands closest to
        the goal, with a few random presses when blocked; no map memory. It shows how much the walls and the map
        matter once perception is perfect.

Recorded in baselines.jsonl on seeds 3001-3100: random delivered 0.012 of the points on average (93 of 100 reached
none); greedy 0.148 (54 of 100 reached none, none reached more than 4). The oracle (image-only, with a map and a
planner) reached all six on 40 of 40 development seeds and on every hidden scene.
"""
import json
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


def run_greedy(w):
    rng = np.random.default_rng(w.seed)

    def calibrate():
        vectors = {}
        for b in range(4):
            p = w.position()
            w.act([b/3])
            vectors[b] = w.position()-p
        return vectors
    vectors, loaded = calibrate(), False
    while not w.done:
        if w.delivered > w.current:
            w.advance()
            if w.loaded and not loaded:
                loaded, vectors = True, calibrate()
            continue
        p, g = w.position(), w.goal()
        b = min(vectors, key=lambda k: np.linalg.norm(p+vectors[k]-g))
        w.act([b/3])
        if np.linalg.norm(w.position()-p) < .03:
            for _ in range(int(rng.integers(2, 8))):
                if w.done:
                    break
                w.act([int(rng.integers(4))/3])


def main():
    mode, out = sys.argv[1], open(sys.argv[2], "a")
    for seed in map(int, sys.argv[3:]):
        w = World(seed, render=False)
        run_random(w, np.random.default_rng(seed)) if mode == "random" else run_greedy(w)
        out.write(json.dumps({"seed": seed, "mode": mode, **w.metrics()})+"\n")
        out.flush()


if __name__ == "__main__":
    main()
