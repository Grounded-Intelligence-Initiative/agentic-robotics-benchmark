"""Author-only geometry, oracle, and controls. Never staged into an agent run."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image

TASK = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(TASK/"environment/setup/runtime"), str(TASK/"solution")]
from oracle_run import Pilot, forecast, run
from rescue import LIMIT, World
from rescue_scene import LANES


def check(seed, mode):
    world = World("rescue", seed, render=mode == "rgb")
    try:
        if mode == "untouched":
            world.observe()
        elif mode == "blind":
            # Give the control perfect pod truth; take away only the heat cue.
            run(world, lane=0, target=world.payload[:2].copy())
        elif mode == "rgb":
            run(world)
        else:
            run(world, lane=world.safe_lane, target=world.payload[:2].copy(),
                locate_only=mode == "locate-only")
        metrics = world.metrics()
        print(json.dumps({"seed": seed, "mode": mode, "failure": world.failure, **metrics}),
              flush=True)
        if mode in ("privileged", "rgb"):
            assert metrics["rescue_progress"] == 1 and metrics["success"] == 1, metrics
            assert metrics["steps"] < LIMIT
            assert not world.held and np.linalg.norm(world.payload[:2]-[0., -5.25]) <= .28
        elif mode == "untouched":
            assert metrics["rescue_progress"] == 0 and metrics["steps"] == 0
        elif mode == "locate-only":
            assert metrics["rescue_progress"] == .40 and not metrics["lifted"]
        elif mode == "blind" and world.safe_lane != 0:
            assert world.failure == "heat" and metrics["rescue_progress"] == .10
            assert not world.held and world.act([0., 1., 0., 1.])["done"]
            assert world.metrics() == metrics
        return metrics
    finally:
        world.close()


def check_pair(seed, output):
    assert seed % 2 == 0
    worlds = [World("rescue", s) for s in (seed, seed+1)]
    try:
        a, b = worlds
        np.testing.assert_array_equal(a.data.qpos, b.data.qpos)
        assert a.safe_lane != b.safe_lane
        pilots = [Pilot(w) for w in worlds]
        for p, w in zip(pilots, worlds):
            p.go(0., -3.28, settle=2)
            assert forecast(p.obs) == w.safe_lane
        board = [w.images()[0] for w in worlds]
        assert not np.array_equal(*board)
        # Compare both actual onboard views from exactly the same later pose.
        for p in pilots:
            p.go(0., -1.12, settle=2)
        junction = [w.images() for w in worlds]
        for left, right in zip(*junction):
            np.testing.assert_array_equal(left, right)
        if output:
            output.mkdir(parents=True, exist_ok=True)
            for i, image in enumerate(board):
                Image.fromarray(image).save(output/f"forecast-{i}.png")
            for i, image in enumerate(junction[0]):
                Image.fromarray(image).save(output/f"junction-{i}.png")
        for p in pilots:
            p.go(LANES[a.safe_lane], -1.12)
            p.go(LANES[a.safe_lane], 1.12)
        assert a.metrics()["entered_b"] == 1 and b.failure == "heat"
        print(json.dumps({"pair": [seed, seed+1], "junction_images_identical": True,
                          "same_action_different_heat_outcome": True}), flush=True)
    finally:
        for w in reversed(worlds):
            w.close()


def check_safety():
    for case in ("overflight", "premature-grip", "high-drop"):
        world = World("rescue", 0, render=False)
        try:
            p = Pilot(world)
            if case == "premature-grip":
                p.grip(1.)
                assert not world.held and world.metrics()["rescue_progress"] == 0.
                continue
            x = LANES[world.safe_lane]
            p.go(0., -3.28)
            p.go(x, -1.12, z=1.75 if case == "overflight" else 1.16)
            if case == "overflight":
                p.go(x, 1.12, z=1.75)
                assert world.failure == "baffle" and not world.metrics()["entered_b"]
                continue
            p.go(x, 1.12)
            px, py = world.payload[:2].copy()
            p.go(px, py)
            p.go(px, py, .43, settle=2)
            p.grip(1.)
            p.go(px, py, settle=2)
            assert world.held and world.metrics()["lifted"]
            p.grip(-1.)
            assert world.failure == "unsafe_release" and not world.metrics()["delivered"]
        finally:
            world.close()
    print(json.dumps({"guard_checks": ["overflight", "premature-grip", "high-drop"]}), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rgb", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--rgb-seeds", type=int, nargs="+", default=list(range(12)))
    args = parser.parse_args()
    seeds = range(12)
    for seed in seeds:
        check(seed, "privileged")
        check(seed, "locate-only")
        check(seed, "untouched")
    blind = [check(seed, "blind") for seed in seeds]
    assert 0 < sum(m["success"] for m in blind) < len(blind)/2
    check_safety()
    if args.rgb:
        check_pair(0, args.output)
        for seed in args.rgb_seeds:
            check(seed, "rgb")


if __name__ == "__main__":
    main()
