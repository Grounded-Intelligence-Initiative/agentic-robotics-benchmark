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
from rescue_scene import GATE_CEILING, HEIGHT_GATE_Y, LANES, PASSAGES


def check(seed, mode):
    world = World("rescue", seed, render=mode == "rgb")
    try:
        if mode == "untouched":
            world.observe()
        elif mode == "blind":
            # Give the controls perfect pod truth and the other correct route.
            run(world, lane=0, height=world.safe_height, target=world.payload[:2].copy())
        elif mode == "blind-height":
            run(world, lane=world.safe_lane, height=0, target=world.payload[:2].copy())
        elif mode == "rgb":
            run(world)
        else:
            run(world, lane=world.safe_lane, height=world.safe_height, target=world.payload[:2].copy(),
                locate_only=mode == "locate-only")
        metrics = world.metrics()
        print(json.dumps({"seed": seed, "mode": mode, "failure": world.failure, **metrics}),
              flush=True)
        if mode in ("privileged", "rgb"):
            assert metrics["rescue_progress"] == 1 and metrics["success"] == 1, metrics
            assert metrics["steps"] < LIMIT
            assert not world.held and np.linalg.norm(world.payload[:2]-[0., -5.25]) <= .28
            _, _, _, lo, hi = PASSAGES[world.safe_height]
            assert lo <= metrics["height_out_z"] <= hi
            assert lo <= metrics["height_back_z"] <= hi
            assert abs(metrics["height_out_z"]-metrics["lane_out_z"]) > .3
            assert abs(metrics["height_back_z"]-metrics["lane_back_z"]) > .3
        elif mode == "untouched":
            assert metrics["rescue_progress"] == 0 and metrics["steps"] == 0
        elif mode == "locate-only":
            assert metrics["rescue_progress"] == .40 and not metrics["lifted"]
        elif mode == "blind" and world.safe_lane != 0:
            assert world.failure == "heat" and metrics["rescue_progress"] == .10
            assert not world.held and world.act([0., 1., 0., 1.])["done"]
            assert world.metrics() == metrics
        elif mode == "blind-height":
            if world.safe_height == 0:
                assert metrics["success"] == 1
            else:
                assert world.failure == "height_heat" and metrics["rescue_progress"] == .20
                assert world.act([0., 0., 1., 0.])["done"] and world.metrics() == metrics
        return metrics
    finally:
        world.close()


def check_pair(seed, axis, output):
    assert seed % 4 == 0
    sibling = seed+(1 if axis == "lane" else 2)
    worlds = [World("rescue", s) for s in (seed, sibling)]
    try:
        a, b = worlds
        np.testing.assert_array_equal(a.data.qpos, b.data.qpos)
        assert (a.safe_lane != b.safe_lane) == (axis == "lane")
        assert (a.safe_height != b.safe_height) == (axis == "height")
        pilots = [Pilot(w) for w in worlds]
        for p, w in zip(pilots, worlds):
            p.go(0., -3.28, settle=2)
            assert forecast(p.obs) == w.safe_lane
            if axis == "height":
                p.go(1., -2.12, settle=2)
                assert forecast(p.obs, height=True) == w.safe_height
        board = [w.images()[0] for w in worlds]
        assert not np.array_equal(*board)
        # Compare both actual onboard views from exactly the same later pose.
        for p in pilots:
            if axis == "height":
                x = LANES[a.safe_lane]
                p.go(x, -1.12)
                p.go(x, 1.12)
                p.go(0., .94, settle=2)
            else:
                p.go(0., -1.12, settle=2)
        junction = [w.images() for w in worlds]
        for left, right in zip(*junction):
            np.testing.assert_array_equal(left, right)
        if output:
            output.mkdir(parents=True, exist_ok=True)
            for i, image in enumerate(board):
                Image.fromarray(image).save(output/f"{axis}-forecast-{i}.png")
            for i, image in enumerate(junction[0]):
                Image.fromarray(image).save(output/f"{axis}-junction-{i}.png")
        for p in pilots:
            if axis == "height":
                x, _, _, lo, hi = PASSAGES[a.safe_height]
                z = (lo+hi)/2
                p.go(x, 1.47, z, settle=1)
                p.go(x, 3.78, z)
            else:
                x = LANES[a.safe_lane]
                p.go(x, -1.12)
                p.go(x, 1.12)
        failure = "heat" if axis == "lane" else "height_heat"
        assert a.metrics()[f"{axis}_out"] == 1 and b.failure == failure
        print(json.dumps({"pair": [seed, sibling], "axis": axis, "junction_images_identical": True,
                          "same_action_different_heat_outcome": True}), flush=True)
    finally:
        for w in reversed(worlds):
            w.close()


def check_safety():
    for case in ("overflight", "height-overflight", "height-skirt", "premature-grip", "high-drop"):
        world = World("rescue", 0, render=False)
        try:
            p = Pilot(world)
            if case == "premature-grip":
                p.grip(1.)
                assert not world.held and world.metrics()["rescue_progress"] == 0.
                continue
            x = LANES[world.safe_lane]
            p.go(0., -3.28)
            p.go(1., -2.12)
            p.go(x, -1.12, z=1.75 if case == "overflight" else 1.16)
            if case == "overflight":
                p.go(x, 1.12, z=1.75)
                assert world.failure == "baffle" and not world.metrics()["lane_out"]
                continue
            p.go(x, 1.12)
            hx, _, _, lo, hi = PASSAGES[world.safe_height]
            hz = (lo+hi)/2
            if case in ("height-overflight", "height-skirt"):
                bypass_x = 3.39 if case == "height-skirt" else hx
                bypass_z = 2.55 if case == "height-overflight" else hz
                p.go(bypass_x, 1.47, bypass_z)
                p.go(bypass_x, 3.78, bypass_z)
                assert world.failure == "bulkhead" and not world.metrics()["height_out"]
                continue
            p.go(hx, 1.47, hz)
            p.go(hx, 3.78, hz)
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
    print(json.dumps({"guard_checks": ["overflight", "height-overflight", "height-skirt",
                                      "premature-grip", "high-drop"]}), flush=True)


def check_fixed_altitude(seed, z):
    w = World("rescue", seed, render=False)
    try:
        p = Pilot(w)
        p.go(0., -3.28, z)
        p.go(1., -2.12, z)
        x = LANES[w.safe_lane]
        hx = PASSAGES[w.safe_height][0]
        p.go(x, -1.12, z)
        p.go(x, 1.12, z)
        p.go(hx, 1.47, z)
        p.go(hx, 3.78, z)
        reached = w.position[1] > HEIGHT_GATE_Y
        assert reached == (z == .75 and w.safe_height == 0), w.metrics()
        print(json.dumps({"mode": "fixed-altitude", "seed": seed, "setpoint_z": z,
                          "passed_height_gate": bool(reached), "failure": w.failure,
                          **w.metrics()}), flush=True)
    finally:
        w.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rgb", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--rgb-seeds", type=int, nargs="+", default=list(range(12)))
    args = parser.parse_args()
    assert PASSAGES[0][4] < PASSAGES[1][3] and GATE_CEILING < PASSAGES[1][3]
    seeds = range(12)
    joints = set()
    for seed in seeds:
        result = check(seed, "privileged")
        joints.add((result["safe_lane"], result["safe_height"]))
        check(seed, "locate-only")
        check(seed, "untouched")
    assert joints == {(lane, height) for lane in range(1, 5) for height in (1, 2)}
    blind = [check(seed, "blind") for seed in seeds]
    assert 0 < sum(m["success"] for m in blind) < len(blind)/2
    blind_height = [check(seed, "blind-height") for seed in seeds]
    assert sum(m["success"] for m in blind_height) == len(blind_height)//2
    for seed in (0, 2):
        for z in (.75, 1.16, 2.2):
            check_fixed_altitude(seed, z)
    check_safety()
    if args.rgb:
        check_pair(0, "lane", args.output)
        check_pair(0, "height", args.output)
        for seed in args.rgb_seeds:
            check(seed, "rgb")


if __name__ == "__main__":
    main()
