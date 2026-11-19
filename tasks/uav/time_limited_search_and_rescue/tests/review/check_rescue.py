"""Author controls for local visual odometry and interchangeable rescue fittings."""
import argparse
import base64
import json
from pathlib import Path
import sys

import numpy as np

TASK = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(TASK/"environment/setup/runtime"), str(TASK/"solution")]
from oracle_run import run
from rescue import LIMIT, World
from rescue_scene import FLANGE_RADII, layout_for


class Record:
    """The controller receives only the public envelope; truth is logged here."""
    def __init__(self, world, capture=False):
        self.world = world
        self.actions, self.frames, self.poses = [], {}, {}
        self.capture = capture

    def save(self, obs):
        step = obs["frame_id"]
        self.poses[step] = self.world.position.copy()
        if self.capture:
            self.frames[step] = obs["images"]
        return obs

    def observe(self):
        return self.save(self.world.observe())

    def act(self, action):
        self.actions.append(action.copy())
        return self.save(self.world.act(action))

    def export(self, step, path, camera):
        path.write_bytes(base64.b64decode(self.frames[step][camera]))


def outcome(world, seed, mode, **extra):
    result = {"seed": seed, "site": seed >> 1, "sign": world.bias_sign, "mode": mode,
              "failure": world.failure, **world.metrics(), **extra}
    print(json.dumps(result), flush=True)
    return result


def check_known_returns(trace):
    stop = next(i for i, e in enumerate(trace) if e["event"] == "sled_found")
    search = [tuple(e["cell"]) for e in trace[:stop] if e["event"] == "local_rgb"]
    crossed = {frozenset(pair) for pair in zip(search, search[1:])}
    returns = [(i, e) for i, e in enumerate(trace) if e["event"] == "known_return"]
    assert [tuple(e["goal"]) for _, e in returns] == [search[0], search[-1]]
    for i, e in returns:
        stations = [tuple(e["cell"]), *map(tuple, e["path"])]
        assert stations[-1] == tuple(e["goal"])
        assert all(frozenset(pair) in crossed for pair in zip(stations, stations[1:]))
        fixes = [tuple(f["cell"]) for f in trace[i+1:] if f["event"] == "station_fix"]
        assert fixes[:len(stations)-1] == stations[1:]


def episode(seed, mode, output=None):
    world = World("rescue", seed)
    try:
        record = Record(world, bool(output))
        pilot, error = None, None
        feedback = mode if mode in ("odometry", "prelift") else "rgb"
        try:
            pilot = run(record, retrace=mode == "retrace", feedback=feedback,
                        fixed_tool="narrow" if mode == "fixed-narrow" else None)
        except RuntimeError as exc:
            error = str(exc)
            tb = exc.__traceback__
            while tb and tb.tb_frame.f_code is not run.__code__:
                tb = tb.tb_next
            if tb:
                pilot = tb.tb_frame.f_locals["p"]
        fixes = [e for e in pilot.trace if e["event"] == "station_fix" and e["applied"]] if pilot else []
        stages = {e["event"]: e for e in pilot.trace if e["event"] in (
            "sled_found", "flange_survey", "depot_choice", "depot_departure",
            "sled_reacquired", "pickup_attempt", "sled_still_on_floor")} if pilot else {}
        wide = world.layout.flange == FLANGE_RADII[1]
        true_errors = []
        loaded_before = []
        for e in fixes:
            col, row = e["cell"]
            actual = record.poses[e["frame"]][:2]
            center = world.layout.xy((col+2, row))
            true_errors.append(float(np.linalg.norm(actual-center)))
            if e["loaded"]:
                old = record.poses[e["before_frame"]][:2]
                loaded_before.append(float(np.linalg.norm(old-center)))
        stopped_error = None
        if mode == "prelift" and pilot and pilot.path:
            col, row = pilot.path[-1]
            stopped_error = float(np.linalg.norm(world.position[:2]-world.layout.xy((col+2, row))))
        result = outcome(world, seed, mode, policy_error=error, fixes=len(fixes),
                         flange="wide" if wide else "narrow", stages=stages,
                         tool_profile=None if world.tool is None else
                         ("wide" if world.layout.depot[world.tool][0] > FLANGE_RADII[0] else "narrow"),
                         loaded_fixes=sum(e["loaded"] for e in fixes),
                         max_true_loaded_arrival_error_m=round(max(loaded_before, default=0.), 4),
                         max_true_fix_error_m=round(max(true_errors, default=0.), 4),
                         distance_from_last_requested_cell_m=stopped_error,
                         max_estimated_offset_m=round(max((e["before_m"] for e in fixes), default=0.), 4))
        if output:
            output.mkdir(parents=True, exist_ok=True)
            if pilot:
                (output/f"navigation-{mode}-{seed}.json").write_text(json.dumps(pilot.trace, indent=2))
            if error or (mode == "rgb" and not result["success"]):
                for camera in (0, 1):
                    record.export(world.steps, output/f"stop-{mode}-{seed}-{camera}.png", camera)
        if mode == "rgb":
            assert not error and result["success"] and result["rescue_progress"] == 1., result
            assert result["cargo_return"] and result["loaded_descent_z"] > 1.7
            assert result["loaded_fixes"] >= 4
            assert any(e["method"] == "patch_revisit" for e in fixes)
            assert result["steps"] < LIMIT
            assert result["tool_profile"] == stages["flange_survey"]["profile"] == result["flange"]
            assert stages["sled_found"]["frame"] < stages["flange_survey"]["frame"] < result["steps_to_tool"]
            assert result["steps_to_tool"] < stages["sled_reacquired"]["frame"] < result["steps_to_lift"]
            assert not result["fit_misses"]
        if mode == "fixed-narrow":
            assert not error and not result["failure"], result
            assert result["tool_profile"] == "narrow" and "flange_survey" not in stages, result
            assert stages["sled_found"]["cell"] == stages["sled_reacquired"]["cell"], result
            assert bool(result["success"]) != wide and bool(result["fit_misses"]) == wide, result
            assert bool(result["lifted"]) != wide, result
        if mode in ("rgb", "fixed-narrow"):
            check_known_returns(pilot.trace)
        if mode == "odometry":
            assert not fixes
        if mode == "prelift":
            assert not result["loaded_fixes"] and result["lifted"], result
        if mode == "retrace":
            assert not error and result["lifted"] and result["inspection_abort"] and result["pod_impact"], result
        if output:
            for event, camera in (("sled_found", 0), ("flange_survey", 1), ("depot_choice", 0)):
                if event in stages and mode == "rgb":
                    record.export(stages[event]["frame"], output/f"{event}-{seed}.png", camera)
            if mode == "rgb":
                (output/f"author-actions-{seed}.json").write_text(json.dumps(record.actions))
                largest = max((e for e in fixes if e["loaded"]), key=lambda e: e["before_m"])
                step = largest["frame"]
                record.export(largest["before_frame"], output/f"arrival-{seed}.png", 0)
                record.export(step, output/f"recentered-{seed}.png", 0)
                record.export(0, output/f"launch-{seed}.png", 0)
                landing = next(e for e in pilot.trace if e["event"] == "landing_probe")
                record.export(landing["frame"], output/f"landing-{seed}.png", 1)
        return record.actions, result
    finally:
        world.close()


def replay(seed, actions, jitter, same_sign=False):
    world = World("rescue", seed, render=False)
    rng = np.random.default_rng(742)
    try:
        for action in actions:
            tracked = action.copy()
            if jitter and any(abs(v) > .03 for v in tracked[:2]):
                tracked[:2] = np.clip(np.asarray(tracked[:2])+rng.normal(0., .025, 2), -1., 1.).tolist()
            if world.act(tracked)["done"]:
                break
        return outcome(world, seed, "tape-jitter" if jitter else "tape", same_sign=same_sign)
    finally:
        world.close()


def structural():
    for i in range(128):
        direct, mirror = layout_for(2*i), layout_for(2*i+1)
        assert direct.outlet == 4-mirror.outlet
        np.testing.assert_allclose(direct.columns, -np.asarray(mirror.columns)[::-1])
        assert direct.rows == mirror.rows
        assert 2.60-1e-8 <= min(np.diff(direct.rows)) <= max(np.diff(direct.rows)) <= 2.80+1e-8
    worlds = [World("rescue", s, render=False) for s in (0, 1)]
    try:
        a, b = worlds
        assert a.layout == b.layout and a.bias_sign == -b.bias_sign
        for world in worlds:
            for _ in range(2):
                world.act([0., .45, 0., 0., 0.])
            for _ in range(6):
                world.act([0.]*5)
        assert (a.position[0]-a.layout.launch[0])*(b.position[0]-b.layout.launch[0]) < 0
        for world in worlds:
            rest = world.position.copy()
            for _ in range(8):
                world.act([0.]*5)
            assert np.linalg.norm(world.position-rest) < .005
        print(json.dumps({"mode": "hold", "stationary_after_settle": True,
                          "matched_signs": True}), flush=True)
    finally:
        for world in worlds:
            world.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sites", type=int, nargs="+", default=[0, 19])
    parser.add_argument("--output", type=Path)
    parser.add_argument("--positives-only", action="store_true")
    parser.add_argument("--fixture-seeds", type=int, nargs="+")
    parser.add_argument("--require-mixed", action="store_true",
                        help="assert the selected fixture seeds cover both flange profiles")
    args = parser.parse_args()
    structural()
    if args.fixture_seeds:
        results = []
        for seed in args.fixture_seeds:
            actions, positive = episode(seed, "rgb", args.output)
            results.append(positive)
            if not args.positives_only:
                control_actions, control = episode(seed, "fixed-narrow", args.output)
                assert control["stages"]["sled_found"] == positive["stages"]["sled_found"]
                found = positive["stages"]["sled_found"]["frame"]
                assert actions[:found] == control_actions[:found], "matched search diverged"
        if args.require_mixed:
            assert {result["flange"] for result in results} == {"narrow", "wide"}
        return
    outcomes = []
    for site in args.sites:
        seed = 2*site
        direct = episode(seed, "rgb", args.output)
        opposite = episode(seed+1, "rgb", args.output)
        tape = direct[0]
        if args.positives_only:
            continue
        matched = []
        for s, positive in ((seed, direct), (seed+1, opposite)):
            for mode in ("odometry", "prelift"):
                actions, result = episode(s, mode, args.output)
                if mode == "prelift":
                    until = result["steps_to_lift"]
                    assert until == positive[1]["steps_to_lift"] and until > 0
                    assert actions[:until] == positive[0][:until], "matched pickup diverged"
                matched.append(result)
        outcomes.extend(matched)
        replay(seed, tape, jitter=True, same_sign=True)
        replay(seed+1, tape, jitter=False)
        replay(seed+1, tape, jitter=True)
    if not args.positives_only:
        assert any(m["mode"] == "prelift" and not m["success"] for m in outcomes), outcomes
        episode(2*args.sites[0], "retrace", args.output)
    world = World("rescue", 0, render=False)
    try:
        assert world.metrics()["rescue_progress"] == 0.
    finally:
        world.close()


if __name__ == "__main__":
    main()
