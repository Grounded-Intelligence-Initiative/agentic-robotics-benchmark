"""Author-only checks; the RGB reference sees the same envelope as the agent."""
import argparse
import base64
import json
import math
from pathlib import Path
import sys

import mujoco

TASK = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(TASK/"environment/setup/runtime"), str(TASK/"solution")]
from oracle_run import DIRECTIONS, TRAVEL, Pilot, add, run
from rescue import LIMIT, World
from rescue_scene import distances, edge, floor_z, layout_for, xy


class Record:
    """No simulator fields are supplied to the reference through this adapter."""
    def __init__(self, world, capture=False):
        self.world = world
        self.actions, self.frames = [], {}
        self.capture = capture

    def save(self, obs):
        if self.capture:
            self.frames[obs["frame_id"]] = obs["images"]
        return obs

    def observe(self):
        return self.save(self.world.observe())

    def act(self, action):
        self.actions.append(action.copy())
        return self.save(self.world.act(action))

    def export(self, step, path, camera):
        path.write_bytes(base64.b64decode(self.frames[step][camera]))


def probe(seed):
    """Author-only pose fixture for semantic paint visibility; not a rollout."""
    w = World("rescue", seed)
    try:
        mismatches = []
        variants = [(x, y) for x in range(5) for y in range(5)]
        for x, y in variants:
            for heading in (0., math.pi/2, -math.pi/2, math.pi):
                z = floor_z((x, y))
                w.data.qpos[:4] = (*xy((x, y)), TRAVEL+z, heading)
                mujoco.mj_forward(w.model, w.data)
                w._observation = None
                p = Pilot(w)
                p.command[:] = w.data.qpos[:4]
                actual = p.scan()[1]
                for d in DIRECTIONS:
                    neighbor = add((x, y), d)
                    expects = None
                    if edge((x, y), neighbor) in w.layout.halls:
                        expects = "landing" if floor_z(neighbor) != z else "open"
                    if x == neighbor[0] and {y, neighbor[1]} == {0, 1}:
                        expects = "inspection" if x == w.layout.inlet else "cargo" if x == w.layout.outlet else None
                    if (expects is not None and actual[d] != expects
                            or expects is None and actual[d] in ("landing", "inspection", "cargo", "open")):
                        mismatches.append(((x, y), heading, d, expects, actual[d]))
        assert not mismatches, mismatches
        print(json.dumps({"mode": "local-probe", "poses": 100, "seed": seed}), flush=True)
    finally:
        w.close()


def case(seed, mode, output=None):
    w = World("rescue", seed)
    try:
        r = Record(w, capture=bool(output))
        try:
            p = run(r, retrace=mode == "retrace", locate_only=mode == "locate")
        except RuntimeError as exc:
            tb = exc.__traceback__
            while tb and tb.tb_frame.f_code is not run.__code__:
                tb = tb.tb_next
            if tb:
                print(json.dumps(tb.tb_frame.f_locals["p"].trace[-30:]), flush=True)
            print(json.dumps(w.metrics()), flush=True)
            raise
        meta = next((x for x in p.trace if x["event"] == "topology_summary"), {})
        looks = [x for x in p.trace if x["event"] == "shutter_probe" and x["confirmed"]]
        m = {"seed": seed, "mode": mode, "failure": w.failure, **w.metrics(),
             "loop_edges": len(meta.get("loop_edges", [])),
             "closed_edge_replans": len(meta.get("invalidated_edges", [])),
             "shutter_probes": len(looks),
             "parallax_probes": sum(x["event"] == "parallax_probe" for x in p.trace)}
        print(json.dumps(m), flush=True)
        if mode == "rgb":
            assert m["success"] == 1 and m["rescue_progress"] == 1., m
            assert m["mezzanine_z"] > 1.70 and m["loaded_descent_z"] > 1.7
            assert m["loaded_cargo_cells"] >= 3 and m["yaw_travel_deg"] > 180
            assert m["closed_edge_replans"] <= 1 and m["shutter_probes"] <= 1
            assert m["steps"] < LIMIT
        elif mode == "retrace":
            assert m["lifted"] and m["inspection_abort"] and m["pod_impact"] and not m["uav_impact"], m
        else:
            assert m["located"] and not m["grasped"] and not m["delivered"], m
        if output:
            output.mkdir(parents=True, exist_ok=True)
            (output/f"navigation-{mode}-{seed}.json").write_text(json.dumps(p.trace, indent=2))
            if mode == "rgb":
                r.export(m["steps"], output/f"delivery-{seed}.png", 0)
                landing = next(x for x in p.trace if x["event"] == "landing_probe" and x["floor"] == 0.)
                r.export(landing["frame"], output/f"landing-local-{seed}.png", 0)
                r.export(landing["frame"], output/f"landing-front-{seed}.png", 1)
                bay = next(x for x in reversed(p.trace)
                           if x["event"] == "local_rgb" and not x["loaded"])
                r.export(bay["frame"], output/f"bay-{seed}.png", 0)
            if looks:
                r.export(looks[0]["frame"], output/f"shutter-{mode}-{seed}.png", 1)
        return r.actions, m
    finally:
        w.close()


def replay(seed, actions, matched_inlet=False):
    w = World("rescue", seed, render=False)
    try:
        for a in actions:
            if w.act(a)["done"]:
                break
        print(json.dumps({"seed": seed, "mode": "fixed-replay", "failure": w.failure,
                          "matched_inlet": matched_inlet,
                          **w.metrics()}), flush=True)
        assert not w.metrics()["success"] and w.metrics()["uav_impact"]
        if matched_inlet:
            assert w.metrics()["entered_wing"], w.metrics()
    finally:
        w.close()


def structural():
    for i in range(128):
        a, b = (layout_for(2*i+j) for j in (0, 1))
        assert a.halls != layout_for(2*i+2).halls and a.outlet == 4-b.outlet
        assert all(r[1] >= 3 for r in a.rooms)
        assert frozenset(edge(*((4-x, y) for x, y in e)) for e in a.halls) == b.halls
        assert len(distances((a.outlet, 1), a.halls-{a.safety})) == 20


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rgb-seeds", nargs="+", type=int, default=[0, 1, 6, 19])
    parser.add_argument("--retrace-seeds", nargs="*", type=int, default=[0, 1])
    parser.add_argument("--output", type=Path)
    parser.add_argument("--probe", type=int)
    args = parser.parse_args()
    structural()
    if args.probe is not None:
        probe(args.probe)
        return
    baseline = None
    positive = []
    for seed in args.rgb_seeds:
        actions, m = case(seed, "rgb", args.output)
        if baseline is None:
            baseline = actions
        positive.append(m)
    assert sum(m["loop_edges"] for m in positive) > 0
    assert sum(m["closed_edge_replans"] for m in positive) > 0
    first_seed = args.rgb_seeds[0]
    mirror = first_seed ^ 1
    same_inlet = next(s for s in range(32) if s not in (first_seed, mirror)
                      and layout_for(s).inlet == layout_for(first_seed).inlet)
    replay(mirror, baseline)
    replay(same_inlet, baseline, matched_inlet=True)
    for seed in args.retrace_seeds:
        case(seed, "retrace", args.output)
    w = World("rescue", 0, render=False)
    try:
        w.observe()
        assert w.metrics()["rescue_progress"] == 0
        print(json.dumps({"mode": "untouched", **w.metrics()}), flush=True)
    finally:
        w.close()


if __name__ == "__main__":
    main()
