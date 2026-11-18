"""Author checks for the procedural wing; run separately from the live ALE agent."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image

TASK = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(TASK/"environment/setup/runtime"), str(TASK/"solution")]
from oracle_run import PLAN, START, Pilot, Plan, path, read_plan, run
from rescue import LIMIT, ROTOR_HIGH, ROTOR_LOW, World
from rescue_scene import PITCH, SLOT_LINTEL, SLOT_SILL, layout_for


def plan_for(layout):
    return Plan(tuple(sorted(layout.halls)), layout.inlet, layout.outlet, layout.rooms)


def logged(world, mode, seed, **extra):
    answer = {"mode": mode, "seed": seed, "inlet": world.layout.inlet,
              "outlet": world.layout.outlet, "failure": world.failure, **world.metrics(), **extra}
    print(json.dumps(answer), flush=True)
    return answer


def layouts_checks():
    for seed in range(64):
        a, b = layout_for(2*seed), layout_for(2*seed+1)
        assert a.outlet != b.outlet and a.inlet == 4-b.inlet
        reflected = {tuple(sorted((4-x, y) for x, y in edge)) for edge in a.halls}
        assert reflected == b.halls
        assert len(a.halls) == 22 and len(set(a.rooms)) == 3
        assert a.outlet != a.inlet and a.halls != layout_for(2*seed+2).halls
        connection, = [e for e in a.halls if sum(x == a.outlet for x, _ in e) == 1]
        assert connection[0][1] == connection[1][1] >= 3
        for r in a.rooms:
            assert r[0] != a.outlet
            assert path((a.inlet, 1), r, a.halls)
            assert path(r, (a.outlet, 1), a.halls)
    assert SLOT_LINTEL-SLOT_SILL > ROTOR_LOW[2]+ROTOR_HIGH[2]
    # Sled lower surface sits >.37 m under the UAV at the closest reachable grasp.
    assert SLOT_LINTEL-SLOT_SILL < .37+ROTOR_HIGH[2]


def route_budget_check():
    longest = (0, 0)
    longest_live = (0, 0)
    for seed in range(1024):
        layout = layout_for(seed)
        x, y, _ = layout.pod
        bay = (int(round(x/PITCH))+2, int(round(y/PITCH)))
        current, left, outbound = (layout.inlet, 1), set(layout.rooms), 0
        while left:
            route = min((path(current, goal, layout.halls) for goal in sorted(left)), key=len)
            outbound += len(route)-1
            current = route[-1]
            left.remove(current)
            egress = len(path(current, (layout.outlet, 1), layout.halls))-1
            longest = max(longest, (outbound+egress, seed))
            if current == bay:
                longest_live = max(longest_live, (outbound+egress, seed))
    # Each edge is six actions. 81 more upper-bound the plan, portal
    # approaches, up/down transitions, room pauses, grip and set-down.
    predicted = 6*longest[0]+81
    print(json.dumps({"mode": "route-budget", "layouts": 1024, "max_edges": longest[0],
                      "seed": longest[1], "planned_action_upper_bound": predicted,
                      "longest_actual_bay": longest_live,
                      "limit": LIMIT}), flush=True)
    assert LIMIT >= predicted+24
    return longest_live[1]


def check(seed, mode, output=None):
    world = World("rescue", seed, render=mode == "rgb")
    try:
        if mode == "untouched":
            world.observe()
        else:
            plan = plan_for(world.layout)
            if mode == "rgb":
                pilot = run(world)
            elif mode == "fixed":
                # Replay layout zero's geometry/route, even though this is another wing.
                first = layout_for(0)
                pilot = run(world, plan_for(first), target=np.asarray(first.pod[:2]))
            else:
                pilot = run(world, plan, target=world.payload[:2].copy(),
                            retrace=mode == "retrace", locate_only=mode == "locate-only",
                            visit_plan=mode != "unsurveyed")
            if output and mode == "rgb":
                output.mkdir(parents=True, exist_ok=True)
                (output/f"route-{seed}.json").write_text(json.dumps(pilot.flown, indent=2))
                for i, frame in enumerate(world.images()):
                    Image.fromarray(frame).save(output/f"delivery-{seed}-{i}.png")
        m = logged(world, mode, seed)
        if mode in ("privileged", "rgb", "unsurveyed"):
            assert m["success"] == 1 and m["rescue_progress"] == 1., m
            assert m["cargo_return"] and m["exit_z"] > m["entry_z"]+.50
            assert m["loaded_cargo_cells"] >= 3
            assert m["steps"] < LIMIT
            assert not world.held and np.linalg.norm(world.payload[:2]) <= .28
            if mode == "unsurveyed":
                assert not m["surveyed"], m
        elif mode == "untouched":
            assert m["rescue_progress"] == 0. and not m["steps"]
        elif mode == "locate-only":
            assert m["entered_wing"] and m["located"] and not m["grasped"] and not m["delivered"]
        elif mode == "retrace":
            assert m["lifted"] and not m["success"] and not m["cargo_return"]
            assert world.failure == "inspection" and world.impact_body == "pod", m
        return m
    finally:
        world.close()


def check_controls():
    for mode in ("closed-cargo", "over-slot", "level-through-sill"):
        world = World("rescue", 0, render=False)
        try:
            pilot = Pilot(world)
            col = world.layout.outlet if mode == "closed-cargo" else world.layout.inlet
            if mode == "level-through-sill":
                pilot.cell((col, 0), .71, settle=2)
                inlet = (col, 1)
                pilot.cell(inlet, .71)
                onward = next(b if a == inlet else a for a, b in sorted(world.layout.raised)
                              if inlet in (a, b))
                pilot.cell(onward, .71)
                reason = "raised"
            else:
                pilot.cell((col, 0))
                pilot.cell((col, 1))
                reason = "cargo" if mode == "closed-cargo" else "inspection"
            assert world.failure == reason and world.impact_body == "uav"
            logged(world, mode, 0)
        finally:
            world.close()


def check_plan(seed, output):
    w = World("rescue", seed)
    try:
        p = Pilot(w)
        p.go(*PLAN, START[2], settle=3)
        plan = read_plan(p.obs)
        assert set(plan.halls) == w.layout.halls
        assert (plan.inlet, plan.outlet) == (w.layout.inlet, w.layout.outlet)
        assert set(plan.rooms) == set(w.layout.rooms)
        if output:
            output.mkdir(parents=True, exist_ok=True)
            Image.fromarray(w.images()[0]).save(output/f"plan-{seed}.png")
            p.cell((plan.inlet, 0), .71, settle=2)
            Image.fromarray(w.images()[1]).save(output/f"inspection-{seed}.png")
        print(json.dumps({"mode": "plan-rgb", "seed": seed, "edges": len(plan.halls),
                          "matches_physical_layout": True}), flush=True)
    finally:
        w.close()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--rgb-seeds", nargs="+", type=int, default=[])
    p.add_argument("--output", type=Path)
    p.add_argument("--privileged-count", type=int, default=12)
    args = p.parse_args()
    layouts_checks()
    long_seed = route_budget_check()
    for seed in range(args.privileged_count):
        check(seed, "privileged")
    for seed in range(4):
        for mode in ("untouched", "locate-only", "retrace"):
            check(seed, mode)
    fixed = [check(i, "fixed") for i in range(6)]
    assert fixed[0]["success"] and sum(m["success"] for m in fixed) <= 2
    assert sum(m["uav_impact"] for m in fixed) >= 3
    unsurveyed = next(seed for seed in range(16)
                     if layout_for(seed).outlet == 4 and layout_for(seed).inlet >= 2)
    check(unsurveyed, "unsurveyed")
    check_controls()
    for seed in dict.fromkeys([*args.rgb_seeds, long_seed] if args.rgb_seeds else []):
        check_plan(seed, args.output)
        check(seed, "rgb", args.output)


if __name__ == "__main__":
    main()
