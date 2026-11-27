"""Check temporal sampling and fail-closed physical predicates with hand-verifiable geometry."""

from collections import Counter
from copy import deepcopy
from pathlib import Path
import json
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robotics_grader import seed_rng
from sonic_loco.obstacles import make_layout
from verify import PHASES, schedule, strict_course


def passage(i, side, x, y, tick, t, clearance):
    return dict(
        gate_index=i,
        side=side,
        forward_m=x,
        left_m=y,
        tick=tick,
        native_time_s=t,
        gap_edge_clearance_m=clearance,
    )


def valid(side, with_tail):
    fixed = (0.2, 1.2) if side == 1 else (-1.2, -0.2)
    # A 1.08 m leaf spans a 1.00 m aperture, overlapping 0.04 m at each closed end.
    # At +0.54 m measured slide only the inner 0.50 m is exposed.
    clear = (0.2, 0.7) if side == 1 else (-0.7, -0.2)
    event = passage(0, side, 2.5, 0.45 * side, 420, 8.39, 0.25)
    event["shutter_crossing"] = dict(
        passage(0, side, 2.36, 0.45 * side, 398, 7.95, 0.25),
        actual_slide_m=0.54,
        clear_lo_m=clear[0],
        clear_hi_m=clear[1],
        clear_width_m=0.5,
    )
    fork = dict(
        gate_kind="timed_fork",
        forward_m=2.5,
        apertures=[
            dict(side=s, gap_min_m=lo, gap_max_m=hi) for s, lo, hi in ((1, 0.2, 1.2), (-1, -1.2, -0.2))
        ],
    )
    stages = [
        dict(stage=name, reason="success", grade={"success": True}, exit_tick=end)
        for name, end in zip(PHASES, (1120, 1360, 1610))
    ]
    result = dict(
        success=True,
        reason="success",
        stages=stages,
        stages_completed=3,
        obstacle_contact=False,
        obstacle_contact_substeps=0,
        first_obstacle_contact_tick=None,
        course_boundary_violation=False,
        missed_passage=False,
        gate_layout=[fork],
        passages=[event],
        passed_gates=1,
        required_gates=1,
    )
    if with_tail:
        result["gate_layout"].append(
            dict(gate_kind="fixed_offset", side=-1, forward_m=4.0, gap_min_m=-1.0, gap_max_m=-0.25)
        )
        result["passages"].append(passage(1, -1, 4.0, -0.6, 840, 16.79, 0.35))
        result["required_gates"] = result["passed_gates"] = 2
    return result


def check_strict():
    for side in (-1, 1):
        for tail in (False, True):
            assert strict_course(valid(side, tail))
    base = valid(1, True)
    for key, value in (
        ("success", False),
        ("reason", "solver_abort"),
        ("obstacle_contact", True),
        ("obstacle_contact_substeps", 1),
        ("first_obstacle_contact_tick", 400),
        ("course_boundary_violation", True),
        ("missed_passage", True),
        ("passed_gates", 1),
        ("stages_completed", 2),
        ("passages", list(reversed(base["passages"]))),
    ):
        assert not strict_course(dict(base, **{key: value})), key
    for key, value in (
        ("side", -1),
        ("actual_slide_m", 0.03),
        ("actual_slide_m", 0.99),
        ("clear_width_m", 0.7),
        ("clear_lo_m", 0.05),
        ("clear_hi_m", 1.2),
        ("forward_m", 2.5),
        ("left_m", 1.05),
        ("gap_edge_clearance_m", 0.0),
        ("native_time_s", float("nan")),
        ("tick", 421),
    ):
        x = deepcopy(base)
        x["passages"][0]["shutter_crossing"][key] = value
        assert not strict_course(x), key
    missing = deepcopy(base)
    del missing["passages"][0]["shutter_crossing"]
    assert not strict_course(missing)
    for metric in (0, 64, 5_000_000):
        assert strict_course(dict(base, code_changes=metric))
        assert not strict_course(dict(missing, code_changes=metric))
    x = deepcopy(base)
    x["stages"][0]["grade"]["success"] = False
    assert not strict_course(x)
    x = deepcopy(base)
    x["passages"][1]["left_m"] = 0.0
    assert not strict_course(x)


def check_schedule():
    cfg = json.loads((Path(__file__).resolve().parents[1] / "grader_config.json").read_bytes())
    geometries = set()
    phases = set()
    for seed in range(64):
        jobs = schedule(seed_rng(seed), cfg)
        assert len(jobs) == 16
        assert Counter(s for _, _, _, _, s, _ in jobs) == {"both": 8, "left": 4, "right": 4}
        assert Counter(t for _, _, _, _, _, t in jobs) == {"none": 8, "left": 4, "right": 4}
        for sign in (-1, 1):
            sub = [j for j in jobs if j[0].turn_deg * sign > 0]
            assert Counter(j[4] for j in sub) == {"both": 4, "left": 2, "right": 2}
            assert Counter(j[5] for j in sub) == {"none": 4, "left": 2, "right": 2}
        for course, robot, key, phase, service, tail in jobs:
            assert len({robot, key, phase}) == 3 and key not in geometries and phase not in phases
            geometries.add(key)
            phases.add(phase)
            layout = make_layout(key, course.first_m, tail=tail, service=service, phase_seed=phase)
            assert layout.service == service and len(layout.gates) == 1 + (tail != "none")
            assert all(0 <= q < 12.0 for q in layout.phases)
            assert tuple(layout.fork.corridor(-1)) != tuple(layout.fork.corridor(1))
            assert cfg["first_distance_m"][0] <= course.first_m <= cfg["first_distance_m"][1]
            assert cfg["turn_magnitude_deg"][0] <= abs(course.turn_deg) <= cfg["turn_magnitude_deg"][1]
            assert cfg["last_distance_m"][0] <= course.last_m <= cfg["last_distance_m"][1]


if __name__ == "__main__":
    check_strict()
    check_schedule()
    print(
        "Temporal crossing, moving-boundary failures and 64 fresh sixteen-course sampling fixtures passed."
    )
