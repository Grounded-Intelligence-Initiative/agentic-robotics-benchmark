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
        dict(
            stage=name,
            reason="success",
            grade={"success": True, "max_stop_dwell_s": 0.75},
            revealed_at_tick=start,
            scorer_terminal_tick=terminal,
            exit_tick=end,
        )
        for name, start, terminal, end in zip(PHASES, (0, 1120, 1360), (1042, 1358, 1610), (1120, 1360, 1610))
    ]
    stages[1]["command_turn_deg"] = 90 * side
    stages[2]["grade"]["min_head_height_during_final_dwell_m"] = 1.26
    arm = "left" if side == 1 else "right"
    bay = dict(
        front_m=1.8,
        back_m=2.5,
        length_m=0.7,
        left_m=0.6,
        width_m=1.0,
        gap_min_m=0.1,
        gap_max_m=1.1,
        underside_m=1.22,
    )
    rack = dict(
        branch=arm,
        missed=False,
        boundary_violation=False,
        entry=passage(0, 1, 1.8, 0.7, 1420, 28.39, 0.4),
        exit=passage(0, 1, 2.5, 0.5, 1460, 29.19, 0.4),
        head_under_shelf=dict(
            tick=1430,
            native_time_s=28.59,
            head_top_m=1.18,
            underside_m=1.22,
            head_clearance_m=0.04,
            head_forward_min_m=1.89,
            head_forward_max_m=2.10,
            head_left_min_m=0.45,
            head_left_max_m=0.65,
        ),
        whole_body_clear=dict(tick=1481, native_time_s=29.61, trailing_body_forward_m=2.51, back_m=2.50),
        min_roof_distance_below_0p30_m=0.03,
        max_head_height_between_planes_m=1.19,
    )
    result = dict(
        rack_layouts={arm: bay},
        rack_passage=rack,
        required_final_head_m=1.25,
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


def check_rack():
    base = valid(1, True)
    for name in ("head_under_shelf", "entry", "exit", "whole_body_clear"):
        changed = deepcopy(base)
        changed["rack_passage"][name] = None
        assert not strict_course(changed)
    for name, value in (
        ("branch", "right"),
        ("missed", True),
        ("boundary_violation", True),
        ("min_roof_distance_below_0p30_m", -0.01),
    ):
        changed = deepcopy(base)
        changed["rack_passage"][name] = value
        assert not strict_course(changed)
    for name, key, value in (
        ("entry", "left_m", 1.4),
        ("exit", "native_time_s", 17.2),
        ("whole_body_clear", "trailing_body_forward_m", 2.48),
        ("whole_body_clear", "tick", 1420),
        ("head_under_shelf", "head_top_m", float("-inf")),
        ("head_under_shelf", "head_top_m", 1.23),
        ("head_under_shelf", "head_forward_min_m", 1.77),
        ("head_under_shelf", "head_left_max_m", 1.36),
        ("head_under_shelf", "head_clearance_m", 0.2),
    ):
        changed = deepcopy(base)
        changed["rack_passage"][name][key] = value
        assert not strict_course(changed)
    for name, key, value in ((0, "scorer_terminal_tick", 1100), (2, "revealed_at_tick", 1350)):
        changed = deepcopy(base)
        changed["stages"][name][key] = value
        assert not strict_course(changed)
    changed = deepcopy(base)
    changed["stages"][2]["grade"]["max_stop_dwell_s"] = 0.7
    assert not strict_course(changed)
    for head in (None, 1.249, float("nan")):
        changed = deepcopy(base)
        changed["stages"][2]["grade"]["min_head_height_during_final_dwell_m"] = head
        assert not strict_course(changed)


def check_schedule():
    cfg = json.loads((Path(__file__).resolve().parents[1] / "grader_config.json").read_bytes())
    geometries = set()
    phases = set()
    for seed in range(64):
        jobs = schedule(seed_rng(seed), cfg)
        assert len(jobs) == 16
        assert Counter(j[4] for j in jobs) == {"both": 8, "left": 4, "right": 4}
        assert Counter(j[5] for j in jobs) == {"none": 8, "left": 4, "right": 4}
        for sign in (-1, 1):
            sub = [j for j in jobs if j[0].turn_deg * sign > 0]
            assert Counter(j[4] for j in sub) == {"both": 4, "left": 2, "right": 2}
            assert Counter(j[5] for j in sub) == {"none": 4, "left": 2, "right": 2}
            expected = {"low-left": 3, "low-right": 3, "high-left": 1, "high-right": 1}
            assert Counter(j[7] for j in sub) == expected
            assert Counter(j[8] for j in sub) == expected
        for course, robot, key, phase, service, tail, rack_seed, left_rack, right_rack in jobs:
            assert len({robot, key, phase, rack_seed}) == 4 and key not in geometries and phase not in phases
            geometries.add(key)
            phases.add(phase)
            layout = make_layout(
                key,
                course.first_m,
                tail=tail,
                service=service,
                phase_seed=phase,
                rack_seed=rack_seed,
                left_rack=left_rack,
                right_rack=right_rack,
            )
            for label, expected_profile in (("left", left_rack), ("right", right_rack)):
                actual = layout.racks.branch(label).record()
                height, offset = expected_profile.split("-")
                assert actual["height_family"] == height and (actual["left_m"] > 0) == (offset == "left")
            assert layout.service == service and len(layout.gates) == 1 + (tail != "none")
            assert all(0 <= q < 12.0 for q in layout.phases)
            assert tuple(layout.fork.corridor(-1)) != tuple(layout.fork.corridor(1))
            assert cfg["first_distance_m"][0] <= course.first_m <= cfg["first_distance_m"][1]
            assert cfg["turn_magnitude_deg"][0] <= abs(course.turn_deg) <= cfg["turn_magnitude_deg"][1]
            assert cfg["last_distance_m"][0] <= course.last_m <= cfg["last_distance_m"][1]


if __name__ == "__main__":
    check_strict()
    check_rack()
    check_schedule()
    print(
        "Timed fork, physical rack underpass, whole-body exit, final dwell and 64 sixteen-course draws passed."
    )
