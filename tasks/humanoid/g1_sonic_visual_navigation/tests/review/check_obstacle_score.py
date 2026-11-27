"""Exercise the obstacle roster and strict native score without running a participant."""
from collections import Counter
from copy import deepcopy
from pathlib import Path
import json
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robotics_grader import seed_rng
from verify import PHASES, layout_family, schedule, strict_course


def valid_result():
    stages = [dict(stage=name, reason="success", grade={"success": True}, exit_tick=tick)
              for name, tick in zip(PHASES, (800, 1020, 1300))]
    events = [
        dict(gate_index=i, side=sign, tick=tick, native_time_s=(tick-.6)*.02,
             forward_m=distance, left_m=.45*sign, gap_edge_clearance_m=.18)
        for i, (sign, tick, distance) in enumerate(((1, 280, 2.1), (-1, 540, 4.2)))
    ]
    gates = [dict(forward_m=e["forward_m"], left_m=e["left_m"], side=e["side"], gap_width_m=.36) for e in events]
    return dict(gate_layout=gates, missed_passage=False, success=True, reason="success", stages_completed=3, stages=stages,
                obstacle_contact=False, obstacle_contact_substeps=0, first_obstacle_contact_tick=None,
                course_boundary_violation=False, required_gates=2, passed_gates=2, passages=events,
                simulated_s=26.)


def check_strict():
    base = valid_result()
    assert strict_course(base, (1, -1))
    variants = (
        ("obstacle_contact", True), ("obstacle_contact_substeps", 1),
        ("first_obstacle_contact_tick", 660), ("course_boundary_violation", True),
        ("missed_passage", True), ("required_gates", 1), ("passed_gates", 1), ("stages_completed", 2),
        ("reason", "solver_abort"), ("success", False), ("passages", list(reversed(base["passages"]))),
        ("passages", base["passages"][:1]), ("stages", base["stages"][:2]),
    )
    for field, value in variants:
        assert not strict_course(dict(base, **{field: value}), (1, -1)), field
    for field, value in (("side", 1), ("gate_index", 0), ("tick", 240), ("tick", 900),
                         ("native_time_s", 0.), ("native_time_s", float("nan")),
                         ("left_m", 1.7), ("forward_m", 3.0), ("native_time_s", 10.7),
                         ("gap_edge_clearance_m", .25), ("gap_edge_clearance_m", 0.), ("gap_edge_clearance_m", -1e-10),
                         ("gap_edge_clearance_m", float("nan"))):
        trial = deepcopy(base)
        trial["passages"][1][field] = value
        assert not strict_course(trial, (1, -1)), field
    missed_stop = deepcopy(base)
    missed_stop["stages"][0]["grade"]["success"] = False
    assert not strict_course(missed_stop, (1, -1))
    # Apply arbitrary coordinate changes to the diagnostic record. The same failed physical
    # trajectory must still earn zero, and the valid trajectory receives the same outcome.
    for changes in (0, 64, 500_000):
        assert strict_course(dict(base, code_changes=changes), (1, -1))
        assert not strict_course(dict(missed_stop, code_changes=changes), (1, -1))


def check_schedule():
    cfg = json.loads((Path(__file__).resolve().parents[1] / "grader_config.json").read_bytes())
    seen = set()
    for seed in range(64):
        jobs = schedule(seed_rng(seed), cfg)
        assert len(jobs) == 16 and len(set(j[1] for j in jobs)) == len(set(j[2] for j in jobs)) == 16
        assert Counter(layout_family(sides) for _, _, _, sides in jobs) == {"single": 4, "same": 6, "opposite": 6}
        assert Counter((course.turn_deg > 0, sides[0]) for course, _, _, sides in jobs) == {
            (False, -1): 4, (False, 1): 4, (True, -1): 4, (True, 1): 4}
        assert all(key not in seen for _, _, key, _ in jobs)
        seen.update(key for _, _, key, _ in jobs)
        for course, _, _, _ in jobs:
            assert cfg["first_distance_m"][0] <= course.first_m <= cfg["first_distance_m"][1]
            assert cfg["last_distance_m"][0] <= course.last_m <= cfg["last_distance_m"][1]
            assert cfg["turn_magnitude_deg"][0] <= abs(course.turn_deg) <= cfg["turn_magnitude_deg"][1]


if __name__ == "__main__":
    check_strict()
    check_schedule()
    print("Sixteen-course stratification and physical scoring passed on 64 seeded public roster draws.")
