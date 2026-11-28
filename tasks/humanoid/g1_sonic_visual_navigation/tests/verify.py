"""Grade actual shutter passage, zero cabinet contact and all three stops in native G1 physics."""

import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading

STAGE = Path(__file__).resolve().parent
sys.path.insert(0, str(STAGE))
sys.path.insert(1, "/opt/sonic")

from robotics_grader import (
    SolverAborted,
    SolverMissing,
    VerifierDeadline,
    clear_agent_account,
    deprivileged,
    ratio_rewards,
    read_anchor,
    require,
    seed_rng,
    stage_config,
    write_verdict,
    zero_verdict,
)
from navigation_io import Course, NavigationEpisode
from solver_host import check_numerics, offline_command, participant_episodes, solver_environment
from sonic_loco import LocomotionSession
from sonic_loco.obstacles import make_layout


PHASES = ("approach", "pivot", "departure")
METRIC = "strict_course_fraction"
ASSETS = Path("/opt/sonic/assets")


def shuffled(rng, entries):
    pool = list(entries)
    result = []
    while pool:
        result.append(pool.pop(rng.choice(range(len(pool)))))
    return result


def stratified(rng, bounds, count):
    lo, hi = bounds
    return shuffled(rng, [lo + (hi - lo) * (i + rng.random()) / count for i in range(count)])


def schedule(rng, cfg):
    require(cfg["episodes"] == 16, "rack course roster expects sixteen courses")
    jobs = []
    diagonal = rng.seed(bits=1)
    for idx, sign in enumerate((-1, 1)):
        variants = [("both", "none"), ("both", "none"), ("both", "left"), ("both", "right")]
        for j, side in enumerate(("left", "right")):
            extra_tail = "left" if (idx + j + diagonal) % 2 == 0 else "right"
            variants.extend(((side, "none"), (side, extra_tail)))
        first = stratified(rng, cfg["first_distance_m"], len(variants))
        angle = stratified(rng, cfg["turn_magnitude_deg"], len(variants))
        last = stratified(rng, cfg["last_distance_m"], len(variants))
        # Both outgoing branches have the same distribution under either unrevealed turn.
        rack_families = ["low-left"] * 3 + ["low-right"] * 3 + ["high-left", "high-right"]
        left_arm = shuffled(rng, rack_families)
        right_arm = shuffled(rng, rack_families)
        for i, (service, tail) in enumerate(shuffled(rng, variants)):
            course = Course(first[i], sign * angle[i], last[i])
            jobs.append(
                (
                    course,
                    rng.seed(bits=64),
                    rng.seed(bits=64),
                    rng.seed(bits=64),
                    service,
                    tail,
                    rng.seed(bits=64),
                    left_arm[i],
                    right_arm[i],
                )
            )
    return shuffled(rng, jobs)


def crossing(event, plane_x, lo, hi, previous_time, previous_tick, stop_tick):
    """Check the interpolation coordinates, clock interval, and positive nearest-edge clearance."""
    x, y = event["forward_m"], event["left_m"]
    when, tick = event["native_time_s"], event["tick"]
    clearance = min(y - lo, hi - y)
    return (
        all(math.isfinite(v) for v in (x, y, lo, hi, when, clearance, event["gap_edge_clearance_m"]))
        and math.isclose(x, plane_x, abs_tol=1e-9)
        and previous_time < when
        and previous_tick <= tick <= stop_tick
        and (tick - 1) * 0.02 - 1e-9 <= when <= tick * 0.02 + 1e-9
        and clearance > 0
        and math.isclose(event["gap_edge_clearance_m"], clearance, abs_tol=1e-9)
    )


def strict_rack_passage(result, pivot, departure):
    """Rebuild the front and back plane crossings, full-body exit and final head-height test."""
    rack = result.get("rack_passage")
    if not rack or rack["missed"] or rack["boundary_violation"]:
        return False
    branch = "left" if pivot["command_turn_deg"] > 0 else "right"
    if rack["branch"] != branch:
        return False
    layout = result["rack_layouts"][branch]
    start_x, end_x = layout["front_m"], layout["back_m"]
    begin, end = rack["entry"], rack["exit"]
    clear = rack["whole_body_clear"]
    if not (begin and end and clear and end_x > start_x):
        return False
    if not math.isclose(end_x - start_x, layout["length_m"], abs_tol=1e-8):
        return False
    lo, hi = layout["gap_min_m"], layout["gap_max_m"]
    if not math.isclose(hi - lo, layout["width_m"], abs_tol=1e-8):
        return False
    start_tick = pivot["exit_tick"]
    stop_tick = departure["scorer_terminal_tick"]
    if stop_tick is None:
        return False
    if not crossing(begin, start_x, lo, hi, start_tick * 0.02, start_tick, stop_tick):
        return False
    if not crossing(end, end_x, lo, hi, begin["native_time_s"], begin["tick"], stop_tick):
        return False
    elapsed, tick = clear["native_time_s"], clear["tick"]
    rear = clear["trailing_body_forward_m"]
    if not (
        math.isfinite(rear)
        and rear > end_x
        and math.isclose(clear["back_m"], end_x, abs_tol=1e-8)
        and end["native_time_s"] <= elapsed <= stop_tick * 0.02 + 1e-9
        and end["tick"] <= tick <= stop_tick
        and (tick - 1) * 0.02 - 1e-9 <= elapsed <= tick * 0.02 + 1e-9
    ):
        return False
    under = rack.get("head_under_shelf")
    if not under or not all(
        math.isfinite(under[k])
        for k in (
            "head_top_m",
            "underside_m",
            "head_clearance_m",
            "head_forward_min_m",
            "head_forward_max_m",
            "head_left_min_m",
            "head_left_max_m",
            "native_time_s",
            "tick",
        )
    ):
        return False
    roof_lo = layout["left_m"] - 0.69
    roof_hi = layout["left_m"] + 0.69
    under_t, under_tick = under["native_time_s"], under["tick"]
    head_gap = layout["underside_m"] - under["head_top_m"]
    if not (
        start_x <= under["head_forward_min_m"] < under["head_forward_max_m"] <= end_x
        and roof_lo <= under["head_left_min_m"] < under["head_left_max_m"] <= roof_hi
        and start_tick <= under_tick <= tick
        and start_tick * 0.02 <= under_t <= elapsed
        and (under_tick - 1) * 0.02 - 1e-9 <= under_t <= under_tick * 0.02 + 1e-9
        and math.isclose(under["underside_m"], layout["underside_m"], abs_tol=1e-8)
        and head_gap >= 0
        and math.isclose(under["head_clearance_m"], head_gap, abs_tol=1e-8)
    ):
        return False
    roof_distance = rack["min_roof_distance_below_0p30_m"]
    if roof_distance is not None and (not math.isfinite(roof_distance) or roof_distance <= 0):
        return False
    head = departure["grade"].get("min_head_height_during_final_dwell_m")
    return (
        result.get("required_final_head_m") == 1.25
        and head is not None
        and math.isfinite(head)
        and head >= result["required_final_head_m"]
    )


def strict_course(result):
    """Demand the actual moving and fixed plane crossings plus all three accepted native stops."""
    if not (result["success"] and result["reason"] == "success" and result["stages_completed"] == 3):
        return False
    if (
        result["obstacle_contact"]
        or result["obstacle_contact_substeps"]
        or result["course_boundary_violation"]
        or result["missed_passage"]
        or result["first_obstacle_contact_tick"] is not None
    ):
        return False
    stages = result["stages"]
    if tuple(s["stage"] for s in stages) != PHASES or not all(
        s["reason"] == "success"
        and s["grade"]["success"]
        and s["grade"]["max_stop_dwell_s"] >= 0.75 - 1e-8
        and s["scorer_terminal_tick"] is not None
        and s["revealed_at_tick"] < s["scorer_terminal_tick"] <= s["exit_tick"]
        for s in stages
    ):
        return False
    if (
        stages[1]["revealed_at_tick"] != stages[0]["exit_tick"]
        or stages[2]["revealed_at_tick"] != stages[1]["exit_tick"]
        or stages[0]["exit_tick"] - stages[0]["scorer_terminal_tick"] < 75
    ):
        return False
    stop_tick = stages[0]["exit_tick"]
    gates, events = result["gate_layout"], result["passages"]
    if not (
        1 <= len(gates) <= 2
        and len(gates) == len(events) == result["required_gates"] == result["passed_gates"]
    ):
        return False
    fork, event = gates[0], events[0]
    if fork["gate_kind"] != "timed_fork" or event["gate_index"] != 0 or event["side"] not in (-1, 1):
        return False
    apertures = {a["side"]: a for a in fork["apertures"]}
    if set(apertures) != {-1, 1}:
        return False
    aperture = apertures[event["side"]]
    lo, hi = aperture["gap_min_m"], aperture["gap_max_m"]
    shutter = event.get("shutter_crossing")
    if not shutter or shutter["side"] != event["side"]:
        return False
    travel = shutter["actual_slide_m"]
    if not math.isfinite(travel):
        return False
    width = hi - lo
    exposed = min(width, max(0.0, travel - (1.08 - width) / 2))
    clear_lo, clear_hi = (lo, lo + exposed) if event["side"] == 1 else (hi - exposed, hi)
    observed = (shutter["clear_lo_m"], shutter["clear_hi_m"], shutter["clear_width_m"])
    if not all(math.isclose(a, b, abs_tol=1e-8) for a, b in zip(observed, (clear_lo, clear_hi, exposed))):
        return False
    if not crossing(shutter, fork["forward_m"] - 0.14, clear_lo, clear_hi, -1.0, -1, stop_tick):
        return False
    if not crossing(event, fork["forward_m"], lo, hi, shutter["native_time_s"], shutter["tick"], stop_tick):
        return False
    if len(gates) == 2:
        tail, next_event = gates[1], events[1]
        if (
            tail["gate_kind"] != "fixed_offset"
            or next_event["gate_index"] != 1
            or next_event["side"] != tail["side"]
        ):
            return False
        if not crossing(
            next_event,
            tail["forward_m"],
            tail["gap_min_m"],
            tail["gap_max_m"],
            event["native_time_s"],
            event["tick"],
            stop_tick,
        ):
            return False
    return strict_rack_passage(result, stages[1], stages[2])


def run_episode(
    solver_factory,
    course,
    robot_seed,
    obstacle_seed,
    phase_seed,
    service,
    tail,
    rack_seed,
    left_rack,
    right_rack,
    cfg,
    assets,
):
    layout = make_layout(
        obstacle_seed,
        course.first_m,
        service=service,
        tail=tail,
        phase_seed=phase_seed,
        rack_seed=rack_seed,
        left_rack=left_rack,
        right_rack=right_rack,
    )
    expected_tail = None if tail == "none" else (1 if tail == "left" else -1)
    measured_tail = None if layout.tail is None else layout.tail.side
    require(
        layout.service == service and measured_tail == expected_tail,
        "native scene did not instantiate the assigned stratum",
    )
    with LocomotionSession(assets, robot_seed, horizon=cfg["horizon"]) as session:
        episode = NavigationEpisode(session, course, obstacle_seed=obstacle_seed, layout=layout)
        try:
            with solver_factory() as solver:
                solver.reset()
                while not episode.done:
                    live = episode.observe()
                    action = solver.act(live["tick"], live)
                    episode.advance(action, live["observation_id"])
            result = episode.summary()
        except SolverAborted:
            result = dict(episode.summary(), success=False, reason="solver_abort")
    return dict(
        result,
        strict_success=strict_course(result),
        robot_seed=robot_seed,
        obstacle_seed=obstacle_seed,
        phase_seed=phase_seed,
        rack_seed=rack_seed,
        left_rack_family=left_rack,
        right_rack_family=right_rack,
        selected_rack_family=left_rack if course.turn_deg > 0 else right_rack,
        initial_phases_s=list(layout.phases),
        service_family=service,
        tail_family=tail,
        direction="left" if course.turn_deg > 0 else "right",
        requested_first_m=course.first_m,
        requested_turn_deg=course.turn_deg,
        requested_last_m=course.last_m,
    )


def summarize(rows, cfg):
    def group(items):
        def total(name):
            return sum(r[name] for r in items)

        n = len(items)
        wins = [r for r in items if r["strict_success"]]
        stopped = [int(r["stages_completed"]) for r in items]
        eligible = total("eligible_code_pairs")
        samples = total("joint_accel_samples")
        out = dict(
            episodes=n,
            successes=len(wins),
            episodes_aborted=sum(r["reason"] == "solver_abort" for r in items),
            applied_native_rows=total("applied_rows"),
            fsq_scalar_codes=total("scalar_codes"),
            changed_fsq_coordinates=total("code_changes"),
            eligible_adjacent_scalar_pairs=eligible,
            coordinate_change_fraction=total("code_changes") / eligible if eligible else 0.0,
            joint_accel_samples=samples,
            joint_accel_rms_rad_s2=math.sqrt(total("joint_accel_square_sum") / samples) if samples else 0.0,
            actual_elapsed_sim_s=total("simulated_s"),
            mean_actual_elapsed_sim_s=total("simulated_s") / n if n else 0.0,
            mean_completed_sim_s=sum(r["simulated_s"] for r in wins) / len(wins) if wins else 0.0,
            mean_completion_sim_s_failure_at_horizon=(
                sum(r["simulated_s"] for r in wins) + (n - len(wins)) * cfg["horizon"] * 0.02
            )
            / n
            if n
            else 0.0,
        )
        out.update(
            required_gates=total("required_gates"),
            passed_gates=total("passed_gates"),
            full_passage_courses=sum(r["passed_gates"] == r["required_gates"] for r in items),
            obstacle_contact_courses=sum(bool(r["obstacle_contact"]) for r in items),
            obstacle_contact_substeps=total("obstacle_contact_substeps"),
            boundary_violations=sum(bool(r["course_boundary_violation"]) for r in items),
        )
        clearances = [e["gap_edge_clearance_m"] for r in items for e in r["passages"]]
        if clearances:
            out["min_passage_pelvis_edge_clearance_m"] = min(clearances)
        for i, name in enumerate(PHASES):
            out[name + "_stops"] = sum(k > i for k in stopped)
            stage_records = [s for r in items for s in r["stages"] if s["stage"] == name]
            out[name + "_revealed"] = len(stage_records)
            if stage_records:
                count = len(stage_records)
                out[name + "_actual_rows"] = sum(
                    s["exit_tick"] - s["revealed_at_tick"] for s in stage_records
                )
                for key in (
                    "forward_m",
                    "lateral_m",
                    "signed_turn_rad",
                    "max_abs_lateral_m",
                    "max_pivot_drift_m",
                    "final_xy_error_m",
                    "max_stop_dwell_s",
                ):
                    out[name + "_mean_" + key] = sum(s["grade"][key] for s in stage_records) / count
                out[name + "_mean_absolute_final_heading_error_rad"] = (
                    sum(abs(s["grade"]["final_heading_error_rad"]) for s in stage_records) / count
                )
                out[name + "_mean_requested_distance_m"] = (
                    sum(s["command_distance_m"] for s in stage_records) / count
                )
                out[name + "_mean_requested_turn_deg"] = (
                    sum(s["command_turn_deg"] for s in stage_records) / count
                )
                for foot in ("left", "right"):
                    for transition in ("unloads", "reloads"):
                        key = foot + "_" + transition
                        out[name + "_" + key] = sum(s["grade"][key] for s in stage_records)
        return out

    metrics = group(rows)
    for sign in ("left", "right"):
        subset = [r for r in rows if r["direction"] == sign]
        metrics.update({sign + "_" + k: v for k, v in group(subset).items()})
    for r in rows:
        name = "terminal_" + str(r["reason"])
        metrics[name] = metrics.get(name, 0) + 1
    for family in ("both", "left", "right"):
        members = [r for r in rows if r["service_family"] == family]
        metrics[family + "_service_courses"] = len(members)
        metrics[family + "_service_successes"] = sum(r["strict_success"] for r in members)
    for kind in ("none", "left", "right"):
        members = [r for r in rows if r["tail_family"] == kind]
        metrics[kind + "_tail_courses"] = len(members)
        metrics[kind + "_tail_successes"] = sum(r["strict_success"] for r in members)
    forks = [event for r in rows for event in r["passages"] if event["gate_index"] == 0]
    if forks:
        widths = [e["shutter_crossing"]["clear_width_m"] for e in forks]
        metrics["minimum_crossed_shutter_width_m"] = min(widths)
        metrics["mean_crossed_shutter_width_m"] = sum(widths) / len(widths)
        metrics["minimum_crossed_shutter_pelvis_edge_clearance_m"] = min(
            e["shutter_crossing"]["gap_edge_clearance_m"] for e in forks
        )
    metrics["fork_left_passages"] = sum(e["side"] == 1 for e in forks)
    metrics["fork_right_passages"] = sum(e["side"] == -1 for e in forks)
    metrics["fork_staging_slow_s"] = sum(r["fork_staging_slow_s"] for r in rows)
    contacts = [r["first_obstacle_contact"] for r in rows if r["first_obstacle_contact"] is not None]
    metrics["first_contact_moving_shutter_courses"] = sum(
        c["obstacle_geom"] in ("shutter_left", "shutter_right") for c in contacts
    )
    metrics["first_contact_fixed_obstacle_courses"] = (
        len(contacts) - metrics["first_contact_moving_shutter_courses"]
    )
    for height in ("low", "high"):
        members = [r for r in rows if r["selected_rack_family"].startswith(height + "-")]
        metrics[height + "_rack_courses"] = len(members)
        metrics[height + "_rack_successes"] = sum(r["strict_success"] for r in members)
        metrics[height + "_whole_body_exits"] = sum(
            bool(r.get("rack_passage") and r["rack_passage"]["whole_body_clear"]) for r in members
        )
    for side in ("left", "right"):
        members = [r for r in rows if r["selected_rack_family"].endswith("-" + side)]
        metrics[side + "_rack_offset_courses"] = len(members)
        metrics[side + "_rack_offset_successes"] = sum(r["strict_success"] for r in members)
    racks = [r["rack_passage"] for r in rows if r.get("rack_passage")]
    metrics["rack_reached_departure"] = len(racks)
    metrics["rack_front_crossings"] = sum(bool(r["entry"]) for r in racks)
    metrics["rack_head_under_shelf"] = sum(bool(r.get("head_under_shelf")) for r in racks)
    metrics["rack_back_crossings"] = sum(bool(r["exit"]) for r in racks)
    metrics["rack_whole_body_exits"] = sum(bool(r["whole_body_clear"]) for r in racks)
    metrics["first_contact_departure_rack_courses"] = sum(
        c["obstacle_geom"].startswith("rack_") for c in contacts
    )
    heads = [
        s["grade"]["min_head_height_during_final_dwell_m"]
        for r in rows
        for s in r["stages"]
        if s["stage"] == "departure" and s["grade"].get("min_head_height_during_final_dwell_m") is not None
    ]
    if heads:
        metrics["minimum_accepted_green_head_apex_m"] = min(heads)
    roof_margins = [
        r["min_roof_distance_below_0p30_m"] for r in racks if r["min_roof_distance_below_0p30_m"] is not None
    ]
    if roof_margins:
        metrics["minimum_measured_roof_separation_m"] = min(roof_margins)
    for index, r in enumerate(rows, 1):
        prefix = f"trial_{index:02d}_"
        fields = dict(
            success=int(r["strict_success"]),
            native_success=int(r["success"]),
            turn_left=int(r["direction"] == "left"),
            left_full_service=int(r["service_family"] in ("left", "both")),
            right_full_service=int(r["service_family"] in ("right", "both")),
            tail_side={"none": 0, "left": 1, "right": -1}[r["tail_family"]],
            right_initial_phase_s=r["initial_phases_s"][0],
            left_initial_phase_s=r["initial_phases_s"][1],
            required_gates=r["required_gates"],
            passed_gates=r["passed_gates"],
            obstacle_contact=int(r["obstacle_contact"]),
            obstacle_contact_substeps=r["obstacle_contact_substeps"],
            course_boundary_violation=int(r["course_boundary_violation"]),
            missed_passage=int(r["missed_passage"]),
            stages_completed=r["stages_completed"],
            applied_rows=r["applied_rows"],
            simulated_s=r["simulated_s"],
            code_changes=r["code_changes"],
            joint_accel_samples=r["joint_accel_samples"],
            joint_accel_square_sum=r["joint_accel_square_sum"],
            requested_first_m=r["requested_first_m"],
            requested_turn_deg=r["requested_turn_deg"],
            requested_last_m=r["requested_last_m"],
            fork_staging_slow_s=r["fork_staging_slow_s"],
        )
        arm = "left" if r["direction"] == "left" else "right"
        layout = r["rack_layouts"][arm]
        fields.update(
            rack_low=int(layout["height_family"] == "low"),
            rack_offset_left=int(layout["left_m"] > 0),
            rack_front_m=layout["front_m"],
            rack_back_m=layout["back_m"],
            rack_length_m=layout["length_m"],
            rack_width_m=layout["width_m"],
            rack_gap_min_m=layout["gap_min_m"],
            rack_gap_max_m=layout["gap_max_m"],
            rack_underside_m=layout["underside_m"],
            rack_center_left_m=layout["left_m"],
            final_station_distance_m=r["final_station_distance_m"],
            required_final_head_m=r["required_final_head_m"],
        )
        rack = r.get("rack_passage")
        if rack:
            fields.update(
                rack_missed=int(rack["missed"]),
                rack_boundary_violation=int(rack["boundary_violation"]),
                selected_outgoing_branch_left=int(rack["branch"] == "left"),
            )
            for key in ("min_roof_distance_below_0p30_m", "max_head_height_between_planes_m"):
                if rack[key] is not None:
                    fields["rack_" + key] = rack[key]
            under = rack.get("head_under_shelf")
            if under:
                for key in (
                    "tick",
                    "native_time_s",
                    "head_top_m",
                    "underside_m",
                    "head_clearance_m",
                    "head_forward_min_m",
                    "head_forward_max_m",
                    "head_left_min_m",
                    "head_left_max_m",
                ):
                    fields["rack_head_under_shelf_" + key] = under[key]
            for name in ("entry", "exit", "whole_body_clear"):
                event = rack[name]
                if event:
                    for key in ("tick", "native_time_s"):
                        fields["rack_" + name + "_" + key] = event[key]
                    for key in (
                        ("trailing_body_forward_m", "back_m")
                        if name == "whole_body_clear"
                        else ("forward_m", "left_m", "gap_edge_clearance_m")
                    ):
                        fields["rack_" + name + "_" + key] = event[key]
        if r["fork_staging_entry_s"] is not None:
            fields["fork_staging_entry_s"] = r["fork_staging_entry_s"]
        if r["first_obstacle_contact_tick"] is not None:
            fields["first_obstacle_contact_tick"] = r["first_obstacle_contact_tick"]
            fields["first_obstacle_contact_time_s"] = r["first_obstacle_contact_time_s"]
            fields["first_contact_is_moving_shutter"] = int(
                r["first_obstacle_contact"]["obstacle_geom"] in ("shutter_left", "shutter_right")
            )
        for j, gate in enumerate(r["gate_layout"]):
            stem = f"gate_{j}_"
            fields[stem + "plane_forward_m"] = gate["forward_m"]
            openings = gate["apertures"] if j == 0 else [gate]
            for opening in openings:
                side = "left" if opening["side"] == 1 else "right"
                fields[stem + side + "_gap_min_m"] = opening["gap_min_m"]
                fields[stem + side + "_gap_max_m"] = opening["gap_max_m"]
        for event in r["passages"]:
            stem = f"gate_{event['gate_index']}_"
            for key in ("side", "tick", "native_time_s", "forward_m", "left_m", "gap_edge_clearance_m"):
                fields[stem + key] = event[key]
            if "shutter_crossing" in event:
                leaf = event["shutter_crossing"]
                for key in (
                    "side",
                    "tick",
                    "native_time_s",
                    "forward_m",
                    "left_m",
                    "actual_slide_m",
                    "clear_lo_m",
                    "clear_hi_m",
                    "clear_width_m",
                    "gap_edge_clearance_m",
                ):
                    fields["shutter_" + key] = leaf[key]
        for stage in r["stages"]:
            name = stage["stage"]
            if stage["reason"] is not None:
                fields[name + "_exit_tick"] = stage["exit_tick"]
            if name == "departure":
                head = stage["grade"].get("min_head_height_during_final_dwell_m")
                if head is not None:
                    fields["min_head_height_during_final_dwell_m"] = head
            terminal_tick = stage["scorer_terminal_tick"]
            if terminal_tick is not None:
                fields[name + "_scorer_terminal_tick"] = terminal_tick
                if stage["grade"]["success"]:
                    fields[name + "_settled_tick"] = terminal_tick
            if stage["reason"] == "success":
                fields[name + "_stop_tick"] = stage["exit_tick"]
        metrics.update({prefix + k: v for k, v in fields.items()})
    return metrics


def run_panel(solver_factory, cfg, rng, assets=ASSETS, on_complete=None):
    rows = []
    for job in schedule(rng, cfg):
        rows.append(run_episode(solver_factory, *job, cfg, assets))
        if on_complete is not None:
            on_complete(tuple(rows))
        print(f"[grader] completed {len(rows)}/{cfg['episodes']} courses", file=sys.stderr, flush=True)
    return summarize(rows, cfg), rows


def budget_verdict(completed, cfg, anchor, spec, elapsed):
    """Keep observed physical successes and charge the original assigned denominator."""
    n = cfg["episodes"]
    metrics = summarize(completed, cfg)
    wins = metrics["successes"]
    metrics.update(
        episodes=n,
        episodes_completed=len(completed),
        episodes_unfinished=n - len(completed),
        elapsed_s=elapsed,
        panel_deadline_exhausted=1,
        mean_completion_sim_s_failure_at_horizon=(
            sum(r["simulated_s"] for r in completed if r["strict_success"])
            + (n - wins) * cfg["horizon"] * 0.02
        )
        / n,
    )
    verdict = ratio_rewards(
        wins / n, anchor, cap=cfg["scoring_cap"], full_at=spec["full_at"], metric_name=METRIC, extra=metrics
    )
    verdict["diagnostic"] = (
        f"Panel wall-time budget exhausted after {len(completed)}/{n} finalized courses; "
        "motion costs cover finalized courses."
    )
    return verdict


def verify():
    cfg = stage_config()
    anchor, spec = read_anchor()
    completed = ()
    published = False
    publication_lock = threading.Lock()

    def checkpoint(rows):
        nonlocal completed
        completed = rows

    def publish(verdict, rows):
        nonlocal published
        with publication_lock:
            if published:
                return
            detail = Path("/logs/verifier/navigation-courses.json")
            manual_path = os.environ.get("ROBOTICS_GRADER_VERDICT_PATH")
            if manual_path and not os.environ.get("ALE_VERIFY_PYTHON"):
                verdict_path = Path(manual_path)
                detail = verdict_path.with_name(verdict_path.stem + "-courses.json")
            detail.write_text(json.dumps(rows, allow_nan=False) + "\n")
            write_verdict(verdict)
            published = True

    def timeout(elapsed):
        snapshot = completed
        publish(budget_verdict(snapshot, cfg, anchor, spec, elapsed), snapshot)

    with VerifierDeadline(cfg["verifier_deadline_s"], timeout):
        check_numerics()
        require(
            os.environ.get("ALE_VERIFY_PYTHON") or Path("/.dockerenv").is_file(),
            "isolated verifier sandbox required",
        )
        require(clear_agent_account(force=True)["cleared"], "could not isolate the solver account")
        child = deprivileged(
            offline_command("--self-test"),
            env=solver_environment(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        output, _ = child.communicate(timeout=15)
        require(child.returncode == 0 and output.strip() == "offline", "offline launcher self-test failed")
        try:
            with participant_episodes(cfg["step_timeout_s"]) as solver_factory:
                metrics, rows = run_panel(solver_factory, cfg, seed_rng(), on_complete=checkpoint)
        except SolverMissing as exc:
            publish(
                zero_verdict(
                    str(exc),
                    anchor is not None,
                    extra={
                        "episodes": cfg["episodes"],
                        "episodes_aborted": cfg["episodes"],
                        "episodes_completed": 0,
                        "episodes_unfinished": cfg["episodes"],
                        "panel_deadline_exhausted": 0,
                        METRIC: 0,
                        "successes": 0,
                    },
                ),
                completed,
            )
            return
        metrics.update(numerics_ok=1, offline_launcher_ok=1, agent_account_cleared=1)
        metrics.update(episodes_completed=len(rows), episodes_unfinished=0, panel_deadline_exhausted=0)
        publish(
            ratio_rewards(
                metrics["successes"] / metrics["episodes"],
                anchor,
                cap=cfg["scoring_cap"],
                full_at=spec["full_at"],
                metric_name=METRIC,
                extra=metrics,
            ),
            rows,
        )


if __name__ == "__main__":
    verify()
