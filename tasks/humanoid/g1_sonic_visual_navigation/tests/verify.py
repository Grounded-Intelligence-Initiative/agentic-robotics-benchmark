"""Grade physically completed obstacle courses in the native G1 SONIC runtime."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys

STAGE = Path(__file__).resolve().parent
sys.path.insert(0, str(STAGE))
sys.path.insert(1, "/opt/sonic")

from robotics_grader import (
    SolverAborted, SolverMissing, VerifierDeadline, clear_agent_account,
    deprivileged, ratio_rewards, read_anchor, require, seed_rng,
    stage_config, write_verdict, zero_verdict,
)
from navigation_io import Course, NavigationEpisode
from solver_host import OfflineSolver, check_numerics, offline_command, solver_environment
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


def layout_family(sides):
    if len(sides) == 1:
        return "single"
    if len(sides) != 2:
        raise ValueError("the registered course has one or two gates")
    return "same" if sides[0] == sides[1] else "opposite"


def schedule(rng, cfg):
    require(cfg["episodes"] == 16, "obstacle roster expects sixteen courses")
    diagonal = rng.seed(bits=1)
    jobs = []
    used_layouts = set()
    for turn_index, turn_sign in enumerate((-1, 1)):
        for side_index, first_side in enumerate((-1, 1)):
            # One of each shape per cell. A random diagonal balances the extra
            # double-gate shape across both turn signs and both first openings.
            extra = "same" if (turn_index + side_index + diagonal) % 2 == 0 else "opposite"
            patterns = shuffled(rng, ("single", "same", "opposite", extra))
            first = stratified(rng, cfg["first_distance_m"], len(patterns))
            angle = stratified(rng, cfg["turn_magnitude_deg"], len(patterns))
            last = stratified(rng, cfg["last_distance_m"], len(patterns))
            for pattern, distance, magnitude, end in zip(patterns, first, angle, last):
                course = Course(distance, turn_sign * magnitude, end)
                for _ in range(1024):
                    obstacle_seed = rng.seed(bits=64)
                    if obstacle_seed in used_layouts:
                        continue
                    layout = make_layout(obstacle_seed, course.first_m)
                    sides = tuple(gate.side for gate in layout.gates)
                    if sides[0] == first_side and layout_family(sides) == pattern:
                        break
                else:
                    raise ValueError("course generator did not produce a registered layout stratum")
                used_layouts.add(obstacle_seed)
                jobs.append((course, rng.seed(bits=64), obstacle_seed, sides))
    return shuffled(rng, jobs)


def strict_course(result, sides):
    """Independently require an ordered positive-clearance crossing and three accepted stops."""
    if not (result["success"] and result["reason"] == "success" and result["stages_completed"] == len(PHASES)):
        return False
    if (result["obstacle_contact"] or result["obstacle_contact_substeps"]
            or result["course_boundary_violation"] or result["missed_passage"]
            or result["first_obstacle_contact_tick"] is not None):
        return False
    stages = result["stages"]
    if (tuple(s["stage"] for s in stages) != PHASES
            or not all(s["reason"] == "success" and s["grade"]["success"] for s in stages)):
        return False
    events = result["passages"]
    if not (result["passed_gates"] == result["required_gates"] == len(sides) == len(events) == len(result["gate_layout"])):
        return False
    previous_time = -1.
    previous_tick = -1
    for index, (side, event, gate) in enumerate(zip(sides, events, result["gate_layout"])):
        tick, when = event["tick"], event["native_time_s"]
        clearance = event["gap_edge_clearance_m"]
        left_m = event["left_m"]
        edge = gate["gap_width_m"] / 2 - abs(left_m - gate["left_m"])
        if (event["gate_index"] != index or event["side"] != side or gate["side"] != side
                or not math.isclose(event["forward_m"], gate["forward_m"], abs_tol=1e-9)
                or not math.isclose(clearance, edge, abs_tol=1e-9)
                or not (tick-1)*.02 - 1e-9 <= when <= tick*.02 + 1e-9
                or not math.isfinite(clearance) or clearance <= 0.
                or not previous_tick < tick <= stages[0]["exit_tick"]
                or not math.isfinite(when) or not previous_time < when <= result["simulated_s"] + 1e-9):
            return False
        previous_tick, previous_time = tick, when
    return True


def run_episode(solver_factory, course, robot_seed, obstacle_seed, sides, cfg, assets):
    with LocomotionSession(assets, robot_seed, horizon=cfg["horizon"]) as session:
        episode = NavigationEpisode(session, course, obstacle_seed=obstacle_seed)
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
    return dict(result, strict_success=strict_course(result, sides), obstacle_seed=obstacle_seed,
                robot_seed=robot_seed, direction="left" if course.turn_deg > 0 else "right",
                first_gap_side=sides[0], gate_sides=list(sides), layout_family=layout_family(sides),
                requested_first_m=course.first_m, requested_turn_deg=course.turn_deg, requested_last_m=course.last_m)


def summarize(rows, cfg):
    def group(items):
        def total(name):
            return sum(r[name] for r in items)

        n = len(items)
        wins = [r for r in items if r["strict_success"]]
        stopped = [int(r["stages_completed"]) for r in items]
        eligible = total("eligible_code_pairs")
        samples = total("joint_accel_samples")
        out = dict(episodes=n, successes=len(wins), episodes_aborted=sum(r["reason"] == "solver_abort" for r in items),
                   applied_native_rows=total("applied_rows"), fsq_scalar_codes=total("scalar_codes"),
                   changed_fsq_coordinates=total("code_changes"), eligible_adjacent_scalar_pairs=eligible,
                   coordinate_change_fraction=total("code_changes") / eligible if eligible else 0.,
                   joint_accel_samples=samples,
                   joint_accel_rms_rad_s2=math.sqrt(total("joint_accel_square_sum") / samples) if samples else 0.,
                   actual_elapsed_sim_s=total("simulated_s"),
                   mean_actual_elapsed_sim_s=total("simulated_s") / n,
                   mean_completed_sim_s=sum(r["simulated_s"] for r in wins) / len(wins) if wins else 0.,
                   mean_completion_sim_s_failure_at_horizon=(
                       sum(r["simulated_s"] for r in wins) + (n - len(wins)) * cfg["horizon"] * .02) / n)
        out.update(required_gates=total("required_gates"), passed_gates=total("passed_gates"),
                   full_passage_courses=sum(r["passed_gates"] == r["required_gates"] for r in items),
                   obstacle_contact_courses=sum(bool(r["obstacle_contact"]) for r in items),
                   obstacle_contact_substeps=total("obstacle_contact_substeps"),
                   boundary_violations=sum(bool(r["course_boundary_violation"]) for r in items))
        clearances = [e["gap_edge_clearance_m"] for r in items for e in r["passages"]]
        if clearances:
            out["min_passage_pelvis_edge_clearance_m"] = min(clearances)
        for i, name in enumerate(PHASES):
            out[name + "_stops"] = sum(k > i for k in stopped)
            stage_records = [s for r in items for s in r["stages"] if s["stage"] == name]
            out[name + "_revealed"] = len(stage_records)
            if stage_records:
                count = len(stage_records)
                out[name + "_actual_rows"] = sum(s["exit_tick"] - s["revealed_at_tick"] for s in stage_records)
                for key in ("forward_m", "lateral_m", "signed_turn_rad", "max_abs_lateral_m",
                            "max_pivot_drift_m", "final_xy_error_m", "max_stop_dwell_s"):
                    out[name + "_mean_" + key] = sum(s["grade"][key] for s in stage_records) / count
                out[name + "_mean_absolute_final_heading_error_rad"] = (
                    sum(abs(s["grade"]["final_heading_error_rad"]) for s in stage_records) / count)
                out[name + "_mean_requested_distance_m"] = sum(s["command_distance_m"] for s in stage_records) / count
                out[name + "_mean_requested_turn_deg"] = sum(s["command_turn_deg"] for s in stage_records) / count
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
    for kind in ("single", "same", "opposite"):
        subset = [r for r in rows if r["layout_family"] == kind]
        metrics[kind + "_courses"] = len(subset)
        metrics[kind + "_successes"] = sum(r["strict_success"] for r in subset)
    for index, r in enumerate(rows, 1):
        prefix = f"trial_{index:02d}_"
        fields = dict(success=int(r["strict_success"]), native_success=int(r["success"]),
                      turn_left=int(r["direction"] == "left"), first_gap_side=r["first_gap_side"],
                      required_gates=r["required_gates"], passed_gates=r["passed_gates"],
                      obstacle_contact=int(r["obstacle_contact"]), obstacle_contact_substeps=r["obstacle_contact_substeps"],
                      course_boundary_violation=int(r["course_boundary_violation"]), missed_passage=int(r["missed_passage"]),
                      stages_completed=r["stages_completed"],
                      applied_rows=r["applied_rows"], simulated_s=r["simulated_s"], code_changes=r["code_changes"],
                      joint_accel_samples=r["joint_accel_samples"], joint_accel_square_sum=r["joint_accel_square_sum"],
                      requested_first_m=r["requested_first_m"], requested_turn_deg=r["requested_turn_deg"],
                      requested_last_m=r["requested_last_m"])
        if r["first_obstacle_contact_tick"] is not None:
            fields["first_obstacle_contact_tick"] = r["first_obstacle_contact_tick"]
            fields["first_obstacle_contact_time_s"] = r["first_obstacle_contact_time_s"]
        for i, gate in enumerate(r["gate_layout"]):
            key = f"gate_{i}_"
            fields[key + "side"] = gate["side"]
            fields[key + "center_left_m"] = gate["left_m"]
            fields[key + "width_m"] = gate["gap_width_m"]
            fields[key + "plane_forward_m"] = gate["forward_m"]
        for event in r["passages"]:
            gate = f"gate_{event['gate_index']}_"
            for key in ("tick", "native_time_s", "forward_m", "left_m", "gap_edge_clearance_m"):
                fields[gate + key] = event[key]
        for s in r["stages"]:
            if s["reason"] == "success":
                fields[s["stage"] + "_stop_tick"] = s["exit_tick"]
        metrics.update({prefix + k: v for k, v in fields.items()})
    return metrics


def run_panel(solver_factory, cfg, rng, assets=ASSETS):
    rows = []
    for course, robot_seed, obstacle_seed, sides in schedule(rng, cfg):
        rows.append(run_episode(solver_factory, course, robot_seed, obstacle_seed, sides, cfg, assets))
        print(f"[grader] completed {len(rows)}/{cfg['episodes']} courses", file=sys.stderr, flush=True)
    return summarize(rows, cfg), rows


def verify():
    cfg = stage_config()
    anchor, spec = read_anchor()

    def timeout(elapsed):
        write_verdict(zero_verdict("verifier wall-time budget exhausted", anchor is not None,
                                  extra={"elapsed_s": elapsed, "episodes": cfg["episodes"]}))

    with VerifierDeadline(cfg["verifier_deadline_s"], timeout):
        check_numerics()
        require(os.environ.get("ALE_VERIFY_PYTHON") or Path("/.dockerenv").is_file(),
                "isolated verifier sandbox required")
        require(clear_agent_account(force=True)["cleared"], "could not isolate the solver account")
        child = deprivileged(offline_command("--self-test"), env=solver_environment(),
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        output, _ = child.communicate(timeout=15)
        require(child.returncode == 0 and output.strip() == "offline", "offline launcher self-test failed")
        try:
            def solver_factory():
                return OfflineSolver(step_timeout=cfg["step_timeout_s"])
            metrics, rows = run_panel(solver_factory, cfg, seed_rng())
        except SolverMissing as exc:
            write_verdict(zero_verdict(str(exc), anchor is not None, extra={
                "episodes": cfg["episodes"], "episodes_aborted": cfg["episodes"], METRIC: 0, "successes": 0}))
            return
        metrics.update(numerics_ok=1, offline_launcher_ok=1, agent_account_cleared=1)
        detail = Path("/logs/verifier/navigation-courses.json")
        manual_path = os.environ.get("ROBOTICS_GRADER_VERDICT_PATH")
        if manual_path and not os.environ.get("ALE_VERIFY_PYTHON"):
            verdict_path = Path(manual_path)
            detail = verdict_path.with_name(verdict_path.stem + "-courses.json")
        detail.write_text(json.dumps(rows, allow_nan=False) + "\n")
        write_verdict(ratio_rewards(metrics["successes"] / metrics["episodes"], anchor,
                                    cap=cfg["scoring_cap"], full_at=spec["full_at"],
                                    metric_name=METRIC, extra=metrics))


if __name__ == "__main__":
    verify()
