"""Score native G1 navigation using only the current lights and robot-local measurements."""
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


PHASES = ("approach", "pivot", "departure")
METRIC = "strict_course_fraction"
ASSETS = Path("/opt/sonic/assets")


def first_codes(action):
    return action["rows"][0]["codes"]


def shuffled(rng, entries):
    pool = list(entries)
    result = []
    while pool:
        i = rng.choice(range(len(pool)))
        result.append(pool.pop(i))
    return result


def stratified(rng, bounds, count):
    lo, hi = bounds
    return shuffled(rng, [lo + (hi - lo) * (i + rng.random()) / count for i in range(count)])


def schedule(rng, cfg):
    count = cfg["episodes"] // 2
    require(count > 0 and 2 * count == cfg["episodes"], "expected a balanced signed course panel")
    jobs = []
    used = set()
    for sign in (-1, 1):
        first = stratified(rng, cfg["first_distance_m"], count)
        angle = stratified(rng, cfg["turn_magnitude_deg"], count)
        last = stratified(rng, cfg["last_distance_m"], count)
        for distance, magnitude, final in zip(first, angle, last):
            key = rng.seed(bits=64)
            while key in used:
                key = rng.seed(bits=64)
            used.add(key)
            jobs.append((Course(distance, sign * magnitude, final), key, seed_rng(rng.seed(bits=64))))
    return shuffled(rng, jobs)


def _different(first, other):
    return sum(x != y for x, y in zip(first_codes(first), first_codes(other)))


def visual_probes(solver, episode, live, alternative, rng):
    """Random-order visual intervention, content-preserving lighting control and exact repeat."""
    other = episode.observe(alternative)
    relit = episode.relight(live, rng.uniform(.74, .90))
    for packet in (other, relit):
        require(all(live[k] == packet[k] for k in live if k not in ("camera", "observation_id")),
                "visual probe changed robot sensors or timing")
    visible = all(live["camera"]["rgb_png"] != packet["camera"]["rgb_png"] for packet in (other, relit))
    require(live["camera"]["depth_mm_png"] == relit["camera"]["depth_mm_png"], "lighting altered depth")
    repeat_index = rng.choice([0, 1, 2])
    packets = [live, other, relit]
    packets.append(packets[repeat_index])
    replies = {}
    for i in shuffled(rng, range(4)):
        replies[i] = solver.act(live["tick"], packets[i])
    if not visible:
        return replies[0], None
    return replies[0], dict(semantic=_different(replies[0], replies[1]),
                            appearance=_different(replies[0], replies[2]),
                            repeat=_different(replies[repeat_index], replies[3]))


def run_episode(solver, course, key, rng, cfg, assets):
    direction = "left" if course.turn_deg > 0 else "right"
    seen = set()
    probes = {"turn": [], "range": []}
    range_tests = {}
    for stage, field, distance in (("approach", "first_distance_m", course.first_m),
                                   ("departure", "last_distance_m", course.last_m)):
        lo, hi = cfg[field]
        delta = rng.uniform(.65, .80)
        if distance + delta <= hi:
            threshold = rng.uniform(.10, .22)
        else:
            delta = -delta
            threshold = -delta + rng.uniform(.05, .14)
        shifted = min(hi, max(lo, distance + delta))
        other = (Course(shifted, course.turn_deg, course.last_m) if stage == "approach"
                 else Course(course.first_m, course.turn_deg, shifted))
        range_tests[stage] = (threshold, other)
    with LocomotionSession(assets, key, horizon=cfg["horizon"]) as session:
        ep = NavigationEpisode(session, course)
        try:
            solver.reset()
            while not ep.done:
                obs = ep.observe()
                stage = ep.stage
                if obs["tick"] % 5 == 0:
                    if stage == "pivot":
                        candidate = Course(course.first_m, -course.turn_deg, course.last_m)
                        due = stage not in seen
                    else:
                        threshold, candidate = range_tests[stage]
                        due = stage not in seen and ep._distance - ep._score.forward <= threshold
                    # Query multiplicity and scene production depend only on public decision timing.
                    # The physical eligibility check affects logging alone.
                    action, checks = visual_probes(solver, ep, obs, candidate, rng)
                    if due and checks is not None:
                        probes["turn" if stage == "pivot" else "range"].append(checks)
                        seen.add(stage)
                else:
                    action = solver.act(obs["tick"], obs)
                ep.advance(action, obs["observation_id"])
            result = ep.summary()
        except SolverAborted:
            result = dict(ep.summary(), success=False, reason="solver_abort")
    return dict(result, direction=direction, probes=probes)


def summarize(rows, cfg):
    def group(items):
        def total(name):
            return sum(r[name] for r in items)

        n = len(items)
        wins = [r for r in items if r["success"]]
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
    n = len(rows)
    for kind in ("turn", "range"):
        events = [event for row in rows for event in row["probes"][kind]]
        count = len(events)
        metrics[kind+"_probe_count"] = count
        fractions = {}
        for name in ("semantic", "appearance", "repeat"):
            changed = sum(e[name] >= 2 for e in events)
            fractions[name] = changed / count if count else 0.
            metrics[kind+"_"+name+"_changed"] = changed
            metrics[kind+"_"+name+"_fraction"] = fractions[name]
        net = fractions["semantic"] - max(fractions["appearance"], fractions["repeat"])
        metrics[kind+"_net_fraction"] = net
        minimum = math.ceil(n/2) if kind == "turn" else n
        metrics[kind+"_feedback_ok"] = int(count >= minimum and net >= cfg[kind+"_minimum_signal"]
                                           and fractions["repeat"] <= cfg["maximum_exact_repeat_noise"])
    metrics["match_lock_ok"] = int(metrics["range_feedback_ok"] and metrics["turn_feedback_ok"])
    return metrics


def run_panel(solver, cfg, rng, assets=ASSETS):
    rows = []
    for course, key, probe_rng in schedule(rng, cfg):
        rows.append(run_episode(solver, course, key, probe_rng, cfg, assets))
        print(f"[grader] completed {len(rows)}/{cfg['episodes']} episodes", file=sys.stderr, flush=True)
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
            with OfflineSolver(step_timeout=cfg["step_timeout_s"]) as solver:
                metrics, _ = run_panel(solver, cfg, seed_rng())
        except (SolverMissing, SolverAborted) as exc:
            write_verdict(zero_verdict(str(exc), anchor is not None,
                                       extra={"episodes": cfg["episodes"], "episodes_aborted": cfg["episodes"],
                                              METRIC: 0, "successes": 0}))
            return
        metrics.update(numerics_ok=1, offline_launcher_ok=1, agent_account_cleared=1)
        write_verdict(ratio_rewards(metrics["successes"] / metrics["episodes"], anchor, cap=cfg["scoring_cap"],
                                    full_at=spec["full_at"], metric_name=METRIC,
                                    locks_ok=metrics["match_lock_ok"], extra=metrics))


if __name__ == "__main__":
    verify()
