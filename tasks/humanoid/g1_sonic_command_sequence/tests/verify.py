"""Grade three revealed commands in one native, continuously controlled G1 simulation."""
from collections import deque
import math
import os
from pathlib import Path
import subprocess
import sys

STAGE = Path(__file__).resolve().parent
sys.path.insert(0, str(STAGE))
sys.path.insert(1, "/opt/sonic")

from robotics_grader import (
    ProbeLock, SolverAborted, SolverMissing, VerifierDeadline, clear_agent_account,
    deprivileged, ratio_rewards, read_anchor, replies_differ, require, seed_rng,
    stage_config, write_verdict, zero_verdict,
)
from sequence_io import Course, SequenceEpisode, rebase_observation
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


def triple(solver, tick, live, alternate, lock, rng):
    repeat = rng.choice([0, 1])
    packets = [live, alternate, (live, alternate)[repeat]]
    replies = {}
    for i in shuffled(rng, range(3)):
        replies[i] = solver.act(tick, packets[i])
    lock.record(replies[0], replies[1])
    noise = int(replies_differ(replies[repeat], replies[2], 0.5, first_codes))
    return replies[0], noise


def run_episode(solver, course, key, rng, cfg, assets):
    windows = cfg["state_probe_tick_windows"]
    due = {stage: deque(rng.choice(range(lo, hi)) for lo, hi in windows[stage]) for stage in PHASES}
    state = ProbeLock(rng, canonical=first_codes, tolerance=0.5)
    branch = ProbeLock(rng, canonical=first_codes, tolerance=0.5)
    state_duplicates = state_noise = branch_duplicates = branch_noise = 0
    direction = "left" if course.turn_deg > 0 else "right"
    with LocomotionSession(assets, key, horizon=cfg["horizon"]) as session:
        ep = SequenceEpisode(session, course)
        history = deque()
        try:
            solver.reset()
            while not ep.done:
                obs = ep.observe()
                tick, local, stage = obs["tick"], obs["stage_tick"], obs["stage"]
                # Never use another leg's target, sensor frame or control history.
                while history and (history[0]["stage"] != stage or tick - history[0]["tick"] > 30):
                    history.popleft()
                if stage == "pivot" and local == 0:
                    other_course = Course(course.first_m, -course.turn_deg, course.last_m)
                    other = ep.observe(alternative=other_course)
                    require(all(obs[k] == other[k] for k in obs if k not in ("command", "observation_id")),
                            "signed branch changed more than its current command")
                    a, b = obs["command"], other["command"]
                    require(all(a[k] == b[k] for k in a if k != "heading_error_rad")
                            and abs(a["heading_error_rad"] + b["heading_error_rad"]) < 1e-8,
                            "branch must flip only the requested relative yaw")
                    action, changed = triple(solver, tick, obs, other, branch, rng)
                    branch_duplicates += 1
                    branch_noise += changed
                elif due[stage] and local >= due[stage][0]:
                    due[stage].popleft()
                    previous = [item for item in history if tick - item["tick"] >= 8]
                    require(previous, "expected a genuine earlier measured state from the current leg")
                    old = rng.choice(previous)
                    other = rebase_observation(old, tick, local, obs["last_observation_id"])
                    if state_duplicates == 0:
                        action, changed = triple(solver, tick, obs, other, state, rng)
                        state_duplicates += 1
                        state_noise += changed
                    else:
                        action = state.probe(solver, tick, obs, perturbed=other)
                else:
                    action = solver.act(tick, obs)
                history.append(obs)
                ep.advance(action, obs["observation_id"])
            result = ep.summary()
        except SolverAborted:
            result = dict(ep.summary(), success=False, reason="solver_abort")
    return dict(result, direction=direction, state_pairs=state.total, state_changes=state.passed,
                state_duplicates=state_duplicates, state_noise=state_noise,
                branch_pairs=branch.total, branch_changes=branch.passed,
                branch_duplicates=branch_duplicates, branch_noise=branch_noise)


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
    for kind, minimum in (("state", n), ("branch", math.ceil(n / 2))):
        for field in ("pairs", "changes", "duplicates", "noise"):
            key = kind + "_" + field
            metrics[key] = sum(r[key] for r in rows)
        pairs, repeats = metrics[kind + "_pairs"], metrics[kind + "_duplicates"]
        signal = metrics[kind + "_changes"] / pairs if pairs else 0.
        noise = metrics[kind + "_noise"] / repeats if repeats else 0.
        net = signal - noise
        metrics[kind + "_response_fraction"] = signal
        metrics[kind + "_repeat_change_fraction"] = noise
        metrics[kind + "_net_response_fraction"] = net
        metrics[kind + "_feedback_ok"] = int(pairs >= minimum and repeats >= math.ceil(n / 2)
                                             and net >= cfg[kind + "_net_minimum"])
    metrics["match_lock_ok"] = int(metrics["state_feedback_ok"] and metrics["branch_feedback_ok"])
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
