"""Score first native route terminals from fresh G1 locomotion goals and measured states."""
from collections import deque
import math
import os
from pathlib import Path
import subprocess
import sys

STAGE = Path(__file__).resolve().parent
sys.path.insert(0, str(STAGE))
sys.path.insert(1, "/opt/sonic")

from loco_io import LiveEpisode, Route, rebase_observation
from robotics_grader import (ProbeLock, SolverAborted, SolverMissing, VerifierDeadline,
                             clear_agent_account, deprivileged, ratio_rewards, read_anchor,
                             replies_differ, require, seed_rng, stage_config, write_verdict, zero_verdict)
from solver_host import OfflineSolver, check_numerics, offline_command, solver_environment
from sonic_loco import LocomotionSession


def first_codes(action):
    return action["rows"][0]["codes"]


def order(rng, values):
    remaining = list(values)
    result = []
    while remaining:
        item = rng.choice(remaining)
        remaining.remove(item)
        result.append(item)
    return result


def schedule(rng, cfg):
    jobs = []
    per_route = cfg["episodes"] // 2
    require(per_route >= 1 and cfg["episodes"] == 2 * per_route, "expected a balanced panel")
    seeds = set()
    for i in range(per_route):
        for route, bounds in (("straight_stop", cfg["straight_distance_m"]),
                              ("right_turn_stop", cfg["right_turn_degrees"])):
            lo, hi = bounds
            # Each family covers every equal-width command stratum exactly once.
            x = lo + (hi - lo) * (i + rng.random()) / per_route
            goal = Route(route, x, 0.) if route == "straight_stop" else Route(route, 0., -math.radians(x))
            seed = rng.seed(bits=64)
            while seed in seeds:
                seed = rng.seed(bits=64)
            seeds.add(seed)
            jobs.append((goal, seed, seed_rng(rng.seed(bits=64))))
    return order(rng, jobs)


def probe_triple(solver, tick, live, alternative, lock, rng):
    repeat = rng.choice([0, 1])
    packets = [live, alternative, [live, alternative][repeat]]
    replies = {}
    for i in order(rng, [0,1,2]):
        replies[i] = solver.act(tick, packets[i])
    lock.record(replies[0], replies[1])
    return replies[0], int(replies_differ(replies[repeat], replies[2], .5, first_codes))


def route_progress(route, obs):
    command = obs["command"]
    if route.name == "straight_stop":
        remaining = math.hypot(command["target_forward_m"], command["target_left_m"])
        return 1. - remaining / route.distance_m
    return 1. - command["heading_error_rad"] / route.heading_delta_rad


def other_command_same_family(route, rng, cfg):
    straight = route.name == "straight_stop"
    lo, hi = cfg["straight_distance_m"] if straight else cfg["right_turn_degrees"]
    magnitude = route.distance_m if straight else math.degrees(-route.heading_delta_rad)
    tail = .15 * (hi - lo)
    alternative = rng.uniform(lo,lo+tail) if magnitude >= (lo+hi)/2 else rng.uniform(hi-tail,hi)
    return Route(route.name, alternative, 0.) if straight else Route(route.name, 0., -math.radians(alternative))


def run_episode(solver, route, seed, rng, cfg, assets):
    windows = cfg["state_probe_windows"]
    ticks = [rng.choice(range(lo,hi,5)) for lo, hi in windows]
    repeat = rng.choice(ticks[:3])
    goal_ticks = [rng.choice(range(70,85,5)), rng.choice(range(245,265,5))]
    feedback = ProbeLock(rng, canonical=first_codes, tolerance=.5)
    goal_lock = ProbeLock(rng, canonical=first_codes, tolerance=.5)
    within_goal = ProbeLock(rng, canonical=first_codes, tolerance=.5)
    milestones = deque([rng.uniform(.50,.62), rng.uniform(.78,.88)])
    duplicates = noise = 0
    result = dict(route=route.name, reason="solver_abort", success=False, applied_rows=0, simulated_s=0.)
    sim = None
    ep = None
    try:
        solver.reset()
        sim = LocomotionSession(assets, seed, horizon=cfg["horizon"])
        ep = LiveEpisode(sim, route)
        history = deque()
        while not ep.score.done:
            obs = ep.observe()
            tick = obs["tick"]
            while history and tick - history[0]["tick"] > 30:
                history.popleft()
            if tick in ticks:
                eligible = [sample for sample in history if tick-sample["tick"] >= 8]
                require(bool(eligible), "valid chunks must leave an earlier proprioceptive sample")
                alt = rebase_observation(rng.choice(eligible), tick, obs["last_observation_id"])
                if tick == repeat:
                    action, changed = probe_triple(solver, tick, obs, alt, feedback, rng)
                    duplicates += 1
                    noise += changed
                else:
                    action = feedback.probe(solver, tick, obs, perturbed=alt)
            elif tick % 5 == 0 and milestones and route_progress(route, obs) >= milestones[0]:
                milestones.popleft()
                changed_goal = other_command_same_family(route, rng, cfg)
                action = within_goal.probe(solver, tick, obs, perturbed=ep.observe(other_route=changed_goal))
            elif tick in goal_ticks:
                if route.name == "straight_stop":
                    alt_route = Route("right_turn_stop", 0., -math.radians(rng.uniform(*cfg["right_turn_degrees"])))
                else:
                    alt_route = Route("straight_stop", rng.uniform(*cfg["straight_distance_m"]), 0.)
                action = goal_lock.probe(solver, tick, obs, perturbed=ep.observe(other_route=alt_route))
            else:
                action = solver.act(tick, obs)
            history.append(obs)
            ep.advance(action, obs["observation_id"])
        result = ep.score.summary()
    except SolverAborted:
        if ep is not None:
            result = dict(ep.score.summary(), reason="solver_abort", success=False)
    finally:
        if sim is not None:
            sim.close()
    result.update(state_pairs=feedback.total, state_changes=feedback.passed, goal_pairs=goal_lock.total,
                  goal_changes=goal_lock.passed, within_goal_pairs=within_goal.total,
                  within_goal_changes=within_goal.passed, duplicate_pairs=duplicates, duplicate_changes=noise)
    return result


def aggregate(rows, cfg):
    def group(items):
        def total(key):
            return sum(r.get(key, 0) for r in items)
        n = len(items)
        wins = [r for r in items if r["success"]]
        sampled = [r for r in items if "final_xy_error_m" in r]
        pairs, changes = total("eligible_code_pairs"), total("code_changes")
        accel_n = total("joint_accel_samples")
        fields = dict(episodes=n, successes=len(wins), falls=sum(r["reason"] == "fall" for r in items),
                      time_limits=sum(r["reason"] == "time_limit" for r in items),
                      path_violations=sum(r["reason"] == "path_violation" for r in items),
                      episodes_aborted=sum(r["reason"] == "solver_abort" for r in items),
                      measured_episodes=len(sampled), applied_native_rows=total("applied_rows"),
                      fsq_scalar_codes=total("scalar_codes"), changed_fsq_coordinates=changes,
                      eligible_adjacent_scalar_pairs=pairs, coordinate_change_fraction=changes/pairs if pairs else 0.,
                      joint_acceleration_samples=accel_n,
                      joint_accel_rms_rad_s2=math.sqrt(total("joint_accel_square_sum")/accel_n) if accel_n else 0.,
                      mean_actual_elapsed_sim_s=total("simulated_s")/n,
                      mean_completion_sim_s_failure_at_horizon=(sum(r["simulated_s"] for r in wins)
                                                               +(n-len(wins))*cfg["horizon"]*.02)/n)
        if wins:
            fields["mean_success_elapsed_sim_s"] = sum(r["simulated_s"] for r in wins)/len(wins)
        for side in ("left", "right"):
            for transition in ("unloads", "reloads"):
                name = side + "_" + transition
                fields[name] = total(name)
        if sampled:
            for k in ("pelvis_path_length_m", "route_efficiency", "final_xy_error_m", "max_abs_lateral_m",
                      "max_pivot_drift_m", "max_stop_dwell_s"):
                fields["mean_"+k] = sum(r[k] for r in sampled)/len(sampled)
            fields["mean_absolute_final_yaw_error_rad"] = sum(abs(r["final_heading_error_rad"]) for r in sampled)/len(sampled)
        return fields
    out = group(rows)
    for name in ("straight_stop", "right_turn_stop"):
        out.update({name+"_"+k:v for k,v in group([r for r in rows if r["route"]==name]).items()})
    for k in ("state_pairs","state_changes","goal_pairs","goal_changes","within_goal_pairs","within_goal_changes",
              "duplicate_pairs","duplicate_changes"):
        out[k] = sum(r[k] for r in rows)
    signal = out["state_changes"]/out["state_pairs"] if out["state_pairs"] else 0.
    noise = out["duplicate_changes"]/out["duplicate_pairs"] if out["duplicate_pairs"] else 0.
    enough = out["state_pairs"] >= len(rows) and out["duplicate_pairs"] >= math.ceil(len(rows)/2)
    goal_signal = out["within_goal_changes"]/out["within_goal_pairs"] if out["within_goal_pairs"] else 0.
    state_ok = enough and signal - noise >= cfg["state_probe_min_signal"]
    goal_ok = out["within_goal_pairs"] >= math.ceil(len(rows)/2) and goal_signal - noise >= cfg["within_goal_min_signal"]
    out.update(state_response_fraction=signal, within_goal_response_fraction=goal_signal,
               identical_response_change_fraction=noise, state_feedback_ok=int(state_ok),
               same_family_goal_feedback_ok=int(goal_ok), match_lock_ok=int(state_ok and goal_ok))
    return out


def run_panel(solver, cfg, rng, assets=Path("/opt/sonic/assets")):
    rows = []
    for goal, seed, schedule_rng in schedule(rng,cfg):
        rows.append(run_episode(solver,goal,seed,schedule_rng,cfg,assets))
        print(f"[grader] completed {len(rows)}/{cfg['episodes']} episodes",file=sys.stderr,flush=True)
    return aggregate(rows,cfg), rows


def verify():
    cfg = stage_config()
    anchor, spec = read_anchor()
    require(cfg.get("horizon",0)>0 and cfg.get("episodes",0)>0,"invalid stage configuration")
    def expire(seconds):
        write_verdict(zero_verdict("wall-time budget exhausted", anchor is not None, {"elapsed_s":seconds}))
    with VerifierDeadline(cfg["verifier_deadline_s"], expire):
        check_numerics()
        require(os.environ.get("ALE_VERIFY_PYTHON") or Path("/.dockerenv").is_file(),
                "the verifier must run inside an isolated task sandbox")
        require(clear_agent_account(force=True)["cleared"], "could not isolate solver account")
        proc=deprivileged(offline_command("--self-test"),env=solver_environment(),
                          stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        output,_=proc.communicate(timeout=15)
        require(proc.returncode==0 and output.strip()=="offline","offline isolation failed")
        try:
            with OfflineSolver(step_timeout=cfg["step_timeout_s"]) as solver:
                metrics,_=run_panel(solver,cfg,seed_rng())
        except (SolverMissing,SolverAborted) as exc:
            write_verdict(zero_verdict(str(exc),anchor is not None,
                                       {"episodes":cfg["episodes"],"episodes_aborted":cfg["episodes"]}))
            return
        metrics.update(numerics_ok=1,offline_launcher_ok=1,agent_account_cleared=1)
        write_verdict(ratio_rewards(metrics["successes"]/metrics["episodes"],anchor,
                                   cap=cfg["scoring_cap"],metric_name="strict_stop_fraction",
                                   locks_ok=metrics["match_lock_ok"],full_at=spec["full_at"],extra=metrics))


if __name__=="__main__":
    verify()
