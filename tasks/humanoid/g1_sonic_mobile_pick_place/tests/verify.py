"""Grade the submitted public JSONL controller on fresh mobile G1 episodes."""
import math
import os
from pathlib import Path
import subprocess
import sys

STAGE = Path(__file__).resolve().parent
sys.path.insert(0, str(STAGE))
sys.path.insert(1, "/opt/sonic")

import numpy as np
from mobile_io import LiveEpisode
from sonic_sim import MobileSession
from robotics_grader import (ProbeLock, SolverAborted, SolverMissing, VerifierDeadline,
                             clear_agent_account, deprivileged, ratio_rewards, read_anchor, replies_differ,
                             require, seed_rng, stage_config, write_verdict, zero_verdict)
from mobile_score import AuditedSession
from offline_solver import OfflineSolver, check_numerics, offline_command, solver_environment


ASSETS = Path("/opt/sonic/assets")
METRIC = "strict_mobile_placement_fraction"


def first_codes(action):
    return action["rows"][0]["codes"]


def probe_triple(solver, tick, live, other, lock, rng):
    packets = [live, other]
    duplicate = rng.choice([0, 1])
    packets.append(packets[duplicate])
    order = rng.choice([(0, 1, 2), (0, 2, 1), (1, 0, 2), (1, 2, 0), (2, 0, 1), (2, 1, 0)])
    replies = {index: solver.act(tick, packets[index]) for index in order}
    lock.record(replies[0], replies[1])
    return replies[0], int(replies_differ(replies[duplicate], replies[2], .5, first_codes))


def episode(solver, shape, seeds, rng, cfg, assets):
    task = shape + "_to_pad_challenge"
    horizon = cfg["horizon"]
    probes = [rng.choice(range(lo, hi, 5)) for lo, hi in cfg["probe_windows"]]
    duplicate_tick = rng.choice(probes[:4])
    lock = ProbeLock(rng, canonical=first_codes, tolerance=.5)
    row = dict(shape=shape, outcome="aborted", noisy=0, duplicates=0, lost=0)
    with (MobileSession(task, seeds[0], assets, horizon) as real,
          MobileSession(task, seeds[1], assets, horizon) as shadow):
        audit = AuditedSession(real)
        live = LiveEpisode(audit)
        try:
            solver.reset()
            while not real.state["done"]:
                obs = live.observe()
                tick = obs["tick"]
                alternate = None
                if tick in probes and not shadow.state["done"]:
                    second = LiveEpisode(shadow)
                    second.last_hand = obs["last_hand"].copy()
                    second.last_observation_id = obs["last_observation_id"]
                    alternate = second.observe()
                    require(alternate["tick"] == alternate["vision_tick"] == tick,
                            "paired mobile probes must be current observations at the same control tick")
                    if alternate["observation_id"] == obs["observation_id"]:
                        alternate = None
                if alternate is None:
                    row["lost"] += int(tick in probes)
                    action = solver.act(tick, obs)
                elif tick == duplicate_tick:
                    action, changed = probe_triple(solver, tick, obs, alternate, lock, rng)
                    row["duplicates"] += 1
                    row["noisy"] += changed
                else:
                    action = lock.probe(solver, tick, obs, perturbed=alternate)
                applied = live.advance(action, obs["observation_id"])
                for command in action["rows"][:applied]:
                    if shadow.state["done"]:
                        break
                    token = (np.asarray(command["codes"], dtype=np.float32) - 16) / 16
                    shadow.step(token, *command["hand"])
            row["outcome"] = real.state["reason"]
        except SolverAborted:
            pass
        row.update(audit.report(), probes=lock.total, sensitive=lock.passed)
        native = real.state["metrics"]
        # One physical trajectory must satisfy both independently implemented scoring paths.
        row["native_placed"] = int(native["placed"])
        row["native_success"] = int(native["success"])
        row["complete"] = int(row["outcome"] == "success" and row["placed"] and row["native_success"])
        return row


def run_panel(solver, cfg, rng, assets=ASSETS):
    shapes = cfg["shapes"]
    jobs = []
    occupied = set()
    for i in range(cfg["episodes"]):
        keys = []
        while len(keys) < 2:
            key = rng.seed(bits=64)
            if key not in occupied:
                occupied.add(key)
                keys.append(key)
        jobs.append((shapes[i % len(shapes)], keys, seed_rng(rng.seed(bits=64))))
    rows = []
    for shape, keys, schedule in jobs:
        rows.append(episode(solver, shape, keys, schedule, cfg, assets))
        print(f"[grader] completed {len(rows)}/{len(jobs)} episodes", file=sys.stderr, flush=True)
    return summarize(rows, cfg), rows


def summarize(rows, cfg):
    n = len(rows)
    def total(name):
        return sum(row[name] for row in rows)
    pairs, duplicates = total("probes"), total("duplicates")
    signal = total("sensitive") / max(1, pairs)
    noise = total("noisy") / max(1, duplicates)
    counted = (pairs >= n * cfg["minimum_probe_opportunities_per_episode"] and duplicates >= math.ceil(n/2))
    result = dict(episodes=n, strict_mobile_placements=total("complete"),
                  episodes_aborted=sum(row["outcome"] == "aborted" for row in rows),
                  time_limits=sum(row["outcome"] == "time_limit" for row in rows),
                  probes_total=pairs, probes_passed=total("sensitive"), duplicate_probes=duplicates,
                  duplicate_changes=total("noisy"), missed_reached_probe_opportunities=total("lost"),
                  probe_change_fraction=signal, duplicate_change_fraction=noise,
                  match_lock_ok=int(counted and signal-noise >= cfg["minimum_probe_fraction"]))
    for name in ("walked", "docked", "contacted", "lifted", "placed", "disqualified", "fell", "native_placed",
                 "native_success", "applied_rows", "applied_codes", "changed_coordinates", "code_transition_slots",
                 "absolute_bin_jump", "simulated_s"):
        result[name] = total(name)
    for key in ("horizontal_path_m", "max_forward_m", "best_dock_hold_s", "minimum_pelvis_height_m",
                "max_lift_m", "unique_code_rows", "base_xy_acceleration_rms_m_s2", "target_step_rms_rad",
                "left_foot_rise_m", "right_foot_rise_m", "single_support_phases"):
        result["mean_"+key] = total(key) / n
    denominator = total("joint_acceleration_samples")
    result["joint_acceleration_samples"] = denominator
    result["joint_accel_square_sum"] = total("joint_accel_square_sum")
    result["joint_accel_rms_rad_s2"] = math.sqrt(result["joint_accel_square_sum"] / denominator) if denominator else 0.
    result["coordinate_change_fraction"] = total("changed_coordinates") / max(1, total("code_transition_slots"))
    for key in ("walk_s", "dock_s", "first_contact_s", "lift_s", "place_s"):
        times = [r[key] for r in rows if r[key] is not None]
        if times:
            result["mean_"+key] = sum(times)/len(times)
    for shape in cfg["shapes"]:
        subset = [r for r in rows if r["shape"] == shape]
        result[shape+"_episodes"] = len(subset)
        result[shape+"_placements"] = sum(r["complete"] for r in subset)
    return result


def verify():
    cfg, (anchor, spec) = stage_config(), read_anchor()
    def timeout(elapsed):
        write_verdict(zero_verdict("verifier wall-time budget exhausted", anchor is not None,
                                   extra={"elapsed_s": elapsed, "episodes": cfg["episodes"]}))
    with VerifierDeadline(cfg["verifier_deadline_s"], timeout):
        check_numerics()
        require(os.environ.get("ALE_VERIFY_PYTHON") or Path("/.dockerenv").is_file(), "isolated sandbox required")
        require(clear_agent_account(force=True)["cleared"], "could not isolate the agent account")
        check = deprivileged(offline_command("--self-test"), env=solver_environment(),
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        output, _ = check.communicate(timeout=15)
        require(check.returncode == 0 and output.strip() == "offline", "offline launcher self-test failed")
        try:
            with OfflineSolver(step_timeout=cfg["step_timeout_s"]) as solver:
                metrics, _ = run_panel(solver, cfg, seed_rng())
        except (SolverAborted, SolverMissing) as exc:
            write_verdict(zero_verdict(str(exc), anchor is not None, extra={"episodes": cfg["episodes"],
                          "episodes_aborted": cfg["episodes"], METRIC: 0.0, "strict_mobile_placements": 0,
                          "joint_acceleration_samples": 0, "joint_accel_square_sum": 0., "joint_accel_rms_rad_s2": 0.}))
            return
        metrics.update(numerics_ok=1, offline_launcher_ok=1, agent_account_cleared=1)
        value = metrics["strict_mobile_placements"] / metrics["episodes"]
        write_verdict(ratio_rewards(value, anchor, cap=cfg["scoring_cap"], full_at=spec["full_at"],
                                    metric_name=METRIC, locks_ok=metrics["match_lock_ok"], extra=metrics))


if __name__ == "__main__":
    verify()
