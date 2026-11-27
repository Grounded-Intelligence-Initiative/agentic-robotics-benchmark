"""Score uninterrupted, physically matched deliveries under opposite visible station signals."""
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import subprocess
import sys

STAGE = Path(__file__).resolve().parent
sys.path.insert(0, str(STAGE))
sys.path.insert(1, "/opt/sonic")
from delivery_io import LiveEpisode, validate_action
from sonic_sim import DeliverySession
from robotics_grader import (SolverAborted, SolverMissing, VerifierDeadline, clear_agent_account,
                             deprivileged, ratio_rewards, read_anchor, require, seed_rng, stage_config, write_verdict)
from mobile_score import AuditedSession
from offline_solver import OfflineSolver, check_numerics, episode_scratch, offline_command, solver_environment


def panel_jobs(config, rng):
    """Both station signals in a pair share the physical layout, including the robot's initial state."""
    jobs = []
    for pair, layout in enumerate(config["layouts"]):
        seed = layout.get("seed")
        if seed is None:
            seed = rng.seed(bits=64)
        for side in ("left", "right"):
            jobs.append((pair, layout["start"], layout["shape"], seed, side, rng.seed(bits=64)))
    for i in range(len(jobs)-1, 0, -1):
        j = rng.choice(range(i+1))
        jobs[i], jobs[j] = jobs[j], jobs[i]
    return jobs


def query(solver, live, obs, rng, probe):
    alternative = live.opposite_cue() if probe else None
    if alternative is None:
        return solver.act(obs["tick"], obs), None
    packets = (obs, alternative)
    duplicate = rng.choice((0, 1))
    order = rng.choice(((0, 1, 2), (0, 2, 1), (1, 0, 2), (1, 2, 0), (2, 0, 1), (2, 1, 0)))
    actions = {}
    for index in order:
        packet = packets[duplicate if index == 2 else index]
        actions[index] = solver.act(obs["tick"], packet)
    changed = actions[0]["rows"][0] != actions[1]["rows"][0]
    noise = actions[duplicate]["rows"][0] != actions[2]["rows"][0]
    return actions[0], (int(changed), int(noise))


def episode(solver, job, config, assets):
    pair, start, shape, seed, side, probe_seed = job
    rng = seed_rng(probe_seed)
    with DeliverySession("visual_delivery", seed, assets, config["horizon"],
                         start_mode=start, shape=shape, target_side=side) as session:
        audit = AuditedSession(session)
        live = LiveEpisode(audit)
        probes = changed = noise = unlit = 0
        # Preselect requests using only the clock and private random draws.
        frames = range(0, config["horizon"], 5)
        count = min(config["probes_per_episode"], len(frames))
        probe_ticks = {rng.choice(frames[i*len(frames)//count:(i+1)*len(frames)//count])
                       for i in range(count)}
        reason = "aborted"
        try:
            if solver is None:
                raise SolverAborted("episode solver failed to start")
            solver.reset()
            while not session.state["done"]:
                obs = live.observe()
                probe = obs["tick"] in probe_ticks
                action, result = query(solver, live, obs, rng, probe)
                if result is not None:
                    probes += 1
                    unlit += int(not session.state["metrics"]["docked"])
                    changed += result[0]
                    noise += result[1]
                try:
                    validate_action(action, obs["tick"], session.horizon)
                except ValueError as exc:
                    raise SolverAborted(str(exc)) from exc
                live.advance(action, obs["observation_id"])
            reason = session.state["reason"]
        except SolverAborted:
            pass
        row = audit.report()
        native = session.state["metrics"]
        row.update(pair=pair, start=start, shape=shape, side=side, outcome=reason, probes=probes, unlit_probes=unlit,
                   altered=changed, duplicate_changes=noise, wrong_pad=int(native["wrong_pad"]),
                   native_success=int(native["success"]), complete=int(reason == "success" and
                       row["docked"] and row["lifted"] and row["placed"] and native["success"]))
        return row


def summarize(rows):
    n = len(rows)
    def total(key):
        return sum(row[key] for row in rows)
    groups = {}
    for row in rows:
        groups.setdefault(row["pair"], {})[row["side"]] = row["complete"]
    if any(set(sides) != {"left", "right"} for sides in groups.values()) or n != 2*len(groups):
        raise ValueError("each physical layout needs exactly one run for each target side")
    pairs = sum(all(sides.values()) for sides in groups.values())
    result = dict(episodes=n, layout_pairs=len(groups), both_targets_released=pairs,
                  paired_delivery_fraction=pairs/len(groups), strict_deliveries=total("complete"),
                  strict_delivery_fraction=total("complete")/n, episodes_aborted=sum(r["outcome"] == "aborted" for r in rows),
                  time_limits=sum(r["outcome"] == "time_limit" for r in rows))
    for key in ("walked", "docked", "contacted", "lifted", "placed", "wrong_pad", "disqualified", "fell",
                "applied_rows", "applied_codes", "changed_coordinates", "code_transition_slots",
                "absolute_bin_jump", "simulated_s", "joint_acceleration_samples", "joint_accel_square_sum",
                "probes", "unlit_probes", "altered", "duplicate_changes"):
        result[key] = total(key)
    result["joint_accel_rms_rad_s2"] = math.sqrt(total("joint_accel_square_sum")/max(1,total("joint_acceleration_samples")))
    result["coordinate_change_fraction"] = total("changed_coordinates")/max(1,total("code_transition_slots"))
    for flag, names in (("start", ("near", "far")), ("shape", ("box", "cylinder")), ("side", ("left", "right"))):
        for name in names:
            group = [r for r in rows if r[flag] == name]
            result[name+"_episodes"] = len(group)
            result[name+"_deliveries"] = sum(r["complete"] for r in group)
    return result


def run_panel(solver_factory, config, rng, assets=Path("/opt/sonic/assets")):
    rows = []
    for job in panel_jobs(config, rng):
        try:
            with solver_factory() as solver:
                row = episode(solver, job, config, assets)
        except SolverAborted:
            row = episode(None, job, config, assets)
        rows.append(row)
        print(f"[grader] completed {len(rows)}/{2*len(config['layouts'])}", file=sys.stderr, flush=True)
    return summarize(rows), rows


def verify():
    config = stage_config()
    anchor, calibration = read_anchor()
    require(anchor is None or anchor == 1., "a numeric anchor requires a measured full paired delivery")
    require(calibration["full_at"] == 1., "all paired deliveries are required for full credit")
    def zero(reason):
        return write_verdict(dict(rewards={"reward": 0.}, metrics={
            "episodes": 2*len(config["layouts"]), "episodes_aborted": 2*len(config["layouts"]),
            "strict_deliveries": 0, "paired_delivery_fraction": 0., "both_targets_released": 0,
            "anchor_recorded": int(anchor is not None)}), diagnostic=reason)
    with VerifierDeadline(config["verifier_deadline_s"], lambda _: zero("verifier wall-time budget exhausted")):
        check_numerics()
        require(os.environ.get("ALE_VERIFY_PYTHON") or Path("/.dockerenv").is_file(), "isolated sandbox required")
        require(clear_agent_account(force=True)["cleared"], "could not isolate agent account")
        with episode_scratch() as scratch:
            check = deprivileged(offline_command("--self-test"), env=solver_environment(scratch),
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            output, errors = check.communicate(timeout=15)
            require(check.returncode == 0 and output.strip() == "offline", "offline launcher self-test failed: " + errors[-200:])

        @contextmanager
        def fresh_solver():
            with episode_scratch() as scratch, OfflineSolver(scratch, step_timeout=config["step_timeout_s"]) as solver:
                yield solver

        try:
            metrics, rows = run_panel(fresh_solver, config, seed_rng())
        except (SolverAborted, SolverMissing) as exc:
            return zero(str(exc))
        score = metrics["paired_delivery_fraction"]
        metrics.update(numerics_ok=1, offline_launcher_ok=1)
        detail = Path("/logs/verifier/delivery-episodes.json")
        detail.write_text(json.dumps(rows, allow_nan=False, indent=2) + "\n")
        write_verdict(ratio_rewards(score, anchor, metric_name="paired_delivery_fraction",
                                    full_at=calibration["full_at"], extra=metrics))


if __name__ == "__main__":
    verify()
