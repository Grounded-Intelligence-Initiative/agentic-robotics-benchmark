"""Score complete deliveries across matched layouts with an independently randomized three-signal sequence."""
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
from offline_solver import OfflineSolver, check_numerics, episode_scratch, offline_command, solver_environment, submission_template


SIGNALS = (("left", "left", "right"), ("left", "right", "left"), ("right", "left", "left"),
           ("right", "right", "left"), ("right", "left", "right"), ("left", "right", "right"))


def panel_jobs(config, rng):
    """All three signal trials share the physical layout, including the robot's initial state."""
    jobs = []
    for pair, layout in enumerate(config["layouts"]):
        seed = rng.choice(layout["eligible_seeds"]) if "eligible_seeds" in layout else layout.get("seed")
        if seed is None:
            seed = rng.seed(bits=64)
        # Draw each nonconstant three-member sequence with probability 1/6.
        index = rng.seed(bits=3)
        while index >= len(SIGNALS):
            index = rng.seed(bits=3)
        for side in SIGNALS[index]:
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
        layout = config.get("layouts", [])
        if layout and "eligible_seeds" in layout[pair]:
            row["eligible_layout_index"] = layout[pair]["eligible_seeds"].index(seed)
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
        groups.setdefault(row["pair"], []).append(row)
    if any(len(group) != 3 or {r["side"] for r in group} != {"left", "right"} for group in groups.values()):
        raise ValueError("each physical layout requires three deliveries covering both destinations")
    complete = sum(all(r["complete"] for r in group) for group in groups.values())
    both = sum({r["side"] for r in group if r["complete"]} == {"left", "right"} for group in groups.values())
    result = dict(episodes=n, layouts=len(groups), all_signals_released=complete, both_targets_released=both,
                  strict_layout_fraction=complete/len(groups), both_targets_fraction=both/len(groups),
                  strict_deliveries=total("complete"),
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
    # ALE retains numeric metrics in verification.json. Keep terminal evidence for every allocation.
    for i, row in enumerate(rows, 1):
        prefix = f"trial_{i:02d}_"
        result[prefix + "layout"] = row["pair"]
        for key, name in (("start", "far"), ("shape", "cylinder"), ("side", "right"),
                          ("outcome", "aborted"), ("outcome", "time_limit")):
            result[prefix + name] = int(row[key] == name)
        for key in ("walked", "docked", "contacted", "lifted", "wrong_pad", "disqualified", "fell", "complete",
                    "applied_rows", "simulated_s", "changed_coordinates", "joint_acceleration_samples",
                    "joint_accel_square_sum", "dock_s", "first_contact_s", "lift_s", "place_s", "eligible_layout_index"):
            if row.get(key) is not None:
                result[prefix + key] = row[key]
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
        print(f"[grader] completed {len(rows)}/{3*len(config['layouts'])}", file=sys.stderr, flush=True)
    return summarize(rows), rows


def verify():
    config = stage_config()
    anchor, calibration = read_anchor()
    require(anchor is None or anchor == 1., "a numeric anchor requires a measured full three-signal delivery")
    require(calibration["full_at"] == 1., "all three deliveries per layout are required for full credit")
    def zero(reason):
        return write_verdict(dict(rewards={"reward": 0.}, metrics={
            "episodes": 3*len(config["layouts"]), "episodes_aborted": 3*len(config["layouts"]),
            "strict_deliveries": 0, "strict_layout_fraction": 0., "all_signals_released": 0, "both_targets_released": 0,
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

        try:
            home = Path(os.environ.get("ALE_HOME", "/home/user"))
            with submission_template(home / "submission") as install:
                @contextmanager
                def fresh_solver():
                    with install() as submission, episode_scratch() as scratch:
                        with OfflineSolver(scratch, submission=submission, step_timeout=config["step_timeout_s"]) as solver:
                            yield solver
                metrics, rows = run_panel(fresh_solver, config, seed_rng())
        except (SolverAborted, SolverMissing) as exc:
            return zero(str(exc))
        score = metrics["strict_layout_fraction"]
        metrics.update(numerics_ok=1, offline_launcher_ok=1)
        detail = Path("/logs/verifier/delivery-episodes.json")
        manual_path = os.environ.get("ROBOTICS_GRADER_VERDICT_PATH")
        if manual_path and not os.environ.get("ALE_VERIFY_PYTHON"):
            verdict_path = Path(manual_path)
            detail = verdict_path.with_name(verdict_path.stem + "-episodes.json")
        detail.write_text(json.dumps(rows, allow_nan=False, indent=2) + "\n")
        write_verdict(ratio_rewards(score, anchor, metric_name="strict_layout_fraction",
                                    full_at=calibration["full_at"], extra=metrics))


if __name__ == "__main__":
    verify()
