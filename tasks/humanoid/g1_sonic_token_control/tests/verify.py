"""Trusted ALE verifier for a native G1 pick-and-place episode."""
import math
from pathlib import Path
import subprocess
import sys

STAGE = Path(__file__).resolve().parent
sys.path.insert(0, str(STAGE))
sys.path.insert(1, "/opt/sonic")

from robotics_grader import (ProbeLock, Solver, SolverAborted, SolverMissing, VerifierDeadline,
                             deprivileged, ratio_rewards, read_anchor, replies_differ, require,
                             seed_rng, stage_config, write_verdict, zero_verdict)
from sonic_io import LiveEpisode, validate_action
from sonic_sim import BenchSession
import numpy as np


OFFLINE_SOURCE = (STAGE / "offline_exec.py").read_text()
ASSETS = Path("/opt/sonic/assets")
SHAPES = ("box", "cylinder")
MAX_REPLY_CHARS = 64 * 1024


def offline_command(entry):
    # The bootstrap imports only the isolated interpreter's standard library.
    return [sys.executable, "-I", "-S", "-c", OFFLINE_SOURCE, entry]


def solver_environment():
    return dict(HOME="/home/user", PATH="/opt/venv/bin:/usr/local/bin:/usr/bin:/bin",
                LANG="C.UTF-8", PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1")


class OfflineSolver(Solver):
    def _spawn(self):
        return deprivileged(offline_command(self.entry), clear_account=self.clear_account,
                            cwd=self.submission, env=solver_environment(), stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
                            bufsize=1, start_new_session=True)

    @staticmethod
    def _pump(proc, lines):
        # Legal replies are under 3 KiB. Bound unsolicited output on the trusted side.
        try:
            while True:
                line = proc.stdout.readline(MAX_REPLY_CHARS + 1)
                if not line or len(line) > MAX_REPLY_CHARS or lines.qsize() >= 32:
                    break
                lines.put(line)
        except (OSError, ValueError):
            pass
        finally:
            lines.put(None)

    def _exchange(self, message):
        reply = super()._exchange(message)
        if message["type"] == "reset":
            if set(reply) != {"ok"} or reply["ok"] is not True:
                raise SolverAborted("reset requires a true acknowledgment")
        elif message["type"] == "act":
            if set(reply) != {"action"}:
                raise SolverAborted("act requires a single action")
            try:
                validate_action(reply["action"], message["t"])
            except ValueError as exc:
                raise SolverAborted(str(exc)) from exc
        return reply


def first_codes(action):
    return action["rows"][0]["codes"]


def controls_probe(solver, tick, actual, alternate, lock, rng):
    """One random duplicate of either input, in a uniformly permuted blind triple."""
    packets = [actual, alternate]
    repeated = rng.choice([0, 1])
    packets.append(packets[repeated])
    order = list(rng.choice([(0, 1, 2), (0, 2, 1), (1, 0, 2),
                             (1, 2, 0), (2, 0, 1), (2, 1, 0)]))
    responses = {}
    for i in order:
        responses[i] = solver.act(tick, packets[i])
    lock.record(responses[0], responses[1])
    noisy = replies_differ(responses[repeated], responses[2], 0.5, first_codes)
    return responses[0], int(noisy)


def advance_shadow(shadow, rows):
    for row in rows:
        if shadow.state["done"]:
            return
        token = (np.asarray(row["codes"], dtype=np.float32) - 16) / 16
        shadow.step(token, *row["hand"])


def episode(solver, shape, seeds, rng, config, assets):
    horizon = config["horizon"]
    task = shape + "_to_pad_challenge"
    probe_ticks = [rng.choice(range(lo, hi, 5)) for lo, hi in config["probe_windows"]]
    duplicate_tick = rng.choice(probe_ticks[:4])
    lock = ProbeLock(rng, canonical=first_codes, tolerance=0.5)
    report = dict(shape=shape, ticks=0, reason="aborted", placed=0, lifted=0,
                  probes=0, sensitive=0, duplicates=0, noisy=0, opportunities_lost=0)
    try:
        solver.reset()
        with (BenchSession(task, seeds[0], assets, horizon) as real,
              BenchSession(task, seeds[1], assets, horizon) as shadow):
            live = LiveEpisode(real)
            while not real.state["done"]:
                actual = live.observe()
                tick = actual["tick"]
                if tick in probe_ticks and not shadow.state["done"]:
                    candidate = LiveEpisode(shadow)
                    candidate.last_hand = actual["last_hand"].copy()
                    candidate.last_observation_id = actual["last_observation_id"]
                    alternate = candidate.observe()
                    require(alternate["tick"] == tick and alternate["vision_tick"] == tick,
                            "shadow and live observations must share the same capture tick")
                    if alternate["observation_id"] == actual["observation_id"]:
                        report["opportunities_lost"] += 1
                        action = solver.act(tick, actual)
                    elif tick == duplicate_tick:
                        action, noisy = controls_probe(solver, tick, actual, alternate, lock, rng)
                        report["noisy"] += noisy
                        report["duplicates"] += 1
                    else:
                        action = lock.probe(solver, tick, actual, perturbed=alternate)
                else:
                    report["opportunities_lost"] += int(tick in probe_ticks)
                    action = solver.act(tick, actual)
                applied = live.advance(action, actual["observation_id"])
                advance_shadow(shadow, action["rows"][:applied])
                report["ticks"] += applied
            state = real.state
            metrics = state["metrics"]
            require(bool(metrics["success"]) == bool(metrics["placed"]), "native terminal mismatch")
            report.update(reason=state["reason"], placed=int(metrics["placed"]),
                          lifted=int(metrics["lifted"]), max_lift_m=float(metrics["max_lift_m"]),
                          goal_error_xy_m=float(metrics["goal_error_xy_m"]),
                          release_hold_s=float(metrics["release_hold_s"]))
    except SolverAborted:
        pass
    report.update(probes=lock.total, sensitive=lock.passed)
    return report


def run_panel(solver, config, rng, assets=ASSETS):
    # Preallocate the roster so a solver's action chunk length, retries or early termination
    # cannot change any subsequent real reset, shadow reset or probe schedule.
    jobs = []
    occupied = set()
    for index in range(config["episodes"]):
        seeds = []
        while len(seeds) < 2:
            seed = rng.seed(bits=64)
            if seed not in occupied:
                occupied.add(seed)
                seeds.append(seed)
        jobs.append((SHAPES[index % 2], seeds, seed_rng(rng.seed(bits=64))))
    rows = []
    for shape, seeds, schedule in jobs:
        rows.append(episode(solver, shape, seeds, schedule, config, assets))
        print(f"[grader] completed {len(rows)}/{len(jobs)} episodes", file=sys.stderr, flush=True)
    count = len(rows)
    def total(key):
        return sum(row[key] for row in rows)
    pairs, duplicates = total("probes"), total("duplicates")
    signal = total("sensitive") / pairs if pairs else 0.0
    noise = total("noisy") / duplicates if duplicates else 0.0
    enough = (pairs >= count * config["minimum_probe_opportunities_per_episode"]
              and duplicates >= math.ceil(count / 2))
    ok = enough and signal - noise >= config["minimum_probe_fraction"]
    metrics = dict(episodes=count, placed=total("placed"), lifted=total("lifted"),
                   falls=sum(r["reason"] == "fall" for r in rows),
                   episodes_aborted=sum(r["reason"] == "aborted" for r in rows),
                   time_limits=sum(r["reason"] == "time_limit" for r in rows),
                   applied_native_rows=total("ticks"), probes_total=pairs,
                   probes_passed=total("sensitive"), duplicate_probes=duplicates,
                   duplicate_changes=total("noisy"), missed_reached_probe_opportunities=total("opportunities_lost"),
                   probe_change_fraction=signal, duplicate_change_fraction=noise, match_lock_ok=int(ok))
    completed = [r for r in rows if r["reason"] != "aborted"]
    for name in ("max_lift_m", "goal_error_xy_m", "release_hold_s"):
        if completed:
            metrics["mean_" + name] = sum(r[name] for r in completed) / len(completed)
    for shape in SHAPES:
        subset = [r for r in rows if r["shape"] == shape]
        metrics[shape + "_episodes"] = len(subset)
        metrics[shape + "_placed"] = sum(r["placed"] for r in subset)
    return metrics, rows


def verify():
    config = stage_config()
    anchor, specification = read_anchor()
    require(config.get("episodes") == 16 and config.get("horizon") == 750, "invalid stage config")
    def expired(elapsed):
        write_verdict(zero_verdict("verifier wall-time budget exhausted", anchor is not None,
                                   extra={"elapsed_s": elapsed}))
    with VerifierDeadline(config["verifier_deadline_s"], expired):
        # A failed security self-test is a grader error and cannot silently enable egress.
        check = deprivileged(offline_command("--self-test"), env=solver_environment(),
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        output, _ = check.communicate(timeout=15)
        require(check.returncode == 0 and output.strip() == "offline", "offline launcher self-test failed")
        try:
            with OfflineSolver(step_timeout=config["step_timeout_s"]) as solver:
                metrics, _ = run_panel(solver, config, seed_rng())
        except (SolverAborted, SolverMissing) as exc:
            write_verdict(zero_verdict(str(exc), anchor is not None,
                                       extra={"episodes": config["episodes"], "episodes_aborted": config["episodes"]}))
            return
        raw = metrics["placed"] / metrics["episodes"]
        verdict = ratio_rewards(raw, anchor, cap=config["scoring_cap"],
                                metric_name="strict_placement_fraction", locks_ok=metrics["match_lock_ok"],
                                full_at=specification["full_at"], extra=metrics)
        write_verdict(verdict)


if __name__ == "__main__":
    verify()
