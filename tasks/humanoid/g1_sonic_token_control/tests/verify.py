"""Trusted ALE verifier for a native G1 pick-and-place episode."""
from collections import deque
import hashlib
import math
import os
from pathlib import Path
import subprocess
import sys

STAGE = Path(__file__).resolve().parent
sys.path.insert(0, str(STAGE))
sys.path.insert(1, "/opt/sonic")

from robotics_grader import (ProbeLock, Solver, SolverAborted, SolverMissing, VerifierDeadline,
                             clear_agent_account, deprivileged, ratio_rewards, read_anchor, replies_differ, require,
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


RUNTIME_ENV = (
    "OPENBLAS_CORETYPE", "OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS",
    "NPY_DISABLE_CPU_FEATURES", "MUJOCO_GL", "PYOPENGL_PLATFORM", "EGL_PLATFORM",
    "__EGL_VENDOR_LIBRARY_FILENAMES", "LIBGL_ALWAYS_SOFTWARE", "GALLIUM_DRIVER", "LP_NUM_THREADS",
)


def solver_environment():
    env = dict(HOME="/home/user", PATH="/opt/venv/bin:/usr/local/bin:/usr/bin:/bin",
               LANG="C.UTF-8", PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1")
    # The image's root-installed startup file pins these before NumPy or MuJoCo imports.
    # Propagate only the explicit public runtime settings to native and Python solvers.
    env.update({key: os.environ[key] for key in RUNTIME_ENV})
    return env


def check_numerics():
    """Check the packaged float64 BLAS result before allocating any hidden episode."""
    i = np.arange(71 * 97, dtype=np.int64)
    j = np.arange(97 * 53, dtype=np.int64)
    a = ((i * 31337 + 11) % 3299 - 1649).reshape(71, 97) / 1291.0
    b = ((j * 30757 + 17) % 3511 - 1755).reshape(97, 53) / 1303.0
    digest = hashlib.sha256(np.asarray(a @ b, dtype="<f8").tobytes()).hexdigest()
    require(digest == "f83f8230aa5e4c8b0e595efe592952b387120788d96d7aed3696fea03728c0cc",
            "float64 matrix product differs from the pinned CPU runtime")


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


class MeasuredSession:
    """Record the token and measured velocity after each successful real step."""
    def __init__(self, session, reset_velocity):
        self.session = session
        self.velocities = deque([reset_velocity], maxlen=5)
        self.previous_token = None
        self.body_code_changes = 0
        self.acceleration_squared_sum = 0.0

    def __getattr__(self, name):
        return getattr(self.session, name)

    def step(self, token, left_hand, right_hand):
        state = self.session.step(token, left_hand, right_hand)
        # LiveEpisode maps a 0..31 bin exactly to (bin - 16) / 16 in float32.
        if self.previous_token is not None:
            self.body_code_changes += int(np.count_nonzero(token != self.previous_token))
        self.previous_token = token
        velocity = state["observation"]["dq"]
        if len(self.velocities) == 5:
            acceleration = (velocity - self.velocities[0]) / 0.1
            self.acceleration_squared_sum += float(np.sum(acceleration ** 2))
        self.velocities.append(velocity)
        return state


def motion_metrics(rows):
    ticks = sum(row["ticks"] for row in rows)
    comparisons = 64 * sum(max(0, row["ticks"] - 1) for row in rows)
    changes = sum(row["body_code_changes"] for row in rows)
    samples = sum(max(0, row["ticks"] - 4) for row in rows)
    metrics = dict(simulated_seconds=ticks / 50, executed_fsq_scalars=64 * ticks,
                   body_code_comparisons=comparisons, body_code_changes=changes,
                   joint_acceleration_samples=samples)
    if comparisons:
        metrics["body_code_churn_fraction"] = changes / comparisons
    if samples:
        square_sum = sum(row["acceleration_squared_sum"] for row in rows)
        metrics["joint_acceleration_rms_rad_s2"] = math.sqrt(square_sum / (29 * samples))
    return metrics


def episode(solver, shape, seeds, rng, config, assets):
    horizon = config["horizon"]
    task = shape + "_to_pad_challenge"
    probe_ticks = [rng.choice(range(lo, hi, 5)) for lo, hi in config["probe_windows"]]
    duplicate_tick = rng.choice(probe_ticks[:4])
    lock = ProbeLock(rng, canonical=first_codes, tolerance=0.5)
    report = dict(shape=shape, ticks=0, reason="aborted", placed=0, lifted=0,
                  probes=0, sensitive=0, duplicates=0, noisy=0, opportunities_lost=0)
    measured = None
    try:
        solver.reset()
        with (BenchSession(task, seeds[0], assets, horizon) as real,
              BenchSession(task, seeds[1], assets, horizon) as shadow):
            state = real.state
            measured = MeasuredSession(real, state["observation"]["dq"])
            live = LiveEpisode(measured)
            while not state["done"]:
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
    report["body_code_changes"] = measured.body_code_changes if measured is not None else 0
    report["acceleration_squared_sum"] = measured.acceleration_squared_sum if measured is not None else 0.0
    report.update(motion_metrics([report]))
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
    metrics.update(motion_metrics(rows))
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
        check_numerics()
        # In the official stage the kit clears the account before starting anything as
        # the solver. Explicitly check its report so no pre-existing service can relay
        # traffic. A maintainer can exercise the same code in an isolated Docker container.
        require(os.environ.get("ALE_VERIFY_PYTHON") or Path("/.dockerenv").is_file(),
                "the verifier must run inside an isolated task sandbox")
        require(clear_agent_account(force=True)["cleared"], "could not isolate the agent account")
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
        metrics.update(numerics_ok=1, offline_launcher_ok=1, agent_account_cleared=1)
        raw = metrics["placed"] / metrics["episodes"]
        verdict = ratio_rewards(raw, anchor, cap=config["scoring_cap"],
                                metric_name="strict_placement_fraction", locks_ok=metrics["match_lock_ok"],
                                full_at=specification["full_at"], extra=metrics)
        write_verdict(verdict)


if __name__ == "__main__":
    verify()
