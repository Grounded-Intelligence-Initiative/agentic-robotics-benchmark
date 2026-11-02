"""Trusted scoring; all submission code and XML are executed at the agent UID."""
import json
import math
import os
from pathlib import Path
import secrets
import selectors
import signal
import subprocess
import time

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
import numpy as np
import mujoco

from audit import audit_trajectory
from submission_snapshot import snapshot_submission
from robotics_grader import (
    Solver, SolverAborted, SolverMissing, deprivileged, ratio_rewards,
    read_anchor, require, stage_dir, write_verdict, zero_verdict, ProbeLock, seed_rng,
)
from world import LIMITS, World, png

SUBMISSION = "/opt/visual_robotics/evaluation/submission"


def reconstruction_probes(seed):
    """New action sequences excite both robot motion and object transport."""
    cases = []
    rng = np.random.default_rng(seed ^ 0x51E5)
    for _ in range(3):
        world = World("arm", seed, render=False)
        actions, positions = [], []
        xy = world.block.copy()
        destination = world.tray + rng.uniform(-.12, .12, 2)
        lift = float(rng.uniform(.9, 1.25))
        waypoints = [([*xy, .85], -1), ([*xy, .30], -1),
                     ([*xy, .30], 1), ([*xy, lift], 1),
                     ([*destination, lift], 1), ([*destination, .32], 1),
                     ([*destination, .30], -1)]
        for goal, grip in waypoints:
            for step in range(25):
                error = np.array(goal)-world.position()
                action = np.r_[np.clip((np.array(goal)-world.command)/.14, -1, 1), grip].tolist()
                world.act(action)
                actions.append(action)
                positions.append(np.r_[world.position(), world.data.body("object").xpos].tolist())
                if np.linalg.norm(error) < .045 and step > 0:
                    break
                if world.done:
                    break
            if world.done:
                break
        pos = np.array(positions)
        require(len(pos) > 10 and np.max(np.linalg.norm(pos[:, 3:]-pos[0, 3:], axis=1)) > .3,
                "reconstruction probe failed to excite object dynamics")
        cases.append((actions, pos))
        world.close()
    return cases


def prediction_output(process, actions, timeout=20, byte_limit=2_000_000):
    """Bound both input and output pipes, including children that never read stdin."""
    payload = json.dumps({"actions": actions}).encode()
    offset, size, chunks = 0, 0, []
    deadline = time.monotonic()+timeout
    os.set_blocking(process.stdin.fileno(), False)
    os.set_blocking(process.stdout.fileno(), False)
    with selectors.DefaultSelector() as selector:
        selector.register(process.stdin, selectors.EVENT_WRITE, "stdin")
        selector.register(process.stdout, selectors.EVENT_READ, "stdout")
        eof = False
        while not eof:
            remaining = deadline-time.monotonic()
            ready = selector.select(timeout=max(0., remaining)) if remaining > 0 else []
            if not ready:
                raise subprocess.TimeoutExpired("prediction", timeout)
            for key, _ in ready:
                if key.data == "stdin":
                    try:
                        offset += os.write(key.fd, payload[offset:offset+8192])
                    except BrokenPipeError:
                        offset = len(payload)
                    if offset == len(payload):
                        selector.unregister(process.stdin)
                        process.stdin.close()
                else:
                    chunk = os.read(key.fd, 8192)
                    if not chunk:
                        eof = True
                        break
                    size += len(chunk)
                    if size > byte_limit:
                        raise ValueError("prediction output exceeds limit")
                    chunks.append(chunk)
    process.wait(timeout=max(.01, deadline-time.monotonic()))
    if process.returncode:
        raise ValueError("prediction process failed")
    return b"".join(chunks)


def parse_prediction(payload, count):
    try:
        value = json.loads(payload)
        rows = value["positions"]
        if (not isinstance(rows, list) or len(rows) != count
                or any(not isinstance(row, list) or len(row) != 6 for row in rows)
                or any(isinstance(x, bool) or not isinstance(x, (int, float))
                       or not math.isfinite(x) for row in rows for x in row)):
            raise ValueError("expected finite numeric N-by-6 position predictions")
        return np.asarray(rows, dtype=float).reshape(count, 6)
    except (ValueError, KeyError, TypeError, OverflowError, RecursionError) as exc:
        raise ValueError("invalid prediction output") from exc


def run_prediction(actions, model_only=False):
    entry = "/opt/visual_robotics/client/model_rollout.py" if model_only else "predict.py"
    process = deprivileged(
        ["/opt/venv/bin/python3", *(["-I"] if model_only else []), entry],
        cwd=SUBMISSION,
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    try:
        return parse_prediction(prediction_output(process, actions), len(actions))
    finally:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()


def validate_model():
    code = (
        "import runpy; "
        "d=runpy.run_path('/opt/visual_robotics/client/model_rollout.py'); "
        "d['load_model']('scene.xml')"
    )
    process = deprivileged(
        ["/opt/venv/bin/python3", "-I", "-c", code], cwd=SUBMISSION,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True,
    )
    try:
        return process.wait(timeout=15) == 0
    finally:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()


def counterfactual_observation(world, observation, rng):
    """Render a displaced pose in separate data; never advance the actual episode."""
    original = world.data
    alternate = mujoco.MjData(world.model)
    alternate.qpos[:] = original.qpos
    alternate.qvel[:] = original.qvel
    alternate.ctrl[:] = original.ctrl
    alternate.eq_active[:] = original.eq_active
    alternate.qpos[:2] += [rng.uniform(.5, .8)*rng.choice((-1, 1)) for _ in range(2)]
    alternate.qpos[:2] = np.clip(alternate.qpos[:2], -3., 3.)
    alternate.qpos[2] = np.clip(alternate.qpos[2]+rng.uniform(.3, .6), .4, 1.8)
    tray_id, tray_position = None, None
    if world.kind == "arm":
        # Stay below the gantry's front rail and within its reachable workspace;
        # an occluded/out-of-workspace counterfactual is not a valid sensitivity probe.
        alternate.qpos[:2] = np.clip(alternate.qpos[:2], -2.4, 2.4)
        alternate.qpos[2] = np.clip(alternate.qpos[2], .4, 1.35)
        # A staged setpoint controller can legitimately hold its command while a
        # servo settles. Change its visual task target as well as the arm pose.
        alternate.qpos[3:5] += [rng.uniform(.35, .6)*rng.choice((-1, 1)) for _ in range(2)]
        tray_id = world.model.geom("tray").id
        tray_position = world.model.geom_pos[tray_id].copy()
        world.model.geom_pos[tray_id, :2] += [rng.uniform(.35, .6)*rng.choice((-1, 1)) for _ in range(2)]
    try:
        mujoco.mj_forward(world.model, alternate)
        world.data = alternate
        return {**observation, "images": [png(im) for im in world.images()]}
    finally:
        world.data = original
        if tray_id is not None:
            world.model.geom_pos[tray_id] = tray_position


def evaluate_controller(kind, seeds):
    results = []
    with Solver(step_timeout=10, max_restarts=3, submission=SUBMISSION) as solver:
        for seed in seeds:
            world = World(kind, seed)
            lock = ProbeLock(seed_rng(), rate=.1, pass_fraction=.5)
            try:
                solver.reset()
                obs = world.observe()
                while not obs["done"]:
                    if lock.due():
                        alternative = counterfactual_observation(world, obs, lock.rng)
                        action = lock.probe(solver, world.steps, obs, perturbed=alternative)
                    else:
                        action = solver.act(world.steps, obs)
                    obs = world.act(action)
                results.append({**world.metrics(), **lock.rewards(), "aborted": 0})
            except (SolverAborted, ValueError, TypeError, OverflowError) as exc:
                # An unusable controller loses this episode, not unrelated valid episodes.
                results.append({**world.metrics(), **lock.rewards(), "success": 0., "aborted": 1})
                print(f"[controller-episode-aborted] {exc}", flush=True)
            finally:
                world.close()
    return results


def main():
    root = Path(stage_dir())
    private = Path("/var/lib/visual_robotics")
    require((private/"state.json").is_file(), "missing protected environment ledger")
    # Stop the live endpoint before running arbitrary deliverables, closing a side channel.
    pid = int((private/"pid").read_text())
    try:
        os.kill(pid, 0)
        require(Path(f"/proc/{pid}/stat").read_text().split()[2] != "Z",
                "world service crashed; this is an environment failure, not an agent failure")
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        require(False, "world service exited unexpectedly")
    state = json.loads((private/"state.json").read_text())
    kind = json.loads((root/"grader_config.json").read_text())["kind"]
    require(state["kind"] == kind, "wrong service episode")
    anchor, calibration = read_anchor()
    metrics = state["metrics"]
    # ALE captures verifier stdout as framework-owned evidence before destroying
    # the sandbox. Do not put this journal on an agent-writable path.
    print("[authoritative-journal] " + json.dumps({
        "kind": kind, "requests": state["requests"], "events": state["events"],
        "metrics": metrics, "replay_seed_post_run": state["seed"],
    }, separators=(",", ":")), flush=True)
    extra = {f"exploration_{k}": v for k, v in metrics.items()}
    locks_ok = True
    extra["trusted_requests"] = state["requests"]
    if kind in ("flight", "arm"):
        try:
            snapshot = snapshot_submission("/home/user/submission", SUBMISSION)
            print("[frozen-submission] " + json.dumps(snapshot), flush=True)
        except (ValueError, OSError) as exc:
            write_verdict(zero_verdict(f"invalid submission: {exc}", anchored=anchor is not None))
            return
        sub = Path(SUBMISSION)
        needed = ["run.sh"] + (["scene.xml", "predict.py"] if kind == "arm" else [])
        if not all((sub/name).is_file() for name in needed):
            write_verdict(zero_verdict("missing deliverables", anchored=anchor is not None))
            return
        error, model_error, consistency, reconstruction = 0., 0., 0., 1.
        if kind == "arm":
            try:
                if not validate_model():
                    raise ValueError("scene is not a runnable robot-and-object model")
                errors, model_errors, consistency_errors = [], [], []
                for actions, expected in reconstruction_probes(state["seed"]):
                    predicted = run_prediction(actions)
                    from_model = run_prediction(actions, model_only=True)
                    errors.append(np.mean((predicted-expected)**2))
                    model_errors.append(np.mean((from_model-expected)**2))
                    consistency_errors.append(np.mean((predicted-from_model)**2))
                error = math.sqrt(float(np.mean(errors)))
                model_error = math.sqrt(float(np.mean(model_errors)))
                consistency = math.sqrt(float(np.mean(consistency_errors)))
                reconstruction = float(np.exp(-max(error, model_error)/.15))
            except (ValueError, KeyError, OSError, subprocess.TimeoutExpired):
                write_verdict(zero_verdict("invalid reconstruction", anchored=anchor is not None))
                return
        # Seeds exist only in grader memory. The model never receives reset parameters.
        seeds = ([state["seed"]] if kind == "arm" else []) + [
            secrets.randbits(64) for _ in range(3 if kind == "arm" else 4)]
        try:
            results = evaluate_controller(kind, seeds)
        except (SolverMissing, SolverAborted, ValueError, TypeError) as exc:
            write_verdict(zero_verdict(f"controller protocol failure: {exc}", anchored=anchor is not None))
            return
        print("[evaluation-replay-seeds-post-run] " + json.dumps(seeds), flush=True)
        success = float(np.mean([r["success"] for r in results]))
        probes = sum(r["probes_total"] for r in results)
        passed = sum(r["probes_passed"] for r in results)
        locks_ok = probes == 0 or passed/probes >= .5
        total_damage = metrics["damage_cost"] + sum(r["damage_cost"] for r in results)
        measured = success * reconstruction * max(0., 1-total_damage/40)
        extra.update({"success_rate": success, "prediction_rmse": error,
                      "probes_total": probes, "probes_passed": passed,
                      "match_lock_ok": float(locks_ok),
                      "scene_rollout_rmse": model_error,
                      "predictor_scene_disagreement": consistency,
                      "reconstruction_quality": reconstruction,
                      "cumulative_damage_cost": total_damage,
                      "evaluation_episodes": len(results),
                      "episodes_aborted": sum(r["aborted"] for r in results),
                      "mean_collisions": np.mean([r["collisions"] for r in results]),
                      "mean_actions": np.mean([r["steps"] for r in results])})
    else:
        if not state["events"]:
            write_verdict(zero_verdict("no interaction", anchored=anchor is not None))
            return
        if kind == "icl":
            path = os.environ.get("ALE_TRAJECTORY_PATH")
            require(path is not None, "engine trajectory is required for direct-model evaluation")
            trajectory = json.loads(Path(path).read_text())
            # This identity comes from the engine-staged, root-only ATIF file.
            # Never use a user-writable marker or the existence of ~/.ale-oracle.
            oracle = trajectory.get("agent", {}).get("name") == "oracle"
            if not oracle:
                require(trajectory.get("schema_version") == "ATIF-v1.7"
                        and trajectory.get("agent", {}).get("model_name"),
                        "unsupported/missing trusted model trace; do not count this as a model failure")
            try:
                audited = audit_trajectory(path, state["events"])
            except (ValueError, KeyError, TypeError):
                audited = False
            extra.update({"scripted_oracle": float(oracle),
                          "model_tool_audit_passed": float(audited)})
            if not (oracle or audited):
                write_verdict(zero_verdict("direct-tool audit failed",
                                          anchored=anchor is not None, extra=extra))
                return
        measured = metrics["success"]
        extra.update(metrics)
        extra["simulation_seconds"] = metrics["steps"] * .2
    write_verdict(ratio_rewards(
        measured, anchor, metric_name=calibration["metric"], extra=extra, locks_ok=locks_ok,
        full_at=calibration["full_at"],
    ))


if __name__ == "__main__":
    main()
