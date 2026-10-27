"""The grader for driving_safety_supervisor. Runs as ROOT in a separate verification
sandbox, after the agent is gone.

Closed-loop pattern: this process owns the simulator (CARLA 0.9.15 through `drive_bridge`,
a py3.10 child process run as its own account) and asks the agent's supervisor, every
0.1 s of simulated time, whether to let the nominal planner's control through or to
replace it. The episode score comes from the simulator's own route progress and infraction
record, never from anything the supervisor reports.

Why the result can be trusted:

* Separate verification: a fresh sandbox holds only the submission; nothing the agent left
  running in its own sandbox can reach the grading run.
* The supervisor runs at the agent's UID (`robotics_grader.Solver`) and, through
  `netguard`, with no IP sockets: CARLA's RPC port listens on loopback and would otherwise
  hand out every actor's true state (and the controls of the world). The guard is tested
  before any episode runs. CARLA runs as a third account (`carlasim`).
* Episode seeds are drawn from the kit's in-memory seed stream; routes, hazard placement,
  timing and weather are generated from them inside the bridge. Most hidden episodes stage
  their kind as a variant practice never shows (`drive_variants`, grading stage only), and
  some add distractors that need no reaction.
* The probe lock re-asks random steps with a road user pasted onto the road in the front
  camera, so a supervisor that does not look at its camera fails the lock and scores 0.
* The episode score (`episode_score`) lives only here; the practice grader reports what
  happened in an episode, not how it scores.
"""

import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from drive_protocol import (  # noqa: E402
    EpisodeOutcome,
    ProtocolError,
    probe_observation,
    run_episode,
)
from drive_variants import graded_specs  # noqa: E402
from episode_score import aggregate  # noqa: E402
from robotics_grader import (  # noqa: E402
    ProbeLock,
    Solver,
    SolverAborted,
    SolverMissing,
    fail,
    ratio_rewards,
    read_anchor,
    seed_rng,
    stage_config,
    write_verdict,
    zero_verdict,
)
from robotics_grader.stage import agent_home, agent_user  # noqa: E402
from sim_client import SimClient, SimError  # noqa: E402


def log(message):
    print("[grader] {}".format(message), file=sys.stderr, flush=True)


def guarded_entry(python):
    guard_src = os.path.join(HERE, "netguard.py")
    check = subprocess.run([python, "-I", "-S", guard_src, "selftest"],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           universal_newlines=True)
    if check.returncode != 0:
        fail("netguard does not hold in this sandbox: {}".format(check.stdout.strip()))
    folder = tempfile.mkdtemp(prefix="drive-guard-",
                              dir="/run" if os.path.isdir("/run") else None)
    os.chmod(folder, 0o755)
    shutil.copy(guard_src, os.path.join(folder, "netguard.py"))
    submission = os.path.join(agent_home(), "submission")
    with open(os.path.join(folder, "run.sh"), "w") as handle:
        handle.write("#!/bin/bash\nexec {py} -I -S {guard} exec {sub} bash run.sh\n".format(
            py=python, guard=os.path.join(folder, "netguard.py"), sub=submission))
    for name in ("netguard.py", "run.sh"):
        os.chmod(os.path.join(folder, name), 0o644)
    return folder


def readable_bridge(sim_user):
    """The bridge with its variants and probe sprites, readable by the simulator account."""
    bridge = os.path.join(HERE, "drive_bridge.py")
    if not sim_user:
        return bridge
    folder = tempfile.mkdtemp(prefix="drive-bridge-")
    os.chmod(folder, 0o755)
    for name in ("drive_bridge.py", "drive_variants.py"):
        shutil.copy(os.path.join(HERE, name), os.path.join(folder, name))
        os.chmod(os.path.join(folder, name), 0o644)
    sprites = os.path.join(folder, "probe_sprites")
    shutil.copytree(os.path.join(HERE, "probe_sprites"), sprites)
    os.chmod(sprites, 0o755)
    for name in os.listdir(sprites):
        os.chmod(os.path.join(sprites, name), 0o644)
    return os.path.join(folder, "drive_bridge.py")


def main():
    config = stage_config()
    rng = seed_rng()
    anchor, anchor_meta = read_anchor()

    entry = os.path.join(agent_home(), "submission", "run.sh")
    if not os.path.isfile(entry):
        write_verdict(zero_verdict(reason="no submission at {}".format(entry),
                                   anchored=anchor is not None))
        return

    towns = list(config.get("towns", ["Town04", "Town05", "Town10HD"]))
    specs = graded_specs(rng, towns)
    log("{} hidden episodes over {} towns".format(len(specs), len(towns)))

    folder = guarded_entry(config.get("guard_python", "/opt/venv/bin/python3"))
    log("supervisor runs as {} under netguard".format(agent_user()))
    lock = ProbeLock(rng, rate=config.get("probe_rate", 0.02),
                     pass_fraction=config.get("probe_pass_fraction", 0.25))

    backend = config.get("backend", "carla")
    sim_user = config.get("sim_user") or None
    sim_log = os.path.join(tempfile.mkdtemp(prefix="drive-simlog-"), "sim.log")
    sim_args = ["--image-size", str(config.get("image_size", 512)),
                "--background", str(config.get("background_vehicles", 6))]
    try:
        sim = SimClient(backend=backend, extra_args=sim_args, log_path=sim_log,
                        bridge=readable_bridge(sim_user), run_as=sim_user,
                        step_timeout=float(config.get("sim_step_timeout_s", 120)))
    except SimError as exc:
        _tail(sim_log)
        fail("simulator did not start: {}".format(exc))

    outcomes, aborted = [], 0
    try:
        try:
            solver = Solver(step_timeout=config.get("step_timeout_s", 10), submission=folder)
        except (SolverMissing, SolverAborted) as exc:
            write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None))
            return
        try:
            for index, spec in enumerate(specs):
                def probe(t, obs, state):
                    if not lock.due():
                        return None
                    return lock.probe(solver, t, obs, perturbed=probe_observation(obs, state))

                try:
                    solver.reset()
                    outcome = run_episode(sim, spec, solver.act, probe=probe)
                except (SolverAborted, ProtocolError) as exc:
                    aborted += 1
                    outcome = EpisodeOutcome({"kind": spec["kind"], "variant": spec["variant"]},
                                             aborted=True,
                                             note="aborted: {}".format(exc)[:120])
                outcomes.append(outcome)
                log("episode {} [{} {} {}+{}] {}".format(
                    index, spec["town"], spec["kind"], spec["variant"], spec["distractors"],
                    outcome.record()))
        finally:
            solver.close()
    except SimError as exc:
        _tail(sim_log)
        fail("simulator failed mid-run: {}".format(exc))
    finally:
        sim.close()
        shutil.rmtree(folder, ignore_errors=True)

    metrics = aggregate(outcomes)
    measured = metrics.pop("safety_drive_score")
    extra = dict(metrics)
    extra.update(lock.rewards())
    extra["episodes_aborted"] = float(aborted)
    verdict = ratio_rewards(
        measured, anchor,
        cap=config.get("scoring_cap", 1.5),
        higher_is_better=anchor_meta.get("direction", "higher") == "higher",
        metric_name="safety_drive_score",
        locks_ok=lock.ok,
        extra=extra,
        full_at=anchor_meta.get("full_at", 1.0),
    )
    write_verdict(verdict)
    log("safety_drive_score={:.4f} RC={:.3f} ped_hits={:.3f} anchor={} ratio={:.3f} "
        "reward={:.3f} lock={} ({}/{}) aborted={}".format(
            measured, metrics["route_completion"], metrics["pedestrian_collision_rate"],
            anchor, verdict["metrics"]["ratio"], verdict["rewards"]["reward"], lock.ok,
            lock.passed, lock.total, aborted))


def _tail(path, lines=60):
    try:
        with open(path) as handle:
            for line in handle.readlines()[-lines:]:
                print("[sim] " + line.rstrip(), file=sys.stderr)
    except OSError:
        pass


if __name__ == "__main__":
    main()
