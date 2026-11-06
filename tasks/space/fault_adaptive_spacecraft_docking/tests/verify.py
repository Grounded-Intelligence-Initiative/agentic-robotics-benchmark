"""Hidden grader for fault-adaptive spacecraft docking.

Runs as root in the separate verifier sandbox (task.toml), from this folder. It owns the simulator (env.py,
byte-identical to the agent's development copy), drives the agent's controller (`bash run.sh` in
/home/user/submission) over the kit's wire, and scores every episode from its own simulator state; nothing the
controller prints is a scoring input.

The hidden episodes are a fixed set (hidden/episodes.json): the same 48 seeds of 64 bits, in the same order, for every
submission, none of them a practice seed. A seed fixes every hidden parameter of its episode (env.py draws them all in
`reset`), and `load_episodes` recomputes each world's digest, so a change to env.py's draws is a task error instead of a
silently different set. Each episode's match lock draws from its own stream, keyed by the set's hidden key and the
episode's seed (the kit's `seed_rng` with explicit entropy), so a submission meets the same probes every time, whatever
ran before it; nothing in a verify draws from OS entropy. A probed step is asked twice with the same `t` (the real
observation and a copy whose delayed camera pose is shifted, in a hidden order); only the real reply reaches the
simulator. An aborted episode (a missed deadline, a crash, a protocol error, a malformed action) scores 0 and stays in
the mean.

The controller has no network while it is graded: `GuardedSolver` starts it through nonet_exec.py (no_new_privs, a
seccomp filter under which socket() and socketpair() work for AF_UNIX only, then the agent's account and a fixed
environment: PATH /opt/venv/bin:/usr/local/bin:/usr/bin:/bin, HOME /home/user, LANG C.UTF-8). A self-test before
episode 0 checks that the filter holds here. Where it cannot be installed, the verify ends as a task error before any
controller starts: the set is public, so nothing is graded with the network open.

The reference program's channel: a submission whose `.oracle_token` (a regular file, never a link) hashes to
hidden/oracle_token.sha256 gets, in every observation, the simulator's current truth (`privileged_block`: every quantity
`env.step` reads that is in effect now, named as env.py names it; never a value drawn for the future, never the noise
stream). Its observation is then {"vector": <the 16 numbers>, "privileged": {"format": PRIV_FORMAT, "state": {...}}};
every other submission gets the plain 16-number list, exactly as before the channel existed. A probed step's copy
shifts the block's true x, y and yaw by the same draws as the camera pose, so the probe asks the same question of every
program. The channel exists to show that the episodes can be done; nothing else about the run changes, and the metrics
record `privileged_mode` and `oracle_token_mismatch`.
"""

import hashlib
import hmac
import json
import math
import os
import pwd
import re
import stat
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from env import FaultAdaptiveDocking  # noqa: E402
from robotics_grader import (  # noqa: E402
    ProbeLock,
    Solver,
    SolverAborted,
    SolverMissing,
    agent_user,
    fail,
    ratio_rewards,
    read_anchor,
    seed_rng,
    stage_config,
    write_verdict,
    zero_verdict,
)
from robotics_grader.stage import secure_exec_path  # noqa: E402

SUBMISSION = os.path.join(os.environ.get("ALE_HOME", "/home/user"), "submission")
HIDDEN = os.path.join(HERE, "hidden")  # (root's alone, like all of this folder at verify time)
SET_PATH = os.path.join(HIDDEN, "episodes.json")
SET_FORMAT = "fault_adaptive_spacecraft_docking fixed set 1"
TOKEN_FILE = ".oracle_token"
TOKEN_DIGEST = os.path.join(HIDDEN, "oracle_token.sha256")
TOKEN_MAX_BYTES = 4096
NONET = os.path.join(HERE, "nonet_exec.py")  # (run as root from this root-only folder)

# The privileged block: what env.step reads that is in effect now (an AST test checks the lists against env.py).
PRIV_FORMAT = "fault_adaptive_spacecraft_docking privileged 1"
PRIV_STATE = ("steps", "x", "y", "vx", "vy", "theta", "omega", "phase", "phase_rate", "port_extension_phase",
              "fuel", "routing_active", "rekey_active", "routing_relay_armed", "nozzle_exponents", "valve_memory",
              "slot_valves", "plume_swirl", "plume_energy", "first_relay_reversals", "rekey_relay_reversals",
              "hold_steps", "best_error")
PRIV_ROUTING = ("routing_order", "routing_phase", "routing_direction", "failure_nozzle",
                "secondary_failure_nozzle", "failure_scale", "secondary_failure_scale")  # null until routing_active
PRIV_CONSTANTS = ("columns", "bias", "port_amplitude", "port_omega", "plume_onset_gap", "plume_strength",
                  "plume_switch_energy", "initial_error")
# Never sent. The pre-drawn future reaches the block through PRIV_STATE / PRIV_ROUTING when it takes effect.
NEVER_SENT_FUTURE = ("failure_step", "failure_exponents", "failure_valve_memory", "routing_initial_shift",
                     "rekey_earliest", "rekey_deadline", "rekey_proximity", "rekey_order", "rekey_initial_shift",
                     "rekey_direction", "rekey_failure_nozzle", "rekey_secondary_failure_nozzle",
                     "rekey_failure_scale", "rekey_secondary_failure_scale", "rekey_exponents",
                     "rekey_valve_memory", "rekey_phase_rate")
NEVER_SENT_OBSERVATION = ("rng", "pose_delay", "blackouts", "rekey_blackout_duration", "rekey_step", "pose_history",
                          "last_visible_pose", "last_accel_body", "imu_angle", "imu_mirror", "imu_yaw_sign",
                          "rekey_imu_linear", "rekey_imu_yaw_sign", "port_encoder_sign", "rekey_port_encoder_sign")
NEVER_SENT_OUTCOME = ("success", "failure_reason")  # (always False and "" while a request is open)


def own_submission(path):
    """Hand the restored artifact back to the agent account.

    In the separate verifier the engine restores /home/user/submission as root (every
    artifact write is the framework's), so its files land root-owned with their recorded
    modes. The kit Solver runs `bash run.sh` AS THE AGENT: the agent must be able to read
    its own controller (a file it had left non-world-readable would otherwise be
    unreadable) and write scratch beside it, exactly as it could while it worked. Walk the
    tree without following links — a freshly restored tree has none, but lchown is the safe
    habit — and give every entry, then the folder itself, to the agent. A no-op when the
    path is missing or is a link; the Solver then scores a real zero through SolverMissing.
    Runs as root, which the verify stage is.
    """
    if not os.path.isdir(path) or os.path.islink(path):
        return
    try:
        account = pwd.getpwnam(agent_user())
    except KeyError:
        return
    uid, gid = account.pw_uid, account.pw_gid
    for dirpath, dirnames, filenames in os.walk(path, followlinks=False):
        for name in dirnames + filenames:
            try:
                os.lchown(os.path.join(dirpath, name), uid, gid)
            except OSError:
                pass
    os.lchown(path, uid, gid)


# ------------------------------------------------------------------------------------------------ the fixed set
def _canon(value):
    """A world value with every float at 13 significant digits: immune to a last-bit libm difference between hosts."""
    if isinstance(value, float):
        return format(value, ".12e")
    if isinstance(value, (list, tuple)):
        return [_canon(v) for v in value]
    return value


def world_digest(seed):
    """sha256 of the world `env.reset(seed)` builds (every attribute but the generator), floats canonicalised."""
    env = FaultAdaptiveDocking()
    env.reset(seed)
    world = {k: _canon(v) for k, v in vars(env).items() if k != "rng"}
    return hashlib.sha256(json.dumps(world, sort_keys=True, separators=(",", ":")).encode("ascii")).hexdigest()


def _is_seed(value):
    return isinstance(value, int) and not isinstance(value, bool) and (1 << 32) <= value < (1 << 64)


def load_episodes(path=None):
    """The fixed hidden set: (seeds in grading order, probe key as an int). A missing or malformed set, or a world
    that no longer matches its digest, is a task error; no message names a value of the set."""
    path = path or SET_PATH
    try:
        with open(path) as f:
            spec = json.load(f)
    except (OSError, ValueError) as exc:
        fail("cannot read the hidden episode set: {}".format(type(exc).__name__))
    if not isinstance(spec, dict) or spec.get("format") != SET_FORMAT:
        fail("the hidden episode set is not of the format '{}'".format(SET_FORMAT))
    key = spec.get("probe_key")
    if not isinstance(key, str) or not re.fullmatch(r"[0-9a-f]{64}", key):
        fail("the hidden episode set's probe_key is not 64 lowercase hex digits")
    episodes = spec.get("episodes")
    if not isinstance(episodes, list) or not episodes:
        fail("the hidden episode set holds no episodes")
    seeds = []
    for k, item in enumerate(episodes):
        if not isinstance(item, dict) or not _is_seed(item.get("seed")) or not isinstance(item.get("world_sha256"), str):
            fail("hidden episode {}: not a seed in [2**32, 2**64) with a world digest".format(k))
        seeds.append(item["seed"])
    if len(set(seeds)) != len(seeds):
        fail("the hidden episode set repeats a seed")
    for k, item in enumerate(episodes):
        if world_digest(item["seed"]) != item["world_sha256"]:
            fail("hidden episode {}: its world no longer matches the recorded digest (env.py's draws changed)".format(k))
    return seeds, int(key, 16)


def episode_stream(probe_key, seed):
    """The match lock's stream for one episode: keyed by the set's key and the episode's seed alone."""
    return seed_rng(entropy=(probe_key << 64) | seed)


# ------------------------------------------------------------------------------------------------ the channel
def token_state(submission, digest_path=None):
    """(privileged, mismatch) for the submission's .oracle_token against hidden/oracle_token.sha256.

    Only a regular file counts (opened without following a link and without blocking, at most TOKEN_MAX_BYTES); a token
    that is there but does not match (a wrong value, a link, a directory, a FIFO, an oversized file) is recorded as a
    mismatch and otherwise ignored."""
    digest_path = digest_path or TOKEN_DIGEST
    path = os.path.join(submission, TOKEN_FILE)
    try:
        with open(digest_path) as f:
            want = f.read().split()[0].strip().lower()
    except (IOError, OSError, IndexError):
        return False, False
    try:
        fd = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0))
    except OSError:
        return False, os.path.lexists(path)
    try:
        st = os.fstat(fd)
        if not stat.S_ISREG(st.st_mode) or st.st_size > TOKEN_MAX_BYTES:
            return False, True
        data = os.read(fd, TOKEN_MAX_BYTES + 1)
    except OSError:
        return False, True
    finally:
        os.close(fd)
    ok = hmac.compare_digest(hashlib.sha256(data.strip()).hexdigest(), want)
    return ok, not ok


def _plain(v):
    return [_plain(x) for x in v] if isinstance(v, (list, tuple)) else v


def privileged_block(env):
    """The simulator's current truth: everything env.step reads that is in effect now. Draws nothing, writes nothing."""
    state = {k: _plain(getattr(env, k)) for k in PRIV_STATE + PRIV_CONSTANTS}
    state.update({k: (_plain(getattr(env, k)) if env.routing_active else None) for k in PRIV_ROUTING})
    return {"format": PRIV_FORMAT, "state": state}


def perturbed_observation(obs, rng):
    """The probed step's copy: the delayed camera x, y and yaw shifted (three draws, in this order). In a token run the
    block's true x, y and yaw move by the same draws, so a program that reads the block answers the copy on its own."""
    token = isinstance(obs, dict)
    changed = list(obs["vector"] if token else obs)
    dx = rng.uniform(-0.28, 0.28)
    dy = rng.uniform(-0.28, 0.28)
    changed[0] += dx
    changed[1] += dy
    dyaw = rng.uniform(-0.18, 0.18)
    yaw = math.atan2(changed[3], changed[2]) + dyaw
    changed[2], changed[3] = math.cos(yaw), math.sin(yaw)
    if not token:
        return changed
    st = dict(obs["privileged"]["state"])
    st["x"] += dx
    st["y"] += dy
    st["theta"] = (st["theta"] + dyaw + math.pi) % (2.0 * math.pi) - math.pi
    return {"vector": changed, "privileged": dict(obs["privileged"], state=st)}


# ------------------------------------------------------------------------------------------------ the network
def agent_ids():
    pw = pwd.getpwnam(agent_user())
    return pw.pw_uid, pw.pw_gid


class GuardedSolver(Solver):
    """The kit's Solver (the kit stays as it is), its program started through nonet_exec.py: no_new_privs, a seccomp
    filter under which socket() and socketpair() work for AF_UNIX only (io_uring and foreign syscall ABIs refused), then
    the agent's account and a fixed environment (PATH /opt/venv/bin:/usr/local/bin:/usr/bin:/bin, HOME /home/user,
    LANG C.UTF-8). There is no unguarded way to start it: main() stops with a task error before this class is used
    where the egress self-test fails."""

    def _spawn(self):
        secure_exec_path()  # (as the kit's deprivileged does: nothing agent-writable left on root's PATH)
        try:
            uid, gid = agent_ids()
        except KeyError:
            raise SolverAborted("no agent account to run the controller as")
        cmd = [sys.executable, "-I", NONET, "--uid", str(uid), "--gid", str(gid), "--cwd", self.submission,
               "--", "bash", self.entry]
        try:
            return subprocess.Popen(cmd, cwd=self.submission, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL, universal_newlines=True, bufsize=1,
                                    start_new_session=True, env={"PATH": "/usr/bin:/bin"})
        except OSError as exc:
            raise SolverAborted("the controller could not be started: {}".format(exc))


def egress_self_test(uid, gid):
    """True if a program started through the launcher cannot open an internet socket (AF_INET, AF_INET6, AF_VSOCK)
    and can open a Unix one."""
    probe = ("import socket,sys\n"
             "bad=0\n"
             "for fam in (socket.AF_INET, socket.AF_INET6, 40):\n"
             "    try:\n"
             "        socket.socket(fam, socket.SOCK_STREAM).close(); bad=1\n"
             "    except OSError:\n"
             "        pass\n"
             "try:\n"
             "    socket.socket(socket.AF_UNIX, socket.SOCK_STREAM).close()\n"
             "except OSError:\n"
             "    bad=1\n"
             "sys.exit(3 if bad else 0)\n")
    try:
        r = subprocess.run([sys.executable, "-I", NONET, "--uid", str(uid), "--gid", str(gid), "--cwd", "/", "--",
                            sys.executable, "-I", "-c", probe],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60, env={"PATH": "/usr/bin:/bin"})
        return r.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def egress_mode():
    """Decided once, before episode 0: "seccomp" (the launcher's filter holds) or "unavailable" (it cannot be installed
    here, or there is no agent account to test it with). main() grades only under "seccomp": with "unavailable" the
    verify ends as a task error, because the set is public and a controller with the network could fetch it."""
    secure_exec_path()
    try:
        uid, gid = agent_ids()
    except KeyError:
        return "unavailable"
    return "seccomp" if egress_self_test(uid, gid) else "unavailable"


# ------------------------------------------------------------------------------------------------ the episodes
def run_episode(seed, solver, lock, privileged):
    """One hidden episode: (quality, info, steps). `lock.rng` is the episode's own stream."""
    env = FaultAdaptiveDocking()
    quality, info = 0.0, {"success": False, "collision": False, "failure_reason": "not_started"}
    try:
        obs = env.reset(seed)
        solver.reset()
        for t in range(env.max_steps):
            wire = {"vector": obs, "privileged": privileged_block(env)} if privileged else obs
            if lock.due():
                action = lock.probe(solver, t, wire, perturbed=perturbed_observation(wire, lock.rng))
            else:
                action = solver.act(t, wire)
            obs, quality, done, info = env.step(action)
            if done:
                break
    except (SolverAborted, ValueError, TypeError, OverflowError) as exc:
        quality, info = 0.0, {"success": False, "collision": False, "failure_reason": "aborted: {}".format(exc)}
    return float(quality), info, env.steps


def run_episodes(config, seeds, probe_key, solver, privileged):
    lock = ProbeLock(
        None,
        rate=config.get("probe_rate", 0.06),
        pass_fraction=config.get("probe_pass_fraction", 0.35),
        tolerance=1e-4,
    )
    qualities = []
    successes = collisions = aborted = 0
    for episode, seed in enumerate(seeds):
        # one probe stream per episode, keyed by the set's hidden key and the episode's seed: the same probes for the
        # same submission every time, whatever ran before
        lock.rng = episode_stream(probe_key, seed)
        quality, info, steps = run_episode(seed, solver, lock, privileged)
        qualities.append(quality)
        successes += int(bool(info.get("success")))
        collisions += int(bool(info.get("collision")))
        aborted += int(str(info.get("failure_reason", "")).startswith("aborted"))
        print(
            "[grader] episode {}: quality={:.4f} success={} steps={} reason={}".format(
                episode, quality, bool(info.get("success")), steps, info.get("failure_reason", "")
            ),
            file=sys.stderr,
            flush=True,
        )
    return qualities, successes, collisions, aborted, lock


def main():
    config = stage_config()
    seeds, probe_key = load_episodes()
    anchor, anchor_meta = read_anchor()
    # read before any of the submission's code runs
    privileged, mismatch = token_state(SUBMISSION)
    if privileged or mismatch:
        print("[grader] {}".format("privileged mode (oracle token)" if privileged else "oracle token mismatch: ignored"),
              file=sys.stderr, flush=True)
    flags = {"privileged_mode": float(privileged), "oracle_token_mismatch": float(mismatch)}
    # Separate verifier: the restored submission is root-owned; hand it to the agent before its program runs.
    own_submission(SUBMISSION)
    egress = egress_mode()
    print("[grader] egress guard: {}".format(egress), file=sys.stderr, flush=True)
    if egress != "seccomp":
        # Fail closed: the set is public, so a controller with the network could fetch it and replay its episode.
        fail("the grading environment cannot block the controller's network (the seccomp filter does not hold "
             "here); nothing is graded with the network open")
    flags["egress_guard_seccomp"] = 1.0  # (a verdict exists only when the filter held)

    try:
        # Raises SolverMissing (no run.sh) or SolverAborted (it never comes up): a real zero, not a broken task.
        solver = GuardedSolver(step_timeout=config.get("step_timeout_s", 30), submission=SUBMISSION)
    except (SolverMissing, SolverAborted) as exc:
        write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None, extra=flags))
        return
    try:
        qualities, successes, collisions, aborted, lock = run_episodes(
            config, seeds, probe_key, solver, privileged)
    finally:
        solver.close()

    measured = sum(qualities) / len(qualities) if qualities else 0.0
    extra = dict(
        lock.rewards(),
        episodes=float(len(qualities)),
        episodes_completed=float(len(qualities) - aborted),
        episodes_aborted=float(aborted),
        docking_successes=float(successes),
        target_collisions=float(collisions),
        **flags
    )
    for k, q in enumerate(qualities):
        extra["episode_{}_quality".format(k)] = q
    verdict = ratio_rewards(
        measured,
        anchor,
        cap=config.get("scoring_cap", 1.5),
        higher_is_better=True,
        metric_name="mean_docking_quality",
        locks_ok=lock.ok,
        extra=extra,
        full_at=anchor_meta.get("full_at", 1.0),
    )
    write_verdict(verdict)
    print(
        "[grader] measured={:.5f} anchor={} ratio={:.4f} reward={:.4f} lock={} successes={}/{} aborted={} "
        "privileged={} egress={}".format(
            measured, anchor, verdict["metrics"]["ratio"], verdict["rewards"]["reward"], lock.ok, successes,
            len(qualities), aborted, privileged, egress
        ),
        file=sys.stderr,
        flush=True,
    )


if __name__ == "__main__":
    main()
