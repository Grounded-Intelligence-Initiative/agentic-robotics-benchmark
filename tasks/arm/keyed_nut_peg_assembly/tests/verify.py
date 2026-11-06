"""Hidden grader for keyed_nut_peg_assembly (closed loop, hidden nuts, fixed hidden episodes).

Runs as root in a SEPARATE verifier sandbox (SPEC 14.10: no agent process exists there; the engine
restores only the declared artifact /home/user/submission). Defensively it still stops any agent-UID
process first (rev-leak R14, SPEC L3), then hands the
restored submission (size-capped) to the agent account in place, at /home/user/submission, the folder the
instruction names (ownership changes never follow a symlink). It owns its own copy of
the robot environment (keyed_env.py, byte-identical to the agent's dev sim) and plays the fixed hidden
episodes of tests/ref/episodes.json in order (each: a nut drawn with the public `sample_nut_spec` from a
secret seed, the target peg, the layout and noise seed), drives the agent's controller over the kit wire at the agent's
UID, and scores every episode from its own simulator state (`NutEnv.episode_score`: 1 if the nut was
assembled on the target peg, else 0). The metric is the fraction of assembled episodes; the staged
`NutEnv.progress` is reported as a diagnostic only.

Reference-solution access: if the submission holds a file `oracle_token` whose SHA-256 equals
tests/oracle_token.sha256, every observation also carries `_gt` (the ground-truth scene state). Only
solution/ has the token; the hash is useless without it. The probe lock is recorded, never gating.

No memory between episodes: every episode starts a new controller process from a fresh copy of the
submission as restored (kept root-only), after every agent-UID process has been stopped and every
file the agent account owns in the writable places outside the submission has been removed.

Additions to the kit wire (the vendored kit stays byte-identical):
  * GuardedSolver: the solver is started through nonet_exec.py, which installs a seccomp filter
    (only AF_UNIX sockets), drops to the agent account and execs `bash run.sh` with a whitelist
    environment. Reply lines are capped at 64 KiB.
  * A start-up self-test proves the network block (`egress_guard`); if the filter cannot be
    installed, /proc/net is polled every step and any agent-UID internet socket zeroes the run.
  * Every agent-UID process is killed (bounded, fail-closed) before the solver starts and after it
    closes.
  * A wall-clock budget below the stage timeout: on overrun the remaining episodes count 0.
  * Every path agent behaviour can reach is charged as a score, never a crash.
"""

import json
import math
import os
import pwd
import shutil
import signal
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import hashlib  # noqa: E402

import keyed_env as E  # noqa: E402
from robotics_grader import (  # noqa: E402
    ProbeLock,
    Solver,
    SolverAborted,
    SolverMissing,
    agent_user,
    ratio_rewards,
    read_anchor,
    seed_rng,
    stage_config,
    write_verdict,
    zero_verdict,
)

STAGE = os.path.dirname(os.path.abspath(__file__))
MAX_SUBMISSION_BYTES = 200 * 1024 * 1024
MAX_SUBMISSION_FILES = 5000
NONET = os.path.join(STAGE, "nonet_exec.py")
EPISODES = os.path.join(STAGE, "ref", "episodes.json")
TOKEN_HASH = os.path.join(STAGE, "oracle_token.sha256")
MAX_LINE = 65536


def log(msg):
    print("[grader] " + msg, file=sys.stderr, flush=True)


def _agent_ids():
    pw = pwd.getpwnam(agent_user())
    return pw.pw_uid, pw.pw_gid


def _agent_pids(uid):
    pids = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open("/proc/{}/status".format(name)) as f:
                zombie = False
                for line in f:
                    if line.startswith("State:"):
                        # a zombie runs no code; it only waits to be reaped (r5: a zombie made the
                        # bounded kill report cleanup_failed in 1 of 24 oracle runs under load)
                        zombie = line.split()[1].startswith(("Z", "X"))
                    elif line.startswith("Uid:"):
                        if int(line.split()[1]) == uid and not zombie:
                            pids.append(int(name))
                        break
        except (OSError, ValueError, IndexError):
            continue
    return pids


def kill_agent_processes(uid, rounds=20, pause=0.25):
    """Kill every process of the agent account; True when none is left (bounded)."""
    for _ in range(rounds):
        pids = _agent_pids(uid)
        if not pids:
            return True
        for pid in pids:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        time.sleep(pause)
    return not _agent_pids(uid)


class GuardedSolver(Solver):
    """The kit Solver, spawned through the seccomp launcher, with a reply-line cap."""

    guard = True
    nproc = 0

    def _spawn(self):
        uid, gid = _agent_ids()
        cmd = [sys.executable, "-I", NONET, "--uid", str(uid), "--gid", str(gid), "--rlimit-nproc", str(self.nproc),
               "--cwd", self.submission, "--", "bash", self.entry]
        if not self.guard:
            cmd = ["/usr/sbin/runuser", "-u", agent_user(), "--", "/usr/bin/env", "-i",
                   "PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin", "HOME=/home/user", "LANG=C.UTF-8",
                   "bash", self.entry]
        try:
            return subprocess.Popen(cmd, cwd=self.submission, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL, universal_newlines=True, bufsize=1,
                                    start_new_session=True, env={"PATH": "/usr/bin:/bin"})
        except OSError as exc:
            raise SolverAborted("solver could not be started: {}".format(exc))

    @staticmethod
    def _pump(proc, lines):
        while True:
            try:
                line = proc.stdout.readline(MAX_LINE + 1)
            except (ValueError, OSError, UnicodeDecodeError):
                break
            if not line:
                break
            if len(line) > MAX_LINE and not line.endswith("\n"):
                break          # oversize reply: treated as a closed wire (episode aborts)
            lines.put(line)
        lines.put(None)


def egress_self_test(uid, gid, nproc=0):
    """True if a probe started through the launcher cannot open an internet socket."""
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
        r = subprocess.run([sys.executable, "-I", NONET, "--uid", str(uid), "--gid", str(gid),
                            "--rlimit-nproc", str(nproc), "--cwd", "/",
                            "--", sys.executable, "-I", "-c", probe],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60,
                           env={"PATH": "/usr/bin:/bin"})
        return r.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def agent_inet_sockets(uid):
    """Count agent-UID internet sockets with a non-loopback peer (detection fallback)."""
    n = 0
    for name in ("tcp", "tcp6", "udp", "udp6", "raw", "raw6"):
        try:
            with open("/proc/net/" + name) as f:
                next(f, None)
                for line in f:
                    parts = line.split()
                    if len(parts) < 8 or int(parts[7]) != uid:
                        continue
                    rem = parts[2].split(":")[0]
                    if rem.strip("0") == "" or rem in ("0100007F", "00000000000000000000000001000000"):
                        continue
                    n += 1
        except (OSError, ValueError, StopIteration):
            continue
    return n


def stage_submission(uid, gid):
    """Make the restored submission the agent account's own folder, in place at ~user/submission: the
    folder the instruction names, where run.sh is started and where the controller may write, as it can in
    the practice grader. Ownership changes never follow a symlink (lchown, a walk that does not
    follow links); size and file count are capped. Returns the path, or None if there is no usable
    submission."""
    src = os.path.join(os.path.expanduser("~" + agent_user()), "submission")
    if not os.path.isdir(src) or os.path.islink(src):
        return None
    total = files = 0
    for dp, dns, fns in os.walk(src, followlinks=False):
        for fn in fns:
            fp = os.path.join(dp, fn)
            files += 1
            if not os.path.islink(fp):
                try:
                    total += os.lstat(fp).st_size
                except OSError:
                    pass
        if files > MAX_SUBMISSION_FILES or total > MAX_SUBMISSION_BYTES:
            return None
    for dp, dns, fns in os.walk(src, followlinks=False):
        for name in dns + fns:
            os.lchown(os.path.join(dp, name), uid, gid)
    os.lchown(src, uid, gid)
    os.chmod(src, 0o700)
    return src


def oracle_access(workdir):
    """True if the submission carries the reference solution's token (see the module docstring)."""
    p = os.path.join(workdir, "oracle_token")
    try:
        if os.path.islink(p) or not os.path.isfile(p) or os.path.getsize(p) > 256:
            return False
        with open(p, "rb") as fh:
            tok = fh.read().strip()
        with open(TOKEN_HASH) as fh:
            want = fh.read().strip()
    except OSError:
        return False
    return hashlib.sha256(tok).hexdigest() == want


SCRUB_ROOTS = ("/tmp", "/var/tmp", "/dev/shm", "/usr/local", "/run", "/var/lock")


def _agent_entries(uid, home):
    """Paths the agent account owns under the writable roots and its home (never following a symlink),
    except the submission folder and what is inside it."""
    sub = os.path.join(home, "submission")
    found = set()
    for root in list(SCRUB_ROOTS) + [home]:
        for dp, dns, fns in os.walk(root, topdown=True, followlinks=False):
            dns[:] = [d for d in dns if os.path.join(dp, d) != sub]
            for name in dns + fns:
                path = os.path.join(dp, name)
                try:
                    if os.lstat(path).st_uid == uid:
                        found.add(path)
                except OSError:
                    continue
    return found


def scrub_agent_files(uid, home, baseline):
    """Remove every file or directory the agent account owns there that was not there before grading
    started (`baseline`): what the image ships is never touched."""
    for path in sorted(_agent_entries(uid, home) - baseline, key=len):
        try:
            if os.path.isdir(path) and not os.path.islink(path):
                shutil.rmtree(path, ignore_errors=True)
            elif os.path.lexists(path):
                os.unlink(path)
        except OSError:
            pass


def fresh_submission(pristine, workdir, uid, gid):
    """Replace the submission folder with a fresh copy of the restored one, owned by the agent."""
    shutil.rmtree(workdir, ignore_errors=True)
    shutil.copytree(pristine, workdir, symlinks=True)
    for dp, dns, fns in os.walk(workdir, followlinks=False):
        for name in dns + fns:
            os.lchown(os.path.join(dp, name), uid, gid)
    os.lchown(workdir, uid, gid)
    os.chmod(workdir, 0o700)


def run_episodes(config, rng, uid, egress_mode, t_start, specs, plan, workdir, with_gt, pristine=None, gid=None,
                 baseline=None):
    budget = float(config.get("grader_budget_s", 6000))
    lock = ProbeLock(rng, rate=config.get("probe_rate", 0.01),
                     pass_fraction=config.get("probe_pass_fraction", 0.1), tolerance=1e-4)
    scores, aborted, faults = [], 0, 0
    per_nut = {}
    per_stage = {}
    progress = []
    egress_hits = 0
    budget_exhausted = False
    GuardedSolver.guard = egress_mode == "seccomp"
    GuardedSolver.nproc = int(config.get("solver_nproc_limit", 0))
    solver = None
    try:
        for ep, item in enumerate(plan):
            if solver is not None:
                try:
                    solver.close()
                except Exception:  # noqa: BLE001 - closing a broken solver must not crash the grader
                    pass
            if pristine is not None:
                kill_agent_processes(uid)
                scrub_agent_files(uid, os.path.dirname(workdir), baseline or set())
                fresh_submission(pristine, workdir, uid, gid)
            solver = None
            nut = item["nut"]
            if time.monotonic() - t_start > budget:
                budget_exhausted = True
                scores.append(0.0)
                per_nut.setdefault(nut, []).append(0.0)
                continue
            env = E.NutEnv(specs[nut], early_end=True)
            score = 0.0
            prog = 0.0
            reason = "ok"
            try:
                obs = env.reset(int(item["seed"]), int(item["peg_bands"]))
                solver = GuardedSolver(step_timeout=config.get("step_timeout_s", 30), submission=workdir,
                                       max_restarts=0)
                solver.reset()
                for t in range(env.max_steps):
                    if time.monotonic() - t_start > budget:
                        budget_exhausted = True
                        raise SolverAborted("grader budget exhausted")
                    if egress_mode == "detect" and agent_inet_sockets(uid):
                        egress_hits += 1
                    wire = E.to_wire(obs)
                    if with_gt:
                        wire["_gt"] = env.gt()
                    if lock.due():
                        other = E.to_wire(env.perturbed_observation(obs, rng))
                        if with_gt:
                            other["_gt"] = wire["_gt"]
                        action = lock.probe(solver, t, wire, perturbed=other)
                    else:
                        action = solver.act(t, wire)
                    obs, done, info = env.step(action)
                    if done:
                        break
                if env.fault is not None:
                    faults += 1
                    reason = "sim fault: " + env.fault
                    score = 0.0
                else:
                    score = env.episode_score()
                    prog = env.progress()
            except (SolverAborted, SolverMissing, E.ActionError, ValueError, TypeError, OverflowError, KeyError,
                    AttributeError, RecursionError) as exc:
                aborted += 1
                score = 0.0
                reason = "aborted: {}".format(str(exc)[:160])
            finally:
                env.close()
            if not (isinstance(score, float) and math.isfinite(score)) or score < 0:
                score = 0.0
            scores.append(score)
            per_nut.setdefault(nut, []).append(score)
            progress.append(prog)
            for name, level in (("lifted", 0.25), ("on_target_peg", 0.35), ("assembled", 1.0)):
                per_stage.setdefault(name, []).append(1.0 if prog >= level else 0.0)
            log("episode {} ({}, {} bands): assembled={:.0f} progress={:.3f} steps={} {}".format(
                ep, nut, item["peg_bands"], score, prog, getattr(env, "t", 0), reason))
    finally:
        try:
            if solver is not None:
                solver.close()
        except Exception:  # noqa: BLE001 - closing a broken solver must not crash the grader
            pass
    return scores, aborted, faults, per_nut, per_stage, egress_hits, budget_exhausted, lock, progress


def main():
    t_start = time.monotonic()
    config = stage_config()
    rng = seed_rng()
    anchor, meta = read_anchor()
    try:
        uid, gid = _agent_ids()
    except KeyError:
        write_verdict(zero_verdict(reason="no agent account", anchored=anchor is not None))
        return
    if not kill_agent_processes(uid):
        write_verdict(zero_verdict(reason="agent processes could not be stopped (cleanup_failed)",
                                   anchored=anchor is not None))
        return
    with open(EPISODES) as fh:
        plan = json.load(fh)["episodes"]
    specs = {}
    for item in plan:
        spec = E.check_nut_spec(item["nut"])
        specs[spec["id"]] = spec
        item["nut"] = spec["id"]
    try:
        workdir = stage_submission(uid, gid)
    except OSError as exc:
        log("staging the submission failed: {}".format(type(exc).__name__))
        workdir = None
    if workdir is None or not os.path.isfile(os.path.join(workdir, "run.sh")):
        write_verdict(zero_verdict(reason="no usable submission", anchored=anchor is not None))
        return
    pristine = os.path.join(tempfile.mkdtemp(prefix="keyed-pristine-"), "submission")   # root-only (umask 077)
    try:
        shutil.copytree(workdir, pristine, symlinks=True)
        os.chmod(pristine, 0o700)
    except OSError as exc:
        log("copying the submission failed: {}".format(type(exc).__name__))
        write_verdict(zero_verdict(reason="no usable submission", anchored=anchor is not None))
        return
    baseline = _agent_entries(uid, os.path.dirname(workdir))      # before any controller has run
    with_gt = oracle_access(workdir)
    if with_gt:
        log("reference-solution token present: observations carry _gt")
    egress_mode = "seccomp" if egress_self_test(uid, gid, int(config.get("solver_nproc_limit", 0))) else "detect"
    log("egress guard: " + egress_mode)
    try:
        (scores, aborted, faults, per_nut, per_stage, egress_hits, budget_exhausted,
         lock, progress) = run_episodes(config, rng, uid, egress_mode, t_start, specs, plan, workdir, with_gt,
                              pristine=pristine, gid=gid, baseline=baseline)
    except (SolverMissing, SolverAborted) as exc:
        kill_agent_processes(uid)
        write_verdict(zero_verdict(reason=str(exc)[:200], anchored=anchor is not None))
        return
    cleanup_ok = kill_agent_processes(uid)
    measured = sum(scores) / len(scores) if scores else 0.0
    if egress_hits or not cleanup_ok or not math.isfinite(measured) or measured < 0:
        measured = 0.0
    extra = dict(lock.rewards(),
                 episodes_scored=float(len(scores) - aborted),
                 episodes_aborted=float(aborted),
                 sim_faults=float(faults),
                 egress_guard_seccomp=1.0 if egress_mode == "seccomp" else 0.0,
                 egress_detected=float(egress_hits),
                 cleanup_failed=0.0 if cleanup_ok else 1.0,
                 budget_exhausted=1.0 if budget_exhausted else 0.0,
                 probe_lock_ok=1.0 if lock.ok else 0.0,
                 reference_token=1.0 if with_gt else 0.0,
                 mean_progress=sum(progress) / len(progress) if progress else 0.0)
    for nut, vals in sorted(per_nut.items()):
        extra["score_" + nut] = sum(vals) / len(vals)
    for key, vals in sorted(per_stage.items()):
        extra["stage_" + key] = sum(vals) / len(vals)
    verdict = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5), higher_is_better=True,
                            metric_name="assembly_success_rate", locks_ok=True, extra=extra,
                            full_at=meta.get("full_at", 1.0))
    write_verdict(verdict)
    log("measured={:.5f} anchor={} ratio={:.3f} reward={:.3f} lock={} aborted={} faults={} egress={}".format(
        measured, anchor, verdict["metrics"]["ratio"], verdict["rewards"]["reward"], lock.ok, aborted, faults,
        egress_mode))


if __name__ == "__main__":
    main()
