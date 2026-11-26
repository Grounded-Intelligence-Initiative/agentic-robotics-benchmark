"""The grader of endoscope_camera_assistance_from_video. Runs as root in the verify stage, in a fresh verifier sandbox
that holds only the agent's /home/user/submission (task.toml: separate verification).

closed_loop: the grader owns the simulator. It plays the fixed hidden set of recorded procedures (hidden/
procedures.json: the same procedures for every submission), and for every control step hands the submission's
controller one observation (the endoscope's frame, 1 to 3 control steps old as the procedure's rig says, and the
holder's four joint readings) and applies the rate command it answers. The frames are drawn by an Isaac Sim process of its own account (endosim/
render_server.py); the view is judged from the simulator's state, never from pixels (endosim/episode.py):

    useful_view_fraction = usable steps / steps, over all procedures

What keeps the number honest:

* tests/ was absent while the agent worked; the controller runs as the agent's account (robotics_grader.Solver), the
  renderer as a third account, each unable to read the others' files or memory. The controller is never given the
  procedure, only frames and joint readings.
* Nothing the controller prints is a scoring input. A controller that stops answering, answers a malformed action or
  uses up its time is charged what standing still costs: the rest of that procedure runs with the holder commanded
  to stop, and is judged like any other step.
* The match lock (robotics_grader.ProbeLock): on some steps the same step index is asked twice, once with the true
  observation and once with a copy built as an observation is (endosim/session.py `probe_copy`: the same frame
  rendered again with the instruments moved over its last step as over another step of the procedure, the joints
  unchanged), in a hidden order; only the true reply moves the holder. A controller that does not read its input
  fails the lock.
* A renderer that fails is restarted and the frame drawn again; a renderer that cannot draw what the public camera
  model says (the canary) is a task error, never a zero.

The reference submission carries a token (solution/); for it alone every observation also holds `privileged`, what a
perfect perception system would have read from the same frame.

`grade()` is also what tests/review/run_native.py calls outside the engine, with its own launchers.
"""

import hashlib
import hmac
import json
import os
import pwd
import shutil
import signal
import stat
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from endosim import model, procedure  # noqa: E402
from endosim.render_client import Renderer, RendererError  # noqa: E402
from endosim.episode import TAKEOVERS  # noqa: E402
from endosim.session import Session  # noqa: E402
from robotics_grader.stage import secure_exec_path  # noqa: E402
from robotics_grader import (  # noqa: E402
    ProbeLock,
    Solver,
    SolverAborted,
    SolverMissing,
    VerifierDeadline,
    agent_user,
    clear_agent_account,
    fail,
    ratio_rewards,
    read_anchor,
    seed_rng,
    stage_config,
    write_verdict,
    zero_verdict,
)

RENDER_USER = "endosim"
RENDER_ROOT = "/opt/endosim-verify"       # the renderer account's own folder (0700), outside /opt/ale
KIT_SHARED = "/opt/venv/lib/python3.12/site-packages/isaacsim/kit"   # Kit's cache / logs / data folders
RENDER_FILES = ("__init__.py", "model.py", "tissue.py", "looks.py", "scene.py", "post.py", "render_server.py")
NONET = os.path.join(HERE, "nonet_exec.py")   # tasks/swarm/hazard_bay_build's launcher, byte for byte (run as root)
LIVE = {"records": [], "session": None, "lock": None, "steps": 0}   # what the deadline's verdict is made of


def log(*parts):
    print("[grader]", *parts, file=sys.stderr, flush=True)


class BudgetExhausted(Exception):
    """The controller has used its whole reply-time budget."""


class TimedSolver(Solver):
    """The kit's Solver with one budget of reply time for the whole evaluation, its program started through
    `nonet_exec.py`: no_new_privs, a seccomp filter under which socket() and socketpair() work for AF_UNIX only (no
    network, as the instruction promises), then the agent's account and a fixed environment (PATH
    /opt/venv/bin:/usr/local/bin:/usr/bin:/bin, HOME /home/user, LANG C.UTF-8). Where the filter cannot be installed
    (the self-test before the first start fails) the program starts through the kit's own launcher, and the metrics
    say so (`network_filter` 0)."""

    budget_s = float("inf")
    used_s = 0.0
    requests = 0
    slowest_s = 0.0
    filtered = None

    def _spawn(self):
        cls = TimedSolver
        if cls.filtered is None:
            cls.filtered = egress_self_test()
            log("the controller's network filter: {}".format("on" if cls.filtered else "could not be installed"))
        if not cls.filtered:
            return super()._spawn()
        secure_exec_path()
        if self.clear_account:
            clear_agent_account()
        pw = pwd.getpwnam(agent_user())
        cmd = [sys.executable, "-I", NONET, "--uid", str(pw.pw_uid), "--gid", str(pw.pw_gid), "--cwd", self.submission, "--",
               "bash", self.entry]
        try:
            return subprocess.Popen(cmd, cwd=self.submission, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                    universal_newlines=True, bufsize=1, start_new_session=True, env={"PATH": "/usr/bin:/bin"})
        except OSError as exc:
            raise SolverAborted("the controller could not be started: {}".format(exc))

    def act(self, t, obs):
        cls = TimedSolver                 # one account for every subclass (a maintainer's local launcher included)
        if cls.used_s >= cls.budget_s:
            raise BudgetExhausted("the controller's reply-time budget ({:g} s) is used up".format(cls.budget_s))
        start = time.monotonic()
        try:
            return super().act(t, obs)
        finally:
            took = time.monotonic() - start
            cls.used_s += took
            cls.requests += 1
            cls.slowest_s = max(cls.slowest_s, took)


def egress_self_test():
    """True if a program started through the launcher, as the agent's account, cannot open an internet socket and can
    open a Unix one."""
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
        pw = pwd.getpwnam(agent_user())
        r = subprocess.run([sys.executable, "-I", NONET, "--uid", str(pw.pw_uid), "--gid", str(pw.pw_gid), "--cwd", "/", "--",
                            sys.executable, "-I", "-c", probe], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60,
                           env={"PATH": "/usr/bin:/bin"})
        return r.returncode == 0
    except (OSError, KeyError, subprocess.SubprocessError):
        return False


def canonical_action(action):
    """A reply's action as four finite floats (what the lock compares), or a protocol error."""
    try:
        return [float(x) for x in model.clip_command(action)]
    except (TypeError, ValueError, OverflowError):
        raise SolverAborted("malformed action: {!r}".format(action)[:160])


# ------------------------------------------------------------------------------------------- the renderer's account
def token_state(submission):
    """True when the submission carries the reference's token (hidden/oracle_token.sha256)."""
    path, want = os.path.join(submission, ".oracle_token"), os.path.join(HERE, "hidden", "oracle_token.sha256")
    try:
        if not stat.S_ISREG(os.lstat(path).st_mode) or os.path.getsize(path) > 4096:
            return False
        with open(path, "rb") as f:
            got = hashlib.sha256(f.read().strip()).hexdigest()
        with open(want) as f:
            return hmac.compare_digest(got, f.read().strip())
    except OSError:
        return False


def _clean_shm(user):
    """Remove what an account left in /dev/shm (Kit opens shared memory there under predictable names)."""
    try:
        uid = pwd.getpwnam(user).pw_uid
    except KeyError:
        return
    for e in os.scandir("/dev/shm") if os.path.isdir("/dev/shm") else []:
        try:
            if e.stat(follow_symlinks=False).st_uid != uid:
                continue
            if e.is_dir(follow_symlinks=False):
                shutil.rmtree(e.path, ignore_errors=True)
            else:
                os.remove(e.path)
        except OSError:
            pass


def _seal_shm(user):
    """Close the renderer's shared memory in /dev/shm to every other account (Kit creates it open to all)."""
    try:
        uid = pwd.getpwnam(user).pw_uid
    except KeyError:
        return
    for e in os.scandir("/dev/shm") if os.path.isdir("/dev/shm") else []:
        try:
            st = e.stat(follow_symlinks=False)
            if st.st_uid == uid and not stat.S_ISLNK(st.st_mode) and stat.S_IMODE(st.st_mode) & 0o077:
                os.chmod(e.path, stat.S_IMODE(st.st_mode) & 0o700)
        except OSError:
            pass


def _signal_account(user, sig):
    """Send `sig` to every process of `user` (the agent's account: stopped while a new renderer comes up)."""
    try:
        uid = pwd.getpwnam(user).pw_uid
    except KeyError:
        return
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            if os.stat("/proc/" + name).st_uid == uid:
                os.kill(int(name), sig)
        except OSError:
            pass


def engine_renderer(config):
    """The renderer as its own account, from a copy of the renderer's files that only that account can read."""
    try:
        pw = pwd.getpwnam(RENDER_USER)
    except KeyError:
        subprocess.run(["/usr/sbin/useradd", "--system", "--no-create-home", "--home-dir", RENDER_ROOT, "--shell",
                        "/usr/sbin/nologin", RENDER_USER], check=True)
        pw = pwd.getpwnam(RENDER_USER)
    _clean_shm(agent_user())
    shutil.rmtree(RENDER_ROOT, ignore_errors=True)
    pkg = os.path.join(RENDER_ROOT, "endosim")
    for d in (RENDER_ROOT, pkg, os.path.join(RENDER_ROOT, "home"), os.path.join(RENDER_ROOT, "tmp"), os.path.join(RENDER_ROOT, "optix")):
        os.makedirs(d, exist_ok=True)
        os.chown(d, pw.pw_uid, pw.pw_gid)
        os.chmod(d, 0o700)
    for name in RENDER_FILES:           # the scene and the image code only: no procedure generator, no episode
        dst = os.path.join(pkg, name)
        shutil.copy(os.path.join(HERE, "endosim", name), dst)
        os.chown(dst, pw.pw_uid, pw.pw_gid)
        os.chmod(dst, 0o600)
    for d in ("cache", "logs", "data"):  # Kit's own folders: the renderer account's alone while grading
        path = os.path.join(KIT_SHARED, d)
        shutil.rmtree(path, ignore_errors=True)
        os.makedirs(path, exist_ok=True)
        os.chown(path, pw.pw_uid, pw.pw_gid)
        os.chmod(path, 0o700)
    env = {"HOME": os.path.join(RENDER_ROOT, "home"), "PATH": "/opt/venv/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
           "LANG": "C.UTF-8", "OMNI_KIT_ACCEPT_EULA": "YES", "ACCEPT_EULA": "Y", "PRIVACY_CONSENT": "Y",
           "TMPDIR": os.path.join(RENDER_ROOT, "tmp"), "OPTIX_CACHE_PATH": os.path.join(RENDER_ROOT, "optix"),
           "NVIDIA_DRIVER_CAPABILITIES": os.environ.get("NVIDIA_DRIVER_CAPABILITIES", ""),
           "NVIDIA_VISIBLE_DEVICES": os.environ.get("NVIDIA_VISIBLE_DEVICES", "")}
    log_path = os.path.join(RENDER_ROOT, "renderer.log")
    # runuser sets HOME from the account's passwd entry (/var/lib/endocam, which only setup makes, in the solver's
    # sandbox; Omniverse RTX keeps its texture cache there): env puts it back to the renderer's own folder.
    prefix = ["/usr/sbin/runuser", "-u", RENDER_USER, "--", "/usr/bin/env", "HOME=" + env["HOME"]]
    renderer = Renderer(RENDER_ROOT, python="/opt/venv/bin/python3", prefix=prefix, env=env,
                        log_path=log_path, start_timeout=float(config.get("renderer_start_timeout_s", 2400)),
                        render_timeout=float(config.get("render_timeout_s", 120)), max_restarts=int(config.get("renderer_restarts", 3)),
                        cwd=os.path.join(RENDER_ROOT, "home"),
                        # Kit creates its /dev/shm entries open to all at every start: the agent's processes are
                        # stopped before a restart and go on only once the new renderer's entries are sealed
                        around_restart=(lambda: _signal_account(agent_user(), signal.SIGSTOP),
                                        lambda: (_seal_shm(RENDER_USER), _signal_account(agent_user(), signal.SIGCONT))))
    return renderer, log_path


def check_canary(result, config):
    """The renderer must draw a marker where the public camera model puts it, for both scopes."""
    tol = float(config.get("canary_max_error_px", 2.5))
    problems = []
    if not result.get("ok"):
        problems.append("the frame clock was not readable")
    for scope, r in sorted(result.get("scopes", {}).items()):
        if r["found"] < r["markers"]:
            problems.append("scope {}: {} of {} markers found".format(scope, r["found"], r["markers"]))
        elif r["max_error_px"] is None or r["max_error_px"] > tol:
            problems.append("scope {}: marker {} px from where the camera model puts it (limit {})".format(scope, r["max_error_px"], tol))
        if r.get("mean_level") is None or not 1.0 <= r["mean_level"] <= 250.0:
            problems.append("scope {}: frame level {}".format(scope, r.get("mean_level")))
    return problems


# ------------------------------------------------------------------------------------------------------ the episodes
def run_procedure(index, item, steps, solver, renderer, lock, config, privileged, state, pool="all"):
    """One hidden procedure. Returns its record; never raises for anything the controller does."""
    proc = procedure.generate(item["seed"], item.get("factors"), steps=steps, pool=pool)
    session = Session(proc, renderer, privileged=privileged)          # the procedure's own video latency (its rig)
    LIVE["session"] = session
    rng = lock.rng
    reason = None
    try:
        if state["budget_exhausted"]:
            raise BudgetExhausted("the controller's reply-time budget was used up in an earlier procedure")
        solver.reset()
        while not session.done:
            obs = session.observation()
            action = None
            if lock.due():
                try:
                    copy = session.probe_copy(obs, rng)       # None: no instrument in, or no step like this one
                except RendererError as exc:        # the renderer failed: no probe on this step
                    log("procedure {} step {}: no probe copy ({})".format(index, session.t, str(exc)[:120]))
                    copy = None
                if copy is not None:
                    action = lock.probe(solver, session.t, obs, perturbed=copy)
            if action is None:
                action = solver.act(session.t, obs)
            session.step(canonical_action(action))
            if session.episode.taken_over:
                break                                  # the surgeon took the camera back: no more requests
    except BudgetExhausted as exc:
        state["budget_exhausted"] = True
        reason = str(exc)
    except SolverAborted as exc:
        reason = str(exc)
    if reason is not None:
        at = session.t
        session.finish_idle()                      # the rest of the procedure with the holder standing still
        log("procedure {} aborted at step {} ({}); the remaining {} steps ran with the holder stopped".format(index, at, reason[:200], steps - at))
    elif session.episode.taken_over:
        at = session.t
        session.finish_idle()                      # every step after a takeover is not usable
        log("procedure {}: the surgeon took the camera back at step {} ({}); the remaining {} steps are not usable".format(
            index, at, session.episode.takeover, steps - at))
    rec = session.episode.summary()
    rec.update(index=index, aborted=reason is not None, abort_reason=reason, facts=proc.facts())
    LIVE["session"] = None
    log("procedure {} ({} scope {}): useful_view_fraction {:.4f} | failed: lens {:.3f} zone {:.3f} zoom {:.3f} horizon {:.3f} motion {:.3f} | "
        "lens contacts {} cleanings {} | takeover {} at {} | critical actions {} (least seen {})".format(
            index, proc.kind, proc.scope, rec["useful_view_fraction"], rec["failed_lens"], rec["failed_zone"], rec["failed_zoom"],
            rec["failed_horizon"], rec["failed_motion"], rec["lens_contacts"], rec["lens_cleanings"], rec["takeover"], rec["takeover_step"],
            rec["critical_actions"], rec["critical_min_seen"]))
    return rec


def grade(submission, config, make_solver, make_renderer, only=None, set_file=None, pool="all"):
    """Play the hidden set against the submission's controller. Returns (records, lock, extra metrics, diagnostic).
    Raises SolverMissing / SolverAborted when there is no controller to drive, RendererError when the environment
    cannot render. (`set_file` and `pool` are for a maintainer's run on the development set only.)"""
    with open(set_file or os.path.join(HERE, "hidden", "procedures.json")) as f:
        hidden = json.load(f)
    items = hidden["procedures"] if only is None else [hidden["procedures"][i] for i in only]
    steps = int(hidden["steps"])
    if not os.path.isfile(os.path.join(submission, "run.sh")):
        raise SolverMissing("no submission at {}/run.sh".format(submission))
    privileged = token_state(submission)
    t0 = time.time()
    renderer, render_log = make_renderer(config)
    started = dict(renderer.started or {})
    log("renderer up in {:.0f} s".format(time.time() - t0))
    try:
        t1 = time.time()
        canary = renderer.canary()
        problems = check_canary(canary, config)
        log("render canary ({:.0f} s): {}".format(time.time() - t1, json.dumps(canary)[:600]))
        if problems:
            # one fresh renderer and a second canary before the sandbox is declared unable to render
            log("render canary failed ({}); restarting the renderer once".format("; ".join(problems)))
            renderer.restart("the render canary failed: " + "; ".join(problems))
            canary = renderer.canary()
            problems = check_canary(canary, config)
            log("render canary, second try: {}".format(json.dumps(canary)[:600]))
        if problems:
            raise RendererError("the render canary failed: " + "; ".join(problems))
        rng = seed_rng()
        lock = ProbeLock(rng, rate=float(config.get("probe_rate", 0.06)), pass_fraction=float(config.get("probe_pass_fraction", 0.1)),
                         canonical=canonical_action)
        TimedSolver.budget_s = float(config.get("controller_budget_s_per_step", 0.25)) * steps * len(items)
        TimedSolver.used_s, TimedSolver.requests, TimedSolver.slowest_s = 0.0, 0, 0.0
        solver = make_solver(float(config.get("step_timeout_s", 10)))          # raises SolverMissing / SolverAborted
        state = {"budget_exhausted": False}
        records = []
        LIVE.update(records=records, lock=lock, steps=steps * len(items))
        try:
            for i, item in enumerate(items):
                records.append(run_procedure(i, item, steps, solver, renderer, lock, config, privileged, state, pool))
        finally:
            solver.close()
        info = {}
        try:
            info = renderer.info()
        except RendererError:
            pass
    except RendererError:
        tail = ""
        try:
            with open(render_log, "rb") as f:
                f.seek(max(0, os.path.getsize(render_log) - 3000))
                tail = f.read().decode("utf-8", "replace")
        except OSError:
            pass
        log("renderer log tail:\n" + tail)
        raise
    finally:
        renderer.close()
    n = max(1, renderer.frames)
    extra = dict(lock.rewards())
    extra.update(
        procedures=float(len(records)), steps_per_procedure=float(steps),
        episodes_aborted=float(sum(r["aborted"] for r in records)),
        budget_exhausted=float(state["budget_exhausted"]), privileged=float(privileged),
        controller_seconds=round(TimedSolver.used_s, 1), controller_requests=float(TimedSolver.requests),
        controller_slowest_reply_s=round(TimedSolver.slowest_s, 3), solver_restarts=float(solver.restarts),
        network_filter=float(bool(TimedSolver.filtered)),
        lens_contacts=float(sum(r["lens_contacts"] for r in records)), lens_cleanings=float(sum(r["lens_cleanings"] for r in records)),
        lens_smears=float(sum(r["lens_smears"] for r in records)),
        takeovers=float(sum(r["takeover"] is not None for r in records)),
        critical_actions=float(sum(r["critical_actions"] for r in records)),
        renderer_restarts=float(renderer.restarts), render_ms_per_frame=round(renderer.render_ms / n, 1),
        extra_renders_per_frame=round(renderer.extra_renders / n, 3), renderer_start_s=float(started.get("start_s", 0.0)),
        wall_seconds=round(time.time() - t0, 1),
    )
    for c in ("lens", "zone", "zoom", "horizon", "motion", "takeover"):
        extra["failed_" + c] = sum(r["failed_" + c] for r in records) / max(1, len(records))
    for why in TAKEOVERS:
        extra["takeover_" + why] = float(sum(r["takeover"] == why for r in records))
    for r in records:
        extra["procedure_{:02d}".format(r["index"])] = round(r["useful_view_fraction"], 4)
    lines = ["hidden set: {} procedures of {} steps; renderer {}".format(len(records), steps, info.get("gpu", "?"))]
    for r in records:
        f = r["facts"]
        lines.append("procedure {:02d} {:<12} scope {:>2} exchanges {} assistant {} smoke {}: {:.4f}{}{}".format(
            r["index"], f["kind"], f["scope"], f["exchanges"], int(f["assistant"]), int(f["smoke"]), r["useful_view_fraction"],
            " (aborted: {})".format(r["abort_reason"][:120]) if r["aborted"] else "",
            " (the surgeon took the camera back at step {}: {})".format(r["takeover_step"], r["takeover"]) if r["takeover"] else ""))
    return records, lock, extra, "\n".join(lines)


def score(records, lock, extra, diagnostic, config, anchor, meta, path=None):
    usable = sum(r["usable_steps"] for r in records)
    steps = sum(r["steps"] for r in records)
    measured = usable / max(1, steps)
    verdict = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5),
                            higher_is_better=meta.get("direction", "higher") == "higher", metric_name=meta.get("metric", "useful_view_fraction"),
                            locks_ok=lock.ok, extra=extra, full_at=meta.get("full_at", 1.0))
    if not lock.ok:
        diagnostic += "\nmatch lock failed: {} of {} probes changed the action".format(lock.passed, lock.total)
    write_verdict(verdict, path=path, diagnostic=diagnostic)
    log("useful_view_fraction {:.4f} anchor {} ratio {:.3f} reward {:.3f} lock {} ({}/{}) aborted {}".format(
        measured, anchor, verdict["metrics"]["ratio"], verdict["rewards"]["reward"], lock.ok, lock.passed, lock.total,
        int(extra["episodes_aborted"])))
    return verdict


def deadline_verdict(config, anchor, meta):
    """The verdict when grading runs out of time ([verifier] timeout_sec): the steps judged so far count as judged,
    every step not reached counts as unusable, and the lock is judged on the pairs it has seen."""
    def write(elapsed):
        done = list(LIVE["records"])
        usable = sum(r["usable_steps"] for r in done)
        if LIVE["session"] is not None:
            usable += LIVE["session"].episode.usable
        lock = LIVE["lock"]
        measured = usable / max(1, LIVE["steps"])
        verdict = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5),
                                higher_is_better=meta.get("direction", "higher") == "higher",
                                metric_name=meta.get("metric", "useful_view_fraction"), locks_ok=bool(lock is not None and lock.ok),
                                extra={"deadline_expired": 1.0, "procedures_finished": float(len(done)), "elapsed_s": round(elapsed, 1)},
                                full_at=meta.get("full_at", 1.0))
        write_verdict(verdict, diagnostic="grading ran out of time after {:.0f} s: {} procedures finished; every step not reached "
                                          "counts as unusable".format(elapsed, len(done)))
    return write


def main():
    config = stage_config()
    anchor, meta = read_anchor()
    submission = os.path.join(os.environ.get("ALE_HOME", "/home/user"), "submission")
    VerifierDeadline(float(config.get("verifier_deadline_s", 8400)), deadline_verdict(config, anchor, meta)).arm()
    try:
        records, lock, extra, diagnostic = grade(submission, config, lambda timeout: TimedSolver(step_timeout=timeout), engine_renderer)
    except (SolverMissing, SolverAborted) as exc:
        # no controller to drive (an untouched sandbox, a run.sh that never says READY): an honest zero
        write_verdict(zero_verdict(reason=str(exc), anchored=anchor is not None))
        return
    except RendererError as exc:
        fail("the grading environment could not render: {}".format(str(exc)[:1500]))
    score(records, lock, extra, diagnostic, config, anchor, meta)


if __name__ == "__main__":
    main()
