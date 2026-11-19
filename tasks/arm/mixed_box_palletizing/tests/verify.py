"""The grader of mixed_box_palletizing. Runs as root in the verify stage, in a SEPARATE verifier
sandbox: the engine restores only the declared artifact /home/user/submission; the agent, its
processes and the lab are gone.

It owns the simulator (pallet_cell.py, byte-identical to the file the agent's lab ran), the hidden
catalogue of cartons and the hidden orders (hidden/), and drives the submitted controller over the
kit's stdio wire at the agent's UID. The metric is the fraction of the orders that ship complete:
an order counts when every one of its cartons ships (PalletCell.finish). Nothing the controller
prints is a scoring input.

Order of events, and why:

1. Root's PATH directories are reclaimed (robotics_grader.stage.secure_exec_path) and the agent's
   account is cleared (robotics_grader.clear_agent_account: the image's desktop session is taken
   down for good; every process of the account is killed). A sandbox in which a process of the
   account is still running after that is not graded (the grader fails: a task error, no reward).
2. The restored submission is handed to the agent account in place (ownership changes never follow
   a link; size and file count are capped).
3. The oracle token is read: a submission whose .oracle_token hashes to
   hidden/oracle_token.sha256 is the reference, and only then does every observation also carry
   `truth` (PalletCell.truth). This happens BEFORE any code of the submission runs, and is decided
   once: nothing the controller does later (writing the file, fetching it) changes it.
4. The network guard is proven: a probe started through nonet_exec.py (a seccomp filter under which
   only AF_UNIX sockets can be created) must fail to open an internet socket. Where the kernel does
   not take the filter (`egress_guard_seccomp` 0 in the metrics), the controller is started with
   `runuser` instead and a watcher thread reads /proc/net every 10 ms; one internet socket of the
   agent account that is not bound to the loopback zeroes the whole run, also when the hard
   deadline writes the verdict. That detects, it does not prevent, and it cannot see everything (a
   socket that lives a few milliseconds, a protocol /proc/net does not list): with
   `require_seccomp` true in grader_config.json the grader refuses to grade without the filter (a
   task error) instead, which is the setting for a host whose kernel takes the filter.
5. Only now is `bash run.sh` started (GuardedSolver), through the launcher, at the agent's UID,
   with a whitelist environment.

Time. Nothing of the grading machine's own speed is charged. Every order has a controller budget
(grader_config.json controller_budget_s): the time the grader waits for the controller (from the
moment a request is ready to be written until its reply is read: the start to READY, the reset
reply, every action, both requests of a repeated `t`) plus a fixed charge per camera image the
controller's actions ask for (image_charge_s). The simulator's and the renderer's own time are not
charged. When the budget is used up the order ends at that step (the action that used it up is
not applied) and is scored on the state it reached. The budget bounds the controller's latency,
not its computing: what it computes between requests is its own business and slows only the run.

Three wall-clock limits stand behind the budgets, none of which an honest run meets:
wall_cap_s (checked before every step: the order under way when it is reached and the ones after
it score 0, `wall_cap_hit`), hard_deadline_s (a thread started before any submission code: it
writes the verdict from the orders finished so far and ends the process, `hard_deadline_hit`;
this is what turns a starved or held-up grader into a score instead of a stage timeout), and the
stage's own timeout above both (task.toml).

No write to the controller can block: its stdin is non-blocking, every request's deadline is
fixed before the request is written (the kit starts it after the write, and one observation with
images is megabytes), and a controller that has not taken a request by then is killed (its
process group and every process of the account); the order aborts like any late reply. The same
holds for the closing message. Reply lines are capped at 64 KiB. A controller found dead at the
start of an order is started again before it, like one that broke the protocol.

An exception inside an order that is not the controller's protocol failing (the simulator, memory,
threads) costs that order and is counted in `grader_errors`; the run goes on. What a submission
can still do is deny itself a verdict (fill the disk the result is written to): no path gives it
a score it did not earn.

Determinism: the orders run in the file's sequence with their fixed seeds, and the match lock
draws from one stream per order (probe_stream: the kit's seed_rng keyed by a BLAKE2b hash of a
domain tag, the order's index and its hidden seed), so which steps are probed does not depend on
the clock, on earlier orders or on the process. A deterministic controller that stays inside its
budgets gets one result.

Match lock: at about 1 % of the steps the same step index is asked twice, once with the real
observation and once with a copy whose joint readings (and the pad position that follows from
them) and force reading are slightly perturbed, in a hidden order; only the reply to the real
observation is applied. A pair in which both replies command no motion is not counted. If fewer
than `probe_pass_fraction` of the counted pairs differ, the run scores 0.

"""

import hashlib
import json
import math
import os
import pwd
import select
import signal
import stat
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pallet_cell as pc  # noqa: E402
from robotics_grader import (  # noqa: E402
    ProbeLock,
    Solver,
    SolverAborted,
    SolverMissing,
    agent_user,
    clear_agent_account,
    ratio_rewards,
    read_anchor,
    seed_rng,
    stage_config,
    write_verdict,
    zero_verdict,
)
from robotics_grader.stage import fail, secure_exec_path  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
NONET = os.path.join(HERE, "nonet_exec.py")
TOKEN_SHA = os.path.join(HERE, "hidden", "oracle_token.sha256")
MAX_SUBMISSION_BYTES = 200 * 1024 * 1024
MAX_SUBMISSION_FILES = 5000
MAX_LINE = 65536            # a reply is one short JSON object
PROBE_TAG = b"mixed_box_palletizing verify probe schedule 1"    # probe_stream's domain tag
SOLVER_ENV = ("PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin", "HOME=/home/user", "LANG=C.UTF-8")


def log(message):
    print("[grader] " + message, file=sys.stderr, flush=True)


# ------------------------------------------------------------------ the agent account

def _agent_ids():
    entry = pwd.getpwnam(agent_user())
    return entry.pw_uid, entry.pw_gid


def _agent_pids(uid):
    """PIDs of the account's processes that still run code (a zombie only waits to be reaped)."""
    pids = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open("/proc/{}/status".format(name)) as handle:
                zombie = False
                for line in handle:
                    if line.startswith("State:"):
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


def stage_submission(uid, gid):
    """Make the restored submission the agent account's own folder, in place at ~user/submission:
    the folder the instruction names, where run.sh is started and where the controller may write.
    Ownership changes never follow a symlink (lchown, a walk that does not follow links); size and
    file count are capped. Returns the path, or None when there is no usable submission."""
    path = os.path.join(os.path.expanduser("~" + agent_user()), "submission")
    if os.path.islink(path) or not os.path.isdir(path):
        return None
    total = files = 0
    for folder, dirs, names in os.walk(path, followlinks=False):
        for name in names:
            files += 1
            item = os.path.join(folder, name)
            if not os.path.islink(item):
                try:
                    total += os.lstat(item).st_size
                except OSError:
                    pass
        if files > MAX_SUBMISSION_FILES or total > MAX_SUBMISSION_BYTES:
            return None
    for folder, dirs, names in os.walk(path, followlinks=False):
        for name in dirs + names:
            os.lchown(os.path.join(folder, name), uid, gid)
    os.lchown(path, uid, gid)
    os.chmod(path, 0o700)
    return path


def oracle_token_ok(workdir):
    """True when the submission's .oracle_token (a regular file, not followed if a link, at most
    256 bytes) hashes to the recorded value. Called once, before any code of the submission runs."""
    path = os.path.join(workdir, ".oracle_token")
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except OSError:
        return False
    try:
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            return False
        data = os.read(fd, 257)
    except OSError:
        return False
    finally:
        os.close(fd)
    if len(data) > 256:
        return False
    with open(TOKEN_SHA) as handle:
        want = handle.read().strip()
    return hashlib.sha256(data.strip()).hexdigest() == want


# ------------------------------------------------------------------ the network guard

def egress_self_test(uid, gid):
    """True when a probe started through the launcher cannot create an internet socket (and can
    still create a UNIX one)."""
    probe = ("import socket,sys\n"
             "bad=0\n"
             "for fam in (socket.AF_INET, socket.AF_INET6, 16, 17, 40):\n"
             "    try:\n"
             "        socket.socket(fam, socket.SOCK_DGRAM).close(); bad=1\n"
             "    except OSError:\n"
             "        pass\n"
             "try:\n"
             "    socket.socket(socket.AF_UNIX, socket.SOCK_STREAM).close()\n"
             "except OSError:\n"
             "    bad=1\n"
             "sys.exit(3 if bad else 0)\n")
    try:
        done = subprocess.run([sys.executable, "-I", NONET, "--uid", str(uid), "--gid", str(gid), "--cwd", "/",
                               "--", sys.executable, "-I", "-c", probe],
                              stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                              timeout=60, env={"PATH": "/usr/bin:/bin"})
        return done.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def _loopback(address):
    """Whether a /proc/net address field (hex, as the kernel prints it) is a loopback address."""
    host = address.rsplit(":", 1)[0].upper()
    if len(host) == 8:                                  # IPv4, bytes in network order read as a little-endian word
        return host.endswith("7F")
    if host == "00000000000000000000000001000000":      # ::1
        return True
    return host.startswith("0000000000000000FFFF0000") and host.endswith("7F")     # ::ffff:127.x.x.x


def agent_inet_sockets(uid):
    """The number of the account's internet sockets (TCP, UDP, raw, ICMP; connected or not) that
    are not bound to a loopback address. A socket bound to the loopback can only reach this
    machine; any other one is counted, whether or not it has a peer yet."""
    count = 0
    for name in ("tcp", "tcp6", "udp", "udp6", "udplite", "udplite6", "raw", "raw6", "icmp", "icmp6"):
        try:
            with open("/proc/net/" + name) as handle:
                next(handle, None)
                for line in handle:
                    parts = line.split()
                    if len(parts) < 8 or int(parts[7]) != uid:
                        continue
                    if not _loopback(parts[1]):
                        count += 1
        except (OSError, ValueError, StopIteration):
            continue
    for name, column in (("sctp/eps", 6), ("sctp/assocs", 9)):      # SCTP lists its sockets apart; any one counts
        try:
            with open("/proc/net/" + name) as handle:
                next(handle, None)
                for line in handle:
                    parts = line.split()
                    if len(parts) > column and parts[column].isdigit() and int(parts[column]) == uid:
                        count += 1
        except (OSError, ValueError, StopIteration):
            continue
    return count


class EgressWatch(object):
    """Where the socket filter cannot be installed: a thread that looks for an internet socket of
    the agent account every `period` seconds for as long as the controller may run. It detects,
    it does not prevent: grader_config.json `require_seccomp` true refuses to grade without the
    filter instead."""

    def __init__(self, uid, period=0.01):
        self.uid, self.period = uid, period
        self.hits = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                if agent_inet_sockets(self.uid):
                    self.hits += 1
            except Exception:  # noqa: BLE001 - the watch must outlive anything /proc does
                pass
            self._stop.wait(self.period)

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=5)


# ------------------------------------------------------------------ the wire

class GuardedSolver(Solver):
    """The kit's Solver, started through the no-network launcher, with a reply-line cap, with
    writes that cannot block (every request has its deadline fixed BEFORE it is written, the pipe
    is non-blocking, and a controller that has not taken a request by its deadline is killed),
    and with a clock of the time spent waiting for the controller."""

    guard = True        # False: the socket filter is not available; `runuser` and EgressWatch instead
    uid = None
    gid = None

    def __init__(self, *args, **kwargs):
        self.waited = 0.0           # s the grader has waited for the controller, all requests
        started = time.monotonic()
        try:
            Solver.__init__(self, *args, **kwargs)
        finally:
            self.waited += time.monotonic() - started

    def _spawn(self):
        if self.guard:
            command = [sys.executable, "-I", NONET, "--uid", str(self.uid), "--gid", str(self.gid),
                       "--cwd", self.submission, "--", "bash", self.entry]
        else:
            command = ["/usr/sbin/runuser", "-u", agent_user(), "--", "/usr/bin/env", "-i"] + list(SOLVER_ENV) + [
                "bash", self.entry]
        try:
            proc = subprocess.Popen(command, cwd=self.submission, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL, universal_newlines=True, bufsize=1,
                                    start_new_session=True, env={"PATH": "/usr/bin:/bin"})
            os.set_blocking(proc.stdin.fileno(), False)
            return proc
        except (OSError, ValueError) as exc:
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
                break               # an oversize reply: treated as a closed wire (the order aborts)
            lines.put(line)
        lines.put(None)

    def _restart(self):
        started = time.monotonic()
        try:
            Solver._restart(self)
        finally:
            self.waited += time.monotonic() - started

    def _kill(self):
        """Kill the controller: its process group, and every process of the account (one that left
        the group may hold the pipe)."""
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except OSError:
            pass
        if self.uid is not None and os.geteuid() == 0:
            kill_agent_processes(self.uid, rounds=8, pause=0.05)

    def _stop(self, grace):
        Solver._stop(self, grace)
        if self.uid is not None and os.geteuid() == 0:
            kill_agent_processes(self.uid, rounds=8, pause=0.05)

    def _send(self, payload, deadline):
        """Write `payload` to the controller's stdin without ever blocking past `deadline`."""
        data = payload.encode("utf-8")
        try:
            fd = self.proc.stdin.fileno()
        except (OSError, ValueError):
            raise SolverAborted("solver stdin closed")
        sent = 0
        while sent < len(data):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._kill()
                raise SolverAborted("solver did not take its request within its {:g}s step deadline".format(
                    self.step_timeout))
            try:
                _, writable, _ = select.select([], [fd], [], min(remaining, 1.0))
                if writable:
                    sent += os.write(fd, data[sent:sent + (1 << 16)])
            except BlockingIOError:
                continue
            except (OSError, ValueError):
                raise SolverAborted("solver stdin closed")

    def _exchange(self, message):
        payload = json.dumps(message) + "\n"        # built before the clock starts: the grader's own time
        started = time.monotonic()
        deadline = started + self.step_timeout
        try:
            self._send(payload, deadline)
            for _ in range(1000):
                line = self._readline(deadline, self.step_timeout).strip()
                if not line:
                    continue
                try:
                    reply = json.loads(line)
                except (ValueError, RecursionError):
                    raise SolverAborted("solver wrote non-JSON on the wire: {!r}".format(line[:120]))
                if isinstance(reply, dict):
                    return reply
                raise SolverAborted("solver reply is not an object")
            raise SolverAborted("solver wrote 1000 blank lines to one request without a reply")
        finally:
            self.waited += time.monotonic() - started

    def reset(self):
        """The kit's reset; a controller that died since its last reply is started again first,
        like one that broke the protocol (the kit alone would find out only by losing this order)."""
        if not self._dead and not self._broken and self.proc is not None and self.proc.poll() is not None:
            self._broken = True
        Solver.reset(self)

    def close(self):
        """Ask the controller to exit (bounded, never blocking), then stop everything it started."""
        if self._closed:
            return
        self._closed = True
        if self.proc is None:
            return
        if self.proc.poll() is None:
            try:
                self._send(json.dumps({"type": "close"}) + "\n", time.monotonic() + 5.0)
            except SolverAborted:
                pass
        self._stop(grace=10)


def probe_stream(index, seed):
    """The match lock's stream for order `index` of the set, whose hidden seed is `seed`: the kit's
    seed_rng keyed by BLAKE2b over PROBE_TAG, the index and the seed, so a function of the order
    alone. The cell draws from the seed only through numpy's SeedSequence; this hash shares
    nothing with it. PROBE_TAG is a domain separator, not a secret: the seed is the secret."""
    digest = hashlib.blake2b(b"%s|%d|%d" % (PROBE_TAG, int(index), int(seed)), digest_size=32).digest()
    return seed_rng(entropy=int.from_bytes(digest, "little"))


def _action(reply):
    """The controller's reply as an action (pallet_cell.parse_action, the lab's rule too), or a
    protocol error that aborts the order."""
    try:
        return pc.parse_action(reply)
    except (ValueError, TypeError, OverflowError, RecursionError):
        raise SolverAborted("malformed action: {!r}".format(reply)[:160])


def _perturbed(obs, rng):
    """A copy of the observation with slightly different joint readings (and the pad pose that
    follows from them) and force reading. Everything else, the images included, is the same."""
    copy = dict(obs)
    q = [v + rng.uniform(-0.001, 0.001) for v in obs["q"]]
    xyz, yaw = pc.fk(q)
    copy["q"] = [round(v, 6) for v in q]
    copy["tcp"] = [round(float(v), 6) for v in (xyz[0], xyz[1], xyz[2], yaw)]
    wrench = list(obs["wrench"])
    for i in range(3):
        wrench[i] = round(wrench[i] + rng.uniform(-0.3, 0.3), 4)
    copy["wrench"] = wrench
    return copy


# ------------------------------------------------------------------ the run

class Run(object):
    """What the run has established so far. The verdict is written from it exactly once: by main
    when the orders are through, or by the hard deadline (HardDeadline) when the grader itself is
    held up; an order that has not finished by then scores 0."""

    def __init__(self, names, lock, anchor, meta, config, truth, secured, t_start):
        self.scores = {name: 0.0 for name in names}
        self.volume = {name: 0.0 for name in names}
        self.notes = {}
        self.lock = lock
        self.anchor, self.meta, self.config = anchor, meta, config
        self.truth, self.secured, self.t_start = truth, secured, t_start
        self.aborted = self.budget_cuts = self.images = self.grader_errors = 0
        self.wall_cap_hit = False
        self.controller_s = []
        self.watch = None               # the EgressWatch, when the socket filter is not available
        self.cleanup_ok = True
        self._written = False
        self._guard = threading.Lock()

    def note(self, name, text):
        self.notes[name] = "{}: {}".format(name, text)
        log(self.notes[name])

    def write(self, cut_off=False):
        """Write the verdict (once; a second caller returns False)."""
        with self._guard:
            if self._written:
                return False
            scores, volume = dict(self.scores), dict(self.volume)
            measured = sum(scores.values()) / len(scores) if scores else 0.0
            egress_hits = self.watch.hits if self.watch is not None else 0     # read now, whoever writes
            if cut_off:
                cleanup_ok = True           # not established: the process ends right after this
            else:
                cleanup_ok = self.cleanup_ok
            if egress_hits or not cleanup_ok or not math.isfinite(measured) or measured < 0:
                measured = 0.0
            spent = list(self.controller_s)
            extra = dict(self.lock.rewards(),
                         episodes=float(len(scores)),
                         episodes_aborted=float(self.aborted),
                         oracle_token=1.0 if self.truth else 0.0,
                         shipped_volume_fraction=sum(volume.values()) / len(volume) if volume else 0.0,
                         egress_guard_seccomp=1.0 if GuardedSolver.guard else 0.0,
                         egress_detected=float(egress_hits),
                         cleanup_failed=0.0 if cleanup_ok else 1.0,
                         controller_s_max=max(spent) if spent else 0.0,
                         controller_s_mean=sum(spent) / len(spent) if spent else 0.0,
                         controller_budget_cuts=float(self.budget_cuts),
                         images_requested=float(self.images),
                         wall_cap_hit=1.0 if self.wall_cap_hit else 0.0,
                         hard_deadline_hit=1.0 if cut_off else 0.0,
                         grader_errors=float(self.grader_errors),
                         grading_wall_s=time.monotonic() - self.t_start,
                         **self.secured)
            verdict = ratio_rewards(
                measured,
                self.anchor,
                cap=self.config.get("scoring_cap", 1.5),
                higher_is_better=self.meta.get("direction", "higher") == "higher",
                metric_name="complete_order_fraction",
                locks_ok=self.lock.ok,
                extra=extra,
                full_at=self.meta.get("full_at", 1.0),
            )
            names = sorted(scores)
            flags = ""
            if egress_hits:
                flags += "; an internet socket of the controller was seen: the run scores 0"
            if not cleanup_ok:
                flags += "; the controller's processes could not be stopped: the run scores 0"
            if cut_off:
                flags += "; the grader was cut off at its hard deadline: the orders not finished by then score 0"
            text = "complete orders {:.4f} of {} orders ({}); match lock {} ({} of {} counted pairs differed){}\n{}".format(
                measured, len(scores), ", ".join("{} {:.0f}".format(n, scores[n]) for n in names),
                "ok" if self.lock.ok else "FAILED", self.lock.passed, self.lock.total, flags,
                "\n".join(self.notes.get(n, "{}: not finished".format(n)) for n in names))
            write_verdict(verdict, diagnostic=text)
            self._written = True            # only once it is on disk: a failed write leaves the other writer its turn
            log("measured={:.4f} anchor={} reward={:.3f} lock={} aborted={} guard={} wall={:.0f}s".format(
                measured, self.anchor, verdict["rewards"]["reward"], self.lock.ok, self.aborted,
                "seccomp" if GuardedSolver.guard else "watch", time.monotonic() - self.t_start))
            return True


class HardDeadline(threading.Thread):
    """The grader's last resort, started before any code of the submission runs: if the run is
    still going `after` seconds from its start (the grader's own work was starved or held up
    where no per-request deadline applies), write the verdict from what is established and end
    the process. The stage's timeout would otherwise end it with no verdict at all."""

    def __init__(self, state, uid, after):
        threading.Thread.__init__(self, daemon=True)
        self.state, self.uid, self.after = state, uid, float(after)

    def run(self):
        while True:
            left = self.state.t_start + self.after - time.monotonic()
            if left <= 0:
                break
            time.sleep(min(left, 5.0))
        log("hard deadline ({:.0f} s) reached: writing the verdict".format(self.after))
        try:
            if self.state.write(cut_off=True):
                if os.geteuid() == 0:
                    kill_agent_processes(self.uid, rounds=8, pause=0.05)
                sys.stderr.flush()
                os._exit(0)
        except BaseException as exc:  # noqa: BLE001 - nothing may keep the process alive here
            log("the hard deadline could not write the verdict: {}".format(type(exc).__name__))
            sys.stderr.flush()
            os._exit(1)


def run(state, config, workdir, episodes, catalog):
    budget = float(config.get("controller_budget_s", 675))
    image_charge = float(config.get("image_charge_s", 0.1))
    wall_cap = float(config.get("wall_cap_s", 13200))
    lock, truth, t_start = state.lock, state.truth, state.t_start
    cell = pc.PalletCell(catalog)

    def seen(observation):
        if truth:
            observation["truth"] = cell.truth()
        return observation

    solver = None
    try:
        # The first start: a controller that does not come up (no READY) is the whole run's zero (main).
        solver = GuardedSolver(step_timeout=config.get("step_timeout_s", 30), submission=workdir)
        for index, episode in enumerate(episodes):
            name = episode["name"]
            if time.monotonic() - t_start > wall_cap:
                state.wall_cap_hit = True
                state.note(name, "not run (the grading run's wall-clock cap was reached)")
                continue
            spent, cut = 0.0, False         # this order's controller time: waits plus image charges
            mark = solver.waited if index else 0.0      # the first order is charged the start too
            talking = False                 # inside a request to the controller
            try:
                rng = probe_stream(index, episode["seed"])
                lock.rng = rng
                obs = seen(cell.reset(episode["order"], episode["seed"]))
                talking = True
                solver.reset()
                talking = False
                while not cell.done:
                    if time.monotonic() - t_start > wall_cap:
                        state.wall_cap_hit = True
                        raise SolverAborted("the grading run's wall-clock cap was reached")
                    if lock.due():
                        other = _perturbed(obs, rng)
                        first_is_probe = rng.random() < 0.5
                        talking = True
                        if first_is_probe:
                            probe, real = solver.act(obs["t"], other), solver.act(obs["t"], obs)
                        else:
                            real, probe = solver.act(obs["t"], obs), solver.act(obs["t"], other)
                        talking = False
                        action, shadow = _action(real), _action(probe)
                        if any(abs(v) > 1e-9 for v in action[:4] + shadow[:4]):
                            lock.record(shadow[:4], action[:4])
                    else:
                        talking = True
                        reply = solver.act(obs["t"], obs)
                        talking = False
                        action = _action(reply)
                    asked = bin(int(round(action[5]))).count("1")
                    spent += (solver.waited - mark) + asked * image_charge
                    mark = solver.waited
                    if spent > budget:
                        cut = True
                        state.budget_cuts += 1
                        break               # the order ends here; the action that used the budget up is not applied
                    state.images += asked
                    obs = seen(cell.step(action))
                result = cell.finish()
            except SolverAborted as exc:
                spent += solver.waited - mark        # the wait that ended in the abort
                state.aborted += 1
                state.controller_s.append(spent)
                state.note(name, "aborted at step {} ({})".format(cell.t, str(exc)[:120]))
                continue
            except Exception as exc:  # noqa: BLE001 - whatever the controller drove the grader into costs the order
                if talking:
                    solver._broken = True           # the wire may be out of step: start the controller again
                spent += solver.waited - mark
                state.aborted += 1
                state.grader_errors += 1
                state.controller_s.append(spent)
                state.note(name, "aborted at step {} (grader error: {}: {})".format(
                    getattr(cell, "t", 0), type(exc).__name__, str(exc)[:100]))
                continue
            state.controller_s.append(spent)
            lost = {}
            for carton in result["cartons"]:
                if not carton["shipped"]:
                    reason = carton["reason"].split(" (")[0]
                    lost[reason] = lost.get(reason, 0) + 1
            complete = all(carton["shipped"] for carton in result["cartons"])
            state.volume[name] = float(result["shipped_volume_fraction"])
            state.note(name, "{}, volume {:.3f}, in {} steps, controller {:.1f} s{}{}".format(
                "complete" if complete else "incomplete", state.volume[name], result["steps"], spent,
                " (budget used up)" if cut else "",
                "" if not lost else "; not shipped: " + ", ".join("{} x {}".format(n, r) for r, n in sorted(lost.items()))))
            state.scores[name] = 1.0 if complete else 0.0       # last: an order counts once all of it is recorded
    finally:
        try:
            if solver is not None:
                solver.close()
        except Exception:  # noqa: BLE001 - closing a broken solver must not crash the grader
            pass
        cell.close()


def main():
    t_start = time.monotonic()
    secured = {"root_path_dirs_secured": float(len(secure_exec_path()))}     # before anything else runs
    config = stage_config()
    anchor, meta = read_anchor()
    anchored = anchor is not None
    root = os.geteuid() == 0
    try:
        uid, gid = _agent_ids()
    except KeyError:
        write_verdict(zero_verdict(reason="no agent account", anchored=anchored, extra=secured))
        return
    if root and os.environ.get("ALE_VERIFY_PYTHON"):
        clear_agent_account()
        if not kill_agent_processes(uid):
            fail("processes of the agent account could not be stopped before grading")
    try:
        workdir = stage_submission(uid, gid) if root else os.path.join(
            os.path.expanduser("~" + agent_user()), "submission")
    except OSError as exc:
        log("staging the submission failed: {}".format(type(exc).__name__))
        workdir = None
    if workdir is None or os.path.islink(os.path.join(workdir, "run.sh")) or not os.path.isfile(
            os.path.join(workdir, "run.sh")):
        write_verdict(zero_verdict(
            reason="no usable submission (no run.sh in /home/user/submission, or more than {} files or {} MB)".format(
                MAX_SUBMISSION_FILES, MAX_SUBMISSION_BYTES // (1024 * 1024)), anchored=anchored, extra=secured))
        return
    truth = oracle_token_ok(workdir)        # decided here, before any code of the submission runs
    GuardedSolver.uid, GuardedSolver.gid = uid, gid
    GuardedSolver.guard = root and egress_self_test(uid, gid)
    if root and not GuardedSolver.guard and config.get("require_seccomp", False):
        fail("the socket filter could not be installed and grader_config.json require_seccomp is true")
    with open(os.path.join(HERE, "hidden", "catalog.json")) as handle:
        catalog = json.load(handle)
    with open(os.path.join(HERE, "hidden", "episodes.json")) as handle:
        episodes = json.load(handle)
    lock = ProbeLock(None, rate=config.get("probe_rate", 0.01), pass_fraction=config.get("probe_pass_fraction", 0.1))
    state = Run([episode["name"] for episode in episodes], lock, anchor, meta, config, truth, secured, t_start)
    watch = None
    if root and not GuardedSolver.guard:
        watch = state.watch = EgressWatch(uid)      # in the state before the deadline thread can write
        watch.start()
    HardDeadline(state, uid, config.get("hard_deadline_s", 13800)).start()     # before any submission code runs
    log("oracle token: {}; network guard: {}".format(truth, "seccomp" if GuardedSolver.guard else "watch"))
    first_start_failed = None
    try:
        run(state, config, workdir, episodes, catalog)
    except (SolverMissing, SolverAborted) as exc:       # no run.sh, or no READY at the first start
        first_start_failed = str(exc)[:200]
    except Exception as exc:  # noqa: BLE001 - recorded; the orders that did not finish score 0
        state.grader_errors += 1
        log("the run ended in {}: {}".format(type(exc).__name__, str(exc)[:200]))
    state.cleanup_ok = kill_agent_processes(uid) if root else True
    if watch is not None:
        watch.stop()
    if first_start_failed is not None:
        with state._guard:
            if state._written:
                return
            state._written = True
        write_verdict(zero_verdict(reason=first_start_failed, anchored=anchored, extra=secured))
        return
    state.write()


if __name__ == "__main__":
    main()
