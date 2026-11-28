"""The controller as a deprivileged child process (grader side), on the runner helpers of `arm.assembly_cell.runner`
(prepare_run_dir, child_env, wrap_limits, runner_account, kill_uid_processes):

    r = Runner(program_dir, run_root, client_path)   # client_path = motion_ctl/client.py (copied alone as mc_client.py:
                                                     # the child never sees the grader's code)
    r.start()                                        -> (ok, info): import + Controller(workdir) within IMPORT_S;
                                                        as root a controller without the no-network filter, and a
                                                        client that ends or fails before its first line (no code
                                                        of the controller has run), are the task's failure
                                                        (RuntimeError) before any setup
    r.call(op, payload, cpu_s, wall_s)               -> (status, reply, cpu_used_s, wall_s)
                                                        status "ok" | "error" (the controller raised) | "cpu" | "timeout"
                                                        | "dead" | "protocol"
    r.close()                                        -> {"runner_processes_killed", "stray_processes"}: after the end
                                                        request, counted once none is left or end_wait_s after it

CPU is measured by the grader: each process's CPU clock (clock_getcpuclockid: every thread it ran, exited ones too, in
nanoseconds) plus its waited-for children (/proc cutime + cstime), for the controller's interpreter and every other
process of the runner account, listed at every call as root (when not root: the child's session, every REFRESH calls);
the client makes the interpreter the subreaper of what it starts, so no descendant leaves the account's view before it
is waited for. A call over its CPU cap or its wall cap kills the child (the next call fails "dead"). A reply line
longer than MAX_LINE bytes (its line end not counted; a complete line as well as one still without its line end) is
a protocol error. What the controller writes to its standard error (and to descriptor 1, which
the client points at it) reaches the log file `stderr_path` through a pipe and a writer thread that keeps its first
`stderr_cap_bytes` and drops the rest, the log's last line then saying how many bytes were dropped (`drain_capped`;
the counts in the stats). Agent code never runs as root: with no unprivileged account or no runuser the run is
refused (fail closed).
"""

import json
import os
import pwd
import queue
import shutil
import subprocess
import threading
import time

from grader.runtime import runner as RUN

# the client's bootstrap: nothing of the controller's folder (the working directory) may be imported by the client
# itself before its no-network filter is in place, so the working directory leaves sys.path (and -P keeps it out)
BOOT = ("import sys; sys.path[:] = [p for p in sys.path if p not in ('', '.')]; sys.path.insert(0, sys.argv[1]); "
        "from mc_client import main; sys.exit(main(sys.argv[2:]))")
IMPORT_S = 60.0
REFRESH = 200
MAX_LINE = 1 << 20              # bytes of one reply line
CLK = os.sysconf("SC_CLK_TCK")
# what json.loads may raise on a hostile line (past its nesting limit RecursionError; MemoryError): a protocol error
LINE_ERRORS = (ValueError, RecursionError, MemoryError)
OOM_ADJ = 1000                  # the controller is the kernel's memory killer's first choice (choom, innermost)
STDERR_CAP_BYTES = 5 << 20      # the controller's log per run (the acceptance passes INTERFACE["stderr_log_max_mb"])
END_WAIT_S = 5                  # after the end request, at most (the acceptance passes INTERFACE["end_wait_s"])
CHOOM = shutil.which("choom") or ("/usr/bin/choom" if os.path.isfile("/usr/bin/choom") else None)


def _runuser():
    for p in (shutil.which("runuser"), "/usr/sbin/runuser", "/sbin/runuser", "/usr/bin/runuser", "/bin/runuser"):
        if p and os.path.isfile(p) and os.access(p, os.X_OK):
            return p
    return None


def drain_capped(fd, log, cap, count=None):
    """The capped writer of the controller's standard error: reads the pipe `fd` in pieces of 64 KiB until its end,
    writes the first `cap` bytes into the open binary file `log` and drops the rest, then writes one last line saying
    how many bytes were dropped (when any were) and closes `log`. `count` (a dict) holds "kept" and "dropped" as they
    grow; returns it."""
    count = {"kept": 0, "dropped": 0} if count is None else count
    count.setdefault("kept", 0)
    count.setdefault("dropped", 0)
    try:
        while True:
            b = os.read(fd, 1 << 16)
            if not b:
                break
            room = cap - count["kept"]
            if room > 0:
                log.write(b[:room])
                count["kept"] += min(room, len(b))
            count["dropped"] += max(0, len(b) - max(room, 0))
    except Exception:  # noqa: BLE001
        pass
    try:
        if count["dropped"]:
            log.write(b"\n[grader] the controller's standard error was cut at %d bytes: %d bytes dropped\n" % (
                cap, count["dropped"]))
        log.close()
    except Exception:  # noqa: BLE001
        pass
    return count


def oom_first(argv):
    """argv started with oom_score_adj OOM_ADJ: `choom -n 1000 --` (util-linux; it execs the program, the same pid,
    the environment untouched), or a shell that writes the value and execs where choom is missing."""
    if CHOOM:
        return [CHOOM, "-n", str(OOM_ADJ), "--"] + list(argv)
    return ["/bin/sh", "-c", 'echo %d > /proc/self/oom_score_adj && exec "$@"' % OOM_ADJ, "sh"] + list(argv)


def _children_ns(pid):
    """CPU (ns) of pid's children that have exited and been waited for (cutime + cstime of /proc/pid/stat)."""
    try:
        with open("/proc/{}/stat".format(pid)) as fh:
            st = fh.read()
        f = st[st.rindex(")") + 2:].split()
        return int((int(f[13]) + int(f[14])) * 1e9 / CLK)
    except (OSError, ValueError, IndexError):
        return 0


def _proc_ns(pid):
    """CPU (ns) of process pid: every thread it ran, live or exited (the process CPU clock of clock_getcpuclockid), plus
    its waited-for children. (Summing the live threads' schedstat would lose the threads that have exited.)"""
    try:
        own = time.clock_gettime_ns(((~int(pid)) << 3) | 2)       # CPUCLOCK_SCHED of the process
    except OSError:
        return 0
    return own + _children_ns(pid)


def _session_pids(sid):
    out = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open("/proc/{}/stat".format(name)) as fh:
                st = fh.read()
            f = st[st.rindex(")") + 2:].split()
            if int(f[3]) == int(sid):
                out.append(int(name))
        except (OSError, ValueError, IndexError):
            continue
    return out


class Runner(object):
    def __init__(self, program_dir, run_root, client_path, user="nobody", python="python3", cpu_cap_s=7200,
                 stderr_path=None, extra_env=None, stderr_cap_bytes=STDERR_CAP_BYTES, end_wait_s=END_WAIT_S):
        self.program_dir = program_dir
        self.run_root = run_root
        self.client_path = client_path
        self.python = python
        self.cpu_cap = int(cpu_cap_s)
        self.stderr_path = stderr_path or (run_root.rstrip("/") + "_stderr.log")
        self.stderr_cap = int(stderr_cap_bytes)
        self.end_wait_s = float(end_wait_s)
        self._err_thread = None
        self._err_count = {"kept": 0, "dropped": 0}
        self.extra_env = extra_env or {}
        acct = RUN.runner_account(user) if os.getuid() == 0 else None
        self.user, self.uid, self.gid = (acct if acct else (None, None, None))
        self.proc = None
        self.leaf = None
        self.q = None
        self.dead = False
        self.n_calls = 0
        self._others = []
        self._cpu_last = None
        # as root (the verifier) a controller whose no-network filter could not be installed is never run: the
        # task's failure (the grader fails closed when it cannot install the filter; infrastructure is never a score)
        self.require_filter = os.getuid() == 0
        self.stats = {"runner_account": self.user or pwd.getpwuid(os.getuid()).pw_name,
                      "runner_uid": self.uid if self.uid is not None else os.getuid(), "runner_processes_killed": 0,
                      "stray_processes": 0, "import_s": None, "deaths": [], "stderr_kept_bytes": 0,
                      "stderr_dropped_bytes": 0, "stderr_cap_bytes": self.stderr_cap}

    # ------------------------------------------------------------------------------------------------------ start
    def start(self):
        if os.path.isdir(self.run_root):
            shutil.rmtree(self.run_root, ignore_errors=True)
        owner = (self.uid, self.gid) if self.uid is not None else None
        rd = RUN.prepare_run_dir(self.run_root, self.program_dir, {}, owner=owner)
        shutil.copyfile(self.client_path, os.path.join(rd["lib"], "mc_client.py"))
        os.chmod(os.path.join(rd["lib"], "mc_client.py"), 0o644)
        self.rd = rd
        env = RUN.child_env(rd["home"], rd["tmp"], rd["lib"], "", threads=1, extra=self.extra_env)
        env.pop("ARB_GIVEN_DIR", None)
        env.pop("PYTHONPATH", None)
        argv = oom_first([self.python, "-B", "-E", "-s", "-P", "-c", BOOT, rd["lib"], rd["home"]])   # choom innermost
        cmd = RUN.wrap_limits(argv, cpu_s=self.cpu_cap, user=None)   # prlimit --cpu --core=0; no process-count limit
        if os.getuid() == 0:
            if not self.user:
                raise RuntimeError("no unprivileged runner account: refusing to run the controller as root")
            ru = _runuser()
            if not ru:
                raise RuntimeError("runuser not found: refusing to run the controller as root")
            cmd = [ru, "-p", "-u", self.user, "--"] + cmd
        err = open(self.stderr_path, "ab")
        t0 = time.time()
        try:
            self.proc = subprocess.Popen(cmd, cwd=rd["program"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         stderr=subprocess.PIPE, env=env, text=True, bufsize=1, start_new_session=True)
        except BaseException:
            err.close()
            raise
        self._err_thread = threading.Thread(target=drain_capped, daemon=True,
                                            args=(self.proc.stderr.fileno(), err, self.stderr_cap, self._err_count))
        self._err_thread.start()
        self.q = queue.Queue()
        q, pr = self.q, self.proc

        def pump():
            # bounded lines: a line of more than MAX_LINE bytes (without its line end), complete or not yet, is a
            # protocol error (the child is killed); nothing after it is read
            fd = pr.stdout.fileno()
            buf = b""
            try:
                over = False
                while not over:
                    chunk = os.read(fd, 65536)
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        if len(line) > MAX_LINE:
                            over = True
                            break
                        q.put(line.decode("utf-8", "replace"))
                    if over or len(buf) > MAX_LINE:
                        q.put("\x00OVERFLOW")
                        buf = b""
                        break
            except (OSError, ValueError):
                pass
            q.put(None)
        threading.Thread(target=pump, daemon=True).start()
        deadline = t0 + IMPORT_S
        # the client's first line comes after its no-network filter and BEFORE the controller is imported, so no code
        # of the controller can forge it: whether the filter is in place
        kind, rep = self._next_line(deadline)
        if kind != "ok" or "filter" not in rep:
            self._kill("no filter line")
            if self.require_filter:
                # no code of the controller has run before this line: as root (the verifier) a client that ends,
                # fails or stays silent before it is the task's failure, never a dead controller's 0
                raise RuntimeError("the controller's client ended before its first line (%s), before any code of the "
                                   "controller ran: the task's failure" % kind)
            return False, {"error": "the client's first line: %s" % kind}
        self.stats["no_network"] = bool(rep.get("filter"))         # the client's seccomp filter (no inet sockets)
        if self.require_filter and not self.stats["no_network"]:
            self._kill("no filter")
            raise RuntimeError("the controller's no-network filter could not be installed: the task's failure, before "
                               "any setup")
        kind, rep = self._next_line(deadline)
        self.stats["import_s"] = round(time.time() - t0, 3)
        if kind == "timeout":
            self._kill("import timeout")
            return False, {"error": "import timeout"}
        if kind == "eof":
            self._kill("died at import")
            return False, {"error": "died at import"}
        if kind != "ok":
            self._kill("protocol at import")
            return False, {"error": "protocol"}
        self._find_leaf()
        if os.getuid() == 0:
            self._assert_deprivileged()
        if not rep.get("ready"):
            self._kill("import failed")
            return False, rep
        return True, rep

    def _next_line(self, deadline):
        """("ok", dict) of the next line, or ("timeout" | "eof" | "protocol", None)."""
        try:
            line = self.q.get(timeout=max(0.001, deadline - time.time()))
        except queue.Empty:
            return "timeout", None
        if line is None:
            return "eof", None
        if line == "\x00OVERFLOW":
            return "protocol", None
        try:
            rep = json.loads(line)
        except LINE_ERRORS:
            return "protocol", None
        return ("ok", rep) if isinstance(rep, dict) else ("protocol", None)

    def _find_leaf(self):
        best = None
        for pid in self._pids_full():
            try:
                with open("/proc/{}/cmdline".format(pid), "rb") as fh:
                    cmd = fh.read()
            except OSError:
                continue
            if b"mc_client" in cmd and os.path.basename(cmd.split(b"\0")[0]).startswith(b"python"):
                best = pid
        self.leaf = best

    def _assert_deprivileged(self):
        if self.leaf is None:
            return
        with open("/proc/{}/status".format(self.leaf)) as fh:
            st = fh.read()
        uid = int([l for l in st.splitlines() if l.startswith("Uid:")][0].split()[1])
        if uid == 0 or (self.uid is not None and uid != self.uid):
            self._kill("uid")
            raise RuntimeError("the controller process runs as uid {}: refused".format(uid))

    # -------------------------------------------------------------------------------------------------------- cpu
    def _pids_full(self):
        if self.uid is not None:
            return RUN.uid_processes(self.uid)
        return _session_pids(self.proc.pid) if self.proc is not None else []

    def cpu_ns(self):
        """CPU (ns) the controller has used so far (the interpreter first, then the other processes: a child reaped in
        between is missed once rather than counted twice)."""
        if self.uid is not None or self.n_calls % REFRESH == 0:
            self._others = [p for p in self._pids_full() if p != self.leaf]
        tot = _proc_ns(self.leaf) if self.leaf else 0
        for p in self._others:
            tot += _proc_ns(p)
        return tot

    # ------------------------------------------------------------------------------------------------------- call
    def call(self, op, payload=None, cpu_s=1.0, wall_s=10.0):
        if self.dead or self.proc is None:
            return "dead", None, 0.0, 0.0
        msg = {"op": op}
        if payload:
            msg.update(payload)
        # the CPU charged to a call is everything the controller used since the previous reply (work done between calls,
        # by threads or children, counts against the next call)
        c0 = self._cpu_last if self._cpu_last else self.cpu_ns()
        t0 = time.time()
        try:
            self.proc.stdin.write(json.dumps(msg, separators=(",", ":")) + "\n")
            self.proc.stdin.flush()
        except (OSError, ValueError):
            self._kill("stdin closed")
            return "dead", None, 0.0, 0.0
        status, rep = "ok", None
        deadline = t0 + float(wall_s)
        while True:
            try:
                line = self.q.get(timeout=min(0.25, max(0.001, deadline - time.time())))
                break
            except queue.Empty:
                if (self.cpu_ns() - c0) * 1e-9 > float(cpu_s):
                    self._kill("{} cpu".format(op))
                    return "cpu", None, (self.cpu_ns() - c0) * 1e-9, time.time() - t0
                if time.time() > deadline:
                    self._kill("{} timeout".format(op))
                    return "timeout", None, (self.cpu_ns() - c0) * 1e-9, time.time() - t0
        self.n_calls += 1
        c1 = max(self.cpu_ns(), c0)                # high-water mark: a process missed in one reading is charged later
        cpu_used = (c1 - c0) * 1e-9
        self._cpu_last = c1
        wall = time.time() - t0
        if line is None:
            self._kill("eof")
            return "dead", None, cpu_used, wall
        if line == "\x00OVERFLOW":
            self._kill("reply too long")
            return "protocol", None, cpu_used, wall
        try:
            rep = json.loads(line)
        except LINE_ERRORS:                        # not JSON, or nested past the parser's limit
            rep = None
        if not isinstance(rep, dict):              # not JSON, or JSON that is not an object (a stray line on fd 1)
            self._kill("protocol")
            return "protocol", None, cpu_used, wall
        if not rep.get("ok"):
            status = "error"
        if cpu_used > float(cpu_s):
            status = "cpu"
        return status, rep, cpu_used, wall

    # ------------------------------------------------------------------------------------------------------- stop
    def _kill(self, why):
        self.stats["deaths"].append(why)
        if self.proc is not None:
            try:
                os.killpg(self.proc.pid, 9)
            except (OSError, ProcessLookupError):
                pass
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        if self.uid is not None:
            k, rem = RUN.kill_uid_processes(self.uid)
            self.stats["runner_processes_killed"] += k
        self.dead = True

    def close(self):
        """The end request, then the count of what is left: the controller's process is given until end_wait_s after
        the request to end, and the processes of the runner account (as root every /proc entry of its uid, one that
        has ended but is not yet waited for included; off root the controller's session) are counted once none is
        left or end_wait_s after the request is over, whichever comes first, so that a helper process the standard
        library ends on its own just after the controller is not counted; every process still there is then killed."""
        t_req = time.time()
        if self.proc is not None and not self.dead:
            try:
                self.proc.stdin.write('{"op":"end"}\n')
                self.proc.stdin.flush()
                self.proc.wait(timeout=self.end_wait_s)
            except (OSError, ValueError, subprocess.TimeoutExpired):
                pass
        left = self._pids_full() if self.proc is not None else []
        while left and time.time() < t_req + self.end_wait_s:
            time.sleep(0.02)
            left = self._pids_full()
        self.stats["stray_processes"] = len(left)
        if left or not self.dead:
            self._kill("close")
        self._end_stderr()
        return dict(self.stats)

    def _end_stderr(self, wait_s=5.0):
        """Waits (at most wait_s) for the stderr writer to reach the pipe's end, then records its counts."""
        if self._err_thread is not None:
            self._err_thread.join(timeout=wait_s)
            if not self._err_thread.is_alive() and self.proc is not None and self.proc.stderr is not None:
                try:
                    self.proc.stderr.close()
                except (OSError, ValueError):
                    pass
        self.stats["stderr_kept_bytes"] = int(self._err_count.get("kept", 0))
        self.stats["stderr_dropped_bytes"] = int(self._err_count.get("dropped", 0))
