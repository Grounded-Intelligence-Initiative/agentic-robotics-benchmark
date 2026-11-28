"""Self-test of a grip controller against the toy plant (toy_plant.py). It checks the interface only and says nothing
about how well the controller would do on the hand: the toy is not the hand, and nothing here is scored.

    python3 check_controller.py [CONTROLLER_DIR] [--cycles N] [--record FILE] [--cell FILE] [--json FILE]

CONTROLLER_DIR holds controller.py (default ~/submission/controller). The controller is started the way the acceptance
starts it: the folder is installed as the host installs it (a copy of its regular files and folders only: symbolic
links, special files and folders named __pycache__ are left out, and every entry left out is reported); the copy is
read-only, as the host's installed copy is for the controller (its folders 0555 and its files 0444, and the folder
that holds it 0555, all before the controller starts: creating, changing or removing a file there fails with a
permission error); and the copy is run by a separate Python process (python3 -B -E -s) with the copy as its working
folder, a fresh empty HOME (also the controller's workdir) and TMPDIR (also TMP and TEMP), the places the documents
name for its writes (at acceptance the host's shared temporary places, /tmp and /dev/shm, are writable too and
emptied between units), and the same explicit environment (one BLAS thread and the BLAS settings below), speaking one
JSON line per
request and reply (a reply line longer than 1048576 bytes, its line end not counted, breaks the format). It is then set
up with a unit record (default ../unit/example_record.json) and the cell's data (default ../cell/cell_data.json; the
toy's own when that file is absent), and stepped on the bus cycle against the toy plant for N cycles (default 3000).
Reported: the entries left out of the installed copy, the import, the setup reply and the effective cycle period it
gives, every step reply that breaks the reply format, every step that raised, the CPU each step used (the controller's
process and everything it started) against the effective period, and every process of the controller still there
(running, or ended without being waited for) once none is left or 5 s after its end request (as at acceptance, the
controller's process ends after its reply to the end request without waiting for its threads; a process it started
and left fails a pass condition at acceptance). A controller folder that is a symbolic link is refused, as at
acceptance.
What it cannot mirror of the read-only copy: here the copy belongs to the account that runs the self-test and the
controller runs as that account, so a controller that changes the copy's modes back can still write there, and it can
write wherever that account can; run as root, the self-test stops no write at all (no mode stops root). At acceptance
the copy belongs to the host's administrator account and the controller runs as another, unprivileged account, which
can change neither.
Exit 1 when there is no controller to start (no folder, a folder that is a symbolic link, no regular controller.py),
when the import fails, when setup fails or its reply is not valid or over its budget, when a step reply breaks the
format, when a step ends the controller as at acceptance (more than 1 s of CPU or 5 s of wall clock, a process that
dies, a protocol error), or when a process of the controller is left after its end request; 0 otherwise (an entry left
out, a step that raised and a late cycle are reported, and change nothing of the exit code). The self-test removes its
temporary folders (under TMPDIR) when it ends.
"""
import json
import os
import queue
import shutil
import stat
import statistics
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.dont_write_bytecode = True
sys.path.insert(0, HERE)

# the controller's environment at acceptance (besides PATH, HOME and the temporary folders)
CONTROLLER_ENV = {"LANG": "C.UTF-8", "MPLBACKEND": "Agg", "PYOPENGL_PLATFORM": "osmesa", "OMP_NUM_THREADS": "1",
                  "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1", "OPENBLAS_CORETYPE": "Haswell",
                  "OPENBLAS_L2_SIZE": "256", "NPY_DISABLE_CPU_FEATURES": "X86_V4 AVX512_ICL AVX512_SPR"}
PYTHON_FLAGS = ["-B", "-E", "-s"]
IMPORT_S, SETUP_CPU_S, SETUP_WALL_S, STEP_CPU_END_S, STEP_WALL_END_S = 60.0, 60.0, 120.0, 1.0, 5.0
MAX_LINE = 1 << 20
END_WAIT_S = 5.0                 # how long the end request is given, as at acceptance
# the writable fields of a step reply: (control-table address, lowest, highest value)
WRITABLE = {"torque_enable": (64, 0, 1), "operating_mode": (11, 0, 16), "position_d_gain": (80, 0, 16383),
            "position_i_gain": (82, 0, 16383), "position_p_gain": (84, 0, 16383), "goal_current": (102, -910, 910),
            "profile_acceleration": (108, 0, 32767), "profile_velocity": (112, 0, 32767),
            "goal_position": (116, -1048575, 1048575)}
MODULES = ("thumb", "index", "middle", "ring")


def controller_env(home, tmp):
    env = {"PATH": os.environ.get("PATH", "/usr/bin:/bin"), "HOME": home, "TMPDIR": tmp, "TMP": tmp, "TEMP": tmp,
           "MUJOCO_GL": os.environ.get("MUJOCO_GL", "osmesa")}
    env.update(CONTROLLER_ENV)
    return env


def effective_period(period_ms, reads):
    """The bus rule: max(period, ceil(1.5 + sum over blocks of (1.0 + (27 + 13 x (11 + length)) x 0.0025))) ms,
    computed in whole byte times (2.5 us)."""
    units = 600 + sum(400 + 27 + 13 * (11 + int(n)) for _a, n in reads)
    return max(int(period_ms), -(-units // 400))


def check_setup(rep):
    if not isinstance(rep, dict) or set(rep) != {"period_ms", "reads"}:
        return "the setup reply must be an object with exactly period_ms and reads"
    p = rep["period_ms"]
    if type(p) is not int or not 5 <= p <= 20:
        return "period_ms must be an integer 5..20"
    reads = rep["reads"]
    if not isinstance(reads, list) or not 1 <= len(reads) <= 4:
        return "reads must hold 1 to 4 blocks"
    for b in reads:
        if not isinstance(b, list) or len(b) != 2 or not all(type(x) is int for x in b):
            return "a read block is [address, length], two integers"
        if b[1] < 1 or b[0] < 64 or b[0] + b[1] > 148:
            return "read block %r lies outside the RAM area 64..147" % (b,)
    return None


def check_step(rep):
    """None when the step reply has the right format, else what is wrong with it."""
    if rep is None:
        return None
    if not isinstance(rep, dict):
        return "the reply is neither an object nor null"
    w = rep.get("write")
    if w is not None:
        if not isinstance(w, dict):
            return "write is neither an object nor null"
        for k, vals in w.items():
            if k not in WRITABLE:
                return "write key %r is not a writable field" % (k,)
            if not isinstance(vals, list) or len(vals) != 13:
                return "write %s must be a list of 13 entries" % k
            _a, lo, hi = WRITABLE[k]
            for i, v in enumerate(vals):
                if v is None:
                    continue
                if type(v) is not int:
                    return "write %s[%d] is not an integer (%r)" % (k, i, v)
                if not lo <= v <= hi:
                    return "write %s[%d] = %d is outside %d..%d" % (k, i, v, lo, hi)
    tz = rep.get("tactile_zero")
    if tz is not None:
        if not isinstance(tz, list) or any(m not in MODULES for m in tz) or len(set(tz)) != len(tz):
            return "tactile_zero must be a list of distinct module names"
    for k in ("grasped", "decline"):
        if rep.get(k) is not None and not isinstance(rep.get(k), bool):
            return "%s must be a boolean or null" % k
    return None


def _proc_cpu_ns(pid):
    try:
        own = time.clock_gettime_ns(((~int(pid)) << 3) | 2)
    except OSError:
        return 0
    try:
        with open("/proc/%d/stat" % pid) as fh:
            f = fh.read()
        f = f[f.rindex(")") + 2:].split()
        return own + int((int(f[13]) + int(f[14])) * 1e9 / os.sysconf("SC_CLK_TCK"))
    except (OSError, ValueError, IndexError):
        return own


def _tree(pid):
    kids = {}
    for name in os.listdir("/proc"):
        if name.isdigit():
            try:
                with open("/proc/%s/stat" % name) as fh:
                    s = fh.read()
                kids.setdefault(int(s[s.rindex(")") + 2:].split()[1]), []).append(int(name))
            except (OSError, ValueError, IndexError):
                pass
    out, todo = [], [pid]
    while todo:
        p = todo.pop()
        out.append(p)
        todo += kids.get(p, [])
    return out


def install_copy(src, dst):
    """The controller folder as the host installs it: a copy of `src` at `dst` holding its regular files and folders
    only (symbolic links, special files and every folder named __pycache__ left out). Returns the entries left out,
    [{"path": <path inside the folder>, "kind": "symbolic link" | "special file" | "__pycache__ folder" |
    "unreadable here"}]."""
    left = []
    os.makedirs(dst)
    for dirpath, dirnames, filenames in os.walk(src, followlinks=False):
        rel = os.path.relpath(dirpath, src)
        target = dst if rel == "." else os.path.join(dst, rel)
        keep = []
        for d in sorted(dirnames):
            p, r = os.path.join(dirpath, d), os.path.normpath(os.path.join(rel, d))
            if os.path.islink(p):
                left.append({"path": r, "kind": "symbolic link"})
            elif d == "__pycache__":
                left.append({"path": r, "kind": "__pycache__ folder"})
            else:
                os.makedirs(os.path.join(target, d), exist_ok=True)
                keep.append(d)
        dirnames[:] = keep
        for f in sorted(filenames):
            p, r = os.path.join(dirpath, f), os.path.normpath(os.path.join(rel, f))
            try:
                mode = os.lstat(p).st_mode
            except OSError:
                left.append({"path": r, "kind": "unreadable here"})
                continue
            if stat.S_ISLNK(mode):
                left.append({"path": r, "kind": "symbolic link"})
            elif not stat.S_ISREG(mode):
                left.append({"path": r, "kind": "special file"})
            else:
                try:
                    shutil.copyfile(p, os.path.join(target, f))
                except OSError:
                    left.append({"path": r, "kind": "unreadable here"})
    return left


def _left_out_note(left):
    kinds = sorted({d["kind"] for d in left})
    return ("%d entr%s of your controller folder %s not installed at acceptance (%s). The host installs only the "
            "regular files and folders of submission/controller/: symbolic links, special files (FIFOs, sockets, "
            "devices) and folders named __pycache__ are left out, and this self-test ran your controller from such a "
            "copy, as the host does. Replace a symbolic link by a copy of the file or folder it points to.%s" % (
                len(left), "y" if len(left) == 1 else "ies", "is" if len(left) == 1 else "are",
                "; ".join("%s: %s" % (d["path"], d["kind"]) for d in left[:10]),
                " An entry marked unreadable here could not be read by this self-test (the host reads it)."
                if "unreadable here" in kinds else ""))


def _folder_link(folder):
    """Why the acceptance refuses the folder as a symbolic link (the folder, or a folder named submission holding
    it), or None."""
    if os.path.islink(folder):
        return ("the controller folder %s is a symbolic link: at acceptance it must be a real folder, else nothing of it "
                "is read and the acceptance score is 0" % folder)
    up = os.path.dirname(folder)
    if os.path.basename(up) == "submission" and os.path.islink(up):
        return ("the folder %s is a symbolic link: at acceptance the submission must be a real folder, else nothing of "
                "it is read and the acceptance score is 0" % up)
    return None


def _read_only(path):
    """The installed copy as the host's copy is for the controller: read-only (its folders 0555, its files 0444)."""
    for dirpath, _dirnames, filenames in os.walk(path, topdown=False):
        for f in filenames:
            os.chmod(os.path.join(dirpath, f), 0o444)
        os.chmod(dirpath, 0o555)


def _remove(path):
    """Removes a temporary folder of the self-test, also where the controller took away write permissions and where
    the self-test made the folders read-only."""
    try:
        if not os.path.islink(path):
            os.chmod(path, 0o700)
    except OSError:
        pass
    for dirpath, dirnames, _files in os.walk(path):
        for d in dirnames:
            try:
                p = os.path.join(dirpath, d)
                if not os.path.islink(p):
                    os.chmod(p, 0o700)
            except OSError:
                pass
    shutil.rmtree(path, ignore_errors=True)


class Child(object):
    def __init__(self, folder, work):
        self.home = os.path.join(work, "home")
        self.tmp = os.path.join(work, "tmp")
        for d in (self.home, self.tmp):
            os.makedirs(d, mode=0o700)
        os.chmod(work, 0o555)            # the folder above the copy, as the host's run folder: not writable either
        argv = [sys.executable] + PYTHON_FLAGS + [os.path.abspath(__file__), "--child", self.home]
        self.p = subprocess.Popen(argv, cwd=folder, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=sys.stderr,
                                  env=controller_env(self.home, self.tmp), start_new_session=True)
        self.q = queue.Queue()
        self.n = 0
        self.closed = False
        self.procs = [self.p.pid]
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        """Reply lines as the acceptance reads them: bytes up to the line end; a line longer than MAX_LINE bytes (its
        line end not counted), complete or still without its line end, is a protocol error, and nothing after it is
        read."""
        fd = self.p.stdout.fileno()
        buf = b""
        try:
            while True:
                chunk = os.read(fd, 65536)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if len(line) > MAX_LINE:
                        self.q.put("\x00OVERFLOW")
                        self.q.put(None)
                        return
                    self.q.put(line.decode("utf-8", "replace"))
                if len(buf) > MAX_LINE:
                    self.q.put("\x00OVERFLOW")
                    self.q.put(None)
                    return
        except (OSError, ValueError):
            pass
        self.q.put(None)

    def cpu_ns(self):
        """The controller's CPU so far: its process (every thread) and every process it started (the list of those is
        refreshed every 100 requests; a process that has ended and been waited for counts in its parent)."""
        if self.n % 100 == 0:
            self.procs = _tree(self.p.pid)
        return sum(_proc_cpu_ns(p) for p in self.procs)

    def call(self, msg, wall_s):
        self.n += 1
        c0, t0 = self.cpu_ns(), time.time()
        try:
            self.p.stdin.write((json.dumps(msg, separators=(",", ":")) + "\n").encode("utf-8"))
            self.p.stdin.flush()
        except (OSError, ValueError):
            pass
        try:
            line = self.q.get(timeout=wall_s)
        except queue.Empty:
            return "timeout", None, (self.cpu_ns() - c0) * 1e-9, time.time() - t0
        cpu, wall = (self.cpu_ns() - c0) * 1e-9, time.time() - t0
        if line is None:
            return "dead", None, cpu, wall
        if line == "\x00OVERFLOW":
            return "protocol", None, cpu, wall
        try:
            rep = json.loads(line)
        except ValueError:
            return "protocol", None, cpu, wall
        if not isinstance(rep, dict):
            return "protocol", None, cpu, wall
        return ("ok" if rep.get("ok") else "error"), rep, cpu, wall

    def close(self):
        """The end request, then, as at acceptance, the count of what is left: the controller's process is given
        until END_WAIT_S after the request to end, and the processes of its session are counted once none is left or
        END_WAIT_S after the request is over, whichever comes first. Returns the processes still there then
        ([{"pid", "state", "command"}]: running, or ended but not waited for; a process that left the session is not
        seen here, but is counted at acceptance), and ends them."""
        self.closed = True
        t_req = time.time()
        try:
            self.p.stdin.write(b'{"op": "end"}\n')
            self.p.stdin.flush()
        except (OSError, ValueError):
            pass
        try:
            self.p.wait(timeout=END_WAIT_S)
        except subprocess.TimeoutExpired:
            pass
        procs = _session(self.p.pid)
        while procs and time.time() < t_req + END_WAIT_S:
            time.sleep(0.02)
            procs = _session(self.p.pid)
        left = [{"pid": pid, "state": st, "command": _command(pid)} for pid, st in procs]
        for d in left:
            try:
                os.kill(d["pid"], 9)
            except OSError:
                pass
        try:
            os.killpg(self.p.pid, 9)
        except OSError:
            pass
        try:
            self.p.wait(timeout=END_WAIT_S)
        except subprocess.TimeoutExpired:
            pass
        return left


def _session(sid):
    """[(pid, state)] of every process in the session `sid` (the controller's: it starts in a session of its own)."""
    out = []
    for name in os.listdir("/proc"):
        if name.isdigit():
            try:
                with open("/proc/%s/stat" % name) as fh:
                    s = fh.read()
                f = s[s.rindex(")") + 2:].split()
                if int(f[3]) == int(sid):
                    out.append((int(name), f[0]))
            except (OSError, ValueError, IndexError):
                pass
    return sorted(out)


def _command(pid):
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as fh:
            cmd = fh.read().replace(b"\0", b" ").decode("utf-8", "replace").strip()
        if not cmd:
            with open("/proc/%d/comm" % pid) as fh:
                cmd = "[%s]" % fh.read().strip()
        return cmd[:200]
    except OSError:
        return "?"


def _left_note(left, own_pid):
    """What the report says about processes still there (running, or ended without being waited for) after the end
    request: what it means at acceptance and what to do about it."""
    text = ("%d process(es) of your controller were still there %.0f s after the end request (running, or ended "
            "without being waited for: %s). At acceptance a "
            "process the controller started that is left after the run fails the pass condition \"no process it "
            "started is left after the run\", and the acceptance score is then 0. End every process your controller "
            "starts, and wait for it (os.waitpid), before step returns or at the latest when the end request comes. "
            "Threads need not be ended: after its reply to the end request the controller's own process runs its "
            "exit handlers and ends, whatever threads still run." % (len(left), END_WAIT_S, "; ".join(
                "pid %d: %s" % (d["pid"], d["command"]) for d in left[:5])))
    if any(d["pid"] == own_pid for d in left):
        text += (" Your controller's own process is among them: it had not ended %.0f s after the end request, kept "
                 "from it by a process pool still running a task or by an exit handler that did not return." % END_WAIT_S)
    return text


def _end_process(out, code):
    """As at acceptance: the standard library's shutdown of process pools runs (concurrent.futures' exit hook, which
    Python's normal exit runs before the exit handlers), then the exit handlers, the replies and the log are flushed,
    and the process ends without waiting for threads that are not daemons."""
    try:
        pools = sys.modules.get("concurrent.futures.process")
        if pools is not None:
            pools._python_exit()
    except BaseException:  # noqa: BLE001
        import traceback
        traceback.print_exc()
    try:
        import atexit
        atexit._run_exitfuncs()
    except BaseException:  # noqa: BLE001
        import traceback
        traceback.print_exc()
    for f in (out, sys.stdout, sys.stderr):
        try:
            f.flush()
        except BaseException:  # noqa: BLE001
            pass
    os._exit(code)


def child_main(workdir):
    """The controller's side: import controller.py from the working folder, answer one JSON line per request. As at
    acceptance, the replies go out on a duplicate of descriptor 1, and descriptor 1 itself writes to stderr from before
    the import: what the controller prints, and what native code writes to descriptor 1, goes to the log."""
    out = os.fdopen(os.dup(1), "w", buffering=1)
    os.dup2(2, 1)
    sys.stdout = sys.stderr
    sys.path[:] = [p for p in sys.path if os.path.abspath(p or ".") != HERE]   # as at acceptance: only the
    sys.path.insert(0, os.getcwd())                                            # controller's own folder

    def say(obj):
        try:
            out.write(json.dumps(obj, separators=(",", ":")) + "\n")
        except (TypeError, ValueError) as e:
            out.write(json.dumps({"ok": False, "error": "reply: %s" % e}) + "\n")
        out.flush()
    try:
        import controller as _c
        ctl = _c.Controller(workdir)
    except BaseException as e:  # noqa: BLE001
        import traceback
        traceback.print_exc()
        say({"ok": False, "ready": False, "error": "import: %s: %s" % (type(e).__name__, e)})
        _end_process(out, 1)
    say({"ok": True, "ready": True})
    for line in sys.stdin:
        req = json.loads(line)
        try:
            if req["op"] == "setup":
                say({"ok": True, "reply": ctl.setup(req["record"], req["cell"])})
            elif req["op"] == "step":
                say({"ok": True, "cmd": ctl.step(req["obs"])})
            else:
                say({"ok": True})
                _end_process(out, 0)
        except BaseException as e:  # noqa: BLE001
            import traceback
            traceback.print_exc()
            say({"ok": False, "error": "%s: %s" % (type(e).__name__, e)})
    _end_process(out, 0)


def _registers(plant_regs, reads):
    from toy_plant import RAM
    out = {}
    for a, n, k, signed in RAM:
        if k in plant_regs:
            bits = 8 * n
            vals = []
            for v in plant_regs[k]:
                v = int(v) & ((1 << bits) - 1)
                if signed and v >= 1 << (bits - 1):
                    v -= 1 << bits
                vals.append(v)
            out[k] = vals
    return out


def _load(path, fallback):
    if path and os.path.isfile(path):
        with open(path) as fh:
            return json.load(fh)
    return fallback()


def _toy_record():
    servos = [{"id": i, "joint": "servo %d" % i, "direction": 1, "home_offset": 2048,
               "eeprom": {"operating_mode": 5, "current_limit": 910, "temperature_limit": 70, "shutdown": 52,
                          "return_delay": 0, "pwm_limit": 885}} for i in range(13)]
    return {"format": "unit-record/1", "serial": "TOY", "servos": servos, "homing_spread_deg": 0.7,
            "supply_v_at_delivery": 12.0, "tactile": {"frame_hz": 88.0, "zero_residual_steps":
                                                     {m: [0] * n for m, n in (("thumb", 127), ("index", 52),
                                                                              ("middle", 52), ("ring", 52))}}}


def _toy_cell(record):
    """The toy's own cell data (used when the cell's file is not found): every posture at 0 deg."""
    joints = [s["joint"] for s in sorted(record["servos"], key=lambda s: s["id"])]
    zero = {j: 0.0 for j in joints}
    return {"format": "cell/1", "note": "the self-test's toy cell data (the cell's own file was not found)",
            "nests": {"pick": {"pos_m": [0.0, 0.1, -0.05], "quat_wxyz": [1.0, 0.0, 0.0, 0.0]},
                      "place": {"pos_m": [0.0, 0.1, -0.05], "quat_wxyz": [1.0, 0.0, 0.0, 0.0]}},
            "taught": {"open": dict(zero), "contact": {w: dict(zero) for w in ("35", "40", "45")},
                       "ring_park": {j: 0.0 for j in joints if j.startswith("ring")}},
            "shifts": 2, "jobs_per_shift": 10}


def _close(ch, rep):
    left = ch.close()
    rep["left_after_end"] = left
    if left:
        rep["left_after_end_note"] = _left_note(left, ch.p.pid)


def run(folder, cycles=3000, record_path=None, cell_path=None):
    rep = {"controller_dir": folder, "import_ok": False, "setup_ok": False, "format_errors": [], "raised": 0,
           "steps": 0, "late_cycles": 0, "ended": None, "left_out": [], "left_after_end": []}
    record = _load(record_path or os.path.join(HERE, "..", "unit", "example_record.json"), _toy_record)
    cell = _load(cell_path or os.path.join(HERE, "..", "cell", "cell_data.json"), lambda: _toy_cell(record))
    why = _folder_link(folder)
    if why:
        rep["ended"] = why
        return rep
    if not os.path.isdir(folder):
        rep["ended"] = "no folder %s" % folder
        return rep
    work = tempfile.mkdtemp(prefix="check_controller_")
    children = []
    try:
        return _run_installed(folder, work, rep, cycles, record, cell, children)
    finally:
        for ch in children:
            if not ch.closed:
                ch.close()
        _remove(work)


def _run_installed(folder, work, rep, cycles, record, cell, children):
    from toy_plant import ToyPlant
    prog = os.path.join(work, "program")
    rep["left_out"] = install_copy(folder, prog)
    if rep["left_out"]:
        rep["left_out_note"] = _left_out_note(rep["left_out"])
    _read_only(prog)
    try:
        is_reg = stat.S_ISREG(os.lstat(os.path.join(prog, "controller.py")).st_mode)
    except OSError:
        is_reg = False
    if not is_reg:
        gone = [d for d in rep["left_out"] if d["path"] == "controller.py"]
        rep["ended"] = ("no controller.py in %s" % folder if not gone else
                        "controller.py is a %s: at acceptance only regular files are installed, so the controller "
                        "has no controller.py and the acceptance score is 0" % gone[0]["kind"])
        return rep
    ch = Child(prog, work)
    children.append(ch)
    t0 = time.time()
    try:
        line = ch.q.get(timeout=IMPORT_S)
    except queue.Empty:
        line = None
    rep["import_s"] = round(time.time() - t0, 3)
    try:
        ready = json.loads(line) if line else None
    except ValueError:
        ready = None
    if not ready or not ready.get("ready"):
        rep["ended"] = "import failed: %s" % ((ready or {}).get("error") or "no ready line")
        _close(ch, rep)
        return rep
    rep["import_ok"] = True
    st, r, cpu, wall = ch.call({"op": "setup", "record": record, "cell": cell}, SETUP_WALL_S)
    rep["setup_cpu_s"], rep["setup_wall_s"] = round(cpu, 4), round(wall, 4)
    err = check_setup(r.get("reply")) if st == "ok" else "setup %s: %s" % (st, (r or {}).get("error"))
    if err is None and cpu > SETUP_CPU_S:
        err = "setup used %.1f s of CPU (budget %.0f s)" % (cpu, SETUP_CPU_S)
    if err:
        rep["ended"] = err
        _close(ch, rep)
        return rep
    rep["setup_ok"] = True
    sr = r["reply"]
    T = effective_period(sr["period_ms"], sr["reads"])
    rep["period_ms"], rep["effective_period_ms"], rep["reads"] = sr["period_ms"], T, sr["reads"]
    plant = ToyPlant(record)
    pending, refused, last_seq, late, k = None, [], -1, 0, 0
    cpus = []
    while k < cycles:
        refused = []
        if pending is not None:
            refused = plant.write(pending.get("write") or {}) if pending.get("write") else []
            plant.flags(bool(pending.get("grasped")), bool(pending.get("decline")))
            pending = None
        plant.advance(2)
        fr = plant.tactile_newest(last_seq)
        if fr:
            last_seq = fr[0]
        obs = {"cycle": k, "t_ms": plant.t, "period_ms": T, "late": late,
               "regs": _registers(plant.read(sr["reads"]), sr["reads"]),
               "tactile": None if fr is None else {"seq": fr[0], "t_capture_ms": fr[1], "t_arrival_ms": fr[2],
                                                   "modules": {m: b.hex() for m, b in fr[3].items()}},
               "cell": plant.cell(), "refused": refused, "event": None}
        st, r, cpu, wall = ch.call({"op": "step", "obs": obs}, STEP_WALL_END_S)
        rep["steps"] += 1
        cpus.append(cpu)
        L = int(cpu * 1000.0 // T) if cpu * 1000.0 > T else 0
        if L:
            rep["late_cycles"] += 1 + L
        if st == "ok":
            e = check_step(r.get("cmd"))
            if e:
                rep["format_errors"].append({"cycle": k, "error": e})
                rep["ended"] = "a reply with the wrong format ends the controller at acceptance"
                break
            pending = r.get("cmd")
        elif st == "error":
            rep["raised"] += 1
            if rep["raised"] <= 3:
                rep.setdefault("raised_examples", []).append(str(r.get("error"))[:300])
        else:
            rep["ended"] = "step %s (at acceptance this ends the controller)" % st
            break
        if cpu > STEP_CPU_END_S:
            rep["ended"] = "a step used %.2f s of CPU (more than %.0f s ends the controller)" % (cpu, STEP_CPU_END_S)
            break
        late = L
        k += 1 + L
        plant.advance(T - 2 + L * T)
    _close(ch, rep)
    if cpus:
        s = sorted(cpus)
        rep["step_cpu_ms"] = {"median": round(1000 * statistics.median(s), 3),
                              "p95": round(1000 * s[int(0.95 * (len(s) - 1))], 3), "max": round(1000 * s[-1], 3)}
        rep["steps_over_period"] = sum(1 for c in cpus if c * 1000.0 > T)
        rep["on_time_fraction"] = round(1.0 - rep["late_cycles"] / max(1, k), 4)
    return rep


def main(argv):
    if argv[:1] == ["--child"]:
        return child_main(argv[1] if len(argv) > 1 else os.getcwd())
    import argparse
    ap = argparse.ArgumentParser(description="self-test of a grip controller against the toy plant (no score)")
    ap.add_argument("controller_dir", nargs="?", default=os.path.expanduser("~/submission/controller"))
    ap.add_argument("--cycles", type=int, default=3000)
    ap.add_argument("--record")
    ap.add_argument("--cell")
    ap.add_argument("--json")
    a = ap.parse_args(argv)
    rep = run(os.path.abspath(a.controller_dir), a.cycles, a.record, a.cell)
    text = json.dumps(rep, indent=1)
    print(text)
    if a.json:
        with open(a.json, "w") as fh:
            fh.write(text + "\n")
    ok = rep["import_ok"] and rep["setup_ok"] and not rep["format_errors"] and not (rep["ended"] or "").startswith(
        ("step", "a step")) and not rep["left_after_end"]
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
