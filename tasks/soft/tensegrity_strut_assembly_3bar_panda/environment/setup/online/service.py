#!/usr/bin/env python3
"""strut-cell-online: the cell service of tensegrity_strut_assembly_3bar_panda's graded episode (dual mode,
experimental).

The agent plays ONE hidden graded episode of the strut cell itself, during its session, through this service. The service
writes the episode's outcome to /var/lib/arb-online (a declared artifact the agent can neither read nor write) and the
separate verifier scores that record (tests/verify.py, tests/online_record.py). The cell is the offline verifier's own:
online/cell/ is a byte copy of the cell's code and data in tests/ (arm/, common/, render/, render_filament/, room/,
episode_trace.py, grader_config.json, kit_info.json, and hidden/bench_plan.json as bench_plan.json): the same scene, the
same episode logic, the same per-episode limits, safety rules, fastening and score.

Started once by environment/setup/setup.sh as root, cwd = the setup stage (/opt/ale/setup, root-only while the agent
works):

    env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent <the numerics pin> \\
        /opt/venv/bin/python3 -I -B online/service.py --config online/online.json

In the foreground, as root and with the network the engine opens for the setup: it installs the cell's code
(online/cell -> /opt/arb-online/cell) and the cell's kit, which the offline verifier downloads at verify start (the
tarball pinned in cell/grader_config.json hidden_gt: its URL at a dataset commit, its sha256 and its file count, checked
as tests/verify.py fetch_hidden_gt checks them) into /opt/arb-online/gt; both root-owned, group `arbcell`, 0750 / 0640:
the lab account reads them, the agent cannot. The command returns once the service serves (exit 0) or failed to start
(exit 1); the service itself is a detached daemon (setsid, stdio on /dev/null) that lives into the agent phase, which
has no network. The layout is arm_hidden_mechanism_box's alelab:
  supervisor (root, comm "arbcell-sup"): binds the unix socket, writes the pid file, forks the worker and restarts it at
      most max_restarts times, and freezes the agent's program on the worker's request (below). It never reads the
      hidden episode and never accepts a connection.
  worker (comm "arbcell-worker"): forked as root. While the graded episode has not started it reads the hidden episode
      (online/episode.json), imports the cell, builds the episode's cell (tests/verify.py run_episode's StrutInfoServer
      and reset(seed): the layout drawn from the seed, the reach check's redraws) and only then drops to the lab account
      irreversibly (setgroups([]), setgid, setuid; not dumpable; no core files) and serves. A worker that finds the
      episode started (a restart) reads nothing hidden: the episode ends with its last record (end_reason
      worker_restart, no resume) and the worker answers `status` only.
  Both run under PR_SET_NO_NEW_PRIVS and a seccomp filter under which socket() / socketpair() work only for AF_UNIX.
  The cell runs on ONE thread of the worker (its main thread): the cameras' Filament renderer is bound to the thread
  that made its engine. Connection threads only read requests and write replies; every cell call, the record, the
  windows and the episode's end run on that thread, one at a time.

The graded episode:
  * `start` charges it once per session: the charge is persisted (state.json, fsync) before anything is simulated and is
    never refunded. A start is accepted only within prep_s of the service's start.
  * The freeze (dual mode): the program freezes at the earlier of the graded start and the start window's close
    (prep_s after the service's start). Before a start is charged, or when the window closes without a start, the
    worker asks the supervisor, over the socketpair it holds since its fork, to copy /home/user/submission into
    submission_snapshot in the record directory (root-owned 0700 / 0600): regular files and directories the agent owns,
    every name opened relative to its open parent without following a link; symbolic links, special files, other
    owners' entries and __pycache__ are skipped. Main's submission cap holds for the copy (tests/verify.py route_gate:
    at most route_gate.max_submission_mb MB of regular files), and the engine's own limits on an artifact (at most
    snapshot_max_entries entries, at most snapshot_max_depth folder levels): beyond one nothing is kept (too_large,
    the program item scores 0). The supervisor
    answers only this one request, with fixed paths, and only while the state file says the program has not been frozen;
    a copy that fails refuses the start and charges nothing; at the window's close it is retried freeze_retries times,
    then the record says so and the program item scores 0 (it never falls back to the session-end submission). The
    record carries snapshot_status, snapshot_at (graded_start | prep_window), snapshot_files, snapshot_bytes and
    snapshot_sha256; a window that closes without a start writes a record of its own (started false, prep_window_closed
    true) and no start is accepted afterwards. The verifier grades that copy as the offline item.
  * With the oracle token (solution/ only: its sha256 is in online/episode.json) the start builds the privileged cell, as
    the offline grader does for a submission that carries the token: privileged=True, so the nominal layout
    (grader_config.json oracle_nominal_layout) and the privileged calls (reference_assemble, get_state_privileged,
    describe_privileged). A wrong token is answered as any other parameter (bad_params).
  * The episode lives in the worker, not in a connection: a disconnect leaves it where it is and a new connection
    resumes it. One client at a time (a second connection is told `busy`). A connection idle for idle_timeout_s is
    closed; the episode stays.
  * It ends at the cell's own end (the simulated-time cap, `done`), at `finish`, graded_s after its start
    (`graded_window`), or when the session ends (the sandbox is frozen or destroyed: the last record applies). It ends
    the way the offline grader ends an episode whose program stops (wire.serve_episode returns, then server.finalize):
    nothing is simulated afterwards and the cell's fastened modules are scored as they stand. A cell call that raises ends
    the episode the same way (the offline wire_error), `done` with the error's type in the record.
  * Not carried over from the program run (O2): the 2400 s wall-clock cap, the 120 s silence, the CPU-time backstop and
    the program contract's gates (imports, the network audit, the step-call gate); the graded window bounds the episode
    instead. Every per-episode rule of the cell holds: 1200 s of simulated time, 500 ticks per step, 600 camera frames,
    the four safety rules (any of them zeroes the episode, as offline) and the fastening rules.
  * The record (record.json, written atomically: a temp file, fsync, rename) is rewritten after a request that changes
    the episode, at most every record_min_interval_s (a pending change is written within about a quarter of a second
    of the last request), and always when the episode starts and ends; requests.jsonl logs every request of the episode
    (method, arguments, simulated time; no images) up to requests_log_max_bytes. At the end the offline verifier's
    episode trace (episode_trace.EpisodeTrace: the cell's joint state every 4 ticks and its event log) goes to trace/
    in the record directory, without the camera frames' images.
  * The client learns the cell's replies (the offline wire's) and `status` (started / ended, steps, simulated time,
    camera frames, time left in each window, the frozen program's file count and size); never the score, the record,
    the episode's id or its seed.

Protocol: one JSON object per line in each direction, in the offline wire's envelope:
    {"id": n, "method": m, "params": {...}}  ->  {"id": n, "result": ...}  |  {"id": n, "error": code, "detail": text}
A cell reply that is a dict holding only "error" is sent as an error, as the offline wire sends it. On connect the
service sends one greeting line, {"service": SERVICE}, or {"service": SERVICE, "error": "busy", ...} and closes.
Methods: `status`, `start`, and the cell's own (describe, get_state, set_control, command, step, get_camera, finish;
the privileged ones only on an episode started with the oracle token, as offline; anything else is the cell's own
answer, `forbidden`). The service's own errors: bad_request, bad_params, line_too_long, not_started, already_started,
start_window_closed, episode_over, internal_error.

Dependencies: the Python 3.12 standard library for the supervisor; the worker imports the cell (numpy, mujoco, its
Filament renderer).
"""

import argparse
import ctypes
import datetime
import hashlib
import importlib
import json
import math
import os
import platform
import pwd
import queue
import resource
import select
import shutil
import signal
import socket
import stat
import struct
import sys
import tarfile
import threading
import time
import traceback
import urllib.request

SERVICE = "strut-cell-online/1"
SERVICE_VERSION = "1"
RECORD_FORMAT = "arb-online-record/1"
DEFAULTS = {
    "task": None,                     # the task's slug (the record's `task`)
    "episode": "episode.json",        # the hidden graded episode (relative to the config file; root-only)
    "cell_src": "cell",               # the byte copy of the verifier's cell (relative to the config file)
    "install_dir": "/opt/arb-online",  # cell/ and gt/ go here: root-owned, the lab account's group, 0750 / 0640
    "assets_dir": "/opt/task-assets/payload",   # the image's task assets: the Panda model, the cameras' scenery
    "server_module": "arm.assembly_strut.server",
    "server_class": "StrutInfoServer",
    "user": "arbcell",                # the dedicated lab account (created by setup.sh)
    "agent_user": "user",             # the evaluated account; the service refuses to run as it
    "socket": "/run/arb-online/cell.sock",
    "state_dir": "/var/lib/arb-online",
    "prep_s": 21600.0,                # a start is accepted within this many seconds of the service's start
    "graded_s": 10800.0,              # the graded window: the episode ends this many seconds after its start
    "idle_timeout_s": 900.0,          # a connection that sends nothing for this long is closed (the episode stays)
    "send_timeout_s": 300.0,          # a reply the client does not take within this long closes the connection
    "call_timeout_s": 900.0,          # one cell call longer than this ends the worker (reference_assemble excepted)
    "max_line_bytes": 1 << 20,
    "record_min_interval_s": 0.5,
    "requests_log_max_bytes": 32 << 20,
    "max_log_bytes": 64 << 20,
    "max_record_summary_bytes": 400000,
    "max_restarts": 3,
    "start_timeout_s": 900.0,
    "require_seccomp": True,
    "kit_tarball": None,              # a local copy of the kit's tarball instead of the download (maintainers' tests)
    "kit_download_tries": 6,
    "trace": True,                    # the offline verifier's episode trace (grader_config.json episode_trace)
    "env": {},                        # PUBLIC constants only (numeric pins, renderer); never hidden values
    # the freeze (dual mode): the root supervisor copies this folder before the graded start is charged
    "submission": "/home/user/submission",
    "snapshot_name": "submission_snapshot",   # inside state_dir
    "freeze_timeout_s": 300.0,
    "freeze_retries": 3,
    "snapshot_caps_walk": False,      # main's grader has no entry or path caps (tests/verify.py route_gate: the size)
    "snapshot_max_depth": 60,         # the engine's artifact copy stops at 64 levels below a declared path
}
LONG_CALLS = ("reference_assemble",)  # privileged: the reference program runs inside one call (bounded by graded_s)
END_REASONS = ("done", "finish", "graded_window", "worker_restart")
# main's path cap (tests/verify.py MAX_PATH_BYTES): no path inside the submission longer than this many bytes
MAX_PATH_BYTES = 4000

PR_SET_NAME = 15
PR_SET_DUMPABLE = 4
PR_CAPBSET_DROP = 24
PR_SET_NO_NEW_PRIVS = 38
PR_SET_SECCOMP = 22
SECCOMP_MODE_FILTER = 2
SECCOMP_RET_ALLOW = 0x7FFF0000
SECCOMP_RET_ERRNO = 0x00050000
EPERM = 1
# capabilities the root supervisor keeps (worker fork as root, then the drop): CHOWN, DAC_OVERRIDE, KILL, SETGID, SETUID
SUPERVISOR_CAPS = (0, 1, 5, 6, 7)
# machine -> (AUDIT_ARCH, nr socket, nr socketpair, nr io_uring_setup, x32 syscall bit)
SECCOMP_ARCH = {
    "x86_64": (0xC000003E, 41, 53, 425, 0x40000000),
    "aarch64": (0xC00000B7, 198, 199, 425, 0),
}


# ---------------------------------------------------------------------------------------------- process hardening
# (alelab's, unchanged)
_libc = None


def _prctl(option, arg2=0, arg3=0, arg4=0, arg5=0):
    global _libc
    if _libc is None:
        _libc = ctypes.CDLL(None, use_errno=True)
        _libc.prctl.argtypes = [ctypes.c_int, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong]
    if _libc.prctl(option, arg2, arg3, arg4, arg5) != 0:
        err = ctypes.get_errno()
        raise OSError(err, os.strerror(err))


def _set_comm(name):
    try:
        buf = ctypes.create_string_buffer(name.encode()[:15])
        _prctl(PR_SET_NAME, ctypes.addressof(buf))
    except OSError:
        pass


def _seccomp_program(machine):
    audit, nr_socket, nr_socketpair, nr_uring, x32 = SECCOMP_ARCH[machine]
    ld, jeq, jge, ret = 0x20, 0x15, 0x35, 0x06
    deny = SECCOMP_RET_ERRNO | EPERM

    def ins(code, jt, jf, k):
        return struct.pack("HBBI", code, jt, jf, k)

    prog = [ins(ld, 0, 0, 4), ins(jeq, 1, 0, audit), ins(ret, 0, 0, deny), ins(ld, 0, 0, 0)]
    if x32:
        prog += [ins(jge, 0, 1, x32), ins(ret, 0, 0, deny)]
    prog += [
        ins(jeq, 0, 1, nr_uring),
        ins(ret, 0, 0, deny),
        ins(jeq, 2, 0, nr_socket),
        ins(jeq, 1, 0, nr_socketpair),
        ins(ret, 0, 0, SECCOMP_RET_ALLOW),
        ins(ld, 0, 0, 16),  # args[0] low word = address family
        ins(jeq, 0, 1, socket.AF_UNIX),
        ins(ret, 0, 0, SECCOMP_RET_ALLOW),
        ins(ret, 0, 0, deny),
    ]
    return b"".join(prog)


def install_unix_only_seccomp():
    """PR_SET_NO_NEW_PRIVS + a filter under which only AF_UNIX sockets can be created."""
    machine = platform.machine()
    if machine not in SECCOMP_ARCH:
        raise OSError(0, "unsupported architecture for the seccomp filter: " + machine)
    prog = _seccomp_program(machine)
    buf = ctypes.create_string_buffer(prog, len(prog))

    class SockFprog(ctypes.Structure):
        _fields_ = [("len", ctypes.c_ushort), ("filter", ctypes.c_void_p)]

    fprog = SockFprog(len(prog) // 8, ctypes.addressof(buf))
    _prctl(PR_SET_NO_NEW_PRIVS, 1)
    _prctl(PR_SET_SECCOMP, SECCOMP_MODE_FILTER, ctypes.addressof(fprog))


def trim_capabilities(keep=SUPERVISOR_CAPS):
    """Drop every capability not in `keep` from the bounding, effective, permitted and inheritable sets."""
    for cap in range(64):
        if cap in keep:
            continue
        try:
            _prctl(PR_CAPBSET_DROP, cap)
        except OSError:
            pass  # cap beyond the kernel's last cap (EINVAL)

    class Header(ctypes.Structure):
        _fields_ = [("version", ctypes.c_uint32), ("pid", ctypes.c_int)]

    class Data(ctypes.Structure):
        _fields_ = [("effective", ctypes.c_uint32), ("permitted", ctypes.c_uint32), ("inheritable", ctypes.c_uint32)]

    mask = 0
    for cap in keep:
        mask |= 1 << cap
    hdr = Header(0x20080522, 0)
    data = (Data * 2)(Data(mask & 0xFFFFFFFF, mask & 0xFFFFFFFF, 0), Data(mask >> 32, mask >> 32, 0))
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.capset(ctypes.byref(hdr), ctypes.byref(data)) != 0:
        err = ctypes.get_errno()
        raise OSError(err, os.strerror(err))


def seccomp_selftest():
    """True when an AF_INET / AF_INET6 socket cannot be created and AF_UNIX still can."""
    blocked = True
    for fam in (socket.AF_INET, socket.AF_INET6):
        try:
            s = socket.socket(fam, socket.SOCK_STREAM)
            s.close()
            blocked = False
        except OSError:
            pass
    try:
        socket.socket(socket.AF_UNIX, socket.SOCK_STREAM).close()
    except OSError:
        return False
    return blocked


def drop_privileges(lab):
    """To the lab account, irreversibly: no supplementary groups, its gid and uid, not dumpable, no core files."""
    os.chdir("/")
    os.setgroups([])
    os.setgid(lab.pw_gid)
    os.setuid(lab.pw_uid)
    try:
        os.setuid(0)
        raise RuntimeError("privilege drop is reversible")
    except PermissionError:
        pass
    _prctl(PR_SET_DUMPABLE, 0)
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


class Log:
    """Append-only JSON-lines log in the lab-only state dir (0600), size-capped (alelab's)."""

    def __init__(self, path, cap):
        self.path, self.cap = path, cap
        self.lock = threading.Lock()
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
        st = os.fstat(fd)
        if not stat.S_ISREG(st.st_mode) or st.st_nlink != 1:
            os.close(fd)
            raise RuntimeError("log path is not a plain file")
        self.fh = os.fdopen(fd, "a", buffering=1)
        self.size = st.st_size

    def chown(self, uid, gid):
        os.fchown(self.fh.fileno(), uid, gid)

    def write(self, **rec):
        rec = {"t": round(time.time(), 3), **rec}
        line = json.dumps(rec, default=str, separators=(",", ":")) + "\n"
        with self.lock:
            if self.size + len(line) > self.cap:
                return
            try:
                self.fh.write(line)
                self.size += len(line)
            except OSError:
                pass


def atomic_write_json(path, obj, mode=0o600):
    """`obj` as JSON at `path`: a temp file, fsync, rename, fsync of the directory."""
    data = json.dumps(obj, separators=(",", ":"), allow_nan=False).encode("utf-8")
    tmp = path + ".tmp"
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, mode)
    try:
        view = memoryview(data)
        while view:
            view = view[os.write(fd, view):]
        os.fsync(fd)
    finally:
        os.close(fd)
    os.replace(tmp, path)
    dfd = os.open(os.path.dirname(path), os.O_RDONLY)
    try:
        os.fsync(dfd)
    finally:
        os.close(dfd)


def read_json(path, cap=4 << 20):
    with open(path, "rb") as fh:
        blob = fh.read(cap + 1)
    if len(blob) > cap:
        raise ValueError("%s is larger than %d bytes" % (path, cap))
    return json.loads(blob)


def finite(x):
    """A JSON-safe copy with every non-finite float replaced by None (the record is written with allow_nan=False)."""
    if isinstance(x, bool) or x is None or isinstance(x, (str, int)):
        return x
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if isinstance(x, dict):
        return {str(k): finite(v) for k, v in x.items()}
    if isinstance(x, (list, tuple, set, frozenset)):
        return [finite(v) for v in x]
    if hasattr(x, "tolist"):
        return finite(x.tolist())
    return str(x)


def _jsonable(obj):
    """json.dumps fallback of a reply (the offline wire's json.dumps has none: numpy values as plain values)."""
    if hasattr(obj, "tolist"):
        return obj.tolist()
    if isinstance(obj, (set, frozenset)):
        return sorted(obj, key=str)
    return str(obj)


def _iso(t):
    return None if t is None else datetime.datetime.fromtimestamp(t, datetime.timezone.utc).isoformat(
        timespec="seconds").replace("+00:00", "Z")


def wire_error(result):
    """A cell reply the offline wire sends as an error (wire.serve_episode: a dict holding "error" and nothing else)."""
    return isinstance(result, dict) and "error" in result and len(result) == 1


def _reject_constant(name):
    raise ValueError("non-finite constant")


def _f(v, default=0.0):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return float(default)
    return v if math.isfinite(v) else float(default)


# ---------------------------------------------------------------------------------------------- the install (root, setup)
def _secure_dir(path, mode, uid, gid):
    """Create (or check) a directory owned by uid:gid with exactly `mode`; refuse symlinks."""
    try:
        os.mkdir(path, mode)
    except FileExistsError:
        pass
    st = os.lstat(path)
    if not stat.S_ISDIR(st.st_mode):
        raise RuntimeError("%s exists and is not a directory" % path)
    if st.st_uid not in (0, uid):
        raise RuntimeError("%s has an unexpected owner" % path)
    os.chown(path, uid, gid)
    os.chmod(path, mode)


def _own_tree(root, gid, dir_mode=0o750, file_mode=0o640):
    """root:gid with dir_mode / file_mode everywhere under `root` (no link followed; a link is removed)."""
    for dp, dns, fns in os.walk(root, followlinks=False):
        os.chown(dp, 0, gid)
        os.chmod(dp, dir_mode)
        for name in list(dns) + list(fns):
            p = os.path.join(dp, name)
            if os.path.islink(p):
                os.unlink(p)
                continue
            if name in fns:
                os.chown(p, 0, gid)
                os.chmod(p, file_mode)


def install_cell(cfg, lab):
    """online/cell (the byte copy of the verifier's cell) -> install_dir/cell, a fresh copy: root-owned, the lab
    account's group, 0750 / 0640 (the worker imports it, and reads its data files, after it drops privileges)."""
    dst = os.path.join(cfg["install_dir"], "cell")
    tmp = dst + ".tmp"
    for p in (tmp, dst):
        shutil.rmtree(p, ignore_errors=True)
    shutil.copytree(cfg["cell_src"], tmp, symlinks=True, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    _own_tree(tmp, lab.pw_gid)
    os.rename(tmp, dst)
    return dst


def install_kit(cfg, lab, say):
    """The cell's kit (gt/: kit.json, mating_graph.json, joints.json, meshes/), as tests/verify.py fetch_hidden_gt gets
    it at verify start: the tarball pinned in the cell's grader_config.json (hidden_gt: the URL at a dataset commit,
    its sha256, its file count), downloaded (setup has the network; kit_tarball, a local copy, replaces the download
    and is checked the same way), checked, unpacked into install_dir/gt: root-owned, the lab account's group, 0750 /
    0640. A failed download, another sha256, a member outside gt/ or another file count fails the setup."""
    spec = read_json(os.path.join(cfg["install_dir"], "cell", "grader_config.json"))["hidden_gt"]
    url, want, nfiles = str(spec["url"]), str(spec["sha256"]), int(spec["files"])
    base = cfg["install_dir"]
    tarball, part, gt = os.path.join(base, ".gt.tar.xz"), os.path.join(base, ".gt.part"), os.path.join(base, "gt")
    for p in (part, gt):
        shutil.rmtree(p, ignore_errors=True)
    if os.path.lexists(tarball):
        os.remove(tarball)
    t0 = time.time()
    if cfg["kit_tarball"]:
        shutil.copyfile(cfg["kit_tarball"], tarball)
    else:
        tries = max(1, int(cfg["kit_download_tries"]))
        for k in range(tries):
            try:
                with urllib.request.urlopen(url, timeout=120) as r, open(tarball, "wb") as fh:
                    for block in iter(lambda: r.read(1 << 20), b""):
                        fh.write(block)
                break
            except Exception as exc:  # noqa: BLE001 - retried, then the setup fails
                say("the cell's kit: download %d/%d failed: %s" % (k + 1, tries, type(exc).__name__))
                if k == tries - 1:
                    raise RuntimeError("could not download the cell's kit")
                time.sleep(min(60.0, 10.0 * (k + 1)))
    h = hashlib.sha256()
    with open(tarball, "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    if h.hexdigest() != want:
        raise RuntimeError("the cell's kit has another sha256 than grader_config.json pins")
    os.mkdir(part, 0o700)
    with tarfile.open(tarball, "r:xz") as tf:
        bad = [m.name for m in tf.getmembers()
               if not (m.isfile() or m.isdir()) or not (m.name == "gt" or m.name.startswith("gt/"))]
        if bad:
            raise RuntimeError("the cell's kit tarball holds unexpected members")
        tf.extractall(part, filter="data")
    os.rename(os.path.join(part, "gt"), gt)
    shutil.rmtree(part, ignore_errors=True)
    os.remove(tarball)
    n = sum(len(fns) for _dp, _dn, fns in os.walk(gt))
    if n != nfiles or not os.path.isfile(os.path.join(gt, "kit.json")):
        raise RuntimeError("the cell's kit unpacked to %d files; grader_config.json expects %d" % (n, nfiles))
    _own_tree(gt, lab.pw_gid)
    say("the cell's kit: %d files, sha256 %s..., in %.1f s" % (n, want[:12], time.time() - t0))
    return gt


# ---------------------------------------------------------------------------------------------- the episode's score
class Scoring:
    """tests/verify.py's per-episode score of one episode summary (the cell's finalize() dict): the cell's stage score
    (the scope's fastened mates' weight over the scope's total weight, the scope of grader_config.json), 0 after a
    safety violation; and the per-episode statistics under the names main's verdict gives their episode means. Never a
    scoring input of anything else."""

    def __init__(self, cell_cfg, gt_dir):
        graph = read_json(os.path.join(gt_dir, "mating_graph.json"), 16 << 20)
        self.weights = {m["id"]: float(m["weight"]) for m in graph["mates"]}
        self.scope_name = str(cell_cfg.get("scope", "single"))
        self.scope_ids = list(graph["scopes"][self.scope_name])
        self.scope_weight = float(sum(self.weights[i] for i in self.scope_ids))

    def score(self, s):
        """(episode_score, the stage score, None) of one summary (tests/verify.py: vals, the clamp of assembly_score)."""
        stage = float(s["stage_score"])
        val = stage * (1.0 if s["safety_ok"] else 0.0)
        return max(0.0, min(1.0, val)), stage, None

    def metrics(self, s, stage_score, _achieved):
        """main's per-episode statistics for this one episode (the mean and the minimum of one value are the value)."""
        dmg = s.get("damage") or {}
        pk, su = dmg.get("peak") or {}, dmg.get("sustained_0.2s") or {}
        calls = s.get("calls") or {}
        m = {"scope_mates": float(len(self.scope_ids)), "scope_weight": self.scope_weight,
             "stage_score_mean_ungated": float(stage_score),
             "mates_achieved_mean": _f(s.get("n_achieved")), "mates_achieved_min": _f(s.get("n_achieved")),
             "mates_fastened_mean": _f(s.get("n_fastened")), "all_mates_rate": 1.0 if s.get("all_mates") else 0.0,
             "safety_violation_rate": 0.0 if s.get("safety_ok") else 1.0, "mean_sim_time_s": _f(s.get("sim_time_s")),
             "mean_episode_wall_s": _f(s.get("episode_wall_s")), "illegal_call_count": _f(s.get("illegal_call_count")),
             "step_calls": _f(s.get("step_calls")) + _f(calls.get("reference_assemble", 0)),
             "dropped_parts_mean": float(len(s.get("dropped_parts") or [])),
             "lost_parts_mean": float(len(s.get("lost_parts") or [])),
             "max_R1_panda_force_N": _f(pk.get("R1_N")), "max_R2_part_force_sustained_N": _f(su.get("R2_N")),
             "max_R3_rod_lateral_sustained_N": _f(su.get("R3_N")), "max_R4_part_speed_m_s": _f(pk.get("R4_m_s")),
             "max_pad_force_N": _f(pk.get("pad_N")), "camera_frames_mean": _f(s.get("camera_frames_total")),
             "render_wall_s_mean": _f(s.get("render_wall_s")),
             "renderer_filament_rate": 1.0 if s.get("renderer") == "filament" else 0.0,
             "nominal_layout_rate": 1.0 if ((s.get("layout") or {}).get("jitter") or {}).get("nominal") else 0.0,
             "fastened_off_design_rate": float(len(s.get("off_design") or [])),
             "privileged_mode": 1.0 if s.get("privileged_mode") else 0.0,
             "render_failures": _f(s.get("render_failures"))}
        for cam in ("top", "wrist"):
            m["camera_frames_mean." + cam] = _f((s.get("camera_frames") or {}).get(cam, 0))
        for mid in (s.get("weights") or {}):
            m["mate_rate." + str(mid)] = 1.0 if any(r["id"] == mid and r["achieved"] for r in s["mates"]) else 0.0
        return m


SUMMARY_KEEP = ("done_reason", "sim_time_s", "ticks", "mates", "states", "weights", "stage_score", "n_achieved",
                "n_fastened", "all_mates", "off_design", "tool_blocked", "violations", "safety_ok", "illegal_call_count",
                "calls", "step_calls", "lost_parts", "dropped_parts", "damage", "camera_frames", "camera_frames_total",
                "render_wall_s", "renderer", "privileged_mode", "lowest_part_z_m", "layout", "captures")
SUMMARY_BULKY = ("layout", "captures", "states", "tool_blocked", "mates")


def compact_summary(s, cap):
    """The record's copy of the episode summary: the keys above (no event logs), at most `cap` bytes of JSON (the bulky
    keys are dropped first when it is larger)."""
    out = {k: s[k] for k in SUMMARY_KEEP if k in s}
    out["violations"] = list(s.get("violations") or [])[:20]
    out = finite(out)
    for k in (None,) + SUMMARY_BULKY:
        if k is not None:
            out.pop(k, None)
        if len(json.dumps(out, separators=(",", ":"))) <= cap:
            return out
    return {"dropped": "the summary is larger than %d bytes" % cap}


# ---------------------------------------------------------------------------------------------- the episode's cell
class Cell:
    """The graded episode's cell, built as tests/verify.py run_episode builds an episode's: the verifier's own server
    class over the kit, the given Panda model, the bench plan and the cell's configuration, reset to the episode's seed,
    with the offline verifier's episode trace attached (grader_config.json episode_trace.agent)."""

    def __init__(self, cfg, cell_dir, gt_dir, seed, meta, log):
        mod = importlib.import_module(cfg["server_module"])
        self.Server = getattr(mod, cfg["server_class"])
        import episode_trace                   # the cell's copy (tests/episode_trace.py)
        self.episode_trace = episode_trace
        self.cell_cfg = read_json(os.path.join(cell_dir, "grader_config.json"))
        self.kit_info = read_json(os.path.join(cell_dir, "kit_info.json"))
        plan_p = os.path.join(cell_dir, "bench_plan.json")
        self.plan = read_json(plan_p) if os.path.isfile(plan_p) else None
        self.gt_dir = gt_dir
        self.panda_dir = os.path.join(cfg["assets_dir"], "given", "panda")
        self.scoring = Scoring(self.cell_cfg, gt_dir)
        self.render = bool((self.cell_cfg.get("camera_layer") or {}).get("enabled", True))
        self.want_trace = bool(cfg["trace"])
        self.seed = int(seed)
        self.meta = dict(meta)                 # the episode's id, its seed's digest, the oracle token's digest
        self.log = log
        self.server = None
        self.trace = None
        self.build(privileged=False)

    def _cell_log(self, msg):
        self.log.write(kind="cell", msg=str(msg)[:8000])

    def build(self, privileged):
        """A fresh cell of the episode: privileged as the offline grader's cell of a submission with the oracle token
        (the nominal layout and the privileged calls), else the hidden episode's."""
        srv = self.Server(self.gt_dir, self.panda_dir, plan=self.plan, config=self.cell_cfg, mode="hidden",
                          privileged=bool(privileged), render=self.render, log=self._cell_log, kit_info=self.kit_info)
        srv.reset(self.seed)
        self.server, self.trace = srv, None
        tc = self.cell_cfg.get("episode_trace") or {}
        if self.want_trace and tc.get("agent"):
            try:
                tr = self.episode_trace.EpisodeTrace(srv, every_ticks=int(tc.get("every_ticks", 4)), log=self._cell_log)
                cam = tr._cam

                def get_camera(name="overview", depth=True):
                    # the trace's own hook keeps each frame's PNG in memory for the record; here only the frame's
                    # time, camera and index are kept (the images would cost about 1 GB in the session's memory)
                    reply = cam(name, depth)
                    if tr.failed is None and isinstance(reply, dict) and reply.get("ok"):
                        tr.cams.append((float(srv.data.time), str(reply.get("camera", name)),
                                        int(reply.get("frame_index", -1)), None))
                    return reply

                srv.get_camera = get_camera
                self.trace = tr
            except Exception as exc:  # noqa: BLE001 - a record only
                self._cell_log("[trace] not attached: %r" % (exc,))
        return srv

    def peek(self):
        """The cell's finalize() dict as it stands, without its two side effects (closing the cameras' render session,
        ending the episode): the record of an episode that is still running."""
        srv = self.server
        srv.__dict__["_close_session"] = lambda: None
        srv.__dict__["_end"] = lambda reason: None
        try:
            return type(srv).finalize(srv, None)
        finally:
            srv.__dict__.pop("_close_session", None)
            srv.__dict__.pop("_end", None)

    def end(self, reason):
        """The offline grader's end of an episode (wire.serve_episode returns, then server.finalize): the cell is
        marked done, its render session closed, nothing more is simulated; returns finalize()'s dict (through the
        trace's hook when a trace is attached)."""
        srv = self.server
        if not srv.done:
            srv._end(reason)
        return srv.finalize(None)

    def save_trace(self, root, summary, privileged):
        if self.trace is None:
            return None
        try:
            self.trace.detach()
            return self.trace.save(root, "graded", summary, extra={"seed_sha256": self.meta.get("seed_sha256"),
                                                                   "privileged": bool(privileged)})
        except Exception as exc:  # noqa: BLE001 - a record only
            self._cell_log("[trace] not written: %r" % (exc,))
            return None


# ---------------------------------------------------------------------------------------------- the service
class Service:
    """The graded episode behind the socket: one client at a time, the two windows, the record. Everything that touches
    the cell runs on the worker's main thread (run()); the connection threads hand their requests over (jobs)."""

    def __init__(self, cfg, log, reqlog, state, state_path, record_path, cell=None, freeze_chan=None):
        self.cfg = cfg
        self.log, self.reqlog = log, reqlog
        self.state, self.state_path, self.record_path = state, state_path, record_path
        self.cell = cell                       # Cell; None in a restarted worker
        self.freeze_chan = freeze_chan         # to the root supervisor: the freeze (dual mode)
        self.prep_s, self.graded_s = float(cfg["prep_s"]), float(cfg["graded_s"])
        self.min_interval = float(cfg["record_min_interval_s"])
        self.jobs = queue.Queue()
        self.active_lock = threading.Lock()
        self.active = False
        self.conn_seq = 0
        self.started = bool(state.get("started"))
        self.t_start = state.get("start_wall")
        self.ended = bool(state.get("ended")) or (self.started and cell is None)
        self.end_reason = state.get("end_reason")
        self.t_end = state.get("end_wall")
        self.privileged = bool(state.get("privileged"))
        self.requests = 0
        self.dirty = False
        self.last_write = 0.0
        self.call = None                       # (monotonic start, method) of the running cell call: the watchdog's
        self.last_record = None                # a restarted worker's view of the last record (status)
        self.cell_error = None
        self.trace_dir = None

    # -- the freeze (dual mode)
    def freeze(self):
        """Ask the root supervisor to freeze the program (copy /home/user/submission into the record directory) and wait
        for it: {"status": "ok" | "too_large" | "error", "files", "bytes", "sha256", "skipped", ...}."""
        if self.freeze_chan is None:
            return {"status": "error", "detail": "no freeze channel"}
        try:
            self.freeze_chan.settimeout(float(self.cfg["freeze_timeout_s"]))
            self.freeze_chan.sendall(b"freeze\n")
            buf = b""
            while not buf.endswith(b"\n") and len(buf) < 65536:
                chunk = self.freeze_chan.recv(65536)
                if not chunk:
                    break
                buf += chunk
            res = json.loads(buf.decode("utf-8"))
            return res if isinstance(res, dict) else {"status": "error", "detail": "bad reply"}
        except (OSError, ValueError) as exc:
            return {"status": "error", "detail": "the supervisor did not answer (%s)" % type(exc).__name__}

    @staticmethod
    def _snap(res, at):
        snap = {k: res.get(k) for k in ("status", "files", "bytes", "sha256", "skipped", "detail")}
        snap["at"] = at
        return snap

    def _frozen_public(self):
        snap = self.state.get("snapshot") or {}
        return {"status": snap.get("status"), "files": int(snap.get("files") or 0),
                "bytes": int(snap.get("bytes") or 0), "at": snap.get("at")} if snap else None

    def _snapshot_fields(self):
        snap = self.state.get("snapshot") or {}
        return {"snapshot_status": snap.get("status"), "snapshot_at": snap.get("at"),
                "snapshot_files": int(snap.get("files") or 0), "snapshot_bytes": int(snap.get("bytes") or 0),
                "snapshot_sha256": snap.get("sha256"), "snapshot_skipped": int(snap.get("skipped") or 0),
                "snapshot_detail": snap.get("detail") if snap.get("status") not in (None, "ok") else None}

    def _close_prep(self):
        """The start window closed without a start (dual mode): the program freezes now, as it would have at a start, so
        the program item keeps main's preparation time; the graded episode can no longer start. A copy that fails is
        retried; then the record says so and the program item scores 0 (no fallback to the session-end submission)."""
        if self.started or self.state.get("prep_closed"):
            return
        res = {"status": "error", "detail": "not tried"}
        for k in range(max(1, int(self.cfg["freeze_retries"]))):
            res = self.freeze()
            if res.get("status") in ("ok", "too_large"):
                break
            self.log.write(kind="freeze_failed", at="prep_window", attempt=k + 1, detail=str(res.get("detail"))[:200])
            time.sleep(2.0)
        snap = self._snap(res, "prep_window")
        self.state.update(prep_closed=True, prep_closed_wall=time.time(), snapshot=snap)
        try:
            self.save_state()
        except OSError:
            self.log.write(kind="state_write_failed")
        rec = {"format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.cell.meta.get("episode"),
               "seed_sha256": self.cell.meta.get("seed_sha256"), "started": False, "ended": False, "end_reason": None,
               "prep_window_closed": True, "prep_window_closed_at": _iso(self.state.get("prep_closed_wall")),
               "requests": 0, "steps": 0, "sim_time_s": 0.0, "wall_s_since_start": 0.0, "episode_score": 0.0,
               "metrics": {}, "privileged": False, "started_at": None, "ended_at": None, "started_after_s": None,
               "windows": self._windows(), "score_error": None, "cell_error": None, "service": SERVICE,
               "service_version": SERVICE_VERSION}
        rec.update(self._snapshot_fields())
        try:
            atomic_write_json(self.record_path, finite(rec))
        except Exception:  # noqa: BLE001
            self.log.write(kind="record_write_failed", tb=traceback.format_exc(limit=8))
        self.log.write(kind="prep_window_closed", snapshot_status=snap.get("status"), files=snap.get("files"))

    def _windows(self):
        return {"prep_s": self.prep_s, "graded_s": self.graded_s, "service_start": _iso(self.state.get("service_start"))}

    # -- persistence
    def save_state(self):
        atomic_write_json(self.state_path, self.state)

    def _record(self, final=None):
        """The record of a started episode: the cell's summary as it stands (or `final`, finalize()'s dict at the end),
        scored as tests/verify.py scores one episode."""
        cell = self.cell
        srv = cell.server
        now = self.t_end if (self.ended and self.t_end) else time.time()
        score_error = None
        try:
            s = dict(final) if final is not None else dict(cell.peek())
            s["episode_wall_s"] = float(now) - float(self.t_start)
            s["render_failures"] = int(getattr(srv, "render_failures", 0) or 0)
            episode_score, scope_score, achieved = cell.scoring.score(s)
            m = cell.scoring.metrics(s, scope_score, achieved)
        except Exception:  # noqa: BLE001 - recorded; the verifier scores the online item 0 then (main: grader_exception)
            err = traceback.format_exc()
            self.log.write(kind="score_failed", tb=err[-4000:])
            score_error = err.strip().splitlines()[-1][:300]
            s, episode_score, m = {}, 0.0, {}
        for r in END_REASONS:
            m["end_reason." + r] = 1.0 if (self.ended and self.end_reason == r) else 0.0
        rec = {
            "format": RECORD_FORMAT, "task": self.cfg["task"], "episode": cell.meta.get("episode"),
            "seed_sha256": cell.meta.get("seed_sha256"), "started": True, "ended": bool(self.ended),
            "end_reason": self.end_reason if self.ended else None, "requests": int(self.requests),
            "steps": int(srv.step_calls), "sim_time_s": round(float(srv.data.time), 4),
            "wall_s_since_start": round(float(now) - float(self.t_start), 3), "episode_score": episode_score,
            "metrics": m, "privileged": bool(self.privileged), "started_at": _iso(self.t_start),
            "ended_at": _iso(self.t_end) if self.ended else None,
            "started_after_s": round(float(self.t_start) - float(self.state.get("service_start") or 0.0), 3),
            "windows": self._windows(), "score_error": score_error, "cell_error": self.cell_error,
            "done_reason": s.get("done_reason"), "trace_dir": self.trace_dir,
            "summary": compact_summary(s, int(self.cfg["max_record_summary_bytes"])),
            "service": SERVICE, "service_version": SERVICE_VERSION, "prep_window_closed": False,
        }
        rec.update(self._snapshot_fields())
        return finite(rec)

    def _write_record(self, final=None):
        try:
            atomic_write_json(self.record_path, self._record(final))
        except Exception:  # noqa: BLE001 - logged; the next change writes it again
            self.log.write(kind="record_write_failed", tb=traceback.format_exc(limit=8))
        self.last_write = time.time()
        self.dirty = False

    def _log_request(self, method, params, error=None):
        try:
            if method == "get_camera" and isinstance(params, dict):
                params = {k: params[k] for k in ("name", "depth") if k in params}
            text = json.dumps(params, default=str)
            args = params if len(text) <= 4096 else text[:4096] + "..."
        except (TypeError, ValueError):
            args = "<unprintable>"
        srv = self.cell.server if self.cell is not None else None
        sim = round(float(srv.data.time), 4) if srv is not None else None
        self.reqlog.write(n=self.requests, method=str(method)[:80], params=args, sim_time=sim, error=error)

    def finalize_after_restart(self):
        """A worker started after the episode's start (a restart) reads nothing hidden: the episode ends with its last
        record (end_reason worker_restart; nothing is resumed)."""
        try:
            rec = read_json(self.record_path, 2 << 20)
            rec = rec if isinstance(rec, dict) else None
        except (OSError, ValueError):
            rec = None
        now = time.time()
        if self.state.get("started") and rec is not None and not rec.get("ended"):
            rec.update(ended=True, end_reason="worker_restart", ended_at=_iso(now))
            for r in END_REASONS:
                (rec.setdefault("metrics", {}))["end_reason." + r] = 1.0 if r == "worker_restart" else 0.0
            try:
                atomic_write_json(self.record_path, finite(rec))
            except Exception:  # noqa: BLE001
                self.log.write(kind="record_write_failed", tb=traceback.format_exc(limit=8))
        if self.state.get("started") and not self.state.get("ended"):
            self.state.update(ended=True, end_reason="worker_restart", end_wall=now)
            try:
                self.save_state()
            except OSError:
                self.log.write(kind="state_write_failed")
        if self.state.get("started") or self.state.get("end_reason") == "state_unreadable":
            self.ended = True
            self.end_reason = self.state.get("end_reason") or "worker_restart"
        self.last_record = rec
        self.log.write(kind="restarted", started=bool(self.state.get("started")), end_reason=self.end_reason)

    # -- the end of the episode
    def _end(self, reason, cell_reason=None):
        """End the graded episode the way the offline grader ends an episode whose program stops at this moment
        (Cell.end), write the final record and the trace."""
        if self.ended:
            return
        now = time.time()
        self.ended, self.end_reason, self.t_end = True, reason, now
        final = None
        if self.cell is not None:
            try:
                final = self.cell.end(cell_reason or reason)
            except Exception:  # noqa: BLE001 - the record then scores the state as it stood (peek)
                self.log.write(kind="end_failed", tb=traceback.format_exc(limit=12))
        self.state.update(ended=True, end_reason=reason, end_wall=now)
        try:
            self.save_state()
        except OSError:
            self.log.write(kind="state_write_failed")
        if self.cell is not None:
            if final is not None:
                self.trace_dir = self.cell.save_trace(os.path.join(self.cfg["state_dir"], "trace"),
                                                      dict(final, end_reason=reason), self.privileged)
            self._write_record(final)
            srv = self.cell.server
            self.log.write(kind="episode_end", reason=reason, sim_time=round(float(srv.data.time), 3),
                           steps=int(srv.step_calls), requests=self.requests)

    # -- requests (on the cell thread)
    @staticmethod
    def _err(rid, code, detail=None):
        msg = {"id": rid, "error": code}
        if detail:
            msg["detail"] = detail
        return msg

    def status(self):
        now = time.time()
        srv = self.cell.server if self.cell is not None else None
        if srv is not None:
            steps, sim, frames = srv.step_calls, srv.data.time, getattr(srv, "frames_used", 0)
        else:
            rec = self.last_record or {}
            steps, sim = rec.get("steps", 0), rec.get("sim_time_s", 0.0)
            frames = (rec.get("metrics") or {}).get("camera_frames_mean", 0)
        t0 = float(self.state.get("service_start") or 0.0)
        running = self.started and not self.ended
        closed = bool(self.state.get("prep_closed"))
        return {
            "service": SERVICE, "started": bool(self.started), "ended": bool(self.ended),
            "end_reason": self.end_reason if self.ended else None, "steps": int(steps or 0),
            "sim_time": round(float(sim or 0.0), 4), "camera_frames": int(frames or 0),
            "start_window_s": self.prep_s, "graded_window_s": self.graded_s,
            "start_window_closed": closed,
            "start_window_left_s": None if (self.started or self.ended or closed)
            else round(max(0.0, t0 + self.prep_s - now), 1),
            "graded_window_left_s": round(max(0.0, float(self.t_start) + self.graded_s - now), 1) if running else None,
            "program_frozen": self._frozen_public(),
        }

    def _start(self, rid, params):
        if self.started:
            return self._err(rid, "already_started", "the graded episode was started at %s; it starts once per "
                                                     "session" % _iso(self.t_start))
        if self.state.get("prep_closed"):
            return self._err(rid, "start_window_closed", "the graded episode can no longer be started (the start "
                                                         "window closed and the program was frozen then)")
        if self.cell is None or self.ended:
            return self._err(rid, "start_window_closed", "the graded episode can no longer be started")
        if not isinstance(params, dict):
            return self._err(rid, "bad_params", "start takes no parameters")
        privileged = False
        if params:
            token = params.get("oracle_token") if list(params) == ["oracle_token"] else None
            want = self.cell.meta.get("oracle_token_sha256")
            if isinstance(token, str) and want and \
                    hashlib.sha256(token.strip().encode("utf-8")).hexdigest() == want:
                privileged = True              # the oracle (solution/): the privileged cell, as offline
            else:
                return self._err(rid, "bad_params", "start takes no parameters")
        now = time.time()
        if now - float(self.state.get("service_start") or 0.0) > self.prep_s:
            self._log_request("start", {}, error="start_window_closed")
            return self._err(rid, "start_window_closed", "the graded episode had to be started within %.0f s of the "
                                                         "service's start" % self.prep_s)
        # the freeze (dual mode): the program in the submission folder, copied by the root supervisor before the
        # charge; a copy that fails refuses the start and charges nothing (over the caps: too_large, started anyway)
        res = self.freeze()
        if res.get("status") not in ("ok", "too_large"):
            self.log.write(kind="freeze_failed", at="graded_start", detail=str(res.get("detail"))[:200])
            self._log_request("start", {}, error="freeze_failed")
            return self._err(rid, "internal_error", "the program in /home/user/submission could not be frozen; nothing "
                                                    "was started")
        if privileged:
            try:
                self.cell.build(privileged=True)
            except Exception:  # noqa: BLE001 - nothing charged
                self.log.write(kind="privileged_build_failed", tb=traceback.format_exc(limit=20))
                return self._err(rid, "internal_error", "the cell could not be built; nothing was started")
        snap = self._snap(res, "graded_start")
        # the charge: persisted before anything is simulated, never refunded
        self.state.update(started=True, start_wall=now, privileged=privileged, snapshot=snap)
        try:
            self.save_state()
        except OSError:
            self.state.update(started=False, start_wall=None, privileged=False, snapshot=None)
            self.log.write(kind="charge_persist_failed", tb=traceback.format_exc(limit=8))
            return self._err(rid, "internal_error", "the start could not be recorded; nothing was started")
        self.started, self.t_start, self.privileged = True, now, privileged
        srv = self.cell.server
        srv.wall_start = now
        self.requests += 1
        self._log_request("start", {"privileged": True} if privileged else {})
        self._write_record()
        self.log.write(kind="episode_start", privileged=privileged, snapshot_status=snap.get("status"))
        return {"id": rid, "result": {"started": True, "sim_time": round(float(srv.data.time), 4),
                                      "graded_window_s": self.graded_s, "program_frozen": self._frozen_public()}}

    def _cell_call(self, rid, method, params):
        if not self.started:
            if self.state.get("prep_closed"):
                return self._err(rid, "start_window_closed", "the start window closed without a start")
            return self._err(rid, "not_started", "the graded episode has not started: call start() first")
        if not self.ended and time.time() - float(self.t_start) >= self.graded_s:
            self._end("graded_window")
        if self.ended or self.cell is None:
            if method == "finish":
                return {"id": rid, "result": {"ok": True}}
            return self._err(rid, "episode_over", "the graded episode is over (%s)" % self.end_reason)
        if not isinstance(params, dict):
            params = {}                        # as the offline wire does
        srv = self.cell.server
        self.requests += 1
        self.call = (time.monotonic(), method)
        try:
            result = srv.handle(method, params)
        except Exception as exc:  # noqa: BLE001 - the offline wire_error: the episode ends as it stands
            self.call = None
            self.cell_error = type(exc).__name__
            self.log.write(kind="cell_error", method=str(method)[:80], tb=traceback.format_exc(limit=20)[-4000:])
            self._log_request(method, params, error="cell_error")
            self._end("done", cell_reason="wire_error")
            return self._err(rid, "internal_error", "the cell failed in %s (%s): the graded episode ended as it stood"
                             % (str(method)[:80], type(exc).__name__))
        finally:
            self.call = None
        srv.wall_used = time.time() - float(self.t_start)
        self._log_request(method, params, error=result.get("error") if wire_error(result) else None)
        if srv.done:
            self._end("finish" if srv.done_reason == "finish" else "done")
        else:
            self.dirty = True
            if time.time() - self.last_write >= self.min_interval:
                self._write_record()
        if wire_error(result):
            return {"id": rid, "error": result["error"]}
        return {"id": rid, "result": result}

    def dispatch(self, req):
        rid = req.get("id")
        method = req.get("method")
        params = req.get("params")
        if params is None:
            params = {}
        if not isinstance(method, str) or not method:
            return self._err(rid, "bad_request", 'a request is one JSON object {"id": n, "method": name, "params": '
                                                 '{...}} on one line')
        if method == "status":
            if params != {}:
                return self._err(rid, "bad_params", "status takes no parameters")
            return {"id": rid, "result": self.status()}
        if method == "start":
            return self._start(rid, params)
        return self._cell_call(rid, method, params)

    def periodic(self):
        """The two windows and a pending record write (on the cell thread, between requests)."""
        now = time.time()
        if (self.cell is not None and not self.started and not self.state.get("prep_closed")
                and now - float(self.state.get("service_start") or 0.0) >= self.prep_s):
            self._close_prep()                 # the program freezes at the start window's close
            return
        if self.cell is None or not self.started or self.ended:
            return
        if now - float(self.t_start) >= self.graded_s:
            self._end("graded_window")
        elif self.dirty and now - self.last_write >= self.min_interval:
            self._write_record()

    # -- threads
    def run(self):
        """The cell thread (the worker's main thread): the requests one at a time, the windows, the record."""
        while True:
            try:
                job = self.jobs.get(timeout=0.25)
            except queue.Empty:
                job = None
            if job is not None:
                req, slot = job
                try:
                    reply = self.dispatch(req)
                except Exception:  # noqa: BLE001
                    self.log.write(kind="internal_error", tb=traceback.format_exc(limit=20))
                    reply = self._err(req.get("id"), "internal_error")
                slot.put(reply)
            try:
                self.periodic()
            except Exception:  # noqa: BLE001
                self.log.write(kind="periodic_error", tb=traceback.format_exc(limit=8))

    def watchdog(self):
        """A cell call that runs past call_timeout_s ends the worker (the supervisor restarts it; the episode then ends
        with its last record, worker_restart). The privileged reference program runs inside one call: exempt."""
        limit = float(self.cfg["call_timeout_s"])
        while True:
            time.sleep(1.0)
            call = self.call
            if call is None or call[1] in LONG_CALLS:
                continue
            if time.monotonic() - call[0] > limit:
                self.log.write(kind="call_timeout", method=str(call[1])[:80], limit_s=limit)
                os._exit(75)

    def accept_loop(self, listener):
        listener.settimeout(None)
        while True:
            try:
                conn, _ = listener.accept()
            except OSError as exc:
                self.log.write(kind="accept_error", err=str(exc))
                time.sleep(0.05)
                continue
            with self.active_lock:
                admit = not self.active
                if admit:
                    self.active = True
                    self.conn_seq += 1
                    cid = self.conn_seq
            if not admit:
                try:
                    conn.settimeout(2.0)
                    conn.sendall((json.dumps({"service": SERVICE, "error": "busy",
                                              "detail": "another client is connected to the graded episode (one at a "
                                                        "time): close it first"}) + "\n").encode())
                except OSError:
                    pass
                conn.close()
                self.log.write(kind="busy")
                continue
            threading.Thread(target=self.handle, args=(conn, cid), name="arbcell-conn", daemon=True).start()

    def handle(self, conn, cid):
        cap = int(self.cfg["max_line_bytes"])
        try:
            try:
                peer_uid = struct.unpack("3i", conn.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))[1]
            except OSError:
                peer_uid = -1
            self.log.write(kind="connect", conn=cid, peer_uid=peer_uid)
            conn.settimeout(float(self.cfg["send_timeout_s"]))
            conn.sendall((json.dumps({"service": SERVICE}) + "\n").encode())
            buf = b""
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    if len(buf) > cap:
                        conn.sendall((json.dumps(self._err(None, "line_too_long", "a request line is at most %d "
                                                                                  "bytes" % cap)) + "\n").encode())
                        return
                    conn.settimeout(float(self.cfg["idle_timeout_s"]))
                    chunk = conn.recv(1 << 16)
                    if not chunk:
                        return
                    buf += chunk
                    continue
                line, buf = buf[:nl], buf[nl + 1:]
                if not line.strip():
                    continue
                try:
                    req = json.loads(line.decode("utf-8"), parse_constant=_reject_constant)
                except (ValueError, RecursionError):
                    req = None
                if not isinstance(req, dict):
                    reply = self._err(None, "bad_request", "a request is one JSON object {\"id\": n, \"method\": "
                                                           "name, \"params\": {...}} on one line")
                else:
                    slot = queue.Queue(1)
                    self.jobs.put((req, slot))
                    reply = slot.get()
                data = (json.dumps(reply, default=_jsonable) + "\n").encode("utf-8")
                conn.settimeout(float(self.cfg["send_timeout_s"]))
                conn.sendall(data)
        except (OSError, socket.timeout):
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass
            with self.active_lock:
                self.active = False
            self.log.write(kind="disconnect", conn=cid)


# ---------------------------------------------------------------------------------------------- worker
def run_worker(cfg, listener, ready_fd, lab, freeze_chan=None):
    """Forked from the supervisor as root: read the hidden episode and build its cell (only while it has not started and
    its start window is open), drop privileges, serve."""
    _set_comm("arbcell-worker")
    os.umask(0o077)
    state_dir = cfg["state_dir"]
    log = Log(os.path.join(state_dir, "service.log"), int(cfg["max_log_bytes"]))
    log.chown(lab.pw_uid, lab.pw_gid)
    try:
        for k, v in cfg["env"].items():
            os.environ[k] = v
        state_path = os.path.join(state_dir, "state.json")
        record_path = os.path.join(state_dir, "record.json")
        reqlog = Log(os.path.join(state_dir, "requests.jsonl"), int(cfg["requests_log_max_bytes"]))
        reqlog.chown(lab.pw_uid, lab.pw_gid)
        try:
            state = read_json(state_path, 1 << 16)
            if not isinstance(state, dict) or not isinstance(state.get("service_start"), (int, float)):
                raise ValueError("bad state")
        except (OSError, ValueError):
            state = None                       # fail closed: no start
            log.write(kind="state_unreadable")
        if state is None or state.get("started") or state.get("prep_closed"):
            # a restart after the start or the start window's close (or an unreadable state): nothing hidden is read,
            # nothing is resumed
            svc = Service(cfg, log, reqlog, state or {"service_start": 0.0, "started": False, "ended": True,
                                                      "end_reason": "state_unreadable"}, state_path, record_path)
            drop_privileges(lab)
            svc.finalize_after_restart()
        else:
            meta = read_json(cfg["episode"], 1 << 16)            # the hidden graded episode (root-only)
            seed = int(meta["seed"]) & ((1 << 63) - 1)
            if hashlib.sha256(str(seed).encode("ascii")).hexdigest() != meta["seed_sha256"]:
                raise ValueError("the graded episode's seed does not match its digest")
            cell_dir = os.path.join(cfg["install_dir"], "cell")
            gt_dir = os.path.join(cfg["install_dir"], "gt")
            sys.path.insert(0, cell_dir)
            public = {"episode": str(meta["episode"]), "seed_sha256": str(meta["seed_sha256"]),
                      "oracle_token_sha256": meta.get("oracle_token_sha256")}
            t0 = time.time()
            cell = Cell(cfg, cell_dir, gt_dir, seed, public, log)
            log.write(kind="cell_built", wall_s=round(time.time() - t0, 2))
            del meta, seed
            svc = Service(cfg, log, reqlog, state, state_path, record_path, cell=cell, freeze_chan=freeze_chan)
            drop_privileges(lab)
        sec_ok = seccomp_selftest()
        log.write(kind="worker_ready", uid=os.getuid(), gid=os.getgid(), seccomp_inet_blocked=sec_ok,
                  mode="episode" if svc.cell is not None else "ended", started=bool(svc.started))
        if cfg["require_seccomp"] and not sec_ok:
            raise RuntimeError("seccomp self-test failed: an AF_INET socket could be created")
    except Exception:  # noqa: BLE001
        log.write(kind="worker_start_failed", tb=traceback.format_exc(limit=30))
        os.write(ready_fd, b"F")
        os._exit(1)
    os.write(ready_fd, b"R")
    os.close(ready_fd)
    threading.Thread(target=svc.accept_loop, args=(listener,), name="arbcell-accept", daemon=True).start()
    threading.Thread(target=svc.watchdog, name="arbcell-watchdog", daemon=True).start()
    try:
        svc.run()
    except Exception:  # noqa: BLE001
        log.write(kind="worker_crashed", tb=traceback.format_exc(limit=30))
    os._exit(1)


# ---------------------------------------------------------------------------------------------- the freeze (root)
class _TooLarge(Exception):
    pass


def _plain_name(name):
    """A file name the snapshot keeps: valid UTF-8 text without line breaks (others are skipped, as the engine's
    artifact copy could not carry them)."""
    try:
        name.encode("utf-8")
    except UnicodeEncodeError:
        return False
    return "\n" not in name and "\r" not in name


def caps_scan(src_fd, caps):
    """Main's submission caps on the live folder, as tests/verify.py's submission_scan applies them at verify start
    (prune_pycache False): one walk that only stats, every entry counted (files, folders, links, special files,
    __pycache__ and what it holds), the size the sum of the lstat sizes of the entries that are not folders (a link to a
    folder counts as a folder and is not entered), no path longer than max_path_bytes (such a folder is not entered).
    Every name is resolved relative to an open directory, no link is entered. Returns (the first cap passed: "entries",
    "bytes" or "path", in main's order, or None; entries; bytes)."""
    max_entries, cap_bytes, max_path = int(caps["max_entries"]), int(caps["max_bytes"]), int(caps["max_path_bytes"])
    entries = total = 0
    over_path = False
    stack = [(os.dup(src_fd), "")]
    try:
        while stack:
            fd, rel_dir = stack.pop()
            subdirs, files = [], []
            try:
                try:
                    it = os.scandir(fd)
                except OSError:
                    continue
                with it:
                    while True:
                        try:
                            e = next(it)
                        except StopIteration:
                            break
                        except OSError:
                            break
                        try:
                            is_dir = stat.S_ISDIR(os.stat(e.name, dir_fd=fd).st_mode)   # follows a link, as main
                        except OSError:
                            is_dir = False
                        entries += 1
                        if entries > max_entries:
                            return "entries", entries, total
                        rel = rel_dir + "/" + e.name if rel_dir else e.name
                        if len(os.fsencode(rel)) > max_path:
                            over_path = True
                            continue
                        if is_dir:
                            try:
                                if not e.is_symlink():
                                    subdirs.append(e.name)
                            except OSError:
                                pass
                        else:
                            files.append(e.name)
                for name in files:
                    if total <= cap_bytes:
                        try:
                            total += os.stat(name, dir_fd=fd, follow_symlinks=False).st_size
                        except OSError:
                            pass
                for name in subdirs:
                    try:
                        sub = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=fd)
                    except OSError:
                        continue
                    stack.append((sub, rel_dir + "/" + name if rel_dir else name))
            finally:
                os.close(fd)
    finally:
        for fd, _rel in stack:
            os.close(fd)
    if total > cap_bytes:
        return "bytes", entries, total
    if over_path:
        return "path", entries, total
    return None, entries, total


def copy_tree(src_fd, dst_root, agent_uid, caps):
    """Copy the regular files and directories the agent owns under the open directory `src_fd` into `dst_root` (files
    0600, directories 0700, root's). Every name is opened relative to its open parent with O_NOFOLLOW (no link is
    followed, no path the agent controls is resolved by name); symbolic links, special files, entries of another owner,
    __pycache__ and odd names are skipped. Bounded by the caps again (the folder may grow while it is copied), and by
    max_depth folder levels (the engine's artifact copy of the record directory keeps at most 64 levels: deeper, it
    would refuse the whole record)."""
    acc = {"files": [], "bytes": 0, "entries": 0, "skipped": 0}
    stack = [(os.dup(src_fd), dst_root, "", 0)]
    try:
        while stack:
            fd, dst_dir, rel_dir, depth = stack.pop()
            try:
                with os.scandir(fd) as it:
                    for e in it:
                        name = e.name
                        acc["entries"] += 1
                        if acc["entries"] > int(caps["max_entries"]):
                            raise _TooLarge("more than %d entries" % int(caps["max_entries"]))
                        if not _plain_name(name):
                            acc["skipped"] += 1
                            continue
                        try:
                            st = os.stat(name, dir_fd=fd, follow_symlinks=False)
                        except OSError:
                            continue                    # gone meanwhile
                        rel = rel_dir + "/" + name if rel_dir else name
                        if st.st_uid != agent_uid:
                            acc["skipped"] += 1
                            continue
                        if stat.S_ISDIR(st.st_mode):
                            if name == "__pycache__":
                                acc["skipped"] += 1
                                continue
                            if depth + 1 >= int(caps["max_depth"]):
                                raise _TooLarge("folders more than %d levels deep" % int(caps["max_depth"]))
                            try:
                                sub = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
                                              dir_fd=fd)
                            except OSError:
                                acc["skipped"] += 1         # replaced by a link or a file meanwhile
                                continue
                            d = os.path.join(dst_dir, name)
                            try:
                                os.mkdir(d, 0o700)
                            except OSError:
                                os.close(sub)
                                raise
                            stack.append((sub, d, rel, depth + 1))
                        elif stat.S_ISREG(st.st_mode):
                            try:
                                src = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC, dir_fd=fd)
                            except OSError:
                                acc["skipped"] += 1
                                continue
                            try:
                                fst = os.fstat(src)
                                if not stat.S_ISREG(fst.st_mode) or fst.st_uid != agent_uid:
                                    acc["skipped"] += 1
                                    continue
                                out = os.open(os.path.join(dst_dir, name), os.O_WRONLY | os.O_CREAT | os.O_EXCL
                                              | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
                                h = hashlib.sha256()
                                size = 0
                                try:
                                    while True:
                                        chunk = os.read(src, 1 << 20)
                                        if not chunk:
                                            break
                                        size += len(chunk)
                                        acc["bytes"] += len(chunk)
                                        if acc["bytes"] > int(caps["max_bytes"]):
                                            raise _TooLarge("more than %d bytes" % int(caps["max_bytes"]))
                                        h.update(chunk)
                                        view = memoryview(chunk)
                                        while view:
                                            view = view[os.write(out, view):]
                                finally:
                                    os.close(out)
                                acc["files"].append((rel, size, h.hexdigest()))
                            finally:
                                os.close(src)
                        else:
                            acc["skipped"] += 1         # a symbolic link, a FIFO, a socket, a device
            finally:
                os.close(fd)
    finally:
        for fd, _d, _r, _k in stack:
            os.close(fd)
    return acc


def snapshot_digest_of(files):
    """The frozen program's digest (tests/online_record.py recomputes it): for every file in the sorted order of its
    relative path, the line `<path>\\0<size>\\0<sha256 of its content>\\n`; the sha256 of all the lines."""
    total = hashlib.sha256()
    for rel, size, digest in sorted(files, key=lambda f: f[0].encode("utf-8")):
        total.update(rel.encode("utf-8") + b"\0" + str(size).encode() + b"\0" + digest.encode() + b"\n")
    return total.hexdigest()


def snapshot_submission(cfg):
    """(root, the supervisor; dual mode) The freeze: main's caps on the agent's submission folder (caps_scan); within
    them a root-owned copy (copy_tree) in the record directory, built under a temporary name and renamed into place
    (status ok); beyond them nothing is copied (too_large); a copy that fails is status error. Never reads anything
    hidden."""
    state_dir = cfg["state_dir"]
    dst = os.path.join(state_dir, cfg["snapshot_name"])
    tmp = dst + ".tmp"
    caps = {"max_bytes": int(cfg["snapshot_max_bytes"]), "max_entries": int(cfg["snapshot_max_entries"]),
            "max_path_bytes": MAX_PATH_BYTES, "max_depth": int(cfg["snapshot_max_depth"])}
    old = os.umask(0o077)
    src_fd = None
    try:
        for p in (tmp, dst):
            if os.path.lexists(p):
                if os.path.isdir(p) and not os.path.islink(p):
                    shutil.rmtree(p)
                else:
                    os.unlink(p)
        try:
            src_fd = os.open(cfg["submission"], os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
        except FileNotFoundError:
            src_fd = None                               # no submission folder: an empty program
        os.mkdir(tmp, 0o700)
        if src_fd is not None:
            over, entries, size = caps_scan(src_fd, caps) if cfg["snapshot_caps_walk"] else (None, 0, 0)
            if over is not None:
                shutil.rmtree(tmp, ignore_errors=True)
                return {"status": "too_large", "files": 0, "bytes": int(size), "sha256": None, "skipped": 0,
                        "detail": "over main's %s cap (%d entries, %d bytes seen)" % (over, entries, size)}
            try:
                acc = copy_tree(src_fd, tmp, int(cfg["agent_uid"]), caps)
            except _TooLarge as exc:
                shutil.rmtree(tmp, ignore_errors=True)
                return {"status": "too_large", "files": 0, "bytes": 0, "sha256": None, "skipped": 0,
                        "detail": "grew past main's caps while it was copied (%s)" % exc}
        else:
            acc = {"files": [], "bytes": 0, "skipped": 0}
        os.rename(tmp, dst)
        dfd = os.open(state_dir, os.O_RDONLY)
        try:
            os.fsync(dfd)
        finally:
            os.close(dfd)
        return {"status": "ok", "files": len(acc["files"]), "bytes": int(acc["bytes"]),
                "sha256": snapshot_digest_of(acc["files"]), "skipped": int(acc["skipped"])}
    except Exception as exc:  # noqa: BLE001 - the start is refused and nothing is charged
        shutil.rmtree(tmp, ignore_errors=True)
        return {"status": "error", "files": 0, "bytes": 0, "sha256": None, "skipped": 0,
                "detail": type(exc).__name__}
    finally:
        if src_fd is not None:
            os.close(src_fd)
        os.umask(old)


def serve_freeze(chan, cfg, sup_log):
    """(the supervisor's thread) The worker's one request on its channel: `freeze` -> the snapshot's result, one JSON
    line. The program is frozen once: refused once the state file says the graded episode started or the program was
    frozen (at the start, or when the start window closed); anything else is answered `error`."""
    buf = b""
    while True:
        try:
            chunk = chan.recv(4096)
        except OSError:
            return
        if not chunk:
            return
        buf += chunk
        if len(buf) > 4096:
            return
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            res = {"status": "error", "files": 0, "bytes": 0, "sha256": None, "skipped": 0, "detail": "unknown request"}
            if line.strip() == b"freeze":
                try:
                    state = read_json(os.path.join(cfg["state_dir"], "state.json"), 1 << 16)
                    done = bool(state.get("started")) or bool(state.get("snapshot"))
                except (OSError, ValueError):
                    done = True                         # fail closed
                if done:
                    res["detail"] = "the program is frozen already"
                else:
                    t0 = time.time()
                    res = snapshot_submission(cfg)
                    res["wall_s"] = round(time.time() - t0, 3)
            sup_log.write(kind="freeze", **{k: v for k, v in res.items() if k != "sha256"})
            try:
                chan.sendall((json.dumps(res) + "\n").encode())
            except OSError:
                return


# ---------------------------------------------------------------------------------------------- supervisor
def load_config(path):
    path = os.path.abspath(path)
    raw = read_json(path, 1 << 20)
    if not isinstance(raw, dict):
        raise ValueError("config must be a JSON object")
    unknown = set(raw) - set(DEFAULTS)
    if unknown:
        raise ValueError("unknown config keys: %s" % sorted(unknown))
    cfg = dict(DEFAULTS)
    cfg.update(raw)
    base = os.path.dirname(path)
    for key in ("episode", "cell_src"):
        cfg[key] = os.path.normpath(os.path.join(base, cfg[key]))
    if not isinstance(cfg["task"], str) or not cfg["task"]:
        raise ValueError("config needs the task's slug")
    for key in ("socket", "state_dir", "install_dir", "assets_dir", "submission"):
        if not os.path.isabs(cfg[key]):
            raise ValueError("%s must be an absolute path" % key)
    if os.path.dirname(cfg["socket"]) in ("/", "/tmp", "/var/tmp", "/dev/shm", "/run"):
        raise ValueError("the socket must live in its own root-owned directory")
    if not isinstance(cfg["env"], dict) or not all(isinstance(k, str) and isinstance(v, str)
                                                   for k, v in cfg["env"].items()):
        raise ValueError("env must map strings to strings")
    # the freeze applies the offline submission's cap (main's grader configuration, route_gate: the size) and the
    # engine's limits on an artifact (100000 entries, 64 levels: snapshot_max_entries, snapshot_max_depth below them)
    rg = read_json(os.path.join(cfg["cell_src"], "grader_config.json"), 1 << 20).get("route_gate") or {}
    cfg["snapshot_max_bytes"] = int(float(rg.get("max_submission_mb", 60)) * 1024 * 1024)
    cfg["snapshot_max_entries"] = int(rg.get("max_entries", 50000))
    return cfg


def _accounts(cfg):
    lab = pwd.getpwnam(cfg["user"])
    if lab.pw_uid == 0:
        raise RuntimeError("the lab account must not be root")
    agent = pwd.getpwnam(cfg["agent_user"])
    if agent.pw_uid == lab.pw_uid or agent.pw_uid == 0:
        raise RuntimeError("the lab account must differ from the agent account")
    return lab, agent


def _close_fds(keep):
    for fd in range(3, 1024):
        if fd not in keep:
            try:
                os.close(fd)
            except OSError:
                pass


def run_supervisor(cfg, lab, notify_fd):
    _set_comm("arbcell-sup")
    os.umask(0o077)
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    try:                                        # the freeze walks with one descriptor per pending folder
        soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
        want = 65536 if hard == resource.RLIM_INFINITY else min(int(hard), 65536)
        if soft == resource.RLIM_INFINITY or soft < want:
            resource.setrlimit(resource.RLIMIT_NOFILE, (want, hard))
    except (OSError, ValueError):
        pass
    sup_log = Log(os.path.join(cfg["state_dir"], "supervisor.log"), int(cfg["max_log_bytes"]))
    try:
        install_unix_only_seccomp()
        seccomp = True
    except OSError as exc:
        seccomp = False
        sup_log.write(kind="seccomp_install_failed", err=str(exc))
        if cfg["require_seccomp"]:
            os.write(notify_fd, b"F")
            os._exit(1)
    try:
        trim_capabilities()
        caps_trimmed = True
    except OSError as exc:
        caps_trimmed = False
        sup_log.write(kind="capability_trim_failed", err=str(exc))
    sock_path = cfg["socket"]
    if os.path.lexists(sock_path):
        os.unlink(sock_path)
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(sock_path)
    os.chmod(sock_path, 0o666)                 # before chown: without CAP_FOWNER root cannot chmod what it gave away
    os.chown(sock_path, lab.pw_uid, lab.pw_gid)
    listener.listen(16)
    pidfile = os.path.join(os.path.dirname(sock_path), "supervisor.pid")
    tmp = pidfile + ".tmp"
    with open(tmp, "w") as fh:
        fh.write("%d\n" % os.getpid())
    os.chmod(tmp, 0o644)
    os.replace(tmp, pidfile)
    sup_log.write(kind="supervisor_start", pid=os.getpid(), seccomp=seccomp, caps_trimmed=caps_trimmed)

    stopping = {"flag": False, "child": None}

    def on_term(signum, frame):
        stopping["flag"] = True
        child = stopping["child"]
        if child:
            try:
                os.kill(child, signal.SIGKILL)
            except ProcessLookupError:
                pass

    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, on_term)
    signal.signal(signal.SIGHUP, signal.SIG_IGN)

    restarts, first = 0, True
    while not stopping["flag"]:
        r, w = os.pipe()
        sup_chan, worker_chan = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)   # the freeze's channel
        pid = os.fork()
        if pid == 0:
            os.close(r)
            sup_chan.close()
            try:
                sup_log.fh.close()
            except OSError:
                pass
            if notify_fd is not None:
                try:
                    os.close(notify_fd)
                except OSError:
                    pass
            signal.signal(signal.SIGTERM, signal.SIG_DFL)
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            run_worker(cfg, listener, w, lab, worker_chan)
            os._exit(1)
        os.close(w)
        worker_chan.close()
        freezer = threading.Thread(target=serve_freeze, args=(sup_chan, cfg, sup_log), name="arbcell-freeze",
                                   daemon=True)
        freezer.start()
        stopping["child"] = pid
        status = b""
        try:
            status = os.read(r, 1)
        except OSError:
            pass
        os.close(r)
        if first:
            first = False
            if notify_fd is not None:
                try:
                    os.write(notify_fd, b"R" if status == b"R" else b"F")
                    os.close(notify_fd)
                except OSError:
                    pass
                notify_fd = None
        while True:
            try:
                _, code = os.waitpid(pid, 0)
                break
            except ChildProcessError:
                code = -1
                break
            except InterruptedError:
                continue
        stopping["child"] = None
        freezer.join(10.0)                     # the worker's end closed with it: the thread ends before the next fork
        sup_chan.close()
        sup_log.write(kind="worker_exit", pid=pid, status=code, ready=status == b"R")
        if stopping["flag"]:
            break
        if status != b"R":
            sup_log.write(kind="worker_failed_to_start")
            break
        restarts += 1
        if restarts > int(cfg["max_restarts"]):
            sup_log.write(kind="restart_limit_reached", restarts=restarts - 1)
            break
        time.sleep(0.5)
    try:
        listener.close()
        if os.path.lexists(sock_path):
            os.unlink(sock_path)
        os.unlink(pidfile)
    except OSError:
        pass
    sup_log.write(kind="supervisor_exit", restarts=restarts)
    os._exit(0)


def _say(msg):
    print("strut-cell-online: %s" % msg, file=sys.stderr, flush=True)


def start(config_path):
    if os.geteuid() != 0:
        _say("must be started as root (environment/setup/setup.sh)")
        return 2
    try:
        cfg = load_config(config_path)
        lab, agent = _accounts(cfg)
        cfg["agent_uid"] = agent.pw_uid
        os.umask(0o077)
        # the install, as root with the setup's network: the cell's code and the cell's kit
        os.makedirs(os.path.dirname(cfg["install_dir"]), exist_ok=True)
        _secure_dir(cfg["install_dir"], 0o750, 0, lab.pw_gid)
        install_cell(cfg, lab)
        install_kit(cfg, lab, _say)
        sockdir = os.path.dirname(cfg["socket"])
        os.makedirs(os.path.dirname(sockdir), exist_ok=True)
        _secure_dir(sockdir, 0o755, 0, 0)
        os.makedirs(os.path.dirname(cfg["state_dir"]), exist_ok=True)
        _secure_dir(cfg["state_dir"], 0o700, lab.pw_uid, lab.pw_gid)
        state_path = os.path.join(cfg["state_dir"], "state.json")
        if not os.path.lexists(state_path):    # the service's start: the start window counts from here (never reset)
            atomic_write_json(state_path, {"version": 1, "service_start": time.time(), "started": False,
                                           "start_wall": None, "ended": False, "end_reason": None})
            os.chown(state_path, lab.pw_uid, lab.pw_gid)
    except Exception as exc:  # noqa: BLE001
        _say("setup error: %s: %s" % (type(exc).__name__, str(exc)[:300]))
        return 1
    r, w = os.pipe()
    pid = os.fork()
    if pid == 0:
        os.close(r)
        os.setsid()
        if os.fork() > 0:
            os._exit(0)
        devnull = os.open(os.devnull, os.O_RDWR)
        for fd in (0, 1, 2):
            os.dup2(devnull, fd)
        if devnull > 2:
            os.close(devnull)
        _close_fds({w})
        os.chdir("/")
        for k in list(os.environ):
            if k not in ("PATH", "LANG", "HOME"):
                del os.environ[k]
        try:
            run_supervisor(cfg, lab, w)
        except Exception:  # noqa: BLE001
            try:
                Log(os.path.join(cfg["state_dir"], "supervisor.log"), int(cfg["max_log_bytes"])).write(
                    kind="supervisor_crashed", tb=traceback.format_exc(limit=20))
            except Exception:  # noqa: BLE001
                pass
        finally:
            os._exit(1)
    os.close(w)
    os.waitpid(pid, 0)
    deadline = time.monotonic() + float(cfg["start_timeout_s"])
    status = b""
    while time.monotonic() < deadline:
        ready, _, _ = select.select([r], [], [], max(0.0, deadline - time.monotonic()))
        if ready:
            status = os.read(r, 1)
            break
    os.close(r)
    if status == b"R":
        _say("ready (socket %s)" % cfg["socket"])
        return 0
    _say("failed to start (details in the service's own log)")
    return 1


def main(argv=None):
    ap = argparse.ArgumentParser(description="strut-cell-online (start as root from environment/setup/setup.sh)")
    ap.add_argument("--config", required=True)
    args = ap.parse_args(argv)
    return start(args.config)


if __name__ == "__main__":
    sys.exit(main())
