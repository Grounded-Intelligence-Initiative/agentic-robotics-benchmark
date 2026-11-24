#!/usr/bin/env python3
"""The graded episode service of freeflyer_uncrewed_station_caretaking (dual mode): ONE hidden site of the offline
grader, which the agent flies itself during its session, on a Unix socket.

environment/setup/setup.sh starts it as root from the setup stage (root-only) after the rehearsal facility, once it has
installed the service's data folder (`data_dir`: the offline grader's own cell, a byte copy of tests/cabincell with
tests/grader_config.json, tests/reference and the station map in the layout of tests/; the drawn site package; the
hidden episode file; the oracle token's digest) for the service account only (mode 0700):

    env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \\
        /opt/venv/bin/python3 -I -B online/graded_service.py --config online/service.json

The command returns once the service serves (exit 0) or failed to start (exit 1); the service itself is a detached
daemon (setsid, stdio on /dev/null) that lives through the agent phase. The daemon pattern is the furniture task's
graded server (alelab's): a root supervisor and a worker.

  supervisor (root, comm "arb-graded-sup"): binds the socket in its own root-owned directory, forks the worker and
      restarts it (bounded), and freezes the agent's submission on the worker's request (the only request it takes:
      fixed paths from the config, no parameter). It never reads the hidden episode, never imports the cell, never
      accepts a connection.
  worker (comm "arb-graded-wrk"): forked as root; reads the hidden episode (which site, its package's sha256, the
      reference's energy on it) and the oracle token's digest, then drops to the service account irreversibly
      (setgroups to the task assets' group only, setgid, setuid; not dumpable, no core files) and builds the graded
      site's cell exactly as the offline grader builds a site's cell (cabincell.sensors_render.make_server, reset with
      the package and the grader's seed rule), with the offline per-site budgets except the program's two wall-clock
      budgets (program time, silence), which are switched off.
  Both run under PR_SET_NO_NEW_PRIVS and a seccomp filter under which socket() / socketpair() work only for AF_UNIX.

The episode:
  * one per session, started by the agent's `graded_start` within `prep_s` of the service's first start; the start is
    persisted (fsync) before anything is simulated and never refunded;
  * it lives in the worker, not in a connection: a disconnect leaves it where it is and a new connection resumes it;
    one client at a time (another connection gets "busy");
  * practice ends when it starts: before the start is persisted, the worker creates the flag file the rehearsal
    facility watches (`practice_closed_flag`, in a directory that only the service account can write); the facility
    then ends an open rehearsal run and refuses new ones for the rest of the session ("practice_closed"), as offline
    all rehearsal happens before the scored runs. A worker that restarts after the start makes sure the flag exists;
  * the program freezes when the episode starts (dual mode): before anything else of the start, the worker asks the
    root supervisor, over a pipe, to copy the agent's submission folder into the record directory (`snapshot_dir`:
    the agent can read neither it nor the record). The copy follows no symlink (each directory and file is opened
    relative to its parent's descriptor with O_NOFOLLOW; symlinks, non-regular files and files that are not the
    agent's own are skipped and counted), applies main's submission cap (`snapshot_max_bytes`, 60 MB; over it, or
    over the engine's entry and depth limits: status "too_large" and no copy, the program item then scores 0) and
    lands atomically (a temporary folder, then a rename). A copy that fails refuses the start, which then charges
    nothing. If the preparation window closes without a start, the program freezes then (a copy that still fails after
    `snapshot_attempts` tries is recorded as failed and the program item scores 0); the episode is then never flown;
  * the cell answers the robot API's calls (robot_api.md) under the offline per-site budgets: 2400 s of simulated
    time, the frame and capture budgets, the fan-stop and light quotas, the first valid report, a step of at most
    5 s. The program's program-time and silence budgets and the program contract do not apply: simulated time passes
    only inside the agent's own calls;
  * it ends at the agent's `graded_finish`, when the cell ends it (the site's simulated time used up, or an internal
    error of the cell), `graded_s` after its start (the service ends it itself), or with the session. The end is the
    offline end of a run whose program stops at that moment: the firmware stops the robot and holds it (a docked
    robot stays docked), the station runs on for 60 s, the cell's record is made and graded (cabincell.grade, the
    reference's energy on the site from the hidden set). If the session ends first (or the worker dies), the
    episode's call log stands for it: calls.jsonl holds every call that reaches the cell other than the four pure
    queries, with its reply's code and mission time and a digest of the physics state after it, so that the
    separate verifier re-simulates the episode in a fresh cell of its own, checks every call's reply and the state
    digest, and ends it there as the service would have;
  * the record (record.json in the record directory) is rewritten atomically (temporary file, fsync, rename) at the
    start, after the requests that change the episode (at most once per `record_min_interval_s` while requests
    arrive), and at the end, when it names the cell's record (cell_record.json, with its sha256) and holds the grade
    of the episode. requests.jsonl logs every request (no images; the oracle token redacted); service.log and
    supervisor.log the service's own events. The directory belongs to the service account (0700, in the root-owned
    /var/lib): the agent can neither read nor write it; the engine collects it as an artifact for the separate
    verifier.

Wire: one JSON object per line, {"id": n, "method": m, "params": {...}} -> {"id": n, "result": {...}} or
{"id": n, "error": code[, "detail": text]} (the cell's and the rehearsal facility's wire). Methods: `graded_status`
(any time: started, ended, how it ended, simulated time, frames and captures used, the time left in each window; never
the score), `graded_start` (once), `graded_finish` (ends the episode), and once started the robot API's calls, which
the episode's CabinCellServer answers as in a scored run (anything else: "forbidden", counted by the cell). The oracle
starts with its token (params {"oracle_token": ...}, checked against the digest): that allows the cell's privileged
call `reference_run` (the privileged reference program inside the cell, as the offline grader allows it to the
oracle's program); a wrong token is ignored (recorded). Python 3.12 stdlib + the cell's packages.
"""

import argparse
import ctypes
import gc
import hashlib
import json
import math
import os
import platform
import pwd
import grp
import resource
import select
import shutil
import signal
import socket
import stat
import struct
import sys
import time
import traceback

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))

SERVICE_VERSION = "caretaking-graded/1"
RECORD_FORMAT = "arb-online-record/1"
DEFAULTS = {
    "note": "",
    "task": None,
    "data_dir": "/opt/arb-graded",            # the service account's data folder (setup.sh installs it, mode 0700)
    "episode_file": "episode.json",           # in data_dir: the hidden episode
    "oracle_token_digest_file": "oracle_token.sha256",
    "cell_dir": "cell",                       # in data_dir: cabincell/, grader_config.json, reference/, setup_payload/
    "user": "arbgraded",                      # the service account (created in the task image)
    "groups": ["rehearsal"],                  # its only group besides its own: the task assets' (/opt/task-assets)
    "agent_user": "user",                     # the evaluated account: may connect; the service never runs as it
    "socket": "/run/arb-graded/graded.sock",
    "flags_dir": "/run/arb-graded/flags",     # the service account's, mode 0755: the rehearsal facility reads it
    "practice_closed_flag": "/run/arb-graded/flags/practice_closed",
    "record_dir": "/var/lib/arb-online",
    "submission_dir": "/home/user/submission",                       # the agent's program (dual mode)
    "snapshot_dir": "/var/lib/arb-online/submission_snapshot",       # its root-owned copy, taken at the start
    "snapshot_max_bytes": 60 << 20,          # main's submission cap (cell.contract.max_submission_mb = 60)
    "snapshot_max_entries": 100000,          # the engine's own artifact traversal limit
    "snapshot_max_depth": 64,                # the engine's own artifact depth limit
    "snapshot_timeout_s": 600.0,
    "snapshot_attempts": 3,                  # at the window's close: tries before the failure is recorded
    "snapshot_retry_delay_s": 5.0,
    "prep_s": 14400.0,
    "graded_s": 10800.0,
    "cell_limits": None,                     # budget overrides of the cell (budgets.Budgets): the two wall-clock ones
    "request_cap_s": 1800.0,                 # one cell request longer than this is a hang: the worker dies (SIGALRM)
    "privileged_request_cap_s": 7200.0,      # the privileged reference program runs inside one request
    "end_cap_s": 1800.0,                     # the end: the settle, the record and the grade
    "reply_timeout_s": 120.0,                # a client that does not take its reply in this time is dropped
    "idle_takeover_s": 600.0,                # an open connection idle this long gives way to a new one
    "record_min_interval_s": 1.0,
    "max_line_bytes": 1 << 20,
    "requests_log_max_bytes": 32 << 20,
    "calls_log_max_bytes": 256 << 20,        # the call log must stay complete: the episode ends before it would not
    "service_log_max_bytes": 8 << 20,
    "max_restarts": 5,
    "start_timeout_s": 600.0,
    "require_seccomp": True,
    "env": {},                               # public constants only (renderer, threads, numeric pins, asset root)
}
CELL_LIMIT_KEYS = ("silence_s", "program_compute_s")
QUERIES = ("status", "pose", "mission", "station_telemetry")     # the pure queries (not in the call log)
SERVICE_METHODS = ("graded_status", "graded_start", "graded_finish")
EXIT_READY, EXIT_FAILED = b"R", b"F"
PUBLIC_END = {"finish": "finish", "graded_window": "graded_window", "worker_restart": "service_restart",
              "call_log_full": "call_log_full"}

# ----------------------------------------------------------------------------------------------- process hardening
# (alelab's: prctl, the AF_UNIX-only seccomp filter, the supervisor's capability trim)
PR_SET_NAME = 15
PR_SET_DUMPABLE = 4
PR_CAPBSET_DROP = 24
PR_SET_NO_NEW_PRIVS = 38
PR_SET_SECCOMP = 22
SECCOMP_MODE_FILTER = 2
SECCOMP_RET_ALLOW = 0x7FFF0000
SECCOMP_RET_ERRNO = 0x00050000
EPERM = 1
# machine -> (AUDIT_ARCH, nr socket, nr socketpair, nr io_uring_setup, x32 syscall bit)
SECCOMP_ARCH = {"x86_64": (0xC000003E, 41, 53, 425, 0x40000000), "aarch64": (0xC00000B7, 198, 199, 425, 0)}
# capabilities the root supervisor keeps (the worker's fork as root, then its drop): CHOWN, DAC_OVERRIDE, KILL,
# SETGID, SETUID
SUPERVISOR_CAPS = (0, 1, 5, 6, 7)
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
        ins(jeq, 0, 1, nr_uring), ins(ret, 0, 0, deny),
        ins(jeq, 2, 0, nr_socket), ins(jeq, 1, 0, nr_socketpair), ins(ret, 0, 0, SECCOMP_RET_ALLOW),
        ins(ld, 0, 0, 16),                       # args[0] low word = the address family
        ins(jeq, 0, 1, socket.AF_UNIX), ins(ret, 0, 0, SECCOMP_RET_ALLOW), ins(ret, 0, 0, deny),
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
            pass                                 # a cap beyond the kernel's last one (EINVAL)

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
    """True when an AF_INET / AF_INET6 socket cannot be created and an AF_UNIX one still can."""
    blocked = True
    for fam in (socket.AF_INET, socket.AF_INET6):
        try:
            socket.socket(fam, socket.SOCK_STREAM).close()
            blocked = False
        except OSError:
            pass
    try:
        socket.socket(socket.AF_UNIX, socket.SOCK_STREAM).close()
    except OSError:
        return False
    return blocked


# ----------------------------------------------------------------------------------------------- files
class Log(object):
    """Append-only JSON-lines file (0600, no symlink), size-capped: lines past the cap are dropped."""

    def __init__(self, path, cap):
        self.path, self.cap = path, int(cap)
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
        st = os.fstat(fd)
        if not stat.S_ISREG(st.st_mode) or st.st_nlink != 1:
            os.close(fd)
            raise RuntimeError("%s is not a plain file" % path)
        self.fd = fd
        self.size = st.st_size
        self.dropped = 0

    def write(self, **rec):
        rec = dict({"t": round(time.time(), 3)}, **rec)
        try:
            line = (json.dumps(rec, default=str, separators=(",", ":")) + "\n").encode("utf-8", "replace")
        except (TypeError, ValueError):
            line = (json.dumps({"t": rec["t"], "unserialisable": str(rec)[:500]}) + "\n").encode()
        if self.size + len(line) > self.cap:
            self.dropped += 1
            return
        try:
            os.write(self.fd, line)
            self.size += len(line)
        except OSError:
            pass

    def chown(self, uid, gid):
        os.fchown(self.fd, uid, gid)


def write_json_atomic(path, obj, mode=0o600):
    """`obj` as JSON at `path`: a temporary file in the same directory, fsync, rename, fsync of the directory."""
    data = json.dumps(obj, allow_nan=False, separators=(",", ":"), sort_keys=True).encode("utf-8")
    return write_bytes_atomic(path, data, mode)


def write_bytes_atomic(path, data, mode=0o600):
    tmp = "%s.tmp" % path
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, mode)
    try:
        os.fchmod(fd, mode)                      # the mode as asked (the worker's umask is 077)
        view = memoryview(data)
        while len(view):
            n = os.write(fd, view)
            view = view[n:]
        os.fsync(fd)
    finally:
        os.close(fd)
    os.replace(tmp, path)
    dfd = os.open(os.path.dirname(path) or ".", os.O_RDONLY)
    try:
        os.fsync(dfd)
    finally:
        os.close(dfd)
    return hashlib.sha256(data).hexdigest()


def read_json(path, limit=1 << 20):
    with open(path, "rb") as fh:
        blob = fh.read(limit + 1)
    if len(blob) > limit:
        raise ValueError("%s is larger than %d bytes" % (path, limit))
    return json.loads(blob)


def _finite(x, default=0.0):
    try:
        f = float(x)
    except (TypeError, ValueError):
        return default
    return f if math.isfinite(f) else default


def state_digest(data):
    """sha256 of the physics state (qpos, qvel and the simulation time, little-endian float64): the call log's check
    of a re-simulation (tests/verify.py computes the same)."""
    import numpy as np
    h = hashlib.sha256()
    h.update(np.ascontiguousarray(data.qpos, dtype="<f8").tobytes())
    h.update(np.ascontiguousarray(data.qvel, dtype="<f8").tobytes())
    h.update(np.asarray([data.time], dtype="<f8").tobytes())
    return h.hexdigest()


# ----------------------------------------------------------------------------------------------- configuration
def load_config(path):
    path = os.path.abspath(path)
    raw = read_json(path)
    if not isinstance(raw, dict):
        raise ValueError("the config must be a JSON object")
    unknown = set(raw) - set(DEFAULTS)
    if unknown:
        raise ValueError("unknown config keys: %s" % sorted(unknown))
    cfg = dict(DEFAULTS)
    cfg.update(raw)
    if not isinstance(cfg["task"], str) or not cfg["task"]:
        raise ValueError("the config needs 'task'")
    if not os.path.isabs(cfg["data_dir"]):
        raise ValueError("data_dir must be an absolute path")
    for key in ("episode_file", "oracle_token_digest_file", "cell_dir"):
        cfg[key] = os.path.normpath(os.path.join(cfg["data_dir"], cfg[key]))
    lim = cfg["cell_limits"]
    if not isinstance(lim, dict) or set(lim) != set(CELL_LIMIT_KEYS) or not all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and v > 0 for v in lim.values()):
        raise ValueError("cell_limits must give exactly %s, positive numbers" % (CELL_LIMIT_KEYS,))
    for key in ("prep_s", "graded_s", "request_cap_s", "privileged_request_cap_s", "end_cap_s", "reply_timeout_s",
                "idle_takeover_s", "record_min_interval_s", "start_timeout_s", "snapshot_timeout_s", "snapshot_retry_delay_s"):
        v = cfg[key]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            raise ValueError("%s must be a positive number" % key)
    for key in ("socket", "flags_dir", "practice_closed_flag", "record_dir", "submission_dir", "snapshot_dir"):
        if not os.path.isabs(cfg[key]):
            raise ValueError("%s must be an absolute path" % key)
    if os.path.dirname(cfg["snapshot_dir"].rstrip("/")) != os.path.abspath(cfg["record_dir"]):
        raise ValueError("the submission snapshot must live in the record directory (the agent cannot reach it)")
    for key in ("snapshot_max_bytes", "snapshot_max_entries", "snapshot_max_depth", "snapshot_attempts",
                "calls_log_max_bytes"):
        v = cfg[key]
        if isinstance(v, bool) or not isinstance(v, int) or v <= 0:
            raise ValueError("%s must be a positive integer" % key)
    if os.path.dirname(cfg["practice_closed_flag"]) != os.path.abspath(cfg["flags_dir"]):
        raise ValueError("the practice-closed flag must live in the flags directory (only the service writes it)")
    sockdir = os.path.dirname(cfg["socket"])
    if sockdir in ("/", "/tmp", "/var/tmp", "/dev/shm", "/run"):
        raise ValueError("the socket must live in its own root-owned directory")
    if os.path.dirname(os.path.abspath(cfg["flags_dir"])) != sockdir:
        raise ValueError("the flags directory must sit in the socket's root-owned directory")
    if os.path.dirname(os.path.abspath(cfg["record_dir"])) in ("/", "/tmp", "/var/tmp", "/dev/shm"):
        raise ValueError("the record directory needs a root-owned parent")
    if not isinstance(cfg["groups"], list) or not all(isinstance(g, str) for g in cfg["groups"]):
        raise ValueError("groups must be a list of group names")
    if not isinstance(cfg["env"], dict) or not all(isinstance(k, str) and isinstance(v, str)
                                                   for k, v in cfg["env"].items()):
        raise ValueError("env must map strings to strings")
    return cfg


def _accounts(cfg):
    lab = pwd.getpwnam(cfg["user"])
    if lab.pw_uid == 0:
        raise RuntimeError("the service account must not be root")
    try:
        agent = pwd.getpwnam(cfg["agent_user"])
    except KeyError:
        agent = None
    if agent is not None and agent.pw_uid == lab.pw_uid:
        raise RuntimeError("the service account must differ from the agent account")
    gids = []
    for name in cfg["groups"]:
        g = grp.getgrnam(name)
        if g.gr_gid == 0:
            raise RuntimeError("the service account must not keep group root")
        if agent is not None and g.gr_gid == agent.pw_gid:
            raise RuntimeError("the service account must not share the agent account's group")
        gids.append(g.gr_gid)
    return lab, agent, gids


def _secure_dir(path, mode, uid, gid):
    """Create (or check) a directory owned by uid:gid with exactly `mode`; refuse symlinks and foreign owners."""
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
    parent = os.lstat(os.path.dirname(os.path.abspath(path)))
    if parent.st_uid != 0 or (parent.st_mode & 0o022):
        raise RuntimeError("the parent of %s must be root-owned and not group/world-writable" % path)


def state_path(cfg):
    return os.path.join(cfg["record_dir"], "service.json")


# ----------------------------------------------------------------------------------------------- the freeze (root side)
class SnapshotTooLarge(Exception):
    pass


_DIR_FLAGS = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | getattr(os, "O_CLOEXEC", 0)
_FILE_FLAGS = os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_NOCTTY | getattr(os, "O_CLOEXEC", 0)


def tree_digest(files):
    """sha256 of the sorted file list with sizes and contents: one line "<path>\\0<size>\\0<sha256 of the
    content>\\n" per regular file, sorted by path (the verifier recomputes it on the restored snapshot)."""
    h = hashlib.sha256()
    for rel, size, digest in sorted(files):
        h.update(rel.encode("utf-8", "surrogateescape") + b"\0" + str(int(size)).encode() + b"\0" +
                 digest.encode() + b"\n")
    return h.hexdigest()


def snapshot_submission(cfg, agent_uid):
    """The supervisor's copy of the agent's submission folder into cfg["snapshot_dir"] (run as root, before the start
    is charged). Returns {"status": "ok" | "too_large", "files", "dirs", "bytes", "sha256", "skipped", "reason"}; raises
    OSError when the copy itself fails. A missing submission folder, or one that is a symlink, is an empty program."""
    src, dst = cfg["submission_dir"].rstrip("/"), cfg["snapshot_dir"].rstrip("/")
    max_bytes, max_entries = int(cfg["snapshot_max_bytes"]), int(cfg["snapshot_max_entries"])
    max_depth = int(cfg["snapshot_max_depth"])
    tmp = "%s.tmp" % dst
    if os.path.lexists(tmp):
        shutil.rmtree(tmp)
    os.mkdir(tmp, 0o700)
    files, skipped = [], {"symlinks": 0, "not_regular": 0, "foreign_owner": 0, "unreadable": 0}
    count = {"entries": 0, "bytes": 0, "dirs": 0}

    def copy_file(dfd, name, out_path):
        try:
            ffd = os.open(name, _FILE_FLAGS, dir_fd=dfd)
        except OSError:
            skipped["unreadable"] += 1
            return
        try:
            st = os.fstat(ffd)
            if not stat.S_ISREG(st.st_mode):
                skipped["not_regular"] += 1
                return
            if agent_uid is not None and st.st_uid != agent_uid:
                skipped["foreign_owner"] += 1
                return
            h, size = hashlib.sha256(), 0
            out = os.open(out_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o644)
            try:
                while True:
                    chunk = os.read(ffd, 1 << 20)
                    if not chunk:
                        break
                    size += len(chunk)
                    count["bytes"] += len(chunk)
                    if count["bytes"] > max_bytes:
                        raise SnapshotTooLarge("the submission holds more than %d bytes" % max_bytes)
                    h.update(chunk)
                    view = memoryview(chunk)
                    while len(view):
                        view = view[os.write(out, view):]
                os.fsync(out)
            finally:
                os.close(out)
            os.chmod(out_path, 0o644)
            files.append((os.path.relpath(out_path, tmp), size, h.hexdigest()))
        finally:
            os.close(ffd)

    def walk(dfd, out_dir, depth):
        for name in sorted(os.listdir(dfd)):
            count["entries"] += 1
            if count["entries"] > max_entries:
                raise SnapshotTooLarge("the submission holds more than %d entries" % max_entries)
            try:
                st = os.stat(name, dir_fd=dfd, follow_symlinks=False)
            except OSError:
                skipped["unreadable"] += 1
                continue
            out_path = os.path.join(out_dir, name)
            if stat.S_ISLNK(st.st_mode):
                skipped["symlinks"] += 1
            elif stat.S_ISDIR(st.st_mode):
                if depth + 1 > max_depth:
                    raise SnapshotTooLarge("the submission is nested deeper than %d levels" % max_depth)
                try:
                    cfd = os.open(name, _DIR_FLAGS, dir_fd=dfd)
                except OSError:
                    skipped["unreadable"] += 1
                    continue
                try:
                    os.mkdir(out_path, 0o755)
                    os.chmod(out_path, 0o755)    # (the supervisor's umask is 077)
                    count["dirs"] += 1
                    walk(cfd, out_path, depth + 1)
                finally:
                    os.close(cfd)
            elif stat.S_ISREG(st.st_mode):
                copy_file(dfd, name, out_path)
            else:
                skipped["not_regular"] += 1

    try:
        parent, name = os.path.split(src)
        pfd = os.open(parent, _DIR_FLAGS)
        try:
            try:
                sfd = os.open(name, _DIR_FLAGS, dir_fd=pfd)
            except OSError:
                sfd = None                       # no folder, or a symlink in its place: an empty program
            if sfd is not None:
                try:
                    walk(sfd, tmp, 0)
                finally:
                    os.close(sfd)
        finally:
            os.close(pfd)
    except SnapshotTooLarge as exc:
        shutil.rmtree(tmp, ignore_errors=True)
        if os.path.lexists(dst):                 # no copy at all: an earlier attempt's snapshot must not be graded
            shutil.rmtree(dst)
        return {"status": "too_large", "reason": str(exc), "files": len(files), "dirs": count["dirs"],
                "bytes": count["bytes"], "sha256": None, "skipped": skipped}
    except BaseException:
        shutil.rmtree(tmp, ignore_errors=True)
        raise
    os.chmod(tmp, 0o755)
    if os.path.lexists(dst):
        shutil.rmtree(dst)
    os.replace(tmp, dst)
    dfd = os.open(os.path.dirname(dst), os.O_RDONLY)
    try:
        os.fsync(dfd)
    finally:
        os.close(dfd)
    return {"status": "ok", "reason": None, "files": len(files), "dirs": count["dirs"], "bytes": count["bytes"],
            "sha256": tree_digest(files), "skipped": skipped}


def serve_snapshot_request(cfg, agent_uid, line, sup_log):
    """One request line of the worker -> the reply object. The request takes no parameter: the paths and the caps are
    the config's."""
    try:
        req = json.loads(line)
    except ValueError:
        req = None
    if not isinstance(req, dict) or req.get("op") != "snapshot":
        return {"status": "failed", "error": "bad_request"}
    t0 = time.monotonic()
    try:
        out = snapshot_submission(cfg, agent_uid)
    except Exception as exc:  # noqa: BLE001 - a failed copy refuses the start; nothing is charged
        sup_log.write(kind="snapshot_failed", why=str(req.get("why"))[:40], err="%s: %s" % (type(exc).__name__, exc))
        return {"status": "failed", "error": "%s" % type(exc).__name__}
    out["wall_s"] = round(time.monotonic() - t0, 3)
    sup_log.write(kind="snapshot", why=str(req.get("why"))[:40], status=out["status"], files=out["files"],
                  bytes=out["bytes"], skipped=out["skipped"], reason=out["reason"], wall_s=out["wall_s"])
    return out


# ----------------------------------------------------------------------------------------------- the wire
def encode(obj):
    return (json.dumps(obj, allow_nan=False, separators=(",", ":")) + "\n").encode("utf-8")


def _jsonable_reply(obj):
    try:
        return encode(obj)
    except (TypeError, ValueError):
        return encode({"id": obj.get("id"), "error": "internal_error"})


def parse_request(line):
    """(id, method, params) as the offline grader's wire reads a request (params null -> {}; any other non-object is
    passed on for the cell to answer), or None for a line that is not one."""
    try:
        obj = json.loads(line)
    except (ValueError, RecursionError):
        return None
    if not isinstance(obj, dict) or "method" not in obj:
        return None
    params = obj.get("params")
    if params is None:
        params = {}
    return obj.get("id"), str(obj.get("method")), params


def peer_uid(sock):
    try:
        _pid, uid, _gid = struct.unpack("3i", sock.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED,
                                                              struct.calcsize("3i")))
        return uid
    except OSError:
        return None


def is_error(result):
    return isinstance(result, dict) and "error" in result and set(result) <= {"error", "detail"}


def _for_log(params, limit=4000):
    """The request's parameters for requests.jsonl (requests carry no images; bounded)."""
    try:
        text = json.dumps(params, separators=(",", ":"), allow_nan=True, default=str)
    except (TypeError, ValueError):
        text = repr(params)
    if len(text) <= limit:
        try:
            return json.loads(text)
        except ValueError:
            return text
    return text[:limit] + "...(truncated)"


def _json_safe(x):
    """A JSON-safe copy (non-finite numbers as null)."""
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if isinstance(x, dict):
        return {str(k): _json_safe(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_json_safe(v) for v in x]
    if x is None or isinstance(x, (str, bool, int)):
        return x
    return str(x)


# ----------------------------------------------------------------------------------------------- the worker
class Client(object):
    def __init__(self, sock, number, uid):
        self.sock, self.number, self.uid = sock, number, uid
        self.buf = bytearray()
        self.last_active = time.monotonic()


class Worker(object):
    """Serves the graded episode (single-threaded; the cell's renderer has its own threads)."""

    def __init__(self, cfg, listener, lab, agent, hidden, token_digest, cell_mods, log, reqlog, sup_fds=None):
        self.cfg = cfg
        self.sup = sup_fds                # (request fd, reply fd) of the pipe to the root supervisor (the freeze)
        self.listener = listener
        self.listener.setblocking(False)
        self.allowed = {0} | ({agent.pw_uid} if agent is not None else set())
        self.hidden = hidden
        self.episode_id = str(hidden["episode"])
        self.token_digest = token_digest
        self.SR, self.GR, self.SV = cell_mods
        self.log, self.reqlog = log, reqlog
        self.prep_s, self.graded_s = float(cfg["prep_s"]), float(cfg["graded_s"])
        self.state = read_json(state_path(cfg))
        for key in ("service_start_unix", "started", "ended"):
            if key not in self.state:
                raise ValueError("service.json lacks %r" % key)
        self.srv = None
        self.client = None
        self.n_conn = 0
        self.n_requests = 0               # cell requests of the episode (after the start; the service's own excluded)
        self.n_queries = 0
        self.n_status = 0
        self.dirty = False
        self.last_flush = 0.0
        self.record_writes = 0
        self.last_record = None
        self.calls_path = os.path.join(cfg["record_dir"], "calls.jsonl")
        self.calls_fd = None
        self.calls_lines = 0
        self.calls_size = 0
        self.last_digest = None
        self.limits = dict(self.state.get("limits") or {})
        self.seed_sha256 = self.state.get("seed_sha256")

    # -- persisted state
    def save_state(self, **updates):
        """Persist the state with `updates` (fsync); the state in memory changes only once it is on disk."""
        state = dict(self.state, **updates)
        write_json_atomic(state_path(self.cfg), state)
        self.state = state

    @property
    def started(self):
        return bool(self.state.get("started"))

    @property
    def ended(self):
        return bool(self.state.get("ended"))

    def prep_left(self, now=None):
        return float(self.state["service_start_unix"]) + self.prep_s - (now or time.time())

    def graded_left(self, now=None):
        return float(self.state["started_at_unix"]) + self.graded_s - (now or time.time())

    # -- the freeze (through the root supervisor)
    def request_snapshot(self, why):
        """Ask the root supervisor to freeze the agent's submission (dual mode): its reply, status "ok" | "too_large" |
        "failed" (a failure of the copy, a broken pipe, a timeout)."""
        if self.sup is None:
            return {"status": "failed", "error": "no supervisor"}
        req_w, rep_r = self.sup
        try:
            os.write(req_w, (json.dumps({"op": "snapshot", "why": why}) + "\n").encode())
        except OSError as exc:
            return {"status": "failed", "error": "request: %s" % type(exc).__name__}
        deadline, buf = time.monotonic() + float(self.cfg["snapshot_timeout_s"]), b""
        while b"\n" not in buf:
            left = deadline - time.monotonic()
            if left <= 0:
                self.sup = None                   # a late reply must never answer a later request
                return {"status": "failed", "error": "timeout"}
            try:
                ready = select.select([rep_r], [], [], left)[0]
            except InterruptedError:
                continue
            if not ready:
                continue
            chunk = os.read(rep_r, 65536)
            if not chunk:
                self.sup = None
                return {"status": "failed", "error": "the supervisor is gone"}
            buf += chunk
        try:
            out = json.loads(buf.split(b"\n", 1)[0])
        except ValueError:
            out = None
        return out if isinstance(out, dict) else {"status": "failed", "error": "bad reply"}

    def take_snapshot(self, why, attempts=1):
        """The freeze, through the supervisor; up to `attempts` tries while a copy fails (the last reply counts)."""
        out = {}
        for attempt in range(1, int(attempts) + 1):
            snap = self.request_snapshot(why)
            out = {k: snap.get(k) for k in ("status", "files", "dirs", "bytes", "sha256", "skipped", "reason",
                                            "wall_s", "error")}
            out["at"], out["attempts"], out["at_unix"] = why, attempt, round(time.time(), 3)
            self.log.write(kind="snapshot", **{k: v for k, v in out.items() if k != "sha256"})
            if out["status"] in ("ok", "too_large") or attempt >= int(attempts):
                return out
            time.sleep(float(self.cfg["snapshot_retry_delay_s"]))
        return out

    def close_practice(self, now=None):
        """Create the flag that ends practice (O13: once the graded episode has started, the rehearsal facility ends an
        open rehearsal run and refuses new ones). Idempotent; fsync before it returns. The flag's directory is the
        service account's (0755): the facility reads it, the agent can neither create nor remove it."""
        flag = self.cfg["practice_closed_flag"]
        if os.path.lexists(flag):
            return
        write_json_atomic(flag, {"closed_at_unix": round(now or time.time(), 3),
                                 "why": "the graded episode started: rehearsal is over for this session"}, mode=0o644)
        self.log.write(kind="practice_closed")

    # -- start-up
    def prepare(self):
        """After the privilege drop: resume the persisted state, build the graded site's cell. Returns a line for the
        log."""
        self.save_state(worker_starts=int(self.state.get("worker_starts", 0)) + 1)
        self.last_record = self._read_last_record()       # what graded_status reports once the episode is over
        self._open_calls_log()
        if self.started:
            self.close_practice()                         # (a worker that died between the flag and the start)
        if self.started and not self.ended:
            # the worker died or restarted with the episode running: it ends there (no resume); its call log, complete
            # up to the last call that was answered, stands for it (the verifier re-simulates it)
            self.save_state(ended=True, end_reason="worker_restart", ended_at_unix=time.time())
            rec = dict(self.last_record) if self.last_record is not None else self.record()
            rec.update(ended=True, end_reason="worker_restart", updated_at_unix=round(time.time(), 3),
                       wall_s_since_start=round(time.time() - float(self.state["started_at_unix"]), 3),
                       calls_log=self._calls_summary())
            self._write_record(rec)
            return "the episode was running when the worker died: ended (worker_restart); the call log stands for it"
        if self.ended or self.state.get("prep_closed"):
            return "the episode is over (%s); serving status only" % (self.state.get("end_reason") or "prep window")
        if self.prep_left() <= 0:
            self.close_prep()
            return "the preparation window closed before a start"
        t0 = time.monotonic()
        self.srv = self.build_cell()
        self._write_record(self.record())
        return "graded cell built in %.1f s; waiting for the start" % (time.monotonic() - t0)

    def build_cell(self):
        """The graded site's cell, as the offline grader builds a site's cell (tests/verify.py run_site): the frame
        renderer bound, the site package, the grader's seed rule (the set's seed, or the package's own when null), the
        run's label; the program's two wall-clock budgets switched off (cell_limits)."""
        h = self.hidden
        pkg = os.path.join(self.cfg["data_dir"], h["package"])
        with open(pkg, "rb") as fh:
            got = hashlib.sha256(fh.read()).hexdigest()
        if got != h["sha256"]:
            raise RuntimeError("the graded site's package does not match its sha256")
        srv = self.SR.make_server(privileged=False, reference_dir=os.path.join(self.cfg["cell_dir"], "reference"),
                                  limits=dict(self.cfg["cell_limits"]),
                                  log=lambda s: self.log.write(kind="cell", msg=str(s)[:4000]))
        srv.reset(pkg, seed=h.get("seed"), label=str(h.get("label") or ""))
        seed_sha = hashlib.sha256(str(int(srv.cell.seed)).encode("ascii")).hexdigest()
        if h.get("seed_sha256") and seed_sha != h["seed_sha256"]:
            raise RuntimeError("the graded site's noise seed is not the recorded one")
        if self.seed_sha256 != seed_sha:
            self.save_state(seed_sha256=seed_sha)
            self.seed_sha256 = seed_sha
        b = srv.cell.budgets
        limits = {"sim_time_max": float(b.sim_time_s), "colour_frames_max": int(b.colour_frames),
                  "depth_frames_max": int(b.depth_frames), "captures_max": int(b.capture_calls)}
        if limits != self.limits:
            self.save_state(limits=limits)
            self.limits = limits
        return srv

    def _read_last_record(self):
        try:
            rec = read_json(os.path.join(self.cfg["record_dir"], "record.json"))
            return rec if isinstance(rec, dict) and rec.get("format") == RECORD_FORMAT else None
        except (OSError, ValueError):
            return None

    # -- the call log (the verifier's re-simulation of an episode that the session or a failure ended)
    def _open_calls_log(self):
        fd = os.open(self.calls_path, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
        st = os.fstat(fd)
        if not stat.S_ISREG(st.st_mode) or st.st_nlink != 1:
            os.close(fd)
            raise RuntimeError("calls.jsonl is not a plain file")
        self.calls_fd, self.calls_size = fd, st.st_size
        if st.st_size:
            with open(self.calls_path, "rb") as fh:
                self.calls_lines = sum(1 for _ in fh)

    def _calls_summary(self):
        return {"file": "calls.jsonl", "lines": int(self.calls_lines), "bytes": int(self.calls_size),
                "last_state_sha256": self.last_digest}

    def calls_room(self, params):
        """Whether the call log can take one more call (the log must stay complete)."""
        try:
            n = len(json.dumps(params, separators=(",", ":"), default=str))
        except (TypeError, ValueError):
            n = 1 << 16
        return self.calls_size + n + 512 <= int(self.cfg["calls_log_max_bytes"])

    def log_call(self, method, params, result):
        """One call that reached the cell (every call but the four pure queries), in order: method, the parameters as
        the cell got them, the reply's code (or error) and mission time, and the physics state's digest after it."""
        if isinstance(result, dict):
            code = result.get("code") if "code" in result else result.get("error")
            t = result.get("t")
        else:
            code, t = None, None
        self.last_digest = state_digest(self.srv.cell.data)
        line = {"n": self.calls_lines + 1, "m": method, "p": params, "code": code,
                "t": t if isinstance(t, (int, float)) and not isinstance(t, bool) else None, "s": self.last_digest}
        try:
            raw = (json.dumps(line, separators=(",", ":"), allow_nan=True, default=str) + "\n").encode("utf-8")
        except (TypeError, ValueError):
            raw = (json.dumps(dict(line, p=None, p_repr=repr(params)[:2000])) + "\n").encode("utf-8")
        view = memoryview(raw)
        while len(view):
            view = view[os.write(self.calls_fd, view):]
        self.calls_lines += 1
        self.calls_size += len(raw)

    # -- the record
    def record(self):
        """The record (O7). While the episode runs it holds the counters; once it has ended, the grade of the cell's
        record (episode_score: the offline site score of the episode) and the cell record's name and sha256."""
        st = self.state
        now = time.time()
        rec = {
            "format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.episode_id,
            "seed_sha256": self.seed_sha256, "service_version": SERVICE_VERSION,
            "started": self.started, "ended": self.ended, "end_reason": st.get("end_reason") if self.ended else None,
            "prep_window_closed": bool(st.get("prep_closed")),
            "privileged": bool(st.get("privileged")), "oracle_token_mismatch": bool(st.get("oracle_token_mismatch")),
            "service_start_unix": float(st["service_start_unix"]), "started_at_unix": st.get("started_at_unix"),
            "started_after_s": (round(float(st["started_at_unix"]) - float(st["service_start_unix"]), 3)
                                if self.started else None),
            "updated_at_unix": round(now, 3),
            "wall_s_since_start": round(now - float(st["started_at_unix"]), 3) if self.started else 0.0,
            "windows": {"prep_s": self.prep_s, "graded_s": self.graded_s},
            "requests": int(self.n_requests), "queries": int(self.n_queries), "status_requests": int(self.n_status),
            "steps": 0, "sim_time_s": 0.0, "episode_score": None, "metrics": {}, "items": None,
            "cell_done_reason": None, "cell_exit": None, "cell_record": None, "score_error": None,
            "calls_log": self._calls_summary(),
            "record_writes": int(self.record_writes + 1),
            "requests_log_dropped": int(self.reqlog.dropped),
        }
        snap = st.get("snapshot") or {}
        rec.update({"snapshot_status": snap.get("status"), "snapshot_at": snap.get("at"),
                    "snapshot_at_unix": snap.get("at_unix"), "snapshot_attempts": int(snap.get("attempts") or 0),
                    "snapshot_files": int(snap.get("files") or 0), "snapshot_bytes": int(snap.get("bytes") or 0),
                    "snapshot_sha256": snap.get("sha256"), "snapshot_skipped": snap.get("skipped") or {},
                    "snapshot_reason": snap.get("reason")})
        srv = self.srv
        if srv is not None and srv.cell is not None:
            rec["steps"] = int(srv.cell.call_counts.get("step", 0))
            rec["sim_time_s"] = round(float(srv.cell.t_mission()), 4)
            rec["cell_done_reason"] = srv.done_reason if srv.done else None
        end = st.get("end") or {}
        if end:
            for k in ("episode_score", "metrics", "items", "cell_done_reason", "cell_exit", "cell_record",
                      "score_error", "steps", "sim_time_s"):
                if k in end:
                    rec[k] = end[k]
        return rec

    def _write_record(self, rec):
        rec = _json_safe(rec)
        try:
            write_json_atomic(os.path.join(self.cfg["record_dir"], "record.json"), rec)
            self.record_writes += 1
            self.last_record = rec
        except (OSError, TypeError, ValueError) as exc:
            self.log.write(kind="record_write_failed", err="%s: %s" % (type(exc).__name__, exc))
        self.last_flush = time.monotonic()
        self.dirty = False

    def flush(self):
        if self.started and not self.ended:
            self._write_record(self.record())
        else:
            self.dirty = False

    def flush_if_due(self, force=False):
        if self.dirty and (force or time.monotonic() - self.last_flush >= float(self.cfg["record_min_interval_s"])):
            self.flush()

    # -- the episode's end
    def end(self, reason):
        """End the running episode (`reason`: finish | done | graded_window | call_log_full) as the offline grader ends
        a run whose program stops at that moment: the cell's finalize() (the firmware's stop and hold, unless docked;
        the station on for the settle; the record), then the grade of that record against the site (cabincell.grade,
        the reference's energy on the site from the hidden set). Writes cell_record.json, then the final record."""
        if not self.started or self.ended:
            return
        srv = self.srv
        end = {"cell_record": None, "score_error": None}
        if srv is not None:
            t0 = time.monotonic()
            signal.alarm(max(1, int(math.ceil(float(self.cfg["end_cap_s"])))))   # a hang ends the worker
            try:
                if not srv.done:
                    srv._end("returned")                  # the program stopped: the run's end as when solve returns
                b = srv.cell.budgets
                end["steps"] = int(srv.cell.call_counts.get("step", 0))
                end["sim_time_s"] = round(float(srv.cell.t_mission()), 4)
                end["cell_done_reason"] = srv.done_reason
                end["used"] = {"colour_frames_used": int(b.n_colour), "depth_frames_used": int(b.n_depth),
                               "captures_used": int(b.n_capture)}
                cell_rec = srv.finalize(exit_code=0)
                raw = (json.dumps(cell_rec, sort_keys=True, separators=(",", ":")) + "\n").encode("ascii")
                sha = write_bytes_atomic(os.path.join(self.cfg["record_dir"], "cell_record.json"), raw)
                end["cell_record"] = {"file": "cell_record.json", "sha256": sha, "bytes": len(raw)}
                end["cell_exit"] = (cell_rec.get("times") or {}).get("exit")
                g = self.GR.grade(cell_rec, srv.cell.site, float(self.hidden["ref_energy_wh"]),
                                  self.GR.load_config(), self.GR.load_runtime())
                end["episode_score"] = float(g["score"])
                end["items"] = {k: float(v) for k, v in (g.get("items") or {}).items()}
                end["metrics"] = {k: float(v) for k, v in (g.get("metrics") or {}).items()
                                  if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)}
                end["R"] = float(g.get("R", 0.0))
            except Exception:  # noqa: BLE001 - an end that cannot be graded is recorded, never a crash of the service
                err = traceback.format_exc()
                self.log.write(kind="end_failed", tb=err[-4000:])
                end["score_error"] = err.strip().splitlines()[-1][:300]
            finally:
                signal.alarm(0)
            end["end_wall_s"] = round(time.monotonic() - t0, 3)
        self.save_state(ended=True, end_reason=reason, ended_at_unix=time.time(), end=_json_safe(end))
        self._write_record(self.record())
        self.log.write(kind="episode_end", reason=reason, cell_done_reason=end.get("cell_done_reason"),
                       sim_time_s=end.get("sim_time_s"), steps=end.get("steps"), requests=self.n_requests,
                       score_error=end.get("score_error"), end_wall_s=end.get("end_wall_s"))
        if srv is not None:
            try:
                srv.close()
            except Exception:  # noqa: BLE001
                pass
        self.srv = None
        gc.collect()

    def close_prep(self):
        """The preparation window closed without a start: the episode never runs (the record says so)."""
        if self.started or self.state.get("prep_closed"):
            return
        # the program freezes now (dual mode): it keeps main's preparation time whether or not the episode is flown.
        # A copy that still fails after the retries is recorded as failed, and the program item then scores 0: the
        # session-end submission must never stand in for it (that would give the program prep_s + graded_s)
        self.save_state(prep_closed=True,
                        snapshot=self.take_snapshot("prep_window", attempts=self.cfg["snapshot_attempts"]))
        if self.srv is not None:
            try:
                self.srv.close()
            except Exception:  # noqa: BLE001
                pass
            self.srv = None
            gc.collect()
        self._write_record(self.record())
        self.log.write(kind="prep_window_closed")

    def check_windows(self):
        now = time.time()
        if not self.started and not self.state.get("prep_closed") and self.prep_left(now) <= 0:
            self.close_prep()
        if self.started and not self.ended and self.graded_left(now) <= 0:
            self.end("graded_window")

    # -- requests
    def status(self):
        """graded_status: where the episode stands (never the score, the site or anything of the record)."""
        now = time.time()
        srv = self.srv
        last = self.last_record or {}
        reason = self.state.get("end_reason") if self.ended else None
        if reason == "done":
            done = (self.state.get("end") or {}).get("cell_done_reason") or last.get("cell_done_reason")
            reason = "sim_time" if done == "time_up" else "cell_error"
        else:
            reason = PUBLIC_END.get(reason, reason)
        if not self.started and self.state.get("prep_closed"):
            reason = "prep_window"
        live = srv is not None and srv.cell is not None
        b = srv.cell.budgets if live else None
        lim = self.limits
        used = (self.state.get("end") or {}).get("used") or {}
        out = {"started": self.started, "ended": self.ended or bool(self.state.get("prep_closed")), "end": reason,
               "sim_time": (round(float(srv.cell.t_mission()), 3) if live else float(last.get("sim_time_s") or 0.0)),
               "sim_time_max": lim.get("sim_time_max"),
               "steps": int(srv.cell.call_counts.get("step", 0)) if live else int(last.get("steps") or 0),
               "colour_frames_used": int(b.n_colour) if live else used.get("colour_frames_used"),
               "colour_frames_max": lim.get("colour_frames_max"),
               "depth_frames_used": int(b.n_depth) if live else used.get("depth_frames_used"),
               "depth_frames_max": lim.get("depth_frames_max"),
               "captures_used": int(b.n_capture) if live else used.get("captures_used"),
               "captures_max": lim.get("captures_max"),
               "prep_window_left_s": None, "graded_window_left_s": None,
               "submission_frozen": (self.state.get("snapshot") or {}).get("status")}
        if not self.started and not self.state.get("prep_closed"):
            out["prep_window_left_s"] = round(max(0.0, self.prep_left(now)), 1)
        if self.started and not self.ended:
            out["graded_window_left_s"] = round(max(0.0, self.graded_left(now)), 1)
        return out

    def op_start(self, params):
        if not isinstance(params, dict) or set(params) - {"oracle_token"}:
            return {"error": "bad_params", "detail": "graded_start takes no parameters"}
        if self.started:
            return {"error": "already_started", "detail": "the graded episode was started already (one per session); "
                                                          "graded_status() tells where it stands"}
        if self.state.get("prep_closed") or self.prep_left() <= 0:
            self.close_prep()
            return {"error": "prep_window_closed", "detail": "the graded episode can no longer be started: the "
                                                             "preparation window is over"}
        if self.srv is None:
            return {"error": "internal_error"}
        # the program freezes first (dual mode): a failed copy refuses the start, and nothing is charged
        snapshot = self.take_snapshot("start")
        if snapshot["status"] not in ("ok", "too_large"):
            return {"error": "snapshot_failed", "detail": "the submission folder could not be frozen, so the graded "
                                                          "episode did not start and nothing was charged; try again"}
        privileged = mismatch = False
        tok = params.get("oracle_token")
        if tok is not None:
            got = hashlib.sha256(str(tok).strip().encode("utf-8")).hexdigest()
            privileged = bool(self.token_digest) and got == self.token_digest
            mismatch = not privileged
        now = time.time()
        # practice ends first (a started episode always has practice closed), then the start is charged and persisted
        # before anything is simulated, and never refunded
        try:
            self.close_practice(now)
        except OSError as exc:
            self.log.write(kind="practice_close_failed", err="%s: %s" % (type(exc).__name__, exc))
            return {"error": "internal_error"}
        self.save_state(started=True, started_at_unix=now, privileged=privileged, oracle_token_mismatch=mismatch,
                        snapshot=snapshot)
        srv = self.srv
        srv.privileged = privileged                       # reference_run on this site; the site stays the graded one
        srv.wall_start, srv.work_wall, srv._last_reply = now, 0.0, None
        self.log.write(kind="episode_start", privileged=privileged, oracle_token_mismatch=mismatch,
                       prep_window_left_s=round(self.prep_left(now), 1), snapshot_status=snapshot.get("status"))
        self._write_record(self.record())
        return self.status()

    def dispatch(self, rid, method, params):
        """One request -> the reply object (never raises)."""
        t0 = time.time()
        self.check_windows()
        err = sim_time = None
        if method == "graded_status":
            self.n_status += 1
            result = self.status()
        elif method == "graded_start":
            result = self.op_start(params)
        elif method == "graded_finish":
            if not self.started:
                result = ({"error": "prep_window_closed", "detail": "the graded episode was never started"}
                          if self.state.get("prep_closed") else
                          {"error": "not_started", "detail": "start the graded episode first (graded_start())"})
            else:
                if not self.ended:
                    self.end("finish")
                result = self.status()
        elif not self.started:
            if self.state.get("prep_closed"):
                result = {"error": "prep_window_closed", "detail": "the graded episode was not started within the "
                                                                   "preparation window"}
            else:
                result = {"error": "not_started", "detail": "start the graded episode first (graded_start())"}
        elif self.ended or self.srv is None:
            result = {"error": "episode_over", "detail": "the graded episode is over; graded_status() tells how it ended"}
        elif method not in QUERIES and not self.calls_room(params):
            self.end("call_log_full")
            result = {"error": "episode_over", "detail": "the graded episode is over: the service's call log is full"}
        else:
            srv = self.srv
            if method in QUERIES:
                self.n_queries += 1
            else:
                self.n_requests += 1
            cap = float(self.cfg["privileged_request_cap_s"] if (method == "reference_run" and srv.privileged)
                        else self.cfg["request_cap_s"])
            signal.alarm(max(1, int(math.ceil(cap))))    # a hung request ends the worker (and with it the episode)
            try:
                result = srv.handle(method, params)
            finally:
                signal.alarm(0)
            sim_time = float(srv.cell.t_mission())
            if method not in QUERIES:
                self.log_call(method, params, result)
                self.dirty = True
            if srv.done:
                self.end("done")
        if is_error(result):
            reply = {"id": rid, "error": str(result["error"])}
            if "detail" in result:
                reply["detail"] = str(result["detail"])
            err = reply["error"]
        else:
            reply = {"id": rid, "result": result}
        if self.started or method in SERVICE_METHODS:
            if method == "graded_status":
                logged = None
            elif method == "graded_start" and isinstance(params, dict):
                logged = {k: ("<redacted>" if k == "oracle_token" else _for_log(v)) for k, v in params.items()}
            else:
                logged = _for_log(params)
            code = result.get("code") if isinstance(result, dict) and "code" in result else None
            self.reqlog.write(n=self.n_requests if method not in SERVICE_METHODS else None, kind=method[:80],
                              params=logged, code=code, error=err, wall_s=round(time.time() - t0, 4),
                              sim_time=None if sim_time is None else round(sim_time, 4),
                              since_start=round(time.time() - float(self.state["started_at_unix"]), 3)
                              if self.started else None)
        return reply

    # -- connections
    def _refuse(self, sock, code, detail):
        try:
            sock.settimeout(1.0)
            sock.sendall(encode({"id": None, "error": code, "detail": detail}))
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def _alive(self, c):
        try:
            return c.sock.recv(1, socket.MSG_PEEK | socket.MSG_DONTWAIT) != b""
        except (BlockingIOError, InterruptedError):
            return True
        except OSError:
            return False

    def drop(self, why):
        c = self.client
        if c is None:
            return
        self.client = None
        try:
            c.sock.close()
        except OSError:
            pass
        self.log.write(kind="connection_closed", conn=c.number, why=why)

    def accept(self):
        while True:
            try:
                sock, _ = self.listener.accept()
            except (BlockingIOError, InterruptedError):
                return
            except OSError as exc:
                self.log.write(kind="accept_failed", err=str(exc))
                return
            self.n_conn += 1
            uid = peer_uid(sock)
            if uid not in self.allowed:
                self.log.write(kind="connection_refused", conn=self.n_conn, uid=uid)
                self._refuse(sock, "forbidden", "this account may not connect")
                continue
            if self.client is not None:
                idle = time.monotonic() - self.client.last_active
                if self._alive(self.client) and idle < float(self.cfg["idle_takeover_s"]):
                    self.log.write(kind="connection_busy", conn=self.n_conn, uid=uid)
                    self._refuse(sock, "busy", "another connection is driving the graded episode; close it first "
                                               "(a connection idle for %.0f s gives way)" % float(self.cfg["idle_takeover_s"]))
                    continue
                self.drop("gone (a new connection arrived)" if idle < float(self.cfg["idle_takeover_s"]) else
                          "idle for %.0f s (a new connection arrived)" % idle)
            sock.setblocking(False)
            self.client = Client(sock, self.n_conn, uid)
            self.log.write(kind="connection", conn=self.n_conn, uid=uid)

    def send(self, c, obj):
        try:
            c.sock.settimeout(float(self.cfg["reply_timeout_s"]))
            c.sock.sendall(_jsonable_reply(obj))
            return True
        except OSError:
            return False
        finally:
            try:
                c.sock.setblocking(False)
            except OSError:
                pass

    def read_client(self):
        c = self.client
        try:
            data = c.sock.recv(1 << 16)
        except (BlockingIOError, InterruptedError):
            return
        except OSError:
            self.drop("error")
            return
        if not data:
            self.drop("disconnected")
            return
        c.last_active = time.monotonic()
        c.buf += data
        cap = int(self.cfg["max_line_bytes"])
        while self.client is c:
            i = c.buf.find(b"\n")
            if i < 0:
                if len(c.buf) > cap:
                    self.send(c, {"id": None, "error": "protocol", "detail": "request line too long"})
                    self.drop("request line too long")
                return
            line = bytes(c.buf[:i])
            del c.buf[:i + 1]
            if len(line) > cap:
                self.send(c, {"id": None, "error": "protocol", "detail": "request line too long"})
                self.drop("request line too long")
                return
            if not line.strip():
                continue
            req = parse_request(line.decode("utf-8", "replace"))
            if req is None:
                self.send(c, {"id": None, "error": "protocol", "detail": "a request is one JSON object with a method"})
                self.drop("protocol error")
                return
            reply = self.dispatch(*req)
            if not self.send(c, reply):
                self.drop("the reply could not be delivered")
                return
            c.last_active = time.monotonic()
            self.flush_if_due()

    def timeout(self):
        now = time.time()
        waits = [5.0]
        if not self.started and not self.state.get("prep_closed"):
            waits.append(self.prep_left(now))
        if self.started and not self.ended:
            waits.append(self.graded_left(now))
        if self.dirty:
            waits.append(float(self.cfg["record_min_interval_s"]) - (time.monotonic() - self.last_flush))
        return max(0.0, min(waits))

    def serve(self):
        while True:
            try:
                self.check_windows()
                rlist = [self.listener] + ([self.client.sock] if self.client is not None else [])
                try:
                    ready, _, _ = select.select(rlist, [], [], self.timeout())
                except InterruptedError:
                    continue
                if self.listener in ready:
                    self.accept()
                if self.client is not None and self.client.sock in ready:
                    self.read_client()
                self.flush_if_due()
            except Exception:  # noqa: BLE001 - the service outlives a failing connection
                self.log.write(kind="serve_loop_error", tb=traceback.format_exc()[-3000:])
                self.drop("service error")
                time.sleep(0.2)


def import_cell(cell_dir):
    """numpy, mujoco and the cell package (cell_dir/cabincell: the offline grader's own cell, a byte copy of
    tests/cabincell), its grader and its server, after the privilege drop: the data folder is the service account's,
    the task assets readable through their group."""
    import numpy  # noqa: F401
    import mujoco
    if cell_dir not in sys.path:
        sys.path.insert(0, cell_dir)
    import cabincell  # noqa: F401  (puts the cell's _lib and its own folder on sys.path)
    from cabincell import grade as GR
    from cabincell import sensors_render as SR
    from cabincell import server as SV
    return (SR, GR, SV), mujoco.__version__


def run_worker(cfg, listener, ready_fd, lab, agent, gids, sup_fds):
    """Forked from the supervisor as root: the hidden episode, the privilege drop, the cell, serving."""
    _set_comm("arb-graded-wrk")
    os.umask(0o077)
    rdir = cfg["record_dir"]
    log = Log(os.path.join(rdir, "service.log"), cfg["service_log_max_bytes"])
    reqlog = Log(os.path.join(rdir, "requests.jsonl"), cfg["requests_log_max_bytes"])
    for fh in (log, reqlog):
        fh.chown(lab.pw_uid, lab.pw_gid)
    calls = os.path.join(rdir, "calls.jsonl")
    if not os.path.lexists(calls):
        os.close(os.open(calls, os.O_WRONLY | os.O_CREAT | os.O_NOFOLLOW, 0o600))
    os.chown(calls, lab.pw_uid, lab.pw_gid, follow_symlinks=False)
    try:
        for k, v in cfg["env"].items():
            os.environ[k] = v
        hidden = read_json(cfg["episode_file"], limit=1 << 16)
        if not isinstance(hidden, dict) or not isinstance(hidden.get("episode"), str) or \
                not isinstance(hidden.get("package"), str) or not isinstance(hidden.get("sha256"), str) or \
                (hidden.get("seed") is not None and (isinstance(hidden.get("seed"), bool) or
                                                    not isinstance(hidden.get("seed"), int))) or \
                not _finite(hidden.get("ref_energy_wh"), 0.0) > 0.0:
            raise ValueError("the hidden episode file needs 'episode', 'package', 'sha256', an integer or null 'seed' "
                             "and a positive 'ref_energy_wh'")
        try:
            with open(cfg["oracle_token_digest_file"]) as fh:
                token_digest = fh.read().strip().lower()
        except OSError:
            token_digest = ""
        # -- the privilege drop (irreversible)
        os.chdir(rdir)                       # MuJoCo writes MUJOCO_LOG.TXT into the working directory
        os.setgroups(list(gids))
        os.setgid(lab.pw_gid)
        os.setuid(lab.pw_uid)
        try:
            os.setuid(0)
            raise RuntimeError("the privilege drop is reversible")
        except PermissionError:
            pass
        _prctl(PR_SET_DUMPABLE, 0)
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
        sec_ok = seccomp_selftest()
        if cfg["require_seccomp"] and not sec_ok:
            raise RuntimeError("seccomp self-test failed: an AF_INET socket could be created")
        cell_mods, mj_version = import_cell(cfg["cell_dir"])
        worker = Worker(cfg, listener, lab, agent, hidden, token_digest, cell_mods, log, reqlog, sup_fds)
        note = worker.prepare()
        log.write(kind="worker_ready", uid=os.getuid(), gid=os.getgid(), groups=os.getgroups(),
                  seccomp_inet_blocked=sec_ok, mujoco=mj_version, version=SERVICE_VERSION, note=note,
                  state={k: v for k, v in worker.state.items() if k not in ("service_start_unix", "end")})
    except BaseException:  # noqa: BLE001
        log.write(kind="worker_start_failed", tb=traceback.format_exc()[-6000:])
        try:
            os.write(ready_fd, EXIT_FAILED)
        except OSError:
            pass
        os._exit(1)
    os.write(ready_fd, EXIT_READY)
    os.close(ready_fd)
    try:
        worker.serve()
    except BaseException:  # noqa: BLE001
        log.write(kind="worker_crashed", tb=traceback.format_exc()[-6000:])
    os._exit(1)


# ----------------------------------------------------------------------------------------------- the supervisor
def _close_fds(keep):
    for fd in range(3, 1024):
        if fd not in keep:
            try:
                os.close(fd)
            except OSError:
                pass


def run_supervisor(cfg, lab, agent, gids, notify_fd):
    _set_comm("arb-graded-sup")
    os.umask(0o077)
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    sup_log = Log(os.path.join(cfg["record_dir"], "supervisor.log"), cfg["service_log_max_bytes"])
    try:
        install_unix_only_seccomp()
        seccomp = True
    except OSError as exc:
        seccomp = False
        sup_log.write(kind="seccomp_install_failed", err=str(exc))
        if cfg["require_seccomp"]:
            os.write(notify_fd, EXIT_FAILED)
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
    os.chmod(sock_path, 0o666)        # before the chown: without CAP_FOWNER root cannot chmod a file it does not own
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
    agent_uid = agent.pw_uid if agent is not None else None
    restarts, first = 0, True
    while not stopping["flag"]:
        r, w = os.pipe()                     # the worker's ready byte
        req_r, req_w = os.pipe()             # the worker's snapshot requests (its only way to ask root for anything)
        rep_r, rep_w = os.pipe()             # the supervisor's replies
        pid = os.fork()
        if pid == 0:
            for fd in (r, req_r, rep_w):
                os.close(fd)
            try:
                os.close(sup_log.fd)
            except OSError:
                pass
            if notify_fd is not None:
                try:
                    os.close(notify_fd)
                except OSError:
                    pass
            for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
                signal.signal(sig, signal.SIG_DFL)
            run_worker(cfg, listener, w, lab, agent, gids, (req_w, rep_r))
            os._exit(1)
        for fd in (w, req_w, rep_r):
            os.close(fd)
        stopping["child"] = pid
        status, code = b"", None
        ready_open, req_open, buf = True, True, b""

        def notify(st):
            nonlocal notify_fd
            if notify_fd is not None:
                try:
                    os.write(notify_fd, EXIT_READY if st == EXIT_READY else EXIT_FAILED)
                    os.close(notify_fd)
                except OSError:
                    pass
                notify_fd = None

        while code is None:
            fds = ([r] if ready_open else []) + ([req_r] if req_open else [])
            try:
                ready = select.select(fds, [], [], 1.0)[0] if fds else []
            except InterruptedError:
                ready = []
            if not fds:
                time.sleep(0.5)
            if r in ready:
                try:
                    status = os.read(r, 1)
                except OSError:
                    status = b""
                ready_open = False
                os.close(r)
                if first:
                    first = False
                    notify(status)
            if req_r in ready:
                try:
                    data = os.read(req_r, 4096)
                except OSError:
                    data = b""
                if not data:
                    req_open = False
                else:
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        reply = serve_snapshot_request(cfg, agent_uid, line[:4096], sup_log)
                        try:
                            os.write(rep_w, (json.dumps(reply, separators=(",", ":")) + "\n").encode())
                        except OSError:
                            pass
                    if len(buf) > 4096:
                        buf = b""
            try:
                done, st = os.waitpid(pid, os.WNOHANG)
            except ChildProcessError:
                done, st = pid, -1
            except InterruptedError:
                done, st = 0, None
            if done:
                code = st
        if ready_open:
            try:
                os.set_blocking(r, False)
                status = os.read(r, 1)
            except OSError:
                pass
            os.close(r)
        if first:
            first = False
            notify(status)
        for fd in (req_r, rep_w):
            try:
                os.close(fd)
            except OSError:
                pass
        stopping["child"] = None
        sup_log.write(kind="worker_exit", pid=pid, status=code, ready=status == EXIT_READY)
        if stopping["flag"]:
            break
        if status != EXIT_READY:
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


def _check_data_dir(cfg, lab):
    """The service account's data folder: a directory of its own, mode 0700 (the agent can neither list nor read it)."""
    st = os.lstat(cfg["data_dir"])
    if not stat.S_ISDIR(st.st_mode) or st.st_uid != lab.pw_uid or (st.st_mode & 0o077):
        raise RuntimeError("%s must be a directory of the service account with mode 0700" % cfg["data_dir"])
    for key in ("episode_file", "oracle_token_digest_file"):
        if not os.path.isfile(cfg[key]):
            raise RuntimeError("%s is missing" % cfg[key])
    if not os.path.isfile(os.path.join(cfg["cell_dir"], "cabincell", "server.py")):
        raise RuntimeError("the cell is missing from %s" % cfg["cell_dir"])


def start(config_path):
    if os.geteuid() != 0:
        print("graded service: must be started as root (from environment/setup/setup.sh)", file=sys.stderr)
        return 2
    try:
        cfg = load_config(config_path)
        lab, agent, gids = _accounts(cfg)
        _check_data_dir(cfg, lab)
        os.umask(0o077)
        sockdir = os.path.dirname(cfg["socket"])
        os.makedirs(os.path.dirname(sockdir), exist_ok=True)
        _secure_dir(sockdir, 0o755, 0, 0)
        _secure_dir(cfg["flags_dir"], 0o755, lab.pw_uid, lab.pw_gid)
        _secure_dir(cfg["record_dir"], 0o700, lab.pw_uid, lab.pw_gid)
        sp = state_path(cfg)
        if not os.path.lexists(sp):          # the service's first start: the preparation window opens now
            write_json_atomic(sp, {"service_start_unix": time.time(), "started": False, "ended": False,
                                   "worker_starts": 0})
            os.chown(sp, lab.pw_uid, lab.pw_gid)
    except Exception as exc:  # noqa: BLE001
        print("graded service: configuration error: %s: %s" % (type(exc).__name__, exc), file=sys.stderr)
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
            run_supervisor(cfg, lab, agent, gids, w)
        except BaseException:  # noqa: BLE001
            try:
                Log(os.path.join(cfg["record_dir"], "supervisor.log"), cfg["service_log_max_bytes"]).write(
                    kind="supervisor_crashed", tb=traceback.format_exc()[-6000:])
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
    if status == EXIT_READY:
        print("graded service: ready (socket %s)" % cfg["socket"], file=sys.stderr)
        return 0
    print("graded service: failed to start (details in its own log)", file=sys.stderr)
    return 1


def main(argv=None):
    ap = argparse.ArgumentParser(description="the graded episode service (start as root from setup.sh)")
    ap.add_argument("--config", required=True)
    args = ap.parse_args(argv)
    return start(args.config)


if __name__ == "__main__":
    sys.exit(main())
