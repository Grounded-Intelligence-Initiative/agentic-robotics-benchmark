#!/usr/bin/env python3
"""arb-online: the graded service of shoe_lacing_and_boxing (the dual mode; experimental).

The agent plays ONE hidden episode itself, during its session, through this service: the offline verifier's own
cell (shoecell.server.ShoeCellServer, mode "hidden", with the grader's configuration) on the graded episode's seed.
The service writes the episode's outcome to a directory the agent can neither read nor write; the engine copies that
directory into the separate verifier as a declared artifact, and tests/verify.py scores the record.

Started once by environment/setup/setup.sh as root, cwd = the stage (/opt/ale/setup):

    env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \\
        python3 -I -B online/server.py --config online/online.json

The command returns once the service serves (exit 0) or failed to start (exit 1); the service itself is a fully
detached daemon (setsid, stdio on /dev/null) and survives into the agent phase.

Process layout (the alelab daemon pattern of arm_hidden_mechanism_box)
  supervisor (root, comm "arbonline-sup"): binds the unix socket, initialises the service state, forks and restarts
      the worker (bounded) and, on the worker's request over a pipe pair, freezes the agent's submission folder at the
      start (below). It never reads the graded episode and never accepts a connection.
  worker (comm "arbonline-wrk"): forked as root, reads the graded episode (episode.json: id and seed) and the digest
      of the maintainers' reference token, checks the service's root-owned copy of the givens against their pinned
      sha256 (given_hashes.json), then drops to the dedicated service account irreversibly (setgroups([]), setgid,
      setuid; not dumpable, no core files), builds the cell on the seed and serves the socket. It never runs as root
      or as the agent account while serving.
  Both run under PR_SET_NO_NEW_PRIVS and a seccomp filter under which socket()/socketpair() work only for AF_UNIX
  (no TCP/UDP at any time).

Protocol: the cell's own JSON-lines wire over the unix socket, one object per line in each direction.
  On connect the service sends one greeting line ({"service": ..., "protocol": 1, ...}), or {"error": "busy"} and
  closes when another client is connected (one client at a time).
  Requests {"id": n, "method": m, "params": {...}} -> {"id": n, "result": ...} or {"id": n, "error": code[, "detail"]}.
  Service methods: status (always), start (once per session). Every other method goes to the cell's own dispatch
  (ShoeCellServer.handle: the public calls; anything else answers "forbidden" and is counted, as on the grader's
  wire) once the graded episode has started.

The graded episode
  * One start per session: `start` is charged and persisted (state.json, fsync) before anything is simulated and is
    never refunded. It is accepted only within `prep_s` of the service's first start.
  * The episode lives in the service, not in a connection: a disconnect leaves it where it is; the next connection
    resumes it. A connection idle for `idle_timeout_s` is closed (the episode stays).
  * It ends at its own end (the cell's done: sim cap, physics fault, internal error), at the agent's `finish`, or
    `graded_s` after its start; requests after that are refused (status still answers; after the cell's own end the
    program may still read its state, within the wire's after-done limits). A worker that dies or restarts ends it
    with its last recorded state (end_reason worker_restart; nothing is resumed). The end is the offline grader's
    end of an episode whose program stops at that moment: the cell is ended and scored as it stands
    (ShoeCellServer.finalize), nothing is simulated afterwards. If the session ends first, the last record applies.
  * The simulation advances only inside the agent's requests; there is no program-time, silence or wall-clock budget
    on the requests (the per-episode limits of the cell, its sim cap, step cap, camera budget and safety rules, are the
    cell's own and apply unchanged).
  * The agent learns what a program learns (the cell's replies) plus `status`: started / ended, steps, simulated
    time, time left in each window. Never the score, the record, the episode id or the seed.

The freeze (the dual mode). The offline item grades the program in /home/user/submission as it was at the earlier of
the graded start and the close of the start window (`prep_s` after the service's first start): before the start is
charged, or the moment the window closes without a start, the worker asks the supervisor (one byte on a pipe; no path,
no argument), which copies the folder into record_dir/submission_snapshot (root-owned 0700 / 0600: neither the agent nor
the service account can read it) with the grader's own snapshot rules (regular files and folders only; links, special
files and __pycache__ dropped; no link followed at any level) and the grader's submission cap (60 MB; over it: status
too_large, no copy, the offline item scores 0). A failed copy refuses the start and charges nothing; at the window's
close it is tried three times, then recorded as `error`. When the window closes without a start the record says so
(started false, prep_window_closed true, the snapshot fields). The record carries
snapshot_status, snapshot_files, snapshot_bytes and snapshot_sha256 (the digest of the sorted (path, size, sha256)
list), which tests/verify.py checks against the copy before it grades it.

Record: record_dir/record.json, rewritten atomically (temp file, fsync, rename, fsync of the directory) when a
request changed the episode and `record_every_s` has passed since the last write or the connection went quiet for
`idle_flush_s`, and always when the episode starts and ends. `episode_score` is the cell's work score of the episode
as if it ended at that moment. record_dir/requests.jsonl logs every request (method, parameters, simulated time; no
images; the reference token redacted) until it reaches `requests_log_max_bytes`. record_dir/service.log and
supervisor.log are the service's own logs (size-capped).

Python 3.12 standard library; the cell needs the task image's /opt/venv (numpy, mujoco).
"""

import argparse
import ctypes
import hashlib
import hmac
import json
import math
import os
import platform
import pwd
import re
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

SERVICE_VERSION = "shoe-lacing-and-boxing-dual/1"
RECORD_FORMAT = "arb-online-record/1"
PROTOCOL = 1
END_REASONS = ("done", "finish", "graded_window", "worker_restart")
CAMERAS = ("overhead", "front", "side", "wrist_left", "wrist_right")
MILESTONES = ("lacing_a", "lacing_b", "erected", "packed", "closed", "shipped")
# cell calls that only read: everything else may change the episode (the physics, the frame budget and the sensor
# noise stream, the illegal-call count), so the record is rewritten after it
READ_ONLY_CALLS = ("describe", "get_state")

ERRORS = {
    "bad_json": "the request is not one valid JSON object on one line",
    "line_too_long": "the request line exceeds the size limit",
    "bad_request": "a request is {\"id\": n, \"method\": name, \"params\": {...}}",
    "busy": "another client is connected to the graded cell (one at a time): end that process or close its "
            "connection, then connect again",
    "not_started": "the graded episode has not been started: call start() first (once per session)",
    "already_started": "the graded episode was started already (one start per session)",
    "prep_window_closed": "the window for starting the graded episode has closed; it can no longer be started (your "
                          "program was frozen when it closed)",
    "episode_over": "the graded episode has ended; only status answers",
    "service_broken": "the graded service cannot persist its state; the graded episode cannot be started",
    "snapshot_failed": "your submission folder could not be frozen, so the graded episode was not started and nothing "
                       "was charged; start() again",
    "internal": "internal error of the graded service",
}

DEFAULTS = {
    "task": None,                    # the task's slug (the record's `task`)
    "user": "arbonline",             # the dedicated service account (created in the task image)
    "agent_user": "user",            # the evaluated account; the service refuses to run as it
    "socket": "/run/arb-online/cell.sock",
    "record_dir": "/var/lib/arb-online",
    "given_dir": None,               # the service's root-owned copy of the givens (setup.sh builds it)
    "private_dir": None,             # the reference controller, readable by the service account only
    "episode": None,                 # the graded episode (root-only), relative to the config file
    "cell_config": None,             # the grader's cell configuration (a byte copy of tests/grader_config.json)
    "given_hashes": None,            # the givens' pinned sha256 (a byte copy of tests/given_hashes.json)
    "token_digest": None,            # sha256 of the maintainers' reference token (root-only)
    "prep_s": None,                  # the window for the start, from the service's first start
    "graded_s": None,                # the graded episode's window, from its start
    "record_every_s": 2.0,
    "idle_flush_s": 0.5,
    "idle_timeout_s": 900.0,
    "reply_timeout_s": 120.0,
    "max_line_bytes": 1 << 20,
    "after_done_max_steps": 50,      # the grader's wire: once the cell's episode is over (sim cap, physics fault)
    "after_done_max_requests": 200,  # the program may still read its state, this much
    "requests_log_max_bytes": 32 << 20,
    "service_log_max_bytes": 64 << 20,
    "max_restarts": 5,
    "start_timeout_s": 900.0,
    "require_seccomp": True,
    "submission_dir": "/home/user/submission",   # the agent's program, frozen at the start (the dual mode)
    "snapshot_max_bytes": 60 << 20,  # the grader's submission cap (grader_config.json route_gate.max_submission_mb)
    "snapshot_max_entries": 50000,   # files and folders: below the engine's artifact bound (100000 entries)
    "snapshot_max_depth": 32,        # folder depth: below the engine's artifact bound (64)
    "snapshot_timeout_s": 300.0,
    "env": {},                       # PUBLIC settings of the cell's process (renderer, BLAS threads); never hidden values
}
ID_RE = re.compile(r"^[A-Za-z0-9_.-]{1,64}$")
METHOD_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]{0,63}$")
HEX64_RE = re.compile(r"^[0-9a-f]{64}$")

PR_SET_NAME = 15
PR_CAPBSET_DROP = 24
PR_SET_DUMPABLE = 4
PR_SET_NO_NEW_PRIVS = 38
PR_SET_SECCOMP = 22
SECCOMP_MODE_FILTER = 2
SECCOMP_RET_ALLOW = 0x7FFF0000
SECCOMP_RET_ERRNO = 0x00050000
EPERM = 1
# capabilities the root supervisor keeps (worker re-fork as root, then the drop): CHOWN, DAC_OVERRIDE, KILL, SETGID,
# SETUID
SUPERVISOR_CAPS = (0, 1, 5, 6, 7)
# machine -> (AUDIT_ARCH, nr socket, nr socketpair, nr io_uring_setup, x32 syscall bit)
SECCOMP_ARCH = {
    "x86_64": (0xC000003E, 41, 53, 425, 0x40000000),
    "aarch64": (0xC00000B7, 198, 199, 425, 0),
}


class ReplyError(Exception):
    def __init__(self, code, detail=None):
        super().__init__(code)
        self.code = code
        self.detail = ERRORS.get(code) if detail is None else detail


def now_utc():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


# ----------------------------------------------------------------------------------------------------------------
# process hardening (alelab's)

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
            pass  # beyond the kernel's last capability (EINVAL)

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
    """True when an AF_INET/AF_INET6 socket cannot be created and AF_UNIX still can."""
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


def drop_privileges(account):
    """setgroups([]), setgid, setuid to `account`; refuses to continue if root can be regained. Not dumpable, no
    core files."""
    os.chdir("/")
    os.setgroups([])
    os.setgid(account.pw_gid)
    os.setuid(account.pw_uid)
    try:
        os.setuid(0)
    except PermissionError:
        pass
    else:
        raise RuntimeError("the privilege drop is reversible")
    _prctl(PR_SET_DUMPABLE, 0)
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


# ----------------------------------------------------------------------------------------------------------------
# files: size-capped logs, atomic JSON

class Log(object):
    """Append-only JSON-lines log (0600, O_NOFOLLOW, a plain file with one link), size-capped: once a line would pass
    the cap, nothing more is written."""

    def __init__(self, path, cap):
        self.path, self.cap = path, int(cap)
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
        st = os.fstat(fd)
        if not stat.S_ISREG(st.st_mode) or st.st_nlink != 1:
            os.close(fd)
            raise RuntimeError("log path is not a plain file: %s" % path)
        self.fh = os.fdopen(fd, "a", buffering=1)
        self.size = st.st_size
        self.full = False

    def write(self, **rec):
        rec = dict({"t": round(time.time(), 3)}, **rec)
        line = json.dumps(_clean(rec), separators=(",", ":"), allow_nan=False) + "\n"
        if self.size + len(line) > self.cap:
            self.full = True
            return False
        try:
            self.fh.write(line)
            self.size += len(line)
        except OSError:
            return False
        return True


def _clean(x):
    """JSON-safe copy: numpy scalars and arrays as plain values, non-finite floats as null, tuples as lists."""
    if isinstance(x, dict):
        return {str(k): _clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_clean(v) for v in x]
    if isinstance(x, bool) or x is None or isinstance(x, str):
        return x
    if isinstance(x, int):
        return x
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if hasattr(x, "tolist"):
        return _clean(x.tolist())
    if hasattr(x, "item"):
        return _clean(x.item())
    return str(x)


def atomic_write_json(path, obj):
    """Write `obj` to `path` atomically: a temp file in the same directory, fsync, rename, fsync of the directory."""
    data = (json.dumps(_clean(obj), indent=1, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")
    tmp = path + ".tmp"
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
    try:
        view = memoryview(data)
        while view:
            view = view[os.write(fd, view):]
        os.fsync(fd)
    finally:
        os.close(fd)
    os.replace(tmp, path)
    dfd = os.open(os.path.dirname(path) or ".", os.O_RDONLY)
    try:
        os.fsync(dfd)
    finally:
        os.close(dfd)


def read_json_file(path, max_bytes=1 << 20):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            raise ValueError("not a regular file: %s" % path)
        with os.fdopen(fd, "rb") as fh:
            fd = None
            blob = fh.read(max_bytes + 1)
    finally:
        if fd is not None:
            os.close(fd)
    if len(blob) > max_bytes:
        raise ValueError("file too large: %s" % path)
    return json.loads(blob)


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


# ----------------------------------------------------------------------------------------------------------------
# configuration and the service state

def load_config(path):
    path = os.path.abspath(path)
    raw = read_json_file(path)
    if not isinstance(raw, dict):
        raise ValueError("the config must be a JSON object")
    unknown = set(raw) - set(DEFAULTS)
    if unknown:
        raise ValueError("unknown config keys: %s" % sorted(unknown))
    cfg = dict(DEFAULTS)
    cfg.update(raw)
    base = os.path.dirname(path)
    for key in ("episode", "cell_config", "given_hashes", "token_digest"):
        if not isinstance(cfg[key], str) or not cfg[key]:
            raise ValueError("the config needs %r" % key)
        cfg[key] = os.path.normpath(os.path.join(base, cfg[key]))
    for key in ("socket", "record_dir", "given_dir", "private_dir"):
        if not isinstance(cfg[key], str) or not os.path.isabs(cfg[key]):
            raise ValueError("%s must be an absolute path" % key)
    if os.path.dirname(cfg["socket"]) in ("/", "/tmp", "/var/tmp", "/dev/shm", "/run"):
        raise ValueError("the socket must live in its own root-owned directory")
    for key in ("prep_s", "graded_s"):
        if not isinstance(cfg[key], (int, float)) or isinstance(cfg[key], bool) or not cfg[key] > 0:
            raise ValueError("the config needs a positive %r" % key)
    if not isinstance(cfg["task"], str) or not ID_RE.match(cfg["task"]):
        raise ValueError("the config needs the task's slug")
    if not isinstance(cfg["env"], dict) or not all(isinstance(k, str) and isinstance(v, str) for k, v in cfg["env"].items()):
        raise ValueError("env must map strings to strings")
    return cfg


def accounts(cfg):
    lab = pwd.getpwnam(cfg["user"])
    if lab.pw_uid == 0:
        raise RuntimeError("the service account must not be root")
    try:
        agent = pwd.getpwnam(cfg["agent_user"])
    except KeyError:
        agent = None
    if agent is not None and agent.pw_uid == lab.pw_uid:
        raise RuntimeError("the service account must differ from the agent account")
    return lab


def secure_dir(path, mode, uid, gid):
    """Create (or check) a directory owned by uid:gid with exactly `mode`; refuse a symlink or a foreign owner."""
    try:
        os.mkdir(path, mode)
    except FileExistsError:
        pass
    st = os.lstat(path)
    if not stat.S_ISDIR(st.st_mode):
        raise RuntimeError("%s exists and is not a directory" % path)
    if st.st_uid not in (0, uid):
        raise RuntimeError("%s has an unexpected owner" % path)
    parent = os.lstat(os.path.dirname(path))
    if parent.st_uid != 0 or (parent.st_mode & 0o022):
        raise RuntimeError("the parent of %s must be root-owned and not group/world-writable" % path)
    os.chown(path, uid, gid)
    os.chmod(path, mode)


def state_path(cfg):
    return os.path.join(cfg["record_dir"], "state.json")


def initialize_state(cfg, uid, gid):
    """The service's persistent state (record_dir/state.json), created once, at the first start of the service:
    the clock of the start window. Never reset: a missing or unreadable state after that means no start (fail
    closed)."""
    path = state_path(cfg)
    if os.path.lexists(path):
        return False
    atomic_write_json(path, {"version": 1, "service_started_utc": now_utc(), "service_started_mono": time.monotonic(),
                             "started": False, "started_utc": None, "started_mono": None, "privileged": False,
                             "ended": False, "end_reason": None, "ended_utc": None})
    os.chown(path, uid, gid)
    return True


def read_state(cfg):
    st = read_json_file(state_path(cfg), 1 << 16)
    if not isinstance(st, dict) or st.get("version") != 1 or not isinstance(st.get("service_started_mono"), (int, float)):
        raise ValueError("malformed service state")
    return st


# ----------------------------------------------------------------------------------------------------------------
# requests

def parse_request(line):
    """One request line -> (id, method, params). The cell's wire format; the parameters are handed to the cell as
    they are (it answers malformed ones with bad_params, as on the grader's wire)."""
    try:
        obj = json.loads(line.decode("utf-8"))
    except (ValueError, RecursionError):        # UnicodeDecodeError is a ValueError
        raise ReplyError("bad_json")
    if not isinstance(obj, dict) or not set(obj) <= {"id", "method", "params"} or "method" not in obj:
        raise ReplyError("bad_request")
    rid = obj.get("id")
    if not (rid is None or isinstance(rid, (int, str)) and not isinstance(rid, bool)) or (isinstance(rid, str) and len(rid) > 64):
        raise ReplyError("bad_request")
    method = obj["method"]
    if not isinstance(method, str) or not METHOD_RE.match(method):
        raise ReplyError("bad_request", "the method must be a name")
    params = obj.get("params")
    return rid, method, ({} if params is None else params)


def is_error(result):
    """A cell answer that is an error reply ({"error": code} or {"error": code, "detail": text}), as on the wire."""
    return isinstance(result, dict) and "error" in result and set(result) <= {"error", "detail"}


def log_params(method, params):
    """The request's parameters for requests.jsonl: the reference token redacted, anything large summarised."""
    if not isinstance(params, dict):
        return {"_type": type(params).__name__}
    out = {}
    for k, v in params.items():
        out[str(k)[:64]] = "<redacted>" if k == "token" else v
    try:
        text = json.dumps(_clean(out), allow_nan=False)
    except (TypeError, ValueError):
        return {"_unloggable": True}
    return out if len(text) <= 4096 else {"_bytes": len(text)}


def check_givens(root, hashes):
    """The service's copy of the givens (`root`: given/, example/, practice/) against their pinned sha256: every
    file root-owned, a regular file, not group/world-writable, with the pinned content. Returns the problems."""
    problems = []
    if not isinstance(hashes, dict) or not hashes:
        return ["no pinned sha256"]
    for rel, digest in sorted(hashes.items()):
        path = os.path.join(root, rel)
        try:
            st = os.lstat(path)
        except OSError:
            problems.append("missing " + rel)
            continue
        if not stat.S_ISREG(st.st_mode) or st.st_uid != 0 or (st.st_mode & 0o022):
            problems.append("not a root-owned, group/world-unwritable regular file: " + rel)
        elif sha256_file(path) != digest:
            problems.append("differs " + rel)
    return problems


# ----------------------------------------------------------------------------------------------------------------
# the freeze (the dual mode): the agent's program is copied when the graded episode starts

SNAPSHOT_DIR = "submission_snapshot"            # in record_dir: root-owned 0700, files 0600
SNAPSHOT_NAME_RE = re.compile(r"^[\x21-\x7e][\x20-\x7e]{0,254}$")   # printable ASCII names (the artifact copy keeps them)


class _TooLarge(Exception):
    pass


def snapshot_digest(entries):
    """sha256 of the sorted list of (relative path, size, sha256 of the content) of the snapshot's files: the record's
    `snapshot_sha256`, which tests/verify.py recomputes from the copy it receives."""
    h = hashlib.sha256()
    for rel, size, digest in sorted(entries):
        h.update(("%s\0%d\0%s\n" % (rel, size, digest)).encode("utf-8"))
    return h.hexdigest()


def _clear_path(path):
    """Remove whatever is at `path`, never following a link (shutil.rmtree works through folder descriptors)."""
    try:
        st = os.lstat(path)
    except FileNotFoundError:
        return
    if stat.S_ISDIR(st.st_mode):
        shutil.rmtree(path)
    else:
        os.unlink(path)


def _open_dir(name, dir_fd=None):
    return os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=dir_fd)


def _snapshot_file(name, sfd, dfd, budget):
    """Copy one regular file (by name inside the open folders); (bytes, sha256), None when it is no longer a regular
    file; raises _TooLarge past `budget` bytes."""
    fs = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC, dir_fd=sfd)
    try:
        if not stat.S_ISREG(os.fstat(fs).st_mode):
            return None
        fo = os.open(name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600, dir_fd=dfd)
        try:
            h, n = hashlib.sha256(), 0
            while True:
                chunk = os.read(fs, 1 << 20)
                if not chunk:
                    break
                n += len(chunk)
                if n > budget:
                    raise _TooLarge()
                h.update(chunk)
                view = memoryview(chunk)
                while view:
                    view = view[os.write(fo, view):]
        finally:
            os.close(fo)
    finally:
        os.close(fs)
    return n, h.hexdigest()


def _snapshot_walk(sfd, dfd, rel, depth, st, caps):
    for name in sorted(os.listdir(sfd)):
        st["entries"] += 1
        if st["entries"] > caps["entries"]:
            raise _TooLarge("more than %d files and folders" % caps["entries"])
        try:
            info = os.stat(name, dir_fd=sfd, follow_symlinks=False)
        except FileNotFoundError:
            continue
        if not SNAPSHOT_NAME_RE.match(name):
            st["skipped"] += 1
            continue
        if stat.S_ISDIR(info.st_mode):
            if name == "__pycache__":
                st["skipped"] += 1
                continue
            if depth + 1 > caps["depth"]:
                raise _TooLarge("folders nested deeper than %d" % caps["depth"])
            csfd = _open_dir(name, sfd)
            try:
                os.mkdir(name, 0o700, dir_fd=dfd)
                cdfd = _open_dir(name, dfd)
                try:
                    _snapshot_walk(csfd, cdfd, rel + name + "/", depth + 1, st, caps)
                finally:
                    os.close(cdfd)
            finally:
                os.close(csfd)
        elif stat.S_ISREG(info.st_mode):
            try:
                got = _snapshot_file(name, sfd, dfd, caps["bytes"] - st["bytes"])
            except _TooLarge:
                raise _TooLarge("more than %d bytes" % caps["bytes"])
            if got is None:
                st["skipped"] += 1
                continue
            st["bytes"] += got[0]
            st["files"].append((rel + name, got[0], got[1]))
        else:
            st["skipped"] += 1               # a symbolic link, a pipe, a socket or a device


def snapshot_submission(src, dst, max_bytes, max_entries, max_depth):
    """The freeze (root side: the supervisor, which never reads the graded episode). Copies the agent's submission
    folder `src` into a fresh root-only folder `dst` (0700 / 0600) the way the grader's own snapshot does
    (cellkit.lockdown.snapshot_dir: regular files and folders only; symbolic links, special files and __pycache__
    dropped), never following a link at any level (every open is relative to an open folder, O_NOFOLLOW), within the
    grader's submission cap (`max_bytes`) and the entry and depth bounds of the engine's artifact copy. Returns
    {status: ok | too_large | error, files, bytes, sha256, skipped, why}; too_large and error leave no copy; a missing
    `src` is an empty program (ok, no files)."""
    res = {"status": "error", "files": 0, "bytes": 0, "sha256": None, "skipped": 0, "why": None}
    st = {"entries": 0, "bytes": 0, "skipped": 0, "files": []}
    caps = {"bytes": int(max_bytes), "entries": int(max_entries), "depth": int(max_depth)}
    try:
        _clear_path(dst)
        os.mkdir(dst, 0o700)
        dfd = _open_dir(dst)
        try:
            info = os.fstat(dfd)
            if info.st_uid != os.geteuid():
                raise RuntimeError("the snapshot folder is not the service's")
            try:
                sfd = _open_dir(src)
            except FileNotFoundError:
                sfd = None
            if sfd is not None:
                try:
                    _snapshot_walk(sfd, dfd, "", 0, st, caps)
                finally:
                    os.close(sfd)
        finally:
            os.close(dfd)
        res.update(status="ok", files=len(st["files"]), bytes=st["bytes"], sha256=snapshot_digest(st["files"]),
                   skipped=st["skipped"])
    except _TooLarge as exc:
        res.update(status="too_large", why=str(exc), files=len(st["files"]), bytes=st["bytes"], skipped=st["skipped"])
    except Exception as exc:  # noqa: BLE001 - reported to the worker, which refuses the start
        res.update(status="error", why="%s: %s" % (type(exc).__name__, str(exc)[:300]))
    if res["status"] != "ok":
        try:
            _clear_path(dst)
        except OSError:
            pass
    return res


def _json_default(obj):
    """json.dumps fallback of the grader's wire: numpy values as plain values, sets sorted, anything else as str."""
    if hasattr(obj, "tolist"):
        return obj.tolist()
    if isinstance(obj, (set, frozenset)):
        return sorted(obj, key=str)
    return str(obj)


# ----------------------------------------------------------------------------------------------------------------
# the graded episode

class GradedService(object):
    """The graded episode in the worker (the service account): the cell, the two windows, the requests, the record.
    Single-threaded: the cell is only touched from the serving loop."""

    def __init__(self, cfg, log, state, episode_id, seed_sha256, token_digest, snapshot_fds=None):
        self.cfg, self.log, self.state = cfg, log, state
        self.snapshot_fds = snapshot_fds        # (request fd, reply fd): the supervisor's freeze of the submission
        self.episode_id, self.seed_sha256, self.token_digest = episode_id, seed_sha256, token_digest
        self.server = None
        self.cell_version = None
        self.record_path = os.path.join(cfg["record_dir"], "record.json")
        self.requests_log = Log(os.path.join(cfg["record_dir"], "requests.jsonl"), cfg["requests_log_max_bytes"])
        self.service_mono = float(state["service_started_mono"])
        self.prep_deadline = self.service_mono + float(cfg["prep_s"])
        self.started = bool(state.get("started"))
        self.started_mono = state.get("started_mono")
        self.graded_deadline = None if self.started_mono is None else float(self.started_mono) + float(cfg["graded_s"])
        self.ended, self.end_reason = bool(state.get("ended")), state.get("end_reason")
        self.prep_closed = bool(state.get("prep_closed"))   # the start window closed without a start (program frozen)
        self.ended_mono, self.ended_utc = None, None
        self.broken = False
        self.dirty = False
        self.last_write = 0.0
        self.record_seq = 0
        self.requests = 0               # the episode's requests (cell calls) since the start
        self.request_seq = 0            # every request of the session (requests.jsonl)
        self.after_done = 0
        self.after_done_steps = 0
        self.final_summary = None
        self.last_record = None
        self.privileged = bool(state.get("privileged"))
        self.token_mismatch = False

    # ---- the cell
    def build(self, seed, cell_cfg):
        """The offline grader's cell (mode hidden, the grader's configuration) on the graded episode's seed: the same
        construction as tests/verify.py's run_episode (ShoeCellServer(...); reset(seed))."""
        lib = os.path.join(self.cfg["given_dir"], "lib")
        if lib not in sys.path:
            sys.path.insert(0, lib)
        from shoecell import CELL_VERSION
        from shoecell.server import ShoeCellServer
        t0 = time.time()
        self.server = ShoeCellServer(self.cfg["given_dir"], config=cell_cfg, mode="hidden", privileged=False,
                                     render=True, log=self._cell_log)
        self.server.reset(int(seed))
        self.cell_version = CELL_VERSION
        self.log.write(kind="cell_built", wall_s=round(time.time() - t0, 2), cell_version=CELL_VERSION)

    def _cell_log(self, msg):
        self.log.write(kind="cell", msg=str(msg)[:4000])

    # ---- windows
    def check_windows(self, now=None):
        now = time.monotonic() if now is None else now
        if self.started and not self.ended and self.server is not None and now >= self.graded_deadline:
            self.end("graded_window")
        if not self.started and not self.prep_closed and now > self.prep_deadline:
            self.close_prep()

    def close_prep(self):
        """The start window closed without a start (the dual mode): the program freezes now, as it would have at a start,
        so the program item keeps main's preparation time; the graded episode can no longer be started. A copy that keeps
        failing is recorded as `error` (tests/verify.py then stops with a task error: the program cannot be graded)."""
        snap = {"status": "error", "why": "not attempted"}
        for attempt in range(3):
            snap = self._request_snapshot()
            if snap.get("status") in ("ok", "too_large"):
                break
            self.log.write(kind="snapshot_failed", at="prep_window", attempt=attempt + 1, why=snap.get("why"))
            time.sleep(5.0)
        snap = {k: snap.get(k) for k in ("status", "files", "bytes", "sha256", "skipped", "why")}
        snap["utc"] = now_utc()
        snap["at"] = "prep_window"
        st = dict(self.state, prep_closed=True, prep_closed_utc=now_utc(), snapshot=snap)
        try:
            atomic_write_json(state_path(self.cfg), st)
        except OSError as exc:
            self.log.write(kind="state_persist_failed", err=str(exc))
        self.state = st
        self.prep_closed = True
        self.server = None                      # the cell is no longer needed
        rec = {"format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.episode_id,
               "seed_sha256": self.seed_sha256, "service_version": SERVICE_VERSION, "cell_version": self.cell_version,
               "started": False, "ended": False, "end_reason": None, "prep_window_closed": True,
               "prep_closed_utc": st["prep_closed_utc"], "requests": 0, "steps": 0, "sim_time_s": 0.0,
               "wall_s_since_start": 0.0, "episode_score": 0.0, "metrics": {}, "written_utc": now_utc(), "seq": 1}
        rec.update(self._snapshot_fields())
        try:
            atomic_write_json(self.record_path, rec)
            self.last_record = rec
        except OSError as exc:
            self.log.write(kind="record_write_failed", err=str(exc))
        self.log.write(kind="prep_window_closed", snapshot=snap)

    # ---- requests
    def handle_line(self, line):
        """One request line -> the reply object (never raises)."""
        self.check_windows()
        t0 = time.monotonic()
        self.request_seq += 1
        rid, method, params = None, None, None
        try:
            rid, method, params = parse_request(line)
            if method == "status":
                result = self.op_status()
            elif method == "start":
                result = self.op_start(params)
            else:
                result = self.op_cell(method, params)
        except ReplyError as exc:
            result = {"error": exc.code, "detail": exc.detail}
        except Exception:  # noqa: BLE001 - a defect of the service never takes the loop down
            self.log.write(kind="internal_error", method=method, tb=traceback.format_exc(limit=20))
            result = {"error": "internal", "detail": ERRORS["internal"]}
        self._log_request(method, params, result, t0)
        if is_error(result):
            reply = {"id": rid, "error": result["error"]}
            if result.get("detail"):
                reply["detail"] = result["detail"]
            return reply
        return {"id": rid, "result": result}

    def op_status(self):
        now = time.monotonic()
        live = self.started and self.server is not None
        rec = self.last_record or {}
        return {"service_version": SERVICE_VERSION, "started": self.started, "ended": self.ended,
                "end_reason": self.end_reason if self.ended else None,
                "steps": int(self.server.step_calls) if live else int(rec.get("steps") or 0),
                "sim_time": float(self.server.scene.data.time) if live else rec.get("sim_time_s"),
                "camera_frames_used": int(self.server.frames_used) if live else None,
                "prep_window_s": float(self.cfg["prep_s"]), "graded_window_s": float(self.cfg["graded_s"]),
                "prep_left_s": None if self.started else max(0.0, self.prep_deadline - now),
                "graded_left_s": None if not self.started else (0.0 if self.ended else max(0.0, self.graded_deadline - now)),
                "can_start": (not self.started) and now <= self.prep_deadline and not self.broken and self.server is not None,
                "prep_window_closed": self.prep_closed, "program_frozen": self._frozen()}

    def _frozen(self):
        """What the agent learns of the freeze: its own program's copy (status, files, bytes), once frozen (at the start
        or when the start window closed)."""
        snap = self.state.get("snapshot") if (self.started or self.prep_closed) else None
        return None if not isinstance(snap, dict) else {k: snap.get(k) for k in ("status", "files", "bytes")}

    def _request_snapshot(self):
        """Ask the root supervisor to freeze the submission folder (snapshot_submission); its reply, or an error."""
        if not self.snapshot_fds:
            return {"status": "error", "why": "no freeze channel"}
        qw, ar = self.snapshot_fds
        try:
            while select.select([ar], [], [], 0)[0]:          # a reply that came after an earlier timeout
                if not os.read(ar, 4096):
                    return {"status": "error", "why": "the freeze channel is closed"}
            os.write(qw, b"S")
            buf, deadline = b"", time.monotonic() + float(self.cfg["snapshot_timeout_s"])
            while b"\n" not in buf:
                left = deadline - time.monotonic()
                if left <= 0:
                    return {"status": "error", "why": "the freeze timed out"}
                if select.select([ar], [], [], left)[0]:
                    chunk = os.read(ar, 4096)
                    if not chunk:
                        return {"status": "error", "why": "the freeze channel is closed"}
                    buf += chunk
            res = json.loads(buf.split(b"\n", 1)[0])
        except (OSError, ValueError) as exc:
            return {"status": "error", "why": "%s: %s" % (type(exc).__name__, exc)}
        return res if isinstance(res, dict) else {"status": "error", "why": "a malformed reply"}

    def op_start(self, params):
        if self.started:
            raise ReplyError("already_started")
        now = time.monotonic()
        if self.prep_closed or now > self.prep_deadline:
            raise ReplyError("prep_window_closed")
        if self.broken or self.server is None:
            raise ReplyError("service_broken")
        if not isinstance(params, dict) or not set(params) <= {"token"}:
            raise ReplyError("bad_request", "start() takes no parameters")
        token = params.get("token")
        privileged = mismatch = False
        if token is not None:        # the maintainers' reference program only; a wrong token is ignored (counted)
            privileged = isinstance(token, str) and self.token_digest is not None and hmac.compare_digest(
                hashlib.sha256(token.strip().encode("utf-8")).hexdigest(), self.token_digest)
            mismatch = not privileged
        # the freeze (the dual mode): the root supervisor copies the submission folder before anything is charged; the
        # copy is the program the offline item grades. A failed copy refuses the start and charges nothing.
        snap = self._request_snapshot()
        if snap.get("status") not in ("ok", "too_large"):
            self.log.write(kind="snapshot_failed", why=snap.get("why"))
            raise ReplyError("snapshot_failed")
        snap = {k: snap.get(k) for k in ("status", "files", "bytes", "sha256", "skipped", "why")}
        snap["utc"] = now_utc()
        snap["at"] = "start"
        now = time.monotonic()
        if now > self.prep_deadline:
            raise ReplyError("prep_window_closed")
        # the charge: persisted before anything is simulated, never refunded
        st = dict(self.state, started=True, started_utc=now_utc(), started_mono=now, privileged=privileged, snapshot=snap)
        try:
            atomic_write_json(state_path(self.cfg), st)
        except OSError as exc:
            self.broken = True
            self.log.write(kind="state_persist_failed", err=str(exc))
            raise ReplyError("service_broken")
        self.state = st
        self.started, self.started_mono = True, now
        self.graded_deadline = now + float(self.cfg["graded_s"])
        self.privileged, self.token_mismatch = privileged, mismatch
        if privileged:
            self.server.privileged = True
            self.server.reference_dir = self.cfg["private_dir"]
        self.server.wall_start = time.time()
        self.log.write(kind="episode_started", privileged=privileged, token_mismatch=mismatch,
                       prep_used_s=round(now - self.service_mono, 1), snapshot=snap)
        self.write_record()
        return {"ok": True, "started": True, "graded_window_s": float(self.cfg["graded_s"]),
                "program_frozen": self._frozen()}

    def op_cell(self, method, params):
        if not self.started:
            raise ReplyError("prep_window_closed" if self.prep_closed else "not_started")
        if self.ended:
            # after the cell's own end (sim cap, physics fault) the program may still read its state and finish, within
            # the grader's after-done limits; after a finish, the graded window or a restart nothing more answers
            if self.end_reason != "done" or self.server is None:
                raise ReplyError("episode_over")
            self.after_done += 1
            self.after_done_steps += 1 if method == "step" else 0
            if (self.after_done > int(self.cfg["after_done_max_requests"])
                    or self.after_done_steps > int(self.cfg["after_done_max_steps"])):
                raise ReplyError("episode_over")
            if method == "finish":
                self.after_done = int(self.cfg["after_done_max_requests"])
            return self.server.handle(method, params)          # the record is final: nothing changes it any more
        self.requests += 1
        result = self.server.handle(method, params)
        if method not in READ_ONLY_CALLS:
            self.dirty = True
        if self.server.done:
            self.end("finish" if method == "finish" else "done")
        return result

    def _log_request(self, method, params, result, t0):
        err = result.get("error") if isinstance(result, dict) else None
        sim = float(self.server.scene.data.time) if (self.started and self.server is not None) else None
        self.requests_log.write(n=self.request_seq, m=method, p=None if params is None else log_params(method, params),
                                sim=sim, err=err if isinstance(err, str) else None,
                                ms=round((time.monotonic() - t0) * 1000.0, 1))

    # ---- the end
    def end(self, reason):
        """The offline grader's end of an episode whose program stops now: the cell is ended (unless it ended itself)
        and scored as it stands; nothing is simulated afterwards."""
        if self.ended:
            return
        srv = self.server
        if not srv.done:
            srv._end(reason)
        self.ended_mono, self.ended_utc = time.monotonic(), now_utc()
        self._stamp_wall()
        self.final_summary = srv.finalize()
        self.ended, self.end_reason = True, reason
        self.write_record()                 # the record first: a restart in between keeps an ended record
        st = dict(self.state, ended=True, end_reason=reason, ended_utc=self.ended_utc)
        try:
            atomic_write_json(state_path(self.cfg), st)
            self.state = st
        except OSError as exc:
            self.log.write(kind="state_persist_failed", err=str(exc))
        self.log.write(kind="episode_ended", reason=reason, cell_done_reason=srv.done_reason,
                       sim_time_s=float(srv.scene.data.time), steps=int(srv.step_calls), requests=self.requests)

    def close_after_restart(self):
        """A worker that finds the episode started (its predecessor died or restarted): the episode ends with its last
        recorded state; nothing is resumed."""
        rec = None
        try:
            rec = read_json_file(self.record_path)
            if not isinstance(rec, dict) or rec.get("format") != RECORD_FORMAT:
                rec = None
        except (OSError, ValueError):
            rec = None
        if rec is None:
            rec = {"format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.episode_id,
                   "seed_sha256": self.seed_sha256, "service_version": SERVICE_VERSION, "started": True, "ended": False,
                   "end_reason": None, "started_utc": self.state.get("started_utc"), "requests": 0, "steps": 0,
                   "sim_time_s": 0.0, "wall_s_since_start": 0.0, "episode_score": 0.0, "metrics": {},
                   "note": "no record had been written when the worker restarted"}
            rec.update(self._snapshot_fields())
        if not rec.get("ended"):
            rec.update(ended=True, end_reason=self.state.get("end_reason") if self.state.get("ended") else "worker_restart",
                       ended_utc=now_utc(), written_utc=now_utc(), seq=int(rec.get("seq") or 0) + 1)
            atomic_write_json(self.record_path, rec)
        self.ended, self.end_reason = True, rec.get("end_reason") or "worker_restart"
        if not self.state.get("ended"):
            st = dict(self.state, ended=True, end_reason=self.end_reason, ended_utc=rec.get("ended_utc"))
            atomic_write_json(state_path(self.cfg), st)
            self.state = st
        self.last_record = rec
        self.log.write(kind="restarted_after_start", end_reason=self.end_reason)

    # ---- the record
    def _stamp_wall(self):
        srv = self.server
        wall = (self.ended_mono or time.monotonic()) - float(self.started_mono)
        srv.wall_used = wall
        srv.program_used = wall - float(srv.work_wall)

    def interim_summary(self):
        """ShoeCellServer.finalize()'s summary without ending the episode: the score as if it ended now."""
        srv = self.server
        self._stamp_wall()
        ws = srv.work.summary(srv.violations)
        return {"done_reason": srv.done_reason, "sim_time_s": float(srv.scene.data.time), "wall_time_s": float(srv.wall_used),
                "ticks": int(srv.tick_count), "work_score": float(ws["work_score"]), "pairs": ws["pairs"],
                "pairs_shipped": int(ws["pairs_shipped"]), "violations": list(srv.violations),
                "safety_ok": len(srv.violations) == 0, "illegal_call_count": int(srv.illegal_calls),
                "internal_errors": int(srv.internal_errors), "calls": dict(srv.calls), "step_calls": int(srv.step_calls),
                "program_time_s": float(srv.program_used), "cell_work_s": float(srv.work_wall),
                "camera_frames_total": int(srv.frames_used), "camera_frames": dict(srv.frames_by_camera),
                "render_wall_s": float(srv.render_wall_s), "render_failures": int(srv.render_failures),
                "build_wall_s": float(srv.build_wall_s)}

    def write_record(self):
        if self.server is None:
            return
        try:
            summary = self.final_summary if self.ended and self.final_summary is not None else self.interim_summary()
            rec = self._record(summary)
            if len(json.dumps(_clean(rec), allow_nan=False)) > (512 << 10):
                rec["cell"]["pair"].pop("detail", None)
            atomic_write_json(self.record_path, rec)
        except Exception:  # noqa: BLE001 - the previous record stays; the next request tries again
            self.log.write(kind="record_write_failed", tb=traceback.format_exc(limit=10))
            return
        self.last_record = rec
        self.last_write = time.monotonic()
        self.dirty = False

    def _record(self, s):
        self.record_seq += 1
        pairs = s.get("pairs") or []
        pair = pairs[0] if pairs else None
        ms = (pair or {}).get("milestones") or {}
        done = s.get("done_reason")
        metrics = {"work_score": float(s["work_score"]), "pairs_shipped": int(s["pairs_shipped"]), "pairs_started": len(pairs),
                   "pair_credit": float(pair["credit"]) if pair else 0.0,
                   "pair_safety_void": 1.0 if (pair and pair.get("safety_void")) else 0.0,
                   "safety_ok": 1.0 if s["safety_ok"] else 0.0, "safety_violations": len(s["violations"]),
                   "laced_fraction": float(((pair or {}).get("detail") or {}).get("laced_fraction") or 0.0),
                   "sim_time_s": float(s["sim_time_s"]), "ticks": int(s["ticks"]), "step_calls": int(s["step_calls"]),
                   "illegal_call_count": int(s["illegal_call_count"]), "cell_internal_errors": int(s["internal_errors"]),
                   "camera_frames_total": int(s["camera_frames_total"]), "render_wall_s": float(s["render_wall_s"]),
                   "render_failures": int(s["render_failures"]), "cell_work_s": float(s["cell_work_s"]),
                   "build_wall_s": float(s["build_wall_s"]), "physics_fault": 1.0 if done == "physics_fault" else 0.0,
                   "sim_cap": 1.0 if done == "sim_cap" else 0.0, "privileged_mode": 1.0 if self.privileged else 0.0,
                   "oracle_token_mismatch": 1.0 if self.token_mismatch else 0.0}
        for k in MILESTONES:
            metrics["milestone_" + k] = float(ms.get(k, 0.0) or 0.0)
        for cam in CAMERAS:
            metrics["camera_frames_" + cam] = int((s.get("camera_frames") or {}).get(cam, 0))
        wall = (self.ended_mono or time.monotonic()) - float(self.started_mono)
        return {
            "format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.episode_id, "seed_sha256": self.seed_sha256,
            "service_version": SERVICE_VERSION, "cell_version": self.cell_version,
            "started": True, "ended": bool(self.ended), "end_reason": self.end_reason if self.ended else None,
            "started_utc": self.state.get("started_utc"), "ended_utc": self.ended_utc if self.ended else None,
            "requests": int(self.requests), "steps": int(s["step_calls"]), "sim_time_s": float(s["sim_time_s"]),
            "wall_s_since_start": float(wall), "episode_score": float(s["work_score"]), "metrics": metrics,
            **self._snapshot_fields(),
            "cell": {"done_reason": done, "violations": [v.get("what") for v in s["violations"]][:20],
                     "calls": s.get("calls") or {},
                     "pair": None if pair is None else {k: pair.get(k) for k in ("credit", "milestones", "safety_void",
                                                                                 "shipped", "shipped_at", "detail")}},
            "written_utc": now_utc(), "seq": self.record_seq,
        }

    def _snapshot_fields(self):
        snap = self.state.get("snapshot") if isinstance(self.state.get("snapshot"), dict) else {}
        return {"snapshot_status": snap.get("status"), "snapshot_files": snap.get("files"),
                "snapshot_bytes": snap.get("bytes"), "snapshot_sha256": snap.get("sha256"),
                "snapshot_skipped": snap.get("skipped"), "snapshot_utc": snap.get("utc"),
                "snapshot_at": snap.get("at"), "snapshot_why": snap.get("why")}

    # ---- the serving loop (one client at a time)
    def serve(self, listener):
        listener.setblocking(False)
        conn, buf, last_active = None, b"", time.monotonic()
        cap = int(self.cfg["max_line_bytes"])
        while True:
            now = time.monotonic()
            self.check_windows(now)
            timeout = 1.0
            if self.dirty:
                timeout = max(0.0, min(timeout, float(self.cfg["idle_flush_s"]),
                                       self.last_write + float(self.cfg["record_every_s"]) - now))
            try:
                readable, _, _ = select.select([listener] if conn is None else [listener, conn], [], [], timeout)
            except InterruptedError:
                continue
            now = time.monotonic()
            if not readable:
                if self.dirty:
                    self.write_record()          # the connection went quiet: the record catches up
                if conn is not None and now - last_active > float(self.cfg["idle_timeout_s"]):
                    self.log.write(kind="idle_close")
                    conn = self._close(conn)
                continue
            if listener in readable:
                new = self._accept(listener)
                if new is not None:
                    if conn is not None:
                        self._refuse_busy(new)
                    elif self._send(new, {"service": "arb-online", "protocol": PROTOCOL, "service_version": SERVICE_VERSION,
                                          "cell_version": self.cell_version}):
                        conn, buf, last_active = new, b"", now
                    else:
                        self._close(new)
            if conn is None or conn not in readable:
                continue
            try:
                chunk = conn.recv(1 << 16)
            except (BlockingIOError, InterruptedError):
                chunk = None
            except OSError:
                chunk = b""
            if chunk == b"":
                conn = self._close(conn)
                continue
            if chunk:
                buf += chunk
                last_active = now
            while conn is not None:
                nl = buf.find(b"\n")
                if nl < 0 and len(buf) <= cap:
                    break
                if nl < 0 or nl > cap:
                    self._send(conn, {"id": None, "error": "line_too_long", "detail": ERRORS["line_too_long"]})
                    conn = self._close(conn)
                    break
                line, buf = buf[:nl], buf[nl + 1:]
                if not line.strip():
                    continue
                reply = self.handle_line(line)
                if not self._send(conn, reply):
                    conn = self._close(conn)
                    break
                last_active = time.monotonic()
                if self.dirty and last_active - self.last_write >= float(self.cfg["record_every_s"]):
                    self.write_record()

    def _accept(self, listener):
        try:
            new, _ = listener.accept()
        except (BlockingIOError, InterruptedError):
            return None
        except OSError as exc:
            self.log.write(kind="accept_error", err=str(exc))
            time.sleep(0.05)
            return None
        new.settimeout(float(self.cfg["reply_timeout_s"]))
        try:
            uid = struct.unpack("3i", new.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))[1]
        except OSError:
            uid = -1
        self.log.write(kind="connect", peer_uid=uid)
        return new

    def _refuse_busy(self, new):
        new.settimeout(1.0)
        self._send(new, {"id": None, "error": "busy", "detail": ERRORS["busy"]})
        self.log.write(kind="busy_refused")
        self._close(new)

    def _send(self, conn, obj):
        """One reply line within the reply deadline; False when the client is gone or does not read it."""
        try:
            conn.sendall((json.dumps(obj, default=_json_default) + "\n").encode("utf-8"))
            return True
        except (OSError, ValueError):
            return False

    def _close(self, conn):
        try:
            conn.close()
        except OSError:
            pass
        self.log.write(kind="disconnect")
        if self.dirty:
            self.write_record()                  # the client is gone: the record catches up at once
        return None


# ----------------------------------------------------------------------------------------------------------------
# worker: the graded episode's process

def run_worker(cfg, listener, ready_fd, lab, snapshot_fds=None):
    """Forked from the supervisor as root: read the graded episode and the pinned inputs, drop privileges, build the
    cell, serve. `snapshot_fds` (request, reply): the pipe pair to the supervisor, which freezes the submission folder
    at the start (the worker never regains root for it)."""
    _set_comm("arbonline-wrk")
    os.umask(0o077)
    log = Log(os.path.join(cfg["record_dir"], "service.log"), cfg["service_log_max_bytes"])
    os.fchown(log.fh.fileno(), lab.pw_uid, lab.pw_gid)
    try:
        ep = read_json_file(cfg["episode"], 1 << 16)
        episode_id, seed = ep.get("episode"), ep.get("seed")
        if not (isinstance(episode_id, str) and ID_RE.match(episode_id) and isinstance(seed, int)
                and not isinstance(seed, bool) and 0 < seed < (1 << 63)):
            raise ValueError("malformed graded episode")
        del ep
        seed_sha256 = hashlib.sha256(str(seed).encode("ascii")).hexdigest()
        token_digest = None
        try:
            with open(cfg["token_digest"]) as fh:
                words = fh.read(256).split()
            token_digest = words[0] if words and HEX64_RE.match(words[0]) else None
        except OSError:
            pass
        cell_cfg = read_json_file(cfg["cell_config"])
        problems = check_givens(os.path.dirname(cfg["given_dir"]), read_json_file(cfg["given_hashes"], 4 << 20))
        if problems:
            raise RuntimeError("the service's copy of the givens is not the pinned one: " + "; ".join(problems[:5]))
        state = read_state(cfg)
        for k, v in cfg["env"].items():
            os.environ[k] = v
        drop_privileges(lab)                 # irreversible: everything below runs as the service account
        sec_ok = seccomp_selftest()
        if cfg["require_seccomp"] and not sec_ok:
            raise RuntimeError("seccomp self-test failed: an AF_INET socket could be created")
        svc = GradedService(cfg, log, state, episode_id, seed_sha256, token_digest, snapshot_fds)
        if svc.started:
            svc.close_after_restart()
        elif not svc.prep_closed:
            svc.build(seed, cell_cfg)           # (a closed start window needs no cell: status only)
        del seed
        log.write(kind="worker_ready", uid=os.getuid(), gid=os.getgid(), seccomp_inet_blocked=sec_ok,
                  started=svc.started, ended=svc.ended, prep_left_s=round(svc.prep_deadline - time.monotonic(), 1))
    except Exception:  # noqa: BLE001
        log.write(kind="worker_start_failed", tb=traceback.format_exc(limit=30))
        os.write(ready_fd, b"F")
        os._exit(1)
    os.write(ready_fd, b"R")
    os.close(ready_fd)
    try:
        svc.serve(listener)
    except Exception:  # noqa: BLE001
        log.write(kind="worker_crashed", tb=traceback.format_exc(limit=30))
    os._exit(1)


# ----------------------------------------------------------------------------------------------------------------
# supervisor (root; never reads the graded episode, never accepts a connection)

def _close_fds(keep):
    for fd in range(3, 1024):
        if fd not in keep:
            try:
                os.close(fd)
            except OSError:
                pass


def supervise_worker(pid, qr, aw, cfg, sup_log):
    """Wait for the worker to exit, meanwhile answering its freeze requests (one byte "S" -> one JSON line): the copy
    of the agent's submission folder at the start (snapshot_submission), done here because it needs root and the
    supervisor never reads the graded episode. Returns the worker's exit status."""
    open_q = True
    while True:
        try:
            ready = select.select([qr] if open_q else [], [], [], 0.5)[0]
        except InterruptedError:
            ready = []
        if ready:
            try:
                req = os.read(qr, 1)
            except OSError:
                req = b""
            if not req:
                open_q = False
            elif req == b"S":
                t0 = time.monotonic()
                res = snapshot_submission(cfg["submission_dir"], os.path.join(cfg["record_dir"], SNAPSHOT_DIR),
                                          cfg["snapshot_max_bytes"], cfg["snapshot_max_entries"], cfg["snapshot_max_depth"])
                sup_log.write(kind="snapshot", wall_s=round(time.monotonic() - t0, 3),
                              **{k: res.get(k) for k in ("status", "files", "bytes", "skipped", "why")})
                try:
                    os.write(aw, (json.dumps(res) + "\n").encode("utf-8"))
                except OSError:
                    pass
        try:
            done, code = os.waitpid(pid, os.WNOHANG)
        except ChildProcessError:
            return -1
        except InterruptedError:
            continue
        if done:
            return code


def run_supervisor(cfg, lab, notify_fd):
    _set_comm("arbonline-sup")
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
    os.chmod(sock_path, 0o666)  # before the chown: without CAP_FOWNER root cannot chmod a file it no longer owns
    os.chown(sock_path, lab.pw_uid, lab.pw_gid)
    listener.listen(16)
    pidfile = os.path.join(os.path.dirname(sock_path), "supervisor.pid")
    with open(pidfile + ".tmp", "w") as fh:
        fh.write("%d\n" % os.getpid())
    os.chmod(pidfile + ".tmp", 0o644)
    os.replace(pidfile + ".tmp", pidfile)
    sup_log.write(kind="supervisor_start", pid=os.getpid(), seccomp=seccomp, caps_trimmed=caps_trimmed)

    stopping = {"flag": False, "child": None}

    def on_term(signum, frame):
        stopping["flag"] = True
        if stopping["child"]:
            try:
                os.kill(stopping["child"], signal.SIGKILL)
            except ProcessLookupError:
                pass

    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, on_term)
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    restarts, first = 0, True
    while not stopping["flag"]:
        r, w = os.pipe()
        qr, qw = os.pipe()               # the worker's freeze requests
        ar, aw = os.pipe()               # the supervisor's replies
        pid = os.fork()
        if pid == 0:
            os.close(r)
            os.close(qr)
            os.close(aw)
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
            run_worker(cfg, listener, w, lab, (qw, ar))
            os._exit(1)
        os.close(w)
        os.close(qw)
        os.close(ar)
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
        code = supervise_worker(pid, qr, aw, cfg, sup_log)
        os.close(qr)
        os.close(aw)
        stopping["child"] = None
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


def start(config_path):
    if os.geteuid() != 0:
        print("arb-online: must be started as root (from environment/setup/setup.sh)", file=sys.stderr)
        return 2
    try:
        cfg = load_config(config_path)
        lab = accounts(cfg)
        os.umask(0o077)
        sockdir = os.path.dirname(cfg["socket"])
        os.makedirs(os.path.dirname(sockdir), exist_ok=True)
        secure_dir(sockdir, 0o755, 0, 0)
        secure_dir(cfg["record_dir"], 0o700, lab.pw_uid, lab.pw_gid)
        initialize_state(cfg, lab.pw_uid, lab.pw_gid)
    except Exception as exc:  # noqa: BLE001
        print("arb-online: configuration error: %s: %s" % (type(exc).__name__, exc), file=sys.stderr)
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
                Log(os.path.join(cfg["record_dir"], "supervisor.log"), cfg["service_log_max_bytes"]).write(
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
        print("arb-online: ready (socket %s)" % cfg["socket"], file=sys.stderr)
        return 0
    print("arb-online: failed to start (details in the service-only log)", file=sys.stderr)
    return 1


def main(argv=None):
    ap = argparse.ArgumentParser(description="the graded service (start as root from environment/setup/setup.sh)")
    ap.add_argument("--config", required=True)
    return start(ap.parse_args(argv).config)


if __name__ == "__main__":
    sys.exit(main())
