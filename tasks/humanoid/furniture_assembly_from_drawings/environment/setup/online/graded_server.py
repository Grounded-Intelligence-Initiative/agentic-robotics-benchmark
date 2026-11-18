#!/usr/bin/env python3
"""The graded episode service of furniture_assembly_from_drawings (online variant): ONE hidden graded episode of the
furniture cell, which the agent plays itself during its session, on a Unix socket.

environment/setup/setup.sh starts it as root (cwd = the setup stage, /opt/ale/setup, root-only) after the practice lab:

    env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \\
        /opt/venv/bin/python3 -I -B online/graded_server.py --config online/graded_config.json

The command returns once the service serves (exit 0) or failed to start (exit 1); the service itself is a detached
daemon (setsid, stdio on /dev/null) that lives through the agent phase. The daemon pattern is alelab's
(tasks/arm/arm_hidden_mechanism_box/environment/setup/lab/alelab/server.py):

  supervisor (root, comm "arb-graded-sup"): binds the socket in its own root-owned directory, forks the worker and
      restarts it (bounded). It never reads the hidden episode, never imports the cell, never accepts a connection.
  worker (comm "arb-graded-wrk"): forked as root; imports numpy, mujoco and the whole cell package (furniture_cell
      next to this file: the offline grader's own cell code), reads the hidden episode (graded_episode.json: its id
      and seed) and the oracle token's digest, then drops to the lab account irreversibly (setgroups([]), setgid,
      setuid; not dumpable, no core files) and builds the graded episode's cell exactly as the offline grader builds
      an episode's cell: CellServer(seed, privileged=False, nominal=False, config) with the grading run's budgets.
  Both run under PR_SET_NO_NEW_PRIVS and a seccomp filter under which socket() / socketpair() work only for AF_UNIX.

The episode:
  * one per session, started by the agent's `start` request within `prep_s` of the service's first start; the start
    is persisted (fsync) before anything is simulated and never refunded;
  * it lives in the worker, not in a connection: a disconnect leaves it where it is and a new connection resumes it;
    one client at a time (another connection gets "busy");
  * it ends at its own end (the cell's simulated-time cap, or an internal error of the cell inside step / command),
    at the agent's `finish`, `graded_s` after its start (the service ends it itself), or with the session (the record
    then holds the episode as it stood after the agent's last request). A worker that dies or restarts ends it with
    its last record (no resume). Ending simulates nothing: the offline grader scores an episode whose program stops
    as it stands, and so does this service;
  * the cell answers the offline grader's public calls under the offline grader's per-episode limits (simulated-time
    cap, camera frames, ticks per step, the safety watch); the offline wire's program-time cap, silence rule and
    program-contract gates do not apply (the simulation advances only inside the agent's own requests);
  * the record (record.json in the record directory) is rewritten atomically (temporary file, fsync, rename) after
    the requests that change the episode, at most once per `record_min_interval_s` while requests arrive, and always
    when the episode ends; it holds the offline grader's per-episode score of the episode had it ended then.
    requests.jsonl logs every request (no images), final.json the cell's full end-of-episode record. The directory
    belongs to the lab account (0700): the agent can neither read nor write it; the engine collects it as an
    artifact for the separate verifier, which scores the record and nothing else.

Wire (the practice lab's and the offline grader's): one JSON object per line, {"id": n, "method": m, "params": {...}}
-> {"id": n, "result": {...}} or {"id": n, "error": code[, "detail": text]}. Methods: `status` (any time: started,
ended, steps, simulated time, frames, the time left in each window; never the score), `start` (once), and once
started the cell's own calls (describe, get_state, command, step, get_camera, finish), which the episode's
CellServer answers as offline (anything else: "forbidden", counted as an illegal call). The oracle starts with its
token (params {"oracle_token": ...}, checked against oracle_token.sha256): that allows the cell's privileged calls
(reference_assemble, truth) on the graded layout; a wrong token is ignored (recorded). Python 3.12 stdlib + numpy +
mujoco (+ the cell).
"""

import argparse
import ctypes
import gc
import hashlib
import importlib
import json
import math
import os
import pkgutil
import platform
import pwd
import resource
import select
import signal
import socket
import stat
import struct
import sys
import time
import traceback

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))

SERVICE_VERSION = "furniture-graded/1"
RECORD_FORMAT = "arb-online-record/1"
DEFAULTS = {
    "note": "",
    "task": None,
    "episode_file": "graded_episode.json",
    "oracle_token_digest_file": "oracle_token.sha256",
    "user": "arbcell",                       # the lab account (created in the task image)
    "agent_user": "user",                    # the evaluated account: may connect; the service never runs as it
    "socket": "/run/furniture-graded/graded.sock",
    "record_dir": "/var/lib/arb-online",
    "prep_s": 14400.0,
    "graded_s": 10800.0,
    "cell": None,                            # the cell's configuration: the offline grading run's budgets
    "request_cap_s": 300.0,                  # one cell request longer than this is a hang: the worker dies (SIGALRM)
    "privileged_request_cap_s": 7200.0,      # the privileged reference assembler runs inside one request
    "reply_timeout_s": 120.0,                # a client that does not take its reply in this time is dropped
    "record_min_interval_s": 1.0,
    "max_line_bytes": 1 << 20,
    "requests_log_max_bytes": 32 << 20,
    "service_log_max_bytes": 8 << 20,
    "max_restarts": 5,
    "start_timeout_s": 300.0,
    "require_seccomp": True,
    "env": {},                               # public constants only (renderer, threads, asset folders)
}
CELL_KEYS = ("sim_cap_s", "camera_budget_frames", "max_ticks_per_step", "settle_s", "program_time_cap_s", "silence_s")
EXIT_READY, EXIT_FAILED = b"R", b"F"
PUBLIC_END = {"finish": "finish", "graded_window": "graded_window", "worker_restart": "service_restart"}

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
    tmp = "%s.tmp" % path
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, mode)
    try:
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
    base = os.path.dirname(path)
    for key in ("episode_file", "oracle_token_digest_file"):
        cfg[key] = os.path.normpath(os.path.join(base, cfg[key]))
    if not isinstance(cfg["task"], str) or not cfg["task"]:
        raise ValueError("the config needs 'task'")
    if not isinstance(cfg["cell"], dict) or set(cfg["cell"]) != set(CELL_KEYS):
        raise ValueError("the config's 'cell' must hold exactly %s" % (CELL_KEYS,))
    for key in ("prep_s", "graded_s", "request_cap_s", "privileged_request_cap_s", "reply_timeout_s",
                "record_min_interval_s", "start_timeout_s"):
        v = cfg[key]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            raise ValueError("%s must be a positive number" % key)
    for key in ("socket", "record_dir"):
        if not os.path.isabs(cfg[key]):
            raise ValueError("%s must be an absolute path" % key)
    if os.path.dirname(cfg["socket"]) in ("/", "/tmp", "/var/tmp", "/dev/shm", "/run"):
        raise ValueError("the socket must live in its own root-owned directory")
    if os.path.dirname(os.path.abspath(cfg["record_dir"])) in ("/", "/tmp", "/var/tmp", "/dev/shm"):
        raise ValueError("the record directory needs a root-owned parent")
    if not isinstance(cfg["env"], dict) or not all(isinstance(k, str) and isinstance(v, str)
                                                   for k, v in cfg["env"].items()):
        raise ValueError("env must map strings to strings")
    return cfg


def _accounts(cfg):
    lab = pwd.getpwnam(cfg["user"])
    if lab.pw_uid == 0:
        raise RuntimeError("the lab account must not be root")
    try:
        agent = pwd.getpwnam(cfg["agent_user"])
    except KeyError:
        agent = None
    if agent is not None and agent.pw_uid == lab.pw_uid:
        raise RuntimeError("the lab account must differ from the agent account")
    return lab, agent


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
    passed on for the cell to answer bad_params), or None for a line that is not one."""
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


# ----------------------------------------------------------------------------------------------- the worker
class Client(object):
    def __init__(self, sock, number, uid):
        self.sock, self.number, self.uid = sock, number, uid
        self.buf = bytearray()


class Worker(object):
    """Serves the graded episode (single-threaded; the cell's renderer has its own thread)."""

    def __init__(self, cfg, listener, lab, agent, hidden, token_digest, CellServer, log, reqlog):
        self.cfg = cfg
        self.listener = listener
        self.listener.setblocking(False)
        self.allowed = {0} | ({agent.pw_uid} if agent is not None else set())
        self.episode_id = str(hidden["episode"])
        self.seed = int(hidden["seed"])
        self.seed_sha256 = hashlib.sha256(str(self.seed).encode("ascii")).hexdigest()
        self.token_digest = token_digest
        self.CellServer = CellServer
        self.log, self.reqlog = log, reqlog
        self.cell_cfg = dict(cfg["cell"])
        self.prep_s, self.graded_s = float(cfg["prep_s"]), float(cfg["graded_s"])
        self.state = read_json(state_path(cfg))
        for key in ("service_start_unix", "started", "ended"):
            if key not in self.state:
                raise ValueError("service.json lacks %r" % key)
        self.srv = None
        self.client = None
        self.n_conn = 0
        self.n_requests = 0               # cell requests of the episode (after the start; status excluded)
        self.n_status = 0
        self.dirty = False
        self.last_flush = 0.0
        self.record_writes = 0
        self.last_record = None

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

    # -- start-up
    def prepare(self):
        """After the privilege drop: resume the persisted state. Returns a line for the log."""
        self.save_state(worker_starts=int(self.state.get("worker_starts", 0)) + 1)
        self.last_record = self._read_last_record()       # what status() reports once the episode is over
        if self.started and not self.ended:
            # the worker died or restarted with the episode running: it ends with its last record (no resume)
            self.save_state(ended=True, end_reason="worker_restart", ended_at_unix=time.time())
            rec = dict(self.last_record) if self.last_record is not None else self.record(None)
            rec.update(ended=True, end_reason="worker_restart", updated_at_unix=round(time.time(), 3),
                       wall_s_since_start=round(time.time() - float(self.state["started_at_unix"]), 3))
            self._write_record(rec)
            return "the episode was running when the worker died: ended (worker_restart) with its last record"
        if self.ended or self.state.get("prep_closed"):
            return "the episode is over (%s); serving status only" % (self.state.get("end_reason") or "prep window")
        if self.prep_left() <= 0:
            self.close_prep()
            return "the preparation window closed before a start"
        t0 = time.monotonic()
        self.srv = self.CellServer(self.seed, privileged=False, nominal=False, config=dict(self.cell_cfg),
                                   log=lambda s: self.log.write(kind="cell", msg=str(s)[:4000]))
        self._write_record(self.record(None))
        return "graded cell built in %.1f s; waiting for the start" % (time.monotonic() - t0)

    def _read_last_record(self):
        try:
            rec = read_json(os.path.join(self.cfg["record_dir"], "record.json"))
            return rec if isinstance(rec, dict) and rec.get("format") == RECORD_FORMAT else None
        except (OSError, ValueError):
            return None

    # -- the record
    def record(self, cell_rec):
        """The record (O7) from the cell's score() / finalize() record (None: no cell, i.e. never started)."""
        st = self.state
        now = time.time()
        rec = {
            "format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.episode_id,
            "seed_sha256": self.seed_sha256, "service_version": SERVICE_VERSION,
            "started": self.started, "ended": self.ended, "end_reason": st.get("end_reason") if self.ended else None,
            "prep_window_closed": bool(st.get("prep_closed")),
            "privileged": bool(st.get("privileged")), "oracle_token_mismatch": bool(st.get("oracle_token_mismatch")),
            "service_start_unix": float(st["service_start_unix"]), "started_at_unix": st.get("started_at_unix"),
            "updated_at_unix": round(now, 3),
            "wall_s_since_start": round(now - float(st["started_at_unix"]), 3) if self.started else 0.0,
            "windows": {"prep_s": self.prep_s, "graded_s": self.graded_s},
            "requests": int(self.n_requests), "status_requests": int(self.n_status),
            "steps": 0, "ticks": 0, "sim_time_s": 0.0, "episode_score": 0.0, "metrics": {},
            "cell_done_reason": None, "score_error": None,
            "record_writes": int(self.record_writes + 1),
            "requests_log_dropped": int(self.reqlog.dropped),
        }
        srv = self.srv
        if srv is not None:
            rec["steps"] = int(srv.step_calls)
            rec["ticks"] = int(srv.cell.ticks)
            rec["sim_time_s"] = round(float(srv.cell.sim_time), 4)
            rec["cell_done_reason"] = srv.cell.done_reason
        if cell_rec is not None:
            rec["score_error"] = cell_rec.get("score_error")
            safety_ok = bool(cell_rec.get("safety_ok", False))
            stage = _finite(cell_rec.get("stage_score"))
            rec["episode_score"] = stage * (1.0 if safety_ok else 0.0)   # the offline grader's per-episode value
            rec["metrics"] = episode_metrics(cell_rec, srv)
            if "sim_time_s" in cell_rec:
                rec["sim_time_s"] = round(_finite(cell_rec["sim_time_s"]), 4)
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
        """Rewrite the record of the running episode (the score as if it ended now)."""
        srv = self.srv
        if srv is None or not self.started or self.ended:
            self.dirty = False
            return
        try:
            cell_rec = dict(srv.cell.score())
            cell_rec["score_error"] = None
        except Exception:  # noqa: BLE001 - an unscoreable state is recorded, never a crash of the service
            err = traceback.format_exc()
            self.log.write(kind="score_failed", tb=err[-3000:])
            cell_rec = {"stage_score": 0.0, "safety_ok": True, "score_error": err.strip().splitlines()[-1][:300]}
        self._write_record(self.record(cell_rec))

    def flush_if_due(self, force=False):
        if self.dirty and (force or time.monotonic() - self.last_flush >= float(self.cfg["record_min_interval_s"])):
            self.flush()

    # -- the episode's end
    def end(self, reason):
        """End the running episode (`reason`: finish | done | graded_window) and write its final record."""
        if not self.started or self.ended:
            return
        srv = self.srv
        cell_rec = None
        if srv is not None:
            srv.wall_used = time.time() - float(self.state["started_at_unix"])
            if not srv.done:
                srv._end(reason)
            try:
                cell_rec = srv.finalize()                 # the cell's own end-of-episode record; frees the renderers
            except Exception:  # noqa: BLE001 - finalize() does not raise by design
                err = traceback.format_exc()
                self.log.write(kind="finalize_failed", tb=err[-3000:])
                cell_rec = {"stage_score": 0.0, "safety_ok": True, "score_error": err.strip().splitlines()[-1][:300]}
        self.save_state(ended=True, end_reason=reason, ended_at_unix=time.time())
        rec = self.record(cell_rec)
        self._write_record(rec)
        if cell_rec is not None:
            full = {k: v for k, v in cell_rec.items() if k != "seed"}      # the seed stays in the service
            full["seed_sha256"] = self.seed_sha256
            try:
                write_json_atomic(os.path.join(self.cfg["record_dir"], "final.json"), _json_safe(full))
            except (OSError, TypeError, ValueError) as exc:
                self.log.write(kind="final_write_failed", err=str(exc))
        self.log.write(kind="episode_end", reason=reason, cell_done_reason=rec.get("cell_done_reason"),
                       sim_time_s=rec["sim_time_s"], steps=rec["steps"], requests=rec["requests"])
        self.srv = None
        gc.collect()

    def close_prep(self):
        """The preparation window closed without a start: the episode never runs (the record says so)."""
        if self.started or self.state.get("prep_closed"):
            return
        self.save_state(prep_closed=True)
        if self.srv is not None:
            try:
                self.srv.close()
            except Exception:  # noqa: BLE001
                pass
            self.srv = None
            gc.collect()
        self._write_record(self.record(None))
        self.log.write(kind="prep_window_closed")

    def check_windows(self):
        now = time.time()
        if not self.started and not self.state.get("prep_closed") and self.prep_left(now) <= 0:
            self.close_prep()
        if self.started and not self.ended and self.graded_left(now) <= 0:
            self.end("graded_window")

    # -- requests
    def status(self):
        now = time.time()
        srv = self.srv
        last = self.last_record or {}
        reason = self.state.get("end_reason") if self.ended else None
        if reason == "done":
            reason = "sim_cap" if last.get("cell_done_reason") == "sim_cap" else "cell_error"
        else:
            reason = PUBLIC_END.get(reason, reason)
        if not self.started and self.state.get("prep_closed"):
            reason = "prep_window"
        out = {"started": self.started, "ended": self.ended or bool(self.state.get("prep_closed")), "end": reason,
               "steps": int(srv.step_calls) if srv is not None else int(last.get("steps", 0)),
               "sim_time": round(float(srv.cell.sim_time), 4) if srv is not None else float(last.get("sim_time_s", 0.0)),
               "sim_time_max": float(self.cell_cfg["sim_cap_s"]),
               "camera_frames_used": int(srv.cell.frames_total) if srv is not None else int(
                   (last.get("metrics") or {}).get("camera_frames_mean", 0)),
               "camera_frames_max": int(self.cell_cfg["camera_budget_frames"]),
               "prep_window_left_s": None, "graded_window_left_s": None}
        if not self.started and not self.state.get("prep_closed"):
            out["prep_window_left_s"] = round(max(0.0, self.prep_left(now)), 1)
        if self.started and not self.ended:
            out["graded_window_left_s"] = round(max(0.0, self.graded_left(now)), 1)
        return out

    def op_start(self, params):
        if not isinstance(params, dict) or set(params) - {"oracle_token"}:
            return {"error": "bad_params", "detail": "start takes no parameters"}
        if self.started:
            return {"error": "already_started", "detail": "the graded episode was started already (one per session); "
                                                          "status() tells where it stands"}
        if self.state.get("prep_closed") or self.prep_left() <= 0:
            self.close_prep()
            return {"error": "prep_window_closed", "detail": "the graded episode can no longer be started: the "
                                                             "preparation window is over"}
        if self.srv is None:
            return {"error": "internal_error"}
        privileged = mismatch = False
        tok = params.get("oracle_token")
        if tok is not None:
            got = hashlib.sha256(str(tok).strip().encode("utf-8")).hexdigest()
            privileged = bool(self.token_digest) and got == self.token_digest
            mismatch = not privileged
        now = time.time()
        # the start is charged and persisted before anything is simulated, and never refunded
        self.save_state(started=True, started_at_unix=now, privileged=privileged, oracle_token_mismatch=mismatch)
        self.srv.privileged = privileged                  # the privileged calls; the layout stays the graded one
        self.srv.wall_start = now
        self.log.write(kind="episode_start", privileged=privileged, oracle_token_mismatch=mismatch,
                       prep_window_left_s=round(self.prep_left(now), 1))
        self.flush()
        return self.status()

    def dispatch(self, rid, method, params):
        """One request -> the reply object (never raises)."""
        t0 = time.time()
        self.check_windows()
        err = sim_time = None
        if method == "status":
            self.n_status += 1
            result = self.status()
        elif method == "start":
            result = self.op_start(params)
        elif not self.started:
            if self.state.get("prep_closed"):
                result = {"error": "prep_window_closed", "detail": "the graded episode was not started within the "
                                                                   "preparation window"}
            else:
                result = {"error": "not_started", "detail": "start the graded episode first (start())"}
        elif self.ended or self.srv is None:
            result = {"error": "episode_over", "detail": "the graded episode is over; status() tells how it ended"}
        else:
            self.n_requests += 1
            srv = self.srv
            cap = float(self.cfg["privileged_request_cap_s"] if (method == "reference_assemble" and srv.privileged)
                        else self.cfg["request_cap_s"])
            signal.alarm(max(1, int(math.ceil(cap))))    # a hung request ends the worker (and with it the episode)
            try:
                result = srv.handle(method, params)
            finally:
                signal.alarm(0)
            sim_time = float(srv.cell.sim_time)
            self.dirty = True
            if srv.done:
                self.end("finish" if method == "finish" else "done")
        if is_error(result):
            reply = {"id": rid, "error": str(result["error"])}
            if "detail" in result:
                reply["detail"] = str(result["detail"])
            err = reply["error"]
        else:
            reply = {"id": rid, "result": result}
        if self.started or method in ("start", "status"):
            if sim_time is None and self.srv is not None:
                sim_time = float(self.srv.cell.sim_time)
            if method == "status":
                logged = None
            elif method == "start" and isinstance(params, dict):
                logged = {k: ("<redacted>" if k == "oracle_token" else _for_log(v)) for k, v in params.items()}
            else:
                logged = _for_log(params)
            self.reqlog.write(n=self.n_requests if method not in ("start", "status") else None, kind=method[:80],
                              params=logged, error=err,
                              wall_s=round(time.time() - t0, 4),
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
                if self._alive(self.client):
                    self.log.write(kind="connection_busy", conn=self.n_conn, uid=uid)
                    self._refuse(sock, "busy", "another connection is driving the graded episode; close it first")
                    continue
                self.drop("gone (a new connection arrived)")
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


HOLE_GROUPS = ("L.top", "L.shelf", "L.bottom", "R.top", "R.shelf", "R.bottom")
PEAKS = ("robot_contact_N", "robot_robot_N", "part_force_N", "part_speed")


def episode_metrics(s, srv):
    """The offline grader's per-episode statistics (tests/verify.py episode_metrics on main, for one episode; the same
    names), from the cell's score() / finalize() record `s` and the episode's CellServer counters."""
    groups = s.get("groups") or {}
    out = {
        "stage_score_mean_ungated": _finite(s.get("stage_score")),
        "credit_mean": _finite(s.get("credit")),
        "locked_joints_mean": _finite(s.get("n_locked")),
        "engaged_joints": _finite(s.get("n_engaged")),
        "back_panel_rate": 1.0 if (s.get("back_panel") or {}).get("credited") else 0.0,
        "placement_rate": 1.0 if (s.get("placement") or {}).get("ok") else 0.0,
        "safety_violation_rate": 0.0 if s.get("safety_ok", True) else 1.0,
        "safety_violations": float(len(s.get("violations") or [])),
        "mean_sim_time_s": _finite(s.get("sim_time_s")),
        "camera_frames_mean": _finite(s.get("camera_frames_total")),
        "render_wall_s_mean": _finite(s.get("render_wall_s")),
    }
    for g in HOLE_GROUPS:
        out["group_rate." + g] = _finite((groups.get(g) or {}).get("credit"))
    for cam, n in sorted((s.get("camera_frames") or {}).items()):
        out["camera_frames_mean." + str(cam)] = _finite(n)
    for peak in PEAKS:
        out["safety_peak_max." + peak] = _finite((s.get("safety_peaks") or {}).get(peak))
    place = s.get("placement") or {}
    for k in ("dist_xy", "dz", "dyaw_deg", "tilt_deg"):
        if k in place:
            out["placement." + k] = _finite(place[k])
    if srv is not None:
        out["cell_work_s_mean"] = _finite(srv.cell.work_wall)
        out["bad_param_calls"] = float(srv.bad_param_calls)
        out["cell_internal_errors"] = float(srv.internal_errors)
        out["render_errors"] = float(srv.render_errors)
        out["illegal_call_count"] = float(srv.illegal_calls)
        out["step_calls"] = float(srv.step_calls)
    return out


def import_cell():
    """numpy, mujoco and every module of the cell package now, as root (after the privilege drop the worker cannot read
    the stage); the drawing generator (drawings.py, maintainers' tooling) is not part of the cell."""
    import numpy  # noqa: F401
    import mujoco
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    import furniture_cell
    from furniture_cell.server import CellServer
    for mod in pkgutil.iter_modules(furniture_cell.__path__):
        if mod.name != "drawings":
            importlib.import_module("furniture_cell." + mod.name)
    return CellServer, mujoco.__version__


def run_worker(cfg, listener, ready_fd, lab, agent):
    """Forked from the supervisor as root: imports, the hidden episode, the privilege drop, the cell, serving."""
    _set_comm("arb-graded-wrk")
    os.umask(0o077)
    rdir = cfg["record_dir"]
    log = Log(os.path.join(rdir, "service.log"), cfg["service_log_max_bytes"])
    reqlog = Log(os.path.join(rdir, "requests.jsonl"), cfg["requests_log_max_bytes"])
    for fh in (log, reqlog):
        fh.chown(lab.pw_uid, lab.pw_gid)
    try:
        for k, v in cfg["env"].items():
            os.environ[k] = v
        CellServer, mj_version = import_cell()
        hidden = read_json(cfg["episode_file"], limit=4096)
        if not isinstance(hidden, dict) or not isinstance(hidden.get("episode"), str) or \
                isinstance(hidden.get("seed"), bool) or not isinstance(hidden.get("seed"), int):
            raise ValueError("the hidden episode file needs a string 'episode' and an integer 'seed'")
        try:
            with open(cfg["oracle_token_digest_file"]) as fh:
                token_digest = fh.read().strip().lower()
        except OSError:
            token_digest = ""
        sys.path.remove(HERE)
        # -- the privilege drop (irreversible)
        os.chdir(rdir)                       # MuJoCo writes MUJOCO_LOG.TXT into the working directory
        os.setgroups([])
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
        worker = Worker(cfg, listener, lab, agent, hidden, token_digest, CellServer, log, reqlog)
        del hidden
        note = worker.prepare()
        log.write(kind="worker_ready", uid=os.getuid(), gid=os.getgid(), seccomp_inet_blocked=sec_ok,
                  mujoco=mj_version, version=SERVICE_VERSION, note=note,
                  state={k: v for k, v in worker.state.items() if k != "service_start_unix"})
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


def run_supervisor(cfg, lab, agent, notify_fd):
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
    restarts, first = 0, True
    while not stopping["flag"]:
        r, w = os.pipe()
        pid = os.fork()
        if pid == 0:
            os.close(r)
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
            run_worker(cfg, listener, w, lab, agent)
            os._exit(1)
        os.close(w)
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
                    os.write(notify_fd, EXIT_READY if status == EXIT_READY else EXIT_FAILED)
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


def start(config_path):
    if os.geteuid() != 0:
        print("graded service: must be started as root (from environment/setup/setup.sh)", file=sys.stderr)
        return 2
    try:
        cfg = load_config(config_path)
        lab, agent = _accounts(cfg)
        os.umask(0o077)
        sockdir = os.path.dirname(cfg["socket"])
        os.makedirs(os.path.dirname(sockdir), exist_ok=True)
        _secure_dir(sockdir, 0o755, 0, 0)
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
            run_supervisor(cfg, lab, agent, w)
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
