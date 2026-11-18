#!/usr/bin/env python3
"""The graded episode service of the messy desk (online variant): one hidden episode, played by the agent.

Started once by environment/setup/setup.sh as root, cwd = the stage (/opt/ale/setup, root-only while the
agent works):

    env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \\
        python3 -I -B online/server.py --config "$PWD/online/online.json"

The command returns once the service serves (exit 0) or failed to start (exit 1); the service itself is a
detached daemon (setsid, stdio on /dev/null) that lives into the agent phase. Python 3.12 standard library
plus the verifier's own sim_client.py and messy_desk_score.py, copied byte for byte next to this file.

Process layout (the alelab daemon pattern):
  supervisor (root, comm "desk-online-sup"): binds the unix socket and writes the pid file, forks the
      worker and restarts it (bounded). It never reads the hidden episode and never accepts a connection.
  worker (comm "desk-online-wrk"): forked as root, reads the hidden episode (its seed) and the reference
      token's digest, copies the simulator (env.py, byte-identical to the verifier's tests/env.py) into the
      lab-only state directory, then drops to the lab account irreversibly (setgroups([]), setgid, setuid,
      not dumpable, no core files) and serves. On its first start it warms the simulator's shader cache
      (one launch of the simulator with a public seed, then closed) before it reports ready, so the
      episode's own launch later takes seconds.
  simulator: env.py under the Isaac interpreter, started by the worker for the episode (as the lab
      account, a clean environment, its Kit data, cache and logs in the lab-only state directory).
  All of them run under PR_SET_NO_NEW_PRIVS and a seccomp filter under which socket()/socketpair() work
  only for AF_UNIX (no network socket at any time).

Protocol: one JSON object per line in each direction over the unix socket; one client at a time (a second
connection gets `busy`). The episode lives in the service, not in a connection: a disconnect leaves it
where it is and the next connection continues it.
  {"op": "status"}                             -> started, ended, steps, horizon, sim_time_s, prep_left_s,
                                                  graded_left_s
  {"op": "start"} (+ "token": the reference    -> the observation at t = 0. One shot: charged and persisted
     solution's credential, optional)             (fsync) before anything is simulated, never refunded
  {"op": "step", "action": [8 or 9 numbers]}   -> the next observation, done
  {"op": "observe"}                            -> the latest observation again (nothing is simulated), done
  {"op": "finish"}                             -> ends the episode now
Errors: {"ok": false, "error": <code>, "message": <fixed text>}. A reply never carries the score, the
record, the episode's id or its seed.

The episode is the offline verifier's (tests/verify.py on main): the same simulator, seed, horizon and
per-episode score (messy_desk_score.EpisodeScorer). What does not carry over is the program's: the
controller time budget, the 30 s reply deadline, the probe lock, the restarts. It ends at its horizon
(`done`), at the agent's finish (`finish`), when the graded window runs out (`graded_window`), or when
the worker or its simulator dies (`worker_restart`); ended before the horizon it is scored as the offline
grader scores an episode it cuts: on the state reached, scaled by the fraction of the horizon that ran.

The record: /var/lib/arb-online/record.json (lab account, 0600, in a 0700 directory of the lab account
under the root-owned /var/lib), rewritten atomically (temp file, fsync, rename) at the start, at the end,
and at most `record_interval_s` after any change in between; requests.jsonl beside it (append-only, no
images, capped). The separate verifier reads the record; nothing the agent writes reaches it.
"""

import argparse
import ctypes
import datetime
import hashlib
import json
import math
import os
import platform
import pwd
import resource
import select
import shutil
import signal
import socket
import stat
import struct
import subprocess
import sys
import threading
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from messy_desk_score import EpisodeScorer  # noqa: E402  (byte-identical to tests/messy_desk_score.py)
from sim_client import SimClient, SimError  # noqa: E402  (byte-identical to tests/sim_client.py)

SERVICE_VERSION = "messy-desk-online/1"
RECORD_FORMAT = "arb-online-record/1"
TASK = "object_finding_and_organizing_on_messy_desk"
OPS = {"status": {"op"}, "start": {"op", "token"}, "step": {"op", "action"}, "observe": {"op"},
       "finish": {"op"}}
REQUIRED = {"status": {"op"}, "start": {"op"}, "step": {"op", "action"}, "observe": {"op"}, "finish": {"op"}}
ERRORS = {
    "bad_json": "request is not one valid JSON object on one line",
    "line_too_long": "request line exceeds the size limit",
    "bad_request": "request fields are missing, unknown or of the wrong type",
    "unknown_op": "unknown op",
    "busy": "another client is connected; one client at a time",
    "not_started": "the graded episode has not been started",
    "already_started": "the graded episode was already started (one start per session)",
    "start_window_closed": "the window for starting the graded episode has closed",
    "episode_over": "the graded episode has ended",
    "graded_window_closed": "the graded window has closed; the episode has ended",
    "bad_action": "action must be 8 finite numbers, plus an optional ninth (the hi-res camera bitmask)",
    "simulator_unavailable": "the graded episode's simulator could not start (it needs about 6 GB of GPU "
                             "memory); nothing was charged, try again",
    "simulator_failed": "the graded episode's simulator failed; the episode has ended",
    "internal": "internal service error",
}
ACTION_DIM = 8
HIRES_BITS = 15
TOKEN_MAX_BYTES = 4096
CONTROL_HZ = 10
# what of each object the privileged block carries: tests/verify.py on main, unchanged
OBJECT_KEYS = ("id", "model", "cls", "pos", "quat", "scale", "state", "in_container", "on_table", "held",
               "contact", "resting")
END_REASONS = ("done", "finish", "graded_window", "worker_restart")

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
# capabilities the root supervisor keeps (worker re-fork as root, then drop): CHOWN, DAC_OVERRIDE, KILL,
# SETGID, SETUID
SUPERVISOR_CAPS = (0, 1, 5, 6, 7)


# ----------------------------------------------------------------------------------------------------------
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
        ins(jeq, 0, 1, nr_uring), ins(ret, 0, 0, deny),
        ins(jeq, 2, 0, nr_socket), ins(jeq, 1, 0, nr_socketpair), ins(ret, 0, 0, SECCOMP_RET_ALLOW),
        ins(ld, 0, 0, 16),  # args[0] low word = address family
        ins(jeq, 0, 1, socket.AF_UNIX), ins(ret, 0, 0, SECCOMP_RET_ALLOW), ins(ret, 0, 0, deny),
    ]
    return b"".join(prog)


def install_unix_only_seccomp():
    """PR_SET_NO_NEW_PRIVS + a filter under which only AF_UNIX sockets can be created (inherited by every
    child, the simulator included)."""
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
    for cap in range(64):
        if cap in keep:
            continue
        try:
            _prctl(PR_CAPBSET_DROP, cap)
        except OSError:
            pass

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
            socket.socket(fam, socket.SOCK_STREAM).close()
            blocked = False
        except OSError:
            pass
    try:
        socket.socket(socket.AF_UNIX, socket.SOCK_STREAM).close()
    except OSError:
        return False
    return blocked


def _utc():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


class Log:
    """Append-only JSON-lines log (0600), size-capped."""

    def __init__(self, path, cap):
        self.path, self.cap = path, int(cap)
        self.lock = threading.Lock()
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
        st = os.fstat(fd)
        if not stat.S_ISREG(st.st_mode) or st.st_nlink != 1:
            os.close(fd)
            raise RuntimeError("log path is not a plain file")
        self.fh = os.fdopen(fd, "a", buffering=1)
        self.size = st.st_size
        self.full = False

    def write(self, **rec):
        rec = {"utc": _utc(), "mono": round(time.monotonic(), 3), **rec}
        line = json.dumps(rec, default=str, separators=(",", ":")) + "\n"
        with self.lock:
            if self.size + len(line) > self.cap:
                self.full = True
                return
            try:
                self.fh.write(line)
                self.size += len(line)
            except OSError:
                pass


def _write_atomic(path, data, mode=0o600):
    tmp = path + ".tmp"
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, mode)
    try:
        os.write(fd, data)
        os.fsync(fd)
    finally:
        os.close(fd)
    os.replace(tmp, path)
    dfd = os.open(os.path.dirname(path), os.O_RDONLY)
    try:
        os.fsync(dfd)
    finally:
        os.close(dfd)


# ----------------------------------------------------------------------------------------------------------
# configuration

CONFIG_KEYS = {
    "user", "agent_user", "socket", "record_dir", "state_dir", "episode", "token_digest", "simulator",
    "isaac_python", "prep_s", "graded_s", "horizon", "sim_ready_timeout_s", "sim_start_attempts",
    "sim_step_timeout_s", "warmup", "start_timeout_s", "idle_timeout_s", "max_line_bytes", "max_restarts",
    "record_interval_s", "requests_log_cap_bytes", "log_cap_bytes", "env", "require_seccomp",
}


def load_config(path):
    path = os.path.abspath(path)
    with open(path, "rb") as fh:
        cfg = json.loads(fh.read(1 << 20))
    if not isinstance(cfg, dict) or set(cfg) != CONFIG_KEYS:
        raise ValueError("config keys differ from the expected set")
    base = os.path.dirname(path)
    for key in ("episode", "token_digest", "simulator"):
        cfg[key] = os.path.normpath(os.path.join(base, cfg[key]))
    for key in ("socket", "record_dir", "state_dir", "isaac_python"):
        if not os.path.isabs(cfg[key]):
            raise ValueError("%s must be an absolute path" % key)
    if os.path.dirname(cfg["socket"]) in ("/", "/tmp", "/var/tmp", "/dev/shm", "/run"):
        raise ValueError("the socket must live in its own root-owned directory")
    for key in ("prep_s", "graded_s", "sim_ready_timeout_s", "sim_step_timeout_s", "start_timeout_s",
                "idle_timeout_s", "record_interval_s"):
        if not isinstance(cfg[key], (int, float)) or isinstance(cfg[key], bool) or not cfg[key] > 0:
            raise ValueError("%s must be a positive number" % key)
    if not isinstance(cfg["horizon"], int) or cfg["horizon"] < 1:
        raise ValueError("horizon must be a positive integer")
    if not isinstance(cfg["env"], dict) or not all(isinstance(k, str) and isinstance(v, str)
                                                   for k, v in cfg["env"].items()):
        raise ValueError("env must map strings to strings")
    return cfg


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
    return lab


def _read_hidden(cfg):
    """The graded episode (read as root, before the privilege drop): (episode id, seed, seed_sha256) and the
    reference token's digest."""
    with open(cfg["episode"], "rb") as fh:
        spec = json.loads(fh.read(1 << 16))
    episode, seed = str(spec["episode"]), int(spec["seed"])
    if not 0 <= seed < (1 << 64):
        raise ValueError("the seed must be a 64-bit integer")
    digest = hashlib.sha256(str(seed).encode()).hexdigest()
    if spec.get("seed_sha256") not in (None, digest):
        raise ValueError("episode.json: seed_sha256 does not match the seed")
    with open(cfg["token_digest"]) as fh:
        token_digest = fh.read().split()[0].strip().lower()
    return episode, seed, digest, token_digest


# ----------------------------------------------------------------------------------------------------------
# request parsing (data only)

def _reject_constant(name):
    raise ValueError("non-finite constant")


def _finite_float(text):
    v = float(text)
    if not math.isfinite(v):
        raise ValueError("non-finite number")
    return v


def _bounded_int(text):
    if len(text) > 19:
        raise ValueError("integer too long")
    return int(text)


def _no_duplicates(pairs):
    out = {}
    for k, v in pairs:
        if k in out:
            raise ValueError("duplicate key")
        out[k] = v
    return out


class ReplyError(Exception):
    def __init__(self, code):
        super().__init__(code)
        self.code = code


def parse_request(line):
    try:
        obj = json.loads(line.decode("utf-8"), parse_constant=_reject_constant, parse_float=_finite_float,
                         parse_int=_bounded_int, object_pairs_hook=_no_duplicates)
    except (ValueError, RecursionError):
        raise ReplyError("bad_json")
    if not isinstance(obj, dict):
        raise ReplyError("bad_json")
    op = obj.get("op")
    if not isinstance(op, str):
        raise ReplyError("bad_request")
    if op not in OPS:
        raise ReplyError("unknown_op")
    if not REQUIRED[op] <= set(obj) <= OPS[op]:
        raise ReplyError("bad_request")
    if op == "start" and "token" in obj and (not isinstance(obj["token"], str)
                                             or len(obj["token"]) > TOKEN_MAX_BYTES):
        raise ReplyError("bad_request")
    return obj


def parse_action(value):
    """tests/verify.py's action rule: 8 finite numbers, plus an optional ninth (the hi-res camera bitmask).
    A malformed action is refused (nothing is simulated); offline it aborted the episode."""
    if not isinstance(value, list) or len(value) not in (ACTION_DIM, ACTION_DIM + 1):
        raise ReplyError("bad_action")
    out = []
    for v in value:
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise ReplyError("bad_action")
        f = float(v)
        if not math.isfinite(f):
            raise ReplyError("bad_action")
        out.append(f)
    return out


def hires_frames(action):
    if len(action) <= ACTION_DIM:
        return 0
    return bin(int(round(action[ACTION_DIM])) & HIRES_BITS).count("1")


# ----------------------------------------------------------------------------------------------------------
# the privileged block of the reference solution (tests/verify.py on main, unchanged)

def privileged_block(state, containers, obs):
    req = state.get("request") or None
    request = None
    if req and req.get("status") == "open":
        request = {k: req.get(k) for k in ("id", "model", "object", "text", "opened_t", "deadline_t")}
    objects = [{k: o.get(k) for k in OBJECT_KEYS} for o in state.get("objects") or []
               if o.get("present") and o.get("pos") is not None]
    return {"t": state.get("t"), "objects": objects, "request": request, "containers": containers,
            "ee_pose": list(obs["ee_pose"]), "joint_pos": list(obs["joint_pos"]),
            "gripper_width": obs["gripper_width"], "ik_ok": state.get("ik_ok")}


def _containers(info):
    return {n: {"xy": [float(c["center_xy"][0]), float(c["center_xy"][1])], "yaw": float(c["yaw"])}
            for n, c in (info.get("containers") or {}).items()}


def token_ok(token, digest):
    """(privileged, mismatch) for a presented token against the digest, as tests/verify.py on main judges a
    submission's .oracle_token: a token that does not match is recorded and otherwise ignored."""
    if token is None:
        return False, False
    ok = hashlib.sha256(token.encode("utf-8", "replace").strip()).hexdigest() == digest
    return ok, not ok


# ----------------------------------------------------------------------------------------------------------
# the simulator, as the lab account

class LabSim(SimClient):
    """The verifier's SimClient (attempts, readiness deadline, step deadline, close), launched with the
    service's own argv, environment and working directory."""

    def __init__(self, env_script, args, env, cwd, pgid_file, **kw):
        self._args, self._env, self._cwd, self._pgid_file = list(args), dict(env), cwd, pgid_file
        super().__init__(env_script, **kw)

    def _launch(self):
        import queue
        self.proc = subprocess.Popen([self.python, "-B", self.env_script] + self._args, stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=self._log, universal_newlines=True,
                                     bufsize=1, start_new_session=True, env=self._env, cwd=self._cwd,
                                     close_fds=True)
        try:
            _write_atomic(self._pgid_file, ("%d\n" % self.proc.pid).encode())
        except OSError:
            pass
        self._lines = queue.Queue()
        threading.Thread(target=self._pump, args=(self.proc, self._lines), daemon=True).start()

    def kill(self):
        self._kill()


def kill_stale_simulator(pgid_file, uid):
    """A simulator left behind by a worker that died: SIGKILL its process group (only if it is the lab
    account's)."""
    try:
        with open(pgid_file) as fh:
            pgid = int(fh.read().strip())
    except (OSError, ValueError):
        return False
    try:
        if os.stat("/proc/%d" % pgid).st_uid != uid:
            return False
        os.killpg(pgid, signal.SIGKILL)
        return True
    except OSError:
        return False


# ----------------------------------------------------------------------------------------------------------
# the episode

class Episode:
    def __init__(self, cfg, lab, hidden, log):
        self.cfg, self.lab, self.log = cfg, lab, log
        self.episode_id, self.seed, self.seed_sha256, self.token_digest = hidden
        self.horizon = int(cfg["horizon"])
        self.lock = threading.RLock()          # the episode and its simulator
        self.write_lock = threading.Lock()     # record writes
        self.phase = "idle"                    # idle | running | ended
        self.end_reason = None
        self.sim = None
        self.scorer = None
        self.steps = 0
        self.requests = 0
        self.bad_actions = 0
        self.hires = 0
        self.privileged = False
        self.mismatch = False
        self.containers = None
        self.last_obs = None
        self.started_mono = None
        self.ended_mono = None
        self.started_utc = None
        self.ended_utc = None
        self.dirty = False
        self.last_write = 0.0
        self.persisted = None                  # the record read back after a worker restart
        rd = cfg["record_dir"]
        self.record_path = os.path.join(rd, "record.json")
        self.req_log_path = os.path.join(rd, "requests.jsonl")
        self.req_log = None
        self.req_log_size = 0
        sd = cfg["state_dir"]
        self.service_file = os.path.join(sd, "service.json")
        self.pgid_file = os.path.join(sd, "sim.pgid")
        self.serving_since = None

    # -- the service clock (persisted: a restarted worker keeps the windows)
    def init_clock(self, first):
        if first:
            self.serving_since = time.monotonic()
            _write_atomic(self.service_file, json.dumps({"serving_since_mono": self.serving_since,
                                                         "utc": _utc()}).encode())
            return
        try:
            with open(self.service_file, "rb") as fh:
                self.serving_since = float(json.loads(fh.read(4096))["serving_since_mono"])
        except Exception as exc:  # noqa: BLE001 - fail closed: no clock, no start
            self.serving_since = float("-inf")
            self.log.write(kind="service_clock_unreadable", err=type(exc).__name__)

    def prep_left(self, now=None):
        now = time.monotonic() if now is None else now
        return max(0.0, float(self.cfg["prep_s"]) - (now - self.serving_since))

    def graded_left(self, now=None):
        now = time.monotonic() if now is None else now
        if self.phase == "idle":
            return float(self.cfg["graded_s"])
        if self.phase == "ended":
            return 0.0
        return max(0.0, float(self.cfg["graded_s"]) - (now - self.started_mono))

    # -- persistence
    def recover(self):
        """A worker that starts while a record exists: the episode it describes is over. One that was still
        running ends with its last recorded state (no resume)."""
        if not os.path.lexists(self.record_path):
            return
        try:
            with open(self.record_path, "rb") as fh:
                rec = json.loads(fh.read(1 << 20))
            if not isinstance(rec, dict) or rec.get("format") != RECORD_FORMAT:
                raise ValueError("not a record")
        except Exception as exc:  # noqa: BLE001
            self.log.write(kind="record_unreadable_at_restart", err=type(exc).__name__)
            rec = None
        self.phase = "ended"
        if rec is None:
            return
        self.persisted = rec
        self.steps = int(rec.get("steps") or 0)
        if not rec.get("ended"):
            rec["ended"], rec["end_reason"], rec["ended_utc"] = True, "worker_restart", _utc()
            rec["written_utc"] = _utc()
            try:
                _write_atomic(self.record_path, json.dumps(rec, sort_keys=True).encode())
            except OSError as exc:
                self.log.write(kind="record_write_failed", err=str(exc))
            self.log.write(kind="episode_ended", reason="worker_restart", steps=self.steps,
                           episode_score=rec.get("episode_score"))

    def _metrics(self):
        s = self.scorer.summary() if self.scorer is not None else {}
        m = {k: s[k] for k in ("requests_opened", "requests_success", "requests_failed", "requests_delivered",
                               "requests_expired", "absent_correct", "absent_wrong", "claims_wrong",
                               "first_holds", "objects_lost", "arrivals", "containers_cleared", "disturbances",
                               "organization", "request_rate", "request_progress", "org_num", "org_den")
             if k in s}
        m["snapshots"] = s.get("snapshots", [])
        m["score_unscaled"] = self.scorer.score_unscaled() if self.scorer is not None else 0.0
        m["episode_cut"] = 1.0 if self.steps < self.horizon else 0.0
        m["hires_frames"] = self.hires
        m["bad_actions"] = self.bad_actions
        m["privileged_mode"] = 1.0 if self.privileged else 0.0
        m["oracle_token_mismatch"] = 1.0 if self.mismatch else 0.0
        return m

    def record(self):
        now = time.monotonic()
        score = self.scorer.score() if self.scorer is not None else 0.0
        end = self.ended_mono if self.ended_mono is not None else now
        return {"format": RECORD_FORMAT, "task": TASK, "episode": self.episode_id, "seed_sha256": self.seed_sha256,
                "started": True, "ended": self.phase == "ended", "end_reason": self.end_reason,
                "requests": self.requests, "steps": self.steps, "horizon": self.horizon,
                "sim_time_s": round(self.steps / float(CONTROL_HZ), 1),
                "wall_s_since_start": round(end - self.started_mono, 1),
                "episode_score": float(score), "metrics": self._metrics(), "service_version": SERVICE_VERSION,
                "started_utc": self.started_utc, "ended_utc": self.ended_utc, "written_utc": _utc()}

    def write_record(self):
        """Snapshot under the episode lock, write outside it (one writer at a time)."""
        with self.write_lock:
            with self.lock:
                if self.phase == "idle":
                    return
                data = json.dumps(self.record(), sort_keys=True, allow_nan=False).encode()
                self.dirty = False
            try:
                _write_atomic(self.record_path, data)
                self.last_write = time.monotonic()
            except (OSError, ValueError) as exc:
                with self.lock:
                    self.dirty = True
                self.log.write(kind="record_write_failed", err=str(exc))

    def flusher(self):
        interval = float(self.cfg["record_interval_s"])
        while True:
            time.sleep(min(0.5, interval))
            if self.dirty and time.monotonic() - self.last_write >= interval:
                self.write_record()

    def window_watch(self):
        """The service enforces the graded window itself: when it runs out the episode ends there and the
        record is finalised, whether or not a request arrives."""
        while True:
            time.sleep(1.0)
            with self.lock:
                if self.phase == "running" and self.graded_left() <= 0.0:
                    self._end("graded_window")

    def log_request(self, op, ok, error=None, action=None, wall=None):
        if self.req_log is None:
            return
        rec = {"i": self.requests, "op": op, "ok": ok, "t": self.steps,
               "sim_time_s": round(self.steps / float(CONTROL_HZ), 1)}
        if wall is not None:
            rec["wall_s"] = round(wall, 2)
        if error:
            rec["error"] = error
        if action is not None:
            rec["action"] = [round(v, 6) for v in action]
        line = json.dumps(rec, separators=(",", ":")) + "\n"
        if self.req_log_size + len(line) > int(self.cfg["requests_log_cap_bytes"]):
            return
        try:
            self.req_log.write(line)
            self.req_log_size += len(line)
        except OSError:
            pass

    # -- the simulator
    def sim_env(self):
        sd = self.cfg["state_dir"]
        env = {"PATH": "/usr/local/bin:/usr/bin:/bin", "LANG": "C.UTF-8", "HOME": os.path.join(sd, "home"),
               "TMPDIR": os.path.join(sd, "tmp"), "XDG_CACHE_HOME": os.path.join(sd, "home", ".cache"),
               "XDG_DATA_HOME": os.path.join(sd, "home", ".local", "share"),
               "XDG_CONFIG_HOME": os.path.join(sd, "home", ".config"), "PYTHONNOUSERSITE": "1",
               "PYTHONDONTWRITEBYTECODE": "1",
               # OptiX keeps its ray-tracing program cache in /var/tmp/OptixCache_<user> unless told otherwise
               "OPTIX_CACHE_PATH": os.path.join(sd, "home", ".cache", "optix")}
        env.update(self.cfg["env"])
        return env

    def launch_sim(self, timeout=None, attempts=None):
        sd = self.cfg["state_dir"]
        return LabSim(os.path.join(sd, "code", os.path.basename(self.cfg["simulator"])),
                      ["--portable-root", os.path.join(sd, "kit")], self.sim_env(), os.path.join(sd, "home"),
                      self.pgid_file, python=self.cfg["isaac_python"],
                      ready_timeout=float(timeout or self.cfg["sim_ready_timeout_s"]),
                      step_timeout=float(self.cfg["sim_step_timeout_s"]),
                      log_path=os.path.join(sd, "sim.log"),
                      attempts=int(attempts or self.cfg["sim_start_attempts"]))

    def close_sim(self, sim):
        """Close a simulator (outside the episode lock: a close waits up to 30 s for the process)."""
        if sim is None:
            return
        try:
            sim.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            sim.kill()
        except Exception:  # noqa: BLE001
            pass
        try:
            os.unlink(self.pgid_file)
        except OSError:
            pass

    def warmup(self):
        """First start of the service: one launch of the simulator (as the lab account, its own Kit cache),
        a reset with a public seed and a few steps with every hi-res camera, then closed, so that the
        renderer's shaders are compiled before the agent's session starts. Bounded as a whole by
        warmup.timeout_s (setup itself has a fixed bound); a failure or an overrun is logged and leaves the
        episode's own launch to compile them."""
        w = self.cfg["warmup"]
        limit = float(w["timeout_s"])
        t0 = time.monotonic()
        out = {}

        def run():
            try:
                sim = self.launch_sim(timeout=limit, attempts=1)
                out["ready_s"] = round(time.monotonic() - t0, 1)
                sim.reset(int(w["seed"]), 50)
                for _ in range(int(w["steps"])):
                    sim.step([0.0] * 7 + [0, HIRES_BITS])
                self.close_sim(sim)
                out["ok"] = True
            except Exception as exc:  # noqa: BLE001
                out["err"] = "%s: %s" % (type(exc).__name__, str(exc)[:300])

        th = threading.Thread(target=run, daemon=True)
        th.start()
        th.join(limit)
        if th.is_alive():
            kill_stale_simulator(self.pgid_file, self.lab.pw_uid)
            th.join(60.0)
            out.setdefault("err", "warm-up overran %.0f s" % limit)
        total = round(time.monotonic() - t0, 1)
        if out.get("ok"):
            self.log.write(kind="warmup_done", ready_s=out.get("ready_s"), total_s=total)
        else:
            self.log.write(kind="warmup_failed", err=out.get("err"), ready_s=out.get("ready_s"), total_s=total,
                           sim_log_tail=self.sim_log_tail())
            kill_stale_simulator(self.pgid_file, self.lab.pw_uid)

    def sim_log_tail(self, n=4000):
        try:
            with open(os.path.join(self.cfg["state_dir"], "sim.log"), "rb") as fh:
                fh.seek(0, os.SEEK_END)
                size = fh.tell()
                fh.seek(max(0, size - n))
                return fh.read().decode("utf-8", "replace")
        except OSError:
            return ""

    # -- ending
    def _end(self, reason):
        """End the running episode here (called with the lock held). Nothing is simulated afterwards; the
        score is the offline grader's for an episode that stops at this step."""
        if self.phase != "running":
            return
        self.phase, self.end_reason = "ended", reason
        self.ended_mono, self.ended_utc = time.monotonic(), _utc()
        self.dirty = True
        self.log.write(kind="episode_ended", reason=reason, steps=self.steps,
                       wall_s=round(self.ended_mono - self.started_mono, 1))
        threading.Thread(target=self._after_end, daemon=True).start()

    def _after_end(self):
        self.write_record()
        with self.lock:
            sim, self.sim = self.sim, None
        self.close_sim(sim)
        if self.req_log is not None:
            try:
                self.req_log.flush()
            except OSError:
                pass

    # -- ops (called with the lock held)
    def op_status(self, req):
        now = time.monotonic()
        return {"started": self.phase != "idle", "ended": self.phase == "ended", "steps": self.steps,
                "horizon": self.horizon, "sim_time_s": round(self.steps / float(CONTROL_HZ), 1),
                "prep_left_s": round(self.prep_left(now), 1), "graded_left_s": round(self.graded_left(now), 1)}

    def op_start(self, req):
        if self.phase != "idle":
            raise ReplyError("already_started")
        if self.prep_left() <= 0.0:
            raise ReplyError("start_window_closed")
        privileged, mismatch = token_ok(req.get("token"), self.token_digest)
        t0 = time.monotonic()
        try:
            self.sim = self.launch_sim()
        except SimError as exc:
            self.log.write(kind="simulator_unavailable", err=str(exc)[:300], sim_log_tail=self.sim_log_tail())
            kill_stale_simulator(self.pgid_file, self.lab.pw_uid)
            self.sim = None
            raise ReplyError("simulator_unavailable")
        launch_s = time.monotonic() - t0
        # charged and persisted before anything is simulated; never refunded
        self.phase = "running"
        self.privileged, self.mismatch = privileged, mismatch
        self.started_mono, self.started_utc = time.monotonic(), _utc()
        self.requests += 1
        try:
            fd = os.open(self.req_log_path, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
            self.req_log = os.fdopen(fd, "a", buffering=1 << 16)
        except OSError as exc:
            self.log.write(kind="requests_log_unavailable", err=str(exc))
        data = json.dumps(self.record(), sort_keys=True).encode()
        try:
            _write_atomic(self.record_path, data)
        except OSError as exc:
            # cannot persist the charge: do not simulate, do not refund either (the start is spent)
            self.log.write(kind="record_write_failed_at_start", err=str(exc))
            self._end("worker_restart")
            raise ReplyError("internal")
        self.last_write = time.monotonic()
        self.log.write(kind="episode_started", launch_s=round(launch_s, 1), privileged=privileged,
                       token_mismatch=mismatch, prep_left_s=round(self.prep_left(), 1))
        try:
            obs, state = self.sim.reset(self.seed, self.horizon)
            self.containers = _containers(self.sim.info()) if privileged else None
            self.scorer = EpisodeScorer(self.horizon)
            if privileged:
                obs["privileged"] = privileged_block(state, self.containers, obs)
        except Exception as exc:  # noqa: BLE001 - the simulator died or answered out of shape: the episode ends here
            self.log.write(kind="simulator_failed", op="reset", err="%s: %s" % (type(exc).__name__, str(exc)[:300]),
                           sim_log_tail=self.sim_log_tail())
            self._end("worker_restart")
            raise ReplyError("simulator_failed")
        self.last_obs = obs
        self.dirty = True
        self.log_request("start", True, wall=time.monotonic() - self.started_mono)
        return {"obs": obs, "t": obs.get("t", 0), "done": False}

    def _running(self):
        if self.phase == "idle":
            raise ReplyError("not_started")
        if self.phase == "ended":
            raise ReplyError("episode_over")
        if self.graded_left() <= 0.0:
            self._end("graded_window")
            raise ReplyError("graded_window_closed")

    def op_step(self, req):
        self._running()
        try:
            action = parse_action(req["action"])
        except ReplyError:
            self.bad_actions += 1
            raise
        try:
            obs, state = self.sim.step(action)
            self.scorer.add_step(state)
        except Exception as exc:  # noqa: BLE001 - the simulator died or answered out of shape: the episode ends here
            self.log.write(kind="simulator_failed", op="step", t=self.steps,
                           err="%s: %s" % (type(exc).__name__, str(exc)[:300]), sim_log_tail=self.sim_log_tail())
            self._end("worker_restart")
            raise ReplyError("simulator_failed")
        self.steps += 1
        self.hires += hires_frames(action)
        if self.privileged:
            obs["privileged"] = privileged_block(state, self.containers, obs)
        self.last_obs = obs
        self.dirty = True
        self.log_request("step", True, action=action, wall=time.monotonic() - self.started_mono)
        if self.steps >= self.horizon:
            self._end("done")
        return {"obs": obs, "t": obs.get("t", self.steps), "done": self.phase == "ended"}

    def op_observe(self, req):
        if self.phase == "idle":
            raise ReplyError("not_started")
        if self.last_obs is None:
            raise ReplyError("episode_over")
        return {"obs": self.last_obs, "t": self.last_obs.get("t", self.steps), "done": self.phase == "ended"}

    def op_finish(self, req):
        if self.phase == "idle":
            raise ReplyError("not_started")
        if self.phase == "running":
            self._end("finish")
        return {"done": True, "steps": self.steps}


# ----------------------------------------------------------------------------------------------------------
# worker: serving

class Server:
    def __init__(self, cfg, episode, listener, log):
        self.cfg, self.episode, self.listener, self.log = cfg, episode, listener, log
        self.active = 0
        self.active_lock = threading.Lock()
        self.conn_seq = 0

    def serve(self):
        threading.Thread(target=self.episode.flusher, daemon=True).start()
        threading.Thread(target=self.episode.window_watch, daemon=True).start()
        self.listener.settimeout(None)
        while True:
            try:
                conn, _ = self.listener.accept()
            except OSError as exc:
                self.log.write(kind="accept_error", err=str(exc))
                time.sleep(0.05)
                continue
            with self.active_lock:
                admit = self.active < 1
                if admit:
                    self.active += 1
                    self.conn_seq += 1
                    cid = self.conn_seq
            if not admit:
                try:
                    conn.settimeout(1.0)
                    conn.sendall(self.error_line("busy"))
                except OSError:
                    pass
                conn.close()
                continue
            threading.Thread(target=self.handle, args=(conn, cid), daemon=True).start()

    @staticmethod
    def error_line(code):
        return (json.dumps({"ok": False, "error": code, "message": ERRORS[code]}) + "\n").encode()

    def handle(self, conn, cid):
        peer_uid = -1
        try:
            try:
                peer_uid = struct.unpack("3i", conn.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))[1]
            except OSError:
                pass
            self.log.write(kind="connect", conn=cid, peer_uid=peer_uid)
            conn.settimeout(float(self.cfg["idle_timeout_s"]))
            cap = int(self.cfg["max_line_bytes"])
            buf = b""
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    if len(buf) > cap:
                        conn.sendall(self.error_line("line_too_long"))
                        return
                    chunk = conn.recv(1 << 16)
                    if not chunk:
                        return
                    buf += chunk
                    continue
                line, buf = buf[:nl], buf[nl + 1:]
                if len(line) > cap:
                    conn.sendall(self.error_line("line_too_long"))
                    return
                if not line.strip():
                    continue
                conn.sendall(self.dispatch(line, cid))
        except (OSError, socket.timeout):
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass
            with self.active_lock:
                self.active -= 1
            self.log.write(kind="disconnect", conn=cid)

    def dispatch(self, line, cid):
        op = None
        ep = self.episode
        try:
            req = parse_request(line)
            op = req["op"]
            with ep.lock:
                if ep.phase != "idle":
                    ep.requests += 1
                try:
                    body = getattr(ep, "op_" + op)(req)
                except ReplyError as exc:
                    if ep.phase != "idle":        # a request to a started episode (a refused start before it: none)
                        ep.log_request(op, False, error=exc.code,
                                       wall=time.monotonic() - ep.started_mono if ep.started_mono else None)
                    raise
                if op not in ("start", "step"):
                    ep.log_request(op, True, wall=time.monotonic() - ep.started_mono if ep.started_mono else None)
            return (json.dumps({"ok": True, "op": op, **body}, allow_nan=False, separators=(",", ":"))
                    + "\n").encode()
        except ReplyError as exc:
            return self.error_line(exc.code)
        except Exception:  # noqa: BLE001
            self.log.write(kind="internal_error", conn=cid, op=op, tb=traceback.format_exc(limit=20))
            return self.error_line("internal")


def _prepare_state(cfg, lab):
    """As root, before the drop: the lab-only state directory (the simulator's code copy, its Kit portable
    root, home and temp directory). The code copy is root-owned and read-only for the lab account."""
    sd = cfg["state_dir"]
    code = os.path.join(sd, "code")
    if not os.path.isdir(code):
        os.mkdir(code, 0o755)
    os.chown(code, 0, 0)
    os.chmod(code, 0o755)
    dst = os.path.join(code, os.path.basename(cfg["simulator"]))
    shutil.copyfile(cfg["simulator"], dst)
    os.chown(dst, 0, 0)
    os.chmod(dst, 0o644)
    for name in ("kit", "home", "tmp"):
        p = os.path.join(sd, name)
        if not os.path.isdir(p):
            os.mkdir(p, 0o700)
            os.chmod(p, 0o700)     # before chown: without CAP_FOWNER root cannot chmod what it no longer owns
            os.chown(p, lab.pw_uid, lab.pw_gid)
        st = os.lstat(p)
        if not stat.S_ISDIR(st.st_mode) or st.st_uid != lab.pw_uid or st.st_mode & 0o077:
            raise RuntimeError("%s is not a private directory of the lab account" % p)


def run_worker(cfg, listener, ready_fd, lab, first):
    """Forked from the supervisor as root: read the hidden episode, drop privileges, warm up, serve."""
    _set_comm("desk-online-wrk")
    os.umask(0o077)
    log = Log(os.path.join(cfg["record_dir"], "service.log"), int(cfg["log_cap_bytes"]))
    os.fchown(log.fh.fileno(), lab.pw_uid, lab.pw_gid)
    try:
        hidden = _read_hidden(cfg)
        _prepare_state(cfg, lab)
        # -- privilege drop (irreversible)
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
        sec_ok = seccomp_selftest()
        log.write(kind="worker_start", uid=os.getuid(), gid=os.getgid(), first=first,
                  seccomp_inet_blocked=sec_ok, version=SERVICE_VERSION)
        if cfg["require_seccomp"] and not sec_ok:
            raise RuntimeError("seccomp self-test failed: an AF_INET socket could be created")
        episode = Episode(cfg, lab, hidden, log)
        kill_stale_simulator(episode.pgid_file, lab.pw_uid)
        episode.recover()
        if first:
            episode.warmup()
        episode.init_clock(first)
        server = Server(cfg, episode, listener, log)
    except Exception:  # noqa: BLE001
        log.write(kind="worker_start_failed", tb=traceback.format_exc(limit=30))
        os.write(ready_fd, b"F")
        os._exit(1)
    log.write(kind="serving", prep_s=cfg["prep_s"], graded_s=cfg["graded_s"], phase=episode.phase)
    os.write(ready_fd, b"R")
    os.close(ready_fd)
    try:
        server.serve()
    except Exception:  # noqa: BLE001
        log.write(kind="worker_crashed", tb=traceback.format_exc(limit=30))
    os._exit(1)


# ----------------------------------------------------------------------------------------------------------
# supervisor (root; never reads the hidden episode, never accepts a connection)

def _close_fds(keep):
    for fd in range(3, 1024):
        if fd not in keep:
            try:
                os.close(fd)
            except OSError:
                pass


def run_supervisor(cfg, lab, notify_fd):
    _set_comm("desk-online-sup")
    os.umask(0o077)
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    sup_log = Log(os.path.join(cfg["state_dir"], "supervisor.log"), int(cfg["log_cap_bytes"]))
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
    os.chmod(sock_path, 0o666)   # before chown: without CAP_FOWNER root cannot chmod a file it no longer owns
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
    pgid_file = os.path.join(cfg["state_dir"], "sim.pgid")
    restarts, first = 0, True
    while not stopping["flag"]:
        r, w = os.pipe()
        pid = os.fork()
        if pid == 0:
            os.close(r)
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
            run_worker(cfg, listener, w, lab, first)
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
        killed = kill_stale_simulator(pgid_file, lab.pw_uid)
        sup_log.write(kind="worker_exit", pid=pid, status=code, ready=status == b"R", first=first,
                      stale_simulator_killed=killed)
        first = False
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
        print("desk-online: must be started as root (from environment/setup/setup.sh)", file=sys.stderr)
        return 2
    try:
        cfg = load_config(config_path)
        lab = _accounts(cfg)
        os.umask(0o077)
        sockdir = os.path.dirname(cfg["socket"])
        os.makedirs(os.path.dirname(sockdir), exist_ok=True)
        _secure_dir(sockdir, 0o755, 0, 0)
        for key in ("record_dir", "state_dir"):
            parent = os.path.dirname(cfg[key])
            st = os.lstat(parent)
            if st.st_uid != 0 or st.st_mode & 0o022:
                raise RuntimeError("the parent of %s must be root-owned and not group/world-writable" % key)
            _secure_dir(cfg[key], 0o700, lab.pw_uid, lab.pw_gid)
        if os.listdir(cfg["record_dir"]):
            raise RuntimeError("the record directory is not empty at the service's first start")
    except Exception as exc:  # noqa: BLE001
        print("desk-online: configuration error: %s: %s" % (type(exc).__name__, exc), file=sys.stderr)
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
                Log(os.path.join(cfg["state_dir"], "supervisor.log"), int(cfg["log_cap_bytes"])).write(
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
        print("desk-online: serving (socket %s)" % cfg["socket"], file=sys.stderr)
        return 0
    print("desk-online: failed to start (details in the service-only log)", file=sys.stderr)
    return 1


def main(argv=None):
    ap = argparse.ArgumentParser(description="the graded episode service (start as root from setup.sh)")
    ap.add_argument("--config", required=True)
    return start(ap.parse_args(argv).config)


if __name__ == "__main__":
    sys.exit(main())
