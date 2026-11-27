#!/usr/bin/env python3
"""graded_service: the graded episode of pantry_jar_refilling (dual mode, experimental; never merged into main).

Started once by environment/setup/setup.sh as root, cwd = the setup stage (/opt/ale/setup, root-only while the agent
works), before the agent's session:

    env -i PATH=/opt/venv/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \\
        /opt/venv/bin/python3 -I -B online/graded_service.py --config online/service.json

It returns once the service serves (exit 0) or failed to start (exit 1); the service itself is a detached daemon
(setsid, stdio on /dev/null) that runs through the agent's session.

The episode. ONE episode of the offline grader's fixed hidden set (online/episode.json, read by the worker only: the
hidden unit's three jars and the seed of one entry of tests/hidden/episodes.json, drawn once) on the grader's own
simulator: setup.sh copies the root-only staged payload's jar_sim.py, jar_render.py, jar_look.json, jar_env.py and
episode.py (byte copies of tests/'s, which the grader runs) into online/cell/, and the worker refuses to start unless
every file matches service.json's digests. The episode is built as the offline grader builds it (jar_env.JarEnv(unit,
unit id, render=True), episode.Episode(env, seed), the image's numerics pins and the grader's rendering settings) and
keeps its rules: at most 12000 control steps (20 simulated minutes), the end after 20 consecutive finished steps, an
unstable simulation (scored 0), a simulator fatal error (scored 0), the credit rule (episode.Episode.score), the
observation naming the unit. What does not carry over is what binds a controller process: the run's time budget (the
controller's own time with its fixed image price), the per-request and start-up deadlines, the restarts, the
repeated-t probes and the wire's line cap. A malformed action is refused (bad_action) and simulates nothing. The
simulation advances only inside the agent's own `step` requests.

Rules:
  * `start` is accepted once per session, within `prep_s` of the service's start (state.json `service_started`); it
    is persisted (fsync) before anything is simulated and never refunded. With the oracle's token (its sha256 is in
    episode.json) every observation carries `truth` (JarEnv.truth), as the offline grader gives it to the
    token-holding submission; a wrong token is ignored and counted.
  * The episode lives in the worker, not in a connection: a disconnect leaves it where it is and a new connection
    resumes it. One client at a time: a second connection is answered `busy` and closed; a connection that sends
    nothing for `idle_s` is closed (the episode stays where it is).
  * It ends at its own end (12000 steps, the finished hold, an unstable simulation, a simulator fatal error), at the
    agent's `finish`, `graded_s` after its start (the service's own timer), or when the session ends (the service is
    frozen or killed: the last record stands). It is then scored as the offline grader scores an episode that ends at
    that moment: as it stands (Episode.score's credit; 0 after an unstable simulation or a fatal error), nothing
    simulated afterwards. An ended episode answers `status`, `observe` and `finish` (which changes nothing) only.
  * When the worker dies or restarts, the episode ends with its last record (end_reason worker_restart): the new worker
    never builds a cell for it. A failure of the cell itself (its renderer) ends the episode and is recorded as
    `cell_error`; tests/verify.py treats it as a task error, as the offline grader does.
  * The agent learns the observations, whether the episode is over, and `status` (started / ended, steps, images, the
    windows' time left): never the score, the record, the episode's id or its seed.
  * The freeze (the dual mode's DM3): before the start is charged, the worker asks the root supervisor to copy the
    agent's submission folder into the root-only record/submission_snapshot/ (regular files and folders the agent owns,
    never following a link; main's caps of 5000 files and 200 MB, plus bounds on entries and depth: over any of them
    the start goes on with snapshot_status too_large and no copy). A copy that fails refuses the start; nothing is
    charged. tests/verify.py grades that copy as the program (main's offline grading) and the record as the graded
    episode. The program freezes at the earlier of the graded start and the close of the start window: when `prep_s`
    runs out with no start, the worker's own timer writes a record that says so (started false, freeze
    prep_window_closed, snapshot pending) and has the supervisor make the same copy; a copy that keeps failing is
    recorded as failed (the program item then scores 0; it never falls back to the session-end folder). A small
    reserve file in the record directory, released for that record, keeps a full disk from stopping it.
  * Practice (O13): the task has no practice service that runs hidden code or hidden units. The simulator, the
    practice units and the practice grader in the agent's home are its own tools, and stay.

Process layout (the alelab daemon pattern of arm_hidden_mechanism_box, as pump_skid_startup_and_operation's service):
  supervisor (root, comm jargrade-sup): PR_SET_NO_NEW_PRIVS and a seccomp filter under which only AF_UNIX sockets can
      be created; trimmed capabilities; binds the socket in its own root-owned directory, forks the worker and
      restarts it (bounded), and on the worker's one-byte request freezes the submission folder (snapshot_submission:
      a fixed source, a fixed target, no other input). It never reads episode.json and never accepts a connection.
  worker (comm jargrade-wkr): forked as root; reads episode.json and state.json, checks and imports the simulator,
      builds the episode (its first observation's five images drawn here), then drops to the account `jargrade`
      (setgroups, setgid, setuid, checked irreversible; not dumpable; no core files) and serves.

Record (/var/lib/arb-online: owner jargrade, mode 0700, in the root-owned /var/lib): record.json (format
arb-online-record/1, rewritten atomically: a temp file, fsync, rename) at the start, at most once a second while
requests change the episode and within a second of the last one, and when the episode ends; requests.jsonl (one line
per request: method, the action, step, the error if any; no images; the token redacted; it stops growing at 32 MB);
state.json (the service's start, the one start, the end); service.log and supervisor.log. task.toml declares the
directory an artifact; tests/verify.py scores record.json.

Wire: JSON lines {"id": n, "method": ..., "params": {...}} -> {"id": n, "result": ...} or {"id": n, "error": code,
"detail": text}; methods status, start, step (params: action), observe, finish; a second connection gets
{"id": null, "error": "busy", "detail": ...} and is closed. ~/graded/client.py (connect_graded) speaks it.

Dependencies: Python 3.12 standard library here; the worker imports the simulator (numpy, MuJoCo, Filament) from the
venv.
"""

import argparse
import ctypes
import hashlib
import importlib
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
import sys
import time
import traceback

SERVICE_VERSION = "pantry-jar-graded/1"
RECORD_FORMAT = "arb-online-record/1"
MAX_LINE = 1 << 20                         # one request line (an action is about 300 bytes)
REQUIRED = ("task", "user", "agent_user", "socket", "record_dir", "work_dir", "prep_s", "graded_s", "episode_file",
            "cell_dir", "cell_sha256", "env", "submission_dir")
DEFAULTS = {"max_restarts": 3, "start_timeout_s": 300.0, "record_interval_s": 1.0, "requests_log_cap_bytes": 32 << 20,
            "record_max_bytes": 1000000, "reply_timeout_s": 120.0, "idle_s": 900.0, "max_log_bytes": 16 << 20,
            "note": "", "snapshot_name": "submission_snapshot", "snapshot_max_bytes": 200 << 20,
            "snapshot_max_files": 5000, "snapshot_max_entries": 20000, "snapshot_max_depth": 32,
            "snapshot_timeout_s": 120.0, "window_freeze_tries": 5, "window_freeze_retry_s": 30.0,
            "reserve_bytes": 1 << 20}
ERRORS = {
    "busy": "another connection holds the graded episode: end that script (or close its connection), then connect again",
    "bad_request": "one JSON object per line, with a method",
    "bad_params": "params must be an object",
    "not_started": "the graded episode has not started: start() starts it, once per session",
    "already_started": "the graded episode was started already (once per session): connect_graded() resumes it",
    "prep_window_closed": "the graded episode can no longer be started: its start window has closed",
    "episode_over": "the graded episode has ended: nothing more is simulated or scored",
    "bad_action": ("an action is {\"l\": [7, 8 or 14 finite numbers], \"r\": [...], \"cams\": an integer mask 0 to 31}; "
                   "nothing was simulated"),
    "forbidden": "the graded episode's calls are status, start, step, observe and finish",
    "internal": "internal error of the graded episode's service",
}
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
# the capabilities the root supervisor keeps (it forks the worker as root, which then drops them all with its uid):
# CHOWN, DAC_OVERRIDE, KILL, SETGID, SETUID
SUPERVISOR_CAPS = (0, 1, 5, 6, 7)


# ---------------------------------------------------------------------------------------------- small helpers
_libc = []


def _prctl(option, arg2=0, arg3=0, arg4=0, arg5=0):
    if not _libc:
        lib = ctypes.CDLL(None, use_errno=True)
        lib.prctl.argtypes = [ctypes.c_int, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong]
        _libc.append(lib)
    if _libc[0].prctl(option, arg2, arg3, arg4, arg5) != 0:
        err = ctypes.get_errno()
        raise OSError(err, os.strerror(err))


def _set_comm(name):
    try:
        buf = ctypes.create_string_buffer(name.encode()[:15])
        _prctl(PR_SET_NAME, ctypes.addressof(buf))
    except OSError:
        pass


def install_unix_only_seccomp():
    """PR_SET_NO_NEW_PRIVS and a filter under which socket()/socketpair() work only for AF_UNIX (io_uring refused)."""
    machine = platform.machine()
    if machine not in SECCOMP_ARCH:
        raise OSError(0, "no seccomp filter for the architecture " + machine)
    audit, nr_socket, nr_socketpair, nr_uring, x32 = SECCOMP_ARCH[machine]
    ld, jeq, jge, ret = 0x20, 0x15, 0x35, 0x06
    deny = SECCOMP_RET_ERRNO | EPERM

    def ins(code, jt, jf, k):
        return struct.pack("HBBI", code, jt, jf, k)

    prog = [ins(ld, 0, 0, 4), ins(jeq, 1, 0, audit), ins(ret, 0, 0, deny), ins(ld, 0, 0, 0)]
    if x32:
        prog += [ins(jge, 0, 1, x32), ins(ret, 0, 0, deny)]
    prog += [ins(jeq, 0, 1, nr_uring), ins(ret, 0, 0, deny),
             ins(jeq, 2, 0, nr_socket), ins(jeq, 1, 0, nr_socketpair), ins(ret, 0, 0, SECCOMP_RET_ALLOW),
             ins(ld, 0, 0, 16),                       # args[0], low word: the address family
             ins(jeq, 0, 1, socket.AF_UNIX), ins(ret, 0, 0, SECCOMP_RET_ALLOW), ins(ret, 0, 0, deny)]
    code = b"".join(prog)
    buf = ctypes.create_string_buffer(code, len(code))

    class SockFprog(ctypes.Structure):
        _fields_ = [("len", ctypes.c_ushort), ("filter", ctypes.c_void_p)]

    fprog = SockFprog(len(code) // 8, ctypes.addressof(buf))
    _prctl(PR_SET_NO_NEW_PRIVS, 1)
    _prctl(PR_SET_SECCOMP, SECCOMP_MODE_FILTER, ctypes.addressof(fprog))


def seccomp_selftest():
    """True when no AF_INET / AF_INET6 socket can be created and an AF_UNIX one can."""
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


def trim_capabilities(keep=SUPERVISOR_CAPS):
    """Drop every capability not in `keep` from the bounding, effective, permitted and inheritable sets."""
    for cap in range(64):
        if cap not in keep:
            try:
                _prctl(PR_CAPBSET_DROP, cap)
            except OSError:
                pass                                  # beyond the kernel's last capability

    class Header(ctypes.Structure):
        _fields_ = [("version", ctypes.c_uint32), ("pid", ctypes.c_int)]

    class Data(ctypes.Structure):
        _fields_ = [("effective", ctypes.c_uint32), ("permitted", ctypes.c_uint32), ("inheritable", ctypes.c_uint32)]

    mask = 0
    for cap in keep:
        mask |= 1 << cap
    hdr = Header(0x20080522, 0)
    data = (Data * 2)(Data(mask & 0xFFFFFFFF, mask & 0xFFFFFFFF, 0), Data(mask >> 32, mask >> 32, 0))
    lib = ctypes.CDLL(None, use_errno=True)
    if lib.capset(ctypes.byref(hdr), ctypes.byref(data)) != 0:
        err = ctypes.get_errno()
        raise OSError(err, os.strerror(err))


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def check_digests(folder, want, what):
    """The files of `folder` are exactly `want` (name -> sha256), regular files, or RuntimeError."""
    have = {}
    for name in sorted(os.listdir(folder)):
        p = os.path.join(folder, name)
        if name == "__pycache__":
            continue
        if os.path.islink(p) or not os.path.isfile(p):
            raise RuntimeError("%s: %s is not a regular file" % (what, name))
        have[name] = sha256_file(p)
    if have != dict(want):
        raise RuntimeError("%s in %s differs from the pinned digests (%s)" % (
            what, folder, ", ".join(sorted(set(have) ^ set(want)) or [n for n in have if have[n] != want.get(n)])[:300]))


def _finite(x):
    """The record's values as strict JSON: non-finite numbers become null, numpy scalars plain numbers."""
    if isinstance(x, dict):
        return {str(k): _finite(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_finite(v) for v in x]
    if isinstance(x, bool) or x is None or isinstance(x, str):
        return x
    if isinstance(x, int):
        return x
    if hasattr(x, "dtype") and getattr(x.dtype, "kind", "") in "iub":
        return bool(x) if x.dtype.kind == "b" else int(x)
    try:
        v = float(x)
    except (TypeError, ValueError):
        return str(x)
    return v if math.isfinite(v) else None


def _jsonable(obj):
    if hasattr(obj, "tolist"):
        return obj.tolist()
    if isinstance(obj, (set, frozenset)):
        return sorted(obj, key=str)
    return str(obj)


class Log(object):
    """Append-only JSON lines in the record directory (0600), size-capped."""

    def __init__(self, path, cap):
        self.cap = int(cap)
        self.fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
        st = os.fstat(self.fd)
        if not stat.S_ISREG(st.st_mode) or st.st_nlink != 1:
            os.close(self.fd)
            raise RuntimeError("%s is not a plain file" % path)
        self.size = st.st_size

    def write(self, **rec):
        rec = dict({"t": round(time.time(), 3), "pid": os.getpid()}, **rec)
        line = (json.dumps(rec, default=str, separators=(",", ":")) + "\n").encode()
        if self.size + len(line) > self.cap:
            return
        try:
            os.write(self.fd, line)
            self.size += len(line)
        except OSError:
            pass

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass


class Store(object):
    """The record directory: atomic JSON files (a temp file, fsync, rename, the directory fsynced)."""

    def __init__(self, root):
        self.root = root

    def path(self, name):
        return os.path.join(self.root, name)

    def write_bytes(self, name, data):
        tmp = self.path(name + ".tmp")
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
        try:
            view = memoryview(data)
            while len(view):
                view = view[os.write(fd, view):]
            os.fsync(fd)
        finally:
            os.close(fd)
        os.replace(tmp, self.path(name))
        dfd = os.open(self.root, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(dfd)
        finally:
            os.close(dfd)

    def write_json(self, name, obj):
        self.write_bytes(name, (json.dumps(_finite(obj), indent=1, allow_nan=False) + "\n").encode("utf-8"))

    def read_json(self, name, cap=4 << 20):
        try:
            fd = os.open(self.path(name), os.O_RDONLY | os.O_NOFOLLOW)
        except OSError:
            return None
        try:
            st = os.fstat(fd)
            if not stat.S_ISREG(st.st_mode) or st.st_size > cap:
                return None
            chunks = []
            while True:
                b = os.read(fd, 1 << 20)
                if not b:
                    break
                chunks.append(b)
        finally:
            os.close(fd)
        try:
            return json.loads(b"".join(chunks).decode("utf-8"))
        except ValueError:
            return None


# ---------------------------------------------------------------------------------------------- configuration
def load_config(path):
    path = os.path.abspath(path)
    with open(path, "rb") as fh:
        raw = json.loads(fh.read(1 << 20))
    if not isinstance(raw, dict):
        raise ValueError("the configuration must be a JSON object")
    unknown = set(raw) - set(REQUIRED) - set(DEFAULTS)
    missing = [k for k in REQUIRED if k not in raw]
    if unknown or missing:
        raise ValueError("configuration keys: unknown %s, missing %s" % (sorted(unknown), missing))
    cfg = dict(DEFAULTS)
    cfg.update(raw)
    base = os.path.dirname(path)
    for key in ("episode_file", "cell_dir"):
        cfg[key] = os.path.normpath(os.path.join(base, cfg[key]))
    for key in ("socket", "record_dir", "work_dir", "submission_dir"):
        if not os.path.isabs(cfg[key]):
            raise ValueError("%s must be an absolute path" % key)
    if os.path.dirname(cfg["work_dir"]) != os.path.dirname(cfg["socket"]):
        raise ValueError("the worker's own directory must sit in the socket's root-owned directory")
    if os.path.dirname(cfg["socket"]) in ("/", "/tmp", "/var/tmp", "/dev/shm", "/run"):
        raise ValueError("the socket must live in its own root-owned directory")
    if os.path.dirname(cfg["record_dir"].rstrip("/")) in ("/", "/tmp", "/var/tmp", "/dev/shm"):
        raise ValueError("the record directory must live in a root-owned parent")
    for key in ("prep_s", "graded_s"):
        cfg[key] = float(cfg[key])
        if not (math.isfinite(cfg[key]) and cfg[key] > 0):
            raise ValueError("%s must be a positive number" % key)
    if "/" in cfg["snapshot_name"] or cfg["snapshot_name"] in ("", ".", ".."):
        raise ValueError("snapshot_name is one folder name")
    if not all(isinstance(k, str) and isinstance(v, str) for k, v in cfg["env"].items()):
        raise ValueError("env maps strings to strings")
    return cfg


def _secure_dir(path, mode, uid, gid):
    """Create (or check) a directory owned by uid:gid with exactly `mode`; refuse links and foreign owners."""
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


def _account(cfg):
    acct = pwd.getpwnam(cfg["user"])
    if acct.pw_uid == 0:
        raise RuntimeError("the service account must not be root")
    agent = pwd.getpwnam(cfg["agent_user"])
    if agent.pw_uid == acct.pw_uid or agent.pw_uid == 0:
        raise RuntimeError("the service account must differ from the agent's, and the agent must not be root")
    return acct, agent


# ---------------------------------------------------------------------------------------------- the freeze (supervisor, root)
class SnapshotTooLarge(Exception):
    """The submission folder exceeds a cap of the freeze (files, bytes, entries or depth)."""


def snapshot_digest(entries):
    """sha256 of the sorted (path, size, content sha256) list: the record's snapshot_sha256, recomputed by the verifier."""
    h = hashlib.sha256()
    for rel, size, sha in sorted(entries):
        h.update(rel.encode("utf-8", "surrogateescape") + b"\0" + str(int(size)).encode() + b"\0" + sha.encode() + b"\n")
    return h.hexdigest()


def _remove_tree(path):
    if os.path.islink(path) or (os.path.lexists(path) and not os.path.isdir(path)):
        os.unlink(path)
    elif os.path.isdir(path):
        shutil.rmtree(path)


def snapshot_submission(cfg, agent_uid):
    """DM3, run by the root supervisor on the worker's request: the agent's submission folder as it is now, copied into
    the root-only folder record_dir/snapshot_name. No input but the request: a fixed source and a fixed target, so the
    worker cannot have root copy anything else. Every path is opened relative to its parent's descriptor with
    O_NOFOLLOW, so no link is ever followed; links, special files, entries the agent's account does not own and an
    entry that changes kind meanwhile are skipped and counted. Main's caps (snapshot_max_files regular files,
    snapshot_max_bytes bytes: the grader's 5000 files and 200 MB) and bounds on entries and depth: over any, status
    too_large and no copy. A file keeps its permission bits (less group and other write), as the engine keeps an
    artifact's mode. The copy is built beside the target and renamed into place. A missing folder (or a link in its
    place) freezes as an empty one, as the engine's artifact copy skips a link. Returns {status, files, bytes, sha256,
    skipped, entries, reason}."""
    dst = os.path.join(cfg["record_dir"], cfg["snapshot_name"])
    tmp = dst + ".partial"
    _remove_tree(tmp)
    os.mkdir(tmp, 0o700)
    st = {"files": 0, "bytes": 0, "entries": 0, "skipped": 0}
    listing = []
    cap_files, cap_bytes = int(cfg["snapshot_max_files"]), int(cfg["snapshot_max_bytes"])
    cap_entries, cap_depth = int(cfg["snapshot_max_entries"]), int(cfg["snapshot_max_depth"])

    def copy_file(sfd, name, dfd, rel):
        try:
            ffd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=sfd)
        except OSError:
            st["skipped"] += 1
            return
        try:
            info = os.fstat(ffd)
            if not stat.S_ISREG(info.st_mode) or info.st_uid != agent_uid:
                st["skipped"] += 1
                return
            st["files"] += 1
            if st["files"] > cap_files:
                raise SnapshotTooLarge("files")
            out = os.open(name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600, dir_fd=dfd)
            h, size = hashlib.sha256(), 0
            try:
                while True:
                    b = os.read(ffd, 1 << 20)
                    if not b:
                        break
                    size += len(b)
                    st["bytes"] += len(b)
                    if st["bytes"] > cap_bytes:
                        raise SnapshotTooLarge("bytes")
                    h.update(b)
                    view = memoryview(b)
                    while len(view):
                        view = view[os.write(out, view):]
                os.fchmod(out, stat.S_IMODE(info.st_mode) & 0o755)
            finally:
                os.close(out)
            listing.append((rel, size, h.hexdigest()))
        finally:
            os.close(ffd)

    def walk(sfd, dfd, prefix, depth):
        if depth > cap_depth:
            raise SnapshotTooLarge("depth")
        with os.scandir(sfd) as it:
            names = sorted(e.name for e in it)
        for name in names:
            st["entries"] += 1
            if st["entries"] > cap_entries:
                raise SnapshotTooLarge("entries")
            try:
                info = os.stat(name, dir_fd=sfd, follow_symlinks=False)
            except OSError:
                st["skipped"] += 1
                continue
            if stat.S_ISREG(info.st_mode):
                copy_file(sfd, name, dfd, prefix + name)
            elif stat.S_ISDIR(info.st_mode) and info.st_uid == agent_uid:
                try:
                    cfd = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=sfd)
                except OSError:
                    st["skipped"] += 1
                    continue
                try:
                    if os.fstat(cfd).st_uid != agent_uid:
                        st["skipped"] += 1
                        continue
                    os.mkdir(name, 0o700, dir_fd=dfd)
                    cdfd = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=dfd)
                    try:
                        walk(cfd, cdfd, prefix + name + "/", depth + 1)
                    finally:
                        os.close(cdfd)
                finally:
                    os.close(cfd)
            else:
                st["skipped"] += 1                    # a link, a socket, a fifo, a device, a foreign folder

    status, reason = "ok", None
    try:
        dfd = os.open(tmp, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        try:
            try:
                sfd = os.open(cfg["submission_dir"], os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
            except OSError:
                sfd = None                            # no folder (or a link in its place): an empty program
            if sfd is not None:
                try:
                    if os.fstat(sfd).st_uid == agent_uid:
                        walk(sfd, dfd, "", 0)
                finally:
                    os.close(sfd)
        finally:
            os.close(dfd)
    except SnapshotTooLarge as exc:
        status, reason = "too_large", str(exc)
    if status == "ok":
        _remove_tree(dst)
        os.rename(tmp, dst)
    else:
        _remove_tree(tmp)
        _remove_tree(dst)                             # no copy: the program item scores 0
    return {"status": status, "reason": reason, "files": st["files"] if status == "ok" else None,
            "bytes": st["bytes"] if status == "ok" else None, "skipped": st["skipped"], "entries": st["entries"],
            "sha256": snapshot_digest(listing) if status == "ok" else None}


def _serve_worker_requests(cfg, agent_uid, pid, qr, aw, log):
    """While the worker lives, answer its freeze requests (one byte b"S" -> one JSON line). Returns its exit status."""
    reading = True
    while True:
        try:
            done, code = os.waitpid(pid, os.WNOHANG if reading else 0)
        except ChildProcessError:
            return -1
        except InterruptedError:
            continue
        if done:
            return code
        try:
            ready, _, _ = select.select([qr], [], [], 1.0)
        except InterruptedError:
            continue
        if not ready:
            continue
        try:
            b = os.read(qr, 1)
        except OSError:
            b = b""
        if not b:
            reading = False                           # the worker closed its end: wait for its exit
            continue
        if b != b"S":
            continue
        t0 = time.time()
        try:
            res = snapshot_submission(cfg, agent_uid)
        except Exception as exc:  # noqa: BLE001 - a failed copy refuses the start (the worker's side)
            res = {"status": "failed", "reason": type(exc).__name__}
            log.write(kind="snapshot_failed", tb=traceback.format_exc(limit=8))
        res["wall_s"] = round(time.time() - t0, 3)
        log.write(kind="snapshot", **{k: res.get(k) for k in ("status", "reason", "files", "bytes", "skipped", "wall_s")})
        try:
            os.write(aw, (json.dumps(res) + "\n").encode())
        except OSError:
            pass


# ---------------------------------------------------------------------------------------------- the cell (worker)
class Cell(object):
    """The graded episode's counter on the grader's simulator, run by the offline grader's rules (module docstring)."""

    def __init__(self, env, ep, E, EP, mujoco, unit_id, first):
        self.env, self.ep, self.E, self.EP, self.mujoco = env, ep, E, EP, mujoco
        self.unit_id = unit_id
        self.first = first                    # the reset's observation (five images), served at the start
        self.images = 0
        self.last = None                      # the latest observation, wire form, as last served
        self.done = False                     # the episode's own end (Episode.step's done)
        self.fatal = None                     # a simulator fatal error (mujoco.FatalError): the episode scores 0
        self.error = None                     # the cell's own failure (its renderer)
        self.privileged = False
        self.refused = 0

    def wire(self, obs):
        """The grader's wire observation (jar_grade.wire_obs): the observation, the unit's id, and the truth for the
        token's episode only."""
        w = self.E.to_wire(obs)
        w["unit"] = self.unit_id
        if self.privileged:
            w["truth"] = self.env.truth()
        return w

    @property
    def over(self):
        return bool(self.done or self.fatal or self.error)

    @property
    def steps(self):
        return int(self.env.t)

    @property
    def max_steps(self):
        return int(self.EP.EPISODE_STEPS)

    def first_observation(self):
        self.last = self.wire(self.first)
        self.first = None
        return self.last

    def step(self, action):
        try:
            cams = self.E.check_action(action)["cams"]
        except (ValueError, TypeError, OverflowError, KeyError, AttributeError):
            self.refused += 1
            return {"error": "bad_action", "detail": ERRORS["bad_action"]}
        try:
            obs, done, _info = self.ep.step(action)
        except self.mujoco.FatalError as exc:       # as the grader: that episode scores 0
            self.fatal = str(exc)[:300]
            return {"obs": None, "done": True}
        except Exception as exc:  # noqa: BLE001 - the renderer (RenderError) or the simulator itself: the grader's own error
            self.error = "%s: %s" % (type(exc).__name__, str(exc)[:300])
            return {"error": "internal", "detail": "the cell failed; the graded episode is over"}
        self.images += bin(int(cams)).count("1")
        self.done = bool(done or self.env.unstable())
        self.last = self.wire(obs)
        return {"obs": self.last, "done": self.done}

    def scored(self):
        """Episode.score as the grader takes it at the episode's end (credit 0 after a fault). After a simulator fatal
        error the grader scores the episode 0 without reading its state, and so does this: (0.0, None)."""
        if self.fatal:
            return 0.0, None
        sc = self.ep.score()
        credit = float(sc["credit"])
        if self.env.unstable() or not math.isfinite(credit) or credit < 0:
            credit = 0.0
        return credit, sc

    def score(self):
        return self.scored()[0]

    def end_text(self):
        if self.error:
            return "cell error"
        if self.fatal:
            return "simulator fatal error at step %d" % self.steps
        if self.env.unstable():
            return "unstable simulation at step %d" % self.steps
        if self.done and self.steps < self.max_steps:
            return "every listed jar closed again and both hands clear at step %d" % self.steps
        if self.steps >= self.max_steps:
            return "all %d steps run" % self.max_steps
        return None

    def metrics(self):
        credit, sc = self.scored()
        sc = sc or {"jars": {}, "penalized": False}
        jars = list(sc["jars"].values())
        n = float(len(jars)) or 1.0
        fault = bool(self.fatal or self.env.unstable())
        return {"mean_episode_credit": credit, "part_done": sum(1.0 for j in jars if j["done"]) / n,
                "part_filled": sum(1.0 for j in jars if j["filled"]) / n,
                "part_sealed": sum(1.0 for j in jars if j["sealed"]) / n,
                "part_halved": 1.0 if sc["penalized"] else 0.0, "steps": float(self.steps),
                "images_requested": float(self.images), "sim_fault": 1.0 if fault else 0.0,
                "privileged_mode": 1.0 if self.privileged else 0.0, "actions_refused": float(self.refused),
                "cell_error": 1.0 if self.error else 0.0}

    def detail(self):
        credit, sc = self.scored()
        return {"end": self.end_text(), "unit": self.unit_id, "credit": credit, "score": sc, "fatal": self.fatal,
                "cell_error": self.error}


def build_cell(cfg, ep):
    """Check and import the grader's simulator, build the graded episode as the offline grader builds it and reset it
    (the first observation, with its five images, is drawn here). Runs as root before the privilege drop: every module
    is read here, since the worker cannot read online/ afterwards. The numerics pins are the image's (its .pth module
    jar_numerics_pin, run again because the supervisor started from a cleared environment), set before numpy loads."""
    for k, v in cfg["env"].items():
        os.environ[k] = v                             # before numpy, MuJoCo and the renderer load: the grader's settings
    import jar_numerics_pin                           # the image's .pth module: its defaults, for this environment
    importlib.reload(jar_numerics_pin)
    check_digests(cfg["cell_dir"], cfg["cell_sha256"], "the simulator")
    sys.path.insert(0, cfg["cell_dir"])
    try:
        E = importlib.import_module("jar_env")
        EP = importlib.import_module("episode")
        importlib.import_module("jar_render")        # jar_sim imports it at the first image: here, while it is readable
        import mujoco
        env = E.JarEnv(ep["unit"], ep["unit_id"], render=True)
        episode = EP.Episode(env, int(ep["seed"]))
        first = episode.reset()
    finally:
        while cfg["cell_dir"] in sys.path:
            sys.path.remove(cfg["cell_dir"])
    return Cell(env, episode, E, EP, mujoco, ep["unit_id"], first)


def read_episode(cfg):
    with open(cfg["episode_file"], "rb") as fh:
        ep = json.loads(fh.read(1 << 20))
    for k in ("episode", "seed", "seed_sha256", "unit_id", "unit", "oracle_token_sha256"):
        if k not in ep:
            raise ValueError("episode.json lacks %s" % k)
    if hashlib.sha256(str(int(ep["seed"])).encode()).hexdigest() != ep["seed_sha256"]:
        raise ValueError("episode.json: the seed does not match its digest")
    return ep


# ---------------------------------------------------------------------------------------------- the episode (worker)
class Client(object):
    def __init__(self, sock, cid, peer_uid):
        self.sock, self.cid, self.peer_uid, self.buf = sock, cid, peer_uid, b""
        self.last_rx = time.time()


class GradedEpisode(object):
    """The one graded episode, its windows and its record, served to one client at a time."""

    def __init__(self, cfg, ep, store, log, listener, cell, state, snap_pipe=None):
        self.cfg, self.ep, self.store, self.log, self.listener = cfg, ep, store, log, listener
        self.snap_pipe = snap_pipe
        self.cell, self.state = cell, state
        self.client = None
        self.conn_seq = 0
        self.requests = 0
        self.dirty = False
        self.last_write = 0.0
        self.token_mismatch = False
        self.req_log = None
        self.req_log_size = 0
        self.req_log_full = False
        self.freeze_tries = 0
        self.freeze_retry_at = 0.0
        self.last = {}                                # what the last record said (a restarted worker has no cell)

    # -- state
    @property
    def started(self):
        return bool(self.state.get("started"))

    @property
    def ended(self):
        return bool(self.state.get("ended"))

    @property
    def end_reason(self):
        return self.state.get("end_reason")

    def begin(self):
        """After the privilege drop: count the worker's start, end an episode a dead worker left running."""
        now = time.time()
        self.state["worker_starts"] = int(self.state.get("worker_starts", 0)) + 1
        if self.started and not self.ended:
            rec = self.store.read_json("record.json")
            if not (isinstance(rec, dict) and rec.get("format") == RECORD_FORMAT and rec.get("episode") == self.ep["episode"]):
                rec = self._minimal_record(now)
            rec.update({"ended": True, "end_reason": "worker_restart"})
            self.store.write_json("record.json", rec)
            self.state.update({"ended": True, "end_reason": "worker_restart", "ended_wall": now})
            self.log.write(kind="episode_end", reason="worker_restart", note="the worker restarted: the last record stands")
        if self.started:
            rec = self.store.read_json("record.json") or {}
            self.last = {k: rec.get(k) for k in ("steps", "sim_time_s", "requests")}
            self.last.update({k: (rec.get("metrics") or {}).get(k) for k in ("images_requested",)})
        if self.state.get("frozen") == "prep_window_closed" and (self.state.get("snapshot") or {}).get("status") == "pending":
            self.freeze_tries = int(self.state.get("freeze_tries", 0))   # a restarted worker goes on with the freeze
        self.store.write_json("state.json", self.state)
        try:
            fd = os.open(self.store.path("requests.jsonl"), os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
            self.req_log_size = os.fstat(fd).st_size
            self.req_log = fd
        except OSError as e:
            self.log.write(kind="requests_log_unavailable", err=str(e))

    def release_reserve(self):
        try:
            os.unlink(self.store.path("reserve"))
        except OSError:
            pass

    def request_snapshot(self):
        """DM3: ask the root supervisor to freeze the submission folder now. Its result (a dict) or None (no answer)."""
        if not self.snap_pipe:
            return None
        qw, ar = self.snap_pipe
        while select.select([ar], [], [], 0)[0]:     # a late answer to an earlier request that timed out
            if not os.read(ar, 65536):
                return None
        try:
            os.write(qw, b"S")
        except OSError:
            return None
        deadline, buf = time.time() + float(self.cfg["snapshot_timeout_s"]), b""
        while not buf.endswith(b"\n"):
            left = deadline - time.time()
            if left <= 0 or not select.select([ar], [], [], left)[0]:
                return None
            chunk = os.read(ar, 65536)
            if not chunk:
                return None
            buf += chunk
        try:
            return json.loads(buf.decode("utf-8"))
        except ValueError:
            return None

    def snapshot_fields(self):
        sn = self.state.get("snapshot") or {}
        return {"freeze": self.state.get("frozen"), "freeze_wall": self.state.get("frozen_wall"),
                "snapshot_status": sn.get("status"), "snapshot_files": sn.get("files"),
                "snapshot_bytes": sn.get("bytes"), "snapshot_sha256": sn.get("sha256"),
                "snapshot_skipped": sn.get("skipped")}

    def _unstarted_record(self, now):
        return {"format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.ep["episode"],
                "seed_sha256": self.ep["seed_sha256"], "started": False, "ended": False, "end_reason": None,
                "requests": 0, "steps": 0, "sim_time_s": 0.0, "wall_s_since_start": None, "episode_score": 0.0,
                "metrics": {}, "privileged": False, "service_version": SERVICE_VERSION,
                "written_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)), **self.snapshot_fields()}

    def freeze_at_window_close(self, now):
        """The start window closed with no start: the program freezes now, as it would have at a start (the same copy by
        the root supervisor), and the record says so before the copy is tried (snapshot pending), so that a copy that
        never succeeds scores the program 0 rather than letting the session-end folder be graded. Retried a few times;
        then recorded as failed."""
        if self.state.get("frozen") != "prep_window_closed":
            self.release_reserve()                    # room for this record even on a full disk
            self.state.update({"frozen": "prep_window_closed", "frozen_wall": now,
                               "snapshot": {"status": "pending"}, "freeze_tries": 0})
            try:
                self.store.write_json("state.json", self.state)
                self.store.write_json("record.json", self._unstarted_record(now))
            except OSError:
                self.log.write(kind="window_close_record_failed", tb=traceback.format_exc(limit=4))
        snap = self.request_snapshot()
        self.freeze_tries += 1
        if isinstance(snap, dict) and snap.get("status") in ("ok", "too_large"):
            sn = {k: snap.get(k) for k in ("status", "reason", "files", "bytes", "sha256", "skipped")}
        elif self.freeze_tries < int(self.cfg["window_freeze_tries"]):
            self.log.write(kind="window_close_freeze_retry", result=snap, tries=self.freeze_tries)
            self.state["freeze_tries"] = self.freeze_tries
            self.freeze_retry_at = time.time() + float(self.cfg["window_freeze_retry_s"])
            try:
                self.store.write_json("state.json", self.state)
            except OSError:
                pass
            return
        else:
            sn = {"status": "failed", "reason": (snap or {}).get("reason") if isinstance(snap, dict) else "no answer"}
        self.state.update({"snapshot": sn, "freeze_tries": self.freeze_tries})
        try:
            self.store.write_json("state.json", self.state)
            self.store.write_json("record.json", self._unstarted_record(time.time()))
        except OSError:
            self.log.write(kind="window_close_record_failed", tb=traceback.format_exc(limit=4))
        self.log.write(kind="frozen_at_window_close", **{k: sn.get(k) for k in ("status", "files", "bytes", "skipped")})

    def _minimal_record(self, now):
        start = float(self.state.get("start_wall") or now)
        rec = self._unstarted_record(now)
        rec.update({"started": True, "wall_s_since_start": now - start, "privileged": bool(self.state.get("privileged"))})
        return rec

    # -- the record
    def _record(self, now):
        c = self.cell
        m = c.metrics()
        m["oracle_token_mismatch"] = 1.0 if self.token_mismatch else 0.0
        return {"format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.ep["episode"],
                "seed_sha256": self.ep["seed_sha256"], "started": True, "ended": self.ended, "end_reason": self.end_reason,
                "requests": int(self.requests), "steps": c.steps, "sim_time_s": c.steps * 0.1,
                "wall_s_since_start": now - float(self.state["start_wall"]), "episode_score": c.score(),
                "metrics": m, "privileged": bool(c.privileged), "cell_error": bool(c.error),
                "started_after_s": float(self.state["start_wall"]) - float(self.state["service_started"]),
                "service_version": SERVICE_VERSION, **self.snapshot_fields(),
                "written_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)), "detail": c.detail()}

    def write_record(self, now):
        try:
            rec = self._record(now)
            data = (json.dumps(_finite(rec), indent=1, allow_nan=False) + "\n").encode("utf-8")
            if len(data) > int(self.cfg["record_max_bytes"]):
                rec.pop("detail", None)
                data = (json.dumps(_finite(rec), indent=1, allow_nan=False) + "\n").encode("utf-8")
            self.store.write_bytes("record.json", data)
            self.last_write, self.dirty = now, False
        except Exception:  # noqa: BLE001 - a failed write is retried at the next flush; the last record stands
            self.log.write(kind="record_write_failed", tb=traceback.format_exc(limit=8))

    def flush(self, now, force=False):
        if self.cell is not None and self.started and (self.dirty or force) and \
                (force or now - self.last_write >= float(self.cfg["record_interval_s"])):
            self.write_record(now)

    def _log_request(self, now, method, params, res):
        if self.req_log is None or self.req_log_full:
            return
        try:
            if method == "start":
                p = json.dumps({"token": "<given>" if isinstance(params, dict) and params.get("token") else None})
            else:
                p = json.dumps(params, default=str)[:600]
        except (TypeError, ValueError):
            p = "<unprintable>"
        err = res.get("error") if isinstance(res, dict) else None
        step = self.cell.steps if self.cell is not None and self.started else None
        line = json.dumps({"n": self.requests, "t": round(now - float(self.state.get("start_wall") or now), 3),
                           "method": method[:64], "params": p, "step": step, "error": err}, default=str) + "\n"
        data = line.encode("utf-8")
        if self.req_log_size + len(data) > int(self.cfg["requests_log_cap_bytes"]):
            self.req_log_full = True
            self.log.write(kind="requests_log_full", bytes=self.req_log_size)
            return
        try:
            os.write(self.req_log, data)
            self.req_log_size += len(data)
        except OSError:
            pass

    # -- the windows and the end
    def prep_deadline(self):
        return float(self.state["service_started"]) + float(self.cfg["prep_s"])

    def graded_deadline(self):
        return float(self.state["start_wall"]) + float(self.cfg["graded_s"])

    def end(self, reason, now):
        """End the episode as the offline grader ends one whose controller stops: over, scored as it stands."""
        if self.ended or not self.started:
            return
        self.state.update({"ended": True, "end_reason": reason, "ended_wall": now})
        try:
            self.store.write_json("state.json", self.state)
        except OSError:
            self.log.write(kind="state_write_failed", tb=traceback.format_exc(limit=4))
        self.flush(now, force=True)
        self.log.write(kind="episode_end", reason=reason, cell_end=self.cell.end_text() if self.cell else None,
                       requests=self.requests)

    def windows(self, now):
        if self.started and not self.ended and now >= self.graded_deadline():
            self.end("graded_window", now)
        if not self.started and not self.state.get("state_lost") and now >= self.prep_deadline() and \
                now >= self.freeze_retry_at and (self.state.get("frozen") != "prep_window_closed" or
                                                  (self.state.get("snapshot") or {}).get("status") == "pending"):
            self.freeze_at_window_close(now)

    # -- requests
    def status(self, now):
        c = self.cell
        if c is not None and self.started:
            steps, images = c.steps, int(c.images)
        else:
            steps = int(self.last.get("steps") or 0)
            images = int(self.last.get("images_requested") or 0)
        if not self.started:
            prep_left = max(0.0, self.prep_deadline() - now)
            graded_left = float(self.cfg["graded_s"]) if prep_left > 0 else 0.0
        else:
            prep_left = 0.0
            graded_left = 0.0 if self.ended else max(0.0, self.graded_deadline() - now)
        return {"started": self.started, "ended": self.ended, "end": self.end_reason, "steps": steps,
                "max_steps": c.max_steps if c is not None else None, "images_requested": images,
                "prep_window_left_s": round(prep_left, 1), "graded_window_left_s": round(graded_left, 1),
                "program_frozen": bool(self.state.get("frozen"))}

    def start(self, params, now):
        if not isinstance(params, dict):
            return {"error": "bad_params", "detail": ERRORS["bad_params"]}
        if self.started:
            return {"error": "already_started", "detail": ERRORS["already_started"]}
        if now > self.prep_deadline() or self.cell is None or self.state.get("frozen"):
            return {"error": "prep_window_closed", "detail": ERRORS["prep_window_closed"]}
        token = params.get("token")
        privileged = False
        if token is not None:
            got = hashlib.sha256(str(token).strip().encode("utf-8")).hexdigest()
            privileged = got == self.ep["oracle_token_sha256"]
            self.token_mismatch = not privileged
        # DM3: the program is frozen before anything is charged; a copy that fails refuses the start
        snap = self.request_snapshot()
        if not isinstance(snap, dict) or snap.get("status") not in ("ok", "too_large"):
            self.log.write(kind="start_refused_snapshot", result=snap)
            return {"error": "internal", "detail": "the submission folder could not be frozen: the graded episode did not "
                                                   "start and nothing is charged"}
        now = time.time()
        # the charge: persisted before anything is simulated, never refunded
        self.state.update({"started": True, "start_wall": now, "privileged": privileged, "frozen": "graded_start",
                           "frozen_wall": now,
                           "snapshot": {k: snap.get(k) for k in ("status", "reason", "files", "bytes", "sha256", "skipped")}})
        try:
            self.store.write_json("state.json", self.state)
        except OSError:
            self.state.update({"started": False, "start_wall": None, "privileged": False, "snapshot": None,
                               "frozen": None, "frozen_wall": None})
            self.log.write(kind="start_not_persisted", tb=traceback.format_exc(limit=4))
            return {"error": "internal", "detail": ERRORS["internal"]}
        self.release_reserve()
        self.cell.privileged = privileged
        obs = self.cell.first_observation()
        self.log.write(kind="episode_start", privileged=privileged, token_mismatch=self.token_mismatch,
                       peer_uid=self.client.peer_uid if self.client else None)
        self.write_record(now)
        return {"ok": True, "obs": obs, "max_steps": self.cell.max_steps, "graded_window_s": float(self.cfg["graded_s"])}

    def dispatch(self, method, params, now):
        if method == "status":
            return self.status(now)
        if method == "start":
            res = self.start(params if params is not None else {}, now)
            self._log_request(now, "start", params, res)
            return res
        if method not in ("step", "observe", "finish"):
            return {"error": "forbidden", "detail": ERRORS["forbidden"]}
        if not self.started:
            return {"error": "not_started", "detail": ERRORS["not_started"]}
        if method == "observe":
            if self.cell is None or self.cell.last is None:
                return {"error": "episode_over", "detail": "%s (%s)" % (ERRORS["episode_over"], self.end_reason)}
            return {"obs": self.cell.last, "done": self.ended}
        if self.ended:
            if method == "finish":
                return {"ok": True, "ended": True, "steps": int(self.status(now)["steps"])}
            return {"error": "episode_over", "detail": "%s (%s)" % (ERRORS["episode_over"], self.end_reason)}
        self.requests += 1
        if method == "finish":
            self.end("finish", now)
            res = {"ok": True, "ended": True, "steps": self.cell.steps}
            self._log_request(now, method, params, res)
            return res
        if not isinstance(params, dict):
            return {"error": "bad_params", "detail": ERRORS["bad_params"]}
        res = self.cell.step(params.get("action"))
        self.dirty = True
        self._log_request(now, method, {"action": params.get("action")}, res)
        if self.cell.over:
            self.end("done", time.time())
        return res

    def _send(self, obj):
        if self.client is None:
            return
        try:
            data = (json.dumps(obj, default=_jsonable) + "\n").encode("utf-8")
            self.client.sock.sendall(data)
        except (OSError, ValueError, TypeError) as e:
            self.drop("send failed (%s)" % type(e).__name__)

    def request(self, line):
        now = time.time()
        self.windows(now)
        try:
            req = json.loads(line.decode("utf-8"))
            if not isinstance(req, dict) or "method" not in req:
                raise ValueError("not a request")
        except (ValueError, UnicodeDecodeError, RecursionError):
            self._send({"id": None, "error": "bad_request", "detail": ERRORS["bad_request"]})
            return
        rid, method = req.get("id"), str(req.get("method"))
        try:
            res = self.dispatch(method, req.get("params"), now)
        except Exception:  # noqa: BLE001 - the service's own failure: the episode is not over, the request failed
            self.log.write(kind="internal_error", method=method[:64], tb=traceback.format_exc(limit=12))
            res = {"error": "internal", "detail": ERRORS["internal"]}
        if isinstance(res, dict) and "error" in res and set(res) <= {"error", "detail"}:
            msg = dict({"id": rid}, **res)
        else:
            msg = {"id": rid, "result": res}
        self._send(msg)
        self.flush(time.time())

    # -- connections
    def accept(self):
        try:
            conn, _ = self.listener.accept()
        except (BlockingIOError, InterruptedError):
            return
        except OSError as e:
            self.log.write(kind="accept_error", err=str(e))
            time.sleep(0.05)
            return
        try:
            peer_uid = struct.unpack("3i", conn.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))[1]
        except OSError:
            peer_uid = -1
        if self.client is not None:
            try:
                conn.settimeout(1.0)
                conn.sendall((json.dumps({"id": None, "error": "busy", "detail": ERRORS["busy"]}) + "\n").encode())
            except OSError:
                pass
            conn.close()
            self.log.write(kind="busy", peer_uid=peer_uid)
            return
        self.conn_seq += 1
        conn.settimeout(float(self.cfg["reply_timeout_s"]))
        self.client = Client(conn, self.conn_seq, peer_uid)
        self.log.write(kind="connect", conn=self.conn_seq, peer_uid=peer_uid)

    def drop(self, why):
        if self.client is None:
            return
        try:
            self.client.sock.close()
        except OSError:
            pass
        self.log.write(kind="disconnect", conn=self.client.cid, why=why)
        self.client = None
        self.flush(time.time(), force=self.dirty)

    def readable(self):
        c = self.client
        try:
            chunk = c.sock.recv(1 << 20)
        except (BlockingIOError, InterruptedError, socket.timeout):
            return
        except OSError as e:
            self.drop("recv failed (%s)" % type(e).__name__)
            return
        if not chunk:
            self.drop("eof")
            return
        c.last_rx = time.time()
        c.buf += chunk
        while self.client is c:
            nl = c.buf.find(b"\n")
            if nl < 0:
                if len(c.buf) > MAX_LINE:
                    self._send({"id": None, "error": "bad_request", "detail": "a request line is longer than 1 MiB"})
                    self.drop("line too long")
                break
            line, c.buf = c.buf[:nl], c.buf[nl + 1:]
            if line.strip():
                self.request(line)

    def serve(self):
        self.listener.setblocking(False)
        while True:
            now = time.time()
            self.windows(now)
            if self.client is not None and now - self.client.last_rx > float(self.cfg["idle_s"]):
                self.drop("idle")
            wait = 60.0
            if self.started and not self.ended:
                wait = min(wait, self.graded_deadline() - now)
            if not self.started and (not self.state.get("frozen") or
                                     (self.state.get("snapshot") or {}).get("status") == "pending"):
                wait = min(wait, max(self.prep_deadline(), self.freeze_retry_at) - now)
            if self.dirty:
                wait = min(wait, self.last_write + float(self.cfg["record_interval_s"]) - now)
            if self.client is not None:
                wait = min(wait, self.client.last_rx + float(self.cfg["idle_s"]) - now)
            socks = [self.listener] + ([self.client.sock] if self.client is not None else [])
            try:
                ready, _, _ = select.select(socks, [], [], max(0.0, wait))
            except InterruptedError:
                continue
            if self.listener in ready:
                self.accept()
            if self.client is not None and self.client.sock in ready:
                self.readable()
            self.flush(time.time())


def run_worker(cfg, acct, listener, ready_fd, snap_pipe):
    """Forked from the supervisor as root: read the episode, build its cell, drop privileges, serve."""
    _set_comm("jargrade-wkr")
    os.umask(0o077)
    store = Store(cfg["record_dir"])
    log = Log(store.path("service.log"), cfg["max_log_bytes"])
    os.fchown(log.fd, acct.pw_uid, acct.pw_gid)
    try:
        ep = read_episode(cfg)
        state = store.read_json("state.json")
        if not isinstance(state, dict) or "service_started" not in state:
            # fail closed: without its state the service cannot tell a start from none, so nothing can start
            log.write(kind="state_unreadable")
            state = {"version": 1, "service_started": 0.0, "started": False, "ended": False, "worker_starts": 0,
                     "state_lost": True}
        cell = None
        fresh = not state.get("started") and not state.get("ended") and not state.get("state_lost") \
            and not state.get("frozen")
        os.environ["TMPDIR"] = cfg["work_dir"]
        os.environ["HOME"] = cfg["work_dir"]
        if fresh:
            cell = build_cell(cfg, ep)
        # -- the privilege drop (irreversible)
        os.chdir("/")
        os.setgroups([])
        os.setgid(acct.pw_gid)
        os.setuid(acct.pw_uid)
        try:
            os.setuid(0)
            raise RuntimeError("the privilege drop is reversible")
        except PermissionError:
            pass
        _prctl(PR_SET_DUMPABLE, 0)
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
        sec_ok = seccomp_selftest()
        if not sec_ok:
            raise RuntimeError("seccomp self-test failed: an inet socket could be created")
        episode = GradedEpisode(cfg, ep, store, log, listener, cell, state, snap_pipe)
        episode.begin()
        log.write(kind="worker_ready", uid=os.getuid(), gid=os.getgid(), seccomp_inet_blocked=sec_ok, fresh=fresh,
                  worker_starts=state.get("worker_starts"), service_version=SERVICE_VERSION)
    except Exception:  # noqa: BLE001
        log.write(kind="worker_start_failed", tb=traceback.format_exc(limit=30))
        os.write(ready_fd, b"F")
        os._exit(1)
    os.write(ready_fd, b"R")
    os.close(ready_fd)
    try:
        episode.serve()
    except Exception:  # noqa: BLE001
        log.write(kind="worker_crashed", tb=traceback.format_exc(limit=30))
    os._exit(1)


# ---------------------------------------------------------------------------------------------- supervisor (root)
def _close_fds(keep):
    for fd in range(3, 1024):
        if fd not in keep:
            try:
                os.close(fd)
            except OSError:
                pass


def run_supervisor(cfg, acct, agent, notify_fd):
    """Root, never reads episode.json, never accepts a connection: binds the socket, forks and restarts the worker."""
    _set_comm("jargrade-sup")
    os.umask(0o077)
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    sup_log = Log(os.path.join(cfg["record_dir"], "supervisor.log"), cfg["max_log_bytes"])
    try:
        install_unix_only_seccomp()
    except OSError as exc:
        sup_log.write(kind="seccomp_install_failed", err=str(exc))
        os.write(notify_fd, b"F")
        os._exit(1)
    try:
        trim_capabilities()
        caps = True
    except OSError as exc:
        caps = False
        sup_log.write(kind="capability_trim_failed", err=str(exc))
    sock_path = cfg["socket"]
    if os.path.lexists(sock_path):
        os.unlink(sock_path)
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(sock_path)
    os.chmod(sock_path, 0o666)
    os.chown(sock_path, acct.pw_uid, acct.pw_gid)
    listener.listen(16)
    sup_log.write(kind="supervisor_start", seccomp=True, caps_trimmed=caps, service_version=SERVICE_VERSION)
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
        qr, qw = os.pipe()                            # worker -> supervisor: freeze requests (one byte)
        ar, aw = os.pipe()                            # supervisor -> worker: the freeze's result (one JSON line)
        pid = os.fork()
        if pid == 0:
            for fd in (r, qr, aw):
                os.close(fd)
            sup_log.close()
            if notify_fd is not None:
                try:
                    os.close(notify_fd)
                except OSError:
                    pass
            signal.signal(signal.SIGTERM, signal.SIG_DFL)
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            run_worker(cfg, acct, listener, w, (qw, ar))
            os._exit(1)
        for fd in (w, qw, ar):
            os.close(fd)
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
        code = _serve_worker_requests(cfg, agent.pw_uid, pid, qr, aw, sup_log)
        for fd in (qr, aw):
            os.close(fd)
        stopping["child"] = None
        sup_log.write(kind="worker_exit", worker=pid, status=code, ready=status == b"R")
        if stopping["flag"] or status != b"R":
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
    except OSError:
        pass
    sup_log.write(kind="supervisor_exit", restarts=restarts)
    os._exit(0)


def _init_state(cfg, acct):
    """state.json at the service's first start: when it started (the start window counts from here), no start yet; and
    the reserve that keeps room for the window-close record."""
    store = Store(cfg["record_dir"])
    if os.path.lexists(store.path("state.json")):
        return
    store.write_json("state.json", {"version": 1, "service_started": time.time(), "started": False, "ended": False,
                                    "start_wall": None, "end_reason": None, "worker_starts": 0,
                                    "prep_s": cfg["prep_s"], "graded_s": cfg["graded_s"]})
    os.chown(store.path("state.json"), acct.pw_uid, acct.pw_gid)
    store.write_bytes("reserve", b"\0" * int(cfg["reserve_bytes"]))
    os.chown(store.path("reserve"), acct.pw_uid, acct.pw_gid)


def start(config_path):
    if os.geteuid() != 0:
        print("graded service: must be started as root (environment/setup/setup.sh)", file=sys.stderr)
        return 2
    try:
        cfg = load_config(config_path)
        acct, agent = _account(cfg)
        os.umask(0o077)
        sockdir = os.path.dirname(cfg["socket"])
        os.makedirs(os.path.dirname(sockdir), exist_ok=True)
        _secure_dir(sockdir, 0o755, 0, 0)
        _secure_dir(cfg["work_dir"], 0o700, acct.pw_uid, acct.pw_gid)    # the worker's HOME and TMPDIR
        parent = os.path.dirname(cfg["record_dir"].rstrip("/"))
        os.makedirs(parent, exist_ok=True)
        pst = os.lstat(parent)
        if pst.st_uid != 0 or (pst.st_mode & 0o022) or not stat.S_ISDIR(pst.st_mode):
            raise RuntimeError("%s must be a root-owned directory nobody else can write" % parent)
        _secure_dir(cfg["record_dir"], 0o700, acct.pw_uid, acct.pw_gid)
        _init_state(cfg, acct)
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
            run_supervisor(cfg, acct, agent, w)
        except Exception:  # noqa: BLE001
            try:
                Log(os.path.join(cfg["record_dir"], "supervisor.log"), cfg["max_log_bytes"]).write(
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
        print("graded service: ready (socket %s)" % cfg["socket"], file=sys.stderr)
        return 0
    print("graded service: failed to start (details in %s/service.log)" % cfg["record_dir"], file=sys.stderr)
    return 1


def main(argv=None):
    ap = argparse.ArgumentParser(description="the graded episode's service (start as root from environment/setup/setup.sh)")
    ap.add_argument("--config", required=True)
    args = ap.parse_args(argv)
    return start(args.config)


if __name__ == "__main__":
    sys.exit(main())
