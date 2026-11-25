#!/usr/bin/env python3
"""graded_service: the graded episode of lab_sample_handling (dual mode, experimental).

Started once by environment/setup/setup.sh as root, cwd = the setup stage (/opt/ale/setup, root-only while the agent
works), before the agent's session:

    env -i PATH=/opt/venv/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \\
        /opt/venv/bin/python3 -I -B online/graded_service.py --config online/service.json

It returns once the service serves (exit 0) or failed to start (exit 1); the service itself is a detached daemon
(setsid, stdio on /dev/null) that runs through the agent's session.

The episode. ONE episode of the offline grader's fixed hidden set (online/episode.json, read by the worker only: the
entry's unit, the unit's hidden properties and the 64-bit seed, drawn once from tests/hidden/episodes.json) on the
grader's own simulator: online/sim/ holds the image's byte copies of tests/lab_sim.py, lab_env.py, box_transport_env.py,
lab_render.py and lab_look.json (/opt/labbench-commission, copied by setup.sh before any other account runs) and
online/episode.py (a byte copy of tests/episode.py: the end rules and the credit); the worker refuses to start unless
every file matches service.json's digests. The episode is built as the offline grader builds one
(`lab_env.LabEnv(units[unit], unit, render=True)` in an `episode.Episode` with its seed, the cameras drawn by
lab_render.py from the render bundle) and runs by the same rules: the same bench, worklist, fills, rotor start, lid
start, noise, physics, observation and action formats, the same end rules (20000 control steps, 20 steps after the lid
is shut once it has been open, an unstable simulation) and the same credit (episode.Episode.score). What does not
carry over is what binds a controller process: the per-reply deadline, the controller's time budget with its image
price, the repeated-step probes, the restarts and the process and network checks. A malformed action is refused
(`malformed_action`, nothing simulated) instead of ending the episode. The simulation advances only inside the agent's
own `step` requests.

Rules:
  * `start` is accepted once per session, within `prep_s` of the service's start (state.json `service_started`); it is
    persisted (fsync) before anything is simulated and never refunded. With the reference's token (its sha256 is in
    episode.json) every observation of the episode carries `truth` (the unit, the seed, the full simulation state), as
    the offline grader gives it to the reference's submission; a wrong token is ignored and counted.
  * The episode lives in the worker, not in a connection: a disconnect leaves it where it is and a new connection
    resumes it (`observe` returns the last observation again). One client at a time: a second connection is answered
    `busy` and closed; a connection silent for `idle_s` is closed (the episode stays).
  * It ends at its own end (the rules above: `done`), at the agent's `finish`, `graded_s` after its start (the
    service's own timer: `graded_window`), or when the session ends (the service is frozen or killed: the last record
    stands). The offline grader's episode runs until its own end; an episode ended before it is scored as it stands,
    with nothing simulated afterwards (as a controller that holds still from then on would leave it in this
    quasi-static bench). An ended episode answers `status` and `finish` (which changes nothing) only.
  * When the worker dies or restarts, the episode ends with its last record (end_reason worker_restart): the new worker
    never builds a bench for it.
  * The agent learns the observations (the offline wire's, with the episode's public end rule when it ends) and
    `status` (started / ended, steps, simulated time, the windows' time left): never the credit, the record, the
    episode's index or its seed.
  * The freeze (DM3): before the start is charged, the worker asks the root supervisor to copy the agent's submission
    folder into the root-only record_dir/submission_snapshot/ (regular files and folders of the agent's account only,
    never following a link; main's caps: 5000 entries that are not folders and 200 MiB, counted as the offline grader's
    stage_submission counts them, and the engine's depth limit: over any of them the start goes on with snapshot_status
    too_large and no copy). A copy that fails refuses the start; nothing is charged. tests/verify.py grades that copy
    as the program (main's offline grading) and the record as the graded episode. The program freezes at the earlier
    of the graded start and the close of the start window: when `prep_s` runs out with no start, the worker's own timer
    has the supervisor make the same copy at that moment (up to `snapshot_attempts` tries) and writes a record that
    says so (started false, freeze prep_window_closed; a copy that still failed is recorded as failed and the program
    item scores 0), so later edits are not graded either.
  * The commissioning service closes when the graded episode starts (O13): before the start is persisted, the worker
    writes the flag `practice_flag` (/run/labgrade/flags/started: world-readable, in a directory only this service's
    account can write, inside the root-owned socket directory), and the commissioning service
    (environment/commission_service/commissiond.py) ends its open benches and answers every request
    `commissioning_closed` from then on. A start whose flag cannot be written is refused (nothing charged); a
    restarted worker writes the flag again if the episode had started.

Process layout (the alelab daemon pattern of arm_hidden_mechanism_box, as pump_skid_startup_and_operation's graded
service):
  supervisor (root, comm labgrade-sup): PR_SET_NO_NEW_PRIVS and a seccomp filter under which only AF_UNIX sockets can
      be created; trimmed capabilities; binds the socket in its own root-owned directory, forks the worker and
      restarts it (bounded), and on the worker's one-byte request freezes the submission folder (take_snapshot: a
      fixed source, a fixed target, no other input). It never reads episode.json and never accepts a connection.
  worker (comm labgrade-wkr): forked as root; reads episode.json and state.json, checks and imports the simulator,
      builds the episode and resets it (its first observation draws the four cameras, so the renderer and the render
      bundle are loaded here), then drops to the account `labgrade` (setgroups, setgid, setuid, checked irreversible;
      not dumpable; no core files) and serves.

Record (/var/lib/arb-online: owner labgrade, mode 0700, in the root-owned /var/lib): record.json (format
arb-online-record/1, rewritten atomically: a temp file, fsync, rename) at the start, at most once a second while
requests change the episode and within a second of the last one, and when the episode ends; requests.jsonl (one line
per request: method, the action, the step, the error if any; no images, the token redacted; it stops growing at
32 MB); state.json (the service's start, the one start, the freeze, the end); service.log and supervisor.log.
task.toml declares the directory an artifact; tests/verify.py scores record.json.

Wire: JSON lines {"id": n, "method": ..., "params": {...}} -> {"id": n, "result": {...}} or {"id": n, "error": code,
"detail": text}. Methods: status, start (params: an optional token), step {action}, observe, finish, close. A second
connection gets {"id": null, "error": "busy", "detail": ...} and is closed. ~/graded/client.py speaks it.

Dependencies: Python 3.12 standard library here; the worker imports the simulator (numpy, MuJoCo) from the venv.
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

SERVICE_VERSION = "lab-sample-graded/1"
RECORD_FORMAT = "arb-online-record/1"
MAX_LINE = 1 << 20                         # one request line (an action is at most 15 numbers)
CONTROL_DT = 0.1                           # s of simulated time per control step (10 Hz)
EPISODE_STEPS = 20000                      # episode.EPISODE_STEPS, for a worker that holds no bench (restarted)
SIM_FILES = ("box_transport_env.py", "episode.py", "lab_env.py", "lab_look.json", "lab_render.py", "lab_sim.py")
REQUIRED = ("task", "user", "agent_user", "socket", "record_dir", "prep_s", "graded_s", "episode_file", "sim_dir",
            "sim_sha256", "env", "practice_flag", "submission_dir")
DEFAULTS = {"max_restarts": 3, "start_timeout_s": 600.0, "record_interval_s": 1.0, "requests_log_cap_bytes": 32 << 20,
            "record_max_bytes": 1000000, "reply_timeout_s": 120.0, "idle_s": 600.0, "max_log_bytes": 16 << 20,
            "note": "", "snapshot_name": "submission_snapshot", "snapshot_max_files": 5000,
            "snapshot_max_bytes": 200 << 20, "snapshot_max_depth": 64, "snapshot_max_dirs": 20000,
            "snapshot_timeout_s": 600.0, "snapshot_attempts": 3, "snapshot_retry_delay_s": 5.0}
ERRORS = {
    "busy": "another connection holds the graded episode: end that script (or close its connection), then connect again",
    "bad_request": "one JSON object per line, with a method",
    "bad_params": "params must be an object (start: an optional token; step: an action)",
    "unknown_method": "methods: status, start, step, observe, finish, close",
    "not_started": "the graded episode has not started: start() starts it, once per session",
    "already_started": "the graded episode was started already (once per session): observe() returns where it stands",
    "prep_window_closed": "the graded episode can no longer be started: its start window has closed",
    "episode_over": "the graded episode has ended: nothing more is simulated or scored",
    "malformed_action": "8, 15 or 9 finite numbers, the camera bits an integer 0-15: nothing was simulated",
    "snapshot_failed": "the submission folder could not be frozen, so the graded episode did not start (nothing was "
                       "charged)",
    "internal": "internal error of the graded episode's service",
}
END_REASONS = ("done", "finish", "graded_window", "worker_restart", "render_error")
METHODS = ("status", "start", "step", "observe", "finish", "close")

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
    for key in ("episode_file", "sim_dir"):
        cfg[key] = os.path.normpath(os.path.join(base, cfg[key]))
    for key in ("socket", "record_dir", "practice_flag", "submission_dir"):
        if not os.path.isabs(cfg[key]) or os.path.normpath(cfg[key]) != cfg[key]:
            raise ValueError("%s must be a normalized absolute path" % key)
    if os.path.dirname(os.path.dirname(cfg["practice_flag"])) != os.path.dirname(cfg["socket"]):
        raise ValueError("the commissioning flag's directory must sit in the socket's root-owned directory")
    if os.path.dirname(cfg["socket"]) in ("/", "/tmp", "/var/tmp", "/dev/shm", "/run"):
        raise ValueError("the socket must live in its own root-owned directory")
    if os.path.dirname(cfg["record_dir"].rstrip("/")) in ("/", "/tmp", "/var/tmp", "/dev/shm"):
        raise ValueError("the record directory must live in a root-owned parent")
    for key in ("prep_s", "graded_s", "idle_s", "reply_timeout_s", "record_interval_s", "start_timeout_s",
                "snapshot_timeout_s", "snapshot_retry_delay_s"):
        cfg[key] = float(cfg[key])
        if not (math.isfinite(cfg[key]) and cfg[key] > 0):
            raise ValueError("%s must be a positive number" % key)
    for key in ("snapshot_max_files", "snapshot_max_bytes", "snapshot_max_depth", "snapshot_max_dirs",
                "snapshot_attempts", "max_restarts"):
        if isinstance(cfg[key], bool) or not isinstance(cfg[key], int) or cfg[key] <= 0:
            raise ValueError("%s must be a positive integer" % key)
    if "/" in cfg["snapshot_name"] or cfg["snapshot_name"] in ("", ".", ".."):
        raise ValueError("snapshot_name is one folder name")
    if sorted(cfg["sim_sha256"]) != sorted(SIM_FILES):
        raise ValueError("sim_sha256 must pin exactly %s" % (SIM_FILES,))
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


def _accounts(cfg):
    """(the service's account, the agent's uid). The service's account is neither root nor the agent's."""
    acct = pwd.getpwnam(cfg["user"])
    if acct.pw_uid == 0:
        raise RuntimeError("the service account must not be root")
    agent = pwd.getpwnam(cfg["agent_user"])
    if agent.pw_uid == acct.pw_uid or agent.pw_uid == 0:
        raise RuntimeError("the service account must differ from the agent's, and the agent must not be root")
    return acct, agent.pw_uid


# ---------------------------------------------------------------------------------------------- the freeze (supervisor, root)
class SnapshotTooLarge(Exception):
    """The submission folder exceeds a cap of the freeze (entries, bytes, folders or depth)."""


def snapshot_digest(listing):
    """sha256 over the lines "<path>\\0<size>\\0<sha256 of content>\\n" of the copied files in sorted path order: the
    record's snapshot_sha256, recomputed by the verifier (tests/verify.py)."""
    h = hashlib.sha256()
    for rel, size, content_sha in sorted(listing):
        h.update(("%s\0%d\0%s\n" % (rel, size, content_sha)).encode("utf-8", "surrogateescape"))
    return h.hexdigest()


def take_snapshot(cfg, agent_uid):
    """DM3, run by the root supervisor on the worker's request: the agent's submission folder as it is now, copied
    into the root-only folder record_dir/snapshot_name. No input but the request: a fixed source and a fixed target,
    so the worker cannot have root copy anything else.

    The walk goes by file descriptor: every folder and file is opened relative to its parent's descriptor with
    O_NOFOLLOW (a folder also with O_DIRECTORY, a file with O_NONBLOCK), so a link put or swapped in anywhere is never
    followed; only regular files and folders owned by the agent's account are copied (links, other file types and
    anything another account owns are skipped and counted). The caps are the offline grader's (stage_submission in
    tests/verify.py): every entry that is not a folder counts as a file and the bytes of every entry that is not a link
    count; over a cap nothing is kept (status too_large), as the offline grader then grades nothing. Folders and depth
    are bounded too (the engine's artifact limits). The copy is built beside the target and renamed into place, then
    synced. A missing folder (or a link, or a folder of another account, in its place) freezes as missing: the
    offline grader would find no usable submission. Returns {status: ok | too_large | missing, files, bytes, skipped,
    sha256}; raises OSError when the copy fails."""
    source, dest_dir = cfg["submission_dir"], cfg["record_dir"]
    final, tmp = cfg["snapshot_name"], cfg["snapshot_name"] + ".partial"
    caps = {"files": int(cfg["snapshot_max_files"]), "bytes": int(cfg["snapshot_max_bytes"]),
            "dirs": int(cfg["snapshot_max_dirs"]), "depth": int(cfg["snapshot_max_depth"])}
    parent, name = os.path.split(source)
    dfd = os.open(dest_dir, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        for old in (tmp, final):
            if os.path.lexists(os.path.join(dest_dir, old)):
                info = os.stat(old, dir_fd=dfd, follow_symlinks=False)
                if stat.S_ISDIR(info.st_mode):
                    shutil.rmtree(old, dir_fd=dfd)
                else:
                    os.unlink(old, dir_fd=dfd)
        missing = {"status": "missing", "files": 0, "bytes": 0, "skipped": 0, "sha256": None}
        try:
            pfd = os.open(parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
        except OSError:
            return missing
        try:
            try:
                sfd = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=pfd)
            except OSError:
                return missing
        finally:
            os.close(pfd)
        count = {"files": 0, "bytes": 0, "skipped": 0, "dirs": 0}
        listing = []
        try:
            if os.fstat(sfd).st_uid != agent_uid:
                return missing
            os.mkdir(tmp, 0o700, dir_fd=dfd)
            tfd = os.open(tmp, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=dfd)
            try:
                _snapshot_tree(sfd, tfd, "", 1, agent_uid, caps, count, listing)
            finally:
                os.close(tfd)
        except SnapshotTooLarge as exc:
            shutil.rmtree(tmp, dir_fd=dfd)
            return {"status": "too_large", "reason": str(exc), "files": count["files"], "bytes": count["bytes"],
                    "skipped": count["skipped"], "sha256": None}
        finally:
            os.close(sfd)
        os.rename(tmp, final, src_dir_fd=dfd, dst_dir_fd=dfd)
        os.sync()
        return {"status": "ok", "files": len(listing), "bytes": sum(x[1] for x in listing),
                "skipped": count["skipped"], "sha256": snapshot_digest(listing)}
    finally:
        os.close(dfd)


def _snapshot_tree(sfd, tfd, rel, depth, agent_uid, caps, count, listing):
    if depth > caps["depth"]:
        raise SnapshotTooLarge("depth")
    with os.scandir(sfd) as it:
        names = sorted(entry.name for entry in it)
    for name in names:
        try:
            st = os.stat(name, dir_fd=sfd, follow_symlinks=False)
        except OSError:
            count["skipped"] += 1
            continue
        sub = rel + "/" + name if rel else name
        if stat.S_ISDIR(st.st_mode):
            count["dirs"] += 1
            if count["dirs"] > caps["dirs"]:
                raise SnapshotTooLarge("folders")
            if st.st_uid != agent_uid:
                count["skipped"] += 1
                continue
            try:
                cfd = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=sfd)
            except OSError:
                count["skipped"] += 1
                continue
            try:
                if os.fstat(cfd).st_uid != agent_uid:
                    count["skipped"] += 1
                    continue
                os.mkdir(name, 0o700, dir_fd=tfd)
                dfd = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=tfd)
                try:
                    _snapshot_tree(cfd, dfd, sub, depth + 1, agent_uid, caps, count, listing)
                finally:
                    os.close(dfd)
            finally:
                os.close(cfd)
            continue
        count["files"] += 1                           # the offline grader counts every entry that is not a folder ...
        if not stat.S_ISLNK(st.st_mode):
            count["bytes"] += st.st_size              # ... and the bytes of every entry that is not a link
        if count["files"] > caps["files"]:
            raise SnapshotTooLarge("files")
        if count["bytes"] > caps["bytes"]:
            raise SnapshotTooLarge("bytes")
        if not stat.S_ISREG(st.st_mode) or st.st_uid != agent_uid:
            count["skipped"] += 1
            continue
        _snapshot_file(sfd, tfd, name, sub, agent_uid, caps["bytes"], count, listing)


def _snapshot_file(sfd, tfd, name, rel, agent_uid, max_bytes, count, listing):
    try:
        ffd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC, dir_fd=sfd)
    except OSError:
        count["skipped"] += 1
        return
    try:
        st = os.fstat(ffd)
        if not stat.S_ISREG(st.st_mode) or st.st_uid != agent_uid:
            count["skipped"] += 1
            return
        mode = 0o700 if st.st_mode & 0o100 else 0o600
        out = os.open(name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, mode, dir_fd=tfd)
        digest, size = hashlib.sha256(), 0
        try:
            while True:
                block = os.read(ffd, 1 << 20)
                if not block:
                    break
                size += len(block)
                if size > max_bytes:                  # a file that grows while it is copied
                    raise SnapshotTooLarge("bytes")
                digest.update(block)
                view = memoryview(block)
                while view:
                    view = view[os.write(out, view):]
            os.fchmod(out, mode)
        finally:
            os.close(out)
        listing.append((rel, size, digest.hexdigest()))
    finally:
        os.close(ffd)


def _serve_worker_requests(cfg, agent_uid, pid, qr, aw, log):
    """While the worker lives, answer its freeze requests (one byte b"S" -> one JSON line). Returns its exit status."""
    reading = True
    while True:
        try:
            if reading:
                done, code = os.waitpid(pid, os.WNOHANG)
            else:
                done, code = os.waitpid(pid, 0)
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
            res = take_snapshot(cfg, agent_uid)
        except Exception as exc:  # noqa: BLE001 - a failed copy refuses the start (the worker's side)
            res = {"status": "failed", "reason": type(exc).__name__}
            log.write(kind="snapshot_failed", tb=traceback.format_exc(limit=8))
        res["wall_s"] = round(time.time() - t0, 3)
        log.write(kind="snapshot", **{k: res.get(k) for k in ("status", "reason", "files", "bytes", "skipped", "wall_s")})
        try:
            os.write(aw, (json.dumps(res) + "\n").encode())
        except OSError:
            pass


# ---------------------------------------------------------------------------------------------- the bench (worker, as root)
def read_episode(cfg):
    with open(cfg["episode_file"], "rb") as fh:
        ep = json.loads(fh.read(1 << 16))
    for k in ("episode", "unit", "unit_props", "seed", "seed_sha256", "oracle_token_sha256"):
        if k not in ep:
            raise ValueError("episode.json lacks %s" % k)
    if hashlib.sha256(str(int(ep["seed"])).encode()).hexdigest() != ep["seed_sha256"]:
        raise ValueError("episode.json: the seed does not match its digest")
    if not 0 <= int(ep["seed"]) < 2 ** 64:
        raise ValueError("episode.json: the seed is not a 64-bit number")
    return ep


def build_bench(cfg, ep, log):
    """Check and import the grader's simulator, build the graded episode as the offline grader builds one
    (verify.run_episodes: LabEnv on the unit's properties inside an episode.Episode with the seed) and reset it: its
    first observation draws the four cameras, which starts the renderer and reads the render bundle here, as root,
    before the privilege drop (the worker cannot read online/ or the bundle afterwards). Returns (modules, episode,
    first observation)."""
    for k, v in cfg["env"].items():
        os.environ[k] = v                             # before numpy and OpenBLAS load: the grader's numerics pins
    check_digests(cfg["sim_dir"], cfg["sim_sha256"], "the simulator")
    sys.path.insert(0, cfg["sim_dir"])
    try:
        E = importlib.import_module("lab_env")
        EP = importlib.import_module("episode")
        importlib.import_module("lab_render")         # lab_sim imports it at the first image; it reads its look here
    finally:
        while cfg["sim_dir"] in sys.path:
            sys.path.remove(cfg["sim_dir"])
    t0 = time.time()
    episode = EP.Episode(E.LabEnv(ep["unit_props"], ep["unit"], render=True), int(ep["seed"]))
    obs = episode.reset()
    for cam, (_bit, w, h, _fovy, _name) in sorted(E.CAMS.items()):
        img = obs[cam]
        if tuple(img.shape) != (h, w, 3) or float(img.std()) < 8.0 or not 20.0 <= float(img.mean()) <= 235.0:
            raise E.RenderError("the first observation's %s image is not sane: shape %s, mean %.1f, std %.1f" % (
                cam, tuple(img.shape), float(img.mean()), float(img.std())))
    log.write(kind="bench_built", wall_s=round(time.time() - t0, 3), unit_known=True)
    return (E, EP), episode, obs


def valid_action(action, all_cams):
    """The offline grader's check (tests/verify.py valid_action): 8 numbers (a Cartesian step), 15 (the same with a
    null-space posture) or 9 (a joint step), all finite; the camera bits (the 8th of 8 or 15, the 9th of 9) an
    integer 0-15."""
    if not isinstance(action, list) or len(action) not in (8, 9, 15):
        return False
    for x in action:
        if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
            return False
    cams = action[8] if len(action) == 9 else action[7]
    return 0 <= cams <= all_cams and float(cams).is_integer()


# ---------------------------------------------------------------------------------------------- the episode (worker)
class Client(object):
    def __init__(self, sock, cid, peer_uid):
        self.sock, self.cid, self.peer_uid, self.buf = sock, cid, peer_uid, b""
        self.last = time.monotonic()


class GradedEpisode(object):
    """The one graded episode, its windows and its record, served to one client at a time."""

    def __init__(self, cfg, ep, store, log, listener, mods, episode, obs, state, snap_pipe=None):
        self.cfg, self.ep, self.store, self.log, self.listener = cfg, ep, store, log, listener
        self.snap_pipe = snap_pipe
        self.E, self.EP = mods if mods else (None, None)
        self.episode, self.obs, self.state = episode, obs, state
        self.last_wire = None                         # the last observation sent (observe returns it again)
        self.client = None
        self.conn_seq = 0
        self.requests = 0
        self.refused = 0
        self.images = 0
        self.stepped = 0                              # the agent's steps that simulated (the observation's t starts
                                                      # where the bench's settling left it, as offline)
        self.dirty = False
        self.last_write = 0.0
        self.token_mismatch = False
        self.done_rule = None
        self.render_error = None
        self.req_log = None
        self.req_log_size = 0
        self.req_log_full = False
        self.freeze_attempts = 0
        self.freeze_retry_at = 0.0
        self.last = {}                                # what the last record said (a restarted worker has no bench)

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

    @property
    def privileged(self):
        return bool(self.state.get("privileged"))

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
            try:
                self.close_practice()                 # a worker that died before writing it: commissioning stays closed
            except OSError:
                self.log.write(kind="practice_flag_failed", tb=traceback.format_exc(limit=4))
            rec = self.store.read_json("record.json") or {}
            self.last = {k: rec.get(k) for k in ("steps", "t", "sim_time_s", "requests")}
        self.store.write_json("state.json", self.state)
        try:
            fd = os.open(self.store.path("requests.jsonl"), os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
            self.req_log_size = os.fstat(fd).st_size
            self.req_log = fd
        except OSError as e:
            self.log.write(kind="requests_log_unavailable", err=str(e))

    def close_practice(self):
        """O13: the flag that closes the commissioning service (it stats it). Written once, atomically; raises
        OSError when it cannot be written."""
        path = self.cfg["practice_flag"]
        if os.path.lexists(path):
            return
        data = json.dumps({"graded_episode_started": True,
                           "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}).encode() + b"\n"
        tmp = path + ".tmp"
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o644)
        try:
            os.fchmod(fd, 0o644)
            os.write(fd, data)
            os.fsync(fd)
        finally:
            os.close(fd)
        os.replace(tmp, path)

    def request_snapshot(self):
        """DM3: ask the root supervisor to freeze the submission folder now. Its result (a dict), or a failure."""
        failed = {"status": "failed", "reason": "no answer from the supervisor"}
        if not self.snap_pipe:
            return failed
        qw, ar = self.snap_pipe
        while select.select([ar], [], [], 0)[0]:     # a late answer to an earlier request that timed out
            if not os.read(ar, 65536):
                return failed
        try:
            os.write(qw, b"S")
        except OSError:
            return failed
        deadline, buf = time.time() + float(self.cfg["snapshot_timeout_s"]), b""
        while not buf.endswith(b"\n"):
            left = deadline - time.time()
            if left <= 0 or not select.select([ar], [], [], left)[0]:
                return failed
            chunk = os.read(ar, 65536)
            if not chunk:
                return failed
            buf += chunk
        try:
            res = json.loads(buf.decode("utf-8"))
        except ValueError:
            return failed
        return res if isinstance(res, dict) else failed

    @staticmethod
    def _snap(snap, trigger, now, attempts):
        out = {k: snap.get(k) for k in ("status", "reason", "files", "bytes", "sha256", "skipped", "wall_s")}
        out.update({"trigger": trigger, "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)),
                    "attempts": attempts})
        return out

    def snapshot_fields(self):
        sn = self.state.get("snapshot") or {}
        return {"freeze": self.state.get("frozen"), "snapshot_trigger": sn.get("trigger"),
                "snapshot_utc": sn.get("utc"), "snapshot_status": sn.get("status"),
                "snapshot_files": sn.get("files"), "snapshot_bytes": sn.get("bytes"),
                "snapshot_sha256": sn.get("sha256"), "snapshot_skipped": sn.get("skipped"),
                "snapshot_attempts": sn.get("attempts")}

    def freeze_at_window_close(self, now):
        """The start window closed with no start: the program freezes now, as it would have at a start (the same copy
        by the root supervisor), and a record says so. A copy that fails is tried again `snapshot_attempts` times
        (`snapshot_retry_delay_s` apart); one that still fails is recorded as failed (the program item then scores 0:
        the session-end submission never stands in for it)."""
        snap = self.request_snapshot()
        self.freeze_attempts += 1
        if snap.get("status") not in ("ok", "too_large", "missing") and \
                self.freeze_attempts < int(self.cfg["snapshot_attempts"]):
            self.log.write(kind="window_close_freeze_retry", attempt=self.freeze_attempts, result=snap)
            self.freeze_retry_at = time.time() + float(self.cfg["snapshot_retry_delay_s"])
            return
        if snap.get("status") not in ("ok", "too_large", "missing"):
            snap = dict(snap, status="failed")
        self.state.update({"frozen": "prep_window_closed", "frozen_wall": now,
                           "snapshot": self._snap(snap, "prep_window", now, self.freeze_attempts)})
        try:
            self.store.write_json("state.json", self.state)
            rec = {"format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.ep["episode"],
                   "seed_sha256": self.ep["seed_sha256"], "started": False, "ended": False, "end_reason": None,
                   "requests": 0, "steps": 0, "sim_time_s": 0.0, "wall_s_since_start": None, "episode_score": 0.0,
                   "metrics": {}, "privileged": False, "service_version": SERVICE_VERSION,
                   "written_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)), **self.snapshot_fields()}
            self.store.write_json("record.json", rec)
        except OSError:
            self.log.write(kind="window_close_record_failed", tb=traceback.format_exc(limit=4))
        self.log.write(kind="frozen_at_window_close", attempts=self.freeze_attempts,
                       **{k: snap.get(k) for k in ("status", "files", "bytes", "skipped")})
        if self.episode is not None:                  # the episode can no longer start: release the bench
            try:
                self.episode.env.close()
            except Exception:  # noqa: BLE001
                pass
            self.episode = None

    def _minimal_record(self, now):
        start = float(self.state.get("start_wall") or now)
        return {"format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.ep["episode"],
                "seed_sha256": self.ep["seed_sha256"], "started": True, "ended": False, "end_reason": None, "requests": 0,
                "steps": 0, "sim_time_s": 0.0, "wall_s_since_start": now - start, "episode_score": 0.0, "metrics": {},
                "privileged": self.privileged, "service_version": SERVICE_VERSION, **self.snapshot_fields()}

    # -- the record
    def _score(self):
        """episode.Episode.score() as if the episode ended now, and the offline grader's per-episode parts
        (verify.run_episodes' per_part)."""
        sc = self.episode.score()
        parts = {"part_cold": sum(sc["cold"].values()) / len(sc["cold"]) if sc["cold"] else 0.0,
                 "part_pairs": min(sum(sc["pairs"]), sc["need"]) / sc["need"] if sc["need"] else 0.0,
                 "part_lid": float(sc["lid"]), "part_run": float(sc["run"]),
                 "part_kept": 0.0 if sc["moved"] else 1.0}
        return sc, parts

    def _record(self, now):
        sc, parts = self._score()
        t = int(self.episode.env.t)
        m = dict(parts, credit=float(sc["credit"]), startable=float(sc["startable"]), unstable=float(sc["unstable"]),
                 sim_faults=1.0 if sc["unstable"] else 0.0, steps=float(self.stepped), t=float(t),
                 need=float(sc["need"]),
                 lid_angle=float(self.episode.env.p.lid_angle()), images_requested=float(self.images),
                 malformed_actions_refused=float(self.refused), privileged_mode=1.0 if self.privileged else 0.0,
                 oracle_token_mismatch=1.0 if self.token_mismatch else 0.0,
                 render_error=1.0 if self.render_error else 0.0)
        detail = {"cold": sc["cold"], "pairs": sc["pairs"], "need": sc["need"], "lid": sc["lid"],
                  "startable": sc["startable"], "run": sc["run"], "moved": sc["moved"][:20],
                  "in_hole": sc["in_hole"], "unstable": sc["unstable"], "done_rule": self.done_rule,
                  "render_error": self.render_error}
        return {"format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.ep["episode"],
                "seed_sha256": self.ep["seed_sha256"], "unit": self.ep["unit"], "started": True, "ended": self.ended,
                "end_reason": self.end_reason, "requests": int(self.requests), "steps": int(self.stepped), "t": t,
                "sim_time_s": round(t * CONTROL_DT, 6), "wall_s_since_start": now - float(self.state["start_wall"]),
                "started_after_s": float(self.state["start_wall"]) - float(self.state["service_started"]),
                "episode_score": float(sc["credit"]), "metrics": m, "privileged": self.privileged,
                "render_error": bool(self.render_error), "service_version": SERVICE_VERSION, **self.snapshot_fields(),
                "written_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)), "detail": detail}

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
        if self.episode is not None and self.started and (self.dirty or force) and \
                (force or now - self.last_write >= float(self.cfg["record_interval_s"])):
            self.write_record(now)

    def _log_request(self, now, method, params, res):
        if self.req_log is None or self.req_log_full:
            return
        if method == "start":
            p = json.dumps({"token": "<given>" if isinstance(params, dict) and params.get("token") else None})
        else:
            try:
                p = json.dumps(params, default=str)[:2000]
            except (TypeError, ValueError):
                p = "<unprintable>"
        err = res.get("error") if isinstance(res, dict) and set(res) <= {"error", "detail"} else None
        t = int(self.episode.env.t) if self.episode is not None else None
        line = json.dumps({"n": self.requests, "wall": round(now - float(self.state.get("start_wall") or now), 3),
                           "method": str(method)[:64], "params": p, "t": t, "error": err}, default=str) + "\n"
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
        """End the episode: marked over, scored as it stands (nothing is simulated afterwards)."""
        if self.ended or not self.started:
            return
        self.state.update({"ended": True, "end_reason": reason, "ended_wall": now})
        try:
            self.store.write_json("state.json", self.state)
        except OSError:
            self.log.write(kind="state_write_failed", tb=traceback.format_exc(limit=4))
        self.flush(now, force=True)
        self.log.write(kind="episode_end", reason=reason, done_rule=self.done_rule, requests=self.requests,
                       steps=self.stepped, step_t=int(self.episode.env.t) if self.episode is not None else None)

    def windows(self, now):
        if self.started and not self.ended and now >= self.graded_deadline():
            self.end("graded_window", now)
        if not self.started and not self.state.get("frozen") and not self.state.get("state_lost") and \
                now >= self.prep_deadline() and now >= self.freeze_retry_at:
            self.freeze_at_window_close(now)

    # -- the wire
    def wire(self, obs):
        w = self.E.to_wire(obs)
        w["unit"] = self.ep["unit"]
        if self.privileged:
            w["truth"] = self.episode.env.truth()
        return w

    def ended_text(self):
        rule = self.done_rule
        if rule == "unstable":
            return "unstable simulation (an arm joint faster than 8 rad/s: the episode scores 0)"
        if rule == "lid":
            return "the lid has been shut for {} steps".format(self.EP.LID_HOLD)
        if rule == "steps":
            return "all {} steps run".format(self.EP.EPISODE_STEPS)
        return {"finish": "finished", "graded_window": "the graded window ran out",
                "worker_restart": "the service restarted", "render_error": "the cameras' renderer failed"}.get(
                    self.end_reason)

    def steps_left(self):
        return max(0, int(self.EP.EPISODE_STEPS) - int(self.episode.env.t)) if self.episode is not None else 0

    # -- requests
    def status(self, now):
        if self.episode is not None:
            steps, t = self.stepped, int(self.episode.env.t)
        else:
            steps, t = int(self.last.get("steps") or 0), int(self.last.get("t") or 0)
        if not self.started:
            prep_left = max(0.0, self.prep_deadline() - now)
            graded_left = float(self.cfg["graded_s"]) if prep_left > 0 else 0.0
        else:
            prep_left = 0.0
            graded_left = 0.0 if self.ended else max(0.0, self.graded_deadline() - now)
        limit = int(self.EP.EPISODE_STEPS) if self.EP is not None else EPISODE_STEPS
        return {"started": self.started, "ended": self.ended, "end": self.end_reason,
                "ended_by": self.ended_text() if self.ended else None, "steps": steps, "t": t,
                "sim_time_s": round(t * CONTROL_DT, 3), "steps_left": 0 if self.ended else max(0, limit - t),
                "prep_window_left_s": round(prep_left, 1), "graded_window_left_s": round(graded_left, 1),
                "program_frozen": bool(self.state.get("frozen"))}

    def start(self, params, now):
        if not isinstance(params, dict):
            return {"error": "bad_params", "detail": ERRORS["bad_params"]}
        if self.started:
            return {"error": "already_started", "detail": ERRORS["already_started"]}
        if now > self.prep_deadline() or self.state.get("frozen") or self.episode is None:
            return {"error": "prep_window_closed", "detail": ERRORS["prep_window_closed"]}
        token = params.get("token")
        privileged = False
        if token is not None:
            got = hashlib.sha256(str(token).strip().encode("utf-8")).hexdigest()
            privileged = got == self.ep["oracle_token_sha256"]
            self.token_mismatch = not privileged
        # DM3: the program is frozen before anything is charged; a copy that fails refuses the start
        snap = self.request_snapshot()
        if snap.get("status") not in ("ok", "too_large", "missing"):
            self.log.write(kind="start_refused_snapshot", result=snap)
            return {"error": "snapshot_failed", "detail": ERRORS["snapshot_failed"]}
        now = time.time()
        # O13: the commissioning service closes when the graded episode starts; no start without the flag
        try:
            self.close_practice()
        except OSError:
            self.log.write(kind="practice_flag_failed", tb=traceback.format_exc(limit=4))
            return {"error": "internal", "detail": ERRORS["internal"]}
        # the charge: persisted before anything is simulated, never refunded
        self.state.update({"started": True, "start_wall": now, "privileged": privileged, "frozen": "graded_start",
                           "frozen_wall": now, "snapshot": self._snap(snap, "start", now, 1)})
        try:
            self.store.write_json("state.json", self.state)
        except OSError:
            self.state.update({"started": False, "start_wall": None, "privileged": False, "snapshot": None,
                               "frozen": None, "frozen_wall": None})
            self.log.write(kind="start_not_persisted", tb=traceback.format_exc(limit=4))
            return {"error": "internal", "detail": ERRORS["internal"]}
        self.log.write(kind="episode_start", privileged=privileged, token_mismatch=self.token_mismatch,
                       peer_uid=self.client.peer_uid if self.client else None, snapshot=snap.get("status"),
                       snapshot_files=snap.get("files"))
        self.write_record(now)
        self.last_wire = self.wire(self.obs)
        return {"started": True, "obs": self.last_wire, "steps_left": self.steps_left(),
                "graded_window_s": float(self.cfg["graded_s"])}

    def step(self, params, now):
        action = params.get("action")
        if not valid_action(action, self.E.ALL_CAMS):
            self.refused += 1
            return {"error": "malformed_action", "detail": ERRORS["malformed_action"]}
        try:
            obs, done, _info = self.episode.step(action)
        except self.E.RenderError as exc:
            self.render_error = str(exc)[:300]
            self.log.write(kind="render_error", err=self.render_error)
            self.end("render_error", time.time())
            return {"error": "episode_over", "detail": "%s (the cameras' renderer failed)" % ERRORS["episode_over"]}
        self.images += bin(int(action[8] if len(action) == 9 else action[7])).count("1")
        self.stepped += 1
        self.obs = obs
        self.dirty = True
        if done:
            if self.episode.env.unstable():
                self.done_rule = "unstable"
            elif self.episode.lid_steps >= self.EP.LID_HOLD:
                self.done_rule = "lid"
            else:
                self.done_rule = "steps"
        self.last_wire = self.wire(obs)
        if done:
            self.end("done", time.time())
        return {"obs": self.last_wire, "done": bool(done), "ended": self.ended_text() if done else None,
                "steps_left": self.steps_left()}

    def dispatch(self, method, params, now):
        if method not in METHODS:
            return {"error": "unknown_method", "detail": ERRORS["unknown_method"]}
        if params is not None and not isinstance(params, dict):
            return {"error": "bad_params", "detail": ERRORS["bad_params"]}
        params = params or {}
        if method == "status":
            return self.status(now)
        if method == "start":
            res = self.start(params, now)
            self._log_request(now, "start", params, res)
            return res
        if not self.started:
            return {"error": "not_started", "detail": ERRORS["not_started"]}
        if self.ended or self.episode is None:
            if method == "finish":
                return {"ended": True, "end": self.end_reason, "ended_by": self.ended_text()}
            return {"error": "episode_over", "detail": "%s (%s)" % (ERRORS["episode_over"], self.ended_text())}
        self.requests += 1
        if method == "observe":
            res = {"obs": self.last_wire, "done": False, "ended": None, "steps_left": self.steps_left()}
        elif method == "finish":
            self.end("finish", now)
            res = {"ended": True, "end": "finish", "ended_by": self.ended_text()}
        else:
            res = self.step(params, now)
        self._log_request(now, method, params, res)
        return res

    def _send(self, obj):
        if self.client is None:
            return
        try:
            data = (json.dumps(obj, default=_jsonable, allow_nan=False) + "\n").encode("utf-8")
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
        except (ValueError, UnicodeDecodeError):
            self._send({"id": None, "error": "bad_request", "detail": ERRORS["bad_request"]})
            return
        rid, method = req.get("id"), str(req.get("method"))
        if method == "close":
            self._send({"id": rid, "result": {"closed": True}})
            self.drop("close")
            return
        try:
            res = self.dispatch(method, req.get("params"), now)
        except Exception:  # noqa: BLE001 - the service's own failure: the episode stays where it is
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
        c.last = time.monotonic()
        c.buf += chunk
        while self.client is c:
            nl = c.buf.find(b"\n")
            if nl < 0:
                if len(c.buf) > MAX_LINE:
                    self._send({"id": None, "error": "bad_request", "detail": "a request line is longer than 1 MB"})
                    self.drop("line too long")
                break
            line, c.buf = c.buf[:nl], c.buf[nl + 1:]
            if line.strip():
                self.request(line)
                if self.client is c:
                    c.last = time.monotonic()

    def serve(self):
        self.listener.setblocking(False)
        while True:
            now = time.time()
            self.windows(now)
            wait = 60.0
            if self.started and not self.ended:
                wait = min(wait, self.graded_deadline() - now)
            if not self.started and not self.state.get("frozen"):
                wait = min(wait, max(self.prep_deadline(), self.freeze_retry_at) - now)
            if self.dirty:
                wait = min(wait, self.last_write + float(self.cfg["record_interval_s"]) - now)
            if self.client is not None:
                idle_left = self.client.last + float(self.cfg["idle_s"]) - time.monotonic()
                if idle_left <= 0:
                    self.drop("idle for %d s" % int(self.cfg["idle_s"]))
                else:
                    wait = min(wait, idle_left)
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
    """Forked from the supervisor as root: read the episode, build its bench, drop privileges, serve."""
    _set_comm("labgrade-wkr")
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
        mods, episode, obs = None, None, None
        fresh = not state.get("started") and not state.get("ended") and not state.get("state_lost") \
            and not state.get("frozen")
        if fresh:
            mods, episode, obs = build_bench(cfg, ep, log)
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
        graded = GradedEpisode(cfg, ep, store, log, listener, mods, episode, obs, state, snap_pipe)
        graded.begin()
        log.write(kind="worker_ready", uid=os.getuid(), gid=os.getgid(), seccomp_inet_blocked=sec_ok, fresh=fresh,
                  worker_starts=state.get("worker_starts"), service_version=SERVICE_VERSION)
    except Exception:  # noqa: BLE001
        log.write(kind="worker_start_failed", tb=traceback.format_exc(limit=30))
        os.write(ready_fd, b"F")
        os._exit(1)
    os.write(ready_fd, b"R")
    os.close(ready_fd)
    try:
        graded.serve()
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


def run_supervisor(cfg, acct, agent_uid, notify_fd):
    """Root, never reads episode.json, never accepts a connection: binds the socket, forks and restarts the worker,
    and freezes the submission folder on the worker's request."""
    _set_comm("labgrade-sup")
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
    os.chmod(sock_path, 0o666)                       # before the chown: without CAP_FOWNER root cannot chmod it after
    os.chown(sock_path, acct.pw_uid, acct.pw_gid)
    listener.listen(16)
    pidfile = os.path.join(os.path.dirname(sock_path), "supervisor.pid")
    with open(pidfile + ".tmp", "w") as fh:
        fh.write("%d\n" % os.getpid())
    os.chmod(pidfile + ".tmp", 0o644)
    os.replace(pidfile + ".tmp", pidfile)
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
        code = _serve_worker_requests(cfg, agent_uid, pid, qr, aw, sup_log)
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
        os.unlink(pidfile)
    except OSError:
        pass
    sup_log.write(kind="supervisor_exit", restarts=restarts)
    os._exit(0)


def _init_state(cfg, acct):
    """state.json at the service's first start: when it started (the start window counts from here), no start yet."""
    store = Store(cfg["record_dir"])
    if os.path.lexists(store.path("state.json")):
        return
    if os.path.lexists(cfg["practice_flag"]):
        os.unlink(cfg["practice_flag"])              # a fresh service: nothing has started, commissioning is open
    store.write_json("state.json", {"version": 1, "service_started": time.time(), "started": False, "ended": False,
                                    "start_wall": None, "end_reason": None, "worker_starts": 0,
                                    "prep_s": cfg["prep_s"], "graded_s": cfg["graded_s"]})
    os.chown(store.path("state.json"), acct.pw_uid, acct.pw_gid)


def start(config_path):
    if os.geteuid() != 0:
        print("graded service: must be started as root (environment/setup/setup.sh)", file=sys.stderr)
        return 2
    try:
        cfg = load_config(config_path)
        acct, agent_uid = _accounts(cfg)
        os.umask(0o077)
        sockdir = os.path.dirname(cfg["socket"])
        os.makedirs(os.path.dirname(sockdir), exist_ok=True)
        _secure_dir(sockdir, 0o755, 0, 0)
        # O13: the flag's directory, writable by the service's account only, readable by everyone
        _secure_dir(os.path.dirname(cfg["practice_flag"]), 0o755, acct.pw_uid, acct.pw_gid)
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
        try:
            run_supervisor(cfg, acct, agent_uid, w)
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
