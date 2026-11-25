#!/usr/bin/env python3
"""graded_service: the graded episode of longtail_manipulation_recovery (dual mode, experimental).

Started once by environment/setup/setup.sh as root, cwd = the setup stage (/opt/ale/setup, root-only while the agent
works), before the agent's session:

    env -i PATH=/opt/venv/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \\
        /opt/venv/bin/python3 -I -B online/graded_service.py --config online/service.json

It returns once the service serves (exit 0) or failed to start (exit 1); the service itself is a detached daemon
(setsid, stdio on /dev/null) that runs through the agent's session.

The episode. ONE failure state of the offline grader's fixed hidden set (online/episode.json, read by the worker only:
the state's entry of tests/hidden/states.json, drawn once), restored and run exactly as the offline grader runs one:
by the grader's own kitchen process, sim_worker.py (byte copies of tests/sim_worker.py, kitchen_sim.py and
wire_rules.py in code_dir), on the state's own files (byte copies of tests/hidden/<id>_prefix.npz, _state.npz and
_card.json in state_dir), in RoboCasa's held-out kitchens: the image's second RoboCasa tree (robocasa_dir: the same
pinned commit, the graded kitchens' assets installed as tests/Dockerfile installs them for the grader), which the
kitchen process imports instead of the agent's practice tree. The worker checks every copied file against
service.json's digests and refuses to start on a difference. The kitchen is rebuilt (the recorded action prefix
replayed, the recorded physical state set, checked against the recording to prefix_tol as the grader checks it) when
the agent starts the episode, not before: a kitchen holds about 6 GB, which the agent's own practice kitchen needs
while it prepares. The episode has the offline episode's rules: the same observation (the card at t = 0, the first
observation's three RGB-D views, images on request, the frame cap), the same action check (wire_rules.check_action:
the 12 controls clipped, the camera mask), the same step budget and the same success rule (the job's own check holding
for wire_rules.SUCCESS_RUN consecutive control steps). What does not carry over is what binds a controller process:
the per-request deadline, the per-episode controller-time cap, the restarts, the pauses, the line and queue caps and
the process rules. A malformed action is refused (`malformed_action`, nothing simulated) instead of ending the
episode. The simulation advances only inside the agent's own `step` requests.

Rules:
  * `start` is accepted once per session, within `prep_s` of the service's start (state.json `service_started`). It
    first builds the episode's kitchen; a kitchen that cannot be built (its process killed for want of memory, say)
    refuses the start and charges nothing. Then it freezes the program (below) and is persisted (fsync) before
    anything is simulated; it is never refunded. With the reference's token (its sha256 is in episode.json) every
    observation carries `_gt` (the state's id), as the offline grader gives it to a submission that ships the token;
    a wrong token is ignored and counted. A replayed prefix that does not reproduce the recording is an error of the
    environment: the record says so (sim_error) and the verifier fails as a task error, as the offline grader does.
  * The episode lives in the worker and its kitchen process, not in a connection: a disconnect leaves it where it is
    and a new connection resumes it (`observe` returns the last observation again). One client at a time: a second
    connection is answered `busy` and closed; a connection silent for `idle_s` is closed (the episode stays).
  * It ends at its own end (the job's success check has held for SUCCESS_RUN consecutive steps: recovered; or the step
    budget is used: not recovered; `done`), at the agent's `finish`, `graded_s` after its start (`graded_window`), or
    when the session ends (the service is frozen or killed: the last record stands). An episode ended before its own
    end is scored as it stands, with nothing simulated afterwards: not recovered, since a recovered episode has ended
    by itself. The kitchen process is closed at the end. An ended episode answers `status` and `finish` only.
  * A kitchen process that fails during the episode ends it (end_reason sim_failed); the record says so (sim_error)
    and the verifier fails as a task error, as the offline grader fails on a simulator failure.
  * When the worker dies or restarts, the episode ends with its last record (end_reason worker_restart); its kitchen
    process ends with it (its stdin closes).
  * The agent learns the observations (the offline wire's: no success flag) and `status` (started / ended, steps, the
    windows' time left): never the record, the state's id or its seed. An episode that has ended before its step budget
    has recovered: a controller sees the same offline, when its episode is over early.
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
  * Practice: the agent's practice is its own (the practice kitchens, kitchen_sim.py and the practice grader run in
    its own account, on files it was given); no service of this task runs hidden code for it, so nothing closes at the
    start (O13 has nothing to end here).

Process layout (the alelab daemon pattern of arm_hidden_mechanism_box, as pump_skid_startup_and_operation's graded
service):
  supervisor (root, comm longgrade-sup): PR_SET_NO_NEW_PRIVS and a seccomp filter under which only AF_UNIX sockets can
      be created; trimmed capabilities; binds the socket in its own root-owned directory, forks the worker and
      restarts it (bounded), and on the worker's one-byte request freezes the submission folder (take_snapshot: a
      fixed source, a fixed target, no other input). It never reads episode.json and never accepts a connection.
  worker (comm longgrade-wkr): forked as root; reads episode.json and state.json, checks the copied code and state
      files, imports wire_rules, then drops to the account `longgrade` (setgroups, setgid, setuid, checked
      irreversible; not dumpable; no core files) and serves. Its kitchen process (sim_worker.py) runs as that account
      too, under the same seccomp filter (inherited).

Record (/var/lib/arb-online: owner longgrade, mode 0700, in the root-owned /var/lib): record.json (format
arb-online-record/1, rewritten atomically: a temp file, fsync, rename) at the start, at most once a second while
requests change the episode and within a second of the last one, and when the episode ends; requests.jsonl (one line
per request: method, the action, the step, the error if any; no images, the token redacted; it stops growing at
32 MB); state.json (the service's start, the one start, the freeze, the end); service.log and supervisor.log.
task.toml declares the directory an artifact; tests/verify.py scores record.json.

Wire: JSON lines {"id": n, "method": ..., "params": {...}} -> {"id": n, "result": {...}} or {"id": n, "error": code,
"detail": text}. Methods: status, start (params: an optional token), step {action}, observe, finish, close. A second
connection gets {"id": null, "error": "busy", "detail": ...} and is closed. ~/graded/client.py speaks it.

Dependencies: Python 3.12 standard library here (and wire_rules, stdlib only); the kitchen process imports the
simulator (numpy, MuJoCo, robosuite, RoboCasa) from the venv and robocasa_dir.
"""

import argparse
import ctypes
import hashlib
import importlib.util
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
import time
import traceback

SERVICE_VERSION = "longtail-graded/1"
RECORD_FORMAT = "arb-online-record/1"
MAX_LINE = 1 << 20                         # one request line (an action is 12 or 13 numbers)
CONTROL_DT = 0.05                          # s of simulated time per control step (20 Hz)
CODE_FILES = ("kitchen_sim.py", "sim_worker.py", "wire_rules.py")
REQUIRED = ("task", "user", "agent_user", "socket", "record_dir", "prep_s", "graded_s", "episode_file", "code_dir",
            "code_sha256", "state_dir", "state_sha256", "robocasa_dir", "sim_home", "sim_env", "submission_dir")
DEFAULTS = {"max_restarts": 3, "start_timeout_s": 120.0, "build_timeout_s": 900.0, "sim_reply_timeout_s": 300.0,
            "prefix_tol": 1e-9, "record_interval_s": 1.0, "requests_log_cap_bytes": 32 << 20,
            "record_max_bytes": 1000000, "reply_timeout_s": 120.0, "idle_s": 600.0, "max_log_bytes": 16 << 20,
            "note": "", "snapshot_name": "submission_snapshot", "snapshot_max_files": 5000,
            "snapshot_max_bytes": 200 << 20, "snapshot_max_depth": 64, "snapshot_max_dirs": 20000,
            "snapshot_timeout_s": 600.0, "snapshot_attempts": 3, "snapshot_retry_delay_s": 5.0,
            "sim_log_cap_bytes": 8 << 20}
ERRORS = {
    "busy": "another connection holds the graded episode: end that script (or close its connection), then connect again",
    "bad_request": "one JSON object per line, with a method",
    "bad_params": "params must be an object (start: an optional token; step: an action)",
    "unknown_method": "methods: status, start, step, observe, finish, close",
    "not_started": "the graded episode has not started: start() starts it, once per session",
    "already_started": "the graded episode was started already (once per session): observe() returns where it stands",
    "prep_window_closed": "the graded episode can no longer be started: its start window has closed",
    "episode_over": "the graded episode has ended: nothing more is simulated or scored",
    "malformed_action": "a list of 12 numbers, or 13 with the camera mask (an integer 0-15): nothing was simulated",
    "kitchen_failed": "the graded episode's kitchen could not be built (its process ended: with too little free memory, "
                      "stop your own kitchen simulator first); the episode did not start and nothing was charged",
    "snapshot_failed": "the submission folder could not be frozen, so the graded episode did not start (nothing was "
                       "charged)",
    "internal": "internal error of the graded episode's service",
}
END_REASONS = ("done", "finish", "graded_window", "worker_restart", "sim_failed")
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
    cfg["episode_file"] = os.path.normpath(os.path.join(base, cfg["episode_file"]))
    for key in ("socket", "record_dir", "submission_dir", "code_dir", "state_dir", "robocasa_dir", "sim_home"):
        if not os.path.isabs(cfg[key]) or os.path.normpath(cfg[key]) != cfg[key]:
            raise ValueError("%s must be a normalized absolute path" % key)
    if os.path.dirname(cfg["socket"]) in ("/", "/tmp", "/var/tmp", "/dev/shm", "/run"):
        raise ValueError("the socket must live in its own root-owned directory")
    if os.path.dirname(cfg["record_dir"].rstrip("/")) in ("/", "/tmp", "/var/tmp", "/dev/shm"):
        raise ValueError("the record directory must live in a root-owned parent")
    for key in ("prep_s", "graded_s", "idle_s", "reply_timeout_s", "record_interval_s", "start_timeout_s",
                "build_timeout_s", "sim_reply_timeout_s", "snapshot_timeout_s", "snapshot_retry_delay_s",
                "prefix_tol"):
        cfg[key] = float(cfg[key])
        if not (math.isfinite(cfg[key]) and cfg[key] > 0):
            raise ValueError("%s must be a positive number" % key)
    for key in ("snapshot_max_files", "snapshot_max_bytes", "snapshot_max_depth", "snapshot_max_dirs",
                "snapshot_attempts", "max_restarts", "sim_log_cap_bytes"):
        if isinstance(cfg[key], bool) or not isinstance(cfg[key], int) or cfg[key] <= 0:
            raise ValueError("%s must be a positive integer" % key)
    if "/" in cfg["snapshot_name"] or cfg["snapshot_name"] in ("", ".", ".."):
        raise ValueError("snapshot_name is one folder name")
    if sorted(cfg["code_sha256"]) != sorted(CODE_FILES):
        raise ValueError("code_sha256 must pin exactly %s" % (CODE_FILES,))
    if not cfg["state_sha256"]:
        raise ValueError("state_sha256 must pin the state's files")
    if not all(isinstance(k, str) and isinstance(v, str) for k, v in cfg["sim_env"].items()):
        raise ValueError("sim_env maps strings to strings")
    if {"HOME", "PYTHONPATH"} & set(cfg["sim_env"]):
        raise ValueError("sim_env sets neither HOME nor PYTHONPATH (the service sets them)")
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


# ---------------------------------------------------------------------------------------------- the episode's files (worker, as root)
def read_episode(cfg):
    with open(cfg["episode_file"], "rb") as fh:
        ep = json.loads(fh.read(1 << 16))
    for k in ("episode", "state", "step_budget", "seed_sha256", "oracle_token_sha256"):
        if k not in ep:
            raise ValueError("episode.json lacks %s" % k)
    st = ep["state"]
    for k in ("id", "task", "seed", "split", "prefix", "qpos", "card"):
        if k not in st:
            raise ValueError("episode.json's state lacks %s" % k)
    if st["id"] != ep["episode"]:
        raise ValueError("episode.json: the episode is not its state's id")
    if hashlib.sha256(str(int(st["seed"])).encode()).hexdigest() != ep["seed_sha256"]:
        raise ValueError("episode.json: the seed does not match its digest")
    if sorted((st["prefix"], st["qpos"], st["card"])) != sorted(cfg["state_sha256"]):
        raise ValueError("episode.json's state files are not the ones service.json pins")
    return ep


def check_tree(cfg):
    """The copied code and state files match their digests; the second RoboCasa tree is the service account's."""
    check_digests(cfg["code_dir"], cfg["code_sha256"], "the kitchen's code")
    check_digests(cfg["state_dir"], cfg["state_sha256"], "the graded state's files")
    init = os.path.join(cfg["robocasa_dir"], "robocasa", "__init__.py")
    if not os.path.isfile(init) or os.path.islink(init):
        raise RuntimeError("the graded kitchens' RoboCasa tree is missing (%s)" % init)


def load_wire_rules(cfg):
    """The grader's wire_rules (stdlib only), imported from the checked copy as root."""
    spec = importlib.util.spec_from_file_location("longtail_wire_rules", os.path.join(cfg["code_dir"], "wire_rules.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class SimFailed(Exception):
    """The kitchen process failed (it exited, wrote something malformed or did not answer in time)."""


class Kitchen(object):
    """The graded state's kitchen: the grader's sim_worker.py as a child process of the worker (the service's account),
    spoken to as tests/verify.py's SimWorker speaks to it."""

    def __init__(self, cfg, ep, privileged, max_frames):
        self.cfg = cfg
        st = ep["state"]
        self.spec = {"id": st["id"], "task": st["task"], "seed": int(st["seed"]), "split": st["split"],
                     "prefix_file": os.path.join(cfg["state_dir"], st["prefix"]),
                     "qpos_T_file": os.path.join(cfg["state_dir"], st["qpos"]),
                     "cards_file": os.path.join(cfg["state_dir"], st["card"]), "roles": st.get("roles"),
                     "step_budget": int(ep["step_budget"]), "max_frames": int(max_frames),
                     "privileged": bool(privileged)}
        env = dict(cfg["sim_env"], HOME=cfg["sim_home"], PYTHONPATH=cfg["robocasa_dir"])
        logpath = os.path.join(cfg["sim_home"], "sim.log")
        self.err = open(logpath, "ab")
        self.proc = subprocess.Popen(["/opt/venv/bin/python3", "-s", "-B", os.path.join(cfg["code_dir"], "sim_worker.py")],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.err, cwd=cfg["code_dir"],
                                     env=env, close_fds=True)
        self.buf = b""
        self.ready = None

    def _write(self, obj):
        try:
            self.proc.stdin.write((json.dumps(obj) + "\n").encode("utf-8"))
            self.proc.stdin.flush()
        except (OSError, ValueError) as exc:
            raise SimFailed("the kitchen process stopped reading (%s)" % type(exc).__name__)

    def _read(self, timeout):
        deadline = time.monotonic() + float(timeout)
        fd = self.proc.stdout.fileno()
        while b"\n" not in self.buf:
            left = deadline - time.monotonic()
            if left <= 0:
                raise SimFailed("the kitchen process did not answer within %d s" % int(timeout))
            if not select.select([fd], [], [], left)[0]:
                continue
            chunk = os.read(fd, 1 << 20)
            if not chunk:
                code = self.proc.poll()
                raise SimFailed("the kitchen process exited (status %s)" % code)
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        try:
            msg = json.loads(line.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            raise SimFailed("the kitchen process wrote a malformed line")
        if not isinstance(msg, dict):
            raise SimFailed("the kitchen process wrote a malformed line")
        return msg

    def build(self):
        """Spawned: send the spec and read its ready line (the build, the prefix replay, the recorded state)."""
        self._write(self.spec)
        self.ready = self._read(self.cfg["build_timeout_s"])
        return self.ready

    def first(self):
        self._write({"first": True})
        return self._read(self.cfg["sim_reply_timeout_s"])

    def step(self, controls, cams):
        self._write({"action": controls, "cams": cams})
        return self._read(self.cfg["sim_reply_timeout_s"])

    def close(self):
        try:
            self._write({"close": True})
            self.proc.stdin.close()
            self.proc.wait(timeout=30)
        except Exception:  # noqa: BLE001
            try:
                self.proc.kill()
                self.proc.wait(timeout=30)
            except Exception:  # noqa: BLE001
                pass
        try:
            self.err.close()
        except OSError:
            pass


# ---------------------------------------------------------------------------------------------- the episode (worker)
class Client(object):
    def __init__(self, sock, cid, peer_uid):
        self.sock, self.cid, self.peer_uid, self.buf = sock, cid, peer_uid, b""
        self.last = time.monotonic()


class GradedEpisode(object):
    """The one graded episode, its windows and its record, served to one client at a time."""

    def __init__(self, cfg, ep, store, log, listener, W, state, snap_pipe=None):
        self.cfg, self.ep, self.store, self.log, self.listener = cfg, ep, store, log, listener
        self.W, self.state, self.snap_pipe = W, state, snap_pipe
        self.budget = int(ep["step_budget"])
        self.kitchen = None
        self.last_wire = None                         # the last observation sent (observe returns it again)
        self.client = None
        self.conn_seq = 0
        self.requests = 0
        self.refused = 0
        self.t = 0                                    # control steps since the restored state (the wire's t)
        self.run = 0                                  # consecutive steps with the job's success check holding
        self.best_run = 0
        self.recovered = False
        self.frames = 0
        self.sim_s = 0.0
        self.render_s = 0.0
        self.ready = {}
        self.sim_error = None
        self.dirty = False
        self.last_write = 0.0
        self.token_mismatch = False
        self.req_log = None
        self.req_log_size = 0
        self.req_log_full = False
        self.freeze_attempts = 0
        self.freeze_retry_at = 0.0
        self.last = {}                                # what the last record said (a restarted worker has no kitchen)

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
            rec = self.store.read_json("record.json") or {}
            self.last = {k: rec.get(k) for k in ("steps", "t", "sim_time_s", "requests")}
        self.store.write_json("state.json", self.state)
        try:
            fd = os.open(self.store.path("requests.jsonl"), os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
            self.req_log_size = os.fstat(fd).st_size
            self.req_log = fd
        except OSError as e:
            self.log.write(kind="requests_log_unavailable", err=str(e))

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

    def _base(self, now):
        return {"format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.ep["episode"],
                "seed_sha256": self.ep["seed_sha256"], "job": self.ep["state"]["task"],
                "service_version": SERVICE_VERSION, "written_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now))}

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
            rec = dict(self._base(now), started=False, ended=False, end_reason=None, requests=0, steps=0, t=0,
                       sim_time_s=0.0, wall_s_since_start=None, episode_score=0.0, recovered=False, metrics={},
                       privileged=False, sim_error=None, **self.snapshot_fields())
            self.store.write_json("record.json", rec)
        except OSError:
            self.log.write(kind="window_close_record_failed", tb=traceback.format_exc(limit=4))
        self.log.write(kind="frozen_at_window_close", attempts=self.freeze_attempts,
                       **{k: snap.get(k) for k in ("status", "files", "bytes", "skipped")})

    def _minimal_record(self, now):
        start = float(self.state.get("start_wall") or now)
        return dict(self._base(now), started=True, ended=False, end_reason=None, requests=0, steps=0, t=0,
                    sim_time_s=0.0, wall_s_since_start=now - start, episode_score=0.0, recovered=False, metrics={},
                    privileged=self.privileged, sim_error=self.state.get("sim_error"), **self.snapshot_fields())

    # -- the record
    def _record(self, now):
        m = {"recovered": 1.0 if self.recovered else 0.0, "steps": float(self.t), "t": float(self.t),
             "success_run": float(self.run), "success_run_max": float(self.best_run),
             "camera_frames": float(self.frames), "sim_s": float(self.sim_s), "render_s": float(self.render_s),
             "prefix_max_err": float(self.ready.get("prefix_max_err", 0.0) or 0.0),
             "kitchen_built_s": float(self.ready.get("built_s", 0.0) or 0.0),
             "malformed_actions_refused": float(self.refused), "privileged_mode": 1.0 if self.privileged else 0.0,
             "oracle_token_mismatch": 1.0 if self.token_mismatch else 0.0,
             "sim_error": 1.0 if self.sim_error else 0.0,
             "recovered_fraction_" + self.ep["state"]["task"]: 1.0 if self.recovered else 0.0}
        return dict(self._base(now), started=True, ended=self.ended, end_reason=self.end_reason,
                    requests=int(self.requests), steps=int(self.t), t=int(self.t),
                    sim_time_s=round(self.t * CONTROL_DT, 6),
                    wall_s_since_start=now - float(self.state["start_wall"]),
                    started_after_s=float(self.state["start_wall"]) - float(self.state["service_started"]),
                    episode_score=1.0 if self.recovered else 0.0, recovered=bool(self.recovered), metrics=m,
                    privileged=self.privileged, sim_error=self.sim_error, **self.snapshot_fields())

    def write_record(self, now):
        try:
            rec = self._record(now)
            data = (json.dumps(_finite(rec), indent=1, allow_nan=False) + "\n").encode("utf-8")
            self.store.write_bytes("record.json", data)
            self.last_write, self.dirty = now, False
        except Exception:  # noqa: BLE001 - a failed write is retried at the next flush; the last record stands
            self.log.write(kind="record_write_failed", tb=traceback.format_exc(limit=8))

    def flush(self, now, force=False):
        if self.started and (self.dirty or force) and \
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
        line = json.dumps({"n": self.requests, "wall": round(now - float(self.state.get("start_wall") or now), 3),
                           "method": str(method)[:64], "params": p, "t": self.t, "error": err}, default=str) + "\n"
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

    def close_kitchen(self):
        if self.kitchen is not None:
            k, self.kitchen = self.kitchen, None
            k.close()

    def end(self, reason, now):
        """End the episode: marked over, scored as it stands (nothing is simulated afterwards); the kitchen closes."""
        if self.ended or not self.started:
            return
        self.state.update({"ended": True, "end_reason": reason, "ended_wall": now, "sim_error": self.sim_error})
        try:
            self.store.write_json("state.json", self.state)
        except OSError:
            self.log.write(kind="state_write_failed", tb=traceback.format_exc(limit=4))
        self.flush(now, force=True)
        self.log.write(kind="episode_end", reason=reason, recovered=self.recovered, requests=self.requests,
                       steps=self.t, sim_error=self.sim_error)
        self.close_kitchen()

    def windows(self, now):
        if self.started and not self.ended and now >= self.graded_deadline():
            self.end("graded_window", now)
        if not self.started and not self.state.get("frozen") and not self.state.get("state_lost") and \
                now >= self.prep_deadline() and now >= self.freeze_retry_at:
            self.freeze_at_window_close(now)

    def ended_text(self):
        if self.end_reason == "done":
            if self.recovered:
                return "ended at step %d, before the step budget (%d steps)" % (self.t, self.budget)
            return "the step budget is used (%d steps)" % self.budget
        return {"finish": "finished", "graded_window": "the graded window ran out",
                "worker_restart": "the service restarted",
                "sim_failed": "the kitchen's process failed"}.get(self.end_reason)

    # -- requests
    def status(self, now):
        if self.started and self.kitchen is None and self.ended and not self.t:
            steps = int(self.last.get("steps") or 0)
        else:
            steps = self.t
        if not self.started:
            prep_left = max(0.0, self.prep_deadline() - now)
            graded_left = float(self.cfg["graded_s"]) if prep_left > 0 and not self.state.get("frozen") else 0.0
        else:
            prep_left = 0.0
            graded_left = 0.0 if self.ended else max(0.0, self.graded_deadline() - now)
        return {"started": self.started, "ended": self.ended, "end": self.end_reason,
                "ended_by": self.ended_text() if self.ended else None, "steps": steps,
                "sim_time_s": round(steps * CONTROL_DT, 3),
                "steps_left": 0 if self.ended else max(0, self.budget - steps),
                "camera_frames_left": max(0, int(self.W.MAX_FRAMES) - self.frames),
                "prep_window_left_s": round(prep_left, 1), "graded_window_left_s": round(graded_left, 1),
                "program_frozen": bool(self.state.get("frozen"))}

    def _wire(self, reply):
        return reply["obs"]

    def start(self, params, now):
        if not isinstance(params, dict):
            return {"error": "bad_params", "detail": ERRORS["bad_params"]}
        if self.started:
            return {"error": "already_started", "detail": ERRORS["already_started"]}
        if now > self.prep_deadline() or self.state.get("frozen") or self.state.get("state_lost"):
            return {"error": "prep_window_closed", "detail": ERRORS["prep_window_closed"]}
        token = params.get("token")
        privileged = False
        if token is not None:
            got = hashlib.sha256(str(token).encode("utf-8")).hexdigest()
            privileged = got == self.ep["oracle_token_sha256"]
            self.token_mismatch = not privileged
        # the kitchen first: a kitchen that cannot be built refuses the start and charges nothing
        t0 = time.time()
        kitchen = None
        try:
            kitchen = Kitchen(self.cfg, self.ep, privileged, self.W.MAX_FRAMES)
            ready = kitchen.build()
            if not ready.get("ready"):
                raise SimFailed("the kitchen could not be built: %s" % str(ready.get("error", "?"))[-300:])
            err = float(ready.get("prefix_max_err", float("inf")))
            if not ready.get("shape_ok") or not err <= float(self.cfg["prefix_tol"]):
                # the environment does not reproduce the recording: an error of the environment (a task error at
                # verify), never the agent's; nothing is charged and the record says so
                self.sim_error = "the replayed prefix differs from the recording (max %.3g, shape_ok %s)" % (
                    err, ready.get("shape_ok"))
                self.log.write(kind="sim_error", err=self.sim_error)
                self.state["sim_error"] = self.sim_error
                self.store.write_json("state.json", self.state)
                self.store.write_json("record.json", dict(self._base(now), started=False, ended=False,
                                                          end_reason=None, episode_score=0.0, recovered=False,
                                                          metrics={}, privileged=False, sim_error=self.sim_error,
                                                          **self.snapshot_fields()))
                kitchen.close()
                return {"error": "internal", "detail": ERRORS["internal"]}
            first = kitchen.first()
            if "obs" not in first:
                raise SimFailed("the kitchen's first observation is missing")
        except SimFailed as exc:
            self.log.write(kind="kitchen_failed", err=str(exc)[:500], wall_s=round(time.time() - t0, 1))
            if kitchen is not None:
                kitchen.close()
            return {"error": "kitchen_failed", "detail": ERRORS["kitchen_failed"]}
        # DM3: the program is frozen before anything is charged; a copy that fails refuses the start
        snap = self.request_snapshot()
        if snap.get("status") not in ("ok", "too_large", "missing"):
            self.log.write(kind="start_refused_snapshot", result=snap)
            kitchen.close()
            return {"error": "snapshot_failed", "detail": ERRORS["snapshot_failed"]}
        now = time.time()
        # the charge: persisted before anything is simulated, never refunded
        self.state.update({"started": True, "start_wall": now, "privileged": privileged, "frozen": "graded_start",
                           "frozen_wall": now, "snapshot": self._snap(snap, "start", now, 1)})
        try:
            self.store.write_json("state.json", self.state)
        except OSError:
            self.state.update({"started": False, "start_wall": None, "privileged": False, "snapshot": None,
                               "frozen": None, "frozen_wall": None})
            self.log.write(kind="start_not_persisted", tb=traceback.format_exc(limit=4))
            kitchen.close()
            return {"error": "internal", "detail": ERRORS["internal"]}
        self.kitchen, self.ready = kitchen, dict(ready)
        self.sim_s, self.render_s = float(first.get("sim_s", 0.0)), float(first.get("render_s", 0.0))
        self.frames = 3                               # the first observation's three RGB-D views
        self.log.write(kind="episode_start", privileged=privileged, token_mismatch=self.token_mismatch,
                       peer_uid=self.client.peer_uid if self.client else None, snapshot=snap.get("status"),
                       snapshot_files=snap.get("files"), kitchen_wall_s=round(now - t0, 1),
                       prefix_max_err=ready.get("prefix_max_err"))
        self.write_record(now)
        self.last_wire = first["obs"]
        return {"started": True, "obs": self.last_wire, "steps_left": self.budget,
                "graded_window_s": float(self.cfg["graded_s"])}

    def step(self, params, now):
        try:
            controls, cams = self.W.check_action(params.get("action"))
        except self.W.Malformed:
            self.refused += 1
            return {"error": "malformed_action", "detail": ERRORS["malformed_action"]}
        except Exception:  # noqa: BLE001 - anything else an action can provoke is malformed too
            self.refused += 1
            return {"error": "malformed_action", "detail": ERRORS["malformed_action"]}
        try:
            reply = self.kitchen.step(controls, cams)
            obs = reply["obs"]
        except (SimFailed, KeyError, TypeError) as exc:
            self.sim_error = "the kitchen's process failed at step %d: %s" % (self.t + 1, str(exc)[:300])
            self.log.write(kind="sim_error", err=self.sim_error)
            self.end("sim_failed", time.time())
            return {"error": "episode_over", "detail": "%s (the kitchen's process failed)" % ERRORS["episode_over"]}
        self.t += 1
        self.frames = int(reply.get("frames", self.frames))
        self.sim_s, self.render_s = float(reply.get("sim_s", self.sim_s)), float(reply.get("render_s", self.render_s))
        self.run = self.run + 1 if reply.get("success") else 0
        self.best_run = max(self.best_run, self.run)
        self.dirty = True
        self.last_wire = obs
        done = False
        if self.run >= int(self.W.SUCCESS_RUN):
            self.recovered, done = True, True
        elif self.t >= self.budget:
            done = True
        if done:
            self.end("done", time.time())
        return {"obs": obs, "done": done, "ended": self.ended_text() if done else None,
                "steps_left": 0 if done else max(0, self.budget - self.t)}

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
        if self.ended or self.kitchen is None:
            if method == "finish":
                return {"ended": True, "end": self.end_reason, "ended_by": self.ended_text()}
            return {"error": "episode_over", "detail": "%s (%s)" % (ERRORS["episode_over"], self.ended_text())}
        self.requests += 1
        if method == "observe":
            res = {"obs": self.last_wire, "done": False, "ended": None, "steps_left": max(0, self.budget - self.t)}
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
    """Forked from the supervisor as root: read the episode, check its files, drop privileges, serve."""
    _set_comm("longgrade-wkr")
    os.umask(0o077)
    store = Store(cfg["record_dir"])
    log = Log(store.path("service.log"), cfg["max_log_bytes"])
    os.fchown(log.fd, acct.pw_uid, acct.pw_gid)
    try:
        ep = read_episode(cfg)
        check_tree(cfg)
        W = load_wire_rules(cfg)
        state = store.read_json("state.json")
        if not isinstance(state, dict) or "service_started" not in state:
            # fail closed: without its state the service cannot tell a start from none, so nothing can start
            log.write(kind="state_unreadable")
            state = {"version": 1, "service_started": 0.0, "started": False, "ended": False, "worker_starts": 0,
                     "state_lost": True}
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
        for path in (cfg["code_dir"], cfg["state_dir"], cfg["robocasa_dir"], cfg["sim_home"]):
            if not os.access(path, os.R_OK | os.X_OK):
                raise RuntimeError("the service's account cannot read %s" % path)
        graded = GradedEpisode(cfg, ep, store, log, listener, W, state, snap_pipe)
        graded.begin()
        log.write(kind="worker_ready", uid=os.getuid(), gid=os.getgid(), seccomp_inet_blocked=sec_ok,
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
    graded.close_kitchen()
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
    _set_comm("longgrade-sup")
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
