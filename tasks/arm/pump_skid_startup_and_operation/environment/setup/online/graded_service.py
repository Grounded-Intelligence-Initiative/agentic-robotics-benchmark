#!/usr/bin/env python3
"""graded_service: the graded episode of pump_skid_startup_and_operation (the online variant, experimental).

Started once by environment/setup/setup.sh as root, cwd = the setup stage (/opt/ale/setup, root-only while the agent
works), before the agent's session:

    env -i PATH=/opt/venv/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \\
        /opt/venv/bin/python3 -I -B online/graded_service.py --config online/service.json

It returns once the service serves (exit 0) or failed to start (exit 1); the service itself is a detached daemon
(setsid, stdio on /dev/null) that runs through the agent's session.

The episode. ONE episode of the offline grader's fixed hidden set (online/episode.json, read by the worker only: its
id and seed, drawn once from tests/hidden/seeds.json) on the grader's own cell: online/skidcell/ is the image's byte
copy of tests/skidcell/ (/opt/skidcell-commission/skidcell/*.py, which setup.sh copies here before any other account
runs), and the worker refuses to start unless every file matches service.json's digests. The cell is built as the
offline grader builds it (mode hidden, the grader's cell configuration, the grader's numerics pins) and runs the same
episode: the same skid, process, faults, safety watch, horizon (1800 s simulated), step cap (1000 ticks a call), camera
budget (600 frames) and score (skidcell.episode). What does not carry over is what binds a program process: program
time, CPU time, silence, the allowance after the end, the import check and the program-contract gates. The simulation
advances only inside the agent's own `step` requests.

Rules:
  * `start` is accepted once per session, within `prep_s` of the service's start (state.json `service_started`); it
    is persisted (fsync) before anything is simulated and never refunded. With the oracle's token (its sha256 is in
    episode.json) the episode's cell gets the privileged calls, as the offline grader gives them to the oracle's
    submission; a wrong token is ignored and counted.
  * The episode lives in the worker, not in a connection: a disconnect leaves it where it is and a new connection
    resumes it. One client at a time: a second connection is answered `busy` and closed.
  * It ends at its own end (the cell's: the sim cap, a physics fault, an internal error), at the agent's `finish`,
    `graded_s` after its start (the service's own timer), or when the session ends (the service is frozen or killed:
    the last record stands). The offline grader ends an episode whose program stops by marking it over and scoring it
    as it stands, with nothing simulated afterwards; so does this service. An ended episode answers `status` and
    `finish` (which changes nothing) only.
  * When the worker dies or restarts, the episode ends with its last record (end_reason worker_restart): the new
    worker never builds a cell for it.
  * The agent learns the cell's replies and `status` (started / ended, steps, simulated time, frames, the windows'
    time left): never the score, the record, the episode's id or its seed.
  * The freeze (the dual mode's DM3): before the start is charged, the worker asks the root supervisor to copy the
    agent's submission folder into the root-only record/submission_snapshot/ (regular files and folders only, never
    following a link; main's 60 MB cap and bounds on entries and depth: over any of them the start goes on with
    snapshot_status too_large and no copy). A copy that fails refuses the start; nothing is charged. tests/verify.py
    grades that copy as the program (main's offline grading) and the record as the graded episode.
  * Practice ends when the graded episode starts (O13): before the start is persisted, the worker writes the flag
    `practice_flag` (/run/skidcell-graded/flags/started: world-readable, in a directory only this service's account
    can write, inside the root-owned socket directory), and the commissioning service ends its open shifts and refuses
    new ones from then on (environment/commission_service/commissiond.py). A start whose flag cannot be written is
    refused (nothing charged); a restarted worker writes the flag again if the episode had started.

Process layout (the alelab daemon pattern of arm_hidden_mechanism_box):
  supervisor (root, comm skidgrade-sup): PR_SET_NO_NEW_PRIVS and a seccomp filter under which only AF_UNIX sockets can
      be created; trimmed capabilities; binds the socket in its own root-owned directory, forks the worker and
      restarts it (bounded), and on the worker's one-byte request freezes the submission folder (snapshot_submission:
      a fixed source, a fixed target, no other input). It never reads episode.json and never accepts a connection.
  worker (comm skidgrade-wkr): forked as root; reads episode.json and state.json, checks and imports the cell and the
      reference program, builds the episode's cell, then drops to the account `skidgrade` (setgroups, setgid, setuid,
      checked irreversible; not dumpable; no core files) and serves.

Record (/var/lib/arb-online: owner skidgrade, mode 0700, in the root-owned /var/lib): record.json (format
arb-online-record/1, rewritten atomically: a temp file, fsync, rename) at the start, at most once a second while
requests change the episode and within a second of the last one, and when the episode ends; requests.jsonl (one line
per request: method, arguments, sim time, the error if any; no images; it stops growing at 32 MB); state.json (the
service's start, the one start, the end); service.log and supervisor.log. task.toml declares the directory an
artifact; tests/verify.py scores record.json.

Wire: the cell's JSON lines ({"id": n, "method": ..., "params": {...}} -> {"id": n, "result": ...} or {"id": n,
"error": code, "detail": text}) plus `status` and `start` (params: an optional `token`); a second connection gets
{"id": null, "error": "busy", "detail": ...} and is closed. skidcell.client.connect_graded speaks it.

Dependencies: Python 3.12 standard library here; the worker imports the cell (numpy, MuJoCo, Pillow) from the venv.
"""

import argparse
import ctypes
import hashlib
import importlib
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
import sys
import time
import traceback

SERVICE_VERSION = "pump-skid-graded/1"
RECORD_FORMAT = "arb-online-record/1"
MAX_LINE = 4 * 1024 * 1024                 # one request line, as the offline wire caps it
REQUIRED = ("task", "user", "agent_user", "socket", "record_dir", "prep_s", "graded_s", "episode_file", "cell_dir",
            "cell_sha256", "reference_dir", "reference_sha256", "given_dir", "cell_config", "env", "practice_flag",
            "submission_dir")
DEFAULTS = {"max_restarts": 3, "start_timeout_s": 180.0, "record_interval_s": 1.0, "requests_log_cap_bytes": 32 << 20,
            "record_max_bytes": 1000000, "reply_timeout_s": 120.0, "max_log_bytes": 16 << 20, "note": "",
            "snapshot_name": "submission_snapshot", "snapshot_max_bytes": 60 << 20, "snapshot_max_entries": 10000,
            "snapshot_max_depth": 32, "snapshot_timeout_s": 120.0}
ERRORS = {
    "busy": "another connection holds the graded episode: end that script (or close its connection), then connect again",
    "bad_request": "one JSON object per line, with a method",
    "bad_params": "start(token=None): params must be an object",
    "not_started": "the graded episode has not started: start() starts it, once per session",
    "already_started": "the graded episode was started already (once per session): connect_graded() resumes it",
    "prep_window_closed": "the graded episode can no longer be started: its start window has closed",
    "episode_over": "the graded episode has ended: nothing more is simulated or scored",
    "internal": "internal error of the graded episode's service",
}
END_REASONS = ("done", "finish", "graded_window", "worker_restart")
EVENT_TYPES = ("demand_step", "filter_slug", "supply_loss", "dosing_airlock", "drum_empty", "pump_trip")

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
    """The .py files of `folder` are exactly `want` (name -> sha256), or RuntimeError."""
    have = {}
    for name in sorted(os.listdir(folder)):
        p = os.path.join(folder, name)
        if name.endswith(".py"):
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
    try:
        v = float(x)
    except (TypeError, ValueError):
        return str(x)
    if hasattr(x, "dtype") and getattr(x.dtype, "kind", "") in "iub":
        return int(x) if x.dtype.kind != "b" else bool(x)
    return v if math.isfinite(v) else None


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
    """The record directory: atomic JSON files (a temp file, fsync, rename, the directory fsynced) and the request log."""

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
    for key in ("episode_file", "cell_dir", "reference_dir"):
        cfg[key] = os.path.normpath(os.path.join(base, cfg[key]))
    for key in ("socket", "record_dir", "given_dir", "practice_flag", "submission_dir"):
        if not os.path.isabs(cfg[key]):
            raise ValueError("%s must be an absolute path" % key)
    if os.path.dirname(os.path.dirname(cfg["practice_flag"])) != os.path.dirname(cfg["socket"]):
        raise ValueError("the practice flag's directory must sit in the socket's root-owned directory")
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
    try:
        agent = pwd.getpwnam(cfg["agent_user"])
    except KeyError:
        agent = None
    if agent is not None and agent.pw_uid == acct.pw_uid:
        raise RuntimeError("the service account must differ from the agent's")
    return acct


# ---------------------------------------------------------------------------------------------- the freeze (supervisor, root)
class SnapshotTooLarge(Exception):
    """The submission folder exceeds a cap of the freeze (bytes, entries or depth)."""


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


def snapshot_submission(cfg):
    """DM3, run by the root supervisor on the worker's request, before the graded start is charged: the agent's
    submission folder as it is now, copied into the root-only folder record_dir/snapshot_name. No input but the request:
    a fixed source and a fixed target, so the worker cannot have root copy anything else. Every path is opened relative
    to its parent's descriptor with O_NOFOLLOW, so no link is ever followed (links, special files and an entry that
    changes kind meanwhile are skipped and counted); main's size cap (snapshot_max_bytes = the grader's
    max_submission_mb) and bounds on entries and depth: over any, status too_large and no copy. The copy is built beside
    the target and renamed into place. A missing folder (or a link in its place) freezes as an empty one, as the
    engine's artifact copy skips a link. Returns {status, files, bytes, sha256, skipped, reason}."""
    dst = os.path.join(cfg["record_dir"], cfg["snapshot_name"])
    tmp = dst + ".partial"
    _remove_tree(tmp)
    os.mkdir(tmp, 0o700)
    st = {"files": 0, "bytes": 0, "entries": 0, "skipped": 0}
    listing = []
    cap_bytes, cap_entries, cap_depth = int(cfg["snapshot_max_bytes"]), int(cfg["snapshot_max_entries"]), \
        int(cfg["snapshot_max_depth"])

    def copy_file(sfd, name, dfd, rel):
        try:
            ffd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=sfd)
        except OSError:
            st["skipped"] += 1
            return
        try:
            if not stat.S_ISREG(os.fstat(ffd).st_mode):
                st["skipped"] += 1
                return
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
            finally:
                os.close(out)
            st["files"] += 1
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
            elif stat.S_ISDIR(info.st_mode):
                try:
                    cfd = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=sfd)
                except OSError:
                    st["skipped"] += 1
                    continue
                try:
                    os.mkdir(name, 0o700, dir_fd=dfd)
                    cdfd = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=dfd)
                    try:
                        walk(cfd, cdfd, prefix + name + "/", depth + 1)
                    finally:
                        os.close(cdfd)
                finally:
                    os.close(cfd)
            else:
                st["skipped"] += 1                    # a link, a socket, a fifo, a device: never followed or copied

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
    return {"status": status, "reason": reason, "files": st["files"], "bytes": st["bytes"], "skipped": st["skipped"],
            "entries": st["entries"], "sha256": snapshot_digest(listing) if status == "ok" else None}


def _serve_worker_requests(cfg, pid, qr, aw, log):
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
            res = snapshot_submission(cfg)
        except Exception as exc:  # noqa: BLE001 - a failed copy refuses the start (the worker's side)
            res = {"status": "failed", "reason": type(exc).__name__}
            log.write(kind="snapshot_failed", tb=traceback.format_exc(limit=8))
        res["wall_s"] = round(time.time() - t0, 3)
        log.write(kind="snapshot", **{k: res.get(k) for k in ("status", "reason", "files", "bytes", "skipped", "wall_s")})
        try:
            os.write(aw, (json.dumps(res) + "\n").encode())
        except OSError:
            pass


# ---------------------------------------------------------------------------------------------- the cell (worker, as root)
def build_cell(cfg, ep, log):
    """Check and import the grader's cell and the reference program, build the episode's cell (mode hidden, unprivileged
    until an oracle token arrives) and reset it to the graded episode's seed. Runs as root before the privilege drop:
    every module is imported here, since the worker cannot read online/ afterwards."""
    for k, v in cfg["env"].items():
        os.environ[k] = v                             # before numpy and OpenBLAS load: the grader's numerics pins
    check_digests(cfg["cell_dir"], cfg["cell_sha256"], "the cell library")
    check_digests(cfg["reference_dir"], cfg["reference_sha256"], "the reference program")
    parent = os.path.dirname(os.path.abspath(cfg["cell_dir"]))
    ref_dir = os.path.abspath(cfg["reference_dir"])
    if os.path.basename(cfg["cell_dir"]) != "skidcell":
        raise RuntimeError("the cell library's folder must be named skidcell")
    sys.path.insert(0, parent)
    try:
        pkg = importlib.import_module("skidcell")
        for name in sorted(cfg["cell_sha256"]):
            if name != "__init__.py":
                importlib.import_module("skidcell." + name[:-3])
        server_mod = sys.modules["skidcell.server"]
        spec = importlib.util.spec_from_file_location("skidcell_reference_controller", os.path.join(ref_dir, "controller.py"))
        ref = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ref)                  # it puts its own folder on sys.path to import skills
        import PIL.Image  # noqa: F401  (the frames' encoder, imported lazily by the cell)
    finally:
        for p in (parent, ref_dir):
            while p in sys.path:
                sys.path.remove(p)

    class GradedCell(server_mod.SkidCellServer):
        """The grader's cell; reference_run runs the reference program loaded above (the offline cell loads it from
        the grader's folder at the call), with the same reply."""

        def reference_run(self):
            t0 = time.time()
            result = ref.run(self)
            return {"ok": True, "wall_s": time.time() - t0, "sim_time": float(self.scene.data.time),
                    "result": server_mod._jsonable(result)}

    def cell_log(msg):
        log.write(kind="cell", msg=str(msg)[:8000])

    cell = GradedCell(cfg["given_dir"], config=cfg["cell_config"], mode="hidden", privileged=False, render=True,
                      log=cell_log, reference_dir=ref_dir)
    cell.reset(int(ep["seed"]))
    return cell, getattr(pkg, "CELL_VERSION", None)


def read_episode(cfg):
    with open(cfg["episode_file"], "rb") as fh:
        ep = json.loads(fh.read(1 << 16))
    for k in ("episode", "seed", "seed_sha256", "oracle_token_sha256"):
        if k not in ep:
            raise ValueError("episode.json lacks %s" % k)
    if hashlib.sha256(str(int(ep["seed"])).encode()).hexdigest() != ep["seed_sha256"]:
        raise ValueError("episode.json: the seed does not match its digest")
    return ep


# ---------------------------------------------------------------------------------------------- the episode (worker)
class Client(object):
    def __init__(self, sock, cid, peer_uid):
        self.sock, self.cid, self.peer_uid, self.buf = sock, cid, peer_uid, b""


class GradedEpisode(object):
    """The one graded episode, its windows and its record, served to one client at a time."""

    def __init__(self, cfg, ep, store, log, listener, cell, cell_version, state, snap_pipe=None):
        self.cfg, self.ep, self.store, self.log, self.listener = cfg, ep, store, log, listener
        self.snap_pipe = snap_pipe
        self.cell, self.cell_version, self.state = cell, cell_version, state
        self.client = None
        self.conn_seq = 0
        self.requests = 0
        self.dirty = False
        self.last_write = 0.0
        self.token_mismatch = False
        self.req_log = None
        self.req_log_size = 0
        self.req_log_full = False
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
            try:
                self.close_practice()                 # a worker that died before writing it: practice stays closed
            except OSError:
                self.log.write(kind="practice_flag_failed", tb=traceback.format_exc(limit=4))
            rec = self.store.read_json("record.json") or {}
            self.last = {k: rec.get(k) for k in ("steps", "sim_time_s", "requests")}
            self.last["frames"] = (rec.get("metrics") or {}).get("camera_frames_mean")
        self.store.write_json("state.json", self.state)
        try:
            fd = os.open(self.store.path("requests.jsonl"), os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
            self.req_log_size = os.fstat(fd).st_size
            self.req_log = fd
        except OSError as e:
            self.log.write(kind="requests_log_unavailable", err=str(e))

    def close_practice(self):
        """O13: the flag that ends practice (the commissioning service stats it). Written once, atomically; raises
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
        return {"snapshot_status": sn.get("status"), "snapshot_files": sn.get("files"), "snapshot_bytes": sn.get("bytes"),
                "snapshot_sha256": sn.get("sha256"), "snapshot_skipped": sn.get("skipped")}

    def _minimal_record(self, now):
        start = float(self.state.get("start_wall") or now)
        return {"format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.ep["episode"],
                "seed_sha256": self.ep["seed_sha256"], "started": True, "ended": False, "end_reason": None, "requests": 0,
                "steps": 0, "sim_time_s": 0.0, "wall_s_since_start": now - start, "episode_score": 0.0, "metrics": {},
                "service_version": SERVICE_VERSION, **self.snapshot_fields()}

    # -- the record
    def _record(self, now):
        c = self.cell
        t = float(c.scene.data.time)
        s = c.monitor.summary(c._voids(), t)                # the score as if the episode ended now (episode.py)
        m = {"raw_score_mean": s["raw_score"], "void_rate": 1.0 if s["voided"] else 0.0,
             "safety_violation_rate": 0.0 if not c.violations else 1.0, "protective_stops_mean": float(len(c.stops)),
             "t101_overflow_rate": 0.0 if s.get("t101_overflow_t") is None else 1.0,
             "window_level_fraction_mean": s["window_fraction"]["level"],
             "window_conc_fraction_mean": s["window_fraction"]["conc"], "mean_sim_time_s": t,
             "illegal_call_count": float(c.illegal_calls), "step_calls": float(c.step_calls), "ticks": float(c.tick_count),
             "camera_frames_mean": float(c.frames_used), "render_wall_s_mean": float(c.render_wall_s),
             "cell_work_s_mean": float(c.work_wall), "cell_internal_errors": float(c.internal_errors),
             "frames_lost": float(c.frames_lost), "physics_fault": 1.0 if c.done_reason == "physics_fault" else 0.0,
             "locked_out": 1.0 if c._lockout else 0.0, "privileged_mode": 1.0 if c.privileged else 0.0,
             "oracle_token_mismatch": 1.0 if self.token_mismatch else 0.0}
        for part in ("startup", "operation", "events", "shutdown", "log"):
            m["part_mean_" + part] = float(s["parts"].get(part, 0.0))
        for cam, n in sorted(c.frames_by_camera.items()):
            m["camera_frames_mean_" + cam] = float(n)
        for typ in EVENT_TYPES:
            sel = [e["credit"] for e in s["events"] if e["type"] == typ and not e.get("excluded")]
            if sel:
                m["event_credit_mean_" + typ] = sum(sel) / len(sel)
        detail = {"parts": s["parts"], "startup": s["startup"], "first_start_t": s["first_start_t"],
                  "lineup_at_start": s["lineup_at_start"], "window_fraction": s["window_fraction"],
                  "events": s["events"][:12], "shutdown": s["shutdown"], "log": s["log"], "voids": s["voids"][:20],
                  "protective_stops": [dict(x) for x in c.stops[:10]], "t101_overflow_t": s.get("t101_overflow_t"),
                  "layout_faults": list(c.layout.get("faults", []))[:8]}
        return {"format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.ep["episode"],
                "seed_sha256": self.ep["seed_sha256"], "started": True, "ended": self.ended, "end_reason": self.end_reason,
                "requests": int(self.requests), "steps": int(c.step_calls), "sim_time_s": t,
                "wall_s_since_start": now - float(self.state["start_wall"]), "episode_score": float(s["score"]),
                "metrics": m, "service_version": SERVICE_VERSION, "cell_version": self.cell_version, **self.snapshot_fields(),
                "cell_done_reason": c.done_reason, "written_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)),
                "detail": detail}

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
            p = json.dumps(params, default=str)[:2000] if method != "start" else json.dumps(
                {"token": "<given>" if isinstance(params, dict) and params.get("token") else None})
        except (TypeError, ValueError):
            p = "<unprintable>"
        err = res.get("error") if isinstance(res, dict) and set(res) <= {"error", "detail"} else None
        sim = float(self.cell.scene.data.time) if self.cell is not None else None
        line = json.dumps({"n": self.requests, "t": round(now - float(self.state.get("start_wall") or now), 3),
                           "method": method[:64], "params": p, "sim_time": sim, "error": err}, default=str) + "\n"
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
        """End the episode as the offline grader ends one whose program stops: marked over, scored as it stands."""
        if self.ended or not self.started:
            return
        if self.cell is not None and not self.cell.done:
            self.cell._end(reason)
        self.state.update({"ended": True, "end_reason": reason, "ended_wall": now})
        try:
            self.store.write_json("state.json", self.state)
        except OSError:
            self.log.write(kind="state_write_failed", tb=traceback.format_exc(limit=4))
        self.flush(now, force=True)
        self.log.write(kind="episode_end", reason=reason, cell_done_reason=self.cell.done_reason if self.cell else None,
                       requests=self.requests)

    def windows(self, now):
        if self.started and not self.ended and now >= self.graded_deadline():
            self.end("graded_window", now)

    # -- requests
    def status(self, now):
        c = self.cell
        if c is not None:
            steps, sim, frames = int(c.step_calls), float(c.scene.data.time), int(c.frames_used)
        else:
            steps, sim, frames = int(self.last.get("steps") or 0), float(self.last.get("sim_time_s") or 0.0), \
                int(self.last.get("frames") or 0)
        budget = int(self.cfg["cell_config"].get("camera_budget_frames", 600))
        if not self.started:
            prep_left = max(0.0, self.prep_deadline() - now)
            graded_left = float(self.cfg["graded_s"]) if prep_left > 0 else 0.0
        else:
            prep_left = 0.0
            graded_left = 0.0 if self.ended else max(0.0, self.graded_deadline() - now)
        return {"started": self.started, "ended": self.ended, "end": self.end_reason, "steps": steps,
                "sim_time_s": sim, "camera_frames_used": frames, "camera_frames_left": max(0, budget - frames),
                "prep_window_left_s": round(prep_left, 1), "graded_window_left_s": round(graded_left, 1)}

    def start(self, params, now):
        if not isinstance(params, dict):
            return {"error": "bad_params", "detail": ERRORS["bad_params"]}
        if self.started:
            return {"error": "already_started", "detail": ERRORS["already_started"]}
        if now > self.prep_deadline() or self.cell is None:
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
        # O13: practice ends when the graded episode starts; no start without the flag that ends it
        try:
            self.close_practice()
        except OSError:
            self.log.write(kind="practice_flag_failed", tb=traceback.format_exc(limit=4))
            return {"error": "internal", "detail": ERRORS["internal"]}
        # the charge: persisted before anything is simulated, never refunded
        self.state.update({"started": True, "start_wall": now, "privileged": privileged,
                           "snapshot": {k: snap.get(k) for k in ("status", "reason", "files", "bytes", "sha256", "skipped")}})
        try:
            self.store.write_json("state.json", self.state)
        except OSError:
            self.state.update({"started": False, "start_wall": None, "privileged": False, "snapshot": None})
            self.log.write(kind="start_not_persisted", tb=traceback.format_exc(limit=4))
            return {"error": "internal", "detail": ERRORS["internal"]}
        self.cell.privileged = privileged
        self.cell.wall_start = now
        self.log.write(kind="episode_start", privileged=privileged, token_mismatch=self.token_mismatch,
                       peer_uid=self.client.peer_uid if self.client else None)
        self.write_record(now)
        return {"ok": True, "started": True, "sim_time": float(self.cell.scene.data.time),
                "graded_window_s": float(self.cfg["graded_s"])}

    def dispatch(self, method, params, now):
        if method == "status":
            return self.status(now)
        if method == "start":
            res = self.start(params if params is not None else {}, now)
            self._log_request(now, "start", params, res)
            return res
        if not self.started:
            return {"error": "not_started", "detail": ERRORS["not_started"]}
        if self.ended:
            if method == "finish":
                return {"ok": True, "sim_time": float(self.status(now)["sim_time_s"]), "ended": True}
            return {"error": "episode_over", "detail": "%s (%s)" % (ERRORS["episode_over"], self.end_reason)}
        self.requests += 1
        res = self.cell.handle(method, {} if params is None else params)
        self.dirty = True
        self._log_request(now, method, params, res)
        if self.cell.done:
            self.end("finish" if method == "finish" else "done", time.time())
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
        try:
            res = self.dispatch(method, req.get("params"), now)
        except Exception:  # noqa: BLE001 - the cell's handle() never raises; this is the service's own failure
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
        c.buf += chunk
        while self.client is c:
            nl = c.buf.find(b"\n")
            if nl < 0:
                if len(c.buf) > MAX_LINE:
                    self._send({"id": None, "error": "bad_request", "detail": "a request line is longer than 4 MB"})
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
            wait = 60.0
            if self.started and not self.ended:
                wait = min(wait, self.graded_deadline() - now)
            if self.dirty:
                wait = min(wait, self.last_write + float(self.cfg["record_interval_s"]) - now)
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


def _jsonable(obj):
    if hasattr(obj, "tolist"):
        return obj.tolist()
    if isinstance(obj, (set, frozenset)):
        return sorted(obj, key=str)
    return str(obj)


def run_worker(cfg, acct, listener, ready_fd, snap_pipe):
    """Forked from the supervisor as root: read the episode, build its cell, drop privileges, serve."""
    _set_comm("skidgrade-wkr")
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
        cell, cell_version = None, None
        fresh = not state.get("started") and not state.get("ended") and not state.get("state_lost")
        if fresh:
            cell, cell_version = build_cell(cfg, ep, log)
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
        episode = GradedEpisode(cfg, ep, store, log, listener, cell, cell_version, state, snap_pipe)
        episode.begin()
        log.write(kind="worker_ready", uid=os.getuid(), gid=os.getgid(), seccomp_inet_blocked=sec_ok, fresh=fresh,
                  worker_starts=state.get("worker_starts"), service_version=SERVICE_VERSION, cell_version=cell_version)
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


def run_supervisor(cfg, acct, notify_fd):
    """Root, never reads episode.json, never accepts a connection: binds the socket, forks and restarts the worker."""
    _set_comm("skidgrade-sup")
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
        code = _serve_worker_requests(cfg, pid, qr, aw, sup_log)
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
        os.unlink(cfg["practice_flag"])              # a fresh service: nothing has started, practice is open
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
        acct = _account(cfg)
        os.umask(0o077)
        sockdir = os.path.dirname(cfg["socket"])
        os.makedirs(os.path.dirname(sockdir), exist_ok=True)
        _secure_dir(sockdir, 0o755, 0, 0)
        # O13: the practice flag's directory, writable by the service's account only, readable by everyone
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
        for k in list(os.environ):
            if k not in ("PATH", "LANG", "HOME"):
                del os.environ[k]
        try:
            run_supervisor(cfg, acct, w)
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
