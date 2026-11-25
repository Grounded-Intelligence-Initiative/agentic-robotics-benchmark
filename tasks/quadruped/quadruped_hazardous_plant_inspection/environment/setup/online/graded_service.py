#!/usr/bin/env python3
"""plant-graded: the graded episode's service of quadruped_hazardous_plant_inspection (dual mode, experimental).

The agent drives ONE hidden graded mission of the robot cell itself, during its session, through this service. The
service writes the mission's outcome to /var/lib/arb-online (a declared artifact the agent can neither read nor
write), and the separate verifier scores that record (tests/verify.py). The cell is the offline grader's own:
/opt/plantgrade/plantcell is a byte copy of tests/plantcell (setup.sh copies it from the image's practice ground,
whose cell is the same bytes; the worker checks every file against cell_manifest.json), cell_config.json is a byte
copy of tests/grader_config.json, and the mission is one site package of the grader's fixed hidden set
(hidden/seeds.json) with its pinned noise seed, built and scored exactly as tests/verify.py builds and scores a
mission (PlantCellServer, mode hidden).

Started once by environment/setup/setup.sh as root, cwd = the setup stage (/opt/ale/setup, root-only while the agent
works):

    env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \\
        /opt/venv/bin/python3 -I -B online/graded_service.py --config online/service.json

The command returns once the service serves (exit 0) or failed to start (exit 1); the service itself is a detached
daemon (setsid, stdio on /dev/null) that lives into the agent phase. The layout is the alelab daemon's
(arm_hidden_mechanism_box):
  supervisor (root, comm "plantgrade-sup"): binds the unix socket, writes the pid file, forks the worker and restarts
      it at most max_restarts times, and freezes the agent's program on the worker's request (below). It never reads
      the hidden mission and never accepts a connection.
  worker (comm "plantgrade-wkr"): forked as root. While the graded episode has not started it reads the hidden mission
      (online/episode.json and its site package), checks and imports the cell, builds the mission (the site, the
      robot on the pad, its sensors: the same build as the grader's) and only then drops to the account `plantgrade`
      irreversibly (setgroups([]), setgid, setuid; not dumpable; no core files) and serves. A worker that finds the
      episode started (a restart) reads nothing hidden: the episode ends with its last record (end_reason
      worker_restart, no resume) and the worker answers `status` only.
  Both run under PR_SET_NO_NEW_PRIVS and a seccomp filter under which socket() / socketpair() work only for AF_UNIX.

The graded episode:
  * `start` charges it once per session: the charge is persisted (state.json, fsync) before anything is simulated and
    is never refunded. A start is accepted only within prep_s of the service's start.
  * The freeze (dual mode, DM3): the program freezes at the earlier of the graded start and the start window's close
    (prep_s after the service's start). Before a start is charged, or when the window closes without a start, the
    worker asks the supervisor, over the socketpair it holds since its fork, to copy /home/user/submission into
    submission_snapshot in the record directory (root-owned 0700 / 0600; regular files and folders of the agent's
    account only, every entry opened relative to its parent with O_NOFOLLOW, so no link is followed; links, special
    files, entries of other accounts, __pycache__ and odd names skipped; within the offline submission's size cap,
    cell_config's route_gate, and the entry and depth caps, else nothing is copied: too_large). The supervisor answers
    only this request, with fixed paths, and only while the state file says the program has not been frozen; a copy
    that fails refuses the start and charges nothing. At the window's close a failed copy is tried freeze_attempts
    times and then recorded as failed (the verifier scores the program item 0: it never falls back to the session-end
    submission). The record carries snapshot_status, snapshot_at (graded_start | prep_window), snapshot_files,
    snapshot_bytes, snapshot_sha256 and snapshot_skipped; a window that closes without a start writes a record of its
    own (started false, prep_window_closed true) and no start is accepted afterwards.
  * Practice ends when the graded episode starts (O13): before the start is charged the worker writes the flag
    practice_flag (world-readable, in a directory only this account can write); the practice ground
    (environment/practice_ground/practiced.py) ends its open missions and refuses new ones once the flag exists. A
    start whose flag cannot be written is refused (nothing charged).
  * The episode lives in the worker, not in a connection: a disconnect leaves it where it is and a new connection
    resumes it. One client at a time (a second connection is told `busy`). A connection idle for idle_timeout_s is
    closed; the episode stays.
  * It ends at the cell's own end of the mission (1350 s of simulated time, the robot lost, the dose rating reached,
    or the cell's own failure inside a call: `done`), at `finish` (`finish`), graded_s after its start
    (`graded_window`), or when the session ends (the sandbox is frozen or destroyed: the last record applies). It ends
    the way the offline grader ends a mission whose program stops: the cell's end of the mission at that instant (the
    link's last look, reports still queued then lost), nothing simulated afterwards, the mission scored as it stands.
  * The record (record.json, written atomically: a temp file, fsync, rename) is rewritten after a request that changes
    the episode, at most every record_min_interval_s (a pending change is written about a quarter of a second after
    the last request), on every disconnect, and always when the episode starts and ends. While the episode runs its
    episode_score is the mission's score as if the mission ended at that moment (the same end of a mission, computed
    on a copy of the reports). requests.jsonl logs every request of the episode (method, arguments, simulated time; no
    images; the oracle token never) up to requests_log_max_bytes.
  * The client learns the cell's replies (the offline wire's) and `status` (started / ended, steps, simulated time,
    frames used, the time left in each window); never the score, the record, the episode's id or its seed.

Protocol: one JSON object per line in each direction, in the offline wire's envelope:
    {"id": n, "method": m, "params": {...}}  ->  {"id": n, "result": ...}  |  {"id": n, "error": code, "detail": text}
On connect the service sends one greeting line, {"service": SERVICE}, or {"service": SERVICE, "error": "busy", ...} and
closes. Methods: `status`, `start`, and every call of the cell (the robot's public calls; the privileged
get_state_privileged and reference_run only on an episode started with the oracle token, as offline; any other name
is the cell's `forbidden`, counted as offline). The service's own errors: bad_request, line_too_long, bad_params (of
start / status), not_started, already_started, start_window_closed, episode_over, internal_error.

Dependencies: the Python 3.12 standard library for the supervisor; the worker imports the cell (numpy, mujoco, cv2).
"""

import argparse
import copy
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
import sys
import threading
import time
import traceback

SERVICE = "plant-graded/1"
SERVICE_VERSION = "1"
RECORD_FORMAT = "arb-online-record/1"
DEFAULTS = {
    "task": None,                     # the task's slug (the record's `task`)
    "episode": "episode.json",        # the hidden graded mission (relative to the config file; root-only)
    "package_dir": ".",               # the folder of the mission's site package (relative to the config file)
    "cell_root": "/opt/plantgrade",   # holds plantcell/ (the grader's cell) and reference/ (the privileged reference)
    "cell_manifest": "cell_manifest.json",   # sha256 of every file of both (relative to the config file)
    "cell_config": "cell_config.json",       # byte copy of tests/grader_config.json
    "user": "plantgrade",             # the dedicated account (created in the task image)
    "agent_user": "user",             # the evaluated account; the service refuses to run as it
    "socket": "/run/plantgrade/graded.sock",
    "practice_flag": "/run/plantgrade/flags/started",
    "state_dir": "/var/lib/arb-online",
    "prep_s": 14400.0,                # a start is accepted within this many seconds of the service's start
    "graded_s": 10800.0,              # the graded window: the episode ends this many seconds after its start
    "idle_timeout_s": 900.0,          # a connection that sends nothing for this long is closed (the episode stays)
    "send_timeout_s": 300.0,          # a reply the client does not take within this long closes the connection
    "call_timeout_s": 900.0,          # one cell call longer than this ends the worker (reference_run excepted)
    "max_line_bytes": 4 << 20,        # a request line (the offline wire's limit)
    "record_min_interval_s": 0.5,
    "requests_log_max_bytes": 32 << 20,
    "max_log_bytes": 64 << 20,
    "max_restarts": 3,
    "start_timeout_s": 600.0,
    "require_seccomp": True,
    "env": {},                        # PUBLIC constants only (numeric pins, renderer); never hidden values
    # the freeze (dual mode): the root supervisor copies this folder at the earlier of the start and the window's close
    "submission": "/home/user/submission",
    "snapshot_name": "submission_snapshot",   # inside state_dir
    "snapshot_max_bytes": None,       # None: the offline submission's cap, cell_config's route_gate.max_submission_mb
    "snapshot_max_entries": 20000,
    "snapshot_max_depth": 32,
    "freeze_timeout_s": 300.0,
    "freeze_attempts": 3,             # at the start window's close (a failed copy at a start refuses the start)
    "note": None,
}
LONG_CALLS = ("reference_run",)       # privileged: the reference program runs inside one call (bounded by graded_s)
ERROR_KEYS = frozenset(("error", "detail"))
METHOD_MAX = 64                       # the offline wire's guard (plantcell/wire_guard.py): a method name, plain
ID_STR_MAX = 256

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
    """To the service's account, irreversibly: no supplementary groups, its gid and uid, not dumpable, no core files."""
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
    """Append-only JSON-lines log in the service-only state dir (0600), size-capped (alelab's)."""

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


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def finite(x):
    """A JSON-safe copy with every non-finite float replaced by None (the record is written with allow_nan=False)."""
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if isinstance(x, dict):
        return {str(k): finite(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [finite(v) for v in x]
    if x is None or isinstance(x, (str, bool, int)):
        return x
    if hasattr(x, "tolist"):
        return finite(x.tolist())
    return str(x)


def _jsonable(obj):
    """json.dumps fallback of a reply (the offline wire's): numpy values as plain values, anything else as its str."""
    if hasattr(obj, "tolist"):
        return obj.tolist()
    if isinstance(obj, (set, frozenset)):
        return sorted(obj, key=str)
    return str(obj)


def _iso(t):
    return None if t is None else datetime.datetime.fromtimestamp(t, datetime.timezone.utc).isoformat(
        timespec="seconds").replace("+00:00", "Z")


def is_error(result):
    """An error reply of the cell ({"error": code} or {"error": code, "detail": text}), as the offline wire reads it."""
    return isinstance(result, dict) and "error" in result and set(result) <= ERROR_KEYS


def plain_id(rid):
    """A request id as the reply echoes it (plantcell/wire_guard.py plain_id): null, true / false, a 64-bit integer or a
    short string; anything else is echoed as null."""
    if rid is None or isinstance(rid, bool):
        return rid
    if isinstance(rid, int) and -(1 << 63) <= rid < (1 << 63):
        return rid
    if isinstance(rid, str) and len(rid) <= ID_STR_MAX:
        return rid
    return None


def plain_method(method):
    """The method name as the offline wire hands it to the cell (plantcell/wire_guard.py): a string of at most 64
    characters; anything else becomes "(not a name)", which the cell refuses as `forbidden`."""
    return method[:METHOD_MAX] if isinstance(method, str) else "(not a name)"


def _reject_constant(name):
    raise ValueError("non-finite constant")


def _f(v, default=0.0):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return float(default)
    return v if math.isfinite(v) else float(default)


# ---------------------------------------------------------------------------------------------- the mission's record
END_REASONS = ("done", "finish", "graded_window", "worker_restart")
# what of the cell's mission record (PlantCellServer.finalize, the offline grader's per-mission record) the graded
# record keeps (not the event log, the trajectory columns or the true start pose)
SUMMARY_KEYS = ("cell_version", "site", "kind", "done_reason", "sim_time_s", "sim_time_limit_s", "ticks", "episode_score",
                "raw_score", "R", "parts", "items", "end", "lost", "dose_gy", "dose_fraction", "distance_m",
                "reports_made", "reports_delivered", "reports_lost", "reports", "frames", "camera_frames_total",
                "lidar_sweeps", "thermal_frames", "depth_frames", "event_counts", "blocked_holds", "blocked_hold_s",
                "render_bundle", "illegal_call_count", "internal_errors", "internal_errors_live", "calls", "step_calls",
                "cell_work_s", "render_wall_s", "render_wall_by_sensor_s", "physics_wall_s", "render_failures",
                "frames_lost", "render_retries", "build_wall_s", "final_state_sha256", "model_hash", "reference")
ITEM_KEYS = ("tag", "type", "slot", "weight", "reports", "delivered", "lost", "E", "V", "A", "error", "tolerance",
             "credit", "dwell_s", "max_3s_reading_gyh", "max_3s_reading_near_gyh", "closest_to_reported_m",
             "approach_failed")
EVENT_KINDS = ("blocked", "protective_sit", "robot_lost", "dose_limit", "time_limit", "physics_fault")


def _item_rows(items):
    """The per-item rows of the cell's mission record (PlantCellServer.finalize's, the same keys)."""
    rows = []
    for r in items:
        row = {k: r.get(k) for k in ITEM_KEYS if k in r}
        ev = r.get("evidence") or {}
        if ev:
            row["evidence_ok"] = bool(ev.get("ok"))
            row["evidence_frame"] = ev.get("frame")
            row["evidence_why"] = list(ev.get("why", []))[:6]
        rows.append(row)
    return rows


def interim_summary(srv, score_fn):
    """The mission's record as if the mission ended at this instant, without ending it. A mission whose program stops
    ends in the cell's world.end(): the link's last look at that instant (a report still queued goes out if the link is
    up, else it is lost), then the score of the world as it stands (PlantCellServer.finalize). Here the same is done on
    a copy of the reports: the live mission is not touched (the link test is a ray cast, the end state the true pose)."""
    w = srv.world
    reports = copy.deepcopy(w.reports)
    if not w.done:
        up = bool(w.link_visible()[0])
        for r in reports:
            if r["status"] == "queued":
                if up:
                    r["status"] = "delivered"
                    r["t_sent"] = round(w.time, 4)
                else:
                    r["status"] = "lost"
    end = w.end_state()
    sc = score_fn(w.site, w.prior, reports, w.frames, w.dose_log, end, w.cfg)
    S = float(sc["S"])
    score_error = None
    if not (math.isfinite(S) and 0.0 <= S <= 1.0 + 1e-9):
        score_error = "the mission's score %r is not a finite number in [0, 1]" % (S,)
        S = 0.0
    kinds = {}
    for e in w.events:
        if e.get("type") in EVENT_KINDS:
            kinds[e["type"]] = kinds.get(e["type"], 0) + 1
    frames = {k: int(v) for k, v in w.n_frames.items()}
    return {
        "site": srv.site_name, "kind": srv.kind, "done_reason": w.done_reason or srv.done_reason,
        "sim_time_s": round(float(w.time), 3), "sim_time_limit_s": float(w.sim_time_limit), "ticks": int(w.tick),
        "episode_score": round(S, 6), "raw_score": float(sc["sum_weighted"]), "R": float(sc["R"]),
        "parts": {(r.get("slot") or r["tag"]): float(r["credit"]) for r in sc["items"]},
        "items": _item_rows(sc["items"]), "end": sc["end"], "lost": bool(w.done_reason == "robot_lost"),
        "dose_gy": round(float(w.dose_gy), 6), "dose_fraction": round(float(w.dose_gy / w.rating_gy), 6),
        "distance_m": round(float(w.distance_true), 3), "reports_made": len(reports),
        "reports_delivered": sum(1 for r in reports if r["status"] == "delivered"),
        "reports_lost": sum(1 for r in reports if r["status"] == "lost"),
        "reports": {k: int(v) for k, v in w.n_reports.items()}, "frames": frames,
        "camera_frames_total": frames.get("nav", 0) + frames.get("inspect", 0), "lidar_sweeps": frames.get("lidar", 0),
        "thermal_frames": frames.get("thermal", 0), "depth_frames": frames.get("depth", 0), "event_counts": kinds,
        "blocked_holds": int(getattr(w, "n_holds", 0)), "blocked_hold_s": float(getattr(w, "hold_s", 0.0)),
        "illegal_call_count": int(srv.illegal_calls), "internal_errors": int(srv.internal_errors),
        "internal_errors_live": int(srv.internal_errors_live), "calls": dict(srv.calls),
        "step_calls": int(srv.step_calls), "cell_work_s": float(srv.work_wall),
        "render_wall_s": float(sum(w.render_wall.values())), "physics_wall_s": float(w.physics_wall),
        "render_failures": int(srv.render_failures), "frames_lost": int(srv.frames_lost),
        "build_wall_s": float(srv.build_wall_s), "as_if_ended_now": True, "score_error": score_error,
    }


def final_summary(srv):
    """The cell's own mission record of the ended mission (PlantCellServer.finalize: the offline grader's), the keys of
    SUMMARY_KEYS; the score checked as the offline grader checks it (a finite number in [0, 1])."""
    s = srv.finalize()
    out = {k: s.get(k) for k in SUMMARY_KEYS if k in s}
    out["items"] = list(s.get("items") or [])
    out["as_if_ended_now"] = False
    try:
        sc = float(s.get("episode_score"))
    except (TypeError, ValueError):
        sc = float("nan")
    out["score_error"] = None if (math.isfinite(sc) and 0.0 <= sc <= 1.0) else \
        "the mission's score %r is not a finite number in [0, 1]" % (s.get("episode_score"),)
    return out


def cell_failed(s):
    """The cell's own failures in the mission (tests/verify.py _cell_failed, which runs such a mission again offline):
    internal_error (an exception in the cell's own work while the mission was live), physics_fault (the simulation's
    state became non-finite) and render_failure (a frame the cell could not render; the call answered
    sensor_unavailable, took nothing and could be repeated)."""
    out = []
    if int(_f(s.get("internal_errors_live"))) > 0:
        out.append("internal_error")
    if s.get("done_reason") == "physics_fault":
        out.append("physics_fault")
    if int(_f(s.get("frames_lost"))) > 0:
        out.append("render_failure")
    return out


def episode_metrics(s):
    """The offline grader's per-mission statistics (tests/verify.py on main, the same names, for one mission) of the
    summary `s`. Never a scoring input."""
    m = {"R_mean": _f(s.get("R")), "dose_fraction_mean": _f(s.get("dose_fraction")),
         "distance_m_mean": _f(s.get("distance_m")), "mean_sim_time_s": _f(s.get("sim_time_s")),
         "illegal_call_count": _f(s.get("illegal_call_count")), "step_calls": _f(s.get("step_calls")),
         "cell_internal_errors": _f(s.get("internal_errors")), "cell_work_s_mean": _f(s.get("cell_work_s")),
         "render_wall_s_mean": _f(s.get("render_wall_s")), "build_wall_s_mean": _f(s.get("build_wall_s")),
         "raw_score": _f(s.get("raw_score")), "reports_made": _f(s.get("reports_made")),
         "reports_delivered": _f(s.get("reports_delivered")), "reports_lost": _f(s.get("reports_lost")),
         "render_failures": _f(s.get("render_failures")), "frames_lost": _f(s.get("frames_lost")),
         "blocked_holds": _f(s.get("blocked_holds"))}
    for part, v in sorted((s.get("parts") or {}).items()):
        m["part_mean." + str(part)] = _f(v)
    for k, v in sorted((s.get("frames") or {}).items()):
        m["frames_mean." + str(k)] = _f(v)
    for k, v in sorted((s.get("reports") or {}).items()):
        m["reports." + str(k)] = _f(v)
    for k in EVENT_KINDS:
        m["events." + k] = _f((s.get("event_counts") or {}).get(k, 0))
    if s.get("done_reason"):
        m["end_rate." + str(s.get("done_reason"))] = 1.0
    return m


# ---------------------------------------------------------------------------------------------- the service
class Service:
    """The graded episode behind the socket: one client at a time, the two windows, the record."""

    def __init__(self, cfg, log, reqlog, state, state_path, record_path, server=None, meta=None, score_fn=None,
                 freeze_chan=None):
        self.cfg = cfg
        self.freeze_chan = freeze_chan          # to the root supervisor: the freeze (dual mode)
        self.log, self.reqlog = log, reqlog
        self.state, self.state_path, self.record_path = state, state_path, record_path
        self.server = server                   # plantcell.server.PlantCellServer; None in a restarted worker
        self.meta = dict(meta or {})           # the episode's id, its seed's digest, the oracle token's digest
        self.score_fn = score_fn               # mission.scoring.score (the cell's)
        self.prep_s, self.graded_s = float(cfg["prep_s"]), float(cfg["graded_s"])
        self.min_interval = float(cfg["record_min_interval_s"])
        self.lock = threading.Lock()           # the episode: its requests, its record, its end
        self.active_lock = threading.Lock()
        self.active = False
        self.conn_seq = 0
        self.started = bool(state.get("started"))
        self.t_start = state.get("start_wall")
        self.ended = bool(state.get("ended")) or (self.started and server is None)
        self.end_reason = state.get("end_reason")
        self.t_end = state.get("end_wall")
        self.requests = 0
        self.dirty = False
        self.last_write = 0.0
        self.call = None                       # (monotonic start, method) of the running cell call: the watchdog's
        self.last_record = None                # a restarted worker's view of the last record (status)

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

    def _close_prep(self):
        """(under self.lock) The start window closed without a start (dual mode): the program freezes now, as it would
        have at a start, so the program item keeps main's preparation time; the graded episode can no longer start. A
        copy that fails is tried freeze_attempts times, then recorded as failed: the verifier then scores the program
        item 0 (it never falls back to the session-end submission). The record says so (started false,
        prep_window_closed true, the snapshot's fields)."""
        if self.started or self.state.get("prep_closed"):
            return
        snap = {"status": "error", "detail": "not tried"}
        for attempt in range(max(1, int(self.cfg["freeze_attempts"]))):
            snap = self.freeze()
            if snap.get("status") in ("ok", "too_large"):
                break
            self.log.write(kind="freeze_failed", at="prep_window", attempt=attempt + 1,
                           detail=str(snap.get("detail"))[:200])
            if "frozen already" in str(snap.get("detail")):
                break
            time.sleep(2.0)
        snap = {k: snap.get(k) for k in ("status", "files", "bytes", "sha256", "skipped")}
        snap["at"] = "prep_window"
        self.state.update(prep_closed=True, prep_closed_wall=time.time(), snapshot=snap)
        try:
            self.save_state()
        except OSError:
            self.log.write(kind="state_write_failed")
        rec = {"format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.meta.get("episode"),
               "seed_sha256": self.meta.get("seed_sha256"), "started": False, "ended": False, "end_reason": None,
               "prep_window_closed": True, "prep_window_closed_at": _iso(self.state.get("prep_closed_wall")),
               "requests": 0, "steps": 0, "sim_time_s": 0.0, "wall_s_since_start": 0.0, "episode_score": 0.0,
               "metrics": {}, "privileged": False, "started_at": None, "ended_at": None,
               "windows": {"prep_s": self.prep_s, "graded_s": self.graded_s,
                           "service_start": _iso(self.state.get("service_start"))},
               "score_error": None, "cell_failed": [], "service": SERVICE, "service_version": SERVICE_VERSION,
               "snapshot_status": snap.get("status"), "snapshot_at": "prep_window",
               "snapshot_files": int(snap.get("files") or 0), "snapshot_bytes": int(snap.get("bytes") or 0),
               "snapshot_sha256": snap.get("sha256"), "snapshot_skipped": int(snap.get("skipped") or 0)}
        try:
            atomic_write_json(self.record_path, finite(rec))
        except Exception:  # noqa: BLE001
            self.log.write(kind="record_write_failed", tb=traceback.format_exc(limit=8))
        self.log.write(kind="prep_window_closed", snapshot_status=snap.get("status"), files=snap.get("files"))
        if self.server is not None:
            try:
                self.server.close()           # the sensors' render contexts: the episode can no longer run
            except Exception:  # noqa: BLE001
                pass

    def _frozen_public(self):
        snap = self.state.get("snapshot") or {}
        return {"status": snap.get("status"), "files": int(snap.get("files") or 0),
                "bytes": int(snap.get("bytes") or 0)} if snap else None

    # -- O13: practice ends when the graded episode starts
    def close_practice(self):
        """The flag that ends practice (the practice ground stats it): written once, atomically, world-readable; raises
        OSError when it cannot be written."""
        path = self.cfg["practice_flag"]
        if os.path.lexists(path):
            return
        data = json.dumps({"graded_episode_started": True, "utc": _iso(time.time())}).encode() + b"\n"
        tmp = path + ".tmp"
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o644)
        try:
            os.fchmod(fd, 0o644)
            os.write(fd, data)
            os.fsync(fd)
        finally:
            os.close(fd)
        os.replace(tmp, path)

    # -- persistence
    def save_state(self):
        atomic_write_json(self.state_path, self.state)

    def _record(self, summary):
        score_error = summary.get("score_error")
        try:
            sc = float(summary.get("episode_score", 0.0))
        except (TypeError, ValueError):
            sc = float("nan")
        if not math.isfinite(sc):
            score_error = score_error or "the mission's score is not a finite number (%r)" % (summary.get("episode_score"),)
            sc = 0.0
        episode_score = max(0.0, min(1.0, sc))
        m = episode_metrics(summary)
        m["privileged_mode"] = 1.0 if self.server.privileged else 0.0
        for r in END_REASONS:
            m["end_reason." + r] = 1.0 if (self.ended and self.end_reason == r) else 0.0
        now = time.time()
        rec = {
            "format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.meta.get("episode"),
            "seed_sha256": self.meta.get("seed_sha256"), "started": True, "ended": bool(self.ended),
            "end_reason": self.end_reason if self.ended else None, "requests": int(self.requests),
            "steps": int(self.server.step_calls), "sim_time_s": round(float(self.server.world.time), 4),
            "wall_s_since_start": round(now - float(self.t_start), 3), "episode_score": episode_score, "metrics": m,
            "privileged": bool(self.server.privileged), "started_at": _iso(self.t_start),
            "ended_at": _iso(self.t_end) if self.ended else None,
            "windows": {"prep_s": self.prep_s, "graded_s": self.graded_s,
                        "service_start": _iso(self.state.get("service_start"))},
            "score_error": score_error, "cell_failed": cell_failed(summary), "summary": summary,
            "service": SERVICE, "service_version": SERVICE_VERSION,
        }
        snap = self.state.get("snapshot") or {}        # the program frozen at the start (dual mode)
        rec.update({"snapshot_status": snap.get("status"), "snapshot_at": snap.get("at"),
                    "snapshot_files": int(snap.get("files") or 0), "snapshot_bytes": int(snap.get("bytes") or 0),
                    "snapshot_sha256": snap.get("sha256"), "snapshot_skipped": int(snap.get("skipped") or 0),
                    "prep_window_closed": False})
        return finite(rec)

    def _write_record(self, summary=None):
        try:
            if summary is None:
                summary = interim_summary(self.server, self.score_fn)
            atomic_write_json(self.record_path, self._record(summary))
        except Exception:  # noqa: BLE001 - logged; the next change writes it again
            self.log.write(kind="record_write_failed", tb=traceback.format_exc(limit=8))
        self.last_write = time.time()
        self.dirty = False

    def _log_request(self, method, params, error=None):
        try:
            text = json.dumps(params, default=str)
            args = params if len(text) <= 4096 else text[:4096] + "..."
        except (TypeError, ValueError):
            args = "<unprintable>"
        sim = round(float(self.server.world.time), 4) if self.server is not None and self.server.world is not None \
            else None
        self.reqlog.write(n=self.requests, method=str(method)[:80], params=args, sim_time=sim, error=error)

    def finalize_after_restart(self):
        """A worker started after the episode's start (a restart) reads nothing hidden: the episode ends with its last
        record (end_reason worker_restart; nothing is resumed). Practice stays closed."""
        try:
            rec = read_json(self.record_path, 2 << 20)
            rec = rec if isinstance(rec, dict) else None
        except (OSError, ValueError):
            rec = None
        now = time.time()
        if self.state.get("started"):
            try:
                self.close_practice()
            except OSError:
                self.log.write(kind="practice_flag_failed", tb=traceback.format_exc(limit=4))
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
    def _end(self, reason):
        """End the graded episode (under self.lock) the way the offline grader ends a mission whose program stops at
        this moment: the cell's end of the mission (the link's last look, reports still queued then lost), nothing
        simulated afterwards, the final record (the cell's own mission record) written."""
        if self.ended:
            return
        now = time.time()
        self.ended, self.end_reason, self.t_end = True, reason, now
        srv = self.server
        summary = None
        if srv is not None:
            try:
                if not srv.done:
                    srv._end(reason)
                summary = final_summary(srv)
            except Exception:  # noqa: BLE001 - recorded: the verifier fails the run (a grading failure)
                self.log.write(kind="final_score_failed", tb=traceback.format_exc(limit=12))
                summary = None
        self.state.update(ended=True, end_reason=reason, end_wall=now)
        try:
            self.save_state()
        except OSError:
            self.log.write(kind="state_write_failed")
        if srv is not None:
            if summary is None:
                try:
                    summary = interim_summary(srv, self.score_fn)
                except Exception:  # noqa: BLE001
                    summary = {"episode_score": 0.0}
                summary["score_error"] = "the cell's final mission record could not be computed"
            self._write_record(summary)
            self.log.write(kind="episode_end", reason=reason, sim_time=round(float(srv.world.time), 3),
                           steps=int(srv.step_calls), requests=self.requests)
            try:
                srv.close()                    # the sensors' render contexts
            except Exception:  # noqa: BLE001
                pass

    # -- requests
    @staticmethod
    def _err(rid, code, detail=None):
        msg = {"id": rid, "error": code}
        if detail:
            msg["detail"] = detail
        return msg

    def status(self):
        now = time.time()
        srv = self.server
        if srv is not None and srv.world is not None:
            steps, sim = srv.step_calls, srv.world.time
            frames = {k: int(v) for k, v in srv.world.n_frames.items()}
        else:
            rec = self.last_record or {}
            steps, sim = rec.get("steps", 0), rec.get("sim_time_s", 0.0)
            frames = dict(((rec.get("summary") or {}).get("frames")) or {})
        t0 = float(self.state.get("service_start") or 0.0)
        running = self.started and not self.ended
        return {
            "service": SERVICE, "started": bool(self.started), "ended": bool(self.ended),
            "end_reason": self.end_reason if self.ended else None, "steps": int(steps or 0),
            "sim_time": round(float(sim or 0.0), 4), "frames_used": frames,
            "start_window_s": self.prep_s, "graded_window_s": self.graded_s,
            "start_window_left_s": None if (self.started or self.ended or self.state.get("prep_closed"))
            else round(max(0.0, t0 + self.prep_s - now), 1),
            "graded_window_left_s": round(max(0.0, float(self.t_start) + self.graded_s - now), 1) if running else None,
            "program_frozen": self._frozen_public() if self.state.get("snapshot") else None,
        }

    def _start(self, rid, params):
        with self.lock:
            if self.started:
                return self._err(rid, "already_started", "the graded episode was started at %s; it starts once per "
                                                         "session" % _iso(self.t_start))
            if self.server is None or self.ended or self.state.get("prep_closed"):
                return self._err(rid, "start_window_closed", "the graded episode can no longer be started (the "
                                                             "program was frozen when the start window closed)"
                                 if self.state.get("prep_closed") else "the graded episode can no longer be started")
            if not isinstance(params, dict):
                return self._err(rid, "bad_params", "start takes no parameters")
            privileged = False
            if params:
                token = params.get("oracle_token") if list(params) == ["oracle_token"] else None
                want = self.meta.get("oracle_token_sha256")
                if isinstance(token, str) and want and \
                        hashlib.sha256(token.strip().encode("utf-8")).hexdigest() == want:
                    privileged = True          # the oracle (solution/): the privileged calls, as offline
                else:
                    return self._err(rid, "bad_params", "start takes no parameters")
            now = time.time()
            if now - float(self.state.get("service_start") or 0.0) > self.prep_s:
                self._log_request("start", {}, error="start_window_closed")
                return self._err(rid, "start_window_closed", "the graded episode had to be started within %.0f s of "
                                                             "the service's start" % self.prep_s)
            # the freeze (dual mode): the program in the submission folder, copied by the root supervisor before the
            # charge; a copy that fails refuses the start and charges nothing (over the cap: too_large, started anyway)
            snap = self.freeze()
            if snap.get("status") not in ("ok", "too_large"):
                self.log.write(kind="freeze_failed", detail=str(snap.get("detail"))[:200])
                self._log_request("start", {}, error="freeze_failed")
                return self._err(rid, "internal_error", "the program in /home/user/submission could not be frozen; "
                                                        "nothing was started")
            snap = {k: snap.get(k) for k in ("status", "files", "bytes", "sha256", "skipped")}
            snap["at"] = "graded_start"
            # O13: practice ends when the graded episode starts; no start without the flag that ends it
            try:
                self.close_practice()
            except OSError:
                self.log.write(kind="practice_flag_failed", tb=traceback.format_exc(limit=4))
                self._log_request("start", {}, error="practice_flag_failed")
                return self._err(rid, "internal_error", "the start could not be recorded; nothing was started")
            # the charge: persisted before anything is simulated, never refunded
            self.state.update(started=True, start_wall=now, privileged=privileged, snapshot=snap)
            try:
                self.save_state()
            except OSError:
                self.state.update(started=False, start_wall=None, privileged=False, snapshot=None)
                self.log.write(kind="charge_persist_failed", tb=traceback.format_exc(limit=8))
                return self._err(rid, "internal_error", "the start could not be recorded; nothing was started")
            self.started, self.t_start = True, now
            self.server.privileged = privileged
            self.server.wall_start = now
            self.requests += 1
            self._log_request("start", {"privileged": True} if privileged else {})
            self._write_record()
            self.log.write(kind="episode_start", privileged=privileged)
            return {"id": rid, "result": {"started": True, "sim_time": round(float(self.server.world.time), 4),
                                          "graded_window_s": self.graded_s, "program_frozen": self._frozen_public()}}

    def _cell_call(self, rid, method, params):
        with self.lock:
            if not self.started:
                return self._err(rid, "not_started", "the graded episode has not started: call start() first")
            if not self.ended and time.time() - float(self.t_start) >= self.graded_s:
                self._end("graded_window")
            if self.ended or self.server is None:
                if method == "finish":
                    return {"id": rid, "result": {"ok": True, "ended": True}}
                return self._err(rid, "episode_over", "the graded episode is over (%s)" % self.end_reason)
            srv = self.server
            self.requests += 1
            self.call = (time.monotonic(), method)
            try:
                result = srv.handle(method, params)
            finally:
                self.call = None
            srv.wall_used = time.time() - float(self.t_start)
            srv.program_used = srv.wall_used - float(srv.work_wall)
            self._log_request(method, params, error=result.get("error") if is_error(result) else None)
            if srv.done:
                self._end("finish" if srv.done_reason == "finish" else "done")
            else:
                self.dirty = True
                if time.time() - self.last_write >= self.min_interval:
                    self._write_record()
        if is_error(result):
            msg = {"id": rid, "error": result["error"]}
            if "detail" in result:
                msg["detail"] = result["detail"]
            return msg
        return {"id": rid, "result": result}

    def handle_request(self, req):
        rid = plain_id(req.get("id"))
        method = req.get("method")
        params = req.get("params")
        if params is None:
            params = {}                        # anything else that is not an object is the cell's bad_params
        if method == "status":
            if params != {}:
                return self._err(rid, "bad_params", "status takes no parameters")
            return {"id": rid, "result": self.status()}
        if method == "start":
            return self._start(rid, params)
        return self._cell_call(rid, plain_method(method), params)

    def flush_record(self):
        """A pending change of the episode written now (a disconnect: the record is current when a session ends)."""
        if not self.dirty or self.server is None:
            return
        if self.lock.acquire(timeout=5.0):
            try:
                if self.dirty and self.started and not self.ended:
                    self._write_record()
            finally:
                self.lock.release()

    # -- threads
    def ticker(self):
        """The start window's close, the graded window's end, and a pending record write a quarter of a second after
        the last change."""
        while True:
            time.sleep(0.25)
            try:
                if (self.server is not None and not self.started and not self.state.get("prep_closed")
                        and time.time() - float(self.state.get("service_start") or 0.0) >= self.prep_s):
                    with self.lock:
                        if not self.started:
                            self._close_prep()          # the program freezes at the start window's close
                    continue
                if self.server is None or not self.started or self.ended:
                    continue
                if time.time() - float(self.t_start) >= self.graded_s:
                    with self.lock:
                        if not self.ended:
                            self._end("graded_window")
                elif self.dirty and time.time() - self.last_write >= 0.25:
                    if self.lock.acquire(timeout=0.2):
                        try:
                            if self.dirty and not self.ended:
                                self._write_record()
                        finally:
                            self.lock.release()
            except Exception:  # noqa: BLE001
                self.log.write(kind="ticker_error", tb=traceback.format_exc(limit=8))

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

    def serve(self, listener):
        threading.Thread(target=self.ticker, name="plantgrade-tick", daemon=True).start()
        threading.Thread(target=self.watchdog, name="plantgrade-dog", daemon=True).start()
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
            threading.Thread(target=self.handle, args=(conn, cid), name="plantgrade-conn", daemon=True).start()

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
                if not isinstance(req, dict) or "method" not in req:
                    reply = self._err(None, "bad_request", "a request is one JSON object {\"id\": n, \"method\": "
                                                           "name, \"params\": {...}} on one line")
                else:
                    try:
                        reply = self.handle_request(req)
                    except Exception:  # noqa: BLE001
                        self.log.write(kind="internal_error", conn=cid, tb=traceback.format_exc(limit=20))
                        reply = self._err(plain_id(req.get("id")), "internal_error")
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
            try:
                self.flush_record()
            except Exception:  # noqa: BLE001
                self.log.write(kind="record_flush_failed", tb=traceback.format_exc(limit=8))
            with self.active_lock:
                self.active = False
            self.log.write(kind="disconnect", conn=cid)


# ---------------------------------------------------------------------------------------------- worker
def check_cell(cfg):
    """Every file of the cell (cell_root/plantcell: the grader's tests/plantcell) and of the privileged reference
    program (cell_root/reference: tests/reference) is the one cell_manifest.json lists, byte for byte, and nothing else
    is there (no link, no other file, no __pycache__). Raises RuntimeError otherwise."""
    manifest = read_json(cfg["cell_manifest"], 1 << 20)
    problems = []
    for group in ("plantcell", "reference"):
        want = manifest.get(group)
        root = os.path.join(cfg["cell_root"], group)
        if not isinstance(want, dict) or not want:
            raise RuntimeError("cell_manifest.json lists no files of %s" % group)
        seen = set()
        for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
            for name in list(dirnames):
                if os.path.islink(os.path.join(dirpath, name)):
                    problems.append("a link: %s" % os.path.join(dirpath, name))
                    dirnames.remove(name)
            for name in filenames:
                p = os.path.join(dirpath, name)
                rel = os.path.relpath(p, root).replace(os.sep, "/")
                st = os.lstat(p)
                if not stat.S_ISREG(st.st_mode):
                    problems.append("not a regular file: %s/%s" % (group, rel))
                elif rel not in want:
                    problems.append("not in the manifest: %s/%s" % (group, rel))
                elif sha256_file(p) != want[rel]:
                    problems.append("differs: %s/%s" % (group, rel))
                seen.add(rel)
        missing = sorted(set(want) - seen)
        if missing:
            problems.append("missing in %s: %s" % (group, ", ".join(missing[:5])))
    if problems:
        raise RuntimeError("the cell is not the grader's: " + "; ".join(problems[:8]))


def run_worker(cfg, listener, ready_fd, lab, freeze_chan=None):
    """Forked from the supervisor as root: read the hidden mission and build it (only while the episode has not
    started), drop privileges, serve."""
    _set_comm("plantgrade-wkr")
    os.umask(0o077)
    sys.dont_write_bytecode = True
    state_dir = cfg["state_dir"]
    log = Log(os.path.join(state_dir, "service.log"), int(cfg["max_log_bytes"]))
    log.chown(lab.pw_uid, lab.pw_gid)
    try:
        for k, v in cfg["env"].items():          # the grader's numerics and rendering (before numpy is imported)
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
            meta = read_json(cfg["episode"], 1 << 16)            # the hidden graded mission (root-only)
            seed = int(meta["seed"])
            if hashlib.sha256(str(seed).encode("ascii")).hexdigest() != meta["seed_sha256"]:
                raise ValueError("the graded episode's seed does not match its digest")
            package = os.path.join(cfg["package_dir"], os.path.basename(str(meta["package"])))
            if sha256_file(package) != meta["package_sha256"]:
                raise ValueError("the graded episode's site package does not match its digest")
            check_cell(cfg)
            sys.path.insert(0, cfg["cell_root"])
            import plantcell.server as cell_server            # noqa: WPS433 (the grader's cell, checked above)
            from mission.scoring import score as score_mission  # noqa: WPS433 (the cell's own score function)
            cell_cfg = read_json(cfg["cell_config"], 1 << 16)
            server = cell_server.PlantCellServer(config=cell_cfg, mode="hidden", privileged=False,
                                                 log=lambda s: log.write(kind="cell", msg=str(s)[:8000]),
                                                 reference_dir=os.path.join(cfg["cell_root"], "reference"))
            server.reset(package, seed=seed, kind="hidden")       # the grader's build of the mission (as root)
            public = {"episode": str(meta["episode"]), "seed_sha256": str(meta["seed_sha256"]),
                      "oracle_token_sha256": meta.get("oracle_token_sha256")}
            del meta, seed
            svc = Service(cfg, log, reqlog, state, state_path, record_path, server=server, meta=public,
                          score_fn=score_mission, freeze_chan=freeze_chan)
            drop_privileges(lab)
            log.write(kind="mission_built", site=server.site_name, build_wall_s=round(float(server.build_wall_s), 2),
                      render_bundle=getattr(server.world, "render_bundle", None))
        sec_ok = seccomp_selftest()
        log.write(kind="worker_ready", uid=os.getuid(), gid=os.getgid(), seccomp_inet_blocked=sec_ok,
                  mode="episode" if svc.server is not None else "ended", started=bool(svc.started))
        if cfg["require_seccomp"] and not sec_ok:
            raise RuntimeError("seccomp self-test failed: an AF_INET socket could be created")
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


def _copy_tree(src_fd, dst_dir, rel, acc, caps, depth):
    """Copy the regular files and directories of the agent's account under the open directory `src_fd` into
    `dst_dir`. Every name is resolved relative to an open directory descriptor with O_NOFOLLOW: no symbolic link is
    followed and no path the agent controls is resolved by name; symbolic links, special files, entries of other
    accounts, __pycache__ and odd names are skipped."""
    with os.scandir(src_fd) as it:
        names = sorted(e.name for e in it)
    for name in names:
        if not _plain_name(name):
            acc["skipped"] += 1
            continue
        try:
            st = os.stat(name, dir_fd=src_fd, follow_symlinks=False)
        except OSError:
            continue                                    # gone meanwhile
        r = name if not rel else rel + "/" + name
        acc["entries"] += 1
        if acc["entries"] > caps["max_entries"]:
            raise _TooLarge("more than %d entries" % caps["max_entries"])
        if caps["owner_uid"] is not None and st.st_uid != caps["owner_uid"]:
            acc["skipped"] += 1                         # not the agent's
            continue
        if stat.S_ISDIR(st.st_mode):
            if name == "__pycache__" or depth >= caps["max_depth"]:
                acc["skipped"] += 1
                continue
            try:
                sub = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=src_fd)
            except OSError:
                acc["skipped"] += 1                     # replaced by a link or a file meanwhile
                continue
            try:
                if caps["owner_uid"] is not None and os.fstat(sub).st_uid != caps["owner_uid"]:
                    acc["skipped"] += 1
                    continue
                d = os.path.join(dst_dir, name)
                os.mkdir(d, 0o700)
                _copy_tree(sub, d, r, acc, caps, depth + 1)
            finally:
                os.close(sub)
        elif stat.S_ISREG(st.st_mode):
            try:
                fd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC, dir_fd=src_fd)
            except OSError:
                acc["skipped"] += 1
                continue
            try:
                fst = os.fstat(fd)
                if not stat.S_ISREG(fst.st_mode) or (caps["owner_uid"] is not None
                                                     and fst.st_uid != caps["owner_uid"]):
                    acc["skipped"] += 1
                    continue
                out = os.open(os.path.join(dst_dir, name), os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW
                              | os.O_CLOEXEC, 0o600)
                h = hashlib.sha256()
                size = 0
                try:
                    while True:
                        chunk = os.read(fd, 1 << 20)
                        if not chunk:
                            break
                        size += len(chunk)
                        acc["bytes"] += len(chunk)
                        if acc["bytes"] > caps["max_bytes"]:
                            raise _TooLarge("more than %d bytes" % caps["max_bytes"])
                        h.update(chunk)
                        view = memoryview(chunk)
                        while view:
                            view = view[os.write(out, view):]
                finally:
                    os.close(out)
                acc["files"].append((r, size, h.hexdigest()))
            finally:
                os.close(fd)
        else:
            acc["skipped"] += 1                         # a symbolic link, a FIFO, a socket, a device


def snapshot_digest_of(files):
    """The frozen program's digest (tests/verify.py recomputes it): for every file in the sorted order of its relative
    path, the line `<path>\\0<size>\\0<sha256 of its content>\\n`; the sha256 of all the lines."""
    total = hashlib.sha256()
    for rel, size, digest in sorted(files, key=lambda f: f[0].encode("utf-8")):
        total.update(rel.encode("utf-8") + b"\0" + str(size).encode() + b"\0" + digest.encode() + b"\n")
    return total.hexdigest()


def snapshot_submission(cfg, owner_uid):
    """(root, the supervisor; dual mode) The freeze: a root-owned copy (directories 0700, files 0600) of the agent's
    submission folder in the record directory. Within the offline submission's caps the copy is complete (status
    ok); beyond them nothing is copied (status too_large); a copy that fails is status error. The copy is built under
    a temporary name and renamed into place. Never reads anything hidden."""
    state_dir = cfg["state_dir"]
    dst = os.path.join(state_dir, cfg["snapshot_name"])
    tmp = dst + ".tmp"
    caps = {"max_bytes": int(cfg["snapshot_max_bytes"]), "max_entries": int(cfg["snapshot_max_entries"]),
            "max_depth": int(cfg["snapshot_max_depth"]), "owner_uid": owner_uid}
    acc = {"files": [], "bytes": 0, "entries": 0, "skipped": 0}
    old = os.umask(0o077)
    try:
        for p in (tmp, dst):
            if os.path.lexists(p):
                if os.path.isdir(p) and not os.path.islink(p):
                    shutil.rmtree(p)
                else:
                    os.unlink(p)
        os.mkdir(tmp, 0o700)
        try:
            src_fd = os.open(cfg["submission"], os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
        except FileNotFoundError:
            src_fd = None                               # no submission folder: an empty program
        try:
            if src_fd is not None and owner_uid is not None and os.fstat(src_fd).st_uid != owner_uid:
                raise RuntimeError("the submission folder is not the agent's")
            if src_fd is not None:
                _copy_tree(src_fd, tmp, "", acc, caps, 0)
        except _TooLarge as exc:
            shutil.rmtree(tmp, ignore_errors=True)
            return {"status": "too_large", "files": len(acc["files"]), "bytes": int(acc["bytes"]), "sha256": None,
                    "skipped": int(acc["skipped"]), "detail": str(exc)}
        finally:
            if src_fd is not None:
                os.close(src_fd)
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
        os.umask(old)


def serve_freeze(chan, cfg, sup_log, owner_uid):
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
                except (OSError, ValueError, AttributeError):
                    done = True                         # fail closed
                if done:
                    res["detail"] = "the program is frozen already"
                else:
                    res = snapshot_submission(cfg, owner_uid)
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
    for key in ("episode", "package_dir", "cell_manifest", "cell_config"):
        cfg[key] = os.path.normpath(os.path.join(base, cfg[key]))
    if not isinstance(cfg["task"], str) or not cfg["task"]:
        raise ValueError("config needs the task's slug")
    for key in ("socket", "state_dir", "practice_flag", "cell_root", "submission"):
        if not os.path.isabs(cfg[key]):
            raise ValueError("%s must be an absolute path" % key)
    if os.path.dirname(cfg["socket"]) in ("/", "/tmp", "/var/tmp", "/dev/shm", "/run"):
        raise ValueError("the socket must live in its own root-owned directory")
    if os.path.dirname(os.path.dirname(cfg["practice_flag"])) != os.path.dirname(cfg["socket"]):
        raise ValueError("the practice flag's directory must sit in the socket's root-owned directory")
    if not isinstance(cfg["env"], dict) or not all(isinstance(k, str) and isinstance(v, str)
                                                   for k, v in cfg["env"].items()):
        raise ValueError("env must map strings to strings")
    if cfg["snapshot_max_bytes"] is None:       # the offline submission's size cap (main's grader configuration)
        rg = read_json(cfg["cell_config"], 1 << 16).get("route_gate", {})
        cfg["snapshot_max_bytes"] = int(float(rg.get("max_submission_mb", 60)) * 1024 * 1024)
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
        raise RuntimeError("the service's account must not be root")
    try:
        agent = pwd.getpwnam(cfg["agent_user"])
    except KeyError:
        agent = None
    if agent is not None and agent.pw_uid == lab.pw_uid:
        raise RuntimeError("the service's account must differ from the agent account")
    return lab, (agent.pw_uid if agent is not None else None)


def _close_fds(keep):
    for fd in range(3, 1024):
        if fd not in keep:
            try:
                os.close(fd)
            except OSError:
                pass


def run_supervisor(cfg, lab, owner_uid, notify_fd):
    _set_comm("plantgrade-sup")
    os.umask(0o077)
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
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
        freezer = threading.Thread(target=serve_freeze, args=(sup_chan, cfg, sup_log, owner_uid),
                                   name="plantgrade-freeze", daemon=True)
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


def start(config_path):
    if os.geteuid() != 0:
        print("plant-graded: must be started as root (environment/setup/setup.sh)", file=sys.stderr)
        return 2
    try:
        cfg = load_config(config_path)
        lab, owner_uid = _accounts(cfg)
        os.umask(0o077)
        sockdir = os.path.dirname(cfg["socket"])
        os.makedirs(os.path.dirname(sockdir), exist_ok=True)
        _secure_dir(sockdir, 0o755, 0, 0)
        # O13: the practice flag's directory, writable by the service's account only, readable by everyone
        _secure_dir(os.path.dirname(cfg["practice_flag"]), 0o755, lab.pw_uid, lab.pw_gid)
        os.makedirs(os.path.dirname(cfg["state_dir"]), exist_ok=True)
        _secure_dir(cfg["state_dir"], 0o700, lab.pw_uid, lab.pw_gid)
        state_path = os.path.join(cfg["state_dir"], "state.json")
        if not os.path.lexists(state_path):    # the service's start: the start window counts from here (never reset)
            if os.path.lexists(cfg["practice_flag"]):
                os.unlink(cfg["practice_flag"])  # a fresh service: nothing has started, practice is open
            atomic_write_json(state_path, {"version": 1, "service_start": time.time(), "started": False,
                                           "start_wall": None, "ended": False, "end_reason": None})
            os.chown(state_path, lab.pw_uid, lab.pw_gid)
    except Exception as exc:  # noqa: BLE001
        print("plant-graded: configuration error: %s (%s)" % (type(exc).__name__, str(exc)[:200]), file=sys.stderr)
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
            run_supervisor(cfg, lab, owner_uid, w)
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
        print("plant-graded: ready (socket %s)" % cfg["socket"], file=sys.stderr)
        return 0
    print("plant-graded: failed to start (details in the service's own log)", file=sys.stderr)
    return 1


def main(argv=None):
    ap = argparse.ArgumentParser(description="plant-graded (start as root from environment/setup/setup.sh)")
    ap.add_argument("--config", required=True)
    args = ap.parse_args(argv)
    return start(args.config)


if __name__ == "__main__":
    sys.exit(main())
