#!/usr/bin/env python3
"""rov-cell-online: the cell service of rov_final_assembly_and_trim_single_arm's graded episode (dual mode,
experimental).

The agent drives ONE hidden graded episode of the station's cell itself, during its session, through this service. The
service writes the episode's outcome to /var/lib/arb-online (a declared artifact the agent can neither read nor write)
and the separate verifier scores that record (tests/verify.py). The cell is the offline verifier's own: online/cell is a
byte copy of the cell's part of tests/ (arm/, common/, render/, render_filament/, room/, episode_trace.py and the cell's
data files: cell_layout.json, fixture_facts.json, trim_table.json, trim_sheet_printed.json, cell_public.json, and
grader_config.json, the cell's configuration), laid out as tests/ is, so the scene, the episode logic, the per-episode
limits, the safety watch and the score function are main's. The cell's hidden kit (gt/: the units' meshes and collision
pieces) is the verifier's too: environment/setup/setup.sh fetches the same pinned tarball (online/fetch_kit.py).

Started once by environment/setup/setup.sh as root, cwd = the setup stage (/opt/ale/setup, root-only while the agent
works):

    env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \\
        /opt/venv/bin/python3 -I -B online/service.py --config online/online.json

The command returns once the service serves (exit 0) or failed to start (exit 1); the service itself is a detached
daemon (setsid, stdio on /dev/null) that lives into the agent phase. The layout is arm_hidden_mechanism_box's alelab
(environment/setup/lab/alelab/server.py there):
  supervisor (root, comm "arbcell-sup"): binds the unix socket, writes the pid file, forks the worker and restarts it at
      most max_restarts times, and freezes the agent's program on the worker's request (below). It never reads the
      hidden episode and never accepts a connection.
  worker (comm "arbcell-worker"): forked as root. While the graded episode has not started it reads the hidden episode
      (online/episode.json: one episode of the fixed hidden set, root-only), imports the cell from its copy
      (/opt/arb-online/cell, root:arbcell 0750: the agent cannot read it), builds the episode's cell exactly as the
      offline grader does (the kit, the trim table and the configuration checked as verify.py checks them; the layout
      digest compared with the hidden set's) and only then drops to the lab account irreversibly (setgroups([]),
      setgid, setuid; not dumpable; no core files) and serves. What the cell reads later (the camera session's meshes
      and scenery, the reference program's module) it reads as the lab account from that copy, the kit
      (/opt/arb-online/gt) and the image's task assets. A worker that finds the episode started (a restart) reads
      nothing hidden: the episode ends with its last record (end_reason worker_restart, no resume) and the worker
      answers `status` only.
  Both run under PR_SET_NO_NEW_PRIVS and a seccomp filter under which socket() / socketpair() work only for AF_UNIX.

The graded episode:
  * `start` charges it once per session: the charge is persisted (state.json, fsync) before anything is simulated and is
    never refunded. A start is accepted only within prep_s of the service's start.
  * The freeze (dual mode): the program freezes at the earlier of the graded start and the start window's close
    (prep_s after the service's start). Before a start is charged, or when the window closes without a start, the
    worker asks the supervisor, over the socketpair it holds since its fork, to copy /home/user/submission into
    submission_snapshot in the record directory (root-owned 0700 / 0600; regular files and directories only, no
    symbolic link followed, special files and __pycache__ skipped; within the offline submission's size cap, the cell
    configuration's route_gate.max_submission_mb, else nothing is copied: too_large). The supervisor answers only this
    one request, with fixed paths, and only while the state file says the program has not been frozen; a copy that
    fails refuses the start and charges nothing. The record carries snapshot_status, snapshot_at (graded_start |
    prep_window), snapshot_files, snapshot_bytes and snapshot_sha256; a window that closes without a start writes a
    record of its own (started false, prep_window_closed true) and no start is accepted afterwards. A window-close copy
    that fails after its retries is recorded as such (snapshot_status error): the verifier then scores the program 0
    and never grades the session-end submission instead.
  * The episode lives in the worker, not in a connection: a disconnect leaves it where it is and a new connection
    resumes it. One client at a time (a second connection is told `busy`). A connection idle for idle_timeout_s is
    closed; the episode stays.
  * The cell's rules are the offline episode's: 1200 s of simulated time, 600 camera frames, step(n) clamped to 500
    ticks, 4 turns of the positioner, the safety watch, the fastening and the scrap rules (the cell's own code). Not
    carried over: the program's wall-clock cap (2400 s) and its 120 s silence rule (the offline wire's), which bound a
    program, not this episode; the episode has the graded window instead.
  * It ends at its own end (the cell's simulated-time cap: `done`; an exception inside the cell ends it as the offline
    wire's wire_error does: `done`), at `finish`, graded_s after its start (`graded_window`), or when the session ends
    (the sandbox is frozen or destroyed: the last record applies). It ends the way the offline grader ends an episode
    whose program stops: nothing is simulated afterwards and the state is scored as it stands (the cell's finalize()).
  * The record (record.json, written atomically: a temp file, fsync, rename) is rewritten after a request that changes
    the episode, at most every record_min_interval_s (a pending change is written within about a quarter of a second
    of the last request), and always when the episode starts and ends; requests.jsonl logs every request of the episode
    (method, arguments, simulated time; no images; the oracle token never) up to requests_log_max_bytes. Its
    episode_score is the offline per-episode score as if the episode ended now: the stage score x hard_ok (verify.py's
    assembly_score of one episode before the contract gate).
  * The offline verifier's replay record of the episode (episode_trace.EpisodeTrace: joint states, events, the camera
    frames the agent received within the trace's own limits, at most 200 MB) is written to trace/graded/ in the record
    directory when the episode ends; it is never a scoring input (none when the session ends first).
  * The client learns the cell's replies (the offline wire's) and `status` (started / ended, steps, simulated time,
    camera frames, time left in each window); never the score, the record, the episode's id or its seed.

Protocol: one JSON object per line in each direction, in the offline wire's envelope:
    {"id": n, "method": m, "params": {...}}  ->  {"id": n, "result": ...}  |  {"id": n, "error": code[, "detail": text]}
On connect the service sends one greeting line, {"service": SERVICE}, or {"service": SERVICE, "error": "busy", ...} and
closes. Methods: `status`, `start`, and the cell's own (describe, get_state, set_control, command, step, fixture_index,
get_camera, finish; the privileged get_state_privileged, describe_privileged and reference_assemble only on an episode
started with the oracle token, as offline; anything else answers {"error": "forbidden"} and is counted, as offline).
The service's own errors: bad_request, bad_params, line_too_long, not_started, already_started, start_window_closed,
episode_over, internal_error.

Dependencies: the Python 3.12 standard library for the supervisor; the worker imports the cell (numpy, mujoco, PIL,
the Filament module).
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

SERVICE = "rov-cell-online/1"
SERVICE_VERSION = "1"
RECORD_FORMAT = "arb-online-record/1"
DEFAULTS = {
    "task": None,                     # the task's slug (the record's `task`)
    "episode": "episode.json",        # the hidden graded episode (relative to the config file; root-only)
    "cell_dir": "/opt/arb-online/cell",   # the cell's copy laid out as tests/ (root:<user> 0750; setup.sh builds it)
    "kit_dir": "/opt/arb-online/gt",      # the cell's hidden kit (root:<user> 0750; online/fetch_kit.py fetches it)
    "robot_dir": "/opt/task-assets/payload/given/robot",   # the given robot models, from the image (root-owned)
    "render_dir": "/opt/task-assets/payload/render",       # the cameras' scenery, from the image (root-owned)
    "asset_root": "/opt/task-assets/payload",              # checked against given_hashes.json (below)
    "given_hashes": "given_hashes.json",   # tests/given_hashes.json's byte copy (relative to cell_dir)
    "work_dir": "/var/lib/arb-online-work",   # the lab account's private scratch (TMPDIR, caches); not an artifact
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
    "max_restarts": 3,
    "start_timeout_s": 600.0,
    "require_seccomp": True,
    "trace": True,                    # the offline verifier's replay record of the episode (episode_trace.py)
    "env": {},                        # PUBLIC constants only (numeric pins, renderer); never hidden values
    # the freeze (dual mode): the root supervisor copies this folder before the graded start is charged
    "submission": "/home/user/submission",
    "snapshot_name": "submission_snapshot",   # inside state_dir
    "snapshot_max_bytes": None,       # None: the offline submission's cap, the cell configuration's route_gate
    "snapshot_max_entries": 20000,
    "snapshot_max_depth": 32,
    "freeze_timeout_s": 300.0,
    "freeze_retries": 3,              # a window-close copy that fails is tried this many times in all
}
# the cell's modules imported as root before the drop (the rest, e.g. the reference program, as the lab account later)
CELL_MODULES = ("arm.assembly_rov.server", "arm.assembly_rov.cell", "arm.assembly_rov.fixtures",
                "arm.assembly_rov.trim_eval", "arm.assembly_rov.layout", "arm.assembly_rov.positioner",
                "arm.assembly_rov.render_twin", "arm.assembly_rov.ur_controller", "arm.assembly_rov.ur_robot",
                "arm.assembly_rov.gripper_wide", "arm.assembly_rov.coupling", "arm.assembly_rov.agent_render",
                "episode_trace")
# privileged: the reference program runs inside one call (bounded by graded_s), as offline inside its wall cap
LONG_CALLS = ("reference_assemble",)
SERVICE_METHODS = ("status", "start")

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
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if isinstance(x, dict):
        return {str(k): finite(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [finite(v) for v in x]
    if x is None or isinstance(x, (str, bool, int)):
        return x
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
    """An error reply of the cell, as the offline wire reads it (arm/assembly_cell/wire.py serve_episode): a dict whose
    only key is "error"; it goes out as {"id": n, "error": ...}, everything else as {"id": n, "result": ...}."""
    return isinstance(result, dict) and "error" in result and len(result) == 1


def _reject_constant(name):
    raise ValueError("non-finite constant")


# ---------------------------------------------------------------------------------------------- the episode's record
END_REASONS = ("done", "finish", "graded_window", "worker_restart")
# kept out of the record's summary: the long logs and tables (the replay record has them), and the seed (it stays in
# online/episode.json; the record carries its digest)
SUMMARY_DROP = ("events", "coupling_events", "mates", "sites", "cell_metrics", "trim_eval", "score", "seed")


def _f(v, default=0.0):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return float(default)
    return v if math.isfinite(v) else float(default)


def episode_score_of(s):
    """The offline per-episode score of one episode's summary (tests/verify.py on main: assembly_score's term
    `stage x hard_ok`; the contract gate is the program's and does not apply to the graded episode), on [0, 1]."""
    return max(0.0, min(1.0, _f(s.get("stage_score")) * (1.0 if s.get("hard_ok") else 0.0)))


def episode_metrics(s):
    """The offline verifier's statistics of one episode (tests/verify.py's episode_metrics on main for a list of this
    one episode: the same names, `ep1.` for its per-episode block) from the episode's summary `s`, the cell's
    finalize() as it stands now. Never a scoring input."""
    te = s.get("trim_eval") or {}
    watch = s.get("watch") or {}
    weld = ((s.get("cell_metrics") or {}).get("weld") or {})
    m = {
        "stage_score_mean_ungated": _f(s.get("stage_score")), "mates_achieved_mean": _f(s.get("n_achieved")),
        "mates_fastened_mean": _f(s.get("n_fastened")), "all_mates_rate": 1.0 if s.get("all_mates") else 0.0,
        "locked_off_mean": float(len(s.get("locked_off") or [])),
        "scrapped_units_mean": float(len(s.get("scrapped") or [])),
        "trim_mean": _f(s.get("trim")), "hard_violation_rate": 0.0 if s.get("hard_ok") else 1.0,
        "robot_contact_episodes": 1.0 if s.get("robot_contact_violations") else 0.0,
        "part_force_episodes": 1.0 if s.get("part_force_violations") else 0.0,
        "mean_sim_s": _f(s.get("sim_time_s")), "mean_episode_wall_s": _f(s.get("episode_wall_s")),
        "max_part_force_sustained_N": _f(watch.get("max_part_force_sustained_N")),
        "max_robot_link_force_N": _f(watch.get("max_robot_link_force_N")),
        "max_flange_force_sustained_N": _f(watch.get("max_flange_force_sustained_N")),
        "max_dome_force_sustained_N": _f(watch.get("max_dome_force_sustained_N")),
        "weld_F_sustained_max_N": _f(weld.get("weld_F_sustained_max_N")),
        "turns_mean": _f(s.get("turns_used")), "camera_frames_mean": _f(s.get("camera_frames_total")),
        "render_wall_s_mean": _f(s.get("render_wall_s")), "render_failures": _f(s.get("render_failures")),
        "steps_clamped": _f(s.get("steps_clamped")),
        "fixture_refused": _f((s.get("fixture_calls") or {}).get("refused")),
        "illegal_call_count": _f(s.get("illegal_call_count")), "step_calls": _f(s.get("step_calls")),
        "reference_calls": _f(s.get("reference_calls")),
    }
    for cam in ("top", "wrist"):
        m["camera_frames_mean." + cam] = _f((s.get("camera_frames") or {}).get(cam))
    p = "ep1."
    m[p + "stage"] = _f(s.get("stage_score"))
    m[p + "hard_ok"] = 1.0 if s.get("hard_ok") else 0.0
    m[p + "trim"] = _f(s.get("trim"))
    m[p + "net_buoyancy_N"] = _f(te.get("dB_N"))
    m[p + "pitch_deg"] = _f(te.get("theta_deg"))
    m[p + "roll_deg"] = _f(te.get("phi_deg"))
    m[p + "ballast_fastened"] = _f(te.get("n_ballast"))
    m[p + "mates"] = _f(s.get("n_achieved"))
    m[p + "locked_off"] = float(len(s.get("locked_off") or []))
    m[p + "scrapped"] = float(len(s.get("scrapped") or []))
    m[p + "illegal_calls"] = _f(s.get("illegal_call_count"))
    for mid in sorted((s.get("weights") or {})):
        hit = any(r.get("id") == mid and r.get("achieved") for r in (s.get("mates") or []))
        m["mate_rate." + mid] = 1.0 if hit else 0.0
    return m


def summary_now(srv):
    """The cell's finalize() as it stands, without ending the episode: the same summary the offline grader takes when a
    program stops now (finalize ends the episode and closes the camera session; here both are undone or skipped, so the
    episode goes on exactly as before). The class's own method is called, so a trace attached to the server (its
    instance attributes) records nothing."""
    done, reason = srv.done, srv.done_reason
    srv._close_session = lambda: None             # an instance attribute shadows the method for this one call
    try:
        return type(srv).finalize(srv, None)
    finally:
        srv.__dict__.pop("_close_session", None)
        srv.done, srv.done_reason = done, reason


# ---------------------------------------------------------------------------------------------- the service
class Service:
    """The graded episode behind the socket: one client at a time, the two windows, the record."""

    def __init__(self, cfg, log, reqlog, state, state_path, record_path, server=None, meta=None, trace=None,
                 freeze_chan=None):
        self.cfg = cfg
        self.freeze_chan = freeze_chan          # to the root supervisor: the freeze (dual mode)
        self.log, self.reqlog = log, reqlog
        self.state, self.state_path, self.record_path = state, state_path, record_path
        self.server = server                   # arm.assembly_rov.server.RovInfoServer; None in a restarted worker
        self.trace = trace                     # episode_trace.EpisodeTrace on the server, or None
        self.meta = dict(meta or {})           # the episode's id, its seed's digest, the oracle token's digest
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
        self.final_summary = None              # the cell's finalize() once the episode has ended
        self.cell_error = None                 # an exception inside the cell that ended the episode (the wire_error)
        self.trace_dir = None

    # -- the freeze (dual mode)
    def freeze(self):
        """Ask the root supervisor to freeze the program (copy /home/user/submission into the record directory) and wait
        for it: {"status": "ok" | "too_large" | "error", "files", "bytes", "sha256", ...}."""
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
        have at a start, so the program item keeps main's preparation time; the graded episode can no longer start.
        A copy that fails is tried freeze_retries times; if it still fails the record says so (snapshot_status error)
        and the verifier scores the program 0: it never falls back to the session-end submission. The record says what
        happened (started false, prep_window_closed true, the snapshot's fields)."""
        if self.started or self.state.get("prep_closed"):
            return
        snap = {"status": "error", "detail": "not tried"}
        for attempt in range(max(1, int(self.cfg["freeze_retries"]))):
            snap = self.freeze()
            if snap.get("status") in ("ok", "too_large"):
                break
            self.log.write(kind="freeze_failed", at="prep_window", attempt=attempt + 1,
                           detail=str(snap.get("detail"))[:200])
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
               "score_error": None, "render_errors": 0, "service": SERVICE, "service_version": SERVICE_VERSION,
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
                self.server._close_session()  # the camera session: the episode can no longer run
            except Exception:  # noqa: BLE001
                pass

    def _frozen_public(self):
        snap = self.state.get("snapshot") or {}
        return {"status": snap.get("status"), "files": int(snap.get("files") or 0),
                "bytes": int(snap.get("bytes") or 0)} if snap else None

    # -- persistence
    def save_state(self):
        atomic_write_json(self.state_path, self.state)

    def _summary(self):
        """The episode's summary: the cell's finalize() once the episode has ended, else as it stands (summary_now)."""
        srv = self.server
        if self.final_summary is not None:
            s = dict(self.final_summary)
        else:
            try:
                s = summary_now(srv)
                s["score_error"] = None
            except Exception:  # noqa: BLE001 - an unscoreable state is recorded (the verifier then fails the run)
                err = traceback.format_exc()
                self.log.write(kind="score_failed", tb=err[-4000:])
                s = {"stage_score": 0.0, "hard_ok": False, "score_error": err.strip().splitlines()[-1][:300]}
        s.setdefault("score_error", None)
        wall = time.time() - float(self.t_start) if not self.ended else float(self.t_end) - float(self.t_start)
        s["episode_wall_s"] = wall
        s["end_reason"] = self.end_reason if self.ended else None
        return s

    def _record(self):
        s = self._summary()
        score_error = s.get("score_error")
        try:
            sc = float(s.get("stage_score", 0.0))
        except (TypeError, ValueError):
            sc = float("nan")
        if not math.isfinite(sc):
            score_error = score_error or "the stage score is not a finite number (%r)" % (s.get("stage_score"),)
        episode_score = episode_score_of(s) if score_error is None else 0.0
        m = episode_metrics(s) if score_error is None else {}
        m["privileged_mode"] = 1.0 if self.server.privileged else 0.0
        for r in END_REASONS:
            m["end_reason." + r] = 1.0 if (self.ended and self.end_reason == r) else 0.0
        compact = {k: v for k, v in s.items() if k not in SUMMARY_DROP}
        compact["violations"] = list(s.get("violations") or [])[:20]
        compact["trim_eval"] = {k: v for k, v in (s.get("trim_eval") or {}).items()
                                if isinstance(v, (int, float, str, bool)) or v is None}
        now = time.time()
        rec = {
            "format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.meta.get("episode"),
            "seed_sha256": self.meta.get("seed_sha256"), "started": True, "ended": bool(self.ended),
            "end_reason": self.end_reason if self.ended else None, "requests": int(self.requests),
            "steps": int(self.server.step_calls), "sim_time_s": round(float(self.server.data.time), 4),
            "wall_s_since_start": round((float(self.t_end) if self.ended else now) - float(self.t_start), 3),
            "episode_score": episode_score, "metrics": m,
            "privileged": bool(self.server.privileged), "started_at": _iso(self.t_start),
            "ended_at": _iso(self.t_end) if self.ended else None,
            "windows": {"prep_s": self.prep_s, "graded_s": self.graded_s,
                        "service_start": _iso(self.state.get("service_start"))},
            "score_error": score_error, "cell_error": self.cell_error,
            "render_errors": int(getattr(self.server, "render_failures", 0) or 0),
            "done_reason": s.get("done_reason") if self.ended else None, "trace_dir": self.trace_dir,
            "summary": compact,
            "service": SERVICE, "service_version": SERVICE_VERSION,
        }
        snap = self.state.get("snapshot") or {}        # the program frozen at the start (dual mode)
        rec.update({"snapshot_status": snap.get("status"), "snapshot_at": snap.get("at"),
                    "snapshot_files": int(snap.get("files") or 0), "snapshot_bytes": int(snap.get("bytes") or 0),
                    "snapshot_sha256": snap.get("sha256"), "snapshot_skipped": int(snap.get("skipped") or 0),
                    "prep_window_closed": False})
        return finite(rec)

    def _write_record(self):
        try:
            atomic_write_json(self.record_path, self._record())
        except Exception:  # noqa: BLE001 - logged; the next change writes it again
            self.log.write(kind="record_write_failed", tb=traceback.format_exc(limit=8))
        self.last_write = time.time()
        self.dirty = False

    def _log_request(self, method, params, error=None):
        try:
            if isinstance(params, dict):
                params = {k: v for k, v in params.items() if k != "oracle_token"}
            text = json.dumps(params, default=str)
            args = params if len(text) <= 4096 else text[:4096] + "..."
        except (TypeError, ValueError):
            args = "<unprintable>"
        sim = None
        if self.server is not None and getattr(self.server, "data", None) is not None:
            sim = round(float(self.server.data.time), 4)
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
    def _end(self, reason):
        """End the graded episode (under self.lock) the way the offline grader ends an episode whose program stops at
        this moment: the cell's finalize() (the episode is marked done, the camera session closed, nothing more is
        simulated, the state scored as it stands), the final record, the replay record."""
        if self.ended:
            return
        now = time.time()
        self.ended, self.end_reason, self.t_end = True, reason, now
        srv = self.server
        self.state.update(ended=True, end_reason=reason, end_wall=now)
        try:
            self.save_state()
        except OSError:
            self.log.write(kind="state_write_failed")
        if srv is None:
            return
        if not srv.done:                       # an exception inside the cell ends it as the offline wire's wire_error
            srv._end("wire_error" if self.cell_error else reason)
        srv.wall_used = now - float(self.t_start)
        try:
            self.final_summary = srv.finalize(None)   # through the trace's hook when one is attached, as offline
            self.final_summary["score_error"] = None
        except Exception:  # noqa: BLE001
            err = traceback.format_exc()
            self.log.write(kind="score_failed", tb=err[-4000:])
            self.final_summary = {"stage_score": 0.0, "hard_ok": False,
                                  "score_error": err.strip().splitlines()[-1][:300]}
        self._save_trace()
        self._write_record()
        self.log.write(kind="episode_end", reason=reason, sim_time=round(float(srv.data.time), 3),
                       steps=int(srv.step_calls), requests=self.requests)

    def _save_trace(self):
        """The offline verifier's replay record of the episode, in the record directory (never a scoring input)."""
        tr, self.trace = self.trace, None
        if tr is None:
            return
        try:
            tr.detach()
            s = dict(self.final_summary or {})
            s["end_reason"] = self.end_reason
            s["episode_wall_s"] = float(self.t_end) - float(self.t_start)
            self.trace_dir = tr.save(os.path.join(self.cfg["state_dir"], "trace"), "graded", s,
                                     extra={"episode": self.meta.get("episode"),
                                            "privileged": bool(self.server.privileged), "end_reason": self.end_reason})
            if self.trace_dir:
                self.trace_dir = os.path.relpath(self.trace_dir, self.cfg["state_dir"])
        except Exception:  # noqa: BLE001 - a record only
            self.log.write(kind="trace_not_written", tb=traceback.format_exc(limit=8))

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
        if srv is not None:
            steps, sim, frames = srv.step_calls, float(srv.data.time), srv.frames_used
        else:
            rec = self.last_record or {}
            steps, sim = rec.get("steps", 0), rec.get("sim_time_s", 0.0)
            frames = (rec.get("metrics") or {}).get("camera_frames_mean", 0)
        t0 = float(self.state.get("service_start") or 0.0)
        running = self.started and not self.ended
        return {
            "service": SERVICE, "started": bool(self.started), "ended": bool(self.ended),
            "end_reason": self.end_reason if self.ended else None, "steps": int(steps or 0),
            "sim_time": round(float(sim or 0.0), 4), "camera_frames": int(frames or 0),
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
            # the charge: persisted before anything is simulated, never refunded
            self.state.update(started=True, start_wall=now, privileged=privileged, snapshot=snap)
            try:
                self.save_state()
            except OSError:
                self.state.update(started=False, start_wall=None, privileged=False, snapshot=None)
                self.log.write(kind="charge_persist_failed", tb=traceback.format_exc(limit=8))
                return self._err(rid, "internal_error", "the start could not be recorded; nothing was started")
            self.started, self.t_start = True, now
            self.server.privileged = privileged    # the layout is the same either way (oracle_nominal_layout false)
            self.server.wall_start = now
            self.requests += 1
            self._log_request("start", {"privileged": True} if privileged else {})
            self._write_record()
            self.log.write(kind="episode_start", privileged=privileged)
            return {"id": rid, "result": {"started": True, "sim_time": round(float(self.server.data.time), 4),
                                          "graded_window_s": self.graded_s, "program_frozen": self._frozen_public()}}

    def _cell_call(self, rid, method, params):
        with self.lock:
            if not self.started:
                return self._err(rid, "not_started", "the graded episode has not started: call start() first")
            if not self.ended and time.time() - float(self.t_start) >= self.graded_s:
                self._end("graded_window")
            if self.ended or self.server is None:
                if method == "finish":
                    return {"id": rid, "result": {"ok": True}}
                return self._err(rid, "episode_over", "the graded episode is over (%s)" % self.end_reason)
            srv = self.server
            self.requests += 1
            if not isinstance(params, dict):       # as the offline wire treats a malformed params object
                params = {}
            self.call = (time.monotonic(), method)
            failed = None
            try:
                result = srv.handle(method, params)
            except Exception:  # noqa: BLE001 - offline this ends the episode (the wire's wire_error), scored as it stands
                failed = traceback.format_exc()
                result = None
            finally:
                self.call = None
            srv.wall_used = time.time() - float(self.t_start)
            if failed is not None:
                self.cell_error = failed.strip().splitlines()[-1][:300]
                self.log.write(kind="cell_error", method=str(method)[:80], tb=failed[-4000:])
                self._log_request(method, params, error="internal_error")
                self._end("done")
                return self._err(rid, "internal_error", "the cell failed on this request; the graded episode is over")
            self._log_request(method, params, error=result.get("error") if is_error(result) else None)
            if srv.done:
                self._end("finish" if srv.done_reason == "finish" else "done")
            else:
                self.dirty = True
                if time.time() - self.last_write >= self.min_interval:
                    self._write_record()
        if is_error(result):
            return {"id": rid, "error": result["error"]}
        return {"id": rid, "result": result}

    def handle_request(self, req):
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

    # -- threads
    def ticker(self):
        """The start window's close (the freeze), the graded window's end, and a pending record write a quarter of a
        second after the last change."""
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
        threading.Thread(target=self.ticker, name="arbcell-ticker", daemon=True).start()
        threading.Thread(target=self.watchdog, name="arbcell-watchdog", daemon=True).start()
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
                    try:
                        reply = self.handle_request(req)
                    except Exception:  # noqa: BLE001
                        self.log.write(kind="internal_error", conn=cid, tb=traceback.format_exc(limit=20))
                        reply = self._err(req.get("id"), "internal_error")
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
def _sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def check_assets(cfg):
    """The image's task assets the cell reads (the robot models, the cameras' scenery) against tests/given_hashes.json's
    byte copy, as the offline verifier checks its copy of the givens (root_payload_ok): a difference is a setup error.
    Returns the number of files checked."""
    hashes = read_json(cfg["given_hashes"], 8 << 20)
    n = 0
    for rel, digest in sorted(hashes.items()):
        if not rel.startswith(("given/robot/", "render/")):
            continue
        p = os.path.join(cfg["asset_root"], rel)
        st = os.lstat(p)
        if not stat.S_ISREG(st.st_mode) or st.st_uid != 0 or (st.st_mode & 0o022):
            raise RuntimeError("not a root-owned, group/world-unwritable regular file: %s" % rel)
        if _sha256_file(p) != digest:
            raise RuntimeError("the task asset %s differs from given_hashes.json" % rel)
        n += 1
    if n == 0:
        raise RuntimeError("given_hashes.json names no robot model or scenery file")
    return n


def build_episode(cfg, entry, log):
    """(as root, before the drop) The graded episode's cell, built as tests/verify.py builds an episode's: the trim
    table and the grader configuration's score block (check_trim_config), the kit (check_rov_kit), the fixtures, the
    server in the hidden mode, reset(entry), the layout digest compared with the hidden set's; the replay record
    attached as the offline verifier attaches it. Returns (server, trace)."""
    mods = {name: importlib.import_module(name) for name in CELL_MODULES}
    S, C = mods["arm.assembly_rov.server"], mods["arm.assembly_rov.cell"]
    FX, TE = mods["arm.assembly_rov.fixtures"], mods["arm.assembly_rov.trim_eval"]
    n = check_assets(cfg)
    log.write(kind="assets_checked", files=n)
    cell_cfg = S.merge_config(read_json(os.path.join(cfg["cell_dir"], "grader_config.json"), 1 << 20))
    table = TE.TrimTable(os.path.join(cfg["cell_dir"], "trim_table.json"))
    S.check_trim_config(cell_cfg, table)
    kit = C.load_kit(cfg["kit_dir"])
    S.check_rov_kit(kit)
    fx = FX.Fixtures(FX.load_cell_layout(), FX.load_facts(), lmag=bool(cell_cfg["layout"].get("lmag", False)))
    srv = S.RovInfoServer(cfg["kit_dir"], cfg["robot_dir"], config=cell_cfg, mode="hidden", privileged=False,
                          render=True, log=lambda s: log.write(kind="cell", msg=str(s)[:8000]), kit=kit, fx=fx,
                          table=table)
    r = srv.reset(entry)
    if r.get("nominal") or r.get("layout_digest") != entry["layout_digest"]:
        raise ValueError("the cell built a layout that is not the fixed hidden set's (its layout digest differs)")
    trace = None
    tcfg = cell_cfg.get("episode_trace") or {}
    if cfg["trace"] and tcfg.get("agent"):
        try:
            trace = mods["episode_trace"].EpisodeTrace(srv, every_ticks=int(tcfg.get("every_ticks", 4)),
                                                       log=lambda s: log.write(kind="trace", msg=str(s)[:2000]))
        except Exception:  # noqa: BLE001 - a record only
            log.write(kind="trace_not_attached", tb=traceback.format_exc(limit=8))
    return srv, trace


def run_worker(cfg, listener, ready_fd, lab, freeze_chan=None):
    """Forked from the supervisor as root: read the hidden episode and build its cell (only while it has not started),
    drop privileges, serve."""
    _set_comm("arbcell-worker")
    os.umask(0o077)
    state_dir = cfg["state_dir"]
    log = Log(os.path.join(state_dir, "service.log"), int(cfg["max_log_bytes"]))
    log.chown(lab.pw_uid, lab.pw_gid)
    try:
        for k, v in cfg["env"].items():
            os.environ[k] = v
        work = cfg["work_dir"]
        os.environ.update({"ARB_AGENT_RENDER_DIR": cfg["render_dir"], "ARB_RECORD_ROOT": work, "TMPDIR": work,
                           "TMP": work, "TEMP": work, "XDG_CACHE_HOME": os.path.join(work, "cache")})
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
            entry = meta["entry"]
            seed = entry["seed"]
            if type(seed) is not int or not 0 <= seed < (1 << 63):
                raise ValueError("the graded episode's seed is not an integer in [0, 2**63)")
            if hashlib.sha256(str(seed).encode("ascii")).hexdigest() != meta["seed_sha256"]:
                raise ValueError("the graded episode's seed does not match its digest")
            sys.path.insert(0, cfg["cell_dir"])
            server, trace = build_episode(cfg, entry, log)
            public = {"episode": str(meta["episode"]), "seed_sha256": str(meta["seed_sha256"]),
                      "oracle_token_sha256": meta.get("oracle_token_sha256")}
            del meta, entry, seed
            svc = Service(cfg, log, reqlog, state, state_path, record_path, server=server, meta=public, trace=trace,
                          freeze_chan=freeze_chan)
            drop_privileges(lab)
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
    """Copy the regular files and directories under the open directory `src_fd` into `dst_dir`. Every name is resolved
    relative to an open directory descriptor with O_NOFOLLOW: no symbolic link is followed and no path the agent
    controls is resolved by name; symbolic links, special files, __pycache__ and odd names are skipped."""
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
                if not stat.S_ISREG(os.fstat(fd).st_mode):
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
    """The frozen program's digest (tests/online_record.py recomputes it): for every file in the sorted order of its
    relative path, the line `<path>\\0<size>\\0<sha256 of its content>\\n`; the sha256 of all the lines."""
    total = hashlib.sha256()
    for rel, size, digest in sorted(files, key=lambda f: f[0].encode("utf-8")):
        total.update(rel.encode("utf-8") + b"\0" + str(size).encode() + b"\0" + digest.encode() + b"\n")
    return total.hexdigest()


def snapshot_submission(cfg):
    """(root, the supervisor; dual mode) The freeze: a root-owned copy (directories 0700, files 0600) of the agent's
    submission folder in the record directory, taken before the graded start is charged. Within the offline
    submission's caps the copy is complete (status ok); beyond them nothing is copied (status too_large); a copy that
    fails is status error (the start is then refused). The copy is built under a temporary name and renamed into
    place. Never reads anything hidden."""
    state_dir = cfg["state_dir"]
    dst = os.path.join(state_dir, cfg["snapshot_name"])
    tmp = dst + ".tmp"
    caps = {"max_bytes": int(cfg["snapshot_max_bytes"]), "max_entries": int(cfg["snapshot_max_entries"]),
            "max_depth": int(cfg["snapshot_max_depth"])}
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
                    res = snapshot_submission(cfg)
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
    cfg["episode"] = os.path.normpath(os.path.join(base, cfg["episode"]))
    cfg["given_hashes"] = os.path.normpath(os.path.join(cfg["cell_dir"], cfg["given_hashes"]))
    if not isinstance(cfg["task"], str) or not cfg["task"]:
        raise ValueError("config needs the task's slug")
    for key in ("socket", "state_dir", "cell_dir", "kit_dir", "robot_dir", "render_dir", "asset_root", "work_dir",
                "submission"):
        if not os.path.isabs(cfg[key]):
            raise ValueError("%s must be an absolute path" % key)
    if os.path.dirname(cfg["socket"]) in ("/", "/tmp", "/var/tmp", "/dev/shm", "/run"):
        raise ValueError("the socket must live in its own root-owned directory")
    if not isinstance(cfg["env"], dict) or not all(isinstance(k, str) and isinstance(v, str)
                                                   for k, v in cfg["env"].items()):
        raise ValueError("env must map strings to strings")
    if not isinstance(cfg["snapshot_max_bytes"], int) or cfg["snapshot_max_bytes"] <= 0:
        raise ValueError("snapshot_max_bytes must be the offline submission's size cap in bytes")
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


def start(config_path):
    if os.geteuid() != 0:
        print("rov-cell-online: must be started as root (environment/setup/setup.sh)", file=sys.stderr)
        return 2
    try:
        cfg = load_config(config_path)
        lab = _accounts(cfg)
        os.umask(0o077)
        sockdir = os.path.dirname(cfg["socket"])
        os.makedirs(os.path.dirname(sockdir), exist_ok=True)
        _secure_dir(sockdir, 0o755, 0, 0)
        os.makedirs(os.path.dirname(cfg["state_dir"]), exist_ok=True)
        _secure_dir(cfg["state_dir"], 0o700, lab.pw_uid, lab.pw_gid)
        os.makedirs(os.path.dirname(cfg["work_dir"]), exist_ok=True)
        _secure_dir(cfg["work_dir"], 0o700, lab.pw_uid, lab.pw_gid)
        state_path = os.path.join(cfg["state_dir"], "state.json")
        if not os.path.lexists(state_path):    # the service's start: the start window counts from here (never reset)
            atomic_write_json(state_path, {"version": 1, "service_start": time.time(), "started": False,
                                           "start_wall": None, "ended": False, "end_reason": None})
            os.chown(state_path, lab.pw_uid, lab.pw_gid)
    except Exception as exc:  # noqa: BLE001
        print("rov-cell-online: configuration error: %s: %s" % (type(exc).__name__, str(exc)[:200]), file=sys.stderr)
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
        print("rov-cell-online: ready (socket %s)" % cfg["socket"], file=sys.stderr)
        return 0
    print("rov-cell-online: failed to start (details in the service's own log)", file=sys.stderr)
    return 1


def main(argv=None):
    ap = argparse.ArgumentParser(description="rov-cell-online (start as root from environment/setup/setup.sh)")
    ap.add_argument("--config", required=True)
    args = ap.parse_args(argv)
    return start(args.config)


if __name__ == "__main__":
    sys.exit(main())
