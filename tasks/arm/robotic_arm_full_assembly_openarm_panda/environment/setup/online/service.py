#!/usr/bin/env python3
"""openarm-cell-online: the cell service of robotic_arm_full_assembly_openarm_panda's graded episode (dual mode,
experimental).

The agent drives ONE hidden graded episode of the OpenArm pair's assembly cell itself, during its session, through this
service. The service writes the episode's outcome to /var/lib/arb-online (a declared artifact the agent can neither read
nor write) and the separate verifier scores that record (tests/verify.py, tests/online_record.py). The cell is the
offline verifier's own: online/cell holds byte copies of the verifier's cell code (tests/arm/... as far as the cell, its
cameras and the privileged reference program import it, tests/render*, tests/room, tests/episode_trace.py), the pair's
kit (tests/gt), the hidden bench plan (tests/hidden/bench_plan.json) and the grader configuration
(tests/grader_config.json, the cell's configuration).

Started once by environment/setup/setup.sh as root, before the agent phase, after setup.sh installed online/cell into a
directory only root and the lab account can read (cell_dir, root:arbcell 0750 / 0640):

    env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \\
        /opt/venv/bin/python3 -I -B online/service.py --config online/online.json

The command returns once the service serves (exit 0) or failed to start (exit 1); the service itself is a detached
daemon (setsid, stdio on /dev/null) that lives into the agent phase. The layout is arm_hidden_mechanism_box's alelab
(environment/setup/lab/alelab/server.py there), as house_cleaning_with_tools's graded service carries it:
  supervisor (root, comm "arbcell-sup"): binds the unix socket, writes the pid file, forks the worker and restarts it at
      most max_restarts times, and freezes the agent's program on the worker's request (below). It never reads the
      hidden episode and never accepts a connection.
  worker (comm "arbcell-worker"): forked as root. While the graded episode has not started it reads the hidden episode's
      seed (online/episode.json, root-only in the setup stage), drops to the lab account irreversibly (setgroups([]),
      setgid, setuid; not dumpable; no core files), then imports the cell from cell_dir, builds the episode's cell
      (arm.assembly_openarm.server.OpenArmServer, reset from the seed: the jittered bench layout, the gallows' jitter,
      the cameras' mounts and lenses, the lighting, the F/T bias, exactly as the offline grader builds an episode) and
      serves. The cell's cameras open their render session lazily (the first get_camera), reading the kit's meshes and
      the scenery (/opt/task-assets/payload/render, the image's root-owned copy of ~/render) then, which is why the
      cell stays readable by the lab account. A worker that finds the episode started (a restart) reads nothing hidden:
      the episode ends with its last record (end_reason worker_restart, no resume) and the worker answers `status` only.
  Both run under PR_SET_NO_NEW_PRIVS and a seccomp filter under which socket() / socketpair() work only for AF_UNIX.

The graded episode:
  * `start` charges it once per session: the charge is persisted (state.json, fsync) before anything is simulated and is
    never refunded. A start is accepted only within prep_s of the service's start. A start with the oracle token
    (solution/: its sha256 equals the digest in episode.json) is privileged, as offline: the cell is rebuilt from the
    same seed in the privileged mode (the bench plan's nominal layout, grader_config oracle_nominal_layout) and the
    privileged calls (reference_assemble, get_state_privileged, describe_privileged) answer; the rebuild comes before
    the freeze and the charge (a failed rebuild refuses the start).
  * The freeze (dual mode): the program freezes at the earlier of the graded start and the start window's close
    (prep_s after the service's start). Before a start is charged, or when the window closes without a start, the
    worker asks the supervisor, over the socketpair it holds since its fork, to copy /home/user/submission into
    submission_snapshot in the record directory (root-owned 0700 / 0600; regular files and directories of the agent's
    own only, every name opened relative to its parent with O_NOFOLLOW, no symbolic link followed, special files and
    __pycache__ skipped; within the offline submission's size cap, the grader configuration's route_gate, else nothing
    is copied: too_large). The supervisor answers only this one request, with fixed paths, and only while the state
    file says the episode has not started and nothing was frozen; a copy that fails refuses the start and charges
    nothing. The record carries snapshot_status, snapshot_at (graded_start | prep_window), snapshot_files,
    snapshot_bytes and snapshot_sha256; a window that closes without a start writes a record of its own (started false,
    prep_window_closed true) and no start is accepted afterwards. The verifier grades that copy as the offline item.
  * The episode lives in the worker, not in a connection: a disconnect leaves it where it is and a new connection
    resumes it. One client at a time (a second connection is told `busy`). A connection idle for idle_timeout_s is
    closed; the episode stays.
  * It ends at the cell's own end (1800 s of simulated time, sim_cap, or an internal error of the cell: `done`), at
    `finish`, graded_s after its start (`graded_window`), or when the session ends (the sandbox is frozen or destroyed:
    the last record applies). It ends the way the offline grader ends an episode whose program stops at that moment:
    the cell is marked done, its finalize() reads the fastened mates as they stand (its stand check afterwards, the
    motors off for 3 s, is a statistic only, as offline), nothing else is simulated.
  * The offline program-run limits do not apply (the simulation advances only inside the agent's requests): no
    wall-clock cap of 3000 s, no 120 s silence abort, no CPU backstop. The cell's own limits do (1800 s of simulated
    time, 900 camera frames, 500 ticks per step, the safety watch). describe() is unchanged (its `budgets` describe the
    program run).
  * The record (record.json, written atomically: a temp file, fsync, rename) is rewritten after a request that changes
    the episode, at most every record_min_interval_s (a pending change is written within about a quarter of a second
    of the last request), and always when the episode starts and ends; requests.jsonl logs every request of the
    episode (method, arguments, simulated time; no images, the oracle token never) up to requests_log_max_bytes.
  * The replay record (the offline grader's maintainers' trace, tests/episode_trace.py, unchanged): the cell's joint
    state every 4 ticks and its event log, saved under trace/graded/ in the record directory every trace_every_s while
    the episode runs and when it ends; without the camera frames' pictures (their index, camera and simulated time
    only), which offline go to the verifier's own disk and here would cost the agent's sandbox up to 2 GB of memory.
  * The client learns the cell's replies (the offline wire's) and `status` (started / ended, steps, simulated time,
    camera frames, time left in each window); never the score, the record, the episode's id or its seed.

Protocol: one JSON object per line in each direction, in the offline wire's envelope (arm/assembly_cell/wire.py):
    {"id": n, "method": m, "params": {...}}  ->  {"id": n, "result": ...}  |  {"id": n, "error": code}
(a cell reply that is exactly {"error": code} is sent as an error, anything else as the result, as offline; the
service's own errors add "detail"). On connect the service sends one greeting line, {"service": SERVICE}, or
{"service": SERVICE, "error": "busy", ...} and closes. Methods: `status`, `start`, and the cell's own (describe,
get_state, set_control, command, step, get_camera, finish; the privileged ones only on an episode started with the
oracle token, as offline). The service's own errors: bad_request, bad_params, line_too_long, not_started,
already_started, start_window_closed, episode_over, internal_error.

Dependencies: the Python 3.12 standard library for the supervisor; the worker imports the cell (numpy, mujoco, PIL,
MuJoCo's Filament module).
"""

import argparse
import ctypes
import datetime
import gc
import hashlib
import importlib
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
import threading
import time
import traceback

SERVICE = "openarm-cell-online/1"
SERVICE_VERSION = "1"
RECORD_FORMAT = "arb-online-record/1"
DEFAULTS = {
    "task": None,                     # the task's slug (the record's `task`)
    "episode": "episode.json",        # the hidden graded episode (relative to the config file; root-only)
    "cell_dir": "/opt/arb-online-cell",   # the installed copy of online/cell (root:lab account 0750 / 0640)
    "cell_config": "grader_config.json",  # relative to cell_dir: the cell's configuration (tests/grader_config.json)
    "kit_dir": "gt",                  # relative to cell_dir: the pair's kit (tests/gt)
    "bench_plan": "hidden/bench_plan.json",   # relative to cell_dir: the hidden bench plan
    "panda_dir": "/opt/task-assets/payload/given/panda",   # the given Panda model (the image's root-owned copy)
    "user": "arbcell",                # the dedicated lab account (setup.sh creates it)
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
    "start_timeout_s": 900.0,
    "require_seccomp": True,
    "trace_every_s": 300.0,           # the replay record is saved this often while the episode runs (and at its end)
    "env": {},                        # PUBLIC constants only (renderer, scenery path); never hidden values
    # the freeze (dual mode): the root supervisor copies this folder before the graded start is charged
    "submission": "/home/user/submission",
    "snapshot_name": "submission_snapshot",   # inside state_dir
    "snapshot_max_bytes": None,       # None: the offline submission's cap, cell_config's route_gate.max_submission_mb
    "snapshot_max_entries": 20000,
    "snapshot_max_depth": 32,
    "freeze_timeout_s": 300.0,
}
# the cell's modules, imported by the worker before it serves (lazy imports of the cell then find them loaded)
CELL_MODULES = ("arm.assembly_openarm.server", "arm.assembly_openarm.cell", "arm.assembly_openarm.coupling",
                "arm.assembly_openarm.layout", "arm.assembly_openarm.motion", "arm.assembly_openarm.reference",
                "arm.assembly_openarm.agent_render", "arm.assembly_full.agent_render", "arm.assembly_info.cell_info",
                "arm.assembly_info.layout_info", "arm.assembly_info.cell_render", "arm.assembly_cell.controller",
                "render.spec", "render.postprocess", "render.backends", "render.scale_refs", "episode_trace")
LONG_CALLS = ("reference_assemble",)  # privileged: the reference assembler runs inside one call (bounded by graded_s)
CAMERAS = ("top", "wrist_left", "wrist_right")

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
    if hasattr(x, "tolist"):
        return finite(x.tolist())
    return str(x)


def _jsonable(obj):
    """json.dumps fallback of a reply (the offline wire's json.dumps has none: the cell replies plain values); numpy
    values as plain values, anything else as its str."""
    if hasattr(obj, "tolist"):
        return obj.tolist()
    if isinstance(obj, (set, frozenset)):
        return sorted(obj, key=str)
    return str(obj)


def _iso(t):
    return None if t is None else datetime.datetime.fromtimestamp(t, datetime.timezone.utc).isoformat(
        timespec="seconds").replace("+00:00", "Z")


def is_error(result):
    """An error reply of the cell, as the offline wire sends it (arm/assembly_cell/wire.py serve_episode): a dict that
    holds "error" and nothing else. Anything else (also {"ok": False, "error": ...}) goes back as the result."""
    return isinstance(result, dict) and "error" in result and len(result) == 1


def _reject_constant(name):
    raise ValueError("non-finite constant")


_SEED_TEXT = re.compile(r"seed -?\d+")


def _no_seed(text):
    """The cell's log lines name the episode's seed (its low 32 bits); the service's log lives in the record directory,
    so the seed is masked there (the graded episode's seed stays in environment/setup/online/episode.json)."""
    return _SEED_TEXT.sub("seed <graded>", str(text))


# ---------------------------------------------------------------------------------------------- the episode's record
END_REASONS = ("done", "finish", "graded_window", "worker_restart")
SUMMARY_DROP = ("events", "coupling_events", "layout", "seed")    # bulky or the episode's draw: kept out of the record


def _f(v, default=0.0):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return float(default)
    return v if math.isfinite(v) else float(default)


def live_summary(srv):
    """The offline episode record (OpenArmServer.finalize) as it stands now, without ending the episode: the same
    fastened-mate record and score, without the stand check (which finalize runs after it has read the score)."""
    rec = srv.sm.record()
    weights = {r["id"]: float(r["weight"]) for r in rec}
    total = sum(weights.values())
    achieved = sum(weights[r["id"]] for r in rec if r["achieved"])
    return {
        "cell_version": getattr(sys.modules.get(type(srv).__module__), "CELL_VERSION", None),
        "done_reason": srv.done_reason,
        "sim_time_s": float(srv.data.time), "ticks": int(srv.tick_count),
        "mates": rec, "states": dict(srv.sm.state), "weights": weights, "stage_score": achieved / total if total else 0.0,
        "n_achieved": int(sum(1 for r in rec if r["achieved"])), "n_fastened": int(sum(1 for r in rec if r["fastened"])),
        "n_mates": len(rec), "all_mates": bool(all(r["achieved"] for r in rec)),
        "violations": list(srv.violations), "safety_ok": len(srv.violations) == 0,
        "illegal_call_count": int(srv.illegal_calls), "calls": dict(srv.calls), "step_calls": int(srv.step_calls),
        "lost_parts": sorted(srv._lost), "dropped_parts": sorted(srv._dropped),
        "damage": dict(srv.dmg), "stand_after_power_off_deg": {},
        "camera_frames": dict(srv.frames_by_camera), "camera_frames_total": int(srv.frames_used),
        "render_wall_s": float(srv.render_wall_s), "renderer": srv.renderer, "privileged_mode": bool(srv.privileged),
    }


def episode_metrics(s):
    """The offline verifier's per-episode statistics (tests/verify.py's _main_locked, the same names, for one episode) of
    one episode's summary `s` (finalize()'s, or live_summary()'s while the episode runs). Never a scoring input."""
    dmg = s.get("damage") or {}
    m = {
        "stage_score_mean_ungated": _f(s.get("stage_score")),
        "mates_achieved_mean": _f(s.get("n_achieved")),
        "mates_fastened_mean": _f(s.get("n_fastened")),
        "all_mates_rate": 1.0 if s.get("all_mates") else 0.0,
        "safety_violation_rate": 0.0 if s.get("safety_ok", True) else 1.0,
        "safety_violations": float(len(s.get("violations") or [])),
        "mean_sim_time_s": _f(s.get("sim_time_s")),
        "mean_episode_wall_s": _f(s.get("episode_wall_s")),
        "illegal_call_count": _f(s.get("illegal_call_count")),
        # main counts reference_assemble as a step call (the oracle's program steps inside it)
        "step_calls": _f(s.get("step_calls")) + _f((s.get("calls") or {}).get("reference_assemble", 0)),
        "dropped_parts_mean": float(len(s.get("dropped_parts") or [])),
        "lost_parts_mean": float(len(s.get("lost_parts") or [])),
        "max_link_force_N": _f(dmg.get("max_link_force_N")),
        "max_pad_force_N": _f(dmg.get("max_pad_force_N")),
        "max_part_force_N": _f(dmg.get("max_part_force_N")),
        "max_part_force_sustained_N": _f(dmg.get("max_part_force_sustained_N")),
        "camera_frames_mean": _f(s.get("camera_frames_total")),
        "render_wall_s_mean": _f(s.get("render_wall_s")),
    }
    for cam in CAMERAS:
        m["camera_frames_mean." + cam] = _f((s.get("camera_frames") or {}).get(cam, 0))
    for r in s.get("mates") or []:
        m["mate_rate." + str(r.get("id"))] = 1.0 if r.get("achieved") else 0.0
    m["flange_fastened_off_design_rate"] = float(sum(1 for r in (s.get("mates") or [])
                                                     if r.get("kind") == "flange" and r.get("fastened")
                                                     and not r.get("achieved")))
    m["stand_after_power_off_max_deg"] = float(max([abs(_f(v)) for v in (s.get("stand_after_power_off_deg") or {}).values()]
                                                   or [0.0]))
    return m


def make_trace(episode_trace, server, every_ticks, log):
    """The offline grader's replay record (tests/episode_trace.py's EpisodeTrace, unchanged) attached to the graded
    episode's cell, keeping each camera frame's index, camera and simulated time but not its picture (offline the
    pictures go to the verifier's own disk; here they would hold up to 900 PNGs in the agent's sandbox's memory)."""

    class GradedTrace(episode_trace.EpisodeTrace):
        def _get_camera(self, name="overview", depth=True):
            reply = self._cam(name, depth)
            if self.failed is None and isinstance(reply, dict) and reply.get("ok"):
                try:
                    self.cams.append((float(self.server.data.time), str(reply.get("camera", name)),
                                      int(reply.get("frame_index", -1)), None))
                except Exception as e:  # noqa: BLE001 - a record only
                    self._fail(e)
            return reply

    return GradedTrace(server, every_ticks=every_ticks, log=log)


# ---------------------------------------------------------------------------------------------- the service
class Service:
    """The graded episode behind the socket: one client at a time, the two windows, the record."""

    def __init__(self, cfg, log, reqlog, state, state_path, record_path, server=None, trace=None, build=None,
                 meta=None, freeze_chan=None):
        self.cfg = cfg
        self.freeze_chan = freeze_chan          # to the root supervisor: the freeze (dual mode)
        self.log, self.reqlog = log, reqlog
        self.state, self.state_path, self.record_path = state, state_path, record_path
        self.server = server                   # arm.assembly_openarm.server.OpenArmServer; None in a restarted worker
        self.trace = trace                     # the replay record (GradedTrace) of the episode's cell
        self.build = build                     # build(privileged) -> (server, trace): the episode's cell from its seed
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
        self.last_trace = 0.0
        self.trace_ticks = -1
        self.cell_errors = 0
        self.final = None                      # finalize()'s summary once the episode has ended
        self.sim_end = None                    # the episode's simulated time when it ended (before finalize's stand check)
        self.call = None                       # (monotonic start, method) of the running cell call: the watchdog's
        self.last_record = None                # a restarted worker's view of the last record (status)

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
        The copy is tried a few times; a copy that still fails is recorded as such (snapshot_status error) and the
        verifier then scores the program 0: it never falls back to the submission at the session's end. The record
        says so (started false, prep_window_closed true, the snapshot's fields)."""
        if self.started or self.state.get("prep_closed"):
            return
        snap = {"status": "error"}
        for attempt in range(3):
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
               "score_error": None, "service": SERVICE, "service_version": SERVICE_VERSION,
               "snapshot_status": snap.get("status"), "snapshot_at": "prep_window",
               "snapshot_time": _iso(self.state.get("prep_closed_wall")),
               "snapshot_files": int(snap.get("files") or 0), "snapshot_bytes": int(snap.get("bytes") or 0),
               "snapshot_sha256": snap.get("sha256"), "snapshot_skipped": int(snap.get("skipped") or 0)}
        try:
            atomic_write_json(self.record_path, finite(rec))
        except Exception:  # noqa: BLE001
            self.log.write(kind="record_write_failed", tb=traceback.format_exc(limit=8))
        self.log.write(kind="prep_window_closed", snapshot_status=snap.get("status"), files=snap.get("files"))
        self._release_cell()                   # the episode can no longer run

    def _frozen_public(self):
        snap = self.state.get("snapshot") or {}
        return {"status": snap.get("status"), "files": int(snap.get("files") or 0),
                "bytes": int(snap.get("bytes") or 0)} if snap else None

    def _release_cell(self):
        srv, self.server = self.server, None
        if self.trace is not None:
            try:
                self.trace.detach()
            except Exception:  # noqa: BLE001
                pass
        if srv is not None:
            try:
                srv._close_session()          # the cameras' render session
            except Exception:  # noqa: BLE001
                pass

    # -- persistence
    def save_state(self):
        atomic_write_json(self.state_path, self.state)

    def _summary(self):
        """The episode's summary: finalize()'s once the episode has ended, else live_summary() as it stands."""
        srv = self.server
        if self.final is not None:
            s = dict(self.final)
        else:
            s = live_summary(srv)
        wall = time.time() - float(self.t_start)
        if self.ended and self.t_end is not None:
            wall = float(self.t_end) - float(self.t_start)
        s["episode_wall_s"] = wall
        s.setdefault("score_error", None)
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
            sc = 0.0
        # the offline per-episode score: the stage score, 0 with a safety violation (tests/verify.py: `vals`)
        episode_score = max(0.0, min(1.0, sc * (1.0 if s.get("safety_ok", False) else 0.0)))
        m = episode_metrics(s)
        m["privileged_mode"] = 1.0 if s.get("privileged_mode") else 0.0
        m["cell_internal_errors"] = float(self.cell_errors)
        for r in END_REASONS:
            m["end_reason." + r] = 1.0 if (self.ended and self.end_reason == r) else 0.0
        compact = {k: v for k, v in s.items() if k not in SUMMARY_DROP}
        compact["violations"] = list(s.get("violations") or [])[:20]
        now = time.time()
        srv = self.server
        rec = {
            "format": RECORD_FORMAT, "task": self.cfg["task"], "episode": self.meta.get("episode"),
            "seed_sha256": self.meta.get("seed_sha256"), "started": True, "ended": bool(self.ended),
            "end_reason": self.end_reason if self.ended else None, "requests": int(self.requests),
            "steps": int(s.get("step_calls") or 0),
            "sim_time_s": round(self.sim_end if self.sim_end is not None else _f(s.get("sim_time_s")), 4),
            "wall_s_since_start": round((float(self.t_end) if self.ended and self.t_end else now) - float(self.t_start), 3),
            "episode_score": episode_score, "metrics": m,
            "privileged": bool(s.get("privileged_mode")), "started_at": _iso(self.t_start),
            "ended_at": _iso(self.t_end) if self.ended else None,
            "windows": {"prep_s": self.prep_s, "graded_s": self.graded_s,
                        "service_start": _iso(self.state.get("service_start"))},
            "score_error": score_error, "summary": compact,
            "cell_done_reason": s.get("done_reason"), "service": SERVICE, "service_version": SERVICE_VERSION,
            "trace": {"dir": "trace/graded", "frames": len(self.trace.q) if self.trace is not None else 0,
                      "failed": self.trace.failed if self.trace is not None else None,
                      "seed": "environment/setup/online/episode.json"},
        }
        if srv is None and self.final is None:
            rec["score_error"] = rec["score_error"] or "no cell"
        snap = self.state.get("snapshot") or {}        # the program frozen at the start (dual mode)
        rec.update({"snapshot_status": snap.get("status"), "snapshot_at": snap.get("at"),
                    "snapshot_time": _iso(self.t_start) if snap.get("at") == "graded_start" else None,
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

    def _save_trace(self, summary=None):
        """The replay record under trace/graded/ in the record directory (tests/episode_trace.py's save(), then its two
        files renamed into place); a failure is logged and never reaches the episode."""
        tr = self.trace
        if tr is None or tr.failed is not None or not tr.q:
            return
        state_dir = self.cfg["state_dir"]
        tmp_root = os.path.join(state_dir, "trace.tmp")
        try:
            shutil.rmtree(tmp_root, ignore_errors=True)
            if summary is None:
                summary = live_summary(self.server) if self.server is not None else None
            out = tr.save(tmp_root, "graded", summary=summary,
                          extra={"episode": self.meta.get("episode"), "seed_sha256": self.meta.get("seed_sha256"),
                                 "privileged": bool(self.server.privileged) if self.server is not None else None,
                                 "seed": "environment/setup/online/episode.json"})
            if out:
                final = os.path.join(state_dir, "trace", "graded")
                os.makedirs(final, mode=0o700, exist_ok=True)
                for fn in ("trace.npz", "meta.json"):
                    os.replace(os.path.join(out, fn), os.path.join(final, fn))
            self.last_trace = time.time()
            self.trace_ticks = int(getattr(self.server, "tick_count", -1)) if self.server is not None else -1
        except Exception:  # noqa: BLE001 - a record only
            self.log.write(kind="trace_save_failed", tb=traceback.format_exc(limit=8))
        finally:
            shutil.rmtree(tmp_root, ignore_errors=True)

    def _log_request(self, method, params, error=None):
        try:
            if isinstance(params, dict):
                params = {k: v for k, v in params.items() if k != "oracle_token"}
            text = json.dumps(params, default=str)
            args = params if len(text) <= 4096 else text[:4096] + "..."
        except (TypeError, ValueError):
            args = "<unprintable>"
        sim = round(float(self.server.data.time), 4) if self.server is not None else None
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
    def _end(self, reason, cell_reason=None):
        """End the graded episode (under self.lock) the way the offline grader ends an episode whose program stops at
        this moment (tests/verify.py run_episode): the cell is marked done, its finalize() reads the fastened mates as
        they stand (and then runs its stand check, a statistic), the final record and the replay record are written."""
        if self.ended:
            return
        now = time.time()
        self.ended, self.end_reason, self.t_end = True, reason, now
        srv = self.server
        if srv is not None and not srv.done:
            srv._end(cell_reason or reason)
        if srv is not None:
            self.sim_end = float(srv.data.time)
            srv.wall_used = now - float(self.t_start)
            try:
                self.final = srv.finalize(None)
            except Exception:  # noqa: BLE001 - recorded; the verifier then fails the run (a task error)
                err = traceback.format_exc()
                self.log.write(kind="finalize_failed", tb=err[-4000:])
                self.final = dict(live_summary(srv), score_error=err.strip().splitlines()[-1][:300])
        self.state.update(ended=True, end_reason=reason, end_wall=now)
        try:
            self.save_state()
        except OSError:
            self.log.write(kind="state_write_failed")
        if srv is not None or self.final is not None:
            self._write_record()
            self._save_trace(summary={k: v for k, v in (self.final or {}).items() if k != "seed"} or None)
            self.log.write(kind="episode_end", reason=reason, cell_reason=(self.final or {}).get("done_reason"),
                           sim_time=round(_f((self.final or {}).get("sim_time_s")), 3),
                           steps=int((self.final or {}).get("step_calls") or 0), requests=self.requests)
        self._release_cell()

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
        elif self.final is not None:
            steps = self.final.get("step_calls", 0)
            sim = self.sim_end if self.sim_end is not None else self.final.get("sim_time_s", 0.0)
            frames = self.final.get("camera_frames_total", 0)
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

    def _rebuild(self, privileged):
        """(under self.lock, before a start) The episode's cell built again from its seed, in the privileged mode for the
        oracle: OpenArmServer.reset() then lays out the bench plan's nominal layout (grader_config
        oracle_nominal_layout), as the offline grader does for a submission with the oracle token."""
        self._release_cell()
        gc.collect()
        self.server, self.trace = self.build(privileged)

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
                    privileged = True          # the oracle (solution/): the privileged mode, as offline
                else:
                    return self._err(rid, "bad_params", "start takes no parameters")
            if time.time() - float(self.state.get("service_start") or 0.0) > self.prep_s:
                self._log_request("start", {}, error="start_window_closed")
                return self._err(rid, "start_window_closed", "the graded episode had to be started within %.0f s of "
                                                             "the service's start" % self.prep_s)
            if privileged:
                try:
                    self._rebuild(privileged=True)
                except Exception:  # noqa: BLE001
                    self.log.write(kind="privileged_build_failed", tb=traceback.format_exc(limit=20)[-4000:])
                    return self._err(rid, "internal_error", "the privileged cell could not be built; nothing was "
                                                            "started")
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
            now = time.time()
            self.state.update(started=True, start_wall=now, privileged=privileged, snapshot=snap)
            try:
                self.save_state()
            except OSError:
                self.state.update(started=False, start_wall=None, privileged=False, snapshot=None)
                self.log.write(kind="charge_persist_failed", tb=traceback.format_exc(limit=8))
                return self._err(rid, "internal_error", "the start could not be recorded; nothing was started")
            self.started, self.t_start = True, now
            self.server.wall_start = now
            self.requests += 1
            self.last_trace = now
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
            self.call = (time.monotonic(), method)
            try:
                result = srv.handle(method, params)
            except Exception:  # noqa: BLE001 - offline, a cell that raises ends its episode (run_episode: wire_error)
                self.call = None
                self.cell_errors += 1
                self.log.write(kind="cell_error", method=str(method)[:80], tb=traceback.format_exc(limit=20)[-4000:])
                self._log_request(method, params, error="cell_internal_error")
                self._end("done", cell_reason="wire_error")
                return self._err(rid, "internal_error", "the cell failed on this request: the graded episode has ended "
                                                        "(as an offline episode ends when its cell fails)")
            finally:
                self.call = None
            srv.wall_used = time.time() - float(self.t_start)     # as the offline wire keeps it (a statistic)
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
        if not isinstance(params, dict):
            params = {}                        # as the offline wire treats a request whose params are not an object
        return self._cell_call(rid, method, params)

    # -- threads
    def ticker(self):
        """The start window's close (the freeze), the graded window's end, a pending record write a quarter of a second
        after the last change, and the replay record every trace_every_s while the episode runs."""
        every = float(self.cfg["trace_every_s"])
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
                elif time.time() - self.last_trace >= every:
                    if self.lock.acquire(timeout=0.2):
                        try:
                            if not self.ended and self.server is not None:
                                if int(self.server.tick_count) != self.trace_ticks:
                                    self._save_trace()
                                else:
                                    self.last_trace = time.time()
                        finally:
                            self.lock.release()
            except Exception:  # noqa: BLE001
                self.log.write(kind="ticker_error", tb=traceback.format_exc(limit=8))

    def watchdog(self):
        """A cell call that runs past call_timeout_s ends the worker (the supervisor restarts it; the episode then ends
        with its last record, worker_restart). The privileged reference assembler runs inside one call: exempt."""
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
def run_worker(cfg, listener, ready_fd, lab, freeze_chan=None):
    """Forked from the supervisor as root: read the hidden episode's seed (only while it has not started), drop
    privileges, then import the cell and build the episode's cell as the lab account, serve."""
    _set_comm("arbcell-worker")
    os.umask(0o077)
    state_dir = cfg["state_dir"]
    log = Log(os.path.join(state_dir, "service.log"), int(cfg["max_log_bytes"]))
    log.chown(lab.pw_uid, lab.pw_gid)
    try:
        for k, v in cfg["env"].items():
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
            meta = read_json(cfg["episode"], 1 << 16)            # the hidden graded episode (root-only), as root
            seed = int(meta["seed"]) & ((1 << 63) - 1)
            if hashlib.sha256(str(seed).encode("ascii")).hexdigest() != meta["seed_sha256"]:
                raise ValueError("the graded episode's seed does not match its digest")
            public = {"episode": str(meta["episode"]), "seed_sha256": str(meta["seed_sha256"]),
                      "oracle_token_sha256": meta.get("oracle_token_sha256")}
            del meta
            drop_privileges(lab)
            # the cell, as the lab account, from the installed copy of online/cell (root:lab account, 0750 / 0640)
            cell_dir = cfg["cell_dir"]
            sys.path.insert(0, cell_dir)
            mods = {name: importlib.import_module(name) for name in CELL_MODULES}
            cell_cfg = read_json(os.path.join(cell_dir, cfg["cell_config"]), 1 << 20)
            plan = read_json(os.path.join(cell_dir, cfg["bench_plan"]), 8 << 20)
            kit_dir = os.path.join(cell_dir, cfg["kit_dir"])
            tcfg = cell_cfg.get("episode_trace") or {}

            def cell_log(msg):
                log.write(kind="cell", msg=_no_seed(msg)[:8000])

            def build(privileged):
                server = mods["arm.assembly_openarm.server"].OpenArmServer(
                    kit_dir, cfg["panda_dir"], plan, config=cell_cfg, privileged=privileged, render=True, log=cell_log)
                server.reset(seed)
                trace = None
                if tcfg.get("agent", True):
                    try:
                        trace = make_trace(mods["episode_trace"], server, int(tcfg.get("every_ticks", 4)), cell_log)
                    except Exception as e:  # noqa: BLE001 - a record only
                        log.write(kind="trace_not_attached", err=repr(e)[:300])
                return server, trace

            server, trace = build(False)
            svc = Service(cfg, log, reqlog, state, state_path, record_path, server=server, trace=trace, build=build,
                          meta=public, freeze_chan=freeze_chan)
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
    """Copy the agent's own regular files and directories under the open directory `src_fd` into `dst_dir`. Every name
    is resolved relative to an open directory descriptor with O_NOFOLLOW: no symbolic link is followed and no path the
    agent controls is resolved by name; symbolic links, special files, __pycache__, odd names and entries the agent's
    account does not own are skipped."""
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
        if caps.get("uid") is not None and st.st_uid != caps["uid"] and \
                (stat.S_ISDIR(st.st_mode) or stat.S_ISREG(st.st_mode)):
            acc["skipped"] += 1                         # not the agent's own
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
                if caps.get("uid") is not None and os.fstat(sub).st_uid != caps["uid"]:
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
                if not stat.S_ISREG(fst.st_mode) or (caps.get("uid") is not None and fst.st_uid != caps["uid"]):
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
    submission folder in the record directory, taken before the graded start is charged (or when the start window
    closes). Within the offline submission's caps the copy is complete (status ok); beyond them nothing is copied
    (status too_large); a copy that fails is status error (the start is then refused). The copy is built under a
    temporary name and renamed into place. Never reads anything hidden."""
    state_dir = cfg["state_dir"]
    dst = os.path.join(state_dir, cfg["snapshot_name"])
    tmp = dst + ".tmp"
    caps = {"max_bytes": int(cfg["snapshot_max_bytes"]), "max_entries": int(cfg["snapshot_max_entries"]),
            "max_depth": int(cfg["snapshot_max_depth"]), "uid": cfg.get("_agent_uid")}
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
    if not isinstance(cfg["task"], str) or not cfg["task"]:
        raise ValueError("config needs the task's slug")
    for key in ("socket", "state_dir", "cell_dir", "panda_dir", "submission"):
        if not os.path.isabs(cfg[key]):
            raise ValueError("%s must be an absolute path" % key)
    for key in ("cell_config", "kit_dir", "bench_plan"):
        if os.path.isabs(cfg[key]) or ".." in cfg[key].split("/"):
            raise ValueError("%s must be a path inside cell_dir" % key)
    if os.path.dirname(cfg["socket"]) in ("/", "/tmp", "/var/tmp", "/dev/shm", "/run"):
        raise ValueError("the socket must live in its own root-owned directory")
    if not isinstance(cfg["env"], dict) or not all(isinstance(k, str) and isinstance(v, str)
                                                   for k, v in cfg["env"].items()):
        raise ValueError("env must map strings to strings")
    if cfg["snapshot_max_bytes"] is None:       # the offline submission's size cap (main's grader configuration)
        rg = read_json(os.path.join(cfg["cell_dir"], cfg["cell_config"]), 1 << 20).get("route_gate", {})
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
        raise RuntimeError("the lab account must not be root")
    try:
        agent = pwd.getpwnam(cfg["agent_user"])
    except KeyError:
        agent = None
    if agent is not None and agent.pw_uid == lab.pw_uid:
        raise RuntimeError("the lab account must differ from the agent account")
    cfg["_agent_uid"] = agent.pw_uid if agent is not None else None
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
        print("openarm-cell-online: must be started as root (environment/setup/setup.sh)", file=sys.stderr)
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
        state_path = os.path.join(cfg["state_dir"], "state.json")
        if not os.path.lexists(state_path):    # the service's start: the start window counts from here (never reset)
            atomic_write_json(state_path, {"version": 1, "service_start": time.time(), "started": False,
                                           "start_wall": None, "ended": False, "end_reason": None})
            os.chown(state_path, lab.pw_uid, lab.pw_gid)
    except Exception as exc:  # noqa: BLE001
        print("openarm-cell-online: configuration error: %s: %s" % (type(exc).__name__, exc), file=sys.stderr)
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
        print("openarm-cell-online: ready (socket %s)" % cfg["socket"], file=sys.stderr)
        return 0
    print("openarm-cell-online: failed to start (details in the service's own log)", file=sys.stderr)
    return 1


def main(argv=None):
    ap = argparse.ArgumentParser(description="openarm-cell-online (start as root from environment/setup/setup.sh)")
    ap.add_argument("--config", required=True)
    args = ap.parse_args(argv)
    return start(args.config)


if __name__ == "__main__":
    sys.exit(main())
