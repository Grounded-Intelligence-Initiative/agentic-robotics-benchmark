"""Hidden grader for battery_pack_disassembly_to_modules (closed loop, a fixed hidden set of packs).

Runs as root in a SEPARATE verifier sandbox: no agent process exists there, the setup stage has not run there, and
the engine restores only the declared artifact /home/user/submission. The grader clears the agent's account with the
kit, hands the restored submission (at most `max_submission_mb`) to the agent account in place, starts `bash run.sh`
in it through nonet_exec.py (a seccomp filter: only AF_UNIX sockets) at the agent's UID and speaks the kit's wire
through `pack_grade.run_packs` (the kit Solver: READY within 120 s, each reply within 30 s, at most 3 restarts, at
most `max_processes` processes of the agent's account; the probe lock, `IdleAwareProbeLock`). Before anything else it
reclaims root's PATH directories (robotics_grader.stage.secure_exec_path).

A verdict always: the kit's `VerifierDeadline` is armed at the start with `verifier_deadline_s` (inside the task's
`[verifier] timeout_sec`); if grading is still running then, the packs graded so far count, every other pack counts
0, and `verifier_deadline_expired` says why. A cell that fails while a controller runs is run once more with the
controller stopped; a second failure is scored when the submission can have caused it (pack_grade.run_packs) and is
a task error only when it cannot.

The cell: every pack of the set runs in a fresh process of `packcell/server.py` (Isaac Sim; physics on the CPU, the
GPU renders the two cameras). The process runs as the image's unprivileged account `packcell`, from a copy of
`tests/packcell/` in a folder only that account can read, with Kit's caches and logs there as well: the agent's
controller, another account, can read neither the cell's code nor its memory, and gets observations only through
this grader. Before the first pack a canary cell is started on a pack outside the set and both cameras are checked
(shape, not flat, sane brightness): a simulator or renderer that does not work is the grading environment's fault
(robotics_grader.stage.fail: no reward written), never a 0 for the submission.

Packs: the fixed hidden set tests/hidden/packs.json, the same for every submission: per pack a 64-bit seed with the
variant and the harness case fixed by the set's rule, the camera offsets of the cell it is taken apart in (rule 2) and
the seeds of its fasteners' sealant (rule 3) and of its appearance (rule 4; tests/review/select_set.py), run in the
file's order. The probe schedule of a pack is a function of its index and seed alone (pack_grade.probe_stream), so
the grader adds no randomness of its own.

The metric `mean_pack_credit` is the mean over the set of each pack's credit: the value of the components that lie
undamaged in the rack, on the stand or in the bin they belong in, over the value of every component of the pack
(packcell/rules.py `credit`, read from the physics engine's state after the pack has ended and come to rest).
Touching a live busbar before isolation or dropping a module ends that pack with the credit earned so far; so does a
controller that is given up on. There is no other term.

Oracle token: a submission whose .oracle_token hashes to tests/hidden/oracle_token.sha256 is the reference; only then
does every observation also carry `truth`. An untokened submission never sees it.
"""

import base64
import hashlib
import json
import math
import os
import pwd
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pack_grade as G  # noqa: E402
from packcell import cellproc  # noqa: E402
from packcell import series as S  # noqa: E402
from robotics_grader import (  # noqa: E402
    Solver,
    SolverAborted,
    SolverMissing,
    agent_user,
    ratio_rewards,
    read_anchor,
    stage_config,
    write_verdict,
    zero_verdict,
)
from robotics_grader.account import clear_agent_account  # noqa: E402
from robotics_grader.deadline import VerifierDeadline  # noqa: E402
from robotics_grader.stage import fail, secure_exec_path  # noqa: E402

STAGE = os.path.dirname(os.path.abspath(__file__))
NONET = os.path.join(STAGE, "nonet_exec.py")
TOKEN_SHA = os.path.join(STAGE, "hidden", "oracle_token.sha256")
PACKS = os.path.join(STAGE, "hidden", "packs.json")
MAX_SUBMISSION_FILES = 5000
MAX_REPLY = 1 << 20            # a reply line longer than 1 MiB closes the wire; an action is a few hundred bytes
CANARY = {"seed": 20261124, "variant": "P4", "harness": "lid"}      # not a graded pack
NUMERICS = {"OPENBLAS_CORETYPE": "Haswell", "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"}   # as the image's ENV
ROOT = os.geteuid() == 0


def log(msg):
    print("[grader] " + msg, file=sys.stderr, flush=True)


def _agent_ids():
    pw = pwd.getpwnam(agent_user())
    return pw.pw_uid, pw.pw_gid


def _pids_of(uid):
    pids = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open("/proc/{}/status".format(name)) as f:
                zombie = False
                for line in f:
                    if line.startswith("State:"):
                        zombie = line.split()[1].startswith(("Z", "X"))
                    elif line.startswith("Uid:"):
                        if int(line.split()[1]) == uid and not zombie:
                            pids.append(int(name))
                        break
        except (OSError, ValueError, IndexError):
            continue
    return pids


def kill_processes(uid, rounds=20, pause=0.25):
    """Kill every process of the account; True when none is left (bounded)."""
    for _ in range(rounds):
        pids = _pids_of(uid)
        if not pids:
            return True
        for pid in pids:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        time.sleep(pause)
    return not _pids_of(uid)


class GuardedSolver(Solver):
    """The kit Solver, spawned through the seccomp launcher, with a reply-line cap. Outside the engine (a
    maintainer's run as an ordinary account) `run.sh` is started directly, with the caller's environment."""

    guard = True

    def _spawn(self):
        if not ROOT:
            return subprocess.Popen(["bash", self.entry], cwd=self.submission, stdin=subprocess.PIPE,
                                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, universal_newlines=True,
                                    bufsize=1, start_new_session=True)
        secure_exec_path()               # again before every start: nothing agent-writable ahead of root's tools
        uid, gid = _agent_ids()
        cmd = [sys.executable, "-I", NONET, "--uid", str(uid), "--gid", str(gid), "--cwd", self.submission,
               "--", "bash", self.entry]
        if not self.guard:
            cmd = ["/usr/sbin/runuser", "-u", agent_user(), "--", "/usr/bin/env", "-i",
                   "PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin", "HOME=/home/user", "LANG=C.UTF-8",
                   "bash", self.entry]
        try:
            return subprocess.Popen(cmd, cwd=self.submission, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL, universal_newlines=True, bufsize=1,
                                    start_new_session=True, env={"PATH": "/usr/bin:/bin"})
        except OSError as exc:
            raise SolverAborted("solver could not be started: {}".format(exc))

    @staticmethod
    def _pump(proc, lines):
        while True:
            try:
                line = proc.stdout.readline(MAX_REPLY + 1)
            except (ValueError, OSError, UnicodeDecodeError):
                break
            if not line:
                break
            if len(line) > MAX_REPLY and not line.endswith("\n"):
                break          # an oversize reply: a closed wire (the pack ends there)
            lines.put(line)
        lines.put(None)


def egress_self_test(uid, gid):
    """True if a probe started through the launcher cannot open an internet socket."""
    probe = ("import socket,sys\n"
             "bad=0\n"
             "for fam in (socket.AF_INET, socket.AF_INET6, 40):\n"
             "    try:\n"
             "        socket.socket(fam, socket.SOCK_STREAM).close(); bad=1\n"
             "    except OSError:\n"
             "        pass\n"
             "try:\n"
             "    socket.socket(socket.AF_UNIX, socket.SOCK_STREAM).close()\n"
             "except OSError:\n"
             "    bad=1\n"
             "sys.exit(3 if bad else 0)\n")
    try:
        r = subprocess.run([sys.executable, "-I", NONET, "--uid", str(uid), "--gid", str(gid), "--cwd", "/",
                            "--", sys.executable, "-I", "-c", probe],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60,
                           env={"PATH": "/usr/bin:/bin"})
        return r.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def agent_inet_sockets(uid):
    """Count agent-UID internet sockets with a non-loopback peer (the fallback when the filter cannot be installed)."""
    n = 0
    for name in ("tcp", "tcp6", "udp", "udp6", "raw", "raw6"):
        try:
            with open("/proc/net/" + name) as f:
                next(f, None)
                for line in f:
                    parts = line.split()
                    if len(parts) < 8 or int(parts[7]) != uid:
                        continue
                    rem = parts[2].split(":")[0]
                    if rem.strip("0") == "" or rem in ("0100007F", "00000000000000000000000001000000"):
                        continue
                    n += 1
        except (OSError, ValueError, StopIteration):
            continue
    return n


def agent_files_fill_a_disk(uid, folders, tops=("/home", "/tmp", "/var/tmp", "/dev/shm"), limit=200000):
    """Why the account's files can have filled a filesystem the cell writes to, or None: one of `folders` has less
    than 1 GiB (or a tenth of its size, if smaller) free while the account holds at least 64 MiB of files in the places
    it can write (bounded walk, links not followed)."""
    held, seen = 0, 0
    for top in tops:
        for dp, dns, fns in os.walk(top, followlinks=False):
            for name in fns + dns:
                seen += 1
                try:
                    st = os.lstat(os.path.join(dp, name))
                except OSError:
                    continue
                if st.st_uid == uid and stat.S_ISREG(st.st_mode):
                    held += st.st_blocks * 512
            if seen > limit:
                break
    if held < 64 << 20:
        return None
    for folder in folders:
        try:
            du = shutil.disk_usage(folder)
        except OSError:
            continue
        if du.free < min(1 << 30, du.total // 10):
            return "the controller's files ({} MiB) can have filled the disk: {} has {} MiB free".format(
                held >> 20, folder, du.free >> 20)
    return None


def stage_submission(uid, gid, max_bytes):
    """Make the restored submission the agent account's own folder, in place. Ownership changes never follow a
    symlink; size and file count are capped. Returns the path, or None if there is no usable submission."""
    src = os.path.join(os.environ.get("ALE_HOME") or os.path.expanduser("~" + agent_user()), "submission")
    if not os.path.isdir(src) or os.path.islink(src):
        return None
    total = files = 0
    for dp, _dns, fns in os.walk(src, followlinks=False):
        for fn in fns:
            fp = os.path.join(dp, fn)
            files += 1
            if not os.path.islink(fp):
                try:
                    total += os.lstat(fp).st_size
                except OSError:
                    pass
        if files > MAX_SUBMISSION_FILES or total > max_bytes:
            return None
    if ROOT:
        for dp, dns, fns in os.walk(src, followlinks=False):
            for name in dns + fns:
                os.lchown(os.path.join(dp, name), uid, gid)
        os.lchown(src, uid, gid)
        os.chmod(src, 0o700)
    return src


def oracle_token_ok(workdir):
    """True when the submission's .oracle_token (a regular file, not followed if a link, at most 256 bytes) hashes
    to the recorded value."""
    path = os.path.join(workdir, ".oracle_token")
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except OSError:
        return False
    try:
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            return False
        data = os.read(fd, 257)
    finally:
        os.close(fd)
    if len(data) > 256:
        return False
    with open(TOKEN_SHA) as f:
        want = f.read().strip()
    return hashlib.sha256(data.strip()).hexdigest() == want


def load_set(path=PACKS):
    with open(path) as fh:
        data = json.load(fh)
    packs = data["packs"]
    seeds = [int(p["seed"]) for p in packs]
    if not packs or any(not 0 <= s < 2 ** 64 for s in seeds) or len(set(seeds)) != len(seeds):
        raise RuntimeError("tests/hidden/packs.json: no packs, a seed outside 64 bits or a repeated seed")
    if any(p.get("variant") not in S.VARIANTS or p.get("harness") not in ("none", "lid", "busbars") for p in packs):
        raise RuntimeError("tests/hidden/packs.json: a pack without its variant or harness case")
    for p in packs:
        cams = p.get("cameras")
        if not isinstance(cams, dict) or sorted(cams) != sorted(S.CAMERAS) or any(
                len(v) != 6 or any(abs(float(x)) > S.MOUNT_TOL[k][u] for x, u in zip(v, ["mm"] * 3 + ["deg"] * 3)) for k, v in cams.items()):
            raise RuntimeError("tests/hidden/packs.json: a pack without its cell's camera offsets within series.MOUNT_TOL")
        if not isinstance(p.get("sealant_seed"), int) or not 0 <= p["sealant_seed"] < 2 ** 64:
            raise RuntimeError("tests/hidden/packs.json: a pack without its sealant seed (rule 3)")
        if not isinstance(p.get("appearance_seed"), int) or not 0 <= p["appearance_seed"] < 2 ** 64:
            raise RuntimeError("tests/hidden/packs.json: a pack without its appearance seed (rule 4)")
    return packs


class CellHost(object):
    """Where the cell's processes run: as root, the image's account `packcell` from a private copy of the cell's
    code; as an ordinary account (a maintainer's run), this account from this folder."""

    def __init__(self, account):
        self.count = 0
        if not ROOT:
            self.user, self.server, self.python = None, None, sys.executable
            self.work = tempfile.mkdtemp(prefix="packcell-grade-")
            self.env = dict(os.environ)
            return
        try:
            pw = pwd.getpwnam(account)
        except KeyError:
            fail("the image has no account {!r} for the cell (environment/Dockerfile creates it)".format(account))
        self.user, self.python = account, "/opt/venv/bin/python3"
        base = pw.pw_dir if os.path.isdir(pw.pw_dir) else "/var/tmp"
        self.work = tempfile.mkdtemp(prefix="grade-", dir=base)
        shutil.copytree(os.path.join(STAGE, "packcell"), os.path.join(self.work, "packcell"),
                        ignore=shutil.ignore_patterns("__pycache__"))
        for d in ("kit", "tmp", "optix", "warp", "home"):
            os.makedirs(os.path.join(self.work, d))
        for dp, dns, fns in os.walk(self.work):
            for name in dns + fns:
                os.chown(os.path.join(dp, name), pw.pw_uid, pw.pw_gid)
        os.chown(self.work, pw.pw_uid, pw.pw_gid)
        os.chmod(self.work, 0o700)
        self.server = os.path.join(self.work, "packcell", "server.py")
        self.env = {"HOME": os.path.join(self.work, "home"), "PATH": "/opt/venv/bin:/usr/local/bin:/usr/bin:/bin",
                    "LANG": "C.UTF-8", "OMNI_KIT_ACCEPT_EULA": "YES", "ACCEPT_EULA": "Y", "PRIVACY_CONSENT": "Y",
                    "PACKCELL_KIT_DIR": os.path.join(self.work, "kit"), "TMPDIR": os.path.join(self.work, "tmp"),
                    "OPTIX_CACHE_PATH": os.path.join(self.work, "optix"), "WARP_CACHE_PATH": os.path.join(self.work, "warp"),
                    "NVIDIA_DRIVER_CAPABILITIES": os.environ.get("NVIDIA_DRIVER_CAPABILITIES", ""),
                    "NVIDIA_VISIBLE_DEVICES": os.environ.get("NVIDIA_VISIBLE_DEVICES", "")}
        self.env.update(NUMERICS)        # as the commissioning cell runs (environment/setup/setup.sh)
        self.uid = pw.pw_uid

    def start(self, timeout):
        self.count += 1
        path = os.path.join(self.work, "cell-{}.log".format(self.count))
        self.last_log = path
        err = open(path, "wb")
        if ROOT:
            os.chmod(path, 0o600)
        try:
            return cellproc.CellProcess(grader=True, env=self.env, python=self.python, user=self.user, cwd=self.work,
                                        stderr=err, start_timeout=timeout, server=self.server)
        finally:
            err.close()

    def log_tail(self, n=1500):
        try:
            with open(self.last_log, "rb") as f:
                f.seek(0, 2)
                f.seek(max(0, f.tell() - n))
                return f.read().decode("utf-8", "replace")
        except (OSError, AttributeError):
            return ""

    def cleanup(self):
        if ROOT:
            kill_processes(self.uid)
        shutil.rmtree(self.work, ignore_errors=True)


def canary(host, config):
    """Start a cell on a pack outside the set and check both cameras. Returns the cell's start and reset seconds."""
    import cv2
    import numpy as np
    t0 = time.monotonic()
    try:
        cell = host.start(float(config.get("cell_start_timeout_s", 900)))
    except (cellproc.CellError, OSError) as exc:
        fail("the cell did not start in the grading environment: {} | {}".format(str(exc)[:600], host.log_tail()[-900:]))
    t1 = time.monotonic()
    try:
        pack = S.draw_pack(CANARY["seed"], variant=CANARY["variant"], harness=CANARY["harness"])
        obs = cell.request({"cmd": "reset", "pack": pack, "cams": ["overhead", "wrist"]},
                           timeout=float(config.get("cell_start_timeout_s", 900)))["obs"]
        for name in ("overhead", "wrist"):
            c = S.CAMERAS[name]
            img = cv2.imdecode(np.frombuffer(base64.b64decode(obs["images"][name]["rgb"]), np.uint8), cv2.IMREAD_COLOR)
            dep = cv2.imdecode(np.frombuffer(base64.b64decode(obs["images"][name]["depth"]), np.uint8), cv2.IMREAD_UNCHANGED)
            ok = (img is not None and dep is not None and img.shape == (c["height"], c["width"], 3) and dep.shape == img.shape[:2]
                  and float(img.std()) >= 8.0 and 20.0 <= float(img.mean()) <= 235.0 and float((dep > 0).mean()) > 0.5)
            if not ok:
                fail("the cell's {} camera does not render in the grading environment (shape {}, mean {:.1f}, std {:.1f})".format(
                    name, None if img is None else img.shape, 0.0 if img is None else float(img.mean()),
                    0.0 if img is None else float(img.std())))
        step = cell.request({"cmd": "step", "action": {"move": {"type": "tcp", "delta": [0, 0, -0.02, 0, 0, 0], "ticks": 10}}}, timeout=120.0)
        if "obs" not in step or abs(step["obs"]["tcp"][2] - obs["tcp"][2] + 0.02) > 0.004:
            fail("the cell's robot does not move as commanded in the grading environment: {}".format(str(step)[:300]))
    except cellproc.CellError as exc:
        fail("the cell failed its canary: {} | {}".format(str(exc)[:600], host.log_tail()[-900:]))
    finally:
        cell.close()
    return round(t1 - t0, 1), round(time.monotonic() - t1, 1)


def keep_records(records):
    """Every pack's record (which items count, the cell's events, counts and times; no seed) for maintainers: in the
    engine a root-only folder outside /opt/ale (the engine deletes the staged tests before it keeps a sandbox), in a
    maintainer's own run beside the verdict. Never a reason to fail the grading."""
    try:
        if ROOT:
            folder = "/opt/arb-verify/battery_pack_disassembly_to_modules"
            os.makedirs(folder, mode=0o700, exist_ok=True)
            os.chmod(folder, 0o700)
            path = os.path.join(folder, "records.json")
        else:
            path = (os.environ.get("ROBOTICS_GRADER_VERDICT_PATH") or os.path.join(tempfile.gettempdir(), "verdict.json")) + ".records.json"
        with open(path, "w") as f:
            json.dump(records, f)
        if ROOT:
            os.chmod(path, 0o600)
    except (OSError, TypeError, ValueError) as exc:
        log("the packs' records were not kept: {}".format(type(exc).__name__))


def main():
    t_start = time.monotonic()
    secured = {"root_path_dirs_secured": float(len(secure_exec_path()))}   # before anything else runs
    config = stage_config()
    anchor, meta = read_anchor()
    run = {"records": [], "lock": None, "packs": [], "solver": None, "egress_mode": "none", "truth": False,
           "egress": {"hits": 0}, "clock": G.Clock()}
    written = []
    verdict_lock = threading.Lock()

    def publish(more, zero=False, note=""):
        """Write the verdict from the packs graded so far (a pack of the set that was not graded counts 0), once."""
        with verdict_lock:
            if written:
                return False
            written.append(True)
            records, lock = list(run["records"]), run["lock"]
            credits = [r["credit"] for r in records]
            measured = sum(credits) / max(len(run["packs"]), len(credits), 1)
            if zero or run["egress"]["hits"] or not math.isfinite(measured) or measured < 0:
                measured = 0.0
            extra = dict(lock.rewards() if lock is not None else {},
                         packs_in_set=float(len(run["packs"])),
                         packs_graded=float(len(records)),
                         packs_aborted=float(sum(1 for r in records if r["aborted"])),
                         packs_done=float(sum(1 for r in records if r["ended"] == "done")),
                         packs_live_contact=float(sum(1 for r in records if r["ended"] == "live_contact")),
                         packs_module_dropped=float(sum(1 for r in records if r["ended"] == "module_dropped")),
                         packs_out_of_sim_span=float(sum(1 for r in records if r["ended"] == "sim_cap")),
                         packs_given_up=float(sum(1 for r in records if r["ended"] == "given_up")),
                         packs_cell_failed=float(sum(1 for r in records if r["ended"] == "cell_failed")),
                         cell_retries=float(sum(1 for r in records if r.get("cell_retried"))),
                         protective_stops=float(sum(r.get("protective_stops", 0) for r in records)),
                         fasteners_stripped=float(sum(r.get("stripped", 0) for r in records)),
                         frames=float(sum(r.get("frames", 0) for r in records)),
                         steps=float(sum(r["steps"] for r in records)),
                         egress_guard_seccomp=1.0 if run["egress_mode"] == "seccomp" else 0.0,
                         egress_detected=float(run["egress"]["hits"]),
                         controller_seconds=round(float(run["clock"].s), 1),
                         controller_restarts=float(getattr(run["solver"], "restarts", 0)),
                         probe_residual_true=float(max([r.get("probe_residual_true", 0.0) for r in records] or [0.0])),
                         probe_residual_copy=float(max([r.get("probe_residual_copy", 0.0) for r in records] or [0.0])),
                         probe_copies=float(sum(r.get("probe_copies", 0) for r in records)),
                         probe_true_residual_larger=float(sum(r.get("probe_true_residual_larger", 0) for r in records)),
                         oracle_token=1.0 if run["truth"] else 0.0,
                         **secured)
            extra.update(more)
            for r in records:
                extra["credit_pack_{}".format(r["index"])] = r["credit"]
            verdict = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5), higher_is_better=True,
                                    metric_name="mean_pack_credit", locks_ok=lock.ok if lock is not None else True,
                                    extra=extra, full_at=meta.get("full_at", 1.0))
            keep_records(records)
            lines = ["pack {} ({}): credit {:.4f}, ended {}, {} of {} items placed, {} steps, {} s simulated{}".format(
                r["index"], r["variant"], r["credit"], r["ended"], r.get("placed", 0), r.get("items", 0), r["steps"],
                r.get("sim_s", 0), ", ABORTED: " + r["aborted"] if r["aborted"] else "") for r in records]
            write_verdict(verdict, diagnostic=note + "mean_pack_credit {:.5f}; probes {}/{} (lock {}); ".format(
                measured, lock.passed if lock is not None else 0, lock.total if lock is not None else 0,
                "ok" if lock is None or lock.ok else "FAILED") + "; ".join(lines))
            log("measured={:.5f} anchor={} ratio={:.3f} reward={:.3f} lock={} ({}/{}, {} idle) egress={} "
                "controller_time={:.0f}s wall={:.0f}s{}".format(
                    measured, anchor, verdict["metrics"]["ratio"], verdict["rewards"]["reward"], lock is None or lock.ok,
                    getattr(lock, "passed", 0), getattr(lock, "total", 0), getattr(lock, "idle_skipped", 0),
                    run["egress_mode"], run["clock"].s, time.monotonic() - t_start, " " + note if note else ""))
            return True

    def out_of_time(elapsed):
        # the timer's thread; the process ends (status 0) when this returns
        publish({"verifier_deadline_expired": 1.0, "verifier_deadline_s": round(elapsed, 1)},
                note="the grader's own deadline expired after {:.0f} s with {} of {} packs graded; the others count 0. "
                .format(elapsed, len(run["records"]), len(run["packs"])))
        proc = getattr(run["solver"], "proc", None)
        if proc is not None:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except OSError:
                pass

    VerifierDeadline(float(config.get("verifier_deadline_s", 20400)) - (time.monotonic() - t_start), out_of_time).arm()
    try:
        uid, gid = _agent_ids()
    except KeyError:
        if ROOT:
            write_verdict(zero_verdict(reason="no agent account", anchored=anchor is not None, extra=secured))
            return
        uid, gid = os.getuid(), os.getgid()
    cleared = clear_agent_account()
    if not cleared.get("cleared") and not cleared.get("skipped"):
        fail("the agent's account could not be cleared before grading ({} process(es) keep coming back: {}); the "
             "controller's confinement cannot be relied on in this sandbox".format(
                 len(cleared.get("left", [])), "; ".join(cleared.get("left", []))[:300] or "?"))
    secured["agent_account_cleared"] = 1.0 if cleared.get("cleared") else 0.0
    if ROOT and not kill_processes(uid):
        write_verdict(zero_verdict(reason="agent processes could not be stopped (cleanup_failed)",
                                   anchored=anchor is not None, extra=secured))
        return
    packs = load_set()
    try:
        workdir = stage_submission(uid, gid, int(float(config.get("max_submission_mb", 200)) * 1024 * 1024))
    except OSError as exc:
        log("staging the submission failed: {}".format(type(exc).__name__))
        workdir = None
    if workdir is None or not os.path.isfile(os.path.join(workdir, "run.sh")):
        write_verdict(zero_verdict(reason="no usable submission (no /home/user/submission/run.sh, or more than {} files "
                                   "or {} MB)".format(MAX_SUBMISSION_FILES, config.get("max_submission_mb", 200)),
                                   anchored=anchor is not None, extra=secured))
        return
    truth = oracle_token_ok(workdir)
    egress_mode = "none"
    if ROOT:
        egress_mode = "seccomp" if egress_self_test(uid, gid) else "detect"
    GuardedSolver.guard = egress_mode == "seccomp"
    GuardedSolver.max_processes = int(config.get("max_processes", 64))
    run.update(packs=packs, truth=truth, egress_mode=egress_mode)
    log("egress guard: {}; oracle token: {}; {} packs in the fixed set".format(egress_mode, truth, len(packs)))
    host = CellHost(str(config.get("cell_account", "packcell")))
    egress = run["egress"]
    clock = run["clock"]

    def stop_account():
        return kill_processes(uid) if ROOT else True

    def charge_files():
        return agent_files_fill_a_disk(uid, [host.work, "/", "/tmp", "/var/tmp", "/dev/shm"]) if ROOT else None

    try:
        start_s, reset_s = canary(host, config)
        log("canary passed: the cell started in {} s, built and rendered a pack in {} s".format(start_s, reset_s))
        secured["cell_canary"] = 1.0

        def watch(_index):
            if egress_mode == "detect" and agent_inet_sockets(uid):
                egress["hits"] += 1

        try:
            solver = clock.timed(GuardedSolver, step_timeout=float(config.get("step_timeout_s", 30)), submission=workdir)
            run["solver"] = solver
            try:
                G.run_packs(solver, packs, truth, config,
                            lambda: host.start(float(config.get("cell_start_timeout_s", 900))),
                            log=log, clock=clock, on_pack=watch, stop_account=stop_account,
                            charge_files=charge_files, progress=run)
            finally:
                try:
                    solver.close()
                except Exception:  # noqa: BLE001 - closing a broken solver must not crash the grader
                    pass
        except cellproc.CellError as exc:
            if ROOT:
                kill_processes(uid)
            fail("the cell failed twice on one pack with nothing of the submission running: {} | {}".format(
                str(exc)[:600], host.log_tail()[-900:]))
        except (SolverMissing, SolverAborted) as exc:
            if ROOT:
                kill_processes(uid)
            with verdict_lock:
                if not written:
                    written.append(True)
                    write_verdict(zero_verdict(reason=str(exc)[:200], anchored=anchor is not None, extra=secured))
            return
        watch(len(packs))
    finally:
        host.cleanup()
    cleanup_ok = kill_processes(uid) if ROOT else True
    publish({"cleanup_failed": 0.0 if cleanup_ok else 1.0}, zero=not cleanup_ok)


if __name__ == "__main__":
    main()
