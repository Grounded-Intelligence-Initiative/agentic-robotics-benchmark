"""Hidden grader for manipulation_failure_diagnosis_during_execution. Runs as root in a separate verifier sandbox.

Only the declared artifact /home/user/submission is restored into this sandbox. The hidden recordings are in the
verifier image (/opt/hidden_episodes, root-only, fetched at build time from tests/assets.sha256, each already cut to
its playback length); the answer key is tests/hidden/key.json (this staged folder, root-only). The grader:

  * stages the submission to the agent account and starts its run.sh through the seccomp launcher (nonet_exec.py):
    only AF_UNIX sockets exist, so the program makes no network or model calls while it is graded;
  * plays each hidden run back in chunks of CHUNK frames over the kit wire: stream_helper.py (on /opt/venv, root)
    decodes the recording one frame at a time, the grader writes each chunk's frames to a fresh file owned by the agent
    account, sends the request, waits for the reply, then deletes the file. Playback of a run ends at its last frame
    or at the program's first alarm;
  * times every reply from the moment the grader starts writing the request to the moment the reply line is read (the
    program's own time; the grader is idle meanwhile) and enforces the per-run real-time allowance. The request's
    deadline starts at the same moment and bounds the write too, so a program that stops reading its input cannot
    hold the grader in a blocked write;
  * scores each run from the key: P (mean timeliness-weighted correct alarm per failure type, averaged over types)
    times (1 - F) (F: fraction of successful runs with an alarm).

Grader-side failures (a missing or corrupt hidden recording or key file, a decoder failure, a failure to write a
chunk) raise TaskError and end the verify stage as a task error: they are never charged to the program. Anything the
program does (no reply, a crash, a malformed alarm, a late reply, falling behind) costs only its own run.

A submission whose `.oracle_token` hashes to oracle_token.sha256 gets the correct reply for each chunk added to its
observation as `_gt`; only the oracle in solution/ has that token.
"""

import fcntl
import hashlib
import hmac
import json
import math
import os
import pwd
import select
import shutil
import signal
import stat
import struct
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

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
from robotics_grader.solver import MAX_BLANK_LINES  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
KEY_PATH = os.path.join(HERE, "hidden", "key.json")
NONET = os.path.join(HERE, "nonet_exec.py")
HELPER = os.path.join(HERE, "stream_helper.py")
HELPER_PYTHON = "/opt/venv/bin/python3"
EPISODES_ROOT = "/opt/hidden_episodes"
EPISODE_FILES = ("episode.json", "video.mp4", "logs.npz")
TOKEN_FILE = ".oracle_token"
TOKEN_MAX_BYTES = 256
MAX_SUBMISSION_BYTES = 300 * 1024 * 1024
MAX_SUBMISSION_FILES = 20000
MAX_LINE = 1 << 16
CLASSES = ("dropped_after_grasp", "grasp_missed", "grasped_wrong_object")
FPS = 20
FRAME_H, FRAME_W = 256, 768
FRAME_BYTES = FRAME_H * FRAME_W * 3
HEADER_KEYS = ("instruction", "candidates", "camera_model", "fps")


class TaskError(Exception):
    """A defect on the grader's side: the verify stage must not score it as the agent's 0."""


def log(msg):
    print("[grader] " + msg, file=sys.stderr, flush=True)


# ------------------------------------------------------------------------------ key and recordings

def load_key():
    try:
        with open(KEY_PATH) as fh:
            key = json.load(fh)
        runs = key["runs"]
        rule = key["rule"]
        chunk = int(rule["chunk_frames"])
        full_s, zero_s = float(rule["full_credit_s"]), float(rule["zero_credit_s"])
        early_s = float(rule["early_tolerance_s"])
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise TaskError("key file unreadable: {}".format(type(exc).__name__))
    if not isinstance(runs, dict) or not runs or chunk < 1 or not 0 <= full_s < zero_s or not 0 <= early_s < full_s:
        raise TaskError("key file malformed")
    for rec in runs.values():
        if rec.get("kind") == "failure":
            if rec.get("failure_type") not in CLASSES or not isinstance(rec.get("subject"), str) \
                    or not 0 <= int(rec.get("onset_frame", -1)) < int(rec.get("stream_frames", 0)):
                raise TaskError("key entry incomplete")
        elif rec.get("kind") != "success" or int(rec.get("stream_frames", 0)) < 1:
            raise TaskError("key entry incomplete")
    return runs, chunk, (full_s, zero_s, early_s)


def load_header(eid):
    try:
        for name in EPISODE_FILES:
            if not os.path.isfile(os.path.join(EPISODES_ROOT, eid, name)):
                raise TaskError("hidden recording file missing")
        with open(os.path.join(EPISODES_ROOT, eid, "episode.json")) as fh:
            ep = json.load(fh)
        return {k: ep[k] for k in HEADER_KEYS}
    except (OSError, ValueError, KeyError) as exc:
        raise TaskError("hidden recording unreadable: {}".format(type(exc).__name__))


# ------------------------------------------------------------------------------ scoring

def timeliness(delay_s, full_s, zero_s, early_s=0.0):
    """Credit for an alarm `delay_s` after the onset: alarms up to `early_s` before it count as at the onset."""
    if delay_s < -early_s:
        return 0.0
    delay_s = max(0.0, delay_s)
    if delay_s <= full_s:
        return 1.0
    if delay_s >= zero_s:
        return 0.0
    return (zero_s - delay_s) / (zero_s - full_s)


def parse_action(action):
    """('none' | 'alarm' | 'malformed', alarm dict or None). Never raises."""
    if isinstance(action, dict) and set(action) == {"alarm"} and action["alarm"] is False:
        return "none", None
    if isinstance(action, dict) and set(action) == {"alarm", "failure_type", "object"} and action["alarm"] is True \
            and action["failure_type"] in CLASSES and isinstance(action["object"], str):
        return "alarm", action
    return "malformed", None


def score_run(rec, outcome, rule):
    """outcome: {'alarm_frame': int or None, 'alarm': dict or None, 'void': bool}. Returns the run's result row."""
    row = {"kind": rec["kind"], "alarm_frame": outcome["alarm_frame"], "void": outcome["void"]}
    alarmed = outcome["void"] or outcome["alarm_frame"] is not None
    if rec["kind"] == "success":
        row.update(alarmed=alarmed, credit=0.0)
        return row
    if outcome["void"] or outcome["alarm_frame"] is None:
        row.update(credit=0.0, false_alarm=False, correct=False, delay_s=None)
        return row
    full_s, zero_s, early_s = rule
    delay = (outcome["alarm_frame"] - int(rec["onset_frame"])) / float(FPS)
    a = outcome["alarm"]
    correct = a is not None and a["failure_type"] == rec["failure_type"] and a["object"] == rec["subject"]
    row.update(false_alarm=delay < -early_s, correct=correct, delay_s=delay,
               credit=(timeliness(delay, full_s, zero_s, early_s) if correct else 0.0))
    return row


def oracle_reply(rec, stop):
    if rec["kind"] == "failure" and stop - 1 >= int(rec["onset_frame"]):
        return {"alarm": True, "failure_type": rec["failure_type"], "object": rec["subject"]}
    return {"alarm": False}


# ------------------------------------------------------------------------------ the agent side

def agent_ids():
    pw = pwd.getpwnam(agent_user())
    return pw.pw_uid, pw.pw_gid


def agent_pids(uid):
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


def stop_agent_processes(uid, rounds=20, pause=0.25):
    """Stop every process of the agent account (bounded). The image's supervisord respawns the desktop, so what
    outlives the bounded kill is frozen and only a still-running one fails the run."""
    for _ in range(rounds):
        pids = agent_pids(uid)
        if not pids:
            return True
        for pid in pids:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        time.sleep(pause)
    deadline = time.monotonic() + 3.0
    while True:
        for pid in agent_pids(uid):
            try:
                os.kill(pid, signal.SIGSTOP)
            except OSError:
                pass
        time.sleep(0.05)
        running = []
        for pid in agent_pids(uid):
            try:
                with open("/proc/{}/status".format(pid)) as f:
                    state = next((line.split()[1] for line in f if line.startswith("State:")), "X")
            except (OSError, IndexError):
                state = "X"
            if state not in ("T", "t", "X"):
                running.append(pid)
        if not running or time.monotonic() > deadline:
            return not running


def stage_submission(uid, gid):
    """Hand the restored submission to the agent account, with size and file-count caps (no link followed)."""
    path = os.path.join(os.path.expanduser("~" + agent_user()), "submission")
    if not os.path.isdir(path) or os.path.islink(path):
        return None
    total = files = 0
    root = os.path.realpath(path)
    for dp, dns, fns in os.walk(path, followlinks=False):
        for fn in dns + fns:
            fp = os.path.join(dp, fn)
            if os.path.islink(fp) and os.path.commonpath([root, os.path.realpath(fp)]) != root:
                return None         # a link that leaves the submission: unusable
        for fn in fns:
            fp = os.path.join(dp, fn)
            files += 1
            try:
                st = os.lstat(fp)
            except OSError:
                continue
            if stat.S_ISREG(st.st_mode):
                total += st.st_size
        if files > MAX_SUBMISSION_FILES or total > MAX_SUBMISSION_BYTES:
            return None
    for dp, dns, fns in os.walk(path, followlinks=False):
        for name in dns + fns:
            os.lchown(os.path.join(dp, name), uid, gid)
    os.lchown(path, uid, gid)
    os.chmod(path, 0o700)
    return path


def token_state(submission, digest_path=None):
    """(privileged, mismatch) for the submission's .oracle_token against oracle_token.sha256."""
    digest_path = digest_path or os.path.join(HERE, "oracle_token.sha256")
    path = os.path.join(submission, TOKEN_FILE)
    try:
        with open(digest_path) as handle:
            want = handle.read().split()[0].strip().lower()
    except (OSError, IndexError) as exc:
        raise TaskError("oracle token digest unreadable: {}".format(type(exc).__name__))
    try:
        fd = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0))
    except OSError:
        return False, os.path.lexists(path)
    try:
        st = os.fstat(fd)
        if not stat.S_ISREG(st.st_mode) or st.st_size > TOKEN_MAX_BYTES:
            return False, True
        data = os.read(fd, TOKEN_MAX_BYTES + 1)
    except OSError:
        return False, True
    finally:
        os.close(fd)
    ok = hmac.compare_digest(hashlib.sha256(data.strip()).hexdigest(), want)
    return ok, not ok


def egress_self_test(uid, gid):
    """True if a probe started through the launcher cannot open an internet socket."""
    probe = ("import socket,sys\n"
             "bad=0\n"
             "for fam in (socket.AF_INET, socket.AF_INET6):\n"
             "    try:\n"
             "        socket.socket(fam, socket.SOCK_STREAM).close(); bad=1\n"
             "    except OSError:\n"
             "        pass\n"
             "sys.exit(3 if bad else 0)\n")
    try:
        r = subprocess.run([sys.executable, "-I", NONET, "--uid", str(uid), "--gid", str(gid), "--cwd", "/",
                            "--", sys.executable, "-I", "-c", probe],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60,
                           env={"PATH": "/usr/bin:/bin"})
        return r.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def write_within(stream, text, deadline, budget):
    """Write `text` to the program's stdin before `deadline`, or abort its run (a non-blocking write under select)."""
    data = memoryview(text.encode("utf-8"))
    try:
        stream.flush()
        fd = stream.fileno()
    except (OSError, ValueError):
        raise SolverAborted("solver stdin closed")
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
    try:
        while data:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SolverAborted("solver did not read the request within its {:g}s deadline".format(budget))
            if not select.select([], [fd], [], remaining)[1]:
                continue
            try:
                data = data[os.write(fd, data):]
            except BlockingIOError:
                continue
            except OSError:
                raise SolverAborted("solver stdin closed")
    finally:
        fcntl.fcntl(fd, fcntl.F_SETFL, flags)


class GuardedSolver(Solver):
    """The kit Solver, started through the seccomp launcher, with a reply-line cap and a deadline that also bounds
    the write of each request."""

    def _spawn(self):
        uid, gid = agent_ids()
        cmd = [sys.executable, "-I", NONET, "--uid", str(uid), "--gid", str(gid), "--cwd", self.submission,
               "--", "bash", self.entry]
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
                line = proc.stdout.readline(MAX_LINE + 1)
            except (ValueError, OSError, UnicodeDecodeError):
                break
            if not line:
                break
            if len(line) > MAX_LINE and not line.endswith("\n"):
                break
            lines.put(line)
        lines.put(None)

    def _exchange(self, message):
        # The kit's exchange as it is, except that the deadline is fixed before the write and bounds it.
        deadline = time.monotonic() + self.step_timeout
        write_within(self.proc.stdin, json.dumps(message) + "\n", deadline, self.step_timeout)
        for _ in range(MAX_BLANK_LINES):
            line = self._readline(deadline, self.step_timeout).strip()
            if not line:
                continue
            try:
                reply = json.loads(line)
            except (ValueError, RecursionError):
                raise SolverAborted("solver wrote non-JSON on the wire: {!r}".format(line[:120]))
            if isinstance(reply, dict):
                return reply
            raise SolverAborted("solver reply is not an object")
        raise SolverAborted("solver wrote {} blank lines to one request without a reply".format(MAX_BLANK_LINES))


# ------------------------------------------------------------------------------ playback

def write_npy(path, data, frames):
    """A NumPy .npy (format 1.0) file of uint8 shape (frames, 256, 768, 3), written with the standard library."""
    header = "{{'descr': '|u1', 'fortran_order': False, 'shape': ({}, {}, {}, 3), }}".format(frames, FRAME_H, FRAME_W)
    pad = 64 - (10 + len(header) + 1) % 64
    header = header + " " * (pad % 64) + "\n"
    with open(path, "wb") as fh:
        fh.write(b"\x93NUMPY\x01\x00" + struct.pack("<H", len(header)) + header.encode("latin1"))
        fh.write(data)


class Playback(object):
    """One hidden recording, read through stream_helper.py; every failure here is a task error."""

    def __init__(self, eid, n):
        self.n = n
        try:
            # The recording and its length go over stdin, never argv: /proc/<pid>/cmdline is world-readable.
            self.proc = subprocess.Popen([HELPER_PYTHON, "-I", HELPER],
                                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                         env={"PATH": "/usr/bin:/bin", "OMP_NUM_THREADS": "1"})
            self.proc.stdin.write(json.dumps({"dir": os.path.join(EPISODES_ROOT, eid), "n": n}).encode() + b"\n")
            self.proc.stdin.close()
            line = self.proc.stdout.readline()
            self.log = json.loads(line)["log"]
        except (OSError, ValueError, KeyError) as exc:
            self.close()
            raise TaskError("playback could not start: {}".format(type(exc).__name__))
        if any(len(v) != n for v in self.log.values()):
            self.close()
            raise TaskError("playback log length mismatch")

    def frames(self, k):
        buf = bytearray(k * FRAME_BYTES)
        view = memoryview(buf)
        got = 0
        while got < len(buf):
            r = self.proc.stdout.readinto(view[got:])
            if not r:
                self.close()
                raise TaskError("playback decoder ended early")
            got += r
        return buf

    def close(self):
        proc = getattr(self, "proc", None)
        if proc is not None and proc.poll() is None:
            proc.kill()
        if proc is not None:
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pass


def play_run(eid, rec, header, solver, uid, gid, workroot, chunk, allowance_s, privileged):
    """Play one run back; returns (outcome, aborted, own_time_s, malformed)."""
    n = int(rec["stream_frames"])
    pb = Playback(eid, n)
    run = tempfile.mkdtemp(prefix="run_", dir=workroot)
    os.chown(run, uid, gid)
    os.chmod(run, 0o700)
    outcome = {"alarm_frame": None, "alarm": None, "void": False}
    own = 0.0
    aborted = malformed = False
    try:
        solver.reset()        # not counted: after an aborted run this restarts the program and waits for READY
        for t, start in enumerate(range(0, n, chunk)):
            stop = min(n, start + chunk)
            data = pb.frames(stop - start)
            path = os.path.join(run, "chunk_{:05d}.npy".format(t))
            try:
                write_npy(path, data, stop - start)
                os.chown(path, uid, gid)
                os.chmod(path, 0o600)
            except OSError as exc:
                raise TaskError("could not write a chunk: {}".format(type(exc).__name__))
            obs = dict(header)
            obs["frames"] = [start, stop]
            obs["images"] = path
            obs["log"] = {k: v[start:stop] for k, v in pb.log.items()}
            if privileged:
                obs["_gt"] = oracle_reply(rec, stop)
            t0 = time.monotonic()
            action = solver.act(t, obs)
            own += time.monotonic() - t0
            try:
                os.unlink(path)
            except OSError:
                pass
            kind, alarm = parse_action(action)
            if own > stop / float(FPS) + allowance_s:
                outcome["void"] = True
                log("run {} fell behind real time at frame {}".format(eid, stop))
                break
            if kind == "none":
                continue
            outcome["alarm_frame"] = stop - 1
            outcome["alarm"] = alarm
            malformed = kind == "malformed"
            break
    except SolverAborted as exc:
        aborted = True
        outcome["void"] = True
        log("run {} aborted: {}".format(eid, str(exc)[:120]))
    finally:
        pb.close()
        shutil.rmtree(run, ignore_errors=True)
    return outcome, aborted, own, malformed


def run_all(config, runs, chunk, rule, uid, gid, workdir, privileged, t_start):
    budget = float(config["grader_budget_s"])
    allowance = float(config["realtime_allowance_s"])
    workroot = tempfile.mkdtemp(prefix="diag_", dir=os.path.dirname(workdir))
    os.chown(workroot, 0, gid)
    os.chmod(workroot, 0o710)
    headers = {eid: load_header(eid) for eid in runs}
    results, counts = {}, {"aborted": 0, "malformed": 0, "behind": 0, "budget": 0}
    own_total = 0.0
    solver = GuardedSolver(step_timeout=float(config["step_timeout_s"]), submission=workdir,
                           max_restarts=int(config["max_restarts"]))
    try:
        for eid in sorted(runs):
            rec = runs[eid]
            if time.monotonic() - t_start > budget:
                counts["budget"] += 1
                results[eid] = score_run(rec, {"alarm_frame": None, "alarm": None, "void": True}, rule)
                continue
            outcome, aborted, own, malformed = play_run(eid, rec, headers[eid], solver, uid, gid, workroot, chunk,
                                                        allowance, privileged)
            own_total += own
            counts["aborted"] += aborted
            counts["malformed"] += malformed
            counts["behind"] += outcome["void"] and not aborted
            results[eid] = score_run(rec, outcome, rule)
            r = results[eid]
            log("run {} {} alarm_frame={} credit={} own={:.1f}s".format(
                eid, rec["kind"], r["alarm_frame"], round(r.get("credit", 0.0), 3), own))
    finally:
        try:
            solver.close()
        except Exception:  # noqa: BLE001 - closing a broken solver must not crash the grader
            pass
        shutil.rmtree(workroot, ignore_errors=True)
    return results, counts, own_total


def aggregate(runs, results):
    per_class = {c: [results[e]["credit"] for e, r in runs.items() if r["kind"] == "failure" and r["failure_type"] == c]
                 for c in CLASSES}
    present = [v for v in per_class.values() if v]
    p = sum(sum(v) / len(v) for v in present) / len(present) if present else 0.0
    succ = [results[e]["alarmed"] for e, r in runs.items() if r["kind"] == "success"]
    f = sum(succ) / len(succ) if succ else 0.0
    return p, f, per_class


def hide_processes():
    """Defence in depth: remount /proc so that other accounts cannot see root's processes (hidepid). Needs
    CAP_SYS_ADMIN, which a container sandbox usually lacks; then the remount fails and nothing else changes (the
    decoder takes its arguments over stdin, so its command line holds nothing about the run). Returns 1.0 if active."""
    for opt in ("hidepid=invisible,gid=0", "hidepid=2,gid=0"):
        try:
            r = subprocess.run(["mount", "-o", "remount," + opt, "/proc"], stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, timeout=30, env={"PATH": "/usr/bin:/bin:/usr/sbin:/sbin"})
            if r.returncode == 0:
                return 1.0
        except (OSError, subprocess.SubprocessError):
            pass
    return 0.0


def main():
    t_start = time.monotonic()
    hidepid = hide_processes()
    config = stage_config()
    anchor, anchor_meta = read_anchor()
    # Grader-side problems raise TaskError and end the stage: not the program's 0.
    runs, chunk, rule = load_key()
    for eid in runs:
        load_header(eid)
    uid, gid = agent_ids()
    if not stop_agent_processes(uid):
        raise TaskError("agent processes could not be stopped before grading")
    try:
        workdir = stage_submission(uid, gid)
    except OSError as exc:
        raise TaskError("staging the submission failed: {}".format(type(exc).__name__))
    if workdir is None or not os.path.isfile(os.path.join(workdir, "run.sh")):
        write_verdict(zero_verdict(reason="no usable submission", anchored=anchor is not None))
        return
    if not egress_self_test(uid, gid):
        raise TaskError("the network block could not be established")
    privileged, mismatch = token_state(workdir)
    log("oracle token: " + ("matched" if privileged else "mismatch, ignored" if mismatch else "absent"))
    try:
        results, counts, own_total = run_all(config, runs, chunk, rule, uid, gid, workdir, privileged, t_start)
    except (SolverMissing, SolverAborted) as exc:
        stop_agent_processes(uid)
        write_verdict(zero_verdict(reason=str(exc)[:200], anchored=anchor is not None))
        return
    cleanup_ok = stop_agent_processes(uid)
    p, f, per_class = aggregate(runs, results)
    measured = p * (1.0 - f)
    fails = [results[e] for e, r in runs.items() if r["kind"] == "failure"]
    extra = {"runs_failure": float(len(fails)), "runs_success": float(len(runs) - len(fails)),
             "P_alarm_score": p, "F_false_alarm_rate_success_runs": f,
             "failure_runs_correct": float(sum(1 for r in fails if r.get("correct"))),
             "failure_runs_false_alarm": float(sum(1 for r in fails if r.get("false_alarm"))),
             "failure_runs_no_alarm": float(sum(1 for r in fails if r["alarm_frame"] is None and not r["void"])),
             "runs_aborted": float(counts["aborted"]), "runs_fell_behind": float(counts["behind"]),
             "alarms_malformed": float(counts["malformed"]), "runs_over_budget": float(counts["budget"]),
             "program_time_total_s": own_total,
             "oracle_token_mismatch": 1.0 if mismatch else 0.0, "privileged_oracle": 1.0 if privileged else 0.0,
             "cleanup_failed": 0.0 if cleanup_ok else 1.0, "proc_hidepid": hidepid}
    for c, v in per_class.items():
        extra["class_score_" + c] = (sum(v) / len(v)) if v else 0.0
    if not cleanup_ok:
        measured = 0.0
    verdict = ratio_rewards(measured, anchor, cap=config.get("scoring_cap", 1.5), higher_is_better=True,
                            metric_name=anchor_meta.get("metric", "online_diagnosis_score"),
                            locks_ok=True, extra=extra, full_at=anchor_meta.get("full_at", 1.0))
    write_verdict(verdict, diagnostic=json.dumps({e: results[e] for e in sorted(results)}, sort_keys=True)[:60000])
    log("measured={:.4f} P={:.4f} F={:.4f} anchor={} reward={:.3f}".format(
        measured, p, f, anchor, verdict["rewards"]["reward"]))


if __name__ == "__main__":
    main()
