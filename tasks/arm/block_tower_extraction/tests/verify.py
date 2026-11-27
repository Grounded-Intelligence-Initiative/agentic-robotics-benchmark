"""Grader of block_tower_extraction (a Franka Panda relocates loose blocks of a block tower to its top without the
tower collapsing): runs as ROOT in the verify stage. Verification is separate (task.toml): a fresh verifier sandbox
from the task image with only the agent's submission restored into it; tests/test.sh built the root-owned copy of the
givens (/opt/arb) before this starts.

The agent left /home/user/submission/play.py (solve(cell): the cell's program; it sees the cell through three RGB-D
cameras and the robot's own joint / tool / gripper / wrist force-torque state and commands tool poses or joint
targets, the gripper, the pusher rod and the turntable). This grader:
  0. FIRST, stdlib only (`_early_lockdown`): records the upload time, clears the agent's account with the benchmark
     kit, kills what is left of the account and keeps it stopped until the verdict is written, locks the stage,
     snapshots the submission root-owned, gates late writes;
  1. checks the root-owned copy of the givens at /opt/arb against given_hashes.json, the numerics fingerprint
     (tests/numerics_check.py against tests/numerics.json) and that the no-network filter works;
  2. gates: contract_ok = play.py is a regular file that parses and defines solve, the route gate, every .py file
     readable by the parser, the network audit (static and runtime: cellkit.nonet), the module imports under the run
     regime (120 s, 120 CPU-s), >= 1 step call, no surviving process, no remote socket;
  3. the oracle token: submission/.oracle_token whose sha256 equals hidden/oracle_token.sha256 switches the cells to
     the privileged mode (get_state_privileged, contacts_above); a wrong token is ignored;
  4. runs the program once per hidden tower (the fixed set of tests/hidden/, each package pinned by its sha256), the
     towers IN PARALLEL, each in its own process, as its own runner account (btrun1..4, so stopping, killing and
     sweeping one run never touches another), through the wire (cellkit.wire.serve_episode): the program's own time
     plus a price per call and per frame against the episode's budget, one silence capped, a wall-clock cap per
     episode, CPU capped per process, the program stopped (SIGSTOP) while the cell works. An episode in which the
     cell's own code failed is run once more; a second failure is a task error. Every way the program can end ends
     its own episode WITH a score: the tower as it stands after the final settle;
  5. criterion: block_score = the mean over the hidden towers of the tower score (towercell.cell: 0 when the tower
     collapsed, else the blocks legally relocated to its top / N_cap), times the contract gate; reward = block_score
     against the recorded reference value (anchor.json value, full_at; robotics_grader.ratio_rewards).
A failure on the grader's side is a TASK ERROR (`fail`), never the agent's zero.
Untouched sandbox: no play.py -> contract_ok 0 -> 0. Oracle: the token + the reference program -> the recorded value.
Records: out/towers.json and logs/<tag>.log under the root-only /opt/arb-verify (outside /opt/ale).
"""

import ast
import json
import os
import pwd
import shutil
import stat as statmod
import subprocess
import sys
import time
import traceback

sys.dont_write_bytecode = True

STAGE_DIR = os.environ.get("ALE_STAGE_DIR", os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, STAGE_DIR)

HOME = os.environ.get("ALE_HOME", "/home/user")
AGENT_USER = os.path.basename(HOME.rstrip("/")) or "user"
SUBMISSION = os.path.join(HOME, "submission")
ROOT_PAYLOAD = os.environ.get("ARB_ROOT_PAYLOAD", "/opt/arb")
ROOT_GIVEN = os.path.join(ROOT_PAYLOAD, "given")
RUNS_ROOT = os.path.join(ROOT_PAYLOAD, "runs")
SNAPSHOT = os.path.join(RUNS_ROOT, "_submission_snapshot")
QUARANTINE = os.path.join(RUNS_ROOT, "_quarantine")
STICKY_DIRS = [d for d in ("/tmp", "/dev/shm", "/var/tmp", "/run/lock") if os.path.isdir(d)]
WATCH_ROOTS = STICKY_DIRS + [HOME]
LATE_GATE_MAX_GAP_S = 600.0
# the image's desktop session (the base image's supervisord: Xvnc, D-Bus, GNOME, PulseAudio) keeps writing these under
# /tmp as the agent's account from the moment the sandbox boots, also in the separate verifier sandbox where nothing of
# the agent ever ran (measured: /tmp/runtime-user/dconf/user rewritten 25 s after boot). They are the image's, not late
# writes of the agent's session: the late-write gate skips them (the account is cleared and frozen below anyway).
DESKTOP_SESSION_PATHS = tuple(os.path.join("/tmp", n) for n in (
    "runtime-" + (os.path.basename(HOME.rstrip("/")) or "user"), ".X11-unix", ".ICE-unix", ".X1-lock", ".X0-lock",
    "dbus-session-bus-address", "logind-tmpl.log", "logind-mock.log", "gnome-ready"))

from cellkit import lockdown, runner, verify_freeze  # noqa: E402  (stdlib only)

FREEZER = [None]        # the agent account's verify_freeze.AgentFreezer: started by _early_lockdown, stopped after the verdict


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def _early_lockdown():
    out = {"agent_account_cleared": 0.0, "agent_account_programs_disabled": 0.0, "agent_account_clear_killed": 0.0,
           "agent_processes_killed": 0.0, "agent_processes_left": 0.0, "stage_locked": 0.0, "stage_parent_locked": 0.0,
           "stage_lock_latency_ms": -1.0, "submission_late_writes": 0.0, "late_writes_quarantined": 0.0, "late_write_gate_active": 0.0,
           "submission_snapshot": 0.0, "submission_snapshot_skipped": 0.0, "saved": None, "parent": None, "t_ref": None,
           "late": [], "reasons": []}
    if os.getuid() != 0:
        return out
    t_ref = lockdown.stage_upload_time(STAGE_DIR)
    out["t_ref"] = t_ref
    # The benchmark kit's clearing of the agent's account, before anything runs as an account (this grader starts the
    # programs with a launcher of its own, cellkit.runner, so it asks for the clearing itself, as the kit says): once
    # per sandbox, a few seconds. The verify stage needs nothing it removes: the cell renders offscreen (classic depth
    # on OSMesa, Filament colour on Vulkan through lavapipe, no display, no D-Bus), the program under test runs as
    # `nobody` (or as the agent's account, started by this process and so never touched by the clearing), and the
    # engine reads the verdict through its own guest service, which supervisord does not run.
    try:
        from robotics_grader import clear_agent_account
        rep = clear_agent_account(user=AGENT_USER)
    except Exception as exc:  # noqa: BLE001 - the kill and the freezer below still hold the account
        rep = {"cleared": False, "skipped": "the clearing raised %s" % type(exc).__name__}
    out["agent_account_cleared"] = 1.0 if rep.get("cleared") else 0.0
    out["agent_account_programs_disabled"] = float(len(rep.get("programs_disabled") or []))
    out["agent_account_clear_killed"] = float(rep.get("killed") or 0)
    if rep.get("skipped"):
        log("[grader] the agent account was not cleared: %s" % rep["skipped"])
    try:
        uid = pwd.getpwnam(AGENT_USER).pw_uid
    except KeyError:
        uid = None
    if uid is not None and uid != 0:
        killed, left = lockdown.stop_and_kill_uid(uid, grace_s=8.0)
        out["agent_processes_killed"], out["agent_processes_left"] = float(killed), float(left)
        # a survivor is a statistic, not a contract failure: verification is separate (task.toml), nothing of the
        # agent's runs in this sandbox, and the image's supervisord respawns the account's desktop as fast as it is
        # killed, so a process counted here is the image's own. The freezer below keeps every process of the account
        # stopped until the verdict is written, whatever the count.
        if left:
            log("[grader] %d process(es) of the agent account (the image's respawned desktop) outlived the kill at verify start: "
                "frozen until the verdict" % left)
    # the image's init respawns the agent account's desktop within seconds of the kill, and the respawned session runs
    # the account's autostart entries: every process of the account stays SIGSTOPped (polled every 0.1 s) until the
    # verdict is written
    FREEZER[0] = verify_freeze.AgentFreezer.for_user(AGENT_USER).start()
    out["saved"] = runner.lock_stage(STAGE_DIR)
    out["stage_locked"] = 1.0 if out["saved"] is not None else 0.0
    parent = os.path.dirname(os.path.abspath(STAGE_DIR))
    if parent and parent not in ("/", os.path.abspath(HOME)) and os.path.isdir(parent):
        try:
            mode = statmod.S_IMODE(os.lstat(parent).st_mode)
            os.chmod(parent, 0o700)
            saved_mode = os.environ.get("ARB_STAGE_PARENT_MODE")
            out["parent"] = (parent, int(saved_mode, 8) if saved_mode else mode)
            out["stage_parent_locked"] = 1.0
        except (OSError, ValueError):
            pass
    gate_active = t_ref is not None
    cfg_json = os.path.join(STAGE_DIR, "config.json")
    if gate_active and os.path.isfile(cfg_json):
        try:
            gate_active = (os.lstat(cfg_json).st_ctime - t_ref) <= LATE_GATE_MAX_GAP_S
        except OSError:
            pass
    out["late_write_gate_active"] = 1.0 if gate_active else 0.0
    if os.path.isdir(SUBMISSION):
        if gate_active:
            out["late"] = lockdown.late_entries(SUBMISSION, t_ref)
            out["submission_late_writes"] = float(len(out["late"]))
            if out["late"]:
                out["reasons"].append("%d entr%s of the submission written after the agent phase ended (%s)" % (
                    len(out["late"]), "y" if len(out["late"]) == 1 else "ies", ", ".join(e[0] for e in out["late"][:5])))
        os.makedirs(RUNS_ROOT, exist_ok=True)
        os.chmod(RUNS_ROOT, 0o755)
        # the snapshot is where symbolic links and special files are dropped (the per-run copies of the snapshot see
        # none, so their count alone read 0)
        out["submission_snapshot_skipped"] = float(lockdown.snapshot_dir(SUBMISSION, SNAPSHOT) or 0)
        out["submission_snapshot"] = 1.0
    if gate_active:
        q = 0
        for root in WATCH_ROOTS:
            late = lockdown.late_entries(root, t_ref, exclude=(SUBMISSION, RUNS_ROOT, STAGE_DIR) + DESKTOP_SESSION_PATHS)
            if late:
                log("[grader] %d late entr%s under %s quarantined: %s" % (
                    len(late), "y" if len(late) == 1 else "ies", root, ", ".join(e[0] for e in late[:10])))
                q += lockdown.quarantine(root, late, QUARANTINE)
        out["late_writes_quarantined"] = float(q)
    out["stage_lock_latency_ms"] = lockdown.process_age_ms()
    return out


EARLY = _early_lockdown() if __name__ == "__main__" else {"saved": None, "parent": None, "reasons": [], "late": [], "t_ref": None}

import hashlib  # noqa: E402
import math  # noqa: E402

from cellkit import netaudit, nonet, wire  # noqa: E402
from cellkit.pauser import Pauser, session_pids, uid_pids  # noqa: E402

HIDDEN = os.path.join(STAGE_DIR, "hidden")
RECORD_ROOT = os.environ.get("ARB_RECORD_ROOT") or ("/opt/arb-verify" if os.getuid() == 0 else os.path.join(STAGE_DIR, "_record"))
LOGS = os.path.join(RECORD_ROOT, "logs")
OUT = os.path.join(RECORD_ROOT, "out")
PROGRAM = "play"
CRITERIA = ["block_score"]
GATES = ["contract_ok"]
IMPORT_WALL_S = 120
IMPORT_CPU_S = 120
OOM_FIRST = ["choom", "-n", "1000", "--"] if shutil.which("choom") and os.getuid() == 0 else []
PARSE_ERRORS = (SyntaxError, RecursionError, MemoryError, ValueError)
SUSPICIOUS = ("/opt/ale", "ALE_STAGE_DIR", "hidden/seeds", "hidden/towers", "oracle_token", "get_state_privileged",
              "contacts_above")
ROUTE_GATE = {"forbidden_calls": [], "forbidden_extensions": [".pt", ".onnx", ".safetensors", ".ckpt", ".h5", ".pb",
                                                                ".pkl", ".npz"], "max_submission_mb": 60}
MAX_RERUNS_PER_VERIFY = 1


def record_dir(path):
    os.makedirs(RECORD_ROOT, exist_ok=True)
    if RECORD_ROOT != STAGE_DIR and os.getuid() == 0:
        os.chmod(RECORD_ROOT, 0o700)
    os.makedirs(path, exist_ok=True)
    return path


def program_source():
    return SNAPSHOT if os.getuid() == 0 and os.path.isdir(SNAPSHOT) else SUBMISSION


def fail(reason):
    log("[grader] TASK ERROR: " + reason)
    raise SystemExit(1)


def sha256_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def load_json(p):
    with open(p) as f:
        return json.load(f)


def _num(v, default=0.0):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return float(default)
    return f if math.isfinite(f) else float(default)


def root_payload_ok(expected):
    problems = []
    owner = 0 if os.getuid() == 0 else os.getuid()
    if not os.path.isdir(ROOT_PAYLOAD):
        return False, ["%s missing (tests/test.sh builds it)" % ROOT_PAYLOAD]
    files = expected.get("files", expected) if isinstance(expected, dict) else {}
    if not files:
        return False, ["given_hashes.json lists no file"]
    for rel, digest in files.items():
        p = os.path.join(ROOT_PAYLOAD, rel)
        try:
            st = os.lstat(p)
        except OSError:
            problems.append("missing " + rel)
            continue
        if os.path.islink(p) or not statmod.S_ISREG(st.st_mode) or st.st_uid != owner or (st.st_mode & 0o022):
            problems.append("not an owner-only-writable regular file: " + rel)
            continue
        if sha256_file(p) != digest:
            problems.append("differs " + rel)
    return not problems, problems


def hidden_towers():
    """The fixed hidden set: [(package dict, sha256, tag)] in the order hidden/seeds.json pins them; each package is
    checked against its recorded sha256 (a missing or changed package is a task error)."""
    pinned = load_json(os.path.join(HIDDEN, "seeds.json"))
    out = []
    for i, ent in enumerate(pinned.get("towers", [])):
        p = os.path.join(HIDDEN, ent["file"])
        if not os.path.isfile(p):
            fail("hidden tower %d missing (%s)" % (i + 1, ent["file"]))
        h = sha256_file(p)
        if h != ent["sha256"]:
            fail("hidden tower %d does not match its recorded sha256" % (i + 1))
        out.append((load_json(p), h, "ep%02d" % (i + 1)))
    if not out:
        fail("the hidden set is empty")
    return out


def route_gate(cfg, src):
    rg = dict(ROUTE_GATE)
    rg.update(cfg.get("route_gate") or {})
    forb_ext = tuple(rg.get("forbidden_extensions", []))
    total, problems, suspicious = 0, [], 0
    for dp, _, fns in os.walk(src):
        for fn in fns:
            p = os.path.join(dp, fn)
            try:
                total += os.lstat(p).st_size
            except OSError:
                continue
            if fn.lower().endswith(forb_ext):
                problems.append("weight / data file %s" % os.path.relpath(p, src))
            if fn.endswith((".py", ".sh")) and not os.path.islink(p):
                try:
                    text = open(p, encoding="utf-8", errors="replace").read()
                except OSError:
                    continue
                suspicious += sum(1 for line in text.splitlines() if any(s in line for s in SUSPICIOUS))
    if total > rg.get("max_submission_mb", 60) * 1024 * 1024:
        problems.append("submission larger than %d MB" % rg.get("max_submission_mb", 60))
    return problems, suspicious


def library_digests():
    out = set()
    d = os.path.join(ROOT_GIVEN, "lib", "towercell")
    for dp, _dn, fns in os.walk(d):
        for fn in fns:
            if fn.endswith(".py"):
                out.add(sha256_file(os.path.join(dp, fn)))
    return out


class Runner(object):
    def __init__(self, cfg, account=None):
        self.acct = runner.runner_account(account or cfg.get("runner_account", "nobody"), AGENT_USER)
        self.name, self.uid, self.gid = self.acct if self.acct else (None, None, None)
        self.lib_dirs = {"towercell": os.path.join(ROOT_GIVEN, "lib", "towercell")}
        self.cpu_factor = float(cfg.get("cpu_cap_factor", 4.0))
        self.killed = self.left = self.unclean_runs = self.stray_files = self.symlinks_skipped = 0
        os.makedirs(RUNS_ROOT, exist_ok=True)
        os.chmod(RUNS_ROOT, 0o755)

    def prepare(self, tag, program_dir):
        root = os.path.join(RUNS_ROOT, tag)
        if os.path.isdir(root):
            shutil.rmtree(root, ignore_errors=True)
        owner = (self.uid, self.gid) if self.acct else None
        dirs = runner.prepare_run_dir(root, program_dir, self.lib_dirs, owner=owner)
        self.symlinks_skipped = max(self.symlinks_skipped, dirs["skipped"])
        return dirs

    def env(self, dirs):
        # the program's own environment carries the image's numerics pin through the venv's .pth (instruction.md)
        return runner.child_env(dirs["home"], dirs["tmp"], dirs["lib"], ROOT_GIVEN, threads=1)

    def snapshot(self, exclude):
        return {r: runner.snapshot_tree(r, exclude) for r in WATCH_ROOTS}

    def kill_runner_processes(self, grace_s=8.0):
        if not self.acct:
            return 0
        killed, left = lockdown.stop_and_kill_uid(self.uid, grace_s=grace_s)
        self.killed += killed
        self.left += left
        if left:
            self.unclean_runs += 1
        return left

    def pauser(self, proc):
        if self.acct:
            uid = self.uid
            return Pauser(lambda: uid_pids(uid))
        sid = proc.proc.pid
        return Pauser(lambda: session_pids(sid))

    def cleanup(self, dirs, before):
        self.kill_runner_processes()
        if before is not None and self.acct:
            for r in WATCH_ROOTS:
                after = runner.snapshot_tree(r, (dirs["root"],))
                new, modified = runner.tree_changes(before[r], after, self.uid)
                self.stray_files += len(new) + len(modified)
                runner.sweep_new_entries(r, before[r], after, self.uid, (dirs["root"],))
        shutil.rmtree(dirs["root"], ignore_errors=True)


def _parse_error(e):
    return str(e) if isinstance(e, SyntaxError) else "%s: %s" % (type(e).__name__, e)


def unparsable_files(src):
    out = []
    for dp, dns, fns in os.walk(src):
        dns[:] = sorted(d for d in dns if d != "__pycache__")
        for fn in sorted(fns):
            if not fn.endswith(".py"):
                continue
            p = os.path.join(dp, fn)
            if os.path.islink(p):
                continue
            try:
                data = open(p, "rb").read()
            except OSError:
                continue
            try:
                ast.parse(data.decode("utf-8", "replace"))
            except SyntaxError:
                continue
            except PARSE_ERRORS as e:
                out.append((os.path.relpath(p, src), _parse_error(e)[:200]))
    return out


def contract_check(cfg, rn):
    src = program_source()
    path = os.path.join(src, PROGRAM + ".py")
    if not os.path.isfile(path) or os.path.islink(path):
        return False, "no %s.py (regular file) in the submission" % PROGRAM, 0, []
    try:
        tree = ast.parse(open(path, encoding="utf-8", errors="replace").read())
    except PARSE_ERRORS as e:
        return False, "%s.py does not parse: %s" % (PROGRAM, _parse_error(e)), 0, []
    has_solve = any(isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name == "solve" for n in ast.walk(tree)) or \
        any(isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "solve" for t in n.targets) for n in ast.walk(tree)) or \
        any(isinstance(n, (ast.Import, ast.ImportFrom)) and any((a.asname or a.name) == "solve" for a in n.names) for n in ast.walk(tree))
    if not has_solve:
        return False, "%s.py defines no solve" % PROGRAM, 0, []
    problems, suspicious = route_gate(cfg, src)
    if problems:
        return False, "route gate: " + "; ".join(problems[:5]), suspicious, []
    bad = unparsable_files(src)
    if bad:
        return False, "a .py file the parser cannot read: %s" % "; ".join("%s (%s)" % x for x in bad[:5]), suspicious, []
    try:
        net = netaudit.scan_imports(src, allow_digests=library_digests())
    except PARSE_ERRORS as e:
        return False, "a .py file the network audit's parser cannot read (%s)" % _parse_error(e)[:200], suspicious, []
    if net:
        return False, "network audit: networking import(s) %s" % ", ".join("%s: %s" % x for x in net[:5]), suspicious, net
    dirs = rn.prepare("import_check", src)
    before = rn.snapshot((dirs["root"],)) if rn.acct else None
    code = ("import sys, importlib; sys.path.insert(0, sys.argv[1]); sys.path.insert(0, sys.argv[2]); "
            "m = importlib.import_module('%s'); assert callable(m.solve)" % PROGRAM)
    argv = runner.wrap_limits(OOM_FIRST + [sys.executable] + runner.PYTHON_FLAGS + ["-c", code, dirs["lib"], dirs["program"]],
                              cpu_s=IMPORT_CPU_S, user=rn.name, nonet=True)
    try:
        r = subprocess.run(argv, cwd=dirs["program"], env=rn.env(dirs), capture_output=True, text=True,
                           timeout=IMPORT_WALL_S, start_new_session=True)
        ok = r.returncode == 0
        why = "ok" if ok else "importing %s.py failed: " % PROGRAM + (r.stderr.strip().splitlines() or ["?"])[-1][:300]
    except subprocess.TimeoutExpired:
        ok, why = False, "importing %s.py took more than %d s" % (PROGRAM, IMPORT_WALL_S)
    finally:
        rn.cleanup(dirs, before)
    return ok, why, suspicious, []


def token_state():
    p = os.path.join(program_source(), ".oracle_token")
    want_p = os.path.join(HIDDEN, "oracle_token.sha256")
    if not os.path.isfile(p) or os.path.islink(p) or not os.path.isfile(want_p):
        return False, False
    try:
        tok = open(p, "rb").read().strip()
        want = open(want_p).read().strip()
    except OSError:
        return False, False
    got = hashlib.sha256(tok).hexdigest()
    return (got == want), (got != want)


def check_numerics():
    """tests/numerics_check.py in a fresh interpreter against tests/numerics.json: a machine that does not reproduce
    the recorded fingerprint would run every program along another trajectory, so it must not grade (a task error)."""
    want = load_json(os.path.join(STAGE_DIR, "numerics.json"))["fingerprint"]
    try:
        r = subprocess.run([sys.executable, "-I", "-B", os.path.join(STAGE_DIR, "numerics_check.py")], cwd=STAGE_DIR,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=300, env=dict(os.environ))
        rep = json.loads(r.stdout.decode("utf-8", "replace").strip().splitlines()[-1])
        got = str(rep["fingerprint"])
    except (OSError, subprocess.SubprocessError, ValueError, KeyError, IndexError) as exc:
        fail("the numerics check did not run: %s: %s" % (type(exc).__name__, str(exc)[:200]))
    if got != want:
        fail("this machine does not reproduce the recorded numerics (tests/numerics.json): got %s, recorded %s; "
             "parts %s" % (got[:16], want[:16], rep.get("parts")))
    return got


def _new_cell(cfg, package, privileged):
    from towercell.server import CellServer
    from towercell.tower import draw_tower
    tower = draw_tower(int(package["seed"]), package.get("overrides"))
    duration = float(package.get("duration_s", 2400.0))
    if os.getuid() != 0 and os.environ.get("BT_DEV_DURATION_S"):
        duration = float(os.environ["BT_DEV_DURATION_S"])       # authoring runs only (never as root, never graded)
    return CellServer(tower, duration_s=duration, privileged=privileged,
                      panda_dir=os.environ.get("BT_PANDA_DIR") or os.path.join(ROOT_GIVEN, "robot", "panda"))


def run_episode(cfg, tower, rn, privileged, tag):
    package, sha, name = tower
    try:
        server = _new_cell(cfg, package, privileged)
    except Exception:  # noqa: BLE001
        fail("%s: the cell could not be built\n%s" % (tag, traceback.format_exc()))
    b = cfg.get("budgets", {})
    program_cap = float(server.program_budget_s)
    cpu_s = rn.cpu_factor * program_cap
    silence_s = float(server.silence_s)
    start_s = float(b.get("start_s", wire.START_S))
    frame_price = float(b.get("frame_price_s", 0.3))
    call_price = float(b.get("call_price_s", 0.005))
    wall_cap = float(b.get("episode_wall_cap_s", 2700.0))
    record_dir(LOGS)
    t0 = time.time()
    dirs = rn.prepare(tag, program_source())
    before = rn.snapshot((dirs["root"],)) if rn.acct else None
    argv = OOM_FIRST + runner.launcher_argv(sys.executable, os.path.join(dirs["lib"]), dirs["program"], PROGRAM)
    proc = None
    sampler = netaudit.SocketSampler(uid=rn.uid if rn.acct else None, pids_fn=None if rn.acct else (lambda: []),
                                     period_s=float(cfg.get("socket_sample_s", 0.5)))
    reason = None
    pauser = None
    with sampler:
        try:
            try:
                proc = wire.AgentProcess(argv, cwd=dirs["program"], user=rn.name, env=rn.env(dirs), cpu_s=cpu_s,
                                         stderr_path=os.path.join(LOGS, "%s.log" % tag), nonet=True)
                pauser = rn.pauser(proc)
            except Exception:  # noqa: BLE001
                fail("%s: the program could not be launched\n%s" % (tag, traceback.format_exc()))
            try:
                reason = wire.serve_episode(server, proc, program_cap, silence_s, log=log, start_s=start_s,
                                            pauser=pauser, frame_price_s=frame_price, call_price_s=call_price,
                                            wall_cap_s=wall_cap)
            except Exception:  # noqa: BLE001
                fail("%s: the wire failed\n%s" % (tag, traceback.format_exc()))
        finally:
            if pauser is not None:
                pauser.thaw()
            if proc is not None:
                proc.terminate()
            rn.cleanup(dirs, before)
    try:
        if not server.done:
            server._end(reason or "crashed")
        summary = server.summary()
    except Exception:  # noqa: BLE001
        fail("%s: the tower could not be scored\n%s" % (tag, traceback.format_exc()))
    try:
        sc = float(summary["tower_score"])
    except (TypeError, ValueError, KeyError):
        sc = float("nan")
    if not (math.isfinite(sc) and 0.0 <= sc <= 1.0):
        fail("%s: the tower's score %r is not a finite number in [0, 1]" % (tag, summary.get("tower_score")))
    summary["tower_score"] = sc
    summary["episode_wall_s"] = time.time() - t0
    summary["end_reason"] = reason
    summary["tag"] = name if os.getuid() != 0 else tag
    summary["program_cap_s"] = program_cap
    summary["pauses"] = pauser.pauses if pauser is not None else 0
    summary["remote_sockets_seen"] = int(sampler.max_seen)
    try:
        server.close()
    except Exception:  # noqa: BLE001
        pass
    return summary


def _cell_failed(s):
    """The grader's own faults in an episode ([] when none): an exception in the cell's code while it answered a call
    whose parameters it had accepted. A physics fault (MuJoCo's instability or a non-finite state after a step) ends
    only its tower, scored as the cell stands: the program's commands can drive the simulation there."""
    out = []
    if int(_num(s.get("internal_errors"))) > 0:
        out.append("internal_error")
    return out


def _episode_with_retry(cfg, tower, rn, privileged, tag, metrics):
    s_a = run_episode(cfg, tower, rn, privileged, tag=tag)
    why = _cell_failed(s_a)
    if not why:
        return s_a
    if metrics.get("episode_retries", 0.0) >= MAX_RERUNS_PER_VERIFY:
        fail("%s: the cell failed (%s) in a second episode of this verify" % (tag, ", ".join(why)))
    log("[grader]   %s: the cell failed (%s): the episode is run once more" % (tag, ", ".join(why)))
    metrics["episode_retries"] = metrics.get("episode_retries", 0.0) + 1.0
    s_b = run_episode(cfg, tower, rn, privileged, tag=tag + "r")
    s_b["retried_after"] = why
    return s_b


def write_verdict(rewards, metrics, contract_why=None):
    from robotics_grader import ratio_rewards, read_anchor
    from robotics_grader import write_verdict as kit_write_verdict
    overall = float(rewards["block_score"])
    anchor, meta = read_anchor()
    extra = {}
    for k, v in metrics.items():
        try:
            extra[k] = float(v)
        except (TypeError, ValueError):
            continue
    extra.update({k: float(rewards[k]) for k in GATES + CRITERIA})
    verdict = ratio_rewards(overall, anchor, higher_is_better=meta.get("direction", "higher") == "higher",
                            metric_name=meta.get("metric", "block_score"), extra=extra,
                            full_at=meta.get("full_at", 1.0))
    text = ("contract_ok %.0f (the program imports, exposes solve, passes the route gate and the network audit and steps "
            "the cell; no surviving process); block_score %.4f (the mean over the hidden towers of: 0 when the tower "
            "collapsed, else the blocks legally relocated to its top / N_cap)") % (rewards["contract_ok"],
                                                                                   rewards["block_score"])
    if contract_why and not rewards["contract_ok"]:
        text += "; the contract failed: %s" % str(contract_why)[:300]
    if os.environ.get("ALE_VERIFY_PYTHON"):
        recorded = {}
        for k, v in sorted(verdict["metrics"].items()):
            try:
                v = float(v)
            except (TypeError, ValueError):
                continue
            if v == v and abs(v) != float("inf"):
                recorded[k.replace(".", "_").replace("/", "_")] = v
        kit_write_verdict({"rewards": verdict["rewards"], "metrics": recorded}, diagnostic=text)
        return overall
    out = os.environ.get("ARB_LOCAL_VERDICT") or os.path.join(STAGE_DIR, "verdict.local.json")
    kit_write_verdict(verdict, path=out, diagnostic=text)
    return overall


def _restore_modes():
    runner.unlock_stage(EARLY.get("saved"))
    mode = os.environ.get("ARB_STAGE_MODE")
    if mode:
        try:
            os.chmod(STAGE_DIR, int(mode, 8))
        except (OSError, ValueError):
            pass
    parent = EARLY.get("parent")
    if parent:
        try:
            os.chmod(parent[0], parent[1])
        except OSError:
            pass


def main():
    cfg = load_json(os.path.join(STAGE_DIR, "grader_config.json"))
    rewards = dict((k, 0.0) for k in GATES + CRITERIA)
    metrics = {"grader_exception": 0.0, "root_payload_ok": 0.0, "agent_deprivileged": 0.0, "privileged_mode": 0.0,
               "oracle_token_mismatch": 0.0, "suspicious_lines": 0.0, "remote_sockets_seen": 0.0}
    for k in ("agent_account_cleared", "stage_locked", "agent_processes_killed", "agent_processes_left",
              "submission_late_writes", "late_writes_quarantined", "submission_snapshot"):
        metrics[k] = float(EARLY.get(k, 0.0) or 0.0)
    try:
        try:
            _main_locked(cfg, rewards, metrics)
        except SystemExit:
            raise
        except Exception:  # noqa: BLE001
            log("[grader] EXCEPTION\n" + traceback.format_exc())
            metrics["grader_exception"] = 1.0
            raise
    finally:
        _restore_modes()
        if FREEZER[0] is not None:
            FREEZER[0].stop()


def _main_locked(cfg, rewards, metrics):
    t_start = time.time()
    if os.getuid() == 0 and not EARLY.get("agent_account_cleared"):
        # the confinement of the program leans on the agent's account staying cleared (security-model.md, section 4)
        if not (FREEZER[0] is not None and FREEZER[0].uid is not None):
            fail("the agent's account was neither cleared nor frozen")
    expected = load_json(os.path.join(STAGE_DIR, "given_hashes.json"))
    ok_root, problems = root_payload_ok(expected)
    metrics["root_payload_ok"] = 1.0 if ok_root else 0.0
    if not ok_root:
        fail("the root-owned copy of the givens at %s is unusable: %s" % (ROOT_PAYLOAD, "; ".join(problems[:5])))
    fp = check_numerics()
    metrics["numerics_ok"] = 1.0
    log("[grader] numerics fingerprint %s" % fp[:12])
    rn = Runner(cfg)
    if FREEZER[0] is not None and rn.acct and rn.uid == FREEZER[0].uid:
        FREEZER[0].stop()
    metrics["agent_deprivileged"] = 1.0 if rn.acct else 0.0
    privileged, mismatch = token_state()
    metrics["privileged_mode"] = 1.0 if privileged else 0.0
    metrics["oracle_token_mismatch"] = 1.0 if mismatch else 0.0
    nn_ok, nn_why = nonet.selftest(wrap=lambda a: runner.wrap_limits(a, cpu_s=60, user=rn.name),
                                   env=runner.child_env("/tmp", "/tmp", "/nonexistent", ROOT_GIVEN))
    metrics["nonet_selftest_ok"] = 1.0 if nn_ok else 0.0
    if not nn_ok:
        fail("the no-network filter for the program does not work here: " + nn_why)
    ok, why, suspicious, net = contract_check(cfg, rn)
    if ok and EARLY.get("reasons"):
        ok, why = False, "; ".join(EARLY["reasons"])
    metrics["suspicious_lines"] = float(suspicious)
    log("[grader] contract_ok=%s (%s); privileged %s; runner %s" % (ok, why, privileged, rn.name or "self"))
    towers = hidden_towers()
    metrics["towers_graded"] = float(len(towers))
    if not ok:
        rewards["contract_ok"] = 0.0
        _finish_stats(rn, metrics, t_start)
        write_verdict(rewards, metrics, contract_why=why)
        return
    metrics["episode_retries"] = 0.0
    agent = run_towers(cfg, towers, privileged, metrics)
    runners_unclean = sum(int(_num(s.get("runner_unclean_runs"))) for s in agent)
    step_calls = sum(int(_num(s.get("step_calls"))) for s in agent)
    for s in agent:
        metrics["remote_sockets_seen"] = max(metrics["remote_sockets_seen"], float(_num(s.get("remote_sockets_seen"))))
        log("[grader]   %s: score %.4f relocated %s placed %s collapsed %s removals %s (illegal %s) top %s frames %s "
            "sim %.0f s wall %.0f s program %.0f/%.0f s end %s" % (
                s.get("tag"), s["tower_score"], s.get("relocated"), s.get("placed_total"), s.get("collapsed"),
                s.get("removals"), s.get("illegal_removals"), s.get("top_level"), s.get("frames_used"),
                _num(s.get("t_end")), _num(s.get("episode_wall_s")), _num(s.get("program_time_s")),
                _num(s.get("program_cap_s")), s.get("end_reason")))
    vals = [float(s["tower_score"]) for s in agent]
    gate = (1.0 if step_calls > 0 else 0.0) * (1.0 if runners_unclean == 0 else 0.0) * \
        (1.0 if metrics["remote_sockets_seen"] <= 0 else 0.0)
    rewards["contract_ok"] = gate
    mean = sum(vals) / len(vals) if vals else 0.0
    rewards["block_score"] = float(max(0.0, min(1.0, gate * mean)))
    for i, s in enumerate(agent):
        metrics["tower_score.ep%02d" % (i + 1)] = float(s["tower_score"])
        metrics["relocated.ep%02d" % (i + 1)] = _num(s.get("relocated"))
    for key in ("relocated", "placed_total", "removals", "illegal_removals", "top_level", "frames_used", "t_end",
                "program_time_s", "episode_wall_s", "work_wall_s"):
        metrics[key + "_mean"] = sum(_num(s.get(key)) for s in agent) / max(1, len(agent))
    metrics["collapse_rate"] = sum(1.0 for s in agent if s.get("collapsed")) / max(1, len(agent))
    for r in sorted(set(str(s.get("end_reason")) for s in agent)):
        metrics["end_rate." + r] = sum(1.0 for s in agent if str(s.get("end_reason")) == r) / max(1, len(agent))
    metrics["step_calls"] = float(step_calls)
    metrics["runner_unclean_runs"] = float(runners_unclean)
    try:
        record_dir(OUT)
        with open(os.path.join(OUT, "towers.json"), "w") as f:
            json.dump(agent, f, indent=1, default=str)
    except Exception:  # noqa: BLE001
        log("[grader] towers.json not written\n" + traceback.format_exc())
    _finish_stats(rn, metrics, t_start)
    overall = write_verdict(rewards, metrics)
    log("[grader] rewards %s overall %.4f (verify wall %.0f s)" % ({k: round(v, 4) for k, v in rewards.items()}, overall,
                                                                   metrics["verify_wall_s"]))


def run_towers(cfg, towers, privileged, metrics):
    """Every hidden tower in a process of its own (os.fork), at the same time, each under its own runner account
    (grader_config.json runner_accounts; without enough accounts, as when not root, they run one after another in
    this process). Returns the summaries in the hidden set's order; a child that dies without a summary is a task
    error."""
    accounts = list(cfg.get("runner_accounts") or [])
    acct_ok = os.getuid() == 0 and len(accounts) >= len(towers) and all(_account_exists(a) for a in accounts[:len(towers)])
    parallel = acct_ok or (os.getuid() != 0 and os.environ.get("BT_PARALLEL", "1") == "1")
    record_dir(OUT)
    if not parallel:
        out = []
        rn = Runner(cfg)
        for i, tower in enumerate(towers):
            s = _episode_with_retry(cfg, tower, rn, privileged, "ep%02d" % (i + 1), metrics)
            s["runner_unclean_runs"] = rn.unclean_runs
            out.append(s)
        return out
    log("[grader] %d towers in parallel (%s)" % (len(towers), ", ".join(accounts[:len(towers)]) if acct_ok else "self"))
    kids = {}
    for i, tower in enumerate(towers):
        path = os.path.join(OUT, "tower%02d.json" % (i + 1))
        if os.path.exists(path):
            os.remove(path)
        pid = os.fork()
        if pid == 0:
            code = 1
            try:
                rn = Runner(cfg, accounts[i] if acct_ok else None)
                m = {"episode_retries": 0.0}
                s = _episode_with_retry(cfg, tower, rn, privileged, "ep%02d" % (i + 1), m)
                s["runner_unclean_runs"] = rn.unclean_runs
                s["episode_retries"] = m["episode_retries"]
                with open(path + ".tmp", "w") as f:
                    json.dump(s, f, default=str)
                os.replace(path + ".tmp", path)
                code = 0
            except SystemExit:
                code = 1
            except BaseException:  # noqa: BLE001
                log("[grader] tower %d: EXCEPTION\n%s" % (i + 1, traceback.format_exc()))
                code = 1
            finally:
                os._exit(code)
        kids[pid] = (i, path)
    out = [None] * len(towers)
    bad = []
    for pid, (i, path) in kids.items():
        _, status = os.waitpid(pid, 0)
        if os.WIFEXITED(status) and os.WEXITSTATUS(status) == 0 and os.path.isfile(path):
            out[i] = load_json(path)
            metrics["episode_retries"] = metrics.get("episode_retries", 0.0) + _num(out[i].get("episode_retries"))
        else:
            bad.append(i + 1)
    if bad:
        fail("tower(s) %s could not be run to a score (see the log above)" % bad)
    if metrics.get("episode_retries", 0.0) > MAX_RERUNS_PER_VERIFY:
        fail("the cell failed in more than %d episode of this verify" % MAX_RERUNS_PER_VERIFY)
    return out


def _account_exists(name):
    try:
        pwd.getpwnam(name)
        return True
    except KeyError:
        return False


def _finish_stats(rn, metrics, t_start):
    metrics["runner_processes_killed"] = float(rn.killed)
    metrics["runner_processes_left"] = float(rn.left)
    metrics["runner_unclean_runs"] = float(rn.unclean_runs)
    metrics["stray_files_removed"] = float(rn.stray_files)
    metrics["agent_processes_frozen"] = float(len(FREEZER[0].frozen)) if FREEZER[0] is not None else 0.0
    metrics["verify_wall_s"] = time.time() - t_start
    shutil.rmtree(RUNS_ROOT, ignore_errors=True)


if __name__ == "__main__":
    main()
