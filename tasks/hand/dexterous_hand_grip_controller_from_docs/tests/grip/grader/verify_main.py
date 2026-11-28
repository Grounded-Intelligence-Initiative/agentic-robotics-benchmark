"""The verify stage of the grip-controller acceptance: runs as ROOT in a separate verifier sandbox; tests/verify.py
puts tests/grip first on sys.path and calls main().

The agent left /home/user/submission/controller/controller.py (class Controller: setup / step; tests/grip/CONTRACT.md).
main():
  0. FIRST, stdlib only (`_early_lockdown`): freezes and kills every process of the agent account until the verdict is
     written, locks the stage, quarantines late entries outside the submission (tests/test.sh closed the stage
     folder already); then (`_snapshot_controller`, before anything of the submission is read) a walk that only
     lists and stats checks the delivered folder submission/controller/ against the caps (bytes, entries, depth;
     a real folder, not a link), and only within them the late-write gate runs on it and a root-owned snapshot of
     it is taken (every later read uses the snapshot);
  1. checks the root-owned copy of the givens at /opt/arb against given_hashes.json (a mismatch is a task error);
  2. the static pass conditions: the controller folder within its caps; controller/controller.py a regular file that
     parses and defines class Controller; the import audit passes (no networking / process / ctypes import, no
     computed import or code; every .py readable, within the per-file cap, in UTF-8 / ASCII / Latin-1, parsable).
     Every parse runs in a child process under its own limits (netaudit.audit_in_child): the grader's own process
     reads no byte of the sources, and a child that does not finish is the submission's failed check;
  3. the token: controller/oracle_token (a regular file of at most TOKEN_MAX_BYTES; a larger one is a mismatch and
     is never opened) whose sha256 equals hidden/oracle_token.sha256 makes the run privileged (the reference
     script's commands reach the controller); a wrong token is ignored and counted;
  4. the fixed hidden set (tests/hidden/scenarios: two hand units, each with two shifts; every file checked against
     its manifest and the code digest by world.scenarios; a missing set is a task error), the two units ONE AFTER
     THE OTHER (unit 1, then unit 2), each in its own unit worker (grader.acceptance) started with its unit, each
     with the controller as `nobody` under the client's no-network filter, from a read-only copy of the
     submission; between the units the runner account is cleared (grader.acceptance.UnitIsolation: its processes
     ended, the unit's run folder and everything it owns in the writable places removed); while they run, the
     root sampler of the runner account's processes (netaudit.SocketSampler) counts their remote sockets and keeps
     them at oom_score_adj 1000 (one found below is killed); each controller's log is capped (wire.drain_capped);
  5. the run's pass conditions: setup answered on every unit, >= 90 % of each unit's cycles on time, no late write, no
     process left behind, no network (`network_gate`: the refusal in force for every unit and no remote socket of the
     runner account; the namespace counters are a statistic that judges nothing); reward = pass conditions x raw / anchor (the grader kit's ratio scoring), raw =
     the mean of S over the four shifts (grader.metrics). Only the aggregate metrics of names.METRIC_KEYS are
     published; per-job records go to the root-only record folder.
Untouched sandbox: no controller.py -> reward 0. Reference: the token -> the echo of the reference script -> 1.
"""
import hashlib
import json
import math
import os
import pwd
import shutil
import signal
import stat as statmod
import sys
import time
import traceback

sys.dont_write_bytecode = True

from grader.runtime import lockdown  # noqa: E402  (stdlib only)
from grader.runtime import runner  # noqa: E402  (stdlib only)
from grader.runtime import verify_freeze  # noqa: E402  (stdlib only)

LATE_GATE_MAX_GAP_S = 600.0
TOKEN_MAX_BYTES = 4096            # a real token is 64 hex digits and a line end
FREEZER = [None]
# a development module for the unit workers (session_class, cell_parts, make_script); None in the verify stage
DEV_HOOKS = None


class Paths(object):
    def __init__(self):
        self.stage = os.environ.get("ALE_STAGE_DIR", os.path.dirname(os.path.abspath(sys.argv[0])))
        self.home = os.environ.get("ALE_HOME", "/home/user")
        self.agent_user = os.path.basename(self.home.rstrip("/")) or "user"
        self.submission = os.path.join(self.home, "submission")
        self.root_payload = os.environ.get("ARB_ROOT_PAYLOAD", "/opt/arb")
        self.runs = os.path.join(self.root_payload, "runs")
        self.snapshot = os.path.join(self.runs, "_submission_snapshot")
        self.quarantine = os.path.join(self.runs, "_quarantine")
        self.sticky = [d for d in ("/tmp", "/dev/shm", "/var/tmp", "/run/lock") if os.path.isdir(d)]
        self.record = os.environ.get("ARB_RECORD_ROOT", "/opt/arb-verify")
        self.logs = os.path.join(self.record, "logs")
        self.out = os.path.join(self.record, "out")
        self.hidden = os.path.join(self.stage, "hidden")
        self.scenarios = os.path.join(self.hidden, "scenarios")
        self.client = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runtime", "client.py")


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def _proc_state(pid):
    """The /proc state letter of `pid` ("X" once it is gone)."""
    try:
        with open("/proc/%d/status" % pid) as fh:
            for line in fh:
                if line.startswith("State:"):
                    return line.split()[1]
    except (OSError, IndexError):
        pass
    return "X"


def _still_running(uid, bound_s=3.0):
    """SIGSTOP every live process of `uid` until a pass finds none still running (a respawn that appears meanwhile is
    stopped by the next pass) or `bound_s` elapsed; returns the PIDs still running (empty: every one is stopped)."""
    deadline = time.time() + float(bound_s)
    while True:
        for pid in lockdown.live_uid_processes(uid):
            try:
                os.kill(pid, signal.SIGSTOP)
            except OSError:
                pass
        time.sleep(0.05)
        running = [pid for pid in lockdown.live_uid_processes(uid) if _proc_state(pid) not in ("T", "t", "X")]
        if not running or time.time() > deadline:
            return running


def _early_lockdown(P):
    out = {"agent_processes_killed": 0.0, "agent_processes_left": 0.0, "agent_processes_running": 0.0,
           "stage_locked": 0.0, "stage_parent_locked": 0.0,
           "stage_lock_latency_ms": -1.0, "submission_late_writes": 0.0, "late_writes_quarantined": 0.0,
           "late_write_gate_active": 0.0, "submission_snapshot": 0.0, "saved": None, "parent": None, "t_ref": None,
           "late": [], "reasons": []}
    if os.getuid() != 0:
        return out
    t_ref = lockdown.stage_upload_time(P.stage)
    out["t_ref"] = t_ref
    try:
        uid = pwd.getpwnam(P.agent_user).pw_uid
    except KeyError:
        uid = None
    if uid is not None and uid != 0:
        killed, left = lockdown.stop_and_kill_uid(uid, grace_s=8.0)
        out["agent_processes_killed"], out["agent_processes_left"] = float(killed), float(left)
    # supervisord (the image's PID 1) starts the agent account's desktop again within seconds of the kill, in the
    # separate verifier too: every process of the account stays SIGSTOPped (polled every 0.1 s; a stopped process does
    # not exit, so it is not started again) until the verdict is written
    FREEZER[0] = verify_freeze.AgentFreezer.for_user(P.agent_user).start()
    if uid is not None and uid != 0 and out["agent_processes_left"]:
        # a process that outlived the kill is the image's own desktop: it is frozen with the rest of the account (SIGSTOP
        # passes for at most 3 s); only one still running after that voids the run (a task error in _main_locked)
        running = _still_running(uid)
        out["agent_processes_running"] = float(len(running))
        if running:
            out["reasons"].append("%d process(es) of the agent account survived the kill at verify start and could not "
                                  "be stopped" % len(running))
        else:
            log("[grader] %d process(es) of the agent account (the image's desktop, started again) outlived the kill at "
                "verify start: frozen until the verdict" % int(out["agent_processes_left"]))
    out["saved"] = runner.lock_stage(P.stage)
    out["stage_locked"] = 1.0 if out["saved"] is not None else 0.0
    parent = os.path.dirname(os.path.abspath(P.stage))
    if parent and parent not in ("/", os.path.abspath(P.home)) and os.path.isdir(parent):
        try:
            mode = statmod.S_IMODE(os.lstat(parent).st_mode)
            os.chmod(parent, 0o700)
            saved_mode = os.environ.get("ARB_STAGE_PARENT_MODE")
            out["parent"] = (parent, int(saved_mode, 8) if saved_mode else mode)
            out["stage_parent_locked"] = 1.0
        except (OSError, ValueError):
            pass
    gate_active = t_ref is not None
    cfg_json = os.path.join(P.stage, "config.json")
    if gate_active and os.path.isfile(cfg_json):
        try:
            gate_active = (os.lstat(cfg_json).st_ctime - t_ref) <= LATE_GATE_MAX_GAP_S
        except OSError:
            pass
    out["late_write_gate_active"] = 1.0 if gate_active else 0.0
    if gate_active:
        q = 0
        for root in P.sticky + [P.home]:
            late = lockdown.late_entries(root, t_ref, exclude=(P.submission, P.runs, P.stage))
            if late:
                log("[grader] %d late entr%s under %s quarantined" % (len(late), "y" if len(late) == 1 else "ies", root))
                q += lockdown.quarantine(root, late, P.quarantine)
        out["late_writes_quarantined"] = float(q)
    out["stage_lock_latency_ms"] = lockdown.process_age_ms()
    return out


def _snapshot_controller(P, early):
    """The delivered folder submission/controller/, before anything of it is read: a walk that only lists and stats
    checks it against the caps (netaudit.tree_shape: bytes, entries, depth; the folder and the submission real folders,
    not links). Within the caps, and as root only: the late-write gate on it (its entries written after the stage upload
    began) and its root-owned snapshot P.snapshot/controller (regular files only), which every later read uses. Outside
    the caps nothing is walked further, copied or read. Returns the walk's record (also early["shape"])."""
    from contract import constants as C
    from grader.runtime import netaudit
    I = C.INTERFACE
    ctl = os.path.join(P.submission, "controller")
    if os.path.islink(P.submission):
        shape = {"ok": False, "why": "is inside a submission folder that is a symbolic link", "entries": 0, "bytes": 0,
                 "depth": 0}
    else:
        shape = netaudit.tree_shape(ctl, I["entries_max"], I["depth_max"], I["size_cap_mb"] << 20)
    early["shape"] = shape
    if not shape["ok"] or os.getuid() != 0:
        return shape
    if early.get("late_write_gate_active") and early.get("t_ref") is not None:
        early["late"] = lockdown.late_entries(ctl, early["t_ref"])
        early["submission_late_writes"] = float(len(early["late"]))
        if early["late"]:
            early["reasons"].append("%d entr%s of the delivered controller written after the agent phase ended (%s)" % (
                len(early["late"]), "y" if len(early["late"]) == 1 else "ies",
                ", ".join(e[0] for e in early["late"][:5])))
    os.makedirs(P.runs, exist_ok=True)
    os.chmod(P.runs, 0o755)
    os.makedirs(P.snapshot, exist_ok=True)
    os.chmod(P.snapshot, 0o700)
    lockdown.snapshot_dir(ctl, os.path.join(P.snapshot, "controller"))
    early["submission_snapshot"] = 1.0
    return shape


def sha256_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def load_json(p):
    with open(p) as f:
        return json.load(f)


def fail(reason):
    log("[grader] TASK ERROR: " + reason)
    raise SystemExit(1)


def program_source(P):
    src = P.snapshot if os.getuid() == 0 and os.path.isdir(P.snapshot) else P.submission
    return os.path.join(src, "controller")


def root_payload_ok(P, expected):
    problems = []
    owner = 0 if os.getuid() == 0 else os.getuid()
    if not os.path.isdir(P.root_payload):
        return False, ["%s missing (tests/test.sh builds it)" % P.root_payload]
    for rel, digest in expected.items():
        p = os.path.join(P.root_payload, rel)
        try:
            st = os.lstat(p)
        except OSError:
            problems.append("missing " + rel)
            continue
        if os.path.islink(p) or st.st_uid != owner or (st.st_mode & 0o022):
            problems.append("not a root-owned, group/world-unwritable regular file: " + rel)
            continue
        if sha256_file(p) != digest:
            problems.append("differs " + rel)
    return not problems, problems


def static_checks(P, shape):
    """{"controller_file", "size", "import_audit"}, the reasons of the failed ones and the audit's hits. `shape` is
    _snapshot_controller's walk: outside its caps nothing of the folder is read. Every parse runs in the audit's child
    process (netaudit.audit_in_child); this process only stats."""
    from contract import constants as C
    from grader.runtime import netaudit
    I = C.INTERFACE
    out = {"controller_file": False, "size": False, "import_audit": False}
    if not shape.get("ok"):
        if shape.get("why") == "missing":
            return out, ["no controller/ folder in the submission"], []
        return out, ["the controller folder %s" % shape.get("why")], []
    out["size"] = True
    src = program_source(P)
    why = []
    try:
        is_reg = statmod.S_ISREG(os.lstat(os.path.join(src, "controller.py")).st_mode)
    except OSError:
        is_reg = False
    if not is_reg:
        why.append("no controller/controller.py (regular file) in the submission")
    res, failed = netaudit.audit_in_child(src, I["source_file_max_mb"] << 20, I["audit_cpu_s"], I["audit_wall_s"])
    if res is None:
        why.append(failed)
        return out, why, [("controller", failed)]
    out["controller_file"] = bool(is_reg and res.get("controller_file"))
    if is_reg and not out["controller_file"]:
        why.append(str(res.get("controller_error") or "controller.py defines no class Controller")[:300])
    net = [tuple(h) for h in res.get("hits") or []]
    n = int(res.get("n_hits") or 0)
    out["import_audit"] = n == 0 and not net
    if net:
        why.append("import audit: %s" % ", ".join("%s: %s" % x for x in net[:5]))
    return out, why, net


def token_state(P):
    """(privileged, mismatch). The token is lstat-ed first: only a regular file of at most TOKEN_MAX_BYTES is opened
    (without following a link, without blocking) and read; a larger one is a mismatch, never opened."""
    p = os.path.join(program_source(P), "oracle_token")
    want_p = os.path.join(P.hidden, "oracle_token.sha256")
    try:
        st = os.lstat(p)
    except OSError:
        return False, False
    if not statmod.S_ISREG(st.st_mode) or not os.path.isfile(want_p):
        return False, False
    if st.st_size > TOKEN_MAX_BYTES:
        return False, True
    try:
        fd = os.open(p, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        try:
            tok = os.read(fd, TOKEN_MAX_BYTES + 1).strip()
        finally:
            os.close(fd)
        want = open(want_p).read().strip()
    except OSError:
        return False, False
    got = hashlib.sha256(tok).hexdigest()
    return (got == want), (got != want)


def hidden_scenarios(P):
    """The fixed hidden set, in unit order. world.scenarios.load checks each file against the manifest, the running
    code's digest and the ceilings (ScenarioStale); a missing or short set is a task error."""
    from contract import constants as C
    from world import scenarios as SC
    man = os.path.join(P.scenarios, "manifest.json")
    manifest = load_json(man)
    kept = manifest.get("kept") or []
    if len(kept) != C.TIMELINE["units"]:
        raise RuntimeError("the hidden set holds %d units, not %d" % (len(kept), C.TIMELINE["units"]))
    return [SC.load(os.path.join(P.scenarios, k["file"]), manifest=man) for k in kept]


def _descendants(pids):
    """Every live process below the given ones (from /proc's parent links)."""
    parent = {}
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open("/proc/%s/stat" % name) as fh:
                st = fh.read()
            parent[int(name)] = int(st[st.rindex(")") + 2:].split()[1])
        except (OSError, ValueError, IndexError):
            continue
    out, frontier = set(), set(int(p) for p in pids)
    while frontier:
        nxt = {c for c, p in parent.items() if p in frontier and c not in out}
        out |= nxt
        frontier = nxt
    return sorted(out)


def write_verdict(P, raw, gates_ok, metrics, text):
    """One gated `reward` (robotics_grader.ratio_rewards against anchor.json: value 1.0 = the reference, full_at 1.0)."""
    if P.stage not in sys.path:
        sys.path.append(P.stage)
    from robotics_grader import ratio_rewards, read_anchor
    from robotics_grader import write_verdict as kit_write_verdict
    anchor, meta = read_anchor()
    verdict = ratio_rewards(float(raw), anchor, higher_is_better=meta.get("direction", "higher") == "higher",
                            metric_name=meta.get("metric", "raw"), locks_ok=bool(gates_ok), extra=metrics,
                            full_at=meta.get("full_at", 1.0))
    if os.environ.get("ALE_VERIFY_PYTHON"):
        recorded = {}
        for k, v in sorted(verdict["metrics"].items()):
            try:
                v = float(v)
            except (TypeError, ValueError):
                continue
            if math.isfinite(v):
                recorded[k.replace(".", "_").replace("/", "_")] = v
        kit_write_verdict({"rewards": verdict["rewards"], "metrics": recorded}, diagnostic=text)
        return verdict
    kit_write_verdict(verdict, path=os.environ.get("ROBOTICS_GRADER_VERDICT_PATH") or
                      os.path.join(P.stage, "verdict.local.json"))
    return verdict


def _restore_modes(P, early):
    runner.unlock_stage(early.get("saved"))
    mode = os.environ.get("ARB_STAGE_MODE")
    if mode:
        try:
            os.chmod(P.stage, int(mode, 8))
        except (OSError, ValueError):
            pass
    parent = early.get("parent")
    if parent:
        try:
            os.chmod(parent[0], parent[1])
        except OSError:
            pass


def main():
    P = Paths()
    early = _early_lockdown(P)
    try:
        _main_locked(P, early)
    except SystemExit:
        raise
    except Exception:  # noqa: BLE001
        log("[grader] grader exception\n" + traceback.format_exc())
        raise
    finally:
        _restore_modes(P, early)
        if FREEZER[0] is not None:
            FREEZER[0].stop()


def _early_ops(early):
    """The verify-start counts that are published (records, never a pass condition): the agent account's processes the
    kill ended, those that outlived it and those still running after the freeze (a task error when not 0)."""
    return {k: float(early.get(k, 0.0)) for k in ("agent_processes_killed", "agent_processes_left",
                                                  "agent_processes_running")}


def _record(P, name, text):
    os.makedirs(P.record, exist_ok=True)
    os.chmod(P.record, 0o700)
    os.makedirs(P.out, exist_ok=True)
    with open(os.path.join(P.out, name), "w") as fh:
        fh.write(text)


def network_gate(no_net, remote_sockets_seen):
    """The network pass condition ("the controller makes no network connection: the host computer refuses every socket
    but a local one", acceptance specification sections 1 and 9): the client's refusal was reported installed for every
    unit (as root a unit without it is a task error before setup: wire.Runner.require_filter) and no remote socket of
    the runner account was seen. The namespace-wide counters (`net_outbound_events`) are a published statistic that
    judges nothing: they count every process of the sandbox (the image's own services, loopback), not the controller."""
    return bool(no_net) and float(remote_sockets_seen) <= 0


def _main_locked(P, early):
    from contract import names as N
    from grader import acceptance as A
    from grader import metrics as M
    from grader.runtime import netaudit
    from grader.runtime import wire as W
    t_start = time.time()
    if early.get("agent_processes_running", 0.0) > 0:
        fail("%d process(es) of the agent account survived the verify-start kill and could not be stopped: the run is "
             "void" % int(early["agent_processes_running"]))
    cfg = load_json(os.path.join(P.stage, "grader_config.json"))
    ops = {k: 0.0 for k in ("privileged_mode", "oracle_token_mismatch", "network_imports", "hidden_set",
                            "root_payload_ok", "controller_no_network", "remote_sockets_seen", "net_outbound_events",
                            "stray_processes", "runner_processes_killed", "runner_files_removed", "units_in_turn",
                            "plant_faults", "runner_oom_adj_killed", "oom_floor_active", "stderr_bytes_dropped",
                            "agent_processes_frozen", "verify_wall_s")}
    ops.update(_early_ops(early))
    shape = _snapshot_controller(P, early)
    expected = load_json(os.path.join(P.stage, "given_hashes.json"))
    ok_root, problems = root_payload_ok(P, expected)
    ops["root_payload_ok"] = 1.0 if ok_root else 0.0
    if not ok_root:
        fail("the root-owned copy of the givens at %s is unusable: %s" % (P.root_payload, "; ".join(problems[:5])))
    privileged, mismatch = token_state(P) if shape["ok"] else (False, False)
    ops["privileged_mode"] = 1.0 if privileged else 0.0
    ops["oracle_token_mismatch"] = 1.0 if mismatch else 0.0
    gates, why, net = static_checks(P, shape)
    ops["network_imports"] = float(len(net))
    gates.update({"setup": False, "on_time": False, "no_late_write": early["submission_late_writes"] <= 0,
                  "clean": True, "network": False})
    static_ok = all(gates[k] for k in ("controller_file", "size", "import_audit"))
    log("[grader] static checks %s (%s); privileged %s" % (static_ok, "; ".join(why + early["reasons"]) or "ok",
                                                          privileged))
    units = []
    unit_stats = {}
    records_out = []
    if static_ok:
        scen = hidden_scenarios(P)
        ops["hidden_set"] = float(len(scen))
        jobs = {}
        for i, sc in enumerate(scen):
            u = i + 1
            jobs[u] = {"scenario": sc, "privileged": bool(privileged), "program_dir": program_source(P),
                       "controller_root": os.path.join(P.runs, "unit%d_controller" % u), "client": P.client,
                       "account": cfg.get("runner_account", "nobody"),
                       "stderr_log": os.path.join(P.logs, "unit%d_controller.stderr.log" % u)}
            if DEV_HOOKS:
                jobs[u]["dev"] = DEV_HOOKS
        os.makedirs(P.runs, exist_ok=True)
        os.chmod(P.runs, 0o755)
        os.makedirs(P.record, exist_ok=True)
        os.chmod(P.record, 0o700)
        os.makedirs(P.logs, exist_ok=True)
        workers = []
        net_c = netaudit.NetCounters()
        acct = runner.runner_account(cfg.get("runner_account", "nobody")) if os.getuid() == 0 else None
        if acct:
            # as root the sampler also keeps the controller's processes at oom_score_adj 1000 for as long as they run:
            # one found below it (it lowered its own value) is killed, a dead controller; the launch chain is spared
            sampler = netaudit.SocketSampler(uid=acct[1], period_s=float(cfg.get("socket_sample_s", 0.5)),
                                             oom_floor=W.OOM_ADJ)
        else:
            sampler = netaudit.SocketSampler(uid=None, pids_fn=lambda: _descendants([p.pid for p in workers]),
                                             period_s=float(cfg.get("socket_sample_s", 0.5)))
        with net_c, sampler:
            res = A.run_workers(jobs, P.runs, log=log, on_start=lambda u, p: workers.append(p),
                                isolation=A.UnitIsolation(cfg.get("runner_account", "nobody"), A.RUNNER_WRITABLE_DIRS))
        ops["remote_sockets_seen"] = float(sampler.max_seen)
        ops["net_outbound_events"] = float(net_c.total)
        ops["runner_oom_adj_killed"] = float(sampler.oom_killed)        # a record (a kill can be counted twice)
        ops["oom_floor_active"] = 1.0 if sampler.oom_floor is not None else 0.0
        for u in sorted(res):
            recs, st = res[u]
            units.append({"unit": u, "records": recs, "events": st["events"]})
            unit_stats[u] = st
            records_out += recs
            ops["stray_processes"] += float(st["runner"].get("stray_processes", 0))
            ops["runner_processes_killed"] += float(st["runner"].get("runner_processes_killed", 0))
            ops["stderr_bytes_dropped"] += float(st["runner"].get("stderr_dropped_bytes", 0))
            ops["plant_faults"] += 1.0 if st.get("controller_ended") == "plant_fault" else 0.0
            iso = st.get("isolation", {})
            ops["runner_processes_killed"] += float(iso.get("before", {}).get("killed", 0) +
                                                   iso.get("after", {}).get("killed", 0))
            ops["runner_files_removed"] += float(iso.get("after", {}).get("files_removed", 0) +
                                                iso.get("after", {}).get("ipc_removed", 0))
        ops["units_in_turn"] = 1.0 if all(st.get("isolation", {}).get("started_after_previous")
                                          for st in unit_stats.values()) else 0.0
        no_net = all(bool(st["runner"].get("no_network")) for st in unit_stats.values())
        ops["controller_no_network"] = 1.0 if no_net else 0.0
        gates["setup"] = all(st["setup_ok"] for st in unit_stats.values())
        gates["on_time"] = all(st["on_time_fraction"] >= float(cfg.get("on_time_min", 0.9))
                               for st in unit_stats.values())
        gates["clean"] = ops["stray_processes"] <= 0       # the runner account's strays alone
        gates["network"] = network_gate(no_net, ops["remote_sockets_seen"])
    else:
        for u in N.UNITS:
            units.append({"unit": u, "records": [], "events": []})
    sc_out = M.score(units, gates)
    ops["agent_processes_frozen"] = float(len(FREEZER[0].frozen)) if FREEZER[0] is not None else 0.0
    ops["verify_wall_s"] = time.time() - t_start
    metrics = M.public_metrics(sc_out, unit_stats, ops)
    text = ("pass conditions %s; raw %.4f (the mean over the four shifts of S = mean carry quality x (0.7 + 0.3 J)); "
            "%s" % ("met" if all(sc_out["gates"].values()) else "NOT met: " + ", ".join(
                k for k, v in sc_out["gates"].items() if not v), sc_out["raw"], "; ".join(why + early["reasons"])))
    _record(P, "score.json", json.dumps(sc_out, sort_keys=True, indent=1, allow_nan=False) + "\n")
    _record(P, "jobs.jsonl", "".join(json.dumps(r, sort_keys=True, allow_nan=False) + "\n" for r in records_out))
    _record(P, "stats.json", json.dumps({"format": "grip-stats/1", "cpu_model": A.host_cpu_model(),
                                         "units": [unit_stats[u] for u in sorted(unit_stats)],
                                         "early": {k: v for k, v in early.items() if k not in ("saved",)},
                                         "ops": ops}, indent=1, sort_keys=True, default=str))
    shutil.rmtree(P.runs, ignore_errors=True)
    verdict = write_verdict(P, sc_out["raw"], all(sc_out["gates"].values()), metrics, text)
    log("[grader] reward %.4f raw %.4f gates %s (verify wall %.0f s)" % (
        verdict["rewards"]["reward"], sc_out["raw"], sc_out["gates"], ops["verify_wall_s"]))
    return verdict
