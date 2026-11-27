"""The stages of one grade, and the worker process verify.py starts.

    res = grade_submission(sub_dir, ctx, guard, record=None)

Stages, each under its own alarm (Guard: the smaller of the stage's cap and the time left before the deadline):
  selftest      the truth files and the reference replay (selftest.py); its failure is a task error
  gate          the public gate (G0 = 0 when it fails: no scene, an unknown name, a cap exceeded ...); a body the gate
                excludes is deleted before physics and its thing is absent
  settle        2 s of the standard simulation with the contract's held things kept still (G0 = 0 on a blow-up)
  registration  the twin's pieces in its own support-top frame -> reference points -> the fit onto the true support
                frame (register.py); a twin that does not register uses nothing
  requests      every scored request planned on the registered twin (the public instrument; with grader_config
                "envelope" true, a site whose swept envelope touches anything of the twin is no site; false: no
                envelope at all) and, when planned, executed in the truth (the
                scans at the tracker's poses) at the nine placements; a request on a thing the twin lacks is declined
                without a plan
  lifts         (dynamic lifts only) a creditable body pinch flagged for a lift is done at a placement only when the
                thing also rises and holds in the truth under each lift variant (fresh model per trial, up to
                lift_workers processes, merged by key)
  use           the use of the pick table (use_score.py)
twin_score = G1 x G0 x use (G1 is verify.py's). A stage that raises scores what depends on it 0, with a diagnostic
(gate, settle: G0 = 0; registration, requests: use = 0); a stage listed in runtime.timeout_is_task_error that runs
out of time is a task error (the verifier was too slow: no outcome may come from the wall clock there), and so is a
lift stage that fails as a whole (a single trial that raises does not lift) or a use that cannot be computed.
"""
import json
import math
import os
import signal
import sys
import time

import numpy as np

from . import FORMAT_PROGRESS
from . import load
from . import register
from . import selftest as ST
from . import truth as T
from . import use_score

OUTCOMES = ("declined", "stop", "miss", "done")


class StageTimeout(Exception):
    pass


class TaskError(Exception):
    """The grader cannot give an honest score (the truth, the configuration, or the verifier's time)."""


class Guard:
    """Run stages under a wall-clock alarm: min(cap of the stage, time left before the deadline)."""

    def __init__(self, deadline=None, caps=None, enabled=True):
        self.deadline = deadline
        self.caps = dict(caps or {})
        self.enabled = enabled

    def left(self):
        return float("inf") if self.deadline is None else self.deadline - time.time()

    def run(self, name, fn, *args):
        if not self.enabled:
            return fn(*args)
        cap = min(float(self.caps.get(name, float("inf"))), self.left())
        if cap <= 0.5:
            raise StageTimeout("no time left for stage %s" % name)
        if not math.isfinite(cap):
            return fn(*args)

        def _alarm(_sig, _frm):
            raise StageTimeout("stage %s ran out of time" % name)

        old = signal.signal(signal.SIGALRM, _alarm)
        signal.setitimer(signal.ITIMER_REAL, cap)
        try:
            return fn(*args)
        finally:
            signal.setitimer(signal.ITIMER_REAL, 0.0)
            signal.signal(signal.SIGALRM, old)


def _err(exc):
    if isinstance(exc, StageTimeout):
        return "timeout: %s" % exc
    if isinstance(exc, MemoryError):
        return "out of memory (the grader's address-space limit) on this twin"
    return "%s: %s" % (type(exc).__name__, str(exc)[:300])


def _clean(x):
    """JSON-safe copy: numpy scalars and arrays to Python, non-finite floats to None, tuples to lists."""
    if isinstance(x, dict):
        return {str(k): _clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_clean(v) for v in x]
    if isinstance(x, np.ndarray):
        return _clean(x.tolist())
    if isinstance(x, (bool, np.bool_)):
        return bool(x)
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (float, np.floating)):
        f = float(x)
        return f if math.isfinite(f) else None
    return x


class Context:
    """What every grade needs: the configurations, the payload modules, the truth (after the self-test)."""

    def __init__(self, gt_dir, gcfg):
        self.gt_dir = gt_dir
        self.gcfg = gcfg
        self.P = load.payload()
        self.HI = self.P["hand_instrument"]
        self.icfg = load.instrument_config()
        self.env = load.envelope(gcfg)          # None: no envelope (grader_config "envelope" false)
        self.keep = {}
        self.selftest = None

    @property
    def gt(self):
        return self.keep["gt"]

    def thing_order(self):
        return list(self.gcfg.get("things") or list(self.gt["things"]["things"]))

    def held(self):
        return T.held_keys(self.gt["things"])


def run_selftest(ctx):
    ctx.selftest = ST.run(ctx.gt_dir, ctx.gcfg, ctx.icfg, ctx.env, ctx.HI, keep=ctx.keep)
    return ctx.selftest


def _dir_or_side(r):
    return r["side"] if r["rule"] == "hook" else r["dir_deg"]


def evaluate_requests(ctx, twin_scene, present):
    """{id: {"planned", "declined_by", "blocker", "plan", "outcomes": [9], "by": [9], "hit": [9], "error"}} for every
    scored request, in id order."""
    HI, icfg, env = ctx.HI, ctx.icfg, ctx.env
    truth_scene = ctx.keep["truth_scene"]
    placements = icfg["placements_mm"]
    out = {}
    for r in sorted(ctx.gt["requests"]["requests"], key=lambda r: r["id"]):
        row = {"planned": False, "declined_by": None, "blocker": None, "plan": None,
               "outcomes": ["declined"] * len(placements), "by": [None] * len(placements),
               "hit": [None] * len(placements), "error": None}
        if r["thing"] not in present:
            row["declined_by"] = "absent"
            out[r["id"]] = row
            continue
        try:
            ex = HI.plan_explain(twin_scene, r["thing"], r["rule"], _dir_or_side(r), icfg, env)
            plan = ex.get("plan")
            row["declined_by"], row["blocker"] = ex.get("declined_by"), ex.get("blocker")
            if plan is not None:
                outs = [HI.execute(plan, truth_scene, off, icfg, env) for off in placements]
                for o in outs:
                    if o.get("outcome") not in OUTCOMES:
                        raise ValueError("the instrument returned the outcome %r" % (o.get("outcome"),))
                row.update({"planned": True, "plan": plan, "outcomes": [o["outcome"] for o in outs],
                            "by": [o.get("by") for o in outs], "hit": [o.get("hit") for o in outs]})
        except (KeyboardInterrupt, SystemExit, StageTimeout):
            raise
        except Exception as exc:  # noqa: BLE001 - a request the instrument cannot handle on this twin is declined
            row.update({"planned": False, "plan": None, "outcomes": ["declined"] * len(placements),
                        "by": [None] * len(placements), "hit": [None] * len(placements), "error": _err(exc)})
        out[r["id"]] = row
    return out


# ---------------------------------------------------------------------------------------------------------------
# lifts

def lift_groups(ctx, rows):
    """[(id, placement index)] in sorted order: the placements at which a lift-flagged request is geometrically
    done; each group runs the lift variants in configuration order and stops at the first that does not lift."""
    flagged = {r["id"] for r in ctx.gt["requests"]["requests"] if r.get("lift") and r["class"] == "creditable"}
    cap = ctx.gcfg.get("lift_max_directions_per_unit")
    if cap is not None:
        per_unit = {}
        keep = set()
        for r in sorted(ctx.gt["requests"]["requests"], key=lambda r: r["id"]):
            if r["id"] in flagged:
                u = use_score.unit_key(r["thing"], r["rule"])
                per_unit[u] = per_unit.get(u, 0) + 1
                if per_unit[u] <= int(cap):
                    keep.add(r["id"])
        flagged = keep
    groups = []
    for rid in sorted(flagged):
        row = rows.get(rid)
        if row is None or not row["planned"]:
            continue
        for i, o in enumerate(row["outcomes"]):
            if o == "done":
                groups.append((rid, i))
    return groups


def _run_groups(ctx, groups, rows, models):
    HI, icfg = ctx.HI, ctx.icfg
    held = ctx.held()
    placements = icfg["placements_mm"]
    out = {}
    for rid, i in groups:
        trials = []
        lifted = True
        for vi, v in enumerate(ctx.gcfg["lift_variants"]):
            xml, bodies = models[vi]
            try:
                res = HI.lift(rows[rid]["plan"], xml, bodies, [float(v[0]), float(v[1])], placements[i], icfg,
                              held=held)
                ok = bool(res.get("lifted"))
                trials.append({"variant": [float(v[0]), float(v[1])], "lifted": ok,
                               "rise_mm": _clean(res.get("rise_mm")), "reason": str(res.get("reason", ""))[:200]})
            except (KeyboardInterrupt, SystemExit, StageTimeout):
                raise
            except Exception as exc:  # noqa: BLE001 - a trial that fails to run does not lift
                ok = False
                trials.append({"variant": [float(v[0]), float(v[1])], "lifted": False, "rise_mm": None,
                               "reason": "error: " + _err(exc)})
            if not ok:
                lifted = False
                break
        out["%s|%d" % (rid, i)] = {"lifted": lifted, "trials": trials}
    return out


def _child_main(conn, ctx, groups, rows, models):
    try:
        res = _run_groups(ctx, groups, rows, models)
        conn.send(json.dumps(_clean(res), sort_keys=True, allow_nan=False))
    except BaseException as exc:  # noqa: BLE001 - reported to the parent, which fails the stage
        conn.send(json.dumps({"__error__": _err(exc)}))
    finally:
        conn.close()


def run_lifts(ctx, rows):
    """{"<id>|<placement>": {"lifted", "trials"}} merged in sorted key order; work split over up to lift_workers
    processes (this one and forked children), each trial on a fresh model compiled from the variant's XML."""
    groups = lift_groups(ctx, rows)
    if not groups:
        return {}
    models = [T.load(ctx.gt_dir, [float(v[0]), float(v[1])]) for v in ctx.gcfg["lift_variants"]]
    n = max(1, min(int(ctx.gcfg.get("lift_workers", 1)), len(groups)))
    shares = [groups[k::n] for k in range(n)]
    children = []
    if n > 1:
        import multiprocessing as mp
        mpc = mp.get_context("fork")
        for share in shares[1:]:
            a, b = mpc.Pipe(duplex=False)
            p = mpc.Process(target=_child_main, args=(b, ctx, share, rows, models), daemon=True)
            p.start()
            b.close()
            children.append((p, a))
    merged = {}
    try:
        merged.update(_run_groups(ctx, shares[0], rows, models))
        for p, a in children:
            msg = json.loads(a.recv())
            if "__error__" in msg:
                raise RuntimeError("a lift worker failed: %s" % msg["__error__"])
            merged.update(msg)
            p.join()
    finally:
        for p, a in children:
            if p.is_alive():
                p.terminate()
                p.join()
            a.close()
    return {k: merged[k] for k in sorted(merged)}


def apply_lifts(ctx, rows, lifts):
    """A geometric done at a lift group whose lift failed becomes a miss (by "lift")."""
    for key, res in lifts.items():
        rid, i = key.rsplit("|", 1)
        i = int(i)
        if not res["lifted"] and rows[rid]["outcomes"][i] == "done":
            rows[rid]["outcomes"][i] = "miss"
            rows[rid]["by"][i] = "lift"
    return rows


def per_request_shares(rows, n_place):
    out = {}
    for rid, row in rows.items():
        o = row["outcomes"]
        out[rid] = {"done": o.count("done") / n_place, "stop": o.count("stop") / n_place,
                    "declined": o.count("declined") / n_place}
    return out


# ---------------------------------------------------------------------------------------------------------------
# one submission

def _cpu():
    """CPU seconds of this process and of its finished children (the lift workers)."""
    import resource
    ch = resource.getrusage(resource.RUSAGE_CHILDREN)
    return time.process_time() + ch.ru_utime + ch.ru_stime


def _new_result():
    """The result of one grade. "timing" holds clock values (wall seconds): logged only, never in a verdict or a
    grade dump; "cpu_s" is the dump's one non-deterministic block."""
    return {"G0": 0.0, "registered": 0.0, "use": 0.0, "done_share": 0.0, "stop_share": 0.0,
            "declined_share": 0.0, "gate": {}, "settle": {}, "registration": {}, "requests": {}, "plans": {},
            "units": {}, "weights": {}, "lifts": {}, "stages": {}, "diagnostics": {}, "cpu_s": {},
            "timing": {"wall_s": {}}, "stats_requests": {}}


def grade_submission(sub_dir, ctx, guard, record=None):
    """G0, the registration, the requests, the lifts and the use of one submission folder."""
    res = _new_result()
    strict = set((ctx.gcfg.get("runtime") or {}).get("timeout_is_task_error") or [])

    def stage(name, fn, *args):
        t, c = time.time(), _cpu()
        try:
            out = guard.run(name, fn, *args)
            res["stages"][name] = {"ok": True}
            return True, out
        except (TaskError, T.TruthError):
            raise
        except BaseException as exc:  # noqa: BLE001 - the twin's fault or the grader's limits: this stage scores 0
            if isinstance(exc, (KeyboardInterrupt, SystemExit)):
                raise
            if isinstance(exc, StageTimeout) and name in strict:
                raise TaskError("the verifier ran out of time in stage %s: the episode is void" % name)
            res["stages"][name] = {"ok": False, "error": _err(exc)}
            return False, None
        finally:
            res["cpu_s"][name] = _cpu() - c
            res["timing"]["wall_s"][name] = time.time() - t
            if record:
                record(res)

    # G0: the gate, then the settle
    ok, run = stage("gate", load.gate_twin, sub_dir)
    if not ok:
        res["diagnostics"]["G0"] = "the gate did not complete: " + res["stages"]["gate"]["error"]
        return res
    res["gate"] = _clean(run.report)
    if not run.ok:
        res["diagnostics"]["G0"] = "gate: " + "; ".join(run.reasons[:4])
        return res
    ok, _run = stage("settle", load.settle_twin, run, ctx.held())
    if not ok:
        res["diagnostics"]["G0"] = "the settle did not complete: " + res["stages"]["settle"]["error"]
        return res
    res["settle"] = load.settle_summary(run)
    if run.blowup:
        res["diagnostics"]["G0"] = "settle: " + "; ".join(run.reasons[-2:])
        return res
    res["G0"] = 1.0
    record and record(res)

    # the registration
    def _register():
        pieces, top_T = load.settled_pieces(run)
        rcfg = dict(ctx.gcfg["registration"], order=ctx.thing_order())
        tw_pts = register.ref_points(pieces, float(rcfg.get("raster_mm", 1.0)), rcfg.get("max_footprint_cells"))
        tr_pts = {k: np.asarray(v, dtype=np.float64) for k, v in ctx.gt["ref_points"]["points"].items()}
        fit_res = register.fit(tw_pts, tr_pts, rcfg)
        return pieces, top_T, tw_pts, fit_res

    ok, out = stage("registration", _register)
    if not ok:
        res["diagnostics"]["registration"] = "the registration failed: " + res["stages"]["registration"]["error"]
        return res
    pieces, top_T, tw_pts, fit_res = out
    res["registration"] = _clean(dict(fit_res, twin_points_mm={k: v for k, v in tw_pts.items()},
                                      present=sorted(pieces)))
    if not fit_res["ok"]:
        res["diagnostics"]["registration"] = "not registered: %s" % fit_res["reason"]
        return res
    res["registered"] = 1.0
    reg_pieces = register.transform_pieces({k: v for k, v in pieces.items() if v}, fit_res)

    # the requests
    def _requests():
        twin_scene = ctx.HI.Scene.from_pieces(reg_pieces)
        return evaluate_requests(ctx, twin_scene, set(reg_pieces))

    ok, rows = stage("requests", _requests)
    if not ok:
        res["diagnostics"]["requests"] = "the requests could not be planned: " + res["stages"]["requests"]["error"]
        return res

    def _keep_rows():
        res["requests"] = {rid: {k: list(row[k]) if isinstance(row[k], list) else row[k]
                                 for k in ("planned", "declined_by", "blocker", "outcomes", "by", "hit", "error")}
                           for rid, row in rows.items()}
        res["plans"] = {rid: row["plan"] for rid, row in rows.items() if row["plan"] is not None}
        res["stats_requests"] = {"errors": sum(1 for r in rows.values() if r["error"]),
                                 "planned": sum(1 for r in rows.values() if r["planned"])}

    _keep_rows()
    # the lifts
    if ctx.gcfg["dynamic_lifts"]:
        ok, lifts = stage("lifts", run_lifts, ctx, rows)
        if not ok:
            # a trial that raises already counts as not lifted; a stage that fails as a whole is the verifier's
            # fault (a worker lost), not the twin's: no score from it
            raise TaskError("the lifts did not run: " + res["stages"]["lifts"]["error"])
        apply_lifts(ctx, rows, lifts)
        res["lifts"] = lifts
        _keep_rows()

    # the use
    n_place = len(ctx.icfg["placements_mm"])
    ok, u = stage("use", use_score.use, per_request_shares(rows, n_place), ctx.gt["requests"],
                  {"c": ctx.gcfg["c"], "thing_cap": ctx.gcfg["thing_cap"]})
    if not ok:
        raise TaskError("the use could not be computed: %s" % res["stages"]["use"]["error"])
    res.update({"use": u["use"], "done_share": u["done_share"], "stop_share": u["stop_share"],
                "declined_share": u["declined_share"] or 0.0, "units": u["units"], "weights": u["weights"],
                "use_raw": u["use_raw"]})
    return res


def twin_score(res, g1=1.0):
    return float(g1) * float(res.get("G0", 0.0)) * (float(res.get("use", 0.0)) if res.get("registered") else 0.0)


# ---------------------------------------------------------------------------------------------------------------
# the worker (a separate process started by verify.py)

class Progress:
    def __init__(self, path):
        self.path = path
        self.doc = {"format": FORMAT_PROGRESS, "selftest": None, "submission": None, "done": False}

    def write(self):
        import resource
        self.doc["peak_rss_mb"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(_clean(self.doc), f, allow_nan=False, sort_keys=True)
        os.replace(tmp, self.path)


def worker_main(job_path):
    """Entry of the worker process: limits itself, self-tests the truth, grades the submission; every step is written
    to the progress file. Exit 0 when done, 3 on a task error."""
    import resource
    with open(job_path) as f:
        job = json.load(f)
    lim = int(job["address_space_mb"]) * 1024 * 1024
    resource.setrlimit(resource.RLIMIT_AS, (lim, lim))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    prog = Progress(job["progress"])
    prog.write()
    deadline = float(job["deadline_wall"])
    caps = job.get("stage_caps_s", {})
    with open(job["grader_config"]) as f:
        gcfg = json.load(f)
    t0, c0 = time.time(), time.process_time()
    try:
        ctx = Context(job["gt"], gcfg)
        st = Guard(deadline, {"selftest": caps.get("selftest", float("inf"))}).run("selftest", run_selftest, ctx)
        prog.doc["selftest"] = dict(st, cpu_s=time.process_time() - c0, wall_s=time.time() - t0)
    except BaseException as exc:  # noqa: BLE001 - a truth that cannot be built or checked is a task error
        if isinstance(exc, KeyboardInterrupt):
            raise
        import traceback
        prog.doc["selftest"] = {"ok": False, "reason": _err(exc), "trace": traceback.format_exc()[-1500:]}
        prog.write()
        return 3
    prog.write()

    def record(res):
        prog.doc["submission"] = res
        prog.write()

    try:
        res = grade_submission(job["submission"], ctx, Guard(deadline, caps), record=record)
    except BaseException as exc:  # noqa: BLE001 - only task errors reach here
        if isinstance(exc, KeyboardInterrupt):
            raise
        import traceback
        prog.doc["task_error"] = {"reason": _err(exc) if not isinstance(exc, TaskError) else str(exc),
                                  "trace": traceback.format_exc()[-1500:]}
        prog.write()
        return 3
    res["twin_score_g1_1"] = twin_score(res)
    prog.doc["submission"] = res
    prog.doc["done"] = True
    prog.write()
    return 0


if __name__ == "__main__":  # pragma: no cover - verify.py starts the worker through a bootstrap
    sys.exit(worker_main(sys.argv[1]))
