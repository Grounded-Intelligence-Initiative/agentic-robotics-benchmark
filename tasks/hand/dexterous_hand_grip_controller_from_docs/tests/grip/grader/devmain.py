"""Maintainers' runs of the acceptance outside the verify stage: a submission, the privileged reference or a reference
with one changed belief, on the frozen set or on one development scenario, in-process or through the wire.

    python3 -I -B <this module> (--frozen DIR | --scenario FILE --dev)
                             (--submission DIR | --oracle --echo DIR | --variant NAME --variants FILE --echo DIR)
                             [--wire] [--units 1,2] [--runs-root DIR] [--dev-hooks MODULE] --out DIR

DIR of --submission / --echo is a controller folder (controller.py and whatever it ships). --units gives the units in
the order they run (positions in the frozen set: 1 = acc1, 2 = acc2; "1,1" runs unit 1 twice). Writes to --out:
score.json (deterministic: reward, raw, pass conditions, per shift S, J, carry mean, events), jobs.jsonl (one job
record per line, in run order) and stats.json (CPU model, host load, per-unit CPU and timing; never in score.json).
In-process runs cannot measure the network or stray processes: those two pass conditions read 1 there. A submission
that fails the static checks is not run (its score is 0 and its shifts read 0).
Exit 0 on a finished run, 1 on a task error, 2 on a usage or input error.
"""
# --- bootstrap A: find the folders of the task's packages above this file and put them first on sys.path ---
import os as _os
import sys as _sys


def _grip_roots(_here):
    _d = _os.path.dirname(_os.path.abspath(_here))
    while True:
        if _os.path.isdir(_os.path.join(_d, "contract")):
            if _os.path.isdir(_os.path.join(_d, "hidden")):
                return [_os.path.join(_d, "hidden"), _d]
            if _os.path.isdir(_os.path.join(_d, "grader")):
                return [_d]
        _p = _os.path.dirname(_d)
        if _p == _d:
            raise SystemExit("bootstrap: no task root above " + str(_here))
        _d = _p


for _q in _grip_roots(__file__):
    while _q in _sys.path:
        _sys.path.remove(_q)
    _sys.path.insert(0, _q)
del _grip_roots, _q
# --- end of bootstrap A ---
import argparse
import glob
import importlib.util
import json
import os
import shutil
import sys
import tempfile

from contract import constants as C
from contract import names as N

from grader import acceptance as A
from grader import metrics as M

CLIENT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runtime", "client.py")


def dumps(obj):
    return json.dumps(obj, sort_keys=True, indent=1, allow_nan=False) + "\n"


def static_checks(folder):
    """{"controller_file", "size", "import_audit"} of a controller folder, and the reasons of the failed ones: the same
    caps walk (netaudit.tree_shape) and the same static checks in a child process (netaudit.audit_in_child) as at
    verify."""
    from grader.runtime import netaudit
    I = C.INTERFACE
    out = {"controller_file": False, "size": False, "import_audit": False}
    shape = netaudit.tree_shape(folder, I["entries_max"], I["depth_max"], I["size_cap_mb"] << 20)
    if not shape["ok"]:
        return out, ["the controller folder %s" % shape["why"]]
    out["size"] = True
    res, failed = netaudit.audit_in_child(folder, I["source_file_max_mb"] << 20, I["audit_cpu_s"], I["audit_wall_s"])
    if res is None:
        return out, [failed]
    why = []
    p = os.path.join(folder, "controller.py")
    is_reg = os.path.isfile(p) and not os.path.islink(p)
    out["controller_file"] = bool(is_reg and res.get("controller_file"))
    if not out["controller_file"]:
        why.append(str(res.get("controller_error") or "no controller.py (regular file)")[:300])
    out["import_audit"] = int(res.get("n_hits") or 0) == 0
    if res.get("hits"):
        why.append("import audit: %s" % ", ".join("%s: %s" % tuple(x) for x in res["hits"][:5]))
    return out, why


def load_controller(folder, tag):
    """A fresh instance of the folder's Controller (its module loaded anew, the folder first on sys.path)."""
    path = os.path.join(folder, "controller.py")
    spec = importlib.util.spec_from_file_location("grip_controller_%s" % tag, path)
    mod = importlib.util.module_from_spec(spec)
    sys.path.insert(0, folder)
    try:
        spec.loader.exec_module(mod)
    finally:
        if sys.path and sys.path[0] == folder:
            sys.path.pop(0)
    return mod.Controller(tempfile.mkdtemp(prefix="grip_ctl_"))


def frozen_scenarios(d, hooks):
    man = os.path.join(d, "manifest.json")
    with open(man) as fh:
        manifest = json.load(fh)
    kept = manifest.get("kept")
    files = [os.path.join(d, k["file"]) for k in kept] if kept else sorted(glob.glob(os.path.join(d, "acc*.scenario")))
    if not files:
        raise SystemExit("no scenario in the frozen set at " + d)
    if hooks and hooks.get("load_scenario"):
        return [hooks["load_scenario"](f) for f in files]
    from world import scenarios as SC
    return [SC.load(f, manifest=man) for f in files]


def parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--frozen")
    src.add_argument("--scenario")
    ap.add_argument("--dev", action="store_true")
    who = ap.add_mutually_exclusive_group(required=True)
    who.add_argument("--submission")
    who.add_argument("--oracle", action="store_true")
    who.add_argument("--variant")
    ap.add_argument("--variants")
    ap.add_argument("--echo")
    ap.add_argument("--wire", action="store_true")
    ap.add_argument("--units", default=None)
    ap.add_argument("--runs-root", default=None)
    ap.add_argument("--dev-hooks", default=None)
    ap.add_argument("--out", required=True)
    return ap.parse_args(argv)


def main(argv=None):
    try:
        a = parse_args(sys.argv[1:] if argv is None else argv)
    except SystemExit as e:
        return 2 if e.code else 0
    if a.scenario and not a.dev:
        print("usage error: --scenario runs development scenarios only (add --dev)", file=sys.stderr)
        return 2
    if (a.oracle or a.variant) and not a.echo:
        print("usage error: --oracle and --variant need --echo <the reference controller's folder>", file=sys.stderr)
        return 2
    if a.variant and not a.variants:
        print("usage error: --variant needs --variants <file>", file=sys.stderr)
        return 2
    hooks = None
    if a.dev_hooks:
        mod = importlib.import_module(a.dev_hooks)
        hooks = {"session_class": mod.session_class(), "cell_parts": mod.cell_parts(), "make_script": mod.make_script,
                 "load_scenario": getattr(mod, "load_scenario", None)}
    if a.frozen:
        scen = frozen_scenarios(a.frozen, hooks)
    elif hooks and hooks.get("load_scenario"):
        scen = [hooks["load_scenario"](a.scenario)]
    else:
        from world import scenarios as SC
        scen = [SC.load(a.scenario, dev=True)]
    default_units = ",".join(str(i + 1) for i in range(len(scen)))
    try:
        order = [int(x) for x in (a.units or default_units).split(",")]
    except ValueError:
        print("usage error: --units is a comma list of unit positions", file=sys.stderr)
        return 2
    if not order or any(u < 1 or u > len(scen) for u in order):
        print("usage error: --units must name units 1..%d" % len(scen), file=sys.stderr)
        return 2
    beliefs = None
    if a.variant:
        with open(a.variants) as fh:
            variants = json.load(fh)
        if a.variant not in variants:
            print("usage error: no variant %r in %s" % (a.variant, a.variants), file=sys.stderr)
            return 2
        beliefs = variants[a.variant]["beliefs"]
    privileged = bool(a.oracle or a.variant)
    folder = os.path.abspath(a.echo if privileged else a.submission)
    os.makedirs(a.out, exist_ok=True)
    gates, why = static_checks(folder)
    gates.update({"setup": False, "on_time": False, "no_late_write": True, "clean": False, "network": False})
    runs, stats_units = [], []
    log = lambda m: print(m, file=sys.stderr, flush=True)     # noqa: E731
    if all(gates[k] for k in ("controller_file", "size", "import_audit")):
        runs_root = os.path.abspath(a.runs_root or os.path.join(a.out, "runs"))
        for pos, u in enumerate(order):
            sc = scen[u - 1]
            runner = None
            if a.wire:
                from grader.runtime import wire as W
                runner = W.Runner(folder, os.path.join(runs_root, "run%d_unit%d" % (pos + 1, u)), CLIENT,
                                  stderr_path=os.path.join(runs_root, "run%d_unit%d.stderr.log" % (pos + 1, u)),
                                  extra_env=dict(A.BLAS_ENV), cpu_cap_s=C.INTERFACE["process_cpu_s"],
                                  stderr_cap_bytes=C.INTERFACE["stderr_log_max_mb"] << 20,
                                  end_wait_s=C.INTERFACE["end_wait_s"])
                os.makedirs(runs_root, exist_ok=True)
                ok, info = runner.start()
                ep = runner if ok else A.DeadEndpoint(info)
            else:
                try:
                    ep = A.LocalEndpoint(load_controller(folder, "%d_%d" % (pos, u)))
                except Exception as e:  # noqa: BLE001
                    ep = A.DeadEndpoint("import: %s: %s" % (type(e).__name__, e))
            recs, st = A.run_unit(sc, u, ep, privileged=privileged, beliefs=beliefs,
                                  session_class=(hooks or {}).get("session_class"),
                                  cell_parts=(hooks or {}).get("cell_parts"),
                                  make_script=(hooks or {}).get("make_script"), log=log)
            st["runner"] = runner.close() if runner is not None else ep.close()
            st["run_position"] = pos + 1
            runs.append({"unit": u, "records": recs, "events": st["events"]})
            stats_units.append(st)
            log("[dev run] run %d unit %d: %s" % (pos + 1, u, ", ".join(
                "%s/%s %s" % (r["shift"], r["job"], ("q=%.3f" % r["q"]) if r["q"] is not None else
                              ("declined" if r["declined"] else "-")) for r in recs)))
        gates["setup"] = all(s["setup_ok"] for s in stats_units)
        gates["on_time"] = all(s["on_time_fraction"] >= C.INTERFACE["on_time_min_fraction"] for s in stats_units)
        gates["clean"] = all(int(s["runner"].get("stray_processes", 0)) == 0 for s in stats_units)
        gates["network"] = all(bool(s["runner"].get("no_network")) for s in stats_units) if a.wire else True
        if a.runs_root is None:
            shutil.rmtree(runs_root, ignore_errors=True)
    else:
        for u in order:
            runs.append({"unit": u, "records": [], "events": []})
    sc_out = M.score(runs, gates)
    with open(os.path.join(a.out, "score.json"), "w") as fh:
        fh.write(dumps(sc_out))
    with open(os.path.join(a.out, "jobs.jsonl"), "w") as fh:
        for r in runs:
            for rec in r["records"]:
                fh.write(json.dumps(rec, sort_keys=True, allow_nan=False) + "\n")
    stats = {"format": "grip-stats/1", "cpu_model": A.host_cpu_model(), "argv": list(sys.argv[1:] if argv is None
                                                                                    else argv),
             "privileged": privileged, "wire": bool(a.wire), "units": stats_units, "static_check_reasons": why,
             "blas_env": {k: os.environ.get(k) for k in A.BLAS_ENV}}
    with open(os.path.join(a.out, "stats.json"), "w") as fh:
        json.dump(stats, fh, indent=1, sort_keys=True, default=str)
    print(json.dumps({"reward": sc_out["reward"], "raw": sc_out["raw"], "gates": sc_out["gates"]}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
