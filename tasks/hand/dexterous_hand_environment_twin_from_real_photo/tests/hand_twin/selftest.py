"""The verify-time self-test of the truth files. Any failure is a task error (TruthError), never a score.

    st = run(gt_dir, gcfg, icfg, envelope, HI, keep=None)

1. Integrity: every file of tests/gt/ against truth_manifest.json (sha256 and size), and no gt file outside it.
2. The constants that shaped the truth agree with the grader's configuration: requests.json's dynamic_lifts equals
   grader_config's (and with dynamic lifts off no request is flagged for a lift); every lift variant of the
   configuration is a (mass scale, friction) pair of the truth's constants, the self-test's lift variant is one of
   them; every thing's mass is its volume times the truth's density; the thing sets of things.json, ref_points.json,
   reference_pieces.npz and the configuration's thing order are the same; the requests and the dropped ids are
   distinct; selftest.json holds every scored id and no other; its lift entries are the first creditable lift request
   of each lift unit (id order); every scored request's rule is one of the instrument's (rule_order); the request
   set is not below the stop line (grader_config request_set: at least min_credit_things things with a credit unit
   and at least min_credit_units credit units; below it the build stops: it is never a score).
3. The reference's geometric outcome at the centre placement, replayed for every scored request from the stored
   settled and registered reference pieces (reference_pieces.npz: no gate or settle runs for the reference here):
   planned on the reference, executed on the scans, equal to selftest.json (outcome, by, hit), and consistent with the
   request's class (creditable: done; stop-only: neither done nor a stop).
4. With dynamic lifts: one lift per lift unit at the configured variant, equal to selftest.json (lifted exactly; the
   rise within the stored value's own rounding).
"""
import itertools
import math
import os

from . import truth as T
from . import use_score

RISE_TOL_MM = 0.05            # selftest.json stores the rise rounded to 0.1 mm


def _err(msg):
    raise T.TruthError(msg)


def check_manifest(gt_dir, manifest):
    listed = {}
    for row in manifest.get("files") or []:
        listed[row["name"]] = row
    present = []
    for dirpath, dirnames, filenames in os.walk(gt_dir):
        dirnames.sort()
        for f in sorted(filenames):
            rel = os.path.relpath(os.path.join(dirpath, f), gt_dir).replace(os.sep, "/")
            if rel != "truth_manifest.json":
                present.append(rel)
    extra = sorted(set(present) - set(listed))
    missing = sorted(set(listed) - set(present))
    if extra or missing:
        _err("the gt tree does not match truth_manifest.json: not listed %s, missing %s" % (extra[:5], missing[:5]))
    bad = []
    for rel in sorted(listed):
        p = os.path.join(gt_dir, *rel.split("/"))
        if os.path.getsize(p) != int(listed[rel]["size"]) or T.sha256_file(p) != listed[rel]["sha256"]:
            bad.append(rel)
    if bad:
        _err("gt files differ from truth_manifest.json: %s" % bad[:5])
    return len(listed)


def lift_units(requests_json):
    """{unit key: the first creditable lift request's id} in id order."""
    out = {}
    for r in sorted(requests_json["requests"], key=lambda r: r["id"]):
        if r["class"] == "creditable" and r.get("lift"):
            out.setdefault(use_score.unit_key(r["thing"], r["rule"]), r["id"])
    return out


def check_constants(gt, gcfg):
    things, req, st = gt["things"], gt["requests"], gt["selftest"]
    dyn = bool(gcfg["dynamic_lifts"])
    if bool(req.get("dynamic_lifts")) != dyn:
        _err("requests.json says dynamic_lifts %s, the grader's configuration %s" % (req.get("dynamic_lifts"), dyn))
    flagged = [r["id"] for r in req["requests"] if r.get("lift")]
    if not dyn and flagged:
        _err("dynamic lifts are off but %d requests are flagged for a lift" % len(flagged))
    bad_flag = [r["id"] for r in req["requests"] if r.get("lift") and (r["rule"] != "body" or r["class"] != "creditable")]
    if bad_flag:
        _err("lift flags on requests that are not creditable body pinches: %s" % bad_flag[:5])
    const = things["constants"]
    pairs = set(itertools.product([float(m) for m in const["mass_scales"]], [float(f) for f in const["frictions"]]))
    variants = [tuple(float(x) for x in v) for v in gcfg["lift_variants"]]
    if not variants or any(v not in pairs for v in variants):
        _err("the lift variants %s are not (mass scale, friction) pairs of the truth's constants" % (variants,))
    lv = tuple(float(x) for x in gcfg["selftest"]["lift_variant"])
    if lv not in variants:
        _err("the self-test's lift variant %s is not one of the lift variants" % (lv,))
    rho = float(const["density_kg_m3"])
    for k, t in things["things"].items():
        if not math.isclose(float(t["mass_kg"]), rho * float(t["volume_m3"]), rel_tol=1e-9, abs_tol=0.0):
            _err("the mass of %s is not its volume times the density" % k)
    keys = set(things["things"])
    if set(gt["ref_points"]["points"]) != keys:
        _err("ref_points.json and things.json name different things")
    if set(gt["reference_pieces"]) != keys:
        _err("reference_pieces.npz and things.json name different things")
    order = gcfg.get("things")
    if order is not None and (sorted(order) != sorted(keys) or len(set(order)) != len(order)):
        _err("the configuration's thing order and things.json name different things")
    ids = [r["id"] for r in req["requests"]] + [d["id"] for d in req["dropped"]]
    if len(ids) != len(set(ids)):
        _err("an id appears twice among the requests and the dropped")
    unknown = sorted({r["thing"] for r in req["requests"]} - keys)
    if unknown:
        _err("scored requests on things the truth does not have: %s" % unknown)
    scored = {r["id"] for r in req["requests"]}
    if set(st["geometric"]) != scored:
        _err("selftest.json's geometric entries are not exactly the scored requests")
    want = sorted(lift_units(req).values()) if dyn else []
    got = sorted(e["id"] for e in st.get("lifts") or [])
    if got != want:
        _err("selftest.json's lifts %s are not the first lift request of each lift unit %s" % (got[:5], want[:5]))
    for e in st.get("lifts") or []:
        if tuple(float(x) for x in e["variant"]) != lv:
            _err("selftest.json's lift %s has variant %s, the configuration %s" % (e["id"], e["variant"], lv))
    try:
        counts = use_score.check_request_set(req, gcfg)
    except use_score.RequestSetBelowStopLine as exc:
        _err("the request set is below the stop line: %s" % exc)
    try:
        use_score.weights(req, gcfg)
    except use_score.CapInfeasible as exc:
        _err("the thing cap cannot hold on this request set: %s" % exc)
    caps = gcfg.get("caps") or {}
    return {"lift_variants": len(variants), "things": len(keys), "scored": len(scored), "dropped": len(req["dropped"]),
            "lift_units": len(want), "caps": dict(caps), "request_set": counts}


def dir_or_side(r):
    return r["side"] if r["rule"] == "hook" else r["dir_deg"]


def replay_geometric(gt, HI, icfg, env, ref_scene, truth_scene):
    centre = icfg["placements_mm"][0]
    wrong = []
    cls_wrong = []
    for r in sorted(gt["requests"]["requests"], key=lambda r: r["id"]):
        ex = HI.plan_explain(ref_scene, r["thing"], r["rule"], dir_or_side(r), icfg, env)
        o = HI.execute(ex["plan"], truth_scene, centre, icfg, env)
        got = {"outcome": o.get("outcome"), "by": o.get("by"), "hit": o.get("hit")}
        exp = gt["selftest"]["geometric"][r["id"]]
        if got != {"outcome": exp.get("outcome"), "by": exp.get("by"), "hit": exp.get("hit")}:
            wrong.append((r["id"], got, exp))
        if (r["class"] == "creditable") != (got["outcome"] == "done") or got["outcome"] == "stop":
            cls_wrong.append((r["id"], r["class"], got["outcome"]))
    if wrong:
        _err("the reference's replay differs from selftest.json on %d requests: %s" % (len(wrong), wrong[:3]))
    if cls_wrong:
        _err("the reference's centre outcome contradicts the class on %d requests: %s" % (len(cls_wrong),
                                                                                          cls_wrong[:3]))
    return len(gt["requests"]["requests"])


def replay_lifts(gt, gt_dir, HI, icfg, env, ref_scene, gcfg):
    entries = gt["selftest"].get("lifts") or []
    if not entries:
        return 0
    lv = [float(x) for x in gcfg["selftest"]["lift_variant"]]
    xml, bodies = T.load(gt_dir, lv)
    by_id = {r["id"]: r for r in gt["requests"]["requests"]}
    held = T.held_keys(gt["things"])
    centre = icfg["placements_mm"][0]
    wrong = []
    for e in sorted(entries, key=lambda e: e["id"]):
        r = by_id[e["id"]]
        plan = HI.plan(ref_scene, r["thing"], r["rule"], dir_or_side(r), icfg, env)
        if plan is None:
            wrong.append((e["id"], "no plan on the reference"))
            continue
        got = HI.lift(plan, xml, bodies, lv, centre, icfg, held=held)
        if bool(got.get("lifted")) != bool(e["lifted"]) or not (
                abs(float(got.get("rise_mm", float("nan"))) - float(e["rise_mm"])) <= RISE_TOL_MM + 1e-9):
            wrong.append((e["id"], got, e))
    if wrong:
        _err("the reference's lifts differ from selftest.json: %s" % wrong[:3])
    return len(entries)


def run(gt_dir, gcfg, icfg, env, HI, keep=None):
    """The self-test; returns its summary, fills keep["gt"], keep["ref_scene"], keep["truth_scene"]."""
    gt = T.read_gt(gt_dir)
    n_files = check_manifest(gt_dir, gt["manifest"])
    consts = check_constants(gt, gcfg)
    rules = sorted({r["rule"] for r in gt["requests"]["requests"]} - set(icfg.get("rule_order") or []))
    if rules:
        _err("scored requests name rules the instrument does not have: %s" % rules)
    if len(icfg.get("placements_mm") or []) != 9:
        _err("the instrument configuration does not hold nine placements")
    if int(gcfg["selftest"].get("geometric_placements", 1)) != 1:
        _err("the self-test replays the centre placement only (selftest.geometric_placements must be 1)")
    ref_scene = HI.Scene.from_pieces(gt["reference_pieces"])
    truth_scene = HI.Scene.from_gt(gt_dir)
    missing = sorted(set(gt["things"]["things"]) - set(truth_scene.things))
    if missing:
        _err("the truth scene lacks %s" % missing)
    n_geo = replay_geometric(gt, HI, icfg, env, ref_scene, truth_scene)
    n_lift = replay_lifts(gt, gt_dir, HI, icfg, env, ref_scene, gcfg) if gcfg["dynamic_lifts"] else 0
    if keep is not None:
        keep.update({"gt": gt, "ref_scene": ref_scene, "truth_scene": truth_scene})
    return {"ok": True, "manifest_files": n_files, "geometric_replayed": n_geo, "lifts_replayed": n_lift,
            "constants": consts}
