"""QC Q6 (task version 1.1): the selection checked again from the STORED site packages and from the privileged
reference's records, not from selection.json; with it the three-mouth counterfactual check of the generator.

    q6_recompute.py OUT.json [--only g|r] [--sites h01,...,p3]
                    [--oracle-summary SUMMARY.json ...] [--practice-run TAG=RUN.json ...]

G part, for each frozen package (10 hidden: tests/hidden/sites/h01-h10.tar.xz, pinned in seeds.json; 3 practice:
environment/practice_ground/sites/p1-p3.tar.xz, pinned in sites.json), opened from its bytes:
  A  integrity: archive sha256 = the pin; site.json / site.xml sha256 = the selection's set; the stored site.xml equals
     the generator's MJCF writer applied to the stored site.json (w3gen.mjcf.site_xml), byte for byte; the generator
     (the development folder's tools/sitegen/w3gen, run on the seed) rebuilds the stored site.json and site.xml byte
     for byte
  B  every G predicate recomputed by the generator's predicate code on the STORED bytes with the generator's bundle
     of the three counterfactual sites (w3gen.predicates.evaluate(..., bundle, cf_render=True): G3 rendered; G7c with
     the classic renderer's frames; G9 and G17-G19 on every counterfactual site): every pass flag and every value the
     selection records (G.flags, G.values, G17_G19_counterfactual) is compared
  CF the counterfactual check (each counterfactual site with its own rubble): for the two other mouths read as the
     true one, the 2 x 2 supersampled camera rays (hit geometry and position), the hit names, the public view, the
     mission() fields and the classic render (RGB and depth) are identical to the true site's; the number of rubble
     boxes of every counterfactual site
  C  an independent second computation (this file's own code, from site.json only): G10 (pad to each point >= 1.2 m,
     points pairwise >= 1.0 m); G16 (point 3 0.64-0.72 m from the back wall and from the corner's side wall); G8a's
     clearance (point 3 to every solid, the rubble included, and to every site edge >= 0.62 m); per rubble box: the
     name prefix rubble__void_debris and kind rubble, one long side on a wall of its run, from the floor to the slab
     underside, 0.20-0.30 m along the run, a gap of exactly the class-P width to the other wall, the intrusion into the
     W_SWEEP-wide centre band >= 0.04 m, >= 0.62 m from point 3, two boxes of one run >= 0.8 m apart; >= 1 box
  D  set level: hidden 10 / practice 3 are the selection's sets in order; the strata quotas (third opening T / XN /
     XL 3 / 3 / 4, corner E / W 5 / 5, near rank 0 / 1 / 2 each <= 4; practice: one of each third, both corners, one
     of each rank); the true mouth west / middle / east >= 2 each; flat / lean roof >= 4 each; seeds, archive,
     site.json and site.xml sha256 disjoint between hidden and practice; 13 different geometries
R part (R1-R8 of the selection rule from the reference's records on the frozen bytes): hidden sites from the
oracle's verify runs (--oracle-summary: qc_verify.py / the freeze's anchor runs, the cell's own records); practice
sites from runs of the pinned reference on the frozen packages (--practice-run: the development folder's
tools/reference/run_station.py outputs, the selection's own path). R1 c = (1, 1, 1), F = 1, no stuck event, not
aborted; R2 reference time <= time limit / 1.6; R3 primitive count in the frozen window; R4 no end triangle; R5 the
blind leg going in >= 16 primitives; R6 strut-axis penetration <= 5 mm; R7 the final state hash equal across the
runs and equal to the pin; R8 no contact sample with any rubble__void_debris box and, at every primitive boundary
inside a box's stretch of its run (0.15 m margin), the centre within 0.07 m of the gap's centreline.
"""
import argparse
import bisect
import hashlib
import io
import json
import lzma
import math
import os
import sys
import tarfile

HERE = os.path.dirname(os.path.abspath(__file__))
DEV = os.path.normpath(os.path.join(HERE, "..", ".."))
TASK = DEV[:-len("-dev")]
TESTS = os.path.join(TASK, "tests")
PG = os.path.join(TASK, "environment", "practice_ground")
sys.path.insert(0, os.path.join(DEV, "tools", "sitegen"))

DEBRIS = "rubble__void_debris"
R3_WINDOW = (116, 146)
R6_MM = 5.0
R8_OFF = 0.07


def sha(b):
    return hashlib.sha256(b).hexdigest()


def members(path):
    raw = open(path, "rb").read()
    tf = tarfile.open(fileobj=io.BytesIO(lzma.decompress(raw)), mode="r:")
    mem = {os.path.basename(m.name): tf.extractfile(m).read() for m in tf.getmembers() if m.isfile()}
    return raw, mem


def frozen_sets():
    seeds = json.load(open(os.path.join(TESTS, "hidden", "seeds.json")))
    pgs = json.load(open(os.path.join(PG, "sites.json")))
    sel = json.load(open(os.path.join(TESTS, "hidden", "selection.json")))
    out = []
    for r in seeds["sites"]:
        out.append(dict(tag=r["tag"], kind="hidden", seed=int(r["site_seed"]), path=os.path.join(TESTS, "hidden", r["archive"]),
                        pin=r["sha256"], pin_json=r["site_json_sha256"], pin_xml=r["site_xml_sha256"],
                        candidate=r["candidate"], pinned_final=r.get("reference_final_state_sha256")))
    prac = {s["name"]: s for s in pgs["sites"] if s.get("kind") == "practice"}
    for r in sel["set"]["practice"]:
        p = prac[r["tag"]]
        out.append(dict(tag=r["tag"], kind="practice", seed=int(r["seed"]), path=os.path.join(PG, p["package"]),
                        pin=p["sha256"], pin_json=r["site_json_sha256"], pin_xml=r["site_xml_sha256"],
                        candidate=r["candidate"], pinned_final=r.get("reference_final_state_sha256")))
    return out, sel


def close(a, b, tol):
    if isinstance(a, bool) or isinstance(b, bool) or a is None or b is None:
        return a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(float(a) - float(b)) <= tol
    return a == b


VALUE_MAP = {   # selection.json G.values key -> (qc path, tolerance)
    "G2_corner_margin_px": (("G2", "corner_margin_px"), 0.006),
    "G3_max_px": (("G3", "as_built", "max_px"), 0.0011),
    "G4_visible_fraction_open_floor": (("G4", "visible_fraction_open_floor_0p20"), 1e-4),
    "G6_mm_per_px_far_centre": (("G6", "mm_per_px_far_centre"), 6e-4),
    "G7_plug_rays_hit": (("G7", "plug_rays_hit"), 0),
    "G7_hidden_box_rays_hit": (("G7", "hidden_box_rays_hit"), 0),
    "G7_debris_rays_hit": (("G7", "debris_rays_hit"), 0),
    "G8a_sp3_clearance_m": (("G8a", "sp3_clearance_m"), 1e-4),
    "G8a_max_credit_touching": (("G8a", "max_credit_touching"), 1e-4),
    "G8a_max_credit_visible": (("G8a", "max_credit_visible"), 1e-4),
    "G8b_blind_route_m": (("G8b", "blind_route_m"), 1e-3),
    "G12_bends": (("G12", "bends"), 0),
    "G14_partition_m": (("G14", "partition_m"), 1e-6),
    "G16_d_back_m": (("G16", "d_back_m"), 1e-4),
    "G16_d_side_m": (("G16", "d_side_m"), 1e-4),
}


def dig(d, path):
    for k in path:
        if not isinstance(d, dict) or k not in d:
            return None
        d = d[k]
    return d


def independent(site):
    """C: this file's own reading of G8a's clearance, G10, G16 and the rubble rule, from site.json only."""
    lx, ly = site["size_m"]
    pts = {int(p["id"]): p["xy"] for p in site["search_points"]}
    pad = site["pad"]["xy"]
    sp3 = pts[3]
    env = site["envelope"]["values"]
    W = float(env["W_SWEEP"])
    P = float(site["classes"]["P"]["w"])
    out = {}
    dpad = {i: math.dist(pad, pts[i]) for i in (1, 2, 3)}
    dsp = {"1-2": math.dist(pts[1], pts[2]), "1-3": math.dist(pts[1], pts[3]), "2-3": math.dist(pts[2], pts[3])}
    out["G10"] = dict(ok=min(dpad.values()) >= 1.2 and min(dsp.values()) >= 1.0,
                      min_pad_m=round(min(dpad.values()), 4), min_pair_m=round(min(dsp.values()), 4))
    corner = site["stratum"]["corner"]
    d_back = ly - sp3[1]
    d_side = sp3[0] if corner == "W" else lx - sp3[0]
    out["G16"] = dict(ok=0.64 - 1e-6 <= d_back <= 0.72 + 1e-6 and 0.64 - 1e-6 <= d_side <= 0.72 + 1e-6,
                      d_back_m=round(d_back, 6), d_side_m=round(d_side, 6))
    solid_kinds = ("wall", "crate", "rubble", "support", "plug", "roof_support", "column")
    clr = min(sp3[0], lx - sp3[0], sp3[1], ly - sp3[1])
    nearest = "site edge"
    for b in site["boxes"]:
        if b["kind"] in ("roof",):
            continue
        if b["kind"] not in solid_kinds and not b["name"].startswith(("support__", "plug__", "rubble__", "wall")):
            continue
        x0, x1, y0, y1 = b["aabb"][:4]
        d = math.hypot(max(x0 - sp3[0], 0.0, sp3[0] - x1), max(y0 - sp3[1], 0.0, sp3[1] - y1))
        if d < clr:
            clr, nearest = d, b["name"]
    out["G8a_clearance"] = dict(ok=clr >= 0.62 - 1e-6, clearance_m=round(clr, 6), nearest=nearest)
    v = site["void"]
    runs = v["runs"]
    hz = [b for b in site["boxes"] if b["name"] == "roof__void_slab"][0]["aabb"][4]
    deb = [b for b in site["boxes"] if b["name"].startswith(DEBRIS)]
    rows = []
    for b in deb:
        x0, x1, y0, y1, z0, z1 = b["aabb"]
        run = [k for k, r in runs.items() if x0 >= r["x0"] - 1e-6 and x1 <= r["x1"] + 1e-6 and y0 >= r["y0"] - 1e-6
               and y1 <= r["y1"] + 1e-6]
        row = dict(name=b["name"], kind=b["kind"], run=run[0] if run else None)
        if not run:
            row["ok"] = False
            rows.append(row)
            continue
        r = runs[run[0]]
        if r["axis"] == "x":
            a0, a1, c0, c1, w0, w1 = x0, x1, y0, y1, r["y0"], r["y1"]
        else:
            a0, a1, c0, c1, w0, w1 = y0, y1, x0, x1, r["x0"], r["x1"]
        on_lo, on_hi = abs(c0 - w0) <= 1e-6, abs(c1 - w1) <= 1e-6
        gap = (w1 - c1) if on_lo else (c0 - w0)
        width = w1 - w0
        intr = (c1 - c0) - (width - W) / 2.0
        d3 = math.hypot(max(x0 - sp3[0], 0.0, sp3[0] - x1), max(y0 - sp3[1], 0.0, sp3[1] - y1))
        row.update(on_one_wall=on_lo != on_hi, floor_to_slab=abs(z0) <= 1e-6 and abs(z1 - hz) <= 1e-6,
                   length_m=round(a1 - a0, 6), gap_m=round(gap, 6), P=P, intrusion_m=round(intr, 6),
                   d_sp3_m=round(d3, 6))
        row["ok"] = bool(b["kind"] == "rubble" and row["on_one_wall"] and row["floor_to_slab"] and
                         0.20 - 1e-6 <= a1 - a0 <= 0.30 + 1e-6 and abs(gap - P) <= 1e-6 and intr >= 0.04 - 1e-6 and
                         d3 >= 0.62 - 1e-6)
        rows.append(row)
    pairs = []
    for rn in ("G1", "N", "K"):
        iv = sorted(((b["aabb"][0], b["aabb"][1]) if runs[rn]["axis"] == "x" else (b["aabb"][2], b["aabb"][3]))
                    for b, row in zip(deb, rows) if row.get("run") == rn) if rn in runs else []
        pairs += [q[0] - p[1] for p, q in zip(iv, iv[1:])]
    out["rubble"] = dict(ok=len(deb) >= 1 and all(r["ok"] for r in rows) and all(g >= 0.8 - 1e-6 for g in pairs),
                         n=len(deb), boxes=rows, pair_gaps_m=[round(g, 6) for g in pairs],
                         min_d_sp3_m=round(min([r.get("d_sp3_m", 9.0) for r in rows] or [9.0]), 6),
                         record_n=len((v.get("rubble") or {}).get("boxes") or []))
    out["ok"] = out["G10"]["ok"] and out["G16"]["ok"] and out["G8a_clearance"]["ok"] and out["rubble"]["ok"]
    return out


def g_part(rows_in, sel, only_tags=None):
    from w3gen import cli, family, mjcf, predicates
    cands = {c["seed"]: c for c in sel["candidates"]}
    res = {}
    for s in rows_in:
        if only_tags and s["tag"] not in only_tags:
            continue
        raw, mem = members(s["path"])
        site = json.loads(mem["site.json"].decode("utf-8"))
        xml = mem["site.xml"].decode("utf-8")
        a = dict(archive_pin=sha(raw) == s["pin"], site_json_pin=sha(mem["site.json"]) == s["pin_json"],
                 site_xml_pin=sha(mem["site.xml"]) == s["pin_xml"],
                 mjcf_writer_identical=mjcf.site_xml(site) == xml, seed_in_site=int(site["seed"]) == s["seed"])
        b = family.build_sites(s["seed"])
        a["rebuilt_site_json_identical"] = cli.dumps(b["site"]).encode("utf-8") == mem["site.json"]
        a["rebuilt_site_xml_identical"] = b["xml"].encode("utf-8") == mem["site.xml"]
        a["ok"] = all(a.values())
        q = predicates.evaluate(site, xml, render=True, g9=None, bundle=b, cf_render=True)
        c = cands[s["seed"]]
        flags = {k: bool(q[k]["pass_"]) for k in c["G"]["flags"]}
        flag_diff = {k: [flags.get(k), v] for k, v in c["G"]["flags"].items() if flags.get(k) != v}
        val_diff = {}
        for k, v in c["G"]["values"].items():
            if k == "G7c":
                g7c = q["G7"]["G7c"] or {}
                mine = {kk: g7c.get(kk) for kk in v}
                if mine != v:
                    val_diff[k] = [mine, v]
                continue
            if k not in VALUE_MAP:
                val_diff[k] = ["not mapped", v]
                continue
            path, tol = VALUE_MAP[k]
            got = dig(q, path)
            if path[0] == "G3" and got is None:
                got = dig(q, ("G3", "max_px"))
            if not close(got, v, tol):
                val_diff[k] = [got, v]
        cf_mine = {}
        for t, p in q["G17_G19"]["counterfactual"].items():
            cf_mine[t] = dict(mouth=p["mouth"], G9=p["G9"], G17=p["G17"], G18=p["G18"], G19=p["G19"], n_boxes=p["n_boxes"],
                              on_route=p["on_route"], route_with_rubble_m=p["route_with_rubble_m"],
                              min_intrusion_into_centre_band_m=min([bx["G18"]["intrusion_into_centre_band_m"]
                                                                    for bx in p["boxes"] if "G18" in bx] or [None]))
        cf_diff = {}
        for t, v in c["G"]["G17_G19_counterfactual"].items():
            m = cf_mine.get(t) or {}
            for kk, vv in v.items():
                if not close(m.get(kk), vv, 1e-6):
                    cf_diff.setdefault(t, {})[kk] = [m.get(kk), vv]
        g7c = q["G7"]["G7c"] or {}
        cf = dict(tau_true=b["tau_star"], other_taus=g7c.get("tau"), rays_identical=g7c.get("rays_identical"),
                  hit_names_identical=g7c.get("hit_names_identical"), mission_identical=g7c.get("mission_identical"),
                  public_identical=g7c.get("public_identical"), render_identical=g7c.get("render_identical"),
                  n_debris_other=g7c.get("n_debris"), n_debris_true=g7c.get("n_debris_true"),
                  hidden_box_rays_hit=q["G7"]["hidden_box_rays_hit"], debris_rays_hit=q["G7"]["debris_rays_hit"],
                  mouth_floor_depth_m=q["G7"]["mouth_floor_depth_m"])
        cf["ok"] = all(bool(x) for x in (cf["rays_identical"] or [False])) and len(cf["rays_identical"] or []) == 2 and \
            all(cf["hit_names_identical"]) and all(cf["mission_identical"]) and all(cf["public_identical"]) and \
            all(cf["render_identical"] or [False]) and cf["hidden_box_rays_hit"] == 0 and \
            all(d_ <= 0.20 for d_ in cf["mouth_floor_depth_m"].values())
        ind = independent(site)
        row = dict(tag=s["tag"], kind=s["kind"], seed=s["seed"], candidate=s["candidate"], A=a,
                   B=dict(pass_all=bool(q["pass"]), failed=q["failed"], flags=flags, flag_diff=flag_diff,
                          value_diff=val_diff, cf_diff=cf_diff, cf=cf_mine,
                          ok=bool(q["pass"]) and not flag_diff and not val_diff and not cf_diff),
                   CF=cf, C=ind, strata=site["stratum"], third=site["r1"]["third"], roof=site["r1"].get("tunnel_roof"),
                   true_mouth=site["void"]["true_mouth"], true_index=site["stratum"]["true_index"],
                   n_rubble=ind["rubble"]["n"])
        row["ok"] = a["ok"] and row["B"]["ok"] and cf["ok"] and ind["ok"]
        res[s["tag"]] = row
        print(json.dumps(dict(tag=s["tag"], ok=row["ok"], A=a["ok"], B=row["B"]["ok"], CF=cf["ok"], C=ind["ok"],
                              flag_diff=flag_diff, value_diff=val_diff, cf_diff=cf_diff)), flush=True)
    return res


def d_part(g, rows_in):
    hid = [g[s["tag"]] for s in rows_in if s["kind"] == "hidden" and s["tag"] in g]
    pra = [g[s["tag"]] for s in rows_in if s["kind"] == "practice" and s["tag"] in g]
    if len(hid) != 10 or len(pra) != 3:
        return dict(ok=None, note="needs all 13 sites")

    def cnt(rows, f):
        out = {}
        for r in rows:
            out[str(f(r))] = out.get(str(f(r)), 0) + 1
        return out
    third = cnt(hid, lambda r: r["third"])
    corner = cnt(hid, lambda r: r["strata"]["corner"])
    rank = cnt(hid, lambda r: r["strata"]["near_rank"])
    tidx = cnt(hid, lambda r: r["true_index"])
    roof = cnt(hid, lambda r: r["roof"])
    p_third = cnt(pra, lambda r: r["third"])
    p_corner = cnt(pra, lambda r: r["strata"]["corner"])
    p_rank = cnt(pra, lambda r: r["strata"]["near_rank"])
    seeds_h = {r["seed"] for r in hid}
    seeds_p = {r["seed"] for r in pra}
    hashes = {}
    for s in rows_in:
        raw, mem = members(s["path"])
        hashes[s["tag"]] = (sha(raw), sha(mem["site.json"]), sha(mem["site.xml"]))
    hh = [hashes[r["tag"]] for r in hid]
    ph = [hashes[r["tag"]] for r in pra]
    disjoint = not (seeds_h & seeds_p) and not any(set(a) & set(b) for a in hh for b in ph)
    distinct_xml = len({h[2] for h in hashes.values()}) == 13
    out = dict(third=third, corner=corner, near_rank=rank, true_index=tidx, roof=roof, practice_third=p_third,
               practice_corner=p_corner, practice_rank=p_rank, disjoint=disjoint, distinct_geometries=distinct_xml)
    cls = {k.split("_")[-1]: v for k, v in third.items()}
    out["ok"] = cls == {"T": 3, "XN": 3, "XL": 4} and corner == {"E": 5, "W": 5} and \
        all(v <= 4 for v in rank.values()) and all(tidx.get(k, 0) >= 2 for k in ("0", "1", "2")) and \
        all(v >= 4 for v in roof.values()) and len(roof) == 2 and len(p_third) == 3 and len(p_corner) == 2 and \
        len(p_rank) == 3 and disjoint and distinct_xml
    return out


def _at(log, ts, t):
    k = bisect.bisect_right(ts, t + 1e-9) - 1
    return log[max(k, 0)]


def r8_offsets(rec, site):
    """per rubble box: contact samples, and the centre's offset from the gap's centreline at every primitive boundary
    inside the box's stretch of its run (0.15 m margin), from the record's 10 Hz log and the executor's history."""
    runs = site["void"]["runs"]
    log = rec.get("log10") or []
    ts = [float(s[0]) for s in log]
    bounds = []
    if rec.get("rows"):                 # a run of the pinned reference: its own rows (the true pose at each boundary)
        for rw in rec["rows"]:
            bounds += [(float(rw["start"][0]), float(rw["start"][1])), (float(rw["actual"][0]), float(rw["actual"][1]))]
    for h in ([] if rec.get("rows") else rec.get("executor_history") or []):
        if h.get("kind") != "primitive":
            continue
        for t in (float(h["t_start"]), float(h["t_end"])):
            s = _at(log, ts, t)
            bounds.append((float(s[1]), float(s[2])))
    mon = ((rec.get("reference") or {}).get("monitor") or {}).get("contact_box_samples") or {}
    out = []
    for b in site["boxes"]:
        if not b["name"].startswith(DEBRIS):
            continue
        x0, x1, y0, y1 = b["aabb"][:4]
        run = [k for k, r in runs.items() if x0 >= r["x0"] - 1e-6 and x1 <= r["x1"] + 1e-6 and y0 >= r["y0"] - 1e-6
               and y1 <= r["y1"] + 1e-6][0]
        r = runs[run]
        if r["axis"] == "x":
            lo, hi = x0, x1
            at_lo = abs(y0 - r["y0"]) <= 1e-6
            gc = 0.5 * (y1 + r["y1"]) if at_lo else 0.5 * (r["y0"] + y0)
            offs = [abs(p[1] - gc) for p in bounds if r["y0"] <= p[1] <= r["y1"] and lo - 0.15 <= p[0] <= hi + 0.15]
        else:
            lo, hi = y0, y1
            at_lo = abs(x0 - r["x0"]) <= 1e-6
            gc = 0.5 * (x1 + r["x1"]) if at_lo else 0.5 * (r["x0"] + x0)
            offs = [abs(p[0] - gc) for p in bounds if r["x0"] <= p[0] <= r["x1"] and lo - 0.15 <= p[1] <= hi + 0.15]
        out.append(dict(name=b["name"], run=run, gap_centre=round(gc, 6), n_boundaries=len(offs),
                        off_max_m=round(max(offs), 4) if offs else None, contact_samples_10hz=int(mon.get(b["name"], 0))))
    return out


def r_from_record(rec, site, time_limit, factor=1.6):
    ref = rec.get("reference") or {}
    sc = rec.get("score") or {}
    mon = ref.get("monitor") or {}
    r8 = r8_offsets(rec, site)
    deb_contacts = sum(v for k, v in (mon.get("contact_box_samples") or {}).items() if k.startswith(DEBRIS))
    v = dict(S=sc.get("S"), c=sc.get("c"), F=sc.get("F"), stuck_events=ref.get("stuck_events"), aborted=ref.get("aborted"),
             sim_time_s=rec.get("sim_time_s"), n_prims=ref.get("n_prims"), end_triangles=ref.get("end_triangles"),
             n_blind_in=ref.get("n_blind_in"), rod_pen_max_mm=mon.get("rod_pen_max_mm"),
             debris_contact_samples_10hz=deb_contacts, rubble=r8, final_state_sha256=rec.get("final_state_sha256"))
    R = dict(R1=v["c"] == {"1": 1.0, "2": 1.0, "3": 1.0} and v["F"] == 1.0 and v["stuck_events"] == 0 and not v["aborted"],
             R2=float(v["sim_time_s"]) <= time_limit / factor + 1e-9,
             R3=R3_WINDOW[0] <= int(v["n_prims"]) <= R3_WINDOW[1],
             R4=int(v["end_triangles"]) == 0, R5=int(v["n_blind_in"]) >= 16,
             R6=float(v["rod_pen_max_mm"]) <= R6_MM,
             R8=deb_contacts == 0 and all(b["contact_samples_10hz"] == 0 and b["off_max_m"] is not None and
                                          b["off_max_m"] <= R8_OFF for b in r8))
    return R, v


def r_part(rows_in, sel, oracle_summaries, practice_runs, only_tags=None):
    lim = json.load(open(os.path.join(TESTS, "hidden", "limits.json")))
    tl = float(lim["time_limit_s"])
    cands = {c["seed"]: c for c in sel["candidates"]}
    recs = {}
    for p in oracle_summaries:
        s = json.load(open(p))
        eps = json.load(open(os.path.join(s["stage"], "out", "episodes.json")))
        seeds = json.load(open(os.path.join(s["stage"], "hidden", "seeds.json")))
        for rec, pin in zip(eps, seeds["sites"]):
            recs.setdefault(pin["tag"], []).append((s["tag"], rec))
    for spec in practice_runs:
        tag, path = spec.split("=", 1)
        d = json.load(open(path))
        rec = dict(d["record"], reference=d.get("summary"), score=d.get("score"), rows=d.get("rows"),
                   sim_time_s=d["record"]["sim_time_s"]) if "record" in d else d
        recs.setdefault(tag, []).append((os.path.basename(path), rec))
    out = {}
    for s in rows_in:
        if only_tags and s["tag"] not in only_tags:
            continue
        raw, mem = members(s["path"])
        site = json.loads(mem["site.json"].decode("utf-8"))
        runs = recs.get(s["tag"], [])
        per = []
        for name, rec in runs:
            R, v = r_from_record(rec, site, tl)
            per.append(dict(run=name, R=R, values=v))
        hashes = [p["values"]["final_state_sha256"] for p in per]
        R7 = len(hashes) >= 1 and all(h == hashes[0] for h in hashes) and hashes[0] == s["pinned_final"]
        c = cands[s["seed"]]
        sel_R = c.get("R") or {}
        allR = {k: all(p["R"][k] for p in per) for k in ("R1", "R2", "R3", "R4", "R5", "R6", "R8")} if per else {}
        allR["R7"] = R7
        sv = c.get("R_values") or {}
        cmp_vals = dict(n_prims=[per[0]["values"]["n_prims"] if per else None, sv.get("n_prims")],
                        sim_time_s=[per[0]["values"]["sim_time_s"] if per else None, sv.get("sim_time_s")],
                        n_blind_in=[per[0]["values"]["n_blind_in"] if per else None, sv.get("n_blind_in")],
                        final_state_sha256=[hashes[0] if hashes else None, sv.get("final_state_sha256")])
        same_as_selection = all(close(a, b, 1e-6) for a, b in cmp_vals.values()) and \
            all(sel_R.get(k) == v for k, v in allR.items())
        row = dict(tag=s["tag"], seed=s["seed"], n_runs=len(per), runs=per, R=allR, R_selection=sel_R,
                   compare_with_selection=cmp_vals, same_as_selection=same_as_selection)
        row["ok"] = bool(per) and all(allR.values()) and same_as_selection
        out[s["tag"]] = row
        print(json.dumps(dict(tag=s["tag"], n_runs=len(per), R=allR, same_as_selection=same_as_selection,
                              off=[b["off_max_m"] for b in per[0]["values"]["rubble"]] if per else None)), flush=True)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--only", choices=("g", "r"), default=None)
    ap.add_argument("--sites", default=None)
    ap.add_argument("--oracle-summary", action="append", default=[])
    ap.add_argument("--practice-run", action="append", default=[])
    a = ap.parse_args()
    rows_in, sel = frozen_sets()
    only = a.sites.split(",") if a.sites else None
    out = {"package": "QC Q6 (version 1.1)", "sites": [s["tag"] for s in rows_in if not only or s["tag"] in only]}
    if a.only in (None, "g"):
        g = g_part(rows_in, sel, only)
        out["G"] = g
        out["D"] = d_part(g, rows_in)
        out["G_ok"] = all(r["ok"] for r in g.values())
        out["counterfactual_ok"] = all(r["CF"]["ok"] for r in g.values())
        out["counterfactual_sites_ok"] = sum(1 for r in g.values() if r["CF"]["ok"])
    if a.only in (None, "r"):
        r = r_part(rows_in, sel, a.oracle_summary, a.practice_run, only)
        out["R"] = r
        out["R_ok"] = all(x["ok"] for x in r.values())
    with open(a.out, "w") as fh:
        json.dump(out, fh, indent=1, default=str)
    print(json.dumps({k: out.get(k) for k in ("G_ok", "counterfactual_ok", "counterfactual_sites_ok", "R_ok")}))
    print(json.dumps((out.get("D") or {})))
    return 0


if __name__ == "__main__":
    sys.exit(main())
