"""The static grader rd04 (the bands and weights of grader_config.json):

    out = score(files, hidden)            # {score, M, D, families, terms, postures, counts, metrics}
    out = score(load(sub_dir), hidden)    # a GateFail scores 0 with its reason codes

M on the hidden bus samples (the files' angles = their table and ankle map on the readings; FK on the files'
compiled model) and D on the ten postures planned by the grader on the files (planner.py) and set down by the true
unit (truth.py, topple.py). score = gate x M x D."""
from __future__ import annotations

import json
import math
import os

import numpy as np

from .files import Files, GateFail
from .grid import InverseFail
from .model import ARM_LINKS, SIDES, SLOT_JOINT
from .planner import PlanFail, plan
from .topple import feet_frame_coords, topple
from .truth import Hidden, TrueSideFail
from .util import DEG, angle_between, band, rotvec

HERE = os.path.dirname(os.path.abspath(__file__))
M_TERMS = ("feet_pos_mm", "feet_normal_deg", "feet_heading_deg", "trunk_deg", "com_mm", "sole_plane_mm", "sole_fp_mm",
           "arms_deg")
D_TERMS = ("flat_deg", "weight_mm", "upright_deg")


def load_config(path=None) -> dict:
    p = path or os.path.join(HERE, "data", "grader_config.json")
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def heading_err(n, xt, xp):
    """deg: the rotation about the unit normal n between xt and xp, both projected on the plane normal to n"""
    xt = xt - (xt @ n) * n
    xp = xp - (xp @ n) * n
    return abs(math.atan2(np.cross(xt, xp) @ n, xt @ xp)) / DEG


def sole_errors(Ft: Files, Ff: Files):
    """per foot (plane mm, footprint mm) of the files' sole against the true one, in the foot link's frame"""
    pl, fp = [], []
    for s in SIDES:
        t, f = Ft.sole[s], Ff.sole[s]
        den = float(t.n @ f.n)
        d = float((f.c - t.c) @ f.n) / den if abs(den) > 1e-12 else float("inf")   # planes at a right angle
        pl.append(1000.0 * abs(d))
        D = np.linalg.norm(f.corners[:, None, :] - t.corners[None, :, :], axis=2)
        fp.append(1000.0 * float(max(D.min(axis=1).max(), D.min(axis=0).max())))
    return pl, fp


def raw_errors(F: Files, H: Hidden):
    """every per-sample error of M and the per-posture factors of D (raw, before the bands); a non-finite value
    anywhere becomes a non-finite error (it scores 0)"""
    with np.errstate(all="ignore"):
        return _raw_errors(F, H)


def _raw_errors(F: Files, H: Hidden):
    k = F.kin
    nan = float("nan")
    pos_e, nrm_e, head_e, trk_e, arm_e = [], [], [], [], []
    for i in range(H.readings.shape[0]):
        thp = F.angles(H.readings[i])
        if not np.all(np.isfinite(thp)):
            pos_e += [nan, nan]; nrm_e += [nan, nan]; head_e += [nan, nan]; trk_e.append(nan); arm_e += [nan, nan]
            continue
        k.fk(thp)
        for s, (a, b) in ARM_LINKS.items():
            Rrel = k.xmat(k.bid[a]).T @ k.xmat(k.bid[b])
            arm_e.append(abs(float(np.linalg.norm(rotvec(Rrel.T @ H.t_arms[i][s])))) / DEG)
        for s in SIDES:
            fb = k.foot[s]
            xp, xm = k.xpos(fb), k.xmat(fb)
            pt, Rt, nt = H.t_feet[i][s]
            pp = xp + xm @ H.truth.sole[s].c
            pos_e.append(1000.0 * float(np.linalg.norm(pp - pt)))
            nrm_e.append(angle_between(xm[:, 2], Rt[:, 2]) / DEG)
            head_e.append(heading_err(nt, Rt[:, 0], xm[:, 0]))
        _, sm = k.site(F.imu_site)
        gtp = k.xmat(k.torso).T @ (sm @ H.g_imu[i])
        trk_e.append(angle_between(gtp, H.t_g_torso[i]) / DEG)
    com_e = []
    for kk in range(H.plan_readings.shape[0]):
        thp = F.angles(H.plan_readings[kk])
        if not np.all(np.isfinite(thp)):
            com_e.append(nan)
            continue
        Rp, tp = H.plan_pose[kk][:3, :3], H.plan_pose[kk][:3, 3]
        k.fk(thp, Rp, tp)
        com = k.com()
        cen = {s: k.xpos(k.foot[s]) + k.xmat(k.foot[s]) @ F.sole[s].c for s in SIDES}
        xf = 0.5 * (k.xmat(k.foot["left"])[:, 0] + k.xmat(k.foot["right"])[:, 0])
        com_e.append(1000.0 * float(np.linalg.norm(feet_frame_coords(com, cen, xf) - H.t_plan[kk])))
    pl_e, fp_e = sole_errors(H.truth, F)
    rows = [_posture_safe(F, H, sp) for sp in H.postures["postures"]]
    return {"feet_pos_mm": pos_e, "feet_normal_deg": nrm_e, "feet_heading_deg": head_e, "trunk_deg": trk_e,
            "com_mm": com_e, "sole_plane_mm": pl_e, "sole_fp_mm": fp_e, "arms_deg": arm_e, "postures": rows}


NUMERIC_ERRORS = (ValueError, ArithmeticError, FloatingPointError, np.linalg.LinAlgError, IndexError)


def _posture_safe(F: Files, H: Hidden, sp: dict) -> dict:
    """_posture; a numeric error raised on the way (no submission should reach one past the gate) fails the posture
    (it scores 0, code 'numeric') instead of crashing the grader"""
    try:
        with np.errstate(all="ignore"):
            return _posture(F, H, sp)
    except NUMERIC_ERRORS as e:
        return {"posture": sp["name"], "flat_deg": float("nan"), "weight_mm": float("nan"), "upright_deg": float("nan"),
                "ok": False, "fail": "numeric", "box_clamps": 0, "stop_clamps": 0, "region_clamps": 0,
                "rest_rot_deg": None, "why": f"{type(e).__name__}"}


def _posture(F: Files, H: Hidden, sp: dict) -> dict:
    T = H.truth; kt = T.kin; kf = F.kin
    row = {"posture": sp["name"], "flat_deg": float("nan"), "weight_mm": float("nan"), "upright_deg": float("nan"),
           "ok": False, "fail": None, "box_clamps": 0, "stop_clamps": 0, "region_clamps": 0, "rest_rot_deg": None,
           "why": None}
    try:
        pl = plan(F, sp, H.postures["frame"])
    except PlanFail as e:
        row["fail"] = e.code
        return row
    try:
        r, ncl_box = F.readings(pl.th)
    except InverseFail as e:
        row["fail"] = "inverse_" + e.code
        return row
    try:
        tht, ncl_stop, ncl_reg = H.true.angles(r)
    except TrueSideFail as e:
        row["fail"] = e.code
        row["box_clamps"] = ncl_box
        return row
    row["box_clamps"], row["stop_clamps"], row["region_clamps"] = ncl_box, ncl_stop, ncl_reg
    kt.fk(tht, pl.Rp, pl.tp)
    tilt, pts, cen = [], [], {}
    for s in SIDES:
        fb = kt.foot[s]
        xp, xm = kt.xpos(fb), kt.xmat(fb)
        tilt.append(angle_between(xm @ T.sole[s].n, [0, 0, 1]) / DEG)
        pts.append(xp + T.sole[s].corners @ xm.T)
        cen[s] = xp + xm @ T.sole[s].c
    row["flat_deg"] = max(tilt)
    pts = np.vstack(pts)
    com_t = kt.com()
    xft = 0.5 * (kt.xmat(kt.foot["left"])[:, 0] + kt.xmat(kt.foot["right"])[:, 0])
    torso_t = kt.xmat(kt.torso)[:, 2].copy()
    Rr, tr, ok, info = topple(pts, com_t)
    kf.fk(pl.th, pl.Rp, pl.tp)
    com_p = kf.com()
    xfp = 0.5 * (pl.foot_x["left"] + pl.foot_x["right"])
    pred = feet_frame_coords(com_p, pl.centres, xfp)
    row["rest_rot_deg"] = info.get("rot_deg"); row["why"] = info.get("why")
    if ok:
        com_r = Rr @ com_t + tr
        cen_r = {s: Rr @ cen[s] + tr for s in SIDES}
        true_c = feet_frame_coords(com_r, cen_r, Rr @ xft)
        row["weight_mm"] = 1000.0 * float(np.linalg.norm(true_c - pred))
        row["upright_deg"] = angle_between(Rr @ torso_t, pl.torso_z) / DEG
        row["ok"] = True
    else:
        row["fail"] = "fall"
    return row


def combine(raw: dict, cfg: dict) -> dict:
    """terms, families, M, D and the per-posture factors from raw errors under the config's bands and weights"""
    B = cfg["bands"]
    t = {name: float(band(np.array(raw[name], float), *B[name]).mean()) for name in M_TERMS}
    fam = {f: float(np.mean([t[x] for x in terms])) for f, terms in cfg["families"].items()}
    W = cfg["weights_M"]
    M = float(sum(W[f] * fam[f] for f in W))
    Dk, sf_, sw_, su_ = [], [], [], []
    for r in raw["postures"]:
        sf = float(band(r["flat_deg"], *B["flat_deg"]))
        if r["ok"]:
            sw = float(band(r["weight_mm"], *B["weight_mm"])); su = float(band(r["upright_deg"], *B["upright_deg"]))
        else:
            sw = su = 0.0
            if r["fail"] != "fall":
                sf = 0.0
        Dk.append(sf * sw * su); sf_.append(sf); sw_.append(sw); su_.append(su)
    D = float(np.mean(Dk))
    t["flat"] = float(np.mean(sf_)); t["weight"] = float(np.mean(sw_)); t["upright"] = float(np.mean(su_))
    return {"score": M * D, "M": M, "D": D, "families": fam, "terms": t, "posture_factors": Dk,
            "posture_flat": sf_, "posture_weight": sw_, "posture_upright": su_}


def score(files, hidden: Hidden, config=None, keep_raw=False) -> dict:
    """{score, M, D, families, terms, postures, counts, gate, metrics}; a GateFail (returned by load) scores 0.
    keep_raw: also return the raw per-sample errors under "_raw" (numeric; for maintainers' tools, never published)"""
    from .metrics import metrics_of
    cfg = config if isinstance(config, dict) else load_config(config)
    if isinstance(files, GateFail):
        out = {"score": 0.0, "M": 0.0, "D": 0.0, "families": {}, "terms": {}, "postures": [], "counts": {},
               "gate": {"pass": False, "codes": files.codes}}
        out["metrics"] = metrics_of(out, None, hidden, cfg)
        return out
    raw = raw_errors(files, hidden)
    c = combine(raw, cfg)
    counts = {"falls": sum(1 for r in raw["postures"] if r["fail"] == "fall"),
              "posture_failures": sum(1 for r in raw["postures"] if r["fail"] not in (None, "fall")),
              "box_clamps": int(sum(r["box_clamps"] for r in raw["postures"])),
              "stop_clamps": int(sum(r["stop_clamps"] for r in raw["postures"])),
              "region_clamps": int(sum(r["region_clamps"] for r in raw["postures"])),
              "nonfinite_samples": int(sum(1 for v in raw["trunk_deg"] if not math.isfinite(v)))}
    out = {"score": c["score"], "M": c["M"], "D": c["D"], "families": c["families"], "terms": c["terms"],
           "postures": [{"posture": r["posture"], "factor": f, "flat": a, "weight": b, "upright": u, "fail": r["fail"]}
                        for r, f, a, b, u in zip(raw["postures"], c["posture_factors"], c["posture_flat"],
                                                 c["posture_weight"], c["posture_upright"])],
           "counts": counts, "gate": {"pass": True, "codes": []}}
    out["metrics"] = metrics_of(out, (files, raw), hidden, cfg)
    if keep_raw:
        out["_raw"] = raw
    return out


def load_hidden(hidden_dir, gt_dir) -> Hidden:
    """the hidden set as the task folder holds it: <hidden_dir>/hidden_set.npz and postures.json;
    <gt_dir>/unit.xml, calibration.yaml (the truth files, read light by load_truth) and true_unit.json
    (rd04-true-unit/1, written from the record)"""
    from .files import load_truth
    T = load_truth(gt_dir)
    return Hidden.from_files(os.path.join(hidden_dir, "hidden_set.npz"), os.path.join(hidden_dir, "postures.json"),
                             os.path.join(gt_dir, "true_unit.json"), T)


def grade_submission(sub_dir, hidden_dir, gt_dir, config=None, late_entries=None, given=None) -> dict:
    """one submission folder graded end to end: {score, M, D, families, terms, postures, counts, gate, metrics} (no
    raw errors). The published part is out["metrics"] only."""
    from .files import load
    H = load_hidden(hidden_dir, gt_dir)
    F = load(sub_dir, given=given, late_entries=late_entries)
    return score(F, H, config=config)
