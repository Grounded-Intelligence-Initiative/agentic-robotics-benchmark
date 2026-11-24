"""reference_check.py: run the reference calibration on fresh units 1..N and grade it like the grader.

    python reference_check.py [N] [--jobs J] [--first F]

For each unit: the defect classes the unit truly has (from the truth: read here only, never by
reference.py), accepted or not (tool tip AND camera), p95 and max tool-tip and camera errors (mm)
on the unit's acceptance conditions, probes, images and virtual time used, damage, the spheres
set aside against the truly knocked ones, the classes the reference selected, the accuracy of the
reference's sphere measurements against the truth (lateral, px, and depth, mm: with the
reference's own intrinsics and, in brackets, re-fitted with the true ones), the camera test in
the image-centre frame (a diagnostic: the frame whose z axis is the ray imaged at the image
centre, which removes the principal point / mounting rotation ambiguity), real time. The same
session's data are also fitted by baselines:
  legacy    the first-order family only (the 84 parameters of arm_model, no defect terms), fitted
            jointly to the touches and the images with the same robust handling (false triggers,
            knocked spheres) and the same image measurements: the approach that solved the
            previous version of the task;
  rerender  the version-1 vision (baseline_rerender.py: each sphere measured by re-rendering it with
            the image model version 1 published: one light at the camera, matte spheres, exact
            pinhole without distortion) on the new images, then the reference's kinematics, joint
            fit and model selection: what the LLM agent did on version 1;
  textbook  the reference's probe-only fit for the arm, the camera's 6 parameters fitted to sphere
            measurements made the textbook way (threshold at half maximum, the blob's centroid as
            the direction of the centre, its distance from the blob's area; nominal intrinsics);
  full      the reference's data and joint fit with every candidate defect under its prior, no
            selection ('fit all candidates');
  map       the single most probable model of the selection (no averaging).
The job (the gauge plate's bores, process_c's pin; calib_cell.run_job): the reference's program run on
the true unit as the grader runs it (bores done, jam), the plate's place as the reference located it
against the truth (largest lateral error of a bore's mouth, mm; tilt error, mrad), and the radial
clearance the program needs: the bore radius less the largest pin radius with which it still does every
bore (bisection of calib_cell.run_job's pin radius: the same program and the same random stream), with
which the pass rate at any clearance follows. The same for job baselines, each planned by the
reference's planner from the same session:
  nomcam    the plate located with the documented nominal camera (mounting, intrinsics), the calibrated arm;
  oneimg    the plate located from one image (the first of the plate's views) instead of all;
  notilt    the plate located without its tilt (held level: x, y, height, yaw only);
  legacy    plate and job with the first-order model without defect classes (the 'legacy' fit above);
  noback    the plate as the reference located it, the program planned with the backlash left out (b = 0:
            the direction signs a row sets ignored).
The warm job (calib_cell.run_job(warm=True): from the unit's published warm start temperatures): the
reference's warm program, its required clearance and pass; the session's thermal fit against the truth
(cooling time constant, own-joint heating); and warm baselines, each on the plate as the reference located it:
  wcold     the reference's cold program submitted as the warm one (temperature ignored);
  wfirst    planned with the first-order thermal model only: the reference's most probable model refitted
            without the thermal defect class (kappa only), the documented nominal heating and cooling;
  wnomcool  the reference's model, the documented nominal heating and cooling instead of the session's fit;
and, with the true kinematics (the hidden unit, a planning study only), the displacement of the pin at the
bores between the cold and the warm state for the same joint targets (`disp`, the cold program's deepest
arrival in each bore, the same repeatability draws).
The service (calib_cell.serviced) and the re-qualification session (requal.py, on the serviced unit, its session
"service" 1 as the lab numbers it): the joint the reference identified against the truth, its zero offset and
backlash errors, the session's damage (a collision), time and probes used; the serviced job's required clearance
and pass on the serviced unit; and serviced-job baselines, each planned by the reference's planner on the plate as
the reference located it:
  snone     no update: the reference's cold program submitted as the serviced one;
  szero     the zero offset updated, the backlash not (the measured model's backlash kept);
  swrong    the best fit of a wrong joint (the joint with the next lowest chi-square);
  sone      the update from a single probe (the first touch only, the joint told: the most favourable case);
  sknown    the joint told, the same touches (the variant with the serviced joint stated).
With --jobs J, units run in J processes (the real times are then not representative); --out F appends one
JSON line per unit to F; --lab runs the lab's own units (hidden/units.json: p1, then u1 to u6, each in its
session 1) instead of fresh ones (p1 too goes through a service, a re-qualification session and a serviced job
here: a study of one more unit only, since the lab never services the practice unit); --quick leaves out the
prediction baselines and the cold job baselines (the reference, its jobs and the warm baselines only).
"""
import json
import os
import sys
import time

# Maintainers' tooling, run from a checkout (nothing runs it during grading): the cell is beside this folder,
# the reference in solution/.
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [_HERE, os.path.dirname(_HERE), os.path.join(_HERE, "..", "..", "solution", "calibrate")]

import numpy as np
from scipy import ndimage

import arm_model as am
import baseline_rerender as v1vision
import calib_cell as cc
import reference as ref
import requal

ABBR = {"coupling": "C", "ratio": "R", "deflection": "D", "cyclic": "H", "thermal": "T", "pedestal": "P"}


def grade(pred, unit):
    tip, cam = cc.acceptance_errors_from_predictions(pred[0], pred[1], unit)
    return (cc.accepted(tip, cam), np.percentile(tip, 95) * 1e3, tip.max() * 1e3,
            np.percentile(cam, 95) * 1e3, cam.max() * 1e3)


def first_order_prior(geometry_only=False):
    """Mean and sd of the model without any defect term (the published family); with
    geometry_only, also b, h, c, kappa held at zero."""
    mu, sd, _ = ref.model_prior(())
    if geometry_only:
        for key in ("b", "h", "c", "kappa"):
            mu[ref.SL[key]] = 0.0
            sd[ref.SL[key]] = 0.0
    return mu, sd


def baseline_family(info, cond, geometry_only=False):
    """The touches (robust handling refitted with this family) and the reference's image
    measurements, fitted jointly with a family without defect terms."""
    mu, sd = first_order_prior(geometry_only)
    good = [d for d in info["data"] if d["ok"]]
    ref.SIGMA = info["sigma"]
    p, rinfo = ref.robust_calibration(good, mu, sd, p0=mu.copy())
    obs = info.get("camera_obs") or []
    p_cam = p.copy()
    if obs:
        p_cam, _, _ = ref.fit_camera(p, obs, rinfo["centres"])
        p_cam, _ = ref.joint_calibration(rinfo["keep"], obs, p_cam, mu, sd, rinfo["free_spheres"], rinfo["centres"])
    return ref.predict(p_cam, cond), p_cam


# ---- the job --------------------------------------------------------------------------------------
JOB_BASELINES = ("nomcam", "oneimg", "notilt", "legacy", "noback")


def required_clearance(unit, rows, warm=False):
    """The radial clearance (m) the program needs on the true unit: the bore radius less the largest pin
    radius with which it does every bore (inf when it fails even with a 1.5 mm pin, or there is none)."""
    if rows is None:
        return np.inf
    rows = np.asarray(rows, float)
    lo, hi = 1.5e-3, cc.BORE_RADIUS
    if not cc.job_passed(cc.run_job(unit, rows, radius=lo, warm=warm)):
        return np.inf
    for _ in range(11):
        mid = 0.5 * (lo + hi)
        if cc.job_passed(cc.run_job(unit, rows, radius=mid, warm=warm)):
            lo = mid
        else:
            hi = mid
    return cc.BORE_RADIUS - lo


def insertion_errors(unit, rows, warm=False):
    """At each bore's deepest arrival (a pin of 1.5 mm, so that nothing jams): the apex's distance from the
    true bore axis (m) and the angle between the pin's axis and the bore's (rad)."""
    res = cc.run_job(unit, np.asarray(rows, float), radius=1.5e-3, trace=True, warm=warm)
    pl = unit["plate"]
    best = {}
    for a in res["arrivals"]:
        E, B, apex = cc.pin_axis(unit, a["q"][None], a["s"], a["temp"], a["offset"])
        p = (apex[0] - pl["O"]) @ pl["R"]
        r = np.hypot(p[0] - cc.BORES[:, 0], p[1] - cc.BORES[:, 1])
        b = int(np.argmin(r))
        if r[b] < cc.BORE_RADIUS and p[2] < 0 and (b not in best or p[2] < best[b][0]):
            ax = (E[0] - B[0]) / np.linalg.norm(E[0] - B[0])
            best[b] = (p[2], r[b], float(np.arccos(np.clip(-ax @ pl["R"][:, 2], -1.0, 1.0))))
    return [(v[1], v[2]) for v in best.values()]


def plate_error(unit, pose):
    """Largest lateral error of a bore mouth's place (m) and the error of the plate's normal (rad)."""
    if pose is None:
        return np.inf, np.inf
    import plate_job as pj
    m_e, a_e = pj.mouths(pose)
    m_t, a_t = pj.mouths(np.array(unit["plate"]["pose"]))
    d = m_e - m_t
    return (float(np.linalg.norm(d - (d @ a_t)[:, None] * a_t, axis=1).max()),
            float(np.linalg.norm(np.cross(a_e, a_t))))


def job_evidence(unit, rows, pose, warm=False):
    out = dict(rows=None if rows is None else len(rows), creq=required_clearance(unit, rows, warm))
    out["plate_lat"], out["plate_tilt"] = plate_error(unit, pose)
    if rows is not None:
        res = cc.run_job(unit, np.asarray(rows, float), warm=warm)
        out.update(done=res["bores_done"], jammed=res["jammed"], passed=cc.job_passed(res), poses=res["poses"])
        out["ins"] = insertion_errors(unit, rows, warm) if np.isfinite(out["creq"]) else []
    else:
        out.update(done=0, jammed=False, passed=False, poses=0, ins=[])
    return out


def displacement(unit, rows):
    """For the same joint targets (the cold program), the true pin apex's displacement across each bore's axis
    between the cold run and the same run from the warm start (the same repeatability draws), at the cold
    run's deepest arrival in each bore (m); [] without a program."""
    if rows is None:
        return []
    rows = np.asarray(rows, float)
    rc = cc.run_job(unit, rows, radius=1.5e-3, trace=True)
    rw = cc.run_job(unit, rows, radius=1.5e-3, trace=True, temp0=cc.warm_start(unit["seed"]))
    pl = unit["plate"]
    a = pl["R"][:, 2]
    best = {}
    for k, ar in enumerate(rc["arrivals"]):
        _, _, apex = cc.pin_axis(unit, ar["q"][None], ar["s"], ar["temp"], ar["offset"])
        p = (apex[0] - pl["O"]) @ pl["R"]
        r = np.hypot(p[0] - cc.BORES[:, 0], p[1] - cc.BORES[:, 1])
        b = int(np.argmin(r))
        if r[b] < cc.BORE_RADIUS and p[2] < 0 and (b not in best or p[2] < best[b][0]):
            best[b] = (p[2], k)
    out = []
    for b, (_, k) in sorted(best.items()):
        if k >= len(rw["arrivals"]):
            continue
        ac, aw = rc["arrivals"][k], rw["arrivals"][k]
        pc = cc.pin_axis(unit, ac["q"][None], ac["s"], ac["temp"], np.zeros(3))[2][0]
        pw = cc.pin_axis(unit, aw["q"][None], aw["s"], aw["temp"], np.zeros(3))[2][0]
        d = pw - pc
        out.append(float(np.linalg.norm(d - (d @ a) * a)))
    return out


def first_order_thermal(info):
    """The reference's most probable model refitted without the thermal defect class (kappa only)."""
    if not info.get("models"):
        return None
    top = info["models"][0]
    classes = tuple(c for c in top["classes"] if c != "thermal")
    mu, sd, _ = ref.model_prior(classes, top["cyc"])
    ref.SIGMA = info["sigma"]
    p = ref.fit_joint(info["data"], info["joint"]["obs"], info["p_base"], mu, sd, info["free_spheres"],
                      info["joint"]["centres"])[0]
    return p


def warm_baselines(info, cond, unit):
    out = {}
    pose = info.get("plate_pose")
    warm = np.asarray(cond["job_warm_temp"], float)
    out["wcold"] = (info.get("job"), pose)
    p_first = first_order_thermal(info)
    if pose is None or p_first is None:
        out["wfirst"] = out["wnomcool"] = (None, pose)
        return out, None
    out["wfirst"] = ref.job_program(dict(info, job_log=[]), p=p_first, warm=warm, thermal=ref.NOMINAL_THERMAL,
                                    plate=pose)
    out["wnomcool"] = ref.job_program(dict(info, job_log=[]), warm=warm, thermal=ref.NOMINAL_THERMAL, plate=pose)
    return out, grade(ref.predict(p_first, cond), unit)


def thermal_accuracy(info, unit):
    th = info.get("thermal")
    if not th:
        return dict(tau=np.nan, tau_true=unit["cell"]["tau_cool"], heat_err=[np.nan] * 6, rms=np.nan)
    H = unit["cell"]["heat"]
    return dict(tau=th["tau"], tau_true=unit["cell"]["tau_cool"], rms=th["rms"],
                heat_err=(np.diag(th["heat"]) - np.diag(H)).tolist(), heat_true=np.diag(H).tolist())


def job_baselines(info, p_legacy):
    p = info["models"][0]["p"] if info.get("models") else info["p_base"]
    out = {}
    p_nc = np.array(p, float)
    p_nc[ref.CAM] = ref.camera_prior()[0]
    p_nc[ref.INTR] = ref.vision_prior()[0][:5]
    out["nomcam"] = ref.job_program(dict(info, job_log=[]), p=p_nc)
    out["oneimg"] = ref.job_program(dict(info, job_log=[]), p=p, use=[0])
    out["notilt"] = ref.job_program(dict(info, job_log=[]), p=p, free=(0, 1, 2, 3))
    out["legacy"] = ref.job_program(dict(info, job_log=[]), p=p_legacy)
    p_nb = np.array(p, float)
    p_nb[ref.SL["b"]] = 0.0
    out["noback"] = ref.job_program(dict(info, job_log=[]), p=p, plan_p=p_nb)
    return out


def baseline_rerender(info, cond):
    """The version-1 vision on the session's images, then the reference's final fit."""
    if not info["images"]:
        return ref.predict(info["p_probe"], cond)
    p_staged, obs = v1vision.measure(info["images"], info["p_probe"], info["centres"])
    pt, pc, _ = ref.final_fit(info["keep"], obs, p_staged, info["centres"], info["free_spheres"], cond, info["mu"],
                              info["sd"])
    return pt, pc


def textbook_measure(rec, p, centres):
    """Sphere centres in the camera frame the textbook way (see the module's docstring). Where
    to look: the reference's prediction (its kinematics, the nominal camera)."""
    img = rec["image"].astype(float)
    Pall = ref.spheres_in_camera(p, rec["q"], rec["s"], rec["temp"], rec["tool"], centres)[0][0]
    out = []
    for k in rec["ks"]:
        box = ref.outline_box(Pall[k])
        if box is None:
            continue
        m = 30
        i0, i1 = int(max(0, box[2] - m)), int(min(img.shape[0], box[3] + m + 1))
        j0, j1 = int(max(0, box[0] - m)), int(min(img.shape[1], box[1] + m + 1))
        win = img[i0:i1, j0:j1]
        bg, peak = np.median(win), win.max()
        lab, n = ndimage.label(win > 0.5 * (bg + peak))
        if n == 0:
            continue
        sizes = ndimage.sum(np.ones_like(win), lab, index=np.arange(1, n + 1))
        best = int(np.argmax(sizes)) + 1
        vc, uc = ndimage.center_of_mass(np.ones_like(win), lab, best)
        u, v = uc + j0, vc + i0
        r = np.sqrt(sizes[best - 1] / np.pi)
        d = ref.F_PX * ref.R_SPHERE / r
        mm = np.array([(u - ref.CX) / ref.F_PX, (v - ref.CY) / ref.F_PX, 1.0])
        out.append(dict(k=k, P=d * mm / np.linalg.norm(mm), cov=None, rms=np.nan))
    return out


def baseline_textbook(info, cond):
    p = np.array(info["p_probe"], float)
    p[ref.CAM] = ref.camera_prior()[0]
    obs = []
    for rec in info["images"]:
        r = dict(rec)
        r["spheres"] = textbook_measure(rec, p, info["centres"])
        obs.append(r)
    p, _, _ = ref.fit_camera(p, obs, info["centres"])
    return ref.predict(p, cond)


def true_intrinsics(unit):
    op = unit["camera"]["optics"]
    return np.array([op["f"], op["cx"], op["cy"], op["k1"], op["k2"]])


def vision_accuracy(info, cell, unit):
    """Errors of the reference's final sphere measurements against the truth (hidden): lateral (px)
    and along the line of sight (mm), with the reference's intrinsics and with the true ones."""
    th_true = true_intrinsics(unit)
    th_ref = info["p_base"][ref.INTR]
    f = th_true[0]
    out = {"lat": [], "dep": [], "lat_t": [], "dep_t": []}
    first = len(info.get("plate_images") or [])         # the plate's views were taken before the spheres' views
    for rec in info["joint"]["obs"]:
        Rc, oc = cell.image_truth[first + rec["n"]]
        Pt = (unit["centres"] - oc) @ Rc
        for m in rec["spheres"]:
            if "uv" not in m:
                continue
            for th, key in ((th_ref, ""), (th_true, "_t")):
                P = ref.cone_fit(m["uv"], m["sig"], th, m["P"])[0]
                T = Pt[m["k"]]
                u = T / np.linalg.norm(T)
                e = P - T
                out["lat" + key].append(np.linalg.norm(e - (e @ u) * u) / np.linalg.norm(T) * f)
                out["dep" + key].append((e @ u) * 1e3)
    return {k: np.array(v) for k, v in out.items()}


def truth_image_centre(unit):
    """The true camera points of the conditions taken in the true image-centre frame."""
    q, s, temp, tool = cc.acceptance_conditions(unit["seed"])
    Rc, oc = cc.true_camera(unit, q, s, temp, tool)
    Rd = ref.image_centre_rotation(true_intrinsics(unit))      # the same geometry, with the true intrinsics
    pts = cc.ACCEPT_CAM_POINTS @ Rd.T
    return np.einsum("nab,kb->nka", Rc, pts) + oc[:, None, :]


def service_evidence(row, unit, info):
    """The re-qualification session on the serviced unit, the serviced job and its baselines (in place in row)."""
    after = cc.serviced(unit)
    cell = cc.Cell(after, session=("service", 1), budget=cc.BUDGET_SERVICE)
    t0 = time.time()
    rows, est, log = requal.requalify(cell, info)
    row["rq_real"] = time.time() - t0
    sv = after["service"]
    j = sv["joint"]
    b_old = float(unit["params"][cc.am.SLICES["b"].start + j])
    rq = info.get("requal", {})
    row["service"] = dict(joint=j, zero=sv["zero"], backlash=sv["backlash"], b_old=b_old)
    row["rq"] = dict(joint=est["joint"] if est else -1, alive=est["alive"] if est else [], zero=est["zero"] if est else np.nan,
                     backlash=est["backlash"] if est else np.nan, damaged=cell.damaged, time=cell.time,
                     probes=cell.probes, images=cell.images, touches=len(rq.get("data", [])),
                     false=sum(1 for d in rq.get("data", []) if d["triggered"] and
                               np.linalg.norm(cc.true_tip(after, d["q"][None], d["s"][None], d["temp"][None],
                                                          cc.TOOLS["probe_a"])[0] - after["centres"][d["k"]]) - ref.R_TOUCH > 0.3e-3),
                     log=log[-3:])
    pose = info.get("plate_pose")
    row["job_serviced"] = job_evidence(after, rows, pose)
    out = {"snone": info.get("job")}
    if est is not None and pose is not None:
        p0 = np.array(info["models"][0]["p"] if info.get("models") else info["p_base"], float)
        recs = [d for d in rq["data"] if d["triggered"]]
        rqo = requal.Requal.__new__(requal.Requal)
        rqo.p0, rqo.centres = p0, np.array(info["centres"], float)
        plan = lambda p: ref.job_program(dict(info, job_log=[]), p=p, plate=pose)[0]     # noqa: E731
        out["szero"] = plan(requal.hypothesis(p0, est["joint"], est["zero"], p0[requal.back_index(est["joint"])]))
        others = {jj: f for jj, f in est["fits"].items() if jj != est["joint"]}
        if others:
            jw = min(others, key=lambda jj: others[jj][1])
            out["swrong"] = plan(requal.hypothesis(p0, jw, *others[jw][0]))
        else:                      # every other joint ruled out: its fit on all touches
            jw = [jj for jj in cc.SERVICE_JOINTS if jj != est["joint"]][0]
            out["swrong"] = plan(requal.hypothesis(p0, jw, *rqo.fit_joint(jw, recs)[0]))
        if recs:
            x1 = rqo.fit_joint(j, recs[:1])[0]
            out["sone"] = plan(requal.hypothesis(p0, j, x1[0], p0[requal.back_index(j)]))
            out["sknown"] = plan(requal.hypothesis(p0, j, *rqo.fit_joint(j, recs)[0]))
    for name in SERVICE_BASELINES:
        row["job_" + name] = job_evidence(after, out.get(name), pose)


def run_unit(seed, quick=False):
    unit = cc.draw_unit(seed)
    cell = cc.Cell(unit, session=1)      # a graded unit's one session in the lab is its session 1
    cond = cc.write_conditions(unit)
    t0 = time.time()
    pred_tip, pred_cam, info = ref.calibrate(cell, cond, return_info=True)
    real = time.time() - t0
    ref.SIGMA = info["sigma"]
    top = info["models"][0] if info.get("models") else None
    row = dict(seed=seed, real=real, probes=cell.probes, images=cell.images, time=cell.time,
               damaged=cell.damaged, free=info["free_spheres"], knocked=unit["knocked"],
               classes="".join(ABBR[c] for c in unit["defects"]["classes"]),
               chosen=("".join(ABBR[c] for c in top["classes"]) + ("%d/%d" % top["cyc"] if top["cyc"] else "")
                       if top else "?"),
               truth_cyc=("%d/%d" % (unit["defects"]["cyc_joint"], unit["defects"]["cyc_order"])
                          if "cyclic" in unit["defects"]["classes"] else ""),
               weight=top["weight"] if top else np.nan, sigma=info["sigma"] * 1e6,
               sigma_true=unit["cell"]["sigma_repeat"] * 1e6, ft=unit["cell"]["false_trigger"])
    row["ok"], row["p95"], row["max"], row["cp95"], row["cmax"] = grade((pred_tip, pred_cam), unit)
    row["timing"] = info.get("timing", {})
    if "pred_cam_ic" in info:
        e = np.linalg.norm(info["pred_cam_ic"] - truth_image_centre(unit), axis=2)
        row["ic"] = (np.percentile(e, 95) * 1e3, e.max() * 1e3)
    else:
        row["ic"] = (np.nan, np.nan)
    va = vision_accuracy(info, cell, unit)
    row["vision"] = va
    th = info["p_base"][ref.INTR]
    row["intr_err"] = th - true_intrinsics(unit)
    row["n_meas"] = len(va["lat"])
    row["map"] = grade(info["pred_map"], unit) if "pred_map" in info else grade(info["pred_base"], unit)
    if not quick:
        pt, pc, _ = ref.final_fit(info["keep"], info["camera_obs"], info["p_staged"], info["centres"],
                                  info["free_spheres"], cond, select=False)
        row["full"] = grade((pt, pc), unit)
        pred_legacy, p_legacy = baseline_family(info, cond)
        row["legacy"] = grade(pred_legacy, unit)
    t1 = time.time()
    row["job"] = job_evidence(unit, info.get("job"), info.get("plate_pose"))
    row["job_warm"] = job_evidence(unit, info.get("job_warm"), info.get("plate_pose"), warm=True)
    row["thermal"] = thermal_accuracy(info, unit)
    row["disp"] = displacement(unit, info.get("job"))
    wb, row["wfirst_pred"] = warm_baselines(info, cond, unit)
    for name, (rows, pose) in wb.items():
        row["job_" + name] = job_evidence(unit, rows, pose, warm=True)
    if not quick:
        for name, (rows, pose) in job_baselines(info, p_legacy).items():
            row["job_" + name] = job_evidence(unit, rows, pose)
    t2 = time.time()
    service_evidence(row, unit, info)
    row["serv_real"] = time.time() - t2
    row["job_real"] = time.time() - t1
    row["plate_tool"] = (info.get("plate_images") or [{}])[0].get("tool", "-")
    row["plate_views"] = len(info.get("plate_images") or [])
    if not quick:
        row["rerender"] = grade(baseline_rerender(info, cond), unit)
        row["textbook"] = grade(baseline_textbook(info, cond), unit) if info["images"] else (False,) + (np.nan,) * 4
    return row


BASELINES = ("legacy", "rerender", "textbook", "full", "map")
WARM_BASELINES = ("wcold", "wfirst", "wnomcool")
SERVICE_BASELINES = ("snone", "szero", "swrong", "sone", "sknown")


def fmt(r):
    s = ("{:>4} {:>5} {:>8} {:>4} {:>6.3f}/{:<6.3f} {:>6.3f}/{:<6.3f} {:>4} {:>3} {:>5.0f} {:>4} {:>9} {:>9} {:>4.0f}/{:<4.0f}"
         " {:>4.2f} |").format(
        r["seed"], r["classes"] + (" " + r["truth_cyc"] if r["truth_cyc"] else ""), r["chosen"],
        "yes" if r["ok"] else "NO", r["p95"], r["max"], r["cp95"], r["cmax"], r["probes"], r["images"], r["time"],
        "YES" if r["damaged"] else "no", ",".join(map(str, r["free"])) or "-", ",".join(map(str, r["knocked"])) or "-",
        r["sigma"], r["sigma_true"], r["weight"])
    va = r["vision"]
    s += " {:>3} {:.3f}/{:+.2f}({:.3f}/{:+.2f}) {:.3f}/{:.3f} |".format(
        r["n_meas"], np.sqrt(np.mean(va["lat"] ** 2)), np.mean(va["dep"]), np.sqrt(np.mean(va["lat_t"] ** 2)),
        np.mean(va["dep_t"]), r["ic"][0], r["ic"][1])
    for b in BASELINES:
        v = r.get(b)
        s += " {:>4} {:.2f}/{:.2f} |".format("pass" if v[0] else "fail", v[1], v[3]) if v else " {:^14} |".format("-")
    j = r["job"]
    s += " {} {}/8 {:>5.3f} {:>5.3f} {:>4.2f} |".format("pass" if j["passed"] else "FAIL", j["done"], j["creq"] * 1e3,
                                                        j["plate_lat"] * 1e3, j["plate_tilt"] * 1e3)
    for b in JOB_BASELINES:
        s += " {:>5.3f}".format(r["job_" + b]["creq"] * 1e3) if "job_" + b in r else "     -"
    w, th = r["job_warm"], r["thermal"]
    s += " | {} {}/8 {:>5.3f} | tau {:>4.0f}/{:<4.0f} | disp {:.3f}-{:.3f} |".format(
        "pass" if w["passed"] else "FAIL", w["done"], w["creq"] * 1e3, th["tau"], th["tau_true"],
        min(r["disp"] or [np.nan]) * 1e3, max(r["disp"] or [np.nan]) * 1e3)
    for b in WARM_BASELINES:
        s += " {:>5.3f}".format(r["job_" + b]["creq"] * 1e3)
    v, e, js = r["service"], r["rq"], r["job_serviced"]
    s += " | j{}->{} {} dz {:+.0f} db {:+.0f} urad {}{}p {:.0f}s | {} {}/8 {:>5.3f} |".format(
        v["joint"], e["joint"], "ok" if e["joint"] == v["joint"] else "WRONG", (e["zero"] - v["zero"]) * 1e6,
        (e["backlash"] - v["backlash"]) * 1e6, "DMG " if e["damaged"] else "", e["probes"], e["time"],
        "pass" if js["passed"] else "FAIL", js["done"], js["creq"] * 1e3)
    for b in SERVICE_BASELINES:
        s += " {:>5.3f}".format(r["job_" + b]["creq"] * 1e3)
    return s + " | {:>4.0f}s {:>4.0f}s".format(r["real"], r["job_real"])


def _jsonable(r):
    out = {}
    for k, v in r.items():
        if isinstance(v, dict):
            out[k] = _jsonable(v)
        elif isinstance(v, (np.ndarray, list, tuple)):
            out[k] = [(_jsonable({"x": x})["x"]) for x in v]
        elif isinstance(v, (np.floating, float)):
            out[k] = float(v) if np.isfinite(v) else (1e9 if v > 0 else -1e9)
        elif isinstance(v, (np.integer, int, bool, np.bool_)):
            out[k] = v.item() if hasattr(v, "item") else v
        else:
            out[k] = v if isinstance(v, (str, type(None))) else str(v)
    return out


def main():
    args = sys.argv[1:]
    jobs, first, out_path, explicit = 1, 1, None, None
    quick = "--quick" in args
    if quick:
        args.remove("--quick")
    import functools
    one = functools.partial(run_unit, quick=quick)
    if "--out" in args:
        i = args.index("--out")
        out_path = args[i + 1]
        args = args[:i] + args[i + 2:]
    if "--lab" in args:          # the lab's own units (hidden/units.json): what `ale validate` runs
        args.remove("--lab")
        with open(os.path.join(os.path.dirname(_HERE), "hidden", "units.json")) as handle:
            lab = json.load(handle)
        explicit = list(lab["practice"].values()) + [lab["graded"][k] for k in sorted(lab["graded"])]
    for flag in ("--jobs", "--first"):
        if flag in args:
            i = args.index(flag)
            val = int(args[i + 1])
            args = args[:i] + args[i + 2:]
            if flag == "--jobs":
                jobs = val
            else:
                first = val
    sink = open(out_path, "a") if out_path else None
    n_units = int(args[0]) if args else 10
    head = ("{:>4} {:>5} {:>8} {:>4} {:>13} {:>13} {:>4} {:>3} {:>5} {:>4} {:>9} {:>9} {:>9} {:>4} |").format(
        "unit", "truth", "chosen", "acc", "tip p95/max", "cam p95/max", "prb", "img", "time", "dmg", "set-aside",
        "knocked", "sig um", "w")
    head += " {:^34} |".format("meas lat px/dep mm (true th), ic cam")
    for b in BASELINES:
        head += " {:^14} |".format(b + " tip/cam")
    head += " {:^29} |".format("job done creq plat mm tilt mrad")
    head += " " + " ".join("{:>5}".format(b[:5]) for b in JOB_BASELINES)
    head += " | {:^16} | {:^13} | {:^15} |".format("warm job creq", "tau est/true", "disp cold-warm")
    head += " " + " ".join("{:>5}".format(b[:5]) for b in WARM_BASELINES)
    head += " | {:^37} | {:^18} |".format("serviced joint, zero, backlash errors", "serviced job creq")
    head += " " + " ".join("{:>5}".format(b[:5]) for b in SERVICE_BASELINES) + " |  real  job"
    print(head)
    print("-" * len(head))
    seeds = explicit if explicit is not None else list(range(first, first + n_units))
    rows = []
    if jobs > 1:
        import multiprocessing as mp
        with mp.get_context("spawn").Pool(jobs) as pool:
            for r in pool.imap(one, seeds):
                rows.append(r)
                print(fmt(r), flush=True)
                if sink:
                    sink.write(json.dumps(_jsonable(r)) + "\n")
                    sink.flush()
    else:
        for seed in seeds:
            r = one(seed)
            rows.append(r)
            print(fmt(r), flush=True)
            if sink:
                sink.write(json.dumps(_jsonable(r)) + "\n")
                sink.flush()
    print("-" * len(head))
    acc = [r for r in rows if r["ok"]]
    a = np.array([[r["p95"], r["max"], r["cp95"], r["cmax"], r["real"], r["time"]] for r in rows])
    print("reference: {}/{} accepted; damaged {}; spheres right {}/{}; top model = true classes {}/{}".format(
        len(acc), len(rows), sum(r["damaged"] for r in rows), sum(r["free"] == r["knocked"] for r in rows), len(rows),
        sum(set(r["chosen"].split("0")[0].split("1")[0].split("2")[0]) == set(r["classes"]) for r in rows), len(rows)))
    if acc:
        b = np.array([[r["p95"], r["max"], r["cp95"], r["cmax"]] for r in acc])
        print("  worst over accepted units: tip p95 {:.3f} mm (margin {:.2f}x), tip max {:.3f} ({:.2f}x), camera p95 {:.3f} "
              "({:.2f}x), camera max {:.3f} ({:.2f}x)".format(
                  b[:, 0].max(), cc.ACCEPT_P95 * 1e3 / b[:, 0].max(), b[:, 1].max(), cc.ACCEPT_MAX * 1e3 / b[:, 1].max(),
                  b[:, 2].max(), cc.ACCEPT_CAM_P95 * 1e3 / b[:, 2].max(), b[:, 3].max(),
                  cc.ACCEPT_CAM_MAX * 1e3 / b[:, 3].max()))
    margin = [min(cc.ACCEPT_P95 * 1e3 / r["p95"], cc.ACCEPT_MAX * 1e3 / r["max"], cc.ACCEPT_CAM_P95 * 1e3 / r["cp95"],
                  cc.ACCEPT_CAM_MAX * 1e3 / r["cmax"]) for r in rows]
    print("  units with at least 1.5x margin on every limit: {}/{}".format(sum(m >= 1.5 for m in margin), len(rows)))
    for name in BASELINES:
        if not all(name in r for r in rows):
            continue
        v = np.array([[r[name][1], r[name][3]] for r in rows], float)
        print("{:>9}: {:>2}/{} accepted; tip p95 median {:.3f} worst {:.3f} mm; camera p95 median {:.3f} worst {:.3f} mm".format(
            name, sum(bool(r[name][0]) for r in rows), len(rows), np.nanmedian(v[:, 0]), np.nanmax(v[:, 0]),
            np.nanmedian(v[:, 1]), np.nanmax(v[:, 1])))
    print("virtual time used: mean {:.0f} s, max {:.0f} s (budget {:.0f}); real time per unit: mean {:.0f} s, max {:.0f} s".format(
        a[:, 5].mean(), a[:, 5].max(), cc.BUDGET["time_s"], a[:, 4].mean(), a[:, 4].max()))
    # the vision
    lat = np.concatenate([r["vision"]["lat"] for r in rows])
    dep = np.concatenate([r["vision"]["dep"] for r in rows])
    lat_t = np.concatenate([r["vision"]["lat_t"] for r in rows])
    dep_t = np.concatenate([r["vision"]["dep_t"] for r in rows])
    dist = 0.35
    print("sphere measurements kept: {} ({:.1f} per unit); lateral error rms {:.3f} px (p95 {:.3f}) = {:.1f} um at 0.35 m; "
          "depth error mean {:+.3f} mm, sd {:.3f} mm".format(len(lat), len(lat) / len(rows), np.sqrt(np.mean(lat ** 2)),
                                                           np.percentile(lat, 95), np.sqrt(np.mean(lat ** 2)) / 1400 * dist * 1e6,
                                                           dep.mean(), dep.std()))
    print("  the same outlines with the TRUE intrinsics: lateral rms {:.3f} px (p95 {:.3f}); depth mean {:+.3f} mm, sd {:.3f} mm".format(
        np.sqrt(np.mean(lat_t ** 2)), np.percentile(lat_t, 95), dep_t.mean(), dep_t.std()))
    ie = np.array([r["intr_err"] for r in rows])
    print("  intrinsics error rms over units: f {:.2f} px, cx {:.2f} px, cy {:.2f} px, k1 {:.4f}, k2 {:.4f}".format(
        *np.sqrt(np.mean(ie ** 2, axis=0))))
    ic = np.array([r["ic"] for r in rows])
    print("  camera test in the image-centre frame (diagnostic): p95 median {:.3f} worst {:.3f} mm; max worst {:.3f} mm; "
          "units within 0.20/0.50: {}/{}, with 1.5x margin: {}/{}".format(
              np.median(ic[:, 0]), ic[:, 0].max(), ic[:, 1].max(), int(np.sum((ic[:, 0] <= 0.2) & (ic[:, 1] <= 0.5))), len(rows),
              int(np.sum((ic[:, 0] <= 0.2 / 1.5) & (ic[:, 1] <= 0.5 / 1.5))), len(rows)))
    cp = np.sort(a[:, 2])
    print("  reference camera p95 over units (sorted, mm):", " ".join("{:.3f}".format(v) for v in cp))
    cm = np.sort(a[:, 3])
    print("  reference camera max over units (sorted, mm):", " ".join("{:.3f}".format(v) for v in cm))
    job_summary(rows)


def job_summary(rows):
    """The job's evidence: the reference's pass count at the task's clearance, its required clearances
    against the clearance, the insertion errors, and every job baseline's pass rate."""
    c = cc.BORE_RADIUS - cc.PIN_RADIUS
    n = len(rows)
    both = sum(bool(r["ok"]) and bool(r["job"]["passed"]) for r in rows)
    print("job: radial clearance {:.3f} mm (bore {:.2f} mm, pin {:.2f} mm); the reference's job passed {}/{}, "
          "prediction and job both {}/{}".format(c * 1e3, 2e3 * cc.BORE_RADIUS, 2e3 * cc.PIN_RADIUS,
                                                 sum(bool(r["job"]["passed"]) for r in rows), n, both, n))
    creq = np.array([r["job"]["creq"] for r in rows]) * 1e3
    fin = creq[np.isfinite(creq)]
    if len(fin):
        print("  clearance the reference's program needs (mm): median {:.3f}, p90 {:.3f}, p99 {:.3f}, max {:.3f} "
              "({:.0f} % / {:.0f} % / {:.0f} % of the clearance at median / p99 / max); not finite {}".format(
                  np.median(fin), np.percentile(fin, 90), np.percentile(fin, 99), fin.max(), 100 * np.median(fin) / (c * 1e3),
                  100 * np.percentile(fin, 99) / (c * 1e3), 100 * fin.max() / (c * 1e3), int(np.sum(~np.isfinite(creq)))))
    ins = [x for r in rows for x in r["job"]["ins"]]
    if ins:
        lat = np.array([x[0] for x in ins]) * 1e3
        ang = np.array([x[1] for x in ins]) * 1e3
        print("  insertion errors at each bore's bottom ({} bores): apex off the axis median {:.3f} p99 {:.3f} max {:.3f} mm; "
              "axis angle median {:.2f} p99 {:.2f} max {:.2f} mrad".format(len(ins), np.median(lat), np.percentile(lat, 99),
                                                                          lat.max(), np.median(ang), np.percentile(ang, 99), ang.max()))
    pl = np.array([r["job"]["plate_lat"] for r in rows]) * 1e3
    print("  plate located: largest mouth error median {:.3f} p90 {:.3f} max {:.3f} mm".format(
        np.median(pl), np.percentile(pl, 90), pl.max()))
    grid = [0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.50, 0.60]
    print("  pass rate of the job against the radial clearance (mm):", " ".join("{:>5.2f}".format(g) for g in grid))
    for name in ("job",) + tuple("job_" + b for b in JOB_BASELINES):
        if not all(name in r for r in rows):
            continue
        cr = np.array([r[name]["creq"] for r in rows]) * 1e3
        print("  {:>10}:".format(name if name != "job" else "reference"),
              " ".join("{:>5.2f}".format(np.mean(cr < g)) for g in grid),
              "  (at the task's {:.3f}: {}/{})".format(c * 1e3, int(np.sum(cr < c * 1e3)), n))
    warm_summary(rows)
    service_summary(rows)


def service_summary(rows):
    """The serviced job's evidence: identification, collisions, the estimates' errors, the serviced job's clearance,
    all criteria together, and the baselines."""
    c = cc.BORE_RADIUS - cc.PIN_RADIUS
    n = len(rows)
    right = sum(r["rq"]["joint"] == r["service"]["joint"] for r in rows)
    print("service: joint identified {}/{}; re-qualification sessions damaged {}; probes used mean {:.1f}, time used mean "
          "{:.0f} s max {:.0f} s; false triggers met {}".format(
              right, n, sum(bool(r["rq"]["damaged"]) for r in rows), np.mean([r["rq"]["probes"] for r in rows]),
              np.mean([r["rq"]["time"] for r in rows]), max(r["rq"]["time"] for r in rows),
              sum(r["rq"]["false"] for r in rows)))
    dz = np.array([r["rq"]["zero"] - r["service"]["zero"] for r in rows]) * 1e6
    db = np.array([r["rq"]["backlash"] - r["service"]["backlash"] for r in rows]) * 1e6
    print("  zero offset error (urad): median |.| {:.0f}, max |.| {:.0f}; backlash error: median |.| {:.0f}, max |.| {:.0f}".format(
        np.nanmedian(np.abs(dz)), np.nanmax(np.abs(dz)), np.nanmedian(np.abs(db)), np.nanmax(np.abs(db))))
    cr = np.array([r["job_serviced"]["creq"] for r in rows]) * 1e3
    f = cr[np.isfinite(cr)]
    if len(f):
        print("  serviced clearance needed (mm): median {:.3f}, p90 {:.3f}, p99 {:.3f}, max {:.3f} (least margin {:.3f} mm); "
              "not finite {}".format(np.median(f), np.percentile(f, 90), np.percentile(f, 99), f.max(), c * 1e3 - f.max(),
                                     int(np.sum(~np.isfinite(cr)))))
    allc = sum(bool(r["ok"]) and bool(r["job"]["passed"]) and bool(r["job_warm"]["passed"]) and bool(r["job_serviced"]["passed"])
               for r in rows)
    print("  units accepted on every criterion (tip and camera limits, cold, warm and serviced job): {}/{}".format(allc, n))
    for name in ("job_serviced",) + tuple("job_" + b for b in SERVICE_BASELINES):
        cr = np.array([r[name]["creq"] for r in rows]) * 1e3
        print("  {:>12}: serviced job passes at the task's {:.3f}: {}/{}".format(name, c * 1e3, int(np.sum(cr < c * 1e3)), n))


def warm_summary(rows):
    """The warm job's evidence: the reference's warm pass count, its clearances, every unit accepted on all
    three criteria, the thermal fit's accuracy, the cold-to-warm displacement, the warm baselines."""
    c = cc.BORE_RADIUS - cc.PIN_RADIUS
    n = len(rows)
    acc = sum(bool(r["ok"]) and bool(r["job"]["passed"]) and bool(r["job_warm"]["passed"]) for r in rows)
    print("warm job (from the published warm start): the reference's warm job passed {}/{}; units accepted (tip and "
          "camera limits, cold job, warm job) {}/{}".format(sum(bool(r["job_warm"]["passed"]) for r in rows), n, acc, n))
    for name, key in (("cold", "job"), ("warm", "job_warm")):
        cr = np.array([r[key]["creq"] for r in rows]) * 1e3
        f = cr[np.isfinite(cr)]
        if len(f):
            print("  {} clearance needed (mm): median {:.3f}, p90 {:.3f}, p99 {:.3f}, max {:.3f} ({:.0f} % of the "
                  "clearance at max; least margin {:.3f} mm); not finite {}".format(
                      name, np.median(f), np.percentile(f, 90), np.percentile(f, 99), f.max(), 100 * f.max() / (c * 1e3),
                      c * 1e3 - f.max(), int(np.sum(~np.isfinite(cr)))))
    ins = [x for r in rows for x in r["job_warm"]["ins"]]
    if ins:
        lat = np.array([x[0] for x in ins]) * 1e3
        print("  warm insertion errors at each bore's bottom ({} bores): apex off the axis median {:.3f} p99 {:.3f} max "
              "{:.3f} mm".format(len(ins), np.median(lat), np.percentile(lat, 99), lat.max()))
    th = [r["thermal"] for r in rows]
    tr = np.array([(t["tau"] - t["tau_true"]) / t["tau_true"] for t in th], float)
    he = np.array([t["heat_err"] for t in th], float)
    print("  thermal fit: tau relative error median {:+.3f}, worst {:.3f}; own-joint heating error rms per link (K/rad) "
          "{}".format(np.nanmedian(tr), np.nanmax(np.abs(tr)), " ".join("{:.4f}".format(v) for v in
                                                                         np.sqrt(np.nanmean(he ** 2, axis=0)))))
    d = [r["disp"] for r in rows if r["disp"]]
    if d:
        allb = np.concatenate(d) * 1e3
        least = np.array([min(x) for x in d]) * 1e3
        most = np.array([max(x) for x in d]) * 1e3
        print("  cold-to-warm displacement of the pin across the bore axes, same joint targets (true kinematics, mm): "
              "all bores median {:.3f} (5 % {:.3f}, 95 % {:.3f}); per unit largest: median {:.3f}, least {:.3f}; units "
              "whose largest exceeds the clearance {}/{}".format(np.median(allb), np.percentile(allb, 5),
                                                               np.percentile(allb, 95), np.median(most), most.min(),
                                                               int(np.sum(most > c * 1e3)), len(d)))
    grid = [0.15, 0.20, 0.25, 0.30, 0.40, 0.60]
    print("  warm pass rate against the radial clearance (mm):", " ".join("{:>5.2f}".format(g) for g in grid))
    for name in ("job_warm",) + tuple("job_" + b for b in WARM_BASELINES):
        cr = np.array([r[name]["creq"] for r in rows]) * 1e3
        print("  {:>10}:".format(name), " ".join("{:>5.2f}".format(np.mean(cr < g)) for g in grid),
              "  (at the task's {:.3f}: {}/{})".format(c * 1e3, int(np.sum(cr < c * 1e3)), n))
    fp = [r["wfirst_pred"] for r in rows if r.get("wfirst_pred")]
    if fp:
        both = sum(bool(r["wfirst_pred"][0]) and r["job_wfirst"]["creq"] < c for r in rows if r.get("wfirst_pred"))
        print("  wfirst: its predictions pass the tip and camera limits {}/{}; predictions and warm job both {}/{}".format(
            sum(bool(x[0]) for x in fp), len(fp), both, len(fp)))

if __name__ == "__main__":
    main()
