"""Observability record of the belt station (maintainers' tool; never run by the grader).

    python3 observe.py <panda dir> [seed ...]      # default: the fixed hidden set (tests/hidden/seeds.json)

Per engine, on an in-process transport (the client accepts any transport) to the cell with its sensor model:
  * the honest program's own eyes (tests/review/honest: heyes.Eyes, hrig.Rig, hseq.Seq.peg_survey) against the cell's
    truth: the engine pose (error at the pulley centres), the tensioner's pointer, the loop on its peg (centreline and
    depth);
  * the sensor audit (cell_manual.md): flange F/T at rest (noise, the reading against the true wrench + bias), depth
    noise of each camera (two frames of a still scene), colour noise, the calibration (the base plate's height from
    back-projected depth);
  * exposure: the clean render of the front camera with its segmentation, each feature's colour
    under the documented extremes of channel gain (0.94-1.06) and brightness (0.92-1.08), saturation, and the
    contrasts a program relies on (the belt below a dark threshold, the face/plate above it; the pointer and window
    marks against the steel pulleys).
Prints one JSON line per engine and a summary line.
"""
import itertools
import json
import math
import os
import shutil
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
sys.path.insert(0, TESTS)
from beltcell.client import Cell  # noqa: E402
from beltcell.server import BeltCellServer  # noqa: E402

FEATURES = {
    "belt": ("belt/g",), "belt_print": ("belt/print",), "plate": ("base_plate",), "pallet": ("pallet",),
    "face": ("eng/face", "eng/block"), "window_marks": ("eng/window",), "peg": ("peg",), "peg_post": ("peg_post",),
    "pulley_steel": tuple("eng/%s/%s" % (k, p) for k in ("C", "WP", "ALT") for p in ("floor", "back", "front")) +
                    ("eng/T/face", "eng/T/back", "eng/T/front"),
    "spoke_marks": tuple("eng/%s/mark" % k for k in ("C", "WP", "ALT", "T")),
    "tensioner_arm": ("eng/T_arm/bar", "eng/T_arm/boss", "eng/T_arm/branch"), "lever": ("eng/T_arm/lever",),
    "pointer": ("eng/T_arm/pointer",),
}
DARK, BRIGHT = 70, 190          # the honest program's thresholds: belt max channel < DARK; pointer min channel > BRIGHT


def loopback(srv):
    box = []

    def send(text):
        req = json.loads(text)
        res = srv.handle(req["method"], req.get("params") or {})
        if isinstance(res, dict) and "error" in res and len(res) == 1:
            box.append(json.dumps({"id": req["id"], "error": res["error"]}))
        else:
            box.append(json.dumps({"id": req["id"], "result": res}))

    def recv():
        return box.pop(0) + "\n"
    return Cell(send, recv)


def given_dir(panda):
    root = tempfile.mkdtemp(prefix="observe_")
    g = os.path.join(root, "given")
    shutil.copytree(os.path.join(TESTS, "setup_payload", "given"), g)
    os.symlink(os.path.abspath(panda), os.path.join(g, "panda"))
    return root, g


def exposure(srv):
    import mujoco
    sc = srv.scene
    m, d = sc.model, sc.data
    rgb, _ = srv.cameras.render(d, "front")
    w, h = srv.cameras.sizes["front"]
    rr = mujoco.Renderer(m, height=int(h), width=int(w))
    rr.enable_segmentation_rendering()
    opt = mujoco.MjvOption()
    opt.geomgroup[3] = 0
    opt.geomgroup[4] = 0
    rr.update_scene(d, camera="front", scene_option=opt)
    seg = rr.render()[..., 0]
    rr.close()
    names = [m.geom(g).name or "" for g in range(m.ngeom)]
    out = {}
    combos = [(np.array(gn), b) for gn in itertools.product((0.94, 1.06), repeat=3) for b in (0.92, 1.08)]
    for feat, prefixes in FEATURES.items():
        ids = [g for g, n in enumerate(names) if n.startswith(prefixes)]
        mask = np.isin(seg, ids)
        n = int(mask.sum())
        if n < 20:
            out[feat] = {"px": n}
            continue
        px = rgb[mask].astype(float)
        p5, p50, p95 = (np.percentile(px, q, axis=0) for q in (5, 50, 95))
        lo = min(float(np.min(p5 * g * b)) for g, b in combos)
        hi = max(float(np.max(p95 * g * b)) for g, b in combos)
        sat = max(float(np.mean(np.any(px * g * b >= 254.5, axis=1))) for g, b in combos)
        drawn = srv.colour["front"]
        act = px * np.asarray(drawn[0]) * float(drawn[1])
        out[feat] = {"px": n, "clean_median": [round(float(v), 1) for v in p50], "extreme_min_channel": round(lo, 1),
                     "extreme_max_channel": round(hi, 1), "saturated_frac_worst": round(sat, 4),
                     "this_engine_median": [round(float(v), 1) for v in np.median(act, axis=0)]}
    # the contrasts the honest program relies on, at the worst extreme
    try:
        out["contrast"] = {
            "belt_max_channel_p95_worst": out["belt"]["extreme_max_channel"],
            "face_plate_min_channel_p5_worst": min(out["face"]["extreme_min_channel"], out["plate"]["extreme_min_channel"]),
            "dark_threshold": DARK,
            "pointer_min_channel_p5_worst": out["pointer"]["extreme_min_channel"],
            "marks_min_channel_p5_worst": out["window_marks"]["extreme_min_channel"],
            "bright_threshold": BRIGHT,
            "steel_max_channel_p95_worst": out["pulley_steel"]["extreme_max_channel"]}
    except KeyError:
        pass
    return out


def sensors(srv, cell, rig, eyes):
    sc = srv.scene
    res = {}
    # F/T at rest: 200 ticks, the raw reading (no tare) against the true tick wrench + the episode's bias
    raws = {s: [] for s in ("left", "right")}
    errs = {s: [] for s in ("left", "right")}
    for _ in range(200):
        cell.step(1)
        st = cell.get_state()
        for s, k in (("left", "L"), ("right", "R")):
            f = st[s]["ft"]
            r6 = np.concatenate([f["force_N"], f["torque_Nm"]])
            raws[s].append(r6)
            errs[s].append(r6 - (np.asarray(sc.ft_tick[k]) + np.asarray(srv.ft_bias[k]) - np.asarray(srv.ft_tare[k])))
    # the same at rest in Cartesian mode (the joint-mode servo holds the arm with a small dither)
    for s_ in ("left", "right"):
        cell.set_control(s_, "cartesian", 1.0)
    cell.step(300)
    cart = {s: [] for s in ("left", "right")}
    for _ in range(200):
        cell.step(1)
        st = cell.get_state()
        for s in cart:
            f = st[s]["ft"]
            cart[s].append(np.concatenate([f["force_N"], f["torque_Nm"]]))
    for s in cart:
        C = np.array(cart[s])
        res["ft_cartesian_rest_" + s] = {"force_rms_N": [round(float(v), 3) for v in C[:, :3].std(axis=0)],
                                        "torque_rms_Nm": [round(float(v), 4) for v in C[:, 3:].std(axis=0)]}
    for s in raws:
        R = np.array(raws[s])
        E = np.array(errs[s])
        res["ft_" + s] = {"force_rms_N": [round(float(v), 3) for v in R[:, :3].std(axis=0)],
                          "torque_rms_Nm": [round(float(v), 4) for v in R[:, 3:].std(axis=0)],
                          "mean_minus_true_plus_bias_N": [round(float(v), 3) for v in E[:, :3].mean(axis=0)],
                          "hand_weight_N": round(float(np.linalg.norm(R[:, :3].mean(axis=0) - np.asarray(srv.ft_bias[s[0].upper()])[:3])), 2)}
    # depth / colour noise: two frames of a still scene per camera
    for cam in ("front", "left_wrist"):
        f1, f2 = eyes.frame(cam), eyes.frame(cam)
        z1, z2 = f1["z"], f2["z"]
        ok = np.isfinite(z1) & np.isfinite(z2)
        dz = (z1 - z2)[ok]
        zm = float(np.median(z1[ok]))
        spec = {"front": 0.0010 + 0.0025 * zm * zm, "left_wrist": 0.0002 + 0.0015 * zm * zm}[cam]
        zz = z1[ok]
        sig = {"front": 0.0010 + 0.0025 * zz * zz, "left_wrist": 0.0002 + 0.0015 * zz * zz}[cam]
        ratio = float(np.std(dz / (np.sqrt(2.0) * sig)))
        rgb_d = f1["rgb"].astype(float) - f2["rgb"].astype(float)
        res["camera_" + cam] = {"noise_over_spec_per_pixel": round(ratio, 3), "median_depth_m": round(zm, 3), "depth_noise_mm": round(1000 * float(np.std(dz)) / math.sqrt(2), 3),
                                "spec_mm": round(1000 * spec, 3), "invalid_frac": round(float(1 - np.isfinite(z1).mean()), 4),
                                "colour_noise_levels": round(float(np.std(rgb_d)) / math.sqrt(2), 2)}
        if cam == "front":
            P, v, u = eyes.backproject(f1)
            sel = (np.abs(P[:, 2]) < 0.015) & (P[:, 0] > 0.05) & (P[:, 0] < 0.45) & (np.abs(P[:, 1]) < 0.6)
            res["calibration_plate_height_mm"] = round(1000 * float(np.median(P[sel, 2])), 3) if sel.any() else None
    return res


def observe(seed, panda, cfg):
    root, given = given_dir(panda)
    os.environ["ARB_GIVEN_DIR"] = given
    sys.path.insert(0, os.path.join(HERE, "honest"))
    import hrig
    import hseq
    from heyes import Eyes
    srv = BeltCellServer(os.path.abspath(panda), config=cfg, render=True, log=lambda s: None)
    srv.reset(seed)
    sc = srv.scene
    cell = loopback(srv)
    out = {"seed": seed, "engine_id": srv.layout["engine_id"]}
    out["exposure_front"] = exposure(srv)
    seq = hseq.Seq(cell, given)
    rig, eyes = seq.r, seq.y
    fr = eyes.frame("front")
    E_raw = eyes.engine_pose(fr)
    E = eyes.engine_pose_calibrated(fr, given)
    errs = {}
    for k, p in rig.dw.pul.items():
        c = p["c"]
        pe = E[0] + E[1] @ np.array([c[0], -0.027, c[1]])
        pt = sc.E_pos + sc.R_E @ np.array([c[0], -0.027, c[1]])
        errs[k] = round(1000 * float(np.linalg.norm(pe - pt)), 2)
    errs_raw = {}
    for k, p in rig.dw.pul.items():
        c = p["c"]
        pe = E_raw[0] + E_raw[1] @ np.array([c[0], -0.027, c[1]])
        pt = sc.E_pos + sc.R_E @ np.array([c[0], -0.027, c[1]])
        errs_raw[k] = round(1000 * float(np.linalg.norm(pe - pt)), 2)
    out["engine_pose"] = {"pulley_centre_err_mm": errs, "uncalibrated_err_mm": errs_raw,
                          "face_normal_err_deg": round(math.degrees(math.acos(min(1.0, float(E[1][:, 1] @ sc.R_E[:, 1])))), 3)}
    rig.set_engine(*E)
    po = eyes.pointer(fr)
    out["pointer"] = {"read_deg": po.get("theta_deg"), "true_deg": round(math.degrees(sc.drive.theta(sc.data)), 2)}
    out["sensors"] = sensors(srv, cell, rig, eyes)
    Q = seq.peg_survey()
    C = np.array([rig.E(p) for p in sc.belt.centres(sc.data)])
    dxz = np.array([np.min(np.hypot(C[:, 0] - q[0], C[:, 1] - q[1])) for q in Q])
    dd = np.array([q[2] - C[int(np.argmin(np.hypot(C[:, 0] - q[0], C[:, 1] - q[1]))), 2] for q in Q])
    out["peg_loop"] = {"points": int(len(Q)), "centreline_err_mm_p50": round(1000 * float(np.median(dxz)), 2),
                       "p95": round(1000 * float(np.percentile(dxz, 95)), 2),
                       "depth_err_mm_p50": round(1000 * float(np.median(np.abs(dd))), 2),
                       "depth_p95": round(1000 * float(np.percentile(np.abs(dd), 95)), 2)}
    out["frames"] = eyes.frames
    shutil.rmtree(root, ignore_errors=True)
    return out


def main():
    panda = sys.argv[1]
    seeds = [int(s) for s in sys.argv[2:]] or json.load(open(os.path.join(TESTS, "hidden", "seeds.json")))["seeds"]
    cfg = json.load(open(os.path.join(TESTS, "grader_config.json")))
    rows = []
    for s in seeds:
        r = observe(s, panda, cfg)
        rows.append(r)
        print(json.dumps(r), flush=True)
    worst = {
        "pulley_err_mm_max": max(max(r["engine_pose"]["pulley_centre_err_mm"].values()) for r in rows),
        "pointer_err_deg_max": max(abs((r["pointer"]["read_deg"] or 999) - r["pointer"]["true_deg"]) for r in rows),
        "peg_loop_p95_max": max(r["peg_loop"]["p95"] for r in rows),
        "belt_max_channel_worst": max(r["exposure_front"]["contrast"]["belt_max_channel_p95_worst"] for r in rows),
        "face_plate_min_channel_worst": min(r["exposure_front"]["contrast"]["face_plate_min_channel_p5_worst"] for r in rows),
        "pointer_min_channel_worst": min(r["exposure_front"]["contrast"]["pointer_min_channel_p5_worst"] for r in rows)}
    print("SUMMARY", json.dumps(worst))


if __name__ == "__main__":
    main()
