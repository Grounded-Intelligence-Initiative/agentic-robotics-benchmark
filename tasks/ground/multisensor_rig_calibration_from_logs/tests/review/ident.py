#!/usr/bin/env python3
"""Identifiability of every scored parameter on a rig's GIVEN sequences: a privileged, truth-initialised calibration.

    <gen venv python> -I -B ident.py --root <rigs root> --set practice --index 0 --name rig_a --hall .. --props .. --husky .. --cache ..
        --tests <task tests dir> --out <json> [--no-camera]

Privileged = it may use the truth to associate data and to start, never as the answer: each estimate below is the
minimiser of a residual made of the LOGGED observations, started at the truth.
  A. LiDAR-IMU rotation, gyro bias, LiDAR clock offset per sequence (then offset + drift by a line through the
     sequences): relative rotations of consecutive sweeps (point-to-plane ICP on sweeps motion-corrected with the true
     motion, started at the true relative pose) against the integrated gyro.
  B. Accelerometer bias: the rests' mean specific force with the estimated rotation (flat floor). IMU position: least
     squares on the accelerometer's samples while moving, the LiDAR's own acceleration taken from the truth (so this
     one says whether the drives excite the lever arm, not how well LiDAR odometry would give it).
  C. Camera-LiDAR extrinsic: maximum of the edge-alignment score (the reference calibrator's: LiDAR depth and
     reflectivity edges against the image's gradient) over the six parameters, started at the truth, on frames at rest
     and in motion. Camera clock offset per sequence: the score's maximum over a time shift with the true extrinsic;
     then offset + drift by a line.
Faulty stretches are left out by the true fault list. The result: where each scored parameter settles, and the
task's metrics of that calibration on the held-out sequence."""
import argparse, csv, json, math, os, sys, time
import numpy as np
from scipy.optimize import least_squares, minimize
from scipy.spatial import cKDTree
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "gen"))
import rigsim, rigsite
ap = argparse.ArgumentParser()
for k in ("root", "set", "index", "name", "hall", "props", "husky", "cache", "tests", "out"):
    ap.add_argument("--" + k, required=True)
ap.add_argument("--no-camera", action="store_true"); ap.add_argument("--stride", type=int, default=1)
a = ap.parse_args()
sys.path.insert(0, a.tests); import rigmetrics as RM
site = rigsite.Site(a.hall, a.props, a.cache)
rig = rigsim.make_rig(rigsim.rig_seed(a.set, a.index), site)
rd, td = os.path.join(a.root, a.set, a.name), os.path.join(a.root, a.set + "_truth", a.name)
faults = json.load(open(os.path.join(td, "faults.json")))["faults"]
DIRS = rigsim.lidar_ray_dirs().reshape(rigsim.LIDAR_RINGS, rigsim.LIDAR_COLS, 3)
CDT = rigsim.column_dt(rig)
T_bl = rig.T_body_lidar; T_li = rig.T_lidar_imu; R_li = T_li[:3, :3]
t0 = time.time()


def col(path, k):
    with open(path) as fh:
        return np.array([float(r[k]) for r in csv.DictReader(fh)])


def bad(seq, sensor, s0, s1):
    return any(f["sequence"] == seq and f["sensor"] == sensor and not (s1 < f["start_s"] - 0.3 or s0 > f["end_s"] + 0.3) for f in faults)


def lidar_pose(traj, tau):
    R, p = rigsim.sensor_pose_w(rig, traj, np.atleast_1d(tau), T_bl)
    return R, p


def deskewed(traj, path, tau0):
    """the sweep's points in the LiDAR frame at column 0's time (true motion), with ring, column"""
    z = np.load(path); r = z["range_mm"].astype(float) / 1000.0
    m = r > 0
    ring, cc = np.nonzero(m)
    p = DIRS[ring, cc] * r[ring, cc][:, None]
    Rj, pj = lidar_pose(traj, tau0 + CDT[cc]); R0, p0 = lidar_pose(traj, tau0)
    pw = np.einsum("nij,nj->ni", Rj, p) + pj
    return (pw - p0[0]) @ R0[0], r, z["reflectivity"].astype(float) / 255.0, ring, cc


def voxel(p, s):
    k = np.floor(p / s).astype(np.int64); _, i = np.unique(k, axis=0, return_index=True); return p[np.sort(i)]


def normals(p, tree, k=10):
    _, nn = tree.query(p, k=k); nb = p[nn]; mu = nb.mean(1, keepdims=True)
    C = np.einsum("nki,nkj->nij", nb - mu, nb - mu) / k
    w, v = np.linalg.eigh(C); return v[:, :, 0]


def icp(src, dst, tree, dn, T, iters=12):
    for it in range(iters):
        ps = src @ T[:3, :3].T + T[:3, 3]
        d, j = tree.query(ps, k=1, distance_upper_bound=0.3)
        m = np.isfinite(d)
        if m.sum() < 100:
            return None
        ps, q, n = ps[m], dst[j[m]], dn[j[m]]
        r = np.sum((ps - q) * n, axis=1); J = np.concatenate([np.cross(ps, n), n], axis=1)
        w = np.minimum(1.0, 0.03 / np.maximum(np.abs(r), 1e-12))
        dx = -np.linalg.solve(J.T @ (J * w[:, None]) + 1e-9 * np.eye(6), J.T @ (w * r))
        dT = np.eye(4); dT[:3, :3] = rigsim.exp_so3(dx[:3]); dT[:3, 3] = dx[3:]; T = dT @ T
        if np.linalg.norm(dx) < 1e-7:
            break
    return T


# ------------------------------------------------------------------ A: sweeps against the gyro
SEQ = []
for k in range(rig.n_visible):
    name = "seq_%02d" % k; sd = os.path.join(rd, name); traj = rig.trajs[k]; T0 = rig.seq_starts[k]
    lst = col(os.path.join(sd, "lidar", "timestamps.csv"), "stamp_s")
    tau_true = rigsim.sensor_to_ref(lst, rig.lidar_offset, rig.lidar_drift) - T0          # privileged: for association and deskew only
    imu = np.loadtxt(os.path.join(sd, "imu.csv"), delimiter=",", skiprows=1)
    rel = []
    prev = None
    for i in range(0, lst.size, a.stride):
        p, _, _, _, _ = deskewed(traj, os.path.join(sd, "lidar", "scans", "%06d.npz" % i), tau_true[i])
        p = voxel(p[(np.linalg.norm(p, axis=1) > 1.0) & (np.linalg.norm(p, axis=1) < 40.0)], 0.2)
        tree = cKDTree(p); nrm = normals(p, tree)
        if prev is not None and not bad(name, "lidar", lst[prev[0]], lst[i]) and not bad(name, "imu", T0 + tau_true[prev[0]], T0 + tau_true[i]):
            Ra, pa = lidar_pose(traj, tau_true[prev[0]]); Rb, pb = lidar_pose(traj, tau_true[i])
            Tt = np.eye(4); Tt[:3, :3] = Ra[0].T @ Rb[0]; Tt[:3, 3] = Ra[0].T @ (pb[0] - pa[0])   # true pose of sweep i in sweep prev
            T = icp(p, prev[1], prev[2], prev[3], Tt.copy())
            if T is not None:
                rel.append((lst[prev[0]], lst[i], rigsim.log_so3(T[:3, :3]), rigsim.log_so3(Tt[:3, :3])))
        prev = (i, p, tree, nrm)
    SEQ.append({"name": name, "T0": T0, "imu": imu, "rel": rel, "lst": lst})
    e = np.array([np.linalg.norm(r[2] - r[3]) for r in rel])
    print("[ident] %s: %d sweep pairs, ICP rotation error median %.4f deg, %.0f s" % (name, len(rel), math.degrees(np.median(e)), time.time() - t0))


def gyro_rot(imu, ta, tb, bias):
    """rotation vector of the integrated gyro between reference times ta and tb (IMU frame, b in a)"""
    t = imu[:, 0]; i0 = np.searchsorted(t, ta); i1 = np.searchsorted(t, tb)
    ts = np.concatenate([[ta], t[i0:i1], [tb]])
    g = np.stack([np.interp(ts, t, imu[:, 1 + j]) for j in range(3)], 1) - bias
    R = np.eye(3)
    for j in range(len(ts) - 1):
        R = R @ rigsim.exp_so3(0.5 * (g[j] + g[j + 1]) * (ts[j + 1] - ts[j]))
    return R


def resid_A(x):
    R = rigsim.exp_so3(x[:3]) @ R_li; b = rig.gyro_bias_scored + x[3:6]
    out = []
    for k, S in enumerate(SEQ):
        o = rig.lidar_offset + rig.lidar_drift * (S["T0"] + 13.0) + x[6 + k]              # the clock's reading error at this sequence
        for sa, sb, rv, _ in S["rel"]:
            Rg = gyro_rot(S["imu"], sa - o, sb - o, b)
            out.append(rigsim.log_so3((R @ Rg @ R.T).T @ rigsim.exp_so3(rv)) / 0.0005)
    return np.concatenate(out)


sol = least_squares(resid_A, np.zeros(6 + len(SEQ)), x_scale=[1e-3] * 3 + [1e-4] * 3 + [1e-3] * len(SEQ), loss="huber", f_scale=3.0, max_nfev=40)
xA = sol.x
R_li_hat = rigsim.exp_so3(xA[:3]) @ R_li; bg_hat = rig.gyro_bias_scored + xA[3:6]
tm = np.array([S["T0"] + 13.0 for S in SEQ]); eo = xA[6:]                                  # clock error at each sequence's middle
cA = np.polyfit(tm, eo, 1) if len(SEQ) > 1 else np.array([0.0, eo[0]])
th = rig.seq_starts[rig.n_visible] + 13.0
RES = {"A": {"imu_rot_deg": math.degrees(np.linalg.norm(xA[:3])), "gyro_bias_err_rps": float(np.linalg.norm(xA[3:6])),
             "lidar_clock_err_per_sequence_ms": (eo * 1e3).round(3).tolist(), "lidar_drift_err_ppm": float(cA[0] * 1e6),
             "lidar_clock_err_at_heldout_ms": float(np.polyval(cA, th) * 1e3), "sweep_pairs": sum(len(S["rel"]) for S in SEQ)}}
print("[ident] A:", json.dumps(RES["A"]), "%.0f s" % (time.time() - t0))
lid_o = rig.lidar_offset + cA[1] - cA[0] * 0.0 * 0; lid_d = rig.lidar_drift + cA[0]
lid_o = rig.lidar_offset + (cA[1])                                                         # e(t) = c0 t + c1 -> offset + c1, drift + c0

# ------------------------------------------------------------------ B: accelerometer
acc_rest = []
for k, S in enumerate(SEQ):
    for ra, rb in rig.rests[k]:
        m = (S["imu"][:, 0] > S["T0"] + ra + 0.6) & (S["imu"][:, 0] < S["T0"] + rb - 0.2)
        if m.sum() > 50 and not bad(S["name"], "imu", S["T0"] + ra, S["T0"] + rb):
            acc_rest.append(S["imu"][m, 4:7])
acc_rest = np.concatenate(acc_rest)
R_bi_hat = T_bl[:3, :3] @ R_li_hat
ba_hat = acc_rest.mean(0) - R_bi_hat.T @ np.array([0.0, 0.0, rigsim.G])
# lever arm: accel_imu - b - R_il (a_L - g)_L = R_il (alpha x t + w x (w x t)), linear in t (LiDAR frame); true LiDAR acceleration
Arows, yrows = [], []
for k, S in enumerate(SEQ):
    traj = rig.trajs[k]; imu = S["imu"]; tau = imu[:, 0] - S["T0"]
    mv = np.abs(np.interp(tau, traj.tau, traj.w)) > 0.25
    for f in faults:
        if f["sequence"] == S["name"] and f["sensor"] == "imu":
            mv &= ~((imu[:, 0] > f["start_s"] - 0.3) & (imu[:, 0] < f["end_s"] + 0.3))
    idx = np.nonzero(mv)[0][::2]
    _, _, aL = traj.point_kinematics(T_bl[:3, 3], tau[idx]); Rb, _ = traj.interp(tau[idx]); Rwl = Rb @ T_bl[:3, :3]
    fL = np.einsum("nji,nj->ni", Rwl, aL - rigsim.GRAVITY_W)                              # specific force at the LiDAR origin, LiDAR frame
    g = (imu[:, 1:4] - bg_hat) @ R_li_hat.T                                               # angular rate in the LiDAR frame
    al = np.gradient(g, 0.005, axis=0)
    # smooth the angular acceleration (the gyro's noise differentiated)
    ker = np.hanning(21); ker /= ker.sum(); al = np.stack([np.convolve(al[:, j], ker, mode="same") for j in range(3)], 1)
    fi = (imu[idx, 4:7] - ba_hat) @ R_li_hat.T
    for n_, i in enumerate(idx):
        Arows.append(RM._hat(al[i]) + RM._hat(g[i]) @ RM._hat(g[i])); yrows.append(fi[n_] - fL[n_])
A_ = np.concatenate(Arows); y_ = np.concatenate(yrows)
t_li_hat, *_ = np.linalg.lstsq(A_, y_, rcond=None)
RES["B"] = {"accel_bias_err_mps2": float(np.linalg.norm(ba_hat - rig.accel_bias_scored)), "imu_position_err_m": float(np.linalg.norm(t_li_hat - T_li[:3, 3])),
            "rest_samples": int(len(acc_rest)), "moving_samples": int(len(y_) // 3)}
print("[ident] B:", json.dumps(RES["B"]), "%.0f s" % (time.time() - t0))
T_li_hat = np.eye(4); T_li_hat[:3, :3] = R_li_hat; T_li_hat[:3, 3] = t_li_hat

# ------------------------------------------------------------------ C: camera
T_lc_hat = rig.T_lidar_camera.copy(); cam_o, cam_d = rig.cam_offset, rig.cam_drift
if not a.no_camera:
    import cv2
    K, D = rig.K, rig.dist; W, H = rigsim.IMG_W, rigsim.IMG_H
    def edges(path):
        """LiDAR edge points of a sweep: depth steps (the near side) and reflectivity steps along each ring; ring, column kept"""
        z = np.load(path); r = z["range_mm"].astype(float) / 1000.0; refl = z["reflectivity"].astype(float) / 255.0
        rn = np.roll(r, -1, axis=1); fn = np.roll(refl, -1, axis=1)
        both = (r > 0.8) & (rn > 0.8) & (r < 30) & (rn < 30)
        same = both & (np.abs(r - rn) < 0.05 + 0.03 * np.minimum(r, rn)); dstep = both & ~same
        wr = np.where(same & (np.abs(refl - fn) > 0.08), np.abs(refl - fn), 0.0)
        wd = np.where(dstep, np.clip(np.abs(r - rn) / 0.5, 0, 1) * 0.5, 0.0)
        out = []
        for wgt, rng_ in ((wr, 0.5 * (r + rn)), (wd, np.minimum(r, rn))):
            ring, cc = np.nonzero(wgt > 0)
            mid = DIRS[ring, cc] + DIRS[ring, (cc + 1) % rigsim.LIDAR_COLS]; mid /= np.linalg.norm(mid, axis=1, keepdims=True)
            out.append((mid * rng_[ring, cc][:, None], wgt[ring, cc], cc))
        return np.concatenate([o[0] for o in out]), np.concatenate([o[1] for o in out]), np.concatenate([o[2] for o in out])
    def edge_img(path):
        g = cv2.cvtColor(cv2.undistort(cv2.imread(path), K, D), cv2.COLOR_BGR2GRAY).astype(np.float64) / 255.0
        gx = cv2.Sobel(g, cv2.CV_64F, 1, 0, ksize=3); gy = cv2.Sobel(g, cv2.CV_64F, 0, 1, ksize=3); e = np.sqrt(gx * gx + gy * gy)
        e = cv2.GaussianBlur(e / max(np.percentile(e, 99), 1e-6), (0, 0), 1.5); return e / max(e.max(), 1e-9)
    def bil(img, u, v):
        u0 = np.floor(u).astype(int); v0 = np.floor(v).astype(int); x = u - u0; y = v - v0
        return (1 - x) * (1 - y) * img[v0, u0] + x * (1 - y) * img[v0, u0 + 1] + (1 - x) * y * img[v0 + 1, u0] + x * y * img[v0 + 1, u0 + 1]
    FR = []
    for k, S in enumerate(SEQ):
        sd = os.path.join(rd, S["name"]); traj = rig.trajs[k]; T0 = S["T0"]
        cst = col(os.path.join(sd, "camera", "timestamps.csv"), "stamp_s")
        tl = rigsim.sensor_to_ref(S["lst"], rig.lidar_offset, rig.lidar_drift) - T0
        for ci in range(8, cst.size - 8, 2):
            if bad(S["name"], "camera", cst[ci] - 0.2, cst[ci] + 0.2):
                continue
            tc = rigsim.sensor_to_ref(cst[ci], rig.cam_offset, rig.cam_drift) - T0
            if ci % 6 and abs(float(np.interp(tc, traj.tau, traj.v))) + abs(float(np.interp(tc, traj.tau, traj.w))) <= 0.2:
                continue                                                                   # at rest: every sixth frame is enough
            li = int(np.argmin(np.abs(tl + 0.05 - tc)))
            if bad(S["name"], "lidar", S["lst"][li] - 0.2, S["lst"][li] + 0.2):
                continue
            pts, w, cc = edges(os.path.join(sd, "lidar", "scans", "%06d.npz" % li))
            Rj, pj = lidar_pose(traj, tl[li] + CDT[cc]); pw = np.einsum("nij,nj->ni", Rj, pts) + pj      # true motion: each edge point in the world
            sp = abs(float(np.interp(tc, traj.tau, traj.v))) + abs(float(np.interp(tc, traj.tau, traj.w)))
            FR.append({"k": k, "stamp": cst[ci], "tc": tc, "pw": pw, "w": w, "img": edge_img(os.path.join(sd, "camera", "frames", "%06d.jpg" % ci)), "traj": traj, "moving": sp > 0.2})
    def score(frames, T_lc, dts=None):
        tot, cnt = 0.0, 0
        for n_, fr in enumerate(frames):
            tc = fr["tc"] + (0.0 if dts is None else dts)
            Rl, pl = lidar_pose(fr["traj"], tc)
            pc = ((fr["pw"] - pl[0]) @ Rl[0] - T_lc[:3, 3]) @ T_lc[:3, :3]
            z = pc[:, 2]; ok = z > 0.5; zs = np.where(ok, z, 1.0)
            u = K[0, 0] * pc[:, 0] / zs + K[0, 2]; v = K[1, 1] * pc[:, 1] / zs + K[1, 2]
            ok &= (u > 3) & (u < W - 4) & (v > 3) & (v < H - 4)
            if ok.sum() < 50:
                continue
            tot += float(bil(fr["img"], u[ok], v[ok]) @ fr["w"][ok]) / float(fr["w"][ok].sum()); cnt += 1
        return tot / max(cnt, 1)
    def pert(x):
        T = rig.T_lidar_camera.copy(); T[:3, :3] = rigsim.exp_so3(x[:3]) @ T[:3, :3]; T[:3, 3] = T[:3, 3] + x[3:]; return T
    st = np.array([0.002] * 3 + [0.01] * 3)
    res = minimize(lambda z: -score(FR, pert(z * st)), np.zeros(6), method="Powell", options={"xtol": 1e-2, "ftol": 1e-6, "maxfev": 1500})
    xC = res.x * st; T_lc_hat = pert(xC)
    # camera clock per sequence: the score over a time shift, true extrinsic, moving frames
    eo = []
    for k in range(len(SEQ)):
        fr = [f for f in FR if f["k"] == k and f["moving"]]
        ds = np.arange(-0.012, 0.0121, 0.001); sc = np.array([score(fr, rig.T_lidar_camera, d) for d in ds])
        j = int(np.argmax(sc)); d = ds[j]
        if 0 < j < len(ds) - 1:
            den = sc[j - 1] - 2 * sc[j] + sc[j + 1]; d = ds[j] + (0.0005 * (sc[j - 1] - sc[j + 1]) / den if den < 0 else 0.0)
        eo.append(-d)                                                                      # the frame is best explained d later: the clock reads d too early
    eo = np.array(eo); cC = np.polyfit(tm, eo, 1) if len(SEQ) > 1 else np.array([0.0, eo[0]])
    cam_o = rig.cam_offset + cC[1]; cam_d = rig.cam_drift + cC[0]
    RES["C"] = {"camera_rot_err_deg": math.degrees(np.linalg.norm(xC[:3])), "camera_trans_err_m": float(np.linalg.norm(xC[3:])),
                "score_at_truth": score(FR, rig.T_lidar_camera), "score_at_optimum": score(FR, T_lc_hat), "score_at_cad": score(FR, rig.T_lidar_camera_prior),
                "camera_clock_err_per_sequence_ms": (eo * 1e3).round(3).tolist(), "camera_drift_err_ppm": float(cC[0] * 1e6),
                "camera_clock_err_at_heldout_ms": float(np.polyval(cC, th) * 1e3), "frames": len(FR), "moving_frames": int(sum(f["moving"] for f in FR))}
    print("[ident] C:", json.dumps(RES["C"]), "%.0f s" % (time.time() - t0))

# ------------------------------------------------------------------ the task's metrics of the settled calibration
truth, ho = RM.load_truth_files(rd, td)
cal = {"T_lidar_camera": T_lc_hat, "T_lidar_imu": T_li_hat, "camera_offset": cam_o, "camera_drift": cam_d,
       "lidar_offset": rig.lidar_offset + cA[1], "lidar_drift": rig.lidar_drift + cA[0], "gyro_bias": bg_hat, "accel_bias": ba_hat}
RES["metrics_settled"] = RM.calibration_metrics(truth, ho, cal)
calT = dict(cal, T_lidar_camera=rig.T_lidar_camera, T_lidar_imu=T_li, camera_offset=rig.cam_offset, camera_drift=rig.cam_drift, lidar_offset=rig.lidar_offset,
            lidar_drift=rig.lidar_drift, gyro_bias=rig.gyro_bias_scored, accel_bias=rig.accel_bias_scored)
RES["metrics_truth"] = RM.calibration_metrics(truth, ho, calT)
print("[ident] settled:", {k: float("%.4g" % v) for k, v in RES["metrics_settled"].items()})
print("[ident] truth:  ", {k: float("%.4g" % v) for k, v in RES["metrics_truth"].items()})
json.dump(RES, open(a.out, "w"), indent=1)
