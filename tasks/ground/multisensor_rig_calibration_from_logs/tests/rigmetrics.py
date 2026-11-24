"""rigmetrics: the raw engineering metrics of one calibration on one rig's held-out data.

Shared, byte for byte, by the grader and by the practice checker the agent gets. It reads
a submission's `calibration.yaml` and `fault_report.json`, and measures, against a rig's
true parameters and its held-out sequence:

* projection_px      median pixel distance between where held-out LiDAR points land in
                     the held-out camera frames with the submitted calibration and where
                     they really appear (extrinsic and both clocks together; every point at
                     its own column's time within the sweep);
* rpe_rot_deg,       mean error of the LiDAR's relative pose over 1 s windows of the
  rpe_trans_m        held-out sequence, predicted by strapdown integration of the held-out
                     IMU samples with the submitted IMU extrinsic and biases;
* time_rms_s         RMS error of the reference times recovered from the held-out camera
                     and LiDAR stamps with the submitted clock models (the worse of the two);
* imu_rot_deg, gyro_bias_err_rps, accel_bias_err_mps2
                     the IMU mount rotation and bias errors;
* fault_f1           F1 of the reported faulty intervals against the injected ones.

Only numpy and PyYAML are needed.
"""

import json
import math

import numpy as np

GRAVITY_W = np.array([0.0, 0.0, -9.80665])
SENSORS = ("camera", "lidar", "imu", "wheel_odom")
FAULT_TYPES = ("camera_frozen", "camera_time_jump", "lidar_degraded", "imu_axis_swap",
               "odom_glitch")
MAX_FAULTS = 50
RPE_WINDOW_S = 1.0
RPE_STRIDE_S = 0.5
FAULT_IOU = 0.3
PROJ_FAIL_PX = 1000.0


class CalibrationError(ValueError):
    pass


# ----------------------------------------------------------------------------- math

def _hat(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def _exp_batch(w):
    """Rodrigues for an (N,3) array of rotation vectors."""
    th = np.linalg.norm(w, axis=1)
    small = th < 1e-10
    ths = np.where(small, 1.0, th)
    k = w / ths[:, None]
    K = np.zeros((w.shape[0], 3, 3))
    K[:, 0, 1], K[:, 0, 2] = -k[:, 2], k[:, 1]
    K[:, 1, 0], K[:, 1, 2] = k[:, 2], -k[:, 0]
    K[:, 2, 0], K[:, 2, 1] = -k[:, 1], k[:, 0]
    s = np.where(small, 0.0, np.sin(th))[:, None, None]
    c = np.where(small, 0.0, 1.0 - np.cos(th))[:, None, None]
    R = np.eye(3)[None] + s * K + c * (K @ K)
    if np.any(small):
        R[small] = np.eye(3)[None] + np.array([_hat(x) for x in w[small]])
    return R


def _angle_batch(R):
    c = (np.trace(R, axis1=1, axis2=2) - 1.0) / 2.0
    return np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))


def euler_zyx_to_R(yaw, pitch, roll):
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    R = np.empty(np.shape(yaw) + (3, 3))
    R[..., 0, 0] = cy * cp
    R[..., 0, 1] = cy * sp * sr - sy * cr
    R[..., 0, 2] = cy * sp * cr + sy * sr
    R[..., 1, 0] = sy * cp
    R[..., 1, 1] = sy * sp * sr + cy * cr
    R[..., 1, 2] = sy * sp * cr - cy * sr
    R[..., 2, 0] = -sp
    R[..., 2, 1] = cp * sr
    R[..., 2, 2] = cp * cr
    return R


class BodyTrack(object):
    """Body trajectory of the held-out sequence on its fine grid (local time from 0)."""

    def __init__(self, tau, pos, ypr):
        self.tau = np.asarray(tau, float)
        self.pos = np.asarray(pos, float)
        self.ypr = np.asarray(ypr, float)
        dt = float(self.tau[1] - self.tau[0])
        self.vel = np.gradient(self.pos, dt, axis=0)
        rates = np.gradient(self.ypr, dt, axis=0)
        yaw, pitch, roll = self.ypr[:, 0], self.ypr[:, 1], self.ypr[:, 2]
        yd, pd, rd = rates[:, 0], rates[:, 1], rates[:, 2]
        self.omega_b = np.stack([
            rd - yd * np.sin(pitch),
            pd * np.cos(roll) + yd * np.cos(pitch) * np.sin(roll),
            -pd * np.sin(roll) + yd * np.cos(pitch) * np.cos(roll)], axis=1)

    def _i(self, arr, tau):
        tau = np.clip(np.asarray(tau, float), self.tau[0], self.tau[-1])
        return np.stack([np.interp(tau, self.tau, arr[:, i]) for i in range(arr.shape[1])], axis=1)

    def pose(self, tau):
        ang = self._i(self.ypr, tau)
        return euler_zyx_to_R(ang[:, 0], ang[:, 1], ang[:, 2]), self._i(self.pos, tau)

    def velocity(self, tau):
        return self._i(self.vel, tau)

    def omega(self, tau):
        return self._i(self.omega_b, tau)


# ----------------------------------------------------------------------------- reading

def _matrix(obj, name):
    try:
        T = np.array(obj, dtype=float)
    except (TypeError, ValueError):
        raise CalibrationError("%s is not a 4x4 numeric matrix" % name)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise CalibrationError("%s is not a finite 4x4 matrix" % name)
    if np.max(np.abs(T[3] - np.array([0.0, 0.0, 0.0, 1.0]))) > 1e-6:
        raise CalibrationError("%s: last row must be [0, 0, 0, 1]" % name)
    R = T[:3, :3]
    if np.max(np.abs(R.T @ R - np.eye(3))) > 1e-3 or np.linalg.det(R) < 0.0:
        raise CalibrationError("%s: rotation block is not a rotation (orthonormal, det +1, to 1e-3)" % name)
    U, _, Vt = np.linalg.svd(R)
    T = T.copy()
    T[:3, :3] = U @ Vt
    return T


def _number(obj, name):
    if isinstance(obj, bool):
        raise CalibrationError("%s is not a number" % name)
    try:
        x = float(obj)
    except (TypeError, ValueError):
        raise CalibrationError("%s is not a number" % name)
    if not math.isfinite(x):
        raise CalibrationError("%s is not finite" % name)
    return x


def _vector3(obj, name):
    if not isinstance(obj, (list, tuple)) or len(obj) != 3:
        raise CalibrationError("%s is not a list of 3 numbers" % name)
    return np.array([_number(x, "%s[%d]" % (name, i)) for i, x in enumerate(obj)])


def parse_calibration(text):
    """Parse calibration.yaml text into a dict of arrays; raises CalibrationError."""
    import yaml
    try:
        doc = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise CalibrationError("calibration.yaml is not valid YAML: %s" % str(exc).splitlines()[0])
    if not isinstance(doc, dict):
        raise CalibrationError("calibration.yaml is not a mapping")
    out = {}
    for key in ("T_lidar_camera", "T_lidar_imu"):
        if key not in doc:
            raise CalibrationError("calibration.yaml has no %s" % key)
        out[key] = _matrix(doc[key], key)
    time = doc.get("time")
    if not isinstance(time, dict):
        raise CalibrationError("calibration.yaml has no time mapping")
    for sensor in ("camera", "lidar"):
        entry = time.get(sensor)
        if not isinstance(entry, dict):
            raise CalibrationError("time.%s is missing" % sensor)
        out[sensor + "_offset"] = _number(entry.get("offset_s"), "time.%s.offset_s" % sensor)
        drift = _number(entry.get("drift_ppm"), "time.%s.drift_ppm" % sensor)
        if abs(drift) > 1e5:
            raise CalibrationError("time.%s.drift_ppm is out of range" % sensor)
        out[sensor + "_drift"] = drift * 1e-6
    imu = doc.get("imu")
    if not isinstance(imu, dict):
        raise CalibrationError("calibration.yaml has no imu mapping")
    out["gyro_bias"] = _vector3(imu.get("gyro_bias_rps"), "imu.gyro_bias_rps")
    out["accel_bias"] = _vector3(imu.get("accel_bias_mps2"), "imu.accel_bias_mps2")
    return out


def parse_fault_report(text):
    """Parse fault_report.json; raises CalibrationError. Returns a list of dicts."""
    try:
        doc = json.loads(text)
    except ValueError as exc:
        raise CalibrationError("fault_report.json is not valid JSON: %s" % exc)
    if not isinstance(doc, dict) or not isinstance(doc.get("faults"), list):
        raise CalibrationError("fault_report.json must be an object with a 'faults' list")
    items = doc["faults"]
    if len(items) > MAX_FAULTS:
        raise CalibrationError("fault_report.json lists more than %d faults" % MAX_FAULTS)
    out = []
    for i, f in enumerate(items):
        if not isinstance(f, dict):
            raise CalibrationError("faults[%d] is not an object" % i)
        seq, sensor, ftype = f.get("sequence"), f.get("sensor"), f.get("type")
        if not isinstance(seq, str) or sensor not in SENSORS or not isinstance(ftype, str):
            raise CalibrationError("faults[%d] needs a sequence name, a sensor in %s and a type"
                                   % (i, "/".join(SENSORS)))
        a = _number(f.get("start_s"), "faults[%d].start_s" % i)
        b = _number(f.get("end_s"), "faults[%d].end_s" % i)
        if b < a:
            raise CalibrationError("faults[%d] ends before it starts" % i)
        out.append({"sequence": seq, "sensor": sensor, "type": ftype, "start_s": a, "end_s": b})
    return out


# ----------------------------------------------------------------------------- metrics

def _to_ref(stamp, offset, drift):
    return (stamp - offset) / (1.0 + drift)


def projection_px(truth, ho, cal):
    """Median reprojection error (px) of held-out LiDAR points in held-out camera frames."""
    K = truth["K"]
    track = ho["track"]
    T0 = ho["seq_start"]
    T_bl = truth["T_body_lidar"]
    errs = []
    for pair in ho["pairs"]:
        pts = pair["points"]
        if len(pts) == 0:
            continue
        ci, li = pair["cam_idx"], pair["lidar_idx"]
        t_c, t_l = ho["cam_tau"][ci], ho["lidar_tau"][li]
        s_c, s_l = ho["cam_stamp"][ci], ho["lidar_stamp"][li]
        tc_hat = _to_ref(s_c, cal["camera_offset"], cal["camera_drift"]) - T0
        # a point of column j is taken j / columns * sweep (LiDAR clock) after the scan's stamp
        dcol = pair["cols"] / float(truth["lidar_columns"]) * truth["lidar_sweep_s"]
        tl_true = t_l + dcol / (1.0 + truth["lidar_drift"])
        tl_hat = _to_ref(s_l + dcol, cal["lidar_offset"], cal["lidar_drift"]) - T0

        def lidar_pose(tau):
            Rb, pb = track.pose(np.atleast_1d(tau))
            return Rb @ T_bl[:3, :3], pb + np.einsum("nij,j->ni", Rb, T_bl[:3, 3])

        def project(t_cam, t_lid, T_lc):
            Rl, pl = lidar_pose(t_lid)
            pw = np.einsum("nij,nj->ni", Rl, pts) + pl
            Rlc, plc = lidar_pose(t_cam)
            Rc = Rlc[0] @ T_lc[:3, :3]
            pc = plc[0] + Rlc[0] @ T_lc[:3, 3]
            q = (pw - pc) @ Rc
            z = q[:, 2]
            ok = z > 0.05
            zs = np.where(ok, z, 1.0)
            u = K[0, 0] * q[:, 0] / zs + K[0, 2]
            v = K[1, 1] * q[:, 1] / zs + K[1, 2]
            return np.stack([u, v], axis=1), ok

        uv_true, ok_t = project(t_c, tl_true, truth["T_lidar_camera"])
        uv_hat, ok_h = project(tc_hat, tl_hat, cal["T_lidar_camera"])
        e = np.linalg.norm(uv_hat - uv_true, axis=1)
        e = np.where(ok_h, np.minimum(e, PROJ_FAIL_PX), PROJ_FAIL_PX)
        errs.append(e[ok_t])
    if not errs:
        return PROJ_FAIL_PX
    return float(np.median(np.concatenate(errs)))


def time_rms_s(truth, ho, cal):
    """The worse of the camera's and the LiDAR's RMS reference-time error (s) on held-out stamps."""
    T0 = ho["seq_start"]
    out = {}
    for sensor, tau, st in (("camera", ho["cam_tau"], ho["cam_stamp"]),
                            ("lidar", ho["lidar_tau"], ho["lidar_stamp"])):
        t_hat = _to_ref(st, cal[sensor + "_offset"], cal[sensor + "_drift"])
        out[sensor] = float(np.sqrt(np.mean((t_hat - (T0 + tau)) ** 2)))
    return max(out["camera"], out["lidar"]), out


def rpe(truth, ho, cal):
    """Mean rotation (deg) and translation (m) error of IMU-propagated LiDAR relative poses."""
    track = ho["track"]
    T_bl = truth["T_body_lidar"]
    tau_i = ho["imu_tau"]
    gyro = ho["gyro"] - cal["gyro_bias"]
    acc = ho["accel"] - cal["accel_bias"]
    R_li = cal["T_lidar_imu"][:3, :3]
    t_li = cal["T_lidar_imu"][:3, 3]
    dt = float(tau_i[1] - tau_i[0])
    n_win = int(round(RPE_WINDOW_S / dt))
    stride = int(round(RPE_STRIDE_S / dt))
    starts = np.arange(int(round(0.5 / dt)), tau_i.size - n_win - int(round(0.5 / dt)), stride)
    tau0 = tau_i[starts]
    Rb, pb = track.pose(tau0)
    vb = track.velocity(tau0)
    wb = track.omega(tau0)
    R_wl = Rb @ T_bl[:3, :3]
    p_wl = pb + Rb @ T_bl[:3, 3]
    # velocity of the LiDAR origin, then of the submitted IMU location
    w_w = np.einsum("nij,nj->ni", Rb, wb)
    v_wl = vb + np.cross(w_w, np.einsum("nij,j->ni", Rb, T_bl[:3, 3]))
    lever = np.einsum("nij,j->ni", R_wl, t_li)
    R = R_wl @ R_li
    p = p_wl + lever
    v = v_wl + np.cross(w_w, lever)
    for k in range(n_win):
        i0 = starts + k
        w_mid = 0.5 * (gyro[i0] + gyro[i0 + 1])
        a0 = np.einsum("nij,nj->ni", R, acc[i0]) + GRAVITY_W
        R_next = R @ _exp_batch(w_mid * dt)
        a1 = np.einsum("nij,nj->ni", R_next, acc[i0 + 1]) + GRAVITY_W
        p = p + v * dt + (2.0 * a0 + a1) / 6.0 * dt * dt
        v = v + 0.5 * (a0 + a1) * dt
        R = R_next
    tau1 = tau_i[starts + n_win]
    Rb1, pb1 = track.pose(tau1)
    R_wl1 = Rb1 @ T_bl[:3, :3]
    p_wl1 = pb1 + Rb1 @ T_bl[:3, 3]
    R_pred = R @ R_li.T
    p_pred = p - np.einsum("nij,j->ni", R_pred, t_li)
    # relative-pose errors: compare the motion over the window, expressed in the start frame
    rot_err = _angle_batch(np.einsum("nji,njk->nik", R_wl1, R_pred))
    trans_err = np.linalg.norm(np.einsum("nji,nj->ni", R_wl, p_pred - p_wl1), axis=1)
    return float(np.mean(rot_err)), float(np.mean(trans_err))


def imu_errors(truth, cal):
    R_err = truth["T_lidar_imu"][:3, :3].T @ cal["T_lidar_imu"][:3, :3]
    c = (np.trace(R_err) - 1.0) / 2.0
    rot = math.degrees(math.acos(min(1.0, max(-1.0, c))))
    return (rot, float(np.linalg.norm(cal["gyro_bias"] - truth["gyro_bias"])),
            float(np.linalg.norm(cal["accel_bias"] - truth["accel_bias"])))


def _iou(a0, a1, b0, b1, pad=0.05):
    a0, a1, b0, b1 = a0 - pad, a1 + pad, b0 - pad, b1 + pad
    inter = max(0.0, min(a1, b1) - max(a0, b0))
    union = max(a1, b1) - min(a0, b0)
    return inter / union if union > 0 else 0.0


def fault_f1(true_faults, reported):
    """F1 of reported faults; a match needs the same sequence and sensor and IoU >= 0.3.

    A matched report with the right type earns 1, with a wrong type 0.5. Each true fault
    and each report is matched at most once (greedy by IoU).
    """
    if not true_faults and not reported:
        return 1.0
    if not true_faults or not reported:
        return 0.0
    cands = []
    for i, t in enumerate(true_faults):
        for j, r in enumerate(reported):
            if r["sequence"] != t["sequence"] or r["sensor"] != t["sensor"]:
                continue
            iou = _iou(t["start_s"], t["end_s"], r["start_s"], r["end_s"])
            if iou >= FAULT_IOU:
                cands.append((iou, i, j))
    cands.sort(reverse=True)
    used_t, used_r, credit = set(), set(), 0.0
    for iou, i, j in cands:
        if i in used_t or j in used_r:
            continue
        used_t.add(i)
        used_r.add(j)
        credit += 1.0 if reported[j]["type"] == true_faults[i]["type"] else 0.5
    if credit == 0.0:
        return 0.0
    precision = credit / len(reported)
    recall = credit / len(true_faults)
    return 2.0 * precision * recall / (precision + recall)


def calibration_metrics(truth, ho, cal):
    """All calibration metrics of one parsed calibration on one rig."""
    proj = projection_px(truth, ho, cal)
    t_mean, t_each = time_rms_s(truth, ho, cal)
    rpe_rot, rpe_trans = rpe(truth, ho, cal)
    imu_rot, bg, ba = imu_errors(truth, cal)
    return {
        "projection_px": proj,
        "time_rms_s": t_mean,
        "time_rms_camera_s": t_each["camera"],
        "time_rms_lidar_s": t_each["lidar"],
        "rpe_rot_deg": rpe_rot,
        "rpe_trans_m": rpe_trans,
        "imu_rot_deg": imu_rot,
        "gyro_bias_err_rps": bg,
        "accel_bias_err_mps2": ba,
    }


# ----------------------------------------------------------------------------- truth files

def load_truth_files(rig_dir, truth_dir):
    """Truth and held-out data of a practice rig from its folders (practice checker)."""
    import os
    import yaml
    with open(os.path.join(rig_dir, "manifest.json")) as fh:
        man = json.load(fh)
    with open(os.path.join(truth_dir, "ground_truth.yaml")) as fh:
        gt = yaml.safe_load(fh)
    with open(os.path.join(truth_dir, "faults.json")) as fh:
        faults = json.load(fh)
    z = np.load(os.path.join(truth_dir, "heldout.npz"))
    if isinstance(faults, dict):
        faults = faults["faults"]
    truth = {
        "K": np.array(man["camera"]["K"], float),
        "lidar_columns": int(man["lidar"]["columns"]),
        "lidar_sweep_s": float(man["lidar"]["sweep_s"]),
        "lidar_drift": float(gt["time"]["lidar"]["drift_ppm"]) * 1e-6,
        "T_body_lidar": np.array(man["T_body_lidar"], float),
        "T_lidar_camera": np.array(gt["T_lidar_camera"], float),
        "T_lidar_imu": np.array(gt["T_lidar_imu"], float),
        "gyro_bias": np.array(gt["imu"]["gyro_bias_rps"], float),
        "accel_bias": np.array(gt["imu"]["accel_bias_mps2"], float),
        "faults": faults,
    }
    pairs = []
    off = 0
    for ci, li, n in zip(z["pair_cam_idx"], z["pair_lidar_idx"], z["pair_npts"]):
        pairs.append({"cam_idx": int(ci), "lidar_idx": int(li),
                      "points": z["pair_points"][off:off + int(n)],
                      "cols": z["pair_cols"][off:off + int(n)].astype(float)})
        off += int(n)
    ho = {
        "seq_start": float(z["seq_start"]),
        "track": BodyTrack(z["traj_tau"], z["traj_pos"], z["traj_ypr"]),
        "imu_tau": z["imu_tau"], "gyro": z["gyro"], "accel": z["accel"],
        "cam_tau": z["cam_tau"], "cam_stamp": z["cam_stamp"],
        "lidar_tau": z["lidar_tau"], "lidar_stamp": z["lidar_stamp"],
        "pairs": pairs,
    }
    return truth, ho
