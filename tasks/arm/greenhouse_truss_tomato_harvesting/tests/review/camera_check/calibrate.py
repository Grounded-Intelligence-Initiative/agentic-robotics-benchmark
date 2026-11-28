#!/usr/bin/env python3
"""The camera-only check, step 1 (maintainers' evidence; nothing runs it at grading; CPU only): the wrist camera's
mounting from the plate views of a captured row section, with what a submission has (the frames, the joint readings,
the plate's surveyed place, the intrinsics, the nominal mounting), against the truth saved beside the frames.

    python3 calibrate.py <capture dir>          (the folder capture.py wrote)
"""
import glob, os, pickle, sys, base64
import numpy as np, cv2
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from rowcell import farm as F, kin
D = sys.argv[1]
meta = pickle.load(open(D + "/meta.pkl", "rb"))
K = np.array(meta["K"], float); plate = meta["plate"]
nx, ny, sq = plate["squares"][0], plate["squares"][1], plate["square"]
o = np.array(plate["origin"], float)
def rgb_of(rec): return cv2.imdecode(np.frombuffer(base64.b64decode(rec["rgb"]), np.uint8), cv2.IMREAD_COLOR)
def obj_points(flip):
    """the board's inner corners in the order the detector may give them: rows along x, either direction each way
    (a chessboard's corners do not say which way round it is labelled; the nominal mounting decides)"""
    ir, jr = flip
    return np.array([o + np.array([i * sq, j * sq, 0.0]) for j in (range(ny - 1, 0, -1) if jr else range(1, ny))
                     for i in (range(nx - 1, 0, -1) if ir else range(1, nx))], float)
nominal = np.array(meta["mount_nominal"], float)
true_mount = F.camera_mount("wrist", meta["cameras"].get("wrist"))
ests = []
for f in sorted(glob.glob(D + "/plate*.pkl")):
    rec = pickle.load(open(f, "rb")); img = rgb_of(rec); gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCornersSB(gray, (nx - 1, ny - 1), flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not ok:
        print(os.path.basename(f), "no board"); continue
    best = None
    for flip in ((False, False), (True, True), (False, True), (True, False)):
        okp, rvec, tvec = cv2.solvePnP(obj_points(flip), corners.reshape(-1, 2).astype(np.float64), K, None, flags=cv2.SOLVEPNP_ITERATIVE)
        Rm, _ = cv2.Rodrigues(rvec); T_cam_cart = kin.pose(Rm, tvec.ravel()); T_cart_cam = kin.inv(T_cam_cart)
        mount = kin.inv(kin.flange(rec["q"])) @ T_cart_cam
        dev = np.linalg.norm(mount[:3, 3] - nominal[:3, 3]) + 0.1 * np.linalg.norm(kin.R_rotvec(mount[:3, :3] @ nominal[:3, :3].T))
        proj, _ = cv2.projectPoints(obj_points(flip), rvec, tvec, K, None)
        rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - corners.reshape(-1, 2)) ** 2, axis=1))))
        if best is None or dev < best[0]: best = (dev, mount, rms)
    ests.append(best[1]); print(os.path.basename(f), "reprojection rms %.2f px" % best[2], "mount t (mm)", np.round(best[1][:3, 3] * 1000, 2))
if ests:
    t = np.mean([m[:3, 3] for m in ests], axis=0)
    U, _, Vt = np.linalg.svd(np.mean([m[:3, :3] for m in ests], axis=0)); Rm = U @ Vt
    est = kin.pose(Rm, t)
    def at(T, p): return T[:3, :3] @ p + T[:3, 3]
    p = np.array([0.0, 0.0, 0.30])        # a point 0.30 m in front of the camera
    e_nom = np.linalg.norm(at(nominal, p) - at(true_mount, p)); e_est = np.linalg.norm(at(est, p) - at(true_mount, p))
    print("views %d; a point 0.30 m in front of the camera, in the flange frame: through the nominal mounting %.2f mm off, through the identified mounting %.2f mm off" % (len(ests), e_nom * 1000, e_est * 1000))
    pickle.dump({"mount_est": est, "mount_true": true_mount, "err_nominal_mm": e_nom * 1000, "err_est_mm": e_est * 1000}, open(D + "/calib.pkl", "wb"))
