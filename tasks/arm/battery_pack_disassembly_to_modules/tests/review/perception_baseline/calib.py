"""The cell's own camera calibration, identified from what the cell offers: the two surveyed checkerboard plates on the
fixture (cell.json, `calibration_plates`), the cameras' exact intrinsics and nominal mounting, the robot's exact
kinematics (a wrist frame's `pose` is the flange's pose times the nominal mounting, so the flange's pose is known).

- `overhead(rgb, ...)`: both plates in one overhead frame, each found in a crop around its nominal projection (the
  mounting offset moves it by up to about 80 pixels), their inner corners together in one PnP. Returns the camera's
  pose in the base frame (4x4, OpenCV axes) and the reprojection error.
- `wrist_view(rgb, flange_T, ...)`: one plate in a wrist frame; its PnP gives the camera in the base frame, and the
  flange's pose turns that into the camera's place on the flange. `average` combines several views.

NumPy and OpenCV only.
"""

import math

import cv2
import numpy as np


def rotvec_R(v):
    R, _ = cv2.Rodrigues(np.asarray(v, dtype=float).reshape(3, 1))
    return R


def R_rotvec(R):
    v, _ = cv2.Rodrigues(np.asarray(R, dtype=float))
    return v.ravel()


def pose_err(Ta, Tb):
    """Position (m) and angle (rad) between two poses."""
    d = np.linalg.inv(Tb) @ Ta
    return float(np.linalg.norm(d[:3, 3])), float(np.linalg.norm(R_rotvec(d[:3, :3])))


def plate_points(plate):
    """The plate's inner corners in the base frame, as a grid [j - 1][i - 1] (i along x, j along y)."""
    (nx, ny), sq = plate["squares"], plate["square"]
    o = np.array(plate["origin"], dtype=float)
    return np.array([[o + [i * sq, j * sq, 0.0] for i in range(1, nx)] for j in range(1, ny)])


def project(K, T, P):
    pc = (np.asarray(P, dtype=float) - T[:3, 3]) @ T[:3, :3]
    return np.stack([pc[..., 0] / pc[..., 2] * K[0][0] + K[0][2], pc[..., 1] / pc[..., 2] * K[1][1] + K[1][2]], axis=-1), pc[..., 2]


def find_board(gray, pattern):
    """Inner corners of a checkerboard (pattern = (points per row, rows)), sub-pixel, or None."""
    ok, c = cv2.findChessboardCornersSB(gray, pattern, flags=cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY)
    if ok:
        return c.reshape(-1, 2)
    ok, c = cv2.findChessboardCorners(gray, pattern, flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None
    c = cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 1e-3))
    return c.reshape(-1, 2)


def solve_pnp(obj, img, K):
    """The camera's pose in the base frame (4x4) from base-frame points and their pixels, and the rms error (px)."""
    Km = np.array(K, dtype=float)
    ok, rv, tv = cv2.solvePnP(np.asarray(obj, dtype=np.float64), np.asarray(img, dtype=np.float64), Km, None,
                              flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, 1e9
    rv, tv = cv2.solvePnPRefineLM(np.asarray(obj, dtype=np.float64), np.asarray(img, dtype=np.float64), Km, None, rv, tv)
    proj, _ = cv2.projectPoints(np.asarray(obj, dtype=np.float64), rv, tv, Km, None)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
    T_cw = np.eye(4)
    T_cw[:3, :3] = rotvec_R(rv)
    T_cw[:3, 3] = tv.ravel()
    return np.linalg.inv(T_cw), rms


def assign(corners, plate, K, T_guess):
    """The detector's ordered corners to the plate's corners: of the four ways the grid can be laid on the plate, the
    one whose PnP pose is nearest to the guess. Returns (object points, image points, pose, rms)."""
    (nx, ny) = plate["squares"]
    grid = plate_points(plate)                              # [ny - 1][nx - 1]
    g = corners.reshape(ny - 1, nx - 1, 2)
    best = None
    for fi in (False, True):
        for fj in (False, True):
            obj = grid[::-1 if fj else 1, ::-1 if fi else 1].reshape(-1, 3)
            T, rms = solve_pnp(obj, g.reshape(-1, 2), K)
            if T is None:
                continue
            dp, da = pose_err(T, T_guess)
            score = dp / 0.01 + da / math.radians(1.0)
            if best is None or score < best[0]:
                best = (score, obj, g.reshape(-1, 2), T, rms)
    return None if best is None else best[1:]


# ways to look at a crop: the renderer's denoiser can wash a colour over part of a plate (the blue harness's, for one),
# which the red channel or the darkest channel sees through; small squares need a zoom
VARIANTS = [("gray", 2, False), ("gray", 3, False), ("r", 2, False), ("r", 3, False), ("gray", 3, True), ("min", 2, False),
            ("min", 3, False)]


def channel(rgb, how):
    if how == "r":
        return np.ascontiguousarray(rgb[..., 0])
    if how == "min":
        return np.ascontiguousarray(rgb.min(axis=2))
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)


def find_plate(rgb, plate, K, T_guess, box=None, variants=VARIANTS):
    """A plate's corners assigned to its points, trying the variants until one gives a PnP of under 0.6 px.
    Returns (object points, image points, pose, rms) or None."""
    for how, zoom, eq in variants:
        g = channel(rgb, how)
        u0, v0 = 0, 0
        if box is not None:
            u0, u1, v0, v1 = box
            g = g[v0:v1, u0:u1]
        if zoom != 1:
            g = cv2.resize(g, None, fx=zoom, fy=zoom, interpolation=cv2.INTER_CUBIC)
        if eq:
            g = cv2.createCLAHE(2.0, (8, 8)).apply(g)
        c = find_board(g, (plate["squares"][0] - 1, plate["squares"][1] - 1))
        if c is None:
            continue
        r = assign(c / zoom + [u0, v0], plate, K, T_guess)
        if r is not None and r[3] < 0.6:
            return r
    return None


def overhead(rgb, K, T_nominal, plates, pad=130):
    """The overhead camera's pose from both plates in one frame. Returns (T, rms px, plates found) or (None, ...)."""
    H, W = rgb.shape[:2]
    objs, imgs, found = [], [], []
    for name, plate in sorted(plates.items()):
        uv, _ = project(K, T_nominal, plate_points(plate).reshape(-1, 3))
        u0, u1 = int(max(uv[:, 0].min() - pad, 0)), int(min(uv[:, 0].max() + pad, W))
        v0, v1 = int(max(uv[:, 1].min() - pad, 0)), int(min(uv[:, 1].max() + pad, H))
        if u1 - u0 < 20 or v1 - v0 < 20:
            continue
        r = find_plate(rgb, plate, K, T_nominal, box=(u0, u1, v0, v1))
        if r is None:
            continue
        objs.append(r[0])
        imgs.append(r[1])
        found.append(name)
    if not objs:
        return None, 1e9, found
    T, rms = solve_pnp(np.concatenate(objs), np.concatenate(imgs), K)
    return T, rms, found


def wrist_view(rgb, K, flange_T, mount_nominal, plate):
    """The wrist camera's place on the flange from one view of `plate` (None when the whole board is not seen)."""
    r = find_plate(rgb, plate, K, flange_T @ mount_nominal, variants=[("gray", 1, False), ("r", 1, False), ("gray", 1, True)])
    if r is None:
        return None, 1e9
    T, rms = r[2], r[3]
    return np.linalg.inv(flange_T) @ T, rms


def average(mounts):
    """The mean of several poses (positions averaged, rotations by the projection of their sum)."""
    t = np.mean([M[:3, 3] for M in mounts], axis=0)
    U, _, Vt = np.linalg.svd(sum(M[:3, :3] for M in mounts))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = R, t
    return out


def robust_average(mounts, tol=0.0008):
    """The average of the views that agree: the view nearest to the others (by the median distance) is the centre,
    and the views within `tol` of it are averaged. Returns (pose, views kept)."""
    if len(mounts) <= 2:
        return average(mounts), list(mounts)
    d = np.array([[pose_err(a, b)[0] for b in mounts] for a in mounts])
    c = mounts[int(np.argmin(np.median(d, axis=1)))]
    keep = [M for M in mounts if pose_err(M, c)[0] < tol]
    return average(keep), keep


def plate_white(rgb, K, T, plates):
    """The plates' white squares as this frame shows them (median RGB): the cell's white reference under the hall's
    light of the moment (the plates' white is the same in every cell)."""
    vals = []
    H, W = rgb.shape[:2]
    for plate in plates.values():
        (nx, ny), sq = plate["squares"], plate["square"]
        o = np.array(plate["origin"], dtype=float)
        pts = np.array([o + [(i + 0.5) * sq, (j + 0.5) * sq, 0.0] for i in range(nx) for j in range(ny) if (i + j) % 2 == 1])
        uv, _ = project(K, T, pts)
        for u, v in uv:
            u, v = int(round(u)), int(round(v))
            if 2 <= u < W - 2 and 2 <= v < H - 2:
                vals.append(rgb[v - 1:v + 2, u - 1:u + 2].reshape(-1, 3))
    return np.median(np.concatenate(vals), axis=0) if vals else None
