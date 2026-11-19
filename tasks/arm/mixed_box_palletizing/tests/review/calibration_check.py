"""calibration_check.py: can a controller without the truth calibrate the cell to about 1 mm?

Maintainers' review tooling. Nothing runs it during grading.

The reference controller is told the truth (joint zero offsets, pallet pose, carton poses). A
submitted controller has to measure them. This script shows, seed by seed, what a controller can
reach from what it legitimately has:

  - the observation dict (`q`, `tcp`, `images` rgb + depth, ...),
  - the drawn constants of task_card.md (camera intrinsics and drawn mounts, survey mark
    positions, kinematics, tool-mark geometry, pallet nominal pose and size, deck, conveyor and
    table heights),
  - its own actions (velocity commands through `PalletCell.step`).

It estimates, in this order:

  1. each camera's extrinsics from its four survey marks fused with a plane fit of its depth
     image (floor / deck, conveyor or table: known heights), and, for comparison, plain
     four-mark PnP;
  2. the four joint zero offsets, by driving the arm (acceleration-limited velocity commands,
     joint limits respected) to 17 poses where the tool mark is in view (12 under the pallet
     camera, 5 under the buffer camera; the infeed camera cannot see it, see pose_plan),
     locating the magenta disc and its yellow dot in rgb + depth, and fitting the offsets (and
     the mark's height, as a nuisance) by least squares against the drawn kinematics;
  3. the pallet's pose (x, y, yaw) from the pallet camera before any carton is on it (edges of
     the deck located to sub-pixel accuracy in rgb, a rectangle of known size fitted to them);
  4. the pose and size of the first carton in the pick station from the infeed camera.

The estimators (section "ESTIMATORS" and "DRIVING THE ARM") read only observations and drawn
constants. The truth (`cell.truth()`, `cell.q_off`, `cell.pallet_xy`, `cell.pallet_yaw`,
`cell.cam_pose`) is read only in section "TRUTH COMPARISON", after the estimates are made.

The truth was used while this script was written, to find two systematic errors of the image
measurements (the centroid of a short cylinder seen at a slant, and the outline of a coloured
disc mixing with different backgrounds on its two sides); both are handled by geometry, not by
constants fitted to the truth. --mark-geometry chooses whether the estimators assume the marks
flat (default: the less a controller is told, the harder) or as thick as pallet_cell.py builds
them and task_card.md now states.

Run it from a scratch directory (MuJoCo writes MUJOCO_LOG.TXT into the working directory, and
`ale lint` refuses stray files in the task folder); a full run of the 8 hidden orders takes
about a minute. MuJoCo rendering must work in process.

    cd "$(mktemp -d)"
    PYTHONDONTWRITEBYTECODE=1 python <task>/tests/review/calibration_check.py               # 8 hidden orders
    PYTHONDONTWRITEBYTECODE=1 python <task>/tests/review/calibration_check.py --practice 1 2 --order p1
    PYTHONDONTWRITEBYTECODE=1 python <task>/tests/review/calibration_check.py --episodes 0 3 --mark-geometry built

Errors reported (all estimated minus true):
  cam   max error over a grid of world points: a point is projected with the true camera and
        back-projected with the estimated one onto the same height; "surf" at the working
        surface (deck 0.144 for the pallet camera, 0.60 for infeed and buffer), "top" at the top
        of the stack (deck + 1.00 for the pallet camera, surface + 0.32 for the others)
  q_off per joint, degrees
  pad   max pad-centre position error over 10 poses above the pallet (deck + 0.2 m and deck +
        1.0 m, centre and four corners), when the controller commands ik(pose) + estimated
        offsets; "yaw" the resulting tool yaw error
  pallet centre (mm) and yaw (degrees)
  carton centre (mm), yaw (degrees), length / width / height (mm)
"""

import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np
from scipy import ndimage
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
TASK = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
import pallet_cell as pc  # noqa: E402  (the task's own simulator)

# ============================================================================================
# DRAWN CONSTANTS (all of these are in task_card.md; read here from pallet_cell's constants,
# which hold the same numbers)
# ============================================================================================

CAM_NAMES = ("infeed", "pallet", "buffer")
CAM_BIT = {"infeed": 1, "pallet": 2, "buffer": 4}
# Known horizontal surfaces each camera sees (world z): the first is the working surface.
CAM_PLANES = {"infeed": (pc.BELT_Z, 0.0), "pallet": (pc.DECK_Z, 0.0), "buffer": (pc.TABLE_Z, 0.0)}
# Thickness of the marks. When this script was written the card called the survey marks "flat on
# their surface" and put the tool mark's disc 0.06 m above the wrist pivot with the dot on it:
# with --mark-geometry card (the default) every mark is taken as flat at that height. The card
# has since been corrected to the cell as built (pallet_cell.py), short cylinders: survey discs 0 to
# 1.6 mm with the dot 1.2 to 2.8 mm above the surface, the tool disc 58 to 62 mm above the pivot
# and the yellow dot 60 to 64 mm, beside the disc rather than on it; --mark-geometry built uses
# that, to show what knowing it is worth. Either way the joint-offset fit carries the tool
# mark's height as a free parameter, so a wrong nominal height does not leak into q2 / q3.
GEOMETRY = {
    "card": {"survey_ring": 0.0, "survey_dot": 0.0, "disc_mid": pc.TOOL_MARK_Z, "disc_top": pc.TOOL_MARK_Z,
             "dot_mid": pc.TOOL_MARK_Z},
    "built": {"survey_ring": 0.0008, "survey_dot": 0.0020, "disc_mid": pc.TOOL_MARK_Z,
              "disc_top": pc.TOOL_MARK_Z + 0.002, "dot_mid": pc.TOOL_MARK_Z + 0.002},
}
MARKS = dict(GEOMETRY["card"])


def drawn_camera(name):
    """K and T_world_cam (OpenCV axes) of a camera as drawn."""
    w, h, fovy, pos, look, up = pc.CAMERAS[name]
    pos = np.array(pos, dtype=float)
    fwd = np.array(look, dtype=float) - pos
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, np.array(up, dtype=float))
    right /= np.linalg.norm(right)
    cam_up = np.cross(right, fwd)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = right, -cam_up, fwd, pos
    f = (h / 2.0) / math.tan(math.radians(fovy) / 2.0)
    K = np.array([[f, 0.0, (w - 1) / 2.0], [0.0, f, (h - 1) / 2.0], [0.0, 0.0, 1.0]])
    return Camera(K, T, w, h)


class Camera(object):
    """A pinhole camera: K, T_world_cam (4x4, OpenCV axes), image size."""

    def __init__(self, K, T, w, h):
        self.K = np.asarray(K, dtype=float)
        self.T = np.asarray(T, dtype=float)
        self.w, self.h = int(w), int(h)

    @property
    def R(self):
        return self.T[:3, :3]

    @property
    def t(self):
        return self.T[:3, 3]

    def project(self, Xw):
        Xc = (np.atleast_2d(Xw) - self.t) @ self.R
        uv = Xc[:, :2] / Xc[:, 2:3] * np.array([self.K[0, 0], self.K[1, 1]]) + self.K[:2, 2]
        return uv, Xc[:, 2]

    def rays_cam(self, uv):
        """Camera-frame rays with unit z (so a depth reading scales them to the point)."""
        uv = np.atleast_2d(uv)
        return np.stack([(uv[:, 0] - self.K[0, 2]) / self.K[0, 0], (uv[:, 1] - self.K[1, 2]) / self.K[1, 1],
                         np.ones(len(uv))], axis=1)

    def to_plane(self, uv, z):
        """World points where the rays through pixels uv meet the horizontal plane at height z."""
        d = self.rays_cam(uv) @ self.R.T
        s = (np.asarray(z, dtype=float) - self.t[2]) / d[:, 2]
        return self.t + d * s[:, None]

    def cloud(self, depth_m):
        """World points of every pixel of a depth image (metres; NaN where there is no reading)."""
        h, w = depth_m.shape
        u, v = np.meshgrid(np.arange(w, dtype=float), np.arange(h, dtype=float))
        Xc = np.stack([(u - self.K[0, 2]) / self.K[0, 0] * depth_m, (v - self.K[1, 2]) / self.K[1, 1] * depth_m,
                       depth_m], axis=-1)
        Xw = Xc @ self.R.T + self.t
        Xw[depth_m <= 0] = np.nan
        return Xc, Xw


def depth_metres(raw):
    """The observation's depth image (uint8 H x W x 2, low and high byte of millimetres) in metres."""
    raw = np.asarray(raw)
    return (raw[..., 0].astype(np.uint16) | (raw[..., 1].astype(np.uint16) << 8)).astype(float) / 1000.0


def sample(img, u, v):
    """Bilinear samples of a 2-D image at (u = column, v = row)."""
    return ndimage.map_coordinates(img, [np.asarray(v, dtype=float), np.asarray(u, dtype=float)], order=1,
                                   mode="nearest")


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def tool_points(q_true, dz=0.0):
    """World positions of the tool mark's disc centre and dot centre (mid-height of each) for
    true joint angles, from the drawn kinematics (task_card.md: Arm, Survey marks); dz raises
    both (the fitted correction of the mark's height)."""
    q1, q2, q3, q4 = (float(v) for v in q_true)
    phi = q2 + q3
    r = pc.A0 + pc.L1 * math.sin(q2) + pc.L2 * math.cos(phi) + pc.A3
    z = pc.H0 + pc.L1 * math.cos(q2) - pc.L2 * math.sin(phi)
    axis = np.array([r * math.cos(q1), r * math.sin(q1), z])
    yaw = q1 + q4
    disc = axis + np.array([0.0, 0.0, MARKS["disc_mid"] + dz])
    dot = axis + np.array([pc.TOOL_DOT_X * math.cos(yaw), pc.TOOL_DOT_X * math.sin(yaw), MARKS["dot_mid"] + dz])
    return disc, dot


# ============================================================================================
# ESTIMATORS (observations and drawn constants only)
# ============================================================================================

# ---- 1. camera extrinsics -------------------------------------------------------------------

def find_survey_marks(rgb, cam_guess, marks, shift=(0.0, 0.0), half=75):
    """Pixel centres of the survey marks within `half` px of where the guessed camera puts them
    (plus `shift`). Returns a list of (index, uv, lift) for the marks found whole inside the
    image; `lift` is the height above its surface of the point whose projection the centre is
    (see below)."""
    gray = rgb.astype(np.float32).mean(axis=2)
    pad = 100
    padded = cv2.copyMakeBorder(gray, pad, pad, pad, pad, cv2.BORDER_REPLICATE)
    found = []
    for k, m in enumerate(marks):
        uv0, zc = cam_guess.project(np.array(m, dtype=float))
        uv0 = uv0[0] + np.asarray(shift)
        r_px = pc.MARK_R * cam_guess.K[0, 0] / zc[0]
        rd_px = pc.MARK_DOT_R * cam_guess.K[0, 0] / zc[0]
        n = int(math.ceil(r_px * 1.6)) * 2 + 1         # template size
        c = (n - 1) / 2.0
        span = half + int(c)                          # template centres within +-half px
        x0, x1 = int(round(uv0[0])) - span, int(round(uv0[0])) + span + 1
        y0, y1 = int(round(uv0[1])) - span, int(round(uv0[1])) + span + 1
        if x1 < 0 or y1 < 0 or x0 > cam_guess.w or y0 > cam_guess.h:
            continue
        x0, x1 = max(x0, -pad), min(x1, cam_guess.w + pad)
        y0, y1 = max(y0, -pad), min(y1, cam_guess.h + pad)
        win = padded[y0 + pad:y1 + pad, x0 + pad:x1 + pad]
        # Templates: the white disc with its black dot, and the dot alone (the cue that is left
        # on the light table, where the white disc hardly shows).
        yy, xx = np.mgrid[0:n, 0:n]
        rr = np.hypot(xx - c, yy - c)
        tpl = np.zeros((n, n), np.float32)
        tpl[rr <= r_px] = 1.0
        tpl[rr <= rd_px] = -1.0
        dot_tpl = np.where(rr <= rd_px, -1.0, 0.0).astype(np.float32)
        dot_tpl[(rr > r_px * 0.5) & (rr <= r_px)] = 0.5
        if win.shape[0] < n or win.shape[1] < n:
            continue
        score = cv2.matchTemplate(win, tpl, cv2.TM_CCOEFF_NORMED) + cv2.matchTemplate(win, dot_tpl, cv2.TM_CCOEFF_NORMED)
        py, px = np.unravel_index(int(np.argmax(score)), score.shape)
        cu, cv_ = x0 + px + c, y0 + py + c
        # Refine with a centroid. The disc and its dot are concentric, so any radially symmetric
        # weight gives the centre. Both are short cylinders (disc 0 to 1.6 mm, dot 1.2 to 2.8 mm
        # above the surface): the outline of a cylinder seen at a slant is symmetric about the
        # projection of its mid-height point, so that is the point a centroid measures.
        # On a dark or grey surface: |I - background| over the whole disc (the white disc
        # dominates, mid-height 0.8 mm). On the light table the white disc does not stand out
        # but its shaded wall does, on one side only: use the dark dot alone (mid-height 2.0 mm).
        ok = False
        for _ in range(3):
            R_bg0, R_bg1 = r_px + 3.0, r_px + 6.0
            lo_x, hi_x = int(math.floor(cu - R_bg1)), int(math.ceil(cu + R_bg1)) + 1
            lo_y, hi_y = int(math.floor(cv_ - R_bg1)), int(math.ceil(cv_ + R_bg1)) + 1
            lo_x, lo_y = max(lo_x, 0), max(lo_y, 0)
            hi_x, hi_y = min(hi_x, cam_guess.w), min(hi_y, cam_guess.h)
            if cu - r_px - 1.5 < 0 or cv_ - r_px - 1.5 < 0 or cu + r_px + 1.5 > cam_guess.w - 1 or \
                    cv_ + r_px + 1.5 > cam_guess.h - 1:
                break                      # the disc is not whole inside the image
            patch = gray[lo_y:hi_y, lo_x:hi_x]
            yy, xx = np.mgrid[lo_y:hi_y, lo_x:hi_x]
            rr = np.hypot(xx - cu, yy - cv_)
            ring = (rr >= R_bg0) & (rr <= R_bg1)
            if ring.sum() < 20:
                break
            bg = float(np.median(patch[ring]))
            if bg > 170.0:
                wgt = np.clip(bg - patch, 0.0, None) * (rr <= rd_px + 1.5)
                lift = MARKS["survey_dot"]
            else:
                wgt = np.abs(patch - bg) * (rr <= r_px + 1.5)
                lift = MARKS["survey_ring"]
            if wgt.sum() <= 0:
                break
            cu, cv_ = float((wgt * xx).sum() / wgt.sum()), float((wgt * yy).sum() / wgt.sum())
            ok = True
        if ok:
            found.append((k, np.array([cu, cv_]), lift))
    return found


def fit_planes(depth_m, cam_guess, heights):
    """The world's up direction in the camera frame and the camera's height, from the depth
    image's pixels on horizontal surfaces of known heights. Returns (n_cam, t_z, inliers)."""
    Xc, _ = cam_guess.cloud(depth_m)
    Xc = Xc[depth_m > 0]
    n = cam_guess.R[2].copy()           # world z in camera coordinates (third row of R)
    tz = cam_guess.t[2]
    tol = 0.05
    for it in range(5):
        zw = Xc @ n + tz
        cls = []
        for h in heights:
            cls.append(np.abs(zw - h) < tol)
        # A pixel belongs to a plane when its height is within the tolerance of it.
        S = np.zeros((3, 3))
        offs, cnts = [], []
        for h, m in zip(heights, cls):
            P = Xc[m]
            if len(P) < 500:
                offs.append(None)
                cnts.append(0)
                continue
            # Depth noise grows with distance: weight by 1 / depth^2.
            wgt = 1.0 / P[:, 2] ** 2
            mu = (wgt[:, None] * P).sum(0) / wgt.sum()
            D = P - mu
            S += (wgt[:, None] * D).T @ D
            offs.append(mu)
            cnts.append(wgt.sum())
        evals, evecs = np.linalg.eigh(S)
        n_new = evecs[:, 0]
        if n_new @ n < 0:
            n_new = -n_new
        n = n_new
        num = sum(c * (h - n @ mu) for h, mu, c in zip(heights, offs, cnts) if mu is not None)
        tz = num / sum(c for c in cnts if c)
        # Tighten the tolerance to about 3 sigma of the depth noise at this distance.
        tol = max(0.012, 0.0015 * 3.0 * float(np.median(Xc[:, 2]))) if it >= 1 else 0.03
    inl = sum(int(m.sum()) for m in cls)
    return n, float(tz), inl


def _fuse(found, marks, R0, n, tz, guess):
    """Yaw about world z and horizontal position that carry the marks' rays, cut with their
    known plane, onto the marks' drawn positions (2-D Procrustes). Returns (T, rms in m)."""
    P, Q = [], []
    for k, uv, lift in found:
        mark = np.array(marks[k], dtype=float)
        ray = guess.rays_cam(uv)[0]
        s = (mark[2] + lift - tz) / (n @ ray)
        P.append((R0 @ (ray * s))[:2])
        Q.append(mark[:2])
    P, Q = np.array(P), np.array(Q)
    mp, mq = P.mean(0), Q.mean(0)
    H = (P - mp).T @ (Q - mq)
    psi = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    Rz = rot_z(psi)
    txy = mq - Rz[:2, :2] @ mp
    rms = float(np.sqrt(np.mean(np.sum((P @ Rz[:2, :2].T + txy - Q) ** 2, axis=1))))
    T = np.eye(4)
    T[:3, :3] = Rz @ R0      # camera to world: the rows of R0 are the world axes in camera coordinates
    T[:3, 3] = [txy[0], txy[1], tz]
    return T, rms


def calibrate_camera(name, rgb, depth_m):
    """Fused calibration: plane fit (tilt + height) from depth, then yaw and horizontal position
    from the survey marks cut with their known plane. Also returns plain four-mark PnP.
    Returns (fused Camera or None, pnp Camera or None, number of marks used)."""
    guess = drawn_camera(name)
    marks = pc.SURVEY_MARKS[name]
    # The mount error moves every mark of a camera by nearly the same amount, up to about 60 px.
    # A first search in wide windows; then, from the median shift of what it found, a second in
    # narrow windows, which another mark nearby (a floor mark seen past the table's edge) cannot
    # capture.
    found = find_survey_marks(rgb, guess, marks, half=75)
    if len(found) >= 2:
        pred = np.array([guess.project(np.array(marks[k], dtype=float))[0][0] for k, _, _ in found])
        shift = np.median(np.array([uv for _, uv, _ in found]) - pred, axis=0)
        found = find_survey_marks(rgb, guess, marks, shift=shift, half=15)
    n, tz, _ = fit_planes(depth_m, guess, CAM_PLANES[name])
    # A rotation whose third row is n (world up in camera coordinates); its first row is the
    # drawn camera's world x, made orthogonal to n.
    r1 = guess.R[0] - (guess.R[0] @ n) * n
    r1 /= np.linalg.norm(r1)
    R0 = np.stack([r1, np.cross(n, r1), n])          # world_from_cam up to a yaw about world z
    fused = None
    if len(found) >= 2:
        T, rms = _fuse(found, marks, R0, n, tz, guess)
        # The marks' distances are exact: a fit that does not close is a wrong detection.
        while rms > 0.002 and len(found) > 2:
            trials = [(_fuse(found[:i] + found[i + 1:], marks, R0, n, tz, guess), i) for i in range(len(found))]
            (T, rms), drop = min(trials, key=lambda tr: tr[0][1])
            found = found[:drop] + found[drop + 1:]
        if rms <= 0.002:
            fused = Camera(guess.K, T, guess.w, guess.h)
    pnp = None
    if len(found) >= 4:
        obj = np.array([np.array(marks[k]) + [0.0, 0.0, lift] for k, _, lift in found], dtype=np.float64)
        img = np.array([uv for _, uv, _ in found], dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj, img, guess.K, None, flags=cv2.SOLVEPNP_IPPE)
        if ok:
            rvec, tvec = cv2.solvePnPRefineLM(obj, img, guess.K, None, rvec, tvec)
            Rcw, _ = cv2.Rodrigues(rvec)
            T = np.eye(4)
            T[:3, :3] = Rcw.T
            T[:3, 3] = (-Rcw.T @ tvec).ravel()
            pnp = Camera(guess.K, T, guess.w, guess.h)
    return fused, pnp, len(found)


# ---- 3. the pallet, 4. the carton: rectangles from sub-pixel edges --------------------------

def edge_offsets(signal, cam, z, points, normals, reach, plateau):
    """For world points on a rectangle's edge (at height z) and their outward horizontal normals,
    the offset (m, along the normal) of the 50 % crossing between the signal inside and outside,
    sampled along the normal on the plane z. NaN where the profile has no clean step."""
    s = np.arange(-reach, reach + 1e-9, 0.0005)
    P = points[:, None, :] + normals[:, None, :] * s[None, :, None]
    P[..., 2] = z
    uv, _ = cam.project(P.reshape(-1, 3))
    prof = sample(signal, uv[:, 0], uv[:, 1]).reshape(len(points), len(s))
    inside = prof[:, (s >= -reach) & (s <= -plateau)].mean(1)
    outside = prof[:, (s >= plateau) & (s <= reach)].mean(1)
    out = np.full(len(points), np.nan)
    for i in range(len(points)):
        a, b = inside[i], outside[i]
        if abs(a - b) < 8.0:
            continue
        f = (prof[i] - b) / (a - b)          # 1 inside, 0 outside
        cross = np.nonzero((f[:-1] >= 0.5) & (f[1:] < 0.5))[0]
        if len(cross) == 0:
            continue
        j = cross[np.argmin(np.abs(s[cross]))]
        out[i] = s[j] + (f[j] - 0.5) / (f[j] - f[j + 1]) * (s[j + 1] - s[j])
    return out


def rect_edges(cx, cy, yaw, hx, hy, step, corner, skip_v=()):
    """Sample points on the four sides of a rectangle: (points, normals, side, along)."""
    R = rot_z(yaw)[:2, :2]
    pts, nrm, side = [], [], []
    for k, (ax, sgn) in enumerate(((0, 1), (0, -1), (1, 1), (1, -1))):
        half_along = hy if ax == 0 else hx
        for a in np.arange(-half_along + corner, half_along - corner + 1e-9, step):
            if ax == 0 and any(abs(a - v) < 0.015 for v in skip_v):
                continue
            local = np.array([sgn * hx, a]) if ax == 0 else np.array([a, sgn * hy])
            nl = np.array([sgn, 0.0]) if ax == 0 else np.array([0.0, sgn])
            pts.append(R @ local + [cx, cy])
            nrm.append(R @ nl)
            side.append(k)
    pts = np.array(pts)
    return np.c_[pts, np.zeros(len(pts))], np.c_[np.array(nrm), np.zeros(len(pts))], np.array(side)


def fit_rect(E, side, cx, cy, yaw, hx, hy, fit_size):
    """Least-squares rectangle through edge points E (world xy), each assigned to a side."""
    def unpack(p):
        return (p[0], p[1], p[2], p[3], p[4]) if fit_size else (p[0], p[1], p[2], hx, hy)

    def res(p):
        x, y, a, hx_, hy_ = unpack(p)
        c, s = math.cos(a), math.sin(a)
        d = E[:, :2] - [x, y]
        lx, ly = c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]
        r = np.empty(len(E))
        for k, (val, h_) in enumerate(((lx, hx_), (-lx, hx_), (ly, hy_), (-ly, hy_))):
            m = side == k
            r[m] = val[m] - h_
        return r

    p0 = [cx, cy, yaw, hx, hy] if fit_size else [cx, cy, yaw]
    sol = least_squares(res, p0, loss="soft_l1", f_scale=0.001)
    return unpack(sol.x)


def top_mask(Xw, z_lo, z_hi, region):
    """The largest connected group of pixels whose world height lies in [z_lo, z_hi] inside the
    region (x0, x1, y0, y1)."""
    z = Xw[..., 2]
    m = np.isfinite(z) & (z > z_lo) & (z < z_hi)
    m &= (Xw[..., 0] > region[0]) & (Xw[..., 0] < region[1]) & (Xw[..., 1] > region[2]) & (Xw[..., 1] < region[3])
    m = ndimage.binary_closing(m, iterations=2)
    m = ndimage.binary_opening(m, iterations=2)
    lab, n = ndimage.label(m)
    if n == 0:
        return None
    sizes = ndimage.sum(m, lab, range(1, n + 1))
    return lab == (1 + int(np.argmax(sizes)))


def coarse_rect(cam, mask, z):
    """Centre, yaw (of the long side, in (-pi/2, pi/2]) and half sizes of a mask's rectangle
    on the plane z."""
    v, u = np.nonzero(mask)
    P = cam.to_plane(np.c_[u, v].astype(float), z)[:, :2]
    (cx, cy), (a, b), ang = cv2.minAreaRect(P.astype(np.float32) * 1000.0)
    yaw = math.radians(ang)
    if b > a:
        a, b = b, a
        yaw += math.pi / 2
    yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2
    return cx / 1000.0, cy / 1000.0, yaw, a / 2000.0, b / 2000.0


def estimate_pallet(rgb, depth_m, cam):
    """Pallet centre (x, y) and yaw from the pallet camera, with the deck's known size."""
    _, Xw = cam.cloud(depth_m)
    hx, hy = pc.PALLET_HALF
    region = (pc.PALLET[0] - hx - 0.15, pc.PALLET[0] + hx + 0.15, pc.PALLET[1] - hy - 0.15, pc.PALLET[1] + hy + 0.15)
    mask = top_mask(Xw, pc.DECK_Z - 0.04, pc.DECK_Z + 0.04, region)
    cx, cy, yaw, _, _ = coarse_rect(cam, mask, pc.DECK_Z)
    # Edge signal: red minus blue separates the wood from the grey floor in any light.
    img = rgb.astype(np.float32)
    signal = img[..., 0] - img[..., 2]
    for reach, plateau in ((0.024, 0.012), (0.016, 0.008), (0.016, 0.008)):
        pts, nrm, side = rect_edges(cx, cy, yaw, hx, hy, 0.003, 0.04, skip_v=(-0.2, 0.0, 0.2))
        off = edge_offsets(signal, cam, pc.DECK_Z, pts, nrm, reach, plateau)
        ok = np.isfinite(off)
        E = pts[ok] + nrm[ok] * off[ok, None]
        cx, cy, yaw, _, _ = fit_rect(E, side[ok], cx, cy, yaw, hx, hy, fit_size=False)
    return np.array([cx, cy]), yaw


def estimate_carton(rgb, depth_m, cam):
    """Centre (x, y), top height, yaw of the long side and size (L, W, H) of the carton in the
    pick station, from the infeed camera."""
    _, Xw = cam.cloud(depth_m)
    sx, sy = pc.STATION
    region = (sx - 0.5, sx + 0.5, sy - 0.45, sy + 0.45)
    mask = top_mask(Xw, pc.BELT_Z + 0.08, pc.BELT_Z + 0.50, region)
    if mask is None:
        return None
    inner = ndimage.binary_erosion(mask, iterations=4)
    z = Xw[..., 2][inner & np.isfinite(Xw[..., 2])]
    top = float(np.median(z))
    z = z[np.abs(z - top) < 0.02]
    top = float(z.mean())
    cx, cy, yaw, hx, hy = coarse_rect(cam, mask, top)
    gray = rgb.astype(np.float32).mean(axis=2)
    for reach, plateau in ((0.012, 0.005), (0.008, 0.003), (0.008, 0.003)):
        pts, nrm, side = rect_edges(cx, cy, yaw, hx, hy, 0.002, 0.02)
        off = edge_offsets(gray, cam, top, pts, nrm, reach, plateau)
        ok = np.isfinite(off)
        E = pts[ok] + nrm[ok] * off[ok, None]
        cx, cy, yaw, hx, hy = fit_rect(E, side[ok], cx, cy, yaw, hx, hy, fit_size=True)
    if hy > hx:
        hx, hy, yaw = hy, hx, yaw + math.pi / 2
    yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2
    return {"xy": np.array([cx, cy]), "yaw": yaw, "size": np.array([2 * hx, 2 * hy, top - pc.BELT_Z]), "top": top}


# ---- 2. the tool mark and the joint offsets -------------------------------------------------

def unmixed_centroid(img, blob):
    """Sub-pixel centroid of a coloured disc (`blob`: its pixels, by colour) in an RGB patch.

    Pixels well inside count 1. A pixel of the outline is a mix of the disc's colour and of
    whatever lies next to it, which differs around the disc (the orange wrist box on one side,
    the floor on the other): it counts by the fraction of disc colour in it, unmixed against the
    background colour found just outside the disc near that pixel."""
    interior = ndimage.binary_erosion(blob, iterations=1)
    zone = ndimage.binary_dilation(blob, iterations=2) & ~interior
    ring = ndimage.binary_dilation(blob, iterations=5) & ~ndimage.binary_dilation(blob, iterations=2)
    colour = np.median(img[ndimage.binary_erosion(blob, iterations=2)], axis=0)
    num = np.stack([ndimage.uniform_filter(img[..., c] * ring, 9) for c in range(3)], axis=-1)
    den = ndimage.uniform_filter(ring.astype(np.float32), 9)[..., None]
    bg = num / np.maximum(den, 1e-6)
    d = colour - bg
    alpha = np.clip(((img - bg) * d).sum(-1) / np.maximum((d * d).sum(-1), 1e-6), 0.0, 1.0)
    wgt = np.where(interior, 1.0, np.where(zone & (den[..., 0] > 0.01), alpha, 0.0))
    yy, xx = np.mgrid[0:img.shape[0], 0:img.shape[1]]
    return np.array([(wgt * xx).sum() / wgt.sum(), (wgt * yy).sum() / wgt.sum()])


def locate_tool_mark(rgb, depth_m, cam, disc_guess, dot_guess):
    """World positions of the magenta disc's centre and the yellow dot's centre (each at its
    mid-height), or None. disc_guess / dot_guess: where the drawn kinematics put them (world).

    Both are short cylinders seen at a slant: the outline's centroid is the projection of the
    mid-height point. The depth pixels inside the disc give the height of its top face."""
    img = rgb.astype(np.float32)
    R_, G_, B_ = img[..., 0], img[..., 1], img[..., 2]
    (u0, v0), zc = cam.project(disc_guess)[0][0], cam.project(disc_guess)[1][0]
    f = cam.K[0, 0]
    r_px = pc.TOOL_MARK_R * f / zc
    half = int(r_px + 0.05 * f / zc + 20)
    x0, x1 = int(max(0, u0 - half)), int(min(cam.w, u0 + half + 1))
    y0, y1 = int(max(0, v0 - half)), int(min(cam.h, v0 + half + 1))
    if x1 - x0 < 2 * r_px or y1 - y0 < 2 * r_px:
        return None
    sl = (slice(y0, y1), slice(x0, x1))
    mag = (R_[sl] > 1.6 * G_[sl] + 20) & (B_[sl] > 1.4 * G_[sl] + 15)
    lab, n = ndimage.label(mag)
    if n == 0:
        return None
    sizes = ndimage.sum(mag, lab, range(1, n + 1))
    k = 1 + int(np.argmax(sizes))
    blob = lab == k
    area_exp = math.pi * r_px ** 2
    if not 0.75 * area_exp < blob.sum() < 1.3 * area_exp:
        return None
    by, bx = np.nonzero(blob)
    if bx.min() < 3 or by.min() < 3 or bx.max() > x1 - x0 - 4 or by.max() > y1 - y0 - 4:
        return None
    uv_disc = unmixed_centroid(img[sl], blob) + [x0, y0]
    # The disc is level: its plane in camera coordinates has the normal of world z.
    n = cam.R[2]
    inner = ndimage.binary_erosion(blob, iterations=3)
    d = depth_m[sl][inner]
    iy, ix = np.nonzero(inner)
    good = d > 0
    if good.sum() < 20:
        return None
    Xc = cam.rays_cam(np.c_[ix[good] + x0, iy[good] + y0].astype(float)) * d[good, None]
    off = Xc @ n
    off = np.median(off) if len(off) < 50 else off[np.abs(off - np.median(off)) < 0.02].mean()
    off_disc = off - (MARKS["disc_top"] - MARKS["disc_mid"])   # the disc's mid-height plane, in camera terms
    ray = cam.rays_cam(uv_disc)[0]
    disc = cam.R @ (ray * off_disc / (n @ ray)) + cam.t
    # The yellow dot, near its expected place relative to the measured disc.
    ud, vd = cam.project(dot_guess)[0][0]
    rd = pc.TOOL_DOT_R * f / zc
    hw = int(rd + 0.02 * f / zc + 6)
    a0, a1 = int(max(0, ud - hw)), int(min(cam.w, ud + hw + 1))
    b0, b1 = int(max(0, vd - hw)), int(min(cam.h, vd + hw + 1))
    if a1 - a0 < 2 * rd or b1 - b0 < 2 * rd:
        return None
    sl2 = (slice(b0, b1), slice(a0, a1))
    yel = (G_[sl2] > 0.75 * R_[sl2]) & (R_[sl2] > 100) & (B_[sl2] < 0.45 * G_[sl2])
    lab, n_ = ndimage.label(yel)
    if n_ == 0:
        return None
    sizes = ndimage.sum(yel, lab, range(1, n_ + 1))
    blob2 = lab == (1 + int(np.argmax(sizes)))
    if not 0.7 * math.pi * rd ** 2 < blob2.sum() < 1.35 * math.pi * rd ** 2:
        return None
    yy, xx = np.nonzero(blob2)
    if xx.min() < 3 or yy.min() < 3 or xx.max() > a1 - a0 - 4 or yy.max() > b1 - b0 - 4:
        return None
    uv_dot = unmixed_centroid(img[sl2], blob2) + [a0, b0]
    ray = cam.rays_cam(uv_dot)[0]
    off_dot = off + (MARKS["dot_mid"] - MARKS["disc_top"])     # the dot's mid-height plane, in camera terms
    dot = cam.R @ (ray * off_dot / (n @ ray)) + cam.t
    return disc, dot


def fit_joint_offsets(samples):
    """Least-squares joint zero offsets (reported minus true) from (q_reported, disc, dot)
    samples, with the tool mark's height above its nominal as a fifth, free parameter.
    Returns (offsets, height correction in m, rms residual in m)."""
    def res(p):
        r = []
        for q, disc, dot in samples:
            pd, pt = tool_points(np.asarray(q) - p[:4], p[4])
            r.extend(pd - disc)
            r.extend(pt[:2] - dot[:2])
        return np.array(r)

    sol = least_squares(res, np.zeros(5), x_scale=np.r_[np.radians([0.5, 0.3, 0.3, 1.0]), 0.005])
    rms = math.sqrt(np.mean(sol.fun ** 2))
    return sol.x[:4], float(sol.x[4]), rms


# ============================================================================================
# DRIVING THE ARM (velocity commands, as a controller sends them)
# ============================================================================================

class Driver(object):
    """Moves the pad (tcp, in the readings' frame) with acceleration-limited velocity commands."""

    VMAX, AMAX, WMAX, WACC = 0.8, 2.0, 1.2, 4.0

    def __init__(self, cell, obs):
        self.cell = cell
        self.obs = obs
        self.cmd = np.zeros(4)
        self.steps = 0

    def step(self, cmd, cams=0):
        cmd = np.asarray(cmd, dtype=float)
        self.obs = self.cell.step([cmd[0], cmd[1], cmd[2], cmd[3], 0, cams, 0], raw=True)
        self.cmd = cmd
        self.steps += 1
        return self.obs

    def move(self, goal, yaw, tol=0.0008, max_steps=200):
        goal = np.asarray(goal, dtype=float)
        dt = pc.CONTROL_DT
        for _ in range(max_steps):
            tcp = np.array(self.obs["tcp"])
            err = goal - tcp[:3]
            dist = float(np.linalg.norm(err))
            yerr = yaw - tcp[3]          # no wrap: the goal yaw was chosen with q4 inside its limits
            if dist < tol and abs(yerr) < 0.002 and np.linalg.norm(self.cmd) < 0.01:
                break
            speed = min(self.VMAX, 2.5 * dist, math.sqrt(2.0 * 0.5 * self.AMAX * dist))
            v = err / dist * speed if dist > 1e-9 else np.zeros(3)
            w = max(-self.WMAX, min(self.WMAX, 2.5 * yerr))
            new = np.zeros(4)
            new[:3] = self.cmd[:3] + np.clip(v - self.cmd[:3], -self.AMAX * dt, self.AMAX * dt)
            new[3] = self.cmd[3] + max(-self.WACC * dt, min(self.WACC * dt, w - self.cmd[3]))
            self.step(new)
        # Stand still until the joints are at rest.
        for _ in range(30):
            self.step(np.zeros(4))
            if np.max(np.abs(self.obs["dq"])) < 2e-4:
                break

    def travel(self, goal, yaw, z_travel):
        """Up to the travel height, around the base axis at that height, down to the goal."""
        tcp = np.array(self.obs["tcp"])
        self.move([tcp[0], tcp[1], z_travel], tcp[3], tol=0.02)
        a0, a1 = math.atan2(tcp[1], tcp[0]), math.atan2(goal[1], goal[0])
        a0 += 2 * math.pi if a0 < -1.0 else 0.0
        a1 += 2 * math.pi if a1 < -1.0 else 0.0
        r0, r1 = math.hypot(tcp[0], tcp[1]), math.hypot(goal[0], goal[1])
        n = max(1, int(math.ceil(abs(a1 - a0) / 0.4)))
        for k in range(1, n + 1):
            f = k / float(n)
            a, r = a0 + (a1 - a0) * f, r0 + (r1 - r0) * f
            self.move([r * math.cos(a), r * math.sin(a), z_travel], tcp[3] + (yaw - tcp[3]) * f, tol=0.03)
        self.move(goal, yaw)

    def look(self, names):
        bits = sum(CAM_BIT[n] for n in names)
        return self.step(np.zeros(4), cams=bits)


def pose_plan():
    """Pad positions (tcp, readings' frame) where the tool mark is in view: (camera, xyz).

    None for the infeed camera: MuJoCo's near clipping plane is a fraction (0.02) of the model's
    extent, and the parked cartons make the extent about 65 to 71 m, so nothing nearer than about
    1.3 to 1.4 m to a camera is drawn. The infeed camera hangs 2.6 m high: the tool mark (0.61 m
    above the pad) would have to be below 1.2 m, the pad below the conveyor. For the buffer
    camera (3.0 m) the pad must stay below about 0.95 m; the pallet camera is not limited."""
    plan = []
    for x, y, z in ((-0.35, 1.25, 0.45), (0.35, 1.25, 0.80), (0.35, 1.75, 0.45), (-0.35, 1.75, 0.80),
                    (0.0, 1.50, 1.20), (-0.20, 1.30, 0.60), (0.20, 1.70, 1.00), (0.40, 1.50, 0.55),
                    (-0.40, 1.50, 1.05), (0.10, 1.32, 1.40), (-0.10, 1.68, 1.40), (0.0, 1.50, 0.35)):
        plan.append(("pallet", (x, y, z)))
    for x, y, z in ((-1.25, -0.25, 0.70), (-1.25, 0.25, 0.85), (-1.65, 0.25, 0.70), (-1.65, -0.25, 0.85),
                    (-1.45, 0.0, 0.78)):
        plan.append(("buffer", (x, y, z)))
    return plan


def choose_yaw(cam, xyz, k):
    """A tool yaw for pose k that keeps the yellow dot in the image and q4 inside its limits."""
    q1 = math.atan2(xyz[1], xyz[0])
    q1 += 2 * math.pi if q1 < pc.Q_MIN[0] else 0.0
    for delta in (-1.2, -0.6, 0.0, 0.6, 1.2)[k % 5:] + (-1.2, -0.6, 0.0, 0.6, 1.2)[:k % 5]:
        for flip in (0.0, math.pi, -math.pi):
            yaw = q1 + delta + flip
            q = pc.ik(xyz, yaw)
            if q is None or np.any(q < pc.Q_MIN + 0.05) or np.any(q > pc.Q_MAX - 0.05):
                continue
            disc, dot = tool_points(q)
            uv, _ = cam.project(np.array([disc, dot]))
            if np.all(uv > 45) and np.all(uv[:, 0] < cam.w - 45) and np.all(uv[:, 1] < cam.h - 45):
                return yaw
    return None


def collect_tool_samples(driver, cams):
    """Drive the arm through the pose plan and locate the tool mark at each pose."""
    samples, used = [], {n: 0 for n in CAM_NAMES}
    for k, (name, xyz) in enumerate(pose_plan()):
        cam = cams[name]
        if cam is None:
            continue
        yaw = choose_yaw(cam, xyz, k)
        if yaw is None:
            continue
        driver.travel(xyz, yaw, z_travel=max(1.15, xyz[2]))
        obs = driver.look([name])
        q = np.array(obs["q"])
        disc_g, dot_g = tool_points(q)           # where the readings put the mark, as drawn
        im = obs["images"][name]
        got = locate_tool_mark(im["rgb"], depth_metres(im["depth"]), cam, disc_g, dot_g)
        if got is None:
            continue
        samples.append((q, got[0], got[1]))
        used[name] += 1
    return samples, used


# ============================================================================================
# ONE ORDER: estimate everything (no truth in here)
# ============================================================================================

def estimate_all(cell, order, seed):
    t0 = time.time()
    obs = cell.reset(order, seed, raw=True)       # the first observation carries all three images
    est = {"cams_fused": {}, "cams_pnp": {}, "marks": {}}
    for name in CAM_NAMES:
        im = obs["images"][name]
        fused, pnp, nm = calibrate_camera(name, im["rgb"], depth_metres(im["depth"]))
        est["cams_fused"][name], est["cams_pnp"][name], est["marks"][name] = fused, pnp, nm
    cams = est["cams_fused"]
    im = obs["images"]["pallet"]
    est["pallet"] = estimate_pallet(im["rgb"], depth_metres(im["depth"]), cams["pallet"]) if cams["pallet"] else None
    driver = Driver(cell, obs)
    # The first carton arrives after about 1 s; look at it once it has settled.
    while driver.obs["delivered"] < 1 or not driver.obs["infeed_present"]:
        driver.step(np.zeros(4))
    for _ in range(6):
        driver.step(np.zeros(4))
    obs = driver.look(["infeed"])
    im = obs["images"]["infeed"]
    est["carton"] = estimate_carton(im["rgb"], depth_metres(im["depth"]), cams["infeed"]) if cams["infeed"] else None
    samples, used = collect_tool_samples(driver, cams)
    est["samples"], est["used"] = len(samples), used
    est["q_off"], est["mark_dz"], est["fit_rms"] = fit_joint_offsets(samples) if len(samples) >= 6 else (None, None, None)
    est["steps"] = driver.steps
    est["time"] = time.time() - t0
    return est


# ============================================================================================
# TRUTH COMPARISON (reporting only)
# ============================================================================================

def true_camera(cell, name):
    P = cell.cam_pose[name]
    return Camera(P["K"], P["T_world_cam"], P["width"], P["height"])


def camera_error(cam_est, cam_true, name):
    """Max world error (mm) of back-projected points at the working surface and at the top."""
    if cam_est is None:
        return float("nan"), float("nan")
    if name == "pallet":
        cx, cy, hx, hy = pc.PALLET[0], pc.PALLET[1], pc.PALLET_HALF[0] + 0.03, pc.PALLET_HALF[1] + 0.03
        z0, z1 = pc.DECK_Z, pc.DECK_Z + pc.STACK_HEIGHT_MAX
    elif name == "infeed":
        cx, cy, hx, hy = pc.STATION[0], pc.STATION[1], 0.40, 0.35
        z0, z1 = pc.BELT_Z, pc.BELT_Z + 0.32
    else:
        cx, cy, hx, hy = pc.TABLE[0], pc.TABLE[1], pc.TABLE_HALF[0] - 0.05, pc.TABLE_HALF[1] - 0.05
        z0, z1 = pc.TABLE_Z, pc.TABLE_Z + 0.32
    out = []
    for z in (z0, z1):
        gx, gy = np.meshgrid(np.linspace(cx - hx, cx + hx, 9), np.linspace(cy - hy, cy + hy, 7))
        X = np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, z)]
        uv, _ = cam_true.project(X)
        inside = (uv[:, 0] >= 0) & (uv[:, 0] <= cam_true.w - 1) & (uv[:, 1] >= 0) & (uv[:, 1] <= cam_true.h - 1)
        Y = cam_est.to_plane(uv[inside], z)
        out.append(1000.0 * float(np.max(np.linalg.norm(Y - X[inside], axis=1))))
    return out[0], out[1]


def pad_errors(q_off_est, q_off_true):
    """Max pad-centre error (mm) and max tool-yaw error (deg) over poses above the pallet, when
    the controller commands the readings ik(pose) + estimated offsets."""
    worst, worst_yaw = 0.0, 0.0
    for dz in (0.2, 1.0):
        for dx, dy in ((0, 0), (0.45, 0.25), (-0.45, 0.25), (0.45, -0.25), (-0.45, -0.25)):
            goal = np.array([pc.PALLET[0] + dx, pc.PALLET[1] + dy, pc.DECK_Z + dz])
            yaw = math.pi / 2
            q_need = pc.ik(goal, yaw)                     # true joint angles that reach the goal
            q_cmd = q_need + q_off_est                     # what the controller asks the readings for
            got, got_yaw = pc.fk(q_cmd - q_off_true)       # where the pad really goes
            worst = max(worst, 1000.0 * float(np.linalg.norm(got - goal)))
            worst_yaw = max(worst_yaw, abs(math.degrees(got_yaw - yaw)))
    return worst, worst_yaw


def compare(cell, est):
    truth = cell.truth()
    row = {}
    for name in CAM_NAMES:
        ct = true_camera(cell, name)
        row["cam_" + name] = camera_error(est["cams_fused"][name], ct, name)
        row["pnp_" + name] = camera_error(est["cams_pnp"][name], ct, name)
    q_true = np.array(truth["q_off"])
    if est["q_off"] is not None:
        row["q_err"] = np.degrees(est["q_off"] - q_true)
        row["pad"] = pad_errors(est["q_off"], q_true)
    else:
        row["q_err"], row["pad"] = np.full(4, np.nan), (float("nan"), float("nan"))
    row["pad_uncal"] = pad_errors(np.zeros(4), q_true)
    if est["pallet"] is not None:
        xy, yaw = est["pallet"]
        row["pallet"] = (1000.0 * float(np.linalg.norm(xy - truth["pallet_xy"])), math.degrees(yaw - truth["pallet_yaw"]))
    else:
        row["pallet"] = (float("nan"), float("nan"))
    c = est["carton"]
    first = [t for t in truth["cartons"] if t["index"] == 0]
    if c is not None and first:
        t = first[0]
        L, W, H = t["size"]
        period = math.pi / 2 if abs(L - W) < 0.02 else math.pi
        dyaw = (c["yaw"] - t["yaw"] + period / 2) % period - period / 2
        tsize = np.array([max(L, W), min(L, W), H])
        row["carton"] = (1000.0 * float(np.linalg.norm(c["xy"] - np.array(t["pos"][:2]))), math.degrees(dyaw),
                         *(1000.0 * (c["size"] - tsize)))
    else:
        row["carton"] = (float("nan"),) * 5
    return row


# ============================================================================================
# MAIN
# ============================================================================================

def load_runs(args):
    if args.practice:
        with open(os.path.join(TASK, "environment", "setup", "lab", "practice.json")) as handle:
            prac = json.load(handle)
        order = prac["orders"][args.order]
        return prac["catalog"], [("{}/s{}".format(args.order, s), order, s) for s in args.practice]
    with open(os.path.join(TESTS, "hidden", "catalog.json")) as handle:
        catalog = json.load(handle)
    with open(os.path.join(TESTS, "hidden", "episodes.json")) as handle:
        episodes = json.load(handle)
    pick = args.episodes if args.episodes else range(len(episodes))
    return catalog, [(episodes[i]["name"], episodes[i]["order"], episodes[i]["seed"]) for i in pick]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--practice", type=int, nargs="+", help="use practice order --order with these seeds")
    ap.add_argument("--order", default="p0", help="practice order id (with --practice)")
    ap.add_argument("--episodes", type=int, nargs="+", help="indices of the hidden episodes to run")
    ap.add_argument("--mark-geometry", choices=sorted(GEOMETRY), default="card",
                    help="thickness of the painted marks the estimators assume (default: card, as a controller knows it)")
    args = ap.parse_args()
    MARKS.update(GEOMETRY[args.mark_geometry])
    print("mark geometry: {}".format(args.mark_geometry))
    catalog, runs = load_runs(args)
    cell = pc.PalletCell(catalog)
    rows = []
    t_all = time.time()
    for label, order, seed in runs:
        est = estimate_all(cell, order, seed)
        row = compare(cell, est)
        rows.append(row)
        print("\n== {}  (seed {}, {} control steps, {:.1f} s, tool samples {} = {})".format(
            label, seed, est["steps"], est["time"], est["samples"],
            "/".join("{}:{}".format(k, v) for k, v in est["used"].items())))
        print("  marks found        infeed {} pallet {} buffer {}".format(*(est["marks"][n] for n in CAM_NAMES)))
        print("  camera, fused (mm) " + "  ".join("{} {:.2f} / {:.2f}".format(n, *row["cam_" + n]) for n in CAM_NAMES)
              + "    [surface / top]")
        print("  camera, PnP   (mm) " + "  ".join("{} {:.2f} / {:.2f}".format(n, *row["pnp_" + n]) for n in CAM_NAMES))
        print("  q_off error (deg)  " + " ".join("{:+.4f}".format(v) for v in row["q_err"])
              + "   fit rms {:.2f} mm, tool mark height {:+.2f} mm".format(
                  *((1000 * est["fit_rms"], 1000 * est["mark_dz"]) if est["fit_rms"] is not None else (np.nan, np.nan))))
        print("  pad over pallet    max {:.2f} mm, yaw {:.3f} deg   (uncalibrated: {:.1f} mm, {:.2f} deg)".format(
            row["pad"][0], row["pad"][1], *row["pad_uncal"]))
        print("  pallet             centre {:.2f} mm, yaw {:+.4f} deg".format(*row["pallet"]))
        print("  carton 0           centre {:.2f} mm, yaw {:+.3f} deg, L {:+.2f} W {:+.2f} H {:+.2f} mm".format(*row["carton"]))
        sys.stdout.flush()

    def stat(vals):
        a = np.abs(np.array(vals, dtype=float))
        return "{:7.3f} {:7.3f}".format(np.nanmedian(a), np.nanmax(a))

    print("\n== summary over {} orders ({:.0f} s): |error| median, max".format(len(rows), time.time() - t_all))
    lines = []
    for n in CAM_NAMES:
        lines.append(("camera {} fused, surface (mm)".format(n), [r["cam_" + n][0] for r in rows]))
        lines.append(("camera {} fused, top (mm)".format(n), [r["cam_" + n][1] for r in rows]))
    for n in CAM_NAMES:
        lines.append(("camera {} 4-mark PnP, surface (mm)".format(n), [r["pnp_" + n][0] for r in rows]))
        lines.append(("camera {} 4-mark PnP, top (mm)".format(n), [r["pnp_" + n][1] for r in rows]))
    for j in range(4):
        lines.append(("q{} offset (deg)".format(j + 1), [r["q_err"][j] for r in rows]))
    lines.append(("pad over pallet (mm)", [r["pad"][0] for r in rows]))
    lines.append(("tool yaw over pallet (deg)", [r["pad"][1] for r in rows]))
    lines.append(("pad over pallet, uncalibrated (mm)", [r["pad_uncal"][0] for r in rows]))
    lines.append(("pallet centre (mm)", [r["pallet"][0] for r in rows]))
    lines.append(("pallet yaw (deg)", [r["pallet"][1] for r in rows]))
    for k, nm in enumerate(("centre (mm)", "yaw (deg)", "length (mm)", "width (mm)", "height (mm)")):
        lines.append(("carton 0 " + nm, [r["carton"][k] for r in rows]))
    for name, vals in lines:
        print("  {:38s} {}".format(name, stat(vals)))


if __name__ == "__main__":
    main()
