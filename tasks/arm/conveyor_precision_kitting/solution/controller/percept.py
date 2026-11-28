"""Perception for the reference controller: camera models from the cell's data sheet, colour segmentation, and a
metric template fit of the six variants (pose, flip) on a known plane. Uses only documented quantities."""
import math

import numpy as np
import cv2

# ---- data sheet (nominal) -------------------------------------------------------------------------------------
H_PART = 0.020
VARIANTS = {
    "A1": (0.044, 0.024, "corner", (0.010, 0.008), "blue"),
    "A2": (0.052, 0.024, "corner", (0.010, 0.008), "blue"),
    "B1": (0.040, 0.028, "slot", (0.010, 0.008), "green"),
    "B2": (0.040, 0.028, "corner", (0.010, 0.008), "green"),
    "C1": (0.038, 0.030, "corner", (0.009, 0.008), "orange"),
    "C2": (0.046, 0.026, "tab", (0.008, 0.006), "orange"),
}
FAMILY = {"blue": ("A1", "A2"), "green": ("B1", "B2"), "orange": ("C1", "C2")}
MARK_R, MARK_INSET = 0.0035, 0.008
# wrist camera in the grip-site frame (site frame: x = closing direction, z = approach direction, pointing down)
WRIST_POS = np.array([0.0, 0.05, -0.097])
WRIST_R = np.diag([-1.0, 1.0, -1.0])           # camera axes (x right, y up, z backwards) in the site frame
WRIST_FOVY, WRIST_WH = 75.0, (160, 120)
INSPECT_POS = np.array([-0.10, 0.04, 0.812])
INSPECT_R = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
INSPECT_FOVY, INSPECT_WH = 40.0, (96, 96)
R_DOWN = np.diag([1.0, -1.0, -1.0])
OVERVIEW_POS, OVERVIEW_TGT, OVERVIEW_FOVY, OVERVIEW_WH = (0.02, 0.0, 1.75), (-0.02, 0.0, 0.80), 58.0, (160, 120)


def look_at(pos, target):
    pos, target = np.asarray(pos, float), np.asarray(target, float)
    z = pos - target
    z = z / np.linalg.norm(z)
    x = np.cross([0.0, 0.0, 1.0], z)
    x = x / np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], axis=1)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def outline(name):
    """Part outline polygon (counter-clockwise, part frame, top view, upright)."""
    L, W, key, (a, b), _f = VARIANTS[name]
    x0, x1, y0, y1 = -L / 2, L / 2, -W / 2, W / 2
    if key == "corner":
        return [(x0, y0), (x1, y0), (x1, y1 - b), (x1 - a, y1 - b), (x1 - a, y1), (x0, y1)]
    if key == "slot":
        return [(x0, y0), (x1, y0), (x1, -a / 2), (x1 - b, -a / 2), (x1 - b, a / 2), (x1, a / 2), (x1, y1), (x0, y1)]
    if key == "tab":
        return [(x0, y0), (x1 - a, y0), (x1 - a, y0 + b), (x1, y0 + b), (x1, y1 - b), (x1 - a, y1 - b), (x1 - a, y1),
                (x0, y1)]
    raise ValueError(key)


class Camera(object):
    def __init__(self, pos, R, fovy, wh):
        self.pos = np.asarray(pos, float)
        self.R = np.asarray(R, float)          # columns: camera x, y, z in the world
        self.w, self.h = wh
        self.f = (self.h / 2.0) / math.tan(math.radians(fovy) / 2)

    def ray(self, u, v):
        """World direction through pixel centre (u, v) (arrays)."""
        x = (np.asarray(u, float) + 0.5 - self.w / 2) / self.f
        y = -(np.asarray(v, float) + 0.5 - self.h / 2) / self.f
        d = np.stack([x, y, -np.ones_like(x)], -1)
        return d @ self.R.T

    def to_plane(self, u, v, z):
        d = self.ray(u, v)
        s = (z - self.pos[2]) / d[..., 2]
        return self.pos + d * s[..., None]

    def project(self, P):
        P = np.atleast_2d(np.asarray(P, float))
        c = (P - self.pos) @ self.R
        u = self.f * c[:, 0] / (-c[:, 2]) + self.w / 2 - 0.5
        v = -self.f * c[:, 1] / (-c[:, 2]) + self.h / 2 - 0.5
        return np.stack([u, v], 1)


def site_R(yaw):
    return rot_z(yaw) @ R_DOWN


def wrist_camera(ee_pos, ee_yaw):
    Rs = site_R(ee_yaw)
    return Camera(np.asarray(ee_pos, float) + Rs @ WRIST_POS, Rs @ WRIST_R, WRIST_FOVY, WRIST_WH)


def inspect_camera():
    return Camera(INSPECT_POS, INSPECT_R, INSPECT_FOVY, INSPECT_WH)


# ---- segmentation ---------------------------------------------------------------------------------------------
def colour_masks(img):
    """Masks of the three part colour families (HSV thresholds; the scene itself is grey)."""
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    h, s, v = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    sat = (s > 55) & (v > 50)
    return {"blue": sat & (h >= 100) & (h <= 125), "green": sat & (h >= 45) & (h <= 85),
            "orange": sat & (h >= 5) & (h <= 25)}


def blobs(img, min_px=12, top_frac=0.72):
    """Connected colour components: list of (family, mask, stats). Within a component only the bright pixels are
    kept: the top face is lit from above, the side faces a camera sees at an angle are darker."""
    out = []
    val = img.max(axis=2).astype(float)
    for fam, m in colour_masks(img).items():
        m8 = m.astype(np.uint8)
        n, lab, stats, _c = cv2.connectedComponentsWithStats(m8, 8)
        for k in range(1, n):
            if stats[k, cv2.CC_STAT_AREA] >= min_px:
                mk = lab == k
                vmax = np.percentile(val[mk], 90)
                mk = mk & (val >= top_frac * vmax)
                out.append((fam, mk, stats[k]))
    return out


def dark_mask(img, thr=60):
    return img.max(axis=2) < thr


# ---- template fit on a plane ------------------------------------------------------------------------------------
def polygon_world(name, x, y, yaw, flipped, z):
    """The visible top outline of a part lying at (x, y, yaw) on plane z (top face at z), flipped = upside down."""
    pts = np.array(outline(name))
    if flipped:
        pts = pts * np.array([1.0, -1.0])        # turned over about its x axis: the top view is mirrored in y
    c, s = math.cos(yaw), math.sin(yaw)
    P = np.stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1], np.full(len(pts), z)], 1)
    return P


def render_mask(cam, P, shape):
    uv = cam.project(P)
    m = np.zeros(shape, np.uint8)
    cv2.fillPoly(m, [np.round(uv * 8).astype(np.int32)], 1, lineType=cv2.LINE_8, shift=3)
    return m.astype(bool)


def fill_holes(mask):
    m = mask.astype(np.uint8)
    h, w = m.shape
    ff = np.zeros((h + 2, w + 2), np.uint8)
    inv = (1 - m).copy()
    cv2.floodFill(inv, ff, (0, 0), 0)
    return mask | (inv > 0)


def fit_part(cam, mask, family, z_top, dark=None, grid_deg=3.0):
    """Best (variant, x, y, yaw, flipped, score) for a blob `mask` of colour `family` seen by `cam`, the part's top
    face at height z_top. Coarse: the blob's metric minimum-area rectangle; fine: IoU of rendered outlines."""
    raw = mask
    mask = fill_holes(mask)
    hole = mask & ~raw
    vs, us = np.nonzero(mask)
    P = cam.to_plane(us, vs, z_top)
    cx, cy = P[:, 0].mean(), P[:, 1].mean()
    hv, hu = np.nonzero(hole)
    mark = None
    if len(hv) >= 2:
        H = cam.to_plane(hu, hv, z_top)
        mark = (H[:, 0].mean(), H[:, 1].mean())
    pts = P[:, :2].astype(np.float32)
    (rx, ry), (rw, rh), ang = cv2.minAreaRect(pts * 1000.0)
    a0 = math.radians(ang) if rw >= rh else math.radians(ang + 90)
    best = None
    x0, x1 = max(0, us.min() - 4), min(mask.shape[1], us.max() + 5)
    y0, y1 = max(0, vs.min() - 4), min(mask.shape[0], vs.max() + 5)
    sub = mask[y0:y1, x0:x1]
    flips = (False,) if mark is not None else (True, False)
    for name in FAMILY[family]:
        for flipped in flips:
            bases = (a0, a0 + math.pi)
            if mark is not None:
                # the datum mark sits near the -x end: the part's x axis points away from it
                ang = math.atan2(cy - mark[1], cx - mark[0])
                bases = (a0,) if abs(wrap(a0 - ang)) < math.pi / 2 else (a0 + math.pi,)
            for base in bases:
                for dyaw in np.radians(np.arange(-9, 9.01, grid_deg)):
                    yaw = base + dyaw
                    # the outline's bounding-box centre is not the blob's centroid: shift by the outline centroid
                    poly = np.array(outline(name))
                    if flipped:
                        poly = poly * np.array([1.0, -1.0])
                    m = cv2.moments(poly.astype(np.float32) * 1000.0)
                    ox, oy = m["m10"] / m["m00"] / 1000.0, m["m01"] / m["m00"] / 1000.0
                    c, s = math.cos(yaw), math.sin(yaw)
                    x, y = cx - (c * ox - s * oy), cy - (s * ox + c * oy)
                    R = render_mask(cam, polygon_world(name, x, y, yaw, flipped, z_top), mask.shape)[y0:y1, x0:x1]
                    inter = np.logical_and(R, sub).sum()
                    union = np.logical_or(R, sub).sum()
                    iou = inter / max(1, union)
                    if best is None or iou > best[5]:
                        best = (name, x, y, wrap(yaw), flipped, iou)
    name, x, y, yaw, flipped, iou = best
    # refine position and yaw by a small local search on IoU
    step = [0.001, 0.001, math.radians(1.0)]
    cur = [x, y, yaw]
    for _ in range(3):
        for k in range(3):
            for sgn in (-1, 1):
                trial = list(cur)
                trial[k] += sgn * step[k]
                R = render_mask(cam, polygon_world(name, trial[0], trial[1], trial[2], flipped, z_top), mask.shape)[y0:y1, x0:x1]
                iou2 = np.logical_and(R, sub).sum() / max(1, np.logical_or(R, sub).sum())
                if iou2 > iou:
                    iou, cur = iou2, trial
        step = [s_ * 0.5 for s_ in step]
    return {"v": name, "x": cur[0], "y": cur[1], "yaw": wrap(cur[2]), "flipped": flipped, "iou": iou,
            "mark": mark is not None}
