"""Perception from the cell's two RGB-D cameras, with the published calibration (cell.json) and the manual's numbers.

Everything here works on decoded frames (RGB uint8, depth in metres) and returns positions in the cell's base frame.
"""

import base64
import math

import cv2
import numpy as np


def decode(image):
    rgb = cv2.cvtColor(cv2.imdecode(np.frombuffer(base64.b64decode(image["rgb"]), np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    dep = cv2.imdecode(np.frombuffer(base64.b64decode(image["depth"]), np.uint8), cv2.IMREAD_UNCHANGED).astype(np.float32)
    return rgb, dep * float(image.get("depth_unit_m", 1e-4))


class Camera(object):
    """A pinhole camera: K, and its pose in the base frame (4x4, camera axes x right, y down, z forward)."""

    def __init__(self, cfg, T):
        K = np.array(cfg["K"], dtype=float)
        self.fx, self.fy, self.cx, self.cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        self.W, self.H = cfg["width"], cfg["height"]
        self.T = np.asarray(T, dtype=float)

    def points(self, depth):
        """Every pixel's point in the base frame (H x W x 3); pixels without a reading are NaN."""
        H, W = depth.shape
        u, v = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
        d = depth.astype(np.float64)
        pc = np.stack([(u - self.cx) * d / self.fx, (v - self.cy) * d / self.fy, d], axis=-1)
        pw = pc @ self.T[:3, :3].T + self.T[:3, 3]
        pw[d <= 0] = np.nan
        return pw

    def point(self, u, v, d):
        pc = np.array([(u - self.cx) * d / self.fx, (v - self.cy) * d / self.fy, d])
        return self.T[:3, :3] @ pc + self.T[:3, 3]

    def project(self, p):
        """Pixel (u, v) and depth of a base-frame point."""
        pc = self.T[:3, :3].T @ (np.asarray(p, dtype=float) - self.T[:3, 3])
        return pc[0] * self.fx / pc[2] + self.cx, pc[1] * self.fy / pc[2] + self.cy, pc[2]


def fit_line(pts):
    """Total least squares line through 2D points: (centroid, unit direction)."""
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c, full_matrices=False)
    return c, vt[0]


def pack_pose(cam, depth, table_z, top_z, size=(0.62, 1.0)):
    """The pack's frame on the fixture from an overhead depth frame: the silhouette of its top (the lid or the tray's
    walls, `top_z`), its four sides fitted as lines in the base frame. Returns (x, y, yaw) of the pack frame's origin
    (under the centre of the tray) and the residual of the fit (m)."""
    P = cam.points(depth)
    z = P[..., 2]
    zz = np.nan_to_num(z, nan=-9.0) - (table_z + top_z)
    mask = (zz > -0.008) & (zz < 0.008)
    mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=4)
    if n < 2:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    blob = (lab == k).astype(np.uint8)
    cnts, _ = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = max(cnts, key=cv2.contourArea)[:, 0, :]
    # the boundary pixels' points in the base frame, at the height of the top (the silhouette's own depth)
    xy = []
    for u, v in c:
        p = P[v, u]
        if np.isfinite(p).all():
            xy.append(p[:2])
    xy = np.array(xy)
    (cx, cy), (w, h), ang = cv2.minAreaRect(xy.astype(np.float32) * 1000.0)
    cx, cy = cx / 1000.0, cy / 1000.0
    # the long side along y: choose the angle whose long axis is nearest to y
    yaw0 = math.radians(ang)
    best = None
    for k4 in range(4):
        yaw = yaw0 + k4 * math.pi / 2
        yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2
        if best is None or abs(yaw) < abs(best):
            best = yaw
    yaw = best
    # refine: each side as a line in the pack's provisional frame
    for _ in range(3):
        Rz = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        loc = (xy - [cx, cy]) @ Rz             # coordinates in the provisional pack frame
        hx, hy = size[0] / 2, size[1] / 2
        sides = []
        for axis, sign, half in ((0, 1, hx), (0, -1, hx), (1, 1, hy), (1, -1, hy)):
            other = 1 - axis
            sel = (np.abs(loc[:, axis] - sign * half) < 0.012) & (np.abs(loc[:, other]) < (hy if other else hx) - 0.03)
            if sel.sum() < 20:
                return None
            sides.append((axis, sign, loc[sel]))
        offs, angs = [], []
        for axis, sign, pts in sides:
            c0, d = fit_line(pts)
            # the side's direction against the provisional axis it should follow
            if axis == 0:               # a side at x = +-hx runs along y
                angs.append(-math.atan2(d[0], d[1]) if d[1] > 0 else -math.atan2(-d[0], -d[1]))
            else:                       # a side at y = +-hy runs along x
                angs.append(math.atan2(d[1], d[0]) if d[0] > 0 else math.atan2(-d[1], -d[0]))
            offs.append((axis, sign, c0[axis]))
        dyaw = float(np.median(angs))
        mx = 0.5 * (sum(o for a, s, o in offs if a == 0 and s > 0) + sum(o for a, s, o in offs if a == 0 and s < 0))
        my = 0.5 * (sum(o for a, s, o in offs if a == 1 and s > 0) + sum(o for a, s, o in offs if a == 1 and s < 0))
        cx, cy = np.array([cx, cy]) + Rz @ [mx, my]
        yaw += dyaw
    width = [o for a, s, o in offs]
    return float(cx), float(cy), float(yaw), {"extent_x": offs[0][2] - offs[1][2], "extent_y": offs[2][2] - offs[3][2]}


# ------------------------------------------------------------------------------------------------ the pack frame
class PackView(object):
    """An overhead frame seen in the pack's frame: every pixel's point in pack coordinates (x, y along the tray's
    short and long sides, z up from the fixture's surface)."""

    def __init__(self, cam, rgb, depth, pose, table_z):
        self.rgb, self.depth = rgb, depth
        self.x0, self.y0, self.yaw = pose
        self.table_z = table_z
        P = cam.points(depth)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = P[..., 0] - self.x0, P[..., 1] - self.y0
        self.px = c * dx + s * dy
        self.py = -s * dx + c * dy
        self.pz = P[..., 2] - table_z
        self.hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)

    def to_base(self, x, y, z):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([self.x0 + c * x - s * y, self.y0 + s * x + c * y, self.table_z + z])

    def disc(self, x, y, r):
        return np.nan_to_num(np.hypot(self.px - x, self.py - y), nan=9.0) < r

    def band(self, a0, a1, b0, b1):
        return (np.nan_to_num(self.px, nan=9.0) >= a0) & (self.px <= a1) & (np.nan_to_num(self.py, nan=9.0) >= b0) & (self.py <= b1)


def rib_scores(view, manual, lid_z):
    """How well each variant's rib pattern matches: the fraction of samples along each of its ribs that stand 2 to 6
    mm above the lid, against the same along the ribs of the other variants that it does not have."""
    lines = {}
    for v, d in manual["variants"].items():
        for r in d["ribs"]:
            lines[(r["along"], round(r["value"], 3))] = True
    h = view.pz - lid_z
    raised = {}
    for (along, val) in lines:
        if along == "y":            # along y at x = val: sample y in [-0.42, 0.42]
            sel = view.band(val - 0.004, val + 0.004, -0.42, 0.42)
        else:                       # along x at y = val
            sel = view.band(-0.24, 0.24, val - 0.004, val + 0.004)
        hh = h[sel]
        hh = hh[np.isfinite(hh)]
        raised[(along, val)] = float(np.mean((hh > 0.002) & (hh < 0.0065))) if hh.size else 0.0
    scores = {}
    for v, d in manual["variants"].items():
        mine = [(r["along"], round(r["value"], 3)) for r in d["ribs"]]
        other = [k for k in lines if k not in mine]
        scores[v] = float(np.mean([raised[k] for k in mine]) - (np.mean([raised[k] for k in other]) if other else 0.0))
    return scores, raised


def head_at(view, x, y, seat_z, r_search=0.009, r_ring=(0.012, 0.016)):
    """What stands at a fastener's place in an overhead frame: ('head', x, y, height, top_z) with the head's height
    above its seat and the centre of its top, ('hole', x, y) for an empty hole, ('covered', ...) when something much
    taller than a head lies there, or ('none',)."""
    sel = view.disc(x, y, r_search)
    ring = view.disc(x, y, r_ring[1]) & ~view.disc(x, y, r_ring[0])
    hr = view.pz[ring]
    hr = hr[np.isfinite(hr)]
    seat = float(np.median(hr)) if hr.size > 10 and abs(float(np.median(hr)) - seat_z) < 0.004 else seat_z
    h = view.pz - seat
    hs = h[sel]
    if not np.isfinite(hs).any():
        return ("none",)
    hmax = float(np.nanmax(hs))
    if hmax > 0.0095:
        return ("covered", hmax)
    if hmax > 0.0028:
        top = sel & (h > 0.5 * hmax)
        hh = h[top]
        height = float(np.percentile(hh, 80))
        return ("head", float(np.mean(view.px[top])), float(np.mean(view.py[top])), height, seat + height)
    low = sel & (h < -0.003)
    if low.sum() >= 4:
        return ("hole", float(np.mean(view.px[low])), float(np.mean(view.py[low])))
    return ("none",)


def head_kind(thread, height):
    """The head type from its height above the seat (manual, section 5): M6 S10 6 mm, T30 4.5 mm; M8 S13 8 mm,
    T40 6 mm."""
    if thread == "M6":
        return "S10" if height > 0.0053 else "T30"
    return "S13" if height > 0.0071 else "T40"


def blue_mask(view, region):
    """The harness' blue cable: saturated blue pixels inside `region` (a boolean mask)."""
    hh, ss, vv = view.hsv[..., 0].astype(int), view.hsv[..., 1].astype(int), view.hsv[..., 2].astype(int)
    m = (hh >= 95) & (hh <= 130) & (ss > 90) & (vv > 40) & region
    m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)) > 0
    return m


def harness_grasp(view, mask, min_px=150):
    """A grasp on the cable: a point along it away from its ends where it runs straight, the direction it runs in,
    and the height of its centre (pack frame). None when no cable is seen."""
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    if n < 2:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[k, cv2.CC_STAT_AREA] < min_px:
        return None
    sel = lab == k
    pts = np.stack([view.px[sel], view.py[sel], view.pz[sel]], axis=1)
    pts = pts[np.isfinite(pts).all(axis=1)]
    if len(pts) < min_px:
        return None
    # order the cable's points along its main direction, then look at windows along it
    c = pts[:, :2].mean(axis=0)
    _, _, vt = np.linalg.svd(pts[:, :2] - c, full_matrices=False)
    s = (pts[:, :2] - c) @ vt[0]
    lo, hi = np.percentile(s, 2), np.percentile(s, 98)
    cands = []
    for f in np.linspace(0.25, 0.75, 11):
        s0 = lo + f * (hi - lo)
        w = np.abs(s - s0) < 0.03
        if w.sum() < 30:
            continue
        q = pts[w]
        qc = q[:, :2].mean(axis=0)
        _, sv, vt2 = np.linalg.svd(q[:, :2] - qc, full_matrices=False)
        straight = sv[1] / max(sv[0], 1e-9)
        zspread = float(np.percentile(q[:, 2], 90) - np.percentile(q[:, 2], 10))
        score = straight + 2.0 * zspread / 0.012 + 0.3 * abs(f - 0.5)
        cands.append((score, qc, vt2[0], float(np.percentile(q[:, 2], 95))))
    if not cands:
        return None
    cands.sort(key=lambda c: c[0])
    out = []
    for _, qc, d, ztop in cands:
        # alternatives at least 4 cm apart along the cable
        if all(math.hypot(qc[0] - o["x"], qc[1] - o["y"]) > 0.04 for o in out):
            out.append({"x": float(qc[0]), "y": float(qc[1]), "dir": [float(d[0]), float(d[1])], "z": ztop - 0.006})
    best = dict(out[0], pixels=int(sel.sum()), extent=float(hi - lo), alternatives=out[1:3])
    return best


def wrist_head(cam, depth, expect, seat_z, r_search=0.0075):
    """A fastener's head in a wrist frame: the points of its top face (within 1.2 mm of its highest point) near the
    expected place, their centre (base frame) and the head's height above `seat_z` (base frame). None when nothing
    stands there higher than 2.5 mm."""
    P = cam.points(depth)
    sel = np.nan_to_num(np.hypot(P[..., 0] - expect[0], P[..., 1] - expect[1]), nan=9.0) < r_search
    h = P[..., 2] - seat_z
    sel &= np.nan_to_num(h, nan=-9.0) > -0.004
    sel &= np.nan_to_num(h, nan=9.0) < 0.012
    if sel.sum() < 20:
        return None
    hs = h[sel]
    hmax = float(np.percentile(hs, 99.5))
    if hmax < 0.0025:
        low = sel & (h < -0.003)
        return {"hole": True, "n_low": int(low.sum())}
    top = sel & (h > hmax - 0.0012)
    # re-centre once on the top face found, so that a head near the window's edge is taken whole
    c = P[top][:, :2].mean(axis=0)
    sel2 = np.nan_to_num(np.hypot(P[..., 0] - c[0], P[..., 1] - c[1]), nan=9.0) < r_search
    sel2 &= (np.nan_to_num(h, nan=-9.0) > -0.004) & (np.nan_to_num(h, nan=9.0) < 0.012)
    hmax = float(np.percentile(h[sel2], 99.5))
    top = sel2 & (h > hmax - 0.0012)
    p = P[top]
    return {"hole": False, "x": float(p[:, 0].mean()), "y": float(p[:, 1].mean()), "z": float(seat_z + hmax),
            "height": hmax, "n": int(top.sum())}
