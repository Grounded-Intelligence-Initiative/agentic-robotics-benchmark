"""Perception from the cell's two RGB-D cameras, with the published calibration (cell.json) and the manual's numbers.

Everything here works on decoded frames (RGB uint8, depth in metres) and returns positions in the cell's base frame.
"""

import base64
import math

import cv2
import numpy as np


# the white balance of this pack's hall light: the plates' white under the nominal light (measured on development
# frames) over the same white now (set by the controller after calibration); colours are read balanced
WHITE_REF = np.array([178.0, 180.0, 180.0])
WB = np.ones(3)


def decode(image):
    rgb = cv2.cvtColor(cv2.imdecode(np.frombuffer(base64.b64decode(image["rgb"]), np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    if np.any(WB != 1.0):
        rgb = np.clip(np.round(rgb.astype(np.float32) * WB.astype(np.float32)), 0, 255).astype(np.uint8)
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
    c = max(cnts, key=cv2.contourArea)
    # the silhouette's convex hull: grime that loses the depth at the lid's edge, or a cable lying over it, cuts
    # notches into the outline that the hull bridges; its edges, densified, are the boundary
    hull = cv2.convexHull(c)[:, 0, :].astype(np.float64)
    pts = []
    for a, b in zip(hull, np.roll(hull, -1, axis=0)):
        n = max(2, int(np.ceil(np.linalg.norm(b - a))))
        pts.append(a + (b - a) * np.linspace(0.0, 1.0, n, endpoint=False)[:, None])
    uv = np.concatenate(pts)
    # each boundary pixel's ray meets the plane of the top (its height is known): no depth needed on the edge
    zt = table_z + top_z
    rays = np.stack([(uv[:, 0] - cam.cx) / cam.fx, (uv[:, 1] - cam.cy) / cam.fy, np.ones(len(uv))], axis=1) @ cam.T[:3, :3].T
    s = (zt - cam.T[2, 3]) / rays[:, 2]
    xy = (cam.T[:3, 3][None, :] + s[:, None] * rays)[:, :2]
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
            if sel.sum() < 20:              # a first guess pulled off by something on the edge: a wider window
                sel = (np.abs(loc[:, axis] - sign * half) < 0.03) & (np.abs(loc[:, other]) < (hy if other else hx) - 0.03)
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
        self.luma = rgb[..., 0] * 0.299 + rgb[..., 1] * 0.587 + rgb[..., 2] * 0.114

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


def head_at(view, x, y, seat_z, r_core=0.0045, r_search=0.009, r_ring=(0.012, 0.016)):
    """What stands at a fastener's place in an overhead frame (depth averaged over several frames: one frame's noise
    is 2.5 mm here): ('head', x, y, height, top_z) with a rough height above the seat (not enough to tell the head's
    type) and the centre of the bump, ('hole', x, y) for an empty hole, ('covered', ...) when something much taller
    than a head lies there, or ('none',). Robust statistics only (medians), and the colour where the sensor has no
    reading: a dark hole returns none, a bright plated head often loses part of its readings."""
    ring = view.disc(x, y, r_ring[1]) & ~view.disc(x, y, r_ring[0])
    hr = view.pz[ring]
    hr = hr[np.isfinite(hr)]
    seat = float(np.median(hr)) if hr.size > 10 and abs(float(np.median(hr)) - seat_z) < 0.004 else seat_z
    h = view.pz - seat
    core = view.disc(x, y, r_core)
    hc = h[core]
    valid = np.isfinite(hc)
    luma = view.luma[core]
    if valid.sum() >= max(4, 0.3 * core.sum()):
        hm = float(np.median(hc[valid]))
    else:
        # mostly no reading: dark (a hole, a black part) or bright (a plated head's highlights)
        hm = None
    if hm is None:
        if luma.size and float(np.median(luma)) < 40:
            return ("hole", float(x), float(y))
        if luma.size and float(np.median(luma)) > 120:
            hm = 0.005
        else:
            return ("none",)
    if hm > 0.0105:
        return ("covered", hm)
    if hm > 0.0025:
        sel = view.disc(x, y, r_search) & (np.nan_to_num(h, nan=-9.0) > 0.5 * hm)
        if sel.sum() >= 4:
            cx, cy = float(np.mean(view.px[sel])), float(np.mean(view.py[sel]))
        else:
            cx, cy = float(x), float(y)
        return ("head", cx, cy, hm, seat + hm)
    if hm < -0.0015 or (luma.size and float(np.median(luma)) < 40):
        return ("hole", float(x), float(y))
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


def wrist_head(cam, depth, expect, seat_z, r_search=0.0085, rgb=None):
    """A fastener's head in a wrist frame (depth noise about 0.3 mm here; a plated head loses part of its top face's
    readings to its highlights): the head is the region near the expected place standing more than 2.5 mm above
    `seat_z`; its outline's convex hull (which closes the gaps the lost readings leave) gives the centre, projected
    onto the plane of the head's top, and the median of its top face gives the height. Returns {"hole": False, x, y,
    z, height, n (hull area in pixels), n_full (the area a whole head of that height would have)}, {"hole": True}
    when nothing stands there and the place is dark or low, or None."""
    P = cam.points(depth)
    rr = np.nan_to_num(np.hypot(P[..., 0] - expect[0], P[..., 1] - expect[1]), nan=9.0)
    h = P[..., 2] - seat_z
    win = (rr < r_search) & (np.nan_to_num(h, nan=-9.0) > -0.004) & (np.nan_to_num(h, nan=9.0) < 0.012)
    if win.sum() < 20:
        return None
    up = win & (np.nan_to_num(h, nan=-9.0) > 0.0025)
    if up.sum() < 30:
        return {"hole": True, "n_low": int((win & (np.nan_to_num(h, nan=9.0) < -0.002)).sum())}
    m = up.astype(np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)), connectivity=8)
    if n < 2:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    blob = (lab == k).astype(np.uint8)
    cnts, _ = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    hull = cv2.convexHull(max(cnts, key=cv2.contourArea))
    area = float(cv2.contourArea(hull))
    mo = cv2.moments(hull)
    if mo["m00"] <= 0:
        return None
    u, v = mo["m10"] / mo["m00"], mo["m01"] / mo["m00"]
    hv = h[(blob > 0) & np.isfinite(h)]
    height = float(np.median(hv[hv > np.median(hv) - 0.0015]))
    # the hull's centre is a pixel: its ray meets the plane of the head's top
    ray = cam.T[:3, :3] @ np.array([(u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, 1.0])
    o = cam.T[:3, 3]
    z = seat_z + height
    s = (z - o[2]) / ray[2]
    c = o + s * ray
    dist = float(np.linalg.norm(c - o))
    return {"hole": False, "x": float(c[0]), "y": float(c[1]), "z": float(z), "height": height, "n": int(area),
            "px_per_m": cam.fx / dist}


# the rim pixels of a head's top lie a little inside its outline (the depth's smoothing), measured on development
# frames with free-radius fits
RIM_INSET = 0.00015


def fit_circle(xy, r=None, iters=8):
    """A circle through 2D points: the algebraic fit, then (with a known radius r) the centre that brings every point
    to r from it, by Gauss-Newton; a robust reweighting drops points far off. Returns (centre, radius, rms)."""
    x, y = xy[:, 0], xy[:, 1]
    A = np.stack([x, y, np.ones_like(x)], axis=1)
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    c = np.array([sol[0] / 2, sol[1] / 2])
    rad = math.sqrt(max(sol[2] + c @ c, 1e-12))
    w = np.ones(len(xy))
    for _ in range(iters):
        d = xy - c
        dist = np.linalg.norm(d, axis=1) + 1e-12
        rr = rad if r is None else r
        res = dist - rr
        s = 1.4826 * float(np.median(np.abs(res))) + 1e-5
        w = 1.0 / (1.0 + (res / (2.5 * s)) ** 2)
        J = -d / dist[:, None]
        if r is None:
            J = np.concatenate([J, -np.ones((len(xy), 1))], axis=1)
        H = J.T @ (J * w[:, None])
        g = J.T @ (w * res)
        step = -np.linalg.solve(H + 1e-12 * np.eye(H.shape[0]), g)
        c = c + step[:2]
        if r is None:
            rad = rad + step[2]
    res = np.linalg.norm(xy - c, axis=1) - (rad if r is None else r)
    return c, (rad if r is None else r), float(np.sqrt(np.average(res ** 2, weights=w)))


def fit_hexagon(xy, apothem, c0):
    """A regular hexagon with a known apothem (half the width across flats) through 2D points on its boundary: its
    centre and turn. For each turn on a grid, each point is given to the side it lies against (the largest projection
    on the sides' normals) and the centre follows by least squares; the turn with the least residual wins. Returns
    (centre, turn, rms)."""
    best = None
    for k in range(30):
        phi = math.radians(2.0 * k)
        N = np.array([[math.cos(phi + j * math.pi / 3), math.sin(phi + j * math.pi / 3)] for j in range(6)])
        c = np.array(c0, dtype=float)
        for _ in range(6):
            side = np.argmax((xy - c) @ N.T, axis=1)
            A = N[side]
            b = np.sum(xy * A, axis=1) - apothem
            c, *_ = np.linalg.lstsq(A, b, rcond=None)
        res = np.max((xy - c) @ N.T, axis=1) - apothem
        s = 1.4826 * float(np.median(np.abs(res))) + 1e-5
        keep = np.abs(res) < 3 * s
        if keep.sum() >= 10:
            side = np.argmax((xy[keep] - c) @ N.T, axis=1)
            A = N[side]
            b = np.sum(xy[keep] * A, axis=1) - apothem
            c, *_ = np.linalg.lstsq(A, b, rcond=None)
            res = np.max((xy[keep] - c) @ N.T, axis=1) - apothem
        rms = float(np.sqrt(np.mean(res ** 2)))
        if best is None or rms < best[2]:
            best = (c, phi, rms)
    return best


def wrist_head_fit(cam, depth, expect, seat_z, thread, r_search=0.009):
    """A fastener's head in a wrist frame when part of it may be under a dab of sealant (service manual, section 7):
    the head's flat top is the dominant height plateau above the seat; its rim is where that plateau drops to the seat
    (under sealant there is no such drop, the dome goes on); a circle fitted to the rim gives the centre (a pan head's
    known radius; a hexagon's rim lies between its half width across flats and across corners, so its radius is
    fitted). Returns the dict of `wrist_head` with "fit": True, or None when the rim is too short to fit (the caller
    then uses `wrist_head`)."""
    P = cam.points(depth)
    rr = np.nan_to_num(np.hypot(P[..., 0] - expect[0], P[..., 1] - expect[1]), nan=9.0)
    h = P[..., 2] - seat_z
    hz = np.nan_to_num(h, nan=-9.0)
    win = (rr < r_search) & (hz > -0.004) & (hz < 0.014)
    up = win & (hz > 0.0025)
    if up.sum() < 30:
        return None
    hv = h[up]
    bins = np.arange(0.0025, 0.0125, 0.00025)
    hist, edges = np.histogram(hv, bins=bins)
    hist = np.convolve(hist, [1, 2, 3, 2, 1], mode="same")
    H = float(edges[int(np.argmax(hist))] + 0.000125)
    # refine the plateau's height: the median of the pixels near the peak
    near = hv[np.abs(hv - H) < 0.0007]
    if near.size < 20:
        return None
    H = float(np.median(near))
    top = win & (np.abs(hz - H) < 0.0008)
    low = (rr < r_search + 0.004) & np.isfinite(h) & (hz < H - 0.002) & (hz > -0.004)
    k3 = np.ones((3, 3), np.uint8)
    rim = top & (cv2.dilate(low.astype(np.uint8), k3) > 0)
    # the rim of the head under the camera: keep the connected part of the plateau nearest the expected place
    n, lab, stats, cent = cv2.connectedComponentsWithStats(cv2.morphologyEx(top.astype(np.uint8), cv2.MORPH_CLOSE, k3), connectivity=8)
    if n < 2:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    plateau = lab == k
    rim &= cv2.dilate(plateau.astype(np.uint8), k3) > 0
    pts = P[rim][:, :2]
    if len(pts) < 25:
        return None
    kind = head_kind(thread, H)
    if kind in ("S10", "S13"):
        c0 = pts.mean(axis=0)
        c, _, rms = fit_hexagon(pts, {"S10": 0.005, "S13": 0.0065}[kind] - RIM_INSET, c0)
        rad = None
        radius = 0.0
    else:
        radius = {"T30": 0.006, "T40": 0.008}[kind] - RIM_INSET
        c, rad, rms = fit_circle(pts, r=radius)
    # how much of the circumference the rim covers (sealant hides the rest)
    ang = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
    cover = len(np.unique(np.floor((ang + math.pi) / (2 * math.pi) * 36).astype(int))) / 36.0
    if cover < 0.4 or rms > 0.0006:
        return None
    pz = seat_z + H
    return {"hole": False, "x": float(c[0]), "y": float(c[1]), "z": float(pz), "height": H, "fit": True, "cover": cover,
            "rms": rms, "radius": None if rad is None else float(rad), "n_rim": int(len(pts))}


# ------------------------------------------------------------------ round 2 (2026-11-26): foam pads, swollen modules
def pad_at(cam, depth, xy, seat_z, r=(0.010, 0.012), above=0.009):
    """Whether a foam pad covers the fastener whose nominal place is `xy` (manual, section 7: a pad 40 to 60 mm
    across, its centre within 8 mm of the place, so it covers a circle of 12 mm around it; 10 to 14 mm high, more
    than any head or dab of sealant). Returns (fraction of the ring's readings standing `above` the seat, count)."""
    P = cam.points(depth)
    rr = np.hypot(P[..., 0] - xy[0], P[..., 1] - xy[1])
    ring = np.isfinite(rr) & (rr >= r[0]) & (rr <= r[1])
    h = P[..., 2][ring] - seat_z
    if h.size < 30:
        return 0.0, int(h.size)
    return float(np.mean(h > above)), int(h.size)


def module_bump(cams, pose, centre, top_z, plate=(0.28, 0.11), rim=0.005, a=(0.09, 0.12), b=(0.035, 0.05), dx=0.015):
    """The height of a module's top-plate bump (manual, section 7) from wrist frames [(camera, depth)]: the plate's
    points in the module's own frame (`pose`: the pack's x, y, yaw; `centre`: the module's centre in the pack frame;
    `top_z`: the plate's nominal height in the base frame), a plane fitted to its rim (the outer `rim`), and the bump
    h (1 - rho^2)^2 over an ellipse of the stated sizes fitted to what stands above the plane (least squares over a
    grid of the ellipse's semi-axes and offset). Returns (h, rim points, plate points) or None."""
    X, Y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for cam, dep in cams:
        P = cam.points(dep).reshape(-1, 3)
        P = P[np.all(np.isfinite(P), axis=1)]
        px, py = P[:, 0] - X, P[:, 1] - Y
        u = c * px + s * py - centre[0]
        v = -s * px + c * py - centre[1]
        # the plate's outer 2 mm left out: the depth there is pulled down by the plate's 2 mm step (the sensor's lateral
        # smoothing), which on the development packs read every healthy module 0.02 to 0.14 mm too high
        keep = (np.abs(u) < plate[0] / 2 - 0.002) & (np.abs(v) < plate[1] / 2 - 0.002) & (np.abs(P[:, 2] - top_z) < 0.005)
        pts.append(np.stack([u[keep], v[keep], P[keep, 2]], axis=1))
    if not pts:
        return None
    Q = np.concatenate(pts)
    edge = (np.abs(Q[:, 0]) > plate[0] / 2 - rim) | (np.abs(Q[:, 1]) > plate[1] / 2 - rim)
    R = Q[edge]
    if len(R) < 200 or len(Q) < 2000:
        return None
    A = np.stack([np.ones(len(R)), R[:, 0], R[:, 1]], axis=1)
    coef = None
    sel = np.ones(len(R), bool)
    for _ in range(3):                      # a plane through the rim, readings far off it left out
        coef = np.linalg.lstsq(A[sel], R[sel, 2], rcond=None)[0]
        res = R[:, 2] - A @ coef
        sel = np.abs(res - np.median(res[sel])) < 3.0 * 1.4826 * np.median(np.abs(res[sel] - np.median(res[sel]))) + 1e-5
    r = Q[:, 2] - (coef[0] + coef[1] * Q[:, 0] + coef[2] * Q[:, 1])
    if len(Q) > 10000:                      # enough for a hundredth of a millimetre; keeps the fit fast
        pick = np.random.default_rng(0).choice(len(Q), 10000, replace=False)
        Q, r = Q[pick], r[pick]
    best = None
    for aa in np.linspace(a[0], a[1], 5):
        for bb in np.linspace(b[0], b[1], 5):
            for d0 in np.linspace(-dx, dx, 5):
                rho2 = ((Q[:, 0] - d0) / aa) ** 2 + (Q[:, 1] / bb) ** 2
                g = np.where(rho2 < 1.0, (1.0 - rho2) ** 2, 0.0)
                gg = float(g @ g)
                if gg <= 0:
                    continue
                h = float(g @ r) / gg
                sse = float(((r - h * g) ** 2).sum())
                if best is None or sse < best[0]:
                    best = (sse, h)
    return best[1], int(len(R)), int(len(Q))
