#!/usr/bin/env python3
"""Reference controller for object_finding_and_organizing_on_messy_desk.

Works from the observation only (two static RGB-D cameras, the hand camera, proprioception,
the request text). Per decision:

  depth back-projection (both static cameras) -> bins located from their rim band (pose
  varies per episode) -> desk / bin walls / bin floors / robot removed -> voxel
  connected-component clustering -> per-cluster colour histogram + sorted extents vs the
  static references -> hover and confirm with the hand camera (dense cloud, its own
  references) -> grasp yaw chosen so that the 0.2 m wide hand clears every bin wall and the
  fingers close across a dimension under the 8 cm opening -> top-down pinch, carry, drop near
  the bin's centre.

Requests: deliver the named object to the red bin (searching loose objects first, then
unpacking its class bin, then the other bins onto the desk) and mark 1 after the hand camera
confirms the drop; mark 2 only once every bin is empty and every loose object has been seen.

Every call recomputes its action from the observation it is given (closed-loop servo), so a
repeated query of the same step with perturbed proprioception answers differently.
"""
import base64
import io
import json
import math
import os
import sys
from collections import deque

import numpy as np
from PIL import Image

# ----------------------------------------------------------------------------------------
# Task geometry (task_card.md)
# ----------------------------------------------------------------------------------------
BIN_X = 0.44
BIN_YS = {"gray": -0.45, "lightblue": -0.15, "blue": 0.15, "red": 0.45}
BIN_OUTER = (0.326, 0.247, 0.190)          # outer footprint x (long), y (short), wall height
BIN_INNER_HALF = (0.140, 0.109)
BIN_FLOOR_Z, BIN_RIM_Z = 0.0035, 0.191
BIN_OF_CLASS = {"A": "gray", "B": "lightblue", "C": "blue"}
CLASS_OF_BIN = {v: k for k, v in BIN_OF_CLASS.items()}
CLASSES = {
    "A": ["cracker box", "sugar box", "pudding box", "gelatin box", "wood block", "foam brick"],
    "B": ["master chef can", "tomato soup can", "tuna fish can", "potted meat can", "mustard bottle", "bleach cleanser"],
    "C": ["banana", "bowl", "mug", "power drill", "scissors", "large marker", "large clamp", "extra large clamp", "pitcher base"],
}
CLASS_OF = {m: c for c, ms in CLASSES.items() for m in ms}
DESK_X, DESK_Y = (0.15, 0.78), (-0.62, 0.62)
REACH, BASE_CLEAR = 0.80, 0.22
IMG = 320                                   # task cameras are IMG x IMG; focal scales with the width
F_PX, C_PX = 171.03317 * IMG / 224.0, IMG / 2.0
CAMS = {
    "agentview": {"f": F_PX, "c": C_PX,
                  "pose": [1.35, 0.0, 0.95, 0.0, -0.375099, 0.0, 0.926985]},
    "left_view": {"f": F_PX, "c": C_PX,
                  "pose": [0.45, -1.15, 0.75, 0.6808448119669411, -0.19091972663321244, 0.19091972663321244, 0.6808448119669411]},
}


def intrinsics(w):
    """Focal and principal point for a w x w frame of the task cameras."""
    return F_PX * w / IMG, w / 2.0
POS_STEP_MAX, ROT_STEP_MAX = 0.05, 0.25
GRIPPER_OPEN_W = 0.08

# hand geometry (Franka hand, fingers open): half width across the fingers, half thickness
HAND_HALF_W, HAND_HALF_T = 0.1025, 0.0315      # measured on the Franka USD (body 0.205 x 0.063)
GRASP_MAX_WIDTH = 0.072

# controller tunables
OBSERVE_POS = (0.30, 0.0, 0.55)
TRAVEL_Z = 0.32
VOXEL = 0.012
Z_ROBOT = 0.30
GRASP_BELOW_TOP = 0.03
GRASP_MIN_ABOVE_FLOOR = 0.032
DROP_ABOVE_RIM = 0.035
HOVER_ABOVE_TOP = 0.16
LOG = os.environ.get("MESSY_DESK_SOLVER_LOG", "")


def log(*a):
    print("[solver]", *a, file=sys.stderr, flush=True)


# ----------------------------------------------------------------------------------------
# Small maths
# ----------------------------------------------------------------------------------------
def quat_to_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def wrap_half(a):
    """Wrap to (-pi/2, pi/2]: a gripper yaw is only defined modulo pi."""
    return (a + math.pi / 2) % math.pi - math.pi / 2


def rotvec(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    ang = math.acos(c)
    if ang < 1e-6:
        return np.zeros(3)
    if ang > math.pi - 1e-3:
        d = np.sqrt(np.maximum(np.diag(R) + 1.0, 0.0) / 2.0)
        return d / max(np.linalg.norm(d), 1e-9) * ang
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * math.sin(ang))
    return v * ang


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# ----------------------------------------------------------------------------------------
# Bins (pose varies per episode: centre x, y and yaw of the long side)
# ----------------------------------------------------------------------------------------
def nominal_bins():
    return {n: (BIN_X, y, 0.0) for n, y in BIN_YS.items()}


def bin_local(b, x, y):
    """Desk xy -> coordinates in bin b's frame (x along its long side). Vectorised."""
    bx, by, yaw = b
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = x - bx, y - by
    return c * dx + s * dy, -s * dx + c * dy


def bin_point(b, lx, ly):
    bx, by, yaw = b
    c, s = math.cos(yaw), math.sin(yaw)
    return bx + c * lx - s * ly, by + s * lx + c * ly


def bin_of_xy(bins, x, y, shrink=0.0):
    for name, b in bins.items():
        lx, ly = bin_local(b, x, y)
        if abs(lx) <= BIN_INNER_HALF[0] - shrink and abs(ly) <= BIN_INNER_HALF[1] - shrink:
            return name
    return None


def hand_corners(x, y, yaw, margin=0.0):
    """Corners + edge midpoints of the open hand's footprint (finger axis = home +y turned by yaw)."""
    u = np.array([-math.sin(yaw), math.cos(yaw)])        # across the fingers
    v = np.array([math.cos(yaw), math.sin(yaw)])          # hand thickness
    hw, ht = HAND_HALF_W + margin, HAND_HALF_T + margin
    pts = []
    for a in (-1.0, -0.5, 0.0, 0.5, 1.0):
        for bb in (-1.0, 0.0, 1.0):
            pts.append((x + a * hw * u[0] + bb * ht * v[0], y + a * hw * u[1] + bb * ht * v[1]))
    return pts


def hand_fits(bins, x, y, yaw, inside=None):
    """Open hand lowered at (x, y) with `yaw`: outside every bin's outer footprint, or (when
    `inside` names a bin) entirely within that bin's interior."""
    pts = hand_corners(x, y, yaw, 0.008)
    ox, oy = BIN_OUTER[0] / 2, BIN_OUTER[1] / 2
    ix, iy = BIN_INNER_HALF
    for name, b in bins.items():
        for px, py in pts:
            lx, ly = bin_local(b, px, py)
            if name == inside:
                if abs(lx) > ix - 0.006 or abs(ly) > iy - 0.006:
                    return False
            elif abs(lx) <= ox + 0.004 and abs(ly) <= oy + 0.004:
                return False
    return True


# ----------------------------------------------------------------------------------------
# Perception
# ----------------------------------------------------------------------------------------
def decode_png(b64, mode=None):
    im = Image.open(io.BytesIO(base64.b64decode(b64)))
    if mode:
        im = im.convert(mode)
    return np.array(im)


def backproject(depth_mm, rgb, cam, max_depth=3.0):
    z = depth_mm.astype(np.float32) / 1000.0
    h, w = z.shape
    f, c = intrinsics(w)
    v, u = np.mgrid[0:h, 0:w]
    valid = (z > 0.05) & (z < max_depth)
    pts = np.stack([z[valid], -(u[valid] - c) / f * z[valid], -(v[valid] - c) / f * z[valid]], 1)
    pose = cam["pose"]
    world = pts @ quat_to_mat(pose[3:7]).T + np.asarray(pose[:3])
    return world, rgb[valid].astype(np.float32) / 255.0


def backproject_rt(depth_mm, rgb, f, c, R, t, max_depth=1.5):
    z = depth_mm.astype(np.float32) / 1000.0
    h, w = z.shape
    f, c = intrinsics(w)
    v, u = np.mgrid[0:h, 0:w]
    valid = (z > 0.02) & (z < max_depth)
    pts = np.stack([z[valid], -(u[valid] - c) / f * z[valid], -(v[valid] - c) / f * z[valid]], 1)
    return pts @ R.T + np.asarray(t), rgb[valid].astype(np.float32) / 255.0


def label_components(idx):
    """26-connectivity labels for integer voxel coordinates (N,3) -> labels (N,) 0-based, count."""
    keys = {}
    for i, v in enumerate(map(tuple, idx)):
        keys.setdefault(v, []).append(i)
    labels = np.full(len(idx), -1, int)
    nb = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1) if (a, b, c) != (0, 0, 0)]
    comp = 0
    for v in list(keys):
        if labels[keys[v][0]] >= 0:
            continue
        dq = deque([v])
        labels[keys[v]] = comp
        while dq:
            cur = dq.popleft()
            for d in nb:
                n = (cur[0] + d[0], cur[1] + d[1], cur[2] + d[2])
                lst = keys.get(n)
                if lst is not None and labels[lst[0]] < 0:
                    labels[lst] = comp
                    dq.append(n)
        comp += 1
    return labels, comp


def fit_bin(p, cx, cy, yaw0=0.0):
    """Refine one bin's pose from rim-band points near its nominal centre: coarse-to-fine grid
    search over (x, y, yaw) maximising the number of points on the outline of the known outer
    footprint. Robust to partial rings (a bin seen from one side only)."""
    ox, oy = BIN_OUTER[0] / 2, BIN_OUTER[1] / 2
    q = p[np.hypot(p[:, 0] - cx, p[:, 1] - cy) < 0.32]
    if len(q) < 30:
        return None
    qx, qy = q[:, 0], q[:, 1]

    def score(x, y, yaw):
        c, sn = math.cos(yaw), math.sin(yaw)
        lx = (qx - x) * c + (qy - y) * sn
        ly = -(qx - x) * sn + (qy - y) * c
        on_long = (np.abs(np.abs(ly) - oy) < 0.02) & (np.abs(lx) < ox + 0.02)
        on_short = (np.abs(np.abs(lx) - ox) < 0.02) & (np.abs(ly) < oy + 0.02)
        return int((on_long | on_short).sum())

    best = (-1, cx, cy, yaw0)
    for yaw in yaw0 + np.radians(np.arange(-12, 12.1, 2.0)):
        for dx in np.arange(-0.06, 0.061, 0.02):
            for dy in np.arange(-0.03, 0.031, 0.015):
                sc = score(cx + dx, cy + dy, yaw)
                if sc > best[0]:
                    best = (sc, cx + dx, cy + dy, yaw)
    _, bx, by, byaw = best
    for yaw in byaw + np.radians(np.arange(-2.5, 2.51, 0.5)):
        for dx in np.arange(-0.02, 0.021, 0.005):
            for dy in np.arange(-0.015, 0.0151, 0.005):
                sc = score(bx + dx, by + dy, yaw)
                if sc > best[0]:
                    best = (sc, bx + dx, by + dy, yaw)
    return best


def detect_bins(pts, col):
    """Bin poses from the rim band of the fused cloud (the upper walls, above most objects):
    each bin is fitted around its nominal pose with the known footprint. A bin with too few
    band points keeps its nominal pose."""
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    band = (z > BIN_RIM_Z - 0.07) & (z < BIN_RIM_Z + 0.02) & (x > DESK_X[0]) & (x < 0.80) & (np.abs(y) < 0.65)
    bins = nominal_bins()
    fits = []
    if band.sum() < 40:
        return bins, fits
    p = pts[band][:, :2]
    for name, (bx, by, _) in list(bins.items()):
        r = fit_bin(p, bx, by)
        if r is None or r[0] < 40:
            continue
        sc, fx, fy, fyaw = r
        bins[name] = (float(fx), float(fy), float(fyaw))
        fits.append({"name": name, "score": sc, "centre": [float(fx), float(fy)], "yaw": float(fyaw)})
    return bins, fits


def scene_mask(pts, bins):
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    # the whole desk top except the robot's base column; objects can slide past the arrival area
    keep = (x > 0.08) & (x < 0.80) & (np.abs(y) < 0.70) & (np.hypot(x, y) > 0.20) & (z > 0.02) & (z < Z_ROBOT)
    ox, oy, oz = BIN_OUTER
    ix, iy = BIN_INNER_HALF
    for b in bins.values():
        lx, ly = bin_local(b, x, y)
        in_outer = (np.abs(lx) <= ox / 2 + 0.014) & (np.abs(ly) <= oy / 2 + 0.014)
        in_inner = (np.abs(lx) <= ix - 0.010) & (np.abs(ly) <= iy - 0.010)
        keep &= ~(in_outer & ~in_inner & (z < oz + 0.025))
        keep &= ~(in_inner & (z < BIN_FLOOR_Z + 0.015))
    return keep


def rgb_to_hsv(c):
    r, g, b = c[:, 0], c[:, 1], c[:, 2]
    mx, mn = c.max(1), c.min(1)
    d = mx - mn
    nz = d > 1e-6
    dd = np.where(nz, d, 1)
    h = np.where(mx == r, ((g - b) / dd) % 6, np.where(mx == g, (b - r) / dd + 2, (r - g) / dd + 4))
    h = np.where(nz, h / 6.0, 0.0)
    s = np.where(mx > 1e-6, d / np.where(mx > 1e-6, mx, 1), 0)
    return np.stack([h, s, mx], 1)


def colour_hist(hsv, nh=12, ns=3, nv=4):
    h, s, v = hsv[:, 0], hsv[:, 1], hsv[:, 2]
    sat = s > 0.25
    hs = np.zeros((nh, ns))
    if sat.any():
        hi = np.minimum((h[sat] * nh).astype(int), nh - 1)
        si = np.minimum(((s[sat] - 0.25) / 0.75 * ns).astype(int), ns - 1)
        np.add.at(hs, (hi, si), 1)
    gv = np.zeros(nv)
    if (~sat).any():
        vi = np.minimum((v[~sat] * nv).astype(int), nv - 1)
        np.add.at(gv, vi, 1)
    hist = np.concatenate([hs.ravel(), gv])
    return hist / max(hist.sum(), 1)


def hist_dist(a, b):
    a, b = np.asarray(a), np.asarray(b)
    return float(0.5 * np.sum((a - b) ** 2 / (a + b + 1e-6)))


def extent_along(p, yaw):
    """Extent of points across the fingers (the finger axis = home +y turned by yaw)."""
    u = np.array([-math.sin(yaw), math.cos(yaw)])
    proj = p[:, :2] @ u
    return float(np.percentile(proj, 97) - np.percentile(proj, 3))


def cluster_features(p, c, bins):
    cen = p.mean(0)
    top = float(np.percentile(p[:, 2], 97))
    xy = p[:, :2] - cen[:2]
    cov = xy.T @ xy / max(len(p), 1)
    evals, evecs = np.linalg.eigh(cov)
    major = evecs[:, 1]
    yaw = float(np.arctan2(major[1], major[0]))
    proj = xy @ evecs
    ext = np.percentile(proj, 97, axis=0) - np.percentile(proj, 3, axis=0)
    b = bin_of_xy(bins, cen[0], cen[1])
    floor = BIN_FLOOR_Z if b else 0.0
    slab = p[p[:, 2] > top - 0.03]
    gxy = slab[:, :2].mean(0) if len(slab) >= 5 else cen[:2]
    slab_yaw = yaw
    if len(slab) >= 12:
        sxy = slab[:, :2] - slab[:, :2].mean(0)
        sev, svec = np.linalg.eigh(sxy.T @ sxy / len(slab))
        slab_yaw = float(np.arctan2(svec[1, 1], svec[0, 1]))
    return {"n": int(len(p)), "centroid": cen.tolist(), "grasp_xy": [float(gxy[0]), float(gxy[1])],
            "top_z": top, "floor": floor, "bin": b, "yaw": yaw, "slab_yaw": slab_yaw,
            "len_major": float(ext[1]), "len_minor": float(ext[0]),
            "dims": sorted([float(ext[1]), float(ext[0]), float(top - floor)]),
            "mean_rgb": c.mean(0).tolist(), "hist": colour_hist(rgb_to_hsv(c)).tolist(), "pts": p}


def perceive(images, bins=None, voxel=VOXEL, min_pts=12, extra=()):
    """images: {cam: (rgb uint8, depth uint16)} for the static cameras, plus optional extra
    views as (rgb, depth, R_cam, t_cam) -> (clusters, bins)."""
    P, C = [], []
    for cam in CAMS:
        rgb, d = images[cam]
        pts, col = backproject(d, rgb, CAMS[cam])
        P.append(pts)
        C.append(col)
    for rgb, d, R, t in extra:
        pts, col = backproject_rt(d, rgb, None, None, R, t)
        P.append(pts)
        C.append(col)
    allp, allc = np.concatenate(P), np.concatenate(C)
    if bins is None:
        bins, _ = detect_bins(allp, allc)
    m = scene_mask(allp, bins)
    pts, col = allp[m], allc[m]
    if len(pts) == 0:
        return [], bins
    lo = pts.min(0) - voxel
    idx = np.floor((pts - lo) / voxel).astype(int)
    labels, n = label_components(idx)
    out = []
    for k in range(n):
        mm = labels == k
        if mm.sum() < min_pts:
            continue
        out.append(cluster_features(pts[mm], col[mm], bins))
    return out, bins


def perceive_eih(rgb, depth, R_cam, t_cam, expected_xy, floor, top_hint, bins, radius=0.13, voxel=0.006):
    """Dense cluster of the object under the hand camera."""
    pts, col = backproject_rt(depth, rgb, CAMS["agentview"]["f"], CAMS["agentview"]["c"], R_cam, t_cam)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    keep = (z > floor + 0.010) & (z < top_hint + 0.06) & (np.hypot(x - expected_xy[0], y - expected_xy[1]) < radius)
    ox, oy, oz = BIN_OUTER
    ix, iy = BIN_INNER_HALF
    for b in bins.values():
        lx, ly = bin_local(b, x, y)
        in_outer = (np.abs(lx) <= ox / 2 + 0.014) & (np.abs(ly) <= oy / 2 + 0.014)
        in_inner = (np.abs(lx) <= ix - 0.008) & (np.abs(ly) <= iy - 0.008)
        keep &= ~(in_outer & ~in_inner & (z < oz + 0.025))
    pts, col = pts[keep], col[keep]
    if len(pts) < 30:
        return None
    lo = pts.min(0) - voxel
    idx = np.floor((pts - lo) / voxel).astype(int)
    labels, n = label_components(idx)
    best, bestn = None, 0
    for k in range(n):
        m = labels == k
        if m.sum() > bestn:
            best, bestn = m, int(m.sum())
    if bestn < 30:
        return None
    c = cluster_features(pts[best], col[best], bins)
    c["floor"] = floor
    c["dims"] = sorted([c["len_major"], c["len_minor"], c["top_z"] - floor])
    return c


class Classifier(object):
    def __init__(self, refs, w_dims=8.0):
        self.refs = refs
        self.w_dims = w_dims

    def dist(self, cl, r):
        return hist_dist(cl["hist"], r["hist"]) + self.w_dims * float(np.abs(np.array(cl["dims"]) - np.array(r["dims"])).sum())

    def __call__(self, cl):
        best = {}
        for r in self.refs:
            d = self.dist(cl, r)
            if r["model"] not in best or d < best[r["model"]]:
                best[r["model"]] = d
        if not best:
            return {"model": "?", "cls": "C", "d": 9.0, "class_margin": -1.0, "ranked": []}
        ranked = sorted(best.items(), key=lambda t: t[1])
        model, d0 = ranked[0]
        cls = CLASS_OF[model]
        other = [d for m, d in ranked if CLASS_OF[m] != cls]
        margin = (other[0] - d0) if other else 1.0
        return {"model": model, "cls": cls, "d": d0, "class_margin": margin, "ranked": ranked[:3]}


# ----------------------------------------------------------------------------------------
# Controller (generator based: `yield policy` where policy(obs) -> action)
# ----------------------------------------------------------------------------------------
class Controller(object):
    def __init__(self, refs):
        if isinstance(refs, list):
            refs = {"static": refs}
        self.classify = Classifier(refs.get("static", []))
        self.classify_eih = Classifier(refs.get("eih", []), w_dims=12.0)
        rel = refs.get("eih_rel")
        self.eih_R = np.array(rel["R"], float) if rel else np.eye(3)
        self.eih_t = np.array(rel["t"], float) if rel else np.zeros(3)
        self.reset()

    def reset(self):
        self.obs = None
        self.q_home = None
        self.gen = None
        self.last_t, self.policy = None, None
        self.bins = nominal_bins()
        self.request_text = None
        self.request_state = None
        self.drop_slot = {b: 0 for b in BIN_YS}
        self.skip_list = []
        self.last_perceive_t = -999

    # -- wire entry -------------------------------------------------------------------------
    def act(self, t, obs):
        if self.q_home is None:
            self.set_home(obs)
            self.gen = self.run()
            self.policy = next(self.gen)
        elif t != self.last_t:
            self.obs = obs
            self.policy = self.gen.send(obs)
        self.last_t = t
        return [float(x) for x in self.policy(obs)]

    def set_home(self, obs):
        """The first observation of an episode defines the home orientation (tool straight
        down, fingers along +y) that every later orientation target is expressed against."""
        self.obs = obs
        self.q_home = np.array(obs["ee_pose"][3:7], float)
        self.R_home = quat_to_mat(self.q_home)

    # -- proprio helpers ------------------------------------------------------------------
    def ee(self):
        return np.array(self.obs["ee_pose"][:3], float)

    def yaw(self, obs=None):
        R = quat_to_mat(np.array((obs or self.obs)["ee_pose"][3:7], float))
        y_axis = R[:, 1]
        return wrap(math.atan2(y_axis[1], y_axis[0]) - math.pi / 2)

    def orient_error(self, yaw, obs=None):
        R_cur = quat_to_mat(np.array((obs or self.obs)["ee_pose"][3:7], float))
        R_des = rot_z(yaw) @ self.R_home
        return rotvec(R_des @ R_cur.T)

    def width(self):
        return float(self.obs["gripper_width"])

    def t(self):
        return int(self.obs["t"])

    def request(self):
        return self.obs.get("request")

    # -- motion primitives --------------------------------------------------------------------
    def servo(self, pos, yaw, grip, mark=0, step_max=POS_STEP_MAX):
        pos = np.asarray(pos, float)

        def policy(o):
            err = pos - np.array(o["ee_pose"][:3], float)
            d = np.clip(err, -step_max, step_max)
            r = np.clip(self.orient_error(yaw, o), -ROT_STEP_MAX, ROT_STEP_MAX)
            return [d[0], d[1], d[2], r[0], r[1], r[2], grip, mark]
        return policy

    def goto(self, pos, yaw=0.0, grip=0.0, tol=0.01, yaw_tol=0.04, max_steps=60, mark=0, step_max=POS_STEP_MAX):
        pos = np.asarray(pos, float)
        for i in range(max_steps):
            err = pos - self.ee()
            if np.linalg.norm(err) < tol and np.linalg.norm(self.orient_error(yaw)) < yaw_tol:
                return True
            yield self.servo(pos, yaw, grip, mark, step_max)
            mark = 0
        return False

    def hold(self, steps, grip, mark=0):
        pos, yaw = self.ee(), self.yaw()
        for i in range(steps):
            yield self.servo(pos, yaw, grip, mark)
            mark = 0

    def gripper_closed_on_object(self):
        return 0.006 < self.width() < 0.077

    def pick(self, xy, yaw, top_z, floor):
        """Top-down pinch grasp at `yaw`. Returns True when something is held after lifting."""
        x, y = xy
        yaw = wrap_half(yaw)
        z_pre = max(top_z + 0.10, floor + 0.12)
        z_travel = max(TRAVEL_Z, z_pre)
        yield from self.goto((x, y, z_travel), yaw, grip=0.0, tol=0.015)
        yield from self.goto((x, y, z_pre), yaw, grip=0.0, tol=0.008, max_steps=30)
        z_g = max(top_z - GRASP_BELOW_TOP, floor + GRASP_MIN_ABOVE_FLOOR)
        yield from self.goto((x, y, z_g), yaw, grip=0.0, tol=0.006, max_steps=25, step_max=0.02)
        yield from self.hold(5, grip=1.0)
        held = self.gripper_closed_on_object()
        yield from self.goto((x, y, z_travel), yaw, grip=1.0, tol=0.015, max_steps=30, step_max=0.03)
        yield from self.hold(2, grip=1.0)
        held = held and self.gripper_closed_on_object()
        return held

    def place_in_bin(self, name):
        """Carry the held object over bin `name`, lower to just above the rim near its centre,
        release. The hand's wide axis runs along the bin's long side."""
        b = self.bins[name]
        slot = self.drop_slot[name] % 3
        self.drop_slot[name] += 1
        x, y = bin_point(b, (-0.035, 0.0, 0.035)[slot], (0.0, 0.02, -0.02)[slot])
        yaw = wrap_half(math.pi / 2 + b[2])
        z_rel = BIN_RIM_Z + DROP_ABOVE_RIM
        yield from self.goto((x, y, TRAVEL_Z), yaw, grip=1.0, tol=0.015)
        yield from self.goto((x, y, z_rel), yaw, grip=1.0, tol=0.01, max_steps=30)
        yield from self.hold(4, grip=0.0)
        yield from self.goto((x, y, TRAVEL_Z), yaw, grip=0.0, tol=0.02, max_steps=30)

    def place_on_desk(self, xy, yaw=0.0):
        x, y = xy
        yield from self.goto((x, y, TRAVEL_Z), yaw, grip=1.0, tol=0.015)
        yield from self.goto((x, y, 0.09), yaw, grip=1.0, tol=0.01, max_steps=30)
        yield from self.hold(4, grip=0.0)
        yield from self.goto((x, y, TRAVEL_Z), yaw, grip=0.0, tol=0.02, max_steps=30)

    def observe(self, settle=2):
        yield from self.goto(OBSERVE_POS, 0.0, grip=0.0, tol=0.02)
        yield from self.hold(settle, grip=0.0)
        imgs = {cam: (decode_png(self.obs["rgb"][cam], "RGB"), decode_png(self.obs["depth"][cam])) for cam in CAMS}
        # the hand camera at the observe pose looks straight down on the strip between the
        # robot and the bins, which the bin walls hide from the static cameras
        eih = (decode_png(self.obs["rgb"]["eye_in_hand"], "RGB"), decode_png(self.obs["depth"]["eye_in_hand"]))
        clusters, bins = perceive(imgs, extra=[(eih[0], eih[1]) + self.cam_pose()])
        self.bins = bins
        for c in clusters:
            c["cls_info"] = self.classify(c) if (c["bin"] is None or c["n"] < 600) else None
        self.last_perceive_t = self.t()
        self.dump("observe", imgs["agentview"][0], clusters)
        return clusters

    def dump(self, tag, rgb, clusters):
        if not LOG:
            return
        try:
            os.makedirs(LOG, exist_ok=True)
            Image.fromarray(rgb).save(os.path.join(LOG, "t%06d_%s.png" % (self.t(), tag)))
            rows = []
            for c in clusters:
                info = c.get("cls_info")
                rows.append({"n": c["n"], "centroid": [round(v, 3) for v in c["centroid"]], "top_z": round(c["top_z"], 3),
                             "bin": c["bin"], "dims": [round(v, 3) for v in c["dims"]], "yaw": round(c["yaw"], 2),
                             "model": info["model"] if info else None, "d": round(info["d"], 3) if info else None,
                             "margin": round(info["class_margin"], 3) if info else None})
            with open(os.path.join(LOG, "t%06d_%s.json" % (self.t(), tag)), "w") as f:
                json.dump({"t": self.t(), "request": self.request(), "bins": {k: [round(v, 3) for v in b] for k, b in self.bins.items()}, "clusters": rows}, f)
        except Exception as exc:  # noqa: BLE001
            log("dump failed: %r" % exc)

    def cam_pose(self):
        q = np.array(self.obs["ee_pose"][3:7], float)
        R_e = quat_to_mat(q)
        t_e = self.ee()
        return R_e @ self.eih_R, t_e + R_e @ self.eih_t

    def inspect(self, c):
        """Hover over a cluster and classify it from the hand camera (dense cloud)."""
        x, y = c["grasp_xy"]
        yield from self.goto((x, y, max(TRAVEL_Z, c["top_z"] + HOVER_ABOVE_TOP)), 0.0, grip=0.0, tol=0.015)
        yield from self.goto((x, y, c["top_z"] + HOVER_ABOVE_TOP), 0.0, grip=0.0, tol=0.006, yaw_tol=0.03, max_steps=40)
        yield from self.hold(3, grip=0.0)
        R_cam, t_cam = self.cam_pose()
        rgb = decode_png(self.obs["rgb"]["eye_in_hand"], "RGB")
        dep = decode_png(self.obs["depth"]["eye_in_hand"])
        d = perceive_eih(rgb, dep, R_cam, t_cam, (x, y), c["floor"], c["top_z"], self.bins)
        if d is None:
            log("t=%d inspect: nothing under the hand camera" % self.t())
            self.dump("inspect_none", rgb, [])
            return None
        d["cls_info"] = self.classify_eih(d)
        d["bin"] = c["bin"]
        self.dump("inspect", rgb, [d])
        return d

    # -- grasp planning -------------------------------------------------------------------------
    def choose_yaw(self, c, inside=None):
        """A hand yaw for cluster c: fingers across its narrow side when possible, never
        colliding with a bin wall, grasped width under the opening. None when impossible."""
        x, y = c["grasp_xy"]
        p = c.get("pts")
        pref = wrap_half(c.get("slab_yaw", c["yaw"]))
        cands = [pref] + [wrap_half(pref + s * d) for d in (0.26, 0.52, 0.79, 1.05, 1.31, 1.57) for s in (1, -1)]
        for yaw in cands:
            if not hand_fits(self.bins, x, y, yaw, inside=inside):
                continue
            if p is not None and len(p) >= 10:
                slab = p[p[:, 2] > c["top_z"] - 0.05]
                if len(slab) >= 10 and extent_along(slab, yaw) > GRASP_MAX_WIDTH:
                    continue
            return yaw
        return None

    def free_spot(self, clusters):
        cands = [(0.68, 0.0), (0.68, -0.2), (0.68, 0.2), (0.68, -0.4), (0.68, 0.4), (0.70, -0.3), (0.70, 0.3),
                 (0.25, -0.15), (0.25, 0.15), (0.26, -0.35), (0.26, 0.35), (0.42, -0.58), (0.42, 0.58), (0.56, -0.58), (0.56, 0.58)]
        occ = [c["centroid"][:2] for c in clusters]
        for x, y in cands:
            if math.hypot(x, y) > REACH - 0.06 or math.hypot(x, y) < BASE_CLEAR + 0.03:
                continue
            if any(math.hypot(x - ox, y - oy) < 0.11 for ox, oy in occ):
                continue
            if not (hand_fits(self.bins, x, y, 0.0) or hand_fits(self.bins, x, y, math.pi / 2)):
                continue
            return (x, y)
        return None

    def spot_yaw(self, xy):
        return 0.0 if hand_fits(self.bins, xy[0], xy[1], 0.0) else math.pi / 2

    def loose(self, clusters):
        return [c for c in clusters if c["bin"] is None and c["cls_info"] is not None]

    @staticmethod
    def _near_any(c, points, r=0.05):
        return any(math.hypot(c["centroid"][0] - x, c["centroid"][1] - y) < r for x, y in points)

    def skipped(self, c):
        for (x, y, t) in self.skip_list:
            if math.hypot(x - c["centroid"][0], y - c["centroid"][1]) < 0.05 and self.t() - t < 600:
                return True
        return False

    def skip(self, c):
        self.skip_list.append((c["centroid"][0], c["centroid"][1], self.t()))

    # -- main behaviour -----------------------------------------------------------------------
    def run(self):
        while True:
            clusters = yield from self.observe()
            req = self.request()
            if req is None:
                self.request_state = None
            elif req["text"] != self.request_text or self.request_state is None:
                self.request_text = req["text"]
                self.request_state = {"model": req["text"].replace("the ", "", 1), "inspected": [], "deadline": req["deadline_t"]}
                log("t=%d request %r deadline %d" % (self.t(), self.request_text, req["deadline_t"]))
            if self.request_state is not None:
                done = yield from self.serve_request(clusters)
                if done:
                    continue
            else:
                did = yield from self.tidy(clusters)
                if did:
                    continue
            yield from self.hold(10, grip=0.0)

    def tidy(self, clusters):
        loose = [c for c in self.loose(clusters) if not self.skipped(c)]
        loose.sort(key=lambda c: -c["cls_info"]["class_margin"])
        for c in loose:
            d = yield from self.inspect(c)
            if d is None:
                self.skip(c)
                return True
            info = d["cls_info"]
            yaw = self.choose_yaw(d)
            log("t=%d loose %s (%s d=%.2f margin=%.2f; static %s) at (%.2f,%.2f) top=%.3f dims=%s yaw=%s" % (
                self.t(), info["model"], info["cls"], info["d"], info["class_margin"], c["cls_info"]["model"],
                d["grasp_xy"][0], d["grasp_xy"][1], d["top_z"], [round(v, 3) for v in d["dims"]], None if yaw is None else round(yaw, 2)))
            if info["class_margin"] < 0.08 or yaw is None:
                self.skip(c)
                return True
            held = yield from self.pick(d["grasp_xy"], yaw, d["top_z"], d["floor"])
            if held:
                yield from self.place_in_bin(BIN_OF_CLASS[info["cls"]])
            else:
                log("t=%d grasp failed" % self.t())
                self.skip(c)
            return True
        for c in clusters:
            if c["bin"] in CLASS_OF_BIN and c["cls_info"] is not None and not self.skipped(c) \
                    and c["cls_info"]["cls"] != CLASS_OF_BIN[c["bin"]] and c["cls_info"]["class_margin"] > 0.15:
                d = yield from self.inspect(c)
                if d is None or d["cls_info"]["cls"] == CLASS_OF_BIN[c["bin"]] or d["cls_info"]["class_margin"] < 0.15:
                    self.skip(c)
                    return True
                yaw = self.choose_yaw(d, inside=c["bin"])
                log("t=%d misplaced %s in %s yaw=%s" % (self.t(), d["cls_info"]["model"], c["bin"], None if yaw is None else round(yaw, 2)))
                if yaw is None:
                    self.skip(c)
                    return True
                held = yield from self.pick(d["grasp_xy"], yaw, d["top_z"], d["floor"])
                if held:
                    yield from self.place_in_bin(BIN_OF_CLASS[d["cls_info"]["cls"]])
                else:
                    self.skip(c)
                return True
        return False

    def serve_request(self, clusters):
        rs = self.request_state
        model = rs["model"]
        cls = CLASS_OF.get(model)
        loose = self.loose(clusters)
        cands = sorted(loose, key=lambda c: min([d for m, d in c["cls_info"]["ranked"] if m == model] or [9.0]))
        for c in cands:
            if self._near_any(c, rs["inspected"]):
                continue
            key = (c["centroid"][0], c["centroid"][1])
            rs["inspected"].append(key)
            d = yield from self.inspect(c)
            if d is None:
                return True
            info = d["cls_info"]
            log("t=%d request check: %s (d=%.2f) vs wanted %s" % (self.t(), info["model"], info["d"], model))
            if info["model"] == model:
                yaw = self.choose_yaw(d)
                if yaw is None:
                    log("t=%d target not graspable where it lies" % self.t())
                    return True
                held = yield from self.pick(d["grasp_xy"], yaw, d["top_z"], d["floor"])
                if held:
                    yield from self.place_in_bin("red")
                    yield from self.hold(8, grip=0.0)
                    ok = yield from self.confirm_red()
                    if ok:
                        yield from self.hold(1, grip=0.0, mark=1)
                        log("t=%d marked delivered" % self.t())
                        self.request_state = None
                    else:
                        log("t=%d red bin check failed; retrying" % self.t())
                        rs["inspected"].remove(key)
                else:
                    log("t=%d grasp of the target failed; retrying" % self.t())
                    rs["inspected"].remove(key)
            return True
        order = [BIN_OF_CLASS[cls]] if cls else []
        order += [b for b in ("gray", "lightblue", "blue") if b not in order]
        for b in order:
            inside = [c for c in clusters if c["bin"] == b]
            if not inside:
                continue
            c = max(inside, key=lambda c: c["top_z"])
            spot = self.free_spot(clusters)
            if spot is None:
                log("t=%d no free desk spot" % self.t())
                return False
            yaw = self.choose_yaw(c, inside=b)
            log("t=%d unpack %s: cluster n=%d top=%.3f at (%.2f,%.2f) yaw=%s -> (%.2f,%.2f)" % (
                self.t(), b, c["n"], c["top_z"], c["grasp_xy"][0], c["grasp_xy"][1], None if yaw is None else round(yaw, 2), spot[0], spot[1]))
            if yaw is None:
                rs["unpack_failures"] = rs.get("unpack_failures", 0) + 1
                if rs["unpack_failures"] > 6:
                    return False
                return True
            held = yield from self.pick(c["grasp_xy"], yaw, c["top_z"], c["floor"])
            if held:
                yield from self.place_on_desk(spot, self.spot_yaw(spot))
            else:
                rs["unpack_failures"] = rs.get("unpack_failures", 0) + 1
                if rs["unpack_failures"] > 6:
                    log("t=%d giving up unpacking %s" % (self.t(), b))
                    return False
            return True
        if any(not self._near_any(c, rs["inspected"]) for c in loose):
            return True
        log("t=%d declare absent: %s" % (self.t(), model))
        yield from self.hold(1, grip=0.0, mark=2)
        self.request_state = None
        return True

    def confirm_red(self):
        b = self.bins["red"]
        x, y = b[0], b[1]
        yield from self.goto((x, y, BIN_RIM_Z + 0.16), 0.0, grip=0.0, tol=0.01, max_steps=30)
        yield from self.hold(2, grip=0.0)
        R_cam, t_cam = self.cam_pose()
        rgb = decode_png(self.obs["rgb"]["eye_in_hand"], "RGB")
        dep = decode_png(self.obs["depth"]["eye_in_hand"])
        d = perceive_eih(rgb, dep, R_cam, t_cam, (x, y), BIN_FLOOR_Z, BIN_RIM_Z, self.bins, radius=0.14)
        self.dump("confirm_red", rgb, [d] if d else [])
        return d is not None and bin_of_xy(self.bins, d["centroid"][0], d["centroid"][1]) == "red"


# ----------------------------------------------------------------------------------------
# Wire
# ----------------------------------------------------------------------------------------
def load_refs():
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, "refs.json")) as f:
        return json.load(f)


def main():
    ctl = Controller(load_refs())
    print("READY", flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        msg = json.loads(line)
        kind = msg.get("type")
        if kind == "reset":
            ctl.reset()
            print(json.dumps({"ok": True}), flush=True)
        elif kind == "act":
            try:
                a = ctl.act(int(msg.get("t", 0)), msg["obs"])
            except Exception as exc:  # noqa: BLE001 - never break the wire
                log("error at t=%s: %r" % (msg.get("t"), exc))
                a = [0.0] * 8
            print(json.dumps({"action": a}), flush=True)
        elif kind == "close":
            print(json.dumps({"ok": True}), flush=True)
            return
        else:
            print(json.dumps({"error": "unknown request"}), flush=True)


if __name__ == "__main__":
    main()
