#!/usr/bin/env python3
"""Reference controller for object_finding_and_organizing_on_messy_desk.

Works from the observation only (two static RGB-D cameras, the hand camera, proprioception,
the request text). Per decision:

  depth back-projection (both static cameras) -> containers located from their rim band (pose
  varies per episode) -> desk / container walls / container floors / robot removed -> voxel
  connected-component clustering -> per-cluster colour histogram + sorted extents vs the
  static references -> hover and confirm with the hand camera (dense cloud, its own
  references) -> grasp yaw chosen so that the 0.2 m wide hand clears every container wall and the
  fingers close across a dimension under the 8 cm opening -> top-down pinch, carry, drop near
  the container's centre.

Requests: deliver the named object to the red container (searching loose objects first, then
unpacking its class container, then the other containers onto the desk) and mark 1 after the hand camera
confirms the drop; mark 2 only once every container is empty and every loose object has been seen.

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
CONTAINER_X = 0.44
CONTAINER_YS = {"gray": -0.45, "lightblue": -0.15, "blue": 0.15, "red": 0.45}
CONTAINER_OUTER = (0.326, 0.247, 0.190)          # outer footprint x (long), y (short), wall height
CONTAINER_INNER_HALF = (0.140, 0.109)
CONTAINER_FLOOR_Z, CONTAINER_RIM_Z = 0.0035, 0.191
CONTAINER_OF_CLASS = {"A": "gray", "B": "lightblue", "C": "blue"}
CLASS_OF_CONTAINER = {v: k for k, v in CONTAINER_OF_CLASS.items()}
CLASSES = {
    "A": ["cracker box", "sugar box", "pudding box", "gelatin box", "wood block", "foam brick"],
    "B": ["master chef can", "tomato soup can", "tuna fish can", "potted meat can", "mustard bottle", "bleach cleanser"],
    "C": ["banana", "bowl", "mug", "power drill", "scissors", "large marker", "large clamp", "extra large clamp", "pitcher base"],
}
CLASS_OF = {m: c for c, ms in CLASSES.items() for m in ms}
# look-alike pairs the classifiers confuse (same colours, a size apart): either may be the other
TWIN = {"large clamp": "extra large clamp", "extra large clamp": "large clamp", "pudding box": "gelatin box", "gelatin box": "pudding box"}


def is_model(info, model, slack=0.25):
    """The classification `info` names `model`, or its look-alike twin with `model` close behind."""
    if info["model"] == model:
        return True
    return TWIN.get(info["model"]) == model and info.get("all", {}).get(model, 9.0) - info["d"] < slack
DESK_X, DESK_Y = (0.15, 0.78), (-0.62, 0.62)
REACH, BASE_CLEAR = 0.80, 0.26
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
GRASP_MAX_WIDTH = 0.080     # the hand's opening
WIDTH_MARGIN = 0.014        # the hand-camera cloud loses ~7 mm per side at an object's silhouette

# controller tunables
OBSERVE_POS = (0.30, 0.0, 0.55)
TRAVEL_Z = 0.32
VOXEL = 0.012
Z_ROBOT = 0.30
GRASP_BELOW_TOP = 0.03
RIM_GRASP_BELOW_TOP = 0.022     # wide hollow objects (bowl, mug, pitcher): straddle the wall at the rim
RIM_MIN_RADIUS, RIM_MIN_DEPTH = 0.035, 0.012
REFINE_RADIUS, REFINE_MAX_SHIFT = 0.07, 0.04    # hand-camera re-centring just before the descent
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
# Containers (pose varies per episode: centre x, y and yaw of the long side)
# ----------------------------------------------------------------------------------------
def nominal_containers():
    return {n: (CONTAINER_X, y, 0.0) for n, y in CONTAINER_YS.items()}


def container_local(b, x, y):
    """Desk xy -> coordinates in container b's frame (x along its long side). Vectorised."""
    bx, by, yaw = b
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = x - bx, y - by
    return c * dx + s * dy, -s * dx + c * dy


def container_point(b, lx, ly):
    bx, by, yaw = b
    c, s = math.cos(yaw), math.sin(yaw)
    return bx + c * lx - s * ly, by + s * lx + c * ly


def container_of_xy(containers, x, y, shrink=0.0):
    for name, b in containers.items():
        lx, ly = container_local(b, x, y)
        if abs(lx) <= CONTAINER_INNER_HALF[0] - shrink and abs(ly) <= CONTAINER_INNER_HALF[1] - shrink:
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


def hand_fits(containers, x, y, yaw, inside=None):
    """Open hand lowered at (x, y) with `yaw`: outside every container's outer footprint, or (when
    `inside` names a container) entirely within that container's interior."""
    pts = hand_corners(x, y, yaw, 0.005)
    ox, oy = CONTAINER_OUTER[0] / 2, CONTAINER_OUTER[1] / 2
    ix, iy = CONTAINER_INNER_HALF
    for name, b in containers.items():
        for px, py in pts:
            lx, ly = container_local(b, px, py)
            if name == inside:
                # inside a container the hand body (0.205 x 0.063) fits only near the centre line:
                # every millimetre of margin counts (the containers are kinematic and do not yield)
                if abs(lx) > ix - 0.004 or abs(ly) > iy - 0.004:
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


EDGE_TAU = 0.012     # depth jump to a 4-neighbour (m) that marks an anti-aliased silhouette pixel


def depth_edges(z, tau=EDGE_TAU):
    """Silhouette pixels: the renderer anti-aliases depth, so pixels on an object's outline carry
    a depth between the object and what lies behind it and back-project into a 'skirt' of flying
    points hanging from the rim down to the desk (2-4 cm around every object, and long strips
    under the fingers). Drop every pixel whose depth differs from any 4-neighbour by more than `tau`."""
    e = np.zeros(z.shape, dtype=bool)
    d = np.abs(np.diff(z, axis=0)) > tau
    e[:-1, :] |= d
    e[1:, :] |= d
    d = np.abs(np.diff(z, axis=1)) > tau
    e[:, :-1] |= d
    e[:, 1:] |= d
    return e


def backproject(depth_mm, rgb, cam, max_depth=3.0):
    z = depth_mm.astype(np.float32) / 1000.0
    h, w = z.shape
    f, c = intrinsics(w)
    v, u = np.mgrid[0:h, 0:w]
    valid = (z > 0.05) & (z < max_depth) & ~depth_edges(z)
    pts = np.stack([z[valid], -(u[valid] - c) / f * z[valid], -(v[valid] - c) / f * z[valid]], 1)
    pose = cam["pose"]
    world = pts @ quat_to_mat(pose[3:7]).T + np.asarray(pose[:3])
    return world, rgb[valid].astype(np.float32) / 255.0


def backproject_rt(depth_mm, rgb, f, c, R, t, max_depth=1.5):
    z = depth_mm.astype(np.float32) / 1000.0
    h, w = z.shape
    f, c = intrinsics(w)
    v, u = np.mgrid[0:h, 0:w]
    valid = (z > 0.02) & (z < max_depth) & ~depth_edges(z)
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


def fit_container(p, cx, cy, yaw0=0.0):
    """Refine one container's pose from rim-band points near its nominal centre: coarse-to-fine grid
    search over (x, y, yaw) maximising the number of points on the outline of the known outer
    footprint. Robust to partial rings (a container seen from one side only)."""
    ox, oy = CONTAINER_OUTER[0] / 2, CONTAINER_OUTER[1] / 2
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


def detect_containers(pts, col):
    """Container poses from the rim band of the fused cloud (the upper walls, above most objects):
    each container is fitted around its nominal pose with the known footprint. A container with too few
    band points keeps its nominal pose."""
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    band = (z > CONTAINER_RIM_Z - 0.07) & (z < CONTAINER_RIM_Z + 0.02) & (x > DESK_X[0]) & (x < 0.80) & (np.abs(y) < 0.65)
    containers = nominal_containers()
    fits = []
    if band.sum() < 40:
        return containers, fits
    p = pts[band][:, :2]
    for name, (bx, by, _) in list(containers.items()):
        r = fit_container(p, bx, by)
        if r is None or r[0] < 40:
            continue
        sc, fx, fy, fyaw = r
        containers[name] = (float(fx), float(fy), float(fyaw))
        fits.append({"name": name, "score": sc, "centre": [float(fx), float(fy)], "yaw": float(fyaw)})
    return containers, fits


# The bare desk top (the stock table): flat at z = -0.0014 out to x = 0.809, then its rounded rim
# rises 1.2 cm (z = +0.0104 at x = 0.839-0.854) and falls away past x = 0.90 (task card).
DESK_PROFILE_X = (0.0, 0.809, 0.839, 0.854, 0.872, 0.884, 0.896, 0.907, 0.918, 0.929, 0.940, 0.950)
DESK_PROFILE_Z = (-0.0014, -0.0014, 0.0104, 0.0104, 0.0094, 0.0075, 0.0047, 0.0011, -0.0035, -0.0089, -0.0152, -0.0223)


def desk_top_z(x):
    return np.interp(np.abs(x), DESK_PROFILE_X, DESK_PROFILE_Z)


def scene_mask(pts, containers):
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    # the whole desk top except the robot's base column; objects can slide past the arrival area.
    # Heights are taken above the desk's own profile, so its raised rim is not an object.
    h = z - desk_top_z(x)
    keep = (x > 0.08) & (x < 0.80) & (np.abs(y) < 0.70) & (np.hypot(x, y) > 0.20) & (h > 0.012) & (z < Z_ROBOT)
    ox, oy, oz = CONTAINER_OUTER
    ix, iy = CONTAINER_INNER_HALF
    for b in containers.values():
        lx, ly = container_local(b, x, y)
        in_outer = (np.abs(lx) <= ox / 2 + 0.014) & (np.abs(ly) <= oy / 2 + 0.014)
        in_inner = (np.abs(lx) <= ix - 0.010) & (np.abs(ly) <= iy - 0.010)
        keep &= ~(in_outer & ~in_inner & (z < oz + 0.025))
        keep &= ~(in_inner & (z < CONTAINER_FLOOR_Z + 0.015))
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


def extent_along(p, yaw, center=None, strip=0.02):
    """Extent of points across the fingers (the finger axis = home +y turned by yaw). With
    `center`, only the strip the finger pads close on (|offset along the hand's thickness| <
    `strip`) counts: a long or curved object is only as wide as its cross-section there."""
    u = np.array([-math.sin(yaw), math.cos(yaw)])
    if center is not None:
        v = np.array([math.cos(yaw), math.sin(yaw)])
        sel = np.abs((p[:, :2] - np.asarray(center[:2])) @ v) < strip
        if sel.sum() >= 8:
            p = p[sel]
    proj = p[:, :2] @ u
    return float(np.percentile(proj, 97) - np.percentile(proj, 3))


def cluster_features(p, c, containers):
    cen = p.mean(0)
    top = float(np.percentile(p[:, 2], 97))
    xy = p[:, :2] - cen[:2]
    cov = xy.T @ xy / max(len(p), 1)
    evals, evecs = np.linalg.eigh(cov)
    major = evecs[:, 1]
    yaw = float(np.arctan2(major[1], major[0]))
    proj = xy @ evecs
    ext = np.percentile(proj, 97, axis=0) - np.percentile(proj, 3, axis=0)
    b = container_of_xy(containers, cen[0], cen[1])
    floor = CONTAINER_FLOOR_Z if b else float(desk_top_z(cen[0]))
    slab = p[p[:, 2] > top - 0.03]
    gxy = slab[:, :2].mean(0) if len(slab) >= 5 else cen[:2]
    slab_yaw = yaw
    if len(slab) >= 12:
        sxy = slab[:, :2] - slab[:, :2].mean(0)
        sev, svec = np.linalg.eigh(sxy.T @ sxy / len(slab))
        slab_yaw = float(np.arctan2(svec[1, 1], svec[0, 1]))
    return {"n": int(len(p)), "centroid": cen.tolist(), "grasp_xy": [float(gxy[0]), float(gxy[1])],
            "top_z": top, "floor": floor, "container": b, "yaw": yaw, "slab_yaw": slab_yaw,
            "len_major": float(ext[1]), "len_minor": float(ext[0]),
            "dims": sorted([float(ext[1]), float(ext[0]), float(top - floor)]),
            "mean_rgb": c.mean(0).tolist(), "hist": colour_hist(rgb_to_hsv(c)).tolist(), "pts": p}


def perceive(images, containers=None, voxel=VOXEL, min_pts=12, extra=()):
    """images: {cam: (rgb uint8, depth uint16)} for the static cameras, plus optional extra
    views as (rgb, depth, R_cam, t_cam) -> (clusters, containers)."""
    P, C = [], []
    for cam in CAMS:
        rgb, d = images[cam]
        pts, col = backproject(d, rgb, CAMS[cam])
        P.append(pts)
        C.append(col)
    allp, allc = np.concatenate(P), np.concatenate(C)
    if containers is None:
        # container poses from the static cameras only: they see all four rims completely,
        # while a hand-camera view clips them at its edge and biases the fit
        containers, _ = detect_containers(allp, allc)
    for rgb, d, R, t in extra:
        pts, col = backproject_rt(d, rgb, None, None, R, t)
        # extra (hand-camera) views only fill the strip between the robot and the containers,
        # the one place the static cameras cannot see; elsewhere they add nothing but noise
        keep = pts[:, 0] < CONTAINER_X - CONTAINER_OUTER[0] / 2 - 0.02
        P.append(pts[keep])
        C.append(col[keep])
    allp, allc = np.concatenate(P), np.concatenate(C)
    m = scene_mask(allp, containers)
    pts, col = allp[m], allc[m]
    if len(pts) == 0:
        return [], containers
    lo = pts.min(0) - voxel
    idx = np.floor((pts - lo) / voxel).astype(int)
    labels, n = label_components(idx)
    out = []
    for k in range(n):
        mm = labels == k
        if mm.sum() < min_pts:
            continue
        out.append(cluster_features(pts[mm], col[mm], containers))
    return out, containers


def perceive_eih(rgb, depth, R_cam, t_cam, expected_xy, floor, top_hint, containers, radius=0.13, voxel=0.006):
    """Dense cluster of the object under the hand camera."""
    pts, col = backproject_rt(depth, rgb, CAMS["agentview"]["f"], CAMS["agentview"]["c"], R_cam, t_cam)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    # height above whatever the point stands on: the container floor inside a container, else the desk profile
    inside = np.zeros(len(pts), bool)
    for b in containers.values():
        lx, ly = container_local(b, x, y)
        inside |= (np.abs(lx) <= CONTAINER_INNER_HALF[0]) & (np.abs(ly) <= CONTAINER_INNER_HALF[1])
    base = np.where(inside, CONTAINER_FLOOR_Z, desk_top_z(x))
    keep = (z - base > 0.008) & (z < top_hint + 0.06) & (np.hypot(x - expected_xy[0], y - expected_xy[1]) < radius)
    ox, oy, oz = CONTAINER_OUTER
    ix, iy = CONTAINER_INNER_HALF
    for b in containers.values():
        lx, ly = container_local(b, x, y)
        in_outer = (np.abs(lx) <= ox / 2 + 0.014) & (np.abs(ly) <= oy / 2 + 0.014)
        in_inner = (np.abs(lx) <= ix - 0.008) & (np.abs(ly) <= iy - 0.008)
        keep &= ~(in_outer & ~in_inner & (z < oz + 0.025))
    pts, col = pts[keep], col[keep]
    if len(pts) < 30:
        return None
    lo = pts.min(0) - voxel
    idx = np.floor((pts - lo) / voxel).astype(int)
    labels, n = label_components(idx)
    # the object under the camera: a large component near the expected position (a neighbour
    # half inside the radius is large too, but far off)
    best, best_score = None, 0.0
    for k in range(n):
        m = labels == k
        cnt = int(m.sum())
        if cnt < 30:
            continue
        d = math.hypot(float(pts[m, 0].mean()) - expected_xy[0], float(pts[m, 1].mean()) - expected_xy[1])
        sc = cnt * math.exp(-(d / 0.06) ** 2)
        if sc > best_score:
            best, best_score = m, sc
    if best is None:
        return None
    c = cluster_features(pts[best], col[best], containers)
    # two objects that touch merge into one component; the grasp point comes from the top
    # surface around the expected position only, so a neighbour does not drag it aside
    p = pts[best]
    near = p[np.hypot(p[:, 0] - expected_xy[0], p[:, 1] - expected_xy[1]) < 0.06]
    if c["len_major"] > 0.12 and len(near) >= 30:
        top_near = float(np.percentile(near[:, 2], 97))
        slab = near[near[:, 2] > top_near - 0.03]
        if len(slab) >= 10:
            c["grasp_xy"] = [float(slab[:, 0].mean()), float(slab[:, 1].mean())]
    c["floor"] = floor if c["container"] else float(desk_top_z(c["centroid"][0]))
    c["dims"] = sorted([c["len_major"], c["len_minor"], c["top_z"] - c["floor"]])
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
        return decide(best)


def decide(best):
    """Per-model distances {model: d} -> the decision: nearest model, its class, and the margin
    to the nearest model of another class."""
    if not best:
        return {"model": "?", "cls": "C", "d": 9.0, "class_margin": -1.0, "ranked": [], "all": {}}
    ranked = sorted(best.items(), key=lambda t: t[1])
    model, d0 = ranked[0]
    cls = CLASS_OF[model]
    other = [d for m, d in ranked if CLASS_OF[m] != cls]
    margin = (other[0] - d0) if other else 1.0
    model_margin = (ranked[1][1] - d0) if len(ranked) > 1 else 1.0
    return {"model": model, "cls": cls, "d": d0, "class_margin": margin, "model_margin": model_margin,
            "ranked": ranked[:3], "all": dict(best)}


STATIC_WEIGHT = 0.7     # hand-camera + 0.7 x static distance: either view alone confuses a few look-alikes


def combine(eih_info, static_info, w=STATIC_WEIGHT):
    """Fuse the hand camera's per-model distances with the static cameras' for the same object."""
    if not static_info or not static_info.get("all"):
        return eih_info
    e, s = eih_info["all"], static_info["all"]
    return decide({m: e.get(m, 9.0) + w * s.get(m, 9.0) for m in set(e) | set(s)})


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
        self.containers = nominal_containers()
        self.request_text = None
        self.request_state = None
        self.drop_slot = {b: 0 for b in CONTAINER_YS}
        self.skip_list = []
        self.last_perceive_t = -999
        self.last_clusters = []
        self.carry_hang = 0.0
        # what the controller knows about the class containers' contents: the models it put in
        # itself (bottom -> top), and whether an initial pile of unseen objects may still lie
        # under them (the instruction: A objects start in gray, B objects in light-blue)
        self.inventory = {b: [] for b in CLASS_OF_CONTAINER}
        self.pile_unknown = {"gray": True, "lightblue": True, "blue": False}

    # -- wire entry -------------------------------------------------------------------------
    def act(self, t, obs):
        if self.q_home is None:
            self.set_home(obs)
            self.gen = self.run()
            self.policy = next(self.gen)
        elif t != self.last_t:
            self.obs = obs
            try:
                self.policy = self.gen.send(obs)
            except Exception as exc:  # noqa: BLE001 - one failed behaviour must not end the episode
                log("t=%d behaviour failed (%r); starting over from the observe pose" % (t, exc))
                self.carry_hang = 0.0
                self.gen = self.run()
                try:
                    self.policy = next(self.gen)
                except Exception as exc2:  # noqa: BLE001
                    log("t=%d restart failed too (%r); holding still" % (t, exc2))
                    self.q_home = None          # rebuild everything on the next step
                    self.policy = lambda o: [0.0] * 8
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

    def travel_z(self):
        """Carry height: the held object's bottom clears the container rims."""
        return max(TRAVEL_Z, CONTAINER_RIM_Z + 0.04 + self.carry_hang)

    def refine_xy(self, xy, anchor_xy, floor, top_z):
        """Hovering 10 cm above the object with the hand camera: re-measure the object's top
        centroid and shift the grasp point by how far it moved from the planned anchor."""
        try:
            R_cam, t_cam = self.cam_pose()
            rgb = decode_png(self.obs["rgb"]["eye_in_hand"], "RGB")
            dep = decode_png(self.obs["depth"]["eye_in_hand"])
            d = perceive_eih(rgb, dep, R_cam, t_cam, anchor_xy, floor, top_z, self.containers, radius=REFINE_RADIUS)
        except Exception as exc:  # noqa: BLE001
            log("refine failed: %r" % exc)
            return xy
        if d is None:
            return xy
        p = d["pts"]
        slab = p[p[:, 2] > d["top_z"] - 0.02]
        if len(slab) < 15:
            return xy
        cx, cy = slab[:, :2].mean(0)
        dx, dy = float(cx - anchor_xy[0]), float(cy - anchor_xy[1])
        if math.hypot(dx, dy) > REFINE_MAX_SHIFT:
            log("t=%d refine: shift %.3f too large, ignored" % (self.t(), math.hypot(dx, dy)))
            return xy
        log("t=%d refine: shift (%+.3f,%+.3f) from %d top points" % (self.t(), dx, dy, len(slab)))
        return (xy[0] + dx, xy[1] + dy)

    def pick(self, xy, yaw, top_z, floor, below_top=GRASP_BELOW_TOP, anchor_xy=None):
        """Top-down pinch grasp at `yaw`, fingers `below_top` under the object's top (a rim grasp
        uses a shallow depth). The grasp point is refined from the hand camera 10 cm above the
        object before the final descent. Returns True when something is held after lifting."""
        x, y = xy
        yaw = wrap_half(yaw)
        z_pre = max(top_z + 0.10, floor + 0.12)
        self.carry_hang = max(0.0, (top_z - floor) - below_top)     # how far the object hangs below the fingertips
        z_travel = max(self.travel_z(), z_pre)
        yield from self.goto((x, y, z_travel), yaw, grip=0.0, tol=0.015)
        yield from self.goto((x, y, z_pre), yaw, grip=0.0, tol=0.008, max_steps=30)
        yield from self.hold(1, grip=0.0)
        x, y = self.refine_xy((x, y), anchor_xy or (x, y), floor, top_z)
        z_g = max(top_z - below_top, floor + GRASP_MIN_ABOVE_FLOOR)
        self.last_grasp = (x, y, z_g, yaw)
        yield from self.goto((x, y, z_g), yaw, grip=0.0, tol=0.006, max_steps=25, step_max=0.02)
        self.last_grasp_actual = [float(v) for v in self.ee()]
        yield from self.hold(5, grip=1.0)
        held = self.gripper_closed_on_object()
        yield from self.goto((x, y, z_travel), yaw, grip=1.0, tol=0.015, max_steps=30, step_max=0.03)
        yield from self.hold(2, grip=1.0)
        held = held and self.gripper_closed_on_object()
        return held

    def place_in_container(self, name, model=None):
        """Carry the held object over container `name`, lower to just above the rim near its centre,
        release. The hand's wide axis runs along the container's long side. `model` (when known)
        goes on the container's inventory."""
        if model is not None and name in self.inventory:
            self.inventory[name].append(model)
        b = self.containers[name]
        slot = self.drop_slot[name] % 3
        self.drop_slot[name] += 1
        # near the centre line, where a top-down hand fits to take it out again later
        x, y = container_point(b, (-0.01, 0.0, 0.01)[slot], (0.0, 0.025, -0.025)[slot])
        yaw = wrap_half(math.pi / 2 + b[2])
        # lowered to just above the pile (not dropped from the rim: objects roll to the ends)
        pile_top = max([c["top_z"] for c in self.last_clusters if c["container"] == name] or [CONTAINER_FLOOR_Z])
        z_rel = min(CONTAINER_RIM_Z + DROP_ABOVE_RIM, pile_top + 0.03) + self.carry_hang
        if not hand_fits(self.containers, x, y, yaw, inside=name) and z_rel < CONTAINER_RIM_Z + 0.02:
            z_rel = CONTAINER_RIM_Z + DROP_ABOVE_RIM + self.carry_hang
        yield from self.goto((x, y, self.travel_z()), yaw, grip=1.0, tol=0.015)
        yield from self.goto((x, y, z_rel), yaw, grip=1.0, tol=0.01, max_steps=40, step_max=0.03)
        yield from self.hold(4, grip=0.0)
        self.carry_hang = 0.0
        yield from self.goto((x, y, TRAVEL_Z), yaw, grip=0.0, tol=0.02, max_steps=30)

    def place_on_desk(self, xy, yaw=0.0):
        """Set the held object down on the desk, its bottom ~2 cm above the top (tall objects
        dropped from higher topple)."""
        x, y = xy
        yield from self.goto((x, y, self.travel_z()), yaw, grip=1.0, tol=0.015)
        yield from self.goto((x, y, 0.02 + self.carry_hang), yaw, grip=1.0, tol=0.01, max_steps=30)
        yield from self.hold(4, grip=0.0)
        self.carry_hang = 0.0
        yield from self.goto((x, y, TRAVEL_Z), yaw, grip=0.0, tol=0.02, max_steps=30)

    def observe(self, settle=2, sweep=False):
        """Fused perception from the observe pose. The hand camera there looks straight down on the
        strip between the robot and the containers, which the container walls hide from the static
        cameras, except its two ends; `sweep` also looks at those from above (a low object at the
        +y end is invisible to every camera otherwise)."""
        extra = []
        if sweep:
            for y_end in (0.47, -0.47):
                yield from self.goto((0.21, y_end, 0.42), 0.0, grip=0.0, tol=0.02, max_steps=40)
                yield from self.hold(2, grip=0.0)
                extra.append((decode_png(self.obs["rgb"]["eye_in_hand"], "RGB"), decode_png(self.obs["depth"]["eye_in_hand"])) + self.cam_pose())
            self.last_sweep_t = self.t()
        yield from self.goto(OBSERVE_POS, 0.0, grip=0.0, tol=0.02)
        yield from self.hold(settle, grip=0.0)
        imgs = {cam: (decode_png(self.obs["rgb"][cam], "RGB"), decode_png(self.obs["depth"][cam])) for cam in CAMS}
        eih = (decode_png(self.obs["rgb"]["eye_in_hand"], "RGB"), decode_png(self.obs["depth"]["eye_in_hand"]))
        clusters, containers = perceive(imgs, extra=[(eih[0], eih[1]) + self.cam_pose()] + extra)
        self.containers = containers
        for c in clusters:
            c["cls_info"] = self.classify(c) if (c["container"] is None or c["n"] < 600) else None
        self.last_perceive_t = self.t()
        self.last_clusters = clusters
        # a container that looks empty to the static cameras may only hide a low object behind
        # its 19 cm walls: the hand camera checks from above before the inventory is dropped
        self.maybe_empty = [b for b in self.inventory if sum(c["n"] for c in clusters if c["container"] == b) < 15
                            and (self.inventory[b] or self.pile_unknown[b])]
        self.dump("observe", imgs["agentview"][0], clusters)
        return clusters

    def verify_empty(self, b):
        """Look into container b from above with the hand camera; True (and the inventory
        dropped) when nothing stands on its floor."""
        cb = self.containers[b]
        yield from self.goto((cb[0], cb[1], CONTAINER_RIM_Z + 0.24), 0.0, grip=0.0, tol=0.01, max_steps=40)
        yield from self.hold(2, grip=0.0)
        R_cam, t_cam = self.cam_pose()
        rgb = decode_png(self.obs["rgb"]["eye_in_hand"], "RGB")
        dep = decode_png(self.obs["depth"]["eye_in_hand"])
        pts, _ = backproject_rt(dep, rgb, None, None, R_cam, t_cam)
        lx, ly = container_local(cb, pts[:, 0], pts[:, 1])
        inside = (np.abs(lx) < CONTAINER_INNER_HALF[0] - 0.01) & (np.abs(ly) < CONTAINER_INNER_HALF[1] - 0.01) \
            & (pts[:, 2] > CONTAINER_FLOOR_Z + 0.012) & (pts[:, 2] < CONTAINER_RIM_Z)
        n = int(inside.sum())
        if n < 40:
            log("t=%d %s container is empty" % (self.t(), b))
            self.inventory[b] = []
            self.pile_unknown[b] = False
            return True
        log("t=%d %s container only looked empty (%d points on its floor)" % (self.t(), b, n))
        return False

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
                             "container": c["container"], "dims": [round(v, 3) for v in c["dims"]], "yaw": round(c["yaw"], 2),
                             "model": info["model"] if info else None, "d": round(info["d"], 3) if info else None,
                             "margin": round(info["class_margin"], 3) if info else None})
            with open(os.path.join(LOG, "t%06d_%s.json" % (self.t(), tag)), "w") as f:
                json.dump({"t": self.t(), "request": self.request(), "containers": {k: [round(v, 3) for v in b] for k, b in self.containers.items()}, "clusters": rows}, f)
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
        d = perceive_eih(rgb, dep, R_cam, t_cam, (x, y), c["floor"], c["top_z"], self.containers)
        if d is None:
            log("t=%d inspect: nothing under the hand camera" % self.t())
            self.dump("inspect_none", rgb, [])
            return None
        d["cls_info"] = combine(self.classify_eih(d), c.get("cls_info"))
        d["container"] = c["container"]
        self.dump("inspect", rgb, [d])
        return d

    # -- grasp planning -------------------------------------------------------------------------
    def choose_yaw(self, c, inside=None, band=None):
        """-> (xy, yaw) for a pinch of cluster c, or None: fingers across its narrowest side, the
        grasped width under the opening, and the hand clear of every container wall. The hand need
        not be centred on the object along the finger axis: the open fingers are 8 cm apart and the
        first one to touch pushes the object to the middle as they close, so near a wall the hand
        may sit up to ~3 cm off the object (inside a container its body fits only near the centre
        line). `band` = the points the fingers will close on (default: the top rim)."""
        x, y = c["grasp_xy"]
        p = c.get("pts")
        pref = wrap_half(c.get("slab_yaw", c["yaw"]))
        cands = [pref] + [wrap_half(pref + s * d) for d in (0.26, 0.52, 0.79, 1.05, 1.31) for s in (1, -1)] + [wrap_half(pref + 1.57)]
        if band is None and p is not None:
            band = p[p[:, 2] > c["top_z"] - 0.015]      # the top rim only: side walls seen obliquely inflate deeper slabs
        if band is None or len(band) < 10:
            yaw = next((yaw for yaw in cands if hand_fits(self.containers, x, y, yaw, inside=inside)), None)
            return None if yaw is None else ((x, y), yaw)
        half_open = GRASP_MAX_WIDTH / 2 - WIDTH_MARGIN / 2
        best = None
        for k, yaw in enumerate(cands):
            u = np.array([-math.sin(yaw), math.cos(yaw)])
            v = np.array([math.cos(yaw), math.sin(yaw)])
            sel = np.abs((band[:, :2] - (x, y)) @ v) < 0.02
            q = band[sel] if sel.sum() >= 8 else band
            proj = (q[:, :2] - (x, y)) @ u
            lo, hi = float(np.percentile(proj, 3)), float(np.percentile(proj, 97))
            w = hi - lo
            if w + WIDTH_MARGIN > GRASP_MAX_WIDTH:
                continue
            # hand offsets along the finger axis that keep the object between the open fingers,
            # nearest to centred first
            centre = (lo + hi) / 2
            span = half_open - w / 2
            for off in sorted(np.arange(-span, span + 1e-9, 0.005), key=lambda o: abs(o)):
                hx, hy = x + (centre + off) * u[0], y + (centre + off) * u[1]
                if hand_fits(self.containers, hx, hy, yaw, inside=inside):
                    cost = w + 0.2 * abs(off) + 0.0005 * k      # narrow first, then centred, ties to the earlier
                    if best is None or cost < best[0]:
                        best = (cost, (float(hx), float(hy)), yaw)
                    break
        return None if best is None else (best[1], best[2])

    def search_pinch(self, c, inside=None, band=None):
        """When the pinch at the centroid is too wide (a clamp across its jaw, scissors across the
        blades), look for one elsewhere on the object: candidate centres on a 1.2 cm grid over the
        top slab, every yaw; feasible = narrower than the opening with margin and enough of the
        object between the fingers; cost = width + 0.5 x distance from the centroid (torque).
        -> ((x, y), yaw) or None."""
        p = c.get("pts")
        if p is None or len(p) < 40:
            return None
        if band is None:
            band = p[p[:, 2] > c["top_z"] - 0.015]
        if len(band) < 20:
            return None
        cx, cy = c["grasp_xy"]
        lo, hi = band[:, :2].min(0), band[:, :2].max(0)
        best = None
        yaws = [k * math.pi / 12 - math.pi / 2 for k in range(12)]
        for gx in np.arange(lo[0], hi[0] + 1e-6, 0.012):
            for gy in np.arange(lo[1], hi[1] + 1e-6, 0.012):
                near = np.hypot(band[:, 0] - gx, band[:, 1] - gy)
                if near.min() > 0.008:                 # not on the object
                    continue
                dist = math.hypot(gx - cx, gy - cy)
                if dist > 0.045:                       # a pinch far out on a handle swings the object off
                    continue
                for yaw in yaws:
                    v = np.array([math.cos(yaw), math.sin(yaw)])
                    sel = np.abs((band[:, :2] - (gx, gy)) @ v) < 0.02
                    if sel.sum() < 15:
                        continue
                    u = np.array([-math.sin(yaw), math.cos(yaw)])
                    proj = (band[sel, :2] - (gx, gy)) @ u
                    lo_u, hi_u = np.percentile(proj, 3), np.percentile(proj, 97)
                    width = float(hi_u - lo_u)
                    if width + WIDTH_MARGIN > GRASP_MAX_WIDTH or abs(lo_u + hi_u) / 2 > 0.012:
                        continue                         # too wide, or the fingers would close off-centre
                    cost = width + 0.5 * dist
                    if best is not None and cost >= best[0]:
                        continue
                    if not hand_fits(self.containers, gx, gy, yaw, inside=inside):
                        continue
                    best = (cost, (float(gx), float(gy)), wrap_half(yaw))
        return None if best is None else (best[1], best[2])

    @staticmethod
    def widens_below_top(c):
        """Bottle-like: seen from above, the body shows as a ring around a narrower top (cap, neck)."""
        p = c.get("pts")
        if p is None or len(p) < 40:
            return False
        top, floor = c["top_z"], c["floor"]
        slab = p[p[:, 2] > top - 0.015]
        body = p[(p[:, 2] < top - 0.04) & (p[:, 2] > floor + 0.02)]
        if len(slab) < 10 or len(body) < 30:
            return False
        cen = slab[:, :2].mean(0)
        r_top = float(np.percentile(np.hypot(slab[:, 0] - cen[0], slab[:, 1] - cen[1]), 90))
        r_body = float(np.percentile(np.hypot(body[:, 0] - cen[0], body[:, 1] - cen[1]), 90))
        return r_body > r_top + 0.008

    def rim_grasp(self, c, inside=None):
        """Wide hollow object (bowl, mug, pitcher): one finger inside, one outside, straddling the wall
        at the rim. Hollow = the cloud dips at least RIM_MIN_DEPTH below the rim around its centre.
        -> ((x, y), yaw) with the finger axis along the radius, or None."""
        p = c.get("pts")
        top = c["top_z"]
        if p is None or len(p) < 40:
            return None
        slab = p[p[:, 2] > top - 0.03]
        if len(slab) < 20:
            return None
        cen = slab[:, :2].mean(0)
        radius = float(np.percentile(np.hypot(slab[:, 0] - cen[0], slab[:, 1] - cen[1]), 90))
        inner = p[np.hypot(p[:, 0] - cen[0], p[:, 1] - cen[1]) < 0.4 * radius]
        depth = top - (float(np.median(inner[:, 2])) if len(inner) >= 5 else top)
        if radius < RIM_MIN_RADIUS or depth < RIM_MIN_DEPTH:
            return None
        for k in range(12):
            th = k * math.pi / 6
            wx, wy = cen[0] + radius * math.cos(th), cen[1] + radius * math.sin(th)
            yaw = wrap_half(th - math.pi / 2)             # finger axis (-sin yaw, cos yaw) = radial
            if math.hypot(wx, wy) > REACH - 0.04 or math.hypot(wx, wy) < BASE_CLEAR + 0.02:
                continue
            if not hand_fits(self.containers, wx, wy, yaw, inside=inside):
                continue
            return (float(wx), float(wy)), yaw
        return None

    def plan_grasps(self, c, inside=None):
        """-> [(xy, yaw, below_top), ...] for cluster c, best first: a pinch across its narrow
        side when it fits the hand, then a rim grasp when the object is hollow. Empty = skip."""
        out = []
        xy = (c["grasp_xy"][0], c["grasp_xy"][1])
        height = c["top_z"] - c["floor"]
        rim = self.rim_grasp(c, inside=inside)
        pick = self.choose_yaw(c, inside=inside)
        shallow = [(pick[0], pick[1], GRASP_BELOW_TOP)] if pick is not None else []
        if rim is not None and (not shallow or c["len_minor"] > 0.055):
            # hollow and wide (bowl, mug, pitcher): the rim grasp, one finger inside, is the plan.
            # A pinch across it only "fits" when the hand camera saw half of it, and then a finger
            # lands on the rim and pushes it away
            return [(rim[0], rim[1], RIM_GRASP_BELOW_TOP)]
        if not shallow:
            alt = self.search_pinch(c, inside=inside)
            if alt is not None:
                shallow = [(alt[0], alt[1], GRASP_BELOW_TOP)]
        deep = []
        if height > 0.08 and c.get("pts") is not None:
            # around the middle of the body, checked against the body's own width
            p = c["pts"]
            depth = float(np.clip(0.45 * height, GRASP_BELOW_TOP + 0.015, 0.065))
            band = p[(p[:, 2] > c["top_z"] - depth - 0.015) & (p[:, 2] < c["top_z"] - depth + 0.015)]
            if len(band) < 10:
                band = p[p[:, 2] > c["floor"] + 0.02]
            pick_d = self.choose_yaw(c, inside=inside, band=band)
            if pick_d is not None:
                deep = [(pick_d[0], pick_d[1], depth)]
        # a bottle held by its cap slips out in transit (and still reads as held), so bottle-like
        # objects try the body first; everything else the top first, the body as the fallback
        out += (deep + shallow) if self.widens_below_top(c) else (shallow + deep)
        if rim is not None:
            out.append((rim[0], rim[1], RIM_GRASP_BELOW_TOP))
        return out

    def try_grasps(self, plans, top_z, floor, anchor_xy):
        """Attempt the planned grasps in order; stop at the first that holds."""
        for xy, yaw, below in plans:
            held = yield from self.pick(xy, yaw, top_z, floor, below_top=below, anchor_xy=anchor_xy)
            if held:
                return True
            log("t=%d grasp (yaw %.2f, depth %.3f) failed" % (self.t(), yaw, below))
        return False

    def free_spot(self, clusters, near=None):
        """A free desk spot for setting an object down: reachable, clear of the robot base and
        of other objects, and one where the open hand fits (to put it down and to take it back).
        The spot closest to `near` (x, y) wins."""
        occ = [c["centroid"][:2] for c in clusters]
        best = None
        for x in np.arange(0.17, 0.761, 0.02):
            for y in np.arange(-0.62, 0.621, 0.03):
                r = math.hypot(x, y)
                if r > REACH - 0.06 or r < BASE_CLEAR + 0.04:
                    continue
                if any(math.hypot(x - ox, y - oy) < 0.11 for ox, oy in occ):
                    continue
                if container_of_xy(self.containers, x, y, shrink=-0.06) is not None:
                    continue
                if not (hand_fits(self.containers, x, y, 0.0) or hand_fits(self.containers, x, y, math.pi / 2)):
                    continue
                d = math.hypot(x - near[0], y - near[1]) if near is not None else r
                if best is None or d < best[0]:
                    best = (d, (float(x), float(y)))
        return best[1] if best else None

    def spot_yaw(self, xy):
        return 0.0 if hand_fits(self.containers, xy[0], xy[1], 0.0) else math.pi / 2

    def loose(self, clusters):
        return [c for c in clusters if c["container"] is None and c["cls_info"] is not None]

    @staticmethod
    def _near_any(c, points, r=0.05):
        return any(math.hypot(c["centroid"][0] - x, c["centroid"][1] - y) < r for x, y in points)

    def skipped(self, c):
        for (x, y, t, dur) in self.skip_list:
            if math.hypot(x - c["centroid"][0], y - c["centroid"][1]) < 0.05 and self.t() - t < dur:
                return True
        return False

    def skip(self, c, dur=600):
        """Leave this spot alone for `dur` steps; a spot that keeps failing is left alone for longer
        (an object wedged between two containers is not worth a visit every few hundred steps)."""
        x, y = c["centroid"][0], c["centroid"][1]
        fails = sum(1 for (px, py, t, d) in self.skip_list if math.hypot(px - x, py - y) < 0.05 and d >= 600)
        self.skip_list.append((x, y, self.t(), dur * (1 + fails) ** 2 if dur >= 600 else dur))

    # -- main behaviour -----------------------------------------------------------------------
    def run(self):
        self.last_sweep_t = -10 ** 9
        while True:
            clusters = yield from self.observe(sweep=self.t() - self.last_sweep_t > 1000)
            for b in list(getattr(self, "maybe_empty", [])):
                yield from self.verify_empty(b)
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
            # no request, or a request that is stuck (nothing left to inspect or unpack): keep the
            # desk organized meanwhile instead of idling until the deadline
            did = yield from self.tidy(clusters)
            if did:
                continue
            yield from self.hold(10, grip=0.0)

    def held_back(self, c):
        """Something a request dug out of a container waits on the desk until the request closes
        (put back, it would be dug out again)."""
        rs = self.request_state
        return bool(rs) and self._near_any(c, rs.get("dug", []), r=0.07)

    def tidy(self, clusters):
        loose = [c for c in self.loose(clusters) if not self.skipped(c) and not self.held_back(c)]
        loose.sort(key=lambda c: -c["cls_info"]["class_margin"])
        for c in loose:
            d = yield from self.inspect(c)
            if d is None:
                self.skip(c)
                return True
            info = d["cls_info"]
            g = self.plan_grasps(d)
            log("t=%d loose %s (%s d=%.2f margin=%.2f; static %s) at (%.2f,%.2f) top=%.3f dims=%s grasps=%s" % (
                self.t(), info["model"], info["cls"], info["d"], info["class_margin"], c["cls_info"]["model"],
                d["grasp_xy"][0], d["grasp_xy"][1], d["top_z"], [round(v, 3) for v in d["dims"]], [(round(x[1], 2), x[2]) for x in g]))
            if info["class_margin"] < 0.08 or not g:
                self.skip(c)
                return True
            held = yield from self.try_grasps(g, d["top_z"], d["floor"], tuple(d["grasp_xy"]))
            if held:
                self.forget(info["model"])
                yield from self.place_in_container(CONTAINER_OF_CLASS[info["cls"]], info["model"])
            else:
                log("t=%d grasp failed" % self.t())
                self.skip(c)
            return True
        for c in clusters:
            if c["container"] in CLASS_OF_CONTAINER and c["cls_info"] is not None and not self.skipped(c) \
                    and c["cls_info"]["cls"] != CLASS_OF_CONTAINER[c["container"]] and c["cls_info"]["class_margin"] > 0.4:
                d = yield from self.inspect(c)
                if d is None or d["cls_info"]["cls"] == CLASS_OF_CONTAINER[c["container"]] or d["cls_info"]["class_margin"] < 0.4 \
                        or d["cls_info"].get("model_margin", 0.0) < 0.3:
                    self.skip(c)
                    return True
                g = self.plan_grasps(d, inside=c["container"])
                log("t=%d misplaced %s in %s grasps=%s" % (self.t(), d["cls_info"]["model"], c["container"], [(round(x[1], 2), x[2]) for x in g]))
                if not g:
                    self.skip(c)
                    return True
                held = yield from self.try_grasps(g, d["top_z"], d["floor"], tuple(d["grasp_xy"]))
                if held:
                    self.forget(d["cls_info"]["model"])
                    yield from self.place_in_container(CONTAINER_OF_CLASS[d["cls_info"]["cls"]], d["cls_info"]["model"])
                else:
                    self.skip(c)
                return True
        return False

    def forget(self, model):
        """`model` is out on the desk or in the hand: drop its latest inventory entry (an object
        that was dug out of a container is still listed there until it is sorted again)."""
        for b in sorted(self.inventory, key=lambda b: -len(self.inventory[b])):
            if model in self.inventory[b]:
                lst = self.inventory[b]
                del lst[len(lst) - 1 - lst[::-1].index(model)]
                return

    def could_be(self, c, model, slack):
        """Static-camera gate: is `model` within `slack` of this loose cluster's best match?"""
        info = c.get("cls_info") or {}
        if not info.get("all"):
            return True
        return info["all"].get(model, 9.0) - info["d"] < slack

    def search_order(self, model):
        """Containers that may hold `model`, most likely first: those the controller put an
        instance into (latest first), then its class container while an initial pile of unseen
        objects may still lie there."""
        names = (model, TWIN.get(model))
        order = [b for b in sorted(self.inventory, key=lambda b: -max([i for i, m in enumerate(self.inventory[b]) if m in names] or [-1]))
                 if any(m in names for m in self.inventory[b])]
        home = CONTAINER_OF_CLASS.get(CLASS_OF.get(model))
        if home and self.pile_unknown.get(home) and home not in order:
            order.append(home)
        return order

    def serve_request(self, clusters):
        """One step of serving the open request; returns False when there is nothing left to try
        (the caller tidies meanwhile). Order: loose objects that look like the target (static
        gate) -> containers that may hold it, digging from the top -> loose objects that looked
        unlike it -> the containers the controller had no reason to search -> declare absent."""
        rs = self.request_state
        model = rs["model"]
        loose = self.loose(clusters)

        def fresh(slack):
            cs = [c for c in loose if not self._near_any(c, rs["inspected"]) and self.could_be(c, model, slack)]
            return sorted(cs, key=lambda c: (c["cls_info"].get("all") or {}).get(model, 9.0))

        cands = fresh(0.5)
        if cands:
            done = yield from self.check_candidate(cands[0])
            return done
        order = self.search_order(model)
        expected = rs.setdefault("expected", list(order))
        searched = rs.setdefault("searched", [])
        if expected and all(b in searched for b in expected):
            # it was known to be in there but is not: the world moved it; look everywhere
            order += [b for b in ("gray", "lightblue", "blue") if b not in order]
        blocked = rs.setdefault("blocked", {})
        for b in order:
            if b in searched or self.t() - blocked.get(b, -999) < 150:
                continue
            inside = [c for c in clusters if c["container"] == b]
            if sum(c["n"] for c in inside) < 15:
                empty = yield from self.verify_empty(b)
                if empty:
                    searched.append(b)
                    continue
                # something low the static cameras cannot see over the walls: look from above
                cb = self.containers[b]
                inside = [{"grasp_xy": [cb[0], cb[1]], "top_z": CONTAINER_RIM_Z - 0.06, "floor": CONTAINER_FLOOR_Z,
                           "container": b, "n": 0, "centroid": [cb[0], cb[1], 0.05], "cls_info": None}]
            done = yield from self.unpack_one(b, inside, clusters)
            return done
        cands = fresh(1.0)
        if cands:
            done = yield from self.check_candidate(cands[0])
            return done
        if any(b not in searched for b in order) or rs.get("seen_target") or rs.get("out_of_reach"):
            # a container that may hold it could not be dug out (nothing in reach), or the target
            # was seen but not taken: never claim absence on a guess; tidy meanwhile, try again
            return False
        if not rs.get("swept"):
            rs["swept"] = True              # look into the blind ends of the near strip first
            self.last_sweep_t = -10 ** 9
            return True
        log("t=%d declare absent: %s (searched %s)" % (self.t(), model, rs.get("searched")))
        yield from self.hold(1, grip=0.0, mark=2)
        self.request_state = None
        return True

    def check_candidate(self, c):
        """Inspect a loose cluster from the hand camera; deliver it when it is the target."""
        rs = self.request_state
        model = rs["model"]
        key = (c["centroid"][0], c["centroid"][1])
        rs["inspected"].append(key)
        d = yield from self.inspect(c)
        if d is None:
            return True
        info = d["cls_info"]
        log("t=%d request check: %s (d=%.2f) vs wanted %s" % (self.t(), info["model"], info["d"], model))
        if not is_model(info, model):
            return True
        rs["seen_target"] = True            # it is here: absence is never claimed for this request
        g = self.plan_grasps(d)
        if not g:
            log("t=%d target not graspable where it lies" % self.t())
            return True
        held = yield from self.try_grasps(g, d["top_z"], d["floor"], tuple(d["grasp_xy"]))
        if not held:
            log("t=%d grasp of the target failed; retrying" % self.t())
            rs["inspected"].remove(key)
            return True
        self.forget(model)
        yield from self.place_in_container("red")
        yield from self.hold(8, grip=0.0)
        ok = yield from self.confirm_red()
        if ok:
            yield from self.hold(1, grip=0.0, mark=1)
            log("t=%d marked delivered" % self.t())
            self.request_state = None
        else:
            log("t=%d red container check failed; retrying" % self.t())
            rs["inspected"].remove(key)
        return True

    def unpack_one(self, b, inside, clusters):
        """Dig one object out of container b: the highest cluster with a grasp the hand fits inside
        the container, re-measured from above with the hand camera. The target itself goes
        straight to the red container; anything else is set down on a free desk spot nearby,
        where the next observation tells what it is."""
        rs = self.request_state
        model = rs["model"]
        # the static clusters of a pile are coarse and merged: look at the top from above first
        for c in sorted(inside, key=lambda c: -c["top_z"])[:2]:
            d = yield from self.inspect(c)
            if d is None:
                continue
            g = self.plan_grasps(d, inside=b)
            info = d["cls_info"]
            log("t=%d unpack %s: top %s (d=%.2f, model margin %.2f) at (%.2f,%.2f) top=%.3f grasps=%s" % (
                self.t(), b, info["model"], info["d"], info.get("model_margin", 0.0), d["grasp_xy"][0], d["grasp_xy"][1],
                d["top_z"], [(round(x[1], 2), x[2]) for x in g]))
            if not g:
                continue
            is_target = (info["model"] == model and info.get("model_margin", 0.0) > 0.3) or (is_model(info, model, 0.15) and info["model"] != model)
            spot = None if is_target else self.free_spot(clusters, near=(d["centroid"][0], d["centroid"][1]))
            if not is_target and spot is None:
                log("t=%d no free desk spot" % self.t())
                return False
            held = yield from self.try_grasps(g, d["top_z"], d["floor"], tuple(d["grasp_xy"]))
            if not held:
                break
            if is_target:
                self.forget(model)
                yield from self.place_in_container("red")
                yield from self.hold(8, grip=0.0)
                ok = yield from self.confirm_red()
                if ok:
                    yield from self.hold(1, grip=0.0, mark=1)
                    log("t=%d marked delivered (dug out of %s)" % (self.t(), b))
                    self.request_state = None
                return True
            yield from self.place_on_desk(spot, self.spot_yaw(spot))
            rs.setdefault("dug", []).append(spot)
            return True
        # nothing in reach: what is left is exposed now; identify it from above. Absence can be
        # claimed once every object left in the container is known and none is the target.
        if not rs.setdefault("identified", {}).get(b):
            rs["identified"][b] = True
            ids, sure = [], True
            for c in sorted(inside, key=lambda c: -c["n"])[:4]:
                d = yield from self.inspect(c)
                if d is None:
                    continue
                info = d["cls_info"]
                ids.append(info["model"])
                if info["model"] == model or TWIN.get(info["model"]) == model or info.get("model_margin", 0.0) < 0.15:
                    sure = False
            log("t=%d %s: left out of reach %s (%s)" % (self.t(), b, ids, "no target" if sure else "maybe the target"))
            if sure:
                self.inventory[b] = list(ids)        # what was dug out is re-listed when it is put back
                self.pile_unknown[b] = False
                rs.setdefault("searched", []).append(b)
                return True
            if model in ids or any(TWIN.get(m) == model for m in ids):
                rs["out_of_reach"] = b
        rs.setdefault("blocked", {})[b] = self.t()
        log("t=%d nothing in reach in %s for now" % (self.t(), b))
        return True

    def confirm_red(self):
        b = self.containers["red"]
        x, y = b[0], b[1]
        yield from self.goto((x, y, CONTAINER_RIM_Z + 0.16), 0.0, grip=0.0, tol=0.01, max_steps=30)
        yield from self.hold(2, grip=0.0)
        R_cam, t_cam = self.cam_pose()
        rgb = decode_png(self.obs["rgb"]["eye_in_hand"], "RGB")
        dep = decode_png(self.obs["depth"]["eye_in_hand"])
        d = perceive_eih(rgb, dep, R_cam, t_cam, (x, y), CONTAINER_FLOOR_Z, CONTAINER_RIM_Z, self.containers, radius=0.14)
        self.dump("confirm_red", rgb, [d] if d else [])
        return d is not None and container_of_xy(self.containers, d["centroid"][0], d["centroid"][1]) == "red"


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
