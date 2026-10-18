#!/usr/bin/env python3
"""Reference controller for object_finding_and_organizing_on_messy_desk.

Works from the observation only (the two static RGB-D cameras, proprioception, the request
text). Pipeline per decision:

  depth back-projection (both static cameras) -> drop desk, bin walls, bin floors, robot
  -> voxel 26-connectivity clustering -> per-cluster colour histogram + sorted extents
  -> nearest reference (refs.json, built from the practice simulator's `lab` command)
  -> plan: sort loose objects into their class bins; on a request, deliver the named object
     (unpacking its class bin, then the other bins, onto the desk when it is not loose) and
     mark 1; mark 2 only after every bin has been unpacked and the model was never seen.

Closed-loop Cartesian control: every step commands the clipped delta towards the current
goal, so a stale or perturbed proprioceptive reading only costs one step.
"""
import base64
import io
import json
import math
import os
import sys
import time
from collections import deque

import numpy as np
from PIL import Image

# ----------------------------------------------------------------------------------------
# Task geometry (task_card.md)
# ----------------------------------------------------------------------------------------
BIN_X = 0.46
BIN_YS = {"gray": -0.375, "lightblue": -0.125, "blue": 0.125, "red": 0.375}
BIN_INNER_HALF = (0.135, 0.085)
BIN_OUTER = (0.297, 0.198, 0.146)
BIN_FLOOR_Z, BIN_RIM_Z = 0.012, 0.146
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
CAMS = {
    "agentview": {"f": 171.03317, "c": 112.0,
                  "pose": [1.1, 0.0, 0.7, 0.0, -0.37224140238349257, 0.0, 0.9281359482056336]},
    "left_view": {"f": 171.03317, "c": 112.0,
                  "pose": [0.45, -1.15, 0.75, 0.6808448119669411, -0.19091972663321244, 0.19091972663321244, 0.6808448119669411]},
}
POS_STEP_MAX, ROT_STEP_MAX = 0.05, 0.25
GRIPPER_OPEN_W = 0.08

# controller tunables
OBSERVE_POS = (0.30, 0.0, 0.55)
TRAVEL_Z = 0.30
VOXEL = 0.012
Z_ROBOT = 0.28
GRASP_BELOW_TOP = 0.025
GRASP_MIN_ABOVE_FLOOR = 0.016
DROP_ABOVE_RIM = 0.05
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


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def wrap_half(a):
    """Wrap to (-pi/2, pi/2]: a gripper yaw is only defined modulo pi."""
    return (a + math.pi / 2) % math.pi - math.pi / 2


# ----------------------------------------------------------------------------------------
# Perception
# ----------------------------------------------------------------------------------------
def decode_png(b64, mode=None):
    im = Image.open(io.BytesIO(base64.b64decode(b64)))
    if mode:
        im = im.convert(mode)
    return np.array(im)


def backproject(depth_mm, rgb, cam, max_depth=3.0):
    f, c = cam["f"], cam["c"]
    z = depth_mm.astype(np.float32) / 1000.0
    h, w = z.shape
    v, u = np.mgrid[0:h, 0:w]
    valid = (z > 0.05) & (z < max_depth)
    pts = np.stack([z[valid], -(u[valid] - c) / f * z[valid], -(v[valid] - c) / f * z[valid]], 1)
    pose = cam["pose"]
    world = pts @ quat_to_mat(pose[3:7]).T + np.asarray(pose[:3])
    return world, rgb[valid].astype(np.float32) / 255.0


def backproject_rt(depth_mm, rgb, f, c, R, t, max_depth=1.5):
    z = depth_mm.astype(np.float32) / 1000.0
    h, w = z.shape
    v, u = np.mgrid[0:h, 0:w]
    valid = (z > 0.02) & (z < max_depth)
    pts = np.stack([z[valid], -(u[valid] - c) / f * z[valid], -(v[valid] - c) / f * z[valid]], 1)
    return pts @ R.T + np.asarray(t), rgb[valid].astype(np.float32) / 255.0


def perceive_eih(rgb, depth, R_cam, t_cam, expected_xy, floor, top_hint, radius=0.09, voxel=0.006):
    """Dense cluster of the object under the hand camera. Points above the desk / bin floor,
    within `radius` of the expected position, below the fingers, not bin wall."""
    pts, col = backproject_rt(depth, rgb, CAMS["agentview"]["f"], CAMS["agentview"]["c"], R_cam, t_cam)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    keep = (z > floor + 0.010) & (z < top_hint + 0.06) & (np.hypot(x - expected_xy[0], y - expected_xy[1]) < radius)
    ox, oy, oz = BIN_OUTER
    ix, iy = BIN_INNER_HALF
    for by in BIN_YS.values():
        in_outer = (np.abs(x - BIN_X) <= ox / 2 + 0.012) & (np.abs(y - by) <= oy / 2 + 0.012)
        in_inner = (np.abs(x - BIN_X) <= ix - 0.006) & (np.abs(y - by) <= iy - 0.006)
        keep &= ~(in_outer & ~in_inner & (z < oz + 0.02))
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
    c = cluster_features(pts[best], col[best])
    c["floor"] = floor
    c["dims"] = sorted([c["len_major"], c["len_minor"], c["top_z"] - floor])
    return c


def bin_of_xy(x, y, shrink=0.0):
    for name, by in BIN_YS.items():
        if abs(x - BIN_X) <= BIN_INNER_HALF[0] - shrink and abs(y - by) <= BIN_INNER_HALF[1] - shrink:
            return name
    return None


def scene_mask(pts):
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    keep = (x > DESK_X[0]) & (x < 0.76) & (y > DESK_Y[0]) & (y < DESK_Y[1]) & (z > 0.02) & (z < Z_ROBOT)
    ox, oy, oz = BIN_OUTER
    ix, iy = BIN_INNER_HALF
    for by in BIN_YS.values():
        in_outer = (np.abs(x - BIN_X) <= ox / 2 + 0.012) & (np.abs(y - by) <= oy / 2 + 0.012)
        in_inner = (np.abs(x - BIN_X) <= ix - 0.008) & (np.abs(y - by) <= iy - 0.008)
        keep &= ~(in_outer & ~in_inner & (z < oz + 0.02))
        keep &= ~(in_inner & (z < BIN_FLOOR_Z + 0.015))
    return keep


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


def cluster_features(p, c):
    cen = p.mean(0)
    top = float(np.percentile(p[:, 2], 97))
    xy = p[:, :2] - cen[:2]
    cov = xy.T @ xy / max(len(p), 1)
    evals, evecs = np.linalg.eigh(cov)
    major = evecs[:, 1]
    yaw = float(np.arctan2(major[1], major[0]))
    proj = xy @ evecs
    ext = np.percentile(proj, 97, axis=0) - np.percentile(proj, 3, axis=0)
    b = bin_of_xy(cen[0], cen[1])
    floor = BIN_FLOOR_Z if b else 0.0
    # grasp point: the xy centroid of the top slab (more reliable than the full centroid for
    # leaning or partially seen objects)
    slab = p[p[:, 2] > top - 0.03]
    gxy = slab[:, :2].mean(0) if len(slab) >= 5 else cen[:2]
    return {"n": int(len(p)), "centroid": cen.tolist(), "grasp_xy": [float(gxy[0]), float(gxy[1])],
            "top_z": top, "floor": floor, "bin": b, "yaw": yaw,
            "len_major": float(ext[1]), "len_minor": float(ext[0]),
            "dims": sorted([float(ext[1]), float(ext[0]), float(top - floor)]),
            "mean_rgb": c.mean(0).tolist(), "hist": colour_hist(rgb_to_hsv(c)).tolist()}


def perceive(images, voxel=VOXEL, min_pts=12):
    """images: {cam: (rgb uint8, depth uint16)} for the static cameras -> cluster list."""
    P, C = [], []
    for cam in CAMS:
        rgb, d = images[cam]
        pts, col = backproject(d, rgb, CAMS[cam])
        m = scene_mask(pts)
        P.append(pts[m])
        C.append(col[m])
    pts, col = np.concatenate(P), np.concatenate(C)
    if len(pts) == 0:
        return []
    lo = pts.min(0) - voxel
    idx = np.floor((pts - lo) / voxel).astype(int)
    labels, n = label_components(idx)
    out = []
    for k in range(n):
        m = labels == k
        if m.sum() < min_pts:
            continue
        out.append(cluster_features(pts[m], col[m]))
    return out


class Classifier(object):
    def __init__(self, refs):
        self.refs = refs

    def dist(self, cl, r):
        return hist_dist(cl["hist"], r["hist"]) + 8.0 * float(np.abs(np.array(cl["dims"]) - np.array(r["dims"])).sum())

    def __call__(self, cl):
        best = {}
        for r in self.refs:
            d = self.dist(cl, r)
            if r["model"] not in best or d < best[r["model"]]:
                best[r["model"]] = d
        ranked = sorted(best.items(), key=lambda t: t[1])
        model, d0 = ranked[0]
        cls = CLASS_OF[model]
        other = [d for m, d in ranked if CLASS_OF[m] != cls]
        margin = (other[0] - d0) if other else 1.0
        return {"model": model, "cls": cls, "d": d0, "class_margin": margin, "ranked": ranked[:3]}


# ----------------------------------------------------------------------------------------
# Controller (generator based: `yield action` receives the next observation)
# ----------------------------------------------------------------------------------------
class Controller(object):
    def __init__(self, refs):
        if isinstance(refs, list):            # legacy flat reference list
            refs = {"static": refs}
        self.classify = Classifier(refs.get("static", []))
        self.classify_eih = Classifier(refs.get("eih", []))
        rel = refs.get("eih_rel")
        self.eih_R = np.array(rel["R"], float) if rel else np.eye(3)
        self.eih_t = np.array(rel["t"], float) if rel else np.zeros(3)
        self.reset()

    def reset(self):
        self.obs = None
        self.q_home = None
        self.gen = None
        self.last_t, self.policy = None, None
        self.seen_models = set()
        self.unpacked = set()
        self.request_text = None
        self.request_state = None
        self.drop_slot = {b: 0 for b in BIN_YS}
        self.spots_used = {}
        self.last_perceive_t = -999
        self.n_marks = 0

    # -- wire entry -------------------------------------------------------------------------
    def act(self, t, obs):
        """The behaviour generator advances once per step index; every call (including a
        repeated query of the same t) recomputes the action from the observation it was
        given, so the answer genuinely depends on the input."""
        if self.q_home is None:
            self.obs = obs
            self.q_home = np.array(obs["ee_pose"][3:7], float)
            self.gen = self.run()
            self.policy = next(self.gen)
        elif t != self.last_t:
            self.obs = obs
            self.policy = self.gen.send(obs)
        self.last_t = t
        return [float(x) for x in self.policy(obs)]

    # -- proprio helpers ------------------------------------------------------------------
    def ee(self):
        return np.array(self.obs["ee_pose"][:3], float)

    def yaw(self, obs=None):
        q = np.array((obs or self.obs)["ee_pose"][3:7], float)
        rel = quat_mul(q, quat_conj(self.q_home))
        return wrap(2.0 * math.atan2(rel[3], rel[0]))

    def width(self):
        return float(self.obs["gripper_width"])

    def t(self):
        return int(self.obs["t"])

    def request(self):
        return self.obs.get("request")

    # -- motion primitives --------------------------------------------------------------------
    def servo(self, pos, yaw, grip, mark=0):
        """A per-step policy: the clipped Cartesian delta from the observed tool pose to the
        goal (recomputed from whatever observation it is applied to)."""
        pos = np.asarray(pos, float)

        def policy(o):
            err = pos - np.array(o["ee_pose"][:3], float)
            d = np.clip(err, -POS_STEP_MAX, POS_STEP_MAX)
            r = float(np.clip(wrap(yaw - self.yaw(o)), -ROT_STEP_MAX, ROT_STEP_MAX))
            return [d[0], d[1], d[2], 0.0, 0.0, r, grip, mark]
        return policy

    def goto(self, pos, yaw=0.0, grip=0.0, tol=0.01, yaw_tol=0.04, max_steps=60, mark=0):
        pos = np.asarray(pos, float)
        for i in range(max_steps):
            err = pos - self.ee()
            dyaw = wrap(yaw - self.yaw())
            if np.linalg.norm(err) < tol and abs(dyaw) < yaw_tol:
                return True
            yield self.servo(pos, yaw, grip, mark)
            mark = 0
        return False

    def hold(self, steps, grip, mark=0):
        """Stay where we are (servoing on the pose at entry) for `steps` steps."""
        pos, yaw = self.ee(), self.yaw()
        for i in range(steps):
            yield self.servo(pos, yaw, grip, mark)
            mark = 0

    def gripper_closed_on_object(self):
        return 0.006 < self.width() < 0.075

    def pick(self, xy, yaw, top_z, floor):
        """Top-down pinch grasp. Returns True when something is held after lifting."""
        x, y = xy
        yaw = wrap_half(yaw)
        z_pre = max(top_z + 0.10, floor + 0.12)
        z_travel = max(TRAVEL_Z, z_pre)
        yield from self.goto((x, y, z_travel), yaw, grip=0.0, tol=0.015)
        yield from self.goto((x, y, z_pre), yaw, grip=0.0, tol=0.008, max_steps=30)
        z_g = max(top_z - GRASP_BELOW_TOP, floor + GRASP_MIN_ABOVE_FLOOR)
        yield from self.goto((x, y, z_g), yaw, grip=0.0, tol=0.006, max_steps=25)
        yield from self.hold(6, grip=1.0)
        held = self.gripper_closed_on_object()
        yield from self.goto((x, y, z_travel), yaw, grip=1.0, tol=0.015, max_steps=30)
        yield from self.hold(2, grip=1.0)
        held = held and self.gripper_closed_on_object()
        return held

    def place_in_bin(self, name):
        """Carry the held object over bin `name`, lower, release. Cycles three drop slots."""
        slot = self.drop_slot[name] % 3
        self.drop_slot[name] += 1
        x = BIN_X + (-0.07, 0.0, 0.07)[slot]
        y = BIN_YS[name] + (0.0, 0.03, -0.03)[slot]
        z_rel = BIN_RIM_Z + DROP_ABOVE_RIM
        yaw = math.pi / 2 if name != "red" else 0.0     # fingers along the bin's long side
        yield from self.goto((x, y, TRAVEL_Z), yaw, grip=1.0, tol=0.015)
        yield from self.goto((x, y, z_rel), yaw, grip=1.0, tol=0.01, max_steps=30)
        yield from self.hold(4, grip=0.0)
        yield from self.goto((x, y, TRAVEL_Z), yaw, grip=0.0, tol=0.02, max_steps=30)

    def place_on_desk(self, xy):
        x, y = xy
        yield from self.goto((x, y, TRAVEL_Z), 0.0, grip=1.0, tol=0.015)
        yield from self.goto((x, y, 0.09), 0.0, grip=1.0, tol=0.01, max_steps=30)
        yield from self.hold(4, grip=0.0)
        yield from self.goto((x, y, TRAVEL_Z), 0.0, grip=0.0, tol=0.02, max_steps=30)

    def observe(self, settle=2):
        yield from self.goto(OBSERVE_POS, 0.0, grip=0.0, tol=0.02)
        yield from self.hold(settle, grip=0.0)
        imgs = {cam: (decode_png(self.obs["rgb"][cam], "RGB"), decode_png(self.obs["depth"][cam]))
                for cam in CAMS}
        clusters = perceive(imgs)
        for c in clusters:
            c["cls_info"] = self.classify(c) if (c["bin"] is None or c["n"] < 600) else None
        self.last_perceive_t = self.t()
        return clusters

    def cam_pose(self):
        """World pose of the hand camera from the tool pose and the fixed ee->camera transform."""
        q = np.array(self.obs["ee_pose"][3:7], float)
        R_e = quat_to_mat(q)
        t_e = self.ee()
        return R_e @ self.eih_R, t_e + R_e @ self.eih_t

    def inspect(self, c):
        """Hover over a cluster and classify it from the hand camera (dense cloud)."""
        x, y = c["grasp_xy"]
        yield from self.goto((x, y, max(TRAVEL_Z, c["top_z"] + 0.13)), 0.0, grip=0.0, tol=0.015)
        yield from self.goto((x, y, c["top_z"] + 0.13), 0.0, grip=0.0, tol=0.006, yaw_tol=0.03, max_steps=40)
        yield from self.hold(3, grip=0.0)
        R_cam, t_cam = self.cam_pose()
        rgb = decode_png(self.obs["rgb"]["eye_in_hand"], "RGB")
        dep = decode_png(self.obs["depth"]["eye_in_hand"])
        d = perceive_eih(rgb, dep, R_cam, t_cam, (x, y), c["floor"], c["top_z"])
        if d is None:
            log("t=%d inspect: nothing under the hand camera" % self.t())
            return None
        d["cls_info"] = self.classify_eih(d)
        d["bin"] = c["bin"]
        return d

    # -- planning helpers ---------------------------------------------------------------------
    def free_spot(self, clusters):
        cands = [(0.66, 0.0), (0.66, -0.2), (0.66, 0.2), (0.66, -0.4), (0.66, 0.4),
                 (0.26, -0.15), (0.26, 0.15), (0.26, -0.35), (0.26, 0.35),
                 (0.40, -0.56), (0.52, -0.56), (0.40, 0.56), (0.52, 0.56)]
        occ = [c["centroid"][:2] for c in clusters]
        for x, y in cands:
            if math.hypot(x, y) > REACH - 0.06:
                continue
            if any(math.hypot(x - ox, y - oy) < 0.11 for ox, oy in occ):
                continue
            return (x, y)
        return None

    def loose(self, clusters):
        return [c for c in clusters if c["bin"] is None and c["cls_info"] is not None]

    def grasp_yaw(self, c):
        """Fingers close across the minor axis: rotate the hand by the major-axis angle."""
        return wrap_half(c["yaw"])

    # -- main behaviour -----------------------------------------------------------------------
    def run(self):
        while True:
            clusters = yield from self.observe()
            req = self.request()
            if req is None:
                self.request_state = None
            elif req["text"] != self.request_text or self.request_state is None:
                self.request_text = req["text"]
                self.request_state = {"model": req["text"].replace("the ", "", 1), "unpacked": [], "deadline": req["deadline_t"]}
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
        """One sorting action: a loose object to its class bin, or a visibly misplaced object."""
        loose = sorted(self.loose(clusters), key=lambda c: -c["cls_info"]["class_margin"])
        for c in loose:
            info = c["cls_info"]
            if info["class_margin"] < 0.0:
                continue
            log("t=%d sort %s (%s, d=%.2f, margin=%.2f) at (%.2f,%.2f) top=%.3f" % (
                self.t(), info["model"], info["cls"], info["d"], info["class_margin"], c["grasp_xy"][0], c["grasp_xy"][1], c["top_z"]))
            held = yield from self.pick(c["grasp_xy"], self.grasp_yaw(c), c["top_z"], c["floor"])
            if held:
                yield from self.place_in_bin(BIN_OF_CLASS[info["cls"]])
            else:
                yield from self.hold(1, grip=0.0)
                log("t=%d grasp failed" % self.t())
            return True
        # misplaced object visible in a bin (small, isolated cluster)
        for c in clusters:
            if c["bin"] in CLASS_OF_BIN and c["cls_info"] is not None and c["cls_info"]["class_margin"] > 0.25 \
                    and c["cls_info"]["cls"] != CLASS_OF_BIN[c["bin"]]:
                log("t=%d misplaced %s in %s" % (self.t(), c["cls_info"]["model"], c["bin"]))
                held = yield from self.pick(c["grasp_xy"], self.grasp_yaw(c), c["top_z"], c["floor"])
                if held:
                    yield from self.place_in_bin(BIN_OF_CLASS[c["cls_info"]["cls"]])
                return True
        return False

    def serve_request(self, clusters):
        rs = self.request_state
        model = rs["model"]
        cls = CLASS_OF.get(model)
        # 1. the named object among the loose ones?
        cands = [c for c in self.loose(clusters) if c["cls_info"]["model"] == model]
        if cands:
            c = min(cands, key=lambda c: c["cls_info"]["d"])
            log("t=%d deliver %s d=%.2f" % (self.t(), model, c["cls_info"]["d"]))
            held = yield from self.pick(c["grasp_xy"], self.grasp_yaw(c), c["top_z"], c["floor"])
            if held:
                yield from self.place_in_bin("red")
                yield from self.hold(8, grip=0.0)
                yield from self.hold(1, grip=0.0, mark=1)
                log("t=%d marked delivered" % self.t())
                self.request_state = None
            return True
        # 2. unpack: class bin first, then the others
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
            log("t=%d unpack %s: cluster n=%d top=%.3f -> (%.2f,%.2f)" % (self.t(), b, c["n"], c["top_z"], spot[0], spot[1]))
            held = yield from self.pick(c["grasp_xy"], math.pi / 2, c["top_z"], c["floor"])
            if held:
                yield from self.place_on_desk(spot)
            return True
        # 3. everything is on the desk and the model was not seen: absent
        log("t=%d declare absent: %s" % (self.t(), model))
        yield from self.hold(1, grip=0.0, mark=2)
        self.request_state = None
        return True


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
