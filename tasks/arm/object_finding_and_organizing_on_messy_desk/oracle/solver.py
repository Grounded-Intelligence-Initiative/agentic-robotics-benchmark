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

# Single-threaded BLAS before numpy loads: the controller's arrays are small, and OpenBLAS's
# worker pool (one thread per host core, inside a container allowed fewer) once dead-locked a
# matrix-vector product, which froze the controller for the rest of the episode.
for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import numpy as np
from PIL import Image

# ----------------------------------------------------------------------------------------
# Task geometry (task_card.md)
# ----------------------------------------------------------------------------------------
# nominal centres (task card): red in front of the base, the class containers behind it, the middle
# one set back behind red and the outer two forward (all of their floor within the arm's reach)
CONTAINER_NOMINAL = {"gray": (0.44, -0.31), "lightblue": (0.575, 0.0), "blue": (0.44, 0.31), "red": (0.25, 0.0)}
CONTAINER_OUTER = (0.326, 0.272, 0.120)          # class containers: outer footprint x (long), y (short), wall height
RED_OUTER = (0.237, 0.272, 0.120)                # the delivery container is shorter along x
CONTAINER_OUTERS = {n: (RED_OUTER if n == "red" else CONTAINER_OUTER) for n in CONTAINER_NOMINAL}
CONTAINER_INNER_HALF = (0.140, 0.120)
CONTAINER_INNER_HALVES = {n: ((0.102, 0.120) if n == "red" else CONTAINER_INNER_HALF) for n in CONTAINER_NOMINAL}
CONTAINER_FLOOR_Z, CONTAINER_RIM_Z = 0.0024, 0.1206
NEAR_STRIP_X = 0.27            # in front of this x the hand camera's views add to the static ones (the strip at the base)
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


CLAMPS = ("large clamp", "extra large clamp")
CLAMP_SPLIT = 0.14      # longest extent (m) seen by the hand camera: the large clamp reads about 0.125, the extra large 0.155-0.19


def is_model(info, model, slack=0.25):
    """The classification `info` names `model`, or its look-alike twin with `model` close behind
    (not for the clamps: those are told apart by size in clamp_by_size)."""
    if info["model"] == model:
        return True
    if info["model"] in CLAMPS:
        return False
    return TWIN.get(info["model"]) == model and info.get("all", {}).get(model, 9.0) - info["d"] < slack


def clamp_by_size(info, dims):
    """The two clamps are one shape at two sizes and their per-model distances barely differ, so
    when the nearest model is a clamp the hand camera's longest extent names it, and the model
    margin is taken against the nearest non-clamp model. (An extra large clamp delivered for a
    requested large one was a wrong claim.)"""
    if not info or info.get("model") not in CLAMPS:
        return info
    info = dict(info)
    info["model"] = "extra large clamp" if max(dims) > CLAMP_SPLIT else "large clamp"
    others = [d for m, d in info.get("all", {}).items() if m not in CLAMPS]
    if others:
        info["model_margin"] = min(others) - info["d"]
    return info
DESK_X, DESK_Y = (0.15, 0.78), (-0.62, 0.62)
REACH, BASE_CLEAR = 0.80, 0.36
IMG = 320                                   # task cameras are IMG x IMG; focal scales with the width
F_PX, C_PX = 171.03317 * IMG / 224.0, IMG / 2.0
CAMS = {
    "agentview": {"f": F_PX, "c": C_PX,
                  "pose": [1.35, 0.0, 0.95, 0.0, -0.375099, 0.0, 0.926985]},
    "left_view": {"f": F_PX, "c": C_PX,
                  "pose": [0.45, -1.15, 0.75, 0.6808448119669411, -0.19091972663321244, 0.19091972663321244, 0.6808448119669411]},
    "oblique": {"f": F_PX, "c": C_PX,          # front-left corner of the desk, looking at the containers
                "pose": [1.25, -0.95, 0.85, 0.3981, -0.2464, 0.1119, 0.8765]},
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
GRIP_SETTLE_MAX = 12        # steps to wait for the fingers to stop after the close command
DRAG_BAND_X, DRAG_BAND_Y = 0.03, 0.012   # container-local half widths of the cross where a straight pinch fits a floor object
DRAG_STEP = 0.015           # one dragging move
DRAG_OVERSHOOT = 0.02       # dragged this much past the band's edge
DRAG_PRESS = 0.012          # fingertips commanded this far below the object's top: the position servo presses
DRAG_MIN = 0.015            # shorter drags are not worth a cycle
DRAG_TRIES_MAX = 2          # drags of one spot per request
DRAG_ENABLED = False        # press-and-drag: the fingertip on a can's top rim tips it or pushes the finger shut (lab, 2026-10-24); kept for later work

# controller tunables
OBSERVE_POS = (0.30, 0.0, 0.55)
TRAVEL_Z = 0.38               # carry height: well above the walls and whatever stands in a pile
VOXEL = 0.012
Z_ROBOT = 0.30
GRASP_BELOW_TOP = 0.03
RIM_GRASP_BELOW_TOP = 0.022     # wide hollow objects (bowl, mug, pitcher): straddle the wall at the rim
RIM_MIN_RADIUS, RIM_MIN_DEPTH = 0.035, 0.012
REFINE_RADIUS, REFINE_MAX_SHIFT = 0.07, 0.04    # hand-camera re-centring just before the descent
REFINE_NEAR = 0.05           # the re-centring looks at the top around the anchor only, not a touching neighbour's
TOP_UNDER_RADIUS = 0.025     # the top that sets the finger height: right under the grasp point
TOP_DROP_MIN = 0.015         # ... used when it lies this far below the planned top
GRASP_MIN_ABOVE_FLOOR = 0.032
DROP_ABOVE_RIM = 0.035
HOVER_ABOVE_TOP = 0.16
PATROL_EVERY = 1200          # idle steps between looks into the class containers (world misplacements)
SEARCH_ALL_MARGIN = 500      # steps left before a deadline that still allow digging in an unlikely container
FINISH_MARGIN = 350           # steps one more tidy action may take before a request's deadline
RETRY_MAX, RETRY_JITTER, RETRY_YAW = 1, 0.012, 0.15   # grasp attempts per object (a shifted retry rescued 0 of 23 failed grasps in the lab and the episode logs, so one), shift across the fingers (m), turn (rad)
MARK_LAST = 20                # steps before a request's deadline at which the mark goes out whatever the arm is doing
DESCENT_SHORT = 0.015         # a final descent that stops this far from the grasp point does not close
STUCK_ERR, STUCK_AFTER = 0.10, 4   # moves ending this far off, this many in a row: the arm is folded up, recover (a descent that stops 3-4 cm short at the edge of reach is not that)
HOLLOW = ("bowl", "mug", "pitcher base")
RED_REACH_X = 0.27            # an object in the red container nearer the base than this is left alone (the hand stops above it)
TIDY_R = (0.40, 0.76)         # loose objects nearer the base or farther out than this are left alone: a straight-down hand there stops short of the desk (measured: 0 of 5 grasps at r < 0.40, under half beyond 0.76), and every attempt costs a stuck-arm recovery; one episode spent its second half in that loop over a clamp pushed to r = 0.80
UNPACK_FAILS_MAX = 2          # failed grasps at one spot of a container within one request before the dig moves on
STALL_ROUNDS = 2              # rounds of "nothing in reach" in a container before the other containers are dug as well
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


def yaw_branch(xy, yaw, tilt=None):
    """Of the two hand yaws that are the same grasp for the symmetric hand (yaw and yaw + pi),
    the one that keeps the wrist's twist small relative to the arm's azimuth toward (x, y). On
    the +y side of the desk a hand turned to -90 degrees ran a joint into its limit (the descent
    stopped short or the arm folded up in 19 of 34 grasps in the episode logs; turned to +90
    degrees, 1 of 7). The lean, defined in the hand's frame, flips with the half turn."""
    phi = math.atan2(xy[1], xy[0])
    y2 = wrap(phi + wrap_half(yaw - phi))
    if is_tilted(tilt) and abs(wrap(y2 - yaw)) > 1.0:
        tilt = (-tilt[0], -tilt[1])
    return y2, tilt


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
    return {n: (x, y, 0.0) for n, (x, y) in CONTAINER_NOMINAL.items()}


def outer_half(name):
    o = CONTAINER_OUTERS.get(name, CONTAINER_OUTER)
    return o[0] / 2, o[1] / 2


def inner_half(name):
    return CONTAINER_INNER_HALVES.get(name, CONTAINER_INNER_HALF)


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
        ix, iy = inner_half(name)
        if abs(lx) <= ix - shrink and abs(ly) <= iy - shrink:
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
    for name, b in containers.items():
        ox, oy = outer_half(name)
        ix, iy = inner_half(name)
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


HAND_BODY_ABOVE_TCP = 0.034     # the hand body's underside above the tool centre point (measured)
FINGER_BELOW_TCP = 0.009        # open fingertips below the tool centre point (contact probe: a can's top meets the
                                # finger with the tool centre 0.9 cm above it; the earlier 0.028 was wrong)
RIM_CLEAR = 0.012               # a hand point this far above a rim passes over that container
# hand tilts tried when the straight-down hand does not fit: (lean along the finger axis, lean
# along the thickness axis), radians; the wrist leans the opposite way, away from the near wall
TILTS = [(0.0, 0.0)] + [(sg * a, 0.0) for a in (0.30, 0.45) for sg in (1, -1)] \
    + [(0.0, sg * b) for b in (0.30, 0.45) for sg in (1, -1)] \
    + [(sa * a, sb * 0.30) for a in (0.35, 0.55) for sa in (1, -1) for sb in (1, -1)]


def _hand_points():
    """Sample points of the open hand in its own frame (finger axis, thickness axis, up), metres
    from the tool centre point: the body, the two fingers, the wrist above."""
    pts = []
    for su in (-1.0, -0.5, 0.0, 0.5, 1.0):
        for sv in (-1.0, 0.0, 1.0):
            for h in (HAND_BODY_ABOVE_TCP, HAND_BODY_ABOVE_TCP + 0.04, HAND_BODY_ABOVE_TCP + 0.08):
                pts.append((su * HAND_HALF_W, sv * HAND_HALF_T, h))
    for su in (-1.0, 1.0):
        for sv in (-1.0, 1.0):
            for h in (-FINGER_BELOW_TCP, 0.0, HAND_BODY_ABOVE_TCP):
                pts.append((su * (GRIPPER_OPEN_W / 2 + 0.008), sv * 0.011, h))
    for su in (-1.0, 0.0, 1.0):          # the wrist, last three points (hand_fits3d relies on that)
        pts.append((su * 0.045, 0.0, 0.17))
    return np.array(pts)


HAND_PTS = _hand_points()


def axis_rot(axis, a):
    k = np.asarray(axis, float)
    k = k / np.linalg.norm(k)
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(a) * K + (1.0 - math.cos(a)) * K @ K


def tilt_rot(yaw, tilt):
    """World rotation on top of the straight-down hand at `yaw`: tilt[0] > 0 leans the fingertips
    toward +finger axis, tilt[1] > 0 toward -thickness axis (the wrist leans the other way)."""
    u = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    v = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    return axis_rot(v, tilt[0]) @ axis_rot(u, tilt[1])


def is_tilted(tilt):
    return tilt is not None and (abs(tilt[0]) > 1e-3 or abs(tilt[1]) > 1e-3)


def hand_fits3d(containers, p, yaw, tilt=(0.0, 0.0), inside=None):
    """The open hand with its tool centre point at p (x, y, z), `yaw` and `tilt`: every hand point
    below a container's rim must be clear of that container's walls (inside its interior when it
    is the container `inside`, outside its footprint otherwise). Points above the rims are free,
    which is what lets a hand reach an object high in a pile, or near a wall when tilted."""
    u = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    v = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    B = np.stack([u, v, np.array([0.0, 0.0, 1.0])], 1)
    R = tilt_rot(yaw, tilt) if is_tilted(tilt) else np.eye(3)
    W = np.asarray(p, float) + (R @ B @ HAND_PTS.T).T
    below = W[:, 2] < CONTAINER_RIM_Z + RIM_CLEAR
    for name, b in containers.items():
        ox, oy = outer_half(name)
        ix, iy = inner_half(name)
        lx, ly = container_local(b, W[:, 0], W[:, 1])
        in_outer = (np.abs(lx) <= ox + 0.004) & (np.abs(ly) <= oy + 0.004)
        if name == inside:
            # the hand comes in from above: below the rim, all of it is inside the interior
            in_inner = (np.abs(lx) <= ix - 0.004) & (np.abs(ly) <= iy - 0.004)
            if np.any(below & ~in_inner):
                return False
            # and it may only lean toward the container's middle: the wrist (and the forearm
            # above it, which the point model does not carry) stays over the interior. A hand
            # leaning out over a wall clears the point model above the rim but lands its wrist
            # on the rim, where it hangs 6-8 cm short of the grasp
            wl = lx[-3:]
            wy = ly[-3:]
            if np.any(np.abs(wl) > ix) or np.any(np.abs(wy) > iy):
                return False
        elif np.any(below & in_outer):
            return False
    return bool(np.all(W[:, 2] > -0.004))


def fmt_tilt(tilt):
    return "%+.2f/%+.2f" % (tilt[0], tilt[1]) if is_tilted(tilt) else "0"


def fmt_plans(g):
    return [(round(x[1], 2), round(x[2], 3), fmt_tilt(x[3] if len(x) > 3 else None)) for x in g]


def approach_dir(yaw, tilt):
    """Unit vector from the wrist toward the fingertips."""
    R = tilt_rot(yaw, tilt) if is_tilted(tilt) else np.eye(3)
    return -(R @ np.array([0.0, 0.0, 1.0]))


def fits_with_approach(containers, p, yaw, tilt, inside):
    """The final grasp pose fits, and so does the pose 4 cm back along the approach."""
    if not hand_fits3d(containers, p, yaw, tilt, inside):
        return False
    return hand_fits3d(containers, np.asarray(p, float) - 0.04 * approach_dir(yaw, tilt), yaw, tilt, inside)


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


def fit_container(p, cx, cy, yaw0=0.0, name=None):
    """Refine one container's pose from rim-band points near its nominal centre: coarse-to-fine grid
    search over (x, y, yaw) maximising the number of points on the outline of the known outer
    footprint. Robust to partial rings (a container seen from one side only)."""
    ox, oy = outer_half(name)
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
        # rim-height points well inside the outline belong to a wall the outline has swallowed
        # (a neighbour's, 3 cm away), or to a tall object: a mild penalty breaks the tie
        inside = (np.abs(lx) < ox - 0.035) & (np.abs(ly) < oy - 0.035)
        return float((on_long | on_short).sum()) - 0.5 * float(inside.sum())

    # the search stays within the poses the task card allows (jitter x +-0.02, y +-0.008, yaw +-5
    # degrees, plus a little slack): a wider search snapped the outline onto the next container's
    # wall and left that wall standing inside this container as a phantom object
    best = (-1e9, cx, cy, yaw0)
    for yaw in yaw0 + np.radians(np.arange(-7, 7.1, 1.0)):
        for dx in np.arange(-0.03, 0.031, 0.01):
            for dy in np.arange(-0.012, 0.0121, 0.004):
                sc = score(cx + dx, cy + dy, yaw)
                if sc > best[0]:
                    best = (sc, cx + dx, cy + dy, yaw)
    _, bx, by, byaw = best
    for yaw in byaw + np.radians(np.arange(-1.0, 1.01, 0.25)):
        for dx in np.arange(-0.006, 0.0061, 0.002):
            for dy in np.arange(-0.004, 0.0041, 0.002):
                sc = score(bx + dx, by + dy, yaw)
                if sc > best[0]:
                    best = (sc, bx + dx, by + dy, yaw)
    return best


def detect_containers(pts, col):
    """Container poses from the rim band of the fused cloud (the upper walls, above most objects):
    each container is fitted around its nominal pose with the known footprint. A container with too few
    band points keeps its nominal pose."""
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    band = (z > CONTAINER_RIM_Z - 0.05) & (z < CONTAINER_RIM_Z + 0.02) & (x > 0.10) & (x < 0.80) & (np.abs(y) < 0.65)
    containers = nominal_containers()
    fits = []
    if band.sum() < 40:
        return containers, fits
    p = pts[band][:, :2]
    for name, (bx, by, _) in list(containers.items()):
        r = fit_container(p, bx, by, name=name)
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
    oz = CONTAINER_OUTER[2]
    for name, b in containers.items():
        ox, oy = outer_half(name)
        ix, iy = inner_half(name)
        lx, ly = container_local(b, x, y)
        in_outer = (np.abs(lx) <= ox + 0.014) & (np.abs(ly) <= oy + 0.014)
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
        keep = pts[:, 0] < NEAR_STRIP_X
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
        c = cluster_features(pts[mm], col[mm], containers)
        if is_wall_sliver(c, containers):
            continue
        out.append(c)
    return out, containers


def is_wall_sliver(c, containers):
    """A thin vertical plate as tall as the container walls, standing on a container's outline: a
    wall left over by a container pose that is off by a centimetre or two, not an object."""
    if c["top_z"] < CONTAINER_RIM_Z - 0.03:
        return False
    x, y = c["centroid"][0], c["centroid"][1]
    for name, b in containers.items():
        ox, oy = outer_half(name)
        lx, ly = container_local(b, x, y)
        on_line = abs(lx) < ox + 0.04 and abs(ly) < oy + 0.04 and (abs(abs(lx) - ox) < 0.04 or abs(abs(ly) - oy) < 0.04)
        if on_line and c["len_minor"] <= 0.02:
            return True
        # a somewhat wider blob whose top is the rim itself, centred within 3 cm of the wall line
        # and still thin: a wall segment merged with its rim, not an object (with 12 cm walls many
        # objects reach rim height, so only a thin blob counts; an object standing against the
        # inner wall has its centre at least 5 cm from the outer wall line)
        if c["top_z"] >= CONTAINER_RIM_Z - 0.015 and c["len_minor"] <= 0.035 and abs(lx) < ox + 0.03 and abs(ly) < oy + 0.03 and (abs(abs(lx) - ox) < 0.03 or abs(abs(ly) - oy) < 0.03):
            return True
    return False


def perceive_eih(rgb, depth, R_cam, t_cam, expected_xy, floor, top_hint, containers, radius=0.13, voxel=0.006):
    """Dense cluster of the object under the hand camera."""
    pts, col = backproject_rt(depth, rgb, CAMS["agentview"]["f"], CAMS["agentview"]["c"], R_cam, t_cam)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    # height above whatever the point stands on: the container floor inside a container, else the desk profile
    inside = np.zeros(len(pts), bool)
    for name, b in containers.items():
        ix, iy = inner_half(name)
        lx, ly = container_local(b, x, y)
        inside |= (np.abs(lx) <= ix) & (np.abs(ly) <= iy)
    base = np.where(inside, CONTAINER_FLOOR_Z, desk_top_z(x))
    keep = (z - base > 0.008) & (z < top_hint + 0.06) & (np.hypot(x - expected_xy[0], y - expected_xy[1]) < radius)
    oz = CONTAINER_OUTER[2]
    for name, b in containers.items():
        ox, oy = outer_half(name)
        ix, iy = inner_half(name)
        lx, ly = container_local(b, x, y)
        in_outer = (np.abs(lx) <= ox + 0.014) & (np.abs(ly) <= oy + 0.014)
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
    p, pc = pts[best], col[best]
    blocker, note = None, None
    sp = stack_split(p, expected_xy)
    if sp is not None:
        own, tidx, note = sp
        if tidx is not None and len(tidx) >= 30:
            blocker = cluster_features(p[tidx], pc[tidx], containers)
            blocker["floor"] = floor if blocker["container"] else float(desk_top_z(blocker["centroid"][0]))
            blocker["dims"] = sorted([blocker["len_major"], blocker["len_minor"], blocker["top_z"] - blocker["floor"]])
        if len(own) >= 30:
            p, pc = p[own], pc[own]
    c = cluster_features(p, pc, containers)
    c["blocker"], c["stack_note"] = blocker, note
    # two objects that touch merge into one component; the grasp point comes from the top
    # surface around the expected position only, so a neighbour does not drag it aside
    near = p[np.hypot(p[:, 0] - expected_xy[0], p[:, 1] - expected_xy[1]) < 0.06]
    if c["len_major"] > 0.12 and len(near) >= 30:
        top_near = float(np.percentile(near[:, 2], 97))
        slab = near[near[:, 2] > top_near - 0.03]
        if len(slab) >= 10:
            c["grasp_xy"] = [float(slab[:, 0].mean()), float(slab[:, 1].mean())]
    c["floor"] = floor if c["container"] else float(desk_top_z(c["centroid"][0]))
    c["dims"] = sorted([c["len_major"], c["len_minor"], c["top_z"] - c["floor"]])
    return c


# ----------------------------------------------------------------------------------------
# Stacking: what rests on what, inferred from the hand camera's cloud. Objects that touch
# merge into one connected component; the component is cut into plateaus (cells of a 1 cm
# height map that connect where the top height changes by less than STACK_STEP). A plateau
# whose boundary mostly drops onto another plateau sits on it; a tilted plateau (a leaning
# object) whose high end borders a lower plateau leans on it. Whatever rests on the object the
# controller wants comes off first.
# ----------------------------------------------------------------------------------------
STACK_CELL = 0.01
STACK_STEP = 0.02
STACK_MIN_CELLS = 15      # smaller plateaus (a cap, a handle, noise) never count as a resting object
STACK_DROP = 0.015        # a neighbouring cell this much lower is "below" the boundary
STACK_ON_FRAC = 0.4       # share of the boundary that must drop onto one plateau to rest on it
STACK_RAMP = 0.04         # height spread that makes a plateau a tilted object
STACK_ENCLOSED = 0.85     # a lower plateau whose outside is this much one other plateau is that object's hollow (a mug's inside), not something under it
STACK_ON_HULL = 0.4       # share of a resting plateau's cells that must lie over the footprint (convex hull) of the plateau it rests on: a plateau merely beside a lower one (a lying mug's ridge next to its own flank, a tall can next to a short one) is not on it
STACK_LEAN_HULL = 0.15    # the same for a leaning plateau: its high end must reach over the supporting footprint


def plateaus(p, cell=STACK_CELL, step=STACK_STEP, min_cells=STACK_MIN_CELLS):
    """-> (cells {(i,j): [point idx]}, hmax {(i,j): top z}, label {(i,j): plateau}, members [[cells]])
    Plateaus smaller than `min_cells` are merged into the neighbour they touch most."""
    cells = {}
    ij = np.floor(p[:, :2] / cell).astype(int)
    for k in range(len(p)):
        cells.setdefault((int(ij[k, 0]), int(ij[k, 1])), []).append(k)
    hmax = {c: float(p[idx, 2].max()) for c, idx in cells.items()}
    label, members = {}, []
    for c0 in sorted(cells, key=lambda c: -hmax[c]):
        if c0 in label:
            continue
        lab = len(members)
        label[c0] = lab
        stack, mem = [c0], [c0]
        while stack:
            c = stack.pop()
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    nb = (c[0] + di, c[1] + dj)
                    if nb in cells and nb not in label and abs(hmax[nb] - hmax[c]) < step:
                        label[nb] = lab
                        stack.append(nb)
                        mem.append(nb)
        members.append(mem)
    for lab in sorted(range(len(members)), key=lambda l: len(members[l])):
        mem = members[lab]
        if not mem or len(mem) >= min_cells:
            continue
        touch = {}
        for c in mem:
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    nb = (c[0] + di, c[1] + dj)
                    ol = label.get(nb)
                    if ol is not None and ol != lab:
                        touch[ol] = touch.get(ol, 0) + 1
        if touch:
            to = max(touch, key=touch.get)
            for c in mem:
                label[c] = to
            members[to].extend(mem)
            members[lab] = []
    return cells, hmax, label, members


def _hull(points):
    """Convex hull of 2-D points (monotone chain) -> vertices counter-clockwise."""
    pts = sorted(set(tuple(p) for p in points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower = []
    for q in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(q)
    upper = []
    for q in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(q)
    return lower[:-1] + upper[:-1]


def inside_hull_frac(cells_a, cells_b, margin=1.0):
    """Share of the cells `cells_a` that lie within the convex hull of `cells_b` grown by `margin`
    cells (all in cell coordinates); a degenerate hull (a line) counts as its bounding box."""
    if not cells_a or not cells_b:
        return 0.0
    hull = _hull(cells_b)
    if len(hull) < 3:
        xs, ys = [c[0] for c in cells_b], [c[1] for c in cells_b]
        x0, x1, y0, y1 = min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin
        return sum(1 for c in cells_a if x0 <= c[0] <= x1 and y0 <= c[1] <= y1) / float(len(cells_a))
    n = 0
    for c in cells_a:
        ok = True
        for i in range(len(hull)):
            a, b = hull[i], hull[(i + 1) % len(hull)]
            ex, ey = b[0] - a[0], b[1] - a[1]
            if (ex * (c[1] - a[1]) - ey * (c[0] - a[0])) / max(math.hypot(ex, ey), 1e-9) < -margin:
                ok = False
                break
        n += ok
    return n / float(len(cells_a))


def stack_relations(p, cells, hmax, label, members):
    """-> {plateau: {"on": {other: share}, "leans": {other: cells}, "top", "bottom", "n_cells"}}
    A plateau enclosed by the one above it (its outside is almost only that plateau) is the hollow
    of one object (a mug's or bowl's inside seen from above), never something under it; and a
    plateau rests or leans on another only when it reaches over that plateau's footprint, so the
    two sides of one lying object (its ridge and its lower flank) or two objects side by side are
    not read as a stack."""
    stats = {}
    for lab, mem in enumerate(members):
        if not mem:
            continue
        zs = p[[k for c in mem for k in cells[c]], 2]
        top = float(np.percentile(zs, 97))
        border, over, high_over, outside = 0, {}, {}, {}
        for c in mem:
            lower, is_border = set(), False
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    nb = (c[0] + di, c[1] + dj)
                    if nb == c:
                        continue
                    ol = label.get(nb)
                    if ol == lab:
                        continue
                    is_border = True
                    outside[ol] = outside.get(ol, 0) + 1      # ol None: empty cell (desk, unseen)
                    if ol is not None and hmax[nb] < hmax[c] - STACK_DROP:
                        lower.add(ol)
            if is_border:
                border += 1
            for ol in lower:
                over[ol] = over.get(ol, 0) + 1
                if hmax[c] > top - 0.03:
                    high_over[ol] = high_over.get(ol, 0) + 1
        stats[lab] = (top, float(np.percentile(zs, 3)), border, over, high_over, outside)

    def enclosed_by(low, high):
        """Is the lower plateau the hollow of the higher one? Either its outside is almost only
        that plateau, or it lies within that plateau's footprint (a mug's inner wall seen from
        above borders the rim on one side and, where the floor is too low to count, nothing on
        the other)."""
        outside = stats[low][5]
        total = sum(outside.values())
        if total > 0 and outside.get(high, 0) >= STACK_ENCLOSED * total:
            return True
        return inside_hull_frac(members[low], members[high], margin=0.5) >= STACK_ENCLOSED

    out = {}
    for lab, (top, bottom, border, over, high_over, outside) in stats.items():
        rel = {"top": top, "bottom": bottom, "n_cells": len(members[lab]), "on": {}, "leans": {}}
        if len(members[lab]) >= STACK_MIN_CELLS and border:
            for ol, cnt in over.items():
                if cnt >= STACK_ON_FRAC * border and not enclosed_by(ol, lab) and inside_hull_frac(members[lab], members[ol]) >= STACK_ON_HULL:
                    rel["on"][ol] = cnt / float(border)
            if top - bottom >= STACK_RAMP:
                for ol, cnt in high_over.items():
                    if cnt >= 3 and ol not in rel["on"] and not enclosed_by(ol, lab) and inside_hull_frac(members[lab], members[ol]) >= STACK_LEAN_HULL:
                        rel["leans"][ol] = cnt
        out[lab] = rel
    return out


def stack_split(p, expected_xy, cell=STACK_CELL):
    """The object at `expected_xy` inside the merged cloud `p` and whatever rests on it.
    -> None (nothing rests on anything here: keep the cloud whole), or
       (own point idx, topper point idx or None, note) where the topper is the largest plateau
       resting or leaning on the object's plateau; own = the object's plateau alone when it is
       itself a topper or carries one."""
    if len(p) < 60:
        return None
    cells, hmax, label, members = plateaus(p, cell)
    live = [lab for lab, mem in enumerate(members) if mem]
    if len(live) < 2:
        return None
    rels = stack_relations(p, cells, hmax, label, members)
    # the plateau under the expected position: the nearest cell, the higher plateau on a tie
    ci, cj = int(math.floor(expected_xy[0] / cell)), int(math.floor(expected_xy[1] / cell))
    target = min(cells, key=lambda c: (max(abs(c[0] - ci), abs(c[1] - cj)), -hmax[c]))
    mine = label[target]
    toppers = [lab for lab in live if lab != mine and (mine in rels[lab]["on"] or mine in rels[lab]["leans"])]
    carried = [lab for lab in live if lab != mine and (lab in rels[mine]["on"] or lab in rels[mine]["leans"])]
    if not toppers and not carried:
        return None
    own = [k for c in members[mine] for k in cells[c]]
    if toppers:
        tl = max(toppers, key=lambda lab: rels[lab]["n_cells"])
        tidx = [k for c in members[tl] for k in cells[c]]
        how = "on" if mine in rels[tl]["on"] else "leaning on"
        note = "%d-cell plateau (top %.3f) %s the %d-cell one (top %.3f)" % (
            rels[tl]["n_cells"], rels[tl]["top"], how, rels[mine]["n_cells"], rels[mine]["top"])
        return own, tidx, note
    note = "it is the topper: %d cells (top %.3f) resting on %s" % (
        rels[mine]["n_cells"], rels[mine]["top"], ["%d cells" % rels[l]["n_cells"] for l in carried])
    return own, None, note


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
        self.drop_slot = {b: 0 for b in CONTAINER_NOMINAL}
        self.skip_list = []
        self.last_perceive_t = -999
        self.last_clusters = []
        self.not_empty_t = {}
        self.carry_hang = 0.0
        self.last_descent_short = False
        self.last_grasp_unreachable = False
        self.stuck_strikes = 0
        self._dragged_free = []
        self.stuck_count, self.recovering, self.stuck_recovered = 0, False, False
        # what the controller knows about the class containers' contents: the models it put in
        # itself (bottom -> top), and whether an initial pile of unseen objects may still lie
        # under them (the instruction: A objects start in gray, B objects in light-blue)
        self.inventory = {b: [] for b in CLASS_OF_CONTAINER}
        self.pile_unknown = {"gray": True, "lightblue": True, "blue": False}
        self.seen_in = {b: set() for b in CLASS_OF_CONTAINER}     # models seen on top of a pile by a patrol
        self.last_patrol_t = -10 ** 9

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
        return self.mark_before_deadline([float(x) for x in self.policy(obs)], obs)

    def mark_before_deadline(self, a, obs):
        """The mark goes out before the deadline whatever the behaviour is in the middle of (a
        tidy grasp the arm got stuck in once carried a delivered request past its deadline). With
        MARK_LAST steps left: delivered when the target went into red, else absent, since an
        expiry scores like a wrong mark and absent may be right. The behaviour sees `marked` and
        drops the request on its next step."""
        rs, req = self.request_state, obs.get("request")
        if rs is None or req is None or rs.get("marked") or a[7] != 0 or req["text"] != self.request_text:
            return a
        if req["deadline_t"] - int(obs["t"]) > MARK_LAST:
            return a
        a[7] = 1.0 if (rs.get("in_red") or rs.get("placed_red")) else 2.0
        rs["marked"] = True
        log("t=%d deadline in %d steps; marking %s now" % (int(obs["t"]), req["deadline_t"] - int(obs["t"]), "delivered" if a[7] == 1.0 else "absent"))
        return a

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

    def orient_error(self, yaw, obs=None, tilt=None):
        R_cur = quat_to_mat(np.array((obs or self.obs)["ee_pose"][3:7], float))
        R_des = rot_z(yaw) @ self.R_home
        if is_tilted(tilt):
            R_des = tilt_rot(yaw, tilt) @ R_des
        return rotvec(R_des @ R_cur.T)

    def width(self):
        return float(self.obs["gripper_width"])

    def t(self):
        return int(self.obs["t"])

    def request(self):
        return self.obs.get("request")

    # -- motion primitives --------------------------------------------------------------------
    def servo(self, pos, yaw, grip, mark=0, step_max=POS_STEP_MAX, tilt=None):
        pos = np.asarray(pos, float)

        def policy(o):
            err = pos - np.array(o["ee_pose"][:3], float)
            d = np.clip(err, -step_max, step_max)
            r = np.clip(self.orient_error(yaw, o, tilt), -ROT_STEP_MAX, ROT_STEP_MAX)
            return [d[0], d[1], d[2], r[0], r[1], r[2], grip, mark]
        return policy

    def goto(self, pos, yaw=0.0, grip=0.0, tol=0.01, yaw_tol=0.04, max_steps=60, mark=0, step_max=POS_STEP_MAX, tilt=None):
        pos = np.asarray(pos, float)
        for i in range(max_steps):
            err = pos - self.ee()
            if np.linalg.norm(err) < tol and np.linalg.norm(self.orient_error(yaw, tilt=tilt)) < yaw_tol:
                self.stuck_count = 0
                return True
            yield self.servo(pos, yaw, grip, mark, step_max, tilt)
            mark = 0
        # An arm that keeps ending far from where it was sent has folded into a configuration the
        # Cartesian servo cannot leave (a joint at its limit): a few straight lifts to the
        # observe height unfold it. Without this, every later move of the episode failed too.
        if np.linalg.norm(pos - self.ee()) > STUCK_ERR and not self.recovering:
            self.stuck_count += 1
            if self.stuck_count >= STUCK_AFTER:
                log("t=%d arm stuck (%d moves ended far off); recovering through the observe pose" % (self.t(), self.stuck_count))
                self.recovering = True
                try:
                    cur = self.ee()
                    yield from self.goto((cur[0], cur[1], 0.45), 0.0, grip=grip, tol=0.03, yaw_tol=0.3, max_steps=40)
                    yield from self.goto(OBSERVE_POS, 0.0, grip=grip, tol=0.02, yaw_tol=0.1, max_steps=80)
                finally:
                    self.recovering = False
                    self.stuck_count = 0
                    self.stuck_recovered = True       # the current grasp gives up (try_grasps, pick)
        return False

    def hold(self, steps, grip, mark=0, yaw=None, tilt=None):
        pos = self.ee()
        yaw = self.yaw() if yaw is None else yaw
        for i in range(steps):
            yield self.servo(pos, yaw, grip, mark, tilt=tilt)
            mark = 0

    def gripper_closed_on_object(self):
        # fully shut reads 0.0000-0.0005; a power drill held by its trigger reads 4 mm; fully open
        # reads 0.0800, and a large clamp pinched across its jaws reads 0.077-0.078 (four lifts of
        # one clamp were dropped again at the 0.077 bound)
        return 0.002 < self.width() < 0.0792

    def travel_z(self):
        """Carry height: the held object's bottom clears the container rims."""
        return max(TRAVEL_Z, CONTAINER_RIM_Z + 0.04 + self.carry_hang)

    def refine_xy(self, xy, anchor_xy, floor, top_z, yaw=0.0):
        """Hovering 10 cm above the object with the hand camera: re-measure the top centroid
        around the planned anchor and shift the grasp point by how far it moved; also measure
        the top within the strip the fingers sweep (the finger span along the closing axis, a
        finger's width across). Two touching objects merge into one component whose top is the
        taller one's, so the component's top drags the grasp point toward the neighbour and sets
        the fingers above the object; the strip, not the point under the tool centre, is what
        the fingers meet, so a hollow object's rim (the pitcher's, under a grasp planned over its
        opening) or an object the hand sits beside along the finger axis keeps its real top.
        -> (xy, local top z or None)"""
        try:
            R_cam, t_cam = self.cam_pose()
            rgb = decode_png(self.obs["rgb"]["eye_in_hand"], "RGB")
            dep = decode_png(self.obs["depth"]["eye_in_hand"])
            d = perceive_eih(rgb, dep, R_cam, t_cam, anchor_xy, floor, top_z, self.containers, radius=REFINE_RADIUS)
        except Exception as exc:  # noqa: BLE001
            log("refine failed: %r" % exc)
            return xy, None
        if d is None:
            return xy, None
        p = d["pts"]
        near = p[np.hypot(p[:, 0] - anchor_xy[0], p[:, 1] - anchor_xy[1]) < REFINE_NEAR]
        if len(near) < 15:
            return xy, None
        slab = near[near[:, 2] > float(np.percentile(near[:, 2], 97)) - 0.02]
        if len(slab) < 15:
            return xy, None
        cx, cy = slab[:, :2].mean(0)
        dx, dy = float(cx - anchor_xy[0]), float(cy - anchor_xy[1])
        if math.hypot(dx, dy) > REFINE_MAX_SHIFT:
            log("t=%d refine: shift %.3f too large, ignored" % (self.t(), math.hypot(dx, dy)))
            dx = dy = 0.0
        g = (xy[0] + dx, xy[1] + dy)
        u, v = np.array([-math.sin(yaw), math.cos(yaw)]), np.array([math.cos(yaw), math.sin(yaw)])
        rel = p[:, :2] - g
        under = p[(np.abs(rel @ u) < GRASP_MAX_WIDTH / 2) & (np.abs(rel @ v) < TOP_UNDER_RADIUS)]
        # the highest few points, not a percentile: the strip over a hollow pitcher or a thin tall
        # box is mostly its inside or its sides, and the 97th percentile sat 5 cm under the rim
        top_local = float(np.percentile(under[:, 2], 99.5)) if len(under) >= 10 else None
        log("t=%d refine: shift (%+.3f,%+.3f) from %d top points, top under the grasp %s (planned %.3f)%s" % (
            self.t(), dx, dy, len(slab), "%.3f" % top_local if top_local is not None else "-", top_z,
            "; stack: " + d["stack_note"] if d.get("stack_note") else ""))
        return g, top_local

    def pick(self, xy, yaw, top_z, floor, below_top=GRASP_BELOW_TOP, anchor_xy=None, tilt=None, keep_top=False):
        """Top-down pinch grasp at `yaw`, fingers `below_top` under the object's top (a rim grasp
        uses a shallow depth). The grasp point is refined from the hand camera 10 cm above the
        object before the final descent. Returns True when something is held after lifting."""
        x, y = xy
        yaw, tilt = yaw_branch((x, y), wrap_half(yaw), tilt)
        z_pre = max(top_z + 0.10, floor + 0.12)
        self.carry_hang = max(0.0, (top_z - floor) - below_top)     # how far the object hangs below the fingertips
        z_travel = max(self.travel_z(), z_pre)
        yield from self.goto((x, y, z_travel), yaw, grip=0.0, tol=0.015)
        yield from self.goto((x, y, z_pre), yaw, grip=0.0, tol=0.008, max_steps=30)
        if self.stuck_recovered:
            return False            # the arm folded up on the way and is back at the observe pose
        yield from self.hold(1, grip=0.0)
        (x, y), top_local = self.refine_xy((x, y), anchor_xy or (x, y), floor, top_z, yaw=yaw)
        if top_local is not None and top_local < top_z - TOP_DROP_MIN and not keep_top:
            top_z = top_local       # the planned top was a taller neighbour's
            self.carry_hang = max(0.0, (top_z - floor) - below_top)
        z_g = max(top_z - below_top, floor + GRASP_MIN_ABOVE_FLOOR)
        self.last_grasp = (x, y, z_g, yaw)
        if is_tilted(tilt):
            # lean the hand away from the near wall above the rim, then go in along the fingers.
            # The tool centre point sits FINGER_BELOW_TCP above the fingertips, so a tilted hand
            # puts its fingertips 0.8-1.6 cm to the side of the TCP: the TCP is sent that much the
            # other way, so that the fingertips, not the TCP, land on the object (uncompensated,
            # tilted grasps lifted 2 of 16 in the episode logs: one finger came down on the object)
            a = approach_dir(yaw, tilt)
            log("t=%d tilted grasp: plan (%.3f,%.3f) anchor (%.3f,%.3f) yaw %.2f tilt %s approach (%.2f,%.2f,%.2f) -> tcp (%.3f,%.3f) z %.3f" % (
                self.t(), x, y, (anchor_xy or (x, y))[0], (anchor_xy or (x, y))[1], yaw, fmt_tilt(tilt), a[0], a[1], a[2],
                x - FINGER_BELOW_TCP * a[0], y - FINGER_BELOW_TCP * a[1], z_g))
            x, y = x - FINGER_BELOW_TCP * a[0], y - FINGER_BELOW_TCP * a[1]
            self.last_grasp = (x, y, z_g, yaw)
            back = max(0.06, (CONTAINER_RIM_Z + 0.06 - z_g) / max(-a[2], 0.3))
            p_pre = np.array([x, y, z_g]) - a * back
            yield from self.goto(p_pre, yaw, grip=0.0, tol=0.01, yaw_tol=0.06, max_steps=40, tilt=tilt)
            yield from self.goto((x, y, z_g), yaw, grip=0.0, tol=0.006, yaw_tol=0.06, max_steps=30, step_max=0.02, tilt=tilt)
            self.last_grasp_actual = [float(v) for v in self.ee()]
            self.last_descent_short = float(np.linalg.norm(self.ee() - np.array([x, y, z_g]))) > DESCENT_SHORT
            if self.last_descent_short:
                log("t=%d descent stopped %.3f m short of the grasp point; not closing" % (self.t(), float(np.linalg.norm(self.ee() - np.array([x, y, z_g])))))
                yield from self.goto(p_pre, yaw, grip=0.0, tol=0.015, yaw_tol=0.08, max_steps=30, step_max=0.03, tilt=tilt)
                yield from self.goto((p_pre[0], p_pre[1], z_travel), yaw, grip=0.0, tol=0.015, max_steps=30, step_max=0.03)
                return False
            yield from self.close_fingers(yaw=yaw, tilt=tilt)
            yield from self.goto(p_pre, yaw, grip=1.0, tol=0.015, yaw_tol=0.08, max_steps=30, step_max=0.03, tilt=tilt)
            yield from self.goto((p_pre[0], p_pre[1], z_travel), yaw, grip=1.0, tol=0.015, max_steps=30, step_max=0.03)
        else:
            yield from self.goto((x, y, z_g), yaw, grip=0.0, tol=0.006, max_steps=25, step_max=0.02)
            self.last_grasp_actual = [float(v) for v in self.ee()]
            self.last_descent_short = float(np.linalg.norm(self.ee() - np.array([x, y, z_g]))) > DESCENT_SHORT
            if self.last_descent_short:
                log("t=%d descent stopped %.3f m short of the grasp point; not closing" % (self.t(), float(np.linalg.norm(self.ee() - np.array([x, y, z_g])))))
                yield from self.goto((x, y, z_travel), yaw, grip=0.0, tol=0.015, max_steps=30, step_max=0.03)
                return False
            yield from self.close_fingers()
            yield from self.goto((x, y, z_travel), yaw, grip=1.0, tol=0.015, max_steps=30, step_max=0.03)
        yield from self.hold(2, grip=1.0)
        # judged after the lift only: a width read right after the close command, while the
        # fingers were still travelling to a wide object, called a pitcher in the hand "nothing",
        # and the object went back down from the carry height
        return self.gripper_closed_on_object()

    def close_fingers(self, yaw=None, tilt=None):
        """Command the fingers shut and wait until they stop moving (at least three, at most
        GRIP_SETTLE_MAX steps): the drive takes several steps to meet a wide object."""
        last = self.width()
        for i in range(GRIP_SETTLE_MAX):
            yield from self.hold(1, grip=1.0, yaw=yaw, tilt=tilt)
            w = self.width()
            if i >= 2 and abs(w - last) < 0.0015:
                break
            last = w

    def maybe_drag(self, d, b):
        """Drag an unreachable object toward the middle at most DRAG_TRIES_MAX times per spot and
        request (the bookkeeping lives on the request state; outside a request, once)."""
        if not DRAG_ENABLED:
            return False
        rs = self.request_state or {}
        done = rs.setdefault("dragged", []) if self.request_state else self._dragged_free
        n = sum(1 for (fb, fx, fy) in done if fb == b and math.hypot(fx - d["grasp_xy"][0], fy - d["grasp_xy"][1]) < 0.05)
        if n >= (DRAG_TRIES_MAX if self.request_state else 1):
            return False
        ok = yield from self.drag_to_middle(d, b)
        if ok:
            done.append((b, d["grasp_xy"][0], d["grasp_xy"][1]))
        return ok

    def drag_to_middle(self, d, b):
        """An object inside container b that no top-down pinch can reach where it lies (the hand
        body would be on a wall): press one open fingertip onto its top and slide it toward the
        container's middle, into the band where a straight pinch fits, then let the caller look
        again and dig it. The hand stays straight down with its body over the interior; the
        fingertip is 4.4 cm out from the tool centre along the finger axis, which is what reaches
        past where the body may go. -> True when a drag was made."""
        cb = self.containers[b]
        x, y = d["grasp_xy"]
        lx, ly = container_local(cb, x, y)
        cands = [(max(-DRAG_BAND_X, min(DRAG_BAND_X, lx)), ly), (lx, max(-DRAG_BAND_Y, min(DRAG_BAND_Y, ly)))]
        tl = min(cands, key=lambda c: math.hypot(c[0] - lx, c[1] - ly))
        dist = math.hypot(tl[0] - lx, tl[1] - ly)
        if dist < DRAG_MIN:
            return False
        tx, ty = container_point(cb, tl[0], tl[1])
        dvec = np.array([tx - x, ty - y]); dvec /= max(float(np.linalg.norm(dvec)), 1e-9)
        dist += DRAG_OVERSHOOT
        top = d["top_z"]
        z_press = top + FINGER_BELOW_TCP - DRAG_PRESS      # for the fit test only; the press height comes from contact
        z_hover = top + FINGER_BELOW_TCP + 0.03
        # fingertip on the object: the tool centre 4.4 cm away along the finger axis, on the side of the
        # middle (u along the drag) or beside it (u across the drag); whichever pose the body fits
        plan = None
        for ang in (0.0, math.pi / 2, -math.pi / 2):
            ux, uy = math.cos(ang) * dvec[0] - math.sin(ang) * dvec[1], math.sin(ang) * dvec[0] + math.cos(ang) * dvec[1]
            yaw = wrap_half(math.atan2(-ux, uy))          # finger axis u = (-sin yaw, cos yaw) = (ux, uy)
            for s_ in (1.0, -1.0):
                tcp = (x + s_ * (GRIPPER_OPEN_W / 2 + 0.004) * ux, y + s_ * (GRIPPER_OPEN_W / 2 + 0.004) * uy)
                ok = all(hand_fits3d(self.containers, (tcp[0] + k * dvec[0], tcp[1] + k * dvec[1], z_press), yaw, (0.0, 0.0), inside=b)
                         for k in (0.0, dist / 2, dist))
                if ok:
                    plan = (tcp, yaw)
                    break
            if plan:
                break
        if plan is None:
            log("t=%d drag %s: no press pose fits for the %s at local (%.2f,%.2f)" % (self.t(), b, d["cls_info"]["model"] if d.get("cls_info") else "object", lx, ly))
            return False
        (px, py), yaw = plan
        yaw, _ = yaw_branch((px, py), yaw, None)
        log("t=%d drag %s: %s at local (%.2f,%.2f) top=%.3f -> %.1f cm toward the middle, press from (%.2f,%.2f) yaw %.2f" % (
            self.t(), b, d["cls_info"]["model"] if d.get("cls_info") else "object", lx, ly, top, 100 * dist, px, py, yaw))
        yield from self.goto((px, py, self.travel_z()), yaw, grip=0.0, tol=0.015)
        yield from self.goto((px, py, z_hover), yaw, grip=0.0, tol=0.006, yaw_tol=0.04, max_steps=40)
        # descend until the fingertip meets the object (the tool centre stops sinking): the fingertip's
        # true height under the tool centre is not trusted here, contact is
        z_floor = max(top - 0.05, d["floor"] + 0.01)
        z_last = float(self.ee()[2]); z_contact = None
        for _ in range(14):
            yield from self.goto((px, py, z_floor), yaw, grip=0.0, tol=0.003, max_steps=1, step_max=0.008)
            z_now = float(self.ee()[2])
            if z_last - z_now < 0.002:
                z_contact = z_now
                break
            z_last = z_now
            if z_now <= z_floor + 0.004:
                break
        if z_contact is None or z_contact < top - 0.03:
            log("t=%d drag: no contact above the object (tool centre sank to %.3f, top %.3f); giving up" % (self.t(), float(self.ee()[2]), top))
            yield from self.goto((px, py, self.travel_z()), yaw, grip=0.0, tol=0.02, max_steps=30)
            return False
        z_press = z_contact - 0.004
        log("t=%d drag: contact at tool centre z %.3f (top %.3f: fingertip %.3f under the tool centre); pressing at %.3f" % (
            self.t(), z_contact, top, z_contact - top, z_press))
        n = int(math.ceil(dist / DRAG_STEP))
        for k in range(1, n + 1):
            f = min(1.0, k * DRAG_STEP / dist)
            yield from self.goto((px + f * dist * dvec[0], py + f * dist * dvec[1], z_press), yaw, grip=0.0, tol=0.005, max_steps=6, step_max=DRAG_STEP)
            ee = self.ee()
            log("t=%d drag step %d/%d: tool centre (%.3f,%.3f,%.3f)" % (self.t(), k, n, ee[0], ee[1], ee[2]))
        ee = self.ee()
        yield from self.goto((ee[0], ee[1], self.travel_z()), yaw, grip=0.0, tol=0.02, max_steps=30)
        log("t=%d drag done; hand ended at (%.2f,%.2f)" % (self.t(), ee[0], ee[1]))
        return True

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
        yaw = yaw_branch((x, y), wrap_half(math.pi / 2 + b[2]))[0]
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
        yaw = yaw_branch((x, y), yaw)[0]
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
        # its 12 cm walls: the hand camera checks from above before the inventory is dropped
        self.maybe_empty = [b for b in self.inventory if sum(c["n"] for c in clusters if c["container"] == b) < 15
                            and (self.inventory[b] or self.pile_unknown[b])
                            and self.t() - self.not_empty_t.get(b, -10 ** 9) > 300]
        self.dump("observe", imgs["agentview"][0], clusters)
        return clusters

    def container_components(self, b):
        """Everything standing on container b's floor, seen from straight above with the hand
        camera (the static cameras miss low objects behind the walls): pseudo-clusters to inspect."""
        cb = self.containers[b]
        yield from self.goto((cb[0], cb[1], CONTAINER_RIM_Z + 0.24), 0.0, grip=0.0, tol=0.01, max_steps=40)
        yield from self.hold(2, grip=0.0)
        R_cam, t_cam = self.cam_pose()
        rgb = decode_png(self.obs["rgb"]["eye_in_hand"], "RGB")
        dep = decode_png(self.obs["depth"]["eye_in_hand"])
        pts, _ = backproject_rt(dep, rgb, None, None, R_cam, t_cam)
        lx, ly = container_local(cb, pts[:, 0], pts[:, 1])
        ix, iy = inner_half(b)
        keep = (np.abs(lx) < ix - 0.008) & (np.abs(ly) < iy - 0.008) \
            & (pts[:, 2] > CONTAINER_FLOOR_Z + 0.012) & (pts[:, 2] < CONTAINER_RIM_Z + 0.1)
        q = pts[keep]
        out = []
        if len(q) < 40:
            return out
        lo = q.min(0) - 0.008
        labels, n = label_components(np.floor((q - lo) / 0.008).astype(int))
        for k in range(n):
            m = labels == k
            if m.sum() < 40:
                continue
            top = float(np.percentile(q[m, 2], 97))
            slab = q[m][q[m][:, 2] > top - 0.02]
            gx, gy = (slab[:, :2].mean(0) if len(slab) >= 5 else q[m, :2].mean(0))
            out.append({"grasp_xy": [float(gx), float(gy)], "top_z": top, "floor": CONTAINER_FLOOR_Z, "container": b,
                        "n": int(m.sum()), "centroid": [float(gx), float(gy), float(q[m, 2].mean())], "cls_info": None})
        return sorted(out, key=lambda c: -c["n"])

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
        ix, iy = inner_half(b)
        inside = (np.abs(lx) < ix - 0.01) & (np.abs(ly) < iy - 0.01) \
            & (pts[:, 2] > CONTAINER_FLOOR_Z + 0.012) & (pts[:, 2] < CONTAINER_RIM_Z)
        n = int(inside.sum())
        if n < 40:
            log("t=%d %s container is empty" % (self.t(), b))
            self.inventory[b] = []
            self.pile_unknown[b] = False
            return True
        log("t=%d %s container only looked empty (%d points on its floor)" % (self.t(), b, n))
        self.not_empty_t[b] = self.t()
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
        d["cls_info"] = clamp_by_size(combine(self.classify_eih(d), c.get("cls_info")), d["dims"])
        d["container"] = c["container"]
        if d.get("stack_note"):
            log("t=%d stack: %s" % (self.t(), d["stack_note"]))
        if d.get("blocker"):
            blk = d["blocker"]
            blk["cls_info"] = clamp_by_size(self.classify_eih(blk), blk["dims"])
            blk["container"] = c["container"]
            log("t=%d stack: %s (d=%.2f) rests on the inspected %s at (%.2f,%.2f) top=%.3f; it comes off first" % (
                self.t(), blk["cls_info"]["model"], blk["cls_info"]["d"], d["cls_info"]["model"],
                blk["grasp_xy"][0], blk["grasp_xy"][1], blk["top_z"]))
        self.dump("inspect", rgb, [d] + ([d["blocker"]] if d.get("blocker") else []))
        return d

    # -- grasp planning -------------------------------------------------------------------------
    def choose_yaw(self, c, inside=None, band=None, below_top=GRASP_BELOW_TOP):
        """-> (xy, yaw, tilt) for a pinch of cluster c, or None: fingers across its narrowest side, the
        grasped width under the opening, and the hand clear of every container wall at the grasp
        height (a hand point above a rim is free; where the straight-down hand hits a wall the hand
        leans away from it, TILTS). The hand need not be centred on the object along the finger
        axis: the open fingers are 8 cm apart and the first one to touch pushes the object to the
        middle as they close, so the hand may sit up to ~3 cm off the object. `band` = the points
        the fingers will close on (default: the top rim)."""
        x, y = c["grasp_xy"]
        z_g = max(c["top_z"] - below_top, c["floor"] + GRASP_MIN_ABOVE_FLOOR)
        p = c.get("pts")
        pref = wrap_half(c.get("slab_yaw", c["yaw"]))
        cands = [pref] + [wrap_half(pref + s * d) for d in (0.26, 0.52, 0.79, 1.05, 1.31) for s in (1, -1)] + [wrap_half(pref + 1.57)]

        def fit(hx, hy, yaw):
            return next((t for t in TILTS if fits_with_approach(self.containers, (hx, hy, z_g), yaw, t, inside)), None)
        if band is None and p is not None:
            band = p[p[:, 2] > c["top_z"] - 0.015]      # the top rim only: side walls seen obliquely inflate deeper slabs
        if band is None or len(band) < 10:
            for yaw in cands:
                tilt = fit(x, y, yaw)
                if tilt is not None:
                    return (x, y), yaw, tilt
            return None
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
                tilt = fit(hx, hy, yaw)
                if tilt is not None:
                    # narrow first, then centred and upright, ties to the earlier
                    cost = w + 0.2 * abs(off) + 0.0005 * k + 0.02 * (abs(tilt[0]) + abs(tilt[1]))
                    if best is None or cost < best[0]:
                        best = (cost, (float(hx), float(hy)), yaw, tilt)
                    break
        return None if best is None else (best[1], best[2], best[3])

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
        z_g = max(c["top_z"] - GRASP_BELOW_TOP, c["floor"] + GRASP_MIN_ABOVE_FLOOR)
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
                    tilt = next((t for t in TILTS if fits_with_approach(self.containers, (gx, gy, z_g), yaw, t, inside)), None)
                    if tilt is None:
                        continue
                    best = (cost + 0.02 * (abs(tilt[0]) + abs(tilt[1])), (float(gx), float(gy)), wrap_half(yaw), tilt)
        return None if best is None else (best[1], best[2], best[3])

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
        if rim is not None and inside is not None and (c.get("cls_info") or {}).get("model") not in HOLLOW:
            rim = None      # inside a container two objects with a gap between them look hollow
        pick = self.choose_yaw(c, inside=inside)
        shallow = [(pick[0], pick[1], GRASP_BELOW_TOP, pick[2])] if pick is not None else []
        if rim is not None and (not shallow or c["len_minor"] > 0.055):
            # hollow and wide (bowl, mug, pitcher): the rim grasp, one finger inside, is the plan.
            # A pinch across it only "fits" when the hand camera saw half of it, and then a finger
            # lands on the rim and pushes it away
            return [(rim[0], rim[1], RIM_GRASP_BELOW_TOP, None)]
        if not shallow:
            alt = self.search_pinch(c, inside=inside)
            if alt is not None:
                shallow = [(alt[0], alt[1], GRASP_BELOW_TOP, alt[2])]
        # No grasp deeper than the hand body allows: the body's underside is HAND_BODY_ABOVE_TCP
        # (3.4 cm) above the tool centre point, so fingers more than ~3 cm below an object's top
        # put the body onto the object and push it over instead of closing on it. Grasps planned
        # 4-6.5 cm below the top lifted 16 of 110 tries in the episode logs; 3 cm lifted 58%.
        out += shallow
        if rim is not None:
            out.append((rim[0], rim[1], RIM_GRASP_BELOW_TOP, None))
        return out

    def try_grasps(self, plans, top_z, floor, anchor_xy, keep_top=False):
        """Attempt the planned grasps in order; stop at the first that holds. A plan that closed
        on nothing is tried once more shifted RETRY_JITTER across the fingers (the direction a
        1-2 cm perception error turns into a fingertip landing on the object) and turned a
        little; at most RETRY_MAX attempts in all, so a hopeless object does not eat the deadline."""
        attempts = 0
        for plan in plans:
            xy, yaw, below = plan[0], plan[1], plan[2]
            tilt = plan[3] if len(plan) > 3 else None
            for k in range(2):
                if attempts >= RETRY_MAX:
                    return False
                if k == 1:
                    v = (-math.sin(yaw), math.cos(yaw))        # across the fingers
                    sg = 1.0 if (attempts % 2 == 0) else -1.0
                    xy = (plan[0][0] + sg * RETRY_JITTER * v[0], plan[0][1] + sg * RETRY_JITTER * v[1])
                    yaw = wrap_half(plan[1] + sg * RETRY_YAW)
                attempts += 1
                self.last_grasp_unreachable = False
                held = yield from self.pick(xy, yaw, top_z, floor, below_top=below, anchor_xy=anchor_xy, tilt=tilt, keep_top=keep_top)
                if held:
                    return True
                log("t=%d grasp (yaw %.2f, depth %.3f, tilt %s%s) failed" % (self.t(), yaw, below, fmt_tilt(tilt), ", retry" if k else ""))
                if self.stuck_recovered:
                    # the arm folded up going there: leave the object for now. Only a second
                    # recovery on the way to the same request's target writes it off (three
                    # recoveries in a row once ran a request past its deadline; one, with the
                    # posture-regularised IK, is a hiccup, and giving the target up on it lost
                    # a request whose object was in plain sight)
                    self.stuck_recovered = False
                    self.stuck_strikes += 1
                    self.last_grasp_unreachable = self.stuck_strikes >= 2
                    log("t=%d leaving this object after the stuck-arm recovery%s" % (
                        self.t(), " (second time: written off)" if self.last_grasp_unreachable else ""))
                    return False
                if self.last_descent_short:
                    self.last_grasp_unreachable = True
                    break           # the hand never got there: a shifted retry will not either
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
    def within_reach(c):
        """A loose object a straight-down hand can get to (TIDY_R): nearer the base the hand stops
        above the desk, farther out the arm is at full stretch and folds up."""
        return TIDY_R[0] <= math.hypot(c["grasp_xy"][0], c["grasp_xy"][1]) <= TIDY_R[1]

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
                self.stuck_strikes = 0
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
            if self.request_state is None and self.t() - self.last_patrol_t > PATROL_EVERY:
                yield from self.patrol()
                continue
            yield from self.hold(10, grip=0.0)

    def held_back(self, c):
        """Something a request dug out of a container waits on the desk while the search goes on
        (put back, it would be dug out again); once the target is in red, or absence is settled,
        it goes back before the request is marked."""
        rs = self.request_state
        if not rs or rs.get("in_red") or rs.get("absent_ready"):
            return False
        return self._near_any(c, rs.get("dug", []), r=0.07)

    def tidy(self, clusters):
        loose = [c for c in self.loose(clusters) if not self.skipped(c) and not self.held_back(c) and self.within_reach(c)]
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
                d["grasp_xy"][0], d["grasp_xy"][1], d["top_z"], [round(v, 3) for v in d["dims"]], fmt_plans(g)))
            if d.get("blocker") and info["class_margin"] >= 0.08:
                acted = yield from self.clear_blocker(d, None, clusters)
                if acted:
                    return True
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
        rs = self.request_state
        for c in clusters:
            if c["cls_info"] is None or self.skipped(c):
                continue
            misfiled = c["container"] in CLASS_OF_CONTAINER \
                and c["cls_info"]["cls"] != CLASS_OF_CONTAINER[c["container"]] and c["cls_info"]["class_margin"] > 0.4
            # an object in red that no open request put there is misfiled too (dropped on the
            # way over it); only its far half is within the hand's reach
            in_red = c["container"] == "red" and c["n"] >= 15 and c["grasp_xy"][0] >= RED_REACH_X \
                and not (rs and (rs.get("placed_red") or rs.get("in_red")))
            if misfiled or in_red:
                d = yield from self.inspect(c)
                if d is None or d["cls_info"]["class_margin"] < (0.4 if misfiled else 0.3) or d["cls_info"].get("model_margin", 0.0) < 0.3 \
                        or (misfiled and d["cls_info"]["cls"] == CLASS_OF_CONTAINER[c["container"]]):
                    self.skip(c)
                    return True
                g = self.plan_grasps(d, inside=c["container"])
                log("t=%d misplaced %s in %s grasps=%s" % (self.t(), d["cls_info"]["model"], c["container"], fmt_plans(g)))
                if d.get("blocker"):
                    acted = yield from self.clear_blocker(d, c["container"], clusters)
                    if acted:
                        return True
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

    def patrol(self):
        """Look into each class container from above and inspect the top of its pile: the world
        sometimes drops an object into a wrong container, and once other objects land on it, it is
        buried. A misfiled object found on top goes to its own container right away."""
        self.last_patrol_t = self.t()
        for b in ("gray", "lightblue", "blue"):
            comps = yield from self.container_components(b)
            for c in sorted(comps, key=lambda c: -c["top_z"])[:2]:
                d = yield from self.inspect(c)
                if d is None:
                    continue
                info = d["cls_info"]
                if info.get("model_margin", 0.0) > 0.3:
                    self.seen_in[b].add(info["model"])
                if info["cls"] == CLASS_OF_CONTAINER[b] or info["class_margin"] < 0.4 or info.get("model_margin", 0.0) < 0.3:
                    continue
                g = self.plan_grasps(d, inside=b)
                log("t=%d patrol: %s misfiled in %s grasps=%s" % (self.t(), info["model"], b, fmt_plans(g)))
                if d.get("blocker"):
                    acted = yield from self.clear_blocker(d, b, self.last_clusters)
                    if acted:
                        self.last_patrol_t = -10 ** 9      # the misfiled object is exposed now: come back
                        return
                if not g:
                    continue
                held = yield from self.try_grasps(g, d["top_z"], d["floor"], tuple(d["grasp_xy"]))
                if held:
                    self.forget(info["model"])
                    self.seen_in[b].discard(info["model"])
                    yield from self.place_in_container(CONTAINER_OF_CLASS[info["cls"]], info["model"])
                    self.last_patrol_t = -10 ** 9          # look again soon: more may lie under it
                return

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
        order = [b for b in CLASS_OF_CONTAINER if any(m in names for m in self.seen_in[b]) and b not in order] + order
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
        if rs.get("marked"):
            # marked at the last moment by mark_before_deadline: nothing more to do for it, and
            # no second mark (it would close the next request)
            self.request_state = None
            return False
        if rs.get("placed_red") and not rs.get("in_red") and rs["deadline"] - self.t() <= FINISH_MARGIN:
            # the target went into red but the hand camera never confirmed it there: a delivered
            # mark that turns out wrong costs nothing over letting the request expire
            log("t=%d the target went into red unconfirmed; marking delivered" % self.t())
            rs["in_red"] = True
        if not rs.get("in_red") and not rs.get("absent_ready") and rs["deadline"] - self.t() <= FINISH_MARGIN:
            # out of time without the object in red: a request that expires scores like a wrong
            # mark, so the better guess is marked. Absent when nothing that looked like the target
            # was seen (one request in five names an absent object), else nothing to gain either way
            log("t=%d out of time for %s (seen %s); marking absent rather than expiring" % (
                self.t(), model, bool(rs.get("seen_target"))))
            rs["absent_ready"] = True
        if rs.get("in_red") or rs.get("absent_ready"):
            # the request is judged together with the desk when it is marked: put back what the
            # search dug out and sort what is loose first, as long as the deadline allows
            if rs["deadline"] - self.t() > FINISH_MARGIN:
                did = yield from self.tidy(clusters)
                if did:
                    return True
            mark = 1 if rs.get("in_red") else 2
            yield from self.hold(1, grip=0.0, mark=mark)
            log("t=%d marked %s" % (self.t(), "delivered" if mark == 1 else "absent"))
            self.request_state = None
            return True

        def fresh(slack):
            cs = [c for c in loose if not self._near_any(c, rs["inspected"]) and self.could_be(c, model, slack) and self.within_reach(c)]
            return sorted(cs, key=lambda c: (c["cls_info"].get("all") or {}).get(model, 9.0))

        if not rs.get("red_checked"):
            # an object dropped into red on the way to its container (it stands under the carry
            # path): when it is the target, delivered is right away; one request was marked
            # absent with its object lying in red
            rs["red_checked"] = True
            red = sorted([c for c in clusters if c["container"] == "red" and c["n"] >= 15], key=lambda c: -c["top_z"])
            if red:
                d = yield from self.inspect(red[0])
                if d is not None and is_model(d["cls_info"], model):
                    log("t=%d the target (%s, d=%.2f) is already in the red container" % (self.t(), d["cls_info"]["model"], d["cls_info"]["d"]))
                    rs["seen_target"] = True
                    rs["in_red"] = True
                return True
        cands = fresh(0.5)
        if cands:
            done = yield from self.check_candidate(cands[0])
            return done
        order = self.search_order(model)
        expected = rs.setdefault("expected", list(order))
        searched = rs.setdefault("searched", [])
        stalled = rs.setdefault("stalled", {})           # container -> rounds that found nothing in reach
        exhausted = [b for b in expected if b in searched or stalled.get(b, 0) >= STALL_ROUNDS]
        if len(exhausted) == len(expected) and rs["deadline"] - self.t() > SEARCH_ALL_MARGIN:
            # not where it was expected (or nowhere expected), or the expected pile cannot be dug
            # any further (an ungraspable object on top kept one request on the class container
            # for its whole deadline while the target lay misfiled next door): the world may have
            # dropped it into another container; dig there too while the deadline allows
            order += [b for b in ("gray", "lightblue", "blue") if b not in order]
        blocked = rs.setdefault("blocked", {})
        for b in sorted(order, key=lambda b: stalled.get(b, 0) >= STALL_ROUNDS):     # stalled piles last
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
        if rs.get("best_gap", 9.0) < 0.3 and CLASS_OF.get(rs.get("best_model")) == CLASS_OF.get(model) and not rs.get("tried_best"):
            # a loose object of the target's class that looked a lot like it: more likely the
            # target misread than an absent one (one request in five names an absent object; a
            # pudding box once went to red for an absent tuna fish can at a gap of 0.35)
            rs["tried_best"] = True
            near = [c for c in loose if math.hypot(c["centroid"][0] - rs["best_key"][0], c["centroid"][1] - rs["best_key"][1]) < 0.06]
            if near:
                rs["inspected"] = [k for k in rs["inspected"] if math.hypot(k[0] - rs["best_key"][0], k[1] - rs["best_key"][1]) >= 0.06]
                done = yield from self.check_candidate(near[0], force=True)
                return done
        log("t=%d absent: %s (searched %s); tidying before the mark" % (self.t(), model, rs.get("searched")))
        rs["absent_ready"] = True
        return True

    def check_candidate(self, c, force=False):
        """Inspect a loose cluster from the hand camera; deliver it when it is the target (or when
        `force`d: the closest look-alike once nothing else is left)."""
        rs = self.request_state
        model = rs["model"]
        key = (c["centroid"][0], c["centroid"][1])
        rs["inspected"].append(key)
        d = yield from self.inspect(c)
        if d is None:
            return True
        info = d["cls_info"]
        log("t=%d request check: %s (d=%.2f) vs wanted %s%s" % (self.t(), info["model"], info["d"], model, " (closest look-alike)" if force else ""))
        if not force and not is_model(info, model):
            gap = info.get("all", {}).get(model, 9.0) - info["d"]
            if gap < rs.get("best_gap", 9.0):
                rs["best_gap"], rs["best_key"], rs["best_model"] = gap, key, info["model"]
            return True
        rs["seen_target"] = True            # it is here: absence is never claimed for this request
        g = self.plan_grasps(d)
        if d.get("blocker"):
            acted = yield from self.clear_blocker(d, None, self.last_clusters)
            if acted:
                rs["inspected"].remove(key)     # look at it again once the top is off
                return True
        if not g:
            log("t=%d target not graspable where it lies" % self.t())
            return True
        held = yield from self.try_grasps(g, d["top_z"], d["floor"], tuple(d["grasp_xy"]))
        if not held:
            if self.last_grasp_unreachable:
                # the arm cannot get down to it where it lies (three identical tries once ate a
                # whole request): leave it; absence is never claimed for a seen target
                log("t=%d the target is out of the hand's reach where it lies; leaving it" % self.t())
                rs["out_of_reach"] = "desk"
                return True
            log("t=%d grasp of the target failed; retrying" % self.t())
            rs["inspected"].remove(key)
            return True
        self.forget(model)
        yield from self.place_in_container("red")
        rs["placed_red"] = True
        yield from self.hold(8, grip=0.0)
        ok = yield from self.confirm_red()
        if ok:
            rs["in_red"] = True
            log("t=%d target in red; tidying before the mark" % self.t())
        else:
            log("t=%d red container check failed; retrying" % self.t())
            rs["inspected"].remove(key)
        return True

    def clear_blocker(self, d, b, clusters):
        """Something rests on the object the controller wants (d["blocker"]): take it off first.
        A request's target goes straight to red; anything else waits on a free desk spot (a
        request puts it back before the mark, the tidying sorts it otherwise). -> True when the
        arm did something (the caller re-observes), False when the blocker cannot be moved now
        (the caller goes on with `d` as it lies)."""
        blk = d["blocker"]
        info = blk["cls_info"]
        rs = self.request_state
        model = rs["model"] if rs else None
        g = self.plan_grasps(blk, inside=b)
        log("t=%d stack: taking %s off %s in %s first; grasps=%s" % (
            self.t(), info["model"], d["cls_info"]["model"], b or "the desk", fmt_plans(g)))
        if not g:
            return False
        if rs:
            fails = sum(1 for (fb, fx, fy) in rs.get("failed", []) if fb == b and math.hypot(fx - blk["grasp_xy"][0], fy - blk["grasp_xy"][1]) < 0.06)
            if fails >= UNPACK_FAILS_MAX:
                return False
        is_target = bool(rs) and ((info["model"] == model and info.get("model_margin", 0.0) > 0.3) or (is_model(info, model, 0.15) and info["model"] != model))
        spot = None if is_target else self.free_spot(clusters, near=(blk["centroid"][0], blk["centroid"][1]))
        if not is_target and spot is None:
            log("t=%d no free desk spot for the blocker" % self.t())
            return False
        # the planned top is the topper's own (a tilted object's top is the only part of it the
        # fingers can close on); the refine's "taller neighbour" correction is for flat piles
        held = yield from self.try_grasps(g, blk["top_z"], blk["floor"], tuple(blk["grasp_xy"]), keep_top=True)
        if not held:
            if rs:
                # a descent that never got there counts double: the same approach 300 steps later
                # stops just as short (two such tries once cost a request its deadline)
                rs.setdefault("failed", []).extend([(b, blk["grasp_xy"][0], blk["grasp_xy"][1])] * (2 if self.last_grasp_unreachable else 1))
            return True
        if is_target:
            rs["seen_target"] = True
            self.forget(model)
            yield from self.place_in_container("red")
            rs["placed_red"] = True
            yield from self.hold(8, grip=0.0)
            ok = yield from self.confirm_red()
            if ok:
                rs["in_red"] = True
                log("t=%d target in red (it was the topper in %s); tidying before the mark" % (self.t(), b))
            return True
        yield from self.place_on_desk(spot, self.spot_yaw(spot))
        if rs:
            rs.setdefault("dug", []).append(spot)
        return True

    def unpack_one(self, b, inside, clusters):
        """Dig one object out of container b: the highest cluster with a grasp the hand fits inside
        the container, re-measured from above with the hand camera. The target itself goes
        straight to the red container; anything else is set down on a free desk spot nearby,
        where the next observation tells what it is."""
        rs = self.request_state
        model = rs["model"]
        # the static clusters of a pile are coarse and merged: look at the top from above first
        for c in sorted(inside, key=lambda c: -c["top_z"])[:3]:
            d = yield from self.inspect(c)
            if d is None:
                continue
            g = self.plan_grasps(d, inside=b)
            info = d["cls_info"]
            log("t=%d unpack %s: top %s (d=%.2f, model margin %.2f) at (%.2f,%.2f) top=%.3f grasps=%s" % (
                self.t(), b, info["model"], info["d"], info.get("model_margin", 0.0), d["grasp_xy"][0], d["grasp_xy"][1],
                d["top_z"], fmt_plans(g)))
            if d.get("blocker"):
                acted = yield from self.clear_blocker(d, b, clusters)
                if acted:
                    return True
            if not g:
                if (yield from self.maybe_drag(d, b)):
                    return True
                continue
            fails = sum(1 for (fb, fx, fy) in rs.get("failed", []) if fb == b and math.hypot(fx - d["grasp_xy"][0], fy - d["grasp_xy"][1]) < 0.06)
            if fails >= UNPACK_FAILS_MAX:
                # the same grasp failed twice already this request (one standing clamp took six
                # identical tries of one request): the time is better spent on the next container
                log("t=%d unpack %s: the grasp at (%.2f,%.2f) failed %d times this request; leaving it" % (
                    self.t(), b, d["grasp_xy"][0], d["grasp_xy"][1], fails))
                continue
            is_target = (info["model"] == model and info.get("model_margin", 0.0) > 0.3) or (is_model(info, model, 0.15) and info["model"] != model)
            spot = None if is_target else self.free_spot(clusters, near=(d["centroid"][0], d["centroid"][1]))
            if not is_target and spot is None:
                log("t=%d no free desk spot" % self.t())
                return False
            held = yield from self.try_grasps(g, d["top_z"], d["floor"], tuple(d["grasp_xy"]))
            if not held:
                # the pile has moved: the caller observes again and the dig goes on here with the
                # next cluster (this spot is skipped after UNPACK_FAILS_MAX). Leaving the
                # container after one failed grasp cost a from-above identification pass and a
                # 150-step ban each time, one grasp attempt per visit, and every buried target
                rs.setdefault("failed", []).extend([(b, d["grasp_xy"][0], d["grasp_xy"][1])] * (2 if self.last_grasp_unreachable else 1))
                return True
            if is_target:
                rs["seen_target"] = True
                self.forget(model)
                yield from self.place_in_container("red")
                rs["placed_red"] = True
                yield from self.hold(8, grip=0.0)
                ok = yield from self.confirm_red()
                if ok:
                    rs["in_red"] = True
                    log("t=%d target in red (dug out of %s); tidying before the mark" % (self.t(), b))
                else:
                    log("t=%d red container check failed after digging the target out of %s" % (self.t(), b))
                return True
            yield from self.place_on_desk(spot, self.spot_yaw(spot))
            rs.setdefault("dug", []).append(spot)
            return True
        # nothing in reach: what is left is exposed now; identify it from above. Absence can be
        # claimed once every object left in the container is known and none is the target.
        if not rs.setdefault("identified", {}).get(b):
            rs["identified"][b] = True
            ids, sure = [], True
            comps = yield from self.container_components(b)
            for c in (comps or sorted(inside, key=lambda c: -c["n"]))[:5]:
                d = yield from self.inspect(c)
                if d is None:
                    continue
                info = d["cls_info"]
                g = self.plan_grasps(d, inside=b)
                fails = sum(1 for (fb, fx, fy) in rs.get("failed", []) if fb == b and math.hypot(fx - d["grasp_xy"][0], fy - d["grasp_xy"][1]) < 0.06)
                if g and fails < UNPACK_FAILS_MAX:
                    # seen from above, this one can be taken after all: dig it now (the pass to
                    # identify what is left comes back once nothing has a grasp)
                    rs["identified"][b] = False
                    log("t=%d unpack %s (from above): %s (d=%.2f) at (%.2f,%.2f) top=%.3f grasps=%s" % (
                        self.t(), b, info["model"], info["d"], d["grasp_xy"][0], d["grasp_xy"][1], d["top_z"], fmt_plans(g)))
                    is_target = (info["model"] == model and info.get("model_margin", 0.0) > 0.3) or (is_model(info, model, 0.15) and info["model"] != model)
                    spot = None if is_target else self.free_spot(clusters, near=(d["centroid"][0], d["centroid"][1]))
                    if not is_target and spot is None:
                        continue
                    held = yield from self.try_grasps(g, d["top_z"], d["floor"], tuple(d["grasp_xy"]))
                    if not held:
                        rs.setdefault("failed", []).extend([(b, d["grasp_xy"][0], d["grasp_xy"][1])] * (2 if self.last_grasp_unreachable else 1))
                        return True
                    if is_target:
                        rs["seen_target"] = True
                        self.forget(model)
                        yield from self.place_in_container("red")
                        rs["placed_red"] = True
                        yield from self.hold(8, grip=0.0)
                        if (yield from self.confirm_red()):
                            rs["in_red"] = True
                            log("t=%d target in red (dug out of %s from above); tidying before the mark" % (self.t(), b))
                        return True
                    yield from self.place_on_desk(spot, self.spot_yaw(spot))
                    rs.setdefault("dug", []).append(spot)
                    return True
                if not g and (yield from self.maybe_drag(d, b)):
                    rs["identified"][b] = False
                    return True
                ids.append(info["model"])
                gap = info.get("all", {}).get(model, 9.0) - info["d"]
                if info["model"] == model or TWIN.get(info["model"]) == model or info.get("model_margin", 0.0) < 0.35 or gap < 0.5:
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
        rs.setdefault("stalled", {})[b] = rs["stalled"].get(b, 0) + 1
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
