"""The reference controller, serving the closed-loop wire.

The cart knows nothing about the lab or the packages ahead of time, and one mistake spills
the package, so it finds out what it needs before it commits:

1. Map. Every picture's depth is projected into the world with the cart's pose: points near
   the floor mark floor seen, points above it mark obstacles. The cart drives only where a
   disc around it lies on floor it has seen, towards the nearest edge of what it has seen.
2. Search. Brass pixels at drawer height are drawer pulls. For each in turn the cart stands
   before the drawer, reads the pull's place and direction from a picture, hooks it, pulls the
   drawer open slowly (any drawer may hold the package), takes the arm out of the camera's
   view and looks inside for an orange handle.
3. Weigh. Latched near one end of the handle, raising the latch 3 mm makes the case pivot on
   its far bottom edge; the force tells the weight times a lever. The same at the other end
   gives the weight and where along the handle the centre of mass sits.
4. Lift right above the centre of mass, carry on shaped motions of the cart (a hanging
   package swings at a known period), one stretch at a fixed heading at a time, turning only
   where the package's arc is clear; lower into the vessel, feel its floor in the force,
   release, clear the rim.

The arm is commanded in joint space from a numerical inverse of its kinematics (on the
range's own model of the Panda, arm_model.py) with the tool pointing straight down. When a
step index is asked again, the state is rebuilt from a snapshot, so a repeated request is
answered from its own observation and never advances anything twice; and of the two
observations the one that fits the arm's own motion is taken for the true one.
"""

import base64
import copy
import json
import math
import os
import sys
import zlib

import mujoco
import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_model as A  # noqa: E402  (the arm's own model: kinematics only)

DT = 0.02
G = 9.81

# --- the site, from the task's description -------------------------------------------------
ROOM_X, ROOM_Y, DOOR_HALF = 7.0, 5.6, 0.65
STAND_HALF, STAND_H = 0.20, 0.30
VESSEL_HEIGHT, VESSEL_FLOOR = 0.20, 0.015
ARM_MOUNT = np.array([0.21, 0.0, 0.385])
CAM_MOUNT = np.array([0.33, 0.21, 1.42])
CAM_PITCH = math.radians(40.0)
CAM_W, CAM_H, CAM_FOVY = 240, 180, 62.0
BOX_HALF_X, BOX_HALF_Z, BAR_Z, BAR_HALF, BAR_RADIUS = 0.085, 0.045, 0.085, 0.065, 0.006
PULL_HALF = 0.08
TRAVEL_Q = A.HOME_Q.copy()
TRAVEL_Q[0] = -1.45

# --- the arm's routines --------------------------------------------------------------------
PROBE_S = 0.055          # probe latch points along the bar
PROBE_LIFT = 0.003
PROBE_UP_T, PROBE_DOWN_T = 0.6, 0.7     # s: tip the case up and set it down gently (its contents slosh)
PROBE_SETTLE, PROBE_READ = 40, 25       # control steps: after the lift starts, wait, then average this long
SERVO_LAG = 0.07         # kv / kp of the joint servos (s)
ADMITTANCE = 0.0012      # sideways give of the latch, m/s per N
FORCE_SMOOTH = 0.35      # low-pass on the force used for the give and for contact tests
TOUCH_SPEED = 0.005      # the last centimetre down onto the vessel's floor, m/s
FEEL_SPEED = 0.02        # m/s: the tool's moves between the points where the latch is tried
SWING_PERIOD = 0.63      # the hanging package's swing, s
TILT_SHARE = 0.40        # share of the tilt limit spent on a carry's acceleration
CARRY_MIN_S = 1.2
SWAY_GAIN = 0.024        # sway damping in the air: latch speed per (sideways force / weight), m/s
DOWN = np.array([0.0, 0.0, -1.0])
PULL_REACH = 0.70        # the arm's reach to a closed drawer's pull when the cart stands before it
PULL_STROKE = 0.34       # how far a drawer is pulled out
CARRY_X = 0.40           # where the package hangs while the cart drives: ahead of the arm's axis,
CARRY_Z = 0.74           # the latch at least this high above the floor (over the vessel's rim)
PLACE_REACH = 0.55       # the arm's reach to the vessel's middle when the cart stands before it
LID_OPEN = 1.85          # the vessel's lid: the angle it stands open at,
LID_HANDLE = (0.25, 0.057)   # its handle's axis along the plate from the hinge and off its lower face
LID_SIZE, LID_THICK = 0.29, 0.012
ZVD = ((0.25, 0.0), (0.5, SWING_PERIOD / 2), (0.25, SWING_PERIOD))

# --- the yard, the trolley and the store, from the task's description -------------------------
STORE_X1 = -5.6          # the store's front wall (its door: |y| < 0.9)
TRAY_TOP = 0.50          # the trolley's tray floor; its rim 5 cm higher
HANDLE_X, HANDLE_Z = 0.53, 0.85         # the trolley's handle: ahead of its middle, and its height
SLOTS = (-0.25, 0.0, 0.25)              # where the cart sets packages on the tray, along the trolley
LOAD_REACH = 0.55        # the arm's reach to a place on the tray when the cart stands beside it
HITCH_AHEAD = 0.45       # the handle this far ahead of the cart's centre while it tows
TOW_SPEED = (0.18, 0.05, 0.2, 0.2)      # v, a, w, alpha with the trolley in tow (gentler than its own
                                        # rolling slows it, so that it never runs into the hook)
LANE = 1.05              # in the store, the cart runs at |y| = LANE between the trolley and the vessels
TURN_X = -8.45           # and turns round with a package at x = TURN_X (between the stands)
CLASS_RGB = np.array([[0.15, 0.30, 0.85], [0.15, 0.65, 0.25], [0.55, 0.20, 0.75], [0.92, 0.92, 0.92]])

# --- the cart --------------------------------------------------------------------------------
GRID = 0.05
MAP_X0, MAP_Y0, MAP_NX, MAP_NY = -3.425, -3.425, 216, 137
CART_R = 0.47            # the cart's disc about its turning centre, with a margin
PACK_R = 0.17            # the hanging package's disc, with a margin
UNSEEN_SLACK = 0.14      # floor not seen may come this much nearer than a thing seen
LOOK_RANGE = 3.0         # a drawer pull shows in a picture from no farther than this
PACK_AHEAD = ARM_MOUNT[0] + CARRY_X
FREE_SPEED = (0.45, 0.45, 0.9, 1.4)      # v, a, w, alpha while searching
BASE_GAIN = np.array([4.0, 4.0, 4.0])


# ==========================================================================================
# kinematics of the arm (in the arm's own frame: origin at its base, x forward)
# ==========================================================================================
class Kinematics(object):
    """Forward and inverse kinematics on the range's own model of the arm. The tool is kept
    pointing straight down with its heading turning with the target's bearing, which leaves
    the seven-joint arm one free motion (its elbow), held near the ready pose turned to the
    same bearing."""

    def __init__(self):
        self.m = A.build_arm()
        self.d = mujoco.MjData(self.m)
        self.site = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, "latch")
        self.jp = np.zeros((3, self.m.nv))
        self.jr = np.zeros((3, self.m.nv))
        _, R = self.fk(A.HOME_Q)
        self.heading = R[:, 0].copy()

    def fk(self, q):
        self.d.qpos[:7] = q
        mujoco.mj_kinematics(self.m, self.d)
        return self.d.site_xpos[self.site].copy(), self.d.site_xmat[self.site].reshape(3, 3).copy()

    def ik(self, p, q0, iters=3):
        q = np.array(q0, float)
        az = math.atan2(p[1], p[0])
        c, s = math.cos(az), math.sin(az)
        heading = np.array([c * self.heading[0] - s * self.heading[1],
                            s * self.heading[0] + c * self.heading[1], 0.0])
        rest = A.HOME_Q.copy()
        rest[0] = az
        for _ in range(iters):
            x, R = self.fk(q)
            mujoco.mj_comPos(self.m, self.d)
            mujoco.mj_jacSite(self.m, self.d, self.jp, self.jr, self.site)
            J = np.vstack([self.jp[:, :7], self.jr[:, :7]])
            e_rot = 0.5 * (np.cross(R[:, 2], DOWN) + np.cross(R[:, 0], heading))
            e = np.concatenate([p - x, e_rot])
            JJt = J @ J.T + 1e-6 * np.eye(6)
            dq = J.T @ np.linalg.solve(JJt, e)
            null = np.eye(7) - J.T @ np.linalg.solve(JJt, J)
            dq += null @ (0.3 * (rest - q))
            q = np.clip(q + dq, A.JOINT_RANGE[:, 0] + 0.01, A.JOINT_RANGE[:, 1] - 0.01)
        return q


KIN = None


def kin():
    global KIN
    if KIN is None:
        KIN = Kinematics()
    return KIN


def minjerk(s):
    s = min(1.0, max(0.0, s))
    return s * s * s * (10 - 15 * s + 6 * s * s)


class Move(object):
    """A straight min-jerk move from a to b in T seconds. With `shaper` (pairs of weight and
    delay) the move is the weighted sum of delayed copies of itself: an input shaper, which
    leaves a pendulum of the shaper's period at rest when the move ends."""

    def __init__(self, a, b, T, shaper=((1.0, 0.0),)):
        self.a, self.b, self.T = np.array(a, float), np.array(b, float), max(T, DT)
        self.shaper = shaper
        self.total = self.T + max(d for _, d in shaper)
        self.k = 0

    def step(self):
        self.k += 1
        t = self.k * DT
        s = sum(w * minjerk((t - d) / self.T) for w, d in self.shaper)
        return self.a + (self.b - self.a) * s, t >= self.total


# ==========================================================================================
# frames
# ==========================================================================================
def rot(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def arm_to_world(base, p):
    xy = np.asarray(base[:2]) + rot(base[2]) @ (np.asarray(p[:2]) + ARM_MOUNT[:2])
    return np.array([xy[0], xy[1], p[2] + ARM_MOUNT[2]])


def world_to_arm(base, p):
    xy = rot(base[2]).T @ (np.asarray(p[:2]) - np.asarray(base[:2])) - ARM_MOUNT[:2]
    return np.array([xy[0], xy[1], p[2] - ARM_MOUNT[2]])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ==========================================================================================
# the camera
# ==========================================================================================
def _camera_frame():
    f = (CAM_H / 2.0) / math.tan(math.radians(CAM_FOVY) / 2.0)
    u, v = np.meshgrid(np.arange(CAM_W) + 0.5 - CAM_W / 2.0, np.arange(CAM_H) + 0.5 - CAM_H / 2.0)
    rays = np.stack([u / f, -v / f, -np.ones_like(u)], axis=-1)        # per unit of depth
    c, s = math.cos(CAM_PITCH), math.sin(CAM_PITCH)
    R = np.array([[0.0, s, -c], [-1.0, 0.0, 0.0], [0.0, c, s]])        # camera axes in the cart's frame
    return rays @ R.T


CAM_DIRS = _camera_frame()               # (H, W, 3): a pixel's point in the cart's frame per metre of depth


def decode(d):
    return np.frombuffer(zlib.decompress(base64.b64decode(d["zlib_b64"])), dtype=d["dtype"]).reshape(d["shape"])


class View(object):
    """One picture, as points in the world."""

    def __init__(self, pic, base):
        depth = decode(pic["depth_mm"]).astype(float) / 1000.0
        self.rgb = decode(pic["rgb"]).astype(float)
        self.valid = depth > 0.0
        self.cart = CAM_MOUNT + CAM_DIRS * depth[:, :, None]           # in the cart's frame
        R = rot(base[2])
        self.P = np.empty_like(self.cart)
        self.P[:, :, :2] = self.cart[:, :, :2] @ R.T + np.asarray(base[:2])
        self.P[:, :, 2] = self.cart[:, :, 2]
        cam = np.asarray(base[:2]) + R @ CAM_MOUNT[:2]
        self.cam = np.array([cam[0], cam[1], CAM_MOUNT[2]])
        # the cart's own body and arm (in its travel pose) are not the world
        c = self.cart
        self.own = (c[:, :, 0] > -0.5) & (c[:, :, 0] < 0.48) & (np.abs(c[:, :, 1]) < 0.36)
        r, g, b = self.rgb[:, :, 0], self.rgb[:, :, 1], self.rgb[:, :, 2]
        lit = self.valid & ~self.own & (r > 70)
        self.orange = lit & (g > 0.43 * r) & (g < 0.53 * r) & (b < 0.10 * r)
        self.brass = lit & (g > 0.72 * r) & (g < 0.92 * r) & (b < 0.16 * r)
        lit_b = self.valid & ~self.own & (b > 60)
        self.cyan = lit_b & (r < 0.3 * b) & (g > 0.65 * b) & (g < 1.05 * b)


def classify(rgb):
    """The class whose colour is nearest in hue and saturation (the shade scales all three
    channels alike): the index into CLASS_RGB, or None for too few pixels."""
    if len(rgb) < 4:
        return None
    c = np.median(np.asarray(rgb, float), axis=0)
    c = c / max(1e-6, float(np.linalg.norm(c)))
    k = CLASS_RGB / np.linalg.norm(CLASS_RGB, axis=1)[:, None]
    return int(np.argmax(k @ c))


def bar_from(view, mask, radius):
    """A bar's middle, direction and length from the pixels on it: the points seen lie on the
    half of its surface that faces the camera, on average pi/4 of its radius off its axis."""
    pts = view.P[mask]
    if len(pts) < 6:
        return None
    c = pts.mean(axis=0)
    d = pts[:, :2] - c[:2]
    w, v = np.linalg.eigh(d.T @ d)
    u = np.array([v[0, 1], v[1, 1], 0.0])
    along = d @ u[:2]
    lo, hi = np.percentile(along, 1.0), np.percentile(along, 99.0)
    mid = c + u * 0.5 * (0.5 * (lo + hi))                      # the mean and the middle of the ends, averaged
    to_cam = view.cam - mid
    to_cam = to_cam - u * (to_cam @ u)
    to_cam = to_cam / np.linalg.norm(to_cam)
    axis = mid - to_cam * (math.pi / 4.0) * radius
    return dict(mid=axis, dir=u, length=float(hi - lo), n=int(len(pts)))


# ==========================================================================================
# the map
# ==========================================================================================
S4 = ndimage.generate_binary_structure(2, 1)
S8 = ndimage.generate_binary_structure(2, 2)


def cell(p):
    return int((p[0] - MAP_X0) / GRID), int((p[1] - MAP_Y0) / GRID)


def centre(i, j):
    return np.array([MAP_X0 + (i + 0.5) * GRID, MAP_Y0 + (j + 0.5) * GRID])


class Map(object):
    def __init__(self, vessel):
        self.floor = np.zeros((MAP_NX, MAP_NY), bool)
        self.obst = np.zeros((MAP_NX, MAP_NY), bool)
        X, Y = np.meshgrid(MAP_X0 + (np.arange(MAP_NX) + 0.5) * GRID,
                           MAP_Y0 + (np.arange(MAP_NY) + 0.5) * GRID, indexing="ij")
        self.X, self.Y = X, Y
        # what is known before the first picture: open ground outside, the vessel's stand
        self.floor[(X < -0.3) & (X > MAP_X0 + 0.3) & (np.abs(Y) < -MAP_Y0 - 0.3)] = True
        for v in vessel:
            self.obst[(np.abs(X - v[0]) < STAND_HALF + 0.03) & (np.abs(Y - v[1]) < STAND_HALF + 0.03)] = True
        self.inside = (X > 0.08) & (X < ROOM_X - 0.08) & (np.abs(Y) < ROOM_Y / 2.0 - 0.08)
        self.near = np.full((MAP_NX, MAP_NY), np.inf)       # from how near a thing here was seen
        self.sides = np.zeros((MAP_NX, MAP_NY), np.uint8)   # and from which of four sides (bits)

    def add(self, view):
        ok = view.valid & ~view.own
        P = view.P[ok]
        i = ((P[:, 0] - MAP_X0) / GRID).astype(int)
        j = ((P[:, 1] - MAP_Y0) / GRID).astype(int)
        keep = (i >= 0) & (i < MAP_NX) & (j >= 0) & (j < MAP_NY)
        i, j, z = i[keep], j[keep], P[keep, 2]
        low = np.abs(z) < 0.03
        high = (z > 0.05) & (z < 1.9)
        self.floor[i[low], j[low]] = True
        self.obst[i[high], j[high]] = True
        rng = np.hypot(P[keep, 0] - view.cam[0], P[keep, 1] - view.cam[1])
        np.minimum.at(self.near, (i[high], j[high]), rng[high])
        close = high & (rng < LOOK_RANGE)
        quarter = (np.floor((np.arctan2(P[keep, 1] - view.cam[1], P[keep, 0] - view.cam[0]) + math.pi)
                            / (math.pi / 2.0)).astype(int) % 4)
        np.bitwise_or.at(self.sides, (i[close], j[close]), (1 << quarter[close]).astype(np.uint8))

    def clear(self, xy, radius):
        """Forget what stood here (something the cart has carried away)."""
        near = (self.X - xy[0]) ** 2 + (self.Y - xy[1]) ** 2 < radius ** 2
        self.obst[near] = False
        self.floor[near] = True

    def free(self):
        return self.floor & ~self.obst

    def room(self, radius):
        """Cells with nothing seen standing within this radius, and floor seen all around
        (a little less far: the floor right behind a thing is in its shadow). What collides
        is a box around each piece, so the gaps in what is seen of it (between a chair's
        legs, through a plant's leaves) are closed first."""
        solid = ndimage.binary_closing(self.obst, structure=np.ones((5, 5), bool))
        return (ndimage.distance_transform_edt(~solid) * GRID > radius) \
            & (ndimage.distance_transform_edt(self.floor | self.obst) * GRID > max(0.1, radius - UNSEEN_SLACK))

    @staticmethod
    def wave(mask, start):
        """Path lengths (m, about) inside `mask` from a cell, by alternating 4- and
        8-neighbour steps; inf where there is no way."""
        dist = np.full(mask.shape, np.inf)
        if not mask[start]:
            return dist
        cur = np.zeros(mask.shape, bool)
        cur[start] = True
        seen = cur.copy()
        dist[start] = 0.0
        k = 0
        while cur.any():
            k += 1
            cur = ndimage.binary_dilation(cur, structure=S8 if k % 2 == 0 else S4) & mask & ~seen
            dist[cur] = k * GRID * 1.1
            seen |= cur
        return dist

    @staticmethod
    def sight(mask, a, b):
        n = max(2, int(np.linalg.norm(np.asarray(b) - np.asarray(a)) / 0.02))
        for s in np.linspace(0.0, 1.0, n):
            p = np.asarray(a) + (np.asarray(b) - np.asarray(a)) * s
            i, j = cell(p)
            if not (0 <= i < MAP_NX and 0 <= j < MAP_NY and mask[i, j]):
                return False
        return True

    @staticmethod
    def nearest(mask, p, reach=0.6):
        """The cell of `mask` nearest to a point, within reach; or None."""
        i0, j0 = cell(p)
        n = int(reach / GRID) + 1
        best = None
        for i in range(max(0, i0 - n), min(MAP_NX, i0 + n + 1)):
            for j in range(max(0, j0 - n), min(MAP_NY, j0 + n + 1)):
                if mask[i, j]:
                    d = float(np.linalg.norm(centre(i, j) - np.asarray(p[:2])))
                    if d <= reach and (best is None or d < best[0]):
                        best = (d, (i, j))
        return None if best is None else best[1]

    @staticmethod
    def path(mask, dist, goal):
        """Way points from the wave's start to a cell: the wave's descent, straightened."""
        if not np.isfinite(dist[goal]):
            return None
        cells = [goal]
        i, j = goal
        while dist[i, j] > 0.0:
            best = None
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    u, v = i + di, j + dj
                    if (di or dj) and 0 <= u < MAP_NX and 0 <= v < MAP_NY and dist[u, v] < dist[i, j]:
                        key = dist[u, v] + 0.01 * (abs(di) + abs(dj))
                        if best is None or key < best[0]:
                            best = (key, u, v)
            if best is None:
                return None
            i, j = best[1], best[2]
            cells.append((i, j))
        pts = [centre(*c) for c in reversed(cells)]
        out = [pts[0]]
        k = 0
        while k < len(pts) - 1:
            n = len(pts) - 1
            while n > k + 1 and not Map.sight(mask, pts[k], pts[n]):
                n -= 1
            out.append(pts[n])
            k = n
        return out


def shifted(mask, offset):
    """mask[i + di, j + dj] at [i, j]: is the cell at this offset (m) in the mask?"""
    di, dj = int(round(offset[0] / GRID)), int(round(offset[1] / GRID))
    out = np.zeros_like(mask)
    a0, a1 = max(0, -di), min(MAP_NX, MAP_NX - di)
    b0, b1 = max(0, -dj), min(MAP_NY, MAP_NY - dj)
    out[a0:a1, b0:b1] = mask[a0 + di:a1 + di, b0 + dj:b1 + dj]
    return out


# ==========================================================================================
# the cart's drive
# ==========================================================================================
class Drive(object):
    """Follows a list of poses one stretch at a time: each a min-jerk move of (x, y, yaw),
    shaped for the hanging package when it carries one."""

    def __init__(self, pose):
        self.ref = np.array(pose, float)
        self.move = None
        self.todo = []
        self.arrived = True

    def busy(self):
        return self.move is not None or bool(self.todo)

    def to(self, xy=None, yaw=None, limits=FREE_SPEED, shaper=((1.0, 0.0),)):
        self.todo.append((None if xy is None else [float(xy[0]), float(xy[1])], yaw, limits, shaper))

    def act(self, obs):
        if self.move is None and self.todo:
            xy, yaw, (v, a, w, al), shaper = self.todo.pop(0)
            goal = self.ref.copy()
            if xy is not None:
                goal[:2] = xy
            if yaw is not None:
                goal[2] = self.ref[2] + wrap(yaw - self.ref[2])
            D, th = float(np.linalg.norm(goal[:2] - self.ref[:2])), abs(goal[2] - self.ref[2])
            T = max(0.4, 1.875 * D / v, math.sqrt(5.774 * D / a), 1.875 * th / w, math.sqrt(5.774 * th / al))
            self.move = Move(self.ref, goal, T, shaper)
        nxt = self.ref
        if self.move is not None:
            nxt, done = self.move.step()
            if done:
                self.move = None
        meas = np.array(obs["base"], float)
        gap = self.ref - meas
        self.arrived = bool(np.linalg.norm(gap[:2]) < 0.0006 and abs(gap[2]) < 0.001
                            and np.linalg.norm(obs["base_vel"]) < 0.004)
        v = (nxt - self.ref) / DT + BASE_GAIN * gap
        self.ref = np.array(nxt, float)
        vx, vy = rot(meas[2]).T @ v[:2]
        return [float(vx), float(vy), float(v[2])]


# ==========================================================================================
# the arm's routines (all points in the arm's frame)
# ==========================================================================================
class Arm(object):
    def __init__(self):
        self.s = dict(k=0, feeling=None, integ=np.zeros(7), p=kin().fk(TRAVEL_Q)[0], move=None, q_ref=TRAVEL_Q.copy(),
                      latch=0, probe=[], forces=[], queue=[("tare",)], weight=None, xc=None, touch=0,
                      adm=np.zeros(3), adm_gain=0.0, adm_free=None, leak=0.0, ff=np.zeros(3), latch_s=[],
                      gain_above=None, tips=[], tare=[], err=0.0, shift=np.zeros(3), fail="")

    def busy(self):
        return self.s["move"] is not None or bool(self.s["queue"])

    # -- helpers ----------------------------------------------------------------------------
    def force(self, obs):
        """Payload force on the latch in the arm's frame (N), tare removed."""
        f = np.array(obs["wrench"][:3]) - self.s["bias"][:3]
        return kin().fk(np.array(obs["q"]))[1] @ f

    def bar_point(self, s_along, dz=0.0):
        st = self.s
        return st["bar0"] + st["bar_dir"] * s_along + np.array([0.0, 0.0, dz])

    def here(self):
        return self.s["p"] + self.s["adm"]

    def go(self, target, speed=0.25, tmin=0.3, relative=True, T=None, shaper=((1.0, 0.0),)):
        """Start a move to `target`. A target named relative to the bar (`relative`) carries
        the package's shift: the give gathered while it stood is how far the latch, and the
        bar in it, has moved."""
        st = self.s
        st["shift"] = st["shift"] + st["adm"]
        target = np.array(target, float) + (st["shift"] if relative else 0.0)
        start = st["p"] + st["adm"]
        st["adm"] = np.zeros(3)
        st["p"] = start
        if T is None:
            T = max(tmin, 1.9 * float(np.linalg.norm(target - start)) / speed)
        st["move"] = Move(start, target, T, shaper)

    # -- the jobs -----------------------------------------------------------------------------
    def open_drawer(self, mid, out):
        """Hook the pull at `mid`, draw it `out` (a unit vector) by the stroke, let go and
        fold away."""
        st = self.s
        st["shift"] = np.zeros(3)
        st["fail"] = ""
        up = np.array([0.0, 0.0, 0.09])
        st["queue"] = [("goto", mid + up, 0.45, "abs"), ("goto", mid, 0.12, "abs"),
                       ("latch", 1, 0.0025), ("held",),
                       ("pull", np.asarray(out) * PULL_STROKE, 3.6),
                       ("settle", 15), ("mark",), ("latch", 0), ("gain", 0.0),
                       ("rise", 0.10), ("goto", kin().fk(TRAVEL_Q)[0], 0.45, "abs")]

    def shut_drawer(self, mid, out, travel):
        """Hook the pull of an open drawer at `mid` and push it back in by `travel`."""
        st = self.s
        st["shift"] = np.zeros(3)
        st["fail"] = ""
        up = np.array([0.0, 0.0, 0.09])
        st["queue"] = [("goto", mid + up, 0.45, "abs"), ("goto", mid, 0.12, "abs"),
                       ("latch", 1, 0.0025), ("held",),
                       ("pull", -np.asarray(out) * travel, 3.0),
                       ("settle", 10), ("latch", 0), ("gain", 0.0),
                       ("rise", 0.10), ("goto", kin().fk(TRAVEL_Q)[0], 0.45, "abs")]

    def shut_lid(self, hinge, across):
        """Close the vessel's lid: hook its handle where the lid stands open, lead it over
        on its circle about the hinge (a point of the hinge line, and the unit vector across
        the vessel's mouth), set it down and let go."""
        st = self.s
        st["shift"] = np.zeros(3)
        st["fail"] = ""
        start = self.lid_point(hinge, across, LID_OPEN)
        st["queue"] = [("rise", 0.06), ("goto", start + np.array([0.0, 0.0, 0.10]), 0.35, "abs"),
                       ("goto", start, 0.10, "abs"),
                       ("latch", 1, 0.0025), ("held",),
                       ("arc", np.array(hinge, float), np.array(across, float), 4.5),
                       ("settle", 15), ("latch", 0), ("gain", 0.0),
                       ("rise", 0.12)]

    @staticmethod
    def lid_point(hinge, across, angle):
        along, off = LID_HANDLE
        c, s = math.cos(angle), math.sin(angle)
        return np.asarray(hinge) + np.asarray(across) * (along * c - off * s) + np.array([0.0, 0.0, along * s + off * c])

    def pick(self, mid, direction, lift_to):
        """Weigh the package by its handle (middle `mid`, along `direction`) and lift it
        until the latch is at height `lift_to`."""
        st = self.s
        st.update(bar0=np.array(mid, float), bar_dir=np.array(direction, float), shift=np.zeros(3), fail="",
                  forces=[], latch_s=[], probe=[], weight=None, xc=None, lift_to=float(lift_to))
        st["queue"] = [("over", self.bar_point(-PROBE_S, 0.10)),
                       ("goto", self.bar_point(-PROBE_S, 0.10), 0.45),
                       ("goto", self.bar_point(-PROBE_S, 0.04), 0.30),
                       ("goto", self.bar_point(-PROBE_S), 0.16),
                       ("feel", -PROBE_S), ("to_bar", -PROBE_S),
                       ("latch", 1), ("held",), ("probe",),
                       ("latch", 0),
                       ("to_bar", PROBE_S),
                       ("feel", PROBE_S), ("to_bar", PROBE_S),
                       ("latch", 1), ("held",), ("probe",), ("solve",)]

    def place(self, vessel, direction, tilt_limit):
        """Lower the hanging package into the vessel (its middle, on the stand's top)."""
        st = self.s
        centre_ = np.array([vessel[0], vessel[1], 0.0]) + np.array(direction, float) * st["xc"]
        z = self.here()[2]
        above = np.array([centre_[0], centre_[1], z])
        low = vessel[2] + VESSEL_FLOOR + BOX_HALF_Z + BAR_Z
        st["queue"] = [("carry", np.array([self.here()[0], above[1], z]), tilt_limit),
                       ("carry", above, tilt_limit),
                       ("goto", np.array([above[0], above[1], low + 0.008]), 0.20, "abs"),
                       ("touch", np.array([above[0], above[1], low - 0.012])),
                       ("relax",),
                       ("latch", 0),
                       ("gain", 0.0),
                       ("goto", np.array([above[0], above[1], vessel[2] + VESSEL_HEIGHT + 0.09]), 0.4, "abs")]

    # -- one control step -----------------------------------------------------------------
    def act(self, obs):
        st = self.s
        st["k"] += 1
        self.run(obs)
        if "bias" in st:
            st["ff"] = st["ff"] + FORCE_SMOOTH * (self.force(obs) - st["ff"])
        if st["gain_above"] and st["p"][2] > st["gain_above"][0]:
            st["adm_gain"] = st["gain_above"][1]
            st["gain_above"] = None
        if obs["latched"] and st["latch"] and st["adm_gain"] > 0.0:
            f = st["ff"].copy()
            if st["adm_free"] is None:
                f[2] = 0.0                               # give sideways only
            else:
                f = f - st["adm_free"] * (f @ st["adm_free"])      # give across the pull, up and down too
            st["adm"] -= st["adm_gain"] * DT * f
        if st["leak"] > 0.0:
            st["adm"] *= 1.0 - DT / st["leak"]
        prev = st["q_ref"]
        q_ref = kin().ik(st["p"] + st["adm"], prev)
        q = np.array(obs["q"])
        st["q_ref"] = q_ref
        tip = kin().fk(q)[0]
        st["tips"] = (st["tips"] + [tip])[-3:]
        st["err"] = float(np.linalg.norm(tip - st["p"] - st["adm"]))
        st["integ"] = np.clip(st["integ"] + 1.5 * DT * (q_ref - q), -0.03, 0.03)
        return (q_ref + SERVO_LAG * (q_ref - prev) / DT + st["integ"]).tolist()

    def run(self, obs):
        st = self.s
        if st["move"] is not None:
            st["p"], done = st["move"].step()
            if done:
                st["move"] = None
            return
        if not st["queue"]:
            return
        item = st["queue"][0]
        kind = item[0]
        if kind == "tare":
            st["tare"].append(obs["wrench"])
            if len(st["tare"]) >= 12:
                st["bias"] = np.mean(st["tare"], axis=0)
                st["queue"].pop(0)
        elif kind == "goto":
            st["queue"].pop(0)
            self.go(item[1], speed=item[2], relative=(len(item) < 4))
        elif kind == "to_bar":
            # to a point of the handle's axis, as it is known now
            st["queue"].pop(0)
            self.go(self.bar_point(item[1]), speed=0.10, relative=False)
        elif kind == "ease":
            st["queue"].pop(0)
            self.go(self.here() + np.array([0.0, 0.0, item[1]]), speed=item[2], relative=False)
        elif kind == "feel":
            self.feel(obs, item[1])
        elif kind == "over":
            # never sideways below where it is going: first straight up to that height
            st["queue"].pop(0)
            here = self.here()
            if here[2] < item[1][2] - 0.005:
                self.go(np.array([here[0], here[1], item[1][2]]), speed=0.35, relative=False)
        elif kind == "rise":
            st["queue"].pop(0)
            self.go(self.here() + np.array([0.0, 0.0, item[1]]), speed=0.3, relative=False)
        elif kind == "mark":
            st["queue"].pop(0)
            st["mark"] = np.median(st["tips"], axis=0)       # where the latch holds the bar now
        elif kind == "settle":
            st["wait"] = st.get("wait", 0) + 1
            if st["wait"] >= item[1]:
                st["wait"] = 0
                st["queue"].pop(0)
        elif kind == "gain":
            st["queue"].pop(0)
            st["adm_gain"] = item[1]
            st["adm_free"] = None
            st["gain_above"] = None
        elif kind == "gain_above":
            st["queue"].pop(0)
            st["gain_above"] = (item[1], item[2])
        elif kind == "latch":
            tol = item[2] if len(item) > 2 else 0.003
            st["wait"] = st.get("wait", 0) + 1
            if item[1] == 1 and st["err"] > tol and st["wait"] < 25:
                return
            st["latch"] = item[1]
            st["held"] = st.get("held", 0) + 1
            if st["held"] >= 2:
                st["wait"] = st["held"] = 0
                st["queue"].pop(0)
        elif kind == "held":
            # the latch closes only on a bar within its reach: if it did not, stop here
            st["queue"].pop(0)
            if not obs["latched"]:
                st["fail"] = "latch"
                st["latch"] = 0
                st["queue"] = [("rise", 0.10), ("goto", kin().fk(TRAVEL_Q)[0], 0.45, "abs")]
        elif kind == "pull":
            st["queue"].pop(0)
            st["adm_gain"] = ADMITTANCE
            st["adm_free"] = np.asarray(item[1]) / np.linalg.norm(item[1])
            self.go(self.here() + item[1], T=item[2], relative=False)
        elif kind == "arc":
            # the lid's handle on its circle about the hinge, from open to shut, giving way
            # across the circle so that a small error in where the hinge is jams nothing
            a = st.get("arc")
            if a is None:
                a = st["arc"] = dict(k=0)
                st["adm_gain"] = ADMITTANCE
            a["k"] += 1
            u = minjerk(a["k"] * DT / item[3])
            angle = LID_OPEN * (1.0 - u) + 0.015 * u
            st["p"] = self.lid_point(item[1], item[2], angle)
            tangent = self.lid_point(item[1], item[2], angle - 0.01) - st["p"]
            st["adm_free"] = tangent / np.linalg.norm(tangent)
            if a["k"] * DT >= item[3]:
                st["arc"] = None
                st["queue"].pop(0)
        elif kind == "probe":
            pr = st["probe"]
            if not pr:
                st["probe"] = ["up", 0, []]
                st["probe_base"] = st["p"].copy()
                st["adm_gain"] = ADMITTANCE
                tip = np.median(st["tips"], axis=0)
                st["latch_s"].append(float((tip - st["bar0"] - st["shift"])[:2] @ st["bar_dir"][:2]))
                # the case tips about its far bottom edge: the latch goes up along that arc, toward
                # the edge by (handle's height / its distance from the edge) of the lift, so that
                # the case is not dragged across the floor (and does not stick and slip)
                s_here = st["latch_s"][-1]
                arc = (BOX_HALF_Z + BAR_Z) / (BOX_HALF_X + abs(s_here))
                toward = -math.copysign(1.0, s_here) * st["bar_dir"] * PROBE_LIFT * arc
                self.go(st["p"] + np.array([0, 0, PROBE_LIFT]) + toward, T=PROBE_UP_T, relative=False)
            elif pr[0] == "up":
                pr[1] += 1
                if pr[1] > PROBE_SETTLE:
                    f = self.force(obs)
                    pr[2].append([float(f[2]), float(f[:2] @ st["bar_dir"][:2])])
                if pr[1] >= PROBE_SETTLE + PROBE_READ:
                    st["forces"].append(np.mean(pr[2], axis=0).tolist())
                    st["probe"] = ["down", 0, []]
                    base0 = st["probe_base"]
                    self.go(np.array([base0[0], base0[1], base0[2] + 0.0006]), T=PROBE_DOWN_T, relative=False)
            elif pr[0] == "down":
                pr[1] += 1
                f = st["ff"]
                if f[2] < 0.5 or st["p"][2] < st["probe_base"][2] - 0.004:
                    pr[2].append(1)
                    if len(pr[2]) >= 3:
                        st["probe"] = []
                        st["adm_gain"] = 0.0
                        st["queue"].pop(0)
                else:
                    pr[2] = []
                    st["p"] = st["p"] + np.array([0, 0, -min(0.006, 0.002 + 0.0004 * f[2]) * DT])
        elif kind == "solve":
            # moments about the edge the case pivots on, with the latch force (vertical fz,
            # along the bar fs) at height H: latched at -PROBE_S it pivots on the +x edge,
            #   weight * (BOX_HALF_X - xc) = (BOX_HALF_X - s1) * fz1 + H * fs1,
            # latched at +PROBE_S on the -x edge,
            #   weight * (BOX_HALF_X + xc) = (BOX_HALF_X + s2) * fz2 - H * fs2
            (fz1, fs1), (fz2, fs2) = st["forces"]
            s1, s2 = st["latch_s"]
            H = BOX_HALF_Z + BAR_Z
            m1 = (BOX_HALF_X - s1) * fz1 + H * fs1
            m2 = (BOX_HALF_X + s2) * fz2 - H * fs2
            st["weight"] = (m1 + m2) / (2 * BOX_HALF_X)
            xc = BOX_HALF_X * (m2 - m1) / (m1 + m2)
            st["xc"] = float(np.clip(xc, -BAR_HALF + 0.004, BAR_HALF - 0.004))
            end = PROBE_S if st["xc"] >= 0 else -PROBE_S
            st["queue"] = [("latch", 0),
                           ("to_bar", end),
                           ("feel", end), ("to_bar", st["xc"]),
                           ("latch", 1, 0.0012), ("held",),
                           ("gain", ADMITTANCE),
                           ("ease", 0.006, 0.02),
                           ("gain_above", st["bar0"][2] + 0.02, SWAY_GAIN * G / st["weight"]),
                           ("lift", st["lift_to"]),
                           ("stow", obs["tilt_limit_deg"])]
        elif kind == "lift":
            st["queue"].pop(0)
            here = self.here()
            self.go(np.array([here[0], here[1], item[1]]), T=2.2, relative=False)
        elif kind == "stow":
            # bring the hanging package to where it rides, ahead of the arm's axis
            st["queue"].pop(0)
            self.carry(np.array([CARRY_X, 0.0, self.here()[2]]), item[1])
            st["leak"] = 4.0
        elif kind == "carry":
            st["queue"].pop(0)
            self.carry(item[1], item[2])
        elif kind == "relax":
            st["adm_gain"] = ADMITTANCE
            st["leak"] = 0.0
            f = st["ff"]
            st["relax"] = st.get("relax", 0) + 1
            if f[2] > 0.5:
                st["p"] = st["p"] + np.array([0, 0, -0.004 * DT])
            quiet = float(np.linalg.norm(f[:2])) < 0.6 and f[2] < 0.8
            st["quiet"] = st.get("quiet", 0) + 1 if quiet else 0
            if st["quiet"] >= 3 or st["relax"] > 40:
                st["adm_gain"] = 0.0
                st["relax"] = st["quiet"] = 0
                st["queue"].pop(0)
        elif kind == "touch":
            fz = abs(float(st["ff"][2]))
            if fz < 0.8 * st["weight"]:
                st["adm_gain"] = ADMITTANCE
                st["leak"] = 0.0
            if fz < 0.35 * st["weight"]:
                st["touch"] += 1
            else:
                st["touch"] = 0
            if st["touch"] >= 3 or st["p"][2] <= item[1][2]:
                st["queue"].pop(0)
            else:
                st["p"] = st["p"] + np.array([0, 0, -TOUCH_SPEED * DT])

    def feel(self, obs, s0):
        """Find the handle with the latch, which closes only within its reach of the handle's
        axis: halve the interval between a point where it closes and one where it does not,
        on either side across the handle and then past the handle's end. The handle's axis
        and middle are then known to a fraction of a millimetre, wherever the package has
        shifted to since it was last seen or felt. The tool moves between the points slowly, the
        latch is asked to close only while the tool stands still, and is opened again in the
        next step."""
        st = self.s
        f = st["feeling"]
        u = st["bar_dir"]
        v = np.array([-u[1], u[0], 0.0])
        sign = 1.0 if s0 >= 0 else -1.0
        if f is None:
            st["bar0"] = st["bar0"] + st["shift"] + st["adm"]
            st["shift"], st["adm"] = np.zeros(3), np.zeros(3)
            # the searches, in turn: (direction, where it surely closes, where it surely does not)
            f = st["feeling"] = dict(origin=self.bar_point(s0), found=[], phase=0, wait=0, at=None,
                                     todo=[("v", 0.0, 0.0105), ("v", 0.0, -0.0105)], lo=None, hi=None)
        if f["lo"] is None:
            f["dir"], f["lo"], f["hi"] = f["todo"][0]
            f["first"] = True
        axis = v if f["dir"] == "v" else u
        if f["phase"] == 0:                       # go to the next point and stand still
            f["at"] = f["lo"] if f["first"] else 0.5 * (f["lo"] + f["hi"])
            # slowly: a latch that closes on a moving tool jerks the package it closes on
            step = f["origin"] + axis * f["at"] - st["p"]
            arrived = float(np.linalg.norm(step)) <= FEEL_SPEED * DT
            if not arrived:
                step = step * (FEEL_SPEED * DT / float(np.linalg.norm(step)))
            st["p"] = st["p"] + step
            f["wait"] += 1
            still = len(st["tips"]) == 3 and np.linalg.norm(st["tips"][-1] - st["tips"][-2]) < 3e-5
            if arrived and f["wait"] >= 4 and ((still and (st["err"] < 0.0004 or f["wait"] >= 14)) or f["wait"] >= 250):
                f["phase"], f["wait"] = 1, 0
                st["latch"] = 1
        elif f["phase"] == 1:                     # did it close? open again at once
            closed = bool(obs["latched"])
            where = float((np.median(st["tips"], axis=0) - f["origin"]) @ axis)
            st["latch"] = 0
            f["phase"] = 2
            if f["first"]:
                f["first"] = False
                if not closed and f["dir"] == "v" and not f["found"] and f.get("moved", 0) < 2:
                    # not here: the handle may lie further along than it was read; try nearer its middle
                    f["moved"] = f.get("moved", 0) + 1
                    f["origin"] = f["origin"] - u * sign * 0.025
                    f["first"] = True
                    return
                if not closed:                    # not where it was thought to be: give up
                    st["fail"] = "feel"
                    st["feeling"] = None
                    st["queue"] = [("rise", 0.10), ("goto", kin().fk(TRAVEL_Q)[0], 0.45, "abs")]
                    return
                f["lo"] = where
            elif closed:
                f["lo"] = where
            else:
                f["hi"] = where
        else:
            f["phase"] = 0
            if not f["first"] and abs(f["hi"] - f["lo"]) < 0.0007:
                f["found"].append(0.5 * (f["lo"] + f["hi"]))
                f["todo"].pop(0)
                f["lo"] = None
                if len(f["found"]) == 2:          # both sides across: the axis is midway
                    st["bar0"] = st["bar0"] + v * 0.5 * (f["found"][0] + f["found"][1]) - u * sign * 0.025 * f.get("moved", 0)
                    f["origin"] = self.bar_point(s0)
                    f["todo"] = [("u", 0.0, sign * (BAR_HALF + 0.035) - s0)]
                    f["reach"] = 0.5 * abs(f["found"][0] - f["found"][1])
                elif len(f["found"]) == 3:        # the latch's reach beyond the handle's end
                    st["bar0"] = st["bar0"] + u * (s0 + f["found"][2] - sign * (BAR_HALF + f["reach"]))
                    st["feeling"] = None
                    st["queue"].pop(0)

    def carry(self, target, tilt_limit):
        """A move of the hanging package: the package leans by atan(a / g) under an
        acceleration a, and a three-impulse shaper at its swing period keeps it from
        swinging beyond that."""
        dist = float(np.linalg.norm(target - self.here()))
        a_max = G * math.tan(math.radians(TILT_SHARE * tilt_limit))
        T = max(CARRY_MIN_S, math.sqrt(5.0 * dist / a_max))
        self.go(target, T=T, relative=False, shaper=ZVD)


# ==========================================================================================
# the mission
# ==========================================================================================
class Mission(object):
    def __init__(self):
        self.arm = Arm()
        self.drive = None
        self.map = None
        self.phase = "tare"
        self.sub = ""
        self.pulls = []          # drawer pulls seen: dict(mid, out, n, state)
        self.skip = []           # edges of the seen floor that could not be cleared
        self.goal = None
        self.target = None
        self.fine = None         # what the next picture is to be read for, and its answer
        self.pictures = 0
        self.wait = 0
        self.look = 1
        self.log = []
        self.package = None
        self.scans = 0
        self.lost = []
        self.spotted = None      # the package being gone for: one of `spots`
        self.spots = []          # packages seen standing in the open: dict(mid, n)
        self.delivered = 0       # how many are in their vessels; the next goes into vessel number `delivered`
        self.bad_sides = []      # headings from which its handle could not be read
        self.given_up = []
        self.tried = {}
        self.stage = "collect"   # collect (onto the trolley), then tow, then unload (into the vessels)
        self.trolley = None      # dict(xy, yaw, handle) as last seen
        self.loaded = []         # on the tray: dict(slot, cls)
        self.vessel_cls = {}     # vessel number -> class, from the plates
        self.vessel_target = None
        self.load = None
        self.retries = {}        # handles the latch missed, by where they are

    def say(self, t, text):
        self.log.append((t, text))
        if os.environ.get("HAZMAT_TALK"):
            print("[{:7.2f}] {}".format(t, text), file=sys.stderr, flush=True)

    # -- pictures -----------------------------------------------------------------------------
    def see(self, pic, base, t):
        view = View(pic, base)
        self.pictures += 1
        if self.phase in ("search", "explore", "to_drawer", "tidy") and not self.arm.busy():
            self.map.add(view)
            self.find_pulls(view)
        if self.phase in ("search", "explore", "to_drawer"):
            # an orange handle in any picture, also in one taken while the arm was at work;
            # but not the one in the drawer being looked into, which has its own reading
            mine = self.target["mid"] if self.phase == "to_drawer" and self.target is not None else None
            self.find_package(view, apart_from=mine)
        if self.phase in ("carry", "place") and self.package is not None and self.arm.s["latch"]:
            self.read_hanging(view, base)
        if self.fine and self.fine.get("want") and self.fine.get("after", 0) <= self.pictures:
            want = self.fine["want"]
            if want == "pull":
                self.fine = dict(answer=self.read_pull(view, self.target["mid"]))
            elif want == "pull_open":
                self.fine = dict(answer=self.read_pull(view, self.fine["near"]))
            elif want == "inside":
                self.map.add(view)
                self.fine = dict(answer=self.read_inside(view, self.fine["near"]))
            elif want == "handle":
                self.fine = dict(answer=self.read_inside(view, self.fine["near"], reach=0.30, band=0.15))
            elif want == "tray_handle":
                self.fine = dict(answer=self.read_inside(view, self.fine["near"], reach=0.12, band=0.06))
            elif want == "trolley":
                self.fine = dict(answer=self.read_trolley(view))
            elif want == "plates":
                self.fine = dict(answer=self.read_plates(view, self.fine["vessels"]))

    def find_pulls(self, view):
        ok = view.brass & (view.P[:, :, 2] > 0.55) & (view.P[:, :, 2] < 0.80)
        if ok.sum() < 3:
            return
        lab, n = ndimage.label(ok, structure=S8)
        for k in range(1, n + 1):
            m = lab == k
            if m.sum() < 3:
                continue
            c = view.P[m].mean(axis=0)
            if any(np.linalg.norm(c[:2] - np.asarray(o)[:2]) < 0.30 for p in self.pulls for o in p["places"]):
                # one seen before: a nearer look is a better one
                for p in self.pulls:
                    if np.linalg.norm(c[:2] - p["mid"][:2]) < 0.30 and p["state"] == "new" and m.sum() > p["n"]:
                        got = self.read_pull(view, c, mask=m)
                        if got:
                            p.update(mid=got["mid"], out=got["out"], n=int(m.sum()), places=[got["mid"]])
                continue
            if sum(1 for q in self.lost if np.linalg.norm(c[:2] - q) < 0.3) >= 2:
                continue
            got = self.read_pull(view, c, mask=m)
            if got:
                self.pulls.append(dict(mid=got["mid"], out=got["out"], n=int(m.sum()), state="new",
                                       places=[got["mid"]]))

    def read_pull(self, view, near, mask=None):
        """A drawer pull's middle, its direction and the way the drawer comes out (towards
        the camera's side of the pull)."""
        if mask is None:
            d = np.linalg.norm(view.P[:, :, :2] - np.asarray(near)[:2], axis=2)
            mask = view.brass & (d < 0.25) & (np.abs(view.P[:, :, 2] - near[2]) < 0.08)
        bar = bar_from(view, mask, 0.006)
        if bar is None:
            return None
        u = bar["dir"]
        out = np.array([-u[1], u[0], 0.0])
        # the drawer's front stands 4 cm behind the pull: the side on which things are seen
        # at the pull's height is the cabinet's
        rel = view.P - bar["mid"]
        side = rel[:, :, :2] @ out[:2]
        behind = view.valid & ~mask & (np.abs(rel[:, :, 2]) < 0.05) & (np.abs(rel[:, :, :2] @ u[:2]) < 0.10) \
            & (np.abs(side) > 0.015) & (np.abs(side) < 0.09)
        if behind.sum() >= 4:
            if np.median(side[behind]) > 0:
                out = -out
        elif out @ (view.cam - bar["mid"]) < 0:
            out = -out
        return dict(mid=bar["mid"], dir=u, out=out, length=bar["length"], n=bar["n"])

    def read_hanging(self, view, base):
        """The hanging package turns about the latch as it likes: follow its handle's
        direction from picture to picture (a handle looks the same turned half round, so the
        direction nearest the last one is taken)."""
        hook = arm_to_world(base, self.arm.here())
        d = np.linalg.norm(view.P[:, :, :2] - hook[:2], axis=2)
        mask = view.orange & (d < 0.11) & (np.abs(view.P[:, :, 2] - hook[2]) < 0.05)
        if mask.sum() < 10:
            return
        pts = view.P[mask][:, :2]
        c = pts - pts.mean(axis=0)
        w, vec = np.linalg.eigh(c.T @ c)
        if w[1] < 6.0 * w[0]:
            return
        u = np.array([vec[0, 1], vec[1, 1], 0.0])
        if u @ self.package["dir"] < 0:
            u = -u
        self.package["dir"] = u
        self.package["seen"] = self.package.get("seen", 0) + 1

    def find_package(self, view, apart_from=None):
        """An orange handle in plain sight: the package stands in the open."""
        ok = view.orange & (view.P[:, :, 2] > 0.06) & (view.P[:, :, 2] < 1.05)
        if ok.sum() < 4:
            return
        lab, n = ndimage.label(ok, structure=S8)
        for k in range(1, n + 1):
            m = lab == k
            if m.sum() < 4:
                continue
            c = view.P[m].mean(axis=0)
            if c[0] < 0.15 or any(np.linalg.norm(c[:2] - q) < 0.3 for q in self.given_up):
                continue                          # outside the lab: one already in its vessel
            if apart_from is not None and np.linalg.norm(c[:2] - np.asarray(apart_from)[:2]) < 0.75:
                continue
            if self.package is not None and np.linalg.norm(c[:2] - self.package["xy"]) < 0.6:
                continue                          # the one in hand
            old = [q for q in self.spots if np.linalg.norm(c[:2] - q["mid"][:2]) < 0.45]
            if not old:
                self.spots.append(dict(mid=c, n=int(m.sum())))
            elif m.sum() > old[0]["n"]:
                old[0].update(mid=c, n=int(m.sum()))

    def read_inside(self, view, near, reach=0.55, band=0.12):
        """Is the package's handle here (in the drawer opened here)? Its middle, direction,
        height."""
        d = np.linalg.norm(view.P[:, :, :2] - np.asarray(near)[:2], axis=2)
        mask = view.orange & (d < reach) & (np.abs(view.P[:, :, 2] - near[2]) < band)
        if mask.sum() < 8:
            return dict(found=False, n=int(mask.sum()))
        rows, cols = np.nonzero(mask)
        if rows.min() < 2 or cols.min() < 2 or rows.max() > CAM_H - 3 or cols.max() > CAM_W - 3:
            return dict(found=False, n=int(mask.sum()), cut=True)       # runs out of the picture
        bar = bar_from(view, mask, BAR_RADIUS)
        # seen whole, its length comes out a pixel or two short at either end
        pixel = float(np.linalg.norm(bar["mid"] - view.cam)) * math.tan(math.radians(CAM_FOVY) / 2.0) / (CAM_H / 2.0)
        if bar["length"] < 2 * BAR_HALF - 0.012 - 2.5 * pixel:
            return dict(found=False, n=int(mask.sum()), cut=True)       # part of it is hidden
        # the handle's height from the case's top, a flat face 4 cm under the handle's axis:
        # the points beside the handle, within the case's outline
        rel = view.P[:, :, :2] - bar["mid"][:2]
        along = rel @ bar["dir"][:2]
        across = rel @ np.array([-bar["dir"][1], bar["dir"][0]])
        top = view.valid & ~mask & (np.abs(along) < 0.06) & (np.abs(across) > 0.018) & (np.abs(across) < 0.045) \
            & (np.abs(view.P[:, :, 2] - (bar["mid"][2] - 0.04)) < 0.015)
        z = float(np.median(view.P[top][:, 2])) + (BAR_Z - BOX_HALF_Z) if top.sum() >= 10 else float(bar["mid"][2])
        mid = np.array([bar["mid"][0], bar["mid"][1], z])
        # the class label on the case's top, beside the handle
        label = top & (np.abs(along) < 0.045) & (np.abs(across) < 0.032)
        cls = classify(view.rgb[label])
        return dict(found=True, mid=mid, dir=bar["dir"], n=bar["n"], length=bar["length"], top=int(top.sum()),
                    cls=cls)

    # -- one control step ---------------------------------------------------------------------
    def act(self, t, obs):
        base = np.array(obs["base"], float)
        if self.drive is None:
            self.drive = Drive(base)
            self.map = Map(obs["vessels"])
        if obs.get("camera"):
            self.see(obs["camera"], base, obs["t_s"])
        getattr(self, "do_" + self.phase)(obs, base)
        q = self.arm.act(obs)
        v = self.drive.act(obs)
        # a picture when one is waited for; otherwise two or three a second while searching
        waiting = bool(self.fine and self.fine.get("want"))
        look = self.look and (waiting or t % 20 == 0 or self.phase == "tare") and self.phase not in ("place", "lid")
        return {"base": v, "q": q, "latch": int(self.arm.s["latch"]), "look": int(look)}

    def idle(self):
        return not self.arm.busy() and not self.drive.busy()

    def do_tare(self, obs, base):
        if "bias" in self.arm.s:
            self.phase, self.sub = "trolley", "turn"
            self.say(obs["t_s"], "tared; looking for the trolley")

    def do_trolley(self, obs, base):
        """Turn round, find the trolley in the yard, turn back to the lab."""
        if not self.idle():
            return
        if self.sub == "turn":
            self.drive.to(yaw=math.pi)
            self.sub = "look"
        elif self.sub == "look":
            self.fine = dict(want="trolley", after=self.pictures + 2)
            self.sub = "read"
        elif self.sub == "read":
            if self.fine.get("want"):
                return
            got = self.fine["answer"]
            if got is None:
                self.say(obs["t_s"], "no trolley in sight")
            else:
                self.trolley = got
                self.say(obs["t_s"], "the trolley at {}, heading {:.2f}".format(np.round(got["xy"], 2), got["yaw"]))
            self.drive.to(yaw=0.0)
            self.phase, self.sub = "search", ""

    # -- searching ----------------------------------------------------------------------------
    def do_search(self, obs, base):
        """Decide where to go next: a drawer not opened yet, else an edge of the seen floor."""
        if not self.idle():
            return
        if self.wait < 12:                          # a moment for a picture from standing still
            self.wait += 1
            return
        self.wait = 0
        room = self.map.room(CART_R)
        here = Map.nearest(room, base, 0.5)
        if here is None:
            self.say(obs["t_s"], "nowhere to stand: staying")
            return
        dist = Map.wave(room, here)
        # 0. the package itself, if it stands in sight: a place to stand before it
        for spot in sorted(self.spots, key=lambda q: float(np.linalg.norm(q["mid"][:2] - base[:2]))):
            self.spotted = spot
            plan = self.plan_stand(self.spotted["mid"], room, dist)
            if plan is not None:
                pre, yaw, c, reach = plan
                way = Map.path(room, dist, c)
                self.target = dict(mid=self.spotted["mid"].copy(), reach=reach,
                                   out=np.array([-math.cos(yaw), -math.sin(yaw), 0.0]))
                self.follow(way[1:-1] + [pre], base, final=yaw)
                self.phase, self.sub = "to_package", "drive"
                self.say(obs["t_s"], "the package stands in the open at {}: to it, facing {:.2f}".format(
                    np.round(self.spotted["mid"], 2), yaw))
                return
            self.say(obs["t_s"], "the package is in sight at {} but there is no place to stand yet".format(
                np.round(self.spotted["mid"], 2)))
        # 1. the nearest drawer not opened
        best = None
        for p in self.pulls:
            if p["state"] != "new":
                continue
            for back in (0.50, 0.65, 0.35, 0.80):
                pre = p["mid"][:2] + p["out"][:2] * (PULL_REACH + ARM_MOUNT[0] + back)
                c = Map.nearest(room, pre, 0.12)
                if c is not None and np.isfinite(dist[c]):
                    if best is None or dist[c] < best[0]:
                        best = (dist[c], p, pre, c)
                    break
        if best is not None:
            _, p, pre, c = best
            way = Map.path(room, dist, c)
            self.target = p
            self.follow(way[1:-1] + [pre], base, final=math.atan2(-p["out"][1], -p["out"][0]))
            self.phase, self.sub = "to_drawer", "drive"
            self.say(obs["t_s"], "to the drawer at {} (out {})".format(np.round(p["mid"], 2), np.round(p["out"][:2], 2)))
            return
        # 2. the nearest edge of the seen floor inside the lab
        free = self.map.free()
        unknown = ~self.map.floor & ~self.map.obst
        edge = free & ndimage.binary_dilation(unknown, structure=S8) & self.map.inside
        clear = ~self.map.obst
        lab, n = ndimage.label(edge, structure=S8)
        best = None
        for k in range(1, n + 1):
            m = lab == k
            if m.sum() < 4:
                continue
            ii, jj = np.nonzero(m)
            mid = centre(ii.mean(), jj.mean()) - 0.5 * GRID
            key = (int(round(mid[0] / 0.4)), int(round(mid[1] / 0.4)))
            if any(np.linalg.norm(mid - s) < 0.3 for s in self.skip) or self.tried.get(key, 0) >= 2:
                continue
            # a place to look at it from: 0.9 to 2.2 m away, in sight
            d = np.hypot(self.map.X - mid[0], self.map.Y - mid[1])
            ring = room & np.isfinite(dist) & (d > 0.9) & (d < 2.2)
            if not ring.any():
                continue
            cost = np.where(ring, dist + 0.5 * d, np.inf)
            c = None
            for flat in np.argsort(cost, axis=None)[:40]:           # the nearest one that sees it
                u = np.unravel_index(flat, cost.shape)
                if not np.isfinite(cost[u]):
                    break
                if Map.sight(clear, centre(*u), mid):
                    c = u
                    break
            if c is None:
                continue
            if best is None or cost[c] < best[0]:
                best = (float(cost[c]), c, mid)
        if best is not None:
            _, c, mid = best
            way = Map.path(room, dist, c)
            end = way[-1]
            self.follow(way[1:], base, final=math.atan2(mid[1] - end[1], mid[0] - end[0]))
            self.goal = mid
            key = (int(round(mid[0] / 0.4)), int(round(mid[1] / 0.4)))
            self.tried[key] = self.tried.get(key, 0) + 1
            self.phase = "explore"
            self.say(obs["t_s"], "to look at {} from {}".format(np.round(mid, 2), np.round(end, 2)))
            return
        # 3. things seen only from afar: a pull on them would not have shown; go nearer
        far = self.map.obst & (self.map.near > LOOK_RANGE) & (self.map.X > -0.05) & (self.map.X < ROOM_X + 0.05)
        lab, n = ndimage.label(far, structure=S8)
        best = None
        for k in range(1, n + 1):
            m = lab == k
            if m.sum() < 3:
                continue
            ii, jj = np.nonzero(m)
            mid = centre(ii.mean(), jj.mean()) - 0.5 * GRID
            if any(np.linalg.norm(mid - q) < 0.3 for q in self.skip):
                continue
            d = np.hypot(self.map.X - mid[0], self.map.Y - mid[1])
            ring = room & np.isfinite(dist) & (d > 1.0) & (d < LOOK_RANGE - 0.6)
            cost = np.where(ring, dist + 0.3 * d, np.inf)
            for flat in np.argsort(cost, axis=None)[:40]:
                u = np.unravel_index(flat, cost.shape)
                if not np.isfinite(cost[u]):
                    break
                at = centre(*u)
                if Map.sight(clear, at, at + (mid - at) * (1.0 - 0.25 / max(0.3, float(d[u])))):
                    if best is None or cost[u] < best[0]:
                        best = (float(cost[u]), u, mid)
                    break
        if best is not None:
            _, c, mid = best
            way = Map.path(room, dist, c)
            end = way[-1]
            self.follow(way[1:], base, final=math.atan2(mid[1] - end[1], mid[0] - end[0]))
            self.skip.append(mid)                    # one visit each
            self.goal = None
            self.phase = "explore"
            self.say(obs["t_s"], "nearer to {} from {}".format(np.round(mid, 2), np.round(end, 2)))
            return
        # 4. things in the room seen from one side only: something may stand behind them
        X, Y = self.map.X, self.map.Y
        middle = (X > 0.7) & (X < ROOM_X - 0.7) & (np.abs(Y) < ROOM_Y / 2.0 - 0.7)
        sides = self.map.sides
        both = (((sides & 1) > 0) & ((sides & 4) > 0)) | (((sides & 2) > 0) & ((sides & 8) > 0))
        lab, n = ndimage.label(self.map.obst & middle & ~both, structure=S8)
        best = None
        for k in range(1, n + 1):
            m = lab == k
            if m.sum() < 4:
                continue
            ii, jj = np.nonzero(m)
            mid = centre(ii.mean(), jj.mean()) - 0.5 * GRID
            seen = [int(((sides[m] >> q) & 1).sum()) for q in range(4)]
            want = (int(np.argmax(seen)) + 2) % 4                       # the side opposite the best seen
            key = (int(round(mid[0] / 0.4)), int(round(mid[1] / 0.4)), want)
            if self.tried.get(key, 0) >= 1:
                continue
            d = np.hypot(X - mid[0], Y - mid[1])
            quarter = np.floor((np.arctan2(mid[1] - Y, mid[0] - X) + math.pi) / (math.pi / 2.0)).astype(int) % 4
            ring = room & np.isfinite(dist) & (d > 0.9) & (d < 2.2) & (quarter == want)
            if not ring.any():
                continue
            cost = np.where(ring, dist + 0.3 * d, np.inf)
            c = np.unravel_index(np.argmin(cost), cost.shape)
            if best is None or cost[c] < best[0]:
                best = (float(cost[c]), c, mid, key)
        if best is not None:
            _, c, mid, key = best
            way = Map.path(room, dist, c)
            end = way[-1]
            self.follow(way[1:], base, final=math.atan2(mid[1] - end[1], mid[0] - end[0]))
            self.tried[key] = 1
            self.goal = None
            self.phase = "explore"
            self.say(obs["t_s"], "round to the other side of {} from {}".format(np.round(mid, 2), np.round(end, 2)))
            return
        # 5. nothing in sight: look around
        if self.scans < 4:
            self.scans += 1
            self.drive.to(yaw=base[2] + math.pi / 2)
            self.phase = "explore"
            self.goal = None
            return
        self.say(obs["t_s"], "nothing left to look at")
        self.phase = "stay"

    def box_clear(self, centre_, yaw, margin, skip=None):
        """Is the cart's box, standing at a pose, clear of everything the map shows (but for
        what is within 14 cm of `skip`: the package itself)?"""
        xs = np.arange(-0.34 - margin, 0.36 + margin + 1e-9, 0.04)
        ys = np.arange(-0.25 - margin, 0.25 + margin + 1e-9, 0.04)
        X, Y = np.meshgrid(xs, ys)
        c, s = math.cos(yaw), math.sin(yaw)
        wx = centre_[0] + c * X - s * Y
        wy = centre_[1] + s * X + c * Y
        i = ((wx - MAP_X0) / GRID).astype(int)
        j = ((wy - MAP_Y0) / GRID).astype(int)
        if i.min() < 0 or j.min() < 0 or i.max() >= MAP_NX or j.max() >= MAP_NY:
            return False
        hit = self.map.obst[i, j]
        if skip is not None:
            hit = hit & (np.hypot(wx - skip[0], wy - skip[1]) > 0.14)
        return not hit.any()

    def plan_stand(self, mid, room, dist):
        """Where to stand to reach a package standing on a piece of furniture: facing it,
        the cart's box clear of the furniture by a margin, with a clear run-up; of all such
        places the one nearest by the way there. (pre-approach point, heading, its cell, reach)"""
        best = None
        for k in range(24):
            yaw = k * math.pi / 12.0
            if any(abs(wrap(yaw - bad)) < 0.6 for bad in self.bad_sides):
                continue
            u = np.array([math.cos(yaw), math.sin(yaw)])
            # a thing on the floor is in the camera's view only from farther back
            for reach in ((0.56, 0.60) if mid[2] < 0.35 else (0.46, 0.50) if mid[2] > 0.75 else (0.46, 0.52, 0.58)):
                stand = mid[:2] - u * (ARM_MOUNT[0] + reach)
                if not self.box_clear(stand, yaw, 0.07, skip=mid):
                    continue
                pre = stand - u * 0.5
                c = Map.nearest(room, pre, 0.12)
                if c is None or not np.isfinite(dist[c]) or not self.box_clear(stand - u * 0.25, yaw, 0.05, skip=mid):
                    break
                cost = float(dist[c]) + 2.0 * abs(reach - 0.5)
                if best is None or cost < best[0]:
                    best = (cost, pre, yaw, c, reach)
                break
        return None if best is None else best[1:]

    def do_to_package(self, obs, base):
        """To a package standing in the open: read its handle from the run-up, stand before
        it, read it again close by, square up, then weigh and lift it."""
        if not self.idle():
            return
        t = obs["t_s"]
        p = self.target
        if self.sub in ("drive", "advance"):
            self.fine = dict(want="handle", near=p["mid"], after=self.pictures + 2)
            self.sub = "read_far" if self.sub == "drive" else "read_near"
            return
        if self.fine.get("want"):
            return
        got = self.fine["answer"]
        if not got["found"] and self.sub == "read_near" and p.get("last") is not None:
            got = p["last"]                      # out of the camera's view close by: the last reading stands
        if not got["found"]:
            # not readable from here: try it from another side, three times at most
            self.say(t, "the handle cannot be read from here ({} pixels)".format(got.get("n", 0)))
            self.bad_sides.append(math.atan2(-p["out"][1], -p["out"][0]))
            if len(self.bad_sides) >= 3 or got.get("n", 0) == 0:
                self.given_up.append(p["mid"][:2].copy())
                self.spots = [q for q in self.spots if np.linalg.norm(q["mid"][:2] - p["mid"][:2]) > 0.45]
                self.spotted = None
                self.bad_sides = []
            self.retreat(base)
            return
        p["mid"] = got["mid"]
        p["last"] = got
        to_it = got["mid"][:2] - base[:2]
        yaw = math.atan2(to_it[1], to_it[0])
        u = np.array([math.cos(yaw), math.sin(yaw)])
        stand = got["mid"][:2] - u * (ARM_MOUNT[0] + p["reach"])
        mid = world_to_arm(base, got["mid"])
        if self.sub == "read_far" or ((abs(mid[0] - p["reach"]) > 0.03 or abs(mid[1]) > 0.03) and p.get("squared", 0) < 3):
            p["squared"] = p.get("squared", 0) + (self.sub == "read_near")
            if not self.box_clear(stand, yaw, 0.04, skip=got["mid"]):
                p["reach"] = min(0.50 if got["mid"][2] > 0.75 else 0.60, p["reach"] + 0.04)
                stand = got["mid"][:2] - u * (ARM_MOUNT[0] + p["reach"])
            self.drive.to(xy=stand, yaw=yaw, limits=(0.2, 0.3, 0.5, 1.0))
            self.sub = "advance"
            return
        if not self.drive.arrived:
            self.sub = "advance"
            return
        p["out"] = np.array([-u[0], -u[1], 0.0])
        self.say(t, "the package: handle at {} in the arm's frame, {} pixels, length {:.3f}".format(
            np.round(mid, 3), got["n"], got["length"]))
        self.begin_pick(obs, base, got)

    def follow(self, way, base, final=None):
        """Drive along way points, facing each stretch; turn to `final` at the end."""
        at = np.array(base[:2])
        for p in way:
            if np.linalg.norm(np.asarray(p) - at) < 0.03:
                continue
            self.drive.to(yaw=math.atan2(p[1] - at[1], p[0] - at[0]))
            self.drive.to(xy=p)
            at = np.asarray(p)
        if final is not None:
            self.drive.to(yaw=final)

    def do_explore(self, obs, base):
        if not self.idle():
            return
        if self.goal is not None:
            self.wait += 1
            if self.wait < 12:
                return
            self.wait = 0
            i, j = cell(self.goal)
            unknown = ~self.map.floor & ~self.map.obst
            if unknown[max(0, i - 3):i + 4, max(0, j - 3):j + 4].sum() > 6:
                self.skip.append(self.goal)          # looked, and it is still not seen: leave it
            self.goal = None
        self.phase = "search"

    def do_stay(self, obs, base):
        pass

    # -- a drawer -----------------------------------------------------------------------------
    def do_to_drawer(self, obs, base):
        p = self.target
        if not self.idle():
            return
        t = obs["t_s"]
        if self.sub == "drive":
            self.fine = dict(want="pull", after=self.pictures + 2)
            self.sub = "read_far"
        elif self.sub == "read_far":
            if self.fine.get("want"):
                return
            got = self.fine["answer"]
            if got is None:
                self.lose(p)
                self.phase = "search"
                self.say(t, "the pull is not where it was seen")
                return
            p.update(mid=got["mid"], out=got["out"])
            yaw = math.atan2(-got["out"][1], -got["out"][0])
            stand = got["mid"][:2] + got["out"][:2] * (PULL_REACH + ARM_MOUNT[0])
            self.drive.to(yaw=yaw)
            self.drive.to(xy=stand, limits=(0.25, 0.3, 0.6, 1.0))
            self.sub = "advance"
        elif self.sub == "advance":
            self.fine = dict(want="pull", after=self.pictures + 2)
            self.sub = "read_near"
        elif self.sub == "read_near":
            if self.fine.get("want"):
                return
            got = self.fine["answer"]
            if got is None:
                self.lose(p)
                self.retreat(base)
                self.say(t, "lost the pull close by")
                return
            p.update(mid=got["mid"], out=got["out"])
            mid = world_to_arm(base, got["mid"])
            out = rot(base[2]).T @ got["out"][:2]
            # stand square to the drawer and at the right reach before touching anything
            off = np.array([mid[0] - PULL_REACH, mid[1]])
            turn = wrap(math.atan2(-out[1], -out[0]))
            if p.get("squared", 0) < 3 and (abs(off[0]) > 0.02 or abs(off[1]) > 0.03 or abs(turn) > 0.02):
                p["squared"] = p.get("squared", 0) + 1
                yaw = base[2] + turn
                stand = got["mid"][:2] + got["out"][:2] * (PULL_REACH + ARM_MOUNT[0])
                self.drive.to(xy=stand, yaw=yaw, limits=(0.2, 0.3, 0.5, 1.0))
                self.sub = "advance"
                return
            if not self.drive.arrived:
                self.sub = "advance"
                return
            self.say(t, "pull at {} in the arm's frame, length {:.3f}".format(np.round(mid, 3), got["length"]))
            self.arm.open_drawer(mid, np.array([out[0], out[1], 0.0]))
            self.look = 0
            self.sub = "open"
        elif self.sub == "open":
            self.look = 1
            if self.arm.s["fail"]:
                self.say(t, "the latch did not close on the pull")
                p["tries"] = p.get("tries", 0) + 1
                if p["tries"] >= 2:
                    p["state"] = "failed"
                    self.retreat(base)
                else:
                    self.sub = "advance"
                return
            opened = p["mid"] + p["out"] * PULL_STROKE
            p["places"].append(opened)
            p["state"] = "open"
            p["mark"] = arm_to_world(base, self.arm.s["mark"])     # where the pull is now
            p["stand"] = base.copy()
            self.fine = dict(want="inside", near=opened + p["out"] * -0.2, after=self.pictures + 2)
            self.sub = "inside"
        elif self.sub == "inside":
            if self.fine.get("want"):
                return
            got = self.fine["answer"]
            if not got["found"]:
                self.say(t, "nothing in this drawer ({} orange pixels): shutting it".format(got["n"]))
                self.shut(base)
                self.sub = "shutting"
                return
            mid = world_to_arm(base, got["mid"])
            u = rot(base[2]).T @ got["dir"][:2]
            if mid[0] > 0.57 and not p.get("closer"):
                # nearer, so that it can be lifted clear of the drawer's front
                p["closer"] = True
                self.drive.to(xy=base[:2] - p["out"][:2] * min(0.12, mid[0] - 0.52), limits=(0.15, 0.2, 0.5, 1.0))
                self.sub = "inside_again"
                return
            self.say(t, "the package: handle at {} in the arm's frame, {} pixels, length {:.3f}".format(
                np.round(mid, 3), got["n"], got["length"]))
            if abs(mid[1]) > 0.10 and not p.get("sidestep"):
                # stand in line with it: the arm reaches best straight ahead
                p["sidestep"] = True
                side = rot(base[2]) @ np.array([0.0, mid[1]])
                self.drive.to(xy=base[:2] + side, limits=(0.15, 0.2, 0.5, 1.0))
                self.fine = dict(want="inside", near=self.fine_near(p), after=self.pictures + 2)
                self.wait = 0
                self.sub = "inside_again"
                return
            if not self.drive.arrived:
                self.sub = "inside_again"
                return
            self.begin_pick(obs, base, got)
        elif self.sub == "inside_again":
            self.fine = dict(want="inside", near=self.fine_near(p), after=self.pictures + 2)
            self.sub = "inside"
        elif self.sub == "shutting":
            self.look = 1
            p["state"] = "shut"
            self.map.clear((p["mid"] + p["out"] * 0.24)[:2], 0.21)       # the drawer is out of the way again
            self.retreat(base)

    def lose(self, p):
        self.pulls = [q for q in self.pulls if q is not p]
        self.lost.append(p["mid"][:2].copy())

    def fine_near(self, p):
        return p["mid"] + p["out"] * (PULL_STROKE - 0.2)

    def shut(self, base):
        """Push the drawer opened last back in, from where the cart stands."""
        p = self.target
        mark = world_to_arm(base, p["mark"])
        out = rot(base[2]).T @ p["out"][:2]
        travel = float((p["mark"] - p["mid"])[:2] @ p["out"][:2]) - 0.004
        self.arm.shut_drawer(mark, np.array([out[0], out[1], 0.0]), max(0.05, travel))
        self.look = 0

    def retreat(self, base):
        p = self.target
        back = np.asarray(base[:2]) + p["out"][:2] * 0.5
        self.drive.to(xy=back)
        self.phase = "search"

    # -- the package --------------------------------------------------------------------------
    def begin_pick(self, obs, base, got):
        mid = world_to_arm(base, got["mid"])
        u = rot(base[2]).T @ got["dir"][:2]
        self.package = dict(dir=got["dir"].copy(), xy=got["mid"][:2].copy(), drawer=self.phase == "to_drawer",
                            cls=got.get("cls"), z=float(got["mid"][2]))
        self.spots = [q for q in self.spots if np.linalg.norm(q["mid"][:2] - got["mid"][:2]) > 0.5]
        self.spotted = None
        # out of a drawer the case has to clear the drawer's front, 6 cm above the handle
        up = got["mid"][2] + (0.24 if self.phase == "to_drawer" else 0.10)
        self.arm.pick(mid, np.array([u[0], u[1], 0.0]), max(CARRY_Z, up) - ARM_MOUNT[2])
        self.look = 0
        self.phase = "pick"

    def do_pick(self, obs, base):
        if self.arm.busy():
            return
        t = obs["t_s"]
        if self.arm.s["fail"]:
            # the handle was read wrong: go back for it and read it again, twice at most
            key = tuple(np.round(self.package["xy"], 1))
            self.retries[key] = self.retries.get(key, 0) + 1
            self.say(t, "the latch did not close on the handle (try {})".format(self.retries[key]))
            if self.retries[key] <= 2 and self.stage == "unload":
                self.unloading["done"] = False
                self.package = None
                self.look = 1
                self.phase, self.sub = "unload", "next"
                return
            if self.retries[key] <= 2 and not self.package["drawer"]:
                self.spots.append(dict(mid=np.array([self.package["xy"][0], self.package["xy"][1], self.package["z"]]), n=1))
                self.package = None
                self.look = 1
                self.phase = "search"
                return
            self.phase = "stay"
            return
        self.say(t, "lifted: weight {:.2f} N, centre of mass {:+.1f} mm along the handle".format(
            self.arm.s["weight"], 1000 * self.arm.s["xc"]))
        self.look = 1
        if self.stage == "unload":
            self.phase = "carry" if self.plan_to_vessel(obs, base) else "stay"
            return
        self.map.clear(self.package["xy"], 0.2)
        self.phase = "carry" if self.plan_way_out(obs, base) else "stay"

    def carry_limits(self, obs):
        a = min(0.30, G * math.tan(math.radians(TILT_SHARE * obs["tilt_limit_deg"])))
        return (0.30, a, 0.35, 0.5)

    def plan_way_out(self, obs, base):
        """Out of the lab with the package hanging ahead of the cart. The cart may slide
        any way at a fixed heading, wherever both it and the package clear what the map
        shows, and turn where the package's arc is clear: the shortest such way is found on a
        graph of (cell, heading), with the heading it stands at and twelve more."""
        lim = self.carry_limits(obs)
        t = obs["t_s"]
        p = self.target
        outside = np.array([-1.0, 0.0])                          # before the door, on open ground
        self.load = self.tray_place(len(self.loaded))
        back = np.asarray(base[:2]) + p["out"][:2] * 0.55       # the way it came to the drawer
        heads = [float(base[2])] + [k * math.pi / 6.0 for k in range(12)]
        N = MAP_NX * MAP_NY
        idx = np.arange(N).reshape(MAP_NX, MAP_NY)
        way = None
        for slack in (0.0, 0.04, 0.08):
            cart_ok = self.map.room(CART_R - slack)
            pack_ok = self.map.room(PACK_R - slack)

            def ahead(yaw):
                return shifted(pack_ok, PACK_AHEAD * np.array([math.cos(yaw), math.sin(yaw)]))

            masks = [cart_ok & ahead(h) for h in heads]
            c0 = Map.nearest(masks[0], back, 0.3)
            start = back
            if c0 is None:
                c0, start = Map.nearest(masks[0], base[:2], 0.3), None
            c1 = Map.nearest(cart_ok, outside, 0.4)
            if c0 is None or c1 is None:
                continue
            rows, cols, cost = [], [], []
            for L, m in enumerate(masks):
                for di, dj, step in ((1, 0, 1.0), (0, 1, 1.0), (1, 1, 1.4142), (1, -1, 1.4142)):
                    a = m[:MAP_NX - di, max(0, -dj):MAP_NY - max(0, dj)]
                    b = m[di:, max(0, dj):MAP_NY + min(0, dj)]
                    both = a & b
                    rows.append(L * N + idx[:MAP_NX - di, max(0, -dj):MAP_NY - max(0, dj)][both])
                    cols.append(L * N + idx[di:, max(0, dj):MAP_NY + min(0, dj)][both])
                    cost.append(np.full(int(both.sum()), step * GRID))
            turns = [(1 + k, 1 + (k + 1) % 12) for k in range(12)]
            k0 = int(math.floor((heads[0] % (2 * math.pi)) / (math.pi / 6.0))) % 12
            turns += [(0, 1 + k0), (0, 1 + (k0 + 1) % 12)]
            for a, b in turns:
                gap = wrap(heads[b] - heads[a])
                ok = masks[a] & masks[b] & ahead(heads[a] + 0.5 * gap)
                rows.append(a * N + idx[ok])
                cols.append(b * N + idx[ok])
                cost.append(np.full(int(ok.sum()), PACK_AHEAD * abs(gap) + 0.25))
            graph = coo_matrix((np.concatenate(cost), (np.concatenate(rows), np.concatenate(cols))),
                               shape=(len(heads) * N, len(heads) * N)).tocsr()
            dist, before = dijkstra(graph, directed=False, indices=int(idx[c0]), return_predecessors=True)
            ends = [L * N + int(idx[c1]) for L in range(len(heads)) if masks[L][c1]]
            if not ends or not np.isfinite(min(dist[e] for e in ends)):
                continue
            node = min(ends, key=lambda e: dist[e])
            way = []
            while node >= 0:
                way.append((node // N, (node % N) // MAP_NY, (node % N) % MAP_NY))
                node = int(before[node])
            way.reverse()
            break
        if way is None:
            self.say(t, "no way out found: staying")
            return False
        if start is not None:
            self.drive.to(xy=start, limits=lim, shaper=ZVD)
        # stretches at one heading, straightened; a turn where the heading changes
        k, turned = 0, []
        while k < len(way):
            L = way[k][0]
            n = k
            while n + 1 < len(way) and way[n + 1][0] == L:
                n += 1
            pts = [centre(i, j) for _, i, j in way[k:n + 1]]
            a = 0
            while a < len(pts) - 1:
                b = len(pts) - 1
                while b > a + 1 and not Map.sight(masks[L], pts[a], pts[b]):
                    b -= 1
                self.drive.to(xy=pts[b], limits=lim, shaper=ZVD)
                a = b
            if n + 1 < len(way):
                self.drive.to(yaw=heads[way[n + 1][0]], limits=lim, shaper=ZVD)
                turned.append(np.round(pts[-1], 1).tolist())
            k = n + 1
        self.drive.to(xy=outside, limits=lim, shaper=ZVD)
        self.drive.to(yaw=math.pi, limits=lim, shaper=ZVD)
        ld = self.load
        self.drive.to(xy=np.array([-1.7, ld["approach"][1]]), limits=lim, shaper=ZVD)
        self.drive.to(xy=ld["approach"], limits=lim, shaper=ZVD)
        self.drive.to(yaw=ld["face"], limits=lim, shaper=ZVD)
        self.say(t, "way out: {} stretches, turning at {}; to place {} on the tray".format(
            len(self.drive.todo), turned, ld["slot"]))
        return True

    def do_carry(self, obs, base):
        if not self.idle() or not self.drive.arrived:
            return
        self.wait += 1
        if self.wait < 45:                             # let the swing die, and take a last look
            return
        self.wait = 0
        here = self.arm.here()
        if here[2] > CARRY_Z - ARM_MOUNT[2] + 0.01 and not self.package.get("lowered"):
            # down to the height at which the vessel is within reach
            self.package["lowered"] = True
            self.arm.s["leak"] = 0.0
            self.arm.s["queue"] = [("carry", np.array([here[0], here[1], CARRY_Z - ARM_MOUNT[2]]), obs["tilt_limit_deg"])]
            return
        u = rot(base[2]).T @ self.package["dir"][:2]
        if self.stage == "collect":
            if not self.package.get("at_tray"):
                # the last stretch: over the tray's rim, to stand beside the place
                self.package["at_tray"] = True
                self.drive.to(xy=self.load["stand"], yaw=self.load["face"], limits=(0.12, 0.12, 0.3, 0.3), shaper=ZVD)
                return
            self.trolley_seen(obs, base)
            v = world_to_arm(base, np.array([self.load["xy"][0], self.load["xy"][1], TRAY_TOP - VESSEL_FLOOR]))
            self.arm.s["leak"] = 0.0
            self.arm.place(v, np.array([u[0], u[1], 0.0]), obs["tilt_limit_deg"])
            self.look = 0
            self.say(obs["t_s"], "at the trolley: place {} at {} in the arm's frame".format(self.load["slot"], np.round(v, 3)))
            self.phase = "place"
            return
        vessel = np.asarray(self.vessel_target, float)
        v = world_to_arm(base, np.array([vessel[0], vessel[1], STAND_H]))
        self.arm.s["leak"] = 0.0
        self.arm.place(v, np.array([u[0], u[1], 0.0]), obs["tilt_limit_deg"])
        self.look = 0
        self.say(obs["t_s"], "at the vessel: {} in the arm's frame".format(np.round(v, 3)))
        self.phase = "place"

    def do_place(self, obs, base):
        if not self.idle():
            return
        if self.stage == "collect":
            self.loaded.append(dict(slot=self.load["slot"], cls=self.package.get("cls")))
            self.say(obs["t_s"], "on the tray: place {}, class {}".format(self.load["slot"], self.package.get("cls")))
            self.arm.s["queue"] = [("goto", kin().fk(TRAVEL_Q)[0], 0.45, "abs")]
            self.phase, self.sub = "leave", ""
            return
        vessel = np.asarray(self.vessel_target, float)
        hinge = world_to_arm(base, np.array([vessel[0] + 0.137, vessel[1], STAND_H + VESSEL_HEIGHT + 0.004]))
        across = rot(base[2]).T @ np.array([-1.0, 0.0])
        self.arm.shut_lid(hinge, np.array([across[0], across[1], 0.0]))
        self.say(obs["t_s"], "released; closing the lid")
        self.phase = "lid"

    def do_lid(self, obs, base):
        if not self.idle():
            return
        t = obs["t_s"]
        if self.arm.s["fail"]:
            self.say(t, "the latch did not close on the lid's handle")
        self.arm.s["queue"] = [("goto", kin().fk(TRAVEL_Q)[0], 0.45, "abs")]
        self.delivered += 1
        if self.stage == "unload":
            self.say(t, "the lid is shut: {} of {} in their vessels".format(self.delivered, len(self.loaded)))
            self.package = None
            self.phase, self.sub = "unload", "next"
            return
        left = [p for p in self.pulls if p["state"] == "open"]
        if not left:
            self.say(t, "the lid is shut; nothing left open")
            self.next_one(obs)
            return
        self.say(t, "the lid is shut; back to shut {} drawer(s)".format(len(left)))
        self.look = 1
        self.phase, self.sub = "tidy", "plan"

    def next_one(self, obs):
        """One more is in its vessel: look for the next, or stop when all are."""
        self.package = None
        self.spotted = None
        self.bad_sides = []
        self.look = 1
        if len(self.loaded) >= int(obs["packages"]):
            self.say(obs["t_s"], "all {} are on the trolley: to tow it to the store".format(len(self.loaded)))
            self.stage = "tow"
            self.phase, self.sub = "hitch", "plan"
            return
        self.skip, self.tried, self.scans = [], {}, 0       # look everywhere again, if need be
        self.say(obs["t_s"], "{} of {} on the trolley: looking for the next".format(len(self.loaded), obs["packages"]))
        self.phase = "search"

    def do_tidy(self, obs, base):
        """Back to a drawer left open (the one the package came out of) to shut it."""
        if not self.idle():
            return
        t = obs["t_s"]
        left = [p for p in self.pulls if p["state"] == "open"]
        if not left:
            self.say(t, "everything is as it was")
            self.next_one(obs)
            return
        p = self.target = left[0]
        if self.sub == "plan":
            room = self.map.room(CART_R)
            here = Map.nearest(room, base, 0.5)
            dist = Map.wave(room, here) if here is not None else None
            stand = p["stand"]
            pre = stand[:2] + p["out"][:2] * 0.5
            c = None if dist is None else Map.nearest(room, pre, 0.2)
            if c is None or not np.isfinite(dist[c]):
                self.say(t, "no way back to the drawer")
                p["state"] = "left"
                return
            way = Map.path(room, dist, c)
            self.follow(way[1:-1] + [pre], base, final=stand[2])
            self.drive.to(xy=stand[:2], yaw=stand[2], limits=(0.25, 0.3, 0.6, 1.0))
            self.sub = "read"
        elif self.sub == "read":
            self.target["mid_keep"] = p["mid"]
            self.fine = dict(want="pull_open", near=p["mark"], after=self.pictures + 2)
            self.sub = "shut"
        elif self.sub == "shut":
            if self.fine.get("want") or not self.drive.arrived:
                return
            got = self.fine["answer"]
            if got is not None:
                p["mark"] = got["mid"]
            self.shut(base)
            self.sub = "done"
        elif self.sub == "done":
            self.look = 1
            p["state"] = "shut"
            self.sub = "plan"


    def do_leave(self, obs, base):
        """Away from the trolley once the arm is folded: straight back from it (square to its
        side), then along the lane to the lab's door (the yard is beyond the map); then shut
        what was left open, or look for the next package."""
        if not self.idle():
            return
        if self.sub == "":
            ld = self.load
            back = ld["approach"] + (ld["approach"] - ld["stand"]) * 0.25
            self.drive.to(xy=back, limits=(0.2, 0.2, 0.5, 1.0))
            self.drive.to(xy=np.array([-1.7, back[1]]))
            self.drive.to(xy=np.array([-1.0, 0.0]), yaw=0.0)
            self.sub = "go"
            return
        self.look = 1
        if any(p["state"] == "open" for p in self.pulls):
            self.phase, self.sub = "tidy", "plan"
        else:
            self.next_one(obs)

    # -- the trolley and the store ----------------------------------------------------------
    def trolley_pose(self):
        tr = self.trolley or dict(xy=np.array([-3.4, 0.0]), yaw=math.pi,
                                  handle=np.array([-3.4 - HANDLE_X, 0.0, HANDLE_Z]))
        a = np.array([math.cos(tr["yaw"]), math.sin(tr["yaw"])])            # along it, to its handle
        return tr, a, np.array([-a[1], a[0]])

    def tray_place(self, slot):
        """Where package number `slot` goes on the tray; where the cart stands beside it, and
        the point it comes from, farther out (the package hangs over the rim from there)."""
        tr, a, n = self.trolley_pose()
        xy = np.asarray(tr["xy"]) + a * SLOTS[slot % len(SLOTS)]
        side = n if abs((xy + n)[1]) < abs((xy - n)[1]) else -n           # the side nearer the yard's middle
        stand = xy + side * (LOAD_REACH + ARM_MOUNT[0])
        return dict(slot=slot, xy=xy, stand=stand, approach=stand + side * 0.45,
                    face=math.atan2(-side[1], -side[0]))

    def trolley_seen(self, obs, base):
        return None

    def read_trolley(self, view):
        """The trolley from its CYAN handle: the handle's axis, and the trolley behind it
        (its handle end is the one toward the store)."""
        ok = view.cyan & (view.P[:, :, 2] > HANDLE_Z - 0.12) & (view.P[:, :, 2] < HANDLE_Z + 0.08)
        if ok.sum() < 8:
            return None
        lab, n = ndimage.label(ok, structure=S8)
        k = 1 + int(np.argmax([int(np.sum(lab == i)) for i in range(1, n + 1)]))
        bar = bar_from(view, lab == k, 0.012)
        if bar is None:
            return None
        u = bar["dir"]
        a = np.array([-u[1], u[0]])
        if a[0] > 0:
            a = -a
        h = bar["mid"]
        return dict(xy=h[:2] - a * HANDLE_X, yaw=math.atan2(a[1], a[0]), handle=np.array([h[0], h[1], HANDLE_Z]),
                    n=bar["n"])

    def read_plates(self, view, vessels):
        """The class of each vessel whose plate is in the picture."""
        out = {}
        for v, sv in enumerate(vessels):
            face = sv[1] - math.copysign(STAND_HALF + 0.003, sv[1])
            m = view.valid & ~view.own & (np.abs(view.P[:, :, 0] - sv[0]) < 0.08) \
                & (np.abs(view.P[:, :, 1] - face) < 0.025) & (view.P[:, :, 2] > 0.11) & (view.P[:, :, 2] < 0.21)
            c = classify(view.rgb[m])
            if c is not None:
                out[v] = c
        return out

    def vessel_for(self, cls, obs):
        for v, c in self.vessel_cls.items():
            if c == cls:
                return v
        used = {p.get("vessel") for p in self.loaded if p.get("done")}
        free = [v for v in range(len(obs["vessels"])) if v not in used and v not in self.vessel_cls]
        return free[0] if free else 0

    def do_hitch(self, obs, base):
        """Out to the trolley, hook its handle, tow it into the store, let go, look at it and
        at the vessels' plates."""
        if not self.idle():
            return
        t = obs["t_s"]
        tr, a, n = self.trolley_pose()
        self.look = 1
        if self.sub == "plan":
            outside = np.array([-1.0, 0.0])
            if base[0] > -0.4:                         # still in the lab: out along the map
                room = self.map.room(CART_R)
                here = Map.nearest(room, base, 0.5)
                dist = Map.wave(room, here) if here is not None else None
                c = None if dist is None else Map.nearest(room, outside, 0.3)
                if c is None or not np.isfinite(dist[c]):
                    self.say(t, "no way out to the trolley")
                    self.phase = "stay"
                    return
                way = Map.path(room, dist, c)
                self.follow(way[1:-1] + [outside], base, final=math.pi)
            side = n if abs((np.asarray(tr["xy"]) + n)[1]) < abs((np.asarray(tr["xy"]) - n)[1]) else -n
            handle = np.asarray(tr["handle"])
            stand = handle[:2] + a * HITCH_AHEAD
            pre = handle[:2] + a * (HITCH_AHEAD + 0.25)       # clear of the store's front wall
            lane = np.asarray(tr["xy"]) + side * 1.3
            self.drive.to(xy=np.array([-1.7, lane[1]]))
            self.drive.to(xy=np.array([pre[0], lane[1]]))
            self.drive.to(xy=pre)
            self.drive.to(yaw=tr["yaw"] + math.pi)
            self.drive.to(xy=stand, limits=(0.2, 0.3, 0.5, 1.0))
            self.sub = "read"
        elif self.sub == "read":
            self.fine = dict(want="trolley", after=self.pictures + 2)
            self.sub = "latch"
        elif self.sub == "latch":
            if self.fine.get("want"):
                return
            if self.fine["answer"] is not None:
                self.trolley = self.fine["answer"]
            h = world_to_arm(base, self.trolley["handle"])
            st = self.arm.s
            st["shift"], st["fail"] = np.zeros(3), ""
            # slowly onto the axis, and still before it closes: the trolley takes whatever speed
            # the latch has when it closes, and jolts what stands on its tray
            st["queue"] = [("goto", h + np.array([0.0, 0.0, 0.09]), 0.45, "abs"), ("goto", h + np.array([0.0, 0.0, 0.015]), 0.12, "abs"),
                           ("goto", h, 0.02, "abs"), ("settle", 30), ("latch", 1, 0.0012), ("held",), ("settle", 25)]
            self.say(t, "hooking the trolley's handle at {} in the arm's frame".format(np.round(h, 3)))
            self.sub = "tow"
        elif self.sub == "tow":
            if self.arm.s["fail"] or not obs["latched"]:
                self.say(t, "the latch did not close on the trolley's handle")
                self.phase = "stay"
                return
            # from where the cart stands (no step in what the drive asks for): turn square to the
            # store's door about the handle, slowly, then straight back through the door
            self.drive.ref = np.array(base, float)
            handle = arm_to_world(base, self.arm.here())
            self.drive.to(xy=handle[:2] - np.array([HITCH_AHEAD, 0.0]), yaw=0.0, limits=TOW_SPEED)
            self.drive.to(xy=np.array([STORE_X1 + 0.6, 0.0]), limits=TOW_SPEED)
            self.drive.to(xy=np.array([STORE_X1 - 2.4, 0.0]), limits=TOW_SPEED)
            self.say(t, "towing the trolley into the store")
            self.sub = "park"
        elif self.sub == "park":
            self.wait += 1
            if self.wait < 50:
                return
            self.wait = 0
            self.arm.s["queue"] = [("latch", 0), ("gain", 0.0), ("rise", 0.10),
                                   ("goto", kin().fk(TRAVEL_Q)[0], 0.45, "abs")]
            self.sub = "back"
        elif self.sub == "back":
            self.drive.to(xy=np.array([STORE_X1 - 2.85, 0.0]))
            self.sub = "see"
        elif self.sub == "see":
            self.fine = dict(want="trolley", after=self.pictures + 2)
            self.sub = "seen"
        elif self.sub == "seen":
            if self.fine.get("want"):
                return
            if self.fine["answer"] is not None:
                self.trolley = self.fine["answer"]
            self.say(t, "the trolley parked at {}, heading {:.2f}".format(np.round(self.trolley["xy"], 2), self.trolley["yaw"]))
            self.drive.to(xy=np.array([TURN_X, -0.45]))
            self.drive.to(yaw=math.pi / 2)
            self.sub = "plates1"
        elif self.sub in ("plates1", "plates2"):
            self.fine = dict(want="plates", vessels=[list(v) for v in obs["vessels"]], after=self.pictures + 2)
            self.sub += "r"
        elif self.sub in ("plates1r", "plates2r"):
            if self.fine.get("want"):
                return
            self.vessel_cls.update(self.fine["answer"])
            if self.sub == "plates1r":
                self.drive.to(xy=np.array([TURN_X, 0.45]))
                self.drive.to(yaw=-math.pi / 2)
                self.sub = "plates2"
                return
            nv = len(obs["vessels"])
            if len(self.vessel_cls) == nv - 1:          # one not read: the class left over
                missing = [v for v in range(nv) if v not in self.vessel_cls][0]
                self.vessel_cls[missing] = [c for c in range(nv) if c not in self.vessel_cls.values()][0]
            self.say(t, "the vessels' classes: {}".format(self.vessel_cls))
            self.stage = "unload"
            self.delivered = 0
            self.phase, self.sub = "unload", "next"

    def do_unload(self, obs, base):
        """From the tray into the vessels, one package at a time."""
        if not self.idle():
            return
        t = obs["t_s"]
        if self.sub == "next":
            self.look = 1
            todo = [p for p in self.loaded if not p.get("done")]
            if not todo:
                self.say(t, "all {} are in their vessels".format(self.delivered))
                self.phase = "stay"
                return
            tr, a, n = self.trolley_pose()
            item = self.unloading = todo[0]
            xy = np.asarray(tr["xy"]) + a * SLOTS[item["slot"] % len(SLOTS)]
            item["xy"] = xy
            item["vessel"] = self.vessel_for(item.get("cls"), obs)
            sy = 1.0 if obs["vessels"][item["vessel"]][1] > 0 else -1.0
            side = n if n[1] * sy > 0 else -n
            stand = xy + side * (LOAD_REACH + ARM_MOUNT[0])
            # into the lane on this side; if the vessel is on the other side, across behind the
            # trolley's handle end (at TURN_X), never past the trolley
            now = 1.0 if base[1] > 0 else -1.0
            self.drive.to(xy=np.array([base[0], now * LANE]))
            if now != sy:
                self.drive.to(xy=np.array([TURN_X, now * LANE]))
                self.drive.to(xy=np.array([TURN_X, sy * LANE]))
            self.drive.to(xy=np.array([xy[0], sy * LANE]))
            self.drive.to(yaw=math.atan2(-side[1], -side[0]))
            self.drive.to(xy=stand, limits=(0.2, 0.3, 0.5, 1.0))
            self.say(t, "to the tray's place {} (class {}, for vessel {})".format(item["slot"], item.get("cls"), item["vessel"]))
            self.sub = "read"
        elif self.sub == "read":
            xy = self.unloading["xy"]
            self.fine = dict(want="tray_handle", near=np.array([xy[0], xy[1], TRAY_TOP + BOX_HALF_Z + BAR_Z]),
                             after=self.pictures + 2)
            self.sub = "pick"
        elif self.sub == "pick":
            if self.fine.get("want"):
                return
            got = self.fine["answer"]
            item = self.unloading
            item["done"] = True
            if not got["found"]:
                self.say(t, "the package on the tray cannot be read")
                self.sub = "next"
                return
            if got.get("cls") is not None and got["cls"] != item.get("cls"):
                self.say(t, "its label reads class {} (was {})".format(got["cls"], item.get("cls")))
                item["cls"] = got["cls"]
                item["vessel"] = self.vessel_for(got["cls"], obs)
            self.vessel_target = np.asarray(obs["vessels"][item["vessel"]], float)
            self.begin_pick(obs, base, got)

    def plan_to_vessel(self, obs, base):
        """With the package hanging: back away from the tray into the lane on the vessel's
        side, turn round between the stands, and stand before the vessel."""
        lim = self.carry_limits(obs)
        vessel = np.asarray(self.vessel_target, float)
        sy = 1.0 if vessel[1] > 0 else -1.0
        stand = vessel - np.array([0.0, sy * (PLACE_REACH + ARM_MOUNT[0])])
        self.drive.to(xy=np.array([base[0], sy * LANE]), limits=lim, shaper=ZVD)
        self.drive.to(xy=np.array([TURN_X, sy * LANE]), limits=lim, shaper=ZVD)
        self.drive.to(yaw=sy * math.pi / 2, limits=lim, shaper=ZVD)
        self.drive.to(xy=np.array([vessel[0], sy * LANE]), limits=lim, shaper=ZVD)    # along the lane: clear of the open lids
        self.drive.to(xy=stand, limits=lim, shaper=ZVD)
        self.say(obs["t_s"], "to the vessel at {}".format(np.round(vessel[:2], 2)))
        return True


class Controller(object):
    """Answers every request from the observation it carries. When a step is asked twice,
    one of the two observations is a shifted copy: both are answered, but what the controller
    remembers is advanced with the true one only, which is the one whose joint readings fit
    what the arm was doing a step earlier."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.m = Mission()
        self.snap = None
        self.last_t = None
        self.expect = TRAVEL_Q.copy()          # the joint readings the next step should bring
        self.kept = (TRAVEL_Q.copy(), np.zeros(7))
        self.off = 0.0

    def act(self, t, obs):
        q, qd = np.array(obs["q"], float), np.array(obs["qd"], float)
        if t == self.last_t and self.snap is not None:
            off = float(np.linalg.norm(q - self.expect))
            first = self.m                             # what the first of the two left behind
            self.m = copy.deepcopy(self.snap)          # answer this one from the state before the step
            action = self.m.act(t, obs)
            if off > self.off:
                self.m = first                         # this one was the shifted copy
            else:
                self.off, self.kept = off, (q, qd)
            return action
        self.expect = self.kept[0] + self.kept[1] * DT
        self.off, self.kept = float(np.linalg.norm(q - self.expect)), (q, qd)
        self.snap = copy.deepcopy(self.m)
        self.last_t = t
        return self.m.act(t, obs)


def main():
    ctl = Controller()
    kin()
    print("READY", flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
        except ValueError:
            print(json.dumps({"error": "bad json"}), flush=True)
            continue
        kind = req.get("type")
        if kind == "reset":
            ctl.reset()
            print(json.dumps({"ok": True}), flush=True)
        elif kind == "act":
            try:
                print(json.dumps({"action": ctl.act(req["t"], req["obs"])}), flush=True)
            except Exception as exc:  # noqa: BLE001: report on the wire, keep serving
                print(json.dumps({"error": str(exc)[:200]}), flush=True)
        elif kind == "close":
            print(json.dumps({"ok": True}), flush=True)
            return
        else:
            print(json.dumps({"error": "unknown request"}), flush=True)


if __name__ == "__main__":
    main()
