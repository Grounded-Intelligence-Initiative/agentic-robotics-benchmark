"""The abdominal cavity's geometry: the tissue surface below the instruments, the wall above them, the trocars.

The tissue at rest is a height field z = h(x, y) in the patient frame (model.py): a bowl (a flat floor that rises into
the side walls) plus smooth mounds (organs) and a fine ripple. It is not simulated but it moves, kinematically, as a
pure function of time and of the instruments (version 0.3): every point of the resting surface is displaced by

    D(x, y, t) = b(t) s(y) (0, -k, 1)                           the breathing: irregular breaths (each its own period
                                                                and depth), more towards the head, and towards the feet
                                                                as it rises
               + sum over grips g: w_g(x, y) (tip_g(t) - a_g - b(t) s(a_g.y) (0, -k, 1))
                                                                traction: a hand that grasps the tissue drags it with
                                                                its tip, w_g a gaussian round the grasped point a_g

so the surface is no rigid body: it heaves by a different amount at every place, and where the surgeon pulls it
follows the instrument. PhysX deformable bodies would need GPU dynamics, which does not repeat across GPU models; the
kinematic field repeats to the last bit on every machine.

The same functions give the renderer its mesh (the vertices moved every frame), the generator the breathing that the
instruments ride on, and the episode its lens-contact test, from one JSON-serialisable parameter dict
(`Tissue.params`). Pure numpy.
"""

import math

import numpy as np

# the wall: the upper half of an ellipsoid (centre, semi-axes), the RCM just inside its top
WALL_CENTRE = (0.0, 0.06, -0.170)
WALL_AXES = (0.19, 0.20, 0.188)
FLOOR_Z = -0.170
BOWL_RISE = 0.10                # how far the floor has risen where it meets the wall's equator
MESH_X = (-0.21, 0.21)
MESH_Y = (-0.16, 0.28)
MESH_STEP = 0.0025
# where the surgeon works: the flat part of the floor in front of the camera trocar
WORK_X = (-0.095, 0.095)
WORK_Y = (0.030, 0.165)


def wall_height(x, y):
    """z of the abdominal wall above (x, y); -inf outside the ellipse."""
    cx, cy, cz = WALL_CENTRE
    ax, ay, az = WALL_AXES
    s = 1.0 - ((np.asarray(x, float) - cx) / ax) ** 2 - ((np.asarray(y, float) - cy) / ay) ** 2
    return np.where(s > 0.0, cz + az * np.sqrt(np.clip(s, 0.0, None)), -np.inf)


def trocar_point(x, y, proud=0.004):
    """A trocar's pivot on the wall at (x, y): the point its instrument's shaft passes through."""
    return np.array([x, y, float(wall_height(x, y)) - proud])


BREATHS_S = 120.0             # how much breathing a procedure draws (longer than any procedure)


def follow_weight(z):
    """How much an instrument's tip at height z rides on the breathing tissue: fully on the floor of the cavity, not
    at all up at the wall where the trocars are."""
    return float(np.clip((-0.05 - float(z)) / 0.05, 0.0, 1.0))


class Tissue(object):
    def __init__(self, params):
        self.params = params
        self.mounds = np.array(params["mounds"], dtype=float).reshape(-1, 6)   # x, y, height, sx, sy, angle
        self.ripples = np.array(params["ripples"], dtype=float).reshape(-1, 4)  # kx, ky, phase, amplitude
        br = params["breath"]                                                   # irregular breaths (version 0.3)
        self.breaths = np.array(br["breaths"], dtype=float).reshape(-1, 2)      # per breath: period (s), depth (m)
        self.starts = np.concatenate([[0.0], np.cumsum(self.breaths[:, 0])]) - float(br["offset_s"])
        self.k = float(br["headward"])                                          # the head-ward share of the motion
        self.grip_sigma = float(br["grip_sigma"])                               # m, how far round a grip it drags

    @staticmethod
    def draw(rng):
        """A cavity of the hidden distribution: 5-8 organ mounds, a fine ripple, the breathing (version 0.3: each
        breath its own period and depth around the patient's own, the gain over y and the head-ward component, and
        how far round its grip a grasping instrument drags the tissue)."""
        mounds = []
        for _ in range(int(rng.integers(5, 9))):
            x = rng.uniform(-0.15, 0.15)
            y = rng.uniform(-0.06, 0.22)
            inside = WORK_X[0] < x < WORK_X[1] and WORK_Y[0] < y < WORK_Y[1]
            height = rng.uniform(0.008, 0.022) if inside else rng.uniform(0.015, 0.045)
            mounds.append([x, y, height, rng.uniform(0.022, 0.055), rng.uniform(0.022, 0.055), rng.uniform(0.0, math.pi)])
        ripples = []
        for _ in range(6):
            k = rng.uniform(60.0, 160.0)
            a = rng.uniform(0.0, 2.0 * math.pi)
            ripples.append([k * math.cos(a), k * math.sin(a), rng.uniform(0.0, 2.0 * math.pi), rng.uniform(0.0004, 0.0011)])
        period, depth = rng.uniform(3.4, 4.6), rng.uniform(0.003, 0.008)
        breaths, total = [], 0.0
        while total < BREATHS_S:
            p = period * rng.uniform(0.85, 1.15)
            breaths.append([p, depth * rng.uniform(0.75, 1.25)])
            total += p
        breath = {"breaths": breaths, "offset_s": rng.uniform(0.0, period), "headward": rng.uniform(0.3, 0.7),
                  "grip_sigma": rng.uniform(0.010, 0.018)}
        return Tissue({"mounds": mounds, "ripples": ripples, "breath": breath})

    def height(self, x, y):
        """z of the resting surface at (x, y) (arrays broadcast)."""
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        cx, cy, _ = WALL_CENTRE
        ax, ay, _ = WALL_AXES
        rho2 = ((x - cx) / ax) ** 2 + ((y - cy) / ay) ** 2
        z = FLOOR_Z + BOWL_RISE * rho2 ** 2
        for mx, my, mh, sx, sy, ang in self.mounds:
            c, s = math.cos(ang), math.sin(ang)
            u = (x - mx) * c + (y - my) * s
            w = -(x - mx) * s + (y - my) * c
            z = z + mh * np.exp(-0.5 * ((u / sx) ** 2 + (w / sy) ** 2))
        for kx, ky, ph, amp in self.ripples:
            z = z + amp * np.sin(kx * x + ky * y + ph)
        return z

    def breathing(self, time_s):
        """The breathing's depth b(t) at a simulated time (m; 0 when breathed out): a raised cosine per breath."""
        i = int(np.searchsorted(self.starts, float(time_s), side="right")) - 1
        i = min(max(i, 0), len(self.breaths) - 1)
        period, depth = self.breaths[i]
        phase = (float(time_s) - self.starts[i]) / period
        return float(depth * 0.5 * (1.0 - math.cos(2.0 * math.pi * min(max(phase, 0.0), 1.0))))

    @staticmethod
    def gain(y):
        """How much of the breathing reaches the tissue at y: less at the camera trocar's end, more towards the head."""
        return 0.45 + 0.95 * np.clip((np.asarray(y, dtype=float) + 0.05) / 0.30, 0.0, 1.0)

    def breath_shift(self, x, y, time_s, b=None):
        """The breathing's displacement of the resting surface at (x, y) (arrays broadcast): (..., 3)."""
        b = self.breathing(time_s) if b is None else b
        g = b * self.gain(y) * np.ones_like(np.asarray(x, dtype=float))
        return np.stack([0.0 * g, -self.k * g, g], axis=-1)

    def displacement(self, x, y, time_s, grips=()):
        """D(x, y, t): where the resting surface's point (x, y, h(x, y)) has moved, breathing and traction (..., 3).
        `grips`: (anchor, tip) pairs, the grasped point at rest and the grasping tip now (3-vectors)."""
        b = self.breathing(time_s)
        d = self.breath_shift(x, y, time_s, b)
        for anchor, tip in grips:
            a = np.asarray(anchor, dtype=float)
            pull = np.asarray(tip, dtype=float) - a - self.breath_shift(a[0], a[1], time_s, b)
            w = np.exp(-((np.asarray(x, dtype=float) - a[0]) ** 2 + (np.asarray(y, dtype=float) - a[1]) ** 2)
                       / (2.0 * self.grip_sigma ** 2))
            d = d + w[..., None] * pull
        return d

    def surface_z(self, x, y, time_s, grips=()):
        """The moving surface's height above (x, y), to first order (the displacement taken at (x, y) itself)."""
        return self.height(x, y) + self.displacement(x, y, time_s, grips)[..., 2]

    def clearance(self, point, time_s, grips=()):
        """How far a point is above the moving surface (negative: inside the tissue)."""
        return float(point[2] - self.surface_z(point[0], point[1], time_s, grips))

    def grid(self):
        """(xs, ys, Z): the renderer's mesh grid of the resting surface."""
        xs = np.arange(MESH_X[0], MESH_X[1] + 1e-9, MESH_STEP)
        ys = np.arange(MESH_Y[0], MESH_Y[1] + 1e-9, MESH_STEP)
        gx, gy = np.meshgrid(xs, ys, indexing="xy")
        return xs, ys, self.height(gx, gy)
