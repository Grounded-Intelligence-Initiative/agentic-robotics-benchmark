"""The abdominal cavity's geometry: the tissue surface below the instruments, the wall above them, the trocars.

The tissue is a height field z = h(x, y) in the patient frame (model.py): a bowl (a flat floor that rises into the
side walls) plus smooth mounds (organs) and a fine ripple. It is not simulated: the whole surface rises and falls with
the patient's breathing (a kinematic offset in z). PhysX deformable bodies would need GPU dynamics, which does not
repeat across GPU models; a camera-holding controller does not push on tissue, so nothing is lost for the job.

The same function gives the renderer its mesh and the episode its lens-contact test, from one JSON-serialisable
parameter dict (`Tissue.params`). Pure numpy.
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


class Tissue(object):
    def __init__(self, params):
        self.params = params
        self.mounds = np.array(params["mounds"], dtype=float).reshape(-1, 6)   # x, y, height, sx, sy, angle
        self.ripples = np.array(params["ripples"], dtype=float).reshape(-1, 4)  # kx, ky, phase, amplitude
        self.breath = params["breath"]                                          # amplitude, rate_hz, phase

    @staticmethod
    def draw(rng):
        """A cavity of the hidden distribution: 5-8 organ mounds, a fine ripple, the breathing."""
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
        breath = [rng.uniform(0.0015, 0.0035), rng.uniform(0.20, 0.30), rng.uniform(0.0, 2.0 * math.pi)]
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
        """The surface's kinematic z offset at a simulated time."""
        amp, rate, phase = self.breath
        return amp * math.sin(2.0 * math.pi * rate * time_s + phase)

    def clearance(self, point, time_s):
        """How far a point is above the breathing surface (negative: inside the tissue)."""
        return float(point[2] - (self.height(point[0], point[1]) + self.breathing(time_s)))

    def grid(self):
        """(xs, ys, Z): the renderer's mesh grid of the resting surface."""
        xs = np.arange(MESH_X[0], MESH_X[1] + 1e-9, MESH_STEP)
        ys = np.arange(MESH_Y[0], MESH_Y[1] + 1e-9, MESH_STEP)
        gx, gy = np.meshgrid(xs, ys, indexing="xy")
        return xs, ys, self.height(gx, gy)
