"""Metric floor plan from the public nadir RGB and tracking, without site inputs."""
import itertools
import math

import numpy as np

from client import camera_image


STEP = .02
X = np.arange(.40, 6.81, STEP)
Y = np.arange(-1.74, 1.75, STEP)
GO1_CLEARANCE = .56  # conservative working width, including tracking/gait allowance
APPROACH = .53


def runs(mask):
    transitions = np.diff(np.r_[False, mask, False].astype(np.int8))
    return zip(np.flatnonzero(transitions == 1), np.flatnonzero(transitions == -1))


class SurveyMap:
    def __init__(self):
        self.visible = np.zeros((len(X), len(Y)), dtype=bool)
        self.occupied = self.visible.copy()
        self.dock = self.visible.copy()

    def add(self, obs):
        rgb = np.asarray(camera_image(obs))
        h, w = rgb.shape[:2]
        focal = h / (2 * math.tan(math.radians(78)/2))
        x, y = obs["tracking_xy_m"]

        def sample(height):
            scale = focal/(1.65-height)
            u = np.rint((X-x)*scale+(w-1)/2).astype(int)
            v = np.rint((y-Y)*scale+(h-1)/2).astype(int)
            valid = ((u[:, None] >= 2) & (u[:, None] < w-2)
                     & (v >= 2) & (v < h-2))
            r, g, b = rgb[v.clip(0, h-1), u[:, None].clip(0, w-1)].astype(int).transpose(2, 0, 1)
            return valid, r, g, b

        valid, r, g, b = sample(.32)
        self.visible |= valid
        self.occupied |= valid & (r > 135) & (r > 1.8*g) & (g > 1.35*b)
        valid, r, g, b = sample(0)
        self.dock |= valid & (b > 135) & (b > 1.8*g) & (g > 1.8*r)

    def passages(self):
        dock_x, dock_y = np.where(self.dock)
        if not len(dock_x):
            raise ValueError("dock not observed")
        goal = np.array([X[dock_x].mean(), Y[dock_y].mean()])
        interior = X < goal[0]-.60
        if not self.visible[interior].all():
            raise ValueError("rack footprint incompletely surveyed")
        bands = self.occupied.mean(axis=1) > .35
        racks = []
        for lo, hi in runs(bands & interior):
            if (hi-lo)*STEP < .10:
                continue
            blocked = self.occupied[lo+2:hi-2].mean(axis=0) > .50
            gaps = [(float(Y[a]-STEP/2), float(Y[b-1]+STEP/2))
                    for a, b in runs(~blocked) if a and b < len(Y)]
            racks.append((float(X[lo]-STEP/2), float(X[hi-1]+STEP/2), gaps))
        if not racks or any(not rack[2] for rack in racks):
            raise ValueError("no complete series of passages in the RGB map")
        return racks, goal

    def route(self, point_rack=None):
        racks, goal = self.passages()
        candidates = []
        for i, (near, far, gaps) in enumerate(racks):
            minimum = 0. if i == point_rack else GO1_CLEARANCE
            candidates.append([
                ((near-APPROACH, (a+b)/2), (far+APPROACH, (a+b)/2))
                for a, b in gaps if b-a >= minimum
            ])
        if not all(candidates):
            raise ValueError("no route with the needed ground clearance")

        def length(path):
            return sum(math.dist(a, b) for a, b in itertools.pairwise([(0., 0.), *path]))

        paths = ([p for passage in combo for p in passage]+[goal.tolist()]
                 for combo in itertools.product(*candidates))
        return min(paths, key=length)


def scan(robot):
    surveyed = SurveyMap()
    obs = robot.observe()
    surveyed.add(obs)
    for row, y in enumerate((0., 1.10, -1.10)):
        xs = np.linspace(0., 6.60, 13)
        for x in xs if row % 2 == 0 else xs[::-1]:
            target = np.array([x, y])
            delta = target-obs["tracking_xy_m"]
            while np.linalg.norm(delta) > 1e-6:
                delta *= min(1., .55/np.linalg.norm(delta))
                obs = robot.act(delta.tolist())
                surveyed.add(obs)
                delta = target-obs["tracking_xy_m"]
    return surveyed
