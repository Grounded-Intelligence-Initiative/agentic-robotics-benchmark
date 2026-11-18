"""Author demonstration: choose a route using only UAV RGB and local tracking."""
import math

import numpy as np

from client import camera_image


HALL_SWEEP = [(x, y) for j, x in enumerate((0., 1.05, 2.10, 3.18))
              for y in ((-.90, 0., .90) if j % 2 == 0 else (.90, 0., -.90))]
RES = .025


def waypoints(side, goal):
    # Clear the island with the whole stance before starting the final lateral leg.
    return [(0., side*1.06), (max(3.29, goal[0]+.07), side*1.06), tuple(goal)]


class SurveyMap:
    def __init__(self):
        self.wall = set()
        self.covered = set()
        self.blue = set()

    @staticmethod
    def project(mask, centre, height):
        r, c = np.nonzero(mask)
        pixel_m = 2*height*math.tan(math.radians(39))/256
        pr = np.stack([c+.5-128, 128-r-.5], axis=1)*pixel_m+centre
        return set(map(tuple, np.rint(pr/RES).astype(int)))

    def add(self, observation):
        centre = observation["tracking_xy_m"]
        r, g, b = camera_image(observation).astype(np.int16).transpose(2, 0, 1)
        orange = (r > g+30) & (g > b+10) & (r > b+65)
        blue = (b > r+58) & (b > g+28)
        self.wall.update(self.project(orange, centre, 1.65-.32))
        self.blue.update(self.project(blue, centre, 1.65))
        self.covered.update(self.project(np.ones_like(orange), centre, 1.65-.32))

    def route(self):
        if not self.blue:
            raise ValueError("no dock visible in the survey")
        blue = np.asarray(sorted(self.blue))*RES
        goal = .5*(blue.min(axis=0)+blue.max(axis=0))+[.026, 0.]
        # Require a surveyed and unobstructed tube down one of the two aisles.
        feasible = []
        for side in (-1, 1):
            tube = {(i, j) for i in range(round(.62/RES), round(2.74/RES))
                    for j in range(round((side*1.06-.27)/RES), round((side*1.06+.27)/RES)+1)}
            if tube <= self.covered and not (tube & self.wall):
                feasible.append(side)
        if len(feasible) != 1:
            raise ValueError(f"survey did not discriminate a corridor: {feasible}")
        return feasible[0], goal


def scan(robot, keep=True):
    mapping = SurveyMap() if keep else None
    obs = robot.observe()
    if mapping is not None:
        mapping.add(obs)
    for point in HALL_SWEEP:
        while True:
            delta = np.asarray(point)-obs["tracking_xy_m"]
            distance = np.linalg.norm(delta)
            if distance < .008:
                break
            obs = robot.act((delta*min(1., .57/distance)).tolist())
            if mapping is not None:
                mapping.add(obs)
    return mapping
