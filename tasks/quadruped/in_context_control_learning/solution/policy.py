"""Scripted image-only reference for the delivery bay. Not an LLM result.

It reads only what the agent gets: the 256x256 top-view image (calibration from the instruction) and the JSON status.
It presses each button once to learn what it does, keeps a map of every wall pixel it has seen, plans with A* over
its own measured button displacements (unknown floor counts as free), re-plans after every press, refines the
displacement estimates from presses that hit nothing, presses every button once more after the loading stop (the
instruction says the robot is set down facing another way there), and advances as soon as a delivery point is
reached. It proves the task can be done from the agent's observations.
"""
import base64
from collections import deque
import heapq
import io
import math

import numpy as np
from PIL import Image
from scipy import ndimage

from coordinates import SIZE, pixel_to_world, world_to_pixel

RADIUS = .16              # the robot's footprint, from the instruction
MARGIN = .04              # planning clearance beyond the footprint (m)
CELL = 2                  # planning grid: 2 x 2 image pixels (6.25 cm)
LOADING_STOP = 3          # from the instruction: the robot is loaded when advancing from the third delivery point


def decode(obs):
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(obs["images"][0]))).convert("RGB"))


def masks(im):
    r, g, b = (im[:, :, i].astype(float) for i in range(3))
    red = (r > 90) & (r > 3*g) & (r > 3*b)
    blue = (b > 80) & (b > 2*r) & (b > 1.8*g)
    tint, green_tint = b-r, g-r
    wall = (tint > 15) & (tint < 140) & (im.mean(axis=2) > 60) & (green_tint > .15*tint) & (green_tint < .8*tint) & ~blue
    lit = im.max(axis=2) > 16
    return red, blue, wall, lit


def centroid(mask, largest=False):
    if largest:
        labels, count = ndimage.label(mask)
        if count:
            sizes = ndimage.sum(mask, labels, np.arange(1, count+1))
            mask = labels == int(np.argmax(sizes)+1)
    ys, xs = np.where(mask)
    if not len(xs):
        return None
    return np.array(pixel_to_world(float(xs.mean()), float(ys.mean())))


class Navigator:
    def __init__(self):
        self.walls = np.zeros((SIZE, SIZE), bool)       # wall pixels seen so far (the map is fixed in an episode)
        self.map_version, self.cache = 0, None
        self.vectors = {}                                # button -> measured displacement per press (m)
        self.last = None                                 # (button, position before the press)
        self.reloaded = False

    def perceive(self, obs):
        im = decode(obs)
        red, blue, wall, lit = masks(im)
        p = centroid(red, largest=True)
        if p is None:
            raise ValueError("robot marker not visible")
        # The robot itself is not a wall: clear a disc around it before adding what is lit now.
        u, v = world_to_pixel(*p)
        yy, xx = np.mgrid[:SIZE, :SIZE]
        body = (xx-u)**2 + (yy-v)**2 < (RADIUS*32+3)**2
        new = wall & ~body & ~self.walls
        if new.any():
            self.walls |= new
            self.map_version += 1
        goal = centroid(blue)
        return p, goal

    def learn(self, p):
        if self.last is None:
            return
        button, before = self.last
        delta = p-before
        old = self.vectors.get(button)
        if old is None:
            self.vectors[button] = delta
        elif np.linalg.norm(delta) > .45*np.linalg.norm(old) and np.dot(delta, old) > 0 and not self._near_wall(before):
            # Presses that hit nothing refine the estimate; a blocked press says nothing about the gait.
            self.vectors[button] = .6*old+.4*delta

    def _near_wall(self, p):
        u, v = world_to_pixel(*p)
        r = int((RADIUS+.12)*32)
        u0, v0 = int(round(u)), int(round(v))
        return self.walls[max(0, v0-r):v0+r+1, max(0, u0-r):u0+r+1].any()

    def grid(self):
        coarse = self.walls.reshape(SIZE//CELL, CELL, SIZE//CELL, CELL).any(axis=(1, 3))
        n = int(math.ceil((RADIUS+MARGIN)/(CELL/32)))
        yy, xx = np.mgrid[-n:n+1, -n:n+1]
        blocked = ndimage.binary_dilation(coarse, structure=xx*xx+yy*yy <= ((RADIUS+MARGIN)/(CELL/32))**2)
        blocked[:2, :] = blocked[-2:, :] = blocked[:, :2] = blocked[:, -2:] = True
        return blocked

    @staticmethod
    def cell(p):
        u, v = world_to_pixel(*p)
        return (int(np.clip(v//CELL, 0, SIZE//CELL-1)), int(np.clip(u//CELL, 0, SIZE//CELL-1)))

    def geodesic(self, blocked, goal):
        """Grid distance (m) from every free cell to the goal, 8-connected."""
        n = SIZE//CELL
        dist = np.full((n, n), np.inf)
        start = self.cell(goal)
        dist[start] = 0
        frontier = [(0., start)]
        step = CELL/32
        moves = [(dy, dx, step*math.hypot(dy, dx)) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx]
        free = ~blocked
        free[start] = True
        while frontier:
            d, (y, x) = heapq.heappop(frontier)
            if d > dist[y, x]:
                continue
            for dy, dx, c in moves:
                ny, nx = y+dy, x+dx
                if 0 <= ny < n and 0 <= nx < n and free[ny, nx] and d+c < dist[ny, nx]:
                    dist[ny, nx] = d+c
                    heapq.heappush(frontier, (d+c, (ny, nx)))
        return dist

    def collides(self, blocked, a, b):
        for f in (.25, .5, .75, 1.):
            if blocked[self.cell(a+(b-a)*f)]:
                return True
        return False

    def choose(self, p, goal):
        key = (self.map_version, self.cell(goal))
        if self.cache is None or self.cache[0] != key:
            blocked = self.grid()
            self.cache = (key, blocked, self.geodesic(blocked, goal))
        _, blocked, dist = self.cache
        vectors = {b: v for b, v in self.vectors.items() if np.linalg.norm(v) > .03}
        longest = max(np.linalg.norm(v) for v in vectors.values())

        def h(q):
            d = dist[self.cell(q)]
            return (d if np.isfinite(d) else 50.)/longest

        start = tuple(np.round(p/.04).astype(int))
        frontier = [(h(p), 0, start, p, None)]
        seen = {start: 0}
        first_of = {}
        expansions = 0
        best = (h(p), None)
        while frontier and expansions < 6000:
            f, g, key, q, first = heapq.heappop(frontier)
            if np.linalg.norm(q-goal) < .18:
                return first
            expansions += 1
            if f-g < best[0] and first is not None:
                best = (f-g, first)
            for button, v in vectors.items():
                nq = q+v
                if self.collides(blocked, q, nq) and not (g == 0 and blocked[self.cell(q)]):
                    continue
                if g == 0 and blocked[self.cell(q)] and blocked[self.cell(nq)] and dist[self.cell(nq)] >= dist[self.cell(q)]:
                    continue
                nkey = tuple(np.round(nq/.04).astype(int))
                if seen.get(nkey, 1e9) <= g+1:
                    continue
                seen[nkey] = g+1
                heapq.heappush(frontier, (g+1+h(nq), g+1, nkey, nq, button if first is None else first))
        if best[1] is not None:
            return best[1]
        # Boxed in by the margin: take the press that gains the most clearance.
        clearance = ndimage.distance_transform_edt(~blocked)
        return max(vectors, key=lambda b: clearance[self.cell(p+vectors[b])])

    def act(self, obs):
        """The next request: ("act", button) or ("advance", None)."""
        p, goal = self.perceive(obs)
        self.learn(p)
        if obs["goal_reached"] and not obs["done"]:
            self.last = None
            return "advance", None
        if obs["delivery"] > LOADING_STOP and not self.reloaded:
            self.vectors, self.reloaded = {}, True       # new heading and speed: learn the buttons again
        if len(self.vectors) < 4:
            button = len(self.vectors)
        else:
            if goal is None:
                raise ValueError("delivery beacon not visible")
            button = self.choose(p, goal)
        self.last = (button, p)
        return "act", button


def solve(client):
    nav = Navigator()
    obs = client.observe()
    while not obs["done"]:
        kind, button = nav.act(obs)
        obs = client.advance() if kind == "advance" else client.act([button/3])
    return obs
