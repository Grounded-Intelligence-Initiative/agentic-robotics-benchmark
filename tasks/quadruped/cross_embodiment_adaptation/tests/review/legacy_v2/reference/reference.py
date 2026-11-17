"""Image-only reference mission program: evidence that the robot's own observations suffice.

No token and no truth: it uses only what the instruction gives an agent (the top-view image with its
published calibration, the observation's fields, the documented footprints and opening widths).

  1. Scout: four probe presses identify the button permutation and the response; then it explores
     until the carrier's planned route is fully seen: plan the route on the current map with unseen
     cells assumed free, send the scout to the first unseen stretch of that route's corridor,
     repeat. Walls (blue-grey), crates (dark lids) and the pad (blue) are segmented by colour.
  2. Switch. Carrier: eight probe presses fit its lagged response (one pole and a steady speed,
     a 20 x 10 ms velocity-tracking model), then A* over whole button pulses in position-velocity
     space on the scouted map, inflated by the worst-case carrier footprint plus a margin, with a
     re-plan whenever the carrier is not where the model said it would be.
"""
import heapq
import os
import sys

import numpy as np
from scipy.ndimage import binary_dilation, label, maximum_filter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from client import Client, top_image  # noqa: E402

N, PX, FIELD = 256, 32., 3.8
VIEW = (1.4, .5)
SCOUT_REACH = .155+.03              # scout half side with legs, plus a margin (its contacts are harmless)
CARRIER_REACH = .43+.07             # worst-case carrier half side with legs (0.86 m square), plus a margin
CARRIER_START = (-2.8, 1.8)          # documented: (-2.8, 1.8), within 0.12 m
H, SUB = .01, 20                     # 0.2 s per press, modelled in 10 ms steps
MAX_EXPANSIONS = 150000

XS = (np.arange(N)+.5)/PX-4          # pixel column -> x
YS = 4-(np.arange(N)+.5)/PX          # pixel row -> y
XX, YY = np.meshgrid(XS, YS)


def cell(x, y):
    return min(N-1, max(0, int((4-y)*PX))), min(N-1, max(0, int((x+4)*PX)))


# --- perception ---------------------------------------------------------------------------- #
def classify(im):
    im = im.astype(int)
    r, g, b = im[..., 0], im[..., 1], im[..., 2]
    unseen = (r == 10) & (g == 10) & (b == 10)
    marker = (r > 110) & (g < 75) & (b < 75) & (r > 2.5*g)
    pad = (b > 210) & (g < 135) & (r < 135)
    wall = (b-r > 45) & (g > 105) & (b < 215)
    dark = (im.mean(axis=2) < 105) & (im.max(axis=2)-im.min(axis=2) < 50) & ~unseen
    return unseen, marker, pad, wall, dark


def blobs(mask, min_pixels):
    """`mask` without its connected components smaller than `min_pixels` (texture specks, not objects)."""
    labels, count = label(mask)
    if not count:
        return mask
    sizes = np.bincount(labels.ravel())
    keep = sizes >= min_pixels
    keep[0] = False
    return keep[labels]


def centroid(mask):
    rows, cols = np.nonzero(mask)
    if not len(rows):
        return None
    return np.array([XS[cols].mean(), YS[rows].mean()])


class Map:
    """-1 unseen, 0 free, 1 obstacle, per top-view pixel (3.1 cm)."""

    def __init__(self):
        self.state = np.full((N, N), -1, np.int8)
        self.goal = None

    def update(self, im, phase):
        unseen, marker, pad, wall, dark = classify(im)
        robot = centroid(marker)
        if robot is None:
            raise RuntimeError("tracking marker not visible")
        goal = centroid(pad)
        if goal is not None:
            self.goal = goal
        obstacle = blobs(wall, 12) | binary_dilation(blobs(dark, 25), iterations=2)
        half = .155 if phase == 0 else .43
        own = (np.abs(XX-robot[0]) < half+.06) & (np.abs(YY-robot[1]) < half+.06)
        seen = ((XX-robot[0])**2+(YY-robot[1])**2 < (VIEW[phase]-.06)**2) & ~unseen
        self.state[seen & obstacle & ~own] = 1
        self.state[seen & ~obstacle & (self.state != 1)] = 0
        self.state[own & seen & (self.state == -1)] = 0
        return robot

    def blocked(self, reach, unseen_free):
        obstacle = self.state == 1
        if not unseen_free:
            obstacle |= self.state == -1
        k = int(np.ceil(reach*PX))
        grown = maximum_filter(obstacle, size=2*k+1)
        edge = (np.abs(XX) > FIELD-reach) | (np.abs(YY) > FIELD-reach)
        return grown | edge


def bfs(blocked, sources):
    """4-connected grid distance (in cells) from the `sources` mask, as an expanding wavefront."""
    dist = np.full((N, N), np.inf)
    frontier = sources & ~blocked
    reached = frontier.copy()
    dist[frontier] = 0
    d = 0
    while frontier.any():
        d += 1
        grown = np.zeros_like(frontier)
        grown[1:] |= frontier[:-1]
        grown[:-1] |= frontier[1:]
        grown[:, 1:] |= frontier[:, :-1]
        grown[:, :-1] |= frontier[:, 1:]
        frontier = grown & ~blocked & ~reached
        dist[frontier] = d
        reached |= frontier
    return dist


def disk_cells(point, radius):
    return (XX-point[0])**2+(YY-point[1])**2 < radius**2


# --- response model --------------------------------------------------------------------------- #
class Model:
    """v' = a v + (1-a) w_b per 10 ms step, w_b = speed x the button's direction."""

    def __init__(self, a, speed, directions):
        self.a, self.speed, self.directions = a, speed, directions
        self.C, self.D = [], []
        c = 1.
        cp = dp = 0.
        for _ in range(SUB):
            c *= a
            cp += H*c
            dp += H*(1-c)
            self.C.append(cp)
            self.D.append(dp)
        self.cv = c
        self.step = speed*H*SUB

    def run(self, v, button):
        w = self.directions[button]*self.speed
        offsets = [(c*v[0]+e*w[0], c*v[1]+e*w[1]) for c, e in zip(self.C, self.D)]
        return offsets, (self.cv*v[0]+(1-self.cv)*w[0], self.cv*v[1]+(1-self.cv)*w[1])


def fit(points, buttons, directions):
    """Pole and steady speed that best reproduce the probe track, starting from rest."""
    best = None
    for a in np.concatenate([np.linspace(.3, .9, 25), np.linspace(.9, .99, 46)]):
        for speed in np.linspace(.3, 1.6, 66):
            model = Model(a, speed, directions)
            p, v, err = np.array(points[0]), (0., 0.), 0.
            for b, q in zip(buttons, points[1:]):
                offsets, v = model.run(v, b)
                p = p+offsets[-1]
                err += float(np.sum((p-q)**2))
            if best is None or err < best[0]:
                best = (err, a, speed)
    return Model(best[1], best[2], directions)


def directions_of(points, buttons):
    """The four buttons' directions: the permutation of +x, -x, +y, -y that best explains the probe
    displacements. A later press of the same button weighs more: a heavy body still coasts in the
    previous direction during the first press of a new button."""
    import itertools
    axes = [np.array(d, float) for d in ((1, 0), (-1, 0), (0, 1), (0, -1))]
    score = np.zeros((4, 4))
    count = {}
    for b, (p, q) in zip(buttons, zip(points, points[1:])):
        count[b] = count.get(b, 0)+1
        d = np.asarray(q)-np.asarray(p)
        for j, axis in enumerate(axes):
            score[b, j] += count[b]*float(d@axis)
    best = max(itertools.permutations(range(4)), key=lambda perm: sum(score[b, perm[b]] for b in range(4)))
    return {b: axes[best[b]] for b in range(4)}


# --- planning ------------------------------------------------------------------------------- #
def astar(model, blocked, dist, p, v, arrive):
    """Shortest press sequence from (p, v) into a cell with dist == 0 region (within `arrive`), or None."""
    def free(x, y, offsets):
        for dx, dy in offsets[1::2]:
            if blocked[cell(x+dx, y+dy)]:
                return False
        return True
    frontier = [(0., 0, 0, (float(p[0]), float(p[1])), (float(v[0]), float(v[1])), None)]
    seen, pushed, expansions = {}, 0, 0
    while frontier and expansions < MAX_EXPANSIONS:
        _, g, _, q, w, chain = heapq.heappop(frontier)
        if arrive(q):
            out = []
            while chain is not None:
                chain, b = chain
                out.append(b)
            return out[::-1]
        key = (round(q[0]/.012), round(q[1]/.012), round(w[0]/.06), round(w[1]/.06))
        if seen.get(key, 1e9) <= g:
            continue
        seen[key] = g
        expansions += 1
        for b in range(4):
            offsets, nw = model.run(w, b)
            if not free(q[0], q[1], offsets):
                continue
            nq = (q[0]+offsets[-1][0], q[1]+offsets[-1][1])
            h = dist[cell(*nq)]
            if not np.isfinite(h):
                continue
            pushed += 1
            heapq.heappush(frontier, (g+1+h/PX/model.step, g+1, pushed, nq, nw, (chain, b)))
    return None


class Robot:
    def __init__(self, client):
        self.client = client
        self.map = Map()
        self.obs = client.observe()
        self.phase = self.obs["phase"]
        self.p = self.map.update(top_image(self.obs), self.phase)
        self.v = (0., 0.)
        self.model = None

    def press(self, button):
        self.obs = self.client.act([button/3])
        self.p = self.map.update(top_image(self.obs), self.phase)
        if self.model is not None:
            _, self.v = self.model.run(self.v, button)
        return self.obs

    def calibrate(self, sequence):
        points, buttons = [self.p.copy()], []
        for b in sequence:
            self.press(b)
            points.append(self.p.copy())
            buttons.append(b)
            if self.obs["done"]:
                break
        directions = directions_of(points, buttons)
        self.model = fit(points, buttons, directions)
        # replay the probes through the fitted model for the velocity now
        v = (0., 0.)
        for b in buttons:
            _, v = self.model.run(v, b)
        self.v = v

    def go(self, reach, unseen_free, target, radius, max_presses, stop=None):
        """Press along an A* plan to within `radius` of `target`; re-plan on deviation or when the map
        changes the plan's clearance. Returns True on arrival."""
        presses = 0
        while presses < max_presses and not self.obs["done"]:
            if np.hypot(*(self.p-target)) < radius:
                return True
            if stop is not None and stop():
                return False
            plan = None
            # The scout may start a plan inside the margin it keeps (it can touch things): then it
            # plans with its bare footprint and may leave the cell it stands in.
            for margin_reach, escape in ((reach, 0.), (.16, .12)) if self.phase == 0 else ((reach, 0.),):
                blocked = self.map.blocked(margin_reach, unseen_free)
                blocked[cell(*self.p)] = False
                if escape:
                    blocked[disk_cells(self.p, escape)] = False
                dist = bfs(blocked, disk_cells(target, radius))
                plan = astar(self.model, blocked, dist, self.p, self.v,
                             lambda q: (q[0]-target[0])**2+(q[1]-target[1])**2 < radius**2)
                if plan:
                    break
            if not plan:
                return False
            for b in plan[:12]:
                offsets, _ = self.model.run(self.v, b)
                expected = self.p+offsets[-1]
                self.press(b)
                presses += 1
                if self.obs["done"] or np.hypot(*(self.p-expected)) > .03 or presses >= max_presses:
                    break
                if stop is not None and stop():
                    break
        return np.hypot(*(self.p-target)) < radius


def carrier_route(map_, unseen_free):
    blocked = map_.blocked(CARRIER_REACH, unseen_free)
    if map_.goal is None:
        return None, None
    dist = bfs(blocked, disk_cells(map_.goal, .2))
    start = cell(*CARRIER_START)
    if not np.isfinite(dist[start]):
        return None, dist
    path, (r, c) = [start], start
    while dist[r, c] > 0:
        r, c = min(((r+dr, c+dc) for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))
                    if 0 <= r+dr < N and 0 <= c+dc < N), key=lambda rc: dist[rc])
        path.append((r, c))
    return path, dist


INSIDE = (np.abs(XX) < FIELD-.02) & (np.abs(YY) < FIELD-.02)


def unseen_on_route(map_, path, skip):
    """An unseen pixel of the first stretch of the route whose corridor (the carrier's worst-case footprint)
    is not yet seen, the one nearest that stretch; pixels in `skip` (given up on) are not asked for again."""
    k = int(np.ceil((CARRIER_REACH+.05)*PX))
    unseen = (map_.state == -1) & INSIDE & ~skip
    # Around the carrier's start: its start jitter (0.12 m) and its probe presses need room on every side.
    around = unseen & ((XX-CARRIER_START[0])**2+(YY-CARRIER_START[1])**2 < (CARRIER_REACH+.45)**2)
    if around.any():
        rows, cols = np.nonzero(around)
        r, c = cell(*CARRIER_START)
        j = int(np.argmin((rows-r)**2+(cols-c)**2))
        return np.array([XS[cols[j]], YS[rows[j]]])
    for r, c in path:
        r0, c0 = max(0, r-k), max(0, c-k)
        window = unseen[r0:r+k+1, c0:c+k+1]
        if window.any():
            rows, cols = np.nonzero(window)
            j = int(np.argmin((rows+r0-r)**2+(cols+c0-c)**2))
            return np.array([XS[cols[j]+c0], YS[rows[j]+r0]])
    return None


def main():
    robot = Robot(Client())
    # Scout: identify the buttons, then explore until the carrier's route is seen end to end.
    robot.calibrate([0, 1, 2, 3])
    skip = np.zeros((N, N), bool)
    while robot.obs["steps_remaining"] > 4:
        path, _ = carrier_route(robot.map, unseen_free=True)
        if path is None:
            break
        target = unseen_on_route(robot.map, path, skip)
        if target is None:
            break
        state = robot.map.state
        r, c = cell(*target)

        def revealed(r=r, c=c):
            return state[r, c] != -1
        before = robot.obs["steps_remaining"]
        robot.go(SCOUT_REACH, True, target, 1.0, min(40, robot.obs["steps_remaining"]-4), stop=revealed)
        if state[r, c] == -1:
            # Not revealed (the scout could not get close): do not ask for this stretch again.
            skip[max(0, r-6):r+7, max(0, c-6):c+7] = True
        elif robot.obs["steps_remaining"] == before:
            continue
    robot.obs = robot.client.advance()
    robot.phase = 1
    robot.p = robot.map.update(top_image(robot.obs), 1)
    robot.model, robot.v = None, (0., 0.)
    # Carrier: fit its response, then deliver along the scouted map.
    robot.calibrate([0, 0, 1, 1, 2, 2, 3, 3])
    goal = robot.map.goal
    for unseen_free in (False, True):
        if robot.obs["done"]:
            break
        robot.go(CARRIER_REACH, unseen_free, goal, .18, robot.obs["steps_remaining"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
