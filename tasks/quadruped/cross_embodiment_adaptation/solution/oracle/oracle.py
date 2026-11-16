"""Privileged oracle mission program (maintainers' reference, not an agent's solution).

It holds the token whose sha256 only the grader knows, so the graded robot service answers its
`truth` call: the whole site and the robot's exact pose, velocity and dynamics. It proves every
hidden mission can be done: 12 scout presses in place, the switch, then the carrier to the pad
under model-predictive control on the true map. The route is the cheapest one on a grid of the
cells the carrier's circumscribed circle can occupy, with a toll near obstacles so that it keeps
to the middle of what is free. Before every press the oracle reads the true state, takes the
point of the route 0.4 m ahead, rolls every sequence of the next five presses through the exact
10 ms model of the lagged base and presses the first button of the sequence that stays nearest
that point, among those whose every sampled point is clear and whose first press touches
nothing even if it slips.

A slip cannot be known in advance, not even here: the oracle only knows, after the press, what
happened.
"""
import itertools
import math
import os
import sys

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from client import Client  # noqa: E402

TOKEN = open(os.path.join(HERE, "token.txt")).read().strip()
RES = .02                 # planning grid, metres
MARGIN = .03              # kept beyond the carrier's circumscribed circle
ARRIVE = .2               # the route's target: this close to the pad's centre (the rule is 0.3 m)
HORIZON = 5               # presses looked ahead
SAMPLES = (4, 9, 14, 19)  # substeps of a press checked for clearance
AHEAD = .4                # the route point the base steers for, metres along the route
ROOM = .12                # clearance below which a point of a sequence is charged
CLOSE = .25               # the charge at zero clearance, metres per press


def truth(robot):
    return robot.request("truth", token=TOKEN)


class Pulse:
    """One 0.2 s button pulse under MuJoCo's Euler step (joint damping implicit, the drive explicit):
    v' = a v + b vt per substep, so a substep's offset is C[k] v0 + D[k] vt."""

    def __init__(self, t):
        h, m, d, k = t["timestep"], t["mass"], t["damping"], t["drive"]
        a = (1-h*k/m)/(1+h*d/m)
        b = (h*k/m)/(1+h*d/m)
        C, D = [], []
        cv, dv, cp, dp = 1., 0., 0., 0.
        for _ in range(int(t["substeps"])):
            cv, dv = a*cv, a*dv+b
            cp, dp = cp+h*cv, dp+h*dv
            C.append(cp)
            D.append(dp)
        self.C, self.D = np.array(C)[list(SAMPLES)], np.array(D)[list(SAMPLES)]
        self.cv, self.dv = cv, dv
        self.coast = h*a/(1-a)          # distance per unit velocity until the base has stopped by itself


class Field:
    """The true map on a grid: cells the carrier's centre must not enter, and the length of the shortest free
    route from every cell to the pad."""

    def __init__(self, t):
        self.field = t["field"]
        n = self.n = int(round(2*self.field/RES))
        centres = -self.field+(np.arange(n)+.5)*RES
        xx, yy = np.meshgrid(centres, centres[::-1])
        near = np.full((n, n), np.inf)
        for cx, cy, hx, hy in t["obstacles"]:
            dx = np.maximum(0., np.abs(xx-cx)-hx)
            dy = np.maximum(0., np.abs(yy-cy)-hy)
            near = np.minimum(near, np.hypot(dx, dy))
        # What the carrier's circumscribed circle (any heading) may enter, with a margin.
        self.clearance = near-(t["half"]*math.sqrt(2)+t["leg"]+MARGIN)
        # The route: through the cells the site guarantees connected (every obstacle at least `route_radius`
        # away), with a toll that keeps it to the middle of what is free.
        routable = near >= t["route_radius"]-RES
        goal = np.array(t["goal"])
        source = ((xx-goal[0])**2+(yy-goal[1])**2 < ARRIVE**2) & routable
        toll = 1+4*np.clip(t["route_radius"]+.15-near, 0, None)/.15
        self.dist = self._geodesic(routable, source, toll)

    def _geodesic(self, free, source, toll):
        n = self.n
        index = np.arange(n*n).reshape(n, n)
        rows, cols, weights = [], [], []
        for dr, dc, length in ((0, 1, 1.), (1, 0, 1.), (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2))):
            a = (slice(0, n-dr), slice(max(0, -dc), n-max(0, dc)))
            b = (slice(dr, n), slice(max(0, dc), n+min(0, dc)))
            ok = free[a] & free[b]
            rows += [index[a][ok], index[b][ok]]
            cols += [index[b][ok], index[a][ok]]
            weights += [length*RES*.5*(toll[a][ok]+toll[b][ok])]*2
        src = np.flatnonzero(source)
        rows.append(np.full(len(src), n*n))
        cols.append(src)
        weights.append(np.full(len(src), 1e-9))
        graph = coo_matrix((np.concatenate(weights), (np.concatenate(rows), np.concatenate(cols))),
                           shape=(n*n+1, n*n+1)).tocsr()
        return dijkstra(graph, directed=True, indices=n*n)[:n*n].reshape(n, n)

    def cells(self, x, y):
        r = np.clip(((self.field-y)/RES).astype(int), 0, self.n-1)
        c = np.clip(((x+self.field)/RES).astype(int), 0, self.n-1)
        return r, c

    def ahead(self, p, length):
        """The point `length` metres down the route from `p` (the pad's own neighbourhood ends it)."""
        r, c = (int(v) for v in self.cells(np.float64(p[0]), np.float64(p[1])))
        if not np.isfinite(self.dist[r, c]):
            # Inside the margin: start from the nearest cell that has a route.
            rows, cols = np.nonzero(np.isfinite(self.dist[max(0, r-25):r+26, max(0, c-25):c+26]))
            if not len(rows):
                return np.asarray(p, float)
            j = int(np.argmin((rows-min(r, 25))**2+(cols-min(c, 25))**2))
            r, c = max(0, r-25)+int(rows[j]), max(0, c-25)+int(cols[j])
        walked = 0.
        while walked < length and self.dist[r, c] > 0:
            best = None
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r+dr, c+dc
                    if (dr or dc) and 0 <= rr < self.n and 0 <= cc < self.n and self.dist[rr, cc] < self.dist[r, c]:
                        if best is None or self.dist[rr, cc] < best[0]:
                            best = (self.dist[rr, cc], rr, cc, RES*math.hypot(dr, dc))
            if best is None:
                break
            _, r, c, step = best
            walked += step
        return np.array([-self.field+(c+.5)*RES, self.field-(r+.5)*RES])


class Controller:
    def __init__(self, t):
        self.field = Field(t)
        self.pulse = Pulse(t)
        self.sequences = np.array(list(itertools.product(range(4), repeat=HORIZON)))
        self.speed = t["gain"]*t["speed"]
        self.body = np.array(t["buttons"], float)
        self.loss = t["slip_loss"][1]
        self.turn = math.radians(t["slip_turn_deg"][1])
        self.goal = np.array(t["goal"])

    def commands(self, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return self.speed*self.body@np.array([[c, s], [-s, c]])        # each button's velocity in the hall

    def choose(self, p, v, yaw):
        """The button to press now."""
        pulse, field = self.pulse, self.field
        targets = self.commands(yaw)[self.sequences]                      # (S, H, 2)
        count = len(self.sequences)
        pos = np.broadcast_to(np.asarray(p, float), (count, 2)).copy()
        vel = np.broadcast_to(np.asarray(v, float), (count, 2)).copy()
        aim = field.ahead(p, AHEAD)
        if math.hypot(aim[0]-self.goal[0], aim[1]-self.goal[1]) < ARRIVE+RES:
            aim = self.goal
        clear = np.full(count, np.inf)
        cost = np.zeros(count)
        arrived = np.zeros(count, bool)
        for k in range(HORIZON):
            u = targets[:, k]
            pts = pos[:, None, :]+pulse.C[None, :, None]*vel[:, None, :]+pulse.D[None, :, None]*u[:, None, :]
            r, c = field.cells(pts[..., 0], pts[..., 1])
            room = field.clearance[r, c].min(axis=1)
            clear = np.where(arrived, clear, np.minimum(clear, room))
            pos = pts[:, -1]
            vel = pulse.cv*vel+pulse.dv*u
            # Distance to the route point after each press, summed over the horizon (sooner is better), and
            # a charge for running close to an obstacle. A sequence that has arrived costs nothing more:
            # the mission ends there.
            left = np.hypot(pos[:, 0]-aim[0], pos[:, 1]-aim[1])+CLOSE*np.clip(1-room/ROOM, 0, 1)
            cost += np.where(arrived, 0., left)
            arrived |= np.hypot(pos[:, 0]-self.goal[0], pos[:, 1]-self.goal[1]) < ARRIVE
        # Where the base would come to rest after the last press.
        rest = pos+vel*pulse.coast
        r, c = field.cells(rest[:, 0], rest[:, 1])
        clear = np.where(arrived, clear, np.minimum(clear, field.clearance[r, c]))
        cost = np.where(clear < 0, np.inf, cost)
        # The first press must not touch anything whatever it does: full grip, or the worst slip turning
        # either way (the margin is there for this).
        safe = np.ones(4, bool)
        base = self.commands(yaw)
        for b in range(4):
            for turn in (-self.turn, 0., self.turn):
                c_, s_ = math.cos(turn), math.sin(turn)
                u = (1-self.loss)*np.array([c_*base[b, 0]-s_*base[b, 1], s_*base[b, 0]+c_*base[b, 1]])
                pts = np.asarray(p, float)+pulse.C[:, None]*np.asarray(v, float)+pulse.D[:, None]*u
                end_v = pulse.cv*np.asarray(v, float)+pulse.dv*u
                pts = np.vstack([pts, pts[-1]+end_v*pulse.coast])
                r, c = field.cells(pts[:, 0], pts[:, 1])
                if field.clearance[r, c].min() < .01-MARGIN:
                    safe[b] = False
        cost = np.where(safe[self.sequences[:, 0]], cost, np.inf)
        if np.isfinite(cost).any():
            return int(self.sequences[int(np.argmin(cost)), 0])
        # Nothing is provably clear: take the sequence that keeps the largest clearance.
        return int(self.sequences[int(np.argmax(clear)), 0])


def main():
    robot = Client()
    obs = robot.observe()
    t = truth(robot)
    # Scout: the 12 presses the switch requires, back and forth in the clear space around the dock.
    back = next(i for i in range(4) if t["buttons"][i] == [-t["buttons"][0][0], -t["buttons"][0][1]])
    for k in range(12):
        obs = robot.act([(0, back)[k % 2]/3])
    obs = robot.advance()
    t = truth(robot)
    controller = Controller(t)
    while not obs["done"]:
        button = controller.choose(t["position"], t["velocity"], t["yaw"])
        obs = robot.act([button/3])
        t = truth(robot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
