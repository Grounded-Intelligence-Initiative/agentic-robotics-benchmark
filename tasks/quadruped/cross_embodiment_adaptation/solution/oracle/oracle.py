"""Privileged oracle mission program (maintainers' reference, not an agent's solution).

It holds the token whose sha256 only the grader knows, so the graded robot service answers its
`truth` call: the whole site and the robot's exact state and dynamics. It proves every hidden
mission can be done: 12 scout actions in place, the switch, then the carrier along a route found
by A* over whole button pulses in position-velocity space (the heavy carrier coasts), each
pulse's 0.2 s predicted exactly and checked for clearance at its 10 ms substeps. It re-plans
whenever the carrier is not where the plan said it would be.
"""
import heapq
import os
import sys
from collections import deque

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from client import Client  # noqa: E402

TOKEN = open(os.path.join(HERE, "token.txt")).read().strip()
FIELD = 3.8
RES = .02
MARGIN = .04
ARRIVE = .24
MAX_EXPANSIONS = 250000


def truth(robot):
    return robot.request("truth", token=TOKEN)


class Pulse:
    """One 0.2 s button pulse under MuJoCo's Euler step (joint damping implicit, the drive explicit):
    v' = a v + b vt per substep, so every substep's offset is C[k] v0 + D[k] vt."""

    def __init__(self, t):
        h, m, d, k = t["timestep"], t["mass"], t["damping"], t["drive"]
        a = (1-h*k/m)/(1+h*d/m)
        b = (h*k/m)/(1+h*d/m)
        self.a, self.b = a, b
        self.C, self.D = [], []
        cv, dv, cp, dp = 1., 0., 0., 0.
        for _ in range(int(t["substeps"])):
            cv, dv = a*cv, a*dv+b
            cp, dp = cp+h*cv, dp+h*dv
            self.C.append(cp)
            self.D.append(dp)
        self.cv, self.dv = cv, dv

    def run(self, v, vt):
        """Substep offsets (list of (dx, dy)) and the final velocity."""
        offsets = [(c*v[0]+e*vt[0], c*v[1]+e*vt[1]) for c, e in zip(self.C, self.D)]
        return offsets, (self.cv*v[0]+self.dv*vt[0], self.cv*v[1]+self.dv*vt[1])


class Planner:
    def __init__(self, t):
        n = self.n = int(round(2*FIELD/RES))
        centres = -FIELD+(np.arange(n)+.5)*RES
        xx, yy = np.meshgrid(centres, centres[::-1])
        reach = t["half"]+t["leg"]+MARGIN
        blocked = (np.abs(xx) > FIELD-reach) | (np.abs(yy) > FIELD-reach)
        for cx, cy, hx, hy in t["obstacles"]:
            blocked |= (np.abs(xx-cx) < hx+reach) & (np.abs(yy-cy) < hy+reach)
        self.blocked = blocked
        self.goal = np.array(t["goal"])
        near = (xx-self.goal[0])**2+(yy-self.goal[1])**2 < ARRIVE**2
        dist = np.full((n, n), np.inf)
        queue = deque()
        for r, c in zip(*np.nonzero(near & ~blocked)):
            dist[r, c] = 0
            queue.append((r, c))
        while queue:
            r, c = queue.popleft()
            d = dist[r, c]+1
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                rr, cc = r+dr, c+dc
                if 0 <= rr < n and 0 <= cc < n and not blocked[rr, cc] and d < dist[rr, cc]:
                    dist[rr, cc] = d
                    queue.append((rr, cc))
        self.dist = dist
        self.pulse = Pulse(t)
        self.targets = [(d[0]*t["gain"]*t["speed"], d[1]*t["gain"]*t["speed"]) for d in t["buttons"]]
        # The fastest a pulse can move the carrier: steady velocity held for the whole 0.2 s.
        self.step = self.pulse.b/(1-self.pulse.a)*t["gain"]*t["speed"]*t["timestep"]*t["substeps"]

    def cell(self, x, y):
        return (min(self.n-1, max(0, int((FIELD-y)/RES))), min(self.n-1, max(0, int((x+FIELD)/RES))))

    def free(self, x, y, offsets):
        blocked = self.blocked
        for dx, dy in offsets[1::2]:
            if blocked[self.cell(x+dx, y+dy)]:
                return False
        return True

    def plan(self, p, v):
        """Shortest button sequence from the true state (p, v) to the pad, or None."""
        gx, gy = self.goal
        frontier = [(0., 0, 0, (float(p[0]), float(p[1])), (float(v[0]), float(v[1])), None)]
        seen = {}
        expansions = pushed = 0
        while frontier and expansions < MAX_EXPANSIONS:
            _, g, _, q, w, chain = heapq.heappop(frontier)
            if (q[0]-gx)**2+(q[1]-gy)**2 < ARRIVE**2:
                buttons = []
                while chain is not None:
                    chain, button = chain
                    buttons.append(button)
                return buttons[::-1]
            key = (round(q[0]/.01), round(q[1]/.01), round(w[0]/.05), round(w[1]/.05))
            if seen.get(key, 1e9) <= g:
                continue
            seen[key] = g
            expansions += 1
            for b, vt in enumerate(self.targets):
                offsets, nw = self.pulse.run(w, vt)
                if not self.free(q[0], q[1], offsets):
                    continue
                nq = (q[0]+offsets[-1][0], q[1]+offsets[-1][1])
                h = self.dist[self.cell(*nq)]
                if not np.isfinite(h):
                    continue
                pushed += 1
                heapq.heappush(frontier, (g+1+h*RES/self.step, g+1, pushed, nq, nw, (chain, b)))
        return None


def main():
    robot = Client()
    obs = robot.observe()
    t = truth(robot)
    # Scout: the 12 actions the switch requires, back and forth along x in free space.
    plus = next(b for b, d in enumerate(t["buttons"]) if d == [1, 0])
    minus = next(b for b, d in enumerate(t["buttons"]) if d == [-1, 0])
    for k in range(12):
        obs = robot.act([(plus if k % 2 == 0 else minus)/3])
    obs = robot.advance()
    t = truth(robot)
    planner = Planner(t)
    while not obs["done"]:
        p, v = t["position"], t["velocity"]
        buttons = planner.plan(p, v)
        if not buttons:
            return 1
        for button in buttons:
            offsets, _ = planner.pulse.run(v, planner.targets[button])
            expected = (p[0]+offsets[-1][0], p[1]+offsets[-1][1])
            obs = robot.act([button/3])
            t = truth(robot)
            p, v = t["position"], t["velocity"]
            if obs["done"] or (p[0]-expected[0])**2+(p[1]-expected[1])**2 > .005**2:
                break
    return 0


if __name__ == "__main__":
    sys.exit(main())
