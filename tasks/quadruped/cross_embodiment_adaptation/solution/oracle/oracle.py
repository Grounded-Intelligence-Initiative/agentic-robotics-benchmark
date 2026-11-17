"""PRIVILEGED v4 author reference, not an image-only solution.

Uses authorized truth, never a seed or prerecorded commands. Searches rounded
oriented routes on the supplied geometry, checking the full payload sweep, then
tracks with low-speed feedback and brakes into the oriented dock. The family has
a constructive service aisle; this planner exploits that published topology.
It proves feasibility, not difficulty or image-based observability by itself.
"""
import math
import heapq
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from client import Client

TOKEN = open(os.path.join(HERE, 'token.txt')).read().strip()


def wrap(a):
    return (a+math.pi) % (2*math.pi)-math.pi


def truth(robot):
    return robot.request('truth', token=TOKEN)


def free(path, half, obstacles, margin=.065):
    """Vectorized four-axis OBB/AABB SAT over the full swept centreline."""
    path = np.asarray(path)
    c, s = np.cos(path[:, 2]), np.sin(path[:, 2])
    a, b = np.asarray(half)+margin
    for cx, cy, hx, hy in obstacles:
        dx, dy = cx-path[:, 0], cy-path[:, 1]
        collision = ((np.abs(dx) <= hx+a*np.abs(c)+b*np.abs(s)) &
                     (np.abs(dy) <= hy+a*np.abs(s)+b*np.abs(c)) &
                     (np.abs(dx*c+dy*s) <= a+hx*np.abs(c)+hy*np.abs(s)) &
                     (np.abs(-dx*s+dy*c) <= b+hx*np.abs(s)+hy*np.abs(c)))
        if collision.any():
            return False
    return True


def rounded_route(start, goal, heading, level, radius):
    n = np.array([math.cos(heading), math.sin(heading)])
    e = np.array([n[1], -n[0]])
    if np.dot(np.array(goal[:2])-start[:2], e) < 0:
        e = -e
    sx, sy = np.dot(start[:2], e), np.dot(start[:2], n)
    gx, gy = np.dot(goal[:2], e), np.dot(goal[:2], n)
    if gx-sx < 2*radius or level-radius < max(sy, gy):
        return None
    points = []
    def add(x, y, dx, dy):
        p, direction = x*e+y*n, dx*e+dy*n
        points.append((p[0], p[1], math.atan2(direction[1], direction[0])))
    def line(a, b, direction):
        count = max(1, math.ceil(math.dist(a, b)/.025))
        for k in range(count):
            p = np.array(a)+(np.array(b)-a)*k/count
            add(*p, *direction)
    line((sx, sy), (sx, level-radius), (0, 1))
    for a in np.linspace(math.pi, math.pi/2, 65):
        add(sx+radius+radius*math.cos(a), level-radius+radius*math.sin(a), math.sin(a), -math.cos(a))
    line((sx+radius, level), (gx-radius, level), (1, 0))
    for a in np.linspace(math.pi/2, 0, 65):
        add(gx-radius+radius*math.cos(a), level-radius+radius*math.sin(a), math.sin(a), -math.cos(a))
    line((gx, level-radius), (gx, gy), (0, -1))
    add(gx, gy, 0, -1)
    return np.array(points)


def plan(t, mode='full'):
    if mode in ('scout_route', 'xy'):
        return xy_route(t, .155 if mode == 'scout_route' else min(t['half']))
    start, goal = (*t['position'], t['state'][2]), (*t['goal'], t['goal_yaw'])
    obstacles = t['obstacles'] if mode != 'blind' else t['obstacles'][-4:]
    half = t['half'] if mode != 'small_body' else [.155, .155]
    if mode == 'circumscribed':
        half = [math.hypot(*t['half'])]*2
    n = np.array([math.cos(start[2]), math.sin(start[2])])
    bottom = max(np.dot(start[:2], n), np.dot(goal[:2], n))
    for level in np.arange(bottom+.9, 3.25, .035):
        for radius in (.85, .95, 1.05):
            path = rounded_route(start, goal, start[2], level, radius)
            if path is not None and free(path, half, obstacles):
                return path
    raise RuntimeError('no swept-volume route in the service-aisle family')


def xy_route(t, radius):
    """Anti-shortcut baseline: shortest inflated XY path, with no heading state."""
    res, count = .05, 152
    centers = -3.8+(np.arange(count)+.5)*res
    xx, yy = np.meshgrid(centers, centers)
    blocked = np.zeros((count, count), dtype=bool)
    for x, y, a, b in t['obstacles']:
        blocked |= (np.abs(xx-x) <= a+radius+.025) & (np.abs(yy-y) <= b+radius+.025)
    def cell(p):
        return (int((p[1]+3.8)/res), int((p[0]+3.8)/res))
    start, goal = cell(t['position']), cell(t['goal'])
    queue, costs, parents = [(0., start)], {start: 0.}, {}
    while queue:
        _, p = heapq.heappop(queue)
        if p == goal:
            route = [p]
            while p != start:
                p = parents[p]
                route.append(p)
            points = np.array([(centers[c], centers[r]) for r, c in route[::-1]])
            points[0], points[-1] = t['position'], t['goal']
            yaw = np.arctan2(np.diff(points[:, 1]), np.diff(points[:, 0]))
            return np.column_stack((points, np.r_[yaw, t['goal_yaw']]))
        for dr, dc in ((0, 1), (1, 0), (0, -1), (-1, 0)):
            q = (p[0]+dr, p[1]+dc)
            if not (0 <= q[0] < count and 0 <= q[1] < count) or blocked[q]:
                continue
            cost = costs[p]+1
            if cost < costs.get(q, float('inf')):
                costs[q], parents[q] = cost, p
                h = abs(q[0]-goal[0])+abs(q[1]-goal[1])
                heapq.heappush(queue, (cost+h, q))
    raise RuntimeError('no XY route')


class Controller:
    def __init__(self, t, mode='full'):
        self.path = plan(t, mode)
        self.progress = 0
        self.params = t['dynamics']

    def command(self, t):
        state = t['state']
        p = np.asarray(state[:2])
        yaw = state[2]
        # Monotone nearest progress prevents a turn from jumping to another leg.
        end = min(len(self.path), self.progress+90)
        near = self.progress+int(np.argmin(np.linalg.norm(self.path[self.progress:end, :2]-p, axis=1)))
        self.progress = near
        aim = near
        while aim < len(self.path)-1 and np.linalg.norm(self.path[aim, :2]-p) < .38:
            aim += 1
        target = self.path[aim, :2].copy()
        distance = np.linalg.norm(p-t['goal'])
        if aim == len(self.path)-1:
            # Extend the docking axis for heading convergence, but stop at the pad.
            target += .35*np.array([math.cos(t['goal_yaw']), math.sin(t['goal_yaw'])])
        delta = target-p
        side = -math.sin(yaw)*delta[0]+math.cos(yaw)*delta[1]
        curvature = 2*side/max(float(delta@delta), .04)
        steering = math.atan(self.params['wheelbase']*curvature)/self.params['steer_max']
        speed = .43
        if near > len(self.path)-65:
            axis = np.array([math.cos(t['goal_yaw']), math.sin(t['goal_yaw'])])
            along = float((np.array(t['goal'])-p)@axis)
            speed = max(-.12, min(speed, .85*along))
            if distance < .065 and abs(wrap(yaw-t['goal_yaw'])) < .09:
                speed, steering = 0., 0.
        return (speed/self.params['speed'], max(-1., min(1., steering)))


def channels(t, command):
    action = [0., 0.]
    for i in range(2):
        action[t['axes'][i]] = float(command[i]*t['signs'][i])
    return action


def run(robot, get_truth=None, mode='full', trace=None):
    get_truth = get_truth or (lambda: truth(robot))
    obs, t = robot.observe(), get_truth()
    plus = next(b for b, d in enumerate(t['buttons']) if d == [1, 0])
    minus = next(b for b, d in enumerate(t['buttons']) if d == [-1, 0])
    for k in range(12):
        obs = robot.act([(plus if k % 2 == 0 else minus)/3])
    obs, t = robot.advance(), get_truth()
    controller = Controller(t, mode)
    while not obs['done']:
        action = channels(t, controller.command(t))
        obs = robot.act(action)
        t = get_truth()
        if trace is not None:
            trace.append(dict(action=action, state=t['state']))
    return obs


def main():
    run(Client())
    return 0


if __name__ == '__main__':
    sys.exit(main())
