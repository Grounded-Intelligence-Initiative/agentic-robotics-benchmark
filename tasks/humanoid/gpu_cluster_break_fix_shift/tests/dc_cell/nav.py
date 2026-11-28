"""The robot's navigation stack (a G1 deployed on a site runs one on its lidar map): `nav_goal [x, y, yaw]` plans a
path for the base on a 5 cm occupancy grid of the hall and walks it.

The grid holds every collider the robot's body can meet between 3 cm and 1.7 m above the floor: the racks and the
mounted equipment, the walls, the cart, the tote, the bench, the shelves (static, rasterised once), and the moving
fixtures as they stand when the plan is made (an extended tray and its levers, an open rear door). The cold
aisle's doors are automatic and never block a plan. Obstacles are inflated by the robot's footprint: the base's centre
never comes closer than R_HARD to one, and keeps clear up to R_SOFT where it can (a cost). The path (A* on the grid,
then shortcut to straight segments) is followed with a look-ahead point; the robot faces along the path, and walks
sideways (its depth across the passage) where a segment's clearance is less than NARROW, as next to an extended tray.
The arms and whatever the hands hold are NOT in the plan: tuck the arms before walking.

Status (get_state()["robot"]["nav"]): idle | moving | arrived | blocked (no progress for BLOCKED_S) | no_path.
A base_goal or base_velocity command cancels a running nav goal.
"""

from __future__ import annotations

import heapq
import math
import os

import mujoco
import numpy as np
from scipy import ndimage

from . import hall as H

RES = 0.05
R_HARD = 0.24
R_SOFT = 0.45
SOFT_W = 6.0
NARROW = 0.40
LOOKAHEAD = 0.35
LOOKAHEAD_NARROW = 0.15       # in a narrow passage the look-ahead point stays close: corners are not cut
ARRIVE = (0.012, 0.03)
BLOCKED_S = 4.0
Z_BAND = (0.03, 1.70)
ENDPOINT_FREE = 0.30          # m around the start and the goal where the hard inflation does not block


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Nav:
    def __init__(self, m, d, skip_prefixes=("g1/", "px_", "tool_", "chain_", "anc_", "cdoor_")):
        self.m, self.d = m, d
        x0, x1 = H.HALL["x"]
        y0, y1 = H.HALL["y"]
        self.ox, self.oy = x0, y0
        self.nx, self.ny = int(math.ceil((x1 - x0) / RES)) + 1, int(math.ceil((y1 - y0) / RES)) + 1
        self.static_g, self.dynamic_g = [], []
        for g in range(m.ngeom):
            if not (m.geom_contype[g] or m.geom_conaffinity[g]) or m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
                continue
            b = int(m.geom_bodyid[g])
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or ""
            if b == 0 and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
                continue
            if any(name.startswith(p) for p in skip_prefixes) or m.body_mocapid[b] >= 0:
                continue
            (self.dynamic_g if self._moves(b) else self.static_g).append(g)
        self.static_occ = self._raster(self.static_g)
        # the cold aisle's automatic doors: planned as standing fully open (their leaves beside the doorways), so a
        # path out of a doorway keeps clear of the open leaves' edges
        door_g = [g for g in range(m.ngeom) if (m.geom_contype[g] or m.geom_conaffinity[g]) and
                  (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[g])) or "").startswith("cdoor_")]
        if door_g and not os.environ.get("ARB_NAV_NO_DOORS"):
            dd = mujoco.MjData(m)
            dd.qpos[:] = d.qpos
            for j in range(m.njnt):
                if (m.joint(j).name or "").startswith("cdoor_"):
                    dd.qpos[m.jnt_qposadr[j]] = m.jnt_range[j][1]
            mujoco.mj_kinematics(m, dd)
            self.static_occ |= self._raster(door_g, dd)
        self.path = None
        self.narrow = None
        self.status = "idle"
        self.goal = None
        self.seg = 0
        self.headings = []
        self.best_progress = (0.0, 0.0)

    def _moves(self, b):
        m = self.m
        while b > 0:
            if m.body_jntnum[b] > 0 or m.body_mocapid[b] >= 0:
                return True
            b = int(m.body_parentid[b])
        return False

    # ------------------------------------------------------------------------------------------------ the grid
    def _cell(self, x, y):
        return int(round((x - self.ox) / RES)), int(round((y - self.oy) / RES))

    def _xy(self, i, j):
        return self.ox + i * RES, self.oy + j * RES

    def _raster(self, geoms, data=None):
        occ = np.zeros((self.nx, self.ny), dtype=bool)
        m, d = self.m, (self.d if data is None else data)
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
        for g in geoms:
            c, h = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
            R = d.geom_xmat[g].reshape(3, 3)
            pts = d.geom_xpos[g] + (c + corners * h) @ R.T
            lo, hi = pts.min(0), pts.max(0)
            if hi[2] < Z_BAND[0] or lo[2] > Z_BAND[1]:
                continue
            i0, j0 = self._cell(lo[0], lo[1])
            i1, j1 = self._cell(hi[0], hi[1])
            i0, j0, i1, j1 = max(i0, 0), max(j0, 0), min(i1, self.nx - 1), min(j1, self.ny - 1)
            if i1 >= i0 and j1 >= j0:
                occ[i0:i1 + 1, j0:j1 + 1] = True
        return occ

    def occupancy(self):
        return self.static_occ | self._raster(self.dynamic_g)

    # ------------------------------------------------------------------------------------------------ planning
    def plan(self, start, goal):
        """Waypoints [(x, y)] from start (x, y) to goal (x, y), or None."""
        occ = self.occupancy()
        dist = ndimage.distance_transform_edt(~occ) * RES
        si, sj = self._cell(*start)
        gi, gj = self._cell(*goal)
        if not (0 <= si < self.nx and 0 <= sj < self.ny and 0 <= gi < self.nx and 0 <= gj < self.ny):
            return None, dist
        if occ[gi, gj]:
            return None, dist
        ii, jj = np.meshgrid(np.arange(self.nx), np.arange(self.ny), indexing="ij")
        near_end = (np.hypot(ii - si, jj - sj) * RES < ENDPOINT_FREE) | (np.hypot(ii - gi, jj - gj) * RES < ENDPOINT_FREE)
        blocked = occ | ((dist < R_HARD) & ~near_end)
        cost = 1.0 + SOFT_W * np.clip((R_SOFT - dist) / (R_SOFT - R_HARD), 0.0, 1.0)
        steps = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, 1.4142), (1, -1, 1.4142),
                 (-1, 1, 1.4142), (-1, -1, 1.4142)]
        g_best = {(si, sj): 0.0}
        came = {}
        heap = [(0.0, 0.0, si, sj)]
        found = False
        while heap:
            f, gc, i, j = heapq.heappop(heap)
            if (i, j) == (gi, gj):
                found = True
                break
            if gc > g_best.get((i, j), 1e18) + 1e-9:
                continue
            for di, dj, w in steps:
                a, b = i + di, j + dj
                if a < 0 or b < 0 or a >= self.nx or b >= self.ny or blocked[a, b]:
                    continue
                ng = gc + w * cost[a, b]
                if ng < g_best.get((a, b), 1e18):
                    g_best[(a, b)] = ng
                    came[(a, b)] = (i, j)
                    heapq.heappush(heap, (ng + math.hypot(gi - a, gj - b), ng, a, b))
        if not found:
            return None, dist
        cells = [(gi, gj)]
        while cells[-1] != (si, sj):
            cells.append(came[cells[-1]])
        cells.reverse()
        # shortcut: the farthest cell in a straight line that keeps the hard clearance (or is near an end) and gives
        # up no clearance the searched path had (up to R_SOFT): a shortcut never cuts a corner closer to an obstacle
        ok = ~blocked
        cd = [float(dist[i, j]) for i, j in cells]
        pts = [cells[0]]
        k = 0
        while k < len(cells) - 1:
            best = k + 1
            for n in range(len(cells) - 1, k + 1, -1):
                need = min(min(cd[k:n + 1]), R_SOFT) - 0.5 * RES
                if self._line_ok(cells[k], cells[n], ok) and self._line_min(cells[k], cells[n], dist, near_end) >= need:
                    best = n
                    break
            pts.append(cells[best])
            k = best
        way = [self._xy(i, j) for i, j in pts]
        if len(way) < 2:                          # start and goal in one cell: one short segment
            return [tuple(start), tuple(goal)], dist
        way[0] = tuple(start)
        way[-1] = tuple(goal)
        return way, dist

    def _line_ok(self, a, b, ok):
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) + 1
        for t in np.linspace(0.0, 1.0, n + 1):
            i = int(round(a[0] + (b[0] - a[0]) * t))
            j = int(round(a[1] + (b[1] - a[1]) * t))
            if not ok[i, j]:
                return False
        return True

    def _line_min(self, a, b, dist, near_end):
        """The smallest clearance along a straight line of cells (cells near the start or the goal not counted)."""
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) + 1
        c = 1e9
        for t in np.linspace(0.0, 1.0, n + 1):
            i = int(round(a[0] + (b[0] - a[0]) * t))
            j = int(round(a[1] + (b[1] - a[1]) * t))
            if not near_end[i, j]:
                c = min(c, float(dist[i, j]))
        return c

    def _side_room(self, p, q, dist, sgn):
        """The clearance a little to one side (sgn +1: left of p -> q) along a segment."""
        h = math.atan2(q[1] - p[1], q[0] - p[0])
        ox, oy = -math.sin(h) * sgn * 0.15, math.cos(h) * sgn * 0.15
        return self._clearance((p[0] + ox, p[1] + oy), (q[0] + ox, q[1] + oy), dist)

    def _clearance(self, p, q, dist):
        n = max(2, int(math.hypot(q[0] - p[0], q[1] - p[1]) / RES) + 1)
        c = 1e9
        for t in np.linspace(0.0, 1.0, n):
            i, j = self._cell(p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t)
            if 0 <= i < self.nx and 0 <= j < self.ny:
                c = min(c, float(dist[i, j]))
        return c

    # ------------------------------------------------------------------------------------------------ following
    def start(self, goal, pose):
        """Plan from the robot's pose (x, y, yaw) to goal (x, y, yaw); returns a summary dict."""
        way, dist = self.plan((float(pose[0]), float(pose[1])), (float(goal[0]), float(goal[1])))
        self.goal = [float(v) for v in goal]
        if way is None:
            self.path, self.status = None, "no_path"
            return {"ok": False, "error": "no_path"}
        self.path = way
        self.seg = 0
        self.headings = []
        self.narrow = [self._clearance(p, q, dist) < NARROW for p, q in zip(way[:-1], way[1:])]
        yaw = float(pose[2])
        for p, q in zip(way[:-1], way[1:]):
            h = math.atan2(q[1] - p[1], q[0] - p[0])
            if self._clearance(p, q, dist) < NARROW and math.hypot(q[0] - p[0], q[1] - p[1]) > 0.10:
                # sideways: the robot's depth across the passage, facing the side with more room (the arms reach
                # forward); when both sides are alike, the turn nearer the present heading
                c1, c2 = _wrap(h + math.pi / 2), _wrap(h - math.pi / 2)
                r1, r2 = self._side_room(p, q, dist, 1.0), self._side_room(p, q, dist, -1.0)
                if abs(r1 - r2) > 0.04:
                    h = c1 if r1 > r2 else c2
                else:
                    h = c1 if abs(_wrap(c1 - yaw)) <= abs(_wrap(c2 - yaw)) else c2
            self.headings.append(h)
            yaw = h
        self.status = "moving"
        self.best_progress = (0.0, 0.0)
        self.t_start = None
        length = sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(way[:-1], way[1:]))
        return {"ok": True, "length_m": round(length, 3), "waypoints": [[round(x, 3), round(y, 3)] for x, y in way]}

    def cancel(self):
        if self.status == "moving":
            self.status = "idle"
        self.path = None

    def _remaining(self, xy):
        """(index of the segment, the projection's distance along the path, the path's length)."""
        best = None
        acc = 0.0
        total = 0.0
        for k, (p, q) in enumerate(zip(self.path[:-1], self.path[1:])):
            v = np.subtract(q, p)
            L = float(np.linalg.norm(v))
            total += L
        acc = 0.0
        for k, (p, q) in enumerate(zip(self.path[:-1], self.path[1:])):
            v = np.subtract(q, p)
            L = float(np.linalg.norm(v))
            if k < self.seg:
                acc += L
                continue
            t = 0.0 if L < 1e-9 else float(np.clip(np.dot(np.subtract(xy, p), v) / (L * L), 0.0, 1.0))
            dd = float(np.linalg.norm(np.add(p, t * v) - np.asarray(xy)))
            if best is None or dd < best[0] - 1e-6:
                best = (dd, k, acc + t * L)
            acc += L
        return best[1], best[2], total

    def _point_at(self, s):
        acc = 0.0
        for k, (p, q) in enumerate(zip(self.path[:-1], self.path[1:])):
            L = math.hypot(q[0] - p[0], q[1] - p[1])
            if acc + L >= s or k == len(self.path) - 2:
                t = 0.0 if L < 1e-9 else min(1.0, max(0.0, (s - acc) / L))
                return (p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])), k
            acc += L
        return self.path[-1], len(self.path) - 2

    def tick(self, t, pose, ctrl):
        """One control tick: the controller's base goal set to the look-ahead point (or the final goal)."""
        if self.status != "moving" or self.path is None:
            return
        xy = (float(pose[0]), float(pose[1]))
        k, s, total = self._remaining(xy)
        self.seg = max(self.seg, k)
        if self.t_start is None:
            self.t_start = t
            self.best_progress = (s, t)
        if s > self.best_progress[0] + 0.02:
            self.best_progress = (s, t)
        elif t - self.best_progress[1] > BLOCKED_S and total - s > 0.05:
            self.status = "blocked"
            ctrl.goal = np.array([pose[0], pose[1], pose[2]], float)
            return
        if total - s <= LOOKAHEAD:
            gx, gy, gyaw = self.goal
            ctrl.goal = np.array([gx, gy, gyaw], float)
            if math.hypot(gx - xy[0], gy - xy[1]) < ARRIVE[0] and abs(_wrap(gyaw - float(pose[2]))) < ARRIVE[1]:
                self.status = "arrived"
            return
        la = LOOKAHEAD if self.narrow is None or not self.narrow[min(k, len(self.narrow) - 1)] else LOOKAHEAD_NARROW
        (cx, cy), kc = self._point_at(s + la)
        ctrl.goal = np.array([cx, cy, self.headings[kc]], float)

    def state(self, pose=None):
        out = {"status": self.status}
        if self.goal is not None:
            out["goal"] = [round(v, 4) for v in self.goal]
        if self.path is not None and pose is not None and self.status == "moving":
            k, s, total = self._remaining((float(pose[0]), float(pose[1])))
            out["remaining_m"] = round(total - s, 3)
        return out
