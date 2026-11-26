"""The reference controller's behaviour: search the seven places and retrieve the remote."""

import math
import os

import numpy as np
from scipy.ndimage import label, median_filter

import remote_env as E
from skills import DOWN_X, DOWN_Y, FWD_V, Robot, down, find_remote, overview_pose, rot_from_axes

CAB_CY = E.CAB_CY
KNOB_X = E.CAB_X0 + E.KNOB_X
FRONT_X = E.CAB_X0 + E.DRAWER_FRONT_X[0]            # a closed drawer's front face
PULL = 0.35
TILT = 52
PAD_SHIFT = 0.0
FLIP = 86
DIP = 0.10
UNDER_TILT = 15                 # degrees the flat hand points down when it reaches in under the chest
UNDER_DEPTH = 0.135             # how far behind the chest's front the fingers take the remote
UNDER_Z = 0.018
HAT_LEAN = 30                   # degrees the hand leans toward the hat when it takes its brim
HAT_BITE = 0.022                # how far in from the brim's edge the fingers take it
HAT_SLIDE = 0.028               # the hand comes down this far outside that, then slides in
HAT_BRIM_Z = 0.027              # the brim's high side above the drawer's floor
HAT_OPEN = 0.045
TIP_RIM = 0.150                 # the basket's front rim before its middle
TIP_BITE = 0.028                # how far down its wall the fingers take it
TIP_LIFT = 0.30
TIP_AT = (0.42, -0.37, 0.44)    # where it is poured out: the held rim's x, its y from the chest's outer side, its height
TIP_STAND = 0.30                # the pedestal stands this far to one side of that along the rail
TIP_TURN = 125                  # degrees the hand turns
TIP_DRAW = 0.13                 # the hand draws back this far toward the rail over the last part of the pour ...
TIP_DRAW_FROM = 80              # ... from this many degrees on
TIP_ASIDE = 0.56                # the emptied basket is put down this far along the rail
UNDER_OUT = 0.26                # the remote's near end is drawn out to this far in front of the chest

class Reference(Robot):
    def run(self, obs):
        """The whole episode as a generator of actions."""
        self.obs = obs
        self.log = []
        # Where does the furniture stand? Everything after this works in the nominal layout's
        # coordinates of the piece it deals with.
        first = obs["images"]["overview"]
        sofa, cab = self.locator.locate(self.kin, E.decode_image(first["depth"]), obs["q"])
        self.layout = (sofa, cab)
        self.log.append(("layout", [round(float(v), 4) for v in sofa], [round(float(v), 4) for v in cab]))
        # ... and the box on the chest, so that the arm keeps clear of it.
        self.kin.use("cabinet")
        depth = median_filter(E.decode_image(first["depth"]), size=3)
        rim = self.box_rim(self.points("overview", depth, overview_pose(), depth > 0))
        self.box_xy = (E.CAB_X0 + E.BOX_X, CAB_CY)
        if len(rim) >= 80:
            self.box_xy = tuple(float(0.5 * (np.percentile(rim[:, i], 2) + np.percentile(rim[:, i], 98))) for i in (0, 1))
        self.log.append(("box", len(rim), [round(v, 3) for v in self.box_xy]))
        self.kin.box_xy = self.box_xy
        self.kin.set_layout(sofa, cab)
        order = getattr(self, "order", ("under", "chest", "box", "left", "right", "lower", "mid", "upper"))
        for name in order:
            if name == "under":
                found = yield from self.search_under()
            elif name == "chest":
                found = yield from self.search_under_chest()
            elif name == "box":
                found = yield from self.search_box()
            elif name in ("lower", "mid", "upper"):
                found = yield from self.search_drawer(name)
            else:
                found = yield from self.search_cushion(name)
            if found:
                return
        yield from self.wait(10)

    # -- drawers ----------------------------------------------------------------------------
    def search_drawer(self, name):
        self.kin.use("cabinet")
        kz = E.KNOB_Z[name]
        ky = CAB_CY + E.KNOB_Y[name]                 # the knob on the sofa's side
        floor = E.DRAWER_FLOOR_Z[name]
        rails = [CAB_CY + 0.55, CAB_CY + 0.45, CAB_CY - 0.45]
        self.grip_cmd = 0.05
        yield from self.move_pose((KNOB_X - 0.10, ky, kz), FWD_V, rails=rails, then=(0.10, 0, 0))
        yield from self.move_line((KNOB_X, ky, kz), step=0.006)
        yield from self.grip(0.0, steps=16)
        yield from self.move_line((KNOB_X - PULL, ky, kz), step=0.006)
        yield from self.grip(0.05, steps=10)
        self.kin.set_drawer(name, PULL)
        yield from self.move_line((KNOB_X - PULL - 0.05, ky, kz))
        yield from self.move_line((KNOB_X - PULL - 0.05, ky, kz + 0.22))
        # Look into the drawer from above.
        x0 = E.CAB_X0 + E.DRAWER_FRONT_X[1] - PULL
        view = (x0 + 0.5 * E.DRAWER_INNER[0] - 0.08, CAB_CY, floor + 0.42)
        yield from self.move_pose(view, DOWN_Y, rails=rails)
        half_w = 0.5 * E.DRAWER_INNER[1] - 0.015
        area = ((x0 + 0.015, min(x0 + E.DRAWER_INNER[0] - 0.01, E.CAB_X0 - 0.01)), (CAB_CY - half_w, CAB_CY + half_w))
        # Is the hat in this drawer? Its crown is the only thing that tall.
        tipped = False
        for attempt in range(2):
            yield from self.wait(6)
            views = yield from self.look(2)
            rgb, depth, pose = views["wrist"]
            depth = median_filter(depth, size=3)
            pts = self.points("wrist", depth, pose, depth > 0)
            crown = pts[(pts[:, 2] > floor + 0.085) & (pts[:, 2] < floor + 0.125) & (pts[:, 0] > area[0][0] + 0.03)
                        & (pts[:, 0] < area[0][1] - 0.03) & (pts[:, 1] > area[1][0] + 0.03) & (pts[:, 1] < area[1][1] - 0.03)]
            self.log.append(("hat_points", name, len(crown)))
            if len(crown) < 150:
                break
            centre = [float(0.5 * (np.percentile(crown[:, i], 2) + np.percentile(crown[:, i], 98))) for i in (0, 1)]
            yield from self.tip_hat(centre, floor)
            tipped = True
            yield from self.move_pose(view, DOWN_Y, rails=rails)
        hit = yield from self.see_remote((floor + 0.007, floor + 0.03), area)
        self.log.append(("drawer", name, hit))
        if hit is None:
            yield from self.close_drawer(name, rails)
            return False
        yield from self.pick(hit, floor, rails)
        return True

    def tip_hat(self, centre, floor):
        """Take the hat out of an open drawer. The high side of its brim is toward the drawer's
        side wall it lies farther from: the hand comes down there leaning toward the hat, one
        finger goes in under the brim and the other over it, and the hat is lifted out by its
        brim and put down on the floor beyond the chest."""
        cx, cy = centre
        e = -1.0 if cy > CAB_CY else 1.0             # the side of the hat the hand comes down on
        self.kin.use("cabinet")
        lean = math.radians(HAT_LEAN)
        a = np.array([0.0, -e * math.sin(lean), -math.cos(lean)])        # the hand's approach direction
        c = np.array([0.0, -e * math.cos(lean), math.sin(lean)])         # closing direction: the upper finger's side
        rot = rot_from_axes(a, c)
        edge = cy + e * E.HAT_BRIM[1]
        grip = np.array([cx, edge - e * HAT_BITE, floor + HAT_BRIM_Z])
        out = np.array([0.0, e, 0.0])
        pre = grip + out * HAT_SLIDE
        self.grip_cmd = HAT_OPEN
        rails = [cy + o for o in (-e * 0.25, -e * 0.35, -e * 0.15, e * 0.25, -e * 0.45, e * 0.35, 0.0, e * 0.15)]
        up = np.array([0.0, 0.0, 0.17])           # straight down between the hat and the wall
        ok = False
        for sym in (False, True):                # (either finger may be the one under the brim)
            q, ok = self.best_q(pre + up, rot @ np.diag([-1.0, -1.0, 1.0]) if sym else rot, [self.q_cmd[0]],
                                sym=False, direct=True, then=-up)
            if ok:
                if self.via is not None:
                    yield from self.move_q(self.via, speed=0.7, settle=False)
                yield from self.move_q(q, speed=0.7)
                break
        if not ok:
            ok = yield from self.move_pose(pre + up, rot, rails=rails, sym=True, then=-up)
        self.log.append(("hat_ik", ok))
        yield from self.move_line(pre, step=0.006)
        yield from self.move_line(grip, step=0.004)
        yield from self.grip(0.0, steps=18)
        yield from self.move_line(grip + np.array([0.0, 0.0, 0.30]), step=0.006)
        # ... and down to the floor beside the open drawer, on the side the pedestal stands on
        # (it cannot pass an open drawer).
        p, _ = self.kin.fk(self.q_cmd)
        base = self.kin.to_frame((0.0, self.q_cmd[0], 0.0))[1]
        side = 1.0 if base > CAB_CY else -1.0
        drop = (0.24, CAB_CY + side * 0.56)
        yield from self.move_line((drop[0], p[1], p[2]), step=0.01)
        shift = min(max(self.q_cmd[0] + drop[1] - p[1], E.RAIL_RANGE[0] + 0.01), E.RAIL_RANGE[1] - 0.01) - self.q_cmd[0]
        if side * shift > 0:
            yield from self.move_q(self.q_cmd + np.array([shift] + [0.0] * 7), speed=0.6)
        p, _ = self.kin.fk(self.q_cmd)
        if abs(drop[1] - p[1]) > 0.02:
            yield from self.move_line((p[0], drop[1], p[2]), step=0.012)
            p, _ = self.kin.fk(self.q_cmd)
        yield from self.move_line((p[0], p[1], 0.32), step=0.012)
        yield from self.grip(0.08, steps=12)
        yield from self.move_line((p[0] - 0.06, p[1], p[2]), step=0.012)

    def close_drawer(self, name, rails):
        z = E.KNOB_Z[name] + 0.045
        self.grip_cmd = 0.0
        yield from self.move_pose((FRONT_X - PULL - 0.09, CAB_CY, z), FWD_V, rails=rails, then=(0.07, 0, 0))
        yield from self.move_line((FRONT_X - 0.014, CAB_CY, z), step=0.006)
        self.kin.set_drawer(name, 0.0)
        yield from self.move_line((FRONT_X - 0.12, CAB_CY, z))
        self.grip_cmd = 0.06

    # -- the hat on the cabinet ----------------------------------------------------------
    @staticmethod
    def box_rim(pts):
        """The points of the box's walls: the only thing above the chest's top."""
        top = E.CAB_H
        return pts[(pts[:, 2] > top + 0.05) & (pts[:, 2] < top + 0.11) & (np.abs(pts[:, 1] - CAB_CY) < 0.3)
                   & (pts[:, 0] > E.CAB_X0) & (pts[:, 0] < E.CAB_X0 + E.CAB_DEPTH)]

    @staticmethod
    def crown(pts):
        """The points of the hat's crown: the only thing well above the chest's top."""
        top = E.CAB_H
        return pts[(pts[:, 2] > top + 0.09) & (pts[:, 2] < top + 0.20) & (np.abs(pts[:, 1] - CAB_CY) < 0.3)
                   & (pts[:, 0] > E.CAB_X0) & (pts[:, 0] < E.CAB_X0 + E.CAB_DEPTH)]

    def search_hat(self):
        """Take the hat off the cabinet's top and look at what was under it."""
        self.kin.use("cabinet")
        top = E.CAB_H
        rails = [CAB_CY + 0.40, CAB_CY + 0.30, CAB_CY + 0.50, CAB_CY - 0.35, CAB_CY + 0.20, CAB_CY - 0.20, CAB_CY + 0.10]
        # Where the hat is: its crown is the only thing well above the top.
        # (Seen from the overview camera, with the arm tucked out of its way.)
        self.grip_cmd = 0.0
        yield from self.go_home(self.rail(CAB_CY + 0.75))
        yield from self.wait(6)
        views = yield from self.look(1)
        rgb, depth, pose = views["overview"]
        depth = median_filter(depth, size=3)
        pts = self.points("overview", depth, pose, depth > 0)
        crown = self.crown(pts)
        self.log.append(("hat_points", len(crown), None if len(crown) < 200 else [round(float(np.percentile(crown[:, i], p)), 3) for i in (0, 1) for p in (2, 98)]))
        if len(crown) < 200:
            return False
        hx = float(0.5 * (np.percentile(crown[:, 0], 2) + np.percentile(crown[:, 0], 98)))
        hy = float(0.5 * (np.percentile(crown[:, 1], 2) + np.percentile(crown[:, 1], 98)))
        self.kin.park("hat")
        # Pinch the brim's overhanging end from the front, lift the hat off and put it down
        edge = hx - E.HAT_BRIM[0]
        grip_z = top + 0.027
        self.grip_cmd = 0.045
        yield from self.move_pose((edge - 0.10, hy, grip_z), FWD_V, rails=rails, then=(0.12, 0, 0))
        yield from self.move_line((edge + 0.022, hy, grip_z), step=0.005)
        yield from self.grip(0.0, steps=16)
        yield from self.move_line((edge + 0.022, hy, grip_z + 0.16), step=0.005)
        yield from self.move_line((edge - 0.20, hy, grip_z + 0.16), step=0.008)
        # ... on the floor beyond the cabinet, out of the way of everything else.
        drop_y = E.CAB_Y[0] - 0.33
        yield from self.move_q(self.q_cmd + np.array([drop_y - hy] + [0.0] * 7), speed=0.6)
        p_now, _ = self.kin.fk(self.q_cmd)
        yield from self.move_line((p_now[0], p_now[1], 0.30), step=0.01)
        yield from self.grip(0.08, steps=12)
        yield from self.move_line((p_now[0] - 0.08, p_now[1], 0.45), step=0.01)
        # What was under it?
        box = ((E.CAB_X0 + 0.01, E.CAB_X0 + E.CAB_DEPTH - 0.02), (CAB_CY - 0.27, CAB_CY + 0.27))
        hit = yield from self.look_down((hx, hy), top, (top + 0.007, top + 0.03), box, rails, height=0.22)
        self.log.append(("under_hat", hit))
        if hit is None:
            return False
        yield from self.pick(hit, top, rails)
        return True

    # -- the box on the cabinet ------------------------------------------------------------
    def box_things(self, top_view):
        """From a view straight down into the box: where the box stands (kept in `kin`) and the
        things in it, each (centre xy, yaw of its long axis, length, top height above the box's
        floor, number of points), the longest first."""
        import cv2
        floor = E.CAB_H + E.BOX_FLOOR_Z
        rim_z = E.CAB_H + E.BOX_H
        rgb, depth, pose = top_view
        depth = median_filter(depth, size=3)
        pts = self.points("wrist", depth, pose, depth > 0)
        near = (np.abs(pts[:, 0] - self.box_xy[0]) < 0.22) & (np.abs(pts[:, 1] - self.box_xy[1]) < 0.25)
        rim = pts[near & (pts[:, 2] > rim_z - 0.014) & (pts[:, 2] < rim_z + 0.012)]
        yaw = 0.0
        if len(rim) >= 150:
            (cx, cy), sides, angle = cv2.minAreaRect(rim[:, :2].astype(np.float32))
            if sides[0] > sides[1]:              # the long side along y
                sides, angle = (sides[1], sides[0]), angle + 90.0
            if all(abs(side - outer) < 0.02 for side, outer in zip(sides, E.BOX_OUTER)):     # the whole rim is in view
                yaw = math.radians((angle + 90.0) % 180.0 - 90.0)
                self.box_xy = (float(cx), float(cy))
                self.kin.box_xy, self.kin.box_yaw = self.box_xy, yaw
                self.kin.set_layout(*self.layout)
        c, s = math.cos(yaw), math.sin(yaw)
        local = (pts[:, :2] - np.array(self.box_xy)) @ np.array([[c, -s], [s, c]])
        inside = (np.abs(local[:, 0]) < 0.5 * E.BOX_INNER[0] - 0.012) & (np.abs(local[:, 1]) < 0.5 * E.BOX_INNER[1] - 0.012)
        sel = pts[inside & (pts[:, 2] > floor + 0.006) & (pts[:, 2] < floor + 0.10)]
        things = []
        if len(sel) >= 40:
            cell = 0.006
            ij = np.floor((sel[:, :2] - sel[:, :2].min(axis=0)) / cell).astype(int) + 1
            grid = np.zeros(ij.max(axis=0) + 2, dtype=bool)
            grid[ij[:, 0], ij[:, 1]] = True
            labels, count = label(grid, structure=np.ones((3, 3)))
            of = labels[ij[:, 0], ij[:, 1]]
            for k in range(1, count + 1):
                thing = sel[of == k]
                if len(thing) < 40:
                    continue
                xy = thing[:, :2]
                centre = xy.mean(axis=0)
                _, v = np.linalg.eigh(np.cov((xy - centre).T))
                axis, side = v[:, 1], v[:, 0]
                along, across = (xy - centre) @ axis, (xy - centre) @ side
                a0, a1 = np.percentile(along, 2), np.percentile(along, 98)
                c0, c1 = np.percentile(across, 2), np.percentile(across, 98)
                if c1 - c0 < 0.035 or a1 - a0 > 0.30:
                    continue                    # a wall of the box, not a thing in it
                things.append((centre + axis * 0.5 * (a0 + a1) + side * 0.5 * (c0 + c1), math.atan2(axis[1], axis[0]),
                               float(a1 - a0), float(np.percentile(thing[:, 2], 90) - floor), len(thing)))
        things.sort(key=lambda t: -t[2])
        return things

    def search_box(self):
        """Look into the basket on the chest. A thing lies in it that is too wide to take: the
        basket is taken by its front rim and tipped out, and the remote looked for among what
        came out."""
        self.kin.use("cabinet")
        floor = E.CAB_H + E.BOX_FLOOR_Z
        rails = [CAB_CY + 0.40, CAB_CY + 0.30, CAB_CY + 0.50, CAB_CY - 0.35, CAB_CY + 0.20, CAB_CY - 0.20, CAB_CY + 0.10]
        bx, by = self.box_xy
        self.grip_cmd = 0.08
        yield from self.move_pose((bx - 0.08, by, floor + 0.21), DOWN_Y, rails=[by + o for o in (0.2, -0.2, 0.1, -0.1, 0.3, -0.3, 0.0)])
        yield from self.wait(6)
        views = yield from self.look(2)
        things = self.box_things(views["wrist"])
        bx, by = self.box_xy
        yaw = getattr(self.kin, "box_yaw", 0.0)
        self.log.append(("box_things", [(round(float(t[0][0]), 3), round(float(t[0][1]), 3), round(t[2], 3), round(t[3], 3)) for t in things]))
        hit = yield from self.see_remote((floor + 0.007, floor + 0.03), ((bx - 0.11, bx + 0.11), (by - 0.16, by + 0.16)))
        if hit is not None:
            self.log.append(("in_box", hit))
            yield from self.pick(hit, floor, rails)
            return True
        if not things:
            return False
        # The front rim's middle, the hand over it, its fingers either side of the wall.
        c, s = math.cos(yaw), math.sin(yaw)
        rim = np.array([bx - c * TIP_RIM, by - s * TIP_RIM, E.CAB_H + E.BOX_H - TIP_BITE])
        held = down(yaw)
        self.grip_cmd = 0.05
        self.kin.park("box")
        ok = yield from self.move_pose(rim + np.array([0.0, 0.0, 0.14]), held, rails=[by + o for o in (0.25, -0.25, 0.35, -0.35, 0.15, -0.15)],
                                       sym=False, then=(0, 0, -0.14))
        self.log.append(("rim_ik", ok))
        yield from self.move_line(rim, step=0.006)
        yield from self.grip(0.0, steps=20)
        yield from self.move_line(rim + np.array([0.0, 0.0, TIP_LIFT]), step=0.004)
        # Carry it (it hangs level from the fingers) over the open floor beyond the chest, bring
        # it down to a hand's height above the floor and pour it out: the hand turns slowly
        # until the basket's far side points down, and the book and what was under it slide out.
        spot = np.array([TIP_AT[0], E.CAB_Y[0] + TIP_AT[1], TIP_AT[2]])
        shift = self.rail(spot[1] + TIP_STAND) - self.q_cmd[0]
        yield from self.move_q(self.q_cmd + np.array([shift] + [0.0] * 7), speed=0.35)
        p, _ = self.kin.fk(self.q_cmd)
        yield from self.move_line((p[0], spot[1], p[2]), step=0.006)
        yield from self.move_line(spot, step=0.006)
        _, rot0 = self.kin.fk(self.q_cmd)

        def turned(t):
            ct, st = math.cos(t), math.sin(t)
            return np.array([[ct, 0.0, st], [0.0, 1.0, 0.0], [-st, 0.0, ct]]) @ rot0

        def pouring(i):
            """The hand's place and turn at step `i` of the pour: over its last part the hand
            also draws back toward the rail, so that what leaves the basket later (the remote,
            from under the book) comes to lie clear of what left it first."""
            t = TIP_TURN * i / n
            back = TIP_DRAW * min(max((t - TIP_DRAW_FROM) / (TIP_TURN - TIP_DRAW_FROM), 0.0), 1.0)
            return spot - np.array([back, 0.0, 0.0]), turned(math.radians(t))

        n = 170
        worst = 0.0
        for i in range(1, n + 1):
            pos, rot = pouring(i)
            q, ep, er = self.kin.ik(pos, rot, self.q_cmd, rail=self.q_cmd[0], iters=60, posture=self.q_cmd)
            worst = max(worst, ep, 0.1 * er)
            self.q_cmd = q
            yield from self.tick()
        self.log.append(("tip", round(float(worst), 4)))
        # A few small shakes, for what clings in the basket's corner.
        for _ in range(3):
            for i in (n - 4, n - 8, n - 12, n - 8, n - 4, n):
                pos, rot = pouring(i)
                q, _, _ = self.kin.ik(pos, rot, self.q_cmd, rail=self.q_cmd[0], iters=60, posture=self.q_cmd)
                self.q_cmd = q
                yield from self.tick()
        yield from self.wait(20)
        # Turn the basket back, put it down on the floor farther along and let go.
        for i in range(n - 1, -1, -3):
            pos, rot = pouring(i)
            q, _, _ = self.kin.ik(pos, rot, self.q_cmd, rail=self.q_cmd[0], iters=60, posture=self.q_cmd)
            self.q_cmd = q
            yield from self.tick()
        yield from self.move_line(spot + np.array([0.0, 0.0, 0.15]), step=0.008)
        shift = max(self.q_cmd[0] - TIP_ASIDE, E.RAIL_RANGE[0] + 0.01) - self.q_cmd[0]
        yield from self.move_q(self.q_cmd + np.array([shift] + [0.0] * 7), speed=0.4)
        p, _ = self.kin.fk(self.q_cmd)
        yield from self.move_line((p[0], p[1], E.BOX_H + 0.03), step=0.01)
        yield from self.grip(0.08, steps=12)
        yield from self.move_line((p[0], p[1], 0.45), step=0.012)
        aside = (p[0] + TIP_RIM, p[1])                # the basket's middle where it now stands
        # What came out lies on the floor under where the basket's far side was. The remote may
        # lie on the book: then it is a slab above the book's cover.
        rails = [spot[1] + o for o in (0.35, 0.25, 0.45, -0.35, 0.15, -0.25)]
        box = ((0.15, 0.85), (spot[1] - 0.30, spot[1] + 0.30))
        on_book = 2.0 * E.BOOK_HALF[2]
        for cx, cy in ((spot[0] - 0.08, spot[1]), (spot[0] + 0.08, spot[1]), (spot[0] + 0.24, spot[1]), (spot[0], spot[1] - 0.16), (spot[0], spot[1] + 0.16)):
            lies_at = on_book
            hit2 = yield from self.look_down((cx, cy), 0.0, (on_book + 0.008, on_book + 0.03), box, rails, height=0.34)
            if hit2 is None:
                lies_at = 0.0
                hit2 = yield from self.see_remote((0.006, 0.03), box)
            if hit2 is None:
                # Against the book? Then it is a slab a little lower than the book's cover: the
                # closed fingers press on it and draw it clear of the book, toward the rail.
                hit2 = yield from self.see_remote((0.008, on_book - 0.0016), box, slab=True)
                if hit2 is not None:
                    (rx, ry), ryaw, _ = hit2
                    away = np.array([-math.sin(ryaw), math.cos(ryaw)])
                    if away[0] > 0:
                        away = -away
                    self.grip_cmd = 0.0
                    top = 2.0 * E.REMOTE_HALF[2]
                    yield from self.move_pose((rx, ry, top + 0.10), down(ryaw), rails=rails, then=(0, 0, -0.10))
                    yield from self.move_line((rx, ry, top - 0.004), step=0.004)
                    yield from self.move_line((rx + 0.08 * away[0], ry + 0.08 * away[1], top - 0.004), step=0.003)
                    yield from self.move_line((rx + 0.08 * away[0], ry + 0.08 * away[1], top + 0.12), step=0.008)
                    hit2 = yield from self.look_down((rx + 0.08 * away[0], ry + 0.08 * away[1]), 0.0, (0.006, 0.03), box, rails, height=0.34)
            self.log.append(("tipped_out", hit2, lies_at))
            if hit2 is not None:
                yield from self.pick(hit2, lies_at, [hit2[0][1] + o for o in (0.35, -0.35, 0.45, -0.45, 0.25, -0.25)])
                return True
        # Not on the floor: did it stay in the basket?
        inside = ((aside[0] - 0.10, aside[0] + 0.10), (aside[1] - 0.15, aside[1] + 0.15))
        hit2 = yield from self.look_down(aside, E.BOX_FLOOR_Z, (E.BOX_FLOOR_Z + 0.007, E.BOX_FLOOR_Z + 0.03), inside,
                                         [aside[1] + o for o in (0.30, 0.20, 0.40, 0.10)], height=0.36)
        self.log.append(("left_in", hit2))
        if hit2 is not None:
            yield from self.pick(hit2, E.BOX_FLOOR_Z, [hit2[0][1] + o for o in (0.30, 0.20, 0.40, 0.10)])
            return True
        return False

    # -- seat pads ------------------------------------------------------------------------
    def pad_grip_pose(self, y, theta, shift=0.0):
        """Hand pose that pinches a pad's front edge when the pad, drawn forward by `shift`, is
        tilted up by `theta` about its rear bottom edge."""
        pivot = np.array([E.SOFA_X0 - E.PAD_OVERHANG + E.PAD_SIZE[0] - shift, y, E.DECK_Z])
        c, s_ = math.cos(theta), math.sin(theta)
        along, up = -(E.PAD_SIZE[0] - 0.018), 0.5 * E.PAD_SIZE[2]
        pos = pivot + np.array([c * along - s_ * up, 0.0, -s_ * along + c * up])
        rot = rot_from_axes((c, 0, -s_), (s_, 0, c))
        return pos, rot

    def tilt_pad(self, y, a, b, n=28):
        for i in range(1, n + 1):
            pos, rot = self.pad_grip_pose(y, a + (b - a) * i / n, PAD_SHIFT)
            q, _, _ = self.kin.ik(pos, rot, self.q_cmd, rail=self.q_cmd[0], iters=40, posture=self.q_cmd)
            self.q_cmd = q
            yield from self.tick()
        yield from self.settle()

    def search_cushion(self, side):
        self.kin.use("sofa")
        y = E.PAD_Y[side]
        other = E.PAD_Y["right" if side == "left" else "left"]
        rails = [y + 0.45, y + 0.30, y - 0.30] if side == "left" else [y - 0.45, y - 0.30, y + 0.30]
        tilt = math.radians(TILT)
        pos, rot = self.pad_grip_pose(y, 0.0)
        self.grip_cmd = 0.08
        yield from self.move_pose(pos - np.array([0.12, 0, 0]), rot, rails=rails, sym=False, then=(0.115, 0, 0))
        yield from self.move_line(pos, step=0.006)
        yield from self.grip(0.0, steps=18)
        self.kin.park("pad_" + side)
        # Draw the pad forward a little first: tilted, its rear edge needs room before the backrest.
        yield from self.move_line(self.pad_grip_pose(y, 0.0, PAD_SHIFT)[0], step=0.004)
        yield from self.tilt_pad(y, 0.0, tilt)
        half = 0.5 * E.PAD_SIZE[1] - 0.03
        box = ((E.SOFA_X0 + 0.03, E.SOFA_X0 + 0.385), (y - half, y + half))
        band = (E.DECK_Z + 0.007, E.DECK_Z + 0.03)
        hit = yield from self.see_remote((E.DECK_Z - 0.01, E.DECK_Z + 0.04), box, cams=1, min_points=20, dark=True)
        self.log.append(("cushion", side, hit))
        if hit is None:
            yield from self.tilt_pad(y, tilt, 0.0)
            yield from self.grip(0.08, steps=12)
            self.kin.unpark("pad_" + side)
            yield from self.move_line(pos - np.array([0.10, 0, 0]))
            return False
        # Lift the pad by its front edge, carry it over to the other seat and lay it on the
        # other pad.
        p0, r0 = self.pad_grip_pose(y, tilt, PAD_SHIFT)
        lift = np.array([-0.06, 0.0, 0.16])
        yield from self.move_line(p0 + np.array([0.0, 0.0, 0.16]), step=0.004)
        yield from self.move_line(p0 + lift, step=0.006)
        shift = other - y
        yield from self.move_q(self.q_cmd + np.array([shift] + [0.0] * 7), speed=0.5)
        yield from self.move_line(p0 + lift + np.array([0.0, shift, -0.10]), step=0.006)
        yield from self.grip(0.08, steps=12)
        p_now, _ = self.kin.fk(self.q_cmd)
        yield from self.move_line(p_now + np.array([-0.12, 0.0, 0.10]))
        rails = [y + o for o in (-0.3, 0.3, -0.45, 0.45, 0.0)]
        hit2 = yield from self.look_down((E.SOFA_X0 + 0.22, y), E.DECK_Z, band, box, rails)
        self.log.append(("deck", side, hit2))
        if hit2 is None:
            return False
        yield from self.pick(hit2, E.DECK_Z, rails)
        return True

    # -- under the sofa -------------------------------------------------------------------
    def search_under(self):
        """Look under the sofa with the wrist camera held low; if the remote is there, fetch the
        hanger and sweep the remote out."""
        self.kin.use("sofa")
        low = rot_from_axes((1, 0, 0), (0, -1, 0))       # camera on the hand's lower side
        best = None
        for y in (E.SEAT_MID_Y + 0.25, E.SEAT_MID_Y - 0.25):
            rails = [y + 0.5, y - 0.5, y + 0.4, y - 0.4]
            self.grip_cmd = 0.0
            # Down in the clear floor strip beside the rail, then forward to the sofa.
            ok = yield from self.move_pose((0.10, y, 0.40), low, rails=rails, sym=False, then=(0.07, 0, -0.295))
            yield from self.move_line((0.10, y, 0.105), step=0.008)
            yield from self.move_line((0.17, y, 0.105), step=0.008)
            hit = yield from self.see_remote((0.004, 0.035), ((E.SOFA_X0 + 0.04, E.SOFA_X0 + 0.34),
                                                              (E.SOFA_Y[0] + 0.10, E.SOFA_Y[1] - 0.10)), min_points=25, sized=False)
            self.log.append(("under", y, ok, hit))
            if hit is not None and (best is None or hit[2] > best[2]):
                best = hit
            yield from self.move_line((0.10, y, 0.105), step=0.008)
            yield from self.move_line((0.10, y, 0.40), step=0.008)
        if best is None:
            return False
        yield from self.fetch_hanger()
        self.kin.use("sofa")
        yield from self.sweep(best)
        return True

    def search_under_chest(self):
        """Look under the chest with the wrist camera held low; if the remote is there, reach in
        for it."""
        self.kin.use("cabinet")
        low = rot_from_axes((1, 0, 0), (0, -1, 0))       # camera on the hand's lower side
        y = CAB_CY
        rails = [y + 0.5, y - 0.5, y + 0.4, y - 0.4]
        self.grip_cmd = 0.0
        ok = yield from self.move_pose((0.10, y, 0.40), low, rails=rails, sym=False, then=(0.07, 0, -0.31))
        yield from self.move_line((0.10, y, 0.09), step=0.008)
        yield from self.move_line((0.17, y, 0.09), step=0.008)
        hit = yield from self.see_remote((0.004, 0.035), ((E.CAB_X0 + 0.06, E.CAB_X0 + 0.34),
                                                          (CAB_CY - 0.22, CAB_CY + 0.22)), min_points=25, sized=False)
        self.log.append(("chest", ok, hit))
        yield from self.move_line((0.10, y, 0.09), step=0.008)
        yield from self.move_line((0.10, y, 0.40), step=0.008)
        if hit is None:
            return False
        found = yield from self.reach_under(hit)
        return found

    def reach_under(self, hit):
        """The remote lies under the chest, pointing in. The carcass is high enough for the hand
        lying flat: it goes in between the legs, fingers first and a little nose down, takes the
        remote's near end, draws it out along the floor and lets go; then the remote is picked
        up from above like any other on the floor."""
        (rx, ry), yaw, _ = hit
        # The remote does not point straight in: its near end is a little to one side of its
        # middle. (The low view gives its direction only roughly: the slope is bounded.)
        slope = math.tan(yaw)
        ry = ry + min(max(slope, -0.25), 0.25) * (E.CAB_X0 + UNDER_DEPTH - rx)
        t = math.radians(UNDER_TILT)
        ahead = np.array([math.cos(t), 0.0, -math.sin(t)])
        flat = rot_from_axes(tuple(ahead), (0, 1, 0))
        grasp = np.array([E.CAB_X0 + UNDER_DEPTH, ry, UNDER_Z])
        pre = grasp - 0.22 * ahead
        # A posture for the flat hand at the floor, and a way to it: the tucked arm turns about
        # its first joint, unfolds to a pose above and behind the place, comes straight down and
        # goes straight in.
        plan = None
        for offset, back in [(o, k) for k in (-0.05, -0.10, 0.0) for o in (-0.35, 0.35, 0.2, -0.2, 0.5, -0.5, 0.42, -0.42, 0.28, -0.28)]:
            above = pre + np.array([back, 0.0, 0.30])
            rail = self.rail(ry + offset)
            home = self.home_at(rail)
            for rot in (flat @ np.diag([-1.0, -1.0, 1.0]), flat):
                for j1 in (-1.3, -0.8, -0.3, 0.3, 0.8, 1.3):
                    for j7 in (0.785, -0.785, 2.3, -2.3):
                        seed = self.kin.home.copy()
                        seed[1], seed[7] = j1, j7
                        q_pre, ep, er = self.kin.ik(pre, rot, seed, rail=rail, iters=300)
                        if ep > 2e-3 or er > 2e-2 or self.kin.collides(q_pre, 0.08):
                            continue
                        q_in, ep, er = self.kin.ik(grasp, rot, q_pre, rail=rail, iters=300, posture=q_pre)
                        if ep > 2e-3 or er > 2e-2 or self.kin.collides(q_in, 0.08):
                            continue
                        q_up, ep, er = self.kin.ik(above, rot, q_pre, rail=rail, iters=300, posture=q_pre)
                        if ep > 2e-3 or er > 2e-2 or self.kin.collides(q_up, 0.08):
                            continue
                        q_mid = home.copy()
                        q_mid[1] = q_up[1]
                        if (self.path_free(home, q_mid, clearance=0.0) and self.path_free(q_mid, q_up, clearance=0.0)
                                and self.path_free(q_up, q_pre, clearance=0.0)):
                            plan = (rail, q_mid, q_up, rot)
                            break
                    if plan:
                        break
                if plan:
                    break
            if plan:
                break
        self.log.append(("reach_plan", plan is not None))
        if plan is None:
            return False
        rail, q_mid, q_up, flat = plan
        yield from self.go_home(rail)
        self.grip_cmd = 0.08
        yield from self.move_q(q_mid, speed=0.6, settle=False)
        yield from self.move_q(q_up, speed=0.6)
        yield from self.move_line(pre, flat, step=0.008)
        yield from self.move_line(grasp, step=0.005)
        yield from self.grip(0.0, steps=18)
        out = E.CAB_X0 - UNDER_OUT
        yield from self.move_line((out + UNDER_DEPTH, ry, UNDER_Z + 0.004), step=0.005)
        yield from self.grip(0.08, steps=12)
        yield from self.move_line((out + UNDER_DEPTH - 0.10, ry, 0.10), step=0.008)
        yield from self.move_line((out + UNDER_DEPTH - 0.10, ry, 0.35), step=0.01)
        rails = [ry + o for o in (-0.4, 0.4, -0.3, 0.3, -0.5, 0.5)]
        box = ((0.10, E.CAB_X0 - 0.005), (ry - 0.25, ry + 0.25))
        hit2 = yield from self.look_down((out + UNDER_DEPTH + 0.06, ry), 0.0, (0.006, 0.035), box, rails, height=0.50, dark=True)
        self.log.append(("floor", hit2))
        if hit2 is None:
            return False
        # From where the arm looked down, without tucking it first: the place is close to the
        # chest's front and under the hat's brim.
        yield from self.pick(hit2, 0.0, rails, drag=0.13, direct=True)
        return True

    def fetch_hanger(self):
        self.kin.use("cabinet")
        y_plane = E.CAB_Y[0] - E.HANG_OFFSET
        board_z = E.PEG_Z - E.HOOK_SEAT[2] + 0.004
        grab_x = E.PEG_X - E.HOOK_SEAT[0] - E.HANGER_BOARD[0] + 0.022
        side = rot_from_axes((0, 1, 0), (0, 0, 1))        # hand pointing at the cabinet's side
        self.grip_cmd = 0.08
        rails = [y_plane - o for o in (0.30, 0.20, 0.40, 0.10)]
        yield from self.move_pose((grab_x, y_plane - 0.13, board_z), side, rails=rails, sym=False, then=(0, 0.13, 0))
        yield from self.move_line((grab_x, y_plane + 0.003, board_z), step=0.006)
        yield from self.grip(0.0, steps=18)
        yield from self.move_line((grab_x, y_plane, board_z + 0.012), step=0.004)
        yield from self.move_line((grab_x, y_plane - 0.12, board_z + 0.012), step=0.006)
        # Turn the hand down: the hanger now lies flat, pointing along +x from the hand.
        yield from self.move_line((0.26, y_plane - 0.12, 1.00), DOWN_Y, step=0.01)
        self.holding = True

    def sweep(self, hit):
        (rx, ry), _, _ = hit
        hy = min(max(ry, E.SEAT_MID_Y - 0.05), E.SEAT_MID_Y + 0.07)

        def hx(yaw):
            # The hand is 0.2 m wide along its closing direction and 0.064 m across it: it
            # stands off the pads' front edges by its own half extent toward the sofa, nearer
            # while the hanger points in under the sofa, so that the remote meets the hanger
            # well inside its far end.
            return E.SOFA_X0 - E.PAD_OVERHANG - 0.015 - (0.1 * abs(math.cos(yaw)) + 0.032 * abs(math.sin(yaw)))

        z = 0.016
        start = math.pi

        def held(yaw):
            # The hand leans a little about its closing direction, so that the hanger's far end
            # is the low one: it then slides on the floor and cannot ride over the remote.
            c, s_ = math.cos(DIP), math.sin(DIP)
            return down(yaw) @ np.array([[1.0, 0.0, 0.0], [0.0, c, -s_], [0.0, s_, c]])

        # Where to stand: the first rail position from which the whole sweep is within reach.
        # Come down well clear of the pads' front edges (the wrist is wider than the hand), then
        # slide in along the floor.
        back = E.SOFA_X0 - 0.17
        plan = None
        for offset in (-0.35, -0.40, -0.45, -0.50, -0.30):
            rail = self.rail(hy + offset)
            for j1 in (0.8, 0.3, -0.8):
                seed = self.kin.home.copy()
                seed[1] = j1
                q_floor, ep, er = self.kin.ik((back, hy, z), held(start), seed, rail=rail)
                if ep < 2e-3 and er < 2e-2:
                    break
            else:
                continue
            q_high, ep, er = self.kin.ik((back, hy, 0.80), held(start), q_floor, rail=rail, posture=q_floor)
            worst = max(ep, 0.1 * er)
            q = q_floor
            for i in range(0, 241, 8):
                yaw = start - math.pi * i / 240
                q, ep, er = self.kin.ik((hx(yaw), hy, z), held(yaw), q, rail=rail, iters=60, posture=q)
                worst = max(worst, ep, 0.1 * er)
            if plan is None or worst < plan[0]:
                plan = (worst, rail, q_floor, q_high)
            if worst < 3e-3:
                break
        _, rail, q_floor, q_high = plan
        self.log.append(("sweep_from", round(rail, 3), round(plan[0], 4)))
        # Carry the hanger over (rail only), turn it to point along +y, bring it down to the floor.
        yield from self.move_q(self.q_cmd + np.array([rail - self.q_cmd[0]] + [0.0] * 7), speed=0.6)
        yield from self.move_q(q_high, speed=0.5)
        yield from self.move_line((back, hy, z), step=0.008)
        yield from self.move_line((hx(start), hy, z), step=0.006)
        # Sweep clockwise (seen from above): the hanger turns about the hand from +y through +x
        # (in under the sofa) to -y, and the remote goes ahead of it.
        n = 240
        for i in range(1, n + 1):
            yaw = start - math.radians(180.0) * i / n
            q, _, _ = self.kin.ik((hx(yaw), hy, z), held(yaw), self.q_cmd, rail=rail, iters=40, posture=self.q_cmd)
            self.q_cmd = q
            yield from self.tick()
        yield from self.settle()
        hx = hx(yaw)
        # Put the hanger aside and pick the remote up from the floor.
        yield from self.move_line((hx - 0.05, hy, 0.25), step=0.008)
        yield from self.move_line((hx - 0.08, hy + 0.40, 0.10), step=0.008)
        yield from self.grip(0.08, steps=12)
        self.holding = False
        yield from self.move_line((hx - 0.08, hy + 0.40, 0.35), step=0.01)
        rails = [hy - 0.7, hy - 0.6, hy - 0.8, hy - 0.5]
        box = ((0.10, E.SOFA_X0 - 0.01), (hy - 0.75, hy + 0.1))
        hit2 = yield from self.look_down((0.26, hy - 0.33), 0.0, (0.006, 0.035), box, rails, height=0.50, dark=True)
        self.log.append(("floor", hit2))
        if hit2 is None:
            return False
        yield from self.pick(hit2, 0.0, [hit2[0][1] + o for o in (-0.35, 0.35, -0.45, 0.45, -0.25, 0.25)])
        return True

    # -- seeing and picking -----------------------------------------------------------------
    def see_remote(self, z_range, xy_box=None, cams=2, min_points=40, dark=False, sized=True, slab=False):
        yield from self.wait(6)
        views = yield from self.look(cams)
        best = None
        for name, (rgb, depth, pose) in views.items():
            depth = median_filter(depth, size=5)
            mask = depth > 0
            pts = self.points(name, depth, pose, mask)
            v, u = np.nonzero(mask)
            hit = find_remote((pts, rgb[v, u].astype(int)), z_range, xy_box, min_points, dark, sized, slab)
            if hit is not None and (best is None or hit[2] > best[2]):
                best = hit
        return best

    def look_down(self, centre, surface_z, z_range, xy_box, rails, height=0.40, dark=False):
        """Look straight down at a surface with the wrist camera and find the remote on it."""
        yield from self.move_pose((centre[0] - 0.06, centre[1], surface_z + height), DOWN_Y, rails=rails)
        hit = yield from self.see_remote(z_range, xy_box, dark=dark)
        return hit

    def pick(self, hit, surface_z, rails, lift=0.16, drag=0.0, direct=False):
        """Top-down pick of the remote at `hit` (centre, long-axis yaw) lying on `surface_z`. With
        `drag`, the held remote is first drawn that far toward the robot just above the surface
        (part of it may still be under something)."""
        (cx, cy), yaw, _ = hit
        rot = down(yaw + 0.5 * math.pi)
        self.grip_cmd = 0.075
        ok = yield from self.move_pose((cx, cy, surface_z + 0.13), rot, rails=rails, then=(0, 0, -0.118), direct=direct)
        self.log.append(("pick_ik", ok))
        yield from self.move_line((cx, cy, surface_z + 0.012), step=0.006)
        yield from self.grip(0.0, steps=18)
        if drag:
            cx = cx - drag
            yield from self.move_line((cx, cy, surface_z + 0.016), step=0.005)
        yield from self.move_line((cx, cy, surface_z + lift), step=0.006)
        yield from self.wait(E.HOLD_STEPS + 6)
        return ok
