"""The reference controller's behaviour: search the seven places and retrieve the remote."""

import math
import os

import numpy as np
from scipy.ndimage import median_filter

import remote_env as E
from skills import DOWN_Y, FWD_V, Robot, down, find_remote, overview_pose, rot_from_axes

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
        self.log.append(("layout", [round(float(v), 4) for v in sofa], [round(float(v), 4) for v in cab]))
        # ... and the hat on the chest, so that the arm keeps clear of it until it is taken off.
        self.kin.use("cabinet")
        depth = median_filter(E.decode_image(first["depth"]), size=3)
        crown = self.crown(self.points("overview", depth, overview_pose(), depth > 0))
        if len(crown) >= 200:
            centre = [float(0.5 * (np.percentile(crown[:, i], 2) + np.percentile(crown[:, i], 98))) for i in (0, 1)]
            adr = self.kin.free_adr["hat"]
            world = self.kin.to_world((centre[0], centre[1], 0.0))
            self.kin.model.qpos0[adr:adr + 2] = world[:2]
            self.kin.data.qpos[adr:adr + 2] = world[:2]
        order = getattr(self, "order", ("under", "chest", "lower", "hat", "upper", "left", "right"))
        for name in order:
            if name == "under":
                found = yield from self.search_under()
            elif name == "chest":
                found = yield from self.search_under_chest()
            elif name == "hat":
                found = yield from self.search_hat()
            elif name in ("lower", "upper"):
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
        hit = yield from self.see_remote((floor + 0.007, floor + 0.03),
                                         ((x0 + 0.015, min(x0 + E.DRAWER_INNER[0] - 0.01, E.CAB_X0 - 0.01)),
                                          (CAB_CY - half_w, CAB_CY + half_w)))
        self.log.append(("drawer", name, hit))
        if hit is None:
            yield from self.close_drawer(name, rails)
            return False
        yield from self.pick(hit, floor, rails)
        return True

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
        (_, ry), _, _ = hit
        t = math.radians(UNDER_TILT)
        ahead = np.array([math.cos(t), 0.0, -math.sin(t)])
        flat = rot_from_axes(tuple(ahead), (0, 1, 0))
        grasp = np.array([E.CAB_X0 + UNDER_DEPTH, ry, UNDER_Z])
        pre = grasp - 0.22 * ahead
        # A posture for the flat hand at the floor, and a way to it: the tucked arm turns about
        # its first joint, unfolds to a pose above and behind the place, comes straight down and
        # goes straight in.
        above = pre + np.array([-0.05, 0.0, 0.30])
        plan = None
        for offset in (-0.35, 0.35, 0.2, -0.2, 0.5, -0.5):
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
    def see_remote(self, z_range, xy_box=None, cams=2, min_points=40, dark=False, sized=True):
        yield from self.wait(6)
        views = yield from self.look(cams)
        best = None
        for name, (rgb, depth, pose) in views.items():
            depth = median_filter(depth, size=5)
            mask = depth > 0
            pts = self.points(name, depth, pose, mask)
            v, u = np.nonzero(mask)
            hit = find_remote((pts, rgb[v, u].astype(int)), z_range, xy_box, min_points, dark, sized)
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
