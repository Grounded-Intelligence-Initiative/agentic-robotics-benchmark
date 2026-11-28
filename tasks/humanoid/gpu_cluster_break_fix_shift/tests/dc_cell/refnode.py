"""The reference program's work at a KX-8H node's front (grader side): the GPU tray's release levers, the tray to its
service stop and home, the hinged cover. Positions are written in the node's rack frame (x along the row, y
from the cold aisle into the rack, z up; refskills.RefSkills.to_world) so they serve every node of both rows."""

from __future__ import annotations

import math

import numpy as np

from . import hall as H
from .refskills import TUCK, mulq, quat_from_axes, quat_mat, rotq

LEVER_OPEN = 0.62
TRAY_OUT = H.TRAY_SERVICE_TRAVEL
FRONT_Y = -0.27            # the robot's base in front of a closed node (rack frame)
LEVER_Y = -0.24            # ... for the levers (closer, lower)
COVER_STANCE = (-0.08, -0.82)   # in front of the extended tray, for the cover (rack frame, x from the rack's middle)


class NodeFront:
    def __init__(self, S):
        self.S = S
        self.c = S.c

    # ------------------------------------------------------------------------------------------------ helpers
    def _w(self, rack, lx, ly, lyaw=math.pi / 2):
        return self.S.to_world(rack, lx, ly, lyaw)

    def _rotate_about_x(self, rack, pivot, p, ang):
        """p turned by ang about the rack's x axis through pivot (the levers' hinges)."""
        Rr = self.S.vec_to_world(rack, np.eye(3))
        rel = Rr.T @ (np.asarray(p) - np.asarray(pivot))
        ca, sa = math.cos(ang), math.sin(ang)
        rl = np.array([rel[0], rel[1] * ca - rel[2] * sa, rel[1] * sa + rel[2] * ca])
        return np.asarray(pivot) + Rr @ rl

    def _q(self, rack, x_axis, y_axis):
        return quat_from_axes(self.S.vec_to_world(rack, x_axis), self.S.vec_to_world(rack, y_axis))

    # ------------------------------------------------------------------------------------------------ levers, tray
    def _lever_grabs(self, host, rack):
        S = self.S
        grabs, quats = {}, {}
        for side, sd in (("left", "l"), ("right", "r")):
            a, b, r = S.feature("lever_%s_%s" % (host, sd))
            g = a + 0.70 * (b - a)
            grabs[side] = g
            quats[side] = S.best_quat(side, g, [self._q(rack, (0, 1, 0), (1, 0, 0)), self._q(rack, (0, 1, 0), (-1, 0, 0)),
                                                self._q(rack, (0, 0.7, -0.7), (1, 0, 0)),
                                                self._q(rack, (0, 0.7, -0.7), (-1, 0, 0))])
        return grabs, quats

    def _grab_levers(self, host, rack):
        S = self.S
        grabs, quats = self._lever_grabs(host, rack)
        app = S.vec_to_world(rack, (0, 1, 0))
        for side in ("left", "right"):
            S.reach(side, grabs[side] - 0.06 * app, quats[side])
            S.reach(side, grabs[side], quats[side])
            if S.grip(side, True) is None:
                S.log("[ref] %s: %s lever grip failed" % (host, side))
                return None, None
        return grabs, quats

    def _swing_levers(self, host, rack, grabs, quats, ang_from, ang_to, steps=7):
        S, c = self.S, self.c
        pivot = {sd: c.body_pose("lever_%s_%s" % (host, sd))[0] for sd in ("l", "r")}
        for ang in np.linspace(ang_from, ang_to, steps)[1:]:
            for side, sd in (("left", "l"), ("right", "r")):
                tgt = self._rotate_about_x(rack, pivot[sd], grabs[side], ang - ang_from)
                c.command({side + "_arm": {"pose": list(tgt) + list(quats[side]), "frame": "room"}})
            S.run(60)

    def tray_extract(self, host):
        """Both levers open, the tray pulled to its service stop (the robot walks back holding both levers), both
        levers folded back. Returns True when the tray is at its stop."""
        S, c = self.S, self.c
        rack = S.rack_of(host)
        S.phase = "tray out %s" % host
        S.walk(*self._w(rack, 0.0, LEVER_Y))
        S.posture(stand=0.64, waist=(0.0, 0.0, 0.30))
        grabs, quats = self._grab_levers(host, rack)
        if grabs is None:
            return False
        self._swing_levers(host, rack, grabs, quats, 0.0, LEVER_OPEN)
        if c.fixtures.latched[host]:
            S.log("[ref] %s: the tray did not unlatch" % host)
        for side in ("left", "right"):
            S.hold_in_base(side)
        S.run(20)
        x, y, yaw = self._w(rack, 0.0, LEVER_Y - TRAY_OUT - 0.005)
        c.command({"base_goal": [x, y, yaw]})
        t0 = c.sim_time
        while c.sim_time - t0 < 20.0 and c.joint_q("tray_" + host) < TRAY_OUT - 0.002 and not c.done:
            S.run(20)
        S.run(30)
        travel = c.joint_q("tray_" + host)
        # let go, the levers folded back flat (they would stick out into the aisle)
        for side in ("left", "right"):
            S.grip(side, False)
        S.run(30)
        S.tuck()
        self.levers_set(host, 0.0)
        S.log("[ref] %s: tray out %.3f m, service stop %s" % (host, travel, c.fixtures.service[host]))
        return travel >= TRAY_OUT - 0.004

    def levers_set(self, host, target):
        """Both levers swung to `target` (0 = folded / closed, LEVER_OPEN = open) where they hang now."""
        S, c = self.S, self.c
        rack = S.rack_of(host)
        cur = c.joint_q("lever_%s_l" % host)
        if abs(cur - target) < 0.04:
            return True
        tray = c.joint_q("tray_" + host)
        S.walk(*self._w(rack, 0.0, LEVER_Y - tray - (0.02 if tray > 0.1 else 0.0)))
        S.posture(stand=0.64, waist=(0.0, 0.0, 0.30))
        grabs, quats = self._grab_levers(host, rack)
        if grabs is None:
            return False
        self._swing_levers(host, rack, grabs, quats, cur, target)
        for side in ("left", "right"):
            S.grip(side, False)
        S.run(30)
        app = S.vec_to_world(rack, (0, 1, 0))
        for side in ("left", "right"):
            S.reach(side, S.site(side) - 0.06 * app, None, timeout=2.0)
        S.tuck()
        return abs(c.joint_q("lever_%s_l" % host) - target) < 0.08

    def tray_insert(self, host):
        """Cover down first. Levers out, the tray pushed home (holding the levers frees the service stop), levers
        closed until the tray latches."""
        S, c = self.S, self.c
        rack = S.rack_of(host)
        if c.fixtures.latched[host]:
            return True
        S.phase = "tray home %s" % host
        self.levers_set(host, LEVER_OPEN)
        tray = c.joint_q("tray_" + host)
        S.walk(*self._w(rack, 0.0, LEVER_Y - tray))
        S.posture(stand=0.64, waist=(0.0, 0.0, 0.30))
        grabs, quats = self._grab_levers(host, rack)
        if grabs is None:
            return False
        for side in ("left", "right"):
            S.hold_in_base(side)
        S.run(20)
        x, y, yaw = self._w(rack, 0.0, LEVER_Y + 0.004)
        c.command({"base_goal": [x, y, yaw]})
        t0 = c.sim_time
        while c.sim_time - t0 < 20.0 and c.joint_q("tray_" + host) > 0.0015 and not c.done:
            S.run(20)
        S.run(40)
        # close the levers (the tray latches when it is home and both levers are down)
        for side in ("left", "right"):
            S.grip(side, False)
        S.run(20)
        S.tuck()
        self.levers_set(host, 0.0)
        S.run(50)
        ok = bool(c.fixtures.latched[host])
        S.log("[ref] %s: tray home %.4f m, latched %s" % (host, c.joint_q("tray_" + host), ok))
        return ok

    # ------------------------------------------------------------------------------------------------ the cover
    def _cover_hand_q(self, rack, ang):
        q0 = self._q(rack, (0, 0, -1), (1, 0, 0))
        return mulq(rotq(self.S.vec_to_world(rack, (0, -1, 0)), 0.5 * ang), q0)

    def _other_arm_clear(self):
        """The right arm tucked back (elbow behind, hand by the hip) before the left hand swings a cover: the cover's
        plate sweeps the space over the tray in front of the leaning torso."""
        S = self.S
        if S.holds("right") is None:
            S.tuck(sides=("right",), back=True, on_hit="skip")

    def _hand_off_cover(self, rack):
        """A cover that slipped out of the hand: the hand opens and backs away from it (out of the rack, then up)."""
        S = self.S
        S.grip("left", False)
        S.run(20)
        S.reach("left", S.site("left") + S.vec_to_world(rack, (0.0, -0.10, 0.0)), None, timeout=3.0, checked=False)
        S.reach("left", S.site("left") + np.array([0.0, 0.0, 0.08]), None, timeout=3.0)
        S.tuck()

    def cover_open(self, host):
        S, c = self.S, self.c
        rack = S.rack_of(host)
        jn = "cover_" + host
        if c.joint_q(jn) > 1.85:
            return True
        S.phase = "cover open %s" % host
        x, y, yaw = self._w(rack, COVER_STANCE[0], COVER_STANCE[1])
        S.walk(x, y, yaw)
        S.posture(stand=0.78, waist=(0.0, 0.0, 0.30))
        a, b, r = S.feature(jn)
        g = 0.5 * (a + b)
        q0 = self._cover_hand_q(rack, 0.0)
        self._other_arm_clear()
        S.reach("left", g + S.vec_to_world(rack, (0, -0.06, 0)) + np.array([0, 0, 0.10]), q0)
        S.reach("left", g + np.array([0.0, 0.0, 0.06]), q0)
        S.reach("left", g, q0)
        if S.grip("left", True) != jn:
            S.grip("left", False)
            S.log("[ref] %s: cover grip failed" % host)
            return False
        hinge = c.body_pose(jn)[0]
        Rr = S.vec_to_world(rack, np.eye(3))
        rel = Rr.T @ (g - hinge)
        S.step = "cover arc"
        for ang in np.linspace(0.0, 1.96, 27)[1:]:
            ca, sa = math.cos(ang), math.sin(ang)
            tgt = hinge + Rr @ np.array([rel[0] * ca - rel[2] * sa, rel[1], rel[0] * sa + rel[2] * ca])
            c.command({"left_arm": {"pose": list(tgt) + list(self._cover_hand_q(rack, ang)), "frame": "room"}})
            t1 = c.sim_time                     # step by step: a hand left behind tears off the cover's edge
            while c.sim_time - t1 < 2.0 and not c.done:
                S.run(10)
                if S.holds("left") != jn or (c.sim_time - t1 >= 0.2 and
                                             float(np.linalg.norm(S.site("left") - tgt)) < 0.010):
                    break
            if S.holds("left") != jn:
                S.log("[ref] %s: the cover slipped out of the hand at %.2f rad (opening)" % (host, c.joint_q(jn)))
                self._hand_off_cover(rack)
                return False
        S.grip("left", False)
        S.run(100)
        p = S.site("left")
        S.reach("left", p + S.vec_to_world(rack, (0, -0.12, 0)), None, timeout=3.0)
        S.reach("left", p + S.vec_to_world(rack, (0.12, -0.16, -0.05)), None, timeout=3.0)
        S.tuck()
        ok = c.joint_q(jn) > 1.85
        if not ok:
            S.log("[ref] %s: cover not on its stay (q %.3f)" % (host, c.joint_q(jn)))
        return ok

    def cover_close(self, host):
        S, c = self.S, self.c
        rack = S.rack_of(host)
        jn = "cover_" + host
        if c.joint_q(jn) < 0.02:
            return True
        S.phase = "cover close %s" % host
        x, y, yaw = self._w(rack, COVER_STANCE[0], COVER_STANCE[1])
        S.walk(x, y, yaw)
        S.posture(stand=0.78, waist=(0.0, 0.0, 0.30))
        a, b, r = S.feature(jn)
        g = 0.5 * (a + b)
        q_now = c.joint_q(jn)
        qh = self._cover_hand_q(rack, q_now)
        # the way the hand left the pull after opening, backwards (clear of the leaning plate)
        self._other_arm_clear()
        S.reach("left", g + S.vec_to_world(rack, (0.12, -0.16, -0.05)), None, timeout=4.0)
        S.reach("left", g + S.vec_to_world(rack, (0.0, -0.12, 0.0)), qh)
        S.reach("left", g, qh)
        if S.grip("left", True) != jn:
            S.grip("left", False)
            S.log("[ref] %s: cover grip (closing) failed" % host)
            return False
        hinge = c.body_pose(jn)[0]
        Rr = S.vec_to_world(rack, np.eye(3))
        # the pull's position at angle 0 from where it is now (rotate back by q_now about the hinge's axis -y)
        rel_now = Rr.T @ (g - hinge)
        cq, sq = math.cos(-q_now), math.sin(-q_now)
        rel = np.array([rel_now[0] * cq - rel_now[2] * sq, rel_now[1], rel_now[0] * sq + rel_now[2] * cq])
        S.step = "cover arc"
        for ang in np.linspace(q_now, 0.0, 23)[1:]:
            ca, sa = math.cos(ang), math.sin(ang)
            tgt = hinge + Rr @ np.array([rel[0] * ca - rel[2] * sa, rel[1], rel[0] * sa + rel[2] * ca])
            c.command({"left_arm": {"pose": list(tgt) + list(self._cover_hand_q(rack, ang)), "frame": "room"}})
            t1 = c.sim_time
            while c.sim_time - t1 < 2.0 and not c.done:
                S.run(10)
                if S.holds("left") != jn or (c.sim_time - t1 >= 0.2 and
                                             float(np.linalg.norm(S.site("left") - tgt)) < 0.010):
                    break
            if S.holds("left") != jn:
                S.log("[ref] %s: the cover slipped out of the hand at %.2f rad (closing)" % (host, c.joint_q(jn)))
                self._hand_off_cover(rack)
                return c.joint_q(jn) < 0.02
            if getattr(self, "trace", False):
                S.log("  cover close %.2f: cover %.3f hand err %.1f mm load %.0f N" % (
                    ang, c.joint_q(jn), 1000 * float(np.linalg.norm(S.site("left") - tgt)), c.grips.tactile("left")))
        S.run(30)
        S.grip("left", False)
        S.run(40)
        S.reach("left", S.site("left") + np.array([0.0, 0.0, 0.08]), None, timeout=3.0)
        S.tuck()
        ok = c.joint_q(jn) < 0.02
        if not ok:
            S.log("[ref] %s: cover did not close (q %.3f)" % (host, c.joint_q(jn)))
        return ok
