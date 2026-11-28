"""The reference program's work at a rack's rear (grader side): the rear door, MPO connectors and OSFP transceivers out
and in, the fibre probe and the cleaner on an end face. Positions in the rack frame (refskills.RefSkills.to_world)."""

from __future__ import annotations

import math

import numpy as np

from . import hall as H
from . import parts as P
from .refskills import ARM_HOME, DryRun, WALK_ARMS, load_table, mat_quat, mulq, quat_from_axes, quat_mat, rotq, rz, wrap

DOOR_OPEN = 1.95             # the rear door to its stop (the stance tables were measured with the door there)
DOOR_R_BASE = 0.36           # the base this far out from the door's handle while the door swings
# collision-aware stances behind a rack (tools/rear_stance.py): per port and hand, relative to the port in the rack
# frame; node ports measured on gpn-a03 (both neighbours present: valid for every node), leaf ports per leaf
PART_TABLE = load_table("rear_part.json")
TOOL_TABLE = load_table("rear_tool.json")


class RearWork:
    def __init__(self, S):
        self.S = S
        self.c = S.c
        self.debug = False           # log every step of plug() (s, lateral, angle, the hand's lag)

    # ------------------------------------------------------------------------------------------------ helpers
    def _rq(self, rack, x_axis, y_axis):
        return quat_from_axes(self.S.vec_to_world(rack, x_axis), self.S.vec_to_world(rack, y_axis))

    def _gentle(self, side, start, direction, until, step=0.002, limit=0.014, q=None, ticks=15):
        """Move the hand from `start` along `direction` in mm steps until until() is true (a latch let go / clicked)."""
        S, c = self.S, self.c
        d = 0.0
        while not until() and d < limit and not c.done:
            d += step
            tgt = np.asarray(start) + np.asarray(direction) * d
            cmd = {"pose": list(tgt) + list(q), "frame": "room"} if q is not None else {"pos": list(tgt), "frame": "room"}
            c.command({side + "_arm": cmd})
            for _ in range(max(1, ticks // 3)):     # (checked often: a push past the click would tear the grip)
                S.run(3)
                if until():
                    break
        return until(), d

    # ------------------------------------------------------------------------------------------------ the door
    def door_open(self, rack):
        S, c = self.S, self.c
        jn = "rdoor_" + rack
        if c.joint_q(jn) > DOOR_OPEN - 0.1:
            return True
        for idx in range(1, 6):                 # one rear door open at a time: an open leaf swings across the hot aisle
            other = "%s%02d" % (rack[0], idx)
            if other != rack and c.joint_q("rdoor_" + other) > 0.03:
                S.tuck(walk=True)
                self.door_close(other)
        S.phase = "rear door open %s" % rack
        a, b, r = S.feature("rdoor_leaf_" + rack)
        h0 = 0.5 * (a + b)
        hinge = c.body_pose("rdoor_leaf_" + rack)[0]
        out = S.vec_to_world(rack, (0, 1, 0))                     # from the rack's rear into the hot aisle
        x0, y0, yaw0 = S.to_world(rack, 0.0, 0.0, -math.pi / 2)
        base0 = np.array([h0[0], h0[1], 0.0]) + out * DOOR_R_BASE - S.vec_to_world(rack, (0.04, 0, 0))
        S.walk(base0[0], base0[1], yaw0)
        S.posture(stand=0.78, waist=(0.0, 0.0, 0.05))
        q0 = self._rq(rack, (0, -1, 0), (1, 0, 0))                # approach into the rack, the pinch across the bar
        S.reach("right", h0 + out * 0.07, q0)
        S.reach("right", h0, q0)
        if S.grip("right", True) != "rdoor_leaf_" + rack:
            S.grip("right", False)
            S.log("[ref] %s: rear door handle grip failed" % rack)
            return False
        hz = np.array([hinge[0], hinge[1], 0.0])
        for ang in np.linspace(0.0, DOOR_OPEN, 15)[1:]:
            R = rz(ang)
            h = np.array([hinge[0], hinge[1], h0[2]]) + R @ (h0 - np.array([hinge[0], hinge[1], h0[2]]))
            bp = hz + R @ (base0 - hz)
            c.command({"right_arm": {"pose": list(h) + list(mulq(rotq((0, 0, 1), ang), q0)), "frame": "room"},
                       "base_goal": [float(bp[0]), float(bp[1]), float(wrap(yaw0 + ang))]})
            S.run(70)
        S.grip("right", False)
        S.run(20)
        # the hand off the handle (out of the leaf's outer face), a step into the aisle, then the arms in
        q_now = c.joint_q(jn)
        n_out = S.vec_to_world(rack, rz(q_now) @ np.array([0.0, 1.0, 0.0]))
        S.reach("right", S.site("right") + n_out * 0.10, None, timeout=2.0)
        b = S.base()
        step = np.array([b[0], b[1], 0.0]) + out * 0.22
        c.command({"base_goal": [float(step[0]), float(step[1]), float(b[2])]})
        S.run(150)
        S.tuck()
        ok = c.joint_q(jn) > DOOR_OPEN - 0.15
        S.log("[ref] %s: rear door %.2f rad" % (rack, c.joint_q(jn)))
        return ok

    def door_close(self, rack):
        S, c = self.S, self.c
        jn = "rdoor_" + rack
        if c.joint_q(jn) < 0.02:
            return True
        S.phase = "rear door close %s" % rack
        q_now = c.joint_q(jn)
        hinge = c.body_pose("rdoor_leaf_" + rack)[0]
        hz = np.array([hinge[0], hinge[1], 0.0])
        a, b, r = S.feature("rdoor_leaf_" + rack)
        h_now = 0.5 * (a + b)
        out = S.vec_to_world(rack, (0, 1, 0))
        x0, y0, yaw0 = S.to_world(rack, 0.0, 0.0, -math.pi / 2)
        R_back = rz(-q_now)
        h0 = np.array([hinge[0], hinge[1], h_now[2]]) + R_back @ (h_now - np.array([hinge[0], hinge[1], h_now[2]]))
        base0 = np.array([h0[0], h0[1], 0.0]) + out * DOOR_R_BASE - S.vec_to_world(rack, (0.04, 0, 0))
        bp = hz + rz(q_now) @ (base0 - hz)
        S.walk(bp[0], bp[1], wrap(yaw0 + q_now))
        S.posture(stand=0.78, waist=(0.0, 0.0, 0.05))
        q0 = self._rq(rack, (0, -1, 0), (1, 0, 0))
        qh = mulq(rotq((0, 0, 1), q_now), q0)
        app = quat_mat(qh)[:, 0]
        S.reach("right", h_now - app * 0.07, qh)
        S.reach("right", h_now, qh)
        if S.grip("right", True) != "rdoor_leaf_" + rack:
            S.grip("right", False)
            S.log("[ref] %s: rear door handle grip (closing) failed" % rack)
            return False
        S.step = "door arc"
        for ang in np.linspace(q_now, 0.0, 29)[1:]:
            R = rz(ang)
            h = np.array([hinge[0], hinge[1], h0[2]]) + R @ (h0 - np.array([hinge[0], hinge[1], h0[2]]))
            bpp = hz + R @ (base0 - hz)
            c.command({"right_arm": {"pose": list(h) + list(mulq(rotq((0, 0, 1), ang), q0)), "frame": "room"},
                       "base_goal": [float(bpp[0]), float(bpp[1]), float(wrap(yaw0 + ang))]})
            # step by step: the base's goal approach lags a goal that keeps moving (by its speed over its gain), and
            # a base left behind pulls the hand off the door's handle
            t1 = c.sim_time
            while c.sim_time - t1 < 2.5 and not c.done:
                S.run(10)
                b = S.base()
                if c.sim_time - t1 >= 0.3 and math.hypot(b[0] - bpp[0], b[1] - bpp[1]) < 0.012 and \
                        float(np.linalg.norm(S.site("right") - h)) < 0.012:
                    break
            if getattr(self, "door_trace", False):
                b = S.base()
                S.log("  close step %.2f: door %.3f hand err %.3f base err %.3f" % (
                    ang, c.joint_q(jn), float(np.linalg.norm(S.site("right") - h)), float(np.hypot(b[0] - bpp[0], b[1] - bpp[1]))))
        # the hand leads the door by the grip's give: carry the handle's arc on past the closed position (the base
        # stays) until the door is home
        ang = 0.0
        while c.joint_q(jn) > 0.012 and ang > -0.40 and not c.done:
            ang -= 0.04
            R = rz(ang)
            h = np.array([hinge[0], hinge[1], h0[2]]) + R @ (h0 - np.array([hinge[0], hinge[1], h0[2]]))
            c.command({"right_arm": {"pose": list(h) + list(mulq(rotq((0, 0, 1), ang), q0)), "frame": "room"}})
            S.run(30)
        S.run(20)
        S.grip("right", False)
        S.run(20)
        S.reach("right", S.site("right") + out * 0.10, None, timeout=2.0)
        S.tuck()
        ok = c.joint_q(jn) < 0.02
        S.log("[ref] %s: rear door %.3f rad (closed %s)" % (rack, c.joint_q(jn), ok))
        return ok

    # ------------------------------------------------------------------------------------------------ stances
    @staticmethod
    def table_key(port):
        dev, name = port.split("/")
        return ("node/" + name) if dev.startswith("gpn-") else port

    def sides_for(self, port):
        """The hands the part table has a stance for at this port, the roomiest first."""
        rec = PART_TABLE.get(self.table_key(port), {})
        return [sd for sd, _ in sorted(((sd, r) for sd, r in rec.items() if r), key=lambda kv: -kv[1]["clearance"])]

    def _stance(self, port, rec):
        S, c = self.S, self.c
        rack = S.rack_of(port.split("/")[0])
        o, R, ax = c.ports.world(port)
        (ox, oy), ryaw = S.rack_pose(rack)
        lp = rz(-ryaw) @ np.array([o[0] - ox, o[1] - oy, 0.0])
        if rec.get("front"):                    # a fan bay: worked from the cold aisle, facing the rack's front
            x, y, yaw = S.to_world(rack, lp[0] + rec["dx"], lp[1] - rec["dy"], math.pi / 2 + rec["dyaw"])
        else:
            x, y, yaw = S.to_world(rack, lp[0] + rec["dx"], lp[1] + rec["dy"], -math.pi / 2 + rec["dyaw"])
        return rack, (x, y, yaw)

    def go_part(self, port, side):
        """To the table's stance for work on a port's part with one hand; returns (rack, hand quat) or None."""
        S = self.S
        rec = (PART_TABLE.get(self.table_key(port)) or {}).get(side)
        if not rec:
            S.log("[ref] no %s-hand stance for %s" % (side, port))
            return None
        rack, (x, y, yaw) = self._stance(port, rec)
        b = S.base()
        if math.hypot(b[0] - x, b[1] - y) < 0.10 and abs(wrap(b[2] - yaw)) < 0.25:
            S.step_to(x, y, yaw)                # (the other hand's stance at the same port: a short step)
        elif math.hypot(b[0] - x, b[1] - y) < 0.10 and abs(wrap(b[2] - yaw)) < 1.2 and self._turn_clear(x, y, yaw):
            # the other hand's stance at the same port, turned: a turn on the spot where the arms and what they hold
            # sweep clear (a hot swap's clock runs)
            S.step_to(x, y, yaw, timeout=8.0)
        else:
            # behind a rack there is nothing at waist height to reach over: the free arms come in at home (where
            # the stance study kept them; a raised hand would meet the open door leaf), a held part as carried
            S.approach(x, y, yaw, work=())
        S.posture(stand=rec["stand"], waist=(rec["waist_yaw"], 0.0, rec["pitch"]))
        self.last_rec = rec
        return rack, self._rq(rack, rec["hand_x"], rec["hand_y"])

    def _turn_clear(self, x, y, yaw, n=6):
        """A base move to (x, y, yaw) from where the robot stands, dry-run in n steps with the arms held where they
        are relative to the base: True when no link and nothing a hand holds comes near anything."""
        S = self.S
        b = S.base()
        D = DryRun(S)
        D.baseline(("left", "right"), True)
        dyaw = wrap(yaw - b[2])
        for k in range(1, n + 1):
            f = k / n
            D.set_base(b[0] + (x - b[0]) * f, b[1] + (y - b[1]) * f, b[2] + dyaw * f)
            if D.hits(("left", "right"), True):
                return False
        return True

    def tool_side(self, port):
        """The hand that works the probe / cleaner on this port's receptacle (the other holds the connector, on the
        side its cable comes from); None when the table has no stance."""
        rec = TOOL_TABLE.get(self.table_key(port)) or {}
        anc = self.c.ports.rec[port].get("anchor")
        # the connector's cable comes from the rack's -x side for a node (the robot's right behind the rack), +x for a
        # leaf: the hand on that side holds the connector, the other works the tool
        pref = "left" if (anc is None or anc[0] < self.c.ports.rec[port]["pos"][0]) else "right"
        sides = sorted((sd for sd, r in rec.items() if r), key=lambda sd: (-round(rec[sd]["clearance"], 3), sd != pref))
        return sides[0] if sides else None

    def go_tool(self, port):
        """To the table's stance for the probe / cleaner on a port's receptacle; returns the tool's roll about its
        axis or None."""
        S = self.S
        side = self.tool_side(port)
        rec = (TOOL_TABLE.get(self.table_key(port)) or {}).get(side) if side else None
        if not rec:
            S.log("[ref] no tool stance for %s" % port)
            return None
        rack, (x, y, yaw) = self._stance(port, rec)
        S.walk(x, y, yaw)
        S.posture(stand=rec["stand"], waist=(rec["waist_yaw"], 0.0, rec["pitch"]))
        other = "right" if side == "left" else "left"
        if rec.get("hold", "walk") != "walk" and S.holds(other):
            S.arms(**{("q_" + other): rec["hold_q"]}, settle=1.5)      # the connector up by its cable's tie
        self.last_tool_rec = rec
        return rec["roll"]

    # ------------------------------------------------------------------------------------------------ connectors
    def unplug(self, port, side):
        """A connector / transceiver / supply / cord out of its port with one hand from the table's stance (gently
        until its latch lets go, then clear of the port); returns the proxy held, or None."""
        S, c = self.S, self.c
        S.phase = "unplug %s" % port
        got = self.go_part(port, side)
        if got is None:
            return None
        rack, q = got
        kind = c.ports.rec[port]["kind"]
        a, b, r = S.feature(port)
        g = a + self.last_rec.get("grip_s", 0.55) * (b - a)
        o, R, ax = c.ports.world(port)
        appr = quat_mat(q)[:, 0]
        S.arm_to(side, g - appr * 0.06, q, self.last_rec.get("arm_q0"))
        S.reach(side, g - appr * 0.02, q)
        S.reach(side, g, q, tol=0.002)
        px = S.grip(side, True)
        if px is None:
            S.grip(side, False)
            S.reach(side, g - appr * 0.06, q, timeout=3.0)
            S.log("[ref] %s: grip failed" % port)
            return None
        ok, d = self._gentle(side, g, ax, lambda: not c.pool.latched.get(px, True), q=q)
        if not ok:
            S.log("[ref] %s: the latch did not let go (interlock?)" % port)
            S.grip(side, False)
            S.reach(side, g - appr * 0.06, q, timeout=3.0)
            return None
        travel = P.KINDS[kind]["travel"]
        # out along the axis, slowly while the guide holds it (the guide's grip on a sliding part grows with its
        # speed), then clear of the port
        out_to = g + ax * (travel + 0.03)
        S._stream_pose(side, out_to, q, v_max=0.08)
        if S.holds(side) == px:
            S.reach(side, out_to, q, timeout=3.0, check_held=False)
        if S.holds(side) != px or c.pool.state.get(px) != "free":
            S.log("[ref] %s: not out (state %s, holds %s)" % (port, c.pool.state.get(px), S.holds(side)))
            return None
        return px

    def plug(self, px, port, side):
        """A held part into its port from the table's stance: over the port's axis (closed loop on the part's own
        pose), in, then pushed gently until it clicks; the hand lets go and backs out."""
        S, c = self.S, self.c
        S.phase = "plug %s" % port
        o, R, ax = c.ports.world(port)
        b = c.pool.body[px]
        kind = c.pool.kind[px]
        Rp = c.d.xmat[b].reshape(3, 3)
        if P.KINDS[kind]["mass"] < 0.5 and \
                float((Rp @ np.asarray(P.KINDS[kind]["axis"], float)) @ ax) < math.cos(math.radians(30.0)):
            # a part held turned away (a connector presented to the probe) is turned square to its port here, away
            # from the rack, before the hand comes close (turned by the rack it would sweep the rack's face)
            S.reach(side, S.site(side), mat_quat(R @ Rp.T @ S.site_R(side)), timeout=4.0, optional=True)
        got = self.go_part(port, side)
        if got is None:
            return False
        rack, q_tab = got
        o, R, ax = c.ports.world(port)
        q = mat_quat(S.site_R(side))
        # the part's axis onto the port's: keep the grasp, turn the hand so that the part's frame matches the port's
        Rp = c.d.xmat[b].reshape(3, 3)
        Rs_new = R @ Rp.T @ S.site_R(side)
        q = mat_quat(Rs_new)
        off = S.site_R(side).T @ (c.d.xpos[b] - S.site(side))      # the part in the hand's frame
        kind = c.pool.kind[px]
        if self.debug:
            pa = c.d.xmat[b].reshape(3, 3) @ np.asarray(P.KINDS[kind]["axis"], float)
            tid = c.pool.tendon.get(px) if hasattr(c.pool, "tendon") else None
            tl = (float(c.d.ten_length[tid]), float(c.m.tendon_range[tid][1])) if tid is not None else None
            S.log("[plug] t %.2f start: part axis %.1f deg off the port's, %.0f mm from it; cable %s; part in hand %s" % (
                c.sim_time, math.degrees(math.acos(max(-1.0, min(1.0, float(pa @ ax))))),
                1000 * float(np.linalg.norm(c.d.xpos[b] - o)), tl, np.round(off, 3)))
        if P.KINDS[kind]["mass"] > 1.0:          # (a light part's latch is reached with the stance's own reach)
            self._room_to_push(side, o - ax * 0.010 - Rs_new @ off, q, self.last_rec.get("arm_qN"))
        s_pre = max(0.05, P.KINDS[kind]["travel"] + 0.03)              # square to the port outside it first
        S.arm_to(side, o + ax * s_pre - Rs_new @ off, q, self.last_rec.get("arm_qN"), check_held=False)
        cmd = S.site(side)
        if self.debug:
            pa = c.d.xmat[b].reshape(3, 3) @ np.asarray(P.KINDS[kind]["axis"], float)
            qe = float(2.0 * math.degrees(math.acos(min(1.0, abs(float(np.dot(mat_quat(S.site_R(side)), q)))))))
            S.log("[plug] t %.2f over the port: hand %.1f mm / %.1f deg from its target, part axis %.1f deg off, "
                  "part in hand %s" % (c.sim_time, 1000 * float(np.linalg.norm(S.site(side) - (o + ax * s_pre - Rs_new @ off))),
                                      qe, math.degrees(math.acos(max(-1.0, min(1.0, float(pa @ ax))))),
                                      np.round(S.site_R(side).T @ (c.d.xpos[b] - S.site(side)), 3)))
        axis_p = np.asarray(P.KINDS[kind]["axis"], float)
        steps = list(np.arange(s_pre, 0.06, -0.04)) + [0.04, 0.015]
        for s_tgt in steps:
            if c.pool.state.get(px) == "slot":
                break                           # taken by the guide: the rest is a push along the axis
            for _ in range(5 if s_tgt <= 0.04 else 2):
                dv = c.d.xpos[b] - o
                off_l = dv - (dv @ ax) * ax
                s_now = float(dv @ ax)
                pa = c.d.xmat[b].reshape(3, 3) @ axis_p                   # the part's axis onto the port's
                w = np.cross(pa, ax)
                ang = math.asin(min(1.0, float(np.linalg.norm(w))))
                if ang > math.radians(0.3):
                    q = mulq(rotq(w / np.linalg.norm(w), ang), q)
                if self.debug:
                    S.log("[plug] t %.2f s_tgt %.3f s %.4f lat %.2f mm ang %.2f deg lag %.1f mm state %s" % (
                        c.sim_time, s_tgt, s_now, 1000 * np.linalg.norm(off_l), math.degrees(ang),
                        1000 * np.linalg.norm(S.site(side) - cmd), c.pool.state.get(px)))
                if np.linalg.norm(off_l) < 0.0003 and abs(s_now - s_tgt) < 0.003 and ang < math.radians(0.6):
                    break
                if c.pool.state.get(px) == "slot":
                    break
                cmd = cmd - off_l + ax * (s_tgt - s_now)
                S.settle(side, cmd, q, t_min=0.1, t_max=0.8)
        tol_fine = 0.5 * P.KINDS[kind]["tol"][0]          # the guide catches a part within its tolerance: half of it
        for attempt in range(0 if c.pool.state.get(px) == "slot" else 3):
            for s_t in np.linspace(0.012, 0.001, 4):
                for _ in range(4):                            # the lateral error down before each millimetre in
                    dv = c.d.xpos[b] - o
                    off = dv - (dv @ ax) * ax
                    if self.debug:
                        S.log("[plug] t %.2f fine %.4f s %.4f lat %.2f mm lag %.1f mm state %s" % (
                            c.sim_time, s_t, float(dv @ ax), 1000 * np.linalg.norm(off), 1000 * np.linalg.norm(S.site(side) - cmd),
                            c.pool.state.get(px)))
                    if c.pool.state.get(px) == "slot":
                        break
                    cmd = cmd - off + ax * (s_t - float(dv @ ax))
                    S.settle(side, cmd, q)
                    dv = c.d.xpos[b] - o
                    if c.pool.state.get(px) == "slot" or \
                            float(np.linalg.norm(dv - (dv @ ax) * ax)) < tol_fine:
                        break
            if c.pool.state.get(px) == "slot":
                break
            # not caught by the guide: back out to just outside it and try the last millimetres again
            dv = c.d.xpos[b] - o
            cmd = cmd + ax * (P.KINDS[kind]["travel"] + 0.004 - float(dv @ ax))
            S.settle(side, cmd, q)
            S.log("[ref] %s: not caught by the guide (attempt %d); again" % (port, attempt + 1))
        if c.pool.state.get(px) == "slot":
            # in the guide, which carries the part and holds it square: the hand follows where it is (the correction
            # for the sag it had while it carried the part in the air, kept, would lift the part against the guide)
            # and advances it along the axis to just short of the seat, then pushes until it clicks
            dv = c.d.xpos[b] - o
            s_now = float(dv @ ax)
            q = mat_quat(S.site_R(side))
            if s_now > 0.003:
                # slowly: the guide's grip on a sliding part grows with its speed (a supply pushed at 0.25 m/s
                # needs more than the hand's grip holds)
                tgt = S.site(side) + ax * (0.002 - s_now)
                S._stream_pose(side, tgt, q, v_max=0.08)
                S.reach(side, tgt, q, tol=0.003, timeout=3.0, checked=False)
            cmd = S.site(side).copy()
            if self.debug:
                dv = c.d.xpos[b] - o
                S.log("[plug] t %.2f guided in: s %.4f lat %.2f mm, hand %.1f mm from its target, holds %s" % (
                    c.sim_time, float(dv @ ax), 1000 * float(np.linalg.norm(dv - (dv @ ax) * ax)),
                    1000 * float(np.linalg.norm(S.site(side) - cmd)), S.holds(side)))
        ok, d = self._gentle(side, cmd, -ax, lambda: bool(c.pool.latched.get(px)), step=0.001, limit=0.012, q=q)
        if self.debug:
            dv = c.d.xpos[b] - o
            S.log("[plug] t %.2f pushed %.1f mm: s %.4f latched %s state %s; hand %.1f mm short of its target, ik %s" % (
                c.sim_time, 1000 * d, float(dv @ ax), c.pool.latched.get(px), c.pool.state.get(px),
                1000 * float((S.site(side) - (cmd - ax * d)) @ (-ax)),
                tuple(round(float(v), 4) for v in c.ctrl.arm_ik_err[side])))
        S.grip(side, False)
        S.run(40)
        appr = quat_mat(q)[:, 0]
        S.reach(side, S.site(side) - appr * 0.06, q, timeout=3.0)
        seated = c.ports.occupied[port] and (c.ports.proxy[port] is None or bool(c.pool.latched.get(px)))
        if not seated:
            S.log("[ref] %s: not seated (latched %s, state %s)" % (port, c.pool.latched.get(px), c.pool.state.get(px)))
        return seated

    def _room_to_push(self, side, tgt, q, seed=None):
        """Before a part goes in: the arm can reach the hand's pose for the part pushed `tgt` (some millimetres
        past its seat: the push builds its force by the servos' give); if not, the torso leans a little more now,
        while the part is still in the air, until it can."""
        S, c = self.S, self.c
        r = c.robot
        w0 = np.array(c.ctrl.waist_cmd, float)
        sq = np.asarray(seed, float) if seed is not None else c.d.qpos[r.arm_q[side]].copy()
        for extra in (0.0, 0.05, 0.10, 0.15):
            qp = c.d.qpos.copy()
            qp[r.waist_q[2]] = w0[2] + extra
            # (seeded by and pulled towards the stance study's arm at the seat: the branch the push will be on)
            _, ep, er = c.ctrl.ik.solve(qp, side, tgt, q, sq, iters=100, q_rest=sq)
            if ep < 0.003:
                break
        else:
            extra = 0.15
        if extra > 0.0:
            S.log("[ref] %s hand: the push home out of reach; leaning %.2f rad more" % (side, extra))
            S.posture(waist=(float(w0[0]), float(w0[1]), float(w0[2] + extra)), settle=0.6)

    def set_aside(self, side, port, toward=None, up=0.05, far=0.08):
        """A held connector / plug moved out of its port's line (towards its cable's tie, out and up) and let go: a
        stiff cord keeps it there. The hand backs off and is free."""
        S, c = self.S, self.c
        px = S.holds(side)
        if px is None:
            return None
        rack = S.rack_of(port.split("/")[0])
        anc = c.ports.rec[port].get("anchor")
        sx = 1.0 if (anc is None or anc[0] >= c.ports.rec[port]["pos"][0]) else -1.0
        if c.ports.rec[port]["kind"] == "plug":
            sx = 1.0 if c.ports.rec[port].get("feed", "A") == "A" else -1.0
        if toward is not None:
            # towards the hand that will take it back (a robot facing the rack's rear has the rack's +x on its left)
            sx = 1.0 if toward == "left" else -1.0
        q = mat_quat(S.site_R(side))
        # well clear of the port's line: a supply pulled out of its bay sweeps 0.22 m out along the bay's axis, past
        # where its own cord's plug would float if it were only nudged aside
        S.reach(side, S.site(side) + S.vec_to_world(rack, (0.5 * far * sx, 0.04, 0.03)), q, timeout=3.0, check_held=False)
        S.reach(side, S.site(side) + S.vec_to_world(rack, (far * sx, 0.06, up)), q, timeout=3.0, check_held=False)
        # how the hand held it (in the part's frame): it is taken back the same way wherever it has drifted to
        b = c.pool.body[px]
        Rp = c.d.xmat[b].reshape(3, 3)
        self.held_as = getattr(self, "held_as", {})
        self.held_as[px] = (Rp.T @ (S.site(side) - c.d.xpos[b]), Rp.T @ S.site_R(side), side)
        S.grip(side, False)
        S.run(30)
        S.reach(side, S.site(side) + S.vec_to_world(rack, (0.0, 0.10, 0.0)), q, timeout=3.0)
        return px

    def regrasp(self, px, port, side):
        """A connector / plug floating where it was let go taken back with the table's grasp for its port (it floats
        square to the port, as it came out)."""
        S, c = self.S, self.c
        got = self.go_part(port, side)
        if got is None:
            return False
        rack, q = got
        a, b, r = S.feature(px)
        g = a + 0.55 * (b - a)
        rel = getattr(self, "held_as", {}).get(px)
        if rel is not None and rel[2] == side:
            # the way it was let go, from where it floats now (a bump while the other hand worked may have turned it)
            bb = c.pool.body[px]
            Rp = c.d.xmat[bb].reshape(3, 3)
            g = c.d.xpos[bb] + Rp @ rel[0]
            q = mat_quat(Rp @ rel[1])
        appr = quat_mat(q)[:, 0]
        S.arm_to(side, g - appr * 0.06, q, self.last_rec.get("arm_q0"))
        S.reach(side, g, q, tol=0.002)
        if S.grip(side, True) != px:
            bb = c.pool.body[px]
            cand = c.grips.candidates(side)
            S.log("[ref] %s: could not take %s back (hand %.1f mm / %.1f deg from the grasp, part state %s, part at %s, "
                  "the grip's candidates %s)" % (port, px, 1000 * float(np.linalg.norm(S.site(side) - g)),
                                                 math.degrees(2 * math.acos(min(1.0, abs(float(np.dot(mat_quat(S.site_R(side)), q)))))),
                                                 c.pool.state.get(px), np.round(c.d.xpos[bb], 3), cand[:3]))
            S.grip(side, False)
            S.reach(side, g - appr * 0.06, q, timeout=3.0)
            return False
        return True

    def hold_close(self, side):
        """A held connector in close to the body (the arm as for walking): back from the rack first, then the arm's
        walking pose (the stance tables for the probe assume this arm there)."""
        S, c = self.S, self.c
        b = S.base()
        S.reach(side, S.site(side) + rz(b[2]) @ np.array([-0.12, 0.0, 0.04]), mat_quat(S.site_R(side)), timeout=3.0)
        S.arms(**{("q_" + side): WALK_ARMS[side]}, settle=1.5)

    def park(self, side, port):
        """A held connector out of the work zone: out from its port, up and to the hand's side."""
        S, c = self.S, self.c
        o, R, ax = c.ports.world(port)
        b = S.base()
        lat = rz(b[2]) @ np.array([0.0, -1.0 if side == "right" else 1.0, 0.0])
        S.reach(side, S.site(side) + ax * 0.07 + np.array([0, 0, 0.08]) + lat * 0.06, mat_quat(S.site_R(side)), timeout=3.0)

    # ------------------------------------------------------------------------------------------------ the probe and the cleaner
    def _tool_on_face(self, side, tool, center, normal, axial, q_hand, face_id=None):
        """Close the loop on a held tool's tip: at center + normal * axial, its axis along the normal; returns the
        tip's (lateral, axial) offset from the face's centre. With face_id the face is read again at every step (a
        connector held in the other hand moves a little)."""
        S, c = self.S, self.c
        n = np.asarray(normal, float) / np.linalg.norm(normal)
        cmd = S.site(side)                      # the loop integrates on the command (the servo's sag stays put)
        qc = np.asarray(q_hand, float)
        for _ in range(12):
            if face_id is not None:
                cen_, n_ = self.face_of(face_id)
                if cen_ is not None:
                    center, n = cen_, n_ / np.linalg.norm(n_)
            tip, tax = c.tools.tip_axis("tool_" + tool)
            e = (center + n * axial) - tip
            w = np.cross(tax, n)                 # the tool's axis onto the face's normal
            ang = math.asin(min(1.0, float(np.linalg.norm(w))))
            if getattr(self, "debug_tool", False):
                S.log("    loop: |e| %.2f mm ang %.2f deg cmd-site %.2f mm" % (1000 * float(np.linalg.norm(e)), math.degrees(ang),
                      1000 * float(np.linalg.norm(cmd - S.site(side)))))
            if np.linalg.norm(e) < 0.00015 and ang < math.radians(0.8):
                break
            if face_id is not None and np.linalg.norm(e) > 0.0006:
                self.wait_still(face_id, t_max=1.5)
                cen_, n_ = self.face_of(face_id)
                if cen_ is not None:
                    center, n = cen_, n_ / np.linalg.norm(n_)
                tip, tax = c.tools.tip_axis("tool_" + tool)
                e = (center + n * axial) - tip
            if ang > math.radians(0.4):
                qc = mulq(rotq(w / np.linalg.norm(w), ang), qc)
            cmd = cmd + e
            S.settle(side, cmd, qc)
        if face_id is not None:
            cen_, n_ = self.face_of(face_id)
            if cen_ is not None:
                center, n = cen_, n_ / np.linalg.norm(n_)
        tip, tax = c.tools.tip_axis("tool_" + tool)
        dv = tip - center
        self.last_tool_angle = math.degrees(math.acos(max(-1.0, min(1.0, float(tax @ n)))))
        return float(np.linalg.norm(dv - (dv @ n) * n)), float(dv @ n)

    def _tool_pose(self, side, tool, normal, roll):
        """(hand quat, tool frame) that point the held tool's axis (its z) along `normal`, turned by `roll` about it."""
        S, c = self.S, self.c
        b = c.tools.body["tool_" + tool]
        Rt = c.d.xmat[b].reshape(3, 3)
        rel = Rt.T @ S.site_R(side)                       # the hand in the tool frame (the grasp as it is)
        z = np.asarray(normal, float) / np.linalg.norm(normal)
        x0 = np.cross(z, np.array([0.0, 0.0, 1.0]))
        if np.linalg.norm(x0) < 1e-3:
            x0 = np.array([1.0, 0.0, 0.0])
        x0 /= np.linalg.norm(x0)
        y0 = np.cross(z, x0)
        xr = math.cos(roll) * x0 + math.sin(roll) * y0
        Rt_new = np.stack([xr, np.cross(z, xr), z], 1)
        return mat_quat(Rt_new @ rel), Rt_new

    def _site_for_tip(self, side, tool, tip, Rt_new):
        """The hand's position that puts the held tool's tip at `tip` with the tool frame Rt_new."""
        S, c = self.S, self.c
        b = c.tools.body["tool_" + tool]
        rel_p = c.d.xmat[b].reshape(3, 3).T @ (S.site(side) - c.d.xpos[b])
        return np.asarray(tip) - Rt_new @ np.asarray(P.TOOLS[tool]["tip"]) + Rt_new @ rel_p

    def face_of(self, face_id):
        for fid, cen, n in self.c.faces():
            if fid == face_id:
                return np.asarray(cen), np.asarray(n)
        return None, None

    def step_back(self, dist=0.35):
        """Straight back from where the robot stands (away from the rack in front of it)."""
        S = self.S
        b = S.base()
        back = np.array([b[0], b[1], 0.0]) + rz(b[2]) @ np.array([-dist, 0.0, 0.0])
        S.walk(back[0], back[1], b[2])

    def clear_of_rack(self, rack, dist=0.62):
        """Straight back from a rack's rear face until the base stands `dist` from it (no farther: a connector held
        on its cable has its service loop's slack only), clear of the rack's open door leaf and posts."""
        S = self.S
        o, yaw = S.rack_pose(rack)
        n = rz(yaw) @ np.array([0.0, 1.0, 0.0])
        b = S.base()
        d0 = float((np.array([b[0], b[1]]) - o) @ n[:2]) - H.RACK_D
        if d0 < 0.0 or d0 >= dist - 0.03:
            return
        S.step_to(b[0] + n[0] * (dist - d0), b[1] + n[1] * (dist - d0), b[2], timeout=8.0)

    def clear_of_door(self, rack, room=0.55, most=0.30):
        """A sideways step away from the hinge of the rack's open rear door (on the rack's -x post: on the right of a
        robot facing the rack's rear) until the base stands `room` from the hinge across the rack (at most `most`:
        a connector held on its cable has its service loop's slack only): the door leaf opens back along the hot
        aisle on that side, where a hand working out to the right would meet it."""
        S, c = self.S, self.c
        if c.joint_q("rdoor_" + rack) < 0.3:
            return
        o, yaw = S.rack_pose(rack)
        ex = rz(yaw) @ np.array([1.0, 0.0, 0.0])
        b = S.base()
        lx = float((np.array([b[0], b[1]]) - o) @ ex[:2])
        hinge_x = -(H.RACK_W / 2 - 0.01)
        shift = min(most, room - (lx - hinge_x))
        if shift < 0.03:
            return
        S.step_to(b[0] + ex[0] * shift, b[1] + ex[1] * shift, b[2], timeout=8.0)

    def _take_tool(self, side, tool, near_rack=True):
        """The tool in `side` (a tool held there goes back to the belt first); belt work happens a step back from the
        rack, clear of its doors and posts."""
        S = self.S
        if S.holds(side) == "tool_" + tool:
            return True
        if near_rack:
            self.step_back()
        if S.holds(side) is not None and S.holds(side).startswith("tool_"):
            S.stow_tool(side, S.holds(side))
        return S.draw_tool(side, "tool_" + tool, approach="top") is not None

    def _at_face(self, tool, face_id, port, side, work):
        """The tool to an end face, `work(q, cen, n)` there, and back out. `port`: the MPO port whose (empty)
        receptacle the face is, worked on from the table's stance; None: a connector face held up in the other hand."""
        S, c = self.S, self.c
        if port is None and S.holds(side) != "tool_" + tool:
            # a connector held up in the other hand is worked on a step out from the rack (its door leaf and posts
            # are beside the stance where it came out)
            dev = face_id.split(":", 1)[-1].split("/")[0]
            rk = S.rack_of(dev)
            self.clear_of_rack(rk)
            self.clear_of_door(rk)
        if not self._take_tool(side, tool, near_rack=port is not None):
            return None
        if port is not None:
            roll = self.go_tool(port)
            if roll is None:
                return None
            if tool == "cleaner":
                roll = self.last_tool_rec.get("roll_cleaner", roll)
        else:
            roll = 0.0
            held = [sd for sd in ("left", "right") if (S.holds(sd) or "").startswith("px_")]
            if held:                            # a tool change moved the body: the connector presented again
                S.posture(stand=0.78, waist=(0.0, 0.0, 0.10), settle=1.0)
                self.present(held[0])
        cen, n = self.face_of(face_id)
        if cen is None:
            S.log("[ref] face %s not reachable" % face_id)
            return None
        best = None
        for k in range(8) if port is None else (0,):
            rr = roll + k * math.pi / 4
            q, Rt_new = self._tool_pose(side, tool, n, rr)
            tgt = self._site_for_tip(side, tool, cen + n * 0.05, Rt_new)
            if port is None:
                qq, ep, er = c.ctrl.ik.solve_global(c.d.qpos.copy(), side, tgt, q, c.d.qpos[c.robot.arm_q[side]].copy())
                sc = ep + 0.05 * er
                if best is None or sc < best[0]:
                    best = (sc, q, tgt, rr)
            else:
                best = (0.0, q, tgt, rr)
        q, tgt, roll = best[1], best[2], best[3]
        if port is not None:                    # in from outside the rack along the face's axis (the study's waypoints)
            Rt_new = self._tool_pose(side, tool, n, roll)[1]
            seed = self.last_tool_rec.get("arm_q0") if tool == "scope" else None
            S.arm_to(side, self._site_for_tip(side, tool, cen + n * 0.30, Rt_new), q, seed)
            S.reach(side, self._site_for_tip(side, tool, cen + n * 0.15, Rt_new), q, timeout=4.0)
        else:
            S.reach(side, self._site_for_tip(side, tool, cen + n * 0.12, self._tool_pose(side, tool, n, roll)[1]), q, timeout=4.0)
        S.reach(side, tgt, q)
        if port is None:                        # a connector held up in the other hand: let the body settle first
            self.wait_still(face_id)
        out = work(q, cen, n)
        cen, n = self.face_of(face_id)
        S.reach(side, S.site(side) + n * 0.06, q, timeout=3.0)
        return out

    def wait_still(self, face_id, tol=0.0002, t_max=4.0):
        """Wait until an end face stays put (within tol over 0.3 s): a connector held in one hand sways with the body
        for a while after the other arm has moved."""
        S, c = self.S, self.c
        t0 = c.sim_time
        prev, _ = self.face_of(face_id)
        while prev is not None and c.sim_time - t0 < t_max and not c.done:
            S.run(30)
            cur, _ = self.face_of(face_id)
            if cur is None or float(np.linalg.norm(cur - prev)) < tol:
                break
            prev = cur

    def _work_side(self, port, side):
        if side is not None:
            return side
        if port is not None:
            return self.tool_side(port)
        held = [sd for sd in ("left", "right") if (self.S.holds(sd) or "").startswith("px_")]
        return "left" if held == ["right"] else "right"

    def scope(self, face_id, port=None, side=None):
        """One picture of an end face with the probe. Returns the capture record."""
        S, c = self.S, self.c
        side = self._work_side(port, side)
        S.phase = "scope %s" % face_id

        def work(q, cen, n):
            for _ in range(5):                  # (a connector held in the other hand drifts: settle again, then shoot)
                lat, axl = self._tool_on_face(side, "scope", cen, n, 0.0005, q, face_id=face_id)
                if lat <= 0.00035:
                    break
                if port is None:
                    self.wait_still(face_id, t_max=6.0)
            c.tool("scope")
            cap = c.cluster.last_capture
            if not cap.get("sharp"):
                S.log("[ref]   capture saw lateral %s mm angle %s deg face %s" % (cap.get("lateral_mm"), cap.get("angle_deg"), cap.get("resolved")))
            S.log("[ref] scope %s: lateral %.2f mm axial %.2f mm angle %.1f deg -> %s sharp %s" % (
                face_id, 1000 * lat, 1000 * axl, self.last_tool_angle, cap.get("id"), cap.get("sharp")))
            return cap
        return self._at_face("scope", face_id, port, side, work)

    def clean(self, face_id, port=None, clicks=2, side=None):
        S, c = self.S, self.c
        side = self._work_side(port, side)
        S.phase = "clean %s" % face_id

        def work(q, cen, n):
            n0 = len([e for e in c.events if e[0] == "clean"])
            for _ in range(clicks):
                for depth in (0.004, 0.0, -0.003, 0.004):
                    lat, axl = self._tool_on_face(side, "cleaner", cen, n, depth, q, face_id=face_id)
                    if depth < 0:
                        S.log("[ref]   cleaner at depth %.1f mm: lateral %.2f mm axial %.2f mm angle %.1f deg" % (
                            1000 * depth, 1000 * lat, 1000 * axl, self.last_tool_angle))
            done = len([e for e in c.events if e[0] == "clean"]) - n0
            S.log("[ref] clean %s: %d clicks" % (face_id, done))
            return done
        return self._at_face("cleaner", face_id, port, side, work) or 0

    def present(self, side):
        """The connector held in `side` up in front of the chest, its end face towards the other hand."""
        S, c = self.S, self.c
        px = S.holds(side)
        b = c.pool.body[px]
        base = S.base()
        Rb = rz(base[2])
        sgn = -1.0 if side == "right" else 1.0                       # the hand's side of the body (base y)
        p_tgt = np.array([base[0], base[1], 0.0]) + Rb @ np.array([0.30, 0.02 * sgn, 1.02])
        # the connector's face looks along its -y: towards the other hand (across the body), level
        y_p = Rb @ np.array([0.0, sgn, 0.0])
        z_p = np.array([0.0, 0.0, 1.0])
        Rp_new = np.stack([np.cross(y_p, z_p), y_p, z_p], 1)
        Rp = c.d.xmat[b].reshape(3, 3)
        Rs_new = Rp_new @ Rp.T @ S.site_R(side)
        off = S.site_R(side).T @ (c.d.xpos[b] - S.site(side))      # the connector in the hand's frame
        S.reach(side, p_tgt - Rs_new @ off, mat_quat(Rs_new), timeout=5.0)
        return mat_quat(Rs_new)

    def put_tool_back(self, side=None):
        S = self.S
        ok = True
        for sd in ((side,) if side else ("left", "right")):
            name = S.holds(sd)
            if name and name.startswith("tool_"):
                ok = S.stow_tool(sd, name) and ok
        return ok
