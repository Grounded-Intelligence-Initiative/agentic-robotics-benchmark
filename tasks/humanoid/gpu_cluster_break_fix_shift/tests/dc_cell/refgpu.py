"""The reference program's GPU-module work in an extended tray (grader side): the four screws with the long-reach
driver (from the belt), a module out of its slot and back onto its guide pins, from the stance tables of ref_data/
(rack frame)."""

from __future__ import annotations

import math

import numpy as np

from . import parts as P
from .refskills import TUCK, load_table, mat_quat, mulq, quat_mat, rotq, rz, wrap
from .robot import ARM_HOME

ORDER = (0, 3, 1, 2)                 # the vendor's 1-4-2-3 (screw indices 0..3)
DRIVER = load_table("gpu_driver.json")["slots"]
LIFT = {"left": load_table("gpu_lift_left.json")["slots"], "right": load_table("gpu_lift_right.json")["slots"]}


class GpuWork:
    def __init__(self, S):
        self.S = S
        self.c = S.c
        self.rel = None              # the driver's pose in the hand (site frame) after the draw
        self.yaw = None              # the driver's turn about its axis (world) for this module
        self.slot = None
        self.debug = False

    # ------------------------------------------------------------------------------------------------ the driver
    def _site_for(self, p_tip, yaw, lift=0.0):
        tip = np.asarray(P.TOOLS["driver"]["tip"])
        Rt = rz(yaw)
        pt = np.asarray(p_tip) - Rt @ tip + np.array([0.0, 0.0, lift])
        Rs = Rt @ self.rel[1].T
        return pt - Rs @ self.rel[0], mat_quat(Rs)

    def _put_tip(self, p_tip, lift=0.0, tol=0.004, timeout=8.0):
        ps, q = self._site_for(p_tip, self.yaw, lift)
        return self.S.reach("right", ps, q, tol=tol, timeout=timeout)

    def _stance(self, host, slot, table, group=0):
        e = table[slot.lower()]
        if "groups" in e:                      # the driver table: one stance per group of screws
            e = e["groups"][group]
        rack = self.S.rack_of(host)
        return e, self.S.to_world(rack, e["lx"], e["ly"], e["lyaw"])

    @staticmethod
    def groups(slot):
        return DRIVER[slot.lower()]["groups"]

    def _replan_yaw(self, port, e, rack_yaw):
        """The driver's turn for this grasp: every screw head and the lift point from the table's posture."""
        c, R_ = self.c, self.c.robot
        from .robot import PELVIS_HEIGHT
        q = c.d.qpos.copy()
        q[R_.base_q[3]] = e["stand"] - PELVIS_HEIGHT
        lt = c.ctrl.legs.targets(e["stand"])
        for sd in ("left", "right"):
            q[R_.leg_q[sd]] = lt
        q[R_.waist_q] = [e.get("waist_yaw", 0.0), 0.0, e["pitch"]]
        heads = [c.screws.head_world(port, k)[0] + np.array([0, 0, -0.001]) for k in range(4)]
        mine = [i for i in ORDER if i in e.get("screws", (0, 1, 2, 3))]
        base = wrap(e["driver_yaw"] + rack_yaw)
        best = None
        for d in np.radians(np.arange(-60, 61, 15)):
            yaw = base + d
            worst, qa = 0.0, q[R_.arm_q["right"]].copy()
            for k, h in enumerate([heads[mine[0]] + np.array([0, 0, 0.16])] + [heads[i] for i in mine]):
                ps_, qq_ = self._site_for(h, yaw)
                if k == 0:
                    qa, ep, er = c.ctrl.ik.solve_global(q, "right", ps_, qq_, qa, iters=40)
                else:
                    qa, ep, er = c.ctrl.ik.solve(q, "right", ps_, qq_, qa, iters=40, w_home=0.0)
                worst = max(worst, ep + 0.05 * er)
            if best is None or worst < best[0] - 2e-4 or (abs(worst - best[0]) <= 2e-4 and abs(d) < abs(best[2])):
                best = (worst, yaw, d)
        self.yaw = best[1]
        return best[0]

    def to_driver_stance(self, host, slot, group=0):
        S, c = self.S, self.c
        port = "%s/%s" % (host, slot.lower())
        S.phase = "driver stance %s" % port
        e, (x, y, yaw) = self._stance(host, slot, DRIVER, group)
        if S.holds("right") == "tool_driver":          # the driver up out of the tray before the walk
            cur = S.site("right")
            S.reach("right", np.array([cur[0], cur[1], cur[2] + 0.20]), mat_quat(S.site_R("right")), timeout=3.0)
            S.posture(stand=0.74, waist=(0.0, 0.0, 0.10), settle=1.0)
        S.approach(x, y, yaw, work=("right",) if S.holds("right") == "tool_driver" else ())
        S.posture(stand=0.74, waist=(0.0, 0.0, 0.10))
        if S.holds("right") != "tool_driver":
            if S.draw_tool("right", "tool_driver", approach="top") is None:
                return False
        ps, Rs = S.site("right"), S.site_R("right")
        pt, Rt = c.body_pose("tool_driver")
        self.rel = (Rs.T @ (pt - ps), Rs.T @ Rt)
        S.arms(q_left=ARM_HOME["left"], settle=0.8)           # the free arm as the stance tables assume it
        _, rack_yaw = S.rack_pose(S.rack_of(host))
        worst = self._replan_yaw(port, e, rack_yaw)
        h0 = c.screws.head_world(port, e.get("screws", [0])[0])[0]
        cur = S.site("right")
        S.reach("right", np.array([cur[0], cur[1], h0[2] + 0.45]), None)          # up first: over the fan wall
        S.posture(stand=e["stand"], waist=(e.get("waist_yaw", 0.0), 0.0, e["pitch"]), settle=1.2)
        ps, q = self._site_for(h0, self.yaw, 0.16)
        self.arm_over("right", e, ps, q)          # into the stance study's own arm configuration over the screws
        self._put_tip(h0, lift=0.16)
        self.slot = port
        self.group = group
        if worst > 0.004:
            S.log("[ref] %s: the driver reaches the screws only to %.1f mm" % (port, 1000 * worst))
        return S.holds("right") == "tool_driver"

    def drive(self, port, k, mode, torque):
        S, c = self.S, self.c
        h, ax = c.screws.head_world(port, k)
        target = h + np.array([0, 0, -0.001])
        self._put_tip(h, lift=0.16)
        self._put_tip(target, lift=0.03)
        self._put_tip(target)
        ps, q = self._site_for(target, self.yaw)
        cmd = S.site("right")
        for _ in range(6):                     # the tip onto the head (the reference sees the true tip; an agent its camera)
            tip, _a = c.tools.tip_axis("tool_driver")
            e = target - tip
            if np.linalg.norm(e[:2]) < 0.0004:
                break
            cmd = cmd + e
            S.settle("right", cmd, q)
        c.tool("driver", mode=mode, torque=torque, seconds=2.5)
        S.run(260)
        res = c.tool("driver", action="status").get("result")
        self._put_tip(h, lift=0.03)
        self._put_tip(h, lift=0.16)
        return res

    def _group_of(self, slot, k):
        for g, e in enumerate(self.groups(slot)):
            if k in e["screws"]:
                return g
        return 0

    def unscrew(self, host, slot):
        port = "%s/%s" % (host, slot.lower())
        S = self.S
        res = []
        for g, e in enumerate(self.groups(slot)):
            if not self.to_driver_stance(host, slot, g):
                return False
            for k in ORDER:
                if k in e["screws"]:
                    S.phase = "loosen %s screw %d" % (port, k + 1)
                    res.append(self.drive(port, k, "loosen", 0.9))
        ok = self.c.screws.loose(port)
        self.S.log("[ref] %s: screws out %s (%s)" % (port, ok, res))
        return ok

    def screw_down(self, host, slot):
        port = "%s/%s" % (host, slot.lower())
        if not self.to_driver_stance(host, slot, self._group_of(slot, ORDER[0])):
            return False
        res = []
        for torque in (0.3, 0.9):
            for k in ORDER:                    # 1-4-2-3 in both passes (a slot with two stances walks between them)
                g = self._group_of(slot, k)
                if g != self.group and not self.to_driver_stance(host, slot, g):
                    return False
                self.S.phase = "tighten %s screw %d %.1f" % (port, k + 1, torque)
                res.append(self.drive(port, k, "tighten", torque))
        ok = self.c.screws.fastened(port) and not self.c.screws.damaged[port]
        self.S.log("[ref] %s: fastened %s, damaged %s (%s)" % (port, self.c.screws.fastened(port), self.c.screws.damaged[port], res))
        return ok

    def leave_driver(self):
        S = self.S
        if S.holds("right") != "tool_driver":
            return True
        S.phase = "driver back to the belt"
        qh = mat_quat(S.site_R("right"))
        S.reach("right", S.site("right") + np.array([0, 0, 0.12]), qh)
        S.posture(stand=0.74, waist=(0.0, 0.0, 0.10), settle=1.2)
        b = S.base()
        S.reach("right", S.site("right") + rz(b[2]) @ np.array([-0.20, 0.0, 0.0]), qh)
        S.retreat(0.30)                    # away from the tray before the hand goes down to the belt
        ok = S.stow_tool("right", "tool_driver")
        S.tuck()
        return ok

    def hand_out(self, side, q):
        """A hand out of the tray the way it came in: straight up clear of the heatsinks and the walls (the arm on
        the branch it went in with), the torso upright with the hand held there, then home (a step back first when
        the arm cannot come in where the robot stands)."""
        S = self.S
        S.reach(side, S.site(side) + np.array([0, 0, 0.08]), q, timeout=3.0)
        S.reach(side, S.site(side) + np.array([0, 0, 0.20]), q, timeout=3.0)
        S.posture(stand=0.74, waist=(0.0, 0.0, 0.10), settle=1.0)
        S.tuck()

    # ------------------------------------------------------------------------------------------------ the module
    def arm_over(self, side, e, p, ql, lower=()):
        """The lifting / driving arm into the stance study's own configuration, the hand at p (RefSkills.arm_to with
        the table's arm at its first work point); when that configuration does not reach p (the study measured the
        work point, not the space above it), the first of the `lower` points it reaches."""
        S = self.S
        seed = e.get("arm_q0")
        if seed is not None and lower:
            sq = np.asarray(seed, float)
            for pt in [p] + list(lower):
                _, ep, er = S.c.ctrl.ik.solve(S.c.d.qpos.copy(), side, pt, ql, sq, iters=80, q_rest=sq)
                if ep <= 0.03 and er <= 0.12:
                    return S.arm_to(side, pt, ql, seed)
        return S.arm_to(side, p, ql, seed)

    def lift_hand(self, host, slot):
        """The hand for lifting this slot's module: the right one in the column next to the cover's hinge."""
        e = LIFT["right"][slot.lower()]
        return "right" if e["port_lx"] < -0.10 else "left"

    def module_out(self, host, slot, side=None):
        """Lift the module (screws out) 30 mm clear of its pins (with `side`'s hand, by default the slot's lifting
        hand); returns (side, proxy) or (None, None)."""
        S, c = self.S, self.c
        port = "%s/%s" % (host, slot.lower())
        side = side or self.lift_hand(host, slot)
        other = "left" if side == "right" else "right"
        S.phase = "module out %s (%s hand)" % (port, side)
        e, (x, y, yaw) = self._stance(host, slot, LIFT[side])
        rack = S.rack_of(host)
        _, rack_yaw = S.rack_pose(rack)
        S.approach(x, y, yaw, work=(side,))        # the lifting hand up before the robot steps in by the tray
        S.arms(**{"q_" + other: ARM_HOME[other]}, settle=0.8)
        S.raise_hand(side)                         # up before leaning over the tray
        S.posture(stand=e["stand"], waist=(e.get("waist_yaw", 0.0), 0.0, e["pitch"]), settle=1.5)
        a, b, r = S.feature(port)
        g = 0.5 * (a + b)
        ql = mulq(rotq((0, 0, 1), rack_yaw), np.asarray(e["hand_quat"]))
        self.arm_over(side, e, g + np.array([0, 0, 0.20]), ql, lower=(g + np.array([0, 0, 0.14]), g + np.array([0, 0, 0.09])))
        S.reach(side, g + np.array([0, 0, 0.07]), ql)
        S.reach(side, g, ql)
        if S.grip(side, True) is None:
            S.grip(side, False)
            S.log("[ref] %s: the %s hand could not take the module" % (port, side))
            return None, None
        px = S.holds(side)
        for dz in (0.005, 0.010, 0.015, 0.020, 0.030, 0.06):
            S.reach(side, g + np.array([0, 0, dz]), ql, check_held=False)
            if S.holds(side) != px:
                S.log("[ref] %s: the module slipped from the %s hand" % (port, side))
                return None, None
        S.log("[ref] %s: module out (%s, state %s)" % (port, px, c.pool.state.get(px)))
        return side, px

    def lift_clear(self, side):
        """A module just lifted off its pins taken straight up out of the tray (clear of the heatsinks around it and
        the tray's walls), slowly: a 1.8 kg module leaves the 30 N grip little margin."""
        S = self.S
        q = mat_quat(S.site_R(side))
        p = S.site(side)
        # straight up in short moves, each dry-run with the module (a slot by the cover's hinge has the open cover
        # leaning over it: there the way out bends towards the robot); the dry runs replay the gentle streamed move
        for dz in (0.06, 0.12, 0.20, 0.26):
            if not S.reachable(side, p + np.array([0.0, 0.0, dz]), q, tol=0.015, why="lift clear"):
                break                              # as high as the arm reaches leaning: carry() straightens up
            S.reach(side, p + np.array([0.0, 0.0, dz]), q, timeout=4.0)

    def module_in(self, host, slot, side, px):
        """A held module down onto the slot's pins (closed loop on the module's own pose), let go: it seats."""
        S, c = self.S, self.c
        port = "%s/%s" % (host, slot.lower())
        S.phase = "module in %s (%s hand)" % (port, side)
        e, (x, y, yaw) = self._stance(host, slot, LIFT[side])
        rack = S.rack_of(host)
        _, rack_yaw = S.rack_pose(rack)
        if math.hypot(S.base()[0] - x, S.base()[1] - y) > 0.03:
            S.approach(x, y, yaw, work=(side,), carry="base")
        other = "left" if side == "right" else "right"
        if S.holds(other) is None:
            S.arms(**{"q_" + other: ARM_HOME[other]}, settle=0.8)
        o, R, ax = c.ports.world(port)
        hp = S.site(side)
        if S.site(side)[2] < 1.0 and math.hypot(hp[0] - o[0], hp[1] - o[1]) > 0.20:
            S.raise_hand(side)                     # a module carried low (from the cart) comes up before the lean
        S.posture(stand=e["stand"], waist=(e.get("waist_yaw", 0.0), 0.0, e["pitch"]), settle=1.5)
        fa = P.KINDS["gpu"]["features"][0]
        g = o + R @ (0.5 * (np.asarray(fa[1]) + np.asarray(fa[2])))
        ql = mulq(rotq((0, 0, 1), rack_yaw), np.asarray(e["hand_quat"]))
        b = c.pool.body[px]
        # the module's pose relative to the hand: carry it to over the slot, then close the loop on the module itself
        self.arm_over(side, e, g + np.array([0, 0, 0.20]), ql, lower=(g + np.array([0, 0, 0.14]), g + np.array([0, 0, 0.09])))
        for dz in (0.14, 0.08, 0.04):              # down in steps (a heavy module, a 30 N grip)
            if S.site(side)[2] < g[2] + dz + 0.01:
                continue                           # (already below: arm_over brought it lower)
            S.reach(side, g + np.array([0, 0, dz]), ql, timeout=4.0, check_held=False)
        corr = np.zeros(3)
        for _ in range(6):
            dv = c.d.xpos[b] - o
            off = dv - (dv @ ax) * ax
            if self.debug:
                S.log("[gpu] over the slot: module off %.1f mm (height %.3f), hand %.1f mm from its target, holds %s" % (
                    1000 * np.linalg.norm(off), float(dv @ ax), 1000 * np.linalg.norm(S.site(side) - (g + np.array([0, 0, 0.025]) + corr)),
                    S.holds(side)))
            if np.linalg.norm(off) < 0.0004:
                break
            if S.holds(side) != px or np.linalg.norm(off) > 0.03:
                S.log("[ref] %s: the module is not where the hand is (off %.0f mm, holds %s)" % (
                    port, 1000 * np.linalg.norm(off), S.holds(side)))
                break
            corr -= off
            S.reach(side, g + np.array([0, 0, 0.025]) + corr, ql, tol=0.0003, timeout=3.0, check_held=False)
        # down onto the pins (they take a module within 1.5 mm, 2 deg); let go there, it seats under its own weight (a
        # push into the seat would only load the grip)
        for dz in (0.020, 0.015, 0.010, 0.005, 0.0):
            if dz <= 0.005 and c.pool.state.get(px) == "slot":
                break
            S.reach(side, g + np.array([0, 0, dz]) + corr, ql, check_held=False)
            if self.debug:
                dv = c.d.xpos[b] - o
                S.log("[gpu] dz %.3f: module lat %.2f mm, height %.4f, state %s" % (
                    dz, 1000 * np.linalg.norm(dv - (dv @ ax) * ax), float(dv @ ax), c.pool.state.get(px)))
        S.grip(side, False)
        S.run(50)
        self.hand_out(side, ql)
        ok = c.ports.occupied[port] and c.ports.proxy[port] is None
        S.log("[ref] %s: module in, seated %s" % (port, ok))
        return ok
