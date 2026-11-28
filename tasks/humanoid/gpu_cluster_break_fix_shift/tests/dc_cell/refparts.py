"""The reference program's part handling: spares off the parts cart, failed parts into the RMA tote, and the hot swaps
of a power supply (its cord out and back in) and of a fan module (one hand holds the spare, the other works)."""

from __future__ import annotations

import math

import mujoco
import numpy as np

from . import parts as P
from . import scene as SC
from .refskills import WALK_ARMS, DryRun, load_table, mat_quat, mulq, quat_from_axes, quat_mat, rotq, rz

CART_TABLE = load_table("cart.json")       # tools/cart_stance2.py: per spare, hand and grasp, from its grip point
TOTE = SC.TOTE_XY                           # the RMA tote's centre (on its stand south of the cart)
TOTE_Z = SC.TOTE_Z                          # the tote's bottom (the stand's top); its rim 0.30 m higher


def other(side):
    return "left" if side == "right" else "right"


def _rack_R(c, port):
    dev = port.split("/")[0]
    rack = (dev[4].upper() + dev[5:7]) if dev.startswith("gpn-") else (dev[5].upper() + dev[6:8])
    b = mujoco.mj_name2id(c.m, mujoco.mjtObj.mjOBJ_BODY, "rack_" + rack)
    return c.d.xmat[b].reshape(3, 3).copy()


def grasp_key(hxp, hyp, gs):
    return "%+.2f%+.2f%+.2f/%+.2f%+.2f%+.2f/%.2f" % (tuple(float(v) + 0.0 for v in np.round(hxp, 2)) +
                                                     tuple(float(v) + 0.0 for v in np.round(hyp, 2)) + (gs,))


def part_grasp(c, port, side):
    """How the stance tables hold the part of `port` in `side`'s hand (refrear's part table, refgpu's lift table):
    the hand's x and y axes in the part's frame (the part seated: the port's frame), the grip point along its handle
    (0..1) and a key naming that grasp; None when the table has no stance for that hand."""
    from .refgpu import LIFT
    from .refrear import PART_TABLE, RearWork
    kind = c.ports.rec[port]["kind"]
    o, R, ax = c.ports.world(port)
    Rr = _rack_R(c, port)
    if kind == "gpu":
        e = LIFT[side].get(port.split("/")[1].lower())
        if not e:
            return None
        Rq = quat_mat(e["hand_quat"])
        hx, hy, gs = Rq[:, 0], Rq[:, 1], 0.5
    else:
        e = (PART_TABLE.get(RearWork.table_key(port)) or {}).get(side)
        if not e:
            return None
        hx, hy, gs = np.asarray(e["hand_x"], float), np.asarray(e["hand_y"], float), float(e.get("grip_s", 0.55))
    hxp, hyp = R.T @ (Rr @ hx), R.T @ (Rr @ hy)
    return hxp, hyp, gs, grasp_key(hxp, hyp, gs)


def grasp_classes(c):
    """kind -> side -> {grasp key: (hand x, hand y in the part frame, grip point)} over every port the tables hold."""
    from .refrear import PART_TABLE
    out = {}
    ports = []
    for key in PART_TABLE:
        ports.append(("gpn-a03/" + key.split("/")[1]) if key.startswith("node/") else key)
    ports += ["gpn-a03/sxm%d" % i for i in range(1, 9)]
    for port in ports:
        if port not in c.ports.rec:
            continue
        kind = c.ports.rec[port]["kind"]
        if kind not in ("fan", "psu", "osfp_n", "osfp_s", "gpu"):
            continue
        for side in ("left", "right"):
            gr = part_grasp(c, port, side)
            if gr is None:
                continue
            out.setdefault(kind, {}).setdefault(side, {})[gr[3]] = (list(map(float, gr[0])), list(map(float, gr[1])), gr[2])
    return out


class PartWork:
    def __init__(self, S, rear):
        self.S, self.c, self.rear = S, S.c, rear

    # ------------------------------------------------------------------------------------------------ the cart
    def spare_on_cart(self, kind):
        """A spare of this kind lying on the cart (its proxy), or None."""
        c = self.c
        locs = c._locations()
        for px in sorted(c.pool.state):
            if c.pool.kind[px] != kind or c.pool.state[px] != "free":
                continue
            inst = c.pool.instance.get(px)
            if inst is not None and locs.get(inst) == "cart":
                return px
        return None

    def spot_of(self, px):
        """The cart spot (named after the spare that lay there at the start) nearest to where a part lies now."""
        c = self.c
        p = c.d.xpos[c.pool.body[px]]
        best = None
        for name, rec in c.pool.rec.items():
            st = rec.get("start")
            if not rec.get("spare") or not st or c.pool.kind[name] != c.pool.kind[px]:
                continue
            d = math.hypot(p[0] - st["pos"][0], p[1] - st["pos"][1])
            if best is None or d < best[0]:
                best = (d, name)
        return best[1] if best else px

    def spare_rec(self, px, side, port):
        """The cart table's stance for taking spare px into `side`'s hand with the grasp `port` needs (the way the
        part goes into that port), and that grasp; (None, None) when there is none."""
        gr = part_grasp(self.c, port, side)
        if gr is None:
            return None, None
        rec = ((CART_TABLE.get(self.spot_of(px)) or {}).get(side) or {}).get(gr[3])
        return rec, gr

    def pick_spare_side(self, kind, port):
        """The hand that brings a spare for `port` (the other one takes the failed part out): one with a cart stance
        for that port's grasp, the left first."""
        px = self.spare_on_cart(kind)
        if px is None:
            return None
        for side in ("left", "right"):
            rec, gr = self.spare_rec(px, side, port)
            if rec and part_grasp(self.c, port, other(side)) is not None:
                return side
        return None

    def take_spare(self, kind, side, port):
        """A spare of this kind off the cart into `side`'s hand, held the way it goes into `port` (the cart table's
        stance for that grasp), lifted, drawn back off its spot and brought to the carrying pose. Returns the proxy
        held or None."""
        S, c = self.S, self.c
        px = self.spare_on_cart(kind)
        if px is None:
            S.log("[ref] no %s on the cart" % kind)
            return None
        rec, gr = self.spare_rec(px, side, port)
        if not rec:
            S.log("[ref] no %s-hand cart stance for %s with %s's grasp" % (side, px, port))
            return None
        S.phase = "take %s" % px
        a, b, r = S.feature(px)
        g = a + rec["grip_s"] * (b - a)
        S.approach(g[0] + rec["dx"], g[1] + rec["dy"], rec["yaw"],
                   posture=(rec["stand"], (rec["waist_yaw"], 0.0, rec["pitch"])))
        a, b, r = S.feature(px)
        g = a + rec["grip_s"] * (b - a)
        hx = np.asarray(rec["hand_x"], float)
        q = quat_from_axes(hx, rec["hand_y"])
        S.arm_to(side, g - hx * 0.08, q, rec.get("arm_q0"))
        S.reach(side, g - hx * 0.02, q)
        S.reach(side, g, q, tol=0.002)
        if S.grip(side, True) != px:
            S.grip(side, False)
            S.reach(side, g - hx * 0.08, q, timeout=3.0)
            S.log("[ref] could not take %s off the cart" % px)
            return None
        up = np.array([0.0, 0.0, 1.0])
        S.reach(side, g + up * 0.03, q, timeout=3.0)
        if kind == "gpu":
            S.reach(side, g + up * 0.15, q, timeout=3.0)
        else:
            hh = np.array([hx[0], hx[1], 0.0])
            hh /= max(1e-6, float(np.linalg.norm(hh)))
            S.reach(side, g + up * 0.03 - hh * 0.12, q, timeout=3.0)
        self.carry(side)
        if S.holds(side) != px:
            S.log("[ref] %s slipped out of the hand leaving the cart" % px)
            return None
        return px

    def carry(self, side):
        """What `side` holds brought to its carrying pose: the torso upright, the hand in front of the chest with the
        part as it is held or turned to point up; of the poses whose move is clear (dry run), the one that stands
        out least of the walking footprint (sideways first: doors and aisles are narrow)."""
        S, c = self.S, self.c
        if S.holds(side) is None:
            return False
        heavy = P.KINDS.get(self.c.pool.kind.get(S.holds(side)), {}).get("mass", 0.0) > 1.0
        if heavy:
            # the arm locked to the torso while it rises (a hand held still in the room would leave the arm
            # stretched out under a heavy part as the shoulder rises and goes back), gently
            S.c.command({side + "_arm": {"q": [float(v) for v in S.c.d.qpos[S.c.robot.arm_q[side]]]}})
            S.run(30)
        S.posture(stand=0.74, waist=(0.0, 0.0, 0.05), settle=1.0, steps=8 if heavy else 1)
        b = S.base()
        Rb = rz(b[2])
        sg = 1.0 if side == "left" else -1.0
        q0 = mat_quat(S.site_R(side))
        lat_axis = Rb @ np.array([0.0, 1.0, 0.0])
        quats = [q0] + [mulq(rotq(lat_axis, a), q0) for a in (-math.pi / 2, -math.pi / 3)]
        best = None
        for attempt in range(2):
            b = S.base()
            Rb = rz(b[2])
            lat_axis = Rb @ np.array([0.0, 1.0, 0.0])
            quats = [q0] + [mulq(rotq(lat_axis, a), q0) for a in (-math.pi / 2, -math.pi / 3)]
            for fwd, out, z in ((0.22, 0.06, 1.05), (0.24, 0.12, 1.05), (0.20, 0.16, 1.10), (0.26, 0.0, 1.00),
                                (0.18, 0.10, 1.15), (0.28, 0.16, 0.98)):
                p = np.array([b[0], b[1], 0.0]) + Rb @ np.array([fwd, sg * out, 0.0])
                p[2] = z
                for qi, q in enumerate(quats):
                    D = DryRun(S)
                    D.baseline((side,))
                    D.active = (side,)
                    if heavy and S.very_heavy(side):
                        # a GPU module goes there by a joint move (streamed; it may change the arm's branch, which a
                        # pose move of near targets cannot): the configuration from the global IK, the move dry-run
                        qa, ep, er = c.ctrl.ik.solve_global(c.d.qpos.copy(), side, p, q, c.d.qpos[c.robot.arm_q[side]].copy())
                        if ep > 0.01:
                            continue
                        if S.try_moves([("arm", side, {"q": [float(v) for v in qa]})], (side,), held=True) is not None:
                            continue
                        D = S._last_dry
                        lat, fw = D.extent(side)
                        score = 3.0 * max(0.0, lat - 0.21) + max(0.0, fw - 0.34) + 0.01 * qi
                        if best is None or score < best[0]:
                            best = (score, p, q, lat, fw, qa)
                        continue
                    D.command({side + "_arm": S._tgt(p, q)})
                    if D.run(lambda D_: D_.arm_done(side), (side,)) is not None:
                        continue
                    ep, er = D.sim.arm_ik_err[side]
                    if ep > 0.01:
                        continue
                    lat, fw = D.extent(side)
                    score = 3.0 * max(0.0, lat - 0.21) + max(0.0, fw - 0.34) + 0.01 * qi
                    if best is None or score < best[0]:
                        best = (score, p, q, lat, fw, None)
            if best is not None or attempt:
                break
            S.retreat(0.30)                     # too close to where it was taken (a cart, a rack): a step back first
        if best is None:
            S.log("[ref] carry %s: no clear carrying pose; the hand stays" % side)
            return False
        if best[5] is not None:
            S.step = "carry %s (joint move)" % side
            S._stream_q(side, best[5])
            S.c.command({side + "_arm": {"q": [float(v) for v in best[5]]}})
            S.run(60)
        else:
            S.reach(side, best[1], best[2], timeout=4.0, checked=False)
        if best[0] > 0.05:
            S.log("[ref] carry %s: the load stands out %.2f m sideways, %.2f m ahead" % (side, best[3], best[4]))
        return True

    def put_on_cart(self, side, spot, port):
        """The part in `side` (held the way `port` holds it) set down where the spare `spot` lay at the start, in the
        spare's own orientation (the cart table's stance and hand pose for that grasp), let go."""
        S, c = self.S, self.c
        px = S.holds(side)
        if px is None:
            return True
        gr = part_grasp(c, port, side)
        rec = ((CART_TABLE.get(spot) or {}).get(side) or {}).get(gr[3]) if gr else None
        if not rec:
            S.log("[ref] no %s-hand cart stance for the spot %s with %s's grasp" % (side, spot, port))
            return False
        S.phase = "cart %s" % px
        st = c.pool.rec[spot]["start"]
        Rsp = quat_mat(st["quat"])
        # the grip point where the part will lie: the spare's grip point at its start pose
        fa = np.asarray(P.KINDS[c.pool.kind[px]]["features"][0][1], float)
        fb = np.asarray(P.KINDS[c.pool.kind[px]]["features"][0][2], float)
        g = np.asarray(st["pos"], float) + Rsp @ (fa + rec["grip_s"] * (fb - fa)) + np.array([0.0, 0.0, 0.006])
        hx = np.asarray(rec["hand_x"], float)
        q = quat_from_axes(hx, rec["hand_y"])
        S.approach(g[0] + rec["dx"], g[1] + rec["dy"], rec["yaw"], carry="base",
                   posture=(rec["stand"], (rec["waist_yaw"], 0.0, rec["pitch"])))
        up = np.array([0.0, 0.0, 1.0])
        if c.pool.kind[px] == "gpu":
            S.arm_to(side, g + up * 0.15, q, rec.get("arm_qN"))
        else:
            hh = np.array([hx[0], hx[1], 0.0])
            hh /= max(1e-6, float(np.linalg.norm(hh)))
            S.arm_to(side, g + up * 0.03 - hh * 0.12, q, rec.get("arm_qN"))
        S.reach(side, g + up * 0.03, q, timeout=4.0, check_held=False)
        S.reach(side, g, q, tol=0.003, timeout=4.0, check_held=False)
        S.grip(side, False)
        S.run(80)
        S.reach(side, S.site(side) - hx * 0.08 + up * 0.04, q, timeout=3.0)
        S.posture(stand=0.74, waist=(0.0, 0.0, 0.05), settle=1.0)
        S.tuck(walk=True)
        on = c._locations().get(c.pool.instance.get(px)) == "cart"
        if not on:
            S.log("[ref] %s is not on the cart (%s)" % (px, c._locations().get(c.pool.instance.get(px))))
        return on

    def hold_in(self, side, quick=False):
        """A held part out of the way: drawn back a little (quick: only that, the posture kept for the other hand's
        work), then to the carrying pose."""
        S = self.S
        b = S.base()
        S.reach(side, S.site(side) + rz(b[2]) @ np.array([-0.08, 0.0, 0.04]), mat_quat(S.site_R(side)), timeout=3.0)
        if not quick:
            self.carry(side)

    # ------------------------------------------------------------------------------------------------ the RMA tote
    def _tote_spot(self, side, R_hand=None):
        """(dx, dy) from the tote's centre for the hand over it: where what the hand holds (as it is held, the hand
        turned as it is now) lies within the tote's walls with room to spare (a long part reaching over a wall would
        catch the wall or the cart's foot), and farthest from the parts already in it (a part let go onto another
        stands up out of the tote); of equally good spots the nearest the robot."""
        S, c = self.S, self.c
        m, d = c.m, c.d
        b = c.pool.body[S.holds(side)]
        ps, Rs = S.site(side), S.site_R(side)
        R_hand = Rs if R_hand is None else R_hand
        pts = []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != b or m.geom_group[g] != 3 or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_BOX:
                continue
            hs, Rg = m.geom_size[g], d.geom_xmat[g].reshape(3, 3)
            for sx in (-1.0, 1.0):
                for sy in (-1.0, 1.0):
                    for sz in (-1.0, 1.0):
                        pts.append(Rs.T @ (d.geom_xpos[g] + Rg @ (hs * np.array([sx, sy, sz])) - ps))
        pts = np.array(pts) if pts else np.zeros((1, 3))
        inside = set(c._tote_parts())
        others = [d.xpos[c.pool.body[p_]][:2] for p_ in c.pool.body if c.pool.instance.get(p_) in inside]
        qh = mat_quat(R_hand)
        q_arm = d.qpos[c.robot.arm_q[side]].copy()
        best = None
        for z in (TOTE_Z + 0.50, TOTE_Z + 0.58, TOTE_Z + 0.66):   # the hand as low as it reaches over the spot
            for dx in (0.24, 0.21, 0.18, 0.15, 0.12, 0.09, 0.06):
                for dy in (0.0, -0.06, 0.06, -0.12, 0.12):
                    w = np.array([TOTE[0] + dx, TOTE[1] + dy, z]) + (R_hand @ pts.T).T
                    if np.max(np.abs(w[:, 0] - TOTE[0])) > 0.25 or np.max(np.abs(w[:, 1] - TOTE[1])) > 0.16 or \
                            np.min(w[:, 2]) < TOTE_Z + 0.34:
                        continue
                    # within the arm's reach on its present branch (the moves there are streams of near targets)
                    if not S.reachable(side, np.array([TOTE[0] + dx, TOTE[1] + dy, z]), qh, tol=0.012):
                        continue
                    room = min([math.hypot(TOTE[0] + dx - o_[0], TOTE[1] + dy - o_[1]) for o_ in others] + [0.30])
                    sc = (round(room, 2), dx)
                    if best is None or sc > best[0]:
                        best = (sc, (dx, dy, z))
            if best is not None:
                break
        return None if best is None else best[1]

    def to_tote(self, side):
        """The part in `side` into the RMA tote: from the tote's north side, held over its middle, let go."""
        S, c = self.S, self.c
        px = S.holds(side)
        if px is None:
            return True
        S.phase = "tote %s" % px
        # from the tote's east side, facing west; the right hand (on the cart's side) from farther south
        # (0.62 m east of its middle: the stand stands 0.32 m out from the middle, the robot 0.25 m round its own)
        S.walk(TOTE[0] + 0.62, TOTE[1] + (0.08 if side == "left" else -0.28), math.pi)
        S.raise_hand(side, z=1.15)
        S.posture(stand=0.74, waist=(0.0, 0.0, 0.15))
        if S.holds(side) != px:
            S.log("[ref] %s left the hand on the way to the tote" % px)
            return False
        # the hand turned (about the vertical) so that what it holds fits over the tote, the way down from here
        # clear of the cart (dry run, the part included)
        q0 = mat_quat(S.site_R(side))
        choice = fallback = None
        for k, a in enumerate((0.0, math.pi / 2, -math.pi / 2, math.pi)):
            q = mulq(rotq((0.0, 0.0, 1.0), a), q0) if a else q0
            spot = self._tote_spot(side, quat_mat(q))
            if spot is None:
                if getattr(S, "debug_tote", False):
                    S.log("[tote] turn %.2f: no spot within the walls and the arm's reach" % a)
                continue
            p1 = np.array([TOTE[0] + spot[0], TOTE[1] + spot[1], spot[2] + 0.12])
            moves = ([("arm", side, S._tgt(S.site(side), q))] if a else []) + [("arm", side, S._tgt(p1, q))]
            hit = S.try_moves(moves, (side,), held=True)
            if getattr(S, "debug_tote", False):
                S.log("[tote] turn %.2f spot %s: %s" % (a, spot, hit[:3] if hit else "clear"))
            if hit is None:
                choice = (q, spot, a)
                break
            if fallback is None:
                fallback = (q, spot, a)
        if choice is None:
            choice = fallback or (q0, (0.10, 0.0, TOTE_Z + 0.62), 0.0)
            S.log("[ref] tote: no clear way down with %s; going anyway" % S.holds(side))
        q, spot, a = choice
        if a:
            S.reach(side, S.site(side), q, timeout=4.0, checked=False)
        S.reach(side, np.array([TOTE[0] + spot[0], TOTE[1] + spot[1], spot[2] + 0.12]), q, timeout=5.0)
        S.reach(side, np.array([TOTE[0] + spot[0], TOTE[1] + spot[1], spot[2]]), q, timeout=4.0, check_held=False)
        pb = c.pool.body[px]
        hp, pp = S.site(side), c.d.xpos[pb].copy()
        S.grip(side, False)
        S.run(150)
        if c.pool.instance.get(px) not in c._tote_parts():
            S.log("[ref] tote drop: the hand at %s, the part at %s (from the tote's middle), now at %s" % (
                np.round(hp - np.array([TOTE[0], TOTE[1], 0.0]), 3), np.round(pp - np.array([TOTE[0], TOTE[1], 0.0]), 3),
                np.round(c.d.xpos[pb] - np.array([TOTE[0], TOTE[1], 0.0]), 3)))
        b = S.base()
        S.reach(side, S.site(side) + np.array([0.0, 0.0, 0.10]) + rz(b[2]) @ np.array([-0.10, 0.0, 0.0]), None, timeout=3.0)
        S.tuck(walk=True)
        inst = c.pool.instance.get(px)
        ok = inst in c._tote_parts()
        if not ok:
            S.log("[ref] %s did not land in the tote (%s)" % (px, c._locations().get(inst)))
        return ok

    # ------------------------------------------------------------------------------------------------ hot swaps
    def cord_out(self, host, k, side):
        """A supply's power cord out of its inlet and let go beside the supply block (a stiff cord stays where it is
        let go). Returns the plug's proxy or None."""
        Rw = self.rear
        port = "%s/plug%d" % (host, k)
        px = Rw.unplug(port, side)
        if px is None:
            return None
        return Rw.set_aside(side, port)

    def cord_in(self, px, host, k, side):
        """The let-go plug taken where it floats and pushed into its supply's inlet."""
        Rw = self.rear
        port = "%s/plug%d" % (host, k)
        if not Rw.regrasp(px, port, side):
            return False
        return Rw.plug(px, port, side)

    def _spare_in_hand(self, spare_px, spare_side, what):
        if spare_px is None or self.S.holds(spare_side) != spare_px:
            self.S.log("[ref] %s: the spare is not in the %s hand (%s); the failed part stays in" % (
                what, spare_side, self.S.holds(spare_side)))
            return False
        return True

    def osfp_swap(self, osfp_port, spare_px, spare_side):
        """SOP-NET-01 step 3 with the spare transceiver in `spare_side`: the MPO out and set aside, the old transceiver
        out with the other hand, the spare in, the MPO back. Returns the old transceiver (held) or None."""
        S, c, Rw = self.S, self.c, self.rear
        work = other(spare_side)
        mpo_port = osfp_port.replace("osfp", "mpo")
        if not self._spare_in_hand(spare_px, spare_side, osfp_port):
            return None
        Rw.door_open(S.rack_of(osfp_port.split("/")[0]))
        mpo = Rw.unplug(mpo_port, work)
        if mpo is None:
            return None
        # just out of the cage's way (the old transceiver comes straight out), within the spare's hand's reach (it
        # takes the connector back)
        Rw.set_aside(work, mpo_port, toward=spare_side, up=0.04, far=0.04)
        old = Rw.unplug(osfp_port, work)
        if old is None:
            return None
        self.hold_in(work, quick=True)
        ok = Rw.plug(spare_px, osfp_port, spare_side)
        S.log("[ref] %s: spare seated %s" % (osfp_port, ok))
        if Rw.regrasp(mpo, mpo_port, spare_side):
            Rw.plug(mpo, mpo_port, spare_side)
        self.carry(work)
        return old

    def psu_swap(self, host, k, spare_px, spare_side):
        """SOP-PWR-01 step 3 with the spare already in `spare_side`: the cord out, the failed supply out with the
        other hand, the spare in (within 30 s of the bay going empty), the cord back. Returns the failed supply's
        proxy (held in the work hand) or None."""
        S, c, Rw = self.S, self.c, self.rear
        work = other(spare_side)
        port = "%s/psu%d" % (host, k)
        if not self._spare_in_hand(spare_px, spare_side, port):
            return None
        Rw.door_open(S.rack_of(host))
        cord = self.cord_out(host, k, work)
        if cord is None:
            return None
        t0 = c.sim_time
        failed = Rw.unplug(port, work)
        if failed is None:
            S.log("[ref] %s psu%d did not come out" % (host, k))
            return None
        self.hold_in(work, quick=True)
        ok = Rw.plug(spare_px, port, spare_side)
        S.log("[ref] %s psu%d: bay empty %.1f s, spare seated %s" % (host, k, c.sim_time - t0, ok))
        if not self.cord_in(cord, host, k, spare_side):
            S.log("[ref] %s plug%d: cord not back in" % (host, k))
        self.carry(work)
        return failed

    def fan_swap(self, host, k, spare_px, spare_side):
        """SOP-THM-01 with the spare already in `spare_side`: the failed module out with the other hand, the spare in
        within 30 s. Returns the failed module's proxy (held) or None."""
        S, c, Rw = self.S, self.c, self.rear
        work = other(spare_side)
        port = "%s/fan%d" % (host, k)
        if not self._spare_in_hand(spare_px, spare_side, port):
            return None
        t0 = c.sim_time
        failed = Rw.unplug(port, work)
        if failed is None:
            S.log("[ref] %s fan%d did not come out" % (host, k))
            return None
        self.hold_in(work, quick=True)
        ok = Rw.plug(spare_px, port, spare_side)
        S.log("[ref] %s fan%d: bay empty %.1f s, spare seated %s" % (host, k, c.sim_time - t0, ok))
        self.carry(work)
        return failed
