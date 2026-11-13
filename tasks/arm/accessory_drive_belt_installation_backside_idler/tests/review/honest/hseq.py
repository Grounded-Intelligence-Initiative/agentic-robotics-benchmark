"""The fitting sequence (drive B: C, ALT, I, WP, T; the back on the idler I and on T; the tensioner on the left opening
clockwise): drape -> tuck_I -> seat_top -> lift_span -> open_T -> mid_pre -> C -> seat_bottom -> mid_high -> return_T
-> check. The right arm handles the belt, the left arm the tensioner's lever (the arm on the tensioner's side). Where
to act comes from the eyes (heyes: the front camera's belt centreline, wrist-camera re-aims, the pointer, the wraps);
loads from the flange F/T; the tensioner's angle from the left hand holding its lever. Public inputs only."""
import math
import sys
import time
import traceback

import numpy as np

import hk
import hrig
from heyes import Eyes

OPT = {
    "DS_UP": 0.08, "DS_SET": 0.040, "DS_LINK": 0.0083, "DS_OPEN": 0.003, "DS_TURN": 90.0, "DS_HOOK_N": 1.2, "DS_RZ": 0.05, "DS_RZ_FLAT": -0.06, "DS_CARRY_D": 0.075,
    "DS_PULL_N": 0.0, "DS_R_SAFE": 0.11, "DS_ANG": 150.0, "DS_OUT_V": 0.02, "DS_LT": (-0.181, 0.325, 0.237),
    "HI_ENABLE": 0, "HU_ENABLE": 0, "HU_X": (0.085, 0.125), "HU_PT": (0.10, 0.20), "HI_GRAB": (0.11, 0.17), "HI_PT": (0.05, 0.215), "DS_TIP": 0.007, "DS_PUSH": 0.011, "SV_POSES": ((0.18, 0.23, 0.34), (0.18, 0.20, 0.34), (0.21, 0.23, 0.34), (0.18, 0.29, 0.30)),
    "TK_X": 0.108, "TK_D": 0.09, "TK_Z": 0.195, "TK_UP": 0.030, "TK_ROUNDS": 2, "TK_FMAX": 12.0, "TK_DMAX": 0.040,
    "C_LOW_MAX": 0.02, "C_TIP": 0.0095, "C_GAP": 0.006, "C_GRAB_WMAX": 0.026, "C_SLACK_D": 0.050, "C_FMAX": 15.0, "C_SETTLE": 0.0, "EYE_EDGE": 0,
    "C_YSIGN": 1, "RT_YSIGN": 1, "MID_AIM": 0, "REL_RESET": 0, "REL_OPEN": 0.008, "C_RETRY": 2, "C_RETRY_D": 0.050,
    "HB_ENABLE": 1, "HB_DL": 4.0, "HB_FREL": 0.6, "HB_RATE": 8.0,
    "SC_ANG": (-90.0, -120.0, -60.0, -150.0, -30.0), "PUSH_FMAX": 12.0,
    "LIFT_TGT": (-0.140, 0.035), "LIFT_XMIN": -0.195, "LIFT_D": 0.07,
    "MH_Z": 0.17, "MH_SKIP_INPLANE": 1, "MH_INPLANE_TOL": 0.006, "MH_DX": 0.006, "MH_FMAX": 27.0, "MH_FMAX_TAUT": 15.0, "MH_OPEN_W": 0.014, "MID_PRE": 1,
    "MP_FMAX": 10.0, "MP_RELAX_N": 3.0, "MP_RELAX_ALWAYS": 1,
    "CF_DRAPE_1": 1.5, "CF_DRAPE_2": 1.0, "CF_MID": 2.0, "CF_C": 1.0,
    "OPEN_ITER": 4, "OPEN_STEP_DEG": 2.0, "OPEN_RATE": 5.0, "OPEN_MAX_N": 25.0,
    "RT_DL": 6.0, "RT_DMIN": 2.0, "RT_FMAX": 28.0, "RT_MOM": 0.8, "RT_MIN_DROP": 0.8, "RT_FB_LEVER": 0.35, "RT_DFY": 5.0,
    "RT_RECOVER": 3, "MH_DEPTH_ITER": 2, "RT_LAST_OPEN": 40.0,
    "FIX_ROUNDS": 3, "FIX_OPEN_DEG": 40.0,
}


def opt(k):
    return OPT[k]


def log(*a):
    print(*a, file=sys.stderr, flush=True)


def tangent(c1, a1, c2, a2):
    """the belt line from circle 1 to circle 2 (signed offsets a: -r for a pulley inside the loop, +r outside)"""
    c1, c2 = np.asarray(c1, float), np.asarray(c2, float)
    D = c2 - c1
    L = float(np.hypot(*D))
    phi = math.atan2(D[1], D[0])
    al = phi - math.asin((a1 - a2) / L)
    n = np.array([-math.sin(al), math.cos(al)])
    return c1 + a1 * n, c2 + a2 * n


class Seq(object):
    def __init__(self, cell, given_dir):
        self.given = given_dir
        self.r = hrig.Rig(cell, given_dir)
        self.y = Eyes(self.r)
        self.y.edge_centre = bool(opt("EYE_EDGE"))
        self.r.lever_eye = self.y.lever_angle
        self.ctx = {"log": []}
        self.log = self.ctx["log"]
        dw = self.r.dw
        self.dw = dw
        self.B, self.T = self.r.BS, self.r.TS          # the belt hand (right), the lever hand (left)
        self.WP = dw.pul["WP"]["c"]
        self.ALT = dw.pul["ALT"]["c"]
        self.I = dw.pul["I"]["c"]
        self.C0 = dw.pul["C"]["c"]
        self.PLANE = dw.plane
        half = 0.5 * dw.belt_thick
        # the run from WP down to C (the left side, both ribs inside): where the span lies with T at its install stop
        self.lineWC = tangent(self.WP, -(dw.pul["WP"]["r"] + half), self.C0, -(dw.pul["C"]["r"] + half))
        self.audit = None

    # ------------------------------------------------------------------ eyes
    def x_on_line(self, z):
        p1, p2 = self.lineWC
        u = (z - p1[1]) / (p2[1] - p1[1])
        return p1[0] + u * (p2[0] - p1[0])

    def pts(self, tag=""):
        """the belt's centreline in E (10 mm apart, in order round the loop)"""
        Q, cov, n = self.y.front_loop()
        self.log.append(("eyes", tag, len(Q), round(cov, 2), n))
        if len(Q) < 10:
            raise RuntimeError("eyes: the belt is not seen (%s)" % tag)
        return Q

    def normal_at(self, Q, k):
        n = len(Q)
        t = Q[(k + 1) % n, :2] - Q[(k - 1) % n, :2]
        t /= max(1e-9, np.linalg.norm(t))
        return np.array([-t[1], t[0]])

    def aim_fn(self, side, cands, twist_max=0.0):
        """aim(i, phase) for Rig.grab: candidates are E points with in-plane normals; at the standoff ('near') the
        hand's wrist camera re-aims on the belt near the candidate"""
        r = self.r
        cam = "left_wrist" if side == "L" else "right_wrist"
        memo = {}

        def closing(nrm):
            y = r.R_E @ np.array([nrm[0], 0.0, nrm[1]])
            if twist_max > 0.0:
                best = None
                for a in np.linspace(-twist_max, twist_max, 11):
                    yr = hk.rot_axis(r.into, math.radians(a)) @ y
                    sc = abs(float(yr @ r.R_E[:, 0]))
                    if best is None or sc < best[0]:
                        best = (sc, yr)
                y = best[1]
            return y

        def aim(i, phase):
            pE, nrm = cands[i]
            if phase == "near":
                fr = self.y.frame(cam)
                loc = self.y.local(fr, pE, hand=side)
                if loc is not None and np.hypot(*(loc[0][:2] - pE[:2])) < 0.03 and abs(loc[0][2] - pE[2]) < 0.04:
                    d0 = cands[i][0][2]
                    pE, nrm = loc[0].copy(), loc[1]
                    if abs(pE[2] - d0) > 0.006:
                        # the wrist camera sees the band from the side and close: when its front edge is hidden (by
                        # the fingers), its depth reads deep; the survey's depth stands then
                        pE[2] = d0
                    if np.dot(nrm, cands[i][1]) < 0:
                        nrm = -nrm
                    self.log.append(("reaim", side, i, [round(float(v), 4) for v in pE], loc[2]))
                else:
                    self.log.append(("reaim_none", side, i, None if loc is None else [round(float(v), 4) for v in loc[0]]))
            memo[i] = (pE, nrm)
            return r.W(*pE), closing(nrm)

        return aim

    # ------------------------------------------------------------------ stages
    def st_look(self):
        """the engine's pose from the front camera (both arms at home: out of its view)"""
        r = self.r
        fr = self.y.frame("front")
        E = self.y.engine_pose_calibrated(fr, self.given)
        r.set_engine(*E)
        self.log.append(("engine", [round(float(v), 4) for v in E[0]], None if self.y.survey_bias is None else
                         [round(1000 * float(v), 2) for v in self.y.survey_bias]))

    def peg_survey(self):
        """the loop on its peg, seen by the belt hand's wrist camera from in front of the peg"""
        r = self.r
        dw = self.dw
        b = self.B
        pos = None
        # from below-left of the peg, the hand's approach tilted at the loop's middle (the loop hangs about 0.45 m;
        # the wrist camera then frames it whole at 0.32-0.46 m)
        tgt = r.W(dw.peg[0], dw.peg[1] - 0.20, 0.06)
        for x, z, dd in opt("SV_POSES"):
            p = r.W(x, z, dd)
            R = hk.hand_R((tgt - p) / np.linalg.norm(tgt - p), r.R_E[:, 2])
            for Rc in (R, r.flip(R)):
                q, ep, er, ok = r.kin.ik(b, p, Rc, q_ref=r.kin.home[b])
                if ok and r.kin.margin(b, q) > 0.15:
                    pos = (p, Rc, dd)
                    break
            if pos is not None:
                break
        if pos is None:
            raise RuntimeError("peg survey: no reachable view")
        r.goto_joint(b, pos[0], pos[1])
        r.wait(0.5)
        fr = self.y.frame("left_wrist" if b == "L" else "right_wrist")
        X = self.y.belt_E(fr, d_range=(-0.01, 0.13), keep=False, extra_keep=self.y._finger_keep(b))
        L, cov = self.y.loop(X)
        Q = self.y.resample(L)
        self.log.append(("peg_survey", round(pos[2], 3), len(X), len(Q), round(cov, 2)))
        if len(Q) < 20:
            raise RuntimeError("eyes: the loop on the peg is not seen")
        return Q

    def drape_grab(self, side, cands, push=False):
        r = self.r
        aim = self.aim_fn(side, cands)
        other = self.T if side == self.B else self.B
        r.grab(side, aim, tip_off=opt("DS_TIP"), pull_check=opt("DS_PULL_N") if side != self.B else 0.0,
               d_safe=opt("DS_R_SAFE") if r.grasp.get(other) is not None else 0.17)
        if push:
            # on the peg the loop slides along it: take it in a little (the drape was laid out with the belt there)
            g = r.goal[side][0].copy()
            r.move(side, g + r.into * opt("DS_PUSH"), speed=0.02, fmax=20.0)

    def st_drape(self):
        r, log = self.r, self.log
        b, t = self.B, self.T
        th = 0.5 * self.dw.belt_thick
        r.relief = {"L", "R"}
        Q = self.peg_survey()
        c = Q[:, :2].mean(axis=0)
        ang = np.degrees(np.arctan2(Q[:, 1] - c[1], Q[:, 0] - c[0]))
        a = int(np.argmin(np.abs((ang - opt("DS_ANG") + 180) % 360 - 180)))
        self.drape_grab(b, [(Q[a], self.normal_at(Q, a))], push=True)
        pa = r.hand_pos(b)
        r.moveE(b, pa[0], pa[1] + 0.02, pa[2], speed=0.05)
        r.moveE(b, pa[0], pa[1] + 0.02, 0.17, speed=0.08)
        r.confirm_load(b, opt("CF_DRAPE_2"), "drape_off_peg", log)
        alt_top = self.ALT[1] + self.dw.pul["ALT"]["r"] + th
        r.moveE(b, self.ALT[0], alt_top + opt("DS_UP"), 0.17, speed=0.10, settle=1.5)
        # the loop's top over ALT's groove, the belt's centre at the groove plane (the link sits DS_LINK beyond the
        # tcp), held DS_SET above its seat: the lower finger (15.5 mm behind its pad) then clears ALT's shoulder
        d_tcp = self.PLANE + opt("DS_LINK")
        r.moveE(b, self.ALT[0], alt_top + opt("DS_UP"), d_tcp, speed=0.04, settle=0.5)
        ok = r.moveE(b, self.ALT[0], alt_top + opt("DS_SET"), d_tcp, speed=0.02, settle=0.5, fmax=8.0)
        log.append(("on_ALT", ok, [round(float(v), 3) for v in r.E(r.tcp(b)[0])]))
        self.drape_release(b)
        r.park(b)
        self._audit("drape_ALT")
        # the run under I: the belt hand takes the loop's left side where it leaves ALT and holds it in the plane just
        # below I (the technician's thumb under the idler) while the lever hand takes the side on over WP; the run from
        # the held point to WP's top then passes below-left of I and, once let go, the loop's weight lifts it onto
        # I's face from below
        d_tcp = self.PLANE + opt("DS_LINK")
        P = self.pts("hung_ALT")
        n = len(P)
        k0 = int(np.argmin(np.hypot(P[:, 0] - self.ALT[0], P[:, 1] - alt_top) + 10.0 * np.abs(P[:, 2] - self.PLANE)))
        sgn = 1 if P[(k0 + 3) % n][0] < P[(k0 - 3) % n][0] else -1
        if opt("HI_ENABLE"):
            self.hold_under_I(P, k0, sgn, d_tcp)
        # the loop's left side up onto WP: the lever hand (it is free until the tensioner is opened)
        tx, tz_hi, tz_lo = opt("DS_LT")
        for attempt in range(2):
            P = self.pts("hung_ALT" if not opt("HI_ENABLE") else "held_under_I")
            n = len(P)
            if r.grasp.get(b) is not None:
                hE = r.E(r.tcp(b)[0])
                k1 = int(np.argmin(np.hypot(P[:, 0] - hE[0], P[:, 1] - hE[1])))
                # the strand away from ALT: the neighbour further from ALT's top
                sg = 1 if np.hypot(*(P[(k1 + 4) % n, :2] - self.ALT)) > np.hypot(*(P[(k1 - 4) % n, :2] - self.ALT)) else -1
                js = [(k1 + sg * j) % n for j in range(18, 36)]
            else:
                k1 = int(np.argmin(np.hypot(P[:, 0] - self.ALT[0], P[:, 1] - alt_top) + 10.0 * np.abs(P[:, 2] - self.PLANE)))
                sg = 1 if P[(k1 + 3) % n][0] < P[(k1 - 3) % n][0] else -1
                # on the loop's left side, 22-38 points (about 0.3 m) round from ALT's top, where the side is straight
                js = [(k1 + sg * j) % n for j in range(12, 50)]
            # not where the side lies on a pulley (C's groove, a rim): a free stretch in front of the face
            def rim_cl(k):
                return min(np.hypot(P[k][0] - pp["c"][0], P[k][1] - pp["c"][1]) - pp["r"] - 0.004 for pp in self.dw.pul.values())
            free = [k for k in js if rim_cl(k) > 0.030]
            js = free or js
            # (a stretch hanging in front of the face first; else one lying in the plane between pulleys, as far from
            # their rims as it goes: the fingers straddle it there)
            # (a side lying flat on the face is taken low, away from ALT: taken near ALT, the lift draws the loop over
            # ALT's top and off it)
            kb = min(js, key=lambda k: abs(P[k][1] - opt("DS_RZ")) if P[k][2] > self.PLANE + 0.012 else
                     0.15 + abs(P[k][1] - opt("DS_RZ_FLAT")) + max(0.0, 0.05 - rim_cl(k)) * 5.0)
            log.append(("T_pt", k1, kb, [round(float(v), 3) for v in P[kb]]))
            self.drape_grab(t, [(P[kb], self.normal_at(P, kb))])
            pb = r.hand_pos(t)
            # (out in front far enough for the held side to pass in front of WP's shoulder; not further, or the run
            # from ALT's top is drawn forward off ALT)
            m1 = r.moveE(t, pb[0], pb[1], opt("DS_CARRY_D"), speed=0.02)
            m2 = r.moveE(t, tx, tz_hi, opt("DS_CARRY_D"), speed=0.06, settle=0.5)
            log.append(("T_carry", m1, m2, round(r.load_f(t), 1), [round(float(v), 3) for v in r.E(r.tcp(t)[0])]))
            try:
                r.confirm_load(t, opt("CF_DRAPE_1"), "drape_over_WP", log)
                break
            except RuntimeError:
                if attempt:
                    raise
                log.append(("T_regrab",))
                r.set_width(t, 0.04, wait=0.3)
                r.grasp[t] = None
                r.park(t)
                r.wait(1.0)
        if opt("HU_ENABLE") and r.grasp.get(b) is None:
            self.hold_run_under_I()
        m3 = r.moveE(t, tx, tz_hi, d_tcp, speed=0.04, settle=0.5)
        ok = r.moveE(t, tx, tz_lo, d_tcp, speed=0.02, settle=0.5, fmax=10.0)
        log.append(("on_WP", m3, ok, round(r.load_f(t), 1), [round(float(v), 3) for v in r.E(r.tcp(t)[0])]))
        self._audit("on_WP")
        self.drape_release(t)
        r.park(t)
        r.wait(0.5)
        if r.grasp.get(b) is not None:
            self.release_run(b)
            r.park(b)
        r.wait(1.0)
        r.relief = set()
        self._audit("drape_WP")

    def hold_run_under_I(self):
        """with the lever hand holding the loop's side out in front, high left of WP: the belt hand takes the top run
        between ALT and I and holds it in the plane below I (the technician's thumb under the idler); the lever hand
        then lays the side over WP, the run from the held point to WP's top passing below I"""
        r, log = self.r, self.log
        b = self.B
        P = self.pts("run_ALT_I")
        x0, x1 = opt("HU_X")
        sel = np.nonzero((P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > self.I[1] - 0.06) & (P[:, 1] < self.I[1] + 0.05))[0]
        if not len(sel):
            log.append(("HU_none",))
            return
        k = int(sel[np.argmin(np.abs(P[sel, 0] - 0.5 * (x0 + x1)))])
        r.grab(b, self.aim_fn(b, [(P[k], self.normal_at(P, k))]), open_w=0.03)
        pk = r.hand_pos(b)
        hx, hz = opt("HU_PT")
        d_tcp = self.PLANE + opt("DS_LINK")
        m1 = r.moveE(b, hx, hz, max(float(pk[2]), d_tcp), speed=0.02, fmax=15.0)
        m2 = r.moveE(b, hx, hz, d_tcp, speed=0.015, fmax=15.0, settle=0.3)
        log.append(("HU_hold", [round(float(v), 3) for v in pk], m1, m2, round(r.load_f(b), 2)))
        self._audit("hold_run_under_I")

    def release_run(self, side):
        """let go of a run held in the plane: open well clear of it, then straight out of the plane"""
        r = self.r
        r.set_width(side, min(0.08, r.width(side) + opt("REL_OPEN")), wait=0.3)
        r.grasp[side] = None
        p = r.tcp(side)[0]
        r.move(side, p - r.into * 0.06, speed=0.02, fmax=40.0)
        r.wait(0.3)
        r.park(side)

    def hold_under_I(self, P, k0, sgn, d_tcp):
        """(option) the belt hand takes the loop's left side where it leaves ALT and holds it in the plane just below
        I while the lever hand takes the side on over WP"""
        r, log = self.r, self.log
        b = self.B
        n = len(P)
        hx, hz = opt("HI_GRAB")
        js = [(k0 + sgn * j) % n for j in range(3, 20)]
        kh = min(js, key=lambda k: math.hypot(P[k][0] - hx, P[k][1] - hz))
        log.append(("HI_pt", k0, kh, [round(float(v), 3) for v in P[kh]]))
        r.grab(b, self.aim_fn(b, [(P[kh], self.normal_at(P, kh))]), open_w=0.03)
        ph = r.hand_pos(b)
        ix, iz = opt("HI_PT")
        r.moveE(b, ph[0], ph[1], max(float(ph[2]), 0.06), speed=0.02, fmax=15.0)
        r.moveE(b, ix, iz, max(float(ph[2]), 0.06), speed=0.02, fmax=15.0)
        mh = r.moveE(b, ix, iz, d_tcp, speed=0.015, fmax=15.0, settle=0.3)
        log.append(("HI_hold", mh, round(r.load_f(b), 2), [round(float(v), 3) for v in r.E(r.tcp(b)[0])]))
        self._audit("hold_under_I")

    def drape_release(self, side):
        """open the fingers a little (wide open, the lower finger would swing into the pulley's rim), slide them
        straight out of the belt plane slowly, then back off"""
        r = self.r
        r.set_width(side, r.width(side) + opt("DS_OPEN"), wait=0.3)
        if opt("DS_TURN") and side == self.B:
            # tilt the lower finger about the approach axis: the belt resting on it slides off sideways onto the pulley
            g, Rg = r.goal[side]
            r.move(side, g, speed=0.02, fmax=40.0, R=hk.rot_axis(r.into, math.radians(opt("DS_TURN"))) @ Rg)
            r.wait(0.5)
        p = r.goal[side][0].copy()
        r.move(side, p - r.into * 0.04, speed=opt("DS_OUT_V"), fmax=40.0)
        r.wait(0.3)
        f = r.load_f(side)
        r.move(side, p - r.into * 0.10, speed=0.05, fmax=40.0)
        self.log.append(("drape_released", side, round(f, 2), round(r.load_f(side), 2)))

    # ------------------------------------------------------------------ the backside idler
    def top_run_at_I(self, P):
        """the belt points over I's span (|x - I.x| < 25 mm, between 120 mm below and 60 mm above I's centre):
        (n, mean z, mean depth)"""
        sel = (np.abs(P[:, 0] - self.I[0]) < 0.025) & (P[:, 1] > self.I[1] - 0.12) & (P[:, 1] < self.I[1] + 0.06)
        if not sel.any():
            return 0, None, None
        return int(sel.sum()), float(P[sel, 1].mean()), float(P[sel, 2].mean())

    def under_I(self, P):
        n, z, dd = self.top_run_at_I(P)
        r_I = self.dw.pul["I"]["r"]
        if n == 0:
            return False, (n, z, dd)
        # (below I's face and not in front of its front flange: a run a few mm forward is drawn in by the tension)
        ok = z < self.I[1] - r_I + 0.006 and dd < opt("TK_DMAX")
        return bool(ok), (n, round(z, 4), round(dd, 4))

    def st_tuck_I(self):
        """the top run onto I's face from below: the run that came to rest in front of I (on its front flange) is
        taken right of I, drawn down below I's flanges, into the plane, raised against I's face and let go there: the
        loop's weight then holds it up against I"""
        r, log = self.r, self.log
        b = self.B
        for rnd in range(int(opt("TK_ROUNDS")) + 1):
            P = self.pts("tuck_I")
            ok, info = self.under_I(P)
            log.append(("tuck_I_seen", rnd, ok, info))
            if ok or rnd == int(opt("TK_ROUNDS")):
                break
            # the grip point: on the top run between I's and ALT's rims
            x0 = opt("TK_X")
            sel = np.nonzero((np.abs(P[:, 0] - x0) < 0.02) & (P[:, 1] > self.I[1] - 0.08) & (P[:, 1] < self.I[1] + 0.05))[0]
            if not len(sel):
                raise RuntimeError("tuck_I: the top run right of I is not seen")
            k = int(sel[np.argmin(np.abs(P[sel, 0] - x0))])
            r.grab(b, self.aim_fn(b, [(P[k], self.normal_at(P, k))]), open_w=0.03)
            pk = r.hand_pos(b)
            log.append(("tuck_grab", [round(float(v), 4) for v in pk]))
            # well out in front: the run between the hand and WP's top then clears I's front flange along I's width
            d_out = max(float(pk[2]), opt("TK_D"))
            r.moveE(b, pk[0], pk[1], d_out, speed=0.01, fmax=opt("TK_FMAX"))
            r.moveE(b, pk[0], opt("TK_Z"), d_out, speed=0.01, fmax=opt("TK_FMAX"))
            d_tcp = self.PLANE + opt("DS_LINK")
            r.moveE(b, pk[0], opt("TK_Z"), d_tcp, speed=0.015, fmax=opt("TK_FMAX"), settle=0.3)
            z_up = self.I[1] - self.dw.pul["I"]["r"] - opt("TK_UP")
            m = r.moveE(b, pk[0], z_up, d_tcp, speed=0.01, fmax=opt("TK_FMAX"), settle=0.3)
            log.append(("tuck_set", m, round(r.load_f(b), 2)))
            self.drape_release(b)
            r.park(b)
            r.wait(0.5)
        self._audit("tuck_I")

    def push_seat(self, pulley, angles, side=None, fmax=None):
        r = self.r
        side = self.B if side is None else side
        cc, rr = self.dw.pul[pulley]["c"], self.dw.pul[pulley]["r"]
        for ang in angles:
            u = np.array([math.cos(math.radians(ang)), math.sin(math.radians(ang))])
            x, z = cc + u * (rr + 0.0115)
            r.push(side, x, z, 0.075, 0.0375, radial_xz=u, fmax=opt("PUSH_FMAX") if fmax is None else fmax)

    def st_seat_top(self):
        self.push_seat("ALT", (40, 70, 100), side=self.B)
        self.push_seat("WP", (100, 140), side=self.T)
        self.r.park(self.T)
        self._audit("seat_top")

    def st_lift_span(self):
        r, log = self.r, self.log
        b = self.B
        r.park(self.T)
        P = self.pts("lift_span")
        # (the left span hangs from WP's left side past T: anything further left than T's sweep is not the belt)
        # (not further left than LIFT_XMIN: the lever hand comes down to the lever's closed position there)
        x_min = max(self.dw.T_centre(self.dw.th_open)[0] - self.dw.T_r - 0.08, opt("LIFT_XMIN"))
        cand = np.nonzero((P[:, 0] < -0.06) & (P[:, 0] > x_min) & (P[:, 1] > 0.0) & (P[:, 1] < 0.07))[0]
        if not len(cand):
            cand = np.nonzero((P[:, 0] < -0.06) & (P[:, 0] > x_min) & (P[:, 1] > -0.03) & (P[:, 1] < 0.11))[0]
        if not len(cand):
            raise RuntimeError("lift_span: no belt seen below T")
        tgt = np.asarray(opt("LIFT_TGT"))
        k = int(cand[np.argmin(np.hypot(P[cand, 0] - tgt[0], P[cand, 1] - tgt[1]))])
        r.grab_twist_max = 25.0
        r.grab(b, self.aim_fn(b, [(P[k], self.normal_at(P, k))], twist_max=25.0))
        r.grab_twist_max = 0.0
        pk = r.hand_pos(b)
        log.append(("lift_pt", [round(float(v), 4) for v in pk]))
        r.moveE(b, pk[0], pk[1], opt("LIFT_D"), speed=0.02, settle=0.2)

    def open_to_install(self, tag):
        r = self.r
        dw = self.dw
        tgt = dw.opener(dw.th_open, -2.0)
        rate = 10.0
        for _ in range(4):
            r.turn_lever(tgt, rate_dps=rate)
            if not r.lever_over:
                break
            r.lever_over = False
            self.log.append((tag + "_slow", round(math.degrees(r.theta_est()), 1), rate))
            r.wait(0.5)
            rate = max(2.0, 0.5 * rate)
        r.wait(0.5)
        n = 0
        for _ in range(int(opt("OPEN_ITER"))):
            th = r.theta_est()
            if not dw.more_open(dw.opener(tgt, -1.0), th) or r.pin_force() > opt("OPEN_MAX_N") or r.lever_over:
                break
            r.turn_lever(dw.opener(r.th_cmd, min(opt("OPEN_STEP_DEG"), abs(math.degrees(tgt - th)))), rate_dps=opt("OPEN_RATE"))
            r.wait(0.3)
            n += 1
        self.log.append((tag, round(r.pin_force(), 1), round(math.degrees(r.theta_est()), 1), round(math.degrees(r.th_cmd), 1), n))

    def st_open_T(self):
        r = self.r
        b = self.B
        r.pin_lever()
        self.open_to_install("T_open")
        self._audit("T_open")
        g = r.hand_pos(b)
        xl = self.x_on_line(g[1])
        o1 = r.moveE(b, xl, g[1], 0.058, speed=0.02)
        r.release(b)
        r.moveE(b, xl, g[1], 0.12, speed=0.04, settle=0.5)
        self.log.append(("span_on_line", o1))
        self._audit("span_on_line")

    def relax_hold(self, side, f_to=3.0, step=0.002, max_d=0.06):
        r = self.r
        moved = 0.0
        while moved < max_d - 1e-9 and r.load(side) > f_to:
            g = r.goal[side][0].copy()
            dv = r.tcp(side)[0] - g
            n = float(np.linalg.norm(dv))
            if n < 2e-4:
                break
            r.move(side, g + dv / n * step, speed=0.01, settle=0.1, fmax=30.0)
            moved += step
        return round(moved, 3), round(r.load(side), 1)

    def release_hold(self):
        r = self.r
        b = self.B
        if opt("REL_RESET"):
            r.relief = set()
            p0, R0 = r.tcp(b)
            r.command_cart(b, p0, R0)
            r.wait(0.3)
        # (a span held in free space: open well clear of it, or a pad drags it out with the fingers)
        r.set_width(b, r.width(b) + opt("REL_OPEN"), wait=0.2)
        p = r.tcp(b)[0]
        r.move(b, p - r.into * 0.05, speed=0.03, fmax=40.0)
        gB = r.hand_pos(b)
        r.moveE(b, gB[0], gB[1], 0.13, speed=0.05, fmax=40.0)

    def st_mid_high(self, pre=False):
        r, log = self.r, self.log
        b = self.B
        if self.ctx.get("hb_done") and not pre:
            log.append(("mid_high", "skipped: hold-bottom return done"))
            return
        zq = opt("MH_Z")
        # inside the loop is to the right of the WP-C line
        xq = self.x_on_line(zq) + opt("MH_DX")
        P = self.pts("mid_pre" if pre else "mid_high")
        left = np.nonzero((P[:, 0] < -0.03) & (np.abs(P[:, 1] - zq) < 0.02))[0]
        if not len(left):
            left = np.nonzero((P[:, 0] < -0.03) & (np.abs(P[:, 1] - zq) < 0.05))[0]
        if not len(left):
            raise RuntimeError("mid_high: no belt on the left span near z %.2f" % zq)
        k = int(left[np.argmin(np.abs(P[left, 1] - zq))])
        n = len(P)
        if not pre and opt("MH_SKIP_INPLANE"):
            near = left[np.abs(P[left, 1] - zq) < 0.03]
            dd = float(np.median(P[near, 2])) if len(near) else 1.0
            if abs(dd - self.PLANE) < opt("MH_INPLANE_TOL"):
                # the span already runs in the plane past T's front flange (the C step drew it in): T is let back
                # onto it without a hold (a hand there would stand between T's face and the span)
                log.append(("mid_high", "span in plane: no hold", round(dd, 4)))
                return
        s_up = 1 if P[(k + 1) % n, 1] > P[(k - 1) % n, 1] else -1
        # (alternatives further down the span: away from the lever hand above)
        cands = [(P[(k - j * s_up) % n], self.normal_at(P, (k - j * s_up) % n)) for j in (0, 1, 2)]
        r.grab(b, self.aim_fn(b, cands), open_w=opt("MH_OPEN_W"), alts=2)
        self.ctx["B_hold_t"] = r.time
        r.relief_total = {}
        r.relief = {b: "yz"}
        aims = [g["aim_E"] for g in r.grasp_log if g.get("ok") and "aim_E" in g]
        pk = np.asarray(aims[-1], float) if (aims and opt("MID_AIM")) else r.E(r.tcp(b)[0])
        log.append(("mid_grab_pt", [round(float(v), 4) for v in pk], [round(float(v), 4) for v in r.E(r.tcp(b)[0])]))
        fm = opt("MP_FMAX") if pre else opt("MH_FMAX")
        if abs(pk[2] - self.PLANE) < 0.008:
            m1, m3 = "in_plane", "in_plane"
            m2 = r.moveE(b, xq, zq, self.PLANE, speed=0.01, fmax=min(fm, opt("MH_FMAX_TAUT")))
            r.wait(0.3)
        elif pre:
            d0 = float(min(max(pk[2], self.PLANE), 0.06))
            m1 = "no_out"
            m2 = r.moveE(b, xq, zq, d0, speed=0.015, fmax=fm)
            m3 = r.moveE(b, xq, zq, self.PLANE, speed=0.01, settle=0.3, fmax=fm) if m2 else "skip"
        else:
            m1 = r.moveE(b, pk[0], pk[1], max(pk[2], 0.06), speed=0.02, fmax=fm)
            m2 = r.moveE(b, xq, zq, max(pk[2], 0.06), speed=0.02, fmax=fm)
            m3 = r.moveE(b, xq, zq, self.PLANE, speed=0.01, settle=0.3, fmax=fm)
        if pre and (m2 is False or m3 is not True or r.load(b) > fm):
            ld = round(r.load(b), 1)
            back = self.relax_hold(b, f_to=opt("MP_RELAX_N"), max_d=0.06)
            log.append(("mid_pre_undo", m2, m3, ld, back))
            r.relief = set()
            self.release_hold()
            r.wait(0.3)
            return
        r.confirm_load(b, opt("CF_MID"), "mid_pre_set" if pre else "mid_high_set", log)
        for _ in range(int(opt("MH_DEPTH_ITER"))):
            d_act = float(r.E(r.tcp(b)[0])[2])
            if d_act - self.PLANE < 0.0008 or r.load(b) > fm:
                break
            g = r.hand_pos(b)
            r.moveE(b, g[0], g[1], g[2] - min(0.004, d_act - self.PLANE), speed=0.005, settle=0.2, fmax=fm)
        log.append(("mid_high", m1, m2, m3, round(r.load(b), 1), round(float(r.E(r.tcp(b)[0])[2]), 4)))
        if not pre:
            r.hold_B = True
            self._audit("mid_high")
            return
        if opt("MP_RELAX_ALWAYS"):
            log.append(("mid_pre_relax", self.relax_hold(b, f_to=opt("MP_RELAX_N"), max_d=0.06)))
        r.relief = set()
        self.release_hold()
        r.wait(0.3)
        self._audit("mid_pre")

    def st_mid_pre(self):
        if opt("MID_PRE"):
            self.st_mid_high(pre=True)
            self.ctx.pop("B_hold_t", None)

    def bottom_depth_at_C(self):
        """(n, mean depth) of the belt seen within C's rim + 12 mm (front camera; arms parked)"""
        self.r.wait(opt("C_SETTLE"))
        P = self.pts("C_depth")
        rr = self.dw.pul["C"]["r"]
        sel = np.hypot(P[:, 0] - self.C0[0], P[:, 1] - self.C0[1]) < rr + 0.012
        return int(sel.sum()), (round(float(P[sel, 2].mean()), 4) if sel.any() else None)

    def c_grab_carry(self):
        r = self.r
        dw = self.dw
        b = self.B
        tip = opt("C_TIP")
        rc = dw.pul["C"]["r"] + dw.shoulder_h + 0.5 * dw.belt_thick + opt("C_GAP")
        d_tcp = dw.front_sh[1] + dw.chamfer + 0.001 + 0.0089
        if r.grasp.get(b) is None:
            r.park(b)
        P = self.pts("C_lowest")
        k = int(np.argmin(P[:, 1]))
        # (drive B: the loop over ALT, I and WP hangs only to about C's lower half, in front of C)
        if P[k, 1] > self.C0[1] + opt("C_LOW_MAX"):
            raise RuntimeError("eyes: the loop's bottom is not seen low enough at C (lowest point z %.3f)" % P[k, 1])
        sgn_ = opt("C_YSIGN") * (-1 if self.ctx.get("c_takes", 0) % 2 else 1)
        self.ctx["c_takes"] = self.ctx.get("c_takes", 0) + 1
        yp = None if not opt("C_YSIGN") else sgn_ * r.R_E[:, 2]
        r.grab(b, self.aim_fn(b, [(P[k], self.normal_at(P, k))]), tip_off=tip, w_max=opt("C_GRAB_WMAX"), y_pref=yp)
        pk = r.hand_pos(b)
        d_front = max(float(pk[2]), 0.065)
        tgt = self.C0 + np.array([0.0, -1.0]) * rc
        r.moveE(b, pk[0], pk[1], d_front, speed=0.03, fmax=opt("C_FMAX"))
        ok1 = r.moveE(b, tgt[0], tgt[1], d_front, speed=0.02, fmax=opt("C_FMAX"))
        r.confirm_load(b, opt("CF_C"), "C_under_rim", self.log)
        ok2 = r.moveE(b, tgt[0], tgt[1], d_tcp, speed=0.015, settle=0.3)
        return ok1, ok2

    def c_release(self):
        r = self.r
        b = self.B
        r.set_width(b, 0.020, wait=0.3)
        p = r.tcp(b)[0]
        r.move(b, p - r.into * 0.06, speed=0.04, fmax=40.0)
        r.wait(0.3)

    def st_C(self):
        r = self.r
        b = self.B
        ok1, ok2 = self.c_grab_carry()
        self.c_release()
        r.wait(0.2)
        self.log.append(("C_edge", ok1, ok2))
        r.park(b)
        n, dC = self.bottom_depth_at_C()
        self.log.append(("after_C", n, dC))
        for i in range(int(opt("C_RETRY"))):
            if dC is not None and dC <= opt("C_RETRY_D"):
                break
            ok1, ok2 = self.c_grab_carry()
            self.c_release()
            r.wait(0.2)
            r.park(b)
            n, dC = self.bottom_depth_at_C()
            self.log.append(("C_retry", i + 1, ok1, ok2, n, dC))
        self._audit("C")
        if opt("HB_ENABLE") and (dC is None or dC > opt("C_SLACK_D")):
            self.hold_bottom_return()

    def hold_bottom_return(self):
        r, log = self.r, self.log
        dw = self.dw
        b, t = self.B, self.T
        fails = self.ctx.setdefault("prog_fail", [])
        self.c_grab_carry()
        l0 = r.load_f(b)
        cur = r.theta_est()
        r.th_cmd = cur
        f0 = r.load_f(t)
        f_min = 1e9
        rate = opt("HB_RATE")
        released = False
        stop = why = None
        while dw.more_open(cur, dw.opener(dw.th_closed, 2.0)):
            cur = r.turn_lever(dw.opener(cur, -rate * hrig.TICK * 5), rate_dps=rate)
            f = r.load_f(t)
            lB = r.load_f(b)
            if r.lever_over:
                r.lever_over = False
                if not released:
                    why = "lever_over"
                    fails.append("HB_release_lever_over")
                    self.c_release()
                    released = True
                    continue
                n_rec = self.ctx.get("lever_recover", 0) + 1
                self.ctx["lever_recover"] = n_rec
                if n_rec > int(opt("RT_RECOVER")):
                    stop = "lever_over_26N"
                    break
                r.wait(1.0)
                cur = r.turn_lever(dw.opener(cur, 2.0), rate_dps=2.0)
                rate = max(2.0, 0.5 * rate)
                continue
            if not released:
                if lB > l0 + opt("HB_DL") or f < opt("HB_FREL") * f0:
                    why = "B_load" if lB > l0 + opt("HB_DL") else "lever_unload"
                    self.c_release()
                    released = True
                continue
            f_min = min(f_min, f)
            if f < 1.5:
                caught = True
                for _ in range(10):
                    r.wait(0.1)
                    if r.load_f(t) > 3.0:
                        caught = False
                        break
                if caught:
                    stop = "unloaded %.2f N" % f
                    break
            elif f_min < 5.0 and f > f_min + 2.0:
                stop = "load_rising"
                break
        if stop is None:
            stop = "closed_stop"
        if not released:
            fails.append("HB_release_after_lever_stop")
            self.c_release()
        log.append(("hb_return", stop, why, round(math.degrees(r.theta_est()), 1)))
        self.lever_off()
        self.ctx["hb_done"] = True
        self._audit("hb_return")

    def lever_off(self):
        r = self.r
        t = self.T
        if r.grasp.get(self.B) is None:
            r.park(self.B)              # (the belt hand first out of the lever hand's way)
        r.unpin()
        g = r.hand_pos(t)
        r.moveE(t, g[0] - 0.05, g[1], 0.15, speed=0.05)
        r.wait(1.0)

    def st_seat_bottom(self):
        if self.ctx.get("hb_done"):
            self.log.append(("seat_bottom", "skipped: hold-bottom return done"))
            return
        for ang in opt("SC_ANG"):
            self.push_seat("C", (ang,), side=self.B)
        self._audit("seat_bottom")

    def b_holding(self):
        r = self.r
        return r.grasp.get(self.B) is not None and r.width(self.B) >= 0.002

    def guided_return(self, B_holding, tag, rate_dps=10.0):
        r, log = self.r, self.log
        dw = self.dw
        b, t = self.B, self.T
        cur = r.theta_est()
        r.th_cmd = cur
        f0 = r.load_f(t)
        released = not B_holding
        why_B = None if B_holding else "not_holding"
        l0 = r.load_f(b) if B_holding else 0.0
        # the force across the held span, positive towards the loop's inside (the drive's right, +x of E: T presses
        # the span in from the left): the force the arm exerts on the hand to hold the span against T's face is then
        # along -x of E; the closing axis may point either way across the span, so its sign comes from the pose
        ysgn = 1.0 if float(r.tcp(b)[1][:, 1] @ (-r.R_E[:, 0])) >= 0 else -1.0
        if not opt("RT_YSIGN"):
            ysgn = 1.0
        fy0 = ysgn * float(r.ft_f(b)[1]) if B_holding else 0.0
        fl_min = 1e9
        f_min = 1e9
        stop = None
        th_last = dw.opener(dw.th_closed, opt("RT_LAST_OPEN"))
        fails = self.ctx.setdefault("prog_fail", [])
        trace = []
        if not B_holding and r.hand_pos(b)[2] < 0.12:
            r.set_width(b, 0.03, wait=0.2)
            gB = r.hand_pos(b)
            r.moveE(b, gB[0], gB[1], 0.13, speed=0.05)
        while dw.more_open(cur, dw.opener(dw.th_closed, 2.0)):
            cur = r.turn_lever(dw.opener(cur, -rate_dps * hrig.TICK * 5), rate_dps=rate_dps)
            f = r.load_f(t)
            if r.lever_over:
                r.lever_over = False
                if not released:
                    fails.append("B_release_lever_over")
                    why_B = "lever_over"
                    r.relief = set()
                    self.release_hold()
                    released = True
                    r.wait(0.5)
                    continue
                n_rec = self.ctx.get("lever_recover", 0) + 1
                self.ctx["lever_recover"] = n_rec
                log.append(("lever_over_recover_" + tag, n_rec, round(math.degrees(cur), 1), round(f, 1)))
                if n_rec > int(opt("RT_RECOVER")):
                    stop = "lever_over_26N"
                    break
                r.wait(1.0)
                cur = r.turn_lever(dw.opener(cur, 2.0), rate_dps=2.0)
                rate_dps = max(2.0, 0.5 * rate_dps)
                continue
            if not released:
                lB, mB, wB = r.load_f(b), r.grip_moment(b), r.width(b)
                trace.append((round(math.degrees(cur), 1), round(f, 2), round(lB, 2), round(ysgn * float(r.ft_f(b)[1]) - fy0, 2), round(mB, 2)))
                fl_min = min(fl_min, lB)
                why = None
                if wB < 0.001:
                    why = "FAIL_grip_slipped"
                elif mB > opt("RT_MOM"):
                    why = "moment"
                elif lB > l0 + opt("RT_DL"):
                    why = "load_up"
                elif ysgn * float(r.ft_f(b)[1]) - fy0 > opt("RT_DFY"):
                    why = "lateral_push"
                elif fl_min < opt("RT_MIN_DROP") * l0 and lB > fl_min + opt("RT_DMIN"):
                    why = "load_above_min"
                elif lB > opt("RT_FMAX"):
                    why = "load_max"
                elif f < opt("RT_FB_LEVER") * f0:
                    why = "FAIL_fallback_lever_load"
                elif dw.more_open(th_last, cur):
                    why = "FAIL_fallback_angle"
                if why is not None:
                    if why.startswith("FAIL"):
                        fails.append("B_release_" + why)
                    why_B = "%s at %.1f deg (load %.1f N, start %.1f, lever %.1f N)" % (why, math.degrees(cur), lB, l0, f)
                    r.relief = set()
                    self.release_hold()
                    released = True
                    continue
            f_min = min(f_min, f)
            if f < 1.5:
                caught = True
                for _ in range(10):
                    r.wait(0.1)
                    if r.load_f(t) > 3.0:
                        caught = False
                        break
                if caught:
                    stop = "unloaded %.2f N" % f
                    break
            elif f_min < 5.0 and f > f_min + 2.0:
                stop = "load_rising %.2f N (min %.2f)" % (f, f_min)
                break
        if stop is None:
            stop = "closed_stop"
        r.relief = set()
        if not released:
            why_B = "after_lever_stop"
            fails.append("B_release_after_lever_stop")
            self.release_hold()
        log.append(("return_" + tag, stop, why_B, round(math.degrees(r.theta_est()), 1), trace[::3]))
        self.lever_off()
        return stop, why_B

    def st_return_T(self):
        if self.ctx.get("hb_done"):
            self.log.append(("return_T", "skipped: hold-bottom return done"))
            return
        r = self.r
        holding = self.b_holding()
        if holding:
            r.wait(0.5)
            holding = self.b_holding()
        if not holding:
            self.log.append(("B_not_holding_at_return",))
        self.guided_return(B_holding=holding, tag="T")
        self._audit("return_T")

    FIX_ANGLES = {"C": (-150, -120, -90, -60, -30, 0), "ALT": (-10, 20, 50, 80, 110), "WP": (80, 110, 140, 170, 200, 230)}

    def check_now(self, tag):
        r = self.r
        dw = self.dw
        for s in ("L", "R"):
            r.park(s)
        th_hand = r.theta_est()
        out = self.y.check(th_prior=th_hand)
        po = out["pointer"]
        win = [math.degrees(v) for v in dw.window]
        if po.get("theta_deg") is None:
            # the blade not made out in the front frame: the angle at which the lever hand left the lever stands
            po.update({"theta_deg": round(math.degrees(th_hand), 2), "from": "hand",
                       "ok": bool(win[0] <= math.degrees(th_hand) <= win[1])})
        bad = [k for k in ("C", "ALT", "WP", "T") if not out[k]["ok"]]
        # (off: not seen on the pulley at all; a belt seen riding its shoulder is lifted, and a push can seat it)
        off = [k for k in ("C", "ALT", "WP") if (out[k].get("cover") or 0.0) < 0.3 and not out[k].get("lifted")]
        if not out["I"]["ok"]:
            off.append("I")
        closed_edge = math.radians(win[1] if dw.sense < 0 else win[0])
        if po.get("theta_deg") is not None and dw.more_open(dw.opener(closed_edge, -2.0), math.radians(po["theta_deg"])):
            off.append("pointer_closed")
        if not po.get("ok") and not bad and not off:
            bad = ["C", "ALT"]
        if po.get("theta_deg") is not None:
            r.th_seen = math.radians(po["theta_deg"])
        self.log.append((tag, {k: out[k] for k in out}, bad, off))
        return bad, off

    def st_check(self):
        r = self.r
        dw = self.dw
        fixed = False
        for attempt in range(int(opt("FIX_ROUNDS"))):
            bad, off = self.check_now("check%d" % attempt)
            fixed = False
            if not bad and not off:
                break
            if off:
                self.log.append(("belt_off", off))
                break
            r.stage = "fix%d" % attempt
            r.pin_lever()
            th_now = r.theta_est()
            r.th_cmd = th_now
            tgt = dw.opener(th_now, opt("FIX_OPEN_DEG"))
            lim = dw.opener(dw.th_open, -2.0)
            r.turn_lever(lim if dw.more_open(tgt, lim) else tgt, rate_dps=10.0)
            for k in [b_ for b_ in bad if b_ != "T"]:
                self.push_seat(k, self.FIX_ANGLES[k], side=self.B)
            self.guided_return(B_holding=False, tag="fix%d" % attempt)
            fixed = True
        if fixed:
            self.check_now("check_end")
        for s in ("L", "R"):
            r.park(s)
        self._audit("check")

    ORDER = ["look", "drape", "tuck_I", "seat_top", "lift_span", "open_T", "mid_pre", "C", "seat_bottom", "mid_high", "return_T", "check"]

    def _audit(self, name):
        if self.audit is not None:
            self.audit(name)

    def run(self):
        stages = {"look": self.st_look, "drape": self.st_drape, "tuck_I": self.st_tuck_I, "seat_top": self.st_seat_top,
                  "lift_span": self.st_lift_span, "open_T": self.st_open_T, "mid_pre": self.st_mid_pre, "C": self.st_C,
                  "seat_bottom": self.st_seat_bottom, "mid_high": self.st_mid_high, "return_T": self.st_return_T, "check": self.st_check}
        t0 = time.time()
        err = None
        for name in self.ORDER:
            self.r.stage = name
            try:
                stages[name]()
            except hrig.Stopped as e:
                err = "stopped: %s" % e
                break
            except Exception as e:  # noqa: BLE001
                err = "%s: %s" % (type(e).__name__, e)
                log(traceback.format_exc())
                self.safe_end()
                break
            log("stage %s done sim %.1f wall %.1f frames %d mincl %.1f" % (name, self.r.time, time.time() - t0, self.y.frames,
                                                                            1000 * self.r.min_clearance))
        self.log.append(("end", err, self.r.stage, round(self.r.time, 1)))
        return err

    def safe_end(self):
        """after a failure: never leave the lever loaded (let the tensioner back guided if the lever hand holds it);
        if that fails too (the belt hand in the way), turn the lever back alone, the planning margin set aside"""
        r = self.r
        try:
            if r.grasp.get(self.T) == ("lever",):
                self.guided_return(B_holding=self.b_holding(), tag="safe")
        except Exception:  # noqa: BLE001
            log(traceback.format_exc())
            try:
                if r.grasp.get(self.T) == ("lever",):
                    r.relief = set()
                    r.CLEAR_MARGIN = 0.0
                    dw = self.dw
                    cur = r.theta_est()
                    r.th_cmd = cur
                    while dw.more_open(cur, dw.opener(dw.th_closed, 2.0)):
                        cur = r.turn_lever(dw.opener(cur, -5.0 * hrig.TICK * 5), rate_dps=5.0)
                        if r.load_f(self.T) < 1.5:
                            break
                    self.log.append(("safe_lever_back", round(math.degrees(r.theta_est()), 1), round(r.load_f(self.T), 1)))
                    r.unpin()
            except Exception:  # noqa: BLE001
                log(traceback.format_exc())
