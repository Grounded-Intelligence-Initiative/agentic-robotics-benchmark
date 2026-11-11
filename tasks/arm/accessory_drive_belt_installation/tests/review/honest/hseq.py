"""The fitting sequence (variant A): drape -> seat_top -> lift_span -> open_T -> mid_pre -> C -> seat_bottom -> mid_high
-> return_T -> check. Where to act comes from the eyes (heyes: the front camera's belt centreline, wrist-camera re-aims,
the pointer, the wraps); loads from the flange F/T; the tensioner's angle from the right hand holding its lever.
Public inputs only."""
import math
import sys
import time
import traceback

import numpy as np

import hk
import hrig
from heyes import Eyes

OPT = {
    "DS_UP": 0.08, "DS_SET": 0.040, "DS_LINK": 0.0083, "DS_OPEN": 0.003, "DS_TURN": 90.0, "DS_PITCH": 0, "DS_PITCHES": (25.0, 40.0, 15.0), "DS_PITCH_WAIT": 0.8, "DS_HOOK_N": 1.2, "DS_NB": 30, "DS_RZ": -0.03, "DS_PULL_N": 0.0, "DS_HOLD": 0, "DS_R_SAFE": 0.11, "DS_ANG": 150.0, "DS_OUT_V": 0.02, "DS_RT": (0.165, 0.34, 0.22),
    "DS_TIP": 0.007, "DS_PUSH": 0.011,
    "C_TIP": 0.0095, "C_GAP": 0.006, "C_GRAB_WMAX": 0.026, "C_SLACK_D": 0.050, "C_FMAX": 15.0, "C_SETTLE": 0.0, "EYE_EDGE": 0, "C_YSIGN": 1, "RT_YSIGN": 1, "MID_AIM": 0, "REL_RESET": 0, "C_RETRY": 2, "C_RETRY_D": 0.050,
    "HB_ENABLE": 1, "HB_DL": 4.0, "HB_FREL": 0.6, "HB_RATE": 8.0,
    "SC_ANG": (-90.0, -60.0, -120.0, -30.0, -150.0), "PUSH_FMAX": 12.0,
    "MH_Z": 0.16, "MH_DX": 0.006, "MH_FMAX": 27.0, "MH_FMAX_TAUT": 15.0, "MH_OPEN_W": 0.014, "MID_PRE": 1,
    "MP_FMAX": 10.0, "MP_RELAX_N": 3.0, "MP_RELAX_ALWAYS": 1,
    "CF_DRAPE_L": 1.0, "CF_DRAPE_R": 1.5, "CF_MID": 2.0, "CF_C": 1.0,
    "OPEN_ITER": 4, "OPEN_STEP_DEG": 2.0, "OPEN_RATE": 5.0, "OPEN_MAX_N": 25.0,
    "RT_DL": 6.0, "RT_DMIN": 2.0, "RT_FMAX": 28.0, "RT_MOM": 0.8, "RT_MIN_DROP": 0.8, "RT_FB_LEVER": 0.35, "RT_DFY": 5.0,
    "RT_RECOVER": 3, "MH_DEPTH_ITER": 2, "RT_LAST": -112.0,
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
        self.WP = dw.pul["WP"]["c"]
        self.ALT = dw.pul["ALT"]["c"]
        self.C0 = dw.pul["C"]["c"]
        self.PLANE = dw.plane
        half = 0.5 * dw.belt_thick
        self.lineCA = tangent(self.C0, -(dw.pul["C"]["r"] + half), self.ALT, -(dw.pul["ALT"]["r"] + half))

    # ------------------------------------------------------------------ eyes
    def x_on_line(self, z):
        p1, p2 = self.lineCA
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
        """the loop on its peg, seen by the left wrist camera from 0.38 m in front of the peg"""
        r = self.r
        dw = self.dw
        R = hk.hand_R(r.into, r.R_E[:, 2])
        r.goto_joint("L", r.W(dw.peg[0] + 0.02, dw.peg[1] - 0.17, 0.38), R)
        r.wait(0.5)
        fr = self.y.frame("left_wrist")
        X = self.y.belt_E(fr, d_range=(-0.01, 0.13), keep=False, extra_keep=self.y._finger_keep("L"))
        L, cov = self.y.loop(X)
        Q = self.y.resample(L)
        self.log.append(("peg_survey", len(X), len(Q), round(cov, 2)))
        if len(Q) < 20:
            raise RuntimeError("eyes: the loop on the peg is not seen")
        return Q

    def drape_grab(self, side, cands):
        r = self.r
        aim = self.aim_fn(side, cands)
        # (on the peg the loop slides along it: a pull out of the plane says nothing there)
        r.grab(side, aim, tip_off=opt("DS_TIP"), pull_check=opt("DS_PULL_N") if side == "R" else 0.0,
               d_safe=opt("DS_R_SAFE") if (side == "R" and r.grasp.get("L") is not None) else 0.17)
        if side == "L":
            # on the peg the loop slides along it: take it in a little (the drape was laid out with the belt there)
            g = r.goal[side][0].copy()
            r.move(side, g + r.into * opt("DS_PUSH"), speed=0.02, fmax=20.0)

    def st_drape(self):
        r, log = self.r, self.log
        th = 0.5 * self.dw.belt_thick
        r.relief = {"L", "R"}
        Q = self.peg_survey()
        c = Q[:, :2].mean(axis=0)
        ang = np.degrees(np.arctan2(Q[:, 1] - c[1], Q[:, 0] - c[0]))
        a = int(np.argmin(np.abs((ang - opt("DS_ANG") + 180) % 360 - 180)))
        self.drape_grab("L", [(Q[a], self.normal_at(Q, a))])
        pa = r.hand_pos("L")
        r.moveE("L", pa[0], pa[1] + 0.02, pa[2], speed=0.05)
        r.moveE("L", pa[0], pa[1] + 0.02, 0.17, speed=0.08)
        r.confirm_load("L", opt("CF_DRAPE_L"), "drape_L_off_peg", log)
        wp_top = self.WP[1] + self.dw.pul["WP"]["r"] + th
        r.moveE("L", self.WP[0], wp_top + opt("DS_UP"), 0.17, speed=0.10, settle=1.5)
        # the loop's top over WP's groove, the belt's centre at the groove plane (the link sits DS_LINK beyond the
        # tcp), held DS_SET above its seat: the lower finger (15.5 mm behind its pad) then clears WP's shoulder
        d_tcp = self.PLANE + opt("DS_LINK")
        r.moveE("L", self.WP[0], wp_top + opt("DS_UP"), d_tcp, speed=0.04, settle=0.5)
        ok = r.moveE("L", self.WP[0], wp_top + opt("DS_SET"), d_tcp, speed=0.02, settle=0.5, fmax=8.0)
        log.append(("L_on_WP", ok, [round(float(v), 3) for v in r.E(r.tcp("L")[0])]))
        hold = bool(opt("DS_HOLD"))
        if not hold:
            self.drape_release("L")
            r.park("L")
            if getattr(self, "_stop_after_L", False):
                return
        # (DS_HOLD: L keeps the loop's top over WP while R takes its right side up onto ALT; L lets go once the loop
        # runs from ALT over WP, so the hanging loop no longer draws the top out of the plane as L's fingers leave)
        tx, tz_hi, tz_lo = opt("DS_RT")
        for attempt in range(2):
            P = self.pts("hung_WP")
            n = len(P)
            k0 = int(np.argmin(np.hypot(P[:, 0] - self.WP[0], P[:, 1] - wp_top) + 10.0 * np.abs(P[:, 2] - self.PLANE)))
            sgn = 1 if P[(k0 + 3) % n][0] > P[(k0 - 3) % n][0] else -1
            # R's grip: on the loop's right side, 22-38 points (about 0.3 m) round from WP's top, where the side is
            # straight (nearest the height DS_RZ: not in the bend at the bottom, which swings under the fingers)
            js = [(k0 + sgn * j) % n for j in range(22, 39)]
            kb = min(js, key=lambda k: abs(P[k][1] - opt("DS_RZ")))
            log.append(("R_pt", k0, kb, [round(float(v), 3) for v in P[kb]]))
            self.drape_grab("R", [(P[kb], self.normal_at(P, kb))])
            pb = r.hand_pos("R")
            tx, tz_hi, tz_lo = opt("DS_RT")
            m1 = r.moveE("R", pb[0], pb[1], 0.10, speed=0.02)
            m2 = r.moveE("R", tx, tz_hi, 0.10, speed=0.06, settle=0.5)
            log.append(("R_carry", m1, m2, round(r.load_f("R"), 1), [round(float(v), 3) for v in r.E(r.tcp("R")[0])]))
            try:
                r.confirm_load("R", opt("CF_DRAPE_R"), "drape_R_over_ALT", log)
                break
            except RuntimeError:
                # the belt slipped out of the grip on the carry (it falls back over WP's top): let go, look again
                if attempt:
                    raise
                log.append(("R_regrab",))
                r.set_width("R", 0.04, wait=0.3)
                r.grasp["R"] = None
                r.park("R")
                r.wait(1.0)
        d_tcp = self.PLANE + opt("DS_LINK")
        m3 = r.moveE("R", tx, tz_hi, d_tcp, speed=0.04, settle=0.5)
        ok = r.moveE("R", tx, tz_lo, d_tcp, speed=0.02, settle=0.5, fmax=10.0)
        log.append(("R_set", m3, ok, round(r.load_f("R"), 1)))
        log.append(("R_on_ALT", ok, [round(float(v), 3) for v in r.E(r.tcp("R")[0])]))
        if hold:
            self.drape_release("L")
            r.park("L")
        self.drape_release("R")
        r.park("R")
        r.wait(1.0)
        r.relief = set()

    def drape_release(self, side):
        """open the fingers a little (wide open, the lower finger would swing into the pulley's rim), slide them
        straight out of the belt plane slowly, then back off"""
        r = self.r
        r.set_width(side, r.width(side) + opt("DS_OPEN"), wait=0.3)
        if opt("DS_TURN") and side == "L":
            # tilt the lower finger about the approach axis: the belt resting on it slides off sideways onto the pulley
            g, Rg = r.goal[side]
            r.move(side, g, speed=0.02, fmax=40.0, R=hk.rot_axis(r.into, math.radians(opt("DS_TURN"))) @ Rg)
            r.wait(0.5)
        if side == "L" and opt("DS_PITCH"):
            # pitch the hand about its x axis (along the belt) through the fingertips, the approach turning downwards:
            # the lower finger's face then slopes down towards the engine face and the belt resting on it slides off
            # its tip into the groove; then out along the approach. A belt that comes with the fingers (its edge
            # caught on the pad's ridges: the flange still carries it) is carried back, and the next pitch is tried.
            g0, R0 = r.goal[side]
            g0, R0 = g0.copy(), R0.copy()
            tipc = g0 + R0[:, 2] * 0.009
            for k, pitch in enumerate(opt("DS_PITCHES")):
                best = None
                for sgn in (1.0, -1.0):
                    Rr = hk.rot_axis(R0[:, 0], sgn * math.radians(pitch))
                    if best is None or (Rr @ R0)[:, 2][2] < best[1][:, 2][2]:
                        best = (Rr, Rr @ R0)
                Rr, Rn = best
                p1 = tipc + Rr @ (g0 - tipc)
                r.move(side, p1, speed=0.01, fmax=40.0, R=Rn)
                r.wait(opt("DS_PITCH_WAIT"))
                r.move(side, p1 - Rn[:, 2] * 0.03, speed=0.02, fmax=40.0)
                r.wait(0.3)
                f = r.load_f(side)
                self.log.append(("drape_pitch", side, pitch, round(f, 2)))
                if f < opt("DS_HOOK_N") or k == len(opt("DS_PITCHES")) - 1:
                    break
                r.move(side, p1, speed=0.02, fmax=40.0)
                r.move(side, g0, speed=0.01, fmax=40.0, R=R0)
                r.wait(0.5)
        p = r.goal[side][0].copy()
        r.move(side, p - r.into * 0.04, speed=opt("DS_OUT_V"), fmax=40.0)
        r.wait(0.3)
        f = r.load_f(side)
        r.move(side, p - r.into * 0.10, speed=0.05, fmax=40.0)
        self.log.append(("drape_released", side, round(f, 2), round(r.load_f(side), 2)))

    def push_seat(self, pulley, angles, side="L", fmax=None):
        r = self.r
        cc, rr = self.dw.pul[pulley]["c"], self.dw.pul[pulley]["r"]
        for ang in angles:
            u = np.array([math.cos(math.radians(ang)), math.sin(math.radians(ang))])
            x, z = cc + u * (rr + 0.0115)
            r.push(side, x, z, 0.075, 0.0375, radial_xz=u, fmax=opt("PUSH_FMAX") if fmax is None else fmax)

    def st_seat_top(self):
        self.push_seat("ALT", (60, 90, 120), side="R")
        self.push_seat("WP", (100, 140), side="L")
        self.r.park("R")

    def st_lift_span(self):
        r, log = self.r, self.log
        r.park("L")
        P = self.pts("lift_span")
        # (the right span hangs between C's right side and T's: anything further right is not the belt)
        x_max = self.dw.T_centre(self.dw.th_closed)[0] + self.dw.T_r + 0.12
        cand = np.nonzero((P[:, 0] > 0.06) & (P[:, 0] < x_max) & (P[:, 1] > 0.0) & (P[:, 1] < 0.07))[0]
        if not len(cand):
            raise RuntimeError("lift_span: no belt seen below T")
        tgt = np.array([0.095, 0.035])
        k = int(cand[np.argmin(np.hypot(P[cand, 0] - tgt[0], P[cand, 1] - tgt[1]))])
        r.grab_twist_max = 25.0
        r.grab("L", self.aim_fn("L", [(P[k], self.normal_at(P, k))], twist_max=25.0))
        r.grab_twist_max = 0.0
        pk = r.hand_pos("L")
        log.append(("lift_pt", [round(float(v), 4) for v in pk]))
        r.moveE("L", pk[0], pk[1], 0.07, speed=0.02, settle=0.2)

    def open_to_install(self, tag):
        r = self.r
        tgt = self.dw.th_open - math.radians(2.0)
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
            if th >= tgt - math.radians(1.0) or r.pin_force() > opt("OPEN_MAX_N") or r.lever_over:
                break
            r.turn_lever(r.th_cmd + min(math.radians(opt("OPEN_STEP_DEG")), tgt - th), rate_dps=opt("OPEN_RATE"))
            r.wait(0.3)
            n += 1
        self.log.append((tag, round(r.pin_force(), 1), round(math.degrees(r.theta_est()), 1), round(math.degrees(r.th_cmd), 1), n))

    def st_open_T(self):
        r = self.r
        r.pin_lever()
        self.open_to_install("T_open")
        g = r.hand_pos("L")
        xl = self.x_on_line(g[1])
        o1 = r.moveE("L", xl, g[1], 0.058, speed=0.02)
        r.release("L")
        r.moveE("L", xl, g[1], 0.12, speed=0.04, settle=0.5)
        self.log.append(("span_on_line", o1))

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
        # the held span pulls the compliant hand off its goal: take the goal to where the hand is before opening, so
        # the hand does not spring (with a finger) into ALT's or T's rim when the span lets go of it
        if opt("REL_RESET"):
            r.relief = set()
            p0, R0 = r.tcp("L")
            r.command_cart("L", p0, R0)
            r.wait(0.3)
        r.set_width("L", r.width("L") + 0.003, wait=0.15)
        p = r.tcp("L")[0]
        r.move("L", p - r.into * 0.05, speed=0.03, fmax=40.0)
        gL = r.hand_pos("L")
        r.moveE("L", gL[0], gL[1], 0.13, speed=0.05, fmax=40.0)

    def st_mid_high(self, pre=False):
        r, log = self.r, self.log
        if self.ctx.get("hb_done") and not pre:
            log.append(("mid_high", "skipped: hold-bottom return done"))
            return
        zq = opt("MH_Z")
        xq = self.x_on_line(zq) - opt("MH_DX")
        P = self.pts("mid_pre" if pre else "mid_high")
        right = np.nonzero((P[:, 0] > 0.03) & (np.abs(P[:, 1] - zq) < 0.02))[0]
        if not len(right):
            right = np.nonzero((P[:, 0] > 0.03) & (np.abs(P[:, 1] - zq) < 0.05))[0]
        if not len(right):
            raise RuntimeError("mid_high: no belt on the right span near z %.2f" % zq)
        k = int(right[np.argmin(np.abs(P[right, 1] - zq))])
        n = len(P)
        s_up = 1 if P[(k + 1) % n, 1] > P[(k - 1) % n, 1] else -1
        cands = [(P[(k + j * s_up) % n], self.normal_at(P, (k + j * s_up) % n)) for j in (0, 1, 2)]
        r.grab("L", self.aim_fn("L", cands), open_w=opt("MH_OPEN_W"), alts=2)
        self.ctx["L_hold_t"] = r.time
        r.relief_total = {}
        r.relief = {"L": "yz"}
        # where the grabbed span lies: the eyes' last aim (the tcp sits wherever the compliant hand ended up)
        aims = [g["aim_E"] for g in r.grasp_log if g.get("ok") and "aim_E" in g]
        pk = np.asarray(aims[-1], float) if (aims and opt("MID_AIM")) else r.E(r.tcp("L")[0])
        log.append(("mid_grab_pt", [round(float(v), 4) for v in pk], [round(float(v), 4) for v in r.E(r.tcp("L")[0])]))
        fm = opt("MP_FMAX") if pre else opt("MH_FMAX")
        if abs(pk[2] - self.PLANE) < 0.008:
            m1, m3 = "in_plane", "in_plane"
            m2 = r.moveE("L", xq, zq, self.PLANE, speed=0.01, fmax=min(fm, opt("MH_FMAX_TAUT")))
            r.wait(0.3)
        elif pre:
            d0 = float(min(max(pk[2], self.PLANE), 0.06))
            m1 = "no_out"
            m2 = r.moveE("L", xq, zq, d0, speed=0.015, fmax=fm)
            m3 = r.moveE("L", xq, zq, self.PLANE, speed=0.01, settle=0.3, fmax=fm) if m2 else "skip"
        else:
            m1 = r.moveE("L", pk[0], pk[1], max(pk[2], 0.06), speed=0.02, fmax=fm)
            m2 = r.moveE("L", xq, zq, max(pk[2], 0.06), speed=0.02, fmax=fm)
            m3 = r.moveE("L", xq, zq, self.PLANE, speed=0.01, settle=0.3, fmax=fm)
        if pre and (m2 is False or m3 is not True or r.load("L") > fm):
            ld = round(r.load("L"), 1)
            back = self.relax_hold("L", f_to=opt("MP_RELAX_N"), max_d=0.06)
            log.append(("mid_pre_undo", m2, m3, ld, back))
            r.relief = set()
            self.release_hold()
            r.wait(0.3)
            return
        r.confirm_load("L", opt("CF_MID"), "mid_pre_set" if pre else "mid_high_set", log)
        for _ in range(int(opt("MH_DEPTH_ITER"))):
            d_act = float(r.E(r.tcp("L")[0])[2])
            if d_act - self.PLANE < 0.0008 or r.load("L") > fm:
                break
            g = r.hand_pos("L")
            r.moveE("L", g[0], g[1], g[2] - min(0.004, d_act - self.PLANE), speed=0.005, settle=0.2, fmax=fm)
        log.append(("mid_high", m1, m2, m3, round(r.load("L"), 1), round(float(r.E(r.tcp("L")[0])[2]), 4)))
        if not pre:
            r.hold_L = True
            return
        if opt("MP_RELAX_ALWAYS"):
            log.append(("mid_pre_relax", self.relax_hold("L", f_to=opt("MP_RELAX_N"), max_d=0.06)))
        r.relief = set()
        self.release_hold()
        r.wait(0.3)

    def st_mid_pre(self):
        if opt("MID_PRE"):
            self.st_mid_high(pre=True)
            self.ctx.pop("L_hold_t", None)

    def bottom_depth_at_C(self):
        """(n, mean depth) of the belt seen within C's rim + 12 mm (front camera; arms parked)"""
        self.r.wait(opt("C_SETTLE"))           # the released bottom swings for a second or so
        P = self.pts("C_depth")
        rr = self.dw.pul["C"]["r"]
        sel = np.hypot(P[:, 0] - self.C0[0], P[:, 1] - self.C0[1]) < rr + 0.012
        return int(sel.sum()), (round(float(P[sel, 2].mean()), 4) if sel.any() else None)

    def c_grab_carry(self):
        r = self.r
        dw = self.dw
        tip = opt("C_TIP")
        rc = dw.pul["C"]["r"] + dw.shoulder_h + 0.5 * dw.belt_thick + opt("C_GAP")
        d_tcp = dw.front_sh[1] + dw.chamfer + 0.001 + 0.0089
        if r.grasp.get("L") is None:
            r.park("L")
        P = self.pts("C_lowest")
        k = int(np.argmin(P[:, 1]))
        if P[k, 1] > self.C0[1] - 0.5 * dw.pul["C"]["r"]:
            raise RuntimeError("eyes: the loop's bottom is not seen below C (lowest point z %.3f)" % P[k, 1])
        sgn_ = opt("C_YSIGN") * (-1 if self.ctx.get("c_takes", 0) % 2 else 1)
        self.ctx["c_takes"] = self.ctx.get("c_takes", 0) + 1
        # (the hand's 180-deg alternatives about its approach axis put the arm in different postures under C: a
        # retake uses the other one)
        yp = None if not opt("C_YSIGN") else sgn_ * r.R_E[:, 2]
        r.grab("L", self.aim_fn("L", [(P[k], self.normal_at(P, k))]), tip_off=tip, w_max=opt("C_GRAB_WMAX"), y_pref=yp)
        pk = r.hand_pos("L")
        d_front = max(float(pk[2]), 0.065)
        tgt = self.C0 + np.array([0.0, -1.0]) * rc
        r.moveE("L", pk[0], pk[1], d_front, speed=0.03, fmax=opt("C_FMAX"))
        ok1 = r.moveE("L", tgt[0], tgt[1], d_front, speed=0.02, fmax=opt("C_FMAX"))
        r.confirm_load("L", opt("CF_C"), "C_under_rim", self.log)
        ok2 = r.moveE("L", tgt[0], tgt[1], d_tcp, speed=0.015, settle=0.3)
        return ok1, ok2

    def c_release(self):
        r = self.r
        r.set_width("L", 0.020, wait=0.3)
        p = r.tcp("L")[0]
        r.move("L", p - r.into * 0.06, speed=0.04, fmax=40.0)
        r.wait(0.3)

    def st_C(self):
        r = self.r
        r.park("R") if r.grasp.get("R") is None else None
        ok1, ok2 = self.c_grab_carry()
        self.c_release()
        r.wait(0.2)
        self.log.append(("C_edge", ok1, ok2))
        r.park("L")
        n, dC = self.bottom_depth_at_C()
        self.log.append(("after_C", n, dC))
        for i in range(int(opt("C_RETRY"))):
            if dC is not None and dC <= opt("C_RETRY_D"):
                break
            # the bottom came to rest in front of C: take it again
            ok1, ok2 = self.c_grab_carry()
            self.c_release()
            r.wait(0.2)
            r.park("L")
            n, dC = self.bottom_depth_at_C()
            self.log.append(("C_retry", i + 1, ok1, ok2, n, dC))
        if opt("HB_ENABLE") and (dC is None or dC > opt("C_SLACK_D")):
            self.hold_bottom_return()

    def hold_bottom_return(self):
        r, log = self.r, self.log
        fails = self.ctx.setdefault("prog_fail", [])
        self.c_grab_carry()
        l0 = r.load_f("L")
        cur = r.theta_est()
        r.th_cmd = cur
        f0 = r.load_f("R")
        f_min = 1e9
        rate = opt("HB_RATE")
        released = False
        stop = why = None
        while cur > self.dw.th_closed + math.radians(2.0):
            cur = r.turn_lever(cur - math.radians(rate) * hrig.TICK * 5, rate_dps=rate)
            f = r.load_f("R")
            lL = r.load_f("L")
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
                cur = r.turn_lever(cur + math.radians(2.0), rate_dps=2.0)
                rate = max(2.0, 0.5 * rate)
                continue
            if not released:
                if lL > l0 + opt("HB_DL") or f < opt("HB_FREL") * f0:
                    why = "L_load" if lL > l0 + opt("HB_DL") else "lever_unload"
                    self.c_release()
                    released = True
                continue
            f_min = min(f_min, f)
            if f < 1.5:
                caught = True
                for _ in range(10):
                    r.wait(0.1)
                    if r.load_f("R") > 3.0:
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
            fails.append("HB_release_after_R_stop")
            self.c_release()
        log.append(("hb_return", stop, why, round(math.degrees(r.theta_est()), 1)))
        r.unpin()
        g = r.hand_pos("R")
        r.moveE("R", g[0] + 0.05, g[1], 0.15, speed=0.05)
        r.wait(1.0)
        self.ctx["hb_done"] = True

    def st_seat_bottom(self):
        if self.ctx.get("hb_done"):
            self.log.append(("seat_bottom", "skipped: hold-bottom return done"))
            return
        for ang in opt("SC_ANG"):
            self.push_seat("C", (ang,), side="L")

    def l_holding(self):
        r = self.r
        return r.grasp.get("L") is not None and r.width("L") >= 0.002

    def guided_return(self, L_holding, tag, rate_dps=10.0):
        r, log = self.r, self.log
        cur = r.theta_est()
        r.th_cmd = cur
        f0 = r.load_f("R")
        released = not L_holding
        why_L = None if L_holding else "not_holding"
        l0 = r.load_f("L") if L_holding else 0.0
        # the force across the held span, positive towards the loop's inside (the drive's left, -x of E): the hand's
        # closing axis may point either way across the span (its 180-deg alternatives), so take its sign from the pose
        # (measured, seed 89: the tensioner's face pushing the span in loads the flange along +x of E, the force the
        # arm exerts on the hand to hold the span against it)
        ysgn = 1.0 if float(r.tcp("L")[1][:, 1] @ r.R_E[:, 0]) >= 0 else -1.0
        if not opt("RT_YSIGN"):
            ysgn = 1.0
        fy0 = ysgn * float(r.ft_f("L")[1]) if L_holding else 0.0
        fl_min = 1e9
        f_min = 1e9
        stop = None
        th_last = math.radians(opt("RT_LAST"))
        fails = self.ctx.setdefault("prog_fail", [])
        trace = []
        if not L_holding and r.hand_pos("L")[2] < 0.12:
            r.set_width("L", 0.03, wait=0.2)
            gL = r.hand_pos("L")
            r.moveE("L", gL[0], gL[1], 0.13, speed=0.05)
        while cur > self.dw.th_closed + math.radians(2.0):
            cur = r.turn_lever(cur - math.radians(rate_dps) * hrig.TICK * 5, rate_dps=rate_dps)
            f = r.load_f("R")
            if r.lever_over:
                r.lever_over = False
                if not released:
                    fails.append("L_release_lever_over")
                    why_L = "lever_over"
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
                cur = r.turn_lever(cur + math.radians(2.0), rate_dps=2.0)
                rate_dps = max(2.0, 0.5 * rate_dps)
                continue
            if not released:
                lL, mL, wL = r.load_f("L"), r.grip_moment("L"), r.width("L")
                trace.append((round(math.degrees(cur), 1), round(f, 2), round(lL, 2), round(ysgn * float(r.ft_f("L")[1]) - fy0, 2), round(mL, 2)))
                fl_min = min(fl_min, lL)
                why = None
                if wL < 0.001:
                    why = "FAIL_grip_slipped"
                elif mL > opt("RT_MOM"):
                    why = "moment"
                elif lL > l0 + opt("RT_DL"):
                    why = "load_up"
                elif ysgn * float(r.ft_f("L")[1]) - fy0 > opt("RT_DFY"):
                    why = "lateral_push"
                elif fl_min < opt("RT_MIN_DROP") * l0 and lL > fl_min + opt("RT_DMIN"):
                    why = "load_above_min"
                elif lL > opt("RT_FMAX"):
                    why = "load_max"
                elif f < opt("RT_FB_LEVER") * f0:
                    why = "FAIL_fallback_lever_load"
                elif cur < th_last:
                    why = "FAIL_fallback_angle"
                if why is not None:
                    if why.startswith("FAIL"):
                        fails.append("L_release_" + why)
                    why_L = "%s at %.1f deg (load %.1f N, start %.1f, lever %.1f N)" % (why, math.degrees(cur), lL, l0, f)
                    r.relief = set()
                    self.release_hold()
                    released = True
                    continue
            f_min = min(f_min, f)
            if f < 1.5:
                caught = True
                for _ in range(10):
                    r.wait(0.1)
                    if r.load_f("R") > 3.0:
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
            why_L = "after_R_stop"
            fails.append("L_release_after_R_stop")
            self.release_hold()
        log.append(("return_" + tag, stop, why_L, round(math.degrees(r.theta_est()), 1), trace[::3]))
        r.unpin()
        g = r.hand_pos("R")
        r.moveE("R", g[0] + 0.05, g[1], 0.15, speed=0.05)
        r.wait(1.0)
        return stop, why_L

    def st_return_T(self):
        if self.ctx.get("hb_done"):
            self.log.append(("return_T", "skipped: hold-bottom return done"))
            return
        r = self.r
        holding = self.l_holding()
        if holding:
            r.wait(0.5)
            holding = self.l_holding()
        if not holding:
            self.log.append(("L_not_holding_at_return",))
        self.guided_return(L_holding=holding, tag="T")

    FIX_ANGLES = {"C": (-150, -120, -90, -60, -30, 0), "ALT": (-10, 20, 50, 80, 110, 140, 170), "WP": (80, 110, 140, 170, 200, 230)}

    def check_now(self, tag):
        r = self.r
        for s in ("L", "R"):
            r.park(s)
        th_hand = r.theta_est()
        out = self.y.check(th_prior=th_hand)
        po = out["pointer"]
        if po.get("theta_deg") is None:
            # the blade not made out in the front frame: the angle at which the right hand left the lever stands
            win_ = [math.degrees(v) for v in self.dw.window]
            po.update({"theta_deg": round(math.degrees(th_hand), 2), "from": "hand",
                       "ok": bool(win_[0] <= math.degrees(th_hand) <= win_[1])})
        bad = [k for k in ("C", "ALT", "WP", "T") if not out[k]["ok"]]
        win = [math.degrees(v) for v in self.dw.window]
        off = [k for k in ("C", "ALT", "WP") if (out[k].get("cover") or 0.0) < 0.3]
        if po.get("theta_deg") is not None and po["theta_deg"] < win[0] - 2.0:
            off.append("pointer_closed")
        if not po.get("ok") and not bad and not off:
            bad = ["C", "ALT"]
        if po.get("theta_deg") is not None:
            r.th_seen = math.radians(po["theta_deg"])
        self.log.append((tag, {k: out[k] for k in out}, bad, off))
        return bad, off

    def st_check(self):
        r = self.r
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
            r.turn_lever(min(th_now + math.radians(opt("FIX_OPEN_DEG")), self.dw.th_open - math.radians(2.0)), rate_dps=10.0)
            for k in [b for b in bad if b != "T"]:
                self.push_seat(k, self.FIX_ANGLES[k], side="L")
            self.guided_return(L_holding=False, tag="fix%d" % attempt)
            fixed = True
        if fixed:
            self.check_now("check_end")
        for s in ("L", "R"):
            r.park(s)

    ORDER = ["look", "drape", "seat_top", "lift_span", "open_T", "mid_pre", "C", "seat_bottom", "mid_high", "return_T", "check"]

    def run(self):
        stages = {"look": self.st_look, "drape": self.st_drape, "seat_top": self.st_seat_top, "lift_span": self.st_lift_span,
                  "open_T": self.st_open_T, "mid_pre": self.st_mid_pre, "C": self.st_C, "seat_bottom": self.st_seat_bottom,
                  "mid_high": self.st_mid_high, "return_T": self.st_return_T, "check": self.st_check}
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
        """after a failure: never leave the lever loaded (let the tensioner back guided if R holds it)"""
        r = self.r
        try:
            if r.grasp.get("R") == ("lever",):
                self.guided_return(L_holding=self.l_holding(), tag="safe")
        except Exception:  # noqa: BLE001
            log(traceback.format_exc())
