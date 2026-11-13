"""The program's robot layer over the cell's wire API (cell_api.md): stepping, the flange F/T (software tare with the
hand's data-sheet weight removed), Cartesian moves with force limits, joint transits planned on the program's own
two-arm model (hk.Kin), grips checked from public signals (finger width, flange load), the tensioner lever held and
turned by the arm on the tensioner's side (the left one on drive B) with its angle read from the hand's own pose.

Every number comes from describe(), the given documents or the given Panda model; the engine pose comes from the
program's eyes (heyes.engine_pose)."""
import math
import os
import sys

import numpy as np

import hk

TICK = 0.01
ARM = {"L": "left", "R": "right"}
PERIOD = 2                       # ticks per control period (one command + step + state read per period)


class Stopped(RuntimeError):
    pass


def log(*a):
    print(*a, file=sys.stderr, flush=True)


class Drawing(object):
    """The work order's drawing (describe()["drive"]) in handy form; E coordinates (x, z, d)."""

    def __init__(self, info):
        dr = info["drive"]
        # grooved pulleys (groove-floor radius) and smooth backside idlers (face radius)
        self.pul = {k: {"c": np.asarray(v["centre_xz"], float), "r": float(v.get("groove_floor_radius", v.get("face_radius"))),
                        "kind": v["kind"]} for k, v in dr["pulleys"].items()}
        self.idlers = [k for k, v in dr["pulleys"].items() if v["kind"] == "idler"]
        g = dr["groove"]
        self.zone = tuple(g["groove_zone_d"])
        self.front_sh = tuple(g["front_shoulder_d"])
        self.back_sh = tuple(g["back_shoulder_d"])
        self.shoulder_h = float(g["shoulder_height_above_floor"])
        self.chamfer = 0.0025                                   # "45 deg, 2.5 mm" (drawing note)
        self.plane = 0.5 * (self.zone[0] + self.zone[1])        # the groove zone's middle: where a seated belt's centre runs
        tp = dr["tensioner_pulley"]
        self.T_r = float(tp["radius"])
        self.T_face = tuple(tp["face_d"])
        t = dr["tensioner"]
        self.pivot = np.asarray(t["pivot_xz"], float)
        self.arm_len = float(t["arm_length"])
        self.th_closed = math.radians(t["closed_stop_deg"])
        self.th_open = math.radians(t["install_stop_deg"])
        self.window = tuple(math.radians(v) for v in t["window_marks_deg"])
        self.sense = 1 if self.th_open > self.th_closed else -1          # +1: opens counter-clockwise as drawn
        lv = t["service_lever"]
        self.lever_r = float(lv["radius_from_pivot"])
        self.lever_off = math.radians(lv["angle_from_pulley_arm_deg"])
        self.lever_d = tuple(lv["d_range"])
        self.lever_dia = float(lv["diameter"])
        pg = dr["peg"]
        self.peg = np.asarray(pg["axis_xz"], float)
        self.peg_r = float(pg["radius"])
        self.peg_d = tuple(pg["d_range"])
        # the route as drawn (counter-clockwise): "C, ALT, I, WP, T (...)"; the back wraps T and the idlers
        self.route = [k.strip() for k in dr["route"].split("(")[0].split(",") if k.strip()]
        b = info["belt"]
        self.belt_thick = 0.0045                                # "20 mm wide x 4.5 mm thick" (belt section)
        self.belt_w = 0.020
        self.belt_len = float(b["nominal_length_m"])

    def T_centre(self, th):
        return self.pivot + self.arm_len * np.array([math.cos(th), math.sin(th)])

    def opener(self, th, deg):
        """th moved deg towards the install stop (negative: towards the closed stop)"""
        return th + self.sense * math.radians(deg)

    def more_open(self, a, b):
        """a is further open than b"""
        return self.sense * (a - b) > 0


class Rig(object):
    CLEAR_MARGIN = 0.026
    SLAB_MARGIN = 0.020
    SLOW_V, SLOW_CL = 0.04, 0.06
    TRANSIT_NEAR, TRANSIT_V_NEAR, TRANSIT_V_FAR = 0.10, 0.03, 0.6
    PARK_E = {"L": (-0.30, 0.48, 0.25), "R": (0.36, 0.40, 0.25)}
    RELIEF = {"gain_dps_per_Nm": 60.0, "max_dps": 25.0, "deadband_Nm": 0.08, "sign": -1.0}
    RELIEF_HOLD = {"gain_dps_per_Nm": 60.0, "max_dps": 25.0, "deadband_Nm": 0.03, "sign": -1.0, "cap_deg": {"y": 8.0, "z": 12.0}}
    FT_EMA = 0.2
    W_TURN = 0.5
    GRASP_W = {"belt": (0.0015, 0.019), "lever": (0.0135, 0.0175)}
    GRASP_W_TWIST = 0.019
    TUG = {"dist": 0.004, "speed": 0.01, "fmax": 8.0, "belt_N": 0.6, "lever_N": 3.0, "avg_s": 0.1}
    LEVER_STIFF = 1.0

    def __init__(self, cell, given_dir):
        self.cell = cell
        self.info = cell.describe()
        self.dw = Drawing(self.info)
        # the arm on the tensioner's side holds its lever; the other one handles the belt
        self.TS = "L" if self.dw.pivot[0] < 0.0 else "R"
        self.BS = "R" if self.TS == "L" else "L"
        self.kin = hk.Kin(self.info, os.path.join(given_dir, "panda"))
        hs = self.info["robots"]["hand_data_sheet"]
        self.tool_m = float(hs["mass_kg"])
        self.tool_c = np.asarray(hs["com_in_hand_frame_m"], float)
        f = self.info["frames"]["engine_nominal_pose"]
        self.set_engine(np.asarray(f["origin_world"], float),
                        np.column_stack([f["x_E_world"], f["y_E_world"], f["z_E_world"]]).astype(float))
        self.st = cell.get_state()
        self.mode = {s: self.st[ARM[s]]["mode"] for s in ARM}
        self.stiff = {s: float(self.st[ARM[s]]["stiffness_scale"]) for s in ARM}
        self.goal = {s: None for s in ARM}
        self.width_cmd = {s: float(self.st[ARM[s]]["gripper_width"]) for s in ARM}
        self.tare = {s: np.zeros(6) for s in ARM}
        self.ema = {s: None for s in ARM}
        self.relief = set()
        self.relief_total = {}
        self.grasp = {s: None for s in ARM}
        self.grasp_log = []
        self.blocked = 0
        self.min_clearance = 1.0
        self.last_cl = 1.0
        self.max_lever_load = 0.0
        self.lever_over = False
        self.grab_twist_max = 0.0
        self.stage = None
        self.n_periods = 0
        self.frames = 0
        self._sync_kin()
        for s in ARM:
            self.do_tare(s)

    # ------------------------------------------------------------------ frames
    def set_engine(self, E_pos, R_E):
        self.E_pos, self.R_E = np.asarray(E_pos, float), np.asarray(R_E, float)
        self.kin.set_engine(self.E_pos, self.R_E)

    def W(self, x, z, dd):
        return self.E_pos + self.R_E @ np.array([x, -dd, z])

    def E(self, p):
        q = (np.asarray(p, float) - self.E_pos) @ self.R_E
        return np.array([q[0], q[2], -q[1]])

    @property
    def into(self):
        return self.R_E[:, 1]

    @property
    def time(self):
        return float(self.st["sim_time"])

    # ------------------------------------------------------------------ state, F/T
    def _sync_kin(self):
        self.kin.load({s: self.st[ARM[s]]["q"] for s in ARM}, {s: self.st[ARM[s]]["gripper_width"] for s in ARM})

    def tcp(self, side):
        a = self.st[ARM[side]]
        return np.asarray(a["tcp_pos"], float), hk.quat_to_mat(a["tcp_quat"])

    def q(self, side):
        return np.asarray(self.st[ARM[side]]["q"], float)

    def width(self, side):
        return float(self.st[ARM[side]]["gripper_width"])

    def _gwrench(self, side):
        R = self.tcp(side)[1]
        Fg = R.T @ np.array([0.0, 0.0, -9.81 * self.tool_m])
        return -np.concatenate([Fg, np.cross(self.tool_c, Fg)])

    def _raw(self, side):
        f = self.st[ARM[side]]["ft"]
        return np.concatenate([np.asarray(f["force_N"], float), np.asarray(f["torque_Nm"], float)])

    def ft(self, side):
        return self._raw(side) - self._gwrench(side) - self.tare[side]

    def ft_f(self, side):
        e = self.ema[side]
        return self.ft(side) if e is None else e

    def do_tare(self, side):
        self.tare[side] = self._raw(side) - self._gwrench(side)
        self.ema[side] = None

    def load(self, side):
        return float(np.linalg.norm(self.ft(side)[:3]))

    def load_f(self, side):
        return float(np.linalg.norm(self.ft_f(side)[:3]))

    def grip_moment(self, side):
        w = self.ft_f(side)
        return float(np.linalg.norm(w[3:] - np.cross(np.array([0.0, 0.0, hk.TCP_Z]), w[:3])))

    # ------------------------------------------------------------------ control
    def set_mode(self, side, mode, stiff=None):
        stiff = self.stiff[side] if stiff is None else float(stiff)
        if mode != self.mode[side] or abs(stiff - self.stiff[side]) > 1e-9:
            self.cell.set_control(ARM[side], mode, stiff)
            self.mode[side] = mode
            self.stiff[side] = stiff
            if mode == "cartesian":
                p, R = self.tcp(side)
                self.goal[side] = (p.copy(), R.copy())

    def set_stiffness(self, side, s):
        self.set_mode(side, self.mode[side], s)

    def cart(self, side):
        if self.mode[side] != "cartesian":
            self.set_mode(side, "cartesian")
        if self.goal[side] is None:
            p, R = self.tcp(side)
            self.goal[side] = (p.copy(), R.copy())

    def _send(self, sides):
        spec = {}
        for s in sides:
            p, R = self.goal[s]
            spec[ARM[s]] = {"tcp_pos": [float(v) for v in p], "tcp_quat": [float(v) for v in hk.mat_to_quat(R)]}
        if spec:
            self.cell.command(**spec)

    def command_cart(self, side, p, R):
        self.goal[side] = (np.asarray(p, float).copy(), np.asarray(R, float).copy())
        self._send([side])

    def command_joint(self, side, q):
        self.cell.command(**{ARM[side]: {"q": [float(v) for v in q]}})

    def _relief_axes(self, side):
        rel = self.relief
        g = self.grasp.get(side)
        if not rel or g is None or g[0] != "belt":
            return "", None
        if isinstance(rel, dict):
            ax = rel.get(side, "")
            return ax, (self.RELIEF_HOLD if "y" in ax else self.RELIEF)
        return ("z", self.RELIEF) if side in rel else ("", None)

    def _relief(self, side, nt):
        axes, P = self._relief_axes(side)
        if not axes or self.mode[side] != "cartesian" or self.goal[side] is None:
            return False
        w = self.ft_f(side)
        mom = w[3:] - np.cross(np.array([0.0, 0.0, hk.TCP_Z]), w[:3])
        p, Rg = self.goal[side]
        turned = False
        for c in axes:
            i = {"x": 0, "y": 1, "z": 2}[c]
            if abs(mom[i]) < P["deadband_Nm"]:
                continue
            rate = float(np.clip(P["sign"] * P["gain_dps_per_Nm"] * mom[i], -P["max_dps"], P["max_dps"]))
            dphi = math.radians(rate) * TICK * nt
            key = side if c == "z" else side + c
            cap = math.radians(P.get("cap_deg", {}).get(c, 1e3))
            tot = self.relief_total.get(key, 0.0)
            dphi = float(np.clip(tot + dphi, -cap, cap)) - tot
            if dphi == 0.0:
                continue
            Rg = hk.rot_axis(Rg[:, i], dphi) @ Rg
            self.relief_total[key] = tot + dphi
            turned = True
        if turned:
            self.goal[side] = (p, Rg)
        return turned

    def tick(self, n=PERIOD, check=None):
        """advance n ticks (one control period: the wrist relief may turn a holding hand's goal first)"""
        turned = [s for s in ARM if self._relief(s, n)]
        if turned:
            self._send(turned)
        r = self.cell.step(int(n))
        self.st = self.cell.get_state()
        self.n_periods += 1
        a = 1.0 - (1.0 - self.FT_EMA) ** n
        for s in ARM:
            raw = self.ft(s)
            e = self.ema[s]
            self.ema[s] = raw.copy() if e is None else e + a * (raw - e)
        if r.get("stopped") or self.st.get("stopped"):
            raise Stopped("protective stop (cell reports stopped) during %s" % self.stage)
        if r.get("done"):
            raise Stopped("episode over (sim time cap) during %s" % self.stage)
        self._sync_kin()
        cl = self.kin.clearance(cap=0.08)
        self.last_cl = cl
        self.min_clearance = min(self.min_clearance, cl)
        if check:
            self.check_clear(check)

    def wait(self, s):
        n = int(round(s / TICK))
        while n > 0:
            k = min(PERIOD if self.relief else 50, n)
            self.tick(k)
            n -= k

    def check_clear(self, what):
        if self.last_cl < self.CLEAR_MARGIN:
            self.blocked += 1
            raise RuntimeError("arm clearance %.1f mm (< margin) during %s (%s; L tcp E %s, R tcp E %s)" % (
                1000 * self.last_cl, what, getattr(self.kin, "last_pair", None), np.round(self.E(self.tcp("L")[0]), 3),
                np.round(self.E(self.tcp("R")[0]), 3)))

    # ------------------------------------------------------------------ Cartesian motion
    def hand_pos(self, side):
        if self.mode[side] == "cartesian" and self.goal[side] is not None:
            return self.E(self.goal[side][0])
        return self.E(self.tcp(side)[0])

    def _v_gov(self, speed):
        return min(speed, self.SLOW_V) if self.last_cl < self.SLOW_CL else speed

    PARK_FAR_E = {"L": (-0.42, 0.55, 0.32), "R": (0.46, 0.52, 0.32)}

    def _precheck(self, side, p0, tgt, R):
        """the straight move's clearance to the other arm where it stands (IK at a few points of the line); when it
        would come within the planning margin and the other arm is free, that arm first steps further out of the way"""
        other = "R" if side == "L" else "L"
        if self.grasp.get(other) is not None or getattr(self, "_in_precheck", False):
            return
        if float(np.linalg.norm(np.asarray(tgt) - p0)) < 0.02:
            return
        q = self.q(side)
        worst = 1.0
        for u in (0.25, 0.5, 0.75, 1.0):
            q, ep, er, ok = self.kin.ik(side, p0 + (np.asarray(tgt) - p0) * u, R, q_ref=q, seeds=0)
            if not ok:
                return
            worst = min(worst, self.kin.clearance({side: q}))
        if worst >= self.CLEAR_MARGIN + 0.01:
            return
        log("[precheck] %s move would pass %.1f mm from %s: %s steps further out" % (side, 1000 * worst, other, other))
        self._in_precheck = True
        try:
            self.retreat(other, clear_d=0.25)
            x, z, dd = self.PARK_FAR_E[other]
            self.move(other, self.W(x, z, dd), speed=0.15, fmax=60.0, R=hk.hand_R(self.into, self.R_E[:, 2]))
            self.wait(0.3)
        finally:
            self._in_precheck = False

    def move(self, side, target, speed=0.05, settle=0.0, fmax=None, R=None):
        self.cart(side)
        if self.last_cl < 0.10:
            self._precheck(side, self.goal[side][0].copy(), target, self.goal[side][1] if R is None else R)
        p0 = self.goal[side][0].copy()
        Rs = self.goal[side][1].copy()
        tgt = np.asarray(target, float)
        dist = float(np.linalg.norm(tgt - p0))
        fmax = 25.0 if fmax is None else fmax
        if R is not None:
            v = hk.rotvec(R @ Rs.T)
            ang = float(np.linalg.norm(v))
            ax = v / ang if ang > 1e-9 else np.array([0.0, 0.0, 1.0])
        else:
            ang, ax = 0.0, None

        def orient(u):
            if R is None:
                return self.goal[side][1]
            if ang < math.radians(3.0):
                return R
            return hk.rot_axis(ax, ang * u) @ Rs

        u = 0.0
        while u < 1.0:
            du = self._v_gov(speed) * TICK * PERIOD / max(dist, 1e-9)
            if ang >= math.radians(3.0):
                du = min(du, self.W_TURN * TICK * PERIOD / ang)
            u = min(1.0, u + du)
            self.command_cart(side, p0 + (tgt - p0) * u, orient(u))
            self.tick(PERIOD, check="a %s move" % side)
            if self.load(side) > fmax:
                back = (tgt - p0) / max(dist, 1e-9) * 0.002
                self.command_cart(side, p0 + (tgt - p0) * u - back, orient(u))
                self.tick(3)
                self.blocked += 1
                return False
        if settle > 0:
            self.wait(settle)
        return True

    def moveE(self, side, x, z, dd, **kw):
        return self.move(side, self.W(x, z, dd), **kw)

    def move2(self, tL, tR, speed=0.05, rot=None):
        for s in ARM:
            self.cart(s)
        p0 = {s: self.goal[s][0].copy() for s in ARM}
        tgt = {"L": p0["L"] if tL is None else np.asarray(tL), "R": p0["R"] if tR is None else np.asarray(tR)}
        dist = max(1e-9, max(float(np.linalg.norm(tgt[s] - p0[s])) for s in tgt))
        rot = rot or {}
        rmax = max([abs(v) for v in rot.values()] + [1e-9])
        u = 0.0
        while u < 1.0:
            du = min(self._v_gov(speed) * TICK * PERIOD / dist, 0.5 * TICK * PERIOD / rmax, 1.0 - u)
            u += du
            for s in ARM:
                Rk = hk.rot_axis(self.into, rot.get(s, 0.0) * du) @ self.goal[s][1]
                self.goal[s] = (p0[s] + (tgt[s] - p0[s]) * u, Rk)
            self._send(["L", "R"])
            self.tick(PERIOD, check="a two-hand move")

    def retreat(self, side, clear_d=0.16):
        self.cart(side)
        pE = self.hand_pos(side)
        if pE[2] < clear_d:
            p0 = self.goal[side][0].copy()
            self.move(side, p0 - self.into * (clear_d - pE[2]), speed=0.08, fmax=60.0)

    def joint_transit(self, side, q_t, v_max):
        self.set_mode(side, "joint", 1.0)
        q0 = self.q(side)
        dq = q_t - q0
        s = 0.0
        prev = self.kin.link_points(side, q0)
        dt = TICK * PERIOD
        while s < 1.0:
            ds = min(1.0 - s, 0.04)
            while True:
                nxt = self.kin.link_points(side, q0 + dq * (s + ds))
                v = float(np.max(np.linalg.norm(nxt - prev, axis=1))) / dt
                if v <= v_max or ds < 1e-4:
                    break
                ds *= 0.5 * v_max / v if v > 0 else 1.0
            s += ds
            prev = nxt
            self.command_joint(side, q0 + dq * s)
            self.tick(PERIOD, check="a %s joint transit" % side)
        for _ in range(150):
            self.tick(PERIOD, check="a %s joint transit" % side)
            if float(np.max(np.abs(self.q(side) - q_t))) < 0.004 and float(np.max(np.abs(np.asarray(self.st[ARM[side]]["dq"])))) < 0.05:
                break

    def pick_R(self, side, pos, Rs, q_ref=None):
        """of the hand orientations Rs (e.g. a grip and its 180-deg turn about the approach axis), the one whose IK at pos
        keeps the arm furthest from its joint limits (then nearest q_ref)"""
        best = None
        for i, R in enumerate(Rs):
            q, ep, er, ok = self.kin.ik(side, pos, R, q_ref=self.q(side) if q_ref is None else q_ref)
            mg = self.kin.margin(side, q) if ok else -1.0
            key = (0 if ok else 1, 0 if mg >= 0.25 else 1, -mg if mg < 0.25 else 0.0, i)
            if best is None or key < best[0]:
                best = (key, R)
        return best[1]

    @staticmethod
    def flip(R):
        return R @ np.diag([-1.0, -1.0, 1.0])

    def goto_joint(self, side, pos, R, d_safe=0.17):
        if self.grasp.get(side) is None:
            self.retreat(side, clear_d=d_safe)
        self.cart(side)
        self.set_stiffness(side, 1.0)
        tE = self.E(pos)
        above = self.W(tE[0], tE[1], max(d_safe, tE[2]))
        q_now = self.q(side)
        plan = self._plan(side, q_now, above, R)
        if plan is None and self.grasp.get(side) is None:
            # no clear direct path: by way of the park pose (if that leg is clear)
            x, z, dd = self.PARK_E[side]
            Rp = hk.hand_R(self.into, self.R_E[:, 2])
            leg = self._plan(side, q_now, self.W(x, z, dd), Rp)
            if leg is not None:
                self.joint_transit(side, leg[0], leg[1])
                q_now = self.q(side)
                plan = self._plan(side, q_now, above, R)
        if plan is not None:
            self.joint_transit(side, plan[0], plan[1])
            self.set_mode(side, "cartesian", 1.0)
            self.command_cart(side, above, R)
        else:
            self.move(side, above, speed=0.15, fmax=60.0, R=R)
        self.wait(0.3)
        self.fix_track(side)
        if tE[2] < d_safe:
            self.move(side, pos, speed=0.06, fmax=40.0, R=R)
        self.wait(0.1)
        self.fix_track(side)

    def _path_cl(self, side, q0, q1, n_s=48):
        """(the path's clearance to the other arm, or None when it passes the slab or the margin)"""
        cl = 1.0
        for k in range(1, n_s + 1):
            qk = q0 + (q1 - q0) * k / float(n_s)
            if self.kin.slab_clearance(side, qk, cap=0.05) < self.SLAB_MARGIN:
                return None
            cl = min(cl, self.kin.clearance({side: qk}))
            if cl < self.CLEAR_MARGIN + 0.015:
                return None
        return cl

    def _plan(self, side, q0, pos, R):
        """(q target, transit speed) of a clear straight joint path to the pose, trying the IK from the current
        posture and from home; None if neither is clear"""
        why = []
        for q_ref in (q0, self.kin.home[side]):
            q_t, ep, er, ok = self.kin.ik(side, pos, R, q_ref=q_ref)
            if not ok or self.kin.margin(side, q_t) < 0.05:
                why.append(("ik", ok, round(self.kin.margin(side, q_t), 2)))
                continue
            cl = self._path_cl(side, q0, q_t)
            if cl is not None:
                return q_t, (self.TRANSIT_V_NEAR if cl < self.TRANSIT_NEAR else self.TRANSIT_V_FAR)
            why.append(("path", round(1000 * self.kin.clearance({side: q_t}), 1), round(1000 * self.kin.slab_clearance(side, q_t, cap=0.05), 1),
                        round(1000 * self.kin.slab_clearance(side, q0, cap=0.05), 1)))
        log("[plan] %s no clear joint path to %s: %s" % (side, np.round(self.E(pos), 3), why))
        return None

    def fix_track(self, side, tol=0.004):
        """the cell's Cartesian null space can drift an arm onto a joint limit, where the hand then stops short of its
        goal: when the tcp is more than tol off its goal, re-posture in joint mode (the program's own IK from the
        current posture, away from the limits) and take the goal up again in Cartesian mode"""
        if self.mode[side] != "cartesian" or self.goal[side] is None:
            return 0.0
        p, R = self.goal[side]
        err = float(np.linalg.norm(self.tcp(side)[0] - p))
        if err <= tol:
            return err
        q, ep, er, ok = self.kin.ik(side, p, R, q_ref=self.q(side))
        if not ok or self.kin.margin(side, q) < 0.1:
            q, ep, er, ok = self.kin.ik(side, p, R, q_ref=self.kin.home[side], seeds=12)
        log("[fix_track] %s err %.1f mm, ik ok %s (%.1f mm, %.2f deg) margin %.2f -> %.2f" % (
            side, 1000 * err, ok, 1000 * ep, math.degrees(er), self.kin.margin(side, self.q(side)), self.kin.margin(side, q)))
        if not ok:
            return err
        if self._path_cl(side, self.q(side), q) is None:
            log("[fix_track] %s re-posture path not clear of the other arm / the drive: left as is" % side)
            return err
        self.joint_transit(side, q, 0.05)
        self.set_mode(side, "cartesian", self.stiff[side])
        self.command_cart(side, p, R)
        self.wait(0.3)
        self.track_fixes = getattr(self, "track_fixes", 0) + 1
        return float(np.linalg.norm(self.tcp(side)[0] - p))

    def park(self, side):
        if self.grasp.get(side) is not None:
            return
        self.retreat(side, clear_d=0.25)
        x, z, dd = self.PARK_E[side]
        R = hk.hand_R(self.into, self.R_E[:, 2])
        self.move(side, self.W(x, z, dd), speed=0.15, fmax=60.0, R=R)
        self.wait(0.3)

    # ------------------------------------------------------------------ gripper
    def set_width(self, side, w, wait=0.3):
        w = float(min(0.08, max(0.0, w)))
        if w > self.width(side) + 0.002:
            self.grasp[side] = None
        self.cell.command(**{ARM[side]: {"width": w}})
        self.width_cmd[side] = w
        self.wait(wait)

    def settle_width(self, side, tol=0.0001, step=0.05, max_s=1.2):
        w = self.width(side)
        t = 0.0
        while t < max_s:
            self.wait(step)
            t += step
            w2 = self.width(side)
            if abs(w2 - w) < tol:
                return w2
            w = w2
        return w

    def _f_avg(self, side, secs):
        n = max(1, int(round(secs / (TICK * PERIOD))))
        acc = np.zeros(3)
        for _ in range(n):
            self.tick(PERIOD)
            acc += self.ft(side)[:3]
        return acc / n

    def tug(self, side, u, dist=None):
        T = self.TUG
        dist = T["dist"] if dist is None else dist
        g = self.goal[side][0].copy()
        f0 = self._f_avg(side, T["avg_s"])
        self.move(side, g + np.asarray(u, float) * dist, speed=T["speed"], fmax=T["fmax"])
        self.wait(0.1)
        f1 = self._f_avg(side, T["avg_s"])
        self.move(side, g, speed=T["speed"], fmax=30.0)
        return float(np.linalg.norm(f1 - f0))

    def grasp_check(self, side, w_max=None):
        w = self.settle_width(side)
        lo, hi = self.GRASP_W["belt"]
        if self.grab_twist_max > 0.0:
            hi = self.GRASP_W_TWIST
        if w_max is not None:
            hi = w_max
        return bool(lo <= w <= hi), {"w_mm": round(1000 * w, 2)}

    def confirm_load(self, side, f_min, what, logl=None):
        f = self.load_f(side)
        self.grasp_log.append({"confirm": what, "side": side, "load": round(f, 2), "f_min": f_min, "t": round(self.time, 2)})
        if logl is not None:
            logl.append(("confirm_load", what, round(f, 2)))
        if f < f_min:
            raise RuntimeError("%s: the belt is not carried (load %.2f N < %.2f): grasp lost" % (what, f, f_min))

    def grab(self, side, aim, approach_d=0.06, tip_off=0.0, open_w=0.03, w_max=None, alts=None, pull_check=0.0, d_safe=0.17, y_pref=None):
        """Grip the belt across its thickness from the front. aim(i) -> (centre_world, closing_world) of the i-th
        candidate (the caller's eyes; called again at the standoff to re-aim). tip_off: the tcp that far in front of
        the belt's centre. Success: the hand reached it (its in-move did not stop on its force limit) and the closed
        fingers stopped on something of the belt's thickness."""
        n_alt = 1 + int(alts or 0)
        cand = 0
        self.set_width(side, open_w, wait=0.05)
        c, y = aim(cand, "far")
        R = hk.hand_R(self.into, y)
        if y_pref is not None:
            if float(R[:, 1] @ np.asarray(y_pref, float)) < 0:
                R = self.flip(R)
            # unless that posture brings the arm down near the base plate / pallet
            q_, _, _, ok_ = self.kin.ik(side, c - self.into * approach_d, R, q_ref=self.q(side))
            if not ok_ or float(np.min(self.kin.link_points(side, q_)[:, 2])) < 0.12:
                R = self.pick_R(side, c - self.into * approach_d, [R, self.flip(R)])
        else:
            R = self.pick_R(side, c - self.into * approach_d, [R, self.flip(R)])
        y_ref = R[:, 1].copy()
        self.goto_joint(side, c - self.into * approach_d, R, d_safe=d_safe)
        last_reached = True
        for attempt in range(3):
            if attempt and n_alt > 1 and not last_reached:
                cand = min(attempt, n_alt - 1)
            c, y = aim(cand, "near")
            if float(np.dot(y, y_ref)) < 0:
                y = -y
            R = hk.hand_R(self.into, y)
            self.move(side, c - self.into * 0.03, speed=0.04, fmax=30.0, R=R)
            # re-aim once more from the 3 cm standoff (the slack belt moves), after the belt settles
            self.wait(0.3)
            c, y = aim(cand, "near")
            if float(np.dot(y, y_ref)) < 0:
                y = -y
            R = hk.hand_R(self.into, y)
            self.move(side, c - self.into * 0.03, speed=0.02, fmax=30.0, R=R)
            self.fix_track(side)
            self.do_tare(side)
            reached = self.move(side, c - self.into * tip_off, speed=0.02, fmax=20.0, R=R)
            last_reached = bool(reached)
            self.wait(0.1)
            if reached:
                self.grasp[side] = ("belt", cand)
                self.set_width(side, 0.0, wait=0.4)
                self.do_tare(side)
                ok, info = self.grasp_check(side, w_max=w_max)
                if ok and pull_check:
                    # the fingers may stop on a link that lies just outside the pads (not held): draw the hand 2 cm
                    # out of the plane; a held link brings the belt (its weight or its pull) onto the flange
                    g0 = self.goal[side][0].copy()
                    self.move(side, g0 - self.into * 0.02, speed=0.02, fmax=30.0)
                    self.wait(0.2)
                    info["pull_N"] = round(self.load_f(side), 2)
                    if info["pull_N"] < pull_check:
                        ok = False
                        self.move(side, g0, speed=0.02, fmax=30.0)
            else:
                ok, info = False, {"reached": False, "load": round(self.load(side), 2)}
            info.update({"side": side, "cand": cand, "attempt": attempt, "t": round(self.time, 2), "ok": bool(ok),
                         "aim_E": [round(float(v), 4) for v in self.E(c)]})
            self.grasp_log.append(info)
            if ok:
                self.grasp[side] = ("belt", cand)
                return info
            self.set_width(side, open_w, wait=0.3)
            self.grasp[side] = None
            self.move(side, self.goal[side][0] - self.into * 0.02, speed=0.03, fmax=40.0)
        raise RuntimeError("grip missed (%s)" % side)

    def release(self, side, retreat=0.05):
        self.set_width(side, 0.03, wait=0.3)
        p = self.goal[side][0] if self.goal[side] is not None else self.tcp(side)[0]
        self.move(side, p - self.into * retreat, speed=0.05, fmax=40.0)

    # ------------------------------------------------------------------ fingertip pushes
    def push(self, side, x, z, d_from, d_to, radial_xz=None, speed=0.02, fmax=12.0):
        other = "R" if side == "L" else "L"
        if self.grasp.get(other) is None and getattr(self, "_parked", {}).get(other) is not True:
            self.park(other)
            self._parked = {other: True, side: False}
        if radial_xz is not None:
            tang = np.array([-radial_xz[1], radial_xz[0]])
            closing = self.R_E @ np.array([tang[0], 0.0, tang[1]])
        else:
            closing = self.R_E[:, 0]
        R = hk.hand_R(self.into, closing)
        R = self.pick_R(side, self.W(x, z, d_from + 0.010), [R, self.flip(R)])
        self.set_width(side, 0.0, wait=0.1)
        tip_off = 0.010
        self.goto_joint(side, self.W(x, z, d_from + tip_off), R)
        self.do_tare(side)
        ok = self.move(side, self.W(x, z, d_to + tip_off), speed=speed, fmax=fmax)
        self.wait(0.1)
        self.move(side, self.W(x, z, d_from + tip_off), speed=0.05, fmax=40.0)
        return ok

    # ------------------------------------------------------------------ the lever (the tensioner-side arm)
    def lever_E(self, th):
        """the lever's grip point in E (18 mm in from its front end) at tensioner angle th"""
        a = th + self.dw.lever_off
        p = self.dw.pivot + self.dw.lever_r * np.array([math.cos(a), math.sin(a)])
        return np.array([p[0], p[1], self.dw.lever_d[1] - 0.018])

    def _lever_frame(self, th):
        gE = self.lever_E(th)
        grip = self.W(*gE)
        piv = self.W(self.dw.pivot[0], self.dw.pivot[1], gE[2])
        ax = -self.into
        u = grip - piv
        u = u - (u @ ax) * ax
        u /= np.linalg.norm(u)
        approach = -u
        closing = np.cross(ax, approach)
        return grip, hk.hand_R(approach, closing), -approach

    def theta_est(self):
        """the tensioner's angle from the lever hand's actual tcp while it holds the lever (the lever's axis runs
        between the pads); otherwise the last value"""
        if self.grasp.get(self.TS) == ("lever",):
            pE = self.E(self.tcp(self.TS)[0])
            v = pE[:2] - self.dw.pivot
            self.th_seen = math.atan2(v[1], v[0]) - self.dw.lever_off
            self.th_seen = (self.th_seen + math.pi) % (2 * math.pi) - math.pi
        return getattr(self, "th_seen", self.dw.th_closed)

    def pin_lever(self):
        th = self.theta_est()
        grip, R, back = self._lever_frame(th)
        self.set_width(self.TS, 0.05, wait=0.05)
        self.goto_joint(self.TS, grip + back * 0.07, R)
        eye = getattr(self, "lever_eye", None)
        if eye is not None:
            # look at the (orange) lever from the standoff and re-aim on where it is
            th_seen = eye()
            first = not hasattr(self, "eye_bias")
            if th_seen is not None and first and abs(th - self.dw.th_closed) < 1e-6:
                # the first look is at the documented start state (the tensioner on its closed stop): the eye's own
                # bias (it sees the bar's near surface, not its axis) is measured here and removed from later looks
                self.eye_bias = th_seen - th
                log("[pin] lever eye bias %.2f deg (closed stop)" % math.degrees(self.eye_bias))
                th_seen = None
            elif th_seen is not None:
                th_seen -= getattr(self, "eye_bias", 0.0)
            if th_seen is not None and abs(th_seen - th) < math.radians(15.0):
                log("[pin] lever seen at %.1f deg (assumed %.1f)" % (math.degrees(th_seen), math.degrees(th)))
                th = th_seen
                self.th_seen = th
                grip, R2, back = self._lever_frame(th)
                if float(np.trace(R2.T @ R)) < 1.0:
                    R2 = self.flip(R2)
                R = R2
                self.move(self.TS, grip + back * 0.07, speed=0.03, fmax=20.0, R=R)
        for attempt in range(3):
            grip, _, back = self._lever_frame(th)
            self.move(self.TS, grip + back * 0.025, speed=0.03, fmax=20.0, R=R)
            self.wait(0.1)
            self.do_tare(self.TS)
            self.move(self.TS, grip, speed=0.02, fmax=15.0, R=R)
            self.set_width(self.TS, 0.0, wait=0.4)
            w = self.settle_width(self.TS)
            lo, hi = self.GRASP_W["lever"]
            info = {"side": self.TS, "lever": True, "w_mm": round(1000 * w, 2), "attempt": attempt, "t": round(self.time, 2)}
            ok = lo <= w <= hi
            if ok:
                info["tug_back"] = round(self.tug(self.TS, back, dist=0.002), 2)
                ok = info["tug_back"] >= self.TUG["lever_N"]
            info["ok"] = bool(ok)
            self.grasp_log.append(info)
            if ok:
                self.grasp[self.TS] = ("lever",)
                break
            self.set_width(self.TS, 0.05, wait=0.3)
            self.move(self.TS, self.tcp(self.TS)[0] + back * 0.03, speed=0.03, fmax=40.0, R=R)
        if self.grasp.get(self.TS) != ("lever",):
            raise RuntimeError("lever grip missed")
        self.do_tare(self.TS)
        self.piv = self.W(self.dw.pivot[0], self.dw.pivot[1], 0.0)
        self.axis = -self.into
        self.rel0 = self.goal[self.TS][0] - self.piv
        self.R_pin = self.goal[self.TS][1].copy()
        self.th0 = self.theta_est()
        self.th_cmd = self.th0
        self.lever_over = False
        self.set_stiffness(self.TS, self.LEVER_STIFF)

    def unpin(self):
        # never let the lever go loaded (a snap-back damages the damper): first guide the tensioner on to its closed
        # stop if the lever still carries more than 3 N (the belt is not holding it)
        if self.grasp.get(self.TS) == ("lever",) and self.load_f(self.TS) > 3.0:
            log("[unpin] lever still loaded %.1f N at %.1f deg: guiding to the closed stop" % (self.load_f(self.TS), math.degrees(self.theta_est())))
            self.th_cmd = self.theta_est()
            self.turn_lever(self.dw.opener(self.dw.th_closed, -1.0), rate_dps=8.0)
            self.wait(0.5)
        # take the hand's goal to where the hand is (the arc command may lead the lever that the belt now holds: let
        # go spring-loaded, the hand would swing along the arc into the tensioner's arm)
        p0, R0 = self.tcp(self.TS)
        self.command_cart(self.TS, p0, R0)
        self.wait(0.4)
        # open a little and slide the fingers off the lever the way the hand came in (radially out of its arc),
        # then out of the face
        self.set_width(self.TS, min(0.08, self.width(self.TS) + 0.006), wait=0.3)
        self.set_stiffness(self.TS, 1.0)
        p, Rh = self.tcp(self.TS)
        self.move(self.TS, p - Rh[:, 2] * 0.03, speed=0.02, fmax=40.0)
        p = self.tcp(self.TS)[0]
        self.move(self.TS, p - self.into * 0.06, speed=0.05, fmax=40.0)

    def pin_force(self):
        return self.load(self.TS)

    def turn_lever(self, th_target, rate_dps=10.0, stop_on_unload=False):
        cur = self.th_cmd
        step = math.radians(rate_dps) * TICK * PERIOD * (1 if th_target > cur else -1)
        while (th_target - cur) * step > 0:
            cur = cur + step if abs(th_target - cur) > abs(step) else th_target
            Rr = hk.rot_axis(self.axis, cur - self.th0)
            self.command_cart(self.TS, self.piv + Rr @ self.rel0, Rr @ self.R_pin)
            self.tick(PERIOD, check="the lever turn")
            f = self.pin_force()
            self.max_lever_load = max(self.max_lever_load, f)
            if stop_on_unload and f < 1.5:
                break
            if f > 26.0:
                self.blocked += 1
                self.lever_over = True
                break
        self.th_cmd = cur
        return cur
