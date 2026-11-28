"""Privileged reference controller for server_hot_swap_maintenance: reads the token truth (the rack's pose, the failed
parts, every part's pose and state) and drives both arms with Cartesian steps computed from the observed tool poses.

Usage: o = Oracle(); action = o.act(obs) per step, obs as on the wire (with obs["truth"])."""

import math

import numpy as np

V_FAST, V_MED, V_SLOW = 0.012, 0.006, 0.003
W_MAX = 0.08
CAD_D = 0.150
GRASP_X = 0.030          # body grasp point behind the caddy's front face
HOLD = {"l": None, "r": None}


def quat_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rotvec(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = math.sin(th)
    if s < 1e-6:
        # near pi
        w, V = np.linalg.eigh((R + R.T) / 2)
        a = V[:, np.argmax(w)]
        return a * th
    return v * th / (2 * s)


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


R_DY = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])     # pointing down, fingers opening along y
R_DX = rotz(math.pi / 2) @ R_DY                                  # pointing down, fingers opening along x
R_FY = np.array([[0, 0, 1.0], [0, -1.0, 0], [1.0, 0, 0]])         # pointing +x, fingers opening along y


def _mk(z, y):
    z, y = np.array(z, float), np.array(y, float)
    return np.stack([np.cross(y, z), y, z], axis=1)


_TILT = math.radians(40.0)            # the support hand points across and down, so that its wrist clears the grid
R_SIDE = {"l": _mk((0, -math.cos(_TILT), -math.sin(_TILT)), (1, 0, 0)),
          "r": _mk((0, math.cos(_TILT), -math.sin(_TILT)), (-1, 0, 0))}   # x-pinch


class Oracle:
    def __init__(self, log=None):
        self.obs = None
        self.gen = None
        self.log = log or (lambda *a: None)
        self.integ = {"l": np.zeros(3), "r": np.zeros(3)}
        self.prev = {"l": None, "r": None}
        self.grip = {"l": -1.0, "r": -1.0}

    # --- observation helpers ----------------------------------------------------------------------------------
    def tool(self, s):
        """The tool's pose from the truth (not from the observation: the script then runs the same whichever copy of
        a repeated step comes first)."""
        a = self.obs["truth"]["tools"][s]
        return np.array(a["pos"], float), np.array(a["R"], float)

    @property
    def T(self):
        return self.obs["truth"]

    def rack(self):
        return np.array(self.T["rack_pos"]), np.array(self.T["rack_R"])

    def caddy(self, n):
        c = self.T["caddies"][n]
        return np.array(c["pos"]), np.array(c["R"])

    def fan(self, n):
        c = self.T["fans"][n]
        return np.array(c["pos"]), np.array(c["R"])

    # --- one arm's step toward a pose -------------------------------------------------------------------------
    def step_to(self, s, p, R, v=V_MED, w=W_MAX, integ=False):
        pc, Rc = self.tool(s)
        e = np.asarray(p, float) - pc
        moved = np.linalg.norm(pc - self.prev[s]) if self.prev[s] is not None else 0.0
        self.prev[s] = pc
        if integ and moved < 0.5 * v:
            self.integ[s] = self.integ[s] + 0.15 * np.clip(e, -0.01, 0.01)
            n_ = np.linalg.norm(self.integ[s])
            if n_ > 0.025:
                self.integ[s] *= 0.025 / n_
        else:
            self.integ[s] *= 0.7
        dp = e.copy()
        n = np.linalg.norm(dp)
        if n > v:
            dp *= v / n
        dp = dp + (self.integ[s] if integ else 0.0)
        dr = rotvec(R @ Rc.T)
        n = np.linalg.norm(dr)
        if n > w:
            dr *= w / n
        return list(dp) + list(dr) + [self.grip[s]], float(np.linalg.norm(e)), float(np.linalg.norm(rotvec(R @ Rc.T)))

    def hold(self, s):
        return [0.0] * 6 + [self.grip[s]]

    # --- generators: each yields one arm's 7-number action per step ------------------------------------------
    def g_move(self, s, target, v=V_MED, tol=0.003, rtol=0.10, settle=2, maxn=150, integ=False):
        """target: callable () -> (p, R) or a tuple."""
        k = 0
        for _ in range(maxn):
            p, R = target() if callable(target) else target
            a, e, er = self.step_to(s, p, R, v=v, integ=integ)
            if e < tol and er < rtol:
                k += 1
                if k >= settle:
                    return True
            else:
                k = 0
            yield (s, a)
        self.log("move timeout", s, "pos err %.4f rot err %.3f" % (e, er))
        return False

    def g_front(self, s, target, v=V_FAST, tol=0.006):
        """Go to `target` by way of the plane 0.10 m in front of the bay: first out to it, then across, then in."""
        rp, rR = self.rack()
        p, R = target() if callable(target) else target
        xs = rp[0] - 0.10
        pc, _ = self.tool(s)
        if pc[0] > xs:
            yield from self.g_move(s, (np.array([xs, pc[1], max(pc[2], p[2])]), R), v=v, tol=0.02)
        yield from self.g_move(s, (np.array([xs, p[1], p[2]]), R), v=v, tol=0.01)
        yield from self.g_move(s, target, v=V_MED, tol=tol)

    def g_set(self, flag, key):
        flag[key] = True
        return
        yield

    def g_grip(self, s, g, n=6):
        self.grip[s] = g
        for _ in range(n):
            yield (s, self.hold(s))

    def g_wait(self, s, n):
        for _ in range(n):
            yield (s, self.hold(s))

    def g_until(self, s, cond, maxn=300):
        for _ in range(maxn):
            if cond():
                return True
            yield (s, self.hold(s))
        return False

    def g_hold_pose(self, s, target, cond, maxn=300, v=V_MED, integ=True):
        for _ in range(maxn):
            if cond():
                return True
            p, R = target() if callable(target) else target
            a, _e, _r = self.step_to(s, p, R, v=v, integ=integ)
            yield (s, a)
        return False

    # --- composition ------------------------------------------------------------------------------------------
    def run(self, *gens):
        """Run generators (each yields (arm, action)) side by side until all are done; yields full actions."""
        live = [g for g in gens if g is not None]
        while live:
            act = {}
            nxt = []
            for g in live:
                try:
                    sa, a = next(g)
                    act[sa] = a
                    nxt.append(g)
                except StopIteration:
                    pass
            live = nxt
            if not act and not live:
                break
            for sa in ("l", "r"):
                act.setdefault(sa, self.hold(sa))
            yield act

    def seq(self, *gens):
        for g in gens:
            yield from g

    # --- geometry ---------------------------------------------------------------------------------------------
    def rack_R_hand(self, base):
        _p, rR = self.rack()
        yaw = math.atan2(rR[1, 0], rR[0, 0])
        return rotz(yaw) @ base

    def caddy_point(self, n, local):
        p, R = self.caddy(n)
        return p + R @ np.asarray(local, float)

    # --- the job ----------------------------------------------------------------------------------------------
    def act(self, obs):
        self.obs = obs
        if self.gen is None:
            self.gen = self.main()
        try:
            a = next(self.gen)
        except StopIteration:
            a = {"l": self.hold("l"), "r": self.hold("r")}
        out = {"l": [float(x) for x in a["l"]], "r": [float(x) for x in a["r"]], "cams": 0}
        for s in ("l", "r"):            # a Cartesian step toward a target, measured from the observed tool position
            if len(out[s]) == 7 and any(abs(x) > 1e-9 for x in out[s][:6]):
                tp, op = self.obs["truth"]["tools"][s]["pos"], obs[s]["ee_pos"]
                for i in range(3):
                    out[s][i] += float(tp[i]) - float(op[i])
        if a.get("done"):
            out["done"] = True
        return out

    def park(self, s):
        y = 0.30 if s == "l" else -0.30
        return (np.array([0.30, y, 0.45]), R_DY)

    def main(self):
        T = self.T
        failed = list(T["failed_slots"])
        spares = ["s0", "s1"]
        import os
        if os.environ.get("ORACLE_ONLY") == "fan":
            failed = []
        for i, k in enumerate(failed):
            yield from self.remove_drive(k)
            yield from self.insert_drive(spares[i], k)
        yield from self.swap_fan(int(T["failed_fan"]))
        for n, c in sorted(self.T["caddies"].items()):
            if c["slot"] is not None and c["seated"] and not c["latched"]:
                yield from self.close_lever(n, "l" if self.T["slot_frames"][c["slot"]][1] - self.rack()[0][1] > 0 else "r")
        yield from self.run(self.g_move("l", self.park("l"), v=V_FAST, tol=0.01),
                            self.g_move("r", self.park("r"), v=V_FAST, tol=0.01))
        while True:
            a = {"l": self.hold("l"), "r": self.hold("r")}
            if self.T.get("complete"):
                a["done"] = True
            yield a

    def remove_drive(self, k):
        n = "c%d" % k
        rp, rR = self.rack()
        up = np.array([0, 0, 1.0])
        Ry = self.rack_R_hand(R_DY)
        Rx = self.rack_R_hand(R_DX)
        Rf = self.rack_R_hand(R_FY)
        # A works the lever from above; B takes the caddy from its inner side (the arm of the other side reaches
        # across: a left-hand slot is taken by the right arm, pointing +y, and the other way round)
        B = "r" if self.T["slot_frames"][k][1] - rp[1] > 0 else "l"
        A = "l" if B == "r" else "r"
        # 1. press the release button on the closed lever with the closed fingers (A)
        btn = lambda: self.caddy_point(n, (-0.007, 0.015, 0.008))
        self.grip[A] = 1.0
        yield from self.run(self.seq(
            self.g_move(A, lambda: (btn() + 0.06 * up, Ry), v=V_FAST, tol=0.004),
            self.g_move(A, lambda: (btn() + 0.012 * up, Ry), v=V_MED, tol=0.002),
            self.g_hold_pose(A, lambda: (btn() - 0.006 * up, Ry), lambda: not self.T["caddies"][n]["latched"],
                             maxn=150, v=V_SLOW),
            self.g_move(A, lambda: (btn() + 0.05 * up, Ry), v=V_FAST, tol=0.006),
            self.g_until(A, lambda: self.T["caddies"][n]["lever"] > 1.5, maxn=60)),
            self.g_move(B, self.park(B), v=V_FAST, tol=0.02))
        # 2. grasp the standing lever plate (A, fingers across its thickness) and draw the caddy 110 mm up
        grasp = lambda: self.caddy_point(n, (-0.0265, 0.0, 0.014))
        yield from self.run(self.seq(
            self.g_grip(A, 1.0 - 0.025 / 0.04, 6),           # 25 mm open: the fingers stay off the neighbours
            self.g_move(A, lambda: (grasp() + 0.06 * up, Rx), v=V_FAST, tol=0.004),
            self.g_move(A, lambda: (grasp(), Rx), v=V_MED, tol=0.0015),
            self.g_grip(A, 1.0, 8)))
        start = self.caddy(n)[0].copy()
        p0, _ = self.tool(A)
        yield from self.run(self.g_hold_pose(
            A, lambda: (p0 + 0.110 * up, Rx), lambda: self.caddy(n)[0][2] - start[2] > 0.103, maxn=300, v=V_SLOW))
        # 3. B takes the body from its side (hand across, fingers across the caddy's thickness); A lets go
        body = lambda: self.caddy_point(n, (0.0, 0.0, -0.032))
        Rs = self.rack_R_hand(R_SIDE[B])
        side = rR[:, 1] * (1.0 if B == "l" else -1.0)    # from the slot toward B's hand
        front = rR[:, 0]
        flag = {"b": False}
        yield from self.run(
            self.g_hold_pose(A, lambda: (p0 + 0.110 * up, Rx), lambda: flag["b"], maxn=600),
            self.seq(self.g_grip(B, -1.0, 2),
                     self.g_move(B, lambda: (body() + 0.12 * side + 0.12 * up, Rs), v=V_FAST, tol=0.008),
                     self.g_move(B, lambda: (body() + 0.08 * side + 0.067 * up, Rs), v=V_MED, tol=0.003),
                     self.g_move(B, lambda: (body(), Rs), v=V_SLOW, tol=0.0008, settle=4),
                     self.g_grip(B, 1.0 - 0.0345 / 0.04, 8),
                     self.g_set(flag, "b")))
        pA, _ = self.tool(A)
        yield from self.run(self.g_grip(A, -1.0, 1), self.g_grip(B, 1.0, 1))
        yield from self.run(self.g_grip(A, -1.0, 8), self.g_grip(B, 1.0, 8))
        yield from self.run(self.seq(
            self.g_grip(A, -1.0, 1),
            self.g_move(A, (pA + 0.06 * up, Rx), v=V_MED, tol=0.01),
            self.g_move(A, self.park(A), v=V_FAST, tol=0.03)))
        pB, _ = self.tool(B)
        yield from self.run(None, self.seq(
            self.g_move(B, (pB + 0.15 * up, Rs), v=V_SLOW, tol=0.006, integ=True),
            self.g_move(B, (pB + 0.15 * up - 0.15 * front, Rs), v=V_SLOW, tol=0.01, integ=True),
            self.g_move(B, (np.array([0.20, -0.12, 0.50]), R_SIDE[B]), v=V_MED, tol=0.02, maxn=150, integ=True),
            self.g_move(B, (np.array([0.20, -0.12, 0.44]), R_SIDE[B]), v=V_MED, tol=0.02, maxn=60, integ=True),
            self.g_grip(B, -1.0, 15),
            self.g_move(B, (np.array([0.20, -0.20, 0.52]), R_SIDE[B]), v=V_FAST, tol=0.02),
            self.g_move(B, self.park(B), v=V_FAST, tol=0.03)))

    def insert_drive(self, sp, k):
        rp, rR = self.rack()
        up = np.array([0, 0, 1.0])
        Ry = self.rack_R_hand(R_DY)
        A = "l"
        slot = np.array(self.T["slot_frames"][k])          # the seated caddy's top centre
        body = lambda: self.caddy_point(sp, (0.0, 0.0, -0.012))
        yield from self.run(self.seq(
            self.g_grip(A, -1.0, 2),
            self.g_move(A, lambda: (body() + 0.10 * up, R_DY), v=V_FAST, tol=0.008),
            self.g_move(A, lambda: (body(), R_DY), v=V_MED, tol=0.002),
            self.g_grip(A, 1.0, 8)))

        off = self.tool(A)[0] - self.caddy(sp)[0]        # the grasp's offset, fixed when taken

        def hand_for(q):
            return q + off, Ry

        lift = self.caddy(sp)[0] + np.array([0, 0, 0.21])
        yield from self.run(self.g_move(A, lambda: hand_for(lift), v=V_MED, tol=0.01, integ=True))
        yield from self.run(self.seq(
            self.g_move(A, lambda: hand_for(slot + 0.175 * up), v=V_MED, tol=0.004, integ=True),
            self.g_move(A, lambda: hand_for(slot + 0.160 * up), v=V_SLOW, tol=0.0012, rtol=0.03, settle=4, maxn=150, integ=True),
            self.g_move(A, lambda: hand_for(slot + 0.045 * up), v=V_SLOW, tol=0.002, integ=True),
            self.g_grip(A, -1.0, 8)))
        pA, _ = self.tool(A)
        yield from self.run(self.g_move(A, (pA + 0.05 * up, Ry), v=V_MED, tol=0.006))
        yield from self.close_lever(sp, A)

    def close_lever(self, sp, A):
        """Swing caddy sp's lever down along its arc, then press the plate (and the caddy) home until it latches."""
        up = np.array([0, 0, 1.0])
        Ry = self.rack_R_hand(R_DY)
        self.grip[A] = 1.0
        hinge = lambda: self.caddy_point(sp, (-0.024, 0.0, 0.0))

        def arc(th):
            pc, Rc = self.caddy(sp)
            d = np.array([math.cos(th), 0.0, math.sin(th)])
            nrm = np.array([-math.sin(th), 0.0, math.cos(th)])
            return hinge() + Rc @ (0.018 * d + 0.013 * nrm + np.array([0, -0.012, 0]))

        yield from self.run(self.g_move(A, lambda: (arc(math.radians(100)) + 0.03 * up, Ry), v=V_MED, tol=0.004))
        for deg in range(100, 15, -6):
            yield from self.run(self.g_move(A, lambda d_=deg: (arc(math.radians(d_)), Ry), v=V_SLOW, tol=0.004,
                                            maxn=30, settle=1))
        press = lambda: self.caddy_point(sp, (-0.008, -0.017, 0.0))
        yield from self.run(self.g_hold_pose(
            A, lambda: (press() - 0.012 * up, Ry), lambda: self.T["caddies"][sp]["latched"], maxn=500, v=V_SLOW))
        yield from self.run(self.g_wait(A, 3))
        pA, _ = self.tool(A)
        yield from self.run(self.seq(
            self.g_move(A, (pA + 0.06 * up, Ry), v=V_MED, tol=0.006),
            self.g_move(A, self.park(A), v=V_FAST, tol=0.03)))

    def swap_fan(self, i):
        rp, rR = self.rack()
        up = np.array([0, 0, 1.0])
        Rx = self.rack_R_hand(R_DX)
        Ry = self.rack_R_hand(R_DY)
        A, B = "l", "r"
        bay = np.array(self.T["fan_frames"][i])
        handle_z = 0.050 + 0.016 - 0.001          # just above the bar centre: the fingertips stay out of the loop
        old = "f%d" % i
        g_sp = lambda: self.fan("fs")[0] + np.array([0, 0, 0.040])
        yield from self.run(self.seq(
            self.g_grip(A, -1.0, 2),
            self.g_move(A, lambda: (g_sp() + 0.08 * up, R_DX), v=V_FAST, tol=0.008),
            self.g_move(A, lambda: (g_sp(), R_DX), v=V_MED, tol=0.002),
            self.g_grip(A, 1.0, 10)))
        lift = g_sp() + 0.10 * up
        yield from self.run(self.g_move(A, (lift, R_DX), v=V_MED, tol=0.01, integ=True))

        foff = self.tool(A)[0] - self.fan("fs")[0]

        def hand_for_fan(q):
            return q + foff, Rx

        hover = bay + rR[:, 1] * 0.17 + 0.14 * up
        yield from self.run(self.g_move(A, lambda: hand_for_fan(np.array([0.45, 0.25, hover[2]])), v=V_MED, tol=0.02,
                                        integ=True))
        h_old = lambda: self.fan(old)[0] + np.array([0, 0, handle_z])
        yield from self.run(
            self.g_move(A, lambda: hand_for_fan(hover), v=V_MED, tol=0.01, integ=True),
            self.seq(self.g_grip(B, -1.0, 2),
                     self.g_move(B, lambda: (h_old() + 0.10 * up, Rx), v=V_FAST, tol=0.006),
                     self.g_move(B, lambda: (h_old() + 0.03 * up, Rx), v=V_MED, tol=0.003),
                     self.g_move(B, lambda: (h_old(), Rx), v=V_SLOW, tol=0.002)))
        yield from self.run(self.g_grip(B, 1.0, 10))
        pB, _ = self.tool(B)
        yield from self.run(self.seq(
            self.g_move(B, (pB + 0.075 * up, Rx), v=V_MED, tol=0.006, integ=True, maxn=60),
            self.g_move(B, (pB + 0.12 * up - rR[:, 1] * 0.17 - rR[:, 0] * 0.05, Rx), v=V_FAST, tol=0.02, integ=True,
                        maxn=60)))
        flag = {"a": False}
        yield from self.run(self.seq(
            self.g_move(A, lambda: hand_for_fan(bay + 0.14 * up), v=V_FAST, tol=0.006, integ=True, maxn=60),
            self.g_move(A, lambda: hand_for_fan(bay + 0.05 * up), v=V_FAST, tol=0.004, integ=True, maxn=60),
            self.g_move(A, lambda: hand_for_fan(bay + 0.015 * up), v=V_MED, tol=0.002, integ=True, maxn=80),
            self.g_grip(A, -1.0, 8),
            self.g_move(A, lambda: (self.fan("fs")[0] + (0.050 + 0.016 + 0.040) * up, Ry), v=V_FAST, tol=0.006, maxn=30),
            self.g_grip(A, 1.0, 4),
            self.g_hold_pose(A, lambda: (self.fan("fs")[0] + (0.050 + 0.016 + 0.004) * up, Ry),
                             lambda: self.T["fans"]["fs"]["seated"], maxn=200, v=V_SLOW),
            self.g_move(A, lambda: (self.fan("fs")[0] + 0.20 * up, Ry), v=V_MED, tol=0.01, maxn=40),
            self.g_move(A, self.park(A), v=V_FAST, tol=0.03)),
            self.seq(
            self.g_move(B, (np.array([0.24, -0.12, 0.48]), R_DY), v=V_MED, tol=0.015, integ=True, maxn=150),
            self.g_move(B, (np.array([0.21, -0.11, 0.31]), R_DY), v=V_MED, tol=0.015, integ=True, maxn=120),
            self.g_grip(B, -1.0, 10),
            self.g_move(B, (np.array([0.21, -0.11, 0.42]), R_DY), v=V_MED, tol=0.015, maxn=60),
            self.g_move(B, (np.array([0.22, -0.20, 0.50]), R_DY), v=V_FAST, tol=0.02),
            self.g_move(B, self.park(B), v=V_FAST, tol=0.03)))
