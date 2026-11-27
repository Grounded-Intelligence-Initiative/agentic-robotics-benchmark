"""The exact two-rod ankle and its true stop region, vendored for the grader's true side.

    ch = Chain("left", (222.0, 152.0))   # bars centre to centre, mm
    phi = ch.ik(p, r, ref)                # crank angles (phiA, phiB) rad, the root nearest ref; None outside reach
    p, r = ch.fk(phiA, phiB, guess)       # foot pitch, roll (rad, the model's hinge angles); ValueError outside reach
    S = stop_region("left")               # the true stop region S (an octagon in (pitch, roll))
    S.excess_deg(p, r)                    # > 0 outside S (deg), <= 0 inside
    S.nearest(p, r)                       # the nearest point of S (rad; Euclidean in the (pitch, roll) plane)

The arithmetic (_rot, _solve_leg, _closure_and_gt, Chain.fk, fw_pitch_roll, Octagon.excess_deg) is copied from the
ankle model that generated the unit's records, expression by expression (scalar IEEE double and libm only), so that
the grader's true side equals that chain bit for bit; the data (geometry, stop region, map constants) are its values,
copied unchanged into data/ankle_vendor.json. Octagon.nearest is the grader's own (a true foot attitude outside S is
moved to S's nearest point, Euclidean in rad).

Conventions: foot angles (pitch, roll) are the model file's foot hinge angles (left pitch axis +y, right -y, the roll
axis -x on both, riding on the pitch link); crank angles phi_k are right-hand rotations about the actuator's output
axis (+x), 0 = the CAD crank pose; bars L_k = home + exposed thread (mm).
"""
from __future__ import annotations

import json
import math
import os

SIDES = ("left", "right")
_DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "ankle_vendor.json")
with open(_DATA, encoding="utf-8") as _f:
    VENDOR = json.load(_f)
_C = VENDOR["constants"]
K_PITCH = float(_C["k_pitch"])
K_ROLL = float(_C["k_roll"])
PITCH_LIMIT = float(_C["pitch_limit_rad"])
ROLL_LIMIT = float(_C["roll_limit_rad"])
BAR_HOME_MM = (float(_C["bar_home_cc_mm"][0]), float(_C["bar_home_cc_mm"][1]))

_PI = math.pi
_TWO_PI = 2.0 * math.pi
_THREE_PI = 3.0 * math.pi
# numerics of the forward map, as the source model's: Newton iterations, convergence test (rad), step limit (rad)
_FK_MAXIT = 60
_FK_TOL = 1e-14
_FK_MAXSTEP = 0.2
_DEAD_GP = 1e-12


# ======================================================================== small vector helpers (tuples of floats)
def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _rot(k, c, s, v):
    """Rodrigues: v rotated about the unit axis k by the angle whose cosine and sine are c, s"""
    kd = k[0] * v[0] + k[1] * v[1] + k[2] * v[2]
    kv0 = k[1] * v[2] - k[2] * v[1]
    kv1 = k[2] * v[0] - k[0] * v[2]
    kv2 = k[0] * v[1] - k[1] * v[0]
    oc = 1.0 - c
    return (v[0] * c + kv0 * s + k[0] * kd * oc, v[1] * c + kv1 * s + k[1] * kd * oc, v[2] * c + kv2 * s + k[2] * kd * oc)


def _unit(v):
    n = math.sqrt(_dot(v, v))
    return (v[0] / n, v[1] / n, v[2] / n)


# ======================================================================== geometry
class _Geom:
    """one leg's fixed geometry (CAD mm)"""

    __slots__ = ("side", "Pp", "ap", "Pr", "ar", "PrPp", "lo0", "lo0Pr", "C", "r", "e1", "e2", "ax")

    def __init__(self, side: str):
        g = VENDOR["geom_cad_mm"][side]
        f = lambda v: tuple(float(x) for x in v)
        self.side = side
        self.Pp = f(g["pitch_point"]); self.ap = _unit(f(g["pitch_dir"]))
        self.Pr = f(g["roll_point"]); self.ar = _unit(f(g["roll_dir"]))
        self.PrPp = _sub(self.Pr, self.Pp)
        self.lo0, self.lo0Pr, self.C, self.r, self.e1, self.e2, self.ax = [], [], [], [], [], [], []
        for k in "AB":
            a = _unit(f(g["act_" + k]["axis_dir"])); p0 = f(g["act_" + k]["axis_point"]); u = f(g["ball_up_" + k])
            du = _sub(u, p0); s = _dot(du, a)
            c = (p0[0] + s * a[0], p0[1] + s * a[1], p0[2] + s * a[2])
            v = _sub(u, c); r = math.sqrt(_dot(v, v))
            e1 = (v[0] / r, v[1] / r, v[2] / r)
            lo = f(g["ball_lo_" + k])
            self.lo0.append(lo); self.lo0Pr.append(_sub(lo, self.Pr)); self.C.append(c); self.r.append(r)
            self.e1.append(e1); self.e2.append(_cross(a, e1)); self.ax.append(a)
        for n in ("lo0", "lo0Pr", "C", "r", "e1", "e2", "ax"):
            setattr(self, n, tuple(getattr(self, n)))


_GEOM = {}


def geom(side: str) -> _Geom:
    if side not in SIDES:
        raise ValueError(f"side {side!r} not in {SIDES}")
    if side not in _GEOM:
        _GEOM[side] = _Geom(side)
    return _GEOM[side]


# ======================================================================== the closed-form IK and the closures
def _solve_leg(g: _Geom, LA: float, LB: float, p: float, r: float, refA: float, refB: float):
    """IK (closed form, the root nearest ref per chain) and the analytic J^-1 at foot pose (p, r):
    (phiA, phiB, ji00, ji01, ji10, ji11, ok); ok False when a chain's acos argument leaves [-1, 1] (NaN included),
    a crank is at its dead point, or J^-1 is not finite"""
    cp = math.cos(p); sp = math.sin(p); cr = math.cos(r); sr = math.sin(r)
    Pp = g.Pp; ap = g.ap; ar = g.ar; t = g.PrPp
    w = _rot(ap, cp, sp, t)
    PrR = (Pp[0] + w[0], Pp[1] + w[1], Pp[2] + w[2])
    arR = _rot(ap, cp, sp, ar)
    ok = True
    out = []
    for k, (L, ref) in enumerate(((LA, refA), (LB, refB))):
        v = _rot(ar, cr, sr, g.lo0Pr[k])
        w = _rot(ap, cp, sp, (t[0] + v[0], t[1] + v[1], t[2] + v[2]))
        lo = (Pp[0] + w[0], Pp[1] + w[1], Pp[2] + w[2])
        Ck = g.C[k]; rr = g.r[k]; e1 = g.e1[k]; e2 = g.e2[k]
        q = (lo[0] - Ck[0], lo[1] - Ck[1], lo[2] - Ck[2])
        A = rr * (e1[0] * q[0] + e1[1] * q[1] + e1[2] * q[2])
        B = rr * (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2])
        kk = (rr * rr + (q[0] * q[0] + q[1] * q[1] + q[2] * q[2]) - L * L) * 0.5
        rho = math.sqrt(A * A + B * B)
        x = kk / rho if rho > 0.0 else 2.0
        if not (-1.0 <= x <= 1.0):
            ok = False
            x = 1.0 if x > 1.0 else -1.0
        base = math.atan2(B, A); dd = math.acos(x)
        c1 = base + dd; c2 = base - dd
        c1 = math.fmod(c1 - ref + _THREE_PI, _TWO_PI) - _PI + ref
        c2 = math.fmod(c2 - ref + _THREE_PI, _TWO_PI) - _PI + ref
        ph = c1 if abs(c1 - ref) < abs(c2 - ref) else c2
        cph = math.cos(ph); sph = math.sin(ph)
        up = (Ck[0] + rr * (cph * e1[0] + sph * e2[0]), Ck[1] + rr * (cph * e1[1] + sph * e2[1]),
              Ck[2] + rr * (cph * e1[2] + sph * e2[2]))
        b = (up[0] - lo[0], up[1] - lo[1], up[2] - lo[2])
        n = math.sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2])
        u = (b[0] / n, b[1] / n, b[2] / n)
        r1 = (lo[0] - Pp[0], lo[1] - Pp[1], lo[2] - Pp[2])
        r2 = (lo[0] - PrR[0], lo[1] - PrR[1], lo[2] - PrR[2])
        dlp = _cross(ap, r1)
        dlr = _cross(arR, r2)
        dup = (rr * (-sph * e1[0] + cph * e2[0]), rr * (-sph * e1[1] + cph * e2[1]), rr * (-sph * e1[2] + cph * e2[2]))
        Gt0 = -(u[0] * dlp[0] + u[1] * dlp[1] + u[2] * dlp[2])
        Gt1 = -(u[0] * dlr[0] + u[1] * dlr[1] + u[2] * dlr[2])
        Gp = u[0] * dup[0] + u[1] * dup[1] + u[2] * dup[2]
        if not (abs(Gp) > _DEAD_GP * rr):
            ok = False
            j0 = 0.0; j1 = 0.0
        else:
            j0 = -Gt0 / Gp; j1 = -Gt1 / Gp
            if not (math.isfinite(j0) and math.isfinite(j1)):
                ok = False
        out.append((ph, j0, j1))
    return out[0][0], out[1][0], out[0][1], out[0][2], out[1][1], out[1][2], ok


def _closure_and_gt(g: _Geom, LA: float, LB: float, phiA: float, phiB: float, p: float, r: float):
    """(gA, gB, Gt00, Gt01, Gt10, Gt11, GpA, GpB): closures |up_k - lo_k| - L_k (mm) and their derivatives with
    respect to (pitch, roll) and to each crank angle"""
    cp = math.cos(p); sp = math.sin(p); cr = math.cos(r); sr = math.sin(r)
    Pp = g.Pp; ap = g.ap; ar = g.ar; t = g.PrPp
    w = _rot(ap, cp, sp, t)
    PrR = (Pp[0] + w[0], Pp[1] + w[1], Pp[2] + w[2])
    arR = _rot(ap, cp, sp, ar)
    res = []
    for k, (L, ph) in enumerate(((LA, phiA), (LB, phiB))):
        v = _rot(ar, cr, sr, g.lo0Pr[k])
        w = _rot(ap, cp, sp, (t[0] + v[0], t[1] + v[1], t[2] + v[2]))
        lo = (Pp[0] + w[0], Pp[1] + w[1], Pp[2] + w[2])
        Ck = g.C[k]; rr = g.r[k]; e1 = g.e1[k]; e2 = g.e2[k]
        cph = math.cos(ph); sph = math.sin(ph)
        up = (Ck[0] + rr * (cph * e1[0] + sph * e2[0]), Ck[1] + rr * (cph * e1[1] + sph * e2[1]),
              Ck[2] + rr * (cph * e1[2] + sph * e2[2]))
        b = _sub(up, lo); n = math.sqrt(_dot(b, b)); u = (b[0] / n, b[1] / n, b[2] / n)
        dlp = _cross(ap, _sub(lo, Pp)); dlr = _cross(arR, _sub(lo, PrR))
        dup = (rr * (-sph * e1[0] + cph * e2[0]), rr * (-sph * e1[1] + cph * e2[1]), rr * (-sph * e1[2] + cph * e2[2]))
        res.append((n - L, -_dot(u, dlp), -_dot(u, dlr), _dot(u, dup)))
    (gA, a0, a1, pA), (gB, b0, b1, pB) = res
    return gA, gB, a0, a1, b0, b1, pA, pB


# ======================================================================== Chain
class Chain:
    """One leg's exact two-rod ankle with bar lengths L = (L_A, L_B) in mm centre to centre"""

    __slots__ = ("side", "L", "g")

    def __init__(self, side: str, L: tuple):
        self.side = side
        self.g = geom(side)
        LA, LB = (float(L[0]), float(L[1]))
        if not (LA > 0.0 and LB > 0.0):
            raise ValueError(f"bar lengths {L} mm")
        self.L = (LA, LB)

    def ik(self, p: float, r: float, ref=(0.0, 0.0)):
        """crank angles (phiA, phiB) for foot pitch p, roll r (rad), the root nearest ref; None outside reach"""
        a, b, _, _, _, _, ok = _solve_leg(self.g, self.L[0], self.L[1], float(p), float(r), float(ref[0]), float(ref[1]))
        return (a, b) if ok else None

    def ik_jinv(self, p: float, r: float, ref=(0.0, 0.0)):
        a, b, j00, j01, j10, j11, ok = _solve_leg(self.g, self.L[0], self.L[1], float(p), float(r), float(ref[0]), float(ref[1]))
        return (a, b), ((j00, j01), (j10, j11)), ok

    def closure(self, phiA: float, phiB: float, p: float, r: float):
        gA, gB = _closure_and_gt(self.g, self.L[0], self.L[1], phiA, phiB, p, r)[:2]
        return (gA, gB)

    def fk(self, phiA: float, phiB: float, guess=(0.0, 0.0)):
        """foot (pitch, roll) rad at crank angles (phiA, phiB): Newton on the closures from guess, the solution nearest
        the guess. Raises ValueError when Newton does not converge (crank angles outside the chain's reach)."""
        p, r = float(guess[0]), float(guess[1])
        phiA = float(phiA); phiB = float(phiB)
        LA, LB = self.L
        for _ in range(_FK_MAXIT):
            gA, gB, a0, a1, b0, b1, _, _ = _closure_and_gt(self.g, LA, LB, phiA, phiB, p, r)
            det = a0 * b1 - a1 * b0
            if not (det != 0.0 and math.isfinite(det)):
                break
            dp = -(b1 * gA - a1 * gB) / det
            dr = -(-b0 * gA + a0 * gB) / det
            m = max(abs(dp), abs(dr))
            if not math.isfinite(m):
                break
            if m > _FK_MAXSTEP:
                dp *= _FK_MAXSTEP / m; dr *= _FK_MAXSTEP / m
            p += dp; r += dr
            if m < _FK_TOL:
                return (p, r)
        gA, gB = self.closure(phiA, phiB, p, r)
        if math.isfinite(gA) and math.isfinite(gB) and abs(gA) < 1e-9 and abs(gB) < 1e-9:
            return (p, r)
        raise ValueError(f"{self.side} ankle fk did not converge at phi = ({phiA}, {phiB}) from guess {guess}")

    def jinv(self, p: float, r: float, phi):
        _, _, a0, a1, b0, b1, pA, pB = _closure_and_gt(self.g, self.L[0], self.L[1], float(phi[0]), float(phi[1]), float(p), float(r))
        return ((-a0 / pA, -a1 / pA), (-b0 / pB, -b1 / pB))

    def j(self, p: float, r: float, phi):
        (a, b), (c, d) = self.jinv(p, r, phi)
        det = a * d - b * c
        return ((d / det, -b / det), (-c / det, a / det))

    def __repr__(self):
        return f"Chain({self.side!r}, ({self.L[0]!r}, {self.L[1]!r}))"


# ======================================================================== the firmware's linear map
def linear_map():
    """K = d(A, B)/d(pitch, roll) of the firmware's map A = k_p p - k_r r, B = -k_p p - k_r r"""
    return ((K_PITCH, -K_ROLL), (-K_PITCH, -K_ROLL))


def fw_pitch_roll(A: float, B: float):
    """the firmware's pitch and roll from the motor coordinates: ((A - B) / (2 k_p), -(A + B) / (2 k_r))"""
    return ((A - B) / (2.0 * K_PITCH), -(A + B) / (2.0 * K_ROLL))


def fw_motors(pitch: float, roll: float):
    """the firmware's motor coordinates of a foot attitude: (k_p p - k_r r, -k_p p - k_r r)"""
    return (K_PITCH * pitch - K_ROLL * roll, -K_PITCH * pitch - K_ROLL * roll)


# ======================================================================== the true stop region S
class Octagon:
    """The ankle's true stop region S of one leg in the model's foot hinge angles: pmin <= pitch <= pmax,
    |roll| <= rmax, the corners cut by lines from (|pitch| = cut_p, |roll| = rmax) to (the pitch edge, |roll| = cut_r)
    (deg)"""

    __slots__ = ("side", "pmin", "pmax", "rmax", "cut_p", "cut_r", "lines", "_vr")

    def __init__(self, side: str, pmin: float, pmax: float, rmax: float, cut_p: float, cut_r: float):
        self.side = side
        self.pmin, self.pmax, self.rmax, self.cut_p, self.cut_r = (float(x) for x in (pmin, pmax, rmax, cut_p, cut_r))
        lines = []
        for pe in (self.pmin, self.pmax):
            s = 1.0 if pe > 0 else -1.0
            k = (self.rmax - self.cut_r) / (abs(pe) - self.cut_p)
            c = self.rmax + k * self.cut_p
            for sr in (1.0, -1.0):
                lines.append((s * k, sr, c))
        self.lines = tuple(lines)
        self._vr = tuple((math.radians(p), math.radians(r)) for p, r in self.vertices_deg())

    def excess_deg(self, p: float, r: float) -> float:
        """the largest violation of S at foot pitch p, roll r (rad), in deg: > 0 outside, <= 0 inside"""
        pd = math.degrees(p); rd = math.degrees(r)
        v = max(self.pmin - pd, pd - self.pmax, abs(rd) - self.rmax)
        for a, b, c in self.lines:
            v = max(v, (a * pd + b * rd - c) / math.hypot(a, b))
        return v

    def contains(self, p: float, r: float) -> bool:
        return self.excess_deg(p, r) <= 0.0

    def vertices_deg(self):
        """the eight corners (pitch, roll) deg, counter-clockwise from (pmax, -cut_r)"""
        return ((self.pmax, -self.cut_r), (self.pmax, self.cut_r), (self.cut_p, self.rmax), (-self.cut_p, self.rmax),
                (self.pmin, self.cut_r), (self.pmin, -self.cut_r), (-self.cut_p, -self.rmax), (self.cut_p, -self.rmax))

    def nearest(self, p: float, r: float):
        """(p, r) itself when inside S, else the nearest point of S's boundary (rad, Euclidean in (pitch, roll));
        the first of equally near points in the vertices' order"""
        if self.contains(p, r):
            return (float(p), float(r))
        best = None
        V = self._vr
        for i in range(8):
            (x0, y0), (x1, y1) = V[i], V[(i + 1) % 8]
            dx, dy = x1 - x0, y1 - y0
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 <= 0.0 else ((p - x0) * dx + (r - y0) * dy) / L2
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            qx, qy = x0 + t * dx, y0 + t * dy
            d2 = (p - qx) * (p - qx) + (r - qy) * (r - qy)
            if best is None or d2 < best[0]:
                best = (d2, qx, qy)
        return (best[1], best[2])


_OCT = {}


def stop_region(side: str) -> Octagon:
    if side not in SIDES:
        raise ValueError(f"side {side!r} not in {SIDES}")
    if side not in _OCT:
        o = VENDOR["stop_region_deg"][side]
        _OCT[side] = Octagon(side, o["pmin"], o["pmax"], o["rmax"], o["cut_p"], o["cut_r"])
    return _OCT[side]
