"""ik: the grader's inverse kinematics on a hand model (grader only; deterministic, no random generator, no clock).

    res = solve_targets(hm, {"thumb": p_t, "index": p_f}, scramble=s, cfg=cfg)
    res.ok, res.q (the involved digits' driven angles, rad), res.residual_mm (per target pad)

Damped least squares (Levenberg-Marquardt) on the driven hinges of the involved digits, clamped to the model's own
ranges after every step, through kin.HandModel (its pad Jacobian includes the coupling of the last hinge). The other
digits stay at q = 0 (their pads do not depend on them).
Restarts: cfg["ik"]["restarts"] (24): restart 0 starts at mid-range, restarts 1 .. 23 at Halton points over the ranges
shrunk by cfg["ik"]["shrink_deg"] at both ends, shifted modulo 1 by fractions derived from `scramble` (a request's
content digest, so the same request gets the same restarts whatever its id). A restart runs at most
cfg["ik"]["max_iters"] steps and ends early when every target pad is within cfg["ik"]["tol_m"] (the first restart that
gets there is the solution), when the damping exceeds LAM_MAX, or after STALL_STEPS accepted steps in a row that each
lower the squared error by less than STALL_REL of it. Otherwise the restart with the smallest residual (the largest of its pads' distances to
their targets) is, ties going to the lower restart index. A configuration at which a closed chain of the model does
not close is a failed step: the restart stops there and keeps its last closed configuration.
ok = the solution's residual is at most cfg["zero_rules"]["ik_accept_mm"] on every target pad.
A two-pad target (a pinch) adds the row normal_weight x (1 + n_thumb . n_finger) (the pads' outward normals), so
that of the configurations that place both pads the one with the pads facing each other is preferred; acceptance
uses the positions only.
"""
import math

import numpy as np

try:
    from . import contract as C
    from . import kin as K
except ImportError:  # loaded as plain modules beside each other
    import contract as C
    import kin as K

LAM0 = 1e-2              # initial damping (the Jacobian is in m / rad, a few cm per rad)
LAM_MIN = 1e-7
LAM_MAX = 1e1
LAM_DOWN = 0.5
LAM_UP = 4.0
STEP_MIN_RAD = 1e-12     # a restart ends when its steps are this small
FD_STEP_RAD = 1e-6       # forward difference of the normal row
STALL_REL = 1e-9         # an accepted step that lowers the squared error by less than this share is a stalled step
STALL_STEPS = 5          # a restart ends after this many stalled steps in a row


class IKResult(object):
    __slots__ = ("q", "residual_mm", "ok", "restart", "iterations", "loop_failures")

    def __init__(self, q, residual_mm, ok, restart, iterations, loop_failures):
        self.q = q
        self.residual_mm = residual_mm
        self.ok = ok
        self.restart = restart
        self.iterations = iterations
        self.loop_failures = loop_failures


def _primes(k):
    out, n = [], 2
    while len(out) < k:
        if all(n % p for p in out if p * p <= n):
            out.append(n)
        n += 1
    return out


def halton(n, dims, shift=None, skip=20):
    """n Halton points in [0, 1)^dims (bases = the first dims primes; point i = radical inverse of i + 1 + skip),
    shifted modulo 1 by `shift` when given."""
    bases = _primes(dims)
    out = np.zeros((n, dims))
    for d, p in enumerate(bases):
        for i in range(n):
            k, f, r = i + 1 + skip, 1.0, 0.0
            while k > 0:
                f /= p
                r += f * (k % p)
                k //= p
            out[i, d] = r
    if shift is not None:
        out = (out + np.asarray(shift, float)[None, :dims]) % 1.0
    return out


def scramble_shift(scramble, dims):
    """dims fractions in [0, 1) from a 64-bit integer: the radical inverse of scramble in base 2, 3, 5, ..."""
    s = int(scramble) & ((1 << 64) - 1)
    out = []
    for p in _primes(dims):
        k, f, r = s + 1, 1.0, 0.0
        while k > 0:
            f /= p
            r += f * (k % p)
            k //= p
        out.append(r % 1.0)
    return np.array(out)


def _joints_of(digits):
    return [j for d in C.DIGITS if d in digits for j in C.DRIVEN_BY_DIGIT[d]]


def starts(lo, hi, scramble, n_restarts, shrink_rad):
    """The restart configurations (n_restarts x n)."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    n = len(lo)
    mid = 0.5 * (lo + hi)
    out = [mid]
    if n_restarts > 1:
        lo2, hi2 = lo + shrink_rad, hi - shrink_rad
        narrow = hi2 <= lo2
        lo2 = np.where(narrow, mid, lo2)
        hi2 = np.where(narrow, mid, hi2)
        H = halton(n_restarts - 1, n, shift=scramble_shift(scramble, n))
        for h in H:
            out.append(lo2 + h * (hi2 - lo2))
    return np.array(out)


class _Problem(object):
    def __init__(self, hm, targets, normal_weight):
        self.hm = hm
        self.digits = [d for d in C.DIGITS if d in targets]
        self.targets = {d: np.asarray(targets[d], float).reshape(3) for d in self.digits}
        self.joints = _joints_of(self.digits)
        rg = hm.ranges()
        self.lo = np.array([rg[j][0] for j in self.joints])
        self.hi = np.array([rg[j][1] for j in self.joints])
        self.cols = {d: [self.joints.index(j) for j in C.DRIVEN_BY_DIGIT[d]] for d in self.digits}
        self.pinch = len(self.digits) == 2 and "thumb" in self.digits and normal_weight > 0.0
        self.w = float(normal_weight)

    def clip(self, q):
        return np.minimum(np.maximum(q, self.lo), self.hi)

    def state(self, q):
        """(KinState, pads {digit: (p, n)}); raises kin.LoopError."""
        st = self.hm.solve({j: float(x) for j, x in zip(self.joints, q)})
        return st, {d: self.hm.pad(st, d) for d in self.digits}

    def residual(self, pads):
        r = [self.targets[d] - pads[d][0] for d in self.digits]
        if self.pinch:
            n_t, n_f = pads[self.digits[0]][1], pads[self.digits[1]][1]
            r.append(np.array([-self.w * (1.0 + float(n_t @ n_f))]))
        return np.concatenate(r)

    def pad_errors(self, pads):
        return {d: float(np.linalg.norm(self.targets[d] - pads[d][0])) for d in self.digits}

    def jacobian(self, q, st, pads):
        n = len(self.joints)
        rows = 3 * len(self.digits) + (1 if self.pinch else 0)
        J = np.zeros((rows, n))
        for k, d in enumerate(self.digits):
            J[3 * k:3 * k + 3, self.cols[d]] = self.hm.pad_jacobian(st, d)
        if self.pinch:
            J[-1, :] = self.w * self.normal_gradient(q, st, pads)
        return J

    def normal_gradient(self, q, st, pads):
        """d(n_thumb . n_finger) / dq. A hinge that turns the pad's body rigidly (every thumb hinge; the finger's first
        two when its last hinge is coupled by a joint equality) changes the pad normal by axis x n; the other columns
        (a closed chain moves passive hinges) by a forward difference of the kinematics."""
        hm = self.hm
        thumb, finger = self.digits
        n_t, n_f = pads[thumb][1], pads[finger][1]
        R0 = st.xmat[hm.palm].reshape(3, 3)
        rigid = set(C.DRIVEN_BY_DIGIT[thumb])
        if hm.coupling_form(finger) == "joint":
            rigid |= {C.CHAINS[finger][0], C.CHAINS[finger][1]}
        f0 = float(n_t @ n_f)
        out = np.zeros(len(self.joints))
        for c, j in enumerate(self.joints):
            if j in rigid:
                w = R0.T @ st.xaxis[hm.jid[j]]
                if C.DIGIT_OF_JOINT[j] == thumb:
                    out[c] = float(np.cross(w, n_t) @ n_f)
                else:
                    out[c] = float(n_t @ np.cross(w, n_f))
                continue
            qp = q.copy()
            h = FD_STEP_RAD if qp[c] + FD_STEP_RAD <= self.hi[c] else -FD_STEP_RAD
            qp[c] += h
            _, pp = self.state(qp)
            out[c] = (float(pp[thumb][1] @ pp[finger][1]) - f0) / h
        return out


def _run_restart(pb, q0, max_iters, tol_m):
    """(q, pad errors, iterations, loop failure) of one restart; q None when its start does not close."""
    try:
        st, pads = pb.state(q0)
    except K.LoopError:
        return None, None, 0, True
    q = q0.copy()
    e = pb.residual(pads)
    errs = pb.pad_errors(pads)
    lam = LAM0
    its = 0
    stall = 0
    loop_fail = False
    while its < max_iters:
        if max(errs.values()) <= tol_m:
            break
        its += 1
        try:
            J = pb.jacobian(q, st, pads)
        except K.LoopError:
            loop_fail = True
            break
        H = J.T @ J
        g = J.T @ e
        try:
            dq = np.linalg.solve(H + (lam * lam) * np.eye(len(q)), g)
        except np.linalg.LinAlgError:
            lam = lam * LAM_UP
            if lam > LAM_MAX:
                break
            continue
        q_new = pb.clip(q + dq)
        if float(np.max(np.abs(q_new - q))) <= STEP_MIN_RAD:
            break
        try:
            st_new, pads_new = pb.state(q_new)
        except K.LoopError:
            loop_fail = True
            break
        e_new = pb.residual(pads_new)
        if float(e_new @ e_new) < float(e @ e):
            gain = float(e @ e) - float(e_new @ e_new)
            stall = stall + 1 if gain <= STALL_REL * float(e @ e) else 0
            q, st, pads, e = q_new, st_new, pads_new, e_new
            errs = pb.pad_errors(pads)
            lam = max(lam * LAM_DOWN, LAM_MIN)
            if stall >= STALL_STEPS:
                break
        else:
            lam = lam * LAM_UP
            if lam > LAM_MAX:
                break
    return q, errs, its, loop_fail


def solve_targets(model, targets, *, scramble, cfg):
    """IK for pad targets {digit: point (palm frame, m)} on a kin.HandModel; see the module docstring."""
    ikc = cfg["ik"]
    pb = _Problem(model, targets, float(ikc.get("normal_weight", 0.0)))
    tol = float(ikc["tol_m"])
    accept = float(cfg["zero_rules"]["ik_accept_mm"]) / 1000.0
    S = starts(pb.lo, pb.hi, scramble, int(ikc["restarts"]), math.radians(float(ikc["shrink_deg"])))
    best = None              # (residual, restart, q, errs)
    total_its, loop_failures = 0, 0
    for r, q0 in enumerate(S):
        q, errs, its, lf = _run_restart(pb, q0, int(ikc["max_iters"]), tol)
        total_its += its
        loop_failures += 1 if lf else 0
        if q is None:
            continue
        res = max(errs.values())
        if best is None or res < best[0]:
            best = (res, r, q, errs)
        if res <= tol:
            break
    if best is None:
        return IKResult({}, {d: float("inf") for d in pb.digits}, False, -1, total_its, loop_failures)
    res, r, q, errs = best
    qd = {j: float(x) for j, x in zip(pb.joints, q)}
    ok = all(v <= accept for v in errs.values())
    return IKResult(qd, {d: 1000.0 * v for d, v in errs.items()}, ok, r, total_its, loop_failures)
