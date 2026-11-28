"""score: one request or one reading scored against the photographed unit (grader only; deterministic).

    r = score_request(hm, calib, unit, request, cfg)     # request = a record of hidden_set.json plus its "group"
    o = score_reading(hm, calib, unit, k, cfg)           # k = 13 integer counts of a hidden reading

A request (pinch, single, impossible) goes the partner's way: the grader's IK on the SUBMITTED model (inside its own
ranges) gives the involved digits' driven angles; the submitted calibration turns them into goal counts (the host's
formula, calib.model_to_goal); the unit executes them after its power-up in the packed posture (hidden_unit.execute),
the other servos holding the packed posture; the pads come from the unit's own kinematics at the executed pose.
  zero reasons (score 0): "ik" (the IK's residual above ik_accept_mm on a target pad), "hard_stop" (a commanded servo
  more than hard_stop_deg beyond a true end), "interpen" (two parts of the unit pressed more than interpen_mm into
  each other at the executed pose).
  pinch   L(e_close; close) x L(e_mid; mid): e_close = |(p_thumb - p_finger) - g u|, e_mid = |(p_thumb + p_finger)/2 - m|
  single  L(e_pad; pad): e_pad = |p - target|
  impossible  1.0 when the IK on the model fails (the model rejects it), 0.0 when it is accepted.
A reading is evaluated on the submitted model at the angles its calibration gives (calib.reading_to_model) and on the
unit at its true angles (hidden_unit.angles_at_counts); per digit o = 0 when the calibration maps a servo of the digit
more than out_of_range_deg outside the model's range or when the digit's closed chain does not close there.
L(e; full, zero) = clamp(ln(zero / e) / ln(zero / full), 0, 1), and 1 for e <= full.
"""
import hashlib
import json
import math

import numpy as np

try:
    from . import contract as C
    from . import calib as CAL
    from . import collide as COL
    from . import ik as IK
    from . import kin as K
except ImportError:  # loaded as plain modules beside each other
    import contract as C
    import calib as CAL
    import collide as COL
    import ik as IK
    import kin as K

FEASIBLE = ("pinch", "single")
GROUPS = ("pinch", "single", "impossible")
JACOBIAN_MIN_M = 1e-9           # a driven hinge whose pad Jacobian along the normal is below this does not limit the force


def L(e, full, zero):
    """Log band: 1 for e <= full, 0 for e >= zero, ln(zero / e) / ln(zero / full) between; 0 for a non-finite e."""
    e = float(e)
    if not math.isfinite(e):
        return 0.0
    if e <= full:
        return 1.0
    if e >= zero:
        return 0.0
    return float(math.log(zero / e) / math.log(zero / full))


def band(cfg, name, e):
    b = cfg["bands"][name]
    return L(e, float(b["full"]), float(b["zero"]))


def request_content(request):
    """The request without its id: the content the IK's restarts are derived from."""
    g = request["group"]
    if g == "pinch":
        return {"group": g, "finger": request["finger"], "m": request["m"], "u": request["u"], "g": request["g"]}
    return {"group": g, "digit": request["digit"], "p": request["p"]}


def scramble_of(request):
    s = json.dumps(request_content(request), sort_keys=True, separators=(",", ":")).encode()
    return int.from_bytes(hashlib.sha256(s).digest()[:8], "big")


def targets_of(request):
    if request["group"] == "pinch":
        m = np.asarray(request["m"], float)
        u = np.asarray(request["u"], float)
        g = float(request["g"])
        return {"thumb": m + 0.5 * g * u, request["finger"]: m - 0.5 * g * u}
    return {request["digit"]: np.asarray(request["p"], float)}


class RequestResult(object):
    __slots__ = ("score", "zero_reasons", "ik_ok", "residual_mm", "errors_mm", "loop_failures", "max_penetration_mm",
                 "overshoot_deg")

    def __init__(self, score, zero_reasons, ik_ok, residual_mm, errors_mm, loop_failures=0, max_penetration_mm=0.0,
                 overshoot_deg=0.0):
        self.score = score
        self.zero_reasons = zero_reasons
        self.ik_ok = ik_ok
        self.residual_mm = residual_mm
        self.errors_mm = errors_mm
        self.loop_failures = loop_failures
        self.max_penetration_mm = max_penetration_mm
        self.overshoot_deg = overshoot_deg


def score_request(model, calib, unit, request, cfg):
    """Score one request on a submitted model (kin.HandModel) and calibration (calib.Calib)."""
    group = request["group"]
    if group not in GROUPS:
        raise ValueError("unknown request group %r" % (group,))
    zr = cfg["zero_rules"]
    targets = targets_of(request)
    res = IK.solve_targets(model, targets, scramble=scramble_of(request), cfg=cfg)
    residual = max(res.residual_mm.values()) if res.residual_mm else float("inf")
    if group == "impossible":
        return RequestResult(0.0 if res.ok else 1.0, (), res.ok, residual, {}, res.loop_failures)
    if not res.ok:
        return RequestResult(0.0, ("ik",), False, residual, {}, res.loop_failures)
    goal = CAL.model_to_goal(calib, {C.MOTOR_ID[j]: v for j, v in res.q.items()})
    ex = unit.execute(goal)
    st = unit.solve(ex.q_exec)
    reasons = []
    over = max(ex.overshoot_deg.values()) if ex.overshoot_deg else 0.0
    if over > float(zr["hard_stop_deg"]):
        reasons.append("hard_stop")
    pen = COL.max_penetration_mm(unit, st)
    if pen > float(zr["interpen_mm"]):
        reasons.append("interpen")
    if group == "pinch":
        f = request["finger"]
        p_t = unit.truth.pad(st, "thumb")[0]
        p_f = unit.truth.pad(st, f)[0]
        u = np.asarray(request["u"], float)
        m = np.asarray(request["m"], float)
        e_close = 1000.0 * float(np.linalg.norm((p_t - p_f) - float(request["g"]) * u))
        e_mid = 1000.0 * float(np.linalg.norm(0.5 * (p_t + p_f) - m))
        errs = {"close": e_close, "mid": e_mid}
        value = band(cfg, "close", e_close) * band(cfg, "mid", e_mid)
    else:
        d = request["digit"]
        p = unit.truth.pad(st, d)[0]
        e_pad = 1000.0 * float(np.linalg.norm(p - np.asarray(request["p"], float)))
        errs = {"pad": e_pad}
        value = band(cfg, "pad", e_pad)
    z = 0.0 if reasons else 1.0
    return RequestResult(z * value, tuple(reasons), True, residual, errs, res.loop_failures, pen, over)


# ------------------------------------------------------------------------------------------------ readings
def pad_force(hm, st, digit, forces):
    """The largest force (N) the digit's pad holds along its outward normal: min over its driven hinges i with
    |g_i| > JACOBIAN_MIN_M of F_i / |g_i|, g_i = J[:, i] . n (J the pad Jacobian, coupling included)."""
    J = hm.pad_jacobian(st, digit)
    n = hm.pad(st, digit)[1]
    g = n @ J
    vals = [float(forces[j]) / abs(float(g[c])) for c, j in enumerate(C.DRIVEN_BY_DIGIT[digit])
            if abs(float(g[c])) > JACOBIAN_MIN_M]
    return min(vals) if vals else float("inf")


class ReadingResult(object):
    __slots__ = ("o", "pos_err_mm", "nrm_err_deg", "force", "force_true", "load", "load_true", "out_of_range",
                 "loop_fail", "turn_miss", "o_hand")

    def __init__(self):
        self.o, self.pos_err_mm, self.nrm_err_deg = {}, {}, {}
        self.force, self.force_true, self.load, self.load_true = {}, {}, {}, {}
        self.out_of_range, self.loop_fail = {}, {}
        self.turn_miss = False
        self.o_hand = 0


def _angle_deg(a, b):
    c = float(np.dot(a, b) / max(np.linalg.norm(a) * np.linalg.norm(b), 1e-300))
    return math.degrees(math.acos(min(1.0, max(-1.0, c))))


def _solve_digits(hm, q, skip=()):
    """{digit: KinState | None}: the model at the angles q (dict over the 13 driven hinges), digit by digit where a
    closed chain fails (a digit's pad depends on its own hinges only). The digits in `skip` are not evaluated (they
    stay at q = 0 and get None)."""
    use = [d for d in C.DIGITS if d not in skip]
    try:
        st = hm.solve({j: q[j] for d in use for j in C.DRIVEN_BY_DIGIT[d]})
        out = {d: st for d in use}
    except K.LoopError:
        out = {}
        for d in use:
            try:
                out[d] = hm.solve({j: q[j] for j in C.DRIVEN_BY_DIGIT[d]})
            except K.LoopError:
                out[d] = None
    out.update({d: None for d in skip})
    return out


def score_reading(model, calib, unit, k, cfg, what=("pose", "force", "load")):
    """Evaluate one reading (13 counts in the partner's frame) on the model and on the unit."""
    zr = cfg["zero_rules"]
    k = np.asarray(k, dtype=np.int64)
    q_sub = CAL.reading_to_model(calib, k)
    q_hid = unit.angles_at_counts(k)
    rg = model.ranges()
    oor_rad = math.radians(float(zr["out_of_range_deg"]))
    out = ReadingResult()
    bad = {}
    for i, j in enumerate(C.MOTOR_ORDER):
        lo, hi = rg[j]
        x = float(q_sub[i])
        beyond = max(lo - x, x - hi, 0.0)
        if not math.isfinite(x) or beyond > oor_rad:
            bad[j] = True
        if not math.isfinite(x) or beyond > math.pi:
            out.turn_miss = True
    qs = {j: float(q_sub[i]) for i, j in enumerate(C.MOTOR_ORDER)}
    qh = {j: float(q_hid[i]) for i, j in enumerate(C.MOTOR_ORDER)}
    oor_digits = [d for d in C.DIGITS if any(bad.get(j, False) for j in C.DRIVEN_BY_DIGIT[d])]
    sts = _solve_digits(model, qs, skip=oor_digits)
    sth = unit.solve(qh)
    forces_sub = model.force_ranges()
    forces_true = unit.truth.force_ranges()
    for d in C.DIGITS:
        oor = d in oor_digits
        lf = sts[d] is None and not oor
        out.out_of_range[d] = oor
        out.loop_fail[d] = lf
        out.o[d] = 0 if (oor or lf) else 1
        if sts[d] is None:
            continue
        if "pose" in what:
            p_s, n_s = model.pad(sts[d], d)
            p_h, n_h = unit.truth.pad(sth, d)
            out.pos_err_mm[d] = 1000.0 * float(np.linalg.norm(p_s - p_h))
            out.nrm_err_deg[d] = _angle_deg(n_s, n_h)
        if "force" in what:
            out.force[d] = pad_force(model, sts[d], d, forces_sub)
            out.force_true[d] = pad_force(unit.truth, sth, d, forces_true)
    out.o_hand = 1 if all(out.o[d] for d in C.DIGITS) else 0
    if "load" in what:
        for g in C.DIGITS + ("hand",):
            if g == "hand":
                if out.o_hand:
                    out.load[g] = model.group_mass_com(sts["thumb"], g)
            elif out.o[g]:
                out.load[g] = model.group_mass_com(sts[g], g)
            out.load_true[g] = unit.truth.group_mass_com(sth, g)
    return out


def pose_item(cfg, rr, d):
    """The pose item of one (reading, digit) pair."""
    if not rr.o[d]:
        return 0.0
    w = cfg["sub_weights"]
    return rr.o[d] * (float(w["pose_point"]) * band(cfg, "pos", rr.pos_err_mm[d])
                      + float(w["pose_normal"]) * band(cfg, "nrm", rr.nrm_err_deg[d]))


def force_item(cfg, rr, d):
    if not rr.o[d]:
        return 0.0
    f, ft = rr.force[d], rr.force_true[d]
    if not (math.isfinite(f) and math.isfinite(ft) and ft > 0.0):
        return 0.0
    return band(cfg, "force", abs(f - ft) / ft)


def load_item(cfg, rr, g):
    o = rr.o_hand if g == "hand" else rr.o[g]
    if not o or g not in rr.load:
        return 0.0
    m_s, c_s = rr.load[g]
    m_t, c_t = rr.load_true[g]
    if not (m_t > 0.0 and math.isfinite(m_s)):
        return 0.0
    e_m = abs(m_s - m_t) / m_t
    e_c = 1000.0 * float(np.linalg.norm(np.asarray(c_s) - np.asarray(c_t)))
    return math.sqrt(band(cfg, "mass", e_m) * band(cfg, "com", e_c))
