"""The registration of a settled twin onto the true support frame.

    pts = ref_points(pieces_mm)                 # {key: (3,) mm}: footprint centroid (x, y) and highest point (z)
    fit_res = fit(twin_pts, truth_pts, cfg)     # q = s Rz(psi) p + (tx, ty, 0), trimmed, scale clamped
    reg = transform_pieces(pieces_mm, fit_res)  # the twin's pieces in the true support frame

The twin's pieces arrive in its own support-top frame (twin_sim.support_frame: z = 0 on the top face). One reference
point per named thing:
  - (x, y) = the centroid of the thing's footprint: the union of its collision pieces projected on z = 0, rasterised
    on a fixed grid of `raster_mm` cells (a cell counts when its centre lies inside the projection of a piece; the
    grid is anchored at the frame's origin, so the raster does not depend on the pieces' order);
  - z = the highest point of its collision pieces.
The truth's points are tests/gt/ref_points.json: the same function applied to the settled reference twin.

The fit, in closed form: the yaw psi is the 2-D Procrustes angle of the centred (x, y) points; the scale s is the
least-squares scale of all three coordinates about the support-frame origin for that yaw (z is never shifted, so the
top stays on the true plane), clamped to cfg["scale_range"]; (tx, ty) is the least-squares shift for the clamped
scale. Trimming: while the worst residual exceeds cfg["within_mm"] and more than cfg["min_things"] things remain, the
worst thing is dropped (residuals equal within 1e-9 mm: the thing later in the task's thing order) and the fit is
made again. The registration holds when at least cfg["min_things"] things end within cfg["within_mm"]; otherwise the
twin is not registered (and its use of the pick table is 0).

No file, no clock, no randomness: the same inputs give the same numbers.
"""
import math

import numpy as np

TIE_MM = 1e-9
_EPS_INSIDE = 1e-9


def _hull2d_equations(xy):
    """Half-plane equations (a, b, c) with a x + b y + c <= 0 inside, of the convex hull of (n, 2) points; None when
    the projection has no area (a flat piece seen edge-on)."""
    from scipy.spatial import ConvexHull
    from scipy.spatial import QhullError
    xy = np.asarray(xy, dtype=np.float64)
    if len(xy) < 3:
        return None
    try:
        hull = ConvexHull(xy)
    except (QhullError, ValueError):
        return None
    if not hull.volume > 0.0:                       # in 2-D, volume is the area
        return None
    return np.asarray(hull.equations, dtype=np.float64)


class FootprintTooLarge(ValueError):
    """A thing's footprint spans more raster cells than the grader rasterises (a twin far larger than any site)."""


def footprint(pieces_mm, raster_mm=1.0, max_cells=None):
    """(cell indices (m, 2) int64, cell centres (m, 2) mm) of the union of the pieces' projections on z = 0, on the
    grid of `raster_mm` cells anchored at the origin (cell (i, j) = [i r, (i + 1) r) x [j r, (j + 1) r)).
    max_cells bounds the cells of the thing's bounding rectangle (FootprintTooLarge beyond)."""
    r = float(raster_mm)
    if not r > 0.0:
        raise ValueError("raster_mm must be positive")
    pieces = [np.asarray(V, dtype=np.float64).reshape(-1, 3) for V in pieces_mm]
    for V in pieces:
        if not np.all(np.isfinite(V)):
            raise ValueError("a piece has a non-finite vertex")
    empty = (np.zeros((0, 2), dtype=np.int64), np.zeros((0, 2), dtype=np.float64))
    if not pieces:
        return empty
    allv = np.concatenate([V[:, :2] for V in pieces], axis=0)
    g_lo = np.floor(allv.min(axis=0) / r).astype(np.int64)
    g_hi = np.ceil(allv.max(axis=0) / r).astype(np.int64)
    shape = (int(g_hi[0] - g_lo[0] + 1), int(g_hi[1] - g_lo[1] + 1))
    if max_cells is not None and float(shape[0]) * float(shape[1]) > float(max_cells):
        raise FootprintTooLarge("a footprint of %d x %d cells (at most %d)" % (shape[0], shape[1], max_cells))
    grid = np.zeros(shape, dtype=bool)                  # cell (i, j) of the thing's bounding rectangle
    for V in pieces:
        eq = _hull2d_equations(V[:, :2])
        if eq is None:
            continue
        lo = np.floor(V[:, :2].min(axis=0) / r).astype(np.int64)
        hi = np.ceil(V[:, :2].max(axis=0) / r).astype(np.int64)
        ii = np.arange(lo[0], hi[0] + 1, dtype=np.int64)
        jj = np.arange(lo[1], hi[1] + 1, dtype=np.int64)
        cx, cy = (ii + 0.5) * r, (jj + 0.5) * r
        inside = np.ones((len(ii), len(jj)), dtype=bool)
        for a, b, c in eq:
            inside &= (a * cx[:, None] + b * cy[None, :] + c) <= _EPS_INSIDE
        grid[ii[0] - g_lo[0]:ii[-1] - g_lo[0] + 1, jj[0] - g_lo[1]:jj[-1] - g_lo[1] + 1] |= inside
    I, J = np.nonzero(grid)                             # C order: sorted by i, then j
    if len(I) == 0:
        return empty
    idx = np.stack([I + g_lo[0], J + g_lo[1]], axis=1).astype(np.int64)
    return idx, (idx.astype(np.float64) + 0.5) * r


def ref_point(pieces_mm, raster_mm=1.0, max_cells=None):
    """(3,) mm reference point of one thing, or None when its footprint holds no cell."""
    pieces = [np.asarray(V, dtype=np.float64).reshape(-1, 3) for V in pieces_mm]
    if not pieces:
        return None
    _idx, centres = footprint(pieces, raster_mm, max_cells)
    if len(centres) == 0:
        return None
    z = max(float(V[:, 2].max()) for V in pieces)
    c = centres.mean(axis=0)
    return np.array([float(c[0]), float(c[1]), z], dtype=np.float64)


def ref_points(pieces_mm, raster_mm=1.0, max_cells=None):
    """{key: (3,) float64 mm} for every thing of `pieces_mm` ({key: [V (n, 3) mm]}) whose footprint holds a cell;
    keys in the input's order."""
    out = {}
    for key, pieces in pieces_mm.items():
        p = ref_point(pieces, raster_mm, max_cells)
        if p is not None:
            out[key] = p
    return out


def rz(psi_rad):
    c, s = math.cos(psi_rad), math.sin(psi_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _solve(P, Q, scale_range):
    """Closed-form (s, psi, t, s_raw) for points P, Q (n, 3) mm."""
    pc, qc = P[:, :2].mean(axis=0), Q[:, :2].mean(axis=0)
    Pc, Qc = P[:, :2] - pc, Q[:, :2] - qc
    a = float(np.sum(Pc[:, 0] * Qc[:, 0] + Pc[:, 1] * Qc[:, 1]))
    b = float(np.sum(Pc[:, 0] * Qc[:, 1] - Pc[:, 1] * Qc[:, 0]))
    psi = math.atan2(b, a) if (a != 0.0 or b != 0.0) else 0.0
    R2 = rz(psi)[:2, :2]
    A = Pc @ R2.T
    num = float(np.sum(A * Qc)) + float(np.sum(P[:, 2] * Q[:, 2]))
    den = float(np.sum(A * A)) + float(np.sum(P[:, 2] * P[:, 2]))
    s_raw = num / den if den > 0.0 else 1.0
    lo, hi = float(scale_range[0]), float(scale_range[1])
    s = min(max(s_raw, lo), hi)
    t = qc - s * (R2 @ pc)
    return s, psi, np.array([float(t[0]), float(t[1])]), s_raw


def apply(fit_res, P_mm):
    """The fitted similarity applied to points (n, 3) mm."""
    P = np.asarray(P_mm, dtype=np.float64).reshape(-1, 3)
    R = rz(math.radians(float(fit_res["psi_deg"])))
    t = np.array([float(fit_res["t_mm"][0]), float(fit_res["t_mm"][1]), 0.0])
    return float(fit_res["s"]) * (P @ R.T) + t


def transform_pieces(pieces_mm, fit_res):
    """{key: [V (n, 3) mm]} mapped into the true support frame by the fit."""
    return {k: [apply(fit_res, V) for V in pieces] for k, pieces in pieces_mm.items()}


def _order_index(order, keys):
    pos = {k: i for i, k in enumerate(order or [])}
    tail = sorted(k for k in keys if k not in pos)
    for i, k in enumerate(tail):
        pos[k] = len(order or []) + i
    return pos


def fit(twin_pts, truth_pts, cfg):
    """The registration of 3.11: {"ok", "s", "s_raw", "psi_deg", "t_mm", "used", "resid_mm", "n_within", "common",
    "dropped", "reason"}. cfg: min_things, within_mm, scale_range, method ("footprint"), optional "order" (the
    task's thing order, used for ties; default: truth_pts' order)."""
    method = cfg.get("method", "footprint")
    if method != "footprint":
        raise ValueError("registration method %r is not implemented" % (method,))
    min_things = int(cfg["min_things"])
    within = float(cfg["within_mm"])
    scale_range = cfg["scale_range"]
    order = list(cfg.get("order") or list(truth_pts))
    common = [k for k in order if k in twin_pts and k in truth_pts]
    common += sorted(k for k in twin_pts if k in truth_pts and k not in common)
    pos = _order_index(order, common)
    out = {"ok": False, "s": 1.0, "s_raw": 1.0, "psi_deg": 0.0, "t_mm": [0.0, 0.0], "used": [],
           "resid_mm": {}, "n_within": 0, "common": list(common), "dropped": [], "reason": None,
           "min_things": min_things, "within_mm": within, "scale_range": [float(scale_range[0]),
                                                                          float(scale_range[1])]}
    P_all = {k: np.asarray(twin_pts[k], dtype=np.float64).reshape(3) for k in common}
    Q_all = {k: np.asarray(truth_pts[k], dtype=np.float64).reshape(3) for k in common}
    if any(not np.all(np.isfinite(P_all[k])) for k in common):
        out["reason"] = "a reference point of the twin is not finite"
        return out
    if len(common) < min_things:
        out["reason"] = "%d named things with a footprint (at least %d needed)" % (len(common), min_things)
        if not common:
            return out
    used = list(common)
    while True:
        P = np.stack([P_all[k] for k in used])
        Q = np.stack([Q_all[k] for k in used])
        s, psi, t, s_raw = _solve(P, Q, scale_range)
        cur = {"s": float(s), "psi_deg": math.degrees(psi), "t_mm": [float(t[0]), float(t[1])]}
        res = {k: float(np.linalg.norm(apply(cur, P_all[k][None, :])[0] - Q_all[k])) for k in used}
        worst = max(res.values())
        if worst > within and len(used) > min_things:
            cands = [k for k in used if res[k] >= worst - TIE_MM]
            drop = max(cands, key=lambda k: pos[k])
            used.remove(drop)
            out["dropped"].append(drop)
            continue
        break
    resid = {k: float(np.linalg.norm(apply(cur, P_all[k][None, :])[0] - Q_all[k])) for k in common}
    n_within = sum(1 for k in used if resid[k] <= within)
    finite = all(math.isfinite(v) for v in (cur["s"], cur["psi_deg"], cur["t_mm"][0], cur["t_mm"][1]))
    out.update(cur)
    out.update({"s_raw": float(s_raw), "used": list(used), "resid_mm": resid, "n_within": int(n_within)})
    out["ok"] = bool(finite and n_within >= min_things)
    if not out["ok"] and out["reason"] is None:
        out["reason"] = ("%d things within %.1f mm after the fit (at least %d needed)" % (n_within, within, min_things)
                         if finite else "the fit is not finite")
    return out
