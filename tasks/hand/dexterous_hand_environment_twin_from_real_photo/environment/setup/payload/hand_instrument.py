"""The pick instrument: plans a request of the pick sheet on one scene and executes the plan in another.

A scene is a set of named solids standing on a floor (the plane z = 0); units are mm, in the frame the instrument
plans in (z up). Every test is geometric and works on point samples of each solid's surface (on a twin's convex
pieces a square lattice of `twin_sample_mm` steps on each hull triangle and its edges at most that far apart; on a
scan, the scan's own samples) plus an inside test (`Solid.contains`) that catches a volume whose centre lies inside a
solid. Physics enters only through `lift`, a dynamic grasp-and-lift trial in MuJoCo.

The rules (one fixed procedure per rule; the numbers come from instrument_config.json and RULE_CONSTANTS below):

  body    a pinch at the height of the target's centroid (the mean of its surface samples): each pad at that
          height, or higher where its own floor clearance needs it. The finger pad (`finger_pad_mm`: thickness
          along the closing direction, width, height) faces the target from the side `dir_deg` (the direction d,
          counter-clockwise from +x seen from above), the thumb pad (`thumb_pad_mm`) from the opposite side. Each
          pad's band is the target's samples within half the pad's height of the pad's centre height and within half
          the pad's width of the line through the target's centroid along d. The grip width is the finger band's
          maximum along d minus the thumb band's minimum along d (the two samples the pads face); it must lie between
          `min_width_mm` and `max_width_mm`. Each pad starts `start_gap_mm` outside the band, a column of
          `column_mm` stands on each pad, and every pad and column must keep `margin_mm` from every other thing and
          `target_margin_mm` from the target. A pad's lowest face is at least `floor_clear_mm` above the floor.
  top     the same pinch at a band `top_below_mm` under the target's highest sample, centred on the target's samples
          in the finger pad's height band there (at least `top_min_band_samples` of them), for every thing alike.
  rim     the finger pad inside an opening at `rim.finger_depth_mm` under the rim's top, the thumb pad outside at
          `rim.thumb_depth_mm`, at azimuth `dir_deg` from the opening's centre (the centre of the samples between
          `rim_slab_mm` under the top); the wall may be at most `rim.max_wall_mm` thick; the inside pad starts
          `start_gap_inside_mm` from the inner wall and must leave `rim_min_inner_mm` between its back and the
          opening's centre.
  hook    a bar (`hook_bar_mm`: horizontal width, vertical thickness) through a through-opening of the target. The
          target's samples are projected along each axis d at `hook_step_deg` steps over half a turn onto a raster
          of `hook_raster_mm`; an opening is a region of empty raster cells that does not reach the raster's border
          and in which the bar's section grown by `margin_mm` on every side fits; its one site is, among the cells at
          which that grown section fits, the one farthest from the opening's boundary. The bar runs `hook_behind_mm`
          behind the opening's material and `hook_beyond_mm` beyond it; side +1 enters along +d, side -1 along -d.
          The sites of every axis are tried from the largest clearance down (a site whose bar's lowest face would lie
          under `hook_floor_clear_mm` above the floor is skipped); the first clear one of the requested side is
          taken. A planned hook is done when the target has material right above the bar.

The `envelope` argument of plan, plan_explain, execute and rehearse is None (no envelope) or a set of convex pieces
per rule fixed in the rule's frame (pinch rules: origin at the midpoint of the two planned pad centres, x from the
finger pad to the thumb pad, z up; hook: origin at the bar's centre in the opening's plane, x along the approach into
the opening, z up), each swept along a straight approach (`approach.from` x `approach.length_mm`): the convex hull of
the piece at the start pose and moved back along it. With one given, a site whose swept pieces come within
`exec_tol_mm` of any thing (the target included) or of the floor is not a site, and in execution such a contact is a
stop. A plan is executed the way it was made (with or without one).

Execution at one placement offset (`placements_mm`): STOP when a pad, a column or the bar comes within `exec_tol_mm`
of any thing or has its centre inside one, or when a pad's or the bar's lowest face is below the floor; otherwise a
pinch is DONE when, for both pads, the nearest sample ahead of the
pad's face (inside the pad's face outline, within `start_gap_mm` + `over_travel_mm`; inside pad: its own start gap)
belongs to the target, else MISS; a hook is DONE when the target has material right above the bar, else MISS. A
request with no plan is DECLINED.

"Comes within t of" a convex volume means: a sample lies inside the volume grown by t along each of its face
normals. Contract: no file writes, no clock, no randomness; the same inputs give byte-identical
json.dumps(..., sort_keys=True) outputs.
"""
import json
import math
import pathlib

import numpy as np

FORMAT = "hand-instrument/1"
PINCH_RULES = ("body", "top", "rim")
RULES = ("body", "top", "rim", "hook")

# The rules' fixed numbers (mm unless named otherwise). instrument_config.json carries them as its "rule_constants"
# block (the same values); any key a config's block gives replaces the value here for planning and execution, except
# "twin_sample_mm": TwinSolid samples with the value here, so a config that gives another value is refused.
RULE_CONSTANTS = {
    "floor_clear_mm": 2.5,            # a pinch pad's lowest face is planned at least this far above the floor
    "pinch_min_samples": 4,           # samples each pad's band must hold
    "min_width_mm": 2.0,              # the narrowest grip width
    "top_below_mm": 12.0,             # the top pinch's band centre under the target's highest sample
    "top_min_band_samples": 5,        # samples the top band must hold
    "rim_slab_mm": [3.0, 9.0],        # the opening's centre: mean of the samples this far under the top
    "rim_min_slab_samples": 30,
    "rim_min_samples": 3,             # samples each rim pad must see on its side of the centre
    "rim_min_inner_mm": 2.0,          # room left between the inside pad's back and the opening's centre
    "hook_border_mm": 5.0,            # free border of the hook raster around the projection
    "hook_closing_steps": 2,          # raster closing (cross element) that seals gaps between samples
    "hook_ring_mm": 12.0,             # the opening's rim: its material within this distance of the opening
    "hook_min_ring_samples": 5,
    "hook_floor_clear_mm": 1.0,       # the bar's lowest face is at least this far above the floor
    "hook_check_beyond_ring_mm": 2.0, # the 'material above the bar' box: 2 mm beyond the rim along the bar,
    "hook_check_height_mm": 30.0,     # 30 mm tall, right above the bar, as wide as the bar
    "hook_min_check_samples": 5,
    "twin_sample_mm": 1.0,            # sample spacing on a twin's convex pieces
}

# The hand's contact parameters in the lift trial (the standard simulation's contact settings).
HAND_CONTACT = {"condim": "4", "solref": "0.004 1", "solimp": "0.95 0.99 0.001 0.5 2", "margin": "0", "gap": "0"}
_NAME = "pickhand"                    # prefix of every element the lift trial adds to a model

_IN_EPS = 1e-9                        # half-space slack of the inside tests


# ------------------------------------------------------------------------------------------------ the request list
def enumerate_requests(names, cfg):
    """The pick sheet's request list: the things with requested = true in names["things"] order; per thing the
    rules in cfg["rule_order"]; body, top and rim once per cfg["directions_deg"] ascending (side None); hook once
    per cfg["hook_sides"] (-1 then +1; dir_deg None); ids "R%04d" from R0001 in that order.
    Returns [{"id", "thing", "rule", "dir_deg", "side"}]."""
    out = []
    for th in names["things"]:
        if not th["requested"]:
            continue
        for rule in cfg["rule_order"]:
            if rule not in RULES:
                raise ValueError("unknown rule %r" % (rule,))
            if rule == "hook":
                for side in sorted(int(s) for s in cfg["hook_sides"]):
                    out.append({"thing": th["key"], "rule": rule, "dir_deg": None, "side": side})
            else:
                for d in sorted(int(v) for v in cfg["directions_deg"]):
                    out.append({"thing": th["key"], "rule": rule, "dir_deg": d, "side": None})
    return [{"id": "R%04d" % (i + 1), **r} for i, r in enumerate(out)]


# ------------------------------------------------------------------------------------------------------- solids
class Solid:
    """A thing's solid. key: str. surface_points(): (N, 3) float64 mm; contains(P): bool (M,) for P (M, 3) mm.
    raster_points(): the samples the hook raster projects (the surface samples unless a subclass says otherwise)."""

    key = None

    def surface_points(self):
        raise NotImplementedError

    def contains(self, P):
        raise NotImplementedError

    def raster_points(self):
        return self.surface_points()


class _Index:
    """The samples of one solid sorted by x (stable), with their bounding box, for box-shaped range queries."""

    def __init__(self, P):
        P = np.ascontiguousarray(np.asarray(P, dtype=np.float64).reshape(-1, 3))
        order = np.argsort(P[:, 0], kind="stable")
        self.P = np.ascontiguousarray(P[order])
        self.x = np.ascontiguousarray(self.P[:, 0])
        if len(P):
            self.lo, self.hi = self.P.min(axis=0), self.P.max(axis=0)
        else:
            self.lo = np.full(3, np.inf)
            self.hi = np.full(3, -np.inf)

    def in_aabb(self, lo, hi):
        """The samples inside the axis-aligned box [lo, hi] (closed)."""
        if np.any(hi < self.lo) or np.any(lo > self.hi):
            return self.P[:0]
        i0 = int(np.searchsorted(self.x, lo[0], side="left"))
        i1 = int(np.searchsorted(self.x, hi[0], side="right"))
        Q = self.P[i0:i1]
        m = (Q[:, 1] >= lo[1]) & (Q[:, 1] <= hi[1]) & (Q[:, 2] >= lo[2]) & (Q[:, 2] <= hi[2])
        return Q[m]


def _points(solid):
    """The solid's surface samples in their own order, (N, 3) float64."""
    return np.asarray(solid.surface_points(), dtype=np.float64).reshape(-1, 3)


def _index(solid):
    ix = getattr(solid, "_hi_index", None)
    if ix is None:
        ix = _Index(solid.surface_points())
        solid._hi_index = ix
    return ix


def _hull(V):
    """Convex hull of the points V (n, 3): (vertices (k, 3), equations (f, 4) with unit outward normals). A flat or
    degenerate point set is thickened by 1e-3 mm across its missing directions first."""
    from scipy.spatial import ConvexHull
    V = np.asarray(V, dtype=np.float64).reshape(-1, 3)
    if len(V) == 0:
        raise ValueError("a convex piece needs at least one vertex")
    c = V.mean(axis=0)
    _, s, vt = np.linalg.svd(V - c, full_matrices=True)
    s = np.concatenate([s, np.zeros(3 - len(s))]) if len(s) < 3 else s
    flat = [vt[i] for i in range(3) if s[i] <= 1e-9 * max(1.0, s[0])]
    if flat:
        extra = [V]
        for n in flat:
            extra += [V + 1e-3 * n, V - 1e-3 * n]
        V = np.concatenate(extra, axis=0)
    h = ConvexHull(V)
    return np.ascontiguousarray(V[h.vertices]), np.ascontiguousarray(h.equations), h.simplices, V


def _inside_eq(P, eq, slack):
    """bool (M,): P inside the half-spaces eq (f, 4) grown by slack (n . p + b <= slack for every face)."""
    ok = np.ones(len(P), dtype=bool)
    for f in eq:
        ok &= (P[:, 0] * f[0] + P[:, 1] * f[1] + P[:, 2] * f[2] + f[3]) <= slack
        if not ok.any():
            break
    return ok


def _sample_triangle(a, b, c, h):
    """Deterministic samples of one triangle: a square lattice of step h in the triangle's plane (laid out from an
    end of its longest edge) and its three edges at steps of at most h, the corners included."""
    T = [a, b, c]
    e = [np.linalg.norm(T[1] - T[0]), np.linalg.norm(T[2] - T[1]), np.linalg.norm(T[0] - T[2])]
    k = int(np.argmax(e))
    a, b, c = T[k], T[(k + 1) % 3], T[(k + 2) % 3]
    out = []
    for p, q in ((a, b), (b, c), (c, a)):
        L = float(np.linalg.norm(q - p))
        n = max(1, int(math.ceil(L / h)))
        t = np.arange(n + 1, dtype=np.float64) / n
        out.append(p[None, :] + t[:, None] * (q - p)[None, :])
    L = float(np.linalg.norm(b - a))
    if L > 0:
        u = (b - a) / L
        w = c - a
        cu = float(w @ u)
        v = w - cu * u
        hv = float(np.linalg.norm(v))
        if hv > 1e-9:
            v = v / hv
            for j in range(1, int(math.floor(hv / h)) + 1):
                y = j * h
                if y >= hv:
                    break
                xl = cu * y / hv
                xr = L + (cu - L) * y / hv
                i0, i1 = int(math.ceil(xl / h)), int(math.floor(xr / h))
                if i1 < i0:
                    continue
                xs = np.arange(i0, i1 + 1, dtype=np.float64) * h
                out.append(a[None, :] + xs[:, None] * u[None, :] + y * v[None, :])
    return np.concatenate(out, axis=0)


class TwinSolid(Solid):
    """A solid from convex pieces [(V_i (n, 3) mm)]: the union of the pieces' hulls. Surface samples on the union's
    surface: every hull face sampled at steps of at most `twin_sample_mm`, the samples strictly inside another piece
    of the same thing removed."""

    def __init__(self, key, pieces):
        self.key = key
        self.pieces = []
        self._eq = []
        tris = []
        for V in pieces:
            verts, eq, simp, Vh = _hull(V)
            self.pieces.append(verts)
            self._eq.append(eq)
            tris.append((Vh, simp))
        if not self.pieces:
            raise ValueError("thing %r has no piece" % (key,))
        h = float(RULE_CONSTANTS["twin_sample_mm"])
        keep = []
        boxes = [(p.min(axis=0), p.max(axis=0)) for p in self.pieces]
        for i, (Vh, simp) in enumerate(tris):
            S = np.concatenate([_sample_triangle(Vh[t[0]], Vh[t[1]], Vh[t[2]], h) for t in simp], axis=0)
            S = np.unique(np.round(S, 6), axis=0)
            drop = np.zeros(len(S), dtype=bool)
            for j, eq in enumerate(self._eq):
                if j == i:
                    continue
                lo, hi = boxes[j]
                m = np.all((S >= lo) & (S <= hi), axis=1) & ~drop
                if m.any():
                    idx = np.nonzero(m)[0]
                    drop[idx[_inside_eq(S[idx], eq, -1e-6)]] = True
            keep.append(S[~drop])
        self._pts = np.ascontiguousarray(np.unique(np.concatenate(keep, axis=0), axis=0))

    def surface_points(self):
        return self._pts

    def contains(self, P):
        P = np.asarray(P, dtype=np.float64).reshape(-1, 3)
        out = np.zeros(len(P), dtype=bool)
        for eq in self._eq:
            out |= _inside_eq(P, eq, _IN_EPS)
        return out


def _npz(x):
    if isinstance(x, (str, pathlib.Path)):
        with np.load(x, allow_pickle=False) as z:
            return {k: z[k] for k in z.files}
    return x


class ScanSolid(Solid):
    """A solid from a surface-sample file ("<key>__pts": int32 (N, 3) in units of 0.01 mm) and an occupancy file
    ("<key>__origin_mm", "<key>__shape", "<key>__bits": 1 mm voxels, np.packbits big-endian of the C-order grid)."""

    def __init__(self, key, pts_mm, origin_mm, grid):
        self.key = key
        self._pts = np.ascontiguousarray(np.asarray(pts_mm, dtype=np.float64).reshape(-1, 3))
        self.origin_mm = np.asarray(origin_mm, dtype=np.float64).reshape(3)
        self.grid = np.asarray(grid, dtype=bool)

    @classmethod
    def from_npz(cls, surface_npz, occupancy_npz, key):
        s, o = _npz(surface_npz), _npz(occupancy_npz)
        pts = np.asarray(s[f"{key}__pts"], dtype=np.int32).astype(np.float64) / 100.0
        shape = tuple(int(v) for v in np.asarray(o[f"{key}__shape"]).reshape(3))
        n = shape[0] * shape[1] * shape[2]
        bits = np.asarray(o[f"{key}__bits"], dtype=np.uint8)
        grid = np.unpackbits(bits, bitorder="big", count=n).astype(bool).reshape(shape, order="C")
        return cls(key, pts, np.asarray(o[f"{key}__origin_mm"], dtype=np.float64), grid)

    def surface_points(self):
        return self._pts

    def contains(self, P):
        P = np.asarray(P, dtype=np.float64).reshape(-1, 3)
        ijk = np.floor(P - self.origin_mm).astype(np.int64)
        ok = np.all((ijk >= 0) & (ijk < np.asarray(self.grid.shape)), axis=1)
        out = np.zeros(len(P), dtype=bool)
        out[ok] = self.grid[ijk[ok, 0], ijk[ok, 1], ijk[ok, 2]]
        return out


class Scene:
    """things: dict[key, Solid] (its order is the order of every report); the floor is the plane z = 0."""

    floor_z = 0.0

    def __init__(self, things):
        self.things = dict(things)

    @classmethod
    def from_pieces(cls, pieces):
        """pieces: {key: [V (n, 3) mm]}, already in the frame the instrument plans in."""
        return cls({k: TwinSolid(k, v) for k, v in pieces.items()})

    @classmethod
    def from_gt(cls, gt_dir):
        """things.json for the keys (its order), scans_surface.npz and scans_occupancy.npz for the solids."""
        gt = pathlib.Path(gt_dir)
        keys = list(json.loads((gt / "things.json").read_text(encoding="utf-8"))["things"])
        s, o = _npz(gt / "scans_surface.npz"), _npz(gt / "scans_occupancy.npz")
        return cls({k: ScanSolid.from_npz(s, o, k) for k in keys})


# -------------------------------------------------------------------------------------------- the configuration
def _rc(cfg):
    rc = dict(RULE_CONSTANTS)
    given = cfg.get("rule_constants", {}) or {}
    unknown = sorted(set(given) - set(RULE_CONSTANTS))
    if unknown:
        raise ValueError("rule_constants: unknown keys %s" % unknown)
    if "twin_sample_mm" in given and float(given["twin_sample_mm"]) != float(RULE_CONSTANTS["twin_sample_mm"]):
        raise ValueError("rule_constants: twin_sample_mm is fixed at %r (TwinSolid samples with it)"
                         % RULE_CONSTANTS["twin_sample_mm"])
    rc.update(given)
    return rc


def _num(cfg):
    f = [float(x) for x in cfg["finger_pad_mm"]]
    t = [float(x) for x in cfg["thumb_pad_mm"]]
    return {"finger": f, "thumb": t, "bar": [float(x) for x in cfg["hook_bar_mm"]],
            "behind": float(cfg["hook_behind_mm"]), "beyond": float(cfg["hook_beyond_mm"]),
            "col": float(cfg["column_mm"]), "gap": float(cfg["start_gap_mm"]),
            "gap_in": float(cfg["start_gap_inside_mm"]), "over": float(cfg["over_travel_mm"]),
            "maxw": float(cfg["max_width_mm"]), "margin": float(cfg["margin_mm"]),
            "tmargin": float(cfg["target_margin_mm"]), "tol": float(cfg["exec_tol_mm"]),
            "rim_fd": float(cfg["rim"]["finger_depth_mm"]), "rim_td": float(cfg["rim"]["thumb_depth_mm"]),
            "rim_wall": float(cfg["rim"]["max_wall_mm"]), "cell": float(cfg["hook_raster_mm"]),
            "hstep": float(cfg["hook_step_deg"]), "rc": _rc(cfg)}


# ----------------------------------------------------------------------------------------------- volumes and tests
def _dirs(theta_deg):
    t = math.radians(float(theta_deg))
    d = np.array([math.cos(t), math.sin(t), 0.0])
    lat = np.array([-d[1], d[0], 0.0])
    return d, lat


def _rot(d):
    lat = np.array([-d[1], d[0], 0.0])
    return np.stack([np.array([d[0], d[1], 0.0]), lat, np.array([0.0, 0.0, 1.0])], axis=1)


def _pad_boxes(center_xy, d, s_face, zc, size, side, col_mm):
    """A pad whose contact face sits at s_face along d from center_xy; its body lies on `side` (+1 beyond the face,
    -1 before it). Returns (pad box, column box), each (c, R, half)."""
    t, w, h = size
    R = _rot(d)
    c = np.array([center_xy[0], center_xy[1], 0.0]) + R[:, 0] * (s_face + side * t / 2)
    c[2] = zc
    pad = (c, R, np.array([t / 2, w / 2, h / 2]))
    cc = c.copy()
    cc[2] = zc + h / 2 + col_mm / 2
    col = (cc, R, np.array([t / 2, w / 2, col_mm / 2]))
    return pad, col


def _local(P, c, R):
    """(P - c) expressed in the box axes R (columns), elementwise (no BLAS)."""
    Q = P - c
    return np.stack([Q[:, 0] * R[0, j] + Q[:, 1] * R[1, j] + Q[:, 2] * R[2, j] for j in range(3)], axis=1)


def _box_aabb(box, grow):
    c, R, half = box
    ext = np.abs(R) @ (np.asarray(half) + grow)
    return c - ext - 1e-9, c + ext + 1e-9


def _in_box(P, box, grow):
    c, R, half = box
    L = _local(P, c, R)
    h = np.asarray(half) + grow
    return (np.abs(L[:, 0]) <= h[0]) & (np.abs(L[:, 1]) <= h[1]) & (np.abs(L[:, 2]) <= h[2])


def _box_hits(scene, box, grow_other, grow_target, target):
    """Keys (scene order) of the things with a sample in the box grown by their margin, or whose solid holds the
    box's centre."""
    hits = []
    for k, sol in scene.things.items():
        g = grow_target if k == target else grow_other
        lo, hi = _box_aabb(box, g)
        ix = _index(sol)
        if np.any(hi < ix.lo) or np.any(lo > ix.hi):
            continue
        Q = ix.in_aabb(lo, hi)
        if (len(Q) and _in_box(Q, box, g).any()) or bool(sol.contains(box[0][None, :])[0]):
            hits.append(k)
    return hits


def _hull_hits(scene, verts, eq, tol):
    """Keys (scene order) of the things with a sample within tol of the convex volume (verts, eq), or whose solid
    holds the volume's centre; "plane" last when the volume reaches within tol of the floor."""
    hits = []
    lo, hi = verts.min(axis=0) - tol - 1e-9, verts.max(axis=0) + tol + 1e-9
    centre = verts.mean(axis=0)[None, :]
    for k, sol in scene.things.items():
        ix = _index(sol)
        if np.any(hi < ix.lo) or np.any(lo > ix.hi):
            continue
        Q = ix.in_aabb(lo, hi)
        if (len(Q) and _inside_eq(Q, eq, tol).any()) or bool(sol.contains(centre)[0]):
            hits.append(k)
    if float(verts[:, 2].min()) - scene.floor_z <= tol:
        hits.append("plane")
    return hits


def _first_blocker(hits, target):
    others = [h for h in hits if h != target and h != "plane"]
    if others:
        return others[0]
    if target in hits:
        return target
    return "plane" if "plane" in hits else None


def _box_json(box):
    c, R, half = box
    return {"c": [float(v) for v in c], "R": [[float(v) for v in row] for row in R],
            "half": [float(v) for v in half]}


def _box_from(js, off=None):
    c = np.asarray(js["c"], dtype=np.float64)
    if off is not None:
        c = c + off
    return (c, np.asarray(js["R"], dtype=np.float64), np.asarray(js["half"], dtype=np.float64))


# ------------------------------------------------------------------------------------------------------ envelope
def _env_rule(envelope, rule):
    """The envelope's entry of a rule; an envelope without a "top" entry gives the top pinch its "raised" entry."""
    if envelope is None:
        return None
    rules = envelope["rules"] if "rules" in envelope else envelope
    if rule == "top" and "top" not in rules:
        return rules["raised"]
    return rules[rule]


def _frame_json(origin, x):
    x = np.array([x[0], x[1], 0.0])
    x = x / np.linalg.norm(x)
    z = np.array([0.0, 0.0, 1.0])
    y = np.cross(z, x)
    R = np.stack([x, y, z], axis=1)
    return {"origin_mm": [float(v) for v in origin], "R": [[float(v) for v in row] for row in R]}


def _swept(plan, envelope, off=None):
    """[(name, vertices (k, 3), equations (f, 4))] of the rule's envelope pieces swept along the approach, in the
    scene's frame (moved by off)."""
    er = _env_rule(envelope, plan["rule"])
    if er is None:
        return []
    o = np.asarray(plan["frame"]["origin_mm"], dtype=np.float64)
    R = np.asarray(plan["frame"]["R"], dtype=np.float64)
    fw = R @ np.asarray(er["approach"]["from"], dtype=np.float64)
    L = float(er["approach"]["length_mm"])
    out = []
    for pc in er["pieces"]:
        V = np.asarray(pc["vertices"], dtype=np.float64).reshape(-1, 3)
        Vw = o[None, :] + np.stack([V[:, 0] * R[j, 0] + V[:, 1] * R[j, 1] + V[:, 2] * R[j, 2] for j in range(3)],
                                   axis=1)
        verts, eq, _, _ = _hull(np.concatenate([Vw, Vw + L * fw[None, :]], axis=0))
        if off is not None:
            verts = verts + off[None, :]
            eq = eq.copy()
            eq[:, 3] = eq[:, 3] - eq[:, :3] @ off
        out.append((str(pc.get("name", "")), verts, eq))
    return out


def swept_envelope(plan, envelope, offset_mm=(0.0, 0.0, 0.0)):
    """The plan's swept envelope pieces: [{"name", "vertices": [[x, y, z], ...], "equations": [[nx, ny, nz, b],
    ...]}] in the scene's frame at the given placement offset (mm)."""
    if plan is None:
        return []
    off = np.asarray(offset_mm, dtype=np.float64)
    return [{"name": n, "vertices": [[float(v) for v in p] for p in V], "equations": [[float(v) for v in e] for e in E]}
            for n, V, E in _swept(plan, envelope, off)]


def _envelope_hits(scene, plan, envelope, tol, off=None):
    hits = []
    for _, V, E in _swept(plan, envelope, off):
        for h in _hull_hits(scene, V, E, tol):
            if h not in hits:
                hits.append(h)
    order = list(scene.things) + ["plane"]
    return sorted(hits, key=order.index)


# ------------------------------------------------------------------------------------------------------ planning
def _explain(plan=None, declined_by=None, blocker=None, blockers=()):
    return {"plan": plan, "declined_by": declined_by, "blocker": blocker, "blockers": list(blockers)}


def _clear_boxes(scene, named_boxes, n, target):
    """The first box (in order) that a thing blocks: (name, hits) or (None, [])."""
    for name, box in named_boxes:
        hits = _box_hits(scene, box, n["margin"], n["tmargin"], target)
        if hits:
            return name, hits
    return None, []


def _plan_pinch(scene, thing, where, theta, n, envelope, rule):
    rc = n["rc"]
    Pt = _points(scene.things[thing])
    tA, wA, hA = n["finger"]
    tB, wB, hB = n["thumb"]
    fc = float(rc["floor_clear_mm"])
    if where == "body":
        c = Pt.mean(axis=0)
        zA = max(c[2], hA / 2 + fc)
        zB = max(c[2], hB / 2 + fc)
    elif where == "top":
        ztop = Pt[:, 2].max()
        zA = max(ztop - float(rc["top_below_mm"]), hA / 2 + fc)
        zB = max(ztop - float(rc["top_below_mm"]), hB / 2 + fc)
        band = Pt[np.abs(Pt[:, 2] - zA) <= hA / 2]
        if len(band) < int(rc["top_min_band_samples"]):
            return _explain(declined_by="width")
        c = band.mean(axis=0)
    else:
        raise ValueError("unknown pinch %r" % (where,))
    rel = Pt - c
    d, lat = _dirs(theta)
    ru = rel[:, 0] * lat[0] + rel[:, 1] * lat[1]
    rs = rel[:, 0] * d[0] + rel[:, 1] * d[1]
    selA = (np.abs(Pt[:, 2] - zA) <= hA / 2) & (np.abs(ru) <= wA / 2)
    selB = (np.abs(Pt[:, 2] - zB) <= hB / 2) & (np.abs(ru) <= wB / 2)
    if selA.sum() < int(rc["pinch_min_samples"]) or selB.sum() < int(rc["pinch_min_samples"]):
        return _explain(declined_by="width")
    sp = rs[selA].max()
    sm = rs[selB].min()
    if sp - sm > n["maxw"] or sp - sm < float(rc["min_width_mm"]):
        return _explain(declined_by="width")
    A = _pad_boxes(c[:2], d, sp + n["gap"], zA, n["finger"], +1, n["col"])
    B = _pad_boxes(c[:2], d, sm - n["gap"], zB, n["thumb"], -1, n["col"])
    bad, hits = _clear_boxes(scene, [("pads", A[0]), ("column", A[1]), ("pads", B[0]), ("column", B[1])], n, thing)
    if bad:
        return _explain(declined_by=bad, blocker=_first_blocker(hits, thing), blockers=hits)
    travel = n["gap"] + n["over"]
    plan = {"kind": "pinch", "rule": rule, "thing": thing, "dir_deg": int(theta), "side": None,
            "theta_deg": float(theta), "width_mm": float(sp - sm),
            "pads": [{"role": "finger", "pad": _box_json(A[0]), "column": _box_json(A[1]), "close": -1,
                      "travel_mm": float(travel)},
                     {"role": "thumb", "pad": _box_json(B[0]), "column": _box_json(B[1]), "close": 1,
                      "travel_mm": float(travel)}],
            "frame": _frame_json((A[0][0] + B[0][0]) / 2, B[0][0] - A[0][0]),
            "envelope": envelope is not None}
    return _with_envelope(scene, plan, envelope, n, thing)


def _with_envelope(scene, plan, envelope, n, thing):
    if envelope is None:
        return _explain(plan=plan)
    hits = _envelope_hits(scene, plan, envelope, n["tol"])
    if hits:
        return _explain(declined_by="envelope", blocker=_first_blocker(hits, thing), blockers=hits)
    return _explain(plan=plan)


def _rim_geometry(scene, thing, phi, n):
    """(c, ztop, r_in, r_out) of the rim at azimuth phi, or a decline reason string."""
    rc = n["rc"]
    Pt = _points(scene.things[thing])
    tA, wA, hA = n["finger"]
    tB, wB, hB = n["thumb"]
    ztop = Pt[:, 2].max()
    s0, s1 = (float(v) for v in rc["rim_slab_mm"])
    slab = Pt[(Pt[:, 2] <= ztop - s0) & (Pt[:, 2] >= ztop - s1)]
    if len(slab) < int(rc["rim_min_slab_samples"]):
        return "no_opening"
    c = slab.mean(axis=0)
    relA = Pt[(Pt[:, 2] >= ztop - n["rim_fd"] - hA / 2)] - c
    relB = Pt[(Pt[:, 2] >= ztop - n["rim_td"] - hB / 2)] - c
    d, lat = _dirs(phi)
    uA = relA[:, 0] * lat[0] + relA[:, 1] * lat[1]
    sA = relA[:, 0] * d[0] + relA[:, 1] * d[1]
    uB = relB[:, 0] * lat[0] + relB[:, 1] * lat[1]
    sB = relB[:, 0] * d[0] + relB[:, 1] * d[1]
    selA = (np.abs(uA) <= wA / 2) & (sA > 0)
    selB = (np.abs(uB) <= wB / 2) & (sB > 0)
    if selA.sum() < int(rc["rim_min_samples"]) or selB.sum() < int(rc["rim_min_samples"]):
        return "no_opening"
    return c, ztop, float(sA[selA].min()), float(sB[selB].max()), d


def _plan_rim(scene, thing, phi, n, envelope):
    g = _rim_geometry(scene, thing, phi, n)
    if isinstance(g, str):
        return _explain(declined_by=g)
    c, ztop, r_in, r_out, d = g
    tA = n["finger"][0]
    if r_in < tA + n["gap_in"] + float(n["rc"]["rim_min_inner_mm"]):
        return _explain(declined_by="no_opening")
    if r_out - r_in > n["rim_wall"]:
        return _explain(declined_by="width")
    A = _pad_boxes(c[:2], d, r_in - n["gap_in"], ztop - n["rim_fd"], n["finger"], -1, n["col"])
    B = _pad_boxes(c[:2], d, r_out + n["gap"], ztop - n["rim_td"], n["thumb"], +1, n["col"])
    bad, hits = _clear_boxes(scene, [("pads", A[0]), ("column", A[1]), ("pads", B[0]), ("column", B[1])], n, thing)
    if bad:
        return _explain(declined_by=bad, blocker=_first_blocker(hits, thing), blockers=hits)
    plan = {"kind": "pinch", "rule": "rim", "thing": thing, "dir_deg": int(phi), "side": None,
            "theta_deg": float(phi), "width_mm": float(r_out - r_in), "r_in_mm": float(r_in),
            "r_out_mm": float(r_out),
            "pads": [{"role": "finger", "pad": _box_json(A[0]), "column": _box_json(A[1]), "close": 1,
                      "travel_mm": float(n["gap_in"] + n["over"])},
                     {"role": "thumb", "pad": _box_json(B[0]), "column": _box_json(B[1]), "close": -1,
                      "travel_mm": float(n["gap"] + n["over"])}],
            "frame": _frame_json((A[0][0] + B[0][0]) / 2, B[0][0] - A[0][0]),
            "envelope": envelope is not None}
    return _with_envelope(scene, plan, envelope, n, thing)


def _hook_rasters(scene, thing, n):
    """Per axis angle (hook_step_deg steps over [0, 180)): the raster and its through-openings.
    Yields (theta, d, lat, c, u, v, s, occ, lab, holes, u0, v0) with holes = [hole masks not touching the border]."""
    from scipy import ndimage
    rc = n["rc"]
    Pt = np.asarray(scene.things[thing].raster_points(), dtype=np.float64).reshape(-1, 3)
    c = Pt.mean(axis=0)
    cell = n["cell"]
    pad = float(rc["hook_border_mm"])
    out = []
    for th in np.arange(0, 180, n["hstep"]):
        d, lat = _dirs(float(th))
        R = Pt - c
        u = R[:, 0] * lat[0] + R[:, 1] * lat[1] + R[:, 2] * lat[2]
        v = Pt[:, 2]
        s = R[:, 0] * d[0] + R[:, 1] * d[1] + R[:, 2] * d[2]
        u0, v0 = u.min() - pad, v.min() - pad
        nu, nv = int((u.max() - u0 + pad) / cell) + 1, int((v.max() - v0 + pad) / cell) + 1
        occ = np.zeros((nu, nv), bool)
        iu_p, iv_p = ((u - u0) / cell).astype(int), ((v - v0) / cell).astype(int)
        occ[iu_p, iv_p] = True
        occ = ndimage.binary_closing(occ, iterations=int(rc["hook_closing_steps"]))
        lab, nl = ndimage.label(~occ)
        border = set(np.unique(np.concatenate([lab[0], lab[-1], lab[:, 0], lab[:, -1]])).tolist())
        holes = [(k, lab == k) for k in range(1, nl + 1) if k not in border]
        out.append((float(th), d, lat, c, u, v, s, occ, holes, u0, v0, iu_p, iv_p))
    return out


def _struct(bw, bh, r, cell):
    nu, nv = int((bw + 2 * r) / cell) + 1, int((bh + 2 * r) / cell) + 1
    return np.ones((max(nu, 1), max(nv, 1)), bool)


def _hook_candidates(scene, thing, n):
    """Every candidate site in search order: dicts with theta, the bar's centre in the raster, the opening's rim
    extent along the axis and the ranking value (the farthest distance to the opening's boundary, in cells)."""
    from scipy import ndimage
    rc = n["rc"]
    bw, bh = n["bar"]
    m = n["margin"]
    cell = n["cell"]
    cands = []
    for (th, d, lat, c, u, v, s, occ, holes, u0, v0, iu_p, iv_p) in _hook_rasters(scene, thing, n):
        for k, hole in holes:
            if hole.sum() < (bw + 2 * m) * (bh + 2 * m) / cell ** 2:
                continue
            er = ndimage.binary_erosion(hole, structure=_struct(bw, bh, m, cell))
            if not er.any():
                continue
            dt = ndimage.distance_transform_edt(hole)
            dt[~er] = -1
            iu, iv = np.unravel_index(np.argmax(dt), dt.shape)
            uc, vc = u0 + (iu + 0.5) * cell, v0 + (iv + 0.5) * cell
            ring = ndimage.binary_dilation(hole, iterations=int(round(float(rc["hook_ring_mm"]) / cell))) & occ
            onring = ring[iu_p, iv_p]
            if onring.sum() < int(rc["hook_min_ring_samples"]):
                continue
            cands.append({"theta": th, "d": d, "lat": lat, "c": c, "uc": float(uc), "vc": float(vc),
                          "s_lo": float(s[onring].min()), "s_hi": float(s[onring].max()),
                          "clear_cells": float(dt.max()), "hole_mm2": float(hole.sum() * cell ** 2)})
    return cands


def _hook_site(cand, side, n):
    bw, bh = n["bar"]
    d, lat, c = cand["d"], cand["lat"], cand["c"]
    s_lo, s_hi = cand["s_lo"], cand["s_hi"]
    if side > 0:
        a, b = s_lo - n["behind"], s_hi + n["beyond"]
    else:
        a, b = s_lo - n["beyond"], s_hi + n["behind"]
    R = np.stack([d, lat, np.array([0.0, 0.0, 1.0])], axis=1)
    cc = c + d * (a + b) / 2 + lat * cand["uc"]
    cc[2] = cand["vc"]
    bar = (cc, R, np.array([(b - a) / 2, bw / 2, bh / 2]))
    rc = n["rc"]
    ca = c + d * (s_lo + s_hi) / 2 + lat * cand["uc"]
    hh = float(rc["hook_check_height_mm"]) / 2
    ca[2] = cand["vc"] + bh / 2 + hh
    above = (ca, R, np.array([(s_hi - s_lo) / 2 + float(rc["hook_check_beyond_ring_mm"]), bw / 2, hh]))
    origin = c + d * (s_lo + s_hi) / 2 + lat * cand["uc"]
    origin[2] = cand["vc"]
    return bar, above, origin, side * d


def _plan_hook(scene, thing, side, n, envelope):
    sides = (1, -1) if int(side) == 0 else (int(side),)
    cands = _hook_candidates(scene, thing, n)
    if not cands:
        return _explain(declined_by="no_opening")
    bh = n["bar"][1]
    fc = float(n["rc"]["hook_floor_clear_mm"])
    ranked = []
    for i, cd in enumerate(cands):
        for j, sd in enumerate(sides):
            ranked.append((-cd["clear_cells"], i, j, cd, sd))
    ranked.sort(key=lambda r: (r[0], r[1], r[2]))
    first_bar = None
    first_env = None
    for _, _, _, cd, sd in ranked:
        bar, above, origin, xdir = _hook_site(cd, sd, n)
        if cd["vc"] - bh / 2 < fc:
            if first_bar is None:
                first_bar = ["plane"]
            continue
        hits = _box_hits(scene, bar, n["margin"], n["tmargin"], thing)
        if hits:
            if first_bar is None:
                first_bar = hits
            continue
        plan = {"kind": "hook", "rule": "hook", "thing": thing, "dir_deg": None, "side": int(sd),
                "theta_deg": float(cd["theta"]), "bar": _box_json(bar), "above": _box_json(above),
                "clear_cells": float(cd["clear_cells"]), "hole_mm2": float(cd["hole_mm2"]),
                "frame": _frame_json(origin, xdir), "envelope": envelope is not None}
        ex = _with_envelope(scene, plan, envelope, n, thing)
        if ex["plan"] is not None:
            return ex
        if first_env is None:
            first_env = ex
    if first_env is not None:
        return first_env
    return _explain(declined_by="bar", blocker=_first_blocker(first_bar or [], thing), blockers=first_bar or [])


def plan_explain(scene, thing, rule, dir_or_side, cfg, envelope):
    """{"plan": dict | None, "declined_by": None | "no_opening" | "width" | "pads" | "column" | "bar" | "envelope"
    | "absent", "blocker": key | "plane" | None, "blockers": [...]}: the decision of plan().
    rule: body | top | rim | hook; dir_or_side: dir_deg for the pinch rules, the side (-1 or +1; 0 = either side) for
    the hook. "absent": the target is not in the scene. "envelope" only when an envelope is given. blocker: the first
    thing (scene order) that blocks, a thing other than the target first; blockers: all."""
    if rule not in RULES:
        raise ValueError("unknown rule %r" % (rule,))
    if thing not in scene.things:
        return _explain(declined_by="absent")
    n = _num(cfg)
    if rule == "body":
        return _plan_pinch(scene, thing, "body", dir_or_side, n, envelope, "body")
    if rule == "top":
        return _plan_pinch(scene, thing, "top", dir_or_side, n, envelope, "top")
    if rule == "rim":
        return _plan_rim(scene, thing, dir_or_side, n, envelope)
    if rule == "hook":
        return _plan_hook(scene, thing, dir_or_side, n, envelope)
    raise ValueError("unknown rule %r" % (rule,))


def plan(scene, thing, rule, dir_or_side, cfg, envelope):
    """The request's plan on the scene (a JSON-serialisable dict), or None when it is declined."""
    return plan_explain(scene, thing, rule, dir_or_side, cfg, envelope)["plan"]


# ----------------------------------------------------------------------------------------------------- execution
def _result(outcome, by=None, hit=None):
    return {"outcome": outcome, "by": by, "hit": hit}


def execute(plan, truth_scene, offset_mm, cfg, envelope):
    """The plan executed in truth_scene with the hand placed off by offset_mm (mm):
    {"outcome": "declined" | "stop" | "miss" | "done", "by": "pad" | "column" | "bar" | "envelope" | None,
    "hit": key | "plane" | None}."""
    if plan is None:
        return _result("declined")
    if bool(plan.get("envelope")) != (envelope is not None):
        raise ValueError("the plan was made %s an envelope; execute it the same way"
                         % ("with" if plan.get("envelope") else "without"))
    n = _num(cfg)
    tol = n["tol"]
    off = np.asarray(offset_mm, dtype=np.float64).reshape(3)
    target = plan["thing"]
    if plan["kind"] == "hook":
        bar = _box_from(plan["bar"], off)
        hits = _box_hits(truth_scene, bar, tol, tol, target)
        if hits:
            return _result("stop", "bar", hits[0])
        if bar[0][2] - bar[2][2] < truth_scene.floor_z:
            return _result("stop", "bar", "plane")
        ehits = _envelope_hits(truth_scene, plan, envelope, tol, off) if envelope is not None else []
        if ehits:
            return _result("stop", "envelope", ehits[0])
        above = _box_from(plan["above"], off)
        sol = truth_scene.things.get(target)
        cnt = 0
        if sol is not None:
            lo, hi = _box_aabb(above, 0.0)
            Q = _index(sol).in_aabb(lo, hi)
            cnt = int(_in_box(Q, above, 0.0).sum()) if len(Q) else 0
        return _result("done" if cnt >= int(n["rc"]["hook_min_check_samples"]) else "miss")
    pads = [(p, _box_from(p["pad"], off), _box_from(p["column"], off)) for p in plan["pads"]]
    for p, pad, col in pads:
        hits = _box_hits(truth_scene, pad, tol, tol, target)
        if hits:
            return _result("stop", "pad", hits[0])
        if pad[0][2] - pad[2][2] < truth_scene.floor_z:
            return _result("stop", "pad", "plane")
        hits = _box_hits(truth_scene, col, tol, tol, target)
        if hits:
            return _result("stop", "column", hits[0])
    ehits = _envelope_hits(truth_scene, plan, envelope, tol, off) if envelope is not None else []
    if ehits:
        return _result("stop", "envelope", ehits[0])
    for p, pad, col in pads:
        c, R, half = pad
        travel = float(p["travel_mm"])
        grow = np.array([travel, 0.0, 0.0])
        lo, hi = _box_aabb((c, R, half + grow), 0.0)
        best, best_key = math.inf, None
        for k, sol in truth_scene.things.items():
            Q = _index(sol).in_aabb(lo, hi)
            if not len(Q):
                continue
            L = _local(Q, c, R)
            infoot = (np.abs(L[:, 1]) <= half[1]) & (np.abs(L[:, 2]) <= half[2])
            ahead = p["close"] * L[:, 0] - half[0]
            m = infoot & (ahead > 0) & (ahead <= travel)
            if m.any():
                a = float(ahead[m].min())
                if a < best:
                    best, best_key = a, k
        if best_key is None:
            return _result("miss")
        if best_key != target:
            return _result("miss", None, best_key)
    return _result("done")


# -------------------------------------------------------------------------------------------- opening clearances
def rim_clearance_mm(scene, thing, phi_deg, cfg):
    """The finger pad's inner clearance at the finger depth (mm): the room between the back of the inside pad,
    started `start_gap_inside_mm` from the inner wall, and the opening's centre, at azimuth phi_deg (the rim rule
    plans where this is at least `rim_min_inner_mm`). None when the target shows no rim there."""
    if thing not in scene.things:
        return None
    n = _num(cfg)
    g = _rim_geometry(scene, thing, phi_deg, n)
    if isinstance(g, str):
        return None
    r_in = g[2]
    return float(r_in - n["gap_in"] - n["finger"][0])


def _fits(hole, bw, bh, r, cell):
    """True when the bar's section grown by r (the same cell counts as the planner's opening test) fits inside the
    hole: a window of that many cells lies wholly in it (a separable minimum filter; the outside counts as wall)."""
    from scipy import ndimage
    nu, nv = _struct(bw, bh, r, cell).shape
    if nu > hole.shape[0] or nv > hole.shape[1]:
        return False
    return bool(ndimage.minimum_filter(hole.astype(np.uint8), size=(nu, nv), mode="constant", cval=0).any())


def hook_clearance_mm(scene, thing, side, cfg):
    """The bar's best raster clearance (mm): the largest r (in steps of half a raster cell) for which the bar's
    section grown by r on every side fits inside a through-opening of the target, over every axis of the hook's
    search (the hook rule needs r >= `margin_mm`). r may be negative (the opening is narrower than the bar). It is
    the same for both sides: the raster looks through the whole thing along the axis. None = no through-opening."""
    if thing not in scene.things:
        return None
    n = _num(cfg)
    bw, bh = n["bar"]
    cell = n["cell"]
    best = None
    for (th, d, lat, c, u, v, s, occ, holes, u0, v0, iu_p, iv_p) in _hook_rasters(scene, thing, n):
        for k, hole in holes:
            iu, iv = np.nonzero(hole)
            sub = np.zeros((iu.max() - iu.min() + 3, iv.max() - iv.min() + 3), dtype=bool)
            sub[iu - iu.min() + 1, iv - iv.min() + 1] = True
            lo = -int(math.ceil(max(bw, bh) / cell))           # r = lo * cell / 2: the section shrunk to one cell
            hi_ = int(math.ceil(2 * max(sub.shape) / cell)) + 2  # r = hi_ * cell / 2: wider than the hole's box
            if not _fits(sub, bw, bh, lo * cell / 2, cell):
                continue
            while hi_ - lo > 1:                                 # the largest step whose grown section still fits
                mid = (lo + hi_) // 2
                if _fits(sub, bw, bh, mid * cell / 2, cell):
                    lo = mid
                else:
                    hi_ = mid
            r = lo * cell / 2
            if best is None or r > best:
                best = r
    return None if best is None else float(best)


# ------------------------------------------------------------------------------------------------------ the lift
def _fmt(v):
    return " ".join("%.9g" % x for x in np.ravel(v))


def _mat2quat(R):
    import mujoco
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=np.float64).reshape(9))
    return q


def _tip(parent, tag, p, size, fric, L):
    import xml.etree.ElementTree as ET
    t, w, ln = size["thickness"] / 1000, size["width"] / 1000, size["length"] / 1000
    fw, fh = size["field"][0] / 1000, size["field"][1] / 1000
    proud = L["field_proud_mm"] / 1000
    s = int(p["close"])
    c = np.asarray(p["pad"]["c"], dtype=np.float64) / 1000.0
    R = np.asarray(p["pad"]["R"], dtype=np.float64)
    body = ET.SubElement(parent, "body", {"name": "%s_%s" % (_NAME, tag), "pos": _fmt(c), "quat": _fmt(_mat2quat(R))})
    ET.SubElement(body, "joint", {"name": "%s_j%s" % (_NAME, tag), "type": "slide", "axis": "%d 0 0" % s,
                                  "range": "0 %.5f" % (float(p["travel_mm"]) / 1000.0),
                                  "damping": "%.9g" % L["pad_damping"], "armature": "%.9g" % L["pad_armature"],
                                  "limited": "true"})
    m = L["tip_masses_kg"]
    common = dict(HAND_CONTACT)
    common.update({"friction": "%.4g %.9g %.9g" % (fric, L["pad_torsional"], L["pad_rolling"]),
                   "priority": "%d" % int(L["pad_priority"]), "contype": "1", "conaffinity": "1"})
    ET.SubElement(body, "geom", {"name": "%s_%s_field" % (_NAME, tag), "type": "box",
                                 "size": _fmt([proud / 2, fw / 2, fh / 2]),
                                 "pos": _fmt([s * (t / 2 - proud / 2), 0, 0]), "mass": "%.9g" % m["field"], **common})
    zb = -fh / 2
    ET.SubElement(body, "geom", {"name": "%s_%s_link" % (_NAME, tag), "type": "box",
                                 "size": _fmt([(t - proud) / 2, w / 2, ln / 2]),
                                 "pos": _fmt([-s * proud / 2, 0, zb + ln / 2]), "mass": "%.9g" % m["link"], **common})
    ztop = fh / 2 + L["column_above_field_mm"] / 1000
    if ztop > zb + ln:
        hh = (ztop - (zb + ln)) / 2
        ET.SubElement(body, "geom", {"name": "%s_%s_col" % (_NAME, tag), "type": "box", "size": _fmt([t / 2, w / 2, hh]),
                                     "pos": _fmt([0, 0, zb + ln + hh]), "mass": "%.9g" % m["column"], **common})


def lift(plan, model_xml, bodies, variant, offset_mm, cfg, held=()):
    """A dynamic grasp-and-lift trial of a pinch plan in MuJoCo, on a fresh MjModel / MjData compiled from model_xml
    plus the two fingertips: {"lifted": bool, "rise_mm": float, "reason": str}.

    model_xml: MJCF whose world frame is the frame the plan was made in (metres, z up, with its floor); its masses,
    frictions and options are used as they are (the trial does not change them). bodies: {key: body name}.
    variant: [mass_scale, friction]: the fingertip pads take `friction` (contact priority above the scene's);
    mass_scale is not applied here (a caller that wants it passes a model_xml whose masses carry it). held: keys whose
    bodies are welded to the world for the trial.
    Fingertips (the `lift` block of the config): each a link box behind its pad, a pad field proud on the closing
    face and a column above, on a slide joint along its closing direction, closed by a motor; both hang from a grip
    body on a slide joint along z with a large armature, driven to its position at every step. Timeline: settle with
    the pads open, close at `force_N`, rise `rise_mm` in `rise_s`, hold `hold_s`. Lifted: the target's centre of
    mass rose at least `lifted_min_mm` by the end, no pad at its travel limit, no fingertip contact at the start or
    before closing, a finite state."""
    import xml.etree.ElementTree as ET
    import mujoco
    if plan is None or plan.get("kind") != "pinch":
        return {"lifted": False, "rise_mm": 0.0, "reason": "no pinch plan"}
    L = cfg["lift"]
    tname = bodies.get(plan["thing"])
    root = ET.fromstring(model_xml)
    wb = root.find("worldbody")
    if wb is None or tname is None:
        return {"lifted": False, "rise_mm": 0.0, "reason": "the target's body is not in the model"}
    byname = {b.get("name"): b for b in wb.iter("body")}
    if tname not in byname:
        return {"lifted": False, "rise_mm": 0.0, "reason": "the target's body is not in the model"}
    for key in held:
        b = byname.get(bodies.get(key))
        if b is None:
            continue
        for j in list(b):
            if j.tag == "freejoint" or (j.tag == "joint" and j.get("type") == "free"):
                b.remove(j)
    off = np.asarray(offset_mm, dtype=np.float64).reshape(3) / 1000.0
    grip = ET.SubElement(wb, "body", {"name": "%s_grip" % _NAME, "pos": _fmt(off), "quat": "1 0 0 0"})
    ET.SubElement(grip, "joint", {"name": "%s_jz" % _NAME, "type": "slide", "axis": "0 0 1",
                                  "armature": "%.9g" % L["lift_armature"], "damping": "0", "limited": "false"})
    gb = L["grip_base"]
    ET.SubElement(grip, "geom", {"name": "%s_base" % _NAME, "type": "sphere", "size": "%.9g" % (gb["radius_mm"] / 1000),
                                 "mass": "%.9g" % gb["mass_kg"], "contype": "0", "conaffinity": "0"})
    fric = float(variant[1]) if L.get("pad_friction", "variant") == "variant" else float(L["pad_friction"])
    _tip(grip, "A", plan["pads"][0], L["finger_tip_mm"], fric, L)
    _tip(grip, "B", plan["pads"][1], L["thumb_tip_mm"], fric, L)
    act = root.find("actuator")
    if act is None:
        act = ET.SubElement(root, "actuator")
    lo, hi = (float(x) for x in L["motor_ctrlrange_N"])
    for tag in ("A", "B"):
        ET.SubElement(act, "motor", {"name": "%s_a%s" % (_NAME, tag), "joint": "%s_j%s" % (_NAME, tag), "gear": "1",
                                     "ctrlrange": "%.9g %.9g" % (lo, hi), "ctrllimited": "true"})
    m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    bt = m.body(tname).id
    hand = {m.body("%s_A" % _NAME).id, m.body("%s_B" % _NAME).id}
    jz = m.joint("%s_jz" % _NAME).id
    qz, vz = m.jnt_qposadr[jz], m.jnt_dofadr[jz]
    aA, aB = m.actuator("%s_aA" % _NAME).id, m.actuator("%s_aB" % _NAME).id
    qA = m.jnt_qposadr[m.joint("%s_jA" % _NAME).id]
    qB = m.jnt_qposadr[m.joint("%s_jB" % _NAME).id]

    def hand_contacts():
        out = 0
        for i in range(d.ncon):
            c = d.contact[i]
            if m.geom_bodyid[c.geom1] in hand or m.geom_bodyid[c.geom2] in hand:
                out += 1
        return out

    start_con = hand_contacts()
    dt = m.opt.timestep
    t_set, t_close, t_rise, t_hold = (float(L[k]) for k in ("t_settle_s", "t_close_s", "rise_s", "hold_s"))
    rise_m = float(L["rise_mm"]) / 1000.0
    force = float(L["force_N"])
    nst = int(round((t_set + t_close + t_rise + t_hold) / dt))
    i_set = int(round(t_set / dt))
    i_lift = int(round((t_set + t_close) / dt))
    pre_close = 0
    x_start = d.xipos[bt].copy()
    nan = False
    for i in range(nst):
        d.ctrl[aA] = 0.0 if i < i_set else force
        d.ctrl[aB] = 0.0 if i < i_set else force
        tl = (i - i_lift) * dt
        if i < i_lift:
            d.qpos[qz] = 0.0
            d.qvel[vz] = 0.0
        elif tl <= t_rise:
            d.qpos[qz] = rise_m * tl / t_rise
            d.qvel[vz] = rise_m / t_rise
        else:
            d.qpos[qz] = rise_m
            d.qvel[vz] = 0.0
        mujoco.mj_step(m, d)
        if i + 1 == i_set:
            x_start = d.xipos[bt].copy()
            pre_close = hand_contacts()
        if not np.all(np.isfinite(d.qpos)):
            nan = True
            break
    rise = float((d.xipos[bt][2] - x_start[2]) * 1000.0) if not nan else 0.0
    trA = float(plan["pads"][0]["travel_mm"]) / 1000.0
    trB = float(plan["pads"][1]["travel_mm"]) / 1000.0
    at_limit = bool(d.qpos[qA] > trA - 1e-4 or d.qpos[qB] > trB - 1e-4) if not nan else False
    if nan:
        reason = "non-finite state"
    elif start_con:
        reason = "fingertip contact at the start"
    elif pre_close:
        reason = "fingertip contact before closing"
    elif at_limit:
        reason = "a pad reached its travel limit"
    elif rise < float(L["lifted_min_mm"]):
        reason = "rose %.1f mm" % rise
    else:
        reason = "lifted"
    return {"lifted": reason == "lifted", "rise_mm": rise, "reason": reason}


# -------------------------------------------------------------------------------------------------- rehearsal
def rehearse(twin_scene, requests, cfg, envelope, lifts=False, model_xml=None, bodies=None, variant=(1.0, 0.6),
             held=()):
    """Plan and execute every request on the twin itself, at every placement of cfg["placements_mm"]:
    [{"id", "thing", "rule", "planned", "declined_by", "blocker", "outcomes": [...], "by": [...], "hit": [...]}],
    and, with lifts=True (model_xml and bodies given), "lift": the lift trial of every planned body request at the
    centre placement under `variant` with the `held` keys welded (None for the other requests)."""
    out = []
    for r in requests:
        key = r["side"] if r["rule"] == "hook" else r["dir_deg"]
        ex = plan_explain(twin_scene, r["thing"], r["rule"], key, cfg, envelope)
        res = [execute(ex["plan"], twin_scene, o, cfg, envelope) for o in cfg["placements_mm"]]
        row = {"id": r["id"], "thing": r["thing"], "rule": r["rule"], "planned": ex["plan"] is not None,
               "declined_by": ex["declined_by"], "blocker": ex["blocker"],
               "outcomes": [x["outcome"] for x in res], "by": [x["by"] for x in res], "hit": [x["hit"] for x in res]}
        if lifts:
            row["lift"] = None
            if model_xml is not None and ex["plan"] is not None and r["rule"] == "body":
                row["lift"] = lift(ex["plan"], model_xml, bodies or {}, list(variant), [0.0, 0.0, 0.0], cfg, held)
        out.append(row)
    return out
