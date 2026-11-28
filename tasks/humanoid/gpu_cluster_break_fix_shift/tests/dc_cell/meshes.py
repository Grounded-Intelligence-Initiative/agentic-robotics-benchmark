"""Procedural meshes of the data-hall cell (cell internals): every mesh is (vertices (n, 3), faces (m, 3) int32,
normals (n, 3), texture coordinates (n, 2)) in metres, made in code and deterministic.

The general generators (rounded_box, lathe, tube, transform, merge, scale_uv) are house_cleaning_with_tools's
(tests/house_cell/props.py), copied unchanged; the hall's own shapes (quads, plates, handles, fin stacks, cable runs)
follow below them.
"""

from __future__ import annotations

import math

import numpy as np

def _normalize(v):
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, 1e-12)


def rounded_box(half, radius=0.0, n=6):
    """A box of half sizes `half` with rounded edges and corners (radius, clipped to the smallest half), as a
    subdivided cube projected onto the rounded shape. Box-projected texture coordinates: each face maps its two
    in-plane axes onto [0, 1] (u along the first remaining axis), scaled by the face's size so a texture keeps its
    aspect across faces of different sizes (one texture unit = 1 m)."""
    h = np.asarray(half, float)
    r = float(min(radius, 0.999 * h.min()))
    core = h - r
    V, F, N, UV = [], [], [], []
    for axis in range(3):
        for sgn in (-1.0, 1.0):
            u_ax, v_ax = [a for a in range(3) if a != axis]
            g = np.linspace(-1.0, 1.0, n + 1)
            uu, vv = np.meshgrid(g, g, indexing="ij")
            P = np.zeros((n + 1, n + 1, 3))
            P[..., axis] = sgn * h[axis]
            P[..., u_ax] = uu * h[u_ax]
            P[..., v_ax] = vv * h[v_ax]
            P = P.reshape(-1, 3)
            if r > 0:
                c = np.clip(P, -core, core)
                d = _normalize(P - c)
                P = c + r * d
                nrm = d
            else:
                nrm = np.zeros_like(P)
                nrm[:, axis] = sgn
            base = len(np.concatenate(V)) if V else 0
            V.append(P)
            N.append(nrm)
            UV.append(np.stack([(P[:, u_ax] + h[u_ax]), (P[:, v_ax] + h[v_ax])], axis=1))
            for i in range(n):
                for j in range(n):
                    a = base + i * (n + 1) + j
                    b, c_, d_ = a + (n + 1), a + (n + 1) + 1, a + 1
                    tri = [(a, b, c_), (a, c_, d_)]
                    # outward winding: the face normal along +sgn of `axis`
                    for t in tri:
                        p0, p1, p2 = P[t[0] - base], P[t[1] - base], P[t[2] - base]
                        if np.dot(np.cross(p1 - p0, p2 - p0), np.eye(3)[axis] * sgn) < 0:
                            t = (t[0], t[2], t[1])
                        F.append(t)
    V = np.concatenate(V)
    N = np.concatenate(N)
    UV = np.concatenate(UV)
    return _weld(V, np.array(F, np.int32), N, UV)


def _weld(V, F, N, UV, tol=1e-7):
    """Merge duplicate vertices (same position, normal and texture coordinate)."""
    key = np.round(np.concatenate([V, N, UV], axis=1) / tol).astype(np.int64)
    _, idx, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    return V[idx], inv.reshape(-1)[F].astype(np.int32), N[idx], UV[idx]


def lathe(profile, segments=40, cap_bottom=False, cap_top=False, phase=0.0):
    """Revolve a profile [(r, z), ...] (bottom to top, the outside of the shape; r >= 0) about z. Texture
    coordinates: u = angle / 2 pi, v = arc length along the profile (m). Smooth normals along the profile, hard
    creases where the profile turns by more than 50 degrees (the vertex is duplicated)."""
    prof = np.asarray(profile, float)
    # split the profile at sharp corners so each run gets its own normals
    runs, cur = [], [prof[0]]
    for i in range(1, len(prof)):
        cur.append(prof[i])
        if 0 < i < len(prof) - 1:
            a = prof[i] - prof[i - 1]
            b = prof[i + 1] - prof[i]
            cosang = float(np.dot(a, b) / max(1e-12, np.linalg.norm(a) * np.linalg.norm(b)))
            if cosang < math.cos(math.radians(50.0)):
                runs.append(np.array(cur))
                cur = [prof[i]]
    runs.append(np.array(cur))
    V, F, N, UV = [], [], [], []
    ang = phase + np.linspace(0.0, 2 * math.pi, segments + 1)
    s_off = 0.0
    for run in runs:
        seg = np.diff(run, axis=0)
        L = np.linalg.norm(seg, axis=1)
        s = np.concatenate([[0.0], np.cumsum(L)]) + s_off
        s_off = s[-1]
        # 2D normals of the profile (outward: rotate the tangent by -90 deg in the (r, z) plane)
        t = np.zeros_like(run)
        t[1:-1] = run[2:] - run[:-2]
        t[0] = run[1] - run[0]
        t[-1] = run[-1] - run[-2]
        t = t / np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-12)
        n2 = np.stack([t[:, 1], -t[:, 0]], axis=1)
        base = sum(len(v) for v in V)
        P = np.zeros((len(run), segments + 1, 3))
        Nn = np.zeros_like(P)
        P[..., 0] = run[:, :1] * np.cos(ang)[None, :]
        P[..., 1] = run[:, :1] * np.sin(ang)[None, :]
        P[..., 2] = run[:, 1:2]
        Nn[..., 0] = n2[:, :1] * np.cos(ang)[None, :]
        Nn[..., 1] = n2[:, :1] * np.sin(ang)[None, :]
        Nn[..., 2] = n2[:, 1:2]
        U = np.broadcast_to(ang[None, :] / (2 * math.pi), (len(run), segments + 1))
        Vv = np.broadcast_to(s[:, None], (len(run), segments + 1))
        V.append(P.reshape(-1, 3))
        N.append(Nn.reshape(-1, 3))
        UV.append(np.stack([U, Vv], axis=-1).reshape(-1, 2))
        w = segments + 1
        for i in range(len(run) - 1):
            for j in range(segments):
                a = base + i * w + j
                F.append((a, a + 1, a + w + 1))
                F.append((a, a + w + 1, a + w))
    V, N, UV = np.concatenate(V), np.concatenate(N), np.concatenate(UV)
    F = list(F)
    for cap, idx in ((cap_bottom, 0), (cap_top, -1)):
        if not cap:
            continue
        r, z = prof[idx]
        if r < 1e-6:
            continue
        nz = -1.0 if idx == 0 else 1.0
        base = len(V)
        ring = np.stack([r * np.cos(ang), r * np.sin(ang), np.full_like(ang, z)], axis=1)
        V = np.concatenate([V, [[0.0, 0.0, z]], ring])
        N = np.concatenate([N, np.tile([0.0, 0.0, nz], (len(ring) + 1, 1))])
        UV = np.concatenate([UV, [[0.5, 0.5]], 0.5 + 0.5 * ring[:, :2] / max(r, 1e-9)])
        for j in range(segments):
            tri = (base, base + 1 + j, base + 2 + j)
            F.append(tri if nz > 0 else (tri[0], tri[2], tri[1]))
    return V, np.array(F, np.int32), N, UV


def tube(path, radius, segments=16, cap=True):
    """A tube of `radius` along a polyline `path` (k, 3), with parallel-transported frames; u around, v along (m)."""
    P = np.asarray(path, float)
    T = np.zeros_like(P)
    T[1:-1] = P[2:] - P[:-2]
    T[0] = P[1] - P[0]
    T[-1] = P[-1] - P[-2]
    T = _normalize(T)
    ref = np.array([0.0, 0.0, 1.0]) if abs(T[0][2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    n0 = _normalize(np.cross(T[0], ref))
    frames = [n0]
    for i in range(1, len(P)):
        n = frames[-1] - np.dot(frames[-1], T[i]) * T[i]
        frames.append(_normalize(n))
    ang = np.linspace(0.0, 2 * math.pi, segments + 1)
    V, N, UV, F = [], [], [], []
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    for i in range(len(P)):
        nrm = frames[i]
        bin_ = np.cross(T[i], nrm)
        for a in ang:
            d = math.cos(a) * nrm + math.sin(a) * bin_
            V.append(P[i] + radius * d)
            N.append(d)
            UV.append((a / (2 * math.pi), s[i]))
    w = segments + 1
    for i in range(len(P) - 1):
        for j in range(segments):
            a = i * w + j
            F.append((a, a + w, a + w + 1))
            F.append((a, a + w + 1, a + 1))
    V, N, UV = np.array(V), np.array(N), np.array(UV)
    if cap:
        for end, sgn in ((0, -1.0), (len(P) - 1, 1.0)):
            base = len(V)
            V = np.concatenate([V, [P[end]], V[end * w:(end + 1) * w]])
            N = np.concatenate([N, np.tile(T[end] * sgn, (w + 1, 1))])
            UV = np.concatenate([UV, np.full((w + 1, 2), 0.5)])
            for j in range(segments):
                tri = (base, base + 1 + j, base + 2 + j)
                F.append(tri if sgn < 0 else (tri[0], tri[2], tri[1]))
    return V, np.array(F, np.int32), N, UV


def transform(mesh, R=None, t=None):
    V, F, N, UV = mesh
    R = np.eye(3) if R is None else np.asarray(R, float)
    t = np.zeros(3) if t is None else np.asarray(t, float)
    return V @ R.T + t, F, N @ R.T, UV


def merge(*meshes):
    V, F, N, UV, off = [], [], [], [], 0
    for m in meshes:
        V.append(m[0])
        F.append(m[1] + off)
        N.append(m[2])
        UV.append(m[3])
        off += len(m[0])
    return np.concatenate(V), np.concatenate(F).astype(np.int32), np.concatenate(N), np.concatenate(UV)


def scale_uv(mesh, su, sv):
    V, F, N, UV = mesh
    return V, F, N, UV * np.array([su, sv])


# ================================================================================================ the hall's shapes
AXES = {"+x": (0, 1.0), "-x": (0, -1.0), "+y": (1, 1.0), "-y": (1, -1.0), "+z": (2, 1.0), "-z": (2, -1.0)}


def box(half):
    """A plain box (rounded_box without rounding): box-projected texture coordinates, one unit = 1 m."""
    return rounded_box(half, 0.0, n=1)


def quad(w, h, facing="+y", u0=0.0, u1=1.0, v0=0.0, v1=1.0):
    """One rectangle w x h centred on the origin facing `facing`; texture coordinates (u0..u1, v0..v1) run left to
    right and bottom to top as a viewer facing it sees it (so a texture image appears upright)."""
    ax, sg = AXES[facing]
    # the face's right and up directions as seen by a viewer looking at it (against the normal)
    if ax == 2:
        right, up = np.array([sg * 1.0, 0, 0]) * (1 if sg > 0 else -1), np.array([0, 1.0, 0])
        right = np.array([1.0, 0, 0])
        up = np.array([0, 1.0, 0]) if sg > 0 else np.array([0, -1.0, 0])
    else:
        up = np.array([0, 0, 1.0])
        n = np.zeros(3)
        n[ax] = sg
        right = np.cross(up, n)       # viewer looks along -n: right = up x n
    n = np.zeros(3)
    n[ax] = sg
    c = [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]
    V = np.array([a * right + b * up for a, b in c])
    UV = np.array([(u0, v0), (u1, v0), (u1, v1), (u0, v1)], float)
    F = np.array([(0, 1, 2), (0, 2, 3)], np.int32)
    if np.dot(np.cross(V[1] - V[0], V[2] - V[0]), n) < 0:
        F = F[:, ::-1].copy()
    return V, F, np.tile(n, (4, 1)), UV


def panel(w, h, t, facing="+y", edge_uv=(0.002, 0.002)):
    """A plate w x h, t thick, whose `facing` face carries the whole texture (0..1) and whose back and edges sample a
    single texel near `edge_uv` (a uniform colour taken from the texture's corner)."""
    ax, sg = AXES[facing]
    half = [0.0, 0.0, 0.0]
    if ax == 2:
        half = [w / 2, h / 2, t / 2]
    elif ax == 1:
        half = [w / 2, t / 2, h / 2]
    else:
        half = [t / 2, w / 2, h / 2]
    V, F, N, UV = box(half)
    UV = np.tile(np.asarray(edge_uv, float), (len(V), 1))
    front = quad(w, h, facing)
    off = np.zeros(3)
    off[ax] = sg * (t / 2 + 1e-5)
    front = transform(front, t=off)
    return merge((V, F, N, UV), front)


def catmull_rom(points, samples=8):
    """A smooth polyline through control points (Catmull-Rom, `samples` per span)."""
    P = np.asarray(points, float)
    if len(P) < 3:
        return P
    ext = np.concatenate([[2 * P[0] - P[1]], P, [2 * P[-1] - P[-2]]])
    out = []
    for i in range(1, len(ext) - 2):
        p0, p1, p2, p3 = ext[i - 1], ext[i], ext[i + 1], ext[i + 2]
        for s in np.linspace(0.0, 1.0, samples, endpoint=False):
            s2, s3 = s * s, s * s * s
            out.append(0.5 * ((2 * p1) + (-p0 + p2) * s + (2 * p0 - 5 * p1 + 4 * p2 - p3) * s2
                              + (-p0 + 3 * p1 - 3 * p2 + p3) * s3))
    out.append(P[-1])
    return np.array(out)


def cable(points, radius, samples=6, segments=8):
    """A cable through control points: a Catmull-Rom spline swept by a circle (drawn only: cables are not simulated)."""
    return tube(catmull_rom(points, samples), radius, segments=segments, cap=True)


def bar_handle(length, standoff, radius, axis="z"):
    """A U-shaped pull handle: a bar of `length` along `axis`, held `standoff` off the mounting face (+y), with two
    legs; centred on the mounting face."""
    L, s = length / 2, standoff
    if axis == "z":
        pts = [(0, 0, -L), (0, s * 0.85, -L), (0, s, -L + s * 0.3), (0, s, L - s * 0.3), (0, s * 0.85, L), (0, 0, L)]
    else:
        pts = [(-L, 0, 0), (-L, s * 0.85, 0), (-L + s * 0.3, s, 0), (L - s * 0.3, s, 0), (L, s * 0.85, 0), (L, 0, 0)]
    return tube(catmull_rom(pts, 5), radius, segments=10, cap=True)


def fin_stack(width, depth, height, n, thickness, base=0.004):
    """A heat-sink: `n` vertical fins (thickness along x) across `width`, each `depth` x `height`, on a base plate."""
    parts = [box((width / 2, depth / 2, base / 2))]
    pitch = width / n
    for i in range(n):
        x = -width / 2 + pitch * (i + 0.5)
        parts.append(transform(box((thickness / 2, depth / 2, height / 2)), t=(x, 0.0, base / 2 + height / 2)))
    return merge(*parts)


def extrude(poly, depth, axis="y"):
    """A prism: a 2-D polygon (convex or star-shaped about its centroid; (a, b) in the plane normal to `axis`)
    extruded `depth` along `axis` (centred)."""
    P = np.asarray(poly, float)
    c = P.mean(axis=0)
    k = len(P)
    V, F, N, UV = [], [], [], []

    def to3(a, b, d):
        if axis == "y":
            return (a, d, b)
        if axis == "x":
            return (d, a, b)
        return (a, b, d)

    for sgn in (-1.0, 1.0):
        base = len(V)
        V.append(to3(c[0], c[1], sgn * depth / 2))
        for a, b in P:
            V.append(to3(a, b, sgn * depth / 2))
        nn = to3(0.0, 0.0, sgn)
        N += [nn] * (k + 1)
        UV += [(0.5, 0.5)] + [(a, b) for a, b in P]
        for i in range(k):
            tri = (base, base + 1 + i, base + 1 + (i + 1) % k)
            F.append(tri)
    for i in range(k):
        a0, b0 = P[i]
        a1, b1 = P[(i + 1) % k]
        e = np.array([a1 - a0, b1 - b0])
        nrm2 = np.array([e[1], -e[0]])
        nrm2 /= max(1e-12, np.linalg.norm(nrm2))
        if np.dot(nrm2, (P[i] + P[(i + 1) % k]) / 2 - c) < 0:
            nrm2 = -nrm2
        base = len(V)
        V += [to3(a0, b0, -depth / 2), to3(a1, b1, -depth / 2), to3(a1, b1, depth / 2), to3(a0, b0, depth / 2)]
        nn = to3(nrm2[0], nrm2[1], 0.0)
        N += [nn] * 4
        L = float(np.linalg.norm(e))
        UV += [(0, 0), (L, 0), (L, depth), (0, depth)]
        F += [(base, base + 1, base + 2), (base, base + 2, base + 3)]
    V, F, N, UV = np.array(V, float), np.array(F, np.int32), np.array(N, float), np.array(UV, float)
    # orient every face by its normal
    tri = V[F]
    fn = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    flip = (fn * N[F[:, 0]]).sum(1) < 0
    F[flip] = F[flip][:, ::-1]
    return V, F, N, UV
