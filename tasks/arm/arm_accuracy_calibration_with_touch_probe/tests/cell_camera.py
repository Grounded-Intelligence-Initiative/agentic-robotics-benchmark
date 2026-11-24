"""cell_camera.py: how the wrist camera forms its images. HIDDEN: the agent does not have this file.

camera.md (public) describes the camera in words: resolution, nominal focal length and its
tolerance, bounds of the principal point and the distortion, depth of field, lamps, surfaces,
noise level, automatic exposure. Everything below that is a number or a law is hidden, and the
per-unit values are drawn from their own hashed streams ("optics", "lamps", "materials",
"sensor"), so that the arm's draw of calib_cell.py is unchanged.

The image (one per call of `render`):
  1. geometry: pinhole with radial distortion (Brown, k1 and k2) about the principal point; a
     pixel's ray is found by inverting the distortion at its centre (fixed-point iterations);
     near the outlines of spheres and posts every pixel is supersampled (Hammersley patterns,
     the sample rays from the distortion linearised at the pixel centre);
  2. scene: the table (z = 0, a square of side 2 TABLE_HALF, a two-octave value-noise grain),
     the floor below it, the walls and the ceiling of the room; the gauge plate (a bright, finely
     textured block, part of the background: its blind bores blackened, their inside lit by a lamp
     only where the lamp is seen through the mouth, its three dark shims under it; supersampled
     wherever its shading has an edge; it hides the spheres and posts behind it); the twelve datum
     spheres (semi-gloss: diffuse plus a Blinn-Phong lobe), their posts (dark, slightly glossy). Two
     or three point lamps at per-unit positions above the cell, inverse-square, plus ambient;
  3. optics: each sphere with its post is a layer at the sphere's depth; the background is
     blurred with a space-variant Gaussian (two blurred copies mixed by the depth of what each
     pixel sees), every layer with its own Gaussian (base blur and defocus by the depth of the
     sphere's centre), composited back to front (alpha); then a wide veiling-glare halo;
  4. sensor: vignetting, automatic exposure (from the noise-free image: its mean and a
     high percentile, on a sub-sampled grid), shot and read noise, black level, 8-bit quantisation,
     fixed hot and dead pixels.
The pixel values come from elementwise arithmetic, square roots, floor, sorting and integer hashing
(the blur taps are rounded to 2^-36); the layers' bounding boxes use a small matrix inverse, which
decides only where pixels are computed, not their values. Given the same camera pose, an image is
the same on every machine; the pose itself comes from the arm's kinematics (sines and cosines).
"""
import hashlib
import math

import numpy as np

from cell_data import IMAGE_W, IMAGE_H, F_PX_NOMINAL, R_SPHERE, R_POST
from cell_data import PLATE_SIZE, BORES, BORE_RADIUS, BORE_DEPTH, SHIM_RADIUS
# public bounds (cell_data.py, camera.md): each unit's value is uniform inside them (hidden draw)
from cell_data import F_TOL, PP_TOL, DISTORTION_MAX_PX as DIST_MAX_PX

# ---- hidden constants -----------------------------------------------------------------------------
# optics (per unit, uniform in these ranges)
DIST_SHAPE = (-0.5, 1.0)         # k2 rc^2 / k1 (rc: the farthest corner): monotone displacement
BLUR0_RANGE = (0.55, 0.85)       # px: base lens blur (Gaussian sigma)
APERTURE_RANGE = (1.1, 1.9)      # px m: defocus sigma = this * |1/z - 1/z_focus|
FOCUS_RANGE = (0.30, 0.40)       # m: focus distance (along the optical axis)
HALO_FRAC_RANGE = (0.01, 0.04)   # fraction of the light in the veiling glare
HALO_SIGMA = 18.0                # px
LAYER_BLUR_MAX = 8.0             # px: cap of a layer's blur (its sphere's centre within ~0.12 m of the camera's plane)
# lamps
N_LAMPS = (2, 3)
LAMP_HEIGHT = (1.3, 2.3)         # m above the table
LAMP_RADIUS = (0.2, 1.8)         # m from the cell's vertical axis
LAMP_POWER = (0.55, 1.0)         # times LAMP_REF^2: irradiance 0.55..1 at LAMP_REF m
LAMP_REF = 1.8
AMBIENT_RANGE = (0.06, 0.16)
# materials
SPHERE_DIFFUSE = (0.62, 0.78)
SPHERE_SPEC = (0.20, 0.40)
SPHERE_SHININESS = 32            # Blinn-Phong exponent (a power of two: squarings only)
POST_DIFFUSE = (0.03, 0.06)
POST_SPEC = 0.10
POST_SHININESS = 16
TABLE_ALBEDO = (0.14, 0.26)
TABLE_GRAIN = (0.06, 0.12)       # relative amplitude of the fine grain (6 mm)
TABLE_MOTTLE = (0.08, 0.15)      # ... of the coarse mottle (35 mm)
TABLE_HALF = 1.0                 # m
FLOOR_Z, FLOOR_ALBEDO = -0.8, 0.10
ROOM_HALF, ROOM_HEIGHT = 3.0, 2.6
WALL_ALBEDO = (0.18, 0.35)
WALL_PATTERN = 0.35              # relative amplitude of the walls' pattern (0.25 m scale)
# sensor
VIGNETTE_RANGE = (0.5, 1.4)      # V = 1 / (1 + v rho^2)^2, rho = distance from the principal point / f
FULL_WELL = 9000.0               # electrons at grey 255 (above black)
READ_NOISE = 10.0                # electrons
BLACK_RANGE = (3.0, 10.0)        # grey levels
AE_MEAN = (60.0, 85.0)           # grey: the exposure aims at this mean ...
AE_HIGH = 235.0                  # ... unless this percentile of the image would exceed this grey
AE_HIGH_FRACTION = 0.997
GAIN_LIMITS = (20.0, 3000.0)
N_HOT, N_DEAD = (3, 8), (2, 6)
# the gauge plate (cell_data.py: its certificate; calib_cell.draw_plate: its pose)
PLATE_ALBEDO = (0.38, 0.55)      # fine-machined, bright grey: diffuse ...
PLATE_SPEC = (0.03, 0.08)        # ... with a weak glossy lobe
PLATE_SHININESS = 16
PLATE_GRAIN = (0.02, 0.05)       # relative amplitude of the machining texture (2 mm)
BORE_ALBEDO = (0.02, 0.05)       # blackened bores
BORE_AMBIENT = 0.35              # fraction of the room's light that reaches into a bore
SHIM_ALBEDO = 0.08
SS_PLATE = 16                    # samples per pixel along the plate's outlines
SS_FINE_N, SS_COARSE_N = 64, 16
BAND_SPHERE, BAND_POST = 3.0, 3.0      # px from an outline: supersampled


def _rng(*key):
    digest = hashlib.blake2b(repr(key).encode(), digest_size=8, person=b"calibcell").digest()
    return np.random.default_rng(int.from_bytes(digest, "big"))


def _hammersley(n):
    m = n.bit_length() - 1
    i = np.arange(n)
    rev = np.zeros(n, dtype=np.int64)
    for b in range(m):
        rev |= ((i >> b) & 1) << (m - 1 - b)
    return (i + 0.5) / n - 0.5, (rev + 0.5) / n - 0.5


SS_FINE = _hammersley(SS_FINE_N)
SS_COARSE = _hammersley(SS_COARSE_N)


# ---- per-unit draws ---------------------------------------------------------------------------
def draw_camera(seed):
    """The unit's camera, lamps, materials and sensor (hidden)."""
    o = _rng("optics", seed)
    f = F_PX_NOMINAL * (1.0 + o.uniform(-F_TOL, F_TOL))
    cx = (IMAGE_W - 1) / 2.0 + o.uniform(-PP_TOL, PP_TOL)
    cy = (IMAGE_H - 1) / 2.0 + o.uniform(-PP_TOL, PP_TOL)
    # radial distortion: the displacement at the farthest corner (either sign, up to the documented
    # bound) and the shape k2 rc^2 / k1 (the displacement keeps one sign and grows with the radius)
    rc = max(np.hypot(u - cx, v - cy) for u in (0, IMAGE_W - 1) for v in (0, IMAGE_H - 1)) / f
    disp = o.uniform(0.25, 1.0) * DIST_MAX_PX * o.choice([-1.0, 1.0])
    eta = o.uniform(*DIST_SHAPE)
    k1 = disp / (f * rc ** 3 * (1.0 + eta))
    k2 = eta * k1 / rc ** 2
    optics = dict(f=f, cx=cx, cy=cy, k1=k1, k2=k2, blur0=o.uniform(*BLUR0_RANGE),
                  aperture=o.uniform(*APERTURE_RANGE), focus=o.uniform(*FOCUS_RANGE),
                  halo=o.uniform(*HALO_FRAC_RANGE))
    lr = _rng("lamps", seed)
    n = int(lr.integers(N_LAMPS[0], N_LAMPS[1] + 1))
    az = lr.uniform(0, 2 * np.pi, n)
    rad = lr.uniform(*LAMP_RADIUS, n)
    pos = np.stack([rad * np.cos(az), rad * np.sin(az), lr.uniform(*LAMP_HEIGHT, n)], axis=1)
    power = lr.uniform(*LAMP_POWER, n) * LAMP_REF ** 2
    lamps = dict(pos=pos, power=power, ambient=lr.uniform(*AMBIENT_RANGE))
    m = _rng("materials", seed)
    mat = dict(sphere_d=m.uniform(*SPHERE_DIFFUSE), sphere_s=m.uniform(*SPHERE_SPEC),
               post_d=m.uniform(*POST_DIFFUSE), table=m.uniform(*TABLE_ALBEDO),
               grain=m.uniform(*TABLE_GRAIN), mottle=m.uniform(*TABLE_MOTTLE),
               wall=m.uniform(*WALL_ALBEDO), hash=int(m.integers(1, 2 ** 31 - 1)))
    pm = _rng("plate_material", seed)        # its own stream: the other draws are unchanged
    mat.update(plate_d=pm.uniform(*PLATE_ALBEDO), plate_s=pm.uniform(*PLATE_SPEC), plate_grain=pm.uniform(*PLATE_GRAIN),
               bore_d=pm.uniform(*BORE_ALBEDO), plate_hash=int(pm.integers(1, 2 ** 31 - 1)))
    s = _rng("sensor", seed)
    nh, nd = int(s.integers(N_HOT[0], N_HOT[1] + 1)), int(s.integers(N_DEAD[0], N_DEAD[1] + 1))
    pix = s.choice(IMAGE_W * IMAGE_H, size=nh + nd, replace=False)
    sensor = dict(vignette=s.uniform(*VIGNETTE_RANGE), black=s.uniform(*BLACK_RANGE),
                  ae_mean=s.uniform(*AE_MEAN), hot=pix[:nh], hot_val=s.integers(60, 256, nh),
                  dead=pix[nh:])
    return dict(optics=optics, lamps=lamps, materials=mat, sensor=sensor)


# ---- geometry -----------------------------------------------------------------------------------
def undistort(xd, yd, k1, k2, iters=10):
    """Ideal normalised coordinates of distorted normalised ones (x_d = x (1 + k1 r^2 + k2 r^4))."""
    x, y = xd, yd
    for _ in range(iters):
        r2 = x * x + y * y
        s = 1.0 + r2 * (k1 + k2 * r2)
        x, y = xd / s, yd / s
    return x, y


def distort(x, y, k1, k2):
    r2 = x * x + y * y
    s = 1.0 + r2 * (k1 + k2 * r2)
    return x * s, y * s


def project(op, P):
    """Pixel (u, v) of camera-frame points P (..., 3) (the truth, for studies)."""
    x, y = distort(P[..., 0] / P[..., 2], P[..., 1] / P[..., 2], op["k1"], op["k2"])
    return op["cx"] + op["f"] * x, op["cy"] + op["f"] * y


class Grid:
    """The unit's ideal normalised coordinates of every pixel centre (cached per unit)."""

    def __init__(self, op):
        u = (np.arange(IMAGE_W) - op["cx"]) / op["f"]
        v = (np.arange(IMAGE_H) - op["cy"]) / op["f"]
        xd = np.broadcast_to(u[None, :], (IMAGE_H, IMAGE_W))
        yd = np.broadcast_to(v[:, None], (IMAGE_H, IMAGE_W))
        self.x, self.y = undistort(xd, yd, op["k1"], op["k2"])
        self.op = op

    def jac_inv(self, x, y):
        """d(ideal x, y) / d(pixel u, v) at ideal points x, y: four arrays."""
        op = self.op
        k1, k2 = op["k1"], op["k2"]
        r2 = x * x + y * y
        s = 1.0 + r2 * (k1 + k2 * r2)
        g = 2.0 * (k1 + 2.0 * k2 * r2)
        a, b, d = s + g * x * x, g * x * y, s + g * y * y          # d(xd, yd)/d(x, y)
        det = a * d - b * b
        f = op["f"]
        return d / (det * f), -b / (det * f), -b / (det * f), a / (det * f)

    def samples(self, pi, pj, ox, oy):
        """Ideal normalised coordinates of sub-pixel samples (pixel pi, pj; offsets ox, oy px)."""
        x, y = self.x[pi, pj], self.y[pi, pj]
        a, b, c, d = self.jac_inv(x, y)
        return (x[:, None] + a[:, None] * ox[None] + b[:, None] * oy[None],
                y[:, None] + c[:, None] * ox[None] + d[:, None] * oy[None])


def _rays(x, y, Rc):
    inv = 1.0 / np.sqrt(x * x + y * y + 1.0)
    cx, cy, cz = x * inv, y * inv, inv
    return (Rc[0, 0] * cx + Rc[0, 1] * cy + Rc[0, 2] * cz,
            Rc[1, 0] * cx + Rc[1, 1] * cy + Rc[1, 2] * cz,
            Rc[2, 0] * cx + Rc[2, 1] * cy + Rc[2, 2] * cz)


# ---- procedural textures (integer hashing, smooth interpolation) ------------------------------------
def _hash01(ix, iy, seed):
    h = (ix.astype(np.int64) * 73856093) ^ (iy.astype(np.int64) * 19349663) ^ seed
    h = (h ^ (h >> 13)) * 1274126177
    h = h ^ (h >> 16)
    return (h & 0xFFFFFF).astype(np.float64) / float(0x1000000)


def value_noise(x, y, scale, seed):
    """Smooth noise in [-1, 1] of wavelength ~scale at points x, y."""
    gx, gy = x / scale, y / scale
    ix, iy = np.floor(gx), np.floor(gy)
    fx, fy = gx - ix, gy - iy
    sx, sy = fx * fx * (3.0 - 2.0 * fx), fy * fy * (3.0 - 2.0 * fy)
    ix, iy = ix.astype(np.int64), iy.astype(np.int64)
    a, b = _hash01(ix, iy, seed), _hash01(ix + 1, iy, seed)
    c, d = _hash01(ix, iy + 1, seed), _hash01(ix + 1, iy + 1, seed)
    return 2.0 * ((a + (b - a) * sx) + ((c + (d - c) * sx) - (a + (b - a) * sx)) * sy) - 1.0


# ---- shading -------------------------------------------------------------------------------------
def _pow2k(x, n):
    while n > 1:
        x = x * x
        n //= 2
    return x


def _lamp_terms(X, Y, Z, lamps):
    """Per lamp: unit vector to the lamp and the irradiance factor power / r^2."""
    out = []
    for (lx, ly, lz), pw in zip(lamps["pos"], lamps["power"]):
        dx, dy, dz = lx - X, ly - Y, lz - Z
        r2 = dx * dx + dy * dy + dz * dz
        inv = 1.0 / np.sqrt(r2)
        out.append((dx * inv, dy * inv, dz * inv, pw / r2))
    return out


def shade_glossy(X, Y, Z, nx, ny, nz, vx, vy, vz, kd, ks, shin, lamps):
    """Diffuse plus Blinn-Phong radiance at points X, Y, Z (normal n, unit vector v to the eye)."""
    L = kd * lamps["ambient"] * np.ones_like(X)
    for lx, ly, lz, E in _lamp_terms(X, Y, Z, lamps):
        ndl = nx * lx + ny * ly + nz * lz
        lit = ndl > 0.0
        hx, hy, hz = lx + vx, ly + vy, lz + vz
        hn = 1.0 / np.sqrt(np.maximum(hx * hx + hy * hy + hz * hz, 1e-30))
        ndh = np.maximum((nx * hx + ny * hy + nz * hz) * hn, 0.0)
        L = L + E * np.where(lit, kd * ndl + ks * _pow2k(ndh, shin), 0.0)
    return L


def background(o, dx, dy, dz, cam):
    """Radiance and depth (distance along the ray) of the table / floor / walls along rays."""
    lamps, mat = cam["lamps"], cam["materials"]
    shape = dx.shape
    L = np.zeros(shape)
    t_out = np.full(shape, 10.0)
    down = dz < -1e-12
    t_tab = np.where(down, -o[2] / np.where(down, dz, -1.0), np.inf)
    X, Y = o[0] + t_tab * dx, o[1] + t_tab * dy
    on_table = down & (o[2] > 0.0) & (np.abs(X) <= TABLE_HALF) & (np.abs(Y) <= TABLE_HALF)
    if np.any(on_table):
        Xt, Yt = X[on_table], Y[on_table]
        tex = (1.0 + mat["grain"] * value_noise(Xt, Yt, 0.006, mat["hash"])
               + mat["mottle"] * value_noise(Xt, Yt, 0.035, mat["hash"] + 7))
        E = lamps["ambient"] * np.ones_like(Xt)
        for lx, ly, lz, El in _lamp_terms(Xt, Yt, np.zeros_like(Xt), lamps):
            E = E + El * np.maximum(lz, 0.0)
        L[on_table] = mat["table"] * tex * E
        t_out[on_table] = t_tab[on_table]
    rest = ~on_table
    # the floor, beyond the table's edge
    t_fl = np.where(down, (FLOOR_Z - o[2]) / np.where(down, dz, -1.0), np.inf)
    Xf, Yf = o[0] + t_fl * dx, o[1] + t_fl * dy
    on_floor = rest & down & (np.abs(Xf) < ROOM_HALF) & (np.abs(Yf) < ROOM_HALF)
    if np.any(on_floor):
        a, b = Xf[on_floor], Yf[on_floor]
        tex = 1.0 + 0.2 * value_noise(a, b, 0.15, mat["hash"] + 11)
        E = lamps["ambient"] * np.ones_like(a)
        for lx, ly, lz, El in _lamp_terms(a, b, np.full_like(a, FLOOR_Z), lamps):
            E = E + El * np.maximum(lz, 0.0)
        L[on_floor] = FLOOR_ALBEDO * tex * E
        t_out[on_floor] = t_fl[on_floor]
    rest &= ~on_floor
    if np.any(rest):
        ddx, ddy, ddz = dx[rest], dy[rest], dz[rest]
        big = 1e9
        tx = np.where(np.abs(ddx) > 1e-12, (np.where(ddx > 0, ROOM_HALF, -ROOM_HALF) - o[0]) / np.where(np.abs(ddx) > 1e-12, ddx, 1.0), big)
        ty = np.where(np.abs(ddy) > 1e-12, (np.where(ddy > 0, ROOM_HALF, -ROOM_HALF) - o[1]) / np.where(np.abs(ddy) > 1e-12, ddy, 1.0), big)
        tz = np.where(ddz > 1e-12, (ROOM_HEIGHT - o[2]) / np.where(ddz > 1e-12, ddz, 1.0), big)
        t = np.minimum(np.minimum(tx, ty), tz)
        Xw, Yw, Zw = o[0] + t * ddx, o[1] + t * ddy, o[2] + t * ddz
        s_coord = np.where(tx <= ty, Yw, Xw) + np.where(tx <= ty, 7.0, 0.0)
        z_coord = np.where(t == tz, Xw, Zw)
        pat = (1.0 + WALL_PATTERN * value_noise(s_coord, z_coord, 0.25, mat["hash"] + 23)
               + 0.15 * value_noise(s_coord, z_coord, 0.9, mat["hash"] + 29))
        grad = 0.75 + 0.25 * np.clip(Zw / ROOM_HEIGHT, 0.0, 1.0)
        L[rest] = mat["wall"] * pat * grad * (lamps["ambient"] + 0.8 * lamps["power"].sum() / LAMP_REF ** 2 / 2.0)
        t_out[rest] = t
    return L, t_out


def _hit_sphere(o, c, dx, dy, dz):
    Px, Py, Pz = c[0] - o[0], c[1] - o[1], c[2] - o[2]
    b = dx * Px + dy * Py + dz * Pz
    disc = b * b - (Px * Px + Py * Py + Pz * Pz - R_SPHERE * R_SPHERE)
    ok = disc > 0.0
    t = b - np.sqrt(np.where(ok, disc, 0.0))
    ok &= t > 1e-6
    return np.where(ok, t, np.inf)


def _hit_post(o, c, dx, dy, dz):
    wx, wy = o[0] - c[0], o[1] - c[1]
    a2 = dx * dx + dy * dy
    b2 = wx * dx + wy * dy
    disc = b2 * b2 - a2 * (wx * wx + wy * wy - R_POST * R_POST)
    ok = (disc > 0.0) & (a2 > 1e-12)
    t = (-b2 - np.sqrt(np.where(ok, disc, 0.0))) / np.where(ok, a2, 1.0)
    z = o[2] + t * dz
    ok &= (t > 1e-6) & (z >= 0.0) & (z <= c[2])
    return np.where(ok, t, np.inf)


def _object_radiance(o, c, dx, dy, dz, cam, t_max=None):
    """Coverage (0/1) and radiance of one sphere-and-post layer along rays (arrays); t_max: what is nearer
    along each ray hides the layer there (the gauge plate)."""
    mat, lamps = cam["materials"], cam["lamps"]
    ts = _hit_sphere(o, c, dx, dy, dz)
    tp = _hit_post(o, c, dx, dy, dz)
    hs = ts <= tp
    t = np.minimum(ts, tp)
    hit = np.isfinite(t)
    if t_max is not None:
        hit &= t < t_max
    L = np.zeros(dx.shape)
    if np.any(hit):
        tt = t[hit]
        X, Y, Z = o[0] + tt * dx[hit], o[1] + tt * dy[hit], o[2] + tt * dz[hit]
        sph = hs[hit]
        nx, ny = X - c[0], Y - c[1]
        nz = np.where(sph, Z - c[2], 0.0)
        inv = np.where(sph, 1.0 / R_SPHERE, 1.0 / R_POST)
        nx, ny, nz = nx * inv, ny * inv, nz * inv
        vx, vy, vz = -dx[hit], -dy[hit], -dz[hit]
        Ls = shade_glossy(X, Y, Z, nx, ny, nz, vx, vy, vz, mat["sphere_d"], mat["sphere_s"], SPHERE_SHININESS, lamps)
        Lp = shade_glossy(X, Y, Z, nx, ny, nz, vx, vy, vz, mat["post_d"], POST_SPEC, POST_SHININESS, lamps)
        L[hit] = np.where(sph, Ls, Lp)
    return hit.astype(float), L


# ---- the gauge plate: a block with blind bores, on three shims --------------------------------------
def plate_hits(o, dx, dy, dz, plate):
    """First hit of rays (origin o, unit directions) with the plate (block less bores) or a shim: distance t
    (inf: none), surface code (0 none, 1 top, 2..5 sides, 6 bore wall, 7 bore bottom, 8 shim, 9 bottom), bore
    index, hit point in the plate frame (X, Y, Z) and the normal in the plate frame."""
    R, O = plate["R"], plate["O"]
    shape = dx.shape
    w = o - O
    ox, oy, oz = (w @ R[:, 0], w @ R[:, 1], w @ R[:, 2])
    px = R[0, 0] * dx + R[1, 0] * dy + R[2, 0] * dz
    py = R[0, 1] * dx + R[1, 1] * dy + R[2, 1] * dz
    pz = R[0, 2] * dx + R[1, 2] * dy + R[2, 2] * dz
    a, b, T = PLATE_SIZE[0] / 2, PLATE_SIZE[1] / 2, PLATE_SIZE[2]
    t_in = np.full(shape, -np.inf)
    t_out = np.full(shape, np.inf)
    face = np.zeros(shape, dtype=np.int64)
    for k, (oo, dd, lo, hi) in enumerate(((ox, px, -a, a), (oy, py, -b, b), (oz, pz, -T, 0.0))):
        small = np.abs(dd) < 1e-12
        inv = 1.0 / np.where(small, 1.0, dd)
        t1, t2 = (lo - oo) * inv, (hi - oo) * inv
        tn, tf = np.minimum(t1, t2), np.maximum(t1, t2)
        miss = small & ((oo < lo) | (oo > hi))
        tn = np.where(small, np.where(miss, np.inf, -np.inf), tn)
        tf = np.where(small, np.where(miss, -np.inf, np.inf), tf)
        # the entry face: the far side of the axis the ray enters through
        code = (2 + 2 * k + (dd > 0)) if k < 2 else np.where(dd < 0, 1, 9)
        upd = tn > t_in
        face = np.where(upd, code, face)
        t_in = np.maximum(t_in, tn)
        t_out = np.minimum(t_out, tf)
    hit = (t_in <= t_out) & (t_in > 1e-6)
    t = np.where(hit, t_in, np.inf)
    face = np.where(hit, face, 0)
    X, Y, Z = ox + t * px, oy + t * py, oz + t * pz
    X, Y, Z = (np.where(hit, v, 0.0) for v in (X, Y, Z))
    nx = np.where(face == 2, 1.0, np.where(face == 3, -1.0, 0.0))
    ny = np.where(face == 4, 1.0, np.where(face == 5, -1.0, 0.0))
    nz = np.where(face == 1, 1.0, np.where(face == 9, -1.0, 0.0))
    bore = np.full(shape, -1, dtype=np.int64)
    top = np.nonzero((face == 1).ravel())[0]
    if len(top):
        Xt, Yt = X.ravel()[top], Y.ravel()[top]
        pxt, pyt, pzt = px.ravel()[top], py.ravel()[top], pz.ravel()[top]
        d2 = (Xt[:, None] - BORES[None, :, 0]) ** 2 + (Yt[:, None] - BORES[None, :, 1]) ** 2
        i = np.argmin(d2, axis=1)
        sel = d2[np.arange(len(top)), i] < BORE_RADIUS ** 2
        if np.any(sel):
            idx, bi = top[sel], i[sel]
            cx, cy = BORES[bi, 0], BORES[bi, 1]
            qx, qy = Xt[sel] - cx, Yt[sel] - cy
            ax, ay, az = pxt[sel], pyt[sel], pzt[sel]
            A = ax * ax + ay * ay
            Bq = qx * ax + qy * ay
            C = qx * qx + qy * qy - BORE_RADIUS ** 2
            tw = np.where(A > 1e-18, (-Bq + np.sqrt(np.maximum(Bq * Bq - A * C, 0.0))) / np.where(A > 1e-18, A, 1.0), np.inf)
            on_wall = tw * az >= -BORE_DEPTH
            tb = -BORE_DEPTH / np.where(az < -1e-12, az, -1e-12)
            dt = np.where(on_wall, tw, tb)
            Xn, Yn, Zn = Xt[sel] + dt * ax, Yt[sel] + dt * ay, dt * az
            for arr, val in ((t, t.ravel()[idx] + dt), (face, np.where(on_wall, 6, 7)), (bore, bi), (X, Xn), (Y, Yn), (Z, Zn),
                             (nx, np.where(on_wall, -(Xn - cx) / BORE_RADIUS, 0.0)),
                             (ny, np.where(on_wall, -(Yn - cy) / BORE_RADIUS, 0.0)), (nz, np.where(on_wall, 0.0, 1.0))):
                arr.reshape(-1)[idx] = val
    # the shims: vertical cylinders (datum frame) from the table to the plate's bottom face; only a ray that
    # misses the block can reach one (they stand under it)
    miss = np.nonzero((face == 0).ravel() & (dz.ravel() < 0.0))[0]
    if len(miss):
        mdx, mdy, mdz = dx.ravel()[miss], dy.ravel()[miss], dz.ravel()[miss]
        best = np.full(len(miss), np.inf)
        which = np.full(len(miss), -1)
        for k, f in enumerate(plate["shims"]):
            wx, wy = o[0] - f[0], o[1] - f[1]
            a2 = mdx * mdx + mdy * mdy
            b2 = wx * mdx + wy * mdy
            disc = b2 * b2 - a2 * (wx * wx + wy * wy - SHIM_RADIUS ** 2)
            ok = (disc > 0.0) & (a2 > 1e-12)
            ts = (-b2 - np.sqrt(np.where(ok, disc, 0.0))) / np.where(ok, a2, 1.0)
            zs = o[2] + ts * mdz
            ok &= (ts > 1e-6) & (zs >= 0.0) & (zs <= f[2]) & (ts < best)
            best = np.where(ok, ts, best)
            which = np.where(ok, k, which)
        got = which >= 0
        if np.any(got):
            idx, ts, f = miss[got], best[got], plate["shims"][which[got]]
            hx, hy = o[0] + ts * mdx[got], o[1] + ts * mdy[got]
            hz = o[2] + ts * mdz[got]
            nwx, nwy = (hx - f[:, 0]) / SHIM_RADIUS, (hy - f[:, 1]) / SHIM_RADIUS
            wpx, wpy, wpz = hx - O[0], hy - O[1], hz - O[2]
            for arr, val in ((t, ts), (face, 8), (bore, -1),
                             (X, wpx * R[0, 0] + wpy * R[1, 0] + wpz * R[2, 0]),
                             (Y, wpx * R[0, 1] + wpy * R[1, 1] + wpz * R[2, 1]),
                             (Z, wpx * R[0, 2] + wpy * R[1, 2] + wpz * R[2, 2]),
                             (nx, nwx * R[0, 0] + nwy * R[1, 0]), (ny, nwx * R[0, 1] + nwy * R[1, 1]),
                             (nz, nwx * R[0, 2] + nwy * R[1, 2])):
                arr.reshape(-1)[idx] = val
    return t, face, bore, (X, Y, Z), (nx, ny, nz)


def plate_radiance(o, dx, dy, dz, plate, cam, hits):
    """Radiance of the plate's surfaces along rays that hit them (hits: plate_hits' result); also a code
    that changes wherever the shading has an edge (the surface, the bore, which lamps light a bore's inside)."""
    t, face, bore, (X, Y, Z), (nx, ny, nz) = hits
    R, O = plate["R"], plate["O"]
    mat, lamps = cam["materials"], cam["lamps"]
    L = np.zeros(dx.shape)
    code = face.copy()
    on = face > 0
    if not np.any(on):
        return L, code
    Xs, Ys, Zs, f, bi = X[on], Y[on], Z[on], face[on], bore[on]
    nxs, nys, nzs = nx[on], ny[on], nz[on]
    Pw = O[None] + np.stack([Xs, Ys, Zs], axis=1) @ R.T                 # datum frame
    nw = np.stack([nxs, nys, nzs], axis=1) @ R.T
    vx, vy, vz = -dx[on], -dy[on], -dz[on]
    block = (f <= 5) | (f == 9)
    tex = 1.0 + mat["plate_grain"] * value_noise(Xs, Ys, 0.002, mat["plate_hash"])
    Lb = shade_glossy(Pw[:, 0], Pw[:, 1], Pw[:, 2], nw[:, 0], nw[:, 1], nw[:, 2], vx, vy, vz,
                      mat["plate_d"], mat["plate_s"], PLATE_SHININESS, lamps) * np.where(f == 1, tex, 1.0)
    # inside a bore: lit by a lamp only where the lamp is seen through the bore's mouth; a share of the ambient
    inside = (f == 6) | (f == 7)
    Li = mat["bore_d"] * lamps["ambient"] * BORE_AMBIENT * np.ones_like(Xs)
    bits = np.zeros(len(Xs), dtype=np.int64)
    cxs, cys = BORES[np.maximum(bi, 0), 0], BORES[np.maximum(bi, 0), 1]
    for j, (lx, ly, lz, E) in enumerate(_lamp_terms(Pw[:, 0], Pw[:, 1], Pw[:, 2], lamps)):
        lpx = lx * R[0, 0] + ly * R[1, 0] + lz * R[2, 0]
        lpy = lx * R[0, 1] + ly * R[1, 1] + lz * R[2, 1]
        lpz = lx * R[0, 2] + ly * R[1, 2] + lz * R[2, 2]
        ndl = nxs * lpx + nys * lpy + nzs * lpz
        tm = -Zs / np.where(lpz > 1e-9, lpz, 1.0)
        mx, my = Xs + tm * lpx - cxs, Ys + tm * lpy - cys
        lit = inside & (ndl > 0.0) & (lpz > 1e-9) & (mx * mx + my * my < BORE_RADIUS ** 2)
        Li = Li + np.where(lit, mat["bore_d"] * E * ndl, 0.0)
        bits |= lit.astype(np.int64) << j
    Ls = shade_glossy(Pw[:, 0], Pw[:, 1], Pw[:, 2], nw[:, 0], nw[:, 1], nw[:, 2], vx, vy, vz, SHIM_ALBEDO, 0.0, 2,
                      lamps)
    L[on] = np.where(block, Lb, np.where(inside, Li, Ls))
    code[on] = f + 16 * bits + 256 * (bi + 1)
    return L, code


def _plate_box(plate, Rc, o, op):
    """Pixel box (j0, j1, i0, i1) that holds the plate and its shims, or None when out of view."""
    a, b, T = PLATE_SIZE[0] / 2, PLATE_SIZE[1] / 2, PLATE_SIZE[2]
    pts = [plate["O"] + plate["R"] @ np.array([x, y, z]) for x in (-a, a) for y in (-b, b) for z in (-T, 0.0)]
    pts += [np.array([f[0], f[1], 0.0]) for f in plate["shims"]]
    P = (np.array(pts) - o) @ Rc
    if np.any(P[:, 2] < 0.02):
        if np.all(P[:, 2] < 0.0):
            return None
        return 0, IMAGE_W, 0, IMAGE_H
    xd, yd = distort(P[:, 0] / P[:, 2], P[:, 1] / P[:, 2], op["k1"], op["k2"])
    u, v = op["cx"] + op["f"] * xd, op["cy"] + op["f"] * yd
    pad = DIST_MAX_PX + 3.0
    j0, j1 = max(0, int(math.floor(u.min() - pad))), min(IMAGE_W, int(math.ceil(u.max() + pad)) + 1)
    i0, i1 = max(0, int(math.floor(v.min() - pad))), min(IMAGE_H, int(math.ceil(v.max() + pad)) + 1)
    if j0 >= j1 or i0 >= i1:
        return None
    return j0, j1, i0, i1


def _plate_layer(o, dx, dy, dz, Lb, tb, plate, cam, grid, Rc, box):
    """The plate drawn into the background (radiance Lb, ray distance tb, in place) inside the pixel box,
    supersampled along every shading edge; returns the ray distance to the plate per pixel of the box."""
    j0, j1, i0, i1 = box
    sx, sy, sz = dx[i0:i1, j0:j1], dy[i0:i1, j0:j1], dz[i0:i1, j0:j1]
    hits = plate_hits(o, sx, sy, sz, plate)
    Lp, code = plate_radiance(o, sx, sy, sz, plate, cam, hits)
    tp = hits[0]
    front = tp < tb[i0:i1, j0:j1]
    code = np.where(front, code, 0)
    sub_L = np.where(front, Lp, Lb[i0:i1, j0:j1])
    sub_t = np.where(front, tp, tb[i0:i1, j0:j1])
    # pixels whose 3 x 3 neighbourhood holds more than one code: supersampled
    pad = np.pad(code, 1, mode="edge")
    H, W = code.shape
    edge = np.zeros(code.shape, bool)
    for di in (0, 1, 2):
        for dj in (0, 1, 2):
            edge |= pad[di:di + H, dj:dj + W] != code
    pi, pj = np.nonzero(edge)
    if len(pi):
        ox, oy = SS_COARSE
        xs, ys = grid.samples(pi + i0, pj + j0, ox, oy)
        rx, ry, rz = _rays(xs, ys, Rc)
        h2 = plate_hits(o, rx, ry, rz, plate)
        Ls, _ = plate_radiance(o, rx, ry, rz, plate, cam, h2)
        off = ~np.isfinite(h2[0])           # the plate stands on the table: what it does not hide is behind it
        if np.any(off):
            Ls[off] = background(o, rx[off], ry[off], rz[off], cam)[0]
        acc = Ls[:, 0].copy()
        for k in range(1, Ls.shape[1]):
            acc += Ls[:, k]
        sub_L[pi, pj] = acc / Ls.shape[1]
    Lb[i0:i1, j0:j1] = sub_L
    tb[i0:i1, j0:j1] = sub_t
    return np.where(front, tp, np.inf)



# ---- blur (separable, explicit taps) ------------------------------------------------------------
_TAPS = {}


def taps(sigma):
    key = int(round(sigma * 50))            # sigma quantised to 0.02 px
    if key not in _TAPS:
        s = max(key / 50.0, 0.2)
        r = int(math.ceil(3.5 * s))
        w = np.array([math.exp(-k * k / (2 * s * s)) for k in range(-r, r + 1)])
        w = np.round(w * 2.0 ** 36) / 2.0 ** 36
        _TAPS[key] = w / w.sum()
    return _TAPS[key]


def blur(E, sigma):
    """Separable Gaussian, rows then columns; edge pixels repeated. (In place, tap by tap: the same
    roundings as out = out + w[k] * shifted.)"""
    w = taps(sigma)
    r = len(w) // 2
    H, W = E.shape
    P = np.pad(E, ((0, 0), (r, r)), mode="edge")
    E1 = w[0] * P[:, 0:W]
    tmp = np.empty_like(E1)
    for k in range(1, len(w)):
        np.multiply(P[:, k:k + W], w[k], out=tmp)
        np.add(E1, tmp, out=E1)
    P = np.pad(E1, ((r, r), (0, 0)), mode="edge")
    out = w[0] * P[0:H]
    for k in range(1, len(w)):
        np.multiply(P[k:k + H], w[k], out=tmp)
        np.add(out, tmp, out=out)
    return out


def _down(E, k):
    H, W = E.shape
    Hk, Wk = H // k, W // k
    A = E[:Hk * k, :Wk * k].reshape(Hk, k, Wk, k)
    acc = A[:, 0, :, 0].copy()
    for a in range(k):
        for b in range(k):
            if a or b:
                acc = acc + A[:, a, :, b]
    return acc / (k * k)


def _up(S, k, H, W):
    """Bilinear upsampling of a k-times block-averaged image back to H x W."""
    Hs, Ws = S.shape
    yi = (np.arange(H) + 0.5) / k - 0.5
    xi = (np.arange(W) + 0.5) / k - 0.5
    y0 = np.clip(np.floor(yi).astype(int), 0, Hs - 1)
    x0 = np.clip(np.floor(xi).astype(int), 0, Ws - 1)
    y1, x1 = np.minimum(y0 + 1, Hs - 1), np.minimum(x0 + 1, Ws - 1)
    fy = np.clip(yi - y0, 0.0, 1.0)[:, None]
    fx = np.clip(xi - x0, 0.0, 1.0)[None, :]
    a = S[y0][:, x0] + (S[y0][:, x1] - S[y0][:, x0]) * fx
    b = S[y1][:, x0] + (S[y1][:, x1] - S[y1][:, x0]) * fx
    return a + (b - a) * fy


def defocus_sigma(op, z):
    """Blur (px) of a surface at depth z (m, along the optical axis)."""
    d = op["aperture"] * np.abs(1.0 / np.maximum(z, 1e-3) - 1.0 / op["focus"])
    return np.sqrt(op["blur0"] ** 2 + d * d)


# ---- the image ------------------------------------------------------------------------------------
def _sphere_box(P, op):
    """Pixel box of a sphere (centre P in the camera frame): from the ideal outline, padded for the
    distortion; None if behind the camera or not an ellipse."""
    d2 = float(P @ P)
    if d2 <= R_SPHERE ** 2 * 1.0001 or P[2] <= R_SPHERE:
        return None
    d = np.sqrt(d2)
    sin_b = R_SPHERE / d
    cos_a = P[2] / d
    if math.acos(min(cos_a, 1.0)) + math.asin(sin_b) > 1.2:
        return None
    Cs = np.linalg.inv(np.outer(P, P) - (d2 - R_SPHERE ** 2) * np.eye(3))
    out = []
    for a in (0, 1):
        r = np.sqrt(max(Cs[a, 2] ** 2 - Cs[2, 2] * Cs[a, a], 0.0))
        lo, hi = sorted([(Cs[a, 2] - r) / Cs[2, 2], (Cs[a, 2] + r) / Cs[2, 2]])
        out.append((lo, hi))
    (x0, x1), (y0, y1) = out
    corners = [(x, y) for x in (x0, x1) for y in (y0, y1)]
    us, vs = [], []
    for x, y in corners:
        xd, yd = distort(x, y, op["k1"], op["k2"])
        us.append(op["cx"] + op["f"] * xd)
        vs.append(op["cy"] + op["f"] * yd)
    pad = DIST_MAX_PX + 2.0
    return min(us) - pad, max(us) + pad, min(vs) - pad, max(vs) + pad


def _post_box(c, Rc, o, op):
    """Pixel box of a post (axis from (c_x, c_y, 0) to c, radius R_POST), clipped to the part in
    front of the camera: points along the axis, each widened by the post's angular radius; None if
    no part of it can be seen."""
    t = np.linspace(0.0, 1.0, 257)
    ax = np.stack([np.full_like(t, c[0]), np.full_like(t, c[1]), t * c[2]], axis=1) - o
    cam = np.stack([ax[:, 0] * Rc[0, i] + ax[:, 1] * Rc[1, i] + ax[:, 2] * Rc[2, i] for i in range(3)], axis=1)
    dist = np.sqrt((cam * cam).sum(1))
    front = cam[:, 2] > R_POST + 0.002
    # a post point that close to the camera's plane is far outside the field unless it is near the
    # camera: then nothing is assumed
    if np.any(~front & (cam[:, 2] > -R_POST - 0.002) & (dist < 0.04)):
        return (0.0, IMAGE_W - 1.0, 0.0, IMAGE_H - 1.0)
    if not np.any(front):
        return None
    cam = cam[front]
    z = cam[:, 2] - R_POST
    xd, yd = distort(cam[:, 0] / z, cam[:, 1] / z, op["k1"], op["k2"])
    # the steps between axis samples are covered by the post's own width (a sample every c_z / 256)
    w = op["f"] * (1.5 * R_POST + c[2] / 256.0) / z * 1.2
    u, v = op["cx"] + op["f"] * xd, op["cy"] + op["f"] * yd
    pad = DIST_MAX_PX + 2.0
    u0, u1 = float(np.min(u - w)) - pad, float(np.max(u + w)) + pad
    v0, v1 = float(np.min(v - w)) - pad, float(np.max(v + w)) + pad
    if u1 < -50 or u0 > IMAGE_W + 50 or v1 < -50 or v0 > IMAGE_H + 50:
        return None
    return max(u0, -50.0), min(u1, IMAGE_W + 50.0), max(v0, -50.0), min(v1, IMAGE_H + 50.0)


def irradiance(cam, grid, Rc, o, centres, plate=None):
    """Noise-free image (before vignetting and exposure): background (with the gauge plate), sphere layers,
    glare."""
    op = cam["optics"]
    H, W = IMAGE_H, IMAGE_W
    Rc, o = np.asarray(Rc, float), np.asarray(o, float)
    dx, dy, dz = _rays(grid.x, grid.y, Rc)
    Lb, tb = background(o, dx, dy, dz, cam)
    pbox = _plate_box(plate, Rc, o, op) if plate is not None else None
    t_plate = None
    if pbox is not None:
        t_plate = np.full((H, W), np.inf)
        j0, j1, i0, i1 = pbox
        t_plate[i0:i1, j0:j1] = _plate_layer(o, dx, dy, dz, Lb, tb, plate, cam, grid, Rc, pbox)
    # depth along the optical axis of what each pixel sees
    zb = tb / np.sqrt(grid.x * grid.x + grid.y * grid.y + 1.0)
    sb = defocus_sigma(op, zb)
    sub = sb[::8, ::8].ravel()
    s_a, s_b = float(np.percentile(sub, 30)), float(np.percentile(sub, 97))
    A = blur(Lb, s_a)
    if s_b > s_a + 0.1:
        B = _up(blur(_down(Lb, 2), s_b / 2.0), 2, H, W) if s_b > 2.5 else blur(Lb, s_b)
        w = np.clip((sb - s_a) / (s_b - s_a), 0.0, 1.0)
        img = A + (B - A) * w
    else:
        img = A
    # the sphere-and-post layers, farthest first
    layers = []
    for c in np.asarray(centres, float):
        w_ = c - o
        P = np.array([w_[0] * Rc[0, i] + w_[1] * Rc[1, i] + w_[2] * Rc[2, i] for i in range(3)])
        boxes = [b for b in (_sphere_box(P, op), _post_box(c, Rc, o, op)) if b is not None]
        if not boxes:
            continue
        if P[2] < 0.02:
            continue
        sig = min(float(defocus_sigma(op, P[2])), LAYER_BLUR_MAX)     # a layer very near the camera
        pad = int(math.ceil(3.5 * sig)) + 3
        u0, u1 = min(b[0] for b in boxes), max(b[1] for b in boxes)
        v0, v1 = min(b[2] for b in boxes), max(b[3] for b in boxes)
        j0, j1 = max(0, int(math.floor(u0)) - pad), min(W, int(math.ceil(u1)) + pad + 1)
        i0, i1 = max(0, int(math.floor(v0)) - pad), min(H, int(math.ceil(v1)) + pad + 1)
        if j0 >= j1 or i0 >= i1:
            continue
        layers.append((float(P @ P), c, P, sig, (i0, i1, j0, j1)))
    layers.sort(key=lambda t: -t[0])
    for _, c, P, sig, (i0, i1, j0, j1) in layers:
        sx, sy, sz = dx[i0:i1, j0:j1], dy[i0:i1, j0:j1], dz[i0:i1, j0:j1]
        hidden = None
        if pbox is not None and pbox[0] < j1 and j0 < pbox[1] and pbox[2] < i1 and i0 < pbox[3]:
            hidden = t_plate[i0:i1, j0:j1]
        alpha, L = _object_radiance(o, c, sx, sy, sz, cam, hidden)
        # pixels near the sphere's or the post's outline: supersampled
        Px, Py, Pz = c[0] - o[0], c[1] - o[1], c[2] - o[2]
        b = sx * Px + sy * Py + sz * Pz
        h = np.sqrt(np.maximum(Px * Px + Py * Py + Pz * Pz - b * b, 0.0))
        fine = (np.abs(h - R_SPHERE) * op["f"] < BAND_SPHERE * b) & (b > 0.0)
        wx, wy = o[0] - c[0], o[1] - c[1]
        a2 = np.maximum(sx * sx + sy * sy, 1e-300)
        cr = wx * sy - wy * sx
        hp = np.sqrt(cr * cr / a2)
        tsp = -(wx * sx + wy * sy) / a2
        zs = o[2] + tsp * sz
        coarse = ((np.abs(hp - R_POST) * op["f"] < BAND_POST * tsp) & (tsp > 0.0) & (zs > -0.02) & (zs < c[2] + 0.02)) & ~fine
        for mask, (ox, oy) in ((fine, SS_FINE), (coarse, SS_COARSE)):
            pi, pj = np.nonzero(mask)
            if len(pi) == 0:
                continue
            xs, ys = grid.samples(pi + i0, pj + j0, ox, oy)
            rx, ry, rz = _rays(xs, ys, Rc)
            a_s, L_s = _object_radiance(o, c, rx, ry, rz, cam,
                                        None if hidden is None else plate_hits(o, rx, ry, rz, plate)[0])
            acc_a, acc_l = a_s[:, 0].copy(), L_s[:, 0].copy()
            for k in range(1, len(ox)):
                acc_a += a_s[:, k]
                acc_l += L_s[:, k]
            alpha[pi, pj] = acc_a / len(ox)
            L[pi, pj] = acc_l / len(ox)        # premultiplied: mean over the pixel of radiance (0 where missed)
        if not np.any(alpha > 0.0):
            continue                               # nothing of this layer is in view (compositing it is the identity)
        Ab, ALb = blur(alpha, sig), blur(L, sig)     # L is premultiplied everywhere (0 where nothing is hit)
        img[i0:i1, j0:j1] = ALb + (1.0 - Ab) * img[i0:i1, j0:j1]
    # veiling glare
    hf = op["halo"]
    S = _down(img, 4)
    G = _up(blur(S, HALO_SIGMA / 4.0), 4, H, W)
    return (1.0 - hf) * img + hf * G


def vignetting(cam, grid):
    op = cam["optics"]
    j = (np.arange(IMAGE_W) - op["cx"]) / op["f"]
    i = (np.arange(IMAGE_H) - op["cy"]) / op["f"]
    rho2 = j[None, :] ** 2 + i[:, None] ** 2
    t = 1.0 + cam["sensor"]["vignette"] * rho2
    return 1.0 / (t * t)


def exposure(E, cam):
    """The automatic exposure: electrons per unit of E (from a sub-sampled grid, exact sums)."""
    sub = E[2::4, 2::4].ravel()
    total = int(np.floor(sub * 2.0 ** 24).astype(np.int64).sum())
    mean = total / 2.0 ** 24 / len(sub)
    k = int(AE_HIGH_FRACTION * (len(sub) - 1))
    p99 = float(np.partition(sub, k)[k])
    g = min(cam["sensor"]["ae_mean"] / max(mean, 1e-9), AE_HIGH / max(p99, 1e-9))
    g = min(max(g, GAIN_LIMITS[0]), GAIN_LIMITS[1])
    g = np.floor(g * 64.0) / 64.0                       # exposure steps
    return g * FULL_WELL / 255.0                         # electrons per unit E


def sensor(E, cam, rng):
    """Vignetted irradiance E -> uint8 image: exposure, shot and read noise, black level, hot and
    dead pixels."""
    k = exposure(E, cam)
    e = E * k
    e = e + rng.standard_normal(E.shape) * np.sqrt(np.maximum(e, 0.0) + READ_NOISE * READ_NOISE)
    v = np.floor(cam["sensor"]["black"] + 255.0 * e / FULL_WELL + 0.5)
    img = np.clip(v, 0, 255).astype(np.uint8)
    flat = img.reshape(-1)
    s = cam["sensor"]
    flat[s["hot"]] = np.minimum(255, flat[s["hot"]].astype(int) + s["hot_val"]).astype(np.uint8)
    flat[s["dead"]] = 0
    return img


def render(cam, grid, Rc, o, centres, rng, vig=None, plate=None):
    E = irradiance(cam, grid, Rc, o, centres, plate)
    if vig is None:
        vig = vignetting(cam, grid)
    return sensor(E * vig, cam, rng)
