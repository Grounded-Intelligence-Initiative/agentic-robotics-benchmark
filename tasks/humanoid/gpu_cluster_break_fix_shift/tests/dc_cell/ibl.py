"""The data hall's image light (cell internals): a procedural high-dynamic-range environment of the hall as seen from
the middle of the pod (the ceiling's linear LED fixtures in rows along the aisles, the pale ceiling between them, the
black rack fronts all round with their status LEDs, the light-grey raised floor) turned into the cube map Filament
lights the scene with, written as a KTX1 file the way Filament's cmgen writes one: prefiltered specular levels plus the
irradiance's spherical harmonics.

The cube-map conventions and the KTX writer follow tasks/ground/earthquake_search_and_rescue/tests/rubblecell/sky.py
(MuJoCo's Filament samples a cube map in a y-up frame: a world direction (x, y, z) (z up) reads the cube at
(x, z, -y)). Everything is deterministic: the same bytes on every machine.
"""

from __future__ import annotations

import math
import struct

import numpy as np

__all__ = ["cube_dirs", "write_ktx", "irradiance_sh", "hall_ibl"]

GL_R11F_G11F_B10F = 0x8C3A
GL_RGB = 0x1907


def cube_dirs(n):
    """(6, n, n, 3) unit directions of the texel centres of an OpenGL cube map (faces +x, -x, +y, -y, +z, -z; row 0 is
    the face's top), and (n, n) solid angles of one face's texels."""
    c = (np.arange(n) + 0.5) / n * 2.0 - 1.0
    sc, tc = np.meshgrid(c, c)
    one = np.ones_like(sc)
    faces = [np.stack([one, -tc, -sc], -1), np.stack([-one, -tc, sc], -1), np.stack([sc, one, tc], -1),
             np.stack([sc, -one, -tc], -1), np.stack([sc, -tc, one], -1), np.stack([-sc, -tc, -one], -1)]
    d = np.stack(faces, 0)
    l2 = (d * d).sum(-1, keepdims=True)
    d = d / np.sqrt(l2)
    dw = (2.0 / n) ** 2 / (l2[0, ..., 0] ** 1.5)
    return d, dw


def _pack_r11g11b10(rgb):
    x = np.clip(np.asarray(rgb, np.float32), 0.0, 64000.0)
    h = x.astype(np.float16).view(np.uint16).astype(np.uint32)
    r = (h[..., 0] >> 4) & 0x7FF
    g = (h[..., 1] >> 4) & 0x7FF
    b = (h[..., 2] >> 5) & 0x3FF
    return (r | (g << 11) | (b << 22)).astype("<u4")


def write_ktx(levels, sh=None):
    """KTX1 bytes of a cube map: `levels` = [(6, n, n, 3) float32, ...] from the largest; `sh` = (9, 3) or None."""
    n0 = levels[0].shape[1]
    kv = b""
    if sh is not None:
        txt = "".join("%g %g %g\n" % tuple(float(v) for v in row) for row in sh).encode()
        body = b"sh\x00" + txt
        kv = struct.pack("<I", len(body)) + body
        kv += b"\x00" * ((4 - len(kv) % 4) % 4)
    hdr = struct.pack("<13I", 0x04030201, GL_R11F_G11F_B10F, 1, GL_RGB, GL_R11F_G11F_B10F, GL_R11F_G11F_B10F,
                      n0, n0, 0, 0, 6, len(levels), len(kv))
    out = [b"\xabKTX 11\xbb\r\n\x1a\n", hdr, kv]
    for lv in levels:
        n = lv.shape[1]
        out.append(struct.pack("<I", n * n * 4))
        for f in range(6):
            out.append(_pack_r11g11b10(lv[f]).tobytes())
    return b"".join(out)


def _to_world(c):
    return np.stack([c[..., 0], -c[..., 2], c[..., 1]], -1)


def irradiance_sh(rad, dw, d):
    """Filament's shader-ready irradiance SH (9 x rgb) of a cube map of radiance (6, n, n, 3)."""
    x, y, z = d[..., 0], d[..., 1], d[..., 2]
    polys = [np.ones_like(x), y, z, x, x * y, y * z, 3 * z * z - 1, x * z, x * x - y * y]
    sp = math.sqrt(math.pi)
    K = [1 / (2 * sp), math.sqrt(3) / (2 * sp), math.sqrt(3) / (2 * sp), math.sqrt(3) / (2 * sp),
         math.sqrt(15) / (2 * sp), math.sqrt(15) / (2 * sp), math.sqrt(5) / (4 * sp), math.sqrt(15) / (2 * sp),
         math.sqrt(15) / (4 * sp)]
    Ahat = [math.pi, 2 * math.pi / 3, 2 * math.pi / 3, 2 * math.pi / 3] + [math.pi / 4] * 5
    w = dw[None, ..., None]
    out = np.zeros((9, 3))
    for i in range(9):
        proj = (rad * polys[i][..., None] * w).sum(axis=(0, 1, 2))
        out[i] = Ahat[i] * K[i] * K[i] * proj / math.pi
    return out


# ------------------------------------------------------------------------------------------------ the hall
HALL = {
    "probe": (1.5, 0.0, 1.4),            # where the environment is seen from (the cold aisle's middle)
    "ceiling_z": 3.6,
    "floor_z": 0.0,
    "strip_y": (0.0, 2.4, -2.4, 4.8, -4.8),   # the LED fixture rows (along x)
    "strip_halfwidth": 0.07,
    "strip_period_x": 1.8,               # one 1.2 m fixture every 1.8 m along a row
    "strip_len": 1.2,
    "strip_rgb": (1.0, 0.985, 0.96),     # 5000 K, a touch warm of the white point
    "strip_radiance": 40.0,
    "ceiling_rgb": (0.30, 0.31, 0.33),
    "floor_rgb": (0.42, 0.42, 0.43),
    "racks_rgb": (0.035, 0.037, 0.040),
    "wall_rgb": (0.20, 0.21, 0.22),
    "led_density": 0.004,
}


def _radiance(dirs, blur):
    """Radiance of the hall seen from the probe along world directions (N, 3), smoothed by `blur` (radians, roughly)."""
    H = HALL
    p = np.asarray(H["probe"], np.float64)
    d = dirs / np.linalg.norm(dirs, axis=-1, keepdims=True)
    out = np.zeros(d.shape, np.float64)
    up = d[:, 2] > 1e-4
    t = (H["ceiling_z"] - p[2]) / np.where(up, d[:, 2], 1.0)
    hx = p[0] + d[:, 0] * t
    hy = p[1] + d[:, 1] * t
    # the ceiling: pale panels; the fixtures: bright strips, widened by the blur (their light spread over a lobe)
    w = H["strip_halfwidth"] + blur * np.maximum(t, 0.5) * 0.8
    strip = np.zeros_like(hx)
    for sy in H["strip_y"]:
        strip = np.maximum(strip, np.clip(1.0 - np.abs(hy - sy) / w, 0.0, 1.0))
    ph = np.mod(hx, H["strip_period_x"])
    along = np.clip(1.0 - np.maximum(0.0, np.abs(ph - H["strip_period_x"] / 2) - H["strip_len"] / 2) / (w * 2), 0.0, 1.0)
    k = (H["strip_halfwidth"] / w) ** 1.0
    lit = strip * along * k
    ceiling = np.asarray(H["ceiling_rgb"])[None] * (0.6 + 0.4 * np.exp(-np.abs(hy) / 6.0))[:, None]
    out[up] = (ceiling * (1 - lit[:, None]) + np.asarray(H["strip_rgb"])[None] * H["strip_radiance"] * lit[:, None])[up]
    # the horizon band: the racks' black fronts (rows run along x), the walls beyond the ends of the aisle
    hor = ~up & (d[:, 2] > -1e-4)
    along_x = np.abs(d[:, 0]) > 0.6
    rack = np.asarray(H["racks_rgb"])[None].repeat(len(d), 0)
    wall = np.asarray(H["wall_rgb"])[None].repeat(len(d), 0)
    side = np.where(along_x[:, None], wall, rack)
    out[hor] = side[hor]
    # below: the floor lit from above, brighter under the aisle's own fixtures
    dn = d[:, 2] < -1e-4
    tf = (p[2] - H["floor_z"]) / np.where(dn, -d[:, 2], 1.0)
    fy = p[1] + d[:, 1] * tf
    floor = np.asarray(H["floor_rgb"])[None] * (0.55 + 0.45 * np.exp(-(fy ** 2) / 1.2))[:, None]
    near_rack = (np.abs(fy) > 0.6) & (np.abs(fy) < 1.8)
    floor[near_rack] = np.asarray(H["racks_rgb"])[None]
    out[dn] = floor[dn]
    # rack fronts close to the horizon (the racks rise from the floor at |y| 0.6 .. 1.8 to 2.25 m)
    lat = np.abs(d[:, 1]) > 0.35
    tr = 0.6 / np.maximum(np.abs(d[:, 1]), 1e-6)
    zr = p[2] + d[:, 2] * tr
    in_rack = lat & (zr > 0.0) & (zr < 2.25)
    out[in_rack] = rack[in_rack]
    return out


def hall_ibl(size=64, levels=None):
    """KTX1 bytes of the hall's image light (prefiltered levels + irradiance SH), deterministic."""
    n = size
    nlev = levels or (int(math.log2(size)) + 1)
    out = []
    for i in range(nlev):
        d, _ = cube_dirs(n)
        tt = i / max(1, nlev - 1)
        pr = 1.0 - math.sqrt(max(0.0, 1.0 - tt))
        blur = max(2.0 / n, (pr * pr) * 1.1)
        rad = _radiance(_to_world(d).reshape(-1, 3), blur).reshape(6, n, n, 3)
        if blur > 0.05:          # rougher levels: a box-filtered copy across each face approximates the wide lobe
            k = max(1, int(round(blur * n / 2)))
            from scipy.ndimage import uniform_filter
            rad = np.stack([uniform_filter(rad[f], size=(2 * k + 1, 2 * k + 1, 1), mode="nearest") for f in range(6)])
        out.append(rad.astype(np.float32))
        n = max(1, n // 2)
    d, dw = cube_dirs(32)
    rad = _radiance(_to_world(d).reshape(-1, 3), 0.2).reshape(6, 32, 32, 3)
    sh = irradiance_sh(rad, dw, d)
    return write_ktx(out, sh)
