"""Procedural textures of the data-hall cell (cell internals): tileable physically based sets (albedo in sRGB, a
tangent-space normal map and an ORM map: ambient occlusion, roughness, metallic) for the hall's surfaces, and
single-use panel textures (a server's bezel, its rear I/O strip, a power supply's face, a switch's port face, a rack
rail with its U numbers, labels and asset tags with real text and bar codes). Everything is drawn with numpy and Pillow
from fixed seeds: the same pixels on every machine. Images are uint8 (rows top-down); `normal_from_height` and
`pack` build the normal and ORM maps.
"""

from __future__ import annotations

import functools
import math

import numpy as np
from PIL import Image, ImageDraw, ImageFont

FONT_DIR = "/usr/share/fonts/truetype/dejavu"


@functools.lru_cache(maxsize=None)
def font(size, bold=False, mono=False):
    name = "DejaVuSansMono" if mono else "DejaVuSans"
    if bold:
        name += "-Bold"
    try:
        return ImageFont.truetype(f"{FONT_DIR}/{name}.ttf", int(size))
    except OSError:
        return ImageFont.load_default()


# ------------------------------------------------------------------------------------------------ helpers
def _u8(img):
    return np.clip(np.round(np.asarray(img, np.float64) * 255.0), 0, 255).astype(np.uint8)


def value_noise(h, w, cells, seed, octaves=4, persistence=0.5):
    """Tileable value noise in [0, 1] (bilinear, wrapped)."""
    rng = np.random.default_rng(seed)
    out = np.zeros((h, w))
    amp, tot = 1.0, 0.0
    cy, cx = cells
    for _ in range(octaves):
        g = rng.random((cy, cx))
        y = np.linspace(0, cy, h, endpoint=False)
        x = np.linspace(0, cx, w, endpoint=False)
        y0, x0 = np.floor(y).astype(int), np.floor(x).astype(int)
        fy, fx = (y - y0)[:, None], (x - x0)[None, :]
        fy, fx = fy * fy * (3 - 2 * fy), fx * fx * (3 - 2 * fx)
        y1, x1 = (y0 + 1) % cy, (x0 + 1) % cx
        a = g[y0][:, x0] * (1 - fx) + g[y0][:, x1] * fx
        b = g[y1][:, x0] * (1 - fx) + g[y1][:, x1] * fx
        out += amp * (a * (1 - fy) + b * fy)
        tot += amp
        amp *= persistence
        cy, cx = cy * 2, cx * 2
    return out / tot


def normal_from_height(h, strength):
    """Tangent-space normal map (uint8) of a height field (rows top-down; tangent x = right, y = up), wrapped."""
    dx = (np.roll(h, -1, axis=1) - np.roll(h, 1, axis=1)) * 0.5
    dy = (np.roll(h, 1, axis=0) - np.roll(h, -1, axis=0)) * 0.5     # up = towards row 0
    n = np.stack([-dx * strength, -dy * strength, np.ones_like(h)], -1)
    n /= np.linalg.norm(n, axis=-1, keepdims=True)
    return _u8(n * 0.5 + 0.5)


def pack(ao, rough, metal):
    """ORM map: R ambient occlusion, G roughness, B metallic (floats or arrays in [0, 1])."""
    shape = None
    for a in (ao, rough, metal):
        if np.ndim(a) == 2:
            shape = np.shape(a)
    shape = shape or (4, 4)
    out = np.zeros(shape + (3,))
    out[..., 0] = ao
    out[..., 1] = rough
    out[..., 2] = metal
    return _u8(out)


def flat_normal(h=4, w=4):
    n = np.zeros((h, w, 3))
    n[..., 2] = 1.0
    return _u8(n * 0.5 + 0.5)


def _col(rgb, shape):
    return np.broadcast_to(np.asarray(rgb, float), tuple(shape) + (3,)).copy()


def hex_field(h, w, pitch_px, seed=0):
    """(distance to the nearest hex-lattice centre in pixels, cell id) for a pointy-top hex lattice of `pitch_px`
    centre spacing, tileable when w is a multiple of the pitch and h of pitch * sqrt(3)."""
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float64)
    a = pitch_px
    rh = a * math.sqrt(3) / 2.0
    best = np.full((h, w), 1e9)
    for oy in (0, 1):
        cy = np.round((yy - oy * rh) / (2 * rh)) * 2 * rh + oy * rh
        cx = np.round((xx - oy * a / 2) / a) * a + oy * a / 2
        for dy in (-2 * rh, 0, 2 * rh):
            for dx in (-a, 0, a):
                d = np.hypot(xx - (cx + dx), yy - (cy + dy))
                best = np.minimum(best, d)
    return best


# ------------------------------------------------------------------------------------------------ tileable sets
# set name: (pixels, metres covered by one repeat)
SETS = {
    "powder_black": (512, 0.5), "powder_grey": (512, 0.5), "brushed_alu": (512, 0.25), "hex_perf": (512, 0.072),
    "honeycomb": (512, 0.06), "floor_tile": (1024, 0.6), "floor_perf": (1024, 0.6), "ceiling": (512, 1.2),
    "wall_paint": (512, 2.0), "black_plastic": (256, 0.2), "rubber": (256, 0.2), "yellow_duct": (256, 0.5),
    "wire_mesh": (512, 0.2), "fan_hub": (256, 0.08),
}


def _powder(n, rgb, seed, rough=0.55):
    peel = value_noise(n, n, (32, 32), seed, octaves=3)
    fine = value_noise(n, n, (128, 128), seed + 1, octaves=2)
    h = 0.6 * peel + 0.4 * fine
    alb = _col(rgb, (n, n)) * (0.96 + 0.08 * (fine[..., None] - 0.5))
    return {"albedo": _u8(_srgb_from_lin(alb)), "normal": normal_from_height(h, 0.9),
            "orm": pack(1.0, rough + 0.08 * (peel - 0.5), 0.0)}


def _lin_from_srgb(c):
    c = np.asarray(c, np.float64)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def _srgb_from_lin(x):
    x = np.clip(np.asarray(x, np.float64), 0.0, 1.0)
    return np.where(x <= 0.0031308, x * 12.92, 1.055 * np.power(x, 1.0 / 2.4) - 0.055)


def make_set(name):
    n = SETS[name][0]
    if name == "powder_black":
        return _powder(n, (0.034, 0.036, 0.040), 11, 0.45)
    if name == "powder_grey":
        return _powder(n, (0.11, 0.115, 0.125), 12, 0.5)
    if name == "black_plastic":
        return _powder(n, (0.018, 0.018, 0.02), 13, 0.45)
    if name == "rubber":
        return _powder(n, (0.012, 0.012, 0.013), 14, 0.85)
    if name == "yellow_duct":
        s = _powder(n, (0.85, 0.55, 0.02), 15, 0.4)
        return s
    if name == "brushed_alu":
        rng = np.random.default_rng(21)
        streak = rng.random((n, 1)) * 0.6 + np.repeat(value_noise(n, 1, (64, 1), 22, octaves=1), 1, axis=1) * 0.4
        streak = np.repeat(streak, n, axis=1)
        streak = 0.7 * streak + 0.3 * value_noise(n, n, (256, 4), 23, octaves=2)
        alb = _col((0.62, 0.63, 0.65), (n, n)) * (0.92 + 0.16 * (streak[..., None] - 0.5))
        return {"albedo": _u8(_srgb_from_lin(alb)), "normal": normal_from_height(streak, 0.6),
                "orm": pack(1.0, 0.38 + 0.1 * (streak - 0.5), 0.75)}
    if name == "hex_perf":
        d = hex_field(n, n, n / 6.0)
        r = n / 6.0 * 0.40
        hole = np.clip((r - d) / 1.5, 0.0, 1.0)
        base = _col((0.028, 0.030, 0.033), (n, n))
        alb = base * (1 - hole[..., None]) + _col((0.012, 0.012, 0.013), (n, n)) * hole[..., None]
        h = 1.0 - hole
        return {"albedo": _u8(_srgb_from_lin(alb)), "normal": normal_from_height(h * 3.0, 1.2),
                "orm": pack(1.0 - 0.7 * hole, 0.6, 0.0)}
    if name == "honeycomb":
        d = hex_field(n, n, n / 8.0)
        cell = n / 8.0 * 0.5
        wall = np.clip((d - cell * 0.86) / 1.2, 0.0, 1.0)
        alb = _col((0.006, 0.006, 0.007), (n, n)) * (1 - wall[..., None]) + _col((0.05, 0.052, 0.056), (n, n)) * wall[..., None]
        return {"albedo": _u8(_srgb_from_lin(alb)), "normal": normal_from_height(wall * 4.0, 1.4),
                "orm": pack(0.35 + 0.65 * wall, 0.5, 0.2 * wall)}
    if name in ("floor_tile", "floor_perf"):
        rng_seed = 31 if name == "floor_tile" else 32
        speck = value_noise(n, n, (256, 256), rng_seed, octaves=2)
        mott = value_noise(n, n, (8, 8), rng_seed + 1, octaves=3)
        base = (0.50, 0.51, 0.52)
        alb = _col(base, (n, n)) * (0.92 + 0.10 * (mott[..., None] - 0.5)) * (0.95 + 0.1 * (speck[..., None] - 0.5))
        yy, xx = np.mgrid[0:n, 0:n]
        e = np.minimum(np.minimum(xx, n - 1 - xx), np.minimum(yy, n - 1 - yy)).astype(float)
        groove = np.clip(1.0 - e / 3.0, 0.0, 1.0)
        trim = (e < 14).astype(float)
        alb = alb * (1 - 0.25 * trim[..., None]) * (1 - 0.6 * groove[..., None])
        h = 1.0 - groove
        ao = 1.0 - 0.6 * groove
        rough = 0.62 + 0.0 * speck
        if name == "floor_perf":
            # a grate: rows of square openings in the middle of the tile
            k = 16
            cellp = (n - 80) / k
            gx = ((xx - 40) % cellp)
            gy = ((yy - 40) % cellp)
            inner = (xx > 40) & (xx < n - 40) & (yy > 40) & (yy < n - 40)
            hole = inner & (gx > cellp * 0.18) & (gx < cellp * 0.82) & (gy > cellp * 0.18) & (gy < cellp * 0.82)
            alb[hole] = np.asarray(_lin_from_srgb((0.06, 0.06, 0.065)))
            h = h - 0.8 * hole
            ao = ao * (1 - 0.7 * hole)
            alb[~hole & inner] *= 0.62
        return {"albedo": _u8(_srgb_from_lin(alb)), "normal": normal_from_height(h * 2.0, 1.0),
                "orm": pack(ao, rough, 0.0)}
    if name == "ceiling":
        yy, xx = np.mgrid[0:n, 0:n]
        grid = ((xx % (n // 2)) < 3) | ((yy % (n // 2)) < 3)
        mott = value_noise(n, n, (64, 64), 41, octaves=3)
        alb = _col((0.62, 0.63, 0.64), (n, n)) * (0.95 + 0.08 * (mott[..., None] - 0.5))
        alb[grid] = np.asarray(_lin_from_srgb((0.70, 0.71, 0.72)))
        return {"albedo": _u8(_srgb_from_lin(alb)), "normal": normal_from_height(grid * 1.0, 0.8),
                "orm": pack(1.0, 0.85, 0.0)}
    if name == "wall_paint":
        mott = value_noise(n, n, (16, 16), 51, octaves=4)
        alb = _col((0.40, 0.42, 0.44), (n, n)) * (0.94 + 0.1 * (mott[..., None] - 0.5))
        return {"albedo": _u8(_srgb_from_lin(alb)), "normal": normal_from_height(mott, 0.4),
                "orm": pack(1.0, 0.9, 0.0)}
    if name == "wire_mesh":
        yy, xx = np.mgrid[0:n, 0:n]
        p = n // 4
        wire = ((xx % p) < 6) | ((yy % p) < 6)
        alb = _col((0.01, 0.01, 0.01), (n, n))
        alb[wire] = np.asarray(_lin_from_srgb((0.08, 0.08, 0.085)))
        return {"albedo": _u8(_srgb_from_lin(alb)), "normal": normal_from_height(wire * 2.0, 1.0),
                "orm": pack(np.where(wire, 1.0, 0.3), 0.5, np.where(wire, 0.6, 0.0))}
    if name == "fan_hub":
        yy, xx = np.mgrid[0:n, 0:n].astype(float)
        c = (n - 1) / 2
        r = np.hypot(xx - c, yy - c) / c
        ang = np.arctan2(yy - c, xx - c)
        blade = (np.sin(7 * ang + 2.5 * r) > 0.2) & (r > 0.32) & (r < 0.95)
        hub = r <= 0.32
        alb = _col((0.004, 0.004, 0.005), (n, n))
        alb[blade] = np.asarray(_lin_from_srgb((0.04, 0.04, 0.045)))
        alb[hub] = np.asarray(_lin_from_srgb((0.06, 0.06, 0.065)))
        return {"albedo": _u8(_srgb_from_lin(alb)), "normal": flat_normal(n, n),
                "orm": pack(np.where(blade | hub, 1.0, 0.4), 0.5, 0.0)}
    raise KeyError(name)


_SET_CACHE = {}


def texture_set(name):
    if name not in _SET_CACHE:
        _SET_CACHE[name] = make_set(name)
    return _SET_CACHE[name]


# ------------------------------------------------------------------------------------------------ panel textures
def _canvas(w, h, rgb):
    return Image.new("RGB", (int(w), int(h)), tuple(int(round(255 * c)) for c in rgb))


def _hexpatch(img, box, pitch, wall_rgb, hole_rgb):
    """Fill a rectangle of a PIL image with a honeycomb (hole colour, walls)."""
    x0, y0, x1, y1 = [int(v) for v in box]
    w, h = x1 - x0, y1 - y0
    d = hex_field(h, w, pitch)
    cell = pitch * 0.5
    wall = np.clip((d - cell * 0.84) / 1.0, 0.0, 1.0)[..., None]
    a = np.asarray(hole_rgb, float) * (1 - wall) + np.asarray(wall_rgb, float) * wall
    img.paste(Image.fromarray(_u8(a)), (x0, y0))


def code128_like(data, height, module=2):
    """A bar-code image (black on white) whose bar pattern is derived from the text (a stand-in with Code 128's look;
    the cell's labels carry the readable text beside it, which is what a program reads)."""
    rng = np.random.default_rng(abs(hash(("bc", data))) % (2 ** 32))
    bars = [2, 1, 1, 2, 3, 2]
    for ch in data:
        v = (ord(ch) * 2654435761) & 0xFFFFFFFF
        for k in range(6):
            bars.append(1 + ((v >> (3 * k)) & 3))
    bars += [2, 3, 3, 1, 1, 1, 2]
    width = sum(bars) * module + 20
    img = Image.new("L", (width, height), 255)
    dr = ImageDraw.Draw(img)
    x = 10
    for i, b in enumerate(bars):
        if i % 2 == 0:
            dr.rectangle([x, 0, x + b * module - 1, height - 1], fill=0)
        x += b * module
    del rng
    return img.convert("RGB")


def label(lines, w_px, h_px, bg=(0.95, 0.95, 0.93), fg=(0.05, 0.05, 0.05), barcode=None, bold_first=True, border=True):
    """A printed label: text lines (and an optional bar code under them) on a sticker."""
    img = _canvas(w_px, h_px, bg)
    dr = ImageDraw.Draw(img)
    n = len(lines) + (1.6 if barcode else 0)
    fs = max(8, int(h_px / (n + 0.8) * 0.8))
    y = int(fs * 0.3)
    for i, t in enumerate(lines):
        f = font(fs if i else int(fs * 1.05), bold=(bold_first and i == 0))
        dr.text((int(w_px * 0.05), y), t, fill=tuple(int(255 * c) for c in fg), font=f)
        y += int(fs * 1.12)
    if barcode:
        bc = code128_like(barcode, int(fs * 1.4))
        bw = min(int(w_px * 0.9), bc.width)
        bc = bc.resize((bw, bc.height))
        img.paste(bc, (int(w_px * 0.05), y + 2))
    if border:
        dr.rectangle([0, 0, w_px - 1, h_px - 1], outline=(200, 200, 196), width=2)
    return np.asarray(img)


def rack_plate(text, w_px=256, h_px=96):
    """A rack's ID plate (TIA-606-B grid name): white engraved letters on black."""
    img = _canvas(w_px, h_px, (0.02, 0.02, 0.02))
    dr = ImageDraw.Draw(img)
    f = font(int(h_px * 0.62), bold=True)
    tw = dr.textlength(text, font=f)
    dr.text(((w_px - tw) / 2, h_px * 0.14), text, fill=(235, 235, 230), font=f)
    return np.asarray(img)


def rail_u(n_units, px_per_u=48, w_px=40):
    """A mounting rail's front strip: square cage-nut holes three per U and the U numbers (from the bottom)."""
    h = n_units * px_per_u
    img = _canvas(w_px, h, (0.035, 0.037, 0.04))
    dr = ImageDraw.Draw(img)
    f = font(int(px_per_u * 0.36))
    for u in range(n_units):
        y_top = h - (u + 1) * px_per_u
        for k, off in enumerate((0.18, 0.5, 0.82)):
            cy = y_top + off * px_per_u
            s = px_per_u * 0.16
            dr.rectangle([w_px * 0.55 - s, cy - s, w_px * 0.55 + s, cy + s], fill=(4, 4, 5))
        if (u + 1) % 1 == 0:
            dr.text((2, y_top + px_per_u * 0.30), "%d" % (u + 1), fill=(200, 200, 196), font=f)
    return np.asarray(img)


def blank_1u(w_px=880, h_px=88):
    img = _canvas(w_px, h_px, (0.03, 0.032, 0.035))
    dr = ImageDraw.Draw(img)
    for x in range(40, w_px - 40, 14):
        dr.line([(x, 14), (x, h_px - 14)], fill=(14, 14, 16), width=3)
    return np.asarray(img)


def node_bezel(w_px, h_px, hostname, brand="KESTREL", model="KX-8H"):
    """The GPU node's front bezel over the GPU tray: a satin-silver honeycomb grille (the fans show dark through it)
    in a graphite frame, the brand band along the bottom with the model name (the status light bar is a geom)."""
    img = _canvas(w_px, h_px, (0.16, 0.165, 0.175))
    m = int(h_px * 0.045)
    _hexpatch(img, (m, m, w_px - m, h_px - int(h_px * 0.15)), max(9, w_px / 40), (0.62, 0.63, 0.65), (0.10, 0.10, 0.11))
    dr = ImageDraw.Draw(img)
    yb = h_px - int(h_px * 0.14)
    dr.rectangle([m, yb, w_px - m, h_px - m], fill=(28, 29, 32))
    f = font(int(h_px * 0.072), bold=True)
    dr.text((m * 2, yb + int(h_px * 0.022)), brand, fill=(222, 224, 228), font=f)
    f2 = font(int(h_px * 0.05))
    tw = dr.textlength(model, font=f2)
    dr.text((w_px - m * 2 - tw, yb + int(h_px * 0.035)), model, fill=(165, 168, 172), font=f2)
    return np.asarray(img)


def drive_front(w_px=256, h_px=64, bay=0):
    """A U.2 drive carrier's front: a vented latch with a release button and the bay number (the LED is a geom)."""
    img = _canvas(w_px, h_px, (0.07, 0.072, 0.077))
    dr = ImageDraw.Draw(img)
    for x in range(int(w_px * 0.10), int(w_px * 0.62), 10):
        dr.rectangle([x, h_px * 0.25, x + 5, h_px * 0.75], fill=(12, 12, 13))
    dr.rectangle([w_px * 0.70, h_px * 0.22, w_px * 0.86, h_px * 0.78], fill=(225, 90, 20))   # orange: hot-swap touch point
    f = font(int(h_px * 0.32))
    dr.text((w_px * 0.02, h_px * 0.30), "%d" % bay, fill=(190, 190, 186), font=f)
    return np.asarray(img)


def rear_io(w_px, h_px, n_osfp=8):
    """A GPU node's rear I/O strip: eight flat-top OSFP cages (one per GPU, labelled 1-8), two storage QSFP cages,
    the BMC's RJ45 and a USB port, with the silkscreen beside each."""
    img = _canvas(w_px, h_px, (0.06, 0.062, 0.066))
    dr = ImageDraw.Draw(img)
    f = font(int(h_px * 0.18), bold=True)
    pitch = w_px * 0.088
    x0 = w_px * 0.04
    for i in range(n_osfp):
        x = x0 + i * pitch
        dr.rectangle([x, h_px * 0.30, x + pitch * 0.86, h_px * 0.86], fill=(110, 112, 116))
        dr.rectangle([x + 4, h_px * 0.36, x + pitch * 0.86 - 4, h_px * 0.80], fill=(6, 6, 7))
        dr.text((x + pitch * 0.28, h_px * 0.04), "%d" % (i + 1), fill=(225, 225, 220), font=f)
    xs = x0 + n_osfp * pitch + w_px * 0.02
    for k in range(2):
        x = xs + k * pitch * 0.9
        dr.rectangle([x, h_px * 0.34, x + pitch * 0.74, h_px * 0.82], fill=(100, 102, 106))
        dr.rectangle([x + 3, h_px * 0.40, x + pitch * 0.74 - 3, h_px * 0.76], fill=(6, 6, 7))
        dr.text((x + 2, h_px * 0.06), "S%d" % (k + 1), fill=(210, 210, 205), font=f)
    x = xs + 2 * pitch * 0.9 + w_px * 0.015
    dr.rectangle([x, h_px * 0.36, x + pitch * 0.55, h_px * 0.84], fill=(20, 20, 22))
    dr.rectangle([x + 4, h_px * 0.48, x + pitch * 0.55 - 4, h_px * 0.84], fill=(2, 2, 2))
    dr.text((x, h_px * 0.06), "BMC", fill=(210, 210, 205), font=font(int(h_px * 0.15), bold=True))
    return np.asarray(img)


def psu_face(w_px=256, h_px=140, idx=1, feed="A"):
    """A power supply's face: a fan grille, the inlet recess, the release tab (green) and its label."""
    img = _canvas(w_px, h_px, (0.05, 0.052, 0.056))
    dr = ImageDraw.Draw(img)
    gx0, gy0, gx1, gy1 = w_px * 0.05, h_px * 0.12, w_px * 0.52, h_px * 0.88
    for x in np.arange(gx0, gx1, 9):
        for y in np.arange(gy0, gy1, 9):
            dr.ellipse([x, y, x + 6, y + 6], fill=(4, 4, 5))
    dr.rectangle([w_px * 0.58, h_px * 0.18, w_px * 0.86, h_px * 0.82], fill=(14, 14, 15))       # inlet recess
    dr.rectangle([w_px * 0.90, h_px * 0.10, w_px * 0.97, h_px * 0.90], fill=(40, 150, 70))      # green release tab
    f = font(int(h_px * 0.13), bold=True)
    dr.text((w_px * 0.06, h_px * 0.0), "PSU%d %s" % (idx, feed), fill=(220, 220, 215), font=f)
    return np.asarray(img)


def switch_ports(w_px, h_px, n_per_row=16, rows=2, name="leaf"):
    """A 1U leaf switch's port face: two rows of finned OSFP cages numbered 1..32 (odd on top), the switch's name."""
    img = _canvas(w_px, h_px, (0.05, 0.052, 0.056))
    dr = ImageDraw.Draw(img)
    f = font(int(h_px * 0.13), bold=True)
    x0 = w_px * 0.10
    pitch = (w_px * 0.86) / n_per_row
    for r in range(rows):
        y0 = h_px * (0.08 + 0.46 * r)
        for i in range(n_per_row):
            x = x0 + i * pitch
            dr.rectangle([x, y0, x + pitch * 0.88, y0 + h_px * 0.38], fill=(120, 122, 126))
            for fx in np.arange(x + 3, x + pitch * 0.88 - 2, 4):
                dr.line([(fx, y0 + 2), (fx, y0 + h_px * 0.38 - 2)], fill=(80, 82, 86), width=1)
            port = 2 * i + 1 + r
            dr.text((x + 2, y0 + h_px * 0.38 - 2), "%d" % port, fill=(220, 220, 215), font=font(int(h_px * 0.09)))
    dr.text((w_px * 0.005, h_px * 0.38), name, fill=(220, 220, 215), font=f)
    return np.asarray(img)


def pdu_face(w_px=96, h_px=1600, feed="A", amps=11.4):
    """A vertical PDU's face: the outlets (C13 / C19) in groups, the metering display."""
    img = _canvas(w_px, h_px, (0.03, 0.032, 0.035))
    dr = ImageDraw.Draw(img)
    color = (200, 40, 30) if feed == "A" else (30, 90, 200)
    dr.rectangle([0, 0, w_px - 1, h_px * 0.03], fill=color)
    dr.rectangle([w_px * 0.15, h_px * 0.05, w_px * 0.85, h_px * 0.08], fill=(8, 30, 40))
    dr.text((w_px * 0.2, h_px * 0.055), "%.1fA" % amps, fill=(90, 220, 255), font=font(int(w_px * 0.2), mono=True))
    y = h_px * 0.11
    k = 0
    while y < h_px * 0.97:
        big = (k % 4 == 0)
        hh = w_px * (0.55 if big else 0.42)
        dr.rectangle([w_px * 0.18, y, w_px * 0.82, y + hh], fill=(12, 12, 13))
        dr.rectangle([w_px * 0.28, y + hh * 0.2, w_px * 0.72, y + hh * 0.8], fill=(2, 2, 2))
        y += hh + w_px * 0.25
        k += 1
    return np.asarray(img)


def sign(text, w_px=512, h_px=128, bg=(0.10, 0.32, 0.70), fg=(1, 1, 1)):
    img = _canvas(w_px, h_px, bg)
    dr = ImageDraw.Draw(img)
    f = font(int(h_px * 0.5), bold=True)
    tw = dr.textlength(text, font=f)
    dr.text(((w_px - tw) / 2, h_px * 0.2), text, fill=tuple(int(255 * c) for c in fg), font=f)
    return np.asarray(img)


def fan_module_front(w_px=180, h_px=520, idx=1):
    """A dual-rotor fan module's face: two round grilles with the blades behind, the module number, the orange pull
    handle's recess (the handle itself is a geom), a vent strip."""
    img = _canvas(w_px, h_px, (0.045, 0.047, 0.05))
    dr = ImageDraw.Draw(img)
    r = w_px * 0.44
    hub = texture_set("fan_hub")["albedo"]
    for k in range(2):
        cx, cy = w_px / 2, h_px * (0.30 + 0.42 * k)
        disk = Image.fromarray(hub).resize((int(2 * r), int(2 * r)))
        mask = Image.new("L", disk.size, 0)
        ImageDraw.Draw(mask).ellipse([0, 0, disk.size[0] - 1, disk.size[1] - 1], fill=255)
        img.paste(disk, (int(cx - r), int(cy - r)), mask)
        for rr in np.linspace(r * 0.25, r, 6):
            dr.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=(70, 72, 76), width=2)
        for a in range(0, 360, 45):
            dr.line([(cx, cy), (cx + r * math.cos(math.radians(a)), cy + r * math.sin(math.radians(a)))],
                    fill=(70, 72, 76), width=2)
    dr.text((w_px * 0.08, h_px * 0.015), "FAN%d" % idx, fill=(215, 215, 210), font=font(int(w_px * 0.15), bold=True))
    return np.asarray(img)


def tray_service_label(w_px=900, h_px=600):
    """The service label on the GPU tray's cover: the slot map and the rules a technician follows."""
    img = _canvas(w_px, h_px, (0.92, 0.92, 0.90))
    dr = ImageDraw.Draw(img)
    f = font(int(h_px * 0.055), bold=True)
    f2 = font(int(h_px * 0.04))
    dr.text((20, 14), "KX-8H GPU TRAY  -  SERVICE", fill=(20, 20, 20), font=f)
    lines = ["1  Shut down (BMC), switch off both feeds' outlets, wait for the standby LED to go dark.",
             "2  Open both blue levers, slide the tray to its service stop, fold the levers back.",
             "3  Cover up to its stay.  Module screws: long-reach T20, 0.9 N m, order 1-4-2-3, two passes.",
             "4  Lift a module straight up 15 mm on its guide pins before any other motion."]
    for i, t in enumerate(lines):
        dr.text((20, 80 + i * int(h_px * 0.065)), t, fill=(30, 30, 30), font=f2)
    gx0, gy0 = 60, int(h_px * 0.48)
    cw, ch = (w_px - 120) / 4, (h_px * 0.46) / 2
    # the map as the tray lies: the label's top edge towards the rear, so the top row is the back row and the bottom
    # row (next to "FRONT") the front row; the screws numbered 1 front-left, 2 front-right, 3 back-left, 4 back-right
    slots = [["SXM1", "SXM3", "SXM4", "SXM2"], ["SXM5", "SXM7", "SXM8", "SXM6"]]
    f3 = font(int(h_px * 0.032), bold=True)
    for r in range(2):
        for c in range(4):
            x0, y0 = gx0 + c * cw, gy0 + r * ch
            dr.rectangle([x0 + 6, y0 + 6, x0 + cw - 6, y0 + ch - 6], outline=(20, 20, 20), width=3)
            dr.text((x0 + cw * 0.28, y0 + ch * 0.40), slots[r][c], fill=(20, 20, 20), font=f)
            for n, (a, b) in (("3", (0.12, 0.18)), ("4", (0.88, 0.18)), ("1", (0.12, 0.82)), ("2", (0.88, 0.82))):
                dr.ellipse([x0 + cw * a - 7, y0 + ch * b - 7, x0 + cw * a + 7, y0 + ch * b + 7], outline=(20, 20, 20), width=2)
                dr.text((x0 + cw * a + (12 if a < 0.5 else -26), y0 + ch * b - 12), n, fill=(20, 20, 20), font=f3)
    dr.text((gx0, gy0 + 2 * ch + 4), "FRONT (fans)", fill=(60, 60, 60), font=f2)
    return np.asarray(img)


def pcb(w_px=1024, h_px=1024, seed=5):
    """A dark-green circuit board with traces, vias and silkscreen marks (tileable enough for a baseboard)."""
    rng = np.random.default_rng(seed)
    img = _canvas(w_px, h_px, (0.02, 0.10, 0.06))
    dr = ImageDraw.Draw(img)
    for _ in range(900):
        x, y = rng.integers(0, w_px), rng.integers(0, h_px)
        L = rng.integers(20, 160)
        if rng.random() < 0.5:
            dr.line([(x, y), (x + L, y)], fill=(10, 58, 34), width=int(rng.integers(1, 4)))
        else:
            dr.line([(x, y), (x, y + L)], fill=(10, 58, 34), width=int(rng.integers(1, 4)))
    for _ in range(1400):
        x, y = rng.integers(0, w_px), rng.integers(0, h_px)
        dr.ellipse([x - 2, y - 2, x + 2, y + 2], fill=(150, 130, 60))
    for _ in range(60):
        x, y = rng.integers(0, w_px - 60), rng.integers(0, h_px - 20)
        dr.text((x, y), "C%d" % rng.integers(1, 999), fill=(200, 205, 200), font=font(14))
    return np.asarray(img)


def aux_front(w_px=880, h_px=176, name="stor-a03-1"):
    """A 2U storage / CPU node's front: twelve 3.5-inch bays with their latches and LED windows, ears, the name."""
    img = _canvas(w_px, h_px, (0.10, 0.104, 0.11))
    dr = ImageDraw.Draw(img)
    bw = (w_px - 80) / 6
    for r in range(2):
        for c in range(6):
            x0, y0 = 40 + c * bw, 10 + r * (h_px - 20) / 2
            dr.rectangle([x0 + 3, y0 + 3, x0 + bw - 3, y0 + (h_px - 20) / 2 - 3], fill=(30, 31, 34))
            for xx in range(int(x0 + 10), int(x0 + bw * 0.7), 8):
                dr.rectangle([xx, y0 + 12, xx + 4, y0 + (h_px - 20) / 2 - 12], fill=(12, 12, 13))
            dr.rectangle([x0 + bw * 0.78, y0 + 10, x0 + bw * 0.92, y0 + (h_px - 20) / 2 - 10], fill=(150, 152, 156))
    dr.rectangle([0, 0, 34, h_px], fill=(45, 46, 50))
    dr.rectangle([w_px - 34, 0, w_px, h_px], fill=(45, 46, 50))
    dr.text((44, h_px - 26), name, fill=(200, 200, 196), font=font(18))
    return np.asarray(img)


def mgmt_switch(w_px=880, h_px=88, ports=48):
    """A 1U management switch's front: 48 RJ45 ports in two rows, four uplinks, the console port."""
    img = _canvas(w_px, h_px, (0.06, 0.062, 0.066))
    dr = ImageDraw.Draw(img)
    pw = (w_px * 0.70) / (ports / 2)
    for i in range(ports):
        c, r = i // 2, i % 2
        x0, y0 = 60 + c * pw, 10 + r * 36
        dr.rectangle([x0 + 2, y0, x0 + pw - 2, y0 + 30], fill=(140, 142, 146))
        dr.rectangle([x0 + 5, y0 + 6, x0 + pw - 5, y0 + 28], fill=(5, 5, 6))
    for k in range(4):
        x0 = 60 + w_px * 0.72 + k * 34
        dr.rectangle([x0, 22, x0 + 28, 50], fill=(120, 122, 126))
    return np.asarray(img)


def patch_panel(w_px=880, h_px=88, cassettes=4):
    """A 1U fibre patch panel: MPO cassettes with their adapter rows (the trunks plug in at the rear)."""
    img = _canvas(w_px, h_px, (0.05, 0.052, 0.056))
    dr = ImageDraw.Draw(img)
    cw = (w_px - 80) / cassettes
    for k in range(cassettes):
        x0 = 40 + k * cw
        dr.rectangle([x0 + 4, 8, x0 + cw - 4, h_px - 8], fill=(24, 25, 28))
        for j in range(12):
            xx = x0 + 14 + j * (cw - 28) / 12
            dr.rectangle([xx, 30, xx + (cw - 28) / 12 - 4, 58], fill=(10, 120, 140))
    return np.asarray(img)


def slot_silk(slots, w_px=880, h_px=144):
    """The tray's slot names (rows of four) as one silkscreen image: row r, column c fills cell (c, r)."""
    img = _canvas(w_px, h_px, (0.02, 0.10, 0.06))
    dr = ImageDraw.Draw(img)
    f = font(int(h_px * 0.30), bold=True)
    for r, row in enumerate(slots):
        for c, name in enumerate(row):
            x0, y0 = c * w_px / 4, r * h_px / 2
            tw = dr.textlength(name, font=f)
            dr.text((x0 + (w_px / 4 - tw) / 2, y0 + h_px * 0.08), name, fill=(232, 236, 232), font=f)
    return np.asarray(img)
