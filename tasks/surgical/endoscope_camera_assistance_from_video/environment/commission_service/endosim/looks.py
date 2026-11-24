"""Procedural tissue textures: a colour map and a normal map per surface, from a procedure's scene look.

Pixels only: nothing here is a scoring input, so numpy's own generator is fine. Runs in the renderer process
(numpy + OpenCV).
"""

import os

import cv2
import numpy as np


def _noise(rng, size, cell):
    """Smooth noise in [0, 1]: white noise on a coarse grid, upsampled bicubically."""
    n = max(4, int(size / cell))
    a = rng.random((n, n)).astype(np.float32)
    a = cv2.resize(a, (size, size), interpolation=cv2.INTER_CUBIC)
    lo, hi = float(a.min()), float(a.max())
    return (a - lo) / max(hi - lo, 1e-6)


def _fractal(rng, size, cell, octaves=4):
    out = np.zeros((size, size), np.float32)
    amp, total = 1.0, 0.0
    for _ in range(octaves):
        out += amp * _noise(rng, size, cell)
        total += amp
        amp *= 0.5
        cell = max(2.0, cell / 2.2)
    return out / total


def _vessels(rng, size, count, scale):
    """A soft mask in [0, 1] of branching vessels (trunks a millimetre wide down to capillaries)."""
    mask = np.zeros((size, size), np.float32)
    for _ in range(int(count)):
        p = rng.uniform(0, size, 2)
        heading = rng.uniform(0, 2 * np.pi)
        stack = [(p, heading, rng.uniform(4.0, 13.0) * scale, int(rng.integers(60, 220)), 1.0)]
        while stack:
            p, heading, width, length, level = stack.pop()
            pts = [p.copy()]
            for _k in range(length):
                heading += rng.normal(0.0, 0.11)
                p = p + 7.0 * scale * np.array([np.cos(heading), np.sin(heading)])
                pts.append(p.copy())
                if width > 1.6 * scale and rng.random() < 0.035:
                    stack.append((p.copy(), heading + rng.choice((-1, 1)) * rng.uniform(0.35, 1.0), width * 0.6, int(length * 0.55),
                                  level * 0.85))
            cv2.polylines(mask, [np.round(np.array(pts)).astype(np.int32)], False, float(level), max(1, int(round(width))), cv2.LINE_AA)
    return np.clip(cv2.GaussianBlur(mask, (0, 0), 2.2 * scale), 0.0, 1.0)


def tissue_maps(look, size=3072, wall=False):
    """(colour uint8 RGB, normal uint8 RGB) for the tissue floor, or the paler abdominal wall."""
    rng = np.random.default_rng(int(look["texture_seed"]) + (7919 if wall else 0))
    scale = size / 4096.0 * (2.0 if wall else 1.0)
    base, dark, pale = (np.array(look[k], np.float32) for k in ("base", "dark", "pale"))
    fat, vessel = np.array(look["fat"], np.float32), np.array(look["vessel"], np.float32)
    if wall:
        base = np.clip(0.5 * base + 0.5 * pale + look["wall_shift"], 0.0, 1.0)
        dark = 0.5 * dark + 0.5 * base
    s = float(look["blotch_scale"]) * scale
    big = _fractal(rng, size, 900.0 * s)
    mid = _fractal(rng, size, 240.0 * s)
    fine = _fractal(rng, size, 20.0 * scale, octaves=3)
    col = base[None, None, :] * (0.72 + 0.56 * mid[..., None])
    shade = np.clip((big - 0.42) * 2.2, 0, 1)[..., None]
    col = col * (1.0 - 0.6 * shade) + dark[None, None, :] * (0.6 * shade)
    pale_w = np.clip((mid - 0.55) * 3.0, 0, 1)[..., None] * 0.6
    col = col * (1.0 - pale_w) + pale[None, None, :] * pale_w
    # fat: lobules where smooth noise passes a threshold, with darker seams between the lobules
    lob = _fractal(rng, size, 260.0 * s, octaves=3)
    cover = (0.35 if wall else 1.0) * float(look["fat_cover"])
    fat_w = np.clip((lob - np.quantile(lob[::16, ::16], 1.0 - cover)) * 14.0, 0, 1)
    cells = _noise(rng, size, 46.0 * scale)
    seams = np.clip(1.0 - np.abs(cells - 0.5) * 11.0, 0, 1)
    fat_col = fat[None, None, :] * (0.82 + 0.36 * fine[..., None]) * (1.0 - 0.3 * seams[..., None])
    col = col * (1.0 - fat_w[..., None]) + fat_col * fat_w[..., None]
    ves = _vessels(rng, size, look["vessels"] * (0.6 if wall else 1.0) * 0.5, scale)
    col = col * (1.0 - 0.7 * ves[..., None]) + vessel[None, None, :] * (0.7 * ves[..., None])
    col = col * (0.9 + 0.2 * fine[..., None])
    colour = (np.clip(col, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)
    # relief: fine wrinkles, raised vessels, fat lobules
    height = 1.5 * fine + 1.2 * ves + 0.9 * fat_w * (1.0 - seams) + 0.8 * mid
    k = 4.5 * float(look["bump"]) * scale
    gx = cv2.Sobel(height, cv2.CV_32F, 1, 0, ksize=3) * k
    gy = cv2.Sobel(height, cv2.CV_32F, 0, 1, ksize=3) * k
    nrm = np.stack([-gx, -gy, np.ones_like(gx)], axis=-1)
    nrm /= np.linalg.norm(nrm, axis=-1, keepdims=True)
    normal = ((nrm * 0.5 + 0.5) * 255.0 + 0.5).astype(np.uint8)
    return colour, normal


def write_png(path, rgb):
    """Written under another name and moved into place: the renderer never opens a half-written file."""
    tmp = path + ".part.png"
    cv2.imwrite(tmp, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_PNG_COMPRESSION, 1])
    os.replace(tmp, path)
