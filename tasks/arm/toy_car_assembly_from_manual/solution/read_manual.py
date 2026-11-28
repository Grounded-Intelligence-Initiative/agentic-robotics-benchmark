"""Read a kit's numbers off its picture manual: the scale from the ruler, the sizes from the orthographic views.

    python3 read_manual.py <manual_dir> [out.json]

Each step picture is two panels: a three-quarter view (left) and an orthographic view (right) drawn at the same scale
as the ruler under it (1 mm ticks, longer ticks every 5 mm and 10 mm). The right panel is read:
    step_2_axles   from above:      chassis length and width; the axles' length, diameter and positions (notches)
    step_3_wheels  from the front:  wheel diameter and thickness; axle height over the chassis bottom; chassis height
    step_4_cabin   from the side:   cabin length and height
    step_5_done    from above:      cabin width
Sizes are rounded to the millimetre (the kits are made in whole millimetres).
"""

import json
import os
import sys

import numpy as np
from PIL import Image
from scipy import ndimage

PANEL = 640
SEP = 4


def right_panel(src):
    """The orthographic panel of a picture (a file path or an RGB array)."""
    if isinstance(src, str):
        src = Image.open(src).convert("RGB")
    im = np.asarray(src)[..., :3].astype(np.int16)
    return im[:, PANEL + SEP:PANEL + SEP + PANEL] if im.shape[1] > PANEL + SEP else im


def classes(im):
    R, G, B = im[..., 0], im[..., 1], im[..., 2]
    mx, mn = im.max(axis=2), im.min(axis=2)
    return {
        "red": (R > 130) & (G < 95) & (B < 85) & (R > 2.2 * G),
        "birch": (R > 140) & (G > 110) & (B > 60) & (R - B > 35) & (R < 1.4 * G),
        "dark": (mx < 75) & (mx - mn < 30),
        "blue": (B > 140) & (R < 110) & (B > R + 70),
        "window": (B > 60) & (B < 140) & (R < 90) & (B > R + 20) & (mx - mn > 25),
    }


def ruler_scale(im):
    """Pixels per millimetre from the horizontal ruler: the longest dark row, and its 10 mm ticks below it."""
    dark = classes(im)["dark"]
    best = None
    for r in range(im.shape[0]):
        lab, n = ndimage.label(dark[r])
        if n == 0:
            continue
        sizes = ndimage.sum(np.ones_like(lab), lab, range(1, n + 1))
        k = int(np.argmax(sizes))
        if best is None or sizes[k] > best[0]:
            cols = np.nonzero(lab == k + 1)[0]
            best = (sizes[k], r, cols.min(), cols.max())
    _n, r0, c0, c1 = best
    # tick length below the bar, per column
    below = np.zeros(im.shape[1], int)
    for c in range(c0, c1 + 1):
        h = 0
        while r0 + 1 + h < im.shape[0] and dark[r0 + 1 + h, c]:
            h += 1
        below[c] = h
    long_cols = np.nonzero(below >= 10)[0]
    groups = np.split(long_cols, np.nonzero(np.diff(long_cols) > 1)[0] + 1)
    starts = np.array([g[0] for g in groups if len(g)], float)
    k = np.arange(len(starts), dtype=float)
    spacing = np.polyfit(k, starts, 1)[0]                     # px per 10 mm, fitted over the whole ruler
    return spacing / 10.0, r0


def blobs(mask, min_px=30):
    mask = ndimage.binary_fill_holes(ndimage.binary_closing(mask, np.ones((3, 3), bool)))
    lab, n = ndimage.label(mask)
    out = []
    for k in range(1, n + 1):
        rows, cols = np.nonzero(lab == k)
        if len(rows) >= min_px:
            out.append({"r0": rows.min(), "r1": rows.max(), "c0": cols.min(), "c1": cols.max(),
                        "rc": rows.mean(), "cc": cols.mean(), "n": len(rows)})
    return out


def extent(b, s):
    """(width, height) in mm of a blob's bounding box at s px/mm."""
    return (b["c1"] - b["c0"] + 1) / s, (b["r1"] - b["r0"] + 1) / s


def read(manual_dir):
    """A manual from its PNG files in manual_dir."""
    return read_images({name: os.path.join(manual_dir, name + ".png")
                        for name in ("step_2_axles", "step_3_wheels", "step_4_cabin", "step_5_done")})


def decode_manual(manual):
    """The manual as a controller receives it ([{"name", "png_b64"}]) -> {name: RGB array}."""
    import base64
    import io
    return {m["name"]: np.asarray(Image.open(io.BytesIO(base64.b64decode(m["png_b64"]))).convert("RGB"))
            for m in manual}


def read_images(pages):
    """The kit's numbers from the pictures {name: file path or RGB array} (names without ".png")."""
    p = lambda name: pages[name[:-4] if name.endswith(".png") else name]
    out = {}
    # step 2, from above: image right = +x, up = +y
    im = right_panel(p("step_2_axles.png"))
    s, rrow = ruler_scale(im)
    cl = classes(im)
    above = np.zeros(im.shape[:2], bool)
    above[:rrow - 2] = True
    reds = blobs(cl["red"] & above)                        # the axles cut the chassis's top in three
    ch = {"r0": min(b["r0"] for b in reds), "r1": max(b["r1"] for b in reds), "c0": min(b["c0"] for b in reds),
          "c1": max(b["c1"] for b in reds), "n": sum(b["n"] for b in reds)}
    L, W = extent(ch, s)
    axles = sorted(blobs(cl["birch"] & above), key=lambda b: -b["n"])[:2]
    alen = np.mean([extent(b, s)[1] for b in axles])
    adia = np.mean([extent(b, s)[0] for b in axles])
    notch = np.mean([abs(b["cc"] - (ch["c0"] + ch["c1"]) / 2) for b in axles]) / s
    # step 3, from the front: image right = +y, up = +z
    im = right_panel(p("step_3_wheels.png"))
    s3, rrow = ruler_scale(im)
    cl = classes(im)
    above = np.zeros(im.shape[:2], bool)
    above[:rrow - 2] = True
    chf = max(blobs(cl["red"] & above), key=lambda b: b["n"])
    wheels = sorted(blobs(cl["dark"] & above), key=lambda b: -b["n"])[:2]
    wdia = np.mean([extent(b, s3)[1] for b in wheels])
    # thickness from the tyre's rows near its top, clear of the hub (drawn a little proud of the tyre)
    dk = ndimage.binary_closing(cl["dark"] & above, np.ones((3, 3), bool))
    th = []
    for b in wheels:
        rows = range(int(b["r0"] + 0.06 * (b["r1"] - b["r0"])), int(b["r0"] + 0.25 * (b["r1"] - b["r0"])) + 1)
        th.append(np.median([dk[r, b["c0"]:b["c1"] + 1].sum() for r in rows]))
    wthk = np.mean(th) / s3
    chassis_h = extent(chf, s3)[1]
    zc = np.mean([(chf["r1"] + 0.5) - b["rc"] for b in wheels]) / s3     # wheel centre over the chassis bottom
    # step 4, from the side: image right = +x, up = +z
    im = right_panel(p("step_4_cabin.png"))
    s4, rrow = ruler_scale(im)
    cl = classes(im)
    cab = max(blobs(cl["blue"]), key=lambda b: b["n"])
    clen, chgt = extent(cab, s4)
    # step 5, from above
    im = right_panel(p("step_5_done.png"))
    s5, rrow = ruler_scale(im)
    cl = classes(im)
    cabt = max(blobs(cl["blue"]), key=lambda b: b["n"])
    cwid = extent(cabt, s5)[1]
    mm = lambda v: round(float(v)) / 1000.0
    out = {
        "chassis": {"length": mm(L), "width": mm(W), "height": mm(chassis_h), "notch_x": mm(notch),
                    "axle_height": round(float(zc), 1) / 1000.0},
        "axle": {"length": mm(alen), "dia": mm(adia)},
        "wheel": {"dia": mm(wdia), "thick": mm(wthk)},
        "cabin": {"length": mm(clen), "width": mm(cwid), "height": mm(chgt)},
        "scale_px_per_mm": [round(float(x), 3) for x in (s, s3, s4, s5)],
    }
    out["chassis"]["base_t"] = round(out["chassis"]["axle_height"] - out["axle"]["dia"] / 2, 4)
    return out


if __name__ == "__main__":
    kit = read(sys.argv[1])
    text = json.dumps(kit, indent=2)
    if len(sys.argv) > 2:
        with open(sys.argv[2], "w") as f:
            f.write(text + "\n")
    print(text)
