#!/usr/bin/env python3
"""Checks of the cameras that need no simulator (maintainers' tool; the grader never runs it).

    python3 test_sensors.py

1. The depth sensor model (`packcell/sensors.py`) against its data sheet (`series.SENSORS`, the interface document's
   section 5), on synthetic scenes: the noise's size and its growth with distance, its spatial correlation and its
   independence between frames, the depth steps of the disparity grid, the working range, no reading at depth jumps,
   in the stereo pair's occlusion, on dark and on bright surfaces, and what that leaves of a head's height: one
   overhead frame does not tell a 4.5 mm head from a 6 mm one (a third typed wrong at best), 32 averaged frames do,
   and so does one close wrist frame.
2. Sealant partly over heads (rule 3): how often, how large, how much of a head it covers, as the manual says.
3. A pack's appearance (rule 4): the hall's light, grime, dust and labels within the manual's ranges; the depth
   independent of the light.
4. The calibration is not in the numbers a program has: every graded and development pack carries its cell's camera
   offsets, drawn under tests/review/select_set.py's rule 2, within series.MOUNT_TOL; what a program can compute from
   the visible files is the nominal mounting (cell.json, the client library, a wrist frame's `pose`), and the true
   camera is that times the cell's offset, which no visible file holds; located through the nominal poses, a point
   is off by millimetres to centimetres in every cell; no two cells share an offset. Neither an offset nor a sealant
   seed is in a visible file.
"""

import glob
import json
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
TASK = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.dont_write_bytecode = True

from packcell import kin, sensors, wire  # noqa: E402
from packcell import series as S  # noqa: E402

GREY = 128


def scene(name, z, h=240, w=320, grey=GREY):
    """A flat surface at distance z filling a h x w crop of camera `name` (K as the camera's), grey colour."""
    K = S.camera_K(name)
    return np.full((h, w), float(z)), np.full((h, w, 3), grey, np.uint8), K


def noise_checks():
    checks = 0
    rng = np.random.default_rng(7)
    so, sw = S.SENSORS["overhead"], S.SENSORS["wrist"]
    # the noise's size: sigma_k z^2 on a well-lit surface, overhead at the fixture and wrist close up; unbiased
    for name, z in (("overhead", 1.65), ("overhead", 1.0), ("overhead", 2.4), ("wrist", 0.35), ("wrist", 0.7)):
        zt, rgb, K = scene(name, z)
        m = sensors.measure(zt, rgb, K, S.SENSORS[name], rng)
        v = m > 0
        assert v.mean() > 0.999, (name, z, v.mean())
        e = (m[v] - z)
        want = S.SENSORS[name]["sigma_k"] * z * z
        assert abs(e.std() / math.hypot(want, sensors.step(S.SENSORS[name], K, z) / math.sqrt(12)) - 1.0) < 0.12, (name, z, e.std(), want)
        assert abs(e.mean()) < 0.15 * want, (name, z, e.mean())
    checks += 1
    # the depth steps: without noise every reading lies on the disparity grid (z = f b / (k step))
    for name, z in (("overhead", 1.65), ("wrist", 0.35)):
        spec = dict(S.SENSORS[name], sigma_k=0.0)
        zt, rgb, K = scene(name, z)
        zt = zt + np.linspace(-0.01, 0.01, zt.shape[1])[None, :]
        m = sensors.measure(zt, rgb, K, spec, rng)
        fb = K[0][0] * spec["baseline_m"]
        k = fb / m[m > 0] / spec["subpixel"]
        assert np.abs(k - np.round(k)).max() < 1e-3, name
        steps = np.diff(np.unique(m[m > 0]))
        assert abs(np.median(steps) / sensors.step(spec, K, z) - 1.0) < 0.05, (name, np.median(steps))
    checks += 1
    # spatially correlated over a few pixels, independent from frame to frame; the mean of many frames converges
    zt, rgb, K = scene("overhead", 1.65, 400, 400)
    fr = [sensors.measure(zt, rgb, K, so, rng) - 1.65 for _ in range(64)]
    a = fr[0]
    c1 = np.corrcoef(a[:, :-1].ravel(), a[:, 1:].ravel())[0, 1]
    c8 = np.corrcoef(a[:, :-8].ravel(), a[:, 8:].ravel())[0, 1]
    ct = np.corrcoef(fr[0].ravel(), fr[1].ravel())[0, 1]
    assert c1 > 0.8 and abs(c8) < 0.25 and abs(ct) < 0.05, (c1, c8, ct)
    mean64 = np.mean(fr, axis=0)
    assert mean64.std() < 0.2 * a.std(), (mean64.std(), a.std())
    checks += 1
    # the working range
    for name, z in (("overhead", 0.45), ("overhead", 3.1), ("wrist", 0.06), ("wrist", 1.05)):
        zt, rgb, K = scene(name, z, 60, 60)
        assert (sensors.measure(zt, rgb, K, S.SENSORS[name], rng) == 0).all(), (name, z)
    checks += 1
    # dark and bright surfaces: no reading at all at the dark end and the bright end, about half midway, all above
    # (below) the thresholds; dropouts in blobs (neighbours agree far more often than chance)
    spec = so
    for grey, lo, hi in ((spec["dark"][0] - 1, 1.0, 1.0), ((spec["dark"][0] + spec["dark"][1]) / 2, 0.35, 0.65),
                         (spec["dark"][1] + 5, 0.0, 0.0), (spec["bright"][1] + 2, 1.0, 1.0),
                         ((spec["bright"][0] + spec["bright"][1]) / 2, 0.35, 0.65), (spec["bright"][0] - 5, 0.0, 0.0)):
        zt, rgb, K = scene("overhead", 1.65, 300, 300, grey=int(round(grey)))
        lost = sensors.measure(zt, rgb, K, spec, rng) == 0
        assert lo - 1e-9 <= lost.mean() <= hi + 1e-9, (grey, lost.mean())
        if 0.2 < lost.mean() < 0.8:
            same = np.mean(lost[:, :-1] == lost[:, 1:])
            assert same > 0.75, (grey, same)
    # dim surfaces are noisier (up to twice at the dark end)
    zt, rgb, K = scene("overhead", 1.65, 300, 300, grey=spec["dark"][1] + 2)
    m = sensors.measure(zt, rgb, K, spec, rng)
    assert m[m > 0].std() > 1.3 * spec["sigma_k"] * 1.65 ** 2, m[m > 0].std()
    checks += 1
    # depth jumps: a 50 mm step at the fixture loses its edge, a 6 mm head's edge is kept; the occlusion band lies on
    # the camera's -x side of a raised block and is about its disparity difference wide
    zt, rgb, K = scene("overhead", 1.65, 120, 200)
    zt[:, 100:] = 1.60
    lost = sensors.measure(zt, rgb, K, dict(so, sigma_k=0.0), rng) == 0
    assert lost[:, 98:102].all() and lost[:, :90].mean() == 0.0 and lost[:, 110:].mean() == 0.0, np.where(lost[60])[0]
    zt, rgb, K = scene("overhead", 1.65, 120, 200)
    zt[:, 100:] = 1.644
    lost = sensors.measure(zt, rgb, K, dict(so, sigma_k=0.0), rng) == 0
    assert lost.mean() == 0.0
    zt, rgb, K = scene("overhead", 1.65, 60, 300)
    zt[:, 150:200] = 1.65 - 0.16                          # a lid's edge above the fixture
    occ = sensors._occluded(zt, K[0][0], so["baseline_m"])
    dd = K[0][0] * so["baseline_m"] * (1 / 1.49 - 1 / 1.65)
    band = np.where(occ[30])[0]
    assert band.size and band.max() < 150 and band.min() >= 150 - math.ceil(dd) - 1 and abs(band.size - dd) < 2, (band, dd)
    checks += 1
    # a head's height: 4.5 mm (T30) against 6.0 mm (S10) on a seat, the top's centre known (the best case)
    def typing_errors(name, z, frames, trials):
        spec = S.SENSORS[name]
        K = S.camera_K(name)
        f = K[0][0]
        px = 0.006 * f / z                                  # a head's radius in pixels
        n = int(8 * px) | 1
        yy, xx = np.mgrid[:n, :n] - n // 2
        r = np.hypot(xx, yy)
        top, ring = r < 0.8 * px, (r > 1.9 * px) & (r < 2.5 * px)
        wrong = 0
        for t in range(trials):
            h = 0.0045 if t % 2 else 0.006
            zt = np.where(r < px, z - h, z)
            rgb = np.full((n, n, 3), GREY, np.uint8)
            acc = np.zeros((n, n))
            cnt = np.zeros((n, n))
            for _ in range(frames):
                m = sensors.measure(zt, rgb, K, spec, rng)
                acc += m
                cnt += m > 0
            avg = np.where(cnt > 0, acc / np.maximum(cnt, 1), np.nan)
            est = np.nanmean(avg[ring]) - np.nanmean(avg[top])
            wrong += (est > 0.00525) != (h > 0.00525)
        return wrong / float(trials)
    one, many, wrist = typing_errors("overhead", 1.65, 1, 200), typing_errors("overhead", 1.65, 32, 100), typing_errors("wrist", 0.35, 1, 200)
    assert one > 0.2 and many < 0.05 and wrist < 0.01, (one, many, wrist)
    checks += 1
    print("sensor model: noise sigma at 1.65 m %.2f mm (overhead), at 0.35 m %.2f mm (wrist); a head typed wrong from "
          "one overhead frame %.0f %%, from 32 averaged %.0f %%, from one wrist frame %.1f %%" % (
              so["sigma_k"] * 1.65 ** 2 * 1e3, sw["sigma_k"] * 0.35 ** 2 * 1e3, 100 * one, 100 * many, 100 * wrist))
    return checks


def visible_files():
    out = [os.path.join(TASK, "instruction.md")]
    for p in glob.glob(os.path.join(TASK, "environment", "setup", "payload", "**", "*"), recursive=True):
        if os.path.isfile(p) and os.path.splitext(p)[1] in (".md", ".json", ".py", ".sh", ".txt"):
            out.append(p)
    return out


def leak_checks():
    checks = 0
    with open(os.path.join(TESTS, "hidden", "selection.json")) as f:
        sel = json.load(f)
    if "cells_key_hex" not in sel:
        print("calibration: SKIPPED, the cells' offsets are not drawn yet (tests/review/select_set.py, rule 2)")
        return checks
    with open(os.path.join(TESTS, "hidden", "packs.json")) as f:
        hidden = json.load(f)["packs"]
    with open(os.path.join(TASK, "environment", "commission_service", "dev_packs.json")) as f:
        dev = json.load(f)["packs"]
    sys.path.insert(0, HERE)
    import select_set
    key2 = bytes.fromhex(sel["cells_key_hex"])
    # every pack carries its cell's offsets, as rule 2 derives them, within the tolerance
    assert [p["cameras"] for p in hidden] == select_set.cells(key2, b"hidden", len(hidden))
    assert [p["cameras"] for p in dev] == select_set.cells(key2, b"dev", len(dev))
    cells = [p["cameras"] for p in hidden + dev]
    for cams in cells:
        for name, off in cams.items():
            tol = S.MOUNT_TOL[name]
            assert len(off) == 6 and all(abs(v) <= tol["mm"] for v in off[:3]) and all(abs(v) <= tol["deg"] for v in off[3:])
    keys = [json.dumps(c, sort_keys=True) for c in cells]
    assert len(set(keys)) == len(keys), "two cells share their offsets"
    checks += 1
    # what a program can compute: the nominal mounting (cell.json, the client library, a wrist frame's pose)
    with open(os.path.join(TASK, "environment", "setup", "payload", "given", "cell", "cell.json")) as f:
        pub = json.load(f)
    for name in S.CAMERAS:
        c = pub["cameras"][name]
        T_file = kin.pose(np.array(c["R"], dtype=float), np.array(c["pos"], dtype=float))
        assert np.abs(T_file - S.camera_mount(name)).max() < 1e-8, name
        assert c["pose_is"] == "nominal" and c["mount_tolerance"] == S.MOUNT_TOL[name]
    assert np.abs(wire._wrist_T() - S.camera_mount("wrist")).max() == 0.0
    # the true camera is the nominal one times the cell's offset, and nothing visible holds the offset
    errs_o, errs_w = [], []
    pts = [np.array([x, y, S.TABLE_Z + S.Z["lid_seat"]]) for x in (0.56, 0.86, 1.16) for y in (-0.45, 0.0, 0.45)]

    def hit(Tc, Tref, p):
        pc = Tref[:3, :3].T @ (p - Tref[:3, 3])
        ray = Tc[:3, :3] @ (pc / pc[2])
        s = (p[2] - Tc[2, 3]) / ray[2]
        return Tc[:3, 3] + s * ray
    for cams in cells:
        for name in S.CAMERAS:
            T_true = S.camera_mount(name, cams[name])
            est = np.linalg.inv(S.camera_mount(name)) @ T_true
            assert np.abs(est - S.mount_offset_T(cams[name])).max() < 1e-9
        To, Tn = S.camera_mount("overhead", cams["overhead"]), S.camera_mount("overhead")
        errs_o.append(max(np.linalg.norm(hit(Tn, To, p)[:2] - p[:2]) for p in pts))
        Wt, Wn = S.camera_mount("wrist", cams["wrist"]), S.camera_mount("wrist")
        q = np.array([0.0, 0.0, 0.35, 1.0])
        errs_w.append(float(np.linalg.norm((Wt @ q)[:3] - (Wn @ q)[:3])))
    assert min(errs_o) > 0.005 and min(errs_w) > 0.0012, (min(errs_o), min(errs_w))
    text = "\n".join(open(p, errors="replace").read() for p in visible_files())
    nums = set(re.findall(r"-?\d+\.\d+", text))
    found = [v for cams in cells for off in cams.values() for v in off if abs(v) >= 0.1 and ("%g" % v) in nums]
    assert not found, ("an offset's value appears in a visible file", found)
    seeds = [str(p[k]) for p in hidden + dev for k in ("seed", "sealant_seed", "appearance_seed") if k in p]
    assert not any(sd in text for sd in seeds), "a pack's seed appears in a visible file"
    assert not any("commission_service" in p for p in visible_files())
    checks += 1
    print("calibration: %d cells (6 graded, %d development), each camera off its nominal mounting; located through the "
          "nominal poses a point on the fixture is off by %.0f to %.0f mm (overhead) and a point 0.35 m in front of the "
          "wrist camera by %.1f to %.1f mm; no offset value in the %d visible files" % (
              len(cells), len(dev), 1e3 * min(errs_o), 1e3 * max(errs_o), 1e3 * min(errs_w), 1e3 * max(errs_w), len(visible_files())))
    return checks


def covered_fraction(head, sl, res=0.0001):
    """The share of a head's top under a dab of sealant (where the dome stands above the head's top)."""
    import cv2
    hd = S.HEADS[head]
    g = np.arange(-0.02, 0.02, res)
    X, Y = np.meshgrid(g, g)
    if hd["kind"] == "hex":
        ac = hd["af"] / math.sqrt(3)
        pts = np.array([[ac * math.cos(math.radians(60 * k)), ac * math.sin(math.radians(60 * k))] for k in range(6)])
        m = np.zeros(X.shape, np.uint8)
        cv2.fillPoly(m, [((pts + 0.02) / res).astype(np.int32)], 1)
        top = m > 0
    else:
        top = np.hypot(X, Y) < hd["head_r"]
    cx, cy = sl["d"] * math.cos(sl["angle"]), sl["d"] * math.sin(sl["angle"])
    q = 1 - ((X - cx) ** 2 + (Y - cy) ** 2) / sl["r"] ** 2
    z = np.where(q > 0, sl["h"] * np.sqrt(np.maximum(q, 0)), 0.0)
    return float((top & (z > hd["height"])).sum()) / float(top.sum())


def sealant_checks():
    """3. Sealant (series.SEALANT, rule 3): how often, where and how large, as the service manual states it."""
    checks = 0
    seen = {g: [0, 0] for g in S.SEALANT["p"]}
    cover = []
    rng = np.random.default_rng(3)
    for k in range(240):
        variant, harness = ("P4", "P5", "P6")[k % 3], ("none", "lid", "busbars")[(k // 3) % 3]
        p = S.draw_pack(int(rng.integers(0, 2 ** 63)), variant=variant, harness=harness, sealant_seed=int(rng.integers(0, 2 ** 63)))
        for f in p["fasteners"]:
            sl = f.get("sealant")
            if f["missing"]:
                assert sl is None
                continue
            seen[f["group"]][1] += 1
            if not sl:
                continue
            seen[f["group"]][0] += 1
            R = S.head_top_radius(f["head"])
            above = sl["h"] - S.HEADS[f["head"]]["height"]
            assert S.SEALANT["d"][0] * R - 1e-6 <= sl["d"] <= S.SEALANT["d"][1] * R + 1e-6
            assert S.SEALANT["r"][0] * R - 1e-6 <= sl["r"] <= S.SEALANT["r"][1] * R + 1e-6
            assert S.SEALANT["above"][0] - 1e-6 <= above <= S.SEALANT["above"][1] + 1e-6
            if len(cover) < 600:
                cover.append(covered_fraction(f["head"], sl))
    for g, (n_on, n) in seen.items():
        assert abs(n_on / float(n) - S.SEALANT["p"][g]) < 0.04, (g, n_on, n)
    med, top = float(np.median(cover)), float(np.max(cover))
    assert 0.12 <= med <= 0.28 and top <= 0.55, (med, top)
    # without a sealant seed (the canary, the photographs' example pack) there is none
    p = S.draw_pack(7, variant="P5", harness="none")
    assert not any(f.get("sealant") for f in p["fasteners"])
    checks += 1
    print("sealant: on %s of the fasteners (lid, busbar, bracket); a dab covers a median %.0f %% of a head's top, at "
          "most %.0f %% in %d dabs" % (", ".join("%.2f" % (seen[g][0] / float(seen[g][1])) for g in ("lid", "busbar", "bracket")),
                                     100 * med, 100 * top, len(cover)))
    return checks


def appearance_checks():
    """4. A pack's appearance (series.APPEARANCE, rule 4): the draw within the manual's ranges, labels clear of bolts,
    handle and ribs, and a surface's depth readings the same under any hall light."""
    A = S.APPEARANCE
    rng = np.random.default_rng(5)
    for k in range(150):
        variant = ("P4", "P5", "P6")[k % 3]
        p = S.draw_pack(int(rng.integers(0, 2 ** 63)), variant=variant, harness="none", appearance_seed=int(rng.integers(0, 2 ** 63)))
        a = p["appearance"]
        assert A["gain"][0] <= a["gain"] <= A["gain"][1] and A["kelvin"][0] <= a["kelvin"] <= A["kelvin"][1]
        assert len(a["lamps"]) == 4 and all(A["lamp"][0] <= v <= A["lamp"][1] for v in a["lamps"])
        assert A["patches"][0] <= len(a["patches"]) <= A["patches"][1]
        assert all(A["patch_axes"][0] <= q[ax] <= A["patch_axes"][1] for q in a["patches"] for ax in ("a", "b"))
        assert A["labels"][0] <= len(a["labels"]) <= A["labels"][1], len(a["labels"])
        bolts = [f["nominal"] for f in p["fasteners"] if f["group"] == "lid"]
        for lb in a["labels"]:
            rad = 0.5 * math.hypot(lb["w"], lb["h"])
            # every corner of the label clear of every bolt's hole (2 mm tolerance) and head by the clearance
            assert all(math.hypot(lb["x"] - bx, lb["y"] - by) > rad + A["label_clear"] for bx, by in bolts)
            assert abs(lb["x"]) + rad < 0.29 and abs(lb["y"]) + rad < 0.48
            for axis, c in S.VARIANTS[variant]["ribs"]:
                assert abs((lb["y"] if axis == "x" else lb["x"]) - c) > 0.015 + rad
        fast = {f["id"]: f for f in p["fasteners"]}
        assert all(not fast[d]["corroded"] and not fast[d]["missing"] for d in a["dusty"])
    # the same surface under a dimmer and a brighter hall: the same dropouts (the colour scaled by the light, the
    # sensor told the light); without the correction a dim hall would lose the dark lid's readings
    zt, rgb, K = scene("overhead", 1.65, 300, 300, grey=46)
    lost = []
    # a render under gain g: the nominal colour through the display curve the other way (inverse of display_lut)
    lin = np.interp((np.arange(256) / 255.0) ** 2.2, sensors._ACES_Y, sensors._ACES_X)

    def render(img, g):
        lut = np.clip(np.round(255.0 * sensors._aces(lin * g) ** (1.0 / 2.2)), 0, 255).astype(np.uint8)
        return lut[img]
    for g in (0.6, 1.0, 1.3):
        dim = render(rgb, g)
        lost.append(float((sensors.measure(zt, dim, K, S.SENSORS["overhead"], np.random.default_rng(1), light_gain=g) == 0).mean()))
        if g == 0.6:
            uncorrected = float((sensors.measure(zt, dim, K, S.SENSORS["overhead"], np.random.default_rng(1)) == 0).mean())
    assert max(lost) - min(lost) < 0.02 and uncorrected > 0.1, (lost, uncorrected)
    zt, rgb, K = scene("wrist", 0.35, 200, 200, grey=205)
    bright = [float((sensors.measure(zt, render(rgb, g), K, S.SENSORS["wrist"], np.random.default_rng(2), light_gain=g) == 0).mean())
              for g in (0.6, 1.0, 1.3)]
    assert max(bright) - min(bright) < 0.06, bright
    print("appearance: 150 draws within the manual's ranges, labels clear of bolts, handle and ribs; a grey-46 surface "
          "loses %.0f %% of its readings at any hall light (%.0f %% in a dim hall without the correction)" % (100 * max(lost), 100 * uncorrected))
    return 1


def main():
    checks = noise_checks() + sealant_checks() + appearance_checks() + leak_checks()
    print("ok: %d checks" % checks)


if __name__ == "__main__":
    main()
