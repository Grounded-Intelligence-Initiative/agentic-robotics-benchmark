#!/usr/bin/env python3
"""Checks of the cameras that need no simulator (maintainers' tool; the grader never runs it).

    python3 test_sensors.py

1. The depth sensor model (`packcell/sensors.py`) against its data sheet (`series.SENSORS`, the interface document's
   section 5), on synthetic scenes: the noise's size and its growth with distance, its spatial correlation and its
   independence between frames, the depth steps of the disparity grid, the working range, no reading at depth jumps,
   in the stereo pair's occlusion, on dark and on bright surfaces, and what that leaves of a head's height: one
   overhead frame does not tell a 4.5 mm head from a 6 mm one (a third typed wrong at best), 32 averaged frames do,
   and so does one close wrist frame.
2. The calibration is not in the numbers a program has: every graded and development pack carries its cell's camera
   offsets, drawn under tests/review/select_set.py's rule 2, within series.MOUNT_TOL; what a program can compute from
   the visible files is the nominal mounting (cell.json, the client library, a wrist frame's `pose`), and the true
   camera is that times the cell's offset, which no visible file holds; located through the nominal poses, a point
   is off by millimetres to centimetres in every cell; no two cells share an offset.
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
    assert not any("commission_service" in p for p in visible_files())
    checks += 1
    print("calibration: %d cells (6 graded, %d development), each camera off its nominal mounting; located through the "
          "nominal poses a point on the fixture is off by %.0f to %.0f mm (overhead) and a point 0.35 m in front of the "
          "wrist camera by %.1f to %.1f mm; no offset value in the %d visible files" % (
              len(cells), len(dev), 1e3 * min(errs_o), 1e3 * max(errs_o), 1e3 * min(errs_w), 1e3 * max(errs_w), len(visible_files())))
    return checks


def main():
    checks = noise_checks() + leak_checks()
    print("ok: %d checks" % checks)


if __name__ == "__main__":
    main()
