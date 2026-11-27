"""document_figures.py: the measured figures that the agent-facing documents quote.

    python document_figures.py [N]        (N units, default 200)

Maintainers' tooling, run from a checkout; nothing runs it during grading. It draws N fresh units
with seeds of its own (SEEDS0 below; never the lab's or the grader's units.json) through the
cell's own code in the folder above this one (calib_cell.py, cell_camera.py) and prints every
figure the documents quote that is measured from units or computed from the model rather than
copied from a constant of cell_data.py. Each figure is tied to the sentence it backs:

  F1  instruction.md, "Facts", "Before calibration", and cell.md, end of section 3: "With the
      drawing's values, the largest tool-tip error on a unit's acceptance conditions is between
      11 and 22 mm for nine units in ten, and the largest camera-test error between 19 and 34 mm"
      (the 5th and 95th percentiles over units of the per-unit largest error, rounded).
  F2  cell.md, section 2 ("the stylus centre of probe_a is then at (0.5926, 0, 0.6174) m") and
      camera.md, "Mounting" ("the camera centre is at (0.4583, 0, 0.6669) m and the optical axis
      points along (-0.7071, 0, -0.7071)"): the drawing at the home pose (computed, not drawn).
  F3  camera.md, "Sensor", "Noise": "0.3 to 0.5 where the image is black, about 0.85 at 20 grey
      levels above black, about 1.7 at 100 and about 2.6 at 240" (rms of the 8-bit output).
  F4  camera.md, "Sensor": "Fewer than 15 hot (brighter) or dead (black) pixels".
  F5  camera.md, "Sensor": "Black level: between 3 and 10 grey levels".
  F6  camera.md, "Sensor": "at the corners between about half and three quarters of the
      brightness at the centre".
  F7  camera.md, "Lens and geometry", and instruction.md, "The camera is uncalibrated": "at the
      image corners it displaces an image point by at most about 4 px".
  F8  camera.md, "Lens and geometry", "Focus": "a surface at the focus distance is imaged with a
      blur of under a pixel; at 0.25 m and at 0.55 m the blur is roughly 1 to 3 px; the room's
      walls are blurred by 2 to 6 px" (walls: 2.5 to 4 m along the optical axis).
  F9  camera.md, "Focus": "A faint veiling glare (a few percent of the light)".
  F10 camera.md, "Sensor", exposure: "It aims at a mean of about a quarter to a third of full
      scale above black, less when that would bring more than the brightest few thousandths of
      the image near saturation ... The brightest highlights can saturate (255)": rendered images
      at the home pose and at three acceptance poses per unit (probe_a mounted).
  F11 the gauge plate: the region it may occupy (cell_data.plate_envelope, from the documented ranges); in
      images from 0.40 m, 60 degrees down, its top face against the table around it and its bores' insides
      (median grey levels above black, per unit), for the documents' description of how it looks.
  F12 cell.md, section 11, the end: "for the same joint targets the pin of a unit at its warm start stands
      typically ... from where it stands on the cold unit, more than the clearance on most units": a cold program
      planned on the true unit (solution/calibrate/plate_job.py's planner with the true kinematics and the true
      heating and cooling, a study only), run cold and from the unit's warm start temperatures with the same
      repeatability draws; at the cold run's deepest arrival in each bore, the distance between the two apexes
      across the bore's axis (5 % to 95 % over every bore of every unit; and per unit the largest, against the
      radial clearance).
  F13 cell.md, section 12, "What it does to the tip": "On the drawing's geometry this moves the stylus centre by up
      to a few millimetres (most for joint 1 at full reach, about ... mm), and the pin of process_c at the plate's
      bores typically by ...": (a) on the drawing, probe_a mounted, at the acceptance conditions' joint positions of
      the units drawn, the stylus centre's displacement when a serviced joint's angle changes by its documented
      largest (0.25 degrees plus 0.4 times BACKLASH_MAX[j]): the largest per joint; (b) a cold program planned on the
      true unit as measured (as F12), run on the unit after its service (calib_cell.serviced) with the same
      repeatability draws: at the deepest arrival in each bore the distance between the two apexes across the bore's
      axis (5 % to 95 % over every bore of every unit; per unit the largest, against the radial clearance).
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))          # tests/: the cell's own code

import numpy as np  # noqa: E402

import arm_model as am  # noqa: E402
import calib_cell as cc  # noqa: E402
import cell_camera as ccam  # noqa: E402

SEEDS0 = 4_200_000_001                               # this script's own seeds: SEEDS0, SEEDS0 + 1, ...


def seeds(n):
    return range(SEEDS0, SEEDS0 + n)


def spread(values, digits=2):
    v = np.asarray(values, float)
    return "5% {:.{d}f}  median {:.{d}f}  95% {:.{d}f}  (min {:.{d}f}, max {:.{d}f})".format(
        *np.percentile(v, [5, 50, 95]), v.min(), v.max(), d=digits)


def f1_drawing_errors(n):
    rows = []
    for seed in seeds(n):
        tip, cam = cc.acceptance_errors(am.nominal(), cc.draw_unit(seed))
        rows.append([np.percentile(tip, 95), tip.max(), np.percentile(cam, 95), cam.max()])
    rows = np.array(rows) * 1e3
    print("F1  the drawing's values on a unit's acceptance conditions (mm, over {} units):".format(n))
    for k, name in enumerate(("tool tip, 95th percentile", "tool tip, largest", "camera test, 95th percentile",
                              "camera test, largest")):
        print("    {:30s} {}".format(name, spread(rows[:, k], 1)))


def f2_home():
    p, q = am.nominal(), cc.Q_HOME[None]
    s, temp, tool = np.ones((1, 6)), np.full((1, 6), am.T0), cc.TOOLS["probe_a"]
    tip = am.tcp(p, q, s, temp, tool)[0]
    R, o = am.camera_pose(p, q, s, temp, tool)
    print("F2  drawing, home pose, probe_a: stylus centre {}  camera centre {}  optical axis {}".format(
        np.round(tip, 4).tolist(), np.round(o[0], 4).tolist(), np.round(R[0][:, 2], 4).tolist()))


def f3_noise(cams):
    """Flat fields through the camera's sensor with the exposure set to give the wanted level."""
    out = {}
    original = ccam.exposure
    try:
        for level in (0.0, 20.0, 100.0, 240.0):
            rms = []
            for k, cam in enumerate(cams[:5]):
                ccam.exposure = lambda E, cam, level=level: level * ccam.FULL_WELL / 255.0
                img = ccam.sensor(np.ones((ccam.IMAGE_H, ccam.IMAGE_W)), cam, np.random.default_rng(k))
                bad = np.zeros(img.size, bool)
                bad[cam["sensor"]["hot"]] = True
                bad[cam["sensor"]["dead"]] = True
                rms.append(img.reshape(-1)[~bad].astype(float).std())
            out[level] = (min(rms), max(rms))
    finally:
        ccam.exposure = original
    print("F3  sensor noise, grey levels rms (min..max over units): " + ", ".join(
        "{:.0f} above black: {:.2f}..{:.2f}".format(lv, *r) for lv, r in out.items()))


def f4_to_f9(cams):
    hot_dead = [len(c["sensor"]["hot"]) + len(c["sensor"]["dead"]) for c in cams]
    print("F4  hot plus dead pixels per unit: {} to {}".format(min(hot_dead), max(hot_dead)))
    black = [c["sensor"]["black"] for c in cams]
    print("F5  black level (grey levels): {:.2f} to {:.2f}".format(min(black), max(black)))
    corner, disp = [], []
    for c in cams:
        op = c["optics"]
        grid = ccam.Grid(op)
        v = ccam.vignetting(c, grid)
        corners = [(0, 0), (0, -1), (-1, 0), (-1, -1)]
        corner += [v[i, j] for i, j in corners]
        d = []
        for i, j in corners:
            u_c, v_c = ccam.project(op, np.array([grid.x[i, j], grid.y[i, j], 1.0]))
            ideal_u, ideal_v = op["cx"] + op["f"] * grid.x[i, j], op["cy"] + op["f"] * grid.y[i, j]
            d.append(np.hypot(u_c - ideal_u, v_c - ideal_v))
        disp.append(max(d))
    print("F6  vignetting, brightness at the corners relative to the principal point: {:.2f} to {:.2f}".format(
        min(corner), max(corner)))
    print("F7  radial distortion, largest displacement at the four corners (px): {:.3f} to {:.3f}".format(
        min(disp), max(disp)))
    blur = {}
    for name, z in (("at focus", None), ("0.25 m", 0.25), ("0.55 m", 0.55), ("walls 2.5 m", 2.5), ("walls 4 m", 4.0)):
        vals = [float(ccam.defocus_sigma(c["optics"], c["optics"]["focus"] if z is None else z)) for c in cams]
        blur[name] = (min(vals), max(vals))
    print("F8  blur (rms spread, px): " + ", ".join("{} {:.2f}..{:.2f}".format(k, *v) for k, v in blur.items()))
    halo = [c["optics"]["halo"] for c in cams]
    print("F9  veiling glare, share of the light: {:.3f} to {:.3f}".format(min(halo), max(halo)))


def f10_images(n_units):
    means, high, sat = [], [], []
    for seed in seeds(n_units):
        unit = cc.draw_unit(seed)
        q, s, temp, _ = cc.acceptance_conditions(seed)
        cell = cc.Cell(unit, session=1)
        poses = [(cc.Q_HOME, np.ones(6), np.full(6, 20.0))] + [(q[i], s[i], temp[i]) for i in range(3)]
        for qi, si, ti in poses:
            cell.q, cell.s, cell.temp, cell.images = np.array(qi), np.array(si, float), np.array(ti), 0
            img = cell.image()["image"].astype(float)
            black = unit["camera"]["sensor"]["black"]
            means.append((img.mean() - black) / 255.0)
            high.append((img >= 235).mean())
            sat.append((img >= 255).mean())
    print("F10 rendered images ({} units x 4 poses): mean above black / full scale {}".format(n_units, spread(means)))
    print("    fraction of pixels >= 235: {}".format(spread(high, 4)))
    print("    fraction of pixels at 255: {}".format(spread(sat, 4)))


def f11_plate(n_units):
    import cell_camera as ccam
    radius, top = cc.plate_envelope()
    print("F11 plate envelope: radius {:.3f} m about PLATE_XY_NOMINAL, up to {:.3f} m above the table".format(radius, top))
    a, b, T = cc.PLATE_SIZE[0] / 2, cc.PLATE_SIZE[1] / 2, cc.PLATE_SIZE[2]
    corners = []
    for seed in range(SEEDS0, SEEDS0 + 20000):
        pl = cc.draw_plate(seed)
        corners += [(pl["O"] + pl["R"] @ np.array([x, y, -T]))[2] for x in (-a, a) for y in (-b, b)]
    print("    bottom face's corners above the table over 20000 plates: {:.1f} to {:.1f} mm".format(
        min(corners) * 1e3, max(corners) * 1e3))
    ratio_top, ratio_bore = [], []
    for seed in seeds(n_units):
        unit = cc.draw_unit(seed)
        cam, pl = unit["camera"], unit["plate"]
        grid = ccam.Grid(cam["optics"])
        el, az = 60 * cc.D, np.arctan2(-pl["O"][1], -pl["O"][0])
        w = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        oc = pl["O"] + 0.40 * w
        v = -w
        x = np.cross(v, [0.0, 0.0, 1.0])
        x /= np.linalg.norm(x)
        Rc = np.stack([x, np.cross(v, x), v], axis=1)
        img = ccam.render(cam, grid, Rc, oc, unit["centres"], cc._rng("figures", seed), plate=pl).astype(float)
        dx, dy, dz = ccam._rays(grid.x, grid.y, Rc)
        t, face, _, _, _ = ccam.plate_hits(oc, dx, dy, dz, pl)
        black = cam["sensor"]["black"]
        core = ndimage_erode(face == 1)
        table = ndimage_erode(face == 0)
        bores = ndimage_erode((face == 6) | (face == 7))
        g_top, g_tab, g_bore = (np.median(img[m]) - black for m in (core, table, bores))
        ratio_top.append(g_top / max(g_tab, 1.0))
        ratio_bore.append(g_bore / max(g_top, 1.0))
    print("    top face / table around it (grey above black, per unit): {}".format(spread(ratio_top)))
    print("    bores' inside / top face: {}".format(spread(ratio_bore, 3)))


def f12_warm_displacement(n_units):
    sys.path.insert(0, os.path.join(_HERE, "..", "..", "solution", "calibrate"))
    import plate_job as pj
    tool = cc.TOOLS[cc.JOB_TOOL]
    every, largest = [], []
    for seed in seeds(n_units):
        unit = cc.draw_unit(seed)

        def fn(q, s, temp, unit=unit):
            R, o = cc.true_frames(unit, q, s, temp, tool)
            return o + np.einsum("nab,b->na", R, pj.TIP_C), R
        rows = pj.Planner(pj.Kin(fn), np.array(unit["plate"]["pose"]), heat=unit["cell"]["heat"],
                          tau=unit["cell"]["tau_cool"]).plan()
        if rows is None:
            continue
        rc = cc.run_job(unit, rows, radius=1.5e-3, trace=True)
        rw = cc.run_job(unit, rows, radius=1.5e-3, trace=True, temp0=cc.warm_start(seed))
        pl = unit["plate"]
        a = pl["R"][:, 2]
        best = {}
        for k, ar in enumerate(rc["arrivals"]):
            p = (cc.pin_axis(unit, ar["q"][None], ar["s"], ar["temp"], ar["offset"])[2][0] - pl["O"]) @ pl["R"]
            r = np.hypot(p[0] - cc.BORES[:, 0], p[1] - cc.BORES[:, 1])
            b = int(np.argmin(r))
            if r[b] < cc.BORE_RADIUS and p[2] < 0 and (b not in best or p[2] < best[b][0]):
                best[b] = (p[2], k)
        d = []
        for _, k in best.values():
            if k >= len(rw["arrivals"]):
                continue
            ac, aw = rc["arrivals"][k], rw["arrivals"][k]
            e = (cc.pin_axis(unit, aw["q"][None], aw["s"], aw["temp"], ac["offset"])[2][0]
                 - cc.pin_axis(unit, ac["q"][None], ac["s"], ac["temp"], ac["offset"])[2][0])
            d.append(np.linalg.norm(e - (e @ a) * a))
        if d:
            every += d
            largest.append(max(d))
    c = cc.BORE_RADIUS - cc.PIN_RADIUS
    print("F12 cold program run warm, the same joint targets ({} units): apex displacement across the bore axis (mm)".format(
        len(largest)))
    print("    every bore: {}".format(spread(np.array(every) * 1e3)))
    print("    per unit, the largest: {}; above the clearance {:.2f} mm on {} of {} units".format(
        spread(np.array(largest) * 1e3), c * 1e3, int(np.sum(np.array(largest) > c)), len(largest)))


def f13_service(n_units):
    sys.path.insert(0, os.path.join(_HERE, "..", "..", "solution", "calibrate"))
    import plate_job as pj
    nom = am.nominal()
    tool_a = cc.TOOLS["probe_a"]
    worst = {}
    for seed in seeds(n_units):
        q, _, _, _ = cc.acceptance_conditions(seed)
        s0, t0 = np.zeros_like(q), np.full_like(q, am.T0)
        base = am.tcp(nom, q, s0, t0, tool_a)
        for j in cc.SERVICE_JOINTS:
            a = cc.SERVICE_ZERO[1] + 0.4 * cc.BACKLASH_MAX[j]
            p = nom.copy()
            p[am.SLICES["rpy"].start + 3 * j + 2] += a
            d = np.linalg.norm(am.tcp(p, q, s0, t0, tool_a) - base, axis=1).max()
            worst[j] = max(worst.get(j, 0.0), float(d))
    print("F13 (a) the stylus centre's largest displacement on the drawing at acceptance poses, the serviced joint's "
          "largest change (mm): " + ", ".join("joint {} {:.2f}".format(j, worst[j] * 1e3) for j in sorted(worst)))
    tool = cc.TOOLS[cc.JOB_TOOL]
    every, largest = [], []
    for seed in seeds(min(n_units, 40)):
        unit = cc.draw_unit(seed)
        after = cc.serviced(unit)

        def fn(q, s, temp, unit=unit):
            R, o = cc.true_frames(unit, q, s, temp, tool)
            return o + np.einsum("nab,b->na", R, pj.TIP_C), R
        rows = pj.Planner(pj.Kin(fn), np.array(unit["plate"]["pose"]), heat=unit["cell"]["heat"],
                          tau=unit["cell"]["tau_cool"]).plan()
        if rows is None:
            continue
        rc = cc.run_job(unit, rows, radius=1.5e-3, trace=True)
        pl = unit["plate"]
        a = pl["R"][:, 2]
        best = {}
        for k, ar in enumerate(rc["arrivals"]):
            pp = (cc.pin_axis(unit, ar["q"][None], ar["s"], ar["temp"], ar["offset"])[2][0] - pl["O"]) @ pl["R"]
            r = np.hypot(pp[0] - cc.BORES[:, 0], pp[1] - cc.BORES[:, 1])
            b = int(np.argmin(r))
            if r[b] < cc.BORE_RADIUS and pp[2] < 0 and (b not in best or pp[2] < best[b][0]):
                best[b] = (pp[2], k)
        d = []
        for _, k in best.values():
            ar = rc["arrivals"][k]
            e = (cc.pin_axis(after, ar["q"][None], ar["s"], ar["temp"], ar["offset"])[2][0]
                 - cc.pin_axis(unit, ar["q"][None], ar["s"], ar["temp"], ar["offset"])[2][0])
            d.append(np.linalg.norm(e - (e @ a) * a))
        if d:
            every += d
            largest.append(max(d))
    c = cc.BORE_RADIUS - cc.PIN_RADIUS
    print("    (b) a cold program run on the serviced unit, the same joint targets ({} units): apex displacement across "
          "the bore axis (mm)".format(len(largest)))
    print("    every bore: {}".format(spread(np.array(every) * 1e3)))
    print("    per unit, the largest: {}; above the clearance {:.2f} mm on {} of {} units".format(
        spread(np.array(largest) * 1e3), c * 1e3, int(np.sum(np.array(largest) > c)), len(largest)))


def ndimage_erode(mask):
    from scipy import ndimage
    return ndimage.binary_erosion(mask, iterations=3)


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 200
    f11_plate(min(n, 12))
    f12_warm_displacement(min(n, 40))
    f13_service(min(n, 200))
    f1_drawing_errors(n)
    f2_home()
    cams = [cc.draw_unit(seed)["camera"] for seed in seeds(n)]
    f3_noise(cams)
    f4_to_f9(cams)
    f10_images(min(n, 12))


if __name__ == "__main__":
    main()
