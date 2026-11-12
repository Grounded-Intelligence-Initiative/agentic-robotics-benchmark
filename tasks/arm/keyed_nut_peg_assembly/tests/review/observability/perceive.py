"""Camera-only perception for keyed_nut_peg_assembly (a maintainer's evidence that the observations suffice; the
grader never runs it). Everything here uses what a controller receives: the observation's RGB images, its
T_world_wrist and the public camera matrices (keyed_env.py's CAMERAS, WRIST, fixed_camera_matrices); nothing else.

  * coarse_nut: each fixed camera's largest blob of the nut's colour (kit-aware hue segmentation that copes with the
    lights washing the colours out), its centroid back-projected onto the ring's mid-height plane;
  * find_nuts (refine=True): a close-up from the wrist camera right above the nut: the blob's principal axis, the handle
    end told by its narrow spread, then a circle fitted to the ring's outer edge away from the handle (radius 28 mm);
  * find_pegs: the shaft tops (bright, unsaturated discs) in the wrist camera's view from the start pose, back-projected
    onto the shaft-top plane and grouped;
  * count_bands: the intensity on the camera-facing side of each shaft at the three band heights against the gaps
    between them, in the fixed left and right cameras.
"""
import colorsys
import math

import numpy as np

TABLE_Z = 0.80
NUT_TOP_Z = TABLE_Z + 0.045
SHAFT_TOP_Z = TABLE_Z + 0.010 + 0.065
NUT_RGB = {"blue": [0.16, 0.36, 0.78], "green": [0.20, 0.62, 0.30], "red": [0.78, 0.18, 0.16],
           "yellow": [0.86, 0.74, 0.16], "orange": [0.85, 0.45, 0.12], "purple": [0.55, 0.25, 0.70],
           "teal": [0.15, 0.65, 0.68], "pink": [0.80, 0.35, 0.55]}
NUT_HUE = {k: colorsys.rgb_to_hsv(*v)[0] * 360.0 for k, v in NUT_RGB.items()}
CAMERAS = {"left": ((-0.30, 0.55, 1.45), (0.05, 0.0, 0.80), 50.0, 320, 240),
           "right": ((-0.30, -0.55, 1.45), (0.05, 0.0, 0.80), 50.0, 320, 240)}
WRIST = (75.0, 256, 192)


def intrinsics(fovy, w, h):
    f = (h / 2.0) / math.tan(math.radians(fovy) / 2.0)
    return np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1.0]])


def look_at(pos, target, up=(0, 0, 1)):
    pos, target = np.asarray(pos, float), np.asarray(target, float)
    f = target - pos
    f = f / np.linalg.norm(f)
    z = -f
    x = np.cross(np.array(up, dtype=float), z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=1)


def fixed_camera(name):
    pos, tgt, fovy, w, h = CAMERAS[name]
    T = np.eye(4)
    T[:3, :3] = look_at(pos, tgt) @ np.diag([1.0, -1.0, -1.0])
    T[:3, 3] = pos
    return intrinsics(fovy, w, h), T


def hsv_image(img):
    x = img.astype(np.float32) / 255.0
    mx, mn = x.max(axis=2), x.min(axis=2)
    d = mx - mn + 1e-6
    r, g, b = x[..., 0], x[..., 1], x[..., 2]
    h = np.where(mx == r, ((g - b) / d) % 6.0, np.where(mx == g, (b - r) / d + 2.0, (r - g) / d + 4.0)) * 60.0
    s = np.where(mx > 1e-6, d / (mx + 1e-6), 0.0)
    return h, s, mx


def backproject(us, vs, K, T, z_plane):
    """Pixel centres (u + 0.5, v + 0.5) -> world points on the plane z = z_plane."""
    pix = np.stack([us + 0.5, vs + 0.5, np.ones_like(us, dtype=float)], axis=0)
    d = T[:3, :3] @ (np.linalg.inv(K) @ pix)
    c = T[:3, 3]
    t = (z_plane - c[2]) / d[2]
    return (c[:, None] + d * t).T


def project(points, K, T):
    Rt = T[:3, :3].T
    pc = (Rt @ (np.asarray(points, float).T - T[:3, 3][:, None]))
    uv = (K @ pc)[:2] / pc[2]
    return uv.T - 0.5


# hues of the nuts' top faces where the lights wash them out (a channel at 255): orange and yellow turn yellower, pink
# bluer; the others keep their hue
HUE_WASHED = dict(NUT_HUE, orange=50.0, yellow=60.0, pink=305.0)


def colour_mask(img, colour, kit_colours=None, hue_tol=30.0):
    """Pixels of `colour`: nearer in hue to it than to the kit's other colours (washed-out pixels compared with the
    washed hues), saturated enough, and not the table's wood (hue 18-46 deg at saturation 0.28-0.58)."""
    h, s, v = hsv_image(img)
    cols = list(kit_colours) if kit_colours else [colour]
    if colour not in cols:
        cols.append(colour)
    washed = v > 0.93

    def dist(c):
        ref = np.where(washed, HUE_WASHED[c], NUT_HUE[c])
        return np.abs(((h - ref) + 180.0) % 360.0 - 180.0)

    d = dist(colour)
    nearest = np.ones_like(d, dtype=bool)
    for c in cols:
        if c != colour:
            nearest &= d <= dist(c)
    wood = (h > 18.0) & (h < 46.0) & (s > 0.28) & (s < 0.58) & ~washed
    if colour in ("orange", "yellow"):
        sat_min = np.where(washed, 0.25, 0.60)
    else:
        sat_min = np.where(washed, 0.10, 0.10)
    return nearest & (d < hue_tol) & (s > sat_min) & (v > 0.15) & ~wood


def nut_from_points(pts):
    """(ring centre xy, handle yaw) from the back-projected points of one nut (ring + handle, one blob)."""
    xy = pts[:, :2]
    m = xy.mean(axis=0)
    c = np.cov((xy - m).T)
    w, V = np.linalg.eigh(c)
    u = V[:, 1]
    s = (xy - m) @ u
    n = np.array([-u[1], u[0]])
    # the ring end is the wide one: compare the lateral spread of the points near each end
    lat0 = (xy - m) @ n
    lo, hi = np.percentile(s, 2.0), np.percentile(s, 98.0)

    def spread(sel):
        return float(np.percentile(lat0[sel], 95) - np.percentile(lat0[sel], 5)) if sel.sum() > 3 else 0.0

    if spread(s > hi - 0.03) > spread(s < lo + 0.03):
        u, s = -u, -s
    n = np.array([-u[1], u[0]])
    # ring centre: 28 mm in from the ring side's extreme, laterally at the ring points' mean
    s_ring = np.percentile(s, 1.0) + 0.028
    ring = np.abs(s - s_ring) < 0.024
    lat = ((xy[ring] - m) @ n).mean() if ring.any() else 0.0
    centre = m + u * s_ring + n * lat
    # handle direction refined from the handle points' (beyond 45 mm) mean direction from the centre
    hp = xy[(xy - centre) @ u > 0.045]
    if len(hp) > 5:
        dvec = (hp - centre).mean(axis=0)
        u = dvec / np.linalg.norm(dvec)
    return centre, math.atan2(u[1], u[0])


def largest_blob(mask):
    import cv2
    n, lab, stats, _c = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    if n <= 1:
        return None
    j = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return lab == j


def fit_circle(xy):
    """Least-squares circle (Kasa): centre, radius."""
    A = np.column_stack([2 * xy[:, 0], 2 * xy[:, 1], np.ones(len(xy))])
    b = (xy ** 2).sum(axis=1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    c = sol[:2]
    return c, math.sqrt(max(0.0, sol[2] + c @ c))


def ring_circle(blob, K, T, centre, handle_yaw):
    """Refine the ring centre by a circle through the blob's outer boundary away from the handle (the top face's
    edge seen from above: radius 28 mm)."""
    import cv2
    cnts, _h = cv2.findContours(blob.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cnts:
        return centre
    c = max(cnts, key=len)[:, 0, :].astype(float)
    pts = backproject(c[:, 0], c[:, 1], K, T, NUT_TOP_Z)[:, :2]
    rel = pts - centre
    ang = np.arctan2(rel[:, 1], rel[:, 0]) - handle_yaw
    ang = (ang + math.pi) % (2 * math.pi) - math.pi
    r = np.linalg.norm(rel, axis=1)
    keep = (np.abs(ang) > math.radians(40.0)) & (r > 0.018) & (r < 0.036)
    if keep.sum() < 20:
        return centre
    cc, rr = fit_circle(pts[keep])
    if abs(rr - 0.028) > 0.005 or np.linalg.norm(cc - centre) > 0.006:
        return centre
    return cc


def find_nuts(img, K, T, colours, min_px=40, kit_colours=None, refine=False):
    out = {}
    for col in colours:
        blob = largest_blob(colour_mask(img, col, kit_colours))
        if blob is None or blob.sum() < min_px:
            out[col] = None
            continue
        vs, us = np.nonzero(blob)
        pts = backproject(us.astype(float), vs.astype(float), K, T, NUT_TOP_Z)
        centre, yaw = nut_from_points(pts)
        if refine:
            centre = ring_circle(blob, K, T, centre, yaw)
        out[col] = (centre, yaw)
    return out


def find_pegs(img, K, T, exclude_rows_from=None):
    """Shaft tops: bright, unsaturated pixels, back-projected onto the shaft-top plane and grouped (3 pegs)."""
    h, s, v = hsv_image(img)
    mask = (s < 0.10) & (v > 0.80)
    if exclude_rows_from is not None:
        mask[exclude_rows_from:, :] = False
    vs, us = np.nonzero(mask)
    if len(us) == 0:
        return []
    pts = backproject(us.astype(float), vs.astype(float), K, T, SHAFT_TOP_Z)[:, :2]
    inside = (np.abs(pts[:, 0]) < 0.40) & (np.abs(pts[:, 1]) < 0.40)
    pts = pts[inside]
    groups = []
    for p in pts:
        for g in groups:
            if np.linalg.norm(g["c"] - p) < 0.03:
                g["pts"].append(p)
                g["c"] = np.mean(g["pts"], axis=0)
                break
        else:
            groups.append({"c": p.copy(), "pts": [p]})
    groups = [g for g in groups if len(g["pts"]) >= 4]
    groups.sort(key=lambda g: -len(g["pts"]))
    return [g["c"] for g in groups[:3]]


def _bilinear(gray, u, v):
    H, W = gray.shape
    if not (0 <= u < W - 1 and 0 <= v < H - 1):
        return float("nan")
    u0, v0 = int(u), int(v)
    a, b = u - u0, v - v0
    return float((1 - a) * (1 - b) * gray[v0, u0] + a * (1 - b) * gray[v0, u0 + 1]
                 + (1 - a) * b * gray[v0 + 1, u0] + a * b * gray[v0 + 1, u0 + 1])


BAND_Z = (0.065, 0.052, 0.039)          # band centres above the table (top - 10 mm - 13 mm k)
GAP_Z = (0.0585, 0.0455, 0.0325)


def count_bands(img, K, T, peg_xy):
    """Bands on the shaft side facing the camera: the intensity at each band's height against the gaps between them.
    Returns (bands, contrast) or (None, 0) when the shaft is not seen clearly."""
    cam = T[:3, 3]
    d = np.array([cam[0] - peg_xy[0], cam[1] - peg_xy[1]])
    d = d / np.linalg.norm(d)
    gray = img.astype(np.float32).mean(axis=2)

    def sample(z):
        pts = [[peg_xy[0] + 0.0112 * d[0] + o * -d[1], peg_xy[1] + 0.0112 * d[1] + o * d[0], TABLE_Z + z]
               for o in (-0.003, 0.0, 0.003)]
        uv = project(pts, K, T)
        return float(np.median([_bilinear(gray, u, v) for (u, v) in uv]))

    band = [sample(z) for z in BAND_Z]
    gap = [sample(z) for z in GAP_Z]
    if any(np.isnan(band + gap)):
        return None, 0.0
    ref = float(np.median(gap))
    dark = [b < 0.55 * ref for b in band]
    contrast = ref - min(band)
    # bands are filled from the top: 1 = (65), 2 = (65, 52), 3 = all three
    n = sum(dark)
    if n == 0 or dark != [True] * n + [False] * (3 - n):
        return None, contrast
    return n, contrast


def coarse_nut(imgs, colour, kit_colours=None):
    """Rough nut position from the fixed cameras: each camera's largest blob of the colour, its pixel centroid
    back-projected onto the ring's mid-height plane; the mean of the cameras that see it (within a few cm)."""
    est = []
    for cam in ("left", "right"):
        if imgs.get(cam) is None:
            continue
        K, T = fixed_camera(cam)
        mask = colour_mask(imgs[cam], colour, kit_colours)
        vs, us = np.nonzero(mask)
        if len(us):
            pts = backproject(us.astype(float), vs.astype(float), K, T, TABLE_Z + 0.025)
            out = (pts[:, 0] < -0.30) | (pts[:, 0] > 0.0) | (np.abs(pts[:, 1]) > 0.30)
            mask[vs[out], us[out]] = False         # only the nuts' region of the table
        blob = largest_blob(mask)
        if blob is None or blob.sum() < 15:
            continue
        vs, us = np.nonzero(blob)
        p = backproject(np.array([us.mean()]), np.array([vs.mean()]), K, T, TABLE_Z + 0.025)[0]
        est.append(p[:2])
    return np.mean(est, axis=0) if est else None
