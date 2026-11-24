"""baseline_rerender.py: the version-1 vision, kept as a baseline (reference_check.py only; not part of
the reference, not public).

What the LLM agent did on version 1 of the task: measure each datum sphere by fitting the image
model that version 1 published (cell_data.py of version 1: a matte white sphere lit by one lamp
at the camera, radiance ALBEDO * (AMBIENT + (1 - AMBIENT) cos(i) (LIGHT_R0 / r)^2), exact
pinhole intrinsics F_PX, CX, CY without distortion, vignetting cos^4, a 7-tap Gaussian blur),
re-rendering the sphere and adjusting its centre (and linearly gain, offset and a background
plane) until the rendered and the taken images agree. Here applied to the new images with the
version-1 constants (which no longer describe the camera).
"""
import numpy as np
from scipy import ndimage

import reference as ref
from reference import spheres_in_camera, R_SPHERE, R_POST

RMS_MAX = 10.0     # version 1 kept fits with rms <= 3; relaxed here, the new images never fit that well

# ---- the version-1 published image model (cell_data.py of version 1) ----------------------------
F_PX = 1400.0
IMAGE_W, IMAGE_H = 1280, 960
CX, CY = (IMAGE_W - 1) / 2.0, (IMAGE_H - 1) / 2.0
ALBEDO = {"sphere": 0.85, "post": 0.12, "table": 0.06}
AMBIENT = 0.05
LIGHT_R0 = 0.22
BLUR_TAPS = np.array([4.4074336693235636e-04, 2.1910314171364801e-02, 2.2831071645846546e-01,
                      4.9867645200647487e-01, 2.2831071645846546e-01, 2.1910314171364801e-02,
                      4.4074336693235636e-04])


def to_pixels(P):
    return CX + F_PX * P[..., 0] / P[..., 2], CY + F_PX * P[..., 1] / P[..., 2]


def radius_px(P):
    return F_PX * R_SPHERE / np.sqrt(np.maximum((P * P).sum(-1) - R_SPHERE ** 2, 1e-12))


def outline_box(P):
    d2 = float(P @ P)
    if P[2] <= 0 or d2 <= R_SPHERE ** 2:
        return None
    sin_b = R_SPHERE / np.sqrt(d2)
    cos_a = P[2] / np.sqrt(d2)
    if np.arccos(min(cos_a, 1.0)) + np.arcsin(sin_b) > 80 * np.pi / 180:
        return None
    Cs = np.linalg.inv(np.outer(P, P) - (d2 - R_SPHERE ** 2) * np.eye(3))
    out = []
    for a, cen in ((0, CX), (1, CY)):
        r = np.sqrt(max(Cs[a, 2] ** 2 - Cs[2, 2] * Cs[a, a], 0.0))
        lo, hi = sorted([(Cs[a, 2] - r) / Cs[2, 2], (Cs[a, 2] + r) / Cs[2, 2]])
        out += [cen + F_PX * lo, cen + F_PX * hi]
    return np.array(out)


# ---- the image of a sphere: the documented image formation, for one sphere -------------------
_SUB = (np.arange(9) + 0.5) / 9 - 0.5                 # 9 x 9 sub-pixel samples near the outline
_SUBX, _SUBY = [a.ravel() for a in np.meshgrid(_SUB, _SUB)]


def _blur_roi(E):
    H, W = E.shape
    P = np.pad(E, ((0, 0), (3, 3)), mode="edge")
    E1 = sum(BLUR_TAPS[k] * P[:, k:k + W] for k in range(7))
    P = np.pad(E1, ((3, 3), (0, 0)), mode="edge")
    return sum(BLUR_TAPS[k] * P[k:k + H] for k in range(7))


def _sphere_radiance(P, x, y):
    """Radiance of the sphere centred at P (camera frame) along the rays through normalised image
    points x, y, and whether they meet it. Light at the camera: incidence = viewing angle."""
    n = 1.0 / np.sqrt(x * x + y * y + 1.0)
    b = (P[0] * x + P[1] * y + P[2]) * n
    disc = b * b - (P @ P - R_SPHERE ** 2)
    hit = (disc > 0) & (b > 0)
    sq = np.sqrt(np.where(hit, disc, 0.0))
    t = np.where(hit, b - sq, 1.0)
    L = ALBEDO["sphere"] * (AMBIENT + (1 - AMBIENT) * (sq / R_SPHERE) * (LIGHT_R0 / t) ** 2)
    return np.where(hit, L, 0.0), hit, b


def sphere_roi_model(P, i0, i1, j0, j1):
    """For the pixel block [i0:i1, j0:j1]: S = blurred (vignetted radiance of the sphere where the
    pixel sees it, averaged over the pixel), C = blurred coverage of the pixel by the sphere."""
    jj, ii = np.meshgrid(np.arange(j0, j1, dtype=float), np.arange(i0, i1, dtype=float))
    x, y = (jj - CX) / F_PX, (ii - CY) / F_PX
    L, hit, b = _sphere_radiance(P, x, y)
    cov = hit.astype(float)
    h = np.sqrt(np.maximum(P @ P - b * b, 0.0))
    band = np.abs(h - R_SPHERE) * F_PX < 2.5 * np.maximum(b, 1e-9)
    if np.any(band):
        bi, bj = ii[band], jj[band]
        Ls, hs, _ = _sphere_radiance(P, ((bj[:, None] + _SUBX) - CX) / F_PX, ((bi[:, None] + _SUBY) - CY) / F_PX)
        L[band] = Ls.mean(axis=1)
        cov[band] = hs.mean(axis=1)
    V = 1.0 / (1.0 + x * x + y * y) ** 2
    return _blur_roi(V * L), _blur_roi(cov)


def _theta_to_P(th):
    m = np.array([th[0], th[1], 1.0])
    return th[2] * m / np.linalg.norm(m)


def _P_to_theta(P):
    return np.array([P[0] / P[2], P[1] / P[2], np.linalg.norm(P)])


def fit_sphere(img, P0, use, roi):
    """Least-squares fit of one sphere's image: its centre P (camera frame) and, linearly, the
    sensor offset, the gain and a background plane. use: bool mask over the block roi of the
    pixels that count. Returns P, covariance of P, rms of the normalised residuals."""
    i0, i1, j0, j1 = roi
    I = img[i0:i1, j0:j1].astype(float)[use]
    var = 0.34 + 0.064 * np.maximum(I - 8.0, 0.0)          # photon + read + quantisation noise (DN^2)
    sw = 1.0 / np.sqrt(var)
    ii, jj = np.nonzero(use)
    du, dv = (jj - jj.mean()) / 100.0, (ii - ii.mean()) / 100.0

    def residual(th):
        S, C = sphere_roi_model(_theta_to_P(th), i0, i1, j0, j1)
        S, B = S[use], 1.0 - C[use]
        A = np.stack([np.ones_like(S), S, B, B * du, B * dv], axis=1) * sw[:, None]
        beta = np.linalg.lstsq(A, I * sw, rcond=None)[0]
        return I * sw - A @ beta, beta

    th = _P_to_theta(P0)
    r, beta = residual(th)
    for it in range(8):
        rpx = radius_px(_theta_to_P(th))
        h = np.array([0.1 / F_PX, 0.1 / F_PX, th[2] * 0.1 / rpx])
        J = np.empty((len(r), 3))
        for k in range(3):
            e = np.zeros(3)
            e[k] = h[k]
            J[:, k] = (residual(th + e)[0] - residual(th - e)[0]) / (2 * h[k])
        step = -np.linalg.lstsq(J, r, rcond=None)[0]
        th_new = th + step
        r_new, beta_new = residual(th_new)
        if r_new @ r_new > r @ r:          # no progress: half step once, then stop
            th_new = th + 0.5 * step
            r_new, beta_new = residual(th_new)
            if r_new @ r_new > r @ r:
                break
        th, r, beta = th_new, r_new, beta_new
        if np.all(np.abs(step) / h < 0.02):
            break
    chi = float(r @ r / max(len(r) - 8, 1))
    Cth = np.linalg.inv(J.T @ J) * max(chi, 1.0)
    # covariance of P from that of theta
    Jp = np.empty((3, 3))
    for k in range(3):
        e = np.zeros(3)
        e[k] = 1e-7 if k < 2 else 1e-6
        Jp[:, k] = (_theta_to_P(th + e) - _theta_to_P(th - e)) / (2 * e[k])
    return _theta_to_P(th), Jp @ Cth @ Jp.T, np.sqrt(chi), beta


def _ray_grid(i0, i1, j0, j1):
    jj, ii = np.meshgrid(np.arange(j0, j1, dtype=float), np.arange(i0, i1, dtype=float))
    x, y = (jj - CX) / F_PX, (ii - CY) / F_PX
    n = 1.0 / np.sqrt(x * x + y * y + 1.0)
    return np.stack([x * n, y * n, n], axis=-1)        # unit rays, camera frame


def _near_sphere(rays, P, margin_px):
    """Rays that pass within margin_px (angular, in px) of the outline of the sphere P, or inside."""
    b = rays @ P
    h = np.sqrt(np.maximum(P @ P - b * b, 0.0))
    return (b > 0) & (h < R_SPHERE + margin_px * b / F_PX)


def _near_post(rays, Rc, oc, c, margin_px, P=None):
    """Rays that pass within margin_px of the post under the sphere centred at c (datum frame);
    with P (a sphere's centre in the camera frame), only where the post is not hidden behind
    that sphere (a camera looking up sees the post in front of the sphere's lower part)."""
    d = rays @ Rc.T                                    # datum-frame directions
    wx, wy = oc[0] - c[0], oc[1] - c[1]
    a2 = np.maximum(d[..., 0] ** 2 + d[..., 1] ** 2, 1e-300)
    h = np.abs(wx * d[..., 1] - wy * d[..., 0]) / np.sqrt(a2)
    ts = -(wx * d[..., 0] + wy * d[..., 1]) / a2
    z = oc[2] + ts * d[..., 2]
    near = (ts > 0) & (h < R_POST + margin_px * ts / F_PX) & (z > -0.01) & (z < c[2] + 0.01)
    if P is None:
        return near
    # where the inflated post is entered, against where the sphere is met
    rad = R_POST + margin_px * np.maximum(ts, 0.0) / F_PX
    b2 = wx * d[..., 0] + wy * d[..., 1]
    disc = b2 * b2 - a2 * (wx * wx + wy * wy - rad * rad)
    t_post = (-b2 - np.sqrt(np.maximum(disc, 0.0))) / a2
    b = rays @ P
    disc_s = b * b - (P @ P - R_SPHERE ** 2)
    t_sph = np.where(disc_s > 0, b - np.sqrt(np.maximum(disc_s, 0.0)), np.inf)
    return near & (t_post < t_sph)


def measure_image(rec, p, centres, margin_px, detect=True):
    """Centres of the datum spheres in one image, in the camera frame: list of dicts (k, P, cov,
    rms). Pose predicted with parameters p (84); objects other than the sphere being fitted are
    masked with margin_px around their predicted outlines."""
    img = rec["image"]
    Pall, Rc, oc = spheres_in_camera(p, rec["q"], rec["s"], rec["temp"], rec["tool"], centres)
    Pall, Rc, oc = Pall[0], Rc[0], oc[0]
    out = []
    H, W = img.shape
    for k in range(len(centres)):
        box = outline_box(Pall[k])
        if box is None or box[0] < 8 or box[1] > W - 9 or box[2] < 8 or box[3] > H - 9:
            continue
        if not 0.2 < np.linalg.norm(Pall[k]) < 0.7:
            continue
        m = margin_px + 6
        i0, i1 = int(max(0, box[2] - m)), int(min(H, box[3] + m + 1))
        j0, j1 = int(max(0, box[0] - m)), int(min(W, box[1] + m + 1))
        P0 = Pall[k]
        if detect:                                          # find the blob near the prediction
            win = img[i0:i1, j0:j1].astype(float)
            lo, hi = np.percentile(win, 10), np.percentile(win, 99.5)
            lab, n = ndimage.label(win > lo + 0.3 * (hi - lo))
            if n == 0:
                continue
            uc0, vc0 = to_pixels(Pall[k])
            sizes = ndimage.sum(np.ones_like(win), lab, index=np.arange(1, n + 1))
            cents = np.array(ndimage.center_of_mass(np.ones_like(win), lab, index=np.arange(1, n + 1)))
            dist = np.hypot(cents[:, 1] + j0 - uc0, cents[:, 0] + i0 - vc0)
            rp = radius_px(Pall[k])
            cand = np.nonzero((sizes > 0.3 * np.pi * rp ** 2) & (dist < rp + margin_px))[0]
            if len(cand) == 0:
                continue
            best = cand[np.argmin(dist[cand])]
            vc, uc = cents[best][0] + i0, cents[best][1] + j0
            r_eq = np.sqrt(sizes[best] / np.pi) / 0.97
            mm = np.array([(uc - CX) / F_PX, (vc - CY) / F_PX, 1.0])
            P0 = R_SPHERE * np.sqrt(1 + (F_PX / r_eq) ** 2) * mm / np.linalg.norm(mm)
            shift = P0 - Pall[k]
        # the pixels that count: the sphere and a ring around it, other objects masked
        rays = _ray_grid(i0, i1, j0, j1)
        use = _near_sphere(rays, P0, 7.0)
        for j in range(len(centres)):
            if j != k and Pall[j][2] > 0:
                use &= ~_near_sphere(rays, Pall[j], margin_px)
            cj = centres[j]
            if j == k and detect:
                # own post: anchored at the detected sphere (the pose may still be off)
                oc_k = oc - Rc @ shift
                use &= ~(_near_post(rays, Rc, oc_k, cj, max(6.0, 0.25 * margin_px), P0))
            else:
                use &= ~(_near_post(rays, Rc, oc, cj, margin_px, P0))
        jj, ii = np.meshgrid(np.arange(j0, j1), np.arange(i0, i1))
        use &= (ii >= 4) & (ii < H - 4) & (jj >= 4) & (jj < W - 4) & (img[i0:i1, j0:j1] < 254)
        # keep a 4 px frame for the blur
        frame = np.zeros_like(use)
        frame[4:-4, 4:-4] = True
        use &= frame
        inside = _near_sphere(rays, P0, -3.0)
        if use.sum() < 200 or (use & inside).sum() < 0.6 * inside.sum():
            continue
        try:
            P, cov, rms, beta = fit_sphere(img, P0, use, (i0, i1, j0, j1))
        except np.linalg.LinAlgError:
            continue
        du, dv = np.array(to_pixels(P)) - np.array(to_pixels(P0))
        if not np.isfinite(rms) or rms > RMS_MAX or np.hypot(du, dv) > 15:
            continue
        out.append(dict(k=k, P=P, cov=cov, rms=rms, n_px=int(use.sum()), gain=beta[1] / 255.0, black=beta[0]))
    return out


def measure(records, p, centres):
    """The version-1 two passes over a session's images (wide masks, the camera fitted, then
    tight masks), with the camera's 6 parameters refitted between them (arm held)."""
    def one_pass(p, margin, detect):
        out = []
        for rec in records:
            r = dict(rec)
            r["spheres"] = measure_image(rec, p, centres, margin, detect)
            out.append(r)
        return out
    p = np.array(p, float)
    p[ref.CAM] = ref.camera_prior()[0]
    obs = one_pass(p, 30.0, True)
    p, _, _ = ref.fit_camera(p, obs, centres)
    obs = one_pass(p, 6.0, False)
    p, _, _ = ref.fit_camera(p, obs, centres)
    return p, obs
