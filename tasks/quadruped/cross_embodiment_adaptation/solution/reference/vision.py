"""Image geometry and registration for the two body cameras (numpy / scipy only).

Frames: a body camera's image is a square, N pixels across; image right is the body's +x, image up its +y,
and the body's centre is the image centre. The map frame is the scout's body frame at its first observation
(origin at the dock point). Angles are counter-clockwise, in radians.
"""
import math

import numpy as np
from numpy.fft import irfft2, rfft2
from scipy import ndimage

N = 256
H = (N-1)/2
SCOUT_MPP = 2.8/N


def gray(image):
    return image.astype(np.float32).mean(axis=2)


def rot(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


def resample(img, A, t, s_in, origin_in, s_out, origin_out, shape_out, order=1, cval=0.):
    """out(q) = img(A q + t): q a point (x right, y up, metres) of the output frame, whose origin is at pixel
    `origin_out` (row, col) with `s_out` metres per pixel; likewise `origin_in`, `s_in` for the input."""
    k = s_out/s_in
    M = k*np.array([[A[1, 1], -A[1, 0]], [-A[0, 1], A[0, 0]]])
    offset = np.array([origin_in[0]-t[1]/s_in, origin_in[1]+t[0]/s_in])-M@np.asarray(origin_out, float)
    return ndimage.affine_transform(img, M, offset=offset, output_shape=shape_out, order=order, mode="constant", cval=cval)


def rotate(img, angle, order=1, cval=0.):
    """The image's content turned counter-clockwise by `angle` about the image centre: out(q) = img(R(-angle) q)."""
    n = img.shape[0]
    h = (n-1)/2
    return resample(img, rot(-angle), (0., 0.), 1., (h, h), 1., (h, h), img.shape, order, cval)


def highpass(g, sigma=5.):
    return g-ndimage.gaussian_filter(g, sigma)


def hann(n=N):
    w = np.hanning(n+2)[1:-1]
    return np.outer(w, w).astype(np.float32)


def _peak(corr, limit):
    """Sub-pixel peak of a circular correlation surface within |shift| <= limit pixels: (value, d_row, d_col)."""
    n = corr.shape[0]
    view = np.full_like(corr, -np.inf)
    view[:limit+1, :limit+1] = corr[:limit+1, :limit+1]
    view[:limit+1, -limit:] = corr[:limit+1, -limit:]
    view[-limit:, :limit+1] = corr[-limit:, :limit+1]
    view[-limit:, -limit:] = corr[-limit:, -limit:]
    r, c = np.unravel_index(int(np.argmax(view)), corr.shape)
    v = corr[r, c]

    def refine(m, z, p):
        d = m-2*z+p
        return 0. if d >= 0 else float(np.clip(.5*(m-p)/d, -.5, .5))
    dr = refine(corr[(r-1) % n, c], v, corr[(r+1) % n, c])
    dc = refine(corr[r, (c-1) % n], v, corr[r, (c+1) % n])
    rr = r+dr if r <= n//2 else r+dr-n
    cc = c+dc if c <= n//2 else c+dc-n
    return float(v), rr, cc


def correlate(a, b, limit=48):
    """Normalized cross-correlation of two zero-mean images over circular shifts: the shift (d_row, d_col) and
    score such that a(r, c) ~ b(r + d_row, c + d_col)."""
    fa, fb = rfft2(a), rfft2(b)
    corr = irfft2(np.conj(fa)*fb, s=a.shape)
    norm = math.sqrt(float((a*a).sum())*float((b*b).sum()))+1e-9
    v, dr, dc = _peak(corr, limit)
    return v/norm, dr, dc


def register(cur, ref, angles, limit=48):
    """Rigid alignment of `cur` on `ref` (both prepared: high-passed, windowed, zero where unknown): for each
    candidate turn the content of `cur` and correlate. Returns the list of (score, angle, d_row, d_col):
    turned-cur(r, c) ~ ref(r + d_row, c + d_col)."""
    out = []
    for a in angles:
        turned = cur if a == 0 else rotate(cur, a)
        s, dr, dc = correlate(turned, ref, limit)
        out.append((s, a, dr, dc))
    return out


def best_angle(results):
    """Parabolic refinement of the best-scoring angle among evenly spaced candidates: (angle, index)."""
    i = int(np.argmax([r[0] for r in results]))
    a = results[i][1]
    if 0 < i < len(results)-1:
        m, z, p = results[i-1][0], results[i][0], results[i+1][0]
        d = m-2*z+p
        if d < 0:
            a += float(np.clip(.5*(m-p)/d, -.5, .5))*(results[i+1][1]-results[i][1])
    return a, i
