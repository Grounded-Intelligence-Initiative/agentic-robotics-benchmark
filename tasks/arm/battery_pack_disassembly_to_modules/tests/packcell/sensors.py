"""The depth channel of the cell's two RGB-D cameras: what a real active-stereo sensor makes of the scene's true depth.

NumPy and OpenCV only; no simulator needed to read or test it (`tests/review/test_rules.py` holds it to its data
sheet). The renderer gives the exact distance to the image plane of every pixel and the colour image; `measure`
turns them into the depth frame a program receives:

1. *Lateral resolution.* The sensor matches small windows, so depth is a little blurred across the image (a Gaussian
   of `blur_px` pixels on the true depth).
2. *Noise.* Temporal noise along the optical axis, standard deviation `sigma_k` z^2 (z in metres) on a well-lit,
   mid-grey surface, rising to twice that on the darkest surfaces that still return a reading. It is spatially
   correlated over a few pixels (white noise smoothed by a Gaussian of `corr_px` pixels), like a stereo matcher's
   error, and independent from frame to frame. The mean over many frames is the true (blurred) depth.
3. *Quantization.* Depth comes from a disparity with a resolution of `subpixel` pixel at a baseline of
   `baseline_m`: the steps grow with the square of the distance (z^2 subpixel / (f baseline)).
4. *Invalid pixels* (depth 0, "no reading"): outside the working range; at depth discontinuities larger than
   `edge_px` pixels of disparity within a 5 x 5 window; where the second view of the stereo pair is occluded (a band
   beside tall edges on the side of the camera's -x); on dark surfaces (the colour image's luma below `dark[1]`: from
   none at `dark[1]` to all at `dark[0]`) and on bright, specular ones (the brightest colour channel above
   `bright[0]`: from none there to all at `bright[1]`). Dropouts come in blobs (a smoothed random field compared with
   the local dropout probability), not as single pixels.

The colour image is not changed. All draws come from the `rng` the caller passes (the cell keeps one per camera
stream, seeded from the pack); nothing here is a hidden per-pack value.
"""

import math

import cv2
import numpy as np


def _smooth(white, sigma_px):
    """White noise smoothed by a Gaussian of `sigma_px` pixels, scaled back to unit standard deviation."""
    k = int(2 * math.ceil(3 * sigma_px) + 1)
    g = cv2.getGaussianKernel(k, sigma_px).ravel()
    out = cv2.GaussianBlur(white, (k, k), sigma_px, borderType=cv2.BORDER_REFLECT)
    return out / float(np.sum(g ** 2))           # the 2D kernel is g g^T: the field's std is sum(g^2)


def _occluded(z, f, baseline, tol_px=0.5):
    """Pixels the second view of the pair (displaced by `baseline` along the camera's +x) cannot see: a pixel at
    column u with disparity d lands at u - d there; it is hidden when a nearer point to its right lands at or left of
    it. Row-wise running minimum from the right."""
    H, W = z.shape
    with np.errstate(divide="ignore"):
        d = np.where(z > 0, f * baseline / np.maximum(z, 1e-9), 0.0)
    u = np.arange(W, dtype=np.float64)[None, :]
    x2 = u - d
    # m[u] = min over u' > u of x2[u']
    rev = np.minimum.accumulate(x2[:, ::-1], axis=1)[:, ::-1]
    m = np.concatenate([rev[:, 1:], np.full((H, 1), np.inf)], axis=1)
    return m < x2 - tol_px


def measure(z_true, rgb, K, spec, rng):
    """The sensor's depth frame (float32 metres, 0 = no reading) for the true depth `z_true` (H x W, metres along the
    optical axis; 0 or non-finite where the renderer saw nothing), the registered colour image `rgb` (H x W x 3
    uint8) and the camera matrix `K`, by the data sheet `spec` (series.SENSORS[name])."""
    z = np.where(np.isfinite(z_true), z_true, 0.0).astype(np.float64)
    H, W = z.shape
    f = float(K[0][0])
    lo, hi = spec["range_m"]
    valid = (z >= lo) & (z <= hi)
    # 1. lateral resolution
    zb = cv2.GaussianBlur(z, (0, 0), spec["blur_px"], borderType=cv2.BORDER_REFLECT)
    # 4a. discontinuities: the disparity's spread in a 5 x 5 window of the true depth
    fb = f * spec["baseline_m"]
    with np.errstate(divide="ignore"):
        disp = np.where(z > 0, fb / np.maximum(z, 1e-9), 0.0)
    k5 = np.ones((5, 5), np.uint8)
    spread = cv2.dilate(disp, k5) - cv2.erode(disp, k5)
    valid &= spread <= spec["edge_px"]
    # 4b. the second view's occlusion
    valid &= ~_occluded(z, f, spec["baseline_m"])
    # 4c. dark and bright surfaces, in blobs
    rgbf = rgb[..., :3].astype(np.float64)
    luma = 0.299 * rgbf[..., 0] + 0.587 * rgbf[..., 1] + 0.114 * rgbf[..., 2]
    top = rgbf.max(axis=2)
    d0, d1 = spec["dark"]
    b0, b1 = spec["bright"]
    p_drop = np.maximum(np.clip((d1 - luma) / (d1 - d0), 0.0, 1.0), np.clip((top - b0) / (b1 - b0), 0.0, 1.0))
    field = _smooth(rng.standard_normal((H, W)), spec["corr_px"])
    valid &= ~(_phi_fast(field) < p_drop)
    # 2. noise: sigma_k z^2, up to twice that on dark surfaces
    dim = np.clip((spec["dim"] - luma) / (spec["dim"] - d0), 0.0, 1.0)
    sigma = spec["sigma_k"] * zb ** 2 * (1.0 + dim)
    zn = zb + sigma * _smooth(rng.standard_normal((H, W)), spec["corr_px"])
    # 3. quantization in disparity
    q = spec["subpixel"]
    with np.errstate(divide="ignore", invalid="ignore"):
        dq = np.round(fb / np.maximum(zn, 1e-6) / q) * q
        zq = np.where(dq > 0, fb / np.maximum(dq, 1e-12), 0.0)
    out = np.where(valid & (zq >= lo) & (zq <= hi), zq, 0.0)
    return out.astype(np.float32)


def _phi_fast(x):
    """The standard normal distribution function (Abramowitz and Stegun 7.1.26 through erf, error below 1.5e-7)."""
    a = np.abs(x) / math.sqrt(2.0)
    t = 1.0 / (1.0 + 0.3275911 * a)
    y = 1.0 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t * np.exp(-a * a)
    return 0.5 * (1.0 + np.sign(x) * y)


def stationary_sigma(spec, z):
    """The data sheet's temporal noise (1 sigma, metres) at distance z on a well-lit surface."""
    return spec["sigma_k"] * z * z


def step(spec, K, z):
    """The data sheet's depth step (metres) at distance z: one disparity step of `subpixel` pixel."""
    return z * z * spec["subpixel"] / (float(K[0][0]) * spec["baseline_m"])
