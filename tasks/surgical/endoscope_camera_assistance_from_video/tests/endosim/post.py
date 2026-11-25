"""From the renderer's frame to the frame a controller receives: the frame clock, smoke, a smeared lens, sensor noise,
the endoscope's image disk, JPEG.

Image space only, numpy + OpenCV, inside the renderer process. None of it is a scoring input: the view rules are
judged on the simulator's state (episode.py).
"""

import cv2
import numpy as np

from . import model
from .scene import CLOCK_PITCH, CLOCK_PX, CLOCK_SLOTS, CLOCK_U0, CLOCK_V

DISK_RADIUS = 310.0
MOTION_BLUR_SHUTTER = 0.5     # the fraction of a control step the shutter is open
AE_TARGET = 96.0              # the automatic exposure's centre-weighted mean level (of 255)
JPEG_QUALITY = 90


def read_clock(rgb):
    """The slot the frame clock shows (0 .. CLOCK_SLOTS-1), or None when it cannot be read unambiguously."""
    half = int(CLOCK_PX // 2) - 1
    vals = []
    for s in range(CLOCK_SLOTS):
        u, v = int(round(CLOCK_U0 + CLOCK_PITCH * s)), int(round(CLOCK_V))
        vals.append(float(rgb[v - half:v + half + 1, u - half:u + half + 1].min(axis=-1).mean()))
    order = np.argsort(vals)
    best, second = vals[order[-1]], vals[order[-2]]
    if best < 90.0 or second > 0.45 * best:
        return None
    return int(order[-1])


class Post(object):
    """Per-procedure image effects. `finish(rgb, state)` -> the frame before masking (kept for probe copies)."""

    def __init__(self, look, seed):
        self.look = look
        self.rng = np.random.default_rng(int(seed) & 0x7FFFFFFF)
        h, w = model.IMAGE_H, model.IMAGE_W
        yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
        r = np.hypot(xx - (w - 1) / 2.0, yy - (h - 1) / 2.0)
        self.mask = np.clip((DISK_RADIUS - r) / 3.0, 0.0, 1.0)[..., None]
        self.centre_weight = np.exp(-(r / 260.0) ** 2).astype(np.float32)
        # a drifting smoke field: smooth noise, larger than the frame so it can scroll
        n = self.rng.random((40, 56)).astype(np.float32)
        self.smoke_field = cv2.resize(n, (w * 2, h * 2), interpolation=cv2.INTER_CUBIC)
        n2 = self.rng.random((90, 120)).astype(np.float32)
        self.smoke_fine = cv2.resize(n2, (w * 2, h * 2), interpolation=cv2.INTER_CUBIC)
        self.xx, self.yy = xx, yy
        self._smear_id, self._smear = 0, None
        self.gain = 1.0                 # the camera's automatic exposure (a slow gain towards a target level)
        self.noise = [self.rng.standard_normal((h, w, 3), dtype=np.float32) for _ in range(6)]
        self.wb = np.array(look["white_balance"], np.float32)[None, None, :]
        self.tint = np.array(look["smoke_tint"], np.float32)[None, None, :] * 255.0

    def _smear_map(self, smear_id):
        if smear_id != self._smear_id:
            rng = np.random.default_rng(1000003 * int(smear_id) + int(self.look["texture_seed"]))
            h, w = model.IMAGE_H, model.IMAGE_W
            blot = cv2.resize(rng.random((7, 9)).astype(np.float32), (w, h), interpolation=cv2.INTER_CUBIC)
            blot = np.clip((blot - 0.35) * 2.2, 0.0, 1.0)
            self._smear = (np.clip(0.55 + 0.45 * blot, 0.0, 1.0)[..., None],
                           np.array([rng.uniform(120, 170), rng.uniform(60, 95), rng.uniform(50, 80)], np.float32)[None, None, :])
            self._smear_id = smear_id
        return self._smear

    def _expose(self, rgb):
        """Automatic exposure, as an endoscope's camera head has it: a gain in linear light that moves the
        centre-weighted mean level towards a target over a few frames, with a soft shoulder for the highlights.
        A function of the pixel value alone once the gain is set, so it is applied as a 256-entry table."""
        lum = rgb[::2, ::2].astype(np.float32) @ np.array([0.30, 0.59, 0.11], np.float32)
        w = self.centre_weight[::2, ::2]
        level = float((lum * w).sum() / w.sum())
        want = float(np.clip((AE_TARGET / max(level, 2.0)) ** 2.2, 0.35, 14.0))
        self.gain = float(np.clip(self.gain + 0.3 * (want - self.gain), 0.35, 14.0))
        lin = np.power(np.arange(256, dtype=np.float32) * (1.0 / 255.0), 2.2) * self.gain
        lin = lin * (1.0 + lin * 0.18) / (1.0 + lin)
        table = np.power(np.clip(lin * 1.72, 0.0, 1.0), 1.0 / 2.2) * 255.0
        return table.astype(np.float32)[rgb]

    def finish(self, rgb, state):
        img = self._expose(rgb) * self.wb
        h, w = model.IMAGE_H, model.IMAGE_W
        dens = float(state.get("smoke") or 0.0)
        if dens > 0.004:
            t = int(state["step"])
            ox, oy = (3 * t) % w, (h - (5 * t) % h) % h                 # the field drifts up and sideways
            field = self.smoke_field[oy:oy + h, ox:ox + w] * 0.65 + self.smoke_fine[(oy * 2) % h:(oy * 2) % h + h, (ox * 3) % w:(ox * 3) % w + w] * 0.35
            a = dens * (0.30 + 0.55 * field) * (0.55 + 0.45 * self.centre_weight)
            src = state.get("smoke_px")
            if src is not None:                                          # the plume above the instrument that makes it
                d2 = (self.xx - float(src[0])) ** 2 + (self.yy - float(src[1]) + 40.0) ** 2
                a = a + dens * 0.5 * np.exp(-d2 / (2.0 * 95.0 ** 2)) * (0.5 + field)
            a = np.clip(a, 0.0, 0.92)[..., None]
            img = img * (1.0 - a) + self.tint * a
        flow = state.get("flow_px") or (0.0, 0.0)
        length = min(MOTION_BLUR_SHUTTER * float(np.hypot(flow[0], flow[1])), 28.0)
        if length > 1.5:
            # the camera moved during the exposure: the picture is smeared along its own motion
            n = int(2 * round(length / 2) + 1)
            kern = np.zeros((n, n), np.float32)
            c = n // 2
            ang = float(np.arctan2(flow[1], flow[0]))
            cv2.line(kern, (int(round(c - c * np.cos(ang))), int(round(c - c * np.sin(ang)))),
                     (int(round(c + c * np.cos(ang))), int(round(c + c * np.sin(ang)))), 1.0, 1)
            img = cv2.filter2D(img, -1, kern / max(float(kern.sum()), 1.0), borderType=cv2.BORDER_REPLICATE)
        smear = int(state.get("smear") or 0)
        if smear:
            m, tint = self._smear_map(smear)
            soft = cv2.GaussianBlur(img, (0, 0), 9.0)
            img = img * (1.0 - m) + (0.82 * soft + 0.18 * tint) * m
        sigma = float(self.look["noise_sigma"])
        if sigma > 0:
            # sensor noise: a fixed pool of noise frames, a fresh random shift and sign of one of them every frame
            n = self.noise[int(self.rng.integers(len(self.noise)))]
            dy, dx = int(self.rng.integers(h)), int(self.rng.integers(w))
            img = img + (sigma if self.rng.random() < 0.5 else -sigma) * np.roll(n, (dy, dx), axis=(0, 1))
        return np.clip(img, 0.0, 255.0)

    def deliver(self, frame):
        """The masked JPEG of a finished frame."""
        out = (frame * self.mask + 0.5).astype(np.uint8)
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(out, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            raise RuntimeError("JPEG encoding failed")
        return buf.tobytes()
