"""Camera frames of the cell: MuJoCo's renderer (OSMesa in the sandbox) plus a RealSense-like sensor model.

RGB: pinhole, no distortion; per-episode white-balance / brightness draw, per-pixel noise. Depth (metres along the
optical axis, registered to the RGB pixels): noise sigma 1.5 mm + 2.5 mm/m^2 z^2, 1 mm quantisation, invalid (0) below
the minimum range, beyond 6 m, on depth discontinuities and on 2 % random pixels. Intrinsics are exact and published
(fx = fy = (H / 2) / tan(fovy / 2), principal point at the image centre).
"""

from __future__ import annotations

import base64
import io
import math

import mujoco
import numpy as np
from PIL import Image

DEPTH_MIN = {"head": 0.28, "ceiling": 0.40}
DEPTH_MAX = 6.0


def intrinsics(width: int, height: int, fovy_deg: float) -> dict:
    f = (height / 2.0) / math.tan(math.radians(fovy_deg) / 2.0)
    return {"width": width, "height": height, "fx": f, "fy": f, "cx": (width - 1) / 2.0, "cy": (height - 1) / 2.0,
            "fovy_deg": fovy_deg, "model": "pinhole, no distortion"}


def camera_pose_world(m, d, cam_id: int) -> tuple[np.ndarray, np.ndarray]:
    """Optical frame in the room: origin, rotation whose columns are the image x (right), y (down), z (forward) axes."""
    p = d.cam_xpos[cam_id].copy()
    R = d.cam_xmat[cam_id].reshape(3, 3)
    # MuJoCo cameras look along -z with +y up; the optical frame (OpenCV) is x right, y down, z forward
    return p, np.stack([R[:, 0], -R[:, 1], -R[:, 2]], axis=1)


class CameraRig:
    def __init__(self, m: mujoco.MjModel, sizes: dict, rng: np.random.Generator):
        self.m = m
        self.sizes = sizes                    # camera name -> (width, height)
        self._renderers = {}
        self.rng = rng
        self.tint = {name: rng.uniform(0.94, 1.06, size=3) for name in sizes}
        self.gain = {name: float(rng.uniform(0.92, 1.08)) for name in sizes}

    def _renderer(self, w, h):
        key = (w, h)
        if key not in self._renderers:
            self._renderers[key] = mujoco.Renderer(self.m, h, w)
        return self._renderers[key]

    def close(self):
        for r in self._renderers.values():
            try:
                r.close()
            except Exception:  # noqa: BLE001
                pass
        self._renderers = {}

    def render(self, d, name: str, depth: bool = True, kind: str = "ceiling"):
        w, h = self.sizes[name]
        r = self._renderer(w, h)
        r.update_scene(d, camera=name)
        rgb = r.render().astype(np.float32) / 255.0
        rgb = np.clip(rgb * self.tint[name] * self.gain[name] + self.rng.normal(0.0, 0.008, size=rgb.shape), 0, 1)
        rgb8 = (rgb * 255.0 + 0.5).astype(np.uint8)
        dep = None
        if depth:
            r.enable_depth_rendering()
            r.update_scene(d, camera=name)
            z = r.render().astype(np.float64)
            r.disable_depth_rendering()
            sigma = 0.0015 + 0.0025 * z * z
            zn = z + self.rng.normal(0.0, 1.0, size=z.shape) * sigma
            invalid = (z < DEPTH_MIN[kind]) | (z > DEPTH_MAX) | (self.rng.random(z.shape) < 0.02)
            # discontinuities: relative jump > 3 % to a 4-neighbour
            jump = np.zeros_like(z, dtype=bool)
            for ax in (0, 1):
                dz = np.abs(np.diff(z, axis=ax)) > 0.03 * np.minimum(z.take(range(z.shape[ax] - 1), axis=ax),
                                                                     z.take(range(1, z.shape[ax]), axis=ax))
                if ax == 0:
                    jump[:-1] |= dz
                    jump[1:] |= dz
                else:
                    jump[:, :-1] |= dz
                    jump[:, 1:] |= dz
            invalid |= jump
            mm = np.clip(np.round(zn * 1000.0), 0, 65535).astype(np.uint16)
            mm[invalid] = 0
            dep = mm
        return rgb8, dep


def encode_frame(name: str, index: int, rgb8: np.ndarray, depth_mm, intr: dict) -> dict:
    buf = io.BytesIO()
    Image.fromarray(rgb8).save(buf, format="PNG", compress_level=1)
    out = {"camera": name, "frame_index": index, "width": int(rgb8.shape[1]), "height": int(rgb8.shape[0]),
           "format": "png", "rgb_png_b64": base64.b64encode(buf.getvalue()).decode("ascii"), "intrinsics": intr}
    if depth_mm is not None:
        buf = io.BytesIO()
        Image.fromarray(depth_mm).save(buf, format="PNG", compress_level=1)
        out.update({"sensor_depth_u16_mm_b64": base64.b64encode(buf.getvalue()).decode("ascii"),
                    "depth_unit_m": 0.001, "depth_invalid_value": 0})
    return out
