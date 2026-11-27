"""The cell's three RGB-D cameras (left, right: fixed; wrist: on the hand): MuJoCo's classic renderer (OSMesa on CPU), PNG colour + 16-bit depth.

A frame is rendered when the program asks for it (`get_camera`), at the current simulated time, and costs one frame of
the shift's pooled frame budget (depth rides along for free); asking again for the same camera at the same time
returns the same frame for free. Colour noise (sigma 2 grey levels) and depth noise (0.5 mm, 0 = no reading beyond
the range) are keyed draws of the frame number. Poses and intrinsics in the OpenCV convention (x right, y down,
z forward).
"""

import base64
import io
import math

import mujoco
import numpy as np
from PIL import Image

from .config import CONFIG

DEPTH_UNIT_M = 0.0001
NAMES = ("left", "right", "wrist")


class Cameras(object):
    def __init__(self, cell):
        self.c = cell
        self.rgb = None
        self.depth = None
        self.cache = {}
        self.renders = 0

    def _renderers(self):
        if self.rgb is None:
            self.rgb = {}
            self.depth = {}
        return self.rgb, self.depth

    def _pair(self, name):
        self._renderers()
        w, h = CONFIG["cameras"][name]["size"]
        if (w, h) not in self.rgb:
            self.rgb[(w, h)] = mujoco.Renderer(self.c.m, h, w)
            dr = mujoco.Renderer(self.c.m, h, w)
            dr.enable_depth_rendering()
            self.depth[(w, h)] = dr
        return self.rgb[(w, h)], self.depth[(w, h)]

    def intrinsics(self, name):
        cc = CONFIG["cameras"][name]
        w, h = cc["size"]
        f = 0.5 * h / math.tan(math.radians(cc["fovy_deg"]) / 2)
        return {"width": w, "height": h, "fx": f, "fy": f, "cx": (w - 1) / 2.0, "cy": (h - 1) / 2.0}

    def pose(self, name):
        d = self.c.d
        cid = self.c.cam[name]
        R = d.cam_xmat[cid].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = d.cam_xpos[cid]
        return T

    def frame(self, name, depth=False):
        c = self.c
        key = (name, c.ticks)
        if key in self.cache:
            out = dict(self.cache[key])
            if depth and "depth_png_b64" not in out:
                out.update(self._depth(name, c.frames_used))
                self.cache[key] = out
            out["repeat"] = True
            return out
        if c.frames_used >= c.frame_budget:
            return {"error": "frame_budget", "detail": "the shift's frame budget is used up"}
        c.frames_used += 1
        c.stats["frames"] += 1
        mujoco.mj_forward(c.m, c.d)
        rr, _ = self._pair(name)
        rr.update_scene(c.d, camera=name)
        px = rr.render().astype(np.int16)
        rng = np.random.default_rng([c.seed, 1, c.frames_used])
        px = px + np.round(rng.normal(0.0, CONFIG["cameras"]["rgb_noise_sigma"], px.shape)).astype(np.int16)
        px = np.clip(px, 0, 255).astype(np.uint8)
        buf = io.BytesIO()
        Image.fromarray(px).save(buf, format="PNG", compress_level=1)
        out = {"camera": name, "stamp": round(c.t, 4), "frame_no": c.frames_used,
               "rgb_png_b64": base64.b64encode(buf.getvalue()).decode("ascii"),
               "T_world_cam": [[round(float(x), 6) for x in row] for row in self.pose(name)],
               "intrinsics": self.intrinsics(name)}
        self.renders += 1
        if depth:
            out.update(self._depth(name, c.frames_used))
        self.cache = {key: out}
        return dict(out)

    def _depth(self, name, frame_no):
        c = self.c
        _, dr = self._pair(name)
        dr.update_scene(c.d, camera=name)
        z = dr.render().astype(np.float64)
        lo, hi = CONFIG["cameras"][name]["depth_range_m"]
        rng = np.random.default_rng([c.seed, 2, frame_no])
        z = z + rng.normal(0.0, CONFIG["cameras"]["depth_noise_m"], z.shape)
        raw = np.round(z / DEPTH_UNIT_M)
        raw[(z < lo) | (z > hi)] = 0
        raw = np.clip(raw, 0, 65535).astype(np.uint16)
        buf = io.BytesIO()
        Image.fromarray(raw).save(buf, format="PNG", compress_level=1)
        return {"depth_png_b64": base64.b64encode(buf.getvalue()).decode("ascii"), "depth_unit_m": DEPTH_UNIT_M,
                "depth_invalid": 0}

    def close(self):
        for dct in (self.rgb, self.depth):
            for r in (dct or {}).values():
                try:
                    r.close()
                except Exception:  # noqa: BLE001
                    pass
        self.rgb = self.depth = None
