"""Save an observation's frames as PNG files to look at.

    from save_frames import save_frames
    save_frames(obs, "/tmp/look", "t%04d" % t)        # writes <prefix>_overhead.png, _overhead_depth.png, ...
"""
import os

import cv2
import numpy as np


def save_frames(obs, directory, prefix="frame", depth_range=(0.90, 0.955)):
    os.makedirs(directory, exist_ok=True)
    for cam, p in obs["frames"].items():
        rgb = np.load(p["rgb"])
        depth = np.load(p["depth"])
        cv2.imwrite(os.path.join(directory, "%s_%s.png" % (prefix, cam)), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        lo, hi = depth_range if cam == "overhead" else (float(depth.min()), float(depth.max()) + 1e-6)
        d8 = np.clip((hi - depth) / (hi - lo) * 255.0, 0, 255).astype(np.uint8)
        cv2.imwrite(os.path.join(directory, "%s_%s_depth.png" % (prefix, cam)), cv2.applyColorMap(d8, cv2.COLORMAP_VIRIDIS))
