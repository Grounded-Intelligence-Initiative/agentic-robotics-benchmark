"""Read the participant's RGB-D stream and measure the visible gap to the workstation."""
import base64
from io import BytesIO

import numpy as np
from PIL import Image

CAMERAS = ("hand", "overview", "side")
PROPRIO = {"q": 29, "dq": 29, "base_gyro": 3, "gravity_body": 3}


def pixels(encoded, size, depth=False):
    with Image.open(BytesIO(base64.b64decode(encoded, validate=True))) as image:
        if image.format != "PNG" or image.size != size:
            raise ValueError("unexpected camera encoding or dimensions")
        if depth:
            array = np.asarray(image)
            if array.dtype.kind not in "ui" or array.ndim != 2 or np.any(array > 65535):
                raise ValueError("expected an unsigned millimetre depth image")
            return array.astype(np.uint16)
        if image.mode != "RGB":
            raise ValueError("expected an RGB observation")
        return np.array(image, dtype=np.uint8)


def observe(packet):
    if packet["protocol"] != "g1-sonic-visual-delivery/1" or packet["task"] != "visual_delivery":
        raise ValueError("expected a visual delivery observation")
    state = packet["observation"]
    if set(state) != set(PROPRIO):
        raise ValueError("expected joint and IMU measurements")
    proprio = {name: np.asarray(state[name], dtype=np.float64) for name in PROPRIO}
    if any(x.shape != (PROPRIO[name],) or not np.isfinite(x).all() for name, x in proprio.items()):
        raise ValueError("invalid joint or IMU vector")
    rgb, views = {}, {}
    small, planner = packet["vision"], packet["planner_vision"]
    if small is not None:
        if planner is None or not all(f["tick"] == packet["tick"] for f in (small, planner)):
            raise ValueError("camera packets must be aligned with the robot measurements")
        for name in CAMERAS:
            rgb[name] = pixels(small["images"][name], (128, 96))
            intr = planner["intrinsics"][name]
            size = (intr["width"], intr["height"])
            views[name] = {"rgb": pixels(planner["images"][name], size), "intrinsics": intr,
                           "depth_mm": pixels(planner["depth_mm"][name], size, depth=True)}
    return proprio, rgb, views


def station_gap(views):
    """Estimate the distance in metres from the robot shell to the center of the table."""
    estimates = []
    for view in views.values():
        rgb = view["rgb"][::3, ::3].astype(np.int16)
        r, g, b = rgb.transpose(2, 0, 1)
        z = view["depth_mm"][::3, ::3].astype(np.float64) * .001
        wood = (r - g > 20) & (g - b > 15) & (g > 55) & (b > 35) & (z > 0)
        shell = (rgb.min(axis=2) > 150) & (np.ptp(rgb, axis=2) < 32) & (z > 0)
        if wood.sum() < 24 or shell.sum() < 10:
            continue
        y, x = np.indices(z.shape)
        intr = view["intrinsics"]
        cx, cy = intr["center_px"]
        focal = intr["focal_px"]
        xyz = np.stack(((x * 3 - cx) * z / focal, (y * 3 - cy) * z / focal, z), axis=-1)
        center = np.median(xyz[wood], axis=0)
        distances = np.linalg.norm(xyz[shell] - center, axis=1)
        estimates.append(np.quantile(distances, .2))
    return float(np.median(estimates)) if estimates else None


def target_visible(images):
    for rgb in images.values():
        r, g, b = rgb.astype(np.int16).transpose(2, 0, 1)
        if np.count_nonzero((g-r > 35) & (g-b > 15) & (b-r > 10)) >= 12:
            return True
    return False
