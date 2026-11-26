"""Estimate tabletop target centers from calibrated color images.

Only the camera calibration, object's nominal dimensions and known pad/table
heights enter the estimator. Its inputs can be saved RGBs from the task sensor.
Color segmentation and a robust fit of the projected solid silhouettes handle
changing image scale and elevated objects. Fully hidden targets return None.
"""

from itertools import product

import numpy as np
from scipy.ndimage import binary_erosion
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


# Silhouette support along evenly spaced image directions is insensitive to interior occlusion.
_ANGLES = np.arange(16) * 2 * np.pi / 16
_DIRECTIONS = np.stack((np.cos(_ANGLES), np.sin(_ANGLES)))
_BOX_CORNERS = np.array(list(product((-1.0, 1.0), repeat=3)))
_CYL_ANGLES = np.arange(32) * 2 * np.pi / 32
_CYL_POINTS = np.c_[
    np.tile(np.cos(_CYL_ANGLES), 2), np.tile(np.sin(_CYL_ANGLES), 2), np.repeat([-1.0, 1.0], 32)
]


def _masks(image):
    if image.ndim != 3 or image.shape[-1] != 3 or image.dtype != np.uint8:
        raise ValueError("Each camera requires an RGB8 image with three channels")
    red, green, blue = np.moveaxis(image.astype(np.int32), -1, 0)
    # Channel ratios cover ambient, lit and shadowed faces. The wooden table
    # reflects much more blue than the orange object relative to its red channel.
    orange = (red > 23) & (green * 10 < 8 * red) & (green * 10 > 3 * red) & (blue * 4 < red)
    pad = (green > 23) & (red * 2 < green) & (blue * 10 < green * 9) & (blue * 2 > green)
    return orange, pad


def _view(mask, cam):
    if mask.shape != (cam["height"], cam["width"]):
        raise ValueError("Raster size does not match camera calibration")
    if mask.sum() < 12:
        return None
    y, x = np.nonzero(mask & ~binary_erosion(mask))
    pixels = np.column_stack((x, y))
    directions = pixels @ _DIRECTIONS
    furthest = np.argmax(directions, axis=0)
    bounds = directions[furthest, np.arange(directions.shape[1])] + 0.74
    # When a silhouette meets the image edge the true extremum is unknown.
    interior = (
        (x[furthest] > 0)
        & (y[furthest] > 0)
        & (x[furthest] < cam["width"] - 1)
        & (y[furthest] < cam["height"] - 1)
    )
    axis = np.array([cam["right"], -np.array(cam["up"]), cam["forward"]]).T
    center_px = np.array(cam["center_px"])
    bbox_center = (pixels.min(axis=0) + pixels.max(axis=0)) / 2
    ray = axis @ np.r_[(bbox_center - center_px) / cam["focal_px"], 1.0]
    ray /= np.linalg.norm(ray)
    return np.array(cam["position"]), axis, center_px, cam["focal_px"], ray, bounds, interior


def _fit(views, vertices, z_guess, kind, top_views=None):
    views = [v for v in views if v is not None and np.count_nonzero(v[-1]) >= 4]
    if not views or (kind != "pad" and len(views) < 2):
        return None, None
    # Two rays provide a metric initial guess even when the object is airborne.
    # A single view can localize the pad on its known horizontal support.
    center = np.mean(
        [pos + ray * ((z_guess - pos[2]) / ray[2]) for pos, _, _, _, ray, _, _ in views], axis=0
    )
    if len(views) >= 2:
        axes = [np.eye(3) - np.outer(view[4], view[4]) for view in views]
        stereo = np.linalg.lstsq(
            np.sum(axes, axis=0), sum(a @ v[0] for a, v in zip(axes, views)), rcond=None
        )[0]
        if kind != "pad" and abs(stereo[2] - z_guess) < 0.55:
            center = stereo
    angle_count = 3 if kind == "box" else 2

    def geometry(params):
        if kind == "pad":
            return np.r_[params, z_guess], vertices
        rotvec = np.r_[params[3:], 0.0] if kind == "cylinder" else params[3:]
        return params[:3], Rotation.from_rotvec(rotvec).apply(vertices)

    top_views = [v for v in (top_views or []) if v is not None]
    top = vertices[:, 2] == vertices[:, 2].max()

    def residual(params):
        xyz, rotated = geometry(params)
        residuals = []
        for view, geom, weight in [(v, rotated, 1.0) for v in views] + [
            (v, rotated[top], 0.65) for v in top_views
        ]:
            pos, axis, midpoint, focal, _, bounds, valid = view
            pts = (xyz + geom - pos) @ axis
            uv = (pts[:, :2] / np.maximum(pts[:, 2:], 0.01)) * focal + midpoint
            pred = (uv @ _DIRECTIONS).max(axis=0)
            residuals.extend((pred - bounds)[valid] * weight)
        if kind != "pad":
            # Weak regularisation resolves silhouette symmetries (e.g. cylinders).
            residuals.extend(params[3:] * 0.45)
        return np.asarray(residuals)

    initial = center[:2] if kind == "pad" else np.r_[center, np.zeros(angle_count)]
    fitted = least_squares(
        residual, initial, loss="soft_l1", f_scale=0.9, max_nfev=28, ftol=0.0002, xtol=0.00002
    )
    xyz, _ = geometry(fitted.x)
    return xyz.tolist(), float(np.median(np.abs(residual(fitted.x))))


def _bright_face(mask, image, q):
    if mask.sum() < 12:
        return np.zeros_like(mask)
    channel = image[:, :, 1]
    upper = np.percentile(channel[mask], q)
    return mask & (channel >= upper * 0.96)


def _pad_center(plane_points, half_width):
    if not plane_points:
        return None
    all_xy = np.concatenate(plane_points, axis=0)
    lo, hi = np.quantile(all_xy, [0.001, 0.999], axis=0)
    # A mostly obscured side may end prematurely. Prefer the opposite edge
    # when the measured footprint is shorter than the known pad dimension.
    mid = (lo + hi) / 2
    for dim in range(2):
        if (hi - lo)[dim] < 2 * half_width - 0.005:
            low_votes = np.count_nonzero(all_xy[:, dim] < lo[dim] + 0.003)
            high_votes = np.count_nonzero(all_xy[:, dim] > hi[dim] - 0.003)
            mid[dim] = lo[dim] + half_width if low_votes > high_votes else hi[dim] - half_width
    return mid.tolist()


def measure_scene(images, cameras, task):
    """Measure object center and pad target; return None when a class is invisible.

    `images` and `cameras` use corresponding view keys. `task` selects nominal
    geometry from the benchmark, including an object's shape and dimensions.
    `goal_pos` is at the intended object center, `pad_pos` is at the pad top.
    """
    # Public tabletop geometry shared by both tasks; no simulator module is
    # imported so recorded RGBs can also be measured in a separate process.
    PAD_HALF_WIDTH, PAD_TOP, TABLE_TOP = 0.069, 0.745, 0.730
    sizes = {"box_to_pad": ("box", 0.026, 0.060), "cylinder_to_pad": ("cylinder", 0.024, 0.050)}
    kind, half_width, half_height = sizes[task.removesuffix("_challenge")]
    obj_sz = np.array([half_width, half_width, half_height])
    object_points = (_BOX_CORNERS if kind == "box" else _CYL_POINTS) * obj_sz
    pad_height = (PAD_TOP - TABLE_TOP) / 2
    pad_points = _BOX_CORNERS * [PAD_HALF_WIDTH, PAD_HALF_WIDTH, pad_height]
    object_views, top_views, pad_views, pad_xy, pixel_counts = [], [], [], [], {}
    for name, image in images.items():
        orange, green = _masks(np.asarray(image))
        pixel_counts[name] = dict(object=int(orange.sum()), pad=int(green.sum()))
        cam = cameras[name]
        object_views.append(_view(orange, cam))
        top = _bright_face(orange, image, 98)
        # A visible top face appears above the complete silhouette in the image.
        if (
            12 <= top.sum() <= 0.5 * orange.sum()
            and np.nonzero(top)[0].mean() < np.nonzero(orange)[0].mean() - 2
        ):
            top_views.append(_view(top, cam))
        pad_views.append(_view(green, cam))
        pad_top = _bright_face(green, image, 85)
        y, x = np.nonzero(pad_top)
        if len(x) >= 12:
            u = (x - cam["center_px"][0]) / cam["focal_px"]
            v = (y - cam["center_px"][1]) / cam["focal_px"]
            rays = np.array(cam["forward"]) + u[:, None] * cam["right"] - v[:, None] * cam["up"]
            pos = np.array(cam["position"])
            plane = pos + rays * ((PAD_TOP - pos[2]) / rays[:, 2:])
            pad_xy.append(plane[:, :2])
    obj, obj_error = _fit(object_views, object_points, TABLE_TOP + obj_sz[2], kind, top_views)
    pad, pad_error = _fit(pad_views, pad_points, TABLE_TOP + pad_height, "pad")
    xy = _pad_center(pad_xy, PAD_HALF_WIDTH)
    if xy is not None:
        pad = [*xy, PAD_TOP]
    elif pad is not None:
        pad[2] = PAD_TOP
    goal = None if pad is None else [pad[0], pad[1], pad[2] + obj_sz[2]]
    # Report geometric uncertainty scale from observed silhouette residual and
    # camera baseline. It measures sensitivity and is not a confidence interval.
    visible = [v for v in object_views if v is not None]
    uncertainty = None
    if obj is not None:
        rays = [np.asarray(obj) - v[0] for v in visible]
        unit = [r / np.linalg.norm(r) for r in rays]
        baseline = max(np.linalg.norm(np.cross(a, b)) for a in unit for b in unit)
        meters_per_pixel = max(np.linalg.norm(r) / v[3] for r, v in zip(rays, visible))
        uncertainty = float((1.0 + obj_error) * meters_per_pixel / max(0.01, baseline))
    pad_uncertainty = None
    if pad is not None:
        pad_ranges = [np.linalg.norm(np.asarray(pad) - v[0]) / v[3] for v in pad_views if v is not None]
        pad_uncertainty = float((1.0 + (pad_error or 0.0)) * max(pad_ranges)) if pad_ranges else None
    return dict(
        estimator="rgb-silhouette-v2",
        object_pos=obj,
        pad_pos=pad,
        goal_pos=goal,
        goal_views=sum(v is not None for v in pad_views),
        goal_uncertainty_scale_m=pad_uncertainty,
        object_views=len(visible),
        object_uncertainty_scale_m=uncertainty,
        visible_pixels=pixel_counts,
        object_fit_error_px=obj_error,
        pad_fit_error_px=pad_error,
    )
