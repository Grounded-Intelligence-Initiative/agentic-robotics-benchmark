"""Locate colored spherical beacons using the public pelvis-mounted RGB-D camera."""

import base64
from io import BytesIO

import numpy as np
from PIL import Image
from scipy import ndimage


FOCAL = 48.0 / np.tan(np.deg2rad(37.5))
RADIUS = 0.12
CAMERA_IN_PELVIS = np.array([0.28, 0.0, 0.33])
HEIGHT, WIDTH = 96, 128
COLORS = ("blue", "red", "magenta", "yellow", "green")
CONNECTIVITY = np.ones((3, 3), dtype=np.uint8)


def horizontal_axes(gravity_body):
    """Return pelvis-frame unit vectors for level forward and left."""
    up = -np.asarray(gravity_body, dtype=float)
    if up.shape != (3,) or not np.all(np.isfinite(up)) or np.linalg.norm(up) < 1e-9:
        raise ValueError("gravity_body must be a finite, nonzero 3-vector")
    up = up / np.linalg.norm(up)
    forward = np.array([1.0, 0.0, 0.0]) - up[0] * up
    if np.linalg.norm(forward) < 1e-8:
        forward = np.cross([0.0, 1.0, 0.0], up)
    forward /= np.linalg.norm(forward)
    return forward, np.cross(up, forward)


def _png(encoded):
    with Image.open(BytesIO(base64.b64decode(encoded))) as image:
        return np.array(image)


def _classify(rgb):
    # Channel order and ratios survive multiplicative shading. Ratios leave
    # room for bright highlights, including clipping in two strong channels.
    r, g, b = rgb.astype(np.float32).transpose(2, 0, 1)
    hi = np.maximum(np.maximum(r, g), b)
    lo = np.minimum(np.minimum(r, g), b)
    colored = (hi >= 48) & (hi - lo >= 0.43 * hi)
    labels = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    masks = (
        b > 1.65 * np.maximum(r, g),
        r > 1.75 * np.maximum(g, b),
        (np.minimum(r, b) > 1.65 * g) & (np.minimum(r, b) > 0.55 * np.maximum(r, b)),
        (np.minimum(r, g) > 1.65 * b) & (np.minimum(r, g) > 0.55 * np.maximum(r, g)),
        g > 1.6 * np.maximum(r, b),
    )
    for index, mask in enumerate(masks, 1):
        labels[colored & mask] = index
    return labels


def _center(rows, cols, depth):
    # RGB edge pixels can contain blended colors while their depth belongs
    # to the background. Keep the densest physically possible depth band.
    z = depth[rows, cols].astype(float) * 0.001
    order = np.argsort(z)
    sorted_z = z[order]
    ends = np.searchsorted(sorted_z, sorted_z + 2 * RADIUS + 0.005)
    start = int(np.argmax(ends - np.arange(len(z))))
    keep = order[start : ends[start]]
    z, rows, cols = z[keep], rows[keep], cols[keep]
    rays = np.column_stack(
        (np.ones_like(z), (63.5 - cols) / FOCAL, (47.5 - rows) / FOCAL)
    )
    points = rays * z[:, None]
    distances = np.linalg.norm(points, axis=1)
    front = points[distances <= distances.min() + 0.008].mean(axis=0)
    # A one-pixel detection has unknown tangential offset. Its ray and
    # known radius still give a usable center even without a fitted sphere.
    initial = front + RADIUS * front / np.linalg.norm(front)
    center = initial.copy()
    if len(points) >= 3:
        for _ in range(8):
            delta = center - points
            lengths = np.maximum(np.linalg.norm(delta, axis=1), 1e-8)
            jacobian = delta / lengths[:, None]
            residual = lengths - RADIUS
            weights = np.minimum(1.0, 0.008 / np.maximum(abs(residual), 1e-9))
            weighted = jacobian.T * weights
            # A weak positional prior stabilizes tiny or nearly collinear patches.
            prior = 0.015
            step = np.linalg.solve(
                weighted @ jacobian + prior * np.eye(3),
                weighted @ residual + prior * (center - initial),
            )
            center -= step
            if np.linalg.norm(step) < 1e-5:
                break
    residuals = np.abs(np.linalg.norm(points - center, axis=1) - RADIUS)
    support = int(np.count_nonzero(residuals < 0.018))
    # A large same-color plane, e.g. a painted surface, cannot fit this sphere.
    if support < 0.60 * len(points) or np.dot(center - front, front) <= 0:
        return None, 0
    return center, support


def camera_arrays(camera):
    """Decode the aligned RGB and optical-axis depth exactly once per observation."""
    rgb = _png(camera["rgb_png"])
    depth = _png(camera["depth_mm_png"])
    if rgb.shape != (HEIGHT, WIDTH, 3) or depth.shape != (HEIGHT, WIDTH):
        raise ValueError("expected 128 by 96 aligned RGB and depth images")
    return rgb, depth


def beacons(camera, gravity_body, *, arrays=None):
    """Return each visible sphere center as [forward, left] metres from the pelvis."""
    rgb, depth = camera_arrays(camera) if arrays is None else arrays
    level = np.stack(horizontal_axes(gravity_body))
    labels = _classify(rgb)
    labels[depth == 0] = 0
    found = {}
    for index, color in enumerate(COLORS, 1):
        mask = labels == index
        if not mask.any():
            continue
        components, count = ndimage.label(mask, CONNECTIVITY)
        areas = np.bincount(components.ravel(), minlength=count + 1)
        best = 0
        for label in np.argsort(areas[1:])[::-1] + 1:
            if areas[label] <= best:
                break
            rows, cols = np.nonzero(components == label)
            center, support = _center(rows, cols, depth)
            if support > best:
                horizontal = level @ (center + CAMERA_IN_PELVIS)
                # The yellow pivot lamp lies six metres from the turning station.
                # Reject the short, gold-painted edges of the outgoing roof.
                if color == "yellow" and not 5.45 < np.linalg.norm(horizontal) < 6.55:
                    continue
                found[color] = horizontal
                best = support
    return found


def gate_openings(rgb, depth, gravity_body, xy, yaw):
    """Map visible cabinet ends into the horizontal odometry frame.

    A vertical end has foreground cabinet pixels on one side and space at least
    30 cm farther away on the other. This also distinguishes two teal panels
    seen through each other. The public opening spans about one metre.
    """
    r, g, b = rgb.astype(float).transpose(2, 0, 1)
    cabinet = (g > 1.6 * r) & (b > 1.6 * r) & (g > 0.65 * b) & (b > 0.65 * g)
    z_mm = depth.astype(np.int32)
    plane = np.stack(horizontal_axes(gravity_body))
    c, s = np.cos(yaw), np.sin(yaw)
    horizontal = np.array([[c, -s], [s, c]]) @ plane
    samples = []
    for direction in (-1, 1):
        # Require multiple rows of a consistent vertical discontinuity, to
        # discard the top edge of a cabinet in a rolled camera.
        next_z = np.roll(z_mm, -2 * direction, axis=1)
        edge = cabinet & (z_mm > 0) & ((next_z == 0) | (next_z > z_mm + 300))
        edge[:, :2] = edge[:, -2:] = False
        runs = ndimage.minimum_filter1d(edge, size=5, axis=0, mode="constant")
        rows, cols = np.nonzero(runs)
        if not len(rows):
            continue
        z = z_mm[rows, cols] * 0.001
        rays = np.column_stack(
            (
                np.ones(len(z)),
                (63.5 - cols - direction * 0.5) / FOCAL,
                (47.5 - rows) / FOCAL,
            )
        )
        points = (rays * z[:, None] + CAMERA_IN_PELVIS) @ horizontal.T + xy
        points[:, 0] += 0.07
        points[:, 1] -= direction * 0.51
        # Outside ends of the long furniture lie beyond the public aisle.
        samples.extend(points[np.abs(points[:, 1]) < 0.94])
    if not samples:
        return []
    values = np.asarray(samples)
    ordered = values[np.argsort(values[:, 0])]
    groups = np.split(ordered, np.flatnonzero(np.diff(ordered[:, 0]) > 0.36) + 1)
    return [np.median(group, axis=0) for group in groups if len(group) >= 3]
