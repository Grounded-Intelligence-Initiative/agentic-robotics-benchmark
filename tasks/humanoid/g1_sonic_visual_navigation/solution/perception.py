"""Estimate moving shutter edges from one calibrated public RGB-D observation."""

import numpy as np

from vision import CAMERA_IN_PELVIS, FOCAL, camera_arrays, horizontal_axes

# Public geometry, in metres in the aisle frame (x forward, y left).
ISLAND_HALF = 0.22
OPENING = 1.0
INGRESS = 0.72
STATIC_HALF_DEPTH = 0.09
SHUTTER_OFFSET_X = -0.14
SHUTTER_HALF_DEPTH = 0.045


def _plane(x_world, mask):
    """Find the range of a front face, rejecting a grazing view of only its top or side."""
    # Metric forward range is nearly constant in both image directions on the face.
    # A stripe from the horizontal top surface can have an apparently strong range mode.
    planar = mask.copy()
    for axis in (0, 1):
        contiguous = np.zeros_like(mask)
        for shift in (-1, 1):
            close = np.abs(x_world - np.roll(x_world, shift, axis=axis)) < 0.0025
            neighbor = np.roll(mask, shift, axis=axis)
            if axis == 1:
                neighbor[:, 0 if shift == 1 else -1] = False
            else:
                neighbor[0 if shift == 1 else -1] = False
            contiguous |= close & neighbor
        planar &= contiguous
    values = x_world[planar]
    if len(values) < 6:
        return None
    x = np.sort(values)
    stop = np.searchsorted(x, x + 0.030, side="right")
    i = int(np.argmax(stop - np.arange(len(x))))
    if stop[i] - i < max(6, 0.06 * len(x)):
        return None
    return float(np.median(x[i : stop[i]]))


def shutters_from_rgbd(camera, gravity_body, xy, yaw, *, barrier_hint=None):
    """Localize the barrier and the inward-facing vertical edge of each amber shutter.

    Input xy and yaw are pelvis odometry relative to reset, in the aisle frame.
    Distances are metres. A side's coverage is the horizontal fraction of its aperture
    inside the camera at shutter range. Confidence is a geometric support score in [0,1].
    Invisible or occluded edges have null measurements, even if the other side is visible.
    """
    rgb, depth = camera_arrays(camera)
    r, g, b = rgb.astype(np.float32).transpose(2, 0, 1)
    valid = depth > 0
    amber = (r > 50) & (r > 1.4 * g) & (g > 1.75 * b) & (g > 0.22 * r) & valid
    teal = (
        (g > 40)
        & (g > 1.65 * r)
        & (b > 1.7 * r)
        & (g > 0.59 * b)
        & (b > 0.72 * g)
        & valid
    )
    height, width = depth.shape
    v, u = np.indices(depth.shape)
    rays = np.stack((np.ones_like(u), (63.5 - u) / FOCAL, (47.5 - v) / FOCAL), axis=-1)
    cs, sn = np.cos(yaw), np.sin(yaw)
    basis = np.array([[cs, -sn], [sn, cs]]) @ np.stack(horizontal_axes(gravity_body))
    directions = rays @ basis.T
    origin = np.asarray(xy, dtype=float) + basis @ CAMERA_IN_PELVIS
    world = origin + depth[..., None] * 0.001 * directions
    x, y = world.transpose(2, 0, 1)

    # The forward-facing amber and teal faces are at different known x offsets.
    # A side face has nonconstant x and cannot dominate ordinary front observations.
    front_shift = SHUTTER_OFFSET_X - SHUTTER_HALF_DEPTH
    if barrier_hint is not None:
        if not np.isfinite(barrier_hint):
            raise ValueError("barrier_hint must be a finite previous barrier center x")
        amber &= np.abs(x - front_shift - barrier_hint) <= 0.35
        teal &= np.abs(x + STATIC_HALF_DEPTH - barrier_hint) <= 0.35
    orange_front = _plane(x, amber)
    # Only the moving fork has amber. A distant static teal gate can fill most of the frame
    # through its open aperture; it cannot corroborate the amber slab at the moving fork.
    gate = None if orange_front is None else orange_front - front_shift
    if gate is not None:
        teal &= np.abs(x + STATIC_HALF_DEPTH - gate) <= 0.10
    static_front = _plane(x, teal)
    consistent = (
        gate is not None
        and static_front is not None
        and abs(static_front + STATIC_HALF_DEPTH - gate) < 0.060
    )
    if gate is None and static_front is not None and barrier_hint is not None:
        gate = static_front + STATIC_HALF_DEPTH
    result = dict(
        barrier_x=gate,
        front_x=None if gate is None else gate + front_shift,
        static_front_x=static_front,
        amber_front_x=orange_front,
        confidence=0.0 if gate is None else 1.0 if consistent else 0.65,
    )
    for side, sign in (("left", 1), ("right", -1)):
        info = dict(
            visible=False,
            clearance=None,
            inner_edge_y=None,
            coverage=0.0,
            confidence=0.0,
            edge_rows=0,
            ingress_y=None if gate is None else sign * INGRESS,
        )
        result[side] = info
        if gate is None or gate + front_shift <= origin[0] + 0.02:
            continue
        shutter_front = gate + front_shift
        # Restrict to the shutter slab. Color/depth mixed silhouette pixels often carry
        # the room behind the orange body. These never enter its metric edge estimate.
        face = amber & (x >= shutter_front - 0.045) & (x <= shutter_front + 0.123)
        object_rows = np.flatnonzero(
            np.any(face | (teal & (np.abs(x - gate) < 0.14)), axis=1)
        )
        if not len(object_rows):
            continue
        row = int(np.median(object_rows))
        fwd = directions[row, :, 0]
        projected_y = origin[1] + (shutter_front - origin[0]) * directions[
            row, :, 1
        ] / np.where(fwd > 0.05, fwd, np.nan)
        valid_y = sign * projected_y[np.isfinite(projected_y)]
        if len(valid_y):
            info["coverage"] = float(
                np.clip(
                    min(ISLAND_HALF + OPENING, valid_y.max())
                    - max(ISLAND_HALF, valid_y.min()),
                    0.0,
                    OPENING,
                )
                / OPENING
            )
        facing = sign * np.sign(cs)
        if facing == 0:
            continue
        step = int(facing)
        # Walk inward along an image row. The next color must leave amber without an
        # intervening nearer occluder. Do not use the image border as a shutter edge.
        neighbor_face = np.roll(face, -step, axis=1)
        neighbor_depth = np.roll(depth.astype(float), -step, axis=1)
        edge = (
            face
            & ~neighbor_face
            & ((neighbor_depth == 0) | (neighbor_depth >= depth.astype(float) - 35))
        )
        edge[:, :2] = edge[:, -2:] = False
        rows, cols = np.nonzero(edge)
        if not len(rows):
            continue
        # Unproject boundary half-pixels using the depth on the orange side.
        edge_y = (
            y[rows, cols] - 0.5 * step / FOCAL * basis[1, 1] * depth[rows, cols] * 0.001
        )
        outward = sign * edge_y
        possible = (outward >= 0.10) & (outward <= 1.36)
        rows, outward = rows[possible], outward[possible]
        if not len(rows):
            continue
        order = np.argsort(outward)
        outward, rows = outward[order], rows[order]
        # Require a vertical line through several different image rows, excluding a
        # lone pixel, a top-face color transition and varying silhouette depths.
        tolerance = float(np.clip((shutter_front - origin[0]) / FOCAL, 0.012, 0.052))
        stop = np.searchsorted(outward, outward + tolerance, side="right")
        i = int(np.argmax(stop - np.arange(len(outward))))
        group_y, group_rows = outward[i : stop[i]], rows[i : stop[i]]
        support = len(np.unique(group_rows))
        if support < 4 or np.ptp(group_rows) < 4:
            continue
        measured = float(np.median(group_y))
        info.update(
            visible=True,
            inner_edge_y=sign * measured,
            clearance=float(np.clip(measured - ISLAND_HALF, 0, OPENING)),
            confidence=float(min(1.0, support / 9.0) * result["confidence"]),
            edge_rows=support,
        )
    return result
