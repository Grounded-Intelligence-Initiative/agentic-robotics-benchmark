"""Estimate the native rack entrance from one public aligned RGB-D camera and pelvis odometry."""

import numpy as np
from scipy import ndimage

from vision import (
    CAMERA_IN_PELVIS,
    FOCAL,
    HEIGHT,
    WIDTH,
    camera_arrays,
    horizontal_axes,
)


V, U = np.indices((HEIGHT, WIDTH))
RAYS = np.stack(
    (np.ones_like(U), ((WIDTH - 1) / 2 - U) / FOCAL, ((HEIGHT - 1) / 2 - V) / FOCAL),
    axis=-1,
)


def _mode(values, tolerance, minimum):
    """Median of the most-supported interval, ignoring isolated points."""
    a = np.sort(values)
    if a.size < minimum:
        return None
    end = np.searchsorted(a, a + tolerance, side="right")
    i = int(np.argmax(end - np.arange(len(a))))
    return float(np.median(a[i : end[i]])) if end[i] - i >= minimum else None


def _vertical_faces(points, mask, normal):
    """Centers and horizontal normals of observable flat faces facing the departure aisle."""
    q = points[:-1, :-1]
    dx = points[:-1, 1:] - q
    dy = points[1:, :-1] - q
    valid = mask[:-1, :-1] & mask[:-1, 1:] & mask[1:, :-1]
    valid &= (np.sum(dx * dx, axis=2) < 0.10**2) & (np.sum(dy * dy, axis=2) < 0.10**2)
    p, a, b = q[valid], dx[valid], dy[valid]
    n = np.cross(a, b)
    length = np.linalg.norm(n, axis=1)
    n /= np.maximum(length, 1e-12)[:, None]
    facing = n[:, :2] @ normal
    keep = (np.abs(n[:, 2]) < 0.20) & (np.abs(facing) > 0.87)
    return p[keep, :2], n[keep, :2] * np.sign(facing[keep, None])


def rack_from_rgbd(camera, gravity_body, xy, yaw, nominal_heading, prior=None):
    """Return reset-local entrance center, outgoing heading, width and physical roof clearance.

    Z is measured from an observed floor plane. An unavailable measurement is None;
    'visible' requires a fresh entrance and height. A previous estimate can disambiguate
    rack segments but is never returned as a fresh observation.
    """
    rgb, mm = camera_arrays(camera)
    r, g, b = rgb.astype(float).transpose(2, 0, 1)
    valid = (mm > 0) & (mm < 7500)
    blue = valid & (b > 35) & (b > 1.25 * g) & (g > 1.32 * r) & (r > 0.13 * b)
    gold = valid & (r > 45) & (r > 1.10 * g) & (g > 0.48 * r) & (b < 0.66 * g)
    axes = horizontal_axes(gravity_body)
    up = np.cross(*axes)
    c, s = np.cos(yaw), np.sin(yaw)
    horizontal = np.array([[c, -s], [s, c]]) @ np.stack(axes)
    points = (RAYS * (mm[..., None] * 0.001) + CAMERA_IN_PELVIS) @ np.vstack(
        (horizontal, up)
    ).T
    points[..., :2] += xy
    nominal = np.array([np.cos(nominal_heading), np.sin(nominal_heading)])
    side = np.array([-nominal[1], nominal[0]])
    z = points[..., 2]
    neutral = rgb.max(axis=2) - rgb.min(axis=2) < 0.36 * rgb.max(axis=2)
    floor_mask = valid & neutral & (z < -0.27) & (z > -1.50) & (V > HEIGHT * 0.42)
    floor = _mode(z[floor_mask], 0.008, 25)
    empty = dict(
        visible=False,
        center_xy=None,
        heading=None,
        width_m=None,
        underside_m=None,
        back_xy=None,
        depth_m=None,
        confidence=0.0,
        missing=["roof", "entrance"],
        floor_z_local=floor,
    )
    if floor is None:
        empty["missing"].append("floor")
    labels, _ = ndimage.label(gold, structure=np.ones((3, 3), int))
    candidates = []
    for i, slices in enumerate(ndimage.find_objects(labels), 1):
        if slices is None:
            continue
        v, u = slices
        if u.stop - u.start < 12:
            continue
        roof = labels == i
        if roof.sum() < 22:
            continue
        roof_points = points[roof]
        fs, ls = roof_points[:, :2] @ nominal, roof_points[:, :2] @ side
        if np.ptp(ls) < 0.34:
            continue
        post = blue.copy()
        f_all = points[..., :2] @ nominal
        l_all = points[..., :2] @ side
        post &= (f_all > fs.min() - 0.31) & (f_all < fs.max() + 0.25)
        post &= (l_all > ls.min() - 0.23) & (l_all < ls.max() + 0.23)
        pp, pn = _vertical_faces(points, post, nominal)
        rp, rn = _vertical_faces(points, roof, nominal)
        norms = np.concatenate((pn, rn))
        if len(norms) < 5:
            continue
        axis = np.median(norms, axis=0)
        axis /= np.linalg.norm(axis)
        lateral = np.array([-axis[1], axis[0]])
        # Positive range along the optical axis sets the half-pixel edge uncertainty.
        pixel_m = float(np.median(mm[roof])) * 0.001 / FOCAL
        roof_z = roof_points[:, 2]
        # Use points belonging to a locally horizontal face. The sloped image of the
        # fascia alone has a different vertical projection on successive pixels.
        zl = z[:-1, :-1]
        level = roof[:-1, :-1] & roof[:-1, 1:] & roof[1:, :-1]
        level &= (np.abs(z[:-1, 1:] - zl) < 0.003) & (np.abs(z[1:, :-1] - zl) < 0.003)
        low = _mode(zl[level], 0.007, 5)
        if low is None:
            low = float(np.percentile(roof_z, 2) - 0.32 * pixel_m)
        height = None if floor is None else float(low - floor)
        heading = float(np.arctan2(axis[1], axis[0]))
        if len(rp) >= 5 and np.count_nonzero(level) >= 5 and empty["center_xy"] is None:
            # A cut image can still measure the roof, without measuring both jambs.
            if roof.sum() > empty.get("roof_pixels", 0):
                empty.update(
                    heading=heading,
                    underside_m=height,
                    roof_pixels=int(roof.sum()),
                    confidence=0.35 if height is not None else 0.2,
                    missing=["both_opening_edges"]
                    + ([] if floor is not None else ["floor"]),
                )
        if len(pp) < 4:
            continue
        face_f = pp @ axis
        # Front and rear posts can both be visible. Keep the foremost supported plane.
        face_f.sort()
        groups = np.split(face_f, np.flatnonzero(np.diff(face_f) > 0.045) + 1)
        planes = [q for q in groups if len(q) >= 4]
        if not planes:
            continue
        front = float(np.median(planes[0]))
        post_pts = points[post, :2]
        pf = post_pts @ axis
        post_y = np.sort(
            post_pts[(pf >= front - 0.035) & (pf <= front + 0.98)] @ lateral
        )
        if len(post_y) < 6:
            continue
        jambs = [
            q
            for q in np.split(post_y, np.flatnonzero(np.diff(post_y) > 0.13) + 1)
            if len(q) >= 3
        ]
        gaps = [
            (l[-1], h[0])
            for l, h in zip(jambs, jambs[1:])
            if 0.80 < h[0] - l[-1] < 1.27
        ]
        if not gaps:
            continue
        inner_a, inner_b = max(gaps, key=lambda v: v[1] - v[0])
        center = front * axis + 0.5 * (inner_a + inner_b) * lateral
        width = float(inner_b - inner_a)
        confidence = (
            float(
                min(
                    1.0,
                    0.35
                    + 0.25 * min(1.0, len(pp) / 30)
                    + 0.20 * min(1.0, len(rp) / 12)
                    + 0.20 * min(1.0, np.count_nonzero(level) / 12),
                )
            )
            if height is not None
            else 0.4
        )
        # Near-horizontal views can hide the far edge of the shelf. Use the distal
        # side of a rear post only when we also measured its front face and side span.
        rear_planes = [q for q in planes[1:] if np.median(q) - front > 0.30]
        depth = None
        if rear_planes:
            rear_face = float(np.median(rear_planes[-1]))
            rear_side = pf[(pf > rear_face + 0.04) & (pf < rear_face + 0.19)]
            if len(rear_side) >= 3:
                depth = float(np.percentile(rear_side, 99.5) - front)
        visible_depth = float(max((roof_points[:, :2] @ axis).max(), pf.max()) - front)
        result = dict(
            visible=height is not None,
            center_xy=center.tolist(),
            heading=heading,
            width_m=width,
            underside_m=height,
            confidence=confidence,
            missing=[] if height is not None else ["floor"],
            floor_z_local=floor,
            roof_pixels=int(roof.sum()),
            post_face_patches=len(pp),
            roof_face_patches=len(rp),
            underside_patches=int(np.count_nonzero(level)),
            edge_resolution_m=pixel_m,
            depth_m=depth,
            observed_depth_extent_m=visible_depth,
            back_xy=(center + depth * axis).tolist() if depth is not None else None,
        )
        ranking = confidence
        if prior is not None and prior.get("center_xy") is not None:
            shift = np.linalg.norm(center - np.asarray(prior["center_xy"]))
            if shift > 0.38:
                # A rear post seen after the entrance passes out of frame cannot be
                # used as a new front measurement for the same static rack.
                continue
            ranking -= 0.35 * shift
        candidates.append((ranking, result))
    return max(candidates, key=lambda c: c[0])[1] if candidates else empty
