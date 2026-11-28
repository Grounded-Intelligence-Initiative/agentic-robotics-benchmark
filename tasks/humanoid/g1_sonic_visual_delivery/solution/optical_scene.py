"""Fit the orange top face and pad from actual public RGB-D surface normals."""

import numpy as np


class SceneTracker:
    """Reconstruct the workpiece center, upright axis and illuminated pad."""

    def __init__(self):
        self.half_height = None
        self.pad_measurements = []
        self.object_center = None
        self.goal = None
        self.up = np.array([0.0, 0.0, 1.0])

    def observe(self, views, body):
        all_obj = []
        tops = []
        norms = []
        pads = []
        for key, (pos, axes) in body.calibrations.items():
            view = views[key[0]]
            dep = view["depth_mm"]
            r, g, b = view["rgb"].astype(np.int32).transpose(2, 0, 1)
            orange = (
                (r > 23) & (g * 10 < 8 * r) & (g * 10 > 3 * r) & (b * 4 < r) & (dep > 0)
            )
            pad = (g > 23) & (r * 2 < g) & (b * 10 < g * 9) & (b * 2 > g) & (dep > 0)
            rays = body.rays[key]
            for label, mask in ((0, orange), (1, pad)):
                y, x = np.where(mask)
                if not len(y):
                    continue
                xyz = pos + (rays[y, x] @ axes) * (dep[y, x, None] * 0.001)
                keep = (
                    (xyz[:, 0] > 0.12)
                    & (xyz[:, 0] < 0.58)
                    & (xyz[:, 1] > -0.59)
                    & (xyz[:, 1] < 0.11)
                    & (xyz[:, 2] > 0.725)
                    & (xyz[:, 2] < 1.2)
                )
                if label:
                    pts = xyz[keep & (np.abs(xyz[:, 2] - 0.745) < 0.007)]
                    if len(pts):
                        pads.append(pts)
                    continue
                if not keep.any():
                    continue
                all_obj.append(xyz[keep])
                # Use horizontal and vertical secants on the same colored face.
                h, w = dep.shape
                keep &= (x >= 2) & (y >= 2) & (x < w - 2) & (y < h - 2)
                y, x = y[keep], x[keep]
                pts = xyz[keep]
                if not len(y):
                    continue
                valid = (
                    orange[y - 2, x]
                    & orange[y + 2, x]
                    & orange[y, x - 2]
                    & orange[y, x + 2]
                )
                y, x, pts = y[valid], x[valid], pts[valid]
                if not len(y):
                    continue

                def at(v, u):
                    return (rays[v, u] @ axes) * (dep[v, u, None] * 0.001)

                normal = np.cross(
                    at(y, x + 2) - at(y, x - 2), at(y + 2, x) - at(y - 2, x)
                )
                normal /= np.maximum(
                    np.linalg.norm(normal, axis=1, keepdims=True), 1e-8
                )
                normal *= np.sign(normal[:, 2:3])
                ok = normal[:, 2] > 0.70
                if ok.any():
                    tops.append(pts[ok])
                    norms.append(normal[ok])
        if all_obj:
            pts = np.concatenate(all_obj)
            lo, hi = np.quantile(pts, [0.015, 0.985], axis=0)
            if self.half_height is None:
                self.half_height = 0.060 if hi[2] > 0.841 else 0.050
            candidate = np.r_[(lo[:2] + hi[:2]) / 2, hi[2] - self.half_height]
            if tops:
                t = np.concatenate(tops)
                n = np.concatenate(norms)
                best = t[:, 2] > hi[2] - 0.025
                if best.sum() > 8:
                    up = np.median(n[best], axis=0)
                    up /= np.linalg.norm(up)
                    plane = np.median(t[best] @ up)
                    best &= abs(t @ up - plane) < 0.004
                    if best.sum() > 6:
                        top = t[best]
                        l, h = np.quantile(top, [0.01, 0.99], axis=0)
                        core = (l + h) * 0.5
                        core += up * (plane - up @ core)
                        estimate = core - self.half_height * up
                        # Occluded top edges give a less reliable horizontal midpoint.
                        weight = np.clip(np.min(h[:2] - l[:2]) / 0.044, 0.0, 1.0)
                        candidate = estimate * weight + candidate * (1.0 - weight)
                        self.up = up
            self.object_center = candidate
        if pads:
            pts = np.concatenate(pads)
            lo, hi = np.quantile(pts, [0.001, 0.999], axis=0)
            center = 0.5 * (lo + hi)
            quality = np.prod(np.minimum(0.138, hi[:2] - lo[:2]))
            for j in (0, 1):
                if hi[j] - lo[j] < 0.132:
                    left = sum(pts[:, j] < lo[j] + 0.003)
                    right = sum(pts[:, j] > hi[j] - 0.003)
                    center[j] = lo[j] + 0.069 if left > right else hi[j] - 0.069
            # Keep the best observed extent for each physically distinct pad when
            # the fingers obscure an edge. The color mask selects the live pad.
            nearby = next(
                (
                    i
                    for i, (_, old) in enumerate(self.pad_measurements)
                    if np.linalg.norm(old[:2] - center[:2]) < 0.09
                ),
                None,
            )
            if nearby is None:
                nearby = len(self.pad_measurements)
                self.pad_measurements.append((quality, center.copy()))
            elif quality > self.pad_measurements[nearby][0]:
                self.pad_measurements[nearby] = (quality, center.copy())
            measured = self.pad_measurements[nearby][1]
            self.goal = np.r_[measured[:2], measured[2] + (self.half_height or 0.055)]
        return self.object_center, self.goal
