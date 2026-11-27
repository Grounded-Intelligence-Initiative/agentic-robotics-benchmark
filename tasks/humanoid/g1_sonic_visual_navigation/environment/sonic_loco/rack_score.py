"""Native geometry measurements for a G1 travelling below a finite shelf."""

import math

import mujoco
import numpy as np

from .racks import frame


class RobotProfile:
    """Support of actual G1 contact primitives and compiled convex mesh hulls."""

    def __init__(self, model, robot_ids):
        self.ids = np.asarray(sorted(robot_ids), dtype=int)
        self.types = model.geom_type[self.ids]
        self.sizes = model.geom_size[self.ids].copy()
        self.meshes = []
        for i, g in enumerate(self.ids):
            if self.types[i] == mujoco.mjtGeom.mjGEOM_MESH:
                mesh = model.geom_dataid[g]
                start = model.mesh_vertadr[mesh]
                verts = model.mesh_vert[start : start + model.mesh_vertnum[mesh]]
                graph = model.mesh_graphadr[mesh]
                if graph >= 0:
                    count = int(model.mesh_graph[graph])
                    verts = verts[model.mesh_graph[graph + 2 + count : graph + 2 + 2 * count]]
                hull = np.asarray(verts, dtype=np.float64)
                self.meshes.append((i, hull))
                if model.mesh(mesh).name == "head_link":
                    self.head_id = int(g)
                    self.head_mesh = hull
        if not hasattr(self, "head_id"):
            raise ValueError("released head_link collision mesh missing")
        self.primitive = {
            typ: np.where(self.types == typ)[0]
            for typ in set(self.types)
            if typ != mujoco.mjtGeom.mjGEOM_MESH
        }

    def head_top(self, data):
        g = self.head_id
        return float(
            data.geom_xpos[g, 2] + np.max(self.head_mesh @ data.geom_xmat[g].reshape(3, 3)[2, :])
        )

    def trailing_point(self, data, axis):
        orient = data.geom_xmat[self.ids].reshape(-1, 3, 3)
        coord = np.einsum("i,gij->gj", axis, orient)
        lower = data.geom_xpos[self.ids] @ axis
        shape = mujoco.mjtGeom
        for typ, idx in self.primitive.items():
            ray = coord[idx]
            sz = self.sizes[idx]
            if typ == shape.mjGEOM_SPHERE:
                ext = sz[:, 0]
            elif typ == shape.mjGEOM_BOX:
                ext = np.sum(np.abs(ray) * sz, axis=1)
            elif typ == shape.mjGEOM_ELLIPSOID:
                ext = np.linalg.norm(ray * sz, axis=1)
            elif typ in (shape.mjGEOM_CYLINDER, shape.mjGEOM_CAPSULE):
                circle = (
                    sz[:, 0]
                    if typ == shape.mjGEOM_CAPSULE
                    else sz[:, 0] * np.linalg.norm(ray[:, :2], axis=1)
                )
                ext = circle + sz[:, 1] * np.abs(ray[:, 2])
            else:
                raise ValueError("unexpected G1 collision primitive")
            lower[idx] -= ext
        for i, hull in self.meshes:
            lower[i] += np.min(hull @ coord[i])
        return float(np.min(lower))


class RackPassage:
    def __init__(self, session, branch):
        self.session = session
        self.rack = session.layout.racks.branch(branch)
        self.origin, self.axes = frame(
            session.layout.racks, branch, session.layout_origin_xy, session.layout_axes
        )
        self.axis3 = np.r_[self.axes[:, 0], 0.0]
        self.origin_on_axis = float(self.origin @ self.axes[:, 0])
        self.previous = self.axes.T @ (np.asarray(session.state["metrics"]["base_xy"]) - self.origin)
        self.time = float(session.data.time)
        self.entry = self.exit = self.cleared = self.head_under = None
        self.missed = self.boundary = False
        self.roof_min = None
        self.max_head_in_span = None
        self._decks = [
            session.model.geom("rack_" + branch + "_deck_" + part).id for part in ("front", "back")
        ]
        self._ids = session.profile.ids
        self._radii = session.model.geom_rbound[self._ids]
        self._lo, self._hi = self.rack.edges

    def record(self, data, time, xy, head_top):
        f = self.rack
        x, y = self.axes.T @ (xy - self.origin)
        px, py = self.previous
        self.boundary |= abs(y) > 1.58
        for key, plane in [("entry", f.front_m), ("exit", f.back_m)]:
            if getattr(self, key) is None and px < plane <= x:
                u = (plane - px) / (x - px)
                cy = float(py + u * (y - py))
                t = float(self.time + u * (time - self.time))
                clearance = min(cy - self._lo, self._hi - cy)
                event = dict(
                    tick=math.ceil(t / 0.02 - 1e-8),
                    native_time_s=t,
                    forward_m=plane,
                    left_m=cy,
                    gap_edge_clearance_m=clearance,
                )
                setattr(self, key, event)
                self.missed |= clearance <= 0 or (key == "exit" and self.entry is None)
        if f.front_m <= x <= f.back_m:
            self.max_head_in_span = (
                head_top if self.max_head_in_span is None else max(self.max_head_in_span, head_top)
            )
        # A genuine underpass has a whole head inside the solid shelf footprint:
        # neither its bounding center nor a single posture/joint value establishes this.
        if (
            self.head_under is None
            and f.front_m - 0.35 <= x <= f.back_m + 0.35
            and head_top <= f.underside_m
        ):
            profile = self.session.profile
            g = profile.head_id
            head = profile.head_mesh @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
            local = (head[:, :2] - self.origin) @ self.axes
            xmin, ymin = local.min(axis=0)
            xmax, ymax = local.max(axis=0)
            if (
                xmin >= f.front_m
                and xmax <= f.back_m
                and ymin >= f.left_m - 0.69
                and ymax <= f.left_m + 0.69
            ):
                self.head_under = dict(
                    tick=math.ceil(time / 0.02 - 1e-8),
                    native_time_s=float(time),
                    head_top_m=head_top,
                    underside_m=f.underside_m,
                    head_clearance_m=f.underside_m - head_top,
                    head_forward_min_m=float(xmin),
                    head_forward_max_m=float(xmax),
                    head_left_min_m=float(ymin),
                    head_left_max_m=float(ymax),
                )
        # Use native convex shape distances for robot parts within 0.3m of the roof.
        # Even before/after the pelvis crosses, a leading hand or a trailing foot can still be inside.
        if f.front_m - 0.75 <= x <= f.back_m + 0.80 and self.cleared is None:
            threshold = 0.30 if self.roof_min is None else max(1e-6, self.roof_min)
            centers = data.geom_xpos[self._ids]
            for deck in self._decks:
                rel = (centers - data.geom_xpos[deck]) @ data.geom_xmat[deck].reshape(3, 3)
                delta = np.maximum(np.abs(rel) - self.session.model.geom_size[deck], 0.0)
                candidates = np.sum(delta * delta, axis=1) <= (self._radii + threshold) ** 2
                for g in self._ids[candidates]:
                    dist = mujoco.mj_geomDistance(
                        self.session.model, data, deck, int(g), threshold, None
                    )
                    if dist < threshold and (self.roof_min is None or dist < self.roof_min):
                        self.roof_min = float(dist)
                        threshold = max(dist, 1e-6)
        if self.exit is not None and self.cleared is None and x > f.back_m:
            trailing = self.session.profile.trailing_point(data, self.axis3) - self.origin_on_axis
            if trailing > f.back_m:
                self.cleared = dict(
                    tick=math.ceil(time / 0.02 - 1e-8),
                    native_time_s=float(time),
                    trailing_body_forward_m=trailing,
                    back_m=f.back_m,
                )
        self.previous = (x, y)
        self.time = time

    def summary(self):
        return dict(
            branch=self.rack.branch,
            entry=self.entry,
            exit=self.exit,
            whole_body_clear=self.cleared,
            head_under_shelf=self.head_under,
            missed=self.missed,
            boundary_violation=bool(self.boundary),
            min_roof_distance_below_0p30_m=self.roof_min,
            max_head_height_between_planes_m=self.max_head_in_span,
        )
