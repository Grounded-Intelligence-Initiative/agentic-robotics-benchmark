"""Routes on the scout's map (grids in the map frame, rows = y descending).

Two readings of the same map:
  certain     a cell may hold the carrier's centre only if every cell within the carrier's radius was seen as
              floor: what the carrier is allowed to drive;
  optimistic  only what was seen occupied blocks: where the scout should go and look.
"""
import math

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

SQRT2 = math.sqrt(2)


def geodesic(passable, source, res, toll=None):
    """Cost of the cheapest 8-connected route from every passable cell to the `source` cells (inf where none)."""
    n, m = passable.shape
    index = np.arange(n*m).reshape(n, m)
    rows, cols, weights = [], [], []
    for dr, dc, length in ((0, 1, 1.), (1, 0, 1.), (1, 1, SQRT2), (1, -1, SQRT2)):
        a = (slice(0, n-dr), slice(max(0, -dc), m-max(0, dc)))
        b = (slice(dr, n), slice(max(0, dc), m+min(0, dc)))
        ok = passable[a] & passable[b]
        w = np.full(int(ok.sum()), length*res) if toll is None else length*res*.5*(toll[a][ok]+toll[b][ok])
        rows += [index[a][ok], index[b][ok]]
        cols += [index[b][ok], index[a][ok]]
        weights += [w, w]
    src = np.flatnonzero(source & passable)
    if not len(src):
        return np.full((n, m), np.inf)
    rows.append(np.full(len(src), n*m))
    cols.append(src)
    weights.append(np.full(len(src), 1e-9))
    graph = coo_matrix((np.concatenate(weights), (np.concatenate(rows), np.concatenate(cols))),
                       shape=(n*m+1, n*m+1)).tocsr()
    return dijkstra(graph, directed=True, indices=n*m)[:n*m].reshape(n, m)


FOOT = .07                       # a cell this close to something solid needs no floor sighting of its own


class Grid:
    """The map on the planning grid (43.75 mm), with two distances measured on a grid twice as fine:
    `gap`, metres to the nearest cell known to be occupied; `room`, metres to the nearest cell that is neither
    known floor nor explained by an obstacle's foot right beside it (never more than `gap`)."""

    def __init__(self, mosaic, scale=4):
        fine = scale//2
        free, solid, seen, self.fine_origin, self.fine_res = mosaic.grids(fine, align=scale)
        self.fine_gap = (ndimage.distance_transform_edt(~solid)*self.fine_res if solid.any()
                         else np.full(free.shape, 1e3))
        unexplained = ~free & (self.fine_gap > FOOT)
        self.fine_room = np.minimum(ndimage.distance_transform_edt(~unexplained)*self.fine_res, self.fine_gap)
        n, m = free.shape[0]//2, free.shape[1]//2

        def blocks(a):
            return a.reshape(n, 2, m, 2)
        self.free = blocks(free).all(axis=(1, 3))
        self.solid = blocks(solid).any(axis=(1, 3))
        self.seen = blocks(seen).mean(axis=(1, 3)) >= .5
        self.room = blocks(self.fine_room).min(axis=(1, 3))
        self.gap = blocks(self.fine_gap).min(axis=(1, 3))
        self.shape = self.free.shape
        self.res = self.fine_res*2
        self.origin = ((self.fine_origin[0]+.5)/2-.5, (self.fine_origin[1]+.5)/2-.5)
        cc, rr = np.meshgrid(np.arange(m), np.arange(n))
        self.X, self.Y = (cc-self.origin[1])*self.res, (self.origin[0]-rr)*self.res

    def fine_cells(self, x, y):
        r = np.clip(np.rint(self.fine_origin[0]-np.asarray(y)/self.fine_res).astype(int), 0, self.fine_room.shape[0]-1)
        c = np.clip(np.rint(self.fine_origin[1]+np.asarray(x)/self.fine_res).astype(int), 0, self.fine_room.shape[1]-1)
        return r, c

    def cell(self, p):
        r = int(round(self.origin[0]-p[1]/self.res))
        c = int(round(self.origin[1]+p[0]/self.res))
        return min(self.shape[0]-1, max(0, r)), min(self.shape[1]-1, max(0, c))

    def cells(self, x, y):
        r = np.clip(np.rint(self.origin[0]-np.asarray(y)/self.res).astype(int), 0, self.shape[0]-1)
        c = np.clip(np.rint(self.origin[1]+np.asarray(x)/self.res).astype(int), 0, self.shape[1]-1)
        return r, c

    def point(self, r, c):
        return np.array([(c-self.origin[1])*self.res, (self.origin[0]-r)*self.res])

    def disc(self, p, radius):
        return (self.X-p[0])**2+(self.Y-p[1])**2 <= radius**2

    def connected(self, passable, a, b):
        """Whether cells `a` and `b` lie in one 8-connected piece of `passable`."""
        if not passable[a] or not passable[b]:
            return False
        labels, _ = ndimage.label(passable, structure=np.ones((3, 3), bool))
        return labels[a] == labels[b]

    def descend(self, field, p, length):
        """The points of the route from `p` down `field`, up to `length` metres of it."""
        r, c = self.cell(p)
        if not np.isfinite(field[r, c]):
            k = int(.5/self.res)
            r0, c0 = max(0, r-k), max(0, c-k)
            rows, cols = np.nonzero(np.isfinite(field[r0:r+k+1, c0:c+k+1]))
            if not len(rows):
                return [np.asarray(p, float)]
            j = int(np.argmin((rows+r0-r)**2+(cols+c0-c)**2))
            r, c = r0+int(rows[j]), c0+int(cols[j])
        out, walked = [self.point(r, c)], 0.
        while walked < length and field[r, c] > 0:
            best = None
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r+dr, c+dc
                    inside = 0 <= rr < self.shape[0] and 0 <= cc < self.shape[1]
                    if (dr or dc) and inside and field[rr, cc] < field[r, c] and (best is None or field[rr, cc] < best[0]):
                        best = (field[rr, cc], rr, cc, self.res*math.hypot(dr, dc))
            if best is None:
                break
            _, r, c, step = best
            walked += step
            out.append(self.point(r, c))
        return out
