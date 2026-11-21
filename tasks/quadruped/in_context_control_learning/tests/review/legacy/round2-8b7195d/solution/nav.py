"""Grid planning and a press-level controller for the delivery robot, shared by the oracle and the image-only
reference. Nothing here reads ground truth: the caller supplies the pose, the obstacle grid and the step sizes it has.

The bay is an 8 m square of 5 cm cells. `distance_field` is the travel distance to the goal from every free cell
(Dijkstra over the 8-connected grid), `next_press` a short A* over press sequences (forward, turn left, turn right)
with that field as its heuristic; it returns the first press of the best sequence, and the caller plans again after
every press.
"""
import heapq
import math

import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

EXTENT = 4.0
CELL = .05
N = int(round(2*EXTENT/CELL))
ROBOT_RADIUS = .16        # from the instruction
FENCE = 3.55              # from the instruction


def cell(x, y):
    return (min(N-1, max(0, int((y+EXTENT)/CELL))), min(N-1, max(0, int((x+EXTENT)/CELL))))


def centres():
    """(X, Y) arrays of the cell centres, indexed [row (y), column (x)]."""
    c = (np.arange(N)+.5)*CELL-EXTENT
    return np.meshgrid(c, c)


def _graph():
    index = np.arange(N*N).reshape(N, N)
    src, dst, length = [], [], []
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if not (dy or dx):
                continue
            a = index[max(0, -dy):N-max(0, dy), max(0, -dx):N-max(0, dx)]
            b = index[max(0, dy):N-max(0, -dy), max(0, dx):N-max(0, -dx)]
            src.append(a.ravel())
            dst.append(b.ravel())
            length.append(np.full(a.size, CELL*math.hypot(dy, dx)))
    src, dst, length = (np.concatenate(v) for v in (src, dst, length))
    order = np.argsort(src, kind="stable")
    src, dst, length = src[order], dst[order], length[order]
    indptr = np.searchsorted(src, np.arange(N*N+1))
    return src, dst, length, indptr


_SRC, _DST, _LEN, _INDPTR = _graph()


def distance_field(blocked, goal, cost=None):
    """Travel distance (m) to `goal` from every cell; inf where no route exists. `cost` scales the price of
    entering a cell (1 everywhere by default)."""
    blocked = blocked.copy()
    gy, gx = cell(*goal)
    blocked[max(0, gy-2):gy+3, max(0, gx-2):gx+3] = False
    flat = blocked.ravel()
    weight = _LEN if cost is None else _LEN*cost.ravel()[_DST]
    keep = ~(flat[_SRC] | flat[_DST])
    indptr = np.concatenate([[0], np.cumsum(np.bincount(_SRC[keep], minlength=N*N))])
    graph = csr_matrix((weight[keep], _DST[keep], indptr), shape=(N*N, N*N))
    # The graph is symmetric in its lengths; with a cell cost it is searched from the goal over reversed edges.
    dist = dijkstra(graph.T.tocsr() if cost is not None else graph, directed=True, indices=gy*N+gx)
    return dist.reshape(N, N)


def inflate(occupied, radius):
    """Cells whose centre is within `radius` of an occupied cell."""
    return ndimage.distance_transform_edt(~occupied)*CELL < radius


def rasterize(boxes, radius):
    """Blocked cells for a robot of footprint `radius` among axis-aligned boxes (cx, cy, half x, half y)."""
    X, Y = centres()
    blocked = np.zeros((N, N), bool)
    for cx, cy, hx, hy in boxes:
        dx = np.maximum(np.abs(X-cx)-hx, 0)
        dy = np.maximum(np.abs(Y-cy)-hy, 0)
        blocked |= np.hypot(dx, dy) < radius
    return blocked


def wrap(angle):
    return (angle+math.pi) % (2*math.pi)-math.pi


def _clear(blocked, x0, y0, x1, y1):
    n = max(2, int(math.hypot(x1-x0, y1-y0)/(CELL*.8))+1)
    for k in range(1, n+1):
        f = k/n
        if blocked[cell(x0+(x1-x0)*f, y0+(y1-y0)*f)]:
            return False
    return True


def next_press(pose, dist, blocked, steps, goal, reach=.2, budget=5000):
    """The first press ("F", "L" or "R") of a short best sequence from `pose` = (x, y, heading in rad) toward the
    goal. `steps` = {"F": metres per forward press, "L": radians per left press, "R": radians per right press}."""
    x, y, th = pose
    forward = max(steps["F"], .02)
    left, right = max(steps["L"], .02), max(steps["R"], .02)
    turn = max(left, right)
    finite = float(np.nanmax(dist[np.isfinite(dist)])) if np.isfinite(dist).any() else 0.

    def h(px, py, pth):
        d = dist[cell(px, py)]
        if not np.isfinite(d):
            d = finite+5.
        return d/forward

    if blocked[cell(x, y)]:
        return _escape(pose, blocked, steps)
    key0 = (round(x/.04), round(y/.04), round(math.degrees(th)/4) % 90)
    frontier = [(h(x, y, th), 0, 0, x, y, th, None)]
    seen = {key0: 0}
    best = (h(x, y, th), None)
    count = 0
    expansions = 0
    while frontier and expansions < budget:
        f, g, _, px, py, pth, first = heapq.heappop(frontier)
        if math.hypot(px-goal[0], py-goal[1]) < reach:
            return first
        expansions += 1
        if first is not None and f-g < best[0]:
            best = (f-g, first)
        for press in ("F", "L", "R"):
            if press == "F":
                nx, ny, nth = px+forward*math.cos(pth), py+forward*math.sin(pth), pth
                if not _clear(blocked, px, py, nx, ny):
                    continue
            else:
                nx, ny, nth = px, py, pth+(left if press == "L" else -right)
            key = (round(nx/.04), round(ny/.04), round(math.degrees(nth)/4) % 90)
            if seen.get(key, 1 << 30) <= g+1:
                continue
            seen[key] = g+1
            count += 1
            heapq.heappush(frontier, (g+1+h(nx, ny, nth), g+1, count, nx, ny, nth, press if first is None else first))
    if best[1] is not None:
        return best[1]
    return _escape(pose, blocked, steps)


def _escape(pose, blocked, steps):
    """Inside the planning margin (or boxed in by it): head for more clearance."""
    x, y, th = pose
    if blocked[cell(x, y)]:
        depth = ndimage.distance_transform_edt(blocked)          # cells to the nearest free one: to be reduced
    else:
        depth = -ndimage.distance_transform_edt(~blocked)        # minus the room around: to be raised
    best, choice = None, "L"
    for press, turn in (("F", 0.), ("L", steps["L"]), ("R", -steps["R"])):
        # A turn is scored by where a following forward press would land, at the price of half a cell.
        nx, ny = x+steps["F"]*math.cos(th+turn), y+steps["F"]*math.sin(th+turn)
        score = depth[cell(nx, ny)]+(.5 if press != "F" else 0.)
        if best is None or score < best:
            best, choice = score, press
    return choice
