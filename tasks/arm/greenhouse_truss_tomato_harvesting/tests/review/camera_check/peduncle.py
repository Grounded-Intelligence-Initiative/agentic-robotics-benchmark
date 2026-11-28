#!/usr/bin/env python3
"""The camera-only check, step 2 (maintainers' evidence; nothing runs it at grading; CPU only): every captured ready
truss's peduncle located from wrist colour frames alone (no depth, no truth): the identified camera mounting
(calibrate.py), the robot's joint readings, a line fitted in every view to the thin green stalk that leaves the main
stem, the line most views agree on. Compared with the truth saved beside the frames.

    python3 peduncle.py <capture dir> [--debug <folder for pictures>]
"""
import glob, os, pickle, sys, base64, math
import numpy as np, cv2
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from rowcell import farm as F, kin
D = sys.argv[1]
DEBUG = sys.argv[sys.argv.index("--debug") + 1] if "--debug" in sys.argv else None
meta = pickle.load(open(D + "/meta.pkl", "rb")); K = np.array(meta["K"], float); Kinv = np.linalg.inv(K)
mount = pickle.load(open(D + "/calib.pkl", "rb"))["mount_est"]
AIM = (640.0, 530.0)                 # where the capture put the truss in the picture (a coarse aim, +-150 px)

def rgb_of(rec): return cv2.imdecode(np.frombuffer(base64.b64decode(rec["rgb"]), np.uint8), cv2.IMREAD_COLOR)

def detect(img, tag=None):
    """Every thin green stalk that leaves the main stem downward near the aim: [(junction pixel, unit direction in
    the picture)], nearest the aim first. A view may show none (hidden behind a fruit) or several (a petiole's
    start, a rachis): the views are reconciled in space, below."""
    b, g, r = [img[..., k].astype(np.float32) for k in range(3)]
    # the light's colour from the backdrop: the picture's most common bright colour
    bright = (r + g + b) > 240
    gain = np.array([np.median(r[bright]), np.median(g[bright]), np.median(b[bright])]) + 1e-3
    rn, gn, bn = r / gain[0], g / gain[1], b / gain[2]
    green = ((gn > 1.12 * rn) & (gn > 1.12 * bn) & ((r + g + b) > 60)).astype(np.uint8)
    green = cv2.morphologyEx(green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    dt = cv2.distanceTransform(green, cv2.DIST_L2, 5)
    # the main stem: thick (a half-width over 9 px at this distance) and long from top to bottom; a leaf's blade is
    # thick and short. Of those, the one nearest the aim.
    core = (dt > 9.0).astype(np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(core)
    best, bd = None, 1e9
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_HEIGHT] < 140 or stats[i, cv2.CC_STAT_WIDTH] > 0.6 * stats[i, cv2.CC_STAT_HEIGHT]:
            continue
        ys, xs = np.nonzero(lab == i)
        d = np.min(np.abs(xs - AIM[0]) + 0.2 * np.abs(ys - AIM[1]))
        if d < bd: best, bd = i, d
    if best is None or bd > 200: return []
    sy, sx = np.nonzero(lab == best)
    svx, svy, sx0, sy0 = cv2.fitLine(np.stack([sx, sy], axis=1).astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
    half = float(np.median(dt[lab == best])) + 6.0                      # the stem's half-width in the picture, about
    yy, xx = np.mgrid[0:img.shape[0], 0:img.shape[1]]
    off = (xx - sx0) * svy - (yy - sy0) * svx                            # signed distance from the stem's axis
    band = (np.abs(off) <= half + 4.0)
    thin = green & (~band).astype(np.uint8) & (dt <= 8.0).astype(np.uint8)
    n2, lab2, stats2, _ = cv2.connectedComponentsWithStats(thin)
    ring = (np.abs(off) <= half + 9.0) & ~band
    out = []
    for i in range(1, n2):
        if stats2[i, cv2.CC_STAT_AREA] < 120: continue
        comp = lab2 == i
        touch = comp & ring
        if not touch.any(): continue
        ty, tx = np.nonzero(touch); j = np.array([tx.mean(), ty.mean()])
        if abs(j[0] - AIM[0]) > 200 or abs(j[1] - AIM[1]) > 260: continue
        ys, xs = np.nonzero(comp)
        dist = np.hypot(xs - j[0], ys - j[1])
        near = (dist > 6) & (dist < 80)                                  # the stalk's first stretch from the stem
        if near.sum() < 50: continue
        pts = np.stack([xs[near], ys[near]], axis=1).astype(np.float32)
        vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
        dvec = np.array([vx, vy]); c = pts.mean(axis=0)
        if np.dot(c - j, dvec) < 0: dvec = -dvec
        if dvec[1] < 0.2: continue                                       # a peduncle goes down from the stem; a petiole goes up
        res = np.abs((pts[:, 0] - x0) * vy - (pts[:, 1] - y0) * vx)
        if np.median(res) > 4.0: continue                                # not a straight stalk
        # the junction: where the stalk's line meets the stem's axis
        A = np.array([[dvec[0], -svx], [dvec[1], -svy]]); rhs = np.array([sx0 - x0, sy0 - y0])
        try:
            s, _u = np.linalg.solve(A, rhs); jx = np.array([x0, y0]) + s * dvec
        except np.linalg.LinAlgError:
            continue
        if np.hypot(*(jx - j)) > 60: continue                            # nearly parallel to the stem: no junction from this view
        out.append((float(np.hypot(j[0] - AIM[0], j[1] - AIM[1])), jx, dvec))
    out.sort(key=lambda c: c[0])
    if DEBUG and tag:
        vis = img.copy(); vis[lab == best] = (0, 80, 255); vis[lab2 > 0] = (255, 255, 0)
        for _, jx, dvec in out:
            cv2.line(vis, tuple(int(v) for v in jx), tuple(int(v) for v in jx + 150 * dvec), (255, 0, 255), 2)
        cv2.imwrite(os.path.join(DEBUG, "det_%s.png" % tag), vis)
    return [(jx, dvec) for _, jx, dvec in out[:3]]


def detect_ring(img, tag=None):
    """The same question asked another way (the stem as the thick part grown back to its outline, the nearest stalk
    that touches it): [(junction pixel, unit direction)] with at most one entry. It sees stalks that run close along
    the stem, which the band of `detect` hides; `detect` sees several stalks a view."""
    b, g, r = [img[..., k].astype(np.float32) for k in range(3)]
    # the light's colour from the backdrop: the picture's most common bright colour
    bright = (r + g + b) > 240
    gain = np.array([np.median(r[bright]), np.median(g[bright]), np.median(b[bright])]) + 1e-3
    rn, gn, bn = r / gain[0], g / gain[1], b / gain[2]
    lum = (rn + gn + bn) / 3.0
    green = (gn > 1.12 * rn) & (gn > 1.12 * bn) & ((r + g + b) > 60) & (lum > 0.26)     # stalks are lighter than leaf blades
    green = cv2.morphologyEx(green.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    dt = cv2.distanceTransform(green, cv2.DIST_L2, 5)
    # the main stem: the thick part (half-width over 9 px at this distance), grown back to its outline
    core = (dt > 9.0).astype(np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(core)
    if n < 2: return []
    # the stem nearest the aim
    best, bd = None, 1e9
    for i in range(1, n):
        ys, xs = np.nonzero(lab == i)
        if len(xs) < 150: continue
        d = np.min(np.hypot(xs - AIM[0], ys - AIM[1]))
        if d < bd: best, bd = i, d
    if best is None: return []
    stem = cv2.dilate((lab == best).astype(np.uint8), np.ones((25, 25), np.uint8)) & green
    thin = green & (1 - cv2.dilate(stem, np.ones((5, 5), np.uint8))) & (dt <= 9.0).astype(np.uint8)
    n2, lab2, stats2, _ = cv2.connectedComponentsWithStats(thin)
    ring = cv2.dilate(stem, np.ones((9, 9), np.uint8))
    cands = []
    for i in range(1, n2):
        if stats2[i, cv2.CC_STAT_AREA] < 150: continue
        comp = lab2 == i
        touch = comp & (ring > 0)
        if not touch.any(): continue
        ty, tx = np.nonzero(touch); j = np.array([tx.mean(), ty.mean()])
        if np.hypot(j[0] - AIM[0], j[1] - AIM[1]) > 260: continue
        ys, xs = np.nonzero(comp)
        dist = np.hypot(xs - j[0], ys - j[1])
        near = (dist > 12) & (dist < 85)            # the stalk's first stretch from the stem
        if near.sum() < 60: continue
        pts = np.stack([xs[near], ys[near]], axis=1).astype(np.float32)
        vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
        dvec = np.array([vx, vy]); c = pts.mean(axis=0)
        if np.dot(c - j, dvec) < 0: dvec = -dvec
        res = np.abs((pts[:, 0] - x0) * vy - (pts[:, 1] - y0) * vx)
        if dvec[1] < 0.2: continue                   # a peduncle goes down from the stem; a petiole goes up
        cands.append((np.hypot(j[0] - AIM[0], j[1] - AIM[1]), j, dvec, np.array([x0, y0]), float(np.median(res)), int(near.sum())))
    if not cands: return []
    cands.sort(key=lambda c: c[0])
    _, j, dvec, p0, res, npx = cands[0]
    # the junction: where the stalk's line meets the stem's axis
    sy, sx = np.nonzero(lab == best)
    svx, svy, sx0, sy0 = cv2.fitLine(np.stack([sx, sy], axis=1).astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01).ravel()
    A = np.array([[dvec[0], -svx], [dvec[1], -svy]]); rhs = np.array([sx0 - p0[0], sy0 - p0[1]])
    try:
        t, _s = np.linalg.solve(A, rhs); jx = p0 + t * dvec
    except np.linalg.LinAlgError:
        jx = j
    return [(jx, dvec)]


def ray(Tc, px):
    d = Tc[:3, :3] @ (Kinv @ np.array([px[0], px[1], 1.0])); return Tc[:3, 3], d / np.linalg.norm(d)

rows = []
by = {}
for f in sorted(glob.glob(D + "/*_v*.pkl")):
    rec = pickle.load(open(f, "rb")); by.setdefault(rec["truss"], []).append((os.path.basename(f)[:-4], rec))
for tid, views in by.items():
    obs = []
    for vi, (tag, rec) in enumerate(views):
        Tc = kin.flange(rec["q"]) @ mount                      # the camera in the cart frame: joints and identified mounting
        img = rgb_of(rec)
        for jx, dvec in detect(img, tag) + detect_ring(img):
            o, dj = ray(Tc, jx); _, d2 = ray(Tc, jx + 100 * dvec)
            nrm = np.cross(dj, d2); nrm /= np.linalg.norm(nrm)     # the plane through the camera that holds the stalk's line
            obs.append((vi, o, dj, nrm, rec))
    tr = views[0][1]["trusses"][tid]; a, b = np.array(tr["peduncle"][0]), np.array(tr["peduncle"][1])
    nseen = len(set(v[0] for v in obs))
    if nseen < 3:
        rows.append((tid, len(views), nseen, None)); continue
    def solve(sub):
        """the junction (the point nearest the rays), the stalk's direction (across the planes' normals), a point on the line"""
        A = np.zeros((3, 3)); rhs = np.zeros(3)
        for _, o, dj, nrm, _r in sub:
            P = np.eye(3) - np.outer(dj, dj); A += P; rhs += P @ o
        J = np.linalg.solve(A, rhs)
        N = np.stack([nrm for _, _, _, nrm, _r in sub]); _, _, Vt = np.linalg.svd(N); d3 = Vt[-1]
        if d3[2] > 0: d3 = -d3
        bp = np.array([nrm @ o for _, o, _, nrm, _r in sub])
        X = np.linalg.lstsq(np.vstack([N, d3[None, :]]), np.append(bp, d3 @ J), rcond=None)[0]
        return J, d3, X
    def agrees(view, J, d3, X):
        """a view agrees with a line when its camera's plane holds it (within 2.5 mm at the line and 4 degrees) and its ray passes the junction within 6 mm"""
        _, o, dj, nrm, _r = view
        return abs(nrm @ (X - o)) < 0.0025 and abs(math.degrees(math.asin(min(1.0, abs(nrm @ d3))))) < 4.0 and np.linalg.norm(np.cross(J - o, dj)) < 0.006
    # the views do not all show the same stalk (a leaf's edge, a petiole): the line most views agree on (consensus over
    # every pair of views), refitted on the views that agree
    best = None
    for i in range(len(obs)):
        for k in range(i + 1, len(obs)):
            if abs(obs[i][3] @ obs[k][3]) > 0.995:
                continue                                  # two nearly parallel planes do not fix a line
            J, d3, X = solve([obs[i], obs[k]])
            if obs[i][0] == obs[k][0] or np.linalg.norm(J - 0.5 * (obs[i][1] + obs[k][1])) > 0.60 or d3[2] > -0.2:
                continue                                  # one view twice; a line far from the cameras; a stalk that does not go down
            inl, seen = [], set()
            for v in obs:                                 # one stalk a view
                if v[0] not in seen and agrees(v, J, d3, X):
                    inl.append(v); seen.add(v[0])
            if best is None or len(inl) > len(best):
                best = inl
    if best is None or len(best) < 3:
        rows.append((tid, len(views), nseen, None)); continue
    J, dir3, X0 = solve(best)
    nd = len(best)
    # against the truth: the cut point of the stated approach, in the approach's own axes
    L = np.linalg.norm(b - a); lo, hi = 0.024, L - 0.010; s = min(hi, max(lo, 0.5 * (lo + hi)))
    pstar = a + s * (b - a) / L
    truss = {"peduncle": {"a": a, "b": b}}
    R = F.approach_axes(truss, 0.0)
    q = X0 + dir3 * (dir3 @ (pstar - X0))                      # the estimated line's point nearest the true cut point
    e = R.T @ (q - pstar)
    ang = math.degrees(math.acos(min(1.0, abs(dir3 @ (b - a) / L))))
    stem_r = views[0][1]["plants"][tid[0]]["stem_r"]
    rows.append((tid, len(views), nd, (e[1] * 1000, e[2] * 1000, ang, (np.linalg.norm(J - a) - stem_r) * 1000)))
for tid, nv, nd, r in rows:
    if r is None: print("%s: %d views, a stalk found in %d, fewer than 3 agree: no estimate" % (tid, nv, nd))
    else: print("%s: %d views, %d agree: the cut point off by %+.2f mm across the mouth, %+.2f mm along the approach; direction off %.1f deg; junction off %.1f mm" % ((tid, nv, nd) + r))
ok = [r for _, _, _, r in rows if r is not None]
if ok:
    ey, ez = np.array([r[0] for r in ok]), np.array([r[1] for r in ok])
    print("SUMMARY %s: %d of %d trusses; across |%.2f| median, |%.2f| max; along the approach |%.2f| median, |%.2f| max (mm)" % (
        os.path.basename(D), len(ok), len(rows), np.median(np.abs(ey)), np.abs(ey).max(), np.median(np.abs(ez)), np.abs(ez).max()))
