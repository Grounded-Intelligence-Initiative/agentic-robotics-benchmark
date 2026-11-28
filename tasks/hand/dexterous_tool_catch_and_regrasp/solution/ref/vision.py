import json, os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


class Cam(object):
    def __init__(self, pos, look_at, fovy_deg, W=192, H=144):
        self.pos = np.array(pos, float)
        f = np.array(look_at, float) - self.pos
        f /= np.linalg.norm(f)
        r = np.cross(f, [0, 0, 1.0]); r /= np.linalg.norm(r)
        u = np.cross(r, f)
        # camera frame: x right, y up, z back
        self.R = np.stack([r, u, -f])  # rows: world->cam
        self.f = (H / 2.0) / np.tan(np.radians(fovy_deg) / 2)
        self.cx, self.cy = W / 2.0, H / 2.0
        self.W, self.H = W, H

    def project(self, p):
        p = np.atleast_2d(p)
        c = (p - self.pos) @ self.R.T
        z = -c[:, 2]
        u = self.cx + self.f * c[:, 0] / z
        v = self.cy - self.f * c[:, 1] / z
        return np.stack([u, v], 1)

    def ray(self, u, v):
        d = np.array([(u - self.cx) / self.f, -(v - self.cy) / self.f, -1.0])
        d = self.R.T @ d
        return self.pos, d / np.linalg.norm(d)


def load_cams(path=None):
    path = path or os.path.join(HERE, "cameras.json")
    c = json.load(open(path))["cameras"]
    return {k: Cam(v["pos"], v["look_at"], v["fovy_deg"]) for k, v in c.items()}


def triangulate(rays):
    A = np.zeros((3, 3)); b = np.zeros(3)
    for o, d in rays:
        P = np.eye(3) - np.outer(d, d)
        A += P; b += P @ o
    return np.linalg.solve(A, b)


# handle colours (RGB 0..1 before shading); classify by hue-ish features
HANDLE_RGB = {
    "hammer": (0.62, 0.45, 0.28),
    "screwdriver": (0.55, 0.16, 0.14),
    "wrench": (0.20, 0.30, 0.58),
    "spatula": (0.22, 0.42, 0.24),
}


def handle_mask(img, tool):
    im = img.astype(np.int16)
    r, g, b = im[..., 0], im[..., 1], im[..., 2]
    if tool == "hammer":
        return (r - b > 25) & (r > g) & (g > b) & (r - g < (r - b) * 0.75)
    if tool == "screwdriver":
        return (r - g > 30) & (r - b > 30) & (np.abs(g - b) < 0.5 * (r - g))
    if tool == "wrench":
        return (b - r > 30) & (b - g > 15)
    if tool == "spatula":
        return (g - r > 18) & (g - b > 15)
    return np.zeros(img.shape[:2], bool)


def blob(mask, min_px=3):
    ys, xs = np.nonzero(mask)
    n = len(xs)
    if n < min_px:
        return None
    mx, my = xs.mean(), ys.mean()
    dx, dy = xs - mx, ys - my
    cov = np.array([[np.mean(dx * dx), np.mean(dx * dy)], [np.mean(dx * dy), np.mean(dy * dy)]])
    w, V = np.linalg.eigh(cov)
    return {"n": n, "c": (mx + 0.5, my + 0.5), "dir": V[:, 1], "len": np.sqrt(max(w[1], 0) * 12), "wid": np.sqrt(max(w[0], 0) * 12)}


def triangulate_blob(cams, bs, bh):
    o1, d1 = cams["side"].ray(*bs["c"])
    o2, d2 = cams["head"].ray(*bh["c"])
    p = triangulate([(o1, d1), (o2, d2)])
    # residual (ray distance)
    n = np.cross(d1, d2)
    res = abs(np.dot(o2 - o1, n)) / max(np.linalg.norm(n), 1e-9)
    return p, res


def axis_from_blobs(cams, bs, bh, p):
    """3D axis direction from the two image line directions: intersection of two planes."""
    normals = []
    for cam, b in ((cams["side"], bs), (cams["head"], bh)):
        u, v = b["c"]
        du, dv = b["dir"]
        o, d1 = cam.ray(u - 3 * du, v - 3 * dv)
        _, d2 = cam.ray(u + 3 * du, v + 3 * dv)
        n = np.cross(d1, d2)
        normals.append(n / np.linalg.norm(n))
    a = np.cross(normals[0], normals[1])
    na = np.linalg.norm(a)
    if na < 1e-6:
        return None
    return a / na
