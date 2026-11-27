"""Joint angles from the stereo endoscope video, by fitting the instrument's silhouette.

The instrument's model is its meshes (the datasheet's instrument_model/, copied next to this
file). A frame is fitted by rendering the model's silhouette in both views with MuJoCo
(software OpenGL), several samples per pixel, and matching that coverage to the frame's metal
coverage (the instrument is grey or black, the tissue pink) around the wrist.

1. Calibrate the endoscope pose from a few frames found by a grid search, starting from the
   pose the robot reports.
2. Track every frame in order: a constant-velocity prediction from the frames before, and a
   grid search over the whole joint range on the first frame and whenever the fit is poor
   (after a burst of lost frames, or when the track is lost).
3. Refine the endoscope pose jointly with the angles of a spread of well-conditioned frames.
4. Re-fit every frame with the refined pose. Frames whose angles the image does not pin
   down (a jaw out of view or seen end-on, the jaws overlapping) are dropped per joint.

fit.py later re-fits the frames whose angles disagree with what the motor log predicts
(refit()), from that prediction: a jaw can hide behind the other one or behind the clevis,
and a fit started on the wrong side of such an ambiguity stays there.
"""

import csv
import json
import math
import os
import sys
import time

import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter, maximum_filter, uniform_filter

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", os.environ["MUJOCO_GL"])
os.environ.setdefault("LP_NUM_THREADS", "1")      # one rendering thread per worker process
import mujoco  # noqa: E402

W, H = 320, 240
F = 210.0
CX, CY = 159.5, 119.5
BASELINE = 4.4
L1_NOM, L2_NOM = 9.1, 9.65
SS = 2                      # render samples per pixel, per axis
MODEL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "instrument_model")
# the datasheet's joint range (pitch +-90 deg, jaws +-120 deg)
LIMITS = np.radians([90.0, 120.0, 120.0])
H_FD = 1e-2                 # finite-difference step of the angles, rad (edges move ~0.3 px)

# Blur (pixels) of model and image coverage in the successive fitting stages.
SIGMAS_TRACK = (3.0, 1.5)
SIGMAS_FINAL = (1.5, 0.0)
SPARSE = 3                  # the first pass fits every SPARSE-th frame
# Behind time (a busy machine): past LATE_S seconds before the last pass, its fits take the
# sharp stage only. The program must answer within 600 s; on an idle sandbox it never is.
LATE_S = float(os.environ.get("WRIST_LATE_S", 360.0))
SIGMAS_FULL = (1.5, 0.7, 0.0)
POSE_SIGMA = 1.5

# points of the moving parts (clevis frame, jaw frames) and of the shaft, for regions of interest
_C = np.array([[x, y, z] for x in (-3.3, 3.3) for y in (-3.3, 3.3) for z in (-3.1, 11.6)])
_J = np.array([[x, y, z] for x in (-2.5, 2.5) for y in (-2.0, 2.0) for z in (-2.5, 9.7)])
_SHAFT = np.array([[x, y, z] for x in (-4.4, 4.4) for y in (-4.4, 4.4) for z in (-30.0, 3.1)])


def rodrigues(v):
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


# the disc the jaws sweep about the yaw axis (clevis frame, relative to the yaw axis)
_SWEEP = np.array([[r * math.sin(a), y, r * math.cos(a)] for a in np.radians(np.arange(-135, 136, 15))
                   for r in (10.0,) for y in (-2.0, 2.0)])


def moving_points(q):
    """Corner points of the clevis and of everywhere the jaws can be, in the instrument frame:
    a fit must see a jaw wherever it is, not only where the current estimate puts it."""
    R = rx(q[0])
    yaw_c = R @ np.array([0.0, 0.0, L1_NOM])
    return np.concatenate([_C @ R.T, yaw_c + _SWEEP @ R.T])


def load_obj(path):
    V, Fc = [], []
    with open(path) as f:
        for line in f:
            if line.startswith("v "):
                V.append([float(x) for x in line.split()[1:4]])
            elif line.startswith("f "):
                idx = [int(x.split("/")[0]) - 1 for x in line.split()[1:]]
                Fc += [[idx[0], idx[i], idx[i + 1]] for i in range(1, len(idx) - 1)]
    return np.array(V, float), np.array(Fc, int)


def build_model(width, height):
    """The clevis in geom group 0, jaw 1 in group 1, jaw 2 in group 2, the shaft in group 3."""
    spec = mujoco.MjSpec()
    spec.visual.quality.offsamples = 0
    spec.visual.map.znear = 0.002
    spec.visual.map.zfar = 50.0
    spec.stat.extent = 100.0
    spec.visual.global_.offwidth = width
    spec.visual.global_.offheight = height
    for name in ("shaft", "clevis", "jaw1", "jaw2"):
        V, Fc = load_obj(os.path.join(MODEL_DIR, name + ".obj"))
        spec.add_mesh(name=name, uservert=V.ravel().tolist(), userface=Fc.ravel().tolist())

    def geom(body, name, group):
        g = body.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=name, group=group)
        g.contype = 0
        g.conaffinity = 0
        g.mass = 1e-3

    geom(spec.worldbody, "shaft", 3)
    clevis = spec.worldbody.add_body(name="clevis")
    clevis.add_joint(name="pitch", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[1, 0, 0])
    geom(clevis, "clevis", 0)
    for g, name in ((1, "jaw1"), (2, "jaw2")):
        jaw = clevis.add_body(name=name, pos=[0.0, 0.0, L1_NOM])
        jaw.add_joint(name=name, type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0])
        geom(jaw, name, g)
    spec.worldbody.add_camera(name="cam", fovy=math.degrees(2.0 * math.atan(0.5 * H / F)))
    return spec.compile()


class Silhouette(object):
    """The model's coverage of a window of a view's pixels, rendered at `ss` x `ss` samples
    per pixel (segmentation colours, so a sample is either on the instrument or not). The
    window is its own camera: the same focal length, its principal point shifted."""

    def __init__(self, ss=SS, scale=1):
        self.ss, self.w, self.h = ss, W // scale, H // scale
        self.f, self.cx, self.cy = F / scale, (self.w - 1) / 2.0, (self.h - 1) / 2.0
        mw, mh = self.w * ss, self.h * ss
        self.m = build_model(mw, mh)
        self.d = mujoco.MjData(self.m)
        self.gl = mujoco.GLContext(mw, mh)
        self.gl.make_current()
        self.con = mujoco.MjrContext(self.m, mujoco.mjtFontScale.mjFONTSCALE_100)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.con)
        self.scn = mujoco.MjvScene(self.m, maxgeom=50)
        self.cam = self.m.camera("cam").id
        self.vcam = mujoco.MjvCamera()
        self.vcam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self.vcam.fixedcamid = self.cam
        self.pert = mujoco.MjvPerturb()
        self.opts = {}

    def _opt(self, groups):
        if groups not in self.opts:
            o = mujoco.MjvOption()
            o.geomgroup[:] = 0
            for g in groups:
                o.geomgroup[g] = 1
            self.opts[groups] = o
        return self.opts[groups]

    def coverage(self, q, view, box=None, groups=(0, 1, 2, 3)):
        v0, v1, u0, u1 = (0, self.h, 0, self.w) if box is None else box
        w, h, ss = u1 - u0, v1 - v0, self.ss
        if w <= 0 or h <= 0:
            return np.zeros((max(h, 0), max(w, 0)), np.float32)
        pos, R = view
        self.d.qpos[:] = q
        mujoco.mj_kinematics(self.m, self.d)
        m, c = self.m, self.cam
        m.cam_pos[c] = pos
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, np.column_stack([R[:, 0], -R[:, 1], -R[:, 2]]).ravel())
        m.cam_quat[c] = quat
        m.cam_sensorsize[c] = [w, h]
        m.cam_intrinsic[c] = [self.f, self.f, u0 + (w - 1) / 2.0 - self.cx,
                              v0 + (h - 1) / 2.0 - self.cy]
        mujoco.mj_camlight(m, self.d)
        mujoco.mjv_updateScene(m, self.d, self._opt(groups), self.pert, self.vcam,
                               mujoco.mjtCatBit.mjCAT_ALL, self.scn)
        self.scn.flags[mujoco.mjtRndFlag.mjRND_SEGMENT] = 1
        self.scn.flags[mujoco.mjtRndFlag.mjRND_IDCOLOR] = 1
        rect = mujoco.MjrRect(0, 0, w * ss, h * ss)
        mujoco.mjr_render(rect, self.scn, self.con)
        buf = np.empty((h * ss, w * ss, 3), np.uint8)
        mujoco.mjr_readPixels(buf, None, rect, self.con)
        # segmentation colours: a sample on a geom has a nonzero red (its id + 1)
        hit = buf[::-1, :, 0] != 0
        return hit.reshape(h, ss, w, ss).sum(axis=(1, 3), dtype=np.float32) * (1.0 / (ss * ss))


_SIL = {}


def silhouette(scale=1):
    """One renderer per process (and per image scale). Only worker processes render: an
    OpenGL context does not survive a fork."""
    key = (os.getpid(), scale)
    if key not in _SIL:
        _SIL[key] = Silhouette(SS if scale == 1 else 1, scale)
    return _SIL[key]


def metal_cover(path):
    """Metal coverage per pixel, as uint8 (a session's masks would not fit in memory as
    floats). The tissue is pink, the metal grey or black: the chroma r - (g + b) / 2 is near 0
    on metal, and it mixes linearly across an edge, so a pixel's coverage is one minus its
    chroma over the chroma the tissue has there. The tissue's chroma varies with its texture
    and the vignetting; it is interpolated from the nearby pixels that are clearly tissue
    (saturation, chroma over red, well above the metal's)."""
    img = np.asarray(Image.open(path), dtype=np.float32) / 255.0
    c = img[..., 0] - 0.5 * (img[..., 1] + img[..., 2])
    s = c / np.maximum(img[..., 0], 0.05)
    med = float(np.median(s))
    # clearly tissue, and not within two pixels of anything that is not: a pixel the metal
    # partly covers would pull the tissue's chroma down, and the silhouette with it
    w = ((s > 0.6 * med) & ~maximum_filter(s < 0.5 * med, size=5)).astype(np.float32)
    num = uniform_filter(c * w, 13, mode="nearest")
    den = uniform_filter(w, 13, mode="nearest")
    far = float(np.median(c[w > 0])) if np.any(w > 0) else 0.2
    tissue = np.where(den > 0.05, num / np.maximum(den, 1e-6), far)
    tissue = np.maximum(tissue, 0.02)
    m = squash(np.clip(1.0 - c / tissue, 0.0, 1.0))
    return np.round(m * 255.0).astype(np.uint8)


def squash(a):
    """Coverage with its ends clipped (image and model alike): the noise on the tissue and the
    JPEG's ringing just inside a dark edge then count for nothing."""
    return np.clip((a - 0.1) * 1.25, 0.0, 1.0)


class Session(object):
    def __init__(self, folder):
        self.folder = folder
        with open(os.path.join(folder, "endoscope_pose.json")) as f:
            pose = json.load(f)
        self.R0 = np.array(pose["R_left_camera"], dtype=float)
        self.p0 = np.array(pose["p_left_camera"], dtype=float)
        ed = os.path.join(folder, "endoscope")
        with open(os.path.join(ed, "frames.csv")) as f:
            rows = list(csv.DictReader(f))
        self.idx = np.array([int(r["frame"]) for r in rows])
        self.stamp = np.array([float(r["camera_t_s"]) for r in rows])
        paths = [os.path.join(ed, side, "{:05d}.jpg".format(k)) for k in self.idx
                 for side in ("left", "right")]
        from multiprocessing import get_context
        with get_context("fork").Pool(2) as pool:
            m = pool.map(metal_cover, paths, chunksize=32)
        self.masks = list(zip(m[0::2], m[1::2]))
        self.excl = None        # per view: pixels to leave out (a still second instrument)
        self.set_pose(np.zeros(6))

    def set_pose(self, g):
        self.g = np.asarray(g, float)
        R = rodrigues(self.g[:3]) @ self.R0
        pL = self.p0 + self.g[3:]
        self.views = [(pL, R), (pL + BASELINE * R[:, 0], R)]

    def project(self, P, view):
        pos, R = view
        q = (P - pos) @ R
        return np.column_stack([F * q[:, 0] / q[:, 2] + CX, F * q[:, 1] / q[:, 2] + CY]), q[:, 2]

    def roi(self, q, margin=10.0, shaft=False):
        """A box of pixels (v0, v1, u0, u1) around the wrist in each view; with `shaft`, also
        around the visible part of the shaft, which ties the endoscope pose to the instrument."""
        pts = moving_points(q)
        if shaft:
            pts = np.concatenate([pts, _SHAFT])
        out = []
        for view in self.views:
            uv, z = self.project(pts, view)
            front = z > 1.0
            if not np.any(front):
                out.append((0, 0, 0, 0))
                continue
            uv = uv[front]
            u0, u1 = int(max(0, uv[:, 0].min() - margin)), int(min(W, uv[:, 0].max() + margin + 1))
            v0, v1 = int(max(0, uv[:, 1].min() - margin)), int(min(H, uv[:, 1].max() + margin + 1))
            out.append((v0, max(v0, v1), u0, max(u0, u1)))
        return out

    def _keep(self, v, box):
        v0, v1, u0, u1 = box
        return None if self.excl is None else ~self.excl[v][v0:v1, u0:u1].ravel()

    def target(self, k, boxes, sigma):
        out = []
        for v, (box, m) in enumerate(zip(boxes, self.masks[k])):
            (a0, a1, b0, b1), (i0, i1, j0, j1) = _pad(box, sigma)
            img = m.reshape(H, W)[a0:a1, b0:b1].astype(np.float32) / 255.0
            t = _blur(img, sigma)[i0:i1, j0:j1].ravel()
            keep = self._keep(v, box)
            out.append(t if keep is None else t * keep)
        return np.concatenate(out)

    def model(self, q, boxes, sigma, views=None):
        sil = silhouette()
        out = []
        for v, (view, box) in enumerate(zip(self.views if views is None else views, boxes)):
            outer, (i0, i1, j0, j1) = _pad(box, sigma)
            c = _blur(squash(sil.coverage(q, view, outer)), sigma)[i0:i1, j0:j1].ravel()
            keep = self._keep(v, box)
            out.append(c if keep is None else c * keep)
        return np.concatenate(out)


def _pad(box, sigma):
    """The box grown by the blur's reach (clipped to the image), and where the box sits in it."""
    v0, v1, u0, u1 = box
    p = int(math.ceil(3.0 * sigma)) + 1 if sigma > 0 else 0
    a0, a1, b0, b1 = max(0, v0 - p), min(H, v1 + p), max(0, u0 - p), min(W, u1 + p)
    return (a0, a1, b0, b1), (v0 - a0, v1 - a0, u0 - b0, u1 - b0)


def _blur(img, sigma):
    return gaussian_filter(img, sigma, mode="nearest") if sigma > 0 else img


def grid_search(sess, k):
    """Coarse pose of one frame over the whole joint range, at half resolution: for each pitch
    the shaft and clevis are rendered once and every jaw angle once per jaw; every jaw pair
    (jaw 1 at the larger angle) is then a maximum of three coverages."""
    sil = silhouette(scale=2)
    tps = np.linspace(-1.5, 1.5, 17)
    tjs = np.linspace(-2.0, 2.0, 21)
    targets, keeps = [], []
    for v, m in enumerate(sess.masks[k]):
        img = m.reshape(H, W).astype(np.float32) / 255.0
        targets.append(gaussian_filter(img.reshape(H // 2, 2, W // 2, 2).mean(axis=(1, 3)), 1.0))
        keeps.append(np.ones((H // 2) * (W // 2), bool) if sess.excl is None else
                     ~sess.excl[v].reshape(H // 2, 2, W // 2, 2).any(axis=(1, 3)).ravel())
    pair = np.tril(np.ones((len(tjs), len(tjs)), bool), -1)    # i > j: jaw 1 = tjs[i]
    best = (np.inf, None)
    for tp in tps:
        cost = np.zeros((len(tjs), len(tjs)))
        for view, y, kp in zip(sess.views, targets, keeps):
            base = gaussian_filter(squash(sil.coverage((tp, 0.0, 0.0), view, None, (0, 3))),
                                   1.0).ravel()
            j1 = np.stack([gaussian_filter(squash(sil.coverage((tp, t, 0.0), view, None, (1,))),
                                           1.0).ravel() for t in tjs], 1)
            j2 = np.stack([gaussian_filter(squash(sil.coverage((tp, 0.0, t), view, None, (2,))),
                                           1.0).ravel() for t in tjs], 1)
            yy = y.ravel()
            keep = ((base > 0) | (j1.max(1) > 0) | (j2.max(1) > 0) | (yy > 0.05)) & kp
            b, a1, a2, yk = base[keep], j1[keep], j2[keep], yy[keep]
            one = np.maximum(b[:, None], a1)
            both = np.maximum(one[:, :, None], a2[:, None, :]) - yk[:, None, None]
            cost += np.einsum("nij,nij->ij", both, both)
        cost[~pair] = np.inf
        i, j = np.unravel_index(int(np.argmin(cost)), cost.shape)
        if cost[i, j] < best[0]:
            best = (cost[i, j], np.array([tp, tjs[i], tjs[j]]))
    return best[1]


def fit_frame(sess, k, q0, sigmas=SIGMAS_FULL, iters=4):
    """Levenberg-Marquardt on one frame's angles, from a blurred silhouette to a sharp one.
    The Jacobian (finite differences: one render per angle and view) is computed once per
    stage and kept for the stage's iterations (the silhouette moves little in between)."""
    q = np.clip(np.array(q0, float), -LIMITS, LIMITS)
    J = np.zeros((1, 3))
    r = np.zeros(1)
    for sigma in sigmas:
        boxes = sess.roi(q)
        y = sess.target(k, boxes, sigma)
        if len(y) == 0:
            break

        def res(x):
            return sess.model(x, boxes, sigma) - y

        r = res(q)
        cost = float(r @ r)
        lam = 1e-3
        J = np.zeros((len(r), 3))
        for i in range(3):
            dq = np.zeros(3)
            dq[i] = H_FD
            J[:, i] = (res(q + dq) - r) / H_FD
        A = J.T @ J
        for _ in range(iters):
            g = J.T @ r
            moved = False
            for _ in range(3):
                dx = -np.linalg.solve(A + lam * (np.diag(np.diag(A)) + 1e-9 * np.eye(3)), g)
                qn = np.clip(q + dx, -LIMITS, LIMITS)
                rn = res(qn)
                cn = float(rn @ rn)
                if cn < cost:
                    q, r, cost = qn, rn, cn
                    lam = max(lam / 4.0, 1e-6)
                    moved = True
                    break
                lam *= 10.0
            if not moved or np.max(np.abs(dx)) < 3e-4:
                break
    swap = q[1] < q[2]      # the jaws look alike: jaw 1 is the one at the larger angle
    rms = float(np.sqrt(np.mean(r ** 2))) if len(r) else np.inf
    try:
        cov = np.linalg.inv(J.T @ J) * max(rms, 0.02) ** 2
        sd = np.sqrt(np.maximum(np.diag(cov), 0.0))
    except np.linalg.LinAlgError:
        sd = np.full(3, np.inf)
    if swap:
        q = q[[0, 2, 1]]
        sd = sd[[0, 2, 1]]
    return q, rms, sd


LOST_RMS = 0.12


def track(sess, init=None, log=None, k0=0, k1=None, frames=None, sigmas=SIGMAS_TRACK, iters=4):
    """Fit frames k0..k1-1 (or the listed `frames`) in order; `init` (one row per frame)
    gives each frame's starting angles, else a constant-velocity prediction from the frames
    before (a grid search where there is none, or where the fit from it is poor)."""
    if frames is None:
        frames = np.arange(k0, len(sess.idx) if k1 is None else k1)
    n = len(frames)
    Q = np.full((n, 3), np.nan)
    RMS = np.full(n, np.inf)
    SD = np.full((n, 3), np.inf)
    t_start, grids, t_grid = time.time(), 0, 0.0
    for i, k in enumerate(frames):
        pred = None
        if init is not None:
            pred = init[i]
        elif i >= 2 and np.all(np.isfinite(Q[i - 2:i])) \
                and sess.stamp[k] - sess.stamp[frames[i - 2]] < 0.5:
            dt1 = sess.stamp[k] - sess.stamp[frames[i - 1]]
            dt0 = sess.stamp[frames[i - 1]] - sess.stamp[frames[i - 2]]
            pred = Q[i - 1] + (Q[i - 1] - Q[i - 2]) * dt1 / max(dt0, 1e-3)
        elif i >= 1 and np.all(np.isfinite(Q[i - 1])):
            pred = Q[i - 1]
        if pred is None or not np.all(np.isfinite(pred)):
            t0 = time.time()
            pred = grid_search(sess, k)
            grids, t_grid = grids + 1, t_grid + time.time() - t0
        q, rms, sd = fit_frame(sess, k, pred, sigmas, iters)
        if rms > LOST_RMS and init is None:
            # a wider basin from the same start first, the whole joint range only then
            q2, rms2, sd2 = fit_frame(sess, k, pred, (6.0,) + tuple(sigmas), iters)
            if rms2 < rms:
                q, rms, sd = q2, rms2, sd2
        if rms > LOST_RMS and init is None:
            t0 = time.time()
            q2, rms2, sd2 = fit_frame(sess, k, grid_search(sess, k), SIGMAS_FULL)
            grids, t_grid = grids + 1, t_grid + time.time() - t0
            if rms2 < rms:
                q, rms, sd = q2, rms2, sd2
        Q[i], RMS[i], SD[i] = q, rms, sd
        if log and i % 500 == 0:
            print("[vision] {} frame {}/{} rms {:.3f}".format(os.path.basename(sess.folder), k,
                                                            len(sess.idx), rms),
                  file=sys.stderr, flush=True)
    if log:
        print("[vision] {} frames in {:.1f} s, {} grid searches ({:.1f} s)".format(
            n, time.time() - t_start, grids, t_grid), file=sys.stderr, flush=True)
    return Q, RMS, SD


_SESS = None


def _pool(sess, workers):
    """Worker processes sharing the session's masks (fork). The parent process never renders:
    every rendering happens in a worker, with the endoscope pose sent along with the task."""
    global _SESS
    from multiprocessing import get_context
    _SESS = sess
    return get_context("fork").Pool(workers)


def _w_track(args):
    g, frames, init, sigmas, iters = args
    _SESS.set_pose(g)
    return track(_SESS, init=init, log=True, frames=frames, sigmas=sigmas, iters=iters)


def track_all(sess, init=None, workers=2, frames=None, sigmas=SIGMAS_TRACK, pool=None, iters=4):
    """track() over the session's frames (or the listed ones), split into `workers`
    consecutive chunks fitted side by side (each chunk starts with its own grid search)."""
    frames = np.arange(len(sess.idx)) if frames is None else np.asarray(frames)
    n = len(frames)
    b = np.linspace(0, n, workers + 1).astype(int)
    jobs = [(sess.g, frames[b[i]:b[i + 1]], None if init is None else init[b[i]:b[i + 1]],
             sigmas, iters) for i in range(workers) if b[i + 1] > b[i]]
    own = pool is None
    pool = _pool(sess, workers) if own else pool
    try:
        parts = pool.map(_w_track, jobs)
    finally:
        if own:
            pool.close()
            pool.join()
    return tuple(np.concatenate([p[j] for p in parts]) for j in range(3))


def spread(Q, ok, n):
    """n of the frames `ok`, far apart in angle space."""
    pick = [ok[0]]
    for _ in range(n - 1):
        d = np.min(np.linalg.norm(Q[ok][:, None, :] - Q[pick][None, :, :], axis=2), axis=1)
        pick.append(ok[int(np.argmax(d))])
    return np.array(pick)


POSE_STEPS = np.array([3e-3, 3e-3, 3e-3, 0.1, 0.1, 0.1])


def _views_of(sess, g):
    R = rodrigues(g[:3]) @ sess.R0
    pL = sess.p0 + g[3:]
    return [(pL, R), (pL + BASELINE * R[:, 0], R)]


def _pose_terms(sess, g, ks, qs, boxes, sigma):
    """Per frame: residual, its Jacobian on the pose (6) and on the frame's angles (3)."""
    out = []
    vs = _views_of(sess, g)
    for k, q, b in zip(ks, qs, boxes):
        y = sess.target(k, b, sigma)
        r0 = sess.model(q, b, sigma, vs) - y
        Jg = np.empty((len(r0), 6))
        for i in range(6):
            dg = np.zeros(6)
            dg[i] = POSE_STEPS[i]
            Jg[:, i] = (sess.model(q, b, sigma, _views_of(sess, g + dg))
                        - sess.model(q, b, sigma, _views_of(sess, g - dg))) / (2 * POSE_STEPS[i])
        Jq = np.empty((len(r0), 3))
        for i in range(3):
            dq = np.zeros(3)
            dq[i] = H_FD
            Jq[:, i] = (sess.model(q + dq, b, sigma, vs) - y - r0) / H_FD
        out.append((r0, Jg, Jq))
    return out


def _pose_res(sess, g, ks, qs, boxes, sigma):
    vs = _views_of(sess, g)
    return [sess.model(q, b, sigma, vs) - sess.target(k, b, sigma)
            for k, q, b in zip(ks, qs, boxes)]


def _w_pose(args):
    what, g, ks, qs, boxes, sigma = args
    return (_pose_terms if what == "terms" else _pose_res)(_SESS, g, ks, qs, boxes, sigma)


def refine_pose(sess, Q, pick, sigma, iters=6, pool=None, workers=2, prune=False):
    """Endoscope pose (6) jointly with the angles of the frames `pick` (Levenberg-Marquardt,
    the angles eliminated per frame; an accepted step is tried again at twice and four times
    its length, since the weak directions converge slowly). With `prune`, the frames fitted
    clearly worse than the others after the first round (a jaw caught in a wrong place) are
    left out. With a pool, the frames are shared among its workers."""
    pick = np.asarray(pick)
    qs = Q[pick].copy()
    g = sess.g.copy()

    def run(what, gv, qv, boxes, ks):
        chunks = np.array_split(np.arange(len(ks)), workers if pool is not None else 1)
        jobs = [(what, gv, ks[c], qv[c], [boxes[i] for i in c], sigma) for c in chunks if len(c)]
        res = pool.map(_w_pose, jobs) if pool is not None else [_w_pose(j) for j in jobs]
        return [x for part in res for x in part]

    # little damping: the pose's weakest direction (a turn about the shaft, which looks the same
    # from every side) has a tiny curvature next to the others, and a Marquardt term would all
    # but freeze it
    lam = 1e-7
    keep = np.ones(len(pick), bool)
    for it in range(iters):
        sess.set_pose(g)
        ks, qk = pick[keep], qs[keep]
        boxes = [sess.roi(q, shaft=True) for q in qk]
        terms = run("terms", g, qk, boxes, ks)
        if prune and it == 1:
            frms = np.array([np.sqrt(np.mean(r ** 2)) for r, _, _ in terms])
            good = frms <= 1.5 * np.median(frms)
            if not np.all(good):
                idx = np.where(keep)[0]
                keep[idx[~good]] = False
                ks, qk = pick[keep], qs[keep]
                boxes = [b for b, ok in zip(boxes, good) if ok]
                terms = [t for t, ok in zip(terms, good) if ok]
        cost = sum(float(r @ r) for r, _, _ in terms)
        cn = cost
        while True:
            S = np.zeros((6, 6))
            bvec = np.zeros(6)
            parts = []
            for r0, Jg, Jq in terms:
                A = Jq.T @ Jq
                A = A + lam * np.diag(np.diag(A)) + 1e-9 * np.eye(3)
                Ai = np.linalg.inv(A)
                Bm = Jq.T @ Jg
                S += Jg.T @ Jg - Bm.T @ Ai @ Bm
                bvec += Jg.T @ r0 - Bm.T @ Ai @ (Jq.T @ r0)
                parts.append((Ai, Bm, Jq, r0))
            sess.pose_S = S.copy()
            S += lam * np.diag(np.diag(S)) + 1e-9 * np.eye(6)
            dg = -np.linalg.solve(S, bvec)
            dqs = np.array([-Ai @ (Jq.T @ r0 + Bm @ dg) for Ai, Bm, Jq, r0 in parts])
            rn = run("res", g + dg, qk + dqs, boxes, ks)
            cn = sum(float(r @ r) for r in rn)
            if cn < cost:
                step = 1.0
                for mult in (2.0, 4.0):
                    rm = run("res", g + mult * dg, qk + mult * dqs, boxes, ks)
                    cm = sum(float(r @ r) for r in rm)
                    if cm >= cn:
                        break
                    step, cn = mult, cm
                g = g + step * dg
                qk = qk + step * dqs
                qs[keep] = qk
                lam = max(lam / 4.0, 1e-12)
                break
            lam *= 6.0
            if lam > 1e6:
                break
        print("[vision] {} pose round {} sigma {} frames {} cost {:.3f} rot {} mm {}".format(
            os.path.basename(sess.folder), it, sigma, int(keep.sum()), cn,
            np.round(np.degrees(g[:3]), 3), np.round(g[3:6], 3)), file=sys.stderr, flush=True)
        if lam > 1e6 or cost - cn < 1e-6 * cost:
            break
    sess.set_pose(g)
    Q[pick[keep]] = qs[keep]


def rotvec(R):
    """Rotation vector of a rotation matrix."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    th = math.acos(c)
    if th < 1e-12:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / (2.0 * math.sin(th)) * th


def turned(sess, g, a, axis=2):
    """The pose g with the endoscope turned by `a` about an axis of the instrument frame through
    its origin: about the shaft (z) the shaft looks the same from every side and only the wrist
    tells the turn; about y the shaft tilts a little and the wrist turns like a yaw offset that
    no joint can make up."""
    Rt = rodrigues(a * np.eye(3)[axis])
    R = Rt @ rodrigues(g[:3]) @ sess.R0
    p = Rt @ (sess.p0 + g[3:])
    return np.concatenate([rotvec(R @ sess.R0.T), p - sess.p0])


def _w_scan(args):
    g, ks, qs, sigmas, shaft = args
    _SESS.set_pose(g)
    cost = 0.0
    fitted = []
    for k, q in zip(ks, qs):
        q2, rms, _ = fit_frame(_SESS, k, q, sigmas, iters=3)
        if shaft:
            b = _SESS.roi(q2, shaft=True)
            r = _SESS.model(q2, b, sigmas[-1]) - _SESS.target(k, b, sigmas[-1])
            cost += float(r @ r)
        else:
            b = _SESS.roi(q2)
            cost += rms ** 2 * sum((v1 - v0) * (u1 - u0) for v0, v1, u0, u1 in b)
        fitted.append(q2)
    return cost, np.array(fitted)


def scan_flat(sess, Q, pick, pool, span=0.15, n=13, sigmas=(1.0,), dist=35.0):
    """Scan the pose along its flattest direction left by the joint refinement (the reduced
    Hessian's weakest eigenvector, rotations weighted by the viewing distance), the frames
    fitted afresh at every step and the shaft counted: +-span mm at the wrist."""
    S = getattr(sess, "pose_S", None)
    if S is None:
        return
    D = np.r_[np.full(3, dist), np.ones(3)]
    w, V_ = np.linalg.eigh(S / np.outer(D, D))
    d = V_[:, 0] / D
    d /= np.linalg.norm(d * D)
    steps = np.linspace(-span, span, n)
    g0 = sess.g.copy()
    jobs = [(g0 + a * d, pick, Q[pick], sigmas, True) for a in steps]
    res = pool.map(_w_scan, jobs)
    costs = np.array([c for c, _ in res])
    i = int(np.clip(np.argmin(costs), 1, n - 2))
    c0, c1, c2 = costs[i - 1:i + 2]
    den = c0 - 2 * c1 + c2
    a = steps[i] + (0.5 * (c0 - c2) / den * (steps[1] - steps[0]) if den > 0 else 0.0)
    sess.set_pose(g0 + a * d)
    Q[pick] = res[int(np.argmin(np.abs(steps - a)))][1]
    print("[vision] {} flattest pose direction {} moved {:.3f} mm (costs {})".format(
        os.path.basename(sess.folder), np.round(d * D, 2), a, np.round(costs / costs.min(), 4)),
        file=sys.stderr, flush=True)


def scan_turn(sess, Q, pick, pool, span=0.035, n=15, sigmas=(1.0,), axis=2, shaft=False):
    """Turn the endoscope about an instrument axis to where the frames `pick`, each fitted
    afresh, are matched best (a parabola through the best three of n turns in +-span rad)."""
    turns = np.linspace(-span, span, n)
    g0 = sess.g.copy()
    jobs = [(turned(sess, g0, a, axis), pick, Q[pick], sigmas, shaft) for a in turns]
    res = pool.map(_w_scan, jobs)
    costs = np.array([c for c, _ in res])
    i = int(np.clip(np.argmin(costs), 1, n - 2))
    c0, c1, c2 = costs[i - 1:i + 2]
    den = c0 - 2 * c1 + c2
    a = turns[i] + (0.5 * (c0 - c2) / den * (turns[1] - turns[0]) if den > 0 else 0.0)
    sess.set_pose(turned(sess, g0, a, axis))
    # the frames' angles fitted at the nearest scanned turn
    Q[pick] = res[int(np.argmin(np.abs(turns - a)))][1]
    print("[vision] {} turn about {} {:.2f} deg (costs {})".format(
        os.path.basename(sess.folder), "xyz"[axis], math.degrees(a),
        np.round(costs / costs.min(), 3)), file=sys.stderr, flush=True)


def _w_grid_fit(args):
    g, k = args
    _SESS.set_pose(g)
    return fit_frame(_SESS, k, grid_search(_SESS, k), (3.0, 1.5))[0]


def _w_shaft(g):
    _SESS.set_pose(g)
    sil = silhouette()
    return [sil.coverage((0.0, 0.0, 0.0), view, None, (3,)) for view in _SESS.views]


def exclude_still(sess, reach):
    """Leave out the pixels that are metal in nearly every frame without being the shaft (as
    the current pose places it, give or take `reach` pixels): a second instrument held still
    in the field, and whatever of the wrist it hides. (Pixels the wrist covers in every frame
    tell nothing about its angles either.)"""
    from scipy.ndimage import binary_dilation
    with _pool(sess, 1) as pool:
        shaft = pool.apply(_w_shaft, (sess.g,))
    excl = []
    for v in range(2):
        frac = np.zeros((H, W))
        for m in sess.masks:
            frac += m[v].reshape(H, W) >= 128
        still = frac / len(sess.masks) >= 0.97
        near_shaft = binary_dilation(shaft[v] > 0.5, iterations=reach) if reach else shaft[v] > 0.5
        excl.append(binary_dilation(still & ~near_shaft, iterations=2))
    sess.excl = excl
    note("{}: {:.1%} of the pixels left out (still metal)".format(
        os.path.basename(sess.folder), np.mean([e.mean() for e in excl])))


def calibrate_pose(sess, pool, n_frames=8, workers=2):
    """First endoscope pose: a grid search of a few frames' angles with the reported pose,
    then the pose and those angles jointly, from a blurred silhouette to a sharp one."""
    pick = np.linspace(0, len(sess.idx) - 1, n_frames).astype(int)
    Q = np.full((len(sess.idx), 3), np.nan)
    Q[pick] = np.array(pool.map(_w_grid_fit, [(sess.g, k) for k in pick]))
    for sigma in (4.0, 2.0, 1.0):
        refine_pose(sess, Q, pick, sigma, iters=5, pool=pool, workers=workers)


def usable(Q, RMS, SD):
    """NaN where the image does not pin an angle down, or where the fit is well worse than the
    session's typical one: a jaw hidden behind the clevis is often fitted into a wrong place
    that explains the image almost as well (fit.py re-fits those frames from the model)."""
    fin = np.isfinite(RMS)
    typical = float(np.median(RMS[fin])) if np.any(fin) else 0.0
    bad = (SD > 0.004) | (RMS[:, None] > min(0.1, 1.5 * typical))
    return np.where(bad, np.nan, Q)


_T0 = time.time()


def note(msg):
    print("[vision] {:6.1f} s  {}".format(time.time() - _T0, msg), file=sys.stderr, flush=True)


def angles(bench, workers=2):
    """Per session: camera stamps and joint angles (NaN where the image does not tell), the
    sessions themselves (for refit()) and the instrument's L1, L2 (those of its meshes)."""
    folders = [os.path.join(bench, n) for n in sorted(os.listdir(bench))
               if os.path.isdir(os.path.join(bench, n, "endoscope"))]
    out = {}
    for folder in folders:
        s = Session(folder)
        note("{}: {} frames loaded".format(os.path.basename(folder), len(s.idx)))
        exclude_still(s, 10)          # the reported pose: the shaft within 10 px
        with _pool(s, workers) as pool:
            calibrate_pose(s, pool, workers=workers)
            note("first endoscope pose")
        exclude_still(s, 1)
        with _pool(s, workers) as pool:
            n = len(s.idx)
            sparse = np.arange(0, n, SPARSE)
            Q = np.full((n, 3), np.nan)
            RMS = np.full(n, np.inf)
            SD = np.full((n, 3), np.inf)
            Q[sparse], RMS[sparse], SD[sparse] = track_all(s, workers=workers, frames=sparse,
                                                           pool=pool)
            note("tracked every {}th frame".format(SPARSE))
            fin = np.isfinite(RMS)
            ok = np.where(fin & (RMS < np.quantile(RMS[fin], 0.5)) & np.all(SD < 0.01, axis=1))[0]
            pick = spread(Q, ok, 40)
            debug = dict(q_track=Q.copy(), rms_track=RMS.copy(), pick=pick, g_cal=s.g.copy())
            # a turn about the shaft barely changes any one frame: scan it over many frames,
            # each fitted afresh; then the whole pose on a smooth silhouette. A turn about the
            # pitch axis looks like a pitch offset to the wrist and only the shaft pins it, but
            # it is NOT absorbed later: the video is the only reference for every joint's zero,
            # and a 3 mrad error there is about 0.06 mm at the jaw tips.
            # (A sharper silhouette does not help the pose: its finite differences are rougher.)
            scan_turn(s, Q, pick[:24], pool)
            refine_pose(s, Q, pick, POSE_SIGMA, iters=8, pool=pool, workers=workers, prune=True)
            scan_turn(s, Q, pick[:24], pool, span=0.014, n=9)
            refine_pose(s, Q, pick, POSE_SIGMA, iters=8, pool=pool, workers=workers, prune=True)
            # the flattest direction left (the absolute pitch rides on it: to the wrist a turn
            # about the pitch axis is a pitch offset, and only the shaft tells it), scanned
            scan_flat(s, Q, pick[:24], pool)
            note("endoscope pose refined")
            # every frame afresh with the refined pose, from the first pass's angles
            # (interpolated in time between the frames it fitted)
            good = np.isfinite(RMS) & (RMS < LOST_RMS)
            init = np.column_stack([np.interp(s.stamp, s.stamp[good], Q[good, j])
                                    for j in range(3)])
            sigmas = SIGMAS_FINAL
            if time.time() - _T0 > LATE_S:
                sigmas = SIGMAS_FINAL[-1:]
                note("behind time: one-stage fits in the last pass")
            Q, RMS, SD = track_all(s, init=init, workers=workers, sigmas=sigmas, pool=pool)
        Q = usable(Q, RMS, SD)
        note("{}: re-fitted, kept pitch {:.1%} jaw1 {:.1%} jaw2 {:.1%}".format(
            os.path.basename(s.folder), *np.mean(np.isfinite(Q), axis=0)))
        out[os.path.basename(s.folder)] = dict(stamp=s.stamp, angles=Q, rms=RMS, sd=SD,
                                               session=s, debug=debug)
    return out, L1_NOM, L2_NOM


def refit(sess, frames, init, l1, l2, workers=2):
    """Fit the listed frames afresh from the given starting angles (one row per frame), from a
    blurred silhouette to a sharp one. Returns their angles (NaN where the image does not
    pin them down) and fit residuals."""
    Q, RMS, SD = track_all(sess, init=np.asarray(init, float), workers=workers,
                           frames=frames, sigmas=SIGMAS_FULL)
    return usable(Q, RMS, SD), RMS
