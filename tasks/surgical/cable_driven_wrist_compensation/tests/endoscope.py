"""The stereo endoscope: renders the instrument model (the dVRK Large Needle Driver's meshes,
instrument_model/*.obj) over the tissue with MuJoCo's software OpenGL renderer. The grader
renders each hidden instrument's calibration video with it (render_frames.py); setup renders
the agent's example instruments with a byte-identical copy in environment/setup/render/
(synced by tests/gen/make_payload.py). Never staged into the agent home.

The pair is rectified: both cameras share one orientation and intrinsics, and the right camera
sits BASELINE mm along the left camera's x axis. Each view is rendered at SUPERSAMPLE x
SUPERSAMPLE samples per pixel; a pixel on the instrument's outline mixes metal and tissue in
proportion to its covered samples. The light comes from the endoscope tip.
"""

import math
import os

import numpy as np

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", os.environ["MUJOCO_GL"])
import mujoco  # noqa: E402

W, H = 320, 240
F = 210.0
CX, CY = 159.5, 119.5
BASELINE = 4.4  # mm
L1 = 9.1        # pitch axis to yaw axis, mm (the jaw meshes hang from the yaw axis)
SUPERSAMPLE = 2
MODEL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "instrument_model")
PARTS = ("shaft", "clevis", "jaw1", "jaw2")
# surface colour, specular, shininess: the shaft is a dark composite tube, the wrist steel
MATERIALS = dict(shaft=((0.13, 0.13, 0.14), 0.35, 0.5), clevis=((0.56, 0.57, 0.60), 0.7, 0.6),
                 jaw1=((0.66, 0.67, 0.70), 0.8, 0.7), jaw2=((0.66, 0.67, 0.70), 0.8, 0.7))


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


def build_model(model_dir=MODEL_DIR, width=W * SUPERSAMPLE, height=H * SUPERSAMPLE,
                occluder=None):
    """The instrument as a MuJoCo model: the shaft fixed, the clevis on a hinge about x (the
    pitch axis), each jaw on a hinge about the clevis's y axis at (0, 0, L1); one camera; and
    `occluder` (pose R, p and joint angles in the instrument frame), a second, still
    instrument of the same kind."""
    spec = mujoco.MjSpec()
    spec.visual.headlight.ambient = [0.10, 0.10, 0.10]
    spec.visual.headlight.diffuse = [0.75, 0.75, 0.75]
    spec.visual.headlight.specular = [0.45, 0.45, 0.45]
    spec.visual.quality.offsamples = 0
    spec.visual.map.znear = 0.002
    spec.visual.map.zfar = 50.0
    spec.stat.extent = 100.0
    spec.visual.global_.offwidth = width
    spec.visual.global_.offheight = height
    for name in PARTS:
        rgb, spc, shin = MATERIALS[name]
        mat = spec.add_material(name=name)
        mat.rgba = list(rgb) + [1.0]
        mat.specular = spc
        mat.shininess = shin
        V, Fc = load_obj(os.path.join(model_dir, name + ".obj"))
        spec.add_mesh(name=name, uservert=V.ravel().tolist(), userface=Fc.ravel().tolist())

    def geom(body, name):
        g = body.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=name, material=name)
        g.contype = 0
        g.conaffinity = 0
        g.mass = 1e-3

    geom(spec.worldbody, "shaft")
    clevis = spec.worldbody.add_body(name="clevis")
    clevis.add_joint(name="pitch", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[1, 0, 0])
    geom(clevis, "clevis")
    for name in ("jaw1", "jaw2"):
        jaw = clevis.add_body(name=name, pos=[0.0, 0.0, L1])
        jaw.add_joint(name=name, type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0])
        geom(jaw, name)
    if occluder is not None:
        R = np.asarray(occluder["R"], dtype=float)
        tp, t1, t2 = occluder["angles"]
        base = spec.worldbody.add_body(name="occ", pos=list(occluder["p"]), quat=_quat(R))
        geom(base, "shaft")
        cl = base.add_body(name="occ_clevis", quat=_quat(_rx(tp)))
        geom(cl, "clevis")
        for name, t in (("jaw1", t1), ("jaw2", t2)):
            geom(cl.add_body(name="occ_" + name, pos=[0.0, 0.0, L1], quat=_quat(_ry(t))), name)
    spec.worldbody.add_camera(name="cam", fovy=math.degrees(2.0 * math.atan(0.5 * H / F)))
    return spec.compile()


def _quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).ravel())
    return q.tolist()


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def camera_quat(R):
    """MuJoCo's camera looks along its -z with y up; ours along z with y down (image rows)."""
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.column_stack([R[:, 0], -R[:, 1], -R[:, 2]]).ravel())
    return q


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """Left-camera pose in the instrument frame: rotation (columns = camera x right, y down,
    z forward) and position."""
    eye = np.asarray(eye, float)
    z = np.asarray(target, float) - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, np.asarray(up, float))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z]), eye


def tissue(R, pos, seed):
    """Tissue on the sphere of radius 70 mm around the wrist, seen from inside, with a fixed
    random texture: (H, W, 3)."""
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    d = np.stack([(u - CX) / F, (v - CY) / F, np.ones(u.shape)], -1).reshape(-1, 3)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    d = d @ np.asarray(R, float).T
    o = np.asarray(pos, float)
    rng = np.random.default_rng(seed)
    k = rng.normal(size=(8, 3))
    ph = rng.uniform(0, 2 * math.pi, 8)
    amp = rng.uniform(0.03, 0.08, 8)
    b = d @ o
    c = o @ o - 70.0 ** 2
    t = -b + np.sqrt(np.maximum(b * b - c, 0.0))
    p = o + t[:, None] * d
    tex = sum(amp[i] * np.sin(0.18 * p @ k[i] + ph[i]) for i in range(8))
    base = np.array([0.80, 0.43, 0.40])
    return np.clip(base[None, :] * (1.0 + tex[:, None]), 0.0, 1.0).reshape(H, W, 3)


class Stereo(object):
    """One recording's endoscope (left camera pose R, pos) and tissue."""

    def __init__(self, R, pos, tissue_seed, model=None, occluder=None):
        self.R, self.pos = np.asarray(R, float), np.asarray(pos, float)
        self.m = build_model(occluder=occluder) if model is None else model
        self.d = mujoco.MjData(self.m)
        self.renderer = mujoco.Renderer(self.m, H * SUPERSAMPLE, W * SUPERSAMPLE)
        self.cam = self.m.camera("cam").id
        self.m.cam_quat[self.cam] = camera_quat(self.R)
        self.views = [self.pos, self.pos + BASELINE * self.R[:, 0]]
        self.bg = [tissue(self.R, p, tissue_seed) for p in self.views]
        u, v = np.meshgrid(np.arange(W), np.arange(H))
        self.vig = 1.0 - 0.38 * (((u - CX) / CX) ** 2 + ((v - CY) / CY) ** 2)

    def close(self):
        self.renderer.close()

    def coverage(self, tp, t1, t2):
        """Per view: the instruments' coverage of each pixel (H, W) and their mean colour there
        (H, W, 3), at joint angles tp (pitch), t1 and t2 (jaws)."""
        self.d.qpos[:] = [tp, t1, t2]
        mujoco.mj_kinematics(self.m, self.d)
        s = SUPERSAMPLE
        out = []
        for p in self.views:
            self.m.cam_pos[self.cam] = p
            mujoco.mj_camlight(self.m, self.d)
            self.renderer.update_scene(self.d, camera=self.cam)
            rgb = self.renderer.render().astype(np.float32) / 255.0
            self.renderer.enable_segmentation_rendering()
            seg = self.renderer.render()[..., 0]
            self.renderer.disable_segmentation_rendering()
            hit = (seg >= 0).astype(np.float32)
            n = hit.reshape(H, s, W, s).sum(axis=(1, 3))
            col = (rgb * hit[..., None]).reshape(H, s, W, s, 3).sum(axis=(1, 3))
            out.append((n / (s * s), col / np.maximum(n, 1.0)[..., None]))
        return out

    def render(self, tp, t1, t2, rng=None, haze=0.0):
        """Both images (H, W, 3) in [0, 1]."""
        imgs = []
        for (a, col), bg in zip(self.coverage(tp, t1, t2), self.bg):
            img = a[..., None] * col + (1.0 - a[..., None]) * bg
            img = img * self.vig[..., None]
            if haze > 0.0:     # smoke: the view washes out towards a bright grey
                img = (1.0 - haze) * img + haze * np.array([0.78, 0.76, 0.75])
            if rng is not None:
                img = img + rng.normal(0.0, 0.012, img.shape)
            imgs.append(np.clip(img, 0.0, 1.0))
        return imgs
