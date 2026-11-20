"""Cameras of the belt station and an offscreen rig with calibration and the sensor model (cell internals; the rig is
adapted from the benchmark's shoe_lacing_and_boxing and pump_skid_startup_and_operation cells).

Fixed camera ``front`` (1280x960, fovy 50 deg): above and between the robots' bases, looking over the drive.
Wrist cameras ``left_wrist`` / ``right_wrist`` (640x480, fovy 58 deg, D405 class) on ``L/hand`` / ``R/hand``: 58 mm to
the hand's -x side, 62 mm behind the tcp, looking at the point 0.15 m ahead of the tcp along the approach axis. Their
mount is published in describe() as T_body_cam.

MuJoCo camera frame: looks along its -z, +y up, +x right.  OpenCV frame (what :meth:`CameraRig.extrinsics` returns):
+x right, +y down, +z forward  ->  R_cv = R_mj diag(1, -1, -1). Depth: the PLANAR depth along the optical axis (not the
ray length), in metres. Intrinsics use integer pixel indices (u = column, v = row, row 0 at the top): a pixel's centre
ray is x/z = (u - cx) / fx with cx = W/2 - 0.5, cy = H/2 - 0.5, fx = fy = (H/2) / tan(fovy/2).

Sensor model (:func:`apply_sensor_model`, per frame; the colour gains and brightness are drawn once per camera and
episode by :func:`draw_colour`):
* colour: per channel gain U(0.94, 1.06) x brightness U(0.92, 1.08), then white pixel noise sigma 2 grey levels;
* depth: white noise sigma_z = s0 + s2 z^2 (front 1.0 mm + 2.5 mm/m^2, wrist 0.2 mm + 1.5 mm/m^2), quantised (front
  1 mm, wrist 0.1 mm); invalid (0) outside the valid range (front 0.30-3 m, wrist 0.07-0.50 m), on depth
  discontinuities (a 4-neighbour more than 3 % of the depth away) and on 2 % of the pixels at random.

Runtime dependencies: mujoco + numpy only.
"""

import math

import mujoco
import numpy as np

from .control import mat_to_quat, quat_to_mat

__all__ = ["CAMERA_SPECS", "SENSOR", "lookat_quat", "build_cameras", "CameraRig", "draw_colour", "apply_sensor_model"]

CAMERA_SPECS = {
    # above and between the robot bases, looking over the drive (the drive's middle, E (0, 0.11) at the face)
    "front": {"size": (1280, 960), "fovy": 50.0, "pos": (0.05, 0.0, 0.80), "lookat": (0.55, 0.0, 0.41), "up": (0.0, 0.0, 1.0),
              "parent": "world", "kind": "front"},
    "left_wrist": {"size": (640, 480), "fovy": 58.0, "parent": "L/hand", "offset": (-0.058, 0.0, -0.062), "ahead": 0.15,
                   "up": (-1.0, 0.0, 0.0), "kind": "wrist"},
    "right_wrist": {"size": (640, 480), "fovy": 58.0, "parent": "R/hand", "offset": (-0.058, 0.0, -0.062), "ahead": 0.15,
                    "up": (-1.0, 0.0, 0.0), "kind": "wrist"},
}

SENSOR = {
    "front": {"sigma0_m": 0.0010, "sigma2_per_m": 0.0025, "quant_m": 0.001, "valid_m": (0.30, 3.0)},
    "wrist": {"sigma0_m": 0.0002, "sigma2_per_m": 0.0015, "quant_m": 0.0001, "valid_m": (0.07, 0.50)},
    "dropout": 0.02, "edge_rel": 0.03,
    "rgb_sigma": 2.0, "gain": (0.94, 1.06), "brightness": (0.92, 1.08),
}


def lookat_quat(pos, target, up=(0.0, 0.0, 1.0)):
    """wxyz quaternion of a MuJoCo camera at pos looking at target with `up` as the image-up hint."""
    pos = np.asarray(pos, float)
    f = np.asarray(target, float) - pos
    f /= np.linalg.norm(f)
    up = np.asarray(up, float)
    r = np.cross(f, up)
    if np.linalg.norm(r) < 1e-9:
        raise ValueError("up hint parallel to the viewing direction")
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    R = np.column_stack([r, u, -f])
    return mat_to_quat(R)


def build_cameras(spec, tcp_local, specs=CAMERA_SPECS, with_arms=True):
    """Add the cameras to the cell spec. Returns {name: {size, fovy, parent, pos, quat}}."""
    out = {}
    for name, c in specs.items():
        if c["parent"] == "world":
            body = spec.worldbody
            pos = np.asarray(c["pos"], float)
            quat = lookat_quat(pos, c["lookat"], c["up"])
        else:
            if not with_arms:
                continue
            body = spec.body(c["parent"])
            tcp = np.asarray(tcp_local, float)
            pos = tcp + np.asarray(c["offset"], float)
            target = tcp + np.array([0.0, 0.0, c["ahead"]])
            quat = lookat_quat(pos, target, c["up"])
        body.add_camera(name=name, pos=pos.tolist(), quat=quat.tolist(), fovy=float(c["fovy"]))
        out[name] = {"size": tuple(c["size"]), "fovy": float(c["fovy"]), "parent": c["parent"], "pos": tuple(pos.tolist()),
                     "quat": tuple(quat.tolist()), "kind": c["kind"]}
    return out


def draw_colour(rng, names):
    """The per-episode colour of each camera: {name: (channel gains (3,), brightness)}."""
    out = {}
    for name in names:
        g = rng.uniform(SENSOR["gain"][0], SENSOR["gain"][1], 3)
        b = float(rng.uniform(SENSOR["brightness"][0], SENSOR["brightness"][1]))
        out[name] = (g, b)
    return out


def apply_sensor_model(kind, rgb, depth, rng, colour):
    """(rgb uint8, depth uint16 in units of SENSOR[kind]['quant_m'], 0 = invalid) from a clean render."""
    g, b = colour
    img = rgb.astype(np.float32) * (np.asarray(g, np.float32) * np.float32(b))[None, None, :]
    img = img + rng.normal(0.0, SENSOR["rgb_sigma"], img.shape).astype(np.float32)
    rgb_out = np.clip(np.round(img), 0, 255).astype(np.uint8)
    S = SENSOR[kind]
    z = depth.astype(np.float64)
    finite = np.isfinite(z)
    zc = np.where(finite, z, 0.0)
    # depth discontinuities: a 4-neighbour more than edge_rel x depth away (on the clean depth)
    edge = np.zeros(z.shape, bool)
    thr = SENSOR["edge_rel"] * zc
    dv = np.abs(np.diff(zc, axis=0))
    dh = np.abs(np.diff(zc, axis=1))
    edge[:-1, :] |= dv > thr[:-1, :]
    edge[1:, :] |= dv > thr[1:, :]
    edge[:, :-1] |= dh > thr[:, :-1]
    edge[:, 1:] |= dh > thr[:, 1:]
    sig = S["sigma0_m"] + S["sigma2_per_m"] * zc * zc
    zn = zc + rng.normal(0.0, 1.0, z.shape) * sig
    q = np.round(zn / S["quant_m"])
    lo, hi = S["valid_m"]
    invalid = (~finite) | (zc < lo) | (zc > hi) | edge | (rng.random(z.shape) < SENSOR["dropout"])
    q = np.clip(q, 0, 65535).astype(np.uint16)
    q[invalid] = 0
    return rgb_out, q


class CameraRig(object):
    """Offscreen RGB + depth rendering of the named cameras with pinhole calibration.

    sizes: {camera name: (width, height)}; one mujoco.Renderer per distinct size.
    The model's visual.global_.offwidth/offheight must be >= every size.
    """

    def __init__(self, model, sizes=None):
        self.model = model
        self.sizes = dict(sizes) if sizes is not None else {k: v["size"] for k, v in CAMERA_SPECS.items()}
        self._renderers = {}
        self._option = mujoco.MjvOption()
        self._option.geomgroup[3] = 0   # collision-only geoms (the belt's contact spheres, the Panda's collision hulls)
        self._option.geomgroup[4] = 0   # marker sites

    def _renderer(self, name):
        w, h = self.sizes[name]
        key = (int(w), int(h))
        r = self._renderers.get(key)
        if r is None:
            r = mujoco.Renderer(self.model, height=key[1], width=key[0])
            self._renderers[key] = r
        return r

    def render(self, data, name):
        """(rgb uint8 HxWx3, depth float32 HxW planar z-depth in metres) of camera `name` at the data state."""
        r = self._renderer(name)
        r.disable_depth_rendering()
        r.update_scene(data, camera=name, scene_option=self._option)
        rgb = r.render().copy()
        r.enable_depth_rendering()
        depth = r.render().astype(np.float32)
        r.disable_depth_rendering()
        return rgb, depth

    def intrinsics(self, name):
        """dict(fx, fy, cx, cy, width, height) for integer pixel indices (see module docstring)."""
        w, h = self.sizes[name]
        fovy = math.radians(float(self.model.cam_fovy[self.model.camera(name).id]))
        fy = 0.5 * h / math.tan(0.5 * fovy)
        return {"fx": fy, "fy": fy, "cx": 0.5 * w - 0.5, "cy": 0.5 * h - 0.5, "width": int(w), "height": int(h)}

    def extrinsics(self, data, name):
        """4x4 T_world_cam in the OpenCV convention (x right, y down, z forward) at the data state."""
        cid = self.model.camera(name).id
        R_mj = data.cam_xmat[cid].reshape(3, 3)
        R_cv = R_mj @ np.diag([1.0, -1.0, -1.0])
        T = np.eye(4)
        T[:3, :3] = R_cv
        T[:3, 3] = data.cam_xpos[cid]
        return T

    def backproject(self, name, u, v, z, T_world_cam):
        """World points of pixels (u, v) at planar depths z through T_world_cam (arrays broadcast)."""
        k = self.intrinsics(name)
        u = np.asarray(u, float); v = np.asarray(v, float); z = np.asarray(z, float)
        x = (u - k["cx"]) * z / k["fx"]
        y = (v - k["cy"]) * z / k["fy"]
        P = np.stack([x, y, z], axis=-1)
        return P @ T_world_cam[:3, :3].T + T_world_cam[:3, 3]

    def project(self, name, data, point_world, T=None):
        """(u, v, z) of a world point in camera `name` (float pixel indices, planar depth)."""
        T = self.extrinsics(data, name) if T is None else T
        R = T[:3, :3]
        pc = R.T @ (np.asarray(point_world, float) - T[:3, 3])
        k = self.intrinsics(name)
        u = k["cx"] + k["fx"] * pc[0] / pc[2]
        v = k["cy"] + k["fy"] * pc[1] / pc[2]
        return float(u), float(v), float(pc[2])

    def describe(self, specs=None):
        """The public calibration: per camera the size, the exact intrinsics, and for the fixed camera its exact pose
        (OpenCV convention, world frame); for the wrist cameras the body they ride on and their exact mount (OpenCV
        pose in that body's frame)."""
        specs = specs or CAMERA_SPECS
        out = {}
        for name in self.sizes:
            c = specs[name]
            k = self.intrinsics(name)
            cid = self.model.camera(name).id
            R_mj = quat_to_mat(np.asarray(self.model.cam_quat[cid], float))
            T = np.eye(4)
            T[:3, :3] = R_mj @ np.diag([1.0, -1.0, -1.0])
            T[:3, 3] = np.asarray(self.model.cam_pos[cid], float)
            S = SENSOR[c["kind"]]
            e = {"width": k["width"], "height": k["height"], "fx": k["fx"], "fy": k["fy"], "cx": k["cx"], "cy": k["cy"],
                 "fovy_deg": float(self.model.cam_fovy[cid]), "depth": "planar z along the optical axis",
                 "depth_unit_m": S["quant_m"], "depth_valid_m": list(S["valid_m"]),
                 "depth_noise_m": "%.4f + %.4f z^2 (z in m)" % (S["sigma0_m"], S["sigma2_per_m"]),
                 "convention": "OpenCV: x right, y down, z forward; pixel (u, v) = (column, row), row 0 at the top"}
            if c["parent"] == "world":
                e["mount"] = "fixed"
                e["T_world_cam"] = T.tolist()
            else:
                e["mount"] = c["parent"]
                e["T_body_cam"] = T.tolist()
                e["note"] = "T_world_cam = T_world_body(%s) @ T_body_cam; the body pose follows from the arm's joint angles" % c["parent"]
            out[name] = e
        return out

    def close(self):
        """Free the GL contexts."""
        for r in self._renderers.values():
            try:
                r.close()
            except Exception:
                pass
        self._renderers = {}
