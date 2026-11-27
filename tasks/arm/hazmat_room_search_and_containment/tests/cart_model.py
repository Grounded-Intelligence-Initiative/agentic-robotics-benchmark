"""The team's hazardous-materials cart, as the range simulates it: a Franka Panda with the hook tool
(arm_model.py) on the deck of robosuite 1.5.2's Omron mobile base (the base's lift column
taken off, so that the arm reaches the floor), with a depth camera on a mast.

This file is the cart and nothing else: its model, how an action reaches its drives, what it
reports, how its camera's pictures are made, and an empty range (`CartRange`: the cart on
open ground among a few plain blocks) that speaks the same observations and actions as a live
episode. The grader simulates the cart with exactly this file; what the cart meets in a live
episode (the lab, its furniture and drawers, the packages, the trolley, the store and its
vessels, the latch closing on anything, what a package is lost by) is not in it.

Frames. World: z up, metres, the ground is z = 0. Cart: origin on the ground at the centre
the cart turns about, x forward, y left. Arm: arm_model.py's own frame, its origin at
ARM_MOUNT in the cart's frame, its axes the cart's.
"""

import base64
import math
import os
import zlib

import mujoco
import numpy as np

from arm_model import (  # noqa: F401
    CONTROL_DT, HOME_Q, N_JOINTS, PHYSICS_DT, SUBSTEPS, add_arm, draw_bias, drive_target,
    finish_contacts, new_world, read_wrench, robosuite_assets, slew)

# --- the cart -----------------------------------------------------------------------------
# The Omron base's body collides as a box of BASE_HALF around BASE_BOX (cart's frame). The
# arm stands on its deck at ARM_MOUNT; the camera is on a mast at CAM_MOUNT.
BASE_BOX = np.array([0.01, 0.0, 0.192])
BASE_HALF = np.array([0.35, 0.25, 0.19])
BASE_MASS = 62.0
ARM_MOUNT = np.array([0.21, 0.0, 0.385])
BASE_JOINTS = ("cart_joint_mobile_forward", "cart_joint_mobile_side", "cart_joint_mobile_yaw")
BASE_V_MAX = np.array([0.5, 0.5, 1.0])       # m/s, m/s, rad/s: the largest commanded velocity
BASE_A_MAX = np.array([0.6, 0.6, 2.0])       # m/s^2, m/s^2, rad/s^2: how fast it may change
BASE_KP = np.array([40000.0, 40000.0, 12000.0])   # the base's position servos (x, y, yaw)
BASE_KV = np.array([3800.0, 3800.0, 700.0])
BASE_LEASH = np.array([0.06, 0.06, 0.15])    # the servos' target never leads the cart by more
TRAVEL_Q = HOME_Q.copy()                     # the arm at the start: the ready pose turned to
TRAVEL_Q[0] = -1.45                          # the right, out of the camera's view and inside a
#                                              disc of 0.45 m about the cart's turning centre

# --- the camera ---------------------------------------------------------------------------
CAM_MOUNT = np.array([0.33, 0.21, 1.42])     # its centre, in the cart's frame
CAM_PITCH = math.radians(40.0)               # it looks forward and this far down
CAM_W, CAM_H, CAM_FOVY = 240, 180, 62.0      # pixels, pixels, vertical field of view (degrees)
CAM_PERIOD = 10                              # at most one picture in 10 control steps (5 Hz)
CAM_RANGE = 8.0                              # m: nothing farther than this is in a picture
CAM_LIGHT = np.array([0.25, 0.15, 1.0]) / np.linalg.norm([0.25, 0.15, 1.0])
CAM_GROUPS = np.array([0, 1, 0, 1, 0, 0], np.uint8)   # geom groups a picture shows: 1 the
#                                              scene, 3 the cart's and the arm's own shapes


# ------------------------------------------------------------------------------------------
# the model
# ------------------------------------------------------------------------------------------
def add_box(body, name, half, pos, rgba, contype, conaffinity, mass=None, friction=None, group=1):
    """A box geom. Collision classes on the range: 1 loose things, 2 the cart and the arm,
    4 the ground and everything fixed; a geom collides with another when its `contype` and
    the other's `conaffinity` share a bit."""
    g = body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=[float(v) for v in half],
                      pos=[float(v) for v in pos], rgba=list(rgba))
    g.contype, g.conaffinity, g.group = contype, conaffinity, group
    if mass is not None:
        g.mass = mass
    if friction is not None:
        g.friction = [float(friction), 0.01, 0.0005]
    return g


def add_cart(w):
    """Put the cart into a world: robosuite's Omron mobile base (its origin moved to the
    centre it turns about, its lift column taken off, its drives replaced by position servos
    on its three joints), the mast with the camera, and the arm on the base's deck. Call it
    after everything else is in the world: the geoms present before the call are the scene."""
    scene = set(g.name for g in w.geoms)
    w.geom("ground").group = 1                   # the camera sees the ground
    base = mujoco.MjSpec.from_file(os.path.join(robosuite_assets(), "bases", "omron_mobile_base.xml"))
    for a in list(base.actuators):
        base.delete(a)
    for j in list(base.joints):
        if j.name == "joint_torso_height":
            base.delete(j)
        else:
            j.frictionloss, j.damping = 0.0, [0.0, 0.0, 0.0]
    for g in list(base.geoms):
        if g.type == mujoco.mjtGeom.mjGEOM_CYLINDER:
            base.delete(g)                        # the lift column: the arm stands on the deck
    for g in base.geoms:
        if g.type == mujoco.mjtGeom.mjGEOM_MESH or g.group == 1:
            g.contype, g.conaffinity = 0, 0
            g.mass = 0.0 if g.type == mujoco.mjtGeom.mjGEOM_MESH else 0.01
        else:
            g.contype, g.conaffinity = 2, 5
            g.mass = BASE_MASS
    # the base body's own origin is the column's axis, ARM_MOUNT[0] ahead of the turning centre
    w.worldbody.add_frame(pos=[float(ARM_MOUNT[0]), 0, 0]).attach_body(base.body("base"), "cart_", "")
    cart = w.body("cart_base")
    x, y, z = CAM_MOUNT[0] - ARM_MOUNT[0], CAM_MOUNT[1], CAM_MOUNT[2]
    mast = cart.add_geom(name="cart_mast", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                         fromto=[x - 0.04, y, 0.38, x - 0.04, y, z - 0.02], size=[0.012, 0, 0],
                         rgba=[0.2, 0.22, 0.25, 1])
    mast.mass, mast.contype, mast.conaffinity = 0.5, 0, 0
    add_box(cart, "cart_camera", [0.03, 0.045, 0.02], [x - 0.04, y, z], [0.1, 0.1, 0.12, 1], 0, 0, mass=0.2)
    c, s = math.cos(CAM_PITCH), math.sin(CAM_PITCH)
    # camera frame: x right, y up, looking along -z; here: forward along the cart's x, pitched down
    cart.add_camera(name="mast", pos=[float(x), float(y), float(z)], fovy=CAM_FOVY, xyaxes=[0, -1, 0, s, 0, c])
    add_arm(w, parent=cart, pos=[0.0, 0.0, float(ARM_MOUNT[2])])
    # The camera sees the cart's and the arm's collision shapes (group 3), which is what they
    # are in every contact; their detailed meshes (group 2) are for people to look at.
    for g in w.geoms:
        if g.name not in scene or not g.name:
            if g.contype:
                g.group = 3
                if g.type == mujoco.mjtGeom.mjGEOM_MESH:
                    g.rgba = [0.85, 0.85, 0.85, 1]
            else:
                g.group = 2
    for i, jn in enumerate(BASE_JOINTS):
        w.add_actuator(name="b{}".format(i), target=jn, trntype=mujoco.mjtTrn.mjTRN_JOINT,
                       gaintype=mujoco.mjtGain.mjGAIN_FIXED, gainprm=[float(BASE_KP[i])] + [0.0] * 9,
                       biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                       biasprm=[0.0, -float(BASE_KP[i]), -float(BASE_KV[i])] + [0.0] * 7)
    finish_contacts(w)                           # every colliding geom, the cart's included


# ------------------------------------------------------------------------------------------
# the camera's pictures
# ------------------------------------------------------------------------------------------
def _camera_rays():
    """Unit vectors through the pixel centres in the camera's own frame (x right, y up,
    looking along -z), row by row from the top left, and each one's share along the axis."""
    f = (CAM_H / 2.0) / math.tan(math.radians(CAM_FOVY) / 2.0)
    u, v = np.meshgrid(np.arange(CAM_W) + 0.5 - CAM_W / 2.0, np.arange(CAM_H) + 0.5 - CAM_H / 2.0)
    rays = np.stack([u / f, -v / f, -np.ones_like(u)], axis=-1).reshape(-1, 3)
    norm = np.linalg.norm(rays, axis=1)
    return rays / norm[:, None], 1.0 / norm


CAM_RAYS, CAM_AXIAL = _camera_rays()


def encode(array):
    """An image as JSON: zlib-compressed bytes in base64, with its shape and type."""
    a = np.ascontiguousarray(array)
    return dict(shape=list(a.shape), dtype=str(a.dtype),
                zlib_b64=base64.b64encode(zlib.compress(a.tobytes(), 1)).decode())


def decode(d):
    return np.frombuffer(zlib.decompress(base64.b64decode(d["zlib_b64"])),
                         dtype=d["dtype"]).reshape(d["shape"])


def geom_colours(model):
    """Every geom's colour: its material's, if it has one, else its own."""
    rgb = model.geom_rgba[:, :3].copy()
    has = model.geom_matid >= 0
    rgb[has] = model.mat_rgba[model.geom_matid[has], :3]
    return rgb


def take_picture(model, data, camera, colours, stamp_s):
    """The camera's colour and depth pictures of the present state.

    The pictures are ray-cast from the model (one ray through the centre of every pixel,
    `mj_ray`, against the geoms of CAM_GROUPS), not rasterised: they are the same on every
    machine and need no display or graphics library. Depth is the distance along the
    camera's axis in millimetres (uint16), 0 where a ray meets nothing within CAM_RANGE.
    Colour (uint8) is the hit surface's own colour, shaded by the angle between the surface
    (its normal taken from the depth picture's own slopes) and a fixed light; a surface's
    three colour channels keep their ratios whatever the shade."""
    R = data.cam_xmat[camera].reshape(3, 3)
    origin = data.cam_xpos[camera].copy()
    vec = np.ascontiguousarray(CAM_RAYS @ R.T)
    n = vec.shape[0]
    geom = np.empty(n, np.int32)
    dist = np.empty(n)
    hit_geom = np.zeros(1, np.int32)
    ray = mujoco.mj_ray
    for k in range(n):
        dist[k] = ray(model, data, origin, vec[k], CAM_GROUPS, 1, -1, hit_geom)
        geom[k] = hit_geom[0]
    hit = (geom >= 0) & (dist <= CAM_RANGE)
    depth = np.where(hit, dist * CAM_AXIAL, 0.0)
    # surface normals from the slopes of the points seen
    P = (vec * np.where(hit, dist, CAM_RANGE)[:, None]).reshape(CAM_H, CAM_W, 3)
    du = np.empty_like(P)
    dv = np.empty_like(P)
    du[:, 1:-1], du[:, 0], du[:, -1] = P[:, 2:] - P[:, :-2], P[:, 1] - P[:, 0], P[:, -1] - P[:, -2]
    dv[1:-1], dv[0], dv[-1] = P[2:] - P[:-2], P[1] - P[0], P[-1] - P[-2]
    normal = np.cross(dv, du).reshape(-1, 3)
    normal /= np.maximum(np.linalg.norm(normal, axis=1), 1e-12)[:, None]
    normal *= -np.sign(np.sum(normal * vec, axis=1))[:, None]          # towards the camera
    shade = 0.45 + 0.55 * np.clip(normal @ CAM_LIGHT, 0.0, 1.0)
    rgb = np.where(hit[:, None], colours[np.maximum(geom, 0)] * shade[:, None], 0.0)
    rgb = np.round(255.0 * rgb).astype(np.uint8).reshape(CAM_H, CAM_W, 3)
    mm = np.round(depth * 1000.0).astype(np.uint16).reshape(CAM_H, CAM_W)
    return dict(stamp_s=round(float(stamp_s), 4), rgb=encode(rgb), depth_mm=encode(mm))


# ------------------------------------------------------------------------------------------
# the cart inside a compiled model
# ------------------------------------------------------------------------------------------
class Cart(object):
    """Where the cart's joints and drives are in a model, how an action reaches them, and
    what the cart reports. A range owns the model and the stepping; it calls `command` once
    per control step, `drive(i)` before each of the step's SUBSTEPS physics steps, and `look`
    after them."""

    def __init__(self, model, data, pose, camera=True):
        m, d = model, data
        self.model, self.data, self.camera = m, d, camera
        name = lambda kind, n: mujoco.mj_name2id(m, kind, n)  # noqa: E731
        joint = lambda n: name(mujoco.mjtObj.mjOBJ_JOINT, n)  # noqa: E731
        self.base_adr = [int(m.jnt_qposadr[joint(j)]) for j in BASE_JOINTS]
        self.base_dof = [int(m.jnt_dofadr[joint(j)]) for j in BASE_JOINTS]
        self.arm_adr = [int(m.jnt_qposadr[joint("joint{}".format(i + 1))]) for i in range(N_JOINTS)]
        self.arm_dof = [int(m.jnt_dofadr[joint("joint{}".format(i + 1))]) for i in range(N_JOINTS)]
        self.arm_ctrl = [name(mujoco.mjtObj.mjOBJ_ACTUATOR, "a{}".format(i + 1)) for i in range(N_JOINTS)]
        self.base_ctrl = [name(mujoco.mjtObj.mjOBJ_ACTUATOR, "b{}".format(i)) for i in range(3)]
        self.cam = name(mujoco.mjtObj.mjOBJ_CAMERA, "mast")
        self.latch = name(mujoco.mjtObj.mjOBJ_SITE, "latch")
        pose = np.array(pose, dtype=float)
        d.qpos[self.base_adr] = pose
        d.qpos[self.arm_adr] = TRAVEL_Q
        d.ctrl[self.base_ctrl] = pose
        d.ctrl[self.arm_ctrl] = TRAVEL_Q
        self.target = TRAVEL_Q.copy()
        self.previous = TRAVEL_Q.copy()
        self.base_target = pose.copy()
        self.base_previous = pose.copy()
        self.base_v = np.zeros(3)
        self.colours = geom_colours(m)
        self.picture = None
        self.last_look = 0

    def command(self, action):
        """Take in an action: {"base": [vx, vy, wz] in the cart's frame, "q": seven joint
        targets, "latch": 0 or 1, "look": 0 or 1}. The arm's targets are slewed as in
        arm_model.py. The base is told a velocity: it is clipped to BASE_V_MAX, may change
        by no more than BASE_A_MAX per second, and moves the base servos' target pose (the
        velocity's x and y turned by the target's yaw); the target never leads the cart's
        true pose by more than BASE_LEASH, so a blocked cart does not wind it up."""
        d = self.data
        self.previous = self.target
        self.target = slew(self.target, action["q"])
        v_cmd = np.clip(np.asarray(action.get("base", [0, 0, 0]), dtype=float), -BASE_V_MAX, BASE_V_MAX)
        self.base_v = self.base_v + np.clip(v_cmd - self.base_v, -BASE_A_MAX * CONTROL_DT,
                                            BASE_A_MAX * CONTROL_DT)
        yaw = self.base_target[2]
        c, s = math.cos(yaw), math.sin(yaw)
        self.base_previous = self.base_target
        new_target = self.base_target + np.array([c * self.base_v[0] - s * self.base_v[1],
                                                  s * self.base_v[0] + c * self.base_v[1],
                                                  self.base_v[2]]) * CONTROL_DT
        actual = d.qpos[self.base_adr]
        self.base_target = actual + np.clip(new_target - actual, -BASE_LEASH, BASE_LEASH)

    def drive(self, substep):
        """Set the servos' set-points for one physics step: both the arm's and the base's
        are interpolated from the previous control step's targets to this one's."""
        self.data.ctrl[self.arm_ctrl] = drive_target(self.previous, self.target, substep)
        self.data.ctrl[self.base_ctrl] = drive_target(self.base_previous, self.base_target, substep)

    def look(self, action, t):
        """With `look` 1 in the action the camera takes a picture at the end of the control
        step, if its last one is at least CAM_PERIOD steps old; the observation that follows
        carries it, and the cart's pose in that observation is the pose it was taken from."""
        self.picture = None
        if self.camera and int(action.get("look", 0)) and t - self.last_look >= CAM_PERIOD:
            self.picture = take_picture(self.model, self.data, self.cam, self.colours, t * CONTROL_DT)
            self.last_look = t

    def base_pose(self):
        """x, y of the centre the cart turns about, and its yaw."""
        return self.data.qpos[self.base_adr].copy()

    def observation(self, t, site, bias, noise_seed, latched, q_offset=None, base_offset=None):
        """An observation as plain JSON. `site`: the episode's limits, the vessels' places and the number of packages."""
        d = self.data
        q = d.qpos[self.arm_adr].copy()
        base = self.base_pose()
        if q_offset is not None:
            q = q + np.asarray(q_offset, dtype=float)
        if base_offset is not None:
            base = base + np.asarray(base_offset, dtype=float)
        wrench = read_wrench(d, bias, noise_seed, t)
        return dict(
            t_s=round(t * CONTROL_DT, 4),
            countdown_s=site["countdown_s"],
            tilt_limit_deg=site["tilt_limit_deg"],
            shock_limit=site["shock_limit"],
            base=[round(float(v), 5) for v in base],
            base_vel=[round(float(v), 5) for v in d.qvel[self.base_dof]],
            q=[round(float(v), 6) for v in q],
            qd=[round(float(v), 5) for v in d.qvel[self.arm_dof]],
            wrench=[round(float(v), 4) for v in wrench],
            latched=int(latched),
            camera=self.picture,
            vessels=[[float(v[0]), float(v[1])] for v in site["vessels"]],
            packages=int(site["packages"]),
        )


class CartRange(object):
    """The empty range: the cart on open ground among a few plain blocks. `reset(seed)` and
    `step(action)` take and return what a live episode's do; the latch closes on nothing
    and nothing can be lost. It is for checking a controller's wire, timing, driving, kinematics
    and reading of pictures against the true cart."""

    max_steps = int(round(60.0 / CONTROL_DT))
    BLOCKS = (                                   # centre x, y; half sizes x, y, z; colour
        (2.0, 0.6, 0.30, 0.20, 0.375, [0.55, 0.40, 0.27]),
        (2.4, -0.9, 0.20, 0.35, 0.60, [0.30, 0.42, 0.55]),
        (0.6, 1.8, 0.45, 0.25, 0.25, [0.45, 0.52, 0.35]),
        (-1.2, -1.5, 0.15, 0.15, 0.90, [0.60, 0.58, 0.52]),
    )

    def __init__(self, camera=True):
        self.camera = camera

    def reset(self, seed):
        rng = np.random.default_rng(int(seed))
        self.site = dict(countdown_s=60.0, tilt_limit_deg=5.0, shock_limit=2.0, vessels=[[-9.3, 1.97], [-7.6, 1.97], [-9.3, -1.97], [-7.6, -1.97]], packages=2)
        self.bias = draw_bias(rng)
        self.noise_seed = int(rng.integers(0, 2 ** 31))
        w = new_world()
        for k, (x, y, hx, hy, hz, rgb) in enumerate(self.BLOCKS):
            add_box(w.worldbody, "block{}".format(k), [hx, hy, hz], [x, y, hz], rgb + [1.0], 4, 3)
        add_cart(w)
        self.model = w.compile()
        self.data = mujoco.MjData(self.model)
        self.cart = Cart(self.model, self.data, [0.0, 0.0, 0.0], camera=self.camera)
        mujoco.mj_forward(self.model, self.data)
        self.t = 0
        if self.camera:
            self.cart.picture = take_picture(self.model, self.data, self.cart.cam, self.cart.colours, 0.0)
        return self.observation()

    def step(self, action):
        self.cart.command(action)
        for i in range(SUBSTEPS):
            self.cart.drive(i)
            mujoco.mj_step(self.model, self.data)
        self.t += 1
        self.cart.look(action, self.t)
        return self.observation(), self.t >= self.max_steps

    def observation(self, q_offset=None, base_offset=None):
        return self.cart.observation(self.t, self.site, self.bias, self.noise_seed, 0,
                                     q_offset=q_offset, base_offset=base_offset)
