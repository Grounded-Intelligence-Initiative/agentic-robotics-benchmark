"""G1/Dex3 tabletop pick-and-place with MuJoCo dynamics and substep contact scoring."""
from collections import OrderedDict
from dataclasses import dataclass, replace
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .cameras import camera_calibration, default_camera_rig
from .decoder import DEFAULT_Q, KP, KD, EFFORT_LIMIT


LEG = ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
ARM = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
SIDES = ("left", "right")
BODY_JOINTS = tuple(f"{s}_{j}_joint" for s in SIDES for j in LEG) + (
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"
) + tuple(f"{s}_{j}_joint" for s in SIDES for j in ARM)
FINGERS = ("thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1")
FINGER_JOINTS = tuple(tuple(f"{s}_hand_{j}_joint" for j in FINGERS) for s in SIDES)
PHYSICS_DT = .002
SUBSTEPS = 10


@dataclass(frozen=True)
class TaskConfig:
    shape: str
    size: tuple
    object_xy: tuple
    pad_xy: tuple
    mass: float
    friction: float = 1.0
    yaw: float = 0.0

    @property
    def half_height(self):
        return self.size[-1]

    @property
    def half_width(self):
        return self.size[0]


TASKS = {
    "box_to_pad": TaskConfig("box", (.026, .026, .060), (.295, -.18), (.265, -.34), .10),
    "cylinder_to_pad": TaskConfig("cylinder", (.024, .050), (.30, -.235), (.34, -.09), .11),
}
# Challenge IDs reserve disjoint yaw, object placement and dynamics ranges.
# The canonical IDs retain their original three-millimetre XY jitter.
TASKS.update({f"{task}_challenge": config for task, config in tuple(TASKS.items())})


def _uniform(seed, task, field, low, high):
    """A 53-bit uniform draw keyed by the uint64 episode seed and named field."""
    digest = hashlib.blake2b(
        (task + "/" + field).encode("ascii"),
        key=seed.to_bytes(8, "little"), person=b"g1sonic.scene.v1", digest_size=8,
    ).digest()
    fraction = (int.from_bytes(digest, "little") >> 11) * 2.0**-53
    return low + (high - low) * fraction


def challenge_config(task, seed):
    """Draw the original physical ranges with independent keyed field domains."""
    config = TASKS[task]
    def draw(field, low, high):
        return _uniform(seed, task, field, low, high)

    x_sign = -1. if draw("object-x-sign", 0., 1.) < .5 else 1.
    x = config.object_xy[0] + x_sign * draw("object-x-offset", .023, .048)
    y = config.object_xy[1] + draw("object-y-offset", -.03, .03)
    direction = -1. if config.shape == "box" else 1.
    goal = (draw("pad-x", .272, .355), y + direction * draw("pad-y-offset", .143, .200))
    yaw_sign = -1. if draw("yaw-sign", 0., 1.) < .5 else 1.
    yaw = yaw_sign * draw("yaw", np.deg2rad(12.), np.deg2rad(42.))
    return replace(config, object_xy=(x, y), pad_xy=goal, yaw=yaw,
                   mass=config.mass * draw("mass-scale", .80, 1.45), friction=draw("friction", .60, 1.15))


TABLE_TOP = .730
PAD_TOP = .745
PAD_HALF_WIDTH = .069


def finger_targets(hand):
    """Return the left/right mirrored Dex3 aperture targets in FINGER_JOINTS order."""
    hand = np.asarray(hand, dtype=float)
    if hand.shape != (2,) or not np.all(np.isfinite(hand)) or np.any((hand < 0) | (hand > 1)):
        raise ValueError("hand requires (left, right) closure fractions in [0,1]")
    # Thumb opposition then finger curling. The signs mirror the Dex3 URDF axes.
    right = np.array([0., -.65, -1., .75, .65, .75, .65])
    return np.concatenate((-right * hand[0], right * hand[1]))


def _vec(values):
    return " ".join(str(float(x)) for x in values)


def _build_model(config, robot_root):
    """Compile a public robot and physical arena; rollouts read the compiled model."""
    root = ET.parse(robot_root / "g1_with_hands.xml").getroot()
    root.find("compiler").set("meshdir", str(robot_root / "assets"))
    opt = root.find("option")
    opt.set("timestep", str(PHYSICS_DT))
    opt.set("cone", "elliptic")
    root.remove(root.find("keyframe"))
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="960")
    ET.SubElement(visual, "headlight", ambient=".45 .45 .45", diffuse=".65 .65 .65", specular=".08 .08 .08")
    asset = root.find("asset")
    ET.SubElement(asset, "texture", name="sky", type="skybox", builtin="gradient",
                  rgb1=".64 .72 .78", rgb2=".32 .40 .45", width="128", height="768")
    ET.SubElement(asset, "texture", name="floor_tiles", type="2d", builtin="checker",
                  rgb1=".19 .22 .25", rgb2=".24 .27 .30", width="256", height="256")
    ET.SubElement(asset, "material", name="floor_mat", texture="floor_tiles", texrepeat="8 8")
    world = root.find("worldbody")
    ET.SubElement(world, "geom", name="floor", type="plane", size="4 4 .05", material="floor_mat")
    ET.SubElement(world, "geom", name="table", type="box", pos=f".457 -.17 {TABLE_TOP-.018}",
                  size=".255 .36 .018", rgba=".55 .39 .25 1")
    for i, (x, y) in enumerate(((.24, -.48), (.67, -.48), (.24, .14), (.67, .14))):
        ET.SubElement(world, "geom", name=f"table_leg_{i}", type="box",
                      pos=_vec((x, y, (TABLE_TOP-.036)/2)), size=_vec((.018,.018,(TABLE_TOP-.036)/2)),
                      rgba=".34 .35 .37 1")
    ET.SubElement(world, "geom", name="goal_pad", type="box",
                  pos=_vec((*config.pad_xy, (TABLE_TOP+PAD_TOP)/2)),
                  size=_vec((PAD_HALF_WIDTH, PAD_HALF_WIDTH, (PAD_TOP-TABLE_TOP)/2)),
                  rgba=".16 .65 .47 1", friction=".85 .005 .0001")
    obj = ET.SubElement(world, "body", name="object", pos=_vec((*config.object_xy, TABLE_TOP+config.half_height+.0003)))
    ET.SubElement(obj, "freejoint", name="object_free")
    obj_geom = ET.SubElement(obj, "geom", name="object", type=config.shape, size=_vec(config.size),
                  mass=str(config.mass), rgba=".96 .49 .10 1", friction=f"{config.friction} .005 .0001", condim="4")
    if config.friction != 1.0:
        # MuJoCo's default equal-priority mixing takes the larger coefficient.
        # Set the held-out object's coefficient on both high- and low-friction trials.
        obj_geom.set("priority", "2")
    for side, y in zip(SIDES, (-.048, .048)):
        wrist = world.find(f".//body[@name='{side}_wrist_yaw_link']")
        ET.SubElement(wrist, "site", name=f"{side}_grasp", pos=_vec((.115,y,0)),
                      size=".004", group="5", rgba="0 0 1 1")
    actuator = root.find("actuator")
    for i, name in enumerate(BODY_JOINTS):
        a = actuator.find(f"position[@joint='{name}']")
        a.set("kp", str(float(KP[i])))
        a.set("kv", str(float(KD[i])))
        a.set("forcerange", _vec((-EFFORT_LIMIT[i], EFFORT_LIMIT[i])))
    for side_names in FINGER_JOINTS:
        for name in side_names:
            a = actuator.find(f"position[@joint='{name}']")
            a.set("kp", "4")
            a.set("kv", ".15")
            max_torque = 2.45 if "thumb_0" in name else 1.4
            a.set("forcerange", _vec((-max_torque, max_torque)))
    # The source leaves the dampratio in the inherited class. Use explicit kv above.
    root.find("default/default/position").attrib.pop("dampratio")
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


@dataclass
class ContactSample:
    hand_force_z: float = 0.
    support_force_z: float = 0.
    opposed: bool = False
    hand_contact: bool = False
    pad_force_z: float = 0.


class TaskScore:
    """Qualify a load-bearing lift followed by a released, stable goal placement."""
    def __init__(self, initial_z, config):
        self.initial_z = initial_z
        self.config = config
        self.lift_time = self.release_time = self.max_lift = self.max_qualified_hold = 0.
        self.lifted = self.placed = self.contacted = False
        self.horizontal_error = self.vertical_error = self.speed = 0.
        self.contacts = ContactSample()
        self.upright = True

    def update(self, *, dt, object_pos, object_velocity, object_rotation, goal_pos, contacts, upright):
        # The first terminal substep owns the verdict reported at the next 50 Hz boundary.
        if self.placed or not self.upright:
            return
        self.contacts = contacts
        self.upright = bool(upright)
        lift = float(object_pos[2] - self.initial_z)
        self.max_lift = max(self.max_lift, lift)
        self.contacted |= bool(contacts.hand_contact)
        weight = self.config.mass * 9.81
        bearing = (contacts.opposed and contacts.hand_force_z >= .5*weight
                   and contacts.support_force_z <= .05*weight and lift >= .04 and upright)
        self.lift_time = self.lift_time + dt if bearing else 0.
        self.max_qualified_hold = max(self.lift_time, self.max_qualified_hold)
        self.lifted |= self.lift_time >= .040 - 1e-10
        delta = object_pos - goal_pos
        self.horizontal_error = float(np.linalg.norm(delta[:2]))
        self.vertical_error = float(abs(delta[2]))
        self.speed = float(np.linalg.norm(object_velocity[:3]))
        if self.config.shape == "box":
            footprint = np.abs(object_rotation[:2]) @ self.config.size
        else:
            axis = np.abs(object_rotation[:2, 2])
            footprint = self.config.half_width*np.sqrt(np.maximum(0., 1.-axis**2)) + self.config.half_height*axis
        supported = (contacts.pad_force_z >= .5*weight and not contacts.hand_contact
                     and contacts.support_force_z - contacts.pad_force_z <= .05*weight
                     and np.all(np.abs(delta[:2]) + footprint <= PAD_HALF_WIDTH)
                     and abs(object_rotation[2,2]) >= .95
                     and self.vertical_error < .009 and self.speed < .08
                     and np.linalg.norm(object_velocity[3:]) < 1.2 and upright)
        self.release_time = self.release_time + dt if self.lifted and supported else 0.
        self.placed = self.release_time >= .250 - 1e-10

    def result(self):
        return dict(success=bool(self.placed), lifted=bool(self.lifted), placed=bool(self.placed),
                    contacted=bool(self.contacted), upright=self.upright, max_lift_m=self.max_lift,
                    goal_error_xy_m=self.horizontal_error, goal_error_z_m=self.vertical_error,
                    object_speed_m_s=self.speed, lift_hold_s=self.lift_time,
                    release_hold_s=self.release_time, max_lift_hold_s=self.max_qualified_hold, opposed=bool(self.contacts.opposed),
                    hand_contact=bool(self.contacts.hand_contact),
                    hand_load_z_n=float(self.contacts.hand_force_z),
                    support_load_z_n=float(self.contacts.support_force_z),
                    pad_load_z_n=float(self.contacts.pad_force_z))


class G1Task:
    def __init__(self, task, seed, robot_root):
        self.task = task
        self.seed = seed
        self.camera_rig = default_camera_rig()
        self.config = challenge_config(task, seed) if task.endswith("_challenge") else TASKS[task]
        m = _build_model(self.config, Path(robot_root))
        self.model = m
        self.data = d = mujoco.MjData(m)
        self.body_joint_ids = np.array([m.joint(name).id for name in BODY_JOINTS])
        finger_names = [name for group in FINGER_JOINTS for name in group]
        finger_ids = np.array([m.joint(name).id for name in finger_names])
        self.qadr = m.jnt_qposadr[self.body_joint_ids].copy()
        self.vadr = m.jnt_dofadr[self.body_joint_ids].copy()
        self.f_qadr = m.jnt_qposadr[finger_ids].copy()
        self.f_vadr = m.jnt_dofadr[finger_ids].copy()
        self.actuator_ids = np.array([m.actuator(name).id for name in BODY_JOINTS])
        self.f_actuator_ids = np.array([m.actuator(name).id for name in finger_names])
        self.pelvis = m.body("pelvis").id
        self.object_body = m.body("object").id
        self.object_geom = m.geom("object").id
        self.pad_geom = m.geom("goal_pad").id
        self.object_qadr = m.joint("object_free").qposadr[0]
        self.object_vadr = m.joint("object_free").dofadr[0]
        self.goal_pos = np.array([*self.config.pad_xy, PAD_TOP+self.config.half_height])
        self._digits = {}
        for geom in range(m.ngeom):
            body = m.body(m.geom_bodyid[geom]).name
            for side in SIDES:
                if body.startswith(side + "_hand_"):
                    self._digits[geom] = side, body.split("_")[2]
                elif body == side + "_wrist_yaw_link" and m.geom_dataid[geom] == m.mesh(side + "_hand_palm_link").id:
                    self._digits[geom] = side, "palm"
        d.qpos[self.qadr] = DEFAULT_Q
        d.ctrl[self.actuator_ids] = DEFAULT_Q
        if task.endswith("_challenge"):
            half_yaw = .5*self.config.yaw
            d.qpos[self.object_qadr+3:self.object_qadr+7] = (np.cos(half_yaw), 0, 0, np.sin(half_yaw))
        else:
            offsets = [_uniform(seed, task, field, -.003, .003) for field in ("jitter-x", "jitter-y")]
            d.qpos[self.object_qadr:self.object_qadr+2] += offsets
        mujoco.mj_forward(m, d)
        # Set the lowest native sole sphere just above the floor at default angles.
        feet = [g for g in range(m.ngeom) if m.geom_group[g] == 3
                and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE
                and "ankle_roll" in m.body(m.geom_bodyid[g]).name]
        floor_gap = min(d.geom_xpos[g,2]-m.geom_size[g,0] for g in feet)
        d.qpos[2] += .0002 - floor_gap
        mujoco.mj_forward(m, d)
        self.score = TaskScore(float(d.xpos[self.object_body,2]), self.config)
        self._sample(0., ContactSample(), d.qvel[self.object_vadr:self.object_vadr+6])
        self._renderers = OrderedDict()

    def contacts(self):
        m, d = self.model, self.data
        sample = ContactSample()
        force = np.zeros(6)
        normals = []
        for index, c in enumerate(d.contact):
            g1, g2 = int(c.geom1), int(c.geom2)
            if self.object_geom not in (g1,g2):
                continue
            mujoco.mj_contactForce(m, d, index, force)
            if force[0] <= 1e-4:
                continue
            other, sign = (g1, 1.) if g2 == self.object_geom else (g2, -1.)
            basis = c.frame.reshape(3,3)
            fz = float(sign * (basis.T @ force[:3])[2])
            digit = self._digits.get(other)
            if digit is not None:
                sample.hand_contact = True
                sample.hand_force_z += fz
                normal = sign * basis[0]
                sample.opposed |= any(digit[0] == prior[0] and digit[1] != prior[1]
                                      and np.dot(normal, n) < 0. for prior, n in normals)
                normals.append((digit, normal.copy()))
            else:
                sample.support_force_z += max(0., fz)
                if other == self.pad_geom:
                    sample.pad_force_z += fz
        return sample

    def _sample(self, dt, contacts, object_velocity):
        d = self.data
        p = self.pelvis
        self.score.update(dt=dt, object_pos=d.xpos[self.object_body],
                          object_velocity=object_velocity,
                          object_rotation=d.xmat[self.object_body].reshape(3,3),
                          goal_pos=self.goal_pos, contacts=contacts,
                          upright=d.xpos[p,2]>.45 and d.xmat[p,8]>.55)

    def step(self, joint_targets, hand=(0.,0.)):
        targets = np.asarray(joint_targets, dtype=float)
        if targets.shape != (29,) or not np.all(np.isfinite(targets)):
            raise ValueError("joint_targets must contain 29 finite angles in hardware order")
        ftargets = finger_targets(hand)
        m, d = self.model, self.data
        d.ctrl[self.actuator_ids] = targets
        d.ctrl[self.f_actuator_ids] = ftargets
        for _ in range(SUBSTEPS):
            # mj_step computes contacts and world poses at the beginning of the
            # interval, then integrates qvel/qpos. Score the matching velocity.
            velocity = d.qvel[self.object_vadr:self.object_vadr+6].copy()
            mujoco.mj_step(m, d)
            self._sample(PHYSICS_DT, self.contacts(), velocity)
        mujoco.mj_forward(m, d)

    def observe(self):
        d = self.data
        return dict(q=d.qpos[self.qadr].copy(), dq=d.qvel[self.vadr].copy(),
                    base_pos=d.xpos[self.pelvis].copy(), base_quat=d.xquat[self.pelvis].copy(),
                    torso_pos=d.body("torso_link").xpos.copy(), torso_quat=d.body("torso_link").xquat.copy(),
                    base_gyro=d.qvel[3:6].copy(), gravity_body=-d.xmat[self.pelvis].reshape(3,3)[2].copy(),
                    left_hand_pos=d.site("left_grasp").xpos.copy(), right_hand_pos=d.site("right_grasp").xpos.copy())

    def metrics(self):
        return self.score.result()

    def render(self, width=640, height=480, camera="overview"):
        if camera not in self.camera_rig["cameras"]:
            raise ValueError("camera must be overview, hand or side")
        size = width, height
        if size not in self._renderers:
            # Both solver raster sizes share this scene and retain their own GL context.
            if len(self._renderers) == 2:
                _, oldest = self._renderers.popitem(last=False)
                oldest.close()
            self._renderers[size] = mujoco.Renderer(self.model, width=width, height=height)
        self._renderers.move_to_end(size)
        renderer = self._renderers[size]
        cam = mujoco.MjvCamera()
        pose = self.camera_rig["cameras"][camera]
        cam.lookat[:] = pose["lookat"]
        cam.distance = pose["distance"]
        cam.azimuth = pose["azimuth"]
        cam.elevation = pose["elevation"]
        renderer.update_scene(self.data, camera=cam)
        return renderer.render().copy()

    def calibration(self, width, height):
        return camera_calibration(self._renderers[width, height], width, height)

    def close(self):
        for renderer in self._renderers.values():
            renderer.close()
        self._renderers.clear()
