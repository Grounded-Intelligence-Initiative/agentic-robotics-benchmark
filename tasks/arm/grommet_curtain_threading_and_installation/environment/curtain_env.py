"""The Mobile FR3 Duo controller interface shared by practice and verification."""

import base64
from io import BytesIO
import json
from math import atan2, cos, pi, sin
from pathlib import Path

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parent
CALIBRATION = ROOT / "robot_description" / "calibration.json"
CONTROL_STEPS = 12  # 20 Hz, with 240 Hz physics.
ACTION_DIM = 20
ARM_DELTA = 0.05
GRIPPER_DELTA = 0.006
LIFT_DELTA = 0.014
BASE_TRANSLATION = 0.02
BASE_ROTATION = 0.05
BASE_RADIUS = 0.25
GRIPPER_WIDTH = 0.085
GRIPPER_ANGLE = 0.8
JAW_SIGNS = (1, -1, 1, -1, -1, 1)


def parse_action(value):
    action = np.asarray(value, dtype=np.float64)
    if action.shape != (ACTION_DIM,) or not np.isfinite(action).all():
        raise ValueError(f"action must contain {ACTION_DIM} finite numbers")
    return action


def base_targets(pose, delta):
    """Return the intermediate translation and final yaw in the initial-base frame."""
    pose, delta = np.asarray(pose), np.asarray(delta)
    move = delta[:2].copy()
    length = np.linalg.norm(move)
    if length > BASE_TRANSLATION:
        move *= BASE_TRANSLATION / length
    x, y, yaw = pose
    dx, dy = move
    local_xy = np.clip([x + cos(yaw) * dx - sin(yaw) * dy, y + sin(yaw) * dx + cos(yaw) * dy],
                       -BASE_RADIUS, BASE_RADIUS)
    rotation = float(np.clip(delta[2], -BASE_ROTATION, BASE_ROTATION))
    return np.r_[local_xy, yaw], rotation


class Cameras:
    def __init__(self, sim):
        import omni.replicator.core as rep
        from pxr import Gf, UsdGeom

        self.sim = sim
        streams = json.loads(CALIBRATION.read_text())["streams"]
        frames = {prim.GetName(): prim.GetPath() for prim in sim.stage.Traverse()}
        products = {}
        self.annotators = []
        for name, stream in streams.items():
            frame = stream["frame_id"]
            if frame not in products:
                size = (stream["width"], stream["height"])
                k = stream["intrinsics"]
                camera = UsdGeom.Camera.Define(sim.stage, frames[frame].AppendChild("CurtainCamera"))
                # Sensor frames are ROS optical (+Z forward); USD cameras look down -Z.
                camera.AddOrientOp().Set(Gf.Quatf(0, 1, 0, 0))
                focal = 20.955 * k["fx"] / size[0]
                camera.CreateFocalLengthAttr(focal)
                camera.CreateHorizontalApertureAttr(focal * size[0] / k["fx"])
                camera.CreateVerticalApertureAttr(focal * size[1] / k["fy"])
                camera.CreateClippingRangeAttr(Gf.Vec2f(.05, 20))
                products[frame] = rep.create.render_product(str(camera.GetPath()), size)
            kind = stream["kind"]
            annotator = rep.AnnotatorRegistry.get_annotator(
                "rgb" if kind == "rgb" else "distance_to_image_plane", device="cpu"
            )
            annotator.attach([products[frame]])
            self.annotators.append((name, stream, annotator))

    def render(self, warmup=False):
        from isaaclab_physx.renderers.isaac_rtx_renderer_utils import ensure_isaac_rtx_render_update

        for _ in range(6 if warmup else 1):
            self.sim.render()
            ensure_isaac_rtx_render_update(force=True)
        obs = {"rgb": {}, "depth": {}}
        for name, stream, annotator in self.annotators:
            h, w = stream["height"], stream["width"]
            data = np.asarray(annotator.get_data())
            kind = stream["kind"]
            if data.shape[:2] != (h, w):
                raise RuntimeError(f"camera {name} returned {data.shape}, expected {h}x{w}")
            if kind == "rgb":
                image = np.ascontiguousarray(data[..., :3])
            else:
                data = data.reshape(h, w)
                lo, hi = stream["range_m"]
                valid = np.isfinite(data) & (lo <= data) & (data <= hi)
                image = np.zeros((h, w), dtype=np.uint16)
                image[valid] = np.rint(data[valid] / stream["depth_unit_m"]).astype(np.uint16)
            buf = BytesIO()
            Image.fromarray(image).save(buf, format="PNG", compress_level=1)
            obs[kind][name] = base64.b64encode(buf.getbuffer()).decode("ascii")
        return obs


class CurtainEnv:
    def __init__(self, log_dir, *, seed):
        import carb
        from curtain_scene import PHYSICS_DT, ROBOT_X, SPINE_TRAVEL, make_scene, make_scorer

        self.dt = PHYSICS_DT
        self.world_x = ROBOT_X
        self.lift_max = SPINE_TRAVEL
        self.sim, self.scene = make_scene(log_dir, seed=seed)
        self.robot = self.scene["mobile_fr3"]
        self.scorer = make_scorer(self.scene)
        names = self.robot.joint_names
        self.arms = [names.index(f"{side}_fr3v2_1_joint{i}") for side in ("left", "right") for i in range(1, 8)]
        self.spine = names.index("franka_spine_vertical_joint")
        jaw_parts = (
            "left_knuckle", "right_knuckle", "left_inner_knuckle", "right_inner_knuckle",
            "left_finger_tip", "right_finger_tip",
        )
        self.jaws = [
            [names.index(f"{side}_robotiq_85_{part}_joint") for part in jaw_parts]
            for side in ("left", "right")
        ]
        self.joints = self.robot.data.default_joint_pos.torch.clone()
        self.arm_limits = self.robot.data.joint_pos_limits.torch[0, self.arms].cpu().numpy()
        self.gripper_targets = np.full(2, GRIPPER_WIDTH)
        self.lift_target = 0.0
        self.base = np.zeros(3)
        self.t = 0

        for _ in range(CONTROL_STEPS):
            self.sim.step(render=False)
        self.scene.update(CONTROL_STEPS * self.dt)
        self.base_z = float(self.robot.data.root_link_pose_w.torch[0, 2])
        carb.settings.get_settings().set_string("/rtx/rendermode", "RealTimePathTracing")
        self.cameras = Cameras(self.sim)

    def observation(self):
        data = self.robot.data
        q = data.joint_pos.torch[0].cpu().numpy()
        dq = data.joint_vel.torch[0].cpu().numpy()
        aperture = GRIPPER_WIDTH * (1 - q[[ids[0] for ids in self.jaws]] / GRIPPER_ANGLE)
        obs = self.cameras.render(warmup=self.t == 0)
        obs["robot_state"] = {
            "time_s": self.t * CONTROL_STEPS * self.dt,
            "arm_q_rad": q[self.arms].tolist(),
            "arm_dq_rad_s": dq[self.arms].tolist(),
            "gripper_width_m": np.clip(aperture, 0, GRIPPER_WIDTH).tolist(),
            "lift_height_m": float(q[self.spine]),
            "base_xy_yaw": self.base.tolist(),
            "arm_target_rad": self.joints[0, self.arms].cpu().tolist(),
            "gripper_target_m": self.gripper_targets.tolist(),
            "lift_target_m": self.lift_target,
        }
        return obs

    def step(self, value):
        import torch

        action = parse_action(value)
        device = self.sim.device
        arm_delta = torch.as_tensor(np.clip(action[:14], -ARM_DELTA, ARM_DELTA), device=device, dtype=torch.float32)
        lower, upper = torch.as_tensor(self.arm_limits.T, device=device)
        self.joints[0, self.arms] = torch.clamp(self.joints[0, self.arms] + arm_delta, lower, upper)
        self.gripper_targets = np.clip(self.gripper_targets + np.clip(action[14:16], -GRIPPER_DELTA, GRIPPER_DELTA),
                                       0, GRIPPER_WIDTH)
        for ids, width in zip(self.jaws, self.gripper_targets, strict=True):
            angle = GRIPPER_ANGLE * (1 - width / GRIPPER_WIDTH)
            self.joints[0, ids] = torch.tensor([angle * sign for sign in JAW_SIGNS],
                                               dtype=self.joints.dtype, device=device)
        self.lift_target = float(np.clip(self.lift_target + np.clip(action[16], -LIFT_DELTA, LIFT_DELTA),
                                         0, self.lift_max))
        self.joints[0, self.spine] = self.lift_target
        self.robot.actuators.target_command.set_position_index(value=self.joints)
        self.scene.write_data_to_sim()

        start = self.base
        translated, turn = base_targets(start, action[17:])
        moving = not np.array_equal(translated[:2], start[:2])
        translating_steps = CONTROL_STEPS // 2 if moving and turn else CONTROL_STEPS if moving else 0
        for tick in range(CONTROL_STEPS):
            if tick < translating_steps:
                pose = start + (translated - start) * ((tick + 1) / translating_steps)
            else:
                fraction = (tick - translating_steps + 1) / (CONTROL_STEPS - translating_steps)
                pose = translated + np.array([0., 0., fraction * turn])
            if tick == 0 or moving or turn:
                x, y, yaw = pose
                half_yaw = (yaw - pi / 2) / 2
                root = torch.tensor([[self.world_x + y, -x, self.base_z, 0., 0.,
                                      sin(half_yaw), cos(half_yaw)]], dtype=torch.float32, device=device)
                self.robot.write_root_link_pose_to_sim_index(root_pose=root, skip_forward=True)
            self.sim.step(render=False)
        translated[2] = atan2(sin(translated[2] + turn), cos(translated[2] + turn))
        self.base = translated
        self.scene.update(CONTROL_STEPS * self.dt)
        self.t += 1

    def measure(self):
        return self.scorer.measure()
