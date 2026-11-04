"""Public camera and contact observation wrapper for native robosuite PickPlace."""
import base64
import io

import numpy as np
import robosuite
from PIL import Image

from contact_overlay import contact_heatmap, render_contact_overlay

CONTROL_HZ = 20
MAX_STEPS = 115
CAMERA_SIZE = 128


def _png(image):
    buffer = io.BytesIO()
    Image.fromarray(image).save(buffer, format="PNG")
    return base64.b64encode(buffer.getvalue()).decode("ascii")


class RapidSortEpisode:
    """Physical pick and placement with synchronized visual and contact frames."""

    def __init__(self, seed, render=True):
        self.seed = int(seed)
        self.render = bool(render)
        self.env = robosuite.make(
            env_name="PickPlaceCan",
            robots="Panda",
            seed=self.seed,
            has_renderer=False,
            has_offscreen_renderer=self.render,
            use_camera_obs=self.render,
            use_object_obs=False,
            camera_names=["agentview", "robot0_eye_in_hand"],
            camera_heights=[CAMERA_SIZE, CAMERA_SIZE],
            camera_widths=[CAMERA_SIZE, CAMERA_SIZE],
            camera_depths=False,
            reward_shaping=False,
            control_freq=CONTROL_HZ,
            horizon=MAX_STEPS,
            hard_reset=False,
        )
        self.raw = self.env.reset()
        self.t = 0
        self.stable_steps = 0
        self.success = False

    def _external_calibration(self):
        sim = self.env.sim
        camera_id = sim.model.camera_name2id("agentview")
        rotation = np.asarray(sim.data.cam_xmat[camera_id]).reshape(3, 3)
        # This basis is x-right, y-down, z-forward in the returned PNG.
        forward_basis = rotation @ np.diag([1.0, -1.0, -1.0])
        return {
            "origin": np.asarray(sim.data.cam_xpos[camera_id]).tolist(),
            "right_down_forward_to_world": forward_basis.tolist(),
            "fovy_deg": float(sim.model.cam_fovy[camera_id]),
        }

    def observe(self):
        raw = self.raw
        if self.render:
            external = raw["agentview_image"][::-1]
            wrist = raw["robot0_eye_in_hand_image"][::-1]
            heat, _ = contact_heatmap(self.env, CAMERA_SIZE, CAMERA_SIZE)
            external_png = _png(external)
            wrist_png = _png(wrist)
            contact_png = _png(np.asarray(render_contact_overlay(wrist, heat)))
        else:
            external_png = wrist_png = contact_png = ""
        return {
            "eef_pos": raw["robot0_eef_pos"].tolist(),
            "eef_quat": raw["robot0_eef_quat"].tolist(),
            "joint_pos": raw["robot0_joint_pos"].tolist(),
            "gripper_qpos": raw["robot0_gripper_qpos"].tolist(),
            "external_camera": self._external_calibration(),
            "external_rgb_png_base64": external_png,
            "wrist_rgb_png_base64": wrist_png,
            "wrist_contact_png_base64": contact_png,
        }

    def step(self, action):
        array = np.asarray(action, dtype=float)
        if array.shape != (7,) or not np.isfinite(array).all():
            raise ValueError("action must be seven finite numbers")
        low, high = self.env.action_spec
        self.raw, _, _, _ = self.env.step(np.clip(array, low, high))
        self.t += 1
        self.stable_steps = self.stable_steps + 1 if self.env._check_success() else 0
        self.success = self.stable_steps >= 5
        return self.observe(), self.success or self.t >= MAX_STEPS

    def close(self):
        self.env.close()
