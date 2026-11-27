"""Fixed calibrated RGB-D cameras. Only rendered pixels leave the simulator."""

import base64
import io
import math
import zlib

import mujoco as mj
import numpy as np
from PIL import Image


WIDTH, HEIGHT = 640, 480
# Name, world position, look-at point, image-up reference, vertical field of view.
CAMERAS = (
    ("left_overhead", (.60, 1.35, 3.25), (.60, 1.35, .72), (1, 0, 0), 52.),
    ("left_oblique", (-1.05, .00, 2.35), (.65, 1.35, .90), (0, 0, 1), 58.),
    ("right_overhead", (.78, -1.24, 3.05), (.78, -1.24, .72), (1, 0, 0), 52.),
    ("right_oblique", (-1.05, -2.45, 2.35), (.78, -1.24, .90), (0, 0, 1), 58.),
)


def calibration(position, look_at, image_up, fovy):
    """Return standard computer-vision intrinsics and camera-to-world transform."""
    forward = np.array(look_at, dtype=float) - position
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, image_up)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    transform = np.eye(4)
    transform[:3, :3] = np.column_stack((right, down, forward))
    transform[:3, 3] = position
    focal = HEIGHT / (2 * math.tan(math.radians(fovy) / 2))
    intrinsic = np.array([[focal, 0, (WIDTH - 1) / 2],
                          [0, focal, (HEIGHT - 1) / 2], [0, 0, 1]])
    return intrinsic, transform


class RGBDSensor:
    def __init__(self, model):
        self.model = model
        # Multisample depth resolves to an off-centre sample, which would make
        # RGB/depth unprojection disagree by millimetres. Configure this sensor's
        # framebuffer without MSAA and immediately restore the model setting.
        samples = model.vis.quality.offsamples
        model.vis.quality.offsamples = 0
        try:
            self.renderer = mj.Renderer(model, height=HEIGHT, width=WIDTH)
        finally:
            model.vis.quality.offsamples = samples
        self.option = mj.MjvOption()
        self.option.sitegroup[:] = 0
        self.calibrations = {
            name: calibration(position, look_at, image_up, fovy)
            for name, position, look_at, image_up, fovy in CAMERAS
        }

    def observe(self, data):
        frames = {}
        renderer = self.renderer
        for name, _, _, _, fovy in CAMERAS:
            intrinsic, transform = self.calibrations[name]
            renderer.disable_depth_rendering()
            renderer.update_scene(data, scene_option=self.option)
            # A fixed monocular view is written into both GL eyes. This leaves
            # the physics model and state untouched and avoids stereo offsets.
            for camera in renderer.scene.camera:
                camera.pos[:] = transform[:3, 3]
                camera.forward[:] = transform[:3, 2]
                camera.up[:] = -transform[:3, 1]
                camera.frustum_center = 0
                camera.frustum_width = 0
                camera.frustum_top = camera.frustum_near * math.tan(math.radians(fovy) / 2)
                camera.frustum_bottom = -camera.frustum_top
                camera.orthographic = 0
            rgb = renderer.render()
            png = io.BytesIO()
            Image.fromarray(rgb).save(png, format="PNG", compress_level=1)
            renderer.enable_depth_rendering()
            depth = renderer.render().astype("<f4", copy=False)
            frames[name] = {
                "width": WIDTH, "height": HEIGHT,
                "K": intrinsic.ravel().tolist(),
                "T_world_camera": transform.ravel().tolist(),
                "rgb_png_base64": base64.b64encode(png.getvalue()).decode("ascii"),
                "depth_f32_zlib_base64": base64.b64encode(zlib.compress(depth.tobytes(order="C"), level=1)).decode("ascii"),
                "depth_units": "m", "depth_convention": "camera_z", "depth_dtype": "<f4",
            }
        return frames

    def close(self):
        self.renderer.close()
