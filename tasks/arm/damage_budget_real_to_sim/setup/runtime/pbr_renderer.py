"""CPU-only MuJoCo Filament cameras, shared by live observations and review renders.

The native renderer is thread-affine. Keep it on one dedicated worker, evict old
model resources, and retain the classic renderer only for trusted ID masks.
Adapted from the benchmark's maintained flight renderer (Apache-2.0).
"""
import atexit
import ctypes
import gc
import os
from pathlib import Path
import queue
import threading

import mujoco
import numpy as np


def cpu_vulkan():
    paths = sorted(Path("/usr/share/vulkan/icd.d").glob("lvp_icd*.json"))
    if not paths:
        raise RuntimeError("CPU Vulkan (mesa-vulkan-drivers) is required; no GPU fallback is allowed")
    os.environ["VK_ICD_FILENAMES"] = str(paths[0])
    os.environ["VK_DRIVER_FILES"] = str(paths[0])
    os.environ.setdefault("LP_NUM_THREADS", "2")


class Renderer:
    def __init__(self, model, height, width):
        self.model, self.height, self.width = model, height, width
        self.scene = mujoco.MjvScene(model, maxgeom=max(1000, model.ngeom*2))
        self.worker = Worker.instance()
        self.sid = self.worker.call(self.worker.add, model)
        self.segment = False
        self.classic = None

    def update_scene(self, data, camera=-1, scene_option=None):
        self.data, self.request_camera = data, camera
        self.option = scene_option or mujoco.MjvOption()
        cam = camera
        if not isinstance(cam, mujoco.MjvCamera):
            cam = mujoco.MjvCamera()
            if camera != -1:
                cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                cam.fixedcamid = self.model.camera(camera).id if isinstance(camera, str) else camera
            else:
                mujoco.mjv_defaultFreeCamera(self.model, cam)
        mujoco.mjv_updateScene(self.model, data, self.option, None, cam,
                              mujoco.mjtCatBit.mjCAT_ALL, self.scene)

    def render(self):
        if self.segment:
            if self.classic is None:
                self.classic = mujoco.Renderer(self.model, self.height, self.width)
            self.classic.update_scene(self.data, camera=self.request_camera, scene_option=self.option)
            self.classic.enable_segmentation_rendering()
            return self.classic.render().copy()
        camera = mujoco.mjv_averageCamera(self.scene.camera[0], self.scene.camera[1])
        values = {name: getattr(camera, name) for name in (
            "pos", "forward", "up", "frustum_center", "frustum_width", "frustum_bottom",
            "frustum_top", "frustum_near", "frustum_far", "orthographic")}
        # Orthographic MuJoCo cameras leave width zero: derive it from the viewport.
        values["frustum_width"] = (camera.frustum_top-camera.frustum_bottom)/2*self.width/self.height
        scale = 2 if max(self.width, self.height) <= 512 else 1
        raw = self.worker.call(self.worker.draw, self.sid, self.data, values,
                               self.width*scale, self.height*scale, self.option.geomgroup.copy())
        # Fixed exposure and spatial supersampling, not temporal post-processing:
        # repeated observations of an unchanged state must be byte-identical.
        linear = raw.astype(np.float32)/255
        if scale > 1:
            linear = linear.reshape(self.height, scale, self.width, scale, 3).mean(axis=(1, 3))
        x = linear*1.4
        tone = np.clip(x*(2.51*x+.03)/(x*(2.43*x+.59)+.14), 0, 1)
        srgb = np.where(tone <= .0031308, 12.92*tone, 1.055*tone**(1/2.4)-.055)
        return np.round(srgb*255).astype(np.uint8)

    def enable_segmentation_rendering(self):
        self.segment = True

    def disable_segmentation_rendering(self):
        self.segment = False

    def close(self):
        if self.sid is not None:
            self.worker.call(self.worker.remove, self.sid)
            self.sid = None
        if self.classic is not None:
            self.classic.close()
            self.classic = None


class Worker:
    singleton = None
    lock = threading.Lock()

    @classmethod
    def instance(cls):
        with cls.lock:
            if cls.singleton is None:
                cls.singleton = cls()
            return cls.singleton

    def __init__(self):
        cpu_vulkan()
        try:
            ctypes.CDLL("libc.so.6").mallopt(-8, 2)
        except (OSError, AttributeError):
            pass
        self.jobs = queue.Queue()
        self.models, self.targets, self.live = {}, {}, []
        self.active, self.counter = None, 0
        self.thread = threading.Thread(target=self.loop, name="cpu-filament", daemon=True)
        self.thread.start()
        self.call(self.start)

    def call(self, method, *args):
        done, result = threading.Event(), []
        self.jobs.put((method, args, done, result))
        if not done.wait(300):
            raise RuntimeError("CPU renderer timed out")
        if not result[0][0]:
            raise RuntimeError(result[0][1])
        return result[0][1]

    def loop(self):
        while True:
            method, args, done, result = self.jobs.get()
            try:
                result.append((True, method(*args)))
            except BaseException as exc:
                result.append((False, f"{type(exc).__name__}: {exc}"))
            done.set()
            if method == self.shutdown:
                return

    def start(self):
        os.environ.pop("DISPLAY", None)
        os.environ.pop("WAYLAND_DISPLAY", None)
        import mujoco._render_filament as F
        from mujoco.experimental.studio import window  # noqa: F401: resource providers
        self.F = F
        self.ctx = F.Context(F.ContextConfig(graphics_api=F.GraphicsApi.GRAPHICS_API_VULKAN))
        self.ctx.set_clear_color([.16, .19, .22])
        # The lighting probe ships with the pinned Apache-2.0 MuJoCo package.
        raw = (Path(mujoco.__file__).parent/"experimental/studio/assets/ibl.ktx").read_bytes()
        config = F.TextureConfig()
        config.width, config.height, config.format = len(raw), 0, F.PixelFormat.PIXEL_FORMAT_KTX
        config.sampler_type, config.color_space = F.SamplerType.TEXTURE_CUBE, F.ColorSpace.COLORSPACE_LINEAR
        self.ibl = self.ctx.create_texture(config)
        data = F.TextureData()
        data.set_image(raw)
        self.ibl.upload(data)

    def add(self, model):
        self.counter += 1
        self.models[self.counter] = model
        return self.counter

    def evict(self):
        while self.live:
            self.live.pop(0)
        gc.collect()
        try:
            ctypes.CDLL("libc.so.6").malloc_trim(0)
        except (OSError, AttributeError):
            pass
        self.active = None

    def remove(self, sid):
        self.models.pop(sid, None)
        if self.active == sid:
            self.evict()

    def draw(self, sid, data, camera, width, height, groups):
        F = self.F
        if self.active != sid:
            self.evict()
            model = self.models[sid]
            resources = F.ModelResources(self.ctx, model)
            scene = self.ctx.create_scene(F.SceneParams())
            scene.configure_from_model(model)
            lights = F.ModelLights(scene, resources)
            objects = F.ModelRenderables(scene, resources)
            params = F.LightParams()
            params.type, params.texture = F.LightType.LIGHT_IMAGE, self.ibl
            params.color, params.intensity = [1., 1., 1.], 32000.
            light = self.ctx.create_light(params)
            scene.add_light(light)
            self.live = [objects, lights, light, scene, resources]
            self.active = sid
        objects, lights, _, scene, _ = self.live
        option = mujoco.MjvOption()
        option.geomgroup[:] = groups
        option.sitegroup[:] = 0
        objects.set_options(option)
        lights.update(data)
        objects.update(data)
        key = (width, height)
        if key not in self.targets:
            target = self.ctx.create_render_target(F.RenderTargetConfig(color_format=F.PixelFormat.PIXEL_FORMAT_RGB8))
            target.resize(width, height)
            pixels = np.zeros((height, width, 3), np.uint8)
            read = F.ReadRequest()
            read.target = target
            read.set_buffer(pixels)
            self.targets[key] = target, read, pixels
        target, read, pixels = self.targets[key]
        request = F.RenderRequest(camera=F.Camera(**camera), draw_mode=F.DrawMode.DRAW_MODE_DEFAULT)
        # Screen-space reflections retain temporal history across unrelated
        # benchmark cameras. Use the fixed environment probe instead.
        request.reflections = False
        request.post_processing = False
        request.scene, request.target = scene, target
        request.viewport.left, request.viewport.bottom = 0, 0
        request.viewport.width, request.viewport.height = width, height
        self.ctx.wait_for_frame(self.ctx.render([request], [read]))
        return pixels.copy()

    def shutdown(self):
        self.models.clear()
        self.evict()
        self.targets.clear()
        self.ibl = None
        gc.collect()
        self.ctx = None


@atexit.register
def close_worker():
    if Worker.singleton is not None and Worker.singleton.thread.is_alive():
        Worker.singleton.call(Worker.singleton.shutdown)
        Worker.singleton.thread.join(30)
