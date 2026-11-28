"""Camera colour of the data-hall cell (cell internals): MuJoCo's Filament renderer (physically based, on Vulkan: Mesa's
lavapipe on the CPU in the sandbox), the hall's own lights (the model's spot lights over the aisles, in candela) plus
the procedural image light of ibl.py, filmic tone mapping, ambient occlusion and a little bloom for the LEDs.

Filament facts this relies on (MuJoCo 3.16, as found for house_cleaning_with_tools's render.py): one context serves
one model's resources at a time, so only the model rendered last is live; the engine is bound to the thread that
created it, so a dedicated thread owns the context and every Filament object and callers post jobs to it; textures of
colour must be marked sRGB; a reflective material (reflectance > 0) costs a planar mirror pass of the whole scene, so
the cell's materials have none; geom rgba changes (the LEDs) reach the renderer on the next frame.
"""

from __future__ import annotations

import atexit
import gc
import math
import os
import queue
import threading
import weakref


def _cap_malloc_arenas():
    try:
        import ctypes
        ctypes.CDLL("libc.so.6").mallopt(-8, 2)
    except (OSError, AttributeError):
        pass


_cap_malloc_arenas()

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from . import ibl as _ibl  # noqa: E402

CAM_NEAR, CAM_FAR = 0.01, 80.0
CALL_TIMEOUT = 600.0

LOOK = {
    "version": "dc-look-1",
    "view": {
        "numeric": {"filament.ao.enabled": 1, "filament.ao.quality": 1, "filament.shadows.type": 0,
                    "filament.shadows.map_size": 2048, "filament.bloom.enabled": 1, "filament.bloom.strength": 0.08,
                    "filament.msaa.enabled": 0, "filament.cg.contrast": 1.06, "filament.cg.saturation": 0.95,
                    "filament.fallback.head_light_intensity": 0.0, "filament.fallback.scene_light_intensity": 0.0,
                    "filament.fallback.environment_light_intensity": 0.0},
        "text": {"filament.cg.tone_mapping": "filmic"},
    },
    "ibl_size": 64,
    "ibl_intensity": 12000.0,
    "clear_color": (0.05, 0.05, 0.055),
}


def add_view_options(spec: mujoco.MjSpec, look: dict = LOOK) -> None:
    """The Filament view options MuJoCo reads from the model's custom fields (`filament.*`)."""
    for key, value in sorted(look["view"]["numeric"].items()):
        spec.add_numeric(name=key, data=[float(value)])
    for key, value in sorted(look["view"]["text"].items()):
        spec.add_text(name=key, data=str(value))


class RenderError(RuntimeError):
    pass


class _View:
    def __init__(self, fil, model):
        self._fil = fil
        self._sid = fil.call(fil.new_session, model)
        self._finalizer = weakref.finalize(self, fil.post, fil.free_session, self._sid)

    def render(self, data, pos, forward, up, fovy, width, height):
        return self._fil.call(self._fil.draw, self._sid, data, pos, forward, up, fovy, width, height)

    def close(self):
        if self._finalizer.detach() is not None:
            self._fil.call(self._fil.free_session, self._sid)


class Filament:
    """The process's Filament renderer: a dedicated thread owns the context and every Filament object."""

    _inst = None
    _guard = threading.Lock()

    @classmethod
    def instance(cls):
        with cls._guard:
            if cls._inst is None or cls._inst.pid != os.getpid():
                cls._inst = cls()
            return cls._inst

    def __init__(self):
        self.pid = os.getpid()
        self.jobs = queue.Queue()
        self.models = {}
        self.active = None
        self.live = []
        self.targets = {}
        self.next_sid = 0
        self.thread = threading.Thread(target=self._loop, name="dc-cell-filament", daemon=True)
        self.thread.start()
        self.call(self._start)

    def open(self, model):
        return _View(self, model)

    def call(self, fn, *args):
        done = threading.Event()
        out = []
        self.jobs.put((fn, args, done, out))
        if not done.wait(CALL_TIMEOUT):
            raise RenderError("the renderer did not answer within %.0f s" % CALL_TIMEOUT)
        ok, val = out[0]
        if not ok:
            raise val
        return val

    def post(self, fn, *args):
        if self.pid == os.getpid() and self.thread.is_alive():
            self.jobs.put((fn, args, None, None))

    def stop(self, timeout=30.0):
        if self.pid == os.getpid() and self.thread.is_alive():
            self.jobs.put((None, (), None, None))
            self.thread.join(timeout)

    def _loop(self):
        while True:
            fn, args, done, out = self.jobs.get()
            if fn is None:
                self._shutdown()
                return
            try:
                res = (True, fn(*args))
            except BaseException as exc:  # noqa: BLE001
                res = (False, RenderError("renderer: %s: %s" % (type(exc).__name__, exc)))
            if done is not None:
                out.append(res)
                done.set()

    # -- the renderer's thread only
    def _start(self):
        for k in ("DISPLAY", "WAYLAND_DISPLAY"):
            os.environ.pop(k, None)
        try:
            ncpu = len(os.sched_getaffinity(0))
        except (AttributeError, OSError):
            ncpu = os.cpu_count() or 4
        os.environ.setdefault("LP_NUM_THREADS", str(max(1, min(8, ncpu))))
        import mujoco._render_filament as F
        from mujoco.experimental.studio import window  # noqa: F401  registers Filament's resource providers
        self.F = F
        self.ctx = F.Context(F.ContextConfig(graphics_api=F.GraphicsApi.GRAPHICS_API_VULKAN))
        self.ctx.set_clear_color([float(c) for c in LOOK["clear_color"]])
        raw = _ibl.hall_ibl(LOOK["ibl_size"])
        tc = F.TextureConfig()
        tc.width, tc.height, tc.format = len(raw), 0, F.PixelFormat.PIXEL_FORMAT_KTX
        tc.sampler_type, tc.color_space = F.SamplerType.TEXTURE_CUBE, F.ColorSpace.COLORSPACE_LINEAR
        self.ibl_tex = self.ctx.create_texture(tc)
        td = F.TextureData()
        td.set_image(raw)
        self.ibl_tex.upload(td)
        self.opt = mujoco.MjvOption()
        self.opt.geomgroup[:] = 0
        self.opt.geomgroup[:3] = 1
        self.opt.sitegroup[:] = 0

    def new_session(self, model):
        sid = self.next_sid
        self.next_sid += 1
        self.models[sid] = model
        return sid

    def free_session(self, sid):
        self.models.pop(sid, None)
        if self.active == sid:
            self._evict()

    def _evict(self):
        while self.live:
            self.live.pop(0)
        gc.collect()
        try:
            import ctypes
            ctypes.CDLL("libc.so.6").malloc_trim(0)
        except (OSError, AttributeError):
            pass
        self.active = None

    def _activate(self, sid):
        if self.active == sid:
            return
        model = self.models[sid]
        self._evict()
        F = self.F
        res = F.ModelResources(self.ctx, model)
        scn = self.ctx.create_scene(F.SceneParams())
        scn.configure_from_model(model)
        lights = F.ModelLights(scn, res)
        rends = F.ModelRenderables(scn, res)
        rends.set_options(self.opt)
        lp = F.LightParams()
        lp.type, lp.texture, lp.color, lp.intensity = F.LightType.LIGHT_IMAGE, self.ibl_tex, [1.0, 1.0, 1.0], \
            float(LOOK["ibl_intensity"])
        ibl = self.ctx.create_light(lp)
        scn.add_light(ibl)
        self.live = [rends, lights, ibl, scn, res]
        self.active = sid

    def _target(self, width, height):
        F = self.F
        key = (width, height)
        if key not in self.targets:
            buf = np.zeros((height, width, 3), np.uint8)
            tgt = self.ctx.create_render_target(F.RenderTargetConfig(color_format=F.PixelFormat.PIXEL_FORMAT_RGB8))
            tgt.resize(width, height)
            read = F.ReadRequest()
            read.target = tgt
            read.set_buffer(buf)
            self.targets[key] = (tgt, read, buf)
        return self.targets[key]

    def draw(self, sid, data, pos, forward, up, fovy, width, height):
        F = self.F
        self._activate(sid)
        rends, lights, _ibl_light, scn, _res = self.live
        lights.update(data)
        rends.update(data)
        tgt, read, buf = self._target(width, height)
        top = CAM_NEAR * math.tan(math.radians(fovy) / 2.0)
        cam = F.Camera(pos=np.asarray(pos, np.float32), forward=np.asarray(forward, np.float32),
                       up=np.asarray(up, np.float32), frustum_center=0.0, frustum_width=top * width / height,
                       frustum_bottom=-top, frustum_top=top, frustum_near=CAM_NEAR, frustum_far=CAM_FAR)
        req = F.RenderRequest(camera=cam, draw_mode=F.DrawMode.DRAW_MODE_DEFAULT)
        req.scene, req.target = scn, tgt
        try:
            req.reflections = False
        except Exception:  # noqa: BLE001
            pass
        req.viewport.left, req.viewport.bottom, req.viewport.width, req.viewport.height = 0, 0, width, height
        self.ctx.wait_for_frame(self.ctx.render([req], [read]))
        return buf.copy()

    def _shutdown(self):
        self.models.clear()
        self._evict()
        self.targets.clear()
        self.ibl_tex = None
        gc.collect()
        self.ctx = None
        gc.collect()


@atexit.register
def _stop_renderer():
    if Filament._inst is not None:
        Filament._inst.stop()


_SRGB_LUT = None


def downsample(img, s):
    """Average s x s pixel blocks in linear light (sRGB in and out)."""
    global _SRGB_LUT
    if s <= 1:
        return img
    if _SRGB_LUT is None:
        c = np.arange(256) / 255.0
        _SRGB_LUT = np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4).astype(np.float32)
    lin = _SRGB_LUT[img]
    h, w = img.shape[0] // s, img.shape[1] // s
    lin = lin[:h * s, :w * s].reshape(h, s, w, s, 3).mean(axis=(1, 3))
    out = np.where(lin <= 0.0031308, lin * 12.92, 1.055 * np.power(np.maximum(lin, 0), 1 / 2.4) - 0.055)
    return np.clip(np.round(out * 255.0), 0, 255).astype(np.uint8)


def look_at(pos, target, up=(0.0, 0.0, 1.0)):
    """(forward, up) unit vectors of a camera at `pos` looking at `target`."""
    p, t = np.asarray(pos, float), np.asarray(target, float)
    f = t - p
    f /= np.linalg.norm(f)
    r = np.cross(f, np.asarray(up, float))
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return f, u
