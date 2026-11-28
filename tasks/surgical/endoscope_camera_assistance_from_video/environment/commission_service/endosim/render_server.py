"""The renderer: one headless Isaac Sim process that draws the endoscope's frames for whoever runs an episode.

    python3 -m endosim.render_server            (started by render_client.Renderer, never by hand)

It knows no procedure: it is told a scene to build (`load`) and then, frame by frame, where everything is (`render`),
and answers with the finished JPEG. The episode, its future and its scoring stay in the parent process. Requests and
replies are length-prefixed JSON on the process's own stdin / stdout; Kit's console output is moved to stderr before
Isaac Sim starts, so nothing else can write on the wire.

Rendering: RTX Real-Time path tracing through a Replicator render product at 640 x 480, one subframe per frame, the
render product's settings pinned (RENDER_SETTINGS). Physics never runs. Every frame is checked against the frame
clock (scene.py) and rendered again when the annotator returned an earlier frame.
"""

import json
import os
import struct
import sys
import time
import traceback

# The wire: this process's original stdout. Kit and its extensions print to fd 1; from here on that is stderr.
_OUT = os.fdopen(os.dup(1), "wb", buffering=0)
os.dup2(2, 1)
_IN = os.fdopen(os.dup(0), "rb", buffering=0)

RENDER_SETTINGS = {
    "omni:rtx:rendermode": "RealTimePathTracing",
    "omni:rtx:post:aa:op": "dlss",
    "omni:rtx:post:dlss:execMode": "quality",
    "omni:rtx:post:motionblur:enabled": False,
    "omni:rtx:post:dof:enabled": False,
    "omni:rtx:post:tonemap:operator": "acesApproximation",
    "omni:rtx:post:tonemap:exposureKey": 0.25,
    "omni:rtx:rt:ecoMode:enabled": False,
}
MAX_TRIES = 8                 # renders of one frame before the clock must have caught up
HIDDEN_CLOCK_CAPTURES = 3     # captures of an unchanged stage after which a frame is taken without its clock


def _send(header, payload=b""):
    body = json.dumps(header).encode("utf-8")
    _OUT.write(struct.pack(">II", len(body), len(payload)) + body + payload)


def _read_exact(n):
    buf = b""
    while len(buf) < n:
        chunk = _IN.read(n - len(buf))
        if not chunk:
            raise EOFError
        buf += chunk
    return buf


def _recv():
    (n,) = struct.unpack(">I", _read_exact(4))
    return json.loads(_read_exact(n).decode("utf-8"))


def log(*a):
    print("[render_server]", *a, file=sys.stderr, flush=True)


class Server(object):
    def __init__(self):
        t0 = time.time()
        from isaacsim import SimulationApp
        # Omniverse RTX refuses to render on an NVIDIA driver older than R550 ("rtx driver verification failed"). The
        # check is off: whether the frames are usable is the canary's to decide (markers within 2.5 px), not the
        # driver's version. On a supported driver it would pass anyway.
        cfg = {"headless": True, "multi_gpu": False, "extra_args": ["--/rtx/verifyDriverVersion/enabled=false"]}
        if os.environ.get("ENDOSIM_GPU"):
            cfg["active_gpu"] = cfg["physics_gpu"] = int(os.environ["ENDOSIM_GPU"])
        self.app = SimulationApp(cfg)
        import carb
        import omni.kit.app
        self.settings = carb.settings.get_settings()
        omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("omni.replicator.core", True)
        for _ in range(2):
            self.app.update()
        # Replicator waits for "asset loading" before its first frame, up to this many seconds per run
        self.settings.set("/exts/omni.replicator.core/maxAssetLoadingTime", 5.0)
        self.settings.set_bool("/app/hydraEngine/waitIdle", True)
        import omni.replicator.core as rep
        self.rep = rep
        self.rp = self.annot = self.scene = self.post = None
        self.tmp = os.path.join(os.environ.get("TMPDIR", "/tmp"), "endosim_tex_%d" % os.getpid())
        os.makedirs(self.tmp, exist_ok=True)
        self.start_s = round(time.time() - t0, 1)
        self.stats = {"frames": 0, "renders": 0, "stale": 0, "render_ms": 0.0}

    # -- stage and render product ------------------------------------------------------------------------------------
    def _new_stage(self):
        import omni.usd
        if self.annot is not None:
            self.annot.detach()
        if self.rp is not None:
            try:
                self.rp.destroy()
            except Exception:  # noqa: BLE001
                pass
        self.rp = self.annot = None
        omni.usd.get_context().new_stage()
        return omni.usd.get_context().get_stage()

    def _product(self, stage):
        from . import model
        for _ in range(3):
            self.app.update()
        self.rp = self.rep.create.render_product("/World/Scope/Cam", (model.IMAGE_W, model.IMAGE_H), name="endoscope")
        path = self.rp.path if hasattr(self.rp, "path") else str(self.rp)
        self.annot = self.rep.AnnotatorRegistry.get_annotator("rgb")
        self.annot.attach([self.rp])
        prim = stage.GetPrimAtPath(path)
        self.pinned, self.absent = {}, []
        for name, value in RENDER_SETTINGS.items():
            attr = prim.GetAttribute(name) if prim and prim.IsValid() else None
            if attr is not None and attr.IsValid():
                attr.Set(value)
                self.pinned[name] = value
            else:
                self.absent.append(name)

    def _capture(self):
        import numpy as np
        t0 = time.perf_counter()
        self.settings.set_bool("/app/player/playSimulations", False)
        try:
            self.rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False)
        finally:
            self.settings.set_bool("/app/player/playSimulations", True)
        arr = np.asarray(self.annot.get_data())
        self.stats["renders"] += 1
        self.stats["render_ms"] += 1000.0 * (time.perf_counter() - t0)
        return arr[..., :3] if arr.ndim == 3 and arr.shape[0] > 0 else None

    def _fresh(self, serial):
        """Render until the frame clock shows `serial`; returns (frame, renders used). One plain app update first:
        it carries the stage's changes to the renderer, and the capture that follows is then usually the right one."""
        from . import post
        from .scene import CLOCK_SLOTS
        self.settings.set_bool("/app/player/playSimulations", False)
        try:
            self.app.update()
        finally:
            self.settings.set_bool("/app/player/playSimulations", True)
        hidden = 0
        for k in range(1, MAX_TRIES + 1):
            rgb = self._capture()
            slot = post.read_clock(rgb) if rgb is not None else -1
            if slot == int(serial) % CLOCK_SLOTS:
                return rgb, k
            if slot is None:
                # the clock cannot be read: something sits right at the lens (the scope run into an instrument's
                # shaft). The annotator lags by one capture at most (measured), so the third capture of an
                # unchanged stage shows it as it is.
                hidden += 1
                if hidden >= HIDDEN_CLOCK_CAPTURES:
                    self.stats["clock_hidden"] = self.stats.get("clock_hidden", 0) + 1
                    return rgb, k
            else:
                self.stats["stale"] += 1
        raise RuntimeError("the frame clock did not reach serial %d in %d renders" % (serial, MAX_TRIES))

    # -- commands --------------------------------------------------------------------------------------------------
    def load(self, spec):
        from . import post, scene
        t0 = time.time()
        stage = self._new_stage()
        self.scene = scene.Scene(stage, spec, self.tmp)
        self._product(stage)
        self.post = post.Post(spec["look"], spec["look"]["texture_seed"])
        return {"ok": True, "load_s": round(time.time() - t0, 2), "pinned": self.pinned, "absent": self.absent}

    def render(self, state, raw=False):
        t0 = time.perf_counter()
        self.scene.apply(state)
        rgb, tries = self._fresh(state["serial"])
        frame = self.post.finish(rgb, state)
        self.stats["frames"] += 1
        jpeg = self.post.deliver(frame)
        head = {"ok": True, "tries": tries, "ms": round(1000.0 * (time.perf_counter() - t0), 1), "jpeg_len": len(jpeg)}
        if raw:
            import cv2
            ok, png = cv2.imencode(".png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            head["raw_len"] = len(png)
            return head, jpeg + png.tobytes()
        return head, jpeg

    def canary(self):
        """Is this renderer usable, and does it draw what the public camera model says? For each scope: a marker
        ball at known points seen from known holder poses; where it lands in the frame against model.project."""
        import numpy as np
        from . import model, post, scene
        out = {"scopes": {}, "ok": True}
        serial = 0
        for scope in ("0", "30"):
            stage = self._new_stage()
            ms = scene.MarkerScene(stage, scope)
            self._product(stage)
            errs, lit, found = [], [], 0
            cases = [((0.0, 0.55, 0.06, 0.0), (0.01, 0.10, -0.15)), ((0.3, 0.7, 0.10, 0.2), (0.07, 0.13, -0.14)),
                     ((-0.4, 0.9, 0.12, -0.3), (-0.08, 0.16, -0.13)), ((0.1, 0.3, 0.08, 0.5), (0.03, 0.02, -0.16)),
                     ((-0.2, 1.0, 0.15, 0.0), (-0.02, 0.19, -0.12))]
            if scope == "30":       # the 30 degree scope looks 30 degrees below its axis: aim the same poses lower
                cases = [(q, (m[0], m[1] - 0.06, m[2])) for q, m in cases]
            for q, marker in cases:
                pose = model.camera_pose(q, scope)
                want = model.project(np.array([marker]), pose, scope)[0]
                serial += 1
                ms.apply(serial, pose, marker)
                rgb = None
                for _ in range(MAX_TRIES * 3):
                    f = self._capture()
                    if f is not None and post.read_clock(f) == serial % scene.CLOCK_SLOTS:
                        rgb = f
                        break
                if rgb is None:
                    out["ok"] = False
                    continue
                g = (rgb[..., 1].astype(int) > 140) & (rgb[..., 0] < 110) & (rgb[..., 2] < 110)
                g[:30, :100] = False
                lit.append(float(rgb.mean()))
                if g.sum() < 4:
                    errs.append(None)
                    continue
                ys, xs = np.nonzero(g)
                found += 1
                errs.append(float(np.hypot(xs.mean() - want[0], ys.mean() - want[1])))
            seen = [e for e in errs if e is not None]
            out["scopes"][scope] = {"markers": len(cases), "found": found, "max_error_px": max(seen) if seen else None,
                                    "errors_px": [None if e is None else round(e, 2) for e in errs],
                                    "mean_level": round(float(np.mean(lit)), 1) if lit else None}
        self.scene = self.post = None
        return out

    def info(self):
        out = {"ok": True, "start_s": self.start_s, "stats": dict(self.stats)}
        try:
            import omni.hydra.engine.stats as es
            out["gpu"] = [str(d.get("description")) for d in es.get_device_info()]
        except Exception as exc:  # noqa: BLE001
            out["gpu_error"] = str(exc)[:200]
        return out


def main():
    try:
        server = Server()
    except Exception:  # noqa: BLE001
        _send({"ready": False, "error": traceback.format_exc()[-2000:]})
        raise
    _send({"ready": True, "start_s": server.start_s})
    while True:
        try:
            msg = _recv()
        except EOFError:
            break
        cmd = msg.get("cmd")
        try:
            if cmd == "quit":
                _send({"ok": True})
                break
            if cmd == "load":
                _send(server.load(msg["spec"]))
            elif cmd == "render":
                head, payload = server.render(msg["state"], raw=bool(msg.get("raw")))
                _send(head, payload)
            elif cmd == "canary":
                _send(server.canary())
            elif cmd == "info":
                _send(server.info())
            else:
                _send({"ok": False, "error": "unknown command %r" % (cmd,)})
        except Exception:  # noqa: BLE001
            log(traceback.format_exc())
            _send({"ok": False, "error": traceback.format_exc()[-1500:]})
    try:
        import shutil
        shutil.rmtree(server.tmp, ignore_errors=True)
        server.app.close()
    except Exception:  # noqa: BLE001
        pass


if __name__ == "__main__":
    main()
