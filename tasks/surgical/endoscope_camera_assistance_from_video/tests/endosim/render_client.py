"""The parent's end of the renderer: start the Isaac Sim process, speak its wire, bound every wait, restart it.

Standard library only: the grader imports this as root, the commissioning service as its own account. The renderer
itself (render_server.py) is started with the launch prefix the caller gives, so that it runs as an account that is
neither root nor the agent's (the grader: `runuser -u <renderer account> --`).
"""

import json
import os
import select
import struct
import subprocess
import sys
import time


class RendererError(Exception):
    """The renderer failed, timed out or answered an error: the environment's fault, never the controller's."""


class Renderer(object):
    def __init__(self, package_dir, python=None, prefix=None, env=None, log_path=None, start_timeout=1800.0,
                 render_timeout=120.0, max_restarts=3, cwd=None):
        self.package_dir = package_dir          # the folder that holds the `endosim` package
        self.python = python or sys.executable
        self.prefix = list(prefix or [])
        self.env = dict(os.environ if env is None else env)
        self.env.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
        self.env.setdefault("ACCEPT_EULA", "Y")
        self.env.setdefault("PRIVACY_CONSENT", "Y")
        self.env["PYTHONDONTWRITEBYTECODE"] = "1"
        self.log_path = log_path
        self.start_timeout, self.render_timeout = float(start_timeout), float(render_timeout)
        self.max_restarts, self.restarts = int(max_restarts), 0
        self.cwd = cwd or package_dir
        self.proc = None
        self.spec = None
        self.started = None
        self.render_ms, self.frames, self.extra_renders = 0.0, 0, 0
        self._start()

    # -- process -----------------------------------------------------------------------------------------------------
    def _start(self):
        log = open(self.log_path, "ab") if self.log_path else subprocess.DEVNULL
        try:
            # -I: nothing of the environment or the working directory on the import path, only the package's folder
            boot = "import sys; sys.path.insert(0, {!r}); from endosim import render_server; render_server.main()".format(self.package_dir)
            self.proc = subprocess.Popen(self.prefix + [self.python, "-I", "-B", "-c", boot], cwd=self.cwd, env=self.env,
                                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log, start_new_session=True)
        finally:
            if self.log_path:
                log.close()
        head, _ = self._read(self.start_timeout)
        if not head.get("ready"):
            raise RendererError("the renderer did not start: {}".format(head.get("error", head)))
        self.started = head

    def _stop(self):
        if self.proc is None:
            return
        try:
            os.killpg(self.proc.pid, 9)
        except (OSError, ProcessLookupError):
            pass
        try:
            self.proc.wait(timeout=20)
        except Exception:  # noqa: BLE001
            pass
        self.proc = None

    def _read_exact(self, n, deadline):
        fd = self.proc.stdout.fileno()
        buf = b""
        while len(buf) < n:
            left = deadline - time.monotonic()
            if left <= 0:
                raise RendererError("the renderer did not answer in time")
            ready, _, _ = select.select([fd], [], [], min(left, 5.0))
            if not ready:
                if self.proc.poll() is not None:
                    raise RendererError("the renderer exited (code {})".format(self.proc.returncode))
                continue
            chunk = os.read(fd, n - len(buf))
            if not chunk:
                raise RendererError("the renderer closed its wire (exit code {})".format(self.proc.poll()))
            buf += chunk
        return buf

    def _read(self, timeout):
        deadline = time.monotonic() + timeout
        n_head, n_payload = struct.unpack(">II", self._read_exact(8, deadline))
        head = json.loads(self._read_exact(n_head, deadline).decode("utf-8"))
        payload = self._read_exact(n_payload, deadline) if n_payload else b""
        return head, payload

    def _call(self, message, timeout):
        if self.proc is None:
            raise RendererError("the renderer is not running")
        body = json.dumps(message).encode("utf-8")
        try:
            self.proc.stdin.write(struct.pack(">I", len(body)) + body)
            self.proc.stdin.flush()
        except (OSError, ValueError) as exc:
            raise RendererError("the renderer's wire is closed: {}".format(exc))
        head, payload = self._read(timeout)
        if not head.get("ok"):
            raise RendererError(str(head.get("error", head))[-800:])
        return head, payload

    def _recover(self, exc):
        """Start a fresh renderer and rebuild the current scene, or give up after `max_restarts`."""
        if self.restarts >= self.max_restarts:
            raise RendererError("the renderer failed {} times; last: {}".format(self.restarts + 1, exc))
        self.restarts += 1
        self._stop()
        self._start()
        if self.spec is not None:
            self._call({"cmd": "load", "spec": self.spec}, 600.0)

    # -- commands --------------------------------------------------------------------------------------------------
    def load(self, spec):
        self.spec = spec
        try:
            return self._call({"cmd": "load", "spec": spec}, 600.0)[0]
        except RendererError as exc:
            self._recover(exc)                 # (the fresh renderer has loaded the scene)
            return {"ok": True, "recovered": True}

    def render(self, state, raw=False):
        """The JPEG of one frame (and, with `raw`, the renderer's untouched frame as PNG)."""
        msg = {"cmd": "render", "state": state, "raw": bool(raw)}
        try:
            head, payload = self._call(msg, self.render_timeout)
        except RendererError as exc:
            self._recover(exc)
            head, payload = self._call(msg, self.render_timeout)
        self.frames += 1
        self.render_ms += float(head.get("ms", 0.0))
        self.extra_renders += int(head.get("tries", 1)) - 1
        jpeg = payload[:head["jpeg_len"]]
        return (jpeg, payload[head["jpeg_len"]:]) if raw else jpeg

    def canary(self):
        try:
            return self._call({"cmd": "canary"}, 900.0)[0]
        finally:
            self.spec = None

    def info(self):
        return self._call({"cmd": "info"}, 60.0)[0]

    def close(self):
        if self.proc is None:
            return
        try:
            self._call({"cmd": "quit"}, 20.0)
        except RendererError:
            pass
        self._stop()
