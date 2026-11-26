"""Host-side native G1/Dex3 token-control session."""
from pathlib import Path

import numpy as np

from .cameras import CAMERAS
from .decoder import Decoder, DecoderHistory
from .physics import G1Task, TASKS


def _integer(value, name, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer in [{minimum}, {maximum}]")
    return value


class BenchSession:
    """Advance one free-base MuJoCo episode with normalized SONIC token rows.

    `assets_root` contains policy/model_decoder.onnx and menagerie_g1/.
    A new instance starts at tick zero. The host reads scoring fields from
    state["metrics"] and gives the solver only the public proprioception and RGB.
    """

    def __init__(self, task, seed, assets_root, horizon=750):
        self.horizon = _integer(horizon, "horizon", 1, 100_000)
        root = Path(assets_root).resolve()
        self._robot_root = root / "menagerie_g1"
        self._decoder = Decoder(root / "policy" / "model_decoder.onnx")
        self._scene = None
        self.reset(task, seed)

    def reset(self, task, seed):
        """Start a fresh seeded episode with independently reset decoder history."""
        if not isinstance(task, str) or task not in TASKS:
            raise ValueError(f"unknown task {task!r}")
        _integer(seed, "seed", 0, 2**64 - 1)
        scene = G1Task(task, seed, self._robot_root)
        self.close()
        self._scene = scene
        self.task, self.tick = task, 0
        self._history = DecoderHistory()
        self.last_target = None
        return self.state

    @property
    def state(self):
        """Return copied public proprioception and native host scoring at this tick."""
        if self._scene is None:
            raise RuntimeError("session is closed")
        metrics = self._scene.metrics()
        reason = None
        if metrics["success"]:
            reason = "success"
        elif not metrics["upright"]:
            reason = "fall"
        elif self.tick >= self.horizon:
            reason = "time_limit"
        return dict(task=self.task, tick=self.tick, seconds=self.tick / 50,
                    observation=self._scene.observe(), metrics=metrics,
                    done=reason is not None, reason=reason)

    def decoder_history(self):
        """Return the 930 non-token input values for a prospective decoder call.

        The next committed step uses the same preprocessing. Reading this array
        leaves the decoder history and native simulation unchanged.
        """
        if self._scene is None:
            raise RuntimeError("session is closed")
        obs = self._scene.observe()
        packed, _ = self._history.observation(np.zeros(64, dtype=np.float32),
                                              obs["q"], obs["dq"], obs["base_gyro"],
                                              obs["gravity_body"])
        return packed[64:].copy()

    def step(self, token, left_hand, right_hand):
        """Consume one finite 64-vector and left/right closures in [0, 1] at 50 Hz."""
        row = np.asarray(token)
        if row.shape != (64,) or row.dtype.kind not in "fiu":
            raise ValueError("token must contain 64 real numbers")
        with np.errstate(over="ignore"):
            row = np.asarray(row, dtype=np.float32)
        if not np.isfinite(row).all():
            raise ValueError("token must fit finite float32")
        fingers = np.asarray((left_hand, right_hand))
        if (fingers.shape != (2,) or fingers.dtype.kind not in "fiu"
                or not np.isfinite(fingers).all() or np.any((fingers < 0) | (fingers > 1))):
            raise ValueError("hand closures must each be a finite number in [0, 1]")
        state = self.state
        if state["done"]:
            raise ValueError(f"episode ended with {state['reason']}")
        obs = state["observation"]
        target = self._decoder.decode_step(row, self._history, obs["q"], obs["dq"],
                                           obs["base_gyro"], obs["gravity_body"])
        self._scene.step(target, fingers)
        self.last_target = target
        self.tick += 1
        return self.state

    def camera(self, name, width=256, height=192):
        """Capture one fixed world-frame camera as an independent H×W×3 RGB8 array."""
        if self._scene is None:
            raise RuntimeError("session is closed")
        _integer(width, "width", 32, 1280)
        _integer(height, "height", 32, 960)
        return self._scene.render(width=width, height=height, camera=name)

    def camera_with_calibration(self, name, width=256, height=192):
        """Capture RGB and the pinhole camera corresponding to this exact raster."""
        rgb = self.camera(name, width, height)
        return rgb, self._scene.calibration(width, height)

    def close(self):
        if self._scene is not None:
            self._scene.close()
            self._scene = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
