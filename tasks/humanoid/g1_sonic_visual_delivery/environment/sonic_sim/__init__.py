"""Host-side native G1/Dex3 token-control session."""
from pathlib import Path

import numpy as np

from .cameras import CAMERAS
from .decoder import Decoder, DecoderHistory
from .mobile import DeliveryTask, HORIZON


def _integer(value, name, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer in [{minimum}, {maximum}]")
    return value


class DeliverySession:
    """Advance one free-base MuJoCo episode with normalized SONIC token rows.

    `assets_root` contains policy/model_decoder.onnx and menagerie_g1/.
    A new instance starts at tick zero. The host reads scoring fields from
    state["metrics"] and gives the solver only the public proprioception and RGB.
    """

    def __init__(self, task, seed, assets_root, horizon=HORIZON, *, start_mode=None, shape=None, target_side=None):
        self.horizon = _integer(horizon, "horizon", 1, 100_000)
        root = Path(assets_root).resolve()
        self._robot_root = root / "menagerie_g1"
        self._decoder = Decoder(root / "policy" / "model_decoder.onnx")
        self._scene = None
        self.reset(task, seed, start_mode=start_mode, shape=shape, target_side=target_side)

    def reset(self, task, seed, *, start_mode=None, shape=None, target_side=None):
        """Start a fresh seeded episode with independently reset decoder history."""
        if task != "visual_delivery":
            raise ValueError(f"unknown task {task!r}")
        _integer(seed, "seed", 0, 2**64 - 1)
        scene = DeliveryTask(seed, self._robot_root, start_mode=start_mode, shape=shape, target_side=target_side)
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
        elif metrics["wrong_pad"]:
            reason = "wrong_pad"
        elif not metrics["approach_valid"]:
            reason = "approach_violation"
        elif not metrics["upright"]:
            reason = "fall"
        elif self.tick >= self.horizon:
            reason = "time_limit"
        return dict(task=self.task, tick=self.tick, seconds=self.tick / 50,
                    observation=self._scene.observe(), metrics=metrics,
                    done=reason is not None, reason=reason)

    @property
    def last_substeps(self):
        return self._scene.last_substeps

    @property
    def episode_geometry(self):
        return self._scene.geometry()

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

    def rgbd(self, name, width, height):
        rgb, depth_m = self._scene.render(width=width, height=height, camera=name, rgbd=True)
        intrinsics = self._scene.calibration(width, height)
        return rgb, depth_m, intrinsics

    def opposite_cue_rgb(self, views):
        scene = self._scene
        ids = [scene.pad_geom, scene.other_pad_geom]
        rgba = scene.model.geom_rgba[ids].copy()
        scene.model.geom_rgba[ids] = rgba[::-1]
        try:
            return [[self.camera(name, width, height) for name in CAMERAS] for width, height in views]
        finally:
            scene.model.geom_rgba[ids] = rgba

    def close(self):
        if self._scene is not None:
            self._scene.close()
            self._scene = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
