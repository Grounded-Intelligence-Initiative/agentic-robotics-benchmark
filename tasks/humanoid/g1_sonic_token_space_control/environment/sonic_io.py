"""Public 50 Hz action and 10 Hz camera interface for G1 SONIC token control."""
import base64
import hashlib
import io
import json
import math

import numpy as np
from PIL import Image

HORIZON = 750
CAMERA_STRIDE = 5
CAMERAS = ("overview", "hand", "side")
PROPRIO = (
    "q", "dq", "base_pos", "base_quat", "base_gyro", "gravity_body",
    "torso_pos", "torso_quat", "left_hand_pos", "right_hand_pos",
)


def stamp_observation(obs):
    """Identify a sensor sample. Both live and alternative samples use this operation."""
    sensor_data = {k: v for k, v in obs.items() if k not in ("observation_id", "last_observation_id")}
    encoded = json.dumps(sensor_data, sort_keys=True, separators=(",", ":"), allow_nan=False)
    obs["observation_id"] = hashlib.sha256(encoded.encode("utf-8")).hexdigest()
    return obs


def validate_action(action, tick, horizon=HORIZON):
    """Validate the whole proposed chunk before applying even its first row."""
    if not isinstance(action, dict) or set(action) != {"rows"}:
        raise ValueError("action must be an object with a rows field")
    rows = action["rows"]
    limit = min(CAMERA_STRIDE - tick % CAMERA_STRIDE, horizon - tick)
    if not isinstance(rows, list) or not 1 <= len(rows) <= limit:
        raise ValueError(f"tick {tick} accepts between 1 and {limit} action rows")
    for i, row in enumerate(rows):
        if not isinstance(row, dict) or set(row) != {"codes", "hand"}:
            raise ValueError(f"row {i} requires codes and hand")
        codes, hand = row["codes"], row["hand"]
        if (not isinstance(codes, list) or len(codes) != 64
                or any(type(x) is not int or not 0 <= x <= 31 for x in codes)):
            raise ValueError(f"row {i} requires 64 integer codes in [0, 31]")
        if (not isinstance(hand, list) or len(hand) != 2
                or any(type(x) not in (int, float) or not 0 <= x <= 1 or not math.isfinite(x) for x in hand)):
            raise ValueError(f"row {i} requires two finite hand closures in [0, 1]")
    return rows


def _camera_frame(session, tick, width, height):
    images = {}
    calibrations = {}
    for name in CAMERAS:
        rgb, calibrations[name] = session.camera_with_calibration(name, width, height)
        output = io.BytesIO()
        Image.fromarray(rgb).save(output, format="PNG", compress_level=2)
        images[name] = base64.b64encode(output.getvalue()).decode("ascii")
    return {"tick": tick, "mime": "image/png", "images": images, "calibration": calibrations}


class LiveEpisode:
    """Bind one reset session to the public wire. Create a new instance after reset."""

    def __init__(self, session):
        self.session = session
        self.last_hand = [0.0, 0.0]
        self.last_observation_id = None
        self._cached_observation = None

    def observe(self):
        if self._cached_observation is not None:
            return self._cached_observation
        state = self.session.state
        tick = state["tick"]
        capture_tick = tick - tick % CAMERA_STRIDE
        fresh = tick == capture_tick
        obs = {
            "protocol": "g1-sonic-rgb/1",
            "task": state["task"].removesuffix("_challenge"),
            "tick": tick,
            "observation": {k: np.asarray(state["observation"][k]).tolist() for k in PROPRIO},
            "vision_tick": capture_tick,
            "vision": _camera_frame(self.session, tick, 128, 96) if fresh else None,
            "planner_vision": _camera_frame(self.session, tick, 480, 360) if fresh else None,
            "decoder_history": np.asarray(self.session.decoder_history()).tolist(),
            "last_hand": self.last_hand.copy(),
            "last_observation_id": self.last_observation_id,
        }
        self._cached_observation = stamp_observation(obs)
        return self._cached_observation

    def advance(self, action, observation_id):
        state = self.session.state
        if state["done"]:
            raise ValueError("episode has ended")
        tick = state["tick"]
        rows = validate_action(action, tick, self.session.horizon)
        for row in rows:
            token = (np.asarray(row["codes"], dtype=np.float32) - 16) / 16
            self.last_hand = list(row["hand"])
            state = self.session.step(token, *self.last_hand)
            if state["done"]:
                break
        self.last_observation_id = observation_id
        self._cached_observation = None
        return state["tick"] - tick
