"""Public 50 Hz action and 10 Hz camera interface for G1 SONIC token control."""
import base64
import hashlib
import io
import json
import math

import numpy as np
from PIL import Image

from sonic_sim.mobile import HORIZON
CAMERA_STRIDE = 5
CAMERAS = ("overview", "hand", "side")
PROPRIO = ("q", "dq", "base_gyro", "gravity_body")


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


def _png(array):
    output = io.BytesIO()
    Image.fromarray(array).save(output, format="PNG", compress_level=2)
    return base64.b64encode(output.getvalue()).decode("ascii")


def _camera_frame(session, tick, width, height, rgbd=False):
    frame = dict(tick=tick, mime="image/png", images={})
    if rgbd:
        frame.update(depth_mm={}, intrinsics={})
    for name in CAMERAS:
        if rgbd:
            rgb, meters, pinhole = session.rgbd(name, width, height)
            valid = np.isfinite(meters) & (meters > 0) & (meters < 12.)
            depth = np.zeros(meters.shape, dtype=np.uint16)
            depth[valid] = np.maximum(1., np.rint(meters[valid]*1000.)).astype(np.uint16)
            frame["depth_mm"][name] = _png(depth)
            frame["intrinsics"][name] = pinhole
        else:
            rgb = session.camera(name, width, height)
        frame["images"][name] = _png(rgb)
    return frame


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
            "protocol": "g1-sonic-visual-delivery/1",
            "task": state["task"],
            "tick": tick,
            "observation": {k: np.asarray(state["observation"][k]).tolist() for k in PROPRIO},
            "vision_tick": capture_tick,
            "vision": _camera_frame(self.session, tick, 128, 96) if fresh else None,
            "planner_vision": _camera_frame(self.session, tick, 320, 240, rgbd=True) if fresh else None,
            "last_hand": self.last_hand.copy(),
            "last_observation_id": self.last_observation_id,
        }
        self._cached_observation = stamp_observation(obs)
        return self._cached_observation

    def opposite_cue(self):
        """Render the other light from this exact state; retain the live sensor timings and depths."""
        live = self.observe()
        if live["vision"] is None:
            return None
        counterfactual = self.session.opposite_cue_rgb(((128, 96), (320, 240)))
        if counterfactual is None:
            return None
        obs = live.copy()
        for key, frames in zip(("vision", "planner_vision"), counterfactual):
            obs[key] = dict(live[key], images={name: _png(rgb) for name, rgb in zip(CAMERAS, frames)})
        return stamp_observation(obs)

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
