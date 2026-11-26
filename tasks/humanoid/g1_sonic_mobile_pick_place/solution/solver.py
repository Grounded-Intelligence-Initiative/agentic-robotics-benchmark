"""ALE JSONL interface for continuous mobile manipulation."""
import argparse
import base64
from dataclasses import dataclass
from io import BytesIO
import json
import sys

import numpy as np
from PIL import Image


@dataclass
class Observation:
    tick: int
    task: str
    proprio: dict
    history: np.ndarray
    last_hand: list
    images: dict
    cameras: dict
    fresh: bool


def observation(packet, mode):
    tick = packet["tick"]
    if packet["protocol"] != "g1-sonic-mobile-rgb/1" or packet["task"] not in ("box_to_pad", "cylinder_to_pad"):
        raise ValueError("expected public G1 tabletop observations")
    frame = packet["vision" if mode == "v4" else "planner_vision"]
    images, cameras = {}, {}
    if frame is not None:
        if frame["tick"] != tick or frame["mime"] != "image/png":
            raise ValueError("expected synchronized native PNGs")
        cameras = frame["calibration"]
        for name in ("overview", "hand", "side"):
            with Image.open(BytesIO(base64.b64decode(frame["images"][name], validate=True))) as image:
                cal = cameras[name]
                if image.mode != "RGB" or image.format != "PNG" or image.size != (cal["width"], cal["height"]):
                    raise ValueError("image format or optical calibration mismatch")
                images[name] = image.copy()
    history = np.asarray(packet["decoder_history"], np.float32)
    if history.shape != (930,) or not np.isfinite(history).all():
        raise ValueError("expected the current 930-element SONIC history")
    return Observation(tick, packet["task"], packet["observation"], history, packet["last_hand"],
                       images, cameras, frame is not None)


class Episode:
    """Stage alternative replies; commit the state identified by the next observation."""
    def __init__(self, policy, mode):
        self.policy, self.mode = policy, mode
        self.reset()

    def reset(self):
        self.policy.reset()
        self.before = self.policy.snapshot()
        self.pending = {}
        self.tick = None

    def act(self, request):
        packet = request["obs"]
        tick, key = packet["tick"], packet["observation_id"]
        if tick != request["t"]:
            raise ValueError("ALE step and sensor tick disagree")
        if tick != self.tick:
            if self.tick is None:
                if tick != 0 or packet["last_observation_id"] is not None:
                    raise ValueError("episode must begin at tick zero")
            else:
                if tick <= self.tick:
                    raise ValueError("nonmonotonic control tick")
                self.before = self.pending[packet["last_observation_id"]][0]
            self.tick, self.pending = tick, {}
        if key not in self.pending:
            self.policy.restore(self.before)
            public = observation(packet, self.mode)
            rows = self.policy.act(public)
            if not 1 <= len(rows) <= 5 - tick % 5:
                raise ValueError("policy chunk crossed a camera boundary")
            self.pending[key] = (self.policy.snapshot(), {"rows": rows})
        return self.pending[key][1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    from mobile_policy import MobilePolicy
    episode = Episode(MobilePolicy(), "v4")
    print("READY", flush=True)
    for line in sys.stdin:
        try:
            request = json.loads(line)
            kind = request["type"]
            if kind == "act":
                reply = {"action": episode.act(request)}
            elif kind == "reset":
                episode.reset()
                reply = {"ok": True}
            elif kind == "close":
                print('{"ok":true}', flush=True)
                break
            else:
                raise ValueError("unknown operation")
        except (KeyError, TypeError, ValueError) as exc:
            reply = {"error": str(exc)[:200]}
        print(json.dumps(reply, allow_nan=False, separators=(",", ":")), flush=True)


if __name__ == "__main__":
    main()
