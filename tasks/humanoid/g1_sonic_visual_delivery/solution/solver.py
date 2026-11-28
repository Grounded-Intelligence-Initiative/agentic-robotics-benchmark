"""Persistent public JSONL reference solver for visual delivery."""
import json
import os
from pathlib import Path
import sys

from camera_input import observe
from delivery_policy import DeliveryPolicy


class Episode:
    def __init__(self, policy):
        self.policy = policy
        self.reset()

    def reset(self):
        self.policy.reset()
        self.tick = None
        self.before = self.policy.snapshot()
        self.pending = {}

    def act(self, req):
        p = req["obs"]
        tick, key = p["tick"], p["observation_id"]
        if req["t"] != tick:
            raise ValueError("request and observation ticks differ")
        if self.tick != tick:
            if self.tick is None:
                if tick or p["last_observation_id"] is not None:
                    raise ValueError("expected a new episode")
            else:
                if tick <= self.tick:
                    raise ValueError("nonmonotonic tick")
                self.before = self.pending[p["last_observation_id"]][0]
            self.tick, self.pending = tick, {}
        if key not in self.pending:
            self.policy.restore(self.before)
            proprio, rgb, views = observe(p)
            rows = self.policy.act(tick, proprio, rgb, views, p["last_hand"])
            self.pending[key] = (self.policy.snapshot(), {"rows": rows})
        return self.pending[key][1]


def main():
    episode = Episode(DeliveryPolicy(Path(os.environ.get("SONIC_ASSETS", "/opt/sonic/assets"))))
    print("READY", flush=True)
    for line in sys.stdin:
        try:
            req = json.loads(line)
            if req["type"] == "act":
                reply = {"action": episode.act(req)}
            elif req["type"] == "reset":
                episode.reset()
                reply = {"ok": True}
            elif req["type"] == "close":
                print('{"ok":true}', flush=True)
                return
            else:
                raise ValueError("unknown operation")
        except (TypeError, KeyError, ValueError) as exc:
            reply = {"error": str(exc)[:160]}
        print(json.dumps(reply, allow_nan=False, separators=(",", ":")), flush=True)


if __name__ == "__main__":
    main()
