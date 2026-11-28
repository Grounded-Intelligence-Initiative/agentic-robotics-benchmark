"""JSONL entry point for the RGB-D shutter navigation reference."""

import json
import os
from pathlib import Path
import sys

from sonic_sim.encoder import ReferenceEncoder
from temporal import TemporalPolicy as VisualPolicy


def main():
    assets = Path(os.environ.get("SONIC_ASSETS", "/opt/sonic/assets"))
    policy = VisualPolicy(ReferenceEncoder(assets / "policy/model_encoder.onnx"))
    print("READY", flush=True)
    for line in sys.stdin:
        kind = None
        try:
            request = json.loads(line)
            kind = request["type"]
            if kind == "act":
                obs = request["obs"]
                if (
                    obs["protocol"] != "g1-sonic-visual-navigation/1"
                    or request["t"] != obs["tick"]
                ):
                    raise ValueError(
                        "expected a synchronized visual navigation observation"
                    )
                result = {"action": policy.act(obs)}
            elif kind == "reset":
                policy.reset()
                result = {"ok": True}
            elif kind == "close":
                result = {"ok": True}
            else:
                raise ValueError("unknown request type")
        except (ValueError, KeyError, TypeError) as exc:
            result = {"error": str(exc)[:200]}
        print(json.dumps(result, allow_nan=False, separators=(",", ":")), flush=True)
        if kind == "close":
            break


if __name__ == "__main__":
    main()
