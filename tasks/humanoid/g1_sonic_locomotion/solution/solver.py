"""ALE solver for goal-relative G1 locomotion through the released SONIC encoder."""
import json
import os
from pathlib import Path
import sys

from gait import GaitPolicy
from sonic_sim.encoder import ReferenceEncoder


def main():
    assets = Path(os.environ.get("SONIC_ASSETS", "/opt/sonic/assets"))
    policy = GaitPolicy(ReferenceEncoder(assets / "policy" / "model_encoder.onnx"))
    print("READY", flush=True)
    for line in sys.stdin:
        try:
            request = json.loads(line)
            kind = request["type"]
            if kind == "act":
                obs = request["obs"]
                if obs["protocol"] != "g1-sonic-locomotion/1" or obs["tick"] != request["t"]:
                    raise ValueError("expected a synchronized public G1 locomotion observation")
                codes = policy.codes(obs["tick"], obs["command"], obs["observation"])
                reply = {"action": {"rows": [{"codes": codes.tolist()}]}}
            elif kind in ("reset", "close"):
                reply = {"ok": True}
            else:
                raise ValueError("unknown request type")
        except (KeyError, TypeError, ValueError) as exc:
            reply = {"error": str(exc)[:200]}
        print(json.dumps(reply, allow_nan=False, separators=(",", ":")), flush=True)
        if reply == {"ok": True} and kind == "close":
            break


if __name__ == "__main__":
    main()
