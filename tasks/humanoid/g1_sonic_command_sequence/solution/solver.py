"""Generate native SONIC rows from the latest revealed command and measured robot state."""
import json
import os
from pathlib import Path
import sys

from gait import GaitPolicy
from sonic_sim.encoder import ReferenceEncoder


def main():
    assets = Path(os.environ.get("SONIC_ASSETS", "/opt/sonic/assets"))
    gait = GaitPolicy(ReferenceEncoder(assets / "policy/model_encoder.onnx"))
    print("READY", flush=True)
    for line in sys.stdin:
        kind = None
        try:
            request = json.loads(line)
            kind = request["type"]
            if kind == "act":
                obs = request["obs"]
                if obs["protocol"] != "g1-sonic-command-sequence/1" or obs["tick"] != request["t"]:
                    raise ValueError("expected a synchronized G1 command-sequence observation")
                codes = gait.codes(obs["stage_tick"], obs["command"], obs["observation"])
                reply = {"action": {"rows": [{"codes": codes.tolist()}]}}
            elif kind in ("reset", "close"):
                reply = {"ok": True}
            else:
                raise ValueError("unknown request type")
        except (KeyError, TypeError, ValueError) as exc:
            reply = {"error": str(exc)[:200]}
        print(json.dumps(reply, allow_nan=False, separators=(",", ":")), flush=True)
        if kind == "close" and reply == {"ok": True}:
            break


if __name__ == "__main__":
    main()
