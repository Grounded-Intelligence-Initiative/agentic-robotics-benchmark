"""PRIVILEGED v3 engine oracle. NOT an image-only participant solution.

ALE stages solution/ only for its oracle harness. An exact initial-frame key
selects author-only seed metadata, then a private world mirrors ordinary socket
actions. No secret endpoint, environment identity claim or participant truth
channel is installed. Unknown images or stale source hashes fail closed.
"""
import base64
import hashlib
import io
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/"private"))
from PIL import Image
from client import Client
from privileged import Controller, Session
from world import VERSION, World


def main():
    manifest = json.loads((ROOT/"initial_fingerprints.json").read_text())
    if manifest["version"] != VERSION or manifest.get("privileged") is not True:
        raise RuntimeError("wrong privileged reference manifest")
    for name, expected in manifest["runtime_sha256"].items():
        if hashlib.sha256((ROOT/"private"/name).read_bytes()).hexdigest() != expected:
            raise RuntimeError(f"stale initial-image manifest: {name}")
    client = Client()
    obs = client.observe()
    pixels = Image.open(io.BytesIO(base64.b64decode(obs["images"][0]))).convert("RGB").tobytes()
    digest = hashlib.sha256(pixels).hexdigest()
    matches = [row for row in manifest["scenes"] if row["image_sha256"] == digest]
    if len(matches) != 1:
        raise RuntimeError("unknown/ambiguous initial image; regenerate author fingerprints in the pinned image")
    world = World(matches[0]["seed"], render=False)
    try:
        world.observe()  # mirror the already-sent initial request, without sending another
        if any(obs[key] != value for key, value in world.status().items()):
            raise RuntimeError("initial service/mirror status mismatch")
        session = Session(world, client)
        session.events.append({"method": "observe", "params": {}, "frame_id": world.frame_id,
                               "steps": 0, "delivered": 0, "state_fingerprint": world.state_fingerprint()})
        Controller(world, session.call).solve()
        print(json.dumps({"reference": "PRIVILEGED v3; not image-only", **world.metrics(),
                          "requests": len(session.events)}), flush=True)
    finally:
        world.close()


if __name__ == "__main__":
    main()
