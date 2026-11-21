"""Author QA: replay a saved ledger, render initial/docked/loaded states only."""
import argparse
import json
from pathlib import Path
import sys

TASK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TASK/"tests"))
from PIL import Image
from pbr_renderer import Renderer
from service import dispatch
from world import SIZE, World


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ledger", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    ledger = json.loads(args.ledger.read_text())
    world = World(ledger["seed"], render=False)
    world.renderer = Renderer(world.model, SIZE, SIZE)

    def capture(label):
        before = world.state_fingerprint()
        pixels = world.image()
        assert before == world.state_fingerprint()
        assert pixels.shape == (384, 384, 3) and pixels.std() > 10
        Image.fromarray(pixels).save(args.output/f"{label}.png")

    try:
        capture("initial")
        for event in ledger["events"]:
            delivered, loaded = world.delivered, world.loaded
            try:
                dispatch(world, event["method"], event["params"])
            except (ValueError, TypeError, KeyError, OverflowError):
                assert "error" in event
            assert world.state_fingerprint() == event["state_fingerprint"]
            if world.delivered != delivered:
                capture(f"dock-{world.delivered}")
            if world.loaded != loaded:
                capture("loaded")
        assert world.state_fingerprint() == ledger["state_fingerprint"]
        print(json.dumps({"images": len(list(args.output.glob("*.png"))), "replay": "exact", **world.metrics()}))
    finally:
        world.close()


if __name__ == "__main__":
    main()
