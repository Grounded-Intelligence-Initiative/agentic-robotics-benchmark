"""Author-only generation of initial-image keys for ALL fixed hidden scenes.

Write to an evidence directory, then review/copy the JSON into solution/. This
contains privileged seed metadata and must never be staged in participant setup.
No success-based filtering or fallback/nearest-image matching is performed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

TASK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TASK/"tests"))
from world import VERSION, World


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    hidden = json.loads((TASK/"tests/hidden_scenes.json").read_text())["seeds"]
    names = ("world.py", "service.py", "geometry.py", "scene_assets.py", "pbr_scene.py", "pbr_renderer.py")
    rows = []
    args.output.mkdir(parents=True, exist_ok=False)
    for index, seed in enumerate(hidden):
        world = World(seed, render=True)
        try:
            before = world.state_fingerprint()
            image = world.image()
            assert before == world.state_fingerprint(), "render changed integration state"
            digest = hashlib.sha256(image.tobytes()).hexdigest()
            from PIL import Image
            Image.fromarray(image).save(args.output/f"initial-{index:02d}.png")
            rows.append({"seed": seed, "scene_index": index, "image_sha256": digest})
            print(json.dumps(rows[-1]), flush=True)
        finally:
            world.close()
    assert len({r["image_sha256"] for r in rows}) == len(hidden), "ambiguous initial frames"
    result = {"version": VERSION, "privileged": True,
              "runtime_sha256": {name: hashlib.sha256((TASK/"tests"/name).read_bytes()).hexdigest() for name in names},
              "scenes": rows}
    (args.output/"initial_fingerprints.json").write_text(json.dumps(result, indent=2)+"\n")


if __name__ == "__main__":
    main()
