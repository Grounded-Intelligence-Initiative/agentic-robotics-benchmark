"""Run the image-based feasibility reference and save actual simulator captures."""
import argparse
import json
from pathlib import Path
import sys

TASK = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(TASK/"tests"), str(TASK/"solution")]
import mujoco
import numpy as np
from PIL import Image
from policy import solve
from scene_assets import sync_visuals
from world import World


class LocalClient:
    def __init__(self, world):
        self.world, self.events = world, []

    def call(self, name, *args):
        obs = getattr(self.world, name)(*args)
        self.events.append({"method": name, "arguments": args, **self.world.metrics()})
        return obs

    def observe(self):
        return self.call("observe")

    def act(self, action):
        return self.call("act", action)

    def advance(self):
        return self.call("advance")

    def report(self, frame_id, point):
        return self.call("report", frame_id, point)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--seeds", nargs="+", type=int, default=[101, 102, 103])
    args = p.parse_args()
    kind = json.loads((TASK/"tests/grader_config.json").read_text())["kind"]
    rows = []
    for seed in args.seeds:
        folder = args.out/str(seed)
        folder.mkdir(parents=True, exist_ok=True)
        w = World(kind, seed)
        client = LocalClient(w)
        camera = mujoco.Renderer(w.model, 800, 1200)
        try:
            sync_visuals(w.model, w.data)
            view = mujoco.MjvCamera()
            view.lookat[:] = [0, 0, .6]
            view.distance, view.azimuth, view.elevation = 8.5, 60, -35
            opt = mujoco.MjvOption()
            opt.geomgroup[3] = 0
            camera.update_scene(w.data, camera=view, scene_option=opt)
            Image.fromarray(camera.render().copy()).save(folder/"reviewer-overview.png")
            Image.fromarray(np.concatenate(w.images(), axis=1)).save(folder/"initial-agent-view.png")
            solve(client, kind, artifacts=folder)
            Image.fromarray(np.concatenate(w.images(), axis=1)).save(folder/"final-agent-view.png")
            row = {"kind": kind, "seed": seed, "policy": "scripted_image_reference", **w.metrics()}
            rows.append(row)
            (folder/"result.json").write_text(json.dumps(row, indent=2)+"\n")
            (folder/"interaction.json").write_text(json.dumps(client.events, indent=2)+"\n")
            print(json.dumps(row), flush=True)
        finally:
            camera.close()
            w.close()
    (args.out/"summary.json").write_text(json.dumps(rows, indent=2)+"\n")
    if not all(r["success"] for r in rows):
        raise SystemExit("Reference did not solve every scene; do not claim validation")


if __name__ == "__main__":
    main()
