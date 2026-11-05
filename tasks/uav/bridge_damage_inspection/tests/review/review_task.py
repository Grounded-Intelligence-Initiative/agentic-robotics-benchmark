"""Reproducible CPU scenes, calibrated successes, and actual camera captures."""
import argparse
import base64
import json
from pathlib import Path
import sys

TASK = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(TASK/"environment/setup/runtime"), str(TASK/"solution")]
import mujoco
import numpy as np
from PIL import Image
from bridge_policy import solve
from long_bridge import BridgeWorld
from pbr_renderer import Renderer


class LocalClient:
    def __init__(self, world, folder):
        self.world, self.folder, self.events = world, folder, []

    def call(self, method, *args, **kwargs):
        obs = getattr(self.world, method)(*args, **kwargs)
        self.events.append({"method": method, "arguments": args or kwargs,
                            "frame_id": obs["frame_id"], **self.world.metrics()})
        if obs["frame_id"] % 3 == 1:
            (self.folder/"frames").mkdir(exist_ok=True)
            (self.folder/"frames"/f'{obs["frame_id"]:04d}.png').write_bytes(base64.b64decode(obs["images"][0]))
        return obs

    def observe(self):
        return self.call("observe")

    def act(self, action):
        return self.call("act", action)

    def report(self, frame_id, point):
        return self.call("report", frame_id, point)

    def advance(self):
        return self.call("advance")


def capture(world, folder):
    camera = Renderer(world.model, 700, 1200)
    try:
        view = mujoco.MjvCamera()
        view.lookat[:] = [world.length/2, 0, 5]
        view.distance, view.azimuth, view.elevation = world.length*1.06, 105, -33
        option = mujoco.MjvOption()
        option.geomgroup[3:5] = 0
        camera.update_scene(world.data, camera=view, scene_option=option)
        Image.fromarray(camera.render()).save(folder/"reviewer-overview.png")
        view.lookat[:] = [world.length*.32, 0, 7.5]
        view.distance, view.azimuth, view.elevation = 22, 135, -30
        camera.update_scene(world.data, camera=view, scene_option=option)
        Image.fromarray(camera.render()).save(folder/"reviewer-detail.png")
    finally:
        camera.close()
    Image.fromarray(world.images()[0]).save(folder/"initial-agent-view.png")
    pose = world.data.qpos.copy()
    for name, component in (("damage", world.defects[0]), ("repair", world.repairs[0])):
        world.data.qpos[:2] = world.component_point(*component)
        mujoco.mj_forward(world.model, world.data)
        Image.fromarray(world.images()[0]).save(folder/f"{name}-agent-view.png")
    world.data.qpos[:] = pose
    mujoco.mj_forward(world.model, world.data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--seeds", type=int, nargs="+", default=[101, 102, 103])
    parser.add_argument("--preview-only", action="store_true")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    rows = []
    for seed in args.seeds:
        folder = args.out/str(seed)
        folder.mkdir(exist_ok=False)
        world = BridgeWorld(seed)
        try:
            capture(world, folder)
            if not args.preview_only:
                client = LocalClient(world, folder)
                solve(client, artifacts=folder)
                (folder/"interaction.json").write_text(json.dumps(client.events, indent=2)+"\n")
            row = {"seed": seed, "scenario": world.scenario,
                   "reference": "RGB detector with privileged actuator calibration",
                   "preview_only": args.preview_only, **world.metrics()}
            rows.append(row)
            (folder/"result.json").write_text(json.dumps(row, indent=2)+"\n")
            print(json.dumps(row), flush=True)
        finally:
            world.close()
    (args.out/"summary.json").write_text(json.dumps(rows, indent=2)+"\n")
    if not args.preview_only and not all(row["success"] for row in rows):
        raise SystemExit("At least one reference failed; keep the failure and repair before claiming feasibility.")


if __name__ == "__main__":
    main()
