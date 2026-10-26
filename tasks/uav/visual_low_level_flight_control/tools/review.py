"""CPU reviewer demo using only public images for the scripted controller."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

TASK = Path(__file__).resolve().parents[1]
os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
os.environ.setdefault("LP_NUM_THREADS", "2")
sys.path[:0] = [str(TASK/"verify"), str(TASK/"oracle")]

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from world import World
from policy import ControllerSession, decode


def digest():
    result = hashlib.sha256()
    for path in sorted(TASK.rglob("*")):
        if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc":
            result.update(str(path.relative_to(TASK)).encode()+b"\0"+path.read_bytes()+b"\0")
    return result.hexdigest()


class Camera:
    def __init__(self, world, width=1280, height=800):
        self.world = world
        self.renderer = mujoco.Renderer(world.model, height, width)
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [0, 0, .8]
        self.camera.distance, self.camera.azimuth, self.camera.elevation = 7.2, 125, -25

    def frame(self):
        option = mujoco.MjvOption()
        option.geomgroup[3] = 0
        self.renderer.update_scene(self.world.data, camera=self.camera, scene_option=option)
        return Image.fromarray(self.renderer.render().copy())

    def close(self):
        self.renderer.close()


def poster(world, obs, output):
    camera = Camera(world)
    scene = camera.frame()
    scene.save(output/"scene.png")
    detail = Camera(world, 400, 280)
    detail.camera.lookat[:] = world.position()
    detail.camera.distance, detail.camera.azimuth, detail.camera.elevation = .32, 135, -30
    closeup = detail.frame()
    closeup.save(output/"crazyflie-detail.png")
    detail.close()
    canvas = Image.new("RGB", (1280, 1040), "#161a1d")
    canvas.paste(scene, (0, 120))
    draw = ImageDraw.Draw(canvas)
    normal = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    bold = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    draw.text((32, 20), "Crazyflie / Visual Flight Control", font=ImageFont.truetype(bold, 36), fill="#f4f5f6")
    draw.text((32, 74), "Camera images > controller > three gates > stable hover",
              font=ImageFont.truetype(normal, 23), fill="#bdc9cd")
    canvas.paste(closeup, (32, 640))
    draw.rectangle((31, 639, 433, 921), outline="#9caab0", width=2)
    draw.text((32, 607), "Unscaled Crazyflie 2 / same simulation", font=ImageFont.truetype(normal, 18), fill="white")
    top = Image.fromarray(decode(obs)).resize((280, 280))
    canvas.paste(top, (968, 640))
    draw.text((968, 607), "Actual agent camera", font=ImageFont.truetype(normal, 18), fill="white")
    draw.text((32, 946), "27 g nominal model  |  CPU MuJoCo  |  Image-only scripted reference",
              font=ImageFont.truetype(normal, 22), fill="#dae2e5")
    draw.text((32, 988), "Simulator capture. Custom attitude servo; not a hardware or frontier-model result.",
              font=ImageFont.truetype(normal, 19), fill="#a7b7bd")
    canvas.save(output/"poster.png")
    camera.close()


def run(seed, output, video=False, mode="reference"):
    output.mkdir(parents=True, exist_ok=True)
    world = World("flight", seed)
    controller = ControllerSession()
    records = []
    writer = camera = None
    if video:
        import imageio.v2 as imageio
        writer = imageio.get_writer(output/"demo.mp4", fps=5, codec="libx264", quality=8, macro_block_size=1)
        camera = Camera(world, 960, 600)
    rng = np.random.default_rng(seed ^ 0x971)
    try:
        obs = world.observe()
        Image.fromarray(np.concatenate(world.images(), axis=1)).save(output/"initial-agent-view.png")
        while not world.done:
            if mode == "reference":
                action = controller.act(world.steps, obs)
            elif mode == "hover_only":
                action = [.027*9.81/.175-1, 0, 0, 0]
            else:
                action = rng.uniform(-1, 1, 4).tolist()
            obs = world.act(action)
            records.append({"action": action, "frame_id": obs["frame_id"], **world.metrics()})
            if world.steps == 26:
                poster(world, obs, output)
                Image.fromarray(np.concatenate(world.images(), axis=1)).save(output/"agent-view.png")
            if writer is not None:
                frame = camera.frame()
                draw = ImageDraw.Draw(frame)
                draw.rectangle((0, 556, 960, 600), fill="#161a1d")
                draw.text((16, 566), f"{world.data.time:.1f}s  |  Gates: {world.passed_gates}/3  |  Contacts: {world.collisions}  |  Reviewer view",
                          font=ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 18), fill="white")
                writer.append_data(np.asarray(frame))
        Image.fromarray(np.concatenate(world.images(), axis=1)).save(output/"final-agent-view.png")
        result = {"seed": seed, "mode": mode, "task_sha256": digest(), **world.metrics()}
        (output/"result.json").write_text(json.dumps(result, indent=2)+"\n")
        (output/"actions.json").write_text(json.dumps(records, indent=2)+"\n")
        return result
    finally:
        if writer is not None:
            writer.close()
            camera.close()
        world.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", nargs="+", type=int, default=[101, 102, 103])
    parser.add_argument("--assets", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--mode", choices=["reference", "hover_only", "random"], default="reference")
    args = parser.parse_args()
    if args.assets:
        os.environ["VISUAL_ROBOTICS_ASSETS"] = str(args.assets.resolve())
    results = []
    for i, seed in enumerate(args.seeds):
        result = run(seed, args.out/str(seed), video=args.video and i == 0, mode=args.mode)
        results.append(result)
        print(json.dumps(result), flush=True)
    (args.out/"summary.json").write_text(json.dumps(results, indent=2)+"\n")
    if args.mode == "reference" and not all(row["success"] == 1 for row in results):
        raise SystemExit("reference failed; do not claim successful validation")


if __name__ == "__main__":
    main()
