"""Run a controller in-process on one seed and save PNG frames of the cell (CPU rendering).

    python3 ~/tools/render_episode.py --controller /home/user/submission/controller.py --seed 1001 \
        --every 20 --out /tmp/frames

The controller module must define `make()` returning an object with `reset()` and `act(t, obs)`
returning an action dict, or the module may define `act(t, obs)` itself. Frames are a top view and an
oblique view side by side.
"""

import argparse
import importlib.util
import os
import sys

sys.path.insert(0, os.path.join(os.path.expanduser("~"), "cell"))
import numpy as np  # noqa: E402
import mujoco  # noqa: E402
import cable_env as E  # noqa: E402


def load(path):
    spec = importlib.util.spec_from_file_location("controller", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if hasattr(mod, "make"):
        ctl = mod.make()
        ctl.reset()
        return ctl.act
    return mod.act


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", required=True)
    ap.add_argument("--seed", type=int, default=1001)
    ap.add_argument("--every", type=int, default=20)
    ap.add_argument("--out", default="/tmp/frames")
    a = ap.parse_args()
    import cv2
    os.makedirs(a.out, exist_ok=True)
    act = load(a.controller)
    env = E.CableRouting()
    obs = env.reset(a.seed)
    r = mujoco.Renderer(env.model, 360, 480)

    def frame(name):
        imgs = []
        for elev, dist in ((-89.0, 0.95), (-35.0, 0.75)):
            cam = mujoco.MjvCamera()
            cam.lookat[:] = [0.0, 0.0, 0.0]
            cam.distance, cam.elevation, cam.azimuth = dist, elev, 90.0
            r.update_scene(env.data, cam)
            imgs.append(r.render())
        cv2.imwrite(os.path.join(a.out, name), np.concatenate(imgs, axis=1)[:, :, ::-1])

    for t in range(E.MAX_STEPS):
        if t % a.every == 0:
            frame("t%04d.png" % t)
        obs, done = env.step(act(t, obs))
        if done:
            break
    res = env.finish()
    frame("final.png")
    print(res)


if __name__ == "__main__":
    main()
