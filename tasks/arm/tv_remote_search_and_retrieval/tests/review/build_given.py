"""Writes what the agent is given (environment/setup/payload/given/) from the grader's simulator:

* robot/panda_on_rail.xml: the robot alone (remote_env.build_robot_xml), its mesh paths set to
  where the image has the robosuite asset files;
* survey/site_N/: the first observation of three episodes that are not graded, as the wire
  delivers it (observation.json) and as pictures (PNG; the depth ones 16 bit, millimetres);
* remote/: the remote's model and two pictures of it.

Nothing else of the room goes to the agent: no mesh of the furniture, no simulator, no episode
rule. Run with the room's assets in tests/room:

    MUJOCO_GL=egl python3 tests/review/build_given.py
"""

import json
import os
import shutil
import sys

import cv2
import numpy as np

TASK = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(TASK, "tests"))
os.environ.setdefault("REMOTE_ROOM_DIR", os.path.join(TASK, "tests", "room"))

import mujoco  # noqa: E402
import remote_env as E  # noqa: E402

OUT = os.path.join(TASK, "environment", "setup", "payload", "given")
IMAGE_ASSETS = "/opt/venv/lib/python3.12/site-packages/robosuite/models/assets"
# Survey episodes: seeds far from the graded ones (tests/hidden/episodes.json holds none of them).
SURVEY = ((91001, "left_cushion"), (91002, "lower_drawer"), (91003, "under_sofa"))


def robot():
    folder = os.path.join(OUT, "robot")
    os.makedirs(folder, exist_ok=True)
    xml = E.build_robot_xml().replace(E._asset_root(), IMAGE_ASSETS)
    with open(os.path.join(folder, "panda_on_rail.xml"), "w") as handle:
        handle.write(xml + "\n")


def survey():
    with open(os.path.join(TASK, "tests", "hidden", "episodes.json")) as handle:
        graded = {int(item["seed"]) for item in json.load(handle)["episodes"]}
    env = E.Env()
    for index, (seed, spot) in enumerate(SURVEY, 1):
        assert seed not in graded
        obs = env.reset(E.make_episode(seed, spot))
        assert not env.progress["seen"]
        folder = os.path.join(OUT, "survey", "site_{}".format(index))
        os.makedirs(folder, exist_ok=True)
        with open(os.path.join(folder, "observation.json"), "w") as handle:
            json.dump(obs, handle)
        for name, image in obs["images"].items():
            rgb, depth = E.decode_image(image["rgb"]), E.decode_image(image["depth"])
            cv2.imwrite(os.path.join(folder, name + "_rgb.png"), rgb[:, :, ::-1])
            cv2.imwrite(os.path.join(folder, name + "_depth_mm.png"), depth)
    env.close()


def remote():
    folder = os.path.join(OUT, "remote")
    os.makedirs(folder, exist_ok=True)
    room = E.room_dir()
    for name in ("remote.obj", "remote.png"):
        shutil.copyfile(os.path.join(room, name), os.path.join(folder, name))
    model = mujoco.MjModel.from_xml_string("""<mujoco>
  <visual><global offwidth="640" offheight="480"/><headlight ambient="0.5 0.5 0.5" diffuse="0.5 0.5 0.5"/></visual>
  <asset><mesh name="remote" file="{r}/remote.obj"/><texture name="remote" type="2d" file="{r}/remote.png"/>
    <material name="remote" texture="remote" specular="0.4" shininess="0.5"/></asset>
  <worldbody><light pos="0 0 1"/><geom type="plane" size="1 1 .1" rgba="0.8 0.8 0.78 1"/>
    <geom type="mesh" mesh="remote" material="remote" pos="0 0 0.009"/>
    <camera name="top" pos="0 0 0.32" xyaxes="1 0 0 0 1 0" fovy="40"/>
    <camera name="side" pos="0.16 -0.22 0.16" xyaxes="0.81 0.59 0 -0.3 0.41 0.86" fovy="40"/></worldbody>
</mujoco>""".format(r=room))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, 480, 640)
    for name in ("top", "side"):
        renderer.update_scene(data, camera=name)
        cv2.imwrite(os.path.join(folder, "remote_{}.png".format(name)), renderer.render()[:, :, ::-1])
    renderer.close()


if __name__ == "__main__":
    robot()
    survey()
    remote()
    print("given material written to", OUT)
