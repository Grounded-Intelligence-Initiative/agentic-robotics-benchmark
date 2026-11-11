"""Standalone reconstruction simulator: scene.xml plus an action sequence -> object XYZ."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import mujoco
import numpy as np


def check_asset_paths(scene, active=None):
    scene = Path(scene)
    active = set() if active is None else active
    key = scene.resolve()
    if key in active or len(active) > 16:
        raise ValueError("cyclic or overly nested XML include")
    active.add(key)
    try:
        for element in ET.parse(scene).iter():
            for name in ("file", "assetdir", "meshdir", "texturedir"):
                if name in element.attrib:
                    path = Path(element.attrib[name])
                    if path.is_absolute() or ".." in path.parts:
                        raise ValueError("model assets must remain inside the frozen submission")
            if element.tag == "include":
                check_asset_paths(scene.parent/element.attrib["file"], active)
    finally:
        active.remove(key)


def load_model(scene):
    check_asset_paths(scene)
    model = mujoco.MjModel.from_xml_path(scene)
    if model.ngeom < 3 or model.nq < 6 or model.nu != 3 or model.neq < 1:
        raise ValueError("expected a three-actuator Cartesian arm and free object")
    agent, obj = model.body("agent").id, model.body("object").id
    if agent <= 0 or obj <= 0:
        raise ValueError("missing agent/object bodies")
    for i in range(3):
        joint = int(model.dof_jntid[i])
        if (model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_SLIDE
                or model.jnt_qposadr[joint] != i or model.jnt_dofadr[joint] != i
                or model.jnt_bodyid[joint] != agent
                or not np.allclose(model.jnt_axis[joint], np.eye(3)[i])
                or model.actuator_trnid[i, 0] != joint):
            raise ValueError("Cartesian joint/actuator contract mismatch")
    if (model.eq_type[0] != mujoco.mjtEq.mjEQ_WELD
            or model.eq_obj1id[0] != agent or model.eq_obj2id[0] != obj):
        raise ValueError("first equality must weld agent to object")
    return model


def predict(scene, actions):
    model = load_model(scene)
    return rollout(model, actions)


def rollout(model, actions):
    data = mujoco.MjData(model)
    data.qpos[:3] = [-1.9, -1.7, 1.]
    command = data.qpos[:3].copy()
    data.ctrl[:] = command
    mujoco.mj_forward(model, data)
    held = False
    positions = []
    for action in actions:
        command = np.clip(command + np.array(action[:3])*.14,
                          [-2.6, -2.6, .17], [2.6, 2.6, 1.5])
        obj = data.body("object").xpos.copy()
        eff = data.body("agent").xpos.copy()
        if action[3] > .5 and not held and np.linalg.norm(eff-obj) < .24:
            model.eq_data[0, 3:6] = obj-eff
            model.eq_data[0, 6:10] = [1, 0, 0, 0]
            data.eq_active[0], held = 1, True
        elif action[3] < -.5:
            data.eq_active[0], held = 0, False
        for _ in range(20):
            data.ctrl[:] = command
            data.qfrc_applied[:3] = [0, 0, 5.01]
            mujoco.mj_step(model, data)
        positions.append(np.r_[data.body("agent").xpos, data.body("object").xpos].tolist())
    return positions


if __name__ == "__main__":
    print(json.dumps({"positions": predict("scene.xml", json.load(sys.stdin)["actions"])}))
