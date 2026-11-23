"""Standalone reconstruction simulator: scene.xml and actions -> arm/object pose."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

# The verifier runs this immutable entry point in isolated Python mode.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_dynamics import ArmDynamics, SUBSTEPS, drive_matrix, parameter, pose_row


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
    if model.ngeom < 3 or model.nq < 11 or model.nu != 3 or model.neq < 1:
        raise ValueError("expected a three-actuator Cartesian arm and free object")
    agent, obj = model.body("agent").id, model.body("object").id
    tool = model.body("tool").id
    if agent <= 0 or obj <= 0:
        raise ValueError("missing agent/object bodies")
    for i in range(3):
        joint = int(model.dof_jntid[i])
        if (model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_SLIDE
                or model.jnt_qposadr[joint] != i or model.jnt_dofadr[joint] != i
                or model.jnt_bodyid[joint] != agent
                or not np.allclose(model.jnt_axis[joint], np.eye(3)[i])):
            raise ValueError("Cartesian joint contract mismatch")
    drive_matrix(model)
    if (model.actuator_trntype[2] != mujoco.mjtTrn.mjTRN_JOINT
            or model.actuator_trnid[2, 0] != model.dof_jntid[2]
            or not np.isclose(model.actuator_gear[2, 0], 1.)):
        raise ValueError("third actuator must directly drive Z at unit gear")
    joint = int(model.dof_jntid[3])
    if (model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_HINGE
            or model.jnt_bodyid[joint] != tool or model.body_parentid[tool] != agent
            or not np.allclose(model.jnt_axis[joint], [0, 1, 0])):
        raise ValueError("tool must have a passive Y-axis pitch joint on agent")
    if (model.eq_type[0] != mujoco.mjtEq.mjEQ_WELD
            or model.eq_obj1id[0] != tool or model.eq_obj2id[0] != obj):
        raise ValueError("first equality must weld tool to object")
    if not np.isclose(model.opt.timestep, .2 / SUBSTEPS):
        raise ValueError("expected .01-second integration")
    if model.geom_bodyid[model.geom("fragile").id] != obj:
        raise ValueError("fragile collision geom must belong to object")
    for name in ("cup_capacity_n", "cap_proof_n", "impact_limit_n", "scuff_limit_n"):
        parameter(model, name)
    return model


def predict(scene, actions):
    model = load_model(scene)
    return rollout(model, actions)


def rollout(model, actions):
    data = mujoco.MjData(model)
    arm = ArmDynamics(model, data)
    positions = []
    for action in actions:
        arm.step(action)
        positions.append(pose_row(data).tolist())
    return positions


if __name__ == "__main__":
    print(json.dumps({"positions": predict("scene.xml", json.load(sys.stdin)["actions"])}))
