"""Inertial links for local IK and gravity compensation."""
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import mujoco


def inertial_robot(xml_path):
    """Prepare public rigid-body links and joint maps. Only this XML is required."""
    xml = ET.parse(xml_path).getroot()
    for parent in xml.iter():
        for child in list(parent):
            if child.tag in {"geom", "asset", "keyframe", "light"}:
                parent.remove(child)
    model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
    if (model.nbody, model.ngeom, model.neq, model.nq) != (45, 0, 0, 50):
        raise ValueError("Unexpected public G1 inertial model")
    body_motors = [i for i in range(model.nu) if "_hand_" not in model.actuator(i).name]
    ids = model.actuator_trnid[body_motors, 0]
    if len(ids) != 29 or model.joint(ids[22]).name != "right_shoulder_pitch_joint":
        raise ValueError("Unexpected body actuator order")
    return SimpleNamespace(model=model, qadr=model.jnt_qposadr[ids],
                           vadr=model.jnt_dofadr[ids], body_joint_ids=ids)
