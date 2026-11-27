"""Released Unitree G1/Dex3 with native position actuators and one flat floor."""
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from sonic_sim.decoder import EFFORT_LIMIT, KD, KP
from .obstacles import MAX_GATES, PANEL_DEPTH, PANEL_HEIGHT, PANEL_LENGTH


SIDES = ("left", "right")
LEG = ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
ARM = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
BODY_JOINTS = tuple(f"{s}_{j}_joint" for s in SIDES for j in LEG) + (
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
) + tuple(f"{s}_{j}_joint" for s in SIDES for j in ARM)
FINGERS = ("thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1")
FINGER_JOINTS = tuple(f"{s}_hand_{j}_joint" for s in SIDES for j in FINGERS)
RIGHT_CLOSURE = np.array([0., -.65, -1., .75, .65, .75, .65])
PHYSICS_DT = .002
SUBSTEPS = 10


def make_model(root):
    xml = ET.parse(root / "g1_with_hands.xml").getroot()
    xml.find("compiler").set("meshdir", str(root / "assets"))
    xml.find("option").set("timestep", str(PHYSICS_DT))
    xml.find("option").set("cone", "elliptic")
    xml.remove(xml.find("keyframe"))
    visual = ET.SubElement(xml, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="960")
    ET.SubElement(visual, "quality", offsamples="0", shadowsize="256")
    ET.SubElement(visual, "headlight", ambient=".45 .45 .45", diffuse=".65 .65 .65", specular=".08 .08 .08")
    asset = xml.find("asset")
    ET.SubElement(asset, "texture", name="sky", type="skybox", builtin="gradient",
                  rgb1=".64 .72 .78", rgb2=".32 .40 .45", width="128", height="768")
    ET.SubElement(asset, "texture", name="tiles", type="2d", builtin="checker",
                  rgb1=".18 .21 .24", rgb2=".30 .32 .34", width="256", height="256")
    ET.SubElement(asset, "material", name="floor_mat", texture="tiles", texrepeat="4 4", texuniform="true")
    ET.SubElement(xml.find("worldbody"), "geom", name="floor", type="plane", size="0 0 .05", material="floor_mat")
    world = xml.find("worldbody")
    for i in range(MAX_GATES):
        for side in ("minus_y", "plus_y"):
            name = f"gate_{i}_{side}"
            body = ET.SubElement(world, "body", name=name, mocap="true", pos=f"{2+i*2} 0 .36")
            ET.SubElement(body, "geom", name=name+"_cabinet", type="box", group="0",
                          size=f"{PANEL_DEPTH/2} {PANEL_LENGTH/2} {PANEL_HEIGHT/2}",
                          rgba=".09 .56 .62 1", contype="2", conaffinity="1")
    for side in ("minus_y", "plus_y"):
        body = ET.SubElement(world, "body", name="lane_"+side, mocap="true")
        ET.SubElement(body, "geom", type="box", name="lane_"+side+"_stripe",
                      size="4.25 .018 .001", rgba=".82 .86 .87 1",
                      contype="0", conaffinity="0", group="0")
    pelvis = xml.find(".//body[@name='pelvis']")
    ET.SubElement(pelvis, "camera", name="egocentric", pos=".28 0 .33",
                  xyaxes="0 -1 0 0 0 1", fovy="75")
    actuators = xml.find("actuator")
    for i, joint in enumerate(BODY_JOINTS):
        motor = actuators.find(f"position[@joint='{joint}']")
        motor.set("kp", str(float(KP[i])))
        motor.set("kv", str(float(KD[i])))
        motor.set("forcerange", f"{-EFFORT_LIMIT[i]} {EFFORT_LIMIT[i]}")
    for joint in FINGER_JOINTS:
        motor = actuators.find(f"position[@joint='{joint}']")
        motor.set("kp", "4")
        motor.set("kv", ".15")
        cap = 2.45 if "thumb_0" in joint else 1.4
        motor.set("forcerange", f"{-cap} {cap}")
    xml.find("default/default/position").attrib.pop("dampratio")
    return mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))


def floor_contacts(model, data, floor_id, sole_side):
    loads = [0., 0.]
    other = 0.
    force = np.empty(6)
    for index, contact in enumerate(data.contact):
        g1, g2 = int(contact.geom1), int(contact.geom2)
        if floor_id not in (g1, g2):
            continue
        geom, sign = (g2, 1.) if g1 == floor_id else (g1, -1.)
        mujoco.mj_contactForce(model, data, index, force)
        vertical = sign * (contact.frame.reshape(3, 3).T @ force[:3])[2]
        if geom in sole_side:
            loads[sole_side[geom]] += vertical
        else:
            other += max(0., float(vertical))
    return float(loads[0]), float(loads[1]), other
