"""Kinematics on a compiled model (FK = MuJoCo's kinematics on the compiled files, which keep the given
frames). Angles are 25-vectors in firmware slot order: slot j holds the angle of the slot's joint; the ankle
slots hold the foot's hinge angles (slot 4 / 10: <side>_ankle_pitch_joint, slot 5 / 11: <side>_ankle_roll_joint), the
order of a command's 25 targets. The pelvis free joint carries the pose (R0, p0) of each call (default: identity at
the origin, the pelvis frame). Every body of the compiled model counts in the centre of mass (jointless child bodies
move with their parent)."""
from __future__ import annotations

import numpy as np
import mujoco

from .util import mat2quat

SIDES = ("left", "right")
# slot -> joint name (slots.json's joints; the ankle motors' slots carry the foot's pitch / roll hinges, A -> pitch,
# B -> roll, as in a command)
SLOT_JOINT = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_yaw_joint",
    "waist_yaw_joint", "neck_yaw_joint", "neck_pitch_joint",
)
ANKLE_SLOTS = {"left": (4, 5), "right": (10, 11)}
JOINT_SLOTS = tuple(j for j in range(25) if j not in (4, 5, 10, 11))
LEG_SLOTS = {"left": (0, 1, 2, 3, 4, 5), "right": (6, 7, 8, 9, 10, 11)}
BODIES = (
    "pelvis_link",
    "left_hip_pitch_link", "left_hip_roll_link", "left_hip_yaw_link", "left_knee_link", "left_ankle_pitch_link",
    "left_ankle_roll_link",
    "right_hip_pitch_link", "right_hip_roll_link", "right_hip_yaw_link", "right_knee_link", "right_ankle_pitch_link",
    "right_ankle_roll_link",
    "waist_yaw_link", "neck_yaw_link", "neck_pitch_link",
    "right_shoulder_pitch_link", "right_shoulder_roll_link", "right_shoulder_yaw_link", "right_elbow_link",
    "right_wrist_yaw_link",
    "left_shoulder_pitch_link", "left_shoulder_roll_link", "left_shoulder_yaw_link", "left_elbow_link",
    "left_wrist_yaw_link",
)
FOOT = {"left": "left_ankle_roll_link", "right": "right_ankle_roll_link"}
TORSO = "waist_yaw_link"
PELVIS = "pelvis_link"
ARM_LINKS = {"left": ("left_shoulder_yaw_link", "left_elbow_link"), "right": ("right_shoulder_yaw_link", "right_elbow_link")}


class Kin:
    """FK on one compiled model (its own MjData)"""

    def __init__(self, m: mujoco.MjModel):
        self.m = m
        self.d = mujoco.MjData(m)
        J, B = mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_BODY
        self.qadr = np.array([int(m.jnt_qposadr[mujoco.mj_name2id(m, J, n)]) for n in SLOT_JOINT], int)
        self.free_adr = 0
        for j in range(m.njnt):
            if int(m.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE):
                self.free_adr = int(m.jnt_qposadr[j])
        self.bid = {n: int(mujoco.mj_name2id(m, B, n)) for n in BODIES}
        self.foot = {s: self.bid[FOOT[s]] for s in SIDES}
        self.torso = self.bid[TORSO]
        self.pelvis = self.bid[PELVIS]
        self.mass = np.array(m.body_mass[1:], float)          # every body but the world (jointless children included)
        self.mtot = float(self.mass.sum())

    def fk(self, th, R0=None, p0=None):
        """run MuJoCo's kinematics at slot angles th with the pelvis at (R0, p0); returns self (read xpos / xmat / ...)"""
        d = self.d
        d.qpos[:] = self.m.qpos0
        a = self.free_adr
        d.qpos[a:a + 3] = np.zeros(3) if p0 is None else np.asarray(p0, float)
        d.qpos[a + 3:a + 7] = np.array([1.0, 0.0, 0.0, 0.0]) if R0 is None else mat2quat(R0)
        d.qpos[self.qadr] = np.asarray(th, float)
        mujoco.mj_kinematics(self.m, d)
        return self

    # ---- read-outs after fk()
    def xpos(self, b):
        return self.d.xpos[b].copy()

    def xmat(self, b):
        return self.d.xmat[b].reshape(3, 3).copy()

    def site(self, sid):
        return self.d.site_xpos[sid].copy(), self.d.site_xmat[sid].reshape(3, 3).copy()

    def com(self):
        """the centre of mass of every body (world frame)"""
        w = self.d.xipos[1:]
        return (self.mass[:, None] * w).sum(0) / self.mtot

    def all_xpos(self):
        return self.d.xpos.copy()

    def all_xmat(self):
        return self.d.xmat.reshape(-1, 3, 3).copy()
