"""R1Pro kinematics from its URDF, numpy only: forward kinematics of an arm (torso + 7 arm joints) and damped
least-squares inverse kinematics. Frames: URDF base_link. Used by the reference controller; also shippable to agents."""
from __future__ import annotations
import os
import xml.etree.ElementTree as ET
import numpy as np

URDF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot.urdf")


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def _axis_rot(axis, q):
    a = np.asarray(axis, float); a = a / np.linalg.norm(a); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K


class Arm:
    """Chain base_link -> <side>_gripper_link. q = [torso 4, arm 7]."""

    def __init__(self, side: str, urdf: str = URDF, tip: str | None = None):
        root = ET.parse(urdf).getroot(); par = {j.find("child").get("link"): j for j in root.findall("joint")}
        chain, link = [], tip or f"{side}_gripper_link"
        while link in par:
            j = par[link]; chain.append(j); link = j.find("parent").get("link")
        self.joints = []
        for j in reversed(chain):
            o = j.find("origin"); xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            T = np.eye(4); T[:3, :3] = _rpy(*rpy); T[:3, 3] = xyz
            ax = j.find("axis"); lim = j.find("limit")
            self.joints.append({"name": j.get("name"), "T": T, "movable": j.get("type") in ("revolute", "continuous", "prismatic"), "prismatic": j.get("type") == "prismatic",
                                "axis": [float(v) for v in ax.get("xyz").split()] if ax is not None else [0, 0, 1],
                                "lo": float(lim.get("lower", -np.pi)) if lim is not None else -np.pi, "hi": float(lim.get("upper", np.pi)) if lim is not None else np.pi})
        self.names = [j["name"] for j in self.joints if j["movable"]]
        self.lo = np.array([j["lo"] for j in self.joints if j["movable"]]); self.hi = np.array([j["hi"] for j in self.joints if j["movable"]])

    def fk(self, q, tool=(0.0, 0.0, 0.0)):
        """-> (position of the tool point, rotation matrix of the gripper link, 4x4 transforms after each movable joint)."""
        T = np.eye(4); k = 0; frames = []
        for j in self.joints:
            T = T @ j["T"]
            if j["movable"]:
                M = np.eye(4)
                if j["prismatic"]: M[:3, 3] = np.asarray(j["axis"]) * q[k]
                else: M[:3, :3] = _axis_rot(j["axis"], q[k])
                T = T @ M; k += 1; frames.append(T.copy())
        return T[:3, 3] + T[:3, :3] @ np.asarray(tool, float), T[:3, :3], frames

    def jacobian(self, q, tool=(0.0, 0.0, 0.0)):
        p, R, frames = self.fk(q, tool); J = np.zeros((6, len(q))); k = 0
        for j in self.joints:
            if not j["movable"]: continue
            F = frames[k]; a = F[:3, :3] @ (np.asarray(j["axis"], float) / np.linalg.norm(j["axis"]))
            if j["prismatic"]: J[:3, k] = a
            else: J[:3, k] = np.cross(a, p - F[:3, 3]); J[3:, k] = a
            k += 1
        return J, p, R

    def ik(self, q0, target_p, target_R=None, tool=(0.0, 0.0, 0.0), free=None, iters=60, damp=0.05, rot_w=0.3, tol=2e-3):
        """Damped least squares from q0. free: boolean mask of joints allowed to move (default: the 7 arm joints)."""
        q = np.array(q0, float); n = len(q)
        free = np.array([i >= n - 7 for i in range(n)]) if free is None else np.asarray(free, bool)
        for _ in range(iters):
            J, p, R = self.jacobian(q, tool); e = np.asarray(target_p, float) - p
            if target_R is not None:
                Re = target_R @ R.T; w = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
                err = np.r_[e, rot_w * w]; A = np.r_[J[:3], rot_w * J[3:]][:, free]
            else: err, A = e, J[:3][:, free]
            if np.linalg.norm(e) < tol and (target_R is None or np.linalg.norm(err[3:]) < 0.02): break
            dq = A.T @ np.linalg.solve(A @ A.T + damp ** 2 * np.eye(A.shape[0]), err)
            q[free] = np.clip(q[free] + np.clip(dq, -0.2, 0.2), self.lo[free], self.hi[free])
        _, p, R = self.jacobian(q, tool)
        return q, float(np.linalg.norm(np.asarray(target_p, float) - p))
