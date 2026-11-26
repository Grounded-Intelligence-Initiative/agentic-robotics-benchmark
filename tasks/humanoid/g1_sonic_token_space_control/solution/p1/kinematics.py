"""Inverse kinematics on a geometry-free public G1 link model."""
import mujoco
import numpy as np

from .waypoints import PickPlaceWaypoints


class PickPlaceAgent(PickPlaceWaypoints):
    """Construct after resetting the session; call act once per 50 Hz tick."""

    def __init__(self, sim, **waypoint_options):
        from sonic_sim.decoder import DEFAULT_Q

        super().__init__(**waypoint_options)
        self.sim = sim
        self.model = sim.model
        self.fk = mujoco.MjData(self.model)
        self.wrist = self.model.body("right_wrist_yaw_link").id
        self.qadr = np.asarray(sim.qadr)[22:29]
        self.vadr = np.asarray(sim.vadr)[22:29]
        self.limits = self.model.jnt_range[np.asarray(sim.body_joint_ids)[22:29]]
        self.nominal = np.array(DEFAULT_Q, dtype=float)
        self.reference = self.nominal.copy()
        self.pocket = np.array([0.115, 0.048, 0.0])
        self.orientation = np.eye(3)

    def _point(self, data):
        return data.xpos[self.wrist] + data.xmat[self.wrist].reshape(3, 3) @ self.pocket

    def _arm_ik(self, goal, rotation, *, robot_observation):
        self.fk.qpos[:3] = robot_observation["base_pos"]
        self.fk.qpos[3:7] = robot_observation["base_quat"]
        self.fk.qpos[self.sim.qadr] = robot_observation["q"]
        self.fk.qpos[self.qadr] = self.nominal[22:]
        jp = np.zeros((3, self.model.nv))
        jr = np.zeros_like(jp)
        for _ in range(22):
            mujoco.mj_kinematics(self.model, self.fk)
            mujoco.mj_comPos(self.model, self.fk)
            point = self._point(self.fk)
            measured = self.fk.xmat[self.wrist].reshape(3, 3)
            ori_error = np.cross(measured.T, rotation.T).sum(axis=0) * 0.5
            posture = self.nominal[22:] - self.fk.qpos[self.qadr]
            err = np.concatenate((goal - point, ori_error * 0.10, posture * 0.012))
            mujoco.mj_jac(self.model, self.fk, jp, jr, point, self.wrist)
            jac = np.vstack((jp[:, self.vadr], jr[:, self.vadr] * 0.10, 0.012 * np.eye(7)))
            dq = np.linalg.solve(jac.T @ jac + 0.00018 * np.eye(7), jac.T @ err)
            self.fk.qpos[self.qadr] = np.clip(self.fk.qpos[self.qadr] + dq / max(1, np.abs(dq).max()/0.22),
                                               self.limits[:, 0], self.limits[:, 1])
            if np.max(np.abs(dq)) < .007:
                break
        return self.fk.qpos[self.qadr].copy()
