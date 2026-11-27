"""Grasp alignment and supported carry for the simulated sorting pieces."""
import math

import mujoco as mj
import numpy as np

OPEN = .04695
CLOSED = 0.
PICKUP_HEIGHTS = {1.45: .095, 1.86: .009, 2.02: .006, 2.24: .010, 2.68: .120}


def tool_position(model, data, site, qpos, q, dofs, target):
    guess = q.copy()
    jacobian = np.zeros((3, model.nv))
    for _ in range(36):
        data.qpos[qpos] = guess
        mj.mj_forward(model, data)
        difference = target - data.site_xpos[site]
        if np.linalg.norm(difference) < .00045:
            break
        mj.mj_jacSite(model, data, jacobian, None, site)
        jac = jacobian[:, dofs[:3]]
        update = jac.T @ np.linalg.solve(jac @ jac.T + .0001*np.eye(3), difference)
        guess[:3] = np.clip(guess[:3] + np.clip(update, -.18, .18), -3.1, 3.1)
        limits = np.array([model.joint(f"left_joint{i}").range for i in range(1, 7)])
        guess = np.clip(guess, limits[:, 0], limits[:, 1])
    return guess, float(np.linalg.norm(difference))


def orientation_at_yaw(matrix, angle):
    cosine, sine = math.cos(angle), math.sin(angle)
    rotation = np.array([[cosine, -sine, 0.], [sine, cosine, 0.], [0., 0., 1.]])
    return rotation @ matrix


class GraspPose:
    def __init__(self, arena):
        self.arena = arena
        self.model = arena.model
        self.data = mj.MjData(self.model)
        self.data.qpos[:] = arena.data.qpos
        self.arm = arena.arms["left"]
        self.qpos = self.arm["qpos"][:6]
        self.dofs = self.arm["dof"]
        self.site = self.model.site("left_grasp_site").id
        self.jacobian = np.zeros((3, self.model.nv))
        self.angular = np.zeros_like(self.jacobian)

    def forward(self, joints):
        self.data.qpos[self.qpos] = joints
        mj.mj_forward(self.model, self.data)
        return (self.data.site_xpos[self.site].copy(),
                self.data.site_xmat[self.site].reshape(3, 3).copy())

    def solve(self, p, orientation, start, rotation_weight=.18):
        q = np.asarray(start).copy()
        best = (float("inf"), q.copy(), 0.)
        limits = self.arm["range"]
        for _ in range(90):
            point, axes = self.forward(q)
            linear = p - point
            angular = .5 * sum(np.cross(axes[:, axis], orientation[:, axis]) for axis in range(3))
            cost = np.linalg.norm(linear) + rotation_weight * np.linalg.norm(angular)
            if cost < best[0]:
                best = (float(cost), q.copy(), float(np.linalg.norm(linear)))
            if np.linalg.norm(linear) < .0007 and np.linalg.norm(angular) < .015:
                break
            mj.mj_jacSite(self.model, self.data, self.jacobian, self.angular, self.site)
            jac = np.r_[self.jacobian[:, self.dofs], rotation_weight * self.angular[:, self.dofs]]
            error = np.r_[linear, rotation_weight * angular]
            delta = jac.T @ np.linalg.solve(jac @ jac.T + .0004*np.eye(6), error)
            q = np.clip(q + np.clip(delta, -.095, .095), limits[:,0], limits[:,1])
        return best[1], best[2]


def tilt_into_pad_support(matrix, heading, angle):
    axis = np.array([math.cos(heading), math.sin(heading), 0.])
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return (np.eye(3) + math.sin(angle)*skew + (1.-math.cos(angle))*(skew@skew))@matrix


def pose_row(time, q, jaw=CLOSED):
    return [time, *np.asarray(q).tolist(), jaw]


def pickup(arena, obj, template):
    pose = GraspPose(arena)
    center = np.asarray(obj["position"])
    rows = [row.copy() for row in template[:7]]
    rows[0][1:7] = arena.data.qpos[pose.qpos].tolist()
    for row in rows:
        timestamp = row[0]
        if timestamp not in PICKUP_HEIGHTS:
            continue
        destination = center + np.array([0., 0., PICKUP_HEIGHTS[timestamp]])
        q = np.asarray(row[1:7])
        q, _ = tool_position(pose.model, pose.data, pose.site, pose.qpos, q, pose.dofs, destination)
        if timestamp != 1.45:
            point, rotation = pose.forward(q)
            pad_center = point + .7 * rotation @ np.array([.009, -.0026, .0104])
            pad_center[2] = max(pad_center[2], center[2] + .002)
            corrected, error = tool_position(pose.model, pose.data, pose.site, pose.qpos, q,
                                             pose.dofs, pad_center)
            if error < .007:
                q = corrected
        row[1:7] = q.tolist()
    return [rows[0]] + [[round(row[0]+2.05, 3), *row[1:]] for row in rows] + [
        pose_row(6., rows[-1][1:7], CLOSED),
    ]


def carry(arena, obj):
    solver = GraspPose(arena)
    q = arena.arms["left"]["ref"].copy()
    point, attitude = solver.forward(q)
    origin = arena.data.body("left_base").xpos
    relative = point - origin
    target = np.asarray(obj["target"]) - origin
    start = math.atan2(relative[1], relative[0])
    finish = math.atan2(target[1], target[0])
    start_radius, target_radius = np.linalg.norm(relative[:2]), np.linalg.norm(target[:2])
    rows = [pose_row(0., q, CLOSED)]
    for i, bearing in enumerate(np.linspace(start, finish, 11)[1:], 1):
        progress = i / 10
        extension = max(0., (progress - .65) / .35)
        radius = start_radius - .065 * math.sin(math.pi * progress / 2)
        radius = (1 - extension) * radius + extension * (target_radius - .023)
        height = (1 - extension) * (relative[2] + .07*progress) + extension * (target[2] + .157)
        position = origin + np.array([radius*math.cos(bearing), radius*math.sin(bearing), height])
        rotation = orientation_at_yaw(attitude, bearing - start)
        tilt = -1.2 * min(1., progress / .44) + .8 * max(0., (progress - .75)/.25)
        rotation = tilt_into_pad_support(rotation, bearing, tilt)
        q, error = solver.solve(position, rotation, q, .18-.158*extension)
        if error > .025:
            return []
        rows.append(pose_row(min(i, 8)*.4 + max(0, i-8)*.25, q, CLOSED))
    before, last = rows[-2:]
    start_q, end_q = np.asarray(before[1:7]), np.asarray(last[1:7])
    rate = (end_q-start_q)/(last[0]-before[0])
    opening_time = last[0] - .05
    follow_time = opening_time + .03
    opening = end_q - .05*rate
    follow = opening + .03*rate
    position, rotation = solver.forward(follow)
    retreat, _ = solver.solve(position + [0., .05, .11], rotation, follow, .09)
    retreat[0] = follow[0] + .12
    retreat = np.clip(retreat, solver.arm["range"][:, 0], solver.arm["range"][:, 1])
    return rows[:-1] + [
        pose_row(opening_time, opening, OPEN),
        pose_row(follow_time, follow, OPEN),
        pose_row(follow_time+.11, retreat, OPEN),
        pose_row(6., retreat, OPEN),
    ]
