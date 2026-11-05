"""Motion primitives of the reference controller (runs INSIDE the grader's process with the privileged scene).

The reference controller is the task's oracle: it reads the exact state (`Rig.truth`, the scene's queries) and drives
the two arms through the server's own controllers and physics tick, so it obeys the same rate limits, the same attach
rules and the same budgets as any submitted program.
"""

import math

import numpy as np

from shoecell.control import ik_solve_seeds, mat_to_quat, quat_mul, quat_to_mat
from shoecell.physics import quat_rotate

__all__ = ["EpisodeOver", "IK_REF_OFFSETS", "Rig", "down_quat", "hook_quat", "rot_quat"]

# reference configurations (added to the home posture) that seed the IK on other branches of the arm: shoulder turned
# either way, elbow more or less bent, the wrist rolled
IK_REF_OFFSETS = [
    (0.8, 0.3, 0.0, 0.3, 0.0, 0.0, 0.0),
    (-0.8, 0.3, 0.0, 0.3, 0.0, 0.0, 0.0),
    (0.0, 0.6, 0.0, 0.6, 0.0, -0.3, 0.0),
    (0.8, 0.6, -0.5, 0.0, 1.0, 0.3, 1.5),
    (-0.8, 0.6, 0.5, 0.0, -1.0, 0.3, -1.5),
    (0.0, -0.3, 0.0, -0.4, 0.0, 0.3, 0.0),
]


class EpisodeOver(Exception):
    pass


def down_quat(yaw):
    """Gripper tool pose: approach (+z) straight down, finger closing axis (+y) along the world direction at `yaw`."""
    c, s = math.cos(yaw), math.sin(yaw)
    x = np.array([-s, c, 0.0])
    y = np.array([c, s, 0.0])
    z = np.array([0.0, 0.0, -1.0])
    return mat_to_quat(np.column_stack([x, y, z]))


def hook_quat(foot_yaw):
    """Hook tool pose: shaft (+z) straight down, the foot (+x) along the world direction at `foot_yaw`."""
    c, s = math.cos(foot_yaw), math.sin(foot_yaw)
    x = np.array([c, s, 0.0])
    z = np.array([0.0, 0.0, -1.0])
    y = np.cross(z, x)
    return mat_to_quat(np.column_stack([x, y, z]))


def rot_quat(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / max(1e-12, np.linalg.norm(axis))
    return np.array([math.cos(angle / 2)] + list(math.sin(angle / 2) * axis))


class Rig(object):
    def __init__(self, server, log=None):
        self.s = server
        self.sc = server.scene
        self.m, self.d = self.sc.model, self.sc.data
        self.log = log or server.log
        self.hook_info = self.sc.info["arms"]["right"]["hook"]
        self.tip_offset_tool = np.array([self.hook_info["foot_len"], 0.0, -self.hook_info["tip_len"]])   # tip END in the tool frame

    # ---------------------------------------------------------------- time
    def tick(self, n=1):
        for _ in range(int(n)):
            if self.s.done:
                raise EpisodeOver(self.s.done_reason)
            if getattr(self.sc, "physics_fault", False):
                self.s._end("physics_fault")
                raise EpisodeOver("physics_fault")
            self.s.physics_tick()
            if self.s.done:
                raise EpisodeOver(self.s.done_reason)
            if getattr(self.sc, "physics_fault", False):
                self.s._end("physics_fault")
                raise EpisodeOver("physics_fault")
            if self.d.time >= self.s.cfg["sim_cap_s"] - 1e-9:
                self.s._end("sim_cap")
                raise EpisodeOver("sim_cap")

    @property
    def over(self):
        """True once the episode is over (server done or a physics blow-up)."""
        return bool(self.s.done or getattr(self.sc, "physics_fault", False))

    def wait(self, seconds):
        self.tick(int(round(seconds / self.s.tick_s)))

    @property
    def time(self):
        return float(self.d.time)

    # ---------------------------------------------------------------- state
    def truth(self):
        return self.s.truth()

    def arm(self, name):
        return self.sc.arms[name]

    def tcp(self, name):
        p, R = self.sc.arms[name].tcp_actual()
        return p.copy(), R.copy()

    def tip_pose(self):
        """World position of the hook tip's END and the tool rotation."""
        p, R = self.tcp("right")
        return p + R @ self.tip_offset_tool, R

    def tcp_for_tip(self, tip_pos, quat):
        """The right tcp position that puts the hook tip's end at `tip_pos` with tool orientation `quat`."""
        R = quat_to_mat(np.asarray(quat, float))
        return np.asarray(tip_pos, float) - R @ self.tip_offset_tool

    # ---------------------------------------------------------------- motion
    def move(self, name, pos, quat, tol_pos=0.004, tol_rot=0.03, timeout=5.0, settle=0.0, width=None):
        """Cartesian move of `name`'s tcp to (pos, quat); returns True when within tolerance before `timeout` sim seconds."""
        arm = self.sc.arms[name]
        if arm.mode != "cartesian":
            arm.set_mode("cartesian")
        arm.command_cartesian(np.asarray(pos, float), np.asarray(quat, float))
        if width is not None and arm.has_gripper:
            self.s.command("left", width=float(width))
        R_goal = quat_to_mat(np.asarray(quat, float))
        n = int(round(timeout / self.s.tick_s))
        ok = False
        for i in range(n):
            self.tick()
            p, R = arm.tcp_actual()
            e_p = float(np.linalg.norm(np.asarray(pos, float) - p))
            e_r = _rot_err(R_goal, R)
            if e_p <= tol_pos and e_r <= tol_rot:
                ok = True
                break
        if settle > 0:
            self.wait(settle)
        return ok

    def move_joint(self, name, q, tol=0.01, timeout=6.0):
        arm = self.sc.arms[name]
        arm.set_mode("joint")
        arm.command_joint(np.asarray(q, float))
        n = int(round(timeout / self.s.tick_s))
        for _ in range(n):
            self.tick()
            if float(np.max(np.abs(arm.q() - np.asarray(q, float)))) <= tol:
                return True
        return False

    def ik(self, name, pos, quat, q_ref=None, free_axis=False, all_branches=True):
        """A collision-free joint solution for the tcp pose, or None.  Seeded from `q_ref` (default: the current
        configuration) with the wrist-flip offsets; when that fails and `all_branches`, also from the home posture and
        from a few elbow / shoulder offsets, so that targets on another branch of the arm are found."""
        arm = self.sc.arms[name]
        R = quat_to_mat(np.asarray(quat, float))
        pos = np.asarray(pos, float)
        refs = [arm.q() if q_ref is None else np.asarray(q_ref, float)]
        if all_branches:
            refs.append(arm.home_q)
            for off in IK_REF_OFFSETS:
                refs.append(np.clip(arm.home_q + np.asarray(off, float), arm.lower, arm.upper))
        for ref in refs:
            q, pe, re, ok, cols = ik_solve_seeds(arm, pos, R, q_ref=ref, free_axis=free_axis)
            if ok and not cols:
                return q
        return None

    def ik_margin(self, name, q):
        """Smallest distance of configuration q to a joint limit (rad)."""
        arm = self.sc.arms[name]
        q = np.asarray(q, float)
        return float(np.min(np.minimum(q - arm.lower, arm.upper - q)))

    def go(self, name, pos, quat, tol_pos=0.004, tol_rot=0.03, timeout=5.0, settle=0.0, via_joint=False):
        """move(); if the local Cartesian controller stalls, solve the pose with seeds, travel there in joint space and
        finish in Cartesian mode."""
        if not via_joint and self.move(name, pos, quat, tol_pos, tol_rot, timeout, settle):
            return True
        q = self.ik(name, pos, quat, q_ref=self.sc.arms[name].q())
        if q is None:
            self.log("[ref] no IK solution for %s at %s" % (name, np.round(pos, 3)))
            return False
        self.move_joint(name, q, timeout=timeout)
        return self.move(name, pos, quat, tol_pos, tol_rot, min(2.0, timeout), settle)

    def joint_margins(self, name, q=None):
        """Per-joint distance to the nearer limit (rad) of configuration q (default: the measured one)."""
        arm = self.sc.arms[name]
        q = arm.q() if q is None else np.asarray(q, float)
        return np.minimum(q - arm.lower, arm.upper - q)

    def lift(self, name, dz, quat=None, **kw):
        p, R = self.tcp(name)
        q = mat_to_quat(R) if quat is None else quat
        return self.move(name, p + np.array([0.0, 0.0, dz]), q, **kw)

    # ---------------------------------------------------------------- tools
    def gripper(self, width, wait=True, timeout=2.5):
        self.s.command("left", width=float(width))
        if not wait:
            return
        arm = self.sc.arms["left"]
        n = int(round(timeout / self.s.tick_s))
        last = arm.width()
        for i in range(n):
            self.tick()
            w = arm.width()
            if abs(w - float(width)) < 0.001:
                break
            if i > 20 and abs(w - last) < 1e-5 and self.s.left_holding is not None:
                break
            last = w

    def grasp(self, timeout=2.5):
        """Close the fingers; returns the name of the handle attached, or None."""
        self.s.command("left", width=0.0)
        n = int(round(timeout / self.s.tick_s))
        for _ in range(n):
            self.tick()
            if self.s.left_holding is not None:
                self.wait(0.05)
                return self.s.left_holding[1]
        return None

    def release(self, wait=0.4):
        self.s.command("left", width=0.08)
        self.wait(wait)

    def hook(self, on):
        r = self.s.hook(bool(on))
        self.tick()
        return bool(r.get("loaded"))

    def release_clamp(self):
        self.s.release_clamp()

    @property
    def holding(self):
        return self.s.left_holding

    @property
    def hook_loaded(self):
        return self.s.hook_loaded is not None


def _rot_err(R_goal, R):
    Re = R_goal @ R.T
    c = (np.trace(Re) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, c)))
