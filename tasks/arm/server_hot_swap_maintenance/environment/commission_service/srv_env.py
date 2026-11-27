"""Server hot-swap maintenance: the closed_loop environment (two arms) over srv_sim.World.

Control at 10 Hz. An action is a JSON object {"l": [...], "r": [...], "cams": mask, "done": bool (optional)}; each
arm's list is one of:
- 7 numbers [dx, dy, dz, drx, dry, drz, grip]: the operational-space controller's step (the tool's target moves by
  (dx, dy, dz) m, at most MAX_DPOS, and turns by the rotation vector, at most MAX_DROT, world frame, from the tool's
  current pose); grip -1 fully open (80 mm) .. +1 closed, a linear map of the fingers' target opening;
- 14 numbers: the same and seven joint angles, the null-space posture from now on;
- 8 numbers [q1..q7, grip]: a joint step (each joint's target at most MAX_DQ from its current angle).
`cams` is a bitmask of the camera frames the next observation carries: 1 overhead, 2 left wrist, 4 right wrist.
`done` true ends the episode after this step.

The observation: t; the work order; per arm ("l", "r"): q, dq, ee_pos, ee_quat (wxyz) of the grip site, wrench
(force, torque at the wrist sensor, world frame, with noise and a per-unit bias), gripper (the fingers' opening, m);
each camera's intrinsics K_<cam>; the requested frames, each an RGB image and a depth image (uint16, mm, 0 = no
reading). Nothing gives a camera's pose: the overhead camera's mount and each wrist camera's mount on its hand are off
nominal by a per-unit amount.
"""

import base64
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import srv_sim as S  # noqa: E402
from srv_sim import RenderError  # noqa: E402,F401 - for the grader

CAMS = tuple(S.CAMERA_BITS)
TORQUE_NOISE = S.WRENCH_NOISE / 10


def encode_image(img):
    return {"dtype": str(img.dtype), "shape": list(img.shape), "b64": base64.b64encode(np.ascontiguousarray(img)).decode()}


def decode_image(d):
    return np.frombuffer(base64.b64decode(d["b64"]), dtype=np.dtype(d["dtype"])).reshape(d["shape"])


def to_wire(obs):
    out = {}
    for k, v in obs.items():
        if isinstance(v, np.ndarray) and (k in CAMS or k.startswith("depth_")):
            out[k] = encode_image(v)
        elif isinstance(v, np.ndarray):
            out[k] = v.tolist()
        elif isinstance(v, dict):
            out[k] = {kk: (vv.tolist() if isinstance(vv, np.ndarray) else vv) for kk, vv in v.items()}
        else:
            out[k] = v
    return out


def check_action(action):
    """The action as {"l": floats, "r": floats, "cams": int, "done": bool}; ValueError if malformed."""
    if not isinstance(action, dict) or set(action) - {"l", "r", "cams", "done"} or not {"l", "r"} <= set(action):
        raise ValueError("malformed action")
    out = {}
    for s in ("l", "r"):
        if not isinstance(action[s], list):
            raise ValueError("malformed action")
        a = [float(x) for x in action[s]]
        if len(a) not in (7, 8, 14) or not all(math.isfinite(x) for x in a):
            raise ValueError("malformed action")
        out[s] = a
    cams = action.get("cams", 0)
    if isinstance(cams, bool) or cams != int(cams) or not 0 <= int(cams) <= S.ALL_CAMS:
        raise ValueError("malformed action")
    out["cams"] = int(cams)
    done = action.get("done", False)
    if not isinstance(done, bool):
        raise ValueError("malformed action")
    out["done"] = done
    return out


class SrvEnv:
    def __init__(self, unit, unit_id="dev", render=True):
        self.unit = unit
        self.unit_id = unit_id
        self.render_images = render
        self.renderers = {}
        self.w = None

    def reset(self, seed):
        self.close()
        self.seed = int(seed)
        self.episode = S.draw_episode(self.seed, self.unit)
        self.w = S.World(self.unit, self.episode)
        self.rng = np.random.default_rng([self.seed % 2 ** 32, self.seed >> 32, 31])
        bias = np.random.default_rng([int(self.unit["seed"]) % 2 ** 32, 77])
        self.wrench_bias = {s: np.concatenate([bias.normal(0, 1.0, 3), bias.normal(0, 0.05, 3)]) for s in S.BASES}
        self.render_next = S.ALL_CAMS
        self.faulted = False
        self.done_flag = False
        return self.observation()

    @property
    def t(self):
        return self.w.t if self.w is not None else 0

    def step(self, action):
        a = check_action(action)
        self.render_next = a["cams"]
        self.done_flag = bool(a["done"])
        try:
            self.w.step({s: a[s] for s in ("l", "r")})
        except FloatingPointError:
            self.faulted = True
        return self.observation()

    def unstable(self):
        return bool(self.faulted or self.w.unstable())

    def order(self):
        return {"text": "Replace the failed drive%s and the failed fan." % ("s" if len(self.episode["failed_slots"]) > 1
                                                                          else ""),
                "failed_drives": len(self.episode["failed_slots"]), "failed_fans": 1}

    def arm_obs(self, s):
        a, d = self.w.arms[s], self.w.d
        q = np.empty(4)
        mujoco.mju_mat2Quat(q, a.R().flatten())
        wr = a.wrench_world()
        noise = np.concatenate([self.rng.normal(0, S.WRENCH_NOISE, 3), self.rng.normal(0, TORQUE_NOISE, 3)])
        return {"q": d.qpos[a.jq].copy(), "dq": d.qvel[a.jv].copy(), "ee_pos": a.pos(), "ee_quat": q,
                "wrench": wr + noise + self.wrench_bias[s], "gripper": a.gripper()}

    def observation(self):
        obs = {"t": int(self.w.t), "order": self.order(), "l": self.arm_obs("l"), "r": self.arm_obs("r")}
        for cam in CAMS:
            w, h, fovy = S.camera_size(cam)
            obs["K_" + cam] = S.intrinsics(fovy, w, h)
            if self.render_images and self.render_next & S.CAMERA_BITS[cam]:
                rgb, dep = S.render(self.w, cam, self.rng, self.renderers)
            else:
                rgb, dep = np.zeros((0, 0, 3), np.uint8), np.zeros((0, 0), np.uint16)
            obs[cam] = rgb
            obs["depth_" + cam] = dep
        return obs

    def perturbed_observation(self, obs, rng):
        """The probe lock's other copy: each arm's joints offset by up to 1 mrad, the tool poses recomputed from them,
        each force by up to 0.2 N; frames unchanged."""
        w, m = self.w, self.w.m
        out = dict(obs)
        d = mujoco.MjData(m)
        d.qpos[:] = w.d.qpos
        for s in ("l", "r"):
            a = w.arms[s]
            d.qpos[a.jq] = np.asarray(obs[s]["q"], float) + rng.uniform(-0.001, 0.001, 7)
        mujoco.mj_kinematics(m, d)
        for s in ("l", "r"):
            a = w.arms[s]
            arm = dict(obs[s])
            Rs = d.site_xmat[a.site].reshape(3, 3)
            qq = np.empty(4)
            mujoco.mju_mat2Quat(qq, Rs.flatten())
            wr = np.asarray(obs[s]["wrench"], float).copy()
            wr[:3] += rng.uniform(-0.2, 0.2, 3)
            arm.update(q=d.qpos[a.jq].copy(), ee_pos=d.site_xpos[a.site].copy(), ee_quat=qq, wrench=wr)
            out[s] = arm
        return out

    def truth(self):
        """For a token run only (the reference): the world's state as the reference reads it."""
        return dict(self.w.truth(), seed=self.seed)

    def close(self):
        S.close_renderers(self.renderers)
