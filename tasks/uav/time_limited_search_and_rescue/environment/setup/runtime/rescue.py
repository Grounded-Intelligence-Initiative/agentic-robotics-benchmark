"""Protected MuJoCo flight, swept clearance checks, and live rescue ledger."""
import base64
import io
import math
import os

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")

import mujoco
import numpy as np
from PIL import Image

from rescue_scene import BOUNDS, PITCH, PLAN, START, build, layout_for, xy

LIMIT = 300
DT = .2
INCREMENTS = np.array([.32, .32, .19])
SCORE = {"surveyed": .05, "entered_wing": .12, "located": .10, "grasped": .12,
         "lifted": .15, "cargo_return": .20}
FAILURES = ("wall", "raised", "inspection", "cargo", "unsafe_release")
ROTOR_LOW = np.array([.21, .21, .235])
ROTOR_HIGH = np.array([.21, .21, .130])
POD_HALF = np.array([.155, .26, .112])


def png(rgb):
    buffer = io.BytesIO()
    Image.fromarray(rgb).save(buffer, format="PNG", compress_level=1)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


class World:
    kind = "rescue"

    def __init__(self, kind, seed, render=True):
        if kind != self.kind:
            raise ValueError("unknown task")
        self.layout = layout_for(seed)
        xml, obstacles = build(self.layout)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.model.light_diffuse[:] *= self.layout.tone
        self.data = mujoco.MjData(self.model)
        self.command = np.array(START)
        self.data.qpos[:3] = self.command
        self.data.ctrl[:] = self.command
        x, y, yaw = self.layout.pod
        self.data.qpos[3:] = (x, y, .073, math.cos(yaw/2), 0., 0., math.sin(yaw/2))
        mujoco.mj_forward(self.model, self.data)
        self.renderer = mujoco.Renderer(self.model, 256, 256) if render else None
        cx = xy((self.layout.outlet, 0))[0]
        obstacles.append(("cargo", (cx, PITCH/2, 1.025), (.56, .085, 1.025)))
        self.kinds, centers, half = zip(*obstacles)
        centers, half = np.asarray(centers), np.asarray(half)
        self.solid_lo, self.solid_hi = centers-half, centers+half
        self.pod_rotation = np.empty(9)
        self.room_centers = np.asarray([xy(room) for room in self.layout.rooms])
        self.steps = self.release_dwell = self.close_attempts = 0
        self.steps_to_lift = self.steps_to_cargo = 0
        self.done = self.shutter_open = False
        self.failure = self.impact_body = None
        self.progress, self.rooms_seen, self.cells_seen = set(), set(), set()
        self.loaded_cargo_cells = set()
        self.entry_z = self.exit_z = 0.
        self._observation = None

    @property
    def held(self):
        return bool(self.data.eq_active[0])

    @property
    def position(self):
        return self.data.qpos[:3]

    @property
    def payload(self):
        return self.data.qpos[3:6]

    def close(self):
        if self.renderer is not None:
            self.renderer.close()

    def images(self):
        if self.renderer is None:
            return []
        result = []
        for camera in ("down", "forward"):
            self.renderer.update_scene(self.data, camera=camera)
            result.append(self.renderer.render().copy())
        return result

    def observe(self):
        if self._observation is None:
            self._observation = {
                "images": [png(frame) for frame in self.images()],
                "frame_id": self.steps, "steps_remaining": LIMIT-self.steps, "done": self.done,
            }
        return self._observation

    def abort(self, reason, body=None):
        self.failure, self.impact_body = reason, body
        self.done = True

    def grip(self, signal):
        delta = self.payload-self.position
        speed = np.linalg.norm(self.data.qvel[:3])
        if signal > .5 and not self.held:
            self.close_attempts += 1
            if np.linalg.norm(delta[:2]) <= .135 and abs(delta[2]+.306) <= .060 and speed <= .30:
                # Capture the actual pose, not a teleport or a success-report command.
                self.model.eq_data[0, 3:6] = delta
                self.model.eq_data[0, 6:10] = self.data.qpos[6:10]
                self.data.eq_active[0] = True
                self.progress.add("grasped")
                self.release_dwell = 0
        elif signal < -.5 and self.held:
            self.data.eq_active[0] = False
            self.release_dwell = 0
            if self.payload[2] > .195 or speed > .34:
                self.abort("unsafe_release")

    def intersect(self, lo, hi):
        hits = np.flatnonzero(np.all((lo < self.solid_hi) & (hi > self.solid_lo), axis=1))
        return self.kinds[hits[0]] if hits.size else None

    def check_clearance(self, prev_drone, prev_pod):
        pos = self.position
        hit = self.intersect(np.minimum(pos, prev_drone)-ROTOR_LOW,
                             np.maximum(pos, prev_drone)+ROTOR_HIGH)
        if hit is not None:
            self.abort(hit, "uav")
        if self.done or "grasped" not in self.progress:
            return
        # Orient the sled's bounding box with its MuJoCo quaternion; check its
        # swept volume separately from the UAV so a clean airframe can still strike.
        mujoco.mju_quat2Mat(self.pod_rotation, self.data.qpos[6:10])
        rotation = self.pod_rotation.reshape(3, 3)
        shift = rotation[:, 2]*.04
        half = np.abs(rotation) @ POD_HALF
        hit = self.intersect(np.minimum(self.payload, prev_pod)+shift-half,
                             np.maximum(self.payload, prev_pod)+shift+half)
        if hit is not None:
            self.abort(hit, "pod")

    def physical_progress(self, previous_y):
        x, y, z = self.position
        px, py, pz = self.payload
        if not self.held and "entered_wing" not in self.progress and previous_y < PITCH/2 <= y:
            self.progress.add("entered_wing")
            self.entry_z = z
        if self.held and pz >= .54:
            self.progress.add("lifted")
            if not self.shutter_open:
                # The training wing's mechanical lift interlock retracts the cargo leaf.
                self.shutter_open = True
                self.steps_to_lift = self.steps
                self.data.mocap_pos[0, 2] = 3.30
                self.solid_lo[-1, 2] = 2.275
                self.solid_hi[-1, 2] = 4.325
        if self.held and previous_y > PITCH/2 >= y and "cargo_return" not in self.progress:
            self.progress.add("cargo_return")
            self.steps_to_cargo = self.steps
            self.exit_z = z
        if (not self.held and "lifted" in self.progress and math.hypot(px, py) <= .28
                and pz <= .095 and np.linalg.norm(self.data.qvel[3:9]) <= .16):
            self.release_dwell += 1
            if self.release_dwell >= 50:
                self.progress.add("delivered")
                self.done = True
        else:
            self.release_dwell = 0

    def observation_progress(self):
        x, y, z = self.position
        if abs(x-PLAN[0]) < .36 and abs(y-PLAN[1]) < .32 and z >= 1.05:
            self.progress.add("surveyed")
        if y > PITCH/2:
            cx, cy = int(round(x/PITCH))+2, int(round(y/PITCH))
            self.cells_seen.add((cx, cy))
            if self.held and cx == self.layout.outlet:
                self.loaded_cargo_cells.add(cy)
        if z >= .85:
            near = np.flatnonzero(np.max(np.abs(self.room_centers-[x, y]), axis=1) < .47)
            self.rooms_seen.update(map(int, near))
            # Center-room view, a conservative visual-discovery checkpoint.
            if np.max(np.abs(self.payload[:2]-[x, y])) < .62:
                self.progress.add("located")

    def act(self, action):
        if (not isinstance(action, list) or len(action) != 4
                or any(type(a) not in (int, float) or not -1 <= a <= 1 for a in action)):
            raise ValueError("action must contain four finite numbers in [-1,1]")
        if self.done:
            return self.observe()
        self.steps += 1
        self._observation = None
        self.grip(action[3])
        self.command = np.clip(self.command+INCREMENTS*action[:3], *np.array(BOUNDS).T)
        self.data.ctrl[:] = self.command
        for _ in range(round(DT/self.model.opt.timestep)):
            if self.done:
                break
            prev_drone, prev_pod = self.position.copy(), self.payload.copy()
            mujoco.mj_step(self.model, self.data)
            self.check_clearance(prev_drone, prev_pod)
            if not self.done:
                self.physical_progress(prev_drone[1])
        if not self.done:
            self.observation_progress()
        if self.steps >= LIMIT:
            self.done = True
        return self.observe()

    def metrics(self):
        delivered = "delivered" in self.progress
        partial = sum(SCORE.get(k, 0.) for k in self.progress)+.02*len(self.rooms_seen)
        return {
            "rescue_progress": 1. if delivered else round(partial, 2),
            "success": int(delivered), "delivered": int(delivered),
            **{name: int(name in self.progress) for name in SCORE},
            **{f"{kind}_abort": int(self.failure == kind) for kind in FAILURES},
            "pod_impact": int(self.impact_body == "pod"),
            "uav_impact": int(self.impact_body == "uav"),
            "rooms_seen": len(self.rooms_seen), "wing_cells_visited": len(self.cells_seen),
            "loaded_cargo_cells": len(self.loaded_cargo_cells),
            "steps": self.steps, "simulation_seconds": round(self.data.time, 3),
            "steps_to_lift": self.steps_to_lift, "steps_to_cargo": self.steps_to_cargo,
            "entry_z": round(self.entry_z, 4), "exit_z": round(self.exit_z, 4),
            "close_attempts": self.close_attempts,
        }
