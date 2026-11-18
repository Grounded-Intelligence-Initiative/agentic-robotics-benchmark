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

from rescue_scene import DECK, RED, START_Z, build, layout_for

LIMIT = 1000
DT = .2
INCREMENTS = np.array([.32, .32, .19])
SCORE = {"entered_wing": .12, "mezzanine": .08, "located": .10, "grasped": .12,
         "lifted": .15, "cargo_return": .20}
FAILURES = ("wall", "raised", "inspection", "cargo", "deck", "landing", "safety", "unsafe_release")
ROTOR_LOW = np.array([.21, .21, .235])
ROTOR_HIGH = np.array([.21, .21, .130])
POD_HALF = np.array([.155, .26, .112])


def png(rgb):
    buffer = io.BytesIO()
    Image.fromarray(rgb).save(buffer, format="PNG", compress_level=1)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def basis(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


class World:
    kind = "rescue"

    def __init__(self, kind, seed, render=True):
        if kind != self.kind:
            raise ValueError("unknown task")
        # Adjacent seeds share the site, gains and launch position but reverse
        # the cross-axis tracking residual. It never affects a hold command.
        self.layout = layout_for(seed >> 1)
        self.bias_sign = 1 if seed & 1 else -1
        self.trim_offset = np.zeros(2)
        self.loaded_trim_m = 0.
        self.bounds = np.asarray(self.layout.bounds).T
        xml, decals, obstacles, shutters = build(self.layout)
        self.model = mujoco.MjModel.from_xml_string(xml, assets=decals)
        self.shutters = shutters
        self.model.light_diffuse[:] *= self.layout.tone
        self.data = mujoco.MjData(self.model)
        self.command = np.array([*self.layout.launch, START_Z, 0.])
        self.data.qpos[:4] = self.command
        self.data.ctrl[:] = self.command
        x, y, yaw = self.layout.pod
        self.data.qpos[4:] = (x, y, self.floor_at(y)+.073, math.cos(yaw/2), 0., 0., math.sin(yaw/2))
        mujoco.mj_forward(self.model, self.data)
        self.renderer = mujoco.Renderer(self.model, 256, 256) if render else None
        self.kinds, centers, half = zip(*obstacles)
        centers, half = np.asarray(centers), np.asarray(half)
        self.solid_lo, self.solid_hi = centers-half, centers+half
        self.pod_rotation = np.empty(9)
        self.rel_quat = np.empty(4)
        self.room_centers = np.asarray([self.layout.xy(room) for room in self.layout.rooms])
        self.columns = np.asarray(self.layout.columns)
        self.rows = np.asarray(self.layout.rows)
        self.steps = self.release_dwell = self.close_attempts = 0
        self.steps_to_lift = self.steps_to_cargo = 0
        self.done = self.shutter_open = False
        self.failure = self.impact_body = None
        self.progress, self.rooms_seen, self.cells_seen = set(), set(), set()
        self.loaded_cargo_cells = set()
        self.entry_z = self.exit_z = 0.
        self.mezzanine_z = self.loaded_descent_z = self.turn_angle = 0.
        self._observation = None

    @property
    def held(self):
        return bool(self.data.eq_active[0])

    @property
    def position(self):
        return self.data.qpos[:3]

    @property
    def payload(self):
        return self.data.qpos[4:7]

    def floor_at(self, y):
        return DECK if y >= self.layout.deck_y else 0.

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
        turn_speed = abs(self.data.qvel[3])
        if signal > .5 and not self.held:
            self.close_attempts += 1
            if (np.linalg.norm(delta[:2]) <= .135 and abs(delta[2]+.306) <= .060
                    and speed <= .30 and turn_speed < .3):
                # Capture the actual pose, not a teleport or a success-report command.
                yaw = self.data.qpos[3]
                delta[:2] = basis(yaw).T @ delta[:2]
                self.model.eq_data[0, 3:6] = delta
                inverse = np.array([math.cos(yaw/2), 0., 0., -math.sin(yaw/2)])
                mujoco.mju_mulQuat(self.rel_quat, inverse, self.data.qpos[7:11])
                self.model.eq_data[0, 6:10] = self.rel_quat
                self.data.eq_active[0] = True
                self.progress.add("grasped")
                self.release_dwell = 0
        elif signal < -.5 and self.held:
            self.data.eq_active[0] = False
            self.release_dwell = 0
            if self.payload[2]-self.floor_at(self.payload[1]) > .195 or speed > .34 or turn_speed > .34:
                self.abort("unsafe_release")

    def intersect(self, lo, hi, pod=False):
        hits = np.flatnonzero(np.all((lo < self.solid_hi) & (hi > self.solid_lo), axis=1))
        return next((self.kinds[i] for i in hits if not (pod and self.kinds[i] == "deck")), None)

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
        mujoco.mju_quat2Mat(self.pod_rotation, self.data.qpos[7:11])
        rotation = self.pod_rotation.reshape(3, 3)
        shift = rotation[:, 2]*.04
        half = np.abs(rotation) @ POD_HALF
        hit = self.intersect(np.minimum(self.payload, prev_pod)+shift-half,
                             np.maximum(self.payload, prev_pod)+shift+half, pod=True)
        if hit is not None:
            self.abort(hit, "pod")

    def physical_progress(self, previous_y):
        x, y, z = self.position
        px, py, pz = self.payload
        if (not self.held and "entered_wing" not in self.progress
                and previous_y < self.layout.inlet_y <= y):
            self.progress.add("entered_wing")
            self.entry_z = z
        if previous_y < self.layout.deck_y <= y and not self.mezzanine_z:
            self.progress.add("mezzanine")
            self.mezzanine_z = z
        if self.held and previous_y > self.layout.deck_y >= y and not self.loaded_descent_z:
            self.loaded_descent_z = z
        if self.held and pz-self.floor_at(py) >= .54:
            self.progress.add("lifted")
            if not self.shutter_open:
                # The training wing's mechanical lift interlock retracts the cargo leaf.
                self.shutter_open = True
                self.steps_to_lift = self.steps
                for name, height in (("cargo", 3.8), ("safety", 1.21)):
                    mid = self.model.body(name+"_shutter").mocapid[0]
                    idx = self.shutters[name]
                    movement = height-self.data.mocap_pos[mid, 2]
                    self.data.mocap_pos[mid, 2] = height
                    self.solid_lo[idx, 2] += movement
                    self.solid_hi[idx, 2] += movement
                for name in self.shutters["warning"]:
                    self.model.geom(name).rgba[:] = RED
        if self.held and previous_y > self.layout.inlet_y >= y and "cargo_return" not in self.progress:
            self.progress.add("cargo_return")
            self.steps_to_cargo = self.steps
            self.exit_z = z
        if (not self.held and "lifted" in self.progress
                and math.hypot(px-self.layout.pad[0], py-self.layout.pad[1]) <= .28
                and pz <= .095 and np.linalg.norm(self.data.qvel[4:10]) <= .16):
            self.release_dwell += 1
            if self.release_dwell >= 50:
                self.progress.add("delivered")
                self.done = True
        else:
            self.release_dwell = 0

    def observation_progress(self):
        x, y, z = self.position
        if y > self.layout.inlet_y:
            cx, cy = int(np.argmin(abs(self.columns-x))), int(np.argmin(abs(self.rows-y)))
            self.cells_seen.add((cx, cy))
            if self.held and cx == self.layout.outlet:
                self.loaded_cargo_cells.add(cy)
        if z-self.floor_at(y) >= .85:
            near = np.flatnonzero(np.max(np.abs(self.room_centers-[x, y]), axis=1) < .47)
            self.rooms_seen.update(map(int, near))
            # Center-room view, a conservative visual-discovery checkpoint.
            if np.max(np.abs(self.payload[:2]-[x, y])) < .62:
                self.progress.add("located")

    def act(self, action):
        if (not isinstance(action, list) or len(action) != 5
                or any(type(a) not in (int, float) or not -1 <= a <= 1 for a in action)):
            raise ValueError("action must contain five finite numbers in [-1,1]")
        if self.done:
            return self.observe()
        self.steps += 1
        self._observation = None
        self.grip(action[4])
        delta = INCREMENTS*action[:3]
        # The inner servo is stable; only translation increments are mis-tracked.
        # At zero input, both the gain term and cross-coupling vanish.
        estimated = delta[:2].copy()
        delta[:2] *= self.layout.gains
        side = self.bias_sign*(.125 if self.held else .032)*estimated[1]
        delta[0] += side
        if self.held:
            self.loaded_trim_m += abs(side)
        rotation = basis(self.command[3])
        self.trim_offset += rotation @ (delta[:2]-estimated)
        delta[:2] = rotation @ delta[:2]
        self.command[:3] = np.clip(self.command[:3]+delta, *self.bounds)
        self.command[3] += math.radians(30)*action[3]
        self.data.ctrl[:] = self.command
        previous_heading = self.data.qpos[3]
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
        self.turn_angle += abs(self.data.qpos[3]-previous_heading)
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
            "mezzanine_z": round(self.mezzanine_z, 4),
            "loaded_descent_z": round(self.loaded_descent_z, 4),
            "yaw_travel_deg": round(math.degrees(self.turn_angle), 1),
            "loaded_trim_m": round(self.loaded_trim_m, 3),
            "command_tracking_offset_m": round(float(np.linalg.norm(self.trim_offset)), 3),
            "close_attempts": self.close_attempts,
        }
