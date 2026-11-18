"""Protected MuJoCo dynamics and milestone ledger for the fictional rescue exercise."""
import base64
import io
import math
import os

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")

import mujoco
import numpy as np
from PIL import Image

from rescue_scene import (BOARD, BOUNDS, COOL, CRATES, DOWN_OFFSET, GATE_CEILING,
                          GATE_HALF_WIDTH, GATE_Y, HEIGHT_BOARD, HEIGHT_GATE_Y,
                          HEIGHT_GATE_ZONE, HEIGHT_HALF_WIDTH, LANES, PAD,
                          PASSAGES, template)

LIMIT = 184
DT = .2
INCREMENTS = np.array([.32, .32, .19])
START = np.array([*PAD, 1.12])
SCORE = {"survey_lane": 5, "survey_height": 5, "lane_out": 10, "height_out": 10,
         "located": 10, "lifted": 20, "height_back": 15, "lane_back": 15, "delivered": 10}
CROSSINGS = ("lane_out", "height_out", "height_back", "lane_back")


def scenario(seed):
    """Each block of four levels independently swaps lane and altitude forecasts."""
    rng = np.random.default_rng(seed >> 2)
    lane = int(rng.permutation(len(LANES))[seed & 1])
    height = int(rng.integers(len(PASSAGES))) ^ ((seed >> 1) & 1)
    x = rng.choice((-2.55, 0., 2.55)) + rng.uniform(-.33, .33)
    # Keep the middle aisle clear of the two cabinets at ground level.
    if abs(x) < 1.0:
        x *= .42
    y = rng.choice((4.64, 7.05)) + rng.uniform(-.25, .25)
    yaw = rng.uniform(-.6, .6)
    tone = rng.uniform(.88, 1.08)
    return lane, height, (x, y, yaw), tone


def png(rgb):
    buffer = io.BytesIO()
    Image.fromarray(rgb).save(buffer, format="PNG", compress_level=1)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


class World:
    kind = "rescue"

    def __init__(self, kind, seed, render=True):
        if kind != self.kind:
            raise ValueError("unknown task")
        self.safe_lane, self.safe_height, (x, y, yaw), tone = scenario(seed)
        self.model = mujoco.MjModel.from_xml_string(template())
        self.model.geom(f"forecast_tile{self.safe_lane}").rgba[:] = COOL
        self.model.geom(f"height_tile{self.safe_height}").rgba[:] = COOL
        self.model.light_diffuse[:] *= tone
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:3] = self.command = START.copy()
        self.data.ctrl[:] = self.command
        self.data.qpos[3:] = (x, y, .073, math.cos(yaw/2), 0., 0., math.sin(yaw/2))
        mujoco.mj_forward(self.model, self.data)
        self.renderer = mujoco.Renderer(self.model, 256, 256) if render else None
        self.steps = self.release_dwell = self.close_attempts = 0
        self.done = False
        self.failure = None
        self.progress = set()
        self.crossing_altitudes = {}
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
        if self.renderer:
            self.renderer.close()

    def images(self):
        if self.renderer is None:
            return []
        frames = []
        for camera in ("down", "forward"):
            self.renderer.update_scene(self.data, camera=camera)
            frames.append(self.renderer.render().copy())
        return frames

    def observe(self):
        if self._observation is None:
            self._observation = {
                "images": [png(frame) for frame in self.images()],
                "frame_id": self.steps, "steps_remaining": LIMIT-self.steps, "done": self.done,
            }
        return self._observation

    def abort(self, reason):
        self.failure = reason
        self.done = True

    def grip(self, signal):
        delta = self.payload - self.position
        speed = np.linalg.norm(self.data.qvel[:3])
        if signal > .5 and not self.held:
            self.close_attempts += 1
            if ("located" in self.progress and np.linalg.norm(delta[:2]) <= .135
                    and abs(delta[2]+.306) <= .060 and speed <= .30):
                # Capture the current rest transform so the pod is not teleported.
                self.model.eq_data[0, 3:6] = delta
                self.model.eq_data[0, 6:10] = self.data.qpos[6:10]
                self.data.eq_active[0] = True
                self.release_dwell = 0
        elif signal < -.5 and self.held:
            self.data.eq_active[0] = False
            self.release_dwell = 0
            if self.payload[2] > .195 or speed > .34:
                self.abort("unsafe_release")

    def check_safety(self):
        x, y, z = self.position
        if abs(y) <= GATE_Y:
            nearest = int(np.argmin(np.abs(np.asarray(LANES)-x)))
            if abs(x-LANES[nearest]) > GATE_HALF_WIDTH or z > GATE_CEILING:
                self.abort("baffle")
            elif nearest != self.safe_lane:
                self.abort("heat")
        if abs(y-HEIGHT_GATE_Y) <= HEIGHT_GATE_ZONE:
            opening = next((i for i, (cx, _, _, lo, hi) in enumerate(PASSAGES)
                            if abs(x-cx) <= HEIGHT_HALF_WIDTH and lo <= z <= hi), None)
            if opening is None:
                self.abort("bulkhead")
            elif opening != self.safe_height:
                self.abort("height_heat")
        for cx, cy, sx, sy, height in CRATES:
            if (abs(x-cx) < sx+.14 and abs(y-cy) < sy+.14
                    and z < height+(.43 if self.held else .24)):
                self.abort("cabinet")

    def record_crossings(self, previous_y):
        _, y, z = self.position
        for prefix, plane in (("lane", 0.), ("height", HEIGHT_GATE_Y)):
            if previous_y < plane <= y:
                self.crossing_altitudes.setdefault(f"{prefix}_out", float(z))
            elif previous_y > plane >= y and self.held and "lifted" in self.progress:
                self.crossing_altitudes.setdefault(f"{prefix}_back", float(z))

    def update_progress(self):
        x, y, z = self.position
        px, py, pz = self.payload
        for name, (cx, cy) in (("survey_lane", BOARD), ("survey_height", HEIGHT_BOARD)):
            if z >= .72 and abs(x-cx) <= .68 and abs(y-cy) <= .68:
                self.progress.add(name)
        if "survey_lane" in self.progress and .81 < y < HEIGHT_GATE_Y-HEIGHT_GATE_ZONE:
            self.progress.add("lane_out")
        if {"lane_out", "survey_height"} <= self.progress and y > HEIGHT_GATE_Y+.80:
            self.progress.add("height_out")
        if ("height_out" in self.progress and z >= .57 and abs(px-x) < 1.45
                and abs(py-y-DOWN_OFFSET) < 1.45):
            self.progress.add("located")
        if self.held and pz >= .62:
            self.progress.add("lifted")
        if (self.held and "lifted" in self.progress and .81 < y < HEIGHT_GATE_Y-.81
                and py < HEIGHT_GATE_Y-.63 and "height_back" in self.crossing_altitudes):
            self.progress.add("height_back")
        if (self.held and "height_back" in self.progress and y < -.82 and py < -.62
                and "lane_back" in self.crossing_altitudes):
            self.progress.add("lane_back")
        if (not self.held and "lane_back" in self.progress
                and np.linalg.norm(self.payload[:2]-PAD) <= .28 and pz <= .095
                and np.linalg.norm(self.data.qvel[3:9]) <= .16):
            self.release_dwell += 1
            if self.release_dwell >= 50:
                self.progress.add("delivered")
                self.done = True
        else:
            self.release_dwell = 0

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
            previous_y = float(self.position[1])
            mujoco.mj_step(self.model, self.data)
            self.check_safety()
            if not self.done:
                self.record_crossings(previous_y)
                self.update_progress()
        if self.steps >= LIMIT:
            self.done = True
        return self.observe()

    def metrics(self):
        points = sum(SCORE[name] for name in self.progress)
        return {
            "rescue_progress": min(points, 65 if self.failure else 100)/100,
            "success": int("delivered" in self.progress and self.failure is None),
            **{name: int(name in self.progress) for name in SCORE},
            "steps": self.steps, "simulation_seconds": round(self.data.time, 3),
            "hazard_abort": int(self.failure in ("heat", "baffle", "cabinet", "bulkhead", "height_heat")),
            "unsafe_release": int(self.failure == "unsafe_release"),
            "close_attempts": self.close_attempts, "safe_lane": self.safe_lane+1,
            "safe_height": self.safe_height+1,
            **{f"{k}_z": round(self.crossing_altitudes.get(k, 0.), 4) for k in CROSSINGS},
        }
