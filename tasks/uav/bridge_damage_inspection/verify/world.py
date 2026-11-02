"""Trusted MuJoCo worlds. This module must never be agent-readable in a task."""
import base64
import io
import math
import os

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
os.environ.setdefault("LP_NUM_THREADS", "2")
import mujoco
import numpy as np
from PIL import Image

from geometry import SIZE, arm_scene, box, header
from scene_assets import decorate, sync_visuals

KINDS = ("flight", "arm", "icl", "cross", "bridge", "rescue")
LIMITS = {"flight": 220, "arm": 180, "icl": 500, "cross": 320,
          "bridge": 160, "rescue": 160}


def crossed_gate(previous, current, center=(0., 0.)):
    """A directed crossing of the aperture, not merely a visit near its plane."""
    gate_x, gate_y = center
    if not previous[0] < gate_x <= current[0]:
        return False
    fraction = (gate_x-previous[0])/(current[0]-previous[0])
    crossing = previous + fraction*(current-previous)
    return bool(abs(crossing[1]-gate_y) < .6 and .6 < crossing[2] < 2.)


def png(image):
    buffer = io.BytesIO()
    Image.fromarray(image).save(buffer, format="PNG")
    return base64.b64encode(buffer.getvalue()).decode()


class World:
    def __init__(self, kind, seed, render=True):
        if kind not in KINDS:
            raise ValueError("unknown task")
        self.kind, self.seed = kind, int(seed)
        self.rng = np.random.default_rng(seed)
        self.render_enabled = render
        self.steps = self.collisions = self.damage = self.reports = 0
        self.phase = 0
        self.done = self.success = self.passed_gate = False
        self.grasped = self.broken = False
        self.stage_steps, self.stage_success = [], []
        self.mapping = self.rng.permutation(4).tolist()
        self.gain = float(self.rng.uniform(.85, 1.15))
        self.goal = np.array([2.2, float(self.rng.uniform(-.4, .4)), 1.2])
        self.start = np.array([-2.2, float(self.rng.uniform(-.3, .3)), .18])
        self.block = np.array([self.rng.uniform(-1.4, -.7), self.rng.uniform(-.8, .8)])
        self.tray = np.array([self.rng.uniform(.9, 1.5), self.rng.uniform(-.8, .8)])
        self.obstacle = np.array([0., self.rng.uniform(-.4, .4)])
        self.target = np.array([self.rng.uniform(-2.4, 2.4), self.rng.uniform(-1.5, 1.5)])
        if kind == "bridge":
            self.target[0] = self.rng.uniform(-1.9, 1.9)
        if kind == "rescue":
            while abs(self.target[0]) < .8 and abs(self.target[1] - .8) < .7:
                self.target = np.array([self.rng.uniform(-2.4, 2.4), self.rng.uniform(-1.5, 1.5)])
        self.last_frames = {}
        self.frame_id = 0
        self.completed = set()
        variant_rng = np.random.default_rng(int(seed) ^ 0x513EA7)
        self.gate = np.zeros(2)
        self.hover_steps = 0
        self.arm_gains = np.full(3, 120.)
        self.walls = []
        self.scene_seed = int(seed) ^ 0x72B49A
        scene_rng = np.random.default_rng(self.scene_seed)
        self.bridge_half_gap = float(scene_rng.uniform(.16, .25))
        self.bridge_half_width = float(scene_rng.uniform(.58, .82))
        self.bridge_y = float(scene_rng.uniform(-.38, .38))
        self.buildings = [
            (float(x+scene_rng.uniform(-.25, .25)), float(y+scene_rng.uniform(-.2, .2)),
             float(scene_rng.uniform(.32, .52)), float(scene_rng.uniform(.32, .52)),
             float(scene_rng.uniform(.75, 1.15)))
            for x, y in ((-1, -2.3), (1, 2.3), (0, .8))
        ]
        self.target_yaw = float(scene_rng.uniform(-math.pi, math.pi))
        self.nav_starts = [np.array([-2.8, y])+scene_rng.uniform(-.12, .12, 2)
                           for y in (-1.8, 1.8, -1.8, 1.8)]
        self.nav_goals = [np.array([2.6, y])+scene_rng.uniform(-.15, .15, 2)
                          for y in (1.7, -1.7, 1.7, -1.7)]
        self.large_radius = float(scene_rng.uniform(.26, .32))
        if kind == "rescue":
            # Keep the complete mannequin clear of opaque roofs; every sampled
            # target must admit real image evidence, not a lucky guessed report.
            for _ in range(1000):
                if all(abs(self.target[0]-x) > sx+.95 or abs(self.target[1]-y) > sy+.95
                       for x, y, sx, sy, _ in self.buildings):
                    break
                self.target = scene_rng.uniform([-2.4, -1.5], [2.4, 1.5])
            else:
                raise RuntimeError("could not sample a visible rescue target")
        if kind == "flight":
            self.gate = variant_rng.uniform([-.6, -.9], [.4, .9])
            self.start[:2] = variant_rng.uniform([-2.9, -.9], [-2.2, .9])
            self.goal[:2] = variant_rng.uniform([2., -1.], [2.7, 1.])
            self.goal[2] = 1.3
        elif kind == "arm":
            # Visually identical initial scenes can have different actuator responses.
            self.arm_gains = variant_rng.uniform(35., 230., 3)
        elif kind in ("icl", "cross"):
            wall_x = float(variant_rng.uniform(-.45, .45))
            narrow = float(variant_rng.uniform(-.45, .45))
            wide = float(variant_rng.choice([-1., 1.])*variant_rng.uniform(1.9, 2.25))
            openings = sorted([(narrow-.3, narrow+.3), (wide-.65, wide+.65)])
            cursor = -3.45
            for lo, hi in openings + [(3.45, 3.45)]:
                self.walls.append((wall_x, (cursor+lo)/2, .12, (lo-cursor)/2))
                cursor = hi
        self._build()

    def _build(self):
        if hasattr(self, "renderer") and self.renderer:
            self.renderer.close()
        self.radius = .13 if self.phase == 0 else self.large_radius
        if self.kind == "arm":
            xml = arm_scene(self.block, self.tray, self.obstacle, self.arm_gains)
        else:
            xml = header()
            if self.kind == "flight":
                gx, gy = self.gate
                for name, pos, size in (
                    ("gate_left", f"{gx} {gy-.8} 1.1", ".07 .08 1.1"),
                    ("gate_right", f"{gx} {gy+.8} 1.1", ".07 .08 1.1"),
                    ("gate_top", f"{gx} {gy} 2.2", ".07 .88 .07"),
                ):
                    xml += box(name, pos, size, "1 .7 .05 1")
                xml += box("goal", f"{self.goal[0]} {self.goal[1]} .025",
                           ".28 .28 .025", ".03 .12 1 1", False)
                xml += """<body name="agent" pos="0 0 0"><freejoint/>
                  <geom name="robot" type="box" size=".17 .14 .055" mass=".8"
                        rgba="1 .02 .02 1"/>
                  <geom type="capsule" fromto="-.23 -.23 0 .23 .23 0" size=".025"
                        mass=".01" rgba=".5 .55 .6 1"/>
                  <geom type="capsule" fromto="-.23 .23 0 .23 -.23 0" size=".025"
                        mass=".01" rgba=".5 .55 .6 1"/></body>"""
            else:
                if self.kind in ("icl", "cross"):
                    self.goal = np.r_[self.nav_goals[self.phase], .22]
                    # Both embodiments see the same sampled map, not a memorized layout.
                    for i, (cx, cy, sx, sy) in enumerate(self.walls):
                        xml += box(f"wall{i}", f"{cx} {cy} .35", f"{sx} {sy} .35",
                                   ".55 .6 .65 1")
                    xml += box("goal", f"{self.goal[0]} {self.goal[1]} .02",
                               ".2 .2 .02", ".03 .12 1 1", False)
                elif self.kind == "bridge":
                    self.target[1] = self.bridge_y
                    # A missing deck segment, not a semantic target label.
                    left = self.target[0] - self.bridge_half_gap
                    right = self.target[0] + self.bridge_half_gap
                    xml += box("deck_left", f"{(-3+left)/2} {self.bridge_y} .55",
                               f"{(left+3)/2} {self.bridge_half_width} .06", ".65 .67 .7 1")
                    xml += box("deck_right", f"{(right+3)/2} {self.bridge_y} .55",
                               f"{(3-right)/2} {self.bridge_half_width} .06", ".65 .67 .7 1")
                    xml += box("water", "0 0 .01", "3.3 1.4 .01", ".02 .25 .75 1", False)
                    for bx in (-2.4, 2.4):
                        xml += box(f"pier{bx}", f"{bx} {self.bridge_y} .25", ".16 .55 .25", ".5 .5 .5 1")
                elif self.kind == "rescue":
                    for i, (x, y, sx, sy, h) in enumerate(self.buildings):
                        xml += box(f"building{i}", f"{x} {y} {h/2}", f"{sx} {sy} {h/2}",
                                   ".5 .55 .6 1")
                    tx, ty = self.target
                    xml += f"""<body pos="{tx} {ty} .12" euler="0 0 {math.degrees(self.target_yaw)}">
                      <geom name="victim" type="capsule" fromto="-.13 0 0 .13 0 0"
                            size=".08" rgba="1 .45 0 1"/>
                      <geom type="sphere" pos=".23 0 0" size=".08" rgba="1 .7 .4 1"/>
                    </body>"""
                z = 1.6 if self.kind in ("bridge", "rescue") else .23
                rad = self.radius if self.kind == "cross" else .13
                xml += f"""<body name="agent" pos="0 0 {z}">
                  <joint name="x" type="slide" axis="1 0 0" damping="5"/>
                  <joint name="y" type="slide" axis="0 1 0" damping="5"/>
                  <geom name="robot" type="box" size="{rad} {rad} .06"
                        mass="1" rgba="1 .02 .02 1"/>"""
                if self.kind in ("icl", "cross"):
                    for lx in (-rad, rad):
                        for ly in (-rad, rad):
                            xml += f'<geom type="capsule" fromto="{lx} {ly} 0 {lx} {ly} -.15" size=".025" mass=".02" rgba=".4 .45 .5 1"/>'
                else:
                    xml += '<geom type="capsule" fromto="-.25 -.25 0 .25 .25 0" size=".025" mass=".01" rgba=".6 .6 .6 1"/>'
                xml += "</body>"
            xml += "</worldbody></mujoco>"
        xml = decorate(xml, self.kind, self.radius if self.kind == "cross" else .13,
                       seed=self.scene_seed)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.agent_id = self.model.body("agent").id
        if self.kind == "flight":
            self.data.qpos[:3] = self.start
        elif self.kind == "arm":
            self.data.qpos[:3] = [-1.9, -1.7, 1.0]
            self.command = self.data.qpos[:3].copy()
            self.data.ctrl[:] = self.command
        else:
            self.data.qpos[:2] = self.nav_starts[self.phase] if self.kind in ("icl", "cross") else [-2.8, -1.8]
        mujoco.mj_forward(self.model, self.data)
        self._contacts = set()
        self.renderer = mujoco.Renderer(self.model, SIZE, SIZE) if self.render_enabled else None

    def position(self):
        return self.data.xpos[self.agent_id].copy()

    def images(self):
        sync_visuals(self.model, self.data)
        images = []
        for camera in ("top", "side"):
            if camera == "side" and self.kind in ("icl", "cross", "bridge", "rescue"):
                images.append(np.full((SIZE, SIZE, 3), 10, dtype=np.uint8))
                continue
            option = mujoco.MjvOption()
            option.geomgroup[3] = 0
            self.renderer.update_scene(self.data, camera=camera, scene_option=option)
            # Fixed diffuse instrument lighting keeps CPU observations affordable;
            # reviewer cameras may additionally render soft scene shadows.
            self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            im = self.renderer.render().copy()
            if camera == "top" and self.kind in ("icl", "cross", "bridge", "rescue"):
                xx = (np.arange(SIZE) + .5) / SIZE * 8 - 4
                yy = 4 - (np.arange(SIZE) + .5) / SIZE * 8
                visible = (xx[None, :] - self.position()[0])**2 + (yy[:, None] - self.position()[1])**2 < 1.4**2
                # Navigation goal is a task-supplied visual beacon, not a map of obstacles.
                if self.kind in ("icl", "cross"):
                    visible |= ((xx[None, :] - self.goal[0])**2 +
                                (yy[:, None] - self.goal[1])**2 < .25**2)
                im[~visible] = 10
            images.append(im)
        return images

    def observe(self):
        self.frame_id += 1
        imgs = self.images()
        self.last_frames[self.frame_id] = {
            "position": self.position().tolist(),
            "target_visible": self._target_visible(),
        }
        return {"images": [png(im) for im in imgs], "frame_id": self.frame_id,
                "steps_remaining": max(0, LIMITS[self.kind] - self.steps),
                "damage_remaining": max(0., 40 - self.damage),
                "phase": self.phase, "done": self.done,
                "completed_stages": len(self.completed)}

    def _target_visible(self):
        if self.kind not in ("bridge", "rescue"):
            return False
        if np.linalg.norm(self.position()[:2] - self.target) >= 1.25:
            return False
        option = mujoco.MjvOption()
        option.geomgroup[3] = 0
        self.renderer.update_scene(self.data, camera="top", scene_option=option)
        self.renderer.enable_segmentation_rendering()
        try:
            segment = self.renderer.render().copy()
        finally:
            self.renderer.disable_segmentation_rendering()
        x = int((self.target[0]+4)*32)
        y = int((4-self.target[1])*32)
        patch = segment[max(0, y-3):y+4, max(0, x-3):x+4, 0]
        expected = self.model.geom("water" if self.kind == "bridge" else "victim").id
        return bool((patch == expected).sum() >= 4)

    def act(self, action):
        if self.done or self.steps >= LIMITS[self.kind]:
            raise ValueError("episode finished")
        if not isinstance(action, list) or not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in action):
            raise ValueError("finite numeric action list required")
        n = 4 if self.kind in ("flight", "arm") else (1 if self.kind in ("icl", "cross") else 2)
        if len(action) != n or any(abs(x) > 1 for x in action):
            raise ValueError("wrong action shape or range")
        if self.kind in ("icl", "cross") and action[0] not in (0., 1/3, 2/3, 1.):
            raise ValueError("buttons must be 0, 1/3, 2/3, or 1")
        self.steps += 1
        previous_position = self.position()
        if self.kind == "arm":
            self.command = np.clip(self.command + np.array(action[:3]) * .14,
                                   [-2.6, -2.6, .17], [2.6, 2.6, 1.5])
            obj = self.data.body("object").xpos.copy()
            if action[3] > .5 and not self.grasped and np.linalg.norm(self.position() - obj) < .24 and not self.broken:
                self.model.eq_data[0, 3:6] = obj - self.position()
                self.model.eq_data[0, 6:10] = [1, 0, 0, 0]
                self.data.eq_active[0] = 1
                self.grasped = True
            elif action[3] < -.5:
                self.data.eq_active[0] = 0
                self.grasped = False
        for _ in range(20):
            self.data.xfrc_applied[:] = 0
            if self.kind == "flight":
                rot = self.data.xmat[self.agent_id].reshape(3, 3)
                self.data.xfrc_applied[self.agent_id, :3] = rot[:, 2] * ((action[0] + 1) * 8.2)
                desired = np.array([action[1] * .3, action[2] * .3, action[3] * .4])
                roll = math.atan2(rot[2, 1], rot[2, 2])
                pitch = math.asin(float(np.clip(-rot[2, 0], -1, 1)))
                yaw = math.atan2(rot[1, 0], rot[0, 0])
                self.data.xfrc_applied[self.agent_id, 3:] = .5 * (desired - [roll, pitch, yaw]) - .12 * self.data.qvel[3:6]
            elif self.kind == "arm":
                self.data.ctrl[:] = self.command
                self.data.qfrc_applied[:3] = [0, 0, 5.01]
            else:
                if self.kind in ("icl", "cross"):
                    index = int(round(action[0] * 3))
                    direction = ((1, 0), (-1, 0), (0, 1), (0, -1))[self.mapping[index]]
                    speed = 1.2 if self.kind == "icl" or self.phase == 0 else 1.0
                    velocity = np.array(direction) * self.gain * speed
                else:
                    velocity = np.array(action) * 1.6
                self.data.qfrc_applied[:2] = 35 * (velocity - self.data.qvel[:2])
            mujoco.mj_step(self.model, self.data)
            robot_contacts = set()
            for contact_index, contact in enumerate(self.data.contact):
                names = {self.model.geom(contact.geom1).name, self.model.geom(contact.geom2).name}
                if self.kind == "arm" and "fragile" in names and not self.broken:
                    force = np.zeros(6)
                    mujoco.mj_contactForce(self.model, self.data, contact_index, force)
                    if np.linalg.norm(force[:3]) > 50:
                        self.broken = True
                        self.damage += 20
                elif self.kind != "arm" and "floor" not in names and (
                        self.model.geom_bodyid[contact.geom1] == self.agent_id
                        or self.model.geom_bodyid[contact.geom2] == self.agent_id):
                    robot_contacts.add(tuple(sorted((int(contact.geom1), int(contact.geom2)))))
            self.collisions += len(robot_contacts-self._contacts)
            self._contacts = robot_contacts
        p = self.position()
        if self.kind == "flight":
            self.passed_gate |= crossed_gate(previous_position, p, self.gate)
            stable = (self.passed_gate and np.linalg.norm(p-self.goal) < .3
                      and np.linalg.norm(self.data.qvel[:3]) < .35
                      and np.linalg.norm(self.data.qvel[3:6]) < .5 and not self.collisions)
            self.hover_steps = self.hover_steps + 1 if stable else 0
            self.success = self.hover_steps >= 10
        elif self.kind == "arm":
            obj = self.data.body("object").xpos
            self.success = bool(np.linalg.norm(obj[:2]-self.tray) < .20 and obj[2] < .19 and not self.grasped and not self.broken)
        elif self.kind in ("icl", "cross"):
            reached = np.linalg.norm(p[:2]-self.goal[:2]) < .3
            if reached:
                self.completed.add(self.phase)
            self.success = bool(reached and (self.kind == "icl" and self.phase == 3 or self.kind == "cross" and self.phase == 1))
        self.done = bool(self.success or self.steps >= LIMITS[self.kind] or self.damage > 40 or
                         np.any(np.abs(p[:2]) > 3.8) or self.kind == "flight" and p[2] > 4)
        return self.observe() if self.render_enabled else {}

    def advance(self):
        if self.kind not in ("icl", "cross") or self.done:
            raise ValueError("no phase transition")
        if self.kind == "icl" and (self.phase not in self.completed or self.phase >= 3):
            raise ValueError("complete current goal before advancing")
        if self.kind == "cross" and (self.phase != 0 or self.steps < 12):
            raise ValueError("scout for at least 12 actions; only one body switch")
        self.stage_steps.append(self.steps)
        self.phase += 1
        if self.kind == "cross":
            self.mapping = self.rng.permutation(4).tolist()
            self.gain = float(self.rng.uniform(.7, .9))
        self._build()
        return self.observe()

    def report(self, frame_id, point):
        if (self.kind not in ("bridge", "rescue") or self.success or self.reports >= 3
                or np.any(np.abs(self.position()[:2]) > 3.8)):
            raise ValueError("report not permitted")
        if not isinstance(frame_id, int) or isinstance(frame_id, bool):
            raise ValueError("frame_id must be an integer")
        if not isinstance(point, list) or len(point) != 2 or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in point):
            raise ValueError("report a finite 2D point")
        self.reports += 1
        frame = self.last_frames.get(frame_id)
        self.success = bool(frame and frame["target_visible"] and np.linalg.norm(np.array(point)-self.target) < .3)
        self.done = bool(self.success or self.reports >= 3 or self.steps >= LIMITS[self.kind])
        return self.observe()

    def metrics(self):
        return {"success": float(self.success), "steps": self.steps,
                "collisions": self.collisions, "damage_cost": self.damage,
                "completed_stages": len(self.completed), "reports": self.reports,
                "hover_seconds": self.hover_steps*.2 if self.kind == "flight" else 0.,
                "scout_actions": self.stage_steps[0] if self.kind == "cross" and self.stage_steps else 0,
                "post_switch_actions": self.steps-self.stage_steps[0]
                    if self.kind == "cross" and self.stage_steps else 0,
                **{f"stage_{i}_actions": end-start for i, (start, end) in enumerate(
                    zip([0]+self.stage_steps, self.stage_steps+[self.steps]))}}

    def close(self):
        if self.renderer:
            self.renderer.close()
