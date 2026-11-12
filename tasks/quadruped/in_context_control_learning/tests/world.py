"""Trusted MuJoCo world of the delivery bay. This module must never be agent-readable in a task.

One episode is one scene, fixed by its seed: a fenced 7.1 m square bay with two partition walls (each with a narrow
and a wide opening), a quadruped-shaped robot that walks without turning (a stabilized planar base), a four-button
remote whose buttons are walk forward / walk back / step left / step right in the robot's own frame (the assignment,
the robot's heading and its gait speeds are drawn per scene), and six delivery points visited in order. After the
third delivery the robot is loaded and set back down facing another way: every gait is slower and every button
moves it in a new direction. The same seed and the same requests give the same episode with or without rendering:
the verifier replays the service's ledger with render=False.
"""
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

from geometry import SIZE, box, header
from scene_assets import decorate
from pbr_renderer import Renderer

KIND = "icl"
VERSION = 2
DELIVERIES = 6
LOAD_AFTER = 3            # the load goes on when the agent advances from this delivery point
ACTION_LIMIT = 500        # button presses per episode
REQUEST_LIMIT = 560       # service requests per episode (observe, act and advance together)
PRESS_SECONDS = .2        # simulated time per press
SUBSTEPS = 20             # MuJoCo steps of 0.01 s per press
FENCE = 3.55              # inner face of the perimeter fence, |x| and |y| (m)
ROBOT_RADIUS = .16        # collision footprint of the base (m)
GOAL_TOLERANCE = .3       # the base's centre within this distance of a delivery point's centre (m)
LIGHT_RADIUS = 1.4        # the robot's floodlight lights the floor this far around the robot's centre (m)
BEACON_RADIUS = .25       # the active delivery point's beacon is visible from anywhere
WALL_HALF = .12           # half thickness of a partition wall (m)
BUTTONS = (0., 1/3, 2/3, 1.)
MOTIONS = ("forward", "backward", "left", "right")
BODY_DIRECTION = {"forward": (1., 0.), "backward": (-1., 0.), "left": (0., 1.), "right": (0., -1.)}


def png(image):
    buffer = io.BytesIO()
    Image.fromarray(image).save(buffer, format="PNG")
    return base64.b64encode(buffer.getvalue()).decode()


def _openings(rng):
    """A narrow and a wide opening of one partition, centres far enough apart that they never merge."""
    narrow = float(rng.uniform(.62, .72))
    wide = float(rng.uniform(1.0, 1.25))
    while True:
        centres = rng.uniform(-2.75, 2.75, 2)
        if abs(centres[0]-centres[1]) >= 1.7:
            return [(float(centres[0]), narrow), (float(centres[1]), wide)]


def _room(x, walls_x):
    return "W" if x < walls_x[0] else "M" if x < walls_x[1] else "E"


class Scene:
    """Everything a seed decides. A pure function of the seed (no rendering, no MuJoCo)."""

    def __init__(self, seed):
        rng = np.random.default_rng(int(seed))
        self.seed = int(seed)
        self.heading = float(rng.uniform(0, 2*math.pi))
        forward = float(rng.uniform(1.2, 1.5))
        self.speeds = {"forward": forward, "backward": forward*float(rng.uniform(.55, .75)),
                       "left": float(rng.uniform(.7, .95)), "right": float(rng.uniform(.7, .95))}
        self.mapping = [MOTIONS[i] for i in rng.permutation(4)]     # button index -> gait
        self.load = float(rng.uniform(.6, .75))
        turn = float(rng.uniform(math.radians(60), math.pi))*(1 if rng.uniform() < .5 else -1)
        self.loaded_heading = (self.heading+turn) % (2*math.pi)
        self.walls_x = [float(rng.uniform(-1.5, -.9)), float(rng.uniform(.9, 1.5))]
        self.openings = [_openings(rng), _openings(rng)]
        margin = .45
        x1, x2 = self.walls_x
        self.rooms = {"W": (-FENCE+margin, x1-WALL_HALF-margin), "M": (x1+WALL_HALF+margin, x2-WALL_HALF-margin),
                      "E": (x2+WALL_HALF+margin, FENCE-margin)}
        side = "W" if rng.uniform() < .5 else "E"
        self.start = self._point(rng, side)
        while True:
            rooms, points = [side], [self.start]
            for _ in range(DELIVERIES):
                previous = rooms[-1]
                choices = ["W", "E"] if previous == "M" else ["M", "E" if previous == "W" else "W"]
                room = choices[int(rng.integers(2))]
                for _ in range(200):
                    point = self._point(rng, room)
                    if np.linalg.norm(point-points[-1]) >= 1.5:
                        break
                rooms.append(room)
                points.append(point)
            if sum({a, b} == {"W", "E"} for a, b in zip(rooms, rooms[1:])) >= 2:
                break
        self.goal_rooms = rooms[1:]
        self.goals = points[1:]

    def _point(self, rng, room):
        lo, hi = self.rooms[room]
        return np.array([rng.uniform(lo, hi), rng.uniform(-FENCE+.45, FENCE-.45)])

    def wall_boxes(self):
        """(name, centre x, centre y, half x, half y) of every collision wall: the fence, then partition segments."""
        boxes = [("fence_w", -FENCE-.06, 0., .06, FENCE+.12), ("fence_e", FENCE+.06, 0., .06, FENCE+.12),
                 ("fence_s", 0., -FENCE-.06, FENCE, .06), ("fence_n", 0., FENCE+.06, FENCE, .06)]
        for k, (x, openings) in enumerate(zip(self.walls_x, self.openings)):
            cursor = -FENCE
            gaps = sorted((c-w/2, c+w/2) for c, w in openings)
            for j, (lo, hi) in enumerate(gaps + [(FENCE, FENCE)]):
                if lo > cursor:
                    boxes.append((f"wall{k}_{j}", x, (cursor+lo)/2, WALL_HALF, (lo-cursor)/2))
                cursor = hi
        return boxes

    def features(self):
        """Scalar features of the scene, for the hidden set's hollowing check (tests/review)."""
        rooms = [_room(self.start[0], self.walls_x)] + self.goal_rooms
        turn = (math.degrees(self.loaded_heading-self.heading)+180) % 360-180
        return {"heading_deg": math.degrees(self.heading), "loaded_turn_deg": turn,
                "forward_speed": self.speeds["forward"],
                "backward_ratio": self.speeds["backward"]/self.speeds["forward"],
                "left_speed": self.speeds["left"], "right_speed": self.speeds["right"], "load": self.load,
                "wall_x0": self.walls_x[0], "wall_x1": self.walls_x[1],
                "narrow0": self.openings[0][0][1], "wide0": self.openings[0][1][1],
                "narrow1": self.openings[1][0][1], "wide1": self.openings[1][1][1],
                "long_legs": sum({a, b} == {"W", "E"} for a, b in zip(rooms, rooms[1:])),
                "straight_path_m": float(sum(np.linalg.norm(b-a) for a, b in
                                             zip([self.start]+self.goals[:-1], self.goals)))}


class World:
    def __init__(self, seed, render=True):
        self.scene = Scene(seed)
        self.seed = self.scene.seed
        self.render_enabled = render
        self.steps = self.collisions = 0
        self.current = 0                 # index of the active delivery point
        self.delivered = 0               # delivery points reached so far
        self.loaded = False              # on from the advance after the LOAD_AFTER-th delivery
        self.done = False
        self.frame_id = 0
        self.reached_at_steps = []       # press count at which each delivery point was reached
        self._contacts = set()
        self._build()

    # --- model ---------------------------------------------------------------------------------------------------
    def _build(self):
        s = self.scene
        xml = header()
        for name, x, y, sx, sy in s.wall_boxes():
            xml += box(name, f"{x} {y} .35", f"{sx} {sy} .35", ".55 .6 .65 1")
        gx, gy = s.goals[0]
        xml += box("goal", f"{gx} {gy} .02", ".2 .2 .02", ".03 .12 1 1", False)
        xml += f"""<body name="agent" pos="0 0 .23">
          <joint name="x" type="slide" axis="1 0 0" damping="5"/>
          <joint name="y" type="slide" axis="0 1 0" damping="5"/>
          <geom name="robot" type="cylinder" size="{ROBOT_RADIUS} .06" mass="1" rgba="1 .02 .02 1"/>
          <body name="chassis" euler="0 0 {math.degrees(s.heading)}"/>
        </body>"""
        xml += "</worldbody></mujoco>"
        xml = decorate(xml, KIND, .15, seed=s.seed ^ 0x72B49A)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.agent_id = self.model.body("agent").id
        self.chassis_id = self.model.body("chassis").id
        self.goal_geom = self.model.geom("goal").id
        self.data.qpos[:2] = s.start
        mujoco.mj_forward(self.model, self.data)
        self.renderer = Renderer(self.model, SIZE, SIZE) if self.render_enabled else None

    def position(self):
        return self.data.xpos[self.agent_id][:2].copy()

    def goal(self):
        return np.array(self.scene.goals[self.current])

    # --- observations ------------------------------------------------------------------------------------------
    def image(self):
        option = mujoco.MjvOption()
        option.geomgroup[3] = 0
        self.renderer.update_scene(self.data, camera="top", scene_option=option)
        im = self.renderer.render().copy()
        xx = (np.arange(SIZE)+.5)/SIZE*8-4
        yy = 4-(np.arange(SIZE)+.5)/SIZE*8
        p, g = self.position(), self.goal()
        # The bay is dark: the ceiling camera sees what the robot's floodlight lights and the active beacon.
        visible = (xx[None, :]-p[0])**2 + (yy[:, None]-p[1])**2 < LIGHT_RADIUS**2
        visible |= (xx[None, :]-g[0])**2 + (yy[:, None]-g[1])**2 < BEACON_RADIUS**2
        im[~visible] = 10
        return im

    def status(self):
        return {"frame_id": self.frame_id, "delivery": self.current+1, "deliveries": DELIVERIES,
                "delivered": self.delivered, "goal_reached": self.delivered > self.current,
                "steps_remaining": max(0, ACTION_LIMIT-self.steps), "done": self.done}

    def observe(self):
        self.frame_id += 1
        result = self.status()
        if self.render_enabled:
            result["images"] = [png(self.image())]
        return result

    # --- actions -----------------------------------------------------------------------------------------------
    def velocity_command(self, button):
        s = self.scene
        motion = s.mapping[button]
        speed = s.speeds[motion]*(s.load if self.loaded else 1.)
        bx, by = BODY_DIRECTION[motion]
        heading = s.loaded_heading if self.loaded else s.heading
        c, sn = math.cos(heading), math.sin(heading)
        return np.array([c*bx-sn*by, sn*bx+c*by])*speed

    def act(self, action):
        if self.done or self.steps >= ACTION_LIMIT:
            raise ValueError("episode finished")
        if (not isinstance(action, list) or len(action) != 1 or isinstance(action[0], bool)
                or not isinstance(action[0], (int, float)) or not math.isfinite(action[0])):
            raise ValueError("action must be a list holding one button value")
        if action[0] not in BUTTONS:
            raise ValueError("buttons must be 0, 1/3, 2/3, or 1")
        button = int(round(action[0]*3))
        self.steps += 1
        command = self.velocity_command(button)
        for _ in range(SUBSTEPS):
            self.data.qfrc_applied[:2] = 35*(command-self.data.qvel[:2])
            mujoco.mj_step(self.model, self.data)
            touching = set()
            for contact in self.data.contact[:self.data.ncon]:
                if (self.model.geom_bodyid[contact.geom1] == self.agent_id
                        or self.model.geom_bodyid[contact.geom2] == self.agent_id):
                    touching.add(tuple(sorted((int(contact.geom1), int(contact.geom2)))))
            self.collisions += len(touching-self._contacts)
            self._contacts = touching
        mujoco.mj_forward(self.model, self.data)
        if self.delivered == self.current and np.linalg.norm(self.position()-self.goal()) < GOAL_TOLERANCE:
            self.delivered += 1
            self.reached_at_steps.append(self.steps)
        self.done = bool(self.delivered == DELIVERIES or self.steps >= ACTION_LIMIT)
        return self.observe()

    def advance(self):
        if self.done:
            raise ValueError("episode finished")
        if self.delivered <= self.current:
            raise ValueError("reach the current delivery point before advancing")
        self.current += 1
        gx, gy = self.goal()
        self.model.geom_pos[self.goal_geom][:2] = [gx, gy]
        if self.current == LOAD_AFTER:
            # Loaded at the third point and set back down facing another way (the body turns, the base does not).
            self.loaded = True
            half = self.scene.loaded_heading/2
            self.model.body_quat[self.chassis_id] = [math.cos(half), 0, 0, math.sin(half)]
            self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)
        return self.observe()

    def metrics(self):
        legs = [b-a for a, b in zip([0]+self.reached_at_steps, self.reached_at_steps)]
        return {"delivered": self.delivered, "deliveries": DELIVERIES,
                "delivered_fraction": self.delivered/DELIVERIES, "success": float(self.delivered == DELIVERIES),
                "steps": self.steps, "collisions": self.collisions, "simulated_seconds": self.steps*PRESS_SECONDS,
                **{f"leg_{i+1}_actions": n for i, n in enumerate(legs)}}

    def close(self):
        if self.renderer:
            self.renderer.close()
            self.renderer = None
