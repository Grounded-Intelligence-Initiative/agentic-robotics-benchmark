"""Trusted MuJoCo world of the delivery bay. This module must never be agent-readable in a task.

One episode is one scene, fixed by its seed: a fenced 7.1 m square bay with two partition walls (each with a narrow
and a wide opening), a quadruped-shaped robot on a stabilized planar base with a turning gait, a four-button remote
whose buttons are walk forward / walk back / turn left / turn right (the assignment and the step sizes are drawn per
scene), and six delivery points visited in order. The robot reports its compass heading and the coordinates of the
active delivery point; its only view of the bay is its own forward camera under its own floodlight. After the third
delivery the robot is loaded and set back down facing another way, and every gait is slower. The same seed and the
same requests give the same episode with or without rendering: the verifier replays the service's ledger with
render=False.
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
VERSION = 3
DELIVERIES = 6
LOAD_AFTER = 3            # the load goes on when the agent advances from this delivery point
ACTION_LIMIT = 500        # button presses per episode
FRAME_LIMIT = 120         # camera frames per episode (robot_observe)
REQUEST_LIMIT = 660       # service requests per episode: every press and frame, the advances, and room for errors
PRESS_SECONDS = .2        # simulated time per press
SUBSTEPS = 20             # MuJoCo steps of 0.01 s per press
FENCE = 3.55              # inner face of the perimeter fence, |x| and |y| (m)
ROBOT_RADIUS = .16        # collision footprint of the base (m)
GOAL_TOLERANCE = .3       # the base's centre within this distance of a delivery point's centre (m)
LIGHT_RADIUS = 1.0        # the floodlight lights what is within this distance of the robot's centre, along the floor
WALL_HALF = .12           # half thickness of a partition wall (m)
WALL_HEIGHT = .7          # walls and fence, without the 0.024 m cap
CAMERA_HEIGHT = .25       # the camera's optical centre above the floor, on the robot's vertical axis (m)
CAMERA_FOV = 90.          # degrees, horizontal and vertical (square image)
DARK = 10                 # pixel value of everything the floodlight does not reach
YAW_INERTIA = 100.        # the stabilized base holds its heading: wall contacts do not turn it (kg m^2)
PAD_RGBA = ".04 .85 .18 1"   # the delivery pad: green, unlike the blue-grey panels and the concrete floor
BUTTONS = (0., 1/3, 2/3, 1.)
MOTIONS = ("forward", "backward", "left", "right")       # left and right are turns in place


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
        # Commanded speeds: m/s for the two walking gaits, rad/s for the two turns. The base settles at 0.875 of a
        # command, so one 0.2 s press is 0.21-0.26 m forward, 0.12-0.2 m back and 12-22 degrees of turn.
        self.speeds = {"forward": forward, "backward": forward*float(rng.uniform(.55, .75)),
                       "left": math.radians(float(rng.uniform(70., 125.))),
                       "right": math.radians(float(rng.uniform(70., 125.)))}
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
                "left_rate_deg": math.degrees(self.speeds["left"]),
                "right_rate_deg": math.degrees(self.speeds["right"]), "load": self.load,
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
        self.frames = 0                  # camera frames taken
        self.reached_at_steps = []       # press count at which each delivery point was reached
        self.privileged = False          # set by unlock(); the grader accepts it only with the oracle's token
        self._contacts = set()
        self._build()

    # --- model ---------------------------------------------------------------------------------------------------
    def _build(self):
        s = self.scene
        xml = header()
        for name, x, y, sx, sy in s.wall_boxes():
            xml += box(name, f"{x} {y} {WALL_HEIGHT/2}", f"{sx} {sy} {WALL_HEIGHT/2}", ".55 .6 .65 1")
        gx, gy = s.goals[0]
        xml += box("goal", f"{gx} {gy} .02", ".2 .2 .02", PAD_RGBA, False)
        # Slide joints first, then the yaw hinge: x and y stay the bay's axes whatever the heading.
        xml += f"""<body name="agent" pos="0 0 .23">
          <joint name="x" type="slide" axis="1 0 0" damping="5"/>
          <joint name="y" type="slide" axis="0 1 0" damping="5"/>
          <joint name="yaw" type="hinge" axis="0 0 1" armature="{YAW_INERTIA}" damping="{5*YAW_INERTIA}"/>
          <geom name="robot" type="cylinder" size="{ROBOT_RADIUS} .06" mass="1" rgba="1 .02 .02 1"/>
          <camera name="onboard" pos="0 0 {CAMERA_HEIGHT-.23}" xyaxes="0 -1 0 0 0 1" fovy="{CAMERA_FOV}"/>
        </body>"""
        xml += "</worldbody></mujoco>"
        xml = decorate(xml, KIND, .15, seed=s.seed ^ 0x72B49A)
        # The onboard camera comes within 0.16 m of a wall: a near plane of 2 cm (the default would be 0.34 m).
        xml = xml.replace("<visual>", '<visual><map znear="0.0006"/>', 1)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.agent_id = self.model.body("agent").id
        self.goal_geom = self.model.geom("goal").id
        self.camera_id = self.model.camera("onboard").id
        # The robot's own body is behind the lens: its geoms go to group 1, which the onboard view does not draw.
        for i in range(self.model.ngeom):
            if self.model.geom_bodyid[i] == self.agent_id:
                self.model.geom_group[i] = 1
        self.data.qpos[:2] = s.start
        self.data.qpos[2] = s.heading
        mujoco.mj_forward(self.model, self.data)
        self.renderer = Renderer(self.model, SIZE, SIZE) if self.render_enabled else None
        focal = (SIZE/2)/math.tan(math.radians(CAMERA_FOV)/2)
        u, v = np.meshgrid(np.arange(SIZE)+.5, np.arange(SIZE)+.5)
        rays = np.stack([(u-SIZE/2)/focal, -(v-SIZE/2)/focal, -np.ones_like(u)], axis=-1).reshape(-1, 3)
        self._rays = rays/np.linalg.norm(rays, axis=1)[:, None]      # unit rays in the camera's own frame

    def position(self):
        return self.data.xpos[self.agent_id][:2].copy()

    def yaw(self):
        return float(self.data.qpos[2])

    def goal(self):
        return np.array(self.scene.goals[self.current])

    # --- observations ------------------------------------------------------------------------------------------
    def lit(self):
        """Which pixels of the onboard view the floodlight reaches."""
        origin = self.data.cam_xpos[self.camera_id].copy()
        rays = self._rays@self.data.cam_xmat[self.camera_id].reshape(3, 3).T
        n = len(rays)
        geomid, dist = np.full(n, -1, np.int32), np.zeros(n)
        groups = np.array([1, 0, 1, 0, 0, 0], np.uint8)
        mujoco.mj_multiRay(self.model, self.data, origin, rays.ravel(), groups, 1, self.agent_id, geomid, dist, None,
                           n, 30.)
        hit = origin[None, :2]+rays[:, :2]*dist[:, None]
        p = self.position()
        visible = (geomid >= 0) & (((hit-p)**2).sum(axis=1) < LIGHT_RADIUS**2)
        return visible.reshape(SIZE, SIZE)

    def image(self):
        option = mujoco.MjvOption()
        option.geomgroup[1] = 0
        option.geomgroup[3] = 0
        self.renderer.update_scene(self.data, camera="onboard", scene_option=option)
        im = self.renderer.render().copy()
        # The bay is dark: the camera sees what the robot's floodlight lights and nothing else.
        im[~self.lit()] = DARK
        return im

    def status(self):
        gx, gy = self.goal()
        return {"delivery": self.current+1, "deliveries": DELIVERIES,
                "delivered": self.delivered, "goal_reached": self.delivered > self.current,
                "goal": {"x": round(float(gx), 2), "y": round(float(gy), 2)},
                "heading_deg": int(round(math.degrees(self.yaw()))) % 360,
                "steps_remaining": max(0, ACTION_LIMIT-self.steps),
                "frames_remaining": max(0, FRAME_LIMIT-self.frames), "done": self.done}

    def reply(self):
        result = self.status()
        if self.privileged:
            x, y = self.position()
            result["truth"] = {"x": float(x), "y": float(y), "yaw": self.yaw()}
        return result

    def observe(self):
        if self.frames >= FRAME_LIMIT:
            raise ValueError("camera frame budget exhausted; the robot can still be driven")
        self.frames += 1
        result = self.reply()
        sx, sy = self.scene.start
        result["start"] = {"x": round(float(sx), 2), "y": round(float(sy), 2),
                           "heading_deg": int(round(math.degrees(self.scene.heading))) % 360}
        if self.render_enabled:
            result["images"] = [png(self.image())]
        return result

    # --- actions -----------------------------------------------------------------------------------------------
    def command(self, button):
        """(forward speed m/s, yaw rate rad/s) that a button commands now."""
        s = self.scene
        motion = s.mapping[button]
        speed = s.speeds[motion]*(s.load if self.loaded else 1.)
        return {"forward": (speed, 0.), "backward": (-speed, 0.), "left": (0., speed), "right": (0., -speed)}[motion]

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
        walk, turn = self.command(button)
        for _ in range(SUBSTEPS):
            yaw = self.data.qpos[2]
            target = np.array([math.cos(yaw)*walk, math.sin(yaw)*walk])
            self.data.qfrc_applied[:2] = 35*(target-self.data.qvel[:2])
            self.data.qfrc_applied[2] = 35*YAW_INERTIA*(turn-self.data.qvel[2])
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
        return self.reply()

    def advance(self):
        if self.done:
            raise ValueError("episode finished")
        if self.delivered <= self.current:
            raise ValueError("reach the current delivery point before advancing")
        self.current += 1
        gx, gy = self.goal()
        self.model.geom_pos[self.goal_geom][:2] = [gx, gy]
        if self.current == LOAD_AFTER:
            # Loaded at the third point and set back down facing another way, standing still.
            self.loaded = True
            self.data.qpos[2] = self.scene.loaded_heading
            self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)
        return self.reply()

    def unlock(self, token_sha256):
        """The oracle's channel: the scene's ground truth, and the true pose in every later reply.

        The service hashes the token it is sent and records the hash; whether it is the oracle's token is decided by
        the grader, which holds the expected hash (tests/oracle_token.sha256) and zeroes any other run that used
        this channel."""
        if not isinstance(token_sha256, str):
            raise ValueError("token required")
        self.privileged = True
        s = self.scene
        result = self.reply()
        result["scene"] = {"walls": [[x, y, sx, sy] for _, x, y, sx, sy in s.wall_boxes()],
                           "mapping": list(s.mapping), "speeds": dict(s.speeds), "load": s.load,
                           "loaded_heading": s.loaded_heading, "goals": [[float(x), float(y)] for x, y in s.goals],
                           "start": [float(s.start[0]), float(s.start[1]), s.heading]}
        return result

    def metrics(self):
        legs = [b-a for a, b in zip([0]+self.reached_at_steps, self.reached_at_steps)]
        return {"delivered": self.delivered, "deliveries": DELIVERIES,
                "delivered_fraction": self.delivered/DELIVERIES, "success": float(self.delivered == DELIVERIES),
                "steps": self.steps, "frames": self.frames, "collisions": self.collisions,
                "simulated_seconds": self.steps*PRESS_SECONDS,
                **{f"leg_{i+1}_actions": n for i, n in enumerate(legs)}}

    def close(self):
        if self.renderer:
            self.renderer.close()
            self.renderer = None
