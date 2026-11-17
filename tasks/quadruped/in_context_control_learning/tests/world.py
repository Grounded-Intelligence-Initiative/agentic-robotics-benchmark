"""Trusted v3 delivery bay: onboard vision and force-driven planar pose docking.

The Go1 meshes are visual only, NOT a gait simulation. Three MuJoCo coordinates
(x, y, yaw), bounded body-frame forces, actuator lag and surface-dependent lateral
traction model a stabilized nonholonomic base. No action assigns pose or velocity.
The separate verifier replays the unchanged service ledger without rendering.
"""
import base64
from dataclasses import dataclass
import hashlib
import io
import json
import math
import os
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
os.environ.setdefault("LP_NUM_THREADS", "2")
import mujoco
import numpy as np
from PIL import Image

from geometry import box, header
from scene_assets import decorate
from pbr_renderer import Renderer

KIND = "icl"
VERSION = 3
DELIVERIES = 6
LOAD_AFTER = 3            # the load goes on when the agent advances from this delivery point
ACTION_LIMIT = 500        # button presses per episode
REQUEST_LIMIT = 560       # service requests per episode (observe, act and advance together)
DT = .01
DEFAULT_FRAMES = 30
MAX_FRAMES = 120
SIZE = 384
FENCE = 3.55              # inner face of the perimeter fence, |x| and |y| (m)
ROBOT_RADIUS = .16        # collision footprint of the base (m)
GOAL_TOLERANCE = .16
GOAL_RELEASE = .19
ANGLE_TOLERANCE = math.radians(12)
ANGLE_RELEASE = math.radians(16)
STOP_SPEED = .045
STOP_YAW = .08
DOCK_FRAMES = 50
TERRAIN_HYSTERESIS = .04
WALL_HALF = .12           # half thickness of a partition wall (m)
BUTTONS = (0., 1/3, 2/3, 1.)
MOTIONS = ("forward", "turn_left", "turn_right", "brake")
BRAKE = 3
# speed scale, longitudinal traction, lateral damping, yaw traction, lag scale
SURFACES = ((1., 1., 38., 1., 1.), (.90, .43, 7., .60, 1.45),
            (.62, .85, 48., .78, 1.15))


def wrap(angle):
    return (angle+math.pi) % (2*math.pi)-math.pi


@dataclass
class Response:
    drive: float = 0.
    turn: float = 0.
    terrain: int = 0


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
        # The permutation is unknown but fixed for the entire episode, including
        # loading. Retain v2's random draws to preserve its registered geometry.
        self.mapping = [MOTIONS[i] for i in rng.permutation(4)]
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
        self.goal_headings = rng.uniform(-math.pi, math.pi, DELIVERIES)
        self.drive_lag = float(rng.uniform(.18, .30))
        self.turn_lag = float(rng.uniform(.16, .26))
        self.turn_rates = (float(rng.uniform(1.05, 1.35)), float(rng.uniform(1.05, 1.35)))
        self.bands = [(float(rng.uniform(-1.8, -1.1)), .48, 1),
                      (float(rng.uniform(.9, 1.7)), .43, 2)]

    def terrain_at(self, position, previous=0):
        """Schmitt boundaries prevent chatter; paint is at the nominal edge."""
        y = float(position[1])
        for centre, half, kind in self.bands:
            margin = TERRAIN_HYSTERESIS if previous == kind else -TERRAIN_HYSTERESIS
            if abs(y-centre) <= half+margin:
                return kind
        return 0

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
        return {"version": VERSION, "heading_deg": math.degrees(self.heading),
                "forward_speed": self.speeds["forward"],
                "backward_ratio": self.speeds["backward"]/self.speeds["forward"],
                "drive_lag": self.drive_lag, "turn_lag": self.turn_lag, "load": self.load,
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
        self.physics_frames = 0
        self.response = Response()
        self.dock_armed = False
        self.dock_frames = 0
        self.reached_at_steps = []       # press count at which each delivery point was reached
        self._contacts = set()
        self._build()
        self.response.terrain = self.scene.terrain_at(self.position())

    # --- model ---------------------------------------------------------------------------------------------------
    def _build(self):
        s = self.scene
        xml = header()
        for name, x, y, sx, sy in s.wall_boxes():
            xml += box(name, f"{x} {y} .35", f"{sx} {sy} .35", ".55 .6 .65 1")
        gx, gy = s.goals[0]
        xml += f'<body name="dock" pos="{gx} {gy} 0" euler="0 0 {math.degrees(s.goal_headings[0])}">'
        xml += box("goal", "0 0 .008", ".25 .25 .008", ".03 .12 1 1", False)
        xml += box("dock_arrow_shaft", "-.04 0 .019", ".14 .018 .003", ".95 .95 .95 1", False)
        for sign in (-1, 1):
            xml += (f'<geom name="dock_arrow_{sign}" type="capsule" fromto=".13 0 .022 .04 {sign*.09} .022" '
                    'size=".018" rgba=".95 .95 .95 1" contype="0" conaffinity="0"/>')
        xml += box("dock_centre", "0 0 .024", ".015 .07 .002", ".95 .95 .95 1", False)
        # A physical elevated beacon, occluded normally and absent outside the FOV.
        xml += '<geom name="beacon" type="sphere" pos="0 0 1.1" size=".06" rgba=".03 .12 1 1" contype="0" conaffinity="0"/>'
        xml += '</body>'
        for centre, half, kind in s.bands:
            color = ".03 .55 .58 1" if kind == 1 else ".75 .48 .10 1"
            xml += box(f"terrain_{kind}", f"0 {centre} .002", f"{FENCE} {half} .002", color, False)
            for j, x in enumerate(np.arange(-FENCE+.1, FENCE, .3)):
                xml += box(f"terrain_mark_{kind}_{j}", f"{x} {centre} .005", ".012 .15 .001", ".13 .16 .17 1", False)
        xml += f"""<body name="agent" pos="0 0 .23">
          <joint name="x" type="slide" axis="1 0 0" damping="0"/>
          <joint name="y" type="slide" axis="0 1 0" damping="0"/>
          <joint name="yaw" type="hinge" axis="0 0 1" damping="0"/>
          <inertial pos="0 0 0" mass="8" diaginertia=".12 .12 .16"/>
          <geom name="robot" type="cylinder" size="{ROBOT_RADIUS} .06" rgba="1 .02 .02 1"/>
          <camera name="onboard" pos="-.12 0 .65" xyaxes="0 -1 0 .573576 0 .819152" fovy="100"/>
          <body name="chassis"/>
        </body>"""
        xml += "</worldbody></mujoco>"
        xml = decorate(xml, KIND, .15, seed=s.seed ^ 0x72B49A)
        root = ET.fromstring(xml)
        # There is no global camera, even in the compiled model.
        for camera in list(root.find("worldbody").findall("camera")):
            root.find("worldbody").remove(camera)
        xml = ET.tostring(root, encoding="unicode")
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.agent_id = self.model.body("agent").id
        self.chassis_id = self.model.body("chassis").id
        self.dock_id = self.model.body("dock").id
        self.data.qpos[:2] = s.start
        self.data.qpos[2] = s.heading
        mujoco.mj_forward(self.model, self.data)
        self.renderer = Renderer(self.model, SIZE, SIZE) if self.render_enabled else None

    def position(self):
        return self.data.xpos[self.agent_id][:2].copy()

    def goal(self):
        return np.array(self.scene.goals[self.current])

    def heading(self):
        return float(self.data.qpos[2])

    def state_fingerprint(self):
        """Private replay attestation, never an observation or service operation.

        Include MuJoCo's complete integration state (including warm starts),
        model mutations and every history-dependent task variable. Frame IDs are
        request bookkeeping, deliberately excluded: observation freezes physics.
        """
        spec = mujoco.mjtState.mjSTATE_INTEGRATION
        state = np.empty(mujoco.mj_stateSize(self.model, spec))
        mujoco.mj_getState(self.model, self.data, state, spec)
        payload = {"version": VERSION, "seed": self.seed, "physics": state.tolist(),
                   "mass": self.model.body_mass.tolist(), "inertia": self.model.body_inertia.tolist(),
                   "body_pos": self.model.body_pos.tolist(), "body_quat": self.model.body_quat.tolist(),
                   "response": vars(self.response), "loaded": self.loaded, "current": self.current,
                   "delivered": self.delivered, "dock_armed": self.dock_armed, "dock_frames": self.dock_frames,
                   "steps": self.steps, "physics_frames": self.physics_frames, "done": self.done,
                   "contacts": sorted(self._contacts), "collisions": self.collisions,
                   "reached_at_steps": self.reached_at_steps}
        return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":"),
                                         allow_nan=False).encode()).hexdigest()

    # --- observations ------------------------------------------------------------------------------------------
    def image(self):
        option = mujoco.MjvOption()
        option.geomgroup[3] = 0
        self.renderer.update_scene(self.data, camera="onboard", scene_option=option)
        return self.renderer.render().copy()

    def status(self):
        return {"frame_id": self.frame_id, "delivery": self.current+1, "deliveries": DELIVERIES,
                "delivered": self.delivered, "goal_reached": self.delivered > self.current,
                "loaded": self.loaded,
                "steps_remaining": max(0, ACTION_LIMIT-self.steps), "done": self.done}

    def observe(self):
        self.frame_id += 1
        result = self.status()
        if self.render_enabled:
            result["images"] = [png(self.image())]
        return result

    # --- actions -----------------------------------------------------------------------------------------------
    def integrate(self, data, response, button):
        """One physical tick, also usable by the AUTHOR-ONLY reference predictor.

        Lateral force is dissipative traction, never a commanded sideways speed.
        Load changes inertia as well as available speed; terrain affects actual
        acceleration and slip, not a noisy or misleading observation overlay.
        """
        motion = self.scene.mapping[button]
        response.terrain = self.scene.terrain_at(data.qpos[:2], response.terrain)
        speed, grip, lateral, yaw_grip, lag = SURFACES[response.terrain]
        load = self.scene.load if self.loaded else 1.
        target_drive = self.scene.speeds["forward"]*.8*speed*load if motion == "forward" else 0.
        target_turn = (self.scene.turn_rates[0] if motion == "turn_left" else
                       -self.scene.turn_rates[1] if motion == "turn_right" else 0.)
        target_turn *= yaw_grip*math.sqrt(load)
        response.drive += (target_drive-response.drive)*(-math.expm1(-DT/(self.scene.drive_lag*lag)))
        response.turn += (target_turn-response.turn)*(-math.expm1(-DT/(self.scene.turn_lag*lag)))
        c, sn = math.cos(data.qpos[2]), math.sin(data.qpos[2])
        forward, left = np.array([c, sn]), np.array([-sn, c])
        vf, vl = float(data.qvel[:2]@forward), float(data.qvel[:2]@left)
        force = float(np.clip(24*grip*(response.drive-vf), -22*grip, 22*grip))
        data.qfrc_applied[:2] = force*forward-lateral*vl*left
        data.qfrc_applied[2] = np.clip(1.4*yaw_grip*(response.turn-data.qvel[2]), -1.8*yaw_grip, 1.8*yaw_grip)
        mujoco.mj_step(self.model, data)

    def update_docking(self):
        if self.delivered > self.current:
            return
        distance = float(np.linalg.norm(self.data.qpos[:2]-self.goal()))
        angle = abs(wrap(self.heading()-self.scene.goal_headings[self.current]))
        if distance > GOAL_RELEASE or angle > ANGLE_RELEASE:
            self.dock_armed = False
        elif distance <= GOAL_TOLERANCE and angle <= ANGLE_TOLERANCE:
            self.dock_armed = True
        stopped = np.linalg.norm(self.data.qvel[:2]) <= STOP_SPEED and abs(self.data.qvel[2]) <= STOP_YAW
        self.dock_frames = self.dock_frames+1 if self.dock_armed and stopped else 0
        if self.dock_frames >= DOCK_FRAMES:
            self.delivered += 1
            self.reached_at_steps.append(self.steps)

    def act(self, action, frames=DEFAULT_FRAMES):
        if self.done or self.steps >= ACTION_LIMIT:
            raise ValueError("episode finished")
        if (not isinstance(action, list) or len(action) != 1 or isinstance(action[0], bool)
                or not isinstance(action[0], (int, float)) or not math.isfinite(action[0])):
            raise ValueError("action must be a list holding one button value")
        if action[0] not in BUTTONS:
            raise ValueError("buttons must be 0, 1/3, 2/3, or 1")
        if isinstance(frames, bool) or not isinstance(frames, int) or not 1 <= frames <= MAX_FRAMES:
            raise ValueError("frames must be an integer from 1 to 120")
        button = int(round(action[0]*3))
        self.steps += 1
        for _ in range(frames):
            self.integrate(self.data, self.response, button)
            self.physics_frames += 1
            touching = set()
            for contact in self.data.contact[:self.data.ncon]:
                if (self.model.geom_bodyid[contact.geom1] == self.agent_id
                        or self.model.geom_bodyid[contact.geom2] == self.agent_id):
                    touching.add(tuple(sorted((int(contact.geom1), int(contact.geom2)))))
            self.collisions += len(touching-self._contacts)
            self._contacts = touching
            self.update_docking()
        mujoco.mj_forward(self.model, self.data)
        self.done = bool(self.delivered == DELIVERIES or self.steps >= ACTION_LIMIT)
        return self.observe()

    def advance(self):
        if self.done:
            raise ValueError("episode finished")
        if self.delivered <= self.current:
            raise ValueError("reach the current delivery point before advancing")
        self.current += 1
        gx, gy = self.goal()
        self.model.body_pos[self.dock_id][:2] = [gx, gy]
        half = self.scene.goal_headings[self.current]/2
        self.model.body_quat[self.dock_id] = [math.cos(half), 0, 0, math.sin(half)]
        self.dock_armed = False
        self.dock_frames = 0
        if self.current == LOAD_AFTER:
            self.loaded = True
            self.model.body_mass[self.agent_id] /= self.scene.load
            self.model.body_inertia[self.agent_id] /= self.scene.load
            # Recompute MuJoCo's derived inertia without resetting the live state.
            qpos, qvel, time = self.data.qpos.copy(), self.data.qvel.copy(), self.data.time
            mujoco.mj_setConst(self.model, self.data)
            self.data.qpos[:], self.data.qvel[:], self.data.time = qpos, qvel, time
        mujoco.mj_forward(self.model, self.data)
        return self.observe()

    def metrics(self):
        legs = [b-a for a, b in zip([0]+self.reached_at_steps, self.reached_at_steps)]
        return {"delivered": self.delivered, "deliveries": DELIVERIES,
                "delivered_fraction": self.delivered/DELIVERIES, "success": float(self.delivered == DELIVERIES),
                "steps": self.steps, "collisions": self.collisions, "simulated_seconds": self.physics_frames*DT,
                **{f"leg_{i+1}_actions": n for i, n in enumerate(legs)}}

    def close(self):
        if self.renderer:
            self.renderer.close()
            self.renderer = None
