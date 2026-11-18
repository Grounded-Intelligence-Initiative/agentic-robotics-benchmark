"""Native free-base Go1 and a separate camera-only aerial survey."""
import base64
import io
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image

from scene_assets import asset_root, geom, materials, robot_meshes, vec


LEGS = ("FR", "FL", "RR", "RL")
JOINTS = tuple(f"{leg}_{part}" for leg in LEGS for part in ("hip", "thigh", "calf"))
WALL_Z, CAMERA_Z, FOV, PIXELS = .32, 1.65, 78, 256
MAX_SCOUT, MAX_TICKS, MAX_HOLD = 64, 28000, 40
DOCK_RADIUS = .17


def layout(seed):
    rng = np.random.default_rng(seed)
    side = int(rng.choice((-1, 1)))  # open side; the other is closed by a real gate
    half = float(rng.uniform(.40, .52))
    end = float(rng.uniform(2.38, 2.60))
    gate = float(rng.uniform(1.85, 2.22))
    bound = float(rng.uniform(1.60, 1.73))
    goal = np.array([rng.uniform(3.08, 3.25), rng.uniform(-.20, .20)])
    walls = {
        "island": (.66, end, -half, half),
        "gate": (gate-.075, gate+.075, -bound, -half) if side == 1
                else (gate-.075, gate+.075, half, bound),
        "back": (-.69, -.55, -bound-.14, bound+.14),
        "front": (3.84, 3.98, -bound-.14, bound+.14),
        "south": (-.55, 3.84, -bound-.14, -bound),
        "north": (-.55, 3.84, bound, bound+.14),
    }
    return rng, side, goal, walls


def scene(root, walls, goal):
    world = root.find("worldbody")
    materials(root)
    ET.SubElement(world, "geom", name="floor", type="plane", size="6 5 .1",
                  material="floor_mat", friction="1 .02 .001")
    ET.SubElement(world, "light", name="worklight", directional="true",
                  pos="0 0 5", dir="0 0 -1", diffuse=".60 .60 .60", castshadow="false")
    for name, (x0, x1, y0, y1) in walls.items():
        size = ((x1-x0)/2, (y1-y0)/2, WALL_Z/2)
        pos = ((x0+x1)/2, (y0+y1)/2, WALL_Z/2)
        geom(world, name, pos=pos, size=size, material="rust_mat", contype=1, conaffinity=1)
        # Painted caps mark the exact footprints; no geometry or depth is sent separately.
        geom(world, name+"_paint", pos=(*pos[:2], WALL_Z-.001),
             size=(*size[:2], .0015), rgba=(.90, .18, .045, 1))
    geom(world, "dock", pos=(*goal, .002), size=(.29, .28, .002), rgba=(.035, .28, .89, 1))
    geom(world, "dock_nose", pos=(goal[0]+.261, goal[1], .005), size=(.026, .26, .001),
         rgba=(.88, .81, .09, 1))
    geom(world, "launch", pos=(0, 0, .002), size=(.25, .20, .002), rgba=(.07, .65, .18, 1))
    return world


def image64(rgb):
    buf = io.BytesIO()
    Image.fromarray(rgb).save(buf, format="PNG", compress_level=1)
    return base64.b64encode(buf.getvalue()).decode("ascii")


class Survey:
    def __init__(self, walls, goal):
        root = ET.fromstring('<mujoco><asset/><worldbody/></mujoco>')
        world = scene(root, walls, goal)
        uav = ET.SubElement(world, "body", name="uav", mocap="true",
                            pos=vec((0, 0, CAMERA_Z+.12)))
        robot_meshes(root, uav, "drone")
        ET.SubElement(uav, "camera", name="down", pos="0 0 -.12", xyaxes="1 0 0 0 1 0",
                      fovy=str(FOV))
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        self.data = mujoco.MjData(self.model)
        self.renderer = None
        self.cached = None
        self.xy = np.zeros(2)

    def move(self, delta):
        self.xy = np.clip(self.xy+delta, [-.35, -1.40], [3.60, 1.40])
        self.cached = None

    def observe(self):
        if self.cached is None:
            if self.renderer is None:
                self.renderer = mujoco.Renderer(self.model, PIXELS, PIXELS)
            self.data.mocap_pos[0, :2] = self.xy
            mujoco.mj_forward(self.model, self.data)
            self.renderer.update_scene(self.data, camera="down")
            self.cached = image64(self.renderer.render())
        return {"tracking_xy_m": self.xy.tolist(), "images": [self.cached]}

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None


class Go1:
    def __init__(self, rng, walls, goal):
        spec = asset_root()/"unitree_go1/go1.xml"
        root = ET.parse(spec).getroot()
        root.find("compiler").set("meshdir", str(spec.parent/"assets"))
        scene(root, walls, goal)
        model = self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        if (tuple(model.actuator(i).name for i in range(model.nu)) != JOINTS
                or model.nq != 19 or model.nv != 18 or model.neq or model.nmocap):
            raise RuntimeError("expected twelve servos and an unstabilized free-base Go1")
        gain = rng.uniform(.90, 1.10, size=model.nu)
        model.actuator_gainprm[:, 0] *= gain
        model.actuator_biasprm[:, 1] *= gain
        self.trunk = model.body("trunk").id
        self.floor = model.geom("floor").id
        self.wall_ids = {model.geom(name).id for name in walls}
        self.feet = [model.geom(leg).id for leg in LEGS]
        self.foot_modes = [set() for _ in LEGS]
        self.data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, self.data, model.key("home").id)
        yaw = float(rng.uniform(-.026, .026))
        self.data.qpos[3:7] = [math.cos(yaw/2), 0., 0., math.sin(yaw/2)]
        mujoco.mj_forward(model, self.data)
        mujoco.mj_step(model, self.data, nstep=200)  # settle the native home pose, before mission time
        mujoco.mj_kinematics(model, self.data)
        self.goal = goal
        self.ticks = self.dwell = self.tipped = 0
        self.actions = 0
        self.outcome = "running"
        self.max_x = 0.
        self.travel = 0.
        self.corridor = 0
        self.dock_error = float(np.linalg.norm(self.xy-goal))
        self.contacts = self.ground_contacts()

    @property
    def xy(self):
        return self.data.xpos[self.trunk, :2]

    @property
    def rotation(self):
        return self.data.xmat[self.trunk].reshape(3, 3)

    @property
    def heading(self):
        return math.atan2(self.rotation[1, 0], self.rotation[0, 0])

    def ground_contacts(self):
        floor = self.floor
        return {int(b if a == floor else a) for a, b in self.data.contact.geom if a == floor or b == floor}

    def observe(self):
        return {
            "tracking_xy_m": self.xy.round(3).tolist(),
            "heading_rad": round(self.heading, 4),
            "gravity_body_unit": (-self.rotation[2]).round(5).tolist(),
            "gyro_body_rad_s": self.data.qvel[3:6].round(4).tolist(),
            "joint_rad": self.data.qpos[7:].round(4).tolist(),
            "joint_rad_s": self.data.qvel[6:].round(4).tolist(),
            "foot_contact": [foot in self.contacts for foot in self.feet],
            "ticks_remaining": MAX_TICKS-self.ticks,
        }

    def act(self, action):
        if not isinstance(action, dict) or set(action) != {"target_rad", "ticks"}:
            raise ValueError("expected target_rad (12 angles) and ticks")
        raw, ticks = action["target_rad"], action["ticks"]
        if type(ticks) is not int or not 1 <= ticks <= min(MAX_HOLD, MAX_TICKS-self.ticks):
            raise ValueError("invalid tick count")
        if not isinstance(raw, list) or len(raw) != 12 or any(type(v) not in (int, float) for v in raw):
            raise ValueError("expected twelve numeric angles")
        target = np.asarray(raw, dtype=float)
        lo, hi = self.model.actuator_ctrlrange.T
        if not np.isfinite(target).all() or (target < lo).any() or (target > hi).any():
            raise ValueError("target outside native joint limits")
        self.actions += 1
        self.data.ctrl[:] = target
        for _ in range(ticks):
            last = self.xy.copy()
            mujoco.mj_step(self.model, self.data)
            mujoco.mj_kinematics(self.model, self.data)
            self.ticks += 1
            self.travel += float(np.linalg.norm(self.xy-last))
            self.max_x = max(self.max_x, float(self.xy[0]))
            if not self.corridor and self.xy[0] >= 1.1:
                self.corridor = 1 if self.xy[1] > 0 else -1
            self.contacts = self.ground_contacts()
            for modes, foot in zip(self.foot_modes, self.feet):
                modes.add(foot in self.contacts)
            upright = self.rotation[2, 2] > .80 and self.data.xpos[self.trunk, 2] > .16
            quiet = np.linalg.norm(self.data.qvel[:3]) < .09 and np.linalg.norm(self.data.qvel[3:6]) < .35
            self.dock_error = float(np.linalg.norm(self.xy-self.goal))
            parked = self.dock_error <= DOCK_RADIUS and abs(self.heading) <= math.radians(16)
            self.dwell = self.dwell+1 if upright and quiet and parked else 0
            self.tipped = 0 if upright else self.tipped+1
            dt = self.model.opt.timestep
            if any(int(a) in self.wall_ids or int(b) in self.wall_ids for a, b in self.data.contact.geom):
                self.outcome = "contact"
            elif self.tipped*dt >= .20:
                self.outcome = "fell"
            elif self.dwell*dt >= .45:
                self.outcome = "docked"
            elif self.ticks == MAX_TICKS:
                self.outcome = "budget"
            if self.outcome != "running":
                break
