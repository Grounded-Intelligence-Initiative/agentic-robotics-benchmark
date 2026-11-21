"""Native free-base Go1 and a separate camera-only aerial survey."""
import base64
import io
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import mujoco
import numpy as np
from PIL import Image

from scene_assets import asset_root, geom, materials, robot_meshes, vec
from mobility import BEVEL, DUCT, PROTECTED, SUPPORT, TERRAIN_BIT, add_fittings, add_load


LEGS = ("FR", "FL", "RR", "RL")
JOINTS = tuple(f"{leg}_{part}" for leg in LEGS for part in ("hip", "thigh", "calf"))
WALL_Z, CAMERA_Z, FOV, PIXELS = .32, 1.65, 78, 256
MAX_SCOUT, MAX_TICKS, MAX_HOLD = 80, 48000, 40
DOCK_RADIUS = .17
POST_Z, POST_R = .72, .105
MAST_Z, MAST_FOV, MAST_PITCH, MAX_LOOKS = .20, 70, math.radians(36), 16
LAUNCH_BIT = 1 << 33
ROUTE_BITS = 3 << 14


@dataclass(frozen=True)
class Rack:
    near: float
    far: float
    wide: tuple
    shortcut: tuple | None = None

    def choice(self, y):
        if self.wide[0] <= y <= self.wide[1]:
            return "wide"
        if self.shortcut and self.shortcut[0] <= y <= self.shortcut[1]:
            return "shortcut"
        return "outside"


def layout(seed):
    rng = np.random.default_rng(seed & ~(LAUNCH_BIT | ROUTE_BITS | TERRAIN_BIT))
    # Independent uniform seed bits; the fixed evaluation seeds cover each pair twice.
    sides = tuple(1 if (seed >> bit) & 1 else -1 for bit in (14, 15))
    bound = float(rng.uniform(1.86, 1.98))
    goal = np.array([rng.uniform(6.25, 6.43), rng.uniform(-.20, .20)])
    walls = {
        "back": (-1.14, -1.00, -bound-.14, bound+.14),
        "front": (7.06, 7.20, -bound-.14, bound+.14),
        "south": (-1.00, 7.06, -bound-.14, -bound),
        "north": (-1.00, 7.06, bound, bound+.14),
    }
    racks = []
    for i, nominal in enumerate((1.39, 3.13, 4.87)):
        x = nominal + float(rng.uniform(-.09, .09))
        depth = float(rng.uniform(.23, .32))
        center = (float(rng.uniform(-.12, .12)) if i == 1 else
                  float(sides[i//2]*rng.uniform(1.00, 1.14)))
        half = float(rng.uniform(.47, .55))
        wide = (center-half, center+half)
        shortcut = None
        if i != 1:
            center = float(rng.uniform(-.15, .15))
            half = float(rng.uniform(.080, .108))
            shortcut = (center-half, center+half)
        rack = Rack(x-depth/2, x+depth/2, wide, shortcut)
        gaps = sorted((wide, shortcut)) if shortcut else [wide]
        edges = [-bound, *(edge for gap in gaps for edge in gap), bound]
        for j, (lo, hi) in enumerate(zip(edges[::2], edges[1::2])):
            walls[f"rack{i+1}_{j}"] = (rack.near, rack.far, lo, hi)
        racks.append(rack)
    return rng, racks, goal, walls


def fixtures_and_launch(seed, walls):
    # Independent streams: reflecting the launch changes neither rack, fixtures nor servo gains.
    site = seed & ~(LAUNCH_BIT | ROUTE_BITS | TERRAIN_BIT)
    fixtures = np.random.default_rng([site, 3701])
    start = np.random.default_rng([site, 3702])
    hand = 1 if seed & LAUNCH_BIT else -1
    bank = fixtures.choice((-1, 1))
    rail = sum(walls["north"][2:])/2
    back = sum(walls["back"][:2])/2
    posts = [(back, bank*fixtures.uniform(-1.40, -.93)), (back, bank*fixtures.uniform(.22, .74)),
             (fixtures.uniform(-.08, .33), bank*rail), (fixtures.uniform(1.45, 1.95), bank*rail),
             (fixtures.uniform(.50, 1.00), -bank*rail)]
    launch = [start.uniform(-.06, .10)+hand*.12, hand*start.uniform(.54, .85),
              -hand*math.radians(start.uniform(42., 70.))]
    return posts, launch


def rotation2(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def scene(root, walls, goal, posts, fittings):
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
    add_fittings(root, world, fittings)
    for i, (x, y) in enumerate(posts):
        geom(world, f"bollard{i}_stem", "cylinder", (x, y, POST_Z/2), (.06, POST_Z/2),
             rgba=(.46, .47, .45, 1), contype=1, conaffinity=1)
        geom(world, f"bollard{i}_cap", "sphere", (x, y, POST_Z), (POST_R,),
             rgba=(.19, .84, .065, 1), contype=1, conaffinity=1)
    return world


def image64(rgb):
    buf = io.BytesIO()
    Image.fromarray(rgb).save(buf, format="PNG", compress_level=1)
    return base64.b64encode(buf.getvalue()).decode("ascii")


class Survey:
    def __init__(self, walls, goal, posts, fittings):
        root = ET.fromstring('<mujoco><asset/><worldbody/></mujoco>')
        world = scene(root, walls, goal, posts, fittings)
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
        self.xy = np.clip(self.xy+delta, [-.75, -1.55], [6.80, 1.55])
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
    def __init__(self, rng, walls, goal, racks, posts, launch, fittings, props=None, budget=MAX_TICKS):
        spec = asset_root()/"unitree_go1/go1.xml"
        root = ET.parse(spec).getroot()
        root.find("compiler").set("meshdir", str(spec.parent/"assets"))
        scene(root, walls, goal, posts, fittings)
        trunk = root.find(".//body[@name='trunk']")
        add_load(trunk, props)
        for i in range(8):
            angle = i*math.pi/4
            # The mast looks up at fixtures. Roll/pitch follow the real trunk.
            image_up = (-math.sin(MAST_PITCH)*math.cos(angle), -math.sin(MAST_PITCH)*math.sin(angle),
                        math.cos(MAST_PITCH))
            ET.SubElement(trunk, "camera", name=f"mast{i}", pos=vec((0, 0, MAST_Z)),
                          xyaxes=vec((math.sin(angle), -math.cos(angle), 0, *image_up)), fovy=str(MAST_FOV))
        model = self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        if (tuple(model.actuator(i).name for i in range(model.nu)) != JOINTS
                or model.nq != 19 or model.nv != 18 or model.neq or model.nmocap):
            raise RuntimeError("expected twelve servos and an unstabilized free-base Go1")
        gain = np.asarray(props.servo_gain) if props else rng.uniform(.90, 1.10, size=model.nu)
        model.actuator_gainprm[:, 0] *= gain
        model.actuator_biasprm[:, 1] *= gain
        self.budget = int(budget)
        self.trunk = model.body("trunk").id
        self.floor = model.geom("floor").id
        obstacles = [*walls, *(f"bollard{i}_{part}" for i in range(len(posts)) for part in ("cap", "stem"))]
        if any(f.kind == "duct" for f in fittings):
            obstacles.extend(DUCT)
        self.wall_ids = {model.geom(name).id: name for name in obstacles}
        self.support_ids = {self.floor: "floor"}
        if any(f.kind == "deck" for f in fittings):
            self.support_ids.update({model.geom(name).id: name for name in SUPPORT})
        if props:
            model.actuator_forcerange[:] *= props.torque_scale
            # The walked surface sets the friction of a foot's contact (geom priority), not the foot.
            for gid in self.support_ids:
                model.geom_friction[gid, 0] = props.friction
                model.geom_priority[gid] = 1
        self.protected = {model.geom(name).id: name for name in PROTECTED}
        self.guard_id = model.geom("crown_guard").id
        self.fittings = fittings
        self.fitting_entered = set()
        self.fitting_cleared = set()
        self.deck_feet = set()
        self.min_duct_gap = None
        self.contact_part = None
        self.feet = [model.geom(leg).id for leg in LEGS]
        self.foot_modes = [set() for _ in LEGS]
        self.data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, self.data, model.key("home").id)
        self.data.qpos[:2] = launch[:2]
        yaw = launch[2]
        self.data.qpos[3:7] = [math.cos(yaw/2), 0., 0., math.sin(yaw/2)]
        mujoco.mj_forward(model, self.data)
        mujoco.mj_step(model, self.data, nstep=200)  # settle the native home pose, before mission time
        mujoco.mj_kinematics(model, self.data)
        self.origin = self.xy.copy()
        self.origin_yaw = self.heading
        self.world_to_odom = rotation2(-self.origin_yaw)
        self.renderer = None
        self.looks = 0
        self.goal = goal
        self.ticks = self.dwell = self.tipped = 0
        self.actions = 0
        self.outcome = "running"
        self.max_x = float(self.xy[0])
        self.travel = 0.
        self.racks = racks
        self.choices = [None]*len(racks)
        self.crossed = [False]*len(racks)
        self.lateral_margin = [None]*len(racks)
        self.contact_wall = None
        self.dock_error = float(np.linalg.norm(self.xy-goal))
        self.dock_zone_entered = False
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
        supported = set()
        for a, b in self.data.contact.geom:
            if b in self.support_ids or b in self.wall_ids:
                a, b = b, a
            if a in self.support_ids:
                supported.add(int(b))
                if self.support_ids[a] == "deck_table" and b in self.feet:
                    self.deck_feet.add(int(b))
                if b in self.protected:
                    self.contact_wall, self.contact_part = self.support_ids[a], self.protected[b]
            elif a in self.wall_ids:
                self.contact_wall = self.wall_ids[a]
                self.contact_part = self.model.geom(b).name or self.model.body(self.model.geom_bodyid[b]).name
        return supported

    def record_fittings(self):
        x, y = self.xy
        for f in self.fittings:
            half = f.length/2 + (BEVEL if f.kind == "deck" else 0.)
            if abs(x-f.x) < half and f.lo < y < f.hi:
                self.fitting_entered.add(f.kind)
                if f.kind == "duct" and abs(x-f.x) < f.length/2-.07:
                    it = self.guard_id
                    z = self.data.geom_xpos[it, 2]+abs(self.data.geom_xmat[it, 6:]) @ self.model.geom_size[it]
                    gap = float(f.clearance-z)
                    self.min_duct_gap = gap if self.min_duct_gap is None else min(gap, self.min_duct_gap)
            if f.kind in self.fitting_entered and x > f.x+half+.30:
                self.fitting_cleared.add(f.kind)

    def observe(self):
        relative = self.heading-self.origin_yaw
        return {
            "odom_xy_m": (self.world_to_odom @ (self.xy-self.origin)).round(3).tolist(),
            "odom_yaw_rad": round(math.atan2(math.sin(relative), math.cos(relative)), 4),
            "gravity_body_unit": (-self.rotation[2]).round(5).tolist(),
            "gyro_body_rad_s": self.data.qvel[3:6].round(4).tolist(),
            "joint_rad": self.data.qpos[7:].round(4).tolist(),
            "joint_rad_s": self.data.qvel[6:].round(4).tolist(),
            "foot_contact": [foot in self.contacts for foot in self.feet],
            "ticks_remaining": self.budget-self.ticks,
        }

    def look(self, detent):
        if type(detent) is not int or not 0 <= detent < 8:
            raise ValueError("camera detent must be an integer from 0 to 7")
        if self.looks >= MAX_LOOKS:
            raise ValueError("ground camera exposure budget spent")
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, PIXELS, PIXELS)
        mujoco.mj_camlight(self.model, self.data)
        self.renderer.update_scene(self.data, camera=f"mast{detent}")
        self.looks += 1
        return {"images": [image64(self.renderer.render())]}

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None

    def record_passages(self):
        x, y = self.xy
        for i, rack in enumerate(self.racks):
            if self.choices[i] is None and x >= rack.near-.34:
                self.choices[i] = rack.choice(y)
            if rack.near-.32 <= x <= rack.far+.32:
                # Lateral trunk-center slack, not a claim about the leg envelope.
                lo, hi = rack.wide if self.choices[i] != "shortcut" else rack.shortcut
                margin = float(min(y-lo, hi-y))
                old = self.lateral_margin[i]
                self.lateral_margin[i] = margin if old is None else min(margin, old)
            if x >= rack.far+.34:
                self.crossed[i] = True

    def act(self, action):
        if not isinstance(action, dict) or set(action) != {"target_rad", "ticks"}:
            raise ValueError("expected target_rad (12 angles) and ticks")
        raw, ticks = action["target_rad"], action["ticks"]
        if type(ticks) is not int or not 1 <= ticks <= min(MAX_HOLD, self.budget-self.ticks):
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
            self.record_passages()
            self.record_fittings()
            self.contacts = self.ground_contacts()
            for modes, foot in zip(self.foot_modes, self.feet):
                modes.add(foot in self.contacts)
            upright = self.rotation[2, 2] > .80 and self.data.xpos[self.trunk, 2] > .16
            quiet = np.linalg.norm(self.data.qvel[:3]) < .09 and np.linalg.norm(self.data.qvel[3:6]) < .35
            self.dock_error = float(np.linalg.norm(self.xy-self.goal))
            self.dock_zone_entered |= self.dock_error <= DOCK_RADIUS
            parked = self.dock_error <= DOCK_RADIUS and abs(self.heading) <= math.radians(16)
            self.dwell = self.dwell+1 if upright and quiet and parked else 0
            self.tipped = 0 if upright else self.tipped+1
            dt = self.model.opt.timestep
            if self.contact_wall:
                self.outcome = "contact"
            elif self.tipped*dt >= .20:
                self.outcome = "fell"
            elif self.dwell*dt >= .45:
                self.outcome = "docked"
            elif self.ticks >= self.budget:
                self.outcome = "budget"
            if self.outcome != "running":
                break
