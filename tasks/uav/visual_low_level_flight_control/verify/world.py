"""Private Crazyflie scene, dynamics and authoritative success conditions."""
import base64
import io
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
os.environ.setdefault("LP_NUM_THREADS", "2")
import mujoco
import numpy as np
from PIL import Image

SIZE = 512
DT = .2
LIMITS = {"flight": 300}
MASS = .027
THRUST_MAX = .35
GATE_HALF = .45
GATE_COLORS = ((1., .72, .04, 1.), (.04, .78, .18, 1.), (.85, .04, .85, 1.))


def png(image):
    buffer = io.BytesIO()
    Image.fromarray(image).save(buffer, format="PNG", compress_level=1)
    return base64.b64encode(buffer.getvalue()).decode()


def numbers(values):
    return " ".join(f"{float(v):.8g}" for v in values)


def asset_root():
    path = Path(os.environ.get("VISUAL_ROBOTICS_ASSETS", "/opt/visual_robotics/assets"))
    if not (path/"bitcraze_crazyflie_2/cf2.xml").is_file():
        raise FileNotFoundError("Fetch the pinned image/visual_assets.json before running")
    return path


def crossed_gate(previous, current, center):
    if not previous[0] < center[0] <= current[0]:
        return False
    fraction = (center[0]-previous[0])/(current[0]-previous[0])
    crossing = previous + fraction*(current-previous)
    # Clearance is for the whole nominal rotor span, not just its center.
    return bool(np.all(np.abs(crossing[1:]-center[1:]) < GATE_HALF-.09))


def scene(gates, goal):
    source = asset_root()/"bitcraze_crazyflie_2/cf2.xml"
    root = ET.parse(source).getroot()
    root.set("model", "crazyflie_visual_gate_flight")
    root.find("compiler").set("meshdir", str(source.parent/"assets"))
    root.find("option").set("timestep", ".002")
    root.find("option").set("gravity", "0 0 -9.81")
    # The benchmark supplies an attitude servo, not a motor/firmware replica.
    for tag in ("actuator", "sensor", "keyframe"):
        element = root.find(tag)
        if element is not None:
            root.remove(element)
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1600", offheight="1000")
    ET.SubElement(visual, "quality", shadowsize="2048")
    ET.SubElement(visual, "headlight", ambient=".5 .5 .5", diffuse=".3 .3 .3")
    ET.SubElement(root.find("asset"), "material", name="floor_mat", rgba=".52 .55 .57 1", specular=".15")
    world = root.find("worldbody")
    body = world.find("body")
    body.set("name", "agent")
    body.set("pos", "0 0 0")
    for camera in list(body.findall("camera")):
        body.remove(camera)
    ET.SubElement(body, "geom", name="tracking_marker", type="ellipsoid",
                  pos="0 0 .017", size=".023 .017 .008", rgba="1 .015 .02 1",
                  contype="0", conaffinity="0", mass="0", group="2")
    ET.SubElement(world, "light", pos="-3 -4 7", dir=".2 .4 -1", diffuse=".65 .65 .65")
    ET.SubElement(world, "light", pos="4 3 6", dir="-.3 -.2 -1", diffuse=".3 .3 .3", castshadow="false")
    ET.SubElement(world, "camera", name="top", pos="0 0 12", orthographic="true", fovy="8")
    ET.SubElement(world, "camera", name="side", pos="0 -12 2", xyaxes="1 0 0 0 0 1",
                  orthographic="true", fovy="8")
    ET.SubElement(world, "geom", name="floor", type="plane", size="8 8 .1", material="floor_mat")

    def box(name, pos, size, color, collision=True):
        ET.SubElement(world, "geom", name=name, type="box", pos=numbers(pos), size=numbers(size),
                      rgba=numbers(color), contype=str(int(collision)), conaffinity=str(int(collision)))

    for i, (x, y, z) in enumerate(gates):
        color = GATE_COLORS[i]
        for side in (-1, 1):
            box(f"gate_{i}_post_{side}", (x, y+side*(GATE_HALF+.03), z), (.035, .03, GATE_HALF+.06), color)
            box(f"gate_{i}_bar_{side}", (x, y, z+side*(GATE_HALF+.03)), (.035, GATE_HALF, .03), color)
            box(f"gate_{i}_leg_{side}", (x, y+side*(GATE_HALF+.03), (z-GATE_HALF)/2),
                (.022, .022, (z-GATE_HALF)/2), (.17, .2, .22, 1))
            box(f"gate_{i}_foot_{side}", (x, y+side*(GATE_HALF+.03), .025), (.18, .10, .025), (.17, .2, .22, 1))
    ET.SubElement(world, "geom", name="goal", type="cylinder", pos=numbers([*goal[:2], .008]),
                  size=".22 .008", rgba=".025 .10 1 1", contype="0", conaffinity="0")
    for x in (-4.2, 4.2):
        box(f"hall_side{x}", (x, .5, 1.7), (.10, 4, 1.7), (.42, .46, .49, 1))
    box("hall_back", (0, 4.3, 1.7), (4.3, .10, 1.7), (.37, .41, .43, 1))
    for i, x in enumerate((-3, -1.5, 0, 1.5, 3)):
        box(f"window{i}", (x, 4.18, 2.25), (.48, .015, .55), (.16, .23, .26, 1), False)
    return ET.tostring(root, encoding="unicode")


class World:
    def __init__(self, kind, seed, render=True):
        if kind != "flight":
            raise ValueError("this task only implements flight")
        self.kind, self.seed = kind, int(seed)
        rng = np.random.default_rng(self.seed)
        self.gates = np.column_stack((np.array([-1.45, 0., 1.45])+rng.uniform(-.1, .1, 3),
                                      rng.uniform(-.55, .55, 3), rng.uniform(1.05, 1.6, 3)))
        self.start = np.array([rng.uniform(-2.95, -2.6), rng.uniform(-.65, .65), .055])
        self.goal = np.array([rng.uniform(2.5, 2.9), rng.uniform(-.7, .7), 1.3])
        self.steps = self.collisions = self.damage = self.phase = self.frame_id = 0
        self.passed_gates = self.hover_ticks = 0
        self.done = self.success = self.airborne = False
        self.failure_reason = None
        self._contacts = set()
        self.model = mujoco.MjModel.from_xml_string(scene(self.gates, self.goal))
        self.data = mujoco.MjData(self.model)
        self.agent_id = self.model.body("agent").id
        self.data.qpos[:3] = self.start
        mujoco.mj_forward(self.model, self.data)
        self.renderer = mujoco.Renderer(self.model, SIZE, SIZE) if render else None

    def position(self):
        return self.data.xpos[self.agent_id].copy()

    def images(self):
        option = mujoco.MjvOption()
        option.geomgroup[3] = 0
        result = []
        for camera in ("top", "side"):
            self.renderer.update_scene(self.data, camera=camera, scene_option=option)
            self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            result.append(self.renderer.render().copy())
        return result

    def observe(self):
        self.frame_id += 1
        return {"images": [png(im) for im in self.images()], "frame_id": self.frame_id,
                "phase": 0, "done": bool(self.done), "completed_stages": [],
                "steps_remaining": max(0, LIMITS["flight"]-self.steps), "damage_remaining": 0}

    def act(self, action):
        if self.done:
            raise ValueError("episode finished")
        if (not isinstance(action, list) or len(action) != 4 or
                any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                    not math.isfinite(v) or abs(v) > 1 for v in action)):
            raise ValueError("four finite numeric actions in [-1, 1] required")
        self.steps += 1
        desired = np.array(action[1:])*[.3, .3, .4]
        for _ in range(round(DT/self.model.opt.timestep)):
            previous = self.position()
            rotation = self.data.xmat[self.agent_id].reshape(3, 3)
            angles = np.array([math.atan2(rotation[2, 1], rotation[2, 2]),
                               math.asin(float(np.clip(-rotation[2, 0], -1, 1))),
                               math.atan2(rotation[1, 0], rotation[0, 0])])
            error = (desired-angles+np.pi) % (2*np.pi)-np.pi
            acceleration = 100*error-20*self.data.qvel[3:6]
            torque = np.clip(self.model.body_inertia[self.agent_id]*acceleration, -.002, .002)
            self.data.xfrc_applied[:] = 0
            self.data.xfrc_applied[self.agent_id, :3] = rotation[:, 2]*(action[0]+1)*THRUST_MAX/2
            self.data.xfrc_applied[self.agent_id, 3:] = rotation@torque
            mujoco.mj_step(self.model, self.data)
            p = self.position()
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                raise RuntimeError("nonfinite simulator state")
            self.airborne |= p[2] > .2
            contacts = set()
            for contact in self.data.contact:
                ids = (int(contact.geom1), int(contact.geom2))
                if not any(self.model.geom_bodyid[g] == self.agent_id for g in ids):
                    continue
                if not self.airborne and any(self.model.geom(g).name == "floor" for g in ids):
                    continue
                contacts.add(tuple(sorted(ids)))
            self.collisions += len(contacts-self._contacts)
            self._contacts = contacts
            if self.passed_gates < len(self.gates) and crossed_gate(previous, p, self.gates[self.passed_gates]):
                self.passed_gates += 1
            stable = (self.passed_gates == len(self.gates) and np.linalg.norm(p-self.goal) < .22
                      and np.linalg.norm(self.data.qvel[:3]) < .25
                      and np.linalg.norm(self.data.qvel[3:6]) < .5 and not self.collisions)
            self.hover_ticks = self.hover_ticks+1 if stable else 0
            self.success = self.hover_ticks*self.model.opt.timestep >= 2.
            outside = np.any(np.abs(p[:2]) > 3.8) or p[2] > 3.5 or p[2] < -.1
            if self.collisions or outside or self.success:
                self.done = True
                self.failure_reason = "collision" if self.collisions else "out_of_bounds" if outside else None
                break
        if self.steps >= LIMITS["flight"]:
            self.done = True
            if not self.success and self.failure_reason is None:
                self.failure_reason = "budget_exhausted"
        return self.observe() if self.renderer is not None else {}

    def advance(self):
        raise ValueError("no phase transition or reset in this task")

    def report(self, frame_id, point):
        raise ValueError("self-reported success is not accepted")

    def metrics(self):
        return {"success": float(self.success), "steps": self.steps, "collisions": self.collisions,
                "damage_cost": 0, "gates_passed": self.passed_gates,
                "hover_seconds": self.hover_ticks*self.model.opt.timestep,
                "simulation_seconds": float(self.data.time), "failure_reason": self.failure_reason}

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
