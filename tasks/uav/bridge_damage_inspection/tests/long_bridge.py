"""Long-horizon, image-only bridge inspection with delayed evidence scoring.

Only this root-owned service sees the defect inventory and geometric state.
The task deliberately abstracts stabilized flight, not inspection bookkeeping.
"""
import base64
import io
import math
import tempfile
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from pbr_renderer import Renderer
from pbr_scene import prop, surface_materials
from scene_assets import asset_root, geom, materials, robot_meshes, texture_box_faces, vec

SIZE = 384
SPAN = 6.0
DECK = 8.0
ALTITUDE = 13.5
DT = 0.5
ACTION_LIMIT = 420
REPORT_LIMIT = 8
FOV = 60.0
SCENARIOS = ("concrete_viaduct", "steel_through_truss", "mixed_maintenance")


def png(image):
    buffer = io.BytesIO()
    Image.fromarray(image).save(buffer, format="PNG")
    return base64.b64encode(buffer.getvalue()).decode()


@lru_cache(maxsize=1)
def labels_directory():
    directory = Path(tempfile.mkdtemp(prefix="bridge-plaques-"))
    font_paths = ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                  "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf")
    font = next((ImageFont.truetype(p, 76) for p in font_paths if Path(p).exists()),
                ImageFont.load_default(size=76))
    for i in range(1, 21):
        image = Image.new("RGB", (256, 128), (225, 225, 208))
        draw = ImageDraw.Draw(image)
        draw.rectangle((5, 5, 250, 122), outline=(35, 40, 42), width=5)
        draw.text((128, 64), f"B{i:02d}", font=font, fill=(18, 22, 25), anchor="mm")
        image.save(directory / f"bay-{i}.png")
    return directory


def solid(parent, name, pos, size, material="floor_mat", **kwargs):
    return geom(parent, name, pos=pos, size=size, material=material,
                contype="1", conaffinity="1", **kwargs)


class BridgeWorld:
    def __init__(self, seed, render=True):
        self.kind, self.seed = "bridge", int(seed)
        self.render_enabled = render
        rng = np.random.default_rng(seed)
        self.scenario = SCENARIOS[int(rng.integers(3))]
        self.spans = int(rng.choice([12, 16, 20]))
        self.length = self.spans * SPAN
        # Spread defects over the route; the initial image never gives the answer.
        groups = np.array_split(np.arange(1, self.spans + 1), 3)
        self.defects = [(int(rng.choice(group)), int(rng.choice([-1, 1]))) for group in groups]
        candidates = [(i, s) for i in range(1, self.spans + 1) for s in (-1, 1)
                      if (i, s) not in self.defects]
        self.repairs = [candidates[i] for i in rng.choice(len(candidates), 7, replace=False)]
        self.steps = self.frame_id = self.collisions = 0
        self.reports = []
        self.last_frames = {}
        self.done = self.success = False
        self.phase = 0
        self.observed_components = set()
        self.scene_seed = seed
        self._contacts = set()
        self._build()

    @staticmethod
    def component_point(bay, side):
        return np.array([(bay - .5) * SPAN, side * 3.2])

    def _build(self):
        root = ET.Element("mujoco", model="long_bridge_inspection")
        ET.SubElement(root, "option", timestep="0.01", gravity="0 0 -9.81")
        visual = ET.SubElement(root, "visual")
        ET.SubElement(visual, "global", offwidth="1200", offheight="800")
        ET.SubElement(visual, "map", znear=".01", zfar="300")
        ET.SubElement(visual, "quality", shadowsize="2048")
        ET.SubElement(visual, "headlight", active="0")
        assets = materials(root)
        world = ET.SubElement(root, "worldbody")
        ET.SubElement(world, "light", pos=f"{self.length/2} -25 65", dir="-.2 .3 -1",
                      type="directional", castshadow="true", diffuse="1 .96 .88", intensity="24000")
        geom(world, "river", pos=(self.length/2, 0, -.3), size=(self.length, 35, .3),
             material="water_mat")
        for y in (-22, 22):
            geom(world, f"river_bank_{y}", pos=(self.length/2, y, 1),
                 size=(self.length, 5, 1), material="floor_mat")
        for i in range(self.spans + 1):
            x = i * SPAN
            for y in (-2.8, 2.8):
                solid(world, f"pier_{i}_{y}", (x, y, 3.45), (.50, .55, 3.45))
                solid(world, f"pier_foot_{i}_{y}", (x, y, .35), (1.0, 1.05, .35))
            solid(world, f"crosshead_{i}", (x, 0, 7.05), (.55, 4.3, .35))
            if self.scenario != "concrete_viaduct":
                for y in (-4.1, 4.1):
                    solid(world, f"tower_{i}_{y}", (x, y, 10.1), (.16, .16, 2.5), "brushed")
                solid(world, f"portal_{i}", (x, 0, 12.5), (.16, 4.2, .16), "brushed")
        for i in range(1, self.spans + 1):
            x = (i - .5) * SPAN
            solid(world, f"road_{i}", (x, 0, DECK-.16), (2.98, 2.35, .16), "road_mat")
            for offset in (-1.65, 1.65):
                geom(world, f"road_edge_{i}_{offset}", pos=(x, offset, DECK+.006),
                     size=(2.95, .035, .006), rgba=(.8, .8, .72, 1))
            geom(world, f"road_center_{i}", pos=(x, 0, DECK+.008), size=(1.1, .035, .007),
                 rgba=(.9, .75, .22, 1))
            for side in (-1, 1):
                y = side * 3.2
                damaged = (i, side) in self.defects
                # A real opening, not a special-color marker painted on an intact slab.
                if damaged:
                    for j, dx in enumerate((-1.72, 1.72)):
                        solid(world, f"slab_{i}_{side}_{j}", (x+dx, y, DECK-.15),
                              (1.25, .78, .15))
                    for j, dy in enumerate((-.64, .64)):
                        solid(world, f"rim_{i}_{side}_{j}", (x, y+dy, DECK-.15), (.47, .14, .15))
                    for j, dy in enumerate((-.27, .21)):
                        geom(world, f"exposed_rebar_{i}_{side}_{j}", "capsule", size=(.018,),
                             fromto=vec([x-.53, y+dy, DECK-.09, x+.19, y+dy+.07, DECK-.09]),
                             material="rust_mat")
                    # Invisible scoring identifier below the aperture. It is never
                    # rendered in RGB and ray evidence must reach this plane.
                    geom(world, f"defect_{i}_{side}", pos=(x, y, DECK-1.2), size=(.46, .49, .01),
                         rgba=(.04, .1, .13, 1), group="4")
                else:
                    solid(world, f"slab_{i}_{side}", (x, y, DECK-.15), (2.98, .78, .15))
                if (i, side) in self.repairs:
                    geom(world, f"repair_{i}_{side}", pos=(x, y, DECK+.004), size=(.47, .50, .004),
                         rgba=(.12, .14, .15, 1), material="road_mat")
                    for dy in (-.51, .51):
                        geom(world, f"repair_seam_{i}_{side}_{dy}", pos=(x, y+dy, DECK+.01),
                             size=(.49, .009, .006), rgba=(.45, .46, .45, 1))
                for rail_y in (side*2.4, side*4.0):
                    solid(world, f"rail_{i}_{rail_y}", (x, rail_y, DECK+.88), (3, .035, .035), "brushed")
                    for dx in (-2.8, 0, 2.8):
                        solid(world, f"post_{i}_{rail_y}_{dx}", (x+dx, rail_y, DECK+.45),
                              (.035, .035, .45), "brushed")
                if self.scenario != "concrete_viaduct":
                    geom(world, f"truss_{i}_{side}", "capsule", size=(.10,), material="brushed",
                         fromto=vec([x-3, side*4.1, DECK+.2, x+3, side*4.1, 12.5]),
                         contype="1", conaffinity="1")
                    solid(world, f"chord_{i}_{side}", (x, side*4.1, 12.5), (3, .12, .12), "brushed")
                if self.scenario == "mixed_maintenance" and i % 3 == 0:
                    solid(world, f"service_screen_{i}_{side}", (x-1.6, side*2.5, DECK+.7),
                          (.5, .08, .7), "rust_mat")
                texture = f"bay_label_{i}_{side}"
                ET.SubElement(assets, "texture", name=texture, type="2d", file=str(labels_directory()/f"bay-{i}.png"))
                ET.SubElement(assets, "material", name=texture, texture=texture, specular="0")
                geom(world, f"plaque_{i}_{side}", pos=(x-1.4, y, DECK+.04), size=(.72, .36, .025),
                     material=texture, rgba=(1, 1, 1, 1))
        # End abutments and approach roads make the overview an actual long bridge.
        for x in (-3.2, self.length+3.2):
            solid(world, f"abutment_{x}", (x, 0, 4), (3.1, 4.6, 4))
            geom(world, f"approach_{x}", pos=(x, 0, 8.02), size=(3.1, 2.35, .02), material="road_mat")
        agent = ET.SubElement(world, "body", name="agent", pos=f"0 0 {ALTITUDE}")
        ET.SubElement(agent, "joint", name="x", type="slide", axis="1 0 0", damping="5")
        ET.SubElement(agent, "joint", name="y", type="slide", axis="0 1 0", damping="5")
        ET.SubElement(agent, "geom", name="robot", type="box", size=".14 .14 .07", mass="1",
                      rgba="0 0 0 0", group="3")
        robot_meshes(root, agent, "air")
        ET.SubElement(agent, "camera", name="onboard", pos="0 0 -.12", fovy=str(FOV))
        surface_materials(root, asset_root())
        prop(root, asset_root(), "utility_box_01", "abutment_switchgear", (-3, 3.5, DECK), (.85, .55, 1.65))
        for x in (-3.5, self.length+3.5):
            prop(root, asset_root(), "concrete_road_barrier", f"approach_barrier_{x}",
                 (x, -3.7, DECK), (2.7, .42, .85))
        texture_box_faces(root)
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        self.data = mujoco.MjData(self.model)
        self.agent_id = self.model.body("agent").id
        self.data.qpos[:2] = [-2., -5.5]
        mujoco.mj_forward(self.model, self.data)
        self.renderer = Renderer(self.model, SIZE, SIZE) if self.render_enabled else None

    def position(self):
        return self.data.xpos[self.agent_id].copy()

    def images(self):
        option = mujoco.MjvOption()
        option.geomgroup[3:5] = 0
        self.renderer.update_scene(self.data, camera="onboard", scene_option=option)
        return [self.renderer.render().copy()]

    def _visible_defects(self):
        """Require a close, unobstructed aperture in the submitted RGB viewpoint."""
        origin = self.data.cam_xpos[self.model.camera("onboard").id]
        visible = []
        for bay, side in self.defects:
            target = np.r_[self.component_point(bay, side), DECK-.08]
            delta = target-origin
            # Tight framing is a clear-photo requirement, not access to the answer.
            if abs(delta[0]) > 2.1 or abs(delta[1]) > 2.1:
                continue
            direction = delta / np.linalg.norm(delta)
            gid = np.array([-1], dtype=np.int32)
            groups = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)
            distance = mujoco.mj_ray(self.model, self.data, origin, direction, groups, True, self.agent_id, gid)
            if distance < 0 or distance > np.linalg.norm(delta)+.1:
                visible.append((bay, side))
        return visible

    def observe(self):
        self.frame_id += 1
        visible = self._visible_defects()
        self.last_frames[self.frame_id] = {"visible_defects": visible, "position": self.position().tolist()}
        p = self.position()[:2]
        for bay in range(1, self.spans+1):
            for side in (-1, 1):
                if np.max(np.abs(self.component_point(bay, side)-p)) < 2.1:
                    self.observed_components.add((bay, side))
        return {"images": [png(im) for im in self.images()] if self.render_enabled else [],
                "frame_id": self.frame_id, "steps_remaining": max(0, ACTION_LIMIT-self.steps),
                "phase": self.phase, "done": self.done, "reports_submitted": len(self.reports)}

    def act(self, action):
        if self.done or self.steps >= ACTION_LIMIT:
            raise ValueError("movement budget exhausted; submit evidence and finalize")
        if (not isinstance(action, list) or len(action) != 2 or
                any(isinstance(v, bool) or not isinstance(v, (float, int)) or
                    not math.isfinite(v) or abs(v) > 1 for v in action)):
            raise ValueError("action must be two finite normalized velocity commands")
        self.steps += 1
        velocity = np.array(action)*3.0
        for _ in range(50):
            self.data.qfrc_applied[:2] = 35*(velocity-self.data.qvel[:2])
            mujoco.mj_step(self.model, self.data)
        p = self.position()
        if not (-5 <= p[0] <= self.length+5 and abs(p[1]) <= 10):
            self.done = True
        return self.observe()

    def report(self, frame_id, point):
        if self.done or len(self.reports) >= REPORT_LIMIT:
            raise ValueError("reports closed or report budget exhausted")
        if (not isinstance(frame_id, int) or isinstance(frame_id, bool) or not isinstance(point, list)
                or len(point) != 2 or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                       or not math.isfinite(v) for v in point)):
            raise ValueError("report needs a frame integer and finite world XY")
        # No correctness feedback: guesses cannot be used to query the hidden labels.
        self.reports.append({"frame_id": frame_id, "point": point})
        return self.observe()

    def advance(self):
        if self.done:
            raise ValueError("episode already finalized")
        self.done, self.phase = True, 1
        self.success = self._score()["inspection_f1"] == 1.0
        return self.observe()

    def _score(self):
        matched = set()
        false_reports = 0
        for report in self.reports:
            visible = self.last_frames.get(report["frame_id"], {}).get("visible_defects", [])
            match = next((tuple(k) for k in visible if tuple(k) not in matched and
                          np.linalg.norm(self.component_point(*k)-report["point"]) <= .45), None)
            if match is None:
                false_reports += 1
            else:
                matched.add(match)
        tp = len(matched)
        return {"inspection_f1": 2*tp/(len(self.defects)+len(self.reports)) if self.reports else 0.,
                "true_reports": tp, "false_reports": false_reports,
                "missed_defects": len(self.defects)-tp}

    def metrics(self):
        score = self._score()
        finalized = self.phase == 1
        return {**score, "inspection_quality": score["inspection_f1"] if finalized else 0.,
                "success": float(finalized and score["inspection_f1"] == 1.),
                "steps": self.steps, "simulation_seconds": self.steps*DT, "reports": len(self.reports),
                "coverage_fraction": len(self.observed_components)/(2*self.spans),
                "bridge_length_m": self.length, "scenario_id": SCENARIOS.index(self.scenario),
                "collisions": self.collisions, "damage_cost": 0.}

    def close(self):
        if self.renderer:
            self.renderer.close()
