"""Grader-owned xArm/AGiPix physics. No object pose writes after initialization.

Small screws are represented by at-rest couplings. A locked part keeps the exact
relative pose at fastening: capture never teleports it to a nominal target.
"""
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

DT = .01
HOME_Q = np.array([0, -.247, 0, .909, 0, 1.15644, 0])


def text(v):
    return " ".join(f"{float(x):.10g}" for x in np.asarray(v).ravel())


def quat(R):
    q = Rotation.from_matrix(R).as_quat()
    return np.roll(q, 1)


def transform(pos, rot=None):
    T = np.eye(4)
    T[:3, 3] = pos
    if rot is not None:
        T[:3, :3] = rot
    return T


def body_pose(d, bid):
    return transform(d.xpos[bid], d.xmat[bid].reshape(3, 3))


def fixture_targets(graph):
    root = transform(graph["fixture_pos_m"])
    poses = {"base": root}
    for name, p in graph["instances"].items():
        poses[name] = root @ transform(p["target_pos_m"], Rotation.from_euler(
            "xyz", p["target_euler_xyz_deg"], degrees=True).as_matrix())
        if p["kind"] == "motor":
            poses[name + "/rotor"] = poses[name] @ transform([0, 0, .018])
    return poses


def default_layout(graph, kit, seed, jitter=True):
    """Draw the WHOLE layout again on overlap. No per-part fallback collisions."""
    rng = np.random.default_rng(seed)
    large = {"guard": [-.15, -.40], "compute_cover": [-.24, .42], "battery": [-.07, .46]}
    large.update({f"leg_{i+1}": [.08, -.25-.05*i] for i in range(4)})
    small = sorted([n for n in graph["instances"] if n not in large], key=lambda n:
                   -float(np.prod(np.ptp(kit["parts"][graph["instances"][n]["kind"]]["bounds_m"],axis=0)[:2])))
    slots = np.array([[x, y] for y in np.arange(.22, .54, .025) for x in np.arange(-.31, .26, .025)] +
                     [[x, y] for y in np.arange(-.42, -.21, .025) for x in np.arange(.14, .31, .025)])
    slots = slots[np.linalg.norm(slots - [-.4, 0], axis=1) < .69]
    # Parts installed after the guard need an approach outside its eventual
    # overhang. Otherwise a valid early assembly can block a later bench pick.
    late = {"mount_guard"}
    for mate in graph["mates"]:
        if any(r in late for r in mate["requires"]):
            late.add(mate["id"])
    late_parts = {m["part"] for m in graph["mates"] if m["id"] in late and m["part"] != "guard"}
    shadow = np.asarray(kit["parts"]["guard_deck"]["bounds_m"])[:, :2] + np.asarray(graph["fixture_pos_m"][:2])
    shadow += [[-.05, -.05], [.05, .05]]
    for attempt in range(100):
        layout, bounds = {}, []
        for name in list(large) + small:
            inst = graph["instances"][name]
            kind = inst["kind"]
            part = kit["parts"][kind]
            bb = np.array(part.get("rest_bounds_m", part["bounds_m"]))
            scale = 1 if jitter else 0
            yaw = rng.uniform(-.2, .2) * scale
            yaw_R = Rotation.from_euler("z", yaw).as_matrix()
            R = yaw_R @ np.asarray(part.get("rest_rotation", np.eye(3)))
            corners = np.array([[x, y, 0] for x in bb[:, 0] for y in bb[:, 1]]) @ yaw_R.T
            choices = np.array([large[name]]) if name in large else slots[rng.permutation(len(slots))] if jitter else slots
            for xy in choices:
                xy = xy + rng.uniform(-.004, .004, 2) * scale
                low, high = xy + corners[:, :2].min(0), xy + corners[:, :2].max(0)
                if name in late_parts and np.all(high > shadow[0]) and np.all(shadow[1] > low):
                    continue
                if not any(np.all(high + .008 > a) and np.all(b + .008 > low) for a, b in bounds):
                    break
            else:
                break
            bounds.append((low, high))
            # Thin parts have an ordinary physical three-point staging support.
            supported = kind in ("sensor_cheek", "sensor_cap", "prop_cw", "prop_ccw")
            z = (.026 if supported else .001) - bb[0, 2]
            layout[name] = dict(pos=[*xy, z], quat=quat(R).tolist(), supported=supported)
        else:
            return layout
    raise RuntimeError("Cannot draw a non-overlapping bench layout; fix the bench plan")


class Cell:
    def __init__(self, assets, graph_path=None, seed=0, jitter=True, layout=None):
        self.assets = Path(assets)
        self.graph = json.loads(Path(graph_path or Path(__file__).with_name("assembly_graph.json")).read_text())
        setup_rng = np.random.default_rng(seed ^ 0xACA7)
        if jitter:
            self.graph["fixture_pos_m"][:2] = (np.asarray(self.graph["fixture_pos_m"][:2]) + setup_rng.uniform(-.01, .01, 2)).tolist()
        self.kit = json.loads((self.assets / "given/kit/kit.json").read_text())
        self.nominal = fixture_targets(self.graph)
        self.layout = layout or default_layout(self.graph, self.kit, seed, jitter)
        root = ET.parse(self.assets / "given/xarm7/xarm7.xml").getroot()
        root.find("compiler").set("meshdir", str(self.assets / "given/xarm7/assets"))
        root.find("option").set("timestep", ".002")
        root.find("option").set("iterations", "80")
        root.find("option").set("cone", "elliptic")
        root.remove(root.find("keyframe"))
        ET.SubElement(root, "visual")
        ET.SubElement(root.find("visual"), "global", offwidth="1280", offheight="960")
        asset, world, eq = root.find("asset"), root.find("worldbody"), root.find("equality")
        world.find("body").set("pos", "-.4 0 0")
        ET.SubElement(world, "light", pos="-.3 -.3 1.8", dir="0 0 -1", diffuse=".8 .8 .8")
        ET.SubElement(world, "light", pos=".6 .5 1.4", dir="-.2 -.2 -1", diffuse=".5 .5 .5")
        ET.SubElement(world, "geom", name="table", type="box", size=".8 .75 .025", pos="0 0 -.025",
                      rgba=".6 .46 .3 1", friction=".8 .01 .001")
        ET.SubElement(world, "geom", name="fixture", type="box", size=".025 .025 .09", pos=text([*self.graph["fixture_pos_m"][:2], .09]),
                      rgba=".28 .3 .34 1")
        ET.SubElement(world, "camera", name="top", pos="-.04 0 1.65", xyaxes="1 0 0 0 1 0", fovy="55")
        ET.SubElement(world, "camera", name="review", pos="1.05 -1.4 1.1", xyaxes=".8 .6 0 -.3 .4 .866", fovy="50")
        hand = world.find(".//body[@name='xarm_gripper_base_link']")
        ET.SubElement(hand, "camera", name="wrist", pos=".055 0 .04", xyaxes="0 1 0 .96 0 .28", fovy="65")
        ET.SubElement(hand, "site", name="ft_site", pos="0 0 0", size=".002")
        sensor = ET.SubElement(root, "sensor")
        ET.SubElement(sensor, "force", name="wrist_force", site="ft_site")
        ET.SubElement(sensor, "torque", name="wrist_torque", site="ft_site")
        # Physical ruler squares: visible calibration cues, no camera matrix API.
        for i in range(11):
            ET.SubElement(world, "geom", type="box", size=".01 .01 .0003", pos=f"{-.34+i*.02} .62 .0003",
                          rgba=".05 .05 .05 1" if i % 2 else ".95 .95 .95 1", contype="0", conaffinity="0")
        for name, p in self.kit["parts"].items():
            ET.SubElement(asset, "mesh", name="vis_" + name,
                          file=str(self.assets / "given/kit/meshes" / p["mesh"]))
            for i, fn in enumerate(p["collision"]):
                ET.SubElement(asset, "mesh", name=f"col_{name}_{i}", file=str(self.assets / "given/kit/collision" / fn))
        self._body_xml(world, "base", "base", self.graph["fixture_pos_m"], [1, 0, 0, 0], fixed=True)
        for name, inst in self.graph["instances"].items():
            rest = self.layout[name]
            self._body_xml(world, name, inst["kind"], rest["pos"], rest["quat"])
            if rest["supported"]:
                # Physical pins contact actual material, not bounding-box corners.
                import trimesh
                mesh = trimesh.load_mesh(self.assets / "given/kit/meshes" / self.kit["parts"][inst["kind"]]["mesh"])
                v = mesh.vertices
                v = v[np.abs(v[:, 2] - v[:, 2].min()) < .0001]
                center = v[:, :2].mean(0)
                idx = [int(np.argmax((v[:, :2] - center) @ np.array([math.cos(a), math.sin(a)]))) for a in (0, 2.094, 4.189)]
                points = v[idx].copy()
                points[:,:2] = .985*points[:,:2] + .015*center
                R = Rotation.from_quat(np.roll(rest["quat"], -1)).as_matrix()
                for k, point in enumerate(points):
                    p = R @ point + rest["pos"]
                    ET.SubElement(world, "geom", name=f"support_{name}_{k}", type="cylinder",
                                  size=f".002 {p[2]/2}", pos=text([p[0], p[1], p[2]/2]), rgba=".4 .42 .46 1")
        for mate in self.graph["mates"]:
            ET.SubElement(eq, "weld", name=mate["id"], body1="part_" + mate["carrier"],
                          body2="part_" + mate["part"], active="false", solref=".004 1", solimp=".95 .99 .001")
        self.xml = ET.tostring(root, encoding="unicode")
        self.m = mujoco.MjModel.from_xml_string(self.xml)
        if jitter:
            for camera, offset, angular in (("top", .02, 1.5), ("wrist", .002, 1)):
                i = self.m.camera(camera).id
                self.m.cam_pos[i] += setup_rng.uniform(-offset, offset, 3)
                nominal_R = Rotation.from_quat(np.roll(self.m.cam_quat[i], -1))
                delta = Rotation.from_euler("xyz", setup_rng.uniform(-angular, angular, 3), degrees=True)
                self.m.cam_quat[i] = np.roll((nominal_R * delta).as_quat(), 1)
                self.m.cam_fovy[i] *= setup_rng.uniform(.98, 1.02)
        self.d = mujoco.MjData(self.m)
        self.arm_qadr = np.array([self.m.jnt_qposadr[self.m.joint(f"joint{i}").id] for i in range(1, 8)])
        self.arm_dadr = np.array([self.m.jnt_dofadr[self.m.joint(f"joint{i}").id] for i in range(1, 8)])
        self.d.qpos[self.arm_qadr] = HOME_Q
        self.d.ctrl[:7] = HOME_Q
        mujoco.mj_forward(self.m, self.d)
        self.ids = {n: self.m.body("part_" + n).id for n in self.nominal}
        self.coupling = Coupling(self)
        self.violations, self.events, self.frames = [], [], []
        self.recording = False
        self.max_force = 0
        self.max_nonpad_force = 0
        self.nonpad_contacts = {}
        self.pad_contacts = {}
        self.over_force = 0
        self.rng = np.random.default_rng(seed ^ 8171)
        self.ft_bias = self.rng.normal(0, .15, 6)
        self.start_time = time.monotonic()
        self.tick_count = 0
        # Every initialization settle tick is physical; exclude startup transients
        # from safety, but do not permit couplings to activate during settling.
        for _ in range(300):
            mujoco.mj_step(self.m, self.d)
        self.d.time = 0

    def _body_xml(self, world, name, kind, pos, q, fixed=False):
        p = self.kit["parts"][kind]
        b = ET.SubElement(world, "body", name="part_" + name, pos=text(pos), quat=text(q))
        if not fixed:
            ET.SubElement(b, "freejoint", name="free_" + name)
        bb = np.array(p["bounds_m"])
        sz = bb[1] - bb[0]
        inertia = p["mass_kg"] / 12 * np.array([sz[1]**2+sz[2]**2, sz[0]**2+sz[2]**2, sz[0]**2+sz[1]**2])
        if kind.startswith("prop_"):
            # A three-bladed propeller's bounding-box centre is far from its hub.
            # Use the true uniform-material volume distribution, not that box.
            import trimesh
            volume = trimesh.load_mesh(self.assets/"given/kit/meshes"/p["mesh"])
            volume.merge_vertices(digits_vertex=7)
            volume.update_faces(volume.unique_faces())
            volume.update_faces(volume.nondegenerate_faces())
            volume.remove_unreferenced_vertices()
            if not volume.is_volume:
                raise RuntimeError("Propeller mass-property mesh is not a closed volume")
            inertia = volume.moment_inertia * p["mass_kg"] / volume.volume
            ET.SubElement(b,"inertial",mass=str(p["mass_kg"]),pos=text(volume.center_mass),
                          fullinertia=text(inertia[[0,1,2,0,0,1],[0,1,2,1,2,2]]))
        else:
            ET.SubElement(b, "inertial", mass=str(p["mass_kg"]), pos=text(bb.mean(0)), diaginertia=text(np.maximum(inertia, 1e-7)))
        ET.SubElement(b, "geom", name="visual_" + name, type="mesh", mesh="vis_"+kind, rgba=text(p["rgba"]),
                      contype="0", conaffinity="0", group="2")
        for i, _ in enumerate(p["collision"]):
            ET.SubElement(b, "geom", name=f"collision_{name}_{i}", type="mesh", mesh=f"col_{kind}_{i}",
                          rgba=text(p["rgba"]), group="3", friction=".8 .02 .002", solref=".004 1", condim="4")
        if kind == "motor":
            rotor = ET.SubElement(b, "body", name="part_"+name+"/rotor", pos="0 0 .018")
            ET.SubElement(rotor, "joint", name="rotor_"+name, type="hinge", axis="0 0 1", damping=".0002", frictionloss=".001")
            ET.SubElement(rotor, "geom", type="cylinder", size=".0125 .003", pos="0 0 .003", mass=".012",
                          rgba=".78 .78 .8 1")
        return b

    def pose(self, name):
        return body_pose(self.d, self.ids[name])

    def tick(self):
        for _ in range(5):
            mujoco.mj_step(self.m, self.d)
        self.tick_count += 1
        self.coupling.tick()
        self._watch()
        if self.recording and self.tick_count % 4 == 0:
            self.frames.append(self.d.qpos.copy())

    def _watch(self):
        force = np.zeros(6)
        part_forces, robot_forces, self_forces, pad_forces = {}, {}, {}, {}
        for ci, c in enumerate(self.d.contact):
            mujoco.mj_contactForce(self.m, self.d, ci, force)
            f = np.linalg.norm(force[:3])
            bodies = [self.m.body(self.m.geom_bodyid[g]).name for g in (c.geom1, c.geom2)]
            geoms = [self.m.geom(g).name or "" for g in (c.geom1, c.geom2)]
            physical_parts = {n.partition("/")[0] for n in bodies if n.startswith("part_")}
            for n in physical_parts:
                part_forces[n] = part_forces.get(n, 0) + f
            arm = [not n.startswith("part_") and n != "world" for n in bodies]
            mounted_base = "link_base" in bodies and "world" in bodies
            for side, other in ((0, 1), (1, 0)):
                if "finger_pad" in geoms[side] and not arm[other]:
                    surface = bodies[other] if bodies[other] != "world" else "world:" + geoms[other]
                    key = geoms[side] + "|" + surface
                    pad_forces[key] = pad_forces.get(key, 0) + f
            if any(arm) and not all(arm) and not mounted_base and not any("finger_pad" in n for n in geoms):
                for n,is_arm in zip(bodies,arm):
                    if is_arm:
                        robot_forces[n] = robot_forces.get(n,0)+f
            if all(arm):
                a,b = [self.m.body(n).id for n in bodies]
                if self.m.body_parentid[a] != b and self.m.body_parentid[b] != a and a != b:
                    pair = tuple(sorted(bodies))
                    self_forces[pair] = self_forces.get(pair, 0) + f
        for pair, f in self_forces.items():
            self._contact_evidence("self:" + "|".join(pair), f)
            if f > 30:
                self.violations.append(dict(t=float(self.d.time),kind="robot_self_collision",force_N=float(f),bodies=pair))
        for n,f in robot_forces.items():
            self._contact_evidence("external:" + n, f)
            if f > 30:
                self.violations.append(dict(t=float(self.d.time),kind="robot_collision",force_N=float(f),body=n))
        for key, f in pad_forces.items():
            self._record_contact(self.pad_contacts, key, f)
        part_max = max(part_forces.values(),default=0)
        self.max_force = max(self.max_force, part_max)
        self.over_force = self.over_force + 1 if part_max > 150 else 0
        if self.over_force == 20:
            self.violations.append(dict(t=float(self.d.time), kind="sustained_part_force", force_N=float(part_max)))
        if not np.isfinite(self.d.qpos).all():
            raise RuntimeError("Nonfinite physical state")
        for n in self.graph["instances"]:
            j = self.m.joint("free_" + n).id
            a = self.m.jnt_dofadr[j]
            if np.linalg.norm(self.d.qvel[a:a+3]) > 3:
                self.violations.append(dict(t=float(self.d.time), kind="part_speed", part=n))

    def _contact_evidence(self, key, force):
        """Owner-only contact evidence; this never changes controls or scoring."""
        self.max_nonpad_force = max(self.max_nonpad_force, float(force))
        self._record_contact(self.nonpad_contacts, key, force)

    def _record_contact(self, events, key, force):
        if force <= .1:
            return
        event = events.setdefault(key, dict(first_s=float(self.d.time), ticks=0, peak_N=0.))
        event["last_s"] = float(self.d.time)
        event["ticks"] += 1
        event["peak_N"] = max(event["peak_N"], float(force))

    def result(self):
        achieved = [n for n, s in self.coupling.state.items() if s == "LOCKED"]
        score = sum(m["weight"] for m in self.graph["mates"] if m["id"] in achieved) / self.graph["total_weight"]
        return dict(score=score if not self.violations else 0, raw_score=score, achieved=achieved,
                    states=self.coupling.state, safety_ok=not self.violations, violations=self.violations[:50],
                    sim_time_s=float(self.d.time), max_force_N=self.max_force,
                    max_nonpad_robot_force_N=self.max_nonpad_force, nonpad_contact_evidence=self.nonpad_contacts,
                    pad_contact_evidence=self.pad_contacts)


class Coupling:
    def __init__(self, cell):
        self.c = cell
        self.state = {m["id"]: "WAITING" if m["requires"] else "FREE" for m in cell.graph["mates"]}
        self.hold = {m["id"]: 0 for m in cell.graph["mates"]}
        self.assigned = {}
        self.candidate = {}
        self.relative = {m["id"]: np.linalg.inv(cell.nominal[m["carrier"]]) @ cell.nominal[m["part"]]
                         for m in cell.graph["mates"]}

    def target(self, mate):
        return self.c.pose(self.carrier(mate)) @ self.relative[mate["id"]]

    def carrier(self, mate):
        name, _, suffix = mate["carrier"].partition("/")
        name = self.assigned.get("mount_" + name, name)
        return name + ("/" + suffix if suffix else "")

    def tick(self):
        c = self.c
        for m in c.graph["mates"]:
            key = m["id"]
            if self.state[key] in ("LOCKED", "LOCKED_OFF"):
                continue
            if not all(self.state[x] in ("LOCKED", "LOCKED_OFF") for x in m["requires"]):
                continue
            target = self.target(m)
            kind = c.graph["instances"][m["part"]]["kind"]
            candidates = [n for n, p in c.graph["instances"].items()
                          if p["kind"] == kind and n not in self.assigned.values()]
            if not candidates:
                continue
            physical = min(candidates, key=lambda n: np.linalg.norm(c.pose(n)[:3,3]-target[:3,3]))
            if self.candidate.get(key) != physical:
                self.hold[key] = 0
                self.candidate[key] = physical
            actual = c.pose(physical)
            error = np.linalg.norm(actual[:3, 3] - target[:3, 3])
            rot = target[:3, :3].T @ actual[:3, :3]
            if m["axial_symmetry"] == 0:
                angle = math.acos(float(np.clip(rot[2, 2], -1, 1)))
            else:
                angle = Rotation.from_matrix(rot).magnitude()
            j = c.m.joint("free_" + physical).id
            v = c.d.qvel[c.m.jnt_dofadr[j]:c.m.jnt_dofadr[j]+6]
            carrier_v = np.zeros(6)
            mujoco.mj_objectVelocity(c.m, c.d, mujoco.mjtObj.mjOBJ_BODY, c.ids[self.carrier(m)], carrier_v, 0)
            still = np.linalg.norm(v[:3]-carrier_v[3:]) < .005 and np.linalg.norm(v[3:]-carrier_v[:3]) < math.radians(2)
            if error > m["capture_m"] or angle > math.radians(8):
                self.state[key], self.hold[key] = "FREE", 0
            elif error > m["position_tolerance_m"] or not still:
                self.state[key], self.hold[key] = "ENGAGED", 0
            else:
                self.state[key] = "HOLDING"
                self.hold[key] += 1
                if self.hold[key] * DT >= m["hold_s"]:
                    e = c.m.equality(key).id
                    rel = np.linalg.inv(c.pose(self.carrier(m))) @ actual
                    c.m.eq_obj1id[e] = c.ids[self.carrier(m)]
                    c.m.eq_obj2id[e] = c.ids[physical]
                    c.m.eq_data[e, 3:6] = rel[:3, 3]
                    c.m.eq_data[e, 6:10] = quat(rel[:3, :3])
                    c.d.eq_active[e] = 1
                    self.assigned[key] = physical
                    self.state[key] = "LOCKED" if angle <= math.radians(m["angle_tolerance_deg"]) else "LOCKED_OFF"
                    c.events.append(dict(t=float(c.d.time), mate=key, state=self.state[key], error_m=float(error)))
