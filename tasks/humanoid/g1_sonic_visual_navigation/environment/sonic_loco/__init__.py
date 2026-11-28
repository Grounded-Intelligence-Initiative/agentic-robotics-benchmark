"""Native 50 Hz token control of a free-base G1/Dex3 on a flat floor."""
from copy import copy
import hashlib
import math
from pathlib import Path

import mujoco
import numpy as np

from sonic_sim.decoder import DEFAULT_Q, Decoder, DecoderHistory

from . import racks
from .rack_score import RobotProfile, RackPassage
from .obstacles import LANE_HALF_WIDTH, LATERAL, PANEL_HEIGHT, PANEL_LENGTH, PLATE_OFFSET_X, cycle, heading_axes
from .model import BODY_JOINTS, FINGER_JOINTS, PHYSICS_DT, RIGHT_CLOSURE, SIDES, SUBSTEPS, floor_contacts, make_model


def _integer(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} must be an integer in [{low}, {high}]")
    return value


def _draw(seed, field, low, high):
    data = hashlib.blake2b(field.encode("ascii"), key=seed.to_bytes(8, "little"),
                           person=b"g1loco.reset.v1", digest_size=8).digest()
    return low + (high - low) * ((int.from_bytes(data, "little") >> 11) * 2.0**-53)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _angle(matrix):
    return math.atan2(matrix[3], matrix[0])


class LocomotionSession:
    """A CPU ONNX decoder, 29 body motors, 14 finger motors and unconstrained native dynamics."""

    def __init__(self, assets_root, seed, horizon=1000):
        self.horizon = _integer(horizon, "horizon", 1, 100_000)
        root = Path(assets_root).resolve()
        self._decoder = Decoder(root / "policy/model_decoder.onnx")
        self.model = model = make_model(root / "menagerie_g1")
        self._pelvis = model.body("pelvis").id
        self._torso = model.body("torso_link").id
        self._feet = [model.site(s + "_foot").id for s in SIDES]
        self._floor = model.geom("floor").id
        names = ("fork_island", "fork_right_jamb", "fork_left_jamb", "shutter_right", "shutter_left",
                 "gate_1_minus_y_cabinet", "gate_1_plus_y_cabinet")
        self.gate_geom_ids = {model.geom(name).id:(int(name.startswith("gate_1_")),name) for name in names}
        self._gate_mocaps = {name:model.body_mocapid[model.body(name+"_anchor").id] for name in names}
        self._rack_mocaps = {}
        for gid in range(model.ngeom):
            name = model.geom(gid).name
            if name and name.startswith("rack_"):
                self._rack_mocaps[name] = model.body_mocapid[model.body(name+"_anchor").id]
                if model.geom_contype[gid]:
                    self.gate_geom_ids[gid]=(2,name)
        self._leaf_bodies = [model.body("shutter_"+side).id for side,_ in LATERAL]
        self._rail_bodies = [model.body("shutter_"+side+"_anchor").id for side,_ in LATERAL]
        joints = [model.joint("shutter_"+side).id for side,_ in LATERAL]
        self._leaf_q = model.jnt_qposadr[joints].copy()
        self._leaf_v = model.jnt_dofadr[joints].copy()
        self._leaf_u = [model.actuator("shutter_"+side).id for side,_ in LATERAL]
        self._stripe_mocaps = [model.body_mocapid[model.body("lane_"+s).id] for s in ("minus_y","plus_y")]
        # The original robot is a single body tree rooted at its free pelvis.
        root = model.body_rootid[self._pelvis]
        self._robot_physical_geoms = frozenset(g for g in range(model.ngeom)
            if model.body_rootid[model.geom_bodyid[g]] == root
            and (model.geom_contype[g] or model.geom_conaffinity[g]))
        self.profile = RobotProfile(model,self._robot_physical_geoms)
        joints = [model.joint(name).id for name in BODY_JOINTS]
        self._qadr = model.jnt_qposadr[joints].copy()
        self._vadr = model.jnt_dofadr[joints].copy()
        self._motors = [model.actuator(name).id for name in BODY_JOINTS]
        self._fingers = [model.actuator(name).id for name in FINGER_JOINTS]
        self._sole_side = {geom: side for side, name in enumerate(SIDES) for geom in range(model.ngeom)
                           if model.geom_bodyid[geom] == model.body(name + "_ankle_roll_link").id
                           and model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_SPHERE}
        if len(self._sole_side) != 8 or model.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError("The G1 model must have eight sole spheres and a floating base")
        self._renderer = None
        self._render_size = None
        self._ego_renderer = None
        self._probe_renderer = None
        self._ego_camera = model.camera("egocentric").id
        self._visual_meshes = np.flatnonzero(model.geom_group == 2)
        self.beacons = ()
        self.reset(seed)

    def reset(self, seed):
        """Reset posture, contacts and history. World XY and yaw use independent keyed draws."""
        _integer(seed, "seed", 0, 2**64-1)
        self.beacons = ()
        self.data = data = mujoco.MjData(self.model)
        self.tick = 0
        self.layout = None
        self.rack_monitor = None
        self.obstacle_contact_substeps = 0
        self.first_obstacle_contact = None
        for mid in (*self._gate_mocaps.values(), *self._rack_mocaps.values(), *self._stripe_mocaps):
            data.mocap_pos[mid] = (0., 0., -4.)
        self._history = DecoderHistory()
        self.last_target = None
        self.last_substeps = ()
        self._sample = None
        self._fall_time = None
        self._distance = self._accel_square = self._target_square = 0.
        self._foot_paths = np.zeros(2)
        data.qpos[self._qadr] = DEFAULT_Q
        data.ctrl[self._motors] = DEFAULT_Q
        data.qpos[:2] = [_draw(seed, axis, -.15, .15) for axis in ("world-x", "world-y")]
        yaw = _draw(seed, "world-yaw", -math.pi, math.pi)
        data.qpos[3:7] = (math.cos(yaw / 2), 0., 0., math.sin(yaw / 2))
        mujoco.mj_forward(self.model, data)
        gap = min(data.geom_xpos[g, 2] - self.model.geom_size[g, 0] for g in self._sole_side)
        data.qpos[2] += .0002 - gap
        mujoco.mj_forward(self.model, data)
        self._measure(data.time, data.qvel[:3])
        return self.state

    def _place_furniture(self, data, layout):
        fork = layout.fork
        racks.place(data,layout.racks,self.layout_origin_xy,self.layout_axes,self.layout_yaw,self._rack_mocaps,self.station_m)
        quat = (math.cos(self.layout_yaw/2), 0., 0., math.sin(self.layout_yaw/2))
        def place(mid, x, y, z):
            data.mocap_pos[mid] = (*((self.layout_origin_xy + self.layout_axes @ [x,y])), z)
            data.mocap_quat[mid] = quat
        def cabinet(name, x, y, color):
            place(self._gate_mocaps[name], x, y, PANEL_HEIGHT/2)
            if data is self.data:
                self.model.geom_rgba[self.model.geom(name).id,:3] = color
        cabinet("fork_island", fork.forward_m, fork.island_center_m, fork.rgb)
        for side, sign in LATERAL:
            lo,hi = fork.corridor(sign)
            outer = hi if sign==1 else lo
            cabinet("fork_"+side+"_jamb", fork.forward_m, outer+sign*PANEL_LENGTH/2, fork.rgb)
            place(self._gate_mocaps["shutter_"+side], fork.forward_m+PLATE_OFFSET_X, (hi+lo)/2, .38)
        for sign, name in [(-1,"gate_1_minus_y_cabinet"), (1,"gate_1_plus_y_cabinet")]:
            if layout.tail is None:
                data.mocap_pos[self._gate_mocaps[name]] = (0., 0., -4.)
            else:
                lo,hi = layout.tail.edges
                outer = hi if sign==1 else lo
                cabinet(name,layout.tail.forward_m,outer+sign*PANEL_LENGTH/2,layout.tail.rgb)
        for mid, sign in zip(self._stripe_mocaps, (-1,1)):
            place(mid, 4., sign*LANE_HALF_WIDTH, .0015)

    def install_layout(self, layout, *, station_m=4.):
        if self.tick:
            raise ValueError("furniture must be configured at reset")
        self.station_m = station_m
        self.layout = layout
        self.layout_origin_xy = np.asarray(self._sample["base_xy"]).copy()
        self.layout_yaw = self._sample["heading_rad"]
        self.layout_axes = heading_axes(self.layout_yaw)
        self._place_furniture(self.data,layout)
        self._strokes = tuple(layout.stroke(sign) for _,sign in LATERAL)
        for i,phase in enumerate(layout.phases):
            frac,rate = cycle(phase)
            self.data.qpos[self._leaf_q[i]] = self.data.ctrl[self._leaf_u[i]] = frac*self._strokes[i]
            self.data.qvel[self._leaf_v[i]] = rate*self._strokes[i]
        mujoco.mj_forward(self.model, self.data)

    def actual_slides(self):
        """Solved positions at the transform timestamp, including intermediate 2 ms samples."""
        if self.layout is None:
            return (0.,0.)
        d = self.data
        return tuple(float(sign*((d.xpos[leaf,:2]-d.xpos[rail,:2])@self.layout_axes[:,1]))
                     for (_,sign),leaf,rail in zip(LATERAL,self._leaf_bodies,self._rail_bodies))

    def _obstacle_contact(self, time_s):
        for contact in self.data.contact:
            a, b = int(contact.geom1), int(contact.geom2)
            if a in self.gate_geom_ids:
                cabinet, robot = a, b
            elif b in self.gate_geom_ids:
                cabinet, robot = b, a
            else:
                continue
            if robot not in self._robot_physical_geoms or contact.dist > 0. or contact.efc_address < 0:
                continue
            if self.first_obstacle_contact is None:
                gate, piece = self.gate_geom_ids[cabinet]
                body = int(self.model.geom_bodyid[robot])
                self.first_obstacle_contact = dict(time_s=float(time_s), native_substep=int(round(time_s/PHYSICS_DT)),
                    tick=int(math.ceil(time_s/(PHYSICS_DT*SUBSTEPS)-1e-8)), gate_index=gate,
                    panel=piece, obstacle_geom=self.model.geom(cabinet).name, robot_body=self.model.body(body).name,
                    robot_geom_id=robot, robot_mesh=(self.model.mesh(self.model.geom_dataid[robot]).name
                        if self.model.geom_type[robot]==mujoco.mjtGeom.mjGEOM_MESH else None), distance_m=float(contact.dist), world_xyz_m=contact.pos.tolist())
            self.obstacle_contact_substeps += 1
            return True
        return False

    def _measure(self, time_s, velocity):
        d = self.data
        pos = d.xpos[self._pelvis]
        torso = d.xpos[self._torso]
        rotation = d.xmat[self._pelvis]
        torso_rotation = d.xmat[self._torso]
        feet = d.site_xpos[self._feet]
        heading, torso_heading = _angle(rotation), _angle(torso_rotation)
        yaw_rate = 0.
        previous = self._sample
        if previous is not None:
            elapsed = time_s - previous["time_s"]
            delta = _wrap(heading - previous["heading_rad"])
            yaw_rate = delta / elapsed
            heading = previous["heading_rad"] + delta
            torso_heading = previous["torso_heading_rad"] + _wrap(torso_heading - previous["torso_heading_rad"])
            self._distance += float(np.linalg.norm(pos[:2] - previous["base_xy"]))
            old_feet = np.asarray([previous["left_foot_pos"], previous["right_foot_pos"]])
            self._foot_paths += np.linalg.norm(feet[:, :2] - old_feet[:, :2], axis=1)
            delta_v = velocity[:2] - previous["velocity_xy"]
            self._accel_square += float(delta_v @ delta_v) / elapsed
        load_l, load_r, other = floor_contacts(self.model, d, self._floor, self._sole_side)
        upright = bool(pos[2] > .45 and rotation[8] > .55)
        if not upright and self._fall_time is None:
            self._fall_time = float(time_s)
        obstacle_contact = self._obstacle_contact(time_s)
        head_top = self.profile.head_top(d)
        if self.rack_monitor is not None:
            self.rack_monitor.record(d,time_s,pos[:2],head_top)
        self._sample = dict(
            head_top_m=head_top, obstacle_contact=obstacle_contact, shutter_slides_m=self.actual_slides(), time_s=float(time_s), elapsed_s=float(time_s), base_xy=pos[:2].tolist(),
            torso_xy=torso[:2].tolist(), velocity_xy=velocity[:2].tolist(), heading_rad=heading,
            torso_heading_rad=torso_heading, yaw_rate_rad_s=yaw_rate, pelvis_height_m=float(pos[2]),
            torso_height_m=float(torso[2]), torso_tilt_rad=math.acos(float(np.clip(torso_rotation[8], -1., 1.))),
            left_foot_pos=feet[0].tolist(), right_foot_pos=feet[1].tolist(),
            left_foot_load_n=load_l, right_foot_load_n=load_r, other_ground_load_n=other,
            left_foot_contact=load_l > 1., right_foot_contact=load_r > 1.,
            upright=upright, fallen=self._fall_time is not None, first_fall_time_s=self._fall_time,
            distance_m=self._distance, left_foot_path_xy_m=float(self._foot_paths[0]),
            right_foot_path_xy_m=float(self._foot_paths[1]),
            base_acceleration_xy_rms_m_s2=math.sqrt(self._accel_square/time_s) if time_s else 0.,
        )
        return self._sample

    def _proprio(self):
        d = self.data
        m = self._sample
        return dict(q=d.qpos[self._qadr].copy(), dq=d.qvel[self._vadr].copy(),
                    base_pos=d.xpos[self._pelvis].copy(), base_quat=d.xquat[self._pelvis].copy(),
                    base_gyro=d.qvel[3:6].copy(), gravity_body=-d.xmat[self._pelvis].reshape(3,3)[2].copy(),
                    torso_pos=d.xpos[self._torso].copy(), torso_quat=d.xquat[self._torso].copy(),
                    base_velocity_world=d.qvel[:3].copy(), heading_rad=m["heading_rad"],
                    yaw_rate_rad_s=m["yaw_rate_rad_s"], left_foot_pos=d.site_xpos[self._feet[0]].copy(),
                    right_foot_pos=d.site_xpos[self._feet[1]].copy(), left_foot_contact=m["left_foot_contact"],
                    right_foot_contact=m["right_foot_contact"])

    @property
    def state(self):
        metrics = {k: v.copy() if isinstance(v, list) else v for k,v in self._sample.items()}
        metrics["target_step_rms_rad"] = math.sqrt(self._target_square / self.tick) if self.tick else 0.
        reason = "fall" if metrics["fallen"] else ("time_limit" if self.tick >= self.horizon else None)
        return dict(tick=self.tick, seconds=self.tick/50, observation=self._proprio(), metrics=metrics,
                    done=reason is not None, reason=reason)

    def decoder_history(self):
        obs = self._proprio()
        packed,_ = self._history.observation(np.zeros(64, dtype=np.float32), obs["q"], obs["dq"],
                                            obs["base_gyro"], obs["gravity_body"])
        return packed[64:].copy()

    def step(self, token, left_hand=0., right_hand=0.):
        """Decode one finite normalized token row, then integrate ten 2 ms free-base steps."""
        if self._fall_time is not None or self.tick >= self.horizon:
            raise ValueError("episode has ended")
        row = np.asarray(token)
        if row.shape != (64,) or row.dtype.kind not in "fiu":
            raise ValueError("token must contain 64 finite real values")
        with np.errstate(over="ignore"):
            row = np.asarray(row, dtype=np.float32)
        hands = np.asarray([left_hand, right_hand])
        if not np.isfinite(row).all():
            raise ValueError("token must fit finite float32")
        if (hands.shape != (2,) or hands.dtype.kind not in "fiu" or not np.isfinite(hands).all()
                or np.any((hands < 0) | (hands > 1))):
            raise ValueError("hand closure must be in [0, 1]")
        obs = self._proprio()
        target = self._decoder.decode_step(row, self._history, obs["q"], obs["dq"], obs["base_gyro"], obs["gravity_body"])
        previous = DEFAULT_Q if self.last_target is None else self.last_target
        self._target_square += float(np.mean((target-previous)**2))
        self.last_target = target
        d = self.data
        d.ctrl[self._motors] = target
        d.ctrl[self._fingers] = np.concatenate((-RIGHT_CLOSURE*hands[0], RIGHT_CLOSURE*hands[1]))
        samples = []
        for i in range(SUBSTEPS):
            start, velocity = float(d.time), d.qvel[:3].copy()
            if self.layout is not None:
                for u, phase, stroke in zip(self._leaf_u, self.layout.phases, self._strokes):
                    d.ctrl[u] = cycle(start+phase)[0]*stroke
            mujoco.mj_step(self.model, d)
            # MuJoCo's solved contacts and derived transforms refer to the start
            # of this interval. Match them to that same instant's free-joint velocity.
            if i:
                samples.append(self._measure(start, velocity))
        mujoco.mj_forward(self.model, d)
        samples.append(self._measure(d.time, d.qvel[:3]))
        # A grader may annotate its returned samples. Keep the private state used
        # by the next transition independent of those returned containers.
        self._sample = {k: v.copy() if isinstance(v, list) else v for k,v in samples[-1].items()}
        self.last_substeps = tuple(samples)
        self.tick += 1
        return self.state

    def relative_goal(self, world_xy, world_heading=None):
        """Express a world target in the pelvis heading frame (forward, left, counterclockwise)."""
        xy = np.asarray(world_xy, dtype=float)
        if xy.shape != (2,) or not np.isfinite(xy).all():
            raise ValueError("world_xy must contain two finite coordinates")
        m = self._sample
        dx,dy = xy - m["base_xy"]
        yaw = m["heading_rad"]
        c,s = math.cos(yaw), math.sin(yaw)
        result = dict(forward_m=float(c*dx+s*dy), left_m=float(-s*dx+c*dy), distance_m=float(math.hypot(dx,dy)))
        if world_heading is not None:
            if not math.isfinite(world_heading):
                raise ValueError("world heading must be finite")
            result["heading_error_rad"] = _wrap(world_heading-yaw)
        return result

    @staticmethod
    def _draw_beacons(renderer, beacons):
        # World-anchored optical landmarks are visible in RGB and depth.
        # They do not contact the robot or change its inertia.
        for position, color in beacons:
            scene = renderer.scene
            if scene.ngeom >= scene.maxgeom:
                raise RuntimeError("renderer geometry budget exhausted")
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE,
                               np.full(3, .12), np.asarray(position), np.eye(3).ravel(),
                               np.array([*color, 1.], dtype=np.float32))
            geom.emission, geom.specular = .6, 0.
            geom.segid = scene.ngeom
            scene.ngeom += 1

    def mirrored_camera(self, layout, beacons):
        """Render the same physical pose with mirrored cabinet openings in a detached MjData."""
        mirror = copy(self.data)
        self._place_furniture(mirror,layout.mirrored())
        mirror.qpos[self._leaf_q] = self.data.qpos[self._leaf_q][::-1]
        mujoco.mj_forward(self.model, mirror)
        return self.egocentric(beacons, probe=True, view_data=mirror)

    def swapped_phase_camera(self, beacons):
        """Exchange only the two measured travel fractions on a detached native world."""
        mirror = copy(self.data)
        strokes = np.asarray(self._strokes)
        mirror.qpos[self._leaf_q] = strokes*(self.data.qpos[self._leaf_q]/strokes)[::-1]
        mujoco.mj_forward(self.model, mirror)
        return self.egocentric(beacons, probe=True, view_data=mirror)

    def alternate_rack_camera(self, alternative, beacons):
        image_world = copy(self.data)
        racks.place(image_world,alternative,self.layout_origin_xy,self.layout_axes,self.layout_yaw,
                    self._rack_mocaps,self.station_m)
        mujoco.mj_forward(self.model,image_world)
        return self.egocentric(beacons,probe=True,view_data=image_world)

    def egocentric(self, beacons, *, depth=True, probe=False, view_data=None):
        data = self.data if view_data is None else view_data
        if probe:
            if self._probe_renderer is None:
                self._probe_renderer = mujoco.Renderer(self.model, width=128, height=96)
            renderer = self._probe_renderer
        else:
            if self._ego_renderer is None:
                self._ego_renderer = mujoco.Renderer(self.model, width=128, height=96)
            renderer = self._ego_renderer
        # Cull only full bounding spheres outside the camera frustum. llvmpipe
        # would otherwise transform the detailed meshes behind the robot camera
        # twice per control query. This affects drawing only.
        cam = self._ego_camera
        meshes = self._visual_meshes
        xyz = ((data.geom_xpos[meshes] - data.cam_xpos[cam])
               @ data.cam_xmat[cam].reshape(3, 3))
        radius = self.model.geom_rbound[meshes]
        forward = -xyz[:, 2]
        slope_y = math.tan(math.radians(37.5))
        slope_x = slope_y * (128 / 96)
        hidden = meshes[(forward < -radius)
                        | (np.abs(xyz[:, 0]) - slope_x*forward > radius*math.hypot(1., slope_x))
                        | (np.abs(xyz[:, 1]) - slope_y*forward > radius*math.hypot(1., slope_y))]
        self.model.geom_group[hidden] = 5
        try:
            renderer.update_scene(data, camera="egocentric")
        finally:
            self.model.geom_group[hidden] = 2
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
        self._draw_beacons(renderer, beacons)
        rgb = renderer.render().copy()
        if not depth:
            return rgb, None
        renderer.enable_depth_rendering()
        try:
            metres = renderer.render()
            valid = np.isfinite(metres) & (metres > 0) & (metres <= 12.)
            mm = np.where(valid, np.rint(np.minimum(metres, 12.) * 1000.), 0).astype(np.uint16)
        finally:
            renderer.disable_depth_rendering()
        return rgb, mm

    def camera(self, name="overview", width=640, height=480):
        """Capture the moving robot from a fixed world azimuth, for replay and review."""
        if name not in ("overview","side","overhead"):
            raise ValueError("camera must be overview, side or overhead")
        _integer(width,"width",32,1280)
        _integer(height,"height",32,960)
        if self._render_size != (width,height):
            if self._renderer is not None:
                self._renderer.close()
            self._renderer = mujoco.Renderer(self.model, width=width, height=height)
            self._render_size = width,height
        cam = mujoco.MjvCamera()
        cam.lookat[:] = (*self._sample["base_xy"], .65)
        cam.distance = 2.85
        cam.azimuth,cam.elevation = {"overview":(117.,-16.),"side":(90.,-10.),"overhead":(90.,-88.)}[name]
        self._renderer.update_scene(self.data,camera=cam)
        self._draw_beacons(self._renderer, self.beacons)
        return self._renderer.render().copy()

    def close(self):
        if self._probe_renderer is not None:
            self._probe_renderer.close()
            self._probe_renderer = None
        if self._ego_renderer is not None:
            self._ego_renderer.close()
            self._ego_renderer = None
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
            self._render_size = None

    def __enter__(self):
        return self

    def __exit__(self,*exc):
        self.close()
