"""One episode (one shift) of the data-hall cell: the model, the robot's controller, the grips, the mechanisms, the
cameras, the budgets.

The public surface (what a program can learn) is describe, get_state, command, step, get_camera, tool, finish (and the
site's services, cluster.py, behind noc); everything else here is the grader's: the true poses, the parts' identities
and faults, the events.
"""

from __future__ import annotations

import base64
import io
import math
import time

import mujoco
import numpy as np

from . import hall as H
from . import look as LK
from . import parts as P
from .cluster import Cluster, PhysicalState
from .control import TICK, RobotController
from .grips import Safety
from .mech import AisleDoors, DCGrips, Fixtures, Leds, Pool, Ports, Screws, Tools
from .nav import Nav
from . import scoring as SC
from .robot import (ARM_HOME, D435_PITCH, D435_POS, GRASP_SITE_POS, HEAD_CAM, PELVIS_HEIGHT, SIDES, STAND_RANGE,
                    robot_index)
from .scene import WRIST_CAM, apply_initial_state, build, wrist_cam_mount
from .walk_core import LegWalker

SUBSTEPS = 5
ROBOT = "g1"
# the motor state get_state reports for every motor a program can read (the waist's 3, the arms' 2 x 7, the hands'
# 2 x 7), as a G1's low-level state does (the body's LowState and the Dex3-1 hands' HandState, one entry per motor):
#   q        the joint angle, exact (the encoders' resolution is far below anything the task needs);
#   dq       the joint velocity, with reading-to-reading noise (DQ_NOISE rad/s, 1 sigma), quantised to DQ_STEP;
#   tau_est  the joint torque estimated from the motor's current: the drive's torque (the servo's plus the gravity
#            feed-forward that holds the robot's own links), read with a calibration error that stays for the shift
#            (a scale error within TAU_GAIN and an offset of TAU_OFFSET x the joint's torque limit, 1 sigma) and
#            reading-to-reading noise (TAU_NOISE x the limit, 1 sigma), quantised to TAU_STEP
TAU_GAIN = 0.05
TAU_OFFSET = 0.015
TAU_NOISE = 0.005
TAU_STEP = 0.01
DQ_NOISE = 0.01
DQ_STEP = 0.001
LOC_NOISE = (0.003, math.radians(0.2))
CAMERAS = {"head": {"mj": "g1/head_cam", "size": HEAD_CAM["size"], "fovy": HEAD_CAM["fovy"], "depth": (0.28, 6.0)},
           "wrist_left": {"mj": "wrist_cam_left", "size": WRIST_CAM["size"], "fovy": WRIST_CAM["fovy"],
                          "depth": (WRIST_CAM["depth_min"], WRIST_CAM["depth_max"])},
           "wrist_right": {"mj": "wrist_cam_right", "size": WRIST_CAM["size"], "fovy": WRIST_CAM["fovy"],
                           "depth": (WRIST_CAM["depth_min"], WRIST_CAM["depth_max"])}}
API = "dc-cell/0.1"
# the cameras' sensor model (each shift its own white balance and exposure per camera; per-pixel noise): colour gain per
# channel, overall gain, pixel noise (sigma, on 0..1); depth along the optical axis: noise sigma 1.5 mm + 2.5 mm/m^2 z^2,
# 1 mm steps, no reading (0) outside the range, on depth discontinuities (a jump of more than 3 % to a neighbour) and on
# 2 % of the pixels at random
CAM_TINT, CAM_GAIN, RGB_NOISE = (0.94, 1.06), (0.92, 1.08), 0.008
DEPTH_SIGMA, DEPTH_JUMP, DEPTH_DROPOUT = (0.0015, 0.0025), 0.03, 0.02


def intrinsics(w, h, fovy):
    f = (h / 2.0) / math.tan(math.radians(fovy) / 2.0)
    return {"width": w, "height": h, "fx": f, "fy": f, "cx": (w - 1) / 2.0, "cy": (h - 1) / 2.0, "fovy_deg": fovy}


class Cell:
    def __init__(self, seed: int = 0, *, cfg: dict | None = None, render: bool = True, robot_start=None):
        self.cfg = cfg = dict(cfg or {})
        self.seed = int(seed)
        t0 = time.time()
        self.spec, self.meta = build(self.scene_cfg(cfg.get("scene", {}), self.seed))
        self.m = self.spec.compile()
        self.d = mujoco.MjData(self.m)
        self.build_s = time.time() - t0
        apply_initial_state(self.m, self.d, self.meta)
        self.robot = robot_index(self.m, ROBOT)
        self.ctrl = RobotController(self.m, self.robot)
        self.events = []
        self.ports = Ports(self.m, self.d, self.meta)
        self.pool = Pool(self.m, self.d, self.meta, self.ports, self.events)
        self.grips = DCGrips(self.m, self.d, self.robot, self.meta, self.ports, self.pool)
        self.pool.grips = self.grips
        self.fixtures = Fixtures(self.m, self.d, self.meta, self.grips)
        self.screws = Screws(self.ports, self.events)
        self.tools = Tools(self.m, self.d, self.meta, self.grips, self.screws, self.events)
        self.tools.faces = self.faces
        self.pool.release_ok = self._release_ok
        self.safety = Safety(self.m, self.d, self.robot, self.grips, loose=self.meta["loose"])
        self.leds = Leds(self.m, self.meta)
        self.aisle_doors = AisleDoors(self.m, self.d)
        mujoco.mj_forward(self.m, self.d)
        self.nav = Nav(self.m, self.d)
        self.cluster = Cluster(self.seed, ports=self.meta["ports"], config=cfg.get("cluster"))
        self._ev_sent = 0
        self._next_cluster = 0.0
        self.report = None
        self._door_joints = [f["joint"] for f in self.meta["fixtures"].values() if f["type"] == "rdoor"] + \
            [self.m.joint(j).name for j in range(self.m.njnt) if self.m.joint(j).name.startswith("cdoor_")]
        self.rng = np.random.default_rng((self.seed * 2654435761 + 11) & ((1 << 63) - 1))
        cg = np.random.default_rng([self.seed & ((1 << 63) - 1), 0xCA3E5])      # the cameras' own draws
        self.cam_tint = {n: cg.uniform(CAM_TINT[0], CAM_TINT[1], size=3) for n in CAMERAS}
        self.cam_gain = {n: float(cg.uniform(CAM_GAIN[0], CAM_GAIN[1])) for n in CAMERAS}
        self.cam_rng = np.random.default_rng(cg.integers(1 << 62))
        self._tau_init()
        self.walker = LegWalker(self.m, ROBOT + "/") if render else None
        self.render = render
        self._view = None
        self._depth = {}
        self._rd = None
        self._q_kin = None
        self.ticks = 0
        self.done = False
        self.frames_total = 0
        self.frames = {n: 0 for n in CAMERAS}
        self.done_reason = None
        self.cams = self if render else None
        self.frame_budget = int(cfg.get("camera_budget_frames", 2000))
        self.sim_cap = float(cfg.get("sim_cap_s", 7200.0))
        self.work_wall = 0.0
        self._reset(robot_start or self.start_pose(self.seed))

    # ------------------------------------------------------------------------------------------------ setup
    @staticmethod
    def scene_cfg(scene, seed):
        """The scene's build config for this shift's pod: the racks' installation offsets and where the cart holds
        each spare are drawn from the seed unless given (scene["pod"] = {} builds the drawing's nominal pod)."""
        from .scene import SPARES, pod_offsets
        scene = dict(scene)
        if "pod" not in scene:
            scene["pod"] = pod_offsets(seed)
        if "spare_jitter" not in scene:
            rng = np.random.default_rng([int(seed) & 0xFFFFFFFF, (int(seed) >> 32) & 0xFFFFFFFF, 0x5BA])
            count, jit = {}, {}
            for kind, *_ in SPARES:
                i = count.get(kind, 0)
                count[kind] = i + 1
                jit["px_%s_%d" % (kind, i)] = (float(rng.uniform(-0.012, 0.012)), float(rng.uniform(-0.012, 0.012)))
            scene["spare_jitter"] = jit
        return scene

    @staticmethod
    def start_pose(seed):
        """Where the shift starts: by the parts cart west of the cold aisle, facing a random way (from the seed)."""
        rng = np.random.default_rng([int(seed) & 0xFFFFFFFF, (int(seed) >> 32) & 0xFFFFFFFF, 0x57A])
        return (float(rng.uniform(-1.35, -0.75)), float(rng.uniform(-0.55, 0.35)), float(rng.uniform(-math.pi, math.pi)))

    def _reset(self, start):
        m, d, r = self.m, self.d, self.robot
        x, y, yaw = start
        d.qpos[r.base_q[:3]] = (x, y, yaw)
        d.qpos[r.base_q[3]] = 0.0
        for s in SIDES:
            d.qpos[r.arm_q[s]] = ARM_HOME[s]
        mujoco.mj_kinematics(m, d)                           # the pelvis where it stands: the belt's holsters
        self.tools.holster_all()
        mujoco.mj_forward(m, d)
        if self.walker is not None:
            self._q_kin = d.qpos.copy()
        self.ctrl.reset_targets(d)
        for _ in range(int(float(self.cfg.get("settle_s", 0.5)) / TICK)):
            self._tick(record=False)
        self.ticks = 0
        self.safety.violations.clear()
        self.safety.count = 0
        self._ev_sent = len(self.events)                     # the settle's events are the scene's own
        self._cluster_tick()
        if self.walker is not None:
            self.walker.reset(self.sim_time, d.qpos[self.walker.base_adr])

    @property
    def sim_time(self):
        return self.ticks * TICK

    def _tick(self, record=True):
        m, d = self.m, self.d
        for s in SIDES:
            self.ctrl.hand_shape[s] = self.grips.shape(s)
        if self.nav.status == "moving":
            self.nav.tick(self.sim_time, d.qpos[self.robot.base_q[:3]], self.ctrl)
        self.ctrl.tick(d)
        for i in range(SUBSTEPS):
            if i == SUBSTEPS - 1 and self.walker is not None:
                self._q_kin[:] = d.qpos
            mujoco.mj_step(m, d)
        t = self.sim_time
        self.grips.tick(t)
        self.pool.tick(t)
        self.fixtures.tick(t, self.events)
        self.tools.tick(t)
        self.aisle_doors.tick(t, self.d.qpos[self.robot.base_q[:2]])
        if record:
            self.safety.tick(t)
            self.ticks += 1
            self._forward_events(t)
            if self.sim_time >= self._next_cluster - 1e-9:
                self._cluster_tick()
            if self.walker is not None:
                self.walker.step(self.sim_time, d.qpos[self.walker.base_adr])

    # ------------------------------------------------------------------------------------------------ the cluster
    def _forward_events(self, t):
        """The physical events (ports, trays, screws, cleaner clicks) to the cluster model, in order."""
        evs = self.events
        while self._ev_sent < len(evs):
            ev = evs[self._ev_sent]
            self._ev_sent += 1
            self.cluster.on_event(ev, t)

    def _phys(self):
        q = lambda jn: float(self.d.qpos[self.fixtures.jq(jn)])
        ps = PhysicalState()
        for host, tr in self.meta["trays"].items():
            ps.tray_pos[host] = q(tr["joint"])
            ps.tray_latched[host] = bool(self.fixtures.latched[host])
            ps.cover_open[host] = q(tr["cover"])
        ps.occupied = dict(self.ports.occupied)
        ps.instance = dict(self.ports.instance)
        for slot in self.screws.turns:
            ps.screws_fastened[slot] = self.screws.fastened(slot)
            ps.screws_loose[slot] = self.screws.loose(slot)
            ps.gpu_damaged[slot] = bool(self.screws.damaged[slot])
        ps.port_broken = dict(self.ports.broken)
        x, y = (float(v) for v in self.d.qpos[self.robot.base_q[:2]])
        ps.robot_xy = (x, y)
        for jn in self._door_joints:                         # the racks' rear doors (rad), the aisle's sliding doors (m)
            ps.doors[jn] = q(jn)
        return ps

    def _locations(self):
        """Where each part out of a port is, for the shift's report: "hand", "cart", "floor" or "elsewhere" (the RMA
        tote's parts are reported apart)."""
        held = {st["obj"] for st in self.grips.state.values() if st["obj"]}
        cart = self.meta["cart"]
        cb = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, cart["body"])
        cx, cy = self.d.xpos[cb][:2]
        tote = set(self._tote_parts())
        out = {}
        for px, st in self.pool.state.items():
            inst = self.pool.instance.get(px)
            if st != "free" or inst is None or inst in tote:
                continue
            p = self.d.xpos[self.pool.body[px]]
            if px in held:
                out[inst] = "hand"
            elif abs(p[0] - cx) <= 0.32 and abs(p[1] - cy) <= 0.47 and cart["top"] - 0.02 <= p[2] <= cart["top"] + 0.30:
                out[inst] = "cart"
            elif p[2] < 0.15:
                out[inst] = "floor"
            else:
                out[inst] = "elsewhere"
        return out

    def _cluster_tick(self):
        """Every 1.0 s of sim time: the cluster's continuous processes, then the LEDs it reports."""
        t = self.sim_time
        self.cluster.tick(t, self._phys())
        for name, st in self.cluster.leds(t).items():
            self.leds.set(name, st)
        self._next_cluster = math.floor(t + 1e-9) + 1.0

    def _tote_parts(self):
        """The part ids whose bodies rest inside the RMA tote (a free proxy within its walls)."""
        tb = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "rma_tote")
        c = self.d.xpos[tb]
        out = []
        held = {st["obj"] for st in self.grips.state.values() if st["obj"]}
        for px, st in self.pool.state.items():
            if st != "free" or self.pool.instance.get(px) is None or px in held:
                continue
            p = self.d.xpos[self.pool.body[px]]
            if abs(p[0] - c[0]) <= 0.28 and abs(p[1] - c[1]) <= 0.20 and 0.0 <= p[2] - c[2] <= 0.32:
                out.append(self.pool.instance[px])
        return out

    # ------------------------------------------------------------------------------------------------ rules' hooks
    def _release_ok(self, port):
        """Whether the part in `port` may leave its slot (the interlocks a technician meets)."""
        kind = self.ports.rec[port]["kind"]
        if kind in ("osfp_n", "osfp_s"):
            mpo = port.replace("osfp", "mpo")
            return not self.ports.occupied.get(mpo, False)
        if kind == "psu":
            plug = port.replace("psu", "plug")
            return not self.ports.occupied.get(plug, False)
        if kind == "gpu":
            return self.screws.loose(port)
        return True

    def faces(self):
        """Every optical end face a scope or cleaner can reach now: (id, centre, outward normal) in the world. An MPO
        connector's face looks into its receptacle (-y of its frame); a transceiver's receptacle face looks out of it."""
        out = []
        for port, rec in self.ports.rec.items():
            if rec["kind"] != "mpo":
                continue
            need = rec["needs"]
            if self.ports.occupied[need] and not self.ports.occupied[port]:
                o, R, ax = self.ports.world(port)                    # the transceiver's receptacle, empty
                out.append(("rx:" + str(self.ports.instance[need]), o, ax))
        for px, st in self.pool.state.items():
            if self.pool.kind[px] == "mpo" and st in ("free",):
                b = self.pool.body[px]
                Rb = self.d.xmat[b].reshape(3, 3)
                out.append(("mpo:" + str(self.pool.instance[px]), self.d.xpos[b].copy(), -Rb[:, 1]))
            if self.pool.kind[px] in ("osfp_n", "osfp_s") and st == "free":
                b = self.pool.body[px]
                Rb = self.d.xmat[b].reshape(3, 3)
                c = self.d.xpos[b] + Rb @ np.asarray(P.OSFP_RECEPTACLE[self.pool.kind[px]], float)
                out.append(("rx:" + str(self.pool.instance[px]), c, Rb[:, 1]))
        return out

    # ------------------------------------------------------------------------------------------------ public API
    def describe(self):
        """The cell's published facts (static for the shift; nothing about the night's faults)."""
        from .control import A_MAX, GOAL_GAIN, GOAL_TOL, V_MAX
        from .grips import GRIP_MAX_FORCE, GRIP_MAX_TORQUE, GRIP_MAX_TORQUE_BAR, POINT_GRIP_MAX_FORCE, SAFETY
        from .nav import NARROW, R_HARD, R_SOFT
        from .robot import ARM_JOINTS, HAND_JOINTS, WAIST_JOINTS
        joints = {"waist": [f"{w}_joint" for w in WAIST_JOINTS],
                  "left_arm": [f"left_{w}_joint" for w in ARM_JOINTS],
                  "right_arm": [f"right_{w}_joint" for w in ARM_JOINTS],
                  "left_hand": [f"left_hand_{w}_joint" for w in HAND_JOINTS],
                  "right_hand": [f"right_hand_{w}_joint" for w in HAND_JOINTS]}
        ranges = {}
        for names in joints.values():
            for jn in names:
                j = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, f"{ROBOT}/" + jn)
                ranges[jn] = [round(float(self.m.jnt_range[j, 0]), 4), round(float(self.m.jnt_range[j, 1]), 4)]
        cams = {}
        for n, c in CAMERAS.items():
            cams[n] = {"intrinsics": intrinsics(c["size"][0], c["size"][1], c["fovy"]), "depth_range_m": list(c["depth"]),
                       "depth_noise_m": {"sigma_at_0": DEPTH_SIGMA[0], "sigma_per_m2": DEPTH_SIGMA[1]},
                       "depth_invalid": {"value": 0, "discontinuity_rel": DEPTH_JUMP, "dropout_fraction": DEPTH_DROPOUT},
                       "colour": {"white_balance_per_channel": list(CAM_TINT), "exposure_gain": list(CAM_GAIN),
                                  "pixel_noise_sigma": RGB_NOISE, "per": "shift and camera"}}
        cams["head"]["mount"] = {"body": "torso_link", "position": list(D435_POS), "pitch_down_rad": D435_PITCH,
                                 "optical_axis": "body +x pitched down"}
        for side in SIDES:
            pos, quat = wrist_cam_mount(side)
            cams["wrist_" + side]["mount"] = {"body": "%s_wrist_yaw_link" % side, "position": [round(v, 4) for v in pos],
                                              "quaternion_wxyz_mujoco_camera": [round(float(v), 6) for v in quat],
                                              "optical_axis": "the camera frame's -z (MuJoCo convention, +y up)"}
        return {
            "api": API,
            "robot": {"model": "unitree_g1_29dof_dex3 (cell variant)",
                      "motor_state": {"joints": list(self.tau_names),
                                      "fields": {"q": "rad, exact", "dq": "rad/s, noisy",
                                                 "tau_est": "N m, estimated from the motor current"},
                                      "torque_limits_Nm": {n: float(v) for n, v in zip(self.tau_names, self.tau_limit)}},
                      "belt": "three holsters on the pelvis: the driver on the right hip, the probe and the cleaner on "
                              "the left (cell_manual.md)"},
            "joints": joints, "joint_ranges": ranges,
            "control": {"tick_s": TICK, "base_velocity_max": list(V_MAX), "base_acceleration_max": list(A_MAX),
                        "base_goal": {"gain_per_s": GOAL_GAIN, "tolerance": list(GOAL_TOL)},
                        "stand_height_range": list(STAND_RANGE), "stand_height_nominal": PELVIS_HEIGHT,
                        "arm_joint_rate_rad_s": 1.5, "waist_rate_rad_s": 0.8,
                        "grasp_site": {"body": "<side>_wrist_yaw_link",
                                       "position": {s: list(GRASP_SITE_POS[s]) for s in SIDES},
                                       "axes": "x = approach (along the fingers), y = pinch axis, z = x cross y"}},
            "navigation": {"footprint_radius_m": R_HARD, "keeps_clear_m": R_SOFT, "narrow_passage_m": NARROW,
                           "grid_m": 0.05, "states": ["idle", "moving", "arrived", "blocked", "no_path"],
                           "note": "the plan keeps the base's centre at least footprint_radius_m from every obstacle "
                                   "except near its start and its goal; the arms and what the hands hold are not in "
                                   "the plan (cell_manual.md)"},
            "grip": {"max_force_N": GRIP_MAX_FORCE, "max_torque_Nm": GRIP_MAX_TORQUE,
                     "max_torque_bar_Nm": GRIP_MAX_TORQUE_BAR, "point_grip_max_force_N": POINT_GRIP_MAX_FORCE},
            "cameras": cams, "camera_budget_frames": self.frame_budget,
            "budgets": {"sim_cap_s": self.sim_cap},
            "safety": {"robot_contact_N": SAFETY["robot_contact_N"],
                       "robot_contact_s": SAFETY["robot_contact_ticks"] * TICK,
                       "throw_speed_m_s": SAFETY["throw_speed"], "strike_speed_m_s": SAFETY["strike_speed"]},
        }

    def _tau_init(self):
        """The motors whose state get_state reports (the waist's, the arms' and the hands'), their joints' position and
        velocity addresses, their drives' torque limits, and the shift's calibration errors: drawn from a generator of
        their own (the localisation noise and the scene's draws are untouched)."""
        from .robot import ARM_JOINTS, HAND_JOINTS, WAIST_JOINTS
        m, r = self.m, self.robot
        names = [f"{w}_joint" for w in WAIST_JOINTS] + [f"{s}_{w}_joint" for s in SIDES for w in ARM_JOINTS] \
            + [f"{s}_hand_{w}_joint" for s in SIDES for w in HAND_JOINTS]
        qadr = list(r.waist_q) + [a for s in SIDES for a in r.arm_q[s]] + [a for s in SIDES for a in r.hand_q[s]]
        acts = list(r.waist_act) + [a for s in SIDES for a in r.arm_act[s]] + [a for s in SIDES for a in r.hand_act[s]]
        dofs = [int(m.jnt_dofadr[m.actuator_trnid[a, 0]]) for a in acts]
        self.tau_names = names
        self.tau_qadr = np.array(qadr, int)
        self.tau_dof = np.array(dofs, int)
        self.tau_act = np.array(acts, int)
        self.tau_limit = np.array([float(m.actuator_forcerange[a, 1]) for a in acts])
        g = np.random.default_rng([self.seed & ((1 << 63) - 1), 0x7A0E57])
        self.tau_gain = 1.0 + g.uniform(-TAU_GAIN, TAU_GAIN, len(names))
        self.tau_offset = g.normal(0.0, TAU_OFFSET, len(names)) * self.tau_limit
        self.tau_rng = np.random.default_rng(g.integers(1 << 62))
        self.dq_rng = np.random.default_rng(g.integers(1 << 62))

    def tau_true(self):
        """The drives' torques now (N m): the servo's force plus the gravity compensation of the robot's links (the
        real robot's motors carry both)."""
        return self.d.actuator_force[self.tau_act] + self.d.qfrc_gravcomp[self.tau_dof]

    def tau_est(self):
        tau = self.tau_true() * self.tau_gain + self.tau_offset \
            + self.tau_rng.normal(0.0, 1.0, len(self.tau_names)) * TAU_NOISE * self.tau_limit
        tau = np.round(tau / TAU_STEP) * TAU_STEP
        return {n: round(float(v), 3) for n, v in zip(self.tau_names, tau)}

    def dq_est(self):
        dq = self.d.qvel[self.tau_dof] + self.dq_rng.normal(0.0, DQ_NOISE, len(self.tau_names))
        dq = np.round(dq / DQ_STEP) * DQ_STEP
        return {n: round(float(v), 4) for n, v in zip(self.tau_names, dq)}

    def _grasp_in_base(self, side):
        d, r = self.d, self.robot
        x, y, yaw = (float(v) for v in d.qpos[r.base_q[:3]])
        c, s = math.cos(yaw), math.sin(yaw)
        Rb = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        sid = r.grasp_site[side]
        p = Rb.T @ (d.site_xpos[sid] - np.array([x, y, 0.0]))
        Rs = Rb.T @ d.site_xmat[sid].reshape(3, 3)
        qq = np.zeros(4)
        mujoco.mju_mat2Quat(qq, Rs.reshape(-1))
        return [round(float(v), 5) for v in p] + [round(float(v), 6) for v in qq]

    def _localised(self):
        x, y, yaw = (float(v) for v in self.d.qpos[self.robot.base_q[:3]])
        return (x + float(self.rng.normal(0, LOC_NOISE[0])), y + float(self.rng.normal(0, LOC_NOISE[0])),
                yaw + float(self.rng.normal(0, LOC_NOISE[1])))

    def get_state(self):
        d, r, c = self.d, self.robot, self.ctrl
        hands = {}
        q = {n: float(d.qpos[a]) for n, a in zip(self.tau_names, self.tau_qadr)}
        for s in SIDES:
            hands[s] = {"closed": c.hand_cmd[s] == "close", "tactile_N": round(self.grips.tactile(s), 2),
                        "grasp_pose_in_robot": self._grasp_in_base(s),
                        "ik_residual": [round(c.arm_ik_err[s][0], 5), round(c.arm_ik_err[s][1], 5)]
                        if c.arm_mode[s] == "pose" else None}
        return {"sim_time": round(self.sim_time, 4), "done": self.done,
                "robot": {"base_pose": [round(v, 5) for v in self._localised()],
                          "stand_height": round(PELVIS_HEIGHT + float(d.qpos[r.base_q[3]]), 4),
                          "base_moving": bool(np.linalg.norm(c.vel) > 1e-4),
                          "q": {k: round(v, 5) for k, v in q.items()}, "dq": self.dq_est(), "tau_est": self.tau_est(),
                          "hands": hands, "nav": self.nav.state(d.qpos[r.base_q[:3]])}}

    def command(self, cmd):
        cmd = dict(cmd)
        nav = None
        if "base_goal" in cmd or "base_velocity" in cmd:
            self.nav.cancel()
        if "nav_goal" in cmd:
            g = cmd.pop("nav_goal")
            if g is None:
                self.nav.cancel()
                self.ctrl.command({"base_velocity": [0.0, 0.0, 0.0]})
            else:
                nav = self.nav.start(g, self.d.qpos[self.robot.base_q[:3]])
                if not nav.get("ok"):
                    self.ctrl.command({"base_velocity": [0.0, 0.0, 0.0]})
        notes = self.ctrl.command(cmd)
        for s in SIDES:
            h = cmd.get(f"{s}_hand")
            if h in ("open", "close"):
                self.grips.command(s, h == "close", self.sim_time)
        out = {"ok": True, "notes": notes}
        if nav is not None:
            out["nav"] = nav
        return out

    def step(self, n_ticks):
        events = []
        n_viol = len(self.safety.violations)
        t = time.time()
        for _ in range(int(n_ticks)):
            if self.done:
                break
            self._tick()
            if self.sim_time >= self.sim_cap - 1e-9:
                self._end("sim_cap")
                events.append("episode_over")
                break
        self.work_wall += time.time() - t
        for v in self.safety.violations[n_viol:]:
            events.append("safety:" + v["what"])
        return {"sim_time": round(self.sim_time, 4), "done": self.done, "events": events}

    def tool(self, name, **params):
        """The smart tools: driver (mode tighten / loosen, torque N m, seconds) and scope (capture)."""
        if name == "driver":
            if params.get("action") == "status":
                return dict(self.tools.run or {"result": "idle"})
            return self.tools.start_driver(params.get("mode"), params.get("torque", P.SCREW["final_Nm"]),
                                           params.get("seconds", 2.0), self.sim_time)
        if name == "scope":
            # the probe's picture of whatever end face is in front of its tip (a blurred nothing when none is);
            # the capture id is the evidence string a ticket quotes
            if self.tools.held_side("tool_scope") is None:
                return {"error": "not_held"}
            v = self.tools.scope_view()
            face, lat, ang = (v[0], v[1], v[2]) if v is not None else ("none", 0.01, 0.0)
            img = self.cluster.scope_image(face, lat, ang, self.sim_time)
            cap = self.cluster.last_capture["id"]
            if params.get("raw"):
                return {"ok": True, "capture_id": cap, "image": img}
            from PIL import Image
            buf = io.BytesIO()
            Image.fromarray(img).save(buf, format="PNG")
            return {"ok": True, "capture_id": cap, "width": int(img.shape[1]), "height": int(img.shape[0]),
                    "image_png_b64": base64.b64encode(buf.getvalue()).decode(), "sim_time": round(self.sim_time, 4)}
        return {"error": "bad_params"}

    def noc(self, method, params=None):
        """The site's services (cluster.py: NOC tickets, DCIM, BMC, PDUs, fabric manager, scheduler, DCGM). A call
        takes cluster.api_cost_s(method) of sim time, during which the robot holds its commands."""
        if self.done:
            return {"error": "episode_over"}
        out = self.cluster.api(method, params, self.sim_time)
        self.step(int(round(self.cluster.api_cost_s(method) / TICK)))
        return out

    # ------------------------------------------------------------------------------------------------ cameras
    def render_state(self):
        if self.walker is None:
            return self.d
        if self._rd is None:
            self._rd = mujoco.MjData(self.m)
        rd = self._rd
        self.pool.draw_cables()
        rd.qpos[:] = self._q_kin
        rd.mocap_pos[:] = self.d.mocap_pos
        rd.mocap_quat[:] = self.d.mocap_quat
        w = self.walker
        rd.qpos[w.qpos_adr] = w.pose_at(self.sim_time, self._q_kin[w.base_adr])
        mujoco.mj_kinematics(self.m, rd)
        mujoco.mj_comPos(self.m, rd)
        mujoco.mj_camlight(self.m, rd)
        return rd

    def get_camera(self, name, depth=True, raw=False):
        c = CAMERAS.get(name)
        if c is None:
            return {"error": "bad_params"}
        if self.frames_total >= self.frame_budget:
            return {"error": "camera_budget"}
        self.frames_total += 1
        self.frames[name] = self.frames.get(name, 0) + 1
        t_cam = time.time()
        rd = self.render_state()
        cid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_CAMERA, c["mj"])
        R = rd.cam_xmat[cid].reshape(3, 3)
        w, h = c["size"]
        if self._view is None:
            self._view = LK.Filament.instance().open(self.m)
        rgb = self._view.render(rd, rd.cam_xpos[cid], -R[:, 2], R[:, 1], c["fovy"], w, h)
        dep = None
        if depth:
            if name not in self._depth:
                r = mujoco.Renderer(self.m, h, w)
                r.enable_depth_rendering()
                self._depth[name] = r
            r = self._depth[name]
            r.update_scene(rd, camera=c["mj"])
            dep = r.render().astype(np.float32)
            lo, hi = c["depth"]
            dep[(dep < lo) | (dep > hi)] = np.nan
        if raw:
            pose = {"position": [float(v) for v in rd.cam_xpos[cid]],
                    "rotation": np.stack([R[:, 0], -R[:, 1], -R[:, 2]], 1).tolist()}
            self.work_wall += time.time() - t_cam
            return {"rgb": rgb, "depth": dep, "pose": pose}
        # the pose as the robot knows it: its kinematics from where it believes it stands (the localisation's error)
        r = self.robot
        x, y, yaw = (float(v) for v in self.d.qpos[r.base_q[:3]])
        lx, ly, lyaw = x + float(self.cam_rng.normal(0, LOC_NOISE[0])), y + float(self.cam_rng.normal(0, LOC_NOISE[0])), \
            yaw + float(self.cam_rng.normal(0, LOC_NOISE[1]))
        cz, sz = math.cos(lyaw - yaw), math.sin(lyaw - yaw)
        Rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
        p = Rz @ (rd.cam_xpos[cid] - np.array([x, y, 0.0])) + np.array([lx, ly, 0.0])
        Rc = Rz @ np.stack([R[:, 0], -R[:, 1], -R[:, 2]], 1)
        pose = {"position": [float(v) for v in p], "rotation": Rc.tolist()}
        # the sensor: white balance, exposure and noise on the colour; noise, steps and dropouts on the depth
        g = self.cam_rng
        rgbf = rgb.astype(np.float32) / 255.0 * self.cam_tint[name] * self.cam_gain[name]
        rgbf = rgbf + g.normal(0.0, RGB_NOISE, size=rgbf.shape)
        rgb = (np.clip(rgbf, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)
        self.work_wall += time.time() - t_cam
        from PIL import Image
        buf = io.BytesIO()
        Image.fromarray(rgb).save(buf, format="PNG")
        out = {"camera": name, "width": w, "height": h, "rgb_png_b64": base64.b64encode(buf.getvalue()).decode(),
               "intrinsics": intrinsics(w, h, c["fovy"]), "pose_in_room": pose, "sim_time": round(self.sim_time, 4)}
        if dep is not None:                                  # 16-bit PNG, millimetres, 0 = no reading
            z = np.nan_to_num(dep.astype(np.float64), nan=0.0)
            valid = ~np.isnan(dep)
            zn = z + g.normal(0.0, 1.0, size=z.shape) * (DEPTH_SIGMA[0] + DEPTH_SIGMA[1] * z * z)
            jump = np.zeros(z.shape, dtype=bool)
            for ax in (0, 1):
                a_, b_ = np.take(z, range(z.shape[ax] - 1), axis=ax), np.take(z, range(1, z.shape[ax]), axis=ax)
                dz = np.abs(b_ - a_) > DEPTH_JUMP * np.minimum(a_, b_)
                if ax == 0:
                    jump[:-1] |= dz
                    jump[1:] |= dz
                else:
                    jump[:, :-1] |= dz
                    jump[:, 1:] |= dz
            ok = valid & ~jump & (g.random(z.shape) >= DEPTH_DROPOUT)
            mm = np.where(ok, np.clip(np.round(zn * 1000.0), 1, 65535), 0).astype(np.uint16)
            buf = io.BytesIO()
            Image.fromarray(mm).save(buf, format="PNG")
            out["sensor_depth_u16_mm_b64"] = base64.b64encode(buf.getvalue()).decode()
            out["depth_unit_m"] = 0.001
            out["depth_invalid_value"] = 0
        return out

    def finish(self):
        self._end("finish")
        return {"ok": True, "sim_time": round(self.sim_time, 4)}

    def _end(self, reason):
        """End the episode (once): the cluster's final report on the state as it stands."""
        if self.report is None:
            self._forward_events(self.sim_time)
            self.report = self.cluster.final_report(self.sim_time, parts_in_tote=self._tote_parts(),
                                                    locations=self._locations())
        if self.done_reason is None:
            self.done_reason = str(reason)
        self.done = True

    def score(self):
        """The episode's record (scoring.score on the final report and the safety watch)."""
        if self.report is None:
            self._end("scored")
        viol = [v.get("what") for v in self.safety.violations]
        rec = SC.score(self.report, safety_ok=not viol, violations=viol)
        rec.update({"sim_time_s": round(self.sim_time, 3), "camera_frames": dict(self.frames),
                    "camera_frames_total": int(self.frames_total), "done_reason": self.done_reason,
                    "job": self.report.get("job"), "incidents": self.report.get("incidents"),
                    "tickets": [{k: t.get(k) for k in ("id", "kind", "host", "cause", "status", "fault_gone",
                                                       "reported_root_cause", "escalated_to", "in_service")}
                                for t in self.report.get("tickets", [])]})
        return rec

    def close(self):
        """Free the renderers (the server's close)."""
        self._view = None
        for r in self._depth.values():
            try:
                r.close()
            except Exception:  # noqa: BLE001
                pass
        self._depth = {}

    # ------------------------------------------------------------------------------------------------ grader / dev side
    def body_pose(self, name):
        b = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, name)
        return self.d.xpos[b].copy(), self.d.xmat[b].reshape(3, 3).copy()

    def joint_q(self, name):
        j = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, name)
        return float(self.d.qpos[self.m.jnt_qposadr[j]])

    def feature_world(self, body_or_port, k=0):
        """The world segment (a, b) and radius of a graspable body's (or a seated port's) k-th bar feature."""
        if body_or_port in self.ports.rec:
            o, R, _ = self.ports.world(body_or_port)
            f = P.KINDS[self.ports.rec[body_or_port]["kind"]]["features"][k]
        else:
            o, R = self.body_pose(body_or_port)
            f = self.meta["features"][body_or_port][k]
        return o + R @ np.asarray(f[1], float), o + R @ np.asarray(f[2], float), float(f[3])
