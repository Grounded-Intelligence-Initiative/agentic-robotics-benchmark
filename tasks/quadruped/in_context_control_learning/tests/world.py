"""One free-base Go1, one camera, one episode-fixed set of twelve joint position drives."""
import base64
import hashlib
from io import BytesIO
import json
import math

import mujoco
import numpy as np
from PIL import Image

from lane import DELIVERIES, LEGS, LENSES, PICKUPS, aisle_dimensions, build, pit_bounds

KIND, VERSION = "icl", 9
WIDTH, HEIGHT = 384, 288
MAX_HOLD = 500
INTEGRATION = mujoco.mjtState.mjSTATE_INTEGRATION


class World:
    def __init__(self, seed, render=True, scene_index=None):
        self.model, self.stations, self.legs_by_port, yaw = build(seed, scene_index)
        self.ports_by_leg = np.argsort(self.legs_by_port)
        self.feet = [self.model.geom(leg).id for leg in LEGS]
        self.aisle = aisle_dimensions(seed,self.stations)
        self.pit = pit_bounds(seed)
        self.duct = self.model.geom("overhead_duct").id
        self.cover = self.model.geom("cable_deck").id
        self.hood = self.model.geom("camera_hood").id
        self.runners = tuple(self.model.geom(n).id for n in ("near_runner", "wide_runner"))
        self.bed = self.model.geom("pit_bed").id
        self.contact_wrench = np.zeros(6)
        self.passage = dict(hood_cross_tick=0, hood_cross_top_m=0., all_feet_through_tick=0,
                            duct_contact_ticks=0, deck_first_foot_tick=0, deck_four_feet_tick=0,
                            deck_max_feet=0, deck_dock_trunk_z_m=0.,
                            near_runner_ticks=0, wide_runner_ticks=0, both_runner_ticks=0,
                            sump_sole_ticks=0, sump_deepest_sole_m=0., runner_peak_feet=0,
                            dock1_min_near_n=0., dock1_min_wide_n=0.)
        mocap = lambda name: self.model.body_mocapid[self.model.body(name).id]
        self.rack, self.flags = mocap("launch_rack"), [mocap("clear_"+leg) for leg in LEGS]
        self.pad = mocap("pad_paint")
        self.overlay = mocap("landing_paint")
        self.route_props = [mocap(n) for n in ("station", "departure", "pointer", "pad_paint", "landing_paint")]
        self.render_enabled = render
        self.renderer = self.view = None
        self.data = mujoco.MjData(self.model)
        self.checkpoint = mujoco.MjData(self.model)
        self.trunk = self.model.body("trunk").id
        self.marker = self.model.body_mocapid[self.model.body("station").id]
        self.steps = self.ticks = self.frame_id = self.recoveries = self.falls = 0
        self.cal_steps = self.cal_ticks = 0
        self.calibration = self.current = self.approaches = self.delivered = 0
        self.dwell = self.near_ticks = self.tip_ticks = 0
        self.dwell_min_load = [0., 0.]
        self.fallen = False
        self.marks = {}
        self.dock_angles = {}
        self._buffer = np.empty(mujoco.mj_stateSize(self.model, INTEGRATION))
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
        self.data.qpos[3:7] = [math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
        self._place_station(0)
        mujoco.mj_forward(self.model, self.data)
        mujoco.mj_step(self.model, self.data, nstep=200)
        mujoco.mj_kinematics(self.model, self.data)
        self._save_checkpoint()

    def _place_station(self, index):
        x, y, yaw = self.stations[index]
        self.data.mocap_pos[self.marker] = [x, y, self.aisle["rise"] if index == 2 else 0]
        self.data.mocap_quat[self.marker] = [math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
        self.data.mocap_pos[self.pad] = self.data.mocap_pos[self.marker]
        self.data.mocap_quat[self.pad] = self.data.mocap_quat[self.marker]
        self.data.mocap_pos[self.overlay,2] = 0. if index==0 else -3.
        if index == 0:
            self.data.mocap_pos[self.pad, 2] = -self.pit[4]
        px, py, prev = self.stations[index-1] if index else (0., 0., 0.)
        travel = math.atan2(y-py, x-px)
        c, s = math.cos(prev), math.sin(prev)
        sign = self.model.body_mocapid[self.model.body("departure").id]
        pointer = self.model.body_mocapid[self.model.body("pointer").id]
        self.data.mocap_pos[sign] = self.data.mocap_pos[pointer] = [
            px + .60*c + .27*s, py + .60*s - .27*c, .43]
        cp, sp = math.cos(prev/2), math.sin(prev/2)
        ca, sa = math.cos((prev-travel)/2), math.sin((prev-travel)/2)
        self.data.mocap_quat[sign] = [cp, 0, 0, sp]
        self.data.mocap_quat[pointer] = [cp*ca, cp*sa, sp*sa, sp*ca]

    def _save_checkpoint(self):
        mujoco.mj_copyData(self.checkpoint, self.model, self.data)
        self.checkpoint_digest = self._physics_digest(self.checkpoint)

    def _restore(self):
        clock = self.data.time
        mujoco.mj_copyData(self.data, self.model, self.checkpoint)
        self.data.time = clock  # only the physical pose/dynamics are re-racked, never the clock
        self.dwell = self.tip_ticks = self.near_ticks = 0
        self.dwell_min_load = [0., 0.]
        self.fallen = False

    def _physics_digest(self, data):
        mujoco.mj_getState(self.model, data, self._buffer, INTEGRATION)
        return hashlib.sha256(self._buffer.tobytes()).hexdigest()

    def state_fingerprint(self):
        # mjSTATE_INTEGRATION includes ctrl, applied forces, warm starts, time and the visual fixture.
        state = [self._physics_digest(self.data), self.checkpoint_digest, self.metrics(),
                 self.current, self.fallen, self.dwell, self.dwell_min_load,
                 self.near_ticks, self.tip_ticks, self.legs_by_port]
        return hashlib.sha256(json.dumps(state, sort_keys=True).encode()).hexdigest()

    def metrics(self):
        dt = self.model.opt.timestep
        milestones = ("clearance", "bank", *(f"{event}_{i}" for i in range(1, DELIVERIES+1)
                                             for event in ("approach", "dock")))
        timing = {f"{name}_{field}": self.marks.get(name, (0, 0))[j] for name in milestones
                  for j, field in enumerate(("pose", "tick"))}
        return {**timing, **self.dock_angles, **self.passage, "calibration_checks": self.calibration, "approaches": self.approaches,
                "delivered": self.delivered, "steps": self.steps, "ticks": self.ticks,
                "clearance_pass": int(self.calibration >= 1), "left_bank_pass": int(self.calibration == 2),
                "simulated_seconds": self.ticks*dt, "calibration_pose_calls": self.cal_steps,
                "route_pose_calls": self.steps-self.cal_steps, "calibration_sim_seconds": self.cal_ticks*dt,
                "route_sim_seconds": (self.ticks-self.cal_ticks)*dt,
                "frames": self.frame_id, "falls": self.falls, "recoveries": self.recoveries,
                "milestone_fraction": (self.calibration + self.approaches + self.delivered)/(2 + 2*DELIVERIES)}

    def status(self):
        return {"phase": "docks" if self.current else "calibration", "calibration_checks": self.calibration,
                "dock": self.current, "approach_reached": bool(self.current and self.approaches == self.current),
                "goal_reached": bool(self.current and self.delivered == self.current),
                "fallen": self.fallen, "done": self.delivered == DELIVERIES, "frame_id": self.frame_id}

    def pickup_covered(self):
        # Four fixed short-range floor sensors. No actuator, cable index or target is consulted.
        soles = self.data.geom_xpos[self.feet]
        near_floor = soles[:,2] - self.model.geom_size[self.feet,0] <= .025
        within_pad = np.linalg.norm(PICKUPS[:,None,:] - soles[None,:,:2], axis=2) <= .075
        return (within_pad & near_floor).any(axis=1)

    def image(self):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=HEIGHT, width=WIDTH)
            self.view = mujoco.MjData(self.model)
        # Never forward the live integration state just to obtain a frame.
        mujoco.mj_copyData(self.view, self.model, self.data)
        if not self.current:
            # Place optical flags only in the display copy. Observing cannot step or support a foot.
            self.view.mocap_pos[self.route_props, 2] = -3
            self.view.mocap_pos[self.rack] = [0, 0, 0]
            for marker, covered, pos in zip(self.flags, self.pickup_covered(), LENSES):
                if not covered:
                    self.view.mocap_pos[marker] = pos
        mujoco.mj_forward(self.model, self.view)
        self.renderer.update_scene(self.view, camera="onboard")
        return self.renderer.render().copy()

    def observe(self):
        images = {}
        if self.render_enabled:
            buf = BytesIO()
            Image.fromarray(self.image()).save(buf, format="PNG")
            images["images"] = [base64.b64encode(buf.getvalue()).decode("ascii")]
        self.frame_id += 1
        return {**self.status(), **images}

    def _measure_passage(self):
        # The top support count is used for inspection parking; impacts are diagnostics, never deductions.
        d,p,s = self.data,self.passage,self.aisle
        if self.current == 2:
            contacts = d.contact.geom
            if (contacts == self.duct).any():
                p["duct_contact_ticks"] += 1
            hx,hy,hz = d.geom_xpos[self.hood]
            if (not p["hood_cross_tick"] and s["hand"]*(hy-s["gate_y"]) >= 0
                    and abs(hx-s["gate_x"]) < s["width"]/2):
                p["hood_cross_tick"] = self.ticks
                axis = d.geom_xmat[self.hood].reshape(3,3)[2]
                p["hood_cross_top_m"] = float(hz+np.abs(axis)@self.model.geom_size[self.hood])
            if (p["hood_cross_tick"] and not p["all_feet_through_tick"]
                    and np.min(s["hand"]*(d.geom_xpos[self.feet,1]-s["gate_y"])) > .078+.023):
                p["all_feet_through_tick"] = self.ticks
        elif self.current == 3:
            contacts = d.contact.geom
            feet = set()
            for ci in np.flatnonzero((contacts == self.cover).any(axis=1)):
                mujoco.mj_contactForce(self.model,d,int(ci),self.contact_wrench)
                if self.contact_wrench[0] > 0:
                    feet.update(set(contacts[ci]).intersection(self.feet))
            if feet and not p["deck_first_foot_tick"]:
                p["deck_first_foot_tick"] = self.ticks
            # A toe touching the side or a chamfer is not four feet supported on the raised top.
            count = 0
            dx,dy,a = s["deck"]
            for f in feet:
                x,y,z = d.geom_xpos[f]
                u,v = x-dx,y-dy
                if (abs(u*math.cos(a)+v*math.sin(a)) < .35 and abs(-u*math.sin(a)+v*math.cos(a)) < .32
                        and z-self.model.geom_size[f,0] > s["rise"]-.020):
                    count += 1
            if count == 4 and not p["deck_four_feet_tick"]:
                p["deck_four_feet_tick"] = self.ticks
            p["deck_max_feet"] = max(count,p["deck_max_feet"])
            return count
        return 0

    def _runner_loads(self):
        """Upward normal load from soles on the two top faces, not side scrapes."""
        d, p = self.data, self.passage
        loads = [0., 0.]
        supported, bottom = set(), False
        for ci, (a, b) in enumerate(d.contact.geom):
            if a in self.feet:
                surface, foot, sign = b, a, -1
            elif b in self.feet:
                surface, foot, sign = a, b, 1
            else:
                continue
            if surface not in (*self.runners, self.bed):
                continue
            c = d.contact[ci]
            normal_z = sign*c.frame[2]
            if normal_z < .9:
                continue
            mujoco.mj_contactForce(self.model, d, ci, self.contact_wrench)
            force = normal_z*self.contact_wrench[0]
            if force <= 0.:
                continue
            if surface == self.bed:
                bottom = True
                sole = d.geom_xpos[foot, 2] - self.model.geom_size[foot, 0]
                p["sump_deepest_sole_m"] = min(float(sole), p["sump_deepest_sole_m"])
            else:
                # The soft-contact midpoint can penetrate farther as load increases.
                # The upward-facing normal and its footprint identify the actual top.
                dx, dy = c.pos[:2]-d.geom_xpos[surface,:2]
                hx, hy = self.model.geom_size[surface,:2]
                if abs(dx) <= hx and abs(dy) <= hy:
                    loads[self.runners.index(surface)] += float(force)
                    supported.add(foot)
        p["sump_sole_ticks"] += int(bottom)
        p["near_runner_ticks"] += int(loads[0] >= 8.)
        p["wide_runner_ticks"] += int(loads[1] >= 8.)
        p["both_runner_ticks"] += int(min(loads) >= 8.)
        p["runner_peak_feet"] = max(p["runner_peak_feet"], len(supported))
        return loads

    def _tick(self):
        supporting_top_feet = self._measure_passage()
        d = self.data
        xyz, rot = d.xpos[self.trunk], d.xmat[self.trunk].reshape(3, 3)
        x0,x1,y0,y1,depth = self.pit
        over_opening = x0 < xyz[0] < x1 and y0 < xyz[1] < y1
        upright = rot[2, 2] > .78 and xyz[2] > .15 - (depth if over_opening else 0.)
        self.tip_ticks = 0 if upright else self.tip_ticks + 1
        if self.tip_ticks >= 100:
            self.fallen, self.falls = True, self.falls + 1
            return
        quiet = np.linalg.norm(d.qvel[:3]) <= .12 and np.linalg.norm(d.qvel[3:6]) <= .50
        if not self.current:
            if self.calibration == 2:
                return
            # Low clearance, then bank left (left body axis slopes toward the floor).
            posture = (.17 <= xyz[2] <= .225 and rot[2, 2] >= .97 if not self.calibration
                       else rot[2, 1] <= -math.sin(math.radians(7)) and abs(rot[2, 0]) < .15)
            ok = upright and quiet and posture and math.hypot(*xyz[:2]) <= .22
            self.dwell = self.dwell + 1 if ok else 0
            if self.dwell >= 100:
                self.calibration += 1
                self.marks["clearance" if self.calibration == 1 else "bank"] = (self.steps, self.ticks)
                self.dwell = 0
            return
        x, y, target_yaw = self.stations[self.current - 1]
        distance = math.hypot(x - xyz[0], y - xyz[1])
        self.near_ticks = self.near_ticks + 1 if upright and distance <= .24 else 0
        if self.near_ticks >= 40 and self.approaches < self.current:
            self.approaches = self.current
            self.marks[f"approach_{self.current}"] = (self.steps, self.ticks)
        yaw = math.atan2(rot[1, 0], rot[0, 0])
        aligned = abs(math.remainder(yaw - target_yaw, 2*math.pi)) <= math.radians(18)
        parked = upright and quiet and aligned and distance <= .14
        if self.current == 1:
            loads = self._runner_loads()
            parked = parked and min(loads) >= 8.
            if parked:
                self.dwell_min_load = ([min(a,b) for a,b in zip(self.dwell_min_load,loads)]
                                       if self.dwell else loads)
            else:
                self.dwell_min_load = [0., 0.]
        self.dwell = self.dwell + 1 if parked and (self.current != 3 or supporting_top_feet >= 2) else 0
        if self.dwell >= 175:
            self.delivered = self.current
            if self.current == 1:
                self.passage["dock1_min_near_n"],self.passage["dock1_min_wide_n"] = self.dwell_min_load
            if self.current == 3:
                self.passage["deck_dock_trunk_z_m"] = float(xyz[2])
            self.marks[f"dock_{self.current}"] = (self.steps, self.ticks)
            self.dock_angles[f"dock_{self.current}_yaw_deg"] = math.degrees(yaw)
            self.dock_angles[f"dock_{self.current}_heading_error_deg"] = abs(math.degrees(
                math.remainder(yaw - target_yaw, 2*math.pi)))

    def act(self, target_rad, ticks):
        if self.fallen or (self.current and self.delivered == self.current):
            raise ValueError("motion paused; recover after a fall or advance after a dock")
        if type(ticks) is not int or not 1 <= ticks <= MAX_HOLD:
            raise ValueError("ticks must be an integer from 1 through 500")
        if not isinstance(target_rad, list) or len(target_rad) != 12 or any(
                type(x) not in (int, float) for x in target_rad):
            raise ValueError("target_rad must contain exactly twelve finite numbers")
        target = np.asarray(target_rad, dtype=float)
        lo, hi = self.model.actuator_ctrlrange.T
        if not np.isfinite(target).all() or (target < lo).any() or (target > hi).any():
            raise ValueError("target outside native joint limits")
        target = target.reshape(4, 3)[self.ports_by_leg].ravel()
        self.steps += 1
        before = self.ticks
        self.data.ctrl[:] = target
        for _ in range(ticks):
            mujoco.mj_step(self.model, self.data)
            mujoco.mj_kinematics(self.model, self.data)
            self.ticks += 1
            self._tick()
            if self.fallen or (self.current and self.delivered == self.current):
                break
        if not self.current:
            self.cal_steps += 1
            self.cal_ticks += self.ticks - before
        return self.status()

    def recover(self):
        if self.current and self.current == self.delivered:
            raise ValueError("already docked")
        self._restore()
        self.recoveries += 1
        return self.status()

    def advance(self):
        if self.current != self.delivered or self.current == DELIVERIES:
            raise ValueError("advance from calibration or a completed non-final dock only")
        if not self.current:
            self._restore()  # all operators start the route at the same physical launch pose
        self.current += 1
        self._place_station(self.current - 1)
        self.dwell = self.tip_ticks = self.near_ticks = 0
        self._save_checkpoint()
        return self.status()

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
