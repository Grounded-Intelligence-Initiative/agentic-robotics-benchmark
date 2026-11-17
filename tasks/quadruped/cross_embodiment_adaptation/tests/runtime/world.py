"""Trusted v4 MuJoCo scout/carrier mission. Stabilized planar bases, not gait physics.

Carrier x/y slides and yaw hinge are force driven by mj_step. Native contacts
include the welded payload and every other descendant of the agent body.
"""
import base64
import io
import math
import os

os.environ.setdefault('MUJOCO_GL', 'osmesa')
os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
os.environ.setdefault('LP_NUM_THREADS', '2')
import mujoco
import numpy as np
from PIL import Image

from geometry import SIZE, box, header
from mechanics import (Site, FIELD, DT, SUBSTEPS, GOAL_TOLERANCE, YAW_TOLERANCE,
                       STOP_SPEED, STOP_YAW, accelerations, docked, wrap)
from scene_assets import decorate, sync_visuals
from pbr_renderer import Renderer

KIND = 'cross'
LEG, SCOUT_HALF, SCOUT_MASS = .025, .13, 1.
SCOUT_ACTIONS, CARRIER_ACTIONS, MIN_SCOUT_ACTIONS = 500, 240, 12
DIRECTIONS = ((1, 0), (-1, 0), (0, 1), (0, -1))
BUTTONS = (0., 1/3, 2/3, 1.)
CAMERAS = ('ego_front', 'ego_rear')


def png(image):
    buffer = io.BytesIO()
    Image.fromarray(image).save(buffer, format='PNG')
    return base64.b64encode(buffer.getvalue()).decode()


class World:
    def __init__(self, seed, render=True):
        self.seed, self.site = int(seed), Site(seed)
        control = np.random.default_rng([self.seed % (2**64), 0xB0770003])
        self.mappings = [control.permutation(4).tolist()]
        self.gains = [float(control.uniform(.85, 1.15))]
        self.axes = control.permutation(2).tolist()
        self.signs = control.choice([-1., 1.], size=2).tolist()
        self.render_enabled, self.phase, self.steps = render, 0, [0, 0]
        self.done = self.success = self.contact_stop = self.switched = self.bumper = False
        self.scout_contacts, self.frame_id, self.renderer = 0, 0, None
        self.steer, self.dock_steps = 0., 0
        self._build()

    @property
    def half(self):
        return (SCOUT_HALF+LEG,)*2 if self.phase == 0 else self.site.carrier_half

    def _build(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
        site = self.site
        xml = header()
        for prefix, rectangles in (('wall', site.partition), ('crate', site.crates), ('boundary', site.boundary())):
            for i, (cx, cy, hx, hy) in enumerate(rectangles):
                xml += box(f'{prefix}{i}', f'{cx} {cy} .35', f'{hx} {hy} .35', '.55 .6 .65 1')
        gx, gy = site.goal
        xml += f'<body name="dock" pos="{gx} {gy} .005" euler="0 0 {math.degrees(site.goal_yaw)}">'
        xml += box('goal', '0 0 0', '1.12 .55 .004', '.03 .12 1 1', False)
        xml += box('dock_nose', '1.05 0 .006', '.07 .55 .002', '1 .85 .05 1', False)
        px, py = site.carrier_start
        xml += f'</body><body name="parking" pos="{px} {py} .004" euler="0 0 {math.degrees(site.carrier_yaw)}">'
        xml += box('parking_pad', '0 0 0', '.35 .23 .003', '.08 .55 .18 1', False)
        xml += box('parking_nose', '.28 0 .006', '.05 .18 .003', '.8 1 .8 1', False)
        xml += '</body><body name="agent" pos="0 0 .23">'
        xml += '<joint name="x" type="slide" axis="1 0 0" damping="0"/>'
        xml += '<joint name="y" type="slide" axis="0 1 0" damping="0"/>'
        # Short body-mounted masts, 25 degrees down from horizontal. No chase
        # camera, world-fixed crop, visibility disk or absolute-position metadata.
        cam_x, cam_z = (.30, .40) if self.phase else (.10, .27)
        xml += f'<camera name="ego_front" pos="{cam_x} 0 {cam_z}" xyaxes="0 -1 0 .422618 0 .906308" fovy="85"/>'
        xml += f'<camera name="ego_rear" pos="{-cam_x} 0 {cam_z}" xyaxes="0 1 0 -.422618 0 .906308" fovy="85"/>'
        if self.phase:
            xml += '<joint name="yaw" type="hinge" axis="0 0 1" damping="0"/>'
            a, b = site.carrier_half
            xml += '<geom name="robot" type="box" size=".28 .18 .06" mass="1" rgba="1 0 0 1"/>'
            xml += f'<body name="payload" pos="0 0 .15"><geom name="load" type="box" size="{a} {b} .09" mass="{site.carrier_mass-1}" rgba=".8 .7 .2 1"/></body>'
        else:
            xml += '<geom name="robot" type="box" size=".155 .155 .06" mass="1.08" rgba="1 0 0 1"/>'
        xml += '</body></worldbody></mujoco>'
        if self.render_enabled:
            xml = decorate(xml, KIND, .38 if self.phase else SCOUT_HALF, seed=site.look_seed)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.agent_id = self.model.body('agent').id
        self.robot_bodies = {self.agent_id}
        for bid in range(self.agent_id+1, self.model.nbody):
            if int(self.model.body_parentid[bid]) in self.robot_bodies:
                self.robot_bodies.add(bid)
        self.data.qpos[:2] = site.carrier_start if self.phase else site.scout_start
        if self.phase:
            self.data.qpos[2] = site.carrier_yaw
        mujoco.mj_forward(self.model, self.data)
        matrix = np.zeros((self.model.nv, self.model.nv))
        mujoco.mj_fullM(self.model, self.data, matrix)
        self.inertia = np.diag(matrix).copy()
        self._contacts = set()
        if self.render_enabled:
            self.renderer = Renderer(self.model, SIZE, SIZE)

    def position(self):
        return self.data.qpos[:2].copy()

    def state(self):
        return (*self.data.qpos[:3], *self.data.qvel[:3], self.steer)

    def _touching(self):
        touching = set()
        for contact in self.data.contact[:self.data.ncon]:
            b1, b2 = (int(self.model.geom_bodyid[g]) for g in (contact.geom1, contact.geom2))
            if (b1 in self.robot_bodies) != (b2 in self.robot_bodies):
                other = contact.geom2 if b1 in self.robot_bodies else contact.geom1
                if self.model.geom(other).name != 'floor':
                    touching.add(int(other))
        return touching

    def _contact(self, touching):
        self.bumper |= bool(touching)
        if self.phase == 0:
            self.scout_contacts += len(touching-self._contacts)
            self._contacts = touching
        elif touching:
            self.data.qvel[:] = 0
            self.contact_stop = True
        return self.contact_stop

    def image(self, camera='ego_front'):
        sync_visuals(self.model, self.data)
        option = mujoco.MjvOption()
        option.geomgroup[3] = 0
        self.renderer.update_scene(self.data, camera=camera, scene_option=option)
        return self.renderer.render().copy()

    def observe(self):
        self.frame_id += 1
        obs = dict(frame_id=self.frame_id, phase=self.phase, done=self.done,
                   steps_remaining=self.remaining(), bumper=self.bumper, contact_stop=self.contact_stop)
        if self.render_enabled:
            obs['images'] = [png(self.image(camera)) for camera in CAMERAS]
        return obs

    def remaining(self):
        return max(0, (SCOUT_ACTIONS, CARRIER_ACTIONS)[self.phase]-self.steps[self.phase])

    def act(self, action):
        if self.done:
            raise ValueError('the mission has ended')
        length = 2 if self.phase else 1
        if (not isinstance(action, list) or len(action) != length or
                any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in action)):
            raise ValueError('scout: [button]; carrier: [channel0, channel1]')
        if self.phase:
            if any(abs(v) > 1 for v in action):
                raise ValueError('carrier channels must be within [-1, 1]')
            command = [self.signs[i]*action[self.axes[i]] for i in range(2)]
        else:
            index = next((i for i, b in enumerate(BUTTONS) if abs(action[0]-b) < 1e-6), None)
            if index is None:
                raise ValueError('scout buttons are 0, 1/3, 2/3 and 1')
            target = np.array(DIRECTIONS[self.mappings[0][index]])*self.gains[0]*1.2
        if self.remaining() == 0:
            raise ValueError('action budget spent; advance() after scouting')
        self.steps[self.phase] += 1
        self.bumper = False
        for _ in range(SUBSTEPS):
            if self.phase:
                acc, self.steer = accelerations(float(self.data.qpos[2]), self.data.qvel[:3],
                                               self.steer, command, self.site.dynamics)
                self.data.qfrc_applied[:3] = self.inertia*np.asarray(acc)
            else:
                self.data.qfrc_applied[:2] = 35*(target-self.data.qvel[:2])-5*self.data.qvel[:2]
            mujoco.mj_step(self.model, self.data)
            mujoco.mj_forward(self.model, self.data)
            if self._contact(self._touching()):
                break
        if self.phase:
            self.dock_steps = self.dock_steps+1 if (not self.contact_stop and docked(self.state(),
                               (*self.site.goal, self.site.goal_yaw))) else 0
            self.success = self.dock_steps >= 3
            self.done = bool(self.success or self.contact_stop or self.remaining() == 0)
        return self.observe()

    def advance(self):
        if self.done or self.phase != 0 or self.steps[0] < MIN_SCOUT_ACTIONS:
            raise ValueError('switch once, after at least 12 scout actions')
        self.phase, self.switched, self.bumper = 1, True, False
        self._build()
        return self.observe()

    def metrics(self):
        return dict(success=float(self.success), switched=float(self.switched),
                    scout_actions=self.steps[0], carrier_actions=self.steps[1], scout_contacts=self.scout_contacts,
                    contact_stop=float(self.contact_stop),
                    final_goal_distance=float(np.linalg.norm(self.position()-self.site.goal)) if self.phase else -1.,
                    final_heading_error=abs(wrap(self.data.qpos[2]-self.site.goal_yaw)) if self.phase else -1.,
                    final_speed=float(np.linalg.norm(self.data.qvel[:2])) if self.phase else -1.)

    def truth(self):
        s = self.site
        t = dict(version=4, phase=self.phase, position=self.position().tolist(),
                 velocity=self.data.qvel.tolist(), obstacles=[list(r) for r in s.obstacles()],
                 half=list(self.half), carrier_half=list(s.carrier_half),
                 carrier_start=list(s.carrier_start), carrier_yaw=s.carrier_yaw,
                 goal=list(s.goal), goal_yaw=s.goal_yaw, steps=list(self.steps),
                 timestep=DT, substeps=SUBSTEPS)
        if self.phase:
            t.update(state=list(self.state()), dynamics=s.dynamics, axes=self.axes, signs=self.signs)
        else:
            t.update(buttons=[list(DIRECTIONS[m]) for m in self.mappings[0]], gain=self.gains[0])
        return t

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
