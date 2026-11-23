"""cable_env: a cable-routing cell in MuJoCo.

A cable leaves a fixed connector on a fixture board and lies on the board in a random shape.
A parallel gripper, commanded in Cartesian space, has to route it through a sequence of
fixtures: pin-pair clips the cable must pass between, and posts it must wrap around on a given
side. Everything about an episode is a pure function of its 64-bit seed.

Two byte-identical copies exist: the grader's (tests/cable_env.py) and the agent's development
copy (~/cell/cable_env.py). Editing the agent's copy changes what the agent develops against,
never what it is scored on.

Frames and units: board plane z = 0, x to the right, y away from the operator, z up; metres,
radians, seconds, newtons.
"""

import math

import numpy as np

import mujoco

__all__ = ["CableRouting", "Episode", "make_episode", "CONTROL_DT", "MAX_STEPS"]

PHYS_DT = 0.002
CONTROL_DT = 0.1
SUBSTEPS = int(round(CONTROL_DT / PHYS_DT))
MAX_STEPS = 1200                      # 120 s of control
BOARD = (0.42, 0.30)                  # half extents of the usable board area
N_SEG = 40
WORKSPACE = ((-0.46, 0.46), (-0.34, 0.34), (0.0, 0.25))
MAX_SPEED = 0.30                      # m/s of the tool centre point
MAX_YAW_RATE = 2.5                    # rad/s
FINGER_SPEED = 0.08                   # m/s of finger travel
FINGER_OPEN = 0.020                   # finger joint position fully open
FINGER_CLOSED = -0.012                # finger joint command when closed
PIN_R = 0.0025
PIN_H = 0.016
POST_R = 0.012
POST_H = 0.030
SEATED_Z = 0.013                      # a clip counts when the cable crosses it below this height
LATCH_Z = 0.007                       # a clip snaps shut when the cable is pushed down below this
POST_CAP_R = 0.018                    # mushroom cap on top of a post
FIN_HALF = 0.011                      # half width of a clip's 45-degree lead-in fin
POST_TOL = 0.010                      # cable within this of the post surface counts as wrapped
CONNECTOR_LIMIT = 40.0                # N; above it for CONNECTOR_HOLD the connector is damaged
CONNECTOR_HOLD = 0.20                 # s
TRACKER_NOISE = 0.001                 # m, per keypoint coordinate
SETTLE_S = 2.0


class Episode(object):
    """Everything an episode is made of (drawn from its seed by `make_episode`)."""


def _rot(a):
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def _seg_dist(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


def make_episode(seed):
    rng = np.random.default_rng(np.random.SeedSequence(int(seed) & ((1 << 64) - 1)))
    ep = Episode()
    ep.seed = int(seed)
    ep.length = float(rng.uniform(0.85, 1.05))
    ep.radius = float(rng.uniform(0.0035, 0.0050))
    # bending stiffness EI of a flexible 7-10 mm cable, as the plugin's Young's modulus
    ep.EI = float(rng.uniform(0.0008, 0.003))
    ep.bend = ep.EI / (math.pi * ep.radius ** 4 / 4.0)
    ep.twist = ep.bend / 2.6            # shear modulus of an isotropic jacket (nu ~ 0.3)
    ep.friction = float(rng.uniform(0.5, 1.0))
    ep.density = float(rng.uniform(1200.0, 2000.0))
    hx, hy = BOARD
    for _attempt in range(500):
        n = int(rng.integers(3, 6))
        ay = float(rng.uniform(-0.18, 0.18))
        conn0 = np.array([-hx + 0.03, ay])
        heading = float(rng.uniform(-0.5, 0.5))
        pts = [conn0.copy()]
        fixtures = []
        cur = conn0 + 0.02 * np.array([math.cos(heading), math.sin(heading)])
        d_in = heading
        ok = True
        total = 0.02
        for k in range(n):
            step = float(rng.uniform(0.12, 0.19))
            nxt = cur + step * np.array([math.cos(d_in), math.sin(d_in)])
            total += step
            # a post always follows a clip (no back-to-back posts)
            kind = "clip" if (k == 0 or fixtures[-1]["type"] == "post" or rng.uniform() < 0.6) else "post"
            if kind == "clip":
                turn = float(rng.uniform(-0.6, 0.6))
            else:
                turn = float(rng.choice([-1, 1]) * rng.uniform(0.7, 1.3))
            d_out = d_in + turn
            if kind == "clip":
                axis = 0.5 * (d_in + d_out)                     # passage direction at the clip
                gap = 2 * ep.radius + float(rng.uniform(0.0025, 0.0040))
                fixtures.append({"type": "clip", "pos": nxt.copy(), "axis": axis, "gap": gap})
            else:
                side = "left" if turn > 0 else "right"
                # the post sits on the inner side of the bend, tangent to the route corner
                inner = d_in + turn / 2.0 + (math.pi / 2 if turn > 0 else -math.pi / 2)
                centre = nxt + (POST_R + ep.radius) * np.array([math.cos(inner), math.sin(inner)])
                fixtures.append({"type": "post", "pos": centre, "corner": nxt.copy(), "side": side})
            pts.append(nxt.copy())
            cur, d_in = nxt, d_out
        end = cur + 0.05 * np.array([math.cos(d_in), math.sin(d_in)])
        pts.append(end)
        total += 0.05
        if total + 0.12 > ep.length:
            continue
        P = np.array([f["pos"] for f in fixtures])
        if np.any(np.abs(P[:, 0]) > hx - 0.05) or np.any(np.abs(P[:, 1]) > hy - 0.05):
            continue
        if np.any(np.abs(np.array(pts)[:, 0]) > hx) or np.any(np.abs(np.array(pts)[:, 1]) > hy):
            continue
        dmin = min(np.linalg.norm(P[i] - P[j]) for i in range(n) for j in range(i + 1, n)) if n > 1 else 1.0
        if dmin < 0.09:
            continue
        # no route leg may pass near a fixture other than its own two ends
        for i in range(len(pts) - 1):
            for j, f in enumerate(fixtures):
                if j in (i - 1, i):
                    continue
                if _seg_dist(f["pos"], pts[i], pts[i + 1]) < 0.04:
                    ok = False
        if not ok:
            continue
        ep.conn_pos = conn0
        ep.conn_heading = heading
        ep.fixtures = fixtures
        ep.route = np.array(pts)
        ep.initial = _initial_shape(rng, ep)
        if ep.initial is not None:
            break
    else:
        raise RuntimeError("no route found for seed %d" % seed)
    ep.rng = rng
    return ep


def _initial_shape(rng, ep):
    """A smooth random planar curve from the connector that keeps clear of every fixture."""
    seg = ep.length / N_SEG
    hx, hy = BOARD
    P = np.array([f["pos"] for f in ep.fixtures])
    for _attempt in range(400):
        # the cable leaves the connector along its axis, then wanders off in a random direction
        h = ep.conn_heading
        bend_to = ep.conn_heading + float(rng.uniform(-2.2, 2.2))
        pts = [ep.conn_pos.copy()]
        curv = 0.0
        good = True
        for i in range(N_SEG):
            if 3 <= i < 9:
                h += (bend_to - ep.conn_heading) / 6.0
            elif i >= 9:
                curv = 0.85 * curv + float(rng.normal(0.0, 0.12))
                h += curv
            nxt = pts[-1] + seg * np.array([math.cos(h), math.sin(h)])
            if abs(nxt[0]) > hx - 0.01 or abs(nxt[1]) > hy - 0.01:
                good = False
                break
            if np.min(np.linalg.norm(P - nxt, axis=1)) < 0.035:
                good = False
                break
            # no self-crossing: keep a distance from all but the last few points
            if len(pts) > 4 and np.min(np.linalg.norm(np.array(pts[:-3]) - nxt, axis=1)) < 0.03:
                good = False
                break
            pts.append(nxt)
        if good:
            return np.array(pts)
    return None


def _xml(ep):
    R = ep.radius
    # The cable plugin takes the model's initial shape as the cable's rest shape, so the cable
    # is built straight along the connector axis (a cable with no memory of a curl) and bent
    # into its starting layout through its joints in `reset`. Composite vertices are offsets in
    # world axes (the composite ignores its body's rotation).
    seg = ep.length / N_SEG
    ch, sh = math.cos(ep.conn_heading), math.sin(ep.conn_heading)
    verts = " ".join("%.6f %.6f 0" % (i * seg * ch, i * seg * sh) for i in range(N_SEG + 1))
    fx = []
    for i, f in enumerate(ep.fixtures):
        x, y = f["pos"]
        if f["type"] == "clip":
            # a fork clip in its own frame: x along the cable's passage, y across the slot
            off = f["gap"] / 2 + PIN_R
            st = 'friction="0.3" solref="0.003 1" solimp="0.95 0.99 0.0005"'
            fx.append('<body name="clip%d" pos="%f %f 0" euler="0 0 %f">' % (i, x, y, f["axis"]))
            for sgn, name in ((1, "a"), (-1, "b")):
                fx.append('<geom name="fix%d%s" type="capsule" fromto="0 %f 0 0 %f %f" size="%f" rgba=".18 .2 .24 1" %s/>'
                          % (i, name, sgn * off, sgn * off, PIN_H - PIN_R, PIN_R, st))
                # lead-in fin rising outward at 45 degrees from the pin top
                fx.append('<geom name="fin%d%s" type="box" size="0.006 %f 0.001" pos="0 %f %f" euler="%f 0 0" '
                          'rgba=".25 .28 .33 1" %s/>' % (i, name, FIN_HALF, sgn * (off + FIN_HALF * 0.7071),
                                                        PIN_H - 0.002 + FIN_HALF * 0.7071, sgn * 0.7854, st))
            # the latch bar across the slot top: on its own collision bit (2), which the cable ignores,
            # and invisible, until the clip snaps shut
            fx.append('<geom name="latch%d" type="capsule" fromto="0 %f %f 0 %f %f" size="%f" rgba=".18 .2 .24 0" '
                      'contype="2" conaffinity="2" %s/>' % (i, off, PIN_H - PIN_R, -off, PIN_H - PIN_R, PIN_R, st))
            fx.append('</body>')
        else:
            fx.append('<geom name="fix%d" type="cylinder" pos="%f %f %f" size="%f %f" rgba=".35 .3 .25 1" '
                      'friction="0.3" solref="0.003 1" solimp="0.95 0.99 0.0005"/>' % (i, x, y, POST_H / 2, POST_R, POST_H / 2))
            fx.append('<geom name="cap%d" type="cylinder" pos="%f %f %f" size="%f 0.003" rgba=".35 .3 .25 1" '
                      'friction="0.3" solref="0.003 1" solimp="0.95 0.99 0.0005"/>' % (i, x, y, POST_H + 0.003, POST_CAP_R))
    return """
<mujoco model="cable_routing">
  <compiler angle="radian"/>
  <extension><plugin plugin="mujoco.elasticity.cable"/></extension>
  <option timestep="{dt}" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <size memory="64M"/>
  <visual><global offwidth="960" offheight="720"/><quality shadowsize="2048"/></visual>
  <worldbody>
    <light pos="0 -0.3 1.2" dir="0 0.25 -1" diffuse=".9 .9 .9"/>
    <geom name="board" type="plane" size="0.5 0.36 0.01" rgba=".82 .83 .80 1" friction="{fr} 0.01 0.001"/>
    <body name="connector" pos="{ax} {ay} {z0}">
      <geom type="box" size="0.012 0.009 0.008" pos="{cx} {cy} 0.004" euler="0 0 {hd}" rgba=".12 .12 .14 1" contype="0" conaffinity="0"/>
      <composite prefix="c" type="cable" vertex="{verts}" initial="ball">
        <plugin plugin="mujoco.elasticity.cable">
          <config key="twist" value="{tw}"/>
          <config key="bend" value="{bd}"/>
          <config key="vmax" value="0"/>
        </plugin>
        <joint kind="main" damping="0.003" armature="1e-5"/>
        <geom type="capsule" size="{R}" rgba=".85 .3 .1 1" condim="4" friction="{fr} 0.01 0.001" density="{rho}" solref="0.004 1" solimp="0.95 0.99 0.0005"/>
      </composite>
    </body>
    {fixtures}
    <body name="tcp_target" mocap="true" pos="0 -0.25 0.15"/>
    <body name="gripper" pos="0 -0.25 0.15">
      <freejoint/>
      <geom type="box" size="0.015 0.03 0.01" pos="0 0 0.045" rgba=".3 .32 .36 1" mass="0.3" contype="0" conaffinity="0"/>
      <body name="finger_l" pos="0 0.012 0">
        <joint name="jl" type="slide" axis="0 1 0" range="-0.013 0.021" damping="2"/>
        <geom name="pad_l" type="box" size="0.008 0.003 0.013" pos="0 0 0.013" rgba=".55 .56 .6 1" friction="1.5 0.02 0.002" condim="6" mass="0.03" solref="0.003 1" solimp="0.95 0.99 0.0005"/>
      </body>
      <body name="finger_r" pos="0 -0.012 0">
        <joint name="jr" type="slide" axis="0 -1 0" range="-0.013 0.021" damping="2"/>
        <geom name="pad_r" type="box" size="0.008 0.003 0.013" pos="0 0 0.013" rgba=".55 .56 .6 1" friction="1.5 0.02 0.002" condim="6" mass="0.03" solref="0.003 1" solimp="0.95 0.99 0.0005"/>
      </body>
    </body>
  </worldbody>
  <equality>
    <weld body1="tcp_target" body2="gripper" solref="0.01 1"/>
  </equality>
  <actuator>
    <position name="act_l" joint="jl" kp="1200" kv="20" ctrlrange="{fc} {fo}" forcerange="-20 20"/>
    <position name="act_r" joint="jr" kp="1200" kv="20" ctrlrange="{fc} {fo}" forcerange="-20 20"/>
  </actuator>
</mujoco>
""".format(dt=PHYS_DT, fr=ep.friction, ax=ep.conn_pos[0], ay=ep.conn_pos[1], z0=R, verts=verts, hd=ep.conn_heading,
           cx=-0.012 * math.cos(ep.conn_heading), cy=-0.012 * math.sin(ep.conn_heading),
           tw=ep.twist, bd=ep.bend, R=R, rho=ep.density, fixtures="\n    ".join(fx),
           fc=FINGER_CLOSED, fo=FINGER_OPEN)


class CableRouting(object):
    """One cell. `reset(seed)` builds the episode; `step(action)` advances 0.1 s."""

    max_steps = MAX_STEPS

    def __init__(self):
        self.model = None
        self.data = None

    # -- construction -----------------------------------------------------------------
    def reset(self, seed):
        self.ep = make_episode(seed)
        for _attempt in range(8):
            obs = self._build(seed)
            P = self.cable_points()
            F = np.array([f["pos"] for f in self.ep.fixtures])
            clear = np.min(np.linalg.norm(P[:, None, :2] - F[None, :, :], axis=2))
            if clear > 0.02 and P[:, 2].max() < 0.008:
                return obs
            nxt = _initial_shape(self.ep.rng, self.ep)
            if nxt is None:
                break
            self.ep.initial = nxt
        return obs

    def _build(self, seed):
        spec = mujoco.MjSpec.from_string(_xml(self.ep))
        site = spec.body("cB_first").add_site()
        site.name = "conn_site"
        sens = spec.add_sensor()
        sens.name = "connector_force"
        sens.type = mujoco.mjtSensor.mjSENS_FORCE
        sens.objtype = mujoco.mjtObj.mjOBJ_SITE
        sens.objname = "conn_site"
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        m, d = self.model, self.data
        self.seg_ids = [i for i in range(m.nbody) if m.body(i).name.startswith("cB")]
        self.seg_len = self.ep.length / N_SEG
        self.mocap = m.body("tcp_target").mocapid[0]
        self.grip_body = m.body("gripper").id
        self.jl = m.jnt_qposadr[m.joint("jl").id]
        self.jr = m.jnt_qposadr[m.joint("jr").id]
        self.sensor_adr = m.sensor_adr[m.sensor("connector_force").id]
        self.noise = np.random.default_rng(np.random.SeedSequence([int(seed) & ((1 << 64) - 1), 7]))
        self.target = np.array([0.0, -0.25, 0.15])
        self.yaw = 0.0
        self.target_yaw = 0.0
        self.grip_cmd = 0.0
        # bend the straight cable into the episode's starting layout
        P0 = self.ep.initial
        heads = np.arctan2(np.diff(P0[:, 1]), np.diff(P0[:, 0]))
        cable_joints = [j for j in range(m.njnt) if m.joint(j).name.startswith("cJ_")]
        mujoco.mj_forward(m, d)
        # the world's vertical in the (straight, so common) frame of the cable bodies
        up = d.xmat[self.seg_ids[0]].reshape(3, 3).T @ np.array([0.0, 0.0, 1.0])
        for j in cable_joints:
            i = self.seg_ids.index(m.jnt_bodyid[j])           # the segment this joint turns
            before = heads[i - 1] if i > 0 else self.ep.conn_heading
            dh = (heads[i] - before + math.pi) % (2 * math.pi) - math.pi
            adr = m.jnt_qposadr[j]
            d.qpos[adr:adr + 4] = np.concatenate([[math.cos(dh / 2)], math.sin(dh / 2) * up])
        self.finger_q = FINGER_OPEN
        d.ctrl[:] = FINGER_OPEN
        mujoco.mj_forward(m, d)
        self.latch_ids = {}
        for i, f in enumerate(self.ep.fixtures):
            if f["type"] == "clip":
                self.latch_ids[i] = m.geom("latch%d" % i).id
        self.latched = set()
        self.steps = 0
        self.load_avg = self.connector_force()
        self.over_limit = 0.0
        self.peak_force = 0.0
        self.damaged = False
        # let the cable settle on the board
        for _ in range(int(1.5 / PHYS_DT)):
            mujoco.mj_step(m, d)
        return self.observe()

    # -- state --------------------------------------------------------------------------
    def cable_points(self):
        d = self.data
        pts = [d.xpos[i].copy() for i in self.seg_ids]
        last = self.seg_ids[-1]
        R = d.xmat[last].reshape(3, 3)
        pts.append(d.xpos[last] + R[:, 0] * self.seg_len)
        return np.array(pts)

    def gripper_state(self):
        d = self.data
        pos = d.xpos[self.grip_body].copy()
        R = d.xmat[self.grip_body].reshape(3, 3)
        yaw = math.atan2(R[1, 0], R[0, 0])
        width = (0.012 + d.qpos[self.jl]) + (0.012 + d.qpos[self.jr]) - 0.006
        force = float(abs(d.actuator_force[0]) + abs(d.actuator_force[1])) / 2.0
        return pos, yaw, max(width, 0.0), force

    def observe(self):
        pts = self.cable_points() + self.noise.normal(0.0, TRACKER_NOISE, size=(N_SEG + 1, 3))
        pos, yaw, width, force = self.gripper_state()
        fixtures = []
        for f in self.ep.fixtures:
            if f["type"] == "clip":
                fixtures.append({"type": "clip", "pos": [round(float(f["pos"][0]), 5), round(float(f["pos"][1]), 5)],
                                 "axis": round(float(f["axis"]), 5), "gap": round(float(f["gap"]), 5),
                                 "latched": len(fixtures) in self.latched})
            else:
                fixtures.append({"type": "post", "pos": [round(float(f["pos"][0]), 5), round(float(f["pos"][1]), 5)],
                                 "radius": POST_R, "side": f["side"]})
        return {
            "time": round(self.steps * CONTROL_DT, 3),
            "gripper": {"pos": [round(float(v), 5) for v in pos], "yaw": round(float(yaw), 5),
                        "width": round(float(width), 5), "grip_force": round(force, 3)},
            "cable": [[round(float(v), 5) for v in p] for p in pts],
            "cable_radius": round(self.ep.radius, 5),
            "cable_length": round(self.ep.length, 5),
            "connector": {"pos": [round(float(self.ep.conn_pos[0]), 5), round(float(self.ep.conn_pos[1]), 5)],
                          "heading": round(float(self.ep.conn_heading), 5)},
            "fixtures": fixtures,
            "connector_force": round(self.load_avg, 3),
        }

    def connector_force(self):
        f = self.data.sensordata[self.sensor_adr:self.sensor_adr + 3]
        return float(np.linalg.norm(f))

    # -- stepping -----------------------------------------------------------------------
    def set_command(self, pos, yaw, grip):
        lo = np.array([w[0] for w in WORKSPACE])
        hi = np.array([w[1] for w in WORKSPACE])
        self.target = np.clip(np.asarray(pos, float), lo, hi)
        self.target_yaw = float(yaw)
        self.grip_cmd = float(min(1.0, max(0.0, grip)))

    def _substep(self):
        m, d = self.model, self.data
        cur = d.mocap_pos[self.mocap].copy()
        delta = self.target - cur
        dist = float(np.linalg.norm(delta))
        stepmax = MAX_SPEED * PHYS_DT
        if dist > stepmax:
            delta *= stepmax / dist
        d.mocap_pos[self.mocap] = cur + delta
        dy = (self.target_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
        dy = max(-MAX_YAW_RATE * PHYS_DT, min(MAX_YAW_RATE * PHYS_DT, dy))
        self.yaw += dy
        d.mocap_quat[self.mocap] = [math.cos(self.yaw / 2), 0.0, 0.0, math.sin(self.yaw / 2)]
        q = FINGER_OPEN + (FINGER_CLOSED - FINGER_OPEN) * self.grip_cmd
        dq = max(-FINGER_SPEED * PHYS_DT, min(FINGER_SPEED * PHYS_DT, q - self.finger_q))
        self.finger_q += dq
        d.ctrl[0] = d.ctrl[1] = self.finger_q
        mujoco.mj_step(m, d)
        f = self.connector_force()
        self.peak_force = max(self.peak_force, f)
        if f > CONNECTOR_LIMIT:
            self.over_limit += PHYS_DT
            if self.over_limit >= CONNECTOR_HOLD:
                self.damaged = True
        else:
            self.over_limit = 0.0

    def _update_latches(self):
        """A clip snaps shut once the cable has been pushed fully down between its pins."""
        if len(self.latched) == len(self.latch_ids):
            return
        P = self.cable_points()
        for i, gid in self.latch_ids.items():
            if i in self.latched:
                continue
            f = self.ep.fixtures[i]
            a = f["axis"]
            n = np.array([-math.sin(a), math.cos(a)]) * (f["gap"] / 2)
            if _crossing(P, f["pos"] - n, f["pos"] + n, LATCH_Z) is not None:
                self.latched.add(i)
                # compiled on its own collision bit (2), which the cable ignores; add bit 1 to close it
                self.model.geom_conaffinity[gid] = 3
                self.model.geom_rgba[gid, 3] = 1.0

    def step(self, action):
        """action: dict with pos [x, y, z], yaw, grip (0 open .. 1 closed), optional done."""
        self.set_command(action["pos"], action["yaw"], action["grip"])
        acc = 0.0
        for k in range(SUBSTEPS):
            self._substep()
            acc += self.connector_force()
            if k % 10 == 9:
                self._update_latches()
        self.load_avg = acc / SUBSTEPS
        self.steps += 1
        unstable = not np.all(np.isfinite(self.data.qpos))
        done = bool(action.get("done", False)) or self.steps >= MAX_STEPS or unstable
        return self.observe() if not unstable else None, done

    # -- scoring ------------------------------------------------------------------------
    def finish(self):
        """Open the gripper, lift it clear, let the cable settle, then evaluate."""
        if not np.all(np.isfinite(self.data.qpos)):
            return {"credit": 0.0, "routed": 0, "fixtures": len(self.ep.fixtures), "unstable": True,
                    "damaged": self.damaged, "peak_connector_force": self.peak_force, "checks": []}
        pos, _, _, _ = self.gripper_state()
        self.set_command(pos, self.target_yaw, 0.0)
        for _ in range(int(0.4 / PHYS_DT)):
            self._substep()
        self.set_command([pos[0], pos[1], 0.15], self.target_yaw, 0.0)
        for _ in range(int(SETTLE_S / PHYS_DT)):
            self._substep()
        return evaluate(self.ep, self.cable_points(), self.damaged, self.peak_force)


def _crossing(P, a, b, zmax):
    """Arc length at which polyline P first crosses segment a-b in the plane below zmax."""
    s = 0.0
    for j in range(len(P) - 1):
        p, q = P[j, :2], P[j + 1, :2]
        r, w = q - p, b - a
        den = r[0] * w[1] - r[1] * w[0]
        ln = float(np.linalg.norm(r))
        if abs(den) > 1e-12:
            t = ((a[0] - p[0]) * w[1] - (a[1] - p[1]) * w[0]) / den
            u = ((a[0] - p[0]) * r[1] - (a[1] - p[1]) * r[0]) / den
            if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
                z = P[j, 2] + t * (P[j + 1, 2] - P[j, 2])
                if z < zmax:
                    return s + t * ln
        s += ln
    return None


def evaluate(ep, P, damaged=False, peak_force=0.0):
    """Which fixtures the settled cable satisfies, in route order. P: (N+1, 3) cable points."""
    checks = []
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    for f in ep.fixtures:
        if f["type"] == "clip":
            a = f["axis"]
            off = f["gap"] / 2 + PIN_R
            n = np.array([-math.sin(a), math.cos(a)]) * off
            s = _crossing(P, f["pos"] - n, f["pos"] + n, SEATED_Z)
            checks.append({"type": "clip", "ok": s is not None, "s": s})
        else:
            c = f["pos"]
            dxy = P[:, :2] - c
            dist = np.linalg.norm(dxy, axis=1)
            j = int(np.argmin(dist))
            touching = dist[j] <= POST_R + ep.radius + POST_TOL and P[j, 2] < POST_H
            jj = min(max(j, 1), len(P) - 2)
            tang = P[jj + 1, :2] - P[jj - 1, :2]
            cross = tang[0] * (c[1] - P[j, 1]) - tang[1] * (c[0] - P[j, 0])
            side_ok = (cross > 0) if f["side"] == "left" else (cross < 0)
            ok = bool(touching and side_ok)
            checks.append({"type": "post", "ok": ok, "s": float(arc[j]) if ok else None})
    routed = 0
    last = -1.0
    for c in checks:
        if not c["ok"] or c["s"] <= last:
            break
        routed += 1
        last = c["s"]
    credit = 0.0 if damaged else routed / float(len(ep.fixtures))
    return {"credit": credit, "routed": routed, "fixtures": len(ep.fixtures), "damaged": bool(damaged),
            "peak_connector_force": float(peak_force), "unstable": False,
            "checks": [{"type": c["type"], "ok": c["ok"]} for c in checks]}
