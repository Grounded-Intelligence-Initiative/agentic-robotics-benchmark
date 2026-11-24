"""cable_env: a cable-harness cell in MuJoCo.

A cable leaves a fixed connector on a harness board and lies on the board in a random shape, with a plug on its free
end. A parallel gripper, commanded in Cartesian space, has to route it through a sequence of fixtures (fork clips the
cable must be pushed down into, posts it must pass on a given side) and then mate the plug with the socket at the end
of the route. The cell is seen through two RGB-D cameras: a fixed one above the board and one on the gripper.
Everything about an episode is a pure function of its 64-bit seed.

Two byte-identical copies exist: the grader's (tests/cable_env.py) and the practice cell's
(environment/setup/runtime/cable_env.py), which a root-only service runs for the agent's practice episodes. The agent
drives that service through ~/cell/client.py and never sees this file.

Frames and units: board plane z = 0, x to the right, y away from the operator, z up; metres, radians, seconds, newtons.
"""

import math
import os
import secrets

import numpy as np

import mujoco

__all__ = ["CableRouting", "Episode", "make_episode", "evaluate", "write_frames", "shift_obs", "CONTROL_DT",
           "MAX_STEPS", "CAMERAS"]

PHYS_DT = 0.002
CONTROL_DT = 0.1
SUBSTEPS = int(round(CONTROL_DT / PHYS_DT))
MAX_STEPS = 1500                      # 150 s of control
BOARD = (0.42, 0.30)                  # half extents of the usable board area
N_SEG = 48
WORKSPACE = ((-0.46, 0.46), (-0.34, 0.34), (0.0, 0.25))
RIM = (0.48, 0.36)                    # inner faces of the board's 40 mm rim
MAX_SPEED = 0.30                      # m/s of the tool centre point
MAX_YAW_RATE = 2.5                    # rad/s
FINGER_SPEED = 0.08                   # m/s of finger travel
FINGER_OPEN = 0.020                   # finger joint position fully open
FINGER_CLOSED = -0.014                # finger joint command when closed (squeezes about 14 N)
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
SETTLE_S = 2.0
# the plug on the cable's free end (its own frame: x forward, out of the cable; y across; z up)
PLUG_L = 0.042                        # the front PLUG_IN of it enters the socket; grip it behind that
PLUG_IN = 0.014
PLUG_W = 0.014
PLUG_H = 0.012
PLUG_MASS = 0.012
BOOT_STIFFNESS = 0.03                 # N m / rad of the flexible boot between the cable and the plug (bending)
# the socket (its own frame: origin at the middle of the mouth on the board, x into the socket)
SOCK_CLEAR = 0.0008                   # per side, across the plug
SOCK_HEAD = 0.0020                    # above the plug
SOCK_DEPTH = PLUG_IN + 0.001
SOCK_WALL = 0.003
SOCK_LATCH = 0.0015                   # the plug latches once its nose is this close to the back wall
CH_LEN = 0.005                        # lead-in chamfers at the mouth: length, thickness, angle
CH_T = 0.0016
CH_A = 0.5236
CH_C, CH_S = math.cos(CH_A), math.sin(CH_A)
CREDIT_UNMATED = 0.4                  # the route's share of the credit when the plug is not mated
# appearance and sensing
CABLE_COLOURS = [(.85, .30, .10), (.08, .08, .09), (.45, .46, .48), (.10, .25, .70), (.92, .92, .90), (.70, .08, .08),
                 (.85, .70, .10)]
CAMERAS = {
    # name: (width, height, vertical field of view in degrees)
    "overhead": (640, 480, 45.0),
    "wrist": (320, 240, 70.0),
}
OVERHEAD_POS = (0.0, 0.0, 0.95)       # looking straight down; image x along +x, image up along +y
WRIST_POS = (-0.05, 0.0, 0.11)        # in the gripper frame, looking forward and down at 24 degrees from vertical
RGB_NOISE = 2.5                       # grey levels
DEPTH_NOISE = 0.0007                  # m
DEPTH_STEP = 0.0002                   # m


class Episode(object):
    """Everything an episode is made of (drawn from its seed by `make_episode`)."""


def _rot(a):
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def _seg_dist(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


def _unit(a):
    return np.array([math.cos(a), math.sin(a)])


def make_episode(seed):
    rng = np.random.default_rng(np.random.SeedSequence(int(seed) & ((1 << 64) - 1)))
    ep = Episode()
    ep.seed = int(seed)
    ep.radius = float(rng.uniform(0.0035, 0.0050))
    # bending stiffness EI of a flexible 7-10 mm cable, as the plugin's Young's modulus
    ep.EI = float(rng.uniform(0.0008, 0.003))
    ep.bend = ep.EI / (math.pi * ep.radius ** 4 / 4.0)
    ep.twist = ep.bend / 2.6            # shear modulus of an isotropic jacket (nu ~ 0.3)
    ep.friction = float(rng.uniform(0.5, 1.0))
    ep.density = float(rng.uniform(1200.0, 2000.0))
    ep.colour = CABLE_COLOURS[int(rng.integers(len(CABLE_COLOURS)))]
    ep.board_tint = float(rng.uniform(0.70, 0.88))
    ep.light = float(rng.uniform(0.7, 1.0))
    hx, hy = BOARD
    for _attempt in range(2000):
        n = int(rng.integers(3, 6))
        ay = float(rng.uniform(-0.18, 0.18))
        conn0 = np.array([-hx + 0.03, ay])
        heading = float(rng.uniform(-0.5, 0.5))
        pts = [conn0.copy()]
        fixtures = []
        cur = conn0 + 0.02 * _unit(heading)
        d_in = heading
        ok = True
        total = 0.02
        for k in range(n):
            step = float(rng.uniform(0.12, 0.18))
            nxt = cur + step * _unit(d_in)
            total += step
            # the route starts and ends with a clip (the one before the socket holds the cable for mating), and a post
            # always follows a clip
            kind = "clip" if (k == 0 or k == n - 1 or fixtures[-1]["type"] == "post" or rng.uniform() < 0.6) else "post"
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
                centre = nxt + (POST_R + ep.radius) * _unit(inner)
                fixtures.append({"type": "post", "pos": centre, "corner": nxt.copy(), "side": side})
            pts.append(nxt.copy())
            cur, d_in = nxt, d_out
        # the socket: its mouth on the route's last leg, facing back along it
        d_sock = float(rng.uniform(0.10, 0.15))
        mouth = cur + d_sock * _unit(d_in)
        total += d_sock + PLUG_L
        pts.append(mouth + SOCK_DEPTH * _unit(d_in))
        # the cable is cut to the route with some spare: enough to reach the socket along a real route, not so much
        # that it lies in loops
        ep.length = total + float(rng.uniform(0.10, 0.20))
        if not 0.80 <= ep.length <= 1.25:
            continue
        # the spare cable runs on past the socket when the cable lies relaxed through the last clip: the plug must
        # then still lie well inside the rim
        tip = mouth + (ep.length - total + PLUG_L + SOCK_DEPTH) * _unit(d_in)
        if abs(tip[0]) > 0.42 or abs(tip[1]) > 0.30:
            continue
        P = np.array([f["pos"] for f in fixtures] + [mouth])
        if np.any(np.abs(P[:, 0]) > hx - 0.05) or np.any(np.abs(P[:, 1]) > hy - 0.05):
            continue
        if np.any(np.abs(np.array(pts)[:, 0]) > hx) or np.any(np.abs(np.array(pts)[:, 1]) > hy):
            continue
        dmin = min(np.linalg.norm(P[i] - P[j]) for i in range(len(P)) for j in range(i + 1, len(P)))
        if dmin < 0.09:
            continue
        # no route leg may pass near a fixture other than its own two ends
        for i in range(len(pts) - 1):
            for j in range(len(P)):
                if j in (i - 1, i):
                    continue
                if _seg_dist(P[j], pts[i], pts[i + 1]) < 0.04:
                    ok = False
        if not ok:
            continue
        ep.conn_pos = conn0
        ep.conn_heading = heading
        ep.fixtures = fixtures
        ep.sock_pos = mouth
        ep.sock_heading = d_in
        ep.route = np.array(pts)
        ep.initial = _initial_shape(rng, ep)
        if ep.initial is not None:
            break
    else:
        raise RuntimeError("no route found for seed %d" % seed)
    ep.rng = rng
    return ep


def _obstacles(ep):
    return np.array([f["pos"] for f in ep.fixtures] + [ep.sock_pos + 0.008 * _unit(ep.sock_heading)])


def _initial_shape(rng, ep):
    """A smooth random planar curve from the connector that keeps clear of every fixture and the socket."""
    seg = ep.length / N_SEG
    hx, hy = BOARD
    P = _obstacles(ep)
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
            nxt = pts[-1] + seg * _unit(h)
            if abs(nxt[0]) > hx - 0.01 or abs(nxt[1]) > hy - 0.01:
                good = False
                break
            if np.min(np.linalg.norm(P - nxt, axis=1)) < 0.04:
                good = False
                break
            # no self-crossing: keep a distance from all but the last few points
            if len(pts) > 4 and np.min(np.linalg.norm(np.array(pts[:-3]) - nxt, axis=1)) < 0.03:
                good = False
                break
            pts.append(nxt)
        if good:
            # the plug, lying on in the direction of the last segment, must be on the board and clear as well
            tip = pts[-1] + (PLUG_L + 0.01) * _unit(h)
            if abs(tip[0]) > hx - 0.01 or abs(tip[1]) > hy - 0.01 or np.min(np.linalg.norm(P - tip, axis=1)) < 0.04:
                continue
            return np.array(pts)
    return None


def _xml(ep):
    R = ep.radius
    # The cable plugin takes the model's initial shape as the cable's rest shape, so the cable is built straight along
    # the connector axis (a cable with no memory of a curl) and bent into its starting layout through its joints in
    # `reset`. Composite vertices are offsets in world axes (the composite ignores its body's rotation).
    seg = ep.length / N_SEG
    ch, sh = math.cos(ep.conn_heading), math.sin(ep.conn_heading)
    verts = " ".join("%.6f %.6f 0" % (i * seg * ch, i * seg * sh) for i in range(N_SEG + 1))
    st = 'friction="0.3" solref="0.003 1" solimp="0.95 0.99 0.0005"'
    fx = []
    for i, f in enumerate(ep.fixtures):
        x, y = f["pos"]
        if f["type"] == "clip":
            # a fork clip in its own frame: x along the cable's passage, y across the slot
            off = f["gap"] / 2 + PIN_R
            fx.append('<body name="clip%d" pos="%f %f 0" euler="0 0 %f">' % (i, x, y, f["axis"]))
            for sgn, name in ((1, "a"), (-1, "b")):
                fx.append('<geom name="fix%d%s" type="capsule" fromto="0 %f 0 0 %f %f" size="%f" rgba=".18 .2 .24 1" %s/>'
                          % (i, name, sgn * off, sgn * off, PIN_H - PIN_R, PIN_R, st))
                # lead-in fin rising outward at 45 degrees from the pin top
                fx.append('<geom name="fin%d%s" type="box" size="0.006 %f 0.001" pos="0 %f %f" euler="%f 0 0" '
                          'rgba=".25 .28 .33 1" %s/>' % (i, name, FIN_HALF, sgn * (off + FIN_HALF * 0.7071),
                                                        PIN_H - 0.002 + FIN_HALF * 0.7071, sgn * 0.7854, st))
            # the latch bar across the slot top: on its own collision bit (2), which the cable ignores, and invisible,
            # until the clip snaps shut
            fx.append('<geom name="latch%d" type="capsule" fromto="0 %f %f 0 %f %f" size="%f" rgba=".18 .2 .24 0" '
                      'contype="2" conaffinity="2" %s/>' % (i, off, PIN_H - PIN_R, -off, PIN_H - PIN_R, PIN_R, st))
            fx.append('</body>')
        else:
            fx.append('<geom name="fix%d" type="cylinder" pos="%f %f %f" size="%f %f" rgba=".35 .3 .25 1" %s/>'
                      % (i, x, y, POST_H / 2, POST_R, POST_H / 2, st))
            fx.append('<geom name="cap%d" type="cylinder" pos="%f %f %f" size="%f 0.003" rgba=".35 .3 .25 1" %s/>'
                      % (i, x, y, POST_H + 0.003, POST_CAP_R, st))
    # the socket housing: two side walls, a roof and a back wall around a cavity open at the mouth, with 30-degree
    # chamfers at the mouth's sides and top
    hw = PLUG_W / 2 + SOCK_CLEAR
    ch_ = PLUG_H + SOCK_HEAD
    sx, sy = ep.sock_pos
    sock = ['<body name="socket" pos="%f %f 0" euler="0 0 %f">' % (sx, sy, ep.sock_heading)]
    for sgn, name in ((1, "l"), (-1, "r")):
        sock.append('<geom name="sock_wall%s" type="box" size="%f %f %f" pos="%f %f %f" rgba=".2 .22 .26 1" %s/>'
                    % (name, SOCK_DEPTH / 2, SOCK_WALL / 2, (ch_ + SOCK_WALL) / 2, SOCK_DEPTH / 2,
                       sgn * (hw + SOCK_WALL / 2), (ch_ + SOCK_WALL) / 2, st))
        # a 30-degree plate whose inner edge meets the cavity's side at the mouth and opens outward
        sock.append('<geom name="sock_chamf%s" type="box" size="%f %f %f" pos="%f %f %f" euler="0 0 %f" '
                    'rgba=".2 .22 .26 1" %s/>' % (name, CH_LEN / 2, CH_T / 2, (ch_ + SOCK_WALL) / 2,
                                                  -CH_LEN / 2 * CH_C, sgn * (hw + CH_LEN / 2 * CH_S + CH_T / 2 * CH_C),
                                                  (ch_ + SOCK_WALL) / 2, -sgn * CH_A, st))
    sock.append('<geom name="sock_roof" type="box" size="%f %f %f" pos="%f 0 %f" rgba=".2 .22 .26 1" %s/>'
                % (SOCK_DEPTH / 2, hw + SOCK_WALL, SOCK_WALL / 2, SOCK_DEPTH / 2, ch_ + SOCK_WALL / 2, st))
    sock.append('<geom name="sock_chamft" type="box" size="%f %f %f" pos="%f 0 %f" euler="0 %f 0" '
                'rgba=".2 .22 .26 1" %s/>' % (CH_LEN / 2, hw + SOCK_WALL, CH_T / 2, -CH_LEN / 2 * CH_C,
                                              ch_ + CH_LEN / 2 * CH_S + CH_T / 2 * CH_C, CH_A, st))
    sock.append('<geom name="sock_back" type="box" size="%f %f %f" pos="%f 0 %f" rgba=".2 .22 .26 1" %s/>'
                % (SOCK_WALL / 2, hw + SOCK_WALL, (ch_ + SOCK_WALL) / 2, SOCK_DEPTH + SOCK_WALL / 2,
                   (ch_ + SOCK_WALL) / 2, st))
    sock.append('</body>')
    b = ep.board_tint
    cr, cg, cb = ep.colour
    return """
<mujoco model="cable_harness">
  <compiler angle="radian"/>
  <extension><plugin plugin="mujoco.elasticity.cable"/></extension>
  <option timestep="{dt}" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <size memory="96M"/>
  <visual><global offwidth="640" offheight="480"/><headlight ambient="{amb} {amb} {amb}" diffuse="{hd} {hd} {hd}"/></visual>
  <worldbody>
    <light pos="0.2 -0.3 1.2" dir="-0.15 0.25 -1" diffuse="{li} {li} {li}" castshadow="false"/>
    <camera name="overhead" pos="{ox} {oy} {oz}" xyaxes="1 0 0 0 1 0" fovy="{ofov}"/>
    <geom name="board" type="plane" size="0.8 0.6 0.01" rgba="{b} {b} {b2} 1" friction="{fr} 0.01 0.001"/>
    <geom name="rim_xp" type="box" size="0.006 0.372 0.020" pos=" 0.486 0 0.020" rgba=".35 .33 .30 1" friction="0.4"/>
    <geom name="rim_xn" type="box" size="0.006 0.372 0.020" pos="-0.486 0 0.020" rgba=".35 .33 .30 1" friction="0.4"/>
    <geom name="rim_yp" type="box" size="0.492 0.006 0.020" pos="0  0.366 0.020" rgba=".35 .33 .30 1" friction="0.4"/>
    <geom name="rim_yn" type="box" size="0.492 0.006 0.020" pos="0 -0.366 0.020" rgba=".35 .33 .30 1" friction="0.4"/>
    <body name="connector" pos="{ax} {ay} {z0}">
      <geom type="box" size="0.012 0.009 0.008" pos="{cx} {cy} 0.004" euler="0 0 {hdg}" rgba=".12 .12 .14 1" contype="0" conaffinity="0"/>
      <composite prefix="c" type="cable" vertex="{verts}" initial="ball">
        <plugin plugin="mujoco.elasticity.cable">
          <config key="twist" value="{tw}"/>
          <config key="bend" value="{bd}"/>
          <config key="vmax" value="0"/>
        </plugin>
        <joint kind="main" damping="0.003" armature="1e-5"/>
        <geom type="capsule" size="{R}" rgba="{cr} {cg} {cb} 1" condim="4" friction="{fr} 0.01 0.001" density="{rho}" solref="0.004 1" solimp="0.95 0.99 0.0005"/>
      </composite>
    </body>
    {fixtures}
    {socket}
    <body name="tcp_target" mocap="true" pos="0 -0.25 0.15"/>
    <body name="gripper" pos="0 -0.25 0.15">
      <freejoint/>
      <camera name="wrist" pos="{wx} {wy} {wz}" xyaxes="0 -1 0 0.911 0 0.413" fovy="{wfov}"/>
      <geom type="box" size="0.012 0.03 0.008" pos="0 0 0.06" rgba=".3 .32 .36 1" mass="0.3" contype="0" conaffinity="0"/>
      <geom type="box" size="0.012 0.003 0.012" pos="0 0.027 0.042" rgba=".3 .32 .36 1" mass="0.01" contype="0" conaffinity="0"/>
      <geom type="box" size="0.012 0.003 0.012" pos="0 -0.027 0.042" rgba=".3 .32 .36 1" mass="0.01" contype="0" conaffinity="0"/>
      <body name="finger_l" pos="0 0.012 0">
        <joint name="jl" type="slide" axis="0 1 0" range="-0.021 0.021" damping="2"/>
        <geom name="pad_l" type="box" size="0.008 0.003 0.013" pos="0 0 0.013" rgba=".55 .56 .6 1" friction="1.5 0.05 0.002" condim="6" priority="2" mass="0.03" solref="0.003 1" solimp="0.95 0.99 0.0005"/>
      </body>
      <body name="finger_r" pos="0 -0.012 0">
        <joint name="jr" type="slide" axis="0 -1 0" range="-0.021 0.021" damping="2"/>
        <geom name="pad_r" type="box" size="0.008 0.003 0.013" pos="0 0 0.013" rgba=".55 .56 .6 1" friction="1.5 0.05 0.002" condim="6" priority="2" mass="0.03" solref="0.003 1" solimp="0.95 0.99 0.0005"/>
      </body>
    </body>
  </worldbody>
  <equality>
    <weld body1="tcp_target" body2="gripper" solref="0.01 1"/>
  </equality>
  <actuator>
    <position name="act_l" joint="jl" kp="1200" kv="20" ctrlrange="{fc} {fo}" forcerange="-25 25"/>
    <position name="act_r" joint="jr" kp="1200" kv="20" ctrlrange="{fc} {fo}" forcerange="-25 25"/>
  </actuator>
</mujoco>
""".format(dt=PHYS_DT, fr=ep.friction, ax=ep.conn_pos[0], ay=ep.conn_pos[1], z0=R, verts=verts, hdg=ep.conn_heading,
           cx=-0.012 * math.cos(ep.conn_heading), cy=-0.012 * math.sin(ep.conn_heading),
           tw=ep.twist, bd=ep.bend, R=R, rho=ep.density, fixtures="\n    ".join(fx), socket="\n    ".join(sock),
           fc=FINGER_CLOSED, fo=FINGER_OPEN, b=b, b2=b * 0.97, cr=cr, cg=cg, cb=cb,
           amb=0.25 * ep.light, hd=0.35 * ep.light, li=0.6 * ep.light,
           ox=OVERHEAD_POS[0], oy=OVERHEAD_POS[1], oz=OVERHEAD_POS[2], ofov=CAMERAS["overhead"][2],
           wx=WRIST_POS[0], wy=WRIST_POS[1], wz=WRIST_POS[2], wfov=CAMERAS["wrist"][2])


def intrinsics(name):
    """Pinhole intrinsics [fx, fy, cx, cy] of a camera, in pixels (square pixels, no distortion)."""
    w, h, fovy = CAMERAS[name]
    f = 0.5 * h / math.tan(math.radians(fovy) / 2.0)
    return [f, f, (w - 1) / 2.0, (h - 1) / 2.0]


class CableRouting(object):
    """One cell. `reset(seed)` builds the episode; `step(action)` advances 0.1 s; `render()` draws both cameras."""

    max_steps = MAX_STEPS

    def __init__(self):
        self.model = None
        self.data = None
        self._renderers = {}

    # -- construction -----------------------------------------------------------------
    def reset(self, seed):
        self.ep = make_episode(seed)
        for _attempt in range(8):
            obs = self._build(seed)
            P = self.cable_points()
            F = _obstacles(self.ep)
            clear = np.min(np.linalg.norm(P[:, None, :2] - F[None, :, :], axis=2))
            pz = self.plug_pose()[0][2]
            if clear > 0.02 and P[:, 2].max() < 0.008 and pz < 0.008:
                return obs
            nxt = _initial_shape(self.ep.rng, self.ep)
            if nxt is None:
                break
            self.ep.initial = nxt
        return obs

    def _build(self, seed):
        self.close_renderers()
        spec = mujoco.MjSpec.from_string(_xml(self.ep))
        site = spec.body("cB_first").add_site()
        site.group = 4
        site.rgba = [0, 0, 0, 0]
        site.name = "conn_site"
        sens = spec.add_sensor()
        sens.name = "connector_force"
        sens.type = mujoco.mjtSensor.mjSENS_FORCE
        sens.objtype = mujoco.mjtObj.mjOBJ_SITE
        sens.objname = "conn_site"
        # the plug on the free end: placed in the last segment's frame, so the frame's axes are read from a first
        # compile of the straight cable
        probe = spec.compile()
        pd = mujoco.MjData(probe)
        mujoco.mj_forward(probe, pd)
        last = probe.body("cB_last").id
        Rl = pd.xmat[last].reshape(3, 3)
        up = Rl.T @ np.array([0.0, 0.0, 1.0])
        fwd = Rl.T @ np.array([math.cos(self.ep.conn_heading), math.sin(self.ep.conn_heading), 0.0])
        seg = self.ep.length / N_SEG
        plug = spec.body("cB_last").add_body()
        plug.name = "plug"
        plug.pos = seg * fwd
        side = np.cross(up, fwd)
        # the plug body's frame: x along the cable, z up (when the cable lies straight and untwisted)
        Rp = np.stack([fwd, side, up], axis=1)
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, Rp.reshape(-1))
        plug.quat = q
        # the boot: the cable can bend where it leaves the plug (two springy hinges, up-down and sideways); it cannot
        # turn the plug about the cable's own axis other than by twisting the cable
        for name, axis in (("boot_pitch", [0.0, 1.0, 0.0]), ("boot_yaw", [0.0, 0.0, 1.0])):
            boot = plug.add_joint()
            boot.name = name
            boot.type = mujoco.mjtJoint.mjJNT_HINGE
            boot.axis = axis
            boot.stiffness = [BOOT_STIFFNESS, 0.0, 0.0]
            boot.damping = [0.002, 0.0, 0.0]
            boot.armature = 1e-5
        g = plug.add_geom()
        g.name = "plug_body"
        g.type = mujoco.mjtGeom.mjGEOM_BOX
        g.size = [PLUG_L / 2, PLUG_W / 2, PLUG_H / 2]
        g.pos = [PLUG_L / 2, 0.0, PLUG_H / 2 - self.ep.radius + 0.0002]
        g.mass = PLUG_MASS
        g.rgba = [0.15, 0.15, 0.17, 1.0]
        g.friction = [0.25, 0.005, 0.0005]
        g.priority = 1                     # its own friction against the board and the socket
        g.condim = 4
        g.solref = [0.003, 1.0]
        g.solimp = [0.95, 0.99, 0.0005, 0.5, 2.0]
        mark = plug.add_geom()
        mark.name = "plug_mark"
        mark.type = mujoco.mjtGeom.mjGEOM_BOX
        mark.size = [0.004, PLUG_W / 2 - 0.002, 0.0003]
        mark.pos = [PLUG_L - PLUG_IN - 0.006, 0.0, PLUG_H - self.ep.radius + 0.0005]
        mark.rgba = [0.85, 0.85, 0.85, 1.0]
        mark.contype = 0
        mark.conaffinity = 0
        mark.mass = 0.0001
        nose = plug.add_site()
        nose.group = 4                      # not drawn by the cameras
        nose.rgba = [0, 0, 0, 0]
        nose.name = "plug_nose"
        nose.pos = [PLUG_L, 0.0, PLUG_H / 2 - self.ep.radius]
        tail = plug.add_site()
        tail.group = 4
        tail.rgba = [0, 0, 0, 0]
        tail.name = "plug_tail"
        tail.pos = [0.0, 0.0, PLUG_H / 2 - self.ep.radius]
        # the boot keeps the plug from folding back onto the last few centimetres of its own cable
        for k in range(3):
            ex = spec.add_exclude()
            ex.bodyname1 = "plug"
            ex.bodyname2 = "cB_last" if k == 0 else "cB_%d" % (N_SEG - 1 - k)
        eq = spec.add_equality()
        eq.name = "plug_latch"
        eq.type = mujoco.mjtEq.mjEQ_WELD
        eq.objtype = mujoco.mjtObj.mjOBJ_BODY
        eq.name1 = "socket"
        eq.name2 = "plug"
        eq.active = False
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        m, d = self.model, self.data
        self.seg_ids = [i for i in range(m.nbody) if m.body(i).name.startswith("cB")]
        self.seg_len = seg
        self.mocap = m.body("tcp_target").mocapid[0]
        self.grip_body = m.body("gripper").id
        self.plug_body = m.body("plug").id
        self.sock_body = m.body("socket").id
        self.nose_site = m.site("plug_nose").id
        self.tail_site = m.site("plug_tail").id
        self.latch_eq = m.equality("plug_latch").id
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
        self.plug_latched = False
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
        """The cable's centre line: N_SEG + 1 points from the connector to the plug (true state)."""
        d = self.data
        pts = [d.xpos[i].copy() for i in self.seg_ids]
        last = self.seg_ids[-1]
        R = d.xmat[last].reshape(3, 3)
        pts.append(d.xpos[last] + R[:, 0] * self.seg_len)
        return np.array(pts)

    def plug_pose(self):
        """(nose position, heading of the plug's axis in the board plane, tilt of the axis from the board) (true)."""
        d = self.data
        nose = d.site_xpos[self.nose_site].copy()
        tail = d.site_xpos[self.tail_site].copy()
        ax = nose - tail
        heading = math.atan2(ax[1], ax[0])
        tilt = math.atan2(ax[2], math.hypot(ax[0], ax[1]))
        return nose, heading, tilt

    def gripper_state(self):
        d = self.data
        pos = d.xpos[self.grip_body].copy()
        R = d.xmat[self.grip_body].reshape(3, 3)
        yaw = math.atan2(R[1, 0], R[0, 0])
        width = (0.012 + d.qpos[self.jl]) + (0.012 + d.qpos[self.jr]) - 0.006
        force = float(abs(d.actuator_force[0]) + abs(d.actuator_force[1])) / 2.0
        return pos, yaw, max(width, 0.0), force

    def privileged(self):
        """The true state a perception system would estimate (for analysis and privileged baselines)."""
        nose, heading, tilt = self.plug_pose()
        return {"cable": [[round(float(v), 5) for v in p] for p in self.cable_points()],
                "plug": {"nose": [round(float(v), 5) for v in nose], "heading": round(heading, 5),
                         "tilt": round(tilt, 5)},
                "cable_radius": round(self.ep.radius, 5), "cable_length": round(self.ep.length, 5)}

    def observe(self):
        """Everything but the camera frames (which `render` draws)."""
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
            "connector": {"pos": [round(float(self.ep.conn_pos[0]), 5), round(float(self.ep.conn_pos[1]), 5)],
                          "heading": round(float(self.ep.conn_heading), 5)},
            "fixtures": fixtures,
            "socket": {"pos": [round(float(self.ep.sock_pos[0]), 5), round(float(self.ep.sock_pos[1]), 5)],
                       "heading": round(float(self.ep.sock_heading), 5), "mated": bool(self.plug_latched)},
            "connector_force": round(self.load_avg, 3),
            "cameras": self.camera_info(),
        }

    def connector_force(self):
        f = self.data.sensordata[self.sensor_adr:self.sensor_adr + 3]
        return float(np.linalg.norm(f))

    # -- cameras ------------------------------------------------------------------------
    def close_renderers(self):
        for r in self._renderers.values():
            try:
                r.close()
            except Exception:
                pass
        self._renderers = {}

    def _renderer(self, name):
        if name not in self._renderers:
            w, h, _ = CAMERAS[name]
            self._renderers[name] = mujoco.Renderer(self.model, h, w)
        return self._renderers[name]

    def camera_pose(self, name):
        """World pose of a camera: (position, 3x3 rotation whose columns are the camera's x, y, z axes; the camera
        looks along -z with image rows running along -y)."""
        cid = self.model.camera(name).id
        return self.data.cam_xpos[cid].copy(), self.data.cam_xmat[cid].reshape(3, 3).copy()

    def render(self, name, shift=(0.0, 0.0)):
        """One RGB-D frame: (rgb uint8 HxWx3, depth float32 HxW in metres along the optical axis), with sensor noise.
        `shift` draws the scene as if everything had moved by (dx, dy) on the board (only for the fixed camera)."""
        m, d = self.model, self.data
        r = self._renderer(name)
        cam = mujoco.MjvCamera()
        cid = m.camera(name).id
        moved = name == "overhead" and (shift[0] != 0.0 or shift[1] != 0.0)
        if moved:
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            pos, R = self.camera_pose(name)
            look = -R[:, 2]
            cam.lookat[:] = [pos[0] - shift[0] + look[0], pos[1] - shift[1] + look[1], pos[2] + look[2]]
            cam.distance = 1.0
            cam.azimuth = 90.0
            cam.elevation = -90.0
        else:
            cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            cam.fixedcamid = cid
        frames = []
        for depth in (False, True):
            if depth:
                r.enable_depth_rendering()
            else:
                r.disable_depth_rendering()
            r.update_scene(d, cam)
            r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            r.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
            r.scene.flags[mujoco.mjtRndFlag.mjRND_SKYBOX] = False
            frames.append(r.render().copy())
        r.disable_depth_rendering()
        rgb = np.clip(frames[0].astype(np.float32) + self.noise.normal(0.0, RGB_NOISE, frames[0].shape), 0, 255)
        dep = frames[1].astype(np.float32) + self.noise.normal(0.0, DEPTH_NOISE, frames[1].shape).astype(np.float32)
        dep = (np.round(dep / DEPTH_STEP) * DEPTH_STEP).astype(np.float32)
        return rgb.astype(np.uint8), dep

    def camera_info(self):
        """Intrinsics of both cameras, the overhead camera's world pose and the wrist camera's pose in the gripper
        frame (the gripper frame: origin at the tool centre point, x forward, y along the finger closing axis)."""
        info = {}
        for name in CAMERAS:
            w, h, _ = CAMERAS[name]
            info[name] = {"width": w, "height": h, "K": [round(v, 4) for v in intrinsics(name)]}
        pos, R = self.camera_pose("overhead")
        info["overhead"]["pos"] = [round(float(v), 5) for v in pos]
        info["overhead"]["R"] = [[round(float(v), 6) for v in row] for row in R]
        cid = self.model.camera("wrist").id
        Rw = np.zeros(9)
        mujoco.mju_quat2Mat(Rw, self.model.cam_quat[cid])
        info["wrist"]["pos_in_gripper"] = [round(float(v), 5) for v in self.model.cam_pos[cid]]
        info["wrist"]["R_in_gripper"] = [[round(float(v), 6) for v in row] for row in Rw.reshape(3, 3)]
        return info

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

    def _plug_in_socket(self):
        """(depth of the plug's nose past the mouth, lateral offset, height of the nose's axis, misalignment angle) in
        the socket's frame."""
        nose, heading, tilt = self.plug_pose()
        rel = nose[:2] - self.ep.sock_pos
        c, s = math.cos(self.ep.sock_heading), math.sin(self.ep.sock_heading)
        depth = rel[0] * c + rel[1] * s
        lateral = -rel[0] * s + rel[1] * c
        mis = abs((heading - self.ep.sock_heading + math.pi) % (2 * math.pi) - math.pi)
        return depth, lateral, nose[2], mis

    def _update_latches(self):
        """A clip snaps shut once the cable has been pushed fully down between its pins; the plug latches once it is
        fully home in the socket."""
        if len(self.latched) < len(self.latch_ids):
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
        if not self.plug_latched:
            depth, lateral, z, mis = self._plug_in_socket()
            # the nose inside the cavity, near its back wall (never a plug lying beyond the socket on its line)
            if SOCK_DEPTH - SOCK_LATCH <= depth <= SOCK_DEPTH + 0.001 and abs(lateral) < SOCK_CLEAR + 0.0004 \
                    and mis < 0.06 and z < PLUG_H / 2 + SOCK_HEAD:
                self._latch_plug()

    def _latch_plug(self):
        m, d = self.model, self.data
        # the weld holds the plug where it is now, relative to the socket
        ps, qs = d.xpos[self.sock_body], d.xquat[self.sock_body]
        pp, qp = d.xpos[self.plug_body], d.xquat[self.plug_body]
        qs_inv = np.zeros(4)
        mujoco.mju_negQuat(qs_inv, qs)
        rel_p = np.zeros(3)
        mujoco.mju_rotVecQuat(rel_p, pp - ps, qs_inv)
        rel_q = np.zeros(4)
        mujoco.mju_mulQuat(rel_q, qs_inv, qp)
        # eq_data for a weld: anchor (3), relative position (3), relative quaternion (4), torque scale
        m.eq_data[self.latch_eq, 0:3] = 0.0
        m.eq_data[self.latch_eq, 3:6] = rel_p
        m.eq_data[self.latch_eq, 6:10] = rel_q
        m.eq_data[self.latch_eq, 10] = 1.0
        d.eq_active[self.latch_eq] = 1
        self.plug_latched = True

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
            return {"credit": 0.0, "routed": 0, "fixtures": len(self.ep.fixtures), "mated": False, "unstable": True,
                    "damaged": self.damaged, "peak_connector_force": self.peak_force, "checks": []}
        pos, _, _, _ = self.gripper_state()
        self.set_command(pos, self.target_yaw, 0.0)
        for _ in range(int(0.4 / PHYS_DT)):
            self._substep()
        self.set_command([pos[0], pos[1], 0.15], self.target_yaw, 0.0)
        for _ in range(int(SETTLE_S / PHYS_DT)):
            self._substep()
        return evaluate(self.ep, self.cable_points(), self.plug_latched, self.damaged, self.peak_force)


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


def evaluate(ep, P, mated=False, damaged=False, peak_force=0.0):
    """Which fixtures the settled cable satisfies, in route order, and whether the plug is mated.
    P: (N_SEG + 1, 3) cable points. credit = route * (CREDIT_UNMATED + (1 - CREDIT_UNMATED) * mated), where route is
    the satisfied share of the fixtures; 0 if the connector was damaged."""
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
    route = routed / float(len(ep.fixtures))
    credit = 0.0 if damaged else route * (CREDIT_UNMATED + (1.0 - CREDIT_UNMATED) * float(bool(mated)))
    return {"credit": credit, "routed": routed, "fixtures": len(ep.fixtures), "mated": bool(mated),
            "damaged": bool(damaged), "peak_connector_force": float(peak_force), "unstable": False,
            "checks": [{"type": c["type"], "ok": c["ok"]} for c in checks]}


def write_frames(env, directory, shift=(0.0, 0.0), wrist=None):
    """Draw both cameras and save each frame as .npy files under `directory`, readable by every account, with names
    that carry nothing but a random token. Returns ({camera: {"rgb": path, "depth": path}}, {camera: (rgb, depth)}).
    `shift` draws the overhead camera as if the scene had moved by (dx, dy) on the board; `wrist` reuses frames
    already drawn for the wrist camera (it moves with the gripper, so a shifted scene looks the same to it)."""
    token = secrets.token_hex(8)
    paths, arrays = {}, {}
    for name in CAMERAS:
        if name == "wrist" and wrist is not None:
            rgb, dep = wrist
        else:
            rgb, dep = env.render(name, shift=shift if name == "overhead" else (0.0, 0.0))
        arrays[name] = (rgb, dep)
        paths[name] = {}
        for kind, arr in (("rgb", rgb), ("depth", dep)):
            path = os.path.join(directory, "%s_%s_%s.npy" % (token, name, kind))
            np.save(path, arr)
            os.chmod(path, 0o644)
            paths[name][kind] = path
    return paths, arrays


def remove_frames(paths):
    for cam in paths.values():
        for path in cam.values():
            try:
                os.unlink(path)
            except OSError:
                pass


def shift_obs(obs, dx, dy):
    """The observation as if everything on the board had moved by (dx, dy): every horizontal coordinate moves;
    the overhead camera's pose stays (its frame must be drawn with the same shift)."""
    def mv(p):
        return [round(p[0] + dx, 5), round(p[1] + dy, 5)] + list(p[2:])
    out = dict(obs)
    g = dict(obs["gripper"])
    g["pos"] = mv(g["pos"])
    out["gripper"] = g
    for key in ("connector", "socket"):
        c = dict(obs[key])
        c["pos"] = mv(c["pos"])
        out[key] = c
    out["fixtures"] = []
    for f in obs["fixtures"]:
        f2 = dict(f)
        f2["pos"] = mv(f["pos"])
        out["fixtures"].append(f2)
    if "privileged" in obs:
        pv = dict(obs["privileged"])
        pv["cable"] = [mv(p) for p in pv["cable"]]
        pl = dict(pv["plug"])
        pl["nose"] = mv(pl["nose"])
        pv["plug"] = pl
        out["privileged"] = pv
    return out
