"""Server hot-swap maintenance: the simulated cell (grader and commissioning service only; never agent-visible).

Two Franka Pandas (MuJoCo Menagerie, parallel grippers, wrist force/torque sensors, operational-space control at
10 Hz) in front of a rack server on a stand. The server's front bay holds 12 hot-swap drive caddies (2 rows x 6
columns) on rails; its top holds 4 hot-swap fan modules. Every caddy and fan is a free rigid body in contact with its
bay's walls (MuJoCo contact physics); each caddy carries a top-hinged release lever (a hinge joint) with a release
button (a sprung slide joint). On top of the contact physics, per sub-step, the sim applies the connectors' force laws
(insertion resistance, retention detent), the lever's spring and closing detent, and the latch (a joint equality that
holds a lever closed while engaged). Status LEDs (emissive geoms) show which drive / fan failed.

Hidden per unit (draw_unit): the rack's pose, the cameras' mount offsets, the button force, the lever detent torque,
the lever break torque, the connector insertion and retention forces (drives and fans), rail friction, caddy mass,
the overheat window, LED brightness, the light. Per episode (draw_episode): which drive(s) and fan failed, the LED
flicker phase, small tray jitter, sensor noise seeds.
"""

import math
import os

import mujoco
import numpy as np

# ----------------------------------------------------------------------------------------------------------------
# constants
TIMESTEP = 0.002
SUBSTEPS = 50                         # 10 Hz control
DT_CTRL = TIMESTEP * SUBSTEPS
MAX_DPOS = 0.03
MAX_DROT = 0.2
MAX_DQ = 0.1
MAX_ARM_SPEED = 8.0
MAX_OBJECT_SPEED = 5.0
BASES = {"l": np.array([0.0, 0.30, 0.0]), "r": np.array([0.0, -0.30, 0.0])}
HOME_Q = np.array([0.0, -0.30, 0.0, -2.20, 0.0, 1.95, math.pi / 4])

# the rack frame R: origin at the chassis's front-bottom edge, centred in y; x into the server (away from the robots),
# z up. The drives load from the top (a top-loading storage server): 3 rows (x) x 4 columns (y) of vertical slots.
RACK_NOM = np.array([0.31, 0.0, 0.18])          # world position of R's origin (nominal); yaw 0
CHASSIS = {"depth": 0.40, "half_w": 0.25, "height": 0.175, "wall": 0.006}
NROW, NCOL = 3, 4
ROW_X0, PITCH_X = 0.045, 0.055                  # slot row centres (R x)
PITCH_Y = 0.078
SLOT_T, SLOT_W = 0.038, 0.072                   # slot inner size: x (thickness way), y (width way)
CAD_T, CAD_W, CAD_D = 0.032, 0.066, 0.150       # caddy: thickness (x), width (y), length (z, vertical)
CAD_NOSE = 0.012                                # bottom lead-in, 3 mm narrower each side
CAD_PROTRUDE = 0.004                            # a seated caddy's top sits this far above the chassis top
SLOT_FLOOR = CHASSIS["height"] + CAD_PROTRUDE - CAD_D - 0.0002   # the slot floor (the caddy's hard stop), R z
LEV_T, LEV_L, LEV_W = 0.005, 0.024, 0.060       # lever plate thickness, length (hinge to free end), width
LEV_STOP = math.radians(90.0)
LEV_HINGE_OUT = 0.008                           # the lever's hinge sits on a bracket this far in front of the caddy
LEV_POP = math.radians(100.0)                   # the lever spring's rest angle (beyond the stop: a released lever stands up)
LEV_SPRING = 0.04                               # N m / rad, the lever's opening spring
LEV_DET_ANGLE = math.radians(12.0)              # the closing detent's span
LEV_LATCH_ANGLE = math.radians(2.0)
BTN_TRAVEL, BTN_TRIP = 0.003, 0.002
CONN_SPAN = 0.005                               # connector engagement span (m) before home
CONN_SEAT = 0.0008                              # click distance
CONN_UNSEAT = 0.0018                            # a seated part pulled out past this is unseated
FAN_W, FAN_H = 0.060, 0.050
FAN_BAY_W, FAN_BAY_DEPTH = 0.064, 0.040
FAN_X = 0.235                                   # fan bays' centre x (R frame)
FAN_YS = (0.120, 0.040, -0.040, -0.120)         # bay 0 at the left (+y)
HANDLE_UP, HANDLE_LEN, HANDLE_T = 0.016, 0.044, 0.008
STAND_TOP = 0.18
TABLE_TOP = 0.20
TABLES = {"l": (np.array([0.38, 0.53]), np.array([0.20, 0.15])),     # centre (x, y), half sizes
          "r": (np.array([0.38, -0.53]), np.array([0.20, 0.15]))}
SPARE_AT = [np.array([0.44, 0.43]), np.array([0.44, 0.56])]          # spare caddies standing in a holder (x, y)
SPARE_HOLDER_H = 0.060
SPARE_FAN_AT = np.array([0.27, 0.46])
BIN_C, BIN_HALF, BIN_H = np.array([0.20, -0.12]), np.array([0.075, 0.10]), 0.12  # the failed-parts bin, front right
DROP_Z = 0.12
STRIKE_N = 60.0
STRIKE_SUBSTEPS = 10
BREAK_SUBSTEPS = 25
WRENCH_NOISE = 0.3
PAD_TORSION = 0.012
GRIP_FORCE = 70.0
GRIP_KP = 30000.0
PAD_FRICTION = 1.2

CAMERA_BITS = {"overhead": 1, "wrist_l": 2, "wrist_r": 4}
ALL_CAMS = 7
CAM_NOM = {"overhead": {"pos": (0.25, 0.0, 1.50), "look": (0.45, 0.0, 0.30), "fovy": 60.0, "w": 640, "h": 480}}
WRIST_NOM = {"pos": (0.065, 0.0, 0.035), "look": (0.0, 0.0, 0.16), "fovy": 70.0, "w": 320, "h": 240}

UNIT_RANGES = {
    "rack_dx": (-0.015, 0.015), "rack_dy": (-0.015, 0.015), "rack_dz": (-0.008, 0.008),
    "rack_yaw_deg": (-3.0, 3.0),
    "btn_force": (4.0, 10.0),            # N to trip the release button
    "lever_detent": (0.12, 0.30),        # N m peak of the closing detent
    "lever_break": (0.12, 0.17),         # N m held on the lever's open stop for 20 ms breaks it
    "ins_force": (10.0, 25.0),           # N, drive connector insertion
    "pull_force": (8.0, 20.0),           # N, drive connector retention
    "fan_ins": (6.0, 16.0), "fan_pull": (6.0, 14.0),
    "rail_mu": (0.15, 0.45),
    "caddy_mass": (0.80, 1.20), "fan_mass": (0.15, 0.30),
    "overheat_s": (20.0, 40.0),
    "led_gain": (0.45, 1.0),
    "light": (0.55, 1.0),
    "cam_off_mm": 15.0, "cam_off_deg": 1.5, "wrist_off_mm": 4.0, "wrist_off_deg": 1.0,
}


class RenderError(RuntimeError):
    pass


# ----------------------------------------------------------------------------------------------------------------
# small maths
def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def axis_angle_mat(v):
    v = np.asarray(v, float)
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def mat_quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R, float).ravel())
    return q


def look_at_xyaxes(pos, target, up=(0.0, 0.0, 1.0)):
    f = np.asarray(target, float) - np.asarray(pos, float)
    f /= np.linalg.norm(f)
    u = np.asarray(up, float)
    if abs(float(np.dot(f, u))) > 0.98:
        u = np.array([1.0, 0.0, 0.0])
    x = np.cross(f, u)
    x /= np.linalg.norm(x)
    y = np.cross(x, f)
    return np.concatenate([x, y])


def xyaxes_mat(xy):
    x, y = np.asarray(xy[:3], float), np.asarray(xy[3:], float)
    z = np.cross(x, y)
    return np.stack([x, y, z], axis=1)


def intrinsics(fovy, w, h):
    f = 0.5 * h / math.tan(math.radians(fovy) / 2)
    return np.array([[f, 0.0, w / 2.0], [0.0, f, h / 2.0], [0.0, 0.0, 1.0]])


def rng_for(seed, tag):
    seed = int(seed)
    return np.random.default_rng([seed & 0xFFFFFFFF, seed >> 32, int(tag)])


# ----------------------------------------------------------------------------------------------------------------
# draws
def draw_unit(seed):
    """A unit's hidden values (one server and cell)."""
    r = rng_for(seed, 1)
    U = {k: float(r.uniform(*v)) for k, v in UNIT_RANGES.items() if isinstance(v, tuple)}
    U["seed"] = int(seed)

    def off(mm, deg):
        d = r.normal(size=3)
        d *= r.uniform(0.5, 1.0) * mm / 1000.0 / max(1e-9, np.linalg.norm(d))
        a = r.normal(size=3)
        a *= r.uniform(0.5, 1.0) * math.radians(deg) / max(1e-9, np.linalg.norm(a))
        return {"dpos": d.tolist(), "drot": a.tolist()}

    U["cam_off"] = {"overhead": off(UNIT_RANGES["cam_off_mm"], UNIT_RANGES["cam_off_deg"]),
                    "wrist_l": off(UNIT_RANGES["wrist_off_mm"], UNIT_RANGES["wrist_off_deg"]),
                    "wrist_r": off(UNIT_RANGES["wrist_off_mm"], UNIT_RANGES["wrist_off_deg"])}
    U["amber_blink"] = bool(r.uniform() < 0.5)
    U["glare_dir"] = (r.normal(size=3) * np.array([1.0, 1.0, 0.3]) + np.array([0.0, 0.0, -1.0])).tolist()
    return U


def draw_episode(seed, unit, n_failed_drives=None):
    r = rng_for(seed, 2)
    nd = int(n_failed_drives) if n_failed_drives is not None else 1
    r.uniform()
    slots = [int(x) for x in r.choice(NROW * NCOL, size=nd, replace=False)]
    fan = int(r.integers(0, len(FAN_YS)))
    return {"seed": int(seed), "failed_slots": sorted(slots), "failed_fan": fan,
            "spare_jitter": r.uniform(-0.006, 0.006, (3, 2)).tolist(),
            "flicker_seed": int(r.integers(0, 2 ** 31)), "blink_phase": float(r.uniform(0, 1))}


def rack_pose(unit):
    p = RACK_NOM + np.array([unit["rack_dx"], unit["rack_dy"], unit["rack_dz"]])
    return p, rotz(math.radians(unit["rack_yaw_deg"]))


def slot_rc(k):
    return k // NCOL, k % NCOL


def slot_center_R(k):
    """The slot's centre at the chassis top (R frame)."""
    row, col = slot_rc(k)
    return np.array([ROW_X0 + row * PITCH_X, -(col - (NCOL - 1) / 2.0) * PITCH_Y, CHASSIS["height"]])


def fan_center_R(i):
    return np.array([FAN_X, FAN_YS[i], CHASSIS["height"]])


# ----------------------------------------------------------------------------------------------------------------
# model
def _box(body, name, size, pos, rgba, quat=None, mu=None, mass=None, contype=1, conaffinity=1, group=0):
    g = body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=list(size), pos=list(pos), rgba=list(rgba),
                      contype=contype, conaffinity=conaffinity, group=group)
    if quat is not None:
        g.quat = list(quat)
    if mu is not None:
        g.friction = [mu, 0.005, 0.0001]
    if mass is not None:
        g.mass = mass
    return g


def _visor_led(body, name, pos, axis_out):
    """A 2.4 mm status LED on a vertical face (outward normal -x of the part) under a 3 mm visor: seen only from in
    front of the face, not from above."""
    x, y, z = pos
    body.add_geom(name=name + "_led", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.0003, 0.0012, 0.0012],
                  pos=[x - 0.0002, y, z], rgba=[0.05, 0.05, 0.05, 1], contype=0, conaffinity=0, group=1)
    body.add_geom(name=name + "_visor", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.0016, 0.0024, 0.0003],
                  pos=[x - 0.0016, y, z + 0.0016], rgba=[0.06, 0.06, 0.07, 1], contype=0, conaffinity=0, group=1)
    for s_ in (-1, 1):
        body.add_geom(name=name + "_cheek%d" % (s_ > 0), type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[0.0016, 0.0003, 0.0016], pos=[x - 0.0016, y + s_ * 0.0021, z + 0.0002],
                      rgba=[0.06, 0.06, 0.07, 1], contype=0, conaffinity=0, group=1)


def _caddy(wb, name, pos, quat, unit, rgba_body, spare=False):
    """A drive caddy, standing: body frame at the top face's centre, x the thickness way (front -x), z up."""
    b = wb.add_body(name=name, pos=list(pos), quat=list(quat))
    b.add_freejoint(name=name + "_free")
    m = unit["caddy_mass"]
    L = CAD_D - CAD_NOSE
    _box(b, name + "_body", (CAD_T / 2, CAD_W / 2, L / 2), (0, 0, -L / 2), rgba_body, mass=m * 0.95)
    _box(b, name + "_nose", (CAD_T / 2 - 0.003, CAD_W / 2 - 0.003, CAD_NOSE / 2), (0, 0, -L - CAD_NOSE / 2),
         rgba_body, mass=m * 0.05)
    b.add_geom(name=name + "_label", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.0002, 0.012, 0.004],
               pos=[-CAD_T / 2 - 0.0002, 0.010, -0.007], rgba=[0.85, 0.85, 0.80, 1], contype=0, conaffinity=0,
               group=1)
    _visor_led(b, name, (-CAD_T / 2, -0.018, -0.0024), -1)
    # lever: hinge along y at the top front edge; closed, the plate lies on the top face; it opens upward
    lv = b.add_body(name=name + "_lever", pos=[-CAD_T / 2 - LEV_HINGE_OUT, 0, 0])
    _box(b, name + "_bracket", (LEV_HINGE_OUT / 2, LEV_W / 2 - 0.004, 0.002), (-CAD_T / 2 - LEV_HINGE_OUT / 2, 0, -0.002),
         (0.20, 0.21, 0.24, 1), mass=0.005)
    j = lv.add_joint(name=name + "_lever_j", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, -1, 0],
                     range=[0.0, LEV_STOP], damping=0.002, armature=1e-4)
    j.solref_limit = [0.004, 1.0]
    j.solimp_limit = [0.95, 0.99, 0.001, 0.5, 2.0]
    _box(lv, name + "_lever_g", (LEV_L / 2, LEV_W / 2, LEV_T / 2), (LEV_L / 2, 0, LEV_T / 2),
         (0.20, 0.21, 0.24, 1), mass=0.02)
    lv.add_geom(name=name + "_lever_m", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.0035, LEV_W / 2 - 0.004, 0.0003],
                pos=[0.006, 0, LEV_T + 0.0003], rgba=[0.55, 0.57, 0.60, 1], contype=0, conaffinity=0, group=1)
    # release button at the free end, sprung, pressed toward the plate
    bt = lv.add_body(name=name + "_btn", pos=[LEV_L - 0.007, 0.015, LEV_T])
    bt.add_joint(name=name + "_btn_j", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 0, -1], range=[0.0, BTN_TRAVEL],
                 stiffness=unit["btn_force"] / BTN_TRIP, damping=2.0, armature=0.05, springref=0.0)
    _box(bt, name + "_btn_g", (0.005, 0.012, 0.0015), (0, 0, 0.0015), (0.85, 0.15, 0.10, 1), mass=0.003)
    return b


def _fan(wb, name, pos, quat, unit, rgba):
    """A fan module: body frame at the bottom centre; handle on top; status LED on the front face's top edge."""
    b = wb.add_body(name=name, pos=list(pos), quat=list(quat))
    b.add_freejoint(name=name + "_free")
    m = unit["fan_mass"]
    _box(b, name + "_g", (FAN_W / 2, FAN_W / 2, FAN_H / 2), (0, 0, FAN_H / 2), rgba, mass=m * 0.9)
    for s in (-1, 1):
        _box(b, name + "_post%d" % (s > 0), (0.003, 0.003, HANDLE_UP / 2), (0, s * (HANDLE_LEN / 2 - 0.003),
             FAN_H + HANDLE_UP / 2), (0.15, 0.15, 0.16, 1), mass=m * 0.02)
    _box(b, name + "_handle", (HANDLE_T / 2, HANDLE_LEN / 2, HANDLE_T / 2), (0, 0, FAN_H + HANDLE_UP - HANDLE_T / 2),
         (0.15, 0.15, 0.16, 1), mass=m * 0.06)
    b.add_geom(name=name + "_grille", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.026, 0.0004, 0],
               pos=[0.0, 0, FAN_H + 0.0004], rgba=[0.05, 0.05, 0.06, 1], contype=0, conaffinity=0, group=1)
    _visor_led(b, name, (-FAN_W / 2, 0.018, FAN_H - 0.004), -1)
    return b


def _checker_plate(body, spec, name, pos, nx, ny, sq, quat=None):
    """A flat checker plate of nx x ny squares of side sq (visual geoms; one contact box under them)."""
    for i in range(nx):
        for j in range(ny):
            dark = (i + j) % 2 == 0
            c = (0.06, 0.06, 0.06, 1) if dark else (0.92, 0.92, 0.90, 1)
            x = (i - (nx - 1) / 2.0) * sq
            y = (j - (ny - 1) / 2.0) * sq
            g = body.add_geom(name="%s_%d_%d" % (name, i, j), type=mujoco.mjtGeom.mjGEOM_BOX,
                              size=[sq / 2, sq / 2, 0.0005], pos=[pos[0] + x, pos[1] + y, pos[2] + 0.0005],
                              rgba=list(c), contype=0, conaffinity=0, group=1)
    _box(body, name + "_base", (nx * sq / 2 + 0.006, ny * sq / 2 + 0.006, 0.0015),
         (pos[0], pos[1], pos[2] - 0.0015), (0.92, 0.92, 0.90, 1))


def panda_dir():
    d = os.environ.get("SRV_PANDA_DIR", "/opt/task-assets/panda")
    if not os.path.isfile(os.path.join(d, "panda.xml")):
        raise RuntimeError("the Panda model is missing at %s" % d)
    return d


def _arm_spec(prefix, unit):
    s = mujoco.MjSpec.from_file(os.path.join(panda_dir(), "panda.xml"))
    for a in list(s.actuators):
        s.delete(a)
    for lt in list(s.lights):
        s.delete(lt)
    for k in list(s.keys):
        s.delete(k)
    lim = [87, 87, 87, 87, 12, 12, 12]
    for i in range(1, 8):
        a = s.add_actuator(name="torq_j%d" % i, target="joint%d" % i, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.ctrlrange = [-lim[i - 1], lim[i - 1]]
        a.ctrllimited = 1
        a.gainprm[0] = 1.0
    g = s.add_actuator(name="grip", target="split", trntype=mujoco.mjtTrn.mjTRN_TENDON)
    g.gaintype = mujoco.mjtGain.mjGAIN_FIXED
    g.biastype = mujoco.mjtBias.mjBIAS_AFFINE
    g.gainprm[0] = GRIP_KP                        # a stiff servo on the fingers' opening, its force capped:
    g.biasprm[0], g.biasprm[1], g.biasprm[2] = 0.0, -GRIP_KP, 0.0   # 35 N each finger at most
    g.ctrlrange = [0.0, 0.04]
    g.ctrllimited = 1
    g.forcerange = [-GRIP_FORCE, GRIP_FORCE]
    g.forcelimited = 1
    for jn in ("finger_joint1", "finger_joint2"):
        j = s.joint(jn)
        j.solref_limit = [0.004, 1.0]
        j.solimp_limit = [0.95, 0.99, 0.001, 0.5, 2.0]
        j.damping = [300.0, 0.0, 0.0]          # the fingers close at about 0.1 m/s
    hand = s.body("hand")
    hand.add_site(name="grip_site", pos=[0, 0, 0.1034], size=[0.002, 0, 0], group=4)
    hand.add_site(name="ft", pos=[0, 0, 0], size=[0.002, 0, 0], group=4)
    off = unit["cam_off"]["wrist_" + prefix]
    p = np.array(WRIST_NOM["pos"]) + np.array(off["dpos"])
    R = axis_angle_mat(off["drot"]) @ xyaxes_mat(look_at_xyaxes(WRIST_NOM["pos"], WRIST_NOM["look"], up=(1, 0, 0)))
    hand.add_camera(name="wrist", pos=list(p), xyaxes=list(R[:, 0]) + list(R[:, 1]), fovy=WRIST_NOM["fovy"])
    # camera body (visual) on the hand
    hand.add_geom(name="cam_housing", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.012, 0.03, 0.012],
                  pos=[WRIST_NOM["pos"][0] + 0.004, 0, WRIST_NOM["pos"][2] - 0.008], rgba=[0.1, 0.1, 0.12, 1],
                  contype=0, conaffinity=0, group=1)
    s.add_sensor(name="ft_force", type=mujoco.mjtSensor.mjSENS_FORCE, objtype=mujoco.mjtObj.mjOBJ_SITE, objname="ft")
    s.add_sensor(name="ft_torque", type=mujoco.mjtSensor.mjSENS_TORQUE, objtype=mujoco.mjtObj.mjOBJ_SITE,
                 objname="ft")
    return s


def build(unit, episode):
    rp, rR = rack_pose(unit)
    rq = mat_quat(rR)
    light = unit["light"]
    xml = """<mujoco model="server_hot_swap">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{dt}" integrator="implicitfast" cone="elliptic" impratio="10" noslip_iterations="3"/>
  <visual><global offwidth="640" offheight="480"/><quality shadowsize="0"/><map znear="0.01" zfar="6"/>
    <headlight ambient="{a} {a} {a}" diffuse="{h} {h} {h}" specular="0.1 0.1 0.1"/></visual>
  <worldbody>
    <light name="ceiling" pos="0.4 0 2.2" dir="0 0 -1" diffuse="{l} {l} {l}" specular="0.3 0.3 0.3" directional="true" castshadow="false"/>
    <light name="glare" pos="0.2 0 1.5" dir="{gx} {gy} {gz}" diffuse="{g} {g} {g}" specular="0.8 0.8 0.8" directional="true" castshadow="false"/>
    <geom name="floor" type="plane" size="3 3 0.1" rgba="0.36 0.37 0.38 1"/>
  </worldbody>
</mujoco>""".format(dt=TIMESTEP, a=0.25 * light, h=0.30 * light, l=0.55 * light, g=0.35 * light,
                    gx=unit["glare_dir"][0], gy=unit["glare_dir"][1], gz=unit["glare_dir"][2])
    spec = mujoco.MjSpec.from_string(xml)
    wb = spec.worldbody

    # cameras (true mounts: nominal + this unit's hidden offset)
    for name, c in CAM_NOM.items():
        off = unit["cam_off"][name]
        p = np.array(c["pos"]) + np.array(off["dpos"])
        R = axis_angle_mat(off["drot"]) @ xyaxes_mat(look_at_xyaxes(c["pos"], c["look"]))
        wb.add_camera(name=name, pos=list(p), xyaxes=list(R[:, 0]) + list(R[:, 1]), fovy=c["fovy"])

    # side tables, the spares' holder and the failed-parts bin
    for s, (c, h) in TABLES.items():
        _box(wb, "table_" + s, (h[0], h[1], TABLE_TOP / 2), (c[0], c[1], TABLE_TOP / 2), (0.45, 0.42, 0.38, 1))
    bc = BIN_C
    _box(wb, "bin_stand", (BIN_HALF[0] + 0.01, BIN_HALF[1] + 0.01, TABLE_TOP / 2), (bc[0], bc[1], TABLE_TOP / 2),
         (0.45, 0.42, 0.38, 1))
    _box(wb, "bin_floor", (BIN_HALF[0], BIN_HALF[1], 0.003), (bc[0], bc[1], TABLE_TOP + 0.003), (0.15, 0.3, 0.6, 1))
    for j, (dx, dy, sx, sy) in enumerate(((BIN_HALF[0], 0, 0.003, BIN_HALF[1]), (-BIN_HALF[0], 0, 0.003, BIN_HALF[1]),
                                          (0, BIN_HALF[1], BIN_HALF[0], 0.003), (0, -BIN_HALF[1], BIN_HALF[0], 0.003))):
        _box(wb, "bin_wall%d" % j, (sx, sy, BIN_H / 2), (bc[0] + dx, bc[1] + dy, TABLE_TOP + BIN_H / 2),
             (0.15, 0.3, 0.6, 1))
    spare_xy = []
    for j, c in enumerate(SPARE_AT):
        jit = episode["spare_jitter"][j]
        cx, cy = c[0] + jit[0], c[1] + jit[1]
        spare_xy.append((cx, cy))
        for sx_ in (-1, 1):
            _box(wb, "holder%d_x%d" % (j, sx_ > 0), (0.004, SLOT_W / 2 + 0.008, SPARE_HOLDER_H / 2),
                 (cx + sx_ * (SLOT_T / 2 + 0.004), cy, TABLE_TOP + SPARE_HOLDER_H / 2), (0.25, 0.25, 0.27, 1))
            _box(wb, "holder%d_y%d" % (j, sx_ > 0), (SLOT_T / 2, 0.004, SPARE_HOLDER_H / 2),
                 (cx, cy + sx_ * (SLOT_W / 2 + 0.004), TABLE_TOP + SPARE_HOLDER_H / 2), (0.25, 0.25, 0.27, 1))
    fj = episode["spare_jitter"][2]
    for s in (-1, 1):
        _box(wb, "fancradle_x%d" % (s > 0), (0.003, FAN_W / 2 + 0.006, 0.008),
             (SPARE_FAN_AT[0] + fj[0] + s * (FAN_W / 2 + 0.006), SPARE_FAN_AT[1] + fj[1], TABLE_TOP + 0.008),
             (0.25, 0.25, 0.27, 1))
    _checker_plate(wb, spec, "plate_l", (0.26, 0.60, TABLE_TOP + 0.003), 6, 4, 0.020)
    _checker_plate(wb, spec, "plate_r", (0.25, -0.62, TABLE_TOP + 0.003), 6, 4, 0.020)

    # the stand and the server chassis (body "rack" at R)
    rack = wb.add_body(name="rack", pos=list(rp), quat=list(rq))
    D, HW, H, T = CHASSIS["depth"], CHASSIS["half_w"], CHASSIS["height"], CHASSIS["wall"]
    grey, dark = (0.55, 0.56, 0.58, 1), (0.16, 0.16, 0.18, 1)
    _box(rack, "stand", (D / 2 + 0.02, HW + 0.02, STAND_TOP / 2 - 0.0005), (D / 2, 0, -STAND_TOP / 2),
         (0.25, 0.25, 0.27, 1))
    _box(rack, "ch_bottom", (D / 2, HW, T / 2), (D / 2, 0, T / 2), grey)
    _box(rack, "ch_front", (T / 2, HW, H / 2), (T / 2, 0, H / 2), grey)
    _box(rack, "ch_side_l", (D / 2, T / 2, H / 2), (D / 2, HW - T / 2, H / 2), grey)
    _box(rack, "ch_side_r", (D / 2, T / 2, H / 2), (D / 2, -HW + T / 2, H / 2), grey)
    _box(rack, "ch_back", (T / 2, HW, H / 2), (D - T / 2, 0, H / 2), grey)
    # the drive grid: walls between rows (x) and columns (y) from the slot floor up to the top
    xa = ROW_X0 - PITCH_X / 2
    xb = ROW_X0 + (NROW - 0.5) * PITCH_X
    yw = NCOL * PITCH_Y / 2
    mu = unit["rail_mu"]
    zh = (H - SLOT_FLOOR) / 2
    _box(rack, "slot_floor", ((xb - xa) / 2, yw, 0.003), ((xa + xb) / 2, 0, SLOT_FLOOR - 0.003),
         (0.10, 0.35, 0.15, 1))
    for i in range(NROW + 1):
        x = xa + i * PITCH_X
        _box(rack, "wall_x%d" % i, ((PITCH_X - SLOT_T) / 2, yw + 0.004, zh), (x, 0, SLOT_FLOOR + zh), dark, mu=mu)
    for j in range(NCOL + 1):
        y = -yw + j * PITCH_Y
        _box(rack, "wall_y%d" % j, ((xb - xa) / 2, (PITCH_Y - SLOT_W) / 2, zh), ((xa + xb) / 2, y, SLOT_FLOOR + zh),
             dark, mu=mu)
    # top plate around the drive grid and the fan bays
    fx0, fx1 = FAN_X - FAN_BAY_W / 2, FAN_X + FAN_BAY_W / 2
    xa0 = xa - (PITCH_X - SLOT_T) / 2
    xb0 = xb + (PITCH_X - SLOT_T) / 2
    _box(rack, "top_front", (xa0 / 2, HW, T / 2), (xa0 / 2, 0, H - T / 2), grey)
    for s in (-1, 1):
        _box(rack, "top_side%d" % (s > 0), ((xb0 - xa0) / 2, (HW - yw - 0.008) / 2, T / 2),
             ((xa0 + xb0) / 2, s * (HW + yw + 0.008) / 2, H - T / 2), grey)
    _box(rack, "top_mid", ((fx0 - xb0) / 2, HW, T / 2), ((fx0 + xb0) / 2, 0, H - T / 2), grey)
    _box(rack, "top_back", ((D - fx1) / 2, HW, T / 2), ((D + fx1) / 2, 0, H - T / 2), grey)
    ys = sorted(FAN_YS)
    edges = [-HW] + sum([[y - FAN_BAY_W / 2, y + FAN_BAY_W / 2] for y in ys], []) + [HW]
    for j in range(0, len(edges), 2):
        y0, y1 = edges[j], edges[j + 1]
        _box(rack, "top_fan%d" % j, (FAN_BAY_W / 2, (y1 - y0) / 2, T / 2), (FAN_X, (y0 + y1) / 2, H - T / 2), grey)
    for i, y in enumerate(ys):
        zf = H - FAN_BAY_DEPTH
        _box(rack, "fb%d_floor" % i, (FAN_BAY_W / 2, FAN_BAY_W / 2, 0.002), (FAN_X, y, zf - 0.002), dark)
        for j, (dx, dy, sx, sy) in enumerate(((FAN_BAY_W / 2 + 0.002, 0, 0.002, FAN_BAY_W / 2 + 0.004),
                                              (-FAN_BAY_W / 2 - 0.002, 0, 0.002, FAN_BAY_W / 2 + 0.004),
                                              (0, FAN_BAY_W / 2 + 0.002, FAN_BAY_W / 2, 0.002),
                                              (0, -FAN_BAY_W / 2 - 0.002, FAN_BAY_W / 2, 0.002))):
            _box(rack, "fb%d_w%d" % (i, j), (sx, sy, FAN_BAY_DEPTH / 2), (FAN_X + dx, y + dy, zf + FAN_BAY_DEPTH / 2),
                 dark)
    # calibration plate on the chassis top, front-left of the drive grid
    _checker_plate(rack, spec, "plate_s", (0.10, 0.205, H), 5, 4, 0.015)

    # caddies, seated in the grid
    for k in range(NROW * NCOL):
        c = slot_center_R(k)
        _caddy(wb, "c%d" % k, rp + rR @ np.array([c[0], c[1], H + CAD_PROTRUDE]), rq, unit, (0.30, 0.31, 0.33, 1))
    # spare caddies standing in the holder on the left table, levers open
    for j, (cx, cy) in enumerate(spare_xy):
        _caddy(wb, "s%d" % j, (cx, cy, TABLE_TOP + CAD_D + 0.0005), (1, 0, 0, 0), unit, (0.30, 0.31, 0.33, 1),
               spare=True)
    for i in range(len(FAN_YS)):
        c = fan_center_R(i)
        _fan(wb, "f%d" % i, rp + rR @ np.array([c[0], c[1], H - FAN_BAY_DEPTH]), rq, unit, (0.22, 0.23, 0.25, 1))
    _fan(wb, "fs", (SPARE_FAN_AT[0] + fj[0], SPARE_FAN_AT[1] + fj[1], TABLE_TOP + 0.0005), (1, 0, 0, 0), unit,
         (0.22, 0.23, 0.25, 1))

    for n in ["c%d" % k for k in range(NROW * NCOL)] + ["s%d" % j for j in range(len(SPARE_AT))]:
        e = spec.add_equality(type=mujoco.mjtEq.mjEQ_JOINT, name=n + "_latch", name1=n + "_lever_j")
        e.data = [0.0] * len(e.data)
        e.solref = [0.004, 1.0]
    for p, base in BASES.items():
        a = _arm_spec(p, unit)
        frame = wb.add_frame(pos=list(base))
        frame.attach_body(a.body("link0"), p + "_", "")
    m = spec.compile()
    # finger pads: torsional friction, more sliding friction
    for gid in range(m.ngeom):
        nm = m.geom(gid).name
        b = m.body(m.geom_bodyid[gid]).name
        if b.endswith("_finger") and m.geom_contype[gid]:
            m.geom_condim[gid] = 4
            m.geom_friction[gid] = [PAD_FRICTION, PAD_TORSION, 0.0001]
        if nm.endswith("_lever_g") or nm.endswith("_handle") or nm.endswith("_body"):
            m.geom_condim[gid] = 4
            m.geom_friction[gid] = [1.0, 0.01, 0.0001]
    return m


# ----------------------------------------------------------------------------------------------------------------
class Arm:
    """One Panda: an operational-space controller toward a target pose of the grip site, or a joint step."""

    def __init__(self, m, d, p):
        self.m, self.d, self.p = m, d, p
        self.jq = np.array([m.joint(p + "_joint%d" % i).qposadr[0] for i in range(1, 8)])
        self.jv = np.array([m.joint(p + "_joint%d" % i).dofadr[0] for i in range(1, 8)])
        self.act = np.array([m.actuator(p + "_torq_j%d" % i).id for i in range(1, 8)])
        self.gact = m.actuator(p + "_grip").id
        self.gq = [m.joint(p + "_finger_joint1").qposadr[0], m.joint(p + "_finger_joint2").qposadr[0]]
        self.site = m.site(p + "_grip_site").id
        self.ft_site = m.site(p + "_ft").id
        self.f_adr = m.sensor(p + "_ft_force").adr[0]
        self.t_adr = m.sensor(p + "_ft_torque").adr[0]
        self.cam = m.camera(p + "_wrist").id
        self._jp, self._jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        self._M = np.zeros((m.nv, m.nv))
        self.q_null = HOME_Q.copy()
        self.grip_cmd = -1.0
        self.mode = "osc"
        self.q_goal = None
        root = m.body(p + "_link0").id
        self.bodies = set()
        for b in range(m.nbody):
            k = b
            while k > 0:
                if k == root:
                    self.bodies.add(b)
                    break
                k = m.body_parentid[k]

    def init_targets(self):
        self.target_pos = self.d.site_xpos[self.site].copy()
        self.target_R = self.d.site_xmat[self.site].reshape(3, 3).copy()

    def pos(self):
        return self.d.site_xpos[self.site].copy()

    def R(self):
        return self.d.site_xmat[self.site].reshape(3, 3).copy()

    def gripper(self):
        return float(self.d.qpos[self.gq[0]] + self.d.qpos[self.gq[1]])

    def wrench_world(self):
        d = self.d
        R = d.site_xmat[self.ft_site].reshape(3, 3)
        return np.concatenate([R @ d.sensordata[self.f_adr:self.f_adr + 3], R @ d.sensordata[self.t_adr:self.t_adr + 3]])

    def command(self, a):
        d = self.d
        if len(a) == 8:
            q = d.qpos[self.jq].copy()
            self.q_goal = q + np.clip(np.asarray(a[:7], float) - q, -MAX_DQ, MAX_DQ)
            self.q_null = np.asarray(a[:7], float).copy()
            self.mode = "joint"
            grip = a[7]
        else:
            if len(a) == 14:
                self.q_null = np.asarray(a[7:14], float).copy()
            dp, dr = np.asarray(a[0:3], float), np.asarray(a[3:6], float)
            n = np.linalg.norm(dp)
            if n > MAX_DPOS:
                dp = dp * (MAX_DPOS / n)
            n = np.linalg.norm(dr)
            if n > MAX_DROT:
                dr = dr * (MAX_DROT / n)
            self.target_pos = d.site_xpos[self.site] + dp
            self.target_R = axis_angle_mat(dr) @ d.site_xmat[self.site].reshape(3, 3)
            self.mode = "osc"
            grip = a[6]
        self.grip_cmd = float(np.clip(grip, -1, 1))

    def torque(self):
        m, d = self.m, self.d
        if self.mode == "joint":
            tau = 200.0 * (self.q_goal - d.qpos[self.jq]) - 20.0 * d.qvel[self.jv] + d.qfrc_bias[self.jv]
        else:
            mujoco.mj_jacSite(m, d, self._jp, self._jr, self.site)
            J = np.vstack([self._jp, self._jr])[:, self.jv]
            mujoco.mj_fullM(m, d, self._M)
            Mq = self._M[np.ix_(self.jv, self.jv)]
            R = d.site_xmat[self.site].reshape(3, 3)
            ep = self.target_pos - d.site_xpos[self.site]
            Re = self.target_R @ R.T
            eo = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
            v = J @ d.qvel[self.jv]
            kp, ko = 400.0, 200.0
            acc = np.concatenate([kp * ep - 2.0 * math.sqrt(kp) * v[:3], ko * eo - 2.0 * math.sqrt(ko) * v[3:]])
            Minv = np.linalg.inv(Mq)
            Lam = np.linalg.inv(J @ Minv @ J.T + 1e-2 * np.eye(6))
            tau = J.T @ (Lam @ acc) + d.qfrc_bias[self.jv]
            Jbar = Minv @ J.T @ Lam
            N = np.eye(7) - J.T @ Jbar.T
            dq = d.qvel[self.jv]
            tau += N @ (5.0 * (self.q_null - d.qpos[self.jq]) - 6.0 * dq)
            tau -= 0.5 * dq
            over = np.abs(dq) - 1.5
            tau -= np.where(over > 0, 30.0 * over * np.sign(dq), 0.0)
        return tau

    def grip_ctrl(self):
        return 0.02 * (1.0 - self.grip_cmd)          # tendon length = half the opening: -1 -> 80 mm, +1 -> 0


# ----------------------------------------------------------------------------------------------------------------
class World:
    def __init__(self, unit, episode):
        self.unit, self.episode = unit, episode
        self.m = m = build(unit, episode)
        self.d = d = mujoco.MjData(m)
        self.rp, self.rR = rack_pose(unit)
        self.arms = {s: Arm(m, d, s) for s in BASES}
        for s, a in self.arms.items():
            d.qpos[a.jq] = HOME_Q
            d.qpos[a.gq] = 0.04
        self.nslot = NROW * NCOL
        self.caddies = ["c%d" % k for k in range(self.nslot)] + ["s%d" % j for j in range(len(SPARE_AT))]
        self.fans = ["f%d" % i for i in range(len(FAN_YS))] + ["fs"]
        self.cb = {n: m.body(n).id for n in self.caddies}
        self.fb = {n: m.body(n).id for n in self.fans}
        self.lev_j = {n: m.joint(n + "_lever_j").id for n in self.caddies}
        self.lev_q = {n: m.jnt_qposadr[self.lev_j[n]] for n in self.caddies}
        self.lev_v = {n: m.jnt_dofadr[self.lev_j[n]] for n in self.caddies}
        self.btn_q = {n: m.jnt_qposadr[m.joint(n + "_btn_j").id] for n in self.caddies}
        self.led = {n: m.geom(n + "_led").id for n in self.caddies + self.fans}
        # latch equalities (one per caddy): added at build? use joint limits instead: we emulate the latch by a
        # stiff spring + damping on the lever joint when latched (cheap and stable)
        self.latched = {n: n.startswith("c") for n in self.caddies}
        self.latch_eq = {n: m.equality(n + "_latch").id for n in self.caddies}
        self.armed = {n: True for n in self.caddies}
        for n in self.caddies:
            d.eq_active[self.latch_eq[n]] = 1 if self.latched[n] else 0
        self.seated = {n: n.startswith("c") for n in self.caddies}
        self.fan_seated = {n: n.startswith("f") and n != "fs" for n in self.fans}
        self.healthy = {n: (n.startswith("c") and int(n[1:]) not in episode["failed_slots"]) for n in self.caddies}
        self.broken = {n: False for n in self.caddies}
        self.stop_torque = {n: 0.0 for n in self.caddies}
        self.break_count = {n: 0 for n in self.caddies}
        self.where = {n: (int(n[1:]) if n.startswith("c") else None) for n in self.caddies}   # slot index or None
        self.fan_where = {n: (int(n[1:]) if n != "fs" else None) for n in self.fans}
        self.fan_empty_s = [0.0] * len(FAN_YS)
        self.events = []
        self.lost = None
        self.strike = 0
        self.t = 0
        self.sim_t = 0.0
        self.chassis_geoms = set(g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body("rack").id)
        self.robot_bodies = set().union(*[a.bodies for a in self.arms.values()])
        self.is_chassis = np.array([g in self.chassis_geoms for g in range(m.ngeom)])
        self.is_robot = np.array([int(m.geom_bodyid[g]) in self.robot_bodies for g in range(m.ngeom)])
        self.cid = np.array([self.cb[n] for n in self.caddies])
        self.fid = np.array([self.fb[n] for n in self.fans])
        self.lever_of_joint = {int(self.lev_j[n]): n for n in self.caddies}
        self.lever_body = {n: m.body(n + "_lever").id for n in self.caddies}
        self.finger_bodies = set(m.body(p + "_" + f).id for p in BASES for f in ("left_finger", "right_finger"))
        self.slot_frames = []
        for k in range(self.nslot):
            c = slot_center_R(k)
            self.slot_frames.append(self.rp + self.rR @ np.array([c[0], c[1], CHASSIS["height"] + CAD_PROTRUDE]))
        self.fan_frames = []
        for i in range(len(FAN_YS)):
            c = fan_center_R(i)
            self.fan_frames.append(self.rp + self.rR @ np.array([c[0], c[1], CHASSIS["height"] - FAN_BAY_DEPTH]))
        self.flicker = np.random.default_rng(int(episode["flicker_seed"]))
        mujoco.mj_forward(m, d)
        for a in self.arms.values():
            a.init_targets()
        # settle the parts with the arms holding
        for _ in range(100):
            self._substep()
        self.sim_t = 0.0

    # --- geometry helpers -------------------------------------------------------------------------------------
    def to_R(self, p):
        return self.rR.T @ (np.asarray(p, float) - self.rp)

    def caddy_pose(self, n):
        b = self.cb[n]
        return self.d.xpos[b].copy(), self.d.xmat[b].reshape(3, 3).copy()

    def _locate(self):
        """Every caddy's and fan's place relative to the bays, vectorised: (caddy slot or -1, caddy depth out of
        home), (fan bay or -1, fan height above its seat)."""
        d = self.d
        P = d.xpos[self.cid]
        q = (P - self.rp) @ self.rR
        az = d.xmat[self.cid].reshape(-1, 3, 3)[:, :, 2] @ self.rR
        row = np.rint((q[:, 0] - ROW_X0) / PITCH_X).astype(int)
        xc = ROW_X0 + row * PITCH_X
        col = np.rint((NCOL - 1) / 2.0 - q[:, 1] / PITCH_Y).astype(int)
        yc = -(col - (NCOL - 1) / 2.0) * PITCH_Y
        ok = ((az[:, 2] > 0.95) & (col >= 0) & (col < NCOL) & (row >= 0) & (row < NROW)
              & (np.abs(q[:, 0] - xc) < SLOT_T / 2) & (np.abs(q[:, 1] - yc) < SLOT_W / 2)
              & (q[:, 2] < CHASSIS["height"] + CAD_D + 0.02))
        slot = np.where(ok, row * NCOL + col, -1)
        depth = q[:, 2] - (CHASSIS["height"] + CAD_PROTRUDE)
        F = d.xpos[self.fid]
        qf = (F - self.rp) @ self.rR
        bay = np.rint((qf[:, 1] - FAN_YS[0]) / (FAN_YS[-1] - FAN_YS[0]) * (len(FAN_YS) - 1)).astype(int)
        bay_y = np.array(FAN_YS)[np.clip(bay, 0, len(FAN_YS) - 1)]
        okf = ((bay >= 0) & (bay < len(FAN_YS)) & (np.abs(qf[:, 0] - FAN_X) < FAN_BAY_W / 2)
               & (np.abs(qf[:, 1] - bay_y) < FAN_BAY_W / 2) & (qf[:, 2] < CHASSIS["height"] + 0.03))
        bay = np.where(okf, bay, -1)
        h = qf[:, 2] - (CHASSIS["height"] - FAN_BAY_DEPTH)
        return slot, depth, bay, h

    def caddy_in_slot(self, n):
        slot, depth, _b, _h = self._locate()
        i = self.caddies.index(n)
        return None if slot[i] < 0 else (int(slot[i]), float(depth[i]))

    # --- mechanisms -------------------------------------------------------------------------------------------
    def _mechanisms(self):
        d, U = self.d, self.unit
        d.xfrc_applied[self.cid] = 0.0
        d.xfrc_applied[self.fid] = 0.0
        slot, depth, bay, h = self._locate()
        ax = -self.rR[:, 2]                # into the slot: down
        for i, n in enumerate(self.caddies):
            b = self.cid[i]
            k, x = int(slot[i]), float(depth[i])
            # lever: latch, spring, closing detent
            th = float(d.qpos[self.lev_q[n]])
            dof = self.lev_v[n]
            if self.latched[n]:
                d.qfrc_applied[dof] = -0.03                        # the latch (an equality) holds it shut
                if d.qpos[self.btn_q[n]] > BTN_TRIP:
                    self.latched[n] = False
                    self.armed[n] = False
                    d.eq_active[self.latch_eq[n]] = 0
                    self.events.append((self.t, "release", n))
            else:
                tau = -LEV_SPRING * (th - LEV_POP)
                if th < LEV_DET_ANGLE:
                    tau += U["lever_detent"] * math.sin(math.pi * th / LEV_DET_ANGLE)
                d.qfrc_applied[dof] = tau
                if th > LEV_DET_ANGLE:
                    self.armed[n] = True
                if self.armed[n] and th < LEV_LATCH_ANGLE and k >= 0 and x < 0.001 and self.seated[n] and d.qpos[self.btn_q[n]] < 0.0005:
                    self.latched[n] = True
                    d.eq_active[self.latch_eq[n]] = 1
                    self.events.append((self.t, "latched", n))
            # connector
            if k < 0:
                self.seated[n] = False
                self.where[n] = None
                continue
            self.where[n] = k
            if self.seated[n]:
                if x > CONN_UNSEAT:
                    self.seated[n] = False
                    self.events.append((self.t, "unseated", n))
                    if self.healthy[n]:
                        self.lose("a healthy drive was unseated (%s)" % n)
                else:
                    f = min(U["pull_force"], 12000.0 * max(x, 0.0) + 1.0)
                    if self.latched[n]:
                        f = min(4.0 * U["pull_force"], 40000.0 * max(x, 0.0) + 2.0)
                    d.xfrc_applied[b, :3] = f * ax            # pulls it home
            elif x < CONN_SPAN:
                f = U["ins_force"] * min(1.0, (CONN_SPAN - x) / 0.003)
                d.xfrc_applied[b, :3] = -f * ax               # resists the insertion
                if x < CONN_SEAT:
                    self.seated[n] = True
                    self.events.append((self.t, "seated", n))
        up = self.rR[:, 2]
        for i, n in enumerate(self.fans):
            b = self.fid[i]
            k, z = int(bay[i]), float(h[i])
            if k < 0:
                self.fan_seated[n] = False
                self.fan_where[n] = None
                continue
            self.fan_where[n] = k
            if self.fan_seated[n]:
                if z > CONN_UNSEAT:
                    self.fan_seated[n] = False
                    self.events.append((self.t, "fan_unseated", n))
                else:
                    d.xfrc_applied[b, :3] = -min(U["fan_pull"], 10000.0 * max(z, 0.0) + 1.0) * up
            elif z < CONN_SPAN:
                d.xfrc_applied[b, :3] = U["fan_ins"] * min(1.0, (CONN_SPAN - z) / 0.003) * up
                if z < CONN_SEAT:
                    self.fan_seated[n] = True
                    self.events.append((self.t, "fan_seated", n))

    def _checks(self):
        m, d = self.m, self.d
        # lever break: the open stop's constraint torque
        nef = d.nefc
        if nef:
            typ = d.efc_type[:nef]
            sel = np.nonzero(typ == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)[0]
            seen = set()
            for i in sel:
                j = int(d.efc_id[i])
                n = self.lever_of_joint.get(j)
                if n is None:
                    continue
                if float(d.qpos[self.lev_q[n]]) > LEV_STOP - 0.05:
                    self.stop_torque[n] = max(self.stop_torque[n], abs(float(d.efc_force[i])))
                    if abs(d.efc_force[i]) > self.unit["lever_break"]:
                        seen.add(n)
            for n in self.caddies:
                if (n in seen and self.where[n] is None and self._touched(self.lever_body[n])
                        and not self._touched(self.cb[n]) and not self._resting(self.cb[n])):
                    # the caddy hangs from its lever alone
                    self.break_count[n] += 1
                    if self.break_count[n] >= BREAK_SUBSTEPS and not self.broken[n]:
                        self.broken[n] = True
                        self.lose("a lever was forced past its stop and broke (%s)" % n)
                else:
                    self.break_count[n] = 0
        # strikes: a robot geom pressing on the chassis
        hard = False
        nc = d.ncon
        if nc:
            g1 = d.contact.geom1[:nc]
            g2 = d.contact.geom2[:nc]
            c1, c2 = self.is_chassis[g1], self.is_chassis[g2]
            r1, r2 = self.is_robot[g1], self.is_robot[g2]
            hit = np.nonzero((c1 & r2) | (c2 & r1))[0]
            for i in hit:
                adr = d.contact.efc_address[i]
                if adr >= 0 and abs(d.efc_force[adr]) > STRIKE_N:
                    hard = True
        self.strike = self.strike + 1 if hard else 0
        if self.strike >= STRIKE_SUBSTEPS:
            self.lose("a robot link struck the server hard")

    def _touched(self, lb):
        """True when a robot's finger touches body lb."""
        m, d = self.m, self.d
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
            if (b1 == lb and b2 in self.finger_bodies) or (b2 == lb and b1 in self.finger_bodies):
                return True
        return False

    def _resting(self, b):
        """True when body b touches anything welded to the world (a table, the holder, the server)."""
        m, d = self.m, self.d
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
            if (b1 == b and m.body_weldid[b2] == 0) or (b2 == b and m.body_weldid[b1] == 0):
                return True
        return False

    def lose(self, why):
        if self.lost is None:
            self.lost = why
            self.events.append((self.t, "lost", why))

    def _substep(self):
        m, d = self.m, self.d
        for a in self.arms.values():
            d.ctrl[a.act] = a.torque()
            d.ctrl[a.gact] = a.grip_ctrl()
        self._mechanisms()
        mujoco.mj_step(m, d)
        self._checks()
        self.sim_t += TIMESTEP

    def step(self, cmds):
        for s, a in cmds.items():
            self.arms[s].command(a)
        for _ in range(SUBSTEPS):
            self._substep()
        self.t += 1
        self._slow_checks()
        if self.unstable():
            raise FloatingPointError("unstable")

    def _slow_checks(self):
        d = self.d
        occupied = [False] * len(FAN_YS)
        for n in self.fans:
            if self.fan_seated[n] and self.fan_where[n] is not None:
                occupied[self.fan_where[n]] = True
        for i in range(len(FAN_YS)):
            self.fan_empty_s[i] = 0.0 if occupied[i] else self.fan_empty_s[i] + DT_CTRL
            if self.fan_empty_s[i] > self.unit["overheat_s"]:
                self.lose("overheat: fan bay %d stayed empty too long" % i)
        for n in self.caddies:
            if d.xipos[self.cb[n]][2] < DROP_Z:             # its centre of mass
                self.lose("a drive caddy was dropped (%s)" % n)
        for n in self.fans:
            if d.xipos[self.fb[n]][2] < DROP_Z:
                self.lose("a fan module was dropped (%s)" % n)

    def unstable(self):
        d = self.d
        if not (np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel))):
            return True
        if any(np.max(np.abs(d.qvel[a.jv])) > MAX_ARM_SPEED for a in self.arms.values()):
            return True
        v = np.zeros(6)
        for b in list(self.cb.values()) + list(self.fb.values()):
            mujoco.mj_objectVelocity(self.m, d, mujoco.mjtObj.mjOBJ_BODY, b, v, 0)
            if np.linalg.norm(v[3:]) > MAX_OBJECT_SPEED:
                return True
        return False

    # --- status LEDs -------------------------------------------------------------------------------------------
    def set_leds(self):
        m, U, E = self.m, self.unit, self.episode
        g = U["led_gain"]
        green = np.array([0.15, 1.0, 0.25]) * g
        amber = np.array([1.0, 0.55, 0.0]) * g
        off = np.array([0.05, 0.05, 0.05])
        on_blink = ((self.sim_t + E["blink_phase"]) % 1.0) < 0.5
        failed = set(E["failed_slots"])
        for n in self.caddies:
            if n.startswith("c") and int(n[1:]) in failed:
                col = amber if (not U["amber_blink"] or on_blink) else off
            elif self.seated[n] and self.where[n] is not None:
                col = green if self.flicker.uniform() > 0.15 else off
            else:
                col = off
            m.geom_rgba[self.led[n], :3] = col
            m.geom_matid[self.led[n]] = -1
        for n in self.fans:
            if n == "f%d" % E["failed_fan"]:
                col = amber if (not U["amber_blink"] or on_blink) else off
            elif self.fan_seated[n] and self.fan_where[n] is not None:
                col = green
            else:
                col = off
            m.geom_rgba[self.led[n], :3] = col

    # --- status for the end and the score ----------------------------------------------------------------------
    def in_bin(self, p):
        return (abs(p[0] - BIN_C[0]) < BIN_HALF[0] and abs(p[1] - BIN_C[1]) < BIN_HALF[1]
                and TABLE_TOP < p[2] < TABLE_TOP + BIN_H + 0.03)

    def status(self):
        d, E = self.d, self.episode
        failed = set(E["failed_slots"])
        ok_slots = []
        for k in range(self.nslot):
            occ = [n for n in self.caddies if self.where[n] == k and self.seated[n]]
            good = len(occ) == 1 and self.latched[occ[0]] and not self.broken[occ[0]]
            if k in failed:
                good = good and occ[0].startswith("s")
            else:
                good = good and occ[0] == "c%d" % k
            ok_slots.append(bool(good))
        fans_ok = []
        for i in range(len(FAN_YS)):
            occ = [n for n in self.fans if self.fan_where[n] == i and self.fan_seated[n]]
            good = len(occ) == 1 and (occ[0] == "fs" if i == E["failed_fan"] else occ[0] == "f%d" % i)
            fans_ok.append(bool(good))
        binned = [self.in_bin(d.xipos[self.cb["c%d" % k]]) for k in sorted(failed)]      # centres of mass
        fan_binned = self.in_bin(d.xipos[self.fb["f%d" % E["failed_fan"]]])
        return {"slots_ok": ok_slots, "fans_ok": fans_ok, "failed_in_bin": binned, "fan_in_bin": bool(fan_binned),
                "lost": self.lost, "broken": [n for n, v in self.broken.items() if v]}

    def complete(self):
        s = self.status()
        return (s["lost"] is None and all(s["slots_ok"]) and all(s["fans_ok"]) and all(s["failed_in_bin"])
                and s["fan_in_bin"])

    def truth(self):
        d = self.d
        out = {"rack_pos": self.rp.tolist(), "rack_R": self.rR.tolist(), "failed_slots": list(self.episode["failed_slots"]),
               "failed_fan": int(self.episode["failed_fan"]),
               "slot_frames": [p.tolist() for p in self.slot_frames], "fan_frames": [p.tolist() for p in self.fan_frames],
               "caddies": {}, "fans": {}, "unit": {k: v for k, v in self.unit.items()}}
        for n in self.caddies:
            p, R = self.caddy_pose(n)
            lv = self.m.body(n + "_lever").id
            out["caddies"][n] = {"pos": p.tolist(), "R": R.tolist(), "lever": float(d.qpos[self.lev_q[n]]),
                                 "lever_pos": d.xpos[lv].tolist(), "latched": self.latched[n],
                                 "seated": self.seated[n], "slot": self.where[n]}
        for n in self.fans:
            out["fans"][n] = {"pos": d.xpos[self.fb[n]].tolist(), "R": d.xmat[self.fb[n]].reshape(3, 3).tolist(),
                              "seated": self.fan_seated[n], "bay": self.fan_where[n]}
        out["fan_empty_s"] = list(self.fan_empty_s)
        out["tools"] = {s: {"pos": a.pos().tolist(), "R": a.R().tolist()} for s, a in self.arms.items()}
        out["complete"] = bool(self.complete())
        return out


# ----------------------------------------------------------------------------------------------------------------
# rendering
def camera_size(cam):
    if cam == "overhead":
        c = CAM_NOM["overhead"]
        return c["w"], c["h"], c["fovy"]
    return WRIST_NOM["w"], WRIST_NOM["h"], WRIST_NOM["fovy"]


def mj_camera(cam):
    return cam if cam == "overhead" else cam.split("_")[1] + "_wrist"


def depth_sensor(depth, rgb, rng):
    """An active-stereo depth reading: noise growing with the square of the distance, 1 mm quantization, holes at
    depth edges, on very dark surfaces and at random; 0 means no reading."""
    z = depth.astype(np.float64)
    sigma = 0.0008 + 0.0025 * z * z
    z = z + rng.normal(size=z.shape) * sigma
    z = np.round(z * 1000.0) / 1000.0
    gy, gx = np.gradient(depth.astype(np.float64))
    edge = np.hypot(gx, gy) > 0.01
    lum = rgb.astype(np.float64).mean(axis=2)
    dark = (lum < 22.0) & (rng.uniform(size=z.shape) < 0.7)
    rnd = rng.uniform(size=z.shape) < 0.02
    hole = edge | dark | rnd | (depth > 3.0)
    z[hole] = 0.0
    return (z * 1000.0).astype(np.uint16)          # mm


def render(world, cam, rng, cache):
    w, h, _ = camera_size(cam)
    key = (w, h)
    try:
        r = cache.get(key)
        if r is None:
            r = mujoco.Renderer(world.m, h, w)
            cache[key] = r
        world.set_leds()
        r.disable_depth_rendering()
        r.update_scene(world.d, camera=mj_camera(cam))
        rgb = r.render().copy()
        r.enable_depth_rendering()
        r.update_scene(world.d, camera=mj_camera(cam))
        dep = r.render().copy()
        r.disable_depth_rendering()
    except Exception as exc:  # noqa: BLE001
        raise RenderError("%s: %s" % (type(exc).__name__, exc))
    noise = rng.normal(0.0, 2.0, rgb.shape)
    rgb = np.clip(rgb.astype(np.float64) + noise, 0, 255).astype(np.uint8)
    return rgb, depth_sensor(dep, rgb, rng)


def close_renderers(cache):
    for r in list(cache.values()):
        try:
            r.close()
        except Exception:  # noqa: BLE001
            pass
    cache.clear()
