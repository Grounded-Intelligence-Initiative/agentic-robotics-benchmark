"""The team's manipulator: a Franka Panda with a hook tool, as the range simulates it.

This file is the arm and nothing else: its model (robosuite 1.5.2's Panda, position servos, a
hook tool with a force/torque sensor on the flange), how a commanded target reaches the
servos, and the force/torque reading. The cart (`cart_model.py`) is built on it, and the
grader simulates the arm with exactly this file; what the arm meets in a live episode (the
packages, the latch closing on them, the drawers, the vessels, what a package is lost by) is
not in it.

World frame: z up, metres; the ground is z = 0; the arm's base is at the origin, facing +x.
"""

import importlib.util
import math
import os

import mujoco
import numpy as np

# --- timing -------------------------------------------------------------------------------
PHYSICS_DT = 0.002
SUBSTEPS = 10
CONTROL_DT = PHYSICS_DT * SUBSTEPS          # 0.02 s: the controller is asked at 50 Hz

# --- the arm ------------------------------------------------------------------------------
N_JOINTS = 7
HOME_Q = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])   # the Panda's ready pose
JOINT_RANGE = np.array([[-2.8973, 2.8973], [-1.7628, 1.7628], [-2.8973, 2.8973],
                        [-3.0718, -0.0698], [-2.8973, 2.8973], [-0.0175, 3.7525],
                        [-2.8973, 2.8973]])
MAX_TARGET_RATE = 1.5                        # rad/s: a target moves no faster than this
KP = np.array([4500.0, 4500.0, 3500.0, 3500.0, 2000.0, 1500.0, 500.0])
KV = 0.07 * KP                               # every servo lags its target by 0.07 s x speed
TORQUE_LIMIT = np.array([80.0, 80.0, 80.0, 80.0, 80.0, 12.0, 12.0])
ARMATURE = np.array([0.3, 0.3, 0.3, 0.3, 0.1, 0.1, 0.1])
TOOL_LENGTH = 0.14                           # flange to latch point, along the flange's axis

# --- the force/torque sensor ----------------------------------------------------------------
FORCE_SIGMA = 0.06                           # N, per axis, per reading
TORQUE_SIGMA = 0.006                         # N m
FORCE_BIAS = 0.4                             # N: a constant per axis, new in every episode
TORQUE_BIAS = 0.03                           # N m

# --- contacts -----------------------------------------------------------------------------
SOLREF = [0.01, 1]                           # MuJoCo solref of every contact: 10 ms, critically damped
SOLIMP = [0.95, 0.99, 0.001, 0.5, 2]


def robosuite_assets():
    """Folder of robosuite's model files (the image installs robosuite 1.5.2 for them)."""
    spec = importlib.util.find_spec("robosuite")
    return os.path.join(list(spec.submodule_search_locations)[0], "models", "assets")


def new_world(ground_friction=0.8):
    """An empty range: the simulation options, a light and the ground."""
    w = mujoco.MjSpec()
    w.compiler.degree = False
    w.compiler.autolimits = True
    w.option.timestep = PHYSICS_DT
    w.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    w.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    w.option.impratio = 5
    w.visual.global_.offwidth, w.visual.global_.offheight = 960, 720
    w.visual.headlight.ambient = [0.4, 0.4, 0.4]
    w.visual.headlight.diffuse = [0.5, 0.5, 0.5]
    w.worldbody.add_light(pos=[0.4, 0.0, 2.5], dir=[0, 0, -1], castshadow=0,
                          diffuse=[0.5, 0.5, 0.5])
    ground = w.worldbody.add_geom(name="ground", type=mujoco.mjtGeom.mjGEOM_PLANE,
                                  size=[8, 8, 0.1], rgba=[0.55, 0.55, 0.52, 1])
    ground.contype, ground.conaffinity = 4, 3
    ground.friction = [float(ground_friction), 0.01, 0.0005]
    return w


def add_arm(w, parent=None, pos=(0.0, 0.0, 0.0)):
    """Stand the arm at `pos` on `parent` (the world, at its origin, by default): robosuite's
    Panda with its torque motors
    replaced by position servos, a hook tool on the flange in place of a hand (the body
    `tool`, its sensor site `ft` at the flange and its latch point, the site `latch`), and the
    force/torque sensor between flange and tool."""
    robot = mujoco.MjSpec.from_file(os.path.join(robosuite_assets(), "robots", "panda", "robot.xml"))
    for a in list(robot.actuators):
        robot.delete(a)
    for c in list(robot.cameras):
        robot.delete(c)
    for g in robot.geoms:
        if g.contype or g.conaffinity:           # the links' collision shells
            g.contype, g.conaffinity = 2, 5
    tool = robot.body("right_hand").add_body(name="tool")
    rod = tool.add_geom(name="tool_rod", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                        fromto=[0, 0, 0, 0, 0, TOOL_LENGTH - 0.012], size=[0.012, 0, 0],
                        rgba=[0.8, 0.65, 0.15, 1])
    rod.mass, rod.contype, rod.conaffinity = 0.25, 2, 5
    tip = tool.add_geom(name="tool_tip", type=mujoco.mjtGeom.mjGEOM_SPHERE, pos=[0, 0, TOOL_LENGTH],
                        size=[0.01, 0, 0], rgba=[0.8, 0.65, 0.15, 1])
    tip.mass, tip.contype, tip.conaffinity = 0.05, 2, 5
    tool.add_site(name="ft", pos=[0, 0, 0], size=[0.005, 0, 0])
    tool.add_site(name="latch", pos=[0, 0, TOOL_LENGTH], size=[0.004, 0, 0], rgba=[1, 0, 0, 1])
    for b in robot.bodies:
        b.gravcomp = 1.0                         # the drives carry the arm's own weight
    for i, j in enumerate(robot.joints):
        j.armature = float(ARMATURE[i])
    (parent if parent is not None else w.worldbody).add_frame(
        pos=[float(v) for v in pos]).attach_body(robot.body("base"), "", "")
    for i in range(N_JOINTS):
        w.add_actuator(name="a{}".format(i + 1), target="joint{}".format(i + 1),
                       trntype=mujoco.mjtTrn.mjTRN_JOINT, gaintype=mujoco.mjtGain.mjGAIN_FIXED,
                       gainprm=[float(KP[i])] + [0.0] * 9, biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                       biasprm=[0.0, -float(KP[i]), -float(KV[i])] + [0.0] * 7,
                       forcerange=[-float(TORQUE_LIMIT[i]), float(TORQUE_LIMIT[i])])
    w.add_sensor(name="ft_force", type=mujoco.mjtSensor.mjSENS_FORCE,
                 objtype=mujoco.mjtObj.mjOBJ_SITE, objname="ft")
    w.add_sensor(name="ft_torque", type=mujoco.mjtSensor.mjSENS_TORQUE,
                 objtype=mujoco.mjtObj.mjOBJ_SITE, objname="ft")


def finish_contacts(w):
    """Every colliding geom gets the range's contact parameters (its own friction is kept)."""
    for g in w.geoms:
        if g.contype or g.conaffinity:
            g.solref, g.solimp, g.condim = SOLREF, SOLIMP, 4


def build_arm():
    """The MuJoCo model of the arm alone on the ground."""
    w = new_world()
    add_arm(w)
    finish_contacts(w)
    return w.compile()


def slew(target, q_cmd):
    """The target the drives move to this control step: the commanded one, clipped to the
    joints' ranges and to MAX_TARGET_RATE from the previous target."""
    q_cmd = np.clip(np.asarray(q_cmd, dtype=float), JOINT_RANGE[:, 0], JOINT_RANGE[:, 1])
    step_max = MAX_TARGET_RATE * CONTROL_DT
    return target + np.clip(q_cmd - target, -step_max, step_max)


def drive_target(previous, target, substep):
    """The servo set-point at a physics substep: the drives interpolate linearly from the
    previous control step's target to this one's, so a new command causes no torque step."""
    return previous + (target - previous) * ((substep + 1.0) / SUBSTEPS)


def read_wrench(data, bias, noise_seed, t):
    """The force/torque reading at control step t: `[fx, fy, fz, tx, ty, tz]` in the frame of
    the site `ft`, the force and torque the arm exerts on the tool and on whatever the tool
    holds, plus the episode's bias and the reading's noise."""
    wrench = np.concatenate([data.sensor("ft_force").data, data.sensor("ft_torque").data])
    noise = np.random.default_rng([int(noise_seed), int(t)]).standard_normal(6)
    return wrench + bias + noise * np.array([FORCE_SIGMA] * 3 + [TORQUE_SIGMA] * 3)


def draw_bias(rng):
    return rng.uniform(-1.0, 1.0, 6) * np.array([FORCE_BIAS] * 3 + [TORQUE_BIAS] * 3)
