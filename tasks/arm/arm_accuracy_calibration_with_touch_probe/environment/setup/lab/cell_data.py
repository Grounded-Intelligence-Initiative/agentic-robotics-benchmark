"""cell_data.py: what the cell's documents state, as numbers. Public: the agent has this file.

The arm's accuracy model is arm_model.py (the manufacturer's first-order model); the defects a
real unit may have beyond it are described in defects.md. Here: the datum spheres as surveyed,
the probe, the tools' certificates (process_c's pin included), the cell's timing, the ranges within
which each cell's heating, noise and condition lie, the wrist camera's figures (camera.md describes
the camera and its images), a session's budget, the tolerances inside which every unit lies, the
gauge plate's certificate and the ranges of its pose, and the jobs' rules.

The gauge plate is a finished part: in a session nothing may touch it. A stylus that touches it or
one of its shims, in a move or in a probe (guarded or not), damages the probe as a collision does
(collided, damaged); it can be seen only, in the wrist camera's images. plate_envelope() bounds the
region it may occupy.

Figures given as a range differ from cell to cell (unit to unit) and are not told for any one
cell; how they are distributed inside the range is not stated.
"""
import numpy as np

import arm_model as am

D = am.D
R_SPHERE = 0.019            # datum sphere radius (certified)
R_STYLUS = 0.002            # stylus ball radius (certified)
R_POST = 0.006              # post under each sphere
T_AMBIENT = 20.0
# Heating (cell to cell): a link warms as its own joint turns, by roughly HEAT_NOMINAL K per radian
# (the factor differs per joint and per unit, within HEAT_FACTOR_RANGE), a little more (not documented
# how much) as its neighbouring joints turn, and cools towards T_AMBIENT with one time constant per unit
# within TAU_COOL_RANGE: during a command (or a job's row) of duration d in which joint j travels
# |dq_j|, dT_i/dt = sum_j H_ij |dq_j| / d - (T_i - T_AMBIENT) / tau (cell.md, section 7). Temperatures
# are read by the link sensors (SIGMA_TEMP).
HEAT_NOMINAL = 0.10         # K per radian the link's joint turns, nominal
HEAT_FACTOR_RANGE = (0.6, 1.6)
TAU_COOL_RANGE = (400.0, 900.0)     # s
MOVE_SPEED_MAX = 2.0        # rad/s, any joint, in `move`
PROBE_SPEED = 0.01          # rad/s, the fastest joint, in `probe`
SETTLE = 0.3                # s after every move and every probe
TOOL_CHANGE = 30.0          # s
SIGMA_REPEAT_RANGE = (1.5e-5, 3.0e-5)   # m per axis, per unit: where the tip really is after each arrival (not modelled)
SIGMA_TEMP = 0.05           # K, temperature readings
ENCODER_STEP = 2 * np.pi / 2 ** 22
FALSE_TRIGGER_RANGE = (0.01, 0.10)      # per unit: probability that a probe triggers before contact
S_THRESHOLD = 1e-4          # rad: a joint that moved less keeps its direction
KNOCKED_SPHERES = (0, 4)                # how many of the twelve datum spheres a unit's cell has knocked
KNOCK_MIN, KNOCK_MAX = 0.5e-3, 3.0e-3   # m: how far a knocked datum sphere stands from its survey
BUDGET = {"time_s": 1000.0, "probes": 120, "images": 51}
# The lab: one measuring session per graded unit, then one re-qualification session (below, the service). The
# practice unit has PRACTICE_SESSIONS measuring session, for rehearsing a measuring session against the lab; it is
# the shop's demonstration unit and is not due for service: it is never serviced and has no re-qualification
# session. Acceptance checks before grading: PRACTICE_CHECKS, none on any unit. Acceptance is the customer's
# inspection (the grader) after delivery; the lab has no acceptance rig and no execute operation.
PRACTICE_SESSIONS = 1
PRACTICE_CHECKS = 0

# ---- tolerances of a unit (each value uniform inside its range, independently) ---------------
TOL_XYZ = 1.0e-3            # m, every joint's xyz against arm_model's nominal; the first joint's: TOL_BASE_XYZ
TOL_BASE_XYZ = 5.0e-3
TOL_RPY = np.array([0.1, 0.1, 0.3]) * D     # rad, every joint's rpy; the first joint's: TOL_BASE_RPY, all three
TOL_BASE_RPY = 0.5 * D
BACKLASH_MAX = np.array([0.06, 0.06, 0.06, 0.10, 0.10, 0.10]) * D   # b_j in [0.2, 1.0] of this
CYCLIC_MAX = np.array([0.015, 0.015, 0.015, 0.025, 0.025, 0.025]) * D           # amplitude of each of the two harmonics in [0.3, 1.0] of this, any phase
COMPLIANCE_MAX = np.array([1e-5, 1e-5, 1e-5, 4e-5, 4e-5, 4e-5])    # c_j in [0.3, 1.0] of this, rad / (N m)
KAPPA_RANGE = (0.5e-5, 5.0e-5)              # 1 / K: the effective growth of a link assembly, not of one material
TOL_CAMERA_XYZ = 3.0e-3                     # m, each axis of the camera's xyz against its nominal (below)
TOL_CAMERA_RPY = 1.0 * D                    # rad, each of the camera's rpy against its nominal

TOOLS = {
    "probe_a": dict(mass=1.0, com=[0.0, 0.0, 0.06], tip=[0.0, 0.0, 0.15]),
    "probe_b": dict(mass=5.0, com=[0.03, 0.0, 0.08], tip=[0.12, 0.0, 0.20]),
    "process_c": dict(mass=3.0, com=[0.01, -0.02, 0.10], tip=[0.05, -0.08, 0.28]),
}
# process_c carries a cylindrical pin (the jobs, below): its axis runs through the tip along the flange's +z axis;
# its end is a hemisphere whose apex is the tip; PIN_LENGTH from the apex back towards the flange. Only the pin
# can touch anything while process_c is mounted.
PIN_RADIUS = 0.00265        # m (diameter 5.30 mm, certified)
PIN_LENGTH = 0.030          # m, from the apex
MOUNTABLE = ("probe_a", "probe_b")
Q_HOME = np.array([0.0, -90.0, 90.0, 0.0, 45.0, 0.0]) * D


def surveyed_spheres():
    """Centres of the twelve datum spheres as the survey gives them (datum frame)."""
    az = np.linspace(-125, 125, 12) * D
    rad = np.where(np.arange(12) % 2 == 0, 0.50, 0.68)
    z = np.array([0.10, 0.40, 0.70])[np.arange(12) % 3]
    return np.stack([rad * np.cos(az), rad * np.sin(az), z], axis=1)


# ---- the wrist camera ---------------------------------------------------------------------------
# Fixed to the flange; its frame in the flange frame is arm_model's "camera" parameters (arm_model.py:
# Trans(xyz) Rx(rpy[0]) Ry(rpy[1]) Rz(rpy[2]); OpenCV convention, z = optical axis, x = image
# right, y = image down). Nominal: xyz (0, 0.06, 0.02) m, rpy (-90, 0, 0) deg, i.e. the camera
# looks along the flange's +y, image right = flange +x, image down = flange -z. The mounted tool
# is behind the camera and never in view. Each unit's camera lies within TOL_CAMERA_XYZ /
# TOL_CAMERA_RPY of nominal (uniform, independently per axis).
CAMERA_XYZ_NOMINAL = am.NOMINAL_CAMERA_XYZ
CAMERA_RPY_NOMINAL = am.NOMINAL_CAMERA_RPY
# The camera's certificate and what its images look like are in camera.md; its figures:
# Pixel (row i, column j) is the square u in [j - 1/2, j + 1/2], v in [i - 1/2, i + 1/2] (square
# pixels). Pinhole with a radial lens distortion about the principal point: a camera-frame point
# (X, Y, Z) is imaged at f * (X / Z, Y / Z) + (cx, cy) displaced radially by the distortion. The
# camera frame's z axis is the optical axis, imaged at the principal point (cx, cy).
IMAGE_W, IMAGE_H = 1280, 960
F_PX_NOMINAL = 1400.0       # px; each unit's focal length within F_TOL of it (not told per unit)
F_TOL = 0.01                # relative
CX_NOMINAL, CY_NOMINAL = (IMAGE_W - 1) / 2.0, (IMAGE_H - 1) / 2.0   # the image centre
PP_TOL = 8.0                # px: the principal point within this of the image centre, each axis (not calibrated)
DISTORTION_MAX_PX = 4.0     # px: the radial distortion displaces an image point by at most this at the corners (not calibrated)
IMAGE_TIME = 2.0            # s per image (the arm at rest; the link temperatures keep cooling)
# An image is taken where the last arrival left the arm: the camera is off its modelled pose by the
# same repeatability error (the unit's repeatability, a translation) as the tool tip after that
# arrival. Images count against BUDGET["images"]; refused when the probe is damaged or the time
# is used up. A re-qualification session has no images (BUDGET_SERVICE, below: the camera is off).


# ---- acceptance -------------------------------------------------------------------------------
# Each unit's N_ACCEPT acceptance conditions (q, s, temp; ACCEPT_TOOL mounted) are given to the
# agent in a file per unit. The submission per unit: the predicted tip of ACCEPT_TOOL for every
# condition (N_ACCEPT x 3, m, datum frame) and the predicted datum-frame position of each of the
# ACCEPT_CAM_POINTS for every condition (N_ACCEPT x 5 x 3). Errors: distances to the truth.
ACCEPT_P95 = 0.20e-3
ACCEPT_MAX = 0.40e-3
N_ACCEPT = 400
ACCEPT_TOOL = "process_c"
ACCEPT_REGION = dict(radius=(0.35, 0.85), z=(0.05, 0.90), azimuth=130 * D)      # where the tool tip is, nominally
ACCEPT_TEMP = ((20.0, 22.0), (35.0, 55.0))      # degrees C: half of the conditions cold, half warm, per link
# The camera test, on the same conditions (the same q, s, temp, ACCEPT_TOOL mounted): these points,
# given in the camera frame (m), are placed in the datum frame by the true unit; the submission predicts those positions;
# the error is the distance. All N_ACCEPT x 5 errors count.
ACCEPT_CAM_POINTS = np.array([
    [0.00, 0.00, 0.35],
    [0.12, 0.09, 0.30],
    [-0.12, 0.09, 0.40],
    [0.15, -0.10, 0.45],
    [-0.10, -0.08, 0.25],
])
ACCEPT_CAM_P95 = 0.40e-3
ACCEPT_CAM_MAX = 0.60e-3


# ---- the gauge plate ----------------------------------------------------------------------------
# A finished part in every cell (the practice unit's included), at its own pose (per unit, not told; the ranges
# below). A rectangular block with N_BORES blind bores in its top face, standing on three shims on the table.
# Plate frame: origin at the centre of the top face, x along the length, y along the width, z out of the top face
# (the plate occupies x in +-L/2, y in +-W/2, z in [-T, 0]). Certificate (exact):
PLATE_SIZE = (0.160, 0.100, 0.020)          # m: L, W, T
N_BORES = 8
BORE_RADIUS = 0.00290                       # m (diameter 5.80 mm)
BORE_DEPTH = 0.012                          # m, flat bottom, from the top face
BORES = np.array([                          # plate-frame x, y of each bore's axis (m); the axis is the plate's z
    [-0.065, -0.030], [-0.062, 0.028], [-0.035, -0.002], [-0.010, 0.032],
    [0.000, -0.028], [0.028, 0.010], [0.052, -0.030], [0.064, 0.022]])
SHIMS = np.array([[-0.060, -0.035], [-0.060, 0.035], [0.060, 0.000]])  # plate-frame x, y of the shims' axes (m)
SHIM_RADIUS = 0.008                         # m: vertical cylinders from the table up to the plate's bottom face
# The pose (per unit): the top face's centre at (x, y) = PLATE_XY_NOMINAL + up to PLATE_XY_TOL on each axis; the
# plate frame's rotation plate_rotation(yaw, tilt_x, tilt_y) with yaw within PLATE_YAW_TOL of PLATE_YAW_NOMINAL
# and each tilt within PLATE_TILT_MAX; the bottom face's centre PLATE_LIFT above the table. Each uniform in its
# range, independently.
PLATE_XY_NOMINAL = (0.230, -0.352)
PLATE_XY_TOL = 0.030                        # m, each axis
PLATE_YAW_NOMINAL = 33.0 * D
PLATE_YAW_TOL = 15.0 * D
PLATE_TILT_MAX = 2.0 * D                    # rad, about the datum x axis and about the datum y axis
PLATE_LIFT = (0.006, 0.010)                 # m (the shims' heights differ: that tilts the plate)


def plate_rotation(yaw, tilt_x, tilt_y):
    """The plate frame's rotation in the datum frame: Rx(tilt_x) Ry(tilt_y) Rz(yaw) (datum-fixed axes: the yaw
    about the vertical first, then the tilts)."""
    return am._rx(tilt_x) @ am._ry(tilt_y) @ am._rz(yaw)


def plate_frame(x, y, lift, yaw, tilt_x, tilt_y):
    """Rotation R (datum <- plate) and origin O (the top face's centre, datum frame) of a plate pose: the top
    face's centre at (x, y), the bottom face's centre `lift` above the table."""
    R = plate_rotation(yaw, tilt_x, tilt_y)
    return R, np.array([x, y, lift + PLATE_SIZE[2] * R[2, 2]])


def plate_envelope():
    """A region that holds the plate and its shims at every pose of the ranges: the vertical cylinder of this
    radius about PLATE_XY_NOMINAL, from the table up to this height (m), each rounded up to the millimetre."""
    half = np.hypot(PLATE_SIZE[0] / 2, PLATE_SIZE[1] / 2)
    slope = np.sin(np.hypot(PLATE_TILT_MAX, PLATE_TILT_MAX))
    radius = half + np.hypot(PLATE_XY_TOL, PLATE_XY_TOL) + PLATE_SIZE[2] * slope
    top = PLATE_LIFT[1] + PLATE_SIZE[2] + half * slope
    return float(np.ceil(radius * 1e3) / 1e3), float(np.ceil(top * 1e3) / 1e3)


# ---- the jobs -----------------------------------------------------------------------------------
# Each graded unit's submission carries the cold job and the warm job (and the serviced job, below): each 1 to JOB_MAX_ROWS
# joint targets (rad, inside the joint limits). The grader runs each once on the true unit, each with its own
# repeatability draws: process_c mounted, the arm at Q_HOME, s = +1, no repeatability error yet, every link at
# T_AMBIENT (the cold job) or at the unit's warm start temperatures (the warm job: job_warm_temp in the unit's
# conditions file, six numbers, each drawn per unit uniform in JOB_WARM_TEMP); each row a joint move at JOB_SPEED
# (rad/s, the joint that moves farthest) followed by SETTLE, the arm warming and cooling as in a session. A row's
# path: the joints on the
# straight line from where they are to the target; each direction sign s_j that the row changes (the rule of a
# session's move) goes linearly from its old value to its new one along the path (the backlash is taken up over
# the row), and the repeatability error (a fresh one at every arrival, as in a session) goes linearly from the
# last arrival's to this arrival's; the link temperatures are those at the row's start. At the row's end the pin
# is where the unit's kinematics put it at the target with the new signs, plus the new repeatability error.
# A bore is done when at the end of a row (the target, the new signs and repeatability error, the link temperatures
# at the row's start) the pin is in it: its apex between JOB_DEPTH below the bore's mouth (the top face's plane)
# and the bore's bottom, less than BORE_RADIUS from its axis, the pin's axis crossing the top face's plane less
# than BORE_RADIUS from the bore's axis (it leaves through the mouth), nothing having been touched so far; the first contact
# of the pin with anything (the plate, a shim, the table, a sphere, a post) jams that job, which stops there.
# The contact check is continuous along every row; a pin that comes within JOB_RESOLUTION of a surface may count
# as touching it. A program whose check needs more than JOB_MAX_POSES poses fails (the check examines few poses
# where the pin is far from every surface, many where it passes close to one for long). A job is passed when every
# bore is done; a unit's acceptance needs both.
JOB_TOOL = "process_c"
JOB_MAX_ROWS = 200
JOB_SPEED = 0.25            # rad/s
JOB_DEPTH = 0.008           # m
JOB_RESOLUTION = 1.0e-5     # m
JOB_MAX_POSES = 15000       # poses the contact check examines in one program (each job) at most
JOB_WARM_TEMP = (45.0, 55.0)    # degrees C: the range of each link's start temperature in the warm job (per unit,
                                # published exactly in its conditions file)


# ---- the service and the re-qualification session ---------------------------------------------------
# After a graded unit's measuring session the unit is serviced (the practice unit never is): the drive (motor and
# encoder) of one joint is replaced and its gear mesh set anew; the gear train itself stays. The serviced joint is one of SERVICE_JOINTS (per
# unit, not told); its zero offset (the third rpy angle of its frame, which acts about the joint's axis) changes by
# an amount of magnitude within SERVICE_ZERO, either sign (not told); its backlash is drawn anew within the range
# of a new unit's (BACKLASH_MAX, 0.2 to 1.0 times; not told). Nothing else of the unit or its cell changes. The
# unit's re-qualification session (one, after the measuring session) has the budget BUDGET_SERVICE; the serviced
# job (the submission's job_serviced) is run on the serviced unit by the cold job's rules. The wrist camera is off
# in the re-qualification session (its cable was unplugged for the drive's replacement; it is reconnected and
# re-verified only when the unit is released), so the session has no images: an image command is refused as
# no_images (as damaged or time_up when one of those applies first), nothing charged. The camera itself is
# unchanged by the service.
SERVICE_JOINTS = (0, 1, 2, 3, 4)
SERVICE_ZERO = (0.10 * D, 0.25 * D)         # rad: the magnitude of the change of the serviced joint's zero offset
BUDGET_SERVICE = {"time_s": 150.0, "probes": 12, "images": 0}
