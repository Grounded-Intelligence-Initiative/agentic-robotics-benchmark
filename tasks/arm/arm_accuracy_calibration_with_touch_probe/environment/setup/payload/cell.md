# Calibration cell: data sheet

The cell's documented facts and figures. The same numbers are constants of `cell_data.py`; the arm's first-order
accuracy model is `arm_model.py`, whose code states it exactly. Names in `code` font are constants and functions of
those two files. The defects a unit may have beyond the first-order model are in `defects.md`; the wrist camera is
in `camera.md`. The gauge plate that stands in every cell is in section 10, the two jobs that put `process_c`'s
pin into its bores, one on a cold arm and one on a warm arm, in section 11; the service that every graded unit undergoes
after its measuring session, its re-qualification session and the serviced job in section 12.

## 1. Frames, units, numbering

- Lengths in metres, angles in radians, time in seconds, masses in kilograms, torques in N m, temperatures in
  degrees Celsius. Degrees of angle appear only where this sheet says so.
- **Joints** are numbered 0 to 5 from the base to the flange; j is the index of every per-joint array in
  `arm_model.py`, `cell_data.py`, the lab's replies and the conditions files (`q[j]`, `Q_MIN[j]`, `xyz[j]`, `b[j]`,
  `temp[j]`, ...). Joint 0 turns the arm about the vertical axis, joints 1 and 2 are the shoulder and the elbow,
  joint 3 turns the forearm about its own axis, joint 4 bends the wrist and joint 5 turns the flange.
- **Datum frame**: the cell's frame, z up; the table top is the plane z = 0. The datum spheres' survey, the tool
  tip, the camera pose and the acceptance tests are in this frame. On the drawing the arm's base frame is the datum
  frame; on a unit it is off by the base tolerances of section 3.
- **Joint frames**: frame j is frame j - 1 (the datum frame for j = 0) followed by a translation `xyz[j]`, three
  fixed rotations Rx(`rpy[j][0]`) Ry(`rpy[j][1]`) Rz(`rpy[j][2]`) about the moving axes, and the joint's rotation
  Rz(theta_j). Joint j turns about the z axis of frame j.
- **Flange frame**: frame 5 translated by `FLANGE_Z` = 0.08 m along its own z axis, not rotated. The tools'
  certificates and the camera's mounting are given in this frame.
- **Camera frame**: `camera.md` (z along the optical axis, x towards increasing image column, y towards increasing
  image row).

## 2. The arm

### Nominal chain (`NOMINAL_XYZ`, `NOMINAL_RPY`) and joint limits (`Q_MIN`, `Q_MAX`)

| Joint j | xyz_j (m) | rpy_j (degrees) | q_j range (degrees) |
|---|---|---|---|
| 0 | (0.00, 0, 0.00) | (0, 0, 0) | -170 to 170 |
| 1 | (0.05, 0, 0.38) | (-90, 0, 0) | -190 to 10 |
| 2 | (0.40, 0, 0.00) | (0, 0, 0) | -80 to 150 |
| 3 | (0.08, 0, 0.00) | (0, 90, 0) | -180 to 180 |
| 4 | (0.00, 0, 0.30) | (0, -90, 0) | -115 to 115 |
| 5 | (0.00, 0, 0.00) | (0, 90, 0) | -180 to 180 |

The home pose `Q_HOME` is (0, -90, 90, 0, 45, 0) degrees. On the drawing (nominal values, 20 °C) the stylus centre
of `probe_a` is then at (0.5926, 0, 0.6174) m in the datum frame.

### Links (`LINK_MASS`, `LINK_COM`)

Mass and centre of gravity of the link that turns with joint j, given in frame j:

| Joint j | mass (kg) | centre of gravity in frame j (m) |
|---|---|---|
| 0 | 8.0 | (0.02, 0, 0.25) |
| 1 | 6.0 | (0.20, 0, 0.00) |
| 2 | 2.5 | (0.04, 0, 0.00) |
| 3 | 3.0 | (0.00, 0, 0.15) |
| 4 | 1.0 | (0.00, 0, 0.00) |
| 5 | 0.5 | (0.00, 0, 0.04) |

### The first-order accuracy model (`arm_model.py`)

The manufacturer's model of a unit has 84 parameters (`N_PARAM`, in the order of `SLICES`): the chain's `xyz` and
`rpy`, and per joint a backlash `b`, a cyclic transmission error `h` (a first and a second harmonic of the encoder
reading), a compliance `c` and a thermal growth coefficient `kappa`, followed by the camera's mounting (`camera.md`).
In words: the angle theta_j that enters the chain is the encoder reading q_j, plus half the backlash in the
direction s_j the joint last moved in (section 5), plus the two harmonics of q_j, plus the compliance times the
gravity torque about the joint; the gravity torque (`gravity_torques`) is that of the links' weights and of the
mounted tool's (its certificate's mass at its centre of gravity, `G` = 9.80665 m/s²), computed on the nominal chain
at the encoder readings. The translation `xyz[j]` is scaled by (1 + kappa_j (temp_j - `T0`)), `T0` = 20 °C, temp_j
being link temperature j (section 6). The tool tip (`tcp`) is the certificate's tip point in the flange frame;
`camera_pose` gives the camera frame. The drawing (`nominal()`) has b = h = c = kappa = 0.

A real unit follows this model to first order and in addition has some of the defects of `defects.md`.

## 3. How a unit's first-order parameters differ from the drawing

Each unit has its own values of the 84 parameters, each uniform inside its range below, independently of the
others (`cell_data.py`, "tolerances of a unit"):

| Parameters | Range on a unit |
|---|---|
| xyz_0 | nominal ± 5.0 mm on each axis (`TOL_BASE_XYZ`) |
| rpy_0 | nominal ± 0.5 degrees (8.727e-3 rad) on each of the three angles (`TOL_BASE_RPY`) |
| xyz_j, j = 1..5 | nominal ± 1.0 mm on each axis (`TOL_XYZ`) |
| rpy_j, j = 1..5 | nominal ± 0.1, ± 0.1, ± 0.3 degrees (1.745e-3, 1.745e-3, 5.236e-3 rad) (`TOL_RPY`) |
| b_j | 0.2 to 1.0 times `BACKLASH_MAX`: 0.06 degrees (1.047e-3 rad) for joints 0 to 2, 0.10 degrees (1.745e-3 rad) for joints 3 to 5 |
| h_j | per harmonic an amplitude of 0.3 to 1.0 times `CYCLIC_MAX` (0.015 degrees, 2.618e-4 rad, for joints 0 to 2; 0.025 degrees, 4.363e-4 rad, for joints 3 to 5) and a phase anywhere in the full turn |
| c_j | 0.3 to 1.0 times `COMPLIANCE_MAX`: 1e-5 rad / (N m) for joints 0 to 2, 4e-5 rad / (N m) for joints 3 to 5 |
| kappa_j | 0.5e-5 to 5.0e-5 per K (`KAPPA_RANGE`: the effective growth of a link assembly, not of one material) |
| camera xyz | nominal ± 3.0 mm on each axis (`TOL_CAMERA_XYZ`) |
| camera rpy | nominal ± 1.0 degree (1.745e-2 rad) on each angle (`TOL_CAMERA_RPY`) |

The first harmonic of h_j is h_j[0] sin q_j + h_j[1] cos q_j, with amplitude sqrt(h_j[0]² + h_j[1]²); the second is
h_j[2] sin 2q_j + h_j[3] cos 2q_j. The third rpy angle of a joint acts about the same axis as Rz(theta_j): it holds
the joint's zero offset.

With the drawing's values, the largest tool-tip error on a unit's acceptance conditions (section 9) is between 11
and 22 mm for nine units in ten, and the largest camera-test error between 19 and 34 mm.

## 4. Datum spheres, posts, table

Twelve white datum spheres of radius `R_SPHERE` = 19.0 mm (certified), each on a post. The survey
(`surveyed_spheres()`): sphere k (k = 0..11) at azimuth -125 + k · 250/11 degrees about the datum z axis, measured
from +x; horizontal distance 0.50 m from the z axis for even k and 0.68 m for odd k; centre height 0.10, 0.40,
0.70 m for k mod 3 = 0, 1, 2.

| k | azimuth (degrees) | distance from z axis (m) | x (m) | y (m) | z (m) |
|---|---|---|---|---|---|
| 0 | -125.000 | 0.50 | -0.286788 | -0.409576 | 0.10 |
| 1 | -102.273 | 0.68 | -0.144544 | -0.664460 | 0.40 |
| 2 | -79.545 | 0.50 | 0.090728 | -0.491700 | 0.70 |
| 3 | -56.818 | 0.68 | 0.372162 | -0.569118 | 0.10 |
| 4 | -34.091 | 0.50 | 0.414075 | -0.280254 | 0.40 |
| 5 | -11.364 | 0.68 | 0.666670 | -0.133984 | 0.70 |
| 6 | 11.364 | 0.50 | 0.490198 | 0.098518 | 0.10 |
| 7 | 34.091 | 0.68 | 0.563142 | 0.381145 | 0.40 |
| 8 | 56.818 | 0.50 | 0.273649 | 0.418469 | 0.70 |
| 9 | 79.545 | 0.68 | 0.123390 | 0.668711 | 0.10 |
| 10 | 102.273 | 0.50 | -0.106283 | 0.488573 | 0.40 |
| 11 | 125.000 | 0.68 | -0.390032 | 0.557023 | 0.70 |

(x and y rounded to the micrometre here; `surveyed_spheres()` gives them in full.)

- **Knocked spheres** (`KNOCKED_SPHERES`, `KNOCK_MIN`, `KNOCK_MAX`): since the survey, 0 to 4 of the twelve spheres
  of a unit's cell have been knocked; each knocked sphere stands 0.5 to 3.0 mm from its surveyed centre. How many,
  which ones and in which direction is not told. The other spheres are at their surveyed centres.
- **Posts**: under each sphere a vertical cylinder of radius `R_POST` = 6 mm from the table (z = 0) up to the
  sphere's actual centre (knocked or not).
- **Table**: for contact (section 5), the plane z = 0 wherever the stylus is. What the camera sees of it:
  `camera.md`.

## 5. The probes, the tools, contact

Tool certificates (`TOOLS`), in the flange frame:

| Tool | mass (kg) | centre of gravity (m) | tip (m) | mountable |
|---|---|---|---|---|
| `probe_a` | 1.0 | (0.00, 0.00, 0.06) | (0.00, 0.00, 0.15) | yes |
| `probe_b` | 5.0 | (0.03, 0.00, 0.08) | (0.12, 0.00, 0.20) | yes |
| `process_c` | 3.0 | (0.01, -0.02, 0.10) | (0.05, -0.08, 0.28) | no: the acceptance and job tool, never mounted in the cell |

`process_c` carries a cylindrical pin of diameter 5.30 mm (`PIN_RADIUS` = 2.65 mm, certified): its axis runs through
the tip along the flange's +z axis; its end is a hemisphere whose apex is the tip; it reaches `PIN_LENGTH` = 30 mm
from the apex back towards the flange. In the jobs (section 11) only the pin can touch anything.

- Both probes end in a stylus ball of radius `R_STYLUS` = 2.0 mm (certified); a probe's tip is the ball's centre.
  Tools are changed only at the home pose (every joint within 1e-6 rad of `Q_HOME`). A session starts with
  `probe_a` mounted.
- **What can touch**: only the stylus ball; the arm, the tool's body and the camera collide with nothing. The ball
  touches a sphere when its centre is within `R_SPHERE` + `R_STYLUS` of the sphere's actual centre, a post when its
  centre is within `R_POST` + `R_STYLUS` of the segment from the post's foot (the actual centre's x and y, z = 0) to
  the sphere's actual centre, the table when its centre is at z ≤ `R_STYLUS`, and the gauge plate or one of its
  shims (section 10) when its centre is within `R_STYLUS` of their surfaces.
- **The gauge plate is never touched**: a move or a probe (also a guarded one, at any speed) whose stylus reaches
  the plate or a shim collides there, as an unguarded move does: the arm stops at the first sample of the path
  (below) that touches it, the reply says `collided = true`, `damaged = true` (a probe's reply also
  `triggered = false`), and the session can measure no more. A probe that stops earlier (on a sphere, a post, the
  table or a false trigger) has not reached the plate.
- **Where the ball is** during a `move` or a `probe` (for this check): the true position of the unit's stylus at the
  path's joint positions, with the command's new direction signs (section 5, below), the new repeatability error
  of this arrival (section 6) and the link temperatures at the command's start.
- **Probe** (guarded): the path is checked at evenly spaced samples, at most 0.0005 rad apart for the joint that
  moves farthest, from the start to the target. If no sample touches, the probe ends at its target with
  `triggered = false`. (The plate: above.) If the first sample (the start) touches, the probe stops at once with `triggered = true`.
  Otherwise the contact is located by 40 halvings of the interval between the last sample without contact and the
  first touching one; the arm stops at the touching end of the final interval, just in contact, with
  `triggered = true`. Probes never damage the stylus, except on the gauge plate (above).
- **False triggers**: with the unit's false-trigger probability (section 7), independently for every probe, a probe
  stops at a point of its path short of the contact it would have found (short of its target if it would have
  found none) and replies `triggered = true`, exactly as a real touch.
- **Move** (unguarded, at every speed): the path is checked at evenly spaced samples, at most 0.002 rad apart for
  the joint that moves farthest, the start and the target included. A move whose start does not touch collides at
  its first touching sample. A move may start in contact (after a probe): then the samples up to the one where the
  path first leaves contact do not count, and any touching sample after that is a collision; a move that never
  leaves contact collides at its second sample. At a collision the arm stops at that sample, the reply says
  `collided = true` and `damaged = true`, the probe is broken and the session can measure no more (section 8).

### Motion and time

| Figure | Value |
|---|---|
| `move` speed (`MOVE_SPEED_MAX`) | as commanded, 0.001 to 2.0 rad/s for the joint that moves farthest |
| `probe` speed (`PROBE_SPEED`) | 0.01 rad/s for the joint that moves farthest |
| settle after every move and probe (`SETTLE`) | 0.3 s |
| tool change (`TOOL_CHANGE`) | 30 s |
| image (`IMAGE_TIME`) | 2.0 s, the arm at rest |
| `wait` | the seconds asked, 0.001 to 600 |

- **Paths**: `move` and `probe` go along the straight line in joint space from the arm's present joint positions to
  the target; all joints start and stop together. A `move` ends exactly at its target, a `probe` where it stops.
- **Time charged** (virtual), with Δq = target - present joint positions: `move`: max_j |Δq_j| / speed + 0.3 s;
  `probe`: f · max_j |Δq_j| / 0.01 + 0.3 s, f the fraction of the path travelled; a move that collides at the
  fraction f of its path: f · (max_j |Δq_j| / speed + 0.3 s) + 1e-6 s; tool change 30 s; image 2 s; `wait` the
  seconds asked. A refused command takes no time.
- **Direction signs** s_j (+1 or -1, the backlash direction of the model): at the start of a session s_j = +1 for
  all six joints. Every `move` and every `probe`, also one that stops early or collides, sets s_j to the sign of
  Δq_j for each joint with |Δq_j| > `S_THRESHOLD` = 1e-4 rad and leaves s_j unchanged for the other joints. Tool
  changes, images and waits leave s unchanged. The lab does not report s.

## 6. Readings

| Figure | Value |
|---|---|
| encoder step (`ENCODER_STEP`) | 2π / 2^22 rad = 1.498e-6 rad |
| temperature reading noise (`SIGMA_TEMP`) | 0.05 K (standard deviation) |

- **Joint positions**: a reply's `q` is the arm's joint positions rounded to the nearest multiple of `ENCODER_STEP`.
  These encoder readings are the q of the model.
- **Link temperatures**: six, `temp[0]` to `temp[5]`; link temperature j is the one that scales xyz_j in the model
  and that rises as joint j turns (section 7). A reply's `temp` is the six link temperatures, each plus reading
  noise of standard deviation 0.05 K drawn anew for every reply. Every session starts with every link at 20 °C
  (`T_AMBIENT`).
- **Time**: a reply's `time_s` and `time_left_s` are the session's virtual time used and left, rounded to 1 ms; the
  cell keeps the time unrounded, also for the 1000 s limit.
- **Repeatability**: at every arrival (every `move`, every `probe`, every tool change) the tool tip comes to rest off
  the position the unit's kinematics give by a fresh random translation, with the unit's repeatability (section 7)
  as its standard deviation on each axis, independent between axes and arrivals. It holds until the next arrival,
  along a probe's whole path and for the images taken before the next arrival, whose camera centre it shifts by the
  same translation. Before a session's first arrival it is zero. It is not part of the model and does not enter
  the acceptance tests.
- Within a command, the contact check (section 5) and an image use the link temperatures at the command's start;
  the reply reports them at its end.

## 7. Figures stated only as ranges

Each of these differs from cell to cell, is fixed for a unit's cell (the same in every session on that unit) and is
not told for any cell; how it is distributed inside its range is not stated.

| Figure | Range | `cell_data.py` |
|---|---|---|
| heating: how much a link warms per radian its own joint turns (it also warms a little as its neighbouring joints turn; how much is not documented) | 0.06 to 0.16 K per radian (0.6 to 1.6 times 0.10), per joint | `HEAT_NOMINAL`, `HEAT_FACTOR_RANGE` |
| cooling: time constant with which the links cool towards 20 °C | 400 to 900 s, one per unit | `TAU_COOL_RANGE` |
| repeatability (standard deviation per axis, section 6) | 1.5e-5 to 3.0e-5 m | `SIGMA_REPEAT_RANGE` |
| false-trigger probability per probe (section 5) | 0.01 to 0.10 | `FALSE_TRIGGER_RANGE` |
| knocked datum spheres | 0 to 4 of the twelve | `KNOCKED_SPHERES` |
| distance of a knocked sphere from its survey | 0.5 to 3.0 mm | `KNOCK_MIN`, `KNOCK_MAX` |

**How the link temperatures change** (in a session, and in the jobs of section 11): each link temperature relaxes
towards 20 °C (`T_AMBIENT`) with the unit's cooling time constant τ and is heated by the joints' travel: during a
command (or a job's row) of duration d in which joint j travels |Δq_j|, link i receives Σ_j H_ij |Δq_j| kelvin of
heating spread evenly over d, so that dT_i/dt = Σ_j H_ij |Δq_j| / d - (T_i - 20 °C) / τ. H_ii is the heating figure
of the table above (link i by its own joint i); H_ij for a neighbouring joint (j = i - 1 or i + 1) is the small
warming by that joint, whose size is not documented; H_ij = 0 for every other joint. Tool changes, images and
waits travel nothing: the links only cool. Neither τ nor any H_ij of a unit is told; the replies' temperature
readings are the only evidence of them.

The camera's per-unit figures (focal length, principal point, distortion, focus, lighting, materials, exposure) are
in `camera.md`.

## 8. A session

| Item | Per measuring session (`BUDGET`) | Per re-qualification session (`BUDGET_SERVICE`, section 12) |
|---|---|---|
| virtual time | 1000 s | 150 s |
| probes | 120 | 12 |
| images | 51 | 0 (the camera is off) |

A session starts at the home pose, every link at 20 °C, `probe_a` mounted, s = +1, nothing used. Refusals (the
first that applies):

- after a collision (`damaged = true`): `move`, `probe`, `tool` and `image` are refused (`damaged`) for the rest of
  the session; `wait` is still carried out;
- once the session's time has reached its budget (1000 s; 150 s in a re-qualification session): `move`, `probe`
  and `image` are refused (`time_up`); a command that starts before that runs to its end, even past it;
- `probe` once the session's probes are used (120; 12), `no_probes`; `image` once its images are used (51; 0: in
  a re-qualification session from its start), `no_images`;
- `tool` away from the home pose (`not_home`).

A refused command changes nothing and charges nothing. A measuring session of a unit that has been serviced
(section 12) refuses every command (`serviced`).

## 9. Acceptance tests

Every unit (graded or practice) has its own `N_ACCEPT` = 400 acceptance conditions, published in the unit's
conditions file. They were drawn as follows: joint positions inside the joint limits for which the drawing (s = 0,
every link at 20 °C) places the tip of `process_c` strictly between 0.35 and 0.85 m from the datum z axis, strictly
between 0.05 and 0.90 m above the table and at less than 130 degrees of azimuth from +x (`ACCEPT_REGION`); each
s_j = +1 or -1; conditions 0, 2, 4, ... cold, every link between 20 and 22 °C, and conditions 1, 3, 5, ... warm,
every link between 35 and 55 °C (`ACCEPT_TEMP`). The tool is `process_c` (`ACCEPT_TOOL`). Nothing else
(reachability, the spheres) was checked.

**Tool-tip test**: per condition, the distance between the predicted tip of `process_c` and the true unit's tip at
that condition (q, s, temperatures, `process_c` mounted). Limits: 95th percentile of the 400 errors ≤ 0.20 mm
(`ACCEPT_P95`), largest ≤ 0.40 mm (`ACCEPT_MAX`).

**Camera test**: per condition, the five points `ACCEPT_CAM_POINTS`, given in the camera frame (m),

| x | y | z |
|---|---|---|
| 0.00 | 0.00 | 0.35 |
| 0.12 | 0.09 | 0.30 |
| -0.12 | 0.09 | 0.40 |
| 0.15 | -0.10 | 0.45 |
| -0.10 | -0.08 | 0.25 |

are placed in the datum frame with the true unit's camera pose at that condition (`process_c` mounted); the error
is the distance between the predicted and the true placement. Limits over all 400 × 5 errors together: 95th
percentile ≤ 0.40 mm (`ACCEPT_CAM_P95`), largest ≤ 0.60 mm (`ACCEPT_CAM_MAX`).

Percentiles are NumPy's default (`numpy.percentile`, linear interpolation). The repeatability error of section 6
does not enter these two tests (it does enter the jobs, section 11).

A unit is accepted when all four limits hold **and** its three job programs, the cold one and the warm one
(section 11) and the serviced one (section 12), have each done every bore. The predictions and the cold and the warm
job are graded on the unit as it was measured, before its service; the serviced job on the unit after its service.

## 10. The gauge plate

Every cell (the practice unit's included) holds a finished part, a gauge plate, at a pose of its own. It may never
be touched in a session (section 5); the wrist camera sees it (`camera.md`).

- **Block**: `PLATE_SIZE` = 160 x 100 x 20 mm. **Plate frame**: origin at the centre of the top face, x along the
  length, y along the width, z out of the top face; the block occupies |x| ≤ 80 mm, |y| ≤ 50 mm, -20 ≤ z ≤ 0 mm.
- **Bores** (certificate, exact): `N_BORES` = 8 blind bores of diameter 5.80 mm (`BORE_RADIUS` = 2.90 mm) and depth
  12 mm (`BORE_DEPTH`, flat bottom), their axes along the plate's z axis through these points of the top face
  (`BORES`, mm):

| bore | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| x | -65 | -62 | -35 | -10 | 0 | 28 | 52 | 64 |
| y | -30 | 28 | -2 | 32 | -28 | 10 | -30 | 22 |

- **Shims**: three vertical cylinders of radius 8 mm (`SHIM_RADIUS`) under the plate, their axes through the
  plate-frame points (-60, -35), (-60, 35), (60, 0) mm of the bottom face (`SHIMS`), standing on the table and
  reaching up to the plate's bottom face. Their different heights tilt the plate.
- **Pose** (per unit, not told; each figure uniform in its range, independently): the top face's centre at
  `PLATE_XY_NOMINAL` = (0.230, -0.352) m ± `PLATE_XY_TOL` = 30 mm on each axis; the plate frame's rotation
  Rx(tilt_x) Ry(tilt_y) Rz(yaw) (`plate_rotation`: the yaw about the vertical first, then the tilts about the datum
  y and x axes), yaw within `PLATE_YAW_TOL` = 15 degrees of `PLATE_YAW_NOMINAL` = 33 degrees, each tilt within
  `PLATE_TILT_MAX` = 2 degrees; the bottom face's centre 6 to 10 mm above the table (`PLATE_LIFT`; its corners,
  tilted, between 1.6 and 14.4 mm). `plate_frame` turns these six figures into the plate frame's rotation and
  origin.
- **Where it can be**: at every pose of these ranges the plate and its shims lie inside the vertical cylinder of
  radius 0.138 m about (0.230, -0.352) m, from the table up to 0.035 m (`plate_envelope()`).

## 11. The jobs

Each graded unit's submission carries three job programs. This section is about two of them, the **cold job** (`job`) and the **warm job** (`job_warm`); the third, the serviced job, is in section 12:
each a list of 1 to `JOB_MAX_ROWS` = 200 joint targets (rows of six numbers, rad, each inside its joint's limits,
the limits included). The grader runs each **once**, on the true unit, in its own copy of the cell; nothing about
any unit's jobs is reported before grading. The two runs are independent of each other: each starts afresh as
below, with its own repeatability draws. Everything in this section holds for both jobs; they differ only in the
link temperatures at the start.

- **Start**: `process_c` mounted (its weight bends the arm, as in the model), the arm at `Q_HOME`, s = +1 for every
  joint, no repeatability error yet. The cold job: every link at 20 °C (`T_AMBIENT`). The warm job: link j at the
  unit's warm start temperature `job_warm_temp[j]`, the six numbers of the unit's conditions file (°C, exact, not
  read with noise; per unit, each drawn uniform in `JOB_WARM_TEMP` = 45 to 55 °C, independently). The rows are run
  in order.
- **A row** is a joint move to its target at `JOB_SPEED` = 0.25 rad/s for the joint that moves farthest, followed by
  `SETTLE` = 0.3 s: it takes max_j |Δq_j| / 0.25 + 0.3 s. The links warm as their joints turn and cool towards 20 °C
  exactly as in a session (section 7: the cell's own figures and law, with d the row's duration): along the warm job
  they cool from their start temperatures while the rows warm them a little; the link temperatures at a row's
  start are those the law gives from the job's start through the rows before it.
- **Direction signs**: a row sets s_j as a session's move does (section 5): the sign of Δq_j for each joint with
  |Δq_j| > `S_THRESHOLD` = 1e-4 rad, unchanged for the others.
- **Repeatability**: every row's arrival draws a fresh random translation of the tip (section 6: the unit's
  repeatability, per axis); it holds until the next row.
- **The path of a row**: the joints go along the straight line in joint space from where they are to the target.
  Along it, each direction sign the row changes goes linearly from its old value (+1 or -1) to its new one (the
  backlash is taken up over the row), and the repeatability translation goes linearly from the previous arrival's
  to this arrival's; the link temperatures are those at the row's start. Where the pin is along the path is what the
  unit's kinematics give at those joint positions, signs and temperatures, plus that translation. At the row's end
  the pin is where the unit's kinematics put it at the target with the new signs, plus the new translation.
- **Done**: a bore is done when, at the end of a row, the pin is in it: its apex inside the bore's hole, between
  `JOB_DEPTH` = 8.0 mm below the mouth (the plane of the top face) and the bore's bottom (12 mm below it), less than
  2.90 mm (`BORE_RADIUS`) from the bore's axis, and the pin's axis leaving the hole through the mouth (it crosses
  the top face's plane less than 2.90 mm from the bore's axis); nothing must have been touched so far. (A pin that
  touches nothing can be there only by having come in through the mouth.) The pose at a row's end is the one
  described above: the target, the new signs, the new repeatability translation and the link temperatures at the
  row's start.
- **Jam**: the first contact of the pin (its cylinder or its hemispherical end) with anything (the plate: its top
  face, its sides, a bore's wall or bottom; a shim; the table; a datum sphere; a post) anywhere along any row stops
  that job there. Bores done before the jam count as done. The contact check is continuous along every row: a
  contact anywhere is found; a pin that comes within `JOB_RESOLUTION` = 0.01 mm of a surface may count as touching it.
- **Passed** when all 8 bores are done; a unit's acceptance needs every one of its jobs passed (these two and the serviced job of section 12).
- **The check's work**, per job: the grader examines poses along every row: first one every 0.002 rad of the joint
  that moves farthest (a row costs at least max_j |Δq_j| / 0.002 + 1 poses, start and target included), then more
  where the pin passes close to a surface, the more the closer and the longer it stays close. Every pose counts;
  a program whose check would need more than `JOB_MAX_POSES` = 15000 poses fails (the check stops there). A
  program that goes into each bore along its axis and out again with the pin clear of the walls by a few
  hundredths of a millimetre needs a few thousand; reaching the limit takes about 30 rad of joint travel (the
  farthest-moving joint's, summed over the rows) or keeping the pin within a few micrometres of a surface over
  long paths.

The radial clearance between the pin and a bore is 0.25 mm. A warm arm is not a cold one: its links have grown and,
on some units, joint zeros have moved with temperature (`defects.md`), so that for the same joint targets the pin
of a unit at its warm start stands typically 0.1 to 1.1 mm (median about 0.50 mm) from where it stands on the cold
unit, more than the clearance on most units: one program does not usually serve both jobs. No job program is run before grading, on any
unit (the practice unit included): acceptance is the customer's inspection after delivery, and the lab has no
acceptance rig (instruction.md).

## 12. The service, the re-qualification session and the serviced job

After its measuring session every graded unit is **serviced**: the drive of one of its joints, the motor with its
encoder, is replaced and the joint's gear mesh is set anew. The gear train, the links, the camera and the cell (the
spheres as they stand, knocked or not; the posts, the table, the gauge plate; the cell's figures of section 7) are
untouched. The practice unit `p1` is not serviced: it is the shop's demonstration unit and is not due for service,
so it has no re-qualification session; the service and the re-qualification session are met for the first time on
a graded unit.

- **The serviced joint**: one of joints 0 to 4 (`SERVICE_JOINTS`; the flange's drive, joint 5, is not serviced in
  the cell), drawn per unit. Which one is not told.
- **What changes**, in the terms of the first-order model (section 2), and nothing else:
  - the joint's zero offset, its third rpy angle (`rpy[j][2]`, which acts about the joint's axis, section 3), moves
    by an amount between 0.10 and 0.25 degrees (1.745e-3 to 4.363e-3 rad, `SERVICE_ZERO`) in magnitude, either
    sign: the new encoder is mounted at an angle of its own;
  - the joint's backlash b_j takes a new value anywhere in the range of a new unit's (section 3: 0.2 to 1.0 times
    `BACKLASH_MAX`), independently of the old: the mesh is set anew.

  The amount, its sign and the new backlash are not told. Every other figure of the unit stays as it was: the other
  first-order parameters, the defects of `defects.md` (properties of the links, the wrist's transmissions and the
  gear trains), the camera's. The encoder readings are the new encoder's: the model's q, as before.
- **What it does to the tip**: at given encoder readings and direction signs, the serviced unit's joint angle theta_j
  differs from the unit's as measured by the change of the zero offset plus s_j times half the change of the
  backlash: in all at most 0.25 degrees plus 0.4 times `BACKLASH_MAX[j]` (0.274 degrees for joints 0 to 2, 0.29
  degrees for joints 3 and 4). A turn by an angle a about joint j's axis moves a point at distance rho from the axis
  by at most a times rho; the links beyond the joint turn with it. On the drawing's geometry this moves the stylus
  centre by up to about 4.5 mm at the acceptance conditions' poses and by up to about 5.1 mm anywhere inside the joint limits (joints 0 and 1, the arm reaching far), and the pin of `process_c` at the plate's
  bores typically by 0.6 to 2.6 mm (median about 1.2 mm), more than the clearance on every unit.
- **The re-qualification session**: one per graded unit (the instance `<unit>-serviced`, instruction.md), on the serviced
  unit in the same cell. It starts as a measuring session does (section 8): at the home pose, every link at 20 °C,
  `probe_a` mounted (a new probe: also after a measuring session that ended in a collision), s = +1, nothing used,
  with the budget `BUDGET_SERVICE`: 150 s of virtual time, 12 probes, 0 images. Every command, rule and refusal of
  sections 5 to 8 holds as in a measuring session, with that budget. The cell's figures (false triggers,
  repeatability, heating and cooling) are the unit's own, as in its measuring session.
- **No images in the re-qualification session**: the wrist camera's cable runs along the arm and was unplugged for
  the drive's replacement; the camera is reconnected and re-verified only when the unit is released, after this
  session, and stays off until then. The camera itself (its mounting and its optics) is unchanged by the service.
  An `image` command in this session is refused as `no_images` (as `damaged` or `time_up` when one of those applies
  first, section 8): it takes no time and charges nothing. It is still a valid command: as the session's first
  command it charges the session, and the unit is serviced (below).
- **Order**: the service comes between the two kinds of session. When the first valid command of a unit's
  re-qualification session arrives, the unit is serviced for good: its measuring session is over. A measuring
  session still open on another connection answers every later command with `refused = serviced`, and so does a
  measuring session opened later, also one the unit never used; nothing is charged for them.
- **The serviced job** (the submission's `job_serviced`): a program by the rules of section 11's cold job, run once
  by the grader on the serviced unit, cold: `process_c` mounted, the arm at `Q_HOME`, every link at 20 °C, s = +1,
  no repeatability error yet; its own repeatability draws. Passed when all 8 bores are done.
- A program planned for the unit as measured, run on the serviced unit, misses: one program does not serve both
  states. No serviced job is run before grading, and `p1` is never serviced (instruction.md).
