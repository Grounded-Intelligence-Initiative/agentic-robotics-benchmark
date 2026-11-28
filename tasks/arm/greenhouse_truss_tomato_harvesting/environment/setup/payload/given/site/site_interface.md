# Harvesting cart: interface document

The site: a rail cart that drives along one plant row of a high-wire tomato greenhouse and carries a Universal Robots
UR20 with a cutter-gripper, two harvest crates, a colour camera on a mast and an RGB-D camera on the robot's wrist.
One job is one row section: 4 plants with 3 trusses each (`../crop/growing_protocol.md`). Your program drives the
cart, the arm and the tool through the wire of section 2. All numbers of this document are also in `site.json` beside
it.

## 1. Frames and places

**The cart frame** is the robot's base frame: origin on the robot's mounting face, x from the aisle toward the plant
row, y along the rail, z up. It rides with the cart. Every position and pose on the wire and in this document is in
this frame, in metres, unless it says otherwise.

**The row frame** is fixed to the greenhouse and parallel to the cart frame. The cart's base stands at (0, c, 0) in
the row frame, where c is the cart's place along the rail, and c = 0 where a row section starts. A point at (x, y, z)
in the row frame is at (x, y - c, z) in the cart frame. Nothing on the wire reports c (section 5).

- **Heights.** The greenhouse floor is at z = -0.62, the top of the cart's deck at z = -0.32. The deck reaches from x
  = -0.45 to 0.5 and from y = -1.1 to 1.1; the robot stands on a pedestal, a column of 0.16 m radius under its base.
- **Crates.** Two open crates stand on the deck and ride with the cart, one behind the robot along the rail and one
  ahead of it. Each is parted into three bays side by side along x, a bay for a truss, by dividers that run the
  crate's length along y, 0.08 m high above the floor and 0.006 m thick, midway between the bays' centre lines.

| crate | centre x, y (m) | inner size x by y (m) | top of the floor, z (m) | walls: height, thickness (m) | bays' centre lines, x (m) |
|---|---|---|---|---|---|
| rear | 0.06, -0.8 | 0.6 x 0.4 | -0.308 | 0.14, 0.008 | -0.14, 0.06, 0.26 |
| front | 0.06, 0.8 | 0.6 x 0.4 | -0.308 | 0.14, 0.008 | -0.14, 0.06, 0.26 |

- **Boards.** A calibration plate and a colour reference card lie flat on risers at the deck's row-side edge, their
  faces at z = -0.2, and ride with the cart (section 6).
- **Mast.** The overview camera sits on a mast at the deck's rear corner on the row side (section 6).
- **The row.** The plants' stems stand in a substrate slab on a gutter along the row: nominally at x = 1.2, the first
  at y = 0.3 and one every 0.5 m (row frame), the slab's top at z = -0.48 (the growing protocol has the tolerances).
  Row markers are fixed along the gutter's aisle side (section 5). A plain screen hangs behind the row.
- **Robot.** Joint limits (degrees, either way): [360, 360, 180, 360, 360, 360]. A row section starts with the cart at
  c = 0, the jaws open and the robot at its home joint angles (rad) [-0.7804, -2.1304, 2.5661, -0.4356, 0.7904,
  1.5708]: the tool centre point at [0.8, 0, 0.5], the tool axis along [1, 0, 0] (level, pointing at the row) and the
  tool frame's x axis along [0, 0, 1]. Kinematic model: `../robot/ur20.json`. The flange frame is the robot's
  `wrist_3_link` frame: origin on the flange face, z out of the flange. The tool frame has the flange frame's axes and
  its origin at the tool centre point, 0.36 m out along z (`../tool/cutter_gripper.md`).

## 2. The wire

The grader starts your program with `bash run.sh` in the submission folder and talks to it over its standard input and
output, one JSON object per line. Your program first prints `READY` (within 120 s), then answers every request (each
within 30 s):

```
{"type": "reset"}                        -> {"ok": true}            the robot stands at a new row section
{"type": "act", "t": 0, "obs": {...}}    -> {"action": {...}}       one action for step t of this row section
{"type": "close"}                        -> {"ok": true}, then exit
```

Standard output is the wire: write logs to standard error. `t` is the step index of the row section, owned by the
grader, starting at 0 after each `reset`.

**A step may be asked more than once with the same `t`.** A repeated request carries the observation the cell would
give with the robot's joints shifted by a small hidden amount (up to 0.01 rad a joint, the same shift for the measured
and the commanded angles): `q` and `q_target` are shifted, and `tcp`, `tcp_target` and a wrist frame's `pose` are
those of the shifted joints, so the readings of one observation agree with each other as they always do. Everything
else in it is unchanged. The other request carries the true observation, and nothing on the wire says which is which
or which comes first. Only the answer to the true observation is executed. Answer every request from the observation
it carries, and advance your own state once per `t`, not once per request. A program whose motions do not depend on
the robot's readings it is sent (a replayed list of actions, absolute joint targets worked out in advance) is
recognised by this and scores nothing: compute each displacement from the `tcp_target` or `q_target` of the
observation you answer. Requests that command no motion of the arm or the cart are not compared. Two replies differ
when one of the 14 numbers the grader reads from an action differs by more than 0.0001
(`rowcell_client.replies_differ` is the grader's rule); if fewer than 0.1 of the compared pairs, over all the row
sections, differ, the reward is 0. `~/commission/run_session.py` prints the count of a session.

A row section ends early on your program's side in two ways; in both, what has been delivered so far still counts:

- **Your program is given up on and started again:** a missed deadline (`READY` within 120 s of a start; each request
  read and answered within 30 s), a closed output, a line that is not a JSON object, an answer without `action`, or
  more than 64 processes of your account (counted before every request; threads are not counted). The row section ends
  where it is, your program is stopped with everything it started, and `run.sh` is started again for the next row
  section: at most three such restarts, after which the remaining row sections are not worked on.
- **The row section ends and your program goes on:** an `action` that is not valid (a malformed field or a value out
  of range) or that the cell rejects, the row section's budget of your program's own time used up, or its step limit
  reached (section 7). The same program, still running, gets the next `reset`.

**When the cell itself fails.** Your program shares the GPU and the sandbox's memory with the cell. If the cell fails
while a row section runs, your program is stopped with everything it started, every file of your account outside the
submission folder and every SysV IPC object of your account are removed, the submission folder is put back as it was
handed over, and the row section is run again from the start in a fresh cell, with `run.sh` started again (that
restart is not one of the three). If the cell fails a second time on that row section after your program was started
again, or your program could not be stopped completely, the row section counts 0 and grading goes on with the next
one.

**The grader's own deadline.** The grader stops 13200 s after it starts, whatever is running then: the row sections
graded by then count and the others count 0.

## 3. Actions

An action is a JSON object; every key is optional:

```
{"move": {"type": "tcp" | "joint" | "hold", "delta": [6 numbers], "ticks": 1..40, "guard_force": N},
 "cart": metres along the rail,
 "gripper": {"opening": metres, "force": N},
 "cmd": "done",
 "cams": ["overview", "wrist"]}
```

- **move.** One action lasts `ticks` control ticks of 0.05 s (one tick without `ticks`). `"tcp"` displaces the
  commanded pose of the tool centre point by `delta` = [dx, dy, dz, rx, ry, rz]: a translation (m, each component
  within +/- 2) and a rotation vector (rad, each within +/- pi), both in the cart frame, the rotation applied on the
  left of the current orientation, on a straight line with a smooth start and stop. `"joint"` displaces the six
  commanded joint angles (rad, each within +/- 2 pi). `"hold"` keeps the commanded pose. The displacement always
  starts from the commanded target (`tcp_target`, `q_target`), not from the measured pose. Speed limits: 0.5 m/s and
  1.6 rad/s at the tool centre point, joints [1.2, 1.2, 1.6, 2, 2, 2] rad/s. A move's peak speed is 1.5 times its
  displacement divided by its duration: a move whose peak would exceed a limit, or that the arm cannot follow (near a
  singularity, at a joint limit, out of reach), is shortened, and the observation says `clipped`. A move of one or two
  ticks has no room for a smooth start and stop.
- **guard_force** (0.5 to 1000 N) stops the arm for the rest of the action as soon as the force on the tool has
  changed by more than that many newtons since the action began (the length of the change of the force vector, looked
  at once per tick). The observation then says `guard`.
- **cart.** Drives the cart that many metres along the rail by its odometer (within +/- 2; positive: toward +y), over
  the same ticks and with the same smooth start and stop, whatever the arm does meanwhile. Speed limit 0.25 m/s at the
  peak (0.33 m in an action of 40 ticks); a longer drive is shortened and the observation says `clipped`. Section 5
  says how far the cart then really went.
- **gripper.** The jaws' target opening (m, 0 to 1; anything above 0.016 is the full opening) and their closing force
  (N, 0 to 10000; anything above 600 is 600). Both keys are required. They stay in force in later actions until
  another `gripper` replaces them; a row section starts with the jaws open and a force of 0
  (`../tool/cutter_gripper.md`).
- **cmd.** `"done"` ends the row section after this action's move.
- **cams.** The cameras to take a frame from at the end of this action (section 6).

An action with an unknown key, a wrong type or a number out of range is not a valid action (section 2).

## 4. Observations

```
{"time": s,                       simulated time since the row section started
 "q": [6], "qd": [6],             measured joint angles (rad) and speeds (rad/s)
 "q_target": [6],                 commanded joint angles
 "tcp": [x, y, z, rx, ry, rz],    measured pose of the tool centre point: position (m), rotation vector (rad)
 "tcp_target": [...],             commanded pose of the tool centre point
 "wrench": [fx, fy, fz, tx, ty, tz],   what the world exerts on the tool: force (N) and moment about the flange's
                                  origin (N m), along the cart frame's axes; the tool's own weight is compensated, a
                                  held truss's is not; noise about 0.3 N and 0.015 N m
 "cart": {"odometer": m},         the cart's travel along the rail as its wheels count it (section 5)
 "gripper": {"opening": m, "force": N},   the jaws' opening and the force they press with (tool data sheet, section 4)
 "status": {"clipped": bool, "guard": bool, "stopped": bool, "cart_limit": bool,
            "locked_s": s, "protective_stops": n},
 "trace": {"force": [...], "opening_mm": [...], "grip_force": [...]},
                                  one sample per tick of the action just executed: the size of the force on the tool
                                  (N), the jaws' opening (mm) and their force (N)
 "events": [strings], "frames_left": n, "time_left": s,
 "ended": null | "done" | "sim_cap" | "stops_latched",
 "images": {"overview": {"rgb": .., "t": s},
            "wrist": {"rgb": .., "depth": .., "depth_unit_m": 0.0001, "t": s, "pose": [x, y, z, rx, ry, rz]}}}
```

`tcp` is the pose the kinematic model gives for `q`, `tcp_target` the one it gives for `q_target`; the joint speeds
are differences of `q` over a tick. Nothing on the wire says where a plant, a truss or a fruit is, whether a truss is
ready, or what has been cut, held, bruised or dropped: the readings show what the robot felt, the frames what its
cameras saw.

**Protective stops.** When the force on the tool exceeds 140 N, the moment 45 N m, or a joint lags its command by more
than 0.12 rad (a collision, a pull on something that does not come, a command the arm cannot follow), the commanded
pose jumps back to where the arm was 0.25 s before and the arm stands still for 3 s (`stopped`, then `locked_s`
counting down); moves of the arm commanded meanwhile are ignored. The robot works unattended: **at the third
protective stop of a row section the safety controller latches** and the row section ends there (`stops_latched`), as
a robot that needs a person to reset it does; what was delivered by then counts.

**What ends a row section**: `done`; the simulated time is used up (`sim_cap`, 430 s per row section); the protective
stops latch (`stops_latched`). After the end the arm holds its pose and everything is left to come to rest for 2 s
before the row section is judged (section 8); the greenhouse's rules apply to what happens to a truss in that time
too. A request after the end changes nothing. Nothing on the wire reports a cut, a fall, a bruise, a crushed peduncle
or a lost plant.

## 5. The cart, its odometer and the row markers

The cart rides on the rails of the aisle. It starts a row section at c = 0 and can stand anywhere from c = -0.2 to c =
2.6 m; a drive that would pass an end stops there and the observation says `cart_limit`.

**The odometer** counts the wheels' turns, from 0 at the start of a row section: over a drive it advances by the
distance the action commanded (as shortened, where it was). **The wheels slip.** The cart travels (1 - s) times what
the odometer counts, where the slip s is the row's own (the state of its rails): between 0.02 and 0.09 (2 to 9 %), the
same throughout a row section and different from row to row. On top of that, each drive's travel scatters by 0.4 % of
its length (one standard deviation). The slip is not on the wire.

**Row markers.** 6 square fiducial markers are fixed along the gutter's aisle side, one every 0.5 m: OpenCV's ArUco
dictionary `DICT_4X4_50`, identifiers 0 to 5 in the direction of +y. They do not ride with the cart. They are surveyed
in the row frame:

| marker | centre x, y, z (row frame, m) |
|---|---|
| 0 | [1.02, 0.05, -0.36] |
| 1 | [1.02, 0.55, -0.36] |
| 2 | [1.02, 1.05, -0.36] |
| 3 | [1.02, 1.55, -0.36] |
| 4 | [1.02, 2.05, -0.36] |
| 5 | [1.02, 2.55, -0.36] |

A marker's frame has its origin at the marker's centre and its z axis out of the marker's face, which looks up and
toward the aisle: its axes in the row frame are x = [0, -1, 0], y = [0.7071, 0, 0.7071], z = [-0.7071, 0, 0.7071] (the
face is tilted 45 degrees from the horizontal). The black square is 0.1 m on a side, border included: 4 x 4 cells of
pattern inside a black border one cell wide, 6 x 6 cells in all, on a white plate that reaches 12 mm beyond the square
on every side. In the marker's frame the pattern's top row is toward +y and its left column toward -x, so the square's
corners, in the order OpenCV's detector returns them, are (-a, a, 0), (a, a, 0), (a, -a, 0), (-a, -a, 0) with a half
the side. The bit patterns (1: white) and the poses (4 x 4 matrices, row frame) are in `site.json`, `row_markers`.

## 6. Cameras

Frames are rendered only when an action asks for them, at most 400 per row section in all (each camera of each request
counts one); a request that would pass the budget gets no frame and the event `cams: refused, the frame budget is used
up`. `rgb` is a PNG (base64), `depth` a 16-bit PNG (base64) of the distance along the optical axis in units of
`depth_unit_m` (0.1 mm), 0 where there is no reading. `rowcell_client.decode_rgb` and `rowcell_client.decode_depth`
read them.

- `overview`: colour only, 1280 x 960 pixels, fx = fy = 736.24, cx = 640, cy = 480, no distortion; nominally mounted
  on the cart (cart frame) at [0.47, -1.05, 1.45] (m), camera axes (x right, y down, z forward) as the columns of R =
  [[0.87, -0.3074, 0.3854], [-0.493, -0.5425, 0.6802], [0, -0.7818, -0.6235]].
- `wrist`: colour and depth, 1280 x 720 pixels, fx = fy = 914.01, cx = 640, cy = 360, no distortion; nominally mounted
  on the flange (flange frame) at [-0.075, 0, 0.21] (m), camera axes (x right, y down, z forward) as the columns of R
  = [[0, -0.9397, 0.342], [1, 0, 0], [0, 0.342, 0.9397]].

A pixel (u, v) with depth d is the camera-frame point ((u - cx) d / fx, (v - cy) d / fy, d). Depth is registered to
the colour image: the same pixel grid and intrinsics. The intrinsics are exact (the cameras' factory calibration). The
overview camera looks along and across the row at the plants ahead of the robot; the wrist camera sits beside the
tool, on the side the clamp pad is on (the tool frame's -x), and looks along the tool axis, tilted toward the mouth:
through its nominal mounting the tool centre point lies 0.167 m in front of it, at the pixel (640, 255).

**The mounting above is nominal.** A robot unit's cameras are mounted by hand, and each sits off its nominal pose by
an offset of its own: up to 20 mm along and 2 degrees about each of its own axes for the overview camera (against its
nominal place on the cart), up to 5 mm and 1.5 degrees for the wrist camera (against its nominal place on the flange).
The true pose is the nominal pose times the offset (a translation and a rotation vector in the camera's own frame).
The offset belongs to the robot unit: it is the same throughout a row section and differs from unit to unit. Every row
section is worked by a unit of its own, the graded ones and each practice row alike: a practice unit shows what the
units are like, not what a graded unit's offsets are. Located through the nominal poses, a point 1.8 m in front of the
overview camera (where it looks at the row) is typically 5 cm off (up to about 12 cm), a point 0.35 m in front of the
wrist camera typically 8 mm (up to about 21 mm), and the tool centre point in the wrist camera's view typically 6 mm
(up to about 14 mm). What is exact: the robot's kinematic model (`../robot/ur20.json`) and its joint readings, the
intrinsics, the tool's geometry on the flange, the survey of the row markers (section 5) and of the two boards on the
cart:

| board | origin (cart frame, m) | layout (x by y) | white or black margin (mm) |
|---|---|---|---|
| calibration plate | [0.26, -0.03, -0.2] | 8 x 6 squares of 25 mm | 12 (white) |
| colour card | [0.292, 0.2, -0.2] | 4 x 3 patches of 30 mm at a pitch of 34 mm | 8 (black) |

A board's frame is parallel to the cart frame, with its origin at the board's corner of least x and least y, on its
face. On the plate the square (i, j), counted from 0 with i along x and j along y, is black when i + j is even; the
white plate reaches the margin beyond the squares on every side. On the card the patch (i, j) covers x from i times
the pitch to that plus the patch's side, and y likewise with j; the black card reaches the margin beyond the patches.
Through its nominal mounting the overview camera has both boards whole in its picture when the arm is not in the way:
the plate about 1.7 m away (a square about 11 pixels wide), the card about 1.9 m away (a patch about 12 pixels wide).
A wrist frame's `pose` is the wrist camera's nominal pose when the frame was taken: the flange's pose at the measured
joint angles times the nominal mounting. It does not contain the unit's offset.

**The colour card's patches** are matt; each one's diffuse reflectance (linear RGB) is:

| patch (i, j) | red | green | blue |
|---|---|---|---|
| (0, 0) | 0.9 | 0.9 | 0.9 |
| (1, 0) | 0.59 | 0.59 | 0.59 |
| (2, 0) | 0.36 | 0.36 | 0.36 |
| (3, 0) | 0.19 | 0.19 | 0.19 |
| (0, 1) | 0.09 | 0.09 | 0.09 |
| (1, 1) | 0.03 | 0.03 | 0.03 |
| (2, 1) | 0.6 | 0.07 | 0.06 |
| (3, 1) | 0.1 | 0.45 | 0.1 |
| (0, 2) | 0.06 | 0.1 | 0.5 |
| (1, 2) | 0.8 | 0.65 | 0.05 |
| (2, 2) | 0.85 | 0.35 | 0.05 |
| (3, 2) | 0.6 | 0.1 | 0.45 |

**Depth.** The wrist camera is an active-stereo RGB-D camera. From its data sheet (also in `site.json`, `cameras`,
`wrist`, `depth`):

|  | wrist |
|---|---|
| working range | 0.2 to 2.5 m |
| depth noise (1 sigma, temporal, on a well-lit surface; z in m) | 0.003 z^2 m (0.37 mm at 0.35 m) |
| noise on dim surfaces | rising as the colour image's luma falls below 70, up to twice at 10 |
| noise correlation | over about 2.5 pixels (Gaussian) |
| lateral smoothing of depth | about 1.5 pixels (Gaussian) |
| stereo baseline, along the camera's +x | 0.05 m |
| disparity resolution (depth steps: z^2 x step / (f x baseline)) | 0.08 pixel (0.21 mm at 0.35 m) |
| no reading at depth jumps | of more than 1.5 disparity pixels within 5 x 5 pixels (4 mm at 0.35 m) |
| matcher's window | 31 x 31 pixels (6.8 mm wide at 0.2 m, 11.9 mm at 0.35 m) |

A pixel has no reading (0) outside the working range (nothing nearer than 0.2 m is measured, the tool centre point and
what is in the mouth included); at a depth jump; on anything narrower in the picture than the matcher's window of 31
pixels that stands more than 1.5 disparity pixels in front of what is behind it (the matcher locks onto the surface
that fills its window); where the stereo pair's second view, displaced along the camera's +x by the baseline, cannot
see the point (a band beside near edges); on dark surfaces (where the colour image's luma, 0.299 R + 0.587 G + 0.114
B, is below 30, more often the darker, never a reading below 10); and on bright ones (where a colour channel is above
225, more often the brighter, never a reading above 250). A peduncle (3.6 to 6.4 mm thick) and a leaf's petiole are
narrower than the window at every distance of the working range and give no reading; a main stem (11 to 13.6 mm) gives
none beyond 0.32 to 0.40 m. These thresholds hold for the colour image under the nominal light: the sensor lights the
scene itself, so a surface's readings do not change with the row's light (below). Lost readings come in small blobs.
The noise is independent from frame to frame: the mean of many frames of a still scene approaches the true depth,
smoothed as stated. The overview camera has no depth.

**The colour image** is the renderer's display image: a path tracer with a denoiser, so two frames of the same scene
differ slightly in their pixel values, and a filmic tone curve (the ACES approximation) and the display's gamma lie
between the light a surface sends to the camera and its pixel value. A pixel value is not proportional to a
reflectance.

**The grow light.** The row's lamps differ from row to row in brightness and in colour: the overall brightness lies
between 0.65 and 1.35 times the nominal; against the nominal white, the light's green share lies between 0.35 and 1
and its blue share between 0.45 and 1 of its red share; and each of the three lamp panels over the row gives between
0.75 and 1.25 of its nominal share. All lamps of a row have the same colour. The light stays the same throughout a row
section. The colour image shows all of it; the depth does not depend on it.

## 7. Budgets

Per row section: 430 s of simulated time (`time_left`), 400 frames (`frames_left`), 20000 steps, and 1200 s of your
program's own time: the time the grader waits for your answers, summed (the cell's simulation and rendering are not
counted). Every single answer within 30 s. The time budget and the step limit are checked before each request: the
answer that crosses the budget is still carried out, and the row section ends before the next request.

## 8. How a row section is judged

When a row section has ended and everything has come to rest (section 4), a truss counts when all of this holds:

- it is ready by its cultivar's rule (growing protocol, section 2);
- its peduncle has been cut, and no part of its plant's main stem has (a cut main stem loses the plant with all its
  trusses, wherever they are by then);
- it is whole (no part of its rachis has been cut), it is neither bruised nor squeezed, and its peduncle is not
  crushed (growing protocol, section 5);
- the tool's pad does not hold it;
- it lies in a crate: every one of its fruits has its centre over the inner footprint of one and the same crate
  (section 1) and not above the top of the crate's walls (z = -0.168), and no fruit's lowest point is more than 5 mm
  below the top of the crate's floor;
- it lies in one layer: it does not lie on another cut truss, and no cut truss lies on it. A truss lies on another
  when one of its fruits rests on one of the other's: the two fruits touch (their centres are closer than 1.1 times
  the sum of their radii) and the upper one's centre is higher than the lower one's by more than 0.5 times that sum.
  This is looked at between any two cut trusses, ready or not, in a crate or not, and neither of such a pair counts.

The row section's yield is the number of trusses that count divided by the number of ready trusses in the row section.
Nothing is subtracted: a truss that is not ready counts nothing, cut or not. Where things are is read from the physics
engine, not from pictures.

## 9. Commissioning

The commissioning service in your sandbox runs this cell on 4 practice row sections. They grow the cultivar Ambra, not
the graded rows' Rossa: its colour chart, its readiness rule, its fruits and peduncles and its stiffness ranges are
another cultivar's (growing protocol, section 2). Each practice row is worked by a robot unit of its own: its cameras'
mounting offsets (section 6) are the same in every session on that row and differ from those of the other practice
rows and of every graded unit; and its light, toughness, stiffness and slip are that row's own. A practice row shows
what rows and units are like, not what a graded row's values are. You get what a commissioning engineer has at the
robot: the same actions and observations as above, the camera frames, the robot's, the cart's and the tool's readings,
with the budgets of section 7. You are told nothing else: not where anything is or what state it is in, no yield, no
debrief.

- `rowcell_client.Session(row)` (in `~/given/lib`) opens a session on practice row 1 to 4 and steps it interactively
  (the service listens on `/run/rowcell/commission.sock`); `/opt/venv/bin/python3 ~/commission/run_session.py --row N`
  runs your `run.sh` in a session the way it is run when it is graded, repeated steps included.
- **5 sessions in all, never refunded**, one at a time: a session is counted when it opens and ends when the row
  section ends, when you close it or after 900 s without a request; a session that ends early, for whatever reason, is
  spent. `/run/rowcell/status.json` shows how many are left.
- A session starts Isaac Sim on the sandbox's GPU: opening one takes up to a minute. The service warms up for the
  first few minutes after the sandbox starts (the renderer compiles its shaders once, in a process of the service's
  own); a session asked for in that time waits for it, and is counted as any other.

No simulator model of a row is given. The cell's own code is not readable from your account (the cell runs as the
account `rowcell`): this document, the tool's data sheet and the growing protocol are what there is. Isaac Sim itself
is installed in the image (`/opt/venv`), and the robot's kinematic model is in `../robot/ur20.json`; you may build
simulations of your own.

## 10. What the cell is

The row, the cart and the robot are simulated: rigid bodies on sprung joints in a physics engine for everything that
moves and collides, a renderer for the cameras, and rules for what a physics engine of rigid bodies does not do.

- The cut and the pad's hold are rules on which stalks cross the blade's and the pad's planes inside the mouth, not a
  model of how a stalk fractures or is squeezed (tool data sheet, section 3). The jaws' bodies stand at the full
  opening at all times: closing the jaws moves the jaws a frame shows, not what collides.
- A main stem is a chain of straight pieces, a truss a peduncle, a rachis in three straight pieces and its spherical
  fruits, a leaf one rigid plate with its stalk, each on sprung joints (growing protocol, section 1). Leaves do not
  collide with each other or with stems.
- Bruising, squeezing and crushing are the rules of the growing protocol, section 5, looked at once per control tick;
  whether a truss lies in a crate, and whether it lies on another, is looked at once, when the row section is judged
  (section 8).

The physics engine and the renderer are not bit-reproducible across machines: the same actions do not give the same
readings twice. The grader starts `run.sh` with `PATH`, `HOME` and `LANG` only.
