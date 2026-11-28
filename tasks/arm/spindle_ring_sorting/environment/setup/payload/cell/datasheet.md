# Ring re-sorting cell RS-7: hand-over survey

The cell re-sorts anodised aluminium spacer rings. Mixed lots arrive on a fixture of seven vertical spindles; a
Franka Panda with long parallel jaws takes the rings off one at a time and threads them onto other spindles until
every product variant sits on a spindle of its own.

This is the integrator's hand-over survey, not a drawing set: the original drawings and the cell's simulation model
did not come with the cell. Dimensions are as surveyed on the floor, with the measuring uncertainty or the spread
found between parts; where only a range is given, nothing more precise is known. Every episode is a real fixture on
the cell as it is that day, and its actual values (section 9) are never told.

## 1. Frames and units

World frame: x away from the robot, y to the robot's left, z up. The table top is the plane z = 0.800 m. The robot's
base stands at (-0.56, 0, 0.80) facing +x. Metres, radians, seconds, newtons. Quaternions are (w, x, y, z).

## 2. The fixture (as surveyed)

- **Base plate**: 90 x 524 x 12 mm (x by y by z), dark grey, lying on the table. Its centre is at (0.070, 0.000) and
  its long side along y; its top face is at z = 0.812.
- **Spindles**: seven, numbered 0 to 6, standing upright on the plate's centre line at y = (k - 3) x 0.068 m
  (k = 0..6), x = 0.070: a pitch of 68 mm. Each is a ground steel pin of **14.5 mm diameter**, **74.0 mm** above the
  plate top (its tip at z = 0.886), with a **small chamfer** at the tip (surveyed as 0.2 to 0.5 mm) and a 25 mm collar at its foot.
- **Capacity**: a spindle holds at most **4 rings** (a full stack leaves about 9 mm of the spindle above the top
  ring). A ring placed higher is not on the spindle.

## 3. The rings (as surveyed)

- Outside diameter **40.0 mm**, bore **15.7 to 16.2 mm** (the survey's spread; 1.2 to 1.7 mm of diametral clearance on a
  spindle), height **16.0 mm**,
  plain faces on both sides (a ring is symmetric: either face down is fine), with three 0.2 mm locating pips on each
  ring's lower face: in a stack the rings sit at a **pitch of 16.2 mm**, the lowest one's lower face 0.2 mm above the
  plate. Mass 45 g. Friction against the spindle and other rings: not surveyed (steel on anodised aluminium,
  dry).
- Six product variants, four rings each (24 rings per fixture). They come in three anodising colours, and every
  colour in two builds that look exactly alike: a plain aluminium ring and a **steel-cored** one (a steel sleeve
  pressed into the aluminium body, the same outside, bore and faces, the same anodising). Only the mass tells them
  apart:

| variant | colour | build       | mass  | nominal RGB (linear, 0..1) |
|---------|--------|-------------|-------|----------------------------|
| A       | blue   | plain       | 45 g  | (0.10, 0.22, 0.72) |
| B       | blue   | steel-cored | 110 g | (0.10, 0.22, 0.72) |
| C       | red    | plain       | 45 g  | (0.72, 0.10, 0.10) |
| D       | red    | steel-cored | 110 g | (0.72, 0.10, 0.10) |
| E       | green  | plain       | 45 g  | (0.14, 0.52, 0.22) |
| F       | green  | steel-cored | 110 g | (0.14, 0.52, 0.22) |

  The anodising varies from ring to ring (section 9), and its sheen gives highlights under the lamps. Nothing on a
  ring shows its build; a ring held still in the jaws weighs 0.44 N (plain) or 1.08 N (steel-cored) on the wrist
  sensor (section 7). The two builds of a colour are different products: a spindle holding both is not sorted.
- A ring comes off a spindle only over the spindle's tip, so only the top ring of a stack can be taken.

## 4. An episode

- **Start**: six spindles carry four rings each, in a mixed order; one spindle is empty (any of the seven). No
  spindle starts with a single variant on it. Every start is a random arrangement reached from a sorted fixture by
  legal moves (any top ring onto any spindle with room), so it can be sorted. The arrangement is not told: you see it.
- **Goal**: every variant on a spindle of its own, all four of its rings on it, and nothing else on that spindle. Which
  spindle a variant ends on is your choice.
- **Length**: at most **10000 control steps** (1000 s). The cell checks the fixture every step: once every variant is
  complete on its own spindle and stays so (no ring held, every ring at rest) for 20 steps in a row, the episode ends
  early.
- **Credit** (at the end of the episode): a ring is **sorted** when it rests threaded on a spindle (its bore around
  the spindle, below the tip, within 8 degrees of the spindle's axis), is not touched by the jaws, is not damaged, and
  every ring on that spindle is of its variant and undamaged. A spindle is **complete** when it holds all four rings of
  one variant that way. A ring anywhere else (on the table, on the plate, lying on a stack, held) is not sorted.
  The episode's score is `0.8 x (sorted rings / 24) + 0.2 x (complete spindles / 6)`.
- **Damage**: a ring pressed against a spindle hard (the contact force between that ring and any spindle above 25 N
  for 3 control steps in a row) gets a scored bore and is scrap for the rest of the episode: it is never sorted, and
  the spindle it ends on is not pure. Forcing a ring that is cocked on a spindle, or pushing one down on a spindle's
  tip, does this. Pushing a stack down from above does not (the rings press on each other, not on the spindle).
- **The pallet moves.** The fixture's base plate sits on a pallet in a conveyor station whose stop is worn. Two or
  three times per episode, at unannounced moments while the hand is clear of the fixture with its jaws open and empty
  (the cell's interlock lets the conveyor index the neighbouring pallets only then), a passing pallet bumps it:
  within about 1 s the whole fixture (plate, spindles and
  the rings on them) shifts by 6 to 15 mm in some direction and turns by up to 1 degree. It then stays there until the
  next bump. Over an episode the fixture stays within 20 mm of where it started. Nothing signals a bump; the cameras
  see it.
- A ring that falls stays where it lands (on the plate, on the table, leaning against a stack). It can be picked up
  again; one that leaves the table is lost.
- **Safety stops**: the episode ends at once if an arm joint moves faster than 8 rad/s, a ring faster than 8 m/s or
  the simulation of the cell becomes unstable; it is scored as the fixture stands then.

## 5. The robot and its jaws

- Franka Emika Panda with its standard hand, with the
  stock fingers replaced by **long jaws**: each finger carries a steel jaw plate (21 mm wide, 6 mm thick, 79 mm long)
  ending in a rubber-faced pad, **20 mm wide (across the closing direction) and 12 mm tall**, its face 4 mm proud of
  the plate. The **TCP** (`ee_pos`) is the point midway between the two pads' centres, **160.4 mm below the hand's
  flange face** (the gripper base). The pads' lower edges are 6 mm below the TCP; the jaw plates do not reach lower.
  The hand's body (palm) is 94.5 mm above the TCP and 63 mm wide across the closing direction, 208 mm long along it.
- The jaws move together, symmetrically about the hand's axis: the opening between the pads is 0 to 80 mm. A ring
  gripped across its outside is centred between the pads along the closing direction; along the pads' faces it stays
  where it was when the jaws closed. Finger force is capped at 20 N per finger.
- Pad friction against a ring: high (rubber faces); not surveyed.

## 6. The cell's controller

Control runs at 10 Hz (one action per 0.1 s). An action is a list of **six numbers** `[dx, dy, dz, dyaw, grip, cams]`:

- `dx, dy, dz`: displacement of the TCP target from the TCP's **current** position, world frame, metres; the vector
  is clipped to a length of 0.02 m. The target is kept within x in [-0.30, 0.30], y in [-0.40, 0.40],
  z in [0.804, 1.20].
- `dyaw`: rotation of the hand about the world vertical, radians, clipped to +-0.15. The hand always points straight
  down. `ee_yaw` is the hand's heading: the jaws close along the horizontal direction (cos ee_yaw, sin ee_yaw); it is
  pi/2 at the start (jaws closing along y). Keep `ee_yaw` within about [-0.2, pi + 0.2] (the hand's last joint has
  a limited range).
- `grip`: -1 = fully open (80 mm) ... +1 = closed; values in between command the opening linearly
  (opening = 0.04 x (1 - grip) m); the pads stop on whatever is between them.
- `cams`: which images the next observation carries: 0 = none, 1 = overview, 2 = wrist, 3 = both. Images cost the
  cell time to render, so ask for them when you need them.

The tracking law is an operational-space controller: each step the target is the current TCP plus the commanded
displacement, and the arm is driven toward it with a stiffness of 400 (N/m)/kg times the arm's effective mass at the
TCP, critically damped, with gravity compensation. It is compliant: a TCP blocked by contact pushes with that
stiffness times the commanded displacement (over the fixture, straight down, about 4 kN/m: a blocked 2 mm step
pushes with about 9 N, a blocked 20 mm step with far more than a ring survives), and a commanded displacement that
is not reached is not accumulated (the next step's target starts again from where the TCP is). Across the hand the
arm is softer. The arm's own weight is compensated; a held ring is not.

## 7. The observation (every step)

| key       | meaning |
|-----------|---------|
| `t`       | control step index in the episode, from 0 |
| `q`       | the 7 arm joint angles (rad) |
| `ee_pos`  | the TCP position, world frame (m) |
| `ee_quat` | the TCP frame's orientation, world frame (w, x, y, z); the TCP frame's z axis points down along the hand |
| `ee_yaw`  | the hand's heading (section 6) |
| `gripper` | the opening between the pads (m) |
| `wrench`  | force (N) and torque (N m) measured by the wrist force/torque sensor at the hand's flange, in the **world** frame: what the arm feels from the hand, so it includes the weight of the hand and jaws (about 5.4 N along +z at rest) and of a held ring (0.44 N plain, 1.08 N steel-cored); a downward push of a ring on a spindle tip reduces `wrench[2]`. The sensor has a zero offset that differs every episode, a slow zero drift and noise of a few tenths of a newton per reading (the vendor's figures are not part of the hand-over; measure them). |
| `overview`| RGB image, 240 x 180 (width x height), of the overview camera, or an empty image (shape [0, 0, 3]) unless the previous action asked for it |
| `wrist`   | RGB image, 128 x 96, of the wrist camera, or empty unless asked for |

The first observation of an episode carries both images. Images are rows top to bottom, RGB, 8 bits; on the wire
`{"dtype": "uint8", "shape": [H, W, 3], "b64": <base64 of the raw bytes, row-major>}`.

## 8. The cameras (published calibration)

Pinhole cameras with square pixels, principal point at the image centre (the centre of pixel (u, v) is at
(u + 0.5, v + 0.5)), no distortion. A camera frame here has x to the right of the image, y up the image and looks
along its -z axis.

- **Overview camera**, fixed on the cell's frame at the front right: position (0.52, -0.42, 1.22), looking at
  (0.06, 0.00, 0.84) with the image's up direction toward world +z; vertical field of view 40 degrees.
- **Wrist camera**, on the hand: position (0, 0.05, -0.1604) in the TCP frame (i.e. 160.4 mm above the TCP, 50 mm off
  the hand's axis along the TCP frame's y axis), its axes in the TCP frame: x_cam = -x_tcp, y_cam = +y_tcp,
  z_cam = -z_tcp (it looks straight down along the hand); vertical field of view 75 degrees. The jaws are in its
  view.

The lamps' brightness differs from day to day, and with it every image.

## 9. Tolerances: how real episodes differ from the survey

Every episode is a real fixture on the cell as it is that day. Each of these has its own actual value within the
given range (uniform, independent), drawn anew for every episode; none is ever told:

| what | tolerance |
|------|-----------|
| fixture (base plate) position | +-4 mm in x and in y |
| fixture heading | +-0.8 degrees about z |
| each spindle's position on the plate | +-1.5 mm in x and in y |
| each spindle's lean | up to 1.0 degree from vertical, in any direction |
| each ring's colour | +-0.03 on each RGB component of its variant's nominal |
| lamp brightness | +-25 % |
| overview camera mounting | +-4 mm per axis, +-0.4 degrees about each axis; focal length +-1 % |
| wrist camera mounting on the hand | +-2 mm per axis, +-0.6 degrees about each axis; focal length +-1 % |
| wrist sensor | zero offset, drift and noise as in section 7 |

The spindle diameter, the chamfer (within its surveyed range), the jaws and the robot do not change between episodes.
