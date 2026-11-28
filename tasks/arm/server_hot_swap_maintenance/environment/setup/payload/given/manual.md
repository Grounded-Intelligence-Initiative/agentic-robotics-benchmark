# Hot-swap cell: manual

All lengths in metres, angles in radians unless stated. World frame: z up, the floor at z = 0.

## 1. The robots

- Two Franka Pandas (MuJoCo Menagerie `franka_emika_panda`, `~/given/robot/panda/panda.xml`, with its hand), bases
  on the floor: **left** `l` at (0, 0.30, 0), **right** `r` at (0, -0.30, 0), both unrotated (facing +x). Start
  posture (both): q = (0, -0.30, 0, -2.20, 0, 1.95, 0.785398), fingers open.
- **Tool frame** (`ee_pos`, `ee_quat`): the Menagerie `hand` body's frame moved 0.1034 along its z axis (between the
  fingertip pads), same orientation: z along the fingers, y the direction the fingers open along. At the start
  posture the tool points down (z = (0, 0, -1)) with y = (0, -1, 0).
- **Operational-space control**: the tool's target pose is set by each 7- or 14-number action (see the instruction).
  Its stiffness is the arm's task-space inertia times 400 s^-2 (position) and 200 s^-2 (orientation), critically
  damped; a null-space pull toward the posture; the arm's own weight compensated (not the load). A pushed or loaded
  hand sits off its target in proportion to the force: to press, command beyond the contact.
- **Gripper**: the fingers' opening follows a stiff servo toward the commanded opening (grip -1 -> 80 mm, +1 -> 0,
  linear), each finger's force capped at 35 N, speed about 0.1 m/s. The pads have sliding friction 1.2 and some
  torsional friction.
- **Wrist force/torque sensor** at the hand's flange (the `hand` body's origin), reported in the world frame: what
  the hand subtree exchanges with the arm, so it includes the hand's own weight (about 7.5 N); noise 0.3 N and
  0.03 N m per reading, plus a constant bias per server and arm (about 1 N, 0.05 N m).

## 2. The server (rack frame R)

The server stands on a stand in front of the robots. **Rack frame R**: origin at the chassis's front bottom edge,
centred across; x into the server (away from the robots), y to the left, z up. Nominal pose: origin at
(0.31, 0, 0.18), no rotation. **Each server's pose is hidden**: x and y within +-15 mm, z within +-8 mm, yaw (about
z) within +-3 deg.

- Chassis: x 0..0.40, y -0.25..0.25, z 0..0.175 (R); its top at R z = 0.175. Stand below it down to the floor.
- **Drive slots**: 12 vertical slots in a grid on the top, slot k = 4 row + col, row 0..2 at R x = 0.045, 0.100,
  0.155 (row 0 nearest the robots), col 0..3 at R y = +0.117, +0.039, -0.039, -0.117 (col 0 on the left). Each
  slot is a channel 38 mm (x) by 72 mm (y), its floor 0.1462 below the top; the walls between slots run up to the top
  surface. Rail friction coefficient (caddy on walls): hidden, 0.15-0.45.
- **Fan bays**: 4 bays behind the drives at R x = 0.235, y = +0.120, +0.040, -0.040, -0.120 (bay 0..3), each a
  square opening 64 x 64 mm, 40 mm deep.
- **Calibration plate S** on the chassis top, left of the drive grid: a checkerboard of 5 (along R x) by 4 (along
  R y) squares of 15 mm, centred at R (0.100, 0.205), its surface at R z = 0.176. Square (i, j) (i along x from the
  front, j along y from the right, both from 0) is dark when i + j is even; its centre is at the plate centre plus
  ((i - 2) * 0.015, (j - 1.5) * 0.015).

## 3. Drive caddies

- A caddy is a box 32 mm (x) by 66 mm (y) by 150 mm tall, its bottom 12 mm a lead-in 3 mm narrower each side. Mass
  hidden, 0.8-1.2 kg (per server, all caddies alike). Seated, its top stands 4 mm proud of the chassis top.
- **Status LED**: a 2.4 mm square on the caddy's front face (the -x face), 2.4 mm below its top, 18 mm right of
  its centre (-y), under a 3 mm visor: it is seen from in front of the face, and not from above steeper than about
  35 deg. Healthy and seated: green, with short dark flickers (activity). Failed: amber (per server steady, or
  blinking at 1 Hz). A caddy not seated: dark. Brightness hidden (45-100 %).
- A white label (24 x 8 mm) on the front face, left of centre, carries nothing you need.
- **Lever**: a plate 24 mm long, 60 mm wide, 5 mm thick, hinged along y on a bracket 8 mm in front of the caddy's
  front face at the height of the caddy's top. Closed, it lies flat on the caddy's top, its free end toward +x. It
  opens upward and stops at 90 deg (standing up). A spring holds an open lever against its stop.
- **Release button**: on the closed lever's top near the free end, 10 mm (x) by 24 mm (y), 3 mm proud, centred
  7 mm in front of the caddy's centre line and 15 mm left of it (+y). Pressed in 2 mm (of 3 mm travel) against its
  spring, it releases the latch and the lever springs up to its stop. The force at 2 mm is hidden: 4-10 N.
- **Latch**: a closed lever is latched. A released lever latches again when it is swung closed through its detent
  (an opening torque over the last 12 deg, peak hidden 0.12-0.30 N m) while its caddy is seated and the button is
  not pressed; it must have been opened past 12 deg since its release.
- **Connector** (at the slot floor): over the last 5 mm of insertion it pushes back with up to the hidden insertion
  force (10-25 N, full from 3 mm out to home); within 0.8 mm of home the caddy seats (clicks). A seated caddy is held
  in by up to the hidden retention force (8-20 N; four times that while its lever is latched); pulled more than
  1.8 mm out it unseats.
- **The lever is rated to draw a caddy along its rails, not to carry it**: once the caddy is off its rails, a lever
  that carries the caddy's weight alone (nothing else holds the caddy) breaks: its stop gives at a torque of
  0.12-0.17 N m held for 50 ms, below what a hanging caddy of this series puts on it. Take the caddy's weight with
  the other hand while it is still in the rails.

## 4. Fan modules

- A fan module is a box 60 x 60 x 50 mm, 0.15-0.30 kg (hidden). Seated, its top stands 10 mm proud of the chassis
  top. Handle: a bar 44 mm long (along y), 8 x 8 mm, its top 16 mm above the module's top, on two posts.
- **Status LED** on the module's front face (-x), 4 mm below its top, 18 mm left of centre (+y), under a visor as on
  the caddies: green (healthy, seated), amber (failed; steady or blinking per server), dark (not seated).
- **Connector**: as the drives', insertion 6-16 N, retention 6-14 N (hidden), seat within 0.8 mm, unseat at 1.8 mm.
- **Overheat**: a bay without a seated fan for longer than the server's window (hidden, 20-40 s, counted in
  simulated time) shuts the server down.

## 5. Spares, bin, tables

- Side tables, top at z = 0.20: left x 0.18..0.58, y 0.38..0.68; right x 0.18..0.58, y -0.68..-0.38.
- **Spare caddies**: two, standing in a holder on the left table (channels 38 x 72 mm, 60 mm tall), levers open,
  centred near (0.44, 0.43) and (0.44, 0.56) (each within +-6 mm per episode), oriented as in the server (front face
  toward -x).
- **Spare fan**: in a low cradle on the left table near (0.27, 0.46) (+-6 mm), handle along y.
- **Failed-parts bin**: on a stand at the front right, inner 150 (x) by 200 (y) mm, centred at (0.20, -0.12), floor
  at z = 0.206, walls 0.12 high. A removed part counts as binned when its centre of mass lies inside the bin's
  footprint, below its rim + 3 cm (z < 0.35).
- **Calibration plates L and R**: checkerboards of 6 (along x) by 4 (along y) squares of 20 mm, surfaces at
  z = 0.204, centred at (0.26, 0.60) (left table) and (0.25, -0.62) (right table); square (i, j) is dark when i + j
  is even, i counted along +x and j along +y, both from 0.

## 6. Cameras

Frames are RGB (8 bit, sensor noise about 2 levels) with a registered depth image (uint16, millimetres) from an
active-stereo sensor: noise 0.8 mm + 2.5 mm x z^2 (z in m), 1 mm steps, no reading (0) at depth edges, on dark
surfaces (often), at about 2 % of pixels at random and beyond 3 m. The light of the hall varies by server.

Intrinsics (pinhole, no distortion, principal point at the image centre) are in every observation (`K_<cam>`).

- **overhead**, 640 x 480, vertical field of view 60 deg. Nominal pose, OpenCV convention (x right, y down, z
  forward), position (0.25, 0.0, 1.50), rotation (columns: the camera's x, y, z axes in the world):

      [[ 0.0, -0.98639,  0.16440],
       [-1.0,  0.0,      0.0    ],
       [ 0.0, -0.16440, -0.98639]]

  Per server the mount is off by up to 15 mm and 1.5 deg (a rotation in the world frame about the camera's
  position).
- **wrist_l**, **wrist_r**, 320 x 240, vertical field of view 70 deg, on each hand. Nominal pose in the arm's tool
  frame (OpenCV convention): position (0.065, 0.0, -0.0684), rotation

      [[ 0.0, -0.88722, -0.46135],
       [ 1.0,  0.0,      0.0    ],
       [ 0.0, -0.46135,  0.88722]]

  (it looks along the fingers, tilted toward them). Per server each wrist camera's mount is off by up to 4 mm and
  1 deg (in the hand's frame).

## 7. Hidden per server (all drawn uniformly within the ranges; the development servers too)

| Value | Range |
|---|---|
| Rack pose: x, y / z / yaw | +-15 mm / +-8 mm / +-3 deg |
| Overhead camera mount | up to 15 mm, 1.5 deg |
| Wrist camera mounts | up to 4 mm, 1 deg |
| Release button force (at 2 mm) | 4-10 N |
| Lever detent peak | 0.12-0.30 N m |
| Lever break torque (carrying alone, 50 ms) | 0.12-0.17 N m |
| Drive connector insertion / retention | 10-25 N / 8-20 N |
| Fan connector insertion / retention | 6-16 N / 6-14 N |
| Rail friction | 0.15-0.45 |
| Caddy mass / fan mass | 0.8-1.2 kg / 0.15-0.30 kg |
| Overheat window | 20-40 s |
| LED brightness, hall light | 45-100 %, 55-100 % |
| Amber LED steady or blinking | either |

Per episode: which drive and which fan failed (one of each), the spares' places (+-6 mm), the LED flicker.
