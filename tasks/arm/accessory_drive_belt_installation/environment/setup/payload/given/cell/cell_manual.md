# The belt station

## Frames and units

World frame: x forward (from the robots towards the engine), y to the left, z up; the base plate's top is z = 0.
Metres, radians, quaternions w x y z, seconds, newtons, newton-metres. `describe()` returns every number below that a
program needs; `describe_example.json` (next to this manual) is one episode's full answer, every key and unit.

The engine frame **E** (every drive dimension of the work order's drawing is given in it): origin on the crank's axis
in the plane of the accessory face; x to the right and z up as seen from the robots; y into the engine. A point at
depth d in front of the face has y = -d. Nominally E's origin is at world (0.55, 0, 0.30) with x_E = -y_world,
y_E = +x_world, z_E = +z_world; each engine stands on its pallet within +-4 mm, +-1 deg of yaw and +-0.5 deg of roll
of that pose: measure it before you work on the drive.

## The station

* The engine on its pallet in front of the robots, its accessory face vertical, 0.55 m from the robots' bases. The
  drive (variant A, the catalogue's drawing): crank pulley C at the bottom, water pump WP on the left, alternator ALT
  at the top right, the automatic tensioner T to the right with its service lever, the pointer and the window marks.
* The new belt hangs as a loop on a padded peg left of the drive (`describe()["drive"]["peg"]`).
* Two Franka Pandas stand on the base plate 0.70 m apart (bases at world y = +0.35, left, and -0.35, right), both
  facing the engine.
* What the cameras see (base colours, sRGB; the lit appearance varies): base plate grey-beige (140, 135, 128); pallet
  wood (158, 122, 77); engine face and block grey (115, 120, 128); pulleys bright steel (179, 184, 189) with a white
  spoke mark on the front; the tensioner's arm and branch dark blue-grey (64, 69, 77), its service lever orange
  (217, 140, 26); the pointer and the window marks white (242, 242, 242); the belt black (15, 15, 15) with white print
  on its back every 50 mm; the peg blue (51, 89, 153) on a dark post.

## The robots

Two Franka Pandas (MuJoCo Menagerie model in `~/given/panda/`), gravity compensated, their seven joints
torque-controlled by the cell's controller (evaluated every 2 ms). Each arm has its own mode and stiffness:

* `joint` mode: each joint follows its own trapezoidal path towards its target, at up to 1.5 rad/s, accelerating and
  braking at up to 3 rad/s^2 (the joints are not synchronised: between two postures the hand does not move on a straight
  line; a target is kept 0.01 rad inside `joint_range`), with PD gains `stiffness_scale` x (4500, 4500, 3500, 3500,
  2000, 2000, 2000) N m/rad and a damping of 10 % of that in N m s/rad, capped per joint at (88, 1183, 74, 293, 56,
  61, 43) N m s/rad (0.8 x each joint's inertia at the home posture per 2 ms step: more would make a light joint
  chatter). Each joint lags its path by its damping / its stiffness seconds x its speed (min(0.1, cap / (`stiffness_scale`
  x kp)): at full stiffness 0.02-0.03 s for joints 1, 3 and 5-7, 0.08 s for joint 4, 0.1 s for joint 2).
* `cartesian` mode: a Cartesian impedance at the tool centre point (tcp): the tcp is pulled towards a reference pose
  that moves on a straight line towards the commanded pose at up to 0.5 m/s and 1.5 rad/s, like a spring of
  `stiffness_scale` x 3000 N/m (and `stiffness_scale` x 150 N m/rad in rotation), critically damped; a null-space term
  keeps the arm near its inverse-kinematics posture. That posture (`cmd_q` in `get_state`) is solved locally from the
  current one and is kept inside `joint_range`: a joint that reaches its limit stays there, and the tcp can then stop
  short of its reference (by millimetres or more) without any error; `cmd_q` at a limit with the tcp off the commanded
  pose shows it, and a different arm posture (a joint move) frees it. A switch to `cartesian` starts the reference at the actual tcp. In
  contact, a tcp whose reference lies d metres beyond a surface presses on it with about `stiffness_scale` x 3000 x d
  newtons; a held load F displaces the tcp by about F / (`stiffness_scale` x 3000) metres.
* `stiffness_scale` in [0.2, 1.0]. Joint torque limits: 87 N m (joints 1-4), 12 N m (5-7).

The tcp lies between the finger pads, 0.1034 m from the hand frame along the hand's +z (the approach axis); the
fingers close along the hand's y axis.

**Hand data sheet.** Hand + two fingers: 0.76 kg, centre of mass at (-9.6, 0, 31.1) mm in the hand frame. The fingers
open 0-80 mm (50 mm/s) with up to 70 N. Each finger: 28.8 mm wide along the hand's x, its pad (the only part of the
finger that counts as a pad) a block 17 mm (x) by 17 mm (z) from hand z = 94.4 mm to 111.4 mm, its inner face the
gripping face; the pad ends 8.0 mm and the fingertip 8.9 mm beyond the tcp. From a pad's gripping face to the back of
its finger is about 15.5 mm: a finger beside a gripped belt takes about 16 mm on that side of the belt (so a gripped
span cannot be set down closer than that to a pulley's rim or a flange).

**Grip rule** (the cell's only rule; `describe()["grip_rule"]`): a closing gripper (its commanded width below its current
width, while its fingers close towards it: until they reach the commanded width or stop on what they hold) holds what
lies between its pads at the pose of closing:
* a belt link whose centre lies between the pads (hand frame: |x| <= 8.5 mm, z 94.4-111.4 mm, each with 3 mm of slack;
  |y| within the opening) with the belt's thickness across the closing axis within 30 deg: the link is held rigidly by
  the hand;
* the round service lever, its axis passing between the pads within 30 deg of square to the closing axis: the hand holds
  two points of the lever's axis, 5 mm either side of the pads' middle, so **the lever may turn between the pads** (a hand
  carrying the lever round its arc need not keep its orientation; it may turn with the lever).
Opening the fingers (commanded width more than 2 mm above the current width) lets go. The hold slips (lets go) when the
held thing pulls with more than 30 N, or when the moment about the grip on a held belt exceeds 1.0 N m about any axis
(bending it flatwise or edgewise, or twisting it), for 50 ms. Open fingers and the backs
of closed fingers otherwise touch the belt without friction, so a span can be pushed with them. The belt meets a
finger pad on its flat face (the pads' small ridges do not touch the belt). A held belt link or lever no longer touches
the hand that holds it; it touches everything else.

**Force/torque sensor** (each arm): at the flange, in the hand frame: the force (N) and torque (N m) the arm exerts on
the hand and everything it holds, the hand's own weight included (about 7.5 N), mean over the 10 ms tick, with 0.3 N /
0.02 N m of white noise per axis and an unknown constant bias per episode (within +-1 N, +-0.05 N m); `tare(arm)` zeroes
it at its current reading. After the hand turns, the hand's weight reads as a change: compensate it from the hand data
above, or tare again. Sign: with the hand pointing straight down at rest (before any tare) it reads about 7.5 N along the hand's -z (the arm
holds the hand up).

## Cameras

* `front`: fixed above and between the robots' bases (world (0.05, 0, 0.80)), looking over the drive; 1280 x 960,
  vertical field of view 50 deg, RGB-D. Depth valid 0.30-3 m, 1 mm steps, noise 1.0 mm + 2.5 mm/m^2 x z^2 (2.6 mm at
  0.8 m).
* `left_wrist`, `right_wrist`: on each hand (D405 class), 58 mm to the hand's -x side and 62 mm behind the tcp, looking
  at the point 0.15 m ahead of the tcp; 640 x 480, vertical field of view 58 deg. Depth valid 0.07-0.50 m, 0.1 mm steps,
  noise 0.2 mm + 1.5 mm/m^2 x z^2 (0.23 mm at 0.15 m, 0.29 mm at 0.25 m).
Their calibration is exact and published in `describe()["cameras"]` (OpenCV convention; the fixed camera's pose, the
wrist cameras' mounts in the hand frame); every frame also carries `T_world_cam` at capture. Depth is planar along the
optical axis; a pixel reads no depth on a depth discontinuity and on 2 % of the pixels at random. Colour: per camera and
episode a channel gain within 0.94-1.06 and a brightness within 0.92-1.08, then about 2 grey levels of noise.

## Safety and damage

* Any robot link or hand surface, the finger pads excepted, pressing on anything (the engine, the drive, the belt, the
  peg, the other arm) with more than 30 N for 30 ms is a protective stop: both arms stop, the episode ends at once
  (`get_state()["stopped"]` and every later `step` say so; no reason is given) and the engine scores 0.
* The tensioner's damper is rated for 60 deg/s of arm speed. Faster (a snap-back when the lever is let go off the
  belt, a violent swing) damages it, and the engine is rejected (scores 0). A belt that touches the base plate is
  scrap (the engine scores 0). Damage is not reported during the episode.

## Initial state

Each episode starts with both arms in `joint` mode at `stiffness_scale` 1.0, at their `home_q`, grippers open (80 mm),
the force/torque sensors not tared (their bias included); the belt hanging on its peg; the tensioner at its closed stop.

## Acceptance (after `finish`)

The robots are switched off and out of the drive; the drive settles for 1 s; a run-in motor turns the crank at
1.5 rev/s for 4 s against the accessories' drag. Then, per engine: g = 1 if nothing is damaged, there was no
protective stop and the tensioner's pointer rests between its window marks, else 0; q = (the pulleys that pass + 1 if
the run-in slip is within spec) / 5, where a pulley passes when the belt wraps it on the right side (ribs on C, WP and
ALT, the back on T) and lies in its groove zone on the groove floor (C, WP, ALT: not on a shoulder or edge) or on the
face between the flanges (T); the slip is within spec when each accessory turned within 3 % of the crank's angle times
its ratio. The engine's score is g x q.

## Time and budgets

The cell advances only inside `step(n)` (n ticks of 10 ms). Per episode: 1500 s of simulated time, 2400 s of program
time (the wall clock minus the cell's simulation and rendering time), 600 camera frames, at most 120 s between a reply
and the next request.
