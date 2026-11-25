# The block tower cell

A Franka Panda stands at the world origin (z up). In front of it, on a turntable, stands a tower of wooden blocks.
The job: take blocks out of the lower part of the tower and stack them on its top, building new levels, without the
tower collapsing. Every number below is also in `cell_config.json` (`describe()` returns it too).

## 1. The tower

* 18 levels of 3 blocks. A block is nominally 75 x 25 x 15 mm (length x width x height), about 16.5 g. The three
  blocks of a level lie side by side, 26.5 mm apart (centre to centre, 1.5 mm gaps), along the level's axis; each level
  is turned 90 deg against the one below. At turntable angle 0 the blocks of level 0 lie along world x; slot s of
  level k sits at (s - 1) x 26.5 mm along the level's lateral axis (-sin yaw_k, cos yaw_k), yaw_k = k x 90 deg.
* The turntable's axis is at (0.65, 0.0); its top, where level 0 rests, is at z = 0.115 m; the table is 60 x 60 mm,
  smaller than the tower's 78 x 75 mm footprint, so blocks of level 0 overhang it. Levels are about 15 mm apart.
* Real towers are not exact. Per tower (hidden, different in every tower):
  * the tower stands off the turntable's axis by up to +-3 mm in x and y and is turned by up to +-1.5 deg;
  * every block sits up to +-2 mm off its nominal place along its length, +-0.3 mm sideways, turned up to +-0.6 deg;
  * every block has its own friction coefficient (0.30-0.50), mass (14.5-18.5 g) and height.
* **Which blocks carry the tower.** Within a level the blocks differ in height by fractions of a millimetre: the
  tallest ones carry everything above, the others (shorter by 0.12-0.30 mm) carry nothing and are loose. A level's
  bearing blocks are the middle one alone, both side blocks, the middle and one side block, or all three. Which ones
  they are is not given and does not show in a camera image; it can be found out by touch.
  * A loose block slides along its length when pushed with about 0.1-0.5 N.
  * A bearing block is clamped by the weight above it: pushing it takes roughly 1-10 N, and before it moves it drags
    the levels above it along. A firm push on a bearing block topples the tower. A gentle push that stops in time
    (well below 1 N for blocks near the top, which carry little) leaves the tower standing.
* A block pushed out far enough that its centre of mass passes the tower's edge tips; a block pulled out while it
  still presses on the level above (a grip too high or too low by a tenth of a millimetre levers it against that
  level) drags that level with it.

## 2. The robot and its tools

* The Panda (MuJoCo Menagerie model, `~/given/robot/panda/`), joint position servos, gravity compensated. Its home
  posture: `home_q`. The cell's controller moves a tool frame along a straight line to a goal pose (orientation turned
  at a capped rate), or the joints to joint goals (`cell_api.md`).
* Tool frames: `tcp`, the point between the finger pads, 0.1034 m along the hand's z axis from the hand's origin;
  `rod_tip`, the tip of the pusher rod.
* **Gripper**: parallel fingers, opening 0 - 0.08 m; open to a width, or close with a squeeze force (N, up to 40).
  The finger pads are 17 mm wide (along the hand's x axis) and 17 mm tall.
* **Pusher rod**: a 6 mm rod mounted under the palm, along the hand's +x axis, 14 mm above the tcp (toward the
  palm), from 0.02 m to 0.14 m along the hand's x axis. It can be deployed or stowed; stowed, it touches nothing.
* **Wrist force/torque sensor**: the force and torque the flange exerts on the hand assembly (hand, fingers, rod), in
  the world frame, 1 kHz internally, reported at each reply with noise (sigma 0.02 N, 0.002 N m) and a fixed bias
  of a few hundredths of a newton. It includes the hand's own weight (about 7 N) and its accelerations: tare it. An
  upward push on the fingers reads as a negative z force.
* **Turntable**: turns the tower to an absolute angle (0.6 rad/s, 0.6 rad/s^2 profile). The tower turns with it (the
  blocks rest on it by friction; slow turns move nothing).
* **Staging post**: a 30 x 24 mm post at (0.42, 0.28), top at z = 0.115, to set a block down on and grip it again.
* **Cameras** (RGB-D, PNG colour + 16-bit depth, noise: colour sigma 2 levels, depth 0.5 mm): `left` and `right`,
  fixed, looking at the tower from the front left and front right (480 x 360); `wrist`, on the hand (320 x 240).
  Poses (OpenCV convention) and intrinsics come with every frame and from `describe()`.

Reach: the arm reaches the tower's robot-facing side comfortably; reaching behind the tower, or far over its top, is
at the edge of its workspace. The cell can turn the tower instead.

## 3. Rules and score

* **Removing a block.** A block of the tower whose centre has moved more than 40 mm from where it stood (measured in
  the turntable's frame) has been removed. The removal is legal only when the block's level was below the
  second-highest complete level (a level is complete while it has all three blocks) at that moment: at the start the
  top two levels (16, 17) may not be touched; once a new level is complete on top, level 16 may be used, and so on.
* **Placing a block.** A removed block counts as placed on top when it comes to rest, not touching the robot, in a
  free slot of the current top level (or, when the top level is complete, of the new level above it): lying across
  the level below (within 5 deg of square to it), resting on it (its centre within 1.5 mm of the height of a block
  lying on that level), its centre within 4 mm of a slot centre (the level below's centre +- 26.5 mm or 0 across the
  new level) and within 6 mm of the level below's centre along its own length. The cell checks every 0.5 s and once
  more at the end. A placed block is part of the tower from then on (and may be taken again under the same rule).
* **Collapse.** If any block of the tower (original or placed) ever falls more than 5 mm below where it stood, or
  tilts more than 5 deg, the tower has collapsed. This is permanent.
* **Score of a tower** = 0 if it collapsed at any time; else (blocks legally removed and placed on top, still in their
  slots after a final settle of 3 s at the end of the episode) / N_cap, N_cap = 30, at most 1. Blocks that end
  anywhere else (on the staging post, in the hand, on the floor) do not count and do not cost anything.
* **Episode**: 2 400 s of simulated time per tower; it ends earlier when your program finishes, ends, crashes or runs
  out of a budget. Stopping when you judge the next block too risky is part of the job: one collapse zeroes the tower.
* Your result: the mean of the tower scores over four hidden towers (different seeds; their tightness patterns, block
  friction and offsets lie in the ranges above; some have many bearing blocks in the middle levels, or low friction).
  It is compared with what a reference program scored on the same towers; the reference ran with a privileged token
  that showed it the true block poses, which blocks carry load, and whether a pulled block touches the level above.

## 4. Practice

`~/commission/run_tower.py` runs your program on the commissioning service: the same cell, on six practice towers
(drawn like the hidden ones, never graded), with the same budgets. 10 episodes per session, 2 at a time; an episode of
the full 2 400 s takes 20-40 minutes of wall time. No score, no ground truth: you see what your program sees.
