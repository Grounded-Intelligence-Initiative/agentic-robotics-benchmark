# The skid cell: the integrator's manual

This is the integrator's description of the robot cell your program drives. Everything stated here as exact is exact;
the skid itself is described in `../equipment_manual.md`, the procedure in `../sop.md`, the wire in `cell_api.md`.

## 1. Frame

World frame = the skid's frame: z up, the floor at z = 0, the skid's front plane y = 0 (the skid occupies y >= 0, its
operable parts protrude towards -y), x along the skid from 0 to 5 m. The robot's rail runs along x at y = -0.70.

## 2. The robot

* **Panda.** MuJoCo Menagerie's `franka_emika_panda` with its hand (copy in `~/given/panda/`, Apache-2.0), on the lift
  of a carriage. The base (link0's origin) stands at (x, -0.70, lift) with its x axis along world +y: the arm faces the
  skid. Joint names `panda/joint1..7`, ranges and the home posture in `describe()`. The links are gravity-compensated.
* **Rail and lift.** The carriage moves along x in [0.30, 4.70] at up to 0.30 m/s; the lift sets the base height in
  [0.05, 0.60] m at up to 0.15 m/s (`command_rail`). Their drives are stiff position servos; `get_state()["rail"]` gives
  where they are. The carriage and the lift do not collide with anything.
* **Reach.** With the tool pointing horizontally into the skid (+y) the Panda reaches handles about 0.5 to 1.1 m above
  its base and 0.4 to 0.6 m in front of it; set the lift so the handle you work on is about 0.8 m above the base.
* **Control** (`set_control`, `command`): `joint` mode tracks joint targets (rate-limited, 1.5 rad/s); `cartesian` mode
  tracks a world-frame pose of the `panda/tcp` site with one damped-least-squares IK step per 10 ms tick on the
  commanded configuration (0.5 m/s, 1.5 rad/s). The joints are position servos (Menagerie gains: 4500 / 3500 / 2000
  N m/rad and 10 % of that in damping): a moving target is tracked about 0.1 s late, and `stiffness_scale` (0.2..1)
  scales their stiffness.
* **Tool.** `panda/tcp` lies between the finger pads, 0.1034 m from the hand along its +z (the approach axis); +y of
  the tcp is the fingers' closing axis. The fingertips end about 9 mm beyond the tcp along +z. The hand is about
  0.2 m wide along its closing axis. Width 0..0.08 m (`command(width=...)`), ramped at 0.05 m/s.
* **Wrist camera** `wrist` rides on the hand (section 5).

## 3. The grip rule (how a handle is held)

A valve handle is held by the cell's grip rule, not by friction. When the fingers close (a `width` below the current
opening) and, while closing, the opening reaches a handle's width + 6 mm with that handle inside the pinch box, the
handle's moving part (a handwheel, a lever, a knob, the selector's wing) is attached rigidly to the hand at the pose of
that instant (the event `grasped`), and the fingers stay at the handle's width. The pinch box, in the tcp frame: |x| <=
12 mm (across the pads), |y| <= half the opening (along the closing axis), -12 mm <= z <= +14 mm (along the approach).
The tcp's approach axis (+z) must lie within 30 degrees of the handle's approach direction (+y, into the skid), and its
closing axis within 30 degrees of the handle's pinch axis:

| handle | where | pinch axis | width |
|---|---|---|---|
| handwheel spoke | middle of each spoke | across the spoke, in the wheel's plane | 13 mm |
| handwheel rim | on the rim midway between two spokes | along the radius | 18 mm |
| lever | at 35, 55, 72 and 90 % of the lever's length | across the lever, in its plane of turning | 20-26 mm |
| needle-valve knob | the knob's centre | any axis across the stem | 38 mm |
| needle-valve knob (small) | the knob's centre | any axis across the stem | 32 mm |
| selector wing | the wing's middle | across the wing | 15 mm |

The held part moves with the hand only as its own joint allows (a handwheel turns about its stem, nothing else): move
the hand on that circle. The grip carries at most 60 N or 6 N m: a load above either for 30 ms slips the handle out of
the hand (`slipped`). Opening the fingers wider than the handle releases it (`released`). The events do not name the device. An 80 mm opening
straddling a spoke of a small wheel touches the rim: open just enough. Buttons are pushed with the fingertips (fingers
closed), not held.

## 4. Safety

The cell watches the contacts between the robot and the skid (and the floor) every 10 ms tick. A robot link or finger
pressing on anything of the skid but a handle or a button with more than 40 N, or on a handle or a button with more
than 150 N, for 3 consecutive ticks, is a safety violation: the episode scores 0 (`safety:<what>` in `step`'s events).
The robot never collides with itself, with the carriage or with a part it holds.

## 5. Cameras

| name | mount | size | fovy |
|---|---|---|---|
| `overview` | fixed, behind the rail, high: the whole skid (too coarse to read a dial, a tag plate or the analyser) | 1280 x 720 | 50 |
| `wrist` | on `panda/hand`: 0.05 m behind the tcp and 0.045 m to the hand's +x side, looking at the point 0.15 m ahead of the tcp; image up = the closing axis | 640 x 480 | 70 |

`describe()["cameras"]` gives the exact intrinsics (fx = fy = (H/2) / tan(fovy/2), cx = W/2 - 0.5, cy = H/2 - 0.5,
pixel (u, v) = (column, row)), the fixed camera's exact pose `T_world_cam` and the wrist camera's mount `T_body_cam`
(OpenCV convention: x right, y down, z forward). A wrist frame carries its `T_world_cam` at the frame's instant. The
colour images carry a little sensor noise (sigma 2 levels); depth is the planar distance along the optical axis in
millimetres (0 = no reading; 2 % dropout, 1.5 mm noise, 0.1-6 m). Lights vary a little from skid to skid.

## 6. Time and budgets

The cell's clock advances only inside `step` (10 ms per tick); the process (flows, levels, alarms, the window's events)
advances with it. Your program's own computing time is not simulated time but is capped per episode (`cell_api.md`).
The schedule: the window opens at 600 s, the shift ends at 1500 s, the episode ends at 1800 s.

## 7. The client library

`~/given/lib/skidcell/client.py` is the one part of the cell your program imports: the proxy (`Cell`), `decode_frame`
and the program launcher the grader uses. The cell itself (the skid, its process, the robot's physics, the scoring)
runs in the grader, never in your account: there is no simulator of the skid to rehearse on. What you know of a skid
before the shift is what this manual, `cell_api.md`, `cell_config.json`, the operating procedure, the P&ID, the
equipment manual and the instrument catalog say; on the shift, your program learns the rest through the cameras and
the robot's state, as an operator would.
