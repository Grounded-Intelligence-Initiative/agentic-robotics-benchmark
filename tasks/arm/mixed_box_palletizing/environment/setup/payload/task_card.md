# Task card: the palletizing cell

SI units, radians. World frame: z up, floor at z = 0, the arm's base axis is the world z axis.

This card is the cell as drawn. What the built cell differs in, and by how much at most, is in the instruction
("What differs from the drawing") and marked here with *(drawn)*.

## Layout

| Thing | Where |
|---|---|
| Pick station | end of the infeed conveyor, centred at (1.45, 0), conveyor surface at z = 0.60. The conveyor runs along world x and is 0.8 m wide. A carton arrives within 40 mm of the centre, its long side along the conveyor or across it, skewed by up to 20 degrees |
| Pallet *(drawn)* | centred at (0, 1.50): 1.2 m along world x, 0.8 m along world y, deck at z = 0.144. The stack may reach 1.00 m above the deck. The real pallet stands up to 25 mm (per axis) off and up to 2 degrees turned; its size and deck height are exact |
| Buffer table | centred at (-1.45, 0): 1.1 m along x, 1.4 m along y, surface at z = 0.60 |

## Arm

Four axes. `q1` turns the arm about the base axis (range -0.80 to 3.95 rad; 0 points along world x, toward the
pick station). The shoulder axis is 0.25 m from the base axis, 0.80 m high. `q2` is the upper arm's angle from the
vertical (upper arm 1.20 m), `q2 + q3` the forearm's angle below the horizontal (forearm 1.30 m). A parallelogram
keeps the wrist level, so the tool axis is always vertical; it is 0.20 m beyond the wrist pivot, and the pad's
face is 0.55 m below the pivot. `q4` turns the tool; the tool's yaw in the world is `q1 + q4` (`q4` range
+-4.2 rad). `arm_kinematics.py` is this kinematics; the lengths are exact, the joint readings are not (each is
off by a constant, new in every order: up to 0.5, 0.3, 0.3 and 1.0 degrees for `q1` to `q4`). The arm's controller
works in the readings: a velocity command moves its target as `arm_kinematics.py` computes it from the readings. The arm's links collide with cartons
and with the conveyor, table and pallet: the forearm comes in above the wrist, so a place deep behind a tall
stack may be out of reach although the pad itself would fit.

The joints are position-servoed to a target that your velocity commands move (see the instruction). The arm's own
weight is compensated; a payload makes it sag by a few millimetres, which `q` and `tcp` show. The target never
runs more than 0.12 m ahead of the arm.

## Gripper

A foam vacuum pad, 0.20 m x 0.14 m (long side along the tool's x axis at yaw 0), on a spring-loaded cup: 30 mm of
stroke, 1200 N/m, so pressing the pad on a surface compresses the cup and shows as a positive `Fz` (12 N at
10 mm). The cup weighs 1.0 kg.

- **Sealing.** With the vacuum on, the pad seals on a carton when its face is within 4 mm below to 2 mm above the
  carton's upward face (level within 6 degrees) and at least 35 % of the pad's area lies on that face.
- **Holding.** Holding force = 30 kPa x the area of the pad that lies on the face (840 N with the whole pad on).
  Holding moment = 0.15 x holding force x sqrt(that area) (21.1 N m with the whole pad on). The seal breaks when
  `(pull + 0.5 shear) / holding force + moment / holding moment > 1`. The load is what the wrist sensor measures,
  without its noise and less the cup's own weight, low-pass filtered with a 10 ms time constant: pull is the
  downward force, shear the horizontal force, moment the torque about the pad-face centre about horizontal axes.
  A carton of 15 kg hanging 80 mm off its centre of gravity uses 0.73 of that at rest. After a break, switch the
  vacuum off to re-arm.

## Cameras

| Name | Bit | Image | Looks at |
|---|---|---|---|
| `infeed` | 1 | 800 x 600 | the pick station, from about 2.0 m above the conveyor |
| `pallet` | 2 | 640 x 480 | the pallet, from about 3.9 m above the deck |
| `buffer` | 4 | 640 x 480 | the buffer table, from about 2.4 m above it |

Drawn mounts (OpenCV convention: the optical axis is the camera's z, image x to the right, image y down):

| Name | Position | Looks | Image x points along | Image y points along | Focal length (px) | Principal point (px) |
|---|---|---|---|---|---|---|
| `infeed` | (1.45, 0, 2.60) | straight down | world +x | world -y | 1249.57 | (399.5, 299.5) |
| `pallet` | (0, 1.50, 4.00) | straight down | world +x | world -y | 1434.21 | (319.5, 239.5) |
| `buffer` | (-1.45, 0, 3.00) | straight down | world -y | world -x | 945.00 | (319.5, 239.5) |

The intrinsics are exact (square pixels, no distortion). The mounts are not: in every order each camera hangs up
to 30 mm (per axis) from its drawn position and points up to 1.5 degrees off. The arm is in the picture when it
is in the way.

## Survey marks

Twelve marks are painted where the drawing says, exactly: a white disc of 25 mm radius with a black centre dot of
8 mm radius, flat on its surface.

| Camera that sees them | Surface | Mark centres (x, y, z) |
|---|---|---|
| `infeed` | conveyor | (1.05, -0.33, 0.60), (1.05, 0.33, 0.60), (1.85, -0.33, 0.60), (1.85, 0.33, 0.60) |
| `pallet` | floor | (-0.75, 0.97, 0), (0.75, 0.97, 0), (-0.75, 2.03, 0), (0.75, 2.03, 0) |
| `buffer` | table | (-1.93, -0.63, 0.60), (-1.93, 0.63, 0.60), (-0.97, -0.63, 0.60), (-0.97, 0.63, 0.60) |

A carton in the pick station or on the table may cover a mark. On top of the tool, on the tool axis, 0.06 m above
the wrist pivot's height (0.61 m above the pad's face with the cup extended), is a magenta disc of 40 mm radius; a
yellow dot of 15 mm radius sits on it 0.07 m from the axis along the tool's x axis.

## Cartons and marks

A carton's frame sits at its centre: x along its length, y along its width, z up. Its centre of gravity is offset
in x and y only (the load inside sits off centre). The top face carries a strip of sealing tape, 48 mm wide,
along the middle of its length, and the marks in the two bands beside the tape. `marks/` shows the six marks; on
a carton they are printed 38 to 85 mm on a side, in any of the four turns, in inks from black to dark grey, brown
or blue, sometimes faint or worn in patches.

## Numbers of the rules

| Rule | Number |
|---|---|
| Shock limit | 60 m/s^2 over a 20 ms window; 20 m/s^2 for fragile |
| Stacking load | printed kg; 0.5 kg for "do not stack"; 80 kg without a mark; 0.5 s low-pass |
| Tilt test | 10 degrees toward +x, -x, +y, -y; 1 s up, 2 s held, 1 s down; 2 s of settling before and after |
| Shipped | upright within 3 degrees; at most 5 mm beyond the real pallet's edge; top at most 1.00 m above the deck; moved at most 30 mm and turned at most 5 degrees in the test; never damaged |
| Friction | carton on carton and on the deck 0.5; on the conveyor 0.5; pad on carton 0.9 |
| Step allowance | 400 control steps per carton of the order, plus 400 |
