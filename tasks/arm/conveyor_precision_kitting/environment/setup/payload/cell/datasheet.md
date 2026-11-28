# Kitting station KS-7: data sheet

KS-7 kits six machined parts per kit for the next assembly step. A Franka Panda takes the parts off an infeed belt,
rights the ones that lie upside down in a turnover station, and seats each in its own keyed pocket of a precision kit
tray. Everything below is the station's drawing and the integrator's hand-over survey: nominal values, and for
whatever the survey found to vary from day to day, the range it varies in. `cell_data.py` (beside this file) holds
the same numbers as Python. Section 10 lists everything that varies; nothing here says which value a given shift
has: what matters on the day has to be measured on the day.

Frames and units: world frame, metres, radians, seconds, newtons. The table top is the plane z = 0.80. The Panda's
base stands at (-0.56, 0, 0.80) facing +x. Yaw is measured about +z from +x. Quaternions are (w, x, y, z).

## 1. A shift

- A shift has one empty kit tray in the kit station and **8 parts** that the feeder puts on the belt one at a time:
  one part of each of the six variants plus two spares of random variants, in random order.
- The first part is released 20 to 26 s into the shift, each next one 44 to 54 s after the one before it.
- The shift ends 60 s after the last part has left the pick zone (or earlier on a safety stop, section 9). Then the
  kit goes to the downstream inspection (section 8).
- Each shift has its own belt speed and drift, command latency, arm response, lamp brightness and colour, part
  shades, part and tray surface condition, kit placement, part poses and real camera mounts (section 10).

## 2. The robot and its controller

**Action** (a list of 7 numbers): `[dx, dy, dz, dyaw, grip, cams, turn]`

| field | meaning |
|---|---|
| dx, dy, dz | displacement of the grip point (between the finger pads), world frame, m. The vector is clipped to 0.03 m. |
| dyaw | rotation of the hand about the world vertical, rad, clipped to ±0.15. The hand always points straight down. |
| grip | -1 = open (80 mm between the pads) … +1 = closed, linear in between. Each finger pushes with at most 20 N. |
| cams | which images the **next** observation carries, a bit mask: 1 overview, 2 wrist, 4 inspection camera (0 to 7). |
| turn | > 0.5 asks the turnover station to cycle (section 6). |

**Command latency.** The motion part (dx, dy, dz, dyaw) and the grip of an action are executed **L control steps**
after the observation they answer: the action you return for observation `t` is executed during step t+L → t+L+1.
The hand-over survey measured **L of 2 or 3** on different days (the cell controller's command buffering); it
is constant within a shift and not reported. During the first L steps of a shift the robot holds. `cams` and `turn`
act at once.

**Tracking law.** When a motion command is executed, the controller sets its target to *the grip point's position at
that moment* plus (dx, dy, dz), and its target yaw to the hand's yaw at that moment plus dyaw, both clipped to the
workspace (x ∈ [-0.38, 0.30], y ∈ [-0.50, 0.42], z ∈ [0.804, 1.20]). It then tracks that target with an operational-
space law for one control step: a task-space spring and damper (nominal stiffness 400 s⁻² in position and 200 s⁻² in
orientation, nominally critically damped), gravity compensated, with a weak null-space pull to a fixed arm posture.
There is no integral action and the target is reset to the current position at every executed command, so a zero
command holds still only as far as nothing pushes the hand; a carried load or a contact makes the hand drift until
the command compensates. The survey found the tracking to vary from day to day: stiffness within ±25 % of nominal,
damping ratio 0.75 to 1.10, gravity compensation within ±3 % (a slow sag or lift of the hand). In free space a
constant command of d per step moves the hand somewhere around 0.5–0.75·d per step after a few steps, depending on
the day.

**Control step.** 0.1 s of cell time, plus 0.05 s for every image the step's `cams` asks for (the cell waits for the
cameras; the belt and everything else keep moving).

**Hand yaw range.** The hand turns through about 360° in all before joint 7 reaches a limit, and where that range
sits depends on where the hand is: roughly 138° + (the bearing of the hand from the robot base, atan2(y, x + 0.56))
± 180° (measured on the cell; stay ~15° inside). The yaw reported in the observation is wrapped to (−π, π]; keep
track of the turns yourself. The home yaw is 90° (the fingers then close along world ±y). Commands past a limit are
not executed beyond it.

**Gripper.** The closing direction at hand yaw ψ is (cos ψ, sin ψ, 0). The finger pads are 16 mm wide (along the
part's long side when you grasp across its width) and reach 4.4 mm below the grip point; the fingers' metal reaches
9.3 mm below it. The grip point sits 97 mm below the wrist camera's lens plane.

## 3. The infeed belt

- The belt's top surface is the plane z = 0.84, 160 mm wide, centred on x = 0.10, running towards −y. Side rails
  20 mm high stand at x = 0.10 ± 0.088 (inner faces).
- The feeder puts each part on the belt at y = +0.62 (under its hood), lying flat, its centre within ±30 mm of the
  belt's centre line, at a uniformly random yaw, **upside down with probability 0.5**.
- **Pick zone: y from +0.40 to −0.40.** A part that is still on the belt past y = −0.40 drops into the reject chute
  and is lost. The robot cannot reach the belt beyond the pick zone's upstream end.
- **Belt speed:** each shift's speed is between 30 and 55 mm/s at its start. Within the shift it varies smoothly by
  up to ±8 % (a periodic variation with a period of 40 to 80 s) and, on top of that, drifts by up to ±10 % (either
  way) over the first 400 s and then stays there. The belt carries parts without slip.

## 4. The parts

Six variants, all machined aluminium plates **20 mm thick** with an anodised colour, lying flat. The outline is cut
through the full thickness; it is drawn here in the part frame: origin at the centre of the outline's bounding box,
x along the length, y across, z up from the bottom face (the part frame's origin is 10 mm above the bottom face).
Every outline has its key feature on the +x end.

| variant | length × width (mm) | key | colour (nominal linear RGB) | mass |
|---|---|---|---|---|
| A1 | 44 × 24 | corner notch 10 (x) × 8 (y) at the (+x, +y) corner | blue (0.20, 0.36, 0.78) | 45 g |
| A2 | 52 × 24 | corner notch 10 × 8 at the (+x, +y) corner | blue (0.22, 0.37, 0.76) | 52 g |
| B1 | 40 × 28 | slot 10 wide, 8 deep, centred in the +x end | green (0.24, 0.58, 0.30) | 44 g |
| B2 | 40 × 28 | corner notch 10 × 8 at the (+x, +y) corner | green (0.26, 0.60, 0.28) | 46 g |
| C1 | 38 × 30 | corner notch 9 × 8 at the (+x, +y) corner | orange (0.86, 0.47, 0.15) | 48 g |
| C2 | 46 × 26 | both +x corners notched 8 (x) × 6 (y): a narrower tab | orange (0.84, 0.49, 0.17) | 45 g |

- Two variants of a colour differ only by size or key; their colours differ by less than a part's own shade
  tolerance (section 10), so colour alone cannot tell them apart.
- **Datum mark:** a dark dot, 7 mm across, on the **top face only**, centred on the x axis 8 mm in from the −x end.
  A part lying upside down shows no mark (and, seen from above, the mirror image of its outline).
- A part is "upside down" when its z axis points down. A part can only be seated right side up; the A, B2 and C1
  outlines are not mirror symmetric, so an upside-down one does not even fit its pocket; B1 and C2 would fit
  geometrically but do not count (section 8).
- Friction of a part on the kit 0.25–0.55 (varies by shift, section 10), on the nests ≈ 0.5, on the table ≈ 0.8, on the belt ≈ 0.9, on the finger pads ≈ 1.5.

## 5. The kit tray

- A plate 250 × 164 × 10 mm (its top face, z = 0.81, is the floor of every pocket) with six walled pockets and three
  fiducial dots; the kit frame's x axis runs along the plate's 250 mm side. Its drawing pose (the kit frame in the
  world) is x = −0.155, y = −0.27, yaw +90° (the long side along the world y axis); each shift the operator sets the
  tray within ±12 mm (x and y) and ±5° (yaw) of it.
- **Pockets** (pocket frame = the frame of its part when seated; centre and yaw in the kit frame):

  | pocket | variant | centre (mm) | yaw |
  |---|---|---|---|
  | 0 | A1 | (−75, +40) | 0 |
  | 1 | B2 | (0, +40) | 0 |
  | 2 | C1 | (+75, +40) | +90° |
  | 3 | A2 | (−75, −40) | 180° |
  | 4 | B1 | (0, −40) | 0 |
  | 5 | C2 | (+75, −40) | 180° |

- A pocket is the part's outline grown by **0.6 mm per side** (1.2 mm diametral clearance): walls 4 mm thick and
  **5 mm high** around the rectangle, plus **key bosses** filling the key feature's region less 0.9 mm per side,
  **2.5 mm high**, square-edged. The walls' inner top edges have a **1.0 mm × 45° chamfer**. A part the wrong way
  round (or upside down, for A1, A2, B2, C1) stops on a key boss, 2.5 mm short of the floor.
- **Fiducials:** dark dots 12 mm across on the plate top, in the kit frame at (−110, +68), (+110, +68) and
  (−110, −68) mm.
- **The trays are a pool**, machined and worn alike but not identical: on any one tray each pocket sits within
  ±1.2 mm (per axis, kit frame) of its drawing position relative to that tray's fiducials. A shift uses one tray.
- **Force limit:** a part pressed against the kit with more than **12 N** (contact force averaged over a control step)
  for 2 consecutive steps is damaged and is scrapped at inspection, seated or not.

## 6. The turnover station

- A base at (x = −0.12, y = +0.28) with two nests side by side: the **input nest** centred at (−0.12, +0.326) and the
  **output nest** at (−0.12, +0.234). Each nest is a recess 64 (x) × 52 (y) mm, 4 mm deep, with a 1.5 mm chamfer;
  its floor is at z = 0.81.
- A cycle clamps the part lying in the input nest, turns it over by 180° about the x axis through (y = +0.28,
  z = 0.82) and lays it on the output nest: a part at (x, y) with yaw θ in the input nest comes out at (x, 0.56 − y)
  with yaw −θ, the other way up. The part lies on the output nest 1.8 s (cell time) after the request; the station
  is busy for 2.4 s in all.
- The station starts a cycle when asked (`turn`) only if it is idle, exactly one part lies at rest in the input nest
  and none in the output nest, and the hand is outside its guard zone (more than 0.12 m horizontally from the
  station's centre, or more than 0.10 m above the nest floors). Otherwise the request is ignored.
- The observation reports `station_busy` (1 while cycling) and `station_cycles` (cycles completed this shift).

## 7. Sensors

**Observation** (a JSON object): `t` (control step index), `time` (cell time, s), `q`, `dq` (the 7 arm joints),
`ee_pos` (grip point, world), `ee_quat`, `ee_yaw` (hand yaw, rad, wrapped to (−π, π]), `gripper` (opening between
the pads, m), `wrench` (force N and torque N·m at the wrist, world axes), `station_busy`, `station_cycles`, and the
three images `overview`, `wrist`, `inspect`, each `{"dtype": "uint8", "shape": [H, W, 3], "b64": <raw row-major RGB
bytes, base64>}`; an image is empty (shape [0, 0, 3]) unless the previous action asked for it. The first observation
of a shift carries all three.

**Wrist force/torque sensor.** It reads the load the gripper receives from the world, in world axes: pressing a part
down onto something reads **+z**; a carried part reads its weight as −z. It is zeroed with the empty gripper at the
start of each shift, then has a per-shift bias (up to ±0.5 N per force axis, ±0.04 N·m per torque axis), a slow drift
(up to ±0.25 N and ±0.01 N·m per minute) and white noise (σ = 0.2 N, 0.015 N·m) on every reading. Inertial loads of
the gripper's own motion appear as well.

**Cameras.** Pinhole, square pixels, principal point at the image centre; the vertical field of view `fovy` is
given, so the focal length in pixels is f = (H/2)/tan(fovy/2). Camera axes as in OpenGL: x right, y up in the image,
the camera looks along its −z. A pixel (u, v) (column, row from the top-left, centre at u + 0.5, v + 0.5) sees the
ray `R · [(u + 0.5 − W/2)/f, −(v + 0.5 − H/2)/f, −1]` from the camera position, R's columns being the camera axes in
the world.

| camera | image W × H | fovy | where |
|---|---|---|---|
| overview | 160 × 120 | 58° | fixed above the work area at (0.02, 0, 1.75), looking at (−0.02, 0, 0.80) (image up = world −x) |
| wrist | 160 × 120 | 75° | on the hand. In the grip-point frame (x = closing direction, z = straight down, y = z × x), the camera is at (0, 0.05, −0.097) with axes x_cam = −x, y_cam = +y, z_cam = −z (it looks straight down). The finger tips cover the bottom ~30 % of the image. |
| inspection | 96 × 96 | 40° | fixed on the table at (−0.10, 0.04, 0.812), looking straight up; image right = world +y, image up = world +x. Its ring light lights what is above it; hold a part 0.08–0.12 m above it to see its bottom face. |

Images are rendered without shadows or anti-aliasing. The lamps' brightness varies by shift (±30 %), and so does
their colour (each of red, green and blue by up to ±6 %).

## 8. Kit inspection (what counts)

At the end of the shift each pocket is inspected. A pocket counts as **seated** when a part of its own variant lies
in it right side up with its bottom face within 0.6 mm of the pocket floor, its centre within 1.2 mm of the pocket's
centre, tilted by less than 2°, its yaw within 3° of the pocket's (so its key is in the key boss), at rest, not
touched by the gripper, and not damaged (section 5). Anything else in a pocket area reads **unseated**, a seated but
damaged part reads **damaged**, and an empty pocket **empty**. Only seated pockets count. A part lying on its
chamfer or on a key boss can look placed from above.

## 9. Safety stops

The shift ends at once (keeping what is seated at that moment) when the simulation becomes unstable: any arm joint
faster than 8 rad/s, a part faster than 6 m/s, or a non-finite state.

## 10. Tolerances: the station on the day

Everything above is the drawing. Each shift the real station differs from it within these bounds (uniformly drawn,
independently):
- **command latency** 2 or 3 control steps; **arm tracking** stiffness ±25 %, damping ratio 0.75–1.10, gravity
  compensation ±3 % (section 2);
- **belt** starting speed 30–55 mm/s, its periodic variation and drift (section 3);
- kit tray pose (±12 mm, ±5°) and each pocket's position on the tray (section 5);
- **surface condition** of the parts and the tray: coefficient of friction between 0.25 and 0.55 (the same for all
  of a shift's parts);
- lamp brightness ±30 % and colour ±6 % per channel; each part's shade ±0.02 per RGB component;
- F/T bias and drift (section 7);
- **camera mounts**: overview ±10 mm per axis and ±1.5° about a random axis, wrist camera ±1.0 mm and ±0.3°,
  inspection camera ±0.8 mm and ±0.3°.

None of these is reported; the controller has to measure what it needs on the day. The parts, pockets, nests and
the belt are made to the drawing.

## 11. The practice cell

The commissioning window gives you a few practice shifts on the real station (`~/practice/`, see the instruction).
The practice service reports, at the end of a practice shift, what the kit inspection found per pocket and how many
parts were lost; nothing else about the cell's state. The practice station keeps moving like the graded one: from
the moment it sends an observation, every whole 0.1 s until the next step request arrives runs one hold step first
(no motion, the gripper as last commanded, no images, no turnover request; the step's `info` reports `hold_steps`).
A step request later than 30 s ends the shift, and so does 90 s of wall-clock time since its start (`info["ended"]`
gives the reason); the kit keeps what is seated at that moment. Keep the machine reasonably free while a practice
shift runs: the station's own simulation shares its CPUs.
