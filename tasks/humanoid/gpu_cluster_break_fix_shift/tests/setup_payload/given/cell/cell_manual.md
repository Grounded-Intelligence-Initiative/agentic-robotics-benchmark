# The data-hall cell: the pod, one Unitree G1 humanoid, the rules

This is the integrator's description of the robot cell your program drives: the frames the site's drawings use, the
robot and its controllers, the belt tools, and the mechanisms of the equipment as the vendor's service guide and the
site's documents describe them, in the numbers the robot works to. Everything stated here as exact is exact;
everything about the night's faults (which ticket, which node, port or slot, which end face is dirty, which part has
failed, what the services will answer) must be found out in each shift, and the pod's own installation offsets are
within the tolerances stated below. The cell never reports the pose or the state of anything but the robot itself:
the site's services (`cell.noc`, `~/given/site/services_api.md`) report what a technician's terminal reports. There is
no practice cell: section 12 lists what you have to prepare the program with.

## 1. The pod and its frames

**Hall frame:** metres, radians, quaternions `w x y z`; **x east, y north, z up**, the floor at z = 0; the origin at the
pod's west end (the west side of racks A01 and B01) on the middle line of the cold aisle. The site's DCIM documents
(`~/given/site/dcim/`: floor plan, rack elevations, cable table, PDU outlet map) and the vendor's drawings
(`~/given/vendor/kx8h_service_guide.md`) give every dimension; this section gives the frames they are drawn in.

* **Two rows of five racks** (600 x 1200 mm, 48U) round a contained cold aisle (y -0.6 .. +0.6): row A's fronts face
  south at y = +0.6 (the racks extend north to y = 1.8), row B's fronts face north at y = -0.6 (they extend south to
  y = -1.8); racks 01 .. 05 from west to east, rack k's middle at x = 0.6 k - 0.3. The hot aisles (1.2 m wide) lie
  behind the rows. Racks 01 are the network racks (four 1U leaf switches each at U16, U18, U20, U22, ports to the rear);
  racks 02 .. 05 hold one 8U KX-8H GPU node each at U8 .. U15 (0.411 - 0.767 m above the floor).
* **A rack's frame:** origin at the middle of its front edge on the floor, x to the right of a person facing its front,
  y into the rack (front to rear), z up. Every vendor and DCIM position of a rack's equipment is in this frame.
* **Installation tolerance (each pod its own):** the drawing is the same for every pod; each pod is built within
  tolerance: a row's racks are bolted together and stand within 15 mm (x), 10 mm (y) and 0.3 deg (yaw, about the row's
  middle) of the drawing as one, and each rack within a further 2 mm, 3 mm and 0.1 deg. Measure what you work on (the
  cameras, section 7): a program that works to the drawing alone misses by up to about 2 cm.
* **The cold aisle's doors:** a sliding double door at each end (x = 0 and x = 3.0), automatic: a door opens while
  someone stands within its sensor's zone in front of it and closes a few seconds after the zone is clear.
* **The rear doors:** one per rack (hinged on the rack's x = -0.3 side seen from the front, i.e. on the left seen from
  the rear), latched shut; opened by its handle (section 9) it swings out into the hot aisle to its stop at 1.95 rad
  (112 deg). An open door's leaf stands across a good part of the hot aisle.
* **The service area west of the pod:** the **parts cart** (its middle at (-0.95, -1.55), 0.60 x 0.90 m, its top at 0.80
  m; the spares on it as the site handbook describes: each the way it goes into its slot, its front to the cart's west
  side; the power supply and the transceivers on a grey foam riser 0.18 m tall along the west edge, the fan modules
  upright on the top, the GPU module by the north edge) and the **RMA tote** (a red bin 0.56 x 0.40 m, 0.30 m deep, on
  a steel stand: its bottom 0.45 m and its rim 0.75 m above the floor, its middle at (-0.95, -2.26)). Each pod's cart
  holds each spare within 12 mm of its drawn place.
* **The robot** starts each shift by the cart, west of the cold aisle (x -1.35 .. -0.75, y -0.55 .. +0.35), facing any
  way.

## 2. What varies from shift to shift

The tickets and their root causes (the site handbook lists the kinds; their mix follows the fleet's statistics), the
nodes, GPUs, slots, ports and supplies they concern, which end faces carry dirt (and how much, where), the services'
readings that follow from them, the ticket that comes in during the shift and when, the distractors a previous shift
left (a locator LED, a degraded part nobody ticketed), the pod's installation offsets (section 1), where on the cart
each spare lies (within 12 mm), the serial numbers, the robot's start pose, and the sensors' noise.

## 3. The robot

One Unitree G1 humanoid (29 DoF with Dex3-1 hands). The model is MuJoCo Menagerie's `unitree_g1/g1_with_hands.xml`
(copy in `~/given/robot/unitree_g1/`, BSD-3-Clause) with the cell's modifications:

* **Locomotion controller.** The pelvis rides four base joints: `base_x`, `base_y` (hall x / y), `base_yaw` and
  `base_z`, driven by force-limited servos (x / y 300 N, yaw 150 N m) that the robot's own locomotion controller
  commands; the legs are posed from the stand height and do not touch the floor (no balance falls are simulated). A
  robot obstructed by equipment stops; its controller does not wind up beyond 5 cm / 0.1 rad, but while it stays
  obstructed the base keeps pushing against the obstacle with up to its servo limit: held for 0.2 s, that is a safety
  violation (section 10). Cancel the goal (or stop the velocity) when the base stops short.
* **Legs in the camera images.** The cameras draw the legs stepping while the base travels or turns (drawn only).
* **Stand height** (pelvis height above the floor) 0.48 - 0.80 m, nominal 0.793 m; lower means a squat (the knees go
  forward).
* Every robot body is gravity-compensated; payloads are not.
* Joints: waist yaw / roll / pitch (pitch at most 0.52 rad), 7 per arm (shoulder pitch, roll, yaw, elbow, wrist roll,
  pitch, yaw), 7 per hand; ranges as in the model (`describe()` lists them).
* **Collision:** body links are capsules (Menagerie's `g1_mjlab.xml` colliders); the robot collides with the equipment,
  the doors, the cart, the tote and the loose parts, never with itself or the floor. **The hands (the
  `wrist_yaw_link` and the finger links) carry no contact**: grasping is the cell's grip rule (section 4). What a hand
  holds collides with everything but that hand, the robot's body included.
* **Grasp site** of each hand, `<side>_grasp`: in the `<side>_wrist_yaw_link` frame at (0.150, -0.046, 0) for the left
  hand and (0.150, +0.046, 0) for the right hand, with the link's axes: **x = approach** (along the fingers), **y = pinch
  axis**, z = x cross y.
* **Cameras:** `head`, an RGB-D camera on `torso_link` at (0.0576, 0.0175, 0.4299), its optical axis along the link's +x
  pitched down by 0.8308 rad (it moves with the waist); `wrist_left` and `wrist_right`, a close-range RGB-D camera on each
  `<side>_wrist_yaw_link` looking along the fingers (mounts in `describe()`).
* **The tool belt:** three holsters on the pelvis (pelvis frame: x forward, y left, z up): the long-reach torque driver
  on the right hip (0.045, -0.170, 0.020), the fibre probe (0.070, 0.170, 0.020) and the one-click cleaner (-0.035,
  0.170, 0.020) on the left hip, each tool upright with its grip up (section 8).

## 4. Grip rule (hands)

Closing a hand (`"close"`) engages a grip on an object when, at that instant, the hand's grasp site lies at one of the
object's grasp features:

* **bars and cylinders**: a part's handle (a fan module's bar handle, a power supply's handle, a transceiver's pull
  tab, an MPO connector's body, a power cord's plug body, a GPU module's lift bar), a tool's grip, a tray lever, a cover's
  pull, a rear door's handle: the grasp site within 15 mm of the surface (at most the radius + 15 mm from the axis),
  between the feature's ends, the pinch axis within 35 deg of perpendicular to the axis. The vendor's guides draw where
  each handle sits on its part.

A loose object (a part, a tool) is then held rigidly at the relative pose of that instant until the hand opens or the
grip slips; a fixture's moving part (a lever, a cover's pull, a door's handle) is held at that point only (it turns
about its hinge in the hand). A closed hand that found nothing holds nothing (open and close again). Grip limits: an
object held by one hand slips when the hand's load exceeds **30 N** or its torque about the grasp site exceeds **5 N m**
(**12 N m** for a bar or cylinder feature: a power grasp); held by both hands, when its net load per hand exceeds 30 N;
a fixture's part held at a point slips beyond **60 N**; an excess must last 50 ms. Mind the heavy parts: a GPU module
weighs 1.8 kg (17.7 N), a power supply 1.1 kg: accelerations and a part that meets something use up the rest of the
30 N quickly. The hand's tactile reading (`get_state()`: `tactile_N`) is its share of the held object's load.

## 5. Control (each 10 ms tick)

* Base: `nav_goal` [x, y, yaw] in the hall (the navigation stack, section 6); or `base_velocity` [vx, vy, wz] in the
  robot's heading frame (x forward, y left; limits 0.5, 0.3 m/s, 0.8 rad/s; accelerations 0.8, 0.8 m/s^2, 1.5 rad/s^2),
  held until replaced; or `base_goal` [x, y, yaw] in the hall: a straight-line approach (proportional, gain 1.2 /s,
  within the same limits, no obstacle avoidance) that stops within 5 mm / 0.01 rad. A goal that keeps moving is followed
  with a lag of its speed over the gain (0.1 m/s: about 8 cm). `stand_height` moves at 0.15 m/s.
* Waist: joint targets [yaw, roll, pitch], at most 0.8 rad/s.
* Arms: joint targets (`q`, 7 values), or a grasp-site pose target (`pose` [x, y, z, qw, qx, qy, qz]) or position target
  (`pos` [x, y, z]: the wrist's orientation left free) in the robot's **base frame** (origin on the floor under the
  pelvis, x along the heading, z up; the default) or in the **hall frame** (`"room"`). A target is solved by the cell's
  damped-least-squares IK over the arm's 7 joints (base, waist and legs as they are): a target within 0.12 m / 0.6 rad of
  the arm's previous target keeps the arm's configuration, a farther one gets a seeded global solve. `path: "line"`
  (default) moves the grasp site on a straight line from the previous target with a minimum-jerk timing (about 0.25 m/s
  and 1 rad/s on average, peak accelerations at most 1 m/s^2 and 4 rad/s^2); `path: "joint"` moves in joint space to the
  target's IK solution. Joint targets move at most 1.5 rad/s with 6 rad/s^2 per joint. An unreachable target leaves the
  arm where the IK gets closest (`get_state()`: `ik_residual` [position m, rotation rad]). A target in the base frame
  travels with the base; one in the hall frame stays where it is while the base or the waist moves.
* Hands: `"open"` or `"close"` (closing applies the grip rule once).

## 6. Navigation

The robot runs a navigation stack on its map of the hall (a G1 on a site runs one on its lidar map): `nav_goal`
[x, y, yaw] plans a path for the base on a 5 cm occupancy grid and walks it; `get_state()["robot"]["nav"]` reports
`status` (`idle`, `moving`, `arrived`, `blocked`: no progress for a while, `no_path`), the goal and the distance left.

* The grid holds every collider the robot's body can meet between 3 cm and 1.7 m above the floor: the racks and their
  equipment, the walls, the cart, the tote (rasterised once), and the moving fixtures as they stand when the plan is
  made (an extended tray and its levers, an open rear door). The cold aisle's doors are automatic: a plan treats them
  as standing fully open (their leaves beside the doorway), so it never waits for one and keeps clear of the open
  leaves' edges.
* Obstacles are inflated by the robot's footprint: the base's centre never comes closer than **0.24 m** to one, and keeps
  clear up to 0.45 m where it can (a cost). Near the path's start and its goal the 0.24 m rule is relaxed (a goal at a
  rack face is allowed): the last and the first few centimetres are the program's responsibility.
* The path (A* on the grid, shortcut to straight segments that keep the clearance the search found) is followed with a
  look-ahead point; the robot faces along the path, and walks sideways (its depth across the passage) where a segment's
  clearance is under 0.40 m, as next to an extended tray. At the end it turns to the goal's yaw.
* **The arms and whatever the hands hold are not in the plan**: tuck the arms before walking. With the arms at their
  default pose (`describe()`, the Menagerie keyframe) the wrists stand out to about 0.29 m from the base's centre, more
  than the footprint: bring them in. A part carried in front of the body is the program's to keep clear.
* A `base_goal` or `base_velocity` command cancels a running nav goal.

## 7. Sensors

* `get_state()`: the localised base pose [x, y, yaw] (independent noise of about 3 mm and 0.2 deg per reading), the
  stand height, whether the base is moving, the navigation's state, the motor state of the waist, the arms and the
  hands (`q`, `dq`, `tau_est`, below), per hand `closed`, `tactile_N`, the grasp site's pose in the robot's base frame
  (exact kinematics) and the IK residual of a pose target.
* **Motor state** (`q`, `dq`, `tau_est`: one {joint name: value} map each), as the G1's low-level state reports it (the
  body's `LowState` and the Dex3-1 hands' `HandState`), for the waist's 3, the arms' 2 x 7 and the hands' 2 x 7 motors
  (`describe()`: `robot.motor_state` lists them with their torque limits):

  | field | unit | what it is | accuracy |
  |---|---|---|---|
  | `q` | rad | the joint angle, from the joint's encoder | exact |
  | `dq` | rad/s | the joint velocity | reading-to-reading noise of about 0.01 rad/s (1 sigma), quantised to 0.001 rad/s |
  | `tau_est` | N m | the joint torque estimated from the motor's current | a calibration offset of up to a few percent of the joint's torque limit (1.5 %, 1 sigma) and a scale error of up to 5 %, both constant through a shift (read them at rest), plus reading-to-reading noise of about 0.5 % of the limit, quantised to 0.01 N m |

  `tau_est` is what the drive delivers: the torque holding the robot's own links up, the payload, the arm's
  acceleration, and the load of anything the arm or the torso presses on or is pressed by. A hand's fingers show their
  drives' own effort only: what a hand holds shows in its `tactile_N` and in the arm's and the waist's torques.
* **Cameras** (`get_camera`): `head`, 640 x 480, vertical field of view 58 deg, depth 0.28 - 6 m; `wrist_left` and
  `wrist_right`, 640 x 480, 65 deg, depth 0.07 - 0.60 m (close-range: a port, a screw head, a handle). Every frame
  carries the camera's pose in the hall at capture time (composed from the localised base pose and the exact
  kinematics: the pose carries the localisation's error of that moment). Pinhole, no distortion: fx = fy = (H / 2) /
  tan(fovy / 2), principal point at the image centre. Depth is along the optical axis, registered to the colour pixels,
  1 mm quantisation, noise of 1.5 mm + 2.5 mm per m^2 of z^2 (1 sigma), invalid (0) outside the range, on depth
  discontinuities (a jump of more than 3 % to a neighbouring pixel) and on about 2 % of the pixels at random. Colour is
  rendered physically based (MuJoCo's Filament renderer): the hall's lights, the equipment's LEDs as they are lit, the
  labels and silk-screens as the vendor's guide draws them; each camera has its own white balance (up to 6 % per
  channel) and exposure (up to 8 %), constant through a shift but different every shift, and pixel noise (about 2 of
  255, 1 sigma). **At most 2000 frames per shift** over all cameras.
* **The fibre probe's picture** (`tool("scope")`): the end face in front of the probe's tip as the probe's camera sees
  it (section 8); the vendor's OSFP / MPO guide shows what clean, dirty, scratched and misaligned pictures look like.

## 8. The belt tools

Each tool rides upright in its holster (section 3), its grip a cylinder (section 4). Drawing one: grip it and lift it
out along the holster's axis. Stowing one: bring it upright over its holster's mouth and let go within 15 mm of the
mouth: it drops into the holster. A tool let go anywhere else falls.

* **The long-reach torque driver** (0.45 kg): a T20 bit at the end of a long extension, its tip 0.19 m below the grip
  along the tool's axis. `tool("driver", mode, torque, seconds)` runs it (`"loosen"` or `"tighten"`, the clutch setting
  in N m, how long). The bit drives a screw only when, at the start of the run and while it runs, its tip sits in the
  screw head's socket: within 1.0 mm of the screw's axis, within 6 deg of it, and axially from 6.5 mm into the socket to
  4.5 mm above the head's top (the bit is spring-loaded: the head may rise or sink by its whole travel without the tool
  being moved). A screw has 6 turns of thread (0.5 mm each) and turns at 5 rev/s; tightening stops when the clutch
  slips at its setting. `tool("driver", action="status")` reports the last run as the driver's display shows it:
  `running`, `clutch` (tightened to the setting), `out` (loosened all the way: the screw is free in its captive washer),
  `no_engagement` (the bit was not in a head). A module's screws are tightened in two passes (the vendor's order and
  torques): a screw tightened past 0.5 N m while another screw of the same module is still more than 0.2 turns out tilts
  the module on its socket and bends the socket's pins (damage, charged).
* **The fibre probe** (0.30 kg), its tip 0.05 m below the grip. `tool("scope")` takes one picture of the end face its tip
  is at: an MPO connector's face (the connector out of its port, held in a hand) or a transceiver's receptacle (the
  connector out). A face is in view when the tip is within 4 mm of the face's centre line and between 2 mm inside and
  3 mm outside the face; the picture is **sharp** (the fibres' cores and claddings resolved, `sharp` in the capture
  record the NOC keeps) when the tip is within 0.5 mm of the face's centre line and the probe's axis within 3 deg of
  the face's normal. Each picture has a capture id (the evidence a ticket or an escalation quotes).
* **The one-click cleaner** (0.10 kg), its tip 0.045 m below the grip. Pushed straight onto a face (within 0.6 mm of its
  centre line and 4 deg of its normal) until its tip is 2 mm past the face, it clicks once: one cleaning of all twelve
  fibres (a particle goes with a probability of 0.8, a film halves, scratches and pits stay; a click now and then leaves
  a fresh speck in a cladding zone). Pull it back out before the next click.

## 9. The equipment's mechanisms

The vendor's service guide (`~/given/vendor/kx8h_service_guide.md`) and the transceiver / MPO guide describe the
procedures; this section gives what the cell holds the robot to.

* **The GPU tray** slides out of the node's upper 6U on rails. It is latched home until both of its blue release levers
  (one each side of the fan wall, hinged at their bottoms, their tops swinging out towards the aisle) are pulled out past
  their detent (0.55 rad); then the tray runs free on its rails. Pulled all the way out it stops at the rails' service
  position, 0.74 m out, and stays there (a bump does not push it back) until a lever's release tab is held again; pushed
  home with both levers folded back (at most 0.12 rad) it latches. A lever left alone stays in its detent while folded.
  (The site's SOP says when a tray may be pulled: the cell charges a tray pulled from a node with AC on, and two trays
  out at once.)
* **The tray's cover** is hinged at the tray's rear edge; lifted by its pull (front edge, a bar), it swings up and rests
  on its stay 20 deg past vertical (1.92 rad), where it stays; closed (at most 0.02 rad) it latches. Held by its pull it
  turns freely about its hinge in the hand (a point grip, 60 N).
* **A GPU module** sits on its socket on two guide pins. It comes off its pins only when all four of its captive screws
  are out (section 8, the driver); lifted by its lift bar it slides up its pins for its first 15 mm and is then free. A
  module brought back over a free socket within 1.5 mm and 2 deg of the socket's frame is taken by the pins and seats
  when let go. Each module weighs 1.8 kg.
* **Field-replaceable parts in slots** (fan modules, power supplies, transceivers, MPO connectors, power cords' plugs):
  a part in its slot is held by the slot. Latched, it stays at its seat. A hand that holds the part and pulls it out
  along the slot's axis harder than its release force (for 50 ms; and only while the part may be released: a
  transceiver whose MPO connector is plugged, a power supply whose cord is plugged, do not let go) unlatches it; the
  slot then guides it straight along its axis while the hand slides it out, until it leaves the slot's mouth and is free.
  A held part brought to a free slot of its kind with its frame within the slot's lateral and angular tolerance of the
  slot's frame at the mouth is taken by the slot and slides in; pushed home and further than its latch force it latches.
  One brought outside the tolerance (but within three times it) and pushed into the mouth jams: pushing a jammed part
  harder than its damage force breaks the slot (damage, charged).

  | part | mass kg | travel in the slot mm | lateral / angular tolerance | release N | latch N | damage N |
  |---|---|---|---|---|---|---|
  | fan module | 0.55 | 62 | 1.5 mm / 3 deg | 8 | 12 | 40 |
  | power supply | 1.10 | 190 | 1.5 mm / 3 deg | 8 | 15 | 45 |
  | power cord's plug (C19) | 0.12 | 12 | 1.2 mm / 4 deg | 10 | 20 | 60 |
  | node transceiver (OSFP, flat top) | 0.06 | 72 | 0.8 mm / 3 deg | 6 | 18 | 28 |
  | leaf transceiver (OSFP, finned) | 0.08 | 72 | 0.8 mm / 3 deg | 6 | 18 | 28 |
  | MPO-12 connector | 0.02 | 8 | 0.5 mm / 3 deg | 5 | 10 | 25 |

  A part's frame and its handle (or pull tab, or body) are drawn in the vendor's guides; a part's axis out of its slot is
  its frame's +y. A slot is free when nothing is in it (a power cord's plug needs its supply in the bay, an MPO
  connector its transceiver).
* **Cables:** a power cord's plug and an MPO connector out of their ports hang on their cables. A C19 cord and an MPO
  trunk are stiff over their last half metre: one let go stays about where it was let go (it neither drops nor swings).
  An MPO trunk leaves 0.60 m of slack beyond its drawn run at its last tie (the service loop): a connector reaches that
  far from where the cable table routes it, and no farther.
* **The rear doors:** each latched shut; its handle (a bar on the leaf's free edge) held and pulled, the door swings out
  about its hinge up to its stop at 1.95 rad (112 deg); a door pushed shut (at most 0.02 rad) latches. Held by its handle
  it turns about its hinge in the hand (a point grip, 60 N).
* **The parts cart and the RMA tote:** the cart's top and the foam riser carry what lies on them; the tote takes what is
  dropped into it from above its walls (a part counts as in the tote when it rests inside its walls).
* **Power and the training job:** the site's services model the rest: the PDUs' outlets, the supplies' redundancy and
  the power brake, the nodes' power states and boots, the fans' thermal effect, the job and its checkpoints, the fabric's
  links and their optical powers, DCGM's runs (`~/given/site/`: the handbook, the SOPs, the services' API).

## 10. Safety (a violation makes the shift score 0; the shift goes on)

* Any robot body other than the hands pressing on anything (equipment, a door, the cart, the tote, a part, the floor
  excepted) with more than **150 N** for **0.2 s** or longer: the robot's own servos can push that hard (the base 300 N,
  the arms' links with their joints' torques), so a contact that a move keeps loading crosses it quickly.
* A part or a tool let go while moving faster than **1.5 m/s** (thrown), or a loose object hit so that it moves faster
  than **4 m/s** (struck).

Equipment incidents (a crashed job, a node that lost power, a wrong outlet switched, a tray pulled with AC on, bent
socket pins, a damaged slot, a hot swap over 30 s, ...) are not safety violations: they are charged against the shift's
points as the instruction states.

## 11. Budgets per shift

7200 s of simulated time (a 2-hour window); the site's services take their own simulated time per call (a boot, a DCGM
run, a soak run while the robot waits; `services_api.md` lists each call's cost); 2000 camera frames over all cameras;
the program-time and CPU caps of the instruction. `step(n)` advances at most 500 ticks (5 s) per call.

## 12. Preparing the program (no practice cell)

Nothing in your session runs the cell. You prepare the program from: this manual and the API document; the site's
handbook, SOPs, services API and DCIM exports; the vendor's guides; `describe.json` (exactly what `describe()`
returns); the robot's model; and the **survey photos** (`~/given/cell/survey/`): frames of the robot's cameras taken in
two other pods of the same design on other nights (not among the graded shifts), each with its camera pose, as
`get_camera` delivers them (`dc_client.load_frame` reads them). The graded shifts then run your program as it is.
