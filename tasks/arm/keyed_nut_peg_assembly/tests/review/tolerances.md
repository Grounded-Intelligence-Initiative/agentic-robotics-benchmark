# The tolerances, the fit, and where each number comes from (maintainer notes, 2026-11-17)

Round 3 of the maintainer review. After round 2 the agent rebuilt the cell from its data sheet as a twin of its own
and developed there with the twin's ground truth: the drawing was the cell. A real cell is not its drawing. From this
round on every episode, graded or commissioning, is a real kit on the cell as it is that day: `keyed_env.sample_unit`
draws each value below from the episode's seed, uniformly within its range and independently, and the observations
never carry it. The data sheet (`environment/setup/payload/datasheet.md`, section 11, and `cell_data.TOL`) states
every range, so nothing is hidden; what a twin built from it can do is randomise. `tests/test_keyed_nut_peg_assembly_commissioning.py`
checks that the simulator's table is the data sheet's.

Sizing rule: every range is what real parts and sensors of this class have. Where a standard or a data sheet gives the
number, it is cited; where the number is an engineering estimate, the note says so and what it rests on.

## Every nut

| value | range | source |
|---|---|---|
| each of the 24 sector boundaries of the hole wall (the channels' walls) | +-1 degree | ISO 2768-1:1989, general tolerances for angular dimensions, classes f and m: +-1 degree where the shorter side is up to 10 mm (a channel wall is 5 to 15 mm tall). It equals +-0.22 mm at the hole radius, about the same standard's linear tolerance of +-0.2 mm for 6 to 30 mm. |
| each of the 8 inner layer boundaries (the channels' floors and ceilings) | +-0.2 mm | ISO 2768-1 class m, linear dimensions over 6 up to 30 mm: +-0.2 mm |
| hole flats from the axis | +-0.1 mm | ISO 2768-1 class m on the 25.6 mm hole (+-0.2 mm on the diameter) |
| groove floor from the axis | +-0.2 mm | ISO 2768-1 class m, 6 to 30 mm |
| the handle's direction against the wall's sector grid | +-0.5 degrees | ISO 2768-1 angular, classes f and m, shorter side over 10 up to 50 mm: +-0 degrees 30 minutes |
| colour, each RGB component | +-0.04 | estimate: the shade difference between paint or anodising batches (a few CIELAB units); it only has to stop a controller from keying on one exact colour value |
| sliding friction against a peg | 0.3 to 0.7 | aluminium on mild steel, clean and dry: 0.47 sliding, 0.61 static (The Engineering ToolBox, "Friction - Coefficients for Common Materials and Surfaces"); the lower end for handling residue and oil films (the same table gives 0.16 to 0.23 for greased or lightly oiled steel pairs). Before this round the nut's coefficient (0.6 to 1.0) never acted on a peg: MuJoCo takes the larger coefficient of a pair and the pegs had 1.0. The pegs now have 0.1, so a nut on a peg slides with the nut's own value. |
| mass | 0.05 to 0.15 kg | unchanged (the contribution's range) |

## Every peg

| value | range | source |
|---|---|---|
| shaft radius | +-0.05 mm | ISO 2768-1 class f on the shaft's diameter (+-0.1 mm for 6 to 30 mm); ground bar stock is tighter |
| pin reach from the shaft axis | +-0.2 mm | ISO 2768-1 class m, 6 to 30 mm |
| pin width and height | +-0.1 mm each | ISO 2768-1 class m, 0.5 to 6 mm |
| pin centre above the collar top | +-0.3 mm | ISO 2768-1 class m, over 30 up to 120 mm |

## The cell on the day

| value | range | source |
|---|---|---|
| finger pads: sliding friction against a nut | 0.6 to 1.2 | rubber on dry hard surfaces in The Engineering ToolBox's table: 0.6 to 0.85 (dry concrete), 0.5 to 0.8 sliding and 0.9 static (dry asphalt), 1.0 (dry tyre on dry road), 1.16 (rubber on rubber). The robosuite gripper model's 2.0 is above anything in that table; worn or dusty pads sit at the lower end. The pads' torsional coefficient is scaled in proportion (the model's 0.05 m at 2.0). |
| each arm joint's friction | the Panda's identified curve x 0.6 to 1.4 | curve: C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, "Dynamic Identification of the Franka Emika Panda Robot With Retrieval of Feasible Parameters Using Penalty-Based Optimization", IEEE RA-L 2019, supplementary material, Table I (phi1 = 0.546, 0.872, 0.641, 1.279, 0.839, 0.303, 0.565 N m; phi2, phi3 as in `keyed_env.JOINT_FRICTION`): the friction a torque-controlled Panda shows at its joints. The cell's tracking law is plain operational-space control with gravity and Coriolis compensation, as on that robot's torque interface, so the friction is not compensated. The +-40 % spread is an estimate for unit-to-unit differences and warm-up (joint friction falls by tens of percent as a gearbox warms: A. C. Bittencourt, S. Gunnarsson, "Static Friction in a Robot Joint: Modeling and Identification of Load and Temperature Effects", ASME J. Dyn. Syst. Meas. Control, 2012). With this controller's stiffness (2400 to 3700 N/m, 40 to 220 N m/rad) it costs a few tenths of a millimetre of steady-state error. |
| wrench zero: offset | +-0.5 N, +-0.05 N m per axis | unchanged (the contribution's values) |
| wrench zero: drift | up to +-0.3 N and +-0.012 N m per minute per axis | estimate, sized against a wrist sensor's data sheet: the Robotiq FT 300-S product sheet (February 2021) gives 0.1 N and 0.003 to 0.005 N m of signal noise and recommends 1 N and 0.01 to 0.02 N m as thresholds for contact detection. The drift over one takt (0.5 N, 0.02 N m) stays within those thresholds: the zero a real sensor of this class keeps between tares. |
| fixed cameras: position, orientation | +-2 mm per axis, +-0.2 degrees about each axis | estimate from what an extrinsic (eye-to-hand) calibration with a board leaves: errors of 1 to 3 mm and 0.1 to 0.5 degrees are the usual figures in the hand-eye calibration literature (R. Y. Tsai, R. K. Lenz, IEEE Trans. Robotics and Automation 1989; I. Enebuse et al., "A Comparative Review of Hand-Eye Calibration Techniques for Vision Guided Robots", IEEE Access 2021) |
| wrist camera on the hand: position, orientation | +-1.5 mm per axis, +-0.3 degrees about each axis | the same literature for eye-in-hand calibration |
| every camera: focal length, principal point | +-0.5 %, +-1 pixel per axis | estimate from what a checkerboard calibration (Z. Zhang, IEEE TPAMI 2000) leaves plus thermal drift of the lens: a few tenths of a percent on the focal length and about a pixel on the principal point at these image sizes (320 x 240 and 256 x 192) |
| lamps: brightness | +-15 % | estimate: lamp ageing and daylight through a shift |

What they add up to (also stated in the data sheet): a nut on the pin rests at 39.5 +- 0.35 mm, stalls at 24.5 and
9.5 +- 0.55 mm; riser middles are 30, 45 or 60 +- 2 degrees apart; a nut left free on the pin drops into a riser
within +-4.5 to +-7 degrees of the riser's real middle (measured, 18 nuts; +-4 to +-5.5 on the drawing's nut). With the
published calibration alone a point on the table is typically 4 mm off in a fixed camera (p90 8 mm, at worst 16 mm) and
2 mm off in the wrist camera from 0.25 m (p90 3.5 mm, at worst 6 mm).

## The fit (amendment 3)

Tolerances and a 100 s takt were not enough (below, and `commissioning.md`): the contribution's hole was 25.6 mm on a
22 mm shaft, 1.8 mm of radial clearance, and that play absorbed every tolerance and most of the cameras' error. A
keyed nut is a locating part. From amendment 3 on:

| what | value | source |
|---|---|---|
| the fit | shaft 24.6 mm in the 25.6 mm hole: 1 mm on the diameter, 0.5 mm of radial clearance on the drawing, 0.35 to 0.65 mm with the tolerances of the hole (+-0.1 mm) and the shaft (+-0.05 mm) | ISO 273 (clearance holes for bolts and screws): the fine series is the shank + 1 mm in this size range (23 mm for 22, 25 mm for 24), the medium series + 2 mm, the coarse series + 4 mm. The contribution's pair was between medium and coarse. A locating fit by a limits-and-fits standard would be tighter still (ISO 286 H11/c11 on 22 mm: 0.11 to 0.37 mm on the diameter); the fine clearance series is what parts made to general tolerances can hold. |
| lead-in | none: the shaft end and the hole's edges are square | the geometry as contributed (a cylinder and a prism); the data sheet says so |
| the shaft, not the hole, was changed | | the nut and its groove are untouched: the inner wall blocks, the pin's reach and the window in which a nut drops into a riser depend on the hole's radius, not on the shaft's (measured: +-4 to +-5.5 degrees on the drawing's nut with either shaft) |
| the hand's fingers | one mechanism: the two finger slides are coupled (an equality constraint, `keyed_env.FINGER_COUPLING`) | the real Franka hand has one drive for both fingers, which move symmetrically. The robosuite model drives each finger on its own, both capped at 20 N: a gripped part then has nothing holding it along the closing direction but the slides' damping, and a sideways force of 2 or 3 N walks nut and fingers through the hand (measured: 10 to 15 mm during a search on the shaft end). With 1.8 mm of clearance that did not show; with a locating fit it would make a search by touch impossible while the reference, which is given the nut's real pose, would not notice. |

The data sheet's section 12 states the fit, why it has to be done by touch, what holds the nut, the cell's compliance
(section 5: the tracking law's static stiffness, measured at seven places of the work area) and what the wrist reads
when the nut is on the shaft end, when the hole comes over the shaft, when the nut's rim meets the shaft's side and
when a tilted nut binds. Every number there is measured on this cell or follows from the drawing.

Clearances tried (the 16 development kits of amendment 2, in-process runs with the grader's cell and credit rule,
before amendment 3's candidates were drawn; fingers coupled):

| radial clearance: drawing (real) | reference (given the real grooves and poses) | round-2 frontier submission, unchanged | the same with insertion by touch (below) |
|---|---|---|---|
| 1.8 mm (1.65 to 1.95), fingers not coupled: amendment 2 | 16 of 16 | 8 of 16 kits, 31 of 48 nuts (9 of 16 through the grader) | |
| 1.0 mm (0.85 to 1.15) | 16 of 16 | 5 of 16, 24 of 48 | 8 of 16, 38 of 48 (an earlier version of the search) |
| 0.75 mm (0.6 to 0.9) | 16 of 16 | 4 of 16, 17 of 48 | 6 of 16, 37 of 48 (an earlier version of the search) |
| 0.5 mm (0.35 to 0.65): amendment 3 | 16 of 16 (624 to 803 steps) | 1 of 16, 8 of 48 | 6 of 16, 37 of 48; 48 of 48 insertions on the shaft |

**Insertion by touch with the agent's observations alone.** A maintainer's controller (kept with the review's
records, not shipped) changes the round-2 frontier submission's insertion only; its perception, grasp and groove
search are as submitted. The first nut of a
kit is brought beside its shaft and its rim is pushed against the shaft's side from two directions, which gives the
shaft's axis in the hand's own terms; the difference to the cameras' plan is kept in the hand's frame and corrects
the plans for the kit's other nuts; a nut that still comes to rest on the shaft end is slid along a small spiral
under a light push until it dips over the shaft. It uses `ee_pos`, `ee_quat` and `wrench` and nothing else. See
`observability/README.md` for its result through the grader on amendment 3's development kits.

What did not work, for the record: reading the direction of the shaft from the wrist's torques while the nut is
pressed on the shaft end. The contact sits somewhere on a thin crescent of the shaft end beside the hole, and with
a nut that sags a few tenths of a degree in the grip it sits at one end of that crescent as often as in its
middle: 13 presses at 11 to 15 N gave the direction within 45 degrees in 10 cases and off by 75 to 100 degrees
in 3. The data sheet says so.

## What was not varied, and why

- The layout rules, the groove family's rules, the controller's law, gains and limits, the nut's outside and handle,
  the collar, the shaft's height and the bands: as drawn. They either vary already (the layout is drawn per
  episode) or a tolerance on them changes nothing a controller could feel or see at these resolutions.
- The pads' torsional friction is not brought down to a flat rubber pad's real value (about 0.005 m): with it the
  handle grasp the reference uses cannot hold a nut, which is what the contribution's notes on `condim` 4 say. It
  stays in proportion to the sliding friction.
- No actuation or image latency, no joint encoder offsets: the first would change the cell's design, not a
  tolerance; the second is far below the cameras' errors (Franka gives a pose repeatability of +-0.1 mm).
- The tolerance code was checked against the earlier simulation at 45e088b (`tolerances=False` repeated the round-2
  runs to within one step). Since amendment 3 the drawing itself differs (the shaft, the coupled fingers), so that
  switch now gives the new drawing's kit on an ideal cell.

## What it does to the reference and to a round-2 controller

Before any candidate ran, on maintainer kits (`sample_kit` seeds 1000 on, layout seed + 4000):

- The reference (given the real grooves through the token): 28 of 32 kits complete in 576 to 821 steps, as many as
  without tolerances (the four it fails are wrist-range cases and one ruined nut, as before). With the tolerances
  switched off (`tolerances=False`) and the earlier kits it repeats its round-2 runs to within one step.
- The round-2 frontier submission (observations only), 16 kits, 1000 steps: 10 of 16 complete without tolerances,
  6 of 16 with them. One group at a time switched off: perfect cameras 10 of 16, no joint friction 8 of 16, the
  model's pad friction 5 of 16. The cameras' calibration error is what it does not cope with: it puts nuts on shaft
  tops several millimetres off and its search (up to 3 mm) does not find the shaft.

After the selection of amendment 2, on the commissioning service's 16 development kits: the same submission
completes 9 of 16 through the grader (12 of 16 with the cameras' error switched off). See `commissioning.md`.
