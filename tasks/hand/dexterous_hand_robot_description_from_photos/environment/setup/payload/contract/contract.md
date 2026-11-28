# Contract of the hand's description

The files you deliver in `/home/user/submission/` are used as they are: the grader reads them, and so does the
partner's software. This page says what they must hold and what the grader does with them. Every rule of the section
"Gate" is checked before anything is scored; `check_submission.py` runs rules 1 to 9 of them on any folder.

## Files

| file | content |
|---|---|
| `robot.xml` | the MJCF of the hand (MuJoCo 3.14): bodies, hinges, the coupling of each finger's last hinge, four pad sites, inertials, 13 position actuators; geometry as you like |
| `assets/` | every file `robot.xml` references (meshes); paths relative to the submission |
| `config.yaml` | the calibration file of the host software for this hand (`docs/host_software.md`): `motor_ids`, `home_offsets` (radians), `joint_signs` |

Other files may be present; the grader does not read them (they count towards the caps of rule 1).

## Frames and units

- The palm frame is the frame of `parts/assembly.json`: x across the palm, y from the wrist towards the fingertips, z
  from the palm side (negative z) towards the back of the hand. The body `palm` is a jointless child of the world body,
  and every position the grader reads is expressed in the frame of the body `palm`, so that frame must be the palm
  frame.
- SI units: metres, radians, kilograms. `robot.xml` states `<compiler angle="radian">` explicitly.
- q = 0 for all 16 hinges is the pose of the given placements (`parts/assembly.json`), figure `zero_pose.png`.
- `<option>`: only `timestep` in [0.0005, 0.005] s and `gravity="0 0 -9.81"`. The grader never steps
  the model: everything it scores is static (kinematics, the actuators' force ranges, masses).

![zero posture](zero_pose.png)

## Names

- Root body `palm`. Each digit is a chain from the palm outwards, each hinge in the body of its own name:
  - thumb: `thumb_roll` > `thumb_side` > `thumb_mp` > `thumb_ip`
  - each finger f in `index`, `middle`, `ring`: `f_abad` > `f_mcp` > `f_pip` > `f_dip`
  The design table (`docs/design_table.md`) gives the servo id and the axis of each name.
- Bodies without a joint may be added anywhere; each belongs to its nearest contract ancestor (its mass counts for
  that digit). Further hinges are allowed only in bodies named `f_loop1` ... `f_loop4` below the body `f_abad` of the
  same finger (the bars of a closed chain). No other joint type.
- Every one of the 16 hinges is limited (`range` in radians, in your own convention of the hinge's sense).
- Sites `thumb_pad`, `index_pad`, `middle_pad`, `ring_pad`, each rigid on its digit's last body (`thumb_ip`, `f_dip`)
  or on a jointless body below it (see "Pads").
- Actuators: exactly 13, `act_<joint>` for each servo-driven hinge (the 16 hinges without `index_dip`, `middle_dip`,
  `ring_dip`): a `<position>` actuator with `kp` greater than 0, or a `<general>` actuator with gaintype fixed,
  biastype affine, gainprm[0] greater than 0 and biasprm[1] = -gainprm[0], on its joint, gear 1,
  `forcelimited="true"` and `forcerange` = [-f, +f] with f finite and greater than 0. The grader reads f as the most torque
  the servo applies to its hinge.

## The coupling of a finger's last hinge

`f_dip` has no servo. It is coupled to `f_pip` in one of two forms:

- exactly one joint equality `<joint joint1="f_dip" joint2="f_pip" polycoef="...">` (and then no extra hinge in that
  finger), or
- `<connect>` equalities whose two sides lie on bodies of finger f (a closed chain through `f_loop` bodies), at
  most 8 active ones in one finger,

not both. An inactive equality couples nothing (and is not counted).

The grader solves the passive hinges of a closed chain itself: Gauss-Newton on the passive joints, by continuation
from q = 0 in steps of at most 2 deg of the driven hinges, to a residual of at most 1e-9 m.
A configuration at which your chain does not close fails the items scored there (it is not a gate rule). The sense of
`f_dip` is read from your model: at q = 0, turning `f_dip` alone by -0.05 rad moves the finger's pad;
if that move points to the palm side (-z), your model flexes the hinge with negative angles, otherwise the range of
`f_dip` is read in the opposite sense. Either sense scores alike.

## Pads

A pad is the sensing surface of a tactile module (ZCP-F561 on each finger, ZCP-DG1Z on the thumb). The pad point
is the area centroid of the module's sensing surface, projected onto that surface; the pad normal is the outward unit
normal of the surface at that point. In the frame of the module's part file (`parts/meshes/<code>.stl`):

| module | pad point (mm) | pad normal |
|---|---|---|
| ZCP-F561 | (10.371, 56.454, -8.489) | (0.98749, 0.15768, 0.00000) |
| ZCP-DG1Z | (0.001, 7.314, 13.000) | (-0.00040, 0.04459, 0.99901) |

The site `<digit>_pad` sits at the pad point of the digit's module, its +z axis along the pad normal (its x and y axes
are free). Figure `pad_definition.png`.

![pad definition](pad_definition.png)

## The calibration file and what the grader does with it

`config.yaml` is the host software's own file: `motor_ids` = the list 0 ... 12; `home_offsets` and `joint_signs`, each
a mapping with one entry per servo id (the key an integer or its decimal string, each id exactly once; an id given
both ways, a missing id or an extra key is rejected) or a list of exactly 13 numbers in `motor_ids` order. Values are
finite numbers (YAML booleans are rejected); each joint sign equals +1 or -1. `joint_lower_limits` and
`joint_upper_limits` may be present and are ignored by the grader (the travel is read from `robot.xml`); any other
top-level key is accepted and ignored, as the host software ignores it (a key given twice is rejected). With
home offset h (rad), sign s and D = 2 pi / 4096 rad per count:

- Reading. A read-out k (the 13 Present Position(132) counts) gives your model the angles q = (k D - h) s.
- Commanding. To reach model angles q, the host software sends raw = s q + h (rad) to each servo as the goal count
  `int(round(raw / D))` (Python's round, half to even).

How the hand is used:

- The partner takes the hand out of its box and switches it on; from then on the hand is commanded only through the
  host software with your `config.yaml`.
- For a request on some digits (a pinch: the thumb and the index finger; a single-pad target: one digit) the grader solves
  the targets on YOUR model with its own IK (below), converts the solution to goal counts through YOUR `config.yaml` as
  above and sends them to the servos of those digits only. Every other servo stays where it was when the hand was
  switched on.
- Zero rules of a request: a commanded servo that would be driven more than 2 deg beyond the real hand's
  end of travel stops there and the request scores 0 (a servo commanded at most 2 deg beyond an end stops
  at the end); two parts of the real hand pressed more than 1 mm into each other at the reached pose score
  the request 0 (pressing deeper would exceed the partner's force cap); every request of a digit that the range check
  locks out scores 0.
- Range check. Before the hand is used, the partner's software drives each servo-driven hinge, alone, from the zero
  posture (every other servo-driven hinge at zero), to both ends of the range your `robot.xml` states for it (through
  your `config.yaml`); a hinge that stops more than 2 deg short of a stated end (below, "Ends of
  travel") is locked out, together with its digit (a pinch is a request of the thumb and of the index
  finger), until the description is corrected. A stated range that ends inside the real hand's travel locks nothing
  out; the IK then cannot reach beyond it. The range check zeroes requests only; readings are scored as they are.
- Zero rule of a reading: a reading that your calibration maps more than 2 deg outside your model's
  range of a joint scores the items of that digit 0 at that reading (for the whole-hand items: any joint).
- Travel. The two ends of each servo-driven hinge's range in `robot.xml` are converted to counts with your
  calibration and compared with the hand's own ends of travel there; a finger's last hinge is compared in the sense
  read from your model.

## Ends of travel

The real hand has no end stops other than its own parts. A servo-driven hinge's end of travel is the angle at which,
as that hinge alone is turned from the zero posture (every other servo-driven hinge at zero, a finger's last hinge
following its crossed four-bar), two parts of its own digit, or a part of its digit and a part of the palm, first
press into each other. Parts that already touch at the zero posture count from where they begin to press into each
other; parts of other digits are not obstacles; screws are not parts. The parts are the solids of `parts/`
themselves (their meshes as given, not their convex hulls: the hulls serve the 1 mm rule only). Two parts
first press into each other at the angle at which their surfaces first touch; no depth is allowed for. Surfaces that
turn on each other about a hinge's axis (a pin in its hole, a bearing in its seat, a horn against its bracket) are
not stops; other surfaces of the same two parts can be. A finger's last hinge is not turned on its own: its two ends
of travel are its angles at the two ends of the hinge before it.

## The grader's IK

Damped least squares on the driven hinges of the digits of the request, clamped to your model's ranges at every
iteration; 24 starts (the middle of the ranges, then Halton points over the ranges shrunk by
2 deg on each side, the same for every grading run); at most 150 iterations per start; a start
stops when every target pad is within 0.1 mm. The solution is accepted when its best residual is at most
1 mm on every target pad; else the request scores 0. A pinch has two targets, `thumb_pad` = m + (g/2) u
and `index_pad` = m - (g/2) u (m the midpoint, u the unit vector from the index pad to the thumb pad, g
the gap), plus the term (1 + n_t . n_f) of the two pad normals with weight 0.002; the score uses the positions
only.

## The collision envelope

The 1 mm rule is measured on the hand with the convex hull of each part listed in `collision_pairs.json` (each made
part, each tactile module, and each servo as its solid is given (the case with its output horn and rear bearing: one
part file)). Pairs that count: parts of two different digits, and parts of a digit and of the palm. Not counted: parts on bodies that are parent and child, and parts whose hulls are closer than
0.001 mm to each other with every joint at zero; of the pairs that count by the rule above, `excluded_pairs`
lists every one that is not counted, with its reason.

## MJCF rules

- Elements: the usual `mujoco`, `compiler`, `option`, `size`, `visual`, `statistic`, `default`, `asset` (`mesh`,
  `texture`, `material`), `worldbody`, `body`, `geom`, `joint`, `site`, `camera`, `light`, `inertial`, `actuator`,
  `contact` (`exclude` between bodies of the hand), `equality` (only `connect` and `joint`, as above), `keyframe`,
  `sensor`, `include` (of a file inside the submission; included files nest at most 5 deep and never in
  a cycle). Not allowed: `attach`, `composite`, `custom`, `deformable`, `distance`, `extension`, `fixed`, `flex`, `flexcomp`, `frame`, `hfield`, `instance`, `numeric`, `pair`, `plugin`, `replicate`, `skin`, `spatial`, `tendon`, `text`, `tuple`; `mocap` or `gravcomp` bodies; the geom attributes
  `fluidcoef`, `fluidshape`; DOCTYPE or ENTITY declarations. `<compiler>` takes only `angle`, `assetdir`, `autolimits`, `balanceinertia`, `coordinate`, `discardvisual`, `eulerseq`, `exactmeshinertia`, `inertiafromgeom`, `inertiagrouprange`, `meshdir`, `strippath`, `texturedir`, `usethread`.
- Every `file=` and every `<compiler>` `meshdir`, `texturedir` or `assetdir` is a relative path without a `..`
  component (it stays inside the submission).
- Contact parameters, where given: sliding friction in [0.3, 3/2], torsional in
  [0, 0.05], rolling in [0, 0.01]; `margin` at most
  0.001 m; `gap` 0; `condim` in {1, 3, 4, 6}; `priority` 0; `solref` in the positive form with timeconst in
  [0.002, 0.05] and dampratio in [0.5, 2]; `solimp` d0 and dwidth in
  [0.5, 0.9999] with d0 at most dwidth, width in [0, 0.01]; `solmix` in
  [0, 1000]; the same bands for a joint's `solreflimit`, `solreffriction`, `solimplimit` and
  `solimpfriction`; joint armature, damping, frictionloss and stiffness at least 0; actuators on joints of the hand
  (below `palm`) only, gear 1; at most 512 bodies.

## Gate

A description that fails any rule below scores 0 (`model_loads` = 0).

1. Files: `robot.xml` and `config.yaml` present and regular; no symbolic links or special files; every `file=`
   relative, without `..`, inside the submission; the submission at most 200 MB (the larger of the apparent
   size and the allocated disk space) and at most 20 000 files and folders; at most 5 000 000 mesh
   triangles, counted per mesh asset from the mesh files before anything is compiled (a binary STL by the triangle
   count it states, an OBJ by its faces, a face of k corners as k - 2 triangles, an MSH by its face count, inline faces
   by their number, a built-in mesh from its parameters; a mesh given by its vertices alone counts 2 V - 4, the most
   its convex hull can have), and again in the compiled model.
2. The MJCF rules above, checked before anything is compiled.
3. `<compiler angle="radian">` explicit; `robot.xml` compiles within 120 s of CPU time with zero compiler
   warnings.
4. Structure: `palm` a jointless child of the world body; the 16 contract hinges, each limited, each in the body of its
   own name, in the chains above; extra hinges only in `f_loop1` ... `f_loop4` below `f_abad`; no other joint type
   (a `freejoint` or a free, ball or slide joint is refused).
5. The coupling of each `f_dip`: one joint equality or connect equalities on bodies of finger f, not both, and at
   most 8 active connect equalities in one finger (no value of a coupling is checked here).
6. The four pad sites, each rigid on its digit's last body or on a jointless body below it.
7. The 13 actuators as above.
8. Every body mass finite and at least 0; the total mass greater than 0.
9. `config.yaml` at most 64 KB; rejected before parsing: a YAML node marker ("&" or "*" outside quoted
   strings), "[" or "{" nested deeper than 4 outside quoted strings, a line indented by more than
   16 spaces; then a safe YAML load; `motor_ids` exactly [0, 1, ..., 12]; `home_offsets` and `joint_signs` in
   the forms above; every home offset within +-1000 rad.
10. Nothing in the submission written after your session ends (the grader checks this; `check_submission.py` cannot).
11. The grader's own work on the description once the rules above have passed (closing each finger's chain, the IK of
    every request, every item) takes at most 300 s of CPU time and at most 1800 s of wall-clock time; a
    description that needs more scores 0 (the grader checks this; `check_submission.py` cannot).

`check_submission.py` runs rules 1 to 9.
