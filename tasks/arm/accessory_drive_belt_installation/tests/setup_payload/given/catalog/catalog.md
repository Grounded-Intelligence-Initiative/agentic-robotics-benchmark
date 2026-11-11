# Drive drawing and data sheets (engine accessory drive, variant A)

Lengths in millimetres unless marked; positions in the engine frame E (`cell_manual.md`): x to the right and z up as
seen from the robots, depth d in front of the accessory face. `describe()["drive"]`, `["belt"]` and `["tensioner"]`
carry the same numbers in metres and degrees. Scaled duty: the tensioner's spring, the belt's tension and the
accessories' drag are about one tenth of a production engine's, so that the cell's arms can work the drive; the run-in
load and the slip specification are scaled with them.

## Routing decal ROUTE-A1

The belt runs counter-clockwise as drawn: C (bottom) -> T (right) -> ALT (top right) -> WP (left) -> C. Ribs on C, WP
and ALT (the pulleys are inside the loop); the belt's back on T (T is outside the loop, pressing on the belt's back).

| pulley | centre (x, z) | groove floor radius | kind |
|---|---|---|---|
| C, crank | (0, 0) | 70 | grooved |
| WP, water pump | (-120, 136) | 50 | grooved |
| ALT, alternator | (84, 228) | 45 | grooved |
| T, tensioner pulley | on the tensioner arm (below) | face 45 | smooth, flanged |

**Grooved pulleys (C, WP, ALT).** Seen from the front: a back shoulder from d = 10 to 16, the groove zone from d = 16 to
38 (22 wide, for the 20 wide belt), a front shoulder from d = 38 to 41, then a 45 deg chamfer 2.5 deep from the shoulder
top down to the floor radius in front. Both shoulders stand 2.5 proud of the groove floor. A belt section is seated when
it lies in the groove zone on the groove floor; a section on a shoulder's top or on the chamfer rides it.

**Tensioner pulley (T).** A smooth face of radius 45 from d = 15 to 40 between two flanges 4 proud of the face (d = 12 to
15 and 40 to 43). The belt's back runs on the face.

## Automatic tensioner TA-1

| | |
|---|---|
| pivot | (140, 160), axis along the pulley axes |
| arm | 70 from the pivot to the pulley's centre; its angle theta is measured counter-clockwise from E +x as drawn |
| travel | closed stop theta = -152 deg (no belt) to the install stop theta = -72 deg; at the install stop the pulley and its flanges stand 10 clear of the straight belt line from C to ALT |
| window marks | theta = -149 deg to -133 deg: two white lines on the face 40-52 from the pivot at theta + 180 deg; a correctly fitted new belt of any length within its tolerance comes to rest between them |
| pointer | a white blade on the arm's boss pointing away from the pulley (theta + 180 deg), 30-46 from the pivot, 8.5 in front of the face |
| spring | torsion spring towards closing: torque = preload 1.84 N m at the closed stop + rate 0.83 N m/rad x (theta - closed stop); preload and rate each within +-15 % of these values per engine |
| damper | viscous, 0.7-1.3 N m s/rad; rated for 60 deg/s of arm speed: a faster arm (the lever let go while the arm is off the belt: a snap-back) damages it |
| pivot friction | 0.04-0.12 N m |
| service lever | a round bar of diameter 16, parallel to the pulley axes from d = 4 to 89, on a branch of the arm 150 from the pivot at 123 deg from the pulley arm (it swings with the arm on a 150 radius circle about the pivot); lever force = spring torque / 0.150 m: about 13 N with the arm in the window, 20 N at the install stop (up to about 23 N on a stiff tensioner) |

Service: swing the tensioner off the belt by its lever along its arc (the lever's path is a circle about the pivot: a
hand pulled along a chord loads it sideways), hold it while the last span goes on, and let it back slowly onto the belt;
never let it go while it is off the belt.

## Poly-V belt 6PK1050

| | |
|---|---|
| section | 20 wide, 4.5 thick; black, its back printed white every 50 |
| nominal length | 1050 (+-0.5 %, so 1044.8-1055.3) |
| construction (as modelled) | a chain of rigid 10-long elements (about 105) with flatwise and edgewise bending joints, the edgewise joints 8 x as stiff as the flatwise; it does not twist; the ribs and the grooves are not modelled (a flat belt that seats in the groove zone) |
| mass | 0.22-0.26 kg/m |
| stiffness | droop class: a 200 free end held level droops 32-71 |
| friction | on the pulleys' grooves and faces mu 0.6-0.9; everything else (the hands, the arms, the peg, the engine) touches it without friction |
| install slack | with the tensioner at its install stop the belt is 35-45 longer than the three-pulley path C-ALT-WP (1009.9 at the belt's middle line): this is all the slack there is to get the last section over a rim |

## Run-in test (the grader's, after `finish`)

The crank turns at 1.5 rev/s for 4 s; the water pump and the alternator are braked with 0.2-0.4 N m each. Slip is
within spec when each accessory turned within 3 % of the crank's angle times its ratio (crank radius / its radius, at the
belt's middle line). A belt on a shoulder may ride up further or walk off during the run-in.
