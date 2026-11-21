# Unitree Go1 with the inspection saddle: datasheet

Values of the robot as simulated (MuJoCo 3.16, 2 ms time step, elliptic friction cones). "Per site" means: drawn
once for each graded site and for each pad episode, uniformly from the stated range, and not reported; a program
finds a site's values from what its robot measures there.

## Body

| quantity | value |
|---|---|
| mass without the saddle | 12.74 kg (trunk 5.20 kg; each leg 1.88 kg: hip 0.68, thigh 1.01, calf and foot 0.19) |
| trunk inertia about its centre (x, y, z) | 0.017, 0.063, 0.072 kg m^2 |
| hip pivots | x = +/-0.1881 m, y = +/-0.04675 m in the trunk frame |
| thigh axis | 0.08 m outboard of the hip pivot |
| thigh, shank | 0.213 m each |
| foot | sphere, radius 0.023 m |
| home pose | hip 0, thigh 0.9, calf -1.8 rad; the trunk origin then stands about 0.27 m above the floor |

## Joints (FR, FL, RR, RL; hip, thigh, calf)

| quantity | value |
|---|---|
| command | position target; torque = stiffness x (target - angle) - damping x speed, clipped at the limit |
| stiffness | 100 N m/rad x a per-joint factor, **per site in [0.85, 1.15]** |
| damping | hip 1, thigh 2, calf 2 N m s/rad |
| torque limit | hip and thigh 23.7 N m, calf 35.55 N m, x one factor for all joints, **per site in [0.85, 1.00]** |
| rotor inertia, dry friction | 0.01 kg m^2, 0.2 N m |
| target range | hip [-0.863, 0.863], thigh [-0.686, 4.501], calf [-2.818, -0.888] rad |

## Load

| quantity | value |
|---|---|
| saddle | 0.70 kg, rigid: pedestal, crown guard, scanner shank and scanner head (dimensions in the instruction) |
| instrument pack | on the saddle's rail, 0.06 m above the trunk origin, on the centre line |
| total carried mass (saddle + pack) | **per site in [2.50, 4.50] kg** |
| pack position along the trunk's x axis | **per site in [+0.05, +0.28] m** ahead of the trunk origin |

A heavier or more nose-ward pack makes the legs sag and the trunk pitch under the same joint targets.

## Ground

| quantity | value |
|---|---|
| sliding friction of every walked surface (floor, aprons, deck, guides) | **per site in [0.40, 0.90]** |
| torsional, rolling friction | 0.02, 0.001 |

## Fittings (see the instruction for the rest of their dimensions)

| quantity | value |
|---|---|
| underside of the suspended duct above the floor | **per site in [0.410, 0.445] m** |
| crest of the deck's guides above the deck | **per site in [0.070, 0.125] m** |

Both show in the survey: the teal cover lies 1.4 mm above the duct's roof (the roof is 0.068 m above the
underside) and 1.4 mm above the centre guide's crest (the deck is 0.065 m above the floor).

## Sensors

| reading | what it is |
|---|---|
| `joint_rad`, `joint_rad_s` | joint angles and speeds, rounded to 1e-4 |
| `gravity_body_unit` | unit vector of gravity in the trunk frame, rounded to 1e-5 |
| `gyro_body_rad_s` | trunk angular velocity in the trunk frame, rounded to 1e-4 |
| `foot_contact` | per foot: touching the floor, an apron, the deck or a guide |
| `odom_xy_m`, `odom_yaw_rad` | displacement (rounded to 1 mm) and yaw change since the settled launch pose, in the launch frame |
| mast camera | 256 x 256 RGB pinhole, 70 degree vertical field of view, 0.20 m above the trunk origin, 36 degrees up, eight pan detents of 45 degrees |

No reading has noise or delay beyond the rounding. There is no joint torque, range, depth or absolute pose reading.
