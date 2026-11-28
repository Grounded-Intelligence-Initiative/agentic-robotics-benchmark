# Kitting cell: integration manual for the hand's grip controller

The hand is fitted to the arm of the lab's kitting cell. The arm carries the hand between a pick nest and a place
nest along fixed profiles; the cell's supervisor presents the specimens, runs the phases of each job and gives the
controller each job's ticket. The controller drives the hand only: it cannot change the arm's motion.

## 1. Frames

- **Hand frame H**: x across the palm, y from the wrist towards the fingertips, z from the palmar side (-) to the back
  of the hand (+). The palm is fixed in H; everything the hand's documents give (`../hand/kinematics_sheet.md`) is in
  H. A pose is a position in m and a unit quaternion [w, x, y, z].
- **Cell frame**: the pick nest's frame at the pick pose, z up. The arm's motion (section 5) is given in it.
- **Nests**: the pick nest and the place nest, each with its frame; their poses in H, with the arm at the pick pose,
  are in the cell data (`cell["nests"]`). A specimen is placed in a nest at the nest's centre within the placement
  tolerance of section 6. Each nest is a plate under the specimen and four walls 3.5 mm
  high round its pocket (`cell["nest_geometry"]`).

## 2. The cell data given to `setup`

`setup(record, cell)` receives the unit's record (`../unit/record_format.md`) and the cell's data, a JSON object:

| key | what |
|---|---|
| `format` | `"cell/1"` |
| `nests` | `{"pick": pose, "place": pose}` in H (section 1) |
| `nest_geometry` | `{"wall_height_mm"}`: the height of each nest's walls above its plate (section 1) |
| `taught` | the taught grasp: the content of `taught_grasp.json` (section 3) |
| `test_lift` | the test lift: `{"rise_mm", "rise_s", "hold_s"}` (section 5) |
| `motion` | the arm's motion law: translation law and peak, tilt rate and ramp, tracking range, lower and set-down |
| `profiles` | the transfer profiles: the content of `profiles.json` (section 5) |
| `specimens` | the specimen sheets: the content of `../specimens/specimen_sheets.json` |
| `placement` | the placement tolerance and the clearance of section 6 |
| `timeline` | the times of section 4, in s |
| `shifts`, `jobs_per_shift` | 2 and 10 |

Nothing in the cell data says which specimens will come: each job's ticket arrives with that job (section 7).
`cell_data.json` beside this manual is the cell data exactly as `setup` receives it (the self-test uses it).

## 3. The taught grasp

The hand grasps every specimen with one taught three-pad grasp: the thumb pad against the index and middle pads,
squeezing along the specimen's squeeze axis; the ring finger is parked out of the way. `taught_grasp.json` gives the
taught postures as joint angles in degrees of the 13 servo joints (joint angle q = direction x (Present
Position - home offset) x 360/4096 deg, with direction and home offset from the unit's record):

- `open`: the open pre-shape in which the hand waits for a specimen;
- `contact`: the taught posture at first contact of the three pads with a specimen of width 35, 40 and 45 mm placed at
  the nest's centre (keys `"35"`, `"40"`, `"45"`);
- `ring_park`: the ring finger's parking posture (ring abad, ring mcp, ring pip).

| joint | open | contact 35 mm | contact 40 mm | contact 45 mm |
|---|---|---|---|---|
| thumb ip | 79.02 | 73.44 | 77.78 | 79.02 |
| thumb mp | -56.99 | -41.02 | -42.82 | -46.58 |
| thumb side | 16.16 | 10.61 | 11.04 | 12.25 |
| thumb roll | 90.02 | 114.68 | 114.29 | 111.30 |
| index pip | -11.24 | -25.98 | -25.11 | -24.19 |
| index mcp | -1.79 | -4.31 | -3.62 | -3.02 |
| index abad | 12.34 | 11.76 | 11.89 | 12.01 |
| middle pip | -6.40 | -18.98 | -18.56 | -18.09 |
| middle mcp | -19.15 | -24.07 | -22.91 | -21.84 |
| middle abad | 13.01 | 12.47 | 12.59 | 12.71 |
| ring pip | -60.00 | -60.00 | -60.00 | -60.00 |
| ring mcp | -80.00 | -80.00 | -80.00 | -80.00 |
| ring abad | 25.00 | 25.00 | 25.00 | 25.00 |

Ring finger parking posture: ring abad 25.00 deg, ring mcp -80.00 deg, ring pip -60.00 deg.

(The table rounds to 0.01 deg; `taught_grasp.json` holds the values.)

## 4. A job, phase by phase

Every observation carries the cell's state: the shift, the job (1 to 10, or null outside a
job), the phase, the time in the phase (ms) and the job's ticket. The phases:

| phase | what happens | duration |
|---|---|---|
| `handover` | the unit is handed over: every servo torque off, the EEPROM of the record, the RAM at the vendor's initial values | from t = 0 to 3 s |
| `await_clear` | the job starts and its ticket is shown; the specimen arrives and is placed in the pick nest as soon as every part of the hand is at least 2 mm clear of its place | until placement; at most 2 s, then the job is lost (not presented) and the next job starts |
| `present` | the specimen sits in the pick nest; the controller closes the grasp and raises `grasped` (or `decline`) | until the `grasped` flag; at most 3 s after the placement, then "release", the specimen is removed and the job is lost (not grasped) |
| `test_lift` | the arm rises 8 mm in 0.5 s and holds for 2.5 s | 3 s |
| `transfer` | the arm carries the hand to the place nest along the ticket's profile (section 5) | T0 2.04 s, R30 and P30 8.44 s (at tracking factor 1) |
| `set_down` | the arm goes down 8 mm in 0.5 s onto the place nest | 0.5 s |
| `release` | the hand is asked to let go; the specimen is then removed | 1 s |
| `rest` | the empty hand waits at the pick pose | 2 s |
| `lower` | after a decline in the test lift, or a specimen not lifted at its end: the arm goes down 8 mm in 0.5 s onto the pick nest, then "release" | 0.5 s |
| `pause` | between the two shifts | 10 s |
| `end` | after the last job of shift 2 | 1 s |

- A `decline` raised in `present` sends the job to `release` at the next cycle; one raised in `test_lift` to `lower`
  and then `release`. After the release of a declined specimen it is removed and the job goes to `rest`.
- After an event that ends a shift (`../acceptance/acceptance_spec.md`, section 4) the specimen is removed at once and
  the timeline goes to `pause` (in shift 1) or `end` (in shift 2).
- The times of every job of a shift are recorded on its shift sheet (`shift_sheet_format.md`, `example_shift.json`).

## 5. The arm's motion

The arm moves the hand; the hand feels the arm's accelerations and the tilt of gravity. Translations are
minimum-jerk moves with acceleration peaks of 6 m/s^2 (a move of length d
takes sqrt(10 / sqrt(3) x d / 6) s); a tilt turns at 10 deg/s with a
0.1 s ramp at each end (a tilt of angle a takes a / 10 + 0.2 s).
In the cell frame:

- test lift: up 8 mm in 0.5 s, then a hold of 2.5 s;
- transfer: up 0.15 m; the profile's tilt; 0.25 m along x; 0.25 m
  along y; 0.3 s still; the tilt back; down 0.15 m;
- set-down: down 8 mm in 0.5 s onto the place nest; lower: down
  8 mm in 0.5 s onto the pick nest.

The three profiles (`profiles.json`):

| profile | tilt | transfer duration (tracking factor 1) |
|---|---|---|
| T0 | none | 2.04 s |
| R30 | 30 deg about the cell's x axis | 8.44 s |
| P30 | 30 deg about the cell's y axis | 8.44 s |

The arm tracks its profile within 10 %: for each job the accelerations the hand feels are the profile's times a
factor between 0.9 and 1.1.

## 6. Placement

The supervisor places a specimen within 2 mm of the nest's centre in any direction and
within 3 deg of the nest's yaw. It places it only once every part of the hand is at least
2 mm clear of the specimen's place.

## 7. Tickets

Each job's ticket is shown in the observations from the job's `await_clear` to its `rest` (null outside a job and at
setup): `{"job": 1..10, "class": "brittle" | "soft" | "full_load", "code": "SH-1" | "SH-2" | "SH-3" | "SP" | "FL", "finish": "matte" | "rough", "profile": "T0" | "R30" | "P30"}`.
The ticket does not carry the specimen's mass, friction, stiffness or label (`../specimens/specimen_sheets.md`).
