# Move the hazardous carriers left in a laboratory to the hazardous-materials store, each into its own vessel, and leave the lab as it was, with no rehearsal

You are the controls engineer of a hazardous-materials response team. Two or three carriers of
hazardous samples were left in a laboratory: in closed drawers, on the floor beside or behind
furniture, on a bench or a table. They belong in the team's store across the yard, each in the
containment vessel of its class, under a shut lid. What is inside a carrier is not described:
it can spill when the carrier is tipped or jolted, and it does not keep forever out of its
vessel. The team's cart, a Franka Panda with a hook tool on an Omron mobile base, with a depth
camera on a mast, has to find every carrier (pulling drawers open by their pulls where it has
to), move all of them to the store and put each into the vessel marked with its class's colour,
shutting the lid, and leave the lab as it found it: every drawer shut again, every stool and
chair where it stood; all of it before the time runs out. A trolley stands in the yard; whether
and how to use it is yours to decide.

The limits below are chosen for the control problem; they are not those of any real
substance or device.

Write the controller that does this. It runs closed loop, one lab per episode, in labs it has
never seen. **There is no rehearsal.** You have the team's own cart, exactly as the range
simulates it, an empty range to drive it on, the models the lab's furniture is made of, and
this description of everything the cart will meet; you do not have a simulator of a lab, of a
drawer, of a carrier or what is in it, of the trolley, of a vessel's lid or of the latch closing
on anything, and there are no practice labs. The first live lab your controller meets is the
first one it is graded on, and an episode cannot be repaired: once a carrier has spilled, that
episode is lost.

You have 3 hours. Whatever is in `/home/user/submission/` when the time ends is graded, so
keep a working controller there.

## The range

World frame: z up, metres; the floor and the ground outside are z = 0. The range is simulated
with MuJoCo 3.16 at a 2 ms time step (implicitfast integrator, elliptic friction cones,
`impratio` 5); `arm_model.py` holds these options (`new_world`) and the contact parameters
every colliding surface has (`SOLREF`, `SOLIMP`, `condim` 4).

- **The lab.** Its inside is 7.0 by 5.6 m: x from 0 to 7.0, y from -2.8 to 2.8. The walls
  are 2.2 m high and 0.10 m thick. The door is a gap in the wall x = 0, from y = -0.65 to
  y = 0.65 (that wall fills x from -0.10 to 0). Outside the door is open ground, the yard.
- **The cart** starts every episode at rest outside the door, the centre it turns about at
  (-1.6, 0), facing the door (heading 0, along +x), its arm in the pose `TRAVEL_Q` of
  `cart_model.py`. See "The cart" below.
- **The store** is a room across the yard: its inside is x from -10.4 to -5.6, y from -2.2 to
  2.2; its walls are 2.2 m high and 0.10 m thick; its door is a gap from y = -0.9 to y = 0.9 in
  its wall x = -5.6 (that wall fills x from -5.6 to -5.5), facing the lab's door.
- **Four vessels** stand in the store along its side walls, each on a stand: a square column
  0.40 by 0.40 m and 0.30 m high, its sides along x and y. The stands' centres are
  (-9.3, 1.97), (-7.6, 1.97), (-9.3, -1.97) and (-7.6, -1.97) (every observation gives them,
  `vessels`). A vessel is a square box on its stand's top, centred on it: inner opening 25 by
  25 cm, rim 20 cm above the stand's top (0.50 m above the ground), floor 1.5 cm thick, walls
  1.2 cm. A vessel takes one carrier.
- **Classes.** Every carrier and every vessel is of one of four classes, each with its colour:
  blue (0.15, 0.30, 0.85), green (0.15, 0.65, 0.25), purple (0.55, 0.20, 0.75) and white
  (0.92, 0.92, 0.92). The four vessels are of the four classes, in an order that differs from
  episode to episode: a vessel's class is the colour of the plate on its stand's face toward
  the store's middle (the face at |y| = 1.77), 22 cm wide (along x) and 14 cm high, centred on
  the face and 0.16 m above the floor. The carriers of a lab are all of different classes.
- **Every vessel's lid** is a plate of 29 x 29 x 1.2 cm, 0.5 kg, hinged on its vessel. Its own
  frame has its origin on the hinge line, at (0.137, 0, 0.504) from its stand's centre on the
  ground (over the vessel's wall on the +x side, 4 mm above the rim), its x axis along the
  hinge (the world's y) and, when the lid is shut, its y axis across the vessel's mouth (the
  world's -x) and its z axis up. In that frame the plate fills y from 0 to 0.29 and z from 0
  to 0.012. The hinge turns the lid about its x axis between 0 (shut: lying over the mouth)
  and 1.85 rad, with stops at both ends, a damping of 0.02 N m s/rad and a dry friction of
  0.05 N m. **It starts open**, at 1.85 rad: standing beside the mouth, leaning 16 degrees
  back past upright. Its handle is a red bar, colour (0.85, 0.15, 0.15): a capsule of 6 mm
  radius from (-0.07, 0.25, 0.057) to (0.07, 0.25, 0.057) in the lid's frame, on two small
  dark posts; the handle and posts collide with nothing. The plate collides with the cart,
  the arm, the tool and the cases, and not with the vessel (the hinge's stops hold it).
- **The trolley** stands in the yard between the two doors: its middle between x = -3.7 and
  x = -3.1 and between y = -0.5 and y = 0.5, its handle end toward the store's door (its
  heading, the direction of its own x axis, within 0.3 rad of the world's -x). In its own frame
  (origin on the floor under its middle, x along its length, z up):
  - its tray is an open box: a floor whose top face is 0.50 m above the ground, 0.90 m long
    (x) and 0.56 m wide (y), with a rim 5 cm high and 2 cm thick along its edges, so that the
    clear floor inside the rim is 0.86 by 0.52 m; a shelf 0.22 m above the ground, 0.86 by
    0.52 m and 2.4 cm thick; four legs 3 cm square at its corners, from 0.10 m above the
    ground up to the tray. The tray and its rim collide with the cart, the arm, the tool and the
    cases; the shelf and the legs with the cart, the arm, the tool, the walls and the furniture.
    Its wheels collide with nothing.
  - its handle is a **cyan** bar, colour (0.10, 0.75, 0.85), across its +x end: a capsule of
    12 mm radius whose axis runs from (0.53, -0.20, 0.85) to (0.53, 0.20, 0.85), on two
    uprights; the handle and the uprights collide with nothing.
  - it rolls on four swivel casters: in the range it is a body that slides and turns on the
    floor against a resistance. Its mass and how freely it rolls are not reported. Things on
    its tray rest on the tray's floor (the friction below).
- **The furniture** is different in every episode and is not reported: you see it through
  the camera. It is made of the models in `/opt/lab_assets/` (42 models from Poly Haven;
  `manifest.json` gives every model's kind and size, its parts, each part's mesh, texture and
  colour, and the tops things can stand on): benches and tables (often one long bench, or two
  tables pushed together, in the middle of the room) with a stool or a chair pushed back from
  each, a waste canister beside some, a microscope, a chemistry set or a laptop and other
  things on top; steel shelving along the walls; a tool cart with things on it; gas bottles,
  an extinguisher, a hand truck, a tool chest and a warning sign along the walls; boxes,
  crates and containers, a stray stool. Pieces stand at any angle. Laptops are the kit's model
  at 0.6 of its size.
  - What collides is a **box around each piece** (its model's bounding box as it stands), for
    the things standing on furniture too. The camera sees the model's true shape, so the
    space under a bench or between a stool's legs is seen through but cannot be driven or
    reached into.
  - **The stools and chairs are not fixed** (the kit's kind `chair`): each is a free body of
    7 kg, most of its weight 0.18 m above the floor, that moves when it is pushed. Everything
    else of the furniture is fixed.
  - In a picture every part of a model shows in one colour, the `rgba` of its manifest
    entry; the floor, the walls and the cabinets show in the `rgba` of the manifest's surface
    texture they carry: the floor one of the two `floor` textures, the walls one of the two
    `wall` textures, the cabinets one of the two `casework` textures and their tops one of the
    two `worktop` textures (the same for every cabinet of a lab).
  - Kept free of furniture in every lab: the floor inside the door (x below 1.5, |y| below
    1.1), and before every run of drawer cabinets (see below) a strip as wide as the run plus
    0.35 m to either side and 1.75 m deep. A disc of 0.62 m radius about the cart's turning
    centre can always travel from the start to the point 0.91 m in front of every closed
    drawer's pull, and 0.5 m further back from it, without touching anything (stools and
    chairs included, where they stand at the start).
- **The drawer cabinets** are the lab's wall benches. Four or five stand with their backs
  against walls (any of the four), square to the wall, alone or in runs of two or three side
  by side (4 mm apart and of one height, so that their tops read as one worktop). Each is
  0.52 m deep, 0.62 to 0.90 m wide and has its top between 0.72 and 0.80 m above the floor
  (call that height H; the width differs from cabinet to cabinet, H from run to run). Its
  casework is one of the manifest's two `casework` textures (its inner and lower panels 0.8
  times as bright), its top one of the two `worktop` textures. Things may stand on a run's
  top, towards the back.
  - Under the top is **one drawer**; below the drawer the front is fixed. The drawer's front
    panel is 0.20 m high and 18 mm thick, as wide as the cabinet less 4.6 cm, its outer face
    flush with the cabinet's front, its top edge at H - 0.023. Behind it the drawer is an
    open-top tray: inner floor at H - 0.211, clear width the cabinet's width less 7 cm, 0.46 m
    from the front panel's inner face to the back wall, its side and back walls (12 mm)
    ending 3 cm below the front panel's top edge.
  - The drawer starts closed and slides straight out of the cabinet's front by up to 0.36 m,
    against a dry friction of 3 N and a damping of 15 N s/m; it weighs 2.5 kg.
  - Its **pull** is a brass bar, colour (0.90, 0.75, 0.10): a horizontal capsule of 6 mm
    radius and 16 cm length, parallel to the front and centred on it, its axis 4 cm out from
    the front panel's outer face and at height H - 0.093, on two small dark posts. The pull
    and its posts collide with nothing: the tool passes through them.
- **The carriers** (called **packages** below and in the files). There are two or three in a
  lab (`packages` in every observation says how many), each placed on its own as described
  under "Where it is", at least 1.2 m apart when they stand in the open, and never two in one
  drawer.
  - The case is a rigid box of 17 x 11 x 9 cm (x, y, z in the package's own frame, the origin
    at the box's centre), colour (0.35, 0.30, 0.22). Its own friction is 0.3; a
    contact takes the larger of the two surfaces' frictions, so the surface it stands on
    decides.
  - The carrying handle is an **orange** bar, colour (0.95, 0.45, 0.05), along the package's
    x axis: a capsule of 6 mm radius from (-0.065, 0, 0.085) to (0.065, 0, 0.085), 0.02 kg, on
    two dark posts of 5 g each at its ends. Neither the bar nor the posts collide with
    anything: the tool passes through them.
  - Its **class label** is a patch on the case's top, under the handle, in its class's
    colour: 10 cm along x and 7 cm along y, centred on the top.
  - What is inside the case is not described: how heavy a package is, how its weight is
    spread and how its contents move differ from package to package and are not reported.
  - **Where it is**, one of three (about two packages in three the first, more than one in
    four the second, the rest the third):
    - in one of the drawers: on the drawer's floor, at any heading, its centre 0.12 to 0.20 m
      behind the front panel's inner face and at least 0.13 m from the tray's side walls;
    - on the floor, at any heading: 0.32 to 0.50 m from one side of a piece of furniture's
      bounding box, at least 0.35 m from the walls, with at least 0.20 m of free floor around
      the case on every side, outside the floor that is kept free of furniture, and with room
      for the cart to drive up to it squarely from at least one side (its body clear of
      everything by 8 cm on a straight run of 0.6 m that ends with the arm's base 0.56 m
      from the package);
    - on the top of a bench or a table (the kit's kind `desk`, where its top is no higher
      than 0.81 m), at any heading, 0.16 to 0.22 m in from one edge of that top
      and at least 0.15 m from the others, with nothing else on that piece near it or between
      it and that edge, and with room for the cart to stand squarely before that edge (its
      body 10 cm off the edge, clear of everything by 8 cm, with a straight run-up of 0.6 m).

    Wherever it is, it may be out of sight from most of the room.
- **Harmless cases.** One or two drawers (never a package's) hold an empty case of the same
  shape, without a label, that cannot be spilled; its handle is blue (0.1, 0.35, 0.9) or green
  (0.1, 0.7, 0.2). Nothing depends on what happens to them.
- **The latch.** With `latch` 1 in an action, the latch closes as soon as its point is within
  6 mm of the axis of a bar (measured to the nearest point of the axis between the bar's
  ends): any case's handle, any drawer's pull, a lid's handle or the trolley's handle, the
  nearest if several are in reach;
  `latched` then reads 1. From then on the point of that body that was at the latch point
  when it closed stays at the latch point (a MuJoCo `connect` constraint with `solref`
  0.006 1): a drawer follows the latch along its slide, a lid about its hinge, the trolley
  wherever it is pulled, and **a case is free to turn about that point in every direction**,
  as on a hook; the latch transmits force and no torque, apart from a small friction torque on
  a package that is not reported. With `latch` 0 it opens at once.
- **Friction.** The floor, the furniture's boxes, the drawers' floors, the stools and chairs,
  the trolley's tray and the vessels share one friction coefficient, different in every episode, between 0.6 and 1.0, and not reported.

## The cart

`cart_model.py` is the cart exactly as the range simulates it, on top of `arm_model.py` (the
arm, the tool, the force/torque sensor and how joint targets reach the drives); read both.
In short:

- **Frames.** The cart's frame has its origin on the ground at the centre the cart turns
  about, x forward, y left. The arm's base is at `ARM_MOUNT` = (0.21, 0, 0.385) in it (the
  arm stands on the base's deck, so that it reaches the floor), the arm's axes along the
  cart's. The tool's **latch point** (the site `latch`) is 0.14 m out
  from the flange along the flange's axis; the tool is a rod of 12 mm radius ending in a
  ball of 10 mm radius centred on the latch point. In `TRAVEL_Q` the tool points straight
  down.
- **The base** is robosuite 1.5.2's Omron mobile base without its lift column. Its body
  collides as a box of 0.70 x 0.50 x 0.38 m centred 1 cm ahead of the turning centre. It
  moves in any direction and turns
  on the spot: an action gives it a velocity `[vx, vy, wz]` in the cart's own frame, which is
  clipped to 0.5 m/s, 0.5 m/s and 1.0 rad/s and may change by no more than 0.6 m/s^2,
  0.6 m/s^2 and 2.0 rad/s^2 (`Cart.command` has the exact rule, and how the velocity moves
  the base's position servos).
- **Collisions.** The base, the arm's links and the tool collide with the floor, the walls,
  the furniture's boxes (the stools' and chairs' too, which then move), the cabinets and their
  drawers, the trolley (which then moves), the stands, the vessels, their lids and the cases;
  the arm does not collide with itself or with the cart.
- **The camera** is on a mast at `CAM_MOUNT` = (0.33, 0.21, 1.42) in the cart's frame, looking
  forward and 40 degrees down: 240 x 180 pixels, 62 degrees vertical field of view. It takes
  a picture only when asked: with `look` 1 in an action it takes one at the end of that
  control step, if its last one is at least 10 steps (0.2 s) old, and the next observation
  carries it together with the cart's pose at that same instant. `take_picture` in
  `cart_model.py` is the exact picture: one ray through the centre of every pixel; `depth_mm`
  is the distance along the camera's axis in millimetres (0 where nothing is within 8 m);
  `rgb` is the hit surface's own colour times a shade between 0.45 and 1 (the three channels
  keep their ratios). A picture shows the lab and everything in it, and the cart's and
  the arm's own collision shapes where they are in view.

## What your controller is told and what it commands

The controller is asked for an action every 0.02 s (50 Hz) of simulated time; ten physics
steps pass between two requests. The observation is a JSON object:

| Field | Meaning |
|---|---|
| `t_s` | simulated time since the episode began |
| `countdown_s`, `tilt_limit_deg`, `shock_limit` | this episode's limits, the same for all its packages (see below) |
| `base` | `[x, y, yaw]` of the cart: its turning centre in the world (m) and its heading (rad), exact |
| `base_vel` | the rates of change of those three |
| `q`, `qd` | the arm's seven joint positions (rad) and velocities (rad/s) |
| `wrench` | the force/torque sensor, `[fx, fy, fz, tx, ty, tz]` (N, N m), in the frame of the site `ft` in `arm_model.py`, which is fixed to the tool: its z axis runs along the tool from the flange to the latch point. It is the force and torque the arm exerts on the tool and on whatever the tool holds: it includes the tool's own weight (0.3 kg) and a constant bias that is different in every episode (up to 0.4 N and 0.03 N m per axis), with noise of 0.06 N and 0.006 N m per reading (`read_wrench`) |
| `latched` | 1 while the latch holds a bar |
| `camera` | `null`, or the picture asked for in the previous action: `stamp_s`, `rgb` and `depth_mm`, each image as `{"shape", "dtype", "zlib_b64"}` (`decode` in `cart_model.py`) |
| `vessels` | the four stands' centres in the store, `[[x, y], ...]` (exact) |
| `packages` | how many packages are in this lab (2 or 3) |

The action is `{"base": [vx, vy, wz], "q": [seven joint position targets, rad], "latch": 0 or
1, "look": 0 or 1}`; `base` and `look` may be left out (zero).

## When a package is spilled, and when the episode is won

Every package can be spilled from the first instant, wherever it is (a drawer jerked open, or
a trolley started off hard, jolts what stands in it), until it is secured. This is evaluated
for each at every 2 ms physics step:

- **Tilt**: the surface of what is in the case leans against the case's top by more than
  `tilt_limit_deg` (5 degrees in every episode) for 0.06 s without interruption. When the
  package and its contents are at rest, that lean is the case's own tilt from the vertical;
  when the case is moved, its contents move too, as they will.
- **Shock**: with `v` the velocity of the package's centre of mass (its case and its contents
  together), the acceleration of a step is `|v - v_previous| / 0.002`; it is low-passed,
  `shock += (a - shock) * 0.002 / (0.012 + 0.002)` per step, and the package spills when
  `shock` exceeds `shock_limit` (2 m/s^2 in every episode).

One package spilled loses the episode. So does the clock: `countdown_s` (1200 s with two
packages, 1500 s with three) must not pass before the episode is won (below), every package
secured and the lab put right.

A package is **secured** at the first control step at whose end its case's centre is inside
the inner opening of the vessel of its own class and no more than 0.364 m above the ground
(resting on that vessel's floor), it moves slower than 1 cm/s, that vessel's lid is shut
(within 0.06 rad of its shut stop), and the latch holds neither that package nor that lid. From
then on it cannot spill. In a vessel of another class it is not secured.

The episode is **won** at the first control step, when the last package is secured or later,
at whose end every package still rests in its shut vessel, the lab is as it was and the latch
holds nothing: every drawer is within 1.5 cm of shut, and every stool and chair stands upright
within 0.10 m and 0.30 rad of where it stood at the start. Where the trolley ends up does not
matter. The episode ends there; a lab not put right when `countdown_s` passes loses it like a
package spilled.

## What is hidden in every episode

| Property | Range |
|---|---|
| The furniture: which pieces, where, at what angle, what stands on them; the floor's, the walls', the casework's and the worktops' textures | as described above |
| The cabinets: how many (4 or 5), in which runs, on which walls, their widths and heights | as described above |
| How many packages there are, where each is (a drawer, the floor, a top) and its class; which drawers hold harmless cases | as described above |
| Which vessel is of which class | as described above |
| The trolley's place and heading | as described above |
| The trolley's mass and how freely it rolls | not described |
| What is in each package: its weight, how the weight is spread, how its contents move | not described |
| Friction of the surfaces, the latch's friction torque on a package, the force/torque sensor's bias | as given above, or not described |

None of these is in any observation. The scored episodes are drawn from 64-bit seeds you do
not have.

## What you have

| Path | What it is |
|---|---|
| `/home/user/arm_model.py` | The arm as the range simulates it: the model (`add_arm`), the servos and how targets reach them, the force/torque reading. |
| `/home/user/cart_model.py` | The cart as the range simulates it, and nothing else: the model (`add_cart`), how an action reaches the base and the arm (`Cart`), the observation's format, the camera's pictures (`take_picture`), and `CartRange`, an empty range: the cart on open ground among four plain blocks (`reset(seed)`, `step(action)`). The grader simulates the cart with these exact two files. |
| `/home/user/range_check/check.py` | Runs your controller over the wire, as the grader does, on the empty range: protocol, reply times, repeated-`t` pairs, where the cart and the latch point went. No lab, no score. |
| `/home/user/helpers/hold_still.py` | A controller that only speaks the wire, holds the cart still and decodes a picture: boilerplate to copy, no control logic. |
| `/opt/lab_assets/` | The furniture kit: `manifest.json`, one folder per model with its parts (an OBJ mesh and a PNG texture each), the surface textures, `README.md`. |

Python is `python3` (MuJoCo 3.16, numpy, scipy; robosuite 1.5.2's model files, without that
package's own dependencies). There is no network. Whatever you want to try on a lab, a
drawer or a package you have to build yourself from this description.

## What you submit

`/home/user/submission/run.sh`, started as `bash run.sh` with `/home/user/submission/` as its
working directory, and whatever it needs beside it: when it is graded, nothing else of your
home directory exists, so a controller that uses `arm_model.py` or `cart_model.py` carries
its own copies in the submission folder (`/opt/lab_assets/` is there). It must print
`READY` on a line of its own and then answer one JSON object per line on stdout (stdout is
the wire: send anything else to stderr):

```
{"type": "reset"}                        -> {"ok": true}             a new episode begins
{"type": "act", "t": 0, "obs": {...}}    -> {"action": {"base": [...], "q": [...], "latch": 0, "look": 1}}
{"type": "close"}                        -> {"ok": true}, then exit
```

`t` is the step index inside the episode, starting at 0 after each `reset`. **A step can be
asked twice with the same `t`**, once with the real observation and once with a copy whose
joint readings `q` are shifted by up to 0.02 rad and whose cart pose `base` by up to 0.01 (m,
m, rad), in either order; only the reply to the real one is applied. Answer each from the
observation it carries, and advance whatever your controller remembers once per `t`, not once
per request. A step whose observation carries a picture is never asked twice. A controller
whose replies to more than half of these pairs do not differ is treated as not reading its
input and scores zero.

`READY` must arrive within 20 s of launch and every reply within 5 s. A missed deadline, a
crash or a malformed action aborts the episode, which then counts as not contained; the
controller is started again for the next episode (three restarts at most).

## How it is graded

After your session the grader runs your controller on 24 hidden live episodes, each a new
lab and set of packages drawn as described above. The score is the
**containment rate**: the fraction of the 24 episodes that are won (every package secured in
the shut vessel of its class and the lab left as it was). It is compared with the rate of a
careful reference controller, which wins nearly all of them, in about 1100 s of an episode
each (at most 1400 s).

An episode lasts up to 75,000 control steps (60,000 with two packages), and the grader has a
time budget of its own: it starts no new episode once 12000 s of wall-clock time have passed
since its first, and an episode it did not start counts as not won. Its own share is about two
minutes an episode plus 75 ms for every picture asked for. A controller that averages under
4 ms a step is safe.
