# Twin contract

This is the technical contract of the twin: what you deliver, the rules the files must follow, the standard
simulation, the pick rules, and how the twin is assessed. Every number below is the number the code uses
(`twin_gate.py`, `twin_sim.py` with `twin_sim_config.json`, `twin_camera.py`, `hand_instrument.py` with
`instrument_config.json`, and the grader's configuration). `site_brief.md` explains what the twin is for;
`hand_card.md` describes the hand and the instrument's volumes.

## 1. What you deliver

`/home/user/submission/` holds:

| path | content |
|---|---|
| `scene.xml` | the twin: one MuJoCo MJCF file (MuJoCo 3.16), no includes |
| `objects.json` | which bodies are which things (section 4) |
| `assets/` (any layout) | the mesh files and textures that `scene.xml` references, by relative path |

- Only `scene.xml`, `objects.json` and the files `scene.xml` references are read. Every other file is ignored, but
  it still counts toward the folder caps, so keep renders, notes, caches and scratch work outside `submission/`.
- Folder caps: at most 5000 entries (files and folders) and 300 MB in total.
- File caps: `scene.xml` at most 5 MB, `objects.json` at most 1 MB, each mesh
  file at most 50 MB, each PNG at most 64 MB and 4096 px a side;
  at most 2000 referenced files, 2000 meshes and
  500 textures; at most 20000 XML elements, nested at most
  64 deep; at most 3000000 mesh faces and
  3000000 mesh vertices in all files together.
- Formats: meshes `.stl`, `.obj`, `.msh` (STL binary); textures `.png` only. A referenced path is relative (to the submission
  folder, or through `meshdir` / `texturedir` / `assetdir`), has no `..` and no empty component, no symbolic link on
  the way, at most 240 characters, and only the characters A-Z a-z 0-9 `_ . + - ( ) /` and spaces. An OBJ's `mtllib` must be a plain file name.

## 2. Frames and units

- World: z up, metres, kilograms, seconds, radians in the compiled model. Where the world origin is and which way x
  points are yours to choose.
- The initial state `qpos0` is the photographed state: every body as written in `scene.xml`. There are no keyframes
  and no joint `ref`.
- The support's top frame (`twin_sim.support_frame`): among the support's boxes, the one whose upward face (the face
  whose outward normal has the largest world z) has the largest area; origin = the centre of that face, z = its outward
  normal, x = world x projected on the face (world y projected on it instead when world x lies within
  10 deg of the face's normal, either way), y = z cross x. The standard simulation's gravity
  points along -z of this frame, and the assessment lays this frame's z = 0 on the real board's top (section 8).
- The direction frame of the pick sheet: in the real scene, x = the photo's line of sight (the survey camera's optical
  axis) projected on the board, pointing away from the camera, y = z cross x, z up; `dir_deg` turns counter-clockwise
  from x seen from above. When you rehearse on your twin (`check_twin.py --rehearse`), the same frame comes from your
  `photo_cam` (`twin_sim.rehearsal_frame`); without `photo_cam`, or when `photo_cam` looks straight down (its line of
  sight has no part along the top plane), it is your support top's own x, with a warning. The assessment does not use
  your camera: it registers your twin to the real scene (section 8).
- Lengths in the pick instrument and its files are in mm, in the frame it plans in.

## 3. The support and the things

The support:

- one body named `support`: a top-level body without joints (static), with at least one collision geom, every
  collision geom of it (its own and those of bodies below it) a box. Its visual geoms are free. Of the real board
  only the plane of its top is used: the assessment lays your support's top frame on it (section 8), and the real
  board's outline and what lies below it play no part. Your support's boxes are simulated, though: the standard
  settle (section 6) runs on them. A thing that is not held and has nothing under it falls, and a thing that moves
  more than 300 mm, or faster than 3 m/s, within the
  settle's first 2 s blows the settle up (G0 = 0); a thing that only slides or tips is
  planned where the settle leaves it. So every thing that is not held must rest on your support's boxes or on another
  thing.

The things, by key (the keys `objects.json` maps your bodies to):

| key | the thing | requests on it |
|---|---|---|
| `wine_glass` | wine glass | yes |
| `teapot` | teapot | yes |
| `cup` | cup | yes |
| `tin` | tin | yes |
| `toy_remote` | toy remote | yes |
| `tube` | tube | yes |
| `candy_box` | candy box | yes |
| `fork` | fork | yes |
| `small_bottle` | small bottle | yes |
| `clog` | clog | none (an obstacle only) |

- Each thing is one top-level body with exactly one free joint (bodies below it are welded to it and belong to it),
  with 1 to 96 collision geoms. A geom collides when its `contype` or
  `conaffinity` is non-zero; every other geom is visual: it must be massless (density 0 or mass 0) and plays no part.
- Collision shapes: box, sphere, capsule, cylinder, ellipsoid or mesh. A mesh geom collides as the convex hull of its
  vertices (MuJoCo's own hull is used), so anything hollow or looped (a vessel's inside, a handle's opening) must be
  made of several convex pieces. The instrument sees each collision geom as a convex polyhedron: a mesh as its hull,
  a box as its eight corners, a sphere, capsule, cylinder or ellipsoid as a polyhedron around it, at most
  0.1 mm larger (`twin_sim.pieces_from_model`).
- Plausibility, per thing (all its bodies together; M its mass, C its centre of mass, V the volume of the union of its
  collision geometry, L its largest extent, d_max the largest distance from C to its collision geometry):
  density M / V within 10-3000 kg/m^3; sliding friction within
  0.1-1.5; smallest principal inertia at least 0.0001 x M x L^2;
  largest principal inertia at most M x d_max^2 (with a 1 % tolerance); C inside the convex hull of
  the collision geometry; at most 50000 collision-mesh vertices per thing; at most
  2 m across. These rules are checked thing by thing: a thing that breaks one of them or the
  count of collision geoms above, or that has a visual geom with mass, is excluded: it counts as absent, without
  failing the twin; a twin with no thing left fails the gate.
- One collision mesh is checked on its own, and a failure there fails the gate (the whole twin, not only its thing):
  a collision mesh with more than 20000 vertices after the compile, or with more than
  three times that many as written (counted before the compile), or without a convex hull.
- A thing your twin does not contain (or that is excluded) is absent: its requests are declined before any planning
  (`declined_by` "absent"), never charged.
- Held things: `candy_box` (candy box), `fork` (fork), `small_bottle` (small bottle) and `clog` (clog). They are kept still in the settle (as the true scene needs them to be); every other thing is free. The pick sheet's "held" list names the same keys.

What is removed before any simulation: every top-level free body that `objects.json` does not map, and every colliding
geom that belongs neither to the support nor to a mapped thing (a floor, a table, a wall). The gate's report lists
them, with a warning; this does not fail the gate. Cameras, lights, visual geoms and the static bodies that hold them
stay.

The camera `photo_cam` is optional. It serves only your own overlays and the direction frame of your rehearsals. If
your twin has a camera named `photo_cam`, it must be `mode="fixed"` on the world or a static body, with
`resolution="1096 852"`, decodable pinhole intrinsics with fx and fy between
500 and 5000 px and the principal point inside the image; otherwise the gate fails. In
OpenCV terms (pixel (u, v) = (column, row), (0, 0) the centre of the top-left pixel, camera axes x right, y down,
z forward), with W x H the resolution (`twin_camera.encode` writes these for you):

```
sensor form:  focalpixel     = (fx, fy)
              principalpixel = ((W - 1)/2 - cx, (H - 1)/2 - cy)      # the image centre MINUS the principal point
fovy form:    fy = (H/2) / tan(fovy/2),  fx = fy,  (cx, cy) = ((W - 1)/2, (H - 1)/2)
camera frame: R_cv = cam_xmat @ diag(1, -1, -1),  p = cam_xpos       # MuJoCo cameras look along their -z, y up
              x_cam = R_cv^T (x_world - p)
```

The survey camera's own numbers are in `camera_sheet.json` (its pose is not recorded).

## 4. `objects.json`

```
{"format": "hand-twin-objects/1", "bodies": {"<body name>": "<thing key>", ...}}
```

1 to 10 bodies; every key one of the ten above, no key twice; the support is not listed. A
listed body that is not a top-level body with a free joint fails the gate.

## 5. Allowed MJCF

`scene.xml` may contain only these elements, under these parents, with these attributes (anything else fails the
gate, with the reason):

| element | allowed under | attributes |
|---|---|---|
| `<mujoco>` | (root) | model |
| `<compiler>` | `<mujoco>` | angle, assetdir, autolimits, discardvisual, eulerseq, fitaabb, inertiafromgeom, inertiagrouprange, meshdir, texturedir, usethread |
| `<option>` | `<mujoco>` | ccd_iterations, ccd_tolerance, cone, gravity, impratio, integrator, iterations, jacobian, ls_iterations, ls_tolerance, noslip_iterations, noslip_tolerance, sdf_initpoints, sdf_iterations, solver, timestep, tolerance |
| `<flag>` | `<option>` | actuation, autoreset, clampctrl, constraint, contact, damper, diagexact, energy, equality, eulerdamp, filterparent, frictionloss, fwdinv, gravity, invdiscrete, ipc, island, limit, midphase, multiccd, nativeccd, refsafe, sensor, spring, warmstart |
| `<statistic>` | `<mujoco>` | center, extent |
| `<visual>` | `<mujoco>` | (none) |
| `<global>` | `<visual>` | azimuth, bvactive, cameraid, elevation, ellipsoidinertia, fovy, glow, ipd, linewidth, offheight, offwidth, orthographic, realtime |
| `<quality>` | `<visual>` | numquads, numslices, numstacks, offsamples, shadowsize |
| `<headlight>` | `<visual>` | active, ambient, diffuse, specular |
| `<map>` | `<visual>` | actuatortendon, alpha, fogend, fogstart, force, haze, shadowclip, shadowscale, stiffness, stiffnessrot, torque, zfar, znear |
| `<scale>` | `<visual>` | actuatorlength, actuatorwidth, camera, com, connect, constraint, contactheight, contactwidth, forcewidth, framelength, framewidth, frustum, jointlength, jointwidth, light, selectpoint, slidercrank |
| `<rgba>` | `<visual>` | actuator, actuatornegative, actuatorpositive, bv, bvactive, camera, com, connect, constraint, contactforce, contactfriction, contactgap, contactpoint, contacttorque, crankbroken, fog, force, frustum, haze, inertia, joint, light, rangefinder, selectpoint, slidercrank |
| `<default>` | `<mujoco>`, `<default>` | class |
| `<asset>` | `<mujoco>` | (none) |
| `<mesh>` | `<asset>`, `<default>` | class, face, file, inertia, name, normal, refpos, refquat, scale, smoothnormal, texcoord, vertex |
| `<texture>` | `<asset>` | builtin, colorspace, file, gridlayout, gridsize, height, hflip, mark, markrgb, name, nchannel, random, rgb1, rgb2, type, vflip, width |
| `<material>` | `<asset>`, `<default>` | class, emission, metallic, name, reflectance, rgba, roughness, shininess, specular, texrepeat, texture, texuniform |
| `<layer>` | `<material>` | role, texture |
| `<worldbody>` | `<mujoco>` | (none) |
| `<body>` | `<worldbody>`, `<body>`, `<frame>` | axisangle, childclass, euler, name, pos, quat, xyaxes, zaxis |
| `<frame>` | `<worldbody>`, `<body>`, `<frame>` | axisangle, childclass, euler, name, pos, quat, xyaxes, zaxis |
| `<inertial>` | `<body>` | axisangle, diaginertia, euler, fullinertia, mass, pos, quat, xyaxes, zaxis |
| `<joint>` | `<body>`, `<default>` | axis, class, group, limited, name, pos, range, type (type free only) |
| `<freejoint>` | `<body>` | group, name |
| `<geom>` | `<worldbody>`, `<body>`, `<frame>`, `<default>` | axisangle, class, conaffinity, condim, contype, density, euler, fitscale, friction, fromto, gap, group, margin, mass, material, mesh, name, pos, priority, quat, rgba, shellinertia, size, solimp, solmix, solref, type, xyaxes, zaxis (type box, capsule, cylinder, ellipsoid, mesh, plane, sphere) |
| `<site>` | `<worldbody>`, `<body>`, `<frame>`, `<default>` | axisangle, class, euler, fromto, group, material, name, pos, quat, rgba, size, type, xyaxes, zaxis |
| `<camera>` | `<worldbody>`, `<body>`, `<frame>`, `<default>` | axisangle, class, euler, focal, focalpixel, fovy, ipd, mode, name, pos, principal, principalpixel, projection, quat, resolution, sensorsize, target, xyaxes, zaxis |
| `<light>` | `<worldbody>`, `<body>`, `<frame>`, `<default>` | active, ambient, attenuation, bulbradius, castshadow, class, cutoff, diffuse, dir, directional, exponent, intensity, mode, name, pos, range, softness, specular, target, type |

Inside `<default>` an element carries only these attributes: `<camera>` and `<light>` those of the table above but `class`, `name` and `target`; `<geom>`, `<joint>`, `<material>` and `<site>` those of the table above but `class` and `name`; `<mesh>` only `inertia` and `scale`. Not allowed, among others:
`<actuator>`, `<attach>`, `<composite>`, `<connect>`, `<contact>`, `<custom>`, `<deformable>`, `<equality>`, `<exclude>`, `<extension>`, `<fixed>`, `<flex>`, `<flexcomp>`, `<hfield>`, `<include>`, `<instance>`, `<key>`, `<keyframe>`, `<lengthrange>`, `<model>`, `<numeric>`, `<pair>`, `<plugin>`, `<replicate>`, `<sensor>`, `<size>`, `<skin>`, `<spatial>`, `<tendon>`, `<text>`, `<tuple>`, `<weld>`. Joints: free joints only, one per thing, no stiffness, damping, armature, friction loss or
margin. No compile warning is allowed. Totals: at most 120 bodies, 3000 geoms,
1500 collision geoms; each coordinate of every body's position relative to its parent at most
100 m in absolute value, and every geom size at most 100 m. Also
checked: at most one `<compiler>` element; a `<texture>`'s `type` one of `2d`, `cube` or `skybox`, its `builtin` one of
`checker`, `flat`, `gradient` or `none`, its `width` and `height` between 0 and 4096; a `<mesh>`'s `inertia` one of
`convex`, `exact`, `legacy` or `shell`; `<global>`'s `offwidth` and `offheight` between 1 and 8192; at most
1200000 numbers in each of an inline mesh's `vertex`, `normal`, `texcoord` and `face`. Textures and
their references are removed before compiling (appearance plays no part).

## 6. The standard simulation

Every simulation of your twin uses `twin_sim.py` with `twin_sim_config.json`, whatever `scene.xml` sets:

- `<option>`: timestep 0.001 s, integrator `implicitfast`, cone `elliptic`, impratio
  10, solver `Newton` (100 iterations, 50
  line-search iterations, tolerance 1e-08), 5 noslip iterations; gravity
  9.81 m/s^2 along -z of the support's top frame; everything else of `mjOption` at MuJoCo's
  defaults (no wind, medium or overrides), and every flag at its default except `autoreset`, which is disabled.
- Contacts: MuJoCo never tests two geoms of the same thing (its bodies are welded together) or two geoms of the
  support (all fixed to the world); every other pair of colliding geoms (two things, or a thing and the support)
  touches with condim 4, solref (0.004, 1), solimp (0.95, 0.99, 0.001, 0.5, 2) and friction
  (0.6, 0.005, 0.0001) (sliding, torsional, rolling), whatever the twin gives.
- Removed: joint stiffness and damping, friction loss, armature, gravity compensation, surface velocity, adhesion and
  fluid forces.
- The settle: 2 s from `qpos0`, held things kept still. It blows up when the state becomes
  non-finite, MuJoCo reports a warning, or within the first 2 s a thing moves faster than
  3 m/s or further than 300 mm; a blow-up fails G0.

## 7. The pick sheet and the pick instrument

The pick sheet (`pick_sheet.json`) lists 342 requests: for each of the
9 things that may be picked, the rules `body` (body pinch), `top` (top pinch), `rim` (rim pinch), `hook` (handle hook); `body`, `top` and `rim` once per
direction (12 directions, every 30 deg, `dir_deg`), `hook` once per side
(-1 and 1, `side`). The ids run from R0001 in that order. Every request is listed alike; which are scored, and
which of those are credited (section 8), is not said.

The instrument (`hand_instrument.py`; its numbers in `instrument_config.json`, the rules' fixed numbers in its
`rule_constants` block; the volumes in `hand_card.md`) is the definition; in words. Lengths are in mm, in the frame
the instrument plans in: z up, the board's top at z = 0, `dir_deg` turning counter-clockwise from x seen from above
(section 2). A thing is seen as point samples of its surface plus an inside test; on a twin the samples are taken on
every triangle of each convex piece's hull: a square lattice of 1 mm steps in the
triangle's plane (laid out from an end of its longest edge) and points along its three edges at most
1 mm apart, the corners included (samples inside another piece of the same thing
are dropped).

- The pads. A pinch uses two pads: the finger pad, 20 x 25 x
  21 mm, and the thumb pad, 23 x 36 x
  28 mm (thickness along the closing direction x width x height), each with a column
  60 mm high above it. A pad's face starts 10 mm from the target's surface
  (6 mm when a pad starts inside a vessel) and closes by up to 15 mm
  past it. A `body` or `top` pinch pad is planned with its lowest face at least
  2.5 mm above the board's top (its centre height is set higher where that needs it);
  a `rim` pad is placed from the rim's top at its depth (below), with no such clearance. Since a pad whose lowest face
  is below the board's top at execution is a stop, a pad stops at every placement that lowers the hand by more than
  the height its lowest face was planned above the board's top; for a rim pad on a low vessel that height can be
  small, or below zero (then the pad stops even at the centre placement).
- `body`: the d of a request is the direction `dir_deg`. Each pad has its own centre height: the mean height of the
  target's samples, or higher where that pad's floor clearance needs it (the two pads differ in height, so on a low
  thing they sit at different heights). A pad's band is the target's samples within half that pad's height of its
  centre height and within half the pad's width of the vertical plane along d through the target's centroid (here
  and below, the mean of the target's samples); each band must hold at least
  4 samples. The finger pad's
  face stands the start gap beyond its band's farthest sample along d (the finger comes from the side `dir_deg` points
  to), the thumb pad's face the start gap beyond its band's farthest sample along -d; the grip width between those two
  samples must lie between 2 and 80 mm.
- `top`: the same pinch with the pads' centre height 12 mm under the target's
  highest sample (each pad's own, higher where that pad's floor clearance needs it), the vertical plane through the centre of the target's
  samples in the finger pad's height band there (at least 5 of them); the
  same for every thing.
- `rim`: the opening's centre is the mean of the target's samples between 3 and
  9 mm under its highest sample (at least
  30 of them). On the azimuth `dir_deg` from that centre the finger pad goes
  inside, its centre 8 mm under the top, its face the inside start gap short of the inner
  wall (the target's nearest sample on that side within the pad's width and above the pad's lowest face); the thumb pad
  goes outside, its centre 11.5 mm under the top, its face the start gap beyond the outer wall
  (the farthest such sample). Each side needs at least 3 samples; the inside
  pad must leave 2 mm between its back and the opening's centre; the wall may
  be at most 25 mm thick.
- `hook`: the bar is a 20 mm wide, 25 mm tall section. All the target's
  samples (the whole thing, not only a handle) are projected along each horizontal axis, every 10 deg over half a turn, onto a vertical raster
  of 1 mm cells with a free border of 5 mm (a cell that
  holds a sample is occupied); gaps between samples are closed (2 closing
  steps). An opening is a connected region of empty cells that does not reach the border and in which a window of
  29 x 34 cells fits wholly at some cell: the bar's section grown by 4 mm on every
  side, each of its two sides counted in cells as its length over the cell size, rounded down, plus one. Its rim is
  the occupied cells within 12 steps of the opening (12 mm over
  the cell size, rounded), a step going from a cell to one that shares an edge with it; the rim must hold at least
  5 samples of the target. Each opening gives one site. The site is,
  among the opening's cells at which the window fits, the one farthest from the opening's boundary; its clearance is
  that distance; the bar runs through it along the axis, from 60 mm behind the rim's material (the hand's side)
  to 15 mm beyond it; side 1 enters along the axis direction (the axis angle measured from x,
  less than half a turn), side -1 against it. The sites of every axis are tried from the largest clearance down: a
  site whose bar's lowest face would be less than 1 mm above the board's
  top is skipped, and the first whose bar keeps the margins is planned. A planned hook is done when
  at least 5 samples of the target lie in the box right above the bar: as
  wide as the bar, 30 mm tall, along the bar over the rim's material and
  2 mm beyond it at each end.
- Margins at the start pose: the pads and their columns, or the bar, keep 4 mm from every other
  thing and 1 mm from the target.
- Only these volumes are judged: the pads with their columns, or the bar, at the start pose.
- Touch: a volume comes within t of a thing when a sample of the thing lies inside the volume grown by t along each
  of the volume's face normals, or the volume's centre lies inside the thing. Touching is coming within
  0.5 mm.
- Planning (on a twin): a request is declined when its target is absent from the twin (before any planning:
  `declined_by` "absent"), when its bands or grip width do not qualify (`width`), when no rim or through-opening
  qualifies (`no_opening`), or when the pads, columns or bar break the margins (`pads`, `column`, `bar`).
  `hand_instrument.plan_explain` gives `declined_by` and the blocking thing.
- Execution (in the real scene), at each of 9 placements (the planned volumes shifted by the
  centre and the eight corners of +-1.5 mm): a stop when a pad, a column or the bar touches anything
  real or lies inside it, or when a pad's or the bar's lowest face is below the board's top; else a pinch is done
  when, for both pads, the nearest sample
  ahead of the pad's face (inside the face's outline and at most the pad's start gap plus the over-travel ahead)
  belongs to the target, and a hook is done when the box above the bar holds the target as said; else a miss.

## 8. How the twin is assessed

`twin_score = G1 x G0 x Use`.

- **G1 submission integrity**: the submission is locked when the session ends; anything written later, or a folder
  beyond the folder caps of section 1 (5000 entries, 300 MB, a file counted at its
  size or at the space it takes on disk, whichever is larger), gives G1 = 0. Every other cap of section 1 is the
  gate's (G0); the gate checks the folder caps as well, with the files' sizes.
- **G0 validity**: `twin_gate.py` passes and the standard settle does not blow up; else G0 = 0.
- **Step 1, settle**: the standard settle of section 6 on the gate's model (excluded and unmapped bodies removed);
  the settled state is the one that is planned on.
- **Step 2, registration**: your support's top frame (section 2) is laid on the real board's top plane. Each thing
  of your twin gets a reference point in that frame: (x, y) = the mean of the centres of its footprint's cells (a
  cell of a 1 mm grid with a cell corner at the frame's origin is in the footprint
  when its centre lies inside the projection of one of the thing's collision pieces on the top plane) and z = the height of
  its highest collision point. The real scene's points are the same points of the team's reference twin, settled, in
  the real board's frame. A yaw psi, a shift (tx, ty) and one overall scale s (clamped to
  [0.8, 1.25]) are fitted in closed form, q = s
  Rz(psi) p + (tx, ty, 0), p a point of your twin and q the real one, on the things both scenes have
  (the clog, which is never picked, included): psi from the (x, y) points about their means, then s as
  the least-squares scale for that yaw with the (x, y) points about their means and the heights as they are (from the
  top plane), clamped to the range, then (tx, ty) as the least-squares shift for the clamped scale (the real points'
  mean (x, y) minus s Rz(psi) times yours). So where your support's top-face centre lies in the plane changes
  nothing; the heights count from the top plane. A thing's residual is the distance in
  space between its mapped point and the real one (the height included); while the worst residual exceeds
  25 mm and more than 4 things remain, the worst is
  dropped and the fit repeated. Fewer than 4 things within
  25 mm: Use = 0. The fit (the scale included) is applied to your whole twin, the
  dropped things included (they stay in the twin and their requests are planned and executed like the others): an
  overall scale error inside that range is forgiven; beyond it the scale stops at the range's end and the rest of
  the error stays.
- **Step 3, picks**: every scored request is planned on the registered twin and executed in the real scene at every
  placement (section 7): declined = 0 and no charge, a miss = 0, done = 1 on a credited request and 0 on
  the others (Use, below), a stop is charged on every scored request. Lifts:
  every scored request, body pinches included, is judged geometrically (no lift trial is run).
- **Use**: requests are grouped in units, a thing and a rule. Some scored requests are credited: they can be done
  with a careful twin (the team's reference twin does them at every placement). The other scored requests are scored
  only for their stops: the team's reference twin does not do them at any placement, a pick done on one of them earns
  nothing, a stop on one of them is charged like any other, and nothing earns more on them than declining them. A
  request's done share and stop share are the fractions of its 9 placements at which it is done
  and at which it stops. For a unit, done = the mean done share over its credited requests (a unit without a
  credited request has no done term), stop = the mean stop share over all its scored requests. The units with a credited request share a total weight of 1. Every unit with a
  credited request weighs the same, 1 / their number, except that no thing (its units with a credited request
  together) weighs more than the larger of 15 % and 1 / n of the total weight, n being the number of things that have a unit with a credited request; a thing above that is set to it, its excess is spread over the
  things below it in proportion to their units with a credited request, and this is repeated until no thing is above
  it; a thing's units with a credited request share its weight equally. A unit without a credited request carries
  only its stops, at 1 / the number of units with a credited request (the weight of such a unit before the cap); it
  adds to no thing's weight. Use = max(0, sum over the units of weight x (done - c x stop)), with c = 2.
- Which requests are scored, and which of them are credited, is not said. Among the requests that are not scored
  are those the photograph cannot decide (a thing's hidden side, a neighbour seen too little).
- What gives zero: G1 or G0 failing; fewer than 4 things registered; stops that
  outweigh the picks done (Use is clipped at 0); a twin that declines everything.
- The team's own reference twin scores 1 (the full mark). Reported metrics: `twin_score`, `g1_integrity`, `g0_validity`, `use`, `registered`, `done_share`, `stop_share`, `declined_share`. Their meaning:
  `twin_score` the score; `g1_integrity` G1; `g0_validity` G0; `use` Use; `registered` 1 when the registration holds, else 0; `done_share` the sum over the units with a credited request of weight x done; `stop_share` the sum over all units of weight x stop (so Use = max(0, `done_share` - c x `stop_share`)); `declined_share` the sum over the units with a credited request of weight x the mean declined share over its credited requests; `use` and the three shares are reported as 0 unless G1, G0 and the registration hold, and `registered` as 0 when G1 is 0.

## 9. Tools in your home folder

| path | what it is |
|---|---|
| `check_twin.py` | the gate, the settle (`--settle`), the rehearsal of the pick sheet on your twin (`--rehearse`, `--ids`, `--lift`), the overlay through `photo_cam` on the photo (`--overlay out.png`), `--json` |
| `twin_gate.py` | the gate: `gate(folder)` returns the report, the physics model and the roles |
| `twin_sim.py`, `twin_sim_config.json` | the standard simulation, the support's frames, the convex pieces |
| `twin_camera.py` | the camera conventions: `encode`, `decode`, `camera_xml`, `project` |
| `hand_instrument.py`, `instrument_config.json` | the pick instrument: `enumerate_requests`, `plan`, `plan_explain`, `execute`, `rehearse` |
| `pick_sheet.json`, `camera_sheet.json` | the requests and the camera's calibration |

`check_twin.py --rehearse --lift` adds the lift trial (`hand_instrument.lift`) of every planned body pinch, at the
centre placement, in your settled twin, with the pads' friction at 0.3 and the pick
sheet's held things welded. The lift trial is offered for your own rehearsal; in the assessment (section 8),
every scored request, body pinches included, is judged geometrically (no lift trial is run).
