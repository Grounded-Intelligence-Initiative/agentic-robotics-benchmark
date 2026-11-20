# earthquake_search_and_rescue: maintainers' notes

Version 0.3 (2026-11-20; section 8 lists what changed since 0.1 of 2026-11-12). Platform `ground` (a tracked UGV).

## 1. The task

A tracked rescue robot (the SR-2 crawler) is set down on the street in front of a building an earthquake brought
down. The agent writes the robot's **autonomous** search-and-rescue program (`rescue.py`, `solve(robot)`): the radio
link drops inside the rubble, so nobody teleoperates it. On each of the 8 hidden sites the program has to search the
collapsed building with the robot's own sensors, get through what can be got through (lift a beam out of a doorway,
take a narrow slot at the narrow track gauge, climb a slab wedged in a doorway with the flipper technique, bridge a
strip of floor that dropped into the basement, crawl under dropped slabs with the mast down), find the survivor, see
them with a camera, report their position in the robot's map frame, put the survival kit within their reach and bring
the robot back to the deployment point. It must not press on the survivor (the robot's protective stop ends the
mission) and must not move load-bearing debris (a crate that carries one end of a slab section).

Long-horizon character: the state accumulates (the map the program builds from the lidar and its odometry, the
openings tried, the debris it cleared, the robot's configuration), the route is unique (a spanning tree of the bays)
and every site has dead ends, decoys (a shop mannequin, a still-warm boiler, heaps of clothes) and the way back.

## 2. Files

| path | what |
|---|---|
| `rubblecell/plan.py` | the site plan from a seed: bays (standing, damaged, void, filled, stair), the spanning tree of openings, the route from the street to the survivor's void, the obstacles on the route and on dead ends, the survivor, the decoys, the start |
| `rubblecell/site.py` | geometry from the plan: walls with their openings, columns, ceilings and dropped slabs, the obstacles, furniture, rubble mounds (heightfield) dressed with chunk meshes, the street, the survivor's pocket; the reference route (waypoints) |
| `rubblecell/scene.py` | the MuJoCo model: textures, the painted heightfield, statics, movable debris (the props' crates and slabs among it, free bodies), people, the robot and its kit, welds (grip, kit latch), the sun and haze |
| `rubblecell/robot.py` | the SR-2: tracks as belt loops of facets with conveyor surface velocity (`surfacevel`), flippers, telescopic gauge, mast and pan-tilt head, 5-axis arm with IK, gripper; the on-board servo loop |
| `rubblecell/sensors.py` | cameras (RGB-D: colour from `look.py`, depth from the sensor twin), the lidar (four depth renders at the lidar's origin), the thermal camera (a segmentation render mapped to temperatures, optics blur, NETD noise), odometry with drift |
| `rubblecell/look.py`, `twin.py`, `pbr.py`, `sky.py`, `decor.py` | the look (section 7): the render twin and the sensor twin, physically based texture sets, the sky and its image light, the city around the cordon |
| `rubblecell/server.py` | the robot API (every request's numbers checked finite), the grip rule, the safety watch (the protective stop on the survivor, props, overturn, falls), the radio link, the visibility record, the score, the trajectory record |
| `reference/reference.py` | the reference program (privileged: the route, the obstacles, the survivor, the true pose) |
| `verify.py`, `cellkit/` | the grader (the arm tasks' machinery: lockdown, runner as `nobody`, no-network filter, the wire with program-time accounting) |
| `hidden/seeds.json`, `hidden/selection.json` | the fixed hidden set of 8 and its pre-registered selection rule (version 2), candidates and runs; `selection_v1.json` is the retired first set's record |
| `review/select_set.py` | the selection rule's implementation (one candidate per run; `--assemble` applies the rule) |
| `review/replay.py` | renders a mission video from a trajectory record (`/opt/arb-verify/out/traj_<tag>.npz` of a kept verifier sandbox) |
| `setup_payload/`, `given_hashes.json` | the grader's byte copy of the agent's givens and their hashes; `setup_payload/training/submission_checks.py` is the code of the contract's static checks, the same file the agent has: `verify.py` loads it from the root-owned copy |

## 3. Scoring

Per site (rubblecell.server.score), on [0, 1]: **located 0.45** (the last survivor report that reached the team over
the radio link; full within 1.0 m of the torso's middle, linear to 0 at 3.0 m; only if a camera or thermal frame the
program took before that report showed the survivor from 6 m or closer: 40 or more pixels of them in a 160 x 120
segmentation of that view), **kit 0.25** (released, at rest within 0.8 m of the head or a hand, centre between z =
-0.45 and 0.75 m, not pressing on the survivor with more than 2 N), **returned 0.30** (upright, tilted < 60 deg,
within 3 m of the deployment point at the end, only when located > 0). instruction.md states every one of these
conditions (the 40-pixel and floor-level conditions were added on 2026-11-13, before the pull request; the grader did
not change). A void (the site scores 0): a prop or its slab moved > 5 cm after the robot, what it holds or other
debris touched either (secondary collapse: the mission ends). A protective stop: the robot, the kit or debris
pressing on the survivor with > 60 N for 3 ticks stops the robot for good and ends the mission, which keeps located
and kit as they stand, gets no return and at most 0.45 (`safety.stop_score_cap`). The robot is lost when tilted
> 70 deg for 2 s or below z = -0.8 m (a floor gap): the mission ends. rescue_score = the mean over the sites; reward
via the anchor (`full_at` 0.98).

## 4. Realism audit (the owner's rule: the agent gets what a real team's robotics engineer would have)

* Given: the robot's manual and API, the team's field guide, the client library, an example program, the grader's
  submission checks, and the training ground (environment/training_ground: a commissioning service under its own
  account, mode 700; the test lanes and six training rubble sites that the reference program completes; after each
  mission the robot's own log: how it ended, reports made and received, events, budgets; no position of the person,
  no error of a report, no score).
* Not given: the simulator's code, any site plan or map, the survivor's position on a graded site, the true pose
  (only odometry with drift), the hidden seeds, any score of a graded site.

## 5. The oracle and the anchor

The oracle installs `rescue.py` with one privileged call (`reference_run`) and the token. The reference program drives
the planned route with the true pose: a line follower along each opening's axis, the obstacle routines (beam:
fold the front flippers, stand 0.59 m off, grasp with a pitched tool, lift 0.30 m straight, back out 0.8 m, swing and
drop; step: front flippers 65 deg to the edge, then 20 / -40 deg; gap: stop 0.75 m short, square up, flippers -8 deg,
cross; narrow: gauge 0.30), looks at the survivor (mast camera + thermal), reports, picks the kit from its cradle,
places it 0.38 m from the nearest head / hand point clear of the body and the pocket's furniture, backs off, turns
and drives the route back. The anchor value is its measured score on the fixed hidden set (anchor.json `source`).

## 6. Known limitations

* Tracks are conveyor surfaces on box facets; skid-steer turning is approximate (it slips and hops a millimetre).
  Drum arcs have low grip so the robot does not climb walls with its sprockets.
* Small debris on the floor and the chunks dressing the rubble mounds are visual only (the tracks ride on the
  mound's heightfield body); the lidar and the depth cameras see them.
* The survivor and the mannequin are built of primitives (capsules and ellipsoids).
* The radio link is a path-loss model through the planned walls and voids, not a ray-traced one.

## 7. The look (cell 0.2.0)

The colour images come from MuJoCo's Filament renderer (physically based; Vulkan on Mesa's lavapipe, the CPU, in
the task image) on a render twin of the scene; depth, the lidar, the thermal image and the visibility record come from
MuJoCo's classic renderer (OSMesa) on a sensor twin. Both twins are compiled from a COPY of the scene's spec, with the
same bodies, joints and cameras: a frame copies the physics qpos into a twin and runs forward kinematics only. The
physics model is never rendered and never touched, and no random generator of the cell is drawn from.

* Render twin (`look.py`, `twin.py`): every surface has a physically based texture set (`pbr.py`: albedo, normal map,
  ambient occlusion / roughness / metallic, procedural from fixed seeds, made at image build into
  `/opt/rubblecell-look`); boxes and primitives are drawn as textured low-poly meshes with texture coordinates in
  metres; the heightfield is drawn by meshes per ground class; static geometry is merged into one mesh per material
  (a CPU rasteriser pays per drawn object). Filament's image light is not shadowed, so each static surface carries
  the share of the sky it sees in its material's occlusion (street 1, a bay 0.15, the upper storeys 0.3, a crawl void
  0.02); the robot is drawn in three such versions and each frame shows the one for where it stands. Exterior walls
  are drawn as an outer and an inner half. The cameras expose automatically by the light level where they stand
  (the robot manual says so). Lights: the sun (lowered to a late-afternoon light for the images only; the physics
  model's sun is not involved, and the thermal model reads the sun's strength only), the sky's image light and its
  backdrop (`sky.py`: a dusty sky with smoke columns, its cube maps written as KTX1 like Filament's cmgen does), the
  robot's two lamps, the emergency vehicles' beacons. No material has mirror reflectance (MuJoCo's Filament renderer
  draws a planar mirror pass of the whole scene per reflective material: ten times a frame's cost on the CPU).
* Decor (`decor.py`): the city around the cordon, drawn only, all of it where no part of the robot can reach (beyond
  the barriers, beyond the neighbours, or more than 3 m up: window frames and air-conditioning units on the upper
  storeys, power lines). It is in the sensor twin too, as one static body appended after every other body, so the
  sensor twin's first ids are the physics model's and the lidar, the depth and the thermal images see what the colour
  image shows (decor at the outdoor temperature).
* Evidence that the physics is unchanged: the compiled physics model's MJB bytes hash identically before any look
  code existed, after building it now, and after building and rendering the whole look, for every hidden site, the
  lanes and two training sites; the reference program's runs in the task image repeat the selection runs of cell
  0.1.2 sample for sample (see `hidden/selection.json`'s amendment for 0.2.0).
* Cost on 8 CPUs (the verifier's): the site and its look build in about 15 s; a 640 x 480 colour frame (2x
  supersampled, ambient occlusion, fog, bloom, soft shadows) renders in about 0.3 s, the mast camera's whole call
  (colour, depth, the visibility record, the encoding) takes about 0.9 s, the driving and gripper cameras about 0.5 s,
  after an episode's first frame (about 15 s: shaders and textures); the lidar about 0.2 s, the thermal camera 0.05 s.
* Replays (`review/replay.py`): the same look at video quality on a GPU, the storeys over the robot cut away.

## 8. What changed in version 0.3 (cell 0.3.0, 2026-11-20, the maintainers' fix round)

* **Load-bearing props are loaded.** A prop was a 4 kg crate with nothing on it (the 180 kg slab hung from a weld and
  touched only the robot): 40 N moved it 5 cm and voided the site. Now the slab section is a free body that lies on
  the crate and on a brick heap at the jamb; the crate carries 750-850 N of it and needs about 1000 N at its base to
  slide 5 cm; with the crate out, the slab comes down. A touch no longer moves it; the robot can still push it out
  (it pushes with more than its own weight: 5 cm after 15 s of pushing at 0.3 m/s, at once when rammed at 0.45 m/s
  or more). The rule counts the crate and its slab as one group. Furniture keeps clear of a prop (a keep-out).
* **The survivor: a protective stop instead of a void.** 60 N for 30 ms used to zero the site, with no signal the
  program could react to; the frozen reference tripped it on a candidate and on a training site. Now it is the robot's
  protective stop (section 3).
* **A request cannot fail the cell.** `step(n_ticks=Infinity)` raised inside the cell, which the grader answered by
  running the site again: a program could ask for a second run. Every request's numbers are now checked finite, a
  command's exception is `bad_params`, and a re-run uses the same run folder.
* **get_state.** The velocity and the gyro carry noise; the angular velocity is no longer rotated twice; reading the
  state draws from its own random stream (it used the odometry's). `pose.z` is still the true height above the
  deployment point's ground.
* **The contract is public and the same everywhere.** The static checks moved into
  `setup_payload/training/submission_checks.py`, which the agent has; instruction.md lists them; `run_mission.py`
  applies them, the read-only working directory, the no-network filter and the import check before it uses a mission.
* **The training ground is a commissioning ground**: no truth, no score, no debrief (section 4); its six rubble sites
  are admitted by the reference program under the selection rule.
* **The hidden set**: 8 sites, rule version 2 (candidates from OS entropy, registered before any run; a hollowing
  rule on the four techniques); `full_at` 0.98.
* **Session**: the sandbox's cores and memory and a "Session time" section in instruction.md, `session_time.sh` with
  a root-owned start stamp; `WORKDIR /` in the image; absolute paths in the training service's restart loop;
  `OPENBLAS_L2_SIZE=256` beside the kernel pin.
* **Left as it is**: the colour frames are not byte-identical between two fresh processes (tens of pixels of
  307,200; not the ambient occlusion, the soft shadows, the bloom, address-space randomisation or lavapipe's thread
  count); depth, thermal and lidar are. The reference program does not read images.
