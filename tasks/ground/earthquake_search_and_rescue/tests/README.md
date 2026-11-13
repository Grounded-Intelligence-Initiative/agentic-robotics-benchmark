# earthquake_search_and_rescue: maintainers' notes

Version 0.1 (2026-11-12). Platform `ground` (a tracked UGV; the key comes with the task cards change, upstream PR #74).

## 1. The task

A tracked rescue robot (the SR-2 crawler) is set down on the street in front of a building an earthquake brought
down. The agent writes the robot's **autonomous** search-and-rescue program (`rescue.py`, `solve(robot)`): the radio
link drops inside the rubble, so nobody teleoperates it. On each of the 4 hidden sites the program has to search the
collapsed building with the robot's own sensors, get through what can be got through (lift a beam out of a doorway,
take a narrow slot at the narrow track gauge, climb a slab wedged in a doorway with the flipper technique, bridge a
strip of floor that dropped into the basement, crawl under dropped slabs with the mast down), find the survivor, see
them with a camera, report their position in the robot's map frame, put the survival kit within their reach and bring
the robot back to the deployment point. It must not press on the survivor and must not move load-bearing debris (a
crate a slab rests on).

Long-horizon character: the state accumulates (the map the program builds from the lidar and its odometry, the
openings tried, the debris it cleared, the robot's configuration), the route is unique (a spanning tree of the bays)
and every site has dead ends, decoys (a shop mannequin, a still-warm boiler, heaps of clothes) and the way back.

## 2. Files

| path | what |
|---|---|
| `rubblecell/plan.py` | the site plan from a seed: bays (standing, damaged, void, filled, stair), the spanning tree of openings, the route from the street to the survivor's void, the obstacles on the route and on dead ends, the survivor, the decoys, the start |
| `rubblecell/site.py` | geometry from the plan: walls with their openings, columns, ceilings and dropped slabs, the obstacles, furniture, rubble mounds (heightfield) dressed with chunk meshes, the street, the survivor's pocket; the reference route (waypoints) |
| `rubblecell/scene.py` | the MuJoCo model: textures, the painted heightfield, statics, movable debris, people, the robot and its kit, welds (grip, kit latch, props), the sun and haze |
| `rubblecell/robot.py` | the SR-2: tracks as belt loops of facets with conveyor surface velocity (`surfacevel`), flippers, telescopic gauge, mast and pan-tilt head, 5-axis arm with IK, gripper; the on-board servo loop |
| `rubblecell/sensors.py` | cameras (RGB-D), the lidar (four depth renders at the lidar's origin), the thermal camera (a segmentation render mapped to temperatures, optics blur, NETD noise), odometry with drift |
| `rubblecell/server.py` | the robot API, the grip rule, the safety watch (survivor contact, props, overturn, falls), the radio link, the visibility record, the score, the trajectory record |
| `reference/reference.py` | the reference program (privileged: the route, the obstacles, the survivor, the true pose) |
| `verify.py`, `cellkit/` | the grader (the arm tasks' machinery: lockdown, runner as `nobody`, no-network filter, the wire with program-time accounting) |
| `hidden/seeds.json`, `hidden/selection.json` | the fixed hidden set and its pre-registered selection rule and runs |
| `review/replay.py` | renders a mission video from a trajectory record (`/opt/arb-verify/out/traj_<tag>.npz` of a kept verifier sandbox) |
| `setup_payload/`, `given_hashes.json` | the grader's byte copy of the agent's givens and their hashes |

## 3. Scoring

Per site (rubblecell.server.score), on [0, 1]: **located 0.45** (the last survivor report that reached the team over
the radio link; full within 1.0 m of the torso's middle, linear to 0 at 3.0 m; only if a camera or thermal frame the
program took before that report showed the survivor from 6 m or closer: 40 or more pixels of them in a 160 x 120
segmentation of that view), **kit 0.25** (released, at rest within 0.8 m of the head or a hand, centre between z =
-0.45 and 0.75 m, not pressing on the survivor with more than 2 N), **returned 0.30** (upright, tilted < 60 deg,
within 3 m of the deployment point at the end, only when located > 0). instruction.md states every one of these
conditions (the 40-pixel and floor-level conditions were added on 2026-11-13, before the pull request; the grader did
not change). Voids (the site
scores 0): the robot, the kit or debris pressing on the survivor with > 60 N for 3 ticks; a prop moved > 5 cm after
the robot touched it (secondary collapse: the mission ends). The robot is lost when tilted > 70 deg for 2 s or below
z = -0.8 m (a floor gap): the mission ends. rescue_score = the mean over the sites; reward via the anchor.

## 4. Realism audit (the owner's rule: the agent gets what a real team's robotics engineer would have)

* Given: the robot's manual and API, the team's field guide, the client library, an example program, and the
  training ground (environment/training_ground: a service under its own account, mode 700; the test lanes and six
  training rubble sites; a debrief after each mission with the person's position in that mission's map frame, as an
  exercise debrief gives).
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

* Colour frames come from MuJoCo's classic renderer (OSMesa); the look (textures, haze, the robot's lamps) is
  procedural. A Filament render twin like the arm tasks' would look better and changes nothing measured.
* Tracks are conveyor surfaces on box facets; skid-steer turning is approximate (it slips and hops a millimetre).
  Drum arcs have low grip so the robot does not climb walls with its sprockets.
* Small debris on the floor and the chunks dressing the rubble mounds are visual only (the tracks ride on the
  mound's heightfield body); the lidar and the depth cameras see them.
* The survivor and the mannequin are built of primitives (capsules and ellipsoids).
* The radio link is a path-loss model through the planned walls and voids, not a ray-traced one.
