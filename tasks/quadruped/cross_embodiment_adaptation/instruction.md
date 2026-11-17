# Cross-Embodiment Adaptation: scout, transfer, and dock

Write an unattended mission program that scouts an unmapped hall with a small robot,
then delivers a long rigid load with a heavier carrier. Both use real Go1 visual meshes
on stabilized planar MuJoCo bases. This is navigation and embodiment transfer, not
learned foot locomotion, articulated payload manipulation, or a hardware digital twin.

## The Hall

The enclosed hall is 7.6 m square. A 0.24 m partition meets its boundaries and has
three openings: a service aisle 1.22-1.38 m wide, a shortcut 0.86-0.94 m wide, and
a scout-only opening 0.46-0.50 m wide. A baffle beyond the shortcut makes a dogleg:
its middle corridor is 0.84-0.90 m wide. Two other crates obstruct the hall.
The short scout route does not certify that the loaded carrier can make its turns.
The service aisle supports a longer, collision-free, oriented route.

Layouts are rotated/reflected and their opening positions, partition offset, crate
positions, carrier start, delivery position, footprint, and drive response vary by
site. Do not assume that the shortest route, a single inflated circular map, or a
sequence of scout motions transfers to the carrier. The green parking mark identifies
the carrier start and its front. The blue delivery bay is 2.24 x 1.10 m, with a yellow
band at its front (+longitudinal) end. Both are paint, not obstacles. They are visible
only through the cameras; there is no always-visible beacon or goal coordinate.

## Bodies and Controls

Each action advances 0.2 s, integrated by MuJoCo in twenty 10 ms steps.

- Scout: a 0.31 m square collision footprint, about 1.08 kg. Send a single button:
  `[0]`, `[1/3]`, `[2/3]`, or `[1]`. The four buttons command the two
  positive/negative planar axes. Assignment and response gain are unknown per site.
  Its yaw is stabilized and fixed, and it can translate sideways. Steady speed is
  approximately 0.9-1.2 m/s. Scout contacts are allowed.
- Carrier: a 1.64-1.82 m long, 0.62-0.70 m wide rigid collision envelope, total mass
  7-10 kg. The rack is welded to the body. Its visible outer rails mark the full
  conservative filled rectangular footprint, including the open space inside the
  rack. Cyan marks the front; white marks the rear. It rotates with the body.
- Carrier actions are `[channel0, channel1]`, each in [-1, 1]. One channel sets
  signed longitudinal target speed; the other sets signed steering angle. Their
  order and signs are independently drawn for each site. Infer them from motion.
  `[0, 0]` always commands braking and centered steering; it does not stop instantly.
  There is no sideways command or turn-in-place command. Reverse is available.
- Carrier full target speed is 0.85-1.05 m/s, maximum steering 0.76-0.85 radians,
  effective wheelbase 0.56-0.62 m. Longitudinal response time is 0.28-0.50 s,
  steering lag 0.18-0.28 s and yaw lag 0.16-0.25 s. A supplied lateral stabilizer
  has 0.10-0.18 s response; slip and inertia remain. A traction circle with
  coefficient 0.10-0.16 limits planar acceleration, and traction also limits yaw
  response. Driving faster changes turn feasibility and braking distance.
- These are force/torque-driven planar constraints with native MuJoCo contacts,
  not an articulated gait or a claim about real Go1 tire/foot friction. Any
  carrier or payload contact with an obstacle causes an immediate terminal stop.

## Cameras and Observations

Each observation contains two 256 x 256 RGB perspective PNGs in `images`, ordered
front then rear. They are body-mounted cameras, not an external chase camera,
calibrated global top view, perfect localization sensor, or depth camera. Both have
85-degree vertical field of view and look 25 degrees below horizontal. Coordinates
relative to the stabilized body origin (x forward, y left, z up) are:

| body | front camera | rear camera | body origin above floor |
|---|---|---|---|
| scout | (0.10, 0, 0.27) m | (-0.10, 0, 0.27) m | 0.23 m |
| carrier | (0.30, 0, 0.40) m | (-0.30, 0, 0.40) m | 0.23 m |

Front looks along +x, rear along -x. Projection/occlusion are from the real rendered
scene. The camera masts and base stabilization are idealized; there is no claimed
hardware calibration. Texture, wall/crate boundaries, rack rails and colored end
marks provide visual evidence for mapping, local pose and footprint estimation.
The complete robot need not be visible in a single onboard frame. There are no
absolute XY/yaw fields, global visibility masks, coordinate conversions, semantic
masks, hidden-state overlays, or synthetic localization landmarks.

Other observation fields are `frame_id`, `phase` (0 scout, 1 carrier), `done`,
`steps_remaining`, `requests_remaining`, `bumper` (contact during the last action),
and `contact_stop`. Repeated `observe()` does not move the world. After switching,
localize the carrier in the scouted map; the two phases have different controls,
footprints and dynamics.

## Mission Rules

- At most 200 scout actions. Call `advance()` once, after at least 12 scout actions.
  It removes the scout and starts the carrier at the green parking mark.
- At most 260 carrier actions. Success requires the carrier center within 0.12 m
  of the delivery-bay center, heading within 7 degrees of its yellow-front
  orientation, speed below 0.055 m/s and yaw rate below 0.07 rad/s, for three
  consecutive action endpoints. Stop and settle; merely crossing the pad fails.
- Carrier contact or exhausting its actions ends the mission in failure.
- At most 640 requests (observe, act, advance) per mission. Scout contacts, route
  length, time and action count do not contribute extra penalties.

## Deliverable and Execution

Leave `/home/user/submission/run.sh` and its dependencies in that folder. Grading
starts `bash run.sh` once per mission. Connect to the Unix socket in `$ROBOT_SOCKET`
(default `/run/visual_robotics/world.sock`), drive the whole mission, and exit.
The public client at `/home/user/robot/client.py` and
`/opt/visual_robotics/client/client.py` provides `observe()`, `act(action)`,
`advance()`, `new_site()`, and `camera_image(obs, index)`. Its historical
`top_image(obs)` name is only a front-image decoding alias, not a world top view.

The verifier is a fresh, separate sandbox. Only the submission is restored, from
a read-only copy (regular files only, no symlinks or __pycache__, at most 2000 files
and 64 MB). The program runs as the unprivileged agent account, with a minimal
environment, fresh writable HOME/TMPDIR, and no network (only Unix sockets).
Use `/opt/venv/bin/python3`; numpy, scipy, Pillow, MuJoCo 3.16 and trimesh are
installed. Write only under HOME/TMPDIR. There are 2 CPUs, 4 GB shared with the
service, no GPU, 300 s of program time excluding service request processing, and
1200 s total wall time per mission.

After a mission ends, the program has 10 s to exit. Limits kill its process tree;
the mission counts as it stands, and the other missions still run. A surviving
program process voids the entire run. A missing run.sh scores zero.

## Score and Commissioning

Your program runs on eight fixed hidden sites, never the commissioning sites.
The score remains the fraction of successful deliveries, relative to the reference
value. No partial credit is added for mapping, motion, heading or closeness.

Commissioning uses a fresh random development site on each `new_site()`, with
the same mechanics and observations. No truth, seed, score, debrief or private
service source is available. Graded missions refuse `new_site()`. Start a fresh
commissioning site before running your program; only one mission runs at a time.
The MCP tools robot_observe, robot_act, robot_advance and robot_new_site use the
same service as your program.

Your development session lasts **2 hours**, with 2 CPUs, 4 GB, no GPU and no network.
Keep a complete run.sh in place early. Do not try to read protected files, change
the service or recover site seeds. Very large tool inputs (roughly 25-30k tokens)
may be dropped by the harness.
