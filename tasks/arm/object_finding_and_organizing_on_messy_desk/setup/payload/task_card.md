# Task card — object_finding_and_organizing_on_messy_desk

Fixed for the task; identical on every seed. Units: metres, radians, seconds.

## Frames and geometry
- Robot base frame = world frame. The Franka Panda base sits at the origin on the desk
  surface; the desk surface is the plane z = 0 (x in [-1.59, 1.59], y in [-0.81, 0.81]).
- Objects arrive on the reachable desk area x in [0.15, 0.78], y in [-0.62, 0.62], outside
  the containers and at least 0.36 m from the base (nearer in, a straight-down hand stops short of the desk).
  While a request is open, no instance of the object it names arrives.
- Four KLT containers standing on the desk, walls 0.12 tall, long side roughly along x. The red
  delivery container sits right in front of the robot base; the three class containers stand
  behind it, the middle one set back behind red and the outer two further forward. Every episode
  jitters each container by up to +/-0.02 in x, +/-0.008 in y and +/-5 degrees of yaw (seeded), so
  localise them from the cameras (the `info` command of the practice simulator reports the
  current poses):

| container | colour | nominal centre (x, y) | outer footprint (x, y) | interior half extents (x, y) | class |
|---|---|---|---|---|---|
| gray | gray | (0.44, -0.31) | 0.326 x 0.272 | (0.140, 0.120) | A |
| lightblue | light blue | (0.575, 0.0) | 0.326 x 0.272 | (0.140, 0.120) | B |
| blue | blue | (0.44, 0.31) | 0.326 x 0.272 | (0.140, 0.120) | C |
| red | red | (0.25, 0.0) | 0.237 x 0.272 | (0.102, 0.120) | requested objects |

  Interior half extents are in the container's own frame; inner floor at z = 0.0024 (a 1 cm rib
  runs along the base of each long wall); rim at z = 0.12.
  The open Franka hand is 0.205 m wide across the fingers and 0.063 m thick; the underside of the
  hand body is about 0.06 m above the fingertips (0.034 m above the tool centre point; the
  fingertips are 0.027 m below it). With the tool centre point below about 0.10 m inside a
  container the hand body is within the walls: with its finger axis along the container's long
  side it fits with its centre within about 3.4 cm of the container's centre line, and turned
  across the long side with about 1.3 cm of slack per side; a hand leaning away from the near
  wall, or a grasp high enough for the body to pass over the rim, reaches the rest of the floor.
  The open fingers are 8 cm apart, so the object itself may sit a little further out.
  "Inside a container" for scoring: the object's centre within the interior footprint (in the
  container's own frame) and its height between the floor − 0.02 and the rim + 0.12. "Placed"
  additionally needs the object at rest (speed < 0.08 m/s) and no finger touching it (the
  simulator's contact report). "On the table": not inside any container, centre below 0.16,
  placed. Below z = −0.2 an object is lost for the rest of the episode.

## Desk surface
The desk top is flat at z = -0.0014 for x < 0.809 (the whole reachable desk). Beyond that the
table's rounded rim rises 1.2 cm (z = +0.0104 for x in 0.839-0.854) and falls away past x = 0.90.
Along y the desk is flat out to |y| = 0.676.

## Classes (YCB models; the two instances of a model are scaled 0.92x and 1.00x of its base size)
Every model is scaled so that, in any resting pose, it offers a pinch of at most about 6.8 cm,
under the hand's 8 cm opening. Objects arrive standing upright on the desk (the scissors, the
marker and the clamps lying flat), their long side pointing away from the nearest container wall
within about 35 degrees. The initial piles are packed one object at a time on top of each other
near the container's centre line: the cans standing, everything else lying on its largest face, or on
its long edge when it is too wide to pinch lying flat (the larger cracker box).
- A: cracker box, sugar box, pudding box, gelatin box, wood block, foam brick
- B: master chef can, tomato soup can, tuna fish can, potted meat can, mustard bottle, bleach cleanser
- C: banana, bowl, mug, power drill, scissors, large marker, large clamp, extra large clamp, pitcher base

Requests name a model as "the <model name>" (for example `the tomato soup can`). At most
two instances of a model are on the desk at once; a request only ever names a model with
exactly one instance present, or one with none (an absent request).

## Cameras (four, 320 x 320, RGB 8-bit PNG and depth 16-bit PNG in millimetres)
- agentview: world pose xyz (1.35, 0.0, 0.95), looking down at the containers from the far side of the desk.
- left_view: world pose xyz (0.45, -1.15, 0.75), looking across the desk from the left.
- oblique: world pose xyz (1.25, -0.95, 0.85), looking at the containers from the front-left corner of the desk.
- eye_in_hand: attached to /World/Franka/panda_hand, offset (0.05, 0, 0.03) in the hand frame.
- Intrinsics (all four): focal 244.33 px, principal point (160, 160). Static poses as xyz + quaternion
  (w, x, y, z) in Isaac's world camera axes (+X forward, +Y left, +Z up): agentview
  (1.35, 0.0, 0.95 | 0.0, -0.3751, 0.0, 0.9270), left_view (0.45, -1.15, 0.75 | 0.6808, -0.1909, 0.1909, 0.6808),
  oblique (1.25, -0.95, 0.85 | 0.3981, -0.2464, 0.1119, 0.8765).
  Depth PNGs hold z-depth along the optical axis in millimetres. `{"cmd": "info"}` on the simulator
  (see below) returns the same matrices and poses.
- Hi-res on demand: a ninth action element (bitmask 1 = agentview, 2 = left_view, 4 = eye_in_hand,
  8 = oblique; 15 = all) adds `rgb_hires` / `depth_hires` to that step's observation: the same
  cameras at 1920 x 1080 (same poses and focal length, horizontal field of view unchanged, the
  vertical one cropped to 16:9; focal 1466 px, principal point (960, 540); `info` -> `hires` lists
  the matrices). The hi-res frames are rendered every step anyway and only encoded and sent when
  asked for, so the plain 10 Hz step is unaffected; a request costs about 0.1 s of extra
  wall-clock time per camera asked for (PNG encoding and transfer: about 3 MB per camera).

## Control
- 10 Hz control, 60 Hz physics (6 substeps per action).
- action = [dx, dy, dz, droll, dpitch, dyaw, grip, mark] (+ an optional ninth element, the hi-res
  camera bitmask, see Cameras); position deltas clipped to
  +/-0.05 m, rotation deltas to +/-0.25 rad (world-axis roll/pitch/yaw), applied to the
  tool centre point's current pose and tracked through inverse kinematics within the step
  (the joint set points are interpolated over the step's six physics substeps, so a 5 cm delta
  completes about 96% within the step, the rest in the next one); grip in [0, 1] maps to
  finger width 0.04 m (open) .. 0 (closed); mark: 0 nothing, 1 "done: delivered",
  2 "done: no such object" (one mark per request, ignored while no request is open).
- A delta whose IK solution is unreachable, or would flip the arm to another joint-space
  branch (a joint jump above 0.8 rad), is refused: the arm stays where it is for that step
  (the simulator's privileged state shows `ik_ok: false`). Stay inside the workspace box and
  move in small steps near its edges.
- Reachable workspace for the tool centre point: x in [-0.1, 0.85], y in [-0.75, 0.75], z in [0.028, 0.9]
  (the tool centre point sits between the finger pads; with the fingers open their tips touch the
  desk at z = 0.027, so z is floored there and commanding lower only presses the hand into the desk).
- Every episode starts with the tool centre point at (0.46, 0.0, 0.39), pointing straight down,
  fingers opening along the base y axis; `dyaw` rotates the hand about the world z axis.

## Running the simulator yourself
    /opt/isaac/bin/python3 /home/user/messy_desk_env.py
speaks JSON lines on stdin/stdout: `{"cmd":"info"}`, `{"cmd":"reset","seed":11,"horizon":2000}`,
`{"cmd":"step","action":[...]}`, `{"cmd":"lab","model":"006_mustard_bottle","instance":0,"pose":[0.5,0.0,0.06,0.0]}` (clear the desk and place one object; a `null` yaw sets it down the way arrivals are; the world state machine pauses until the next reset), `{"cmd":"close"}`. Replies carry `obs` (what your controller
will receive) and `state` (privileged ground truth for your own development only; the real
grader never sends it over the wire). `/home/user/sim_client.py` wraps this.
