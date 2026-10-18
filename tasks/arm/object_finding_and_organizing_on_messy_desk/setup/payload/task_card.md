# Task card — object_finding_and_organizing_on_messy_desk

Fixed for the task; identical on every seed. Units: metres, radians, seconds.

## Frames and geometry
- Robot base frame = world frame. The Franka Panda base sits at the origin on the desk
  surface; the desk surface is the plane z = 0 (x in [-1.59, 1.59], y in [-0.81, 0.81]).
- Objects arrive on the reachable desk area x in [0.15, 0.78], y in [-0.62, 0.62], outside
  the bins and at least 0.22 m from the base.
- Four KLT bins (outer footprint 0.297 x 0.198, walls 0.19 tall, long side roughly along x)
  in a loose row. Nominal centres are at x = 0.44; every episode jitters each bin by up to
  +/-0.04 in x, +/-0.01 in y and +/-7 degrees of yaw (seeded), so localise them from the
  cameras (the `info` command of the practice simulator reports the current poses):

| bin | colour | nominal centre y | class |
|---|---|---|---|
| gray | gray | -0.39 | A |
| lightblue | light blue | -0.13 | B |
| blue | blue | 0.13 | C |
| red | red | 0.39 | requested objects |

  Interior half extents (x, y) = (0.135, 0.085) in the bin's own frame; floor at z = 0.0156;
  rim at z = 0.19.
  "Inside a bin" for scoring: the object's centre within the interior footprint and its
  height between the floor and 0.12 above the rim, at rest (speed < 0.02 m/s).

## Classes (YCB models; the two instances of a model are scaled 0.95x and 1.05x of its base size)
- A: cracker box, sugar box, pudding box, gelatin box, wood block, foam brick
- B: master chef can, tomato soup can, tuna fish can, potted meat can, mustard bottle, bleach cleanser
- C: banana, bowl, mug, power drill, scissors, large marker, large clamp, extra large clamp, pitcher base

Requests name a model as "the <model name>" (for example `the tomato soup can`). At most
two instances of a model are on the desk at once; a request only ever names a model with
exactly one instance present, or one with none (an absent request).

## Cameras (224 x 224, RGB 8-bit PNG and depth 16-bit PNG in millimetres)
- agentview: world pose xyz (1.10, 0.0, 0.70), looking at the bins from the far side of the desk.
- left_view: world pose xyz (0.45, -1.15, 0.75), looking across the desk from the left.
- eye_in_hand: attached to /World/Franka/panda_hand, offset (0.05, 0, 0.03) in the hand frame.
- Intrinsics (all three): focal 171.03 px, principal point (112, 112). Static poses as xyz + quaternion
  (w, x, y, z) in Isaac's world camera axes (+X forward, +Y left, +Z up): agentview
  (1.10, 0.0, 0.70 | 0.0, -0.3722, 0.0, 0.9281), left_view (0.45, -1.15, 0.75 | 0.6808, -0.1909, 0.1909, 0.6808).
  Depth PNGs hold z-depth along the optical axis in millimetres. `{"cmd": "info"}` on the simulator
  (see below) returns the same matrices and poses.

## Control
- 10 Hz control, 60 Hz physics (6 substeps per action).
- action = [dx, dy, dz, droll, dpitch, dyaw, grip, mark]; position deltas clipped to
  +/-0.05 m, rotation deltas to +/-0.25 rad (world-axis roll/pitch/yaw), applied to the
  tool centre point's current pose and tracked through inverse kinematics within the step
  (a 5 cm delta is completed in one step inside the workspace); grip in [0, 1] maps to
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
`{"cmd":"step","action":[...]}`, `{"cmd":"lab","model":"006_mustard_bottle","instance":0,"pose":[0.5,0.0,0.06,0.0]}` (clear the desk and place one object; the world state machine pauses until the next reset), `{"cmd":"close"}`. Replies carry `obs` (what your controller
will receive) and `state` (privileged ground truth for your own development only; the real
grader never sends it over the wire). `/home/user/sim_client.py` wraps this.
