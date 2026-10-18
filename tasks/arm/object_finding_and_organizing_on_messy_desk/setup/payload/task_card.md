# Task card — object_finding_and_organizing_on_messy_desk

Fixed for the task; identical on every seed. Units: metres, radians, seconds.

## Frames and geometry
- Robot base frame = world frame. The Franka Panda base sits at the origin on the desk
  surface; the desk surface is the plane z = 0 (x in [-1.59, 1.59], y in [-0.81, 0.81]).
- Objects arrive on the reachable desk area x in [0.15, 0.78], y in [-0.62, 0.62], outside
  the bins and at least 0.22 m from the base.
- Four KLT bins (outer 0.297 x 0.198 x 0.146, long side along x), centres at x = 0.46:

| bin | colour | centre y | class |
|---|---|---|---|
| gray | gray | -0.375 | A |
| lightblue | light blue | -0.125 | B |
| blue | blue | 0.125 | C |
| red | red | 0.375 | requested objects |

  Interior half extents (x, y) = (0.135, 0.085); floor at z = 0.012; rim at z = 0.146.
  "Inside a bin" for scoring: the object's centre within the interior footprint and its
  height between the floor and 0.12 above the rim, at rest (speed < 0.02 m/s).

## Classes (YCB models; the two instances of a model are scaled 0.9x and 1.1x of its base size)
- A: cracker box, sugar box, pudding box, gelatin box, wood block, foam brick
- B: master chef can, tomato soup can, tuna fish can, potted meat can, mustard bottle, bleach cleanser
- C: banana, bowl, mug, power drill, scissors, large marker, large clamp, extra large clamp, pitcher base

Requests name a model as "the <model name>" (for example `the tomato soup can`). At most
two instances of a model are on the desk at once; a request only ever names a model with
exactly one instance present, or one with none (an absent request).

## Cameras (224 x 224, RGB 8-bit PNG and depth 16-bit PNG in millimetres)
- agentview: world pose xyz (1.10, 0.0, 0.70), looking at the bins from the far side of the desk.
- left_view: world pose xyz (0.45, -1.15, 0.75), looking across the desk from the left.
- eye_in_hand: attached to /World/Franka/panda_hand, offset (0.05, 0, 0.04) in the hand frame.
- Intrinsics: `{"cmd": "info"}` on the simulator (see below) returns the 3x3 matrices.

## Control
- 10 Hz control, 60 Hz physics (6 substeps per action).
- action = [dx, dy, dz, droll, dpitch, dyaw, grip, mark]; position deltas clipped to
  +/-0.05 m, rotation deltas to +/-0.25 rad, applied to the tool centre point in the base
  frame through inverse kinematics; grip in [0, 1] maps to finger width 0.04 m (open) .. 0 (closed);
  mark: 0 nothing, 1 "done: delivered", 2 "done: no such object" (one mark per request,
  ignored while no request is open).
- Reachable workspace for the tool centre point: x in [-0.1, 0.85], y in [-0.75, 0.75], z in [0.01, 0.9].

## Running the simulator yourself
    /opt/isaac/bin/python3 /home/user/messy_desk_env.py
speaks JSON lines on stdin/stdout: `{"cmd":"info"}`, `{"cmd":"reset","seed":11,"horizon":2000}`,
`{"cmd":"step","action":[...]}`, `{"cmd":"lab","model":"006_mustard_bottle","instance":0,"pose":[0.5,0.0,0.06,0.0]}` (clear the desk and place one object; the world state machine pauses until the next reset), `{"cmd":"close"}`. Replies carry `obs` (what your controller
will receive) and `state` (privileged ground truth for your own development only; the real
grader never sends it over the wire). `/home/user/sim_client.py` wraps this.
