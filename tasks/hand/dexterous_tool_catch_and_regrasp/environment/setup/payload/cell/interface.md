# Tool-catching cell TC-1: hand, sensors and controller interface

Units: metres, radians, seconds, newtons. World frame: z up, the floor at z = 0. The thrower stands about 2 m away
along +x and tosses towards the hand. Quaternions are (w, x, y, z).

## The hand and its wrist

- Hand: a Wonik Allegro right hand (16 joints: index `ff`, middle `mf`, ring `rf`, thumb `th`, four joints each). Its
  kinematics and collision geometry are those of the MuJoCo Menagerie model (`wonik_allegro/right_hand.xml`, BSD-2).
  Its actuators are the cell's catching variant: position servos with stiffness 2 N m/rad, a torque limit of
  0.8 N m per joint and low joint damping; from open to a closed fist the fingers take about 60 ms.
- Wrist: the hand sits on a servoed 6-DoF wrist (a positioner with a spherical wrist). You command a target pose; the
  servo moves towards it with at most 2.0 m/s and 20 m/s² linear and 8 rad/s and 80 rad/s² angular, and tracks the
  resulting path with a stiff impedance (it gives a little at an impact). The target position is clipped to the
  workspace box x in [-0.45, 0.45], y in [-0.45, 0.45], z in [0.70, 1.45].
- Hand frame H (the palm frame): origin at the base of the fingers on the palm's centre line; **+X** the palm normal
  (the side the fingers close towards), **+Y** towards the index finger (and the thumb's base), **+Z** along the
  fingers. The palm itself spans about Z in [-0.095, 0] and Y in [-0.057, 0.057], its surface at X ≈ 0.011.
  The power-grasp point, where a closed fist holds a handle, is at H = (0.03, 0, -0.01).
- Home pose (at the start of every throw): wrist at (0, 0, 1.00), palm up (X_H = +z), fingers towards the thrower
  (Z_H = +x), so Y_H = -y; fingers open.

Finger joints, in the order of every 16-vector (`finger_q`, `finger_qd`, `finger_tau`, the `fingers` action):

| index | joint | range (rad) | | index | joint | range (rad) |
|---|---|---|---|---|---|---|
| 0 | ffj0 (spread) | -0.47 .. 0.47 | | 8 | rfj0 (spread) | -0.47 .. 0.47 |
| 1 | ffj1 | -0.196 .. 1.61 | | 9 | rfj1 | -0.196 .. 1.61 |
| 2 | ffj2 | -0.174 .. 1.709 | | 10 | rfj2 | -0.174 .. 1.709 |
| 3 | ffj3 | -0.227 .. 1.618 | | 11 | rfj3 | -0.227 .. 1.618 |
| 4 | mfj0 (spread) | -0.47 .. 0.47 | | 12 | thj0 (opposition) | 0.263 .. 1.396 |
| 5 | mfj1 | -0.196 .. 1.61 | | 13 | thj1 | -0.105 .. 1.163 |
| 6 | mfj2 | -0.174 .. 1.709 | | 14 | thj2 | -0.189 .. 1.644 |
| 7 | mfj3 | -0.227 .. 1.618 | | 15 | thj3 | -0.162 .. 1.719 |

Targets outside a joint's range are clipped to it. Positive flexion closes a finger towards +X of the hand.

## Timing

- The cell runs at **50 Hz** (one control step = 20 ms of cell time). Every step it sends one observation and expects
  one action.
- **Sensing/actuation latency: 4 steps.** The action you send in reply to the observation of step k is applied from
  step k + 4 on (80 ms later); until then the actions you sent before keep acting. At the start of every throw the
  four actions in the pipe hold the home pose with open fingers.
- **The world keeps moving.** The cell does not wait for you. Your reply time for a step is measured from the moment
  the observation is written to you until your reply has been read. A reply within one control period (20 ms) is
  on time. A slower reply acts as if it had arrived at the step it arrived at: ceil(time / 20 ms) - 1 further steps
  run with your previous action, and the next observation you receive is the one after them (the observations in
  between are never shown to you; the observation's `step` and `t` tell you where the cell is). A reply later than
  5 periods (100 ms) is discarded: 5 steps run with your previous action. The 4-step latency comes on top.

## A run of throws

The cell works in runs of throws. Every throw starts with the hand back at its home pose, fingers open, and a tool
held still by the thrower at the release point, in view of both cameras. After a short wait (a few tenths of a
second) the thrower tosses it towards the hand: a flight of well under a second, the tool tumbling end over end and
rolling a little. A throw lasts **9 s (450 steps)** of cell time, or ends at once when the tool drops (touches the
floor or falls below z = 0.4). Then the next throw starts.

What varies from throw to throw and is never told: which of the four tools comes (the observation names it; a run
whose length is a multiple of four brings every tool equally often, in random order), the
release point and the toss, the spin, the tool's exact size (a few percent), how its mass is spread between handle
and metal parts (the head of a hammer is heavier on some), and the friction between tool and fingers. The
observation names the tool from the start of the throw; **the grip zone wanted for this throw (`zone`) is announced
2.0 s after the throw starts** (after any catch). The job always wants the other grip: if the hand holds the tool at
that moment, the zone announced is the one whose centre is farther from where the hand holds it.

## Observation

One JSON object per step:

| key | meaning |
|---|---|
| `throw` | index of the throw in the run (0, 1, ...) |
| `step`, `t` | step index and time (s) since the throw started |
| `tool` | `"hammer"`, `"screwdriver"`, `"wrench"` or `"spatula"` |
| `zone` | `null` until t = 2.0 s, then `"end"` or `"choke"` (see `tools.md`) |
| `wrist_pos`, `wrist_quat` | measured pose of the hand frame H in the world (origin, orientation) |
| `wrist_linvel`, `wrist_angvel` | its linear and angular velocity, world frame |
| `finger_q`, `finger_qd`, `finger_tau` | the 16 joint angles, velocities and actuator torques |
| `touch` | 21 numbers: total normal contact force (N) on each link, in this order: palm; ff base, proximal, medial, distal, tip; mf (same five); rf (same five); th (same five) |
| `ft_force`, `ft_torque` | the wrist force/torque sensor between the wrist and the palm: the force and moment the wrist applies to the hand (palm, fingers and anything they hold), at the origin of H, in the H frame (N, N m). **It is not tared**: at rest it carries the hand's own weight (about 0.64 kg) and that weight's moment, which shift with the finger posture and the hand's orientation. Measured at the home pose (palm up), open hand: force (6.32, 0.00, 0.00) N, torque (0.00, -0.07, -0.05) N m; closed fist: force (6.19, -0.13, -0.15) N, torque (-0.01, -0.18, 0.00) N m. A held tool adds its weight and the moment of its weight about H's origin, plus contact and inertial loads |
| `img_side`, `img_head` | the two camera images, 192 x 144 RGB (see below) |

On the wire each image is `{"dtype": "uint8", "shape": [144, 192, 3], "b64": <base64 of the raw RGB bytes,
row-major, top row first>}`; decode with
`np.frombuffer(base64.b64decode(d["b64"]), np.uint8).reshape(d["shape"])`.

## Action

```json
{"wrist_pos": [x, y, z], "wrist_rotvec": [rx, ry, rz], "fingers": [16 joint targets]}
```

`wrist_pos` and `wrist_rotvec` are the target pose of the hand frame H in the world: its origin and its orientation
as a rotation vector (axis times angle, rad) taking world axes to H's axes. The servo limits above apply.
`fingers` are the 16 joint position targets. Every number must be finite.

## Cameras

Two fixed RGB cameras, 192 x 144 pixels each, no lens distortion, calibrated in `calibration/cameras.json`:
position, look-at point and vertical field of view. The camera looks from `pos` towards `look_at` with world +z as
"up"; in MuJoCo's convention its frame has x to the right of the image, y up in the image and z pointing backwards
(away from the scene); the focal length in pixels is f = (144 / 2) / tan(fovy / 2), the principal point the image
centre, and row 0 is the top of the image.

- `side`: looks across the flight path from the -y side.
- `head`: looks over the robot's shoulder from behind and above the hand, towards the thrower.

`calibration/background_side.png` and `calibration/background_head.png` are each camera's view of the empty cell
(no tool, no hand, no wrist), taken with the same renderer and lighting the cell runs with.

## What counts (per throw)

A throw earns nothing if the tool drops, or if over the last 2 s of the throw it is not held (touching at least
three links of the hand for at least 90 % of that time). A held tool earns a catch credit of 0.1 plus up to 0.9 for a
**functional grasp at the end of the throw**, the product of four factors measured over the last 2 s (the hold
window, t from 7.0 s to 9.0 s):

1. **Grip zone**: the power-grasp point of the hand, H = (0.03, 0, -0.01), lies on the tool's axis within the
   requested zone of the handle (`tools.md`); the credit falls linearly to 0 over 2 cm outside it.
2. **Working end up**: the tool's axis from butt to working end points up: full credit within 20° of vertical,
   none beyond 45°.
3. **Across the palm**: the tool's axis lies along the hand's Y axis (a power grasp across the palm, not a pinch
   along the fingers): full credit within 30°, none beyond 55°.
4. **Steady**: over the hold window neither the tool moves in the hand nor the hand moves in the world by more
   than 1 cm (full credit), falling to none at 3 cm.

Where the hand is in the workspace during the hold, and how it got the tool there, is up to you.
