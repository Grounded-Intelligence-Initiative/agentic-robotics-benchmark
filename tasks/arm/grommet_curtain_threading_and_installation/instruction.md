# Install a six-eyelet curtain

A Mobile FR3 Duo starts on the ground beside a residential window, facing the assembly area to the right of the window. A curtain with six steel eyelets and its separate rod lie on the ground. Two wall brackets are already mounted above the window.

Thread the rod through consecutive eyelets starting from either end of the curtain. Alternate the entry face at each eyelet. Carry the assembly to the window, raise the lift, lower the rod into both brackets, and withdraw both grippers. Keep the curtain threaded on the rod between the brackets.

The initial robot configuration is fixed. The rod and curtain share a sampled planar translation within ±0.10 m along each axis and a yaw offset within ±10°. The brackets remain fixed.

## Deliverable

Write a self-contained controller in `/home/user/submission/` with `run.sh` as its entry point. The verifier runs `bash run.sh` from that directory. Keep diagnostics on stderr. Stdout carries one protocol response per line.

Print `READY` when the controller starts. Then read JSON requests from stdin.

| Request | Reply |
|---|---|
| `{"type":"reset"}` | `{"ok":true}` after clearing episode state |
| `{"type":"act","t":0,"obs":{...}}` | `{"action":[...]}` with exactly 20 finite numbers |
| `{"type":"close"}` | `{"ok":true}` and exit |

The control period is 0.05 simulated seconds. The episode lasts 3,600 control steps (180 simulated seconds). Each request has an 8-second wall-clock reply deadline. A step may be queried more than once with the same `t`. Answer each request from the observation it carries. Repeated requests with the same `t` do not advance physical time.

## Observations

The observation has three fields.

| Field | Content |
|---|---|
| `rgb` | A dictionary of 8 camera streams, each a base64-encoded RGB PNG. The cameras are the two head eyes, both wrists, and the front, rear, left and right of the chassis. |
| `depth` | A dictionary of 7 camera streams, each a base64-encoded 16-bit grayscale PNG. Values are optical-axis depth in millimetres. Zero means no valid return. The head depth uses the left head optical frame. The wrists and all four chassis cameras have their own depth frames. |
| `robot_state` | Control time and robot feedback as described below. |

The scene is sampled at the control boundary for each observation. Camera resolution is 640 × 360 for the head and chassis and 424 × 240 for each wrist. The stream keys, calibration and valid depth ranges are in `/home/user/given/calibration.json`.

All arm arrays are ordered left arm joints 1–7 followed by right arm joints 1–7. Gripper arrays are ordered left, right.

| `robot_state` key | Meaning |
|---|---|
| `time_s` | Episode time in seconds |
| `arm_q_rad`, `arm_dq_rad_s` | Measured arm joint angles and velocities |
| `gripper_width_m` | Measured equivalent opening from each gripper's drive joint |
| `lift_height_m` | Measured lift position |
| `base_xy_yaw` | Applied chassis pose in the episode's initial chassis frame, in m, m, rad |
| `arm_target_rad`, `gripper_target_m`, `lift_target_m` | Joint, gripper and lift targets in effect at this observation |

The controller uses camera observations to locate the rod and curtain. Their simulator poses and the scoring state are private to the verifier.

## Actions

Send twenty **increments**, in this order. A positive gripper increment opens the gripper. A positive lift increment raises the carriage.

| Indices | Increments | Per-step limit |
|---|---|---|
| 0–6 | Left arm joint angles 1–7, rad | ±0.05 rad per joint |
| 7–13 | Right arm joint angles 1–7, rad | ±0.05 rad per joint |
| 14–15 | Left and right gripper opening, m | ±0.006 m each |
| 16 | Lift target height, m | ±0.014 m |
| 17–18 | Forward and left chassis displacement, m | 0.02 m total planar displacement |
| 19 | Chassis yaw increment, rad | ±0.05 rad |

The arm, gripper and lift increments are added to the targets reported in `robot_state`. Zero keeps the target in effect. The controller clips the resulting arm targets to joint limits, gripper targets to 0–0.085 m, and lift target to 0–0.827 m. The physics controller follows these targets over time.

For the chassis, positive forward moves along the current heading, positive left moves to the left of that heading, and positive yaw turns counterclockwise viewed from above. Negative values mean backward, right, and clockwise, respectively. A step with translation and yaw uses the heading at the start of the step for the whole translation. It **translates first, then turns**. For example, the last three numbers `[0.01, 0.0, -0.04]` move 1 cm forward, then turn 0.04 rad clockwise. The chassis position remains within ±0.25 m along both axes of its initial frame.

## Robot model and calibration

- `/home/user/given/mobile_fr3_duo.urdf` contains the kinematic tree of the simulated FR3 Duo. For each arm use `<side>_fr3v2_1_link0` as the arm base, `<side>_fr3v2_1_link8` as the flange, and `<side>_tcp` as the nominal grasp frame between the gripper fingers. Use the matching left or right joint names in the URDF. The base is planar and is controlled separately.
- `/home/user/given/calibration.json` gives the transforms from the chassis `base_link` to the lift carriage at zero height and from the carriage to each arm base. The URDF's `franka_spine_vertical_joint` gives the height-dependent transform.
- The calibration file gives the intrinsics and parent-to-camera transform for each stream. Optical frames use +X right, +Y down and +Z forward. Chassis coordinates use +X forward, +Y left and +Z up. All transforms are row-major 4 × 4 matrices, with `p_parent = T_parent_child @ p_child`. Camera frames attached to the lift or arms move with their URDF parents.

Run a short rollout on a public practice layout with `python3 /opt/curtain/practice.py /home/user/submission 120`. The practice run uses the same observations and action interpretation as evaluation. Adjust the number of steps as needed.

## Scoring

The final scene determines the score. Starting from each end of the rod, count the consecutive threaded eyelets starting at a curtain end, with adjacent eyelets and alternating faces. Stop at the first violation and keep the larger of the two counts. Each counted eyelet is worth 0.1, up to 0.6.

Installation adds 0.4 when both brackets support the rod, the grippers have withdrawn, and at least one eyelet remains threaded between the brackets. A full score requires all six correctly threaded eyelets and successful installation.
