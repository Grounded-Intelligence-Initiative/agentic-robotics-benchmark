# Mechanical

> The humanoid robot form factor, load capability, joint layout, motion limits, actuation, and mechanical design.

- Section: Humanoid › Overview › System Tour

## Explore the Design Interactively

Interactive 3D model (opens in a browser): the humanoid robot CAD Viewer.

The public humanoid robot repository provides the mechanical design files.

> [!WARNING]
>
> **Outdated content below — update in progress**
>
> Only the images and descriptions on this **Mechanical System Tour** page, from **Structural Architecture** onward, are outdated. We are updating them to reflect the current design, including replacing older illustrations showing actuated toes. The rest of the manual, including the assembly instructions, is up to date.

## Structural Architecture

The humanoid robot is organized into the head, torso, pelvis, left and right arms, and left and right legs. These modules form the load-bearing frame and define the primary mechanical interfaces between body assemblies.

## Form Factor

| Characteristic | Specification |
| - | - |
| Height | 1.2 m (3.94 ft) |
| Mass | 35 kg (77 lb) |
| Powered degrees of freedom | 25 |
| Legs | 6 DOF per leg |
| Arms | 5 DOF per arm |
| Body | 1 waist-yaw DOF |
| Head | 2 neck DOF: yaw and pitch |
| Structural materials | 7075 aluminum and MJF PA12 nylon |



## Load Capability

| Activity | Rated load | Peak load |
| - | -: | -: |
| Squat | 5 kg | Not specified |
| Bicep curl | 5 kg per arm | 15 kg per arm |
| Front raise | 5 kg per arm | 15 kg per arm |
| Lateral raise | 6 kg per arm | 18 kg per arm |

Rated loads describe sustained operation for the published activity. Peak loads are short-duration limits rather than continuous operating targets.

## Powered Joint Layout

| Region | Joint axes | Powered joints |
| - | - | -: |
| Each leg | Hip pitch, hip roll, hip yaw, knee, ankle pitch, ankle roll | 6 |
| Each arm | Shoulder pitch, shoulder roll, shoulder yaw, elbow, wrist yaw | 5 |
| Waist | Yaw | 1 |
| Neck | Yaw, pitch | 2 |
| **Total** | | **25** |

For locomotion, the two neck joints are locked to use a 23-DOF body model.

## Joint Direction and Limits

The signed values below use the robot model coordinate convention. Mirrored left and right joints can use opposite signs for the same physical movement.

The frame convention is blue for `z`, red for `x`, and green for `y`.



### Actuator ID Mapping



> [!NOTE]
>
> **Interactive joint-limit visualization coming soon**
>
> An interactive visualization will be added so you can move every joint and test its limits. Until then, use the signed limits in the tables below.

### Legs

| Joint | Left range (rad) | Left axis | Right range (rad) | Right axis |
| - | -: | - | -: | - |
| Hip pitch | -2.09 to 1.00 | `0 1 0` | -1.00 to 2.09 | `0 -1 0` |
| Hip roll | -0.79 to 0.79 | `1 0 0` | -0.79 to 0.79 | `1 0 0` |
| Hip yaw | -0.79 to 0.79 | `0 0 -1` | -0.79 to 0.79 | `0 0 -1` |
| Knee | 0.00 to 1.50 | `0 1 0` | -1.50 to 0.00 | `0 -1 0` |
| Ankle pitch | -0.35 to 0.35 | `0 1 0` | -0.35 to 0.35 | `0 -1 0` |
| Ankle roll | -0.10 to 0.10 | `-1 0 0` | -0.10 to 0.10 | `-1 0 0` |

### Arms

| Joint | Left range (rad) | Left axis | Right range (rad) | Right axis |
| - | -: | - | -: | - |
| Shoulder pitch | -3.14 to 0.87 | `0 1 0` | -0.87 to 3.14 | `0 -1 0` |
| Shoulder roll | -1.57 to 0.00 | `-1 0 0` | 0.00 to 1.57 | `-1 0 0` |
| Shoulder yaw | -1.57 to 1.57 | `0 0 -1` | -1.57 to 1.57 | `0 0 -1` |
| Elbow | 0.00 to 2.44 | `0 -1 0` | -2.44 to 0.00 | `0 1 0` |
| Wrist yaw | -3.14 to 3.14 | `0.766 0 -0.64` | -3.14 to 3.14 | `0.766 0 -0.64` |

### Waist and Neck

| Joint | Range (rad) | Axis |
| - | -: | - |
| Waist yaw | -1.57 to 1.57 | `0 0 1` |
| Neck yaw | -1.57 to 1.57 | `0 0 1` |
| Neck pitch | -0.79 to 0.79 | `0 1 0` |

## Actuation and Mechanical Power

| Joint roles | Actuator family | Rated torque | Peak torque |
| - | - | -: | -: |
| Hip pitch and waist yaw | `RJ-8867` | 40 Nm | 120 Nm |
| Hip roll and shoulder pitch | `RJ-6381` | 30 Nm | 90 Nm |
| Hip yaw and shoulder yaw | `RJ-5378` | 20 Nm | 60 Nm |
| Knee and shoulder roll | `RJ-5669` | 25 Nm | 75 Nm |
| Elbow, wrist yaw, and neck | `RJ-5660` | 12 Nm | 36 Nm |

Rated torque is the continuous actuator output under normal thermal conditions. Peak torque is available only for short events.

### Full Actuator Specifications

The table below maps every physical actuator ID to its joint role and model parameters. The parallel ankles use physical actuators A and B, so their joint-space pitch and roll limits are defined by the ankle mechanism rather than by one actuator alone.

| ID | Physical role | Model | Motion range (rad) | Velocity limit (rad/s) | Kt | Armature | Rated torque (Nm) | Peak torque (Nm) | Static friction | Dynamic friction |
| -: | - | - | - | -: | -: | -: | -: | -: | -: | -: |
| 1 | `left_hip_pitch_joint` | `RJ-8867` | -2.09 to 1.00 | 12.57 | 2.75 | 0.095625 | 40.0 | 120.0 | 0.70 | 0.050 |
| 2 | `left_hip_roll_joint` | `RJ-6381` | -0.79 to 0.79 | 3.98 | 7.15 | 0.11 | 30.0 | 90.0 | 0.20 | 0.020 |
| 3 | `left_hip_yaw_joint` | `RJ-5378` | -0.79 to 0.79 | 5.45 | 5.70 | 0.038 | 20.0 | 60.0 | 0.70 | 0.050 |
| 4 | `left_knee_joint` | `RJ-5669` | 0.00 to 1.50 | 12.25 | 2.53 | 0.0339552 | 25.0 | 75.0 | 0.70 | 0.020 |
| 5 | `left_ankle_A` | `RJ-5660` | See ankle mechanism | 9.32 | 1.81 | 0.0565056 | 40.0 | 145.4 | 0.40 | 0.015 |
| 6 | `left_ankle_B` | `RJ-5660` | See ankle mechanism | 9.32 | 1.81 | 0.0565056 | 17.0 | 57.6 | 0.40 | 0.015 |
| 7 | `right_hip_pitch_joint` | `RJ-8867` | -1.00 to 2.09 | 12.57 | 2.75 | 0.095625 | 40.0 | 120.0 | 0.70 | 0.050 |
| 8 | `right_hip_roll_joint` | `RJ-6381` | -0.79 to 0.79 | 3.98 | 7.15 | 0.11 | 30.0 | 90.0 | 0.20 | 0.020 |
| 9 | `right_hip_yaw_joint` | `RJ-5378` | -0.79 to 0.79 | 5.45 | 5.70 | 0.038 | 20.0 | 60.0 | 0.70 | 0.050 |
| 10 | `right_knee_joint` | `RJ-5669` | -1.50 to 0.00 | 12.25 | 2.53 | 0.0339552 | 25.0 | 75.0 | 0.70 | 0.020 |
| 11 | `right_ankle_A` | `RJ-5660` | See ankle mechanism | 9.32 | 1.81 | 0.0565056 | 40.0 | 145.4 | 0.40 | 0.015 |
| 12 | `right_ankle_B` | `RJ-5660` | See ankle mechanism | 9.32 | 1.81 | 0.0565056 | 17.0 | 57.6 | 0.40 | 0.015 |
| 13 | `left_shoulder_pitch_joint` | `RJ-6381` | -3.14 to 0.87 | 3.98 | 7.15 | 0.11 | 30.0 | 90.0 | 0.20 | 0.020 |
| 14 | `left_shoulder_roll_joint` | `RJ-5669` | -1.57 to 0.00 | 12.25 | 2.53 | 0.0339552 | 25.0 | 75.0 | 0.70 | 0.020 |
| 15 | `left_shoulder_yaw_joint` | `RJ-5378` | -1.57 to 1.57 | 5.45 | 5.70 | 0.038 | 20.0 | 60.0 | 0.70 | 0.050 |
| 16 | `left_elbow_joint` | `RJ-5660` | 0.00 to 2.44 | 9.32 | 1.81 | 0.0282528 | 12.0 | 36.0 | 0.40 | 0.015 |
| 17 | `left_wrist_yaw_joint` | `RJ-5660` | -3.14 to 3.14 | 9.32 | 1.81 | 0.0282528 | 12.0 | 36.0 | 0.40 | 0.015 |
| 18 | `right_shoulder_pitch_joint` | `RJ-6381` | -0.87 to 3.14 | 3.98 | 7.15 | 0.11 | 30.0 | 90.0 | 0.20 | 0.020 |
| 19 | `right_shoulder_roll_joint` | `RJ-5669` | 0.00 to 1.57 | 12.25 | 2.53 | 0.0339552 | 25.0 | 75.0 | 0.70 | 0.020 |
| 20 | `right_shoulder_yaw_joint` | `RJ-5378` | -1.57 to 1.57 | 5.45 | 5.70 | 0.038 | 20.0 | 60.0 | 0.70 | 0.050 |
| 21 | `right_elbow_joint` | `RJ-5660` | -2.44 to 0.00 | 9.32 | 1.81 | 0.0282528 | 12.0 | 36.0 | 0.40 | 0.015 |
| 22 | `right_wrist_yaw_joint` | `RJ-5660` | -3.14 to 3.14 | 9.32 | 1.81 | 0.0282528 | 12.0 | 36.0 | 0.40 | 0.015 |
| 23 | `waist_yaw_joint` | `RJ-8867` | -1.57 to 1.57 | 12.57 | 2.75 | 0.095625 | 40.0 | 120.0 | 0.70 | 0.050 |
| 24 | `neck_yaw_joint` | `RJ-5660` | -1.57 to 1.57 | 9.32 | 1.81 | 0.0282528 | 12.0 | 36.0 | 0.40 | 0.015 |
| 25 | `neck_pitch_joint` | `RJ-5660` | -0.79 to 0.79 | 9.32 | 1.81 | 0.0282528 | 12.0 | 36.0 | 0.40 | 0.015 |

### Parameter Definitions

- `Kt` is the actuator torque constant: `torque (Nm) = Kt * current (A)`.
- `Armature` represents reflected rotor inertia in joint space and contributes corresponding damping. For a geared actuator, `armature = JG^2`, where `J` is rotor inertia and `G` is the actuator-to-output angular-velocity ratio.
- `Rated torque` is the continuous output the actuator can sustain under normal thermal conditions.
- `Peak torque` is the short-duration maximum used for events such as impacts, push-off, or fast recovery.
- `Static friction` is the torque required to start the joint moving from rest.
- `Dynamic friction` is the lower resisting torque after the joint is moving.

### Parallel Ankle

The humanoid robot uses a parallel RSU ankle (Revolute-Spherical-Universal) rather than a serial single-DOF mechanism. Two revolute actuators work through parallel linkages to produce ankle pitch and roll.



In simulation, ankle pitch and roll are modeled as directly actuated ideal joints. On the physical robot, the two actuators move endpoints on a straight bar, so a kinematic mapping converts actuator A/B positions into pitch/roll coordinates for Sim2Real deployment.



> [!NOTE]
>
> **Fixed flat feet**
>
> The humanoid robot no longer uses passive toe joints. The current design uses fixed flat feet.

The mapping uses:

- `theta_p`: ankle pitch angle in joint space
- `theta_r`: ankle roll angle in joint space
- `theta_A`, `theta_B`: actuator A and B rotation angles
- `r_A`, `r_B`: effective linkage radii for actuators A and B
- `r`: common radius when `r_A = r_B = r`
- `d`: ankle pivot to bar midpoint-line distance
- `c`: distance between the two bar endpoints

The simplified model assumes small angles, rigid symmetric geometry, constant effective radii, and no compliance, backlash, or slip. Positive `y` points away from the ground, and the right-leg convention is used for the derivation.

Mapping equations from actuator space to pitch/roll space:

```text
theta_p = (r_A * theta_A - r_B * theta_B) / (2 * d)
theta_p = (theta_A - theta_B) * (r / (2 * d))

theta_r = -(r_A * theta_A + r_B * theta_B) / c
theta_r = -(theta_A + theta_B) * (r / c)
```

```c
float right_pitch = (right_A - right_B) / (2.0f * K_PITCH);
float right_roll  = -(right_A + right_B) / (2.0f * K_ROLL);

float left_pitch  = (left_A - left_B) / (2.0f * K_PITCH);
float left_roll   = -(left_A + left_B) / (2.0f * K_ROLL);
```

See the public ankle mechanism derivation for the complete derivation.

## Further Reading

- Parallel ankle context
- Unitree G1 basic motion reference
- Omniverse articulation stability and armature
