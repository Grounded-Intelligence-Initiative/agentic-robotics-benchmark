# Overview

> The most developer-friendly humanoid robot!

- Section: Humanoid

The humanoid robot represents a more open approach to humanoid robotics, giving researchers and developers the freedom to understand, repair, customize, and develop the robot across its full stack.



| Specification | The humanoid robot |
| - | - |
| Height | Approximately 1.2 m (3.94 ft) |
| Weight | Approximately 35 kg (77 lb) |
| Powered joints | 25 |
| Legs | 6 powered joints per leg |
| Arms | 5 powered joints per arm |
| Body and neck | 1 waist joint and 2 neck joints |
| Compute | Radxa CM5 (part `100_15X`) for motion control; Raspberry Pi 5 (part `700_09X`) for media and networking |
| Sensing and media | IMU, monocular camera, stereo microphone array, and speaker |
| Power | Rechargeable 13S4P lithium-ion battery system |

The Compute Module consists of two connected stacks, the **Head Compute Unit** and the **Robot Processing Unit (RPU)**:

- The Head Compute Unit contains the Pi 5, Head Board, and Media Hat Board. The Media Hat Board provides the buzzer and brownout protection.
- The RPU is the Motion Control Board (MCB) and the Power Distribution Board (PDB) above it. The CM5 mounts on the MCB.

## Key Features

- **Sim2Real Ready**: Train with the provided MuJoCo and URDF models using Isaac Lab or MJLab templates, then deploy zero-shot to the humanoid robot with no extra tuning.
- **Full SDK + Emulator Workflow (Coming Soon)**: Build, test, and deploy with one SDK for robot control, policies, telemetry, and media. Applications run 1:1 from the emulator to the humanoid robot.
- **Customizable Attachments**: Customize your own humanoid robot by adapting the wrist, head, covers, and back panel with hands, tools, sensors, aesthetics, and compute tailored to your needs.
- **Repairable by Design**: Operate under load with off-the-shelf failure points that protect major assemblies and can be swapped in minutes.

## A Fully Open Platform

The humanoid robot makes its mechanical design, electrical design, simulation models, firmware, and bill of materials available directly to builders.

- **Design Files**: Browse the mechanical CAD files and electrical design on GitHub. View on GitHub ↗
- **Simulation Files**: Access the MJCF and URDF models for simulation. MJCF ↗ · URDF ↗
- **Firmware**: Download and flash the software images needed to run the humanoid robot. Coming soon

## Bill of Materials

Enter your email below to receive the Bill of Materials.

The Bill of Materials is sent by email: enter an email address in the form on the web version of this page.

## Software Availability

| Capability | Status |
| - | - |
| Full-body locomotion policy | Available in the supplied firmware |
| Directional remote teleoperation | Available |
| Direct actuator control | Available through the Humanoid Edge or Humanoid SDK |
| Robot telemetry | Available at 10 Hz |
| MuJoCo and URDF models | Provided with the humanoid robot files |
| Full-body teleoperation | In development |
| Humanoid Client SDK | In development |
| The maker's platform | Coming soon |

---

1. Assemble the humanoid robot: Unpack the kit, assemble the robot, and verify the build.
2. Operate the humanoid robot: Bring the robot up, drive it, and keep it running.
3. Program the humanoid robot: Write software that drives the robot, with the SDK and API.
4. Train the humanoid robot: Train new locomotion policies and deploy them to the robot.
5. Extend the humanoid robot: Add hands, sensors, compute, and custom hardware.
