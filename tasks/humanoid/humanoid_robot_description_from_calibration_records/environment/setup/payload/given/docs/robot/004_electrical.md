# Electrical

> The humanoid robot battery, power, control electronics, actuator communications, sensors, and media hardware.

- Section: Humanoid › Overview › System Tour

## Overall Architecture

The Compute Module comprises two connected stacks: the **Head Compute Unit** in the head and the **Robot Processing Unit (RPU)** in the torso.

| Subsystem | Role |
| - | - |
| Head Compute Unit | Provides media, networking, edge services, and the buzzer |
| RPU | Runs locomotion and actuator control, handles IMU feedback and actuator CAN communications, and distributes power |
| Actuator network | Carries battery power and CAN communication through the robot |
| Sensors and media | Provide inertial state, video, audio input, and audio output |
| Battery system | Supplies energy for actuators, compute, sensing, and peripherals |

## Compute Module

**Compute Module**: two board stacks, one in the head and one in the torso.

**In the head: Head Compute Unit** (top to bottom)

1. Media Hat Board (top): buzzer and Pi 5 brownout protection
2. Raspberry Pi 5 (part `700_09X`): media, networking, and edge services
3. Head Board (base): head power and peripheral connections

**In the torso: Robot Processing Unit (RPU)** (top to bottom)

1. Power Distribution Board (PDB) (top): distributes power to connected hardware
2. Radxa CM5 (part `100_15X`): runs motion-control firmware and locomotion
3. Motion Control Board (MCB) (base): hosts the CM5 and connects to the actuator network

- Data link: Pi 5 to MCB via the 1000 mm Ethernet cable.
- Head power: PDB to Head Board via the 1000 mm XT30-Male to XT30-Female power cable.

Mounts, fan, camera, and other peripherals are omitted.

### Head Compute Unit

The head stack contains three boards. The Head Board sits at the base, the Raspberry Pi 5 (part `700_09X`) mounts above it on standoffs, and the Media Hat Board mounts on the Pi 5 header. The Media Hat Board provides the buzzer and helps prevent brownouts on the Pi 5.

The **Head Compute Unit** is the electronics stack, not the entire mechanical head assembly. Its camera, mounts, covers, and neck hardware are identified separately in the build instructions.

### Robot Processing Unit (RPU)

The torso stack also contains three boards. The Radxa CM5 (part `100_15X`) mounts on the Motion Control Board (MCB). The Power Distribution Board (PDB) then fits above the MCB. Together, the MCB and PDB form the **Robot Processing Unit (RPU)**.

The CM5 handles high-frequency, low-latency workloads, including locomotion, actuator control, and IMU feedback. The MCB provides the interfaces for communication with all 25 actuators. The PDB distributes power to connected hardware.

### Connecting the Two Assemblies

A 1000 mm Ethernet cable links the Pi 5 to the MCB. A separate 1000 mm XT30-Male to XT30-Female power cable connects the PDB to the Head Board. The Head Compute Unit and RPU work together while remaining in separate locations on the robot.

Keeping media, external networking, and application services on the Pi 5 separates those workloads from the CM5's time-sensitive control loop.

## Actuator Power and Communication

The published system specification identifies five CAN interfaces at 1 Mbps and one at 500 kbps. The actuator network distributes battery power and CAN communication from the body outward through six physical branches.

### Daisy-Chain Architecture

Each actuator has two electrically equivalent `XT30 (2+2)` ports. Each port carries `BATT+`, `BATT-`, `CAN_H`, and `CAN_L`, allowing power and communication to continue from one actuator to the next.

There is no fixed electrical input or output port. Use the port and orientation shown in the applicable assembly step so the cable follows the intended mechanical path.

The six branches enter the body at left hip pitch, right hip pitch, waist yaw, left shoulder pitch, right shoulder pitch, and neck yaw. Within each branch, actuators form a chain from the central body toward the distal joint.

At each branch entry, `CAN_H` and `CAN_L` route toward the RPU while `BATT+` and `BATT-` route toward the robot power bus.

For example, each leg follows:

- `hip pitch -> hip roll -> hip yaw -> knee -> ankle A -> ankle B`

This topology keeps the number of body-entry connections small while allowing each limb to carry power and CAN through the same connector system.

> [!WARNING]
>
> **Keep the robot unpowered**
>
> Keep the battery disconnected and its output protected. Do not connect, disconnect, route, strip, or join conductors while any part of the robot is energized.

> [!NOTE]
>
> **Wire-to-actuator visualization coming soon**
>
> An interactive visualization will be added here to show how every wire connects through the actuator network.

> [!NOTE]
>
> **CAN mapping under confirmation**
>
> The public sources do not yet reconcile the six physical branch-entry paths with the controller specification of five 1 Mbps CAN interfaces plus one 500 kbps interface. Treat the physical topology and controller-interface count as separate facts until the release-specific bus assignment is confirmed.

## Sensors and Media Hardware

| Device | Current specification | Primary role |
| - | - | - |
| IMU | 6-axis | Body orientation and angular-motion sensing |
| Camera | 2 MP monocular camera | Robot video and visual input |
| Microphone | Stereo microphone array | Audio capture |
| Speaker | 10 W, 4 ohm | Robot audio output |

The Raspberry Pi 5 is the currently supported edge computer for camera, audio, networking, and edge services. Other Raspberry Pi models are not validated for the humanoid robot.

## Battery

The current pack design is a 13-series, 4-parallel lithium-ion battery.

| Parameter | Current specification |
| - | -: |
| Battery type | Lithium-ion |
| Cell model | INR18650 |
| Pack configuration | 13S4P |
| Nominal voltage | 46.8 V |
| Typical capacity | 10.5 Ah |
| Minimum capacity | 10.05 Ah |
| Energy | 491.4 Wh |
| Charge voltage | 54.6 V |
| Discharge cut-off | 35.75 V |
| Standard charge current | 2.1 A |
| Maximum charge current | 15 A |
| Standard discharge current | 2.1 A |
| Maximum discharge current | 30 A |
| Storage voltage | 45.5 to 48.1 V |
| Battery mass | Approximately 2 kg |

### Operating Temperature

| Condition | Temperature range |
| - | -: |
| Charging | 0°C to 45°C |
| Discharging | -20°C to 60°C |
| Storage, less than 1 month | -10°C to 45°C |
| Long-term storage, less than 3 months | -10°C to 35°C |

### Protection Circuit

The pack includes a protection circuit module for over-charge, over-discharge, and over-current protection.

| Protection item | Threshold |
| - | -: |
| Over-charge protection | 4.25 V per cell |
| Over-discharge protection | 2.75 V per cell |
| Over-current protection | 30 A or less |
