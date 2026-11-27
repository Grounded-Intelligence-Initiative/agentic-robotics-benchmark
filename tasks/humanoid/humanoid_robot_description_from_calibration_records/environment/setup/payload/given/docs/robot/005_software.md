# Software

> The humanoid robot firmware, motion control, edge services, telemetry, media, and application interfaces.

- Section: Humanoid › Overview › System Tour

The humanoid robot separates low-level motion control from media, networking, and application connectivity. This lets developers work at different layers without treating the robot as a single software process.

## Compute Responsibilities

| Layer | Runs on | Responsibility |
| - | - | - |
| Motion control | Radxa CM5 in the RPU | Actuator communication, IMU state, control modes, command handling, and firmware telemetry |
| Edge services | Raspberry Pi 5 in the Head Compute Unit | Camera, microphone, speaker, networking, media transport, and external connectivity |
| Application | External computer or service | Operator interfaces, data processing, teleoperation logic, autonomy, and project-specific behavior |

The CM5 and Pi 5 use matched software images and communicate over a direct Ethernet link. The Pi 5 sits in the Head Compute Unit, and the CM5 sits in the Robot Processing Unit (RPU) in the torso.

## Data Flow

The documented low-level interface uses LiveKit and WebRTC to connect an application to the robot.

| Path | Direction | Content | Transport |
| - | - | - | - |
| Commands | Application to robot | Mode, velocity, and joint trajectory requests | Reliable DataChannel |
| Telemetry | Robot to application | Joint state, current, temperature, IMU, and alerts at 10 Hz | Lossy DataTrack |
| System events | Robot to application | Errors and diagnostics | Reliable DataChannel |
| Video | Robot to application | H.264 video, requested at 1280 x 720 and 30 fps | WebRTC media track |
| Audio | Bidirectional | Microphone input and speaker output | Opus media tracks |

The Humanoid API reference documents these release-specific control, telemetry, media, and protocol interfaces.

## Control Model

The current interface describes three firmware control states:

| State | Purpose |
| - | - |
| DAMP | Software-requested compliant actuator state |
| STAND | Transition toward a standing pose |
| MOVE | Policy-based velocity control or direct joint trajectory control |

Direct trajectory messages contain targets for all 25 powered joints. Policy control accepts planar velocity commands, while telemetry reports the robot state back to the application.

> [!WARNING]
>
> **DAMP is not an emergency stop**
>
> DAMP is a software-requested compliant state. Use the released startup, support, shutdown, and emergency-disconnect procedures when operating the physical robot.

## Included Locomotion

The CM5 firmware image includes the humanoid robot locomotion policy. After both computers are flashed and the robot completes first-time setup, no separate policy installation is required.

Velocity commands provide forward, backward, lateral, and turning control. The locomotion policy runs against the 23-DOF body model, with the two neck joints locked.

## Direct Control and Teleoperation

Trajectory requests provide direct control over all 25 actuator targets. An active trajectory session streams at approximately 50 Hz and automatically transitions to DAMP if packets stop.

Directional remote driving through velocity commands is available. The camera, microphone array, speaker, and bidirectional media tracks provide the audiovisual path for remote interaction. Full-body joint teleoperation is in development.

## Python Client SDK and Emulator

We are actively developing and battle-testing the Python Client SDK and emulator internally, with a public release coming soon.

- Humanoid API: Open the detailed control, telemetry, media, protocol, and wire-format reference.
