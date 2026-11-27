# Wire format

> The humanoid robot session channels, EdgeTelemetry fields, CloudCommand structure, and EdgeEvent types.

- Section: Humanoid › Program the humanoid robot › Humanoid API

This page is a lookup for the messages exchanged between an application and the humanoid robot. For value ranges, joint order, PD gains, and worked Python examples, see Protocols.

## Session channels

A session is a WebRTC room (implemented via LiveKit). Inside it are three data channels plus two media tracks:

| Channel | Reliability | Direction | Purpose |
| - | - | - | - |
| `commands` | Reliable, ordered | Application → Robot | Send `CloudCommand` — a `oneof` of `velocity`, `trajectory`, or `mode` |
| `telemetry` | Lossy | Robot → Application | Receive `EdgeTelemetry` at 10 Hz |
| `system` | Reliable, ordered | Robot → Application | Receive `EdgeEvent` — robot-side events (boot, errors, alerts, mode transitions) |
| Video track | — | Robot → Cloud | FPV and third-person camera streams |
| Audio track | — | Bidirectional | Robot microphone and speaker |

Applications join the robot's LiveKit room and exchange these messages using the generated bindings supplied with the supported software release.

## EdgeTelemetry

Delivered over the lossy `state` channel as protobuf. Each packet contains:

| Field | Description |
| - | - |
| `timestamp_us` | Edge clock timestamp in µs (`CLOCK_REALTIME`) |
| `fw_timestamp_us` | Firmware clock timestamp in µs |
| `sequence` | Monotonically increasing packet counter |
| `fw_mode` | Current firmware mode (`FW_MODE_DAMP` / `FW_MODE_STAND` / `FW_MODE_MOVE`) |
| `joint_pos` | Joint positions in radians (25-element array, one per actuator) |
| `joint_vel` | Joint velocities in rad/s |
| `joint_current` | Actuator currents in amps |
| `joint_temp` | Actuator temperatures in °C |
| `imu_quat` | IMU orientation quaternion `[w, x, y, z]` |
| `imu_gyro` | IMU angular velocity `[x, y, z]` in rad/s |
| `imu_gravity` | Projected gravity vector from the IMU `[x, y, z]` |
| `error_flags` | Bitfield of active error codes |
| `alert_count` | Number of active alerts |
| `active_alerts` | Repeated `FirmwareAlert` — the alerts themselves (actuator overtemp, fall detect, CAN faults) |
| `fw_age_ms` | How stale the firmware data was when the edge forwarded it (ms) |
| `last_video_timestamp_us` | Timestamp of the most recent video frame (backend correlates with LiveKit RTP) |
| `last_audio_timestamp_us` | Timestamp of the most recent audio frame |

Use the generated `EdgeTelemetry` binding to decode each packet.

## CloudCommand

Sent on the reliable `commands` channel. `CloudCommand` is a protobuf `oneof` — exactly one of the three fields is set per message:

| Variant | Purpose |
| - | - |
| `velocity` | `VelocityCommand` with `vx`, `vy`, `vyaw` |
| `trajectory` | Joint-space trajectory — see TrajectoryRequest |
| `mode` | Firmware mode transition (`damp` / `stand` / `move`) |

### VelocityCommand

| Field | Description |
| - | - |
| `vx` | Forward / backward velocity (m/s) |
| `vy` | Left / right lateral velocity (m/s) |
| `vyaw` | Rotational velocity (rad/s) |

## EdgeEvent

Delivered over the reliable `system` channel. Carries boot progress, error reports, alert triggers, and mode-transition notifications from the robot. Use it to react to state changes rather than polling telemetry.
