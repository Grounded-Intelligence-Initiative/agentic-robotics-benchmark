# Protocols

> Command and telemetry field reference for the humanoid robot API.

- Section: Humanoid › Program the humanoid robot › Humanoid API

All commands and telemetry use Protocol Buffers, sent over LiveKit DataChannels and DataTracks.

The field definitions on this page must be used with the generated bindings from the matching supported humanoid robot software release.

## Commands

Commands are sent from your application to the robot on the `commands` data channel.

### CloudCommand

Every command is wrapped in a `CloudCommand` envelope with a timestamp and sequence number.

| Field | Type | Description |
| - | - | - |
| `timestamp_us` | uint64 | Timestamp in microseconds |
| `sequence` | uint32 | Monotonic sequence number |
| `velocity` | VelocityCommand | Walk command (one of) |
| `trajectory` | TrajectoryRequest | Direct joint control (one of) |
| `mode` | ModeCommand | Mode switch (one of) |

Only one of `velocity`, `trajectory`, or `mode` should be set per command.

---

### VelocityCommand

Make the robot walk with the given velocity. Automatically enters MOVE/POLICY mode.

| Field | Type | Envelope | Unit | Description |
| - | - | - | - | - |
| `vx` | float | -1.0 to 1.0 | m/s | Forward (+) / backward (-) velocity |
| `vy` | float | -1.0 to 1.0 | m/s | Strafe left (+) / right (-) velocity |
| `vyaw` | float | -1.5 to 1.5 | rad/s | Turn left (+) / right (-) rate |

The edge does not clamp these values - it rejects non-finite input and forwards the rest. See Robot Control for the full envelope note.

#### Example (Python)

```python
import asyncio
import time
from livekit import rtc
from humanoid_protocol.v1.edge_cloud_pb2 import CloudCommand, VelocityCommand


async def main():
    room = rtc.Room()
    await room.connect(livekit_url, token)

    # Send velocity command
    cmd = CloudCommand(
        timestamp_us=int(time.time() * 1e6),
        sequence=1,
        velocity=VelocityCommand(vx=0.3, vy=0.0, vyaw=0.0)
    )
    await room.local_participant.publish_data(
        cmd.SerializeToString(), topic="commands", reliable=True
    )

    await asyncio.sleep(1)


if __name__ == "__main__":
    asyncio.run(main())
```

---

### TrajectoryRequest

Command a direct joint trajectory. Positions use **firmware order** (25 joints) - see Joint Order.

| Field | Type | Description |
| - | - | - |
| `full` | FullTrajectory | Custom joint trajectory |

### FullTrajectory

A sequence of joint position targets with timing.

| Field | Type | Description |
| - | - | - |
| `segments` | JointSegment\[] | Ordered list of target poses |

### JointSegment

| Field | Count | Range | Unit | Description |
| - | - | - | - | - |
| `positions` | 25 | actuator-specific | radians | Target joint positions in firmware order. Every entry drives its actuator - no skip sentinel. |
| `kp` | 25 | 0 - 500 | - | Position gain per joint (optional - edge fills defaults if omitted) |
| `kd` | 25 | 0 - 5.0 | - | Velocity damping gain per joint (optional - edge fills defaults if omitted) |

Each `JointSegment` is applied as an instantaneous PD target. There is no in-firmware interpolation between segments - clients are responsible for any smoothing.

---

### ModeCommand

Switch the robot's control mode.

| Field | Type | Values | Description |
| - | - | - | - |
| `mode` | Mode enum | MODE_STAND=0, MODE_DAMP=1 | Target mode |

> **Watch out:** The command enum (`Mode`) and the telemetry enum (`FirmwareMode`) use **different numbering** for the same modes:
>
> | | DAMP | STAND |
> | - | - | - |
> | Commands (`Mode`) | 1 | 0 |
> | Telemetry (`FirmwareMode`) | 0 | 1 |
>
> Don't compare them numerically - use the named constants.

See Robot Control for mode transitions.

---

## Telemetry

Sent from the robot at 10 Hz on the `telemetry` DataTrack (lossy - dropped packets are acceptable since the next update arrives in 100ms).

| Field | Type | Count | Description |
| - | - | - | - |
| `timestamp_us` | uint64 | 1 | Edge clock (microseconds) |
| `fw_timestamp_us` | uint64 | 1 | Firmware clock (microseconds) |
| `sequence` | uint32 | 1 | Monotonic counter |
| `fw_mode` | FirmwareMode | 1 | FW_MODE_DAMP=0, FW_MODE_STAND=1, FW_MODE_MOVE=2 |
| `joint_pos` | float | 25 | Joint positions (radians) |
| `joint_vel` | float | 25 | Joint velocities (rad/s) |
| `joint_current` | float | 25 | Actuator current (amps) |
| `joint_temp` | float | 25 | Actuator temperature (celsius) |
| `imu_quat` | float | 4 | Orientation quaternion \[w, x, y, z] |
| `imu_gyro` | float | 3 | Angular velocity \[x, y, z] (rad/s) |
| `imu_gravity` | float | 3 | Projected gravity vector \[x, y, z] |
| `error_flags` | uint32 | 1 | Active errors bitfield |
| `active_alerts` | FirmwareAlert | repeated | Current hardware alerts - see below |
| `fw_age_ms` | uint32 | 1 | Staleness of firmware data when forwarded (ms) |
| `last_video_timestamp_us` | uint64 | 1 | Most recent video frame timestamp (for media sync) |
| `last_audio_timestamp_us` | uint64 | 1 | Most recent audio frame timestamp (for media sync) |

### FirmwareAlert

Hardware alerts forwarded from the firmware safety layer. Present in `EdgeTelemetry.active_alerts` whenever an alert condition is active. Clears automatically when the condition resolves (e.g. actuator cools below threshold).

| Field | Type | Description |
| - | - | - |
| `id` | uint32 | Alert type identifier |
| `severity` | uint32 | Severity level (see table below) |
| `value` | uint32 | Measured value that triggered the alert |
| `threshold` | uint32 | Threshold that was exceeded |
| `first_set_us` | uint64 | Firmware timestamp when alert first fired |
| `source_id` | uint32 | Source (e.g. actuator index) |

**Severity values:**

| Value | Meaning |
| - | - |
| 0 | Critical |
| 1 | Warning |
| 2 | Info |

Common alerts: actuator overtemperature (trips at 80 C, clears at 70 C), fall detection, CAN bus faults.

#### Example (Python)

```python
import asyncio
from livekit import rtc
from humanoid_protocol.v1.edge_cloud_pb2 import EdgeTelemetry


async def read_telemetry(track):
    stream = track.subscribe()
    async for frame in stream:
        t = EdgeTelemetry.FromString(frame.payload)
        print(f"Mode: {t.fw_mode}")
        print(f"Joint positions: {list(t.joint_pos)}")
        print(f"IMU quaternion: {list(t.imu_quat)}")


async def main():
    room = rtc.Room()
    await room.connect(livekit_url, token)

    # Telemetry arrives on a DataTrack, not a DataChannel
    @room.on("data_track_published")
    def on_data_track(track):
        asyncio.create_task(read_telemetry(track))

    await asyncio.sleep(10)


if __name__ == "__main__":
    asyncio.run(main())
```

---

## System Events

Sent from the robot on the `system` data channel (reliable - these must not be lost).

### EdgeEvent

| Field | Type | Description |
| - | - | - |
| `timestamp_us` | uint64 | Timestamp in microseconds |
| `sequence` | uint32 | Monotonic counter |
| `error` | EdgeError | Error report (one of) - emitted on error |
| `diagnostics` | EdgeDiagnostics | Status snapshot (one of) - emitted every ~1s |
| `controller` | ControllerEvent | Control source change (one of) - emitted on change |

### EdgeError

| Field | Type | Description |
| - | - | - |
| `subsystem` | Subsystem | Source subsystem (see enum below) |
| `code` | string | e.g. "CAMERA_OPEN_FAILED", "FW_LINK_TIMEOUT" |
| `message` | string | Human-readable detail |

**Subsystem enum**: `SUBSYSTEM_FW_LINK=0`, `SUBSYSTEM_CAMERA=1`, `SUBSYSTEM_MIC=2`, `SUBSYSTEM_SPEAKER=3`, `SUBSYSTEM_BLE=4`, `SUBSYSTEM_CLOUD=5`, `SUBSYSTEM_FIRMWARE=6`

### EdgeDiagnostics

Emitted every ~1 second on the `system` channel. Provides a snapshot of edge and firmware health.

| Field | Type | Description |
| - | - | - |
| `can_health` | CanBusHealth\[] | Per-bus CAN statistics (frames sent/received, errors, bus-offs) |
| `onnx_avg_ms` | float | Average ONNX inference time (ms) |
| `onnx_max_ms` | float | Max ONNX inference time (ms) |
| `onnx_count` | uint64 | Total inference count |
| `controller` | string | Active controller: "ble", "cloud", or "" |
| `provisioned` | bool | Robot setup complete |
| `cloud_connected` | bool | LiveKit connection active |
| `ble_connected` | bool | Phone connected via BLE |
| `camera_active` | bool | Camera capturing |
| `mic_active` | bool | Microphone active |

### ControllerEvent

Fired when the active control source changes.

| Field | Type | Description |
| - | - | - |
| `previous` | string | Previous controller |
| `current` | string | New controller |
| `reason` | string | "acquired", "override", "timeout", "disconnect" |
