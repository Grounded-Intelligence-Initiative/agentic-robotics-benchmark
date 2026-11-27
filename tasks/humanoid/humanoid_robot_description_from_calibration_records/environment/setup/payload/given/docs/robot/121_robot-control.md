# Robot Control

> Control modes, state machine, commands, and telemetry for the humanoid robot.

- Section: Humanoid › Program the humanoid robot › Humanoid API

Send commands on the `commands` DataChannel, receive telemetry on the `telemetry` DataTrack. See the Humanoid API overview for connection setup.

> [!WARNING]
>
> **Control requires commissioned hardware**
>
> Use these commands only on a commissioned robot with version-matched software, validated support and operating procedures, and an independent hardware emergency stop. DAMP is a software control state, not an emergency-stop system.

## Control Modes

See the state machine diagram on the overview page for a visual reference.

| Mode | Description |
| - | - |
| **DAMP** | Actuators go compliant (limp). Safe default. The robot enters DAMP automatically on: boot, fall detected, actuator overtemp, trajectory session timeout (2 s), or explicit `ModeCommand(DAMP)`. |
| **STAND** | Robot ramps to a standing pose over ~2 s. The ramp is **advisory** - the firmware accepts MOVE commands the moment they arrive. If you want a settled pose before commanding MOVE, wait ~2 s after sending `ModeCommand(STAND)`. |
| **MOVE/POLICY** | Neural network locomotion at 50 Hz. Send `VelocityCommand(vx, vy, vyaw)` to control walking speed and direction. |
| **MOVE/TRAJECTORY** | Direct PD control of joints. No neural network. Requires **continuous streaming** (~50 Hz) - if trajectory packets stop for 2 s during an active session, the robot auto-DAMPs. Every value in `positions[25]` actively drives its actuator - there is no skip sentinel. For joints you don't want to actively command, send a hold target of your choice (e.g. the joint's current value read from `EdgeTelemetry.joint_pos`). |

### Allowed Transitions

| From | To | Command | Notes |
| - | - | - | - |
| Any | DAMP | `ModeCommand(DAMP)` | Always allowed as a software stop request; not a hardware emergency stop. |
| DAMP | STAND | `ModeCommand(STAND)` | Robot ramps to standing pose over \~2 s. Ramp is advisory - wait \~2 s before sending MOVE if you want a settled pose. |
| STAND | MOVE/POLICY | `VelocityCommand` | Sending velocity auto-enters POLICY. |
| STAND | MOVE/TRAJECTORY | `TrajectoryRequest` | Sending trajectory auto-enters TRAJECTORY. |
| MOVE | STAND | `ModeCommand(STAND)` | Returns to default standing pose. |

> **There is no DAMP -> MOVE shortcut.** While the firmware reports `FW_MODE_DAMP`, both `VelocityCommand` and `TrajectoryRequest` are dropped at the edge. Pass through `ModeCommand(STAND)` first, for velocity and trajectory alike.

**Trajectory session timeout**: while a trajectory teleop session is active, your application must stream `TrajectoryRequest` packets at ~50 Hz. If no trajectory packet arrives for **2 s**, the robot auto-DAMPs. This catches client disconnects (tab close, WiFi drop, grip release).

**Velocity staleness**: velocity has its own watchdog. If **2 s** pass with no new nonzero `VelocityCommand`, the edge issues a zero velocity - the robot holds in MOVE at a standstill rather than DAMPing or returning to STAND. Clients that deduplicate a held control must re-send the held nonzero command periodically (the reference clients use a 500 ms keepalive) or the robot will stop under them.

On the cloud path this watchdog is suppressed while the operator is still confirmed present in the session. The moment presence cannot be confirmed, the 2 s rule applies.

---

## Commands

### Velocity (Walk)

```python
import itertools
import time
from humanoid_protocol.v1.edge_cloud_pb2 import CloudCommand, VelocityCommand

# Monotonic sequence counter shared by all examples on this page.
_seq = itertools.count(1)
def seq() -> int:
    return next(_seq)


cmd = CloudCommand(
    timestamp_us=int(time.time() * 1e6),
    sequence=seq(),
    velocity=VelocityCommand(vx=0.5, vy=0.0, vyaw=0.0)
)
await room.local_participant.publish_data(
    cmd.SerializeToString(), topic="commands", reliable=True
)
```

| Field | Envelope | Unit | Description |
| - | - | - | - |
| `vx` | -1.0 to 1.0 | m/s | Forward (+) / backward (-) |
| `vy` | -1.0 to 1.0 | m/s | Strafe left (+) / right (-) |
| `vyaw` | -1.5 to 1.5 | rad/s | Turn left (+) / right (-) |

> **The edge does not clamp velocity.** It rejects non-finite values and forwards everything else verbatim; the firmware applies its own limits. The envelope above is the ceiling the reference operator clients pin every command to before sending, and it is the envelope this documentation is written against. Clamp on your side - do not rely on the robot to catch a runaway command.

> **Send a `ModeCommand(STAND)` first.** `VelocityCommand` packets received while the robot is in `FW_MODE_DAMP` are silently dropped - no error event is emitted. Wait until telemetry reports `FW_MODE_STAND` (or `FW_MODE_MOVE`) before sending velocity.

### Trajectory (Direct Joint Control)

Send 25 positions in firmware order. Every value actively drives its actuator - there is no skip sentinel. For joints you don't want to actively move, send a hold target of your choice (typical pattern: read `EdgeTelemetry.joint_pos` and override only the joints you're commanding).

Each `TrajectoryRequest` is applied as an instantaneous PD target - there is no in-firmware interpolation between segments. Clients are responsible for any smoothing across packets.

```python
import time
from humanoid_protocol.v1.edge_cloud_pb2 import (
    CloudCommand, TrajectoryRequest, FullTrajectory, JointSegment, EdgeTelemetry
)

# Keep a reference to the latest telemetry frame; update it from your
# telemetry subscription handler (see "Telemetry" below).
latest_telemetry: EdgeTelemetry | None = None

# ... once `latest_telemetry` has been populated by your subscription handler:
assert latest_telemetry is not None, "wait for the first telemetry frame"

# Seed positions from the latest telemetry pose so unspecified joints
# hold their current value; override only the ones you're driving.
positions = list(latest_telemetry.joint_pos)   # 25 floats
positions[12] = 0.5           # L_Shoulder_Pitch
positions[13] = -0.2          # L_Shoulder_Roll
positions[15] = 1.2           # L_Elbow

cmd = CloudCommand(
    timestamp_us=int(time.time() * 1e6),
    sequence=seq(),
    trajectory=TrajectoryRequest(
        full=FullTrajectory(segments=[
            JointSegment(positions=positions)
        ])
    )
)
await room.local_participant.publish_data(
    cmd.SerializeToString(), topic="commands", reliable=True
)
```

| Field | Count | Range | Unit | Description |
| - | - | - | - | - |
| `positions` | 25 | actuator-specific | radians | Target positions in firmware order. Every entry drives its actuator - no skip sentinel. |
| `kp` | 25 | 0 - 500 | - | Position gain (optional - edge fills defaults if omitted) |
| `kd` | 25 | 0 - 5.0 | - | Velocity damping gain (optional - edge fills defaults if omitted) |

> **KP/KD overrides are all-or-nothing.** To override gains, send exactly **25** values in `kp` and/or `kd`. Any other count (including partial overrides like 23 or 24 values) is silently replaced with the edge defaults.

### Mode Switch

```python
import time
from humanoid_protocol.v1.edge_cloud_pb2 import CloudCommand, ModeCommand, Mode


# Stand up
cmd = CloudCommand(
    timestamp_us=int(time.time() * 1e6),
    sequence=seq(),
    mode=ModeCommand(mode=Mode.MODE_STAND)
)

# Request software DAMP (not a hardware emergency stop)
cmd = CloudCommand(
    timestamp_us=int(time.time() * 1e6),
    sequence=seq(),
    mode=ModeCommand(mode=Mode.MODE_DAMP)
)
```

---

## Command Validation

The robot silently drops malformed or safety-gated commands - **no `EdgeError` event is emitted on the `system` channel** when this happens. Validate inputs client-side before sending. The current drop conditions:

| Command | Dropped when... |
| - | - |
| `VelocityCommand` | any of `vx`, `vy`, `vyaw` is non-finite (NaN/Inf), **or** firmware is in `FW_MODE_DAMP` |
| `TrajectoryRequest` | `JointSegment.positions` length != 25, **or** any position is non-finite, **or** `FullTrajectory.segments` is empty, **or** the request uses the unimplemented `id` source (LUT name) instead of `full`, **or** firmware is in `FW_MODE_DAMP` |
| `ModeCommand(STAND)` | firmware is **fault-DAMPed** - it entered DAMP by itself with `error_flags` set or a critical alert active. The robot will not stand back up into a fault. |
| `ModeCommand(DAMP)` | never dropped - always accepted |

Any command is also dropped if it arrives from a source that is not the arbiter's active controller.

Velocity values are **not** clamped by the edge - only non-finite values are rejected. KP/KD arrays of any length other than 25 are not rejected - edge silently substitutes its own defaults.

> **A failed STAND is not always a connectivity problem.** Check `error_flags` and `active_alerts` before retrying. Some firmware faults - a detected fall in particular - latch and clear only on a firmware restart; no command recovers them, and the locomotion policy has no get-up behaviour.

---

## Telemetry

The robot streams telemetry at 10 Hz on the `telemetry` DataTrack (lossy).

| Field | Type | Count | Description |
| - | - | - | - |
| `timestamp_us` | uint64 | 1 | Edge clock (microseconds) |
| `fw_timestamp_us` | uint64 | 1 | Firmware clock (microseconds) - use with `timestamp_us` for latency measurement |
| `sequence` | uint32 | 1 | Monotonic counter |
| `fw_mode` | FirmwareMode | 1 | FW_MODE_DAMP=0, FW_MODE_STAND=1, FW_MODE_MOVE=2 |
| `joint_pos` | float | 25 | Joint positions (radians) |
| `joint_vel` | float | 25 | Joint velocities (rad/s) |
| `joint_current` | float | 25 | Actuator current (amps) |
| `joint_temp` | float | 25 | Actuator temperature (celsius) |
| `imu_quat` | float | 4 | Orientation \[w, x, y, z] |
| `imu_gyro` | float | 3 | Angular velocity (rad/s) |
| `imu_gravity` | float | 3 | Projected gravity vector |
| `error_flags` | uint32 | 1 | Active errors bitfield |
| `active_alerts` | FirmwareAlert | repeated | Current hardware alerts (actuator overtemp, fall detect, CAN faults) |
| `fw_age_ms` | uint32 | 1 | How stale the firmware data was when forwarded (milliseconds) |
| `last_video_timestamp_us` | uint64 | 1 | Timestamp of the last video frame sent (for media sync) |
| `last_audio_timestamp_us` | uint64 | 1 | Timestamp of the last audio frame sent (for media sync) |

```python
import asyncio
from humanoid_protocol.v1.edge_cloud_pb2 import EdgeTelemetry


async def read_telemetry(track):
    stream = track.subscribe()
    async for frame in stream:
        t = EdgeTelemetry.FromString(frame.payload)
        print(f"Mode: {t.fw_mode}, Joints: {list(t.joint_pos)}")


# Subscribe to the robot's telemetry DataTrack
@room.on("data_track_published")
def on_data_track(track):
    asyncio.create_task(read_telemetry(track))
```

---

## Joint Order

All trajectory commands use **firmware order** - 25 joints matching the CAN bus actuator layout. These are the indices for the `positions` array.

| Group | Indices | Joints |
| - | - | - |
| Left Leg | 0-5 | L_Hip_Pitch, L_Hip_Roll, L_Hip_Yaw, L_Knee, L_Ankle_A, L_Ankle_B |
| Right Leg | 6-11 | R_Hip_Pitch, R_Hip_Roll, R_Hip_Yaw, R_Knee, R_Ankle_A, R_Ankle_B |
| Left Arm | 12-16 | L_Shoulder_Pitch, L_Shoulder_Roll, L_Shoulder_Yaw, L_Elbow, L_Wrist_Yaw |
| Right Arm | 17-21 | R_Shoulder_Pitch, R_Shoulder_Roll, R_Shoulder_Yaw, R_Elbow, R_Wrist_Yaw |
| Waist + Neck | 22-24 | Waist_Yaw, Neck_Yaw, Neck_Pitch |

### PD Gains

KP/KD gains are injected automatically if you don't provide them. To override, send exactly 25 values in `kp` and/or `kd` matching the joint order above. For reference:

| Parameter | Range | Typical |
| - | - | - |
| Kp (position gain) | 0 - 500 | 40-150 |
| Kd (velocity gain) | 0 - 5.0 | 2.0-5.0 |
