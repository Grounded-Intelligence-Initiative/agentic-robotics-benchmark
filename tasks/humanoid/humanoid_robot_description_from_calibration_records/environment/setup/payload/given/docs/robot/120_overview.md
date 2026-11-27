# Overview

> Software architecture, data flow, and API overview for the humanoid robot.

- Section: Humanoid › Program the humanoid robot › Humanoid API

The humanoid robot exposes an API for directional driving, direct actuator control, 10 Hz telemetry, and media streaming through LiveKit WebRTC. The locomotion policy is included in the supplied Radxa CM5 firmware for the Robot Processing Unit (RPU).

> [!WARNING]
>
> **Match the robot software release**
>
> Use this low-level reference only with the generated protobuf bindings supplied by the supported humanoid robot software release on the robot. Do not infer field compatibility across releases. API commands are not a commissioning procedure, and a software DAMP command does not replace the independent hardware emergency-stop system.

## Architecture

```d2
direction: right

your_app: Your Application {
  style.fill: "#e8f4fd"
  style.stroke: "#2196F3"
}

robot: the humanoid robot Robot {
  style.fill: "#e8f5e9"
  style.stroke: "#4CAF50"
}

your_app -> robot: Commands\n(walk, stand, trajectory) {
  style.stroke: "#2196F3"
  style.stroke-width: 2
}
robot -> your_app: Video\n(H.264, 30fps) {
  style.stroke: "#E91E63"
  style.stroke-width: 2
}
robot -> your_app: Audio (mic)\n(Opus) {
  style.stroke: "#FF9800"
  style.stroke-width: 2
}
your_app -> robot: Audio (speaker)\n(Opus) {
  style.stroke: "#9C27B0"
  style.stroke-width: 2
}
robot -> your_app: Telemetry\n(joints, IMU, errors @ 10Hz) {
  style.stroke: "#4CAF50"
  style.stroke-width: 2
}
```

**Data flow**: Your application connects to a LiveKit room. The robot publishes video and telemetry. You send commands back. Everything travels over WebRTC - low latency, NAT traversal, adaptive bitrate.

## Command Processing

Your commands don't reach the actuators directly - the robot validates, transforms, and safety-checks every command before acting on it. It remaps joints, injects PD gains, gates commands when in a safety state, and auto-DAMPs if your application disconnects. See Robot Control for details.

**Two DataChannels**, **one DataTrack**, plus media tracks carry traffic between your application and the robot:

| Name | Direction | Content | Delivery |
| - | - | - | - |
| `commands` | You -> Robot | Control commands | DataChannel, reliable |
| `telemetry` | Robot -> You | Joint state, IMU, alerts at 10 Hz | DataTrack, lossy |
| `system` | Robot -> You | Errors, diagnostics | DataChannel, reliable |
| Video | Robot -> You | H.264, 30 fps | LiveKit video track |
| Audio | bidirectional | Opus, mic up + speaker down | LiveKit audio track |

## State Machine

The robot has 3 control modes (DAMP, STAND, MOVE) and three ways to drive it:

- **A. Walk** - `ModeCommand(STAND)` -> wait ~2 s -> `VelocityCommand`. Robot stands, then the locomotion policy walks at the commanded velocity.
- **B. Joint teleop** - `ModeCommand(STAND)` -> wait ~2 s -> `TrajectoryRequest` @ ~50 Hz. Robot stands first, then accepts direct joint targets from a settled pose.
- **C. Direct joint control** - `TrajectoryRequest` straight from DAMP, no `ModeCommand` needed. Direct PD engages immediately. The robot transitions from compliant (limp) to actively driven on the first packet - your first target should be a sensible pose.

The STAND ramp takes ~2 s but is **advisory**, not enforced - the firmware accepts MOVE commands the moment they arrive. Clients that want a settled pose before walking or teleoperating must wait themselves.

```d2
direction: down

DAMP: DAMP\nActuators limp. Safe default. {
  style.fill: "#ffebee"
  style.stroke: "#f44336"
}

STAND: STAND\nRamps to standing pose (~2s, advisory) {
  style.fill: "#e8f5e9"
  style.stroke: "#4CAF50"
}

MOVE: MOVE {
  style.fill: "#e3f2fd"
  style.stroke: "#2196F3"

  POLICY: POLICY\nVelocityCommand -> robot walks
  TRAJECTORY: TRAJECTORY\nTrajectoryRequest -> direct joint control
}

DAMP -> STAND: ModeCommand(STAND) {
  style.stroke: "#4CAF50"
}
STAND -> DAMP: ModeCommand(DAMP) {
  style.stroke: "#f44336"
}
STAND -> MOVE: VelocityCommand or\nTrajectoryRequest {
  style.stroke: "#2196F3"
}
DAMP -> MOVE.TRAJECTORY: TrajectoryRequest\n(direct, no STAND) {
  style.stroke: "#FF9800"
  style.stroke-width: 2
}
MOVE -> STAND: ModeCommand(STAND) {
  style.stroke: "#4CAF50"
}
MOVE -> DAMP: ModeCommand(DAMP)\nor auto (fall, overtemp,\ntimeout, disconnect) {
  style.stroke: "#f44336"
  style.stroke-dash: 3
}
```

## Prerequisites

Before connecting to the robot, you need:

1. **LiveKit server** - Self-host (see below) or use LiveKit Cloud.
2. **Python LiveKit SDK** - Install with `pip install livekit livekit-api`.
3. **The humanoid robot protobuf bindings** - Use the generated Python bindings supplied with your supported humanoid robot software release. The examples below import them from `humanoid_protocol.v1.edge_cloud_pb2`.

For a typed Python client, use the Python SDK, which speaks this wire over UDP on the local network or over the LiveKit room, and adds the keepalive, waits and error model on top. This page is the low-level interface underneath it. The maker's platform is coming soon and is not required for this API.

## Self-Hosting LiveKit

Install the LiveKit server:

- **macOS:** `brew install livekit`
- **Linux:** `curl -sSL  | bash`

Start in development mode:

```bash
LIVEKIT_ENABLE_DATA_TRACKS=true livekit-server --dev --bind 0.0.0.0
```

This runs a server at `ws://localhost:7880` with dev credentials (`devkey` / `secret`).

`LIVEKIT_ENABLE_DATA_TRACKS=true` is **required** - without it, the robot's telemetry DataTrack will fail to publish.

### Generate an Access Token

```bash
livekit-cli create-token \
    --api-key devkey --api-secret secret \
    --join --room robot --identity my-client \
    --valid-for 24h
```

For full LiveKit documentation, see .

## Quick Start

```python
import asyncio
import time
from livekit import rtc
from humanoid_protocol.v1.edge_cloud_pb2 import (
    CloudCommand, VelocityCommand, ModeCommand, Mode, EdgeTelemetry
)

LIVEKIT_URL = "ws://localhost:7880"
LIVEKIT_TOKEN = "your-token"
seq = 0


def next_seq():
    global seq
    seq += 1
    return seq


async def main():
    room = rtc.Room()
    await room.connect(LIVEKIT_URL, LIVEKIT_TOKEN)
    print(f"Connected to room: {room.name}")

    # Subscribe to robot's video track
    @room.on("track_subscribed")
    def on_track(track, publication, participant):
        if isinstance(track, rtc.RemoteVideoTrack):
            pass  # handle video frames

    # Subscribe to robot's telemetry DataTrack
    async def read_telemetry(track):
        stream = track.subscribe()
        async for frame in stream:
            t = EdgeTelemetry.FromString(frame.payload)
            print(f"Mode: {t.fw_mode}, Joints: {list(t.joint_pos)[:5]}...")

    @room.on("data_track_published")
    def on_data_track(track):
        asyncio.create_task(read_telemetry(track))

    # Stand up first (robot boots in DAMP).
    # Note: command Mode and telemetry FirmwareMode use different numbering -
    # always use the named constants instead of raw ints.
    stand = CloudCommand(
        timestamp_us=int(time.time() * 1e6),
        sequence=next_seq(),
        mode=ModeCommand(mode=Mode.MODE_STAND)
    )
    await room.local_participant.publish_data(
        stand.SerializeToString(), topic="commands", reliable=True
    )
    await asyncio.sleep(3)  # wait for standing pose to settle

    # Walk forward for 5 seconds at 10 Hz.
    # Velocity commands persist - once set, the robot keeps walking at the
    # last commanded velocity until you change it or send DAMP. Streaming at
    # 10 Hz here is just to demonstrate updates, not a safety requirement.
    for _ in range(50):
        cmd = CloudCommand(
            timestamp_us=int(time.time() * 1e6),
            sequence=next_seq(),
            velocity=VelocityCommand(vx=0.5, vy=0, vyaw=0)
        )
        await room.local_participant.publish_data(
            cmd.SerializeToString(), topic="commands", reliable=True
        )
        await asyncio.sleep(0.1)

    # Stop - send DAMP
    stop = CloudCommand(
        timestamp_us=int(time.time() * 1e6),
        sequence=next_seq(),
        mode=ModeCommand(mode=Mode.MODE_DAMP)
    )
    await room.local_participant.publish_data(
        stop.SerializeToString(), topic="commands", reliable=True
    )


if __name__ == "__main__":
    asyncio.run(main())
```

## Sections

- Robot Control - commands, state machine, telemetry, joint order
- Media - camera and video streaming
- Protocols - data structures, field limits, examples
- Wire Format - session channels and message shapes
