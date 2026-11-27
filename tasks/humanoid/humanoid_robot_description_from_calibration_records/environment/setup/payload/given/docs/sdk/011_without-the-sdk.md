# Without the SDK

> State, commands, camera and audio over the robot's LiveKit room with the livekit and humanoid-protocol packages alone.

- Section: Python SDK

The robot's LiveKit room carries everything the `livekit` connection mode uses: state and
commands as data messages, the camera and the microphone as tracks. Any LiveKit client can
join it. This page shows how, in Python, without `maker-sdk`; the scripts are in the
`livekit_raw/` folder of the examples.

```bash
pip install livekit humanoid-protocol
export MAKER_CREDENTIAL=...   # an SDK credential from Humanoid Manager
```

`humanoid-protocol` holds the protobuf messages; `RobotState` and `RobotCommand` are what the
SDK sends and reads. Set `MANAGER_URL` in `manager_token.py` to your Humanoid Manager
address. The role of the credential applies: an Observe credential can read state and the
tracks but its commands are dropped (Connection Modes).

> [!WARNING]
>
> **These scripts skip what the SDK does for you**
>
> No limits, no link watchdog, no zero velocity on exit and no fence between commands. Run
> `send_commands.py` only with the robot hanging from its gantry hook, and keep the humanoid robot
> Manager open at the **E-Stop**. Work through
> Before You Run a Script first.

## A Join Token

Humanoid Manager issues join tokens at `POST /api/livekit/token` to a request that carries an
SDK credential as a Bearer token. The answer names the LiveKit `url`, the `room`, the
`token`, your participant `identity` and the credential's `role`. `manager_token.py` fetches
one; it refuses redirects, so the credential is never forwarded to another host, and it
rewrites a `localhost` LiveKit URL to the manager's host, which is what a robot that runs
its own LiveKit server reports.

```python
class NoRedirect(urllib.request.HTTPRedirectHandler):
    """Refuse every redirect: urllib would send the credential on to the new address."""

    def redirect_request(self, *args: Any, **kwargs: Any) -> None:
        return None


def fetch_grant(manager: str = MANAGER_URL, credential: str = CREDENTIAL) -> dict[str, Any]:
    """The LiveKit URL, the robot's room and a join token, from Humanoid Manager."""
    if not credential:
        sys.exit("Set MAKER_CREDENTIAL to an SDK credential from Humanoid Manager.")
    request = urllib.request.Request(
        manager.rstrip("/") + "/api/livekit/token",
        data=b"{}",
        method="POST",
        headers={"Authorization": f"Bearer {credential}", "Content-Type": "application/json"},
    )
    with urllib.request.build_opener(NoRedirect).open(request, timeout=5) as response:
        grant: dict[str, Any] = json.load(response)  # url, room, token, identity, role
    # The URL is the one the robot uses itself; localhost there is Humanoid Manager's host.
    url = urlsplit(grant["url"])
    if url.hostname in ("localhost", "127.0.0.1"):
        host = urlsplit(manager).hostname or ""
        netloc = host if url.port is None else f"{host}:{url.port}"
        grant["url"] = url._replace(netloc=netloc).geturl()
    return grant


```

From `examples/livekit_raw/manager_token.py` in the maker-sdk repository (the excerpt the page shows).

The other scripts in the folder import `fetch_grant()` from this file.

## Read State

The robot publishes `RobotState` on the `state` data track at the firmware's state rate.
`read_state.py` joins, decodes each message with `RobotState.FromString` and prints the mode,
protocol version and sequence number for 5 s:

```python
async def print_states(track: rtc.RemoteDataTrack) -> None:
    async for frame in track.subscribe():
        state = humanoid_state_pb2.RobotState.FromString(bytes(frame.payload))
        mode = humanoid_common_pb2.ControlMode.Name(state.current_mode)
        print(f"{mode}  protocol {state.protocol_version}  sequence {state.sequence}")


async def main() -> None:
    grant = fetch_grant()
    room = rtc.Room()
    readers: set[asyncio.Task[None]] = set()

    def on_data_track(track: rtc.RemoteDataTrack) -> None:
        if track.info.name == "state":
            task = asyncio.ensure_future(print_states(track))
            readers.add(task)  # keep a reference so the task is not collected
            task.add_done_callback(readers.discard)

    room.on("data_track_published", on_data_track)
    await room.connect(grant["url"], grant["token"])
    try:
        await asyncio.sleep(SECONDS)
    finally:
        await room.disconnect()


```

From `examples/livekit_raw/read_state.py` in the maker-sdk repository (the excerpt the page shows).

`current_mode` is the firmware's mode enum, `error_flags` the latched fault bits, and
`active_alerts` the current alerts. The field list is in The Protocol.

## Send a Command

Commands are `RobotCommand` messages published as reliable data on the `commands` topic.
Each carries `protocol_version = 1`, a `sequence` you increment, and `timestamp_us`. The
robot drops a command whose protocol version it does not speak, and reports nothing.

A robot whose participant attribute `humanoid.commands_track` is `"1"` also reads a data track
named `commands` that you publish. Use it for a stream of setpoints, a velocity or a
trajectory: a lost frame is replaced by the next, where a lost packet holds the rest back
until it is resent. Every frame needs its `sequence`, and the robot drops a setpoint older
than a command it already took from you, so keep STAND, DAMP and a zero velocity on the topic.

`send_commands.py` keeps a guard of its own, always on, unlike `guard.py` in the SDK
examples: it listens to state for 2 s and sends nothing when the state is stale, the protocol
version is wrong, a critical alert or `error_flags` is set, the battery is protecting itself
or below 20 %, or an actuator is at 60 °C or more. The limits are constants at the top of the
file; change them as you need. If the robot is in DAMP, it sends one STAND and exits.

```python
reason = why_not_stand(*latest) if latest else "no state received"
if reason is not None:
    print("not sending STAND:", reason)
    return 1
payload = command(humanoid_common_pb2.CONTROL_MODE_STAND)
await room.local_participant.publish_data(payload, reliable=True, topic="commands")
```

From `examples/livekit_raw/send_commands.py` in the maker-sdk repository (the excerpt the page shows).

A velocity is the same message with `set_velocity` filled in. Without the SDK, nothing re-sends
it, so send zero velocity yourself before you leave the room. Humanoid Edge stops a held
velocity when a participant sends zero or leaves the room
(Watchdogs).

## Camera

The camera is the robot's video track. `camera.py` subscribes, takes one frame from a
`VideoStream`, converts it to RGB and writes `frame.ppm`:

```python
async def save_one_frame(track: rtc.Track, done: asyncio.Future[str]) -> None:
    stream = rtc.VideoStream(track)
    async for event in stream:
        rgb = event.frame.convert(rtc.VideoBufferType.RGB24)
        header = f"P6 {rgb.width} {rgb.height} 255\n".encode()
        Path(PATH).write_bytes(header + bytes(rgb.data))
        done.set_result(f"saved {PATH}, {rgb.width}x{rgb.height}")
        break
    await stream.aclose()


async def main() -> None:
    grant = fetch_grant()
    room = rtc.Room()
    done: asyncio.Future[str] = asyncio.get_running_loop().create_future()
    tasks: set[asyncio.Task[None]] = set()

    def on_track(track: rtc.Track, *_: object) -> None:
        if track.kind == rtc.TrackKind.KIND_VIDEO and not tasks:
            tasks.add(asyncio.ensure_future(save_one_frame(track, done)))

    room.on("track_subscribed", on_track)
    await room.connect(grant["url"], grant["token"])
    try:
        print(await asyncio.wait_for(done, TIMEOUT_S))
    finally:
        await room.disconnect()


```

From `examples/livekit_raw/camera.py` in the maker-sdk repository (the excerpt the page shows).

## Audio

The microphone is the robot's audio track. `audio.py` reads an `AudioStream` for a few
seconds and writes `microphone.wav`:

```python
async def record(track: rtc.Track, done: asyncio.Future[str]) -> None:
    stream = rtc.AudioStream(track)
    chunks: list[bytes] = []
    rate = channels = 0
    async for event in stream:
        frame = event.frame
        rate, channels = frame.sample_rate, frame.num_channels
        chunks.append(bytes(frame.data))
        if sum(len(c) for c in chunks) >= SECONDS * rate * channels * 2:
            break
    await stream.aclose()
    with wave.open(PATH, "wb") as wav:
        wav.setnchannels(channels)
        wav.setsampwidth(2)  # 16-bit samples
        wav.setframerate(rate)
        wav.writeframes(b"".join(chunks))
    done.set_result(f"saved {PATH}, {SECONDS:.0f} s at {rate} Hz")


async def main() -> None:
    grant = fetch_grant()
    room = rtc.Room()
    done: asyncio.Future[str] = asyncio.get_running_loop().create_future()
    tasks: set[asyncio.Task[None]] = set()

    def on_track(track: rtc.Track, *_: object) -> None:
        if track.kind == rtc.TrackKind.KIND_AUDIO and not tasks:
            tasks.add(asyncio.ensure_future(record(track, done)))

    room.on("track_subscribed", on_track)
    await room.connect(grant["url"], grant["token"])
    try:
        print(await asyncio.wait_for(done, SECONDS + TIMEOUT_S))
    finally:
        await room.disconnect()


```

From `examples/livekit_raw/audio.py` in the maker-sdk repository (the excerpt the page shows).

To play sound on the robot, publish an audio track of your own; the robot plays it on its
speaker.

## Related

- Connection Modes: the `livekit` mode and the SDK credential
- Camera and Audio: the same tracks through the SDK
- Safety: what the SDK adds on top of the wire, and what it does not
- SDK Reference: the protocol messages
- Robot API: the HTTP API of Humanoid Manager
