# SDK Reference

> The public surface of maker-sdk: Robot, connection configs, preflight, Sent, State, media, the store, transports and errors.

- Section: Python SDK

Everything here is importable from `maker.humanoid` unless a module is named.
`maker.__version__` is the distribution version. Units are metres per second, radians per
second, radians, degrees Celsius.

## Where to Start

| Do this | Use |
| - | - |
| say where the robot is | `Connection`, `Robot` |
| read the facts | `preflight`, `armed` |
| balance in place, walk, strafe, turn | `balance`, `set_velocity` |
| stand up, or go compliant | `stand`, `damp` |
| pose a joint | `set_joints`, `trajectory` |
| wait for something | `wait_until`; `stand`, `balance` and `damp` wait for their robot mode themselves |
| read what it reports | `State`, `get_state` |
| check what it can do | `has`, `require`, `RobotInfo` |
| see what went out | `Sent`, `Outcomes` |
| pictures and sound | `Media` |
| record a run | `record` |
| save a robot for next time | `Store and CLI` |
| handle failure | `Errors` |

## Connection

```python
ConnectionConfig(udp=None, livekit=None, limits=None, mode=None, name=None, hints={})
```

| Member | |
| - | - |
| `udp: UdpConfig \| None` | the robot's address on the local network |
| `livekit: ManagerConfig \| LiveKitConfig \| None` | the room, through Humanoid Manager or directly |
| `limits: Limits \| None` | velocity limits for this robot |
| `mode: ConnectMode \| None` | the connection mode `connect()` uses when given none |
| `name` | the saved robot this config came from, if any |
| `available_modes() -> tuple[ConnectMode, ...]` | which of `"udp"`, `"hybrid"`, `"livekit"` this config can reach |
| `default_mode() -> ConnectMode` | `mode` when set, else `only_mode()` |
| `only_mode() -> ConnectMode` | the mode the set slots imply: `udp` alone is udp, `livekit` alone is livekit, both are `hybrid`; `ConnectError` with none set |
| `ConnectionConfig.from_environment(environ=None, *, robot=None)` | from `MAKER_*`, else the saved robot |
| `transport_for(mode, ...)` | builds the transport for a mode |

| Config | |
| - | - |
| `UdpConfig(host, command_port=8850, state_bind=("0.0.0.0", 8851), state_source=None)` | commands to `host:8850`, state on `state_bind`. `state_source` pins the one address state may come from; without it, samples that do not match this robot are dropped |
| `ManagerConfig(url, credential, label=None, timeout=5.0)` | Humanoid Manager mints the room and a join token per connect. `.host`; `.resolve()`; `.check() -> ManagerGrant(url, room, identity, role)` validates the credential without joining |
| `LiveKitConfig(url, room, token)` | room details you hold; `token` is a string or a callable minting one per join. One token is one participant |
| `default_label()` | the label the SDK uses for this host |

The LiveKit identity is `sdk-<credential id>-<host>-<6 random hex>`; `robot.info.endpoint`
reads `room@url as <identity>` on `livekit` and `host:8850 + room@url as <identity>` on
`hybrid`. Two sessions with the same label evict each other.

## `Robot`

```python
Robot(source: ConnectionConfig | Transport | None = None, *, limits=None, link_timeout=2.0)
```

`None` resolves the robot from the environment, else the saved robot. `limits` here wins
over `MAKER_LIMITS`, which wins over the saved robot's limits, which win over `Limits()`.

### `connect`

```python
connect(mode=None, *, timeout=5.0, media_timeout=3.0, connect_timeout=10.0,
        allow_version_skew=False, require_state=True, persist=None) -> Robot
```

Attach on `"udp"`, `"hybrid"` or `"livekit"`; `mode` may be left out when the config names
one or can reach only one. Returns once the first `State` has arrived and its protocol
version matches. Raises `ConnectError` before touching the network when the config lacks
what the mode needs, naming the `maker robots add` command that fixes it. `media_timeout`
bounds the wait for camera, microphone and speaker tracks. `require_state=False` returns as
soon as the connection is open. `persist=True`, or `MAKER_PERSIST=1`, saves the robot after
a successful connect with a `ManagerConfig`. `close()` then `connect()` again to switch
modes.

`open(*, timeout=5.0, allow_version_skew=False, require_state=True)` does the same over a
`Transport` passed to the constructor. `RuntimeError` on an open robot.

### Preflight

```python
Action = Literal["stand", "move", "trajectory"]
preflight(action: Action = "move") -> Preflight
armed: bool | None
```

`preflight` is a report of the facts. It reads the latest state, sends nothing, never waits
and never raises for a robot condition. `action` is `"stand"`, `"move"` (`balance()` and
`set_velocity()`) or `"trajectory"` (`set_joints()` and `trajectory()`). Only the blocking
codes, which all mean no live state, stop a command; see Verbs. `armed` is `True` in MOVE and in
STAND once the robot has been upright (gravity z below -0.87) for 0.5 s since STAND began;
`False` in DAMP or before; `None` with no state, closed, in UNKNOWN, or in STAND with no
gravity.

| `Preflight` member | |
| - | - |
| `action` | the action checked |
| `problems: tuple[Problem, ...]` | blocking first, then information |
| `state: State \| None` | the sample the check read |
| `armed: bool \| None` | |
| `ok: bool` | no blocking problem: there is live state |
| `blocking: tuple[Problem, ...]` | `not_connected`, `no_state` or `stale_state` |
| `has(code) -> bool` | |
| `str()` | `ready to move` or `not ready to move:`, followed by one `  - code: message` line per problem, information suffixed ` (information)` |
| `explain()` | one line: `not ready to move: message (code); fix. ...`, blocking problems only; the message of the `NotReadyError` a command raises |

`Problem(code, message, blocking)`; `str()` is `code: message`. Blocking: `not_connected`,
`no_state`, `stale_state`. Information, which nothing in the SDK acts on: `faulted` (the
alerts that latched DAMP, named), `alerts` (active firmware alerts, named) and `not_armed`
(`move` from STAND before the arming hold). The codes and their meaning are in
What the SDK Reports. State older than 0.5 s is stale;
armed means gravity z below -0.87 for 0.5 s. The SDK holds no joint temperature or battery
threshold: write your own rule, as in Write Your Own Guard.

### Verbs

Each returns a `Sent`. The robot reports no per-command verdict: `Sent.outcome` stays
`None` and `wait_outcome()` returns `Unknown` in every mode. Read the effect from
`robot.get_state()`.

Every verb is sent in any robot mode, and the firmware decides what it does. The robot mode,
a latched fault, an alert, a joint temperature or the battery never stop one. The one
refusal is no live state: `set_velocity`, `stand`, `balance` outside MOVE, `set_joints` and
`trajectory` read one cached sample before they send, which adds no delay on a live stream.
Without one (`no_state`, `stale_state`) they wait up to `timeout`, then raise
`NotReadyError` with nothing sent. After a STAND sent moments ago whose report has not
arrived, they wait up to 1 s for that report, then send anyway. A closed `Robot` raises
`NotConnectedError`, a lost link `LinkLostError` and a protocol version mismatch
`ProtocolMismatchError`, at once. `balance` in MOVE and `damp` are sent at once. A wait that
sees a latched fault after the command was sent raises `RobotFaultedError` with `.sent` set.

#### `set_velocity`

```python
set_velocity(vx=0.0, vy=0.0, vyaw=0.0, *, duration=None, wait=None, hold=True,
             timeout=None) -> Sent
```

Walk. Sent in any robot mode: in MOVE the robot walks; in an armed STAND it enters MOVE;
in DAMP the robot drops it. Without live state it waits up to `timeout` (5 s by default),
then raises `NotReadyError` with nothing sent. Clamped to `limits`; re-sent
at 10 Hz until `balance()`, another verb, `close()`, or `duration` seconds pass, when zero
is sent. `wait` defaults to `True`: block until the hold has ended and its zero has gone
out, or another verb took over; that needs a `duration`, else `ValueError`. `wait=False`
returns once sent and keeps the velocity until the next command, or until `duration`.
`hold=False` sends exactly one packet and holds nothing, for a loop that clocks its own
commands; the live-state check is made once, as in `trajectory()`, and `duration` or `wait=True` with
it is a `ValueError`. `ValueError` on a non-finite value or a non-positive duration. See
Move the Robot.

#### `balance`

```python
balance(*, timeout=5.0, wait=True) -> Sent
```

Zero velocity: the robot is in MOVE and the walking policy balances it in place. In MOVE it
is sent at once; it ends any velocity hold and sends zero, so this is how a walk ends.
Outside MOVE it sends zero velocity as soon as there is live state, without waiting for
arming. From an armed STAND that puts the robot in MOVE; with `wait`, it returns once the
robot reports MOVE, and raises `WaitTimeoutError` when the whole call passes `timeout`, saying
so when the robot was not armed. From DAMP it raises `WaitTimeoutError` at once: "the robot
is in DAMP: stand() first". `RobotFaultedError` when the wait sees a latched fault. Not an
emergency stop.

#### `stand`

```python
stand(*, timeout=10.0, wait=True) -> Sent
```

Put the robot in STAND: the actuators hold the standing pose, without a balance loop. Sent
from any robot mode. From MOVE, hang the robot from its gantry hook or seat it on a stool or
bench first: a free-standing robot tips over. Sends STAND once and, with `wait`, returns when the robot
reports STAND and is armed; `WaitTimeoutError` when the whole call passes `timeout`,
`RobotFaultedError` when a fault latches meanwhile. `wait=False` returns once sent. Ends a
running `set_joints()` and a held velocity.

#### `damp`

```python
damp(*, timeout=5.0, wait=True) -> Sent
```

Every actuator compliant now. A standing robot folds to the ground, so hang it from its
gantry hook or seat it on a stool or bench first. Sent at once, whatever the robot's state.
With `wait`, returns once the robot reports DAMP or FAULT_DAMP, `WaitTimeoutError` after
`timeout`. Not an emergency stop: in an emergency use the E-Stop in Humanoid Manager, or cut
power at the battery unit.

#### `set_joints`

```python
set_joints(positions, *, duration=2.0, hz=50.0, kp=None, kd=None,
           wait=True, tolerance=0.05, timeout=None) -> Sent
```

Sent in any robot mode; the firmware follows a trajectory in MOVE or an armed STAND. Waits
for live state first (a stale pose is waited out), then interpolates from the last reported pose to `positions` over `duration`, clocking
`trajectory()` setpoints from a thread at `hz`, then holds the target until another verb.
Any verb from any thread ends it before its next setpoint leaves. `wait=True` blocks until
every joint is within `tolerance` rad, `WaitTimeoutError` otherwise. An ankle target more
than 0.02 rad past the firmware's limits (0.35 rad pitch, 0.1 rad roll) is a `ValueError`
before anything is sent. `timeout` bounds the
whole call; left `None`, the live-state check may wait 5 s and the target `duration + 2` s. See
Control Joints.

#### `trajectory`

```python
trajectory(positions, *, kp=None, kd=None, timeout=0.0) -> Sent
```

One setpoint, radians, firmware order; clock these yourself or use `set_joints`. Sent in any
robot mode. Checks for live state once, with no delay on a live stream, so a loop at 50 Hz
can call this. On a stale stream it raises `NotReadyError` at once and sends nothing; pass
`timeout` to wait that long instead.
`ValueError` unless `len(positions) == info.dof`, or when only one of `kp` and `kd` is
given. A gain of 0 is not "no gain": the firmware substitutes its damping constants, so the
joint does not hold a position. `None` applies the robot's per-joint gain table. The robot
enters DAMP 2 s after the last setpoint.

### Waits

Every wait reads the robot's own report, checks for a fault before its predicate, and
refuses to succeed on a stale stream. `stand()`, `balance()`, `damp()` and
`set_joints(wait=True)` wait this way for their own effect.

#### `wait_until`

```python
wait_until(predicate, *, timeout=10.0, stale_after=None, poll=0.05) -> State
```

Blocks until `predicate(state)` is true and returns that first `State`. Raises
`WaitTimeoutError` (the message names the robot mode and the age of the last sample),
`StateStaleError` when the stream goes quiet for `stale_after`, and `RobotFaultedError`
when a fault latches. `stale_after` defaults to `link_timeout`. A wait for one robot mode is
`wait_until(lambda s: s.mode is Mode.MOVE)`.

### Properties and Callbacks

| Name | Type | Notes |
| - | - | - |
| `config` | `ConnectionConfig` | what this Robot was bound to |
| `info` | `RobotInfo` | transport, endpoint, dof, joint names, protocol version, limits, capabilities |
| `has(cap)`, `require(*caps)` | `bool`, raises `UnsupportedError` | this connection's set: `drive`, `state`, `battery`, `camera`, `microphone`, `speaker` |
| `camera`, `microphone`, `speaker` | `Camera`, `Microphone`, `Speaker` | see Media; `UnsupportedError` when the connection does not carry them |
| `record(path)` | `Recording` | context manager: every state sample and every command as JSON lines |
| `get_state()` | `State` | the latest sample; `NotConnectedError` before the first |
| `armed` | `bool \| None` | see Preflight |
| `connected` | `bool` | open and not `LinkLostError` |
| `outcomes()` | `Iterator[Refused]` | received refusals, oldest first; empty with this robot |
| `on_state` | `Callable[[State], None] \| None` | every accepted sample, on the transport thread |
| `on_alert` | `Callable[[Alert, str], None] \| None` | an alert `"raised"` or `"cleared"` |
| `on_mode_change` | `Callable[[Mode, Mode], None] \| None` | before, after |
| `on_refused` | `Callable[[Refused], None] \| None` | never fires with this robot |
| `on_link_lost` | `Callable[[LinkLostError], None] \| None` | |
| `close()` | | zero velocity if held, drop the link; idempotent; never raises |

`Robot` is a context manager; `__exit__` calls `close()`.

## `Sent`

| Member | Type | Meaning |
| - | - | - |
| `name` | `str` | the verb: `set_velocity`, `balance`, `stand`, `damp`, `trajectory`; `set_joints()` sends its setpoints as `trajectory` |
| `sequence` | `int` | the command's sequence number |
| `command` | `Velocity \| ModeCommand \| Trajectory` | what went out, after clamping |
| `clamped` | `bool` | `command` differs from what you asked |
| `sent_at` | `float` | `time.monotonic()` |
| `outcome` | `Applied \| Refused \| None` | non-blocking; `None` while pending |
| `wait_outcome(timeout=None)` | `Applied \| Refused \| Unknown` | `None` uses the transport default |
| `require(timeout=None, *, unknown_ok=True)` | as above | raises `CommandRefusedError`; with `unknown_ok=False` also `OutcomeUnknownError` |

## Outcomes

| Type | Fields |
| - | - |
| `Applied` | `sequence` |
| `Refused` | `sequence`, `reason: Refusal`, `detail: str` (diagnostic text; never branch on it), `verb` |
| `Unknown` | `sequence`, `waited_s`, `verb` |

`Refusal` (IntEnum): `UNSPECIFIED`, `FW_DAMPED`, `FAULT_DAMPED`, `GATE`, `UNSUPPORTED_SOURCE`,
`BAD_LENGTH`, `EMPTY_TRAJECTORY`, `NON_FINITE`, `PARSE`, `UNKNOWN_COMMAND`, `UNKNOWN_MODE`,
`SHUTTING_DOWN`, `NO_CAMERA`, `NOT_ACTIVE`, `UNRECOGNIZED`; `retryable` property;
`Refusal.from_wire()`.

## State

```python
@dataclass(frozen=True)
class State:
    mode: Mode                                   # DAMP | STAND | MOVE | FAULT_DAMP | UNKNOWN
    joints: tuple[Joint, ...]                    # firmware order
    gravity: tuple[float, float, float] | None   # body frame; z near -1 when upright
    gyro: tuple[float, float, float] | None      # rad/s
    quat: tuple[float, float, float, float] | None   # w, x, y, z
    error_flags: int
    alerts: tuple[Alert, ...]
    sequence: int
    fw_timestamp_us: int
    protocol_version: int
    battery: Battery | None                      # None when the robot reports no battery
    received_at: float
    edge_timestamp_us: int                       # Humanoid Edge's wall clock (µs); livekit only, 0 on udp and hybrid
    # properties
    age_s: float
    upright: bool | None                         # gravity z < -0.8; None without gravity
    euler: tuple[float, float, float] | None     # roll, pitch, yaw (rad) from quat
    yaw: float | None                            # rad, counter-clockwise, (-pi, pi]
    faulted: bool                                # FAULT_DAMP, error_flags or any critical alert
    joint_pos: tuple[float, ...]
    def joint(self, name: str) -> Joint          # KeyError on an unknown name
```

`Joint(name, pos, vel, current, temp)`: `vel`, `current` and `temp` are `None` when the robot
did not report them. `Alert(id, severity, value, threshold, source_id, first_set_us=0)` with
`critical` (`severity == 0`) and `name`. `Battery(voltage_v, current_a, soc_percent,
max_cell_temp_c, protection: BatteryProtection)` with `charging` and `protecting`;
`BatteryProtection` is an `IntFlag`.

`RobotInfo(transport, endpoint, dof, joint_names, protocol_version, limits, capabilities)`
with `joint_index(name)`, `has(capability)` and a readable `str()`.

`Limits(vx=0.4, vy=0.4, vyaw=0.8)`: the defaults are the firmware caps (0.4 m/s, 0.4 m/s,
0.8 rad/s).
`Limits.parse("0.3,0.3,0.6")`. Negative or non-finite values raise `ValueError`.

## Media

```python
robot.camera.photo(*, timeout=5.0) -> Frame     # one fresh frame; WaitTimeoutError when quiet
robot.camera.latest() -> Frame | None           # newest frame; None before the first
robot.camera.frames(*, timeout=5.0)             # iterator, always the newest frame
robot.camera.subscribe(cb)                      # cb(Frame) on the transport thread
robot.camera.stream(*, timeout=5.0)             # every frame, in order
robot.camera.capture_clip(seconds, audio=True) -> Clip
robot.microphone.chunks(*, timeout=5.0)         # every AudioChunk in order; .dropped counts loss
robot.speaker.play(chunk)
robot.speaker.play_pcm(pcm_s16le, sample_rate_hz=16000, channels=1)
```

`Frame(width, height, encoding, data, stride_bytes, key_frame, frame_id, timestamp_ns,
sequence)` with `shape`, `age_s`, `to_numpy()`, `to_jpeg(quality=85)`.
`AudioChunk(sample_rate_hz, channels, samples_per_channel, encoding, data, stream_id,
timestamp_ns, sequence)` with `duration_s`, `to_numpy()`. `Clip(frames, audio, started_at)`
with `duration_s`, `fps`, `save_wav(path) -> Path`, `save_frames(dir)`,
`save_mp4(path, *, fps=None) -> Path`, `frames_as_numpy()`. numpy, Pillow and OpenCV are
needed only at the call that uses them, and named in the `ImportError`.

## Recording

```python
with robot.record("run.jsonl") as rec:
    ...
rec.samples, rec.commands_written

from maker.humanoid import recording
for line in recording.load("run.jsonl"):        # dicts with kind "state" | "sent", t, mode
    ...
```

## Store and CLI

`~/.maker/robots.toml` (`MAKER_HOME` moves it), `0600` in a `0700` directory.

| | |
| - | - |
| `RobotStore()` | `get(name=None)`, `put(robot, *, default=None, allow_manager_change=False)`, `remove(name)`, `use(name)`, `names`, `save()`; `len()`, iteration and `in` |
| `StoredRobot(name, manager_url=None, credential=None, room=None, mode=None, udp_host=None, limits=None)` | `resolved_mode()`, `missing(mode)`, `fix_command(mode)`, `manager(label=)`, `connection(label=)` |
| `maker.humanoid.store.persist(config, room, *, name=None, mode=None)` | what `connect(persist=True)` calls: records the manager, credential, room, `udp_host`, `mode` and `limits`; `ValueError` without a `ManagerConfig` |

| Command | |
| - | - |
| `maker setup [--no-check]` | wizard: name, mode, that mode's fields, live checks |
| `maker robots [--json]` | the saved robots |
| `maker robots add NAME --mode --udp --manager --credential --limits VX,VY,VYAW [--default] [--no-check] [--no-input]` | add or update; unspecified values are kept |
| `maker robots remove NAME`, `maker robots use NAME` | forget, make default |
| `maker status [--watch] [--json]` | readiness, robot mode, battery, hottest joint, faults, alerts, state rate |
| `maker stand [-y]`, `maker balance [-y]`, `maker walk --vx --vy --vyaw --duration S [-y]`, `maker damp [-y]` | the verbs; each prints a plan with the robot's facts and asks `Proceed? [y/N]`, `-y`/`--yes` goes ahead without asking. `Not feasible:` only without live state; `balance` in MOVE sends at once without asking |
| `maker --robot NAME --mode MODE`, `maker --version` | |

Environment: `MAKER_ROBOT`, `MAKER_MODE`, `MAKER_UDP_HOST`, `MAKER_MANAGER_URL`,
`MAKER_CREDENTIAL`, `MAKER_LIMITS`, `MAKER_HOME`, `MAKER_PERSIST`. `MAKER_MANAGER_URL` and
`MAKER_CREDENTIAL` go together; one alone is a `ConnectError`. Environment wins over the
saved robot and is not merged with it. See Command Line.

## Transports

`UdpTransport(host, *, command_port, state_bind, state_source)`,
`LiveKitTransport(url, room, *, token, media_timeout, connect_timeout)`,
`HybridTransport(host, *, livekit_url, room, token, command_port, state_bind, ...)`. All
implement `Transport`; a `ConnectionConfig` builds the right one for the mode.
`LiveKitTransport.identity` and `.room` read back what the token claimed.

## Errors

```
MakerError
  ConnectError
    ProtocolMismatchError(expected, observed)
  NotConnectedError
  LinkLostError
  NotReadyError(message, *, action, preflight)      # .problems, .has(code)
    RobotFaultedError(message, *, state, sent=None)  # plus action, preflight
  WaitTimeoutError(message, *, last, sent=None)     # also TimeoutError
    StateStaleError
  CommandRefusedError(refused)
  OutcomeUnknownError(unknown)      # also TimeoutError
  UnsupportedError(capability, transport)
```

`NotReadyError` means there was no live state to send against (`no_state` or
`stale_state`) within the command's `timeout`; nothing was sent. A closed `Robot` raises
`NotConnectedError`, a lost link `LinkLostError`, a protocol version mismatch
`ProtocolMismatchError`. It reads `not ready to <action>: <message> (<code>); <fix>`, with ` (waited <t> s)` appended when the command waited. `RobotFaultedError` is raised
by a wait that saw a latched fault after the command went out; `sent` is set. `WaitTimeoutError.sent` is the command whose effect
did not arrive. Caller
mistakes are builtins: `ValueError` (non-finite velocity, limit or duration, wrong trajectory
length, a lone kp or kd, an unknown connection mode), `KeyError` (unknown joint name),
`RuntimeError` (`open()` on an open robot). A mode the config cannot reach raises
`ConnectError`.

## Threading

`Robot` is synchronous and thread-safe. Callbacks run on the transport thread, the one that
delivers state: keep them short. `robot.damp()` from a callback, such as `on_alert` damping
on an alert, sends DAMP and returns at once. The other commands that wait for the robot, and
`wait_until()`, raise `RuntimeError` there with nothing sent; pass `wait=False`, or call them
from your own thread.

## Per-Robot Tables

`maker.humanoid.robots.HUMANOID_BIPED_JOINTS` is the 25-actuator order of the humanoid robot biped
firmware profile, chosen by the reported joint count. `maker.humanoid.robots.PROTOCOL_VERSION`
is the protocol version the SDK was built against; `connect()` compares it with what the robot
echoes and raises `ProtocolMismatchError` on a difference.

## The Protocol

Commands on udp go to port 8850 and state arrives on 8851, each datagram one bare
`humanoid.io.RobotCommand` or `RobotState`. On livekit, commands are data messages on the
`commands` topic and state is the `state` data track. A robot whose participant attribute
`humanoid.commands_track` is `"1"` also reads a `commands` data track from each client: the SDK
sends a non-zero velocity and a trajectory there, and Humanoid Edge orders it against the topic
by sequence number, dropping a setpoint older than a command it already took. Every command carries
`protocol_version = 1`, a sequence number and `timestamp_us`. UDP commands are not
authenticated; the SDK credential and its role apply to the LiveKit room: every command on
livekit, and the camera and audio on hybrid. Without the SDK
and the `livekit_raw/` examples speak it with the `livekit` and `humanoid-protocol` packages
alone.
