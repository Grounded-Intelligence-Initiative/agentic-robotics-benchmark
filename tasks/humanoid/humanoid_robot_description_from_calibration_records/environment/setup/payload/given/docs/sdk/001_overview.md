# Overview

> maker-sdk: walking, joint control, robot state, camera and audio for the humanoid robot from Python.

- Section: Python SDK

`maker-sdk` is the Python SDK for the maker's robots, and supports the humanoid robot. It drives the robot
with the robot's own verbs, `stand`, `set_velocity`, `damp` and `trajectory`, plus two the
SDK adds: `balance` and `set_joints`. The
package is `maker`; the robot is the `maker.humanoid` subpackage. It is on
PyPI and its source is on
GitHub.

Everything the SDK sends passes through the same arbiter and safety layer on the robot as
the Drive Robot page and the gamepad; nothing in it bypasses the robot.

> [!WARNING]
>
> **Before the robot walks**
>
> For a first run, hang the robot from its gantry hook with both feet on the floor and give it
> 2 m of clear floor ahead. Nothing in the SDK is an emergency stop: keep the E-Stop in the humanoid robot
> Manager open, or be ready to cut power at the battery unit. Read Safety first.

## Try It

Each script below runs on a simulated humanoid robot: press **Run** and the robot opens in the
corner of the page. Run them in order, Stand, then Balance, then Walk, as on a real robot.

**Stand:**

```python
# guard(robot)  # optional: uncomment this and the import to stop on your rule
try:
    # Returns once the robot reports STAND and has been upright for 0.5 s (armed): the
    # firmware enters MOVE only after that.
    robot.stand()
except (NotReadyError, WaitTimeoutError) as exc:  # no live state, a fault, or not armed
    print(exc)  # what the robot reports and what to do
    sys.exit(1)
print(f"robot mode {robot.get_state().mode.name}, armed {robot.armed}")
```

From `examples/stand.py` in the maker-sdk repository (the excerpt the page shows).

**Balance:**

```python
# guard(robot)  # optional: uncomment this and the import to stop on your rule
try:
    robot.balance()  # returns once the robot reports MOVE
except (NotReadyError, WaitTimeoutError) as exc:  # no live state, a fault, or not MOVE
    print(exc)  # what the robot reports and what to do
    sys.exit(1)
print(f"robot mode {robot.get_state().mode.name}, balancing in place")
```

From `examples/balance.py` in the maker-sdk repository (the excerpt the page shows).

**Walk:**

```python
# guard(robot)  # optional: uncomment this and the import to stop on your rule
mode = robot.get_state().mode
if mode is not Mode.MOVE:  # this script's own rule: walk from MOVE only
    print(f"robot mode {mode.name}: walk.py walks in MOVE only; run balance.py first")
    sys.exit(1)
try:
    # Held and re-sent at 10 Hz for DURATION_S, then zero velocity; returns after that.
    sent = robot.set_velocity(vx=VX, vy=VY, vyaw=VYAW, duration=DURATION_S)
except NotReadyError as exc:  # no live state (nothing sent), or a fault ended the walk
    print(exc)
    sys.exit(1)
if sent.clamped:
    print("the SDK reduced the speed to the robot's limits:", sent.command)
print(f"robot mode {robot.get_state().mode.name}, balancing in place")
```

From `examples/walk.py` in the maker-sdk repository (the excerpt the page shows).

**Read state:**

```python
# robot.get_state() is the latest sample the robot sent. It is a snapshot: read it again
# for newer values. A field the robot does not report is None, never a guess.
s = robot.get_state()

# Robot mode: DAMP (no actuator holds a position), STAND (a held standing pose, no
# balance) or MOVE (the walking policy balances the robot, at zero or any velocity).
print(f"robot mode   {s.mode.name}")

# Armed: the firmware accepts MOVE only after STAND has been held upright for 0.5 s.
# None means the SDK cannot tell (no gravity vector reported).
print(f"armed        {robot.armed}")

# Faulted: a critical alert latched DAMP. The robot stays in DAMP, and error_flags stay
# set, until the firmware restarts. The preflight check names what latched.
print(f"faulted      {s.faulted}")
for problem in robot.preflight("stand").problems:
    if problem.code == "faulted":
        print(f"  {problem.message}")

# Battery, from the battery management system. None when the robot reports none.
if s.battery is not None:
    b = s.battery
    print(
        f"battery      {b.soc_percent:.0f} %, {b.voltage_v:.1f} V, {b.current_a:+.1f} A, "
        f"warmest cell {b.max_cell_temp_c:.0f} °C, protecting {b.protecting}"
    )
else:
    print("battery      not reported")

# Actuators: one Joint per actuator, with the firmware's name, position (rad), velocity
# (rad/s), current (A) and temperature (°C). The firmware warns at 60 °C and latches
# DAMP at 80 °C.
temps = sorted(((j.temp, j.name) for j in s.joints if j.temp is not None), reverse=True)
if temps:
    hottest = ", ".join(f"{name} {temp:.0f} °C" for temp, name in temps[:HOTTEST])
    print(f"hottest      {hottest}")
else:
    print("hottest      actuator temperatures not reported")

# IMU: gravity as the body sees it. z is close to -1 when the robot is upright;
# upright is True below -0.8. euler is (roll, pitch, yaw) in rad from the IMU quaternion.
if s.gravity is not None:
    gx, gy, gz = s.gravity
    print(f"gravity      ({gx:+.2f}, {gy:+.2f}, {gz:+.2f}), upright {s.upright}")
if s.euler is not None:
    roll, pitch, yaw = s.euler
    print(f"orientation  roll {roll:+.2f}, pitch {pitch:+.2f}, yaw {yaw:+.2f} rad")

# Alerts the firmware reports now. Severity 0 is critical: it latches DAMP.
alerts = [f"{a.name}{' (critical)' if a.critical else ''}" for a in s.alerts]
print(f"alerts       {', '.join(alerts) or 'none'}")

# Freshness: how old the sample is. Decisions to move are made on samples at most
# 0.5 s old; robot.preflight() reports stale_state beyond that.
print(f"state age    {s.age_s * 1000:.0f} ms")

# The same fields as a stream, a few times a second.
for _ in range(STREAM_LINES):
    time.sleep(STREAM_PERIOD_S)
    s = robot.get_state()
    now = time.monotonic()
    rate = sum(1 for t in tuple(arrivals) if now - t <= 1.0)
    battery = f"{s.battery.soc_percent:.0f} %" if s.battery else "n/a"
    print(
        f"{s.mode.name:5}  armed {robot.armed}  upright {s.upright}  faulted {s.faulted}  "
        f"battery {battery}  age {s.age_s * 1000:.0f} ms  rate {rate:.0f} Hz"
    )
```

From `examples/read_state.py` in the maker-sdk repository (the excerpt the page shows).

**Camera and audio:**

```python
if robot.has("camera"):
    photo = robot.camera.photo()  # the next fresh frame, rgb8
    Path(PHOTO).write_bytes(photo.to_jpeg())
    print(f"saved {PHOTO}, {photo.width}x{photo.height}")
    clip = robot.camera.capture_clip(CLIP_S, audio=robot.has("microphone"))
    frames = clip.save_frames(CLIP_DIR)
    print(f"saved {len(frames)} frames to {CLIP_DIR}/ at {clip.fps:.1f} fps")
    if clip.audio:
        print("saved", clip.save_wav(CLIP_WAV))
else:
    print("this connection carries no camera; use hybrid or livekit")

if robot.has("speaker"):
    # 16-bit mono PCM: a sine wave at TONE_HZ for TONE_S.
    tone = b"".join(
        struct.pack("<h", int(8000 * math.sin(2 * math.pi * TONE_HZ * i / SAMPLE_RATE_HZ)))
        for i in range(int(SAMPLE_RATE_HZ * TONE_S))
    )
    robot.speaker.play_pcm(tone, sample_rate_hz=SAMPLE_RATE_HZ)
    print(f"played {TONE_S:.0f} s at {TONE_HZ:.0f} Hz")
```

From `examples/camera_and_audio.py` in the maker-sdk repository (the excerpt the page shows).

They are the Quickstart's scripts. This guide has the detail behind
every line of them; Drive from Python
gets you from `pip install` to the first walk, step by step, and the
Python SDK page is the overview.

## What It Does

| You want to | The SDK gives you |
| - | - |
| **Walk, strafe, turn** | `set_velocity(vx, vy, vyaw)`, held for a duration or until `balance()`; the robot walks in MOVE |
| **Change robot mode** | `stand()`, `balance()`, `damp()` |
| **Move joints** | `set_joints(pose)` and `trajectory(setpoint)` with the walking policy off |
| **Read the robot** | `robot.get_state()`: mode, joints, IMU, battery, alerts; waits that read it |
| **Report the facts** | `preflight()` and `maker status` report robot mode, arming, faults and alerts; a command refuses only without live state (`NotReadyError`), and the firmware decides the rest |
| **Guard your script** | `guard.py`, an example guard of your own with limits you edit; the motion examples carry it commented out |
| **See and hear** | camera frames, microphone chunks, a speaker to play into |
| **Keep a record** | every state sample and every command as JSON lines |

## Connection Modes

| Mode | Control and state | Camera and audio | Works from |
| - | - | - | - |
| `udp` | UDP on the robot's network | none | the robot's network |
| `hybrid` | UDP on the robot's network | a LiveKit room | the robot's network |
| `livekit` | a LiveKit room | a LiveKit room | wherever Humanoid Manager is reachable |

The `Robot` object is the same in all three. A saved robot remembers its mode; see
Connection Modes.

## Before You Start

The SDK drives a robot that is already assembled, powered and set up. These pages cover the
parts it depends on:

- Start the robot and
  turn it off
- Humanoid Manager: SDK credentials and the robot's settings
- Stand Up and Walk: what the robot does in each robot mode
- Standing and Balancing: the stand, balance, walk, damp sequence and why
- Stopping the Robot: the E-Stop and cutting power. Nothing in the SDK is an emergency stop
- Before You Run a Script: the checklist for every script that moves the robot

## Pages

- Quickstart: Install, check, stand, balance, walk for 3 s, damp.
- Connection Modes: udp, hybrid and livekit, and how a saved robot chooses.
- Move the Robot: stand, balance, set_velocity, what each command waits for, limits.
- Control Joints: set_joints and trajectory on a supported robot.
- Read State: One snapshot of everything the robot reports, and how to wait on it.
- Camera and Audio: Frames, clips, microphone and speaker on hybrid and livekit.
- Record a Session: State and commands as JSON lines.
- Without the SDK: The LiveKit room with livekit and humanoid-protocol only.
- Safety: The checklist before a script runs, bringing the robot to rest, what the SDK reports, and your own guard.
- Examples: Every example script in the repository.
- Command Line: maker setup, robots, status, stand, balance, walk, damp.
- SDK Reference: Every class, verb, field and error.
- Troubleshooting: What each error means and what to do.

## Requirements

Python 3.12 or newer. `pip install maker-sdk` installs `humanoid-protocol`, `protobuf`,
`livekit`, `questionary` and `rich`. numpy, Pillow and OpenCV are optional; a method that
needs one names it in its `ImportError`.

## Related

- Drive from Python: the hands-on path from a built robot to its first walk
- Python SDK: the overview in the program section
- API: the HTTP and WebSocket interface of Humanoid Manager
- maker-sdk on PyPI
- maker-sdk on GitHub
