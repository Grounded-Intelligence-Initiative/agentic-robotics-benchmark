# Drive from Python

> Give the Python SDK a way in, install it on your computer, then stand, balance, walk, record and play sound from short scripts.

- Section: Humanoid › Operate the humanoid robot

`maker-sdk` drives the humanoid robot from Python on your computer: the same Damp, Stand and Walk
sequence as the Drive Robot page, plus the camera, the microphone and the speaker. This page takes a
freshly built robot from its gantry hook to walking under a script, and back. Everything on
it comes from the example scripts in the SDK repository, which the
Python SDK section explains line by line.

> [!WARNING]
>
> **Nothing in the SDK is an emergency stop**
>
> Keep Humanoid Manager open at the **E-Stop** whenever the robot stands, and the battery unit
> within reach to cut power (Stopping the Robot). Every
> command a script sends passes through the robot's safety layer, and the Drive Robot page and a
> paired gamepad outrank it: end your Drive Robot session and unpair the gamepad before you run a script.

## Before You Start

- The robot is assembled, powered and signed in to Humanoid Manager, and you have driven it
  once from the Drive Robot page (Stand Up and Walk).
- The robot hangs from its gantry hook with both feet on the floor and 2 m of clear floor
  ahead. The keyboard step also walks it backward and sideways: for that, keep clear floor
  all around it.
- Your computer is on the same network as the robot and runs Python 3.12 or newer.

## Choose How the SDK Connects

The SDK reaches the robot in one of three connection modes. Pick one now; the rest of the
page tells you which step each one needs.

| Connection mode | Commands and state travel | Camera and audio | Needs |
| - | - | - | - |
| `udp` | over the robot's network | no | UDP control turned on and state sent to your computer |
| `hybrid` | over the robot's network | yes | the same, plus an SDK credential |
| `livekit` | through the robot's LiveKit room | yes | an SDK credential with the Control role |

Choose `udp` to drive only, `hybrid` to drive and use the camera and microphone from the
same script, and `livekit` when your computer is not on the robot's network. Details are in
Connection Modes.

## Get an SDK Credential

The `hybrid` and `livekit` connection modes join the robot's LiveKit room with an SDK
credential from Humanoid Manager. Skip this step for `udp`.

1. In Humanoid Manager, open **Developer** in the sidebar. Under **How a script connects**, note the **Manager URL**: it is the address the SDK uses to reach the robot.

   

   You do not need the **LiveKit URL** or the **Room**. The SDK asks the robot for both each time it connects, and when the LiveKit URL reads `localhost`, the SDK uses the robot's address instead.

2. Under **Issue a credential**, enter a **Name** for the computer that will use it, such as `my-laptop`. Issue one per computer, so you can revoke one without cutting off the others.

3. Under **What it may do**, choose **Control** to drive the robot from your computer. **Observe** watches the camera and listens to the microphone only; the robot drops its commands.

   

4. Select **Issue Credential**, then copy the credential with the copy button next to it. It is shown only this once: the robot keeps the credential's ID, not the credential, so it cannot show it again. If you lose it, revoke it and issue another.

   

5. The credential appears under **Issued credentials** with its role and ID. To take it back, select **Revoke** next to it; the computer holding it can no longer connect, and a session that is already running keeps working until its LiveKit token expires, at most 12 hours.

   

## Turn On UDP Control

The `udp` and `hybrid` connection modes carry commands over the robot's network and the
robot sends its state to one computer. Skip this step for `livekit`.

1. In Humanoid Manager, open **Advanced Settings** in the sidebar and find the **Humanoid Edge**
   section.
2. Set `udp-control` to on.
3. Set `udp-state-host` to your computer's address on the robot's network. Leave
   `udp-state-port` at its default, 8851.
4. Select **Save & Restart Humanoid Edge** and wait for the robot to come back.

## Install the SDK and Save the Robot

1. On your computer, install the package:

   ```bash
   pip install maker-sdk
   ```

2. Save the robot:

   ```bash
   maker setup
   ```

   The wizard asks for a name, the connection mode you chose, and what that mode needs: the
   robot's address for `udp`, the **Manager URL** from the Developer page and the credential
   for `livekit`, both for `hybrid`. It checks each field against the robot before it
   saves, so a wrong address or a revoked credential shows up here, not in a script.

3. Check that the SDK sees the robot:

   ```bash
   maker status
   ```

   The panel shows the robot mode, whether the state stream is fresh, the battery and any
   fault, and a READY or NOT READY badge with the first reason. It sends nothing.

4. Get the example scripts:

   ```bash
   git clone 
   cd maker-sdk/examples
   ```

## Check the Robot from a Script

`check.py` connects, prints what the robot reports (robot mode, arming, faults, alerts, the
hottest joint, battery) and `preflight()` for a stand, a walk and a trajectory, and sends
nothing. Run it before anything that moves the robot:

```python
# The SDK counts the 0.5 s a robot in STAND must be upright to arm from its own samples.
time.sleep(0.6)
s = robot.get_state()  # one sample: every fact below comes from it
print(f"robot mode {s.mode.name}, armed {robot.armed}, faulted {s.faulted}")
print(f"alerts {', '.join(a.name for a in s.alerts) or 'none'}")
temps = [j.temp for j in s.joints if j.temp is not None]
print(f"hottest joint {max(temps):.0f} C" if temps else "joint temperatures not reported")
print(f"battery {s.battery.soc_percent:.0f} %" if s.battery else "battery not reported")
print(f"state {s.age_s:.2f} s old")
for action in ("stand", "move", "trajectory"):
    print(robot.preflight(action))  # "ready to move" means live state, and the facts
```

From `examples/check.py` in the maker-sdk repository (the excerpt the page shows).

```bash
python check.py
```

The SDK refuses a command only when there is no live state, and the firmware decides the
rest. `stand.py`, `balance.py`, `walk.py` and `keyboard.py` carry `guard(robot)` from
`guard.py` commented out: an example guard of your own that stops the script on a fault, an
alert, a hot actuator or a low battery. Uncomment its two lines in a script to turn it on,
and edit its limits; see
Write Your Own Guard.

## Walk It Around

The scripts move the robot through the same three robot modes as the Drive Robot page's postures:
DAMP, STAND and MOVE. STAND holds a pose without a balance loop, so the robot stays on its
gantry hook until it is in MOVE; see Standing and Balancing.

1. With the robot in DAMP on its gantry hook, stand it up:

   ```bash
   python stand.py
   ```

   The actuators take the standing pose and the script returns once the robot is armed:
   STAND held upright for 0.5 s.

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

2. Balance it:

   ```bash
   python balance.py
   ```

   The robot enters MOVE at zero velocity and the walking policy balances it in place.

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

3. Slacken the gantry gradually until the robot carries its own weight.

4. Drive it from the keyboard. The keyboard walks the robot backward, sideways and round as
   well as forward, so keep clear floor all around it:

   ```bash
   python keyboard.py
   ```

   | Key | Effect |
   | - | - |
   | `w` / `s` | walk forward / back |
   | `a` / `d` | strafe left / right |
   | `q` / `e` | turn left / right |
   | space | balance in place |
   | `t` | stand (asks first in MOVE) |
   | `b` | damp, after asking |
   | `x` | quit with zero velocity |

   Each key press holds its velocity for about 0.3 s, so releasing a key stops the robot.
   Press space somewhere level, with the robot not mid-stride, before you go on. `walk.py`
   does the same without the keyboard: 0.3 m/s forward for 3 s, then balance in place.

```python
while True:
    show(robot, last)
    key = next_key(TICK_S)
    if key is None:
        continue
    if key in QUIT:
        break
    if key in MOVES:
        last = move(robot, key)
    elif key == " ":
        last = balance(robot)
    elif key == "t":
        last = stand(robot, next_key)
    elif key == "b":
        last = confirm_and_damp(robot, next_key)
```

From `examples/keyboard.py` in the maker-sdk repository (the excerpt the page shows).

5. Support the robot: take up the gantry's slack so it holds the robot again.

6. Damp it:

   ```bash
   python damp.py
   ```

   The script shows the robot mode and asks `Is the robot supported? Damp now? [y/N]`.
   Answer `y` only with the robot supported: every actuator stops holding its position and a
   free-standing robot falls.

```python
# damp() is never refused, so there is no check here, only the question.
if not YES:
    mode = robot.get_state().mode.name
    question = f"Robot mode {mode}. Is the robot supported? Damp now? [y/N] "
    if input(question).strip().lower() not in ("y", "yes"):
        print("nothing sent")
        sys.exit(0)
try:
    robot.damp()  # returns once the robot reports DAMP
except WaitTimeoutError as exc:  # sent, but DAMP was not reported: the message says what next
    print(exc)
    sys.exit(1)
print(f"robot mode {robot.get_state().mode.name}")
```

From `examples/damp.py` in the maker-sdk repository (the excerpt the page shows).

To go through STAND before DAMP, as the Drive Robot page does, run `python rest.py` instead: it asks
whether the robot is hanging from its gantry hook or seated on a stool or bench, then goes
MOVE, STAND, DAMP. See Bringing the Robot to Rest.

The same four steps run from the terminal without a script: `maker stand`, `maker balance`,
`maker walk --vx 0.3 --duration 3` and `maker damp`; each one asks before it sends. See
Command Line.

## Record Video

The camera, the microphone and the speaker come through the robot's LiveKit room, so this
step and the next two need the `hybrid` or `livekit` connection mode. The scripts send no
motion command and run whatever robot mode the robot is in.

`camera_and_audio.py` saves a photo, then a 3 s clip as one JPEG per frame in `clip/` with
its sound in `clip.wav`, then plays a tone on the speaker:

```bash
pip install pillow
python camera_and_audio.py
```

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

Camera and Audio covers frames, clips and MP4 output.

## Record the Microphone

`record_audio.py` records 3 s from the robot's microphone into `microphone.wav`:

```bash
python record_audio.py
```

```python
if not robot.has("microphone"):
    print("this connection carries no microphone; use hybrid or livekit")
    sys.exit(1)
# Every chunk in order, as 16-bit PCM. chunks() raises WaitTimeoutError when the
# microphone says nothing for its timeout.
chunks: list[AudioChunk] = []
recorded_s = 0.0
for chunk in robot.microphone.chunks(timeout=5.0):
    chunks.append(chunk)
    recorded_s += chunk.duration_s
    if recorded_s >= SECONDS:
        break
with wave.open(PATH, "wb") as wav:
    wav.setnchannels(chunks[0].channels)
    wav.setsampwidth(2)  # 16-bit samples
    wav.setframerate(chunks[0].sample_rate_hz)
    wav.writeframes(b"".join(c.data for c in chunks))
print(f"saved {PATH}, {recorded_s:.1f} s at {chunks[0].sample_rate_hz} Hz")
```

From `examples/record_audio.py` in the maker-sdk repository (the excerpt the page shows).

## Play a Sound on the Speaker

`play_audio.py` plays a 16-bit PCM WAV file, `hello.wav`, through the robot's speaker in 1 s
chunks. It reads `hello.wav` from the directory you run it from; with no file there, it plays a
one-second 440 Hz tone. Over `hybrid` and
`livekit`, the credential needs the Control role for the speaker:

```bash
python play_audio.py
```

```python
if not robot.has("speaker"):
    print("this connection carries no speaker; use hybrid or livekit")
    sys.exit(1)
if os.path.exists(PATH):
    with wave.open(PATH, "rb") as wav:
        if wav.getsampwidth() != 2:
            print(f"{PATH} is not 16-bit PCM; convert it first")
            sys.exit(1)
        rate, channels = wav.getframerate(), wav.getnchannels()
        pcm = wav.readframes(wav.getnframes())
    name = PATH
else:  # 16-bit mono PCM: one second of a 440 Hz sine wave
    rate, channels, name = 16_000, 1, f"a 440 Hz tone (no {PATH} here)"
    pcm = b"".join(
        struct.pack("<h", int(8000 * math.sin(2 * math.pi * 440 * i / rate)))
        for i in range(rate)
    )
# Sent in short pieces, in order: play_pcm() blocks until the room has taken each one,
# so the file plays at its own pace.
step = int(rate * CHUNK_S) * 2 * channels
for start in range(0, len(pcm), step):
    robot.speaker.play_pcm(pcm[start : start + step], sample_rate_hz=rate, channels=channels)
print(f"played {name}, {len(pcm) / (2 * channels * rate):.1f} s at {rate} Hz")
```

From `examples/play_audio.py` in the maker-sdk repository (the excerpt the page shows).

## Related

- Python SDK: every page of the SDK guide
- Quickstart: the stand, balance and walk scripts, line by line
- Safety: what the SDK reports, your own guard, and the watchdogs
- Stand Up and Walk: the same sequence from the Drive Robot page
- Humanoid Manager: where credentials live
