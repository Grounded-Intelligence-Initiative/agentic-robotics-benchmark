# Camera and Audio

> Frames, clips, microphone and speaker over the robot's LiveKit room, on hybrid and livekit.

- Section: Python SDK

The robot publishes its camera and microphone as tracks in its LiveKit room and plays an audio
track you publish. The SDK carries them in the `hybrid` and `livekit` connection modes; `udp`
carries drive and state only. One `pip install maker-sdk` covers all of it.

## Ask First

Capabilities are what this connection carries, so test rather than assume:

```python
if robot.has("camera"):
    frame = robot.camera.photo()

robot.require("drive", "camera")   # UnsupportedError when one is missing
robot.info.capabilities            # frozenset({'drive', 'state'}) on udp
```

The full set is `drive`, `state`, `battery`, `camera`, `microphone` and `speaker`. A
capability is claimed from a track that arrived within `media_timeout` (3 s) of connecting.
A room that publishes no video makes `robot.has("camera")` `False` and `robot.camera` raise
`UnsupportedError`, rather than hand out a stream that never yields.

## A Photo, a Clip and a Tone

`camera_and_audio.py` saves a photo and a short clip with sound, then plays a tone on the
speaker. It moves nothing and needs the `hybrid` or `livekit` connection mode:

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

`photo()` returns the next fresh `Frame`, rgb8, and raises `WaitTimeoutError` when the camera
is quiet for `timeout` (5 s). `frame.to_jpeg(quality=85)` needs Pillow and `frame.to_numpy()`
needs numpy; each names the package in its `ImportError`. `capture_clip(seconds, audio=True)`
returns a `Clip` with `frames`, `audio`, `duration_s` and `fps`, and `save_wav`,
`save_frames` (Pillow) and `save_mp4` (OpenCV) to write it out.

`play_pcm(pcm_s16le, sample_rate_hz=16000, channels=1)` takes raw 16-bit samples;
`play(chunk)` takes an `AudioChunk` as the microphone yields them.

## A Stream

```python
robot.camera.latest()                          # newest Frame or None; never blocks
for frame in robot.camera.frames(timeout=5.0): # latest-wins iterator
    ...
robot.camera.subscribe(on_frame)               # one call per Frame, on the transport thread

for chunk in robot.microphone.chunks():        # pcm_s16le AudioChunks, in order
    chunk.data
robot.microphone.dropped                       # chunks a slow consumer lost
```

`Frame.age_s` says how old a frame is. A control loop that steers from the camera should
treat an old frame as no frame, send each velocity with a short `duration` so a stalled loop
ends the walk, and call `balance()` whenever it loses its target. Such a loop drives the
robot: it needs a robot balancing in MOVE with clear floor around it, and the
checklist before it runs.

## Record the Microphone

`record_audio.py` reads the microphone for 3 s and writes `microphone.wav`. It moves nothing:

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

`chunks()` yields `AudioChunk`s in order, 16-bit PCM, each with `data`, `sample_rate_hz`,
`channels` and `duration_s`; it raises `WaitTimeoutError` when the microphone is quiet for
`timeout` (5 s). A consumer that falls behind loses chunks, counted in `microphone.dropped`.

## Play a Sound

`play_audio.py` plays a WAV file, `hello.wav` in the directory you run it from, on the
robot's speaker, one second at a time. With no `hello.wav` there, it plays a one-second
440 Hz tone:

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

The file must be 16-bit PCM; `play_pcm` takes its samples with their sample rate and channel
count and returns once they are handed to the room.

## Media Before the Firmware

`connect(require_state=False)` opens the room without waiting for the robot to report state.
Camera, microphone and speaker work whether or not the firmware is running; `robot.get_state()`
and `damp()` raise `NotConnectedError` until the first state frame arrives. `stand()`,
`balance()`, `set_velocity()`, `set_joints()` and `trajectory()` wait up to their `timeout`
for it, then raise `NotReadyError` with nothing sent.

## Agents in the Same Room

A voice or vision agent framework subscribes to the robot's tracks the way it subscribes to
a person's webcam, and the SDK joins the same room as one more participant to do the
driving, on the `livekit` connection mode. Give the agent a walk bounded by a `duration` and
a `balance()` as tools, and keep the same
checklist for the agent as for a script.

## Related

- Connection Modes: hybrid and livekit, and the SDK credential they need
- Without the SDK: the same tracks with the `livekit` package alone
- Examples: every media script
- Safety: what to check before a camera-driven loop moves the robot
- SDK Reference: `Frame`, `AudioChunk`, `Clip`
