# Media

> Camera, video, and bidirectional audio for the humanoid robot.

- Section: Humanoid › Program the humanoid robot › Humanoid API

The humanoid robot streams video and audio in real-time via WebRTC.

## Camera

| Spec | Value |
| - | - |
| Capture rate | 30 fps |
| Resolution | 1280x720 (requested - actual depends on hardware) |
| Encoding | H.264 |
| Streaming | LiveKit video track (WebRTC) |

The robot captures frames from its USB camera at 30fps and publishes them as a LiveKit video track. Any participant in the LiveKit room can subscribe to the track to receive the stream.

The camera must be plugged in before the robot starts - the handle is grabbed once on boot.

## Subscribing to Video

```python
import asyncio
from livekit import rtc


async def main():
    room = rtc.Room()
    await room.connect(livekit_url, token)

    @room.on("track_subscribed")
    def on_track(track, publication, participant):
        if track.kind == rtc.TrackKind.KIND_VIDEO:
            # process video frames
            pass

    # Keep connection alive
    await asyncio.sleep(10)


if __name__ == "__main__":
    asyncio.run(main())
```

## Audio

The robot supports bidirectional audio over LiveKit.

| Direction | Spec | Description |
| - | - | - |
| Robot -> You | native device rate, 16-bit, mono | Microphone capture from the robot. Rate is auto-detected (e.g. 16 kHz on ReSpeaker XVF3800). LiveKit resamples on the client - request any rate via `AudioStream(sample_rate=...)`. |
| You -> Robot | 48 kHz, 16-bit, mono | Played through the robot's speaker |

### Subscribing to Robot Audio

```python
import asyncio
from livekit import rtc


async def handle_audio(track):
    stream = rtc.AudioStream(track, sample_rate=48000, num_channels=1)
    async for event in stream:
        pcm_data = bytes(event.frame.data)  # int16 PCM
        # process audio frames
    await stream.aclose()


async def main():
    room = rtc.Room()
    await room.connect(livekit_url, token)

    @room.on("track_subscribed")
    def on_track(track, publication, participant):
        if track.kind == rtc.TrackKind.KIND_AUDIO:
            asyncio.ensure_future(handle_audio(track))

    await asyncio.sleep(10)


if __name__ == "__main__":
    asyncio.run(main())
```

### Sending Audio to the Robot

Publish an audio track to the LiveKit room. The robot subscribes and plays it through its speaker.

```python
import asyncio
from livekit import rtc


async def main():
    room = rtc.Room()
    await room.connect(livekit_url, token)

    source = rtc.AudioSource(sample_rate=48000, num_channels=1)
    track = rtc.LocalAudioTrack.create_audio_track("my-audio", source)
    await room.local_participant.publish_track(
        track,
        rtc.TrackPublishOptions(source=rtc.TrackSource.SOURCE_MICROPHONE),
    )

    # Push 10ms frames of int16 PCM
    frame = rtc.AudioFrame.create(sample_rate=48000, num_channels=1, samples_per_channel=480)
    # fill frame.data with your audio samples
    await source.capture_frame(frame)

    await asyncio.sleep(10)


if __name__ == "__main__":
    asyncio.run(main())
```

See the Humanoid API overview for connection setup.
