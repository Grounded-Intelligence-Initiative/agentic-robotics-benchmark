# Connection Modes

> UDP, Hybrid and LiveKit: what each carries, what it needs, and how the SDK picks one.

- Section: Python SDK

The SDK reaches the robot in one of three connection modes. The `Robot` object and every
verb are the same in all three; the mode decides what the connection carries and where it
works from.

| Mode | Control and state | Camera and audio | Works from |
| - | - | - | - |
| `udp` | UDP on the robot's network | none | the robot's network |
| `hybrid` | UDP on the robot's network | a LiveKit room | the robot's network |
| `livekit` | a LiveKit room | a LiveKit room | wherever Humanoid Manager is reachable |

A saved robot carries a mode, so `Robot().connect()` needs no argument. Pass one to override
it for a session: `connect("udp")`.

## In Code

`connect.py` builds the connection in code. Set `MODE` at the top of the file to `"udp"`,
`"hybrid"` or `"livekit"`; the three `ConnectionConfig` shapes sit side by side:

```python
# udp: control and state over UDP on the robot's network. No camera or audio.
udp = ConnectionConfig(udp=UdpConfig(host=ROBOT_ADDRESS))

# hybrid: control and state over UDP, camera and audio over LiveKit.
hybrid = ConnectionConfig(
    udp=UdpConfig(host=ROBOT_ADDRESS),
    livekit=ManagerConfig(url=MANAGER_URL, credential=CREDENTIAL),
)

# livekit: everything through the robot's LiveKit room, wherever Humanoid Manager is reachable.
livekit = ConnectionConfig(livekit=ManagerConfig(url=MANAGER_URL, credential=CREDENTIAL))

config = {"udp": udp, "hybrid": hybrid, "livekit": livekit}[MODE]

try:
    with Robot(config).connect(MODE) as robot:
        state = robot.get_state()
        print(f"connected over {MODE} to {robot.info.endpoint}")
        print(f"robot mode {state.mode.name}, armed {robot.armed}")
        if state.battery is not None:
            print(f"battery {state.battery.soc_percent:.0f} %")
        else:
            print("battery not reported")
        capabilities = ("drive", "state", "battery", "camera", "microphone", "speaker")
        print("capabilities:", ", ".join(c for c in capabilities if robot.has(c)))
except CompatibilityError as exc:
    # The robot answered, but its model or Robot OS line is not supported.
    # Updating the SDK or robot is appropriate; blindly retrying is not.
    print(f"incompatible robot: {exc}")
except ConnectError as exc:
    # The connection itself failed: robot, credentials, room or transport.
    # Check the reported cause; a retry may make sense for a transient failure.
    print(f"could not connect: {exc}")

```

From `examples/connect.py` in the maker-sdk repository (the excerpt the page shows).

The other example scripts call `Robot()` with no config and read the environment or a saved
robot instead; see How the SDK Picks a Connection.

## UDP

Commands go to the robot on UDP port 8850 and state comes back on port 8851. There is no
media and no sign-in: anyone on the robot's network who can reach port 8850 can command it,
so trust the network before you turn it on.

The robot sends UDP state to one host. In Humanoid Manager, set the Humanoid Edge parameters
`udp-control` to on and `udp-state-host` to your computer's address
(Humanoid Manager).

One client per state port: a second script on the same computer binds
`UdpConfig(state_bind=("0.0.0.0", <port>))` and the robot's `udp-state-port` matches it.

## Hybrid

Control and state travel as in udp, so a velocity command does not cross the internet. The
camera, microphone and speaker travel over the robot's LiveKit room, which the SDK joins
with an SDK credential from Humanoid Manager. Use hybrid on the robot's network when a script
needs the camera or sound.

It needs the robot's address, its Humanoid Manager URL and an SDK credential. Issue the
credential on the **Developer** page of Humanoid Manager
(Get an SDK Credential). The
credential's role applies to the LiveKit room: a **Control** credential can talk through the
speaker, an **Observe** credential watches and listens only. Commands in hybrid travel over
UDP, which has no sign-in, so the role does not limit driving.

## LiveKit

Everything, commands included, goes through the robot's LiveKit room. Commands travel on the
data topic `commands` and state on the data track `state`, at about 10 Hz. When the robot's
Humanoid Edge reads it, a non-zero velocity and a trajectory go on the SDK's own data track
`commands` instead: a lost frame is replaced by the next one, where a lost packet would hold
the rest of the stream back until it is resent. Mode commands and a zero velocity always go
as packets. Use livekit away
from the robot's network, or when an agent framework joins the same room and the SDK is one
more participant in it.

It needs the Humanoid Manager URL and an SDK credential
(Get an SDK Credential). The SDK asks Humanoid
Manager for a room token, joins, and reports `robot.info.endpoint` as `room@url as <identity>`. The
credential's role applies:
a **Control** credential drives, an **Observe** credential watches and listens, and the robot
drops its commands.

The Humanoid Manager URL is the one you open in the browser: ``,
`` or a bare host name. The LiveKit URL Humanoid Manager hands out is
the one the robot itself uses; when it says `localhost`, the SDK substitutes the manager's
host. A manager that redirects is reported as a `ConnectError`, not followed.

Every SDK session joins the room as its own participant, `sdk-<credential id>-<host>-<6 hex>`.
`ManagerConfig(label="...")` fixes the suffix so a restarted script replaces its previous session
instead of joining beside it. To join with a token you minted yourself, pass
`LiveKitConfig(url, room, token)` instead of `ManagerConfig`; one token is one participant.
To use the room without the SDK at all, see Without the SDK.

## How the SDK Picks a Connection

`Robot()` with no argument builds its connection from the environment or a saved robot, in
this order:

1. Environment variables: `MAKER_UDP_HOST`, or `MAKER_MANAGER_URL` and `MAKER_CREDENTIAL`, or
   both pairs. They are not merged with a saved robot.
2. A saved robot from `~/.maker/robots.toml`: the one named by `MAKER_ROBOT`, else the file's
   default, else the only one.
3. Otherwise `ConnectError`, naming `maker setup` and the variables.

`MAKER_MODE` sets the mode `connect()` uses with no argument and `MAKER_LIMITS` the velocity
limits, whichever source supplied the connection. `MAKER_HOME` moves the store directory.

A saved robot holds everything one mode needs, and can hold more than one mode's fields:

```toml title="~/.maker/robots.toml"
default = "lab"

[robots.lab]
mode = "hybrid"
udp_host = "192.168.22.32"
manager_url = ""
credential = "..."
room = "robot-maker-0001"

[robots.lab.limits]
vx = 0.3
```

`udp` needs `udp_host`; `hybrid` needs `udp_host`, `manager_url` and `credential`; `livekit`
needs `manager_url` and `credential`. A mode whose field is missing raises `ConnectError`
naming the field and the `maker robots add` command that fills it. Save and change robots
with the command line; the file is created with mode 0600
because it holds SDK credentials.

## What Connect Waits For

`connect()` returns once the robot has reported state, so `robot.get_state()` is valid on the
next line. It raises `ConnectError` after `timeout` (5 s by default) without state. On hybrid and
livekit the media tracks are awaited for `media_timeout` (3 s); the capabilities the SDK claims
are the tracks that arrived.

`connect(require_state=False)` opens the room without waiting for the firmware: camera,
microphone and speaker work at once. State unlocks when the first state frame arrives; a
motion verb sent before then waits for it up to its `timeout`.

A protocol version other than the one the SDK speaks raises `ProtocolMismatchError`.
`allow_version_skew=True` connects anyway, and the robot may misread the commands the SDK
sends. Update the SDK or the robot instead.

> [!WARNING]
>
> **A lost link ends a held velocity**
>
> After 2 s without state the SDK raises `LinkLostError`, sends zero velocity if it holds one,
> and every verb raises until you `close()` and `connect()` again. On `udp` and `hybrid`, the
> robot also zeroes a velocity 2 s after the last command it received. On `livekit`, the humanoid robot
> Edge stops a held velocity when the SDK sends zero or leaves the room; see
> Watchdogs.

## Related

- Quickstart: point the SDK at the robot and run the first scripts
- Command Line: `maker setup` and `maker robots`
- Drive from Python: the SDK credential and the UDP settings, step by step
- Humanoid Manager: the robot's settings
- SDK Reference: `ConnectionConfig`, `UdpConfig`, `ManagerConfig`, `LiveKitConfig`
