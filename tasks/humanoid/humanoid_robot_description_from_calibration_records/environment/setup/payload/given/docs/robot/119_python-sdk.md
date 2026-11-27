# Python SDK

> Install maker-sdk, run the example scripts and operate the robot from the command line.

- Section: Humanoid › Program the humanoid robot

`maker-sdk` drives the robot from Python: stand, balance, walk, joints, state, camera and
audio, over the robot's network or its LiveKit room. It is a client of Humanoid Edge, so every
command passes through the robot's safety layer, and the Drive Robot page and a paired gamepad
outrank it.

Two guides cover it. Drive from Python is the hands-on
path: the robot's settings, a credential, install, and a first walk with video and sound.
The Python SDK section is the depth: every connection mode, verb, error and example.

> [!WARNING]
>
> **Before the robot moves**
>
> For a first run, hang the robot from its gantry hook with both feet on the floor, and give a
> walk 2 m of clear floor ahead. Nothing in the SDK is an emergency stop: keep the
> E-Stop in Humanoid Manager open, or be ready to cut power at the battery unit. Read
> Safety before the first run.

## Install

Python 3.12 or newer.

```bash
pip install maker-sdk
maker setup
```

`maker setup` asks for a connection mode and what it needs, checks them against the robot,
and saves the robot, so every script and command below finds it without arguments. The
`udp` mode needs the robot to send UDP state to your computer; `hybrid` and `livekit` need an
SDK credential from the Developer page in Humanoid Manager
(Drive from Python).

## Run the Examples

```bash
git clone 
cd maker-sdk/examples
```

1. `python check.py` reports what the robot reports: robot mode, arming, faults, alerts,
   the hottest joint and the battery. It sends nothing.
2. `python stand.py` stands the robot up and returns once it is armed. Hang the robot from
   its gantry hook with both feet on the floor first: STAND has no balance loop.
3. `python balance.py` puts the robot in MOVE at zero velocity, where the walking policy
   balances it. Slacken the gantry only after it returns.
4. `python walk.py` walks forward at 0.3 m/s for 3 s, then balances in place. Give the robot
   2 m of clear floor ahead.
5. `python rest.py` brings the robot to rest: MOVE, then STAND, then DAMP. It asks first
   whether the robot is hanging from its gantry hook or seated on a stool or bench. `python
   damp.py` puts the robot in DAMP directly: every actuator stops holding its position, so a
   standing robot falls. Hang it from its gantry hook or seat it on a stool or bench first.
6. `python keyboard.py` drives from the keyboard: `w`/`s` walk, `a`/`d` strafe, `q`/`e`
   turn, space balances in place, `t` stands (asks first in MOVE), `b` damps after asking,
   `x` quits. It walks the robot backward and sideways too: keep clear floor all around it.

The SDK refuses a command only when there is no live state; the firmware decides the rest.
`stand.py`, `balance.py`, `walk.py` and `keyboard.py` carry `guard(robot)` from `guard.py`
commented out: an example guard of your own that stops the script on a fault, an alert, a
hot actuator or a low battery. Uncomment its two lines in a script to turn it on. The rest of the scripts, including the camera, microphone and speaker, are in
Examples; the sequence and why it is ordered this way are in
Standing and Balancing.

## Operate from the Command Line

The same verbs, without a script:

```bash
maker status
maker stand
maker balance
maker walk --vx 0.3 --duration 3
maker damp
```

`maker status` sends nothing. `maker stand`, `maker balance`, `maker walk` and `maker damp`
show the robot's facts and ask `Proceed? [y/N]` before they send, unless you pass `-y`.
`stand`, `balance` and `walk` refuse only when there is no live state. See
Command Line.

## Learn More

- Quickstart: the check, stand, balance, walk and damp scripts, line by line
- Move the Robot: velocity, holding it, limits, ending a walk
- Connection Modes: `udp`, `hybrid` and `livekit`
- SDK Reference: every class, verb and error
