# Move the Robot

> stand, balance, set_velocity, what each command waits for, and the velocity limits.

- Section: Python SDK

Walking is one verb, `set_velocity(vx, vy, vyaw)`, held for a duration or until you end it.
Everything else on this page is about getting the robot into the robot mode that accepts it,
and ending a walk safely.

| Robot mode | What the robot does | Verb that reaches it |
| - | - | - |
| **DAMP** | every actuator compliant; the robot rests on a stool or bench, hangs from its gantry hook, or folds to the ground | `damp()` |
| **STAND** | joints stiffened into the standing pose; no balance loop | `stand()` |
| **MOVE** | the walking policy balances and walks | `balance()` from an armed STAND |

Every verb is sent in any robot mode, and the firmware decides what it does. The firmware
enters MOVE from STAND only once the robot is **armed**: STAND held upright for 0.5 s. In
DAMP, the robot drops velocity commands and nothing reports it. The SDK refuses a command
only without live state; a rule about the robot mode, a fault or a hot actuator is your
script's, as in Write Your Own Guard.

> [!WARNING]
>
> **A walking robot needs clear floor**
>
> Support the robot for a first run, hanging from its gantry hook with both feet on the floor,
> and keep 2 m of clear floor ahead. Keep Humanoid Manager open at the **E-Stop**; nothing in
> the SDK is an emergency stop. Work through
> Before You Run a Script and
> Standing and Balancing first.

## Stand Up

`stand()` puts the robot in STAND: the actuators hold a standing pose by position control
alone, without a balance loop. Call it on a robot in DAMP that hangs from its gantry hook
with both feet on the floor, as in Stand Up and Walk.
`stand()` is also sent from MOVE: hang the robot from its gantry hook or seat it on a stool
or bench first, or it tips over; see
Bringing the Robot to Rest.

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

To stop first on your own rule from `guard.py`, uncomment the two guard lines in
`stand.py`. `stand()` waits for live state, then sends STAND; without live state it
raises `NotReadyError` with nothing sent. Once sent, it reads the robot's own report and
returns when the robot is armed. It raises `WaitTimeoutError` when that takes more than 10 s
and `RobotFaultedError` when a fault latches meanwhile.

## Balance

`balance()` puts the robot in MOVE at zero velocity: the walking policy balances it in
place, and this is the only robot mode in which the robot stands free. Call it with the
robot still supported, and slacken the gantry only after it returns.

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

Outside MOVE, `balance()` sends zero velocity as soon as there is live state, which is what
puts an armed robot in MOVE, and returns once the robot reports MOVE; `WaitTimeoutError` when
it does not within `timeout` (5 s by default), saying so when the robot was not armed. From
DAMP it raises `WaitTimeoutError` at once: `the robot is in DAMP: stand() first`. In MOVE,
`balance()` is sent at once: it ends any held velocity and sends zero, so it is also how a
walk ends.

The firmware reports STAND at once but enters MOVE only after STAND has been held upright,
gravity z below -0.87, for 0.5 s. A velocity sent before that is neither refused nor
reported: the robot stays in STAND. `stand()` returns only once the robot is armed, so
`stand()` then `balance()` enters MOVE. `robot.armed` is the arming test as a property:
`True` once armed or in MOVE, `False` in DAMP or before the hold has passed, `None` with no
state, when the Robot is closed, in robot mode UNKNOWN, or in STAND when the robot reports no
gravity.

## Walk

`set_velocity()` is sent in any robot mode: in MOVE the robot walks, an armed STAND enters
MOVE, and in DAMP the robot drops it. Without live state it raises `NotReadyError` with
nothing sent. `walk.py` keeps a rule of its own: it walks in MOVE only, and exits naming
`balance.py` otherwise.

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

Axes: `vx` forward in m/s, `vy` left in m/s, `vyaw` counter-clockwise in rad/s. A walk is
`set_velocity(vx=0.2, duration=3.0)`, a strafe `set_velocity(vy=0.15, duration=3.0)`, a turn
on the spot `set_velocity(vyaw=0.3, duration=3.0)`; combine them in one call.

## Hold a Velocity

By default `set_velocity()` needs a `duration`: the SDK re-sends the velocity at 10 Hz for
that long, sends zero, and returns once the zero has gone out (`wait=True`). With
`wait=False` it returns at once with a `Sent` and the hold runs in the background, with or
without a `duration`, until something ends it:

| The hold ends when | What is sent |
| - | - |
| `duration` seconds pass | zero velocity, by the SDK |
| `balance()` | zero velocity |
| another `set_velocity()` | the new velocity |
| `stand()`, `damp()`, `trajectory()`, `set_joints()` | that verb |
| `close()`, or the end of the `with` block | zero velocity, then the link drops |
| the link is lost (no state for 2 s) | zero velocity; every verb raises until `close()` and `connect()` |
| the firmware latches a fault or restarts | nothing; the hold is dropped |

The next verb or the end of the `with` block cuts an unexpired `wait=False` hold short, so a
script that calls `set_velocity(duration=3.0, wait=False)` and returns does not walk for 3 s.

`set_velocity(hold=False)` sends exactly one packet and re-sends nothing: your own loop is
the clock, as in `stream_velocity.py`. It takes no `duration` and no `wait`, and the
live-state check runs once per call, so a live stream adds no delay between packets.

On `udp` and `hybrid`, the robot zeroes the velocity 2 s after the last packet it received;
it stays in MOVE, balancing. On `livekit`, Humanoid Edge stops a held velocity when the SDK
sends zero or leaves the room, so always bound a hold with a `duration`; see
Watchdogs.

## Drive from the Keyboard

`keyboard.py` turns key presses into short bounded holds of about 0.3 s, so releasing a key
stops the robot. `w`/`s` walk forward and back, `a`/`d` strafe, `q`/`e` turn, space sends
`balance()`, `t` stands (from MOVE it asks first whether the robot is on its gantry hook or
seated on a stool or bench), `b` damps after asking, and `x` or Ctrl-C quits with zero
velocity and `close()`. Every key is sent in any robot mode, and the status line shows what
the robot reports. Uncomment its two guard lines to run `guard(robot)` before the first
key. From DAMP, `t` then space
brings the robot to STAND and then to MOVE. A one-line status shows the robot mode, arming,
battery and the last command.

The keyboard walks the robot backward, sideways and round as well as forward: keep clear
floor all around it, not only ahead.

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

## End a Walk

`balance()` sends zero velocity. The robot stays in MOVE and the walking policy keeps it
balanced in place. End a walk this way; a `set_velocity()` with a `duration` does the same
when the duration runs out. `stand()` after walking removes the balance loop and a
free-standing robot falls; the SDK sends it as asked, so support the robot first.

To finish a session in DAMP, support the robot first, hanging from its gantry hook or seated
on a stool or bench, then run `rest.py` (MOVE, STAND, then DAMP, asking first) or `damp.py`;
see Bringing the Robot to Rest.

## Turn by Heading

`State.yaw` is the heading in radians, counter-clockwise. Turn until the heading has changed
by an angle instead of guessing a duration:

```python
import math

from maker.humanoid import Robot

with Robot().connect() as robot:
    before = robot.get_state().yaw
    robot.set_velocity(vyaw=0.3, duration=15.0, wait=False)  # NotReadyError only without live state
    robot.wait_until(
        lambda s: s.yaw is not None
        and abs(math.remainder(s.yaw - before, math.tau)) >= math.radians(90),
        timeout=15.0,
    )
    robot.balance()
```

`math.remainder` keeps the difference in (-pi, pi], so the wrap at pi does not count as a
full turn. `wait_until` checks for a fault before it evaluates the predicate and raises
`StateStaleError` when the stream goes quiet, so a robot that fell or stopped talking is
never read as "turned".

## Limits

The Motion Control Board firmware caps velocity at 0.4 m/s forward and sideways and 0.8 rad/s
turning. `Limits()` defaults to those caps, so `Sent.clamped` is `True` exactly when the
robot would not have walked at the speed you asked for; `sent.command` is the velocity that
went out. Lower limits keep a script inside a smaller envelope:

```python
from maker.humanoid import Limits, Robot

robot = Robot(limits=Limits(vx=0.2, vy=0.2, vyaw=0.4))
```

`Robot(limits=)` wins over `MAKER_LIMITS="vx,vy,vyaw"`, which wins over the saved robot's
`[robots.NAME.limits]`, which wins over the defaults. Values above the firmware caps are sent
as asked and the firmware clamps them. Negative or non-finite limits raise `ValueError`.

## Related

- Safety: the stand, balance, walk, damp sequence and why
- Read State: `robot.armed`, `wait_until`, faults
- Stand Up and Walk: the same robot modes from the Drive Robot page
- SDK Reference: `set_velocity`, `balance`, `stand`, `damp`, `Limits`
- Troubleshooting: `NotReadyError`, `WaitTimeoutError` and `RobotFaultedError`
