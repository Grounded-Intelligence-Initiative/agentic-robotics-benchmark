# Safety

> What to check before a script runs, how to stand, balance and rest the robot, what to do when something goes wrong, what the SDK, the robot and the firmware each stop, what the SDK reports, and how to write your own guard.

- Section: Python SDK

The SDK is a client of the robot. It sends what you ask, bounded by your limits, and reads
back what the robot reports. Safety is the firmware's job, command handling is the robot's,
and a guard is your script's: the SDK does not refuse a command because of what the robot
reports. When in doubt, open Humanoid Manager and press the E-Stop.

## Before You Run a Script

1. **Support the robot**, as Supporting the Robot
   describes: hanging from its gantry hook with both feet on the floor for a first stand and
   walk, hanging from its gantry hook or seated on a stool or bench for joint control, for
   STAND from MOVE and for DAMP. A walking robot needs 2 m of clear floor ahead of it.
2. **Clear the area.** Keep people, cables and furniture out of the robot's reach and keep the
   robot in sight.
3. **Have a way to stop the robot open.** Keep Humanoid Manager open with its **E-Stop** in the
   sidebar or the phone top bar (it needs no drive session), and the battery unit within
   reach to cut power. Stopping the Robot explains each.
4. **Make sure nothing else is driving.** End any Drive Robot session and unpair the gamepad. Both
   outrank a script, and a paired gamepad holds control even while idle.
5. **Check the robot.** `python check.py` or `maker status` reports battery, actuator
   temperatures, faults, alerts and whether the state stream is fresh. Neither sends
   anything. Decide for yourself whether to go ahead, or let your own guard
   decide.
6. **Start slow.** Pass `Robot(limits=Limits(vx=0.2, vy=0.2, vyaw=0.4))` or save limits on the
   robot, and use short `duration` values until the script behaves.

> [!WARNING]
>
> **Nothing in the SDK is an emergency stop**
>
> `balance()`, `damp()` and `close()` are commands sent over a network link. None of them cuts
> power, and none reaches a robot the link has lost. Keep the **E-Stop** open and the battery
> unit within reach whenever the robot stands.

## Standing and Balancing

The robot has three robot modes, and only one of them keeps it upright on its own. In
**DAMP** no actuator holds a position. In **STAND** the actuators hold a standing pose and
nothing balances it: a free-standing robot in STAND tips over. In **MOVE** the walking
policy balances the robot, at zero velocity or any other. So the robot must be supported in
DAMP and STAND, and stands free only once `balance()` has returned and it reports MOVE.

This is the sequence for a script, and the order in which the Quickstart
runs it as separate files:

```python
import sys

from maker.humanoid import NotReadyError, Robot, RobotFaultedError, WaitTimeoutError

with Robot().connect() as robot:
    # 1. The robot hangs from its gantry hook with both feet on the floor.
    try:
        robot.stand()    # DAMP -> STAND; returns once the robot is armed
        robot.balance()  # armed STAND -> MOVE at zero velocity; returns once MOVE is reported
    except RobotFaultedError as exc:  # latched until the firmware restarts
        sys.exit(f"faulted: {exc}")
    except NotReadyError as exc:  # no live state: nothing was sent
        sys.exit(str(exc))
    except WaitTimeoutError as exc:  # sent, but the robot did not arm or reach MOVE in time
        sys.exit(str(exc))
    # 2. Only now: slacken the gantry until the robot carries its own weight.
    input("Robot balancing in MOVE. Slacken the gantry, then press Enter to walk. ")
    # 3. Walk. Every set_velocity() ends with zero velocity; the robot stays in MOVE.
    robot.set_velocity(vx=0.2, duration=3.0)
    robot.balance()  # zero velocity: balancing in place
    # 4. Support the robot again before it leaves MOVE.
    input("Take up the gantry's slack so it holds the robot, then press Enter to damp. ")
    robot.damp()  # every actuator stops holding its position
```

Why each step is where it is:

1. **`stand()` on the gantry hook, both feet on the floor.** STAND holds a pose and does not
   balance, so the hook carries the robot. `stand()` sends STAND and returns once the robot
   reports STAND and has been upright for 0.5 s: the firmware enters MOVE only after that, so
   `stand()` returning means the robot is armed. It raises `NotReadyError` with nothing sent
   only when there is no live state.
2. **`balance()` before the support comes off.** From an armed STAND, `balance()` sends zero
   velocity, which is what puts the robot in MOVE, and returns once the robot reports MOVE.
   It does not wait for arming: after `stand()` the robot is already armed. MOVE at zero
   velocity is the only state in which the robot stands free, so the hook keeps carrying the
   robot until `balance()` has returned: until then the robot is in STAND, and a robot in
   STAND that loses its support tips over.
3. **Slacken the gantry only after `balance()` returns.** The SDK cannot see the gantry; the
   script asks you. Slacken it gradually until the robot carries its own weight, with 2 m of
   clear floor ahead.
4. **Walk, then `balance()`.** `set_velocity()` holds the velocity for its `duration` and
   ends with zero velocity. `balance()` in MOVE is the same zero velocity, sent at once: it
   ends a walk and the robot balances in place.
5. **Support the robot before `damp()`.** `damp()` is sent at once, and a standing robot
   falls when its actuators let go. Take up the gantry's slack, or seat the robot on a stool
   or bench, first. A script cannot see the support, so this one asks before it damps; to go
   through STAND first, as the robot's own controls do
   (Bring It Back Down), see
   Bringing the Robot to Rest.

Catch all three errors around `stand()` and `balance()`. A `RobotFaultedError` means the
command was sent and a critical alert, a fall included, latched DAMP until the firmware
restarts; a `NotReadyError` means there was no live state and nothing was sent; a
`WaitTimeoutError` means the command was sent and the robot did not report the mode in
time, so read `robot.get_state().mode` before you decide what to do next. Keep Humanoid Manager
open at the **E-Stop** throughout: nothing in the SDK is an emergency stop.

## Bringing the Robot to Rest

The SDK sends every mode change you ask for, in any robot mode, and the firmware decides what
it does. Two of them need the robot supported first, because the SDK cannot see the support:

- **MOVE to STAND.** STAND has no balance loop: it stiffens the joints into a pose, and a
  free-standing robot asked to stand from MOVE tips over. `stand()` from MOVE is sent as
  asked, so hang the robot from its gantry hook or seat it on a stool or bench first.
- **Anything to DAMP.** DAMP makes every actuator compliant, so a standing robot falls. Support
  the robot the same way before `damp()`.

`rest.py` goes from MOVE to STAND to DAMP, the order the robot's own controls use. It asks
first whether the robot is on its gantry hook or seated on a stool or bench, unless you set
`YES = True`; sends STAND and waits until the robot reports it; then sends DAMP and waits
until the robot reports that, printing each robot mode. A robot already in DAMP is left as it
is:

```python
mode = robot.get_state().mode
print(f"robot mode {mode.name}")
if mode in (Mode.DAMP, Mode.FAULT_DAMP):
    print("already at rest; nothing sent")
    sys.exit(0)
if not YES:
    question = "Is the robot on its gantry hook or seated on a stool or bench? [y/N] "
    if input(question).strip().lower() not in ("y", "yes"):
        print("nothing sent")
        sys.exit(0)
try:
    # STAND first: the robot holds its pose, so DAMP does not drop it from a walk.
    robot.stand(wait=False)
    robot.wait_until(lambda s: s.mode is Mode.STAND, timeout=STAND_TIMEOUT_S)
    print(f"robot mode {robot.get_state().mode.name}")
    robot.damp()  # returns once the robot reports DAMP
except (NotReadyError, WaitTimeoutError) as exc:  # no live state, a fault, or not reported
    print(exc)
    sys.exit(1)
print(f"robot mode {robot.get_state().mode.name}")
```

From `examples/rest.py` in the maker-sdk repository (the excerpt the page shows).

`rest.py` is not an emergency stop: for that, use the E-Stop in Humanoid Manager, or cut power
at the battery unit.

## If Something Goes Wrong

1. **Press the E-Stop in Humanoid Manager, or cut power at the battery unit.** The E-Stop damps
   every actuator at once and a free-standing robot falls; use it when a robot lying on the
   floor is better than what is happening now. If the robot's software is not running, the
   E-Stop is disabled: cut power at the battery unit. Do not try to catch a falling robot.
2. **Stop the script.** Ctrl-C in a script that uses `with Robot().connect()` leaves the
   block, and `close()` sends zero velocity. A script that keeps running keeps re-sending
   its velocity at 10 Hz; `maker balance` from another terminal does not end it.
3. **Read what the robot reports, and write it down.** `maker status` shows the robot mode, a
   **FAULTED** badge and what latched, from the current alerts and the latched `error_flags`.
   In a script, `state.faulted` and the `faulted` line from `preflight()` name the causes,
   and the waits of `stand()`, `balance()`, `set_velocity()`, `set_joints()` and
   `wait_until()` raise `RobotFaultedError`.
4. **Recover the robot.** Follow
   Recovering After an E-Stop or a Fall:
   support and inspect the robot. Its **Stop Robot** and **Start Robot** steps restart the firmware, so do
   step 3 first.
5. **Restart the firmware, if the recovery did not.** A critical alert holds the robot in
   DAMP, and `error_flags` set, until the firmware restarts; restarting clears the record of
   what latched. Use **Restart** on the **Firmware (RPU)** row of the
   Troubleshoot page, or **Stop Robot** and
   **Start Robot** on Overview.

Do not call `stand()` on a free-standing robot that is walking or falling. STAND has no
balance loop: it stiffens the joints into a pose, and a free-standing robot in that pose tips
over. End a walk with `balance()`, which leaves the robot in MOVE, balancing in place.

## Who Stops What

| Layer | Stops | Does not stop |
| - | - | - |
| **SDK** | a held velocity: zero on `balance()`, `close()`, the end of a `with` block, a lost link | anything after a fault; another program's velocity |
| **The robot** | velocity and trajectory in DAMP (dropped); a velocity 2 s after the last command on `udp` and `hybrid`; a trajectory 2 s after the last setpoint (DAMP) | a velocity a script keeps re-sending |
| **Firmware** | MOVE before an armed STAND (the robot stays in STAND, nothing reports it); everything, on a critical alert (DAMP, latched) | a command because of where it came from: it obeys whichever arrived last; STAND from MOVE on a free-standing robot |

The SDK refuses a command only when there is no live state. It also clamps every velocity
to your `Limits`, runs every wait against fresh state,
and fences a running `set_joints()` so that any verb ends it. `close()` sends zero velocity
only when this `Robot` holds one; a `Robot` that sent nothing does not stop another program.
After `close()` stops re-sending a trajectory, the robot enters DAMP 2 s later. Neither the
robot nor the firmware checks command timestamps.

## Who Holds Control

The robot gives the Drive Robot page priority over a paired gamepad, the gamepad over `udp`, and
`udp` over `livekit`. The state stream does not say who holds control, so a walk that does
not happen, with no fault and no refusal, is the first thing to check.

> [!WARNING]
>
> **An idle paired gamepad locks the SDK out**
>
> A gamepad paired to the robot holds control while nobody touches it. Every velocity a script
> sends is dropped without a report. Unpair the gamepad on the **Controller** page of the humanoid robot
> Manager before you run a script; see Gamepad.

## Limits

The firmware caps `vx` and `vy` at 0.4 m/s and `vyaw` at 0.8 rad/s. `Limits()` defaults to
those caps and the SDK clamps every velocity to them before it leaves, so `Sent.clamped` is
`True` exactly when the robot would not have walked at the speed you asked for. A tighter
envelope is `Robot(limits=Limits(vx=0.2, vy=0.2, vyaw=0.4))`, `MAKER_LIMITS="0.2,0.2,0.4"`,
or `limits` on a saved robot. Limits above the caps are sent as asked and the firmware clamps
them.

## Arming

MOVE is entered only from STAND held upright, gravity z below -0.87 (under 30 degrees of
tilt), for 0.5 s. A velocity sent earlier is not refused and not reported: the robot stays in
STAND. `stand()` returns once the robot is armed, so `stand()` then `balance()` enters MOVE,
and `robot.armed` reads the same test the firmware applies. `balance()` and `set_velocity()`
send without waiting for arming: a velocity that reaches an armed STAND puts the robot in
MOVE, and `balance()` that reaches a STAND that is not armed times out saying so. The SDK counts
the 0.5 s from its own samples, so a new session sees an armed robot as armed 0.5 s after it
connects. A robot that reports no gravity leaves `robot.armed` as `None`.

## Faults

A critical alert, such as a fall, actuator over-temperature, battery protection, a joint past
its limit, lost actuator communication or a watchdog, makes the firmware latch DAMP. The
latch, and `error_flags`, stay until the firmware restarts. The SDK reports it as
`State.faulted` and as `faulted` from `preflight()`, and drops a held velocity without
sending anything more. A command is still sent; the wait that sees the fault, in `stand()`,
`balance()`, `set_velocity()`, `set_joints()` or `wait_until()`, raises `RobotFaultedError`
with `.sent` set. `damp()` returns, since the robot is already in DAMP.

## Watchdogs

| Watchdog | Where | What happens |
| - | - | - |
| **Velocity** | the robot, `udp` and `hybrid` | zero velocity 2 s after the last velocity command; the robot stays in MOVE, balancing |
| **Trajectory** | the robot, every mode | DAMP 2 s after the last setpoint |
| **Link** | SDK, every mode | no state for 2 s: zero velocity if one is held, `on_link_lost` fires, every verb raises `LinkLostError` until `close()` and `connect()` |

A script that holds a velocity re-sends it at 10 Hz, so the velocity watchdog does not stop
it: end it with `balance()`, a `duration`, or `close()`.

On a `livekit` connection, Humanoid Edge stops a held velocity when the SDK sends zero or
leaves the room. A script that hangs while it holds a velocity keeps the robot walking until
LiveKit drops the participant. Bound every hold with a `duration`, and prefer `udp` or
`hybrid` for a control loop.

## What the SDK Reports

The SDK does not refuse a command because of what the robot reports. Its robot mode, a
latched fault, an alert, an actuator temperature or the battery never stop `stand()`,
`balance()`, `set_velocity()`, `trajectory()` or `set_joints()`, and the firmware decides
what a command does. The firmware warns at 60 °C and latches DAMP at 80 °C on an actuator,
and warns below 20 % battery.

The one refusal is no live state. `stand()`, `balance()` outside MOVE, `set_velocity()`,
`trajectory()` and `set_joints()` read the latest state sample before they send; on a live
stream that adds no delay. Without one (`no_state`, `stale_state`) they wait up to the
command's `timeout`, then raise `NotReadyError` with nothing sent. After a STAND sent moments
ago whose report has not arrived, they wait up to 1 s for that report, then send anyway. A
closed `Robot` raises `NotConnectedError`, a lost link `LinkLostError` and a protocol version
mismatch `ProtocolMismatchError`, at once. `trajectory()` and
`set_velocity(hold=False)` check once and do not wait unless given a `timeout`. `balance()`
in MOVE and `damp()` are sent at once.

```text
not ready to move: the latest state is 1.2 s old (limit 0.5 s) (stale_state); check the network link to the robot (waited 5.0 s)
```

Waits stay truthful. `stand()` returns at STAND and armed, `balance()` at MOVE, `damp()` at
DAMP. A latched fault seen during a wait raises `RobotFaultedError`, after the command was
sent, and a robot that does not get there raises `WaitTimeoutError` saying what it reports:

```python
from maker.humanoid import NotReadyError, Robot, RobotFaultedError, WaitTimeoutError

with Robot().connect() as robot:
    try:
        robot.stand()
        robot.balance()
        robot.set_velocity(vx=0.3, duration=3.0)
    except RobotFaultedError as exc:  # sent; latched until the firmware restarts
        print("faulted:", exc)
    except NotReadyError as exc:  # no live state: nothing was sent
        print(exc)
    except WaitTimeoutError as exc:  # sent, but the robot did not arm or reach MOVE in time
        print(exc)
```

`RobotFaultedError` is a `NotReadyError`, so catch it first to tell a latched fault apart.

`robot.preflight(action)` lists the facts for `"stand"`, `"move"` (`balance()` and
`set_velocity()`) or `"trajectory"` (`trajectory()` and `set_joints()`). It sends nothing,
never waits and never raises for a robot condition. `check.ok` is `True` when nothing is
blocking, which means there is live state; `check.problems` lists the problems, blocking
first. Only the blocking codes stop a command; the others are information, and nothing in the
SDK acts on them. `check.py` prints the raw facts and `preflight()` for all three actions:

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

| Code | Blocks | Meaning | What clears it |
| - | - | - | - |
| `not_connected` | yes | closed, link lost, or protocol mismatch | `close()` and `connect()` |
| `no_state` | yes | the session is open and the robot has not reported state | wait for the firmware; see No State on udp or hybrid |
| `stale_state` | yes | latest state older than 0.5 s | a command waits for the next sample; otherwise check the link |
| `faulted` | no | a critical alert is latched, or the robot reports FAULT_DAMP; the alerts that caused it are named | restart the firmware |
| `alerts` | no | the firmware reports active alerts, named | |
| `not_armed` | no | `move` from STAND before the 0.5 s upright hold | hold STAND upright |

## Write Your Own Guard

A rule about what your script will not drive through is yours to write. Read
`robot.get_state()` once, decide, and only then send:

```python
s = robot.get_state()  # one sample; every fact from it
if s.faulted or any(j.temp is not None and j.temp >= 80 for j in s.joints):
    raise SystemExit(f"not driving: robot mode {s.mode.name}, faulted {s.faulted}")
```

`guard.py` is an example guard to start from. Its limits are constants at the top of the
file: `MAX_JOINT_TEMP_C` (80 °C, where the firmware latches DAMP), `MIN_BATTERY_PERCENT`
(20 %), `REFUSE_WHEN_FAULTED` and `REFUSE_ON_ALERTS` (warning and critical alerts, including
the firmware's warning at 60 °C). It prints what it finds and exits with status 1 when your
rule fails:

```python
def guard(robot: Robot) -> None:
    """Print what the robot reports, and exit with status 1 when your rule fails."""
    s = robot.get_state()  # one sample: every fact below comes from it
    stop: list[str] = []

    print(f"guard: robot mode {s.mode.name}, armed {robot.armed}, faulted {s.faulted}")
    if s.faulted and REFUSE_WHEN_FAULTED:
        stop.append("the firmware latched DAMP; it stays so until the firmware restarts")

    temps = [(j.temp, j.name or f"joint {i}") for i, j in enumerate(s.joints) if j.temp is not None]
    if temps:
        hottest, joint = max(temps)
        print(f"guard: hottest joint {joint} {hottest:.0f} C")
        if hottest >= MAX_JOINT_TEMP_C:
            stop.append(f"{joint} is at {hottest:.0f} C (your limit {MAX_JOINT_TEMP_C:.0f} C)")
    else:
        print("guard: joint temperatures not reported")

    if s.battery is not None:
        print(f"guard: battery {s.battery.soc_percent:.0f} %")
        if s.battery.soc_percent < MIN_BATTERY_PERCENT:
            stop.append(
                f"battery at {s.battery.soc_percent:.0f} % (your limit {MIN_BATTERY_PERCENT:.0f} %)"
            )
        if s.battery.protecting:
            stop.append("the battery management system is protecting the pack")
    else:
        print("guard: battery not reported")

    for alert in s.alerts:
        print(f"guard: alert {alert.name} (severity {alert.severity})")
    serious = sorted({a.name for a in s.alerts if a.severity <= 1})  # 0 critical, 1 warning
    if serious and REFUSE_ON_ALERTS:
        stop.append(f"active alerts: {', '.join(serious)}")

    if stop:
        print("guard: not going ahead: " + "; ".join(stop))
        print("guard: this is your rule in guard.py; change it, or delete the guard(robot) line")
        sys.exit(1)


```

From `examples/guard.py` in the maker-sdk repository (the excerpt the page shows).

No example uses it by default. The motion examples, `stand.py`, `balance.py`, `walk.py`,
`keyboard.py`, `stream_velocity.py`, `move_joints.py` and `wait_until.py`, carry it as two
commented lines; uncomment both to run it before the script sends anything:

```python
from guard import guard

guard(robot)
```

Change the constants or change `guard()` to make the rule yours. Run on its own,
`python guard.py` prints what it finds and sends nothing.

## Shutdown

Leaving the `with` block calls `close()`: zero velocity if one is held, then the link drops.
The robot stays in MOVE, balancing in place, and the Drive Robot page or the gamepad can take it from
there. Call `damp()` at the end of a session only on a supported robot, hanging from its
gantry hook or seated on a stool or bench: every actuator stops holding its position and a standing
robot falls.

`damp.py` asks before it sends, then waits for the robot to report DAMP:

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

Turning the robot off is in Power.

## Related

- Stopping the Robot: the E-Stop, pause, damp and cutting power
- Stand Up and Walk: the same sequence from the Drive Robot page
- Move the Robot: stand, balance and walk
- Control Joints: joint control on a supported robot
- Troubleshooting: each error and its fix
