# Read State

> One snapshot of everything the robot reports, the facts `preflight()` reports, and how to wait on it.

- Section: Python SDK

`robot.get_state()` returns the latest sample the robot reported, decoded: robot mode, joints, IMU,
battery, alerts. Reading it costs nothing, because the robot streams it anyway. A field the
robot does not report is `None`, never a guessed zero.

`read_state.py` is a tour of the state: robot mode, arming, what latched, battery, the
hottest actuators, gravity, state age and rate, alerts. It sends nothing and works in every
connection mode.

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

| Field | Meaning |
| - | - |
| `state.mode` | `Mode.DAMP`, `Mode.STAND`, `Mode.MOVE`, `Mode.FAULT_DAMP` or `Mode.UNKNOWN` |
| `state.joints` | one `Joint(name, pos, vel, current, temp)` per actuator, firmware order |
| `state.joint("L_Knee")` | a joint by name; `KeyError` on an unknown name |
| `state.joint_pos` | one radian value per actuator |
| `state.gravity` | gravity projected into the body frame; z near -1 when upright |
| `state.gyro`, `state.quat`, `state.euler`, `state.yaw` | angular rate, orientation, and heading in radians counter-clockwise |
| `state.battery` | `Battery(voltage_v, current_a, soc_percent, max_cell_temp_c, protection)` or `None` |
| `state.alerts`, `state.error_flags`, `state.faulted` | alerts and faults reported by the firmware |
| `state.age_s` | seconds since this sample arrived |

## Check the Facts

`robot.preflight(action)` reads the latest state and lists the facts for `"stand"`,
`"move"` or `"trajectory"`: whether there is live state, a latched fault, active alerts, and
in STAND whether the robot is armed. It sends nothing, never waits and never raises for a
robot condition. `check.py` prints the raw facts and all three:

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

`check.ok` is `True` when nothing is blocking, which means there is live state;
`check.problems` lists the problems, blocking first, then information; `check.has(code)`
tests one; `str(check)` is readable. Only `not_connected`, `no_state` and `stale_state`
block; `faulted`, `alerts` and `not_armed` are information. Each code is in
What the SDK Reports.

`stand()`, `balance()` outside MOVE, `set_velocity()`, `set_joints()` and `trajectory()` wait
for live state before they send, up to `timeout` seconds, then raise `NotReadyError`,
carrying `.preflight` and `.problems`, with nothing sent. Nothing else stops them: the robot
mode, a fault, an alert, a hot actuator or a low battery is for your script to decide on,
from `robot.get_state()`, as `guard.py` does; see
Write Your Own Guard.

`robot.armed` is the arming test on its own: `True` in MOVE, or in STAND once the robot has
been upright for 0.5 s; `False` in DAMP or before that; `None` without state, closed, in
UNKNOWN, or in STAND with no gravity reported.

## Wait on the Robot

Waits read the robot's own report, never a sleep. The code below moves the robot, so it
belongs after the checklist and
Standing and Balancing:

```python
if robot.get_state().mode is Mode.DAMP:
    robot.stand()                               # returns once the robot reports STAND and is armed
robot.balance()                                 # returns once the robot reports MOVE
robot.set_velocity(vx=0.2, duration=2.0)        # checks the robot, then sends

robot.wait_until(lambda s: s.mode is Mode.MOVE and s.upright, timeout=10.0)
```

`stand()`, `balance()`, `damp()`, `set_joints(wait=True)` and `wait_until` read the robot's report and raise
`WaitTimeoutError` (also a `TimeoutError`) when the condition is not met in time, with
`.last` holding the last state seen; `.sent` on a command carries what went out. A stream
that goes quiet raises `StateStaleError` instead: a robot that did not reach the state and a
stream that stopped need different fixes. A fault is checked before the predicate and raises
`RobotFaultedError`, so a fall is never read as success. `stale_after` on `wait_until`
defaults to the link timeout of 2 s.

## Upright

`state.upright` is `True` when gravity z is below -0.8, about 37 degrees of tilt, and `None`
when the robot reports no gravity: unknown and fallen are different answers. It is not the
arming test; the firmware arms at -0.87, under 30 degrees, and `robot.armed` reads that.

## Faults

`state.faulted` is `True` when `error_flags` is set or any alert is critical (severity 0).
A critical alert, such as a fall, actuator over-temperature, battery protection, a joint past
its limit, lost actuator communication or a watchdog, makes the firmware latch DAMP. It stays
latched, and `error_flags` stay set, until the firmware restarts. The robot may report it as
`Mode.FAULT_DAMP`. Commands are still sent; the waits of `stand()`, `balance()`,
`set_velocity()`, `set_joints()` and `wait_until` raise `RobotFaultedError`; `preflight()`
reports `faulted`; nothing a script sends clears it.

`alert.name` is the firmware's name for an alert, `alert.critical` its severity test.

## What the SDK Knows About the Body

```python
info = robot.info
info.dof, info.joint_names          # 25 and the firmware's actuator order on the humanoid robot
info.capabilities                   # frozenset({'drive', 'state', 'battery', 'camera', ...})
info.protocol_version, info.limits, info.endpoint, info.transport
```

Joint names come from `maker.humanoid.robots.HUMANOID_BIPED_JOINTS`, chosen by the reported
joint count. `robot.has("camera")` and `robot.require("drive", "battery")` test the
capabilities; the set is what this connection carries, so it holds no media on udp.

## Callbacks

For event-driven code, register a callback instead of polling. Callbacks run on the
transport thread, the one that delivers state: keep them short. `robot.damp()` from a
callback, such as `on_alert` damping on an alert, sends DAMP and returns at once. The other
commands that wait for the robot, and `wait_until()`, raise `RuntimeError` there with nothing
sent; pass `wait=False`, or call them from your own thread.

| Callback | Arguments |
| - | - |
| `robot.on_state` | every accepted `State` |
| `robot.on_mode_change` | `(before, after)`, both `Mode` |
| `robot.on_alert` | `(alert, change)`, with `change` `"raised"` or `"cleared"` |
| `robot.on_link_lost` | the `LinkLostError` |

## Related

- Move the Robot: what to do once the robot is ready
- Safety: every preflight code
- SDK Reference: every field and type
