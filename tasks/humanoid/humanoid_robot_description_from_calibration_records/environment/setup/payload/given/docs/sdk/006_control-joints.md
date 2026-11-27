# Control Joints

> set_joints and trajectory: per-joint targets on a supported robot, with the walking policy off.

- Section: Python SDK

Two verbs put the actuators under position control. `set_joints(positions)` moves every
joint to a pose for you; `trajectory(positions)` sends one setpoint and expects you to clock
the next.

> [!WARNING]
>
> **Joint control turns the walking policy off**
>
> While a `set_joints()` or a `trajectory()` is in force, nothing balances the robot. Use them
> only with the robot supported, hanging from its gantry hook or seated on a stool or bench. A fall
> latches DAMP until the firmware restarts. Work through
> Before You Run a Script first.

## Move to a Pose

`set_joints` interpolates from the pose the robot last reported to the one you ask for, over
`duration` seconds, and clocks the setpoints at `hz` from a thread. It returns once every
joint is within `tolerance` radians of its target, then holds the target by re-sending it
until another verb takes over.

`move_joints.py` bends one elbow (`JOINT = "L_Elbow"`) and moves nothing else. Run
`stand.py` first, keep the robot supported, and read what happens when the script exits
before you run it. The script exits unless you set `ROBOT_SUPPORTED = True` at the top of
the file. To stop on your own rule from `guard.py` before it moves anything, uncomment its
two guard lines:

```python
# guard(robot)  # optional: uncomment this and the import to stop on your rule
state = robot.get_state()  # one sample: the pose to move from and back to
start = state.joint_pos
home = state.joint(JOINT).pos
target = list(start)
target[robot.info.joint_index(JOINT)] += TURN_RAD
try:
    # Moves from the current pose over DURATION_S and returns within 0.05 rad of the
    # target. set_joints() holds it until the next verb.
    robot.set_joints(target, duration=DURATION_S)
except (NotReadyError, WaitTimeoutError) as exc:  # no live state, or the joints did not follow
    print(exc)
    sys.exit(1)
print(f"{JOINT} moved from {home:.2f} to {robot.get_state().joint(JOINT).pos:.2f} rad")
robot.set_joints(start, duration=DURATION_S)
print(f"{JOINT} back at {robot.get_state().joint(JOINT).pos:.2f} rad")
```

From `examples/move_joints.py` in the maker-sdk repository (the excerpt the page shows).

`positions` is one radian value per actuator, `robot.info.dof` of them, in firmware order.
`robot.info.joint_index(name)` gives the index of a named joint and `robot.info.joint_names`
lists them all. `set_joints()` is sent in any robot mode; the firmware follows a trajectory
in MOVE or in an armed STAND, and drops it in DAMP. It waits for live state, which also gives
it a fresh pose to plan from; without live state it raises `NotReadyError` with nothing sent,
and the script prints the message.

> [!WARNING]
>
> **Support the robot in MOVE too**
>
> In MOVE the robot is usually standing free on its own balance. A `set_joints()` or a
> `trajectory()` there turns the walking policy off, and a free-standing robot falls. The SDK
> does not stop it. Before joint control in any robot mode, hang the robot from its gantry hook
> or seat it on a stool or bench.

`wait=False` returns once the plan is running. `timeout` bounds the whole call, the
live-state check and the wait; left `None`, the check may wait 5 s and the target `duration + 2` s, and a
joint that is not within `tolerance` by then raises `WaitTimeoutError`.

The ankles are limited: 0.35 rad of pitch and 0.1 rad of roll. A target more than 0.02 rad
past an ankle limit raises `ValueError` before anything is sent.

## Finish

When a trajectory is left alone, the robot enters DAMP 2 s after the last setpoint. That is
what happens when `move_joints.py` exits: the held target stops being re-sent and every
actuator stops holding its position, which is why the robot must be supported. To end joint
control on your own terms, send `damp()` yourself, with the robot still supported; see
Shutdown.

## Stream Your Own Setpoints

`trajectory` is the raw form. Each call is one setpoint; you keep the clock, bound the
loop, and end in DAMP:

```python
import time

deadline = time.monotonic() + 10.0
try:
    for target in controller:  # one pose per tick, robot.info.dof values each
        if time.monotonic() > deadline:
            break
        robot.trajectory(target)  # NotReadyError, nothing sent, on a stale stream
        time.sleep(0.02)  # 50 Hz
finally:
    robot.damp()  # the robot is supported
```

Every `trajectory()` checks for live state once; on a live stream it reads one cached sample
and adds no delay to the loop. The robot enters DAMP 2 s after the last setpoint, so a stalled loop
leaves the robot in DAMP, not frozen in its last pose. `ValueError` when
`len(positions) != robot.info.dof`, and for a target past an ankle limit.

## Gains

Without `kp` and `kd`, the robot applies its own per-joint gain table. Pass both to
override them, one value per joint; a lone `kp` or `kd` is a `ValueError`. A gain of 0 is
not "no gain": the firmware substitutes its damping constants, so the joint does not hold a
position.

## Which Command Is in Effect

The firmware obeys whichever command arrived last.

A `set_joints()` yields to any other verb: `damp()`, `stand()`, `balance()`, a velocity or
a `trajectory()` from any thread ends it before its next setpoint leaves.

A loop you clock with `trajectory()` does not. While it streams, a verb sent from another
thread can be overwritten by your next setpoint, and nothing raises. Stop your loop, then
send the verb.

## Wait for a Joint

`wait_until.py` starts a `set_joints(wait=False)` and acts once the elbow has bent part of
the way, with `robot.wait_until` reading the robot's own report. Like `move_joints.py`, it
needs the robot supported and `ROBOT_SUPPORTED = True`, and carries the same two commented
guard lines:

```python
# guard(robot)  # optional: uncomment this and the import to stop on your rule
state = robot.get_state()  # one sample: the pose to move from and back to
start, home = state.joint_pos, state.joint(JOINT).pos
target = list(start)
target[robot.info.joint_index(JOINT)] += TURN_RAD
try:
    robot.set_joints(target, duration=DURATION_S, wait=False)  # returns at once, moving
    began = time.monotonic()
    passed = robot.wait_until(lambda s: s.joint(JOINT).pos >= home + PASS_RAD, timeout=5.0)
except (NotReadyError, WaitTimeoutError) as exc:  # no live state, a fault, or never got there
    print(exc)
    last = getattr(exc, "last", None)  # the last state a wait saw, when there was one
    if last is not None and last.mode is Mode.DAMP:
        print("robot mode DAMP: the joints do not follow in DAMP, run stand.py first")
    sys.exit(1)
took = time.monotonic() - began
print(f"{JOINT} at {passed.joint(JOINT).pos:.2f} rad after {took:.1f} s of {DURATION_S} s")
# A new set_joints() ends the first move where it is: back, and wait until there.
robot.set_joints(start, duration=DURATION_S)
print(f"{JOINT} back at {robot.get_state().joint(JOINT).pos:.2f} rad")
```

From `examples/wait_until.py` in the maker-sdk repository (the excerpt the page shows).

## Related

- Safety: faults, watchdogs, and what a fall latches
- Read State: `state.joints`, `state.joint(name)`, `info.joint_names`
- SDK Reference: `set_joints` and `trajectory` signatures
