# Quickstart

> Install maker-sdk, save the robot, check it, then stand, balance, walk for 3 s and damp from Python.

- Section: Python SDK

By the end of this page the robot stands up from DAMP, balances on its own, walks forward
for 3 s and rests again, from five short scripts.

> [!WARNING]
>
> **Before the first walk**
>
> For a first run, hang the robot from its gantry hook with both feet on the floor and 2 m of
> clear floor ahead. Keep Humanoid Manager open at the **E-Stop**, or be ready to cut power at
> the battery unit: nothing in the SDK is an emergency stop. Work through
> Before You Run a Script and
> Standing and Balancing first.

1. ## Install

   Python 3.12 or newer.

   ```bash
   pip install maker-sdk
   ```

   or, in a project managed by uv, `uv add maker-sdk`. One package drives every connection
   mode; there are no extras.

2. ## Get the Examples

   The scripts on this page are in the `examples` directory of the
   maker-sdk repository on GitHub.
   Clone it and run them from there:

   ```bash
   git clone 
   cd maker-sdk/examples
   ```

   Each code block below shows the main part of a script. The full script also imports the
   SDK, sets its constants and connects with `with Robot().connect() as robot:`, which waits
   for the robot's first state.

3. ## Get an SDK Credential

   The `hybrid` and `livekit` connection modes join the robot's LiveKit room with an SDK
   credential from Humanoid Manager; `livekit` also carries commands through it. Skip this step
   for `udp`, which needs no credential.

   1. In Humanoid Manager, open **Developer** in the sidebar. Under **How a script connects**, note the **Manager URL**: it is the address the SDK uses to reach the robot.

      

      You do not need the **LiveKit URL** or the **Room**. The SDK asks the robot for both each time it connects, and when the LiveKit URL reads `localhost`, the SDK uses the robot's address instead.

   2. Under **Issue a credential**, enter a **Name** for the computer that will use it, such as `my-laptop`. Issue one per computer, so you can revoke one without cutting off the others.

   3. Under **What it may do**, choose **Control** to drive the robot from your computer. **Observe** watches the camera and listens to the microphone only; the robot drops its commands.

      

   4. Select **Issue Credential**, then copy the credential with the copy button next to it. It is shown only this once: the robot keeps the credential's ID, not the credential, so it cannot show it again. If you lose it, revoke it and issue another.

      

   5. The credential appears under **Issued credentials** with its role and ID. To take it back, select **Revoke** next to it; the computer holding it can no longer connect, and a session that is already running keeps working until its LiveKit token expires, at most 12 hours.

      

4. ## Save the Robot

   ```bash
   maker setup
   ```

   The wizard asks for a name, a connection mode, and what the mode needs: the robot's
   address for `udp`, the Manager URL and the credential for `livekit`, both for `hybrid`. It
   checks each field against the robot before it saves, and every script below then finds the
   robot with `Robot()` and no arguments.

   `udp` and `hybrid` carry commands and state on the robot's network, and the robot sends
   UDP state to one computer. In Humanoid Manager, open **Advanced Settings** and, under
   **Humanoid Edge**, set `udp-control` to on and `udp-state-host` to your computer's address,
   then select **Save & Restart Humanoid Edge**. Without a saved robot, `export
   MAKER_UDP_HOST=<robot address>` points the SDK at the robot for one shell. To build the
   connection in code, see Connection Modes.

5. ## Check

   `check.py` connects, prints what the robot reports (robot mode, armed, faults, alerts, the
   hottest joint, battery and the state's age) and `preflight()` for a stand, a walk and a
   trajectory, and sends nothing. Run it before anything that moves the robot:

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

   The SDK does not refuse a command because of what the robot reports: a fault, an alert, a
   hot actuator or a low battery is yours to judge, and the firmware decides what a command
   does. The one refusal is no live state. The scripts below carry `guard(robot)` from
   `guard.py` commented out: an example guard of your own that stops the script when the robot
   is faulted, has a warning or critical alert, has an actuator at 80 °C or more, or has a
   battery below 20 %. Uncomment its two lines in a script to turn it on, and edit its limits;
   see Write Your Own Guard.

6. ## Stand

   With the robot in DAMP, hanging from its gantry hook with both feet on the floor:

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

   ```bash
   python stand.py
   ```

   The actuators hold a standing pose and `stand()` returns once the robot is armed: STAND
   held upright for 0.5 s. It sends nothing else. Without live state, `stand()` raises
   `NotReadyError` with nothing sent, and the script prints why; when the robot does not arm
   within 10 s, it raises `WaitTimeoutError`. `stand()` is sent from any robot mode, but STAND
   has no balance loop: after a walk, hang the robot from its gantry hook or seat it on a stool
   or bench before you call it.

7. ## Balance

   With the robot armed and still on its gantry hook:

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

   ```bash
   python balance.py
   ```

   `balance()` sends zero velocity, which puts an armed robot in MOVE: the walking policy
   balances it in place, and this is the only robot mode in which it stands free. It returns
   once the robot reports MOVE, or raises `WaitTimeoutError` after 5 s. Once it has returned,
   slacken the gantry gradually until the robot carries its own weight.

8. ## Walk

   With the robot balancing in MOVE:

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

   ```bash
   python walk.py
   ```

   What each line does:

   | Line | Effect |
   | - | - |
   | `# guard(robot)` | Your own rule from `guard.py`, off by default: uncomment it and its import to exit when the robot reports what it will not drive through |
   | `if mode is not Mode.MOVE` | This script's own rule: it walks in MOVE only |
   | `set_velocity(vx=VX, vy=VY, vyaw=VYAW, duration=DURATION_S)` | Walks at the set velocity for the duration; the SDK re-sends the velocity while it is held, sends zero velocity when the duration ends, and returns after that |
   | `except NotReadyError` | There was no live state and nothing was sent, or a fault ended the walk (a `RobotFaultedError` with `.sent` set); the message says what the robot reports and what to do |
   | `sent.clamped` | `True` when the SDK reduced the speed to the robot's limits; `sent.command` is what went out |

   After the walk the robot stays in MOVE, balancing in place. The velocity and duration are
   constants at the top of the file. Leaving the `with` block closes the connection; a held
   velocity gets a zero first. The script never changes the robot mode: on a robot in STAND it
   prints `robot mode STAND: walk.py walks in MOVE only; run balance.py first` and exits. That
   rule is the script's, not the SDK's: `set_velocity()` is sent in any robot mode.

9. ## Put the Robot Down

   `damp()` makes every actuator compliant. A standing robot folds, so support it first: take
   up the gantry's slack, or seat the robot on a stool or bench. `rest.py` goes from MOVE to
   STAND to DAMP instead, and asks first; see
   Bringing the Robot to Rest.

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

   ```bash
   python damp.py
   ```

   The script asks for confirmation unless `YES` is set to `True` at the top of the file.
   `damp()` is not an emergency stop; for that, use the E-Stop in Humanoid Manager, or cut power
   at the battery unit (Stopping the Robot).

## What Ready Means

Ready means live state. A command refuses only without it: it waits up to its `timeout`
for a fresh sample (by default 10 s for `stand()`, 5 s for `balance()`, `set_velocity()` and
`set_joints()`; `trajectory()` checks once), then raises `NotReadyError` with nothing sent. Every
other fact is reported, not acted on:

```python
from maker.humanoid import NotReadyError, Robot

with Robot().connect() as robot:
    print(robot.preflight("move"))  # the facts: blocking only without live state
    try:
        robot.set_velocity(vx=0.3, duration=3.0)
    except NotReadyError as e:
        print(e)  # not ready to move: the latest state is 1.2 s old (limit 0.5 s) (stale_state); ...
```

`balance()` in MOVE and `damp()` are sent at once. The full picture is in
What the SDK Reports.

## Next Steps

- Move the Robot: turning, holding a velocity, limits
- Safety: what the SDK does and does not do for you
- Examples: every script in the repository, including
  `keyboard.py` for driving from the keyboard
- Command Line: `maker setup` to save a robot, `maker status --watch`
  while you develop

## Related

- Drive from Python: the same path with the robot's settings, video and sound
- Stand Up and Walk: the same sequence from the Drive Robot page
- maker-sdk on PyPI
- maker-sdk on GitHub
