# Troubleshooting

> What each SDK error means and what to do about it.

- Section: Python SDK

Start with `maker status`: it connects, reads state for a second, and prints what the robot
reports without sending anything. The SDK refuses a command only when there is no live
state; everything else the robot reports is for your script to judge.

## `ConnectError`: Nothing to Connect To

`Robot()` found neither `MAKER_UDP_HOST` nor `MAKER_MANAGER_URL` with `MAKER_CREDENTIAL`
in the environment, and no saved robot. Run `maker setup`, or pass a `ConnectionConfig`.
With several robots saved and no default, `maker robots use NAME` picks one, or set
`MAKER_ROBOT`. `MAKER_MANAGER_URL` without `MAKER_CREDENTIAL`, or the reverse, is the same
error: they go together.

## `ConnectError`: The Saved Robot Lacks What the Mode Needs

The message names the fix, such as `maker robots add lab --udp <robot address>`. `hybrid`
needs the robot's address, the Humanoid Manager URL and an SDK credential; `udp` the address;
`livekit` the Manager URL and the credential. `cfg.available_modes()` says which modes a config can reach.

## `ConnectError`: Humanoid Manager Refused the Credential

The SDK credential is wrong or revoked, or the URL is not Humanoid Manager. Issue a new one on
the Developer page of Humanoid Manager with the Control role
(Get an SDK Credential) and run
`maker setup` again. A credential with the Observe role
connects but cannot drive on livekit. A manager that redirects is reported rather than followed: use
the address it redirected to.

## `ConnectError`: No State From the Robot on UDP

The robot never reported in. The robot sends UDP state to one address, so check, in order:

1. `udp-control` is on in the Humanoid Edge parameters in Humanoid Manager.
2. `udp-state-host` is this machine's address as the robot sees it, and `udp-state-port`
   is the port the SDK binds (8851 by default).
3. Nothing blocks port 8851 inbound: a firewall or a VPN. See
   No State on udp or hybrid.

`maker robots add NAME --udp HOST` runs this check and prints the same advice.

## No State on udp or hybrid

On `udp` and `hybrid`, the robot listens for commands on UDP port 8850 and sends its state to
one address: the `udp-state-host` and `udp-state-port` (8851 by default) in the Humanoid Edge
parameters. Commands go out from your computer, but state comes in to it, so a firewall on
your computer that blocks inbound UDP 8851 from the robot leaves the SDK with no state:
`connect()` raises `ConnectError`, a command raises `NotReadyError` with `no_state` or
`stale_state`, and `maker stand`, `balance` and `walk` print `Not feasible:`. `livekit` is
not affected.

First check that `udp-state-host` is this computer's address as the robot sees it; the robot
sends state to that one host only. Then allow inbound UDP 8851 from the robot:

- **Linux** with ufw: `sudo ufw allow from <robot address> to any port 8851 proto udp`. With
  firewalld: `sudo firewall-cmd --permanent --add-rich-rule='rule family="ipv4" source address="<robot address>" port port="8851" protocol="udp" accept'`, then
  `sudo firewall-cmd --reload`.
- **macOS**: the firewall allows or blocks per application. Open **System Settings >
  Network > Firewall > Options** and allow incoming connections for the Python you run the
  SDK with, or answer **Allow** when macOS asks the first time a script connects.
- **Windows**: answer **Allow** for private networks when Windows asks the first time a
  script connects, or in an administrator PowerShell run
  `New-NetFirewallRule -DisplayName "the humanoid robot state" -Direction Inbound -Protocol UDP -LocalPort 8851 -Action Allow`.

Run `maker status` again: the state rate and a fresh sample show that state arrives.

## `ConnectError`: Could Not Bind the State Port

Another process holds port 8851; one client per port on udp. Close it, or use
`state_bind=("0.0.0.0", <other port>)` and set `udp-state-port` to match.

## `ProtocolMismatchError`

The robot speaks another protocol version than the SDK was built against. Update whichever is
behind. `connect(allow_version_skew=True)` proceeds anyway, and the robot may misread the commands the
SDK sends.

## `NotReadyError`

There was no live state to send against, and nothing was sent. `stand()`, `balance()`
outside MOVE, `set_velocity()`, `set_joints()` and `trajectory()` read the latest state
sample before they send. The codes are `no_state` (the robot has not reported) and
`stale_state` (the latest sample is older than 0.5 s); both are waited out for up to the
command's `timeout` first, and the message then ends in `(waited <timeout> s)`. A closed
`Robot` raises `NotConnectedError`, a lost link `LinkLostError` and a protocol version
mismatch `ProtocolMismatchError` instead, at once.
`e.problems` carries them, `e.has(code)` tests one and `e.preflight` is the whole report.

Check the link with `maker status`. On `udp` or `hybrid`, see
No State on udp or hybrid.

The robot mode, a fault, an alert, a hot actuator or a low battery never raise
`NotReadyError`: the command is sent and the firmware decides. A rule against them is yours
to write; see Write Your Own Guard.

## `RobotFaultedError`

A wait found `state.faulted` set after the command was sent: a critical alert, such as a
fall, over-temperature or battery protection, has latched DAMP. The message and `state.alerts`
name it; write it down before anything restarts the firmware, since restarting clears the
record of what latched. Nothing a script sends clears it. Then follow
Recovering After an E-Stop or a Fall
to support and inspect the robot; its **Stop Robot** and **Start Robot** steps restart the firmware. See
If Something Goes Wrong.
`RobotFaultedError` is a `NotReadyError`, so `except NotReadyError` catches it too; catch it
first to tell a fault apart. `e.sent` is the command that went out.

## `WaitTimeoutError`

A command was sent and the robot did not report the effect in time. `e.last` is the last
state seen and `e.sent` what went out.

- `stand()`: the robot did not report STAND and arming within `timeout` (10 s). See the next
  section.
- `balance()` from STAND: the robot did not report MOVE within `timeout` (5 s) after the
  zero velocity went out. The message says when the robot was not armed: wait until
  `robot.armed` is `True`, then `balance()` again. Otherwise another controller may hold
  it: the Humanoid Manager Drive Robot page or a paired gamepad outranks a script, and a command it
  outranks has no effect.
- `balance()` from DAMP: raised at once, `the robot is in DAMP: stand() first`.
- `damp()`: the robot did not report DAMP within `timeout` (5 s). Check that state is still
  arriving with `maker status`, and that a `trajectory()` loop is not overwriting the verb.
- `set_joints(wait=True)`: a joint did not reach its target within `tolerance`. It may be
  blocked, or the gains may be too soft to hold it.
- `wait_until()`: the predicate stayed false; the message names the robot mode and the age
  of the last sample.

## `stand()` Times Out and the Robot Stays DAMP

- **A fault is latched**: `stand()` raises `RobotFaultedError` instead; see above.
- **The robot dropped the command**: another source holds control. The Drive Robot page outranks a
  paired gamepad, which outranks udp, which outranks livekit, and a paired gamepad holds
  control while idle. Unpair it and try again.
- **The robot reports STAND but is not armed**: it is not upright. The firmware arms after
  0.5 s of STAND under 30 degrees of tilt; `robot.armed` reads the test.

## The Robot Tipped Over After `stand()` in MOVE

`stand()` is sent from any robot mode. STAND has no balance loop, so a free-standing robot
asked to stand from MOVE tips over. Before `stand()` in MOVE, hang the robot from its gantry
hook or seat it on a stool or bench; `rest.py` asks first. To stand still after a walk, call
`balance()`: zero velocity keeps the robot in MOVE, balancing in place. See
Bringing the Robot to Rest.

## `set_velocity` Returned but the Robot Did Not Walk

In order of likelihood:

1. The hold was cut short: with `wait=False`, `set_velocity` returns at once, and the next
   verb or the end of the `with` block ends the hold. Keep the connection open for the
   duration, or leave `wait` at its default so the call returns after the hold.
2. The robot is not in MOVE. `set_velocity()` is sent in any robot mode: in DAMP the robot
   drops it, and in a STAND that is not armed it leaves the robot in STAND. MOVE is
   entered only from STAND held upright for 0.5 s, and an early velocity is neither refused
   nor reported. Check `state.mode` and `robot.armed`, and call `stand()` then `balance()`
   first.
3. Another source holds control; see above. The state stream does not say who does.

## A `stand()` or `damp()` Did Nothing

A loop is streaming `trajectory()` setpoints and the next setpoint overwrote the verb. A
`stand()` sent into the loop is overwritten this way. A `damp()` usually holds, because
the robot drops trajectories once it reports DAMP, but a setpoint that arrives
first still wins. Stop the loop, then send the verb. A `set_joints()` does not have this
problem: any verb ends it.

## `StateStaleError`

The robot went quiet for longer than `stale_after` (default: the link timeout, 2 s) during
`wait_until()` or a command's wait. Treat it as absent, not slow: the connection dropped,
the robot's software stopped, or it is reporting to a different machine. `LinkLostError`
follows. A stream that is merely late before a command is sent is `stale_state`, waited out
and then a `NotReadyError`.

## `LinkLostError`

No state for 2 s. The SDK sent zero velocity if it held one, and every verb now raises this until you
`close()` and `connect()` again. On udp and hybrid, the robot also zeroes the velocity 2 s
after the last command it received.

## `NotConnectedError`

`robot.get_state()` or `damp()` before the robot reported (after
`connect(require_state=False)`), or any verb after `close()`. `close()` during a `set_velocity(..., wait=True)` raises this in the waiting
thread.

## `UnsupportedError`

This connection does not carry that capability. `camera`, `microphone` and `speaker` need
`hybrid` or `livekit`; on those, a capability is claimed only from a track that arrived
within `media_timeout` of connecting. Gate with `robot.has("camera")`.

## `ImportError` Naming Pillow, numpy or OpenCV

`Frame.to_jpeg`, `to_numpy`, `Clip.save_frames` and `Clip.save_mp4` need those packages at
call time. The SDK does not depend on them; install the one named.

## Every `Sent.wait_outcome()` Returns `Unknown`

The robot reports no per-command verdict, so `Unknown` is what every command gets and
`on_refused` never fires. `Unknown` is neither success nor refusal: read the effect with
`wait_until` and `robot.get_state()`, or let `stand()`, `balance()` and `damp()` wait for it.

## Velocity Is Slower Than Asked

Check `sent.clamped` and `sent.command`. The SDK clamps to `Limits`, whose defaults are the
firmware caps of 0.4 m/s and 0.8 rad/s; a robot does not walk faster than that. A tighter
limit came from `Robot(limits=)`, `MAKER_LIMITS`, or the saved robot's `limits`.

## Two Sessions Keep Disconnecting Each Other

Both joined the room with the same identity. With `ManagerConfig`, each session gets its own
unless you set the same `label`; with `LiveKitConfig`, one token is one participant, so mint
two.

## Related

- Safety: what each layer stops, what the SDK reports, and your own guard
- Troubleshooting the Robot: problems that are not the SDK's
