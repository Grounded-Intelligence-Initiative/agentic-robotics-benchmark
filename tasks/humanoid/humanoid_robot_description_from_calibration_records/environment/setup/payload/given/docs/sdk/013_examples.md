# Examples

> The runnable scripts that ship with the SDK, grouped by what they show.

- Section: Python SDK

Each file is one short, runnable script, kept in the
examples directory of the
SDK repository. Every script except `connect.py` and the `livekit_raw/` scripts connects to
the saved robot (`maker setup`), or to the robot the `MAKER_*` environment variables describe; set `MAKER_ROBOT=NAME` to pick another saved
robot. The settings a script uses, such as a speed, a duration or a joint name, are constants
at the top of the file. Read the docstring at the top of a file before you run it: it says
what the robot must be doing. Before a script that moves the robot, work through
Before You Run a Script.

A script with a **Run** button runs on a simulated humanoid robot: the robot opens in the
corner of the page. Its questions about supporting the robot are answered yes there, since
the simulation holds the robot up.

The SDK sends what a script asks and reports what the robot says; it refuses a command only
when there is no live state. A guard is yours: `guard.py` is an example of one, and no
script uses it by default. Each motion script carries two commented guard lines; uncomment
them to stop the script, before it sends anything, when the robot reports what your rule
will not drive through, and edit the limits in `guard.py`. Nothing here is an emergency
stop: use the E-Stop in Humanoid Manager, or cut power at the battery unit.

```bash
pip install maker-sdk
git clone 
cd maker-sdk
python examples/check.py
```

## Check and Guard

| Script | What it shows |
| - | - |
|  (runs on the simulated robot) | the robot's facts (robot mode, armed, faults, alerts, the hottest joint, battery, state age) and `preflight()` for stand, move and trajectory; sends nothing |
|  (runs on the simulated robot) | an optional guard of your own: joint temperature, battery, a latched fault and active alerts, with limits at the top of the file; prints what it finds and exits with status 1 when the rule fails. Off by default: the motion scripts carry it commented out |

## Connect

| Script | What it shows |
| - | - |
|  | the `udp`, `hybrid` and `livekit` configs side by side in code; set `MODE` at the top and it prints the endpoint, robot mode, arming, battery and capabilities |

## State

| Script | What it shows |
| - | - |
| read_state.py (runs on the simulated robot) | a tour of `robot.get_state()`: mode, arming, faults, battery, the hottest actuators, gravity, alerts, state age and rate, and the mode and alert callbacks |

## Stand, Balance, Walk, Rest, Damp

Run them in this order, with the robot on its feet, hanging from its gantry hook.

| Script | What it shows |
| - | - |
|  (runs on the simulated robot) | `stand()` from any robot mode; it returns once the robot is armed, and the script prints the `NotReadyError` or `WaitTimeoutError` otherwise. From MOVE, hang the robot from its gantry hook or seat it on a stool or bench first |
|  (runs on the simulated robot) | `balance()` from an armed STAND: the robot enters MOVE and balances in place; the script prints the `NotReadyError` or `WaitTimeoutError` otherwise |
|  (runs on the simulated robot) | a bounded `set_velocity(duration=3.0)` in MOVE: the SDK holds the velocity, sends zero, and the robot balances in place; reports `sent.clamped`. Walking in MOVE only is the script's own rule |
| stream_velocity.py (runs on the simulated robot) | `set_velocity(hold=False)` from your own 50 Hz loop, one packet per tick, then `balance()`; MOVE only, the script's own rule |
|  (runs on the simulated robot) | MOVE to STAND to DAMP, after asking whether the robot is on its gantry hook or seated on a stool or bench; prints each robot mode |
|  (runs on the simulated robot) | `damp()` on a supported robot, after asking; it returns once the robot reports DAMP |

## Keyboard

| Script | What it shows |
| - | - |
|  | drive from the keyboard with short bounded holds: `w`/`s` walk, `a`/`d` strafe, `q`/`e` turn, space `balance()`, `t` stand (asks first in MOVE), `b` damp after asking, `x` quit |

## Joints

| Script | What it shows |
| - | - |
| move_joints.py (runs on the simulated robot) | `set_joints()` on one named joint, on a supported robot; exits unless `ROBOT_SUPPORTED = True` is set in the file; the robot enters DAMP 2 s after the script exits |
| wait_until.py (runs on the simulated robot) | `set_joints(wait=False)`, then `wait_until()` acts the moment the joint passes an angle in the robot's own report, on a supported robot; exits unless `ROBOT_SUPPORTED = True` |

## Media

| Script | What it shows |
| - | - |
| camera_and_audio.py (runs on the simulated robot) | a photo, a short clip saved as JPEG frames with its sound as WAV, and a tone on the speaker; `hybrid` or `livekit` |
| record_audio.py (runs on the simulated robot) | three seconds of the microphone saved as WAV |
| play_audio.py (runs on the simulated robot) | a WAV file played on the robot's speaker, one second at a time |

## Recording

| Script | What it shows |
| - | - |
| record_and_replay.py (runs on the simulated robot) | `robot.record()` for three seconds, then reading the file back with `recording.load()` |

## Without the SDK

The scripts in `livekit_raw/` use only the `livekit` and `humanoid-protocol` packages; see
Without the SDK. They read `MAKER_CREDENTIAL` and the
`MANAGER_URL` set in `manager_token.py`.

| Script | What it shows |
| - | - |
| livekit_raw/manager_token.py | a join token from Humanoid Manager; the other scripts in the folder import it |
| livekit_raw/read_state.py | `RobotState` from the `state` data track for 5 s |
| livekit_raw/send_commands.py | its own guard by hand, then one STAND `RobotCommand` on the `commands` topic; the robot must hang from its gantry hook |
| livekit_raw/camera.py | one frame from the video track, saved as PPM |
| livekit_raw/audio.py | the microphone track, saved as WAV |

## Related

- Quickstart: check, stand, balance, walk and damp, step by step
- Safety: writing your own guard, and bringing the robot to rest
- Command Line: the same verbs without writing a script
