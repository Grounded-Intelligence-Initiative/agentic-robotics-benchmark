# Command Line

> maker: save a robot, check it, and stand, balance, walk and damp it from a terminal.

- Section: Python SDK

`pip install maker-sdk` installs the `maker` command. It saves robots for the SDK and drives
one through the same verbs a script uses. `stand`, `balance`, `walk` and `damp` show what
they are about to do, with the robot's facts, and ask before they send anything. The facts
are for you to judge: no command refuses because of them, and the firmware decides what a
command does. The one refusal is no live state. `balance` on a robot already in MOVE sends
at once without asking. Every command is sent to the robot; none is an emergency stop.

```bash
maker --robot NAME --mode {udp,hybrid,livekit} <command>
maker --version
maker <command> -h
```

`--robot` picks a saved robot, `--mode` overrides its connection mode for this command.
Without them, the command uses `MAKER_ROBOT`, then the default saved robot, then the only
saved robot.

## Setup

```bash
maker setup
```

The wizard asks for a name, a connection mode, and only the fields that mode needs: the
robot's address for `udp`, the Humanoid Manager URL and an SDK credential for `livekit`, both
for `hybrid`. The credential is typed masked and never printed back. Each field is checked
live before it is saved:

| Check | What passes |
| - | - |
| **Credential** | `accepted, role control, room robot-...`; a credential with the Observe role saves with a warning: it can watch, not drive over livekit |
| **udp** | state arrives from the address; the message names the robot mode it saw |
| **livekit** | the room is joined; the message lists the tracks it found |

A failed check asks "Save anyway?". When another robot is already the default, the wizard
asks whether this one should be. `--no-check` skips the live checks. The wizard needs a
terminal; without one it exits 2. SDK credentials come from the Developer page in Humanoid
Manager; Drive from Python
shows the steps.

Saved robots live in `~/.maker/robots.toml`, readable only by you.

## Robots

```bash
maker robots                 # table: default mark, name, mode, address, manager, room
maker robots add lab --mode hybrid --udp 192.168.1.20 --manager  --credential ...
maker robots add lab --limits 0.3,0.3,0.6
maker robots use lab
maker robots remove lab
```

`add` on an existing name keeps the saved values it is not given, so the second `add` above
only sets limits. Flags it needs and does not have are asked for in a terminal; with
`--no-input`, missing flags exit 2. `--default` makes the robot the default; `--no-check`
skips the live check. With every flag it needs, or with `--no-input`, a failed check saves
nothing and exits 1; in the wizard, you can choose to save despite a failed check.
`use` and `remove` exit 1 on a name that is not saved.

## Status

```bash
maker status
maker status --watch
```

Connects, reads state for a second, and prints a panel: the robot, its connection mode and
address; a badge, NOT READY when there is no live state, FAULTED when the firmware latched
DAMP, READY otherwise, with the reason; the robot mode and whether it is armed; battery; the
hottest joint; faults; alerts; the state rate; and whether the state is fresh. It sends
nothing. `--watch` refreshes until `q` or Ctrl-C.

## Stand, Balance, Walk, Damp

```bash
maker stand
maker balance
maker walk --vx 0.2 --duration 3
maker walk --vyaw 0.3 --duration 2
maker damp
```

| Command | What it does |
| - | - |
| `stand` | from any robot mode. A robot already in STAND is sent nothing; if it has not armed, `maker stand` waits up to 10 s for it to arm and exits 3 when it does not. From MOVE it first warns to hang the robot from its gantry hook or seat it on a stool or bench: STAND has no balance loop. Asks, sends `stand()` and waits up to 10 s for the robot to report STAND and arm, then prints `Armed: ready to balance, maker balance`. Exit 3 when it does not arm |
| `balance` | outside MOVE, asks, sends zero velocity and waits up to 5 s for the robot to report MOVE, then prints `Balancing in MOVE: ready to walk, e.g. maker walk --vx 0.3 --duration 3`. From DAMP or an unarmed STAND it exits 3 saying why. In MOVE, never asks: sends zero velocity twice, 0.1 s apart, and the robot stays in MOVE, balancing in place. A script that holds a velocity re-sends it at 10 Hz and must itself be stopped |
| `walk` | `--duration` is required, above 0 and at most 10 s; at least one of `--vx`, `--vy`, `--vyaw` must be nonzero. Asks, holds the velocity for the duration, then sends zero velocity: the robot stays in MOVE, balancing in place. The firmware walks in MOVE only: a walk that leaves the robot outside MOVE exits 3 with `Run maker balance first`. Ctrl-C sends zero velocity and exits 130. A fault that ends the walk early is named, with a note that it latches until the firmware restarts, and exits 3 |
| `damp` | asks, sends `damp()` and waits up to 10 s for the robot to report DAMP, then prints `Robot mode DAMP.`, or the fault that holds it. Hang the robot from its gantry hook or seat it on a stool or bench first |

> [!WARNING]
>
> **damp is not an emergency stop**
>
> DAMP makes every actuator compliant: a standing robot folds to the ground, so hang it from
> its gantry hook or seat it on a stool or bench first. In an emergency use the **E-Stop** in
> Humanoid Manager, or cut power at the battery unit; see
> Stopping the Robot.

### Confirm Before Sending

`stand`, `balance` outside MOVE, `walk` and `damp` print one line: the robot, its connection
mode and address, the robot's facts (robot mode, armed, faults, active alerts, the hottest
joint, battery), and what will happen. Then they ask `Proceed? [y/N]`.

```text
$ maker walk --vx 0.2 --duration 3
lab (hybrid, 192.168.22.32) · MOVE · armed · faults none · alerts none · hottest joint 41 C (L_Knee) · battery 82 % → walk vx 0.20 m/s, vy 0.00 m/s, vyaw 0.00 rad/s for 3.0 s, then balance in place
Proceed? [y/N] y
Walking. Ctrl-C ends the walk.
Walk done. Robot mode MOVE, balancing in place.
```

```text
$ maker stand
lab (udp, 192.168.22.32) · DAMP · faults none · alerts none · hottest joint 38 C (L_Knee) · battery 82 % → stand, then wait until armed
Proceed? [y/N] y
Standing. Waiting for the robot to arm (0.5 s upright)...
Armed: ready to balance, maker balance
```

```text
$ maker balance
lab (udp, 192.168.22.32) · STAND · armed · faults none · alerts none · hottest joint 41 C (L_Knee) · battery 82 % → balance: MOVE at zero velocity, the walking policy balances the robot
Proceed? [y/N] y
Balancing in MOVE: ready to walk, e.g. maker walk --vx 0.3 --duration 3
```

```text
$ maker damp
lab (udp, 192.168.22.32) · MOVE · armed · faults none · alerts none · hottest joint 45 C (L_Knee) · battery 80 % → damp: every actuator stops holding its position and a standing robot falls, so the robot must be supported. Not an emergency stop: use the E-Stop in Humanoid Manager, or cut power at the battery unit.
Proceed? [y/N]
```

1. Read the line and judge the facts: a fault, an alert, a hot joint or a low battery does
   not stop the command. A speed above the robot's velocity limits shows the value that is
   sent, then the one you asked for, for example `vx 0.40 m/s (asked 0.60)`.
2. Type `y` and press Enter to go ahead. Enter alone, or anything else, cancels: the command
   prints `Cancelled; nothing sent.` and exits 4.

`-y` or `--yes` prints the line and goes ahead without asking. With no terminal to ask on
and no `--yes`, the command sends nothing and exits 2.

### Not Feasible

The one refusal is no live state. Before the plan, `stand`, `balance` and `walk` check for a
fresh state sample. Without one they do not ask: they print `Not feasible:` on stderr, send
nothing and exit 3:

```text
Not feasible: no fresh state from lab: the latest state is 1.2 s old (limit 0.5 s). Check the link with `maker status`.
```

On `udp` or `hybrid`, a robot that never reports state is often a firewall; see
No State on udp or hybrid.

A command that was sent and did not get there also exits 3, with what the robot reports:
`balance` in DAMP (`the robot is in DAMP: stand() first`), `balance` on a robot that has not
armed, `walk` that left the robot outside MOVE, a `stand` that does not arm within 10 s, and
a fault during a command.

## For Scripts and Other Programs

`maker robots --json` and `maker status --json` print one JSON object and nothing else.
`--yes` runs `stand`, `balance`, `walk` and `damp` without a question; branch on the exit
code.

A script or another program runs `maker status --json` first, judges the facts itself, then runs
`maker balance --yes` and `maker walk ... --yes`:

```bash
maker status --json | jq '.ready, .robot_mode, .faults, .alerts, .max_joint_temp_c'
maker walk --vx 0.2 --duration 3 --yes
```

`status` carries `robot`, `mode`, `host`, `verdict`, `ready` (there is live state),
`problems` (each with `code`, `message`, `blocking`), `robot_mode`, `armed`,
`battery_percent`, `faults`, `alerts`, `error_flags`,
`max_joint_temp_c`, `state_rate_hz`, `state_age_s` and `fresh`. `robots` carries `default`
and a `robots` list with `name`, `default`, `mode`, `udp_host`, `manager_url`, `room`,
`credential_saved` and `limits`; the credential itself is never printed.

## Exit Codes

| Code | Meaning |
| - | - |
| **0** | done |
| **1** | an error; the message is `maker <command>: <message>` on stderr |
| **2** | usage: a missing flag, a bad value, a wizard without a terminal, or a question with no terminal and no `--yes` |
| **3** | not feasible or not reached: no live state (`Not feasible:` is printed, nothing sent), or the command was sent and the robot did not get there (not armed in time, still in DAMP, a fault) |
| **4** | cancelled: you answered no; nothing was sent |
| **130** | interrupted with Ctrl-C |

## Related

- Connection Modes: what each mode needs
- Safety: the stand, balance, walk, damp sequence and why
- What the SDK Reports: every code `preflight()` reports
- SDK Reference: the store the CLI writes
