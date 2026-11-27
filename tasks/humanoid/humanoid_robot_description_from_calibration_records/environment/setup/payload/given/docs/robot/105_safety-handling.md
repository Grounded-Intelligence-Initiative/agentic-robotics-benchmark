# Safety & Handling

> The robot's states, how to support and carry it, and the rules for working around it. Read this before the firmware first runs.

- Section: Humanoid › Operate the humanoid robot

The humanoid robot weighs 35 kg and has 25 actuators. Read this page before you start the firmware for the first time, and Stopping the Robot before your first drive.

> [!WARNING]
>
> **Every stop in the console is software**
>
> The **E-Stop** and every other stop in Humanoid Manager are carried out by the robot's own computers. None of them replaces cutting power. Whenever the firmware is running, know how to cut power.

## The Robot's States

| State | What the actuators do | Holds itself up? |
| - | - | - |
| **Unpowered** | The firmware is stopped. Actuators are offline and every joint moves freely by hand | No |
| **Damp** | The firmware is running. Actuators are powered but limp | No |
| **Stand** | Actuators hold a standing pose | Keep it supported — see Stand Up and Walk |
| **Walk** | The locomotion policy balances the robot, and moves it only when you drive | Yes, on a flat floor |

> [!WARNING]
>
> **Handle the robot only while it is unpowered**
>
> Pose, inspect and carry the robot only while it is **Unpowered**. A damped robot is just as limp, but its actuators are live: a posture change, a fault or a stray command can move a joint without warning.

## Supporting the Robot

Until the robot is walking, support it in one of these ways:

- **Hanging from its gantry hook.** Keeps the legs free for calibration and standing up.
- **Seated on a bench.** Stable enough that the robot can go limp without tipping.
- **Held by a person** at the torso, never the joints, who expects it to go limp.

Support it again before you damp it or turn it off.

> [!WARNING]
>
> **A stop releases the actuators without warning**
>
> An **E-Stop** or a fault makes the robot go limp at once. Keep clear space around a free-standing robot.

## Lifting and Carrying

Until detailed lifting guidance is published:

1. Turn the robot off and cut power.
2. Carry it with two people, holding the torso. The limbs swing freely and shift the robot's weight as you move.

## Before Every Session

- **Clear the area.** Keep a space several times the robot's footprint free of people and obstacles.
- **Use a flat floor.** Never stand or walk the robot on a bench, on a slope or near an edge.
- **Keep it in sight.** The **Drive Robot** page shows no video in this release.
- **One person drives.** Only one Drive Robot session or gamepad controls the robot at a time. Make sure everyone nearby knows who has it.
- **Support it through transitions.** From firmware start until it is walking, and again before it damps or turns off.
- **Know your stops.** Read Stopping the Robot, including how to cut power.

**Next:** Set Up commissions a newly assembled robot.
