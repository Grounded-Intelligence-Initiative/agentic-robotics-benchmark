# Stopping the Robot

> Every way to stop the humanoid robot, what each leaves the robot doing, and how to recover after an E-Stop or a fall.

- Section: Humanoid › Operate the humanoid robot

The humanoid robot has five stops, and they leave the robot in very different states. Pick the one whose result you can accept.

| Stop | Where | What happens |
| - | - | - |
| **Pause** | Drive Robot: `Space` or **Pause** | A walking robot stops moving and keeps balancing in place |
| **E-Stop** | Drive Robot, sidebar, phone top bar, gamepad | Every actuator damps at once. A free-standing robot **falls**. The firmware keeps running, and the robot stays stopped until it restarts |
| **Damp & shut down** | Drive Robot posture controls | The same release as an E-Stop, done deliberately on a supported robot |
| **Stop Robot** | Overview | The firmware stops and the actuators go offline. The console stays up |
| **Cut power** | The battery unit | Removes all power. The only stop that doesn't depend on software |

## Pause

Pause is the everyday stop. It sets the velocity to zero and the robot keeps balancing where it is. The Drive Robot page shows **Paused — holding still**; select **Resume Driving** to carry on.

Letting go of the keys, a touch pad or the stick also brings the robot to a halt: input is held, and release means zero velocity.

Use Pause only while the robot is walking. It always commands Walk at zero velocity, so pressing it in Stand or Damp asks the robot to walk.

## E-Stop

The E-Stop damps every actuator immediately, with no confirmation. Use it when a limp robot on the floor is better than whatever is happening now.

| Where | How |
| - | - |
| Drive Robot, on a desktop | **E-Stop**, or press `X` |
| Sidebar device card on a desktop, on every page | **E-Stop** — press and hold for about half a second |
| Top bar on a phone, on every page | **E-Stop** — press and hold for about half a second. A tap only flashes **Hold to E-Stop** |
| A gamepad | `L2` / `LT` with `□` / `X` — see Gamepad |

On a phone, the Drive Robot page has no **E-Stop** button of its own — use the one in the top bar. **The sidebar and top-bar E-Stops need no drive session:** anyone signed in can stop the robot.

> [!WARNING]
>
> **The confirmation only means the command was sent**
>
> "E-Stop latched. Restart the firmware before motion." confirms the send, not that the robot obeyed — watch the robot. And the **E-Stop** is disabled when **Humanoid Edge** isn't running, because there is no path to the robot; use **Stop Robot** on Overview instead, or cut power.

## Damp & Shut Down

Damping releases the actuators on purpose, for when the robot is supported and you are done. On the Drive Robot page, select **Damp**, tick **The robot is physically supported**, then hold **Hold 2 s to damp**. Bring It Back Down covers the full sequence from walking.

## Stop Robot

**Stop Robot** on Overview stops the firmware. All 25 actuators go offline and driving stops; Humanoid Edge and the console stay up. Support the robot first — see Power On and Off.

## Cutting Power

Press the button on the battery unit to power the robot down. Before working on the electronics, also disconnect the battery from the XT90 connector on the Robot Processing Unit (RPU), as described in Electronics Disassembly.

## Recovering After an E-Stop or a Fall

After an E-Stop or a fault, the robot is limp but its firmware is still running. Recover in this order:

1. **Wait until the robot is still.** Don't try to catch a falling robot.
2. **Find out why, before you turn it off.** While the firmware still runs, the Drive Robot page shows **Robot stopped for safety** with the causes, and **Error flags** in its Telemetry tab. Then read the **Firmware** log on the Troubleshoot page. Note the time — the logs roll over. Reading the screen doesn't involve touching the robot.
3. **Turn it off.** Select **Stop Robot** on Overview, so the actuators are unpowered before you touch the robot.
4. **Inspect and support it.** Check for damage, then lift the robot back onto its gantry hook or a bench — see Lifting and Carrying. After a hard fall, run **Check Actuators** on the Actuators page before going further.
5. **Start it again.** Select **Start Robot** on Overview, then stand it up from its support: Stand Up and Walk.

Restarting the firmware also clears a fault latched by a fall. Until the firmware restarts, the robot refuses to stand.

After an E-Stop the robot stays stopped until the firmware restarts, and the Drive Robot page's **Emergency stop triggered** card walks you through it: **Confirm** that the area is clear, then select **Restart** under **Restart firmware**. If the robot was still supported and nothing fell, you can then take control and select **Stand** straight away.
