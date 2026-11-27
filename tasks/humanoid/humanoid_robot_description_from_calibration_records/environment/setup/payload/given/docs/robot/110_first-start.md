# First Start

> Confirm the robot is assembled, start the motion-control firmware for the first time and check that it came up healthy.

- Section: Humanoid › Operate the humanoid robot

Two steps remain, and they are deliberately separate: **confirming assembly** records the robot as built, and **starting the firmware** powers its actuators.

## Before You Start

- All **25 of 25** actuators respond without faults, and every joint is calibrated at the confirmed home pose.
- The robot hangs from its gantry hook or sits on a bench, the area is clear and hands are out of the joints.
- On the **Firmware** page, the **RPU** card's **Network** and **Control** checks are both healthy.
- You know how to cut power.

## Confirm Assembly

The firmware won't start until assembly is confirmed. Open **Firmware**; until you confirm, it shows a banner asking **"Is the robot fully assembled?"**



1. Select **Confirm Assembly**.
2. Select **Check Now** to sweep every actuator; it takes about 15 seconds. You want **25 of 25 actuators responding**. A result such as "Actuator 7 isn’t responding." sends you back to Calibrate Actuators.
3. Type `CONFIRM` and select **Confirm Assembly**. The page shows "Recorded. You can start the firmware now."

Confirming records the robot as assembled on the RPU, so the record survives a Raspberry Pi 5 reflash, and sets the robot's onboard services, including Humanoid Edge, to start on boot. It doesn't start the firmware, and it leaves the firmware's own **Start on boot** switch off.

> [!NOTE]
>
> **Humanoid Manager can't undo this**
>
> Reversing a confirmation takes `humanoidctl reset-setup` on the command line — see Command Line. Confirm only a robot that is fully assembled, wired and calibrated.

## Start the Firmware

> [!WARNING]
>
> **Starting the firmware powers the actuators**
>
> The actuators come on in Damp — powered but limp — so the robot can't hold itself up. Keep it supported and keep clear of the joints.

On the **Firmware** page, select **Start Firmware** and confirm. From now on you can also start the robot with **Start Robot** on Overview — see Power On and Off.

**Stop Firmware** is always available while the firmware runs, whatever else is wrong.

## Check It Came Up Healthy

Open **Overview**:



| Check | Healthy |
| - | - |
| Firmware | **Running** in the status strip |
| Actuators | **25 of 25 responding** |
| Battery | A percentage. A blank reading means Humanoid Edge isn't running |
| Setup checklist | Gone from the sidebar |

If anything is off, see Troubleshooting. Leave the firmware's **Start on boot** switch off for now; Start on Boot explains why.

**Next:** Stand Up and Walk — the robot is commissioned.
