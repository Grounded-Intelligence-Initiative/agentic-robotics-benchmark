# Assign Actuator IDs

> Program each of the 25 actuators with its sticker number, one at a time on the bench, before the robot is assembled.

- Section: Humanoid › Assemble the humanoid robot › Preparation

Each actuator needs its joint ID before it goes into the robot. You program the number on its sticker, one actuator at a time. Allow about an hour and a half.

> [!WARNING]
>
> **One actuator at a time, before assembly only**
>
> Programming writes to whatever is on the setup connector. With more than one actuator connected, it can permanently reprogram the wrong one. Never program with more than one actuator connected.

## Before You Start

- You are signed in to Humanoid Manager, and both RPU checks are green — see Check the RPU Link.
- All 25 actuators are within reach, each with its numbered masking tape.
- You have an XT30 (2+2) cable to connect each actuator to the **setup connector**, the left-leg actuator port on the RPU.

## Enable Programming

1. Open **Actuators** in the sidebar. It opens on the **Actuator ID** tab.
2. Read the warning, then select **Enable Programming**.



Programming stays enabled until you leave or reload the page.

If the tools are locked, a pill at the top of the page names the reason:

| Pill | Fix |
| - | - |
| **Stop the firmware to change actuators** | Select **Open Firmware ›** and stop the firmware |
| **RPU unreachable** or **RPU not controllable** | See Check the RPU Link |
| **Robot state unknown** | Select **Retry** |

## Program Each Actuator

Repeat for all 25 actuators:

1. **Connect one actuator** to the setup connector.

   

2. Select **Detect Actuator** to confirm it's connected. **Current ID** shows the ID it holds now. New actuators have ID 1 by default.

   

3. Type its sticker number, from 1 to 25, in **New ID**. In this example the tape reads 1, so enter 1.

   

4. Select **Program & Verify**. Check the ID and joint in the confirmation against the sticker, then select **Program Actuator**.

   > [!NOTE]
   >
   > **Leave Don't ask again unticked at first**
   >
   > **Don't ask again this session** skips this confirmation. Leave it unticked until you have the rhythm, because a wrong ID affects assembly and operation later.

   

5. Wait for "Actuator is now N · _joint_". Unplug the actuator and set it aside, in sticker order.

   

> [!NOTE]
>
> **Tips**
>
> - The map marks the lowest unprogrammed ID as **next up**, but any order works.
> - **Don't ask again this session** skips the confirmation. Leave it unticked until you have the rhythm.
> - Reloading the page clears the map, not the IDs. To recheck an actuator, connect it on its own and select **Detect Actuator**.

## If Something Goes Wrong

| Message | Fix |
| - | - |
| "No actuator detected. Check power and wiring." | Check that one actuator is connected, powered and firmly cabled. Wait a few seconds, then retry |
| "Couldn’t set the ID. Check that exactly one actuator is connected and powered." | Disconnect every other actuator, then retry |
| "Enter a sticker ID from 1 to 25" | Enter a number from 1 to 25 in **New ID** |
| The tools are greyed out | Read the pill at the top of the page — see Enable Programming |
| A sticker is missing, unreadable or duplicated | Stop, and sort out the labels before assembly |

## Next

> [!NOTE]
>
> **Confirm actuator ID assignment**
>
> On the website, readers tick this confirmation before the assembly steps unlock: "I have assigned and verified the correct ID for each of the 25 actuators, one at a time, matching its numbered label." It is the reader's own confirmation, saved in their browser, not an automatic check of the robot.

When all 25 are programmed, continue to Electronics Disassembly. During assembly, fit each actuator into the joint that matches its sticker. Afterwards, Calibrate Actuators checks all 25 and records their zero positions.

**Previous:** Enable the Battery BMS Switch · **Next:** Electronics Disassembly
