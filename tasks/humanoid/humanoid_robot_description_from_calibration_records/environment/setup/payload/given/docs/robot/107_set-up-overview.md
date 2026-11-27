# Set Up Overview

> Commission an assembled humanoid robot: connect to it, calibrate its actuators, confirm assembly and start the firmware.

- Section: Humanoid › Operate the humanoid robot

Set Up takes an assembled robot to one you can drive: on your network, all 25 actuators verified and calibrated, assembly confirmed and the firmware running. You do it once per robot, and again only after repairs.

## Before You Start

- **Assembly is finished** and the Finish Assembly checks pass.
- **Both computers are flashed** — see Firmware & Software Flashing.
- **All 25 actuator IDs are assigned.** That happens before assembly, one bare actuator at a time — see Assign Actuator IDs. If it never happened, stop: an ID can't be assigned safely once the harness is in place.
- **You have read Safety & Handling.** Calibration and first start both involve supporting the robot.

## The Steps

1. **Connect & Sign In.** Reach Humanoid Manager in a browser, put the robot on your Wi-Fi and replace the default password.

2. **Calibrate Actuators.** Check that all 25 actuators respond, and record each one's zero position at the home pose. The firmware stays stopped.

3. **First Start.** Confirm assembly, start the firmware for the first time and check that the robot came up healthy.

## The Setup Checklist

Humanoid Manager tracks the same work in the sidebar's **Finish setting up** checklist:

| Row | Done when | Covered in |
| - | - | - |
| Reach the RPU | Humanoid Manager can control the Robot Processing Unit (RPU) | Connect & Sign In |
| Build the robot | Assembly is confirmed. Its sub-steps track the work: you tick **Assign actuator IDs** and **Assemble the robot** yourself, and the robot ticks **Check liveness** and **Check calibration** from its own checks | Calibrate Actuators, then First Start |
| Start the firmware | The firmware is running | First Start |
| Set a password | The default password is replaced | Connect & Sign In |
| Join Wi-Fi | Shown only while the robot is on its setup hotspot | Connect & Sign In |

Rows that depend on the RPU stay greyed out ("Waiting on the RPU") until it is reachable, so fix the link first. **Hide Checklist** hides it in your browser; while it is hidden, Overview and Settings show **Setup isn’t finished** with a **Show Setup Checklist** button that brings it back. The checklist closes once the robot is fully set up.

**Next:** Connect & Sign In.
