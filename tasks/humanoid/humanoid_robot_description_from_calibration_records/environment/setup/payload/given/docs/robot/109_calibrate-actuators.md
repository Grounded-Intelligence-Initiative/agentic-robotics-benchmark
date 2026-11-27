# Calibrate Actuators

> Check that all 25 actuators respond and record their zero positions, before the firmware ever runs.

- Section: Humanoid › Operate the humanoid robot

Check that all 25 actuators answer without faults, then record each one's zero position at the robot's home pose. Everything happens on the **Actuators** page of Humanoid Manager.

> [!WARNING]
>
> **Keep the firmware stopped and the robot supported**
>
> Support the robot and its limbs so nothing can fall or sag, keep people clear of the joints, and know how to cut power. The firmware stays stopped until First Start.

## Before You Start

- The Finish Assembly checks pass, with every actuator in the joint matching its sticker number. Inspect wiring with the battery disconnected, and never connect or reseat an actuator cable while powered.
- On the **Firmware** page, the **RPU** card's **Network** and **Control** checks are both healthy.
- The firmware is **stopped**. On a robot that has run before, stop it from **Firmware** first.
- You can pose the robot at its home position and hold it still. It is easiest with two people: one holds the pose, the other works the console.

Open **Actuators**. Its tools work only while the firmware is stopped and the RPU is reachable, because the running firmware owns the actuators' CAN buses. If the page shows a lock pill such as **Stop the firmware to change actuators**, follow its **Open Firmware ›** link, fix the cause, then select **Retry**.

The page has three tabs — **Actuator ID**, **Liveness Check** and **Calibration**. You can open them in any order. The setup checklist ticks **Check liveness** and **Check calibration** from the robot's own checks; the rest of the checks below are yours to make.

## Actuator IDs Are Already Assigned

Each actuator's ID tells it which joint it is. IDs are assigned **before assembly**, one bare actuator at a time on the setup connector, with the **Actuator ID** tab. On an assembled robot this is already done.

> [!WARNING]
>
> **Don't use the Actuator ID tab on an assembled robot**
>
> Assigning an ID permanently programs whichever actuator is on the setup connector, and an assembled harness has no safe way to single one out. If IDs were never assigned, follow Assign Actuator IDs with the actuators loose on the bench.

## Check Liveness

Open the **Liveness Check** tab and select **Check Actuators**. Nothing moves; this only reads.

You want **All 25 actuators responding**, with every joint green on the map. Orange means **responding with error**; red means **no response**.



| Column | Meaning |
| - | - |
| Actuator | The ID assigned before assembly |
| Joint | The joint that ID belongs to |
| Status | **alive** or **no response**. The badge turns orange when a live actuator reports an error |
| Position (rad) | Current position, in radians |
| Temp °C | Actuator and driver temperature |
| Error | The fault it is reporting, if any |

**Don't continue until all 25 rows are healthy.** Nothing in the software stops you from carrying on with a short count, and an actuator that doesn't answer here becomes an uncontrolled joint later.

| What you see | What to do |
| - | - |
| A joint shows **no response** | Power down, then inspect its power and CAN connections. The ID tells you where in the chain to look. Restore power and check again |
| A joint is alive but reporting a fault | Resolve the fault first. If it is overheating, let it cool, then check again |
| Far fewer than 25 respond | Suspect a whole branch: each limb is one daisy chain running outward from the body |

> [!NOTE]
>
> **Watch the complete calibration video before calibrating**
>
> Watch the full video below and continue watching until the calibration process is complete. Then follow the written steps for the same process in Humanoid Manager.

Video: How to calibrate the actuators

## Calibrate

Calibration defines the robot's home pose: each actuator stores its current position as zero, and every joint angle is measured from there. A joint calibrated while drooping is off by that much on every move afterwards.

> [!WARNING]
>
> **Confirm the home pose before calibrating**
>
> This guide does not yet specify the joint-by-joint home-pose angles or an alignment fixture. Don't assume that any upright or straight-looking pose is correct. If you don't have the confirmed home-pose reference for your robot, stop here and get it first.

1. Open the **Calibration** tab.
2. Pose the robot at its confirmed home position. The joints move freely by hand, so support each limb rather than expecting it to stay put.
3. Read the warning and select **Enable Calibration**.
4. Pick a joint under **Joint**, select **Calibrate**, then **Calibrate Actuator** to confirm. The robot reads the new zero back and marks the joint **calibrated** only if it landed within 0.02 rad.
5. Repeat for all 25 joints — there is no bulk action. Redo any joint marked **failed** or **calibrated, unverified**. Calibrating a joint again simply overwrites its zero.



## Verify Before Starting the Firmware

Keep the robot supported at the home pose, with the firmware stopped.

1. Every joint on the map reads **calibrated**. Treat any other result as unfinished.
2. Run **Check Actuators** again: **All 25 actuators responding**, with no errors.
3. Each joint's **Position (rad)** reads close to 0 while the robot is still at the home pose. Investigate any joint that doesn't.
4. No limb sagged or shifted during calibration. If one did, restore the pose, recalibrate that joint and check again.

These checks confirm reported health and the recorded zeros. They are not a motion test — don't start the firmware and drive the robot as a shortcut to checking calibration.

## Recalibration After Repairs

IDs and zero points are stored in the actuators themselves, so they survive a restart, a Raspberry Pi 5 reflash and a software update. Recalibrate a joint only when its zero is wrong:

- **After replacing an actuator.** Assign its ID with the single-actuator bench procedure first.
- **When a joint was calibrated in the wrong pose.**
- **When the robot drifts, stumbles or leans** and the liveness check and firmware log show nothing wrong.

Stop the firmware, then check liveness and calibrate the affected joint at the home pose. Check all 25 before starting the firmware again.

Recalibration doesn't fix wiring faults, an actuator that stops responding or a robot that damps itself — calibrating over those only hides the evidence. Diagnose them first with **Check Actuators** and the firmware log.

**Next:** First Start.
