# Troubleshooting

> Symptom-first fixes: reach the console, restore the RPU link, find out why the robot stopped, and read its logs.

- Section: Humanoid › Operate the humanoid robot

Start from the symptom. Most fixes involve the **Troubleshoot** page, which shows the robot's health checks — the Robot Processing Unit (RPU) link, the video service and the Bluetooth radio — with a **Restart** button for each service. **Check Again** runs every check again.

> [!WARNING]
>
> **Restart doesn't ask first**
>
> **Restart** on the **Firmware (RPU)** row restarts the firmware immediately, which takes the actuators offline. Support the robot before using it.



## The Console Will Not Load

| Symptom | What to do |
| - | - |
| `http://<name>.local` doesn't answer | Allow two minutes from power-on — longer if the robot falls back to its hotspot — and check that your device is on the same network as the robot |
| `.local` doesn't resolve | Common on Android. On the setup hotspot, use ``. On your own network, use the address your router assigned the robot |
| It loads on the hotspot but not on your Wi-Fi | Some routers, especially guest networks, stop devices from reaching each other. Use your main network |
| The robot never joins a 5 GHz network | Try a 2.4 GHz network, or a 5 GHz channel from 36 to 48 |
| The setup hotspot never appears | It starts only when the robot has nothing to join: at power-on, after a failed join, or after you forget its last network |
| The robot dropped off your Wi-Fi | The hotspot doesn't return on its own once the robot has booted. Power-cycle the robot so it rejoins or starts the hotspot |

## The RPU Is Unreachable

Nothing that moves the robot works until this link is healthy. The **Firmware** page splits it into two checks:

| Failing check | Meaning | What to do |
| - | - | - |
| **Network** | The RPU isn't answering on the wire | Check the Ethernet cable between the Raspberry Pi 5 (part `700_09X`) and the Motion Control Board, and power to the Head Compute Unit and RPU. Power down before reseating connections |
| **Control** | The RPU answers, but the console can't manage it | The RPU is still booting or its services are down. Wait a minute and reload the page; if it persists, power-cycle the robot |

While the RPU is unreachable, the setup checklist greys out the rows that depend on it ("Waiting on the RPU"), and the Actuators and Firmware pages lock their tools. That is expected, not a second fault.

## The Firmware Will Not Start

| Symptom | What to do |
| - | - |
| **Start Robot** or **Start Firmware** is greyed out | Read its tooltip, which names what's missing |
| Assembly was never confirmed | Confirm it on the **Firmware** page — see First Start |
| **Waiting for actuator check…** | An actuator check is using the CAN buses for about 15 seconds. Wait, then try again |
| The RPU is unreachable | Fix the link first — see above |

## The Actuators Page Is Locked

Its tools work only with the firmware stopped and the RPU reachable. The lock pill names the cause — for example **Stop the firmware to change actuators**. Follow its **Open Firmware ›** link, fix the cause, then select **Retry**.

## The Robot Will Not Move

See If You Cannot Drive. The usual causes: no session, the robot isn't in Walk, Humanoid Edge isn't running, or a gamepad has control.

## The Robot Stopped or Fell on Its Own

The firmware damps the robot when it detects a fault. Look for **Robot stopped for safety** on the Drive Robot page, which lists the causes, and for **Error flags** in its Telemetry tab, then check the **Firmware** log for that moment. An overheating actuator and a lost command stream look the same from outside, and completely different in the log.

To get going again, follow Recovering After an E-Stop or a Fall.

## The Robot Drifts, Stumbles or Leans

Suspect a zero point, especially after a repair — see Recalibration After Repairs. Rule out wiring and actuator faults first.

## You Forgot the Password

See Forgotten Password.

## Reading the Logs

The **Logs** viewer on the Troubleshoot page shows five sources — **Edge**, **LiveKit**, **RPi kernel**, **Firmware** and **RPU kernel** — newest first, updating live. Scroll down for older entries.

**RPi kernel** is the Pi 5's kernel log; **RPU kernel** is the kernel log from the Radxa CM5 in the RPU.

Humanoid Manager keeps the newest 4,000 entries per source. Older history survives a restart only if persistent logging is enabled on that computer. There is no export, so when you report a problem, copy the relevant lines or take a screenshot while they are still there.

The Drive Robot page's Logs tab shows the Edge and Firmware logs without leaving the page, and Overview's Logs panel shows the newest lines across services.
