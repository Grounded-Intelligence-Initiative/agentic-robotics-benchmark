# Humanoid Manager

> Reference for the robot's on-board console: its pages, the Overview panels, account and access, and the command line.

- Section: Humanoid › Operate the humanoid robot

Humanoid Manager is the console that runs on the robot's Raspberry Pi 5 — think of it as the robot's router login page. You use it to set up, start, drive, update and troubleshoot the humanoid robot from a browser. There is no app to install and no cloud account.

Open it at `http://<name>.local`, where `<name>` is the hostname you set when flashing the Pi 5. The console serves plain HTTP on port 80. Until you set a password, the login is pre-filled, so the first sign-in is one click — see Connect & Sign In.

In the console, **RPU** means the Robot Processing Unit in the torso, whose Radxa CM5 runs the motion-control firmware.



## Pages

| Group | Page | What it's for |
| - | - | - |
| — | **Overview** | The robot's vital signs and its main action — see Overview |
| — | **Drive Robot** | Drive the robot — see Stand Up and Walk |
| Robot | **Actuators** | Assign IDs, check and calibrate actuators — see Calibrate Actuators |
| Robot | **Firmware** | Confirm assembly, start and stop the firmware, Start on boot and the RPU link — see First Start |
| Robot | **Audio & Video** | Choose which of the robot's devices it may use — **Camera**, **Microphone** and **Speaker**. Driving isn't affected |
| Connections | **Wi-Fi** | Join and forget networks — see Connect & Sign In |
| Connections | **Controller** | Pair a Bluetooth gamepad — see Gamepad |
| System | **Software Update** | Install a software update — see Software Updates |
| System | **Developer** | Issue credentials for the SDK — see Developer Credentials |
| System | **Advanced Settings** | Every setting on the robot, raw. Prefer the dedicated pages where they exist |
| System | **Troubleshoot** | Health checks, service restarts and logs — see Troubleshooting |
| User menu | **Settings** | The robot's name, your password, appearance, the console's version and settings reset — see Account and Access |

On a phone, the sidebar becomes a menu behind **Open Menu** in the top bar.

The sidebar's device card shows the hostname, overall health, battery and any connected gamepad on every page. The E-Stop is in the device card on a desktop and in the top bar on a phone, on every page; press and hold it — see Stopping the Robot.

## Overview

Overview has one main action, which depends on the robot's state:

| Firmware | Main action |
| - | - |
| Stopped | **Start Robot** |
| Running | **Drive Robot**, plus **Stop Robot** |
| RPU unreachable or not controllable | None — fix the link first |

Its panels:

| Panel | What to look for |
| - | - |
| Status pill | Beside the robot's name: **Ready to drive**, **Driven from a gamepad**, or the most urgent problem, such as **Can’t reach robot**, **Firmware stopped** or **Video off** (the robot's video service, LiveKit, isn't running). A note under it says what to do, and **Open Software Update** appears there when an update is ready |
| Status strip | **Robot link** (**Connected**, **Not controllable** or **Not responding**), **Firmware** (**Running** or **Stopped**), **Controller** (the paired gamepad, or **Not connected**) and **Wi-Fi** (your network's name) |
| Battery | A percentage beside the robot figure, amber below 50% and red below 20%. A blank reading means Humanoid Edge isn't running |
| Actuators | Each joint's result from the last actuator check, such as **25 of 25 responding · checked 09:02**. **Last checked 09:02** means the result is old, not that anything failed |
| Logs | **Logs** and **Recent events**, the newest lines across services, with **View All** for the full viewer |

## The Setup Checklist

Until the robot is fully set up, the sidebar shows a **Finish setting up** checklist. The Setup Checklist explains each row.

## Account and Access

### Password

Change it on **Settings**, from the user menu — see Set a Password.

### Forgotten Password

Humanoid Manager has no reset link. Over SSH, clear the stored password:

```bash
sudo humanoidctl config reset manager.password_hash
```

The login goes back to the pre-filled default. Sign in and set a new password straight away.

### Reset All Settings

**Reset All Settings**, under **Advanced** on the Settings page, returns the robot's RPU, video (LiveKit) and Humanoid Edge settings to factory defaults. Humanoid Edge's parameters, including the robot's serial, go back to their defaults too.

It keeps your login, saved Wi-Fi networks, SDK credentials, the assembly confirmation, and actuator IDs and calibration.

### Developer Credentials

A developer credential lets a computer running the Python SDK join the robot's LiveKit room. You need one to connect the SDK in its **hybrid** or **livekit** connection mode, which is also how the SDK sees the camera and hears the microphone. The Drive Robot page needs no credential, and neither does the SDK's **udp** mode.

The robot runs its own LiveKit server and is set up to use it from the first boot, so there is nothing to configure before you issue a credential.

**Issue a Credential**

Open **Developer** in the sidebar. Under **Issue a credential**, name the computer that will use it, choose **Observe** or **Control**, and select **Issue Credential**. The credential is shown once, so copy it then; if you lose it, revoke it under **Issued credentials** and issue another. Drive from Python walks through the page step by step and connects the SDK with `maker setup`.

**Observe and Control**

| Role | What it allows in the LiveKit room |
| - | - |
| **Observe** (default) | Watch the camera and listen to the microphone. The LiveKit server refuses its commands, whatever program holds it |
| **Control** | Everything Observe allows, plus sending commands and talking through the speaker. Commands still pass through the robot's safety layer, and the console keeps priority |

The role limits driving only in the **livekit** mode. In **hybrid**, commands travel over UDP on the robot's network, which has no sign-in, so an Observe credential still gets the camera and microphone and the SDK can still drive. Hybrid also needs the robot to accept UDP commands and send state to your computer; see Hybrid.

**Revoke a Credential**

Under **Issued credentials**, select **Revoke** next to the credential and confirm. The computer holding it can no longer connect. A session that is already running keeps working until its current LiveKit token expires, at most 12 hours.

**Reset All Settings** keeps issued credentials.

**The LiveKit Secret Stays on the Robot**

The credential is not the LiveKit key or secret. Each time the SDK connects, it trades the credential for a short-lived LiveKit token from Humanoid Manager, so the key and secret never leave the robot and you never need them.

To use a LiveKit server other than the robot's own, set **LiveKit URL**, **LiveKit key** and **LiveKit secret** under **Humanoid Edge** in **Advanced Settings**, and select **Save & Restart Humanoid Edge**. Saving checks that the server accepts the key and secret first. If the **LiveKit URL** row on the Developer page reads **not set**, the robot publishes no video or audio until you set it here.



### Command Line

Everything the console does is also available over SSH on the Pi 5. Sign in with the username and password you set in Raspberry Pi Imager when flashing it, then use `humanoidctl` with `sudo`:

```bash
humanoidctl status                        # services, RPU link and setup state
humanoidctl start|stop|restart <service>  # humanoid-edge, humanoid-firmware or livekit-server
humanoidctl enable|disable <service>      # start on boot
humanoidctl config show                   # every setting, secrets masked
humanoidctl config set <key> <value>
humanoidctl config reset <key|section>
humanoidctl apply                         # write settings to the services
humanoidctl reset-setup --yes             # undo Confirm assembly; stops the firmware
humanoidctl update status                 # the current software update
```

The browser and the command line control the same services, so anything you start from one you can stop from the other.

## What It Manages

Humanoid Manager and Humanoid Edge run on the Pi 5. Humanoid Edge carries commands and telemetry between the console and the RPU, whose Radxa CM5 runs the motion-control firmware. The Pi 5 and RPU share a private Ethernet link, which the console reports as two checks: **Network** (the RPU answers on the wire) and **Control** (the console can manage it). An unplugged cable and a stopped service never look the same.

## What It Doesn't Do

- **Run the control loop.** The firmware on the CM5 does; the console starts it, stops it and relays your drive input.
- **Command actuators while the firmware runs.** The Actuators page works only with the firmware stopped.
- **Show video.** The Drive Robot page's camera panel reads **Video isn’t available in the browser yet**.
- **Restart or shut down the robot's computers.**
- **Manage more than one robot.** There is no fleet view.
- **Flash boards.** It is installed and updated as part of the robot's software.
