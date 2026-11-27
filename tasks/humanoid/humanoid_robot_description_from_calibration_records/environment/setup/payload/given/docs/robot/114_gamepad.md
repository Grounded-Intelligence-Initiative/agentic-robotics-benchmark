# Gamepad

> Drive the humanoid robot with a gamepad, either paired to the robot over Bluetooth or connected to your browser.

- Section: Humanoid › Operate the humanoid robot

A gamepad can drive the humanoid robot in two ways, and they behave differently:

| | Paired to the robot | Connected to your browser |
| - | - | - |
| Connects to | The robot, over Bluetooth | Your computer, over USB or Bluetooth |
| Needs the Drive Robot page open | No | Yes, with a session held; it drives only in Walk |
| Lasts | Stays paired between sessions | This page visit only |
| Controls | Drive, postures and E-Stop | Drive, postures and E-Stop |

Either way, the Drive Robot page's rules apply: the robot moves only in Walk, releasing the stick stops it, and your speed limits cap everything. Read Stand Up and Walk first.

## Pair a Gamepad to the Robot

Open **Controller** in the sidebar. Nearby gamepads appear automatically while the page is open.

1. Check that the **Bluetooth** row is on. If it reads **Not responding**, turn it off and on again to reset it. **Blocked** means a hardware switch or the firmware blocks the radio, which the console can't clear.
2. Put the gamepad in pairing mode, wait for it to appear under **Nearby**, and choose **Connect**.
3. Once connected, it shows on the sidebar's device card, and the Drive Robot page announces it.



The **Diagnostics** section helps when pairing misbehaves.

## Who Drives

A Drive Robot session outranks a paired gamepad; with no session open, the gamepad drives.

- While the gamepad drives, the Drive Robot page shows **Driven from a gamepad**. **Take Control** takes over from it.
- While you hold a session, the Drive Robot page shows **Gamepad detected** with an **Enable** button. **Enable** ends your session and hands control to the gamepad.

A gamepad connected to your browser shows as **Gamepad detected on this computer**. **Enable** arms it for this page visit. Once you hold a session its posture buttons work, in the same Damp → Stand → Walk order as the on-screen switch, and it drives only while the robot is in Walk. Disconnecting it or leaving the page disarms it.



## Browser Gamepad Controls

These follow the standard gamepad layout:

| Control | Xbox | PlayStation | Action |
| - | - | - | - |
| Left stick | Left stick | Left stick | Drive: forward, back and sideways |
| D-pad | D-pad | D-pad | Drive, overriding the stick |
| Right stick | Right stick | Right stick | Turn |
| Bottom face button | `A` | `✕` | Walk |
| Right face button | `B` | `○` | Stand |
| Left face button, held for 2 s | `X` | `□` | Damp |
| Left trigger with the left face button | `LT` + `X` | `L2` + `□` | E-Stop |

## Paired Gamepad Controls

The Drive Robot page's **Controller Guide** (titled **Driven from a gamepad** on a phone) appears whenever a gamepad drives. Dismiss it with its ✕; the ⓘ on the gamepad pill brings it back. It lists:

| Control | Action |
| - | - |
| Left stick | **Drive** |
| D-pad | **Drive override** |
| Right stick | **Turn (yaw)** |
| `○` / `B` | **Stand** — balances on both feet |
| `✕` / `A` | **Walk** — moves only when you drive it |
| `□` / `X` | **Damp** — hold 2 s; support the robot first |
| `L2` / `LT` with `□` / `X` | **E-Stop** — fault damp; restart the firmware to recover |



> [!WARNING]
>
> **Test the buttons before you drive**
>
> The guide shows PlayStation and Xbox names side by side, because the robot can't always tell which make is paired. Before your first drive, try each button with the robot supported on its gantry hook, so you know which one changes posture.

## E-Stop from a Gamepad

`L2` / `LT` held with `□` / `X` damps the robot exactly like the console's E-Stop: at once, and a free-standing robot falls. The robot stays stopped until the firmware restarts. It works even while a dialog is open. To recover, see Recovering After an E-Stop or a Fall.
