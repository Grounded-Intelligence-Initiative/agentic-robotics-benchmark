# Drive Robot Controls

> Drive the humanoid robot from a keyboard or a phone, set speed limits, and read telemetry and logs.

- Section: Humanoid › Operate the humanoid robot

Once the robot is in Walk — see Stand Up and Walk — drive it from the **Drive Robot** page, on a laptop or a phone. The camera panel says **Video isn’t available in the browser yet**, so drive within sight of the robot.

## How Input Works

- **Input is held.** Release a key, a touch pad or the stick and the robot's velocity goes to zero. It doesn't damp.
- **Further is faster.** How far you push sets the speed, up to your speed limits.
- **Losing focus is a stop.** If the Drive Robot tab loses focus, input clears and velocity goes to zero; you keep control. Keys do nothing while a dialog or the menu is open.
- **Missing input is a stop.** If Humanoid Manager hears nothing from your browser for two seconds, it sets the velocity to zero.
- **A dropped connection ends your control.** The page shows "Driving connection lost · reconnecting…" and reconnects without control; take control again.

## Keyboard

On a desktop:

| Key | Action |
| - | - |
| `W` / `S` | Forward / back |
| `A` / `D` | Strafe left / right |
| `Q` / `E` | Turn left / right |
| `Shift` | Half speed, while held |
| `Space` | Pause — see Pause |
| `X` | E-Stop |

The drag pad beside the keys does the same with a mouse: drag the knob further to go faster.

## Phone

- **Joystick** (the default): the **Drive** pad on the left moves forward, back and sideways; the **Turn** pad on the right turns; **Pause** sits between them. Touch and drag, and release to stop.
- **Buttons**: a D-pad and rotate keys that drive at your full speed limit. The page returns to Joystick on your next visit.
- **Landscape**: turn the phone sideways for a full-screen driving view. Set the control mode and speed limits in portrait first — landscape has neither.

On a phone, the Drive Robot page has no E-Stop button of its own: use **E-Stop** in the top bar — press and hold.



## Speed Limits

Select the speed chip (it reads like **0.4 m/s · 46°/s**) to open two sliders:

- **Max speed** caps forward, back and sideways speed, up to 0.4 m/s.
- **Turn speed** caps turning, up to 46°/s.

Both start at their maximum. Lower them before your first sessions — the robot has real momentum. The limits are saved in your browser only, so set them again on another device.

`Shift` halves your speed while held. A gamepad never exceeds these limits.

## Telemetry and Logs

Select **Logs** in the Drive Robot header to open the information pane. It docks beside the camera on a desktop and slides up as a sheet on a phone.

- **Telemetry**: control mode, command sequence and error flags; position, velocity, current and temperature for all 25 joints; the IMU; and battery charge, voltage, current, maximum cell temperature and protection flags. It updates live.
- **Logs**: the newest 40 entries from Humanoid Edge or the firmware, refreshed every two seconds. **Full logs** opens the Troubleshoot page.

The Drive Robot page's status line shows the robot's control mode, and **FIRMWARE FAULT · inspect telemetry before motion** when the firmware reports a fault; a **Robot stopped for safety** card lists why — see Recovering After an E-Stop or a Fall.

## If You Cannot Drive

| What you see | What to do |
| - | - |
| **Controls locked** | Take control first — see Take Control |
| "Switch to Walk to drive" | The robot moves only in Walk. Stand it up first |
| **Start Humanoid Edge** in place of the controls | Humanoid Edge, the service linking the console to the robot, isn't running. Follow the link and start it on the Firmware page |
| "You’re watching. Another window has the controls." | Another browser holds the session. Select **Take Control Here** to take over, or close it there |
| **Driven from a gamepad** | A gamepad paired to the robot is driving. **Take Control** to override it — see Gamepad |
| "Driving connection lost · reconnecting…" | Wait for it to reconnect, then take control again |
| **Can’t drive right now** | The page names what is missing — for example "Start the robot’s firmware first." — and links to the page that fixes it |
| The robot drifts or stumbles in Walk | Suspect a zero point — see Recalibration After Repairs |

## Driving from Code

The SDK and the Humanoid API can do everything the Drive Robot page does, using credentials issued on the Developer page. The robot takes commands from one controller at a time.
