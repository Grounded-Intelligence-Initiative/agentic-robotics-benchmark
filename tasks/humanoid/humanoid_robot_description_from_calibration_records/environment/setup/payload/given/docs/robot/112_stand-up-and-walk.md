# Stand Up and Walk

> Take control on the Drive Robot page, bring a supported robot from Damp to Walk, release its support, and bring it back down.

- Section: Humanoid › Operate the humanoid robot

Every drive starts and ends here. The robot is always in one of three postures — **Damp**, **Stand** and **Walk** — and you move between them one step at a time. This page takes a supported robot to walking on its own, and back.

> [!WARNING]
>
> **This moves a 35 kg robot**
>
> Keep the robot supported until it is in Walk, and support it again before you leave Walk. Clear the area, keep the robot in sight, and know your stops.

## Before You Start

- The firmware is running: Overview shows **Drive Robot**. If it doesn't, see Start the Robot.
- The robot hangs from its gantry hook with both feet flat on a clear, level floor.
- For your first sessions, lower **Max speed** well below its default — see Speed Limits.

## Take Control

Open **Drive Robot** from the sidebar, or select **Drive Robot** on Overview.

1. Select **Take Control**. The page asks: "Area clear? Check around the robot before it can move."
2. Check around the robot, then confirm. On a desktop, select **Start Driving** before its ten-second countdown ends. On a phone, press and hold **Start Driving** for one second.

You now hold the robot's only drive session, and the posture controls unlock. Taking control sends nothing to the robot by itself.



Only one window drives at a time. A second browser opens in watch mode — "You’re watching. Another window has the controls." — and its **Take Control Here** takes the session over. A gamepad paired to the robot drives only when nobody holds a session — see Gamepad.

## The Three Postures

| Posture | The Drive Robot page says | Support |
| - | - | - |
| **Damp** | "Releases all actuators instantly. The robot will fall unless it’s seated on a bench or hanging from its gantry hook." | Required |
| **Stand** | "Actuators engage, the robot rises and balances on both feet." | Keep it on the gantry hook |
| **Walk** | "Accepts drive input — moves only when you drive it." | None, on a flat floor |

On a desktop, select a posture on the **Damp** · **Stand** · **Walk** switch. On a phone, the posture pill opens the **Robot posture** sheet. You can't skip a step: on a desktop, asking to go straight from Damp to Walk, or back, opens a guide (**Start Walking**, or **Damping the robot**) that takes you through Stand; on a phone the far step stays locked.

## Stand Up

If the robot is lying on the ground, use the easier method shown in the video below:

Video: How to stand up the robot from the ground

You have two options:

1. **Recommended:** Follow the video above to stand the robot up from the ground. Keep the area clear and stay ready to use the stop procedure.
2. **Alternative:** Follow this order:
   1. Select **Stand**. The robot stiffens into Stand mode.
   2. Lift the robot straight up.
   3. Hold the robot and do not let go.
   4. Select **Walk** so the robot can auto-balance.
   5. Let go only after **Walk** is active and the robot is balancing.

After either option, wait until the Drive Robot page shows the robot in **Stand**. **Walk** unlocks only once the robot reports it.

## Start Walking

1. Select **Walk** (**Start Driving** on a phone). The robot balances in place and moves only when you drive it.
2. Slacken the gantry gradually until the robot carries its own weight.
3. Drive with the Drive Robot controls or a gamepad.

## Bring It Back Down

Never damp a free-standing robot — it falls.

1. **Stop driving.** Select **Pause** or press `Space`, somewhere level, with the robot not mid-stride.
2. **Support it.** Take up the gantry's slack so it holds the robot again.
3. **Select Stand.** On a phone, press and hold **Hold to stand** for one second.
4. **Select Damp.** Tick **The robot is physically supported**, then hold **Hold 2 s to damp**.

On a desktop, selecting **Damp** straight from Walk opens a guide that takes you through Stand first.

## End Your Session

On a desktop, select **End Control**. On a phone, leave the page — End Control isn't available there.

Ending your session stops the Drive Robot page sending commands. It doesn't change the robot's posture or stop the firmware, so bring the robot down before you leave. Then turn it off when you are done.
