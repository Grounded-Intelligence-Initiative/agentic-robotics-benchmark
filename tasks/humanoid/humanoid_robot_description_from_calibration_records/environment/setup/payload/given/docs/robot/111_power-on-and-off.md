# Power On and Off

> Power the robot on, start it, turn it off, power it down, and decide whether the firmware starts on boot.

- Section: Humanoid › Operate the humanoid robot

A session with a commissioned robot runs: power on, start the robot, drive, turn the robot off, power off.

## Power On

1. Support the robot on its gantry hook or a bench.
2. Power it on at the battery unit.
3. Wait about two minutes. The Raspberry Pi 5 joins a saved Wi-Fi network, or starts its setup hotspot if it can't, and its link to the Robot Processing Unit (RPU) comes up.
4. Open `http://<name>.local`.

The firmware starts by itself only if its **Start on boot** switch is on, which it isn't by default. Otherwise the actuators stay unpowered and the robot stays limp until you start it.

## Start the Robot

1. Check that the robot is supported and everyone is clear of it.
2. On **Overview**, select **Start Robot** and confirm.

Start Robot starts Humanoid Edge if it isn't running, then the firmware. The actuators come on in Damp: powered, but limp.

If **Start Robot** is greyed out, its tooltip says why — most often the RPU is unreachable, or assembly was never confirmed (confirm it on the **Firmware** page, as in First Start). While an actuator check is running, the button reads **Waiting for actuator check…**.

Once the firmware is running, Overview's main action becomes **Drive Robot**. Before you drive, glance at Overview: a battery percentage, **Running** for the firmware, and **25 of 25 responding**. Overview explains every panel.

## Turn the Robot Off

1. If the robot is walking, bring it back down first — see Bring It Back Down.
2. Make sure it is supported.
3. On **Overview**, select **Stop Robot** and confirm.

This stops the firmware and nothing else. All 25 actuators go offline and driving stops; Humanoid Edge and the console stay up, so the robot is still reachable. **Stop Firmware** on the Firmware page does the same, and works whenever the firmware is running.

## Power Off

1. Turn the robot off, as above, with it seated or hanging from its gantry hook.
2. Press the button on the battery unit.

Humanoid Manager can't restart or shut down the robot's computers.

## Start on Boot

The **Start on boot** switch on the Firmware page starts the firmware automatically whenever the robot powers on. It is off by default, and available once assembly is confirmed.

Leave it off unless the robot lives somewhere it is always safely supported. With it on, the actuators power up after every power cut, with nobody there to check.



## Battery and Charging

The battery level shows in the sidebar's device card, the phone top bar, Overview and the Drive Robot header. It turns amber below 50% and red below 20%. The Drive Robot page's Telemetry tab has the detail: charge, voltage, current, maximum cell temperature and protection flags.

When it runs low, Overview and the Drive Robot page say **Battery low**: sit the robot down and charge it. If the battery runs out, the robot goes limp.
