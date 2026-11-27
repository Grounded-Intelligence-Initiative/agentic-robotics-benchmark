# Connect & Sign In

> Reach Humanoid Manager in a browser, put the robot on your Wi-Fi and replace the default password.

- Section: Humanoid › Operate the humanoid robot

Humanoid Manager is the console the robot serves from its Raspberry Pi 5. This page gets you into it, onto your Wi-Fi and off the default password. If you already used Humanoid Manager during assembly, check you can still reach it, set a password if you haven't, and move on to Calibrate Actuators.

## Find the Robot

Power the robot on and wait about two minutes. The Pi 5 then either joins a saved Wi-Fi network — such as the one you entered in Raspberry Pi Imager when flashing it — or starts its own setup hotspot.

**If the robot joined your Wi-Fi**, open this address from a device on the same network:

```
http://<name>.local
```

`<name>` is the hostname you set when flashing the Pi 5.

**If it didn't**, join the setup hotspot from your laptop or phone:

- **Network name:** `humanoid-XXXX`, where `XXXX` is the last four characters of the Pi 5 (part `700_09X`)'s Wi-Fi MAC address.
- **Password:** the 16-digit code from the Pi 5's retail box — see Firmware & Software Flashing.

Then open `http://<name>.local`. If `.local` doesn't resolve — common on Android — use the hotspot's address, usually ``. When the robot has a saved network it can't reach, the hotspot appears about 90 seconds later than usual.

## Sign In

The default login is pre-filled until you set a password, so signing in is one click. Humanoid Manager opens on **Overview**, with the **Finish setting up** checklist in the sidebar.

## Join Your Wi-Fi

Skip this if the robot is already on your network.

> [!NOTE]
>
> **The robot has a single radio**
>
> While the setup hotspot is on, the robot can't scan for networks, so there is no list to pick from — you type your network's name. When the robot starts joining, the hotspot and this page drop. That is expected.

1. Open **Wi-Fi** in the sidebar and select **Join Other Network…**.
2. Enter your network's name exactly as broadcast, and its password. Leave the password blank for an open network.
3. Note the address in the dialog — `http://<name>.local` — or select **Copy address**. Then select **Join**.
4. Move your device back to your own Wi-Fi and open that address.

If the robot can't join, the setup hotspot comes back on its own; join it and try again. If the console never loads on your network, see The Console Will Not Load.



### Changing Networks Later

Once the robot is on a network, the Wi-Fi page lists nearby networks with their signal strength. Select one to join it; **Rescan** refreshes the list. Saved networks are under **Saved networks**, each with a **Forget** button. Forgetting the last one turns the setup hotspot back on.

## Set a Password

The default login is public knowledge, and anyone on your network can reach the console.

1. Open the user menu at the bottom of the sidebar and select **Settings**.
2. Enter the current password, then your new password twice.
3. Select **Save Changes**.

This clears the **Set a password** row and the default-password warning. There is no strength rule, so choose a strong password yourself. If you forget it, see Forgotten Password.

## Check the RPU Link

The checklist's first row, **Reach the RPU**, should now be done: Humanoid Manager can control the Robot Processing Unit (RPU), whose Radxa CM5 runs the motion-control firmware. If the row isn't done, open **Firmware** and read the **RPU** card's two checks:

- **Network** failing: check the Ethernet cable between the Pi 5 and the Motion Control Board in the RPU, and that both the Head Compute Unit and RPU have power. Power down before reseating connections.
- **Control** failing: the RPU is still booting or its services are down. Wait a minute, then reload.

The RPU Is Unreachable covers the rest.

**Next:** Calibrate Actuators.
