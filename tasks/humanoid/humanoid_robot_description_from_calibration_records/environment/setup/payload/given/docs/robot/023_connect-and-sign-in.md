# Connect and Sign In

> Connect the Head Compute Unit and the RPU, power them on, sign in to Humanoid Manager and check the link between them.

- Section: Humanoid › Assemble the humanoid robot › Preparation › Firmware & Software Flashing

Humanoid Manager is the console the robot serves from its Raspberry Pi 5 (part `700_09X`). No terminal is needed to connect and sign in.

## Instructions

1. ### Connect and Power On

   1. Connect the 1000 mm Ethernet cable directly between the Pi 5 in the Head Compute Unit and the RPU. This link is private to the two computers; it doesn't go to your router.
   2. Power on both assemblies by pressing the switch on the battery module. **4 lights** should turn on: 3 on the Head Compute Unit and 1 on the RPU, numbered in the photo below.
   3. Wait about two minutes for them to boot.

   

   If any of the 4 lights stays off, check the power cables at both ends and that the battery switch is on.

2. ### Sign In

   1. Put your laptop on the Wi-Fi network you entered when flashing the Pi 5.
   2. Open `http://<name>.local`, where `<name>` is the hostname you set in Raspberry Pi Imager.
   3. Select **Sign in** — the default login is pre-filled. You can set your own password later.

   #### If `<name>.local` Doesn't Open

   Addresses ending in `.local` are found by local network discovery. If your computer blocks it, the page will not load even when the robot is on your network. Check the setting for your computer:

   **macOS:**

   macOS asks each app for permission to find devices on your local network. If you dismissed that prompt, allow it again:

   1. Open **System Settings** → **Privacy & Security** → **Local Network**.
   2. Turn on the switch for your browser (Safari, Chrome, Firefox, …).
   3. Quit and reopen the browser, then open `http://<name>.local` again.

   **Windows:**

   Windows only discovers local devices on networks marked as private:

   1. Open **Settings** → **Network & internet** → **Wi-Fi**, then select the network you are connected to.
   2. Set **Network profile type** to **Private network**.
   3. Open `http://<name>.local` again.

   **Linux:**

   Linux needs the Avahi service for `.local` addresses. Most desktop distributions include it; if not, install it. On Ubuntu and Debian:

   ```bash
   sudo apt install avahi-daemon libnss-mdns
   ```

   Then open `http://<name>.local` again.

   #### If the Robot Didn't Join Your Wi-Fi

   The robot starts its own setup hotspot instead.

   1. Join the network named `humanoid-XXXX`, where `XXXX` is the last four characters of the Pi 5's Wi-Fi MAC address (it tells robots apart if you have several). The password is the 16-digit code from the Pi 5's retail box — see Firmware & Software Flashing.
   2. Open `http://<name>.local`, or `` if that doesn't resolve.
   3. Select **Sign in** — the default login is pre-filled.

   > [!NOTE]
   >
   > **Use a laptop on the hotspot**
   >
   > The hotspot has no internet, so phones often drop it. If you use a phone, turn off mobile data.

   You can assign actuator IDs on the hotspot. To move the robot onto your Wi-Fi later, see Join Your Wi-Fi. If the console doesn't load, see The Console Will Not Load.

3. ### Check the RPU Link

   Humanoid Manager reports the status of the RPU from the controller running on its Radxa CM5 (part `100_15X`).

   Open **Firmware** in the sidebar. Both checks on the RPU card must be green before you continue.

   

   | If this is red | Do this |
   | - | - |
   | **Network** | Check the 1000 mm Ethernet cable at both ends, and that both assemblies have power |
   | **Control** | The RPU is still booting. Wait a minute, then reload the page |

   If a check stays red, see The RPU Is Unreachable.

   > [!NOTE]
   >
   > **Leave the assembly prompt for now**
   >
   > The Firmware page asks **Is the robot fully assembled?**, and **Start Firmware** is greyed out. Both are expected until the robot is built.

4. ### Set a Password

   Anyone on the robot's network can sign in with the default login. Set a password now.

**Previous:** Flash the Raspberry Pi 5 · **Next:** Enable the Battery BMS Switch
