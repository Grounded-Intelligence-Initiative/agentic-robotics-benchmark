# Flash the Raspberry Pi 5

> Install Raspberry Pi Imager, write the humanoid robot image, and configure the robot's name, account, and WiFi.

- Section: Humanoid › Assemble the humanoid robot › Preparation › Firmware & Software Flashing

## Instructions

1. ### Install Raspberry Pi Imager

   Download Raspberry Pi Imager from the official Raspberry Pi site:

   > [!WARNING]
   >
   > **Make sure to use version < 2.0**
   >
   > Make sure you use the right Raspberry Pi Imager below. For additional info why (optional):
   >
   > Issue · raspberrypi/rpi-imager

   Releases: raspberrypi/rpi-imager

2. ### Open Raspberry Pi Imager

   Insert the microSD card into your laptop, then open Raspberry Pi Imager. If it asks to update, click **No**.

   

3. ### Choose the Humanoid Image

   Under **Choose OS**, scroll to the bottom and pick **Use custom**, then select `humanoid-rpi-<version>.img.xz`.

   

   

4. ### Choose the microSD Card

   Under **Choose Storage**, select your microSD card, then click **Next**.

   

5. ### Edit the OS Customisation Settings

   > [!NOTE]
   >
   > **Apply OS customisation**
   >
   > When Imager offers to apply OS customisation, choose “Edit settings” and apply them. This is where your robot gets its name, your account, and your WiFi — it boots straight onto your network, already yours. (If you skip this, nothing is lost: the robot falls back to broadcasting its own setup network.)

   

   Recommended settings, field by field:

   | Setting | What to enter |
   | - | - |
   | Set hostname | Your robot's name — lowercase letters, digits and hyphens (e.g. `humanoid1`). This becomes the Humanoid Manager address, `http://<name>.local`. Give each robot a unique name if you own several. |
   | Set username and password | Tick it. Any username except the reserved `humanoid`, `humanoid-api` and `humanoid-pair`; pick a strong password — it is your login and sudo password. The image ships with no default account: skip this and there is no way to log into the OS (Humanoid Manager still works). A reserved username is refused at first boot — the robot then behaves like an uncustomised flash. |
   | Configure wireless LAN | Tick it. Enter the WiFi the robot will live on — SSID exactly as broadcast (case-sensitive) and its password. Tick Hidden SSID only if your network is hidden. Set “Wireless LAN country” to the country the robot is in — the dialog often remembers GB from a previous run; a wrong country limits which 5 GHz channels the robot may use. |
   | Set locale settings | Tick it and choose your timezone and keyboard layout (log timestamps and console input). |
   | Services → Enable SSH | Tick it. “Password authentication” uses the account password from above. Choose “public-key only” instead if you use SSH keys — paste your public key there. |

   > [!NOTE]
   >
   > **WiFi compatibility**
   >
   > A 2.4 GHz network is the most reliable choice. The Pi 5 (part `700_09X`)'s radio cannot see 5 GHz channels 149–165 at all — if your router is pinned there and the robot never joins, move the router to channels 36–48 or enable a 2.4 GHz SSID.

   

6. ### Enable SSH

   Change the tab over to **Services** to set up SSH as shown below. Once done, click **Save** to continue.

   

7. ### Write the Image

   Click **Yes** to apply the OS customisation settings:

   

   Then click **Yes** to confirm that the microSD card will be erased and start writing:

   

8. ### Insert the microSD Card

   When writing finishes, insert the microSD card into the slot on the underside of the Raspberry Pi 5, as shown by the arrow below. Push it in until it is fully seated.

   

**Previous:** Flash the Radxa CM5 · **Next:** Connect and Sign In
