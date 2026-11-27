# Enable the Battery BMS Switch

> Turn on the BMS hardware switch.

- Section: Humanoid › Assemble the humanoid robot › Preparation › Firmware & Software Flashing

> [!NOTE]
>
> **Check whether you need this step**
>
> Press the battery's switch button. If it does not turn the Compute Module setup off and on, complete this step to enable the switch.
>
> **If the switch already turns the setup off and on, skip this step** and continue to Assign Actuator IDs. If you are not sure, contact us through our support channel.

## Before You Begin

- Complete Connect and Sign In. The Head Compute Unit and RPU must be connected by the 1000 mm Ethernet cable and powered on.
- Keep your laptop on the same Wi-Fi network as the robot, or on the robot's setup hotspot.
- Have the **hostname**, **username** and **password** you set in Raspberry Pi Imager ready, with SSH enabled. Without them you cannot log in to the Pi 5 (part `700_09X`); contact us through our support channel if you cannot access it.

## Open a Terminal

A **terminal** (also called a **TTY session**) is a window where you type commands and press **Enter** to run them. This step uses **two terminal windows** at the same time: **Terminal 1** watches the battery's messages, and **Terminal 2** sends commands.

**Windows:**

Open the **Start** menu, type `Terminal` (or `PowerShell` on older Windows 10), and open it. Windows 10 and 11 already include the `ssh` command.

To open a second terminal, press **Ctrl+Shift+T** for a new tab, or open **Terminal** again from the Start menu. Paste with **Ctrl+V** or a right-click.

**macOS:**

Press **Cmd+Space**, type `Terminal`, and press **Enter**.

To open a second terminal, press **Cmd+T** for a new tab or **Cmd+N** for a new window. Paste with **Cmd+V**.

**Linux:**

Press **Ctrl+Alt+T**, or open **Terminal** from your applications menu.

To open a second terminal, press **Ctrl+Shift+T** for a new tab or **Ctrl+Shift+N** for a new window. Paste with **Ctrl+Shift+V**.

Placing the two windows side by side makes it easier to watch Terminal 1 while you type in Terminal 2.

## Connect to the RPU

Your laptop cannot reach the RPU directly. You first log in to the Raspberry Pi 5 over **SSH** (a secure remote login), then log in to the RPU from the Pi 5. Do this in **each** terminal.

1. ### Log In to the Pi 5

   Replace `<your-pi-username>` and `<name>` with the username and hostname you set in Raspberry Pi Imager; for example, `ssh `:

   ```bash
   ssh <your-pi-username>@<name>.local
   ```

   - The first time you connect, you are asked `Are you sure you want to continue connecting (yes/no/[fingerprint])?`. Type `yes` and press **Enter**.
   - Enter your Pi 5 password when asked. **Nothing appears on screen while you type the password**; this is normal. Press **Enter** when done.
   - If `<name>.local` is not found and your laptop is on the robot's setup hotspot, use `ssh <your-pi-username>@10.42.0.1` instead.

   When you are logged in, the line where you type starts with `<your-pi-username>@<name>`.

2. ### Log In to the RPU

   From the Pi 5, run:

   ```bash
   ssh humanoid@10.0.0.2
   ```

   Type `yes` if asked to continue connecting, then enter the RPU password, `humanoid`. When you are logged in, the line where you type starts with `humanoid@`.

When a command below starts with `sudo`, you may be asked for a password again. Enter the RPU password, `humanoid`.

## Steps

1. ### Open Terminal 1

   Open a TTY session on the RPU (**Terminal 1**), as described in Connect to the RPU. This session will be used for observing data on the CAN bus.

2. ### Start Watching the CAN Bus

   In Terminal 1, run:

   ```bash
   sudo candump can_bms
   ```

   This keeps running and prints each message on the CAN bus, so you can no longer type commands in this terminal. Leave it open.

3. ### Open Terminal 2

   Open another TTY session on the RPU (**Terminal 2**), the same way. This session will be used for sending commands to the CAN bus.

4. ### Read the BMS Register

   In Terminal 2, run the following command, then look in Terminal 1 and note down the data received from the BMS (the eight bytes after `[8]`):

   ```bash
   sudo cansend can_bms 137#5A
   ```

   Terminal 1 should show two lines like these:

   ```text
   can_bms  137   [1]  5A
   can_bms  137   [8]  00 02 00 06 00 07 00 0D
   ```

   The first line is the request you just sent. The second line is the BMS's reply: its register is the eight bytes after `[8]`, in this example `00 02 00 06 00 07 00 0D`. Your values may differ; note down your own.

5. ### Increment Byte 3

   The register bytes are **hexadecimal** (base 16: digits `0`–`9`, then `A`–`F`). Increment byte 3 (zero-indexed) of the BMS register by 1 in hexadecimal, leaving the other seven bytes unchanged. For example, `00 02 00 06 00 07 00 0D` becomes `00 02 00 07 00 07 00 0D`:

   BMS register (8 bytes); byte 3 is the one that changes:

   | Byte | 0 | 1 | 2 | **3** | 4 | 5 | 6 | 7 |
   | - | - | - | - | - | - | - | - | - |
   | Before | 00 | 02 | 00 | **06** | 00 | 07 | 00 | 0D |
   | After | 00 | 02 | 00 | **07** | 00 | 07 | 00 | 0D |

   Count in hexadecimal, not decimal: `09` becomes `0A` (not `10`), and `0F` becomes `10`.

   > [!WARNING]
   >
   > **If byte 3 reads FF**
   >
   > `FF` is the largest value a byte can hold, so it cannot be incremented. Do not continue, and do not enter factory mode. Contact us through our support channel.

   Write the incremented value down on paper or in a note, without spaces. You will type it into the command in Write the Updated Register. For the example above, you would write down:

   ```text
   000200070007000D
   ```

   > [!WARNING]
   >
   > **Check 0 and D carefully**
   >
   > Depending on the font, `0` (zero) and `D` can look alike, as can `0` and `O`. Read each character carefully and double-check what you wrote against Terminal 1 before continuing. Register values only contain the digits `0`–`9` and the letters `A`–`F`.

6. ### Enter Factory Mode

   In Terminal 2, run the following command to enter factory mode:

   ```bash
   sudo cansend can_bms 12E#5678
   ```

   Terminal 1 should show:

   ```text
   can_bms  12E   [2]  56 78
   can_bms  12E   [1]  5A
   ```

   The first line is your command. The second line, `5A`, is the BMS confirming that it is in factory mode.

7. ### Write the Updated Register

   In Terminal 2, run the following command with your incremented register bytes, written without spaces:

   > [!WARNING]
   >
   > **The RPU will turn off**
   >
   > If this command executes successfully, the RPU will turn off. Both terminals then stop responding or show a message such as `Connection to 10.0.0.2 closed`; this is expected.

   ```bash
   sudo cansend can_bms 137#<register-bytes-after-increment>
   ```

   For example:

   ```bash
   sudo cansend can_bms 137#000200070007000D
   ```

   Before the RPU turns off, Terminal 1 may show:

   ```text
   can_bms  137   [8]  00 02 00 07 00 07 00 0D
   can_bms  137   [1]  5A
   ```

   The first line is the register you wrote; check that byte 3 shows your incremented value (`07` in this example). The second line, `5A`, is the BMS confirming the write.

8. ### Turn the Battery Back On

   Turn the battery back on using the switch, and wait for the RPU to boot. This takes about two minutes.

9. ### Reconnect Both Terminals

   The RPU restarted, so both terminals lost their connection to it. Set them up again the same way as before.

   **In Terminal 1:**

   1. If the line where you type starts with `<your-pi-username>@<name>`, you are still logged in to the Pi 5; go to the next step. Otherwise, log in to the Pi 5 again as in Log In to the Pi 5:

      ```bash
      ssh <your-pi-username>@<name>.local
      ```

   2. Log in to the RPU as in Log In to the RPU, using the password `humanoid`:

      ```bash
      ssh humanoid@10.0.0.2
      ```

   3. Start watching the CAN bus again, as in Start Watching the CAN Bus:

      ```bash
      sudo candump can_bms
      ```

   **In Terminal 2:**

   Repeat steps 1 and 2 above to log in to the Pi 5 and then the RPU. Do not run `candump` here; Terminal 2 is for sending the next command.

   When Terminal 1 is running `candump` and the line where you type in Terminal 2 starts with `humanoid@`, continue to the next step.

10. ### Exit Factory Mode

    In Terminal 2, run the following command to exit factory mode:

    ```bash
    sudo cansend can_bms 12F#2828
    ```

    Terminal 1 should show:

    ```text
    can_bms  12F   [2]  28 28
    can_bms  12F   [1]  5A
    ```

    The first line is your command. The second line, `5A`, is the BMS confirming that it has left factory mode.

11. ### Close the Terminals

    Press **Ctrl+C** in Terminal 1 to stop `candump`. Type `exit` and press **Enter** to log out; type it twice to leave both the RPU and the Pi 5, then close the terminals.

**Previous:** Connect and Sign In · **Next:** Assign Actuator IDs
