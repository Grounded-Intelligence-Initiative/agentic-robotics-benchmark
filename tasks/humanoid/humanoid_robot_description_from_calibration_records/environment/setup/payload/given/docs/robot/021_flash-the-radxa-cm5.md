# Flash the Radxa CM5

> Install the flashing tool, enter MaskROM mode, and write the image to the Radxa CM5.

- Section: Humanoid › Assemble the humanoid robot › Preparation › Firmware & Software Flashing

The Radxa CM5 (part `100_15X`) has no SD card slot. You flash its onboard storage over USB-C through the RPU, with the CM5 in a special MaskROM mode.

## Instructions

1. ### Install the Flashing Tool

   The tool differs by operating system. Windows uses a graphical tool; macOS and Linux use a command-line one. Open the section for your computer.

   > [!NOTE]
   >
   > **macOS and Linux: run the commands from your home folder**
   >
   > Before installing, run `cd ~` so the terminal is in your home folder. Running the install from a protected folder such as `~/Documents`, `~/Desktop` or `~/Downloads` can fail with a permission error.

   **Linux:**

   On Ubuntu 22.10 or newer, and current Debian:

   ```bash
   sudo apt install rkdeveloptool
   ```

   On older distributions the package may not exist, in which case build it from source:

   ```bash
   sudo apt install -y libudev-dev libusb-1.0-0-dev dh-autoreconf pkg-config build-essential git
   git clone 
   cd rkdeveloptool && autoreconf -i && ./configure && make -j"$(nproc)"
   sudo cp rkdeveloptool /usr/local/sbin/
   ```

   Check it worked:

   ```bash
   rkdeveloptool -V
   ```

   No driver is needed. You will need `sudo` for the flashing commands.

   **macOS:**

   Install via Homebrew — no compiling required:

   ```bash
   brew tap IgorKha/rkdeveloptool
   brew install rkdeveloptool
   ```

   If you encounter an untrusted tap error:

   ```text
   Error: Refusing to load formula igorkha/rkdeveloptool/rkdeveloptool from untrusted tap igorkha/rkdeveloptool
   ```

   Fix it via the recovery instructions and run:

   ```bash
   brew trust igorkha/rkdeveloptool
   ```

   Check it worked:

   ```bash
   rkdeveloptool -V
   ```

   If the tap is unavailable, build from source instead:

   ```bash
   # We must use g++ on MacOS when building
   brew install automake autoconf libusb pkg-config gcc@15
   # Clone and build
   git clone 
   cd rkdeveloptool && autoreconf -i && CXX=/opt/homebrew/bin/g++-15 ./configure && make
   ```

   **Windows:**

   Windows uses RKDevTool, a graphical tool, plus a USB driver. Both come from Radxa:

   1. Download and extract DriverAssistant v5.14, then run `DriverInstall.exe` as administrator. Without this driver Windows will not recognise the board.
   2. Download and extract RKDevTool v2.96, then launch `RKDevTool.exe`.
   3. You will also need 7-Zip to decompress the `.img.xz` image into a plain `.img` before flashing.

   The commands in Write the Image below are for macOS and Linux. For the RKDevTool click-through, follow Radxa's RKDevTool guide — with the board in MaskROM mode it will show “Found One MASKROM Device”, and you write the loader and image from there.

   If you have access to a Mac or Linux machine, we recommend using it — that is the path we test against.

2. ### Put the Board in MaskROM Mode

   

   Video: Putting the RPU in MaskROM mode

   The order matters — the button must already be held down when the board first receives power. Connect power last.

   1. Plug the USB-C cable provided from your laptop into the USB-C port on the RPU shown above.
   2. Hold down the MaskROM button on the RPU.
   3. While still holding it, press the on button on the battery to power the RPU.
   4. Release the MaskROM button.
   5. Confirm the laptop sees it:

      ```bash
      rkdeveloptool ld
      ```

   You should see `Mode=Maskrom`. For example:

   ```text
   DevNo=1 Vid=0x2207,Pid=0x350b,LocationID=202    Maskrom
   ```

   If the list is empty, re-seat the cable and repeat — the button must be held as the board is first powered, and the cable must carry data.

3. ### Write the Image

   The bootloader file `rk3588_spl_loader.bin` is the `rk3588_spl_loader_v1.15.113.bin` you downloaded in Get the Images — rename it to match, or use its full name in the commands below. Writing takes a few minutes.

   **macOS and Linux:**

   ```bash
   rkdeveloptool db rk3588_spl_loader.bin
   # You should see: Downloading bootloader succeeded.

   xz -dk humanoid-rpu-<version>.img.xz

   rkdeveloptool wl 0 humanoid-rpu-<version>.img
   # The write process will begin. After a few minutes it will reach:
   # Write LBA from file (100%)

   rkdeveloptool rd
   # You should see: Reset Device OK.
   ```

   On Linux these need `sudo`. The `xz -dk` line decompresses the image and keeps the original; the write itself needs the plain `.img`.

   **Windows:**

   In RKDevTool, with the board showing “Found One MASKROM Device”, load `rk3588_spl_loader.bin` as the loader and the decompressed `humanoid-rpu-<version>.img` as the image, then start the write. Step-by-step screens are in Radxa's RKDevTool guide.

   Decompress the `.img.xz` with 7-Zip first — RKDevTool cannot read the compressed file.

4. ### Disconnect the RPU

   When it finishes, you should see an LED blinking blue on the RPU. You can now unplug the USB-C data cable and remove the battery power.

**Previous:** Get the Images · **Next:** Flash the Raspberry Pi 5
