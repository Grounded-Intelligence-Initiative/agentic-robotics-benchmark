# Firmware & Software Flashing

> Get the software images, flash the Radxa CM5 and Raspberry Pi 5, then connect the Head Compute Unit and the RPU and sign in to Humanoid Manager.

- Section: Humanoid › Assemble the humanoid robot › Preparation

This flow covers downloading the images, flashing both boards, connecting them and signing in to Humanoid Manager.

> [!NOTE]
>
> **Check the battery switch**
>
> After **Connect and Sign In**, press the battery's switch button. If it does not turn the Compute Module setup off and on, you must enable the battery BMS switch.
>
> If the switch already turns the setup off and on, skip that step and continue to actuator ID assignment.

## Before you begin

> [!WARNING]
>
> **Keep the Raspberry Pi 5 box**
>
> IMPORTANT! Keep the Pi 5 (part `700_09X`)'s retail box or scan the QR and record down the 16-digit code. The 16-digit code is your robot's fallback AP WiFi password — it is unique to your board.
>
> Note: the QR code can also be found on the back of the Pi 5 just in case you have lost the box.
>
> 

### You will also need

- The Compute Module Setup from the previous step
- A PC with a USB-C port and WiFi
- A USB-C data cable (charge-only cables will not work)
- A microSD card, 32 GB or larger (and adapter to the PC to prepare for flashing of the microSD card)

You will install two free tools along the way — one for each board. Both are covered in the steps below, with instructions for Windows, macOS, and Linux.

## Get the Images

1. ### Request the Images

   > [!NOTE]
   >
   > **Request the latest board images**
   >
   > The humanoid robot's software is still a work in progress, so we share the board images directly, along with instructions for using them safely. Contact us and we'll provide a link to the latest matched images for the Radxa CM5 and Raspberry Pi 5.
   >
   > **Existing customers can reach us through their usual support channel.**
   >
   > Everyone else, fill in this form and we'll be in touch.

   | Image | Download | Asset |
   | - | - | - |
   | Radxa CM5 (part `100_15X`) | Contact us for the download link | Latest CM5 image |
   | CM5 SPL bootloader | Radxa | `rk3588_spl_loader_v1.15.113.bin` |
   | Raspberry Pi 5 | Contact us for the download link | Latest Pi 5 image |
   | Checksums (optional) | Contact us | `SHA256SUMS` |

   The SPL bootloader is needed when you flash the Radxa CM5. The Pi 5 image is loaded into Raspberry Pi Imager when you flash the Raspberry Pi 5. The image filenames retain the `humanoid-rpu` and `humanoid-rpi` software names.

   > [!WARNING]
   >
   > **Use images from the same release round**
   >
   > Use images from the same release round for both boards. The two boards run matched software; mixing an old image with a new one can leave joints uncontrolled. Each release notes which CM5 and Pi 5 image versions belong together.

2. ### Check Your Downloads (Optional)

   Put `SHA256SUMS` in the same folder as the downloaded files, then run:

   ```bash
   # Linux
   sha256sum -c SHA256SUMS --ignore-missing

   # macOS
   shasum -a 256 -c SHA256SUMS --ignore-missing
   ```

   Each downloaded file should report `OK`. On Windows, run `Get-FileHash <file>` in PowerShell and compare the result with the matching line in `SHA256SUMS`.

**Previous:** Integration Setup · **Next:** Flash the Radxa CM5
