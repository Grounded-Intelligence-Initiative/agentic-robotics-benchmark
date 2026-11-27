# RPU Setup

> Assemble the Motion Control Board, Radxa CM5, and Power Distribution Board into the Robot Processing Unit (RPU).

- Section: Humanoid › Assemble the humanoid robot › Preparation › Compute Module Setup

The Robot Processing Unit (RPU) consists of the Motion Control Board (MCB), with the Radxa CM5 (part `100_15X`) mounted on it, and the Power Distribution Board (PDB) on top.

## Parts needed

| Part | Count |
| - | -: |
| `100_16C` fan mount | 1 |
| Fan | 1 |
| Motion Control Board | 1 |
| Radxa CM5 | 1 |
| Power Distribution Board | 1 |
| M3x8 Socket bolt | 2 |
| M3x18 Socket bolt | 4 |
| 4-pin JST-XH cable | 1 |
| WAGO connectors or approved soldering and insulation supplies | As needed |

## Instructions

1. Align the 2 holes on part `100_16C` onto the U-shape area of the PDB.

2. Take **2 of** M3x8 Socket bolts to tighten into the PDB to secure the part `100_16C`

   

3. Take the fan and align the 4 holes on each corner to the holes on part `100_16C`. Make sure that the fan pushes air downwards through the U-shape hole on the PDB as indicated by the arrow on the fan (refer to the image below for a clearer look).

   

4. Take **4 of** M3x18 Socket bolts and secure the fan to part `100_16C`.

5. Grab the MCB with the Ethernet RJ45 port facing away from you.

   Take the CM5 out of the package and lightly place it into the corresponding pins of the MCB in the orientation shown below in the images and visualization.

   Evenly press down on the left, right and bottom of the MCB till the CM5 module slides into place. Apply pressure evenly to ensure all pins have an even contact with the MCB.

   It should make a "click" sound once it sit in place correctly. Watch the video below for more instruction.

   

   Video: Install the Radxa CM5 on the Motion Control Board

6. Align the PDB with the MCB via the 2.54 mm pins, pressing down gently till the connectors are in full contact. Refer to the visualization below to install the two boards together.

7. Prepare the power connection for the fan by connecting it to a 4-pin JST-XH cable using Wago as shown in the image below.

   Connect the fan wire to the 2 center wires of the cable (You should be able to see in the image what the white cable only has 2 middle wire instead of all 4 wires, the 2 wires on each end has been cut and not used). You could also solder the wire, if you prefer a cleaner solution.

   Plug the fan power connector into the port shown in the visualization and image below.

   

   

**Previous:** Head Compute Unit Setup · **Next:** Integration Setup
