# Step 9: Complete the Head Enclosure

> Route the head Ethernet and power cables to the RPU, connect the speaker cable, and install the three head covers.

- Section: Humanoid › Assemble the humanoid robot › Assemble › Torso Module › Head Assembly

## Parts needed

| Part | Count |
| - | -: |
| Integrated assembly from Step 8 | 1 |
| `700_04C` top head cover | 1 |
| `700_06C` left head cover | 1 |
| `700_07C` right head cover | 1 |
| M4x10 Flat bolt | 10 |
| 1000 mm XT30-Male to XT30-Female power cable (highlighted in red), connected to the Head Board in Head Compute Unit Setup | 1 |
| 1000 mm Ethernet cable (highlighted in blue), connected to the Raspberry Pi 5 (part `700_09X`) in Head Compute Unit Setup | 1 |
| 600 mm double-ended XT30 (2+2) cable | 1 |
| WAGO connector for the speaker cable | As needed |

## Instructions

1. Identify the 1000 mm Ethernet cable (highlighted in blue) coming out of the back of the head.

2. Take the free connector of the 1000 mm Ethernet cable (highlighted in blue) which poke out from the neck and route it up through part `700_03C` to connect to the Pi 5.

   Then, take the other free connector in the torso and plug it to the specified Motion Control Board port in the RPU. This is exactly the same as the step that you have done previously to assign the actuator ID.

3. Take the free connector of the 1000 mm XT30-Male to XT30-Female power cable (highlighted in red) which poke out from the neck and route it up through part `700_03C` to connect to the Head Board.

   Then, take the other free connector and connect it to the specified port on the Power Distribution Board.

4. Connect the speaker cable from speaker `100_13X` to the bare end of the 300 mm single-ended JST-XH cable (highlighted in orange) from the Head Board using the WAGO connector.

   

   Video: How to use a WAGO connector (watch 0:26 to 1:30)

5. Take the 600 mm double-ended XT30 (2+2) cable (highlighted in yellow) and plug it into neck pitch RJ-5660 actuator (part `700_13X`) (ID 25) as shown.

   Then, connect the other end to the specified port on the PDB.

6. Take part `700_04C` and slide it on top of the Pi 5 and align with part `700_03C`.

7. Take part `700_06C` and fit it to the left side of the head.

8. Take **5 of** M4x10 Flat bolts and lightly tighten into the 5 holes on part `700_06C` to align with the head. Once aligned, tighten all **5 bolts** to secure the part together.

9. Take part `700_07C` and fit it to the right side of the head.

10. Take **5 of** M4x10 Flat bolts and lightly tighten into the 5 holes on part `700_07C` to align with the head. Once aligned, tighten all **5 bolts** to secure the part together.
