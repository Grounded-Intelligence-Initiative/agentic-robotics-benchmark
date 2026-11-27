# Step 3: Install the Neck Yaw Actuator

> Attach neck yaw actuator ID 24 to part 100_03A, connect the 200 mm and 600 mm XT30 (2+2) cable connectors to its ports, and secure it in the torso neck.

- Section: Humanoid › Assemble the humanoid robot › Assemble › Torso Module › Torso Assembly

## Parts needed

| Part | Count |
| - | -: |
| Torso assembly from Step 2 | 1 |
| RJ-5660 actuator (part `100_09X`) (ID 24) | 1 |
| `100_03A` | 1 |
| M3x12 Flat bolt | 4 |
| M4x12 Flat bolt | 6 |
| 200 mm double-ended XT30 (2+2) cable | 1 |
| 600 mm double-ended XT30 (2+2) cable | 1 |
| 1000 mm Ethernet cable | 1 |
| 1000 mm XT30-Male to XT30-Female power cable | 1 |
| 300 mm single-ended JST-XH cable | 1 |
| Threadlocker | As needed |

## Instructions

1. Take RJ-5660 actuator (ID 24) and lay it flat on a surface such that the actuator shaft is facing toward the ceiling, the port is at the bottom.

2. Take part `100_03A` and align the 4 inner holes with the threaded holes on the actuator. Confirm that all four cables remain routed through the port cutout as shown below.

   > [!WARNING]
   >
   > **Route all four cables before aligning part 100_03A**
   >
   > Before aligning part `100_03A` with the actuator, route these four cables through the part's port cutout and into the torso:
   >
   > - the 1000 mm Ethernet cable from the Raspberry Pi 5 (part `700_09X`) (in blue);
   > - the 600 mm double-ended XT30 (2+2) cable for the neck pitch connection (in green);
   > - the 1000 mm XT30-Male to XT30-Female power cable from the Head Board (in red); and
   > - the 300 mm single-ended JST-XH cable for the speaker connection (in purple).
   >
   > Do this before aligning or installing the actuator. After part `100_03A` and the actuator are installed, the connectors will not fit back through the port cutout. Leave enough slack for each cable's later connection, and do not pull on the connectors.

3. Take **4 of** M3x12 Flat bolts and tighten them lightly into the 4 holes on part `100_03A` to align it with the actuator.

4. After the parts are aligned:

   > [!NOTE]
   >
   > **Not ready to apply threadlocker?**
   >
   > You can assemble without threadlocker while you check the fit, alignment, and orientation. When you are confident, disassemble as needed, apply threadlocker to the bolts, and reassemble them following these instructions.

   - remove **4 of** M3x12 Flat bolts one at a time in a star formation,
   - apply threadlocker,
   - then tighten them back to secure Part `100_03A` and the actuator together.

5. Connect one connector of the 200 mm double-ended XT30 (2+2) cable to the specified neck yaw actuator port, then route the cable through the cutout in part `100_03A`. Leave the free connector accessible for the neck pitch actuator.

   Connect one connector of the 600 mm double-ended XT30 (2+2) cable to the other neck yaw actuator port and leave the free connector accessible for the RPU connection.

6. Take the actuator and part `100_03A` and align it with the neck on part `100_01C`. The 600 mm double-ended XT30 (2+2) cable should be routed through the neck to the inner hollow of the body. The 200 mm double-ended XT30 (2+2) cable should be routed through part `100_03A` and face forward toward the chest of part `100_01C`. Then, carefully lower down the actuator into the neck hole of part `100_01C`.

7. Take **6 of** M4x12 Flat bolts and lightly tighten into each of the holes on part `100_03A` to align Part `100_03A` with the neck portion of part `100_01C`.

8. After the parts are aligned:

   > [!NOTE]
   >
   > **Not ready to apply threadlocker?**
   >
   > You can assemble without threadlocker while you check the fit, alignment, and orientation. When you are confident, disassemble as needed, apply threadlocker to the bolts, and reassemble them following these instructions.

   - remove **6 of** M4x12 Flat bolts one at a time in a star formation,
   - apply threadlocker,
   - then tighten them back to secure Part `100_03A` and part `100_01C`.
