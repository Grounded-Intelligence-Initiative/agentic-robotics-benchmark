# Step 8: Install the Neck Pitch Actuator and Head Base

> Attach bracket 700_02B to neck yaw actuator ID 24, install neck pitch actuator ID 25, and secure the prepared head assembly through head base 700_03C.

- Section: Humanoid › Assemble the humanoid robot › Assemble › Torso Module › Head Assembly

## Parts needed

| Part | Count |
| - | -: |
| Integrated assembly from Step 7 | 1 |
| `700_02B` | 1 |
| `700_01B` | 1 |
| Head assembly from Head Compute Unit Setup (`700_03C` head base with `700_05C` head carrier) | 1 |
| RJ-5660 actuator (part `700_13X`) (ID 25) | 1 |
| 200 mm double-ended XT30 (2+2) cable, connected to neck yaw RJ-5660 actuator (part `100_09X`) (ID 24) | 1 |
| M3x12 Flat bolt | 5 |
| M3x12 Socket bolt | 2 |
| M3x20 Socket bolt | 6 |
| Threadlocker | As needed |

## Instructions

1. Position Torso Assembly in a straight manner such that we can see its neck, as now we will be working on installing the head module.

2. Take part `700_02B` and align the 5 holes on it with the neck actuator shaft.

3. Take **5 of** M3x12 Flat bolts and lightly tighten them into the 5 holes on part `700_02B`

4. After the parts are aligned:

   > [!NOTE]
   >
   > **Not ready to apply threadlocker?**
   >
   > You can assemble without threadlocker while you check the fit, alignment, and orientation. When you are confident, disassemble as needed, apply threadlocker to the bolts, and reassemble them following these instructions.

   - remove **5 of** M3x12 Flat bolts one at a time in a star formation,
   - apply threadlocker,
   - then tighten them back to secure Part `700_02B` and the actuator together.

5. Take RJ-5660 actuator (ID 25) and slot it through the arch of part `700_02B`. Make sure that the two ports are perfectly parallel to part `100_03A`.

6. Connect the free connector of the 200 mm double-ended XT30 (2+2) cable from neck yaw RJ-5660 actuator (ID 24) to the specified port on neck pitch actuator (ID 25).

7. Take part `700_01B` and align the 2 holes with part `700_02B`.

8. > [!NOTE]
   >
   > **Not ready to apply threadlocker?**
   >
   > You can assemble without threadlocker while you check the fit, alignment, and orientation. When you are confident, disassemble as needed, apply threadlocker to the bolts, and reassemble them following these instructions.

   Take **2 of** M3x12 Socket bolts, apply threadlocker to each of them and tighten them into the 2 holes on part `700_01B` to secure it to part `700_02B`

9. Take the head assembly from Head Compute Unit Setup and align part `700_03C` with the actuator (ID 25) such that the 6 holes on the side of the part is aligned with the 6 holes on the actuator shaft. While doing this you can also routed all the wire that is exposed from the back of the head to route it through the port cutout on part `100_03A`.

10. Take **6 of** M3x20 Socket bolts and lightly tighten into the 6 holes on part `700_03C` to align it with the actuator.

11. After the parts are aligned:

    > [!NOTE]
    >
    > **Not ready to apply threadlocker?**
    >
    > You can assemble without threadlocker while you check the fit, alignment, and orientation. When you are confident, disassemble as needed, apply threadlocker to the bolts, and reassemble them following these instructions.

    - remove **6 of** M3x20 Socket bolts one at a time in a star formation,
    - apply threadlocker,
    - then tighten them back to secure Part `700_03C` and the actuator together.

12. Check that the head cables connected in Head Compute Unit Setup are still seated: the USB-A to JST-ZHR4 cable (highlighted in yellow) between the camera module and the Pi 5 (part `700_09X`), the 100 mm USB-C to USB-A cable (highlighted in purple) between the Head Board and the Pi 5, and the 200 mm JST-XH to JST-XH cable (highlighted in green) between the Media Hat Board and the Head Board. The 1000 mm XT30-Male to XT30-Female power cable (highlighted in red) and the 300 mm single-ended JST-XH cable (highlighted in orange) from the Head Board stay unconnected at their free ends until Step 9.
