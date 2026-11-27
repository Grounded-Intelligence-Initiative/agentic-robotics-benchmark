# Integration Setup

> Connect the Head Compute Unit to the Robot Processing Unit (RPU), and connect the battery.

- Section: Humanoid › Assemble the humanoid robot › Preparation › Compute Module Setup

Connect the Head Compute Unit and the Robot Processing Unit (RPU), then connect the battery to the RPU.

## Parts needed

| Part | Count |
| - | -: |
| Battery assembly from Battery Setup | 1 |
| RPU from RPU Setup | 1 |
| Head Compute Unit from Head Compute Unit Setup | 1 |

## Instructions

1. Place the battery assembly, the RPU and the Head Compute Unit on a flat surface.

2. Take the 1000 mm Ethernet cable (highlighted in blue) from the Head Compute Unit and plug it into the Ethernet port on the RPU as shown below.

3. Take the 1000 mm XT30-Male to XT30-Female power cable (highlighted in red) from the Head Compute Unit and plug it into one of the three distributed power ports on the RPU as shown below.

4. Take the battery power cable (highlighted in red) and plug it into the XT90-M (2+2) power port on the RPU as shown below.

5. The initial setup is complete. Now you are ready to flash the Raspberry Pi 5 (part `700_09X`) and the CM5 (part `100_15X`) with the appropriate software in the next step.

   > [!NOTE]
   >
   > **Test the system connection**
   >
   > You can turn the system on by pressing the button on the switch of the battery module. The lights on the head board and Radxa CM5 should turn on with a solid LED. You can now proceed to the image flashing process.

   > [!WARNING]
   >
   > **If the switch does not turn the setup on and off**
   >
   > Your battery's BMS switch needs to be enabled. Continue to Firmware & Software Flashing as normal: after flashing both boards and signing in, the Enable the Battery BMS Switch step turns the switch on.

**Previous:** RPU Setup · **Next:** Firmware & Software Flashing
