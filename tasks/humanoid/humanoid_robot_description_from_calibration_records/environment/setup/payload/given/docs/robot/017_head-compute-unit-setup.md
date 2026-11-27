# Head Compute Unit Setup

> Assemble the Head Board base, Raspberry Pi 5, and Media Hat Board top into the Head Compute Unit.

- Section: Humanoid › Assemble the humanoid robot › Preparation › Compute Module Setup

The Head Compute Unit consists of the Head Board at the base, the Raspberry Pi 5 (part `700_09X`), and the Media Hat Board on top. The Media Hat Board provides the buzzer and helps prevent Pi 5 brownouts.

## Parts needed

| Part | Count |
| - | -: |
| `700_05C` | 1 |
| `700_08X` camera module | 1 |
| Head Board | 1 |
| Raspberry Pi 5 | 1 |
| RPI5 RTC Battery | 1 |
| Media Hat Board | 1 |
| `700_03C` | 1 |
| M2.5x6 Socket bolt | 8 |
| M2.5x20 hex standoffs (part `700_12X`) | 4 |
| M4x14 Socket bolt | 4 |
| USB-A to JST-ZHR4 cable (highlighted in yellow) | 1 |
| 1000 mm XT30-Male to XT30-Female power cable (highlighted in red) | 1 |
| 1000 mm Ethernet cable (highlighted in blue) | 1 |
| 100 mm USB-C to USB-A cable (highlighted in purple) | 1 |
| 200 mm JST-XH to JST-XH cable (highlighted in green) | 1 |
| 300 mm single-ended JST-XH cable (highlighted in orange) | 1 |

## Instructions

1. Take part `700_08X`, which is the camera module, and fit it into the square holes on part `700_05C`. Make sure that the camera module has its port at the bottom, as shown in the visualization below.

2. Take **4 of** M2.5x6 Socket bolts and tighten them into the 4 holes to secure the camera module to part `700_05C`.

3. Take the Head Board and cut all the bottom pins using the flush cutters, as shown in the video below.

   After the bottom pins of the Head Board are trimmed flush, put it on the platform of part `700_05C`. Make sure that the 4 mounting holes align with each other.

   Video: Trim the Head Board pins

4. Take **4 of** M2.5x20 hex standoffs and tighten them into the 4 threaded holes on top of the Head Board to secure it to part `700_05C`.

5. Take the Pi 5 and align its 4 mounting corners with the 4 M2.5x20 hex standoffs.

6. Take **4 of** M2.5x6 Socket bolts to secure the Pi 5 to the 4 M2.5x20 hex standoffs.

   > [!WARNING]
   >
   > **Remember to take a picture of the QR codes of the Pi 5 box**
   >
   > Remember to take a picture of the QR code on the box of the Pi 5. It contains the password to the Pi 5 Access Point which allow you to ssh into it remotely. It is unique to each Pi so don't lose it until the set up is complete.
   >
   > 

7. Take the Media Hat Board and install it onto the Pi 5 pins according to the visualization below.

8. Make the 1000 mm XT30-Male to XT30-Female power cable (highlighted in red). Take 1 XT30-Male connector, 1 XT30-Female connector, and 2 wires of 1000 mm, then solder them together as shown in the video below.

   > [!WARNING]
   >
   > **Match + to + and − to −**
   >
   > This cable carries power to the Head Board. Reversing the polarity can permanently damage the board.
   >
   > - Each XT30 connector has a **+** and a **−** marking on its housing. Check them before soldering.
   > - One wire must join **+ to +** and the other must join **− to −**. Never connect + on one connector to − on the other.
   > - If your wires are red and black, use **red for +** and **black for −** on both connectors.
   > - Slide heat-shrink tubing onto each wire before soldering, then shrink it over each joint so no bare metal is exposed.
   > - When both ends are done, check that each wire runs between matching markings. If you have a multimeter, use continuity mode to confirm + to + and − to −, and that + and − are not shorted together.

   Video: How to build a custom XT30 cable - Josh Pieper

9. Take part `700_03C` and put it on a flat surface with the inside of the part facing toward the ceiling.

   Take the provided USB-A to JST-ZHR4 cable (highlighted in yellow) and plug the JST connector end into the camera module `700_08X`. Then, route the cable through part `700_03C` until the USB-A connector is accessible through the back of the head.

   Plug the male connector of the 1000 mm XT30-Male to XT30-Female power cable (highlighted in red) from Instruction 8 into the Head Board as shown in the visualization below. Then, route the cable through part `700_03C` as above, but through the big gap at the back of the head instead.

   Then, align the 4 mounting holes on part `700_05C` with those on part `700_03C` according to the visualization below.

10. Take **4 of** M4x14 Socket bolts and tighten them into the 4 threaded holes on part `700_05C` to secure it to part `700_03C`.

11. Take the USB connector end of the USB-A to JST-ZHR4 cable (highlighted in yellow) and plug it into the Pi 5 USB port as shown in the visualization below.

12. Take the 1000 mm Ethernet cable (highlighted in blue) and plug one end into the Pi 5 Ethernet port as shown. Then, route the cable through the gap at the back of the head.

13. Take the 100 mm USB-C to USB-A cable (highlighted in purple) and connect the Head Board USB-C port to the Pi 5 USB port as shown in the visualization below.

14. Take the 200 mm JST-XH to JST-XH cable (highlighted in green) and connect the Media Hat Board JST port to the Head Board JST port as shown in the visualization below.

    Take the RPI5 RTC Battery (a coin battery) and plug it into the RTC battery port on the Pi 5. It keeps the Pi 5 clock running while the robot is powered off.

    > [!NOTE]
    >
    > **Check the photos for the coin battery position**
    >
    > Open the **Pictures** tab below to see exactly where its connector plugs into the Pi 5: the small port marked `BAT`, beside the USB-C power port.

    

    

15. Now, we need to make the speaker wire. Take the given JST-XH cable and cut it in half. Then extend the two `LSP+` and `LSP-` pins to a length of 30 cm using AWG 26 wire (which is given in the cable bag). To know which one is the `LSP+` and `LSP-` you can actually use the label on the Head Board speaker port below to figure it out. See the image and video below for more information.

    > [!NOTE]
    >
    > **Head Board can support up to 2 speakers**
    >
    > The Head Board can actually support up to 2 speakers, so the `LSP` is for the Left Speaker and the `RSP` is for the Right Speaker. You can remake the JST-XH cable to support more speakers in the future if you want to.

    Once the cable is made, plug it into the Head Board JST port as shown below. At the bare cable end, we will connect it to a WAGO, which will be connected to the speaker cable later on.

    

    Video: Making the speaker wire

**Previous:** Battery Setup · **Next:** RPU Setup
