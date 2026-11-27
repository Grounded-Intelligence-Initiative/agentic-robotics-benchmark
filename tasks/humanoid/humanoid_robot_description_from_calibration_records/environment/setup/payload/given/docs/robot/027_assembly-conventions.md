# Assembly Conventions

> Read bolt sizes and counts, measure Socket and Flat bolts, and follow the threadlocker, tightening, and soldering guidance.

- Section: Humanoid › Assemble the humanoid robot › Assemble

## Bolt Names and Counts

The manual uses compact bolt names such as M3x8 Socket bolt and M4x12 Flat bolt.

- **M3** is the metric thread size: a nominal thread diameter of 3 mm.
- **x8** means an 8 mm bolt length. The starting point for this measurement depends on the head type, as explained below.
- **Socket** has a raised cylindrical head. **Flat** has a countersunk head with a tapered underside that fits into a matching recess.

For example, "Take **4 of** M3x8 Socket bolts" means select four bolts of that size and type. Use "of" for every count, including one: "Take **1 of** M3x20 Socket bolt." Parts tables use a singular bolt name and put the quantity in the **Count** column.

Orange, dotted-underlined bolt names open a 3D preview on click or tap. A bolt name without an underline has no verified preview yet. Use the preview to recognize the head shape, not to judge the bolt's length on screen.

## Measuring Bolt Length

Use a ruler or caliper and measure in millimetres. Start at the ruler's zero mark, not the edge of the ruler.

| Type | Measure from | Measure to | Include the head? |
| - | - | - | - |
| Socket | The flat underside of the head, where it meets the shaft | The end of the bolt | **No** |
| Flat | The flat top surface of the countersunk head | The end of the bolt | **Yes** |

### Socket Bolts

Measure from **under the head to the end of the bolt**. Include any unthreaded shaft, but exclude the head itself; do not measure only the threaded section. A M3x8 Socket bolt measures 8 mm under the head, so its overall length is longer than 8 mm.

### Flat Bolts

Measure the **entire bolt, including the head**, from its flat top surface to the end. A M4x12 Flat bolt measures 12 mm overall.

> [!WARNING]
>
> **Check the head type as well as the size**
>
> Socket and Flat bolts of the same thread size and listed length are not interchangeable. Check the head type and measure the length before installation. Do not force a bolt that does not fit, or substitute a different length to make it fit.

## Threadlocker

Cut the nozzle tip, squeeze a small pool of threadlocker into the bottle cap, then wipe the bolt threads through it rather than dripping straight from the bottle. That is the easiest way to keep the amount under control. The video below walks through the whole process.



Video: Applying threadlocker to a bolt

> [!WARNING]
>
> **Only apply threadlocker once you are certain the joint is correct**
>
> Threadlocker is very hard to undo. If you apply it to a bolt that turns out to be in the wrong hole, the wrong orientation, or the wrong part, you may not be able to back that bolt out again.
>
> You can assemble the whole robot without threadlocker while you check the fit, alignment, and orientation. Once you are confident everything is correct, disassemble as needed, apply threadlocker, and reassemble following each step's bolt-handling and tightening sequence.
>
> Without threadlocker, bolts can loosen and damage the parts. You can proceed to setup for brief testing, but do not operate the robot for more than **10 hours in total** without threadlocker. Stop testing immediately if any bolts loosen. Complete the threadlocker pass afterward, as described in Finish Assembly.

> [!WARNING]
>
> **Use less threadlocker on M3 screws**
>
> From M4 upwards, one drop is the right amount. M3 screws need slightly less than that, so do not let a full drop land on one.
>
> Excess threadlocker spreads past the threads and makes the screw much harder to back out later.

> [!NOTE]
>
> **Use heat to free a threadlocked bolt**
>
> If you have already applied threadlocker and need to remove a bolt, heat the bolt head with a soldering iron or similar for a few seconds. The heat softens the threadlocker and the bolt will turn out far more easily than forcing it cold.
>
> Keep the heat on the bolt head only, and keep it away from printed plastic parts, cables, and actuators.

## Over-tightening

> [!WARNING]
>
> **Never use an electric screwdriver on M3 screws or smaller**
>
> The M3 hex screws are small, and an electric screwdriver rounds out the head. Electric screwdrivers are prohibited on M3 and smaller — turn those by hand.
>
> More generally, avoid an electric screwdriver altogether unless you know the tool well and can control its torque. A hand tool is slower, but it is far harder to strip a head or a thread with one. If you are unsure, use hand tools. See Tools for the recommended set.

## Solder

Some steps join cable leads by soldering them together, and the head needs a power cable made from XT30 connectors and wire. Use the videos below as the reference for both techniques. See Tools for the soldering iron, solder, heat-shrink tubing, and fume extraction we use.

> [!WARNING]
>
> **Slide heat-shrink tubing on before you solder**
>
> Once a joint is soldered, you cannot fit tubing over it. Slide heat-shrink tubing onto the wire first, then shrink it over the finished joint so no bare metal is exposed.

### Soldering a Wire Joint

Where a step asks you to solder the leads routed from one actuator to the leads of another, join each corresponding conductor following the released pinout. Match each wire to its counterpart before you heat anything; a swapped or poorly made joint can break power or degrade the CAN signal between actuators.

Video: How to solder two wires together

After each joint cools, tug it gently to confirm it holds, then shrink the tubing over it.

### Making an XT30 Power Cable

The 1000 mm XT30-Male to XT30-Female power cable that powers the Head Board is made during Head Compute Unit Setup from 1 XT30-Male connector, 1 XT30-Female connector, and 2 wires of 1000 mm. Solder one wire to each connector terminal as shown in the video below.

Video: How to build a custom XT30 cable - Josh Pieper

> [!WARNING]
>
> **Match + to + and − to −**
>
> Reversing the polarity of a power cable can permanently damage the board it powers.
>
> - Each XT30 connector has a **+** and a **−** marking on its housing. Check them before soldering.
> - One wire must join **+ to +** and the other must join **− to −**. Never connect + on one connector to − on the other.
> - If your wires are red and black, use **red for +** and **black for −** on both connectors.
> - When both ends are done, check that each wire runs between matching markings. If you have a multimeter, use continuity mode to confirm + to + and − to −, and that + and − are not shorted together.

The actuator XT30 (2+2) cables are separate cables that carry both power and CAN signals. Do not make substitutes for them from XT30 power connectors.
