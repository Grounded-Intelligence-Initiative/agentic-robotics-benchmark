# HVB-P traction battery: workshop manual, removal of the modules

Issue 3. Applies to the plug-in hybrid packs HVB-P4, HVB-P5 and HVB-P6 (4, 5 and 6 modules in one common housing).
This manual describes the pack as built, what a pack returned from the field may look like, and the order in which it
is taken apart to module level. Dimensions are nominal unless a tolerance is given.

## 1. Safety: high voltage

The busbars, the modules' terminal posts and the junction studs are live as long as the service disconnect (MSD) is
in its socket: the modules are then connected in series (up to 320 V). **Nothing conductive may touch a busbar, a
busbar bolt, a terminal post or a junction stud before the pack is isolated.** In the disassembly cell an insulation
monitor trips when a tool touches (comes within 3 mm of) one of them before isolation: the cell stops for good and
the pack is condemned (section 9).

Isolation procedure:

1. Remove the lid (section 6, steps 1 and 2).
2. Pull the service disconnect straight up out of its socket and take it away.
3. Wait 30 s: the pack's link capacitor discharges through its bleed resistor.

After that the string is open and no part carries more than one module's voltage (below 60 V): busbars and their
bolts may be touched and removed with the tools. The lid's bolts, the lid, the service disconnect, the low-voltage
plugs, the brackets and their bolts and the modules' housings are never live.

**A loose fastener is a short circuit.** A bolt or a sheared head that falls onto a busbar still in its seat (or
comes within 3 mm of it) bridges it to its neighbour or to the tray, before isolation and after it (each module
keeps its own voltage on its terminals): the cell stops for good and the pack is condemned (section 9). A freed
fastener rides out on the magnetic bit only as section 5 says.

## 2. The series

All three variants use the same tray and the same lid outline; they differ in the number of modules, in where the
service disconnect and the battery management unit (BMS) sit, and in the lid:

| variant | modules | lid bolts | lid ribs (raised strips on the lid) | MSD at y = | BMS at y = |
|---|---|---|---|---|---|
| HVB-P4 | 4 | 10 | along y at x = -140 mm; along y at x = 140 mm | 330 mm | -250 mm |
| HVB-P5 | 5 | 12 | along x at y = -350 mm; along x at y = -150 mm; along x at y = 150 mm; along x at y = 350 mm | -330 mm | 220 mm |
| HVB-P6 | 6 | 14 | along y at x = -140 mm; along y at x = 140 mm; along x at y = -300 mm; along x at y = 300 mm | 0 mm | 270 mm |

The variant is not marked on the pack. The variants differ in their lid (rib pattern, number of lid bolts) and in
their number of modules.

Drawings: `manual/P4_lid.png`, `manual/P4_inside.png` (and P5, P6). Photographs of an example pack and of the
fastener heads: `manual/photo_*.jpg` (section 8).

## 3. The pack frame and the heights

The pack frame P has its origin on the fixture's surface under the centre of the tray; x and y run along the tray's
short and long sides, z is up. On the cell's fixture a pack is placed with its frame at the fixture's nominal
position, x and y parallel to the cell's x and y (`cell/cell_interface.md`), within the placement tolerance of
section 7. Every position in this manual is in the pack frame, in millimetres.

The tray is 620 x 1000 mm outside and 160 mm high; its walls are 20 mm thick, its floor 12 mm.

| height z (mm) | what |
|---|---|
| 160 | top of the tray's walls; underside of the lid |
| 164 | top of the lid; seat of the lid bolts' heads |
| 200 | centre of the lid's handle bar (where the gripper takes it) |
| 122 | top of a module's housing |
| 144 | centre of a module's handle bar (6 mm above it: the bar's top) |
| 132 | top of the terminal posts and junction studs; underside of the busbars |
| 134 | mid-height of a busbar (where the gripper takes it, on its orange sleeve) |
| 136 | top of a busbar; seat of the busbar bolts' heads |
| 40 | top of a bracket; seat of the bracket bolt's head |
| 29 | mid-height of a bracket (where the gripper takes it) |
| 135 | centre of the service disconnect's handle bar |
| 77 | mid-height of the low-voltage plugs J1 and J2 (where the gripper takes them) |

A bolt's head stands above its seat by the head height of section 5.

## 4. Components and their nominal places

Grip features (the gripper closes on them from above; the closing direction is the direction the fingers
move in, in the pack frame):

| component | grip feature | closing direction | width (mm) | mass (kg) |
|---|---|---|---|---|
| lid | handle bar at the lid's centre | y | 30 | 5 |
| module M1.. | handle bar on top, at the module's centre | y | 30 | 11.5 |
| service disconnect (MSD) | handle bar on top | y | 20 | 0.35 |
| plugs J1, J2 | the plug's body | x | 22 | 0.05 |
| busbar between two modules (runs along y) | the orange sleeve at its middle | x | 26 | 0.12 |
| first and last busbar (run along x) | the orange sleeve at its middle | y | 26 | 0.12 |
| bracket | the block itself | x (30 mm) or y (60 mm) | 30 / 60 | 0.15 |
| harness remnant (section 7) | the cable, anywhere along it | across the cable | 12 | 0.2 |

What holds what:

- the lid: all of its bolts;
- a busbar: its two bolts (`a` and `b`);
- a bracket: its one bolt;
- a module: its two brackets (`a` at the far end, `b` at the service-bay end) and the two busbars on its terminal
  posts; it comes free when all four have been taken off;
- the service disconnect: its latch; it comes out when pulled straight up with more than 60 N;
- a plug J1, J2: its latch; it comes out when pulled straight up with more than 30 N.

A component that is still held does not come away; pulling harder only overloads the robot (the cell stops it).

Appearance: the lid is dark grey with a yellow handle bar; the tray is bare aluminium; a module is grey with a
lighter top plate, a yellow handle bar and two copper terminal posts at its service-bay end; busbars are copper
with an orange sleeve; brackets are yellow-passivated steel blocks; the service disconnect is orange with a black
handle on an orange base; the BMS is a black box, J1 a grey and J2 a white plug with a black latch; the junction
blocks are red; fasteners are zinc-plated (bright) unless corroded.

### HVB-P4

| fastener | thread | x | y | seat z | holds |
|---|---|---|---|---|---|
| L01 | M6 | -300 | -360 | 164 | lid |
| L02 | M6 | -300 | -120 | 164 | lid |
| L03 | M6 | -300 | 120 | 164 | lid |
| L04 | M6 | -300 | 360 | 164 | lid |
| L05 | M6 | 300 | -360 | 164 | lid |
| L06 | M6 | 300 | -120 | 164 | lid |
| L07 | M6 | 300 | 120 | 164 | lid |
| L08 | M6 | 300 | 360 | 164 | lid |
| L09 | M6 | 0 | -490 | 164 | lid |
| L10 | M6 | 0 | 490 | 164 | lid |
| B00a | M6 | -145 | -260 | 136 | B00 |
| B00b | M6 | -45 | -260 | 136 | B00 |
| B01a | M6 | -45 | -190 | 136 | B01 |
| B01b | M6 | -45 | -110 | 136 | B01 |
| B02a | M6 | -45 | -40 | 136 | B02 |
| B02b | M6 | -45 | 40 | 136 | B02 |
| B03a | M6 | -45 | 110 | 136 | B03 |
| B03b | M6 | -45 | 190 | 136 | B03 |
| B04a | M6 | -45 | 260 | 136 | B04 |
| B04b | M6 | -145 | 260 | 136 | B04 |
| R1af | M8 | 258 | -225 | 40 | R1a |
| R1bf | M8 | -78 | -225 | 40 | R1b |
| R2af | M8 | 258 | -75 | 40 | R2a |
| R2bf | M8 | -78 | -75 | 40 | R2b |
| R3af | M8 | 258 | 75 | 40 | R3a |
| R3bf | M8 | -78 | 75 | 40 | R3b |
| R4af | M8 | 258 | 225 | 40 | R4a |
| R4bf | M8 | -78 | 225 | 40 | R4b |

| component | kind | grip point x | y | z |
|---|---|---|---|---|
| lid | lid | 0 | 0 | 200 |
| msd | msd | -200 | 330 | 135 |
| J1 | connector | -200 | -295 | 77 |
| J2 | connector | -200 | -205 | 77 |
| B00 | busbar | -95 | -260 | 134 |
| B01 | busbar | -45 | -150 | 134 |
| B02 | busbar | -45 | 0 | 134 |
| B03 | busbar | -45 | 150 | 134 |
| B04 | busbar | -95 | 260 | 134 |
| R1a | bracket | 258 | -225 | 29 |
| R1b | bracket | -78 | -225 | 29 |
| M1 | module | 90 | -225 | 144 |
| R2a | bracket | 258 | -75 | 29 |
| R2b | bracket | -78 | -75 | 29 |
| M2 | module | 90 | -75 | 144 |
| R3a | bracket | 258 | 75 | 29 |
| R3b | bracket | -78 | 75 | 29 |
| M3 | module | 90 | 75 | 144 |
| R4a | bracket | 258 | 225 | 29 |
| R4b | bracket | -78 | 225 | 29 |
| M4 | module | 90 | 225 | 144 |

### HVB-P5

| fastener | thread | x | y | seat z | holds |
|---|---|---|---|---|---|
| L01 | M6 | -300 | -400 | 164 | lid |
| L02 | M6 | -300 | -200 | 164 | lid |
| L03 | M6 | -300 | 0 | 164 | lid |
| L04 | M6 | -300 | 200 | 164 | lid |
| L05 | M6 | -300 | 400 | 164 | lid |
| L06 | M6 | 300 | -400 | 164 | lid |
| L07 | M6 | 300 | -200 | 164 | lid |
| L08 | M6 | 300 | 0 | 164 | lid |
| L09 | M6 | 300 | 200 | 164 | lid |
| L10 | M6 | 300 | 400 | 164 | lid |
| L11 | M6 | 0 | -490 | 164 | lid |
| L12 | M6 | 0 | 490 | 164 | lid |
| B00a | M6 | -145 | -335 | 136 | B00 |
| B00b | M6 | -45 | -335 | 136 | B00 |
| B01a | M6 | -45 | -265 | 136 | B01 |
| B01b | M6 | -45 | -185 | 136 | B01 |
| B02a | M6 | -45 | -115 | 136 | B02 |
| B02b | M6 | -45 | -35 | 136 | B02 |
| B03a | M6 | -45 | 35 | 136 | B03 |
| B03b | M6 | -45 | 115 | 136 | B03 |
| B04a | M6 | -45 | 185 | 136 | B04 |
| B04b | M6 | -45 | 265 | 136 | B04 |
| B05a | M6 | -45 | 335 | 136 | B05 |
| B05b | M6 | -145 | 335 | 136 | B05 |
| R1af | M8 | 258 | -300 | 40 | R1a |
| R1bf | M8 | -78 | -300 | 40 | R1b |
| R2af | M8 | 258 | -150 | 40 | R2a |
| R2bf | M8 | -78 | -150 | 40 | R2b |
| R3af | M8 | 258 | 0 | 40 | R3a |
| R3bf | M8 | -78 | 0 | 40 | R3b |
| R4af | M8 | 258 | 150 | 40 | R4a |
| R4bf | M8 | -78 | 150 | 40 | R4b |
| R5af | M8 | 258 | 300 | 40 | R5a |
| R5bf | M8 | -78 | 300 | 40 | R5b |

| component | kind | grip point x | y | z |
|---|---|---|---|---|
| lid | lid | 0 | 0 | 200 |
| msd | msd | -200 | -330 | 135 |
| J1 | connector | -200 | 175 | 77 |
| J2 | connector | -200 | 265 | 77 |
| B00 | busbar | -95 | -335 | 134 |
| B01 | busbar | -45 | -225 | 134 |
| B02 | busbar | -45 | -75 | 134 |
| B03 | busbar | -45 | 75 | 134 |
| B04 | busbar | -45 | 225 | 134 |
| B05 | busbar | -95 | 335 | 134 |
| R1a | bracket | 258 | -300 | 29 |
| R1b | bracket | -78 | -300 | 29 |
| M1 | module | 90 | -300 | 144 |
| R2a | bracket | 258 | -150 | 29 |
| R2b | bracket | -78 | -150 | 29 |
| M2 | module | 90 | -150 | 144 |
| R3a | bracket | 258 | 0 | 29 |
| R3b | bracket | -78 | 0 | 29 |
| M3 | module | 90 | 0 | 144 |
| R4a | bracket | 258 | 150 | 29 |
| R4b | bracket | -78 | 150 | 29 |
| M4 | module | 90 | 150 | 144 |
| R5a | bracket | 258 | 300 | 29 |
| R5b | bracket | -78 | 300 | 29 |
| M5 | module | 90 | 300 | 144 |

### HVB-P6

| fastener | thread | x | y | seat z | holds |
|---|---|---|---|---|---|
| L01 | M6 | -300 | -400 | 164 | lid |
| L02 | M6 | -300 | -200 | 164 | lid |
| L03 | M6 | -300 | 0 | 164 | lid |
| L04 | M6 | -300 | 200 | 164 | lid |
| L05 | M6 | -300 | 400 | 164 | lid |
| L06 | M6 | 300 | -400 | 164 | lid |
| L07 | M6 | 300 | -200 | 164 | lid |
| L08 | M6 | 300 | 0 | 164 | lid |
| L09 | M6 | 300 | 200 | 164 | lid |
| L10 | M6 | 300 | 400 | 164 | lid |
| L11 | M6 | -150 | -490 | 164 | lid |
| L12 | M6 | 150 | -490 | 164 | lid |
| L13 | M6 | -150 | 490 | 164 | lid |
| L14 | M6 | 150 | 490 | 164 | lid |
| B00a | M6 | -145 | -410 | 136 | B00 |
| B00b | M6 | -45 | -410 | 136 | B00 |
| B01a | M6 | -45 | -340 | 136 | B01 |
| B01b | M6 | -45 | -260 | 136 | B01 |
| B02a | M6 | -45 | -190 | 136 | B02 |
| B02b | M6 | -45 | -110 | 136 | B02 |
| B03a | M6 | -45 | -40 | 136 | B03 |
| B03b | M6 | -45 | 40 | 136 | B03 |
| B04a | M6 | -45 | 110 | 136 | B04 |
| B04b | M6 | -45 | 190 | 136 | B04 |
| B05a | M6 | -45 | 260 | 136 | B05 |
| B05b | M6 | -45 | 340 | 136 | B05 |
| B06a | M6 | -45 | 410 | 136 | B06 |
| B06b | M6 | -145 | 410 | 136 | B06 |
| R1af | M8 | 258 | -375 | 40 | R1a |
| R1bf | M8 | -78 | -375 | 40 | R1b |
| R2af | M8 | 258 | -225 | 40 | R2a |
| R2bf | M8 | -78 | -225 | 40 | R2b |
| R3af | M8 | 258 | -75 | 40 | R3a |
| R3bf | M8 | -78 | -75 | 40 | R3b |
| R4af | M8 | 258 | 75 | 40 | R4a |
| R4bf | M8 | -78 | 75 | 40 | R4b |
| R5af | M8 | 258 | 225 | 40 | R5a |
| R5bf | M8 | -78 | 225 | 40 | R5b |
| R6af | M8 | 258 | 375 | 40 | R6a |
| R6bf | M8 | -78 | 375 | 40 | R6b |

| component | kind | grip point x | y | z |
|---|---|---|---|---|
| lid | lid | 0 | 0 | 200 |
| msd | msd | -200 | 0 | 135 |
| J1 | connector | -200 | 225 | 77 |
| J2 | connector | -200 | 315 | 77 |
| B00 | busbar | -95 | -410 | 134 |
| B01 | busbar | -45 | -300 | 134 |
| B02 | busbar | -45 | -150 | 134 |
| B03 | busbar | -45 | 0 | 134 |
| B04 | busbar | -45 | 150 | 134 |
| B05 | busbar | -45 | 300 | 134 |
| B06 | busbar | -95 | 410 | 134 |
| R1a | bracket | 258 | -375 | 29 |
| R1b | bracket | -78 | -375 | 29 |
| M1 | module | 90 | -375 | 144 |
| R2a | bracket | 258 | -225 | 29 |
| R2b | bracket | -78 | -225 | 29 |
| M2 | module | 90 | -225 | 144 |
| R3a | bracket | 258 | -75 | 29 |
| R3b | bracket | -78 | -75 | 29 |
| M3 | module | 90 | -75 | 144 |
| R4a | bracket | 258 | 75 | 29 |
| R4b | bracket | -78 | 75 | 29 |
| M4 | module | 90 | 75 | 144 |
| R5a | bracket | 258 | 225 | 29 |
| R5b | bracket | -78 | 225 | 29 |
| M5 | module | 90 | 225 | 144 |
| R6a | bracket | 258 | 375 | 29 |
| R6b | bracket | -78 | 375 | 29 |
| M6 | module | 90 | 375 | 144 |

## 5. Fasteners

Two thread sizes, each with two head types as built. A group (lid, busbars, brackets) is built with one head type, but
which one differs from pack to pack, and single fasteners have been replaced in service by the other head of the same
thread.

| head | thread | shape | size | head height (mm) |
|---|---|---|---|---|
| S10 | M6 | hexagon on a round flange | 10 mm across flats, flange dia. 14.2 mm | 6 |
| S13 | M8 | hexagon on a round flange | 13 mm across flats, flange dia. 17.8 mm | 8 |
| T30 | M6 | round pan head with a six-lobe (Torx) recess | head dia. 12 mm, recess 5.5 mm across | 4.5 |
| T40 | M8 | round pan head with a six-lobe (Torx) recess | head dia. 16 mm, recess 6.7 mm across | 6 |

Lid and busbar fasteners are M6, bracket fasteners M8.

| thread | assembly torque (N m) | breakaway, clean (N m) | breakaway, corroded (N m) | breakaway, seized (N m) | head's limit (N m) | shear torque, clean (N m) | shear torque, corroded or seized (N m) | wind-up (degrees per N m) | turns to free |
|---|---|---|---|---|---|---|---|---|---|
| M6 | 10 | 11.5 | 16 to 19 | 23 to 32 | 22 | 20 | 14 to 20 | 1.2 to 1.8 | 7 |
| M8 | 23 | 26.45 | 36.8 to 43.7 | 52.9 to 73.6 | 45 | 46 | 32.2 to 46 | 0.6 to 0.9 | 9 |

- **Loosening.** A fastener breaks away at its breakaway torque over the first 30 degrees and then runs out at about
  12 % of the assembly torque. After the number of turns in the table it is free; it stays on the bit (the bits are
  magnetic) until it is ejected.
- **Corroded fasteners** (rust-brown head, a rust stain around it) need the higher breakaway torque. A nutrunner
  whose torque limit is below a fastener's breakaway torque stalls at the limit without turning it.
- **The shank's strength.** A fully engaged bit that holds a fastener's shear torque, loosening or tightening, twists
  its shank off: the head comes away on the bit (eject it like a fastener), the stub stays in the thread and holds
  its part for good (the line's rework station drills it out; this cell cannot). A sheared fastener counts nothing,
  and the part it holds cannot be taken off. A clean fastener's shank has its thread's shear torque. Corrosion eats
  into the shank of a corroded or a seized fastener: its shear torque is anywhere in the range of the table, and
  nothing on the outside shows how much. A corroded fastener's breakaway can lie above its own shear torque; it then
  behaves as a seized one.
- **The yield point and the wind-up.** Under the torque the bit holds, the shank and the drive twist: the
  nutrunner's angle reading winds up by the wind-up of the table (degrees per N m, elastic, the same fastener's
  wind-up each time; it springs back when the torque is let off). Every shank yields at 86 to 92 % of its own shear
  torque: above its yield point it twists 6 times as fast, and that twist stays. At its shear torque it twists off.
- **The head's limit** holds both ways: a spindle stalled at a limit above it, tightening or loosening, wears the
  head through within half a second.
- **Seized fasteners.** Up to 4 fasteners of a pack, in any group (a bracket fastener twice as often as a lid or a
  busbar fastener), are seized by corrosion or by thread-locker; a seized fastener may look clean or rusty. Its
  breakaway is 1.15 to 1.6 times its thread's shear torque: no torque its shank survives turns it at first. Working
  breaks a seized thread's bond: each time a fully driven bit has held the fastener against loosening at 80 % or
  more of its own shear torque for 0.5 s in all, and then against tightening at the assembly torque or more for 0.25 s,
  what its breakaway exceeds a clean one's falls by a factor of 0.55 to 0.8 (one factor per fastener). Held at a lower
  torque, or tightened without the loosening before it, the bond does not change. A corroded fastener behaves the
  same way.
- **What the nutrunner reads.** Its torque transducer reads to within 0.15 N m plus 1 % of the reading (one
  standard deviation, every sample) about its own calibration line (`tool_catalog.md`, section 2), its angle encoder
  to within 0.2 degrees; the trace has one sample per control tick.
- **Tightening by mistake.** A seated fastener does not turn further: the spindle stalls at its limit, which does no
  harm below the head's limit and the fastener's yield point.
- **The wrong bit, or a bit that is not square on the head, strips the head at once** (section 3 of
  `tool_catalog.md` has the tolerances). A stripped head no longer takes its bit. The extractor bit EX removes a
  fastener with a rounded hexagon; a reamed Torx recess is smooth, nothing grips it, and its part stays held for good.
- **Carrying a freed fastener.** The bits are magnetic, but a freed fastener's shank is still in its hole: draw it out
  straight along its axis, 15 mm (M6) or 20 mm (M8), before the bit moves sideways (by 1.5 mm or more it catches the
  hole's edge and falls off). On the way out the magnet holds it while the tool's centre point (as the robot's
  joints put it) accelerates less than 4 m/s^2 and the tool's axis stays within 25 degrees of the fastener's axis;
  otherwise it falls where it is (onto a busbar: section 1).
- **Missing fasteners.** Field packs come back with fasteners missing (an empty, dark hole). There is nothing to
  remove there; what the fastener held may already be loose.

## 6. Order of work

1. Remove every bolt of the lid.
2. Lift the lid off by its handle and put it on the lid stand.
3. Pull the service disconnect; wait 30 s (section 1).
4. Pull the low-voltage plugs J1 and J2 from the BMS.
5. Remove the busbar bolts, then the busbars.
6. Remove the bracket bolts, then the brackets.
7. Lift each module out by its handle and stand it in a bay of the module rack; a swollen module stays in its seat
   (section 7).

Steps 4 to 6 may be interleaved as long as nothing live is touched before step 3 is complete and every component is
free before it is lifted. Sort everything that comes off (section 9).

## 7. What a field pack may look like

- **Placement on the fixture.** The fixture's stops locate the tray within +/- 12 mm in x and y and +/- 1.5 degrees
  about z of the nominal position.
- **Fastener positions.** Holes are drilled within 2 mm of their nominal positions.
- **Head types** vary as described in section 5; some fasteners are corroded, some are missing.
- **Sealant.** Packs resealed in service carry a dab of grey sealant on some fasteners (about 35 % of the lid's bolts,
  25 % of the busbars' and the brackets'): a dome standing on the seat, its centre 0.3 to 0.9 head radii off the
  fastener's axis, 0.6 to 1 head radii in radius, its top 1 to 2.5 mm above the head's top (a head's radius: half its
  width across flats, or a pan head's radius). It covers part of the head's top, typically a fifth and at most about
  half, and runs over the head's edge onto the seat. It is soft: a bit goes through it and engages as it would
  without it, and the dab comes away with its fastener. It hides part of the head from the cameras.
- **Grime, dust and labels.** Field packs come back dirty: 4 to 9 patches of dark grime or light dust lie on the
  lid (roughly elliptical, 30 to 140 mm across), anywhere, over bolts too; 1 to 4 service labels (white, with a
  printed bar code) are stuck to the lid's flat areas, never over a bolt, its handle or a rib; and some clean heads
  carry a dull film of dust (about 30 %) that takes the shine off them. None of it holds anything or changes how a
  bit engages.
- **Swollen modules.** A module whose cells have swollen pushes its top plate up: a smooth bulge 0.45 to 0.8 mm high at
  its centre (the height h of the shape h (1 - r^2)^2, r = 1 on the ellipse's edge), over an ellipse 180 to 240 mm long
  and 70 to 100 mm wide whose centre lies within 15 mm of the plate's centre along the module; the top plate of a
  healthy module carries the same smooth shape at a crown of 0.1 to 0.33 mm. A module whose top stands 0.4 mm or more above
  the plane of the plate's rim (its outer 5 mm) at the bulge's highest point is swollen. At most one module of a pack
  is swollen. A swollen module stays in the tray for the line's quarantine station: free it like any module (its
  brackets and busbars come off), but do not lift, push or pull it: moved 5 mm from its seat, its cells vent, and
  the pack is condemned (section 9). Left in its seat, freed, level and undamaged, it counts its value (section 9); a
  healthy module left in the tray counts nothing.
- **Foam pads.** A service shop sometimes glued a pad of soft open-cell foam over a lid bolt or a busbar bolt: light
  grey-beige, square, 40 to 60 mm across and 10 to 14 mm high above the seat, turned at any angle, its centre within
  8 mm of the fastener's nominal place, cut off at the lid's edge; up to 2 pads in a pack. A pad hides the head, or
  the empty hole of a missing
  fastener, from both cameras. It is glued to the lid or the busbar and leaves with it; the gripper cannot take it
  off. It is soft: a bit pressed through it reaches the head (or the seat) and engages as it would without it, and it
  does not hold the bit up. The fastener under a pad is at its place of section 4, within the 2 mm above.
- **A harness remnant.** Some packs carry a piece of blue low-voltage harness (cable 12 mm thick, 0.2 to 0.4 m long,
  a grey plug shell at each end) that was cut and left behind: lying on the lid over one or two lid bolts, or inside
  the pack along the busbars over two or three busbar bolts. It is loose. A bolt under it cannot be reached. The
  gripper takes the cable anywhere, across its direction. A turning bit pressed on the cable chafes it through. A harness lying on the busbars lies on live parts until the pack is
  isolated.

## 8. Photographs

`manual/photo_example_lid.jpg` and `manual/photo_example_inside.jpg`: an example pack (an HVB-P5) under the cell's
overhead camera, lid on and lid off. `manual/photo_heads.jpg`: the four head types, clean and corroded, an empty hole,
and three heads with a dab of sealant (section 7), as the wrist camera sees them from about 0.35 m (cropped and
enlarged). For these photographs the example pack's lid was fitted with M8 heads as well, which a lid never carries.
The example pack is not one of the packs you will meet.

## 9. Sorting and recovered value

| component | goes to | value |
|---|---|---|
| module | a bay of the module rack, standing on the bay's floor (one module per bay); a swollen module: its seat in the tray, freed (section 7) | 8 |
| lid | the lid stand, lying flat | 5 |
| service disconnect | electronics bin | 3 |
| busbar | copper bin | 2 |
| bracket | steel bin | 1 |
| plug J1, J2 | electronics bin | 1 |
| harness remnant | electronics bin | 1 |
| fastener | either fasteners bin | 0.5 |

A component counts when it lies undamaged where it belongs. Damaged means: a fastener with a stripped head or a
sheared shank, a harness chafed by the nutrunner, a module that was dropped or that vented.

**A pack counts only when it is taken apart to module level.** That is the cell's job: a pack goes on to the next
station only with every healthy module standing in the rack and a swollen one freed in its seat for quarantine. A
pack with a module still held in its tray, or a module anywhere else, goes to the shredder as a whole pack, and none
of what was sorted from it is booked to the cell: it counts 0.

**A condemned pack counts nothing.** A pack in which a safety event happened goes to the facility's quarantine
whole, with everything already sorted from it, as the facility's rule for a possible thermal event is: electrolyte
and hydrogen fluoride from a damaged cell may have reached what was sorted, and none of it is recovered. The safety
events are: a tool on a live part before isolation (section 1), a loose fastener on a busbar (section 1), a module
that vents (section 7) and a module that is dropped (it falls more than about 25 mm: its cells are crushed).
