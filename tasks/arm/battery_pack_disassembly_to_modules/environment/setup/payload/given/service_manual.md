# HVB-P traction battery: workshop manual, removal of the modules

Issue 3. Applies to the plug-in hybrid packs HVB-P4, HVB-P5 and HVB-P6 (4, 5 and 6 modules in one common housing).
This manual describes the pack as built, what a pack returned from the field may look like, and the order in which it
is taken apart to module level. Dimensions are nominal unless a tolerance is given.

## 1. Safety: high voltage

The busbars, the modules' terminal posts and the junction studs are live as long as the service disconnect (MSD) is
in its socket: the modules are then connected in series (up to 320 V). **Nothing conductive may touch a busbar, a
busbar bolt, a terminal post or a junction stud before the pack is isolated.** In the disassembly cell an insulation
monitor trips when a tool touches (comes within 3 mm of) one of them before isolation: the cell stops for good and
the pack is taken away as it is.

Isolation procedure:

1. Remove the lid (section 6, steps 1 and 2).
2. Pull the service disconnect straight up out of its socket and take it away.
3. Wait 30 s: the pack's link capacitor discharges through its bleed resistor.

After that the string is open and no part carries more than one module's voltage (below 60 V): busbars and their
bolts may be touched and removed. The lid's bolts, the lid, the service disconnect, the low-voltage plugs, the
brackets and their bolts and the modules' housings are never live.

## 2. The series

All three variants use the same tray and the same lid outline; they differ in the number of modules, in where the
service disconnect and the battery management unit (BMS) sit, and in the lid:

| variant | modules | lid bolts | lid ribs (raised strips on the lid) | MSD at y = | BMS at y = |
|---|---|---|---|---|---|
| HVB-P4 | 4 | 10 | along y at x = -140 mm; along y at x = 140 mm | 330 mm | -250 mm |
| HVB-P5 | 5 | 12 | along x at y = -350 mm; along x at y = -150 mm; along x at y = 150 mm; along x at y = 350 mm | -330 mm | 220 mm |
| HVB-P6 | 6 | 14 | along y at x = -140 mm; along y at x = 140 mm; along x at y = -300 mm; along x at y = 300 mm | 0 mm | 270 mm |

The variant is not marked on the pack. Tell it from the lid (rib pattern, number of lid bolts) and, once the lid is
off, from the number of modules.

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
thread: **look at every head before choosing the bit.**

| head | thread | shape | size | head height (mm) |
|---|---|---|---|---|
| S10 | M6 | hexagon on a round flange | 10 mm across flats, flange dia. 14.2 mm | 6 |
| S13 | M8 | hexagon on a round flange | 13 mm across flats, flange dia. 17.8 mm | 8 |
| T30 | M6 | round pan head with a six-lobe (Torx) recess | head dia. 12 mm, recess 5.5 mm across | 4.5 |
| T40 | M8 | round pan head with a six-lobe (Torx) recess | head dia. 16 mm, recess 6.7 mm across | 6 |

Lid and busbar fasteners are M6, bracket fasteners M8.

| thread | assembly torque (N m) | breakaway, clean (N m) | breakaway, corroded (N m) | never exceed (N m) | turns to free |
|---|---|---|---|---|---|
| M6 | 10 | 11.5 | 16 to 19 | 22 | 7 |
| M8 | 23 | 26.45 | 36.8 to 43.7 | 45 | 9 |

- **Loosening.** A fastener breaks away at its breakaway torque over the first 30 degrees and then runs out at about
  12 % of the assembly torque. After the number of turns in the table it is free; it stays on the bit (the bits are
  magnetic) until it is ejected.
- **Corroded fasteners** (rust-brown head, a rust stain around it) need the higher breakaway torque. A nutrunner
  whose torque limit is below a fastener's breakaway torque stalls at the limit without turning it: raise the limit,
  never above the "never exceed" value of the thread.
- **Tightening by mistake.** A seated fastener does not turn further: the spindle stalls at its limit. With the limit
  above the "never exceed" value the head gives way within half a second.
- **The wrong bit, or a bit that is not square on the head, wears the head** (section 3 of `tool_catalog.md` has the
  tolerances). A worn-out (stripped) head no longer takes its bit; only the extractor bit EX removes such a fastener.
- **Missing fasteners.** Field packs come back with fasteners missing (an empty, dark hole). There is nothing to
  remove there; what the fastener held may already be loose.

## 6. Order of work

1. Remove every bolt of the lid.
2. Lift the lid off by its handle and put it on the lid stand.
3. Pull the service disconnect; wait 30 s (section 1).
4. Pull the low-voltage plugs J1 and J2 from the BMS.
5. Remove the busbar bolts, then the busbars.
6. Remove the bracket bolts, then the brackets.
7. Lift each module out by its handle and stand it in a bay of the module rack.

Steps 4 to 6 may be interleaved as long as nothing live is touched before step 3 is complete and every component is
free before it is lifted. Sort everything that comes off (section 9).

## 7. What a field pack may look like

- **Placement on the fixture.** The fixture's stops locate the tray within +/- 12 mm in x and y and +/- 1.5 degrees
  about z of the nominal position.
- **Fastener positions.** Holes are drilled within 2 mm of their nominal positions.
- **Head types** vary as described in section 5; some fasteners are corroded, some are missing.
- **A harness remnant.** Some packs carry a piece of blue low-voltage harness (cable 12 mm thick, 0.2 to 0.4 m long,
  a grey plug shell at each end) that was cut and left behind: lying on the lid over one or two lid bolts, or inside
  the pack along the busbars over two or three busbar bolts. It is loose. A bolt under it cannot be reached; take
  the harness away first (grip the cable anywhere, across its direction). Never run the nutrunner on it: a turning
  bit pressed on the cable chafes it through. A harness lying on the busbars lies on live parts until the pack is
  isolated.

## 8. Photographs

`manual/photo_example_lid.jpg` and `manual/photo_example_inside.jpg`: an example pack (an HVB-P5) under the cell's
overhead camera, lid on and lid off. `manual/photo_heads.jpg`: the four head types, clean and corroded, and an empty
hole, as the wrist camera sees them from about 0.25 m. The example pack is not one of the packs you will meet.

## 9. Sorting and recovered value

| component | goes to | value |
|---|---|---|
| module | a bay of the module rack, standing on the bay's floor (one module per bay) | 10 |
| lid | the lid stand, lying flat | 4 |
| service disconnect | electronics bin | 3 |
| busbar | copper bin | 2 |
| bracket | steel bin | 1 |
| plug J1, J2 | electronics bin | 1 |
| harness remnant | electronics bin | 1 |
| fastener | either fasteners bin | 0.25 |

A component counts when it lies undamaged where it belongs. Damaged means: a fastener with a stripped head, a harness
chafed by the nutrunner, a module that was dropped.
