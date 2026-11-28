# Truss tomatoes on the high wire: growing protocol and harvest rules

This greenhouse grows truss tomatoes on the high wire and harvests them by the truss: a truss is cut off whole, with
all its fruits on it, once enough of them have coloured. This document says how the crop stands in a row, what the two
cultivars are, what a row section may look like on the day, and what the greenhouse's rules for the harvest are.
Positions and heights are in the row frame of `../site/site_interface.md` (section 1); where two numbers are given,
they are the range over plants or rows.

## 1. The plant

- **The row section.** 4 plants stand in a substrate slab on the gutter, one every 0.5 m: the first at y = 0.3, all at
  x = 1.2, each within 30 mm of that in x and 50 mm in y, the slab's top at z = -0.48. Each plant carries 3 trusses
  within the robot's reach, 12 in the row section.
- **The main stem** is 11 to 13.6 mm thick and is trained up a support string that hangs from the high wire; 1.8 m of
  it stand in the section. It leans by up to about 8 degrees along the row and 3 degrees across it, and changes its
  lean a little from one stretch to the next. It gives way: pushed or pulled, it bends and swings back, and so does
  everything that grows on it. In the cell a stem is five straight pieces of 0.36 m on sprung joints (the cultivar's
  stiffness, section 2; the joint at the slab 2.5 times as stiff), and the string holds its top sideways as a spring.
- **Where the trusses are.** A plant's trusses grow from the main stem at heights of z = 0 to 0.12; 0.27 to 0.45; 0.54
  to 0.78 m, lowest first (each 0.27 to 0.33 m above the one below).
- **A truss** is a peduncle, a rachis and the fruits. The *peduncle* is the stalk from the main stem to the knuckle.
  It leaves the stem toward the aisle, turned 8 to 50 degrees to one side of the straight line to the aisle (the
  trusses of a plant to alternating sides; where that would have left a truss without a free approach, section 4,
  anywhere within 50 degrees to either side), and slopes 30 to 62 degrees below the horizontal. At the *knuckle* the
  truss turns down: from there the *rachis* hangs nearly plumb, 0.9 times as thick as the peduncle. The *fruits* sit
  on pedicels of 5 to 10 mm on alternating sides of the rachis: the first one's pedicel leaves the rachis 12 to 20 mm
  plus that truss's fruit radius below the knuckle, the next ones every fruit radius plus 3 to 8 mm. A fruit is round,
  with a green calyx where its pedicel ends; its skin is glossy, so lamps show on it as highlights. The fruits of a
  truss are within 6 % of each other in radius, and a fuller truss has smaller fruits (the cultivar's largest radius
  falls by 1.2 mm with every fruit above its fewest).
- **How a truss moves.** A truss hangs from its knuckle like a short, heavy chain. In the cell the peduncle sits on
  the stem with a sprung joint (the cultivar's stiffness, section 2), the rachis is three straight pieces of equal
  length, the knuckle bends against 0.15 N m/rad and the rachis's two joints against 0.15 N m/rad, and about its own
  axis each piece turns against 0.02 N m/rad only (the pedicels give: a truss laid down settles flat).
- **Leaves.** Above every truss the stem carries two leaf nodes, 85 to 125 mm and 165 to 215 mm above the truss, each
  about 137.5 degrees (+/- 20) further round the stem than the one before. A leaf is a petiole, 70 to 110 mm long and
  4.4 to 6.4 mm thick, rising 8 to 32 degrees above the horizontal, and beyond it a blade 220 to 300 mm long and 130
  to 180 mm wide that droops 20 to 60 degrees below the horizontal. In the cell a leaf is one rigid plate with its
  petiole on a sprung joint at the stem (0.35 to 0.9 N m/rad): pushed, it turns aside and comes back.
- **Leaf removal.** The grower takes leaves off from below as the trusses ripen. On the day, 55 to 90 % of the leaves
  are still on, depending on the row, and of the two above a plant's lowest truss 25 to 40 %; section 4 says which
  leaves are always off.

What the crop's green parts look like (diffuse reflectance, linear RGB); the fruits' colours are in section 2:

| part | red | green | blue |
|---|---|---|---|
| main stem | 0.2 | 0.36 | 0.1 |
| peduncle, rachis, pedicels, petioles | 0.22 | 0.4 | 0.11 |
| leaf blade | 0.07 | 0.24 | 0.05 |
| calyx | 0.1 | 0.26 | 0.05 |

## 2. The cultivars

Two cultivars are grown. The rows your program is graded on grow Rossa; the practice rows of the commissioning service
grow Ambra.

|  | Rossa | Ambra |
|---|---|---|
| fruits on a truss | 4 to 6 | 5 to 7 |
| a truss's fruit radius (mm) | 27 to 32 | 21 to 25 |
| a fruit's mass (g) | 65 to 130 | 31 to 62 |
| a truss's fruits together (kg) | 0.26 to 0.62 | 0.15 to 0.32 |
| peduncle length, stem to knuckle (mm) | 45 to 75 | 40 to 65 |
| peduncle diameter (mm) | 4.4 to 6.4 | 3.6 to 5.2 |
| rachis length, knuckle to tip (mm) | 144 to 256 | 141 to 242 |
| peduncle's joint to the stem (N m/rad, per truss) | 6 to 12 | 3 to 7 |
| main stem's joints (N m/rad, per row) | 35 to 90 | 25 to 60 |
| support string, sideways (N/m, per row) | 90 to 260 | 60 to 180 |
| a truss is ready when | at least 80 % of its fruits (rounded up) are at stage 4 or beyond | at least 60 % of its fruits (rounded up) are at stage 5 or beyond |
| that is | 4 of 4, 4 of 5, 5 of 6 | 3 of 5, 4 of 6, 5 of 7 |

**Ripening stages and the colour chart.** A fruit ripens through the stages 1 to 6. Its stage shows in the colour of
its skin and nowhere else. Each cultivar has its colour chart: the skin's diffuse reflectance (linear RGB) at each
stage.

| Rossa, stage | red | green | blue |
|---|---|---|---|
| 1 | 0.3 | 0.44 | 0.12 |
| 2 | 0.46 | 0.47 | 0.13 |
| 3 | 0.6 | 0.42 | 0.11 |
| 4 | 0.68 | 0.3 | 0.09 |
| 5 | 0.66 | 0.17 | 0.06 |
| 6 | 0.56 | 0.05 | 0.03 |

| Ambra, stage | red | green | blue |
|---|---|---|---|
| 1 | 0.26 | 0.42 | 0.14 |
| 2 | 0.44 | 0.5 | 0.14 |
| 3 | 0.66 | 0.58 | 0.1 |
| 4 | 0.8 | 0.58 | 0.07 |
| 5 | 0.84 | 0.44 | 0.05 |
| 6 | 0.82 | 0.3 | 0.03 |

A fruit between two stages has a colour between theirs, in proportion. No fruit is on the border between two stages:
every fruit's ripeness is within 0.35 of a whole stage. A fruit's own colour is the chart's within 5 % (lighter or
darker as a whole).

**How a plant ripens.** On a truss the fruit nearest the knuckle is the ripest, and each next one is about 0.45 of a
stage behind the one before it (every fruit within 0.3 of that line). On a plant the lowest truss is the ripest. The
ready trusses of a plant are therefore its lowest ones: none, one or two. The truss above a plant's ready ones is
often close behind them, a single fruit short of ready.

## 3. What differs from row to row

No two row sections are alike, and none of the following is on the wire. Each is the row's own and stays the same
while the row section is worked.

- where each plant stands, how its stem leans, where its trusses and leaves are and how they hang (section 1);
- how many fruits each truss has, their size, and the length and thickness of each peduncle (section 2);
- which trusses are ready: 4 to 6 of the 12, at most two on a plant (section 2);
- how many leaves are still on (section 1);
- the grow light: its brightness, 0.65 to 1.35 times the nominal, and its colour, a green share of 0.35 to 1 and a
  blue share of 0.45 to 1 of its red share against the nominal white (`../site/site_interface.md`, section 6);
- the toughness of the row's stalks, 2.4 to 5.2 N/mm^2 (the season, the irrigation): what it takes to cut one
  (`../tool/cutter_gripper.md`, section 3);
- how stiff the stems and their strings are (section 2), and how quickly a plant that was touched comes to rest (a
  damping ratio of 0.15 to 0.35);
- the rails: the cart's wheels slip by 2 to 9 % (`../site/site_interface.md`, section 5).

## 4. The free approach

The grower keeps every ready truss reachable: a leaf that would close a ready truss's free approach has been taken
off, and a plant's trusses hang so that one is free. What that means exactly: for every ready truss, with everything
at rest, one approach is free. In it the tool comes with its x axis along the peduncle, pointing to the main stem, and
its mouth centred on the peduncle's axis midway between the point 24 mm from the stem and the point 10 mm short of the
knuckle, and it travels the last 300 mm in a straight line along its own z axis, perpendicular to the peduncle, in the
vertical plane through the peduncle and pointing downward (the tool comes from above and from the aisle). The approach
is free when, on that line, the tool with open jaws, the wrist camera's housing and the robot's wrist behind the
flange (taken as a cylinder of 0.12 m radius about the tool's axis, 0.32 m long) come no nearer than 8 mm to any main
stem, any fruit or any other truss's peduncle, and no nearer than 12 mm to any leaf. Nothing is promised about a truss
that is not ready or about any other way in.

## 5. The greenhouse's rules

A truss is sold whole and unmarked. What follows cannot be undone, and each rule holds from the moment the row section
starts until it has been judged, on the plant, in the tool and in the crate, for ready trusses and for unready ones
alike.

- **Only ready trusses are harvested.** A truss cut before it is ready is worth nothing, and it cannot be put back.
- **The cut is made on the peduncle,** between the main stem and the knuckle. A cut through the rachis, anywhere along
  it, parts the truss: it is waste. A cut petiole drops its leaf and costs nothing else. **A cut main stem loses the
  plant:** none of that plant's trusses counts, those already in a crate included.
- **A cut truss must not fall.** With the pad on the plant's side of the cut nothing holds the truss
  (`../tool/cutter_gripper.md`, section 3).
- **Bruised by a blow.** A truss is bruised, and waste, when the velocity of a piece of its rachis, with the fruits on
  it, changes by more than 1.5 m/s within one control tick of 0.05 s: a fall, a hard landing, a blow, a jerk of the
  tool that holds it (what a free fall of 115 mm has gathered when it ends, or 30 m/s^2 for one tick).
- **Squeezed.** A truss is squeezed, and waste, when the tool, the robot's wrist or forearm, or the cart with its deck
  and crates presses on one of its fruits or on its rachis with more than 25 N (the contact force, looked at once per
  control tick): a fruit pushed against a crate's wall, a truss pressed onto a crate's floor, an arm that leans on a
  truss on its plant.
- **Crushed.** A peduncle that the clamp pad presses with more than 1.4 times that peduncle's own cutting force is
  crushed, and its truss is waste (`../tool/cutter_gripper.md`, section 3).
- **One layer.** A crate takes one layer of trusses. A truss that lies on another damages both: neither counts. A
  truss lies on another when one of its fruits rests on one of the other's (they touch, their centres closer than 1.1
  times the sum of their radii, and the upper one's centre is higher by more than 0.5 times that sum). It makes no
  difference whether either of the two was ready.
- **The robot works unattended.** At the third protective stop in a row section its safety controller latches, and the
  work on that row section ends (`../site/site_interface.md`, section 4).

## 6. A delivered truss

A ready truss is delivered, and counts, when it has been cut on its peduncle, its plant still stands, it is whole, not
bruised, not squeezed and not crushed, the tool has let go of it, and it lies in one of the cart's two crates in a
single layer, every fruit inside the crate. The exact conditions are in `../site/site_interface.md`, section 8. A row
section's yield is the number of its ready trusses that count divided by the number of its ready trusses; nothing is
subtracted.
