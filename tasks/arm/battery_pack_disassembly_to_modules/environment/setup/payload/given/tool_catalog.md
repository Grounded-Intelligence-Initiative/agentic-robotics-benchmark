# Disassembly cell: tool catalog

Two tools hang in the tool stand; the robot carries one at a time on its tool changer. The wrist camera sits on the
changer's robot side, so it is there with either tool (`cell/cell_interface.md`, section 5).

## 1. Tool changer

Each tool has a dock on the tool stand. A tool is set down or picked up with the flange in the dock position, the
tool's axis straight down and the flange's x axis along the cell's x axis, within 6 mm and 3 degrees, at rest.
`tool_release` sets the mounted tool down in its own dock; `tool_lock` picks up the tool of the dock the bare flange
is in. Either takes 2 s, during which the robot does not move. A tool that holds something (a fastener on the bit,
a part in the gripper) cannot be set down.

## 2. Nutrunner NR-60

A straight electric nutrunner with a sprung, floating spindle and exchangeable magnetic bits.

|  |  |
|---|---|
| mass | 4.2 kg |
| tool centre point | the bit's tip, 300 mm from the flange along the tool axis with the spindle unloaded |
| spindle float | 25 mm; spring 4 N per mm of compression (`stroke_mm` in the observation) |
| speed | up to 300 rpm either way; a negative speed loosens |
| torque limit | settable from 0 to 60 N m; the spindle stalls at the limit |
| readings | torque (N m, the cell's transducer: below), speed (rpm), angle (degrees, cumulative, signed), stroke (mm) |
| bit and spindle diameter | below 22 mm over the last 130 mm |
| body | diameter 60 mm, from 30 to 170 mm from the flange |

**The torque transducer is the cell's own.** Each NR-60 in the line has its own transducer, and the spindle's
controller works from its reading: while the spindle runs, the reading is a gain of 0.88 to 1.12 times the torque on the
bit plus an offset of -0.3 to 0.3 N m, the same throughout a cell's life and different from cell to cell, and the
spindle stalls where the reading meets the commanded torque limit (so a commanded limit L holds the torque
(L - offset) / gain). With the spindle standing the reading is 0. A development cell's transducer shows what the
line's transducers are like, not what a graded cell's gain and offset are. Every torque in the service manual is a
torque on the bit, not a reading.

**Bits.** S10 and S13: sockets for 10 and 13 mm hexagons. T30 and T40: Torx bits. EX: an extractor for stripped
heads. The bit is exchanged in the bit station of the tool stand: the bit's tip in the station position, tool axis
straight down, within 6 mm and 3 degrees, at rest; `bit:<name>` then takes 2.5 s. The nutrunner starts every pack
with S10.

What a turning bit does on a head:

| bit | on S10 | on S13 | on T30 | on T40 |
|---|---|---|---|---|
| S10 | drives it | turns freely | turns freely | turns freely |
| S13 | strips it | drives it | turns freely | turns freely |
| T30 | turns freely | turns freely | drives it | strips it |
| T40 | turns freely | turns freely | turns freely | drives it |
| EX | strips it | strips it | strips it | strips it |

## 3. Engaging a fastener

The bit drives a fastener when all of this holds:

- it is the head's own bit (table above);
- the bit's axis is within 0.8 mm of the fastener's axis for the 10 mm socket, 1.2 mm for the 13 mm socket (its larger
  lead-in chamfer), 0.5 mm for a Torx bit, and the two axes are within 2 degrees of each other;
- the bit is pressed on the head with at least 8 N (the spindle's spring: stroke times the spring rate); a Torx bit
  in addition needs 10 N plus 1.5 N for every N m it holds, turning or stalled at its limit, or it cams out of the
  recess;
- the torque limit is at least what the fastener needs (the service manual's torque table).

A bit that is on the head but outside these tolerances (up to 3.5 mm off for a socket, 2.5 mm for a Torx bit, up to
8 degrees), a bit one size too large or too small in the "strips it" cells, or a Torx bit that cams out, is on the
head without driving it. Run with a torque limit of 1 N m or more, it strips the head at once: a socket rounds the
hexagon's corners, a Torx bit reams the recess out, and no reading warns before it happens. Below that limit the
spindle only stalls on the head, and the reading looks the same as on a head the bit drives. Further off than that,
or with a bit that does not fit at all, the bit turns freely (torque about 0.15 N m) and nothing happens to the head.

A spindle that is stalled at its limit on a fastener it should drive is doing no harm, as long as the limit is below
the head's limit and the fastener's yield point (the service manual, section 5; at its shear torque the shank twists
off).

**The extractor EX** bites into a rounded hexagon (S10, S13): within 1.5 mm and 2 degrees, pressed on with at least
50 N, with the torque limit at 1.2 times the fastener's breakaway torque (below its shear torque, which holds for the
extractor too); it turns the fastener out at half the spindle's speed. In a reamed Torx recess it finds nothing to
bite on and turns freely; such a fastener stays in for good. On a sound head it strips it.

**A freed fastener** stays on the bit. `eject` lets it fall; do that over a fasteners bin. With a fastener on the
bit the nutrunner engages nothing else and the bit cannot be exchanged.

## 4. Gripper GP-85

A parallel gripper with two long fingers.

|  |  |
|---|---|
| mass | 2.6 kg |
| opening | 0 to 85 mm, 80 mm/s; the fingers move along the flange's y axis |
| tool centre point | between the fingertips, 235 mm from the flange along the tool axis, 15 mm above the fingers' ends |
| fingers | 130 mm long, 22 mm wide (along the flange's x axis) |
| body | 50 x 100 mm (flange x by y), from 30 to 120 mm from the flange |
| readings | opening (m), holding (true while the fingers hold a part) |

**Gripping.** The gripper takes a part by one of its grip features (the service manual, section 4). Closing the
fingers on a feature holds the part when

- the tool axis points down onto the part within 20 degrees of the part's own vertical,
- the fingers' closing direction is within 20 degrees of the feature's (25 degrees across a harness cable),
- the feature lies between the fingers: the tool centre point within 12 mm of the grip point along the tool axis and
  along the flange's x axis, and the feature inside the opening,
- the opening before closing is at least the feature's width.

The fingers stop at the feature's width and `holding` becomes true. The part stays as it lay relative to the
fingers; it is not centred. Opening the gripper (commanding an opening more than 1 mm wider) lets go.

The gripper holds any of the pack's components against its own weight in any motion the robot's limits allow. It
does not tear a held component loose: a part that is still fastened stays where it is and the robot is overloaded
(the cell stops it). The plugs and the service disconnect are the exception: they are meant to be pulled out
(service manual, section 4).
