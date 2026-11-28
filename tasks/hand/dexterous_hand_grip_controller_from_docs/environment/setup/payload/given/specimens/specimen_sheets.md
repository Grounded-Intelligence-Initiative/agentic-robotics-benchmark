# Specimen sheets

The acceptance uses instrumented specimens. Each is two half-shells joined by a spring along the squeeze axis, so
that the cell measures, at every tick, the spring's force and travel, the specimen's pose and which pads touch it. A
specimen is 35 to 45 mm wide along the squeeze axis, 50 mm
across and 50 mm high; the pads' line of contact is 10 mm below its
top. The machine-readable form of these sheets is `specimen_sheets.json` (it is also `cell["specimens"]` at setup).

## 1. Classes and size codes

| class | code | mass range | squeeze limit |
|---|---|---|---|
| brittle shell | SH-1 | 0.040 to 0.700 kg | spring force at most 4 N |
| brittle shell | SH-2 | 0.068 to 0.700 kg | spring force at most 6.4 N |
| brittle shell | SH-3 | 0.117 to 0.700 kg | spring force at most 10.6 N |
| soft pack | SP | 0.030 to 0.700 kg | spring travel at most 6.4 mm |
| full-load block | FL | 0.150 to 0.700 kg | none |

- **Brittle shell** (codes SH-1, SH-2, SH-3): its spring force must stay at or below the code's ceiling; above the
  ceiling for 10 ms or more the shell is crushed.
- **Soft pack** (code SP): its spring is between 1 and 10 N/mm;
  its spring travel must stay at or below the travel limit; above it for 10 ms or more the pack is
  crushed.
- **Full-load block** (code FL): no squeeze limit.

## 2. Finishes

The ticket names the specimen's finish; the friction coefficient between a pad and the specimen lies in the finish's
range:

| finish | friction coefficient |
|---|---|
| matte | 0.40 to 0.55 |
| rough | 0.55 to 0.70 |

## 3. The label rule

Each specimen is labelled by its mass: one of at most 0.20 kg is to be carried; one of
0.50 kg or more is to be declined. Nothing in between is presented. The label is not on the
ticket, and every class and code covers both labels within its mass range.

## 4. Placement

In the pick nest within 2 mm of the nest's centre in any direction and within
3 deg of its yaw (`../cell/cell_manual.md`, section 6).
