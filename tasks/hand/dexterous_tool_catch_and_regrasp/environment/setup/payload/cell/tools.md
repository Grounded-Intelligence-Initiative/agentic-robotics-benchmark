# The tool set: nominal drawings

Every tool has its own frame: origin on the tool's axis at the **butt** of the handle (the end away from the working
end; where the handle has a rounded end cap, at the centre of that cap's base, so the cap reaches a little behind
x = 0), **+x along the tool's axis towards the working end**. Dimensions are full sizes in metres at nominal size; the tools in the cell
differ from these by a few percent in size, and the metal parts of some are heavier or lighter than nominal (a
heavier hammer head, for example). Colours are as the cameras see them under the cell's lights (RGB, 0..1, before
shading).

The **handle** is the part of a tool made for the hand. Each throw asks for one of two **grip zones** on it,
measured along the tool's axis from the butt:

- `end`: the butt-side 30 % of the handle (a full swing grip);
- `choke`: the working-end-side 30 % of the handle (a choked-up, control grip).

| tool | handle (x from butt) | `end` zone | `choke` zone | nominal mass | centre of mass (x) |
|---|---|---|---|---|---|
| hammer | 0.030 .. 0.200 | 0.030 .. 0.081 | 0.149 .. 0.200 | 0.57 kg | 0.245 |
| screwdriver | 0.030 .. 0.110 | 0.030 .. 0.054 | 0.086 .. 0.110 | 0.22 kg | 0.114 |
| wrench | 0.010 .. 0.130 | 0.010 .. 0.046 | 0.094 .. 0.130 | 0.35 kg | 0.186 |
| spatula | 0.030 .. 0.170 | 0.030 .. 0.072 | 0.128 .. 0.170 | 0.23 kg | 0.175 |

## Hammer (overall length 0.316)

- Handle: wood (0.62, 0.45, 0.28), round, diameter 0.032: a cylinder from x = 0 to 0.26 with hemispherical
  ends (x -0.016 .. 0.276 overall), axis on the tool axis.
- Neck: steel (0.55, 0.56, 0.58), a square block 0.024 long (x 0.260 .. 0.284), 0.022 x 0.022.
- Head: steel, a bar 0.022 (x) by 0.084 (y) by 0.022 (z), centred at x = 0.295, its long side across the tool
  (along the tool's y).

## Screwdriver (overall length 0.29)

- Handle: red plastic (0.55, 0.16, 0.14), round, diameter 0.036, from x = -0.003 to 0.143 including rounded ends.
- Shaft: steel, round, diameter 0.008, x 0.116 .. 0.284 including rounded ends.
- Tip: steel blade 0.012 x 0.012 x 0.005 at x = 0.285.

## Wrench (overall length 0.285)

- Handle: blue vinyl-dipped grip (0.20, 0.30, 0.58), a flat bar 0.13 long (x 0 .. 0.13), 0.028 wide (y),
  0.012 thick (z).
- Shaft: steel, a flat bar x 0.13 .. 0.25, 0.018 wide, 0.008 thick.
- Jaw: steel, 0.036 (x) by 0.044 (y) by 0.010 (z), centred at x = 0.268.

## Spatula (overall length 0.34)

- Handle: green rubber (0.22, 0.42, 0.24), round, diameter 0.028, x 0.006 .. 0.194 including rounded ends.
- Neck: steel strip x 0.18 .. 0.24, 0.014 wide, 0.006 thick.
- Blade: nylon (0.78, 0.74, 0.64), 0.10 (x) by 0.08 (y) by 0.006 thick, centred at x = 0.29.
