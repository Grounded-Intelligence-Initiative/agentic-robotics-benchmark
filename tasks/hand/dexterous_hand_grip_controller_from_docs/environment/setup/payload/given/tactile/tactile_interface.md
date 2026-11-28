# Tactile modules: interface note

The hand carries four tactile modules ("pads"), one on the last link of each digit: the thumb tactile module
ZCP-DG1Z on the thumb (127 taxels) and a fingertip tactile module ZCP-F561 on each of
the index, middle and ring fingers (52 taxels each). The modules connect to the tactile interface board
ZCP-384F, which pushes one frame of all four modules to the host computer over its own serial line. This
note gives what the cell's runtime passes on and how a module's bytes are encoded; it is written from the makers'
driver code and the module vendor's figures (`parameter_table.md`).

## 1. What the controller receives

Each observation carries the newest tactile frame that arrived since the previous observation, or null when none
arrived:

```
"tactile": {"seq": <frame number>, "t_capture_ms": <float>, "t_arrival_ms": <float>,
            "modules": {"thumb": "<hex>", "index": "<hex>", "middle": "<hex>", "ring": "<hex>"}
           }
```

- `t_capture_ms`: when the board captured the frame; `t_arrival_ms`: when the frame had arrived at the host; both in ms
  since the unit's hand-over.
- The board captures a frame every 1 / f s, f being the unit's tactile frame rate (its record, `tactile.frame_hz`).
- A frame is 880 bytes; at 921600 baud and 10 bits per byte it arrives
  9.6 ms after its capture.
- `modules`: each module's part of the frame, two hex digits per byte: 387 bytes for the thumb,
  162 bytes for a finger.

## 2. A module's bytes

| bytes | content | encoding |
|---|---|---|
| 0-5 | the resultant block: x, y, z | 3 x int16, little-endian, 0.01 N per LSB, in the module frame |
| 6 + 3 i, 7 + 3 i | taxel i: Fx, Fy | int8 each, 0.1 N per step |
| 8 + 3 i | taxel i: Fz | uint8, 0.1 N per step |

- Taxels are numbered from 0 in the order of the taxel maps (`../hand/taxel_maps/`: the map's row n is taxel n - 1).
- Fz is unsigned. A value below the taxel's zero wraps: one step below zero reads 0xFF (25.5 N). A value
  of 255 steps or more reads 255. Fx and Fy saturate at -128 and 127.
- The module frame is the pad frame of the kinematics sheet (`../hand/kinematics_sheet.md`, the pads table): its
  origin, its x axis and its normal (z, out of the pad).
- Signs. A taxel's Fz is its share of the force pressing on the pad: pressing reads positive. A taxel's Fx and Fy are
  its share of the force along the pad's surface that the contact applies to the pad, along the module frame's x and
  y axes. The resultant block is the total force the contact applies to the pad: x and y along the module frame's
  axes, z counted positive into the pad, so pressing reads a positive z. A re-zero zeroes the resultant block too.

## 3. Zero and drift

- Each taxel's reading is taken against its zero. The unit's record gives each taxel's zero residual at delivery
  (-1, 0 or +1 step).
- Each taxel's zero drifts slowly while the hand works.
- `tactile_zero` in a step reply re-zeroes the named modules: each named module's zero becomes the values it reports
  at that tick. In the makers' driver: "Any contact present at call time becomes the new zero".

## 4. Scale

The scale is uncalibrated: 0.01 N and 0.1 N per step are the modules'
own units. How a module's reading relates to the true force on its pad has not been calibrated on this hand; the
makers' notes (`../makers/handoff.md`, section 6) are the only evidence there is.
