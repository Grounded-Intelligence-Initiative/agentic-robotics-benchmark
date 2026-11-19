# V7 fixed suite

All eight seeds are unchanged from v6, `cb7d6acb9bd03f3f0ac4b0587bfd25c2702c7c7a`.
Seed bits 14 and 15 set the two route choices; bit 33 sets launch handedness.
The launch bit was selected for coverage using seed bits alone, before physics
tests. All other draws exclude these three bits. Changing one decision bit
therefore preserves continuous geometry, gains and bollard arrangement. Flipping
the launch bit changes only the launch, not the survey; flipping the final
route bit preserves launch and first passage. Rack dimensions have the same
distribution as v6, not necessarily the same numeric values for a given seed.
There is no rejection loop or substitution based on an outcome.

| Site | Wide passage sides, west to east | Launch bank |
|---|---|---|
| 0 | South, North | North |
| 1 | South, South | South |
| 2 | North, South | North |
| 3 | North, North | South |
| 4 | North, South | South |
| 5 | South, South | North |
| 6 | North, North | North |
| 7 | South, North | South |

The middle opening forces a return toward center for all four combinations.
At each choice, either side of the short route can be walled off; the short
route itself is never visually closed. Its aperture is narrower than the
nominal opposite-thigh-axis spacing. Both alternatives are physical gaps
between identically painted collision boxes, not semantic labels.

Two unscored mirror controls (sites 0 and 3 with bit 33 flipped) test the
same geometry, survey pixels and gains with opposite-bank starts. No
counterfactual seed is sent to a controller.
