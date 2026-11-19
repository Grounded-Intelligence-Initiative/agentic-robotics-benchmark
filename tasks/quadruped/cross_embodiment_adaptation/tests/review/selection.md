# V8 fixed suite

All eight seeds are unchanged from v7, `7f0bc6e603fe0c715c0e77f8897a9a21b0527861`.
Seed bits 14 and 15 set the two route choices; bit 33 sets launch handedness.
The launch bit was selected for coverage using seed bits alone, before physics
tests. V8 uses bit 39 for fitting order, chosen by seed bits alone to balance
order against both rack sides and launch bank. Other draws exclude these four bits. Changing one decision bit
therefore preserves continuous geometry, gains and bollard arrangement. Flipping
the launch bit changes only the launch, not the survey; flipping the final
route bit preserves launch and first passage. Rack dimensions have the same
distribution as v6, not necessarily the same numeric values for a given seed.
There is no rejection loop or substitution based on an outcome.

| Site | Wide passage sides, west to east | Launch bank | First fitting |
|---|---|---|---|
| 0 | South, North | North | Duct |
| 1 | South, South | South | Duct |
| 2 | North, South | North | Deck |
| 3 | North, North | South | Deck |
| 4 | North, South | South | Duct |
| 5 | South, South | North | Deck |
| 6 | North, North | North | Duct |
| 7 | South, North | South | Deck |

The middle opening forces a return toward center for all four combinations.
At each choice, either side of the short route can be walled off; the short
route itself is never visually closed. Its aperture is narrower than the
nominal opposite-thigh-axis spacing. Both alternatives are physical gaps
between identically painted collision boxes, not semantic labels.

An unscored order swap changes only which wide opening contains which
fitting. A flat control removes the duct and deck but preserves the
inspection body, collision rules, map layout, gains and launch. No
counterfactual seed or simulator geometry is sent to a controller.
