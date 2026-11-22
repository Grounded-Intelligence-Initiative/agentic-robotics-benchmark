# The cameras' renderer and what it shows

Since 2026-11-21 the aircraft's two cameras are drawn by MuJoCo 3.16's Filament renderer on Mesa's lavapipe (Vulkan
on the CPU) instead of MuJoCo's own OpenGL renderer. The simulation, the cameras' poses and optics, the sensors, the
scoring and the eight graded sites are unchanged. This note records what a picture shows now, what was measured, and
how to repeat the measurements. The module is `tests/runtime/rescue_render.py` (a byte copy in
`environment/setup/runtime/`); its numbers are in `rescue_look.json` beside it.

## What is drawn

- **Every geom of the site's model, where the simulation has it, in its own shape and size.** Nothing of the model is
  left out, moved or resized (a moving body's boxes are drawn 0.5 mm smaller per side, so that a part at rest against
  a standing surface never lies in that surface's plane).
- **Surfaces instead of plain colours.** Walls: plaster with a darker dado band and timber skirting. Ground floor:
  worn concrete. Upper floor: tiles. Ceilings: plaster. Painted steel and panels: the model's colour under a wear
  pattern. Floor markings: worn paint in the model's colour. Signs: the model's own pictures. The litter, its
  fitting, the mannequin, the collets, the tool stands and the aircraft: the model's shapes and colours.
- **The wayfinding colours are the model's own values.** The look that people see in the task's videos tones them
  down and paints the dado blue downstairs and green upstairs; in the cameras the dado is nearly grey, because blue and
  green are the hue families of the cargo door and of the bays and decks.
- **Drawn only, not in the simulated model** (the same rule at every site, read from the model's wall boxes):
  - a glazed window in every wall segment of the outer hull (the wing's west, east and north walls on both storeys
    and its staging row's west and east walls; not behind the hoist guides);
  - a timber frame round every ordinary doorway: boards on the walls beside the opening, a head at 2.10 m, and wall
    above the head up to the slab. The opening is drawn at its structural width. Structurally a doorway is open up
    to the slab; the pictures show it closed above 2.16 m. The instruction says so.
- **Not drawn in the cameras, although the task's videos show them:** furniture and props, people, paper and dust,
  ceiling lamps, the redesigned lifting frame and the blanket on the litter, the doors and windows of the receiving
  rooms, a second aircraft. The fitting on the litter is the model's flange, stem and cover.
- **Light.** One image light and ambient occlusion; no lamp and no shadow map. The image light is the same in every
  compass direction (a brighter tone from above, a darker one from below), because every room has its windows, if
  any, on other sides. Its level and the six surface maps' gains were fitted so that floors, walls and ceilings of
  both storeys come out as in path-traced stills of the look through the same two cameras at three states of a
  development flight: after the fit the six surfaces are within 4 % of the stills per colour channel
  (display-linear). A site's tone (0.92 to 1.05, as before) scales the light.

## A picture is a function of the site and the state

- One worker process draws one site (`rescue_render.Cameras`, started by `rescue.World` and ended with it): every
  Filament object is made once per process, in one order. Filament's temporal dithering is switched off. Images are
  drawn at twice their size and averaged in linear light with integer arithmetic.
- Where two faces of different surfaces lie in one plane, look the same way and overlap (two crossing floor
  markings, a deck frame's underside in a slab's), one of them is drawn 0.37 mm proud per level, the face alone; a
  wall's top in a slab's top plane is drawn 2 mm lower. Without this, which face a pixel shows changed from one
  process to the next.
- `tests/review/camera_digests.py` repeats the check: for a seed, the aircraft over every room cell of both storeys
  on four headings (164 states, 328 images), drawn in order, drawn again in reverse order by the same process, and
  drawn again by fresh processes; `--dump` and `--expect` compare two runs' digests (two hosts).

RESULTS_DETERMINISM

## What the pictures cost

RESULTS_COST

## Information parity

RESULTS_PARITY
