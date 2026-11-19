# Sources of racecar_cad_design_from_photos

This note says where the task's car and its other material come from. The agent never sees this folder; everything the
agent sees uses generic names and invented part numbers only.

## The car

The car of this task is one model: the benchmark owner's own CAD and scene model of a tenth-scale four-wheel-drive
autonomous race car. The owner supplied the model for the benchmark and has agreed to its public distribution in the
forms this task uses. The model is used as it is: no hidden variant, no redesign, no rebuilt geometry, no stand-in for
any component.

What the task derives from the model:

| Derived material | Where | How |
|---|---|---|
| The true solids of the 23 made parts | `reference/parts/` (and, byte-identical, `../solution/reference/parts/`) | Each solid of the model at the pose it has in the model. Parts that the model holds as meshes only are sewn from the mesh itself, triangle by triangle. |
| The given components (the rolling base assembly and 15 component types) | the payload: `base/`, `purchased/` | The model's own solids, each type written to one STEP file in a documented frame. The compute module and the antennas keep the uniform scale the model gives them. |
| The photographs | the payload: `photos/` | Renders of the model, complete, taken apart stage by stage, and part by part on a cutting mat. |
| The documents (`base/README.md`, datasheets, drawings, `parts_list.md`) | the payload | Written for this task from the model's measured geometry; none is a copy of a maker's document. |

Two treatments are applied to derived copies; the owner's model files themselves are never modified:

- Lettering is removed from two given solids (a raised maker's name on one, an engraved type designation on the
  other), so that no agent-visible file carries a name. The removed volume is the lettering alone. A third place,
  mould numbers and date marks without any name, stays as modelled in its solid and is flattened in the rendered
  mesh only.
- The made plate and the printed part exist in the model as meshes whose hole walls are polygons. Each such wall is
  put back to the conic section it approximates (an ellipse or a circle fitted to the polygon, at most 0.02 mm from
  the mesh on the plate and 0.06 mm on the printed part), so that the grader's hole detector reads the holes. Nothing
  else of those parts is changed.

In the photographs only: the two circuit boards, which carry product photographs as textures in the model, are
rendered in a plain colour, and so is the block under one of them.

Screws, nuts, washers, the ball studs with their sockets, the wiring with its plugs and the rear wing frame are in the
model and in the photographs, but in neither the given files nor the reference. The raised print layer on the
underside of the compute module's carrier board, which carries a maker's name and logo, is left out of the given
file, the photographs and the reference alike.

## Invented names

The part numbers in the agent's files (those beginning `GLH-`, `EST-`, `WNL-`, `PRD-`, `RTN-` and `SP-`) and the
components' generic names are invented for the benchmark. No platform, product or maker name appears in any file the agent sees,
in the STEP headers, in the photographs or on the given solids.

## Third-party software

- FreeCAD 1.1.4 (LGPL-2.0-or-later, https://github.com/FreeCAD/FreeCAD) is installed unmodified in the task image
  from its release AppImage, pinned by sha256 in `environment/Dockerfile`.
- The FreeCAD MCP server neka-nat/freecad-mcp at commit `78bf0230a03324b56c9a0b7f55150fb5be2a0d42` (MIT,
  https://github.com/neka-nat/freecad-mcp) is vendored unmodified in `environment/vendor/freecad-mcp/` with its
  `LICENSE`.
- The grader's Python packages are those of the benchmark's shared base image (`environment/requirements.txt`).

## Background of the photographs

The photographs are rendered with Blender Cycles on a bench in the lab room of the project's licensed art assets (the
room bundle `room_lab_v4r3`), as in the sibling CAD-from-photos tasks. The world light is the HDRI `schadowplatz_4k`
and the bench top carries the scanned material `oak_veneer_01`; both are public CC0 material by the project's records
and were already used by the sibling tasks, and the owner decided to keep them. None of these assets is shipped with the task: only the rendered
photographs are.

## Where the files are

The payload (photographs, `base/`, `purchased/`, `parts_list.md`) is served by the benchmark's asset dataset under
`tasks/ground/racecar_cad_design_from_photos/payload/`, at the commit pinned in `environment/assets.sha256`. The
hidden truth is in this folder and in `../solution/reference/` and is never part of the payload. The public source
files of the model are listed by digest in `public_blocklist.json`; a submission that contains one of them scores 0.
