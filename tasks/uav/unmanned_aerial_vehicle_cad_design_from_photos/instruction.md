# Recover a modified research quadrotor's CAD

The editable CAD for a modified indoor research quadrotor was lost. A colleague photographed the complete unit and each custom part during teardown. The unit is no longer available for measurement. Reconstruct its mechanical design in FreeCAD from those photographs and the supplied purchased-component documents.

This is one modified physical design. A public platform's nominal model does not describe its dimensions, mounts or stock. Infer the design from the supplied evidence.
Unchanged published CAD or mesh exports must not be included in the submission; exact copies are rejected.

## How to work

Use the FreeCAD MCP server to inspect and edit the running FreeCAD session. Its `execute_code` tool runs Python inside that session. Other tools remain available for image measurement, calculations and scripts. The Python environment is `/opt/venv/bin/python3`; the native headless CAD executable is `/opt/freecad/usr/bin/freecadcmd`.

Save early and periodically. You have six hours. A single model response is limited to 64k output tokens; do not wait until the end to create the deliverable.

## Given files

- `photos/P01_1.jpg` through `photos/P18_4.jpg`: four perspective views per tagged manufactured part. Views 1–2 show one side and views 3–4 the turned-over part. Paper tags identify instances, including repeated parts.
- `photos/assembled_1.jpg` through `photos/assembled_6.jpg`: the same unit assembled in its reference configuration. Propellers, wiring, screws and straps are omitted to expose the rigid structure. The rotor operating envelope is specified in the motor documentation.
- `purchased/`: fictional vendor STEP models, keep-out models, dimensioned drawings and datasheets. Use these bought components unchanged; infer their installed placements from the photographs.

The cutting mat has a 10 mm grid. The checker card has ten by seven squares, each 12 mm wide. The camera is perspective and uncalibrated; filenames do not encode its pose. Materials are carbon laminate at 1.60 g/cm³ for P01–P03 and printed PLA at 1.24 g/cm³ for P04–P18. Infer stock thicknesses and geometric dimensions from the evidence. Bought-component masses are in their datasheets.

## Deliverable

Save `/home/user/submission/drone.FCStd` with:

1. One valid, closed B-rep solid in a PartDesign Body for each tag P01 through P18. Set each body's **Label** to exactly its paper tag. Do not merge distinct tagged parts or provide meshes in place of solids.
2. The purchased components as separate shape objects: four MTR-29 motors, one LDR-60 sensor, one CAM-38 camera, one CPU-70 compute module, one BAT-130 battery and four POST-D spacers. Imported geometry must agree with the vendor model. Labels may include the part number and an instance suffix.
3. One native FreeCAD Assembly containing all these parts, with a grounded base P01 and Fixed joints connecting the rigid structure and mounted components. Motor shafts do not need revolute joints: their axes are inferred from their physical geometry. Do not model a serial arm mechanism.
4. The assembly saved in the photographed assembled configuration. Coordinate origin and overall rigid orientation are arbitrary; units must be millimetres. Do not include an artificial ground, cutting mat, photogrammetry scene, detached alternatives or duplicate solids in the assembly.

No JSON description of the assembly is required or used for scoring. Only the saved document is evaluated. Screws, wiring, straps and rotating propeller blades are outside the modelling scope.
Any unrecognised extra solid is included in clearance checks and in mass calculations at the PLA density.

## How it is graded

The hidden verifier reads the document using FreeCAD, exports the actual world-coordinate solids and reads the native joints. It aligns the complete assembly using P01 with a rigid transformation only. It does not resize the model or move individual parts into place.

| Criterion | Weight | What matters |
|---|---:|---|
| Manufactured shape | 0.30 | Whole-part surface accuracy and solid overlap at the installed placement, including stock thickness |
| Functional features | 0.25 | Mount bores, slots, pockets, cut-outs and stepped retaining geometry |
| Installation | 0.20 | One-to-one motor/sensor/component positions and physical axes; connection to the grounded rigid assembly |
| Mass properties | 0.10 | Total mass, centre of mass and principal moments of inertia |
| Component fit | 0.10 | Contact with the supporting parts and freedom from solid penetration |
| Rotor clearance | 0.05 | Four complete swept rotor discs clear the frame, mounted components and each other |

An unreadable document, an invalid or non-solid assembly shape, a custom part outside a PartDesign Body, duplicate custom-part labels, or a missing P01 cannot be evaluated and receives zero. Other missing parts receive zero for their own contributions. Approximate work receives graded partial credit.

Surface error receives full credit at 0.35 mm and fades linearly to zero at 5 mm; solid overlap complements that surface term. Component position error receives full credit at 0.6 mm and zero at 12 mm. Axis error receives full credit at 0.5° and zero at 12°. Functional-region overlap rises from zero at IoU 0.25 to full at 0.95. Mass error has a 2% full-credit band, centre-of-mass error a 1 mm band and inertia a 4% band. Empty space and disconnected fragments are not substitutes for a model: shape-overlap floors rise from IoU 0.15 to 0.55 and limit feature, installation, fit, mass and clearance credit.

Motor shaft +z and camera optical +x follow the vendor drawings. Each rotor's swept disc has a 65 mm radius, is centred 26 mm above its motor's mounting face and has a 4 mm total thickness. Internal blind mounting holes are part of the purchased component, not empty space that you must fill.

The choice of shell or MCP tools does not affect your score. Geometry and assembly correctness do.
