# The hand: documentation set

Documentation of the lab's four-digit robot hand (three fingers and a thumb, 13 servo actuators VKR-4LBA, four
tactile modules), its use in the kitting cell, and the acceptance of its grip controller. Start with
`acceptance/acceptance_spec.md` (what the controller must do and how it is accepted), then `controller/` (the
interface), `servo/` and `tactile/` (the servos, the bus and the tactile frames), `hand/` and `unit/` (the hand, a
unit's own record and how units differ), `cell/` and `specimens/` (where and what it grasps). `makers/` holds the
hand makers' own notes, test procedures and results, as they wrote them; links, local paths and commit ids were removed
before hand-over.

## acceptance/

- `acceptance/acceptance_spec.md`: the acceptance specification: the run, calls and budgets, events that end a shift, job losses, scores, pass conditions

## controller/

- `controller/check_controller.py`: the self-test of the interface against a toy plant
- `controller/controller_interface.md`: the controller interface: calls, formats, budgets, allowed imports
- `controller/example_controller.py`: an example controller (it only opens the hand)
- `controller/toy_plant.py`: the toy plant of the self-test (13 toy servos; not the hand's current law)

## servo/

- `servo/bus_and_host.md`: the servo bus and the cell's host runtime: the cycle, reads, writes and their order
- `servo/performance_curve.csv`: the servo's performance curve, transcribed from the vendor's graph
- `servo/servo_manual.md`: the servo vendor's manual page (specifications, control table, register descriptions)

## tactile/

- `tactile/parameter_table.md`: the tactile module vendor's parameter table (transcribed)
- `tactile/tactile_interface.md`: the tactile modules: what the controller receives, byte encoding, zero, scale

## hand/

- `hand/kinematics_sheet.md`: the hand's joints, links, four-bar law and pad frames
- `hand/meshes/README.md`: the links' meshes
- `hand/meshes/index_abad.stl`: mesh of the link index_abad (binary STL)
- `hand/meshes/index_dip.stl`: mesh of the link index_dip (binary STL)
- `hand/meshes/index_pip.stl`: mesh of the link index_pip (binary STL)
- `hand/meshes/index_pitch.stl`: mesh of the link index_pitch (binary STL)
- `hand/meshes/index_rocker.stl`: mesh of the link index_rocker (binary STL)
- `hand/meshes/middle_abad.stl`: mesh of the link middle_abad (binary STL)
- `hand/meshes/middle_dip.stl`: mesh of the link middle_dip (binary STL)
- `hand/meshes/middle_pip.stl`: mesh of the link middle_pip (binary STL)
- `hand/meshes/middle_pitch.stl`: mesh of the link middle_pitch (binary STL)
- `hand/meshes/middle_rocker.stl`: mesh of the link middle_rocker (binary STL)
- `hand/meshes/palm.stl`: mesh of the link palm (binary STL)
- `hand/meshes/ring_abad.stl`: mesh of the link ring_abad (binary STL)
- `hand/meshes/ring_dip.stl`: mesh of the link ring_dip (binary STL)
- `hand/meshes/ring_pip.stl`: mesh of the link ring_pip (binary STL)
- `hand/meshes/ring_pitch.stl`: mesh of the link ring_pitch (binary STL)
- `hand/meshes/ring_rocker.stl`: mesh of the link ring_rocker (binary STL)
- `hand/meshes/thumb_ip.stl`: mesh of the link thumb_ip (binary STL)
- `hand/meshes/thumb_mp.stl`: mesh of the link thumb_mp (binary STL)
- `hand/meshes/thumb_roll.stl`: mesh of the link thumb_roll (binary STL)
- `hand/meshes/thumb_side.stl`: mesh of the link thumb_side (binary STL)
- `hand/taxel_maps/README.md`: the taxel maps of the two module types
- `hand/taxel_maps/finger_module.csv`: taxel map of the fingertip tactile module
- `hand/taxel_maps/thumb_module.csv`: taxel map of the thumb tactile module

## unit/

- `unit/example_record.json`: the record of another unit
- `unit/range_sheet.md`: how a delivered unit may differ from the documents
- `unit/record_format.md`: the format of a unit's own record

## cell/

- `cell/cell_data.json`: the cell data exactly as setup receives it
- `cell/cell_manual.md`: the kitting cell: frames, cell data, taught grasp, phases, the arm's motion, tickets
- `cell/example_shift.json`: the shift sheet of a shift of another unit (tickets and phase times)
- `cell/profiles.json`: the arm's transfer profiles
- `cell/shift_sheet_format.md`: the shift sheet format
- `cell/taught_grasp.json`: the taught grasp (joint angles, deg)

## specimens/

- `specimens/specimen_sheets.json`: the specimen sheets, machine-readable
- `specimens/specimen_sheets.md`: the specimen sheets: classes, size codes, mass ranges, squeeze limits, finishes, label rule

## makers/

- `makers/endurance_analysis.md`: the makers' analysis and processing record of the endurance run
- `makers/handoff.md`: the makers' hand-off notes of their reliability campaign
- `makers/impact_results.md`: the makers' draft results of the impact test
- `makers/sop_endurance.md`: the makers' procedure of their 12 h endurance run
- `makers/sop_impact.md`: the makers' procedure of their pendulum impact test
