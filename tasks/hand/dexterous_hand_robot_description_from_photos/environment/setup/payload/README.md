# What is here

The material of one four-digit robot hand (three fingers and a thumb on a palm; 16 hinges, 13 servo actuators
VKR-4LBA), the unit the lab built, photographed and shipped. Paths are relative to this folder (`/home/user/`).

| path | what it holds |
|---|---|
| `photos/` | `photo_01.jpg` ... `photo_10.jpg`: the hand clamped by its wrist face on a post, five postures, two views each (palm side, back side), with the bus read-out of each; `photo_11.jpg`: the hand switched off and packed in its open box; `photos.json`: per photograph its posture, view and read-out |
| `bus/` | `readouts.csv` (the read-outs: per servo id its Present Position(132) count, temperature and input voltage), `readouts_header.txt` (the session, the tool, the units), `control_tables/servo_00.txt` ... `servo_12.txt` (each servo's control table as read off the bus) |
| `parts/` | `meshes/` (one STL per made part tag P01 ... P52, per given part type and per fastener type, each in its own frame, metres), `assembly.json` (every instance: `id`, `part`, `mesh`, `group` (its sub-assembly) and `T` (its 4 x 4 placement in the palm frame at q = 0)), `bom.csv` (the bill of materials), `print_settings.csv` (the print settings of each made part) |
| `docs/servo_VKR-4LBA/` | the servo actuator's manual pages: `manual.md` (specification, performance graph, mass, its part file), `control_table.md` (the registers and their notes), `performance_curve.png` |
| `docs/` | `design_table.md` (servo ids, axes, travel, direction rule), `assembly_guide.md` (the assembly steps that fit the servos), `host_software.md` (the host software's calibration file, formula and start-up), `bench_record.md` (how the lab built, photographed and packed the hand), `handover_note.md` (what the partner does with it), `weigh_slip.md` (what the lab weighed) |
| `contract/` | `contract.md` (the contract of your files and what the grader does with them), `zero_pose.png`, `pad_definition.png`, `collision_pairs.json` (the parts and pairs of the 1 mm rule) |
| `check_submission.py`, `checker/` | the grader's own gate: `/opt/venv/bin/python3 ~/check_submission.py ~/submission` |
| `submission/` | where your files go: `robot.xml`, `assets/`, `config.yaml` |
