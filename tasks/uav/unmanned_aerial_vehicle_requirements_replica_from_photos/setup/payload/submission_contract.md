# Submission contract
Use the schema files as the machine-readable authority. JSON examples below describe keys, not measured answers.
`requirements.yaml` has `facets` (list of `{id, value, unit, sources}`) and `derived` (mapping from quantity ID to
`{value, unit, sources}`). Sources are arrays of exact IDs, for example `["B-07", "DS-03", "LR-B"]`.
Facet IDs must be unique. Report these facets (units in parentheses):
- `replica_count` (count)
- `core_policy` (text)
- `control_stack` (text)
- `payload_mass` (kg)
- `mission_duration` (s)
- `auxiliary_power` (W)
- `tip_clearance` (mm)
- `reserve_fraction` (1)
- `delivered_lot` (text)
- `battery_series` (count)
- `motor_mount` (mm)
- `old_envelope_status` (text)

Text facet vocabulary: core_policy="retain"; control_stack="PX4/ROS2";
delivered_lot="A" or "B"; old_envelope_status="current" or "superseded".
For each quantitative facet report its requirement value, not a paraphrase. Reference at least one source that
actually establishes it. Facet credit requires the right value, unit and a supporting source; aggregation is F1
against the required facets, so extra unsupported facets reduce precision. Numeric facet tolerances: payload
0.001 kg; mission 0.1 s; auxiliary power 0.1 W; clearance 0.01 mm; reserve 0.001; mount 0.1 mm; counts exact.

Required derived IDs (evaluate on YOUR chosen BOM):
- `all_up_mass_kg` (kg)
- `cg_x_mm` (mm)
- `cg_y_mm` (mm)
- `cg_z_mm` (mm)
- `battery_peak_a` (A)
- `minimum_voltage_v` (V)
- `mission_energy_wh` (Wh)
- `usable_energy_wh` (Wh)
- `inertia_xx_kg_m2` (kg.m2)
- `inertia_xy_kg_m2` (kg.m2)
- `inertia_xz_kg_m2` (kg.m2)
- `moment_peak_x_nm` (N.m)
- `inertia_yy_kg_m2` (kg.m2)
- `inertia_yz_kg_m2` (kg.m2)
- `moment_peak_y_nm` (N.m)
- `inertia_zz_kg_m2` (kg.m2)
- `moment_peak_z_nm` (N.m)
- `thrust_peak_FL_n` (N)
- `esc_rms_FL_a` (A)
- `thrust_peak_FR_n` (N)
- `esc_rms_FR_a` (A)
- `thrust_peak_RL_n` (N)
- `esc_rms_RL_a` (A)
- `thrust_peak_RR_n` (N)
- `esc_rms_RR_a` (A)
Missing derived quantities receive zero credit for that quantity. Compare each against the actual unit with your
BOM; relative error <=3% receives full credit, >=15% receives zero, linear between. Denominator floors are 5 mm
for CoM, 0.00001 kg.m2 for inertia, 0.01 N.m for moments and 0.2 in the stated unit for other quantities.
Inertia quantities are signed entries of the complete CoM inertia matrix (off-diagonal entries include the minus
sign), not unsigned products of inertia. Moment quantities are the maximum absolute required rotor moment about
the shaft midpoint over the complete duty, including gust and offset-CoM terms.

`bom.json` has `motors` mapping FL/FR/RL/RR to SKUs, plus `propeller`, `esc`, `battery` SKU strings.
`kinematic_sketch.json` has `dimensions_mm`, mapping each required key to `{nominal, tolerance, sources}`,
and `spin`, mapping FL/FR/RL/RR to CW/CCW. Tolerance must be 0.1..3 mm. It documents the fabrication intent;
credit compares nominal to truth (<=0.5 mm full, >=3.5 mm zero, linear between), so broad tolerances earn no shortcut.
Dimensions contribute 95% of sketch credit, spin mapping the remaining 5%.

Dimension definitions:
- motor_x_mm, motor_y_mm: positive half-spans of the shaft rectangle, NOT diagonal wheelbase.
- frame_x_mm, frame_y_mm, frame_z_mm: bounding extents of the retained core only, including its original guards
  and feet; excluding the added cage, scanner, ballast pod, and stand-off posts. See BL-01/02.
- guard_inner_mm: diameter of the smallest shaft-centred circle wholly inside every guard's inner opening.
- motor_mount_mm: hole-to-hole pitch in the square mount.
- battery_bay_x_mm, battery_bay_y_mm, battery_bay_z_mm: internal clear cage dimensions in vehicle axes.
- battery_x_mm, battery_y_mm, battery_z_mm: cage interior centre relative to shaft-plane origin.
- payload_x_mm, payload_y_mm, payload_z_mm: ballast pod box centre relative to that origin.
- sensor_x_mm, sensor_z_mm: scanner box centre. Its y is zero.
- payload_box_x_mm, payload_box_y_mm, payload_box_z_mm: external payload box extents.
- sensor_box_x_mm, sensor_box_y_mm, sensor_box_z_mm: external scanner box extents.

To call `procedure.evaluate(bom, geometry, catalogue, duty)`, reconstruct the numeric inputs from the evidence.
Geometry consists of `dimensions_mm` (plain numbers; add esc_x_mm=0, esc_y_mm=0, esc_z_mm=-8), `payload_mass_kg`,
and `fixed_components`, a list of `{name,mass_kg,position_mm:[x,y,z],box_mm:[x,y,z]}` for dry_core and sensor.
Reconstruct the final dry_core mass from BL-01 before calling the procedure; the calibration master is not the
photographed unit.
Catalogue keys: motors, props, escs, batteries, lots, battery_lot, esc_lot. Each product dictionary is indexed by SKU.
Product keys and SI units are named explicitly in the public Python function; vendor pages supply all values.
Catalogue `usd` is the unit price from Q-01. Lot dictionaries contain the retained factors computed by B-06.
Duty is a dictionary with g, reference_voltage_v, aux_power_w, usable_fraction, tip_clearance_mm,
reaction_signs, states, eco_usd and replicas. Each state has name, seconds, az_m_s2, ax_m_s2, omega_rad_s,
alpha_rad_s2, gust_n (three-vector in N).
`procedure.passing_designs` can enumerate the quoted choices on YOUR reconstructed inputs. There is no hidden
measurement query. The submitted deliverables do not include those reconstructed helper dictionaries.

All five criteria use the actual geometry and delivered lot. B-10 is all-or-nothing acceptance; B-11 defines economy.
Structure and submission-integrity failures multiply the total by zero. The grader executes no submitted code.
