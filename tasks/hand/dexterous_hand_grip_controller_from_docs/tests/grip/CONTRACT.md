# grip: the runtime contract between the grader and the hand's grip controller (binding)

This file is the single source of truth for every format and number the controller sees. The grading code (the
packages contract, plant, world and grader) implements it; the controller author's documents state it in a vendor's,
a maker's or a cell integrator's words. Change both or neither. Every number below is also in contract/constants.py
(the list at the end of this file is checked against it by a test); the data formats are the JSON schemas in
contract/schemas/, each with one example.
Units: the grading side is in SI; registers are in their raw units (counts of 360/4096 deg, mA, 0.229 rpm, 0.1 V,
1 deg C, 0.113 % of PWM); the documents use degrees and millimetres. Times in the formats are integer ms since the
hand-over unless a key says otherwise (the tactile times are float ms).

## 1. Frames, names, servos

- **Hand frame H**: x across the palm, y from the wrist towards the fingertips, z from the palmar side (-) to the
  back (+). The palm is fixed in H; the cell's nests move in H while the arm carries the hand. Every pose is a
  position in m and a unit quaternion [w, x, y, z]. The slip and rotation of a specimen are measured in H.
- **Cell frame**: the pick nest's frame at the pick pose, z up. The arm's motion (section 9) is given in it.
- **Servos**: 13 servo actuators on one bus, ids 0-12: 0 thumb ip, 1 thumb mp, 2 thumb side, 3 thumb roll, 4 index
  pip, 5 index mcp, 6 index abad, 7 middle pip, 8 middle mcp, 9 middle abad, 10 ring pip, 11 ring mcp, 12 ring abad.
  Each finger's last joint (dip) is coupled to its pip axis by a four-bar with a rocker; no servo drives it and no
  register reads it. Every list of 13 entries is in servo id order.
- **Joint angle**: q = direction x (Present Position - home_offset) x 360/4096 deg, with direction and home_offset
  from the unit's record. The record's home offsets differ from the unit's true ones by up to the homing spread the
  record states; no encoder wraps inside any servo's travel.
- **Tactile modules ("pads")**: thumb (127 taxels), index, middle, ring (52 taxels each), on the last link of each
  digit. The taught grasp is a three-pad grasp: the thumb against the index and middle fingers; the ring finger is
  parked.

## 2. What setup receives

`setup(record, cell)`:
- `record` (schemas/record.json): the unit's own record: serial, per servo its joint, direction, home offset and
  EEPROM settings (Operating Mode 5, Current Limit 910 mA, Temperature Limit 70 deg C, Shutdown 52, Return Delay 0,
  PWM Limit 885), the homing spread, the supply voltage at delivery, the tactile frame rate and each taxel's zero
  residual (in steps of 0.1 N, -1, 0 or +1).
- `cell` (schemas/cell.json): the pick and place nests (poses in H; each nest is a plate under the specimen and
  four walls 3.5 mm high round its pocket), the taught grasp (the open pre-shape, the
  postures at contact for 35, 40 and 45 mm widths, the ring finger's parking posture; joint angles in deg), the test
  lift, the arm's motion and the three transfer profiles, the specimen sheets (classes, codes, mass ranges, squeeze
  ceilings, the soft packs' travel limit, finishes, widths, the label rule), placement and the timeline numbers.
  Nothing at setup says which jobs come: each job's ticket arrives in that job's observations.

## 3. The controller

`submission/controller/controller.py` defines `class Controller` with:

| call | when | budget |
|---|---|---|
| `__init__(self, workdir)` | once per unit; `workdir` = an empty writable directory | 60 s wall |
| `setup(self, record, cell)` -> setup reply | once per unit, before the hand-over | 60 s CPU, 120 s wall |
| `step(self, obs)` -> step reply | once per cycle | the effective period of CPU (section 4) |

CPU counts every thread and every process the controller starts (also those that have exited by the time of the
reply). Imports allowed: the Python standard library and any package installed in the image, except the network
audit's refusals (names.AUDIT_*: no networking module, no ctypes / cffi, no subprocess / pty, none of the calls that
start a program as attributes of os / posix / subprocess or imported from them (os.system, os.popen, os.exec*,
os.spawn*, ...; a function of the controller's own with such a name is allowed), no import under a computed name, no
exec / eval / compile of computed code). The process runs as an
unprivileged account with no network (a seccomp filter on it and on everything it starts refuses every socket but a
local Unix one), a fresh HOME and TMPDIR, BLAS threads pinned to 1 and the same BLAS environment as the grader, and
only the submission's own files: a real folder within the size cap 80 MB, 10,000 files and folders and 12 levels;
every .py at most 4 MB, in UTF-8 / ASCII / Latin-1 and parsable (also one never imported), the static checks within
300 s CPU and 600 s wall (in a child process under its own limits). Each process of the controller: at most 7,200 s of CPU (prlimit
--cpu), no core file, oom_score_adj 1000 (the kernel ends it first when the verifier's memory runs out: a dead
controller), kept there while it runs (the root sampler of the runner account's processes kills one found below it:
a dead controller); no process-count limit. Its standard error (and descriptor 1, pointed at it) goes to a log of at
most 5 MB per unit; the rest is dropped, the log's last line counting it. After its reply to the end request the
controller's process runs the standard library's shutdown of process pools and its exit handlers and ends without
waiting for its threads; a process it started is not ended for it. The processes of the runner account are counted
once none is left or 5 s (INTERFACE.end_wait_s) after the end request, whichever comes first: every one still
there then (running, or ended but not waited for) counts as left behind.

Setup reply (schemas/setup_reply.json): `{"period_ms": 5..20, "reads": [[address, length], ...]}` with 1 to 4 blocks,
each inside the RAM area (address >= 64, address + length <= 148). Anything else is a failed setup.

## 4. The bus cycle

- The effective period T (ms) = max(period_ms, ceil(1.5 + sum over the read blocks of (1.0 + (27 + 13 x (11 +
  length)) x 0.0025))): 1.5 ms covers the cycle's sync writes; each block is one sync read of the 13 servos (a 27-byte
  request, 13 status packets of 11 + length bytes, 2.5 us per byte at 4 Mbps) plus 1.0 ms of converter turnaround.
  Examples: one block of 10 bytes at period 5 -> 5 ms; four blocks of 10 bytes -> 9 ms.
- Cycle k starts at tick t_k = k x T (1 ms ticks, the firmware's 1 kHz loop). At t_k the previous step's command is
  written; it takes effect at tick t_k + 1. The registers given to step(k) are sampled at tick t_k + 2. Simulated
  time does not move while the controller computes.
- Late steps: a step that uses more than T of CPU is late by floor(cpu / T) cycles: those cycles get no observation
  and no command (the servos keep their registers); the next observation's `late` says how many were skipped. A step
  that raises sends no command that cycle and is counted. A step over 1 s of CPU or 5 s of wall, a dead process, a
  protocol error (section 5), or more than 3,600 s of the controller's own reply wall time summed over one unit (from
  sending an observation to receiving its reply; the plant's time is not counted) ends the controller for that unit:
  the servos keep their last registers, the supervisor runs on until an event or the end, and whatever is not done
  scores 0.

## 5. The step reply (schemas/cmd.json)

`{"write": {field: [13 entries]}, "tactile_zero": [module, ...], "grasped": bool, "decline": bool}`; a null reply is
no command; missing keys read as null / false; other top-level keys are ignored.
- `write` fields and their control-table addresses: torque_enable (64), operating_mode (11), position_d_gain (80),
  position_i_gain (82), position_p_gain (84), goal_current (102), profile_acceleration (108), profile_velocity (112),
  goal_position (116). Each value is a list of 13 entries, an integer or null (null = not written); an integer is a
  JSON number written without a fraction or exponent (600.0 is not an integer; the same holds for the setup reply).
  A write key outside this list, a list not of 13, a non-integer, a boolean, or a value outside the register's printed
  range (names.REGISTERS: torque_enable 0-1, operating_mode 0-16, gains 0-16,383, goal_current -910..910, the profile
  fields 0-32,767, goal_position -1,048,575..1,048,575) is a protocol error. A reply line over 1 MiB is a protocol
  error.
- Order of application: all writes of one reply take effect at the same tick, applied in this order, within a field in
  servo id order: (1) torque_enable entries equal to 0; (2) operating_mode, refused for a servo whose torque is still
  on after step 1, and refused for a value that is not an operating mode (0, 1, 3, 4, 5, 16); an accepted mode write
  resets that servo's position gains to the table's defaults (P 2000, I 0, D 0), Profile Velocity and Profile
  Acceleration to 0, Goal PWM to the PWM Limit and Goal Current to the Current Limit; (3) the RAM fields by ascending
  address (position_d_gain, position_i_gain, position_p_gain, goal_current, profile_acceleration, profile_velocity,
  goal_position), so gains written together with a mode change survive it; Goal Current is clamped to the Current
  Limit; (4) torque_enable entries equal to 1, refused for a servo stopped by its protection (only a reboot clears
  that, and there is no reboot in this cell). A refused write changes nothing and is listed in the next
  observation's `refused`.
- `tactile_zero`: modules from thumb, index, middle, ring; each named module's zero becomes the values it reports at
  that tick, contact included ("Any contact present at call time becomes the new zero").
- `grasped`: the specimen is held; ends the closing window (section 8). `decline`: this specimen will not be carried
  (section 8); decline wins over grasped.

## 6. The observation (schemas/obs.json)

`{"cycle", "t_ms", "period_ms" (the effective period), "late", "regs", "tactile", "cell", "refused", "event"}`:
- `regs`: every register of names.REGISTERS lying wholly inside the declared blocks, by key, a list of 13 integers
  signed as the vendor's table says (two's complement: Goal PWM, Goal Current, Goal Velocity, Goal Position, Present
  PWM, Present Current, Present Velocity, Present Position, Velocity Trajectory, Position Trajectory). Every RAM
  register at 64-147 reads a value.
- `tactile`: the newest frame that arrived since the previous observation, or null: `{"seq", "t_capture_ms",
  "t_arrival_ms", "modules": {module: hex}}` (section 7).
- `cell`: `{"shift": 1 | 2, "job": 1..10 | null, "phase", "t_phase_ms", "ticket"}`; phases: handover, rest,
  await_clear, present, test_lift, lower, transfer, set_down, release, pause, end. The ticket (`{"job", "class",
  "code", "finish", "profile"}`) is shown from the job's await_clear to its rest and is null outside a job.
- `refused`: the previous reply's refused writes as [field, servo id] pairs ([] if none).
- `event`: null, or `{"kind": "crush" | "drop" | "shutdown", "shift", "t_ms"}` from the tick of an event that ended a
  shift to the end of the unit.
No pose, no force, no truth.

## 7. The tactile modules

- A frame holds, per module, a 6-byte resultant block (x, y, z as int16 little-endian, 0.01 N per LSB, in the module
  frame), then 3 bytes per taxel: Fx and Fy as int8 and Fz as uint8, 0.1 N per step, rounded. Fz below the zero wraps:
  one step below zero reads 0xFF (25.5 N); a value at or above 255 steps reads 255. Fx and Fy saturate at -128 / 127.
  The hex string of a module is that frame part, two hex digits per byte (thumb 387 bytes, a finger 162 bytes).
- The scale is uncalibrated: 0.01 N and 0.1 N per step are the modules' own units; how a reading relates to the
  true force on the pad is not documented (the makers' notes are the only evidence).
- Frames are captured every 1 / frame rate (the unit's record states its rate) and arrive 9.6 ms after capture (an
  880-byte frame at 921,600 baud, 10 bits per byte, rounded up to 0.1 ms). Each taxel's zero drifts slowly; the
  record states the zero residuals at delivery.

## 8. The timeline (the supervisor)

- Hand-over at t = 0: every servo torque off, the EEPROM of the record (Operating Mode 5), the RAM at the table's
  defaults. Shift 1 opens at t = 3.0 s. A unit has two shifts of ten jobs; between them a pause of 10.0 s; after the
  last job of shift 2, "end" for 1.0 s.
- A job: "await_clear" (the ticket is shown; the specimen arrives and is placed in the pick nest as soon as it
  clears every hand geometry by 2 mm; not clear within 2.0 s: the job is lost as not presented and the next job
  starts); "present" from the placement on (the closing window ends at the grasped flag or 3.0 s after the
  placement; no flag by then: "release", the specimen is removed, the job is lost as not grasped); on the flag
  "test_lift" (3.0 s); then "transfer", "set_down", "release" (a 1.0 s window), the specimen is removed, "rest"
  2.0 s at the pick pose.
- Decline: a decline raised in "present" goes to "release" at the next cycle; a decline raised in "test_lift", or a
  specimen not lifted at the end of the test lift, goes to "lower" (8 mm down in 0.5 s) and then "release". A decline
  counts when it is raised between the placement and the start of "transfer" and the specimen then rests in the pick
  nest with no pad contact 1.0 s after the release request. A flag raised in any other phase is ignored.
- After an event (section 11) the specimen is removed at that tick, the shift's remaining jobs are not run (voided),
  and the timeline goes at that tick to "pause" (in shift 1; shift 2 then starts 10.0 s later) or to "end" (in shift
  2). A servo stopped by its protection stays off for the rest of the unit. A stop during the pause voids shift 2.

## 9. The arm's motion (the controller cannot change it)

Given in the cell frame; translations are minimum-jerk moves with 6 m/s^2 acceleration peaks (a move of length d
takes sqrt(10 / sqrt(3) x d / 6) s); a tilt turns at 10 deg/s with a 0.1 s ramp at each end (it takes angle / 10 +
0.2 s).
- Test lift: up 8 mm in 0.5 s, then a 2.5 s hold.
- Transfer: up 0.15 m; tilt (profiles R30: 30 deg about the cell's x axis; P30: 30 deg about its y axis; T0: none);
  0.25 m along x; 0.25 m along y; 0.3 s still; tilt back; down 0.15 m (T0 2.04 s, R30 and P30 8.44 s).
- Set-down: down 8 mm in 0.5 s onto the place nest. Lower (after a decline in the test lift): down 8 mm in 0.5 s
  onto the pick nest.
- Tracking: the accelerations the hand feels are the profile's times a factor between 0.9 and 1.1 for each job.

## 10. Specimens and labels

- Classes and codes: brittle shell (SH-1, SH-2, SH-3), soft pack (SP), full-load block (FL). Each specimen is two
  half-shells on a spring along the squeeze axis: width 35-45 mm along that axis, 50 mm across, 50 mm high, the pad
  line 10 mm below the top. Soft packs' springs: 1-10 N/mm.
- Each code prints one mass range: SH-1 0.040-0.700 kg, SH-2 0.068-0.700 kg, SH-3 0.117-0.700 kg, SP 0.030-0.700 kg,
  FL 0.150-0.700 kg. Label rule: a specimen of at most 0.20 kg is to be carried, one of 0.50 kg or more is to be
  declined; nothing in between is presented.
- Finishes: matte (friction coefficient 0.40-0.55), rough (0.55-0.70).
- Placement: within 2 mm of the nest's centre in any direction, and within 3 deg of its yaw.
- Squeeze ceilings: each brittle code has a printed ceiling on the spring force; each soft pack a printed limit on
  the spring travel (the values are in the specimen sheets and in the world package's ceilings file).

## 11. Acceptance and score

The acceptance is two hand units of the task's fixed hidden set, each with its two back-to-back shifts. The units
run one after the other (unit 1, then unit 2; before each unit no process of the runner account is alive, after it
every one is ended and the unit's run folder, the account's files in the writable places and its System V IPC objects
are removed), on a verifier of 4 CPUs and 8,192 MB, of which the grader and the unit worker use at most 2,048 MB (the
reference's verify measured 1,348 MB); the verifier's time limit is 13,500 s (constants.verify_time_terms); reaching
it is a task error (no score).
- Events that end the shift (its remaining jobs score 0; a voided decline job counts as not declined): crush (a
  brittle shell's spring force above its code's ceiling for 10 ms or more, or a soft pack's travel above its limit
  for 10 ms or more, at any time the specimen exists); drop (during the transfer the specimen moves more than 20 mm in
  H, or no pad touches it for 50 ms); shutdown (any servo's torque turned off by its overheat or overload protection,
  at any time).
- Job-only losses: not presented; not grasped (no grasped flag within 3.0 s of the placement); not lifted (at the end
  of the test lift the specimen touches the pick nest or its lowest point is less than 4 mm above it); not seated (a
  carry job whose specimen is not seated on the place nest at the release request, see below); a release later
  than 1.0 s (no 50 ms without pad contact within 1.0 s of the request), or one that moves the specimen 3 mm or more
  horizontally on the nest over that second; a decline job not declined; a carry job declined.
- A carry job: q = carried x L(slip; 1.5 mm, 6.0 mm) x L(rotation; 3.0 deg, 12.0 deg) x released, with L = 1 at or
  below its first end, 0 at or above its second, linear between. Slip and rotation are the largest translation and
  rotation of the specimen in H relative to its pose at the start of the transfer, over the transfer; carried = no
  drop and the specimen seated on the place nest at the release request (inside the place nest's pocket and its
  lowest point at most 1.0 mm above the place nest's plate); released = 50 ms without pad contact within
  1.0 s of the request and less than 3 mm of horizontal motion on the nest over that second.
- A shift: S = (mean of q over the shift's carry jobs; voided and declined carry jobs count 0) x (0.7 + 0.3 J), J =
  max(0, TPR - FPR) over the shift's jobs (TPR = declined decline jobs / decline jobs, FPR = declined carry jobs /
  carry jobs).
- raw = the mean of S over the four shifts; reward = (pass conditions) x raw / anchor, anchor = the privileged
  reference's raw (tests/anchor.json). Pass conditions, each 0 or 1: controller.py a regular file defining class
  Controller; size within the cap; the static import audit; setup answers with a valid reply on every unit; at least
  90 % of each unit's cycles on time (in a privileged run no cycle is late); no late write; no process left behind;
  no network (seccomp, failing closed). A non-finite number anywhere is a task error, never a score.
- The verifier publishes aggregates only (names.METRIC_KEYS); per-job records stay with the grader.

## 12. Privileged runs (the reference only)

When the file `oracle_token` beside controller.py hashes to `tests/hidden/oracle_token.sha256`, the grader adds
`"truth"` (the unit's truth) to the setup payload and `"truth_cmd"` (the reference script's step reply) to every
observation; a wrong token is ignored and counted. The privileged setup payload also carries `"truth_reply"` (the
reference script's setup reply), which the reference returns from setup. A privileged run is immune to the late-cycle rule and to the wall
caps, so the reference's trajectory and score never depend on the host's load; its would-be-late cycles and largest
step CPU are recorded with the run's statistics. Never documented to the controller's author.

## The numbers (each equal to contract/constants.py; checked by contract/tests/test_contract.py)

NUMBERS-BEGIN
SERVO.resolution_counts_per_rev = 4096
SERVO.position_deg_per_count = 0.087890625
SERVO.velocity_rpm_per_unit = 0.229
SERVO.current_ma_per_unit = 1.0
SERVO.pwm_percent_per_unit = 0.113
SERVO.voltage_v_per_unit = 0.1
SERVO.temperature_c_per_unit = 1
SERVO.profile_acceleration_rev_per_min2_per_unit = 214.577
SERVO.pwm_limit_full = 885
SERVO.gain_divisors.position_p_gain = 128
SERVO.gain_divisors.position_i_gain = 65536
SERVO.gain_divisors.position_d_gain = 16
SERVO.gain_divisors.velocity_p_gain = 128
SERVO.gain_divisors.velocity_i_gain = 65536
SERVO.gain_divisors.feedforward_1st_gain = 4
SERVO.gain_divisors.feedforward_2nd_gain = 4
SERVO.stall_rows[0] = {"supply_v": 9.0, "torque_nm": 0.85, "current_a": 0.61}
SERVO.stall_rows[1] = {"supply_v": 11.1, "torque_nm": 1.03, "current_a": 0.8}
SERVO.stall_rows[2] = {"supply_v": 12.0, "torque_nm": 1.12, "current_a": 0.88}
SERVO.standby_current_ma = 17
SERVO.input_voltage_v = [6.5, 12.0]
SERVO.recommended_supply_v = 11.1
MODE_CHANGE_RESET.position_p_gain = 2000
MODE_CHANGE_RESET.position_i_gain = 0
MODE_CHANGE_RESET.position_d_gain = 0
MODE_CHANGE_RESET.profile_velocity = 0
MODE_CHANGE_RESET.profile_acceleration = 0
MODE_CHANGE_RESET.goal_pwm = "pwm_limit"
MODE_CHANGE_RESET.goal_current = "current_limit"
UNIT_EEPROM.operating_mode = 5
UNIT_EEPROM.current_limit = 910
UNIT_EEPROM.temperature_limit = 70
UNIT_EEPROM.shutdown = 52
UNIT_EEPROM.return_delay = 0
UNIT_EEPROM.pwm_limit = 885
FIRMWARE.loop_hz = 1000
FIRMWARE.overload_observation = "about 2 s at full output"
FIRMWARE.reboot_available = false
RANGE_SHEET.supply_v.range = [11.1, 12.4]
RANGE_SHEET.supply_v.basis = ["published", "measured"]
RANGE_SHEET.home_offset_record_error_deg.range = [-0.7, 0.7]
RANGE_SHEET.home_offset_record_error_deg.basis = ["measured"]
RANGE_SHEET.joint_friction_nm.range = [0.015, 0.04]
RANGE_SHEET.joint_friction_nm.basis = ["measured", "our estimate"]
RANGE_SHEET.reflected_inertia_kg_m2.range = [0.003, 0.0045]
RANGE_SHEET.reflected_inertia_kg_m2.basis = ["our estimate"]
RANGE_SHEET.stall_torque_vs_curve_frac.range = [-0.08, 0.08]
RANGE_SHEET.stall_torque_vs_curve_frac.basis = ["our estimate"]
RANGE_SHEET.standby_current_ma.range = [12.6, 16.5]
RANGE_SHEET.standby_current_ma.basis = ["measured"]
RANGE_SHEET.start_temperature_c.range = [37, 48]
RANGE_SHEET.start_temperature_c.basis = ["measured"]
RANGE_SHEET.tactile_frame_hz.range = [83, 91]
RANGE_SHEET.tactile_frame_hz.basis = ["published", "measured"]
TACTILE.scale = "uncalibrated"
TACTILE.resultant_bytes = 6
TACTILE.resultant_n_per_lsb = 0.01
TACTILE.taxel_bytes = 3
TACTILE.taxel_n_per_step = 0.1
TACTILE.fz_wrap_reads = 255
TACTILE.fz_saturation_reads = 255
TACTILE.fxy_saturation = [-128, 127]
TACTILE.frame_bytes = 880
TACTILE.baud = 921600
TACTILE.bits_per_byte = 10
TACTILE.arrival_after_capture_ms = 9.6
TACTILE.rezero_rule = "Any contact present at call time becomes the new zero"
TACTILE.zero_residual_steps = [-1, 1]
SPECIMENS.width_mm = [35, 45]
SPECIMENS.lateral_mm = 50
SPECIMENS.height_mm = 50
SPECIMENS.pad_line_below_top_mm = 10
SPECIMENS.soft_k_n_per_mm = [1, 10]
SPECIMENS.mass_range_kg.SH-1 = [0.04, 0.7]
SPECIMENS.mass_range_kg.SH-2 = [0.068, 0.7]
SPECIMENS.mass_range_kg.SH-3 = [0.117, 0.7]
SPECIMENS.mass_range_kg.SP = [0.03, 0.7]
SPECIMENS.mass_range_kg.FL = [0.15, 0.7]
SPECIMENS.finish_mu.matte = [0.4, 0.55]
SPECIMENS.finish_mu.rough = [0.55, 0.7]
LABEL_RULE.carry_max_kg = 0.2
LABEL_RULE.decline_min_kg = 0.5
PLACEMENT.offset_radius_mm = 2.0
PLACEMENT.yaw_deg = 3.0
PLACEMENT.clearance_mm = 2.0
NEST_GEOMETRY.wall_height_mm = 3.5
MOTION.translation_law = "minimum-jerk"
MOTION.translation_peak_mps2 = 6.0
MOTION.test_lift.rise_mm = 8
MOTION.test_lift.rise_s = 0.5
MOTION.test_lift.hold_s = 2.5
MOTION.lower.drop_mm = 8
MOTION.lower.s = 0.5
MOTION.set_down.drop_mm = 8
MOTION.set_down.s = 0.5
MOTION.transfer.up_m = 0.15
MOTION.transfer.x_m = 0.25
MOTION.transfer.y_m = 0.25
MOTION.transfer.dwell_s = 0.3
MOTION.tilt_rate_deg_s = 10
MOTION.tilt_ramp_s = 0.1
MOTION.tracking_scale = [0.9, 1.1]
PROFILE_TILT.T0 = null
PROFILE_TILT.R30.axis = [1, 0, 0]
PROFILE_TILT.R30.deg = 30
PROFILE_TILT.P30.axis = [0, 1, 0]
PROFILE_TILT.P30.deg = 30
TIMELINE.shift1_opens_s = 3.0
TIMELINE.clear_window_s = 2.0
TIMELINE.grasp_window_s = 3.0
TIMELINE.test_lift_s = 3.0
TIMELINE.lower_s = 0.5
TIMELINE.set_down_s = 0.5
TIMELINE.release_window_s = 1.0
TIMELINE.rest_s = 2.0
TIMELINE.pause_s = 10.0
TIMELINE.end_s = 1.0
TIMELINE.jobs_per_shift = 10
TIMELINE.shifts_per_unit = 2
TIMELINE.units = 2
INTERFACE.init_wall_s = 60
INTERFACE.setup_cpu_s = 60
INTERFACE.setup_wall_s = 120
INTERFACE.period_ms = [5, 20]
INTERFACE.read_blocks_max = 4
INTERFACE.read_area = [64, 147]
INTERFACE.command_effective_ticks = 1
INTERFACE.read_sample_ticks = 2
INTERFACE.step_cpu_end_s = 1.0
INTERFACE.step_wall_end_s = 5.0
INTERFACE.unit_reply_wall_s = 3600
INTERFACE.reply_line_max_bytes = 1048576
INTERFACE.on_time_min_fraction = 0.9
INTERFACE.size_cap_mb = 80
INTERFACE.entries_max = 10000
INTERFACE.depth_max = 12
INTERFACE.source_file_max_mb = 4
INTERFACE.audit_cpu_s = 300
INTERFACE.audit_wall_s = 600
INTERFACE.process_cpu_s = 7200
INTERFACE.stderr_log_max_mb = 5
INTERFACE.end_wait_s = 5
BUS.baud = 4000000
BUS.sync_write_ms = 1.5
BUS.turnaround_ms = 1.0
BUS.request_bytes = 27
BUS.status_overhead_bytes = 11
BUS.ms_per_byte = 0.0025
BUS.servos = 13
SCORE.slip_band_mm = [1.5, 6.0]
SCORE.rot_band_deg = [3.0, 12.0]
SCORE.release_no_contact_ms = 50
SCORE.release_window_s = 1.0
SCORE.release_push_mm = 3.0
SCORE.crush_run_ms = 10
SCORE.crush_impact_window_ms = 150
SCORE.drop_slip_mm = 20.0
SCORE.drop_no_contact_ms = 50
SCORE.lifted_min_above_nest_mm = 4.0
SCORE.seated_max_above_nest_mm = 1.0
SCORE.decline_rest_check_s = 1.0
SCORE.shift_base_weight = 0.7
SCORE.shift_j_weight = 0.3
SESSION.wall_s = 21600
SESSION.cpus = 4
SESSION.memory_mb = 8192
SESSION.gpus = 0
SESSION.network = false
SESSION.verifier_timeout_s = 13500
SESSION.host_runtime_mb = 2048
VERIFY_TIME.plant_cpu_s_per_sim_s = 2.0
VERIFY_TIME.unit_overhead_s = 30
VERIFY_TIME.lockdown_s = 11
VERIFY_TIME.other_fixed_s = 60
VERIFY_TIME.margin = 1.35
VERIFY_TIME.round_to_s = 100
NUMBERS-END
