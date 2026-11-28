# The servo bus and the cell's host runtime

The 13 servo actuators VKR-4LBA share one bus (the bus distribution board BWD-RS8C in the palm, the
power and bus hub board JQT-NLP9 with its USB bus converter) at 4 Mbps, Protocol 2.0, Return Delay Time 0.
The controller does not talk to the bus itself: the cell's host runtime runs the bus cycle, and the controller
chooses what is read and what is written. The registers are those of the vendor's control table
(`servo_manual.md`); their keys in the observation and the reply are the vendor's data names in lower case with
underscores (Present Current -> `present_current`).

## 1. The cycle

`setup` returns `{"period_ms": P, "reads": [[address, length], ...]}`:

- P: an integer from 5 to 20 (ms);
- `reads`: 1 to 4 blocks, each inside the RAM area of the control table (address >=
  64, address + length <= 148). Each block is read from all 13 servos
  in every cycle with one sync read.

The effective period T (ms) of the cycle follows from the bus traffic: 1.5 ms for the cycle's sync writes
and, for each read block, 1 ms of converter turnaround plus a 27-byte request and
13 status packets of (11 + length) bytes at 2.5 us per byte:

  T = max(P, ceil(1.5 + sum over the blocks of (1 + (27 + 13 x (11 + length)) x 0.0025)))

Examples: one block [126, 10] (Present Current to Present Position) at P = 5: T = 5 ms; four blocks of 10 bytes at P = 5: T = 9 ms; one block [64, 84] (the whole RAM area) at P = 5: T = 6 ms; one block [126, 10] at P = 10: T = 10 ms.

Cycle k starts at tick t_k = k x T (ticks of 1 ms, the servos' 1000 Hz control loop). At t_k the
previous step's writes are sent; they take effect at tick t_k + 1. The registers given
to step k are sampled at tick t_k + 2. Time on the bus is counted in cycles: a step's CPU
time decides whether it is late (below), its wall-clock time does not, within the limits of the acceptance
specification.

- **Late steps**: a step that uses more CPU than T is late by floor(cpu / T) cycles: those cycles get no observation
  and no command (the servos keep their registers), and the next observation's `late` gives the number of cycles
  skipped.
- A step that raises sends no command that cycle.

## 2. What a step reads

The observation (`../controller/controller_interface.md`) carries, under `regs`, every register of the table below that
lies wholly inside one of the read blocks, by its key: a list of 13 integers in servo id order, signed
where the vendor's table uses two's complement. Every register of the RAM area at 64 to
147 reads a value.

| address | size | data name | key | access | signed | unit | printed range |
|---|---|---|---|---|---|---|---|
| 64 | 1 | Torque Enable | `torque_enable` | RW | no | - | 0 ~ 1 |
| 65 | 1 | LED | `led` | RW | no | - | 0 ~ 1 |
| 68 | 1 | Status Return Level | `status_return_level` | RW | no | - | 0 ~ 2 |
| 69 | 1 | Registered Instruction | `registered_instruction` | R | no | - | 0 ~ 1 |
| 70 | 1 | Hardware Error Status | `hardware_error_status` | R | no | - | - |
| 76 | 2 | Velocity I Gain | `velocity_i_gain` | RW | no | - | 0 ~ 16,383 |
| 78 | 2 | Velocity P Gain | `velocity_p_gain` | RW | no | - | 0 ~ 16,383 |
| 80 | 2 | Position D Gain | `position_d_gain` | RW | no | - | 0 ~ 16,383 |
| 82 | 2 | Position I Gain | `position_i_gain` | RW | no | - | 0 ~ 16,383 |
| 84 | 2 | Position P Gain | `position_p_gain` | RW | no | - | 0 ~ 16,383 |
| 88 | 2 | Feedforward 2nd Gain | `feedforward_2nd_gain` | RW | no | - | 0 ~ 16,383 |
| 90 | 2 | Feedforward 1st Gain | `feedforward_1st_gain` | RW | no | - | 0 ~ 16,383 |
| 98 | 1 | Bus Watchdog | `bus_watchdog` | RW | no | 20 [msec] | 1 ~ 127 |
| 100 | 2 | Goal PWM | `goal_pwm` | RW | yes | 0.113 [%] | -PWM Limit(36) ~ PWM Limit(36) |
| 102 | 2 | Goal Current | `goal_current` | RW | yes | 1.0 [mA] | -Current Limit(38) ~ Current Limit(38) |
| 104 | 4 | Goal Velocity | `goal_velocity` | RW | yes | 0.229 [rev/min] | -Velocity Limit(44) ~ Velocity Limit(44) |
| 108 | 4 | Profile Acceleration | `profile_acceleration` | RW | no | 214.577 [rev/min2] 1 [msec] | 0 ~ 32,767 |
| 112 | 4 | Profile Velocity | `profile_velocity` | RW | no | 0.229 [rev/min] | 0 ~ 32,767 |
| 116 | 4 | Goal Position | `goal_position` | RW | yes | 1 [pulse] | Min Position Limit(52) ~ Max Position Limit(48) |
| 120 | 2 | Realtime Tick | `realtime_tick` | R | no | 1 [msec] | 0 ~ 32,767 |
| 122 | 1 | Moving | `moving` | R | no | - | 0 ~ 1 |
| 123 | 1 | Moving Status | `moving_status` | R | no | - | - |
| 124 | 2 | Present PWM | `present_pwm` | R | yes | 0.113 [%] | - |
| 126 | 2 | Present Current | `present_current` | R | yes | 1.0 [mA] | - |
| 128 | 4 | Present Velocity | `present_velocity` | R | yes | 0.229 [rev/min] | - |
| 132 | 4 | Present Position | `present_position` | R | yes | 1 [pulse] | - |
| 136 | 4 | Velocity Trajectory | `velocity_trajectory` | R | yes | 0.229 [rev/min] | - |
| 140 | 4 | Position Trajectory | `position_trajectory` | R | yes | 1 [pulse] | - |
| 144 | 2 | Present Input Voltage | `present_input_voltage` | R | no | 0.1 [V] | - |
| 146 | 1 | Present Temperature | `present_temperature` | R | no | 1 [℃] | - |
| 147 | 1 | Backup Ready | `backup_ready` | R | no | - | 0 ~ 1 |

## 3. What a step writes

The reply's `write` maps a field to a list of 13 entries in servo id order, each an integer or null (null:
not written). The fields:

| field | address | size | area | printed range | allowed values |
|---|---|---|---|---|---|
| `torque_enable` | 64 | 1 | RAM | 0 ~ 1 | 0 to 1 |
| `operating_mode` | 11 | 1 | EEPROM | 0 ~ 16 | 0 to 16 |
| `position_d_gain` | 80 | 2 | RAM | 0 ~ 16,383 | 0 to 16383 |
| `position_i_gain` | 82 | 2 | RAM | 0 ~ 16,383 | 0 to 16383 |
| `position_p_gain` | 84 | 2 | RAM | 0 ~ 16,383 | 0 to 16383 |
| `goal_current` | 102 | 2 | RAM | -Current Limit(38) ~ Current Limit(38) | -910 to 910 |
| `profile_acceleration` | 108 | 4 | RAM | 0 ~ 32,767 | 0 to 32767 |
| `profile_velocity` | 112 | 4 | RAM | 0 ~ 32,767 | 0 to 32767 |
| `goal_position` | 116 | 4 | RAM | Min Position Limit(52) ~ Max Position Limit(48) | -1048575 to 1048575 |

All writes of one reply take effect at the same tick, applied in this order, within a field in servo id order:

1. `torque_enable` entries equal to 0;
2. `operating_mode` (an EEPROM register): refused for a servo whose torque is still on after step 1, and refused for a
   value that is not an operating mode (0, 1, 3, 4, 5, 16). An accepted write resets that servo's position gains to the
   vendor's defaults (P 2000, I 0, D
   0), Profile Velocity and Profile Acceleration to 0, Goal PWM to the PWM Limit and
   Goal Current to the Current Limit, as the vendor's manual says;
3. the RAM fields by ascending address (`position_d_gain` 80, `position_i_gain` 82, `position_p_gain` 84, `goal_current` 102, `profile_acceleration` 108, `profile_velocity` 112, `goal_position` 116), so that gains written together with a mode change
   survive it; Goal Current is clamped to the Current Limit;
4. `torque_enable` entries equal to 1: refused for a servo stopped by its own protection (only a reboot clears that,
   and the cell has no reboot).

A refused write changes nothing and is listed in the next observation's `refused` as [field, servo id]. A value
outside its register's printed range is not refused: it is a protocol error that ends the controller
(`../acceptance/acceptance_spec.md`, section 3).

## 4. The servo firmware as observed on the bench

- The control loop runs at 1000 Hz.
- The overload protection stops a servo after about 2 s at full output (a bench observation, our estimate;
  the vendor prints no threshold).
- There is no reboot in the cell: a servo whose protection has turned its torque off stays off for the rest of the
  unit.

## 5. The tactile frames

The tactile interface board has its own serial line; the runtime passes its frames on in the observation
(`../tactile/tactile_interface.md`).
