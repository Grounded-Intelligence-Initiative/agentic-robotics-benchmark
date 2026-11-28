# Host software (excerpt)

An excerpt of the documentation and the code of the hand's host software, the designers' Python package that drives
the 13 servos over the bus (protocol 2.0, 4,000,000 baud). Everything the partner's computer does with a calibration
file is here.

## The calibration file

The calibration is saved to the host computer, not to the hand itself. Saved calibration files store only
hand-specific calibration fields keyed by motor ID, in YAML (`config.yaml`):

```yaml
motor_ids:
- 0
- 1
- 2
# ...
- 12
home_offsets:
  0: 3.169204308748572
  1: 3.1461945992939455
  2: 1.888330344497239
  # ...
  12: 0.0
joint_signs:
  0: 1.0
  1: 1.0
  # ...
  12: 1.0
```

At runtime these maps are converted into arrays ordered to match `motor_ids`. `home_offsets` are raw actuator
positions, in radians, where the software joint position is defined as zero. `joint_signs` are +1 or -1 per motor;
the default is +1 for every motor. When the file is loaded, a field may be a mapping keyed by motor ID (the ID as an
integer or as its decimal string) or a list in `motor_ids` order. The file may also hold `joint_lower_limits` and
`joint_upper_limits` (radians per motor; default -pi and +pi): the software clips every commanded joint position to
them before it converts it. Controller gains, current limits, operating mode, baud rate and the servo's unit
constants are code defaults; they are not stored in the file.

## Positions

With D = 2 pi / 4096 radians per count (4096 counts per revolution):

- Reading. The software reads Present Position(132) of each servo as a signed 32-bit count k, takes raw = k D
  (radians) and returns the joint position (raw - home_offset) x joint_sign.
- Commanding. For a joint position q (radians) the software computes raw = q x joint_sign + home_offset (radians),
  converts it to counts as `int(round(raw / D))` (Python's round: half to even) and writes that count to Goal
  Position(116) of the servo (4 bytes, two's complement).

## Start-up

A program that drives the hand first runs the software's configure step on all 13 servos: torque off; Drive
Mode(10) velocity-based profile; Operating Mode(11) 5 (current-based position control); position gains
P 800, I 0, D 500; Goal Current(102) 600 (about 1 mA per unit; at most 910); torque on. The
software's model check expects the model number 2604 of VKR-4LBA from every servo.
