# VKR-4LBA servo actuator: manual

The pages of the servo vendor's manual that concern this hand, re-issued under the servo's catalogue code
VKR-4LBA. All 13 servos of the hand are this model. The control table and its notes are on the next page,
`control_table.md`.

## Specifications

| Item | Specifications |
|---|---|
| MCU | ARM Cortex-M0+ (64 MHz, 32 bit) |
| Position sensor | contactless absolute encoder (12 bit, 360 deg) |
| Motor | coreless DC |
| Baud rate | 9,600 bps to 4 Mbps |
| Control algorithm | PID control |
| Resolution | 4096 pulse/rev |
| Operating modes | Current Control Mode; Velocity Control Mode; Position Control Mode (0 to 360 deg); Extended Position Control Mode (multi-turn); Current-based Position Control Mode; PWM Control Mode (Voltage Control Mode) |
| Weight | 27 g |
| Dimensions (W x H x D) | 19 x 35 x 22 mm |
| Gear ratio | 323.04 : 1 |
| Stall torque | 0.85 N m (at 9.0 V, 0.61 A, 1.397 N m/A); 1.03 N m (at 11.1 V, 0.80 A, 1.290 N m/A); 1.12 N m (at 12 V, 0.88 A, 1.274 N m/A) |
| No load speed | 43 rev/min (at 9.0 V); 53 rev/min (at 11.1 V); 57 rev/min (at 12 V) |
| Operating temperature | -5 to +70 deg C |
| Input voltage | 6500 to 12000 mV (recommended: 11.1 V) |
| Command signal | digital packet |
| Physical connection | TTL multidrop bus (5 V logic); TTL half duplex asynchronous serial communication (8 bit, 1 stop, no parity) |
| ID | 253 IDs (0 to 252) |
| Feedback | position, velocity, current, realtime tick, trajectory, temperature, input voltage, etc. |
| Case material | full metal |
| Gear material | full metal gear, 2 bearings |
| Standby current | 17 mA |

## Performance graph

![performance graph](performance_curve.png)

`performance_curve.png`: the vendor's performance graph (N-T curve) of the servo, drawn again from the published
graph: current (A), speed (rev/min) and efficiency (%) against the load torque (N m).

NOTE: The given stall torque rating for a servo is different from its continuous output rating, and may also differ
from its expected real world performance. Stall torque is the maximum momentary torque output the servo is capable
of, and is generally how RC servos are measured. The performance graph, or N-T curve, is measured under conditions
simulating a gradually increasing load. Generally, the maximum torque shown through performance graph testing is less
than the maximum stall torque. The actual real world performance of the servo will generally be closer to the
performance graph measurements, not the rated stall torque.

## Mass properties

The vendor's mass-properties sheet of the servo states a mass of 30 g; the specification table above
states 27 g.

## Part file

`parts/meshes/VKR-4LBA.stl` is the servo as one solid: the case, the output horn and the rear bearing (the case
screws and the connectors of the real servo are not modelled). In the frame of that file the output axis is the z axis
(x = 0, y = 0) and the output horn is on the +z face. From the front view of the servo (looking at the output horn),
counter-clockwise is the increasing direction of Goal Position(116) and Present Position(132) (Normal Mode of Drive
Mode(10), see `control_table.md`).
