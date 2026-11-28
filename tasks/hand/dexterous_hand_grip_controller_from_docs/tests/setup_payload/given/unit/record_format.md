# The unit's record

Every delivered unit of the hand comes with its own record, written when the unit was homed and checked at delivery.
The controller receives it as the first argument of `setup(record, cell)`. `example_record.json` is the record of
another unit, HU-DJWFZ, in exactly this format.

## Fields

| field | what |
|---|---|
| `format` | `"unit-record/1"` |
| `serial` | the unit's serial number, `HU-` and five characters |
| `servos` | one entry per servo, in id order 0 to 12 (below) |
| `homing_spread_deg` | the spread of the homing (deg): how far a record's home offset may be from the servo's true zero |
| `supply_v_at_delivery` | the bus supply voltage measured at delivery (V) |
| `tactile.frame_hz` | the unit's tactile frame rate (frames per second) |
| `tactile.zero_residual_steps` | per module (`thumb`: 127 values, `index`, `middle`, `ring`: 52 values each, in taxel order), the residual of each taxel's Fz zero at delivery in steps of 0.1 N: -1, 0 or +1 |

Each `servos` entry:

| field | what |
|---|---|
| `id` | the servo's bus id |
| `joint` | the joint it drives (`../hand/kinematics_sheet.md`) |
| `direction` | +1 or -1: the sign between the servo's Present Position and the joint angle |
| `home_offset` | the Present Position (counts) at the joint's zero |
| `eeprom` | the servo's EEPROM settings: `operating_mode`, `current_limit` (mA), `temperature_limit` (deg C), `shutdown` (the Shutdown bit mask), `return_delay` (the Return Delay Time register), `pwm_limit` |

The joint angle of a servo joint is q = direction x (Present Position - home_offset) x 360/4096 deg.

The servo ids and their joints:

| id | joint |
|---|---|
| 0 | thumb ip |
| 1 | thumb mp |
| 2 | thumb side |
| 3 | thumb roll |
| 4 | index pip |
| 5 | index mcp |
| 6 | index abad |
| 7 | middle pip |
| 8 | middle mcp |
| 9 | middle abad |
| 10 | ring pip |
| 11 | ring mcp |
| 12 | ring abad |

Every delivered unit has the same EEPROM settings on every servo: Operating Mode 5 (current-based position control), Current Limit 910 mA, Temperature Limit 70 deg C, Shutdown 52, Return Delay Time 0, PWM Limit 885. EEPROM settings can be written only
with the servo's torque off; the cell's runtime writes none of them except the Operating Mode
(`../servo/bus_and_host.md`).
