# Range sheet: how a delivered unit may differ from the documents

Every unit of the hand differs from the documents. This sheet gives, for each quantity in which units differ, the
range a delivered unit lies in and the basis of the range:

- **measured**: from the hand makers' measurements on their hand (`../makers/`);
- **published**: from a vendor's or the makers' published figures;
- **our estimate**: the lab's engineering estimate, where nothing was measured or published.

| quantity | applies to | range | basis |
|---|---|---|---|
| supply voltage | the unit | 11.1 to 12.4 V | published, measured |
| home-offset error of the record | each servo | -0.7 to 0.7 deg | measured |
| joint friction | each servo joint | 0.015 to 0.04 N m | measured, our estimate |
| reflected inertia | each servo joint | 0.003 to 0.0045 kg m^2 | our estimate |
| stall torque against the performance curve | each servo | within 8 % | our estimate |
| standby current | each servo | 12.6 to 16.5 mA | measured |
| temperature at the start of the first shift | each servo | 37 to 48 deg C | measured |
| tactile frame rate | the unit | 83 to 91 Hz | published, measured |

Notes:

- The supply voltage is that of the unit's bus during the acceptance; the servos report it in Present Input Voltage.
  The unit's record states the supply measured at delivery.
- The home-offset error is the difference between the home offset in the unit's record and the servo's true zero.
  The record states the homing spread of its own homing. No servo's encoder wraps inside its mounted travel.
- Joint friction and reflected inertia are those of each servo joint, at the joint.
- The stall torque is a servo's torque at full output, against the vendor's performance curve
  (`../servo/performance_curve.csv`).
- The standby current is the servo's own draw with no load (the vendor prints 17 mA).
- The start temperature is a servo's Present Temperature when the first shift opens.
- The tactile frame rate of each unit is stated in its record.
- The servo firmware's overload stop after about 2 s at full output (`../servo/bus_and_host.md`, section 4)
  is a bench observation: our estimate.
- The tactile modules' scale is uncalibrated (`../tactile/tactile_interface.md`); this sheet gives no range for it.
