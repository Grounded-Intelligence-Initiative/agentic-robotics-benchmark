# VKR-4LBA servo actuator: control table

The Control Table is a data structure used by the servo to manage the state of the device. Users can read data
registers to get information about the status of the device with Read Instruction Packets, and modify data registers
to control the device with Write Instruction Packets. The number in brackets after a register's name is its address.
Two's complement is applied for negative values.

The Control Table is divided into 2 Areas. Data in the RAM Area is reset to initial values when the power is reset
(volatile). On the other hand, data in the EEPROM Area is maintained even when the device is powered off
(non-volatile). Data in the EEPROM Area can only be written to if Torque Enable(64) is cleared to '0' (Torque OFF).
Each data in the Control Table is restored to initial values when the device is turned on. Default values in the
EEPROM area are initial values of the device (factory default settings). If any values in the EEPROM area are
modified by a user, modified values will be restored as initial values when the device is turned on. Initial values
in the RAM area are restored when the device is turned on. Data larger than 2 bytes are saved in little endian.

## Control table of the EEPROM area

| Address | Size (Byte) | Data Name | Access | Initial Value | Range | Unit |
|---|---|---|---|---|---|---|
| 0 | 2 | Model Number | R | 2,604 | - | - |
| 2 | 4 | Model Information | R | - | - | - |
| 6 | 1 | Firmware Version | R | - | - | - |
| 7 | 1 | ID | RW | 1 | 0 ~ 252 | - |
| 8 | 1 | Baud Rate | RW | 3 | 0 ~ 6 | - |
| 9 | 1 | Return Delay Time | RW | 250 | 0 ~ 254 | 2 [μsec] |
| 10 | 1 | Drive Mode | RW | 0 | 0 ~ 13 | - |
| 11 | 1 | Operating Mode | RW | 3 | 0 ~ 16 | - |
| 12 | 1 | Secondary(Shadow) ID | RW | 255 | 0 ~ 252 | - |
| 13 | 1 | Protocol Type | RW | 2 | 2 ~ 22 | - |
| 20 | 4 | Homing Offset | RW | 0 | -1,044,479 ~ 1,044,479 | 1 [pulse] |
| 24 | 4 | Moving Threshold | RW | 10 | 0 ~ 1,023 | 0.229 [rev/min] |
| 31 | 1 | Temperature Limit | RW | 70 | 0 ~ 100 | 1 [℃] |
| 32 | 2 | Max Voltage Limit | RW | 140 | 55 ~ 140 | 0.1 [V] |
| 34 | 2 | Min Voltage Limit | RW | 55 | 55 ~ 140 | 0.1 [V] |
| 36 | 2 | PWM Limit | RW | 885 | 0 ~ 885 | 0.113 [%] |
| 38 | 2 | Current Limit | RW | 910 | 0 ~ 910 | 1.0 [mA] |
| 44 | 4 | Velocity Limit | RW | 280 | 0 ~ 2,047 | 0.229 [rev/min] |
| 48 | 4 | Max Position Limit | RW | 3,072 | 0 ~ 4,095 | 1 [pulse] |
| 52 | 4 | Min Position Limit | RW | 1,024 | 0 ~ 4,095 | 1 [pulse] |
| 60 | 1 | Startup Configuration | RW | 0 | 3 | - |
| 62 | 1 | PWM Slope | RW | 255 | 1 ~ 255 | 3.955 [mV/msec] |
| 63 | 1 | Shutdown | RW | 52 | - | - |

## Control table of the RAM area

| Address | Size (Byte) | Data Name | Access | Initial Value | Range | Unit |
|---|---|---|---|---|---|---|
| 64 | 1 | Torque Enable | RW | 0 | 0 ~ 1 | - |
| 65 | 1 | LED | RW | 0 | 0 ~ 1 | - |
| 68 | 1 | Status Return Level | RW | 2 | 0 ~ 2 | - |
| 69 | 1 | Registered Instruction | R | 0 | 0 ~ 1 | - |
| 70 | 1 | Hardware Error Status | R | 0 | - | - |
| 76 | 2 | Velocity I Gain | RW | 1,800 | 0 ~ 16,383 | - |
| 78 | 2 | Velocity P Gain | RW | 50 | 0 ~ 16,383 | - |
| 80 | 2 | Position D Gain | RW | 0 | 0 ~ 16,383 | - |
| 82 | 2 | Position I Gain | RW | 0 | 0 ~ 16,383 | - |
| 84 | 2 | Position P Gain | RW | 2,000 | 0 ~ 16,383 | - |
| 88 | 2 | Feedforward 2nd Gain | RW | 0 | 0 ~ 16,383 | - |
| 90 | 2 | Feedforward 1st Gain | RW | 0 | 0 ~ 16,383 | - |
| 98 | 1 | Bus Watchdog | RW | 0 | 1 ~ 127 | 20 [msec] |
| 100 | 2 | Goal PWM | RW | - | -PWM Limit(36) ~ PWM Limit(36) | 0.113 [%] |
| 102 | 2 | Goal Current | RW | - | -Current Limit(38) ~ Current Limit(38) | 1.0 [mA] |
| 104 | 4 | Goal Velocity | RW | - | -Velocity Limit(44) ~ Velocity Limit(44) | 0.229 [rev/min] |
| 108 | 4 | Profile Acceleration | RW | 0 | 0 ~ 32,767 | 214.577 [rev/min2] 1 [msec] |
| 112 | 4 | Profile Velocity | RW | 0 | 0 ~ 32,767 | 0.229 [rev/min] |
| 116 | 4 | Goal Position | RW | - | Min Position Limit(52) ~ Max Position Limit(48) | 1 [pulse] |
| 120 | 2 | Realtime Tick | R | - | 0 ~ 32,767 | 1 [msec] |
| 122 | 1 | Moving | R | 0 | 0 ~ 1 | - |
| 123 | 1 | Moving Status | R | 0 | - | - |
| 124 | 2 | Present PWM | R | - | - | 0.113 [%] |
| 126 | 2 | Present Current | R | - | - | 1.0 [mA] |
| 128 | 4 | Present Velocity | R | - | - | 0.229 [rev/min] |
| 132 | 4 | Present Position | R | - | - | 1 [pulse] |
| 136 | 4 | Velocity Trajectory | R | - | - | 0.229 [rev/min] |
| 140 | 4 | Position Trajectory | R | - | - | 1 [pulse] |
| 144 | 2 | Present Input Voltage | R | - | - | 0.1 [V] |
| 146 | 1 | Present Temperature | R | - | - | 1 [℃] |
| 147 | 1 | Backup Ready | R | - | 0 ~ 1 | - |
| 168 | 2 | Indirect Address 1 | RW | 224 | 64 ~ 251 | - |
| 170 | 2 | Indirect Address 2 | RW | 225 | 64 ~ 251 | - |
| 172 | 2 | Indirect Address 3 | RW | 226 | 64 ~ 251 | - |
| … | … | … | … | ··· | ··· | ··· |
| 218 | 2 | Indirect Address 26 | RW | 249 | 64 ~ 251 | - |
| 220 | 2 | Indirect Address 27 | RW | 250 | 64 ~ 251 | - |
| 222 | 2 | Indirect Address 28 | RW | 251 | 64 ~ 251 | - |
| 224 | 1 | Indirect Data 1 | RW | 0 | 0 ~ 255 | - |
| 225 | 1 | Indirect Data 2 | RW | 0 | 0 ~ 255 | - |
| 226 | 1 | Indirect Data 3 | RW | 0 | 0 ~ 255 | - |
| … | … | … | … | ··· | ··· | ··· |
| 249 | 1 | Indirect Data 26 | RW | 0 | 0 ~ 255 | - |
| 250 | 1 | Indirect Data 27 | RW | 0 | 0 ~ 255 | - |
| 251 | 1 | Indirect Data 28 | RW | 0 | 0 ~ 255 | - |

## Register notes

### Drive Mode(10)

Bit 0 (0x01), Normal/Reverse Mode: [0] Normal Mode: CCW (positive), CW (negative); [1] Reverse Mode: CCW (negative),
CW (positive). If the value of Bit 0 of the Drive Mode(10) is set to 1, the rotational direction is inverted: Goal
Position and Present Position will have an inverted direction. Bit 2 (0x04), Profile Configuration: [0] velocity-based
profile, [1] time-based profile. Bit 3 (0x08), Torque On by Goal Update: [0] a given command is performed only if
Torque Enable(64) is '1'; [1] a given command is performed regardless of Torque Enable(64). Bits 1 and 4 to 7 are
unused, always '0'.

### Operating Mode(11)

| Value | Operating Mode | Description |
|---|---|---|
| 0 | Current Control Mode | The servo only controls current (torque) regardless of speed and position. |
| 1 | Velocity Control Mode | This mode controls velocity (endless turning, like a wheel mode). |
| 3 (default) | Position Control Mode | This mode controls position. Operating position range is limited by the Max Position Limit(48) and the Min Position Limit(52). This mode is ideal for articulated robots that each joint rotates less than 360 degrees. |
| 4 | Extended Position Control Mode (multi-turn) | This mode controls position; 512 turns are supported (-256 rev to 256 rev). Max Position Limit(48) and Min Position Limit(52) are not used. |
| 5 | Current-based Position Control Mode | This mode controls both position and current (torque). Up to 512 turns are supported (-256 rev to 256 rev). This mode is ideal for a system that requires both position and current control such as articulated robots or grippers. |
| 16 | PWM Control Mode (Voltage Control Mode) | This mode directly controls PWM output. |

NOTE: When the Operating Mode(11) switches to another mode, the values of the gains (Velocity PI(76, 78), Position
PID(80, 82, 84), Feedforward(88, 90)) are reset to fit the selected mode; the profile generator and the limit values
are reset as well: Profile Velocity(112) and Profile Acceleration(108) to '0'; Goal PWM(100) and Goal Current(102) to
PWM Limit(36) and Current Limit(38). When the Operating Mode(11) is Current-based Position Control Mode, the Position
PID(80, 82, 84) and PWM Limit(36) values are reset.

### Homing Offset(20)

The Homing Offset(20) adjusts the home position. The offset value is added to the Present Position(132):
Present Position(132) = Actual Position + Homing Offset(20). Unit about 0.088 deg; range -1,044,479 to 1,044,479
(-255 to 255 rev). In Position Control Mode (joint mode, less than 360 degrees), invalid Homing Offset(20) values are
ignored (valid range -1,024 to 1,024). Even if Drive Mode(10) is set to the Reverse Mode, the sign of the Homing
Offset(20) value is not reversed.

### Current Limit(38)

The Current Limit(38) indicates the maximum current (torque) output limit. The Goal Current(102) can't be configured
with any values exceeding the Current Limit(38). The Current Limit(38) is used in Torque Control Mode and Current-based
Position Control Mode; decreasing it decreases the torque of the servo. Unit about 1 mA; range 0 to 910. This servo
measures current at its input power source; results may differ from measuring the phase current of a motor.

### Min/Max Position Limit(52, 48)

The Min and Max Position Limit(52, 48) limit the maximum and minimum positions for Position Control Mode (joint mode)
within the range of 1 rotation (0 to 4,095). Goal Position(116) will also be limited by the position limit range.
These values are not used in Extended Position Control Mode and Current-based Position Control Mode. Max Position
Limit(48) and Min Position Limit(52) are only used in Position Control Mode with a single turn.

### Torque Enable(64)

Torque Enable(64) determines torque ON / OFF. Writing '1' turns on the torque, and all data in the EEPROM area is
locked. 0 (default): torque off; 1: torque on and lock the EEPROM area. NOTE: Present Position(132) can be reset when
Operating Mode(11) and Torque Enable(64) are updated. For more details, please refer to the Homing Offset(20) and
Present Position(132).

### Goal Current(102)

Use Goal Current(102) to set a desired current when the Operating Mode(11) is Torque Control Mode. Also, the Goal
Current(102) can be used to set a limit to current in Current-based Position Control Mode. Goal Current(102) can not
be set larger than the Current Limit(38). Unit about 1 mA. Applying high current to the motor for a long period of
time might damage the motor.

### Goal Position(116)

The Goal Position(116) sets the desired position. From the front view of the servo, CCW is an increasing direction,
whereas CW is a decreasing direction. Unit 0.088 deg/pulse; 1 rev = 4,096 counts.

| Mode | Values |
|---|---|
| Position Control Mode | Min Position Limit(52) to Max Position Limit(48) (initial value 0 to 4,095) |
| Extended Position Control Mode | -1,048,575 to 1,048,575 (-256 rev to 256 rev) |
| Current-based Position Control Mode | -1,048,575 to 1,048,575 (-256 rev to 256 rev) |

NOTE: When turning off the power supply or changing Operation Mode on Extended Position Control Mode, the value of
Present Position is reset to the absolute position value of single turn.

### Present Position(132)

The Present Position(132) indicates the present position. For more details, see the Goal Position(116).

NOTE: Present Position(132) represents a 4 byte continuous range from -2,147,483,648 to 2,147,483,647 when Torque is
turned off regardless of Operating Mode(11).

However, Present Position(132) will be reset to an absolute position value within one full rotation in the
following cases:

- When the Operating Mode(11) is changed to Position Control Mode.
- When torque is turned on in Position Control Mode.
- When the actuator is turned on or when rebooted using a Reboot Instruction.

Note that a Present Position(132) value that has been reset to the absolute value within a single rotation will still
be affected by the configured Homing Offset(20) value.
