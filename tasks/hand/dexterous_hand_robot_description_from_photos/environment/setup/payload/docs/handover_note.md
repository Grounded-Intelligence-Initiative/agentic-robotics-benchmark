# Handover note

From the partner lab, on how it will use the hand (one unit) and the files the lab sends with it.

- On arrival the partner takes the hand out of its box and switches it on directly, then mounts it on its arm.
- The partner commands the hand only through the hand's host software (`docs/host_software.md`), with the calibration
  file `config.yaml` the lab delivers, and with the host software's own settings (operating mode, gains, Goal
  Current). It does not change the servos' control tables.
- The partner does not home the hand and does not accept homing as a calibration.
- The partner pinches small parts between the thumb's pad and the index finger's pad, and it places single pads on targets.
  For such a command it moves only the servos of the digits involved; the other servos keep the position they had
  when the hand was switched on.
- The partner keeps the pinch force under a force cap of its own.
- The partner simulates the hand with the MuJoCo description the lab delivers (`robot.xml`) to plan these commands.
