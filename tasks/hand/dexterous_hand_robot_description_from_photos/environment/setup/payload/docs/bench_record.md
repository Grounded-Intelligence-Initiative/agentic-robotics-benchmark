# Bench record

The lab's record of this hand (one unit), from its build to its shipping.

1. Build. The hand was built from the designers' files: the made parts of `parts/` printed with the settings of
   `parts/print_settings.csv`, the given parts and fasteners of `parts/bom.csv`, assembled as `parts/assembly.json`
   places them. The servo ids were set as the assembly guide says (`docs/assembly_guide.md`, the ids of
   `docs/design_table.md`). The guide's checks that a horn or an encoder is at 180 degrees were skipped on every servo:
   each output horn was fitted where its servo happened to stand.
2. Calibration. The host software kept the hand's calibration file on the bench PC. The bench PC was reinstalled
   afterwards; the calibration file was not kept, and no homing record exists.
3. Photo session. The hand was clamped by its wrist face on a post in the photo cove and switched on once, at the start
   of the session; it stayed on until the end of the session: no reboot, no change of operating mode (mode 5
   throughout, `bus/control_tables/`). For each of the five postures the torque was switched off, the hand was posed by
   hand and the torque was switched on again to hold it; then the two photographs of that posture were taken, each with
   one read-out of the 13 servos (`bus/readouts.csv`; `photos/photos.json` names the read-out of each photograph).
4. Packing. After the session the hand was switched off, taken off the post and laid palm up in the foam cut-out of its
   box; `photos/photo_11.jpg` shows it so, with the box still open. The box was then closed and shipped to the
   partner.
