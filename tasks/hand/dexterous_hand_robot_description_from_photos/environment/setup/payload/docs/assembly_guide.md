# Assembly guide (excerpt)

An excerpt of the hand designers' assembly guide: the steps that set the servos up and fit them to the printed parts,
re-issued with this task's names. Motor numbers are the servo ids of `design_table.md`.

## Set motor ID

1. Open the servo to connect the wires.
2. Connect the wires. Always push the buttons when pulling to prevent wire disconnection. Ensure the wire connection
   for the board matches the reference image.
3. Turn on the Vakrell Bus Studio software. Set the protocol to 2.0, the port to the USB serial port and the baud rate to
   1,000,000.
4. Set the motor ID and change the baud rate to 4,000,000.
5. Verify that the horn is positioned at exactly 180 degrees when correctly connected.
6. Assemble the output horn using an M2 x 6 flat head screw from the servo kit.
7. Repeat this process for the other motors, assigning IDs from 0 to 12 as shown in the attached image.

## Index, middle and ring finger

1. Assemble motor 5 to its printed part.
2. Assemble motor 6 to its printed part, ensuring the encoder value is 180 degrees and the motor aligns perfectly with
   the printed parts.
3. Bolt in the printed parts. Guide the wires through the designated paths in the printed parts to avoid pinching.
4. Connect motor 4 and motor 5 again, double-checking the output horn direction and ensuring the part alignment
   remains at 180 degrees.
5. Assemble both side plates to motor 4, verifying the encoder is at 180 degrees and all printed parts align
   correctly. Tighten the screws connecting both side plates. Verify that the assembled joint has minimal friction and
   moves smoothly.

(The motor numbers are those of the index finger; the middle finger uses motors 7, 8 and 9 and the ring finger motors
10, 11 and 12 in the same order.)

## Thumb

1. Assemble the distal part to motor 0, making sure the encoder is at 180 degrees and the parts are aligned.
2. Assemble the printed parts to motors 1 and 0, again ensuring the encoder is at 180 degrees and aligned.
3. Align motor 0 and motor 1, keeping the wire length appropriate for the position.
4. Assemble the respective part with motor 2.
5. Align motor 2 and motor 3, confirming the encoder value is at 180 degrees and the parts match the picture.

## Tactile modules

Connect the wire to each tactile module, route it through the designated path, and screw the module to the last
printed part of its digit. Bend each digit through its full range of motion to ensure enough slack for the wires.
