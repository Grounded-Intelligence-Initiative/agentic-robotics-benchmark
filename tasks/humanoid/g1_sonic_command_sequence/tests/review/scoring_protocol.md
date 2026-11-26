# Reviewing the sequential-command verifier

Each official panel randomly orders eight left and eight right courses. A fresh 64-bit reset initializes each G1 and its decoder history. Within each direction the verifier independently stratifies the first straight distance, turn magnitude and departure distance. The current command enters the wire after its predecessor's measured stop. The solver can read the active stage clock and the measured robot state. It can read neither the course seed nor a future leg.

The verifier owns the robot and applies 64 legal integer codes for each accepted 20 ms row. The row is decoded on current proprioceptive history and drives 10 native floating-base MuJoCo steps. Both hands stay open. An episode earns one physical success only after the approach, pivot and departure have each passed in this world. Every scheduled episode remains in the denominator. The course uses the following limits.

| Stage | Goal and path | Stop |
| --- | --- | --- |
| Approach | Walk 4.5–5.5 m along the initial heading, within the 0.45 m corridor | Within 0.25 m of the goal, heading error ≤10°, upright, planar speed ≤0.15 m/s and absolute yaw rate ≤0.25 rad/s for 0.75 s, then an additional 75 qualifying rows |
| Pivot | Turn 75–105° in the revealed direction about the actual pelvis at reveal; radial excursion ≤0.45 m, wrong-direction turn ≤15° | Within 0.30 m of the measured pivot center, with the same heading, speed and dwell limits |
| Departure | Walk 1–2 m along the heading measured at the end of the pivot, within the 0.45 m corridor | Same disk and limits as the approach, with a 0.75 s hold |

The respective limits are 1,300, 450 and 650 rows. A fall, path violation, interrupted extra hold or timeout ends the course. A chunk stops on a stage switch, even if subsequent proposed rows remain. The next request exposes the newly revealed command and the native history from the end of that row.

**Branch response.** At pivot reveal, two requests have identical clocks, entire preceding trajectory, measured robot state and decoder history. They ask for opposite turn signs at the same magnitude. A duplicate estimates same-input variation. The order is private and only the assigned branch controls the robot.

**State response.** During a stage, another query presents a recent measured sample from that same course and stage. The target stays fixed in the world, and its pelvis-relative coordinates correspond to that sample. Both packets have the current time and previous accepted receipt. A duplicate estimates same-input variation. The real packet's answer drives the next row.

For each kind of query the scorekeeper subtracts the duplicate-change fraction from the alternative-change fraction. A change means that any coordinate in the first native integer row differs. The panel needs a net state rate of at least 0.10 and a net branch rate of at least 0.50. It needs at least 16 state pairs, eight branch pairs and eight duplicate pairs of each kind. Failing either gate gives zero normalized reward. The raw count still describes what the robot accomplished. An action change establishes sensitivity at the tested inputs; a single token-coordinate comparison does not measure the physical value of that change.

**Cost.** Applied body codes cost 64 FSQ scalars per executed row. An unused tail or counterfactual reply has no motor-token cost. Each new native row following an earlier row supplies 64 adjacent scalar comparisons, including across stage boundaries. A change fraction is the number of coordinates that change divided by that count. Smoothness uses all 29 body-joint velocities with 0.10 s differences across the entire continuous episode. Completion time includes all three stages and the extra standing hold. The verifier reports time and motion measurements alongside the primary strict completion count.

The [registered calibration](anchor_protocol.json) defines the four null-anchor measurements and two fresh final gates. [compute_anchor.py](compute_anchor.py) recomputes the normalizer from the original ALE oracle receipts. [check_controls.py](check_controls.py) reproduces same-prefix physical courses and tests an exact clock-based tape under both turn signs using the original encoder, decoder and public seed 7.
