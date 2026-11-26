# Reference solver

The solver synthesizes a stepping motion from the public route command, heading and measured pelvis velocity. Each 20 ms it sends ten prospective G1 joint poses, joint velocities and relative yaw anchors to the released GEAR-SONIC V1 reference encoder. It returns the encoder's current 64 integer coordinates, each with 32 possible values. The environment runs the released decoder on these codes and live robot history, applies motor torques and keeps both hands open.

For walking, a 0.9 s cycle alternates foot pitch, knee flexion, ankle compensation and shoulder swing. The signed cross-track error steers the robot toward the line between its reset and destination. Forward distance and measured speed reduce the step amplitude as the goal approaches. For a right pivot, the remaining signed angle and measured yaw rate set the yaw targets. The remaining distance to the pivot center corrects the stride and lateral motion. Both routes use the same joint template and ease into the first steps. The solver reads each request independently, including repeated queries at the same tick.

## Run public practice

In the configured task environment, run `bash solution/solve.sh` to install the reference under `$ALE_HOME/submission` (default `/home/user/submission`). Then run the public practice commands.

```sh
python /home/user/practice.py --task straight_stop --seed 7
python /home/user/practice.py --task right_turn_stop --seed 7
```

Use `--distance-m` for a different forward distance or `--turn-deg` for a different negative angle. The practice output includes displacement, signed turn, path excursion, stop dwell, foot loading transitions, token changes and joint acceleration. The native physical scorer checks its stop conditions at 2 ms intervals.

## Provenance

The periodic joint references and feedback rules in `gait.py` were developed for this benchmark. They are generated numerically on each request. All learned weights used by this reference are the publicly released SONIC models identified by the environment's asset manifest. The task image supplies the original encoder and decoder. The solver requires NumPy, ONNX Runtime and the public encoder wrapper under `/opt/sonic`.
