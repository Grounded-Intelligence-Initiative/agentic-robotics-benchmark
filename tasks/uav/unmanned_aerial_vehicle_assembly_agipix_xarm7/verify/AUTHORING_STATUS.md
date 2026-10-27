# AGiPix-derived quadrotor assembly with xArm 7

This task asks an agent to write `assemble.py: solve(cell)` to assemble a 23-component mechanical quadrotor kit from loose parts. Only the lower chassis is clamped. The assembler is the stock MuJoCo Menagerie xArm 7 with its parallel gripper, actuator limits and frictional contacts.

The submitted program receives top/wrist RGB-D, robot encoders and a noisy wrist wrench. There is no practice cell, object-pose API or assembly-progress feedback. Each submission runs on three fresh randomized workcells. Source meshes, collision pieces and a 27-page picture manual support offline engineering and perception work.

## Mechanical scope

The kit derives from AGiPix manufacturing DXFs and propeller meshes, with explicit adaptations for this mechanical assembly cell. It is not an exact replica or a flight certification exercise. Landing struts and purchased avionics use documented engineering geometry; captive mounting posts and tiny fasteners are bundled with their components. Wiring, soldering, energization, firmware and flight are excluded.

Each correctly fastened connection contributes exactly 1/23. Safety failure zeroes that physical episode. At-rest fastening preserves the achieved pose: it does not snap or pull a part to a target. Propellers fasten to separately hinged, unpowered motor rotors. Identical physical instances are interchangeable within their component kind.

The hidden reference may inspect state for planning, but manipulates parts only through the same bounded arm actuators and actual gripper contacts. Planning copies may change pose for collision queries; the live cell never teleports a picked component or welds it to the gripper.

## Sources and runtime

- AGiPix: `SasaKuruppuarachchi/agipix`, commit `147a736b9fea7ac0e8d08c9a5fb0b90e5baa224a`, CC0-1.0.
- xArm 7: `google-deepmind/mujoco_menagerie`, commit `f3b6013629c7b94f8ee93fd977b161fa8350f9bc`, BSD-3-Clause.
- Public asset archive: dataset commit `cfc24dda62912313c553b53f26f5a2bbca23df00`, pinned by `image/assets.sha256`; the canonical fetcher verifies its digest before extraction.
- Runtime: Python 3.12, MuJoCo 3.17, eight CPU cores and 16 GiB RAM. Agent RGB uses Filament with Mesa lavapipe and a baked workshop. Registered optical depth uses the same geometry and pinhole. No GPU is required for grading.
- Owner review videos use OVRTX outside the evaluation image. Nominal design diagrams, initialized geometry checks and actual robot trajectories are labeled separately.

## Acceptance status

This is an authoring version. The complete 23-connection physical reference, its measured anchor and frontier-model calibration are still pending. `verify/anchor.json` deliberately remains null until actual engine reference measurement. It must not be populated from an initialized geometry test or partial rollout.

Completed evidence includes an anonymous build from the pinned public assets; real ALE untouched reward 0; a three-workcell unprivileged RGB-D/control integration run; original-surface insertion checks; boundary and safety-monitor diagnostics; and randomized layout, rest and camera-coverage checks. Physical partial rollouts have separately completed the drivetrain and avionics groups. Their counts cannot be combined into a full-reference success.

The requested frontier target is reward at most 0.05 on the 0–1 scale. No frontier result has yet been measured. There is no score cap, exponent or difficulty-dependent normalization that enforces this target.

Run the final gates with the engine commit pinned by the repository:

```bash
uv run ale lint /absolute/path/to/this/task
uv run ale validate /absolute/path/to/this/task --runs-dir /absolute/path/to/runs
```

The task is ready for benchmark admission only after untouched reward 0, a physically complete reference reward 1, and an audited frontier measurement on the same final version.
