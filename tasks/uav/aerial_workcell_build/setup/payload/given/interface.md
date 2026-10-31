# Interface: what your scene must look like to the grader

The grader opens `/home/user/submission/scene.usda` (or `scene.usd`) in its own Isaac Sim 6.2 process, after your
session has ended, and measures it. It runs none of your Python. Everything the cell does must therefore be in the
stage itself: geometry, physics (rigid bodies, colliders, masses, materials, joints, drives, limits) and, where a
mechanism needs logic, OmniGraph action graphs.

## Stage

- Metres, kilograms, seconds; `upAxis = "Z"`; `metersPerUnit = 1`; `kilogramsPerUnit = 1`; default prim `/World`.
- One physics scene at `/World/PhysicsScene`, gravity 9.81 m/s² along -Z. The grader steps physics at 120 Hz.
- Files: the root layer, plus anything it references, sublayers or loads as a texture, must live under
  `/home/user/submission/` (referenced by relative path) or under the offline asset library `/opt/assets/isaac/`
  (referenced by absolute path). Nothing else is reachable at grading time.
- At t = 0 everything is at rest: the bin's lid closed, the button up, the items standing on their tables, the robots
  on their stands. Nothing may be set moving by the scene itself.

## Required prims

| path | what |
|---|---|
| `/World/Cell/Floor` | a static collider whose top is the floor plane z = 0 |
| `/World/Cell/Table_A`, `/World/Cell/Table_B` | the two tables (static colliders), see the photos for which is which |
| `/World/Bin/Body` | the bin's body, a rigid body carrying `UsdPhysics.ArticulationRootAPI`, fixed to the world where the bin stands |
| `/World/Bin/Lid` | the lid, a rigid body of the same articulation |
| `/World/Bin/Lid/Handle` | the handle's geometry, under the lid |
| `/World/Bin/Button` | the button's cap, a rigid body of the same articulation |
| `/World/Bin/LidHinge` | the revolute joint Body → Lid |
| `/World/Bin/ButtonJoint` | the prismatic joint Body → Button |
| `/World/Items/<name>` | one rigid body per object on the tables; `<name>` is up to you |
| `/World/Robots/Drone_L`, `/World/Robots/Drone_S` | the larger and the smaller robot, each one floating articulation (nothing fixes it to the world) with everything it carries, sensors included |

Other prims (lights, scenery, the stands, your own helpers) may go anywhere under `/World`.

## OmniGraph rules

Action graphs (`evaluator_name = "execution"`) are allowed for mechanism logic. The grader steps the simulation the
normal way (timeline playing, physics stepped), so graphs triggered by `isaacsim.core.nodes.OnPhysicsStep` (in an
on-demand graph) or `omni.graph.action.OnPlaybackTick` run. Allowed node types are those of the packages
`omni.graph.action`, `omni.graph.nodes` and `omni.graph.core` (variables), plus `isaacsim.core.nodes.OnPhysicsStep`,
`IsaacArticulationState`, `IsaacArticulationController`, `IsaacReadSimulationTime` and
`isaacsim.sensors.physics.IsaacReadJointState`, with these exceptions, which fail the scene:

- any script node (`omni.graph.scriptnode.*`, `omni.replicator.core.OgnConditionScript`, anything that runs Python);
- writing a prim's transform, a body's velocity or any attribute other than joint drive attributes
  (`drive:*`) and your own `arb:*` attributes (`WritePrimAttribute` / `WritePrims*` / `WritePrimLocalTransform` /
  `MoveToTransform` targets are checked);
- timeline control (`StartTimeline`, `StopTimeline`, `SetTimeline`, `LoopTimeline`).

`~/given/check_structure.py <scene>` checks this page's rules (paths, files, graph nodes) without scoring anything.
