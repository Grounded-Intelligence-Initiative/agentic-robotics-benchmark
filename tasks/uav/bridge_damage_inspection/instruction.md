# Long-Bridge UAV Inspection

## Mission
Find and document **three damaged service-walkway slabs** on a long bridge.
The bridge is 72, 96 or 120 metres long and has repeated 6-metre bays, two
walkways, piers, railings and (depending on the scene) steel trusses and maintenance
screens. Dark repair patches and intact expansion joints are not damage. A damaged
slab has a real opening with exposed reinforcement and the river visible below it.

Keep an inspection record across the flight: what you checked, where each defect
was, and which close-up frame proves it. Submit your findings and then finalize.
Direct operation, writing a search program and external notes are all allowed.
This is a long-horizon inspection task, not a test that forbids programmatic memory.

## Observation and flight
- `robot_observe` returns one **onboard, downward-looking RGB camera** (384x384,
  60-degree vertical field of view). The camera moves with the UAV. There is no
  global map, state vector, GPS overlay, segmentation image or target list.
- The camera is north-up: image right is +X and image up is +Y. Flight is stabilized
  at 13.5 m above the river; the bridge deck is at 8 m. The aircraft starts at
  world XY `[-2,-5.5]`, beside the near abutment. Camera height is 13.38 m.
- `robot_act({"action":[vx,vy]})` accepts normalized commands in [-1,1].
  Full command requests 3 m/s; the vehicle has finite response time and drag.
  Each call advances **0.5 simulated seconds**. There are **420 actions**, not a
  wall-clock race. Use observations to navigate and check progress.
- Bay B01 spans X=0..6, B02 spans X=6..12, and so on. Physical plaques identify bays.
  Each bay has a walkway slab centered at X=`6*(bay_number-0.5)` and Y=`-3.2` or
  `+3.2`. These are construction dimensions, not a map of the damaged locations.
  Inspect both sides. The length and defect positions vary between episodes.
- Stay within X=-5..(far abutment+5) and Y=-10..10. Leaving the site ends the flight.
  There is no reset and no budget refund when reconnecting.

## Evidence submission
Call `robot_report({"frame_id":123,"point":[x,y]})` once per suspected defect.
The original, server-recorded frame must show that defect close up: the UAV must be
within 2.1 m on each horizontal axis, with the aperture unobstructed. Coordinates
must be within 0.45 m of the slab center. Evidence can come from an earlier frame.
At most eight reports are accepted; duplicate reports count as false positives.

Reports are recorded **without correctness feedback**. After submitting your list,
call `robot_advance({})` to finalize. You can still report and finalize after the
last movement action. Nothing more can be changed after finalization.

The score is evidence-verified detection F1: `2*TP / (3 + submitted_reports)`.
A correct location without valid image evidence receives no credit. Missing
defects, wrong locations, duplicates and invented frame IDs reduce the score.
An unfinalized inspection receives zero. Coverage and action use are diagnostic
metrics, not arbitrary penalties. No separate controller file is required.

## Access and integrity
The root-owned environment owns the authoritative action and image ledger.
Observations include `frame_id`, `steps_remaining`, `phase`, `done` and
`reports_submitted`; they do not reveal report correctness or hidden damage.
`observe` does not advance physics. The service accepts at most 2048 requests.
The public socket client is `/home/user/robot/client.py`; use
`/opt/venv/bin/python3` for its dependencies. No internet or GPU is required.

Do not read protected files, recover seeds, modify the service, fabricate frames
or change the ledger. Agent-written success claims have no scoring authority.
The UAV uses licensed Crazyflie visual geometry with abstract stabilized planar
dynamics. This is a simulated inspection benchmark, not a hardware digital twin.
