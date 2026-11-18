# Training Wing Author Notes

This is a new version after `5b4fe09`, not a revision to that candidate's
trial results. It replaces both forecast boards with a procedural indoor
wing and a physical scale model of its topology. There are 20 room cells,
22 connecting doorways (a spanning tree and three extra connections),
and three marked candidate bays. The viewed rooms, rather than the
floor plan, reveal the actual sled. Mirrored seed siblings reflect the
doors, bays, inlet and exit; other bases resample connectivity and the pod.
The cargo corridor occupies an outer column and has a single connection
to the search rooms, at row three or four. This forces several loaded
hallway segments that an outbound bay route never needs.
The downward camera is a local perspective camera, so tall opaque
walls can occlude adjacent rooms rather than giving a floating map view.

## Physics and scoring

The Crazyflie mesh rides on three MuJoCo slides; the pod is a MuJoCo free
body and can be attached only by a proximity- and speed-gated weld.
The verifier checks separate swept bounding volumes for the airframe
and the real posed sled against the same obstacle boxes that are rendered.
These are conservative simulation envelopes, not validated hardware
flight, contact compliance, or payload aerodynamics.

The inspection slot is 0.46 m high. The airframe/gripper envelope
fits, but the suspended sled and airframe together do not: the airframe
can fit at 0.71 m while the sled strikes the sill. The cargo shutter is
physically closed until the first lift moves the sled center to 0.54 m,
then retracts. The first interior pocket has raised sills on all inward
paths, requiring a climb after insertion. The walls reach above the
altitude envelope. Neither inspection-slot retracing nor early entry
through the cargo route is viable.

Full reward comes from physical lift and settled release on the pad.
No map, route, progress-point, or time-efficiency threshold gates a valid
delivery. Partial progress measures plan vicinity .05, wing entry .12,
up to three viewed bays .02 each, target vicinity .10, grasp .12,
lift .15, and loaded egress .20. Bay/target vicinity is a conservative
camera-placement proxy, not semantic proof of what the agent recognized.
The private ledger also records ingress and loaded exit heights, room
and cell coverage, action counts, per-stage timing, and impact component.
There is no credit from submitted files, and no scoring of path optimality.

## Reproduce

From the pinned engine, with `TASK` set to the task directory:

```bash
ale lint "$TASK"
ale validate "$TASK" --runs-dir /tmp/ale-rescue-wing-validate
ale run "$TASK" --agent oracle --runs-dir /tmp/ale-rescue-wing-oracle
```

The ALE oracle reads only the public camera stream. From the first plan
view it reconstructs a graph, then it searches candidate rooms in
shortest-path order, visually aligns over the sled, and takes the cargo
route back. Its `reference-route.json` records visited center waypoints
as zero-based `[column,row]` from left and front; row zero is the staging
strip. It never imports the simulator or reads a seed.

In the author example (seed 0), the RGB pilot entered at `[3,1]`,
searched the three marked bays, and lifted in `[1,4]`. The loaded
approach reached the service connection at `[3,4] -> [4,4]` and took
`[4,3] -> [4,2] -> [4,1] -> [4,0]` out, finishing in 229 actions.
The 1,024-layout route audit found at most 29 interior edge traversals
for the tested search order across all candidate bay positions. At six
actions per edge plus a conservative 81 for flight outside the graph,
pickup and release, that gives 255 against the 300-action limit.

Author checks use environment image `IMAGE` and an optional writable
mount at `/frames` for the plan, inspection view, delivery, and route:

```bash
docker run --rm --cpus 2 --memory 3g --network none \
  -e PYTHONDONTWRITEBYTECODE=1 -v "$PWD:/task:ro" \
  --entrypoint /opt/venv/bin/python3 "$IMAGE" \
  /task/tests/review/check_rescue.py --rgb-seeds 0 1 6 7 12 19

for mode in untouched oracle; do
  docker run --rm --cpus 2 --memory 3g --network none --user 0:0 \
    -v "$PWD:/root/source:ro" --entrypoint /bin/bash "$IMAGE" \
    /root/source/tests/review/staged_smoke.sh "$mode"
done
```

When RGB seeds are requested, the audit appends its longest selected-bay
case. The deterministic author checks cover geometry, route length, positive
and unsurveyed deliveries, locate-only, matched pickup/retrace, fixed
layout replay, bypass and level-flight attempts, and RGB plan decoding.
Staged smoke checks the real Unix socket, agent UID and protected verdict,
not the ALE framework gate. These are author feasibility controls, not
participant-model results or a sealed benchmark sample.
