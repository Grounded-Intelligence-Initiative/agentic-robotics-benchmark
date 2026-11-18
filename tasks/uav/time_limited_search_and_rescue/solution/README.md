# Mezzanine Wing Author Notes

This is an independent successor to `aad334db`. It removes the launch
map. The only layout evidence is local RGB revealed by actual movement:
opaque rooms, real door mouths, local floor aprons, and semantic
INSPECTION, RESCUE BAY, MEZZANINE and CARGO landmarks. Seed siblings
mirror the geometry. Between base layouts, the interior topology,
landings, bay, egress connection and safety shutter vary.

The RGB reference integrates public setpoint increments in its own
launch-relative frame. It turns to face doorways, reads their local
aprons, keeps unobserved neighbors as frontiers, verifies doorway
signatures on revisits, and records loop edges when two explored sides
connect. If an expected apron is obscured by the carried load, the
reference turns and looks again before updating the map. A visible red
apron prompts a turn to inspect the forward camera; the map drops the
edge only after seeing the red physical panel. The shutter always lies
on a cycle. Some positive
routes can avoid encountering the shutter, and no scored checkpoint
requires it. The reference also locates and aligns to the sled using
RGB, not an exported map or target position.

This is topological loop closure at calibrated indoor modules, not
drift-tolerant metric SLAM or a test of motor-level stabilization. The
pose integration uses known actuator increments and zero wind, but
the route and target require visual observations. Labels are semantic
place names, not unique room IDs. A capable scripted or model policy
can solve it. Author controls establish feasibility and mechanism,
not participant difficulty or a guarantee of model failure.

The upper deck is a physical 0.60 m floor that supports the free-body
sled. Its raised landings require a vertical transition, including
when descending along the loaded cargo run. The UAV is a stabilized
MuJoCo slide and yaw body with a speed- and proximity-gated weld.
Swept airframe and posed sled envelopes check walls and gates
separately; these are simulation proxies, not flight-validated
aerodynamics. The safety shutter and cargo leaf physically move when
lifted; there is no invisible route failure based on a seed bit.

The live ledger measures entry, reaching the mezzanine, bay/target
viewing vicinity, attachment, lift, loaded return and settled delivery.
It also reports entry/landing/exit altitudes, actual yaw travel,
visited-cell and bay counts, phase timing, and airframe versus sled
impacts. Viewing vicinity is a physical camera-placement proxy, not
proof the agent recognized the target. No artifact, route recall,
optimality, or partial score gates full delivery.

## Reproduce

With the pinned ALE engine and `TASK` pointing to this directory:

```bash
ale lint "$TASK"
ale run "$TASK" --agent oracle --runs-dir /tmp/ale-rescue-local-oracle
ale validate "$TASK" --runs-dir /tmp/ale-rescue-local-validate
```

Run author controls with the built environment image and a writable
optional `/frames` mount:

```bash
docker run --rm --network none --cpus 2 --memory 3g \
  -e PYTHONDONTWRITEBYTECODE=1 -v "$PWD:/task:ro" \
  --entrypoint /opt/venv/bin/python3 "$IMAGE" \
  /task/tests/review/check_rescue.py
```

The actual RGB pilot is exercised on separate layouts; a fixed replay
of one positive route is then run on a mirror and on a different
layout with the same inlet. The same
RGB pickup is paired with return via the original inspection slot.
That control attempts the slot after a real lift, not by attaching
the sled programmatically. The oracle writes `reference-route.json`
with detected local doors, turn-to-look shutter views, reobservations,
and loop/edge updates. Staged smoke uses the agent UID, real socket and
protected verifier but is distinct from the official ALE gate.
The 780-action allowance leaves over 200 actions beyond the 558-action
calibration run with repeated loaded camera probes. The agent wall-clock
limit is unchanged at 30 minutes. The reference trace's local doorway
order is launch-relative forward, right, left, then back.

In the seed 0 reference example, 18 cells were discovered. Frame 346
shows the red panel at a remembered connection; the pilot removed that
one edge, descended with the sled, and delivered in 417 actions.

```bash
for mode in untouched oracle; do
  docker run --rm --network none --cpus 2 --memory 3g --user 0:0 \
    -v "$PWD:/root/source:ro" --entrypoint /bin/bash "$IMAGE" \
    /root/source/tests/review/staged_smoke.sh "$mode"
done
```
