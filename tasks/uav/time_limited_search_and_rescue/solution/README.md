# Rescue Author Notes

This PR39 upgrade replaces target reporting with a complete physical sortie.
The licensed Crazyflie shell and Poly Haven cabinets, paving and wall textures
are rendered in MuJoCo on CPU. The fixed-heading UAV uses three servo-driven
MuJoCo slides; the pod is a free body attached only by a proximity- and
speed-gated weld. These are simulation abstractions, not a flight controller
or a hardware rescue procedure.

The private seed selects one of four lateral lanes and independently one of
two vertically and laterally separated bulkhead windows. In each block of
four seeds, toggling bit 0 swaps the lane, toggling bit 1 swaps the height,
and everything else remains fixed (including pod and lighting). Wrong
openings terminate at the physics substep, with no retries. Both onboard
views are identical across each matched pair at its corresponding junction
at the same pose. The upper window requires more altitude than the first
barrier permits, and neither the sides nor roof provides a bypass.

The root-owned ledger scores .05 per board, .10 per outbound crossing,
.10 for seeing the pod, .20 for lifting it, .15 per loaded return crossing,
and .10 for settled delivery. It records actual center altitude at each
crossing. Unsafe aborts cap the score at .65; agent artifacts confer no credit.

## Run the Oracle

Using the pinned ALE engine (no model):

```bash
ale lint "$TASK"
ale validate "$TASK" --runs-dir /tmp/ale-rescue-validate
ale run "$TASK" --agent oracle --runs-dir /tmp/ale-rescue-oracle
```

Here `TASK` is the full path to this task directory. ALE stages
`solution/solve.sh` in the oracle episode only. It drives the same public
client as the agent and infers both the heat cue and the sled from RGB.
The privileged author pilot is separate and is never staged for interaction.

## Reproduce Author Checks

With the built environment image, run this in the task directory:

```bash
docker run --rm --network none --cpus 2 --memory 3g \
  -e PYTHONDONTWRITEBYTECODE=1 -v "$PWD:/root/source:ro" \
  --entrypoint /opt/venv/bin/python3 "$IMAGE" \
  /root/source/tests/review/check_rescue.py --rgb

for mode in untouched oracle; do
  docker run --rm --network none --cpus 2 --memory 3g --user 0:0 \
    -v "$PWD:/root/source:ro" --entrypoint /bin/bash "$IMAGE" \
    /root/source/tests/review/staged_smoke.sh "$mode"
done
```

The first check exercises twelve seeds covering all joint lane/height
variants. At both junctions it compares actual RGB bytes and drives the
same route in the corresponding matched pair to confirm different outcomes.
Controls include privileged, untouched, locate-only, cue-blind lane and
altitude pilots with perfect pod coordinates, fixed-altitude flights, and
attempts to fly over or around the bulkhead. Pass `--output /frames` with
a writable mount to save the forecast and both onboard junction images.
The staged smoke uses fresh containers with the root service, agent UID,
actual Unix socket and protected verdict; it is not the ALE framework
gate. No live model call is needed for either.
