# Rescue Author Notes

This PR39 upgrade replaces target reporting with a complete physical sortie.
The licensed Crazyflie shell and Poly Haven cabinets, paving and wall textures
are rendered in MuJoCo on CPU. The fixed-heading UAV uses three servo-driven
MuJoCo slides; the pod is a free body attached only by a proximity- and
speed-gated weld. These are simulation abstractions, not a flight controller
or a hardware rescue procedure.

The private seed selects one of four cool openings, a pod location and yaw,
and a lighting variant. Seed neighbors `2k` and `2k+1` share pod, lighting,
initial pose, and geometry; they differ only in the earlier forecast tile
and which opening aborts the sortie. The first heat violation terminates
at the physics substep, with no retries. At the junction, both onboard
views are identical across a matched pair at the same pose. The split is
across separate episodes; no reset or seed control exists in the public API.

The root-owned ledger scores survey .10, safe entry into B .15, visibility
of the pod from the down camera .15, lift .25, loaded return into A .20,
and a settled delivery .15. Unsafe aborts cap the score at .65. The verifier
reads only this protected ledger; the agent's artifact files confer no credit.

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

The first check exercises twelve fixed seeds in six matched layouts.
It compares actual RGB bytes at the decision point and then drives the same
route in the pair to confirm different outcomes. It also runs a privileged
pilot, an untouched world, locate-only, a fixed-lane pilot given the pod's
exact coordinates, and negative cases for overflight, premature grip, and
high release. The staged smoke uses fresh containers with the root service,
agent UID, actual Unix socket and protected verdict; it is not the ALE
framework gate. No live model call is needed for either.
