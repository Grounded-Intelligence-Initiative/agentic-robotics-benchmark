# Rescue Fixture Author Notes

Based on `8ae8c9c00396ce64d955847ae0d37c96a27eae49`. The station fixes,
raised entrances, mezzanine, lift-triggered shutters, swept clearance
checks, and blue-pad set-down are retained. Time, action allowance and
the RGB-only observation envelope are unchanged.

The common cover makes the overhead view good for finding the sled but
poor for reading the flange. The reference moves to the side, lowers
and turns toward the fitting to see its profile. It records that profile
and the bay's location on its camera-built topological map. It uses the
same inspection slot to reach the staging depot and return, recognizing
the two heads by their image geometry, then reacquires the sled from RGB
at the remembered bay. No map, simulator state, or seed enters the pilot.

Both collets are rigid bodies, welded either to their rack stands or to
the aircraft. The modeled jaws use the same inner dimensions for rendering
and capture: a flange must fit inside the seat and be wider than the
retaining lip. This is a self-centering capture abstraction, not a finger
contact solver. A compatible close couples the UAV and sled at the actual
relative pose. A wrong head or bare gripper leaves the sled on the floor.
Transiting with a head uses its enlarged swept clearance envelope. No
survey, equipment identity, or sequence of actions is checked by scoring.
Guessing and swapping after a miss is still a legitimate recovery.

## Author Review

The fixed-narrow control starts on the same seed, searches with the same
RGB controller, and flies the same depot/revisit workflow. It skips the
low survey and takes the narrow head regardless of what was found. The
referee records the survey, acquisition, revisit, close, lift, cargo
crossing and settled delivery separately. The older visual feedback
and loaded retrace controls are still available without `--fixture-seeds`.

The CPU author matrix on koa04 produced the following action frames.
The two fixed-head misses had no impacts or aborts; each reached the
original bay and attempted a close with the narrow head mounted.

| Seed | Sled | Found | Survey | Mounted | Revisited | Lifted | Cargo exit | Settled | Fixed narrow |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | Wide | 270 | 281 | 460 | 621 | 633 | 813 | 851 | No latch at original bay |
| 3 | Wide | 214 | 225 | 420 | 596 | 608 | 806 | 844 | No latch at original bay |
| 40 | Narrow | 57 | 65 | 153 | 218 | 230 | 375 | 414 | Settled at 398 |
| 77 | Narrow | 67 | 80 | 186 | 273 | 285 | 593 | 632 | Settled at 616 |

With `TASK` set to the task directory, use the pinned engine and a
cached rescue environment image `IMAGE` containing the visual assets:

```bash
ale lint "$TASK"
python3 scripts/lint_domain.py "$TASK"
docker run --rm --network none --cpus 1 --memory 3g \
  -e LP_NUM_THREADS=1 -e PYTHONDONTWRITEBYTECODE=1 \
  -v "$TASK:/task:ro" --entrypoint /opt/venv/bin/python3 "$IMAGE" \
  /task/tests/review/check_rescue.py --fixture-seeds 0 3 40 77
```

Mount a writable directory and pass `--output` to retain local camera
frames and navigation traces. These are author diagnostics, not native
ALE validation or model runs.
