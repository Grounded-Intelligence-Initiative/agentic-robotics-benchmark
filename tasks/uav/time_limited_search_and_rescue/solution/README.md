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

The original CPU author matrix on koa04 at `404fe53` produced these action frames.
The two fixed-head misses had no impacts or aborts; each reached the
original bay and attempted a close with the narrow head mounted.

| Seed | Sled | Found | Survey | Mounted | Revisited | Lifted | Cargo exit | Settled | Fixed narrow |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | Wide | 270 | 281 | 460 | 621 | 633 | 813 | 851 | No latch at original bay |
| 3 | Wide | 214 | 225 | 420 | 596 | 608 | 806 | 844 | No latch at original bay |
| 40 | Narrow | 57 | 65 | 153 | 218 | 230 | 375 | 414 | Settled at 398 |
| 77 | Narrow | 67 | 80 | 186 | 273 | 285 | 593 | 632 | Settled at 616 |

## Route Regression

An unseen official oracle case on that revision stopped at action 120,
before reaching the depot. A station's aft apron could be hidden by the
sled, leaving a missing reverse edge. With its requested goal unreachable,
the old planner chose an unscanned frontier instead; on arrival its
zero-length path caused an index error. The untouched control scored zero.

The revised reference records both directions of each completed crossing
after arrival. Both depot legs follow a shortest path made of those
crossings, with the existing floor transitions and RGB register at every
station. A zero-length path means stay; no path is distinct from arrival.
During egress a confirmed shutter removes both directions. The pilot can
then take a visible detour or resume frontier exploration. None of these
route choices is a scoring condition.

On the recorded failure seed, the revised RGB reference surveyed at action
103, mounted the narrow head at 205, reacquired the sled at 286, lifted at
298, crossed the cargo exit at 549 and settled the delivery at 588. Each
depot leg used three previously flown crossings; the opposite tracking
direction delivered at 594. The referee checks the return paths against
the actual search trace and the sequence of station fixes, independently
of the planner's own graph.

The expanded CPU author check delivered 17/17: the four earlier cases,
the failed site in both tracking directions, three fresh sites in both
directions, four further unfiltered draws, and a long-route site selected
from a fixed 64-site draw before flight. The long route took 856 actions;
the overall range was 414-917, so the 1,000-action allowance was retained.
This is sampled headroom, not a bound over all layouts. The matched
fixed-narrow control delivered on all three narrow cases and missed the
latch on all six wide cases, without impacts. All 52 depot legs across
the 26 RGB and fixed-head traces passed the crossing and recenter audit.

With `TASK` set to the task directory, use the pinned engine and a
cached rescue environment image `IMAGE` containing the visual assets:

```bash
ale lint "$TASK"
python3 scripts/lint_domain.py "$TASK"
python3 "$TASK/tests/review/test_routes.py" -v
docker run --rm --network none --cpus 1 --memory 3g \
  -e LP_NUM_THREADS=1 -e PYTHONDONTWRITEBYTECODE=1 \
  -v "$TASK:/task:ro" --entrypoint /opt/venv/bin/python3 "$IMAGE" \
  /task/tests/review/check_rescue.py --fixture-seeds 0 3 40 77 --require-mixed
```

Mount a writable directory and pass `--output` to retain local camera
frames and navigation traces. These are author diagnostics, not native
ALE validation or model runs.
