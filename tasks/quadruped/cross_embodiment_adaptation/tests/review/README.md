# V6 author evidence

V6 adds two independent aperture decisions and a mandatory center-return
partition. The robot cannot replace a clearance check with an open/closed
test or use one side decision for the whole route. The same orange caps show
the actual collision footprints of all racks. UAV images sampled from both
center and side positions show the near-center slot and the ends of the
offset opening at the calibrated rack-top plane. No color means "passable."
See [fixed suite](selection.md) for the four route combinations.

The three arms use identical public RGB, target observations, route length
objective, and twelve-joint controller. Each point-robot intervention
removes only the minimum-width check at its named rack.

| Arm | First wide choice and crossing | Middle crossed | Second wide choice and crossing | Safe docks | Cause on failures |
|---|---:|---:|---:|---:|---|
| RGB + Go1 width | 8/8 | 8/8 | 8/8 | 8/8 | None |
| Point robot at first rack | 0/8 | 0/8 | 0/8 | 0/8 | First shortcut contact, 8/8 |
| Point robot at second rack | 8/8 | 8/8 | 0/8 | 0/8 | Second shortcut contact, 8/8 |

All 24 runs used 42 survey moves; none fell or exhausted a budget. Safe
missions used 17,022-18,174 of 36,000 physics ticks, 1,447-1,542 of
4,000 requests, and 5.37-6.63 s wall time on koa04. Minimum recorded
lateral trunk-center slack at a rack was 0.441 m (not swept-volume
clearance). Final dock center errors were 0.087-0.094 m against a
0.17 m requirement. Both ablations saw the same projected site maps.

## Reproduction

Use the pinned task dependencies and pinned visual assets, with OSMesa
software rendering. For example, in the task image:

```bash
/opt/venv/bin/python3 -B tests/review/test_v6_contract.py
/opt/venv/bin/python3 -B tests/review/v6_controls.py /out/fresh
# Optional root-only isolation test in a disposable task container:
bash tests/review/test_v5_isolation.sh
```

The CPU-only author run used MuJoCo 3.16.0 and CPython 3.11 on
`koa04:/home/wang100/projects/cross-affordance-author-20261119`.
`controls/results.jsonl` has 24 per-site/arm rows (SHA-256
`8ee33350dde916dd7cddb1e1d3352e7e422611e67071d19ed3fb67360cdab126`);
`controls/` also holds each service ledger and live RGB samples from sites
0 and 1. Every arm drove the graded service over its Unix socket. This was
an author control, not a blinded run or formal ALE validation.

## Verifier

Scoring is unchanged: fraction of eight safe docks, anchor 1.0. No changes
to site seeds, wall/program budgets, submission staging, process isolation,
or the switch/deployment rules. Scout actions increase from 64 to 80,
target ticks from 28,000 to 36,000, and requests from 3,000 to 4,000.

The verifier retains the first-choice and first-crossing diagnostics.
Choices latch at the first approach plane (0.34 m before the rack face);
crossings latch 0.34 m past the rear face. It adds shortcut choices,
middle and final crossings, second-choice accuracy, dock-zone entry,
contact location, target-budget and request exhaustion, and program
exit/time caps as aggregate and per-site counters. First/second decision
accuracy is correct divided by entered, not divided by all starts.
Per-site telemetry includes target ticks and final dock distance. Private
`record.json` also retains raw passage choices, lateral margin, contact
wall, and target outcome.

The UAV and tracking are idealized; the target is a simulated, unstabilized
native Go1 with no participant target practice. This is evidence for
geometry-to-route transfer, not general hardware locomotion.
