# Shift sheets

The cell's supervisor writes a shift sheet for every shift it runs: the tickets of the shift's jobs and the times of
their phases. `example_shift.json` is the sheet of shift 1 of another unit, HU-DJWFZ. A
sheet holds tickets and times only: no mass, force, label or outcome.

```
{"format": "shift-sheet/1", "serial": "<unit serial>", "shift": 1 | 2,
 "jobs": [{"job": 1, "ticket": {"job": 1, "class": ..., "code": ..., "finish": ..., "profile": ...},
           "phases": [["await_clear", t0_ms, t1_ms], ["present", t0_ms, t1_ms], ...]}, ...]}
```

- `jobs`: the shift's jobs in order (10 per shift).
- `ticket`: as the controller sees it in the job's observations (`cell_manual.md`, section 7).
- `phases`: each phase the job went through, with its start and end in ms since the unit's hand-over (phase names:
  `cell_manual.md`, section 4).
