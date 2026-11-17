# V3 Fixed-Suite Policy

The eight seed values in tests/hidden/sites.json are unchanged from the maintainer
suite at base 878913e. The candidate list is also unchanged. V3 does not choose new
seeds or discard failed sites. The v2 selection record is historical and resides
in legacy_v2/selection.md and legacy_v2/selection.json.

Before v3 rollout diagnostics, the bounded generator was defined as a constructive
service-aisle family: a reflected/rotated partition, jittered openings, dogleg
baffle, two crates, variable elongated footprint, drive response and docking pose.
Each seed feeds random.Random(seed XOR 0xC8050003). No model, oracle result or
rollout is consulted by generation, and there is no rejection loop. The geometric
witness is a diagnostic only; it neither chooses sites nor controls scoring.

All eight fixed seeds are always evaluated. Development diagnostics use the
contiguous, independently specified range 1000-1019, including every result.
Failures require a general implementation fix and a complete rerun, never
per-seed overrides, seed replacement, or an outcome-selected hidden suite.

V3 native diagnostics are in v3_hidden_native.json. They do not inherit the v2
8/8 official anchor, v2 24/24 image-only result, or v2 200-seed map ablation.
The parent will record the actual separate-verifier result and final anchor.
