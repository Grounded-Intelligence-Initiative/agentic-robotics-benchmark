# V4 Fixed-Suite Policy

The eight seeds and complete candidate list are retained byte-for-byte from
maintainer round-2 head 9d8cc14, not from the superseded base 878913e suite.
Its original floor-localization geometry, selection record, source and measured
outcomes are archived intact at legacy_round2/. V4 does not reselect any seed.

The v4 generator is the bounded native-payload family designed before integration:
a reflected/rotated partition, jittered openings, dogleg baffle, two crates,
variable elongated footprint, drive response and docking pose. Every seed feeds
random.Random(seed XOR 0xC8050003); the salt is retained from checkpoint 07f3a3a.
The version label is now 4 to distinguish this redesign from maintainer round 2's
v3 protocol. No solver result, model failure or rollout is consulted by generation.
There is no rejection loop, per-seed override or selection by oracle outcome.

All eight latest hidden seeds must run. Development diagnostics use all seeds
1000-1019, specified before integration. A failure requires a general fix and
complete rerun, never seed replacement. The geometric witness is a diagnostic,
not an acceptance filter or scoring rule.

V4 preserves exactly the maintainer's 500 scout actions, 240 carrier actions,
1000 requests, 600 s program time, 1800 s mission wall limit, 16200 s verifier
and 7200 s authoring session.

Pre-integration candidate receipts are under legacy_candidate_v3/ and cannot
support a score on this latest suite. Current native measurements are regenerated
as v4_hidden_native.json and v4_development_native.json. Official ALE validation,
the final anchor and frontier evaluation remain the parent's responsibility.
