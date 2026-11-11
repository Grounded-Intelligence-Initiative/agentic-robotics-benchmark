# full_at rule (pre-registered before the evaluation it governs)

Registered 2026-11-10 11:08Z, before the evaluation (full_at may be lowered to the level the honest procedure reaches
stably). The text below is the registered rule (sha256 fb05a0e1...) with two references rewritten for publication on
2026-11-10: the fresh set's private path (now shipped as tests/review/fresh_v2_seeds.json, same bytes) and the
development step named in 4; the rule itself is unchanged. The evaluation's result: `oracle_record.md`.

1. The honest oracle (solution/honest/, byte-identical to tests/review/honest/) is evaluated ONCE, in the task image,
   on the sealed fresh set v2 (tests/review/fresh_v2_seeds.json, 16 seeds, drawn
   2026-11-10 09:32:51Z, sha256 7df50e2d79832a8df9d05232516a6adbb822364dc5a4c96ac7d238d5ef99b883) plus the 4 hidden v2
   engines (tests/hidden/seeds.json), one single-episode verify per engine.
2. Let p = (accepted engines) / (engines evaluated) on those 20.
3. If p >= 0.85: full_at = floor_to_0.01(p - 1/16). The anchor value is the oracle's belt_acceptance on the 4 hidden
   engines in that evaluation.
4. If p < 0.85: no full_at is set; the honest program goes back to development.
5. The oracle version evaluated is the one whose sha256 is recorded in tests/hidden/selection.json's latest amendment
   before the evaluation; no change to it after the evaluation counts for this rule.
