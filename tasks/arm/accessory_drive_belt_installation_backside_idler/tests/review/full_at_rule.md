# full_at rule (pre-registered before the evaluation it governs)

Registered before the honest program has been run on any engine of the fresh set or the hidden set below.

1. The honest program (solution/honest/, byte-identical to tests/review/honest/) is evaluated ONCE on the sealed fresh
   set (tests/review/fresh_seeds.json, 16 seeds from os.urandom, drawn 2026-11-12T18:33:16Z, sha256
   19f7d77e63b2529412bdf2afc96a1949180809c8dd978eba6578019184425da5) plus the 4 hidden engines (tests/hidden/seeds.json),
   one episode per engine, with the cell and the grader of the frozen files recorded in tests/hidden/selection.json.
2. Let p = (engines scored 1.0) / (engines evaluated) on those 20.
3. If p >= 0.85: full_at = floor_to_0.01(p - 1/16). The anchor value is the honest program's mean score on the 4 hidden
   engines in that evaluation.
4. If p < 0.85: no full_at is set; the honest program goes back to development (and a new fresh set is drawn before the
   next evaluation).
5. The honest program evaluated is the one whose sha256 is recorded in tests/hidden/selection.json (or its latest
   amendment) before the evaluation; no change to it after the evaluation counts for this rule.
6. Before freezing, the same evaluation is repeated in the task image (one single-episode verify per engine); a lower
   p there replaces this one under 3-4.
