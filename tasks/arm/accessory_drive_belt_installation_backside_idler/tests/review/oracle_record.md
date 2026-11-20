# Honest program record (drive B)

The honest program (solution/honest/, byte-identical to tests/review/honest/) uses the public API, the given documents,
the given Panda model and the cameras / force-torque sensors only. Records by version, out of sample first.

## Evaluation under full_at_rule.md (2026-11-12)

Pre-registered 18:33Z (fresh_seeds.json sha256 19f7d77e..., full_at_rule.md sha256 157b66d2..., hidden set registered
and drawn 18:33:26Z, selected before the evaluation); honest program hseq.py sha256 6cb09f21..., hrig.py 9c66e50d...,
heyes.py ebf23fd3...; one episode per engine, in-process cell on the development host (the task-image repeat of rule 6
is still to be done).

| set | engines | scored 1.0 | notes |
|---|---|---|---|
| fresh (16) | 16 | 15 | f03 (seed 2017950024322729554): protective stop in the bottom seating pushes at C -- the right hand struck the pallet (467 N), its transit to the push start swept low |
| hidden (4) | 4 | 4 | simulated 410-902 s |

Development host: p = 19/20 = 0.95.

## Rule 6: the same evaluation in the task image (2026-11-12 19:06-19:5xZ)

Cold build of the image (`docker build --no-cache --pull` of a clean copy of environment/, MuJoCo 3.16.0, numpy 2.5.3;
the Panda assets fetched at the pinned commit and checked file by file), then one single-episode verify per engine
(tests/test.sh, the runner account, the no-network filter; a copy of tests/ with that engine as the one hidden episode),
one CPU per episode.

| set | engines | scored 1.0 | notes |
|---|---|---|---|
| fresh (16) | 16 | 14 | f03 as above (identical simulated time, 264.49 s); f01 (seed 8229530716408625650): the C step's carry check read 0.82 N against its 1.00 N threshold, the program judged the grasp lost and let the tensioner back (simulated 325.8 s; 417.4 s and 1.0 on the host: the image's numerics differ in the last bits) |
| hidden (4) | 4 | 4 | simulated 415-907 s; episode walls in the image 227-797 s on one CPU over all 20 |

p = 18/20 = 0.90 (the lower of the two stands): **full_at = floor_to_0.01(0.90 - 1/16) = 0.83**; the anchor value (the
mean on the 4 hidden engines) is 1.0.

## Development (in sample)

Development seeds 1-12 over seven rounds: 6, 9, 6/10, 8, 8/11, 10, 12 of 12 at score 1.0; the final version also 12/12
on seeds 13-24 (never used in development). No stop, damage or physics fault in those 24 episodes; simulated 330-918 s.

## Engine validation on the frozen bytes

`ale validate` (engine 1cdbc02) on the frozen pilot copy (manifest sha256 096096ad32c0...): validate-630aff72 and
validate-7c77e689, both untouched 0.0 / oracle 1.0 (the oracle = this honest program, graded like any submission on
the 4 hidden engines).
