# V8 author controls: the inspection load has to fit

Two switchback rack openings contain a suspended duct and a loading cradle
in variable order. The Go1 carries a fixed, mass-bearing inspection saddle:
a protective crown above the trunk and a scanner projecting ahead and below
it. These are collision geoms on a rigid 0.70 kg child body, not a virtual
height threshold. The native free joint and twelve position servos remain.

For a level body on the floor, the duct leaves at most **0.227 m** of trunk
height (0.426 minus 0.199). For the same level orientation with feet in
the cradle tracks, passing the guide crest requires **more than 0.245 m**
above the tread (0.082 plus 0.163). The 0.176 m scanner is wider than a
0.150 m wheel track. This is a geometric explanation for the reference
route, not a pose constraint. Foot, leg, ramp, tread and guide support are
legal. A body can tilt or choose another foothold; only unsafe physical
contact, falling or the existing budgets can prevent a dock.

The UAV sees two covers with the same dimensions and finish. The reference
tracks complete cover outlines over separated RGB exposures, fitting
`point_xy = camera_xy + depth * image_ray_xy`. The elevated cover is on the
duct roof, the lower one on the deck's center guide. The roof thickness,
riser dimensions and load envelope are disclosed construction dimensions.
The mast cannot look through the duct and the controller gets no terrain
label, contact cause, height telemetry, map pose or seed. After registering
the ground mast images, it uses only launch-frame odometry and native joint
commands. The first deployment is irreversible.

## Matched experiments

All arms use the same survey, registered transform, route, initial turn,
stride, swing, joint IK and dock controller. Only the leg-extension setpoint
changes: adaptive uses 0.205 m near the measured duct and 0.265 m elsewhere;
fixed arms never switch their selected setpoint. The controller ramps from
the native home pose at startup. The 0.217 m fixed arm was chosen because
it actually solved *both* isolated fixtures of the rejected plain-deck
prototype. Neither the intermediate control nor the prototype is a scored
task variant.

The flat intervention removes only the two fittings. It retains the
inspection load, independently translated/rotated starts, rack/slit
geometry, gains, registration and contact rules. Sites 0 and 2 also run with
the fittings interchanged and the outer survey rows reversed and shifted from
1.50 to 1.42 m off center. [Selection](selection.md) gives the fixed suite.

| Final layout / arm | Safe docks | Observed effect |
|---|---:|---|
| Main / adaptive | 8/8 | Both fittings cleared, four distinct feet on each treaded deck |
| Main / ordinary | 0/8 | Crown struck duct on all eight |
| Main / crouch | 0/8 | Scanner struck entry apron on five, center guide on three |
| Main / fixed 0.217 m | 0/8 | Scanner struck center guide on all eight |
| Flat / ordinary | 8/8 | Same load and contact rules |
| Flat / crouch | 8/8 | Same load and contact rules |
| Order + flight perturbation / adaptive | 2/2 | Both fittings cleared; four feet supported on deck |
| Order + flight perturbation / ordinary, crouch | 0/2 each | Two duct strikes and two apron strikes, respectively |

| Site | Adaptive ticks | Crown gap in central duct (mm) | Final dock error (mm) | Fixed crouch struck |
|---|---:|---:|---:|---|
| 0 | 23,560 | 18.1 | 85 | Center guide |
| 1 | 22,360 | 16.8 | 80 | West apron |
| 2 | 21,437 | 15.9 | 91 | West apron |
| 3 | 23,282 | 16.9 | 93 | West apron |
| 4 | 24,243 | 18.8 | 92 | West apron |
| 5 | 23,445 | 14.7 | 88 | Center guide |
| 6 | 21,713 | 16.5 | 94 | West apron |
| 7 | 22,659 | 16.2 | 83 | Center guide |

On the eight main sites the maximum absolute cover-height error was 2.45 mm;
worst launch registration error was 9.51 mm and 0.090 degrees. All four
arms at each site produced identical RGB passage maps, measured fitting
coordinates and applied launch transforms. No final trial fell or exhausted
its physics budget. Each main reference used 53 scout moves, eight mast
exposures and at most 2,066 of 4,000 requests and 24,243 of 36,000 ticks.

## Development and limits

The first, steep apron stranded the ordinary gait. A gentler plain deck
separated the two extreme gaits but admitted a single intermediate gait, so
that design was rejected. The final raised guides and two wheel tracks add a
real underbody clearance conflict. In the straight-lane probe the ordinary
gait clears the deck with all four feet supporting on it and hits the duct;
crouch does the opposite. Five fixed heights from 0.211 to 0.235 m were
probed on both final fittings; none cleared both. The formerly successful
0.217 m compromise struck the center guide with the scanner.

Early whole-site runs also exposed launch-turn and rack-turn contacts that
were not the intended effect. The final west staging bay has turning room;
all arms settle, align, then change aisles in measured open bays. The flat
controls check the resulting controller rather than charging those earlier
failures to terrain adaptation.

This is a known nominal inspection configuration in calibrated simulation,
not hardware validation or generalization across arbitrary new morphologies.
Mounts and fittings are rigid; the dock floor is level; odometry is ideal.
The geometric bounds apply to a level body on the tracks, not every possible
pitch, jump, gait or use of the raised surfaces. A safe alternative succeeds
without reporting any maneuver to the verifier.

## Reproduction

On a CPU with the pinned task dependencies, visual assets and OSMesa software
rendering, run from the task root. Use fresh output directories:

```bash
python -B tests/review/test_v8_contract.py
python -B tests/review/v8_fixture_probe.py --compromise
python -B tests/review/v8_controls.py final --arms adaptive ordinary crouch intermediate
python -B tests/review/v8_controls.py flat-final --flat --arms ordinary crouch
python -B tests/review/v8_controls.py swapped --swap --alternate-survey --sites 0 2 --arms adaptive ordinary crouch
python -B tests/review/check_v8_controls.py .
```

Final evidence is on `koa04:/home/wang100/projects/cross-mobility-author-20261119`
(MuJoCo 3.16.0, CPU-only OSMesa, public Unix socket). Each case retains
its service ledger; selected sites include actual UAV and mast PNGs.
These are author controls, not formal ALE validation or participant results.

| Receipt | SHA-256 |
|---|---|
| `final/results.jsonl` (32) | `1ec7a0fba0a30ba956f90c2794d7fef0e57566a3606c1e8944eff073f68aea83` |
| `flat-final/results.jsonl` (16) | `c3c9293910b36316a9db25bffe5b46ebea452fd7d7118fe934611047a4177fe4` |
| `swapped/results.jsonl` (6) | `a0228e0045e17fcc528064bba5dcdb471d142905be5d00c0ef5df8a14f010393` |
| `cradle-compromise.jsonl` (10 straight probes) | `371efd4569f4366b77866027e98272e47af7ecead55957e796c565044461e3ff` |

The verifier still scores safe docks out of eight, anchor 1.0. It now
records duct/deck entries and downstream clearances, the number of distinct
feet supported by the level tread, minimum vertical crown-to-soffit gap in
the central part of the duct, fixture/floor contact causes and the colliding
robot part in the private record. None awards or deducts a maneuver point.
There are no new participant calls and no change to the wall, action, camera
or request budgets. Flat/swap runs are author counterfactuals, not retries.
