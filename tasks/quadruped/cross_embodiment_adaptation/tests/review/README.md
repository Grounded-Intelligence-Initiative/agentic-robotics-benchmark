# Review notes: version 5 (maintainers, 2026-11-20)

The task: a UAV surveys an unknown warehouse; a loaded Unitree Go1, commanded joint by joint, has to cross it to
a dock on its first deployment there, crouching under a suspended duct and standing tall over a deck's guides.
The graded set, the per-site properties and how their ranges were set: [selection.md](selection.md).

## What changed against the author's v8 (`d07a9c15`)

| | v8 | version 5 |
|---|---|---|
| the Go1 in development | never available; the instruction forbade simulating it | a commissioning pad (bare floor, bollards, mast camera; 40 episodes, 600,000 ticks; no truth, score or debrief), a datasheet, and any simulation of the agent's own |
| per-site physics | servo stiffness +/-10 % | carried mass 2.5-4.5 kg and where it sits, servo stiffness +/-15 %, torque limit, friction, the duct's clearance (0.410-0.445 m) and the guides' rise (0.070-0.125 m), all stated with their ranges, none reported |
| between graded sites | files in /tmp, /dev/shm, the account's home and lock directories survived | everything the account leaves is removed and shared files are restored (`tests/isolation.py`) |
| oracle | a reference from the public observations | privileged (a token; `solution/oracle`); the public reference is kept here as a control |
| tick budget | 36,000 | 48,000: 1.44 times the oracle's slowest of 48 development sites |
| arithmetic | as the host's CPU has it | pinned in the image, checked before grading (`tests/numerics_check.py`) |
| anchor | 1.0, `full_at` 1 | 1.0, `full_at` 0.98 |

Why: one frontier episode on v8 scored 4 of 8 in 29 of 120 minutes by building its own Go1 model and rehearsing
whole missions in it, which the instruction forbade and nothing could check; the same program docked 21 of 32
development sites. A prohibition a grader cannot check measures obedience, so it is gone, and the difficulty is
moved to what no rehearsal fixes in advance: each site's robot and fittings have to be measured there.

## Controls (`controls.py`, `replay_submission.py`)

Measured in the task's image; graded sites as text records only. None is a participant program or a gate.

| controller | graded sites (8) | development sites |
|---|---|---|
| oracle (`solution/oracle`, with truth) | 8 | 46 of 48 (two scanner-on-floor contacts under low ducts) |
| oracle without stance-height feedback | 7 | |
| oracle without pitch trim | 5 | |
| oracle without either | 4 | |
| oracle with the fittings' nominal clearances | 4 | |
| oracle without either feedback and with nominal clearances | 1 | |
| public reference (nominal load and postures) | 1 | |
| the saved v8 frontier program, replayed without a model call | 0 | 8 of 32 (was 21 of 32 on v8) |
| do nothing, survey only, stand still | 0 | |

On the graded sites the oracle uses 27,115 to 30,269 ticks, keeps 14 to 26 mm between the guard and the duct's
underside and puts four feet on every deck.

What the table says: a controller that knows the site but does not correct its stance from its joint angles and
gravity reading loses half the sites; one that assumes the nominal fittings loses half; with both it is left
with one. Friction, in the stated range, changed no controller's result here and is kept as part of the robot's
description, not as a lever.

## Isolation probe

A submission that writes a marker into every directory its account can write (and plants desktop start-up
files) and reports what it finds on the next site: under v8's grader every later site read the earlier ones'
markers in /tmp, /var/tmp, /dev/shm, the account's home and the lock directory; under this grader none does
(46 to 142 entries removed after each mission). It gets no inet socket and cannot read the tests, the records or
the robot models.

## Machines

`tests/numerics_check.py` gives one fingerprint on an AMD Zen 3 and an AMD Zen 5 host (simulation, camera frames
and a program's NumPy / SciPy arithmetic). Before the pin, the same controller's dock errors differed between
the two in the ninth decimal.

## Reproduction

In the task's image, as root, from the task's folder:

```bash
/opt/venv/bin/python3 -B tests/review/controls.py oracle --hidden
/opt/venv/bin/python3 -B tests/review/controls.py oracle --n 48
/opt/venv/bin/python3 -B tests/review/controls.py oracle:noheight+notrim+nominalfit --hidden
/opt/venv/bin/python3 -B tests/review/controls.py public --hidden
/opt/venv/bin/python3 -B tests/review/replay_submission.py <a submission folder> <out dir> --n 32
/opt/venv/bin/python3 -I -B tests/numerics_check.py
```
