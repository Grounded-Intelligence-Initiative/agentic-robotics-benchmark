# Difficulty knobs (for maintainers; nothing runs this file)

Every knob that sets how hard the task is lives in one place. Change it there, and keep the byte copies in step:
`tests/cell.py` == `environment/cell/cell.py` (the grader's cell and the practice cell's). The agent-visible documents
(`environment/setup/payload/cell/*.md`, `instruction.md`) quote some of these values; update them when you change one.
After a change, re-measure the reference (null-anchor `ale validate`) and re-record `tests/anchor.json`.

## The throws and the hidden variation (`cell.py`, `KNOBS`)

| knob | value | effect |
|---|---|---|
| `latency_steps` | 4 | control steps between an observation and the step its reply acts at (80 ms at 50 Hz) |
| `flight_time` | 0.5 .. 0.7 s | release to arrival in the catch zone; shorter = faster tosses, less time to estimate the flight |
| `release_dist` | 1.9 .. 2.3 m | the thrower's release point along +x |
| `spin_endover` | 7.0 rad/s | maximum end-over-end spin; more spin makes aiming at the handle in flight unreliable |
| `spin_roll` | 3.0 rad/s | maximum roll about the tool's axis |
| `scale` | 0.95 .. 1.05 | tool size factor |
| `head_mass` | 0.6 .. 1.6 | density factor of the non-handle parts (moves the centre of mass) |
| `friction` | 0.6 .. 1.0 | tool-to-hand sliding friction (the tool's geoms have contact priority, so this value is the one used) |

## What counts (`cell.py`, `KNOBS` and `ThrowJudge`)

| knob | value | effect |
|---|---|---|
| `zone_frac` | 0.30 | share of the handle each requested grip zone (`end` / `choke`) covers |
| `announce_t` | 2.0 s | when the requested zone appears in the observation (after any catch: in-hand regrasp is needed even after a perfect handle catch about half the time) |
| `handle_margin` | 0.02 m | grip-location credit ramps to 0 this far outside the zone |
| `axis_tol_deg` | 20 / 45 deg | working end up: full credit / none |
| `palm_tol_deg` | 30 / 55 deg | tool axis across the palm (hand Y): full credit / none |
| `steady_tol` | 0.01 / 0.03 m | tool motion in the hand and hand motion in the world over the hold window: full / none |
| `catch_credit` | 0.1 | credit of a tool caught and held to the end; the functional grasp earns the other 0.9 |
| `zone_rule` | "far" | at the announcement the job asks for the grip zone farther from where the hand holds the tool (a regrasp is always needed); "random": the throw's own draw |
| `THROW_STEPS`, `HOLD_STEPS` | 450, 100 | 9 s per throw, the last 2 s judged |

## The hand and the wrist (`cell.py`)

`FINGER_KP` 2.0 N m/rad, `FINGER_TAU` 0.8 N m, `FINGER_DAMPING` 0.03 N m s/rad (Menagerie: 0.1), wrist limits
`V_LIN, A_LIN` = 2.0 m/s, 20 m/s^2 and `V_ANG, A_ANG` = 8 rad/s, 80 rad/s^2, cameras `CAMS` and `IMG_W, IMG_H`
(192 x 144).

## Grading and practice

| knob | where | value |
|---|---|---|
| runs, throws per run | `tests/grader_config.json` `episodes`, `throws_per_episode` | 36 x 12 (every run: each tool 3x; flight time and end-over-end spin stratified over the run) |
| per-step reply budget | `control_period_s`, `late_limit_periods` | 20 ms; a reply later than 5 periods is discarded |
| hard per-request deadline | `step_timeout_s` | 10 s (aborts the run) |
| probe lock | `probe_rate`, `probe_pass_fraction` | 0.01, 0.1 |
| practice budget | `environment/practice_service/practiced.py` `RUNS`, `THROWS` | 4 runs x 4 throws |
