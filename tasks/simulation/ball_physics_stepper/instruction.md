Build a small **2-D ball physics simulator**, not a controller. Leave a self-contained
program at `/home/user/submission/run.sh`. It must retain state and return the ball's
position, velocity and simulated time after **every** request, not just a final score.

## Physical model

Coordinates are metres, y points up, time is seconds, mass is kilograms, and impulses
are N s. A ball has no rotation. Gravity is constant and downward. There is no drag,
horizontal wall, friction, or ball-ball collision. The optional plate is a stationary,
infinite horizontal surface, not a finite paddle. Its top is at `plate_y`; the ball's
centre may not go below `plate_y + radius`. `plate_y: null` removes it entirely.

Each `step` first applies an instantaneous impulse J: `v <- v + J / mass`. Then advance
by `dt` using continuous motion: `x += vx*dt`, `y += vy*dt - gravity*dt*dt/2`, and
`vy -= gravity*dt`. Split the interval at collisions. At downward contact with the
plate, set `y = plate_y + radius` and `vy = -restitution*vy`; `vx` is unchanged.
Continue through the rest of the interval, including any further bounces. A collision
exactly at the end of the interval returns the **post-impact** velocity.

A ball at rest on the plate stays supported against gravity. A later upward impulse
can lift it off. To terminate inelastic bouncing, when gravity is positive and the
outgoing vertical speed after an impact is at most `1e-7` m/s, set `vy=0` and keep the
ball on the plate for the remaining interval. A downward impulse while on the plate
is reflected immediately; apply the same small-speed rule. For `dt=0`, apply the
impulse and any immediate contact response but do not advance time. At zero gravity,
do not apply the small-speed rule; a stationary ball remains stationary.

## Streaming protocol

The evaluator launches `bash run.sh` in `/home/user/submission` as your own account.
Print `READY` once within 4 seconds. Read one JSON object per stdin line and emit one
JSON object per stdout line within 1 second. Send diagnostics to stderr. A reset can
occur repeatedly in the same process and must discard all previous state/configuration.

```json
{"type":"reset","config":{"gravity":10,"mass":2,"radius":0.1,"restitution":0.8,"plate_y":null},"state":{"position":[0,5],"velocity":[1,0]}}
```

Return `{"state":{"position":[0,5],"velocity":[1,0],"time":0}}`.

```json
{"type":"step","dt":0.2,"impulse":[2,4]}
```

Return `{"state":{"position":[0.4,5.2],"velocity":[2,0],"time":0.2}}`.
The next step starts from that returned physical state; it does not start over.
`{"type":"close"}` requires `{"ok":true}`, followed by a clean exit.

All fields shown above are present in valid reset/step requests. JSON numbers in every
returned state must be finite. Return exactly two position components, two velocity
components, and one time. Inputs are valid: gravity in [0,20], mass in [0.25,5], radius
in [0.01,0.5], restitution in [0,1], plate_y either null or in [-2,2], starting x in
[-5,5], starting y at or above contact (or in [-5,15] without a plate), initial velocity
components in [-10,10], dt in [0,2], and impulse components in [-5,5]. Up to 100 steps
may follow a reset. The reset time is always zero. Contact states with zero or upward
velocity are valid, including upward speeds below `1e-7`; the small-speed rule applies
after an impact, not to an arbitrary free-flight state.
For numerical contact detection, a centre within `1e-12` m of the plate contact height
is treated as exactly at contact when a step begins (reset itself returns the input state).

## Evaluation and development

The evaluator sends fresh parameter settings and sequences of steps. It independently
computes the expected state at reset and after each step. A trajectory passes only if
**all** its responses match: for each scalar, absolute error must be at most
`2e-6 + 2e-7 * abs(expected)`. The headline metric is the fraction of trajectories
that pass. Tests cover free flight, impulses and mass, zero gravity, elastic/inelastic
impacts, repeated impacts, resting contact, varying dt and repeated resets. Failures
remain in the denominator. Invalid output, crashes or protocol timeouts score zero.
Results are normalized against a measured reference implementation; its value and
hidden test seeds are not provided. Do not return a self-reported accuracy or reward.

Python 3.12 and numpy are installed. No network, GPU or external assets are needed.
A protocol skeleton is at `/home/user/starter.py`. A small public test suite is at
`/home/user/practice_grader/grade.py`; run `python3 /home/user/practice_grader/grade.py`.
It reports raw test outcomes only. It is not the full evaluation, and its hand-worked
examples are not a simulator implementation. You may write your simulator in any
installed language as long as `run.sh` satisfies this protocol.
