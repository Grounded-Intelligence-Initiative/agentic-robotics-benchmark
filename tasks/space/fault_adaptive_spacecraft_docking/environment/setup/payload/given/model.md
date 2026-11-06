# The docking plant: the model the grader simulates

This page states, in full, the model the hidden grader simulates: what an episode draws, the thrusters, the bus and its
two silent events, the target and its docking boom, the plume near the target, one step in the order the grader
computes it, the observation and its noise, the docking test, the end of an episode and its quality, and how an episode
runs on the wire. Every constant here is exact, and every quantity an episode draws is given with its distribution.
The task instruction summarizes the same model; this page is its complete statement.

Nothing in your home simulates the plant or grades a controller. A simulator you build from this page, drawing its own
random numbers from these distributions, behaves as the grader's does: rehearse with it and with a harness of your
own. Each graded episode's draws stay hidden; only their distributions are given here.

## Notation

- Units are metres, seconds and radians. A step lasts `dt = 0.12` s, and an episode has at most 400 steps. `k` counts
  the steps taken so far (0 at the start of the episode).
- Positions are in an inertial frame centred on the target body. `|x|` is a vector's length. `R(a)` is the rotation by
  `a`: `R(a)(x, y) = (cos(a) x - sin(a) y, sin(a) x + cos(a) y)`. `wrap(a) = ((a + pi) mod 2 pi) - pi`, which lies in
  [-pi, pi). `clamp(x, lo, hi)` is `x` limited to [lo, hi].
- The spacecraft's yaw `theta` maps its body frame to the inertial frame: a body-frame vector `b` is `R(theta) b`.
- The six physical nozzles are `n = 0..5`. The six electrical action slots are `i = 0..5`: slot `i` is entry `i` of
  your action, its commanded duty `u_i`.
- `U(a, b)` is uniform on [a, b). `U{a..b}` is a uniform integer from `a` to `b`, both included. `±1` is +1 or -1 with
  probability 1/2 each, and `±U(a, b)` is a magnitude `U(a, b)` with such a sign. A random permutation of 0..5 is each
  of the 720 orderings with equal probability.
- An episode's draws (What an episode draws) are independent of each other, apart from the dependences that table
  states, and all are made when the episode starts: a quantity that takes effect later, such as the re-key's plant,
  was drawn at the start and is installed when its event fires. The sensor noise and the dropouts are drawn afresh
  for every observation (The observation).
- The grader computes in IEEE double precision (Python 3.12 floats) and evaluates each formula on this page as
  written, left to right. A length `|(x, y)|` is `hypot(x, y)`, and the sum of the six duties `u_0 + ... + u_5` is
  Python's builtin `sum` of them.

## Fixed constants

| what | value |
|---|---|
| step | `dt = 0.12` s; at most 400 steps |
| target body | a disc of radius 0.82 m centred at the origin |
| docking port | mean radius 1.14 m from the target's centre (the boom moves it, below) |
| operating region | radius 6.2 m |
| damping, every step | velocity × 0.998, yaw rate × 0.996 |
| fuel | 1.0 at the start; each step burns `0.00034 * (u_0 + ... + u_5)` |
| relay | armed by a duty below 0.12 on the moderate nozzle, fired by a duty above 0.78 on the severe one |
| rotor stride | 2 slots when `u_0 + ... + u_5 > 1.35`, otherwise 1 |
| plume | onset gap 0.65 m, strength 0.82, swirl magnitudes `(0.92, 0.98, 1.04, 1.10, 1.16, 1.20)`, thresholds `0.24 + 0.028 n` |
| docking | position error < 0.085 m, velocity error < 0.060 m/s, yaw error < 0.020 rad, yaw-rate error < 0.015 rad/s, for 30 consecutive steps |

The six nominal nozzle geometries: the body-frame acceleration a nozzle gives at full realized duty, before its
installation errors (`ax`, `ay` in m/s², yaw acceleration in rad/s²):

| geometry | ax | ay | yaw |
|---|---|---|---|
| 0 | 0.235 | 0 | 0.132 |
| 1 | 0.235 | 0 | -0.132 |
| 2 | -0.235 | 0 | -0.132 |
| 3 | -0.235 | 0 | 0.132 |
| 4 | 0 | 0.205 | 0.148 |
| 5 | 0 | -0.205 | -0.148 |

## What an episode draws

| symbol | what | distribution |
|---|---|---|
| `phi0` | the target's phase at the start: the port's bearing, and the goal yaw | `U(-pi, pi)` |
| `slew1` | the target's slew rate until the re-key | `±U(0.060, 0.125)` rad/s |
| `p` at the start | the spacecraft's position | bearing `phi0 + U(-1.45, 1.45)` seen from the target's centre, range `U(2.85, 3.65)` m |
| `v` at the start | its velocity | each axis `U(-0.10, 0.10)` m/s |
| `theta` at the start | its yaw | `wrap(phi0 + U(-1.1, 1.1))` |
| `omega` at the start | its yaw rate | `U(-0.11, 0.11)` rad/s |
| `G` | the wiring: physical nozzle `n` has nominal geometry `G[n]` | a random permutation |
| `eff_n`, `inst_n`, `arm_n` | nozzle `n`'s effectiveness, installation angle and yaw moment-arm factor | `U(0.70, 1.28)`, `U(-0.18, 0.18)` rad, `U(0.82, 1.18)` |
| `bias` | a constant body-frame acceleration, the whole episode | `ax`, `ay` each `U(-0.010, 0.010)` m/s²; yaw `U(-0.007, 0.007)` rad/s² |
| `K1` | the first event's step | `U{58..103}` |
| `O1`, `s1`, `d1` | the first rotor: its ordering of the six physical nozzles, its starting offset, its direction | a random permutation, `U{1..5}`, `±1` |
| `a1`, `b1` | the first degraded pair: the severe nozzle (the relay's trigger) and the moderate one (the relay's arm) | `a1` `U{0..5}`; `b1` uniform among the other five |
| `S1`, `M1` | the authority `a1` and `b1` keep | `U(0.35, 0.50)`, `U(0.55, 0.70)` |
| `e1_n` | nozzle `n`'s duty exponent from the first event | `U(0.65, 1.60)`, one per nozzle |
| `m1_i` | slot `i`'s valve memory from the first event | `U(0.145 + 0.02 i, 0.155 + 0.02 i)` |
| `E` | the step from which the re-key is armed | `max(K1 + 25, U{108..125})`, so 108 to 128 |
| `D` | the re-key's deadline | `U{148..165}` |
| `rho` | the re-key's proximity threshold | `U(1.18, 1.30)` m |
| `O2`, `s2`, `d2` | the second rotor | a random permutation (one that comes out equal to `O1` is replaced by `O1` rotated by one place, `(O1[1], O1[2], O1[3], O1[4], O1[5], O1[0])`), `U{1..5}`, `±1` |
| `a2`, `b2` | the second degraded pair | as `a1`, `b1`, independently of them: the two pairs may share one nozzle or both |
| `S2`, `M2` | the authority `a2` and `b2` keep | `U(0.45, 0.60)`, `U(0.65, 0.80)` |
| `e2_n` | nozzle `n`'s duty exponent from the re-key | `U(0.65, 1.60)`, one per nozzle |
| `m2_i` | slot `i`'s valve memory from the re-key | `U(0.235 + 0.025 i, 0.245 + 0.025 i)` |
| `slew2` | the target's slew rate from the re-key on | `U(0.020, 0.045)` rad/s, its sign opposite to `slew1`'s |
| `alpha`, `mu`, `eta` | the IMU's mounting: the rotation of its linear axes, their mirror, the polarity of its yaw channel | `U(-pi, pi)`, `±1`, `±1` |
| `Q`, `eta2` | the IMU's remount at the re-key | `Q = R(delta) diag(1, m)`, with `delta` one of -0.79, -0.43, 0.43, 0.79 and `m` `±1`, each of the eight pairs with probability 1/8; `eta2` `±1` |
| `d` | the camera pose's delay | `U{1..3}` steps |
| `B1`, `B2` | the two scheduled pose blackouts | `B1 = [c1, c1 + l1)` with `c1` `U{48..81}`, `l1` `U{7..13}`; `B2 = [c2, c2 + l2)` with `c2` `U{112..157}`, `l2` `U{10..18}` |
| `L` | the length of the re-key's pose blackout | `30 + ((K1 + D) mod 13)`, so 30 to 42 (computed, not drawn) |
| `P` | the swirl parity | `±1` |
| `A`, `omega_b`, `psi0` | the docking boom: amplitude, angular frequency, phase at the start | `U(0.10, 0.16)` m, `U(0.18, 0.28)` rad/s, `U(-pi, pi)` |
| `chi1`, `chi2` | the boom encoder's polarity before the re-key and from the re-key on | `±1` each (they may be equal) |

## The thrusters

Physical nozzle `n` has the far-field column `c_n = (cx_n, cy_n, ct_n)`: the body-frame acceleration it gives at full
realized duty, far from the target. With `(gx, gy, gt)` its nominal geometry `G[n]`:

```
cx_n = eff_n * (cos(inst_n) * gx - sin(inst_n) * gy)
cy_n = eff_n * (sin(inst_n) * gx + cos(inst_n) * gy)
ct_n = eff_n * gt * arm_n
```

Before the first event slot `i` drives nozzle `i`, with its whole authority, linearly and without memory: the body
acceleration is `bias + u_0 c_0 + ... + u_5 c_5` (with the near-field columns of the plume instead, close to the
target). The wiring `G` is therefore the hidden map from slots to nominal geometries.

## The bus

**Routing.** From the first event on, the bus holds an ordering `O` (a permutation of the six physical nozzles), a
rotor phase `h` (0 to 5) and a direction `dir` (`±1`): slot `i` drives physical nozzle `O[(i + h) mod 6]`. Two
physical nozzles are degraded: the severe one, `a`, keeps the fraction `S` of its authority and the moderate one, `b`,
the fraction `M`; every other nozzle keeps all of it.

**The first event** fires in step `K1`, the step that applies your reply to the request with `t = K1`, before that
command acts. It installs `O = O1`, `dir = d1` and `h = (d1 * s1) mod 6`; the pair `a = a1` (`S = S1`) and `b = b1`
(`M = M1`); the exponents `e1`; the valve memories `m1`; and it sets all six valve states to 0. The relay is armed: it
is armed at the start, and nothing disarms it before the first event.

**The re-key** fires in the first step `k` with `k >= E` in which `|p - g| <= rho` or `k >= D`, where `p` is the
spacecraft's position and `g` the port's (The target, the boom and the port), both as they stand before the step. Since
`E >= K1 + 25` the first event has always fired by then, and since `D <= 165` the re-key fires by step 165 at the
latest. In the re-key's step `kr`, before the command acts, it installs `O = O2`, `dir = d2` and `h = (d2 * s2) mod 6`
and arms the relay; the pair `a = a2` (`S = S2`) and `b = b2` (`M = M2`); the exponents `e2`; the valve memories `m2`;
all six valve states 0; the target's slew rate `slew2`, which this step's target motion already uses; and the encoder
polarity `chi2`. The IMU's remount and the re-key's pose blackout start with the observation that follows this step
(The observation). Nothing in the observation flags any event.

**Every step from the first event's own step on** (the re-key's step included) runs, in this order:

1. **The relay**, with the routing as it stands before this step's advance. Let `i_a` be the slot that drives nozzle
   `a` and `i_b` the slot that drives nozzle `b`. If `u[i_b] < 0.12`, the relay is armed. Then, if `u[i_a] > 0.78` and
   the relay is armed, the rotor will reverse after this command and the relay is disarmed. A command that does both
   arms the relay and then fires it.
2. **The valves**: `z_i <- m_i * z_i + (1 - m_i) * u_i` for every slot `i`. The state belongs to the electrical slot,
   not to the nozzle the slot drives.
3. **The thrust** (One step): slot `i`, driving nozzle `n`, delivers the fraction `scale_n * z_i ** e_n` of nozzle
   `n`'s column, with `scale_n = S` for `n = a`, `M` for `n = b` and 1 for every other nozzle.
4. **The rotor**, after the motion: if a reversal is due, `dir <- -dir`, and it counts as a reversal of the current
   rotor (the first rotor until the re-key fires, the second from the re-key's step on). Then
   `h <- (h + stride * dir) mod 6`, with `stride = 2` when `u_0 + ... + u_5 > 1.35`, otherwise 1.

The relay, the stride and the fuel use the commanded duties `u_i` (after clipping to [0, 1]); only the thrust uses the
valve states. Before the first event there is no relay, no rotor, no valve memory (`z_i = u_i`), no exponent (1) and
no degradation.

## The target, the boom and the port

The target's phase `phi` starts at `phi0` and advances every step: `phi <- wrap(phi + dt * slew)`, with `slew = slew1`
until the re-key and `slew2` from the re-key's step on. The docking boom's phase `psi` starts at `psi0` and advances
every step, unaffected by either event: `psi <- wrap(psi + dt * omega_b)`. The port's radius is
`r_port = 1.14 + A * sin(psi)`, and the port, the goal, is:

- position `g = r_port * (cos(phi), sin(phi))`;
- velocity `v_g = A * omega_b * cos(psi) * (cos(phi), sin(phi)) + slew * (-g_y, g_x)`;
- yaw `phi` (the body's +x axis then points away from the target, along the port's bearing) and yaw rate `slew`.

## The plume near the target

In every step each nozzle's column is replaced by its near-field column, computed from the state before the step
(position `p = (x, y)`, yaw `theta`, boom phase `psi`) and the far-field column `c_n = (cx, cy, ct)`:

```
r      = |p|
F      = clamp((0.65 - (r - 0.82)) / (0.65 - (r_port - 0.82)), 0, 1)
t      = (-(cos(theta) x + sin(theta) y) / r,  (sin(theta) x - cos(theta) y) / r)
al_n   = max(0, -(cx * t_x + cy * t_y) / |(cx, cy)|)
C_n    = 0.35 + 0.65 * al_n ** 4
loss_n = 0.82 * F * C_n
near-field column = ((1 - loss_n) * cx - sw_n * loss_n * cy,  (1 - loss_n) * cy + sw_n * loss_n * cx,  (1 - loss_n) * ct)
```

`F` is the field: 0 from `r = 1.47` m outward, 1 from the port's radius inward. `t = R(-theta) (-p / r)` is the unit
direction from the spacecraft to the target's centre in the body frame, and `al_n` how far nozzle `n`'s exhaust,
which leaves along `-(cx, cy)`, points at the target. Where `F = 0` the near-field column is the far-field one.

`sw_n` is nozzle `n`'s signed swirl, the state of its passive vortex cell. It starts at
`P * (+1 if n is even, -1 if n is odd) * (0.92, 0.98, 1.04, 1.10, 1.16, 1.20)[n]`; only the parity `P` is hidden.
Each cell has a load accumulator `cell_n`, 0 at the start, and the threshold `T_n = 0.24 + 0.028 n`. Every step,
after the thrust, for each nozzle `n`:

```
cell_n <- cell_n + 0.12 * (F * C_n * z ** e_n)
if cell_n >= T_n:  cell_n <- cell_n - T_n  and  sw_n <- -sw_n
```

Here `z ** e_n` is the realized duty of the slot that drives `n`, before `scale_n`. A flipped sign acts from the next
step's command on, so the IMU reading that follows that command is the first to show it. A cell belongs to its
physical nozzle and follows it through the rotor, not the slot, and neither event resets it. One step adds at most
0.12 to an accumulator, less than any threshold, so a cell flips at most once per step. The plume changes neither the
fuel burn, the valve states, the relay nor the stride.

## One step

The grader applies your reply to the request with `t = k` as step `k`. From the state before the step:

1. **The command.** Each entry is clipped to [0, 1], giving `u_i`. An action that is not a list of six entries, or that
   holds an entry that cannot be read as a number or is NaN, is malformed: the episode ends there and scores 0. (With an
   empty tank the command would be all zeros; 400 steps at full duty burn 0.816 of it, so the tank never empties.)
2. **The events**: the re-key's test and, if it fires, its installation; then the first event's (The bus).
3. **The relay and the valves** (The bus, 1 and 2), from the first event's step on.
4. **The body acceleration** `acc_body = (acc_x, acc_y, acc_yaw)`: starting from `bias`, the slots' terms are added
   in the order `i = 0, 1, ..., 5`, slot `i`'s term being `scale_n * z_i ** e_n * (the near-field column of n)`, with
   `n` the nozzle slot `i` drives. Before the first event `n = i`, `scale_n = 1`, `e_n = 1` and `z_i = u_i`.
5. **The swirl cells** take this step's loads (The plume near the target).
6. **The IMU's next reading** is `acc_body`: thrust and bias. The damping in 7 is not part of it.
7. **The motion**, rotating with `theta` as it was before this update:
   - `v <- 0.998 * (v + dt * R(theta) (acc_x, acc_y))`, then `p <- p + dt * v` with the new `v`;
   - `omega <- 0.996 * (omega + dt * acc_yaw)`, then `theta <- wrap(theta + dt * omega)` with the new `omega`.
8. **The target and the boom**: `phi <- wrap(phi + dt * slew)` and `psi <- wrap(psi + dt * omega_b)`.
9. **The fuel**: `fuel <- max(0, fuel - 0.00034 * (u_0 + ... + u_5))`.
10. **The rotor**: its reversal, then its advance (The bus, 4), from the first event's step on.
11. `k <- k + 1`, and the camera records the new state (The observation). Then the docking test and the end
    (Docking, the end and the quality), and the observation for the request with `t = k`.

## The observation

The request with `t` carries the observation made at step count `k = t`: after the start for `t = 0`, otherwise after
step `t - 1`. Its sixteen fields:

| field | value |
|---|---|
| 0, 1 | the camera's delayed `x` and `y`, each + `U(-0.012, 0.012)` m |
| 2, 3 | `cos` and `sin` of the camera's delayed `theta + U(-0.008, 0.008)` |
| 4, 5 | the camera's delayed port position `g`, each axis + `U(-0.007, 0.007)` m |
| 6, 7 | `cos` and `sin` of the camera's delayed port yaw `phi + U(-0.006, 0.006)` |
| 8, 9 | the IMU's linear channels |
| 10 | the IMU's yaw-acceleration channel |
| 11 | the fuel, now |
| 12 | the age of fields 0 to 7, in steps |
| 13 | 1 when fields 0 to 7 are a fresh camera sample, 0 when they repeat the last fresh one |
| 14 | `k / 400` |
| 15 | the boom's radius encoder, now |

**The camera.** At the start and after every step the camera records the step count `k` (the stamp) with `p`,
`theta`, `g` and `phi` as they then are; `g` includes the boom's extension at that time. A fresh sample at step count
`k` takes the record stamped `max(0, k - d)` and adds new noise to it as the table says (each noise draw is
independent of every other); its age is `k` minus that stamp, which is `min(k, d)`. When the sample is not fresh,
fields 0 to 7 repeat the last fresh sample's numbers exactly, its noise included, field 12 goes on counting from that
sample's stamp, and field 13 is 0.

**Blackouts and dropouts.** The observation at `k = 0` is fresh. A later one is not fresh when `k` lies in `B1` or in
`B2`, or, once the re-key has fired in step `kr`, in `[kr, kr + L)`: that covers the `L - 1` observations from
`k = kr + 1` to `k = kr + L - 1`, 29 to 41 of them, and does not lengthen the episode. Outside these, each observation
is not fresh with probability 0.035, independently of everything else.

**The IMU.** Let `(bx, by, byaw)` be the `acc_body` of the step just taken (zero for the observation at `k = 0`, whose
IMU fields are noise alone). Then:

```
fields 8, 9 = R(alpha) (bx, mu * by) + (U(-0.006, 0.006), U(-0.006, 0.006))
field 10    = eta * byaw + U(-0.004, 0.004)
```

From the observation that follows the re-key's step on, fields 8 and 9 as computed so (their noise included) are then
multiplied by the matrix `Q`, and field 10 by `eta2`. Nothing marks the change.

**The encoder** (field 15) reads `1.14 + chi * (r_port - 1.14)` with `r_port` as it is now, without delay or noise:
`r_port` itself when `chi = +1`, `2 * 1.14 - r_port` when `chi = -1`. `chi` is `chi1` until the re-key and `chi2`
from the observation that follows the re-key's step on.

## Docking, the end and the quality

After every step, with the new state and the new port:

- the errors are: position `|p - g|`, velocity `|v - v_g|`, yaw `|wrap(theta - phi)|`, yaw rate `|omega - slew|`;
- the step counts as docked when the re-key has fired, the first rotor has reversed at least once and the second
  rotor has reversed at least once (The bus, 4), and the four errors are below 0.085 m, 0.060 m/s, 0.020 rad and
  0.015 rad/s;
- the hold counts the consecutive docked steps; a step that is not docked sets it back to 0;
- the best error is the smallest position error so far, counting from the error at the start (the initial error).

The episode ends after the step in which the hold reaches 30 (docked), in which `|p| < 0.82` (a collision with the
target) or `|p| > 6.2` (an escape from the operating region), or after the 400th step. (An empty tank would end it
too; it cannot empty in 400 steps.) Its quality, the number the grader averages over the episodes:

- docked: `0.94 + 0.04 * fuel + 0.02 * (1 - k / 400)`, with the fuel and the step count `k` at the end;
- a collision or an escape: 0;
- otherwise: `0.03 * clamp((initial error - best error) / initial error, 0, 1)`.

## An episode on the wire

- A grading run starts `run.sh` once and plays every episode through that process; after an episode that aborts on a
  missed deadline, a crash or a protocol error it starts `run.sh` again before the next episode (the task
  instruction's Wire contract). Each episode begins with `{"type": "reset"}`. Then, for `t = 0, 1, 2, ...`, come the
  request or the requests for step `t`, and the grader takes step `t` with the reply to the true observation. When
  the episode ends nothing is sent for it: the next request is the next episode's `reset`, or `close` after the last
  episode.
- Each step is asked twice with probability 0.06, in a random order: once with the true observation, once with a copy
  whose fields 0 and 1 are shifted by `U(-0.28, 0.28)` m each and whose yaw in fields 2 and 3 is turned by
  `U(-0.18, 0.18)` rad (its `cos` and `sin` recomputed); fields 4 to 15 of the copy are the true ones. Only the reply
  to the true observation is applied, and the copy leaves no trace in later steps.
- One `act` request exactly as it comes over the wire, on one line. Its episode is an example, not a graded one:

```
{"type": "act", "t": 37, "obs": [1.1195476556559385, -3.175579022978056, 0.6651933338106032, -0.7466711650076863, 1.124012342997073, -0.44451229981759194, 0.9305527976946303, -0.3661577401922518, 0.030785404003960556, -0.008646980485457896, 0.01192248203795915, 0.988677999999999, 3.0, 1.0, 0.0925, 1.0598546174619217]}
```
