# Draft text — Sec. IV-A impact resistance (results paragraphs)

Numbers from `campaign.py` on `20260907-173702_impact_pooled` (first 30 usable
strikes) and the ring dial batteries `20260907-150313` / `20260907-175701`.
Figure reference is a placeholder.

---

**Impact robustness.** To test the claim that the backdrivable joints yield
under a sudden load rather than resist it, the ring fingertip was struck with
a rigid pendulum (Fig. X(d)): a 285 g, 533 mm rod swung about a bolt through
its end hole and released from 60°, so that its free end, capped with a
hemisphere, met the palmar pad at the bottom of the swing at 2.80 m/s with
0.37 J of kinetic energy. A uniform rod struck at its tip behaves like a point
mass of one third of its mass at the tip speed, so the pad saw a 95 g
equivalent striker — the energy and momentum of a 95 g object dropped from
0.40 m, or, in energy alone, a 0.43 kg bob released from 45° on a 0.30 m
pendulum, or a 0.5 kg hand assembly striking a fixed obstacle at 1.2 m/s. The
hand was mounted palm-up with the finger held in its normal current-based
position control (600 mA limit) at 12° PIP and 46° MCP, and it was not
re-homed at any point. Of 38 strikes, eight were excluded by a rule fixed
before analysis (operator-noted misses and glancing contacts with a peak
deflection below 5°) and the first 30 usable strikes are reported.

**Results.** On every strike the PIP deflected 6.3 ± 0.6° (maximum 7.5°,
leaving 4.5° to its extension stop) and the MCP 4.3°. Both swung back through the
commanded pose at about 90 ms, undershot by 1.8°, and settled within ±1° of it by
about 165 ms (median over strikes; Fig. X(a)). The servo's effort during the strike was small: the peak actuator current
was 262 mA on average (44 % of the 600 mA goal-current limit) and 318 mA at
most, so the joint yielded at under half the torque the actuator could have
applied and recovered without the current limit ever being reached
(Fig. X(b)). The compliance seen here is therefore that of an ordinary
position controller (P = 800, D = 500) acting through a backdrivable
transmission, not a limit-clipped servo giving way; the same joint that
backdrives at 0.02 N·m when unpowered gives 6° to a 0.37 J strike while
powered, and nothing tripped. At the 61 mm lever arm of the struck pad, the 0.02 N·m breakaway
torque of Table III is a contact force of 0.33 N, two orders of magnitude below
the estimated peak contact force of 10–67 N (half-sine contact of 40 to
10 ms), which is why the joint gives way rather than resists. No hardware
error was raised and no damage was visible after the block. Fingertip
repeatability, measured with a dial indicator over 100 unidirectional
approaches to the same pose, was σ = 6.2 µm before the block and 7.4 µm after
it (R = 3σ of 18 and 22 µm; Fig. X(c)). The ratio of 1.20 is not significant
(F(99,99) = 1.44, p = 0.07); the one-sided 95 % bound is that σ did not grow by
more than 41 %, against a smallest resolvable change of 22 % at n = 100, so the
result is a bounded null rather than a demonstrated change. The indicator was
re-mounted between the two batteries, so only the spreads are compared.
Tactile data were not recorded during the strikes: the sensor's 83 Hz frame
rate cannot resolve a transient of tens of milliseconds, and contact was
confirmed on video.

---

Caption:

> Fig. X. Pendulum impact on the ring fingertip, 30 strikes at 0.37 J
> (2.8 m/s, 95 g effective mass). (a) PIP (struck) and MCP deflection, all
> strikes with the median in bold. (b) PIP actuator effort during the strike:
> the current peaks at 262 mA on average, well inside the 600 mA limit. (c) Fingertip repeatability from 100 dial-indicator approaches before
> and after the block, R = 3σ of 18 and 22 µm; the indicator was re-mounted
> between sessions, so only the spreads compare. (d) Test setup, not to scale:
> a 285 g, 533 mm rod pivoted at one end is released from 60° and strikes the
> ring fingertip pad at the bottom of its swing while the finger holds its pose.
