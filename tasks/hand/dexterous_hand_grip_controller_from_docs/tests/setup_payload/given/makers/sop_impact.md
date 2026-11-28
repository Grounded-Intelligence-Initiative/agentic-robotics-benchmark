# SOP — Pendulum impact test (paper section IV-A)

Substantiates the claim arXiv v1 asserts without data: that backdrivable joints
**yield** under sudden loads rather than resisting them.

Two results are required:

- **Yield.** During the strike the joint deflects and returns, with the actuator
  current at or near its goal-current limit — torque-limited, not rigidly
  resisting. Whether the current actually reaches the clip is a result to
  report, not a precondition (Sec. 2).
- **No damage.** The residual offset does not accumulate over 30 strikes, and
  the `post_impact` dial battery shows no degradation beyond its resolvable bound.

**The quantitative bridge from Table III.** In the hold pose (Sec. 3) a force
normal to the fingertip pad acts on a **36 mm arm about both the MCP and the
PIP**, so the 0.02 N·m breakaway torque is only **0.55 N at the pad**. The strike
below peaks at an estimated 32–51 N, i.e. 1.2–1.8 N·m at the joint — roughly
**two orders of magnitude** more. At any plausible collision energy this joint
cannot resist; it necessarily gives way. Table III and the impact result together
make a claim neither makes alone.

**Striker: the 285 g, 21 in (533 mm) rigid stick**, swung about a bolt through the
hole at one end and striking with the other. A uniform rod struck at its tip
behaves like a point mass of **one third of its mass — 95 g — moving at the tip
speed**, the same class as the 100 g bob this SOP was first written around, so
the framing carries over with the numbers changed. `--striker bob` keeps the old
numbers if a bob is ever used instead.

---

## 1. Rig (buildable in under an hour)

| Part | Spec |
|---|---|
| Pivot | Bolt through the hole, **snug**: a bushing or sleeve in the hole and washers each side, no perceptible play. The pivot takes an impulsive reaction of about half the contact impulse, so lock-nut it. |
| Stand | Rigid. The pivot sits ≈ 0.52 m above the pad: a bar clamped across two heavy stands, or a bolted frame. It must not rock. |
| Striker face | Ø20 mm **hemisphere** on the free end — printed PLA/POM cap or a steel dome nut. Never the bare cut end: it would cut the elastomer and concentrate the load on a few taxels. |
| Angle gate | Printed protractor arc at the pivot with hard pins at **20°** and **45°** |
| Release | Radial: withdraw a pin, or a magnet on a stick against a steel washer glued to the rod, so the rod leaves with zero tangential velocity |

**Calibrate the striker (3 minutes) — measure, don't assume uniform:**

1. Weigh the rod alone and the cap alone (±1 g). Enter `--mass-kg` and
   `--tip-mass-kg`.
2. Measure the hole centre to the near end, `a`. Enter `--pivot-offset-m`. Each
   10 mm of offset removes ≈ 4 % of the energy.
3. Balance the bare rod on an edge: the balance point should sit `L/2 − a` from
   the hole (267 mm for a = 0).
4. Time 20 free small-amplitude swings (≤ 10°) of the capped rod. The runner
   prints the predicted period for your inputs — **1.196 s** for the bare rod at
   a = 0, 1.180 s at a = 15 mm. Within 3 % validates the uniform-rod model;
   outside it, find out why before striking.

---

## 2. Energy

**Working point as run on 7 Sep 2026: rod 0.285 kg, L = 0.533 m, θ = 60° →
E = 373 mJ, v_tip = 2.80 m/s, m_eff = 95 g, p_eff = 0.27 N·s** (the runner was
started with `--angle-deg 45`; the manifests were corrected to 60° afterwards
with the originals kept as `*.orig`). The 45° row below was the planned point.

Uniform rod about its end: `E = m g (L/2)(1 − cos θ)`, `v_tip = sqrt(3 g L (1 − cos θ))`,
`m_eff = I / L² = m / 3`, `p_eff = m_eff · v_tip`. Total kinetic energy equals
`½ m_eff v_tip²`, so the finger sees a 95 g mass at the tip speed.

| θ | E | v_tip | p_eff | dE/E per ±0.5° |
|---|---|---|---|---|
| 20° (warm-up) | 45 mJ | 0.97 m/s | 0.092 N·s | ±5.0 % |
| 30° | 100 mJ | 1.45 m/s | 0.138 N·s | ±3.3 % |
| **45°** | **218 mJ** | **2.14 m/s** | **0.204 N·s** | **±2.1 %** |
| 60° | 373 mJ | 2.80 m/s | 0.266 N·s | ±1.5 % |

(The original 100 g / 300 mm bob at 45° was 86 mJ at 1.31 m/s; the rod
reproduces that at 27.8°.)

Why 373 mJ is representative — give the reader the equivalences. Energy is
the invariant; quote the one that also keeps the 95 g effective mass when the
peak force matters:

| Scenario | Equivalent to the 60° rod strike |
|---|---|
| A 95 g object (the rod's effective mass) dropped on the fingertip | from **0.40 m** — same energy *and* same momentum |
| A point mass released from 45° on a 0.30 m pendulum | **0.43 kg** bob at 1.31 m/s — same energy, 2.1× the momentum |
| 0.5 kg hand assembly striking a fixed obstacle | **1.22 m/s** — above the 0.25–1.0 m/s shared-workspace range |
| 0.3 kg distal hand mass | 1.58 m/s |
| A 250 g tool dropped on the fingertip | from **152 mm** |

Estimated peak contact force is **42–67 N** for a 10 ms half-sine (e = 0–0.6),
and **10–17 N** if the contact lasts the full 40 ms loading phase measured on 7
Sep; either way under the ISO/TS 15066 transient limit of 140 N for the hand
region — a citable framing for "a real collision, but non-destructive". The
short-duration figure exceeds the tactile module's 25 N per-point range, one reason
tactile is not recorded here.

**Energy needs no contact duration.** E is the striker's kinetic energy at the
bottom of the swing, fixed by mass, geometry and release angle alone. Duration
enters only the peak-force *estimate* above. What the encoders at ~430 Hz do
resolve is the **loading phase**, first motion to peak deflection: **41 ms
median (IQR 39–46 ms, ~18 samples)** on 7 Sep, with the joint back inside
0.02 rad about 100 ms after first motion and a peak joint speed of 2.7 rad/s,
i.e. a fingertip at ~0.24 m/s against a striker at 2.80 m/s. The finger did not
follow the rod: the rod rebounded and kept most of its energy. How much the
finger absorbed is only measurable from the rebound angle on the 240 fps video,
`E_release − E_rebound` less the free-swing loss; without it, quote the strike
energy as delivered *to the contact*, not as absorbed by the finger.

**Expected response.** At most ~55 mJ goes into the finger (the rest stays in
the rebounding rod or is dissipated in the pad). Against the 600 mA clip with
P = 800 that is a peak deflection of roughly **8–13°**, against **29.8° of free
travel** to the extension stop at both the MCP and the PIP in the hold pose. The
position loop only demands 600 mA at ~8° of error (600 / (800/128) ≈ 96 counts),
so the current **may or may not** reach the clip during the deflection; the D
term usually spikes it in the first few milliseconds. Report the peak-current
fraction either way — `campaign.py` does.

**45°, not a smaller angle with a heavier striker.** Energy sensitivity to
release angle is `dE/E = [sinθ/(1−cosθ)]·dθ`, independent of length: ±0.5° is
±2.1 % at 45° but ±5 % at 20°.

**Sequence:** 3 warm-up strikes at 20° (45 mJ) in their own run — check that the
trigger fires, the contact is single and square, and the deflection is a few
degrees — then the N = 30 block (run at 60° on 7 Sep; 45° is the milder
option). Top up misses with a second run at the same `--hold-rad` and merge with
`experiments.analysis.pool_impact_runs`.

---

## 3. Alignment and hold pose

**Target: the palmar (sensing) face of the struck finger's distal link** — the
**ring finger** for this campaign (`--digit ring`; the index, middle and ring
modules are identical) — marked with a 5 mm dot. Palmar because that is the surface that meets the world in a grasp,
and because a palmar strike drives the finger into **extension** — the direction
the servo is holding against, which is the yield the claim is about.

**Hold pose: yours to choose, torque enabled, mode 5, 600 mA.** Set it one of
three ways:

- `--teach`: torque drops on the struck finger only, you pose it by hand where
  the striker will meet it, ENTER captures. The rest of the hand keeps holding.
- `--hold-rad PIP MCP ABD`: motor radians, flexion negative.
- neither: the coded 30° MCP / 30° PIP pose.

Whichever you use, the runner prints the geometry of that pose from the FK model
and stores it in the manifest, so `campaign.py` converts drift and torque with
the arms of the pose actually struck:

- **free travel to the extension stop** at PIP and MCP — keep **≥ 20°** at both.
  A 45° strike is predicted to deflect 8–13°; at the hard stop the joint cannot
  yield and you would be characterising the stop. The runner warns below 20°.
- **where the pad faces.** The coupled DIP turns the fingertip well past the
  palm normal: for the 30/30 pose the pad normal is 58° from the palm normal
  toward the fingertips, so an upright hand is not struck squarely by a
  horizontal swing. The runner prints the palm-up tilt that makes the pad face
  the swing (**nose-down ≈ 32°** for 30/30, fingers toward the pivot). Use a
  bench vise on the wrist adapter or an angle plate; check squareness on video,
  ±5°.
- **lever arms**: torque arm of a pad-normal force about MCP and PIP (36 mm for
  30/30, so the 0.02 N·m breakaway is 0.55 N at the pad) and fingertip
  displacement per joint radian (73 mm/rad for 30/30).

Mid-range flexion of both joints, pad facing the swing, is all the pose needs.
Deeper flexion buys travel but turns the pad further toward the wrist; the
printed tilt follows it.

Alignment steps:
1. Hang a plumb line from the pivot; mark the bottom-of-swing point.
2. Position the hand so the fingertip dot sits there, pad normal parallel to
   the swing direction (the tilt above).
3. Set the pivot height so the striker face is at the dot's height at the bottom.
4. Verify the swing plane contains the finger's flexion plane (±3°).

Tolerance is forgiving in velocity (contacting 5° early loses 0.7 % of v) but
**tight in contact point**: ±3 mm on a 36 mm arm is ±8 % in joint torque. Verify
the contact point on video for every strike and report the measured scatter; the
tactile frames give the contact patch location as a cross-check.

**Mount rigidity — verify, don't assume.** Put an ArUco marker on the palm and
one on the bench in the 240 fps frame and measure the palm's peak displacement;
report it as a fraction of the fingertip deflection. Then repeat 3 warm-up
strikes against a steel bolt of similar length in the same mount to separate
mount compliance from finger compliance.

---

## 4. Running it

Size the capture rate first (torque off; Return Delay Time lives in EEPROM but
does not touch the calibration file or its hash):

```bash
python -m experiments.bench.measure_read_rate --set-return-delay-us 0 --samples 2000
```

Set `--capture-hz` to about `1/p99_period`, not `1/mean`. Then:

```bash
python -m experiments.impact_test --dry-run --strikes 5        # rehearse
# warm-up, its own run directory; --teach poses the finger by hand first
python -m experiments.impact_test --digit ring --teach --strikes 3 --angle-deg 20 \
    --mass-kg 0.285 --length-m 0.5334 --pivot-offset-m <a> --tip-mass-kg <cap> \
    --capture-hz <measured> --operator <initials>
# the block: repeat the warm-up's pose numerically (it is printed and in run.yaml)
python -m experiments.impact_test --digit ring --hold-rad <PIP> <MCP> <ABD> --strikes 30 --angle-deg 45 \
    --mass-kg 0.285 --length-m 0.5334 --pivot-offset-m <a> --tip-mass-kg <cap> \
    --capture-hz <measured> --operator <initials>
```

Use the same pose for the warm-up and the block: teach it once, then pass the
printed `--hold-rad` values so the alignment does not move. The banner prints
the pose geometry, then E, v_tip, p_eff, m_eff and the predicted free-swing
period — compare the period with Sec. 1 step 4 before the first strike.

**Tactile is off by default.** The evidence is the encoder deflection and the
actuator current, and the default run reads nothing from the tactile interface board.
`--tactile` additionally records raw frames of the struck finger in every capture
window (median window 1, fast publish rate; `tac_t`, `tac_seq`, `tac_fz_n` in
the strike's `.npz`), with the wrap guard off because taxels under the striker
approach the 25 N full scale — a wrapped 25.5 N reading is zeroed and counted in
`tac_wrapped` instead.

The runner arms a ring buffer, triggers on a velocity excursion (velocity, not
absolute current: during a torqued hold the velocity baseline is ~0 while
current is pose-dependent), then dumps ±2 s to `impact_events/strike_NNNN.npz`
with a readable `.yaml` sidecar. It prompts for a note after each strike.

**Current at the 600 mA limit is a result to report, not to engineer.** Leave
the operating default in place; `campaign.py` reports the fraction of strikes at
≥ 97 % of the limit and the peak-current fraction. Record the gains and current
limit: they *are* the experimental condition.

---

## 5. Safety and strike hygiene

- Eye protection for everyone in the room. **Nobody's hand in the swing plane**:
  the tip arrives at 2.1 m/s with 0.20 N·s behind it, and the rod is half a
  metre long.
- **Every strike must be a single contact.** The rod rebounds and swings back;
  the half-period is ≈ 0.6 s, enough time to catch it on the return. Verify
  single contact on video for **every** strike; reject and repeat doubles, and
  **log the rejection count** — it belongs in the paper.
- Check the pivot bolt every 10 strikes.
- Macro photo of the pad, the DIP four-bar and its pins every 10
  strikes — a distal strike loads the printed four-bar before the PIP servo.
- **Stop rule:** any visible crack, delamination, permanent set, or any hardware
  error bit → stop, photograph, report.

---

## 6. Tactile honesty statement (mandatory)

The tactile stream ceiling is 83.3 Hz (12 ms), so a 5–10 ms contact transient is
**aliased, not measured**, whatever the reader does. With the default run
(tactile off) write:

> Tactile data were not recorded during the impact test: the sensor's 83.3 Hz
> frame rate cannot resolve a 10 ms transient, so the joint response was
> characterised from the actuator encoders and current alone, with contact
> confirmed on high-speed video. Impact energy is determined geometrically from
> the pendulum and cross-checked against the video.

If the run used `--tactile`, `strikes.csv` also carries per strike: frame rate
achieved, summed Fz in the 0.5 s before the trigger, its peak in the 0.3 s
after, its mean in the settled window 1.5–2.0 s after, the pre→post shift,
taxels active at the peak, wrapped samples, and a contact flag (peak ≥ 0.5 N
above the pre-strike mean). `campaign.py` then reports contact confirmed on *k*
of *N* strikes and the mean and worst baseline shift, and emits the sentence
with those numbers. The peak value is an aliased lower bound of a 10 ms event,
not a force measurement; do not quote it as one.

**Video energy cross-check:** for 10 of the 30 strikes, measure the tip's
inter-frame displacement in the two frames before contact at 240 fps (≈ 8.9 mm
per frame at 2.14 m/s) and compute v directly. Report `v_video / v_geometric`;
2–5 % agreement is a strong, free validation.

---

## 7. What N = 30 buys, and how to phrase a null

Fit a linear trend to cumulative residual vs strike index. `SE(slope) = σ_r/47.4`
for n = 30, so the detectable total drift is `1.27 σ_r`. Joint drift converts to
fingertip displacement at **73 mm/rad** (73.7 for the MCP, 72.2 for the PIP in the
hold pose, from the FK model; `campaign.py --moment-arm-mm`):

| Per-strike residual noise σ_r | Detectable 30-strike drift |
|---|---|
| 0.0005 rad | 0.046 mm at the fingertip |
| 0.001 rad | 0.093 mm |
| 0.002 rad | 0.185 mm |

**σ_r is measured for you:** `campaign.py` derives it from the scatter of the
per-strike residuals themselves, so no separate sham block is needed. Then the
claim is bounded rather than bare:

> Cumulative joint offset over 30 impacts at 0.218 J was −0.0004 ± 0.0013 rad
> (fitted slope × N, 95 % CI), i.e. below 0.13 mm at the fingertip; the
> experiment resolves a cumulative drift of 0.093 mm.

Do **not** re-home between strikes — the residual is only meaningful against a
fixed software zero.

**The encoder residual sees only the actuator side.** A cracked four-bar or a
shifted fingertip is invisible to it. That is what the `post_impact` dial battery
in Sec. 8 is for; it is not optional.

---

## 8. Afterwards

Cool 30 minutes, then:

```bash
python examples/smoke_test.py
sha256sum <config folder>/config.yaml   # must still match the baseline hash
python -m experiments.repeatability --phase post_impact --digit ring   --target-rad -0.230   # the struck finger
python -m experiments.repeatability --phase post_impact --digit middle --target-rad -0.210   # unstruck control
```

The targets match the `post_endurance` runs, which double as the pre-impact
reference, so the before/after spreads are measured at the same pose.
