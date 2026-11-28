# Handoff — ICRA reliability campaign

State as of **10 Sep 2026** (second pass). Branch `<branch name removed>`, last commit
`<commit id removed>` (impact block). **Untracked, not committed:**
`experiments/analysis/fig_reliability_v2.py` and
`experiments/docs/fig_reliability_draft_preview.png` — the current IV-C figure.
Commit them.

Covers the repeatability → endurance → repeatability arc for paper section IV-C
and the impact block for section IV-A (run 7 Sep; see §8 item 7). Still open:
the post-impact dial battery, the ring-baseline decision (§8 item 2b), and the
tactile force scale (§6), which gates every absolute-newton claim.

**Added in this pass (10 Sep), no code changed:** §4.4 campaign-report numbers
now that `dial_mm` exists; §4.5 the analysis behind the endurance figure
(current headroom, per-motor pattern, force-per-mA, box relaxation model); §5 the
five-panel figure spec; §6 the tactile force-scale problem; §7 decisions taken on
rerun, duration and paper framing.

---

## 1. What exists

```
hand_api/          three shipped-API defects fixed, plus FK (see §2)
experiments/
  common/                clock, telemetry, safety, poses, metrics, capture, rig
  sim/                   simulated bus + tactile, for rehearsing without hardware
  bench/                 check_fk.py, measure_read_rate.py
  analysis/              load, figures, campaign report, publication figures:
                           fig_reliability_v2.py    IV-C five-panel figure — CURRENT (untracked)
                           fig_endurance_paper.py   IV-C four-panel v1 — superseded, keep for reference
                           fig_impact_paper.py      IV-A figure
                           campaign.py              per-digit / pooled σ, F-tests, bounds, claim sentences
                           pool_impact_runs.py
  docs/                  SOP_repeatability.md, SOP_endurance.md, SOP_impact.md, this file,
                         fig_reliability_draft_preview.png (untracked)
  repeatability.py  endurance_run.py  impact_test.py
```

Run everything from the repo root with the venv python
(`<venv python>`); plain `python3` has no numpy.

Motor → joint map (from `kinematics.py`, thumb side and roll pair corrected per the URDF
check in §2): m00 thumb IP, m01 thumb MP, m02 thumb side, m03 thumb roll;
m04/m07/m10 index/middle/ring **PIP**; m05/m08/m11 index/middle/ring **MCP**;
m06/m09/m12 index/middle/ring abad. The repeatability battery flexes the
PIP motor to its target with MCP at −0.95 rad.

---

## 2. Defects found in the shipped API

Each of these silently corrupted a measurement. All are fixed and covered by tests.

| Defect | Consequence |
|---|---|
| **Coupled DIP was identically zero** over the whole working range. The lookup table spans positive PIP angles; motor-space flexion is negative, so `clamp=True` pinned every real pose to 0. | `read_joint_pos()` reported all three DIP joints rigid however flexed the finger was. Any FK-derived fingertip position was wrong. |
| **Tactile Fz parsed unsigned.** One LSB of negative baseline drift transmits `0xFF`, decoding as **+25.5 N**. | A full-scale spike from a near-zero contact. Guarded by an opt-in `fz_wrap_threshold_n`. |
| **Failed sync reads returned stale data silently.** | A comms fault was indistinguishable from "nothing moved". `CommStats` now counts reads, failures and stale returns. |

Also added: the temperature/voltage reader (registers were defined but never read),
and a pure-numpy URDF forward-kinematics module validated against pinocchio to
9e-17 m across all 21 links.

**Paper-text corrections found along the way:**

- Measured taxel pitch is **2.82 mm** (52-point fingertip) and 2.52 mm (thumb),
  not the 1 mm in arXiv v1 §III-D — that is a vendor figure for a different
  configuration.
- Measured tactile frame rate is **90.7 Hz**, above the 83.3 Hz quoted.
- The URDF joint limits match Table II exactly on all 13 joints, which also
  confirms the thumb mapping (motor 2 = side, motor 3 = roll —
  `kinematics.py`'s docstring has these the other way round).
- **§III-D "normal force range up to 25 N" is ambiguous.** The per-taxel wire
  encoding is one unsigned byte at 0.1 N/LSB, i.e. 0–25.5 N *per taxel*. Either
  the 25 N is per taxel (and the module total is larger) or the per-taxel byte
  shares the module's full scale (and summing taxels over-reads). Check the
  datasheet before the number is reused. See §6.

---

## 3. Defects found on hardware

These only appeared on the real hand; the simulator has no encoder quantisation
and no acceleration ramp, so dry runs passed throughout.

| Defect | How it presented |
|---|---|
| **Settle gate never closed.** 0.05 rad/s across 13 motors, against a quantisation floor peaking at 0.048. | Every move ran to timeout; every sample was taken mid-motion while `settled` read 0. Gate now 0.10 rad/s on the moving digit only. |
| **Position D gain of 500 self-oscillates.** Measured 90 mrad peak-to-peak at the PIP (~4.8 mm at the fingertip), 60 mA chatter. | Would have blurred the dial needle and corrupted the free-pose wear metric. Both runners now set **D = 200** and read it back from all 13 motors. |
| **Velocity-only gate returned `settled` on the first poll**, before the joint began accelerating. | Reported arrival with the hand still at the open pose. Stillness now has to be *sustained* (`--settle-dwell-s`) and motion must have been *observed* first. |
| **Travel budget starved.** The close move got `hold_s − sampling` ≈ 0.30 s for ~1.4 rad of travel needing ~0.5 s. | Fingers were told to open before arriving — "brush the box and retreat". `--hold-s` now bounds only the press; travel has its own timeout. |
| **Tracking-error abort at 0.6 rad.** | A current-limited grasp *is* the finger stopping short; killed a run at cycle 10. Tracking error is now recorded but never aborts. |
| **Homing motor 9 without its pre-move.** The middle abad stop is only reachable with motor 8 at −90°. | Produced a spurious 4.3° "drift" while every other motor stayed under 0.7°. |
| **Signal handlers absent, then wrong.** | Killing a runner left all 13 motors energised; then a flag-only SIGINT handler made Ctrl-C inert at prompts. Now escalates on a second signal. |

---

## 4. Campaign data

All runs are in `<data folder>/`. Calibration pinned at
`sha256 8d35675d…`, **unchanged across the whole campaign** — the before/after
comparison is valid. `campaign.py` confirms home offsets identical across all
nine repeatability sessions.

**Directory names are misleading.** The four post-endurance repeatability
directories are still *named* `*_repeatability_baseline_*`; only `phase:` inside
`run.yaml` / `run_end.yaml` was relabelled to `post_endurance`. Never glob —
always pass these nine directories explicitly (the exact list is in §9).

### 4.1 Baseline repeatability (6–7 Sep, 23:46 – 00:26)

100 trials per digit, camera mode, pure position control (mode 3). Hand at
37–39 °C during the battery.

| digit | target (rad) | D gain | run |
|---|---|---|---|
| index | −0.170 | *not recorded* (pre-fix, D=500) | `20260906-234607` |
| middle | −0.190 | 200 | `20260907-000438` |
| ring | −0.210 | 200 | `20260907-001922` — **σ = 357 µm, unusable; see §8 item 2b** |
| ring (alt) | −0.190 | 200 | `20260907-001114` — 100 trials, `dial_mm` **not transcribed** |
| thumb | −0.350 | 200 | `20260907-002627` |

### 4.2 Endurance — `20260907-021852`

**17,028 cycles in 12.00 h. Completed, zero operator interventions, zero thermal
pauses, zero hardware errors.** 3.3× arXiv v1's 5,143 cycles. Note `campaign.py`
prints "interventions 1 incident (TACTILE_RECONNECT×9)": those nine are tactile
*frame drops* that the driver re-established automatically, not operator
interventions. The paper calls them "tactile frame drops"; keep that wording.

Conditions: taught grasp on a closed cardboard box grasped across a corner,
−0.10 rad preload, 0.20 s press, 2.5 s period (24 cycles/min), mode 5 at 600 mA,
P=800 D=200. Hold current ~74 mA mean, peak 150 mA: **position-limited, not
force-limited, at roughly a quarter of the current limit.**

Values below are medians over hour 0–1 → hour 11–12 unless stated. Column names
are the `cycles.csv` columns the figure reads.

| Result | Column | Value |
|---|---|---|
| Temperature, hottest motor | `temp_max_c` | 44 °C in the first 30 min → **48 °C** from ~1 h, flat to the end (v1: 49 °C) |
| Unloaded tracking error, worst joint | `open_err_max_rad` | **21.5 mrad steady; total excursion 2 encoder counts** (1 count = 1.53 mrad). The worst joint is **m02, thumb side**; the other twelve sit at 1.5–11 mrad |
| Unloaded tracking error, RMS over 13 | `open_err_rms_rad` | 8.7 → 8.9 mrad |
| Loaded tracking error, max | `closed_err_max_rad` | 75.5 → 69.3 mrad |
| Loaded tracking error, RMS | `closed_err_rms_rad` | **48.3 → 41.6 mrad (−14%)** — panel (c) uses this one |
| Hold current, mean of \|I\| over 13 motors | `cur_hold_mean_ma` | **79.3 → 68.8 mA (−13%)** |
| Peak current per cycle | `cur_peak_ma` | median 150 → 135 mA; **max 183 mA; limit 600 mA; 0 cycles ≥ 590** |
| Grasp force, summed taxel Fz (**scale unverified, §6**) | `fz_sum_n_*` | thumb 5.6 → 3.4, index 6.5 → 6.4, **middle 20.4 → 14.6 (−28%)**, ring 8.7 → 7.5 |
| Middle force per unit current | `fz_sum_n_middle / (\|m07\|+\|m08\|)` | **0.084 → 0.073 N/mA (−11%)** |
| Contact centroid shift | `centroid_mm_*` | **middle +1.37 mm**; thumb −0.07, index −0.20, ring +0.15 |
| Active taxels (> 0.2 N) | `n_active_*` | thumb 13 → 8, **index 10 → 4**, middle 16 → 16, ring 16 → 16 |
| Bus integrity | `comm_reads/failures/stale` | **749,261 reads, 0 failures, 0 stale** |
| Tactile availability | `tactile_ok`, `events.jsonl` | **1 lost cycle in 17,028**; 9 `TACTILE_RECONNECT` at elapsed h 1.14, 3.36, 7.15, 7.33, 8.62, 9.72, 10.21, 10.54, 10.66 |
| Wrapped Fz samples | `n_wrapped_*` | 0 |

Grasp-force drift depends on the reference window. `campaign.py` normalises to
the run's own 300-cycle baseline and reports −24.9 / −1.3 / −31.9 / −9.1 %/10 h
(thumb / index / middle / ring). The earlier handoff quoted −33 / −2 / −39 / −11
from first-vs-last 500 cycles. The figure and the text should use **one** stated
method; hourly medians (first vs last hour) are recommended because the first
30 min contains a seating transient that inflates any first-500-cycle reference.

### 4.3 Post-endurance repeatability (7 Sep, 14:48 – 15:10)

100 trials per digit, targets 0.02 rad deeper than baseline in every case
(`--preload-rad` default applied on a fresh jog). Hand at 39–40 °C, 30 min after
the endurance run ended.

| digit | target (rad) | run (directory still named `baseline`) |
|---|---|---|
| index | −0.190 | `20260907-144802` |
| middle | −0.210 | `20260907-145518` |
| ring | −0.230 | `20260907-150313` |
| thumb | −0.370 | `20260907-151006` |

Post-impact: ring `20260907-175701` (100 trials, −0.230), hand at 37–39 °C.

### 4.4 Repeatability results (campaign report, 10 Sep)

R = 3s, single axis, dial counts 0.0005 in read to 0.1 count (1.27 µm). Means are
fixture-dependent and printed for the record only. Spreads are in mm (no in-situ
scale block was run, so nothing is scale-normalised; all sessions used the same
indicator on the same tab, so mm-to-mm comparison is fair).

| digit | baseline σ / R (µm) | post-endurance σ / R (µm) | ratio | F(99,99), p | 95 % one-sided upper bound on ratio |
|---|---|---|---|---|---|
| index | 6.7 / 20.1 | 4.1 / 12.3 | 0.61 | 0.370, < 0.001 | **0.72** |
| middle | 10.1 / 30.3 | 6.7 / 20.1 | 0.67 | 0.443, < 0.001 | **0.79** |
| thumb | 8.5 / **25.6** (operator re-verified 2026-09-15; workbook still reads 16.2 / 48.7, applied as a derived-value override in `fig_reliability_icra.py`) | 6.8 / 20.3 | 0.42 | 0.174, < 0.001 | **0.49** |
| ring | **357 / 1071 — unusable** | 6.2 / 18.5 | — | — | — |
| ring, post-impact vs post-endurance | 6.2 / 18.5 → 7.4 / 22.1 | | 1.20 | 1.435, 0.074 | 1.41 (not resolved; growth ≤ 41 % not excluded) |

Minimum detectable ratio at n = 100/100 is 1.22. Means (mm): index 0.420 → 0.127,
middle 0.378 → 0.371, ring 0.428 → 0.494 → 0.433, thumb 0.276 → 0.225.

**How to read this — important for the paper.**

- **Every usable digit's spread shrank after 17,028 cycles, and significantly.**
  That is the strongest possible "did not degrade" result, but it is *not*
  evidence of improvement and must not be written as such. Temperature does not
  explain it (37–39 °C baseline vs 39–40 °C after). Index is partly explained by
  the D-gain fix (500 → 200). Middle and thumb were D = 200 both times and still
  shrank 33 % and 58 %. Candidates: running-in of printed parts (real, but
  needs a control to claim), indicator engagement/mounting, reading habit.
  Write: *"post-endurance σ was at most 0.72×, 0.79× and 0.49× baseline for
  index, middle and thumb (95 % one-sided); the protocol resolves a 22 % change
  per digit."* Do not write "improved".
- **The pooled index+middle+ring comparison is contaminated** by the ring
  baseline (pooled s 0.206 mm, largest/smallest 53×). `campaign.py` warns about
  this itself. Report per digit until §8 item 2b is resolved.
- `campaign.py`'s claim sentence prints "did not degrade by more than about
  −28 %" — a negative bound reads as nonsense. Rephrase around the ratio bound.
- The thumb is a different mechanism and is never pooled.

### 4.5 Endurance analysis behind the figure

**Current headroom.** Peak current per cycle: median 150 mA falling to 135 mA,
maximum 183 mA in 17,028 cycles, against the 600 mA limit. No cycle came within
10 mA of it. Mean hold current 74 mA; the hottest motor, **index MCP (m05),
held 132 mA continuously** and set the 48 °C steady state. This is the sentence
that makes the box a stated operating point: *"at roughly a quarter of the
current limit the hand reaches thermal equilibrium at 48 °C"*.

**Per-motor hold current, \|mA\|, hour 0–1 → hour 11–12.** Motors on the box
corner unloaded; motors elsewhere did not. Wear or heating would affect all
loaded motors together; this pattern is object geometry.

| motor | joint | hold current | change |
|---|---|---|---|
| m00 | thumb IP | 70 → 41 | −41 % |
| m01 | thumb MP | 75 → 50 | −33 % |
| m02 | thumb side | 4 → 32 | thumb pushed sideways as the corner collapsed |
| m04 | index PIP | 141 → 104 | −27 % |
| m05 | index MCP | 132 → 132 | flat, hottest motor |
| m07 | middle PIP | 135 → 104 | −23 % |
| m08 | middle MCP | 119 → 98 | −18 % |
| m10 | ring PIP | 147 → 135 | −8 % |
| m11 | ring MCP | 116 → 119 | flat |

**Force per unit current (the discriminator).** In a servo position loop
current ∝ torque, and torque × lever arm = fingertip force, so Fz / I measures
how efficiently the mechanism turns motor effort into contact force. A loosened
four-bar or higher friction would lower it. Middle finger, the only clean contact:
Fz −28 %, flexion current −20 %, **Fz/I −11 to −12 %** (0.084 → 0.073 N/mA).
About 70 % of the force loss is the servo doing less work at the same command
(the object yielding); the residual is the same order as what the 1.4 mm
centroid migration does to a summed reading on a partial patch, and is the
upper bound on any transmission-side change. Index and thumb are unusable for
this ratio: their active-taxel counts fell 10 → 4 and 13 → 8, the contact slid
partly off the sensing area. Ring rebounds after hour 9 with flat current —
load redistribution.

**Peak current is not thermal.** It fell 10 % over the run while temperature was
flat from hour 1, so it tracks the object like hold current does.

**Box relaxation model (panel d).** The middle finger's summed Fz, rolling
median W = 400 cycles, fitted from 0.5 h (seating transient excluded) to
F(t) = F∞ + ΔF·e^(−t/τ) — the stress relaxation of a standard linear solid held
at constant displacement, which is what a stiff servo pressing a viscoelastic
box is:

| F∞ | ΔF | τ | R² |
|---|---|---|---|
| 11.9 N | 11.3 N | **8.2 h** | **0.98** |

Log-time and power-law creep fit equally well (R² 0.98); the SLS form was chosen
because it has a name and a mechanism. Thumb, index and ring (3–8 N) fit no
monotonic model (R² 0.0–0.5) and are drawn unfitted. A joint fit with one shared
τ across all four fingers was tried and rejected (index R² 0.00, ring 0.02).

**What distinguishes object from hand, and what does not.** Servo-side signals
(loaded error falls, hold current falls, Fz falls) are consistent with *both* a
yielding box and growing backlash — a motor that reaches deeper with less effort
looks the same either way. Unloaded tracking error (panel b) does *not* break the
tie: it is encoder-internal and blind to everything distal to the actuator. What
does: (i) Fz/I stationarity within 12 %, (ii) the per-motor pattern above, (iii)
the contact-patch collapse on index/thumb, (iv) the external dial repeatability
(panel e). The stream files hold ~10 samples per cycle and no tactile, so a
per-cycle force–displacement curve to find contact onset is **not** available.

---

## 5. The IV-C figure — `fig_reliability_v2.py`

Single ICRA column, 3.45 × 9.0 in, five panels, (a)–(d) share the 12 h axis.
Output: `<run>/figures/fig_reliability_draft.{pdf,png}`; preview copy at
`experiments/docs/fig_reliability_draft_preview.png`. Colours: `FINGER_COLORS`
in `figstyle.py`, one colour per finger reused across the paper; hold current in
purple `#7C3AED` because it is not a finger. Palette passes the CVD validator
with the ring/middle pair at the floor, so **direct labels on every trace are
required, not decorative**.

| panel | argues | data | notes |
|---|---|---|---|
| (a) Thermal equilibrium in 1 h, zero interventions | hand is thermally stable; comms held | `temp_max_c` line, min–max band over 13 motors; rug of the 9 `TACTILE_RECONNECT` events, labelled "tactile frame drops" in the figure; text: 9 frame dropss, 1 cycle lost, bus 0 errors | a flat zero gets a text line, not a panel |
| (b) Servo tracking unchanged to encoder resolution | actuators still go where told | `open_err_max_rad` rolling median W = 400; band ± 1 count | **TODO relabel ylabel "worst joint, unloaded"** and say "max over 13 joints" in the caption — it is one joint (m02), not a per-finger quantity. It is *not* the wear claim |
| (c) Fingers reach deeper with less effort | the object yielded; hand-side change bounded | three series ÷ own hour-1 median × 100: `closed_err_rms_rad` −14 %, `cur_hold_mean_ma` −13 %, middle Fz/(\|m07\|+\|m08\|) −11 % | one axis, in-panel key lower-left; caption must define `cur_hold_mean_ma` as mean \|I\| over 13 motors |
| (d) Grasp force follows the object relaxing | force decline is the box | `fz_sum_n_*` rolling median, absolute N; SLS fit on middle; F∞ dotted | **axis label must say the scale is uncalibrated until §6 is resolved**; caption defines Fz (baseline-corrected, clipped, summed over taxels) and notes thumb/finger modules have matched taxel area (§6) |
| (e) Fingertip repeatability before vs after | external check the servo panels cannot give | **PLACEHOLDER** — fill from §4.4: per-digit R = 3s baseline vs post-endurance, box or overlaid histograms | index flagged (D = 500 baseline) or dropped; ring baseline dropped; middle and thumb carry it; annotate the one-sided ratio bounds |

Absolute-newton values were deliberately kept in (d) rather than percent, so
that middle's 20 N against 4–8 N for the others is visible and index's early
+55 % contact-migration transient does not become a fake headline.

---

## 6. Tactile force scale — UNRESOLVED; gates every absolute-newton claim

**What `fz_sum_n` is.** Per cycle, one tactile frame at the pressed pose and one
at the open pose. Per taxel: pressed − open (per-cycle drift removal), clipped at
0, then **summed over all taxels**. Per-taxel wire format is one unsigned byte
per axis at 0.1 N/LSB (Fz 0–25.5 N per taxel). `n_active` counts taxels above
0.2 N; `centroid_mm` is the force-weighted centroid using the packaged geometry
CSVs. `fz_max_n` is computed but **not logged**.

**The driver discards the vendor's own total.** Each finger block in the tactile
frame is 6 bytes of resultant force (3 × int16 Fx/Fy/Fz) followed by the
per-taxel array. `hand_api/tactile/tactile_driver.py` `_parse_digits` skips the six
bytes ("skipped here", ~line 399) and returns only the distributed points. A
vendor-calibrated, 16-bit total force is decoded past on every frame and no run
in the campaign has it. Adding it is a few lines in the parser plus columns in
the runners — **not done; the owner said do not modify code. Their call.**

**Why the sum is suspect.**
- 25.5 N per-taxel full scale over ~5 mm² is 5 MPa — physically meaningless, so
  the per-taxel byte is unlikely to be a tributary-area force partition.
- Cross-check against the motors: at the start of the run the four summed Fz
  totalled 41 N with the 13 motors averaging 74 mA; the middle finger alone read
  20.4 N at 135 mA in its flexion motors. v1's fingertip load test overheated an
  actuator at 11.8 N. **The sum probably over-reads by several ×.**

**Density.** Summing is density-invariant only if each taxel reports the load on
its own patch with no overlap. Elastomer crosstalk double-counts, and a denser
array then reads a larger sum for the same load. For this hand the concern does
not bite *across modules*: fingertip ZCP-F561 is 52 taxels at 2.82 mm pitch over
263 mm² (5.05 mm²/taxel, 19.8 /cm²); thumb ZCP-DG1Z is 127 taxels at 2.52 mm over
622 mm² (4.90 mm²/taxel, 20.4 /cm²). Per-taxel area differs by 3 %, so thumb vs
finger traces in panel (d) are on the same footing. Say so in the caption.

**What survives a constant scale error:** τ, the −28 % decline, the Fz/I
*relative* change, every non-tactile panel. **What does not:** any value in
newtons.

**Fix path, in order of value.** (1) Run the plan's §2.A.1 in-situ gauge
calibration (press each fingertip on the force gauge through a flat probe,
0–20 N stepped, 3 cycles, 4 digits) → slope, R², RMSE, hysteresis per digit;
already scheduled for IV-D and now has a second job. (2) Log the resultant on
all future runs. (3) If the factor is constant, rescale the 12 h data
retrospectively. Until then label the axis "summed taxel Fz (N, uncalibrated)".

---

## 7. Decisions taken; paper framing for IV-C

- **The 12 h data is usable. Publish on it. No rerun for duration, no 24 h.**
  12→24 h moves no reviewer; what they read for is an external degradation
  measurement with a stated resolution, which is §4.4. Abstract/plan lines
  saying "24 h / 60k" become "12 h / 17k".
- **The box is a protocol decision, stated, not a caveat confessed.** A rigid
  object under a fixed position command is a 17,000-cycle stall test on the one
  unit still needed for other work. Write: *"loaded to a representative,
  non-stall grasp on a compliant object (mean hold 74 mA, peak 150 mA against a
  600 mA limit), so the test measures cyclic wear rather than actuator thermal
  limits, which IV-B characterises separately."*
- **Optional, not required:** one overnight control run on a firmer non-crushing
  object (the silicone-sleeve-over-aluminium the README names), small preload,
  300 mA limit, plus ~1 h dial battery next morning. Frame as an *object
  control*, not a second endurance test; wear claims stay based on run 1.
  Gives a flat trace beside the relaxing box in (d) and >30k cumulative cycles.
  Only if a night is free before the 15th.
- **Hand-time priority:** impact battery (post-impact dial, §8 item 7) and IV-F
  policy demos before any extra endurance.
- **Claim order in IV-C**, each unconditional on the box: (1) 12 h, 17,028
  cycles unattended, zero operator interventions; (2) thermal steady state 48 °C
  in 1 h at a quarter of the current limit; (3) 749,261 bus reads / 0 failures,
  tactile 17,027 of 17,028 cycles, 9 board dropouts self-recovered; (4) unloaded
  servo tracking within 2 encoder counts; (5) post-endurance repeatability ≤
  0.72× / 0.79× / 0.49× baseline (index / middle / thumb, 95 % one-sided). Then
  the object, in two sentences, as an observation the hand's own telemetry
  resolved: loaded error −14 %, hold current −13 %, middle centroid +1.4 mm, load
  redistributed middle → ring after hour 9, Fz/I within 12 %.
- **Never write:** unqualified "no degradation"; "grasp-force drift" as a hand
  metric; "repeatability improved"; an exponential fit on the non-monotonic
  fingers; any newton value without the §6 caveat.
- **Section IV-C text changes vs arXiv v1:** Fig. 9 + Fig. 10 → this one figure;
  define R = 3s; replace "5,143 cycles / 49 °C" with the new numbers; move the
  "0.051 mm / 0.207 mm" sentence out (the mean is fixture-dependent).

---

## 8. Open items

**1. Done (10 Sep): the four post-endurance runs are relabelled `phase: post_endurance`**
(`run.yaml` and `run_end.yaml`, originals kept as `*.orig`, audit entry under
`corrections`). Directory names were **not** renamed — see §4.

**2. Done (10 Sep): `dial_mm` is filled for all nine batteries** from
`<data folder>/Repeatability_Data.xlsx` (sheet per digit and time point,
100 rows in trial order; the dial counts 0.0005 in, read to 0.1 count = 1.27 µm).
`trials.csv.orig` keeps the empty originals. The ring post-impact battery is
`20260907-175701` (the 17:56 run is a 2-trial abort). Results are in §4.4.
**Pass the nine run directories explicitly** — a glob sweeps in the aborted
stubs and the duplicate-run guard then drops the whole time point.

**2b. Ring baseline battery (`20260907-001922`) needs a decision.** σ = 357 µm
is 50× every other battery; either the indicator was not engaged or the wrong
run was transcribed. The other 100-trial ring baseline, `20260907-001114`
(target −0.19, 39 °C, D = 200), has `dial_mm` all NaN — check whether the xlsx
ring-baseline sheet actually belongs to it (trial timestamps `t_read_wall` vs
the video). Until resolved, the ring's endurance comparison is not reportable
and the pooled comparison is unusable; middle and thumb carry the claim.

**3. Pose mismatch between time points.** Every post-endurance target is exactly
0.02 rad deeper than its baseline (the `--preload-rad` default applied on a fresh
jog). Small — 0.02 rad is ~1.1 mm at the fingertip — but it means the two spreads
were measured at slightly different configurations. Use `--target-rad <baseline
value>` at the next time point to reproduce the pose exactly.

**4. Index baseline ran at D=500**, where the joint self-oscillates; the other
three and all post-endurance runs are D=200. Its σ ratio of 0.61 is therefore
partly the gain fix. Either exclude index from the pooled comparison or note it
explicitly. Middle and thumb are clean.

**5. Duplicate and stub runs.** Several n<10 aborted runs remain (five index
baseline attempts, two middle, one post-endurance index pair, one post-impact
ring). `campaign.py` flags duplicates rather than averaging them, but deleting
the stubs is cleaner. Do not delete `20260907-001114` until 2b is decided.

**6. Object not characterised.** The SOP's pre/post indentation measurement on
the box was not done, so the crushing is inferred from the loaded-error trend,
the current pattern and the centroid migration rather than measured directly. If
the box still exists in its fixture, a caliper on the grasped corner converts
the inference into a number for the caption.

**7. Impact block done (7 Sep 2026), post-impact dial battery pending.**
Ring finger, 60° release (manifests corrected from the 45° typed on the command
line; originals kept as `*.orig`): 373 mJ, 2.80 m/s, 95 g effective mass. Two
runs, `20260907-173702_impact` (30) and `20260907-175210_impact` (8), merged
into `20260907-173702_impact_pooled`: **30 usable of 38** after `exclusions.yaml`
(notes 0/1 = miss, peak < 5° = glancing, then the first 30 usable kept because the
block was planned as N = 30; the 31st usable strike is set aside, not deleted). PIP deflected 6.3 ± 0.6° (max 7.5° of
12.1° free travel), swung back through the hold pose at ~90 ms, undershot 1.8°, and
settled within ±1° by ~165 ms (median); peak current 262 mA mean / 318 max,
never at the 600 mA clip; no hardware errors. Settled hold pose over the block:
MCP +0.04 ± 0.07°, PIP +0.27 ± 0.21°, ABD +0.29 ± 0.45°. Condition: P = 800,
D = 500 (the impact runner does not lower D as the other runners do), mode 5,
600 mA. Figure: `python -m experiments.analysis.fig_impact_paper <pooled dir>`,
panel (d) is a placeholder until the dial readings exist. Still to do:
`repeatability --phase post_impact --digit ring --target-rad -0.230` and
`--digit middle --target-rad -0.210`, then transcribe the needle from video.
(The ring post-impact battery `20260907-175701` now exists and is transcribed —
§4.4; the **middle control** battery is still missing.)
The original planning note follows. The striker is the
285 g / 533 mm rigid stick swung about the hole at one end (`--striker rod`, the
default, modelled as a uniform rod): 45° gives **218 mJ at 2.14 m/s with a 95 g
effective mass**, 2.5× the original 100 g bob's energy. Two things the SOP now
insists on: calibrate the stick (weigh after capping, measure the hole offset,
balance it, time 20 swings against the predicted 1.196 s period), and mount the
hand **palm-up, nose-down ≈ 32°** — in the hold pose the pad normal is 58° off
the palm normal, so an upright hand is not struck squarely. `--capture-hz` must
be set from `measure_read_rate.py` first — the default 500 is a placeholder.
The struck finger is the **ring** (`--digit ring`, now the default); post-impact
battery: ring plus middle as the unstruck control, at the post_endurance targets
(−0.230 / −0.210). The hold pose is the operator's:
`--teach` (torque off on the struck finger, pose by hand, ENTER) or `--hold-rad`;
the runner prints and stores free travel, pad-facing tilt and lever arms for
that pose, and `campaign.py` uses them. Tactile is off by default — the encoder
deflection and current spike are the evidence; `--tactile` would add raw frames
(`tac_*` columns and the npz).

**8. Tactile force scale unverified (§6).** Gates every newton in IV-C and IV-D.
Run the in-situ gauge calibration; label the (d) axis as uncalibrated until then.

**9. Vendor resultant force is discarded by the parser (§6).** Code change,
deliberately not made in this pass. Decide, then do it before any further
tactile-bearing run so the next dataset has both estimates.

**10. Figure finishing** (`fig_reliability_v2.py`): fill panel (e) from §4.4;
relabel (b) "worst joint, unloaded"; (d) axis "summed taxel Fz (N, uncalibrated)";
caption items listed in §5; then commit the script and the preview.

**11. Paper text.** §III-D 25 N per-taxel vs per-module ambiguity (§2); one
stated method for grasp-force drift (§4.2); "zero operator interventions" not
"zero incidents" (§4.2); rephrase `campaign.py`'s negative-percent claim sentence
(§4.4).

---

## 9. Reproducing the analysis

```bash
PY=<venv python>
E=<data folder>

# acceptance check on the endurance run
$PY -m experiments.analysis.plot_rehearsal $E/20260907-021852_endurance

# the current IV-C figure (five panels; (e) is a placeholder until wired to campaign.py)
$PY -m experiments.analysis.fig_reliability_v2 \
    $E/20260907-021852_endurance \
    $E/20260907-021852_endurance/figures/fig_reliability_draft
# prints the SLS fit parameters and the reconnect times

# superseded v1 four-panel figure, kept for reference
$PY -m experiments.analysis.fig_endurance_paper $E/20260907-021852_endurance \
    $E/20260907-021852_endurance/figures/fig_endurance_paper

# full campaign report — pass the nine batteries EXPLICITLY (directory names lie; see §4)
$PY -m experiments.analysis.campaign \
  $E/20260906-234607_repeatability_baseline_index \
  $E/20260907-000438_repeatability_baseline_middle \
  $E/20260907-001922_repeatability_baseline_ring \
  $E/20260907-002627_repeatability_baseline_thumb \
  $E/20260907-144802_repeatability_baseline_index \
  $E/20260907-145518_repeatability_baseline_middle \
  $E/20260907-150313_repeatability_baseline_ring \
  $E/20260907-151006_repeatability_baseline_thumb \
  $E/20260907-175701_repeatability_post_impact_ring \
  $E/20260907-021852_endurance --latex
```

`campaign.py` produces the per-digit and pooled σ comparisons, the F-tests, the
95% one-sided bound on the σ ratio, LaTeX table rows, and the claim sentences.
Pooling index/middle/ring (same module) gives df = 297, which resolves a change
in σ of **12%**, against 22% for any single digit — but see §4.4: the pool is
unusable until the ring baseline is resolved.

---

## 10. Statistics the paper must state

Verified by Monte Carlo (coverage 0.946–0.957 against a nominal 0.95):

- **Define the statistic.** `R = 3s`, single-axis, ISO 9283-flavoured. v1's
  undefined "0.051 mm" was this, and it was never explained.
- **Never claim an unqualified null.** With n = 100 per digit the protocol
  resolves a σ change of 22%; pooled across the three identical fingers, 12%. The
  claim is *"repeatability did not degrade by more than X%"*, not *"is
  unchanged"*. With σ having *decreased* in every usable digit, the natural form
  is the one-sided upper bound on the post/pre ratio (0.72 / 0.79 / 0.49).
- **Never claim improvement** from a σ decrease across sessions without a
  same-session control; see §4.4.
- **Only spreads compare across time points.** The indicator is re-mounted, so
  the absolute mean is fixture-dependent — v1's "average 0.207 mm" is that number
  and should not be differenced.
- **Unloaded tracking error is encoder-internal**, not externally tracked. It
  measures servo tracking and is blind to everything distal to the actuator
  output. It is the *servo health* signal; panel (e) is the *wear* signal.
- **Summed taxel Fz is not a calibrated force** until §6 is resolved. Trends and
  ratios are safe; newtons are not.
- **State the drift method** (window, statistic) once, and use it everywhere.
