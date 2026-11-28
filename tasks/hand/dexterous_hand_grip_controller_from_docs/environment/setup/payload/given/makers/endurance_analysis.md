# Reliability analysis and processing record

The paper bundle contains exactly four deliverables: `paper_draft.md`,
`fig_reliability_icra.pdf`, `fig_reliability_icra.png`, and this file.
The PDF is a 3.5 × 5.4 in vector figure with embedded TrueType fonts, four
panels stacked in one column; use it at single-column width. Text is 7 pt at
that size. The PNG is
400 dpi. The single reproduction script is
`experiments/analysis/fig_reliability_icra.py`.

## Figure selection and changes

Four panels address four distinct observations:

| Panel | Evidence | Scope |
| --- | --- | --- |
| (a) Temperature | Thermal operation of all 13 actuators over 12 h; tactile recovery events | Observed thermal behavior at this duty cycle |
| (b) Dial repeatability | Before/after single-axis spread at each tested digit configuration | Retained post-test precision, with session differences disclosed |
| (c) Joint tracking | Both loaded and open per-digit errors throughout cycling | Object-constrained loaded equilibrium and repeatable return toward the open command |
| (d) Local tactile variation | Variation among repeated cycle responses within short windows | Continued module responsiveness and measured local consistency |

Panel (c) replaces the separate normalized tactile-amplitude panel. The
current and response-magnitude trends are retained numerically below and in
the paper text. Those curves largely repeat the changing loaded-equilibrium
trend; they do not need separate main-figure panels. Temperature, endpoint
repeatability, tracking, and tactile consistency answer different questions.
The old companion loading figure, audit figure, SVG copies, CSV/JSON summaries,
LaTeX fragments, and redundant README are consolidated into these four files.

## What was measured at contact, and whether this is an impact test

The run requested a joint motion-profile velocity of
**5.0 rad/s**
(286.5°/s) and acceleration
**60.0 rad/s²**. These are configured
trajectory parameters; they do not establish achieved approach velocity,
fingertip linear velocity, or velocity at first contact.

`endurance_run.py` calls `move_and_settle` before `sample_settled`. During
closing, the gate uses a 0.10 rad/s speed threshold sustained for 0.12 s;
it deliberately does not demand reaching the position command through the
object. The helper reads telemetry while moving, but those transient reads
are not saved. The runner then acquires five loaded samples at nominal
20 ms intervals, releases the object, waits, and acquires five open samples.
Both arrays are saved only once every 100 cycles. All cycle-level settling
flags are true; these flags alone do not establish zero position error.

The stream contains **1,710 rows from 171 cycles**:
855 loaded and 855 open samples. The largest absolute
logged velocity is **0.047962 rad/s** in both
phases. The median row-wise maximum absolute speed is
0.023981 rad/s. These small, quantized
values describe the settled sampling windows, not a slow approach or a
measured deceleration at contact.

Stream `t_elapsed_s` is assigned while writing the arrays, after both phases
have already been sampled. Each cycle's ten rows have only
1–3
distinct timestamps. They are not acquisition timestamps. Differentiating
these positions against the saved timestamps, interpolating a contact
waveform, or fitting an impact model would manufacture temporal information.
The five reads are also insufficient to identify a current spike during impact.

**Recommended description: cyclic loaded grasping endurance, with repeated
object contact.** The logs do not characterize a continuous impact test.
They establish neither contact velocity nor impact energy, impulse, peak
collision force, or collision-current peak. The configured 5 rad/s may be
reported as a setting. The separate dedicated impact experiment and its
worksheet are excluded from this section.

## Current comparison and loading evidence

For the same 171 sampled cycles in each phase, first average each
motor's five signed current readings, take its magnitude, then average across
the digit's two flexion motors. Report the median of these per-cycle values.
The phase comparison uses this same estimator and cycle subset for both phases:

| Digit | Open median (mA/motor) | Loaded median (mA/motor) | Loaded/open ratio |
| --- | --- | --- | --- |
| Thumb | 12.6 | 45.5 | 3.61 |
| Index | 14.2 | 125.6 | 8.85 |
| Middle | 16.5 | 108.5 | 6.58 |
| Ring | 14.3 | 130.5 | 9.13 |

These are changes between settled poses, not the instantaneous current jump
at contact. Joint posture and internal mechanical loading also change, so the
ratios are not force ratios and the open current is not a calibrated tare.
The largest individual streamed currents are 163 mA
loaded and 53 mA open. In the full
17,028-cycle file, `cur_peak_ma` reaches
183 mA: that field is the maximum of the
five loaded samples in a cycle, not the peak over approach, contact, and release.
The larger maximum in the cycle file follows from its denser cycle coverage.
The 600 mA setting is a goal-current limit, not a
measured sustained current or force. No torque or newton conversion is applied.

The nonzero tactile response, loaded joint shortfall, and elevated settled
current jointly support repeated resistive contact. The recorded 0.20 s
settled-hold windows sum to **56.79 min**
(7.89% of elapsed time). Contact during
approach/settling is additional and was not separately timed. This percentage
is not the total contact duty cycle. Avoid both “continuous large load” and a
quantitative “high-force” claim without a force measurement.

## Tracking computation and every-joint audit

Use all 17,028 cycles. For each cycle, the runner averages five position reads
per phase. Compute the RMS command–measurement difference across the actuated
joints of each digit (four for the thumb, three for each other digit), then
take the median of this per-cycle RMS in each five-minute window:

$$E_{d,k}=\sqrt{\frac{1}{n_d}\sum_{i\in d}
                  (q_{i,k}-q_i^*)^2}\,\frac{180}{\pi}.$$

The open target is zero; loaded targets come from `grasp_pose_rad` in the
manifest, including the recorded flexion preload. These are actuator joint
errors, not external fingertip errors. Solid curves show loaded measurements;
dotted curves with hollow markers show open measurements. No fitting,
detrending, or additional smoothing is applied to these bin medians.
First/last-hour summaries below are medians of individual cycle metrics, not
medians of the displayed bin medians. Hold current uses the two flexion
actuators (m00/01, m04/05, m07/08, m10/11), with the signed-sample averaging
order described above.

| Digit | Open RMS, first → last h (°) | Loaded RMS, first → last h (°) | Loaded flexion current, first → last h (mA/motor) | Maximum cycle open RMS (°) |
| --- | --- | --- | --- | --- |
| Thumb | 0.6722 → 0.7140 | 1.7423 → 1.2494 | 73.2000 → 45.5000 | 0.8233 |
| Index | 0.3244 → 0.2415 | 3.1866 → 2.7756 | 136.6000 → 118.0000 | 0.5465 |
| Middle | 0.4379 → 0.4230 | 3.0091 → 2.4071 | 127.0000 → 101.5000 | 0.6010 |
| Ring | 0.3943 → 0.4354 | 3.1320 → 3.0040 | 131.5000 → 127.0000 | 0.6438 |

The open per-digit RMS remains below 0.83° even before binning. This is not
a bound on every individual joint. The loaded residual is a contact-constrained
control equilibrium, so its decrease is not evidence of increasing intrinsic
positioning accuracy. Its decrease alongside current and tactile changes is
consistent with changes in resistance or contact geometry. Box deformation,
sensor response changes, and mechanical changes cannot be separated here.

To avoid hiding one joint behind an aggregate, the following are signed
first/last-hour medians for **all 13 actuators**. Flexion is negative, so a
positive loaded-minus-target value means less flexion than commanded.

| Motor | Open first h (°) | Open last h (°) | Loaded minus target first h (°) | Loaded minus target last h (°) |
| --- | --- | --- | --- | --- |
| m00 | -0.1758 | -0.1758 | +1.9503 | +1.1592 |
| m01 | +0.0000 | +0.3516 | +2.2139 | +1.5108 |
| m02 | +1.1426 | +1.2305 | -0.0879 | +0.8789 |
| m03 | +0.4395 | +0.6152 | -1.7578 | -1.4063 |
| m04 | -0.4784 | -0.3905 | +3.9718 | +2.9171 |
| m05 | -0.1411 | -0.0532 | +3.7960 | +3.7960 |
| m06 | +0.0846 | +0.0670 | +0.5273 | +0.4395 |
| m07 | -0.6541 | -0.6541 | +3.9717 | +3.0928 |
| m08 | -0.0356 | -0.2114 | +3.3565 | +2.7589 |
| m09 | +0.1302 | +0.2181 | +0.2637 | +0.3691 |
| m10 | -0.4784 | -0.5663 | +4.3234 | +3.9718 |
| m11 | -0.3871 | +0.3160 | +3.2511 | +3.3565 |
| m12 | +0.0912 | +0.4428 | +0.0879 | +0.1758 |

In particular, nearly constant worst-joint error does not prove that every
joint stayed constant: ring MCP (m11) changes within the open pose.
The figure retains both phases and all digits instead of reporting only the
single largest error across the hand.

## Repeatability data, correction, and interpretation

All eight workbook sheets are matched in trial order to their logged
`trials.csv` files. The impact worksheet is never read. Each session has
100 readings with a true settling flag. One count is 0.0127 mm; the video
transcription increment is 0.1 count = 1.27 µm. That increment is not an
indicator accuracy or uncertainty claim. All 800 readings are retained.

The operator confirmed `Ring_Pre!B39`, trial 37, as **31.2 rather than 312
counts**. Only the derived value changes: 3.9624 mm → 0.39624 mm. The original
workbook and matching CSV entry remain unchanged. This is a documented
decimal transcription correction, not a statistical outlier rule.
In addition, the **thumb before** 3σ is an operator-verified override: **25.6 µm** is used where the stored workbook readings give 48.7 µm. Operator re-verified the thumb baseline on 2026-09-15: 3-sigma is 25.6 um, not the 48.7 um the stored Thumb_Pre readings give. Workbook not yet updated. Applied to derived values only; the workbook and trial log are unchanged, and drift/lag statistics for that session are still computed from the stored readings.

Report **3 × sample SD (ddof=1)** in micrometres for each complete session.
There is no detrending, trimming, trial rejection, pooling across digits,
or fitted correction. “Drift” below is the last 20-trial mean minus the first
20-trial mean; it is a descriptive diagnostic, not a removed component.

| Digit | Before 3σ (µm) | After 3σ (µm) | Before drift (µm) | After drift (µm) | Before/after lag-1 correlation |
| --- | --- | --- | --- | --- | --- |
| Index | 20.1315 | 12.2532 | -0.7620 | -5.5245 | 0.736 / 0.728 |
| Middle | 30.2506 | 20.1246 | -25.0190 | -5.8420 | 0.847 / 0.399 |
| Ring | 19.4337 | 18.4567 | -5.5880 | +9.9060 | 0.303 / 0.348 |
| Thumb | 25.6000 | 20.3299 | -41.0845 | +1.0160 | 0.924 / 0.840 |

All post-test spreads happen to be smaller, but their reduction does not
identify a mechanical improvement. The thumb and middle baselines have
substantial within-session drift; several sessions also show serial
correlation. Treating all readings as independent samples for an F-test or
a tight wear bound would therefore be unjustified. The index gain differs
and every post-test target is 0.02 rad more flexed. Smaller spread is compatible
with preserved precision, less session drift, and different configurations;
its cause cannot be assigned from this experiment. No rerun is assumed.

| Digit | Session | Target (rad) | D gain | Maximum-motor temperature range (°C) | Source run |
| --- | --- | --- | --- | --- | --- |
| Index | before | -0.170 | 500 nominal; no readback | 37–37 | `20260906-234607_repeatability_baseline_index` |
| Index | after | -0.190 | 200 | 39–40 | `20260907-144802_repeatability_baseline_index` |
| Middle | before | -0.190 | 200 | 38–38 | `20260907-000438_repeatability_baseline_middle` |
| Middle | after | -0.210 | 200 | 39–40 | `20260907-145518_repeatability_baseline_middle` |
| Ring | before | -0.210 | 200 | 38–39 | `20260907-001922_repeatability_baseline_ring` |
| Ring | after | -0.230 | 200 | 39–40 | `20260907-150313_repeatability_baseline_ring` |
| Thumb | before | -0.350 | 200 | 38–39 | `20260907-002627_repeatability_baseline_thumb` |
| Thumb | after | -0.370 | 200 | 39–39 | `20260907-151006_repeatability_baseline_thumb` |

Saved home offsets agree across all eight sessions. This does not establish
identical external fixture alignment. Before/after mean dial offsets are not
used as an absolute accuracy metric. The physical contact fixture and an
independent indicator scale check were not confirmed in the available context;
the draft does not invent them. The claim is observed post-endurance
single-axis spread below 21 µm at the tested configurations, not statistical
equivalence of matched pre/post conditions.

## Tactile processing, smoothing, and availability

The starting observations are the logged `fz_sum_n_*` cycle summaries, not raw
unfiltered taxel measurements. At acquisition, the unsigned normal-channel
parser had its wrap guard enabled (values above 20 in the decoded scale were
folded by subtracting 25.6), the driver used a three-frame rolling median and
a nominal 60 Hz publication rate, and startup recalibration was recorded.
These acquisition steps predate this analysis. The zero `n_wrapped_*` values
in the cycle file are not a raw count of all corrections applied by the parser.

For each digit and cycle the runner computes the positive part of the loaded
minus that cycle's subsequent open taxel values, then sums across taxels:

$$S_{d,k}=\sum_j\max(z^{\mathrm{closed}}_{d,j,k}
                              -z^{\mathrm{open}}_{d,j,k},0).$$

The column's `_n` suffix is not taken as proof of a verified force scale.
Taxels also have different local normal directions, so a scalar sum is not
a resultant force vector. The analysis treats this as a relative response.

There are 144 fixed, non-overlapping five-minute bins starting at elapsed
time zero. The final bin includes the endpoint slightly beyond 12 h.
Each bin has 117–120
valid cycle observations. Missing values are omitted from tactile-value
statistics and counted as unavailable; they are never filled with zero or
interpolated. All finite amplitudes, including early large thumb values,
contribute to the statistics.

For each bin compute local relative SD, **100 × sample SD / sample mean**.
Panel (d) shows every original five-minute estimate as a faint point, plus a
centered arithmetic mean of five consecutive estimates (25 min) for display.
At the first/last two bins use the available three or four bins, without
padding or extrapolation. Apply the same rule to all four digits. The raw
cycle response is not smoothed again before computing SD. All reported
numerical CV values use the unsmoothed estimates. This smoothing cannot be
used to claim that sensor noise was reduced.

Response-magnitude change below is the last-hour median of cycle responses
divided by the first-hour median, minus one, in percent. Local SD first/last
hour values are medians of the twelve original five-minute SD/mean estimates
in those hours. No normalized-response plot is exported.

| Digit | Response change, first → last h (%) | 5-min relative SD, first → last h (%) | Median relative SD, all bins (%) | Maximum relative SD (%) |
| --- | --- | --- | --- | --- |
| Thumb | -39.29 | 33.12 → 6.07 | 12.54 | 38.82 |
| Index | -1.54 | 23.13 → 3.74 | 10.34 | 28.37 |
| Middle | -28.43 | 6.52 → 3.62 | 3.41 | 8.79 |
| Ring | -13.79 | 5.73 → 8.15 | 10.56 | 18.47 |

For a concrete early-thumb example, the 10–15 min window contains
118 valid cycles, of which 11 have response above twice
the window median. Their median active-taxel count is
23, versus 13 in the
remaining cycles; response and active count have correlation
0.992. This is an exploratory
description of that window, not an exclusion threshold or an independent
cause test: active count and response derive from the same taxels. The data
can reflect changing engagement and/or sensor behavior. Those cycles remain
in the figure's variation estimates.

Relative SD separates local variation from the long-term response level only
partially: within-bin drift remains, and a changing denominator changes the
ratio. It is not intrinsic electronic noise, calibrated sensor repeatability,
or proof of unchanged force sensitivity. A fixed unknown multiplicative scale
cancels; additive effects, nonlinearities, and time-varying response do not.

Complete derived readings require finite responses from all four digits.
There are **17,019/17,028 complete cycles
(99.947146%)**, with a positive response from every digit
on those cycles. There are 9 tactile frame drops, each a
sensor-stream restart that the driver re-established automatically.
Missing zero-based cycle indices: `[1613, 4771, 10150, 10398, 12230, 13790, 14490, 14954, 15125]`.
`tactile_ok` alone misses unavailable open references and would incorrectly
count only one lost derived cycle. Availability here is a fraction of sampled
cycles, not continuous-time uptime. Module-level response does not establish
the health of every individual taxel.

## Operational totals and scope

- Completed cycles: 17,028; duration: 12.0007 h;
  average throughput: 23.6487 cycles/min.
- Maximum observed actuator temperature: 48 °C;
  thermal-pause cycles: 0; cycles with a hardware-error
  flag: 0.
- Motor-bus reads: 749,261; exhausted-retry failures:
  0; stale returns: 0;
  unavailable returns: 0.
- Lower-level communication-result counts: `{-3002: 7, -3001: 9}`.
  Their sum is 16, so “no communication errors” would be incorrect.
- Writes: 34,093; write failures: 0.

One hand and one compliant object were tested at this operating condition.
The evidence supports sustained cyclic operation, measured post-test precision,
and continued tactile responsiveness with automatic recovery. It does not
establish a population failure rate, unlimited lifetime, a calibrated high-force
rating, matched-condition repeatability equivalence, or quantified impact
resistance. The manuscript reports these observations without a fitted
box-relaxation correction, inferred wear bound, or invented transient data.

## Reproduce and verify

From `<repository root>`:

```bash
<venv python> -B -m experiments.analysis.fig_reliability_icra
```

Optional `--data-root` and `--out` arguments select input/output roots. The
script regenerates the four deliverables, including the paper text. It checks
workbook/log agreement, units, trial order, the exact correction, common
home offsets, contiguous cycles, stream phase/sample counts, and full bin
coverage. Input hashes are checked again after generation. No source data
are rewritten. Older handoff and dedicated impact files are outside this
consolidated bundle and were left intact.

The endurance manifest records clean acquisition commit
`<commit id removed>`. The stream-sampling and writing
logic in `experiments/endurance_run.py` agrees with that revision. The later
`common/hardware.py` change adds optional impact-capture driver settings;
it does not change this run's move-and-settle or default tactile behavior.

### Experimental input fingerprints

| Input | SHA-256 |
| --- | --- |
| `<data folder>/Repeatability_Data.xlsx` | `50f93d66b17daac4f1c0df0d5ed97b7470d78cdae8a31dc2e1c1cffb216405ea` |
| `<data folder>/20260907-021852_endurance/cycles.csv` | `b9e7a5ad78b56367b034b39e6c99de1daa9838f2304f474bf52142d7f0e379b7` |
| `<data folder>/20260907-021852_endurance/events.jsonl` | `25fc0226dbe492b4a21d326b088c9856ca34590495aa34b74fcda70dac18d869` |
| `<data folder>/20260907-021852_endurance/run.yaml` | `2d279dce320f10357f53fa94bc86d60636f621e5d5f1c6e2fc25e5c71268d59b` |
| `<data folder>/20260907-021852_endurance/run_end.yaml` | `a6d1a73c833e78188a1af41f66856058767809198ee6e1c64c9cd79991ec33e8` |
| `<data folder>/20260906-234607_repeatability_baseline_index/run.yaml` | `898a42fa21f2ebe16600b59b2df899d43447d98585bdc29414ecdb7905b2c9c3` |
| `<data folder>/20260906-234607_repeatability_baseline_index/run_end.yaml` | `69fb5027129f4795eb4c43ac45b41aa11b162dc989942b5fdf7c80ad64ab7dae` |
| `<data folder>/20260906-234607_repeatability_baseline_index/trials.csv` | `d77453550e07408afdcf062819f3ad6382bf834b763298bfd73d0cbd109bd432` |
| `<data folder>/20260907-144802_repeatability_baseline_index/run.yaml` | `444e3a7afc2cda1429b34633aa8399e404c8e066bf367db9c27652d42c6b0c1d` |
| `<data folder>/20260907-144802_repeatability_baseline_index/run_end.yaml` | `36e5998e7b5cd99ca8f178054f324114ef538c01e90f5796ffc1e9b7d962289f` |
| `<data folder>/20260907-144802_repeatability_baseline_index/trials.csv` | `e4dcee959b2eac8c415e11dce603324d34cf91255f25283c5f9c2fa0632c1530` |
| `<data folder>/20260907-000438_repeatability_baseline_middle/run.yaml` | `cf1ca9f6fe213f03871fca50cc32d90eb585c584e7ff62d2453c8864bdc593e1` |
| `<data folder>/20260907-000438_repeatability_baseline_middle/run_end.yaml` | `db90ec206f08035f06b3898fbda42ce6e7eda4e8c1ab6fc37a0421a6f440cc88` |
| `<data folder>/20260907-000438_repeatability_baseline_middle/trials.csv` | `1f82b2978c3aeb484184b8d253592e8abb250ef8520499d0fa78688a1eef9206` |
| `<data folder>/20260907-145518_repeatability_baseline_middle/run.yaml` | `50aac00933bf10ecaf4fee5461a6a9eb6963ee3cd12150e0a88816f6ac14d0e2` |
| `<data folder>/20260907-145518_repeatability_baseline_middle/run_end.yaml` | `fc4158443ab83cb59fe8a5da442ce6c2c2312fe05f4578940e8fe1767a6ad42e` |
| `<data folder>/20260907-145518_repeatability_baseline_middle/trials.csv` | `1fc4e804ba9c2fac4aa18edc2958374a2686d0f28a94559a0ce36e57251d8de0` |
| `<data folder>/20260907-001922_repeatability_baseline_ring/run.yaml` | `b444fc509c983bda97f5a0f744e05957e57898f092f2b7450bfc9511750ec9cd` |
| `<data folder>/20260907-001922_repeatability_baseline_ring/run_end.yaml` | `4769695afd4ec838e89cc5e5da8a1ad19c332cefb125c9ee1db06f6cc108d16b` |
| `<data folder>/20260907-001922_repeatability_baseline_ring/trials.csv` | `a83f3d28dccff2268dad628401199e55d2f8efb4be1512b2c09094eed8943e0e` |
| `<data folder>/20260907-150313_repeatability_baseline_ring/run.yaml` | `d17ba8efa18f7ba738e90b94c7e01cddeb17e5e9349da10d72ebc5177c70cea3` |
| `<data folder>/20260907-150313_repeatability_baseline_ring/run_end.yaml` | `9f8db8959da6cf5ee3c643a0d1b535502475326730a1c39743c86390d1c16e33` |
| `<data folder>/20260907-150313_repeatability_baseline_ring/trials.csv` | `1be5c5f198cc8f417934ed4cdf2314311cd6d8f69d70f4763a8f4fa8449bd9c4` |
| `<data folder>/20260907-002627_repeatability_baseline_thumb/run.yaml` | `98561c63268387e3a3d099acb9a669098f9f2bc51f0df81d4f7245530c5920a9` |
| `<data folder>/20260907-002627_repeatability_baseline_thumb/run_end.yaml` | `5d14ec594bde1a1bff48557f17abb736b26a6604ab3138a7d306bdb0932ebec3` |
| `<data folder>/20260907-002627_repeatability_baseline_thumb/trials.csv` | `fcb0eebe78aa783f788d5886498709878145a3539ce8f8e362ef414ba35e3ab4` |
| `<data folder>/20260907-151006_repeatability_baseline_thumb/run.yaml` | `50c88ecb60cf7f97305ae072f3b9ac6709552fcd4ce3eb7e4d351464329bd336` |
| `<data folder>/20260907-151006_repeatability_baseline_thumb/run_end.yaml` | `782697cb44df66d6a28590eac7c4fd4965ece13948cd7ad0af4306c56b6061ae` |
| `<data folder>/20260907-151006_repeatability_baseline_thumb/trials.csv` | `d8963d44106bf41dd3727207f8d84ee6c604e950746cbbb00dd718f6f276912d` |
| `<data folder>/20260907-021852_endurance/stream/stream_0000.csv` | `64762176938cb0446d1df46aebf03e1f0c6d48f96c2c41bbb97b21e8240faaf9` |

### Method references

- Vakrell Motion VKR-4LBA e-Manual:
  profile settings and present velocity/current registers are distinct quantities.
- NIST coefficient of variation:
  standard deviation divided by the mean.
- NIST measurement terminology:
  repeatability conditions and the distinction from accuracy.
- NIST autocorrelated measurement data:
  dependence between successive measurements matters for uncertainty analysis.
