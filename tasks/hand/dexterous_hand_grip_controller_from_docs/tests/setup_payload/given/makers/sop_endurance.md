# SOP — 12 h endurance run (paper section IV-C)

Target: **≥ 12 h at 42.9 cycles/min ≈ 30,857 cycles**. The rate deliberately
matches the arXiv v1 run (5,143 cycles in 2 h) so the two are directly
comparable.

---

## 1. The object

The object is the biggest confound in this experiment: **if it softens over
~31,000 compressions, the falling grasp force looks exactly like a degrading
hand.** Everything below follows from that.

### If you are using a paper/cardboard box

Workable, and fine for the rehearsal, but be clear-eyed about the trade:
cardboard **plastically deforms and fatigues**. Panels crease, corners round
over, and the stiffness at cycle 30,000 is not the stiffness at cycle 1. It is
also hygroscopic, so an overnight humidity swing changes it further.

Consequences, and what to do about them:

- **Grasp-force drift becomes partly a property of the box.** Do not report it
  as a hand result unless the pre/post check below says the box held up.
- **Everything else is unaffected**: actuator temperature, open-pose tracking
  error, comm integrity, hardware errors, and the before/after repeatability
  battery. Those carry the reliability claim on their own, so the run is still
  worth doing.
- **Choose the stiffest box you have** — a small, full, taped-shut one deforms
  far less than an empty one. Grasp across a corner or an edge rather than a
  flat panel; panel buckling is nonlinear and hysteretic.
- **Expect to say so in the paper.** A sentence like *"the compliant object was
  a closed cardboard box; its indentation stiffness changed by X% over the run,
  so normalised grasp force is reported as an upper bound on hand-side drift"*
  is honest and costs nothing. Silently reporting box fatigue as hand data does
  not survive review.

If a longer-lived object is available later, a platinum-cure silicone sleeve
(Shore A 40-50, ~7 mm wall) over an aluminium core is the right answer:
compression set under 5%, no plasticiser, stable across 25-50 °C. Printed TPU is
*not* — its cyclic softening is largest in the first few thousand cycles,
precisely the window that would masquerade as hand drift.

### Measure the object, do not assert it

Before and after the run, with equipment you already have:

1. Scribe three sites on the box, one under each of the index, middle and thumb
   contacts, indexed so you can find them again.
2. Rest a plunger indicator with a ~Ø10 mm hemispherical tip on a mark.
3. Zero at a light preload, add a known weight (500 g works), record indentation
   at 5 s and again at 30 s. The second reading catches creep.
4. Five repeats per site.

> **Decide the rule before you look at the result:** if any site changes by more
> than 5%, the grasp-force drift is reported with an explicit caveat naming the
> object. Under 5%, put the bound in the caption.

Also record: object mass before/after (±0.1 g), macro photos of the three
contact sites, and note whether the box was replaced or re-oriented at any point
(it must not be).

---

## 2. Fixturing

The requirement is simply that **the object cannot move relative to the hand**,
and that you can prove it afterwards.

- Clamp or bolt the object to the same surface the hand is mounted on. A box can
  be taped and clamped to a block; what matters is that it does not creep.
- Mark the object's footprint on the bench so any shift is visible.
- Keep the camera fixed and get both the hand mount and the object in frame. Two
  printed fiducials (ArUco or just high-contrast crosses) give a free post-hoc
  check that neither moved. If you have no camera, at least photograph the
  marked footprint at t = 0 and t = 12 h.

---

## 3. Grasp pose

Teach it by hand. The hand backdrives at ~0.02 N·m, so posing it physically is
faster and more accurate than nudging joints through a keyboard:

```bash
python -m experiments.endurance_run --teach-grasp \
    --grasp-pose-file <config folder>/grasp_pose.yaml \
    --max-cycles 215 --operator <initials> --object "<what is in the fixture>"
```

1. Torque releases — the hand goes limp. A live table shows per-digit current,
   tactile force and taxel count while you pose it.
2. Place the fingers on the object until each pad **just touches**. Do not press
   in; the preload does that.
3. **Enter** captures the joint positions. Torque comes back on and holds them,
   and the table is shown again so you can confirm.
4. **Enter** again starts the run.

The captured pose is saved to `--grasp-pose-file` and recorded in the manifest.
Reuse the same file for the real run and any resume, so the grasp is identical
across the campaign. `--grasp-depth` remains as a fallback that scales a coded
pose, which cannot know what is in the fixture.

The jogged pose is where the fingers **touch**. Each cycle then presses
`--grasp-preload-rad` further in (default −0.10 rad per digit): stopping at first
touch leaves the grasp position-limited, where force swings wildly with
sub-millimetre object drift, whereas pressing in drives the contact motors to the
current limit and makes the force stable.

Two further knobs, both recorded in the manifest:

- `--hold-s` (default 0.40) caps how long the hand presses each cycle, measured
  from the close command to the open command. Sustained saturation is the
  thermal limit on this run. The achieved value is logged per cycle as `hold_s`
  and checked by the rehearsal script.
- `--current-limit-ma` (default 600, from the hand config) sets how compliant
  the grasp is in operating mode 5. Lower yields sooner. It is written and read
  back from all 13 motors at start-up.

Whichever route, the pose must satisfy all four:

1. **All four pads in contact, near pad centre.** Edge contact makes ΣFz
   hypersensitive to sub-millimetre pose drift — you would be measuring geometry
   and calling it force. Check the tactile centroid lands within ~3 mm of the pad
   centroid (`centroid_mm_*` in `cycles.csv`).
2. **Contact motors reach the 600 mA goal-current limit while closing.** This is
   the important one. In `operating_mode=5` a saturated grasp is *force*-limited,
   so grasp force is insensitive to sub-millimetre object drift. If the motors
   never saturate the grasp is *position*-limited and a 0.2 mm object relaxation
   changes force enormously. Verify in the rehearsal via `cur_hold_mean_ma`.
3. **Hold briefly** — sustained saturation is the thermal problem.
4. **Leave the abduction motors (6, 9, 12) fixed and uncycled.** They then serve
   as unloaded thermal controls in the same environment: any drift on them is
   environmental, not mechanical wear.

---

## 4. Pre-run checklist — all must pass

1. Calibration SHA-256 matches the pinned baseline.
2. `hand.ping()` returns all 13; `verify_models()` empty.
3. All hardware error status bytes zero.
4. Torque-enable readback 1 on all 13; goal current 600; **P/I/D = 800/0/200**.
   Note the D: the hand's configured default is 500, at which a free joint
   self-oscillates (90 mrad peak-to-peak measured at the PIP, ~4.8 mm at the
   fingertip, 60 mA of chatter). The open-pose tracking error is taken with the
   finger unloaded, which is exactly that condition, so D=500 would corrupt the
   primary wear metric across all 31k cycles. The runner sets 200 itself and
   reads it back from all 13 motors, aborting on a mismatch.
5. **USB latency timer = 1 ms** (`cat /sys/bus/usb-serial/devices/<tty>/latency_timer`).
   At the 16 ms default the control rate silently caps near 62.5 Hz. Fix with
   `./setup_servo_latency.sh`.
6. Bus voltage within ±0.2 V of nominal, measured **at the hand, under load**.
7. Tactile modules: all four fingers enumerate; **one** `recalibrate()` done with nothing
   touching the pads; 60 s open-hand check shows per-taxel Fz in [0, 0.2] N and
   **zero wrapped samples**.
8. Baseline integrity recorded: `python examples/smoke_test.py`, plus the
   calibration hash appended to `<data folder>/calibration_log.txt`. It
   must match the hash taken at the repeatability baseline.
9. Object compliance check done; object and fingertips IPA-cleaned and photographed.
10. Motors in a *similar* thermal state to the repeatability baseline, and the
    temperatures recorded either way. Matching the conditions matters more than
    hitting any particular number, and the register only resolves 1 °C.
11. Camera: position taped and marked, **manual** focus/exposure/white balance,
    both fiducials in frame, ≥ 12 h of storage and mains power.
12. ≥ 20 GB free; logger verified for 60 s.
13. Host suspend disabled (`systemd-inhibit` or equivalent).
14. **300-cycle rehearsal completed**, and its `cycles.csv` checked for all of:
    `cur_hold_mean_ma` at the 600 mA limit (otherwise re-tune `--grasp-depth`);
    `closed_settled` and `open_settled` both 1 on every cycle; `n_wrapped_*`
    exactly 0; `n_active_*` around 8-15 per finger; `comm_failures` and
    `comm_stale` 0; `period_s` close to 1.40. Then reset and start the real run
    without touching anything.

> **The settle flags are not a formality.** The gate compares measured velocity
> against a threshold, and a stationary joint still reports 1-2 encoder counts
> (0.024-0.048 rad/s). The gate was previously 0.05 rad/s across all 13 motors,
> which essentially never closed: every move ran to its timeout and every sample
> was taken at an arbitrary moment rather than at a settled pose. It is now 0.10.
> If the rehearsal shows `settled=0`, stop and raise it further -- the data is
> worthless otherwise, and it will look perfectly normal.

---

## 5. Running it

```bash
# always dry-run first: this exercises the entire 12 h state machine in ~2 min
python -m experiments.endurance_run --dry-run --duration-h 12

# the real run, under tmux
python -m experiments.endurance_run --duration-h 12 \
    --operator <initials> \
    --object "silicone 45A sleeve, 7 mm wall, over 36 mm alu core" \
    --grasp-depth 1.0
```

Before either, the 5-minute rehearsal:

```bash
python -m experiments.endurance_run --max-cycles 215 \
    --operator <initials> --object "<what is in the fixture>"
```

Monitor from another terminal with `watch -n5 cat <run_dir>/progress.json`.
Do not build a dashboard.

**Recovering from a crash:**

```bash
python -m experiments.endurance_run --resume <run_dir> --duration-h 12
```

Resume repairs a torn final row, continues cycle numbering without gaps or
duplicates, and appends a `RESUME` record to `events.jsonl`. Resume happens at
cycle boundaries only.

---

## 6. Abort criteria (automatic)

| Condition | Action |
|---|---|
| Any motor ≥ 55 °C | warn |
| Any motor ≥ 62 °C | pause and cool to 48 °C, then resume |
| Any motor ≥ 72 °C | **abort** |
| Hardware error: overheat / encoder / shock / overload | **abort** |
| Any motor's torque disabled itself | **abort** |
| Bus voltage outside 10.5–14.5 V | **abort** |
| Comm failure rate ≥ 25 % | **abort** |
| Tracking error ≥ 0.6 rad for 10 consecutive cycles | **abort** (object escaped or a part failed) |
| Free disk < 2 GB | **abort** |

The servo's own shutdown is at 80 °C; aborting at 72 °C keeps a margin and is
defensible as "we stopped well before the actuator's own protection." v1 settled
near 49 °C, so 62 °C should never be reached — reaching it *is* a finding.

Human checks: observe from ≥ 0.5 m every 2 h, take one photo from the marked
position, write one line in the logbook. **Do not touch anything.**

---

## 7. "Zero interventions" — define it before you claim it

> **Intervention** = touching the hand, object, wiring, connectors or power;
> restarting or attaching to the logger; re-homing; re-calibrating the tactile
> sensors; cycling torque; re-seating the object; adjusting the camera; changing
> any commanded pose, gain, current limit or profile parameter; or any physical
> contact with the baseplate.
>
> **Not an intervention:** observing from a distance, reading the terminal,
> taking the scheduled photo, reading a separate ambient logger.

Report "0 interventions" only if the ledger is literally empty. If an abort
occurs, **report the run as aborted at time T with the cause** — do not silently
restart and present the second attempt as the first.

> **Pre-registered restart rule:** at most one restart is permitted after an
> abort caused by an identified *external* fault (power, host, camera). An abort
> caused by the hand itself terminates the experiment and is reported.

---

## 8. Tactile handling

- `recalibrate()` **exactly once**, before the run, nothing touching the pads.
  **Never again during or after it** — a mid-run re-zero destroys the drift
  signal the experiment is trying to measure.
- The runner records an **open-hand Fz baseline every cycle** and subtracts it.
  That both compensates sensor drift and yields a free "tactile drift over 12 h"
  number for the tactile-integration section. In dry runs this holds reported
  grasp force flat at 4.4 N while 31 N of simulated drift accumulates.
- `--fz-wrap-threshold-n 20` guards the unsigned-Fz wrap: one LSB of negative
  baseline drift transmits `0xFF`, which decodes to **+25.5 N**. Set the
  threshold above the largest force you would call real and below full scale.
  Leave it off if genuine forces approach 25 N.
- `n_active_*` and `centroid_mm_*` are the confound controls. If grasp force
  drifts while both stay flat, the drift is force. If the centroid migrates by
  more than ~2 mm the contact patch moved and the drift is geometric.

---

## 9. Confounds a reviewer will raise

| # | Confound | Handled by |
|---|---|---|
| C1 | Object softens | Silicone + measured pre/post indentation, 5 % rule |
| C2 | Tactile sensor drifts | Per-cycle open-hand baseline; normalised reporting |
| C3 | Contact patch migrates | `n_active_*`, `centroid_mm_*` |
| C4 | Ambient temperature / humidity drift | Log ambient separately; plot on the same axis. If the printed parts are nylon, moisture changes stiffness — name the material |
| C5 | Self-heating changes friction | Temperature plotted with everything; motors 6/9/12 as unloaded controls |
| C6 | Supply droop | `volt_min_v` logged; ±0.5 V abort band |
| C7 | Silent comm dropouts look like perfect tracking | `comm_stale` / `comm_fail_rate` columns |
| C8 | Silent motor shutdown | Hardware error status and torque readback polled each cycle |
| C9 | Object or hand physically shifts | Two fiducials in the timelapse |
| C10 | Unsigned Fz wrap | `n_wrapped_*` reported |
| C11 | Grasp is position- not force-limited | Pose tuned until motors saturate |
| C12 | Cycle count not comparable to v1 | Rate matched to 42.9/min deliberately |

---

## 10. Video

- **Timelapse** 1 frame / 10 s → 4,320 frames → 3 min at 24 fps. Doubles as an
  independent drift witness.
- **Slow motion** 240 fps, 10 consecutive cycles at t = 0, 6 and 12 h, same
  framing. Those three clips side by side are the strongest supplementary asset.
- Keep a monitor showing the runner status in shot so the video is
  self-authenticating.
- Fixed macro shots of each fingertip pad at t = 0 and t = 12 h, same lighting.
