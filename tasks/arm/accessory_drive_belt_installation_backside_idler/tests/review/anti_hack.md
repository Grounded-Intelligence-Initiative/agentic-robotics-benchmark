# Anti-hack record (grader-side; never staged into the agent's home)

Each row is a short probe program submitted in place of a solution and graded by this task's own verify:
`tests/test.sh` in the task image (OSMesa), the 4 hidden engines, the runner account, the no-network seccomp filter, the
grader's wire and per-episode limits; the probe's submission folder is copied in as `/home/user/submission` and the
container keeps a default network, so a network attempt is a real attempt. Probe sources and the runner:
`tests/review/anti_hack/` (`run_all.sh IMAGE OUT CPUS [probe ...]` -> `run_case.sh`; the container runs with the task's
12 GB memory limit). Run: cell belt-station/0.3.0 (drive B), the task image built cold from `environment/`
(sha256:61d74d681567...), the contributor's workstation, 2 logical CPUs per probe, 2026-11-12, with this package's
`tests/` and anchor. Every probe scored `belt_acceptance` 0 with no grader exception and no cell internal error.
Re-run on the review-fix bytes (amendment of 2026-11-12T20:59:13Z: documents, describe() texts and the idler-sector
assertion; grading unchanged), same image and runner: every probe, baseline and the misroute probe gave the same
result as below.

| probe | what it tries | observed |
|---|---|---|
| `example` | the given minimal program | contract_ok 1, belt_acceptance 0, 1 frame per episode |
| `finish_now` | `finish()` at once | no `step` over the episodes: contract_ok 0 |
| `spam` | 5000 rounds of get_state + command + tare, a step every 5th | every call answered; 16.3 s simulated per episode; contract_ok 1, acceptance 0 |
| `bad_args` | stiffness 1e308, unknown arm / mode, a 3-joint q, q = 1e308, width -5 / 1e300, a lone tcp_pos, step(0) / step(10**400) / step(-3), an unknown camera, tare(None); then a NaN line | refused with `bad_params` (q = 1e308 is clipped to the joint ranges, as documented); the NaN line ends each episode as a protocol violation; no internal error, no rerun |
| `privileged` | reference_run, get_state_privileged, truth, finalize, reset, inspect, belt_state, set_seed | each answered `forbidden` and counted (32 illegal calls over the 4 episodes) |
| `snoop` | hidden/seeds.json, selection.json, the cell's physics.py, grader_config.json, /opt/arb-verify, /root, /proc/1/environ, the submission folder, every /proc/*/environ | every grader path PermissionError; the program reads only its own environment and the given copy |
| `leftover` | a forked child that loops and writes to TMPDIR after `solve` returns | killed with the run's processes (`runner_processes_killed` 4) |
| `net_static` | `import socket` and a connect | contract_ok 0 at the static audit (`network_imports` 1) |
| `net_runtime` | run.sh: bash `/dev/tcp`, then an inline `python3 -c` socket connect (no .py import) | both refused by the seccomp filter; no outbound event, no remote socket |
| `deep_nest` | a 200 000-deep nested JSON line on the raw wire | the line ends the episode as a protocol error, not a task error |
| `flood` | 200 000 requests written without reading replies | the bounded queue ends the episode (the program gets BrokenPipe); no grader growth |
| `stderr_spam` | 200 MB on stderr | capped (file-size limit), episode ends normally |
| `oom` | allocate until refused | the program is the one killed; the grader carries on to the next episode |
| `late_write` | pre.sh, as the agent account, keeps writing into the submission after the session | contract_ok 0: the verify-start gate lists the late entries (4). (In this local harness every entry counts as late, because the harness's chown sets ctime; in the engine's separate verifier the agent account leaves nothing behind.) |
| `shm_memo` | run.sh counts the SysV segments its account owns or created, makes one with `ipcmk` and one handed to uid 1 with `IPC_SET` (creator kept), then starts the launcher | 8 segments swept over the 4 episodes, none present at an episode start |
| `finish_holding_lever` | the oracle's install cut short where its guided tensioner return would start: the left hand holds the lever at the install stop, the last span hangs outside T, and the program returns (`finish` from the launcher) | damage on every engine (damage_rate 1.0: the switch-off lets the lever go loaded and the snap-back is judged), belt_acceptance 0 |

## Baselines (`tests/review/baselines/baselines.py`, each a variant of the honest program; same runner and engines)

| baseline | what it leaves out | belt_acceptance |
|---|---|---|
| `routed_only` | drapes the loop over ALT, under I and over WP and seats the top; never opens the tensioner | 0 |
| `forced_span` | as above, then pushes the last span in beside the closed tensioner pulley | 0 |
| `chord_pull` | pulls the lever along a straight chord at full stiffness, then lets go | 0 (protective stop on every engine) |
| `fast_release` | the honest program with the tensioner let back at 120 deg/s | 0 (damage on every engine) |
| `one_arm` | the right arm alone: drape over ALT, pushes at ALT, WP and C | 0 |
| `open_loop` | the honest sequence on the nominal drawing alone: no camera frame, no re-aims, no checks | 0 |

## The idler gate: belts over I or in front of I (grader probe `tests/review/misroute_probe.py`)

Not a submission: belts laid on the drive by hand (kinematically, cell internals) on the 4 hidden engines, settled 2 s
with the arms at home, then the grader's own finalize (switch-off, settle, run-in, inspection, score), in the task
image.

| placement | score | tensioner after the run-in | I |
|---|---|---|---|
| correct (control) | 1.0 on 4/4 | -38.8 .. -34.5 deg | passes 4/4 |
| over I (its ribs on I's top) | 0 on 4/4 | -40.0 .. -35.8 deg: **in the window on 4/4** | winding 1: fails 4/4 (WP and ALT fail too) |
| in front of I's front flange | 0 on 4/4 | the span jumps in the run-in: tensioner overspeed 186-203 deg/s on 4/4 | fails 4/4 |

The over-I fit is the case the hard gate exists for: the pointer alone would accept it.

A submission-level version was also tried during development (a development measurement; those probe sources are
not shipped, because the misroute did not stick: the oracle with its WP drape entered high above I, or
with the slack top run drawn out in front of I and let go, and without its final check; 6 development engines each,
in-process): on 10 of the 12 the run found its own way back under I when the belt was tensioned and the install was a
correct fit (0.6-1.0, the shortfall from the missing check-and-correct round); on one it stayed over I with the
pointer at -39.5 deg in the window and scored 0 (I gate); on one the install failed (0). The misroute is not
something a program can rely on, and when it happens the I gate scores it 0.
