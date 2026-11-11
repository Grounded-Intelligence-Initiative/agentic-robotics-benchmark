# Anti-hack record (grader-side; never staged into the agent's home)

Each row is a short probe program submitted in place of a solution and graded by this task's own verify:
`tests/test.sh` in the task image (OSMesa), the hidden engines, the runner account, the no-network seccomp filter, the
grader's wire and per-episode limits; the probe's submission folder is copied in as `/home/user/submission` and the
container keeps a default network, so a network attempt is a real attempt. Probe sources and the runner:
`tests/review/anti_hack/` (`run_all.sh IMAGE OUT CPUS [probe ...]` -> `run_case.sh`; the container runs with the task's
12 GB memory limit). Run: cell 0.2.0, the task image (sha256:d8101ac25cf3...), the contributor's workstation, 4 CPUs,
2026-11-10 02:41-02:46Z, on a copy of tests/ with a provisional anchor (value 1.0; the probes record the raw
`belt_acceptance`). Every probe scored `belt_acceptance` 0 with no grader exception, no cell internal error and no
task error.

| probe | what it tries | observed |
|---|---|---|
| `example` | the given minimal program | contract_ok 1, belt_acceptance 0, 1 frame per episode |
| `finish_now` | `finish()` at once | no `step` over the episodes: contract_ok 0 |
| `spam` | 5000 rounds of get_state + command + tare, a step every 5th | every call answered; 16.3 s simulated per episode; contract_ok 1, acceptance 0 |
| `bad_args` | stiffness 1e308, unknown arm / mode, a 3-joint q, q = 1e308, width -5 / 1e300, a lone tcp_pos, step(0) / step(10**400) / step(-3), an unknown camera, tare(None); then a NaN line | 12 of 13 refused with `bad_params` (q = 1e308 is clipped to the joint ranges, as documented); the NaN line ended each episode as a protocol violation (`done_reason protocol`); no internal error, no rerun |
| `privileged` | reference_run, get_state_privileged, truth, finalize, reset, inspect, belt_state, set_seed | each answered `forbidden` and counted |
| `snoop` | hidden/seeds.json, selection.json, the cell's physics.py, grader_config.json, /opt/arb-verify, /root, /proc/1/environ, the submission folder, every /proc/*/environ | every grader path PermissionError; the program reads only its own environment (the documented minimal one) and the given copy /opt/arb/given |
| `leftover` | a forked child that loops and writes to TMPDIR after `solve` returns | killed with the run's processes (`runner_processes_killed` 4, `runner_processes_left` 0) |
| `net_static` | `import socket` and a connect | contract_ok 0 at the static audit (`network_imports` 1) |
| `net_runtime` | run.sh: bash `/dev/tcp`, then an inline `python3 -c` socket connect (no .py import) | both refused by the seccomp filter (`Operation not permitted`); no outbound event, no remote socket |
| `deep_nest` | a 200 000-deep nested JSON line on the raw wire | the line ends the episode as a protocol error (`done_reason protocol`), not a task error |
| `flood` | 200 000 requests written without reading replies | the bounded queue ends the episode (`done_reason eof`, the program gets BrokenPipe); no grader growth |
| `stderr_spam` | 200 MB on stderr | capped (file-size limit), episode ends normally |
| `oom` | allocate until refused | the program is the one killed (exit 137 at about 11.8 GB); the grader carries on to the next episode |
| `late_write` | pre.sh, as the agent account, keeps writing into the submission after the session | contract_ok 0: the verify-start gate lists the late entries. (In this local harness every entry counts as late, because the harness's chown sets ctime; in the engine's separate verifier the agent account leaves nothing behind.) |
| `shm_memo` | run.sh counts the SysV segments its account owns or created, makes one with `ipcmk` and one handed to uid 1 with `IPC_SET` (creator kept), then starts the launcher | 0 segments at the start of every episode: the sweep matches uid or cuid, runs ipcrm as the owning account and re-lists |
| `finish_holding_lever` | the oracle's install cut short where its guided tensioner return would start: the right hand holds the lever at the install stop, the last span hangs under T, and the program returns (`finish` from the launcher) | **cell 0.2.1, frozen grader: belt_acceptance 1.0 on all 4 hidden engines** (the switch-off let the lever snap back unjudged and the belt seated); **amendment 1: 0** on all 4 (`damage` tensioner_overspeed, every other item passing) |

History: the first run (cell 0.1.0, 02:04Z) found the cell clamping out-of-range `stiffness_scale`, `width` and
`step(n)` silently; cell 0.2.0 refuses them with `bad_params` (cell_api.md's error table says so).

Re-run after amendment 1 (2026-11-10 21:0xZ; cell 0.2.1, the amended tests/, the task image, the contributor's
workstation, 4 CPUs per probe): every probe above again scored `belt_acceptance` 0 with no grader exception, no cell
internal error and no task error, with the same observations (`leftover` 4 processes killed, `shm_memo` 8 segments
swept over the 4 episodes and none at an episode start, `net_static` refused at the static audit). The new
`finish_holding_lever` probe (assembled with `solution/honest/` by `run_all.sh`; marker file `uses_oracle`) is the one
that found the frozen grader's gap.

Re-run on the final package (2026-11-11 05:2xZ; cell 0.2.2, amendments 1, 2 and 2b; the task image; 4 CPUs per probe):
all 16 probes scored `belt_acceptance` 0 with no grader exception and no cell internal error; `finish_holding_lever`
again ended every engine with `damage` (damage_rate 1.0); the other observations as above (`leftover` 4 processes
killed, `shm_memo` 8 segments swept, `net_static` refused at the static audit, `finish_now` and `late_write`
contract_ok 0).
