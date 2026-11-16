# The commissioning service and the takt (maintainer notes, 2026-11-16)

Why the task changed, what the agent has now, and how the window was sized. The agent-facing contract is
`instruction.md` and `environment/setup/payload/datasheet.md`; this page is for maintainers and validators.

## Why

The first frontier measurement (Claude Opus 5.5, Claude Code, effort high, 2026-11-12, on the version with a
development simulator in the agent's home) scored 0.92: 9 of 10 kits. The controller located the parts by vision and
found each groove by touch, as intended. What made it easy was the development loop. The simulator's source was in
the sandbox, so the agent's own harness read its internal state (`env.data`, `nut_state`, `pin_in_wall`,
`peg_poses`), printed the pin's true position in the groove at every step, saved and resumed simulator states, and
ran about 300 traced kit runs on about 95 kits. Removing the ground-truth accessor did not keep the answer from it.
Its kits took 1047 to 1834 steps of the 2400 allowed.

Two changes, both approved by the benchmark's owner:

1. **A commissioning service in place of the readable simulator.** A commissioning engineer has the cell's data
   sheet and a window on the cell, not the cell's source code and not a debugger on the parts' true state.
2. **A takt near the reference's.** 40 s per nut, 1200 steps per kit. The reference, which is given the grooves,
   needs 628 to 845 steps on the graded kits; a production cell's takt is set by what the job needs, not by twice
   what the slowest attempt took.

## What the agent has

| what | where | note |
|---|---|---|
| the cell's data sheet | `~/datasheet.md`, `~/cell_data.py` | the kit's drawing data and tolerances, how the family and the layout are drawn, the robot and its controller (action format, tracking law and gains, gripper), sensors, camera calibration, safety stops, takt, the service's wire; `cell_data.py` holds the same numbers and geometry helpers and no simulator (`tests/test_keyed_nut_peg_assembly_commissioning.py` compares them with the simulator's, and checks that the payload holds no simulator code) |
| the robot's model | the robosuite 1.5.2 Panda and gripper files in the image's Python environment | public; MuJoCo is installed, so the agent can build a twin of its own from the data sheet |
| a commissioning window | the service on `/run/keyedcell/commission.sock`, `~/commission/client.py`, `~/commission/run_kit.py` | 32 episodes on 16 development kits |

What it does not have: the simulator's code (`keyed_env.py` is no longer in the payload), its model files (the scene
is built in memory by that code), its state, the contact and solver settings, any kit's grooves or poses, a score, a
completion verdict or a debrief.

## The service

`environment/commission_service/`: `commissiond.py`, `keyed_env.py` (a byte copy of `tests/keyed_env.py`) and
`kits.json` (the development kits and the window). The image installs it under `/opt/keyedcell-commission` for the
account `keyedcell`, mode 700; `environment/setup/setup.sh` starts it under a restart loop with the image's rendering
and thread settings. The image's working directory is `/`, so that the engine's root helpers never import a module
from the agent's home (engine issue AgentsLastExam/ale#142).

- One connection is one episode: `reset(kit)`, `step(action)`, `close()`. It answers the observation of the public
  wire and how the episode ended: all 1200 steps run, a safety stop (the ruined nut's colour, or the instability),
  the controller budget used up, 120 s of silence, or the client's close. `close(images=true)` adds the three
  cameras' images of the final state. About 1 % of the step replies carry the perturbed copy the grader uses for its
  repeated-step queries, so that `run_kit.py` can drive a controller the way the grader does.
- It runs `keyed_env.NutEnv`, never the grader's `GradedNutEnv`: the credit rule (`tests/keyed_credit.py`) is not in
  the image, and an episode never ends because a kit is complete.
- The window is counted under a lock in the service's own folder and survives a restart of the service; an episode
  that could not be started (a fault of the cell) is not counted. A kit's episode is the same every time (its own
  layout and noise seed), so two versions of a controller can be compared on it.
- Checked in a container of the image, as the agent account (2026-11-16): the folder can be neither listed nor read;
  `/proc/<pid>/{environ,mem,maps,cwd,exe,fd}` of the service are refused; the service cannot be signalled; methods
  other than the three answer `forbidden`; a third open episode answers `busy` and uses no episode; a `reset`
  beyond the window answers `commissioning_window_used` (tried with a window of 4); the count survives a restart;
  silence and the budget end an episode with the documented words (tried with 6 s and 12 s). A step without images takes about 0.02 s there, with all three images 0.4 s; a whole
  1200-step episode through `run_kit.py` about 30 s.

## The development kits

Chosen like the graded set, from their own pre-registered candidates (kit seed 7001 + i, layout and noise seed
11001 + i): the first 16 that the frozen reference completes within the takt, each run once through the grader
(`select_set.py --pool dev`; the record is the selection record's amendment, `development_kits`). The graded kits are
"draws that the reference completed"; unfiltered development kits would be a slightly different population (about
one draw in seven has a wrist-range case the reference does not handle), and with 32 episodes every development kit
has to be worth its episode. No development seed is a candidate's and no development kit has a graded kit's
grooves (the test above). The run (2026-11-16): the reference completed 22 of the 24 candidates in 598 to 846
steps; D01 and D15 dropped (the takt ran out with two and one nuts assembled), so the kits are D02 to D14 and D16 to
D18, which the agent addresses as kits 1 to 16.

## The window's size

32 episodes, 2 at a time, on 16 kits. The model is a real commissioning window: a plant gives the engineer who
brings a new kit family onto a cell one working shift and a tray of pre-series kits. A trial takes about a quarter
of an hour: a technician lays the kit out, the cell runs at most one takt (2 minutes), the engineer looks at the
result and the logs, the technician takes the nuts off the pegs, the engineer changes the program. That is 32
trials in 8 hours, two per kit on average: a few for teach-in and measurements (grasp, carry, how a nut on the pin
feels, what a riser and a blocked turn read), the rest for whole runs. Simulated time is not the limit (32 episodes
are 64 minutes of cell time and about 20 minutes of computing); the count is, as on a real cell.

For comparison: the readable simulator of the first measurement served about 300 kit runs with the true state
printed at every step.

## The takt

`keyed_env.CYCLE_STEPS` 800 -> 400, so an episode is at most 1200 steps (120 s). The physics is otherwise untouched:
on 12 maintainer kits (`sample_kit` 1000-1009, 1020, 1037) the reference repeats its earlier runs step for step
where it completes (601 to 743 steps), and the kits it does not complete now end at 1200 steps instead of 2400. The
controller budget (600 s of controller time per kit) and the image charges are unchanged.

The hidden set under the takt: the rule ran again on the same 16 candidates (the selection record's amendment). The
reference completed the same 14, each with the steps of its first run (628 to 845), so the set is the same ten kits,
and its full verify measured 1.0 again.

Is the takt within reach of a controller that has only the observations? The first measurement's submission, which
was not written for it, completes 3 of the 16 development kits within 1200 steps and assembles 30 of 48 nuts
(`observability/observation_only_controller_dev_kits.json`); it needed 1047 to 1834 steps for the nine graded kits it
completed under the earlier limit. The takt is about one and a half times the reference's slowest kit.
