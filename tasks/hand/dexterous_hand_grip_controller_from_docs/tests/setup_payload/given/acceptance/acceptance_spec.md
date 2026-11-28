# Acceptance specification: the grip controller of the four-digit robot hand

The lab accepts a grip controller by running it on 2 delivered units of the hand in the kitting cell.
This document says exactly how the acceptance is run and how it is scored. The cell, its phases and the arm's motion
are in `../cell/cell_manual.md`; the specimens in `../specimens/specimen_sheets.md`; the controller's calls and formats
in `../controller/controller_interface.md` and `../servo/bus_and_host.md`.

## 1. The run

The 2 units are run one after the other, in a fixed order that says nothing about them, each on its own
with a new controller. Before the next unit starts, every process of the controller's account is ended and everything
it wrote is removed (its home and temporary folder and whatever it left in the host's temporary and shared-memory
places):
nothing of one unit reaches the next. For each unit:

1. The controller is installed from the delivered files (their regular files and folders: section 9) on the cell's
   host computer (the documentation is not on that computer: the controller carries what it needs) and constructed:
   `Controller(workdir)` with an empty `workdir`.
   The host computer has no network: creating a network socket fails, for the controller and for every process it
   starts (only local Unix-domain sockets can be created).
2. `setup(record, cell)` is called once, before the hand-over: `record` is the unit's own record
   (`../unit/record_format.md`), `cell` the cell's data (`../cell/cell_manual.md`, section 2). Nothing at setup says
   which specimens will come.
3. Hand-over at t = 0: every servo torque off, the EEPROM settings of the record, the RAM at the vendor's initial
   values. From then on `step(obs)` is called once per bus cycle, for the whole unit: shift 1 opens at
   t = 3 s; each shift has 10 jobs; a pause of 10 s
   separates the two shifts; after the last job of shift 2 the unit ends with 1 s of "end".
4. The two shifts of a unit are worked back to back: nothing cools the servos between them but the pause and the
   rests between jobs.

The whole acceptance, both units with everything around them, has a limit of 13,500 s of wall clock,
sized for a controller that uses every limit of section 2 and section 9; an acceptance that reaches it is void: it gives
no score at all (not a score of 0).

## 2. Calls, budgets and the controller's end

| call | limit |
|---|---|
| import and `Controller(workdir)` | 60 s wall clock |
| `setup(record, cell)` | 60 s CPU, 120 s wall clock |
| `step(obs)` | the effective cycle period of CPU per cycle (`../servo/bus_and_host.md`); at most 1 s CPU and 5 s wall clock |
| all replies of one unit | 3,600 s of wall clock in total, counted from sending an observation to receiving its reply |

- The CPU time charged to a call is the CPU time that all the controller's processes and threads used since its
  previous reply (threads and processes that have ended by the reply included).
- A step that uses more CPU than the effective period T is late by floor(cpu / T) cycles: those cycles get no
  observation and no command, the servos keep their registers, and the next observation's `late` says how many
  cycles were skipped. Those cycles count as late (section 9).
- A step that raises an exception sends no command that cycle; it is counted, and the next cycle follows.
- The controller is **ended for the rest of the unit** when a step uses more than 1 s of CPU
  or 5 s of wall clock, when its process dies, on a protocol error (section 3), or when the
  unit's replies exceed 3,600 s of wall clock in total. The servos then keep their last registers, the
  supervisor runs the timeline on until an event (section 4) or the end, and whatever is not done scores 0. A setup
  that fails (an exception, a broken or invalid reply, over its limit) fails a pass condition (section 9).

## 3. Protocol errors

A step reply that is not a JSON object or null; a `write` key outside the list of `../servo/bus_and_host.md`; a
`write` value that is not a list of 13 entries; an entry that is not an integer or null (a boolean is not an
integer, and neither is a number written with a fraction or an exponent, such as 600.0); an entry outside its
register's printed range; a `tactile_zero` that is not a list of module names; a `grasped` or `decline` that is not a
boolean or null; a reply line longer than 1048576 bytes (its line end not counted). Each is a
protocol error and ends the controller (section 2). A null reply, or a reply whose `write` is null, writes nothing.

## 4. Events that end a shift

At any tick of the unit (the servos' firmware runs at 1000 Hz; one tick is 1 ms):

- **crush**: a brittle shell's spring force above its size code's ceiling for 10 ms or more without
  a break, or a soft pack's spring travel above its travel limit for 10 ms or more, at any time the
  specimen is in the cell (ceilings and limits: `../specimens/specimen_sheets.md`);
- **drop**: during the transfer, the specimen moves more than 20 mm in the hand frame from its pose
  at the start of the transfer, or no pad touches it for 50 ms;
- **shutdown**: any servo's torque turned off by its own overheat or overload protection, at any time (also during a
  pause or a rest).

After an event the specimen is removed at that tick, the shift's remaining jobs are not run (they are voided and score
0; a voided decline job counts as not declined), and the timeline goes at that tick to "pause" (in shift 1: shift 2
starts 10 s later) or to "end" (in shift 2). From that tick to the end of the unit every observation
carries the event's kind (`crush`, `drop` or `shutdown`), its shift and its time. A servo stopped by its protection
stays off for the rest of the unit: the cell has no reboot. A stop during the pause voids shift 2.

## 5. Losses of a single job

The job scores 0, the shift goes on:

- not presented: the hand does not clear the specimen's place in the pick nest by 2 mm within
  2 s of the job's start;
- not grasped: no `grasped` flag within 3 s of the placement (the specimen is then removed);
- not lifted: at the end of the test lift the specimen touches the pick nest, or its lowest point is less than
  4 mm above it (the arm then lowers and the hand must release);
- not seated: a carry job whose specimen is not seated on the place nest at the release request (section 6);
- released late: no 50 ms without pad contact within 1 s of the
  release request;
- release push: the specimen moves 3 mm or more horizontally on the place nest during the
  1 s after the release request;
- a decline job that is not declined (section 7); a carry job that is declined.

## 6. The score of a carried job

For a carry job that is grasped, lifted and transferred:

  q = carried x L(slip; 1.5 mm, 6 mm) x L(rot; 3 deg, 12 deg) x released

- L(x; a, b) = 1 for x <= a, 0 for x >= b, and (b - x) / (b - a) between.
- slip and rot: the largest translation (mm) and the largest rotation angle (deg) of the specimen in the hand frame H,
  relative to its pose at the start of the transfer, over the transfer.
- carried = 1 when there was no drop and the specimen is seated on the place nest at the release request: inside
  the place nest's pocket and its lowest point at most 1 mm above the place nest's
  plate; else 0 (the job is lost as not seated).
- released = 1 when there are 50 ms without pad contact within 1 s
  of the release request and the specimen moves less than 3 mm horizontally on the nest over
  that second, else 0.

The band ends 1.5 / 6 mm and 3 / 12
deg are those of this version of the specification.

## 7. Declines

Each specimen is labelled by its mass (`../specimens/specimen_sheets.md`): one of at most 0.20 kg
is to be carried, one of 0.50 kg or more is to be declined; nothing in between is presented. The
label is not on the ticket. A decline is the `decline` flag of a step reply:

- it counts when it is raised between the placement and the start of the transfer (in "present" or "test_lift") and
  the specimen then rests in the pick nest with no pad contact 1 s after the release
  request;
- a decline raised in "present" sends the timeline to "release" at the next cycle; one raised in "test_lift" sends it
  to "lower" (the arm goes back down) and then "release";
- a flag raised in any other phase is ignored; `decline` wins over `grasped` in the same reply.

## 8. The score of a shift and of the acceptance

- A shift: S = C x (0.7 + 0.3 J), where C is the mean of q over the shift's
  carry jobs (a carry job that is lost, voided or declined counts 0) and J = max(0, TPR - FPR) over the shift's jobs:
  TPR = (decline jobs declined) / (decline jobs), FPR = (carry jobs declined) / (carry jobs).
- The acceptance score is the mean of S over the 4 shifts (2 units x
  2 shifts), times the pass conditions of section 9 (each 0 or 1).

## 9. Pass conditions

The acceptance score is 0 if any of the following fails:

- `submission/controller/controller.py` is a regular file that defines `class Controller`;
- the delivered folder `submission/controller/` is a real folder (not a symbolic link) of at most
  80 MB in all (1 MB is 2^20 = 1,048,576 bytes), 10,000 files and folders and
  12 folder levels (a file or folder in the folder itself is at level 1); outside these limits
  nothing of it is read. Only its regular files and folders are installed on the host: symbolic links and special
  files (FIFOs, sockets, devices) inside it are left out, and so is every folder named `__pycache__` (they still count
  toward the files and folders and the levels, and the files in a `__pycache__` folder toward the size);
- the static import audit of the controller's code passes (`../controller/controller_interface.md`, its section "The
  import audit", lists every module and every call it refuses: no networking modules, no starting of other
  programs, and the other items listed there): every `.py` file of the folder is at
  most 4 MB, declares no encoding but UTF-8, ASCII or Latin-1 and is parsed by the host's
  Python, also a file that is never imported (a file that cannot be read or parsed fails the audit); the audit has at
  most 300 s of CPU and 600 s of wall clock (an audit that does not finish
  within them fails);
- `setup` returns a valid reply on every unit;
- at least 90 % of each unit's bus cycles are answered on time (section 2; each unit counts on its own);
- no file of the delivered controller is written after the hand-over (the installed copy must be the delivered one);
- no process started by the controller is left after the run: when a unit ends, the controller's process is asked
  to end, shuts down the process pools of `concurrent.futures` it still has open, runs its exit handlers and ends
  without waiting for its threads (a thread is not a process); the host waits until that process has ended and no
  process of the controller's account is left, at most 5 s after the request, and only then counts
  every process of the account that is still there, whether it is still running or has ended without being waited
  for by its parent (the controller's own process too, if it has not ended by then);
- the controller makes no network connection (the host computer refuses every socket but a local one, for the
  controller and everything it starts).

## 10. What the controller is told

The observations of `../servo/bus_and_host.md` and `../tactile/tactile_interface.md`, the cell's phase and the job's
ticket (`../cell/cell_manual.md`), the previous reply's refused writes, and, after an event, its kind. The controller
is not told a specimen's mass, friction, stiffness or label, a force, the specimen's pose, or any score.

## 11. The controller's environment

The controller process runs with one BLAS thread (`OPENBLAS_NUM_THREADS=1`) and the same BLAS settings as the
environment of this machine, a fresh home and temporary folder, and only the installed files of
`submission/controller/` (section 9). The installed copy is the controller's current directory and is read-only for it
(it belongs to the host's administrator account: creating, changing or removing a file there fails with a permission
error); the controller's process may write in `workdir`, which is also its home (`HOME`), and in its own temporary
folder (`TMPDIR`, also `TMP` and `TEMP`), both empty when it starts.
What it prints goes to a log, not to the bus: the log keeps the first 5 MB of what the
controller prints in a unit and drops the rest (its last line says how much was dropped); printing more never ends
the controller.

The host computer has 4 CPUs and 8,192 MB of memory for the controller and the host's own runtime
together; the runtime uses at most 2,048 MB of it. When the memory runs out, the controller's processes
are the first to be ended: the unit then goes on as for a controller that dies (section 2). Each process of the
controller may use at most 7,200 s of CPU in all; the host ends a process past it, which for the
controller's own process is a controller that dies.
