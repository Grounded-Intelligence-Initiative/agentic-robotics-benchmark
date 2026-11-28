# Write the grip controller for the lab's four-digit robot hand

You are the controls engineer of a small lab. The lab fits a four-digit robot hand ("the hand": three fingers and a thumb; 13 servo
actuators on one bus; in each finger a four-bar that couples the last joint to the pip axis; a tactile module on the
last link of each digit) to a robot arm in a kitting cell. For each job the cell's supervisor places an instrumented
specimen in the pick nest. The hand closes the taught three-pad grasp (the thumb against the index and middle
fingers), the arm makes a test lift, carries the specimen to the place nest along a fixed profile and sets it down,
and the hand lets go. Some specimens are too heavy for the hand: those are to be declined, not carried.

Your job is the hand's grip controller: the program that runs on the cell's host computer, reads the servos'
registers and the tactile frames every cycle and writes the servos' registers, so that every specimen that is to be
carried is held without crushing it, without dropping it and without letting it move in the hand, every specimen that
is to be declined is declined, and every servo stays inside its protection limits through two shifts worked back to
back.

The hand is not available to you, and there is no practice hand. You have only what the hand's makers, the vendors of
its parts and the lab hand over, in `/home/user/given/` (start with `given/README.md`):

- `hand/`: the kinematics sheet (joints, the four-bar's law, the pads' frames, the links' mass properties), the links'
  meshes and the taxel maps of the tactile modules;
- `servo/`: the servo vendor's manual page, the servo's performance curve (transcribed from the vendor's graph) and
  the cell's bus and host runtime;
- `tactile/`: the tactile interface note and the module vendor's parameter table;
- `makers/`: the makers' hand-off notes, their test procedures and their results, as they wrote them;
- `unit/`: the format of a unit's own record, the record of another unit, and the range sheet (how a unit may differ
  from the documents);
- `cell/`: the cell manual, the taught grasp, the arm's transfer profiles, the shift sheet format and an example shift;
- `specimens/`: the specimen sheets (classes, size codes, mass ranges, squeeze ceilings, finishes, the label rule);
- `acceptance/`: the lab's acceptance specification: exactly how the acceptance is run and scored;
- `controller/`: the controller interface, an example controller, and `check_controller.py`, a self-test of the
  interface against a toy plant.

No simulation model of the hand comes with it, and no unit's true parameters are known: every unit differs from the
documents within the ranges of `unit/range_sheet.md`, and the tactile modules are uncalibrated. The acceptance runs on
hand units you have never seen. Each unit's own record and the cell's data are given to your controller at setup;
each job's ticket (class, size code, finish, transfer profile) arrives with that job, never earlier.

## What to deliver

`/home/user/submission/controller/controller.py` with `class Controller`: `__init__(workdir)`, `setup(record, cell)`
returning the cycle period and the register blocks to read, and `step(obs)` returning the register writes, an optional
tactile re-zero and the two flags `grasped` and `decline`; plus any files it needs, all inside
`/home/user/submission/controller/`: a real folder, not a symbolic link, of at most 80 MB (1 MB is
2^20 = 1,048,576 bytes), 10,000 files and folders and 12 folder levels (a file or folder in
the folder itself is at level 1). Only its regular files and folders are installed on the host: symbolic links and
special files (FIFOs, sockets, devices) inside it are left out, and so is every folder named `__pycache__`. The
installed copy is the controller's current directory and is read-only for it (it belongs to the host's administrator
account: creating, changing or removing a file there fails with a permission error); the controller's process may
write in `workdir`, which is also its home (`HOME`), and in its own temporary folder (`TMPDIR`, also `TMP` and
`TEMP`), both empty when it starts. The interface, the budgets and the rules are in
`given/controller/controller_interface.md` and
`given/servo/bus_and_host.md`. The controller runs on the cell's host computer: an unprivileged account, no network,
only the installed files of `submission/controller/` (the documents are not there), Python 3.12 with the packages of
this machine (numpy, scipy, mujoco, pinocchio, trimesh, ...).

## How it is accepted

The acceptance (`given/acceptance/acceptance_spec.md`) runs on 2 units of the hand. Each unit works
2 shifts of 10 jobs back to back, with a pause of 10 s
between them. The units run one after the other, each with a new controller, on the cell's host computer:
4 CPUs and 8,192 MB of memory for the controller and the host's own runtime together, of which the
runtime uses at most 2,048 MB. The whole acceptance has a limit of 13,500 s of wall clock,
sized for a controller that uses every budget below; an acceptance that reaches it is void: it gives no score. Every
rule that scores zero, and how the score is computed:

- **Ends the shift** (the shift's remaining jobs are not run and score 0): a crush (a brittle shell's spring force
  above its size code's printed ceiling for 10 ms or more, or a soft pack's spring travel above its
  printed limit for 10 ms or more, at any time); a drop (during the transfer the specimen moves more
  than 20 mm in the hand, or no pad touches it for 50 ms); a shutdown (any
  servo's overheat or overload protection turns its torque off, at any time; that servo stays off for the rest of the
  unit).
- **Loses the job** (it scores 0): the hand does not clear the specimen's place in the pick nest by
  2 mm within 2 s of the job's start (not presented); no `grasped` flag
  within 3 s of the placement; the specimen
  not lifted at the end of the test lift (it touches the pick nest or its lowest point is less than
  4 mm above it); the specimen not seated on the place nest when the release is requested
  (not seated: it must lie inside the place nest's pocket with its lowest point at most
  1 mm above the place nest's plate); a release later than 1 s, or one that moves the
  specimen 3 mm or more horizontally on the place nest; a specimen that is to be declined but is not declined;
  a specimen that is to be carried but is declined.
- **A carried job scores** q = carried x L(slip; 1.5 mm, 6 mm) x L(rotation;
  3 deg, 12 deg) x released, where L is 1 at or below its first value, 0 at or
  above its second and linear between, and slip and rotation are the specimen's largest motion in the hand during the
  transfer.
- **A shift scores** S = (the mean of q over its carry jobs) x (0.7 + 0.3 J),
  J = max(0, TPR - FPR) over its jobs (TPR: the share of the decline jobs declined; FPR: the share of the carry jobs
  declined).
- **The acceptance score** is the mean of S over the four shifts. It is 0 if any pass condition fails: `controller.py`
  is a regular file that defines `class Controller`; the controller folder is a real folder of at most
  80 MB, 10,000 files and folders and 12 folder levels (a file or
  folder in the folder itself is at level 1); it passes the static import audit (no networking, no starting of other
  programs, the other items of the controller interface; every `.py` file at most 4 MB,
  in UTF-8, ASCII or Latin-1, and parsed by the host's Python, also one that is never imported; the audit has at most
  300 s of CPU and 600 s of wall clock and fails when it does not finish
  within them); `setup` returns a valid reply on every unit; at least 90 % of each unit's cycles are
  on time; no file of the delivered controller is written after the hand-over; no process it started is left after
  the run; it makes no network connection.
- **The controller is ended** for the rest of a unit (the servos keep their last registers; whatever is not done
  scores 0) when a step uses more than 1 s of CPU or 5 s of wall
  clock, when its process dies (the host ends any process of the controller that has used 7,200 s of CPU
  in all, and ends the controller's processes first when the memory runs out), on a protocol error (a malformed
  reply), or when its replies take more than 3,600 s of wall clock summed over one unit.

## Notes

- Session time: you have 6 hours. Leave a working controller in `submission/controller/` well before
  the end; an episode cut by the time limit is not graded.
- Run `python3 given/controller/check_controller.py /home/user/submission/controller` before you finish: it checks that
  the controller imports, answers `setup` and `step` in the expected format and keeps the cycle's CPU budget, against a
  toy plant of 13 toy servos, one of them pressing a spring; it also lists every entry of the folder that is
  not installed (a symbolic link, a special file, a `__pycache__` folder) and every process of the controller left
  after its end request. It exits 1 when it finds no controller to start (no folder, a folder that is a symbolic
  link, no regular `controller.py`), when the import fails, when `setup` fails or its reply is not valid or over its
  budget, when a step reply breaks the format, when a step ends the controller as at acceptance (more than
  1 s of CPU or 5 s of wall clock, a process that dies, a protocol
  error), or when a process of the controller is left after its end request; otherwise it exits 0: an entry it left
  out of the installed copy, a step that raised and a late cycle are in its report but do not change the exit code.
  The toy's current law is not the hand's, and the self-test says nothing about how well the controller will do.
- You may build any model or simulator of your own from the documents; nothing of it is graded, only the controller's
  behaviour at acceptance.
