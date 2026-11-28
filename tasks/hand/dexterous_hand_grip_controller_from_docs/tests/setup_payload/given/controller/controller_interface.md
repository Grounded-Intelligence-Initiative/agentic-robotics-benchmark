# The grip controller's interface

This is how the cell's host runs your grip controller at a hand unit's acceptance. Every limit and every bus figure
here is also in the acceptance specification or in `../servo/bus_and_host.md`, with the same value: the three
documents are written from one list of values.

## The deliverable

`submission/controller/controller.py` defines `class Controller`. The folder `submission/controller/` may hold any
other files the controller needs. It is a real folder, not a symbolic link, of at most 80 MB in
all (1 MB is 2^20 = 1,048,576 bytes), 10,000 files and folders and 12 folder levels (a
file or folder in the folder itself is at level 1). Only its regular files and folders are installed on the host:
symbolic links and special files (FIFOs, sockets, devices) inside it are left out, and so is every folder named
`__pycache__`. The installed copy is the controller's current directory and is read-only for it (it belongs to the
host's administrator account: creating, changing or removing a file there fails with a permission error); the
controller's process may write in `workdir`, which is also its home (`HOME`), and in its own temporary folder
(`TMPDIR`, also `TMP` and `TEMP`), both empty when it starts. The host imports `controller.py` from the installed
copy and creates one `Controller` per hand unit:

| call | when | budget |
|---|---|---|
| `Controller(workdir)` | once per unit; `workdir` is an empty folder you may write to | 60 s of wall time for the import and the constructor |
| `setup(record, cell)` | once per unit, before the hand-over | 60 s of CPU, 120 s of wall time |
| `step(obs)` | once per bus cycle | the cycle's effective period of CPU (below) |

`record` is the unit's own record (the format of `unit/example_record.json`, explained in `unit/record_format.md`).
`cell` is the cell's data: the nests, the taught grasp, the test lift, the arm's motion and the transfer profiles, the
specimen sheets, placement and the timeline (the cell's documents explain each). Nothing at setup says which jobs
will come: each job's ticket arrives in that job's observations.

The controller runs as an unprivileged account without network, in a fresh home and temporary folder, with one BLAS
thread and the same numerics settings as this machine (`OPENBLAS_CORETYPE=Haswell`, `OPENBLAS_L2_SIZE=256`,
`NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR"`). It may use the Python standard library and the packages
installed in this machine's image, but no networking module, no `ctypes` or `cffi`, no way of starting another
program, no import under a computed name and no `exec` / `eval` / `compile` of computed code: the import audit below
says exactly what is refused. CPU counts every thread and every process the controller starts. Anything a controller
prints goes to a log, not to the bus; the log keeps the first 5 MB of what the controller
prints in a unit and drops the rest (its last line says how much was dropped); printing more never ends the
controller.

When a unit ends, the host asks the controller's process to end: the process shuts down the process pools of
`concurrent.futures` it still has open, runs its exit handlers (`atexit`) and ends without waiting for its threads
(a thread is not a process: it need not be ended). A process the controller started is not ended for it: the host
waits until the controller's process has ended and no process of the controller's account is left, at most
5 s after the request, and only then counts every process of the account that is still there (running,
or ended without being waited for) as a process left after the run; the pass condition "no process it started is
left after the run" then fails (end, and wait for, every process your controller starts).

The units are accepted one after the other, each with a new controller; nothing the controller writes is kept for the
next unit. The host has 4 CPUs and 8,192 MB of memory for the controller and the host's own
runtime, which uses at most 2,048 MB; when the memory runs out the controller's processes are ended
first, and the unit goes on as for a controller that dies. Each process of the controller may use at most
7,200 s of CPU in all. The whole acceptance has a limit of 13,500 s of wall clock; an
acceptance that reaches it is void: it gives no score.

## The import audit

Before the controller is started, every `.py` file of the installed copy of `submission/controller/` is checked as
text, also one the controller never imports. A `.py` entry that is not a regular file (a symbolic link, a FIFO) is
not installed (see the deliverable), so it is neither checked nor importable (the self-test lists it among the
entries it left out). One refused file fails the import audit, a pass condition of the acceptance (section 9 of the
acceptance specification): the acceptance score is then 0. A file is refused when it:

- is larger than 4 MB, cannot be read, declares an encoding other than UTF-8, ASCII or
  Latin-1, or does not parse with the host's Python;
- imports a refused module or a module inside one (`import m`, `import m.x`, `from m import x`, `from m.x import y`,
  and `from p import x` when `p.x` is refused, as `from multiprocessing import connection`); the refused modules:
  `_ctypes`, `_posixsubprocess`, `_socket`, `_ssl`, `aiohttp`, `asyncio`, `boto3`, `botocore`, `cffi`, `ctypes`,
  `ftplib`, `grpc`, `http`, `httplib`, `httpx`, `huggingface_hub`, `imaplib`, `logging.handlers`,
  `multiprocessing.connection`, `multiprocessing.managers`, `nntplib`, `paramiko`, `poplib`, `pty`, `pycurl`,
  `requests`, `selectors`, `smtplib`, `socket`, `socketserver`, `ssl`, `subprocess`, `telnetlib`, `urllib`, `urllib2`,
  `urllib3`, `websocket`, `websockets`, `wget`, `xmlrpc` and `zmq`;
- calls or names one of
  `system`, `popen`, `execv`, `execve`, `execvp`, `execvpe`, `execl`, `execle`, `execlp`, `execlpe`, `spawnv`,
  `spawnve`, `spawnvp`, `spawnvpe`, `spawnl`, `spawnle`, `spawnlp`, `spawnlpe`, `posix_spawn`, `posix_spawnp` or
  `fork_exec`
  as an attribute of the module `os`, `posix`, `subprocess` or `_posixsubprocess`,
  called or not (`run = os.system` too), under the module's own name or any other name bound to it anywhere in the
  file (`import os as o` and then `o.system`; also `o = os`, `sys.modules["os"]`, `__import__("os")`), or through
  `getattr(os, "system")`, `vars(os)["system"]` or `os.__dict__["system"]`; or imports one of them from such a module
  (`from os import system`; `from os import *` imports them all and is refused too). A function, method or attribute
  of the controller's own that only has such a name is not refused (`def system(...)`, `self.popen()`);
- calls an import function (below) with a string literal that names a refused module or a module inside one; or,
  except for `spec_from_file_location` and `load_module`, calls one with a first argument that is not a plain string
  literal (a name, an f-string, any other expression, or no argument) or uses one otherwise than by calling it or
  assigning it to a name (passing, storing or returning it); the import functions are
  `__import__`, `import_module`, `find_spec`, `spec_from_file_location` and `load_module`;
- calls a code function (`exec`, `eval` and `compile`: below) with a first argument that is not a plain string literal
  (or with none), or uses one otherwise than by calling it or assigning it to a name (as in `map(eval, xs)`).

The import functions and the code functions are matched as what they are:

- `exec`, `eval`, `compile` and `__import__` as the built-ins: by the plain name where the file does not bind that
  name itself; as an attribute of the module `builtins` under any name bound to it (`import builtins as b`,
  `b = builtins`, `sys.modules["builtins"]`, `__builtins__`), also through `getattr(builtins, "eval")`,
  `vars(builtins)["eval"]`, `builtins.__dict__["eval"]` or `__builtins__["eval"]`; or a name imported from it
  (`from builtins import eval as e`) or assigned from one of these (`e = eval`). A file binds a name itself where
  it is a parameter, or an assigned, imported, defined, loop, `with` or `except` name, of the function it is used in
  or of a function around it, or where a statement at the module's own top level (not inside an `if`, `try`, loop
  or `with` block) defines, imports or assigns it and the module never deletes it;
- `__import__` also as an attribute of `importlib`, and the other import functions as an attribute of that name
  of `importlib` or of any module inside it, also at the end of a chain of attributes and calls that starts
  there (`importlib.util.find_spec`, `importlib.machinery.SourceFileLoader(n, p).load_module`), under any name
  bound to such a module or object (`import importlib as il`, `import importlib.util as u`, `from importlib import
  util`, `u = importlib.util`, `sys.modules["importlib"]`, `getattr(importlib, "import_module")`), or a name
  imported from `importlib` or a module inside it (`from importlib import import_module as im`, `from
  importlib.util import find_spec`, `from importlib import *`) or assigned from one of these.

A function, method or attribute of the controller's own that only has such a name is not refused (`def compile(...)`
and then `compile(x)`, `self.find_spec(x)`), nor is a function of another module that has it (`from re import
compile` and then `compile(p)`, `re.compile(p)`, `model.eval()`).

These checks have at most 300 s of CPU and 600 s of wall clock, and checks
that do not finish within them fail the audit.

## The setup reply

`{"period_ms": P, "reads": [[address, length], ...]}`: the cycle period you ask for, an integer 5 to 20 ms, and 1 to 4
register blocks read from every servo every cycle, each inside the RAM area of the control table (address 64 or more,
address + length at most 148). Anything else fails the setup; a unit whose setup fails gets no cycles, and its jobs
count as not done.

## The bus cycle

Each read block is one sync read of the 13 servos: a 27-byte request and 13 status packets of 11 + length bytes at
0.0025 ms per byte, plus 1 ms of converter turnaround; the cycle's sync writes take 1.5 ms. The effective period is
T = max(P, ceil(1.5 + the sum over the blocks of (1 + (27 + 13 x (11 + length)) x 0.0025))) ms. One block of 10 bytes
at P = 5 gives 5 ms; four such blocks give 9 ms.

Cycle k starts at t_k = k x T (the servos' firmware runs in 1 ms ticks). At t_k the reply of the previous step is
written to the servos; it is in force from tick t_k + 1. The registers your step receives are sampled at tick
t_k + 2. The hand does not move on while your step computes.

A step that uses more than T of CPU is late by floor(cpu / T) cycles: those cycles get no observation and no command
(the servos keep their registers), its reply is written after them, and the next observation's `late` says how many
cycles were skipped. A step that raises sends no command that cycle. At least 90 % of each unit's cycles must be on
time. A step over 1 s of CPU or 5 s of wall time, a controller that dies, a reply that breaks the format below, or more
than 3,600 s of your own reply time summed over one unit ends the controller for that unit: the servos keep their last
registers, the cell runs on, and whatever is left undone counts as not done.

## The step reply

`{"write": {field: [13 entries]}, "tactile_zero": [module, ...], "grasped": true | false, "decline": true | false}`,
or `null` for no command; a missing key reads as no write, no re-zero, false.

- `write`: any of `torque_enable` (64), `operating_mode` (11), `position_d_gain` (80), `position_i_gain` (82),
  `position_p_gain` (84), `goal_current` (102), `profile_acceleration` (108), `profile_velocity` (112) and
  `goal_position` (116). Each value is a list of 13 entries in servo id order, each an integer (written without a
  fraction: `600`, not `600.0`) inside the register's printed range, or `null` for "not written". Any other field, a
  list of another length, a boolean, a non-integer or a value out of range breaks the format. A reply line longer than
  1048576 bytes (its line end not counted) breaks it too.
- All writes of one reply are applied at the same tick in this order, within a field by servo id: (1) the
  `torque_enable` entries equal to 0; (2) `operating_mode` (refused for a servo whose torque is still on, and for a
  value that is not an operating mode; an accepted write resets that servo's position gains, profile, Goal PWM and
  Goal Current as the servo manual says); (3) the RAM fields by ascending address, so gains written together with a
  mode change survive it; (4) the `torque_enable` entries equal to 1 (refused for a servo stopped by its protection).
  A refused write changes nothing and is listed in the next observation's `refused`.
- `tactile_zero`: the modules (`thumb`, `index`, `middle`, `ring`) whose zero is set to what they read at that tick,
  contact included.
- `grasped`: the specimen is held (it ends the closing window). `decline`: this specimen will not be carried.
  `decline` wins over `grasped`. When each flag counts is in the cell's documents and the acceptance specification.

## The observation

`{"cycle", "t_ms", "period_ms", "late", "regs", "tactile", "cell", "refused", "event"}`:

- `period_ms` is the effective period T; `t_ms` the time of the sample in ms since the hand-over.
- `regs`: every register lying wholly inside your read blocks, by its key (the vendor's data name in snake case, e.g.
  `present_current`), a list of 13 integers, signed as the control table says.
- `tactile`: the newest frame that arrived since the previous observation, or `null`:
  `{"seq", "t_capture_ms", "t_arrival_ms", "modules"}`, where `modules` is `{module: hex}` (the frame layout is
  in `tactile/tactile_interface.md`).
- `cell`: `{"shift", "job", "phase", "t_phase_ms", "ticket"}`; the ticket (`{"job", "class", "code", "finish",
  "profile"}`) is shown from the job's `await_clear` to its `rest`, and is `null` outside a job.
- `refused`: the previous reply's refused writes as `[field, servo id]` pairs.
- `event`: `null`, or `{"kind": "crush" | "drop" | "shutdown", "shift", "t_ms"}` from the moment an event ended a shift
  to the end of the unit.

Nothing else: no pose, no force.

## The self-test

`python3 check_controller.py [your controller folder]` starts your controller as the host does in these respects:
it installs a copy of the folder that holds what the host installs (regular files and folders only) and lists every
entry it left out (`left_out` in its report); it makes that copy read-only for the controller, as the host's installed
copy is (its folders and files, and the folder that holds it, before the controller starts), though it cannot mirror
all of it: there the copy belongs to the account that runs the self-test and the controller runs as that account, so a
controller that changes the copy's modes back can still write there, and run as root the self-test stops no write;
and it starts the controller in a process of its own with the same
environment, speaking the same JSON lines. It does not run the import audit, and it does not apply the host's
network filter, the CPU limit of each process or the host's memory size. It sets the controller up with
`unit/example_record.json` and `cell/cell_data.json`, and steps it on the bus cycle against `toy_plant.py`, a toy
plant that is not the hand. It reports the import, the setup reply and the effective period, every reply that
breaks the format, every step that raised, the CPU of each step against the period, and every process of your
controller left after its end request, waited for and counted as the host does (at most 5 s; running,
or ended without being waited for), with exit code 1: at acceptance such a process fails the pass condition "no
process it started is left after the run" (a process that has left your controller's session is counted at
acceptance, but the self-test does not see it). It says nothing about how your controller would do on the hand.
`example_controller.py` shows the interface: it opens the hand and holds it there, and completes no job.
