# Fit an engine's accessory belt with two Franka Pandas, by touch, force and camera

You are the automation engineer of a compact-engine dressing line. Its belt station fits the poly-V accessory belt:
two Franka Pandas stand side by side in front of the engine, each with a parallel-jaw gripper, a force/torque sensor at
its flange and a wrist RGB-D camera; a third RGB-D camera looks over the drive. Each engine arrives on its pallet fully
dressed except for its belt; the new belt hangs as a loop on a peg left of the drive.

Each episode is one engine. Fit the belt the way a technician does, following the routing decal (variant A):

1. lift the belt off its peg and route it over the crank pulley C, the water pump WP and the alternator ALT (their ribs
   in the grooves), leaving the last span for the automatic tensioner T (its pulley runs on the belt's back);
2. swing the tensioner off the belt by its service lever, along its arc, and hold it there against its spring;
3. with the other arm, put the last span on (under the tensioner pulley, in line with the other pulleys);
4. let the tensioner back slowly onto the belt -- never let it go while it is off the belt;
5. check that every rib section is in its groove zone (not on a shoulder or edge), that the belt runs on the tensioner
   pulley's face, and that the tensioner's pointer rests between its window marks; correct what is not.

Nothing on the API tells you whether a span is seated, whether a grip slipped, whether the tensioner caught the belt or
whether a pulley is wrapped: the cell reports only what the robots' own sensors and the cameras report (joint angles,
tool poses, grippers' openings, wrist forces/torques, RGB-D frames). How the belt hangs, slides and seats, how the
tensioner behaves and what the grip holds are described at data-sheet level in `~/given/catalog/catalog.md` and
`~/given/cell/cell_manual.md`; the engine's exact pose, the belt's stiffness, mass, length and friction and the
tensioner's spring and damper vary from engine to engine within the documented ranges and are for your program to find
out on the station.

## What you produce (in `/home/user/submission/`)

`belt.py` exposing `solve(cell) -> None` (plus any helper modules, optional `run.sh`). The grader runs it once per
hidden engine in its own cell. It talks to the cell through `cell` (API in `~/given/cell/cell_api.md`). Per episode:
1500 s of simulated time (the clock advances only inside `step`); 2400 s of program time, which is the episode's wall
clock minus the time the cell spends simulating (inside `step`) and rendering (inside `get_camera`), so it measures your
program's own computation and its other requests, and 4x that in CPU seconds (all threads together); at most 120 s from
a reply to your next request; 600 camera frames in total.

How the program is run: a fresh process under a dedicated unprivileged account that is not yours, from a per-run
directory with a read-only copy of your submission folder (regular files only), an empty writable HOME / TMPDIR, a
minimal environment (`PATH`, `HOME`, `TMPDIR`, `LANG`, `MUJOCO_GL`, `PYTHONPATH` = the given library `~/given/lib`,
`ARB_GIVEN_DIR` = a read-only copy of `~/given`) and the launcher `python3 -B -E -s -c <bootstrap> <lib> belt` (or
`bash run.sh`), with no network (a seccomp filter refuses every socket but a Unix-domain one, for the program and
everything it starts). Before the first episode the grader imports your module once under this regime to check that it
defines `solve`: the import must finish within 120 s of wall clock and 120 CPU seconds (all threads together), or every
episode scores 0; do heavy precomputation inside `solve`, not at module level. Resolve given files through
`os.environ.get("ARB_GIVEN_DIR", os.path.expanduser("~/given"))`, write only under the run's HOME / TMPDIR. A program
that crashes ends that episode with what it achieved; the other episodes still run. An episode in which the cell
itself failed (a physics fault, a frame it could not render) is run once more from scratch, and the second run counts;
at most one episode per grading is re-run, and a further cell failure scores 0 for that engine.
Your submission must be final when your session ends: the submission folder is then copied out of your sandbox and
graded in a fresh sandbox that holds nothing else of yours (no other file, no process you started), and a surviving
process of the run account voids the contract gate.

## What you are given

| path | content |
|---|---|
| `~/given/cell/cell_manual.md` | the station: frames, the engine and its drive, the robots and their control modes, the hand's data sheet, the grip rule, the force/torque sensors, the cameras, safety and damage, the initial state, the acceptance, time and budgets |
| `~/given/cell/cell_api.md` | the cell's programming manual (the wire API) |
| `~/given/cell/cell_config.json` | the cell's configuration as far as it is public |
| `~/given/cell/describe_example.json` | one episode's full `describe()` answer (every key and unit; the same in every episode) |
| `~/given/catalog/catalog.md` | the drive drawing (pulleys, grooves, the tensioner, its lever, pointer and window marks), the belt's and the tensioner's data sheets, the run-in test |
| `~/given/survey/` | photographs (RGB-D frames with their camera poses) of a station with an engine without a belt and with a correctly fitted belt |
| `~/given/lib/beltcell/` | the cell's client: `client` (the proxy, `decode_frame`, the program launcher); the cell itself runs in the grader, not in your account |
| `~/given/panda/` | the robot's MuJoCo model (Apache-2.0) |
| `~/example/belt.py` | a minimal program (reads the work order and a frame, moves an arm, finishes; scores 0): the starting point |

Tools: Python 3.12 (`/opt/venv/bin/python3`; call it by that path, the bare `python3` on your shell's `PATH` may be
another interpreter) with numpy, scipy, OpenCV (`cv2`, headless), Pillow, MuJoCo 3.16. No network in your session.

## How you are graded (verify runs the hidden engines)

Gates (multiply the score): `contract_ok` = `belt.py` is a regular file that parses and defines `solve`, passes the
route gate (no weight or data files: nothing ending in `.pt`, `.pth`, `.onnx`, `.safetensors`, `.ckpt`, `.h5`, `.pb`,
`.pkl`, `.npz`; no `.py` or `.sh` file containing `torch.optim`, `.backward(` or `jax.grad`; submission <= 60 MB),
imports under the run regime above (the import check), makes at least one `step` call over the episodes, holds no
network socket during any episode, and your own `.py` files import no networking module (`socket`, `ssl`, `urllib`,
`http`, `requests`, `asyncio`, `ctypes`, `subprocess`, ...), load no module under a computed name, execute no computed
code (`exec` / `eval` / `compile` of a non-literal) and start no other program (`os.system`, `os.exec*`, `os.spawn*`, ...).

`belt_acceptance` = the mean over the engines of g x q, judged after `finish` on the cell's own state (the acceptance
in `cell_manual.md`): the robots are switched off and out of the drive, the drive settles for 1 s and a run-in motor
turns the crank at 1.5 rev/s for 4 s against the accessories' drag; then g = 1 if nothing was damaged (the tensioner's
damper is rated for 60 deg/s of arm speed: a snap-back damages it; a belt that touches the base plate is scrap), there
was no protective stop (a robot link or hand surface, the finger pads excepted, pressing on anything -- the other arm
included -- with more than 30 N for 30 ms ends the episode) and the tensioner's pointer rests between its window marks,
else 0; q = (the pulleys C, WP, ALT and T that the belt wraps on the right side and lies on correctly -- in the groove
zone on the groove floor, or on the tensioner pulley's face between its flanges -- + 1 if each accessory turned within
3 % of the crank's angle times its ratio) / 5. The reward you are judged on is `belt_acceptance` relative to a hidden
reference value. Stats (not scored): the items, sim / wall / program time, frames, illegal calls, stops, peak lever load.

## No rehearsal

Like a cell program deployed to a customer's station without a trial run, your program meets each engine for the first
time in its graded episode: there is no simulator of the station, the drive or the belt in your sandbox. What you know
before the episode is what the documents above say; the rest your program learns on the station through the cameras,
the force/torque sensors and the grippers. The robot's own model (`~/given/panda/`, MuJoCo) is yours to simulate the
arms with, and the client accepts any transport (`Cell(send, recv)`), so you may build test rigs of your own.

## Notes

- Photos and camera frames are meant to be processed programmatically; view only the images you need.
- Do not modify the given files in your home directory; the grader reads its own copies.
- Environment facts (harness limits): a single tool call whose input runs above roughly 25-30k tokens is dropped by the
  harness (it is never executed and no error comes back), and a response that reaches the 64k output-token limit is lost
  and, after at most a few retries, ends the session with whatever is in `/home/user/submission/` at that moment. Write
  large files in several parts (one function group per write), keep every single response well below that limit, write
  a first complete submission early and refine it.
- Session time: this session is stopped after 6 hours of wall-clock time from its start. A session stopped by that limit
  is not graded at all, so a late finish scores nothing. Note the start time with `date` when you begin, keep a complete
  submission in place from early on, and end the session yourself (stop working and say that you are done) at least
  15 minutes before the limit.
