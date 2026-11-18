# Lace, box and ship pairs of sneakers with a two-arm finishing cell, seen only through its cameras

You are the automation engineer of a children's-sneaker factory's finishing line. Lacing is the last manual step of
the line: a pair comes off the last with the lace threaded through the toe-most eyelet pair only, and someone has to
lace the shoe criss-cross, pull it snug, erect the flat shoe-box blank, put the pair in, put the lid on and send the
box on. Today the cell does it: two Franka Panda arms on a bench, the LEFT one with a parallel-jaw gripper, the RIGHT
one with a lacing hook (a thin shaft with an L-foot and an upturned tip that catches a lace aglet), a lacing cradle
that clamps a seated shoe, an infeed tray where the pair and its lid are set down, a blank spot where the flat
box blank lands, and an outfeed line. You see the cell only through its cameras: an overhead, a front and a side
RGB-D camera, and a wrist RGB-D camera on each arm. Their calibration (intrinsics; the fixed cameras' poses; the wrist
cameras' mounts) is exact and published in the cell manual, and so are the batch's dimensions (the shoe model, the
box). Nothing on the API tells you where a shoe, an eyelet, a lace end, a box panel, the lid or the cradle is, whether
a lace is through a ring, whether a wall has locked, or how much credit you have earned: you must perceive it.

The work order of an episode is one pair of shoes; for it the cell must:

1. **Lace** each shoe: seat it sole-down, heel-back in the cradle (the cell clamps it; `release_clamp()` frees it),
   then thread each lace end through the remaining eyelet row(s) criss-cross: from row 1 (toe-most, already threaded,
   both ends leaving the shoe outward) a lace end enters row 2 on the OTHER side from the outside and comes back up
   out of the cavity through the throat slot (a batch with more rows continues: row 3 on its own side from the
   outside, up through the slot again, and so on: every ring is entered from the outside, the sides alternate), and
   pull the lace snug after each pass. The shoe is a hollow lasted shell: the eyelet flaps are rigid and the cavity
   under them is reachable only through the throat slot, which is where the hook works (it fetches the lace end from
   under a flap and lifts it out through the slot). Both aglets must end up outside the shoe.
2. **Erect** the box blank: fold the four walls up until they snap-lock (87 degrees held for 0.3 s locks a wall at
   90 degrees).
3. **Pack** the laced pair: both shoes inside the box, sole down, heel-to-toe (one shoe pointing each way), nothing
   resting on the walls.
4. **Close** the box: fetch the lid tray from the infeed tray and set it over the erected walls (centred within
   12 mm, aligned within 6 degrees, resting).
5. **Ship**: push the closed box with the pair inside across the outfeed line; the cell takes it away, which completes
   the work order (a box without both shoes inside stays where it is).

One batch per episode: every pair has the same shoe model (length, width, two eyelet rows, spacing, throat gap) and
box size, published in `describe()["objects"]`. Per pair the colours vary, the shoes and the lid land on the infeed
tray either way round with a random turn (a shoe may lie on its side) and the blank lands either way round. The lace
is tacked at row 1 (stitched to the insole between the two row-1 rings), so each tail keeps its length.

## What is graded: one graded episode, played by you during this session

Nothing in `/home/user/submission/` is graded. Your score is the outcome of ONE graded episode that you play yourself,
during this session, through the cell's graded service: one work order (one pair) on a hidden seed (not one you can
choose or read; the cell builds its scene like every episode's), run by the cell's own code under the same rules as
every episode: 900 s of simulated time, at most 500 ticks per `step`, 450 camera frames, the safety rules and the work
score below. Simulated time advances only inside your own requests (`step`; a frame renders the state as it is): the
time you spend thinking, perceiving and writing code between two requests costs no simulated time, and the graded
episode has no program-time or silence limit (the program-time and silence budgets that `describe()["budgets"]`
lists apply to programs run on the practice wire, not here).

Connect from any process of yours with `shoecell.client.connect_graded()`. It returns the same `cell` proxy a
`solve(cell)` receives (API in `~/given/cell/cell_api.md`, section "The graded episode"), plus `status()`, `start()`
and `close()`:

```python
import sys
sys.path.insert(0, "/home/user/given/lib")
from shoecell.client import connect_graded, decode_frame

with connect_graded() as cell:              # one client at a time: a second one gets CellError("busy: ...")
    print(cell.status())                    # started / ended, steps, sim_time, prep_left_s, graded_left_s
    cell.start()                            # ONCE per session: starts the graded episode
    info = cell.describe()
    rgb, depth = decode_frame(cell.get_camera("overhead"))
    cell.set_control("left", "cartesian", 0.6)
    cell.command("left", tcp_pos=[0.15, 0.0, 0.30], tcp_quat=[0.0, 1.0, 0.0, 0.0])
    print(cell.step(100))
# leaving the block closes the connection; the episode stays where it is, and the next connect_graded() resumes it
```

You can drive it from many short scripts, step by step, or run a program of yours on it (`import packer;
packer.solve(cell)` on a connected `cell`). The rules:

* **One start.** `start()` succeeds once per session. It is charged before anything is simulated and never given
  back, and nothing restarts or resets the graded episode (`reset` and `truth` answer `forbidden` there).
* **Two windows.** The graded episode must be started within 4 hours of the start of your session (`status()` gives
  `prep_left_s`); after that `start()` is refused and the task scores 0. Once started, it runs for at most 3 hours of
  wall-clock time (`graded_left_s`).
* **One client; the episode persists.** One connection at a time. The episode lives in the service, not in your
  connection: a script that ends, crashes or disconnects leaves it exactly where it was (the last commands stay
  commanded, and nothing moves until the next `step`). A connection that sends no request for 15 minutes is closed;
  the episode stays.
* **How it ends.** At its own end (900 s simulated, or a physics fault), when you call `cell.finish()`, when its
  3 hours run out, or when your session ends. `finish()` ends it for good: a `solve(cell)` that calls `cell.finish()`
  at its end (the example does) ends the graded episode too. After the end only `status()` answers (after its own end
  you may still read its state and frames, as a program may).
* **How it is scored.** When it ends, it is scored as it stands at that moment, on the cell's own physical state, by
  the rules below, exactly as the grader scores an episode whose program stops there; nothing is simulated afterwards
  (a box still on its way to the outfeed line is not shipped). If your session ends before the graded episode has
  ended, it is scored as it stood after your last request.
* **What you learn.** What a program learns from the cell (its replies, frames and events) plus `status()`; never
  the score or the episode's seed. Nothing else you do or leave is graded.

## What you are given

| path | content |
|---|---|
| `~/given/cell/cell_manual.md` | the integrator's cell description: bench, the two robots' base poses and tools, the cameras with their exact calibration, the zones (infeed tray, blank spot, cradle, outfeed line) with their nominal positions and the per-episode jitter, what a shoe, a lace, a box blank and a lid are (geometry, eyelet layout, colours), the attach rules of the gripper and the hook, the cradle clamp, the carton's snap-locks, the lid rule, safety rules, budgets |
| `~/given/cell/cell_api.md` | the cell's programming manual (the wire API) |
| `~/given/cell/cell_config.json` | the cell's configuration as far as it is public |
| `~/given/lib/shoecell/` | the cell's library: `client` (the proxy, `decode_frame`, `connect_graded`: the graded episode), `local` (`LocalCell`: the same cell in your own process on a seed of your choice, with `truth()` for development), `practice` (the wire), and the cell's own code (`physics`, `control`, `cameras`, `episode`, `server`): the graded service runs this exact code on the graded episode |
| `~/given/panda/` | the robots' MuJoCo model (Apache-2.0) |
| `~/practice/grade.py` | runs your program over the real wire against the practice cell on a visible seed and prints the raw work score with the per-pair milestones |
| `~/example/packer.py` | a complete, policy-free program (reads frames, moves both arms, finishes; earns nothing): a starting point for a `solve(cell)` |

Tools: Python 3.12 (`/opt/venv/bin/python3`; call it by that path, the bare `python3` on your shell's `PATH` may be
another interpreter) with numpy, scipy, OpenCV (`cv2`, headless), Pillow, MuJoCo 3.14. No network in your session.

## How the graded episode is scored

`work_score` = the accepted work of the graded episode's pair, which can earn
1.0 = lacing of shoe A (0.30) + lacing of shoe B (0.30) + box erected (0.12) + pair packed (0.13) + lid closed (0.10)
+ box shipped (0.05), judged on the cell's own physical state when the box is shipped or when the episode ends, and
0 for a pair during which a safety violation occurred. The work order is a laced pair in a shipped box: "packed",
"closed" and "shipped" are credited in proportion to the pair's laced shoes (times 1/2 with one shoe laced through every
row, times 0 with none), so a pair boxed unlaced earns the box-erected credit only. The
lacing of a shoe = (rows laced in order from row 2, both ends, correct side, entered from the outside, counted
consecutively until the first miss) / (rows - 1), times 0.7 + 0.3 x (the fraction of those rows that are snug: no more
than 50 % slack between consecutive rows, plus 3 cm for the dip under the flap and the return through the throat), and
0 if an aglet is left inside the shoe (passed in and never brought back out). "Packed" needs both shoes inside the
erected box, sole down (within 15 degrees), heel-to-toe (headings within 30 degrees of opposite), not resting on a
wall. "Closed" needs the lid on the box as above with both shoes inside, sole down, not resting on a wall (heel-to-toe
is the packing's alone). "Shipped" needs such a closed box with its floor centre across the outfeed line. The reward
you are judged on is the graded episode's `work_score` relative to a hidden reference value.

`safety`: a Panda link other than the gripper's finger pads or the hook pressing more than 30 N on the bench, the
cradle or an object, or any contact between the two arms above 30 N, for 30 ms (three control ticks) in a row, or an
object faster than 3 m/s, is a violation that voids the credit of the pair being worked on when it happens (even if
that pair is later shipped). A physics fault (the simulation diverging under a violent action) ends the episode with
the credit of the pairs shipped before it.

## Rules of the cell (details in `cell_manual.md` / `cell_api.md`)

* The gripper grasps what its documented attach rule allows: closing the fingers on a grasp handle (an aglet, the
  heel counter or the toe cap of a shoe, the free edge of a box panel) that lies within the pinch box between the
  finger pads attaches it to the hand at its current pose (an aglet is seated along the approach axis); opening
  releases. The hook's clip catches an aglet within 12 mm of its tip and seats it along the tip; `hook(False)`
  releases; closing the gripper on an aglet that sits on the hook takes it off the hook. Neither tool reports what it
  holds beyond the gripper width and the clip sensor.
* A shoe set down sole-down, heel-back in the cradle seat is clamped 0.3 s after it comes to rest; `release_clamp()`
  frees it. The clamp never reports.
* A wall of the blank that reaches 87 degrees for 0.3 s snap-locks at 90 degrees. Locks never release. Nothing
  reports a lock, and nothing reports whether the lid sits right.
* Calls other than the documented public ones answer `{"error": "forbidden"}` and are counted.

## Practice

`LocalCell(seed=...)` and `~/practice/grade.py` give you the same cell on seeds of your choice, with `truth()` (every
pose, the eyelets, the lace passes, the carton state, the milestones) for development only: the graded cell never
answers it. The graded episode uses another seed; the cell builds its scene with the same code.

## Notes

- Photos and camera frames are meant to be processed programmatically; view only the images you need.
- Do not modify the given files in your home directory; the graded service runs its own copies.
- Environment facts (harness limits): a single tool call whose input runs above roughly 25-30k tokens is dropped by the
  harness (it is never executed and no error comes back), and a response that reaches the 64k output-token limit is lost
  and, after at most a few retries, ends the session (the graded episode is then scored as it stood). Write large files
  in several parts (one function group per write) and keep every single response well below that limit.
- Session time: this session is stopped after 7 hours of wall-clock time from its start. A session stopped by that limit
  is not graded at all, so a late finish scores nothing. Note the start time with `date` when you begin, start the
  graded episode early enough to finish it (an episode started late cannot use its whole 3 hours), and end the session
  yourself (stop working and say that you are done) at least 15 minutes before the limit.
