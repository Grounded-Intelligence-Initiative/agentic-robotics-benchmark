# Clean up a studio apartment after a dinner with a Unitree G1 humanoid

You are the robot integrator of a home-cleaning service. A customer had a dinner for two in their studio apartment
and has booked a clean-up; one Unitree G1 humanoid (29 DoF, Dex3-1 hands) is on site, just inside the door. Your job:
write the program that makes the robot work through the customer's cleaning list below, using only what the robot can
sense: its head camera, three ceiling cameras of the apartment, its joint encoders, its localisation and its hands'
tactile load. The apartment's fixed furniture is surveyed exactly (`~/given/cell/cell_manual.md`); nothing else is
given: where the litter, the dishes and the books lie, where and how big the grease spill is, where the crumbs are,
which of the two bins is the recycling one, whether a lid is open or the water runs, how much soap or grease the sponge
holds — all of that has to be seen in the camera images (and remembered). Every episode is a different mess.

## The cleaning list

1. **Trash into the bins.** The two drinks cans and the plastic water bottle go into the **blue recycling bin**; the
   three crumpled paper napkins and the apple core into the **grey general-waste bin**. The two flip-lid bins stand
   against the west wall south of the kitchen (which one stands where varies). A lid opens by lifting its front tab;
   lifted past about 55 degrees it stays open, below that it falls shut.
2. **Dishes to the dish tray.** The two plates and the two glasses from the dining table go onto the grey dish tray on
   the kitchen counter south of the sink (on the tray or on a dish on it), unbroken: a dish that hits something hard
   breaks.
3. **Books back on the shelf.** The two books on the dining table go into the top compartment of the bookshelf's
   west bay (north wall), east of the books already there, standing or lying.
4. **The dining table degreased.** Take the grease spill off the dining table's top with the kitchen sponge (in its
   steel tray on the counter north of the sink), dish soap (the pump bottle by the sink: press the sponge down on the
   pump's head) and water (the faucet's lever). A sponge only takes grease off while pressed flat on the table and
   sliding; soap makes it work, a sponge full of grease smears — rinse it under the running water and soap it again.
5. **The floor swept.** The crumbs round the dining table — those on the floor and any wiped off the table — swept with
   the broom into the lobby dustpan (both hang in clips on the south wall) and emptied into the general-waste bin.
6. **Things put back.** The sponge rinsed and back in its tray, the broom and the dustpan back in their clips, the water
   off, both bin lids closed.

The cell manual states every rule the cell applies (the grip rule, the mechanisms of the lids, the faucet, the pump
and the clips, how wiping, rinsing, soaping, sweeping and the dustpan work, what breaks, safety) with its numbers.

## What you produce (in `/home/user/submission/`)

`clean.py` exposing `solve(cell) -> None` (plus any helper modules, optional `run.sh`). The grader runs it once per
hidden episode (**3 episodes**) in its own physics cell; the program talks to the cell through `cell` (API in
`~/given/cell/cell_api.md`). Per episode: 4800 s of simulated time; 3600 s of program time, which is the episode's wall
clock minus the time the cell spends simulating (inside `step`) and rendering (inside `get_camera`), so it measures your
program's own computation and its other requests; at most 120 s from a reply to your next request; 1500 camera frames
in total.

How the program is run: a fresh process under a dedicated unprivileged account that is not yours, from a per-run
directory with a read-only copy of your submission folder (regular files only), an empty writable HOME / TMPDIR, a
minimal environment (`PATH`, `HOME`, `TMPDIR`, `LANG`, `MUJOCO_GL`, `PYTHONPATH` = a copy of `~/given/lib`,
`ARB_GIVEN_DIR` = a read-only copy of `~/given`) and the launcher `python3 -B -E -s -c <bootstrap> <lib> clean`
(or `bash run.sh`). Resolve given files through `os.environ.get("ARB_GIVEN_DIR", os.path.expanduser("~/given"))`, write
only under the run's HOME / TMPDIR. A program that crashes ends that episode with what it achieved; the other episodes
still run. Your submission must be final when your session ends: the submission folder is then copied out of your
sandbox and graded in a fresh sandbox that holds nothing else of yours (no other file, no process you started).

## What you are given

| path | content |
|---|---|
| `~/given/cell/cell_manual.md` | the cell: the apartment's survey (exact), the robot and its controllers, the grip rule, the sensors and cameras, the fixtures and the tools, the cleaning rules, what breaks, safety, budgets |
| `~/given/cell/cell_api.md` | the cell's programming interface and how to run a program against the practice cell |
| `~/given/robot/unitree_g1/` | the robot's MuJoCo model (Menagerie, BSD-3-Clause); the cell's modifications are in the manual |
| `~/given/lib/house_client/` | the cell's client (`Cell`, `connect_practice`, `decode_frame`, the launcher) |

**Practice cell.** A practice service runs the real cell for you (one fresh practice episode per connection, drawn
like the graded ones, never the same; at most 60 practice episodes in this session): see `cell_api.md`. It answers the
same public calls and nothing else. There is no answer key and no checker: you judge your program's work from the
cameras, like on site.

Tools: Python 3.12 at **`/opt/venv/bin/python3`** (use this path; the plain `python3` on your PATH may lack the
packages) with numpy, scipy, OpenCV (`cv2`, headless), Pillow, matplotlib and MuJoCo 3.16. No network in your session.

## How you are graded (3 hidden episodes)

Gates (multiply the score): `contract_ok` = `clean.py` is a regular file that parses and defines `solve`, imports under
the run regime above, makes at least one `step` call over the episodes, causes no network activity, and your own `.py`
files import no networking module (`socket`, `ssl`, `urllib`, `http`, `requests`, `asyncio`, `ctypes`, `subprocess`,
...), load no module under a computed name, execute no computed code (`exec` / `eval` / `compile` of a non-literal) and
start no other program. Per episode `safety` (cell_manual.md section 9): a violation zeroes that episode.

`cleaning_score` = the mean over the episodes of the checklist's credit / 10, read from the apartment's state when the
episode ends:
* trash, 2.0: each of the seven pieces 2/7 when it lies inside the right bin (below its rim);
* dishes, 1.0: each plate and glass 0.25 when it stands on the dish tray (or on a dish on it), unbroken;
* books, 1.0: each book 0.5 when it rests in the target compartment (its middle inside it, on the shelf or on the
  other book), not held;
* table, 2.5 x min(1, removed / 0.95): removed = 1 - (the grease left on the table, smears included) / (the grease at
  the start);
* floor, 2.5 x min(1, binned / 0.9): binned = the share of all the crumbs (the table's and the floor's) that lie in the
  general-waste bin;
* putting back, 1.0, for what was used: the sponge back in its tray and rinsed (its grease load below a tenth of what
  it holds) 0.3, once at least half of the grease is off; the broom and the dustpan each hanging in its clip 0.15,
  once at least half of the crumbs are in the general-waste bin; the faucet off 0.2, once the water has run; each bin
  lid closed 0.1, once that bin holds something that belongs in it.

The cell never tells your program what it has achieved; it reads the apartment's state when the episode ends.

## Notes

- Images (camera frames) are meant to be processed by your program; look at the ones you need.
- The survey of the fixed furniture, the camera calibrations, the robot's kinematics and joint angles are exact; the
  localisation is slightly noisy; everything about the mess must be measured in each episode.
- Do not modify the given files in your home directory; the grader reads its own copies.
- Environment facts (harness limits): a single tool call whose input runs above roughly 25-30k tokens is dropped by the
  harness (it is never executed and no error comes back), and a response that reaches the 64k output-token limit is
  lost and, after at most a few retries, ends the session with whatever is in `/home/user/submission/` at that moment.
  Write large files in several parts, keep every single response well below that limit, write a first complete
  submission early and refine it.
- Session time: this session is stopped after 4 hours of wall-clock time from its start. A session stopped by that limit
  is not graded at all, so a late finish scores nothing. Note the start time with `date` when you begin, keep a complete
  submission in place from early on, and end the session yourself (stop working and say that you are done) at least
  15 minutes before the limit.
