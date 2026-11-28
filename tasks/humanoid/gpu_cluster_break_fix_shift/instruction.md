# Work a night shift in an AI training pod with a Unitree G1 humanoid

You are the automation engineer of a data-centre operator. One of its AI training pods (two rows of five racks
round a contained cold aisle: eight 8-GPU servers, two network racks with the InfiniBand leaf switches) is looked after
at night by one Unitree G1 humanoid (29 DoF, Dex3-1 hands) working as the site's remote-hands technician. Your job:
write the program that works one maintenance shift the way the site's technicians do: take the tickets the NOC has
open, diagnose each fault with the site's services and the robot's own eyes and tools, repair what can be repaired on
site by the vendor's and the site's procedures, escalate what cannot, keep the training job that runs on the pod alive,
record what was done, and leave the pod as the shift found it except for the repairs. Nothing tells the program the
root causes: the tickets say what the monitoring saw, the services answer what a technician's terminal answers, the
cameras and the fibre probe show what a technician would see. Every shift is a different night: other faults, other
nodes and ports, other dirt, another pod of the same design with its own installation tolerances. There is no practice
cell: the program goes to the site as it is.

## The shift

The NOC queue holds three tickets when the shift starts; a fourth arrives during the shift.

1. **A GPU fault** on one node (the node is already drained): the health check's XID and the GPU's PCI address. Work
   it by SOP-GPU-01: the evidence first (the BMC's event log, DCGM), a cold power cycle (AC removed at the rack's
   switched PDU outlets), then the module itself: tray out, cover open, the module's four screws, the module out and
   back in or a spare in; a GPU that is not the module's fault (the fault stays with the slot) goes to the vendor.
2. **An optical link fault** between a node's GPU port and its rail's leaf switch: the fabric manager's counters and
   the transceivers' per-lane optical powers. Work it by SOP-NET-01, one step at a time: inspect, clean and inspect
   again both ends (the MPO connector's end face and the transceiver's receptacle) with the fibre probe and the
   one-click cleaner, swap a transceiver that has aged, hand a damaged fibre to the cabling team; the node is drained
   from the job before anything is unplugged, and a repaired link is soaked before the node goes back.
3. **A hot-swap ticket**: a failed power supply (SOP-PWR-01; mind the redundancy rule) or a failed fan module
   (SOP-THM-01), replaced while the node runs, within the vendor's 30 seconds.
4. **The ticket that comes in during the shift** (a fan or a second link).

Every ticket is closed in the NOC with its root-cause code and the parts record (the serial numbers removed and
installed, by position) or escalated with its evidence. At the end of the shift the pod is closed up: trays home and
latched, covers shut, rear doors shut, every PDU outlet on, failed parts in the RMA tote, nothing left lying about. The
site handbook says what else a technician leaves in order (a locator LED left on by a previous shift; a degraded part
nobody ticketed is reported, not touched).

## What you produce (in `/home/user/submission/`)

`shift.py` exposing `solve(cell) -> None` (plus any helper modules, optional `run.sh`). The grader runs it once per
hidden shift (**3 shifts**) in its own physics cell; the program talks to the cell through `cell` (API in
`~/given/cell/cell_api.md`; the site's services through `cell.noc(...)`, `~/given/site/services_api.md`). Per shift:
7200 s of simulated time (a 2-hour maintenance window; the services' own durations, a node's boot, a DCGM run, a soak,
count in it); 3600 s of program time, which is the shift's wall clock minus the time the cell spends simulating (inside
`step` and the services' calls) and rendering (inside `get_camera` and the probe's pictures), so it measures your
program's own computation and its other requests; at most 120 s from a reply to your next request; 2000 camera frames
in total; at most 14,400 s of CPU time for the program's process (all threads; SIGXCPU ends it beyond that), whose
numerical libraries run with `OMP_NUM_THREADS` / `OPENBLAS_NUM_THREADS` / `MKL_NUM_THREADS` = 2.

How the program is run: a fresh process under a dedicated unprivileged account that is not yours, from a per-run
directory with a read-only copy of your submission folder (regular files only), an empty writable HOME / TMPDIR, a
minimal environment (`PATH`, `HOME`, `TMPDIR`, `LANG`, `MUJOCO_GL`, `PYTHONPATH` = a copy of `~/given/lib`,
`ARB_GIVEN_DIR` = a read-only copy of `~/given`) and the launcher `python3 -B -E -s -c <bootstrap> <lib> shift`
(or `bash run.sh`). Resolve given files through `os.environ.get("ARB_GIVEN_DIR", os.path.expanduser("~/given"))`,
write only under the run's HOME / TMPDIR. A program that crashes ends that shift with what it achieved; the other
shifts still run. Your submission must be final when your session ends: the submission folder is then copied out of
your sandbox and graded in a fresh sandbox that holds nothing else of yours (no other file, no process you started).

## What you are given

| path | content |
|---|---|
| `~/given/site/site_handbook.md` | the site: the pod, the tickets and their codes, the shift's rules, the parts cart and the RMA tote, the tools, closing the shift |
| `~/given/site/sop/` | the site's procedures: SOP-GPU-01 (GPU faults), SOP-NET-01 (optical links), SOP-PWR-01 (power supplies), SOP-THM-01 (fans) |
| `~/given/site/services_api.md` | the site's services through `cell.noc(...)`: NOC tickets, DCIM, BMC, PDUs, fabric manager, scheduler, DCGM |
| `~/given/site/dcim/` | the DCIM exports: floor plan, rack elevations, cable table, PDU outlet map (the drawing's values; each pod is installed within the stated tolerances) |
| `~/given/vendor/` | the server's service guide (KX-8H: views, LEDs, the GPU tray, the screws' torque and order, the GPU index to slot map, the hot-swap parts) and the transceiver / MPO guide (polarity, end-face zones, the probe's pictures, cleaning) |
| `~/given/cell/cell_manual.md` | the cell: the robot and its controllers, the navigation stack, the grip rule, the sensors and cameras, the belt tools, the mechanisms (trays, covers, screws, slots, latches, doors, cables), safety, budgets |
| `~/given/cell/cell_api.md` | the cell's programming interface |
| `~/given/cell/describe.json` | exactly what the cell's `describe()` returns |
| `~/given/cell/survey/` | survey photos: camera frames from two other pods of the same design (not among the graded ones), as the cameras deliver them |
| `~/given/robot/unitree_g1/` | the robot's MuJoCo model (Menagerie, BSD-3-Clause); the cell's modifications are in the manual |
| `~/given/lib/dc_client/` | the cell's client (`Cell`, `decode_frame`, `decode_scope`, `load_frame` for the survey photos, the launcher) |

**No practice cell.** Nothing in your session runs the cell: no practice shifts, no answer key, no checker. You prepare
the program from the documents, the survey photos and the robot's model (you may build any test rig of your own); it
then works the graded shifts as it is.

Tools: Python 3.12 at **`/opt/venv/bin/python3`** (use this path; the plain `python3` on your PATH may lack the
packages) with numpy, scipy, OpenCV (`cv2`, headless), Pillow, matplotlib and MuJoCo 3.16. No network in your session.

## How you are graded (3 hidden shifts)

Gates (multiply the score): `contract_ok` = `shift.py` is a regular file that parses and defines `solve`, imports under
the run regime above (within 120 s and 120 CPU seconds), makes at least one `step` call over the shifts, holds no
remote network socket while it runs, your own `.py` files import no networking module (`socket`, `ssl`, `urllib`,
`http`, `requests`, `asyncio`, `ctypes`, `subprocess`, ...), load no module under a computed name, execute no computed
code (`exec` / `eval` / `compile` of a non-literal) and start no other program, the submission folder holds no model
weight file (a name ending in `.pt`, `.pth`, `.onnx`, `.safetensors`, `.ckpt`, `.h5` or `.pb`) and is at most 60 MB,
and no process of the program outlives its shift. Per shift `safety` (cell_manual.md): a violation zeroes that shift.

`shift_score` = the mean over the 3 shifts (three fixed hidden shifts) of the shift's points less the incident charges,
/ 10, read from the pod's state and the NOC when the shift ends:
* the GPU ticket, 3.5: a fault that can be fixed on site: the fault gone 1.5, then a DCGM run of level 2 or more passed
  after the last physical intervention 1.0, then the node back in service 0.5; the record 0.5 (the right root-cause
  code and the parts record as the parts really moved). A fault that cannot: escalated to the vendor 1.5, with the
  evidence the SOP asks for 1.0, the node kept out of service 0.5, the record 0.5;
* the link ticket, 3.5: a fault that can be fixed on site: the fault gone 1.5, then a passed soak of at least 600 s
  after the last physical intervention 1.0, then the node back in service 0.5; the record 0.5. A damaged fibre:
  escalated to cabling 1.5, with a sharp probe picture of the damaged end face 1.0, the node kept out 0.5, the record 0.5;
* the hot-swap ticket, 1.0: the fault gone 0.6, the record 0.4;
* the mid-shift ticket, 1.0: a fan as the hot-swap ticket, a link scaled from the link ticket;
* closing, 1.0 (once some ticket earned credit): trays home and latched, covers shut 0.3; rear doors shut 0.15; every
  PDU outlet on 0.15; the failed parts in the RMA tote and nothing healthy in it, nothing left lying about 0.25; every
  ticket's status true to what was done, the previous shift's locator LED off and the untouched degraded part reported
  0.15;
* charges (each incident once): the training job crashed, a node crashed or lost its power while on, the wrong
  outlet switched, a thermal shutdown, a tray pulled from a node with AC on, bent socket pins or a damaged port 1.5 each;
  the power brake (too few healthy supplies) or two trays out at once 1.0; a hot swap over 30 s 0.5; each good part
  replaced 0.5; each node returned to service with its fault still there 1.0; each part left on the floor 0.5; a
  shift's points after the charges are at least 0.

The cell never tells your program what it has achieved; it reads the pod's state when the shift ends. A failure on the
grading side (the cell, the renderer: a camera request answered `camera_unavailable`) is not scored as your work: the
run is then an infrastructure failure.

## Notes

- Images (camera frames, probe pictures) are meant to be processed by your program; look at the ones you need.
- The drawings, the vendor's dimensions, the camera calibrations, the robot's kinematics and joint angles are exact;
  each pod's racks stand within the stated installation tolerances of the drawing, the localisation and the joint
  velocities and torque estimates are slightly noisy; everything about the night's faults must be found out in each
  shift.
- Do not modify the given files in your home directory; the grader reads its own copies.
- Environment facts (harness limits): a single tool call whose input runs above roughly 25-30k tokens is dropped by the
  harness (it is never executed and no error comes back), and a response that reaches the 64k output-token limit is
  lost and, after at most a few retries, ends the session with whatever is in `/home/user/submission/` at that moment.
  Write large files in several parts, keep every single response well below that limit, write a first complete
  submission early and refine it.
- Session time: this session is stopped after 6 hours of wall-clock time from its start. A session stopped by that limit
  is not graded at all, so a late finish scores nothing. Note the start time with `date` when you begin, keep a complete
  submission in place from early on, and end the session yourself (stop working and say that you are done) at least
  15 minutes before the limit.
