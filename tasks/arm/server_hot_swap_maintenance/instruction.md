# Hot-swap a storage server's failed drive and fan

You are a robotics engineer in a data centre. Two Franka Pandas stand in front of a top-loading storage server. A
work order says that one drive and one fan failed. Your job is to write the controller that does the swap: find the
failed drive and the failed fan by their amber status LEDs, take them out, bin them, and put spares in. It is graded closed loop on servers you have not seen: each has its own pose, camera mounts, latch forces and
masses. You have 3 hours of wall time (see "Session time"). Whatever is in `/home/user/submission/` when the time
ends is graded, so keep a working controller there.

`/home/user/given/manual.md` is the cell's manual: the geometry, the mechanisms, the sensors and every hidden range.
Read all of it. `/home/user/given/robot/panda/` is the robot model (MuJoCo Menagerie's Franka Emika Panda,
Apache-2.0); the cell uses its kinematics as given. Python is `/opt/venv/bin/python3` (MuJoCo 3.16, NumPy, SciPy,
OpenCV, Pillow). There is no network. Your sandbox has 8 CPUs and 16 GB.

## The job

Each episode is one work order on one server:
1. find which drive slot and which fan bay hold the failed parts (amber LED; every healthy part shows green);
2. the failed drive: press its lever's release button, draw the caddy up its rails by the lever, and before
   it leaves the rails take its weight with the other arm; carry it out and drop it in the failed-parts bin; then
   take a spare caddy from the holder, insert it in the **same** slot, swing its lever closed and press it home
   until the latch catches (the caddy seated and its lever latched);
3. swap the failed fan for the spare fan: the bay must not stay empty longer than the server's overheat window
   (20-40 s, hidden), so have the spare at hand before the old one comes out; bin the old fan;
4. when done, send `"done": true`.

The episode **ends at once and scores 0** on any of these (irreversible):
- a healthy drive unseated (pulled more than 1.8 mm out of its connector);
- a drive caddy or a fan module dropped (its centre below 0.12 m);
- a lever broken: a lever carrying its caddy alone, off the rails, breaks;
- the overheat: a fan bay without a seated fan for longer than the window;
- a robot pressing on the server (chassis or stand) with more than 60 N for 20 ms.

**Credit, all or nothing.** An episode scores 1 when, at its end, the failed slot holds a spare caddy seated and
latched, every other slot still holds its own caddy seated and latched (a released healthy lever must be latched
again), every fan bay holds a seated fan (the failed bay the spare), every removed part lies in the bin, and nothing
irreversible happened; else 0. An episode ends at the first of: `"done": true`, 6000 control steps (10 simulated
minutes), an irreversible incident, an unstable simulation (score 0). The score is the mean credit over the graded
episodes: 10 episodes on 10 hidden servers, a fixed hidden set, run in a hidden order.

## Observation and action

The controller runs at 10 Hz. The observation is a JSON object:
- `t` (the step), `unit` (the server's id), `order`: `{"text", "failed_drives", "failed_fans"}`;
- per arm, under `l` and `r`: `q`, `dq` (7 joints), `ee_pos`, `ee_quat` (wxyz) of the tool frame (the grip site
  between the fingertips, see the manual), `wrench` (force and torque at the wrist sensor, world frame, with noise
  and a per-server bias), `gripper` (the fingers' opening, m);
- per camera `K_<cam>` (intrinsics) and, when the previous action asked for it, the frame: `<cam>` an RGB image
  `{"dtype": "uint8", "shape": [H, W, 3], "b64"}` and `depth_<cam>` a depth image `{"dtype": "uint16", "shape":
  [H, W], "b64"}` in millimetres, 0 where the sensor has no reading. Cameras: `overhead` (640 x 480, fixed),
  `wrist_l`, `wrist_r` (320 x 240, on the hands). The first observation carries all three; otherwise a frame is
  empty (`[0, 0, 3]`, `[0, 0]`) unless asked for.

**No pose is given for any camera.** Each server's overhead camera and each wrist camera sit off their nominal mounts
(manual: nominal poses and the error ranges). Calibration plates (checkerboards at known places) and the arms' own
kinematics are there to identify them. The server's pose is hidden too (manual: ranges).

The action is `{"l": [...], "r": [...], "cams": mask, "done": false}` (`done` optional). Each arm's list is one of:
- **7 numbers** `[dx, dy, dz, drx, dry, drz, grip]`: the operational-space controller moves the tool frame's target
  from its current pose by (dx, dy, dz) m (clipped to 0.03 m) and turns it by the rotation vector (drx, dry, drz)
  rad (clipped to 0.2 rad), world frame. `grip` from -1 (fingers' target opening 80 mm) to +1 (closed), linear;
  the fingers squeeze with at most 35 N each and move at about 0.1 m/s;
- **14 numbers**: the same 7, then seven joint angles, the null-space posture from now on;
- **8 numbers** `[q1..q7, grip]`: a joint step (each joint's target at most 0.1 rad from its current angle).

The controller compensates the arm's own weight, not what the hand holds or pushes. `cams` is a bitmask of the frames
the next observation carries: 1 overhead, 2 left wrist, 4 right wrist. A frame is large and each costs 0.05 s of
your controller's time budget at grading.

## Development: the commissioning cell

There is no simulator in your home. The commissioning service runs the graded cell's own build on four development
servers `d1`..`d4` (each has its own hidden values, drawn from the same ranges as the graded ones; none is a graded
server). **You have 5 commissioning sessions in all**, one episode each, counted when a session opens and never
refunded:

    /opt/venv/bin/python3 ~/commission/run_session.py --status
    /opt/venv/bin/python3 ~/commission/run_session.py --unit d1 [--submission DIR] [--log FILE]

`run_session.py` starts your `run.sh` the way the grader does and drives it over the same wire. At the end it prints
the cell's message (why the episode ended: done, the step limit, or the incident that stopped the cell), the steps and
your controller's time, and writes a per-step log of the arms' readings and your actions. It never says whether the
work order was done right: there is no credit and no ground truth. Save the frames you want from inside your
controller. Everything else (perception, calibration, motion plans) you develop offline with the manual and the
robot model.

## Deliverable and grading

Put your controller in `/home/user/submission/` with a `run.sh`. Only that folder exists at grading (a separate
sandbox): copy into it everything your controller needs. At most 5000 files and 200 MiB, a real folder.

**The wire.** The grader starts `bash run.sh` in that folder, as your user, with only local (AF_UNIX) sockets and with
only `PATH`, `HOME`, `LANG` and the numerics variables set. One JSON object per line on stdin/stdout:
- the controller prints `READY` within 120 s of starting;
- `{"type": "reset"}` -> `{"ok": true}`: an episode starts;
- `{"type": "act", "t": t, "obs": {...}}` -> `{"action": {...}}`, within 30 s, one line of at most 4 MiB;
- `{"type": "close"}` -> `{"ok": true}`, then exit.

Stdout carries only these messages; log to stderr. `~/helpers/wire_skeleton.py` is optional boilerplate.

**Repeated steps.** About 1 % of steps are asked twice with the same `t`, in a hidden order: once with the real
observation, once with a copy whose arm joints are moved by up to 1 mrad (tool poses recomputed) and forces by up to
0.2 N (frames unchanged). Only the reply to the real one is applied. If fewer than 10 % of the counted pairs get
replies that differ (by more than 1e-4 in some number), the whole run scores 0; a pair where both replies command no
motion (both arms 7 numbers, the first six within 1e-9 of zero) is not counted. A controller whose replies depend on
the arms' readings (Cartesian steps computed from the observed tool pose) passes; a replayed action stream, a cache
keyed by `t`, or steps computed from images alone do not.

**Failures and budgets.** A crash, a late or oversize reply or a malformed action scores that episode 0 and grading
goes on (after a crash or a late reply the next episode restarts the controller; after 3 restarts the rest score 0).
Your controller has 9000 s of its own time over the run (the waits for READY, resets and replies, plus 0.05 s per
frame asked for); when it runs out, the rest score 0. The simulation's own time is not counted. Your controller runs
as one process for the whole run. It cannot open internet sockets; leftover processes the grader cannot stop score
the run 0.

**Numerics.** Every `/opt/venv` Python computes the same here and at grading: the image pins NumPy's BLAS kernels
and threads (`OPENBLAS_CORETYPE=Haswell`, `OPENBLAS_L2_SIZE=256`, `OPENBLAS_NUM_THREADS=1`, `OMP_NUM_THREADS=1`,
`NPY_DISABLE_CPU_FEATURES`, `OPENCV_IPP=avx2`). Do not change them.

## Session time

This session is stopped 3 hours of wall time after it started; `bash /home/user/session_time.sh` prints the
minutes used and left. Run it when you begin and before decisions that depend on the time left. Keep a complete,
runnable submission in place from early on.
