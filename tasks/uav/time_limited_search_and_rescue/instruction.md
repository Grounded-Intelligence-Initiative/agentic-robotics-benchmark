# Civilian Rescue: Warehouse Handoff

Recover the rescue mannequin on its rigid medical litter. Deliver it intact to the **receiving station accepting this incident**. A dispatch arrow points toward that station. At receiving, a lowered shutter bears a large cross; an open doorway exposes a horizontal deck. The incident remains fixed during your sortie. You may inspect either approach, change your mind, or use any safe route.

Entry safety briefing: center on the violet apron, settle at approximately 0.71 m, cross the inspection slot facing into it, and climb only after the aircraft has cleared the frame.

**Aircraft and cameras.** `robot_act({"action":[right,forward,up,yaw_left,grip]})` accepts five numbers in [-1,1]. Each action advances 0.2 s. Maximum nominal setpoint increments are 0.32 m on each horizontal axis, 0.19 m vertically and 30 degrees in yaw. Horizontal axes use the heading at the **start** of the action; zero holds. Translation gains vary a few percent, and loaded forward travel can drift sideways by about 0.13 m per metre. Launch is at 1.30 m facing the wing; altitude limits are 0.39-4.69 m above launch.

`robot_observe()` and each action return two 256-by-256 RGB views, `frame_id`, `done` and `steps_remaining`; no pose, map or depth. Both cameras rotate with the aircraft and have 94-degree vertical field of view (focal length 119.4 pixels). Camera 0 looks down from 0.46 m forward and 0.145 m below center; right/up in the image are body right/forward. Camera 1 looks forward, pitched 6.9 degrees down, from 0.17 m forward and 0.045 m above center. An attached board can obscure the down view.

**Equipment card**

| Equipment | Physical contract |
|---|---|
| Litter and heads | Board: 1.80 by 0.44 m; violet end grips 1.62 m apart. The cyan flange under the common dark cover is 8.2 or 18.6 cm across. Small and large split collets fit the corresponding flange. The clamp preserves the actual pickup yaw. |
| Gripper | Above +0.5 closes, below -0.5 releases, otherwise holds; grip acts before movement. Mount or return a head on its own stand at 0.430 +/- 0.035 m, within 0.085 m horizontally. Latch within 0.135 m of the fitting, aircraft center 0.306 +/- 0.060 m above board center (about 0.40 m above the floor). Steady to at most 0.30 m/s and below 0.30 rad/s. A mismatch does not attach. |
| Wing | Room pitches 2.60-2.80 m; storeys 2.60 m apart, slab 0.22 m thick. Amber hoist thresholds lead to square bores of 1.08 or 2.34 m; back-wall guides are 1.40 m apart. Ordinary white doorways clear 1.48 m. The violet inspection slot is 1.02 m wide with sill/lintel at 0.43/0.89 m. The blue cargo shutter retracts on a held lift of 0.54 m above the local floor. |
| Transfer | The turning pocket is 4.20 by 4.60 m. Jambs give 1.10 m width over 0.56 m depth; lime toe strips extend 1.24 m toward their approach. The aircraft, head and entire load must clear structural surfaces. Contact ends the sortie. |
| Handoff | The green deck is 2.50 by 1.48 m. Release with board center no higher than 0.195 m above the floor, speed and yaw rate no greater than 0.34 m/s and 0.34 rad/s. For full credit the released board must physically rest entirely on the deck, center within 0.28 m of the blue target center and at most 0.095 m high, for 0.25 s. The settling limit is 0.16 for the norm of its linear and angular velocities (m/s, rad/s). |

Budget: **4,400 actions, 9,000 total calls, 30 minutes**, no reset. Observing does not advance simulation time. The public client is `/home/user/robot/client.py` with `/opt/venv/bin/python3`. Partial rescue progress is recorded; full credit is the physical handoff, not a particular sequence of visits.
