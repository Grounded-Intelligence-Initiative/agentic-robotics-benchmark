# Scout-assisted vacuum in an unfamiliar home

Write a controller for a ground vacuum and a faster indoor scout. Read a physical work card, clean two spills in each requested room and one under the hall carton, deliver the carton to the specified room mat and then to the specified hall mat, and return the vacuum to its dock. Leave the treatment powder in both unrequested rooms in place and protect the woven runners. The floor plan, furniture, sight lines, camera orientations, mat positions and work order vary. All stay fixed within a home.

Submit an executable `/home/user/submission/run.sh` and any helpers there. It must print `READY` and flush stdout, then process one JSON object per line. Reply to `{"type":"reset"}` and `{"type":"close"}` with `{"ok":true}`; forget all episode state on reset. For `{"type":"act","t":0,"obs":{...}}` return e.g. `{"action":["E","N",2,true]}`. Log only to stderr. Python, Pillow, NumPy and MuJoCo are installed in `/opt/venv`. You may coordinate both robots in one program and build your own map.

**What you get:** `instruction`; `ground` and `air` (base64 PNG, RGB, 84 by 84); `moved` (the counts of cell moves actually completed on each robot's previous command, initially zero); `left` (shared ticks remaining); and `flight` (drone ticks remaining). The two robots start together at the dock. There are no coordinates, room boundaries, top-down house map or online scores on the wire.

**Cameras and orientation.** Rectified downward RGB, 3 x 3 cells for the vacuum and 7 x 7 for the scout. Each camera has its own unknown, fixed 0/90/180/270-degree mounting orientation. The small yellow dash on the teal starting dock marks **geographic north**; both cameras see it on the first frame. Black is unobserved; walls occlude both robots, furniture occludes the vacuum. The robots themselves are masked.

**Visual work order.** The dispatch cabinet is at the opposite end of the entrance hall from the dock. Its upward-facing card can be seen only by the scout. The rooms and destinations are never included in the text observation. When viewed north-up the lime-bordered card has four swatches:

| Top left and top right | The two rooms to clean, in either order. Orange `F68C18` = oak, cyan `00D9EC` = sage, purple-blue `7042FA` = slate, pink `F62AC4` = rose. |
|---|---|
| Bottom left | First deliver the carton to the jade `36B44A` or violet `9B54E0` room mat. |
| Bottom right | Then park the carton on the red `CB4D5D` or blue `397ECB` hall mat. |

`/home/user/practice/dispatch-example.png` shows a real public scout frame, the enlarged north-up card and its interpretation. The colors are symbols on a printed card, not sampled floor colors.

Floor colors (approximate sRGB): hall `C5BBA5`, oak `BC936B`, sage `98AD95`, slate `8B9EAC`, rose `C2A2A4`. Walls have grey caps `656577`; furniture has varied materials. Dust and treatment powder both look brown (`684127`); the order determines which may be removed. Carton `E4AC29`, dock `24A99C`, runner `945078` with light stripes. Natural textures, lighting and occlusion affect the pixels. Public example frames are in `/home/user/practice/cameras.jsonl`. The engine renders 3D meshes with discrete motion; it does not test flight stabilization.

**Actions:** `[vacuum_direction, scout_direction, scout_stride, suction_on]`. For each robot independently `N/E/S/W` mean **up/right/down/left in its own image**; `stay` is also valid. Stride is 0, 1 or 2, and suction is a JSON boolean. Each request advances **one shared tick**. The ground robot attempts one cell move. Entering the carton pushes it one cell in the same direction if the space beyond is clear; it cannot pull. The brush acts on the vacuum's final cell even when stationary: it removes powder or dust permanently and scuffs delicate runners. Crossing with the brush off is safe. The scout can pass above the carton and furniture but not through walls. Its first 36 ticks elapse even when hovering; afterward `air` is null. Both act concurrently; each home allows 200 shared ticks, including blocked moves.

**Scoring:** 45% for the mean fraction cleaned across the two requested rooms and the hall, multiplied by the fraction of four treatment patches preserved; 12.5% for reaching the selected room mat with the carton; 12.5% for leaving it on the selected hall mat *after* that delivery; 30% for completing all work, docking and leaving runners and treatment intact. No check-in action is required at either mat. Only physical outcomes count, not whether the solver claims it read the card. Full completion ends the episode. An aborted episode receives zero. The verifier averages six private homes.
