# Build the MuJoCo twin of a photographed board of household things

A service-robot team will send an arm carrying a four-digit robot hand to a client's site to clear ten household
things from a wooden board laid on a table. The site cannot be visited again; one colour photograph from the team's
own calibrated survey camera is all there is. You are the team's simulation engineer.

Build the MuJoCo twin of the board and the ten things from the photo: one static support (the board) and ten named
free bodies, each with convex collision geometry, a plausible mass and friction. The team rehearses picks on that
twin: every scored request of the pick sheet is planned on your twin with the public pick instrument, and the planned
pick is then executed in the real scene. Your twin is assessed by what those executions achieve, with protective stops
charged: a pick your twin makes possible but the real board does not allow can cost more than a pick your twin
declines.

## What you have (`/home/user/`)

| path | content |
|---|---|
| `photo.png` | the photo, 1096 x 852 colour |
| `camera_sheet.json` | the survey camera's calibration: focal lengths and principal point in pixels, the pixel convention, no distortion recorded; pose and depth not recorded |
| `site_brief.md` | the job: the ten things, the board, what the twin is used for |
| `twin_contract.md` | the technical contract: files and caps, frames, the support and the things, `objects.json`, the allowed MJCF, the standard simulation, the pick rules, and every step, outcome and number of the assessment |
| `hand_card.md` | the hand and the pick instrument: the pad volumes, the hook bar, the gaps, clearances and margins, the lift trial |
| `pick_sheet.json` | every request the team will rehearse (342: a thing, a rule, a direction or a side); which are scored, and which of those are credited, is not said |
| `hand_instrument.py`, `instrument_config.json` | the pick instrument the team plans and executes with (the same code as the grader's) and its numbers |
| `check_twin.py` | runs the validity gate on your submission (the same code as the grader's) and prints PASS / FAIL with every reason; `--settle` runs the standard settle; `--rehearse` plans and executes the pick sheet on your own twin; `--overlay out.png` draws your twin's outlines through `photo_cam` on the photo; `--json`. It reads nothing about the real scene and scores nothing |
| `twin_gate.py`, `twin_sim.py`, `twin_sim_config.json`, `twin_camera.py` | the gate, the standard simulation with the support's frames and the convex pieces, and the camera conventions, as importable modules |
| `submission/` | where your twin goes (empty) |

There is no checker of the real scene, no practice scene and no way to query the truth.

## What you deliver (`/home/user/submission/`)

- `scene.xml`: the twin in MJCF, world z up, metres: one static body `support` whose collision geoms are boxes (its
  top is the board's top), and the things, each a top-level body with a free joint and
  1 to 96 convex collision geoms (a mesh collides as its convex hull,
  so a hollow or a loop must be made of several pieces); `objects.json` maps 1 to 10 of
  them to the ten things' keys, and a thing it does not map counts as absent (its requests are declined); a camera
  `photo_cam` is optional.
- `assets/`: the meshes `scene.xml` references (relative paths).
- `objects.json`: `{"format": "hand-twin-objects/1", "bodies": {"<body name>": "<thing key>", ...}}`.

Files that `scene.xml` does not reference are ignored but count toward the caps (5000 entries,
300 MB): keep renders, caches and scratch work outside `submission/`. `twin_contract.md` has every rule.

## How it is assessed

`twin_score = G1 x G0 x Use`, in [0, 1]. Full definitions are in `twin_contract.md`, section 8.

- **G1 submission integrity**: nothing in `submission/` written after your session ends; a real folder within the
  folder caps.
- **G0 validity**: `twin_gate.py` passes (what `check_twin.py` runs) and the standard settle does not blow up. A thing
  that fails a plausibility rule (density 10-3000 kg/m^3,
  friction 0.1-1.5, inertia, centre of mass) is excluded and counts as absent, without failing the
  twin.
- **Use**: your twin is settled, laid on the real board's plane and registered to the real scene (one yaw, a shift in
  the plane and one overall scale clamped to [0.8, 1.25],
  fitted on the things). Then every scored request is planned on the registered twin and executed in the real
  scene at 9 placements of the hand (the centre and the corners of +-1.5 mm).
  A declined request scores 0 and a miss 0; a done request scores 1 if it is credited (a careful twin can do it)
  and 0 if it is not; a protective stop is charged 2 times its share on every scored request. The scored
  requests that are not credited count only for their stops: the team's reference twin does not do them at any
  placement, a pick done on one of them earns nothing, and nothing earns more on them than declining them. Every
  unit of a thing and a rule that has a credited request weighs the same (a unit without one carries only its
  stops); no thing weighs more than the larger of 15 % and 1 / n of the total weight, n being the number of things that have a unit with a credited request. Fewer than 4 things that
  register within 25 mm give Use = 0.

## Notes

- Machine: 6 h, 8 CPUs, 32 GB of memory, no GPU, no network. No
  pretrained vision models are installed.
- Tools: Python 3.12 (`/opt/venv`, on PATH) with numpy 2.5.4, scipy 1.18.1, OpenCV (`cv2`,
  headless), scikit-image, Pillow, trimesh, manifold3d, CoACD, Shapely and MuJoCo 3.16.0 (CPU rendering
  through OSMesa).
- Do not modify the given files in your home directory; the grader uses its own copies.
- Harness limits: a single tool call whose input is above roughly 25-30k tokens is dropped (it never runs and no
  error comes back); a response that reaches the 64k output-token limit is lost and, after a few retries, ends the
  session; a session whose output stream stays silent for 600 s is ended. Keep each call and response well below
  these limits, write large files in parts, run long jobs in the background with their output in a log file, and
  write a first complete submission early, then refine it, running `check_twin.py` after each change.
