# The hidden scene set: pre-registered selection (round 2: onboard camera, turning gait)

Written and committed, with `candidates.json`, before any candidate was run (2026-11-16). It replaces the rule,
candidates and hidden set of 2026-11-12, which belonged to the ceiling-camera version of the task (world version 2);
none of those seeds is reused.

## What is frozen

The oracle and everything that decides an episode, by sha256:

| file | sha256 |
|---|---|
| `solution/oracle_policy.py` | `498da5e11b4616c0322e0c043154e01c36a8dbf6a23b06d31259ab7ce0fd1e4c` |
| `solution/oracle_run.py` | `b34617b1dd9f4c2c3e2dd1e40b6631eaeafac6d625e5f24fbbe50733e945f17d` |
| `solution/nav.py` | `74bbfa4f4e2795dc1ce0f4b6d16293a688913aa6dbe8757510ba7d532daa5b4d` |
| `solution/client.py` | `cb8b15b6d4c3da4dbf44a538c58297e1dfcae1a2354fe594ef6697cabcc6d956` |
| `solution/solve.sh` | `a8ba427a1d7d9308735d1a7985742ada112586a46b7f732894473d99b591ab3f` |
| `solution/oracle_token` | `21232e265d6f07e04de5ccb77eaef313c188d812aefee0f3907a38bca4ca6084` |
| `environment/setup/setup_runtime.py` | `8a2eca7fa10336343c110ed6436eedfeaa0e9c3a74639998c740da002dfa95ec` |
| `environment/setup/setup.sh` | `20f4ab4de3c08b4d3989ffc632e23c3145f4323357b3bc176535f0b343a2f428` |
| `environment/setup/runtime/world.py` | `9c99dd56fe6ac15ed152f2510ba7874a56af22d055ddf9a176ededd2ecb32956` |
| `environment/setup/runtime/service.py` | `3fe905a46bb8a07adf0a5ebda9e5d3c0d53917ee86e82e05e2e4c896cd09f404` |
| `environment/setup/runtime/geometry.py` | `cf4db0fac7a4131c9fe29872f4ab243cb67d00393109cd10935d18b2be3e17c4` |
| `environment/setup/runtime/scene_assets.py` | `e468fa420b51d164cfccf6a58820e4e47b7bd0c650400defd06d97c936bb6121` |
| `environment/setup/runtime/pbr_scene.py` | `4e4aee9b33f20436c451b0b4af5ae18a277448c0d7ba17477db3e9dcbbdaedc7` |
| `environment/setup/runtime/pbr_renderer.py` | `9d73722325be6264b3b4a9aa80627b4a3a43049d8a7546569ffbed9e04c43eae` |
| `environment/setup/payload/client.py` | `cb8b15b6d4c3da4dbf44a538c58297e1dfcae1a2354fe594ef6697cabcc6d956` |
| `environment/setup/payload/mcp_stdio.py` | `a71ebaef6f8641cb1c12f01c494197751cdbb514bfff3495f64d5a1f9986ff2e` |
| `environment/Dockerfile` | `42c1ebc57330d9710a5d0f17e973cd20d9fc6629a0e59b402d8dd0467aafe980` |
| `environment/requirements.txt` | `c0f139f069942f0036d73ee0165acf9c59883314616a602e2ff945557b7a0852` |
| `environment/assets.sha256` | `04896329a57cd21cb96f04e65cba7483df0ae32364f5ad26c2e0d313bd4c7941` |
| `environment/visual_assets.json` | `b1821b53c8fb7d611bca38855950b5b47458fff8d83c196074c70c47e015bfbe` |
| `environment/fetch_visual_assets.py` | `3d988a90a4cbc8959938a5053de57c176fcc8e6c4ef51a1da1a43f4639727041` |
| `tests/grader.py` | `056ecb0d0aad8d8a543188f9bbb3eff25e3456e7d9b24face53f2fdff4da5bae` |
| `tests/audit.py` | `f249cdec63abefde56deec9ba0e9350d1b9049d8f654642f22b8c57cad1a949d` |
| `tests/oracle_token.sha256` | `4d88c19dc9a5c8e288d30b09df91ac38ed031417b17bcfa919dfe9eb0595fd52` |
| `tests/review/run_episode.sh` | `2784dcd666ace581a5ace797a6fb6fb4c335901795e4ad46f3c6fc02436c1260` |
| `tests/review/select_hidden_set.py` | `e07edfd12ce13d5f5f89ef0e70afc3d6affeaff0d6f29cab20250bd1bd34f037` |

`tests/` holds byte-identical copies of the six runtime modules (`tests/review/check_task.py` checks them).

## Candidates

`candidates.json`: 40 seeds, each `os.urandom(8)` with the top bit cleared, in the order drawn.

## The rule

1. Build the task image from `environment/` of this commit.
2. Run every candidate once, in its own container from that image (no network, 2 CPUs, 4 GB), exactly as the engine
   runs an episode: `tests/review/run_episode.sh` runs `setup_runtime.py` with that one seed as the hidden set, the
   oracle (`solution/oracle_run.py`, with the oracle token) as the agent account through the robot socket, then the
   grader on the ledger the service wrote, with an oracle trajectory (the separate verifier's inputs).
   `tests/review/select_hidden_set.py` drives this and records one row per candidate in `selection.jsonl`.
3. Keep, in drawn order, the first 24 candidates whose verdict is reward 1.0 with all six delivery points reached
   and the privileged channel accepted (the token's hash matching `tests/oracle_token.sha256`, and the grader's
   replay agreeing with the service, are part of that verdict; a disagreement is a task error and the candidate is
   dropped). Every other result stays in `selection.jsonl`, with the reason.
4. `tests/hidden_scenes.json` lists the kept seeds in drawn order; `environment/setup/scenes.json` is the same file.
   Setup draws one of them per episode (`secrets.randbelow`).
5. Hollowing check: the kept scenes' features (`Scene.features()` and the oracle's presses) against all candidates'.
   A kept set that differs systematically from the candidates (narrower draws, easier layouts) is reported here.
6. Anchor: the oracle's delivered fraction on the kept set, which is 1.0 by construction; `full_at` 0.98, since
   every kept episode scores exactly what its selection run scored (the episode is a deterministic function of the
   seed and the requests). `ale validate` then runs the oracle again on a scene drawn from the set, through the
   engine, in the separate-verifier layout.

The oracle is privileged (ground truth through the token), so which candidates pass does not depend on how hard a
scene is to see or to map. The image-only reference (`solution/reference_policy.py`) is not part of the rule; its
results on development seeds are in `../README.md`.

## Result

(Appended after the run, below this line.)

Run 2026-11-16 (07:16 UTC) on a 48-core x86-64 machine, 20 containers at a time, from commit `690f0e3` (this rule's
commit, exported with `git archive`; the 26 frozen hashes above were checked on the exported tree before the run),
with the task image built from its `environment/` (the Dockerfile and its inputs have not changed since the image of
2026-11-12).

- All 40 candidates were run once. Every one scored reward 1.0 with all six delivery points reached, the privileged
  channel accepted, and the grader's replay agreeing with the service. Kept: candidates 0 to 23 in drawn order;
  candidates 24 to 39 were run but are not needed (the rule keeps the first 24). Nothing was dropped.
- On the kept set the oracle used 159 to 313 presses (median 228) of the 500, at most 319 of the 660 requests, no
  camera frame and no wall contact, in 1.4 to 8.9 s of wall-clock time per episode.
- Anchor: delivered fraction 1.0 on the kept set (24 of 24 at 6/6); `full_at` 0.98.

Hollowing check (kept 24 against all 40 candidates; `steps` is the oracle's presses; speeds are the commanded ones,
of which a press covers 0.175 s worth):

```
feature           kept mean   all mean  kept min  kept max
heading_deg         192.829    171.005     2.794   332.461
loaded_turn_deg      15.792      4.525  -169.814   176.840
forward_speed         1.341      1.335     1.227     1.482
backward_ratio        0.652      0.648     0.552     0.742
left_rate_deg        93.898     92.604    70.057   120.831
right_rate_deg      100.487     97.351    73.031   124.025
load                  0.680      0.678     0.611     0.748
wall_x0              -1.238     -1.224    -1.498    -0.916
wall_x1               1.161      1.182     0.904     1.499
narrow0               0.671      0.670     0.627     0.718
wide0                 1.103      1.127     1.018     1.235
narrow1               0.671      0.668     0.621     0.716
wide1                 1.112      1.110     1.002     1.248
long_legs             2.792      2.875     2.000     5.000
straight_path_m      26.288     26.462    20.226    33.711
steps               232.750    234.075   159.000   313.000
```

The kept scenes cover the drawn ranges; no feature is narrowed (all 40 passed, so the kept set is simply the first
24 draws).
