# The hidden scene set: pre-registered selection

Written and committed, with `candidates.json`, before any candidate was run (2026-11-12).

## What is frozen

The oracle and everything that decides an episode, by sha256:

| file | sha256 |
|---|---|
| `solution/policy.py` | `f510518aa5b76a75e5e01d4ccc3f380535560f64ab76e905cfc6b9fdf37608a3` |
| `solution/coordinates.py` | `5b35b3b45b8247c5239b639060d8532d3017f2a0bbe08a6e4c4f5b7f043b97ee` |
| `solution/oracle_run.py` | `ae4587d9993d05dd0dcb0991dffb21fd13e3a8ae6024ffd94fce7ddced96a969` |
| `solution/client.py` | `cb8b15b6d4c3da4dbf44a538c58297e1dfcae1a2354fe594ef6697cabcc6d956` |
| `solution/solve.sh` | `a8ba427a1d7d9308735d1a7985742ada112586a46b7f732894473d99b591ab3f` |
| `environment/setup/setup_runtime.py` | `8a2eca7fa10336343c110ed6436eedfeaa0e9c3a74639998c740da002dfa95ec` |
| `environment/setup/runtime/world.py` | `09d3c00033ddfd85a72404f5f86a11ea50fc5cbdd5799553b1c7c42f4447ae5b` |
| `environment/setup/runtime/service.py` | `d846bcf16722c20af72bc82aedff856859d355d2257bf433852a96ee3ca404ea` |
| `environment/setup/runtime/geometry.py` | `cf4db0fac7a4131c9fe29872f4ab243cb67d00393109cd10935d18b2be3e17c4` |
| `environment/setup/runtime/scene_assets.py` | `e468fa420b51d164cfccf6a58820e4e47b7bd0c650400defd06d97c936bb6121` |
| `environment/setup/runtime/pbr_scene.py` | `ffeddab7db9454e7cdc77646872acd662063111831d3e432971df8b8a5f061fe` |
| `environment/setup/runtime/pbr_renderer.py` | `9d73722325be6264b3b4a9aa80627b4a3a43049d8a7546569ffbed9e04c43eae` |
| `environment/setup/payload/client.py` | `cb8b15b6d4c3da4dbf44a538c58297e1dfcae1a2354fe594ef6697cabcc6d956` |
| `environment/setup/payload/mcp_stdio.py` | `414146999d1faa11d14e3f50217f30c59ae53cb071ea1a73fad8a1bc8854b7c1` |
| `environment/Dockerfile` | `42c1ebc57330d9710a5d0f17e973cd20d9fc6629a0e59b402d8dd0467aafe980` |
| `environment/assets.sha256` | `04896329a57cd21cb96f04e65cba7483df0ae32364f5ad26c2e0d313bd4c7941` |
| `environment/visual_assets.json` | `b1821b53c8fb7d611bca38855950b5b47458fff8d83c196074c70c47e015bfbe` |
| `environment/fetch_visual_assets.py` | `3d988a90a4cbc8959938a5053de57c176fcc8e6c4ef51a1da1a43f4639727041` |
| `tests/grader.py` | `e5937463e237392468990b47d31007f0972ff618f04cf8c2169b92645478ad5b` |
| `tests/audit.py` | `f249cdec63abefde56deec9ba0e9350d1b9049d8f654642f22b8c57cad1a949d` |
| `tests/review/run_episode.sh` | `3636417d80be0dd39760dc5522c4f6bc5ff48a16f58161a2e8e78adc3cdad42e` |
| `tests/review/select_hidden_set.py` | `f4648bd63f9fa1872246ab8c219efe55cd22a5af6c5c8b5c6d4bb870246ef75a` |

`tests/` holds byte-identical copies of the six runtime modules (`tests/review/check_task.py` checks them).

## Candidates

`candidates.json`: 40 seeds, each `os.urandom(8)` with the top bit cleared, in the order drawn.

## The rule

1. Build the task image from `environment/` of this commit.
2. Run every candidate once, in its own container from that image (no network, 2 CPUs, 4 GB), exactly as the engine
   runs an episode: `tests/review/run_episode.sh` runs `setup_runtime.py` with that one seed as the hidden set, the
   oracle (`solution/oracle_run.py`) as the agent account through the robot socket, then the grader on the ledger the
   service wrote, with an oracle trajectory (the separate verifier's inputs). `tests/review/select_hidden_set.py`
   drives this and records one row per candidate in `selection.jsonl`.
3. Keep, in drawn order, the first 24 candidates whose verdict is reward 1.0 with all six delivery points reached
   (the grader's replay agreeing with the service is part of that verdict; a disagreement is a task error and the
   candidate is dropped). Every other result stays in `selection.jsonl`, with the reason.
4. `tests/hidden_scenes.json` lists the kept seeds in drawn order; `environment/setup/scenes.json` is the same file.
   Setup draws one of them per episode (`secrets.randbelow`).
5. Hollowing check: the kept scenes' features (`Scene.features()` and the oracle's presses) against all candidates'.
   A kept set that differs systematically from the candidates (narrower draws, easier layouts) is reported here.
6. Anchor: the oracle's delivered fraction on the kept set, which is 1.0 by construction; `full_at` 0.98, since
   every kept episode scored exactly what its selection run scored. `ale validate` then runs the oracle again on a
   scene drawn from the set, through the engine, in the separate-verifier layout.

## Result

(Appended after the run, below this line.)

Run 2026-11-12 on a 48-core x86-64 machine, 20 containers at a time, from commit `e2e584e` (this rule's commit; the
frozen hashes above were checked before the run), image built from its `environment/`.

- All 40 candidates were run once. Every one scored reward 1.0 with all six delivery points reached, and in every
  one the grader's replay agreed with the service. Kept: candidates 0 to 23 in drawn order; candidates 24 to 39 were
  run but are not needed (the rule keeps the first 24). Nothing was dropped.
- On the kept set the oracle used 194 to 406 presses (median 300) of the 500, at most
  412 of the 560 requests, 13 wall contacts in all, and 36 to 157 s of wall-clock time per episode.
- Anchor: delivered fraction 1.0 on the kept set (24 of 24 at 6/6); `full_at` 0.98.

Hollowing check (kept 24 against all 40 candidates; `steps` is the oracle's presses):

```
feature           kept mean   all mean  kept min  kept max
heading_deg         167.266    176.757    29.130   356.706
loaded_turn_deg       5.636      1.379  -154.985   161.628
forward_speed         1.332      1.329     1.200     1.494
backward_ratio        0.664      0.653     0.559     0.747
left_speed            0.818      0.818     0.701     0.933
right_speed           0.845      0.830     0.724     0.938
load                  0.674      0.677     0.607     0.744
wall_x0              -1.190     -1.173    -1.472    -0.981
wall_x1               1.239      1.203     0.904     1.500
narrow0               0.672      0.673     0.623     0.718
wide0                 1.146      1.144     1.016     1.247
narrow1               0.656      0.664     0.622     0.707
wide1                 1.124      1.105     1.005     1.233
long_legs             2.875      2.600     2.000     6.000
straight_path_m      27.049     25.978    21.413    32.930
steps               298.833    287.850   194.000   406.000
```

The kept scenes cover the drawn ranges; no feature is narrowed (all 40 passed, so the kept set is simply the first
24 draws).
