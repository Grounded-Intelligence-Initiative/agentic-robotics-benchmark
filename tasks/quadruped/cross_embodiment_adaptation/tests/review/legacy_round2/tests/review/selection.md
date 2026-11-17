# The hidden sites: pre-registered selection rule

Registered 2026-11-16 for the second maintainers' round (no global coordinates: body cameras, an odometer that slips,
the carrier finds itself in the scout's map), committed together with the candidate list and the frozen code before
any candidate was run. The first round's sites (2026-11-12) belong to another world and are gone with it.

1. **Candidates.** 24 site seeds, 64-bit integers from `os.urandom(8)` (little-endian), in drawn order:
   `tests/hidden/candidates.json` (`select_sites.py draw`). None is a development seed; the commissioning service in
   the agent's sandbox draws its own random seeds.
2. **Cells.** A: the middle of the wide opening lies within 2.25 m, along the partition, of the dock (the scout comes
   on it early); B: farther (the scout has to survey the partition to find it). 2.25 m is the median on development
   seeds 7000-7399. Four sites per cell, eight in all.
3. **Run.** Each candidate once, in drawn order, exactly as the grader runs a mission: `select_sites.py run` calls the
   grader's own `run_mission` (the graded robot service, the program as the agent account under the no-network
   launcher, the same budgets) with the frozen privileged oracle (`solution/oracle/`). A candidate whose cell is
   already full is recorded and not run.
4. **Keep** the first four candidates per cell whose mission the oracle completes (the carrier reaches the pad). A
   candidate that fails, or whose run the robot service itself fails, is dropped and recorded with its reason. If a
   cell is not full after the 24 candidates, 8 more are drawn the same way and the run continues; that would be
   recorded here.
5. **Hollowing check.** The kept sites' features (partition offset, the wide opening's distance from the dock and its
   width, crate count, carrier size and mass, the carrier's start offset from the dock, the two bodies' headings
   apart, the floor camera's span, the oracle's carrier presses) against all candidates' and a 200-seed development
   sample's (`hollowing.py`).
6. **Anchor.** One full verify of the oracle submission on the kept sites in a fresh container laid out like the
   separate verifier; the anchor is its success rate. `full_at` is 0.98 when every kept site's outcome and carrier
   press count equal its selection run (the world's slips are drawn from the site's seed and the press count, and the
   oracle is deterministic).

Frozen inputs (sha256), as committed with this rule:

| file | sha256 |
|---|---|
| `tests/runtime/geometry.py` | `cf4db0fac7a4131c9fe29872f4ab243cb67d00393109cd10935d18b2be3e17c4` |
| `tests/runtime/pbr_renderer.py` | `9d73722325be6264b3b4a9aa80627b4a3a43049d8a7546569ffbed9e04c43eae` |
| `tests/runtime/pbr_scene.py` | `c66f525a7f201c512c473c9f8bfe43f37d3327a6aaa0a53f490efd153faac085` |
| `tests/runtime/scene_assets.py` | `9da6363a656ee172c8748ae1d7af188183271583bce4606b0758845fbe8f2790` |
| `tests/runtime/service.py` | `b1683726f465ee91c77308b291e938d3b29052ba358bf5eeaeb2f9368fed7865` |
| `tests/runtime/world.py` | `227d40314962e6a7e10bdee1af39849e028bee4934d2772b497fdf386ded52aa` |
| `tests/grader.py` | `4106784d5bd82c50185ece40732254f488f1136445a5e3a1b260e91cda7518c3` |
| `tests/nonet.py` | `4c588e267f937023c101ae78ac384c98974788ff5f05dc44dbefa21882bc5baf` |
| `solution/oracle/oracle.py` | `34e71017e1a0d05fb1f45239f0d8065012292e82183067688195f6615eaba05a` |
| `solution/oracle/run.sh` | `d54a62aea53c2395148dc6226a728f1306d54ad25646c8fcc33d42942ae2967f` |
| `solution/oracle/client.py` | `630abc48ee7a7f90266a4b4a9f5aea105b985b4117294d58e80d41cca89ac3a3` |
| `tests/hidden/oracle_token.sha256` | `8c54a6cadfd0a4098345e6173287ecc29c2106ffc59461c98ee063d1752414b7` |
| `tests/grader_config.json` | `d3c01e631a468a9ca4ab6a7f7b75846991c78c00742ab4c2b6df25e4058be87c` |

## Result

Run 2026-11-16 (`select_sites.py run`, task image, verifier layout, the frozen inputs above). Seventeen candidates were
looked at in drawn order; the eight that ran all arrived, so the oracle dropped none. Cell A was full after orders
0-3; the nine later candidates of cell A were recorded and not run; cell B filled at order 16. Kept
(`tests/hidden/sites.json`): cell A orders 0, 1, 2, 3; cell B orders 4, 10, 11, 16.

| order | cell | run | oracle | carrier presses | kept |
|---|---|---|---|---|---|
| 0 | A | yes | arrived | 67 | yes |
| 1 | A | yes | arrived | 76 | yes |
| 2 | A | yes | arrived | 90 | yes |
| 3 | A | yes | arrived | 89 | yes |
| 4 | B | yes | arrived | 110 | yes |
| 5 | A | no (cell already full) | - | - | no |
| 6 | A | no (cell already full) | - | - | no |
| 7 | A | no (cell already full) | - | - | no |
| 8 | A | no (cell already full) | - | - | no |
| 9 | A | no (cell already full) | - | - | no |
| 10 | B | yes | arrived | 103 | yes |
| 11 | B | yes | arrived | 81 | yes |
| 12 | A | no (cell already full) | - | - | no |
| 13 | A | no (cell already full) | - | - | no |
| 14 | A | no (cell already full) | - | - | no |
| 15 | A | no (cell already full) | - | - | no |
| 16 | B | yes | arrived | 101 | yes |

Hollowing check (`hollowing.py`; mean [min, max]; `heading_apart` in degrees):

| set | n | wall_x | wide_to_dock | wide_width | crates | carrier_side | carrier_mass | start_offset | heading_apart | floor_span | oracle carrier presses |
|---|---|---|---|---|---|---|---|---|---|---|---|
| kept (hidden) | 8 | -0.15 [-0.43, -0.02] | 1.95 [0.30, 3.81] | 1.58 [1.52, 1.63] | 2.75 [2.00, 3.00] | 0.81 [0.79, 0.83] | 6.50 [5.48, 8.00] | 0.63 [0.42, 0.78] | 100.56 [16.20, 171.00] | 0.60 [0.56, 0.64] | 89.6 [67, 110] |
| all 24 candidates | 24 | -0.08 [-0.43, 0.36] | 1.51 [0.07, 4.27] | 1.57 [1.50, 1.64] | 2.71 [2.00, 4.00] | 0.81 [0.77, 0.86] | 6.45 [5.15, 8.00] | 0.61 [0.29, 0.78] | 95.49 [4.60, 176.20] | 0.60 [0.56, 0.64] | - |
| development 7000-7199 | 200 | -0.04 [-0.45, 0.45] | 2.21 [0.01, 4.90] | 1.57 [1.50, 1.65] | 2.58 [2.00, 4.00] | 0.81 [0.76, 0.86] | 6.52 [5.09, 8.03] | 0.54 [0.07, 0.80] | 89.75 [1.60, 178.40] | 0.60 [0.56, 0.64] | 87.0 [51, 142] |

Cells among the 24 candidates: A 17, B 7; among the development sample: A 102, B 98. The draw happened to hold many more
A than B candidates (17 to 7; the development sample splits 102 to 98), so cell B took until order 16; nothing was
re-drawn for that, as the rule says. By chance none of the kept sites has four crates and their partitions all lie on
the dock's side of the hall's middle; the carrier's sizes span 0.79-0.83 m of the 0.76-0.86 m range.

Anchor: the oracle's full verify on the kept sites in a fresh container (verifier layout): 8 of 8, every site's carrier
press count equal to its selection run (67, 76, 90, 89, 110, 103, 81, 101), so `tests/anchor.json` is 1.0 with
`full_at` 0.98.
