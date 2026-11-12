# The hidden sites: pre-registered selection rule

Registered 2026-11-12, committed together with the candidate list and the frozen code before any candidate was run.

1. **Candidates.** 24 site seeds, 64-bit integers from `os.urandom(8)` (little-endian), in drawn order:
   `tests/hidden/candidates.json` (`select_sites.py draw`). None is a development seed; the commissioning service in
   the agent's sandbox draws its own random seeds.
2. **Cells.** A: the wide opening's centre at y >= 0 (the carrier's own half of the hall); B: at y < 0 (the carrier must
   cross to the far half first). Four sites per cell, eight in all.
3. **Run.** Each candidate once, in drawn order, exactly as the grader runs a mission: `select_sites.py run` calls the
   grader's own `run_mission` (the graded robot service, the program as the agent account under the no-network
   launcher, the same budgets) with the frozen privileged oracle (`solution/oracle/`). A candidate whose cell is
   already full is recorded and not run.
4. **Keep** the first four candidates per cell whose mission the oracle completes (the carrier reaches the pad). A
   candidate that fails, or whose run the robot service itself fails, is dropped and recorded with its reason. If a
   cell is not full after the 24 candidates, 8 more are drawn the same way and the run continues; that would be
   recorded here.
5. **Hollowing check.** The kept sites' features (partition x, wide opening centre and width, crate count, carrier
   size and mass, the oracle's carrier presses) against all candidates' and a 200-seed development sample's.
6. **Anchor.** One full verify of the oracle submission on the kept sites in a fresh container laid out like the
   separate verifier; the anchor is its success rate. `full_at` is 0.98 when every kept site's outcome equals its
   selection run.

Frozen inputs (sha256), as committed with this rule:

| file | sha256 |
|---|---|
| `tests/runtime/geometry.py` | `cf4db0fac7a4131c9fe29872f4ab243cb67d00393109cd10935d18b2be3e17c4` |
| `tests/runtime/pbr_renderer.py` | `9d73722325be6264b3b4a9aa80627b4a3a43049d8a7546569ffbed9e04c43eae` |
| `tests/runtime/pbr_scene.py` | `c66f525a7f201c512c473c9f8bfe43f37d3327a6aaa0a53f490efd153faac085` |
| `tests/runtime/scene_assets.py` | `9da6363a656ee172c8748ae1d7af188183271583bce4606b0758845fbe8f2790` |
| `tests/runtime/service.py` | `660e456e8b39f7d41b338cf56006b7121304faf9dd4e7543e50bd13045e55772` |
| `tests/runtime/world.py` | `21474b4540176ba35b6fee71a0d3a6e23b1381868c3116d76c2adf8a0010d2c6` |
| `tests/grader.py` | `b870fcb5dc5229ae311842a1774124a0217ae7a42992005b4f8a35c561973fe2` |
| `tests/nonet.py` | `4c588e267f937023c101ae78ac384c98974788ff5f05dc44dbefa21882bc5baf` |
| `solution/oracle/oracle.py` | `c39d4888f2cefd64bf65c3cc8fcf136f6c66fb4faba4206d502a49445266e1fa` |
| `solution/oracle/run.sh` | `d54a62aea53c2395148dc6226a728f1306d54ad25646c8fcc33d42942ae2967f` |
| `solution/oracle/client.py` | `21461db2936eec4af353480832d054ad6610e63293664c9474edca3ec53c0ccf` |
| `tests/hidden/oracle_token.sha256` | `8c54a6cadfd0a4098345e6173287ecc29c2106ffc59461c98ee063d1752414b7` |
| `tests/grader_config.json` | `114b66e5e4c015987d539d8b451aa8af2162377ae199c44e3540c91c75b5fa5c` |

## Result

RESULT_TABLE
