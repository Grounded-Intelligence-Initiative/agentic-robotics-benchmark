# Reviewer material (never staged for the agent)

| file | what it shows |
|---|---|
| `selection.md` | the pre-registered rule that chose the 8 hidden sites, the frozen inputs' sha256 and the result |
| `select_sites.py` | draws the candidates (`os.urandom`) and runs them through the grader's own `run_mission` with the frozen oracle |
| `map_ablation.py`, `map_ablation.json` | how much of the scouted map the carrier needs: the oracle's carrier planner (exact state and dynamics, so perfect control) with the full map, with the partition but no crates, and with no map, on development seeds |
| `reference_dev.json` | the image-only reference program (`solution/reference/`, no token) run by the grader on development seeds: the observations suffice |

All of it runs inside the task image. Development seeds are small integers (1000-9999); the hidden sites are 64-bit
draws from `os.urandom` (`tests/hidden/`).
