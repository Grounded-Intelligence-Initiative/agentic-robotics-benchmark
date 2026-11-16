# Reviewer material (never staged for the agent)

| file | what it shows |
|---|---|
| `selection.md` | the pre-registered rule that chose the 8 hidden sites, the frozen inputs' sha256 and the result |
| `select_sites.py` | draws the candidates (`os.urandom`) and runs them through the grader's own `run_mission` with the frozen oracle |
| `hollowing.py` | the kept sites' features against all candidates' and a development sample's |
| `ablation.py`, `ablation.json` | what the carrier needs to know: the oracle's carrier controller (exact dynamics, so perfect control) with the truth; without the scout's map; with the partition but no crates; with the whole map but its start taken to be the dock point, its heading taken to be the scout's, dead reckoning on the leg odometer, or its slips' turns unnoticed. Development seeds |
| `reference_dev.json` | the reference program from the robots' own observations (`solution/reference/`, no token) run by the grader on development seeds: the observations suffice |

All of it runs inside the task image. Development seeds are small integers (1000-9999); the hidden sites are 64-bit
draws from `os.urandom` (`tests/hidden/`).
