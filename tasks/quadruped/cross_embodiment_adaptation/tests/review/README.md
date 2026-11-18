# V5 author evidence

On all eight unchanged official seeds, the scene had one of two physical
aisles open (three south, five north). Each comparison keeps the site, native
Go1 and joint controller fixed. The camera arm surveyed RGB and inferred both
the aisle and the dock. Cue-blind arms ran the exact same survey movements,
discarded all its images, and used a fixed aisle and the same nominal dock.
The privileged arm used the true open aisle, dock center and pose, but sent
the identical twelve-angle commands through `World.act`.

| Arm | Switched | Entered aisle | First aisle correct | Crossed island | Safe docks |
|---|---:|---:|---:|---:|---:|
| Privileged feasibility | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| Scout RGB + public target feedback | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| Cue-blind, always south | 8/8 | 8/8 | 3/8 | 3/8 | 1/8 |
| Cue-blind, always north | 8/8 | 8/8 | 5/8 | 5/8 | 5/8 |

All eight wrong-aisle runs walked 2.74 to 3.11 m before physical contact stopped
them; all eight correctly chosen blind arms crossed, so the route effect does
not depend on the less precise blind dock estimate (two missed docks).
The RGB arm traveled 5.41 to 5.84 m, using 811 to 881 target actions; all four feet
were seen lifting and contacting the ground on every mission. No arm fell.
See [raw outcomes](v5-evidence/results.jsonl), sampled target paths for sites
0 and 1, and [a real UAV launch frame](v5-evidence/site0-launch.png) and
[end-of-survey frame](v5-evidence/site0-survey-end.png). These are author-only,
including the privileged and scripted controllers.

The verifier reports `missions_corridor_entered`, `missions_first_aisle_correct`,
`missions_fell_before_aisle`, total falls, partition crossings and forward
progress, plus per-site equivalents. First aisle is latched at the first
physical crossing of x = 1.10 m. Interpret route accuracy as correct / entered;
an unentered aisle is not a wrong choice. Only safe docks affect reward.

## Reproduction

Run the following in the task's pinned image on the provisioned CPU host,
with the task mounted read-only and a fresh writable evidence directory:

```bash
/opt/venv/bin/python3 -B tests/review/test_v5_contract.py
/opt/venv/bin/python3 -B tests/review/v5_controls.py /out/fresh
bash tests/review/test_v5_isolation.sh  # root, disposable task container
```

Author runs: `koa04:/home/wang100/projects/ale-hardening-20261118-checks/cross/final/evidence`,
cached `ale-cross-maintainer-env:20261116` (419e6251d06b, MuJoCo 3.16.0),
Docker runc, no network or GPU devices, 2 CPU cap, 4 GiB. The isolation
smoke applied v5's asset directory modes to that existing image, then tested
the actual setup and service as root with requests/file checks as the
participant UID. It was not a full pinned ALE validation; that and blind Sol
are owner-run. Previous planar-task receipts are historical; see
[v4 notes](v4_README.md).

## Scope

This is transfer of route evidence across sensor and control embodiments in
simulation. The UAV is a fixed-altitude kinematic camera platform, the Go1
uses native free-base rigid-body dynamics and position servos on a flat floor,
and local tracking/inertial sensing are idealized. This is not a hardware
digital twin or proof of a general Go1 gait on unknown terrain. Zero-shot
means a frozen submission, no participant target commissioning and no
retries. The installed target asset and runtime are root-only; the open-source
nominal model and analytical knowledge are not intrinsically secret. An audit
of the participant trajectory is still needed for the no-offline-rollout and
no-pretrained-gait rules. Author feasibility runs are not zero-shot model
evidence.
