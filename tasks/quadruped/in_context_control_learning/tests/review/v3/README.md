# V3 Evidence and Handoff

Implementation baseline: `c974197` with maintainer fixes. All changes are inside
this task. Task identity, the 24 hidden seeds and their order, asset manifests,
real Go1 meshes, secure grader kit, separate verifier and 3-hour agent timeout
are unchanged. No commits or pushes were made by the implementation worker.

## Design

The onboard 384 x 384 perspective camera replaces the calibrated ceiling camera.
Four unlabeled, episode-fixed buttons select forward, left yaw, right yaw or
braking. MuJoCo integrates bounded body-frame force and torque with drive/yaw
filter history, lateral slip, painted terrain bands with boundary hysteresis,
and increased mass/inertia at the third loading stop. The abstraction is a
stabilized planar base, not articulated gait. Six ordered docks require centre,
arrow heading and a continuous stopped dwell. Partial credit is exactly docks/6.
The 500-press/560-request budget remains; durations are 1-120 physics ticks.

`instruction.md` describes all these mechanics. The only extra public status is
`loaded`. No hidden state endpoint exists. `state_fingerprint()` hashes the full
MuJoCo integration state, model mass/inertia/poses and task/filter/hysteresis
memory into root-only event/final ledger fields. Observation/rendering does not
change that fingerprint. The independent security reviewer owns the audit,
grader and security-test edits; the implementation worker did not edit them.

## Measured Results

Host: `koa04`; image `ale-icl-maintainer-env:20261116`, image ID
`sha256:33860ab2d415874bfd4205d7133a070805b26c19321595f1dbd7a4591184c31e`.
Every experiment used CPU-only Docker, 2 CPUs, 4 GiB, no network, software rendering.

- `summary.json`: all 24 fixed hidden seeds, original order, no filtering: 6/6
  each; exact replay; zero contacts; 159-273 presses (median 202.5), at most 279
  requests. Total 350.6 s, individual 11.7-19.1 s. This is PRIVILEGED offline
  feasibility, not image-only or frontier evidence. `receipt.json` binds the
  world, service, grader, seed list and reference sources to the run.
- `975827769149366836.verdict.json`, `.oracle.txt`, `.time`: the first registered
  hidden seed additionally ran through the real rendered root-owned service,
  the current `solution/oracle_run.py` as unprivileged `user`, and ledger replay.
  Reward 1.0, 6/6, 159 presses, 165 requests, 0 contacts; oracle wall time 106.5 s.
  This task-level harness is not a substitute for actual pinned ALE validation.
- `contracts-12-final.log`: 12 dynamics/precision/load/copy/API tests and the
  independent reviewer's 17 audit/replay tests passed. Tests cover force/lag
  history, nonholonomic motion, terrain hysteresis, non-scalar load transients,
  pose/speed/dwell precision, docking hysteresis, non-farmable delivery credit,
  no privileged operations, frozen observations and full-state fingerprints.
- `dock-1.png`: native onboard view replayed from the service ledger. Eight
  initial/docked/loaded views were rendered in the remote `views/` directory;
  every event fingerprint and the final fingerprint still matched exactly.

Raw evidence, including every ledger and result (not just successful summaries):

`koa04:/home/wang100/projects/ale-validator-hardening-20261116/evidence/icl-worker-20261117T0040/`

Subdirectories: `hidden24-initial/`, `dev3001/`, `fingerprints/`, `live-first/`,
and `views/`. `dev3001` was the first development smoke (6/6, 178 presses).
Two initial launch attempts failed before world construction because the output
bind mount was not writable under Docker user namespace remapping. They were
infrastructure errors, not dropped feasibility outcomes. The full cohort's SSH
attachment later disconnected; all 24 results and the completed passing summary
were recovered from the still-running container's durable output directory.

## Oracle Boundary

The current reference is expressly PRIVILEGED. ALE stages `solution/` only for
the engine's oracle harness. That folder alone contains the seed/image manifest
and private dynamics mirror. The setup stage and Dockerfile do not copy it to
participants. An exact first-image digest selects a seed; the mirror then drives
only normal service requests within the same budgets. All three runtime copies
(`environment/setup/runtime/`, `tests/`, `solution/private/`) match byte for byte.
The manifest and feasibility receipt match the frozen sources, including world
SHA-256 `ffe7b9323170f10f119da0179eda13bdc4a021d9cd2ad181a19b69615b9f0e13`.

`solution/policy.py` and `coordinates.py` are obsolete v2 image-only code, retained
for history and not invoked. The old v2 selection/baseline measurements are
explicitly archived. `tests/anchor.json` now cites only this v3 cohort, value 1,
full_at 1, preserving meaningful exact partial-delivery scoring.

## Exact Remote Commands

After source-only staging to the isolated `icl/` directory, on **koa04**:

```bash
ROOT=/home/wang100/projects/ale-validator-hardening-20261116
IMAGE=ale-icl-maintainer-env:20261116
OUT=$(mktemp -d "$ROOT/evidence/icl-worker-recheck.XXXXXX")
chmod 1777 "$OUT"
ARGS=(--rm --user 0 --cpus 2 --memory 4g --network none
  -e MUJOCO_GL=osmesa -e LIBGL_ALWAYS_SOFTWARE=1
  -e LP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=1
  -v "$ROOT/icl:/task:ro" -v "$OUT:/out")
docker run "${ARGS[@]}" --entrypoint /opt/venv/bin/python3 "$IMAGE" /task/tests/review/check_task.py
docker run "${ARGS[@]}" --entrypoint /opt/venv/bin/python3 "$IMAGE" /task/tests/review/privileged_reference.py --all-hidden --output /out/hidden24
docker run "${ARGS[@]}" --entrypoint bash "$IMAGE" /task/tests/review/run_episode.sh 975827769149366836 /out
docker run "${ARGS[@]}" --entrypoint /opt/venv/bin/python3 "$IMAGE" /task/tests/review/render_ledger.py --ledger /out/975827769149366836.ledger.json --output /out/views
```

Approximate runtimes on this host: contracts 10-15 s, full headless cohort 6 min,
one rendered service oracle about 2 min. The output directory must be fresh and
writable by the remapped container UID; never loosen permissions on source or
private service directories. No GPU or credentials are used or staged.

To regenerate author-only image keys after an intentional source/renderer change:

```bash
docker run "${ARGS[@]}" --entrypoint /opt/venv/bin/python3 "$IMAGE" /task/tests/review/build_oracle_fingerprints.py --output /out/fingerprints
```

Review the output and synchronize its JSON only into `solution/`, never setup.
Do not change hidden seeds or retain only successful episodes. Re-run the entire
fixed cohort after a dynamics change; do not keep this anchor as stale evidence.

## Remaining Gates

The parent owns actual pinned ALE oracle/nop validation, independent security
review and the global report. A fresh direct-MCP-only frontier run is still
required: expired OAuth prevented one during implementation. No v3 difficulty
or score-below-0.5 claim is made. The exact image-key manifest was verified on
the pinned renderer/image on koa04; a different software rendering stack must
regenerate all keys (unknown images fail closed). There is no working v3
image-only scripted solver, and no claim of articulated gait or hardware fidelity.
