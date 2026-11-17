# PRIVILEGED v3 Feasibility Reference

`solve.sh` runs `oracle_run.py`. This is deliberately privileged author code,
not an image-only policy and not evidence of model difficulty. It reads hidden
scene metadata and exact dynamic state from an oracle-only mirror. The user
expressly authorized this distinction for the v3 feasibility proof.

The pinned ALE engine uploads `solution/` only when the selected harness is
`oracle` (`episode_protocol.py`, `_run_agent`/`_upload_oracle`). None of this
directory is copied by participant setup or the Dockerfile. No privileged
operation was added to the public socket/MCP protocol. The service accepts only
observe, act (with optional frames), and advance. The engine-recorded oracle
trajectory is the existing separate-verifier audit exemption, not an agent
environment variable or a task-provided identity claim.

An exact SHA-256 of the first ordinary RGB observation selects a hidden seed from
`initial_fingerprints.json`. Source hashes must also match. Unknown images fail
closed; there is no nearest-image guess. The bundled `private/` modules mirror
the live dynamics, and `privileged.py` plans through the wide openings, predicts
actuator lag and coasting on separate MuJoCo data, and stops at each pose. All real
actions go through the same budgeted socket operations as the task; no pose,
velocity, filter, budget or completion state is changed by the controller.

## Evidence and Reproduction

Run `tests/review/privileged_reference.py --all-hidden --output <new-directory>`
in the task image on remote CPU. It runs all 24 fixed seeds in registered order
and writes every ledger, metric, failure, replay result and source digest. This
is offline feasibility; `tests/review/run_episode.sh` additionally exercises the
actual root-owned service, unprivileged oracle process and separate replay.

`tests/review/build_oracle_fingerprints.py --output <new-directory>` renders all
24 initial frames without success filtering. Its JSON belongs only in this
oracle directory, never in participant payload. Runtime/private/test copy parity
must hold after every dynamics or renderer change.

## Historical v2 Policy

`policy.py` and `coordinates.py` are retained as historical v2 image-only code.
They require the removed calibrated ceiling view and fixed-heading translation
model; they are obsolete for v3 and are not invoked by `solve.sh`. The v2 24/24,
development and baseline measurements in `tests/review/hidden_set/selection.jsonl`
and `baselines.jsonl` do not measure v3. Do not cite them as v3 feasibility or
difficulty evidence. The hidden seed files are frozen and must not be reselected.

The Go1 meshes and pinned visual assets are preserved. Dynamics are a force-driven
planar base with yaw, inertia and slip, not articulated locomotion or actual gait.
