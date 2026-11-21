# v4 Maintainer Integration

Base: `c974197`; local pre-merge checkpoint: `955e8b7`.
Maintainer commits reviewed: `690f0e3` and `8b7195d`.
No push is authorized. All changes are inside this task directory.

## Decisions

- Keep round2's exact hidden 24 seeds, 40 candidates and ordering. Keep its
  historical selection rows as provenance, never as v4 success evidence.
- Keep six hours (21600 seconds), 660 requests, 500 presses and 120 camera frames.
  Observe alone returns images. Act and advance return status; frames_remaining
  counts camera observations, while act's frames argument counts physics ticks.
- Keep the native force-driven planar model, fixed unknown button permutation,
  actuator history, terrain hysteresis, mass/inertia loading and stable pose docks.
  No harder geometry or shorter budget was added to obtain failure.
- Bump world/config/bridge/reference labels to v4, since maintainer round2 is v3.
- Preserve both round2 pbr_scene changes byte-for-byte: panel emission and the
  generator moved clear of the north fence. Preserve the camera near-plane fix
  via structured XML (`visual/map znear=0.0006`).
- Preserve root-only ledger state fingerprints including complete MuJoCo
  integration state, model mass/inertia, filters, terrain and docking memory,
  counters and camera consumption. Rendering does not mutate the fingerprint;
  observation changes camera accounting but not physics or docking state.
- Reject round2's public unlock/token truth path. No such operation exists in
  the active service or MCP bridge. Grader merge changes relative to the reviewed
  checkpoint are the camera metric comparison and clarification of the private
  oracle. The reviewed fail-closed audit and fingerprint checks remain intact.
- Preserve the participant payload, separate verifier, secure grader kit, pinned
  assets, real Go1 meshes and task identity. No solution metadata is staged for
  participant harnesses. The oracle uses exact first-image matching against all
  fixed seeds and a private mirror, with no fallback or environment-name trust.
- Archive the maintainer's image-only reference and token reference rather than
  claiming either works on v4. No current participant/image-only success claim.

## Regression Fix

The author cohort helper explicitly catches `(Exception, SystemExit)` from
grader replay, persists the failed result and continues the remaining seeds.
It still propagates KeyboardInterrupt. Three regression tests cover exit codes
0/1/message, a first-seed replay failure followed by all remaining fixed seeds,
and interruption behavior. The old candidate was rerun after this fix and is
recorded separately under `../v3/exitfix/`; v4 has its own fresh receipt.

## Frozen Production Digests

- world.py: `e0c8a2fec8e42eb8ac8747373ff1e59d02a54b5ae1621a887454f9254c898164`
- service.py: `1d3881e70d7cca1a7c517535a79247cce40c0b0f33e29567c5f8506f56df928b`
- pbr_scene.py: `4e4aee9b33f20436c451b0b4af5ae18a277448c0d7ba17477db3e9dcbbdaedc7`
- initial_fingerprints.json: `d2ebcf6d98bcad548378bec6777d2205ff911b9aa41bc6490e92db58a4f520d5`
- hidden_scenes.json: `e42f4ba3a8167e21525db326c94ed82cc9395e44e2ac570ef01d44ab2176e0dd`

The manifest binds all six runtime modules across runtime/tests/solution-private
and all 24 fixed seeds. The cohort receipt separately binds the controller,
cohort helper, grader and seed file. Parent owns actual ALE validation and the
global security/difficulty report; old candidate ALE results are not v4 results.
