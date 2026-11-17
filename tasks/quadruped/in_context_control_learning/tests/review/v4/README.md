# Completed v4 Author Evidence

All runs used the unchanged round2 hidden 24 seeds in registered order. No seed
was dropped, substituted, reordered or selected on a v4 result. This is expressly
PRIVILEGED feasibility evidence, not image-only or frontier-model performance.

| Check | Result |
| --- | --- |
| Fixed native CPU cohort | 24/24 at 6/6; exact replay; zero contacts; one image each |
| Presses | min 167, median 200.5, max 246 of 500 |
| Total requests | max 252 of 660 |
| Runtime | 351.192 seconds total; 11.471-18.649 seconds per seed |
| Live root service + unprivileged oracle | reward 1.0, 6/6, 214 presses, 1 image, 220 requests, 25.9 seconds |
| Contract tests | 13 dynamics/API + 17 security + 3 author-runner regressions passed |
| Native rendering during replay | 8 nonblank 384x384 views; every event and final fingerprint exact |
| Manifest | 24 unique initial images; exact registered order; all six runtime hashes match all three copies |
| Actual pinned ALE validate | untouched reward 0; privileged oracle reward 1; both completed all lifecycle phases |

`summary.json` contains every cohort result. `receipt.json` binds the world,
service, grader, hidden seeds, controller and P2-fixed cohort helper. `contracts.txt`
is the native test output. `live-first/` records the separate socket-service pilot.
`initial.png` and `dock-1.png` are representative native-rendered states, not
participant overlays. All ledgers, per-seed results, 24 initial images, remaining
views and container logs remain on koa04 at:

`/home/wang100/projects/ale-validator-hardening-20261116/evidence/icl-worker-v4-20261117T0108/`

Source mount: `/home/wang100/projects/ale-validator-hardening-20261116/icl`, read-only.
Image: `ale-icl-maintainer-env:20261116`,
`sha256:33860ab2d415874bfd4205d7133a070805b26c19321595f1dbd7a4591184c31e`.
Engine pin: `bfb75050aca2c54b45c5ab23bae6028a834a468a`.
Each container used no network, no GPU, 2 CPUs and 4 GiB memory with OSMesa/software
rendering, `LP_NUM_THREADS=2` and `OPENBLAS_NUM_THREADS=1`. See `../README.md` for exact commands.

Container names: `icl-worker-v4-hidden24`, `icl-worker-v4-fingerprints`,
`icl-worker-v4-live-first`, `icl-worker-v4-views`, `icl-worker-v4-contracts-final`.
All completed with exit status 0. The first-seed pilot uses a synthetic oracle
trajectory and is not actual ALE validation. The separate, real engine run
`validate-065d48b8` completed on 2026-11-17 UTC. See `ale_validation.json` for its
47 active source/data hashes, engine identity, phase results and actual verdicts.
Its evidence remains at `koa04:/home/wang100/projects/ale-validator-hardening-20261116/parent/icl-ale-validate-v4/`.
The older candidate's validation is not used as v4 evidence. No fresh v4
frontier-model result is available. The Opus difficulty target and image-only
feasibility remain open.

The P2 fix was also tested on the old candidate's full 24-seed cohort before this
merge; those receipts live under `../v3/exitfix/`, clearly separated from v4.
Production runtime/solution/grader hashes stayed frozen during current evidence
generation and the independent ALE snapshot. Only anchor provenance/measurements
were finalized afterwards; value=1.0 and full_at=1.0 never changed.
