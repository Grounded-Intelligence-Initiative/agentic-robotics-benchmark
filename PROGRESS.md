# PROGRESS.md · progress log for agentic-robotics-benchmark

> **What this is:** the progress source of truth for this repository, maintained by people
> and agents alike. It records milestone state, the last stopping point, known gotchas and the
> next step. **Disk is the truth** (git + this file + `CLAUDE.md`); a session's context is a
> volatile cache. Read this file first; update it (state / next step / gotchas) and commit
> before every hand-off. Where it disagrees with the code or `git log`, the code wins: fix
> this file.
>
> Last updated: 2026-10-15, later the same day (§6: the website is display-only; the
> spacecraft task is the worked example and `uav_hover` is gone; `ale-export` and the template
> download are retired; the registry's schema lives here).

## 0. 2026-09-07 — created by the rebrand from "ALE Robotics"

Origin: the maintainers' repository `ale-robotics-benchmark-private` at `86c2285` (its
history, including the Chinese progress log, stays there, archived) and the public mirror
`ale-robotics-benchmark` (archived, tag `legacy-v0.5-final`). This repository starts with a
fresh history and replaces both: it is public and carries every task folder whole, `oracle/`
and `verify/` included.

- **Layout flattened.** `ale-domain/tasks/` → `tasks/`, `ale-domain/templates/` →
  `templates/`, `ale-domain/identity-history.json` → `identity-history.json`,
  `ale-domain/_evidence/` → `docs/evidence/`, `ale-domain/README.md` → `tasks/README.md`.
  Verified with the engine loader: `drone_hover`'s `spec_hash` and `content_digest` are
  unchanged (a folder's location is not part of its identity), so `identity-history.json`
  still describes the tree.
- **Staging retired.** Both contribution channels (a pull request opened by hand; the
  website upload whose bot opens the same pull request) land a task directly at
  `tasks/<direction>/<slug>/`. Deleted: `contrib-staging/`, `scripts/promote_contribution.py`,
  `.github/workflows/promote-on-merge.yml`, `scripts/publish_public.py` and its tests.
  `.github/workflows/pr-checks.yml` now runs the full check set on the final path, needs no
  secret (fork pull requests run it), and its registry identity cross-check treats a slug the
  history does not know as a notice, not a failure. `scripts/lint_domain.py` gained battery 7
  (placement: `tasks/<metadata.direction>/<name>/`); `harness/ale_onboard.py` and the lint
  walk only `tasks/` and `templates/`, never the repository root (that would descend into
  `vendor/ale`). `.github/PULL_REQUEST_TEMPLATE.md` is new.
- **Renamed.** `pyproject` name `agentic-robotics-benchmark`; the download is
  `agentic-robotics-task-template.zip` with folder `agentic-robotics-task-template/`;
  `export_registry.py` release name "Agentic Robotics Benchmark" (`provenance.tasks_repo`
  follows the directory name); `GETTING-STARTED.md` rewritten for the two channels and the
  new domain `agentic-robotics-benchmark.org`; README, README.zh-CN, CLAUDE.md, CONTRIBUTING.md
  (new), docs and skills rewritten for the brand and the flat paths. Added `LICENSE`
  (Apache-2.0, from the old public mirror) and `NOTICE`.
- **Unchanged on purpose** (runtime identifiers stay until a separate decision): `ALE_*`
  environment variables, the protocol names `ale-engine-run/v1` and
  `ale-robotics-task-registry/v2`, the console scripts `ale-onboard` / `ale-export`,
  `harness/export_run.py`'s exporter and benchmark names (they travel in bundle manifests
  the server reads), the engine pin `vendor/ale` @ `90a7c1c`, and every file of
  `tasks/control/drone_hover` (its `task.yaml` and `verify/verify.py` still carry three
  comments saying "ALE Robotics domain policy"; editing them would change the task's
  `content_digest`, so they go when the task goes: owner decision, delete `drone_hover` when
  the first community task is merged).
- **Not carried over** (owner decision; they live in the archived private repository):
  the Chinese `PROGRESS.md` history, `docs/prompts/` (session hand-off briefs),
  `docs/dev/archive/` (the pre-ALE snapshot and audits), `docs/tree_diagram/` (the taxonomy
  visualisation source; the workspace repository keeps the taxonomy markdown).
- **Verified at creation:** `uv run pytest -q` 225 passed, 1 skipped (the website registry
  cross-check, which runs when the website checkout is next to this repository); `ale lint ../../tasks` and
  `ale lint ../../templates` ok from `vendor/ale`; `scripts/lint_domain.py .` ok;
  `robotics_grader` selftest passed; the template's vendored kit byte-identical to
  `shared/`; the workflow file parses; a clean registry export reproduces
  `identity-history.json` byte for byte and the template zip builds (22 entries); ruff reports
  the same nine pre-existing style notes as before (non-blocking in CI).

## 1. 2026-09-08 — pushed; the pending contribution re-opened here

- `main` pushed to `https://github.com/Grounded-Intelligence-Initiative/agentic-robotics-benchmark`
  after the owner signed off the privacy checklist; the first `pr-checks` run on `main` is
  green (engine lint over `tasks/` + `templates/`, identity cross-check, static checks).
- The contribution `fault_adaptive_spacecraft_docking` (old repository PR #14, uploaded on
  the website by Jiankai-Sun, revision 2) is re-opened here as https://github.com/Grounded-Intelligence-Initiative/agentic-robotics-benchmark/pull/1 on branch
  `contrib/fault_adaptive_spacecraft_docking`: commit 1 = the 20 uploaded files byte for
  byte at `tasks/control/fault_adaptive_spacecraft_docking/`; commit 2 = `verify/robotics_grader/`
  re-vendored from `shared/` (the upload carried the 2026-09-05 kit). Old PR #14 closed with
  a pointer. Still to do by a maintainer before merge: `ale validate` (the anchor was a
  dry-run measurement; the oracle runs three hours).

## 2. 2026-09-08 — website live; both channels rehearsed

- The website (`agentic-robotics-benchmark-website`, formerly `ale-robotics-website`) is live
  on `https://agentic-robotics-benchmark.org` with the two-channel Contribute page, the
  download built from this repository at `b04359f`, and the registry exported from the same
  commit; `ale-robotics.org` and its `www` 301 to the new domain.
- Dress rehearsal with a template-derived task `rehearsal_reach_20260908` (uav/control;
  anchor `0.0813` measured by `ale validate` on the maintainer's machine, oracle exactly 1.0):
  channel A (pull request #2, opened by hand) and channel B (pull request #3, opened by the
  website's bot after the upload precheck) produced byte-identical trees at
  `tasks/control/rehearsal_reach_20260908/` (20 files, same modes, nothing outside the
  folder), and `pr-checks.yml` passed on both. Both were closed without merging and their
  branches deleted, as planned.
- Website follow-ups found by the rehearsal and fixed the same day: the `www` alias of the
  new domain (Caddyfile `WWW_DOMAIN`), and a 500 on `POST /api/v1/challenges` caused by the
  rename creating a second release row (the website's seed now keys the release by version
  and merges duplicates).

## 3. 2026-09-08 — the paper-onboarding skill link removed

- `.claude/skills/onboard_paper` was a symlink to `skill/`, the paper-reproduction
  onboarding skill of the project's first months; its target had been deleted long before
  the rebrand copied the dangling link over. Removed (owner decision: the benchmark is not
  about reproducing papers, so no replacement). The repository has no `.claude/` directory
  now; the agent-facing skills are the ones under `skills/onboard-*/`, routed by
  `skills/onboard-task/SKILL.md`. `docs/research/agentic-robotics-survey.md` pointed at
  the old `skills/SKILL.md`; it then pointed at the router (the survey left this repository
  on 2026-10-17).

## 4. 2026-10-12 — `drone_hover` is now `tasks/uav/uav_hover`; tasks are placed by platform

- Owner decisions: `tasks/control/drone_hover` becomes `tasks/uav/uav_hover`
  (`name: uav_hover`), a dummy task that frontier models complete easily, kept only to show
  what a task's file structure looks like. This replaces "delete it with the first merged
  community task". `tasks/README.md` is now one paragraph saying so: there is no open-loop /
  closed-loop distinction any more.
- A landed task sits at `tasks/<metadata.platform>/<name>/` (was
  `tasks/<metadata.direction>/<name>/`). Changed with it: `scripts/lint_domain.py` battery 7,
  the instantiate target of `harness/ale_onboard.py`, the template download's getting-started
  text (`scripts/build_template_zip.py`), the comment in `templates/task/task.yaml`, the pull
  request template, the comments in `pr-checks.yml`, README, README.zh-CN, CONTRIBUTING,
  CLAUDE.md, docs and skills. The tests now refuse a direction-named folder.
- Identity: the new `name` gives the task a new `spec_hash` and `content_digest`, so the
  registry identity cross-check reports `uav_hover` as a new task; `identity-history.json`
  keeps the `drone_hover` record. `image/` is untouched, so the image identity (and an image
  already built from it) carries over; its Dockerfile still says `drone-hover` in two log
  lines and one comment.
- Verified locally (the `pr-checks.yml` steps): `ale lint ../../tasks` and
  `../../templates` ok; registry export + identity cross-check ok (`uav_hover` new);
  `robotics_grader` selftest and the vendored-kit diff ok; `uv run pytest -q` all passed with
  the engine venv present (the template-zip test, which refuses a dirty tree, passed on a
  clean throwaway commit); `scripts/lint_domain.py .` ok; ruff reports the same nine
  pre-existing notes.

## 5. 2026-10-15 — the benchmark owns its taxonomy; `direction` and `metric` metadata retired

- Owner decisions: this repository is self-sustaining, and the website is only a front end
  that displays what this repository exports. The platform vocabulary is
  `tasks/platforms.json`, fixed keys `arm` (robotic arm), `hand` (dexterous hand),
  `quadruped`, `humanoid`, `uav`, `driving` (autonomous driving), `underwater` (UUV/AUV),
  `space` (space robot), `soft`, `surgical`, `swarm`, `subsystems`. `metadata.direction` is
  removed: platform is the only taxonomy field. `metadata.metric` is removed: upstream ALE
  does not define it (its metadata is "optional descriptive values; no runtime behavior"), and
  the metric and its direction are declared in `verify/anchor.json` only, where the grader
  reads them.
- `scripts/lint_domain.py`: battery 7 refuses a `metadata.platform` outside the vocabulary;
  the metric checks (named, not hardware-dependent, `direction` higher or lower) read
  `verify/anchor.json`; the two "task.yaml and anchor.json agree" checks are gone; stale
  `direction` / `metric` metadata is ignored, like `genre` / `paper`.
- `scripts/export_registry.py`: `platforms` comes from `tasks/platforms.json`; a task whose
  platform is not a key is refused; `metric_name` / `metric_direction` / `metric_label` come
  from `verify/anchor.json`; no per-task `direction`, no `directions`, no `category_tree`, no
  `hardware_invariant` (a self-declaration nothing checked; the lint's token rule is what
  enforces invariance); `--carry-over` is removed; `tasks/platforms.json` is held to the same
  clean-tree rule as the task folders, since it goes into the document. `schema_version` stays
  `ale-robotics-task-registry/v2` on purpose: the website's sync treats any other version as
  a legacy merge and its seed refuses unknown versions, whereas a v2 document without
  `direction` fails the website's current schema closed (nothing is written).
- `harness/ale_onboard.py`: the spec's `platform` must be a key (the `misc` fallback is gone),
  and the spec's metric name and direction are written into the copied `verify/anchor.json`
  (a name without a direction is `higher`, never the template's `lower`). The gates check that
  the grader reports the anchor's metric: an oracle record without it is `needs_input` (the
  anchor's name is what the export publishes and the server's recompute looks up). An anchor
  still at `TODO` names no metric, and `needs_anchor` then asks for the name with the value.
- Template: `metadata: {platform: TODO}` with the key list; the anchor's `metric` is `TODO`
  (the template grader's `mean_goal_distance` is an example, not a default);
  `templates/README.md`, the template-zip guide, README, README.zh-CN, CONTRIBUTING,
  CLAUDE.md, docs and skills follow; the two 2026-09-02 design records carry a dated note. The
  pointers that called `tasks/README.md` the task contract now point at `templates/README.md`,
  and the upload docs carry an interim note (the website still runs the old form: open the
  pull request yourself). The test that read the sibling website checkout is deleted (the
  dependency pointed the wrong way).
- `uav_hover`: metadata reduced to `platform: uav`; its anchor gains `"direction": "higher"`
  (its grader already hard-codes higher), so its `spec_hash` and `content_digest` change again
  (still not in `identity-history.json`; CI reports it as a new task).
- Verified locally (the `pr-checks.yml` steps): `ale lint ../../tasks` and `../../templates` ok
  (`tasks/platforms.json` is invisible to the engine); registry export ok (`platforms` = the 12
  keys; no `direction`, `directions` or `category_tree`) and identity cross-check ok (`uav_hover`
  new); `robotics_grader` selftest and the vendored-kit diff ok; `uv run pytest -q` all passed
  (the template-zip test on a clean throwaway commit); `scripts/lint_domain.py .` ok; ruff the
  same nine pre-existing notes.
- Still bound to the website (not changed here, candidates for a later pass): website policy
  copied into this repository's text (upload URL, size limits, the precheck rule list); the
  JSON Schemas of the registry and of `ale-engine-run/v1` exist only in the website
  (`packages/contracts/schemas`); `provenance.tasks_repo` is the clone's directory name; the
  `episode_id` pattern in `harness/export_run.py` mirrors the server's.

## 6. 2026-10-15 — display-only website; the spacecraft task is the worked example

- Owner decisions: the website is a display-only front end. It shows the tasks, and its
  submission page links the contribution guidelines (two Google Docs, English and Chinese,
  `/view` links). It takes no uploads and no run or score submissions, and the leaderboard is
  removed for now. The registry and submission-bundle JSON Schemas are decoupled from the
  website. `ale-export` is removed. Contribution follows the guidelines: sign up, register
  the Task Name (= folder = `task.yaml` `name`), measure difficulty with frontier models
  (target roughly below 0.5), and open a pull request.
- #6 (kit). The grader's seed stream is BLAKE2b keyed by OS entropy. Before, splitmix64 let
  a solver rebuild the stream from probe perturbations and compute the next episode's seed.
  - After an aborted episode the solver is restarted: at most 3 times, with its process group
    stopped and READY required again. A stray reply line no longer cascades into every later
    episode.
  - The template charges an aborted episode what standing still would cost. It used to drop
    aborted episodes, so a weak policy could abort its bad starts and score 1.0.
  - Malformed actions abort an episode instead of crashing the grader.
- #1 merged (merge commit, contributor history kept) after a maintainer pass on its branch.
  - The agent held a byte-identical simulator copy, and 31-bit LCG seeding let it
    brute-force each episode's seed. Episodes now use 64-bit, keyed-hash seeding.
  - The instruction now matches the grader.
  - The anchor was re-measured: 40 fresh oracle runs, mean 0.7440 (sd 0.063); `full_at` went
    from 0.90 to 0.75. With 0.90, 2 of 31 fresh runs failed `ale validate`.
  - Eight fresh validates passed. The `contrib/fault_adaptive_spacecraft_docking` branch is
    kept because the guidelines link to it.
- This change:
  - `tasks/uav/uav_hover` removed. A dummy that frontier models solve cannot be a real
    example or a submittable task. `tasks/README.md` names the spacecraft task as the worked
    example.
  - Removed: `harness/{export_run,bundle_crypto,redact,trajectory,integrity}.py`, their
    tests and fixtures, `docs/submission-format.md`, `scripts/build_template_zip.py` and
    its tests, and the `zstandard` and `pyrage` dependencies.
  - `schemas/task-registry.schema.json` is the registry's contract, tested against the
    export.
  - `provenance.tasks_repo` is the origin's `owner/name`.
  - Docs follow the guidelines: one path, a frontier-model difficulty test, and no website
    channel. The security model now lists the limits that matter (open egress during verify;
    the probe lock is not a proof of honesty).
- Measured on 2026-10-13, on the previous version of the task: Fable 5.1 1.0, Opus 5.5 1.0,
  Opus 5 0.84, Fable 5 0.74. The 2026-10-15 runs on the merged version use an evaluation copy
  whose image only adds harness workarounds (outside the repository, `arb-eval`): the stream
  idle timeout, a `.pth` so the harness's hard-coded `PATH` still finds the venv's numpy, and
  for the later runs `CLAUDE_CODE_MAX_OUTPUT_TOKENS`. Results (n = 1 per configuration):
  - Opus 5.5: reward 1.0 twice (`mean_docking_quality` 0.940 and 0.982; ratio 1.26 and 1.32;
    46/48 and 48/48 docked).
  - Opus 5: the first run ended in an API error before writing `run.sh`. The second hit the
    3 h agent timeout (no score); its leftover submission, scored afterwards in the sandbox,
    reaches reward 1.0 (0.740, ratio 0.99).
  - Fable 5.1 and Fable 5: empty submissions, reward 0. Every long turn hit the response
    cap (64k, then 100k) or the gateway's 30-minute request limit (at 128k) with no text and
    no tool call, so this is an evaluation-stack failure, not difficulty.
  - Conclusion: the task does not stump frontier models; its difficulty must go up (results
    posted on #1 for its author). About $104 at list prices.

## Open items

- Website, display-only pull request: remove the backend (API, database service, sign-in,
  upload, review, admin), the leaderboard and the run pages; `/submit` links the two
  guidelines; the registry is validated against this repository's schema; install a fresh
  export made from a clean `main`. The production database volume is left untouched on the
  server.
- After this merges: export the registry from a clean `main`, commit `identity-history.json`
  (the spacecraft task is new; the `drone_hover` record stays as history), and install the
  export in the website.
- Raise the spacecraft task's difficulty in a new pull request (Jack or the maintainers):
  Opus 5.5 saturates it (§6). The guidelines' target is a frontier `reward` roughly below 0.5,
  from a harder engineering or perception challenge, not penalties. Then re-measure the anchor
  and re-run the frontier models.
- Pull request #4 (`ball_physics_stepper`) must re-vendor the kit from #6 (CI's byte
  comparison), and should move to 64-bit, keyed-hash seeding if its agent holds a simulator
  copy.
- The guidelines' example `task.yaml` link points at the `contrib/` branch. Point it at
  `main` once the docs are next edited.
- Engine (upstream) issues that skew agent runs:
  - the claude-code harness kills a model stream after 300 s without a byte;
  - it hard-codes the agent's `PATH`, so the image's `/opt/venv/bin` is lost;
  - the CLI caps a response at 64k tokens and the gateway ends a request after 1800 s in total
    (`gateway/server.py`), which leaves long-thinking models with empty turns;
  - egress is opened before verify while a closed_loop solver runs.

## 7. 2026-10-23 — messy-desk PR #12 diagram refinement

- Redrew the object and request automata as compact, color-coded Mermaid flowcharts
  in the task's scoring README and PR description. Kept state names, object values,
  outcome branches and request cycling visible; moved qualifications into captions.
- The object diagram is a lifecycle overview: transit, arrivals, disturbances and instance
  reuse are omitted explicitly. The request diagram groups the two successful outcomes
  and the incorrect-claim / expiry outcomes. No simulator or scoring behavior changed.
- Checked Mermaid parsing and rendered both diagrams locally; task lint passes.
- Follow-up: publish tightly cropped, opaque white PNGs with editable Mermaid sources
  to keep the backgrounds stable in dark mode. Keep the object layout; compact the
  request diagram vertically with Idle above the new-request transition and no Closed node.
- Moved transition labels beside the arrows, removed opaque label backgrounds, and
  reduced label emphasis so the arrow paths remain continuous and unobstructed.
- Validation remains pending: stabilize oracle grasping, measure its anchor, run
  `ale validate`, and measure frontier-model difficulty before marking the PR ready.

## 8. 2026-10-23 — first-target-hold request credit

- Request points award +1 for the first current target hold per request, topped up
  to 2 points by verified delivery; a correct absence declaration also earns 2 points. Repeated
  holds add nothing, and failure/expiry preserves only the first-hold credit already earned.
- R divides total points by twice the number of opened requests, including an ongoing one, so
  partial credit is immediate. The separate request success-rate diagnostic remains available.
- Updated the environment event, hidden/practice scorer copies, instructions, formula and
  diagrams. Historical contact is insufficient; the target must currently be in held state.
  The existing loss predicate remains centre z < -0.2 m; diagrams explicitly say falls off
  table. Releases onto the table or into a container are not loss events.
- Focused request-credit tests cover deduplication, completion, failure, absence, current
  holds and copy parity. Full Isaac rollout validation and anchor measurement remain pending.

## 9. 2026-10-23 — messy desk: dev-wire live score, exact depth, oracle grasping (Claude session)

- Practice tools: every `step`/`reset` reply of the simulator carries the running score (the same
  `EpisodeScorer`), `{"cmd": "score"}` adds per-object value/baseline, `{"cmd": "request", ...}`
  opens a request on demand; `practice_grader/grade.py --live N --score-file PATH`. Verified end
  to end with a dummy wire agent (obs keys as documented, untouched = 0, marks: claim_wrong /
  absent_correct / ignored without a request). Known gap: a request opened by the dev command
  is not counted by the dev scorer until the next `step` (its event rides the `request` reply).
- Simulator: `anti_aliasing: 0`. The default DLSS accumulated previous frames into the depth
  images: ghost depth under moving fingers and a skirt of floating points around every object,
  which inflated measured widths by ~2 cm and hid flat objects. `BASE_SCALE` master chef can
  0.62, tuna can 0.72 (both exceeded the 8 cm hand); instance factors 0.92 / 1.00.
- Oracle: containers are fitted from the static cameras only (the clipped hand-camera rim view
  biased the red container's fit and leaked its walls as loose clusters); `plan_grasp` = pinch,
  else rim grasp for hollow wide objects; `pick` re-centres from the hand camera 10 cm above the
  object; silhouette pixels dropped before back-projection. Grasp validation on single objects
  is running; refs.json must be rebuilt from `calib_collect2.py` output once grasps are stable.
- Frontier run decided: Opus 5.5, claude-code harness, subscription auth, 1 episode first.

## 10. 2026-10-23 — messy desk: scene physics fixes found while debugging the oracle (Claude session)

- Every YCB object used to spawn lying on its side: the stock `Axis_Aligned` meshes are authored
  with their height along local Y and `_spawn` only applied a yaw. Desk arrivals now stand
  upright (Rx(-90)); scissors, marker and both clamps (local Y = their length) lie flat. Desk
  spawns are set down 4 mm above the desk (`half_h` from the USD bound) instead of dropped.
- The stock table is flat only for |x| < 0.709 and then rises into a 1.2 cm rim, which tipped
  over objects that arrived at x > 0.71 and appeared as a 15 cm "object" strip in the hand camera.
  The room is now shifted +0.10 m in x (`ROOM_OFFSET_X`): the whole reachable desk is flat.
- Scales: every model offers a pinch of at most ~6.8-7.2 cm in any resting pose (cracker box
  0.42, sugar 0.73, pudding 0.63, gelatin 0.80, potted meat 0.70, mustard 0.71, foam brick 0.92,
  tomato soup can 0.92: at 6.8 cm the 8 cm hand kept only 6 mm per side and missed).
- Measured: the open fingertips stop on the desk at TCP z = 0.028 (TCP_Z_MIN unchanged); the
  visual finger mesh suggests 1.2 cm, the collision geometry is longer. Thin objects lying flat
  (marker, scissors) are pinched with the fingertips at the desk; a pinch along a long object
  wedges it between the fingers instead (lifted with the hand open).
- Desk arrivals point their long side away from the nearest container wall (+-0.6 rad): with the
  20 cm Franka hand, a long object lying along a wall could not be pinched in place (Monte Carlo:
  81 % -> 95 % pinchable).
- `sim_client`: a start attempt that hangs (Isaac's renderer occasionally deadlocks on the first
  frame of a fresh process, seen twice today) is killed and retried, 3 x 480 s.
- Instruction: draft banner removed; interpreters stated by absolute path (the claude-code harness
  hard-codes PATH without /opt/venv/bin); request example matches the env's text.
- Oracle: perception subtracts the desk profile; hand-camera component chosen by size x proximity;
  width check keeps a 14 mm margin (the hand-camera cloud loses ~7 mm per side); tall objects are
  grasped mid-body; rim grasps go 22 mm deep; the behaviour loop restarts after any exception.

## 11. 2026-10-23 — messy desk: arm motion, reach, piles, oracle search (Claude session)

Opus 5.5 frontier run in flight (`ale run ... --agent claude-code --model claude-opus-5-5 --set
agent.version=2.1.309`; the harness default 2.1.227 is rejected by the model). `ale run` uploads
the task's `verify/` folder from the local path when the verify stage starts, so `verify/` stays at
commit 2278f3f until that run is scored; the changes below live in `setup/payload/` and `oracle/`.

- Arm motion (env): each step's IK joint targets are interpolated over the 6 physics substeps.
  Applied at once, the stiff drives covered the delta in 1-2 substeps and the jerk flung held
  objects out of the fingers (12 of 42 test grasps dropped in transit, e.g. the sugar box slid
  3.7 cm during a 3 cm/step lift). A 5 cm delta now completes ~96 % within the step.
- Reach (env): the arm cannot reach down at r < ~0.25 m around |y| = 0.15 (IK refuses every
  step); arrivals keep 0.26 m from the base (was 0.22). At x = 0.19-0.22 a hand turned 90 degrees
  hits the container walls.
- Initial piles (env): objects are set down on top of the pile near the container's centre line
  and settled one by one (were thrown in from 0.25-0.65 m and rolled to the ends). A top-down
  Franka hand (0.205 m wide) fits inside a 0.28 m container only within +-2-3 cm of its centre
  line: 20 % of pile objects were reachable, now 36 % (1.3x longer containers would give 47 %,
  not worth shrinking the desk strips).
- `lab` with a null yaw sets an object down the way arrivals are (dev aid, task card updated).
- Oracle: references rebuilt for the current scales (161 static, 166 hand-camera views; the old
  LOO split was buggy: "_p" in "potted"/"pitcher" excluded whole models); hand-camera + 0.7 x
  static distances fused (LOO class 166/166); narrowest-pinch yaw; a pinch-position search for
  clamps/scissors; gentle placing near the centre line; container inventory (what it put where,
  which initial piles are still unseen); request search digs the reachable top object, delivers it
  straight to red when it is the target, and never claims absence while a container is blocked.
- Grasp test (42 arrival-like spawns, both instances of every model): 18/42 -> 33/42.
- Frontier run 1 (Opus 5.5, old env, `be35f322`): the agent was still debugging at the 4 h limit
  (569 tool calls) and the engine does not verify a timed-out agent phase (status `timeout`, no
  metrics). The instruction now states the hard 4 h limit and that a cut-off session is not graded.
  Its last submission is kept in the run's `artifacts/submission`.
- Domain lint: the practice grader's live line no longer reads a `"score"` key (the lint's
  leaky-output pattern); only the two expected null-anchor problems remain.

## Gotchas

- `export_registry.py` refuses a dirty tree on purpose: `content_digest` covers every
  regular file, ignored `__pycache__/` included. Export from a clean commit; `--allow-dirty`
  is for throwaway exports and never writes the canonical `identity-history.json`.
- The template and the worked example each validate in well under a minute.
- Engine commands run from `vendor/ale` with `VIRTUAL_ENV=` cleared and
  `DOCKER_HOST=unix:///var/run/docker.sock` pinned (two docker contexts on the dev machine).
  `uvx ruff`, not `uv run ruff` (ruff is not a project dependency).
- `uv venv --python 3.12` puts its managed interpreter under `/root/.local/share/uv`, which
  the agent account cannot traverse; task images set `UV_PYTHON_INSTALL_DIR=/opt/uv-python`
  and keep the Dockerfile's agent-UID import self-check. The agent UID must traverse (not
  only read) everything the venv points at.
- The engine stages `ale_verify` into whatever answers `python3` and refuses < 3.12.
- `ale validate` does not keep a stage's stderr; it is in the episode's
  `trace.execution.jsonl`.
- A null anchor is not an error: the engine fails the run with `oracle_not_full` and
  `harness/ale_onboard.py` reads that as `needs_anchor` with the measured value.
- A task that hands the agent a copy of its simulator must seed episodes with 64 bits and
  draw hidden values through a keyed hash; an LCG or a 31-bit seed gives the episode away.

### 2026-10-23 — provisional anchor
- Oracle a5c3982 measured through the task's verify stage (task image, network off): episode 0 S = 0.641 (O 0.72, R 0.56); episodes 1–2 running. Anchor recorded as 0.641058, full_at 0.8 (template default), marked provisional.
- Opus 5.5 run 1 (old env build, timed out): submission graded offline on practice seed 11, 20000 steps: S = 0.353 (O 0.61, R 0.10). Run 2 on the current build in progress.
- Open: full_at from the 3-episode spread; official `ale validate` once the GPU is free.
- Agent budget lowered 4 h → 1 h (template default; the 4 h was a scaffold-time choice, not a benchmark requirement). Opus runs 1–2 are kept and reported as extended trials with a 4 h budget. Official `ale validate` queued to start when run 2 releases the GPU.
- 2026-10-23 (owner decisions): four requests per episode (one per 5000-step window, `3a698f5`); O scores the episode's final state, and mark 3 (*done: episode*) ends the episode once its last request has closed (`a1321ca`; earlier done marks are ignored so all four requests count in R). The oracle sends mark 3 after its fourth request when nothing is left to sort. Anchor must be re-measured on this build (old-env oracle: 0.641 / 0.436 / 0.482, per-episode sd ~0.11). Validate is gated on a new anchor. Oracle pick work in progress (hard objects: mustard, bleach, clamps, scissors, pitcher, tuna).
- Superseded the final-state score the same day (`d266c95`, owner): each request is a find-and-organize task judged at its close; O = mean of organization snapshots at every request close plus the last step; no episode-level done mark. While a request is open its object belongs to red only (+1 in red or held, -1 in any other container, 0 on the table); its baseline is re-read at the opening and restored the step after the close. Oracle: after the target is in red (or absence is settled) it puts back what it dug out and sorts loose objects, then marks (FINISH_MARGIN 350 steps before the deadline).
