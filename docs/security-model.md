# Security model — what is hidden from the agent, and what a badge proves

This page states what the benchmark defends against, how, and — just as carefully — what
it does not prove. It has two halves: the **grading trust boundary** (how the grader's
number stays authoritative against a hostile agent inside the sandbox) and the
**submission trust levels** (what the leaderboard can and cannot conclude from an uploaded
run). The mechanics are the engine's (`vendor/ale/docs/specs/standard-environment.md`) plus
the verify-stage kit (`shared/robotics_grader/`); the ingest rules are in
[`dev/20260902-engine-run-submission.md`](dev/20260902-engine-run-submission.md) §5.

## 1. Assets and adversaries

Assets: (1) **result integrity** — the score is what the grader computed, not what the agent
claims; (2) **hidden-protocol secrecy during the run** — the per-episode seeds, the grader,
the anchor and the oracle are unreachable from inside the sandbox while the agent works;
(3) **credential secrecy** — nothing credential-shaped leaves the host inside a bundle;
(4) **tamper-evidence** — an edited run record is detectable.

The primary adversary is the **tested agent**: arbitrary code running for hours as the
sandbox's agent account, and again — through the solver it leaves behind — while the grader
runs. It may try to inflate its metric, echo canned output, pre-create or overwrite the
verdict, or read the grader. A secondary adversary is a **submitter** who edits a bundle
after the fact, or relabels an authoring run as a model run. We do **not** defend against a
compromised host or a malicious operator: that is exactly the gap the `verified`
attestation exists to close (§4).

## 2. What stage isolation hides

Anti-cheat is **architectural isolation, not detection**. The mechanism is the engine's
stage protocol; the task only has to respect it.

| barrier | what it does | who enforces it |
|---|---|---|
| **Timing** | `verify/` (grader, `env.py`, `anchor.json`, `grader_config.json`) and `oracle/` are uploaded to the sandbox only when their stage runs. During the agent phase they do not exist anywhere in the sandbox. | engine (`standard-environment.md`: "`verify/` is also absent throughout this phase") |
| **Root-only staging root** | `/opt/ale` is `drwx------ root:root`; even paths that do exist there are untraversable for the agent account. | engine |
| **UID boundary** | `verify/run.sh` runs as root; the agent's solver is spawned **at the agent's UID** (`robotics_grader.Solver`, `deprivileged`, via `runuser`). The solver cannot read the grader's memory or environment, and Yama's `ptrace_scope` blocks tracing across UIDs. Artifacts the agent produced (checkpoints, pickles) are deserialized deprivileged too. | kit |
| **Verdict path** | The reward map is written to `ALE_VERDICT_PATH`, inside the root-only tree, through `ale_verify`'s atomic write; the agent can neither pre-create nor overwrite it. The engine cross-checks the reward file against `verification.json`. | engine + kit |
| **Seeds in memory** | The grader draws hidden seeds from OS entropy inside its own process (`seed_rng`); they never touch disk. The practice grader the agent gets uses different, visible seeds and prints the raw metric only. | kit + `scripts/lint_domain.py` |
| **No self-reported numbers** | The metric is computed from the grader's own simulator state. Nothing the solver prints is parsed as a score; the wire carries observations and actions only. | kit |
| **Network** | `network.mode: block` during the agent phase; the agent cannot fetch anything, including the public mirror. | engine |
| **Image hygiene** | The task image may not `COPY` `verify/`, `oracle/`, the anchor or the manifest — an image layer is readable by anyone who can run the image. | `scripts/lint_domain.py` |

Real-run evidence for the first four rows: [`evidence/isolation-audit.md`](evidence/isolation-audit.md)
(`/opt/ale` unreadable, `verify/` absent, a fake verdict refused, the kit untouchable).

On top of isolation sit the **match locks**, which are detection for canned output:
closed_loop graders fire hidden perturbation probes (`ProbeLock`) and require the action to
respond, so a constant, replayed or observation-blind policy scores zero
(`match_lock_ok = 0`); open_loop graders re-simulate every answer under their own dynamics
and reject one that does not start where it claims; artifact_rollout is structurally immune
(the grader is the only thing that ever runs the artifact).

**What "hidden" means, precisely.** The anchor and the oracle are hidden from the *tested
agent during its run*, by stage isolation. They are **not** secrets from maintainers, from
the server, or from the public: this repository is public and every task folder is in it
whole, `verify/anchor.json` included (the engine's loader refuses a folder without
`oracle/run.sh`, and a stub would change the task's identity). A human can read the anchor;
the agent in the sandbox cannot, and the agent is what is being measured.

## 3. What the exporter protects

`ale-export` packs what the engine wrote and adds nothing the server trusts on its own word:

- the `agent` block is derived from every episode's `lock.json` and must agree across the
  bundle; there is no `--agent-model` override;
- episodes run by the authoring harnesses (`oracle`, `nop`, `scripted`) are refused unless
  `--allow-builtin`, which stamps `submission_origin: smoke` — the server never scores those;
- redaction is by **value pattern** over string leaves (`harness/redact.py`: API keys, AWS
  tokens, bearer/authorization material, cookies) — never by key name, which would corrupt
  the engine's token-limit fields; `lock.task.source.path` becomes `"<local>"`;
- `members` hashes are computed over the stored (redacted) bytes; the receipt records the
  compressed and encrypted sha256 and the recipient key;
- only the recipient's **public** age key is ever handled on the packing side.

## 4. What the server proves — `validated` versus `verified`

For a bundle it can parse, the server rejects (writing no rows) on any hash or consistency
failure and otherwise recomputes the score from the **registry's** anchor, `full_at` and cap
(`engine:score_matches_recompute`, an exact port of `robotics_grader.scoring.ratio_rewards`).

The recompute catches: anchor / `full_at` / cap drift between the run's grader and the
registry (`task_identity`, `score_matches_recompute`), edits to any single member
(`member_hashes`, `manifest_consistent`, `verification_consistent`), arithmetic bugs in ported
graders (`reward_consistent`), relabelled authoring runs (`scorable_harness`,
`agent_phase_present`), and cherry-picked task subsets (release policy `zero`). It cannot
catch a *consistent* rewrite of every file: the members are plain JSON and the solver/verify
image is a local build whose identity is recorded but bound to no reference. `validated` is
therefore a **consistency badge**; authenticity comes only from a maintainer attestation after
an organisation re-run.

Concretely:

| status | meaning |
|---|---|
| `rejected` | a hash, manifest, verification or recompute check failed; **no** per-task rows are written |
| `received` | consistent, but not scored or not fully trusted: unknown task, drifted identity (`task_content_drift`, `engine_schema_drift`), anchorless grader run, authoring harness, or no challenge |
| `validated` | every check passed, the challenge verified, the anchor present, a scorable harness — the numbers are self-consistent with the registry |
| `verified` | a **maintainer attestation** (`VerificationEvent` with actor, `method = official_rerun`, reason, and `evidence_hash` = the re-run bundle's receipt `encrypted_sha256`), never an automatic outcome. An upload onto a `verified` row is refused with 409: revoke, re-upload, re-verify |

Things the server deliberately does **not** count: episodes with `env_error`, `task_error`,
`cancelled` or `interrupted` status (infrastructure failures — shown, never scored);
`-reverify-` regrades from a challenge upload (a regrade replaces its source episode only on
the maintainer path); tasks not `active` in the release.

## 5. Honest limits

- **Image provenance.** A locally built task image has an observed identity in `lock.json`
  but no registry reference to compare against; `engine:image_matches_anchor_run` is
  informational.
- **Engine provenance.** `lock.framework.commit` outside the registry's `engine.commits`
  is a warning, not a rejection — external partners may run a newer engine.
- **The host.** A self-hosted run is a consistent, tamper-evident *recording*, not proof
  that it happened under controlled conditions. Only an organisation re-run closes that gap,
  and the leaderboard says so in exactly these words: `validated` is a consistency badge,
  `verified` is an attestation.
- **Public anchors.** Because the repository is public and complete, the target value is public knowledge
  outside the sandbox. The benchmark measures what the agent does inside a blocked-network
  sandbox that does not contain it; it does not measure what a human who read the repository
  could type into an instruction.

## 6. Constraints the tooling must preserve

- The task image never contains `verify/`, `oracle/`, the anchor, or the manifest.
- `setup/payload/` never reads the anchor or the grader; the practice grader prints the raw
  metric only, never a ratio or score.
- `rewards` in the verdict carries only the gated `reward`; raw values ride in `metrics`.
- The exporter never scores, never reads `verify/`, never types the agent identity.
- Credentials live in the engine's `.env` on the host, never in a task folder or a bundle.
- Run outputs (`runs/`) are git-ignored; nothing is uploaded without an explicit
  `ale-export` + upload step.
