# Ground-truth scoring oracle

The scoring reference is exact GT, not the trained baseline. The pinned engine
explicitly permits solution/solve.sh to write a known-good state from reference
data it carries (vendor/ale/docs/specs/task-authoring.md, solution section).
This author-only reference is absent from evaluated-agent execution.

solve.sh runs the unchanged public-data baseline training inside baseline/.
Its checkpoint, selected and full training logs, config and sources remain intact.
It then packages the trained tensors under baseline.* and exact GT tensors under
oracle.* in the canonical model.pt. Top-level training_log.jsonl is an exact copy
of the real baseline log. Config explicitly identifies privileged GT predictions
and records the baseline checkpoint/log and oracle fixture hashes. No training
records are invented. The top-level train.sh reproduces both stages.

The GT detector hashes only the current RGB pixels plus dimensions, then returns
the corresponding boxes. Empty frames return shape (0,5); unknown pixels fail
closed. It reads no verifier paths, labels, sequence IDs or future frames. The
same normal worker validates artifacts and streams its predictions to the same
COCO metric. No grader branch recognizes an oracle marker or awards a constant
reward. Exact predictions must actually survive normal inference and scoring.

This is a privileged answer fixture, NOT an allowed agent strategy or a claim
of perfect learned perception. baseline tensors establish real training provenance;
the GT buffers determine the oracle predictions. To measure the learned detector,
run run_baseline.sh and evaluate its output without the GT packaging stage.

prepare_gt_fixture.py generates gt_fixture.json from the frozen private export.
It checks the exact annotation SHA256, RGB dimensions, prediction count and
conflicting labels on identical pixels. GT coordinates and pixel hashes stay in
solution/, as the repository prescribes for reference answers. They are outside
the agent image build context, setup payload and public asset dataset. The tested
agent has no solution/ stage and no network access. The evaluator's GT directory
continues to be root-only; its permissions are unchanged.

The fixture is derived from NPS-Drones data under the source BSD-3-Clause terms
included in tests/nps-LICENSE.txt. Reproduce with the task's frozen private export:

    python solution/prepare_gt_fixture.py /path/to/test solution/gt_fixture.json

Fixture/worker checks and author experiments are evidence of correct packaging
and scoring, not a replacement for formal ALE GPU lifecycle validation.

## Real Docker verification

DCC job 62774192 completed with scheduler exit 0:0. The exact existing verifier
image sha256:7fab623064e3b756800f93e83207c0b34b5bb63ee1e24270d7a0c87c75668f21
ran the current, unmodified verify.py and worker. No device or score override
was installed. Missing artifacts scored 0; a truncated checkpoint scored 0; the
GT submission scored AP50=1 and reward=1 over all 2,475 held-out frames in
203.38 seconds. All three containers exited zero. Real root/user separation,
root-only test labels and the verifier's network-denial probe remained in effect.

This CPU check packaged GT with the unchanged first trained baseline from job
62755551; its original weights and logs remained intact. It is distinct from the
fresh full solve.sh training job 62774159. Neither result is a claim of the
formal GPU Docker ALE lifecycle, and no learned detector is claimed to score1.

## Fresh full oracle execution

DCC job 62774159 executed the complete solve.sh at source
9c19c6a096c34ff034cc018a8b3715b6a2f5ec22 and completed with scheduler exit 0:0.
Actual 24,000-step training, selected 12,000 checkpoint export and GT packaging
finished in 1,184.85 seconds. Normal worker inference then processed all 2,475
frames and returned 3,491 GT detections in 88.05 seconds. AP50, AP50:95, APsmall,
AR300 and reward were exactly 1; pre/post evaluation artifact hashes were unchanged.
This is an H200/Apptainer author run, not the formal GPU Docker ALE lifecycle.
