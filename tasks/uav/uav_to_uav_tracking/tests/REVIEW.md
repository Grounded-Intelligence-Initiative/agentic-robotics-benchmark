# Verification review

The parent process holds ground truth and computes single-class COCO AP50. It streams
only current RGB pixels to the child, and checks finite Nx5 output, bounds, confidence,
maxDets=300, startup/step/total deadlines and byte-limited replies. Writes have deadlines
too, so a model that stops reading cannot hang the grader. Every complete sequence
starts a fresh process, and model.reset() is called before inference.

The task uses a separate ALE verifier. Python imports and checkpoint deserialization
run through canonical robotics_grader.deprivileged at the user's UID. tests/ stays
root-only. Only the trusted label-free worker is copied to a root-owned readable
temporary directory. The network seccomp wrapper is reused from the repository UAV
search task and prohibits non-Unix sockets, io_uring and nonnative syscall ABIs.
The verifier requires successful clearing of the agent account and probes seccomp
installation before loading any submission.

The separate verifier Dockerfile downloads the official video and annotation archives
at fixed SHA256 values, decodes clips 41–50 with the same pinned exporter and image
encoding used for public data, and retains only every fifth frame. The final data
directory is /opt/uav-heldout/test under a root-owned 0700 parent. Raw downloads are
removed in the same build layer. The agent image contains only the public train/val
release pinned in environment/assets.sha256; no hidden release is uploaded there.

Author checks: test_contract.py covers AP interpolation, negative frames, IoU thresholds,
300 detections, original YX coordinate order and 1-based time_layer indexing,
malformed output, startup failure, stalled read,
stalled prediction, oversized replies and rejection of private archive paths/links.
The public and private metric/worker copies
must remain byte-identical. The shared grading package is copied unchanged.

Raw AP50 compares detections directly with ground-truth boxes. Exact ground-truth
predictions without false positives attain 1.0; an empty prediction set attains
0.0. The task fixes anchor=1 and full_at=1 at this measured GT endpoint, so
the unrounded reward equals raw AP50. The engine serializes primary reward to
two decimals and retains full-precision AP50/reward_exact. A trained baseline is
not the denominator.
The aggregate evidence is in gt_reference_measurement.json (DCC job 62738778).
This establishes the metric's endpoints, not a trained detector's accuracy or
the engine's oracle gate. Authors can check a complete frozen export with:

```bash
python tests/metric_selfcheck.py /path/to/split/annotations.json --output metric-selfcheck.json
```

This author-only helper never runs in the agent stage, exports no label-derived
submission. Its report provides the measured GT endpoint used by the anchor and contains aggregate
metrics and annotation/source hashes for reproduction.

These are not proofs that submitted training logs are authentic. The requirement to
train must be audited from the agent trajectory and reproducible source. There is no
claimed cryptographic training gate. Hidden test data may be public in the source
research corpus; sealed development and explicit external-data rules provide the
benchmark's operational holdout, not globally novel images.

The full source export contains 51,951 train images, 1,190 validation images and
2,475 hidden images. Exact JPEG SHA256 overlap across all split pairs is zero;
this does not prove absence of related scenes or near-duplicate views.

The original agent and verifier Dockerfiles built under real Docker in a DCC-hosted
KVM guest on 2026-11-26. Agent-UID checks found every public image and no hidden data.
The verifier's rebuilt annotation and image-hash manifests matched the frozen export.
CPU container smoke tests used the unmodified verifier and pinned ale_verify library:
missing and truncated artifacts returned zero without crashing; an empty detector ran
all 2,475 frames across ten sequences while asserting a non-root UID, denied label/anchor
reads, denied verdict writes and denied Internet sockets. These tests are not ALE
validation and did not exercise CUDA or ALE's sandbox lifecycle.

## Remaining admission requirements

GPU execution of this exact verifier, isolation checks under the full ALE sandbox
lifecycle and a formal frontier episode remain unverified.

There is also a scoring/admission contract conflict: CONTRIBUTING.md and the pinned
engine's docs/specs/episode-protocol.md require an untouched reward of 0 and an
oracle reward of 1 after rounding. The real learned reference has mean AP50
0.5171147779. solve.sh now runs that learned reference only; the fixed GT scale
must not be normalized to the baseline, and hidden answers must not be packaged
as a detector. It therefore does not currently satisfy the oracle=1 gate.
Maintainer agreement on supporting this raw-metric training task is required before
admission. A qualified GPU host alone does not resolve the score requirement.

Earlier GT-packaged submissions are retained only as historical grader endpoint
evidence in RESULTS.md. They are not the current solution and cannot establish
successful learned training or satisfy this task's intended reference contract.
The current source contains no pixel-keyed hidden-label fixture or GT model builder.
