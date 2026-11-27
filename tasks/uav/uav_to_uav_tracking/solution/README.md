# Trained baseline and status

The reference trains Faster R-CNN with a ResNet-18 FPN
from generic ImageNet backbone initialization; it does not use a UAV-trained model.
Training and validation use the same visible assets and four-hour budget as an agent.
The fixed recipe uses 24,000 AdamW steps and limits its CUDA allocator to at most 24 GiB.
The detector consumes one frame at a time. The task allows causal temporal methods.

Run run_baseline.sh to reproduce the learned baseline. It installs no packages or downloads. Training
writes model.pt, source, config and actual per-step losses under the declared
submission path. Verification only loads the frozen state dictionary and predicts.

prepare_release.py is the release exporter: it decodes all original videos, requires
one original annotation entry per decoded frame, and writes materialized JPEG files.
Train retains every frame; validation and hidden evaluation retain indices 1,6,11,... .
The public archive contains train/val only. The separate verifier generates hidden
frames directly from the pinned original sources, in a root-only directory.
prepare_data.py remains an author utility for existing PNG frames; its symlink outputs
must be materialized before distribution. Never reuse known-bad formatted labels.
Do not put data, weights, venvs or generated logs into this Git folder.

Scoring uses the measured GT endpoint, anchor=1 and full_at=1, so reward equals
raw AP50. This training recipe is a baseline, not the scoring reference.
The author-only GT oracle is solve.sh; see ORACLE.md for how it retains actual
training provenance and returns known reference boxes through the normal worker.
Before admission, pass the pinned ALE GPU validation and archive a formal
frontier-agent trajectory. Author tests do not establish that lifecycle gate.

The full NPS native DCC run completed 24,000 steps in about 25 minutes, but its
visible-validation detection quality remained low. A train-subset evaluation showed
a substantial generalization gap. Short frozen-BatchNorm and higher-resolution
inference experiments did not resolve it and are not part of this reference.
Completing training within budget does not establish benchmark difficulty.
Do not change the score scale based on this baseline's accuracy.
