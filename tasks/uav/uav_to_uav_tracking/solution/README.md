# Trained baseline and status

The reference trains Faster R-CNN with a ResNet-18 FPN
from generic ImageNet backbone initialization; it does not use a UAV-trained model.
Training and validation use the same visible assets and four-hour budget as an agent.
The fixed recipe uses 24,000 AdamW steps and limits its CUDA allocator to at most 24 GiB.
The detector consumes one frame at a time. The task allows causal temporal methods.

Run solve.sh (or its run_baseline.sh alias) to reproduce the learned reference. It installs no packages or downloads. Training
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
raw AP50. solve.sh submits only the learned detector and its actual training
artifacts; it never packages hidden answers. GT/empty endpoint checks belong only
to tests/metric_selfcheck.py. The measured reference mean AP50 is 0.5171147779,
not 1. The pinned ALE admission rule requires oracle reward=1; this reference does
not currently satisfy that rule on the fixed AP50 scale. Maintainer agreement on
this contract, GPU ALE validation and a formal frontier episode remain required.
See tests/REVIEW.md. Author tests do not establish admission.

The full NPS native DCC run completed 24,000 steps in about 25 minutes, but its
visible-validation detection quality remained low. A train-subset evaluation showed
a substantial generalization gap. Short frozen-BatchNorm and higher-resolution
inference experiments did not resolve it and are not part of this reference.
Completing training within budget does not establish benchmark difficulty.
Do not change the score scale based on this baseline's accuracy.
