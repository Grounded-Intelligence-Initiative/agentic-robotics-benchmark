# Train a UAV-to-UAV video detector

Build and train a computer-vision model that locates other UAVs in RGB images captured
from a moving UAV. This edition of the registered UAV-to-UAV tracking task evaluates
causal detection; object identities and flight control are outside its scope.

## Data and allowed resources

Read /opt/uav-data/DATA_CARD.md. The training and validation COCO annotations and
images are under /opt/uav-data/train/ and /opt/uav-data/val/. There is one foreground
category: UAV (category_id=1). Empty images are valid negatives. Sequences are split
by complete source clip, never by random frames. The held-out split is unavailable
during development. Validation and held-out sequences use every fifth source frame;
frames are delivered in chronological order within each sequence.

You have four hours, one GPU, 8 CPUs and 32 GiB RAM. Use only the provided data,
packages and optional general-purpose ResNet-18 ImageNet initialization. No network
is available. Do not use UAV-domain pretrained models, external data, held-out labels,
or lookup tables of test frames. Train or fine-tune a visual model on the training
split. You choose the architecture, preprocessing, augmentations and optimization.

## Submission

Leave these files in /home/user/submission/:

- model.pt: your trained, nonempty PyTorch state dictionary mapping parameter/buffer
  names to tensors (torch.load with weights_only=True).
- model.py: exports Detector(checkpoint_path, device).
- train.sh: reproducible training entry point, with supporting source files.
- config.json: a nonempty JSON object with architecture, training settings, seed and
  data/version identifiers (at most 1 MiB).
- training_log.jsonl: actual step/loss records from your training run. Each record
  must contain a positive integer `step` that strictly increases and a finite numeric
  `loss`. Sparse logging is allowed. The log must be nonempty, at most 128 MiB,
  with each line at most 64 KiB.

Detector must implement reset() and predict(frame). Each frame is an H×W×3 RGB
uint8 NumPy array at its original resolution. Return an N×5 array of
[x1, y1, x2, y2, confidence], in original image pixel coordinates. Coordinates use
continuous exclusive upper bounds: 0 <= x1 < x2 <= W and 0 <= y1 < y2 <= H.
Confidence is finite and in [0,1]. Return shape (0,5) when no UAV is present.
Return at most 300 boxes. Do not return category or track-ID columns.

The grader starts a fresh model process for each sequence and calls reset().
predict may use past frames; it receives no paths, labels, future frames or
sequence IDs. Do not train, download, or update learned parameters during inference.
The initialization deadline is 120 seconds; each prediction must return within
10 seconds, within an overall 6000-second inference budget. Invalid output, missing
artifacts, crashes or timeouts receive zero. These limits are resource constraints,
not parts of the accuracy metric.

## Local evaluation

The minimal model interface is in ~/example/model.py. It deliberately has no
training recipe. After creating a submission, run:

```bash
python ~/practice_grader/grade.py --submission /home/user/submission --data /opt/uav-data/val
```

The primary metric is single-class COCO AP at IoU 0.50, averaged at 101 recall
thresholds, with at most 300 detections per image. AP50:95 and small-object AP are
diagnostics. Every selected frame, including negatives, contributes to evaluation.
Raw AP50 is in [0,1]; exact ground-truth matches without extra detections attain 1.0.
The benchmark reward equals raw AP50: ground-truth predictions score 1.0 and empty
predictions score 0.0. Trained baseline scores do not rescale your score.
The engine rounds its serialized primary reward to two decimals; full-precision
AP50 and reward_exact remain in the verification metrics.
Training logs alone do not prove that training occurred; your source and run
trajectory must support reproduction.
