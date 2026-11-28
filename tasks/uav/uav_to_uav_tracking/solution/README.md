# Reference and baseline

`solve.sh` is a privileged ground-truth reference used to check scoring. It first
trains the baseline, preserving its weights, source and real training logs under
`submission/baseline/`, then packages exact reference boxes in `model.pt`.
`gt_model.py` identifies frames by their RGB pixels and returns the corresponding
boxes through the normal inference worker. Unknown frames fail. The grader has
no oracle-specific scoring branch. A perfect reference score is not learned accuracy.

Evaluated agents receive neither `solution/` nor the hidden labels. To reproduce
the learned detector without ground-truth packaging, run `bash run_baseline.sh`.
It trains Faster R-CNN/ResNet-18 FPN from the provided ImageNet initialization for
24,000 AdamW steps, then exports the 12,000-step checkpoint of that schedule.
Checkpoint selection and fixed postprocessing use public validation data only.
The CUDA allocator is capped at 24 GiB. Output files follow `instruction.md`.

Generate the privileged fixture from the pinned private dataset with:

```bash
python solution/prepare_gt_fixture.py /path/to/test solution/gt_fixture.json
```

`prepare_release.py` reproduces the JPEG/COCO export from the original videos;
`prepare_data.py` supports existing extracted PNG frames. Source dataset licence
terms are retained in `tests/nps-LICENSE.txt`. Validation results are in
`tests/RESULTS.md`.
