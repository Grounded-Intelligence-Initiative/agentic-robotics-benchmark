# Trained baseline and reproduction

The baseline trains during the agent stage. `solve.sh` copies the source files into
`/home/user/submission` and runs `train.sh`; setup does not train, download data,
or clear existing outputs. The tested agent receives an interface and public
data, not this recipe.

The reference is Faster R-CNN with a ResNet-18 FPN, frozen BatchNorm and frozen
early backbone layers. It starts from generic ImageNet initialization and trains
with AdamW, sequence-balanced sampling, positive-biased multiscale crops and
photometric augmentation. `train.sh` runs a 24,000-step warmup/cosine schedule,
then selects its 12,000-step checkpoint. A run with a 12,000-step schedule would
be a different experiment. `export_checkpoint.py` preserves the full training
log and the selected checkpoint's exact log prefix, configuration and hashes.

Inference expands each predicted box about its center by a fixed factor of
1.75, clips it to the current image and applies CPU NMS at IoU 0.3. The same
global rule applies to every frame. It uses no sequence identifiers, labels,
future frames or per-image fitted constants. These parameters and the checkpoint
were selected using public validation only; annotations were not changed.

## Development evidence

Native DCC job 62722505 trained 24,000 steps in 1,155.71 seconds on an RTX 5000
Ada with a 24 GiB CUDA allocator cap. Its selected checkpoint has SHA256
`3853cd1caf386eef5b6134d130d9c0b1c7c84b9e54f62ca241ff62826d9ac8c5`.
The public validation set contains 1,190 frames from clips 37 through 40.

CPU job 62723478 measured AP50 0.1200373361 before calibration and 0.3298975571
after box scale 1.75 / NMS 0.3. A follow-up using the identical cached predictions
tested larger scales 2.0, 2.25, 2.5 and 3.0; all reduced AP50. The selected
postprocessing's replay matched that experiment exactly, with zero coordinate
difference. Job 62723806 then ran the exact selected `Detector.predict()` over all
1,190 public validation frames in 569.89 seconds on CPU. It reproduced all four
reported metrics exactly, including AP50 0.3298975571; the job completed with exit
code zero. Source/weight hashes were recorded with that result.

Job 62727576 subsequently executed the complete packaged training recipe from
generic ImageNet initialization in the final agent Docker-derived SIF on an H200.
All 24,000 steps completed in 1,207.57 seconds. Its selected 12,000-step checkpoint
has SHA256 `972cb29f7a9e47bb9eb65067cc0838b06dd9702d9c6ca7ad063f601462270e7e`.
Public CUDA inference over the same 1,190 frames measured AP50 0.3639820186.
The full training log, selected prefix and executed source hashes were verified.
This is a reproduction of the same recipe, not an autonomous agent episode.

That fixed checkpoint was then evaluated once on all 2,475 held-out frames in the
separate verifier Docker image (job 62728682). The author adapter changed only
the inference device to CPU; root/user and network isolation remained active.
Raw AP50 was 0.5451401442 and AP50:95 was 0.1464820651. These measurements did not
change the recipe, checkpoint selection, anchor or scoring settings. The reported
reward remained zero because the official anchor was null. Do not treat this
CPU check as GPU/ALE admission evidence or use it for further model selection.

The native run, cached replay and CPU checks are author development evidence.
They do not establish CUDA execution in the isolated ALE verifier or formal
frontier-agent difficulty. The task now uses the measured GT endpoint (AP50=1)
as its fixed scoring reference. These baseline measurements do not set the anchor
or full_at. See tests/gt_reference_measurement.json for the endpoint evidence.

## Reproduce in the task image

Run `bash solution/solve.sh` to reproduce the trained baseline. Training needs
the public data and generic initialization installed by the image, plus CUDA.
The final artifacts are `model.pt`, `model.py`, `train.sh`, training sources,
`config.json` and `training_log.jsonl` under `/home/user/submission`. Public
practice grading and the private verifier both report raw AP50 as the score.

The learned reference is not a perfect detector. `solve.sh` exports its actual
learned weights, without GT buffers or per-image answer lookup. `run_baseline.sh`
is an alias for the same entry point. The metric endpoint check lives separately
in tests/metric_selfcheck.py and is not an admissible model submission. With the
fixed AP50 scale, the current reference does not meet ALE's oracle reward=1 rule;
this admission-contract issue remains open. No hidden-score tuning or baseline
normalization is used to manufacture a pass.

## Fixed baseline repetitions

DCC job 62755551 completed three fresh runs of this unchanged recipe on H200,
with scheduler exit 0:0. All trained for 24,000 steps, selected the 12,000-step
checkpoint, and evaluated every public/hidden frame. There was no selection
among repetitions and no subsequent hidden-score tuning.

| Trial | Training seconds | Public AP50 | Hidden AP50 |
| --- | ---: | ---: | ---: |
| 1 | 1709.3332 | 0.3582768621 | 0.5010793326 |
| 2 | 1681.5912 | 0.3684237752 | 0.5382002540 |
| 3 | 1677.4463 | 0.3419622896 | 0.5120647472 |

Hidden mean AP50 is 0.5171147779; sample standard deviation is 0.0190687653.
Final model hashes match recorded selected checkpoints, and each selected log is
the exact 12,000-step prefix of its complete 24,000-step log. These are author
repeatability measurements, not a score anchor or official ALE oracle results.
