# Author evidence and remaining acceptance work

The scoring contract is GT=1, empty=0, reward=raw AP50. The aggregate endpoint
measurement in gt_reference_measurement.json covers all 55,616 frozen frames.
The learned baseline is not the denominator. Exceeding it does not give full score.

Fresh Google job 62755066 completed on DCC with scheduler exit 0:0. Model
`gemini-3.8-flash`, high effort, received fresh history, the four-hour budget and
only public task inputs. It wrote its own Faster R-CNN/ResNet-18 FPN training
source, used generic ImageNet initialization, and trained with AdamW for four
epochs (12,986 step/loss records). Final public AP50 was 0.5659924301; a frozen
model then scored AP50 0.5974065114 / AP50:95 0.1917051914 on all 2,475 hidden
frames. No hidden feedback reached the agent. Its final source matches emitted
tool commands and all final artifact/complete-tool-log hashes match.

Model SHA256: e5dee6e8dfe69077ef18e7adc8b8f175ba2f8cc7acd7c4c16b349da013edebf5.
Agent duration: 5,858.65 seconds; model/tool calls: 129/128. Observed peak GPU
memory: 14,533 MiB (sampled, not a hard partition). Final config lacks an explicit
NPS version identifier required by the instruction; artifacts were not amended.

The original interrupted pilot (62742608) remains recorded: raw hidden AP50
0.3701426269, scheduler OUT_OF_MEMORY / 0:125, control-plane pause and HOME repair.
It must not be described as a clean trial. Three fixed baseline repetitions
completed with exit zero; all results are in solution/REFERENCE.md.

These experiments used source 68dbd88629c811da1267bd13313072092de5dbf9. Their
raw metrics are unchanged by the later GT scoring revision, but the experiments
are not new ALE verdicts for that revision. They ran on H200 with an independent
function-calling harness and Apptainer, not the declared RTX4090/Docker ALE stack.

Formal GPU ALE lifecycle/UID isolation and a formal frontier episode remain
unverified. The current solution/solve.sh submits only a learned detector. Its
measured AP50 does not satisfy the pinned engine's oracle=1 admission rule;
that contract requires maintainer resolution without rescaling AP50 or submitting
hidden answers. The author result
0.5974 is above the repository's approximate frontier guideline of 0.5, but is
not a full score. Report this candidly without tuning on hidden data, shortening
the declared budget or rescaling the score to manufacture an acceptance result.

## Historical GT-only grader endpoint evidence

The following results used the retired GT-packaging implementation. It looked up
hidden boxes by pixel hash after training, so its perfect predictions were not
learned. That implementation and its hidden-answer fixture have been removed from
the current task. These immutable runs validate scoring endpoints only; they do
not describe the current solve.sh or establish learned-reference success.

### Real Docker verification

DCC job 62774192 completed with scheduler exit 0:0. The exact existing verifier
image sha256:7fab623064e3b756800f93e83207c0b34b5bb63ee1e24270d7a0c87c75668f21
ran the unmodified verify.py and worker at that recorded revision. No device or score override
was installed. Missing artifacts scored 0; a truncated checkpoint scored 0; the
GT submission scored AP50=1 and reward=1 over all 2,475 held-out frames in
203.38 seconds. All three containers exited zero. Real root/user separation,
root-only test labels and the verifier's network-denial probe remained in effect.

This CPU check packaged GT with the unchanged first trained baseline from job
62755551; its original weights and logs remained intact. It is distinct from the
fresh full solve.sh training job 62774159. Neither result is a claim of the
formal GPU Docker ALE lifecycle, and no learned detector is claimed to score1.

### Historical full GT execution

DCC job 62774159 executed the complete solve.sh at source
9c19c6a096c34ff034cc018a8b3715b6a2f5ec22 and completed with scheduler exit 0:0.
Actual 24,000-step training, selected 12,000 checkpoint export and GT packaging
finished in 1,184.85 seconds. Normal worker inference then processed all 2,475
frames and returned 3,491 GT detections in 88.05 seconds. AP50, AP50:95, APsmall,
AR300 and reward were exactly 1; pre/post evaluation artifact hashes were unchanged.
This is an H200/Apptainer author run, not the formal GPU Docker ALE lifecycle.
