# Validation results

These are author measurements on NPS train/validation/held-out clips, preceding
this documentation cleanup. Runtime algorithms and scoring are unchanged.
They are not formal ALE validation records for this submission.

| Check | Result |
| --- | --- |
| Agent and verifier Docker builds | Passed |
| Docker ordinary-user startup and hidden-label permissions | Passed |
| Docker CPU verifier: missing / truncated / privileged GT submission | 0 / 0 / 1 |
| Full H200 reference: 24,000 training steps, checkpoint export, GT packaging | Completed; held-out AP50=1 |
| Docker-derived Apptainer on H200: CUDA optimizer steps and 1,190 public frames | Passed |
| Formal GPU `ale validate` | Incomplete: Docker unavailable on the GPU host |

The Docker CPU tests used the normal verifier with real root/user and network
isolation. H200 author runs used Apptainer and do not establish the complete ALE
sandbox lifecycle. H200 is tested; RTX 4090 is allowed but untested.

## Learned detector results

Three fixed baseline runs scored held-out AP50 of 0.5010793326, 0.5382002540 and
0.5120647472 (mean 0.5171147779). All trained for 24,000 steps and selected the
12,000-step checkpoint without using held-out scores for selection.

One independent function-calling run of `gemini-3.8-flash` (high effort, four-hour
budget, H200, offline Apptainer tools) trained for four epochs and produced 12,986
step/loss records. Public AP50 was 0.5659924301; held-out AP50 was 0.5974065114 and
AP50:95 was 0.1917051914 on 2,475 frames. Hidden feedback was withheld from the agent.
Its final config omitted an explicit dataset-version identifier; artifacts were
not amended. This run was outside `ale run` and is above the repository's
approximate 0.5 difficulty target. Formal benchmarking and difficulty review remain
pending. The trained baseline does not determine the score normalization.
