# NOTICE - sources of this task's data (verify / maintainers only; never shown to the agent)

- **Events.** The IEDD-VQA test split of IEDD: H. Feng, X. Zhang, M. Tian, P. Zhang, Z. Li, J. Huang, X. Wang,
  J. Zhu, J. Wang, D. Yin, L. Xiong, "An interactive enhanced driving dataset for autonomous driving",
  arXiv 2602.20575 (v2, 2026-07-18). Files of the Hugging Face dataset `Egikk/IEDD` at commit TODO(author), which
  are the files of Zenodo record 10.5281/zenodo.18742437 (v1, 2026-02-23, CC BY 4.0): `IEDD-VQA_test.json` (one
  sample per clip, with IEDD-VQA's own answers), `IEDD-VQA_test_videos.zip` (the clips) and `IEDD-waymo.zip` (the
  `interaction.csv` annotations; every sample of the split is from Waymo Open Motion, `waymo_train`).
- **Reference code.** `environment/Dockerfile` fetches `utils/IEDD_compute.py`, `utils/data_models.py`, `utils/para.py` and
  the Apache-2.0 `LICENSE` of GitHub `egik-von/IEDD` at commit 0a530a0e, checks each against its sha256, and the agent
  gets them as `iedd_reference/`.
  The answers follow this code, which differs from constants printed in the paper (for example d_0 = 8 and
  beta_Q = 0.4 in the paper, `D0 = 15` and `BETA_Q = 0.5` in the code, and the code squashes each component of Q).
- **Terms.** IEDD releases its derived annotations and renderings under CC BY 4.0 (Zenodo) and does not redistribute
  raw source data. TODO(author): the maintainers confirmed on <date> that re-hosting these CC BY 4.0 renderings in
  the benchmark's asset dataset is acceptable alongside the Waymo Open Dataset terms.
- **Selection.** `tests/review/build_task_data.py` (seed 20261125): pairwise rows (Car-follow, Merging, Crossing,
  Head-on) with an intensity series, no other row of the scene for the same pair and interval, a clip, and IEDD-VQA
  answers that agree with the row (the same two vehicles, the same type, a peak within 0.006 of the row's); 24 test
  events and the labelled events from disjoint sets of scenes, at most two per scene, round-robin over the types and
  over equal-width bins of the peak. `ref/source.json` maps every event to its IEDD scene, row, interval, vehicles,
  IEDD-VQA sample id and clip.
- **Why the pair is given and efficiency is not asked.** A clip draws one vehicle in red and the others in black, and
  the same red vehicle over the same interval often has several partners (328 of the 374 pair events of the GitHub
  sample), so `clips.json` names both vehicles. IEDD computes the efficiency E_i over a vehicle's whole track in the
  scene ("E over full lifecycle" in `IEDD_compute.py`), which a clip of the event's interval does not show.
- **Answers.** `ref/answers.csv`: IEDD's interaction type, and the largest `Q_i_Over_Time` value of either vehicle over
  the interval at four decimals (IEDD-VQA's "Q peaks ... with Q =" answer; IEDD's Q_i(t) of a vehicle is the largest
  over all of its interactions at t). IEDD-VQA's tasks L1 (interacting vehicle and type) and L3 (physical
  quantification) ask the same from a VLM; the paper reports zero-shot VLMs far from them.
- **Where they live.** The clips and the labelled answers are in the benchmark's Hugging Face dataset under
  `tasks/ground/bev_interaction_quantification/payload/`, fetched at the commit pinned in `environment/assets.sha256`
  (docs/task-assets.md). The answers of the test events are only in `tests/ref/` and `solution/ref/`.
