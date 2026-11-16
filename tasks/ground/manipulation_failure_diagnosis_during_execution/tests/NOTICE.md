# NOTICE - sources of this task's assets (verify / maintainers only; never shown to the agent)

**Kitchens and objects.** The scenes in every recording come from **RoboCasa** (S. Nasiriany et al.,
https://robocasa.ai): the Hugging Face dataset `robocasa/robocasa-assets` (objects `objaverse/`, `aigen_objs/`,
fixtures, textures) and the NVIDIA dataset `nvidia/PhysicalAI-Robotics-Manipulation-Objects-Kitchen-MJCF`
(`lightwheel/` objects and fixtures). Both datasets are tagged **CC-BY-4.0** on Hugging Face (read from the Hub API on
2026-11-09). This notice is the attribution the licence asks for. The upstream licences of the individual
Objaverse-derived objects were not checked one by one; some Objaverse objects carry other licences.

**Recordings.** The videos and robot logs are rollouts of the frozen XR-1 policy (Xiaomi-Robotics-1, Apache-2.0) in
RoboCasa365 kitchens, recorded by the task author:
- practice runs: from the author's earlier TRACE-MM recordings (`composite-failures-v1`, `xr1-failures-v2`), which are
  also public in the Hugging Face dataset `xzhang975/trace-mm-task1-xr1-failures`; re-encoded (x264 crf 26, metadata
  stripped) and given opaque ids here; plus practice runs of PickPlaceDrawerToCounterNear (a near-object variant of a
  RoboCasa task) recorded for this task on 2026-11-13 and 2026-11-16 with new seeds, not part of that dataset;
- hidden runs: recorded for this task from 2026-11-11 to 2026-11-14 with new seeds, not part of that dataset.
The recordings are rendered from the CC-BY-4.0 assets above and are offered under **CC-BY-4.0** with this attribution
(licence choice to be confirmed by the task owner). No XR-1 weights or code are shipped, and no policy runs in this
task: the grader plays recordings back.

**Contamination risk.** Most practice runs are already public, and once the payload is uploaded the hidden runs are in
the public asset dataset too, while their answer key is in this public repository (`tests/hidden/key.json`). A model
trained on both could have seen the answers. The tested agent cannot reach either during a run (no network; the hidden
runs are only in the verifier image), so this is a risk for future measurements, not an in-sandbox leak.

`/opt/models/clip-vit-base-patch16` is OpenAI's CLIP ViT-B/16 (openai/clip-vit-base-patch16 on Hugging Face at
57c21647), MIT licence for the code; the model card states no separate licence for the weights, as for the merged
tasks that ship the same files. Files mirrored unmodified.
