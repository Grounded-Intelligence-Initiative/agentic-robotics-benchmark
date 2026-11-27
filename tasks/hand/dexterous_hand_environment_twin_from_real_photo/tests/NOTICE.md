# Sources and credits of dexterous_hand_environment_twin_from_real_photo

This note says where the photographed scene, its truth and the hand of this task come from. It is a maintainer file:
the agent never sees this folder, and no file the agent gets names the dataset, the hand, its designers or their
school. Recorded in November 2026 from the sources themselves; nothing here is a legal opinion.

## The scene

The photograph, the camera's intrinsics, the scans of the ten things and their tracked poses come from the
**HouseCat6D** dataset: HyunJun Jung, Guangyao Zhai, Shun-Cheng Wu, Patrick Ruhkamp, Hannah Schieber, Giulia Rizzoli,
Pengyuan Wang, Hongcheng Zhao, Lorenzo Garattoni, Sven Meier, Daniel Roth, Nassir Navab and Benjamin Busam,
"HouseCat6D - A Large-Scale Multi-Modal Category Level 6D Object Perception Dataset with Household Objects in
Realistic Scenarios", Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR) 2024,
pages 22498-22508 (arXiv:2212.10428; https://sites.google.com/view/housecat6d; https://github.com/Junggy/HouseCat6D),
Technical University of Munich. The dataset is released under **CC BY 4.0** (Creative Commons Attribution 4.0
International, https://creativecommons.org/licenses/by/4.0/), as its repository states. This notice is the
attribution the licence asks for. The material is provided by its licensors as-is, without warranties.

- **Copy used:** the dataset's own server, http://www.campar.in.tum.de/public_datasets/2023_housecat6d_new/, read in
  November 2026 by HTTP range requests from `obj_models.zip` (the ten things' scans) and `test_scene.zip` (scene 4
  of the test split: the colour image of frame 339, that frame's camera pose, the scene's object poses, its camera
  intrinsics, its class list and that frame's occlusion record).
- **Agent-visible:** `photo.png` (the colour image of frame 339 of that scene, byte for byte unchanged; in
  development in `environment/setup/payload/`, published through the benchmark's asset repository) and the camera's
  focal lengths and principal point rounded to two decimals (`camera_sheet.json`). The givens never name the source;
  the things are called by plain names.
- **Modifications (the truth in `tests/gt/` and the reference twin in `../solution/answer/`):**
  - the support is the plane through the things' lowest points, with a frame whose x axis is the corrected camera's
    line of sight projected on it; the poses are the dataset's tracked poses expressed in that frame, unchanged;
  - each thing's collision shape is a set of convex pieces cut from its scan, which never stand proud of the scan
    where the thing touches the support or another thing; the scans' surface samples and occupancy grids, at
    millimetre spacing, are derived from the scans in the same frame;
  - the masses (the scans' volumes at one nominal density) and the frictions are this task's own placeholders, used
    only for the rest check and the lifts and never scored;
  - which pick requests are scored, and how, was computed by this task's own instrument on the reference twin and on
    copies of it moved by the trackers' stated error;
  - the reference twin is the same pieces at the tracked poses on a board of this task's own.
- **Attribution of the adaptations:** these adaptations (`tests/gt/`, `../solution/answer/`, the photograph in the
  asset repository) are shared with this source and modification note; the rest of the task is the benchmark's own.

## The hand

The hand is the **MIDAS Hand**, right hand, an open-source four-digit robot hand by Alvin Zhu, Mingzhang Zhu, Beom Jun
Kim, Quanyou Wang, Jose Victor S. H. Ramos and Dennis Hong (University of California, Los Angeles), described in
"MIDAS Hand: Modular low-Impedance Direct-drive Anthropomorphic Sensing Hand" (arXiv:2607.14487v1, submitted
2026-07-16) and published at https://midas-hand.com/. It is used exactly as its designers released it; nothing of
the hand is modelled again, scaled or simplified for this task.

| Source | Taken from | Version | Licence as stated by the source |
|---|---|---|---|
| The release CAD of the whole hand (STEP, 19,364,138 bytes) | the download link of the project's CAD page | the designers' STEP export of 2026-05-28, sha256 `228f76b548ed71e754d8ab69e247e42ecf66caff9e8bbacc8462e63b9106c5a7` | CERN-OHL-P-2.0 |
| The project's website | https://midas-hand.com/ | as published in November 2026 | website text and media CC BY 4.0; hardware materials CERN-OHL-P-2.0; software MIT |
| The paper | https://arxiv.org/abs/2607.14487 | v1 | CC BY 4.0 |
| The servo maker's CAD model of the hand's servo, ROBOTIS XM335 (`XM335.stp`) | https://www.robotis.com/service/download.php?no=2236 | sha256 `df2e22d7110e8de1db42a71e409d6cc3984195fcdbb0144b79e0415441b407e7` | the maker's own CAD download |

What this task makes of them: the fingertip volumes of the hand card and the finger pad box of the pick instrument
are the extents of the hand's links, measured on the posable model of the hand that the hand tasks share (built from
the release CAD, with each servo drawn from its maker's CAD model at the designers' placement).

Modification notice (CERN-OHL-P-2.0, section 3.3): the volumes of the hand in this task's payload (`hand_card.md`,
`instrument_config.json`) were made from the release CAD in November 2026 by measuring the extents of the links'
solids. The geometry of no part was changed.
