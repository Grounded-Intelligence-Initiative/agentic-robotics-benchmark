# Sources and credits of dexterous_hand_robot_description_from_photos

This note says where the hand of this task and its other material come from. The agent never sees this folder: every
file the agent gets calls the hand "the hand", its bought parts by generic names and invented part numbers, and its
servo by an invented catalogue code. Nothing here is a legal opinion.

## The hand

The hand is the **MIDAS Hand**, right hand, an open-source four-digit robot hand by Alvin Zhu, Mingzhang Zhu, Beom Jun
Kim, Quanyou Wang, Jose Victor S. H. Ramos and Dennis Hong (University of California, Los Angeles), described in
"MIDAS Hand: Modular low-Impedance Direct-drive Anthropomorphic Sensing Hand" (arXiv:2607.14487v1) and published at
https://midas-hand.com/. Its design is used exactly as its designers released it. The unit of this task is one build
of that design with states of its own (where each output horn was fitted, its ends of travel by contact of the
released parts, its masses from the released print settings, its calibration), authored for this task.

## Sources, with what each source says about its licence

| Source | Version | Licence as stated by the source |
|---|---|---|
| The release CAD of the whole hand, `Right_Hand_Assembly.step` (the project's CAD page) | sha256 `228f76b548ed71e754d8ab69e247e42ecf66caff9e8bbacc8462e63b9106c5a7` | CERN-OHL-P-2.0 |
| The print projects of the made parts (the project's CAD page, ready-to-print files) | as published on 2026-11-20 | CERN-OHL-P-2.0 |
| The assembly guide ("MIDAS Assembly Instruction") | as published on 2026-11-20 | CERN-OHL-P-2.0 (assembly design materials) |
| The host software, https://github.com/midas-hand-org/midas_hand_api | commit `6a08263fc607bf219a6b23a9afa5795e3863255a` | MIT, Copyright (c) 2026 MIDAS Hand Contributors |
| The paper, https://arxiv.org/abs/2607.14487 (Table II, the joint range of motion) | v1 | CC BY 4.0 |
| The servo maker's product page (ROBOTIS e-manual, XM335-T323): specification, performance graph, control table | the page of 2026-11-20 | (c) ROBOTIS |
| The servo maker's mass-properties sheet of the XM335 (the mass) | as downloaded on 2026-11-20 | (c) ROBOTIS |


## What this task makes of the sources

| Material | Where | How it derives from the sources |
|---|---|---|
| The part files | the payload: `parts/meshes/`, `parts/assembly.json`, `parts/bom.csv` | The solids of the release STEP as triangle meshes (one per made part, given part type and fastener type) in each part's own frame, and every instance's placement; no face changed. The servo's file is the designers' one-solid servo model. |
| The print settings | the payload: `parts/print_settings.csv` | The settings of each made part's object in the designers' print projects (the object's own setting, else the project's), with the material by its generic name |
| The servo pages | the payload: `docs/servo_VKR-4LBA/` | The maker's specification, notes and control table re-issued under an invented code; the performance graph drawn again from a digitised copy of the maker's graph |
| The design table, assembly guide and host-software excerpts | the payload: `docs/` | The paper's range-of-motion table, the guide's servo steps and the host software's README and code, re-issued with neutral names |
| The photographs, records and lab documents | the payload | Authored for this task: path-traced renders of the release geometry, the bus records of the authored unit, the lab's bench record, handover note and weigh slip |

Modification notice (CERN-OHL-P-2.0, section 3, its third sub-section): the files of this task that derive from the hardware design files
(the part meshes and placements, the print settings, the photographs, the grader's model of the unit) were made from
`Right_Hand_Assembly.step` and the print projects in November 2026 by taking each part family out of the assembly,
converting its format and measuring it. The geometry of no part was changed.
