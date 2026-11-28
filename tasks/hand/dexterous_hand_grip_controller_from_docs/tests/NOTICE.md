# Sources and credits of dexterous_hand_grip_controller_from_docs

This note says where the hand of this task and the documents the agent gets come from. The agent never sees this
folder: every file the agent gets calls the hand "the hand" and its bought parts by generic names and invented part
numbers. Recorded on 2026-11-24 from the sources themselves; nothing here is a legal opinion.

## The hand

The hand is the **MIDAS Hand**, right hand, an open-source four-digit robot hand by Alvin Zhu, Mingzhang Zhu, Beom Jun
Kim, Quanyou Wang, Jose Victor S. H. Ramos and Dennis Hong (University of California, Los Angeles), published at
https://midas-hand.com/. It is used exactly as its designers released it: thirteen bus servos, a crossed four-bar per
finger that couples the last joint, four tactile modules. There is no hidden variant of the hand; the grader's model
is built from the designers' own geometry and masses.

## Sources, with what each source says about its licence

| Source | Taken from | Version | Licence as stated by the source |
|---|---|---|---|
| The hand's hardware design (CAD, mechanical descriptions, BOM) | https://midas-hand.com/ (its licensing section) | as downloaded on 2026-11-20 (UTC) | CERN-OHL-P-2.0 ("Applies to CAD, STEP, 3MF, URDF/mechanical descriptions, BOM, and assembly design materials") |
| The designers' hand software: the API, its tactile code and taxel maps, the hand-off note, the test procedures and the endurance and impact results | https://github.com/midas-hand-org/midas_hand_api, branch `ICRA_test` | commit `82e4ec77ff6c3c4c21b1730e99b42c98f8769095` (2026-09-15) | MIT, Copyright (c) 2026 MIDAS Hand Contributors |
| The simulation model and meshes the grader's model is built from | https://github.com/midas-hand-org/midas_hand_mujoco | commit `6c7d17f4a8cb5414e81653aa2795acc9610e1730` (2026-05-29) | MIT, Copyright (c) 2026 MIDAS Hand Contributors |
| The servo's product page and control table (ROBOTIS e-manual, XM335-T323) | https://emanual.robotis.com/docs/en/dxl/x/xm335-t323/ | the page as downloaded on 2026-11-20 (UTC; sha256 prefix `47cdfda12bf14917`) | (c) 2026 ROBOTIS |
| The tactile modules' product pages | https://www.paxini.com/ (the sensor product pages) | the pages as downloaded on 2026-11-20 (UTC) | Copyright (c) 2026 PaXini Tech. All rights reserved |
| The project website's text | https://midas-hand.com/ | as downloaded on 2026-11-20 (UTC) | CC BY 4.0 |

## What this task makes of the sources

| Material | Where | How it derives from the sources |
|---|---|---|
| The servo manual, the bus notes, the performance table | the payload: `given/servo/` | The vendor's page text re-issued with neutral names (its numbers and sentences unchanged; links removed); the performance graph transcribed into a table and labelled as transcribed. |
| The tactile interface note, the module parameter table and the taxel maps | the payload: `given/tactile/`, `given/hand/taxel_maps/` | Facts of the designers' tactile code and of the module vendor's table, written for this task with neutral names; the taxel maps are the designers' CSV files with neutral file names. |
| The designers' hand-off note, test procedures and results | the payload: `given/makers/` | Re-issued with neutral names, their numbers and sentences unchanged (people's names, links, paths and commit ids removed). |
| The kinematics sheet and the link meshes | the payload: `given/hand/` | Written for this task from the grader's model: joint axes, the four-bar's link lengths, pad frames, limits, masses; one mesh per link, the union of that link's convex pieces of the designers' meshes. |
| The grader's model of the hand | `grip/plant/model/` | The designers' meshes as convex pieces, every name neutral, the four-bar loops closed, the masses of the designers' model. |
| The unit records, the cell's documents, the specimen sheets, the acceptance specification and the controller's interface | the payload | Written for this task; the hidden units and jobs are drawn for this task and are not the designers' data. |

Modification notice (CERN-OHL-P-2.0, section 3.3): the files of this task that derive from the hardware design (the
grader's model and its convex pieces, the link meshes and the kinematics sheet) were made from the designers'
released model in November 2026 by renaming every part, splitting the meshes into convex pieces, closing the
four-bar loops and measuring them. The geometry of no part was redesigned.
