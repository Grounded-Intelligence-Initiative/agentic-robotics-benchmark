# Design table (excerpt)

An excerpt of the hand designers' published tables, re-issued with this task's names. The hand has 16 hinges: 13 are
driven by one servo actuator VKR-4LBA each; the last hinge of each finger has no servo and is coupled to the hinge
before it (the PIP axis of the same finger) through a crossed four-bar.

## Servo ids, axes and travel

The servo ids are set at assembly (`assembly_guide.md`, "Set motor ID") and are the ids the host software uses
(`motor_ids` 0 to 12, `host_software.md`). The contract name is the name of the hinge in the hand's MuJoCo
description (`contract/contract.md`). The travel column is the designers' range-of-motion table, in degrees; the DIP
rows are the passive hinges.

| servo id | digit | axis | contract name | travel (deg) |
|---|---|---|---|---|
| 0 | thumb | IP axis | `thumb_ip` | [-90, +90] |
| 1 | thumb | MP axis | `thumb_mp` | [-90, +90] |
| 2 | thumb | side axis | `thumb_side` | [-45, +52] |
| 3 | thumb | roll axis | `thumb_roll` | [0, +123] |
| 4 | index finger | PIP axis | `index_pip` | [-83, 0] |
| 5 | index finger | MCP axis | `index_mcp` | [-103, 0] |
| 6 | index finger | ab/ad axis | `index_abad` | [-45, +45] |
| 7 | middle finger | PIP axis | `middle_pip` | [-83, 0] |
| 8 | middle finger | MCP axis | `middle_mcp` | [-103, 0] |
| 9 | middle finger | ab/ad axis | `middle_abad` | [-45, +45] |
| 10 | ring finger | PIP axis | `ring_pip` | [-83, 0] |
| 11 | ring finger | MCP axis | `ring_mcp` | [-103, 0] |
| 12 | ring finger | ab/ad axis | `ring_abad` | [-45, +45] |
| none | index finger | DIP axis | `index_dip` | [-107, 0] |
| none | middle finger | DIP axis | `middle_dip` | [-107, 0] |
| none | ring finger | DIP axis | `ring_dip` | [-107, 0] |

## Direction rule

Joint directions follow the right-hand rule, with the thumb pointing along the servo's output-horn axis. The host
software's default joint sign is +1 for every servo.
