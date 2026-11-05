# Original-design restoration — draft task, not ready for evaluation

The owner requires original AGIPIX sensor geometry, structure, dimensions and assembly relationships to be preserved
(2026-10-29). The prior synthetic reference used fictional sensor/electronics shapes, changed plate dimensions and new
mounts. That version is superseded. Its Astra score 0.3514282007 at `af93954` is historical, not an original-AGIPIX result.

The upstream repository's `cad/README.md` links https://a360.co/4gY1h8M. The public share enables downloads. Original
`Agipix_v2`, version `2`, has now been exported to STEP and imported/saved/reopened in FreeCAD: 132 valid closed leaf
solids. This includes original MID-360, D455 (`RS455` in source), electronics, connectors, props and mounting structure.

- STEP size: 106,421,450 bytes.
- STEP SHA-256: `f13c51d63c1372c830553803be94ee616bfabfc8cfd1cc4d19981156e7ec878f`.
- Original product contains named B-rep items; solid count is not physical-part count.
- Manufacturing CAD and Git simulation USD are separate versions. They must not be silently mixed.
- Reference CAD and restoration evidence: https://drive.google.com/drive/folders/1CydqUXWpm8wf0XxpoUpmpKkzSyf1LVGT.

Hardening must assess existing hole/slot/pocket geometry, mating axes, sensor pose, retention and source-supported
clearances. No replacement sensor boxes, new holes/ribs/mounts, changed thickness, folding mechanism or gimbal is
authorised. Correct equivalent-instance matching and use precision-supported bands; the score target does not justify
altering hardware or hiding required observations.

The current instruction, public payload and grader still describe the superseded synthetic v1. This PR remains draft:
replace those together using a reviewed original-parts/module registry, regenerate photographs/component files, and
validate the new native oracle and grader before launching or claiming a source-faithful evaluation. Do not combine
the restored source reference with v1 photographs or fictional datasheets. The pending asset manifest already blocks
canonical publication. No new model score or ≤0.10 result is claimed.
