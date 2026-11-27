# NPS UAV-to-UAV detection release

Source: the [official Purdue UAV dataset page](https://charlesbouman.org/UAV_Dataset/)
and its 50 original videos. The page explicitly links the BSD 3-Clause distribution
license saved as LICENSE.txt. Cite Jing Li et al., "Fast and Robust UAV to UAV
Detection and Tracking from Video," IEEE TETC 10(3), 1519–1531, 2022.
The public image retains this notice at `/opt/uav-data/LICENSE.txt`; the separate
verifier image retains it at `/opt/uav-heldout/LICENSE.txt`, copied from
`tests/nps-LICENSE.txt`. Task code and the canonical shared grader retain the
benchmark's Apache-2.0 licence and notice in `tests/licenses/APACHE-2.0.txt` and
`tests/licenses/NOTICE.txt` of the source task package.

This release uses the unversioned original `Video_Annotation.zip`, SHA256
`abc9110cd287d76d2c6ed70f1163386d881bd567e39930431d393a2b383271b1`.
The source video archive SHA256 is
`84f0878dfb61a1ecd9b4f865e4913debdda7dd3e4eabf08a06e33cfc3c2fd998`.
Updated v1/v2 annotation archives were preserved during acquisition but are not mixed
into this task. No Dogfight-format or previously converted YOLO labels are used.

## Public data

- Training: complete source clips 1–36, every frame.
- Validation: complete source clips 37–40, source indices 1,6,11,... .
- The benchmark evaluates additional source sequences unavailable in the agent image.

Frames retain original resolution and are decoded using PyAV 15.1.0, then stored
as RGB JPEG with Pillow 11.3.0, quality 95 and subsampling 0. This is a lossy image
encoding; validation and evaluation use the same encoding. Source indices and original
`time_layer` keys are both 1-based. Original tuples are `(y1,x1,y2,x2)`, converted to
COCO continuous `(x,y,width,height)`, clipped to image bounds; degenerate boxes are
discarded. Every decoded frame must have an annotation entry, including empty entries.

Each split contains `annotations.json`, `images/` and `images.sha256.json`. COCO images
include `sequence_id` and `frame_index`, and the only foreground category is UAV
(`category_id=1`). Empty frames are valid negatives and must not be dropped during
evaluation. Chronology is defined by `frame_index`, not filesystem iteration order.

Splits are fixed by complete source clip, before reference training. This prevents
adjacent frames of the same clip entering different splits. Exact cross-split image
hashes are audited separately; that check alone does not establish absence of related
flights, repeated scenery or near-duplicate content across source clips. The underlying
research corpus is public; the operational holdout is enforced by sandbox isolation
and the no-network/external-data rules, not by a claim of globally unpublished imagery.

Optional initialization is the generic torchvision ResNet-18 ImageNet checkpoint
at `/opt/uav-data/resnet18-f37072fd.pth`, SHA256
`f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec`.
No UAV-domain checkpoint is included. AOT is not part of this release.
