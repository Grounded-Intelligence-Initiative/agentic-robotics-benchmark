"""Reference architecture. This file is visible only in the oracle solution stage."""

from collections import OrderedDict
from pathlib import Path
import numpy as np
import torch
from torch import nn
from torchvision.ops.misc import FrozenBatchNorm2d
from torchvision.models import resnet18
from torchvision.models._utils import IntermediateLayerGetter
from torchvision.ops import FeaturePyramidNetwork, nms
from torchvision.ops.feature_pyramid_network import LastLevelMaxPool
from torchvision.models.detection import FasterRCNN
from torchvision.models.detection.anchor_utils import AnchorGenerator

POSTPROCESS = {"box_scale": 1.75, "nms_iou": 0.3}


def postprocess(prediction, width, height):
    """Global calibration selected on public validation, using current-frame boxes only."""
    # CPU NMS also makes the calibration path independent of CUDA tie-breaking.
    boxes = prediction["boxes"].detach().cpu().clone()
    scores = prediction["scores"].detach().cpu()
    boxes[:, 0::2].clamp_(0, width)
    boxes[:, 1::2].clamp_(0, height)
    valid = (boxes[:, 2] > boxes[:, 0]) & (boxes[:, 3] > boxes[:, 1])
    boxes, scores = boxes[valid], scores[valid]
    sizes = boxes[:, 2:] - boxes[:, :2]
    centers = boxes[:, :2] + sizes / 2
    sizes = sizes * POSTPROCESS["box_scale"]
    boxes = torch.cat((centers - sizes / 2, centers + sizes / 2), dim=1)
    boxes[:, 0::2].clamp_(0, width)
    boxes[:, 1::2].clamp_(0, height)
    keep = nms(boxes, scores, POSTPROCESS["nms_iou"])[:300]
    return torch.cat((boxes[keep], scores[keep, None]), dim=1).numpy()


class Backbone(nn.Module):
    out_channels = 128

    def __init__(self, initialization=None):
        super().__init__()
        base = resnet18(weights=None, norm_layer=FrozenBatchNorm2d)
        if initialization is not None:
            base.load_state_dict(
                torch.load(initialization, map_location="cpu", weights_only=True)
            )
        for name, parameter in base.named_parameters():
            if not name.startswith(("layer3", "layer4")):
                parameter.requires_grad_(False)
        self.body = IntermediateLayerGetter(
            base,
            return_layers={"layer1": "0", "layer2": "1", "layer3": "2", "layer4": "3"},
        )
        self.fpn = FeaturePyramidNetwork(
            [64, 128, 256, 512], 128, extra_blocks=LastLevelMaxPool()
        )

    def forward(self, image):
        return self.fpn(self.body(image))


def build_model(initialization=None):
    anchors = AnchorGenerator(
        sizes=((8,), (16,), (32,), (64,), (128,)), aspect_ratios=((0.5, 1.0, 2.0),) * 5
    )
    return FasterRCNN(
        Backbone(initialization),
        num_classes=2,
        rpn_anchor_generator=anchors,
        min_size=800,
        max_size=1333,
        box_score_thresh=0.001,
        box_detections_per_img=300,
    )


class Detector:
    def __init__(self, checkpoint_path, device):
        self.device = torch.device(device)
        self.model = build_model()
        self.model.load_state_dict(
            torch.load(checkpoint_path, map_location="cpu", weights_only=True)
        )
        self.model.to(self.device).eval()

    def reset(self):
        pass

    @torch.inference_mode()
    def predict(self, frame):
        tensor = (
            torch.from_numpy(np.array(frame, copy=True))
            .permute(2, 0, 1)
            .float()
            .div_(255)
            .to(self.device)
        )
        pred = self.model([tensor])[0]
        return postprocess(pred, frame.shape[1], frame.shape[0])
