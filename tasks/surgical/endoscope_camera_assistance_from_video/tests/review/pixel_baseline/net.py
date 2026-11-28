"""The pixel baseline's network: a small fully convolutional net from a 320 x 240 frame to, on an 80 x 60 grid (one
cell = 8 px of the 640 x 480 frame), a heat map of the left tip and of the right tip and each one's apparent shaft
diameter, plus one number for a smeared lens. About 1.3 M parameters."""

import torch
import torch.nn as nn
import torch.nn.functional as F

GRID_W, GRID_H, CELL = 80, 60, 8.0


def _block(cin, cout, stride):
    return nn.Sequential(nn.Conv2d(cin, cout, 3, stride, 1, bias=False), nn.BatchNorm2d(cout), nn.ReLU(inplace=True),
                         nn.Conv2d(cout, cout, 3, 1, 1, bias=False), nn.BatchNorm2d(cout), nn.ReLU(inplace=True))


class TipNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.e1, self.e2, self.e3, self.e4 = _block(3, 32, 2), _block(32, 64, 2), _block(64, 128, 2), _block(128, 192, 2)
        self.d3, self.d2 = _block(192 + 128, 128, 1), _block(128 + 64, 96, 1)
        self.head = nn.Conv2d(96, 4, 1)           # heat left, heat right, log size left, log size right
        self.smear = nn.Linear(192, 1)

    def forward(self, x):
        a = self.e1(x)
        b = self.e2(a)
        c = self.e3(b)
        d = self.e4(c)
        u = self.d3(torch.cat([F.interpolate(d, size=c.shape[-2:], mode="bilinear", align_corners=False), c], 1))
        u = self.d2(torch.cat([F.interpolate(u, size=b.shape[-2:], mode="bilinear", align_corners=False), b], 1))
        return self.head(u), self.smear(d.mean((2, 3))).squeeze(1)


def prepare(frame_rgb_uint8):
    """A (480, 640, 3) uint8 RGB frame (numpy) as the net's input tensor (1, 3, 240, 320), on the CPU."""
    t = torch.from_numpy(frame_rgb_uint8).permute(2, 0, 1).float().unsqueeze(0) / 255.0
    return F.interpolate(t, size=(240, 320), mode="bilinear", align_corners=False)


def decode(out, hand):
    """(u, v, apparent size px, confidence) of one hand's tip from the head's output (4, 60, 80) on the CPU."""
    heat = torch.sigmoid(out[hand])
    conf, idx = heat.flatten().max(0)
    r, c = int(idx) // GRID_W, int(idx) % GRID_W
    r0, r1, c0, c1 = max(r - 1, 0), min(r + 2, GRID_H), max(c - 1, 0), min(c + 2, GRID_W)
    w = heat[r0:r1, c0:c1]
    rr = torch.arange(r0, r1, dtype=torch.float32)[:, None].expand_as(w)
    cc = torch.arange(c0, c1, dtype=torch.float32)[None, :].expand_as(w)
    s = float(w.sum()) + 1e-9
    u = (float((w * cc).sum()) / s + 0.5) * CELL
    v = (float((w * rr).sum()) / s + 0.5) * CELL
    return u, v, float(torch.exp(out[2 + hand, r, c])), float(conf)
