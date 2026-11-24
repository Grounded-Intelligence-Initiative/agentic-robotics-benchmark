"""Maintainers' tool: train the pixel baseline's net on a development set, as an agent could in its sandbox.

    <python with torch> tests/review/pixel_baseline/train.py --dev DIR --out model.pt [--val dev09,dev10] [--epochs 30]

Reads only what the agent is given: the development recordings' frames and labels (tip pixels of the left and right
instrument when they are in the picture, their apparent size, the smeared-lens flag). Augments with a mirror image
(left and right swapped), colour shifts, haze and noise. Prints, per epoch, the validation recordings' detection
rate and median pixel error.
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np
import torch
import torch.nn.functional as F

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from net import CELL, GRID_H, GRID_W, TipNet, decode  # noqa: E402


def load(dev, names):
    frames, labels = [], []
    for name in names:
        root = os.path.join(dev, name)
        for line in open(os.path.join(root, "labels.jsonl")):
            lab = json.loads(line)
            img = cv2.imread(os.path.join(root, lab["frame"]))
            frames.append(cv2.cvtColor(cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB))
            row = np.zeros((2, 4), np.float32)             # per hand: u, v, apparent size, seen
            for inst in lab["instruments"]:
                if inst["role"] in ("left", "right") and inst["in_image"]:
                    row[0 if inst["role"] == "left" else 1] = [inst["tip_px"][0], inst["tip_px"][1], inst["apparent_px"], 1.0]
            labels.append((row, float(lab["lens_smeared"])))
    x = torch.from_numpy(np.stack(frames)).permute(0, 3, 1, 2).contiguous()
    tips = torch.from_numpy(np.stack([l[0] for l in labels]))
    smear = torch.tensor([l[1] for l in labels])
    return x, tips, smear


def targets(tips, device):
    """Heat maps (B, 2, 60, 80), size targets and masks from tips (B, 2, 4) in 640 x 480 pixels."""
    b = tips.shape[0]
    yy = torch.arange(GRID_H, device=device).view(1, 1, GRID_H, 1).float()
    xx = torch.arange(GRID_W, device=device).view(1, 1, 1, GRID_W).float()
    cu = (tips[..., 0] / CELL - 0.5).view(b, 2, 1, 1)
    cv_ = (tips[..., 1] / CELL - 0.5).view(b, 2, 1, 1)
    seen = tips[..., 3].view(b, 2, 1, 1)
    heat = torch.exp(-((xx - cu) ** 2 + (yy - cv_) ** 2) / (2 * 1.3 ** 2)) * seen
    return heat, torch.log(tips[..., 2].clamp(min=4.0)).view(b, 2, 1, 1), seen


def augment(x, tips, gen):
    """x uint8 (B, 3, 240, 320) on the device -> float; mirror, colour, haze, noise, a small shift."""
    b, dev = x.shape[0], x.device
    x = x.float() / 255.0
    tips = tips.clone()
    flip = torch.rand(b, device=dev, generator=gen) < 0.5
    x = torch.where(flip.view(b, 1, 1, 1), x.flip(3), x)
    tips[flip, :, 0] = 640.0 - tips[flip, :, 0]
    tips[flip] = tips[flip].flip(1)                           # the mirrored left instrument is a right one
    # colour: per-image gain per channel, gamma, saturation
    gain = 1.0 + 0.16 * (torch.rand(b, 3, 1, 1, device=dev, generator=gen) - 0.5)
    gamma = torch.exp(0.5 * (torch.rand(b, 1, 1, 1, device=dev, generator=gen) - 0.5))
    x = (x * gain).clamp(0, 1) ** gamma
    grey = x.mean(1, keepdim=True)
    sat = 0.55 + 0.9 * torch.rand(b, 1, 1, 1, device=dev, generator=gen)
    x = (grey + (x - grey) * sat).clamp(0, 1)
    # haze: a smooth field towards light grey, on a third of the images
    field = F.interpolate(torch.rand(b, 1, 6, 8, device=dev, generator=gen), size=x.shape[-2:], mode="bicubic", align_corners=False).clamp(0, 1)
    amount = torch.rand(b, 1, 1, 1, device=dev, generator=gen) * (torch.rand(b, 1, 1, 1, device=dev, generator=gen) < 0.35)
    a = (amount * (0.35 + 0.65 * field)).clamp(0, 0.9)
    x = x * (1 - a) + 0.85 * a
    x = x + 0.012 * torch.randn(x.shape, device=dev, generator=gen)
    # a shift of up to 24 px of the 320 x 240 picture (48 px of the frame), edges repeated
    dx = int(torch.randint(-24, 25, (1,), generator=gen, device=dev))
    dy = int(torch.randint(-18, 19, (1,), generator=gen, device=dev))
    x = torch.roll(x, (dy, dx), (2, 3))
    tips[..., 0] += 2.0 * dx
    tips[..., 1] += 2.0 * dy
    inside = (tips[..., 0] >= 0) & (tips[..., 0] < 640) & (tips[..., 1] >= 0) & (tips[..., 1] < 480)
    tips[..., 3] = tips[..., 3] * inside
    # the endoscope's disk again (the roll moved it)
    yy, xx = torch.meshgrid(torch.arange(240, device=dev), torch.arange(320, device=dev), indexing="ij")
    disk = (((xx - 159.5) ** 2 + (yy - 119.5) ** 2) <= 155.0 ** 2).float()
    return (x * disk).clamp(0, 1), tips


def evaluate(net, x, tips, smear, device):
    net.eval()
    errs, found, total, false_pos, absent, sm_ok = [], 0, 0, 0, 0, 0
    with torch.no_grad():
        for i in range(0, len(x), 64):
            out, sm = net(x[i:i + 64].to(device).float() / 255.0)
            out, sm = out.cpu(), sm.cpu()
            sm_ok += int(((sm > 0).float() == smear[i:i + 64]).sum())
            for j in range(out.shape[0]):
                for hand in (0, 1):
                    u, v, s, conf = decode(out[j], hand)
                    t = tips[i + j, hand]
                    if t[3] > 0:
                        total += 1
                        if conf > 0.4:
                            found += 1
                            errs.append(float(np.hypot(u - float(t[0]), v - float(t[1]))))
                    else:
                        absent += 1
                        false_pos += int(conf > 0.4)
    errs = np.array(errs) if errs else np.array([np.nan])
    return {"found": found / max(1, total), "median_px": float(np.median(errs)), "within_25px": float((errs < 25).mean()),
            "false_positive": false_pos / max(1, absent), "smear_acc": sm_ok / len(x)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--val", default="dev09,dev10")
    ap.add_argument("--epochs", type=int, default=30)
    a = ap.parse_args()
    device = "cuda" if torch.cuda.is_available() else "cpu"
    names = sorted(n for n in os.listdir(a.dev) if n.startswith("dev") and os.path.isdir(os.path.join(a.dev, n)))
    val = [n for n in a.val.split(",") if n]
    t0 = time.time()
    xt, tt, st = load(a.dev, [n for n in names if n not in val])
    xv, tv, sv = load(a.dev, val) if val else (xt[:64], tt[:64], st[:64])
    print("train {} frames, validation {} frames ({}), loaded in {:.0f} s, device {}".format(len(xt), len(xv), val, time.time() - t0, device), flush=True)
    net = TipNet().to(device)
    opt = torch.optim.AdamW(net.parameters(), lr=2e-3, weight_decay=1e-4)
    steps = a.epochs * (len(xt) // 32)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=2e-3, total_steps=steps)
    gen = torch.Generator(device=device)
    gen.manual_seed(0)
    xt_d, tt_d, st_d = xt.to(device), tt.to(device), st.to(device)
    for epoch in range(a.epochs):
        net.train()
        perm = torch.randperm(len(xt), device=device, generator=gen)
        tot = 0.0
        for i in range(0, len(perm) - 31, 32):
            idx = perm[i:i + 32]
            x, tips = augment(xt_d[idx], tt_d[idx], gen)
            heat, logsize, seen = targets(tips, device)
            out, sm = net(x)
            pos = heat > 0.5
            w = 1.0 + 30.0 * heat
            loss = (F.binary_cross_entropy_with_logits(out[:, :2], heat, reduction="none") * w).mean()
            if pos.any():
                loss = loss + 0.5 * (F.l1_loss(out[:, 2:], logsize.expand_as(heat), reduction="none") * pos).sum() / pos.sum()
            loss = loss + 0.2 * F.binary_cross_entropy_with_logits(sm, st_d[idx])
            opt.zero_grad(set_to_none=True)
            loss.backward()
            opt.step()
            sched.step()
            tot += float(loss.detach())
        if epoch % 3 == 2 or epoch == a.epochs - 1:
            print("epoch {:2d} loss {:.4f} | validation {} | {:.0f} s".format(epoch + 1, tot / (len(perm) // 32),
                  json.dumps({k: round(v, 3) for k, v in evaluate(net, xv, tv, sv, device).items()}), time.time() - t0), flush=True)
    torch.save(net.state_dict(), a.out)
    print("saved", a.out, flush=True)


if __name__ == "__main__":
    main()
