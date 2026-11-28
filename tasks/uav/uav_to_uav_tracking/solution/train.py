"""Train the baseline detector on public training frames."""

import argparse, collections, hashlib, json, math, random, time
from pathlib import Path
import numpy as np
from PIL import Image
import torch
from torch.utils.data import Dataset, DataLoader, WeightedRandomSampler
from torchvision.transforms import ColorJitter
from model import POSTPROCESS, build_model


class Frames(Dataset):
    def __init__(self, root):
        self.root = Path(root)
        d = json.loads((self.root / "annotations.json").read_text())
        self.images = d["images"]
        self.boxes = collections.defaultdict(list)
        for a in d["annotations"]:
            x, y, w, h = a["bbox"]
            self.boxes[a["image_id"]].append([x, y, x + w, y + h])
        sizes = collections.Counter(im["sequence_id"] for im in self.images)
        self.weights = [1 / sizes[im["sequence_id"]] for im in self.images]
        self.jitter = ColorJitter(0.35, 0.35, 0.25, 0.05)

    def __len__(self):
        return len(self.images)

    def __getitem__(self, i):
        r = self.images[i]
        with Image.open(self.root / r["file_name"]) as f:
            im = f.convert("RGB")
        boxes = np.array(self.boxes[r["id"]], dtype=np.float32).reshape(-1, 4)
        if random.random() < 0.65:
            width, height = im.size
            side = min(random.choice([384, 512, 768, 960]), width, height)
            if len(boxes) and random.random() < 0.8:
                b = boxes[random.randrange(len(boxes))]
                cx, cy = (b[:2] + b[2:]) / 2
                left = int(
                    np.clip(cx - random.uniform(0.15, 0.85) * side, 0, width - side)
                )
                top = int(
                    np.clip(cy - random.uniform(0.15, 0.85) * side, 0, height - side)
                )
            else:
                left = random.randint(0, width - side)
                top = random.randint(0, height - side)
            im = im.crop((left, top, left + side, top + side))
            old = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
            boxes -= np.array([left, top, left, top])
            boxes = np.clip(boxes, 0, side)
            area = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
            boxes = boxes[(area > 1) & (area >= 0.5 * old)]
        if random.random() < 0.5:
            im = im.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
            boxes[:, [0, 2]] = im.width - boxes[:, [2, 0]]
        im = self.jitter(im)
        if random.random() < 0.1:
            im = im.convert("L").convert("RGB")
        tensor = torch.from_numpy(np.array(im)).permute(2, 0, 1).float().div_(255)
        return tensor, {
            "boxes": torch.from_numpy(boxes),
            "labels": torch.ones(len(boxes), dtype=torch.int64),
        }


def collate(batch):
    return tuple(zip(*batch))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--data", required=True)
    p.add_argument("--initialization", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--steps", type=int, default=24000)
    p.add_argument("--batch-size", type=int, default=2)
    p.add_argument("--workers", type=int, default=4)
    p.add_argument("--seed", type=int, default=20261126)
    a = p.parse_args()
    random.seed(a.seed)
    np.random.seed(a.seed)
    torch.manual_seed(a.seed)
    torch.set_num_threads(4)
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is required")
    memory = torch.cuda.get_device_properties(0).total_memory
    torch.cuda.set_per_process_memory_fraction(min(1, 24 * 1024**3 / memory))
    torch.backends.cudnn.benchmark = False
    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=True)
    config = dict(
        vars(a),
        architecture="fasterrcnn_resnet18_fpn_frozenbn",
        postprocessing=POSTPROCESS,
        augmentation="sequence balanced, 65% multiscale crops, horizontal flip, color jitter, 10% grayscale",
        annotation_sha256=hashlib.sha256(
            (Path(a.data) / "annotations.json").read_bytes()
        ).hexdigest(),
        initialization_sha256=hashlib.sha256(
            Path(a.initialization).read_bytes()
        ).hexdigest(),
        gpu=torch.cuda.get_device_name(),
        torch_version=torch.__version__,
        gpu_memory_limit_bytes=min(memory, 24 * 1024**3),
    )
    (out / "config.json").write_text(json.dumps(config, indent=2))
    data = Frames(a.data)
    sampler = WeightedRandomSampler(
        data.weights, a.steps * a.batch_size, replacement=True
    )
    loader = DataLoader(
        data,
        batch_size=a.batch_size,
        sampler=sampler,
        num_workers=a.workers,
        collate_fn=collate,
        pin_memory=True,
    )
    model = build_model(a.initialization).cuda().train()
    groups = [
        {
            "params": [
                p
                for n, p in model.named_parameters()
                if p.requires_grad and n.startswith("backbone.body")
            ],
            "lr": 0.0001,
        },
        {
            "params": [
                p
                for n, p in model.named_parameters()
                if p.requires_grad and not n.startswith("backbone.body")
            ],
            "lr": 0.001,
        },
    ]
    optimizer = torch.optim.AdamW(groups, weight_decay=0.0001)

    def schedule(step):
        return min(1.0, (step + 1) / 500) * (
            0.1 + 0.9 * 0.5 * (1 + math.cos(math.pi * min(step, a.steps) / a.steps))
        )

    scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, schedule)
    started = time.monotonic()
    with (out / "training_log.jsonl").open("w", buffering=1) as log:
        for step, (ims, targets) in enumerate(loader, 1):
            ims = [x.cuda(non_blocking=True) for x in ims]
            targets = [
                {k: v.cuda(non_blocking=True) for k, v in t.items()} for t in targets
            ]
            losses = model(ims, targets)
            loss = sum(losses.values())
            if not torch.isfinite(loss):
                raise RuntimeError("non-finite loss")
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 10)
            optimizer.step()
            scheduler.step()
            record = {
                "step": step,
                "loss": float(loss.detach()),
                "elapsed_s": time.monotonic() - started,
                "gpu_memory_allocated": torch.cuda.memory_allocated(),
                **{k: float(v.detach()) for k, v in losses.items()},
            }
            log.write(json.dumps(record) + "\n")
            if step % 100 == 0 or step == 1:
                print(json.dumps(record), flush=True)
            if step in {6000, 12000, a.steps}:
                torch.save(
                    {k: v.cpu() for k, v in model.state_dict().items()},
                    out / f"model-{step}.pt",
                )
            if step >= a.steps:
                break
    torch.save({k: v.cpu() for k, v in model.state_dict().items()}, out / "model.pt")
    print(
        json.dumps(
            {
                "training_seconds": time.monotonic() - started,
                "peak_gpu_memory": torch.cuda.max_memory_allocated(),
            }
        ),
        flush=True,
    )


if __name__ == "__main__":
    main()
