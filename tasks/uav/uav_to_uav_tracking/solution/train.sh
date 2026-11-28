#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
python train.py --data /opt/uav-data/train --initialization /opt/uav-data/resnet18-f37072fd.pth --output . --steps 24000 --seed 20261126
python export_checkpoint.py --select-in-place . --step 12000
