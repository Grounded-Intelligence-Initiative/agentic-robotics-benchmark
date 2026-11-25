#!/usr/bin/env bash
# Rebuild this task's events from the IEDD release (about 220 MB from Hugging Face Egikk/IEDD) in <work dir>.
#   bash tests/review/build_from_hf.sh <work dir> [extra build_task_data.py options]
# IEDD_REVISION pins the Hugging Face commit (default: main; the commit used is printed for tests/NOTICE.md).
set -euo pipefail
WORK="${1:?usage: build_from_hf.sh <work dir> [build_task_data.py options]}"
shift
TASK="$(cd "$(dirname "$0")/../.." && pwd)"
REV="${IEDD_REVISION:-main}"
mkdir -p "$WORK"
hf download Egikk/IEDD IEDD-VQA_test.json IEDD-VQA_test_videos.zip IEDD-waymo.zip \
    --repo-type dataset --revision "$REV" --local-dir "$WORK"
[ -d "$WORK/clips" ] || unzip -q "$WORK/IEDD-VQA_test_videos.zip" -d "$WORK/clips"
[ -d "$WORK/csv" ] || unzip -q "$WORK/IEDD-waymo.zip" -d "$WORK/csv"
python3 -c "import sys; from huggingface_hub import HfApi; print('Egikk/IEDD commit:', HfApi().dataset_info('Egikk/IEDD', revision=sys.argv[1]).sha)" "$REV"
python3 "$TASK/tests/review/build_task_data.py" --csv-dir "$WORK/csv" --videos "$WORK/clips" \
    --vqa-json "$WORK/IEDD-VQA_test.json" --task-dir "$TASK" --out "$WORK/upload" "$@"
