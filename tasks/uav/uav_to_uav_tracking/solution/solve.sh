#!/usr/bin/env bash
set -euo pipefail
TASK_SOLUTION="$(cd "$(dirname "$0")" && pwd)"
TASK_OUTPUT="${ALE_HOME:-/home/user}/submission"
mkdir -p "$TASK_OUTPUT/baseline"
# Preserve the genuine training source and logs. Only this author oracle carries GT.
cp "$TASK_SOLUTION/model.py" "$TASK_SOLUTION/train.py" "$TASK_SOLUTION/train.sh" "$TASK_SOLUTION/export_checkpoint.py" "$TASK_OUTPUT/baseline/"
cp "$TASK_SOLUTION/build_gt_submission.py" "$TASK_SOLUTION/gt_model.py" "$TASK_SOLUTION/gt_fixture.json" "$TASK_OUTPUT/"
cat > "$TASK_OUTPUT/train.sh" <<'REPRODUCE'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
bash baseline/train.sh
python build_gt_submission.py --baseline baseline --output . --fixture gt_fixture.json
REPRODUCE
chmod +x "$TASK_OUTPUT/train.sh"
bash "$TASK_OUTPUT/train.sh"
