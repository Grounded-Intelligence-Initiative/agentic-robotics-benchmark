#!/usr/bin/env bash
# Framework interpreter owns ale_verify; root-only libraries supply the simulator.
set -euo pipefail
export PYTHONPATH=/opt/station:/opt/physics/lib/python3.12/site-packages
export MUJOCO_GL=egl
case "${YAM_EVALUATION_PROFILE:-full}" in
  single-set) exec "${ALE_VERIFY_PYTHON:-python3}" verify.py --single-set ;;
  full) exec "${ALE_VERIFY_PYTHON:-python3}" verify.py ;;
  *) echo "Unknown YAM evaluation profile" >&2; exit 1 ;;
esac
