#!/usr/bin/env bash
set -euo pipefail
exec "${ALE_VERIFY_PYTHON:-python3}" verify.py
