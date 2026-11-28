#!/bin/bash
cd "$(dirname "$0")"
if [ -n "$DEXDEBUG" ]; then
  exec /opt/venv/bin/python3 -u controller.py
else
  exec /opt/venv/bin/python3 -u controller.py 2>/dev/null
fi
