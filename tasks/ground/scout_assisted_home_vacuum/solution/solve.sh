#!/bin/sh
set -eu
mkdir -p /home/user/submission
cp "$(dirname "$0")/controller.py" /home/user/submission/controller.py
printf '%s\n' '#!/bin/sh' 'exec /opt/venv/bin/python3 -u /home/user/submission/controller.py' > /home/user/submission/run.sh
chmod +x /home/user/submission/run.sh
