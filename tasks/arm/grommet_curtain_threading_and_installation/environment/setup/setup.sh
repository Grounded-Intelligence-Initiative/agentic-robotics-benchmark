#!/usr/bin/env bash
set -euo pipefail

install -d -o user -g user /home/user/given /home/user/submission
install -o user -g user -m 0644 /opt/curtain/robot_description/{mobile_fr3_duo.urdf,calibration.json} /home/user/given/
