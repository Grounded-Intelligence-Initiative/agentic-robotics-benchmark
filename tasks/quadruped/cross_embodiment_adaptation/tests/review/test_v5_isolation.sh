#!/usr/bin/env bash
# Run as root in a disposable container built from the task Dockerfile.
set -euo pipefail
task=$(cd "$(dirname "$0")/../.." && pwd)
stage=$(mktemp -d /root/ale-cross-setup.XXXXXX)
cp -R "$task/environment/setup/." "$stage"
bash "$stage/setup.sh"
for path in \
  /opt/visual_robotics/assets/unitree_go1/go1.xml \
  /opt/visual_robotics/assets/provenance.json \
  /tmp/visual_assets/visual_assets.json \
  "$stage/runtime/articulated.py" \
  /var/lib/visual_robotics/runtime/articulated.py; do
  test -r "$path"
  runuser -u user -- test ! -r "$path"
done

probe='
from client import Client, camera_image

r = Client()
obs = r.observe()
assert obs["phase"] == 0 and camera_image(obs).shape == (256, 256, 3)
for _ in range(4):
    obs = r.act([.2, .0])
try:
    obs = r.advance()
except ValueError:
    assert not graded
else:
    assert graded and obs["phase"] == 1 and "images" not in obs
    lower = obs["ticks_remaining"]
    assert r.act({"target_rad": [0., .9, -1.8] * 4, "ticks": 12})["ticks_remaining"] == lower - 12
'
runuser -u user -- env PYTHONPATH=/opt/visual_robotics/client \
  /opt/venv/bin/python3 -B -c "graded = False"$'\n'"$probe"

# Independently start the graded service on a different socket.
dir=$(mktemp -d /root/ale-cross-graded.XXXXXX)
printf '1\n' > "$dir/seed"
socket=/run/visual_robotics/isolation-test.sock
/opt/venv/bin/python3 -B "$stage/runtime/service.py" --mode graded --directory "$dir" \
  --socket "$socket" --seed-file "$dir/seed" &
service=$!
trap 'kill "$service" 2>/dev/null || true' EXIT
for _ in {1..100}; do
  test ! -e "$dir/ready" || break
  sleep .1
done
runuser -u user -- env PYTHONPATH=/opt/visual_robotics/client ROBOT_SOCKET="$socket" \
  /opt/venv/bin/python3 -B -c "graded = True"$'\n'"$probe"
printf 'Asset/runtime read denial, scout render, and grader-only native deployment passed.\n'
