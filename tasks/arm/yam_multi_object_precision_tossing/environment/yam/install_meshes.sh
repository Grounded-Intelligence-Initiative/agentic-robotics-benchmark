#!/bin/sh
set -eu

manifest=$(dirname "$0")/MESHES.sha256
dest=${1:?pass the parent directory for assets/}
base=https://raw.githubusercontent.com/i2rt-robotics/i2rt/1276f63d640eb45c226efd3dc08430b810372e94/i2rt/robot_models/arm/yam/v1

while read -r digest path; do
    curl --fail --silent --show-error --location --retry 3 --create-dirs \
        --output "$dest/$path" "$base/$path"
    (cd "$dest" && printf '%s  %s\n' "$digest" "$path" | sha256sum --check --status)
done < "$manifest"
