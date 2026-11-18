#!/usr/bin/env bash
# Saves a mapping session's live SLAM map as a track that racing can load:
#   src/rc_maps/tracks/<track>/map.png + map.yaml
# then `colcon build --packages-select rc_maps` to install it.
#
# Usage: scripts/save_map.sh <track> [namespace]   (default: rc_car)
set -euo pipefail

TRACK="${1:?usage: save_map.sh <track> [namespace]}"
NAMESPACE="${2:-rc_car}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/src/rc_maps/tracks/${TRACK}"

mkdir -p "${DIR}"
ros2 run nav2_map_server map_saver_cli -t "/${NAMESPACE}/map" -f "${DIR}/map" --fmt png
