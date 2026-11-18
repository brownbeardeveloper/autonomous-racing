#!/usr/bin/env bash
# Drives the car from this terminal: w/s throttle, a/d steering, space stop.
# Needs a session in mode:=teleop, on the car or in sim, and the e-stop reset.
# A node of its own, not part of the launch: only a terminal gives it keys.
#
# Usage: scripts/keyboard_teleop.sh [namespace]   (default: rc_car)
set -euo pipefail

NAMESPACE="${1:-rc_car}"

exec ros2 run rc_control keyboard_teleop_node --ros-args -r "__ns:=/${NAMESPACE}" \
    --params-file "$(ros2 pkg prefix --share rc_control)/config/basic.yaml"
