#!/usr/bin/env bash
# Replays a recorded bag into Foxglove, in tmux:
#   window "bridge": foxglove_bridge on port 8766, running on the bag's clock
#   window "play":   ros2 bag play, started paused - Space plays and pauses
#
# Usage: scripts/replay_bag.sh <bag dir> [ros2 bag play args, e.g. -r 2 --start-offset 120]
# Foxglove: Open connection -> Foxglove WebSocket -> ws://<jetson ip>:8766
# End with `tmux kill-session -t replay`.
#
# The bag carries drive_cmd and estop, which a running safety_node would relay
# to /drive. So the replay gets a DDS domain that never leaves this machine, and
# a port of its own so it is never mistaken for the live car on 8765.
set -euo pipefail

SESSION=replay
# Inside tmux, `attach` refuses to nest: switch this client to the session instead.
if [ -n "${TMUX:-}" ]; then ATTACH=switch-client; else ATTACH=attach; fi
BAG="$(realpath "${1:?usage: replay_bag.sh <bag dir> [ros2 bag play args]}")"
shift

if [ ! -f "${BAG}/metadata.yaml" ]; then
    echo "${BAG} has no metadata.yaml - an unclosed bag needs \`ros2 bag reindex ${BAG} -s sqlite3\` first." >&2
    exit 1
fi
if tmux has-session -t "${SESSION}" 2>/dev/null; then
    echo "tmux session '${SESSION}' is already running - \`tmux attach -t ${SESSION}\`, or kill it first." >&2
    exit 1
fi

SETUP="source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1"

tmux new-session -d -s "${SESSION}" -n bridge
tmux send-keys -t "${SESSION}:bridge" "${SETUP} && ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8766 use_sim_time:=true" C-m

tmux new-window -t "${SESSION}:" -n play
tmux send-keys -t "${SESSION}:play" "${SETUP} && ros2 bag play $(printf '%q ' "${BAG}" "$@")--clock --start-paused" C-m

exec tmux "${ATTACH}" -t "${SESSION}"
