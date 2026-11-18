#!/usr/bin/env bash
# Starts a car session in tmux, so it survives the WiFi and SSH dropping:
#   window "car":   car.launch.py in the given mode
#   window "bag":   ros2 bag record into bags/, started once the lidar publishes
#   window "map":   mapping only - save_map.sh for <track> typed in, Enter saves
#   window "drive": teleop only - keyboard_teleop.sh, the keys that drive the car
#
# Usage: scripts/start_car_session.sh mapping <track>   (a new track name)
#        scripts/start_car_session.sh racing <track>
#        scripts/start_car_session.sh teleop
#
# Detach with Ctrl+b d; run this again, or `tmux attach -t car`, to come back.
# To end, Ctrl+C the bag first and wait for "Recording stopped", then the stack.
set -euo pipefail

SESSION=car
# Inside tmux, `attach` refuses to nest: switch this client to the session instead.
if [ -n "${TMUX:-}" ]; then ATTACH=switch-client; else ATTACH=attach; fi

if tmux has-session -t "${SESSION}" 2>/dev/null; then
    echo "tmux session '${SESSION}' is already running - attaching instead of starting a second stack."
    exec tmux "${ATTACH}" -t "${SESSION}"
fi

usage() {
    echo "usage: start_car_session.sh mapping|racing <track>" >&2
    echo "       start_car_session.sh teleop" >&2
    exit 1
}
MODE="${1:-}"
TRACK="${2:-}"
case "${MODE}" in
    mapping|racing) [[ "${TRACK}" =~ ^[A-Za-z0-9_-]+$ ]] || usage ;;
    teleop)         [ -z "${TRACK}" ] || usage ;;
    *)              usage ;;
esac

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ "${MODE}" = mapping ] && [ -e "${WS}/src/rc_maps/tracks/${TRACK}/map.yaml" ]; then
    echo "track '${TRACK}' already has a map - pick a new name, or delete src/rc_maps/tracks/${TRACK}/ to remap it." >&2
    exit 1
fi

# Mapping gets no map:=, which would have pure_pursuit_node look for a
# racingline.csv the new track does not have yet.
case "${MODE}" in
    mapping) LAUNCH_ARGS="mode:=mapping" ;;
    racing)  LAUNCH_ARGS="mode:=racing map:=${TRACK}" ;;
    teleop)  LAUNCH_ARGS="mode:=teleop" ;;
esac

SETUP="source /opt/ros/humble/setup.bash && source ~/microros_ws/install/setup.bash && source ${WS}/install/setup.bash"
mkdir -p "${WS}/bags"

tmux new-session -d -s "${SESSION}" -n car -c "${WS}"
tmux send-keys -t "${SESSION}:car" "${SETUP} && ros2 launch rc_bringup car.launch.py ${LAUNCH_ARGS}" C-m

# Best effort subscribes to the scan whatever QoS urg_node2 offers. /drive is
# left out, drive_cmd and safety_node/brake_bool already show what it carried.
# drive_cmd still reaches a running safety_node on replay: scripts/replay_bag.sh
# keeps the bag off the car's network.
tmux new-window -t "${SESSION}:" -n bag -c "${WS}/bags"
tmux send-keys -t "${SESSION}:bag" "${SETUP} && until timeout 10 ros2 topic echo --once --qos-reliability best_effort /rc_car/scan sensor_msgs/msg/LaserScan >/dev/null; do echo 'waiting for /rc_car/scan'; done && ros2 bag record -a -x '^/drive\$' --storage-preset-profile resilient -o \$(date +%F_%H-%M-%S)" C-m

# Typed in but not run: more laps without a loop closure only add drifted
# copies of the track, so the moment to save is after one lap, by hand.
# Only the underlay here: map_saver_cli is in it, and colcon warns about
# overriding rc_maps when the workspace being built is also sourced.
if [ "${MODE}" = mapping ]; then
    tmux new-window -t "${SESSION}:" -n map -c "${WS}"
    tmux send-keys -t "${SESSION}:map" "source /opt/ros/humble/setup.bash && clear && echo 'Drive one lap plus a few metres, then press Enter to save the map as ${TRACK}.'" C-m
    tmux send-keys -t "${SESSION}:map" "scripts/save_map.sh ${TRACK} && colcon build --packages-select rc_maps"
fi

# Selected on attach, so the keys go to the car. Nothing moves before the e-stop is reset.
FIRST_WINDOW=car
if [ "${MODE}" = teleop ]; then
    tmux new-window -t "${SESSION}:" -n drive -c "${WS}"
    tmux send-keys -t "${SESSION}:drive" "${SETUP} && scripts/keyboard_teleop.sh" C-m
    FIRST_WINDOW=drive
fi

tmux select-window -t "${SESSION}:${FIRST_WINDOW}"
exec tmux "${ATTACH}" -t "${SESSION}"
