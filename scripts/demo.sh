#!/usr/bin/env bash
# Send the demo operator commands: add the scenario's patrol points (fixed or random flags), then start.
# Run in a second terminal after scripts/run.sh.
#   bash scripts/demo.sh [name]      e.g. default, low_battery, campus
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
NAME=${1:-default}
SHARE=$(ros2 pkg prefix ugv_patrol_planner)/share/ugv_patrol_planner
CONFIG=$SHARE/config/$NAME.yaml
[ -f "$CONFIG" ] || CONFIG=$SHARE/config/params.yaml

# --keep-alive: stay up long enough for the message to be delivered before exiting.
send() { ros2 topic pub --once --keep-alive 1 /operator/command std_msgs/msg/String "{data: '$1'}" > /dev/null; echo "sent: $1"; }

python3 -m ugv_patrol_planner.mission.flags "$CONFIG" "$SHARE/scenarios/$NAME.yaml" | while read -r r c; do send "add $r $c"; done
send "start"
