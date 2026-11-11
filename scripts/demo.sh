#!/usr/bin/env bash
# Send the demo operator commands: add the scenario's patrol points, then start.
# Run in a second terminal after `ros2 launch ugv_patrol_planner patrol.launch.py`.
#   bash scripts/demo.sh [scenario.yaml]
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
SCENARIO=${1:-"$(dirname "$0")/../scenarios/default.yaml"}

# --keep-alive: stay up long enough for the message to be delivered before exiting.
send() { ros2 topic pub --once --keep-alive 1 /operator/command std_msgs/msg/String "{data: '$1'}" > /dev/null; echo "sent: $1"; }

python3 - "$SCENARIO" <<'EOF' | while read -r r c; do send "add $r $c"; done
import sys, yaml
for r, c in yaml.safe_load(open(sys.argv[1])).get('patrol_points', []):
    print(r, c)
EOF
send "start"
