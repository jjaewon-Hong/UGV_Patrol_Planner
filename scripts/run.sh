#!/usr/bin/env bash
# Start everything (controller, simulators, 2D view, RViz2) without typing the long commands.
#   bash scripts/run.sh                 default scenario
#   bash scripts/run.sh low_battery     scenarios/low_battery.yaml
#   bash scripts/run.sh campus          scenarios/campus.yaml with config/campus.yaml (campus map)
# A scenario uses config/<name>.yaml when it exists, otherwise config/params.yaml.
# Then, in another terminal:  bash scripts/demo.sh <same name>
set +u
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
NAME=${1:-default}
SHARE=$(ros2 pkg prefix ugv_patrol_planner)/share/ugv_patrol_planner
CONFIG=$SHARE/config/$NAME.yaml
[ -f "$CONFIG" ] || CONFIG=$SHARE/config/params.yaml
ros2 launch ugv_patrol_planner patrol.launch.py rviz:=true scenario:="$SHARE/scenarios/$NAME.yaml" config:="$CONFIG"
