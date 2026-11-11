#!/usr/bin/env bash
# Start everything (controller, simulators, 2D view, RViz2) without typing the long commands.
#   bash scripts/run.sh                 default scenario
#   bash scripts/run.sh low_battery     scenarios/low_battery.yaml
# Then, in another terminal:  bash scripts/demo.sh
set +u
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
SCENARIO=$(ros2 pkg prefix ugv_patrol_planner)/share/ugv_patrol_planner/scenarios/${1:-default}.yaml
ros2 launch ugv_patrol_planner patrol.launch.py rviz:=true scenario:="$SCENARIO"
