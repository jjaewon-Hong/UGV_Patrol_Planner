#!/usr/bin/env bash
# Build both packages in ~/ros2_ws (outside the repo, so build/install/log never land in git):
#   ugv_patrol_planner (Python nodes, data) and ugv_patrol_planner_cpp (C++ controller in cpp/).
#   bash scripts/build.sh
# Afterwards, in each new terminal: source ~/ros2_ws/install/setup.bash
set -eo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
WS=~/ros2_ws

mkdir -p "$WS/src"
ln -sfn "$REPO" "$WS/src/ugv_patrol_planner"
ln -sfn "$REPO/cpp" "$WS/src/ugv_patrol_planner_cpp"
[ -f "$REPO/share/data/dem_100x100.npy" ] || python3 "$REPO/tools/make_synthetic_dem.py" "$REPO/share/data/dem_100x100.npy"

set +u
source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --packages-select ugv_patrol_planner ugv_patrol_planner_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "Built. Run: source $WS/install/setup.bash"
