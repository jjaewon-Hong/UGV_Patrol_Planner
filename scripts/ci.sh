#!/usr/bin/env bash
# Every automatic check, in the order CI runs them (Docker image or a local ROS 2 Jazzy machine).
#   bash scripts/ci.sh [cpp_build_dir]
# Needs: ROS 2 Jazzy sourced and this package built with colcon (the Dockerfile does both).
set -eo pipefail
set +u
cd "$(dirname "$0")/.."
BUILD=${1:-build_cpp}

echo "== Python unit tests"
python3 -m pytest -q

echo "== C++ core: build and unit tests"
cmake -S cpp -B "$BUILD" -DCMAKE_BUILD_TYPE=Release > /dev/null
cmake --build "$BUILD" -j
(cd "$BUILD" && ctest --output-on-failure)

echo "== C++ vs Python A*: same paths"
python3 tools/compare_cpp.py "$BUILD/ugv_plan" 100

echo "== Scenario tests S1~S5 on ROS 2"
python3 scripts/scenario_test.py
