#!/usr/bin/env bash
# Stop every running instance of the patrol planner: the launch, all its nodes and its RViz2.
#   bash scripts/stop.sh
# run.sh calls this first, so two runs never publish to the same topics at once
# (e.g. two visualizers sending different RViz markers make the map flicker).
PATTERN='ros2 launch ugv_patrol_planner|lib/ugv_patrol_planner(_cpp)?/|rviz2 -d .*(ugv_patrol_planner|patrol)\.rviz|__node:=map_frame'

running() { pgrep -f -- "$PATTERN" > /dev/null; }

running || exit 0
pkill -INT -f -- "$PATTERN"                  # ask nicely first (same as Ctrl-C)
for _ in $(seq 1 10); do running || exit 0; sleep 1; done
pkill -TERM -f -- "$PATTERN"
for _ in $(seq 1 5); do running || exit 0; sleep 1; done
pkill -KILL -f -- "$PATTERN"                 # leftovers that ignored both
sleep 1
running && { echo "stop.sh: some patrol planner processes are still running" >&2; exit 1; }
exit 0
