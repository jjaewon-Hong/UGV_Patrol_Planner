#!/usr/bin/env bash
# Check the WebAssembly build in Node: scenarios S1~S5 on the module, and the same A* path as the native build.
#   bash scripts/test_web.sh [native_build_dir]      run scripts/build_web.sh first
set -eo pipefail
set +u
cd "$(dirname "$0")/.."
BUILD=${1:-build_cpp}

if ! command -v node > /dev/null; then
  source "${EMSDK:-$HOME/emsdk}/emsdk_env.sh" > /dev/null 2>&1
fi

NATIVE=""
if [ -x "$BUILD/ugv_plan" ]; then
  NATIVE=$(echo "42 30 64 21" | "$BUILD/ugv_plan" share/data/campus_pred/dem.npy share/data/campus_pred/obstacles.npy 10)
fi
node web/test/node_test.mjs "$NATIVE"
