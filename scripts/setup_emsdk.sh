#!/usr/bin/env bash
# Install the Emscripten SDK into ~/emsdk (needed only for the web demo, scripts/build_web.sh).
#   bash scripts/setup_emsdk.sh [version]      default: latest
set -eo pipefail
VERSION=${1:-latest}
cd ~
[ -d emsdk ] || git clone --depth 1 https://github.com/emscripten-core/emsdk.git
cd emsdk
git pull --ff-only || true
./emsdk install "$VERSION"
./emsdk activate "$VERSION"
source ./emsdk_env.sh > /dev/null 2>&1
emcc --version | head -1
node --version
