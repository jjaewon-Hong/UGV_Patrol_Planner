#!/usr/bin/env bash
# Build the web demo into web/: WebAssembly module, map files, pictures.
#   bash scripts/build_web.sh
# Needs the Emscripten SDK (scripts/setup_emsdk.sh, or EMSDK set) and python3 with numpy and yaml.
# Then:  bash scripts/serve_web.sh   and open http://localhost:8000
set -eo pipefail
set +u
cd "$(dirname "$0")/.."

if ! command -v emcmake > /dev/null; then
  source "${EMSDK:-$HOME/emsdk}/emsdk_env.sh" > /dev/null 2>&1
fi

echo "== WebAssembly module"
emcmake cmake -S cpp -B build_web -DCMAKE_BUILD_TYPE=Release -DUGV_BUILD_TESTS=OFF -DUGV_BUILD_NODE=OFF > /dev/null
cmake --build build_web --target ugv_web -j
cp build_web/ugv_web.js build_web/ugv_web.wasm web/

echo "== Maps"
python3 tools/export_web.py web/maps

echo "== Pictures"
mkdir -p web/img
cp docs/images/run_2d.png docs/images/run_rviz.png docs/images/label_compare.png web/img/

ls -l web/ugv_web.js web/ugv_web.wasm web/maps
