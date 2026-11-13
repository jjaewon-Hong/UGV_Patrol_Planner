#!/usr/bin/env bash
# Serve the built web demo locally:  bash scripts/serve_web.sh [port]   then open http://localhost:8000
cd "$(dirname "$0")/../web"
python3 -m http.server "${1:-8000}"
