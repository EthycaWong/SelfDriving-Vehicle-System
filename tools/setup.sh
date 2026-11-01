#!/bin/sh
# One-time setup: installs tools, downloads assets and models, and builds everything.
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

brew install cmake raylib arduino-cli
arduino-cli config init --overwrite
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32

python3 -m venv .venv
.venv/bin/pip install -r vision/requirements.txt
.venv/bin/python tools/fetch_assets.py
.venv/bin/python tools/prep_trees.py

cmake -S . -B build
cmake --build build -j8
./build/bake_trees "$ROOT"
./tools/build_firmware.sh
echo "Setup finished. Run tools/run_all.sh next."
