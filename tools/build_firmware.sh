#!/bin/sh
# Compiles the board firmware into build/firmware, where wokwi.toml expects it.
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
arduino-cli compile --fqbn esp32:esp32:esp32s2 \
  --build-property "tools.ctags.path=$ROOT/tools/no_ctags" \
  --output-dir "$ROOT/build/firmware" "$ROOT/firmware"
