#!/bin/sh
# Starts the classifier and the simulation. Start the Wokwi simulator in VS Code first.
#   tools/run_all.sh          uses the Wokwi board on port 4000
#   tools/run_all.sh --mock   uses tools/mock_arduino.py instead
# Any other arguments go to the simulation.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PY="$ROOT/.venv/bin/python"

MOCK=0
if [ "$1" = "--mock" ]; then
  MOCK=1
  shift
fi

cleanup() {
  [ -n "$VISION" ] && kill "$VISION" 2>/dev/null
  [ -n "$BOARD" ] && kill "$BOARD" 2>/dev/null
}
trap cleanup EXIT INT TERM

cmake --build "$ROOT/build" -j8 --target selfdrive_sim >/dev/null || exit 1

if [ "$MOCK" = 1 ]; then
  "$PY" "$ROOT/tools/mock_arduino.py" --duration 3600 >/dev/null &
  BOARD=$!
fi

READY="$(mktemp)"
(cd "$ROOT/vision" && exec "$PY" vision_server.py) >"$READY" 2>/dev/null &
VISION=$!
echo "Loading the classifier..."
until grep -q ready "$READY"; do
  kill -0 "$VISION" 2>/dev/null || { echo "The classifier failed to start."; exit 1; }
  sleep 0.2
done
rm -f "$READY"

"$ROOT/build/selfdrive_sim" --root "$ROOT" "$@"
