# Used to upload binaries to the board

#!/bin/bash
set -e  # exit on error

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PORT=
FQBN="esp32:esp32:esp32"

PROJECT_DIR="$SCRIPT_DIR/../BTStation_ESP32"
BUILD_PATH="$PROJECT_DIR/build/esp32.esp32.esp32"
CONFIG_PATH="$PROJECT_DIR/arduino_cli.yaml"

arduino-cli upload \
    --config-file "$CONFIG_PATH" \
    --input-dir "$BUILD_PATH" \
    --fqbn "$FQBN" \
    --port "$PORT"